/*
 * nv_imx477.c - imx477 sensor driver
 *
 * Copyright (c) 2023. FRAMOS.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
//#define DEBUG 1
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/gpio.h>
#include <linux/module.h>
#include <linux/seq_file.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <media/max96793.h>
#include <media/max96792.h>

#include <media/tegra_v4l2_camera.h>
#include <media/tegracam_core.h>

#include "fsm_imx477_mode_tbls.h"
#include "media/framos_sensor_common.h"

#define IMX477_K_FACTOR 1000LL
#define IMX477_M_FACTOR 1000000LL
#define IMX477_G_FACTOR 1000000000LL
#define IMX477_T_FACTOR 1000000000000LL

#define IMX477_MAX_ANALOG_GAIN_TIMES 22261
#define IMX477_ANALOG_GAIN_OFFSET 1024
#define IMX477_MAX_ANALOG_GAIN 978
#define IMX477_DIGITAL_GAIN_COEF 1000
#define IMX477_DIGITAL_GAIN_STEP 39 // 1/256 * DIG_COEF * 10
#define IMX477_DIGITAL_GAIN_MIN  1*IMX477_DIGITAL_GAIN_COEF
#define IMX477_DIGITAL_GAIN_MAX  16*IMX477_DIGITAL_GAIN_COEF - 1

#define IMX477_MAX_BLACK_LEVEL_8BPP            63 //  Max Legal BL is ((2^BPP)-1)/4
#define IMX477_MAX_BLACK_LEVEL_10BPP        255 //  Max Legal BL is ((2^BPP)-1)/4
#define IMX477_MAX_BLACK_LEVEL_12BPP        1023 //  Max Legal BL is ((2^BPP)-1)/4
#define IMX477_DEFAULT_BLACK_LEVEL_8BPP        16
#define IMX477_DEFAULT_BLACK_LEVEL_10BPP    64
#define IMX477_DEFAULT_BLACK_LEVEL_12BPP    256

#define IMX477_IVT_PXCK_DIV_VAL 5
#define IMX477_IMAGE_PIPE_LINES 4
#define IMX477_MIN_INTEGRATION_LINES 1
#define IMX477_MIN_INTEGRATION_LINES_OFFSET 22
#define IMX477_FINE_STORAGE_TIME 0x790LL
#define IMX477_MAX_FRM_LENGTH_LINES 65535
#define IMX477_MAX_COARSE_INTEG_TIME_LINES 65535

#define IMX477_MAX_CSI_LANES 4
#define IMX477_TWO_LANE_MODE 2

#define IMX477_INCK 27000000LL

/**
 * list HEAD of private data of all probed sensors on the platform 
 */
LIST_HEAD(imx477_sensor_list);

/**
 * Declaration
 */
static void imx477_configure_second_slave_address(void);
static struct mutex serdes_lock__;

static const struct of_device_id imx477_of_match[] = {
    { .compatible = "framos,fsm-imx477",},
    { },
};
MODULE_DEVICE_TABLE(of, imx477_of_match);

static const char * const imx477_test_pattern_menu[] = {
    [0] = "No pattern",
    [1] = "Solid color",
    [2] = "100% color bars",
    [3] = "Fade to grey color bars",
    [4] = "PN9",
};

static const u32 ctrl_cid_list[] = {
    TEGRA_CAMERA_CID_GAIN,
    TEGRA_CAMERA_CID_EXPOSURE,
    TEGRA_CAMERA_CID_FRAME_RATE,
    TEGRA_CAMERA_CID_SENSOR_MODE_ID,
    TEGRA_CAMERA_CID_OPERATION_MODE,
    TEGRA_CAMERA_CID_BROADCAST,
    TEGRA_CAMERA_CID_BLACK_LEVEL,
    TEGRA_CAMERA_CID_TEST_PATTERN,
};

struct imx477 {
    struct i2c_client            *i2c_client;
    struct v4l2_subdev           *subdev;
    u64                          frame_length;
    u64                          min_frame_length;
    u64                          current_pixel_format;
    u32                          integration_offset;
    u32                          line_time;
    i2c_broadcast_ctrl          broadcast_ctrl;
    struct mutex                pw_mutex;
    struct list_head            entry; 
    struct camera_common_data   *s_data;
    struct tegracam_device      *tc_dev;
};

static const struct regmap_config sensor_regmap_config = {
    .reg_bits = 16,
    .val_bits = 8,
    .cache_type = REGCACHE_RBTREE,
#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 4, 0)
    .use_single_rw = true,
#else
    .use_single_read = true,
    .use_single_write = true,
#endif
};

static inline int imx477_read_reg(struct camera_common_data *s_data,
                u16 addr, u8 *val)
{
    int err = 0;
    u32 reg_val = 0;

    err = regmap_read(s_data->regmap, addr, &reg_val);
    *val = reg_val & 0xFF;

    return err;
}

static int imx477_write_reg(struct camera_common_data *s_data,
                u16 addr, u8 val)
{
    int err;
    struct device *dev = s_data->dev;

    err = regmap_write(s_data->regmap, addr, val);
    if (err)
        dev_err(dev, "%s: i2c write failed, 0x%x = %x\n",
            __func__, addr, val);

    return err;
}

/**
 * I2C command is braodcasted using the 2nd i2c address
 */
static int imx477_write_reg_broadcast(struct camera_common_data *s_data,
                                        u16 addr, u8 val)
{   
    int err;
    struct device *dev = s_data->dev;
    
    err = regmap_write(s_data->broadcast_regmap, addr, val);
    if (err) {
        dev_err(dev, "%s: i2c write failed, %x = %x\n", 
            __func__, addr, val);
    }
    
    return err;
}

/**
 * Reads multiple sequential registers at the same time using grouphold
 */
static int imx477_read_buffered_reg(struct camera_common_data *s_data,
                                 u16 addr_low, u8 number_of_registers, u64 *val)
{
    struct device *dev = s_data->dev;
    int err, i;
    u8 reg;

    *val = 0;

    if (!s_data->group_hold_active){
        err = imx477_write_reg(s_data, GRP_PARAM_HOLD, 0x01);
        if (err) {
            dev_err(dev, "%s: error setting register hold\n", __func__);
            return err;
        }
    }

    for (i = 0; i < number_of_registers; i++) {
        err = imx477_read_reg(s_data, addr_low - i, &reg);
        *val += reg << (i * 8);
        if (err) {
            dev_err(dev, "%s: error reading buffered registers\n", __func__);
            return err;
        }
    }

    if (!s_data->group_hold_active){
        err = imx477_write_reg(s_data, GRP_PARAM_HOLD, 0x00);
        if (err) {
            dev_err(dev, "%s: error unsetting register hold\n", __func__);
            return err;
        }
    }

    return err;
}

/**
 * Writes multiple sequential registers at the same time using reghold
 */
static int imx477_write_buffered_reg(struct camera_common_data *s_data, 
                                u16 addr_low, u8 number_of_registers, u64 val)
{
    int err, i;
    struct device *dev = s_data->dev;

    if (!s_data->group_hold_active){
        err = imx477_write_reg(s_data, GRP_PARAM_HOLD, 0x01);
        if (err) {
            dev_err(dev, "%s: GRP_PARAM_HOLD error\n", __func__);
            return err;
        }
    }

    for (i = 0; i < number_of_registers; i++) {
        err = imx477_write_reg(s_data, addr_low - i, (u8)(val >> (i * 8)));
        if (err) {
            dev_err(dev, "%s: BUFFERED register write error\n", __func__);
            return err;
        }
    }

    if (!s_data->group_hold_active){
        err = imx477_write_reg(s_data, GRP_PARAM_HOLD, 0x00);
        if (err) {
            dev_err(dev, "%s: GRP_PARAM_HOLD erroror\n", __func__);
            return err;
        }
    }

    return err;
}

/**
 * Broadcast multiple sequential registers at the same time using reghold
 */
static int imx477_broadcast_buffered_reg(struct camera_common_data *s_data, 
                                u16 addr_low, u8 number_of_registers, u32 val)
{
    int err, i;
    struct device *dev = s_data->dev;

    if (!s_data->group_hold_active){
        err = imx477_write_reg_broadcast(s_data, GRP_PARAM_HOLD, 0x01);
        if (err) {
            dev_err(dev, "%s: GRP_PARAM_HOLD error\n", __func__);
            return err;
        }
    }

    for (i = 0; i < number_of_registers; i++) {
        err = imx477_write_reg_broadcast(s_data, addr_low - i, 
                    (u8)(val >> (i * 8)));
        if (err) {
            dev_err(dev, "%s: BUFFERED register write error\n", __func__);
            return err;
        }
    }

    if (!s_data->group_hold_active){
        err = imx477_write_reg_broadcast(s_data, GRP_PARAM_HOLD, 0x00);
        if (err) {
            dev_err(dev, "%s: GRP_PARAM_HOLD erroror\n", __func__);
            return err;
        }
    }

    return err;
}

static int imx477_write_table(struct imx477 *priv,
                const imx477_reg table[])
{
    struct camera_common_data *s_data = priv->s_data;

    return regmap_util_write_table_8(s_data->regmap,
                     table,
                     NULL, 0,
                     IMX477_TABLE_WAIT_MS,
                     IMX477_TABLE_END);
}

static int imx477_set_group_hold(struct tegracam_device *tc_dev, bool val)
{
    struct camera_common_data *s_data = tc_dev->s_data;
    struct device *dev = tc_dev->dev;
    int err;

    s_data->group_hold_active = val;

    err = imx477_write_reg(s_data, GRP_PARAM_HOLD, val);
    if (err) {
        dev_err(dev, "%s: GRP_PARAM_HOLD error\n", __func__);
        return err;
    }

    return err;
}

static int imx477_update_ctrl(struct tegracam_device *tc_dev, int ctrl_id, u64 current_val, u64 default_val, u64 min_val, u64 max_val)
{
    struct imx477 *priv = (struct imx477 *)tegracam_get_privdata(tc_dev);
    struct v4l2_ctrl *ctrl;

    /* Update Black level control*/
    ctrl = fm_find_v4l2_ctrl(tc_dev, ctrl_id);
    if (ctrl) {
        switch (ctrl->id) {
        case TEGRA_CAMERA_CID_BLACK_LEVEL:
            *ctrl->p_new.p_s64 = current_val;
            *ctrl->p_cur.p_s64 = current_val;
            ctrl->minimum = min_val;
            ctrl->maximum = max_val;
            ctrl->default_value = default_val;
            priv->s_data->blklvl_max_range = max_val;
            break;
        case TEGRA_CAMERA_CID_TEST_PATTERN:
            ctrl->qmenu = imx477_test_pattern_menu;
            ctrl->maximum = max_val;
            break;
        }
    }
    return 0;
}

static int imx477_set_black_level(struct tegracam_device *tc_dev, s64 val)
{
    struct camera_common_data *s_data = tc_dev->s_data;
    struct device *dev = tc_dev->dev;
    int err;
    s64 black_level_reg;

    if (s_data->colorfmt->code == MEDIA_BUS_FMT_SRGGB8_1X8)
        black_level_reg = val << 4;
    else if (s_data->colorfmt->code == MEDIA_BUS_FMT_SRGGB10_1X10)
        black_level_reg = val << 2;
    else
        black_level_reg = val;

    err = imx477_write_reg(s_data, MANUAL_DATA_PEDESTAL_EN, 1);
    if (err)
        goto fail;

    err = imx477_write_buffered_reg(s_data, MANUAL_DATA_PEDESTAL_VALUE_LOW, 2, black_level_reg);
    if (err)
        goto fail;

    dev_dbg(dev, "%s: black level: %lld\n",  __func__, val);

    return 0;

fail:
    dev_dbg(dev, "%s: BLACK LEVEL control error\n", __func__);
    return err;
}

/**
 * Setting digital gain is described in the chapter 5.5.2 "Digital gain settings"
 * in the IMX477 software reference manual
 */
static int imx477_get_digital_gain(struct tegracam_device *tc_dev, s64 val)
{
    struct device *dev = tc_dev->dev;
    u32 integer_part, decimal_part, digital_gain;

    if (val > IMX477_DIGITAL_GAIN_MAX)
        val = IMX477_DIGITAL_GAIN_MAX;
 
    integer_part = val / IMX477_DIGITAL_GAIN_COEF;
    decimal_part = ( (val % IMX477_DIGITAL_GAIN_COEF) * 10 )  
                                                / IMX477_DIGITAL_GAIN_STEP;
    decimal_part = (decimal_part > 0xFF) ? 0xFF : decimal_part;

    digital_gain = ( integer_part << 8 ) + decimal_part;

    dev_dbg(dev, "%s: Digital gain val: %lld, Digital gain reg: %xh\n", 
                                                __func__, val, digital_gain);

    return digital_gain;
}

static int imx477_set_gain(struct tegracam_device *tc_dev, s64 val)
{
    struct camera_common_data *s_data = tc_dev->s_data;
    struct imx477 *priv = (struct imx477 *)tc_dev->priv;
    struct device *dev = tc_dev->dev;
    const struct sensor_mode_properties *mode =
        &s_data->sensor_props.sensor_modes[s_data->mode];
    int err = 0, i;
    u32 analog_gain;
    u32 digital_gain = 1 << 8;
    s64 digital_part;

    /* translate value */
    analog_gain = IMX477_ANALOG_GAIN_OFFSET - (IMX477_ANALOG_GAIN_OFFSET 
                                * mode->control_properties.gain_factor) / val;
    analog_gain =  (analog_gain > IMX477_MAX_ANALOG_GAIN)  ? 
                            IMX477_MAX_ANALOG_GAIN : analog_gain;
    
    /* digital gain */    
    if (val > IMX477_MAX_ANALOG_GAIN_TIMES) {
        digital_part = (val * IMX477_DIGITAL_GAIN_COEF + IMX477_MAX_ANALOG_GAIN_TIMES/2)
                            /  IMX477_MAX_ANALOG_GAIN_TIMES;
        digital_gain = imx477_get_digital_gain(tc_dev, digital_part);
    }

    /* write digital and analog gain */
    if (priv->broadcast_ctrl == BROADCAST) {
        if (!s_data->group_hold_active)
            err = imx477_write_reg_broadcast(s_data, GRP_PARAM_HOLD, 0x01);

        for (i = 0; i < 2; i++)
            err |= imx477_write_reg_broadcast(s_data, ANA_GAIN_GLOBAL_LOW - i, 
               (u8)(analog_gain >> (i * 8)));

        for (i = 0; i < 2; i++)
            err |= imx477_write_reg_broadcast(s_data, DIG_GAIN_GR_LOW - i, 
               (u8)(digital_gain >> (i * 8)));

        if (!s_data->group_hold_active)
            err |= imx477_write_reg_broadcast(s_data, GRP_PARAM_HOLD, 0x00);
    }
    else {
        if (!s_data->group_hold_active)
            err = imx477_write_reg(s_data, GRP_PARAM_HOLD, 0x01);

        for (i = 0; i < 2; i++)
            err |= imx477_write_reg(s_data, ANA_GAIN_GLOBAL_LOW - i, 
               (u8)(analog_gain >> (i * 8)));

        for (i = 0; i < 2; i++)
            err |= imx477_write_reg(s_data, DIG_GAIN_GR_LOW - i, 
               (u8)(digital_gain >> (i * 8)));

        if (!s_data->group_hold_active)
            err |= imx477_write_reg(s_data, GRP_PARAM_HOLD, 0x00);
    }
    if (err){
        dev_dbg(dev, "%s: GAIN control error\n", __func__);
        return err;
    }

    dev_dbg(dev, "%s:  gain val [%lld] reg [%d]\n",  __func__, val, analog_gain);

    return 0;
}

static int imx477_set_exposure(struct tegracam_device *tc_dev, s64 val)
{
    struct camera_common_data *s_data = tc_dev->s_data;
    struct imx477 *priv = (struct imx477 *)tc_dev->priv;
    struct device *dev = tc_dev->dev;
    struct v4l2_ctrl *ctrl;
    int err = 0;
    u64 integration_time_line;
    int i;
    u8 cit_lshift = 0;

    dev_dbg(dev, "%s: integration time: %lld [us]\n", __func__, val);

    /* Check value with internal range */
    if (val > s_data->exposure_max_range) {
        val = s_data->exposure_max_range;
    }
    else if (val < s_data->exposure_min_range) {
        val = s_data->exposure_min_range;
    }

    integration_time_line = ((val - priv->integration_offset)
                                     * IMX477_K_FACTOR) / priv->line_time ;

    /* Check for long exposure */
    if (integration_time_line > IMX477_MAX_COARSE_INTEG_TIME_LINES) {
        fm_calc_lshift(&integration_time_line, &cit_lshift, 
                                IMX477_MAX_COARSE_INTEG_TIME_LINES);
        integration_time_line = integration_time_line << cit_lshift; 
    }

    if (integration_time_line < IMX477_MIN_INTEGRATION_LINES)
        integration_time_line = IMX477_MIN_INTEGRATION_LINES;
    else if (integration_time_line > (priv->frame_length - IMX477_MIN_INTEGRATION_LINES_OFFSET))
        integration_time_line = priv->frame_length - IMX477_MIN_INTEGRATION_LINES_OFFSET;

    if (priv->broadcast_ctrl == BROADCAST)             
        err = imx477_broadcast_buffered_reg(s_data, COARSE_INTEG_TIME_LOW, 2,
                                    integration_time_line >> cit_lshift);  
    else {
        /* write like this because two different groups need to be written */
        if (!s_data->group_hold_active)
            err = imx477_write_reg(s_data, GRP_PARAM_HOLD, 0x01);
            

        for (i = 0; i < 2; i++) {
            err |= imx477_write_reg(s_data, COARSE_INTEG_TIME_LOW - i,
             (u8)((integration_time_line >> cit_lshift) >> (i * 8)));
        }

        err |= imx477_write_reg(s_data, CIT_LSHIFT, cit_lshift);

        if (!s_data->group_hold_active)
            err |= imx477_write_reg(s_data, GRP_PARAM_HOLD, 0x00);
    }

    if (err) {
        dev_err(dev, "%s: failed to set exposure\n", __func__);
        return err;
    }

    /* Update new ctrl value */
    ctrl = fm_find_v4l2_ctrl(tc_dev, TEGRA_CAMERA_CID_EXPOSURE);
    if (ctrl) {
        /* Value could be adjusted, set the right value */
        *ctrl->p_new.p_s64 = val;
        /* This ctrl is affected on FRAME RATE control also */
        *ctrl->p_cur.p_s64 = val;
    }

    dev_dbg(dev,
     "%s: set integration time: %lld [us], coarse1:%llu [line], frame length: %llu [line]\n",
     __func__, val, integration_time_line, priv->frame_length);

    return err;
}

static int imx477_set_frame_rate(struct tegracam_device *tc_dev, s64 val)
{
    struct camera_common_data *s_data = tc_dev->s_data;
    struct imx477 *priv = (struct imx477 *)tc_dev->priv;
    struct device *dev = tc_dev->dev;
    int err = 0;
    u64 frame_length;
    u64 exposure_max_range, exposure_min_range;
    const struct sensor_mode_properties *mode =
        &s_data->sensor_props.sensor_modes[s_data->mode];
    int i;
    u8 fll_lshift = 0;

    frame_length = (((u64)mode->control_properties.framerate_factor * 
                              IMX477_G_FACTOR) / (val * priv->line_time));

    if (frame_length < priv->min_frame_length)
        frame_length = priv->min_frame_length;
    
    if (frame_length > IMX477_MAX_FRM_LENGTH_LINES) {
        fm_calc_lshift(&frame_length, &fll_lshift, 
                                IMX477_MAX_FRM_LENGTH_LINES);
        frame_length = frame_length << fll_lshift; 
    }

    priv->frame_length = frame_length;

    /* Update exposure range, before writing the new frame length */
    exposure_min_range = IMX477_MIN_INTEGRATION_LINES 
                                            * priv->line_time / IMX477_K_FACTOR;
    exposure_min_range += priv->integration_offset;   

    exposure_max_range = (priv->frame_length - IMX477_MIN_INTEGRATION_LINES_OFFSET) 
                                            * priv->line_time / IMX477_K_FACTOR;
    exposure_max_range += priv->integration_offset; 

    fm_update_ctrl_range(tc_dev, TEGRA_CAMERA_CID_EXPOSURE,
                             exposure_min_range, exposure_max_range);

    /* write like this because two different groups need to be written */
    if (!s_data->group_hold_active)
        err = imx477_write_reg(s_data, GRP_PARAM_HOLD, 0x01);

    for (i = 0; i < 2; i++)
        err |= imx477_write_reg(s_data, FRM_LENGTH_LINES_LOW - i, 
                   (u8)((priv->frame_length >> fll_lshift) >> (i * 8)));

    err |= imx477_write_reg(s_data, FLL_LSHIFT, fll_lshift);

    if (!s_data->group_hold_active)
        err |= imx477_write_reg(s_data, GRP_PARAM_HOLD, 0x00);

    if (err) {
        dev_err(dev, "%s: failed to set frame length\n", __func__);
        return err;
    }

    dev_dbg(dev,
        "%s: val: %lld, frame_length set: %llu\n",
             __func__, val, priv->frame_length);

    return 0;
}

/**
 * Test pattern is described in the chapter 8.1 "Test pattern output"
 * in the IMX477 software reference manual
 */
static int imx477_set_test_pattern(struct tegracam_device *tc_dev, u32 val)
{
    struct camera_common_data *s_data = tc_dev->s_data;
    struct device *dev = tc_dev->dev; 
    int err;

    err = imx477_write_reg(s_data, TP_MODE, val);
    if (err) 
        goto fail;

    dev_dbg(dev, "%s++ Test mode pattern: %u\n", __func__, val);

    return 0;
fail:
    dev_err(dev, "%s: error setting test pattern\n", __func__);
    return err;
}

/**
 * Update max framerate range
 */
static int imx477_update_framerate_range(struct tegracam_device *tc_dev)
{
    struct camera_common_data *s_data = tc_dev->s_data;
    struct imx477 *priv = (struct imx477 *)tegracam_get_privdata(tc_dev);
    struct sensor_control_properties *ctrlprops = NULL;
    u64 max_framerate;

    ctrlprops = 
        &s_data->sensor_props.sensor_modes[s_data->mode].control_properties;

    switch (s_data->mode) {
    case IMX477_MODE_SCALE_HV3:
        priv->min_frame_length = IMX477_DEFAULT_HEIGHT 
                                + IMX477_MIN_FRAME_LENGTH_DELTA;
        break;
    default:
        priv->min_frame_length = s_data->fmt_height 
                                + IMX477_MIN_FRAME_LENGTH_DELTA;
    }

    max_framerate = (IMX477_G_FACTOR * IMX477_M_FACTOR) /
                                     (priv->min_frame_length * priv->line_time);
    
    fm_update_ctrl_range(tc_dev, TEGRA_CAMERA_CID_FRAME_RATE, 
                            ctrlprops->min_framerate, max_framerate);

    return 0;
}

/**
 * Set operation mode of sensor 
 */
static int imx477_set_operation_mode(struct tegracam_device *tc_dev, u32 val)
{
    struct camera_common_data *s_data = tc_dev->s_data;
    struct device *dev = tc_dev->dev;
    int err = 0;
    u8 xmaster;

    /* If sensor is streaming, return */
    if (tc_dev->is_streaming)
        return 0;

    if (val == SLAVE_MODE)
        xmaster = 0;
    else
        xmaster = 1;

    if (s_data->power->state == SWITCH_ON) {
        err = imx477_write_reg(s_data, MASTER_SLAVE_SEL, xmaster);
        if (err) 
            dev_err(dev, "%s: error setting operation mode\n", __func__);
    }

    return err;
}

static int imx477_set_broadcast_ctrl(struct tegracam_device *tc_dev, 
                                        struct v4l2_ctrl *ctrl)
{
    struct imx477 *priv = (struct imx477 *)tegracam_get_privdata(tc_dev);
    struct device *dev = tc_dev->dev;
    int err;

    dev_dbg(dev, "%s++\n", __func__);

    if (ctrl->flags & V4L2_CTRL_FLAG_INACTIVE) {
        ctrl->val = UNICAST;
        dev_info(dev, "%s: Broadcast control is inactive\n", __func__);
        return 0;
    }

    err = common_get_broadcast_client(tc_dev, ctrl, &sensor_regmap_config);  
    if (err) {
        return err;
    }

    priv->broadcast_ctrl =  *ctrl->p_new.p_u8;
    imx477_configure_second_slave_address();

    return 0;
}

static int imx477_power_on(struct camera_common_data *s_data)
{
    int err = 0;
    struct camera_common_power_rail *pw = s_data->power;
    struct camera_common_pdata *pdata = s_data->pdata;
    struct device *dev = s_data->dev;
    struct imx477 *priv = (struct imx477 *)s_data->priv;

    dev_dbg(dev, "%s: power on\n", __func__);

    mutex_lock(&priv->pw_mutex);

    if (pdata && pdata->power_on) {
        err = pdata->power_on(pw);
        if (err)
            dev_err(dev, "%s failed.\n", __func__);
        else
            pw->state = SWITCH_ON;
        mutex_unlock(&priv->pw_mutex);
        return err;
    }

    if (!pw->mclk) {
        dev_err(dev, "%s: mclk not available\n",  __func__);
        goto imx477_mclk_fail;
    }

    /* Power ON sequence according to IMX477 datasheet */

    if (pw->dvdd) {
        err = regulator_enable(pw->dvdd);
        if (err)
            goto imx477_dvdd_fail;
    }

    if (pw->iovdd) {
        err = regulator_enable(pw->iovdd);
        if (err)
            goto imx477_iovdd_fail;
    }

    if (pw->avdd) {
        err = regulator_enable(pw->avdd);
        if (err)
            goto imx477_avdd_fail;
    }

    err = clk_prepare_enable(pw->mclk);
    if(err) {
        dev_err(dev, "%s: failed to enable mclk\n",  __func__);
        return err;    
    }

    usleep_range(1, 2);

    /* Set sensor reset to 1 */
    if (strcmp(s_data->pdata->gmsl, "gmsl")) {
        if (gpio_is_valid(pw->reset_gpio))
            fm_gpio_set(s_data, pw->reset_gpio, 1);
    }
    else {
        /* For now no separate power on required for serializer device */
        dev_dbg(dev, "%s: max96792_power_on\n", __func__);
        max96792_power_on(priv->s_data->dser_dev, &priv->s_data->g_ctx);
    }

    /* MUST be before sleep */
    pw->state = SWITCH_ON;

    /* Additional sleep required in the case of hardware power-on sequence */
    usleep_range(30000, 31000);

    /* Must be set first after power on according to EXCK_FREQ entry in IMX477 register map */
    imx477_write_reg(s_data, EXCK_FREQ_HIGH, 0x1B);
    imx477_write_reg(s_data, EXCK_FREQ_LOW, 0x00);

    mutex_unlock(&priv->pw_mutex);

    imx477_configure_second_slave_address();

    return 0;

imx477_avdd_fail:
    regulator_disable(pw->iovdd);

imx477_iovdd_fail:
    regulator_disable(pw->dvdd);

imx477_dvdd_fail:
imx477_mclk_fail:
    mutex_unlock(&priv->pw_mutex);
    dev_err(dev, "%s failed.\n", __func__);

    return -ENODEV;
}

static int imx477_power_off(struct camera_common_data *s_data)
{
    struct camera_common_power_rail *pw = s_data->power;
    struct camera_common_pdata *pdata = s_data->pdata;
    struct device *dev = s_data->dev;
    struct imx477 *priv = (struct imx477 *)s_data->priv;
    int err = 0;

    dev_dbg(dev, "%s: power off\n", __func__);

    mutex_lock(&priv->pw_mutex);

    if (pdata && pdata->power_off) {
        err = pdata->power_off(pw);
        if (!err)
            goto power_off_done;
        else
            dev_err(dev, "%s failed.\n", __func__);
        mutex_unlock(&priv->pw_mutex);
        return err;
    }

    /* Power OFF sequence according to IMX477 datasheet */

    clk_disable_unprepare(pw->mclk);

    if (strcmp(s_data->pdata->gmsl, "gmsl")) {
        if (gpio_is_valid(pw->reset_gpio))
            fm_gpio_set(s_data, pw->reset_gpio, 0);
    }
    else {
        dev_dbg(dev, "%s: max96792_power_off\n", __func__);
        max96792_power_off(priv->s_data->dser_dev, &priv->s_data->g_ctx);
    }

    if (pw->avdd)
        regulator_disable(pw->avdd);
    if (pw->iovdd)
        regulator_disable(pw->iovdd);
    if (pw->dvdd)
        regulator_disable(pw->dvdd);

power_off_done:
    pw->state = SWITCH_OFF;
    mutex_unlock(&priv->pw_mutex);

    return 0;
}

/**
 * Acquires regulators, clock and GPIO defined in platform_data
 */
static int imx477_power_get(struct tegracam_device *tc_dev)
{
    struct device *dev = tc_dev->dev;
    struct camera_common_data *s_data = tc_dev->s_data;
    struct camera_common_power_rail *pw = s_data->power;
    struct camera_common_pdata *pdata = s_data->pdata;
    const char *mclk_name;
    const char *parentclk_name;
    struct clk *parent;
    int err = 0;

    if (!pdata) {
        dev_err(dev, "pdata missing\n");
        return -EFAULT;
    }

    mclk_name = pdata->mclk_name ?
            pdata->mclk_name : "extperiph1";
    pw->mclk = devm_clk_get(dev, mclk_name);
    if (IS_ERR(pw->mclk)) {
        dev_err(dev, "unable to get clock %s\n", mclk_name);
        return PTR_ERR(pw->mclk);
    }

    parentclk_name = pdata->parentclk_name;
    if (parentclk_name) {
        parent = devm_clk_get(dev, parentclk_name);
        if (IS_ERR(parent)) {
            dev_err(dev, "unable to get parent clcok %s",
                parentclk_name);
        } else
            clk_set_parent(pw->mclk, parent);
    }

    /* Analog 2.8v */
    if (pdata->regulators.avdd)
        err |= camera_common_regulator_get(dev,
                &pw->avdd, pdata->regulators.avdd);
    /* Dig 1.2v */
    if (pdata->regulators.dvdd)
        err |= camera_common_regulator_get(dev,
                &pw->dvdd, pdata->regulators.dvdd);
    /* IO 1.8v */
    if (pdata->regulators.iovdd)
        err |= camera_common_regulator_get(dev,
                &pw->iovdd, pdata->regulators.iovdd);
    if (err) {
        dev_info(dev, "%s: unable to get regulator(s)\n", __func__);
        goto done;
    }

    if (strcmp(s_data->pdata->gmsl, "gmsl")) {
        pw->reset_gpio = pdata->reset_gpio;

        if (pdata->use_cam_gpio) {
            err = cam_gpio_register(dev, pw->reset_gpio);
            if (err) {
                dev_err(dev, "%s ERR can't register cam gpio %u!\n",
                 __func__, pw->reset_gpio);
            }
        }
    }

done:
    pw->state = SWITCH_OFF;
    return err;
}

/**
 * Frees regulators acquired in power_get
 */
static int imx477_power_put(struct tegracam_device *tc_dev)
{
    struct camera_common_data *s_data = tc_dev->s_data;
    struct camera_common_power_rail *pw = s_data->power;
    struct camera_common_pdata *pdata = s_data->pdata;
    struct device *dev = tc_dev->dev;

    if (unlikely(!pw))
        return -EFAULT;

    if (likely(pw->avdd)) {
        devm_regulator_put(pw->avdd);
    }

    if (likely(pw->iovdd)) {
        devm_regulator_put(pw->iovdd);
    }

    if (likely(pw->dvdd)) {
        devm_regulator_put(pw->dvdd);
    }

    pw->avdd = NULL;
    pw->iovdd = NULL;
    pw->dvdd = NULL;

    if (strcmp(s_data->pdata->gmsl, "gmsl")) {
        if (pdata && pdata->use_cam_gpio)
            cam_gpio_deregister(dev, pw->reset_gpio);
        else {
            if (gpio_is_valid(pw->reset_gpio))  
                gpio_free(pw->reset_gpio);
        }
    }

    return 0;
}

/**
 * Read frame length register to confirm communication
 */
static int imx477_communication_verify(struct tegracam_device *tc_dev)
{
    struct camera_common_data *s_data = tc_dev->s_data;
    struct imx477 *priv = (struct imx477 *)tegracam_get_privdata(tc_dev);
    struct device *dev = tc_dev->dev;
    int err;
    u64 vmax;

    err = imx477_read_buffered_reg(s_data, FRM_LENGTH_LINES_LOW, 2, &vmax);
    if (err) {
        dev_err(dev, "%s: failed to read VMAX\n", __func__);
        return err;
    }
    /* Initialize frame length */
    priv->frame_length = vmax;

    return err;
}

static struct camera_common_pdata *imx477_parse_dt(struct tegracam_device *tc_dev)
{
    struct device *dev = tc_dev->dev;
    struct device_node *np = dev->of_node;
    struct camera_common_pdata *board_priv_pdata;
    const struct of_device_id *match;
    struct camera_common_pdata *ret = NULL;
    const char *gmsl;
    int err;
    int gpio;

    if (!np)
        return NULL;

    match = of_match_device(imx477_of_match, dev);
    if (!match) {
        dev_err(dev, "Failed to find matching dt id\n");
        return NULL;
    }

    board_priv_pdata = devm_kzalloc(dev,
                    sizeof(*board_priv_pdata), GFP_KERNEL);
    if (!board_priv_pdata)
        return NULL;

    err = camera_common_parse_clocks(dev,
                     board_priv_pdata);
    if (err) {
        dev_err(dev, "Failed to find clocks\n");
        goto error;
    }

    err = of_property_read_string(np, "gmsl", &gmsl);
    if (err) {
        dev_warn(dev, "initializing mipi...\n");
        board_priv_pdata->gmsl = "mipi";
        dev_dbg(dev, "no gmsl property found...\n");
    } else if (!strcmp(gmsl, "gmsl")) {
        dev_warn(dev, "initializing GMSL...\n");
        board_priv_pdata->gmsl = "gmsl";
        dev_dbg(dev, "gmsl property found...\n");
    }

    if (strcmp(gmsl, "gmsl")) {
        gpio = of_get_named_gpio(np, "reset-gpios", 0);
        if (gpio < 0) {
            if (gpio == -EPROBE_DEFER)
                ret = ERR_PTR(-EPROBE_DEFER);
            dev_err(dev, "reset-gpios not found %d\n", err);
            goto error;
        }
        board_priv_pdata->reset_gpio = (unsigned int)gpio;
    }

    gpio = of_get_named_gpio(np, "vsync-gpios", 0);
    if (gpio > 0) {
        gpio_direction_input(gpio);
    }

    fm_get_gpio_ctrl(board_priv_pdata);

    err = of_property_read_string(np, "avdd-reg",
        &board_priv_pdata->regulators.avdd);
    err |= of_property_read_string(np, "iovdd-reg",
        &board_priv_pdata->regulators.iovdd);
    err |= of_property_read_string(np, "dvdd-reg",
        &board_priv_pdata->regulators.dvdd);
    if (err)
        dev_dbg(dev, "avdd, iovdd and/or dvdd reglrs. not present, "
            "assume sensor powered independently\n");

    return board_priv_pdata;

error:
    devm_kfree(dev, board_priv_pdata);
    return ret;
}

static int imx477_set_pixel_format(struct tegracam_device *tc_dev)
{
    struct device *dev = tc_dev->dev;
    struct imx477 *priv = (struct imx477 *)tegracam_get_privdata(tc_dev);
    struct camera_common_data *s_data = tc_dev->s_data;
    int err;

    switch (s_data->colorfmt->code) {
    case MEDIA_BUS_FMT_SRGGB8_1X8:
        err = imx477_write_table(priv, mode_table[IMX477_8BIT_MODE]);
        break;
    case MEDIA_BUS_FMT_SRGGB10_1X10:
        err = imx477_write_table(priv, mode_table[IMX477_10BIT_MODE]);
        break;
    case MEDIA_BUS_FMT_SRGGB12_1X12:
        err = imx477_write_table(priv, mode_table[IMX477_12BIT_MODE]);
        break;
    default:
        dev_err(dev, "%s: unknown pixel format\n", __func__);
        return -EINVAL;
    }

    return err;
}

/**
 * Configure CSI lane mode registers
 */
static int imx477_set_csi_lane_mode(struct tegracam_device *tc_dev)
{
    struct camera_common_data *s_data = tc_dev->s_data;
    struct device *dev = tc_dev->dev;
    int err;

    if (s_data->numlanes == IMX477_TWO_LANE_MODE) {
        err = imx477_write_reg(s_data, CSI_LANE_MODE, 1);
        if (err) {
            dev_err(dev, "%s: error setting two lane mode\n", __func__);
            return err;
        }
    }

    dev_dbg(dev, "%s: sensor is in %d CSI lane mode\n",
                  __func__, s_data->numlanes);

    return 0;
}

/**
 * Calculate 1H time
 * Assuming single_PLL and no power_save_mode
 */
static int imx477_calculate_line_time(struct tegracam_device *tc_dev)
{
    struct imx477 *priv = (struct imx477 *)tc_dev->priv;
    struct camera_common_data *s_data = tc_dev->s_data;
    struct device *dev = tc_dev->dev;
    u64 line_length;
    u32 ivtck;
    u32 ivtpxck;
    u8 ivt_prepllck_div;
    u64 ivt_pll_mpy;
    u8 ivt_syck_div;
    int err;

    dev_dbg(dev, "%s:++\n", __func__);

    //line_length
    err = imx477_read_buffered_reg(s_data, LINE_LENGTH_PCK_LOW, 2, &line_length);
    if (err)
        goto fail; 

    //ivt_syck_div
    err = imx477_read_reg(s_data, IVT_SYCK_DIV, &ivt_syck_div);
    if (err)
        goto fail;

    //ivt_prepllck_div
    err = imx477_read_reg(s_data, IVT_PREPLLCK_DIV, &ivt_prepllck_div);
    if (err)
        goto fail;
    
    //ivt_pll_mpy
    err = imx477_read_buffered_reg(s_data, IVT_PLL_MPY_LOW, 2, &ivt_pll_mpy);
    if (err)
        goto fail;

    ivtck = IMX477_INCK * ivt_pll_mpy / ivt_prepllck_div;
    ivtpxck = ivtck / (ivt_syck_div * IMX477_IVT_PXCK_DIV_VAL) / IMX477_M_FACTOR;
    priv->line_time = ((line_length / IMX477_IMAGE_PIPE_LINES) 
                                            * IMX477_K_FACTOR) / ivtpxck;   

    priv->integration_offset = ( IMX477_FINE_STORAGE_TIME * priv->line_time ) / 
                                            ( line_length * IMX477_K_FACTOR );

    dev_dbg(dev, 
"%s: Line length: %llu [pix], ivt_syck_div: %u, ivt_prepllck_div: %u, ivt_pll_mpy: %llu, Line time: %u[ns]\n",
 __func__, line_length, (u32)ivt_syck_div, (u32)ivt_prepllck_div, 
                                                ivt_pll_mpy, priv->line_time);

    return 0;

fail:
    dev_err(dev, "%s: Fail to calculate line time\n", __func__);
    return err;
}

/**
 * Adjust HMAX register, and other properties for selected data rate  
 */
static int imx477_adjust_hmax_register(struct tegracam_device *tc_dev)
{
    struct camera_common_data *s_data = tc_dev->s_data;
    struct device *dev = tc_dev->dev;
    int err;
    u64 line_length;

    dev_dbg(dev, "%s:++\n", __func__);

    if(s_data->numlanes == 2){
        imx477_read_buffered_reg(s_data, LINE_LENGTH_PCK_LOW, 2, &line_length);
            
        line_length *=2;

        err = imx477_write_buffered_reg(s_data, LINE_LENGTH_PCK_LOW, 
                                                            2, line_length);
        if (err) {
            dev_err(dev, "%s: error setting line length\n", __func__);
            return err;
        }
    
        dev_dbg(dev, "%s: Line length: %llu\n",  __func__, line_length);
    }

    return 0;
}

/**
 * Finds first sensor with broadcast enabled
 */
static bool imx477_find_broadcast_sensor(struct imx477 *broadcast_private)
{
    struct imx477 *current_private;

    list_for_each_entry(current_private, &imx477_sensor_list, entry) {
        mutex_lock(&current_private->pw_mutex);
        if (current_private->broadcast_ctrl == BROADCAST
            && current_private->s_data->power->state == SWITCH_ON) {         
            mutex_unlock(&current_private->pw_mutex);
            memcpy(broadcast_private, current_private, sizeof(*broadcast_private));
            return true;
        }
        mutex_unlock(&current_private->pw_mutex);
    }
    return false;
}

/**
 * Enables second slave address on all sensors
 * Enables acknowledge on one sensor
 */
static void imx477_enable_second_slave_address(struct imx477 *ack_private)
{
    struct imx477 *current_private;
    int err;

    list_for_each_entry(current_private, &imx477_sensor_list, entry) {
        mutex_lock(&current_private->pw_mutex);
        /* Continue if sensor is in Power-off state */         
        if (current_private->s_data->power->state != SWITCH_ON) {
            mutex_unlock(&current_private->pw_mutex);
            continue;
        }

        err = imx477_write_reg(current_private->s_data,
                                            SLAVE_ADD_EN_2ND, 1);
        err |= imx477_write_reg(current_private->s_data,
                                            SLAVE_ADD_ACKEN_2ND, 0);
        if (err)
            dev_warn(&current_private->i2c_client->dev, 
                "%s: Fail to write Second I2C register\n", __func__);

        mutex_unlock(&current_private->pw_mutex);

        dev_dbg(&current_private->i2c_client->dev, 
                "%s: Sensors 2nd slave address configured\n", __func__);
    }

    err = imx477_write_reg(ack_private->s_data, 
                                SLAVE_ADD_ACKEN_2ND, 1);
    if (err)
        dev_warn(&ack_private->i2c_client->dev, 
            "%s: Fail to write Second I2C register\n", __func__);

    dev_dbg(&ack_private->i2c_client->dev, 
            ": Sensors 2nd slave address configured with acknowlege\n"); 
}

/**
 * Disables second slave address in all sensors
 */
static void imx477_disable_second_slave_address(void)
{
    struct imx477 *current_private;
    int err;

    list_for_each_entry(current_private, &imx477_sensor_list, entry) {
        mutex_lock(&current_private->pw_mutex);
        /* Continue if sensor is in Power-off state */         
        if (current_private->s_data->power->state != SWITCH_ON) {
            mutex_unlock(&current_private->pw_mutex);
            continue;
        }

        err = imx477_write_reg(current_private->s_data, SLAVE_ADD_EN_2ND, 0);
        if (err)
            dev_warn(&current_private->i2c_client->dev, 
                "%s: Fail to write Second I2C register\n", __func__);

        mutex_unlock(&current_private->pw_mutex);

        dev_dbg(&current_private->i2c_client->dev, 
                "%s: Sensors 2nd slave address disabled\n", __func__);
    }
}

/**
 * If broadcast active, configrue 2nd slave address
 * Only one sensor can have valid 2nd slave address ACK enabled  
 */
static void imx477_configure_second_slave_address(void)
{
    struct imx477 broadcast_private = {};

    if (imx477_find_broadcast_sensor(&broadcast_private)) {
        imx477_enable_second_slave_address(&broadcast_private);
    } else {
        imx477_disable_second_slave_address();
    }
}

/**
 * XVS is triggering pin
 */
static int imx477_configure_triggering_pins(struct tegracam_device *tc_dev)
{
    struct camera_common_data *s_data = tc_dev->s_data;
    struct device *dev = tc_dev->dev;
    int err = 0;

    switch (fm_get_v4l2_ctrl_value(tc_dev, TEGRA_CAMERA_CID_OPERATION_MODE, CURRENT)) {
    case MASTER_MODE:
        /* XVS pin - output */
         err = imx477_write_reg(s_data, XVS_IO_CTRL, 1);
        /* output enable */
         err |= imx477_write_reg(s_data, EXTOUT_EN, 1);

        break;

    case SLAVE_MODE:
         /* XVS pin - input */
        err = imx477_write_reg(s_data, XVS_IO_CTRL, 0);
        dev_dbg(dev, "%s: Sensor is in Slave mode\n", __func__);

        break;

    default:
        dev_err(dev, "%s: unknown operating mode.\n", __func__);
        return -EINVAL;
    }

    if (err) {
        dev_err(dev, "%s: Fail to configure triggering pin\n", __func__);
        return err;
    }

    return 0;
}

/**
 * According to the V4L2 documentation, driver should not return error when 
 * invalid settings are detected.
 * It should apply the settings closest to the ones that the user has requested.
 */
int imx477_check_unsupported_mode(struct camera_common_data *s_data,
                                     struct v4l2_mbus_framefmt *mf)
{
    struct device *dev = s_data->dev;
    bool unsupported_mode = false;

    dev_dbg(dev, "%s++\n", __func__);

    /**
     * Binning is supported only in 10 bit modes according to the
     * Table 5 "Typical image output of main capture mode" in IMX477 datasheet
     */
    if (mf->code == MEDIA_BUS_FMT_SRGGB12_1X12) {
        switch (s_data->mode_prop_idx) {
        case IMX477_MODE_CROP_BINNING_H2V2:
            unsupported_mode = true;
            dev_warn(dev, 
            "%s: selected mode is not supported with RAW12, switching to default\n",
                  __func__);
            break;
        default:
            unsupported_mode = false;
        }
    }

    if (unsupported_mode){
        mf->width    = s_data->frmfmt[s_data->def_mode].size.width;
        mf->height    = s_data->frmfmt[s_data->def_mode].size.height;
    }

    return 0;
}

static int imx477_set_ivtck(struct tegracam_device *tc_dev)
{
    struct camera_common_data *s_data = tc_dev->s_data;
    struct device *dev = tc_dev->dev;
    int err;
    u64 ivt_pll_mpy = 311; 

    err = imx477_write_buffered_reg(s_data, IVT_PLL_MPY_LOW, 2, ivt_pll_mpy);
    if (err){
        dev_err(dev, "%s: unable to set IVT_PLL_MPY\n", __func__);
        return err;
    }

    return 0;
}

int imx477_after_set_pixel_format(struct camera_common_data *s_data)
{
    struct device *dev = s_data->dev;
    struct tegracam_device *tc_dev = to_tegracam_device(s_data);
    struct imx477 *priv = (struct imx477 *)tc_dev->priv;
    struct v4l2_ctrl *ctrl;
    int err;

    dev_dbg(dev, "%s++\n", __func__);

    /* Update Black level V4L control*/
    if(priv->current_pixel_format != s_data->colorfmt->code) {
        ctrl = fm_find_v4l2_ctrl(tc_dev, TEGRA_CAMERA_CID_BLACK_LEVEL);
        switch (s_data->colorfmt->code) {
            case MEDIA_BUS_FMT_SRGGB8_1X8:
                if (priv->current_pixel_format == MEDIA_BUS_FMT_SRGGB12_1X12)
                    err = imx477_update_ctrl(tc_dev, TEGRA_CAMERA_CID_BLACK_LEVEL, (*ctrl->p_cur.p_s64 >> 4),
                                        IMX477_DEFAULT_BLACK_LEVEL_8BPP, 0, IMX477_MAX_BLACK_LEVEL_8BPP);
                else // MEDIA_BUS_FMT_SRGGB10_1X10
                    err = imx477_update_ctrl(tc_dev, TEGRA_CAMERA_CID_BLACK_LEVEL, (*ctrl->p_cur.p_s64 >> 2),
                                        IMX477_DEFAULT_BLACK_LEVEL_8BPP, 0, IMX477_MAX_BLACK_LEVEL_8BPP);
                priv->current_pixel_format = MEDIA_BUS_FMT_SRGGB8_1X8;
                break;
            case MEDIA_BUS_FMT_SRGGB10_1X10:
                if (priv->current_pixel_format == MEDIA_BUS_FMT_SRGGB12_1X12)
                    err = imx477_update_ctrl(tc_dev, TEGRA_CAMERA_CID_BLACK_LEVEL, (*ctrl->p_cur.p_s64 >> 2),
                                         IMX477_DEFAULT_BLACK_LEVEL_10BPP, 0, IMX477_MAX_BLACK_LEVEL_10BPP);
                else // MEDIA_BUS_FMT_SRGGB8_1X8
                    err = imx477_update_ctrl(tc_dev, TEGRA_CAMERA_CID_BLACK_LEVEL, (*ctrl->p_cur.p_s64 << 2),
                                         IMX477_DEFAULT_BLACK_LEVEL_10BPP, 0, IMX477_MAX_BLACK_LEVEL_10BPP);
                priv->current_pixel_format = MEDIA_BUS_FMT_SRGGB10_1X10;
                break;
            case MEDIA_BUS_FMT_SRGGB12_1X12:
                if (priv->current_pixel_format == MEDIA_BUS_FMT_SRGGB10_1X10)
                    err = imx477_update_ctrl(tc_dev, TEGRA_CAMERA_CID_BLACK_LEVEL, (*ctrl->p_cur.p_s64 << 2),
                                         IMX477_DEFAULT_BLACK_LEVEL_12BPP, 0, IMX477_MAX_BLACK_LEVEL_12BPP);
                else // MEDIA_BUS_FMT_SRGGB8_1X8
                    err = imx477_update_ctrl(tc_dev, TEGRA_CAMERA_CID_BLACK_LEVEL, (*ctrl->p_cur.p_s64 << 4),
                                         IMX477_DEFAULT_BLACK_LEVEL_12BPP, 0, IMX477_MAX_BLACK_LEVEL_12BPP);
                priv->current_pixel_format = MEDIA_BUS_FMT_SRGGB12_1X12;
                break;
            default:
                dev_err(dev, "%s: unknown pixel format\n", __func__);
                return -EINVAL;
        }
        if (err)
            return err;
    }

    return 0;
}

static int imx477_set_mode(struct tegracam_device *tc_dev)
{
    struct imx477 *priv = (struct imx477 *)tegracam_get_privdata(tc_dev);
    struct camera_common_data *s_data = tc_dev->s_data;
    struct device *dev = tc_dev->dev;
    int err;

    err = imx477_write_table(priv, mode_table[IMX477_GLOBAL_SETTINGS]);
    if (err) {
        dev_err(dev, "%s: unable to set global settings\n", __func__);
        return err;
    }

    err = imx477_write_table(priv, mode_table[IMX477_IMAGEQUALITY_SETTINGS]);
    if (err) {
        dev_err(dev, "%s: unable to set image quality settings\n", __func__);
        return err;
    }

    err = imx477_write_table(priv, mode_table[IMX477_INIT_SETTINGS]);
    if (err) {
        dev_err(dev, "%s: unable to initialize sensor settings\n", __func__);
        return err;
    }

    err = imx477_set_ivtck(tc_dev);
    if (err) {
        dev_err(dev, "%s: unable to set clock settings\n", __func__);
        return err;
    }

    err = imx477_set_csi_lane_mode(tc_dev);
    if (err) {
        dev_err(dev, "%s: error setting CSI lane mode\n", __func__);
        return err;
    }

    err = imx477_set_pixel_format(tc_dev);
    if (err) {
        dev_err(dev, "%s: unable to write format to image sensor\n", __func__);
        return err;
    }

    err = imx477_write_table(priv, mode_table[s_data->mode]);
    if (err)
        return err;

    err = imx477_set_operation_mode(tc_dev, fm_get_v4l2_ctrl_value(tc_dev, TEGRA_CAMERA_CID_OPERATION_MODE, CURRENT));
    if (err) {
        dev_err(dev, "%s: unable to set operation mode\n", __func__);
        return err;
    }
    
    err = imx477_configure_triggering_pins(tc_dev);
    if (err) {
        dev_err(dev, "%s: unable configure XVS/XHS pins\n", __func__);
        return err;
    }

    err = imx477_set_test_pattern(tc_dev, fm_get_v4l2_ctrl_value(tc_dev, TEGRA_CAMERA_CID_TEST_PATTERN, CURRENT));
    if (err) {
        dev_err(dev, "%s: unable to set Test pattern\n", __func__);
        return err;
    }

    err = imx477_adjust_hmax_register(tc_dev);
    if (err) {
        dev_err(dev, "%s: unable to adjust hmax\n", __func__);
        return err;
    }

    /* Override V4L GAIN, EXPOSURE and FRAME RATE controls */
    s_data->override_enable = true;

    err = imx477_calculate_line_time(tc_dev);
    if (err)
        return err;

    err = imx477_update_framerate_range(tc_dev);
    if (err)
        return err;

    dev_dbg(dev, "%s: set mode %u\n", __func__, s_data->mode);

    return 0;
}

static int imx477_start_streaming(struct tegracam_device *tc_dev)
{
    struct imx477 *priv = (struct imx477 *)tegracam_get_privdata(tc_dev);
    struct device *dev = tc_dev->dev;
    struct camera_common_data *s_data = tc_dev->s_data;
    int err;

    if (!strcmp(s_data->pdata->gmsl, "gmsl")) {
        err = max96793_setup_streaming(priv->s_data->ser_dev, s_data);
        if (err)
            goto exit;
        err = max96792_setup_streaming(priv->s_data->dser_dev, dev, s_data);
        if (err) 
            goto exit;
        err = max96792_start_streaming(priv->s_data->dser_dev, dev);
        if (err)
            goto exit;
    }

    err = imx477_write_table(priv, mode_table[IMX477_MODE_START_STREAM]);
    if (err)
        goto exit;

    return 0;
exit:
    dev_err(dev, "%s: error setting stream\n", __func__);
    return err;
}

static int imx477_stop_streaming(struct tegracam_device *tc_dev)
{
    struct imx477 *priv = (struct imx477 *)tegracam_get_privdata(tc_dev);
    struct camera_common_data *s_data = tc_dev->s_data;
    struct device *dev = tc_dev->dev;
    int err;

    if (!strcmp(s_data->pdata->gmsl, "gmsl")) {
        /* disable serdes streaming */
        max96792_stop_streaming(priv->s_data->dser_dev, dev);
    }

    err = imx477_write_table(priv, mode_table[IMX477_MODE_STOP_STREAM]);
    if (err)
        return err;

    /*
     * Wait for one frame to make sure sensor is set to
     * software standby in V-blank
     *
     * delay = frame length rows * Tline 
     */
    usleep_range(priv->frame_length * priv->line_time / IMX477_K_FACTOR, 
                priv->frame_length * priv->line_time / IMX477_K_FACTOR + 1000);

    return 0;
}


static struct camera_common_sensor_ops imx477_common_ops = {
    .numfrmfmts = ARRAY_SIZE(imx477_frmfmt),
    .frmfmt_table = imx477_frmfmt,
    .power_on = imx477_power_on,
    .power_off = imx477_power_off,
    .write_reg = imx477_write_reg,
    .read_reg = imx477_read_reg,
    .parse_dt = imx477_parse_dt,
    .power_get = imx477_power_get,
    .power_put = imx477_power_put,
    .set_mode = imx477_set_mode,
    .start_streaming = imx477_start_streaming,
    .stop_streaming = imx477_stop_streaming,
    .check_unsupported_mode = imx477_check_unsupported_mode,
    .after_set_pixel_format = imx477_after_set_pixel_format,
};

static int imx477_gmsl_serdes_setup(struct imx477 *priv)
{
    int err = 0;
    int des_err = 0;
    struct device *dev;

    if (!priv || !priv->s_data->ser_dev || !priv->s_data->dser_dev || !priv->i2c_client)
        return -EINVAL;

    dev = &priv->i2c_client->dev;

    dev_dbg(dev, "%s: imx477_gmsl_serdes_setup\n", __func__);

    mutex_lock(&serdes_lock__);

    max96792_reset_control(priv->s_data->dser_dev, &priv->i2c_client->dev);

    if(!strcmp(priv->s_data->pdata->gmsl, "gmsl")) {
        err = max96792_gmsl_setup(priv->s_data->dser_dev); 
        if (err) {
            dev_err(dev, "deserializer gmsl setup failed\n");//
            goto error;
        }

        err = max96793_gmsl_setup(priv->s_data->ser_dev);
        if (err) {
            dev_err(dev, "serializer gmsl setup failed\n");
            goto error;
        }
    }

    dev_dbg(dev, "%s: max96792_setup_link\n", __func__);
    /* setup serdes addressing and control pipeline */
    err = max96792_setup_link(priv->s_data->dser_dev, &priv->i2c_client->dev);
    if (err) {
        dev_err(dev, "gmsl deserializer link config failed\n");
        goto error;
    }

    dev_dbg(dev, "%s: max96793_setup_control\n", __func__);
    err = max96793_setup_control(priv->s_data->ser_dev);

    /* proceed even if ser setup failed, to setup deser correctly */
    if (err)
        dev_err(dev, "gmsl serializer setup failed\n");

    err = max96793_gpio10_xtrig1_setup(priv->s_data->ser_dev, "mipi");
    if (err){
        dev_err(dev, "gmsl serializer gpio10/xtrig1 pin config failed\n");
        goto error;
    }

    dev_dbg(dev, "%s: max96792_setup_control\n", __func__);
    des_err = max96792_setup_control(priv->s_data->dser_dev, &priv->i2c_client->dev);
    if (des_err) {
        dev_err(dev, "gmsl deserializer setup failed\n");
        /* overwrite err only if deser setup also failed */
        err = des_err;
    }

error:
    mutex_unlock(&serdes_lock__);
    return err;
}

static void imx477_gmsl_serdes_reset(struct imx477 *priv)
{
    mutex_lock(&serdes_lock__);

    /* reset serdes addressing and control pipeline */
    max96793_reset_control(priv->s_data->ser_dev);
    max96792_reset_control(priv->s_data->dser_dev, &priv->i2c_client->dev);

    max96792_power_off(priv->s_data->dser_dev, &priv->s_data->g_ctx);

    mutex_unlock(&serdes_lock__);
}

static int imx477_board_setup(struct imx477 *priv)
{
    struct camera_common_data *s_data = priv->s_data;
    struct device *dev = s_data->dev;
    struct device_node *node = dev->of_node;
    struct device_node *ser_node;
    struct i2c_client *ser_i2c = NULL;
    struct device_node *dser_node;
    struct i2c_client *dser_i2c = NULL;
    struct device_node *gmsl;
    int value = 0xFFFF;
    const char *str_value;
    const char *str_value1[2];
    int  i;
    int err = 0;

    dev_dbg(dev, "%s++\n", __func__);
    if (!strcmp(s_data->pdata->gmsl, "gmsl")) {
       err = of_property_read_u32(node, "reg", &priv->s_data->g_ctx.sdev_reg);
        if (err < 0) {
            dev_err(dev, "reg not found\n");
            return err;
        }

        err = of_property_read_u32(node, "def-addr",
                        &priv->s_data->g_ctx.sdev_def);
        if (err < 0) {
            dev_err(dev, "def-addr not found\n");
            return err;
        }

        ser_node = of_parse_phandle(node, "nvidia,gmsl-ser-device", 0);
        if (ser_node == NULL) {
            dev_err(dev,
                "missing %s handle\n",
                    "nvidia,gmsl-ser-device");
            return err;
        }

        err = of_property_read_u32(ser_node, "reg", &priv->s_data->g_ctx.ser_reg);
        if (err < 0) {
            dev_err(dev, "serializer reg not found\n");
            return err;
        }

        ser_i2c = of_find_i2c_device_by_node(ser_node);
        of_node_put(ser_node);

        if (ser_i2c == NULL) {
            dev_err(dev, "missing serializer dev handle\n");
            return err;
        }
        if (ser_i2c->dev.driver == NULL) {
            dev_err(dev, "missing serializer driver\n");
            return err;
        }

        priv->s_data->ser_dev = &ser_i2c->dev;

        dser_node = of_parse_phandle(node, "nvidia,gmsl-dser-device", 0);
        if (dser_node == NULL) {
            dev_err(dev,
                "missing %s handle\n",
                    "nvidia,gmsl-dser-device");
            return err;
        }

        dser_i2c = of_find_i2c_device_by_node(dser_node);
        of_node_put(dser_node);

        if (dser_i2c == NULL) {
            dev_err(dev, "missing deserializer dev handle\n");
            return err;
        }
        if (dser_i2c->dev.driver == NULL) {
            dev_err(dev, "missing deserializer driver\n");
            return err;
        }

        priv->s_data->dser_dev = &dser_i2c->dev;

        /* populate g_ctx from DT */
        gmsl = of_get_child_by_name(node, "gmsl-link");
        if (gmsl == NULL) {
            dev_err(dev, "missing gmsl-link device node\n");
            err = -EINVAL;
            return err;
        }

        err = of_property_read_string(gmsl, "dst-csi-port", &str_value);
        if (err < 0) {
            dev_err(dev, "No dst-csi-port found\n");
            return err;
        }
        priv->s_data->g_ctx.dst_csi_port =
            (!strcmp(str_value, "a")) ? GMSL_CSI_PORT_A : GMSL_CSI_PORT_B;

        err = of_property_read_string(gmsl, "src-csi-port", &str_value);
        if (err < 0) {
            dev_err(dev, "No src-csi-port found\n");
            return err;
        }
        priv->s_data->g_ctx.src_csi_port =
            (!strcmp(str_value, "a")) ? GMSL_CSI_PORT_A : GMSL_CSI_PORT_B;

        err = of_property_read_string(gmsl, "csi-mode", &str_value);
        if (err < 0) {
            dev_err(dev, "No csi-mode found\n");
            return err;
        }

        if (!strcmp(str_value, "1x4")) {
            priv->s_data->g_ctx.csi_mode = GMSL_CSI_1X4_MODE;
        } else if (!strcmp(str_value, "2x4")) {
            priv->s_data->g_ctx.csi_mode = GMSL_CSI_2X4_MODE;
        } else if (!strcmp(str_value, "2x2")) {
            priv->s_data->g_ctx.csi_mode = GMSL_CSI_2X2_MODE;
        } else {
            dev_err(dev, "invalid csi mode\n");
            return err;
        }

        err = of_property_read_string(gmsl, "serdes-csi-link", &str_value);
        if (err < 0) {
            dev_err(dev, "No serdes-csi-link found\n");
            return err;
        }
        priv->s_data->g_ctx.serdes_csi_link =
            (!strcmp(str_value, "a")) ?
                GMSL_SERDES_CSI_LINK_A : GMSL_SERDES_CSI_LINK_B;

        err = of_property_read_u32(gmsl, "st-vc", &value);
        if (err < 0) {
            dev_err(dev, "No st-vc info\n");
            return err;
        }
        priv->s_data->g_ctx.st_vc = value;

        err = of_property_read_u32(gmsl, "vc-id", &value);
        if (err < 0) {
            dev_err(dev, "No vc-id info\n");
            return err;
        }
        priv->s_data->g_ctx.dst_vc = value;

        err = of_property_read_u32(gmsl, "num-lanes", &value);
        if (err < 0) {
            dev_err(dev, "No num-lanes info\n");
            return err;
        }
        priv->s_data->g_ctx.num_csi_lanes = value;

        priv->s_data->g_ctx.num_streams =
                of_property_count_strings(gmsl, "streams");
        if (priv->s_data->g_ctx.num_streams <= 0) {
            dev_err(dev, "No streams found\n");
            err = -EINVAL;
            return err;
        }

        for (i = 0; i < priv->s_data->g_ctx.num_streams; i++) {
            of_property_read_string_index(gmsl, "streams", i,
                            &str_value1[i]);
            if (!str_value1[i]) {
                dev_err(dev, "invalid stream info\n");
                return err;
            }
            if (!strcmp(str_value1[i], "raw12")) {
                priv->s_data->g_ctx.streams[i].st_data_type =
                                GMSL_CSI_DT_RAW_12;
            } else if (!strcmp(str_value1[i], "embed")) {
                priv->s_data->g_ctx.streams[i].st_data_type =
                                GMSL_CSI_DT_EMBED;
            } else if (!strcmp(str_value1[i], "ued-u1")) {
                priv->s_data->g_ctx.streams[i].st_data_type =
                                GMSL_CSI_DT_UED_U1;
            } else {
                dev_err(dev, "invalid stream data type\n");
                return err;
            }
        }

        priv->s_data->g_ctx.s_dev = dev;
        
        mutex_init(&serdes_lock__);
        /* Pair sensor to serializer dev */
        err = max96793_sdev_pair(priv->s_data->ser_dev, &priv->s_data->g_ctx);
        if (err) {
            dev_err(dev, "gmsl ser pairing failed\n");
            return err;
        }

        /* Register sensor to deserializer dev */
        err = max96792_sdev_register(priv->s_data->dser_dev, &priv->s_data->g_ctx);
        if (err) {
            dev_err(dev, "gmsl deserializer register failed\n");
            return err;
        }

        /*
        * gmsl serdes setup
        *
        * Sensor power on/off should be the right place for serdes
        * setup/reset. But the problem is, the total required delay
        * in serdes setup/reset exceeds the frame wait timeout, looks to
        * be related to multiple channel open and close sequence
        * issue (#BUG 200477330).
        * Once this bug is fixed, these may be moved to power on/off.
        * The delays in serdes is as per guidelines and can't be reduced,
        * so it is placed in probe/remove, though for that, deserializer
        * would be powered on always post boot, until 1.2v is supplied
        * to deserializer from CVB.
        */

        err = imx477_gmsl_serdes_setup(priv);
        if (err) {
            dev_err(dev, "%s gmsl serdes setup failed\n", __func__);
            return err;
        }
    
    }
    
    err = camera_common_mclk_enable(s_data);
    if (err) {
        dev_err(dev,
            "Error %d turning on mclk\n", err);
        return err;
    }

    err = imx477_power_on(s_data);
    if (err) {
        dev_err(dev,
            "Error %d during power on sensor\n", err);
        return err;
    }

    err = imx477_communication_verify(priv->tc_dev);
    if (err) {
        dev_err(dev, "%s: unable to communicate with sensor\n",  __func__);
        goto error2;
    }

    err = imx477_calculate_line_time(priv->tc_dev);
    if (err) {
        dev_err(dev, "%s: unable to calculate line time\n", __func__);
        goto error2;
    }

    priv->min_frame_length = IMX477_DEFAULT_HEIGHT 
                                + IMX477_MIN_FRAME_LENGTH_DELTA;
error2:
    imx477_power_off(s_data);
    camera_common_mclk_disable(s_data);

    return err;
}

static int imx477_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
    struct i2c_client *client = v4l2_get_subdevdata(sd);

    dev_dbg(&client->dev, "%s:\n", __func__);

    return 0;
}

static const struct v4l2_subdev_internal_ops imx477_subdev_internal_ops = {
    .open = imx477_open,
};

static struct tegracam_ctrl_ops imx477_ctrl_ops = {
    .numctrls = ARRAY_SIZE(ctrl_cid_list),
    .ctrl_cid_list = ctrl_cid_list,
    .set_gain = imx477_set_gain,
    .set_exposure = imx477_set_exposure,
    .set_frame_rate = imx477_set_frame_rate,
    .set_group_hold = imx477_set_group_hold,
    .set_test_pattern = imx477_set_test_pattern,
    .set_operation_mode = imx477_set_operation_mode,
    .set_broadcast_ctrl = imx477_set_broadcast_ctrl,
    .set_black_level = imx477_set_black_level,
};

static int imx477_probe(struct i2c_client *client,
            const struct i2c_device_id *id)
{
    struct device *dev = &client->dev;
    struct tegracam_device *tc_dev;
    struct imx477 *priv;
    struct sensor_control_properties *ctrlprops = NULL;
    int err;
    unsigned short backup_addr;
    int data;
    int i;
    dev_info(dev, "probing v4l2 sensor\n");
    backup_addr=client->addr;

    for (i=FSM_START_ADDRESS; i<= FSM_END_ADDRESS; i++){
        client->addr=i;
        data = 0;
        data = i2c_smbus_read_byte_data(client, 0x00);

        if(data >= 0){
            dev_info(dev, "Detected sensor eeprom at address 0x%x!", client->addr);
            break;
        }
    }
    if(data < 0)
    {
        dev_err(dev,"Failed to detect sensor eeprom in address range 0x%x-0x%x...aborting\n", FSM_START_ADDRESS, client->addr);
        return data;
    }

    client->addr=backup_addr;
    if (!IS_ENABLED(CONFIG_OF) || !client->dev.of_node)
        return -EINVAL;

    priv = devm_kzalloc(dev,
            sizeof(struct imx477), GFP_KERNEL);
    if (!priv)
        return -ENOMEM;

    tc_dev = devm_kzalloc(dev,
            sizeof(struct tegracam_device), GFP_KERNEL);
    if (!tc_dev)
        return -ENOMEM;

    mutex_init(&priv->pw_mutex);
    priv->i2c_client = tc_dev->client = client;
    tc_dev->dev = dev;
    strncpy(tc_dev->name, "imx477", sizeof(tc_dev->name));
    tc_dev->dev_regmap_config = &sensor_regmap_config;
    tc_dev->sensor_ops = &imx477_common_ops;
    tc_dev->v4l2sd_internal_ops = &imx477_subdev_internal_ops;
    tc_dev->tcctrl_ops = &imx477_ctrl_ops;

    err = tegracam_device_register(tc_dev);
    if (err) {
        dev_err(dev, "tegra camera driver registration failed\n");
        return err;
    }
    priv->tc_dev = tc_dev;
    priv->s_data = tc_dev->s_data;
    priv->subdev = &tc_dev->s_data->subdev;
    tegracam_set_privdata(tc_dev, (void *)priv);

    priv->broadcast_ctrl = UNICAST;
    priv->s_data->broadcast_regmap = NULL;
    priv->current_pixel_format = MEDIA_BUS_FMT_SRGGB10_1X10;

    /* Get default device tree properties of first sensor mode */
    ctrlprops = 
        &priv->s_data->sensor_props.sensor_modes[0].control_properties;

    priv->s_data->exposure_min_range = ctrlprops->min_exp_time.val;
    priv->s_data->exposure_max_range = ctrlprops->max_exp_time.val;

    INIT_LIST_HEAD(&priv->entry);

    err = imx477_board_setup(priv);
    if (err) {
        dev_err(dev, "board setup failed\n");
        return err;
    }

    err = tegracam_v4l2subdev_register(tc_dev, true);
    if (err) {
        dev_err(dev, "tegra camera subdev registration failed\n");
        return err;
    }

    err = imx477_update_ctrl(tc_dev, TEGRA_CAMERA_CID_BLACK_LEVEL, IMX477_DEFAULT_BLACK_LEVEL_10BPP,
                                    IMX477_DEFAULT_BLACK_LEVEL_10BPP, 0, IMX477_MAX_BLACK_LEVEL_10BPP);
    if (err)
        return err;
    
    err = imx477_update_ctrl(tc_dev, TEGRA_CAMERA_CID_TEST_PATTERN, 0, 0, 0, (ARRAY_SIZE(imx477_test_pattern_menu)-1));
    if (err)
        return err;

    list_add_tail(&priv->entry, &imx477_sensor_list);

    dev_info(dev, "Detected imx477 sensor\n");

    return 0;
}

static int
imx477_remove(struct i2c_client *client)
{
    struct camera_common_data *s_data = to_camera_common_data(&client->dev);
    struct imx477 *priv = (struct imx477 *)s_data->priv;

    if (!strcmp(s_data->pdata->gmsl, "gmsl")) {
        imx477_gmsl_serdes_reset(priv);
    }

    tegracam_v4l2subdev_unregister(priv->tc_dev);
    tegracam_device_unregister(priv->tc_dev);

    if (!strcmp(s_data->pdata->gmsl, "gmsl")) {
        mutex_destroy(&serdes_lock__);
    }

    return 0;
}

static const struct i2c_device_id imx477_id[] = {
    { "fsm-imx477", 0 },
    { }
};

MODULE_DEVICE_TABLE(i2c, imx477_id);

static struct i2c_driver imx477_i2c_driver = {
    .driver = {
        .name = "fsm-imx477",
        .owner = THIS_MODULE,
        .of_match_table = of_match_ptr(imx477_of_match),
    },
    .probe = imx477_probe,
    .remove = imx477_remove,
    .id_table = imx477_id,
};

module_i2c_driver(imx477_i2c_driver);

MODULE_DESCRIPTION("Media Controller driver for Sony IMX477");
MODULE_AUTHOR("FRAMOS GmbH");
MODULE_LICENSE("GPL v2");
