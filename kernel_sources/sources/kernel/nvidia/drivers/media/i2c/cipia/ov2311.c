/*
 * ov2311.c - ov2311 sensor driver
 *
 * Copyright (c) 2018-2019, NVIDIA CORPORATION.  All rights reserved.
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
 *
 */

#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/gpio.h>
#include <linux/module.h>

#include <linux/seq_file.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>

#include <media/max96717.h>
#include <media/cipia-max9296.h>

#include <media/max25614.h>

#include <media/tegracam_core.h>

#include "ov2311_mode_tbls.h"

#define USE_MAX9296_DRIVER_WITH_MAX96717_DRIVER	(1)


#define OV2311_MIN_GAIN                 (0x40)
#define OV2311_MAX_GAIN                 (0x400)
#define OV2311_DEFAULT_FRAME_LENGTH     (1300)
#define OV2311_ANALOG_GAIN_ADDR         (0x0204)

#define OV2311_LONG_EXP_ADDR            0x0202
#define OV2311_MID_EXP_ADDR             0x0216
#define OV2311_SHORT_EXP_ADDR           0x0218
#define OV2311_FINE_SHORT_EXP_ADDR      0x0400

#define OV2311_LINEAR_MIN_LONG_EXP          100
#define OV2311_LINEAR_MAX_LONG_EXP          20000
#define OV2311_LINEAR_MIN_MID_EXP           16
#define OV2311_LINEAR_MAX_MID_EXP           913
#define OV2311_LINEAR_MIN_SHORT_EXP         0
#define OV2311_LINEAR_MAX_SHORT_EXP         8

#define DEBUG_PRINTS_ENABLED

// Debug define, when disabled exposure and gain functions are shorted and return 0 immediately when called
#define ENABLE_EXPOSURE (0)

#ifdef DEBUG_PRINTS_ENABLED
#define DEBUG_PRINT(f_, ...) printk((f_), __VA_ARGS__)
#else
#define DEBUG_PRINT(f_, ...)
#endif

const struct of_device_id ov2311_of_match[] = {
	{ .compatible = "nvidia,ov2311",},
	{ },
};
MODULE_DEVICE_TABLE(of, ov2311_of_match);

static const u32 ctrl_cid_list[] = {
	TEGRA_CAMERA_CID_GAIN,
	TEGRA_CAMERA_CID_EXPOSURE,
	TEGRA_CAMERA_CID_FRAME_RATE,
	TEGRA_CAMERA_CID_SENSOR_MODE_ID,
};

struct ov2311 {
	struct i2c_client	*i2c_client;
	const struct i2c_device_id *id;
	struct v4l2_subdev	*subdev;
	struct device		*ser_dev;
	struct device		*dser_dev;
	struct device		*ledfl_dev;
	struct gmsl_link_ctx	g_ctx;
	u32	frame_length;
	struct camera_common_data	*s_data;
	struct tegracam_device		*tc_dev;
};

static const struct regmap_config sensor_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
	.cache_type = REGCACHE_RBTREE,
};


static int test_mode;
module_param(test_mode, int, 0644);

static inline int ov2311_read_reg(struct camera_common_data *s_data,
				u16 addr, u8 *val)
{
	int err = 0;
	u32 reg_val = 0;

	err = regmap_read(s_data->regmap, addr, &reg_val);
	*val = reg_val & 0xFF;

	DEBUG_PRINT("DBG-MMM %s:read addr 0x%x > val 0x%x\n",__func__, addr, (unsigned char) *val);

	return err;
}

#if (ENABLE_EXPOSURE==1)
static int ov2311_write_reg(struct camera_common_data *s_data,
				u16 addr, u16 val)
{
	int err;
	struct device *dev = s_data->dev;

	DEBUG_PRINT("DBG-MMM %s:write addr 0x%x < val 0x%x\n", __func__, addr, (unsigned int) val);

	err = regmap_write(s_data->regmap, addr, val);
	if (err)
		dev_err(dev, "%s:i2c write failed, 0x%x = %x\n",
			__func__, addr, val);

	return err;
}
#endif

static int ov2311_write_table(struct ov2311 *priv,
				const ov2311_reg table[])
{
	struct camera_common_data *s_data = priv->s_data;

	return regmap_util_write_table_8(
		s_data->regmap,
		table,
		NULL, 0,
		OV2311_TABLE_WAIT_MS,
		OV2311_TABLE_END
	);
}

static struct mutex serdes_lock__;

static int ov2311_gmsl_serdes_setup(struct ov2311 *priv)
{
	int err = 0;
	struct device *dev;

	if (!priv || !priv->ser_dev || !priv->dser_dev || !priv->i2c_client)
		return -EINVAL;

	dev = &priv->i2c_client->dev;
	DEBUG_PRINT("DBG-MMM %s \n", __func__);

	mutex_lock(&serdes_lock__);

	/* For now no separate power on required for serializer device */
	cipia_max9296_power_on(priv->dser_dev);

	/* setup serdes addressing and control pipeline */
	err = cipia_max9296_setup_link(priv->dser_dev, dev);
	if (err) {
		dev_err(dev, "gmsl deserializer link config failed\n");
		goto ret;
	}

	// MAX9296A PHY A Optimization
	err  = cipia_max9296_write_reg(priv->dser_dev, 0x1458, 0x28);
	err |= cipia_max9296_write_reg(priv->dser_dev, 0x1459, 0x68);

	// MAX9296A PHY B Optimization
	err |= cipia_max9296_write_reg(priv->dser_dev, 0x1558, 0x28);
	err |= cipia_max9296_write_reg(priv->dser_dev, 0x1559, 0x68);

	// REG1, bit[1:0], 0x01: 3Gbps, 0x02: 6Gbps
	err |= cipia_max9296_write_reg(priv->dser_dev, 0x0001, 0x01);
	// One-shot reset
	err |= cipia_max9296_write_reg(priv->dser_dev, 0x0010, 0x31);

	msleep(100);
	
	err = max96717_setup_control(priv->ser_dev);
	if (err) {
		dev_err(dev, "gmsl serializer setup failed\n");
		goto ret;
	}

	err = cipia_max9296_setup_control(priv->dser_dev, dev);
	if (err) {
		dev_err(dev, "gmsl deserializer setup failed\n");
		goto ret;
	}

ret:
	mutex_unlock(&serdes_lock__);
	return err;
}

static void ov2311_gmsl_serdes_reset(struct ov2311 *priv)
{
	mutex_lock(&serdes_lock__);
	DEBUG_PRINT("DBG-MMM %s \n", __func__);

	/* reset serdes addressing and control pipeline */
	max96717_reset_control(priv->ser_dev);
	cipia_max9296_reset_control(priv->dser_dev, &priv->i2c_client->dev);

	/* unpair sensor from serializer device */
	max96717_sdev_unpair(priv->ser_dev, priv->g_ctx.s_dev);

	/* Unregister sensor from deserializer device */
	cipia_max9296_sdev_unregister(priv->dser_dev, priv->g_ctx.s_dev);

	cipia_max9296_power_off(priv->dser_dev);

	mutex_unlock(&serdes_lock__);
}

static int ov2311_power_on(struct camera_common_data *s_data)
{
	int err = 0;
	struct camera_common_power_rail *pw = s_data->power;
	struct camera_common_pdata *pdata = s_data->pdata;
	struct device *dev = s_data->dev;
	DEBUG_PRINT("DBG-MMM %s \n", __func__);


	dev_dbg(dev, "%s: power on\n", __func__);
	if (pdata && pdata->power_on) {
		err = pdata->power_on(pw);
		if (err)
			dev_err(dev, "%s failed.\n", __func__);
		else
			pw->state = SWITCH_ON;
		return err;
	}

	pw->state = SWITCH_ON;

	return 0;
}

static int ov2311_power_off(struct camera_common_data *s_data)
{
	int err = 0;
	struct camera_common_power_rail *pw = s_data->power;
	struct camera_common_pdata *pdata = s_data->pdata;
	struct device *dev = s_data->dev;
	DEBUG_PRINT("DBG-MMM %s \n", __func__);

	dev_dbg(dev, "%s:\n", __func__);

	if (pdata && pdata->power_off) {
		err = pdata->power_off(pw);
		if (!err)
			goto power_off_done;
		else
			dev_err(dev, "%s failed.\n", __func__);
		return err;
	}

power_off_done:
	pw->state = SWITCH_OFF;

	return 0;
}

static int ov2311_power_get(struct tegracam_device *tc_dev)
{
	struct device *dev = tc_dev->dev;
	struct camera_common_data *s_data = tc_dev->s_data;
	struct camera_common_power_rail *pw = s_data->power;
	struct camera_common_pdata *pdata = s_data->pdata;
	const char *mclk_name;
	const char *parentclk_name;
	struct clk *parent;
	int err = 0;

	mclk_name = pdata->mclk_name ?
			pdata->mclk_name : "cam_mclk1";
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

	pw->state = SWITCH_OFF;

	return err;
}

static int ov2311_power_put(struct tegracam_device *tc_dev)
{
	struct camera_common_data *s_data = tc_dev->s_data;
	struct camera_common_power_rail *pw = s_data->power;

	if (unlikely(!pw))
		return -EFAULT;

	return 0;
}

static int ov2311_set_group_hold(struct tegracam_device *tc_dev, bool val)
{
	struct device *dev = tc_dev->dev;
	int err;

	/* this function does nothing it is here only to serve the API */

	err = 0;

	if (err) {
		dev_dbg(dev,
			"%s: Group hold control error. (val is %d) \n", __func__, val);
		return err;
	}

	return 0;
}

static int ov2311_set_gain(struct tegracam_device *tc_dev, s64 gain)
{
#if (ENABLE_EXPOSURE==1)
	struct ov2311 *priv = (struct ov2311 *)tegracam_get_privdata(tc_dev);
	struct device *dev = tc_dev->dev;
	int err;
	s64 requestedGain = gain;
	struct camera_common_data *s_data = tc_dev->s_data;
	const struct sensor_mode_properties *mode =
			&s_data->sensor_props.sensor_modes[s_data->mode];

	DEBUG_PRINT("DBG-MMM requested gain from v4l2  %lld", gain);
	gain = gain * 0x40 / mode->control_properties.gain_factor;
	/* at this point gain should be between 0x40 to 0x400 */
	DEBUG_PRINT("DBG-MMM gain reg val (0x40 to 0x400) is 0x%llx and actual gain is (0x%llx/0x40)", gain, gain);

	if (gain > OV2311_MAX_GAIN)
		gain = OV2311_MAX_GAIN;
	if (gain < OV2311_MIN_GAIN)
		gain = OV2311_MIN_GAIN;
	dev_info(dev, "%s: Requested gain %lld (factor=%d). regVal=0x%llx\n", 
		__func__,
		requestedGain,
		mode->control_properties.gain_factor,
		gain
	);
	err = ov2311_write_reg(priv->s_data, OV2311_ANALOG_GAIN_ADDR, gain);

	if (err)
		goto fail;

	return 0;

fail:
	dev_info(dev, "%s: GAIN control error\n", __func__);
	return err;
#else
	return 0;
#endif
}

static int ov2311_set_frame_rate(struct tegracam_device *tc_dev, s64 val)
{
	struct ov2311 *priv = (struct ov2311 *)tegracam_get_privdata(tc_dev);

	priv->frame_length = OV2311_DEFAULT_FRAME_LENGTH;
	DEBUG_PRINT("DBG-MMM trying to change frame rate!!! val = %lld", val);
	return 0;
}

static int ov2311_set_exposure(struct tegracam_device *tc_dev, s64 val)
{
#if (ENABLE_EXPOSURE==1)
	struct ov2311 *priv = (struct ov2311 *)tegracam_get_privdata(tc_dev);
	struct camera_common_data *s_data = tc_dev->s_data;
	int err = 0;
	u16 l_exp, m_exp, s_exp; // Long, Mid and Short exposure setting
	u16 s_exp_fine=0, exp_tmp_val; //Only used in Linear mode

	const struct sensor_mode_properties *mode =
			&s_data->sensor_props.sensor_modes[s_data->mode];

	val = (int)(val * 1000000 / mode->control_properties.exposure_factor);
	dev_info(&priv->i2c_client->dev, "%s: Requested exposure val %lld usec (factor=%d)", 
		__func__, 
		val,
		mode->control_properties.exposure_factor
	);

	/* Divide by 8 for moving from usec units to 1h-time units which is the reg-val units*/
	exp_tmp_val = (u16) ((val - 8) >> 3); 
	l_exp = exp_tmp_val;
	/* Decision: we are not dealing with the find integ time registers. */
	m_exp = l_exp >> 4;
	if(m_exp > (OV2311_LINEAR_MAX_MID_EXP >> 3)) {
		m_exp = 113;
	}
	/* the default value there represents 7.5 usec. */
	exp_tmp_val = val >> 8;
	/* When the coarse is down to 0 then the total exp is 7.5 usec */
	s_exp = exp_tmp_val >> 3;
	/* when the coarse is 1 then the total exp time is 8 + 7.5 = 15.5 usec and so on */
	exp_tmp_val = exp_tmp_val % 8; // (0-7)
	/* Decision: we try to maintain the exposure ratio as close as possible to 1:16:256 */
	if(exp_tmp_val < 6) {
		s_exp_fine = 0xc70;
	} else {
		s_exp_fine = 0x1f30;
	}

	err = ov2311_write_reg(priv->s_data, OV2311_LONG_EXP_ADDR, l_exp);
	err |= ov2311_write_reg(priv->s_data, OV2311_MID_EXP_ADDR, m_exp);
	err |= ov2311_write_reg(priv->s_data, OV2311_SHORT_EXP_ADDR, s_exp);
	err |= ov2311_write_reg(priv->s_data, OV2311_FINE_SHORT_EXP_ADDR, s_exp_fine);

	if (err)
		goto fail;

	return 0;

fail:
	dev_dbg(&priv->i2c_client->dev,
		"%s: set exposure time error\n", __func__);
	return err;
#else
	return 0;
#endif
}

static struct tegracam_ctrl_ops ov2311_ctrl_ops = {
	.numctrls = ARRAY_SIZE(ctrl_cid_list),
	.ctrl_cid_list = ctrl_cid_list,
	.set_gain = ov2311_set_gain,
	.set_exposure = ov2311_set_exposure,
	.set_frame_rate = ov2311_set_frame_rate,
	.set_group_hold = ov2311_set_group_hold,
};

static struct camera_common_pdata *ov2311_parse_dt(struct tegracam_device *tc_dev)
{
	struct device *dev = tc_dev->dev;
	struct device_node *node = dev->of_node;
	struct camera_common_pdata *board_priv_pdata;
	const struct of_device_id *match;
	int err;

	if (!node)
		return NULL;

	match = of_match_device(ov2311_of_match, dev);
	if (!match) {
		dev_err(dev, "Failed to find matching dt id\n");
		return NULL;
	}

	board_priv_pdata = devm_kzalloc(dev, sizeof(*board_priv_pdata), GFP_KERNEL);

	err = of_property_read_string(node, "mclk",
					  &board_priv_pdata->mclk_name);
	if (err)
		dev_err(dev, "mclk not in DT\n");

	return board_priv_pdata;
}

static int ov2311_set_mode(struct tegracam_device *tc_dev)
{
	struct ov2311 *priv = (struct ov2311 *)tegracam_get_privdata(tc_dev);
	struct device *dev = tc_dev->dev;
	const struct of_device_id *match;
	int err;
	const char* device_name;

	match = of_match_device(ov2311_of_match, dev);
	if (!match) {
		dev_err(dev, "Failed to find matching dt id\n");
		return -EINVAL;
	}
	
	device_name = dev_name(dev);

	dev_info(dev, "%s: set %s device\n", __func__, device_name);
	dev_info(dev, "%s: Dev name(%s)", __func__, device_name);

	err = ov2311_write_table(priv, mode_table[OV2311_MODE_1600X1300]);
	if (err)
		return err;
	return 0;
}

static int ov2311_start_streaming(struct tegracam_device *tc_dev)
{
	struct ov2311 *priv = (struct ov2311 *)tegracam_get_privdata(tc_dev);
	struct device *dev = tc_dev->dev;
	int err;

	DEBUG_PRINT("DBG-MMM %s \n", __func__);

	dev_dbg(dev, "%s: start streaming\n", __func__);
	/* enable serdes streaming */
	err = max96717_setup_streaming(priv->ser_dev);
	if (err)
		goto exit;

	err = cipia_max9296_setup_streaming(priv->dser_dev, dev);
	if (err)
		goto exit;

#ifndef USE_MAX9296_DRIVER_WITH_MAX96717_DRIVER
	// Assume that MAX9296 already streams,
	// based on the settings inside the max9296 driver for OV2311 camera
	err = cipia_max9296_start_streaming(priv->dser_dev, dev);
	if (err)
	 	goto exit;
#endif

	err = ov2311_write_table(priv,
		mode_table[OV2311_MODE_START_STREAM]);

	if (err)
		return err;

	msleep(20);

	return 0;

exit:
	dev_err(dev, "%s: error setting stream\n", __func__);

	return err;
}

static int ov2311_stop_streaming(struct tegracam_device *tc_dev)
{
	struct device *dev = tc_dev->dev;
	struct ov2311 *priv = (struct ov2311 *)tegracam_get_privdata(tc_dev);
	int err;

	DEBUG_PRINT("DBG-MMM %s \n", __func__);

	dev_dbg(dev, "%s: stop streaming\n", __func__);
	/* disable serdes streaming */
	cipia_max9296_stop_streaming(priv->dser_dev, dev);

	err = ov2311_write_table(priv, mode_table[OV2311_MODE_STOP_STREAM]);
	if (err)
		return err;

	return 0;
}

static struct camera_common_sensor_ops ov2311_common_ops = {
	.numfrmfmts = ARRAY_SIZE(ov2311_frmfmt),
	.frmfmt_table = ov2311_frmfmt,
	.power_on = ov2311_power_on,
	.power_off = ov2311_power_off,
//	.write_reg = ov2311_write_reg,
//	.read_reg = ov2311_read_reg,
	.parse_dt = ov2311_parse_dt,
	.power_get = ov2311_power_get,
	.power_put = ov2311_power_put,
	.set_mode = ov2311_set_mode,
	.start_streaming = ov2311_start_streaming,
	.stop_streaming = ov2311_stop_streaming,
};

static int ov2311_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	dev_dbg(&client->dev, "%s:\n", __func__);

	return 0;
}

static const struct v4l2_subdev_internal_ops ov2311_subdev_internal_ops = {
	.open = ov2311_open,
};

static int ov2311_board_setup(struct ov2311 *priv)
{
	struct tegracam_device *tc_dev = priv->tc_dev;
	struct device *dev = tc_dev->dev;
	struct device_node *node = dev->of_node;
	struct device_node *ser_node;
	struct i2c_client *ser_i2c = NULL;
	struct device_node *dser_node;
	struct i2c_client *dser_i2c = NULL;
	struct device_node *ledfl_node;
	struct i2c_client *ledfl_i2c = NULL;
	struct device_node *gmsl;
	int value = 0xFFFF;
	const char *str_value;
	const char *str_value1[2];
	int  i;
	int err;

	err = of_property_read_u32(node, "reg", &priv->g_ctx.sdev_reg);
	if (err < 0) {
		dev_err(dev, "reg not found\n");
		goto error;
	}

	err = of_property_read_u32(node, "def-addr",
					&priv->g_ctx.sdev_def);
	if (err < 0) {
		dev_err(dev, "def-addr not found\n");
		goto error;
	}

	ser_node = of_parse_phandle(node, "nvidia,gmsl-ser-device", 0);
	if (ser_node == NULL) {
		dev_err(dev,
			"missing %s handle\n",
				"nvidia,gmsl-ser-device");
		goto error;
	}

	err = of_property_read_u32(ser_node, "reg", &priv->g_ctx.ser_reg);
	if (err < 0) {
		dev_err(dev, "serializer reg not found\n");
		goto error;
	}

	ser_i2c = of_find_i2c_device_by_node(ser_node);
	of_node_put(ser_node);

	if (ser_i2c == NULL) {
		dev_err(dev, "missing serializer dev handle\n");
		goto error;
	}
	if (ser_i2c->dev.driver == NULL) {
		dev_err(dev, "missing serializer driver\n");
		goto error;
	}

	priv->ser_dev = &ser_i2c->dev;

	dser_node = of_parse_phandle(node, "nvidia,gmsl-dser-device", 0);
	if (dser_node == NULL) {
		dev_err(dev,
			"missing %s handle\n",
				"nvidia,gmsl-dser-device");
		goto error;
	}

	dser_i2c = of_find_i2c_device_by_node(dser_node);
	of_node_put(dser_node);

	if (dser_i2c == NULL) {
		dev_err(dev, "missing deserializer dev handle\n");
		goto error;
	}
	if (dser_i2c->dev.driver == NULL) {
		dev_err(dev, "missing deserializer driver\n");
		goto error;
	}

	priv->dser_dev = &dser_i2c->dev;

	ledfl_node = of_parse_phandle(node, "ledfl", 0);
	if (ledfl_node == NULL) {
		dev_err(dev, "missing %s handle\n", "ledfl");
		goto error;
	}

	ledfl_i2c = of_find_i2c_device_by_node(ledfl_node);
	of_node_put(ledfl_node);

	if (ledfl_i2c == NULL) {
		dev_err(dev, "missing LED flash dev handle\n");
		goto error;
	}
	if (ledfl_i2c->dev.driver == NULL) {
		dev_err(dev, "missing LED flash driver\n");
		goto error;
	}

	priv->ledfl_dev = &ledfl_i2c->dev;

	/* populate g_ctx from DT */
	gmsl = of_get_child_by_name(node, "gmsl-link");
	if (gmsl == NULL) {
		dev_err(dev, "missing gmsl-link device node\n");
		err = -EINVAL;
		goto error;
	}

	err = of_property_read_string(gmsl, "dst-csi-port", &str_value);
	if (err < 0) {
		dev_err(dev, "No dst-csi-port found\n");
		goto error;
	}
	priv->g_ctx.dst_csi_port =
		(!strcmp(str_value, "a")) ? GMSL_CSI_PORT_A : GMSL_CSI_PORT_B;

	err = of_property_read_string(gmsl, "src-csi-port", &str_value);
	if (err < 0) {
		dev_err(dev, "No src-csi-port found\n");
		goto error;
	}
	priv->g_ctx.src_csi_port =
		(!strcmp(str_value, "a")) ? GMSL_CSI_PORT_A : GMSL_CSI_PORT_B;

	err = of_property_read_string(gmsl, "csi-mode", &str_value);
	if (err < 0) {
		dev_err(dev, "No csi-mode found\n");
		goto error;
	}

	if (!strcmp(str_value, "1x4")) {
		priv->g_ctx.csi_mode = GMSL_CSI_1X4_MODE;
	} else if (!strcmp(str_value, "2x4")) {
		priv->g_ctx.csi_mode = GMSL_CSI_2X4_MODE;
	} else if (!strcmp(str_value, "4x2")) {
		priv->g_ctx.csi_mode = GMSL_CSI_4X2_MODE;
	} else if (!strcmp(str_value, "2x2")) {
		priv->g_ctx.csi_mode = GMSL_CSI_2X2_MODE;
	} else {
		dev_err(dev, "invalid csi mode\n");
		goto error;
	}

	err = of_property_read_string(gmsl, "serdes-csi-link", &str_value);
	if (err < 0) {
		dev_err(dev, "No serdes-csi-link found\n");
		goto error;
	}
	priv->g_ctx.serdes_csi_link =
		(!strcmp(str_value, "a")) ?
			GMSL_SERDES_CSI_LINK_A : GMSL_SERDES_CSI_LINK_B;

	
	err = of_property_read_u32(gmsl, "num-lanes", &value);
	if (err < 0) {
		dev_err(dev, "No num-lanes info\n");
		goto error;
	}
	priv->g_ctx.num_csi_lanes = value;

	priv->g_ctx.num_streams =
			of_property_count_strings(gmsl, "streams");
	if (priv->g_ctx.num_streams <= 0) {
		dev_err(dev, "No streams found\n");
		err = -EINVAL;
		goto error;
	}

	for (i = 0; i < priv->g_ctx.num_streams; i++) {
		of_property_read_string_index(gmsl, "streams", i,
						&str_value1[i]);
		if (!str_value1[i]) {
			dev_err(dev, "invalid stream info\n");
			goto error;
		}
		if (!strcmp(str_value1[i], "raw12")) {
			priv->g_ctx.streams[i].st_data_type =
							GMSL_CSI_DT_RAW_12;
		} else if (!strcmp(str_value1[i], "embed")) {
			priv->g_ctx.streams[i].st_data_type =
							GMSL_CSI_DT_EMBED;
		} else if (!strcmp(str_value1[i], "ued-u1")) {
			priv->g_ctx.streams[i].st_data_type =
							GMSL_CSI_DT_UED_U1;
		} else {
			dev_err(dev, "invalid stream data type\n");
			goto error;
		}
	}

	priv->g_ctx.s_dev = dev;

	return 0;

error:
	dev_err(dev, "board setup failed\n");
	return err;
}

static int ov2311_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct device_node *node = dev->of_node;
	struct tegracam_device *tc_dev;
	struct ov2311 *priv;
	int err;

	DEBUG_PRINT("DBG-MMM %s \n", __func__);
	dev_info(dev, "probing v4l2 sensor.\n");

	if (!IS_ENABLED(CONFIG_OF) || !node)
		return -EINVAL;

	priv = devm_kzalloc(dev, sizeof(struct ov2311), GFP_KERNEL);
	if (!priv) {
		dev_err(dev, "unable to allocate memory!\n");
		return -ENOMEM;
	}
	tc_dev = devm_kzalloc(dev,
			sizeof(struct tegracam_device), GFP_KERNEL);
	if (!tc_dev)
		return -ENOMEM;

	priv->i2c_client = tc_dev->client = client;
	tc_dev->dev = dev;
	strncpy(tc_dev->name, "ov2311", sizeof(tc_dev->name));
	tc_dev->dev_regmap_config = &sensor_regmap_config;
	tc_dev->sensor_ops = &ov2311_common_ops;
	tc_dev->v4l2sd_internal_ops = &ov2311_subdev_internal_ops;
	tc_dev->tcctrl_ops = &ov2311_ctrl_ops;

	err = tegracam_device_register(tc_dev);
	if (err) {
		dev_err(dev, "tegra camera driver registration failed\n");
		return err;
	}
	priv->tc_dev = tc_dev;
	priv->s_data = tc_dev->s_data;
	priv->subdev = &tc_dev->s_data->subdev;
	tegracam_set_privdata(tc_dev, (void *)priv);

	err = ov2311_board_setup(priv);
	if (err) {
		tegracam_device_unregister(tc_dev);
		dev_err(dev, "board setup failed\n");
		return err;
	}

	/* Pair sensor to serializer dev */
	err = max96717_sdev_pair(priv->ser_dev, &priv->g_ctx);
	if (err) {
		dev_err(&client->dev, "gmsl ser pairing failed\n");
		return err;
	}

	/* Register sensor to deserializer dev */
	err = cipia_max9296_sdev_register(priv->dser_dev, &priv->g_ctx);
	if (err) {
		dev_err(&client->dev, "gmsl deserializer register failed\n");
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
	err = ov2311_gmsl_serdes_setup(priv);
	if (err) {
		dev_err(&client->dev,
			"%s gmsl serdes setup failed\n", __func__);
		return err;
	}

	/* Configure LED flash driver */
	err  = max25614_write_reg(priv->ledfl_dev, 0x03, 0x5b);
	err |= max25614_write_reg(priv->ledfl_dev, 0x04, 0x10);
	err |= max25614_write_reg(priv->ledfl_dev, 0x05, 0x00);
	err |= max25614_write_reg(priv->ledfl_dev, 0x06, 0x01);
	err |= max25614_write_reg(priv->ledfl_dev, 0x09, 0x50);
	err |= max25614_write_reg(priv->ledfl_dev, 0x02, 0xe1);
	if (err) {
		dev_err(&client->dev,
			"%s LED flash setup failed\n", __func__);
		return err;
	}

	err = tegracam_v4l2subdev_register(tc_dev, true);
	if (err) {
		dev_err(dev, "tegra camera subdev registration failed\n");
		return err;
	}

	dev_info(&client->dev, "Detected OV2311 sensor\n");

	return 0;
}

static int ov2311_remove(struct i2c_client *client)
{
	struct camera_common_data *s_data = to_camera_common_data(&client->dev);
	struct ov2311 *priv = (struct ov2311 *)s_data->priv;

	ov2311_gmsl_serdes_reset(priv);

	tegracam_v4l2subdev_unregister(priv->tc_dev);

	//tegracam_device_unregister(priv->tc_dev);

	return 0;
}

static const struct i2c_device_id ov2311_id[] = {
	{ "ov2311", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, ov2311_id);

static struct i2c_driver ov2311_i2c_driver = {
	.driver = {
		.name = "ov2311",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(ov2311_of_match),
	},
	.probe = ov2311_probe,
	.remove = ov2311_remove,
	.id_table = ov2311_id,
};

static int __init ov2311_init(void)
{
	mutex_init(&serdes_lock__);

	return i2c_add_driver(&ov2311_i2c_driver);
}

static void __exit ov2311_exit(void)
{
	mutex_destroy(&serdes_lock__);

	i2c_del_driver(&ov2311_i2c_driver);
}

module_init(ov2311_init);
module_exit(ov2311_exit);

MODULE_DESCRIPTION("Media Controller driver for Omnivision OV2311");
MODULE_AUTHOR("Software Supreme");
MODULE_LICENSE("GPL v2");
