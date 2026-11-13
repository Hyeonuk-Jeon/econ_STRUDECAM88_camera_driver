/*
 * ox05b.c - ox05b sensor driver
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

/* #include <linux/debugfs.h> */

#include <media/max96717.h>
#include <media/cipia-max9296.h>

#include <media/max25614.h>

#include <media/tegracam_core.h>

#include "ox05b_mode_tbls.h"

#define OX05B_GAIN_SHIFT_ADDR			0x3507
#define OX05B_GAIN_HIGH_ADDR			0x3508
#define OX05B_GAIN_LOW_ADDR				0x3509

#define OX05B_DEFAULT_GAIN				0x0010 /* 1.0x */
#define OX05B_MIN_GAIN					0x0001
#define OX05B_MAX_GAIN					0x1FFF

#define OX05B_GROUP_HOLD_ADDR			0x3208
#define OX05B_GROUP_HOLD_START			0x00
#define OX05B_GROUP_HOLD_END			0x10
#define OX05B_GROUP_HOLD_LAUNCH_LBLANK	0x60
#define OX05B_GROUP_HOLD_LAUNCH_VBLANK	0xA0
#define OX05B_GROUP_HOLD_LAUNCH_IMMED	0xE0
#define OX05B_GROUP_HOLD_BANK_0			0x00
#define OX05B_GROUP_HOLD_BANK_1			0x01

#define OX05B_EXPO_HIGH_ADDR			0x3500
#define OX05B_EXPO_MID_ADDR				0x3501
#define OX05B_EXPO_LOW_ADDR				0x3502

#define OX05B_TIMING_VTS_HIGH_ADDR		0x380E
#define OX05B_TIMING_VTS_LOW_ADDR		0x380F
#define OX05B_TIMING_FORMAT1			0x3820
#define OX05B_TIMING_FORMAT1_VBIN		(1 << 1)
#define OX05B_TIMING_FORMAT1_FLIP		(1 << 2)
#define OX05B_TIMING_FORMAT2			0x3821
#define OX05B_TIMING_FORMAT2_HBIN		(1 << 0)
#define OX05B_TIMING_FORMAT2_MIRROR		(1 << 2)
#define OX05B_TIMING_RST_FSIN_HIGH_ADDR	0x3826
#define OX05B_TIMING_RST_FSIN_LOW_ADDR	0x3827

const struct of_device_id ox05b_of_match[] = {
	{ .compatible = "nvidia,ox05b",},
	{ },
};
MODULE_DEVICE_TABLE(of, ox05b_of_match);

static const u32 ctrl_cid_list[] = {
	TEGRA_CAMERA_CID_GAIN,
	TEGRA_CAMERA_CID_EXPOSURE,
	TEGRA_CAMERA_CID_FRAME_RATE,
	TEGRA_CAMERA_CID_SENSOR_MODE_ID,
};

struct ox05b {
	struct i2c_client			*i2c_client;
	const struct i2c_device_id 	*id;
	struct v4l2_subdev			*subdev;
	struct device				*ser_dev;
	struct device				*dser_dev;
	struct device				*ledfl_dev;
	struct gmsl_link_ctx		g_ctx;
	u32							frame_length;
	struct camera_common_data	*s_data;
	struct tegracam_device		*tc_dev;
	s32							group_hold_prev;
};

static const struct regmap_config sensor_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
	.cache_type = REGCACHE_NONE,
};

static int test_mode;
module_param(test_mode, int, 0644);

static inline int ox05b_read_reg(struct camera_common_data *s_data,
				u16 addr, u8 *val)
{
	int err = 0;
	u32 reg_val = 0;

	err = regmap_read(s_data->regmap, addr, &reg_val);
	*val = reg_val & 0xFF;

	return err;
}

static int ox05b_write_reg(struct camera_common_data *s_data,
				u16 addr, u16 val)
{
	int err;
	struct device *dev = s_data->dev;

	err = regmap_write(s_data->regmap, addr, val);
	if (err)
		dev_err(dev, "%s:i2c write failed, 0x%x = %x\n",
			__func__, addr, val);

	return err;
}

static int ox05b_write_table(struct ox05b *priv,
				const ox05b_reg table[])
{
	struct camera_common_data *s_data = priv->s_data;

	return regmap_util_write_table_8(
		s_data->regmap,
		table,
		NULL, 0,
		OX05B_TABLE_WAIT_MS,
		OX05B_TABLE_END
	);
}

static struct mutex serdes_lock__;

static int ox05b_gmsl_serdes_setup(struct ox05b *priv)
{
	int err = 0;
	struct device *dev;

	if (!priv || !priv->ser_dev || !priv->dser_dev || !priv->i2c_client)
		return -EINVAL;

	dev = &priv->i2c_client->dev;

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

static void ox05b_gmsl_serdes_reset(struct ox05b *priv)
{
	mutex_lock(&serdes_lock__);

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

static int ox05b_power_on(struct camera_common_data *s_data)
{
	int err = 0;
	struct camera_common_power_rail *pw = s_data->power;
	struct camera_common_pdata *pdata = s_data->pdata;
	struct device *dev = s_data->dev;

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

static int ox05b_power_off(struct camera_common_data *s_data)
{
	int err = 0;
	struct camera_common_power_rail *pw = s_data->power;
	struct camera_common_pdata *pdata = s_data->pdata;
	struct device *dev = s_data->dev;

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

static int ox05b_power_get(struct tegracam_device *tc_dev)
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

static int ox05b_power_put(struct tegracam_device *tc_dev)
{
	struct camera_common_data *s_data = tc_dev->s_data;
	struct camera_common_power_rail *pw = s_data->power;

	if (unlikely(!pw))
		return -EFAULT;

	return 0;
}

static int ox05b_set_group_hold(struct tegracam_device *tc_dev, bool val)
{
	struct ox05b *priv = (struct ox05b *)tc_dev->priv;
	struct device *dev = &priv->i2c_client->dev;
	int gh_prev = switch_ctrl_qmenu[priv->group_hold_prev];
	int err;

	if (gh_prev == SWITCH_OFF) {
		/* group hold start */
		err = ox05b_write_reg(tc_dev->s_data, OX05B_GROUP_HOLD_ADDR,
				      		  (OX05B_GROUP_HOLD_START |
							  OX05B_GROUP_HOLD_BANK_0));
		if (err)
			goto fail;
		priv->group_hold_prev = 1;
	} else {
		/* group hold end */
		err = ox05b_write_reg(tc_dev->s_data, OX05B_GROUP_HOLD_ADDR,
				       		  (OX05B_GROUP_HOLD_END |
							  OX05B_GROUP_HOLD_BANK_0));
		/* quick launch */
		err |= ox05b_write_reg(tc_dev->s_data,
				       		   OX05B_GROUP_HOLD_ADDR,
				       		   (OX05B_GROUP_HOLD_LAUNCH_VBLANK |
							   OX05B_GROUP_HOLD_BANK_0));
		if (err)
			goto fail;
		priv->group_hold_prev = 0;
	}

	return 0;

fail:
	dev_err(dev, "%s: GROUP HOLD control error\n", __func__);
	return err;
}

static int ox05b_set_gain(struct tegracam_device *tc_dev, s64 val)
{
	struct ox05b *priv = (struct ox05b *)tc_dev->priv;
	struct device *dev = &priv->i2c_client->dev;
	ox05b_reg regs[4];
	u16 gain;
	int err;

	if (val < OX05B_MIN_GAIN)
		gain = OX05B_MIN_GAIN;
	else if (val > OX05B_MAX_GAIN)
		gain = OX05B_MAX_GAIN;
	else
		gain = val;

	dev_dbg(dev, "%s: gain: %d\n", __func__, gain);

	regs[0].addr	= OX05B_GAIN_SHIFT_ADDR;
	regs[0].val		= 0x03;
	regs[1].addr	= OX05B_GAIN_HIGH_ADDR;
	regs[1].val		= gain >> 8;
	regs[2].addr	= OX05B_GAIN_LOW_ADDR;
	regs[2].val		= gain & 0xff;
	regs[3].addr	= OX05B_TABLE_END;
	regs[3].val		= 0;

	ox05b_set_group_hold(priv->tc_dev, true);
	err = ox05b_write_table(priv, regs);
	if (err)
		goto fail;

	return 0;

fail:
	dev_err(dev, "%s: GAIN control error\n", __func__);
	return err;
}

static int ox05b_set_frame_rate(struct tegracam_device *tc_dev, s64 val)
{
	struct ox05b *priv = (struct ox05b *)tegracam_get_privdata(tc_dev);

	priv->frame_length = priv->s_data->frmfmt->size.height;

	return 0;
}

static int ox05b_set_exposure(struct tegracam_device *tc_dev, s64 val)
{
	struct ox05b *priv = (struct ox05b *)tc_dev->priv;
	struct device *dev = &priv->i2c_client->dev;
	ox05b_reg regs[4];
	u32 coarse_time;
	int err;

	coarse_time = (u32)val;

	dev_dbg(dev, "%s: coarse_time: %d\n", __func__, coarse_time);

	regs[0].addr	= OX05B_EXPO_HIGH_ADDR;
	regs[0].val		= (coarse_time >> 16) & 0xff;
	regs[1].addr	= OX05B_EXPO_MID_ADDR;
	regs[1].val		= (coarse_time >> 8) & 0xff;
	regs[2].addr	= OX05B_EXPO_LOW_ADDR;
	regs[2].val		= (coarse_time & 0xff);
	regs[3].addr	= OX05B_TABLE_END;
	regs[3].val		= 0;

	ox05b_set_group_hold(priv->tc_dev, true);
	err = ox05b_write_table(priv, regs);
	if (err)
		goto fail;

	return 0;

fail:
	dev_err(dev, "%s: EXPOSURE control error\n", __func__);
	return err;
}

static struct tegracam_ctrl_ops ox05b_ctrl_ops = {
	.numctrls = ARRAY_SIZE(ctrl_cid_list),
	.ctrl_cid_list = ctrl_cid_list,
	.set_gain = ox05b_set_gain,
	.set_exposure = ox05b_set_exposure,
	.set_frame_rate = ox05b_set_frame_rate,
	.set_group_hold = ox05b_set_group_hold,
};

static struct camera_common_pdata *ox05b_parse_dt(struct tegracam_device *tc_dev)
{
	struct device *dev = tc_dev->dev;
	struct device_node *node = dev->of_node;
	struct camera_common_pdata *board_priv_pdata;
	const struct of_device_id *match;
	int err;

	if (!node)
		return NULL;

	match = of_match_device(ox05b_of_match, dev);
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

static int ox05b_set_mode(struct tegracam_device *tc_dev)
{
	struct ox05b *priv = (struct ox05b *)tegracam_get_privdata(tc_dev);
	struct device *dev = tc_dev->dev;
	const struct of_device_id *match;
	int err;
	const char* device_name;

	match = of_match_device(ox05b_of_match, dev);
	if (!match) {
		dev_err(dev, "Failed to find matching dt id\n");
		return -EINVAL;
	}
	
	device_name = dev_name(dev);

	dev_info(dev, "%s: set %s device\n", __func__, device_name);
	dev_info(dev, "%s: Dev name(%s)", __func__, device_name);

	err = ox05b_write_table(priv, mode_table[OX05B_MODE_2592X1944]);
	if (err)
		return err;
	return 0;
}

static int ox05b_start_streaming(struct tegracam_device *tc_dev)
{
	struct ox05b *priv = (struct ox05b *)tegracam_get_privdata(tc_dev);
	struct device *dev = tc_dev->dev;
	int err;

	dev_info(dev, "%s: start streaming\n", __func__);
	/* enable serdes streaming */
	err = max96717_setup_streaming(priv->ser_dev);
	if (err)
		goto exit;

	err = cipia_max9296_setup_streaming(priv->dser_dev, dev);
	if (err)
		goto exit;

	err = cipia_max9296_start_streaming(priv->dser_dev, dev);
	if (err)
	 	goto exit;

	err = ox05b_write_table(priv,
		mode_table[OX05B_MODE_START_STREAM]);

	if (err)
		return err;

	msleep(20);

	return 0;

exit:
	dev_err(dev, "%s: error setting stream\n", __func__);

	return err;
}

static int ox05b_stop_streaming(struct tegracam_device *tc_dev)
{
	struct device *dev = tc_dev->dev;
	struct ox05b *priv = (struct ox05b *)tegracam_get_privdata(tc_dev);
	int err;

	dev_dbg(dev, "%s: stop streaming\n", __func__);
	/* disable serdes streaming */
	cipia_max9296_stop_streaming(priv->dser_dev, dev);

	err = ox05b_write_table(priv, mode_table[OX05B_MODE_STOP_STREAM]);
	if (err)
		return err;

	return 0;
}

static struct camera_common_sensor_ops ox05b_common_ops = {
	.numfrmfmts = ARRAY_SIZE(ox05b_frmfmt),
	.frmfmt_table = ox05b_frmfmt,
	.power_on = ox05b_power_on,
	.power_off = ox05b_power_off,
	.parse_dt = ox05b_parse_dt,
	.power_get = ox05b_power_get,
	.power_put = ox05b_power_put,
	.set_mode = ox05b_set_mode,
	.start_streaming = ox05b_start_streaming,
	.stop_streaming = ox05b_stop_streaming,
};

static int ox05b_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	dev_dbg(&client->dev, "%s:\n", __func__);

	return 0;
}

static const struct v4l2_subdev_internal_ops ox05b_subdev_internal_ops = {
	.open = ox05b_open,
};

static int ox05b_board_setup(struct ox05b *priv)
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

#ifdef OX05B_DEBUGFS_REG_RD_WR

static int ox05b_stats_show(struct seq_file *s, void *data)
{
	return 0;
}

static int ox05b_debugfs_open(struct inode *inode, struct file *file)
{
	return single_open(file, ox05b_stats_show, inode->i_private);
}

static ssize_t ox05b_debugfs_read(struct file *s,
								   char __user *user_buf,
								   size_t count, loff_t *ppos)
{
	struct ox05b *priv = ((struct seq_file *)s->private_data)->private;

	char buf[256];
	int buf_size;
	int i;

	if (!user_buf || count < 1)
		return -EFAULT;

	memset(buf, 0, sizeof(buf));
	buf_size = min(count, sizeof(buf) - 1);

	for (i = 0; i < buf_size; i++) {
		u16 addr = *ppos + i;

		if (ox05b_read_reg(priv->s_data, addr, &buf[i]))
			return -EIO;

		printk("%s: Reading 0x%4x => 0x%2x\n", __func__, addr, buf[i]);
	}

	if (copy_to_user(user_buf, buf, buf_size))
		return -EFAULT;

	return count;
}

static ssize_t ox05b_debugfs_write(struct file *s,
								   const char __user *user_buf,
								   size_t count, loff_t *ppos)
{
	struct ox05b *priv = ((struct seq_file *)s->private_data)->private;
	struct i2c_client *i2c_client = priv->i2c_client;

	char buf[256];
	int buf_size;
	u16 addr;
	u8 val;

	if (!user_buf || count <= 1)
		return -EFAULT;

	memset(buf, 0, sizeof(buf));
	buf_size = min(count, sizeof(buf) - 1);
	if (copy_from_user(buf, user_buf, buf_size))
		return -EFAULT;

	addr = *(u16 *)buf;
	val  = *(u8 *)&buf[2];

	dev_info(&i2c_client->dev, "%s: Writing 0x%2x => 0x%4x\n", __func__, val, addr);
	ox05b_write_reg(priv->s_data, addr, val);

	return count;
}

static const struct file_operations ox05b_debugfs_fops = {
	.open = ox05b_debugfs_open,
	.read = ox05b_debugfs_read,
	.write = ox05b_debugfs_write,
	.llseek = seq_lseek,
	.release = single_release,
};

static int ox05b_debugfs_init(const char *dir_name,
							  struct dentry **d_entry,
							  struct dentry **f_entry,
							  struct ox05b *priv)
{
	struct dentry  *dp, *fp;
	struct i2c_client *i2c_client = priv->i2c_client;

	dp = debugfs_create_dir("ox05b", NULL);
	if (dp == NULL) {
		dev_err(&i2c_client->dev, "%s: debugfs create dir failed\n",
			__func__);
		return -ENOMEM;
	}

	fp = debugfs_create_file("ox05b", S_IRUGO | S_IWUSR | S_IRUSR ,
							 dp, priv, &ox05b_debugfs_fops);
	if (!fp) {
		dev_err(&i2c_client->dev, "%s: debugfs create file failed\n",
			__func__);
		debugfs_remove_recursive(dp);
		return -ENOMEM;
	}

	if (d_entry)
		*d_entry = dp;
	if (f_entry)
		*f_entry = fp;

	return 0;
}

#endif

static int ox05b_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct device_node *node = dev->of_node;
	struct tegracam_device *tc_dev;
	struct ox05b *priv;
	int err;

	dev_info(dev, "probing v4l2 sensor.\n");

	if (!IS_ENABLED(CONFIG_OF) || !node)
		return -EINVAL;

	priv = devm_kzalloc(dev, sizeof(struct ox05b), GFP_KERNEL);
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
	strncpy(tc_dev->name, "ox05b", sizeof(tc_dev->name));
	tc_dev->dev_regmap_config = &sensor_regmap_config;
	tc_dev->sensor_ops = &ox05b_common_ops;
	tc_dev->v4l2sd_internal_ops = &ox05b_subdev_internal_ops;
	tc_dev->tcctrl_ops = &ox05b_ctrl_ops;

	err = tegracam_device_register(tc_dev);
	if (err) {
		dev_err(dev, "tegra camera driver registration failed\n");
		return err;
	}
	priv->tc_dev = tc_dev;
	priv->s_data = tc_dev->s_data;
	priv->subdev = &tc_dev->s_data->subdev;
	tegracam_set_privdata(tc_dev, (void *)priv);

	err = ox05b_board_setup(priv);
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
	err = ox05b_gmsl_serdes_setup(priv);
	if (err) {
		dev_err(&client->dev,
			"%s gmsl serdes setup failed\n", __func__);
		return err;
	}

	/* Configure LED flash driver */
	err  = max25614_write_reg(priv->ledfl_dev, 0x03, 0X60);
	err |= max25614_write_reg(priv->ledfl_dev, 0x04, 0x10);
	err |= max25614_write_reg(priv->ledfl_dev, 0x05, 0x00);
	err |= max25614_write_reg(priv->ledfl_dev, 0x06, 0x01);
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

#ifdef OX05B_DEBUGFS_REG_RD_WR
	err = ox05b_debugfs_init(NULL, NULL, NULL, priv);
	if (err)
		return err;
#endif

	dev_info(&client->dev, "Detected OX05B sensor\n");

	return 0;
}

static int ox05b_remove(struct i2c_client *client)
{
	struct camera_common_data *s_data = to_camera_common_data(&client->dev);
	struct ox05b *priv = (struct ox05b *)s_data->priv;

	ox05b_gmsl_serdes_reset(priv);

	tegracam_device_unregister(priv->tc_dev);

	return 0;
}

static const struct i2c_device_id ox05b_id[] = {
	{ "ox05b", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, ox05b_id);

static struct i2c_driver ox05b_i2c_driver = {
	.driver = {
		.name = "ox05b",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(ox05b_of_match),
	},
	.probe = ox05b_probe,
	.remove = ox05b_remove,
	.id_table = ox05b_id,
};

static int __init ox05b_init(void)
{
	mutex_init(&serdes_lock__);

	return i2c_add_driver(&ox05b_i2c_driver);
}

static void __exit ox05b_exit(void)
{
	mutex_destroy(&serdes_lock__);

	i2c_del_driver(&ox05b_i2c_driver);
}

module_init(ox05b_init);
module_exit(ox05b_exit);

MODULE_DESCRIPTION("Media Controller driver for Omnivision OX05B");
MODULE_AUTHOR("Software Supreme");
MODULE_LICENSE("GPL v2");
