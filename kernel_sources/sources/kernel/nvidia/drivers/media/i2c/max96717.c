/*
 * max96717.c - max96717 GMSL Serializer driver
 *
 * Copyright (c) 2018-2022, NVIDIA CORPORATION.  All rights reserved.
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

#include <media/camera_common.h>
#include <linux/module.h>
#include <media/max96717.h>



#define MAX96717_DRIVER_VER_0	(0)
#define MAX96717_DRIVER_VER_1	(1)

#define MAX96717_DRIVER_VER		(MAX96717_DRIVER_VER_0)



/* register specifics */
#define MAX96717_DEV_ADDR 0x00
#define MAX96717_I2C4_ADDR 0x44
#define MAX96717_I2C5_ADDR 0x45

#define MAX96717_CTRL0_ADDR 0x10
#define MAX96717_RESET_ALL 0x80

#if (MAX96717_DRIVER_VER != MAX96717_DRIVER_VER_0)
#define MAX96717_MIPI_RX0_ADDR 0x330
#define MAX96717_MIPI_RX1_ADDR 0x331
#define MAX96717_MIPI_RX2_ADDR 0x332
#define MAX96717_MIPI_RX3_ADDR 0x333

#define MAX96717_PIPE_X_DT_ADDR 0x314
#define MAX96717_PIPE_Y_DT_ADDR 0x316
#define MAX96717_PIPE_Z_DT_ADDR 0x318
#define MAX96717_PIPE_U_DT_ADDR 0x31A

#define MAX96717_SRC_CTRL_ADDR 0x2BF
#define MAX96717_SRC_PWDN_ADDR 0x02BE
#define MAX96717_SRC_OUT_RCLK_ADDR 0x3F1
#define MAX96717_START_PIPE_ADDR 0x311
#define MAX96717_PIPE_EN_ADDR 0x2
#define MAX96717_CSI_PORT_SEL_ADDR 0x308

#define MAX96717_STREAM_PIPE_UNUSED 0x22
#define MAX96717_CSI_MODE_1X4 0x00
#define MAX96717_CSI_MODE_2X2 0x03
#define MAX96717_CSI_MODE_2X4 0x06

#define MAX96717_CSI_PORT_B(num_lanes) (((num_lanes) << 4) & 0xF0)
#define MAX96717_CSI_PORT_A(num_lanes) ((num_lanes) & 0x0F)

#define MAX96717_CSI_1X4_MODE_LANE_MAP1 0xE0
#define MAX96717_CSI_1X4_MODE_LANE_MAP2 0x04

#define MAX96717_CSI_2X4_MODE_LANE_MAP1 0xEE
#define MAX96717_CSI_2X4_MODE_LANE_MAP2 0xE4

#define MAX96717_CSI_2X2_MODE_LANE_MAP1 MAX96717_CSI_2X4_MODE_LANE_MAP1
#define MAX96717_CSI_2X2_MODE_LANE_MAP2 MAX96717_CSI_2X4_MODE_LANE_MAP2

#define MAX96717_ST_ID_0 0x0
#define MAX96717_ST_ID_1 0x1
#define MAX96717_ST_ID_2 0x2
#define MAX96717_ST_ID_3 0x3

#define MAX96717_PIPE_X_START_B 0x80
#define MAX96717_PIPE_Y_START_B 0x40
#define MAX96717_PIPE_Z_START_B 0x20
#define MAX96717_PIPE_U_START_B 0x10

#define MAX96717_PIPE_X_START_A 0x1
#define MAX96717_PIPE_Y_START_A 0x2
#define MAX96717_PIPE_Z_START_A 0x4
#define MAX96717_PIPE_U_START_A 0x8

#define MAX96717_START_PORT_A 0x10
#define MAX96717_START_PORT_B 0x20

#define MAX96717_CSI_LN2 0x1
#define MAX96717_CSI_LN4 0x3

#define MAX96717_EN_LINE_INFO 0x40

#define MAX96717_VID_TX_EN_X 0x10
#define MAX96717_VID_TX_EN_Y 0x20
#define MAX96717_VID_TX_EN_Z 0x40
#define MAX96717_VID_TX_EN_U 0x80

#define MAX96717_VID_INIT 0x3
#define MAX96717_SRC_RCLK 0x89

#define MAX96717_RESET_ALL 0x80
#define MAX96717_RESET_SRC 0x60
#define MAX96717_PWDN_GPIO 0x90

#define MAX96717_MAX_PIPES 0x4
#endif

struct max96717_client_ctx {
	struct gmsl_link_ctx *g_ctx;
	bool st_done;
};

struct max96717 {
	struct i2c_client *i2c_client;
	struct regmap *regmap;
	struct max96717_client_ctx g_client;
	struct mutex lock;
	/* primary serializer properties */
	__u32 def_addr;
	__u32 pst2_ref;
	bool is_3ghz;
};

static struct max96717 *prim_priv__;

#if (MAX96717_DRIVER_VER != MAX96717_DRIVER_VER_0)
struct map_ctx {
	u8 dt;
	u16 addr;
	u8 val;
	u8 st_id;
};
#endif

int max96717_write_reg(struct device *dev, u16 addr, u8 val)
{
	struct max96717 *priv = dev_get_drvdata(dev);
	int err;

	err = regmap_write(priv->regmap, addr, val);
	// if (err)
	// 	dev_err(dev, "%s:i2c write failed, 0x%x = %x\n",
	// 		__func__, addr, val);

	/* delay before next i2c command as required for SERDES link */
	usleep_range(100, 110);

	return err;
}
EXPORT_SYMBOL(max96717_write_reg);

int max96717_read_reg(struct device *dev, u16 addr, u32 *val)
{
	struct max96717 *priv = dev_get_drvdata(dev);
	int err;

	err = regmap_read(priv->regmap, addr, val);
	if (err)
		dev_err(dev, "%s:i2c read failed, 0x%x = %x\n",
			__func__, addr, *val);

	return err;
}
EXPORT_SYMBOL(max96717_read_reg);

int max96717_setup_streaming(struct device *dev)
{
	struct max96717 *priv = dev_get_drvdata(dev);
	int err = 0;

#if (MAX96717_DRIVER_VER == MAX96717_DRIVER_VER_0)

	// In this versio is assumed that Setup Streaming is already done in
	// max96717_setup_control()

	priv->g_client.st_done = true;

#else
#error "Unsupported driver version - revisit the code below"
	u32 csi_mode;
	u32 lane_map1;
	u32 lane_map2;
	u32 port;
	u32 rx1_lanes;
	u32 st_pipe;
	u32 pipe_en;
	u32 port_sel = 0;
	struct gmsl_link_ctx *g_ctx;
	u32 i;
	u32 j;
	u32 st_en;

	struct map_ctx map_pipe_dtype[] = {
		{GMSL_CSI_DT_RAW_12, MAX96717_PIPE_Z_DT_ADDR, 0x2C,
			MAX96717_ST_ID_2},
		{GMSL_CSI_DT_UED_U1, MAX96717_PIPE_X_DT_ADDR, 0x30,
			MAX96717_ST_ID_0},
		{GMSL_CSI_DT_EMBED, MAX96717_PIPE_Y_DT_ADDR, 0x12,
			MAX96717_ST_ID_1},
	};

	mutex_lock(&priv->lock);

	if (!priv->g_client.g_ctx) {
		dev_err(dev, "%s: no sdev client found\n", __func__);
		err = -EINVAL;
		goto error;
	}

	if (priv->g_client.st_done) {
		dev_dbg(dev, "%s: stream setup is already done\n", __func__);
		goto error;
	}

	g_ctx = priv->g_client.g_ctx;

	switch (g_ctx->csi_mode) {
	case GMSL_CSI_1X4_MODE:
		csi_mode = MAX96717_CSI_MODE_1X4;
		lane_map1 = MAX96717_CSI_1X4_MODE_LANE_MAP1;
		lane_map2 = MAX96717_CSI_1X4_MODE_LANE_MAP2;
		rx1_lanes = MAX96717_CSI_LN4;
		break;
	case GMSL_CSI_2X2_MODE:
		csi_mode = MAX96717_CSI_MODE_2X2;
		lane_map1 = MAX96717_CSI_2X2_MODE_LANE_MAP1;
		lane_map2 = MAX96717_CSI_2X2_MODE_LANE_MAP2;
		rx1_lanes = MAX96717_CSI_LN2;
		break;
	case GMSL_CSI_2X4_MODE:
		csi_mode = MAX96717_CSI_MODE_2X4;
		lane_map1 = MAX96717_CSI_2X4_MODE_LANE_MAP1;
		lane_map2 = MAX96717_CSI_2X4_MODE_LANE_MAP2;
		rx1_lanes = MAX96717_CSI_LN4;
		break;
	default:
		dev_err(dev, "%s: invalid csi mode\n", __func__);
		err = -EINVAL;
		goto error;
	}

	port = (g_ctx->src_csi_port == GMSL_CSI_PORT_B) ?
			MAX96717_CSI_PORT_B(rx1_lanes) :
			MAX96717_CSI_PORT_A(rx1_lanes);

	max96717_write_reg(dev, MAX96717_MIPI_RX0_ADDR, csi_mode);
	max96717_write_reg(dev, MAX96717_MIPI_RX1_ADDR, port);
	max96717_write_reg(dev, MAX96717_MIPI_RX2_ADDR, lane_map1);
	max96717_write_reg(dev, MAX96717_MIPI_RX3_ADDR, lane_map2);

	for (i = 0; i < g_ctx->num_streams; i++) {
		struct gmsl_stream *g_stream = &g_ctx->streams[i];

		g_stream->st_id_sel = GMSL_ST_ID_UNUSED;
		for (j = 0; j < ARRAY_SIZE(map_pipe_dtype); j++) {
			if (map_pipe_dtype[j].dt == g_stream->st_data_type) {
				/*
				 * TODO:
				 * 1) Remove link specific overrides, depends
				 * on #2.
				 * 2) Add support for vc id based stream sel
				 * overrides TX_SRC_SEL. would be useful in
				 * using same mappings in all ser devs.
				 */
				if (g_ctx->serdes_csi_link ==
					GMSL_SERDES_CSI_LINK_B) {
					map_pipe_dtype[j].addr += 2;
					map_pipe_dtype[j].st_id += 1;
				}

				g_stream->st_id_sel = map_pipe_dtype[j].st_id;
				st_en = (map_pipe_dtype[j].addr ==
						MAX96717_PIPE_X_DT_ADDR) ?
							0xC0 : 0x40;

				max96717_write_reg(dev, map_pipe_dtype[j].addr,
					(st_en | map_pipe_dtype[j].val));
			}
		}
	}

	for (i = 0; i < g_ctx->num_streams; i++)
		if (g_ctx->streams[i].st_id_sel != GMSL_ST_ID_UNUSED)
			port_sel |= (1 << g_ctx->streams[i].st_id_sel);

	if (g_ctx->src_csi_port == GMSL_CSI_PORT_B) {
		st_pipe = (MAX96717_PIPE_X_START_B | MAX96717_PIPE_Y_START_B |
			MAX96717_PIPE_Z_START_B | MAX96717_PIPE_U_START_B);
		port_sel |= (MAX96717_EN_LINE_INFO | MAX96717_START_PORT_B);
	} else {
		st_pipe = MAX96717_PIPE_X_START_A | MAX96717_PIPE_Y_START_A |
			MAX96717_PIPE_Z_START_A | MAX96717_PIPE_U_START_A;
		port_sel |= (MAX96717_EN_LINE_INFO | MAX96717_START_PORT_A);
	}

	pipe_en = (MAX96717_VID_TX_EN_X | MAX96717_VID_TX_EN_Y |
		MAX96717_VID_TX_EN_Z | MAX96717_VID_TX_EN_U | MAX96717_VID_INIT);

	max96717_write_reg(dev, MAX96717_START_PIPE_ADDR, st_pipe);
	max96717_write_reg(dev, MAX96717_CSI_PORT_SEL_ADDR, port_sel);
	max96717_write_reg(dev, MAX96717_PIPE_EN_ADDR, pipe_en);

	priv->g_client.st_done = true;

error:
	mutex_unlock(&priv->lock);
#endif

	return err;
}
EXPORT_SYMBOL(max96717_setup_streaming);

int max96717_setup_control(struct device *dev)
{
	struct max96717 *priv = dev_get_drvdata(dev);
	int err = 0;
	struct gmsl_link_ctx *g_ctx;

	mutex_lock(&priv->lock);

	if (!priv->g_client.g_ctx) {
		dev_err(dev, "%s: no sensor dev client found\n", __func__);
		err = -EINVAL;
		goto error;
	}

	g_ctx = priv->g_client.g_ctx;

	/* dev addr pass-through2 ref */
	if (prim_priv__)
		prim_priv__->pst2_ref++;

	msleep(100);
	dev_err(dev, "max96717_setup_control GMSL use 3GHz -> %d\n", (int)priv->is_3ghz);
	if (priv->is_3ghz) {
		err  = max96717_write_reg(dev, 0x0302, 0x10);		// 717 MANDATORY REGISTER
		err |= max96717_write_reg(dev, 0x0001, 0x04);		// REG1, bit[3:2], 0x04: 3Gbps, 0x08: 6Gbps
		
		err |= max96717_write_reg(dev, 0x0383, 0x00);		// Disable tunneling mode
		err |= max96717_write_reg(dev, 0x0318, 0x6B);		// RAW10 to pipe Z

		err |= max96717_write_reg(dev, 0x0331, 0x10);		// MIPI_RX1, bit[5:4], 0x30: Four data lanes, 0x10: Two data lanes
		err |= max96717_write_reg(dev, 0x02BE, 0x89);		// GPIO0 - GPIO pin output is driven to 0
		err |= max96717_write_reg(dev, 0x02D0, 0x89);		// GPIO6 - GPIO pin output is driven to 0
	} else {
		err  = max96717_write_reg(dev, 0x0302, 0x10);			// 717 MANDATORY REGISTER
		err |= max96717_write_reg(dev, 0x0001, 0x08);			// REG1, bit[3:2], 0x04: 3Gbps, 0x08: 6Gbps

		err |= max96717_write_reg(dev, 0x0383, 0x00);			// Disable tunneling mode
		err |= max96717_write_reg(dev, 0x0318, 0x6B);			// RAW10 to pipe Z

		err |= max96717_write_reg(dev, 0x0331, 0x30 | 0x40);	// MIPI_RX1, bit[5:4], 0x30: Four data lanes, 0x10: Two data lanes
		err |= max96717_write_reg(dev, 0x02BE, 0x89);			// GPIO0 - GPIO pin output is driven to 0
		err |= max96717_write_reg(dev, 0x02D0, 0x89);			// GPIO6 - GPIO pin output is driven to 0
	}
	
	/* check if serializer device exists */
	if (err) {
		dev_err(dev, "%s: ERROR: ser device not found\n", __func__);
		goto error;
	}

	g_ctx->serdev_found = true;

error:
	mutex_unlock(&priv->lock);
	return err;
}
EXPORT_SYMBOL(max96717_setup_control);

int max96717_reset_control(struct device *dev)
{
	struct max96717 *priv = dev_get_drvdata(dev);
	int err = 0;

	mutex_lock(&priv->lock);
	if (!priv->g_client.g_ctx) {
		dev_err(dev, "%s: no sdev client found\n", __func__);
		err = -EINVAL;
		goto error;
	}

	priv->g_client.st_done = false;

	if (prim_priv__) {
		prim_priv__->pst2_ref--;

		max96717_write_reg(dev, MAX96717_DEV_ADDR,
					(prim_priv__->def_addr << 1));

		max96717_write_reg(&prim_priv__->i2c_client->dev,
					MAX96717_CTRL0_ADDR, MAX96717_RESET_ALL);
	}

error:
	mutex_unlock(&priv->lock);
	return err;
}
EXPORT_SYMBOL(max96717_reset_control);

int max96717_sdev_pair(struct device *dev, struct gmsl_link_ctx *g_ctx)
{
	struct max96717 *priv;
	int err = 0;

	if (!dev || !g_ctx || !g_ctx->s_dev) {
		dev_err(dev, "%s: invalid input params\n", __func__);
		return -EINVAL;
	}

	priv = dev_get_drvdata(dev);
	mutex_lock(&priv->lock);
	if (priv->g_client.g_ctx) {
		dev_err(dev, "%s: device already paired\n", __func__);
		err = -EINVAL;
		goto error;
	}

	priv->g_client.st_done = false;

	priv->g_client.g_ctx = g_ctx;

error:
	mutex_unlock(&priv->lock);
	return 0;
}
EXPORT_SYMBOL(max96717_sdev_pair);

int max96717_sdev_unpair(struct device *dev, struct device *s_dev)
{
	struct max96717 *priv = NULL;
	int err = 0;

	if (!dev || !s_dev) {
		dev_err(dev, "%s: invalid input params\n", __func__);
		return -EINVAL;
	}

	priv = dev_get_drvdata(dev);

	mutex_lock(&priv->lock);

	if (!priv->g_client.g_ctx) {
		dev_err(dev, "%s: device is not paired\n", __func__);
		err = -ENOMEM;
		goto error;
	}

	if (priv->g_client.g_ctx->s_dev != s_dev) {
		dev_err(dev, "%s: invalid device\n", __func__);
		err = -EINVAL;
		goto error;
	}

	priv->g_client.g_ctx = NULL;
	priv->g_client.st_done = false;

error:
	mutex_unlock(&priv->lock);
	return err;
}
EXPORT_SYMBOL(max96717_sdev_unpair);

static  struct regmap_config max96717_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
	.cache_type = REGCACHE_RBTREE,
};

static int max96717_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	struct max96717 *priv;
	int err = 0;
	int value = 0;
	struct device_node *node = client->dev.of_node;

	dev_info(&client->dev, "[MAX96717]: probing GMSL Serializer\n");

	priv = devm_kzalloc(&client->dev, sizeof(*priv), GFP_KERNEL);
	priv->i2c_client = client;
	priv->regmap = devm_regmap_init_i2c(priv->i2c_client,
				&max96717_regmap_config);
	if (IS_ERR(priv->regmap)) {
		dev_err(&client->dev,
			"regmap init failed: %ld\n", PTR_ERR(priv->regmap));
		return -ENODEV;
	}

	mutex_init(&priv->lock);

	if (of_get_property(node, "is-prim-ser", NULL)) {
		if (prim_priv__) {
			dev_err(&client->dev,
				"prim-ser already exists\n");
				return -EEXIST;
		}

		err = of_property_read_u32(node, "reg", &priv->def_addr);
		if (err < 0) {
			dev_err(&client->dev, "reg not found\n");
			return -EINVAL;
		}

		prim_priv__ = priv;
	}

	err = of_property_read_u32(node, "low-freq-mode", &value);
	if (err < 0) {
		dev_err(&client->dev, "low-freq-mode not found\n");
		return err;
	}
	priv->is_3ghz = value;
	dev_err(&client->dev, "max96717_probe GMSL use 3GHz -> %d\n", (int)priv->is_3ghz);

	dev_set_drvdata(&client->dev, priv);

	/* dev communication gets validated when GMSL link setup is done */
	dev_info(&client->dev, "%s:  success\n", __func__);

	return err;
}

static int max96717_remove(struct i2c_client *client)
{
	struct max96717 *priv;

	if (client != NULL) {
		priv = dev_get_drvdata(&client->dev);
		mutex_destroy(&priv->lock);
		i2c_unregister_device(client);
		client = NULL;
	}

	return 0;
}

static const struct i2c_device_id max96717_id[] = {
	{ "max96717", 0 },
	{ },
};

static const struct of_device_id max96717_of_match[] = {
	{ .compatible = "nvidia,max96717", },
	{ },
};
MODULE_DEVICE_TABLE(of, max96717_of_match);
MODULE_DEVICE_TABLE(i2c, max96717_id);

static struct i2c_driver max96717_i2c_driver = {
	.driver = {
		.name = "max96717",
		.owner = THIS_MODULE,
	},
	.probe = max96717_probe,
	.remove = max96717_remove,
	.id_table = max96717_id,
};

static int __init max96717_init(void)
{
	return i2c_add_driver(&max96717_i2c_driver);
}

static void __exit max96717_exit(void)
{
	i2c_del_driver(&max96717_i2c_driver);
}

module_init(max96717_init);
module_exit(max96717_exit);

MODULE_DESCRIPTION("GMSL Serializer driver max96717");
MODULE_AUTHOR("Software Supreme>");
MODULE_LICENSE("GPL v2");
