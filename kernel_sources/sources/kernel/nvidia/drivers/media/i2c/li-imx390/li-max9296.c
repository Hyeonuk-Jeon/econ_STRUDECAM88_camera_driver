/*
 * max9296.c - max9296 GMSL Deserializer driver
 *
 * Copyright (c) 2018-2022, Leopard Imaging Inc.  All rights reserved.
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

#include <linux/gpio.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <media/camera_common.h>
#include <linux/module.h>
#include <media/max9296.h>

/* register specifics */
#define MAX9296_DST_CSI_MODE_ADDR 0x330
#define MAX9296_LANE_MAP1_ADDR 0x333
#define MAX9296_LANE_MAP2_ADDR 0x334

#define MAX9296_LANE_CTRL0_ADDR 0x40A
#define MAX9296_LANE_CTRL1_ADDR 0x44A
#define MAX9296_LANE_CTRL2_ADDR 0x48A
#define MAX9296_LANE_CTRL3_ADDR 0x4CA

#define MAX9296_TX11_PIPE_X_EN_ADDR 0x40B
#define MAX9296_TX45_PIPE_X_DST_CTRL_ADDR 0x42D

#define MAX9296_PIPE_X_SRC_0_MAP_ADDR 0x40D
#define MAX9296_PIPE_X_DST_0_MAP_ADDR 0x40E
#define MAX9296_PIPE_X_SRC_1_MAP_ADDR 0x40F
#define MAX9296_PIPE_X_DST_1_MAP_ADDR 0x410
#define MAX9296_PIPE_X_SRC_2_MAP_ADDR 0x411
#define MAX9296_PIPE_X_DST_2_MAP_ADDR 0x412

#define MAX9296_PIPE_X_ST_SEL_ADDR 0x50

#define MAX9296_PWDN_PHYS_ADDR 0x332
#define MAX9296_PHY1_CLK_ADDR 0x320
#define MAX9296_CTRL0_ADDR 0x10

/* data defines */
#define MAX9296_CSI_MODE_4X2 0x1
#define MAX9296_CSI_MODE_2X4 0x4
#define MAX9296_LANE_MAP1_4X2 0x44
#define MAX9296_LANE_MAP2_4X2 0x44
#define MAX9296_LANE_MAP1_2X4 0x4E
#define MAX9296_LANE_MAP2_2X4 0xE4

#define MAX9296_LANE_CTRL_MAP(num_lanes) \
	(((num_lanes) << 6) & 0xF0)

#define MAX9296_ALLPHYS_NOSTDBY 0xF0
#define MAX9296_ST_ID_SEL_INVALID 0xF

#define MAX9296_PHY1_CLK 0x2C

#define MAX9296_RESET_ALL 0x80

/* Dual GMSL MAX9296A/B */
#define MAX9296_MAX_SOURCES 2

#define MAX9296_MAX_PIPES 4

#define MAX9296_PIPE_X 0
#define MAX9296_PIPE_Y 1
#define MAX9296_PIPE_Z 2
#define MAX9296_PIPE_U 3
#define MAX9296_PIPE_INVALID 0xF


#define MAX9296_CSI_CTRL_0 0
#define MAX9296_CSI_CTRL_1 1
#define MAX9296_CSI_CTRL_2 2
#define MAX9296_CSI_CTRL_3 3

#define MAX9296_INVAL_ST_ID 0xFF

/* Use reset value as per spec, confirm with vendor */
#define MAX9296_RESET_ST_ID 0x00

struct li_max9296_source_ctx {
	struct gmsl_link_ctx *g_ctx;
	bool st_enabled;
};

struct pipe_ctx {
	u32 id;
	u32 dt_type;
	u32 dst_csi_ctrl;
	u32 st_count;
	u32 st_id_sel;
};

struct max9296 {
	struct i2c_client *i2c_client;
	struct regmap *regmap;
	u32 num_src;
	u32 max_src;
	u32 num_src_found;
	u32 src_link;
	bool splitter_enabled;
	struct li_max9296_source_ctx sources[MAX9296_MAX_SOURCES];
	struct mutex lock;
	u32 sdev_ref;
	bool lane_setup;
	bool link_setup;
	struct pipe_ctx pipe[MAX9296_MAX_PIPES];
	u8 csi_mode;
	u8 lane_mp1;
	u8 lane_mp2;
	int reset_gpio;
	int pw_ref;
	struct regulator *vdd_cam_1v2;
};


static void gpio_set_output_value(unsigned int gpio,int val){
	if(gpio_cansleep(gpio)){
		gpio_direction_output(gpio,val);
		gpio_set_value_cansleep(gpio,val);
	}else{
		gpio_direction_output(gpio,val);
		gpio_set_value(gpio,val);
	}
}

static int li_max9296_write_reg(struct device *dev,
	u16 addr, u8 val)
{
	struct max9296 *priv;
	int err;

	priv = dev_get_drvdata(dev);

	err = regmap_write(priv->regmap, addr, val);
	if (err)
		dev_err(dev,
		"%s:i2c write failed, 0x%x = %x\n",
		__func__, addr, val);

	/* delay before next i2c command as required for SERDES link */
	usleep_range(100, 110);

	return err;
}

static int li_max9296_read_reg(struct device *dev,
	u16 addr, u8* val)
{
	struct max9296 *priv;
	int err;
	u32 reg_val;
	priv = dev_get_drvdata(dev);
	reg_val = 0;

	err = regmap_read(priv->regmap, addr, &reg_val);
	*val = reg_val & 0xff;
	//if (err)
	//	dev_err(dev,
	//	"%s:i2c write failed, 0x%x = %x\n",
	//	__func__, addr, val);

	/* delay before next i2c command as required for SERDES link */
	usleep_range(100, 110);

	return err;
}
static int li_max9296_get_sdev_idx(struct device *dev,
			struct device *s_dev, unsigned int *idx)
{
	struct max9296 *priv = dev_get_drvdata(dev);
	unsigned int i;
	int err = 0;

	mutex_lock(&priv->lock);
	for (i = 0; i < priv->max_src; i++) {
		if (priv->sources[i].g_ctx->s_dev == s_dev)
			break;
	}
	if (i == priv->max_src) {
		dev_err(dev, "no sdev found\n");
		err = -EINVAL;
		goto ret;
	}

	if (idx)
		*idx = i;

ret:
	mutex_unlock(&priv->lock);
	return err;
}

static void li_max9296_pipes_reset(struct max9296 *priv)
{
	/*
	 * This is default pipes combination. add more mappings
	 * for other combinations and requirements.
	 */

	struct pipe_ctx pipe_defaults[] = {
		{MAX9296_PIPE_X, GMSL_CSI_DT_RAW_12,
			MAX9296_CSI_CTRL_1, 0, MAX9296_INVAL_ST_ID},
		{MAX9296_PIPE_Y, GMSL_CSI_DT_RAW_12,
			MAX9296_CSI_CTRL_1, 0, MAX9296_INVAL_ST_ID},
		{MAX9296_PIPE_Z, GMSL_CSI_DT_EMBED,
			MAX9296_CSI_CTRL_1, 0, MAX9296_INVAL_ST_ID},
		{MAX9296_PIPE_U, GMSL_CSI_DT_EMBED,
			MAX9296_CSI_CTRL_1, 0, MAX9296_INVAL_ST_ID}
	};

	/*
	 * Add DT props for num-streams and stream sequence, and based on that
	 * set the appropriate pipes defaults.
	 * For now default it supports "2 RAW12 and 2 EMBED" 1:1 mappings.
	 */
	memcpy(priv->pipe, pipe_defaults, sizeof(pipe_defaults));
}

static void li_max9296_reset_ctx(struct max9296 *priv)
{
	unsigned int i;

	priv->link_setup = false;
	priv->lane_setup = false;
	priv->num_src_found = 0;
	priv->src_link = 0;
	priv->splitter_enabled = false;
	li_max9296_pipes_reset(priv);
	for (i = 0; i < priv->num_src; i++)
		priv->sources[i].st_enabled = false;
}

int li_max9296_power_on(struct device *dev)
{
	struct max9296 *priv = dev_get_drvdata(dev);
	int err = 0;

	mutex_lock(&priv->lock);
	if (priv->pw_ref == 0) {
		usleep_range(1, 2);
		if (priv->reset_gpio)
			gpio_set_output_value(priv->reset_gpio, 0);

		usleep_range(30, 50);

		if (priv->vdd_cam_1v2) {
			err = regulator_enable(priv->vdd_cam_1v2);
			if (unlikely(err))
				goto ret;
		}

		usleep_range(30, 50);

		/*exit reset mode: XCLR */
		if (priv->reset_gpio) {
			gpio_set_output_value(priv->reset_gpio, 0);
			usleep_range(30, 50);
			gpio_set_output_value(priv->reset_gpio, 1);
			usleep_range(30, 50);
		}

		/* delay to settle reset */
		msleep(20);
	}

	priv->pw_ref++;

ret:
	mutex_unlock(&priv->lock);

	return err;
}
EXPORT_SYMBOL(li_max9296_power_on);

void li_max9296_power_off(struct device *dev)
{
	struct max9296 *priv = dev_get_drvdata(dev);

	mutex_lock(&priv->lock);
	priv->pw_ref--;

	if (priv->pw_ref == 0) {
		/* enter reset mode: XCLR */
		usleep_range(1, 2);
		if (priv->reset_gpio)
			gpio_set_output_value(priv->reset_gpio, 0);

		if (priv->vdd_cam_1v2)
			regulator_disable(priv->vdd_cam_1v2);
	}

	mutex_unlock(&priv->lock);
}
EXPORT_SYMBOL(li_max9296_power_off);

static int li_max9296_write_link(struct device *dev, u32 link)
{

	if (link == GMSL_SERDES_CSI_LINK_A) {
		li_max9296_write_reg(dev, MAX9296_CTRL0_ADDR, 0x01);
		li_max9296_write_reg(dev, MAX9296_CTRL0_ADDR, 0x21);
	} else if (link == GMSL_SERDES_CSI_LINK_B) {
		li_max9296_write_reg(dev, MAX9296_CTRL0_ADDR, 0x02);
		li_max9296_write_reg(dev, MAX9296_CTRL0_ADDR, 0x22);
	} else {
		dev_err(dev, "%s: invalid gmsl link\n", __func__);
		return -EINVAL;
	}

	/* delay to settle link */
	msleep(300);

	return 0;
}

int li_max9296_setup_link(struct device *dev, struct device *s_dev)
{
	struct max9296 *priv = dev_get_drvdata(dev);
	int err = 0;
	unsigned int i = 0;

	err = li_max9296_get_sdev_idx(dev, s_dev, &i);
	if (err)
		return err;

	mutex_lock(&priv->lock);

	if (!priv->splitter_enabled) {
		err = li_max9296_write_link(dev,
				priv->sources[i].g_ctx->serdes_csi_link);
		if (err)
			goto ret;

		priv->link_setup = true;
	}

ret:
	mutex_unlock(&priv->lock);

	return err;
}
EXPORT_SYMBOL(li_max9296_setup_link);


static int li_max9296_link_locked(struct device* dev){
	u8 val;
	usleep_range(100,110);
	li_max9296_read_reg(dev,0x0013,&val);

	if(0 == (val & 0x08))
		return -EINVAL;
	return 0;
}

int li_max9296_setup_control(struct device *dev, struct device *s_dev)
{
	struct max9296 *priv = dev_get_drvdata(dev);
	int err = 0;
	unsigned int i = 0;

	err = li_max9296_get_sdev_idx(dev, s_dev, &i);
	if (err)
		return err;

	mutex_lock(&priv->lock);

	if (!priv->link_setup) {
		dev_err(dev, "%s: invalid state\n", __func__);
		err = -EINVAL;
		goto error;
	}

	if (priv->sources[i].g_ctx->serdev_found) {
		priv->num_src_found++;
		priv->src_link = priv->sources[i].g_ctx->serdes_csi_link;
	}

	/* Enable splitter mode */
	if ((priv->max_src > 1U) &&
		(priv->num_src_found > 0U) &&
		(priv->splitter_enabled == false)) {
		li_max9296_write_reg(dev, MAX9296_CTRL0_ADDR, 0x03);
		li_max9296_write_reg(dev, MAX9296_CTRL0_ADDR, 0x23);

		priv->splitter_enabled = true;

		/* delay to settle link */
		msleep(100);
	}

	err = li_max9296_link_locked(dev);
	if(err){
		err = li_max9296_write_link(dev,priv->sources[i].g_ctx->serdes_csi_link);
		if(err){
			dev_err(dev, "[%s]: write link failed.\n",__func__);
		}
	}

	li_max9296_write_reg(dev,
			MAX9296_PWDN_PHYS_ADDR, MAX9296_ALLPHYS_NOSTDBY);

	priv->sdev_ref++;

	/* Reset splitter mode if all devices are not found */
	if ((priv->sdev_ref == priv->max_src) &&
		(priv->splitter_enabled == true) &&
		(priv->num_src_found > 0U) &&
		(priv->num_src_found < priv->max_src)) {
		err = li_max9296_write_link(dev, priv->src_link);
		if (err)
			goto error;

		priv->splitter_enabled = false;
	}

error:
	mutex_unlock(&priv->lock);
	return err;
}
EXPORT_SYMBOL(li_max9296_setup_control);

int li_max9296_reset_control(struct device *dev, struct device *s_dev)
{
	struct max9296 *priv = dev_get_drvdata(dev);
	int err = 0;

	mutex_lock(&priv->lock);
	if (!priv->sdev_ref) {
		dev_info(dev, "%s: dev is already in reset state\n", __func__);
		goto ret;
	}

	priv->sdev_ref--;
	if (priv->sdev_ref == 0) {
		li_max9296_reset_ctx(priv);
		li_max9296_write_reg(dev, MAX9296_CTRL0_ADDR, MAX9296_RESET_ALL);

		/* delay to settle reset */
		msleep(100);
	}

ret:
	mutex_unlock(&priv->lock);

	return err;
}
EXPORT_SYMBOL(li_max9296_reset_control);

int li_max9296_sdev_register(struct device *dev, struct gmsl_link_ctx *g_ctx)
{
	struct max9296 *priv = NULL;
	unsigned int i;
	int err = 0;

	if (!dev || !g_ctx || !g_ctx->s_dev) {
		dev_err(dev, "%s: invalid input params\n", __func__);
		return -EINVAL;
	}

	priv = dev_get_drvdata(dev);

	mutex_lock(&priv->lock);

	if (priv->num_src > priv->max_src) {
		dev_err(dev,
			"%s: MAX9296 inputs size exhausted\n", __func__);
		err = -ENOMEM;
		goto error;
	}

	/* Check csi mode compatibility */
	if (!((priv->csi_mode == MAX9296_CSI_MODE_2X4) ?
			((g_ctx->csi_mode == GMSL_CSI_1X4_MODE) ||
				(g_ctx->csi_mode == GMSL_CSI_2X4_MODE)) :
			((g_ctx->csi_mode == GMSL_CSI_2X2_MODE) ||
				(g_ctx->csi_mode == GMSL_CSI_4X2_MODE)))) {
		dev_err(dev,
			"%s: csi mode not supported\n", __func__);
		err = -EINVAL;
		goto error;
	}
	dev_info(dev,"Serdes CSI link of serializer: %2x\n",g_ctx->serdes_csi_link);
	for (i = 0; i < priv->num_src; i++) {
		if (g_ctx->serdes_csi_link ==
			priv->sources[i].g_ctx->serdes_csi_link) {
			//dev_err(dev,
			//	"%s: serdes csi link is in use\n", __func__);
			//err = -EINVAL;
			//goto error;
			break;
		}
		/*
		 * All sdevs should have same num-csi-lanes regardless of
		 * dst csi port selected.
		 * Later if there is any usecase which requires each port
		 * to be configured with different num-csi-lanes, then this
		 * check should be performed per port.
		 */
		if (g_ctx->num_csi_lanes !=
				priv->sources[i].g_ctx->num_csi_lanes) {
			dev_err(dev,
				"%s: csi num lanes mismatch\n", __func__);
			err = -EINVAL;
			goto error;
		}
	}

	priv->sources[priv->num_src].g_ctx = g_ctx;
	priv->sources[priv->num_src].st_enabled = false;

	priv->num_src++;

error:
	mutex_unlock(&priv->lock);
	return err;
}
EXPORT_SYMBOL(li_max9296_sdev_register);

int li_max9296_sdev_unregister(struct device *dev, struct device *s_dev)
{
	struct max9296 *priv = NULL;
	int err = 0;
	unsigned int i = 0;

	if (!dev || !s_dev) {
		dev_err(dev, "%s: invalid input params\n", __func__);
		return -EINVAL;
	}

	priv = dev_get_drvdata(dev);
	mutex_lock(&priv->lock);

	if (priv->num_src == 0) {
		dev_err(dev, "%s: no source found\n", __func__);
		err = -ENODATA;
		goto error;
	}

	for (i = 0; i < priv->num_src; i++) {
		if (s_dev == priv->sources[i].g_ctx->s_dev) {
			priv->sources[i].g_ctx = NULL;
			break;
		}
	}

	if (i == priv->num_src) {
		dev_err(dev,
			"%s: requested device not found\n", __func__);
		err = -EINVAL;
		goto error;
	}
	priv->num_src--;

error:
	mutex_unlock(&priv->lock);
	return err;
}
EXPORT_SYMBOL(li_max9296_sdev_unregister);

static int li_max9296_get_available_pipe(struct device *dev,
				u32 st_data_type, u32 dst_csi_port)
{
	struct max9296 *priv = dev_get_drvdata(dev);
	int i;

	for (i = 0; i < MAX9296_MAX_PIPES; i++) {
		/*
		 * TODO: Enable a pipe for multi stream configuration having
		 * similar stream data type. For now use st_count as a flag
		 * for 1 to 1 mapping in pipe and stream data type, same can
		 * be extended as count for many to 1 mapping. Would also need
		 * few more checks such as input stream id select, dst port etc.
		 */
		dev_dbg(dev, "%s: Pipe ID: %d, Dest CSI PORT: %d, Dest CSI Controller: %d, dt_type: %d, st_dt_type: %d, st_count: %d\n", __func__, i, dst_csi_port, priv->pipe[i].dst_csi_ctrl, priv->pipe[i].dt_type, st_data_type, priv->pipe[i].st_count);
		
		if(!priv->pipe[i].st_count)
		{
			if ((priv->pipe[i].dt_type == st_data_type) &&
				((dst_csi_port == GMSL_CSI_PORT_A) ?
					(priv->pipe[i].dst_csi_ctrl == MAX9296_CSI_CTRL_0) ||
					(priv->pipe[i].dst_csi_ctrl == MAX9296_CSI_CTRL_1) :
					(priv->pipe[i].dst_csi_ctrl == MAX9296_CSI_CTRL_2) ||
					(priv->pipe[i].dst_csi_ctrl == MAX9296_CSI_CTRL_3))
				)
			{
				break;
			} else if ((priv->pipe[i].dt_type == st_data_type) &&
					(dst_csi_port == GMSL_CSI_PORT_B))
			{
				priv->pipe[i].dst_csi_ctrl = MAX9296_CSI_CTRL_2;
				break;
			}
		}

	}

	if (i == MAX9296_MAX_PIPES) {
		dev_err(dev, "%s: all pipes are busy\n", __func__);
		return -ENOMEM;
	}

	return i;
}

struct reg_pair {
	u16 addr;
	u8 val;
};

static int li_max9296_setup_pipeline(struct device *dev,
		struct gmsl_link_ctx *g_ctx)
{
	struct max9296 *priv = dev_get_drvdata(dev);
	struct gmsl_stream *g_stream;
	struct reg_pair *map_list;
	u32 arr_sz = 0;
	int pipe_id = 0;
	u32 i = 0;
	u32 j = 0;
	u32 vc_idx = 0;

	dev_dbg(dev, "%s: Num_Streams: %d\n", __func__, g_ctx->num_streams);
	for (i = 0; i < g_ctx->num_streams; i++) {
		/* Base data type mapping: pipeX/RAW12/CSICNTR1 */
		struct reg_pair map_pipe_raw12[] = {
			/* addr, val */
			{MAX9296_TX11_PIPE_X_EN_ADDR, 0x7},
			{MAX9296_TX45_PIPE_X_DST_CTRL_ADDR, 0x15},
			{MAX9296_PIPE_X_SRC_0_MAP_ADDR, 0x2C},
			{MAX9296_PIPE_X_DST_0_MAP_ADDR, 0x2C},
			{MAX9296_PIPE_X_SRC_1_MAP_ADDR, 0x00},
			{MAX9296_PIPE_X_DST_1_MAP_ADDR, 0x00},
			{MAX9296_PIPE_X_SRC_2_MAP_ADDR, 0x01},
			{MAX9296_PIPE_X_DST_2_MAP_ADDR, 0x01},
		};

		/* Base data type mapping: pipeX/EMBED/CSICNTR1 */
		struct reg_pair map_pipe_embed[] = {
			/* addr, val */
			{MAX9296_TX11_PIPE_X_EN_ADDR, 0x7},
			{MAX9296_TX45_PIPE_X_DST_CTRL_ADDR, 0x15},
			{MAX9296_PIPE_X_SRC_0_MAP_ADDR, 0x12},
			{MAX9296_PIPE_X_DST_0_MAP_ADDR, 0x12},
			{MAX9296_PIPE_X_SRC_1_MAP_ADDR, 0x00},
			{MAX9296_PIPE_X_DST_1_MAP_ADDR, 0x00},
			{MAX9296_PIPE_X_SRC_2_MAP_ADDR, 0x01},
			{MAX9296_PIPE_X_DST_2_MAP_ADDR, 0x01},
		};

		g_stream = &g_ctx->streams[i];
		g_stream->des_pipe = MAX9296_PIPE_INVALID;

		if (g_stream->st_data_type == GMSL_CSI_DT_RAW_12) {
			map_list = map_pipe_raw12;
			arr_sz = ARRAY_SIZE(map_pipe_raw12);
		} else if (g_stream->st_data_type == GMSL_CSI_DT_EMBED) {
			map_list = map_pipe_embed;
			arr_sz = ARRAY_SIZE(map_pipe_embed);
		} else if (g_stream->st_data_type == GMSL_CSI_DT_UED_U1) {
			dev_dbg(dev,
				"%s: No mapping for GMSL_CSI_DT_UED_U1\n",
				__func__);
			continue;
		} else {
			dev_err(dev, "%s: Invalid data type\n", __func__);
			return -EINVAL;
		}

		pipe_id = li_max9296_get_available_pipe(dev,
				g_stream->st_data_type, g_ctx->dst_csi_port);
		if (pipe_id < 0)
			return pipe_id;
		
		dev_dbg(dev, "%s: Pipe ID: %d\n", __func__, pipe_id);

		// By default, the dst_csi_port is assumed to be GMSL_CSI_PORT_A
		// To route a pipe to GMSL_CSI_PORT_B,
		// Update the value for the MAX9296_PIPE_X_DST_0_MAP_ADDR register
		// The register address will be later changed to represent PIPE_Y or Z or U
		if(g_ctx->dst_csi_port == GMSL_CSI_PORT_B)
		{
			map_list[1].val = 0x2A; // 0x2A means PHY2 (2nd CSI Controller) as per the datasheet
		}

		for (j = 0, vc_idx = 3; j < arr_sz; j++, vc_idx += 2) {
			/* update pipe configuration */
			map_list[j].addr += (0x40 * pipe_id);
			/* update vc id configuration */
			if (vc_idx < arr_sz)
				map_list[vc_idx].val |=
					(g_ctx->dst_vc << 6);

			dev_dbg(dev, "%s: Addr: 0x%X, Val: 0x%X\n", __func__, map_list[j].addr, map_list[j].val);

			li_max9296_write_reg(dev, map_list[j].addr,
						map_list[j].val);
		}

		/* Set stream id select input */
		if (g_stream->st_id_sel == GMSL_ST_ID_UNUSED) {
			dev_err(dev, "%s: Invalid stream st_id_sel\n",
				__func__);
			return -EINVAL;
		}

		g_stream->des_pipe = MAX9296_PIPE_X_ST_SEL_ADDR + pipe_id;

		/* Update pipe internals */
		priv->pipe[pipe_id].st_count++;
		priv->pipe[pipe_id].st_id_sel = g_stream->st_id_sel;
	}

	return 0;
}

int li_max9296_start_streaming(struct device *dev, struct device *s_dev)
{
	struct max9296 *priv = dev_get_drvdata(dev);
	struct gmsl_link_ctx *g_ctx;
	struct gmsl_stream *g_stream;
	int err = 0;
	unsigned int i = 0;

	err = li_max9296_get_sdev_idx(dev, s_dev, &i);
	if (err)
		return err;

	mutex_lock(&priv->lock);
	g_ctx = priv->sources[i].g_ctx;

	for (i = 0; i < g_ctx->num_streams; i++) {
		g_stream = &g_ctx->streams[i];
		dev_dbg(dev, "%s: Destination Pipe: %d, Stream ID: %d\n", __func__, g_stream->des_pipe, g_stream->st_id_sel);
		if (g_stream->des_pipe != MAX9296_PIPE_INVALID)
			li_max9296_write_reg(dev, g_stream->des_pipe,
						g_stream->st_id_sel);
	}
	mutex_unlock(&priv->lock);

	return 0;
}
EXPORT_SYMBOL(li_max9296_start_streaming);

int li_max9296_stop_streaming(struct device *dev, struct device *s_dev)
{
	struct max9296 *priv = dev_get_drvdata(dev);
	struct gmsl_link_ctx *g_ctx;
	struct gmsl_stream *g_stream;
	int err = 0;
	unsigned int i = 0;

	err = li_max9296_get_sdev_idx(dev, s_dev, &i);
	if (err)
		return err;

	mutex_lock(&priv->lock);
	g_ctx = priv->sources[i].g_ctx;

	for (i = 0; i < g_ctx->num_streams; i++) {
		g_stream = &g_ctx->streams[i];

		if (g_stream->des_pipe != MAX9296_PIPE_INVALID)
			li_max9296_write_reg(dev, g_stream->des_pipe,
						MAX9296_RESET_ST_ID);
	}

	mutex_unlock(&priv->lock);

	return 0;
}
EXPORT_SYMBOL(li_max9296_stop_streaming);

int li_max9296_setup_streaming(struct device *dev, struct device *s_dev)
{
	struct max9296 *priv = dev_get_drvdata(dev);
	struct gmsl_link_ctx *g_ctx;
	int err = 0;
	unsigned int i = 0;
	u16 lane_ctrl_addr;

	err = li_max9296_get_sdev_idx(dev, s_dev, &i);
	if (err)
		return err;

	mutex_lock(&priv->lock);
	if (priv->sources[i].st_enabled)
		goto ret;

	g_ctx = priv->sources[i].g_ctx;

	err = li_max9296_setup_pipeline(dev, g_ctx);
	if (err)
		goto ret;

	/* Derive CSI lane map register */
	switch(g_ctx->dst_csi_port) {
	case GMSL_CSI_PORT_A:
	case GMSL_CSI_PORT_D:
		lane_ctrl_addr = MAX9296_LANE_CTRL1_ADDR;
		break;
	case GMSL_CSI_PORT_B:
	case GMSL_CSI_PORT_E:
		lane_ctrl_addr = MAX9296_LANE_CTRL2_ADDR;
		break;
	case GMSL_CSI_PORT_C:
		lane_ctrl_addr = MAX9296_LANE_CTRL0_ADDR;
		break;
	case GMSL_CSI_PORT_F:
		lane_ctrl_addr = MAX9296_LANE_CTRL3_ADDR;
		break;
	default:
		dev_err(dev, "%s: invalid gmsl csi port!\n", __func__);
		err = -EINVAL;
		goto ret;
	};

	/*
	 * rewrite num_lanes to same dst port should not be an issue,
	 * as the device compatibility is already
	 * checked during sdev registration against the des properties.
	 */
	li_max9296_write_reg(dev, lane_ctrl_addr,
		MAX9296_LANE_CTRL_MAP(g_ctx->num_csi_lanes-1));

	if (!priv->lane_setup) {
		li_max9296_write_reg(dev,
			MAX9296_DST_CSI_MODE_ADDR, priv->csi_mode);
		li_max9296_write_reg(dev,
			MAX9296_LANE_MAP1_ADDR, priv->lane_mp1);
		li_max9296_write_reg(dev,
			MAX9296_LANE_MAP2_ADDR, priv->lane_mp2);
		li_max9296_write_reg(dev,
			MAX9296_PHY1_CLK_ADDR, MAX9296_PHY1_CLK);

		priv->lane_setup = true;
	}
	//li_max9296_write_reg(dev, 0x03EF, 0xC0);
	//li_max9296_write_reg(dev, 0x03E2, 0x00);
	//msleep(10);
	//li_max9296_write_reg(dev, 0x03EA, 0x00);
	//li_max9296_write_reg(dev, 0x03EB, 0x00);
	//li_max9296_write_reg(dev, 0x03E5, 0x35);
	//li_max9296_write_reg(dev, 0x03E6, 0xB7);
	//li_max9296_write_reg(dev, 0x03E7, 0x0c);
	//li_max9296_write_reg(dev, 0x03F1, 0x40);
	//li_max9296_write_reg(dev, 0x03E0, 0x04);

	priv->sources[i].st_enabled = true;

ret:
	mutex_unlock(&priv->lock);
	return err;
}
EXPORT_SYMBOL(li_max9296_setup_streaming);

static const struct of_device_id li_max9296_of_match[] = {
	{ .compatible = "li,li_max9296", },
	{ },
};
MODULE_DEVICE_TABLE(of, li_max9296_of_match);

static int li_max9296_parse_dt(struct max9296 *priv,
				struct i2c_client *client)
{
	struct device_node *node = client->dev.of_node;
	int err = 0;
	const char *str_value;
	int value;
	const struct of_device_id *match;

	if (!node)
		return -EINVAL;

	match = of_match_device(li_max9296_of_match, &client->dev);
	if (!match) {
		dev_err(&client->dev, "Failed to find matching dt id\n");
		return -EFAULT;
	}

	err = of_property_read_string(node, "csi-mode", &str_value);
	if (err < 0) {
		dev_err(&client->dev, "csi-mode property not found\n");
		return err;
	}

	if (!strcmp(str_value, "2x4")) {
		priv->csi_mode = MAX9296_CSI_MODE_2X4;
		priv->lane_mp1 = MAX9296_LANE_MAP1_2X4;
		priv->lane_mp2 = MAX9296_LANE_MAP2_2X4;
	} else if (!strcmp(str_value, "4x2")) {
		priv->csi_mode = MAX9296_CSI_MODE_4X2;
		priv->lane_mp1 = MAX9296_LANE_MAP1_4X2;
		priv->lane_mp2 = MAX9296_LANE_MAP2_4X2;
	} else {
		dev_err(&client->dev, "invalid csi mode\n");
		return -EINVAL;
	}

	err = of_property_read_u32(node, "max-src", &value);
	if (err < 0) {
		dev_err(&client->dev, "No max-src info\n");
		return err;
	}
	priv->max_src = value;

	priv->reset_gpio = of_get_named_gpio(node, "reset-gpios", 0);
	if (priv->reset_gpio < 0) {
		dev_err(&client->dev, "reset-gpios not found %d\n", err);
		return err;
	}

	/* digital 1.2v */
	if (of_get_property(node, "vdd_cam_1v2-supply", NULL)) {
		priv->vdd_cam_1v2 = regulator_get(&client->dev, "vdd_cam_1v2");
		if (IS_ERR(priv->vdd_cam_1v2)) {
			dev_err(&client->dev,
				"vdd_cam_1v2 regulator get failed\n");
			err = PTR_ERR(priv->vdd_cam_1v2);
			priv->vdd_cam_1v2 = NULL;
			return err;
		}
	} else {
		priv->vdd_cam_1v2 = NULL;
	}

	return 0;
}

static struct regmap_config li_max9296_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
	.cache_type = REGCACHE_RBTREE,
};

static int li_max9296_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	struct max9296 *priv;
	int err = 0;

	dev_info(&client->dev, "[li-MAX9296]: probing GMSL Deserializer\n");

	priv = devm_kzalloc(&client->dev, sizeof(*priv), GFP_KERNEL);
	priv->i2c_client = client;
	priv->regmap = devm_regmap_init_i2c(priv->i2c_client,
				&li_max9296_regmap_config);
	if (IS_ERR(priv->regmap)) {
		dev_err(&client->dev,
			"regmap init failed: %ld\n", PTR_ERR(priv->regmap));
		return -ENODEV;
	}

	err = li_max9296_parse_dt(priv, client);
	if (err) {
		dev_err(&client->dev, "unable to parse dt\n");
		return -EFAULT;
	}

	li_max9296_pipes_reset(priv);

	if (priv->max_src > MAX9296_MAX_SOURCES) {
		dev_err(&client->dev,
			"max sources more than currently supported\n");
		return -EINVAL;
	}

	mutex_init(&priv->lock);

	dev_set_drvdata(&client->dev, priv);

	if(priv->max_src > 1U){
		li_max9296_write_reg(&client->dev, MAX9296_CTRL0_ADDR, 0x03);
		li_max9296_write_reg(&client->dev, MAX9296_CTRL0_ADDR, 0x23);
		msleep(500);
		err = li_max9296_link_locked(&client->dev);
		
	}		
	if(err){
		li_max9296_write_reg(&client->dev, MAX9296_CTRL0_ADDR,0x01);
		li_max9296_write_reg(&client->dev, MAX9296_CTRL0_ADDR,0x21);
		msleep(500);
		err = li_max9296_link_locked(&client->dev);
		if(err){
			li_max9296_write_reg(&client->dev, MAX9296_CTRL0_ADDR,0x02);
			li_max9296_write_reg(&client->dev, MAX9296_CTRL0_ADDR,0x22);
		}
	}
	err = 0;

	/* dev communication gets validated when GMSL link setup is done */
	dev_info(&client->dev, "%s:  success\n", __func__);

	return err;
}


static int li_max9296_remove(struct i2c_client *client)
{
	struct max9296 *priv;

	if (client != NULL) {
		priv = dev_get_drvdata(&client->dev);
		mutex_destroy(&priv->lock);
		//i2c_unregister_device(client);
		client = NULL;
	}

	return 0;
}

static const struct i2c_device_id li_max9296_id[] = {
	{ "li_max9296", 0 },
	{ },
};

MODULE_DEVICE_TABLE(i2c, li_max9296_id);

static struct i2c_driver li_max9296_i2c_driver = {
	.driver = {
		.name = "li_max9296",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(li_max9296_of_match),
	},
	.probe = li_max9296_probe,
	.remove = li_max9296_remove,
	.id_table = li_max9296_id,
};

static int __init li_max9296_init(void)
{
	return i2c_add_driver(&li_max9296_i2c_driver);
}

static void __exit li_max9296_exit(void)
{
	i2c_del_driver(&li_max9296_i2c_driver);
}

module_init(li_max9296_init);
module_exit(li_max9296_exit);

MODULE_DESCRIPTION("Dual GMSL Deserializer driver max9296");
MODULE_AUTHOR("Leopard Imaging Inc.");
MODULE_LICENSE("GPL v2");
