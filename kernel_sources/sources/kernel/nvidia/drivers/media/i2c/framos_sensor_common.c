/*
 * framos_sensor_common.c - common logic
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
 
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/gpio.h>
#include <linux/module.h>
#include <linux/seq_file.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>

#include <media/tegra_v4l2_camera.h>
#include <media/tegracam_core.h>

#include "media/framos_sensor_common.h"
#include "i2c/framos_i2c_generic.h"

void fm_get_board_model(struct tegracam_device *tc_dev, u8 *som_board_model)
{
	struct device *dev = tc_dev->dev;
	struct device_node *root;
    const char *model;

	root = of_find_node_by_path("/");
	if (of_property_read_string(root, "model", &model)) {
		dev_err(dev, "Cannot read board model\n");
    }
	of_node_put(root);

    if(!strcmp(model, "NVIDIA Jetson Nano Developer Kit")){
        *som_board_model = JETSON_NANO_DEV_KIT;
    }
    else if(!strcmp(model, "quill-3310")){
        *som_board_model = JETSON_TX2;
    }
    else if(!strcmp(model, "Jetson-AGX")){
        *som_board_model = JETSON_XAVIER;
    }
    else if(!strcmp(model, "NVIDIA Jetson Xavier NX Developer Kit")){
        *som_board_model = JETSON_XAVIER_NX_DEV_KIT;
    }
	else if(!strcmp(model, "lanai-3636")){
        *som_board_model = JETSON_TX2_NX;
    }

    pr_info("MODEL: %u\n", *som_board_model);
    return;
}
EXPORT_SYMBOL(fm_get_board_model);

void fm_calc_lshift(u64 *val, u8 *lshift, s64 max){
    *lshift = 1;
    *val = *val >> *lshift;

    while (*val > max) {
        (*lshift)++;
        *val = *val >> *lshift;
    }

    return;
}
EXPORT_SYMBOL(fm_calc_lshift);

void fm_get_gpio_ctrl(struct camera_common_pdata *board_priv_pdata) {
    struct device_node *node = NULL;

    node = of_find_node_by_name(NULL, "framos_platform_adapter");
    if(node) {
        board_priv_pdata->use_cam_gpio = of_property_read_bool(node, "cam,use-cam-gpio"); 
        of_node_put(node);
    }
    else {
        board_priv_pdata->use_cam_gpio = 0;
    }

    return;
}
EXPORT_SYMBOL(fm_get_gpio_ctrl);

void fm_gpio_set(struct camera_common_data *s_data,
			    unsigned int gpio, int val)
{
	struct camera_common_pdata *pdata = s_data->pdata;

	if (pdata && pdata->use_cam_gpio)
		cam_gpio_ctrl(s_data->dev, gpio, val, 1);
	else {
		if (gpio_cansleep(gpio))
			gpio_set_value_cansleep(gpio, val);
		else
			gpio_set_value(gpio, val);
	}
}
EXPORT_SYMBOL(fm_gpio_set);

s64 fm_get_v4l2_ctrl_value(struct tegracam_device *tc_dev,
                                    int ctrl_id, v4l2_ctrl_value ctrl_value)
{
    struct device *dev = tc_dev->dev;
    struct v4l2_ctrl *ctrl;
    s64 value;

       ctrl = fm_find_v4l2_ctrl(tc_dev, ctrl_id);
       if (ctrl) {
               switch (ctrl_value) {
                   case MIN:
                           value = ctrl->minimum;
                       break;
               case MAX:
                           value = ctrl->maximum;
                       break;
               case DEFAULT:
                       value = ctrl->default_value;
                       break;
               case CURRENT:
                       value = *ctrl->p_cur.p_s64;
                       break;
               case NEW:
                       value = *ctrl->p_new.p_s64;
                       break;
               default:
                dev_err(dev, "%s: unknown v4l2 control value\n", __func__);
                       return -EINVAL;
               }
       return value;
       }

       dev_err(dev, "%s: v4l2 control does not exist in list\n", __func__);
    return -EINVAL;
}
EXPORT_SYMBOL(fm_get_v4l2_ctrl_value);


struct v4l2_ctrl *fm_find_v4l2_ctrl(struct tegracam_device *tc_dev, 
                                                int ctrl_id)
{
	struct camera_common_data *s_data = tc_dev->s_data;
	struct device *dev = tc_dev->dev;
	struct tegracam_ctrl_handler *handler = s_data->tegracam_ctrl_hdl;
    struct v4l2_ctrl *ctrl;
    int i;

	for (i = 0; i < handler->numctrls; i++) {
       	ctrl = handler->ctrls[i];
 
        if ( ctrl->id == ctrl_id )
            return ctrl;
	}

    dev_warn(dev, "%s: Couldn't find control with [ %x ] id\n", 
                    __func__, ctrl_id);
    return NULL;
       
}
EXPORT_SYMBOL(fm_find_v4l2_ctrl);


void fm_update_ctrl_range(struct tegracam_device *tc_dev,
                                              int ctrl_id, u64 min, u64 max)
{
	struct camera_common_data *s_data = tc_dev->s_data;
	struct device *dev = tc_dev->dev;
	struct sensor_control_properties *ctrlprops = NULL;
    struct v4l2_ctrl *ctrl;
	const struct tegracam_ctrl_ops *ops = tc_dev->tcctrl_ops;
    int err = 0;

    ctrlprops =
		&s_data->sensor_props.sensor_modes[s_data->mode].control_properties;

    ctrl = fm_find_v4l2_ctrl(tc_dev, ctrl_id);
    if (ctrl == NULL) {
        return;
    }

    switch (ctrl_id) {
    case TEGRA_CAMERA_CID_EXPOSURE:
        s_data->exposure_min_range = min;
        s_data->exposure_max_range = max;

        /* Current value must be recalculated */
        dev_dbg(dev, "%s: recalculate exposure for set frame length.\n", __func__);
        err = ops->set_exposure(tc_dev, *ctrl->p_cur.p_s64);

	    dev_dbg(dev, "%s:  mode: %u, exposure range [%llu, %llu]\n",
                 __func__, s_data->mode, s_data->exposure_min_range, 
                s_data->exposure_max_range);
        break;
    case TEGRA_CAMERA_CID_EXPOSURE_SHORT:
        s_data->short_exposure_min_range = min;
        s_data->short_exposure_max_range = max;

        /* Current value must be recalculated */
        dev_dbg(dev, "%s: recalculate short exposure for set frame length.\n", __func__);
        err = ops->set_exposure_short(tc_dev, *ctrl->p_cur.p_s64);

	    dev_dbg(dev, "%s:  mode: %u, short exposure range [%llu, %llu]\n",
                 __func__, s_data->mode, s_data->short_exposure_min_range, 
                s_data->short_exposure_max_range);
        break;
    case TEGRA_CAMERA_CID_FRAME_RATE:
        ctrlprops->min_framerate = min;
        ctrlprops->max_framerate = max;
        /* default value must be in range */
        ctrlprops->default_framerate = clamp_val(ctrlprops->default_framerate,
                                                 ctrlprops->min_framerate,
                                                 ctrlprops->max_framerate);

	    dev_dbg(dev, "%s:  mode: %u, framerate range [%u, %u]\n",
                 __func__, s_data->mode,
                ctrlprops->min_framerate, ctrlprops->max_framerate);
        break;
    }

	if (err) {
		dev_err(dev,
			"%s: ctrl %s range update failed\n", __func__, ctrl->name);
	}
}
EXPORT_SYMBOL(fm_update_ctrl_range);

/**
 * Get I2C broadcast client
 */
int common_get_broadcast_client(struct tegracam_device *tc_dev,
                                struct v4l2_ctrl *ctrl, 
                                const struct regmap_config *sensor_regmap_config)
{
	struct camera_common_data *s_data = tc_dev->s_data;
	struct device *dev = tc_dev->dev; 

    if (!s_data->broadcast_regmap) {
        struct i2c_client *broadcast_client = 
                                        get_broadcast_client(tc_dev->client);

        /* Disable broadcast mode if an error occurs */
        if (IS_ERR_OR_NULL(broadcast_client)) {  
            dev_warn_once(dev, "%s: couldn't get broadcast client\n", __func__);          
            ctrl->val = UNICAST;
            v4l2_ctrl_activate(ctrl, 0);
            return 0;
        }
      
        s_data->broadcast_regmap = devm_regmap_init_i2c(broadcast_client,
			                 sensor_regmap_config);

        /* Disable broadcast mode if an error occurs */
        if (IS_ERR_OR_NULL(s_data->broadcast_regmap)) {
            ctrl->val = UNICAST;            
            v4l2_ctrl_activate(ctrl, 0);

            dev_err (dev, "regmap init failed: %ld\n",
                PTR_ERR (s_data->broadcast_regmap));
            return -ENODEV;
        }
	}

    return 0;
}
EXPORT_SYMBOL(common_get_broadcast_client);

MODULE_DESCRIPTION("Framos Image Sensor common logic");
MODULE_AUTHOR("FRAMOS GmbH");
MODULE_LICENSE("GPL v2");
