/*
 * framos_sensor_common.h - common logic
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

#ifndef __framos_sensor_common__
#define __framos_sensor_common__

/**
 * FSA hardware timeouts in useconcds
 */ 
#define FCON_RST_TIMEOUT            25000  //20ms + 5ms
#define FCON_SEQ_FLAG_TIMEOUT       40000  //30ms + 10ms
#define FCON_REG_DISC_TIMEOUT       200000 //200ms


//CTI addition: range for camera eeprom search 
#define FSM_START_ADDRESS 0x55
#define FSM_END_ADDRESS 0x57

/**
 * Enum SOM
 */
enum {
    JETSON_TX2,
    JETSON_XAVIER,
    JETSON_NANO_DEV_KIT,
    JETSON_XAVIER_NX_DEV_KIT,
    JETSON_TX2_NX,
};

/**
 * Enum describing available operation modes
 */
typedef enum {
    MASTER_MODE,
    SLAVE_MODE,
} operation_mode;

/**
 * Enum describing available shutter modes
 */
typedef enum {
    NORMAL_EXPO,
    SEQ_TRIGGER,
} shutter_mode;

/**
 * Enum describing available streaming modes
 */
typedef enum {
    STANDALONE_STREAM,
    SYNC_STREAM,
    EXTERNAL_HW_SYNC_STREAM,
} streaming_mode;

/**
 * Enum describing available sync modes
 */
typedef enum {
    NO_SYNC,
    INTERNAL_SYNC,
    EXTERNAL_SYNC,
} sync_mode;

/**
 * Enum describing available communication modes
 */
typedef enum {
    UNICAST,
    BROADCAST,
} i2c_broadcast_ctrl;

typedef enum {
    MIN,
    MAX,
    DEFAULT,
    CURRENT,
    NEW,
} v4l2_ctrl_value;


void fm_get_board_model(struct tegracam_device *tc_dev, u8 *som_board_model);

void fm_calc_lshift(u64 *val, u8 *lshift, s64 max);

struct v4l2_ctrl *fm_find_v4l2_ctrl(struct tegracam_device *tc_dev, 
                                                int ctrl_id);
void fm_update_ctrl_range(struct tegracam_device *tc_dev,
                                              int ctrl_id, u64 min, u64 max);

void fm_gpio_set(struct camera_common_data *s_data,
			    unsigned int gpio, int val);

void fm_get_gpio_ctrl(struct camera_common_pdata *board_priv_pdata);

int cam_gpio_register(struct device *dev,
			unsigned pin_num);

void cam_gpio_deregister(struct device *dev,
			unsigned pin_num);

int cam_gpio_ctrl(struct device *dev,
			unsigned pin_num, int ref_inc, bool active_high);

int common_get_broadcast_client(struct tegracam_device *tc_dev,
                                struct v4l2_ctrl *ctrl, 
                                const struct regmap_config *sensor_regmap_config);

s64 fm_get_v4l2_ctrl_value(struct tegracam_device *tc_dev,
                                    int ctrl_id, v4l2_ctrl_value ctrl_value);
#endif /* __framos_sensor_common__ */
