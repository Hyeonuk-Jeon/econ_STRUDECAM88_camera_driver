/*
 * Copyright (c) 2018, NVIDIA Corporation.  All rights reserved.
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

/**
 * @file
 * <b>max96793 API: For Maxim Integrated max96793 serializer.</b>
 *
 * @b Description: Defines elements used to set up and use a
 *  Maxim Integrated max96793 serializer.
 */

#ifndef __max96793_H__
#define __max96793_H__

#include <media/gmsl-link.h>
#include <media/tegra_v4l2_camera.h>
#include <media/tegracam_core.h>
/**
 * \defgroup max96793 max96793 serializer driver
 *
 * Controls the max96793 serializer module.
 *
 * @ingroup serdes_group
 * @{
 */


/**
 * @brief  Powers on a serializer device and performs the I2C overrides
 * for sensor and serializer devices.
 *
 * The I2C overrides include setting proxy I2C slave addresses for the devices.
 *
 * Before the client calls this function it must ensure that
 * the deserializer device is in link_ex exclusive link mode
 * by calling the deserializer driver's max9296_setup_link() function.
 *
 * @param  [in]  dev            The serializer device handle.
 *
 * @return  0 for success, or -1 otherwise.
 */
int max96793_setup_control(struct device *dev);

/**
 * Reverts I2C overrides and resets a serializer device.
 *
 * @param  [in]  dev            The serializer device handle.
 *
 * @return  0 for success, or -1 otherwise.
 */
int max96793_reset_control(struct device *dev);

/**
 * @brief  Pairs a sensor device with a serializer device.
 *
 * To be called by sensor client driver.
 *
 * @param  [in]  dev            The deserializer device handle.
 * @param  [in]  g_ctx          The @ref gmsl_link_ctx structure handle.
 *
 * @return  0 for success, or -1 otherwise.
 */
int max96793_sdev_pair(struct device *dev, struct gmsl_link_ctx *g_ctx);

/**
 * @brief Unpairs a sensor device from a serializer device.
 *
 * To be called by sensor client driver.
 *
 * @param  [in]  dev            The serializer device handle.
 * @param  [in]  s_dev          The sensor device handle.
 *
 * @return  0 for success, or -1 otherwise.
 */
int max96793_sdev_unpair(struct device *dev, struct device *s_dev);

/**
 * Sets up the serializer device's internal pipeline for a specified
 * sensor/serializer pair.
 *
 * @param  [in]  dev            The serializer device handle.
 *
 * @return  0 for success, or -1 otherwise.
 */
int max96793_setup_streaming(struct device *dev, struct camera_common_data *s_data);

int max96793_gmsl_setup(struct device *dev);

int max96793_gpio10_xtrig1_setup(struct device *dev, char *image_sensor_type);

int max96793_xvs_setup(struct device *dev, bool direction);

enum {
    max96793_OUT,
    max96793_IN,
};

/** @} */

#endif  /* __max96793_H__ */
