/*
 * sl_zedx.c - Stereolabs ar0234 sensor driver
 *
 * Copyright (c) 2022-2023, Stereolabs.  All rights reserved.
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

#define DEBUG 1
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/gpio.h>
#include <linux/module.h>

#include <linux/seq_file.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>

#include <media/tegracam_core.h>
#include "zedx_mode_tbls.h"

#define MAX_RADIAL_COEFFICIENTS 6
#define MAX_TANGENTIAL_COEFFICIENTS 2
#define MAX_FISHEYE_COEFFICIENTS 6

#define CAMERA_MAX_SN_LENGTH            32
#define MAX_RLS_COLOR_CHANNELS          4
#define MAX_RLS_BREAKPOINTS             6


extern int write_reg_Dser(int slaveAddr, int channel,
		u16 addr, u8 val);

extern int read_reg_Dser(int slaveAddr, int channel,
		u16 addr, unsigned int *val);

extern int pipes_01_en_Dser(int channel, u8 link);

extern int pipes_23_en_Dser(int channel, u8 link);

extern int links_dis_Dser(int channel);
extern int cc_en_Dser(int channel, u8 link);
extern int i2c_setup_Dser(int channel, u8 *links);
extern int fast_reset_Dser(int channel);
extern int slow_reset_Dser(int channel);
extern int links_check_Dser(int channel,u8 *links);
extern int fps_set_Dser(int channel, s64 val);

#define AR0234_MIN_GAIN (1)
#define AR0234_MAX_GAIN (8)
#define AR0234_MAX_GAIN_REG (0x40)
#define AR0234_DEFAULT_FRAME_LENGTH (0x30D)
#define AR0234_DEFAULT_FPS (60)
#define AR0234_MIN_LINE_LENGTH_PCK (0x300)
#define AR0234_COARSE_TIME_SHS1_ADDR 0x3012
#define AR0234_ANALOG_GAIN 0x3060
#define AR0234_DEFAULT_RES (0x04D0)
#define AR0234_BINNED_RES  (0x0268)
#define AR0234_DEFAULT_EXP  (5)

/**
 * Structure definitions
 */

/**
 * struct of_device_id* - Link the driver to the dts.
 * @compatible: String that includes the name of the
 *				company and the name of the device.
 *
 * To add support for your ZED-X, the dts compatibility
 * must match "stereolabs,zedx".
 */
const struct of_device_id zedx_of_match[] = {
	{.compatible = "stereolabs,zedx"},
	{},
};

/**
 * macro MODULE_DEVICE_TABLE - Allow hotplug of the device
 * Exposes the vendor/device id for the compiler.
 */
MODULE_DEVICE_TABLE(of, zedx_of_match);

/**
 * List of control defined in tegra-v4l2-camera.h
 */
static const u32 ctrl_cid_list[] = {
	TEGRA_CAMERA_CID_GAIN,
	TEGRA_CAMERA_CID_EXPOSURE,
	TEGRA_CAMERA_CID_EXPOSURE_SHORT,
	TEGRA_CAMERA_CID_FRAME_RATE,
	TEGRA_CAMERA_CID_EEPROM_DATA,
	TEGRA_CAMERA_CID_HDR_EN,
	TEGRA_CAMERA_CID_SENSOR_MODE_ID,
	TEGRA_CAMERA_CID_STEREO_EEPROM,
};

/**
 * struct fisheye_lens_distorion_coeff - Coefficients for the distortion model.
 * @coeff_count: uint 32 - Number of radial coefficients.
 * @k: float* - Table of coefficients.
 * @mapping_type: uint 32 - 0 -> equidistant, 1 -> equisolid,
 *							2 -> orthographic, 3 -> stereographic
 *
 * For lens correction when a wide FOV is used.
 */
typedef struct
{
	u32 coeff_count;
	float k[MAX_FISHEYE_COEFFICIENTS];
	u32 mapping_type;
} fisheye_lens_distortion_coeff;

/**
 * struct polynomial_lens_distorion_coeff - Coefficients for the distortion model.
 * @radial_coeff_count: uint 32 - Number of radial coefficients.
 * @k: float* - Table of radial coefficients.
 * @tangential_coeff_count: uint 32 - Number of tangential coefficients.
 * @p: float* - Table of tangential coefficients.
 *
 * For lens correction when a wide FOV is used.
 */
typedef struct
{
	u32 radial_coeff_count;
	float k[MAX_RADIAL_COEFFICIENTS];
	u32 tangential_coeff_count;
	float p[MAX_TANGENTIAL_COEFFICIENTS];
} polynomial_lens_distortion_coeff;

/**
 * struct camera_intrinsics - Camera calibration settings.
 * @width: uint 32 - Width of the image in pixel.
 * @height: uint32 - Height of the image in pixel.
 * @fx: float - focal length accross the x axis in pixel.
 * @fy: float - focal length accross the y axis in pixel.
 * @tangential_coeff_count: uint 32 - Number of tangential coefficients.
 * @skew: float - Skew of the sensor.
 * @cx: float - x coordinate of the optical center in pixels.
 * @cy: float - y coordinate of the optical center in pixels.
 * @distortion_type: uint32 - type of distortion, depending on the lens.
 *					0: pinhole, assuming polynomial distortion
 *					1: fisheye, assuming fisheye distortion)
 *					2: ocam (omini-directional)
 * @distortion_coefficients: union of structures - Distortion coefficients
 *							 corresponding to the lens. See the structs above
 *							 for more details.
 *
 * Camera and lens intrinsic calibration settings.
 */
typedef struct
{
	u32 width, height;
	float fx, fy;
	float skew;
	float cx, cy;
	u32 distortion_type;
	union distortion_coefficients
	{
		polynomial_lens_distortion_coeff poly;
		fisheye_lens_distortion_coeff fisheye;
	} dist_coeff;
} camera_intrinsics;

/**
 * struct camera_extrinsics - Camera and IMU rotation and translation parameters.
 * @rx: float - Rotation parameter expressed in Rodrigues notation:
 *		angle = sqrt(rx^2+ry^2+rz^2), unit axis = s = [rx,ry,rz]/angle.
 * @ry: float - Rotation parameter expressed in Rodrigues notation.
 * @rz: float - Rotation parameter expressed in Rodrigues notation.
 * @tx: float - Translation parameter.
 * @ty: float - Translation parameter.
 * @tz: float - Translation parameter.
 *
 * All rotations and translations are done with respect to the same reference point.
 */
typedef struct
{
	float rx, ry, rz;
	float tx, ty, tz;
} camera_extrinsics;

/**
 * struct imu_params - IMU calibration settings.
 * @linear_acceleration_bias: float[3] - 3D vector containing the accelerometer bias.
 * 							  Must be added to the accelerometer readings.
 * @angular_velocity_bias: float[3] - 3D vector containing the gyroscope bias.
 * 						   Must be added to the gyroscope readings.
 * @gravity_acceleration: float[3] - Gravity acceleration when the camera is horizontal.
 * @extr: camera_extrinsincs - Rotation and transltation parameters.
 * @update_rate: float
 * @linear_acceleration_noise_density: float - Accelerometer noise parameter
 * @linear_acceleration_random_walk: float - Accelerometer noise parameter
 * @angular_velocity_noise_density: float - Gyroscope noise parameter
 * @angular_velocity_random_walk: float - Gyroscope noise parameter
 *
 * IMU calibration settings.
 */
typedef struct
{
	float linear_acceleration_bias[3];
	float angular_velocity_bias[3];
	float gravity_acceleration[3];
	camera_extrinsics extr;
#ifndef CONFIG_TEGRA_L4T_351
	float update_rate;
	float linear_acceleration_noise_density;
	float linear_acceleration_random_walk;
	float angular_velocity_noise_density;
	float angular_velocity_random_walk;
#endif
} imu_params;

#ifndef CONFIG_TEGRA_L4T_351
/**
 * radial_lsc_params - Radial LSC parameters for image correction
 *
 * @image_height: Image height
 * @image_width: Image width
 * @n_channels: Number of color channels
 * @rls_x0: X-coordinate of center point for each color channel
 * @rls_y0: Y-coordinate of center point for each color channel
 * @ekxx: Ellipse xx parameter for each color channel
 * @ekxy: Ellipse xy parameter for each color channel
 * @ekyx: Ellipse yx parameter for each color channel
 * @ekyy: Ellipse yy parameter for each color channel
 * @rls_n_points: Number of breakpoints in LSC radial transfer function
 * @rls_rad_tf_x: LSC radial transfer function X for each color channel and breakpoint
 * @rls_rad_tf_y: LSC radial transfer function Y for each color channel and breakpoint
 * @rls_rad_tf_slope: LSC radial transfer function slope for each color channel and breakpoint
 * @r_scale: rScale parameter
 *
 * Structure containing parameters for radial LSC (Lens Shading Correction)
 * for image correction purposes.
 */
typedef struct {
	u16 image_height;
	u16 image_width;
	u8 n_channels;
	float rls_x0[MAX_RLS_COLOR_CHANNELS];
	float rls_y0[MAX_RLS_COLOR_CHANNELS];
	double ekxx[MAX_RLS_COLOR_CHANNELS];
	double ekxy[MAX_RLS_COLOR_CHANNELS];
	double ekyx[MAX_RLS_COLOR_CHANNELS];
	double ekyy[MAX_RLS_COLOR_CHANNELS];
	u8 rls_n_points;
	float rls_rad_tf_x[MAX_RLS_COLOR_CHANNELS][MAX_RLS_BREAKPOINTS];
	float rls_rad_tf_y[MAX_RLS_COLOR_CHANNELS][MAX_RLS_BREAKPOINTS];
	float rls_rad_tf_slope[MAX_RLS_COLOR_CHANNELS][MAX_RLS_BREAKPOINTS];
	u8 r_scale;
} radial_lsc_params;
#endif

/**
 * struct NvCamSyncSensorCalibData - Camera calibration settings.
 * @cam_intr: struct camera_intrinsics - Intrinsics calibration parameters, see above.
 * @cam_extr: struct camera_extrinsics - Extrinsics calibration parameters, see above.
 * @imu_present: uint 8 - IMU availability. 0 means no IMU is present.
 * @imu: struct imu_params - IMU calibration settings.
 * @serial_number: u8* - table containing the ZED serial number.
 * @rls: radial_lsc_params - Radial lens shading correction parameters
 *				for the optics.
 *
 * Camera calibration settings.
 */
typedef struct
{
	camera_intrinsics cam_intr;
	camera_extrinsics cam_extr;
	u8 imu_present;
	imu_params imu;
#ifndef CONFIG_TEGRA_L4T_351
	u8 serial_number[CAMERA_MAX_SN_LENGTH];
	radial_lsc_params rls;
#endif
} NvCamSyncSensorCalibData;

/**
 * struct LiEeprom_Content_Struct - Structure representing the EEPROM content.
 * @version: uint 32 - EEPROM layout version.
 * @factory_data: uint 32 - Factory flag to set when factory flashed.
 * 				  It needs to be reset to 0 when the user modifies it.
 * @left_cam_intr: struct camera_intrinsics - Left camera intrinsics calibration parameters,
 * 				   see above.
 * @right_cam_intr: struct camera_intrinsics - Right camera intrinsics calibration parameters,
 * 				   see above.
 * @cam_extr: struct camera_extrinsics - Extrinsics calibration parameters
 * 			  for the cameras, see above.
 * @imu_present: uint 32 - IMU availability. 0 means no IMU is present.
 * @imu: struct imu_params - IMU calibration settings.
 * @serial_number: u8* - u8 table of containing the serial number of the ZED X.
 * @left_rls: radial_lsc_params - Radial lens shading correction parameters for
 *			the left camera sensor.
 * @right_rls: radial_lsc_params - Radial lens shading correction parameters for
 *			the right camera sensor.
 *
 * Calibration parameters of the ZED-X. Parsed using the EEPROM content.
 */
typedef struct
{
	u32 version;
	u32 factory_data;
	camera_intrinsics left_cam_intr;
	camera_intrinsics right_cam_intr;
	camera_extrinsics cam_extr;
	u32 imu_present;
	imu_params imu;
#ifndef CONFIG_TEGRA_L4T_351
	u8 serial_number[CAMERA_MAX_SN_LENGTH];
	radial_lsc_params left_rls;
	radial_lsc_params right_rls;
#endif
} LiEeprom_Content_Struct;

/**
 * atomic_t master_frame_length - For multiple sensor synchronisation.
 * @master_frame_length: atomic_t - duration of the frame exposure in number of sensor
 * 			  lines.
 *
 * This atomic variable allows a sensor bound to a slave deserializer to be synchronized
 * with a sensor bound to a master deserializer. Both sensors need to be linked by a
 * synchronisation GPIO. See the <sync_mode> entry in the deserializer dts for more informations.
 */
atomic_t master_frame_length = ATOMIC_INIT(0);

/**
 * struct zedx - ZED-X private data.
 * @eeprom: struct camera_common_eeprom_data * - EEPROM i2c interface details.
 *			Here EEPROM_NUM_BLOCKS = 2, hence the EEPROM i2c addresses are x54 and x55.
 * @eeprom_buf: uint 8 - The EEPROM is accessed once, emptying or filling all its memory
 * 				using this buffer.
 * @i2c_client: struct i2c_client * - i2c adapter of the zedx.
 * @id: struct i2c_device_id * - i2c adapter id.
 * @subdev: struct v4l2_subdev * - To register the zedx as a v4l2 device.
 * @frame_length: uint 32 - Exposure duration of a frame.
 * @s_data: struct camera_common_data * - Nvidia camera common data for v4l2 support.
 * @tc_dev: struct tegracam_device * - Nvidia camera data for tegra and argus.
 * @channel: uint 32 - camera id.
 * @master: boolean. Indicates if camera is set as master (first sensor)
 * @sync_sensor_index: uint 32 - Sync source for the camera.
 * @EepromCalib: NvCamSyncSensorCalibData - EEPROM data structure.
 *
 * ZED-X private data for this driver. One structure is initialized by camera sensor,
 * so two per ZED-X.
 */
struct zedx
{
	struct camera_common_eeprom_data eeprom[AR0234_EEPROM_NUM_BLOCKS];
	u8 eeprom_buf[AR0234_EEPROM_SIZE];
	struct i2c_client *i2c_client;
	const struct i2c_device_id *id;
	struct v4l2_subdev *subdev;
	s64 fps;
	u32 pix_clk;
	u32 line_length_pck;
	u32 total_frame_length;
	u16 frame_length_line;
	struct camera_common_data *s_data;
	struct tegracam_device *tc_dev;
	u32 channel;
	bool master;
	bool sync_slave;
	u32 sync_sensor_index;
	NvCamSyncSensorCalibData EepromCalib;
};

/**
 * const struct regmap_config - max9295 register mapping configuration.
 * @reg_bits: uint 32 - 16 bits register addresses.
 * @val_bits: uint 32 - 8 bits register contents.
 * @cache_type: enum - Supported cache type.
 *
 * Register configuration of the ar0234 camera sensor.
 */
static const struct regmap_config serializer_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
	.cache_type = REGCACHE_RBTREE,
};

/**
 * const struct regmap_config - ar0234 register mapping configuration.
 * @reg_bits: uint 32 - 16 bits register addresses.
 * @val_bits: uint 32 - 16 bits register contents.
 * @cache_type: enum - Supported cache type.
 *
 * Register configuration of the ar0234 camera sensor.
 */
static const struct regmap_config sensor_regmap_config = {
	.reg_bits = 16,
	.val_bits = 16,
	.cache_type = REGCACHE_RBTREE,
};

/* Miscellaneous functions*/

/**
 * zedx_get_coarse_time_regs_shs1() - Prepare an integration time reading.
 * @regs: A structure to write the register address and value.
 * @coarse_time: A 16 bits mask coressponding to the integration time of the sensor.
 *
 * Prepare the register mask and address to get the integration time of the ar0234
 * sensor as multiples of line_length_pck_.
 *
 * Context: Inlined function, can sleep.
 * Return: Void.
 */
static inline void zedx_get_coarse_time_regs_shs1(ar0234_reg *regs,
		u16 coarse_time)
{
	regs->addr = AR0234_COARSE_TIME_SHS1_ADDR;
	regs->val = (coarse_time)&0xffff;
}

/**
 * zedx_get_gain_reg() - Prepare a gain reading.
 * @regs: A structure to write the register address and value.
 * @coarse_time: A 16 bits mask corresponding to the gain of the sensor.
 *
 * Prepare the register mask and address to get the gain of the ar0234 sensor.
 *
 * Context: Inlined function, can sleep.
 * Return: Void.
 */
static inline void zedx_get_gain_reg(ar0234_reg *regs,
		u16 gain)
{
	regs->addr = AR0234_ANALOG_GAIN;
	regs->val = (gain)&0xffff;
}

/**
 * zedx_set_line_length_px_clk() - set the line length for the sensor.
 * @regs: A structure to write the register address and value.
 * @px_clk_per_line: The number of clock period per sensor line.
 *
 * This function sets the number of Pixel Clock period per sensor line. This
 * way the exposure time can be set precisely.
 *
 * Context: Inlined function, can sleep.
 * Return: Void.
 */
static inline void zedx_set_line_length_px_clk(ar0234_reg *regs, u16 px_clk_per_line)
{
	regs->addr = 0x300C;
	regs->val = (px_clk_per_line)&0xffff;
}

 /**
 * zedx_read_reg() - Read in the registers of the ar0234 sensor.
 * @s_data: Camera data, for the regmapping.
 * @addr: Address of the register to write.
 * @val: Value to write in the register
 *
 * Read 16 bits in a register of the ar0234 sensor.
 *
 * Context: Non critical function, can sleep.
 * Return: 0 in case of success and a negative errno in case of error.
 */
static inline int zedx_read_reg(struct camera_common_data *s_data,
		u16 addr, u16 *val)
{
	int err = 0;
	u32 reg_val = 0;

	err = regmap_read(s_data->regmap, addr, &reg_val);
	*val = reg_val & 0xFFFF;

	return err;
}

/**
 * zedx_write_reg() - Write in the registers of the ar0234 sensor.
 * @s_data: Camera data, for the regmapping.
 * @addr: Address of the register to write.
 * @val: Value to write in the register
 *
 * Write 16 bits in a register of the ar0234 sensor.
 *
 * Context: Non critical function, can sleep.
 * Return: 0 in case of success and a negative errno in case of error.
 */
static int zedx_write_reg(struct camera_common_data *s_data,
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

/**
 * zedx_links_check() - Check the status of the GMSL links.
 * @priv: Driver private data.
 * @addr: Table of u8, each entry is the indice of a connected cable.
 *
 * Get the connected GMSL links and return their indices with the u8 table
 * provided as an argument. This table must have a size of 4, for a quad
 * deserializer.
 *
 * Context: Non critical function, can sleep.
 * Return: The number of detected links in case of success,
 * or 5 if it a slave device calls the function,
 * and a negative errno in case of error.
 */
static int zedx_links_check(struct zedx *priv, u8 *links)
{
	struct tegracam_device *tc_dev = priv->tc_dev;
	struct device *dev = tc_dev->dev;
	int ret = 0;

	// This functions returns the number of connected GMSL links,
	// and their indices in the links pointer
	ret = links_check_Dser(priv->channel, links);

	dev_info(dev, "%s: %d link(s) detected\n", __func__, ret);

	if (ret < 0)
	{
		dev_warn(dev, "%s: fail while reading the GMSL links state\n", __func__);
		return -ENODEV;
	}

	return ret;
}

/**
 * zedx_write_table() - Write in the registers.
 * @priv: Driver data.
 * @table: Table of registers to write in the deserializer.
 *
 * Write a table of 16 bits registers sequentially to the serializer
 * or deserializer. The timings are important.
 * For each register, it tries two times before returning with an error.
 *
 * Context: Non critical function, can sleep.
 * Return: 0 in case of success and a negative errno in case of error.
 */
static int zedx_write_table(struct zedx *priv,
		const struct index_reg_8 table[])
{
	struct tegracam_device *tc_dev = priv->tc_dev;
	struct device *dev = tc_dev->dev;
	int i = 0;
	int ret = 0;
	int retry = 5;

	/* While we haven't reach the end of the table */
	while (table[i].source != 0x00)
	{
		// Camera registers
		if (table[i].source == 0x06)
		{
			retry = 1;

			if (table[i].addr == AR0234_TABLE_WAIT_MS)
			{
				msleep(table[i].val);
				i++;
				continue;
			}
retry_sensor:
			ret = zedx_write_reg(priv->s_data, table[i].addr, table[i].val);
			if (ret)
			{
				retry--;
				if (retry > 0)
				{
					dev_warn(dev, "ZED-X_write_reg: try %d\n", retry);
					msleep(4);
					goto retry_sensor;
				}
				return -1;
			}
			else
			{
				if (0x301a == table[i].addr || 0x3060 == table[i].addr)
					msleep(100);
			}
		}
		// SerDes registers
		else
		{
			retry = 5;

			if (!priv->master)
			{
				i++;
				continue;
			}

retry_serdes:
			ret = write_reg_Dser(table[i].source, priv->channel, table[i].addr, (u8)table[i].val);
			if (ret && (table[i].addr != 0x0000))
			{
				retry--;
				if (retry > 0)
				{
					dev_warn(dev, "write_reg_Dser: try %d\n", retry);
					msleep(4);
					goto retry_serdes;
				}
				return -1;
			}
			if (0x0010 == table[i].addr || 0x0000 == table[i].addr || 0x0006 == table[i].addr || 0x0018 == table[i].addr)
				msleep(300);
			else
				msleep(100);
		}
		i++;
	}
	return 0;
}


/**
 * zedx_power_on() - Power on the Deserializer.
 * @s_data: Structure to the camera GPIOs.
 *
 * Trigger a GPIO to power on the deserializer.
 *
 * Context: Non critical function, can sleep.
 * Return: 0 in case of success and a negative errno in case of error.
 */
static int zedx_power_on(struct camera_common_data *s_data)
{
	int err = 0;
	struct camera_common_power_rail *pw = s_data->power;
	struct camera_common_pdata *pdata = s_data->pdata;
	struct device *dev = s_data->dev;

	if (verbosity_level>=1)
		dev_dbg(dev, "%s: power on\n", __func__);

	if (pdata && pdata->power_on)
	{
		err = pdata->power_on(pw);
		if (err)
			dev_err(dev, "%s failed.\n", __func__);
		else
			pw->state = SWITCH_ON;
		return err;
	}
	// Be careful, reset_gpio is an unsigned int,
	// but it is treated as a signed one sometimes.
	// The value 0 is reserved for errors here. to think
	if (verbosity_level>=1)
		dev_info(dev, "%s Value of the power GPIO: %u\n", __func__, pw->reset_gpio);

	if (pw->reset_gpio > 0)
	{
		gpio_set_value(pw->reset_gpio, 1);
		usleep_range(1000, 2000);
	}

	pw->state = SWITCH_ON;

	return 0;
}

/**
 * zedx_power_off() - Power off the deserializer.
 * @s_data: Structure to the camera GPIOs.
 *
 * Trigger a GPIO to power off the deserializer.
 *
 * Context: Non critical function, can sleep.
 * Return: 0 in case of success and a negative errno in case of error.
 */
static int zedx_power_off(struct camera_common_data *s_data)
{
	return 0;
}

/**
 * zedx_power_get() - Get the power state.
 * @tc_dev: tegracam_device driver structure.
 *
 * Get the power status of the deserializer and the frame clock.
 *
 * Context: Non critical function, can sleep.
 * Return: 0 in case of success and a negative errno in case of error.
 */
static int zedx_power_get(struct tegracam_device *tc_dev)
{
	struct device *dev = tc_dev->dev;
	struct camera_common_data *s_data = tc_dev->s_data;
	struct camera_common_power_rail *pw = s_data->power;
	struct camera_common_pdata *pdata = s_data->pdata;
	const char *mclk_name;
	const char *parentclk_name;
	struct clk *parent;
	int err = 0;

	mclk_name = pdata->mclk_name ? pdata->mclk_name : "cam_mclk1";
	pw->mclk = devm_clk_get(dev, mclk_name);
	if (IS_ERR(pw->mclk))
	{
		dev_err(dev, "unable to get clock %s\n", mclk_name);
		return PTR_ERR(pw->mclk);
	}

	parentclk_name = pdata->parentclk_name;
	if (parentclk_name)
	{
		parent = devm_clk_get(dev, parentclk_name);
		if (IS_ERR(parent))
		{
			dev_err(dev, "unable to get parent clock %s",
					parentclk_name);
		}
		else
			clk_set_parent(pw->mclk, parent);
	}
	if (!err)
	{
		pw->reset_gpio = pdata->reset_gpio;
		pw->af_gpio = pdata->af_gpio;
		pw->pwdn_gpio = pdata->pwdn_gpio;
	}

	pw->state = SWITCH_OFF;

	return err;
}


/**
 * zedx_power_put() - Check the power rail of the camera.
 * @tc_dev: tegracam_device driver structure.
 *
 * Check that the camera_common_power_rail structure is initialized.
 *
 * Context: Non critical function, can sleep.
 * Return: 0 in case of success and a negative errno in case of error.
 */
static int zedx_power_put(struct tegracam_device *tc_dev)
{
	struct camera_common_data *s_data = tc_dev->s_data;
	struct camera_common_power_rail *pw = s_data->power;

	if (unlikely(!pw))
		return -EFAULT;

	return 0;
}

/* tegracam driver interface functions */

/**
 * zedx_set_group_hold - Dummy function
 * @tc_dev: tegracam_device driver structure.
 * @val: boolean to set the group hold.
 *
 * Does nothing: for tegracam driver only.
 *
 * Context: Non critical function, can sleep.
 * Return: 0.
 */
static int zedx_set_group_hold(struct tegracam_device *tc_dev, bool val)
{
	return 0;
}

/**
 * zedx_set_gain - Set the gain of the camera sensor.
 * @tc_dev: tegracam_device driver structure.
 * @val: value of the sensor gain.
 *
 * Encode val in 16 bits and writes it in the sensor gain register
 * of the ar0234.
 *
 * Context: Non critical function, can sleep.
 * Return: 0 in case of success and a negative errno in case of error.
 */
static int zedx_set_gain(struct tegracam_device *tc_dev, s64 val)
{
	struct camera_common_data *s_data = tc_dev->s_data;
	struct device *dev = tc_dev->dev;
	ar0234_reg reg_list[1];
	int err;
	u16 gain = (u16)val;
	u16 gain_reg = 0;

	/* From the datasheet, gain_reg = 0brYYYyyyyrXXXxxxx,
	 * where r is reserved, X is for context A and Y the context B.
	 * The low or high case indicate fine or coarse gain bit.
	 * The coarse gain is given by 2**(0bAAA).
	 * The recommended minimum gain is 1.684. */
	if (val <= 168)
	{
		gain_reg = 0x0D;
	}
	/* Coarse gain = 2**0 */
	else if (val < 200)
	{
		gain_reg = (32 * (1000 - (100000 / gain))) / 1000;
	}
	/* Coarse gain = 2**1 */
	else if (val < 400 && val >= 200)
	{
		gain = gain / 2;
		gain_reg = (16 * (1000 - (100000 / gain))) / 1000 * 2;
		gain_reg = gain_reg + 0x10;
	}
	/* Coarse gain = 2**2  */
	else if (val < 800 && val >= 400)
	{
		gain = gain / 4;
		gain_reg = (32 * (1000 - (100000 / gain))) / 1000;
		gain_reg = gain_reg + 0x20;
	}
	/* Coarse gain = 2**3 */
	else if (val < 1600 && val >= 800)
	{
		gain = gain / 8;
		gain_reg = (16 * (1000 - (100000 / gain))) / 1000 * 2;
		gain_reg = gain_reg + 0x30;
	}
	/* Max Gain = 2**4 */
	else if (val >= 1600)
	{
		gain_reg = 0x40;
	}

	if (gain > AR0234_MAX_GAIN_REG)
		gain = AR0234_MAX_GAIN_REG;
	zedx_get_gain_reg(reg_list, gain_reg);
	err = zedx_write_reg(s_data, reg_list[0].addr,
			reg_list[0].val);
	if (err)
	{
		dev_info(dev, "%s: GAIN control error\n", __func__);
	}

	return err;
}

/**
 * zedx_set_exposure - Set the frame exposure of the camera sensor.
 * @tc_dev: tegracam_device driver structure.
 * @val: value of the frame exposure duration in nanoseconds.
 *
 * Set the coarse frame exposure duration of the ar0234 sensor in nanoseconds.
 *
 * Context: Non critical function, can sleep.
 * Return: 0 in case of success and a negative errno in case of error.
 */
static int zedx_set_exposure(struct tegracam_device *tc_dev, s64 val)
{
	struct zedx *priv = (struct zedx *)tegracam_get_privdata(tc_dev);
	//struct camera_common_data *s_data = tc_dev->s_data;
	//const struct sensor_mode_properties *mode =
	//	&s_data->sensor_props.sensor_modes[s_data->mode];
	ar0234_reg reg_list[1];
	int err;
	u64 coarse_time;
	u32 max_coarse_time;
	u32 shs1;

	/* maximum coarse time in number of line */
	max_coarse_time = priv->total_frame_length-1 ;

	/* coarse time in number of line */
	coarse_time = priv->pix_clk * val / priv->line_length_pck / 1000000;

	if (coarse_time > max_coarse_time)
		coarse_time = (u64)max_coarse_time;

	shs1 = (u32) coarse_time;

	/* 0 and 1 are prohibited */
	if (shs1 < 2)
		shs1 = 2;

	zedx_get_coarse_time_regs_shs1(reg_list, shs1);
	err = zedx_write_reg(priv->s_data, reg_list[0].addr,
			reg_list[0].val);

	if (err)
		dev_dbg(&priv->i2c_client->dev,"%s: set coarse time error\n", __func__);

	if (verbosity_level)
	{
		dev_info(&priv->i2c_client->dev,
				"%s: coarse_time value is %lld\n",
				__func__, coarse_time);
		dev_info(&priv->i2c_client->dev,
				"%s: max_coarse_time value is %d\n",
				__func__, max_coarse_time);
		dev_info(&priv->i2c_client->dev,
				"%s: expo value is %lld\n",
				__func__, val);
		dev_info(&priv->i2c_client->dev,
				"%s: line length in pixel clock value is %d\n",
				__func__, priv->line_length_pck);
	}

	return err;
}

/**
 * zedx_set_frame_rate - Set the frame rate of the camera sensor.
 * @tc_dev: tegracam_device driver structure.
 * @val: value of the frame rate power 10^6.
 *
 * Set the frame rate at 30, 60 or 120 fps. For 30 FPS, the line length in
 * number of pixel clocks need to be adjusted.
 *
 * Context: Non critical function, can sleep.
 * Return: 0 in case of success and a negative errno in case of error.
 */
static int zedx_set_frame_rate(struct tegracam_device *tc_dev, s64 val)
{
	struct zedx *priv = (struct zedx *)tegracam_get_privdata(tc_dev);
	struct device *dev = tc_dev->dev;
	u32 fps_val = 0, vertical_blanking = 0;
	u16 row_start = 0, row_stop = 0;
	u8 bin_coeff = 0;
	ar0234_reg reg_list[1];
	int err;

	err = (int)fps_set_Dser(priv->channel, val);

	if (!priv->sync_slave)
	{
		if (err == 0xEEEE)
		{
			if (verbosity_level)
				dev_warn(dev, "%s: Unsupported value\n", __func__);

			return 0;
		}
		if (err == 0xFFFF)
		{
			dev_warn(dev, "%s: Deserializer write error\n", __func__);
			return -1;
		}
	}

	if (priv->sync_slave)
	{
		priv->frame_length_line = (u32) atomic_read(&master_frame_length);
		if (verbosity_level)
			dev_info(dev, "%s: serializer in slave mode, FLL = %d\n", __func__, priv->frame_length_line);
	}

	/* To differentiate the 1200p or 600p binned mode */
	bin_coeff = (priv->frame_length_line == AR0234_BINNED_RES) ? 2 : 1;

	/* Transform val in true FPS to avoid using a u64*/
	fps_val = (u32)(val/1000000);

	/* Compute the new line length in terms of pixel clocks (LLPCK).
	 * The time to sample a line is equal to LLPCK/PCK
	 * where PCK is the pixel clock.
	 * This calcul is based on the resolution and the FPS value requested
	 * and it is valid accross all combinations supported by this driver.
	 * Minimum LLPCK is 612 and maximum LLPCK depends on the maximum exposure
	 * time (fps) and resolution (binned or not binned). */
	priv->line_length_pck = AR0234_MIN_LINE_LENGTH_PCK*(u32)bin_coeff;
	priv->line_length_pck = priv->line_length_pck*AR0234_DEFAULT_FPS/fps_val;

	zedx_set_line_length_px_clk(reg_list, priv->line_length_pck);

	err = zedx_write_reg(priv->s_data, reg_list[0].addr,
		reg_list[0].val);

	zedx_read_reg(priv->s_data,AR0234_ROW_START, &row_start);

	zedx_read_reg(priv->s_data,AR0234_ROW_STOP, &row_stop);

	/* Compute the vertical blanking in term of pixel clocks.
	 * It is given by (FLL+5-N_rows)*LLPCK in the datasheet.
	 * Here, the bin_coeff denotes the 600p binning case,
	 * where n_rows is divided by 2. */
	vertical_blanking = (priv->frame_length_line+5-(row_stop-row_start+1)/bin_coeff)*(priv->line_length_pck);

	/* Total frame length in number of lines (F in the datasheet).
	 * F*LLPCK*PCK should be slightly lower than 1/fps to guarrantee
	 * that the exposure is over before the next trigger */
	priv->total_frame_length = vertical_blanking/priv->line_length_pck+(row_stop-row_start+1)/bin_coeff;

	priv->fps = val;

	zedx_set_exposure(tc_dev, AR0234_DEFAULT_EXP*100000);

	if (verbosity_level)
	{
		dev_info(dev, "%s: requested fps value is %lld \n", __func__, val);

		dev_info(dev, "%s: wrote %d in the line length pck reg 0x300C \n",
				__func__, priv->line_length_pck);

		dev_info(dev, "%s: total frame length in number of lines is %d \n",
				__func__, priv->total_frame_length);

		dev_info(dev, "%s: vertical blanking in pixel clock is %d \n", __func__, vertical_blanking);
	}

	return 0;
}


/**
 * zedx_fill_string_ctrl - Fill the eeprom buffer.
 * @tc_dev: tegracam_device driver structure.
 * @v4l2_ctrl: video for linux driver structure.
 *
 * Fill the eeprom buffer memory 16 bits at a time. The eeprom
 * is written later by the eeprom driver (avoiding any bottleneck).
 *
 * Context: Non critical function, can sleep.
 * Return: 0 in case of success and a negative errno in case of error.
 */
static int zedx_fill_string_ctrl(struct tegracam_device *tc_dev,
		struct v4l2_ctrl *ctrl)
{
	/* eeprom is read by the zedsdk */

	return 0;
}

/**
 * zedx_fill_eeprom - Fill the calibration structures.
 * @tc_dev: tegracam_device driver structure.
 * @v4l2_ctrl: video for linux driver structure.
 *
 * Copy the content of the eeprom in the driver calibration structures.
 *
 * Context: Non critical function, can sleep.
 * Return: 0 in case of success and a negative errno in case of error.
 */
static int zedx_fill_eeprom(struct tegracam_device *tc_dev,
		struct v4l2_ctrl *ctrl)
{
#ifndef CONFIG_TEGRA_L4T_351
	struct zedx *priv = tc_dev->priv;
	LiEeprom_Content_Struct tmp;
	u32 test = 0;

	switch (ctrl->id)
	{
		case TEGRA_CAMERA_CID_STEREO_EEPROM:
			memset(&(priv->EepromCalib), 0, sizeof(NvCamSyncSensorCalibData));
			memset(ctrl->p_new.p, 0, sizeof(NvCamSyncSensorCalibData));
			memcpy(&tmp, priv->eeprom_buf, sizeof(LiEeprom_Content_Struct));

			if (priv->sync_sensor_index == 1) {
				priv->EepromCalib.cam_intr =  tmp.left_cam_intr;
			} else if (priv->sync_sensor_index == 2) {
				priv->EepromCalib.cam_intr =  tmp.right_cam_intr;
			} else {
				priv->EepromCalib.cam_intr =  tmp.left_cam_intr;
			}
			priv->EepromCalib.cam_extr = tmp.cam_extr;
			priv->EepromCalib.imu_present = tmp.imu_present;
			priv->EepromCalib.imu = tmp.imu;
			memcpy(priv->EepromCalib.serial_number, tmp.serial_number,
					CAMERA_MAX_SN_LENGTH);

			if (priv->sync_sensor_index == 1)
				priv->EepromCalib.rls = tmp.left_rls;
			else if (priv->sync_sensor_index == 2)
				priv->EepromCalib.rls = tmp.right_rls;
			else
				priv->EepromCalib.rls = tmp.left_rls;

			memcpy(ctrl->p_new.p, (u8 *)&(priv->EepromCalib),
					sizeof(NvCamSyncSensorCalibData));
			break;
		default:
			return -EINVAL;
	}

	memcpy(&test, &(priv->EepromCalib.cam_intr.fx), 4);

	ctrl->p_cur.p = ctrl->p_new.p;
#endif
	return 0;
}

/**
 * struct tegracam_ctrl_ops - Structure of control functions.
 * @numctrls: number of control function pointers.
 * @ctrl_cid_list: List of uint 32 control IDs.
 * @string_ctrl_size: Size of the calibration EEPROM.
 * @compound_ctrl_size: Size of the driver calibration data struct.
 * @set_gain: Gain setting function.
 * @set_exposure: Exposure setting function.
 * @set_exposure_short: Exposure setting function.
 * @set_frame_rate: Frame rate setting function.
 * @set_group_hold: Doing Nothing for now.
 * @fill_string_ctrl: Control over the EEPROM, to fill it.
 * @fill_compound_ctrl: Fill the calibration structures from the EEPROM.
 *
 * Structure containing all the control operation function required by
 * NVidia tegracam driver.
 */
static struct tegracam_ctrl_ops zedx_ctrl_ops = {
	.numctrls = ARRAY_SIZE(ctrl_cid_list),
	.ctrl_cid_list = ctrl_cid_list,
	.string_ctrl_size = {AR0234_EEPROM_STR_SIZE},
	.compound_ctrl_size = {sizeof(NvCamSyncSensorCalibData)},
	.set_gain = zedx_set_gain,
	.set_exposure = zedx_set_exposure,
	.set_exposure_short = zedx_set_exposure,
	.set_frame_rate = zedx_set_frame_rate,
	.set_group_hold = zedx_set_group_hold,
	.fill_string_ctrl = zedx_fill_string_ctrl,
	.fill_compound_ctrl = zedx_fill_eeprom,
};

/**
 * zedx_parse_dt - Device tree parser.
 * @tc_dev: tegracam_device driver structure.
 *
 * Copy the content of the device tree. Mainly initializing
 * the GPIOs, especially the master clock to trigger the frame capture.
 *
 * Context: Non critical function, can sleep.
 * Return: A struct camera_common_pdata pointer in case of success
 *         and a NULL pointer in case of error.
 */
static struct camera_common_pdata *zedx_parse_dt(struct tegracam_device *tc_dev)
{
	struct device *dev = tc_dev->dev;
	struct device_node *node = dev->of_node;
	struct camera_common_pdata *board_priv_pdata;
	const struct of_device_id *match;
	int err;
	int gpio = 0;

	if (!node)
		return NULL;

	match = of_match_device(zedx_of_match, dev);
	if (!match)
	{
		dev_err(dev, "Failed to find a matching device tree ID\n");
		return NULL;
	}

	board_priv_pdata = devm_kzalloc(dev, sizeof(*board_priv_pdata), GFP_KERNEL);

	err = of_property_read_string(node, "mclk",
			&board_priv_pdata->mclk_name);
	if (err)
		dev_err(dev, "mclk not in DT\n");

	/* of_get_named return a signed int, but gpios from the
	 * camera_common_struct are unsigned ones... for now reset_gpio = 0
	 * when no gpios are detected but maybe it should be btwn
	 * 0x8000 or 0xFFFF (msb to one), meaning an unreachable gpio.
	 */

	gpio = of_get_named_gpio(node,
			"reset-gpio", 0);

	if (gpio > 0)
	{
		board_priv_pdata->reset_gpio = gpio;
		gpio_direction_output(board_priv_pdata->reset_gpio, 1);
	}
	else
	{
		board_priv_pdata->reset_gpio = 0;
	}

	gpio = of_get_named_gpio(node,
			"pwdn-gpios", 0);

	if (gpio > 0)
		gpio_direction_output(gpio, 1);

	gpio = of_get_named_gpio(node,
			"pwr-gpios", 0);

	if (gpio > 0)
		gpio_direction_output(gpio, 1);

	board_priv_pdata->has_eeprom =
		of_property_read_bool(node, "has-eeprom");

	return board_priv_pdata;
}

/**
 * zedx_set_mode - Set the camera mode.
 * @tc_dev: tegracam_device driver structure.
 *
 * Read the camera mode in the device tree and sets it.
 * For instance it can set the camera in 1080p 30 fps (mode 0).
 *
 * Context: Non critical function, can sleep.
 * Return: 0 in case of success, and a negative errno in case of error.
 */
static int zedx_set_mode(struct tegracam_device *tc_dev)
{
	struct zedx *priv = (struct zedx *)tegracam_get_privdata(tc_dev);
	struct camera_common_data *s_data = tc_dev->s_data;
	struct device *dev = tc_dev->dev;
	const struct of_device_id *match;
	u16 vt_pix_clk_div,vt_sys_clk_div,pll_multiplier,pre_pll_clk_div;
	int err;

	match = of_match_device(zedx_of_match, dev);
	if (!match)
	{
		dev_err(dev, "Failed to find matching dt id\n");
		return -EINVAL;
	}

	err = zedx_write_table(priv, mode_table[AR0234_MODE_STOP_STREAM]);
	if (err)
		return err;

	if (s_data->mode_prop_idx < 0)
		return -EINVAL;

	//dev_dbg(dev, "%s: mode index:%d\n", __func__, s_data->mode_prop_idx);
	err = zedx_write_table(priv, mode_table[s_data->mode_prop_idx]);
	if (err)
		return err;

	/* Valid */
	if (!priv->sync_slave)
	{
		zedx_read_reg(priv->s_data, AR0234_FLL, &priv->frame_length_line);

		atomic_set(&master_frame_length,(int) priv->frame_length_line);

		if (verbosity_level)
			dev_info(dev, "%s: serializer in master mode\n", __func__);

	}

	zedx_read_reg(priv->s_data,AR0234_PLL_MULTIPLIER,&pll_multiplier);
	zedx_read_reg(priv->s_data,AR0234_PRE_PLL_CLK_DIV,&pre_pll_clk_div);
	zedx_read_reg(priv->s_data,AR0234_VT_SYS_CLK_DIV,&vt_sys_clk_div);
	zedx_read_reg(priv->s_data,AR0234_VT_PIX_CLK_DIV,&vt_pix_clk_div);

	priv->pix_clk = EXTCLK*pll_multiplier;

	priv->pix_clk = priv->pix_clk/(pre_pll_clk_div*vt_sys_clk_div*vt_pix_clk_div);

	err = zedx_set_frame_rate( tc_dev, priv->fps );

	err = zedx_set_exposure( tc_dev, AR0234_DEFAULT_EXP*100000 );

	return 0;
}

/**
 * zedx_start_streaming - Start the frame capture.
 * @tc_dev: tegracam_device driver structure.
 *
 * Put the camera into streaming mode.
 *
 * Context: Non critical function, can sleep.
 * Return: 0 in case of success, and a negative errno in case of error.
 */
static int zedx_start_streaming(struct tegracam_device *tc_dev)
{
	struct zedx *priv = (struct zedx *)tegracam_get_privdata(tc_dev);
	int err;
	err = zedx_write_table(priv, mode_table[AR0234_MODE_START_STREAM]);
	return err;
}

/**
 * zedx_stop_streaming - Stop the frame capture.
 * @tc_dev: tegracam_device driver structure.
 *
 * Stop the camera streaming.
 *
 * Context: Non critical function, can sleep.
 * Return: 0 in case of success, and a negative errno in case of error.
 */
static int zedx_stop_streaming(struct tegracam_device *tc_dev)
{
	struct zedx *priv = (struct zedx *)tegracam_get_privdata(tc_dev);
	int err;

	err = zedx_write_table(priv, mode_table[AR0234_MODE_STOP_STREAM]);
	if (err)
		return err;

	return 0;
}

/**
 * struct camera_common_sensor_ops - Tegra camera operation functions
 * @numfrmfmts: Number of camera modes.
 * @frmfmt_table: Camera modes table.
 * @power_on: Deserializer power on function.
 * @power_off: Deserializer power off function.
 * @parse_dt: Device tree parser, get the frame trigger and other gpios.
 * @power_get: Get the Deserializer power state.
 * @power_put: Put the Deserializer in a certain power mode.
 * @set_mode: Set the streaming mode.
 * @start_streaming: Start the camera stream.
 * @stop_streaming: Stop the camera stream.
 *
 * Basic camera operation functions.
 */
static struct camera_common_sensor_ops zedx_common_ops = {
	.numfrmfmts = ARRAY_SIZE(ar0234_frmfmt),
	.frmfmt_table = ar0234_frmfmt,
	.power_on = zedx_power_on,
	.power_off = zedx_power_off,
	.parse_dt = zedx_parse_dt,
	.power_get = zedx_power_get,
	.power_put = zedx_power_put,
	.set_mode = zedx_set_mode,
	.start_streaming = zedx_start_streaming,
	.stop_streaming = zedx_stop_streaming,
};

/* Driver operations */

/**
 * zedx_open() - Open the camera device.
 * @sd: v4l2 subdevice data.
 * @fh: v4l2 subdevice file header.
 *
 * This function is called whenever a user accesses the camera.
 *
 * Context: Can sleep.
 * Return: 0 in case of success and a negative errno in case of unregistered i2c device.
 */
static int zedx_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	if (client == NULL)
		return -EADDRNOTAVAIL;
	dev_dbg(&client->dev, "%s: Accessing the camera\n", __func__);

	return 0;
}


/**
 * struct v4l2_subdev_internal_ops - v4l2 operations for the device.
 * @open: opening function.
 *
 * Only to register the v4l2 device. the opening function only prints a message.
 */
static const struct v4l2_subdev_internal_ops zedx_subdev_internal_ops = {
	.open = zedx_open,
};


/**
 * zedx_board_setup() - Electrical init of the zedx camera.
 * @priv: Driver data structure.
 *
 * This functions is called when probing the device. Its role is to
 * initialize the eeprom, enable the frame trigger master clock,
 * power on the serializer, deserializer and read the eeprom.
 *
 * Context: Can sleep.
 * Return: 0 in case of success, and a negative errno otherwise.
 */
static int zedx_board_setup(struct zedx *priv)
{
	struct camera_common_data *s_data = priv->s_data;
	struct device *dev = s_data->dev;
	int err = 0;

	if (verbosity_level>=1)
		dev_dbg(dev, "%s++\n", __func__);

	/* eeprom interface */
	err = zedx_power_on(s_data);
	if (err)
	{
		dev_err(dev,"Error %d during power on sensor\n", err);
		return err;
	}

	return 0;
}

/**
 * zedx_probe() - Initialize the zedx camera.
 * @client: i2c adapter structure to fill.
 * @id: i2c device id.
 *
 * This functions is called when loading the module. Its main role
 * is to setup the serializer and the deserializer of the zedx. In
 * addition, it links a bunch of device driver structures, register
 * a tegracam device and a v4l2 device.
 *
 * Context: Can sleep.
 * Return: 0 in case of success, and a negative errno otherwise.
 */
static int zedx_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct device_node *node = dev->of_node;
#if 0
	struct device_node *i2cmux_node, *deser_node ;
#endif
	struct tegracam_device *tc_dev;
	struct zedx *priv;
	const char *str;
	const char *video_name;
	int n_links;
	u8 links[4] = {0,0,0,0};

	int err;
	dev_info(dev, "Driver Version : v%d.%d.%d\n",ZEDX_DRIVER_VERSION_MAJOR,ZEDX_DRIVER_VERSION_MINOR,ZEDX_DRIVER_VERSION_PATCH);
	dev_info(dev, "Probing v4l2 sensor.\n");

	if (!IS_ENABLED(CONFIG_OF) || !node)
		return -EINVAL;

	priv = devm_kzalloc(dev, sizeof(struct zedx), GFP_KERNEL);
	if (!priv)
	{
		dev_err(dev, "unable to allocate memory!\n");
		return -ENOMEM;
	}
	tc_dev = devm_kzalloc(dev,
			sizeof(struct tegracam_device), GFP_KERNEL);
	if (!tc_dev)
		return -ENOMEM;

	err = of_property_read_string(node, "channel", &str);
	if (err)
	{
		dev_err(dev, "channel not found\n");
		return -EINVAL;
	}

	priv->channel = str[0] - 'a';


	err = of_property_read_string(node, "mode", &str);
	if (err)
	{
		dev_err(dev, "Device mode not found\n");
		return -EINVAL;
	}

	if (str[0] == 'm')
	{
		priv->master = true;
	}
	else
	{
		priv->master = false;
	}

	if (verbosity_level>=1)
		dev_dbg(dev, "%s: channel %d\n", __func__, priv->channel);


	err = of_property_read_string(node, "devnode", &video_name);
	if (err)
		dev_err(dev, "devnode not found\n");

	err = of_property_read_u32(node, "sync_sensor_index",
			&priv->sync_sensor_index);
	if (err)
		dev_err(dev, "sync name index not in DT\n");

#if 0
	i2cmux_node = of_get_parent(node);

	if (!i2cmux_node)
	{
		dev_warn(dev, "%s: could not get the i2c mux node.\n", __func__);
	}

	deser_node = of_get_child_by_name(i2cmux_node, "max929x_b");

	if (!deser_node)
	{
		dev_info(dev, "%s: deserializer node not found.\n",__func__);
		of_node_put(i2cmux_node);
		priv->sync_slave = false;
	}
	else
	{
		of_property_read_string(deser_node, "sync_mode", &str);

		if (str[0]=='s')
		{
			dev_info(dev, "%s: deserializer is in slave sync mode.\n", __func__);
			priv->sync_slave = true;
		}
		else
		{
			dev_info(dev, "%s: deserializer is in master sync mode.\n", __func__);
			priv->sync_slave = false;
		}

		of_node_put(deser_node);

		of_node_put(i2cmux_node);

	}
#else
	priv->sync_slave = false;
#endif

	priv->i2c_client = tc_dev->client = client;
	priv->fps = AR0234_DEFAULT_FPS*1000000;

	tc_dev->dev = dev;
	strncpy(tc_dev->name, video_name, sizeof(tc_dev->name));
	tc_dev->dev_regmap_config = &sensor_regmap_config;
	tc_dev->sensor_ops = &zedx_common_ops;
	tc_dev->v4l2sd_internal_ops = &zedx_subdev_internal_ops;
	tc_dev->tcctrl_ops = &zedx_ctrl_ops;

	err = tegracam_device_register(tc_dev);
	if (err)
	{
		dev_err(dev, "tegra camera driver registration failed\n");
		return err;
	}
	priv->tc_dev = tc_dev;
	priv->s_data = tc_dev->s_data;
	priv->subdev = &tc_dev->s_data->subdev;
	tegracam_set_privdata(tc_dev, (void *)priv);

	zedx_power_on(tc_dev->s_data);

	msleep(100);

	if (priv->master)
		slow_reset_Dser(priv->channel);

	n_links = zedx_links_check(priv, links);

	if (n_links == 2 && priv->master)
	{

		/* First, pipe the video streams correctly in the deserializer.
		 * This way, the cables can be plugged in anyway.
		 */
		err = links_dis_Dser(priv->channel);
		err = pipes_01_en_Dser(priv->channel, links[0]);
		err = pipes_23_en_Dser(priv->channel, links[1]);

		/* The following lines alternatively configure both zedx serializers.
		 * It is done one ZED X at a time because the serializers are connected
		 * to the same I2C Control Channel (CC) after reset.
		 * That is a channel reserved to the Serializer/Deserializer.
		 * Even if both serializers are on different I2C buses,
		 * they are both on the same Control Channel, hence they can both be addressed from
		 * one I2C bus, with the same address, breaking everything.
		 * To prevent that, we could change one of the serializer address,
		 * but it brings some difficulties for reset, or we can configure
		 * each serializer sequentially, and we split the control
		 * channel at run time: I2C bus 0 is the CC for the first serializer,
		 * and I2C bus 1 is the CC for the second serializer.
		 */

		/* Configuration of the first GMSL link. */
		printk(KERN_INFO "sl-zedx: checking base");
		err = cc_en_Dser(priv->channel, links[0]);
		err = zedx_write_table(priv, mode_table[AR0234_MODE_BASE_SER]);

		if (err)
        {
			printk(KERN_INFO "sl-zedx: checking proto");
            err = zedx_write_table(priv, mode_table[AR0234_MODE_PROTO_SER]);
            // Configuration of the second GMSL link.
            err = cc_en_Dser(priv->channel, links[1]);
            err = zedx_write_table(priv, mode_table[AR0234_MODE_PROTO_SER]);
        }
		
		else{
			// Configuration of the second GMSL link.
			err = cc_en_Dser(priv->channel, links[1]);
			err = zedx_write_table(priv, mode_table[AR0234_MODE_BASE_SER]);
		}

		/* Now that the serializers are set, we setup the separate CC */
		err = i2c_setup_Dser(priv->channel, links);


		if (err)
		{
			dev_info(&client->dev, "Multiple ZED-X camera detect error\n");
			tegracam_device_unregister(tc_dev);
			return err;
		}
		else
		{
			dev_info(&client->dev, "Multiple ZED-X camera detect success\n");
		}
	}
	else if (n_links == 1 && priv->master)
	{
		/* On the Deserializer side of things: */
		/* First disable all GMSL links */
		err = links_dis_Dser(priv->channel);
		/* Enable only the correct pipe, according to the links_check return params. */
		err = pipes_01_en_Dser(priv->channel, links[0]);

		/* On the Serializer side of things: */
		/* Configure the Serializer */
		printk(KERN_INFO "sl-zedx: checking base");
		err = zedx_write_table(priv, mode_table[AR0234_MODE_BASE_SER]);
		if(err){
			printk(KERN_INFO "sl-zedx: checking proto");
			err = zedx_write_table(priv, mode_table[AR0234_MODE_PROTO_SER]);
		}		


		if (err)
		{
			dev_info(&client->dev, "Single ZED-X detect error\n");
			return err;
		}
		else
		{
			dev_info(&client->dev, "Single ZED-X camera detect success\n");;
		}
	}
	else if (n_links > 0 && !priv->master)
	{
		/* If it is a slave, nothing to do */
		if (verbosity_level>=1)
			dev_info(&client->dev, "Slave ar0234 detected\n");
	}
	else
	{
		tegracam_device_unregister(tc_dev);
		return -1;
	}

	err = zedx_write_table(priv, mode_table[AR0234_MODE_STOP_STREAM]);
	// should work as is -> check zedx link check for more compat info, especially about used port
	if (err)
	{
		dev_info(&client->dev, "ZED-X detect error\n");
		tegracam_device_unregister(tc_dev);
		return err;
	}

	msleep(100);
	err = zedx_board_setup(priv);
	if (err)
	{
		dev_err(dev, "board setup failed\n");
		tegracam_device_unregister(tc_dev);
		return err;
	}

	err = tegracam_v4l2subdev_register(tc_dev, true);
	if (err)
	{
		dev_err(dev, "tegra camera subdev registration failed\n");
		tegracam_device_unregister(tc_dev);
		return err;
	}

	dev_info(&client->dev, "Detected ZED-X sensor\n");
	return 0;
}

/**
 * zedx_remove() - Remove the zedx camera.
 * @client: i2c adapter structure to remove.
 *
 * This functions is called when discharging the module. Its main role
 * is to unregister the v4l2 and tegracam devices. Otherwise, the memory
 * is freed automatically thanks to the devmkzalloc() used in the probing
 * function.
 *
 * Context: Can sleep.
 * Return: 0 in case of success.
 */
static int zedx_remove(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct camera_common_data *s_data = to_camera_common_data(&client->dev);
	struct zedx *priv = (struct zedx *)s_data->priv;
	tegracam_v4l2subdev_unregister(priv->tc_dev);
	tegracam_device_unregister(priv->tc_dev);
	if (verbosity_level>=1)
		dev_info(dev, " ZED-X sensor successfully removed\n");
	return 0;
}

/**
 * struct i2c_device_id - Name and ID of the i2c device.
 *  The id should be the same as the one in the dts module part.
 */
static const struct i2c_device_id zedx_id[] = {
	{"zedx", 0},
	{}};

MODULE_DEVICE_TABLE(i2c, zedx_id);

/**
 * struct i2c_driver - i2c driver functions
 * Here the name should be the same as the first part of the V4L2 devname
 * and the first part of the badge in the dts.
 * For more infos, please refer to nvidia's documentation
 */
static struct i2c_driver zedx_i2c_driver = {
	.driver = {
		.name = "zedx",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(zedx_of_match),
	},
	.probe = zedx_probe,
	.remove = zedx_remove,
	.id_table = zedx_id,
};

/* Register the i2c driver in the kernel */
module_i2c_driver(zedx_i2c_driver);

MODULE_DESCRIPTION("Media Controller driver for Stereolabs ZED-X");
MODULE_AUTHOR("Stereolabs");
MODULE_AUTHOR("Louis Guerlin <support@stereolabs.com");
MODULE_LICENSE("GPL v2");
