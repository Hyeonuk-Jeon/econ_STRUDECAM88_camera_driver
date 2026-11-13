// SPDX-License-Identifier: GPL-2.0

#if defined(CONFIG_ARCH_TEGRA_186_SOC) ||                                      \
	defined(CONFIG_ARCH_TEGRA_194_SOC) ||                                  \
	defined(CONFIG_ARCH_TEGRA_210_SOC) ||                                  \
	defined(CONFIG_ARCH_TEGRA_234_SOC)
#define NVIDIA_BUILD
#endif

#include <linux/ctype.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/media-bus-format.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_graph.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/uaccess.h>
#include <linux/version.h>
#include <linux/v4l2-mediabus.h>
#include <media/v4l2-async.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#ifdef NVIDIA_BUILD
#include <media/camera_common.h>
#endif
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 13, 0)
#include <media/v4l2-fwnode.h>
#else
#include <media/v4l2-of.h>
#endif
#include <media/v4l2-subdev.h>
#include "basler-camera-driver.h"

#ifdef NVIDIA_BUILD
#define DRIVER_VERSION	"1.6.3-nvidia"
#else
#define DRIVER_VERSION	"1.6.3"
#endif
#define SENSOR_NAME	"basler-camera"

/*
 * ABRM register offsets
 *
 */
#define ABRM_GENCP_VERSION		0x0
#define ABRM_MANUFACTURER_NAME		0x4
#define ABRM_MODEL_NAME			0x44
#define ABRM_FAMILY_NAME		0x84
#define ABRM_DEVICE_VERSION		0xC4
#define ABRM_MANUFACTURER_INFO		0x104
#define ABRM_SERIAL_NUMBER		0x144
#define ABRM_USER_DEFINED_NAME		0x184
#define ABRM_DEVICE_CAPABILITIES	0x1C4

/*
 * ABRM register bits
 *
 */
#define ABRM_DEVICE_CAPABILITIES_USER_DEFINED_NAMES_SUPPORT	0x1
#define ABRM_DEVICE_CAPABILITIES_STRING_ENCODING		0x0f
#define ABRM_DEVICE_CAPABILITIES_FAMILY_NAME			0x100


/*
 * Maximum read i2c burst
 *
 * TODO: To be replace by a register call of SBRM
 *
 */
#define I2C_MAXIMUM_READ_BURST	8

static int basler_write_register_chunk(struct i2c_client *client, u8 *buffer,
				      u16 buffer_size);
static int basler_read_register_chunk(struct i2c_client *client, u8 *buffer,
				      u16 buffer_size, __be16 register_address);

static int basler_camera_s_ctrl(struct v4l2_ctrl *ctrl);
static int basler_camera_g_volatile_ctrl(struct v4l2_ctrl *ctrl);
static int basler_camera_validate(const struct v4l2_ctrl *ctrl, u32 idx,
				  union v4l2_ctrl_ptr ptr);
static void basler_camera_init(const struct v4l2_ctrl *ctrl, u32 idx,
			       union v4l2_ctrl_ptr ptr);
static bool basler_camera_equal(const struct v4l2_ctrl *ctrl, u32 idx,
				union v4l2_ctrl_ptr ptr1,
				union v4l2_ctrl_ptr ptr2);

#ifdef NVIDIA_BUILD
static const int basler_30fps[] = {
	30,
};

/*
 * WARNING: frmfmt ordering need to match mode definition in
 * device tree!
 */
static const struct camera_common_frmfmt basler_frmfmt[] = {
	/* size, framerates, num framerates, hdr, mode */
	{{1920, 1080}, basler_30fps, 1, 0, 0}
	/* Add modes with no device tree support after below */
};
#endif

struct basler_camera_dev {
	struct i2c_client *i2c_client;
	struct v4l2_subdev* sd;
	struct media_pad pad;
	struct v4l2_mbus_framefmt format;

	/* lock to protect all members below */
	struct mutex lock;

	int power_count;
	struct v4l2_ctrl_handler ctrl_handler;

	struct basler_device_information device_information;

	/* Storage for register address and data size for register reads */
	struct register_access ra_tmp;

	struct regulator *vdd;

#ifdef NVIDIA_BUILD
	struct camera_common_data *s_data;
#else
	struct v4l2_subdev sd_;
#endif
};

/**
 * basler_write_burst - issue a burst I2C message in master transmit mode
 * @client: Handle to slave device
 * @ra_p: Data structure that hold the register address and data that will be
 *	written to the slave
 *
 * Returns the number of bytes written on success or a negative error code.
 */
static int basler_write_burst(struct basler_camera_dev *sensor,
			      struct register_access *ra_p)
{
	struct i2c_client *client = sensor->i2c_client;
	int ret;
	u16 old_address;

	if (ra_p->data_size > sizeof(ra_p->data)) {
		dev_err(&client->dev,
			"i2c burst array too big, max allowed %zu, got %d\n",
			sizeof(ra_p->data), ra_p->data_size);
		return -EINVAL;
	}

	old_address = ra_p->address;
	ra_p->address = cpu_to_be16(ra_p->address);

	if (I2CREAD == (ra_p->command | I2CREAD)) {
		sensor->ra_tmp.address = ra_p->address;
		sensor->ra_tmp.data_size = ra_p->data_size;

		/* restore original address */
		ra_p->address = old_address;

		return ra_p->data_size;
	}

	if (I2CWRITE == (ra_p->command | I2CWRITE)) {
		ret = basler_write_register_chunk(client, (u8 *)ra_p,
				ra_p->data_size + sizeof(ra_p->address));
		if (ret < 0)
			return ret;

		ra_p->data_size = ret;

		/* restore original address */
		ra_p->address = old_address;

		return ret;
	}

	return -EPERM;
}

static int basler_write_register_chunk(struct i2c_client *client, u8 *buffer,
				       u16 buffer_size)
{
	int ret;

	ret = i2c_master_send(client, buffer, buffer_size);
	if (ret < 0)
		return ret;

	if (ret != buffer_size)
		return -EIO;

	return ret;
}

/**
 * basler_read_burst - issue a burst I2C message in master transmit mode
 * @client: Handle to slave device
 * @ra_p: Data structure store the data read from slave
 *
 * Note: Before data can read use basler_write_burst with read command
 *       to send the register address
 *
 * Returns the number of bytes read on success or a negative error code.
 */
static int basler_read_burst(struct basler_camera_dev *sensor,
			     struct register_access *ra_p)
{
	struct i2c_client *client = sensor->i2c_client;
	int ret;

	ret = basler_read_register_chunk(client, ra_p->data,
			sensor->ra_tmp.data_size, sensor->ra_tmp.address);
	if (ret < 0)
		ra_p->data_size = 0;
	else
		ra_p->data_size = ret;

	return ret;
}

static int basler_read_register_chunk(struct i2c_client *client, u8 *buffer,
				      u16 buffer_size, __be16 register_address)
{
	struct i2c_msg msgs[2] = {};
	int ret = 0;

	msgs[0].addr = client->addr;
	msgs[0].flags = 0;
	msgs[0].buf = (u8 *)&register_address;
	msgs[0].len = sizeof(register_address);

	msgs[1].addr = client->addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].buf = buffer;
	msgs[1].len = buffer_size;

	ret = i2c_transfer(client->adapter, msgs, 2);
	if (ret < 0) {
		pr_err("i2c_transfer() failed: %d\n", ret);
		return ret;
	}

	if (ret != 2) {
		pr_err("i2c_transfer() incomplete");
		return -EIO;
	}

	return msgs[1].len;
}

static int basler_read_register(struct i2c_client *client, u8 *buffer,
				u8 buffer_size, u16 register_address)
{
	int ret = 0;
	u8 l_read_bytes = 0;

	do {
		__be16 l_register_address =
			cpu_to_be16(register_address + l_read_bytes);

		ret = basler_read_register_chunk(
			client, (u8 *)buffer + l_read_bytes,
			(u16)min(I2C_MAXIMUM_READ_BURST,
				  ((int)buffer_size - l_read_bytes)),
			l_register_address);
		if (ret < 0) {
			pr_err("basler_read_register_chunk() failed: %d\n",
			       ret);
			return ret;
		}

		if (ret == 0) {
			pr_err("basler_read_register_chunk() read 0 bytes.\n");
			return -EIO;
		}

		l_read_bytes = l_read_bytes + ret;
	} while (l_read_bytes < buffer_size);

	return l_read_bytes;
}

static int
basler_retrieve_device_information(struct i2c_client *client,
				   struct basler_device_information *bdi)
{
	int ret = 0;
	u64 deviceCapabilities = 0;
	u64 stringEncoding = 0;
	__be64 deviceCapabilitiesBe = 0;
	__be32 gencpVersionBe = 0;

	bdi->_magic = BDI_MAGIC;

	ret = basler_read_register(client, (u8 *)&deviceCapabilitiesBe,
				   sizeof(deviceCapabilitiesBe),
				   ABRM_DEVICE_CAPABILITIES);
	if (ret < 0) {
		pr_err("basler_read_register() failed: %d\n", ret);
		return ret;
	}

	if (ret != sizeof(deviceCapabilitiesBe)) {
		pr_err("basler_read_register() not read full buffer = %d bytes\n",
		       ret);
		return -EIO;
	}

	deviceCapabilities = be64_to_cpu(deviceCapabilitiesBe);
	stringEncoding = (deviceCapabilities &
			  ABRM_DEVICE_CAPABILITIES_STRING_ENCODING) >> 4;
	pr_debug("deviceCapabilities = 0x%llx\n", deviceCapabilities);
	pr_debug("String Encoding = 0x%llx\n", stringEncoding);

	ret = basler_read_register(client, (u8 *)&gencpVersionBe,
				   sizeof(gencpVersionBe), ABRM_GENCP_VERSION);
	if (ret < 0) {
		pr_err("basler_read_register() failed: %d\n", ret);
		return ret;
	}

	if (ret != sizeof(gencpVersionBe)) {
		pr_err("basler_read_register() not read full buffer = %d bytes\n",
		       ret);
		return -EIO;
	}

	bdi->gencpVersion = be32_to_cpu(gencpVersionBe);
	pr_debug("l_gencpVersion = %d.%d\n",
		 (bdi->gencpVersion & 0xffff0000) >> 16,
		 bdi->gencpVersion & 0xffff);

	ret = basler_read_register(client, bdi->deviceVersion,
				   GENCP_STRING_BUFFER_SIZE,
				   ABRM_DEVICE_VERSION);
	if (ret < 0) {
		pr_err("basler_read_register() failed: %d\n", ret);
		return ret;
	}

	if (ret != GENCP_STRING_BUFFER_SIZE) {
		pr_err("basler_read_register() not read full buffer = %d bytes\n",
		       ret);
		return -EIO;
	}

	pr_debug("bdi->deviceVersion = %s\n", bdi->deviceVersion);

	ret = basler_read_register(client, bdi->serialNumber,
				   GENCP_STRING_BUFFER_SIZE,
				   ABRM_SERIAL_NUMBER);
	if (ret < 0) {
		pr_err("basler_read_register() failed: %d\n", ret);
		return ret;
	}

	if (ret != GENCP_STRING_BUFFER_SIZE) {
		pr_err("basler_read_register() not read full buffer = %d bytes\n",
		       ret);
		return -EIO;
	}

	pr_debug("bdi->serialNumber = %s\n", bdi->serialNumber);

	ret = basler_read_register(client, bdi->manufacturerName,
				   GENCP_STRING_BUFFER_SIZE,
				   ABRM_MANUFACTURER_NAME);
	if (ret < 0) {
		pr_err("basler_read_register() failed: %d\n", ret);
		return ret;
	}

	if (ret != GENCP_STRING_BUFFER_SIZE) {
		pr_err("basler_read_register() not read full buffer = %d bytes\n",
		       ret);
		return -EIO;
	}

	pr_debug("bdi->manufacturerName = %s\n", bdi->manufacturerName);

	ret = basler_read_register(client, bdi->modelName,
				   GENCP_STRING_BUFFER_SIZE, ABRM_MODEL_NAME);
	if (ret < 0) {
		pr_err("basler_read_register() failed: %d\n", ret);
		return ret;
	}

	if (ret != GENCP_STRING_BUFFER_SIZE) {
		pr_err("basler_read_register() not read full buffer = %d bytes\n",
		       ret);
		return -EIO;
	}

	pr_debug("bdi->modelName = %s\n", bdi->modelName);

	if (deviceCapabilities & ABRM_DEVICE_CAPABILITIES_FAMILY_NAME) {
		ret = basler_read_register(client, bdi->familyName,
					   GENCP_STRING_BUFFER_SIZE,
					   ABRM_FAMILY_NAME);
		if (ret < 0) {
			pr_err("basler_read_register() failed: %d\n", ret);
			return ret;
		}

		if (ret != GENCP_STRING_BUFFER_SIZE) {
			pr_err("basler_read_register() not read full buffer = %d bytes\n",
			       ret);
			return -EIO;
		}

		pr_debug("bdi->familyName = %s\n", bdi->familyName);
	} else
		pr_notice("ABRM FamilyName not supported\n");

	if (deviceCapabilities &
	    ABRM_DEVICE_CAPABILITIES_USER_DEFINED_NAMES_SUPPORT) {
		ret = basler_read_register(client, bdi->userDefinedName,
					   GENCP_STRING_BUFFER_SIZE,
					   ABRM_USER_DEFINED_NAME);
		if (ret < 0) {
			pr_err("basler_read_register() failed: %d\n", ret);
			return ret;
		}

		if (ret != GENCP_STRING_BUFFER_SIZE) {
			pr_err("basler_read_register() not read full buffer = %d bytes\n",
			       ret);
			return -EIO;
		}

		pr_debug("bdi->userDefinedName = %s\n", bdi->userDefinedName);
	} else
		pr_notice("ABRM UserDefinedName not supported\n");

	ret = basler_read_register(client, bdi->manufacturerInfo,
				   GENCP_STRING_BUFFER_SIZE,
				   ABRM_MANUFACTURER_INFO);
	if (ret < 0) {
		pr_err("basler_read_register() failed: %d\n", ret);
		return ret;
	}

	if (ret != GENCP_STRING_BUFFER_SIZE) {
		pr_err("basler_read_register() not read full buffer = %d bytes\n",
		       ret);
		return -EIO;
	}

	pr_debug("bdi->manufacturerInfo = %s\n", bdi->manufacturerInfo);

	/*
	 * If the strings are in ASCII - print it.
	 */
	if (stringEncoding == 0) {
		pr_info("ABRM: Manufacturer: %s, Model: %s, Device: %s, Serial: %s\n",
			bdi->manufacturerName, bdi->modelName,
			bdi->deviceVersion, bdi->serialNumber);
	}

	return 0;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 20, 0) // since 4.20.0-rc1
static int basler_retrieve_csi_information(struct basler_camera_dev *sensor,
					   struct basler_csi_information *bci)
{
	struct device *dev = &sensor->i2c_client->dev;
	struct device_node *ep;
	struct v4l2_fwnode_endpoint bus_cfg = {
		.bus_type = V4L2_MBUS_CSI2_DPHY,
	};
	int ret = 0;
	unsigned short i;

	/* We need a function that searches for the device that holds
	 * the csi-2 bus information. For now we put the bus information
	 * also into the sensor endpoint itself.
	 */
	ep = of_graph_get_next_endpoint(dev->of_node, NULL);
	if (!ep) {
		dev_err(dev, "missing endpoint node\n");
		return -ENODEV;
	}

	ret = v4l2_fwnode_endpoint_alloc_parse(of_fwnode_handle(ep), &bus_cfg);
	of_node_put(ep);
	if (ret) {
		dev_err(dev, "failed to parse endpoint\n");
		goto done;
	}

	if (bus_cfg.bus_type != V4L2_MBUS_CSI2_DPHY ||
	    bus_cfg.bus.mipi_csi2.num_data_lanes == 0 ||
	    bus_cfg.nr_of_link_frequencies == 0) {
		dev_err(dev, "missing CSI-2 properties in endpoint\n");
		ret = -ENODATA;
		goto done;
	}

	bci->max_lane_frequency = bus_cfg.link_frequencies[0];
	bci->lane_count = bus_cfg.bus.mipi_csi2.num_data_lanes;
	for (i = 0; i < bus_cfg.bus.mipi_csi2.num_data_lanes; ++i)
		bci->lane_assignment[i] = bus_cfg.bus.mipi_csi2.data_lanes[i];

done:
	v4l2_fwnode_endpoint_free(&bus_cfg);
	return ret;
}
#else
static int basler_retrieve_csi_information(struct basler_camera_dev *sensor,
					   struct basler_csi_information *bci)
{
	struct device *dev = &sensor->i2c_client->dev;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 13, 0)
	struct v4l2_fwnode_endpoint *endpoint;
#else
	struct v4l2_of_endpoint *endpoint;
#endif
	struct device_node *ep;
	int ret = 0;
	unsigned short i;

	/* We need a function that searches for the device that holds
	 * the csi-2 bus information. For now we put the bus information
	 * also into the sensor endpoint itself.
	 */
	ep = of_graph_get_next_endpoint(dev->of_node, NULL);
	if (!ep) {
		dev_err(dev, "missing endpoint node\n");
		return -ENODEV;
	}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 13, 0)
	endpoint = v4l2_fwnode_endpoint_alloc_parse(of_fwnode_handle(ep));
	of_node_put(ep);
#else
	endpoint = v4l2_of_alloc_parse_endpoint(ep);
#endif
	if (IS_ERR(endpoint)) {
		dev_err(dev, "failed to parse endpoint\n");
		return PTR_ERR(endpoint);
	}

	if (endpoint->bus_type != V4L2_MBUS_CSI2 ||
	    endpoint->bus.mipi_csi2.num_data_lanes == 0 ||
	    endpoint->nr_of_link_frequencies == 0) {
		dev_err(dev, "missing CSI-2 properties in endpoint\n");
		ret = -ENODATA;
		goto done;
	}

	bci->max_lane_frequency = endpoint->link_frequencies[0];
	bci->lane_count = endpoint->bus.mipi_csi2.num_data_lanes;
	for (i = 0; i < endpoint->bus.mipi_csi2.num_data_lanes; ++i)
		bci->lane_assignment[i] = endpoint->bus.mipi_csi2.data_lanes[i];

done:
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 13, 0)
	v4l2_fwnode_endpoint_free(endpoint);
#else
	v4l2_of_free_endpoint(endpoint);
#endif
	return ret;
}
#endif

static inline struct basler_camera_dev *
to_basler_camera_dev(struct v4l2_subdev *subdev)
{
#ifdef NVIDIA_BUILD
	return to_camera_common_data(subdev->dev)->priv;
#else
	return container_of(subdev, struct basler_camera_dev, sd_);
#endif
}

static inline struct v4l2_subdev *ctrl_to_sd(struct v4l2_ctrl *ctrl)
{
	return container_of(ctrl->handler, struct basler_camera_dev,
			     ctrl_handler)
			->sd;
}

static void basler_camera_set_default_format(struct basler_camera_dev *sensor)
{
	struct v4l2_mbus_framefmt *fmt;

	fmt = &sensor->format;
	fmt->code = MEDIA_BUS_FMT_YUYV8_1X16;
	fmt->colorspace = V4L2_COLORSPACE_SRGB;
	fmt->ycbcr_enc = V4L2_MAP_YCBCR_ENC_DEFAULT(fmt->colorspace);
	fmt->quantization = V4L2_MAP_QUANTIZATION_DEFAULT(true,
							  fmt->colorspace,
							  fmt->ycbcr_enc);
	fmt->xfer_func = V4L2_MAP_XFER_FUNC_DEFAULT(fmt->colorspace);
	fmt->width = 1920;
	fmt->height = 1080;
	fmt->field = V4L2_FIELD_NONE;
}

/**
 * basler_camera_set_fmt - set format of the camera
 *
 * Note: We just store the format so we can answer the `g_fmt` op.
 *       Proper configuration of the camera will be done from user space
 *
 * Returns zero for pad 0 and -EINVAL for all other pads
 */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 14, 0)
static int basler_camera_set_fmt(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state,
				 struct v4l2_subdev_format *format)
#else
static int basler_camera_set_fmt(struct v4l2_subdev *sd,
				 struct v4l2_subdev_pad_config *cfg,
				 struct v4l2_subdev_format *format)
#endif
{
	struct basler_camera_dev *sensor = to_basler_camera_dev(sd);

	if (format->pad)
		return -EINVAL;

	mutex_lock(&sensor->lock);

	sensor->format = format->format;

	mutex_unlock(&sensor->lock);

	return 0;
}

/**
 * basler_camera_get_fmt - Get current format
 *
 * Note: We just return the format from the last `s_fmt` invocation.
 *       Proper configuration of the camera will be done from user space
 *
 * Returns zero for pad 0 and -EINVAL for all other pads
 */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 14, 0)
static int basler_camera_get_fmt(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state,
				 struct v4l2_subdev_format *format)
#else
static int basler_camera_get_fmt(struct v4l2_subdev *sd,
				 struct v4l2_subdev_pad_config *cfg,
				 struct v4l2_subdev_format *format)
#endif
{
	struct basler_camera_dev *sensor = to_basler_camera_dev(sd);

	if (format->pad)
		return -EINVAL;

	mutex_lock(&sensor->lock);

	format->format = sensor->format;

	mutex_unlock(&sensor->lock);

	return 0;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 14, 0)
static int basler_camera_enum_mbus_code(struct v4l2_subdev *sd,
					struct v4l2_subdev_state *state,
					struct v4l2_subdev_mbus_code_enum *code)
#else
static int basler_camera_enum_mbus_code(struct v4l2_subdev *sd,
					struct v4l2_subdev_pad_config *cfg,
					struct v4l2_subdev_mbus_code_enum *code)
#endif
{
	static uint16_t supported_datatypes[] = {
		MEDIA_BUS_FMT_RGB888_1X24, // RGB888
		MEDIA_BUS_FMT_UYVY8_1X16, // YUV422 8-bit
		MEDIA_BUS_FMT_YUYV8_2X8, // YUV422 8-bit
		MEDIA_BUS_FMT_SRGGB8_1X8, // RAW8
		MEDIA_BUS_FMT_SRGGB10_1X10, // RAW10
		MEDIA_BUS_FMT_SRGGB12_1X12 // RAW12
	};

	if (code->index >= ARRAY_SIZE(supported_datatypes))
		return -EINVAL;

	code->code = supported_datatypes[code->index];

	v4l2_info(sd, "%s: index=%u -> code=%04X\n", __func__, code->index,
		  code->code);

	return 0;
}

/*
 * basler_camera_enum_framesizes() - returns the supported frame sizes by index.
 * @sd  the v4l2 subdevice
 * @cfg the pad configuration
 * @fse the frame size returned.
 *
 * Return 0 on success and an error otherwise.
 */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 14, 0)
static int
basler_camera_enum_framesizes(struct v4l2_subdev *sd,
			      struct v4l2_subdev_state *state,
			      struct v4l2_subdev_frame_size_enum *fse)
#else
static int
basler_camera_enum_framesizes(struct v4l2_subdev *sd,
			      struct v4l2_subdev_pad_config *cfg,
			      struct v4l2_subdev_frame_size_enum *fse)
#endif
{
	v4l2_info(sd, "%s (...)\n", __func__);

#if 0
	if (fse->code != V4L2_PIX_FMT_UYVY &&
		fse->code != V4L2_PIX_FMT_SRGGB8 &&
		fse->code != V4L2_PIX_FMT_ABGR32 &&
		fse->code != V4L2_PIX_FMT_GREY)
		return -EINVAL;
#endif
	if (fse->index >= 1)
		return -EINVAL;

	fse->min_width = 4;
	fse->min_height = 4;
	fse->max_width = 5000;
	fse->max_height = 4000;

	v4l2_info(sd, "%s (...) => %d/%d .. %d/%d\n", __func__, fse->min_width,
		  fse->min_height, fse->max_width, fse->max_height);

	return 0;
}

/**
 * basler_camera_s_stream - start camera streaming
 *
 * Note: Will be done in user space
 *
 * Returns always zero
 */
static int basler_camera_s_stream(struct v4l2_subdev *sd, int enable)
{
	return 0;
}

static int basler_camera_s_power(struct v4l2_subdev *sd, int on)
{
	struct basler_camera_dev *sensor = to_basler_camera_dev(sd);

	mutex_lock(&sensor->lock);

	/* Update the power count. */
	sensor->power_count += on ? 1 : -1;
	WARN_ON(sensor->power_count < 0);

	mutex_unlock(&sensor->lock);

	return 0;
}

static long basler_camera_priv_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct basler_camera_dev *sensor = to_basler_camera_dev(sd);
	int ret;

	switch (cmd) {
	case BASLER_IOC_G_INTERFACE_VERSION:
	{
		__u32 __user *uptr = (__u32 __user *)arg;

		__u32 version = (((__u32)BASLER_INTERFACE_VERSION_MAJOR) << 16) | BASLER_INTERFACE_VERSION_MINOR;
		ret = put_user(version, uptr);
		if (ret)
			return -EFAULT;

		return 0;
	}

	case BASLER_IOC_READ_REGISTER:
	{
		struct register_access __user *ra_p = (struct register_access __user *)arg;
		struct register_access ra;
		u8 buf[BASLER_REGISTER_ACCESS_BUF_SIZE];
		__be16 reg_addr;

		ret = copy_from_user(&ra, ra_p, sizeof(struct register_access));
		if (ret)
			return -EFAULT;

		reg_addr = cpu_to_be16(ra.address);

		ret = basler_read_register_chunk(client, buf, ra.data_size, reg_addr);
		if (ret < 0)
			return ret;

		ret = copy_to_user(ra_p->data, buf, ra.data_size);
		if (ret)
			return -EFAULT;

		return 0;
	}

	case BASLER_IOC_WRITE_REGISTER:
	{
		struct register_access __user *ra_p = (struct register_access __user *)arg;
		struct register_access ra;

		ret = copy_from_user(&ra, ra_p, sizeof(struct register_access));
		if (ret)
			return -EFAULT;

		ra.address = cpu_to_be16(ra.address);

		ret = basler_write_register_chunk(client, (u8 *)&ra, ra.data_size + sizeof(ra.address));
		if (ret < 0)
			return ret;

		return 0;
	}

	case BASLER_IOC_G_DEVICE_INFORMATION:
	{
		struct basler_device_information __user *uptr = (struct basler_device_information __user *)arg;

		ret = copy_to_user(uptr, &sensor->device_information, sizeof(struct basler_device_information));
		if (ret)
			return -EFAULT;

		return 0;
	}

	case BASLER_IOC_G_CSI_INFORMATION:
	{
		struct basler_csi_information __user *uptr = (struct basler_csi_information __user *)arg;
		struct basler_csi_information csi_info;

		ret = basler_retrieve_csi_information(sensor, &csi_info);
		if (ret < 0)
			return ret;

		ret = copy_to_user(uptr, &csi_info, sizeof(struct basler_csi_information));
		if (ret)
			return -EFAULT;

		return 0;
	}

	default:
		break;
	}

	return -EINVAL;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 10, 0)
static int basler_camera_g_mbus_config(struct v4l2_subdev *sd,
				       unsigned int pad,
				       struct v4l2_mbus_config *cfg)
#else
static int basler_camera_g_mbus_config(struct v4l2_subdev *sd,
				       struct v4l2_mbus_config *cfg)
#endif
{

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 20, 0)
	cfg->type = V4L2_MBUS_CSI2_DPHY;
#else
	cfg->type = V4L2_MBUS_CSI2;
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 18, 0)
	cfg->bus.mipi_csi2.num_data_lanes = 2;
	cfg->bus.mipi_csi2.flags = 0;
#else
	cfg->flags = V4L2_MBUS_CSI2_CONTINUOUS_CLOCK | V4L2_MBUS_CSI2_CHANNEL_0;
	// cfg->flags |= V4L2_MBUS_CSI2_1_LANE;
	cfg->flags |= V4L2_MBUS_CSI2_2_LANE;
	//cfg->flags |= V4L2_MBUS_CSI2_3_LANE;
	//cfg->flags |= V4L2_MBUS_CSI2_4_LANE;
#endif

	return 0;
}

static const struct v4l2_subdev_core_ops basler_camera_core_ops = {
	.s_power = basler_camera_s_power,
	.ioctl = basler_camera_priv_ioctl,
};

static const struct v4l2_subdev_video_ops basler_camera_video_ops = {
	.s_stream = basler_camera_s_stream,
#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 10, 0)
	.g_mbus_config = basler_camera_g_mbus_config,
#endif
};

static const struct v4l2_subdev_pad_ops basler_camera_pad_ops = {
	.set_fmt = basler_camera_set_fmt,
	.get_fmt = basler_camera_get_fmt,
	.enum_mbus_code = basler_camera_enum_mbus_code,
	.enum_frame_size = basler_camera_enum_framesizes,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 10, 0)
	.get_mbus_config = basler_camera_g_mbus_config,
#endif
};

static const struct v4l2_subdev_ops basler_camera_subdev_ops = {
	.core = &basler_camera_core_ops,
	.video = &basler_camera_video_ops,
	.pad = &basler_camera_pad_ops,
};

static const struct v4l2_ctrl_ops basler_camera_ctrl_ops = {
	.g_volatile_ctrl = basler_camera_g_volatile_ctrl,
	.s_ctrl = basler_camera_s_ctrl,
};

static const struct v4l2_ctrl_type_ops basler_camera_ctrl_type_ops = {
	.validate = basler_camera_validate,
	.init = basler_camera_init,
	.equal = basler_camera_equal,
};

/**
 * basler_camera_validate
 *
 * Note: Not needed by access-register control
 *
 * Returns always zero
 */
static int basler_camera_validate(const struct v4l2_ctrl *ctrl, u32 idx,
				  union v4l2_ctrl_ptr ptr)
{
	return 0;
}

/**
 * basler_camera_init
 *
 * Note: Not needed by access-register control
 *
 */
static void basler_camera_init(const struct v4l2_ctrl *ctrl, u32 idx,
			       union v4l2_ctrl_ptr ptr)
{
}

/**
 * basler_camera_equal
 *
 * Note: Not needed by access-register control
 *
 * Returns always zero
 */
static bool basler_camera_equal(const struct v4l2_ctrl *ctrl, u32 idx,
				union v4l2_ctrl_ptr ptr1,
				union v4l2_ctrl_ptr ptr2)
{
	return 0;
}

/* We need to define a new V4L2_CTRL_TYPE value that does not collide
 * with an existing one.
 *
 * The basic V4L2_CTRL_TYPE_XXX values are defined in linux/videodev2.h
 * as enum v4l2_ctrl_type, but several other headers extend these
 * definitions by adding more values through #defines. There seems to be
 * no central coordination facility to avoid collisions when doing so.
 * Therefore, the best we can do is selecting some obscure random value,
 * cross our fingers and hope for the best.
 */
#define V4L2_CTRL_TYPE_UNIQUE	(0x370de8ca)

static const struct v4l2_ctrl_config ctrl_access_register = {
	.ops = &basler_camera_ctrl_ops,
	.type_ops = &basler_camera_ctrl_type_ops,
	.id = V4L2_CID_BASLER_ACCESS_REGISTER,
	.name = "basler-access-register",
	.type = V4L2_CTRL_TYPE_UNIQUE,
	.flags = V4L2_CTRL_FLAG_EXECUTE_ON_WRITE | V4L2_CTRL_FLAG_VOLATILE,
	.step = 1,
	.dims = { 1 },
	.elem_size = sizeof(struct register_access),
};

static const struct v4l2_ctrl_config ctrl_basler_device_information = {
	.ops = &basler_camera_ctrl_ops,
	.type_ops = &basler_camera_ctrl_type_ops,
	.id = V4L2_CID_BASLER_DEVICE_INFORMATION,
	.name = "basler-device-information",
	.type = V4L2_CTRL_TYPE_UNIQUE,
	.flags = V4L2_CTRL_FLAG_READ_ONLY | V4L2_CTRL_FLAG_VOLATILE,
	.step = 1,
	.dims = { 1 },
	.elem_size = sizeof(struct basler_device_information),
};

static const struct v4l2_ctrl_config ctrl_basler_interface_version = {
	.ops = &basler_camera_ctrl_ops,
	.type_ops = &basler_camera_ctrl_type_ops,
	.id = V4L2_CID_BASLER_INTERFACE_VERSION,
	.name = "basler-interface-version",
	.type = V4L2_CTRL_TYPE_INTEGER,
	.flags = V4L2_CTRL_FLAG_READ_ONLY,
	.min = 0x0,
	.max = 0xffffffff,
	.def = (BASLER_INTERFACE_VERSION_MAJOR << 16) |
	       BASLER_INTERFACE_VERSION_MINOR,
	.step = 1,
};

static const struct v4l2_ctrl_config ctrl_basler_csi_information = {
	.ops = &basler_camera_ctrl_ops,
	.type_ops = &basler_camera_ctrl_type_ops,
	.id = V4L2_CID_BASLER_CSI_INFORMATION,
	.name = "basler-csi-information",
	.type = V4L2_CTRL_TYPE_UNIQUE,
	.flags = V4L2_CTRL_FLAG_READ_ONLY | V4L2_CTRL_FLAG_VOLATILE,
	.step = 1,
	.dims = { 1 },
	.elem_size = sizeof(struct basler_csi_information),
};

static int basler_camera_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct v4l2_subdev *sd = ctrl_to_sd(ctrl);
	struct basler_camera_dev *sensor = to_basler_camera_dev(sd);
	int ret;
	struct register_access *fp_ra_new;

	switch (ctrl->id) {
	case V4L2_CID_BASLER_ACCESS_REGISTER:

		if (ctrl->elem_size != sizeof(struct register_access)) {
			dev_err(sd->dev,
				"%s: size mismatch V4L2_CID_BASLER_ACCESS_REGISTER: elem = %u, expected = %zu",
				__func__, ctrl->elem_size, sizeof(struct register_access));
			return -ENOMEM;
		}

		fp_ra_new = (struct register_access *)ctrl->p_new.p;
		ret = basler_write_burst(sensor, fp_ra_new);
		if (ret < 0)
			return -EIO;

		break;

	default:
		return -EINVAL;
	}

	return 0;
}

static int
basler_camera_g_ctrl_access_register(struct device *dev,
				     struct basler_camera_dev *sensor,
				     const struct v4l2_ctrl *ctrl)
{
	struct register_access *fp_ra_new = NULL;
	int ret;

	if (ctrl->elem_size != sizeof(struct register_access)) {
		dev_err(dev,
			"%s: size mismatch V4L2_CID_BASLER_ACCESS_REGISTER: elem = %u, expected = %zu",
			__func__, ctrl->elem_size,
			sizeof(struct register_access));
		return -ENOMEM;
	}

	fp_ra_new = (struct register_access *)ctrl->p_new.p;
	ret = basler_read_burst(sensor, fp_ra_new);
	if (ret < 0)
		return ret;

	if (ret == 0)
		return -EIO;

	return 0;
}

static int
basler_camera_g_ctrl_device_information(struct device *dev,
					struct basler_camera_dev *sensor,
					const struct v4l2_ctrl *ctrl)
{
	struct basler_device_information *l_bdi = NULL;

	if (ctrl->elem_size != sizeof(struct basler_device_information)) {
		dev_err(dev,
			"%s: size mismatch V4L2_CID_BASLER_DEVICE_INFORMATION: elem = %u, expected = %zu",
			__func__, ctrl->elem_size,
			sizeof(struct basler_device_information));
		return -ENOMEM;
	}

	l_bdi = (struct basler_device_information *)ctrl->p_new.p;
	memcpy(l_bdi, &sensor->device_information,
	       sizeof(struct basler_device_information));

	return 0;
}

static int
basler_camera_g_ctrl_csi_information(struct device *dev,
				     struct basler_camera_dev *sensor,
				     const struct v4l2_ctrl *ctrl)
{
	struct basler_csi_information *l_bci = NULL;
	int ret;

	if (ctrl->elem_size != sizeof(struct basler_csi_information)) {
		dev_err(dev,
			"%s: size mismatch V4L2_CID_BASLER_CSI_INFORMATION: elem = %u, expected = %zu",
			__func__, ctrl->elem_size,
			sizeof(struct basler_csi_information));
		return -ENOMEM;
	}

	l_bci = (struct basler_csi_information *)ctrl->p_new.p;
	ret = basler_retrieve_csi_information(sensor, l_bci);
	if (ret < 0)
		return ret;

	return 0;
}

static int basler_camera_g_volatile_ctrl(struct v4l2_ctrl *ctrl)
{
	struct v4l2_subdev *sd = ctrl_to_sd(ctrl);
	struct device *dev = sd->dev;
	struct basler_camera_dev *sensor = to_basler_camera_dev(sd);

	switch (ctrl->id) {
	case V4L2_CID_BASLER_ACCESS_REGISTER:
		return basler_camera_g_ctrl_access_register(dev, sensor, ctrl);
	case V4L2_CID_BASLER_DEVICE_INFORMATION:
		return basler_camera_g_ctrl_device_information(dev, sensor, ctrl);
	case V4L2_CID_BASLER_CSI_INFORMATION:
		return basler_camera_g_ctrl_csi_information(dev, sensor, ctrl);
	default:
		break;
	}

	return -EINVAL;
}

/**
 * basler_camera_link_setup
 *
 * Note: Function is needed by imx8qm
 *
 * Returns always zero
 */
static int basler_camera_link_setup(struct media_entity *entity,
				    const struct media_pad *local,
				    const struct media_pad *remote, u32 flags)
{
	return 0;
}

static const struct media_entity_operations basler_camera_sd_media_ops = {
	.link_setup = basler_camera_link_setup,
};

static int basler_camera_alloc_control(struct v4l2_ctrl_handler *hdl,
				       const struct v4l2_ctrl_config *cfg,
				       struct device *dev)
{
	struct v4l2_ctrl *ctrl;
	int ret = 0;

	ctrl = v4l2_ctrl_new_custom(hdl, cfg, NULL);
	if (ctrl == NULL) {
		dev_err(dev,
			"Registering ctrl %s failed: %d\n",
			cfg->name,
			hdl->error);
		ret = hdl->error;
	}

	return ret;
}

static int basler_camera_init_controls(struct basler_camera_dev *sensor)
{
	struct v4l2_ctrl_handler *hdl = &sensor->ctrl_handler;

	v4l2_ctrl_handler_init(hdl, 32);

	/* we can use our own mutex for the ctrl lock */
	hdl->lock = &sensor->lock;

	if (basler_camera_alloc_control(hdl, &ctrl_access_register, &sensor->i2c_client->dev) ||
	    basler_camera_alloc_control(hdl, &ctrl_basler_device_information, &sensor->i2c_client->dev) ||
	    basler_camera_alloc_control(hdl, &ctrl_basler_interface_version, &sensor->i2c_client->dev) ||
	    basler_camera_alloc_control(hdl, &ctrl_basler_csi_information, &sensor->i2c_client->dev)) {
		/* free up all controls allocated so far */
		v4l2_ctrl_handler_free(hdl);
		dev_dbg(sensor->sd->v4l2_dev->dev, "%s: ctrl setup error.\n",
			__func__);
		return hdl->error;
	}

	sensor->sd->ctrl_handler = hdl;
	return 0;
}

#ifdef NVIDIA_BUILD
static int get_num_of_sensor_modes_in_dt(struct device *dev)
{
	char temp_str[256];
	struct device_node *node = NULL;
	int num_modes = 0;
	int err = 0;

	while (true) {
		err = snprintf(temp_str, sizeof(temp_str), "mode%d", num_modes);
		if (err < 0)
			break;
		node = of_get_child_by_name(dev->of_node, temp_str);
		of_node_put(node);
		if (node == NULL)
			break;
		num_modes++;
	}
	dev_dbg(dev, "get_num_of_sensor_modes: %i\n", num_modes);
	return num_modes;
}

static bool node_has_property(struct device_node *node,
			      const char *property_name)
{
	const char *unused;
	if (of_property_read_string(node, property_name, &unused)) {
		return false;
	}
	return true;
}

static struct property *create_property(const char *name, const char *value)
{
	struct property *prop;
	prop = kzalloc(sizeof(*prop), GFP_KERNEL);
	if (!prop)
		return NULL;

	prop->name = kstrdup_const(name, GFP_KERNEL);
	prop->value = kstrdup_const(value, GFP_KERNEL);
	prop->length = strlen(value) + 1;
	if (!prop->name || !prop->value)
		goto err_free;

	/* mark the property as dynamic */
	of_property_set_flag(prop, OF_DYNAMIC);
	return prop;
err_free:
	kfree(prop->name);
	kfree(prop->value);
	kfree(prop);
	return NULL;
}

static int add_node_property_to_changeset(struct of_changeset *ocs,
					  struct device_node *node,
					  const char *prop_name,
					  const char *prop_value)
{
	struct property *prop;
	if (node_has_property(node, prop_name)) {
		return -EEXIST;
	}
	prop = create_property(prop_name, prop_value);
	if (!prop) {
		return -EINVAL;
	}
	return of_changeset_add_property(ocs, node, prop);
}
#define BASLER_BUFF_SIZE 20
static int basler_set_devicetree_nodes_for_nvidia_modes(
	struct device *dev, struct basler_csi_information *csi_info)
{
	int ret = 0;
	struct device_node *node = NULL;
	struct of_changeset ocs;
	const int bits_per_pixel = 8;
	const int pixels_per_mipi_clock_tick = 2;
	// The pixel clock is needed to intialize the VI graph within the nvidia kernel.
	// So we can not set this value during runtime (for example set_fmt).
	// nvidia searches through all modes (we only have one) and takes the highest value.
	// we take an upper guess: Mipi lane clock * 2 * number of lanes / bits per pixel
	// for "bits per pixel" we take the smallest possible value which is 8.
	u64 pixel_clock = csi_info->max_lane_frequency *
			  pixels_per_mipi_clock_tick * csi_info->lane_count /
			  bits_per_pixel;
	char char_buffer[BASLER_BUFF_SIZE];

	node = of_get_child_by_name(dev->of_node, "mode0");
	of_node_put(node);
	if (node == NULL) {
		dev_err(dev,
			"basler_set_devicetree_nodes_for_nvidia_modes() mode0 was not found: %d\n",
			ret);
		return -EINVAL;
	}

	of_changeset_init(&ocs);

	// here we write the properties that are mandatory for nvidia, but these are mostly values
	// that are used to calculate the MIPI clock rate and the pixel format, which we override.
	// So, we write them hard via code into device tree, to get rid of noise within our device
	// tree files
	// we don't override properties, so they still can be set in the device tree
	snprintf(char_buffer, BASLER_BUFF_SIZE, "%llu", pixel_clock);
	add_node_property_to_changeset(&ocs, node, "pix_clk_hz", char_buffer);
	snprintf(char_buffer, BASLER_BUFF_SIZE, "%u", csi_info->lane_count);
	add_node_property_to_changeset(&ocs, node, "num_lanes", char_buffer);
	// the rest are dummy values
	add_node_property_to_changeset(&ocs, node, "csi_pixel_bit_depth", "10");
	add_node_property_to_changeset(&ocs, node, "active_w", "100");
	add_node_property_to_changeset(&ocs, node, "active_h", "100");
	add_node_property_to_changeset(&ocs, node, "line_length", "100");
	add_node_property_to_changeset(&ocs, node, "mode_type", "bayer");
	add_node_property_to_changeset(&ocs, node, "pixel_phase", "bggr");
	add_node_property_to_changeset(&ocs, node, "phy_mode", "DPHY");

	ret = of_changeset_apply(&ocs);
	if (ret) {
		dev_err(dev, "of_changeset_apply() failed: %d\n", ret);
		goto changeset_cleanup;
	}

changeset_cleanup:
	of_changeset_destroy(&ocs);
	return ret;
}

static int
basler_set_common_data_for_nvidia(struct device *dev,
				  struct basler_camera_dev *sensor,
				  struct camera_common_data *common_data)
{
	int ret = 0;
	u64 pixel_clock, mipi_clock;
	int num_lanes, pixel_depth;
	int num_of_sensor_modes;
	struct basler_csi_information csi_info;
	ret = basler_retrieve_csi_information(sensor, &csi_info);
	if (ret) {
		dev_err(dev, "basler_retrieve_csi_information() failed: %d\n",
			ret);
		return ret;
	}
	num_of_sensor_modes = get_num_of_sensor_modes_in_dt(dev);
	if (num_of_sensor_modes != 1) {
		dev_err(dev, "We need exactly one sensor mode but found %d\n",
			num_of_sensor_modes);
		return -EINVAL;
	}
	common_data = devm_kzalloc(dev, sizeof(*common_data), GFP_KERNEL);

	common_data->priv = sensor;
	common_data->dev = dev;
	sensor->s_data = common_data;
	sensor->sd = &common_data->subdev;
	sensor->sd->dev = dev;

	common_data->def_clk_freq = csi_info.max_lane_frequency;
	// the rest are dummy values
	common_data->frmfmt = &basler_frmfmt[0];
	common_data->numfmts = ARRAY_SIZE(basler_frmfmt);
	common_data->def_mode = 0;
	common_data->def_width = 1920;
	common_data->def_height = 1080;
	common_data->fmt_width = common_data->def_width;
	common_data->fmt_height = common_data->def_height;

	ret = basler_set_devicetree_nodes_for_nvidia_modes(dev, &csi_info);
	if (ret) {
		dev_err(dev,
			"basler_set_devicetree_nodes_for_nvidia_modes() failed: %d\n",
			ret);
		return ret;
	}

	common_data->colorfmt =
		camera_common_find_datafmt(MEDIA_BUS_FMT_UYVY8_1X16);

	ret = camera_common_initialize(common_data, "basler-camera");
	if (ret) {
		dev_err(dev, "camera_common_initialize() failed: %d\n", ret);
		return ret;
	}
	mipi_clock = csi_info.max_lane_frequency;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 10, 0)
	// the mipi clock rate is calculated within the camera_common_initialize function.
	// but we want to use the value specified in the device tree endpoint.
	common_data->sensor_props.sensor_modes[0]
		.signal_properties.mipi_clock.val = mipi_clock;
#else
	// mipi clock is based on the pixel clock.
	// nothing to do here
#endif
	if (csi_info.lane_count != common_data->sensor_props.sensor_modes[0]
					   .signal_properties.num_lanes) {
		dev_err(dev,
			"consistency check for lane count failed: in the device tree of this sensor are two different lane counts. Please check your device tree.\n");
		return -EINVAL;
	}
	return ret;
}
#endif

static int basler_camera_probe(struct i2c_client *client,
			       const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct basler_camera_dev *sensor;
	int ret;

	dev_dbg(dev, " %s driver start probing\n", SENSOR_NAME);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(dev, "I2C_FUNC_I2C not supported\n");
		return -ENODEV;
	}

	sensor = devm_kzalloc(dev, sizeof(*sensor), GFP_KERNEL);
	if (!sensor)
		return -ENOMEM;

	/* VDD: 5V camera supply */
	sensor->vdd = devm_regulator_get_optional(dev, "vdd");
	if (IS_ERR(sensor->vdd)) {
		if (PTR_ERR(sensor->vdd) != -ENODEV) {
			dev_err(dev, "failed to get VDD regulator\n");
			return PTR_ERR(sensor->vdd);
		}
		dev_dbg(dev, "no VDD regulator found\n");
		sensor->vdd = NULL;
	}

	if (sensor->vdd != NULL) {
		/* Turn on camera supply and leave it on */
		ret = regulator_enable(sensor->vdd);
		if (ret) {
			dev_err(dev, "failed to enable VDD regulator\n");
			return ret;
		}
	}

	/*
	 * Give the camera time to power up and initialize its I2C interface.
	 * The duration is deliberately chosen much longer than the actual start-up
	 * time of the currently supported cameras to account for future models and
	 * to error on the safe side since it's not critical
	 */
	msleep(1000);

	sensor->i2c_client = client;

	basler_camera_set_default_format(sensor);

#ifdef NVIDIA_BUILD
	ret = basler_set_common_data_for_nvidia(dev, sensor, sensor->s_data);
	if (ret) {
		dev_err(dev, "basler_set_common_data_for_nvidia() failed: %d\n", ret);
		goto reg_off;
	}
#else
	sensor->sd = &sensor->sd_;
#endif

	v4l2_i2c_subdev_init(sensor->sd, client, &basler_camera_subdev_ops);

	ret = basler_retrieve_device_information(client,
						 &sensor->device_information);
	if (ret) {
		dev_dbg(dev,
			"basler_retrieve_device_information() failed: %d\n",
			ret);
		goto reg_off;
	}

	sensor->sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	sensor->pad.flags = MEDIA_PAD_FL_SOURCE;
	sensor->sd->entity.function = MEDIA_ENT_F_CAM_SENSOR;
	sensor->sd->entity.ops = &basler_camera_sd_media_ops;
	ret = media_entity_pads_init(&sensor->sd->entity, 1, &sensor->pad);
	if (ret) {
		dev_dbg(dev, "media_entity_pads_init() failed: %d\n", ret);
		goto reg_off;
	}

	mutex_init(&sensor->lock);

	ret = basler_camera_init_controls(sensor);
	if (ret) {
		dev_dbg(dev,
			"basler_camera_init_controls() failed: %d\n",
			ret);
		goto entity_cleanup;
	}

	ret = v4l2_async_register_subdev(sensor->sd);
	if (ret) {
		dev_dbg(dev,
			"v4l2_async_register_subdev() failed: %d\n",
			ret);
		goto handler_cleanup;
	}

	dev_dbg(dev, " %s driver probed\n", SENSOR_NAME);
	dev_info(dev, "Basler Camera Driver v" DRIVER_VERSION " loaded successfully\n");
	return 0;

handler_cleanup:
	sensor->sd->ctrl_handler = NULL;
	v4l2_ctrl_handler_free(&sensor->ctrl_handler);
entity_cleanup:
	mutex_destroy(&sensor->lock);
	media_entity_cleanup(&sensor->sd->entity);
reg_off:
	if (sensor->vdd != NULL) {
		regulator_disable(sensor->vdd);
	}

	return ret;
}

static int basler_camera_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct basler_camera_dev *sensor = to_basler_camera_dev(sd);

	v4l2_async_unregister_subdev(sensor->sd);
	sensor->sd->ctrl_handler = NULL;
	v4l2_ctrl_handler_free(&sensor->ctrl_handler);
	mutex_destroy(&sensor->lock);
	media_entity_cleanup(&sensor->sd->entity);

	if (sensor->vdd != NULL) {
		regulator_disable(sensor->vdd);
	}

	return 0;
}

static const struct i2c_device_id basler_camera_id[] = {
	{ "basler-camera", 0 },
	{},
};
MODULE_DEVICE_TABLE(i2c, basler_camera_id);

static const struct of_device_id basler_camera_dt_ids[] = {
	{ .compatible = "basler,basler-camera" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, basler_camera_dt_ids);

static struct i2c_driver basler_camera_i2c_driver = {
	.driver = {
			.name = "basler-camera",
			.of_match_table = basler_camera_dt_ids,
		},
	.id_table = basler_camera_id,
	.probe = basler_camera_probe,
	.remove = basler_camera_remove,
};

module_i2c_driver(basler_camera_i2c_driver);

MODULE_DESCRIPTION("Basler camera subdev driver");
MODULE_AUTHOR("Sebastian Suesens <sebastian.suesens@baslerweb.com>");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRIVER_VERSION);
