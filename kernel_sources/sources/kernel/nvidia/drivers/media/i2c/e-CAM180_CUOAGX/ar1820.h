/**
 * Copyright (c) 2017-2018 e-con Systems India Pvt. Ltd.  All rights reserved.
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

/* Defines related to MCU */
#define CMD_SIGNATURE			0x43
#define TX_LEN_PKT			5
#define RX_LEN_PKT			6
#define HEADER_FOOTER_SIZE		4
#define CMD_STATUS_MSG_LEN		7
#define MAX_CTRL_DATA_LEN 		100
#define MAX_CTRL_UI_STRING_LEN 		32
#define MAX_NUM_FRATES			10
#define NUM_CTRLS			10
#define VERSION_SIZE			32
#define MAX_ATTEMPTS			10
#define MCU_CMD_STATUS_SUCCESS		0x0000
#define MCU_CMD_STATUS_PENDING		0xF000
#define MCU_CMD_STATUS_ISP_UNINIT	0x0FF1
#define EXTENDED_CTRL_SIZE		8

#define AR1820_FUSE_ID_SIZE		6

static const u32 custom_ctrl_cid_list[] = {
	TEGRA_CAMERA_CID_GAIN,
	TEGRA_CAMERA_CID_EXPOSURE,
	TEGRA_CAMERA_CID_EXPOSURE_SHORT,
	TEGRA_CAMERA_CID_FRAME_RATE,
	TEGRA_CAMERA_CID_FUSE_ID,
	TEGRA_CAMERA_CID_HDR_EN,
	TEGRA_CAMERA_CID_SENSOR_MODE_ID,
};

static const u32 def_ctrl_cid_list[] = {
	TEGRA_CAMERA_CID_GAIN,
	TEGRA_CAMERA_CID_EXPOSURE,
	TEGRA_CAMERA_CID_EXPOSURE_SHORT,
	TEGRA_CAMERA_CID_FRAME_RATE,
	TEGRA_CAMERA_CID_FUSE_ID,
	//TEGRA_CAMERA_CID_HDR_EN,
	TEGRA_CAMERA_CID_SENSOR_MODE_ID,
	//TEGRA_CAMERA_CID_FRAME_SYNC,
};

typedef enum _errno
{
        ERRCODE_SUCCESS = 0x00,
        ERRCODE_BUSY = 0x01,

} RETCODE;

typedef enum _cmd_id
{
        CMD_ID_VERSION = 0x00,
        CMD_ID_GET_STREAM_INFO = 0x02,
        CMD_ID_GET_CTRL_INFO = 0x03,
        CMD_ID_INIT_CAM = 0x04,
        CMD_ID_GET_STATUS = 0x05,
        CMD_ID_STREAM_ON = 0x07,
        CMD_ID_STREAM_OFF = 0x08,
        CMD_ID_STREAM_CONFIG = 0x09,

        /* Reserved 0x0B to 0x0F */

        CMD_ID_SET_CTRL = 0x11,

	/* Configuring MIPI Lanes */
	CMD_ID_LANE_CONFIG = 0x17,

} HOST_CMD_ID;

enum
{
        FRAME_RATE_DISCRETE = 0x01,
        FRAME_RATE_CONTINOUS = 0x02,
};

enum
{
        CTRL_STANDARD = 0x01,
        CTRL_EXTENDED = 0x02,
};


typedef struct _isp_stream_info
{
        uint32_t fmt_fourcc;
        uint16_t width;
        uint16_t height;
        uint8_t frame_rate_type;
        union
        {
                struct
                {
                        uint16_t frame_rate_num;
                        uint16_t frame_rate_denom;
                } disc;
                struct
                {
                        uint16_t frame_rate_min_num;
                        uint16_t frame_rate_min_denom;
                        uint16_t frame_rate_max_num;
                        uint16_t frame_rate_max_denom;
                        uint16_t frame_rate_step_num;
                        uint16_t frame_rate_step_denom;
                } cont;
        } frame_rate;
} ISP_STREAM_INFO;


typedef struct _isp_ctrl_ui_info {
	struct {
		char ctrl_name[MAX_CTRL_UI_STRING_LEN];
		uint8_t ctrl_ui_type;
		uint8_t ctrl_ui_flags;
	}ctrl_ui_info;

	/* This Struct is valid only if ctrl_ui_type = 0x03 */
	struct {
		uint8_t num_menu_elem;
		char **menu;
		long *menu_int;
	}ctrl_menu_info;
} ISP_CTRL_UI_INFO;

typedef struct _isp_ctrl_info_std
{
        uint32_t ctrl_id;
        uint8_t ctrl_type;
        union
        {
                struct
                {
                        int32_t ctrl_min;
                        int32_t ctrl_max;
                        int32_t ctrl_def;
                        int32_t ctrl_step;
                } std;
                struct
                {
                        uint8_t val_type;
                        uint32_t val_length;
                        // This size may vary according to ctrl types
						uint64_t ctrl_min;
						uint64_t ctrl_max;
						uint64_t ctrl_def;
						uint64_t ctrl_step;
						uint8_t val_data[MAX_CTRL_DATA_LEN];
				} ext;
        } ctrl_data;
	ISP_CTRL_UI_INFO ctrl_ui_data;

} ISP_CTRL_INFO;

struct cam_mode {
	__u32 frame_length;
	__u32 coarse_time;
	__u32 coarse_time_short;
	__u16 gain;
};

struct cam_hdr {
	__u32 coarse_time_short;
};

struct cam_ae {
	__u32 frame_length;
	__u32 coarse_time;
	__u32 coarse_time_short;
	__s32 gain;
};

#ifdef __KERNEL__
struct cam_power_rail {
	struct regulator *dvdd;
	struct regulator *avdd;
	struct regulator *iovdd;
	struct clk *mclk;
	unsigned int pwdn_gpio;
	unsigned int cam1_gpio;
	unsigned int reset_gpio;
	unsigned int af_gpio;
};

struct cam {
	struct camera_common_power_rail	power;
	struct i2c_client		*i2c_client;
	struct v4l2_subdev		*subdev;
	u32				frame_length;
	ISP_STREAM_INFO			*stream_info;
	ISP_CTRL_INFO 			*cam_ctrl_info;
	uint32_t 			*ctrldb;
	struct camera_common_data	*s_data;
	struct camera_common_frmfmt 	*cam_frmfmt;
	int 				num_ctrls;
	int 				*streamdb;
	struct tegracam_device		*tc_dev;
	int				current_mode;
	int 				format_fourcc;
	uint16_t 			mode_index;
	uint8_t				mipi_lane_config;
	unsigned int			reset_gpio;
	unsigned int			boot_gpio;
	unsigned int			trigger_sel_gpio;
	unsigned int			flash_gpio;
	bool				use_sensor_mode_id;
	bool 				use_master_slave_mode;
	struct mutex 			cam_i2c_mutex;
	uint8_t 			last_sync_mode;
};

struct cam_platform_data {
	const char *mclk_name; /* NULL for default default_mclk */
	unsigned int cam1_gpio;
	unsigned int reset_gpio;
	unsigned int af_gpio;
	int (*power_on)(struct cam_power_rail *pw);
	int (*power_off)(struct cam_power_rail *pw);
};
#endif /* __KERNEL__ */

static int cam_set_mode(struct tegracam_device *tc_dev);
static int cam_read(struct i2c_client *client, u8 * val, u32 count);
static int cam_write(struct i2c_client *client, u8 * val, u32 count);
static int cam_list_fmts(struct i2c_client *client, struct cam *priv,
			  ISP_STREAM_INFO *stream_info,int *frm_fmt_size);
static int cam_list_ctrls(struct i2c_client *client, struct cam *priv,
                          ISP_CTRL_INFO * cam_ctrl_info);


unsigned char errorcheck(char *data, unsigned int len);
static int is_fw_update_required(struct i2c_client *client, struct cam *priv, 
		unsigned char * fw_version, unsigned char *txt_fw_version);
static int cam_get_cmd_status(struct i2c_client *client, uint8_t * cmd_id,
                              uint16_t * cmd_status, uint8_t * ret_code);
static int cam_init(struct i2c_client *client);
static int cam_stream_config(struct i2c_client *client, struct cam *priv, uint32_t format,
                             int mode, uint16_t frate_index);
static int cam_set_ctrl(struct i2c_client *client,struct cam *priv,  uint32_t ctrl_id,
                        uint8_t ctrl_type, int64_t curr_val);
static int cam_fw_update(struct i2c_client *client, unsigned char *cam_fw_version);

static int cam_stream_on(struct i2c_client *client, struct cam *priv);
static int cam_stream_off(struct i2c_client *client, struct cam *priv);
static int cam_set_exposure(struct tegracam_device *tc_dev, s64 val);

static const u32 ctrl_cid_list[] = {
        TEGRA_CAMERA_CID_GAIN,
        TEGRA_CAMERA_CID_EXPOSURE,
        TEGRA_CAMERA_CID_EXPOSURE_SHORT,
        TEGRA_CAMERA_CID_FRAME_RATE,
        TEGRA_CAMERA_CID_FUSE_ID,
        TEGRA_CAMERA_CID_HDR_EN,
        TEGRA_CAMERA_CID_SENSOR_MODE_ID,
};

enum {
AR1820_MODE_0_4912X3684_12BIT_6FPS,
AR1820_MODE_1_4912X3684_10BIT_6FPS,
AR1820_MODE_2_4096X2160_10BIT_14FPS,
AR1820_MODE_3_1920X1080_10BIT_59FPS,
};

static const int ar1820_6_fps[] = {
	6,
};

static const int ar1820_3_fps[] = {
	3,
};


static const int ar1820_14_fps[] = {
	14,
};

static const int ar1820_59_fps[] = {
	59,
};


/*
 * WARNING: frmfmt ordering need to match mode definition in
 * device tree!
 */
static const struct camera_common_frmfmt ar1820_frmfmt[] = {
	{{4912, 3684},	ar1820_6_fps, 1, 0, AR1820_MODE_0_4912X3684_12BIT_6FPS},
	{{4912, 3684},	ar1820_6_fps, 1, 0, AR1820_MODE_1_4912X3684_10BIT_6FPS},
	{{4096, 2160},	ar1820_14_fps, 1, 0, AR1820_MODE_2_4096X2160_10BIT_14FPS},
	{{1920, 1080},	ar1820_59_fps, 1, 0, AR1820_MODE_3_1920X1080_10BIT_59FPS},
	/* Add modes with no device tree support after below */
};

