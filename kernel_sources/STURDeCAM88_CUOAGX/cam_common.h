#ifndef _CAM_COMMON_H
#define _CAM_COMMON_H

#define WB_enable
#define MAX_CTRL_DATA_LEN 100
#define MAX_CTRL_UI_STRING_LEN 32

enum supported_cam_list {
	Ox08B40 = 0,
};

enum {
	FRAME_RATE_DISCRETE = 0x01,
	FRAME_RATE_CONTINOUS = 0x02,
};

enum {
	CTRL_STANDARD = 0x01,
	CTRL_EXTENDED = 0x02,
};

typedef enum _errcode {
	ERRCODE_SUCCESS = 0x00,
	ERRCODE_BUSY = 0x01,
	ERRCODE_INVAL = 0x02,
	ERRCODE_PERM = 0x03,
	ERRCODE_NODEV = 0x04,
	ERRCODE_IO = 0x05,
	ERRCODE_HW_SPEC = 0x06,
	ERRCODE_AGAIN = 0x07,
	ERRCODE_ALREADY = 0x08,
	ERRCODE_NOTIMPL = 0x09,
	ERRCODE_RANGE = 0x0A,

	/*   Reserved 0x0B - 0xFE */

	ERRCODE_UNKNOWN = 0xFF,
} ERRCODE;

typedef enum _cmd_id {
	CMD_ID_VERSION = 0x00,
	CMD_ID_GET_SENSOR_ID = 0x01,
	CMD_ID_GET_STREAM_INFO = 0x02,
	CMD_ID_GET_CTRL_INFO = 0x03,
	CMD_ID_INIT_CAM = 0x04,
	CMD_ID_GET_STATUS = 0x05,
	CMD_ID_DE_INIT_CAM = 0x06,
	CMD_ID_STREAM_ON = 0x07,
	CMD_ID_STREAM_OFF = 0x08,
	CMD_ID_STREAM_CONFIG = 0x09,
	CMD_ID_GET_CTRL_UI_INFO = 0x0A,

	/* Reserved 0x0B to 0x0F */

	CMD_ID_GET_CTRL = 0x10,
	CMD_ID_SET_CTRL = 0x11,

	/* Reserved 0x12, 0x13 */
	CMD_ID_ISP_READ = 0x12,
	CMD_ID_ISP_WRITE = 0x13,

	CMD_ID_FW_UPDT = 0x14,
	CMD_ID_ISP_PDOWN = 0x15,
	CMD_ID_ISP_PUP = 0x16,

	/* Configuring MIPI Lanes */
	CMD_ID_LANE_CONFIG = 0x17,
	CMD_ID_MIPI_CLK_CONFIG = 0x18,

	CMD_ID_SENSOR_READ = 0x19,
	CMD_ID_SENSOR_WRITE = 0x1A,

	CMD_ID_SENSOR_READ_SET = 0x1B,
	CMD_ID_SENSOR_READ_GET = 0x1C,

	CMD_ID_IMU_READ = 0x1D,
	CMD_ID_IMU_READ_SET = 0xA1,
	CMD_ID_IMU_READ_GET = 0xA2,
	CMD_ID_GET_IMU_ID = 0x1E,

	/* Below CMD ID's for OX08B40 Sensor */
	CMD_ID_TEMPERATURE_SET = 0xAA,
	CMD_ID_TEMPERATURE_GET = 0xAB,
	CMD_ID_READ_SERIAL_NUMBER = 0x30,
	CMD_ID_SHARE_LENS_ID = 0x32,
	/* Reserved - 0xB0 to 0xFE (except 0x43) */

	CMD_ID_UNKNOWN = 0xFF,

} HOST_CMD_ID;

enum {
	DISABLE = 0,
	ENABLE = 1,
};

typedef struct _isp_stream_info {
	uint32_t fmt_fourcc;
	uint16_t width;
	uint16_t height;
	uint8_t frame_rate_type;
	union {
		struct {
			uint16_t frame_rate_num;
			uint16_t frame_rate_denom;
		} disc;
		struct {
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
	} ctrl_ui_info;

	/* This Struct is valid only if ctrl_ui_type = 0x03 */
	struct {
		uint8_t num_menu_elem;
		char **menu;
	} ctrl_menu_info;
} ISP_CTRL_UI_INFO;

typedef struct _isp_ctrl_info_std {
	uint32_t ctrl_id;
	uint8_t ctrl_type;
	union {
		struct {
			int32_t ctrl_min;
			int32_t ctrl_max;
			int32_t ctrl_def;
			int32_t ctrl_step;
		} std;
		struct {
			uint8_t val_type;
			uint32_t val_length;
			// This size may vary according to ctrl types
			uint8_t val_data[MAX_CTRL_DATA_LEN];
		} ext;
	} ctrl_data;
	ISP_CTRL_UI_INFO ctrl_ui_data;
} ISP_CTRL_INFO;

struct cam {
	struct camera_common_power_rail power;
	struct v4l2_ctrl_handler ctrl_handler;
	struct i2c_client *i2c_client;
	struct v4l2_subdev *subdev;
	struct media_pad pad;

	struct camera_common_data *s_data;
	struct camera_common_pdata *pdata;

	uint16_t frame_index;
	uint32_t format_fourcc;
	int frmfmt_mode;
	int num_ctrls;
	ISP_STREAM_INFO *stream_info;
	ISP_CTRL_INFO *mcu_ctrl_info;
	/* Total formats */
	int *streamdb;
	uint32_t *ctrldb;
	/* Array of Camera framesizes */
	struct camera_common_frmfmt *mcu_cam_frmfmt;
	uint16_t prev_index;
	uint16_t mipi_clk_config;
	uint16_t mipi_lane_config;
	int ref_count;
	int cam_num;

	uint8_t pwm_sync;
	uint8_t trigger_mode;

	uint32_t reset_reg;
	uint32_t boot_reg;
	uint8_t cam_id;

	struct pwm_device *pwm_dev;
	bool got_pwm_handle;

	uint8_t phy;
	uint32_t ser_def_addr;
	uint32_t ser_reassigned_addr;
	uint32_t dser_addr;
	struct v4l2_ctrl *ctrls[];
};

enum {
	V4L2_CID_FACEDETECT = (V4L2_CID_AUTO_FOCUS_RANGE+1),
	V4L2_CID_FACEMARK,
	V4L2_CID_SMILEDETECT,
	V4L2_GET_FACEINFO,
	V4L2_CID_ROI_WINDOW,
	V4L2_CID_ROI_FOCUS,
	V4L2_CID_ROI_EXPOSURE,
	V4L2_CID_TRIGGER_FOCUS,

	/* New Controls */
	V4L2_CID_CAM_MODE,
	V4L2_CID_CUSTOM_TRIGGER,
	V4L2_CID_COLORKILL,
	V4L2_CID_FRAME_SYNC,
	V4L2_CID_PWM_CONFIG,
	V4L2_CID_TRIGGER_MODE,
	V4L2_CID_CUSTOM_EXPOSURE_AUTO,
	V4L2_CID_EXPOSURE_COMPENSATION_MAX,
	V4L2_CID_EXPOSURE_COMPENSATION_MIN,
	V4L2_CID_FRAME_RATE_CONTROL,
	V4L2_CID_SPECIAL_EFFECT,
	V4L2_CID_TEMPERATURE,
	V4L2_CID_LFM_FACTOR,
#ifdef WB_enable
	V4L2_CID_RED_GAIN,
	V4L2_CID_BLUE_GAIN,
	V4L2_CID_GREEN_GAIN,
#endif
};

#define V4L2_CID_CAM_MODE (V4L2_CID_BASE+22)

#endif				//_CAM_COMMON_H
