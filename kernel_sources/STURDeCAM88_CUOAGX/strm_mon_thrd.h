#ifndef _STRM_MON_THRD_H
#define _STRM_MON_THRD_H

#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/of_device.h>
#include <media/camera_common.h>
#include <linux/firmware.h>

#include "cam_common.h"
#include "serdes.h"

#define MAX_CAMERAS 4

#define ISP_STREAMING_STATE 0x05

#define OX08b40_ISP_CHIP_ID_REG 0x80FC
#define OX08b40_SENSOR_CHIP_ID_REG 0x300A
#define OX08B40_ISP_CHIP_ID 0x4000
#define OX08B40_ISP_STANDBY_REG 0x80CC
#define OX08B40_SENSOR_STANDBY_REG 0x0100
#define OX08B40_ISP_STREAMING_STATE 0x77
#define OX08B40_SENSOR_STREAMING_STATE 0x1
#define OX08B40_ISP_FRM_CNT_REG 0xA288
#define OX08B40_SENSOR_FRM_CNT_REG 0x483E

#define TWO_BYTE 2

static struct task_struct *strm_mon_thrd;
uint8_t check_queue_init;

/*IMU read*/
uint16_t accX, accY, accZ, gyroX, gyroY, gyroZ, temp;
int16_t accX_whole, accY_whole, accZ_whole, gyroX_whole, gyroY_whole, gyroZ_whole;
uint16_t accX_frac, accY_frac, accZ_frac, gyroX_frac, gyroY_frac, gyroZ_frac;
int16_t isp_temperature_integer, sensor_temperature_integer;
uint16_t isp_temperature_decimal, sensor_temperature_decimal;
struct kobject *kobj_ref_value_read;
uint8_t is_value_sysfs_dir;
struct kobject *kobj_ref_value_read_cam[8];

struct econ_cam {
	uint8_t des_slave_addr;
	uint8_t ser_slave_addr;
	uint8_t mcu_slave_addr;
	uint8_t isp_slave_addr;
	uint8_t sensor_slave_addr;
	uint8_t gmsl_link_status;
	uint8_t hw_link_error;
	uint8_t ser_pclkdet;
	uint8_t is_i2c_trans;
};
struct econ_stream_monitor {
	struct i2c_client *client;
	struct cam *cam;
	struct econ_cam *ecam_state;
};

typedef struct sensor_reg {
	u16 reg;
	u16 val;
} SENSOR_REG;

/*current control values*/
int32_t curr_brightness[MAX_CAMERAS];
int32_t curr_contrast[MAX_CAMERAS];
int32_t curr_saturation[MAX_CAMERAS];
int32_t curr_gain[MAX_CAMERAS];
int32_t curr_cam_mode[MAX_CAMERAS];
int32_t curr_powerline_freq[MAX_CAMERAS];
int32_t curr_exp_mode[MAX_CAMERAS];
int32_t curr_exp_val[MAX_CAMERAS];
int32_t curr_fps[MAX_CAMERAS];
int32_t curr_lfm_mode[MAX_CAMERAS];
int32_t curr_sharpness[MAX_CAMERAS];
int32_t curr_hflip[MAX_CAMERAS];
int32_t curr_vflip[MAX_CAMERAS];
int32_t curr_fsync[MAX_CAMERAS];
int32_t curr_exp_comp_max[MAX_CAMERAS];
int32_t curr_exp_comp_min[MAX_CAMERAS];
#ifdef WB_enable
int32_t curr_auto_wb[MAX_CAMERAS];
int32_t curr_red_gain[MAX_CAMERAS];
int32_t curr_blue_gain[MAX_CAMERAS];
int32_t curr_green_gain[MAX_CAMERAS];
#endif

uint8_t thread_handle[MAX_CAMERAS] = {1, 1, 1, 1};


typedef struct {
	int ctrl_id;
	int *curr_array;
	const char *name;
} ctrl_map_t;

const ctrl_map_t available_ctrls[] = {
	{ V4L2_CID_BRIGHTNESS,               curr_brightness,     "brightness" },
	{ V4L2_CID_CONTRAST,                 curr_contrast,       "contrast" },
	{ V4L2_CID_SATURATION,               curr_saturation,     "saturation" },
	{ V4L2_CID_GAIN,                     curr_gain,           "gain" },
	{ V4L2_CID_CAM_MODE,                 curr_cam_mode,       "cam_mode" },
	{ V4L2_CID_POWER_LINE_FREQUENCY,      curr_powerline_freq, "powerline_freq" },
	{ V4L2_CID_EXPOSURE_AUTO,		 curr_exp_mode,       "exposure_auto" },
	{ V4L2_CID_EXPOSURE_ABSOLUTE,        curr_exp_val,        "exposure_absolute" },
#ifdef CUSTOM_FPS
	{ V4L2_CID_FRAME_RATE_CONTROL,       curr_fps,            "frame_rate" },
#endif
	{ V4L2_CID_LFM_FACTOR,               curr_lfm_mode,       "lfm_factor" },
	{ V4L2_CID_SHARPNESS,                curr_sharpness,      "sharpness" },
	{ V4L2_CID_HFLIP,                    curr_hflip,          "hflip" },
	{ V4L2_CID_VFLIP,                    curr_vflip,          "vflip" },
	{ V4L2_CID_FRAME_SYNC,               curr_fsync,          "frame_sync" },
	{ V4L2_CID_EXPOSURE_COMPENSATION_MAX,    curr_exp_comp_max,       "exposure_compensation_max" },
	{ V4L2_CID_EXPOSURE_COMPENSATION_MIN,    curr_exp_comp_min,       "exposure_compensation_min" },
#ifdef WB_enable
	{ V4L2_CID_AUTO_WHITE_BALANCE,       curr_auto_wb,        "auto_white_balance" },
	{ V4L2_CID_RED_GAIN,                 curr_red_gain,       "red_gain" },
	{ V4L2_CID_BLUE_GAIN,                curr_blue_gain,      "blue_gain" },
	{ V4L2_CID_GREEN_GAIN,               curr_green_gain,     "green_gain" },
#endif
};

const int available_ctrls_count = ARRAY_SIZE(available_ctrls);

struct econ_stream_monitor *strm_mon[8];

struct kobject *kobj_ref;
static int is_strm_mon_mem;
extern uint8_t g_max_cam;
extern uint8_t num_cam;
extern uint8_t total_num_cam;
extern uint8_t strm_thrd_num;

/*For Queue*/
extern wait_queue_head_t econ_err_hand_q;
extern int econ_frame_err_track;
extern int econ_num_uncorr_err;
extern char econ_dev_name[32];
wait_queue_head_t check_econ_err_hand_q;

uint8_t is_stop_strm_mon_thread;

extern const char *dev_name_comp[8];

uint8_t strm_mon_ser_status;

int create_sysfs_dir(struct i2c_client *client, struct cam *priv);
int remove_sysfs_dir(void);
int stream_monitor_thread(void *strm_dev);
int remove_value_read_sysfs_entries(void);

extern int is_stream_monitor_thrd;
extern int ecam_status_check;

uint8_t create_stream_monitor_thread(void);
uint8_t stop_stream_monitor_thread(void);

extern SERDES_PARSE SER1_CONF[];
extern SERDES_PARSE SER2_CONF[];
extern SERDES_PARSE DSER_CONF_8CAM[];
extern SERDES_PARSE SER1_I2C_CONF[];
extern SERDES_PARSE SER2_I2C_CONF[];
extern SERDES_PARSE SER1_PKT_HEADER_CHANGE[];
extern SERDES_PARSE SER2_PKT_HEADER_CHANGE[];

extern uint8_t dser1_ser_status;
extern uint8_t dser2_ser_status;
extern uint8_t ser1_array_size;
extern uint8_t ser2_array_size;
extern uint8_t ser1_i2c_array_size;
extern uint8_t ser2_i2c_array_size;
extern uint8_t dser8_array_size;
extern uint8_t ser1_pkt_header_array_size;
extern uint8_t ser2_pkt_header_array_size;

int cam_get_serial_number(struct i2c_client *client);
int mcu_set_ctrl(struct i2c_client *client, uint32_t arg_ctrl_id, uint8_t ctrl_type, int32_t curr_val);
int gen_mcu_stream_config(struct i2c_client *client, struct cam *priv);
int mcu_isp_init(struct i2c_client *client);
int mcu_get_sensor_id(struct i2c_client *client, uint16_t *sensor_id);
int mcu_get_fw_version(struct i2c_client *client,
					   unsigned char *fw_version, unsigned char *txt_fw_version);
int mcu_mipi_configuration(struct i2c_client *client, struct cam *priv, u8 cmd_id);
int toggle_mcu_reset_pin(struct i2c_client *client, struct cam *priv);
int toggle_mcu_boot_pin(struct i2c_client *client, struct cam *priv);

int sensor_isp_write(struct i2c_client *err_client,
					 uint16_t reg_addr, uint16_t reg_val);
int sensor_isp_read(struct i2c_client *err_client, uint16_t reg_addr,
					uint8_t reg_len);
int ox08b40_sensor_read(struct i2c_client *err_client, uint16_t reg_addr,
						uint8_t reg_len);
int ox08b40_sensor_read_SET(struct i2c_client *err_client, uint16_t reg_addr,
							uint8_t reg_len);
int ox08b40_sensor_read_GET(struct i2c_client *err_client, uint16_t reg_addr,
							uint8_t reg_len);
int dser_err_handler(struct econ_stream_monitor *err_cam, uint8_t err_handle);
int serdes_chip_status(struct econ_stream_monitor *err_cam);
int get_specific_link_lock_status(struct econ_stream_monitor *err_cam,
								  uint16_t link_status_addr);
int dser_video_lock_status(struct econ_stream_monitor *err_cam);
int ser_err_handler(struct econ_stream_monitor *err_cam, uint8_t *link_enable_val, uint8_t err_handle);
int serializer_reassignment(struct econ_stream_monitor *err_cam);
int reconfigure_serializer(struct econ_stream_monitor *err_cam);
int mcu_err_handle(struct econ_stream_monitor *err_cam,
				   uint8_t err_handle);
int mipi_sensor_mipi_config(struct i2c_client *client,
							struct cam *priv, u8 cmd_id);
int reconfigure_deserializer(struct econ_stream_monitor *err_cam);
int isp_sensor_err_handle(struct econ_stream_monitor *err_cam, uint8_t err_handle);
int mcu_get_imu_angle_GET(struct i2c_client *client);
int mcu_get_imu_angle_SET(struct i2c_client *client);
int mcu_get_imu_angle(struct i2c_client *client);
int mcu_get_isp_sensor_temperature(struct i2c_client *client);
int mcu_get_isp_sensor_temperature_SET(struct i2c_client *client);
int mcu_get_isp_sensor_temperature_GET(struct i2c_client *client);
int serdes_err_handle(struct econ_stream_monitor *err_cam);
int ser_des_chip_link_status(struct econ_stream_monitor *err_cam);
int des_chip_gmsl_link_lock_status(struct econ_stream_monitor *err_cam);
int ser_chip_video_status(struct econ_stream_monitor *err_cam);
int ser1_detect(struct econ_stream_monitor *err_cam);
int ser2_detect(struct econ_stream_monitor *err_cam);
int check_ser2_for_ser1_error(struct econ_stream_monitor *err_cam);
int check_ser1_for_ser2_error(struct econ_stream_monitor *err_cam);
int ser2_i2c_reassignment(struct econ_stream_monitor *err_cam);

int check_ser_i2c_translation(struct econ_stream_monitor *err_cam);
int i2c_trans_reg_read(struct econ_stream_monitor *err_cam, uint16_t reg_addr);
int re_write_i2c_trans_settings(struct econ_stream_monitor *err_cam,
		SERDES_PARSE *ser_conf, u32 reg_cnt);

#endif				//_STRM_MON_THRD_H
