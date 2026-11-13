#ifndef _ECAM_GMSL_OCTA_YUV_COMMON_H
#define _ECAM_GMSL_OCTA_YUV_COMMON_H

#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/of_device.h>
#include <media/camera_common.h>
#include <linux/firmware.h>
#include <linux/signal.h>
#include <linux/pwm.h>

#include "cam_common.h"

#define CMD_SIGNATURE 0x43
#define TX_LEN_PKT 5
#define RX_LEN_PKT 6
#define HEADER_FOOTER_SIZE 4
#define CMD_STATUS_MSG_LEN 7

#define MAX_NUM_CONTROLS 30
#define MAX_NUM_FRATES 10

/*MCU FW update state*/
#define MCU_FW_FORCE_UPDATE 2
#define MCU_FW_MISMATCH_UPDATE 1

#define VERSION_SIZE 32
#define VERSION_FILE_OFFSET 99
#define VERSION_FILE_OFFSET_SPI (VERSION_FILE_OFFSET + 92)

#define Ox08B40_DEFAULT_WIDTH 3840
#define Ox08B40_DEFAULT_HEIGHT 2160

#define DEFAULT_DATAFMT MEDIA_BUS_FMT_UYVY8_1X16

#define MCU_BOOT_LOW 0x40
#define MCU_BOOT_HIGH 0x18
#define MCU_RST_LOW 0x40
#define MCU_RST_HIGH 0x98

#define BYTE_1 1
#define BYTE_2 2
#define BYTE_3 3
#define BYTE_4 4

#define FREE_SAFE(dev, ptr)	\
	do {			\
		if (ptr) {		\
					\
			devm_kfree(dev, ptr);	\
		}			\
	} while (0)

#define PWM_5HZ_PERIOD  200000000
#define PWM_5HZ_DUTY    3000000

#define MAX_CAMERAS 4

uint8_t monitor_call;
/* Sysfs Variable for probe status */
extern uint8_t sysfs_probe_dir;
extern struct kobject *kobj_ref_probe;
extern struct kobject *kobj_ref_probe_cam[8];
extern char probe_status_str[20];
extern uint8_t total_cam_probed;

/*IMU read*/
extern uint16_t accX, accY, accZ, gyroX, gyroY, gyroZ, temp;
extern int16_t accX_whole, accY_whole, accZ_whole, gyroX_whole, gyroY_whole, gyroZ_whole;
extern uint16_t accX_frac, accY_frac, accZ_frac, gyroX_frac, gyroY_frac, gyroZ_frac;

extern int16_t isp_temperature_integer, sensor_temperature_integer, isp_temperature_decimal, sensor_temperature_decimal;
extern struct kobject *kobj_ref_value_read;
extern struct kobject *kobj_ref_value_read_cam[8];
char imu_read[50];
char temp_read[50];

struct probe_status {
	char status[50];
	int percentage;
	char mcu_fw_status[50];
	int mcu_fw_status_percentage;
	char spi_fw_status[50];
	int spi_fw_status_percentage;
};

extern struct probe_status probe_status_array[];

int is_stream_monitor_thrd;

extern int32_t curr_brightness[MAX_CAMERAS];
extern int32_t curr_contrast[MAX_CAMERAS];
extern int32_t curr_saturation[MAX_CAMERAS];
extern int32_t curr_gain[MAX_CAMERAS];
extern int32_t curr_cam_mode[MAX_CAMERAS];
extern int32_t curr_powerline_freq[MAX_CAMERAS];
extern int32_t curr_exp_mode[MAX_CAMERAS];
extern int32_t curr_exp_val[MAX_CAMERAS];
extern int32_t curr_fps[MAX_CAMERAS];
extern int32_t curr_lfm_mode[MAX_CAMERAS];
extern int32_t curr_sharpness[MAX_CAMERAS];
extern int32_t curr_hflip[MAX_CAMERAS];
extern int32_t curr_vflip[MAX_CAMERAS];
extern int32_t curr_fsync[MAX_CAMERAS];
extern int32_t curr_exp_comp[MAX_CAMERAS];
#ifdef WB_enable
extern int32_t curr_auto_wb[MAX_CAMERAS];
extern int32_t curr_red_gain[MAX_CAMERAS];
extern int32_t curr_blue_gain[MAX_CAMERAS];
extern int32_t curr_green_gain[MAX_CAMERAS];
#endif

extern uint8_t thread_handle[MAX_CAMERAS];

char serial_num[16] = {0};
char lens_identifier[9] = {0};
const struct firmware *lens_cfg_file;
const char *lens_config_filename = "lens_id_configuration.csv";
char *isp_bootdata_name;
unsigned char *lens_config_buf;
uint16_t boot_identified;
typedef enum {
	NARROW_ANGLE = '1',
	WIDE_ANGLE = '2',
} Lens_Type;

/* Global Mutex Defenition */
DEFINE_MUTEX(g_i2c_mutex);

static uint8_t g_prev_connected_cam_id;
uint8_t g_max_cam;

/* Global Variables releated to MCU Fw */
const char *mcu_fw_name;
const char *mcu_fw_name_ISP;
const struct firmware *mcu_fw;
static uint8_t is_fw_loaded;
unsigned char *mcu_fw_buf;
unsigned char *mcu_fw_buf_ISP;
const struct firmware *mcu_fw_ISP;

/* Global Variable to integrate Load Switch */
static uint8_t num_cam_probed;

int frm_fmt_size;
int strm_cam_num;

/* Error Recovery */
uint8_t create_stream_monitor_thread(void);
uint8_t stop_stream_monitor_thread(void);
int ecam_status_check = 1;
int total_num_cam;
int strm_thrd_num;
const char *dev_name_comp[8];

typedef struct {
	int ctrl_id;
	int *curr_array;
	const char *name;
} ctrl_map_t;

// Controls table
extern const ctrl_map_t available_ctrls[];
extern const int available_ctrls_count;

int mcu_bload_ascii2hex(unsigned char ascii);
int mcu_bload_get_version(struct i2c_client *client);
int mcu_bload_go(struct i2c_client *client);
int mcu_bload_update_fw(struct i2c_client *client);
int mcu_bload_read(struct i2c_client *client,
				   unsigned int g_bload_flashaddr, char *bytearray,
				   unsigned int len);
int mcu_bload_verify_flash(struct i2c_client *client,
						   unsigned short int orig_crc);
static int mcu_jump_bload(struct i2c_client *client);
int mcu_bload_parse_send_cmd(struct i2c_client *client,
							 unsigned char *bytearray, int rec_len);
int mcu_bload_erase_flash(struct i2c_client *client);
unsigned short int mcu_bload_calc_crc16(unsigned char *buf, int len);
unsigned char mcu_bload_inv_checksum(unsigned char *buf, int len);
static int cam_read(struct i2c_client *client, u8 *val, u32 count);
unsigned char errorcheck(char *data, unsigned int len);
static int cam_write(struct i2c_client *client, u8 *val, u32 count);
static int mcu_list_ctrls(struct i2c_client *client,
						  ISP_CTRL_INFO *mcu_cam_ctrl, struct cam *priv);
static int mcu_get_ctrl(struct i2c_client *client, uint32_t arg_ctrl_id,
						uint8_t *ctrl_type, int32_t *curr_val);
int mcu_get_cmd_status(struct i2c_client *client,
					   uint8_t *cmd_id, uint16_t *cmd_status,
					   uint8_t *ret_code);

/* Error Recovery Functions */
int create_sysfs_dir(struct i2c_client *client, struct cam *priv);
int remove_sysfs_dir(void);
int remove_value_read_sysfs_entries(void);

/* sysfs functions */
uint8_t create_probe_sysfs_entries(void);
uint8_t create_cam_specific_sysfs_entries(uint8_t num_cam, uint8_t cam_id);
extern uint8_t create_value_read_sysfs_entries(uint8_t num_cam, uint8_t cam_id);

#endif				//_ECAM_GMSL_OCTA_YUV_COMMON_H
