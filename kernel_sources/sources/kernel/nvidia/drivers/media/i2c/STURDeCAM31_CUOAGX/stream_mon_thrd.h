#define ISP_CHIP_ID_REG 0x8A54

#define ISP_CHIP_ID 0x1A04
#define ISP_FRM_CNT_REG 0x7DC8
#define ISP_STANDBY_REG 0x6005

#define ISP_STREAMING_STATE 0x05

#define TWO_BYTE 2

struct econ_cam 
{
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
struct econ_stream_monitor
{
	struct i2c_client *client;
	struct isx031 *cam;
	struct econ_cam *ecam_state;
	
};

typedef struct sensor_reg {
	u16 reg;
	u16 val;
} SENSOR_REG;

SENSOR_REG par_sens_frm_cnt[] = {
	{0x0040, 0x8D00},
	{0xFC00, 0X0000},
	{0XFC02, 0X0200},
	{0X0040, 0X8D05},
	{0XFFFF, 0X0000},
	{0X0040, 0X8D08},
	{0XFFFF, 0X0000},
	{0X098E, 0X7C00},
};
SENSOR_REG par_sens_chip_id[] = {
	{0x0040, 0x8D00},
	{0xFC00, 0X3000},
	{0XFC02, 0X0200},
	{0X0040, 0X8D05},
	{0XFFFF, 0X0000},
	{0X0040, 0X8D08},
	{0XFFFF, 0X0000},
	{0X098E, 0X7C00},
};
SENSOR_REG par_sens_mode[] = {
	{0x0040, 0x8D00},
	{0xFC00, 0X0000},
	{0XFC02, 0X0200},
	{0X0040, 0X8D05},
	{0XFFFF, 0X0000},
	{0X0040, 0X8D08},
	{0XFFFF, 0X0000},
	{0X098E, 0X7C00},
};

enum chip_type{
	ISP,
	SENSOR,
};
enum find_cam{
	CAM30_43 = 0,
	CAM30_44 = 1,
	CAM31_43 = 2,
	CAM31_44 = 3,
	CAM32_43 = 4,
	CAM32_44 = 5,
};
uint8_t sensor_name_index = 0xFF; 
const char *sensor_name = NULL;
const char *dev_name_comp[8] = {NULL};
#if 0
char dev_name_compare[6][26] = {
	"vi-output, ar0230 30-0043",
	"vi-output, ar0230 30-0044",
	"vi-output, ar0230 31-0043",
	"vi-output, ar0230 31-0044",
	"vi-output, ar0230 32-0043",
	"vi-output, ar0230 32-0044"
};
#endif
extern int frame_index_assign;
int serdes_err_handle(struct econ_stream_monitor *err_cam);
int ser_des_chip_link_status(struct econ_stream_monitor *err_cam);
int des_chip_gmsl_link_lock_status(struct econ_stream_monitor *err_cam,	uint8_t err_handle);
int ser_chip_video_status(struct econ_stream_monitor *err_cam,uint8_t err_handle);
int ser1_detect(struct econ_stream_monitor *err_cam,uint8_t err_handle);
int ser2_detect(struct econ_stream_monitor *err_cam,uint8_t err_handle);
int check_ser2_for_ser1_error(struct econ_stream_monitor *err_cam);
int check_ser1_for_ser2_error(struct econ_stream_monitor *err_cam);
int ser2_i2c_reassignment(struct econ_stream_monitor *err_cam);

int check_ser_i2c_translation(struct econ_stream_monitor *err_cam);
int i2c_trans_reg_read( struct econ_stream_monitor *err_cam,uint16_t reg_addr);
int re_write_i2c_trans_settings(struct econ_stream_monitor *err_cam,
		SERDES_PARSE *ser_conf,u32 reg_cnt);
int mcu_err_handle(struct econ_stream_monitor *err_cam,	uint8_t err_handle);
int isp_sensor_err_handle(struct econ_stream_monitor *err_cam,	uint8_t err_handle);
int sensor_isp_write(struct i2c_client *err_client,uint16_t reg_addr, uint16_t reg_val);
int sensor_isp_read(struct i2c_client *err_client, uint16_t reg_addr, uint8_t reg_len);
int sensor_frame_count_read (struct i2c_client *err_client);
int sensor_chip_id_read (struct i2c_client *err_client);
uint8_t check_serializer_power_disconnect(struct econ_stream_monitor *err_cam);
int find_err_cam(char *err_cam_name);
#if 0
static int mcu_mipi_configuration(struct i2c_client *client, struct isx031 *priv, u8 cmd_id);
#endif
/*
int ser_des_chip_status(struct i2c_client *err_client, struct cam *err_priv);
int des_chip_gmsl_link_lock_status(struct econ_stream_monitor *err_cam);
int ser_chip_video_status(struct econ_stream_monitor *err_cam);
int ser_des_link_status_check(struct i2c_client *err_client, struct cam *err_priv);
int mcu_err_handle(struct i2c_client *err_client, struct cam *err_priv);
int isp_sensor_err_handle(struct i2c_client *err_client, struct cam *err_priv);
*/
