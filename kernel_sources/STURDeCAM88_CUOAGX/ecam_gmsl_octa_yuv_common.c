/*
 * Copyright (c) 2024-2025, e-con Systems.  All rights reserved.
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

#include <linux/ktime.h>

#include "ecam_gmsl_octa_yuv_common.h"
#include "serdes.h"
#include "serdes_common.h"
#include "mcu_firmware.h"

/*
 * Debug trace helper for the camera init sequence.
 * Enable at module load with: modprobe ecam_gmsl_yuv_common cam_init_trace=1
 * or at build time by defining CAM_INIT_TRACE_DEFAULT to 1.
 */
#ifndef CAM_INIT_TRACE_DEFAULT
#define CAM_INIT_TRACE_DEFAULT 1
#endif

static bool cam_init_trace = CAM_INIT_TRACE_DEFAULT;
module_param(cam_init_trace, bool, 0644);
MODULE_PARM_DESC(cam_init_trace,
	"Enable [CAM-TRACE] debug prints across the camera init sequence (default: on)");

#define cam_trace(dev, fmt, ...) \
	do { \
		if (cam_init_trace) \
			dev_info((dev), "[CAM-TRACE] " fmt, ##__VA_ARGS__); \
	} while (0)

int ox08b40_sensor_read_SET(struct i2c_client *err_client, uint16_t reg_addr, uint8_t reg_len)
{
	uint8_t mc_data[512], err = ERRCODE_SUCCESS;
	uint32_t reg_val = 0;
	uint16_t payload_len = 0;

	payload_len = 0x03;
	/*lock semaphore*/
	mutex_lock(&g_i2c_mutex);

	mc_data[0] = CMD_SIGNATURE;
	mc_data[1] = CMD_ID_SENSOR_READ_SET;
	mc_data[2] = payload_len >> 8;
	mc_data[3] = payload_len & 0xFF;
	mc_data[4] = errorcheck(&mc_data[2], 2);
	err = cam_write(err_client, mc_data, TX_LEN_PKT);
	msleep(20);
	if (err != ERRCODE_SUCCESS) {
		dev_err(&err_client->dev, " %s(%d) MCU Write Error - %d\n", __func__,
				__LINE__, err);
		goto exit;
	}
	msleep(20);

	mc_data[0] = CMD_SIGNATURE;
	mc_data[1] = CMD_ID_SENSOR_READ_SET;
	mc_data[2] = reg_addr >> 8;
	mc_data[3] = reg_addr & 0xFF;
	mc_data[4] = reg_len;
	mc_data[5] = errorcheck(&mc_data[2], 3);

	err = cam_write(err_client, mc_data, RX_LEN_PKT);
	msleep(20);
	if (err != ERRCODE_SUCCESS) {
		dev_err(&err_client->dev, " %s(%d) MCU Write Error - %d\n", __func__,
				__LINE__, err);
		goto exit;
	}

exit:
	/* unlock semaphore */
	mutex_unlock(&g_i2c_mutex);
	if (err == ERRCODE_SUCCESS)
		return reg_val;
	else
		return err;
}

int ox08b40_sensor_read_GET(struct i2c_client *err_client, uint16_t reg_addr, uint8_t reg_len)
{
	uint8_t mc_data[512], mc_ret_data[512], err = ERRCODE_SUCCESS;
	uint32_t reg_val = 0;
	uint16_t send_len = 0, payload_len = 0;

	payload_len = 0x01;
	/*lock semaphore*/
	mutex_lock(&g_i2c_mutex);

	mc_data[0] = CMD_SIGNATURE;
	mc_data[1] = CMD_ID_SENSOR_READ_GET;
	mc_data[2] = payload_len >> 8;
	mc_data[3] = payload_len & 0xFF;
	mc_data[4] = errorcheck(&mc_data[2], 2);
	err = cam_write(err_client, mc_data, TX_LEN_PKT);
	msleep(20);
	if (err != ERRCODE_SUCCESS) {
		dev_err(&err_client->dev, " %s(%d) MCU Write Error - %d\n", __func__,
				__LINE__, err);
		goto exit;
	}
	msleep(20);

	mc_data[0] = CMD_SIGNATURE;
	mc_data[1] = CMD_ID_SENSOR_READ_GET;
	mc_data[2] = 0x00;
	mc_data[3] = 0x00;
	mc_data[4] = errorcheck(&mc_data[2], 2);

	err = cam_write(err_client, mc_data, 4);
	msleep(20);
	if (err != ERRCODE_SUCCESS) {
		dev_err(&err_client->dev, " %s(%d) MCU Write Error - %d\n", __func__,
				__LINE__, err);
		goto exit;
	}

	memset(mc_ret_data, 0, 512);
	err = cam_read(err_client, mc_ret_data, 5);
	msleep(20);
	if (err != 0) {
		dev_err(&err_client->dev, " %s(%d) Error - %d\n", __func__,
				__LINE__, err);
		goto exit;
	}
	send_len = (mc_ret_data[2] << 8) | mc_ret_data[3];

	memset(mc_ret_data, 0, 512);
	/*NEED TO CHECK*/
	msleep(20);
	err = cam_read(err_client, mc_ret_data, send_len + HEADER_FOOTER_SIZE);
	msleep(20);
	if (err != 0) {
		dev_err(&err_client->dev, " %s(%d) Error - %d\n", __func__,
				__LINE__, err);
		goto exit;
	}

	switch (send_len-2) {
	case BYTE_1:
		reg_val = mc_ret_data[4];
		break;
	case BYTE_2:
		reg_val = (mc_ret_data[4] << 8) | mc_ret_data[5];
		break;
	case BYTE_3:
		reg_val = (mc_ret_data[4] << 16) | (mc_ret_data[5] << 8) | mc_ret_data[6];
		break;
	case BYTE_4:
		reg_val = (mc_ret_data[4] << 24) | (mc_ret_data[5] << 16) |
		(mc_ret_data[6] << 8) | mc_ret_data[7];
		break;
	}

exit:
	/* unlock semaphore */
	mutex_unlock(&g_i2c_mutex);
	if (err == ERRCODE_SUCCESS)
		return reg_val;
	else
		return err;
}

int ox08b40_sensor_read(struct i2c_client *err_client, uint16_t reg_addr,
		uint8_t reg_len)
{
	uint16_t reg_val = 0;

	ox08b40_sensor_read_SET(err_client, reg_addr, reg_len);
	msleep(100);
	reg_val = ox08b40_sensor_read_GET(err_client, reg_addr, reg_len);
	return reg_val;
}

int sensor_isp_write(struct i2c_client *err_client,
			uint16_t reg_addr, uint16_t reg_val)
{
	uint8_t mc_data[512], err = 0;

	mc_data[0] = CMD_SIGNATURE;
	mc_data[1] = CMD_ID_ISP_WRITE;
	mc_data[2] = 0x00;
	mc_data[3] = 0x05;
	mc_data[4] = errorcheck(&mc_data[2], 2);

	cam_write(err_client, mc_data, TX_LEN_PKT);
	if (err != ERRCODE_SUCCESS) {
		dev_err(&err_client->dev, " %s(%d) MCU Write Error - %d\n"
					, __func__, __LINE__, err);
		err = -EINVAL;
		goto exit;
	}

	msleep(20);

	mc_data[0] = CMD_SIGNATURE;
	mc_data[1] = CMD_ID_ISP_WRITE;
	mc_data[2] = reg_addr >> 8;
	mc_data[3] = reg_addr & 0xFF;
	mc_data[4] = 0x02;
	mc_data[5] = reg_val >> 8;
	mc_data[6] = reg_val & 0xFF;
	mc_data[7] = errorcheck(&mc_data[2], 5);

	err = cam_write(err_client, mc_data, 8);

	msleep(20);

	if (err != ERRCODE_SUCCESS) {
		dev_err(&err_client->dev, " %s(%d) MCU Write Error - %d\n"
					, __func__, __LINE__, err);
		err = -EINVAL;
		goto exit;
	}
	msleep(100);


exit:
	return err;

}

int sensor_isp_read(struct i2c_client *err_client, uint16_t reg_addr,
							uint8_t reg_len)
{
	uint8_t mc_data[512], mc_ret_data[512], err;
	uint16_t reg_val = 0;
	uint16_t send_len = 0;

	msleep(20);

	mc_data[0] = CMD_SIGNATURE;
	mc_data[1] = CMD_ID_ISP_READ;
	mc_data[2] = 0x00;
	mc_data[3] = 0x03;
	mc_data[4] = errorcheck(&mc_data[2], 2);

	err = cam_write(err_client, mc_data, TX_LEN_PKT);
	if (err != ERRCODE_SUCCESS) {
		dev_err(&err_client->dev, " %s(%d) MCU Write Error - %d\n",
			__func__, __LINE__, err);
	}

	mc_data[0] = CMD_SIGNATURE;
	mc_data[1] = CMD_ID_ISP_READ;
	mc_data[2] = reg_addr >> 8;
	mc_data[3] = reg_addr & 0xFF;
	mc_data[4] = reg_len;
	mc_data[5] = errorcheck(&mc_data[2], 3);

	err = cam_write(err_client, mc_data, RX_LEN_PKT);

	msleep(20);

	if (err != ERRCODE_SUCCESS) {
		dev_err(&err_client->dev, " %s(%d) MCU Write Error - %d\n",
			__func__, __LINE__, err);
	}

	memset(mc_ret_data, 0, 512);
	err = cam_read(err_client, mc_ret_data, RX_LEN_PKT);

	msleep(20);

	if (err != ERRCODE_SUCCESS) {
		dev_err(&err_client->dev, " %s(%d) Error - %d\n", __func__,
				__LINE__, err);
	}
	send_len = (mc_ret_data[2] << 8) | mc_ret_data[3];

	memset(mc_ret_data, 0, 512);
	err = cam_read(err_client, mc_ret_data, send_len + HEADER_FOOTER_SIZE);

	msleep(20);

	if (err != ERRCODE_SUCCESS) {
		dev_err(&err_client->dev, " %s(%d) Error - %d\n", __func__,
				__LINE__, err);
	}

	reg_val = mc_ret_data[4] << 8;
	reg_val = reg_val | mc_ret_data[5];

	return reg_val;
}

static int mcu_cam_stream_on(struct i2c_client *client)
{
	uint32_t payload_len = 0;

	uint16_t cmd_status = 0;
	uint8_t retcode = 0, cmd_id = 0;
	int retry = 5, status_retry = 1000, err = 0;
	uint8_t mc_data[512];

	/*lock semaphore*/
	mutex_lock(&g_i2c_mutex);

	while (retry-- > 0) {
		/* First Txn Payload length = 0 */
		payload_len = 0;

		mc_data[0] = CMD_SIGNATURE;
		mc_data[1] = CMD_ID_STREAM_ON;
		mc_data[2] = payload_len >> 8;
		mc_data[3] = payload_len & 0xFF;
		mc_data[4] = errorcheck(&mc_data[2], 2);

		err = cam_write(client, mc_data, TX_LEN_PKT);

		msleep(20);

		if (err != 0) {
			dev_err(&client->dev,
				" %s(%d) MCU Stream On Write Error - %d\n",
					__func__, __LINE__, err);
			continue;
		}

		mc_data[0] = CMD_SIGNATURE;
		mc_data[1] = CMD_ID_STREAM_ON;
		err = cam_write(client, mc_data, 2);

		msleep(20);

		if (err != 0) {
			dev_err(&client->dev,
				" %s(%d) MCU Stream On Write Error - %d\n",
					__func__, __LINE__, err);
			continue;
		}

		while (status_retry-- > 0) {
			/* Some Sleep for init to process */
			cmd_id = CMD_ID_STREAM_ON;
			if (mcu_get_cmd_status(client,
				&cmd_id, &cmd_status, &retcode) < 0) {
				dev_err(&client->dev,
					" %s(%d) MCU Get CMD Stream On Error",
					       __func__, __LINE__);
				err = -1;
				goto exit;
			}

			if ((cmd_status == MCU_CMD_STATUS_SUCCESS) &&
					(retcode == ERRCODE_SUCCESS)) {
				pr_info(" %s %d MCU Stream On Success !!",
						__func__, __LINE__);
				err = 0;
				goto exit;
			}

			if ((retcode != ERRCODE_BUSY) &&
				((cmd_status != MCU_CMD_STATUS_PENDING))) {
				dev_err(&client->dev,
					"(%s) %d MCU Get CMD Stream On ErrorSTATUS = 0x%04x RET = 0x%02x",
							__func__, __LINE__,
								cmd_status, retcode);
				err = -1;
				goto exit;
			}
			mdelay(1);
		}
		if (retry == 0)
			err = -1;
		break;
	}
	msleep(20);
exit:
	/* unlock semaphore */
	mutex_unlock(&g_i2c_mutex);
	return err;

}


static int mcu_cam_stream_off(struct i2c_client *client)
{
	uint32_t payload_len = 0;

	uint16_t cmd_status = 0;
	uint8_t retcode = 0, cmd_id = 0;
	int retry = 1000, err = 0;

	/* call ISP init command */
	uint8_t mc_data[512];

	/*lock semaphore*/
	mutex_lock(&g_i2c_mutex);

	/* First Txn Payload length = 0 */
	payload_len = 0;

	mc_data[0] = CMD_SIGNATURE;
	mc_data[1] = CMD_ID_STREAM_OFF;
	mc_data[2] = payload_len >> 8;
	mc_data[3] = payload_len & 0xFF;
	mc_data[4] = errorcheck(&mc_data[2], 2);

	cam_write(client, mc_data, TX_LEN_PKT);

	msleep(20);

	mc_data[0] = CMD_SIGNATURE;
	mc_data[1] = CMD_ID_STREAM_OFF;
	err = cam_write(client, mc_data, 2);

	msleep(20);

	if (err != 0) {
		dev_err(&client->dev,
			" %s(%d) CAM Stream OFF Write Error - %d\n",
				__func__, __LINE__, err);
		goto exit;
	}

	while (--retry > 0) {
		/* Some Sleep for init to process */

		cmd_id = CMD_ID_STREAM_OFF;
		if (mcu_get_cmd_status(client, &cmd_id, &cmd_status, &retcode) <
				0) {
			dev_err(&client->dev,
				" %s(%d) CAM Get CMD Stream Off Error\n",
					__func__, __LINE__);
			err = -1;
			goto exit;
		}

		if ((cmd_status == MCU_CMD_STATUS_SUCCESS) &&
				(retcode == ERRCODE_SUCCESS)) {
			pr_info(" %s %d CAM Get CMD Stream off Success !!\n",
					__func__, __LINE__);
			err = 0;
			goto exit;
		}

		if ((retcode != ERRCODE_BUSY) &&
				((cmd_status != MCU_CMD_STATUS_PENDING))) {
			dev_err(&client->dev,
					"(%s) %d CAM Get CMD Stream offError STATUS = 0x%04x RET = 0x%02x\n",
					__func__, __LINE__, cmd_status, retcode);
			err = -1;
			goto exit;
		}
		mdelay(1);
	}
exit:
	/* unlock semaphore */
	mutex_unlock(&g_i2c_mutex);
	return err;
}

static int mcu_get_ctrl_ui(struct i2c_client *client,
		ISP_CTRL_INFO *mcu_ui_info, int index)
{
	uint32_t payload_len = 0;
	uint8_t errcode = ERRCODE_SUCCESS, orig_crc = 0, calc_crc = 0;
	int ret = 0, i = 0, err = 0;
	uint8_t mc_data[512], mc_ret_data[512];

	/* lock semaphore */
	mutex_lock(&g_i2c_mutex);

	/* First Txn Payload length = 0 */
	payload_len = 2;

	mc_data[0] = CMD_SIGNATURE;
	mc_data[1] = CMD_ID_GET_CTRL_UI_INFO;
	mc_data[2] = payload_len >> 8;
	mc_data[3] = payload_len & 0xFF;
	mc_data[4] = errorcheck(&mc_data[2], 2);

	cam_write(client, mc_data, TX_LEN_PKT);

	msleep(20);

	mc_data[0] = CMD_SIGNATURE;
	mc_data[1] = CMD_ID_GET_CTRL_UI_INFO;
	mc_data[2] = index >> 8;
	mc_data[3] = index & 0xFF;
	mc_data[4] = errorcheck(&mc_data[2], 2);
	err = cam_write(client, mc_data, 5);

	msleep(20);

	if (err != 0) {
		dev_err(&client->dev, " %s(%d) Error - %d\n", __func__,
				__LINE__, err);
		ret = -EIO;
		goto exit;
	}

	err = cam_read(client, mc_ret_data, RX_LEN_PKT);

	msleep(20);

	if (err != 0) {
		dev_err(&client->dev, " %s(%d) Error - %d\n", __func__,
				__LINE__, err);
		ret = -EIO;
		goto exit;
	}

	/* Verify CRC */
	orig_crc = mc_ret_data[4];
	calc_crc = errorcheck(&mc_ret_data[2], 2);
	if (orig_crc != calc_crc) {
		dev_err(&client->dev, " %s(%d) CRC 0x%02x != 0x%02x\n",
				__func__, __LINE__, orig_crc, calc_crc);
		ret = -EINVAL;
		goto exit;
	}

	payload_len =
		((mc_ret_data[2] << 8) | mc_ret_data[3]) + HEADER_FOOTER_SIZE;
	errcode = mc_ret_data[5];
	if (errcode != ERRCODE_SUCCESS) {
		dev_err(&client->dev, " %s(%d) Errcode - 0x%02x\n",
				__func__, __LINE__, errcode);
		ret = -EINVAL;
		goto exit;
	}

	memset(mc_ret_data, 0x00, payload_len);
	err = cam_read(client, mc_ret_data, payload_len);

	msleep(20);

	if (err != 0) {
		dev_err(&client->dev, " %s(%d) Error - %d\n", __func__,
				__LINE__, err);
		ret = -EIO;
		goto exit;
	}

	/* Verify CRC */
	orig_crc = mc_ret_data[payload_len - 2];
	calc_crc =
		errorcheck(&mc_ret_data[2], payload_len - HEADER_FOOTER_SIZE);
	if (orig_crc != calc_crc) {
		dev_err(&client->dev, " %s(%d) CRC 0x%02x != 0x%02x\n",
				__func__, __LINE__, orig_crc, calc_crc);
		ret = -EINVAL;
		goto exit;
	}

	/* Verify Errcode */
	errcode = mc_ret_data[payload_len - 1];
	if (errcode != ERRCODE_SUCCESS) {
		dev_err(&client->dev, " %s(%d) Errcode - 0x%02x\n",
				__func__, __LINE__, errcode);
		ret = -EIO;
		goto exit;
	}

	strncpy((char *)mcu_ui_info->ctrl_ui_data.ctrl_ui_info.ctrl_name, &mc_ret_data[2], MAX_CTRL_UI_STRING_LEN);

	mcu_ui_info->ctrl_ui_data.ctrl_ui_info.ctrl_ui_type = mc_ret_data[34];
	mcu_ui_info->ctrl_ui_data.ctrl_ui_info.ctrl_ui_flags = mc_ret_data[35] << 8 |
		mc_ret_data[36];

	if (mcu_ui_info->ctrl_ui_data.ctrl_ui_info.ctrl_ui_type == V4L2_CTRL_TYPE_MENU) {
		mcu_ui_info->ctrl_ui_data.ctrl_menu_info.num_menu_elem = mc_ret_data[37];

		mcu_ui_info->ctrl_ui_data.ctrl_menu_info.menu =
			devm_kzalloc(&client->dev,
				((mcu_ui_info->ctrl_ui_data.ctrl_menu_info.num_menu_elem + 1) * sizeof(char *)), GFP_KERNEL);
		for (i = 0; i < mcu_ui_info->ctrl_ui_data.ctrl_menu_info.num_menu_elem; i++) {
			mcu_ui_info->ctrl_ui_data.ctrl_menu_info.menu[i] =
				devm_kzalloc(&client->dev, MAX_CTRL_UI_STRING_LEN, GFP_KERNEL);
			strncpy((char *)mcu_ui_info->ctrl_ui_data.ctrl_menu_info.menu[i],
					&mc_ret_data[38 + (i * MAX_CTRL_UI_STRING_LEN)], MAX_CTRL_UI_STRING_LEN);

			pr_info(" Menu Element %d : %s\n",
					i, mcu_ui_info->ctrl_ui_data.ctrl_menu_info.menu[i]);
		}

		mcu_ui_info->ctrl_ui_data.ctrl_menu_info.menu[i] = NULL;
	}

exit:
	/* unlock semaphore */
	mutex_unlock(&g_i2c_mutex);

	return ret;
}


int mcu_set_ctrl(struct i2c_client *client, uint32_t arg_ctrl_id,
		uint8_t ctrl_type, int32_t curr_val)
{
	struct camera_common_data *s_data = to_camera_common_data(&client->dev);
	struct cam *priv = (struct cam *)s_data->priv;
	uint8_t mc_data[512];

	uint32_t payload_len = 0;

	uint16_t cmd_status = 0, index = 0xFFFF;
	uint8_t retcode = 0, cmd_id = 0;
	int loop = 0, ret = 0, err = 0, retry = 1000;
	uint32_t ctrl_id = 0;

	/* lock semaphore */
	mutex_lock(&g_i2c_mutex);

	ctrl_id = arg_ctrl_id;

	if (arg_ctrl_id == V4L2_CID_PWM_CONFIG)
		priv->pwm_sync = curr_val;

	if (priv->cam_id == Ox08B40 && (arg_ctrl_id == V4L2_CID_FRAME_SYNC || arg_ctrl_id == V4L2_CID_PWM_CONFIG)) {
		priv->trigger_mode = curr_val;
		if (priv->trigger_mode == 0) {
			dev_info(&client->dev, "Master mode set_I\n");
			if (serdes_write_16b_reg(client, priv->ser_reassigned_addr, 0x02D3, MCU_RST_HIGH) < 0) {
				dev_err(&client->dev, "%s(%d): Failed\n", __func__, __LINE__);
				return -EIO;
			}
			if (priv->got_pwm_handle) {
				pwm_disable(priv->pwm_dev);
				dev_info(&client->dev, "PWM disabled\n");
			}
		} else if (priv->trigger_mode == 1 || priv->trigger_mode == 2) {
			dev_info(&client->dev, "Trigger mode set_I\n");
			if (serdes_write_16b_reg(client, priv->ser_reassigned_addr, 0x02D3, 0xC4) < 0) {
				dev_err(&client->dev, "%s(%d): Failed\n", __func__, __LINE__);
				return -EIO;
			}

			if (serdes_write_16b_reg(client, priv->ser_reassigned_addr, 0x02D5, 0x27) < 0) {
				dev_err(&client->dev, "%s(%d): Failed\n", __func__, __LINE__);
				return -EIO;
			}
			if (priv->got_pwm_handle) {
				pwm_enable(priv->pwm_dev);
				dev_info(&client->dev, "PWM enabled\n");
			}
		}
	}

	/* call ISP Ctrl config command */
	for (loop = 0; loop < priv->num_ctrls; loop++) {
		if (priv->ctrldb[loop] == ctrl_id) {
			index = loop;
			break;
		}
	}

	if (index == 0xFFFF) {
		ret = -EINVAL;
		goto exit;
	}

	/* First Txn Payload length = 0 */
	payload_len = 11;

	mc_data[0] = CMD_SIGNATURE;
	mc_data[1] = CMD_ID_SET_CTRL;
	mc_data[2] = payload_len >> 8;
	mc_data[3] = payload_len & 0xFF;
	mc_data[4] = errorcheck(&mc_data[2], 2);

	cam_write(client, mc_data, TX_LEN_PKT);

	msleep(20);

	/* Second Txn */
	mc_data[0] = CMD_SIGNATURE;
	mc_data[1] = CMD_ID_SET_CTRL;

	/* Index */
	mc_data[2] = index >> 8;
	mc_data[3] = index & 0xFF;

	/* Control ID */
	mc_data[4] = ctrl_id >> 24;
	mc_data[5] = ctrl_id >> 16;
	mc_data[6] = ctrl_id >> 8;
	mc_data[7] = ctrl_id & 0xFF;

	/* Ctrl Type */
	mc_data[8] = ctrl_type;

	/* Ctrl Value */
	mc_data[9] = curr_val >> 24;
	mc_data[10] = curr_val >> 16;
	mc_data[11] = curr_val >> 8;
	mc_data[12] = curr_val & 0xFF;

	/* CRC */
	mc_data[13] = errorcheck(&mc_data[2], 11);

	err = cam_write(client, mc_data, 14);

	msleep(20);

	if (err != 0) {
		dev_err(&client->dev, " %s(%d) Error - %d\n", __func__,
				__LINE__, err);
		ret = -EIO;
		goto exit;
	}

	while (retry-- > 0) {
		cmd_id = CMD_ID_SET_CTRL;
		if (mcu_get_cmd_status
				(client, &cmd_id, &cmd_status, &retcode) < 0) {
			dev_err(&client->dev, " %s(%d) Error\n",
					__func__, __LINE__);
			ret = -EINVAL;
			goto exit;
		}

		if ((cmd_status == MCU_CMD_STATUS_SUCCESS) &&
				(retcode == ERRCODE_SUCCESS)) {
			ret = 0;
			goto exit;
		}

		if ((retcode != ERRCODE_BUSY) &&
				((cmd_status != MCU_CMD_STATUS_PENDING))) {
			pr_err
				("(%s) %d ISP Error STATUS = 0x%04x RET = 0x%02x\n",
				 __func__, __LINE__, cmd_status, retcode);
			ret = -EIO;
			goto exit;
		}
		msleep(20);
	}
	if (retry <= 0) {
		pr_err
			("(%s) %d Error setting control = 0x%04x RET = 0x%02x\n",
			 __func__, __LINE__, cmd_status, retcode);
		ret = -EIO;
		goto exit;

	}
exit:
	/* unlock semaphore */
	mutex_unlock(&g_i2c_mutex);

	return ret;
}

static int cam_g_volatile_ctrl(struct v4l2_ctrl *ctrl)
{
	struct cam *priv =
		container_of(ctrl->handler, struct cam, ctrl_handler);
	struct i2c_client *client = priv->i2c_client;
	int err = 0;

	uint8_t ctrl_type = 0;
	int ctrl_val = 0;

	if (!priv || !priv->pdata)
		return -EINVAL;

	if (priv->power.state == SWITCH_OFF)
		return 0;

	err = mcu_get_ctrl(client, ctrl->id, &ctrl_type, &ctrl_val);
	if (err < 0)
		return err;

	if (ctrl_type == CTRL_STANDARD) {
		ctrl->val = ctrl_val;
	} else {
		/* Not Implemented */
		return -EINVAL;
	}

	return err;
}

static int cam_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct cam *priv =
		container_of(ctrl->handler, struct cam, ctrl_handler);
	struct i2c_client *client = priv->i2c_client;
	int err = 0, retry = 5, loop;
	uint8_t cam_num = 0;
	bool found = false;

	struct camera_common_data *s_data = to_camera_common_data(&client->dev);
	int mode = 0;

	mode = s_data->mode;

	if (!priv || !priv->pdata)
		return -EINVAL;

	if (priv->power.state == SWITCH_OFF)
		return 0;

	cam_num = priv->cam_num;

	for (loop = 0; loop < available_ctrls_count; loop++) {
#ifdef DEBUG
		pr_warn("ctrl->id = 0x%08x , available ctrl id = 0x%08x\n", ctrl->id, available_ctrls[loop].ctrl_id);
#endif
		if (ctrl->id == available_ctrls[loop].ctrl_id) {
			available_ctrls[loop].curr_array[cam_num] = ctrl->val;
#ifdef DEBUG
			pr_info("cam_set_num: %d, ctrl_id: 0x%x, value: %d\n", cam_num, ctrl->id, available_ctrls[loop].curr_array[cam_num]);
#endif
			found = true;
			break;
		}
	}
	if (!found)
		dev_err(&client->dev, "CTRL 0x%x - Not supported\n", ctrl->id);

	while (retry-- > 0) {
#ifdef DEBUG
		dev_info(&client->dev, "mcu_set_ctrl: Name:%s[%x-%d]", ctrl->name, ctrl->id, ctrl->val);
#endif
		err = mcu_set_ctrl(client, ctrl->id, CTRL_STANDARD, ctrl->val);
		if (err < 0) {
			dev_info(&client->dev, " %s (%d ) retry\n", __func__, __LINE__);
			if (retry <= 0) {
				dev_err(&client->dev, " %s (%d )\n", __func__, __LINE__);
				break;
			}
			continue;
		}
		break;
	}

	return err;
}

int mcu_stream_config(struct i2c_client *client, uint32_t format,
		int mode, int frame_index)
{
	struct camera_common_data *s_data = to_camera_common_data(&client->dev);
	struct cam *priv = (struct cam *)s_data->priv;

	uint32_t payload_len = 0;

	uint16_t cmd_status = 0, index = 0xFFFF;
	uint8_t retcode = 0, cmd_id = 0;
	int loop = 0, ret = 0, err = 0, retry = 1000;
	uint8_t mc_data[512];

	/* lock semaphore */
	mutex_lock(&g_i2c_mutex);

	for (loop = 0; (&priv->streamdb[loop]) != NULL; loop++) {
		if (priv->streamdb[loop] == mode) {
			index = loop + frame_index;
			break;
		}
	}

	pr_info(" Index = 0x%04x , format = 0x%08x, width = %hu, height = %hu, frame num = %hu\n",
			index, format,
			priv->mcu_cam_frmfmt[mode].size.width,
			priv->mcu_cam_frmfmt[mode].size.height,
			priv->mcu_cam_frmfmt[mode].framerates[frame_index]);

	if (index == 0xFFFF) {
		ret = -EINVAL;
		goto exit;
	}

issue_cmd:
	/* First Txn Payload length = 0 */
	payload_len = 14;

	mc_data[0] = CMD_SIGNATURE;
	mc_data[1] = CMD_ID_STREAM_CONFIG;
	mc_data[2] = payload_len >> 8;
	mc_data[3] = payload_len & 0xFF;
	mc_data[4] = errorcheck(&mc_data[2], 2);

	cam_write(client, mc_data, TX_LEN_PKT);

	msleep(20);

	mc_data[0] = CMD_SIGNATURE;
	mc_data[1] = CMD_ID_STREAM_CONFIG;
	mc_data[2] = index >> 8;
	mc_data[3] = index & 0xFF;

	/* Format Fourcc - currently only UYVY */
	mc_data[4] = format >> 24;
	mc_data[5] = format >> 16;
	mc_data[6] = format >> 8;
	mc_data[7] = format & 0xFF;

	/* width */
	mc_data[8] = priv->mcu_cam_frmfmt[mode].size.width >> 8;
	mc_data[9] = priv->mcu_cam_frmfmt[mode].size.width & 0xFF;

	/* height */
	mc_data[10] = priv->mcu_cam_frmfmt[mode].size.height >> 8;
	mc_data[11] = priv->mcu_cam_frmfmt[mode].size.height & 0xFF;

	/* frame rate num */
	mc_data[12] = priv->mcu_cam_frmfmt[mode].framerates[frame_index] >> 8;
	mc_data[13] = priv->mcu_cam_frmfmt[mode].framerates[frame_index] & 0xFF;

	/* frame rate denom */
	mc_data[14] = 0x00;
	mc_data[15] = 0x01;

	mc_data[16] = errorcheck(&mc_data[2], 14);

	pr_info("Debugging CRC data for resolution.\n");
	pr_info("width1, width2, frame-rate-num1, num2, crc:<%d, %d, %d, %d, %d>\n",
			mc_data[8], mc_data[9], mc_data[12], mc_data[13], mc_data[16]);

	err = cam_write(client, mc_data, 17);

	msleep(20);

	if (err != 0) {
		dev_err(&client->dev, " %s(%d) Error - %d\n", __func__,
				__LINE__, err);
		ret = -EIO;
		goto exit;
	}

	while (--retry > 0) {
		cmd_id = CMD_ID_STREAM_CONFIG;
		if (mcu_get_cmd_status
				(client, &cmd_id, &cmd_status, &retcode) < 0) {
			dev_err(&client->dev,
					" %s(%d) MCU GET CMD Status Error : loop : %d\n",
					__func__, __LINE__, loop);
			ret = -EIO;
			goto exit;
		}

		if ((cmd_status == MCU_CMD_STATUS_SUCCESS) &&
				(retcode == ERRCODE_SUCCESS)) {
			ret = 0;
			goto exit;
		}

		if (retcode == ERRCODE_AGAIN) {
			/* Issue Command Again if Set */
			retry = 1000;
			goto issue_cmd;
		}

		if ((retcode != ERRCODE_BUSY) &&
				((cmd_status != MCU_CMD_STATUS_PENDING))) {
			dev_err(&client->dev,
					"(%s) %d Error STATUS = 0x%04x RET = 0x%02x\n",
					__func__, __LINE__, cmd_status, retcode);
			ret = -EIO;
			goto exit;
		}

		/* Delay after retry */
		mdelay(10);
	}

	dev_err(&client->dev, " %s(%d) Error - %d\n", __func__,
			__LINE__, err);
	ret = -ETIMEDOUT;

exit:
	if (!ret)
		priv->prev_index = index;

	/* unlock semaphore */
	mutex_unlock(&g_i2c_mutex);

	return ret;
}


int gen_mcu_stream_config(struct i2c_client *client, struct cam *priv)
{
	int  err = 0, retry = 5;

	if (priv->cam_id == Ox08B40) {
		if (priv->trigger_mode == 0) {

			dev_info(&client->dev, "Master mode set\n");
			if (serdes_write_16b_reg(client, priv->ser_reassigned_addr, 0x02D3, MCU_RST_HIGH) < 0) {
				dev_err(&client->dev, "%s(%d): Failed\n", __func__, __LINE__);
				return -EIO;
			}
			if (priv->got_pwm_handle) {
				pwm_disable(priv->pwm_dev);
				dev_info(&client->dev, "PWM disabled\n");
			}
		} else if (priv->trigger_mode == 1) {
			dev_info(&client->dev, "Trigger mode set\n");
			if (serdes_write_16b_reg(client, priv->ser_reassigned_addr, 0x02D3, 0xC4) < 0) {
				dev_err(&client->dev, "%s(%d): Failed\n", __func__, __LINE__);
				return -EIO;
			}

			if (serdes_write_16b_reg(client, priv->ser_reassigned_addr, 0x02D5, 0x27) < 0) {
				dev_err(&client->dev, "%s(%d): Failed\n", __func__, __LINE__);
				return -EIO;
			}
			if (priv->got_pwm_handle) {
				pwm_enable(priv->pwm_dev);
				dev_info(&client->dev, "PWM enabled\n");
			}
		}
	}

	while (retry-- > 0) {

		/* call stream config with width, height, frame rate */
		err =	mcu_stream_config(client, priv->format_fourcc,
			     priv->frmfmt_mode, priv->frame_index);
		if (err < ERRCODE_SUCCESS) {
			dev_err(&client->dev, "%s: Failed stream_config\n",
								__func__);
			if (retry != 0)
				continue;
			if (err < ERRCODE_SUCCESS) {
				dev_err(&client->dev, " %s (%d )\n",
							__func__, __LINE__);
				return err;
			}
		}

		mdelay(50);
		break;
	}
	if (retry <= 0) {
		dev_err(&client->dev, "%s(%d): Failed\n",
				__func__, __LINE__);
		return err;
	}

	return ERRCODE_SUCCESS;
}

static int mcu_get_ctrl(struct i2c_client *client, uint32_t arg_ctrl_id,
		uint8_t *ctrl_type, int32_t *curr_val)
{
	struct camera_common_data *s_data = to_camera_common_data(&client->dev);
	struct cam *priv = (struct cam *)s_data->priv;

	uint32_t payload_len = 0;
	uint8_t errcode = ERRCODE_SUCCESS, orig_crc = 0, calc_crc = 0;
	uint16_t index = 0xFFFF;
	int loop = 0, ret = 0, err = 0;
	uint8_t mc_data[512], mc_ret_data[512];

	uint32_t ctrl_id = 0;

	/* lock semaphore */
	mutex_lock(&g_i2c_mutex);

	ctrl_id = arg_ctrl_id;

	/* Read the Ctrl Value from Micro controller */

	for (loop = 0; loop < priv->num_ctrls; loop++) {
		if (priv->ctrldb[loop] == ctrl_id) {
			index = loop;
			break;
		}
	}

	if (index == 0xFFFF) {
		ret = -EINVAL;
		goto exit;
	}

	/* First Txn Payload length = 2 */
	payload_len = 2;

	mc_data[0] = CMD_SIGNATURE;
	mc_data[1] = CMD_ID_GET_CTRL;
	mc_data[2] = payload_len >> 8;
	mc_data[3] = payload_len & 0xFF;
	mc_data[4] = errorcheck(&mc_data[2], 2);

	cam_write(client, mc_data, TX_LEN_PKT);

	msleep(20);

	mc_data[0] = CMD_SIGNATURE;
	mc_data[1] = CMD_ID_GET_CTRL;
	mc_data[2] = index >> 8;
	mc_data[3] = index & 0xFF;
	mc_data[4] = errorcheck(&mc_data[2], 2);
	err = cam_write(client, mc_data, 5);

	msleep(20);

	if (err != 0) {
		dev_err(&client->dev, " %s(%d) Error - %d\n", __func__,
				__LINE__, err);
		ret = -EIO;
		goto exit;
	}

	err = cam_read(client, mc_ret_data, RX_LEN_PKT);

	msleep(20);

	if (err != 0) {
		dev_err(&client->dev, " %s(%d) Error - %d\n", __func__,
				__LINE__, err);
		ret = -EIO;
		goto exit;
	}

	/* Verify CRC */
	orig_crc = mc_ret_data[4];
	calc_crc = errorcheck(&mc_ret_data[2], 2);
	if (orig_crc != calc_crc) {
		dev_err(&client->dev, " %s(%d) CRC 0x%02x != 0x%02x\n",
				__func__, __LINE__, orig_crc, calc_crc);
		ret = -1;
		goto exit;
	}

	if (((mc_ret_data[2] << 8) | mc_ret_data[3]) == 0) {
		ret = -EIO;
		goto exit;
	}

	errcode = mc_ret_data[5];
	if (errcode != ERRCODE_SUCCESS) {
		dev_err(&client->dev, " %s(%d) Errcode - 0x%02x\n",
				__func__, __LINE__, errcode);
		ret = -EIO;
		goto exit;
	}

	payload_len =
		((mc_ret_data[2] << 8) | mc_ret_data[3]) + HEADER_FOOTER_SIZE;
	memset(mc_ret_data, 0x00, payload_len);
	err = cam_read(client, mc_ret_data, payload_len);

	msleep(20);

	if (err != 0) {
		dev_err(&client->dev, " %s(%d) Error - %d\n", __func__,
				__LINE__, err);
		ret = -EIO;
		goto exit;
	}

	/* Verify CRC */
	orig_crc = mc_ret_data[payload_len - 2];
	calc_crc =
		errorcheck(&mc_ret_data[2], payload_len - HEADER_FOOTER_SIZE);
	if (orig_crc != calc_crc) {
		dev_err(&client->dev, " %s(%d) CRC 0x%02x != 0x%02x\n",
				__func__, __LINE__, orig_crc, calc_crc);
		ret = -EINVAL;
		goto exit;
	}

	/* Verify Errcode */
	errcode = mc_ret_data[payload_len - 1];
	if (errcode != ERRCODE_SUCCESS) {
		dev_err(&client->dev, " %s(%d) Errcode - 0x%02x\n",
				__func__, __LINE__, errcode);
		ret = -EINVAL;
		goto exit;
	}

	/* Ctrl type starts from index 6 */
	*ctrl_type = mc_ret_data[6];

	switch (*ctrl_type) {
	case CTRL_STANDARD:
		*curr_val =
			mc_ret_data[7] << 24 | mc_ret_data[8] << 16 | mc_ret_data[9]
			<< 8 | mc_ret_data[10];
		break;

	case CTRL_EXTENDED:
		/* Not Implemented */
		break;
	}

exit:
	/* unlock semaphore */
	mutex_unlock(&g_i2c_mutex);

	return ret;
}

static const struct v4l2_ctrl_ops cam_ctrl_ops = {
	.g_volatile_ctrl = cam_g_volatile_ctrl,
	.s_ctrl = cam_s_ctrl,
};

static int cam_try_add_ctrls(struct cam *priv, int index,
		ISP_CTRL_INFO *mcu_ctrl)
{
	struct i2c_client *client = priv->i2c_client;
	struct v4l2_ctrl_config custom_ctrl_config;

	if (!priv || !priv->pdata)
		return -EINVAL;

	if (mcu_ctrl->ctrl_id == V4L2_CID_FRAME_RATE_CONTROL)
		return 0;

	priv->ctrl_handler.error = 0;

	/* Try Enumerating in standard controls */
	priv->ctrls[index] =
		v4l2_ctrl_new_std(&priv->ctrl_handler,
				&cam_ctrl_ops,
				mcu_ctrl->ctrl_id,
				mcu_ctrl->ctrl_data.std.ctrl_min,
				mcu_ctrl->ctrl_data.std.ctrl_max,
				mcu_ctrl->ctrl_data.std.ctrl_step,
				mcu_ctrl->ctrl_data.std.ctrl_def);
	if (priv->ctrls[index] != NULL) {
		pr_info("%d. Initialized Control 0x%08x - %s\n",
				index, mcu_ctrl->ctrl_id,
				priv->ctrls[index]->name);
		return 0;
	}

	if (mcu_ctrl->ctrl_id == V4L2_CID_EXPOSURE_AUTO)
		goto custom;

	/* Try Enumerating in standard menu */
	priv->ctrl_handler.error = 0;
	priv->ctrls[index] =
		v4l2_ctrl_new_std_menu(&priv->ctrl_handler,
				&cam_ctrl_ops,
				mcu_ctrl->ctrl_id,
				mcu_ctrl->ctrl_data.std.ctrl_max,
				0, mcu_ctrl->ctrl_data.std.ctrl_def);
	if (priv->ctrls[index] != NULL) {
		pr_info("%d. Initialized Control Menu 0x%08x - %s\n",
				index, mcu_ctrl->ctrl_id,
				priv->ctrls[index]->name);
		return 0;
	}

custom:
	priv->ctrl_handler.error = 0;
	memset(&custom_ctrl_config, 0x0, sizeof(struct v4l2_ctrl_config));

	if (mcu_get_ctrl_ui(client, mcu_ctrl, index) != ERRCODE_SUCCESS) {
		dev_err(&client->dev, "Error Enumerating Control 0x%08x !!\n",
				mcu_ctrl->ctrl_id);
		return -EIO;
	}

	/* Fill in Values for Custom Ctrls */
	custom_ctrl_config.ops = &cam_ctrl_ops;
	custom_ctrl_config.id = mcu_ctrl->ctrl_id;

	/* Do not change the name field for the control */
	custom_ctrl_config.name = mcu_ctrl->ctrl_ui_data.ctrl_ui_info.ctrl_name;

	/* Sample Control Type and Flags */
	custom_ctrl_config.type = mcu_ctrl->ctrl_ui_data.ctrl_ui_info.ctrl_ui_type;
	custom_ctrl_config.flags = mcu_ctrl->ctrl_ui_data.ctrl_ui_info.ctrl_ui_flags;

	custom_ctrl_config.min = mcu_ctrl->ctrl_data.std.ctrl_min;
	custom_ctrl_config.max = mcu_ctrl->ctrl_data.std.ctrl_max;
	custom_ctrl_config.step = mcu_ctrl->ctrl_data.std.ctrl_step;
	custom_ctrl_config.def = mcu_ctrl->ctrl_data.std.ctrl_def;

	if (custom_ctrl_config.type == V4L2_CTRL_TYPE_MENU) {
		custom_ctrl_config.step = 0;
		custom_ctrl_config.type_ops = NULL;

		custom_ctrl_config.qmenu =
			(const char *const *)(mcu_ctrl->ctrl_ui_data.ctrl_menu_info.menu);
	}

	priv->ctrls[index] =
		v4l2_ctrl_new_custom(&priv->ctrl_handler,
				&custom_ctrl_config, NULL);

	if (priv->ctrls[index] != NULL) {
		pr_info("%d. Initialized Custom Ctrl 0x%08x - %s\n",
				index, mcu_ctrl->ctrl_id,
				priv->ctrls[index]->name);
		return 0;
	}

	dev_err(&client->dev,
			"%d.  default: Failed to init 0x%08x ctrl Error - %d\n",
			index, mcu_ctrl->ctrl_id, priv->ctrl_handler.error);
	return -EINVAL;
}

static int cam_ctrls_init(struct cam *priv, ISP_CTRL_INFO *mcu_cam_ctrls)
{
	struct i2c_client *client = priv->i2c_client;
	int err = 0, i = 0;
	int retry = 5;

	/* Custom Ctrl */
	if (!priv || !priv->pdata)
		return -EINVAL;

	while (--retry >= 0) {
		err = mcu_list_ctrls(client, mcu_cam_ctrls, priv);
		if (err < 0) {
			dev_err(&client->dev,
				"Init ctrls failure. retrying\n");
			continue;
		} else {
			break;
		}
	}

	if (retry < 0) {
		dev_err(&client->dev, "(%s): Unable to init ctrls\n",
							__func__);
		goto error;
	}

	v4l2_ctrl_handler_init(&priv->ctrl_handler, priv->num_ctrls+1);
	priv->subdev->ctrl_handler = &priv->ctrl_handler;
	for (i = 0; i < priv->num_ctrls; i++) {

		if (mcu_cam_ctrls[i].ctrl_type == CTRL_STANDARD) {
			cam_try_add_ctrls(priv, i,
					&mcu_cam_ctrls[i]);
		} else {
			/* Not Implemented */
		}
	}

	return 0;

error:
	v4l2_ctrl_handler_free(&priv->ctrl_handler);
	return err;
}

static int cam_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct camera_common_data *s_data = to_camera_common_data(&client->dev);
	struct cam *priv = (struct cam *)s_data->priv;
	int err = 0;

	if (!priv || !priv->pdata)
		return -EINVAL;
	if (!enable) {
		/* Perform Stream Off Sequence - if any */
		if (!strm_cam_num) {
			pr_info("No Cameras are Streamed On");
			return -1;
		}
		strm_cam_num--;
		pr_info("CAM %d stream off called\n", strm_cam_num);

		/* Decrement the refs count when streaming is disabled */

		err = mcu_cam_stream_off(client);
		if (priv->ref_count == 1) {
			/* Decrement the refs count only when the ref count is 1 */
			module_put(s_data->owner);
			priv->ref_count = 0;
		}
		if (err != 0) {
			dev_err(&client->dev, "%s (%d) Stream_Off\n",
						__func__, __LINE__);
			return err;
		}

		pr_info("*********** STREAM NUM STATUS*******-off: %d", strm_cam_num);
		if ((!strm_cam_num) & (!monitor_call)) {
			monitor_call = 1;
			err = stop_stream_monitor_thread();
			if (err != 0) {
				pr_info("Error Stoping Stream Monitor Thread");
				return err;
			}
			monitor_call = 0;
		}

		/* Reset Frame rate index */
		priv->frame_index = 0;

		return err;

	}

	/* Perform Stream On Sequence - if any  */
	pr_info("CAM %d stream on called\n", strm_cam_num);

	pr_info("*********** STREAM NUM STATUS********- on: %d", strm_cam_num);

	err = mcu_cam_stream_on(client);
	if (err != 0) {
		dev_err(&client->dev, "%s (%d) Stream_On\n",
						__func__, __LINE__);
		return err;
	}

	/* Increment the refs count when streaming is enabled */
	if (!try_module_get(s_data->owner))
		return -ENODEV;

	priv->ref_count = 1;

	strm_cam_num++;

	if (is_stream_monitor_thrd == 0 && ecam_status_check == 1) {

		pr_info("*************Streaming Monitor Thread**********\n");

		err = create_stream_monitor_thread();
		if (!err)
			pr_info("Created Streaming Monitor Thread.\n");
		else
			pr_info("Could not create thread stopping\n");
	}

	mdelay(100);


	return 0;
}

static int cam_g_input_status(struct v4l2_subdev *sd, u32 *status)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct camera_common_data *s_data = to_camera_common_data(&client->dev);
	struct cam *priv = (struct cam *)s_data->priv;
	struct camera_common_power_rail *pw = &priv->power;

	if (!priv || !priv->pdata)
		return -EINVAL;

	*status = pw->state == SWITCH_ON;

	return ERRCODE_SUCCESS;
}

static int cam_g_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *param)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct camera_common_data *s_data = to_camera_common_data(&client->dev);
	struct cam *priv = (struct cam *)s_data->priv;

	if (!priv || !priv->pdata)
		return -ENOTTY;

	param->parm.capture.capability |= V4L2_CAP_TIMEPERFRAME;

	param->parm.capture.timeperframe.denominator =
		priv->mcu_cam_frmfmt[priv->frmfmt_mode].framerates[priv->frame_index];
	param->parm.capture.timeperframe.numerator = 1;

	return 0;
}

static int cam_s_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *param)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct camera_common_data *s_data = to_camera_common_data(&client->dev);
	struct cam *priv = (struct cam *)s_data->priv;
	int ret = 0, err = 0;

	if (!priv || !priv->pdata)
		return -EINVAL;

	for (ret = 0; ret < priv->mcu_cam_frmfmt[priv->frmfmt_mode].num_framerates;
			ret++) {
		if ((priv->mcu_cam_frmfmt[priv->frmfmt_mode].framerates[ret] ==
					param->parm.capture.timeperframe.denominator)) {
			priv->frame_index = ret;

			param->parm.capture.capability |= V4L2_CAP_TIMEPERFRAME;
			param->parm.capture.timeperframe.denominator =	priv->mcu_cam_frmfmt[priv->frmfmt_mode].framerates[priv->frame_index];
			param->parm.capture.timeperframe.numerator = 1;

			err = gen_mcu_stream_config(client, priv);

			if (err < 0) {
				dev_err(&client->dev, " %s (%d )\n", __func__, __LINE__);
				return err;
			}
			mdelay(10);

			return 0;

		}
	}

	/* if S_PARM is called with invalid parameters, set the right parameters and return success */
	param->parm.capture.capability |= V4L2_CAP_TIMEPERFRAME;
	param->parm.capture.timeperframe.denominator =	priv->mcu_cam_frmfmt[priv->frmfmt_mode].framerates[priv->frame_index];
	param->parm.capture.timeperframe.numerator = 1;

	return 0;
}

static int cam_get_fmt(struct v4l2_subdev *sd, struct v4l2_subdev_pad_config *cfg,
		struct v4l2_subdev_format *format)
{
	return camera_common_g_fmt(sd, &format->format);
}

static int cam_set_fmt(struct v4l2_subdev *sd, struct v4l2_subdev_pad_config *cfg,
		struct v4l2_subdev_format *format)
{
	int ret, err = 0;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct camera_common_data *s_data = to_camera_common_data(&client->dev);
	struct cam *priv = (struct cam *)s_data->priv;
	int flag = 0;

	if (!priv || !priv->pdata)
		return -EINVAL;
	switch (format->format.code) {
	case MEDIA_BUS_FMT_UYVY8_1X16:
		priv->format_fourcc = V4L2_PIX_FMT_UYVY;
		break;

	default:
		/* Not Implemented */
		if (format->which != V4L2_SUBDEV_FORMAT_TRY)
			return -EINVAL;
	}


	if (format->which == V4L2_SUBDEV_FORMAT_TRY) {
		ret = camera_common_try_fmt(sd, &format->format);
	} else {

		for (ret = 0; ret < s_data->numfmts; ret++) {
			if ((priv->mcu_cam_frmfmt[ret].size.width == format->format.width)
					&& (priv->mcu_cam_frmfmt[ret].size.height ==
						format->format.height)) {
				priv->frmfmt_mode = priv->mcu_cam_frmfmt[ret].mode;
				flag = 1;
				break;
			}
		}

		if (flag == 0)
			return -EINVAL;

		mdelay(10);

		err = gen_mcu_stream_config(client, priv);
		if (err < 0) {
			dev_err(&client->dev, " %s (%d )\n", __func__, __LINE__);
			return err;
		}

		ret = camera_common_s_fmt(sd, &format->format);
	}

	if (priv->phy == PHY_A) {
		if ((serdes_write_16b_reg(client, priv->dser_addr, 0x0002, 0x23)) < 0) {
			dev_err(&client->dev, "%s(%d): Failed\n", __func__, __LINE__);
			return -EIO;
		}
		msleep(20);

		if ((serdes_write_16b_reg(client, priv->dser_addr, 0x0002, 0x33)) < 0) {
			dev_err(&client->dev, "%s(%d): Failed\n", __func__, __LINE__);
			return -EIO;
		}
		msleep(100);
	} else if (priv->phy == PHY_B) {
		if ((serdes_write_16b_reg(client, priv->dser_addr, 0x0002, 0x13)) < 0) {
			dev_err(&client->dev, "%s(%d): Failed\n", __func__, __LINE__);
			return -EIO;
		}
		msleep(20);

		if ((serdes_write_16b_reg(client, priv->dser_addr, 0x0002, 0x33)) < 0) {
			dev_err(&client->dev, "%s(%d): Failed\n", __func__, __LINE__);
			return -EIO;
		}
		msleep(100);
	}

	return ret;

}


static int cam_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	return 0;
}

static const struct v4l2_subdev_internal_ops cam_subdev_internal_ops = {
	.open = cam_open,
};

static const struct media_entity_operations cam_media_ops = {
	.link_validate = v4l2_subdev_link_validate,
};


static struct v4l2_subdev_video_ops cam_subdev_video_ops = {
	.s_stream = cam_s_stream,
	.g_input_status = cam_g_input_status,
	.g_parm = cam_g_parm,
	.s_parm = cam_s_parm,
};

static struct v4l2_subdev_core_ops cam_subdev_core_ops = {
	.s_power = camera_common_s_power,
};

static struct v4l2_subdev_pad_ops cam_subdev_pad_ops = {
	.enum_mbus_code = camera_common_enum_mbus_code,
	.set_fmt = cam_set_fmt,
	.get_fmt = cam_get_fmt,
	.enum_frame_size = camera_common_enum_framesizes,
	.enum_frame_interval = camera_common_enum_frameintervals,
};

static struct v4l2_subdev_ops cam_subdev_ops = {
	.core = &cam_subdev_core_ops,
	.video = &cam_subdev_video_ops,
	.pad = &cam_subdev_pad_ops,
};

const static struct of_device_id cam_of_match[] = {
	{.compatible = "nvidia,ox08b40",},
	{}, /* Sentinal entry */
};
MODULE_DEVICE_TABLE(of, cam_of_match);



static int mcu_list_fmts(struct i2c_client *client,
		ISP_STREAM_INFO *stream_info, int *frm_fmt_size, struct cam *priv)
{
	uint32_t payload_len = 0, err = 0;
	uint8_t errcode = ERRCODE_SUCCESS, orig_crc = 0, calc_crc = 0, skip = 0;
	uint16_t index = 0, mode = 0;
	uint8_t mc_data[512], mc_ret_data[512];

	int loop = 0, num_frates = 0, ret = 0;

	/* lock semaphore */
	mutex_lock(&g_i2c_mutex);

	/* List all formats from MCU and append to mcu_cam_frmfmt array */
	/*NEED TO CHECK*/
	for (index = 0; index < 10; index++) {
		/* First Txn Payload length = 0 */
		payload_len = 2;

		mc_data[0] = CMD_SIGNATURE;
		mc_data[1] = CMD_ID_GET_STREAM_INFO;
		mc_data[2] = payload_len >> 8;
		mc_data[3] = payload_len & 0xFF;
		mc_data[4] = errorcheck(&mc_data[2], 2);

		cam_write(client, mc_data, TX_LEN_PKT);

		msleep(20);

		mc_data[0] = CMD_SIGNATURE;
		mc_data[1] = CMD_ID_GET_STREAM_INFO;
		mc_data[2] = index >> 8;
		mc_data[3] = index & 0xFF;
		mc_data[4] = errorcheck(&mc_data[2], 2);
		err = cam_write(client, mc_data, 5);

		msleep(20);

		if (err != 0) {
			dev_err(&client->dev, " %s(%d) Error - %d\n",
					__func__, __LINE__, err);
			ret = -EIO;
			goto exit;
		}

		err = cam_read(client, mc_ret_data, RX_LEN_PKT);

		msleep(20);

		if (err != 0) {
			dev_err(&client->dev, " %s(%d) Error - %d\n",
					__func__, __LINE__, err);
			ret = -EIO;
			goto exit;
		}

		/* Verify CRC */
		orig_crc = mc_ret_data[4];
		calc_crc = errorcheck(&mc_ret_data[2], 2);
		if (orig_crc != calc_crc) {
			pr_err
				(" %s(%d) CRC 0x%02x != 0x%02x\n",
				 __func__, __LINE__, orig_crc, calc_crc);
			ret = -EINVAL;
			goto exit;
		}

		if (((mc_ret_data[2] << 8) | mc_ret_data[3]) == 0) {
			if (stream_info == NULL)
				*frm_fmt_size = index;
			else
				*frm_fmt_size = mode;
			break;
		}

		payload_len =
			((mc_ret_data[2] << 8) | mc_ret_data[3]) +
			HEADER_FOOTER_SIZE;
		errcode = mc_ret_data[5];
		if (errcode != ERRCODE_SUCCESS) {
			pr_err
				(" %s(%d) Errcode - 0x%02x\n",
				 __func__, __LINE__, errcode);
			ret = -EIO;
			goto exit;
		}

		memset(mc_ret_data, 0x00, payload_len);
		err = cam_read(client, mc_ret_data, payload_len);

		msleep(20);

		if (err != 0) {
			dev_err(&client->dev, " %s(%d) Error - %d\n",
					__func__, __LINE__, err);
			ret = -1;
			goto exit;
		}

		/* Verify CRC */
		orig_crc = mc_ret_data[payload_len - 2];
		calc_crc =
			errorcheck(&mc_ret_data[2],
					payload_len - HEADER_FOOTER_SIZE);
		if (orig_crc != calc_crc) {
			pr_err
				(" %s(%d) CRC 0x%02x != 0x%02x\n",
				 __func__, __LINE__, orig_crc, calc_crc);
			ret = -EINVAL;
			goto exit;
		}

		/* Verify Errcode */
		errcode = mc_ret_data[payload_len - 1];
		if (errcode != ERRCODE_SUCCESS) {
			pr_err
				(" %s(%d) Errcode - 0x%02x\n",
				 __func__, __LINE__, errcode);
			ret = -EIO;
			goto exit;
		}

		if (stream_info != NULL) {
			/* check if any other format than UYVY is queried - do not append in array */
			stream_info->fmt_fourcc =
				mc_ret_data[2] << 24 | mc_ret_data[3] << 16 | mc_ret_data[4]
				<< 8 | mc_ret_data[5];
			stream_info->width = mc_ret_data[6] << 8 | mc_ret_data[7];
			stream_info->height = mc_ret_data[8] << 8 | mc_ret_data[9];
			stream_info->frame_rate_type = mc_ret_data[10];

			switch (stream_info->frame_rate_type) {
			case FRAME_RATE_DISCRETE:
				stream_info->frame_rate.disc.frame_rate_num =
					mc_ret_data[11] << 8 | mc_ret_data[12];

				stream_info->frame_rate.disc.frame_rate_denom =
					mc_ret_data[13] << 8 | mc_ret_data[14];

				break;

			case FRAME_RATE_CONTINOUS:
				pr_info(" The Stream format at index 0x%04x has FRAME_RATE_CONTINOUS,which is unsupported !!\n",
					 index);

			}

			switch (stream_info->fmt_fourcc) {
			case V4L2_PIX_FMT_UYVY:
				/* cam_codes is already populated with V4L2_MBUS_FMT_UYVY8_1X16 */
				/* check if width and height are already in array - update frame rate only */
				for (loop = 0; loop < (mode); loop++) {
					if ((priv->mcu_cam_frmfmt[loop].size.width ==
							stream_info->width)
						&& (priv->mcu_cam_frmfmt[loop].size.height ==
							stream_info->height)) {

						num_frates =
						priv->mcu_cam_frmfmt
						[loop].num_framerates;
						*((int *)(priv->mcu_cam_frmfmt[loop].framerates) + num_frates)
						= (int)(stream_info->frame_rate.disc.frame_rate_num /
							stream_info->frame_rate.disc.frame_rate_denom);

						priv->mcu_cam_frmfmt
							[loop].num_framerates++;

						priv->streamdb[index] = loop;
						skip = 1;
						break;
					}
				}

				if (skip) {
					skip = 0;
					continue;
				}

				/* Add Width, Height, Frame Rate array, Mode into mcu_cam_frmfmt array */
				priv->mcu_cam_frmfmt[mode].size.width = stream_info->width;
				priv->mcu_cam_frmfmt[mode].size.height =
					stream_info->height;
				num_frates = priv->mcu_cam_frmfmt[mode].num_framerates;

				*((int *)(priv->mcu_cam_frmfmt[mode].framerates) + num_frates) =
					(int)(stream_info->frame_rate.disc.frame_rate_num /
						stream_info->frame_rate.disc.frame_rate_denom);

				priv->mcu_cam_frmfmt[mode].num_framerates++;

				priv->mcu_cam_frmfmt[mode].mode = mode;
				priv->streamdb[index] = mode;
				mode++;
				break;

			default:
				pr_info(" The Stream format at index 0x%04x has format 0x%08x ,which is unsupported !!\n",
					 index,
					 stream_info->fmt_fourcc);
			}
		}
	}

exit:
	/* unlock semaphore */
	mutex_unlock(&g_i2c_mutex);

	return ret;
}

static int mcu_list_ctrls(struct i2c_client *client,
		ISP_CTRL_INFO *mcu_cam_ctrl, struct cam *priv)
{
	uint32_t payload_len = 0;
	uint8_t errcode = ERRCODE_SUCCESS, orig_crc = 0, calc_crc = 0;
	uint16_t index = 0;
	int ret = 0, err = 0, loop = 0, retry = 100, ctrl_id, value;
	uint8_t mc_data[512], mc_ret_data[512];
	uint32_t cam_num = 0;

	/* lock semaphore */
	mutex_lock(&g_i2c_mutex);

	/* Array of Ctrl Info */
	while (retry-- > 0) {
		/* First Txn Payload length = 0 */
		payload_len = 2;

		mc_data[0] = CMD_SIGNATURE;
		mc_data[1] = CMD_ID_GET_CTRL_INFO;
		mc_data[2] = payload_len >> 8;
		mc_data[3] = payload_len & 0xFF;
		mc_data[4] = errorcheck(&mc_data[2], 2);

		err = cam_write(client, mc_data, TX_LEN_PKT);

		msleep(20);

		if (err != 0) {
			dev_err(&client->dev, " %s(%d) Error - %d\n",
					__func__, __LINE__, err);
			continue;
		}
		mc_data[0] = CMD_SIGNATURE;
		mc_data[1] = CMD_ID_GET_CTRL_INFO;
		mc_data[2] = index >> 8;
		mc_data[3] = index & 0xFF;
		mc_data[4] = errorcheck(&mc_data[2], 2);
		err = cam_write(client, mc_data, 5);

		msleep(20);

		if (err != 0) {
			dev_err(&client->dev, " %s(%d) Error - %d\n",
					__func__, __LINE__, err);
			continue;
		}

		err = cam_read(client, mc_ret_data, RX_LEN_PKT);

		msleep(20);

		if (err != 0) {
			dev_err(&client->dev, " %s(%d) Error - %d\n",
					__func__, __LINE__, err);
			continue;
		}

		/* Verify CRC */
		orig_crc = mc_ret_data[4];
		calc_crc = errorcheck(&mc_ret_data[2], 2);
		if (orig_crc != calc_crc) {
			dev_err(&client->dev,
					" %s(%d) CRC 0x%02x != 0x%02x\n",
					__func__, __LINE__, orig_crc, calc_crc);
			continue;
		}

		if (((mc_ret_data[2] << 8) | mc_ret_data[3]) == 0) {
			priv->num_ctrls = index;
			break;
		}

		payload_len =
			((mc_ret_data[2] << 8) | mc_ret_data[3]) +
			HEADER_FOOTER_SIZE;
		errcode = mc_ret_data[5];
		if (errcode != ERRCODE_SUCCESS) {
			dev_err(&client->dev,
					" %s(%d) Errcode - 0x%02x\n",
					__func__, __LINE__, errcode);
			continue;
		}

		memset(mc_ret_data, 0x00, payload_len);
		err = cam_read(client, mc_ret_data, payload_len);

		msleep(20);

		if (err != 0) {
			dev_err(&client->dev, " %s(%d) Error - %d\n",
					__func__, __LINE__, err);
			continue;
		}

		/* Verify CRC */
		orig_crc = mc_ret_data[payload_len - 2];
		calc_crc =
			errorcheck(&mc_ret_data[2],
					payload_len - HEADER_FOOTER_SIZE);
		if (orig_crc != calc_crc) {
			dev_err(&client->dev,
					" %s(%d) CRC 0x%02x != 0x%02x\n",
					__func__, __LINE__, orig_crc, calc_crc);
			continue;
		}

		/* Verify Errcode */
		errcode = mc_ret_data[payload_len - 1];
		if (errcode != ERRCODE_SUCCESS) {
			dev_err(&client->dev,
					" %s(%d) Errcode - 0x%02x\n",
					__func__, __LINE__, errcode);
			continue;
		}

		if (mcu_cam_ctrl != NULL) {

			/* append ctrl info in array */
			mcu_cam_ctrl[index].ctrl_id =
				mc_ret_data[2] << 24 | mc_ret_data[3] << 16 | mc_ret_data[4]
				<< 8 | mc_ret_data[5];
			mcu_cam_ctrl[index].ctrl_type = mc_ret_data[6];

			switch (mcu_cam_ctrl[index].ctrl_type) {
			case CTRL_STANDARD:
				mcu_cam_ctrl[index].ctrl_data.std.ctrl_min =
					mc_ret_data[7] << 24 | mc_ret_data[8] << 16
					| mc_ret_data[9] << 8 | mc_ret_data[10];

				mcu_cam_ctrl[index].ctrl_data.std.ctrl_max =
					mc_ret_data[11] << 24 | mc_ret_data[12] <<
					16 | mc_ret_data[13]
					<< 8 | mc_ret_data[14];

				mcu_cam_ctrl[index].ctrl_data.std.ctrl_def =
					mc_ret_data[15] << 24 | mc_ret_data[16] <<
					16 | mc_ret_data[17]
					<< 8 | mc_ret_data[18];

				mcu_cam_ctrl[index].ctrl_data.std.ctrl_step =
					mc_ret_data[19] << 24 | mc_ret_data[20] <<
					16 | mc_ret_data[21]
					<< 8 | mc_ret_data[22];
				break;

			case CTRL_EXTENDED:
				/* Not Implemented */
				break;
			}

			priv->ctrldb[index] = mcu_cam_ctrl[index].ctrl_id;

	if (priv->cam_id == Ox08B40) {
		cam_num = priv->cam_num;
		ctrl_id = mcu_cam_ctrl[index].ctrl_id;
		value = mcu_cam_ctrl[index].ctrl_data.std.ctrl_def;

		for (loop = 0; loop < available_ctrls_count; loop++) {
			if (available_ctrls[loop].ctrl_id == ctrl_id) {
				available_ctrls[loop].curr_array[cam_num] = value;
#ifdef DEBUG
				pr_info("cam_num: %d, ctrl_id: 0x%x, value: %d\n", cam_num, ctrl_id, available_ctrls[loop].curr_array[cam_num]);
#endif
				break;
			}
		}
	}
		}
		index++;
		if (retry == 0) {
			ret = -EIO;
			goto exit;
		}
	}

exit:
	/* unlock semaphore */
	mutex_unlock(&g_i2c_mutex);

	return ret;

}

int mcu_get_cmd_status(struct i2c_client *client,
		uint8_t *cmd_id, uint16_t *cmd_status,
		uint8_t *ret_code)
{
	uint32_t payload_len = 0;
	uint8_t orig_crc = 0, calc_crc = 0;
	int err = 0;
	uint8_t mc_data[512], mc_ret_data[512];

	/* No Semaphore in Get command Status */

	/* First Txn Payload length = 0 */
	payload_len = 1;

	mc_data[0] = CMD_SIGNATURE;
	mc_data[1] = CMD_ID_GET_STATUS;
	mc_data[2] = payload_len >> 8;
	mc_data[3] = payload_len & 0xFF;
	mc_data[4] = errorcheck(&mc_data[2], 2);

	cam_write(client, mc_data, TX_LEN_PKT);

	msleep(20);

	mc_data[0] = CMD_SIGNATURE;
	mc_data[1] = CMD_ID_GET_STATUS;
	mc_data[2] = *cmd_id;
	err = cam_write(client, mc_data, 3);

	msleep(20);

	if (err != 0) {
		dev_err(&client->dev, " %s(%d) Error - %d\n", __func__,
				__LINE__, err);
		return -EIO;
	}

	payload_len = CMD_STATUS_MSG_LEN;
	memset(mc_ret_data, 0x00, payload_len);
	err = cam_read(client, mc_ret_data, payload_len);

	msleep(20);

	if (err != 0) {
		dev_err(&client->dev, " %s(%d) Error - %d\n", __func__,
				__LINE__, err);
		return -EIO;
	}

	/* Verify CRC */
	orig_crc = mc_ret_data[payload_len - 2];
	calc_crc = errorcheck(&mc_ret_data[2], 3);
	if (orig_crc != calc_crc) {
		dev_err(&client->dev, " %s(%d) CRC 0x%02x != 0x%02x\n",
				__func__, __LINE__, orig_crc, calc_crc);
		return -EINVAL;
	}

	*cmd_id = mc_ret_data[2];
	*cmd_status = mc_ret_data[3] << 8 | mc_ret_data[4];
	*ret_code = mc_ret_data[payload_len - 1];

	return ERRCODE_SUCCESS;
}

int mcu_get_sensor_id(struct i2c_client *client, uint16_t *sensor_id)
{
	uint32_t payload_len = 0;
	uint8_t errcode = ERRCODE_SUCCESS, orig_crc = 0, calc_crc = 0;

	int ret = 0, err = 0;
	uint8_t mc_data[512], mc_ret_data[512];

	/* lock semaphore */
	mutex_lock(&g_i2c_mutex);

	/* First Txn Payload length = 0 */
	payload_len = 0;

	mc_data[0] = CMD_SIGNATURE;
	mc_data[1] = CMD_ID_GET_SENSOR_ID;
	mc_data[2] = payload_len >> 8;
	mc_data[3] = payload_len & 0xFF;
	mc_data[4] = errorcheck(&mc_data[2], 2);

	cam_write(client, mc_data, TX_LEN_PKT);

	msleep(20);

	mc_data[0] = CMD_SIGNATURE;
	mc_data[1] = CMD_ID_GET_SENSOR_ID;
	err = cam_write(client, mc_data, 2);

	msleep(20);

	if (err != 0) {
		dev_err(&client->dev, " %s(%d) Error - %d\n", __func__,
				__LINE__, err);
		ret = -EIO;
		goto exit;
	}

	err = cam_read(client, mc_ret_data, RX_LEN_PKT);

	msleep(20);

	if (err != 0) {
		dev_err(&client->dev, " %s(%d) Error - %d\n", __func__,
				__LINE__, err);
		ret = -EIO;
		goto exit;
	}

	/* Verify CRC */
	orig_crc = mc_ret_data[4];
	calc_crc = errorcheck(&mc_ret_data[2], 2);
	if (orig_crc != calc_crc) {
		dev_err(&client->dev, " %s(%d) CRC 0x%02x != 0x%02x\n",
				__func__, __LINE__, orig_crc, calc_crc);
		ret = -EINVAL;
		goto exit;
	}

	errcode = mc_ret_data[5];
	if (errcode != ERRCODE_SUCCESS) {
		dev_err(&client->dev, " %s(%d) Errcode - 0x%02x\n",
				__func__, __LINE__, errcode);
		ret = -EIO;
		goto exit;
	}

	payload_len =
		((mc_ret_data[2] << 8) | mc_ret_data[3]) + HEADER_FOOTER_SIZE;

	memset(mc_ret_data, 0x00, payload_len);
	err = cam_read(client, mc_ret_data, payload_len);

	msleep(20);

	if (err != 0) {
		dev_err(&client->dev, " %s(%d) Error - %d\n", __func__,
				__LINE__, err);
		ret = -EIO;
		goto exit;
	}

	/* Verify CRC */
	orig_crc = mc_ret_data[payload_len - 2];
	calc_crc = errorcheck(&mc_ret_data[2], 2);
	if (orig_crc != calc_crc) {
		dev_err(&client->dev, " %s(%d) CRC 0x%02x != 0x%02x\n",
				__func__, __LINE__, orig_crc, calc_crc);
		ret = -EINVAL;
		goto exit;
	}

	/* Verify Errcode */
	errcode = mc_ret_data[payload_len - 1];
	if (errcode != ERRCODE_SUCCESS) {
		dev_err(&client->dev, " %s(%d) Errcode - 0x%02x\n",
				__func__, __LINE__, errcode);
		ret = -EIO;
		goto exit;
	}

	*sensor_id = mc_ret_data[2] << 8 | mc_ret_data[3];

exit:
	/* unlock semaphore */
	mutex_unlock(&g_i2c_mutex);

	return ret;
}


int mcu_get_imu_id(struct i2c_client *client, uint16_t *imu_id)
{
	uint32_t payload_len = 0;
	uint8_t errcode = ERRCODE_SUCCESS, orig_crc = 0, calc_crc = 0;

	int ret = 0, err = 0;
	uint8_t mc_data[512], mc_ret_data[512];

	/* lock semaphore */
	mutex_lock(&g_i2c_mutex);

	/* First Txn Payload length = 0 */
	payload_len = 0;

	mc_data[0] = CMD_SIGNATURE;
	mc_data[1] = CMD_ID_GET_IMU_ID;
	mc_data[2] = payload_len >> 8;
	mc_data[3] = payload_len & 0xFF;
	mc_data[4] = errorcheck(&mc_data[2], 2);

	err = cam_write(client, mc_data, TX_LEN_PKT);

	msleep(20);

	if (err != 0) {
		dev_err(&client->dev, " %s(%d) Error - %d\n", __func__,
				__LINE__, err);
		ret = -EIO;
		goto exit;
	}

	mc_data[0] = CMD_SIGNATURE;
	mc_data[1] = CMD_ID_GET_IMU_ID;
	err = cam_write(client, mc_data, 2);

	msleep(20);

	if (err != 0) {
		dev_err(&client->dev, " %s(%d) Error - %d\n", __func__,
				__LINE__, err);
		ret = -EIO;
		goto exit;
	}

	err = cam_read(client, mc_ret_data, RX_LEN_PKT);

	msleep(20);

	if (err != 0) {
		dev_err(&client->dev, " %s(%d) Error - %d\n", __func__,
				__LINE__, err);
		ret = -EIO;
		goto exit;
	}

	/* Verify CRC */
	orig_crc = mc_ret_data[4];
	calc_crc = errorcheck(&mc_ret_data[2], 2);
	if (orig_crc != calc_crc) {
		dev_err(&client->dev, " %s(%d) CRC 0x%02x != 0x%02x\n",
				__func__, __LINE__, orig_crc, calc_crc);
		ret = -EINVAL;
		goto exit;
	}

	errcode = mc_ret_data[5];
	if (errcode != ERRCODE_SUCCESS) {
		dev_err(&client->dev, " %s(%d) Errcode - 0x%02x\n",
				__func__, __LINE__, errcode);
		ret = -EIO;
		goto exit;
	}

	payload_len =
		((mc_ret_data[2] << 8) | mc_ret_data[3]) + HEADER_FOOTER_SIZE;

	memset(mc_ret_data, 0x00, payload_len);
	err = cam_read(client, mc_ret_data, payload_len);

	msleep(20);

	if (err != 0) {
		dev_err(&client->dev, " %s(%d) Error - %d\n", __func__,
				__LINE__, err);
		ret = -EIO;
		goto exit;
	}

	/* Verify CRC */
	orig_crc = mc_ret_data[payload_len - 2];
	calc_crc = errorcheck(&mc_ret_data[2], 2);
	if (orig_crc != calc_crc) {
		dev_err(&client->dev, " %s(%d) CRC 0x%02x != 0x%02x\n",
				__func__, __LINE__, orig_crc, calc_crc);
		ret = -EINVAL;
		goto exit;
	}

	/* Verify Errcode */
	errcode = mc_ret_data[payload_len - 1];
	if (errcode != ERRCODE_SUCCESS) {
		dev_err(&client->dev, " %s(%d) Errcode - 0x%02x\n",
				__func__, __LINE__, errcode);
		ret = -EIO;
		goto exit;
	}

	*imu_id = mc_ret_data[2] << 8 | mc_ret_data[3];

	if (*imu_id == 0) {
		ret = -EIO;
		goto exit;
	}

exit:
	/* unlock semaphore */
	mutex_unlock(&g_i2c_mutex);

	return ret;
}

int mcu_get_imu_angle_SET(struct i2c_client *client)
{
	uint8_t mc_data[512], err;
	uint32_t reg_val = 0;


	/*lock semaphore*/
	mutex_lock(&g_i2c_mutex);

	mc_data[0] = CMD_SIGNATURE;
	mc_data[1] = CMD_ID_IMU_READ_SET;
	mc_data[2] = 0x00;
	mc_data[3] = 0x02;
	mc_data[4] = errorcheck(&mc_data[2], 2);
	err = cam_write(client, mc_data, TX_LEN_PKT);

	msleep(20);

	if (err != ERRCODE_SUCCESS) {
		dev_err(&client->dev, " %s(%d) MCU Write Error - %d\n", __func__,
				__LINE__, err);
	}
	msleep(20);

	mc_data[0] = CMD_SIGNATURE;
	mc_data[1] = CMD_ID_IMU_READ_SET;
	mc_data[2] = 0x00;
	mc_data[3] = 0x02;
	mc_data[4] = errorcheck(&mc_data[2], 2);
	err = cam_write(client, mc_data, 5);

	msleep(20);

	if (err != ERRCODE_SUCCESS) {
		dev_err(&client->dev, " %s(%d) MCU Write Error - %d\n", __func__,
				__LINE__, err);
	}

	/* unlock semaphore */
	mutex_unlock(&g_i2c_mutex);

	return reg_val;
}

int mcu_get_imu_angle_GET(struct i2c_client *client)
{
	uint32_t payload_len = 0;
	uint16_t send_len = 0;
	uint8_t errcode = ERRCODE_SUCCESS, orig_crc = 0, calc_crc = 0;

	int ret = 0, err = 0, resultX = 0, resultY = 0, resultZ = 0;
	uint8_t mc_data[512], mc_ret_data[512];

	/* lock semaphore */
	mutex_lock(&g_i2c_mutex);

	/* First Txn Payload length = 0 */
	payload_len = 2;

	mc_data[0] = CMD_SIGNATURE;
	mc_data[1] = CMD_ID_IMU_READ_GET;
	mc_data[2] = payload_len >> 8;
	mc_data[3] = payload_len & 0xFF;
	mc_data[4] = errorcheck(&mc_data[2], 2);

	cam_write(client, mc_data, TX_LEN_PKT);

	msleep(20);

	if (err != ERRCODE_SUCCESS) {
		dev_err(&client->dev, " %s(%d) MCU Write Error - %d\n", __func__,
				__LINE__, err);
	}
	msleep(20);
	mc_data[0] = CMD_SIGNATURE;
	mc_data[1] = CMD_ID_IMU_READ_GET;
	mc_data[2] = 0x00;
	mc_data[3] = 0x00;
	mc_data[4] = errorcheck(&mc_data[2], 2);

	err = cam_write(client, mc_data, RX_LEN_PKT);

	msleep(20);

	if (err != ERRCODE_SUCCESS) {
		dev_err(&client->dev, " %s(%d) MCU Write Error - %d\n", __func__,
				__LINE__, err);
	}

	err = cam_read(client, mc_ret_data, RX_LEN_PKT);

	msleep(20);

	if (err != 0) {
		dev_err(&client->dev, " %s(%d) Error - %d\n", __func__,
				__LINE__, err);
		ret = -EIO;
		goto exit;
	}
	send_len = (mc_ret_data[2] << 8) | mc_ret_data[3];

	payload_len =
		((mc_ret_data[2] << 8) | mc_ret_data[3]) + HEADER_FOOTER_SIZE;

	memset(mc_ret_data, 0x00, payload_len);
	err = cam_read(client, mc_ret_data, payload_len);

	msleep(20);

	if (err != 0) {
		dev_err(&client->dev, " %s(%d) Error - %d\n", __func__,
				__LINE__, err);
		ret = -EIO;
		goto exit;
	}

	/* Verify CRC */
	orig_crc = mc_ret_data[payload_len - 2];
	calc_crc = errorcheck(&mc_ret_data[2], payload_len - HEADER_FOOTER_SIZE);
	if (orig_crc != calc_crc) {
		dev_err(&client->dev, " %s(%d) CRC 0x%02x != 0x%02x\n",
				__func__, __LINE__, orig_crc, calc_crc);
	}

	/* Verify Errcode */
	errcode = mc_ret_data[payload_len - 1];
	if (errcode != ERRCODE_SUCCESS) {
		dev_err(&client->dev, " %s(%d) Errcode - 0x%02x\n",
				__func__, __LINE__, errcode);
	}

	accX = mc_ret_data[2] | (mc_ret_data[3]<<8);
	accY = mc_ret_data[4] | (mc_ret_data[5]<<8);
	accZ = mc_ret_data[6] | (mc_ret_data[7]<<8);
	gyroX = mc_ret_data[8] | (mc_ret_data[9]<<8);
	gyroY = mc_ret_data[10] | (mc_ret_data[11]<<8);
	gyroZ = mc_ret_data[12] | (mc_ret_data[13]<<8);
	temp = mc_ret_data[14] | (mc_ret_data[15]<<8);

	/*Accelerometer RAW data Calculation*/
	resultX = (accX * 61 * 9810) / (1000 * 1000);
	resultY = (accY * 61 * 9810) / (1000 * 1000);
	resultZ = (accZ * 61 * 9810) / (1000 * 1000);

	accX_whole = resultX / 1000;
	accX_frac = resultX % 1000;

	accY_whole = resultY / 1000;
	accY_frac = resultY % 1000;

	accZ_whole = resultZ / 1000;
	accZ_frac = resultZ % 1000;

	pr_info("ACCX whole=%d , frac=%d", accX_whole, accX_frac);
	pr_info("ACCY whole=%d , frac=%d", accY_whole, accY_frac);
	pr_info("ACCZ whole=%d , frac=%d", accZ_whole, accZ_frac);


	/*Gyroscope RAW data calculation*/
	resultX = (gyroX / 131) * 1000;
	resultY = (gyroY / 131) * 1000;
	resultZ = (gyroZ / 131) * 1000;

	gyroX_whole = resultX / 1000;
	gyroX_frac = resultX % 1000;

	gyroY_whole = resultY / 1000;
	gyroY_frac = resultY % 1000;

	gyroZ_whole = resultZ / 1000;
	gyroZ_frac = resultZ % 1000;


exit:
	/* unlock semaphore */
	mutex_unlock(&g_i2c_mutex);

	return ret;
}



int mcu_get_imu_angle(struct i2c_client *client)
{
	int ret = 0;

	ret = mcu_get_imu_angle_SET(client);
	msleep(200);
	ret = mcu_get_imu_angle_GET(client);

	return ret;
}

int mcu_get_isp_sensor_temperature_SET(struct i2c_client *client)
{
	uint8_t mc_data[512], err;
	uint32_t reg_val = 0;

	/*lock semaphore*/
	mutex_lock(&g_i2c_mutex);

	mc_data[0] = CMD_SIGNATURE;
	mc_data[1] = CMD_ID_TEMPERATURE_SET;
	mc_data[2] = 0x00;
	mc_data[3] = 0x02;
	mc_data[4] = errorcheck(&mc_data[2], 2);
	err = cam_write(client, mc_data, TX_LEN_PKT);

	msleep(20);

	if (err != ERRCODE_SUCCESS) {
		dev_err(&client->dev, " %s(%d) MCU Write Error - %d\n", __func__,
				__LINE__, err);
	}
	msleep(20);

	mc_data[0] = CMD_SIGNATURE;
	mc_data[1] = CMD_ID_TEMPERATURE_SET;
	mc_data[2] = 0x00;
	mc_data[3] = 0x00;
	mc_data[4] = errorcheck(&mc_data[2], 2);
	err = cam_write(client, mc_data, 5);

	msleep(20);

	if (err != ERRCODE_SUCCESS) {
		dev_err(&client->dev, " %s(%d) MCU Write Error - %d\n", __func__,
				__LINE__, err);
	}

	/* unlock semaphore */
	mutex_unlock(&g_i2c_mutex);

	return reg_val;
}


int mcu_get_isp_sensor_temperature_GET(struct i2c_client *client)
{
	int8_t mc_data[512], mc_ret_data[512], err;
	uint32_t reg_val = 0;
	uint16_t send_len = 0;

	/*lock semaphore*/
	mutex_lock(&g_i2c_mutex);

	mc_data[0] = CMD_SIGNATURE;
	mc_data[1] = CMD_ID_TEMPERATURE_GET;
	mc_data[2] = 0x00;
	mc_data[3] = 0x02;
	mc_data[4] = errorcheck(&mc_data[2], 2);
	err = cam_write(client, mc_data, TX_LEN_PKT);

	msleep(20);

	if (err != ERRCODE_SUCCESS) {
		dev_err(&client->dev, " %s(%d) MCU Write Error - %d\n", __func__,
				__LINE__, err);
	}
	msleep(20);

	mc_data[0] = CMD_SIGNATURE;
	mc_data[1] = CMD_ID_TEMPERATURE_GET;
	mc_data[2] = 0x00;
	mc_data[3] = 0x00;
	mc_data[4] = errorcheck(&mc_data[2], 2);
	err = cam_write(client, mc_data, 5);

	msleep(20);

	if (err != ERRCODE_SUCCESS) {
		dev_err(&client->dev, " %s(%d) MCU Write Error - %d\n", __func__,
				__LINE__, err);
	}

	memset(mc_ret_data, 0, 512);
	err = cam_read(client, mc_ret_data, RX_LEN_PKT);

	msleep(20);

	if (err != 0) {
		dev_err(&client->dev, " %s(%d) Error - %d\n", __func__,
				__LINE__, err);
	}
	send_len = (mc_ret_data[2] << 8) | mc_ret_data[3];


	memset(mc_ret_data, 0, 512);
	/*NEED TO CHECK*/
	err = cam_read(client, mc_ret_data, send_len + HEADER_FOOTER_SIZE);

	msleep(20);

	if (err != 0) {
		dev_err(&client->dev, " %s(%d) Error - %d\n", __func__,
				__LINE__, err);
	}

	isp_temperature_integer = mc_ret_data[2];
	isp_temperature_decimal = mc_ret_data[3];
	sensor_temperature_integer = mc_ret_data[4];
	sensor_temperature_decimal = mc_ret_data[5];

	if (isp_temperature_integer >= 0xc0) {
		isp_temperature_integer = -1*(isp_temperature_integer - 0xC0);
		isp_temperature_decimal = (isp_temperature_decimal * 0x64) / 0x100;
	}

	if (sensor_temperature_integer > 0xc0)
		sensor_temperature_integer = -1*(sensor_temperature_integer - 0xC0);

	pr_info("\n\r ISP temperature is %d.%d°C", isp_temperature_integer, isp_temperature_decimal);
	pr_info("\n\r Sensor temperature is %d.%d°C", sensor_temperature_integer, sensor_temperature_decimal);


	/* unlock semaphore */
	mutex_unlock(&g_i2c_mutex);

	return reg_val;
}
int mcu_get_isp_sensor_temperature(struct i2c_client *client)
{
	int ret = 0;

	ret = mcu_get_isp_sensor_temperature_SET(client);
	msleep(500);
	ret = mcu_get_isp_sensor_temperature_GET(client);

	return ret;
}


int mcu_isp_init(struct i2c_client *client)
{
	uint32_t payload_len = 0;

	uint16_t cmd_status = 0;
	uint8_t retcode = 0, cmd_id = 0;
	int retry = 10, err = 0;
	uint8_t mc_data[512];

	pr_info("%s\n", __func__);

	/* call ISP init command */

	/* First Txn Payload length = 0 */
	payload_len = 0;

	mc_data[0] = CMD_SIGNATURE;
	mc_data[1] = CMD_ID_INIT_CAM;
	mc_data[2] = payload_len >> 8;
	mc_data[3] = payload_len & 0xFF;
	mc_data[4] = errorcheck(&mc_data[2], 2);

	cam_write(client, mc_data, TX_LEN_PKT);

	mc_data[0] = CMD_SIGNATURE;
	mc_data[1] = CMD_ID_INIT_CAM;
	err = cam_write(client, mc_data, 2);
	if (err != 0) {
		dev_err(&client->dev, " %s(%d) Error - %d\n", __func__,
				__LINE__, err);
		goto exit_isp_init;
	}

	while (--retry > 0) {
		/* Some Sleep for init to process */
		msleep(400);

		cmd_id = CMD_ID_INIT_CAM;
		if (mcu_get_cmd_status
				(client, &cmd_id, &cmd_status, &retcode) < 0) {
			dev_err(&client->dev, " %s(%d) Error\n",
					__func__, __LINE__);
			goto exit_isp_init;
		}

		if ((cmd_status == MCU_CMD_STATUS_SUCCESS) &&
			((retcode == ERRCODE_SUCCESS)
				|| (retcode == ERRCODE_ALREADY))) {
			dev_info(&client->dev, "ISP Initialized !!\n");
			mutex_unlock(&g_i2c_mutex);
			return 0;
		}

		if ((retcode != ERRCODE_BUSY) &&
				((cmd_status != MCU_CMD_STATUS_PENDING))) {
			dev_err(&client->dev,
				"(%s) %d Init Error STATUS = 0x%04x RET = 0x%02x\n",
					__func__, __LINE__, cmd_status, retcode);
			goto exit_isp_init;
		}
	}

	dev_err(&client->dev, "ETIMEDOUT Error\n");
	return -ETIMEDOUT;

exit_isp_init:
	mutex_unlock(&g_i2c_mutex);
	return -EIO;
}

int ecam_mcu_core_initialize(struct i2c_client *client, struct cam *priv)
{
	int errcode = ERRCODE_SUCCESS;
	uint8_t retry;
	int16_t sensor_id = 0;
	int16_t imu_id = 0;

	cam_trace(&client->dev, "ecam_mcu_core_initialize: ENTER\n");

	/* Gets Sensor ID */
	retry = 0;
	while (retry++ < 5) {
		if (mcu_get_sensor_id(client, &sensor_id) < 0)
			dev_err(&client->dev, "Unable to get Sensor ID!.. Retrying..\n");
		else
			break;
	}

	if (retry > 5) {
		dev_err(&client->dev,
				"Unable to get Sensor ID!.. Exiting..\n");
	}
	dev_info(&client->dev, "Sensor ID = 0x%x\n", sensor_id);

	/* Issue retry for init ISP */
	retry = 10;
	while (retry-- > 0) {
		if (mcu_isp_init(client) < 0) {
			msleep(100);
			dev_err(&client->dev,
				"Unable to INIT ISP, retry = %d\n", retry);
			continue;
		} else {
			break;
		}
	}

	if (retry == 0) {
		dev_err(&client->dev, "Unable to INIT ISP\n");
		return -EFAULT;
	}

	msleep(20);

	/*GET IMU ID*/
	retry = 0;
	if (priv->cam_id == Ox08B40) {
		while (retry++ < 5) {
			if (mcu_get_imu_id(client, &imu_id) < 0)
				dev_err(&client->dev, "Unable to get Sensor ID!.. Retrying..\n");
			else
				break;
		}
		if (retry > 5) {
			dev_err(&client->dev,
				"Unable to get IMU ID!.. Exiting..\n");
		}
		dev_info(&client->dev, "IMU ID = 0x%x\n", imu_id);
	}

	cam_trace(&client->dev,
		"ecam_mcu_core_initialize: EXIT rc=%d (sensor_id=0x%x)\n",
		errcode, sensor_id);
	return errcode;
}

int mcu_configurations(struct i2c_client *client, struct cam *priv)
{
	int errcode = ERRCODE_SUCCESS;
	int retry;

	cam_trace(&client->dev, "mcu_configurations: ENTER\n");

	/* Query the number of Controls from MCU */
	retry = 5;
	while (--retry >= 0) {
		if (mcu_list_ctrls(client, NULL, priv) < 0) {
			dev_err(&client->dev, "%s, init controls failure.retrying\n", __func__);
			continue;
		} else {
			break;
		}
	}
	if (retry < 0) {
		dev_err(&client->dev, "%s, Failed to init controls\n",
							__func__);
		errcode = -EFAULT;
		goto mcu_config_error;
	}

	/*Query the number for Formats available from MCU */
	retry = 5;
	while (--retry >= 0) {
		if (mcu_list_fmts(client, NULL, &frm_fmt_size, priv) < 0) {
			dev_err(&client->dev,
				"%s, init formats failure. retrying(%d)\n",
								__func__, retry);
			continue;
		} else {
			break;
		}
	}

	if (retry < 0) {
		dev_err(&client->dev, "%s, Failed to init formats!\n",
							__func__);
		errcode = -EFAULT;
		goto mcu_config_error;
	}


	priv->mcu_ctrl_info = devm_kzalloc(&client->dev,
		sizeof(ISP_CTRL_INFO) * priv->num_ctrls, GFP_KERNEL);
	if (!priv->mcu_ctrl_info) {
		errcode = -ENOMEM;
		goto mcu_config_error;
	}

	priv->ctrldb = devm_kzalloc(&client->dev,
		     sizeof(uint32_t) * priv->num_ctrls, GFP_KERNEL);
	if (!priv->ctrldb) {
		errcode = -ENOMEM;
		goto mcu_config_error;
	}

	priv->stream_info = devm_kzalloc(&client->dev,
		sizeof(ISP_STREAM_INFO) * (frm_fmt_size + 1), GFP_KERNEL);

	priv->streamdb = devm_kzalloc(&client->dev,
		      sizeof(int) * (frm_fmt_size + 1), GFP_KERNEL);
	if (!priv->streamdb) {
		errcode = -ENOMEM;
		goto mcu_config_error;
	}

	priv->mcu_cam_frmfmt = devm_kzalloc(&client->dev,
		sizeof(struct camera_common_frmfmt) * (frm_fmt_size),
								GFP_KERNEL);
	if (!priv->mcu_cam_frmfmt) {
		dev_err(&client->dev, "Unable to allocate memory\n");
		errcode = -ENOMEM;
		goto mcu_config_error;
	}

mcu_config_error:
	cam_trace(&client->dev,
		"mcu_configurations: EXIT rc=%d (num_ctrls=%d, frm_fmt_size=%d)\n",
		errcode, priv->num_ctrls, frm_fmt_size);
	return errcode;
}

static int cam_read(struct i2c_client *client, u8 *val, u32 count)
{
	int ret;
	struct i2c_msg msg = {
		.addr = client->addr,
		.flags = 0,
		.buf = val,
	};

	msg.flags = I2C_M_RD;
	msg.len = count;
	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret < 0)
		goto err;

	return 0;

err:
	dev_err(&client->dev, "Failed reading register ret = %d!\n", ret);
	return ret;
}

unsigned char errorcheck(char *data, unsigned int len)
{
	unsigned int i = 0;
	unsigned char crc = 0x00;

	for (i = 0; i < len; i++)
		crc ^= data[i];

	return crc;
}

static int cam_write(struct i2c_client *client, u8 *val, u32 count)
{
	int ret;
	struct i2c_msg msg = {
		.addr = client->addr,
		.flags = 0,
		.len = count,
		.buf = val,
	};

	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret < 0) {
		dev_err(&client->dev,
			"Failed writing register ret = %d!\n", ret);
		return ret;
	}

	return 0;
}

int mcu_mipi_configuration(struct i2c_client *client, struct cam *priv, u8 cmd_id)
{
	int ret = 0, err, retry = 1000;
	uint16_t payload_data;
	uint32_t payload_len = 0;
	uint16_t cmd_status = 0;
	uint8_t retcode = 0;
	uint8_t mc_data[512];

	/* lock semaphore */
	mutex_lock(&g_i2c_mutex);

	payload_len = 2;

	mc_data[0] = CMD_SIGNATURE;
	mc_data[1] = cmd_id;
	mc_data[2] = payload_len >> 8;
	mc_data[3] = payload_len & 0xFF;
	mc_data[4] = errorcheck(&mc_data[2], 2);

	cam_write(client, mc_data, TX_LEN_PKT);

	/* Second Txn */
	mc_data[0] = CMD_SIGNATURE;
	mc_data[1] = cmd_id;

		switch (cmd_id) {
		case CMD_ID_LANE_CONFIG:
			/*Lane configuration */
			payload_data = priv->mipi_lane_config == 4 ? NUM_LANES_4 : NUM_LANES_2;
			mc_data[2] = payload_data >> 8;
			mc_data[3] = payload_data & 0xFF;
			break;
		case CMD_ID_MIPI_CLK_CONFIG:
			/* MIPI CLK Configuration */
			payload_data = priv->mipi_clk_config;
			mc_data[2] = payload_data >> 8;
			mc_data[3] = payload_data & 0xFF;
			break;
		default:
			dev_err(&client->dev, "MCU MIPI CONF Error\n");
			err = -1;
			goto exit;
		}

	/* CRC */
	mc_data[4] = errorcheck(&mc_data[2], payload_len);
	err = cam_write(client, mc_data, payload_len+3);

	if (err != 0) {
		dev_err(&client->dev, " %s(%d) MCU Set Ctrl Error - %d\n", __func__,
		       __LINE__, err);
		ret = -1;
		goto exit;
	}

	while (--retry > 0) {
		msleep(20);
		if (mcu_get_cmd_status(client, &cmd_id, &cmd_status, &retcode) <
		    0) {
			dev_err(&client->dev, " %s(%d) MCU Get CMD Status Error\n", __func__,
				__LINE__);
			ret = -1;
			goto exit;
		}

		if ((cmd_status == MCU_CMD_STATUS_ISP_UNINIT) &&
		    (retcode == ERRCODE_SUCCESS)) {
			ret = 0;
			goto exit;
		}

		if ((retcode != ERRCODE_BUSY) &&
		    ((cmd_status != MCU_CMD_STATUS_ISP_UNINIT))) {
			dev_err(&client->dev,
				"(%s) %d MCU Get CMD Error STATUS = 0x%04x RET = 0x%02x\n",
					__func__, __LINE__, cmd_status, retcode);
			ret = -1;
			goto exit;
		}
	}
	err = -ETIMEDOUT;

 exit:
	/* unlock semaphore */
	mutex_unlock(&g_i2c_mutex);

	return ret;
}

int toggle_mcu_reset_pin(struct i2c_client *client, struct cam *priv)
{

	cam_trace(&client->dev, "toggle_mcu_reset_pin: ENTER\n");

	if (serdes_write_16b_reg(client, priv->ser_reassigned_addr,
		priv->reset_reg, MCU_RST_LOW) < 0) {
		dev_err(&client->dev, "%s(%d): Failed\n",
				__func__, __LINE__);
		return -EIO;
	}
#ifdef DEBUG
	pr_info("RESET pin-(%x) : %x ", priv->reset_reg, MCU_RST_LOW);
#endif
	msleep(20);
	if (serdes_write_16b_reg(client, priv->ser_reassigned_addr,
				priv->reset_reg, MCU_RST_HIGH) < 0) {
		dev_err(&client->dev, "%s(%d): Failed\n",
				__func__, __LINE__);
		return -EIO;
	}
#ifdef DEBUG
	pr_info("RESET pin-(%x) : %x ", priv->reset_reg, MCU_RST_HIGH);
#endif
	cam_trace(&client->dev, "toggle_mcu_reset_pin: EXIT OK\n");
	return 0;
}

int toggle_mcu_boot_pin(struct i2c_client *client, struct cam *priv)
{
	if (serdes_write_16b_reg(client, priv->ser_reassigned_addr,
		priv->boot_reg, MCU_BOOT_LOW) < 0) {
		dev_err(&client->dev, "%s(%d): Failed\n",
				__func__, __LINE__);
		return -EIO;
	}
#ifdef DEBUG
	pr_info("Boot pin-(%x) : %x ", priv->boot_reg, MCU_BOOT_LOW);
#endif
	msleep(20);

	if (serdes_write_16b_reg(client, priv->ser_reassigned_addr,
				priv->boot_reg, MCU_BOOT_HIGH) < 0) {
		dev_err(&client->dev, "%s(%d): Failed\n",
				__func__, __LINE__);
		return -EIO;
	}
#ifdef DEBUG
	pr_info("Boot pin-(%x) : %x ", priv->boot_reg, MCU_BOOT_HIGH);
#endif
	msleep(20);

	if (toggle_mcu_reset_pin(client, priv) <  0)
		return -EIO;

	return 0;
}

int mcu_get_fw_version(struct i2c_client *client,
		unsigned char *fw_version, unsigned char *txt_fw_version)
{
	uint32_t payload_len = 0;
	uint8_t errcode = ERRCODE_SUCCESS, orig_crc = 0, calc_crc = 0;
	int ret = 0, err = 0, loop, i = 0, retry = 5;
	unsigned long txt_fw_pos = strlen(mcu_fw_buf)-VERSION_FILE_OFFSET;
	uint8_t mc_data[512], mc_ret_data[512];

	/* lock semaphore */
	mutex_lock(&g_i2c_mutex);

	/* Get Text Firmware version*/
	for (loop = txt_fw_pos; loop < (txt_fw_pos+64); loop = loop+2) {
		*(txt_fw_version+i) = (mcu_bload_ascii2hex(mcu_fw_buf[loop]) << 4 |
				mcu_bload_ascii2hex(mcu_fw_buf[loop+1]));
		i++;
	}

	while (retry-- > 0) {
		/* Query firmware version from MCU */
		payload_len = 0;

		mc_data[0] = CMD_SIGNATURE;
		mc_data[1] = CMD_ID_VERSION;
		mc_data[2] = payload_len >> 8;
		mc_data[3] = payload_len & 0xFF;
		mc_data[4] = errorcheck(&mc_data[2], 2);
		err = cam_write(client, mc_data, TX_LEN_PKT);
		if (err != 0) {
			dev_err(&client->dev,
					" %s(%d) MCU CMD ID Write PKT fw Version Error - %d\n",
				__func__, __LINE__, ret);
			ret = -EIO;
			continue;
		}


		mc_data[0] = CMD_SIGNATURE;
		mc_data[1] = CMD_ID_VERSION;

		err = cam_write(client, mc_data, 2);
		if (err != 0) {
			dev_err(&client->dev,
					" %s(%d) MCU CMD ID Write PKT fw Version Error - %d\n",
				__func__, __LINE__, ret);
			ret = -EIO;
			continue;
		}

		err = cam_read(client, mc_ret_data, RX_LEN_PKT);

		if (err != 0) {
			dev_err(&client->dev, " %s(%d) MCU CMD ID Read PKT fw Version Error - %d\n",
				__func__, __LINE__, ret);
			ret = -EIO;
			continue;
		}

		/* Verify CRC */
		orig_crc = mc_ret_data[4];
		calc_crc = errorcheck(&mc_ret_data[2], 2);
		if (orig_crc != calc_crc) {
			dev_err(&client->dev,
					" %s(%d) MCU CMD ID fw Version Error CRC 0x%02x != 0x%02x\n",
					__func__, __LINE__, orig_crc, calc_crc);
			ret = -EINVAL;
			continue;
		}
		errcode = mc_ret_data[5];
		if (errcode != ERRCODE_SUCCESS) {
			dev_err(&client->dev, " %s(%d) MCU CMD ID fw Errcode - 0x%02x\n", __func__,
					__LINE__, errcode);
			ret = -EIO;
			continue;
		}

		/* Read the actual version from MCU*/
		payload_len =
			((mc_ret_data[2] << 8) | mc_ret_data[3]) + HEADER_FOOTER_SIZE;
		memset(mc_ret_data, 0x00, payload_len);
		err = cam_read(client, mc_ret_data, payload_len);
		if (err != 0) {
			dev_err(&client->dev, " %s(%d) MCU fw CMD ID Read Version Error - %d\n", __func__,
					__LINE__, ret);
			ret = -EIO;
			continue;
		}

		/* Verify CRC */
		orig_crc = mc_ret_data[payload_len - 2];
		calc_crc = errorcheck(&mc_ret_data[2], 32);
		if (orig_crc != calc_crc) {
			dev_err(&client->dev, " %s(%d) MCU fw  CMD ID Version CRC ERROR 0x%02x != 0x%02x\n",
					__func__, __LINE__, orig_crc, calc_crc);
			ret = -EINVAL;
			continue;
		}


		/* Verify Errcode */
		errcode = mc_ret_data[payload_len - 1];
		if (errcode != ERRCODE_SUCCESS) {
			dev_err(&client->dev, " %s(%d) MCU fw CMD ID Read Payload Error - 0x%02x\n", __func__,
					__LINE__, errcode);
			ret = -EIO;
			continue;
		}
		if (ret == ERRCODE_SUCCESS)
			break;
	}


	if (retry < 0 && ret != ERRCODE_SUCCESS) {
		pr_info(" %s with exit code = %d %d\n", __func__, ret, __LINE__);
		goto exit;
	}
	for (loop = 0 ; loop < VERSION_SIZE ; loop++)
		*(fw_version+loop) = mc_ret_data[2+loop];

	/* Check for forced/always update field in the text firmware version*/
	if (txt_fw_version[17] == '1') {
		dev_err(&client->dev, "Forced Update Enabled - Firmware Version - (%.32s)\n",
				fw_version);
		ret = MCU_FW_FORCE_UPDATE;
		goto exit;
	}
	for (i = 0; i < VERSION_SIZE; i++) {


		if (txt_fw_version[i] != fw_version[i]) {
			pr_info("Previous Firmware Version - (%.32s)\n", fw_version);
			pr_info("Current Firmware Version - (%.32s)\n", txt_fw_version);
			ret = MCU_FW_MISMATCH_UPDATE;
			goto exit;
		}
	}
	ret = ERRCODE_SUCCESS;

exit:
	/* unlock semaphore */
	mutex_unlock(&g_i2c_mutex);

	return ret;
}

int mcu_bload_erase_flash(struct i2c_client *client)
{
	unsigned short int pagenum = 0x0000;
	int ret = 0, i = 0, checksum = 0;
	uint16_t size = 0;


	/* --------------- ERASE FLASH --------------------- */

	for (i = 0; i <= NUM_ERASE_CYCLES; i++) {

		checksum = 0x00;
		/*   Write Erase Pages CMD */
		g_bload_buf[0] = BL_ERASE_MEM_NS;
		g_bload_buf[1] = ~(BL_ERASE_MEM_NS);

		ret = cam_write(client, g_bload_buf, 2);
		if (ret < 0) {
			dev_err(&client->dev, "Write Failed\n");
			return -1;
		}

		/*   Wait for ACK or NACK */
		ret = cam_read(client, g_bload_buf, 1);
		if (ret < 0) {
			dev_err(&client->dev, "Read Failed\n");
			return -1;
		}

		if (g_bload_buf[0] != RESP_ACK) {
			/*   NACK Received */
			dev_err(&client->dev, " NACK Received... exiting..\n");
			return -1;
		}

		if (i != NUM_ERASE_CYCLES) {
			g_bload_buf[0] = (MAX_PAGES - 1) >> 8;
			g_bload_buf[1] = (MAX_PAGES - 1) & 0xFF;
		} else { /* Last page erase handled separately */
			g_bload_buf[0] = 0x0 >> 8;
			g_bload_buf[1] = 0x0 & 0xFF;
		}

		g_bload_buf[2] = g_bload_buf[0] ^ g_bload_buf[1];

		ret = cam_write(client, g_bload_buf, 3);
		if (ret < 0) {
			dev_err(&client->dev, "Write Failed\n");
			return -1;
		}

		/*   Wait for ACK or NACK */
		ret = cam_read(client, g_bload_buf, 1);
		if (ret < 0) {
			dev_err(&client->dev, "Read Failed\n");
			return -1;
		}

		if (g_bload_buf[0] != RESP_ACK) {
			/*   NACK Received */
			dev_err(&client->dev, " NACK Received... exiting..\n");
			return -1;
		}

		if (i != NUM_ERASE_CYCLES) {
			for (pagenum = 0; pagenum < MAX_PAGES; pagenum++) {
				g_bload_buf[(2 * pagenum)] =
					(pagenum + (i * MAX_PAGES)) >> 8;
				g_bload_buf[(2 * pagenum) + 1] =
					(pagenum + (i * MAX_PAGES)) & 0xFF;
				checksum =
					checksum ^ g_bload_buf[(2 * pagenum)] ^
					g_bload_buf[(2 * pagenum) + 1];

			}
			g_bload_buf[2 * MAX_PAGES] = checksum;
		} else {
			g_bload_buf[0] = FW_VERSION_NUM_PAGE >> 8;
			g_bload_buf[1] = FW_VERSION_NUM_PAGE & 0xFF;
			g_bload_buf[2] = g_bload_buf[0] ^ g_bload_buf[1];
		}


		if (i != NUM_ERASE_CYCLES)
			size = (2 * MAX_PAGES) + 1;
		else
			size = NUM_ERASE_CYCLES + 1;

#ifdef DEBUG
		pr_info("size: %d (i=%d)", size, i);
#endif
		ret = cam_write(client, g_bload_buf, size);
		if (ret < 0) {
			dev_err(&client->dev, "Write Failed\n");
			return -1;
		}

poll_busy:
		/*   Wait for ACK or NACK */
		ret = cam_read(client, g_bload_buf, 1);
		if (ret < 0) {
			dev_err(&client->dev, "Read Failed\n");
			return -1;
		}

		if (g_bload_buf[0] == RESP_BUSY)
			goto poll_busy;

		if (g_bload_buf[0] != RESP_ACK) {
			/*   NACK Received */
			dev_err(&client->dev, " NACK Received... exiting.......\n");
			return -1;
		}

		pr_info(" ERASE Sector %d success !!\n", i + 1);
	}

	/* ------------ ERASE FLASH END ----------------------- */

	return 0;
}

int mcu_bload_parse_send_cmd(struct i2c_client *client,
		unsigned char *bytearray, int rec_len)
{
	IHEX_RECORD *ihex_rec = NULL;
	unsigned char checksum = 0, calc_checksum = 0;
	int i = 0, ret = 0;

	if (!bytearray)
		return -1;

	ihex_rec = (IHEX_RECORD *) bytearray;
	ihex_rec->addr = htons(ihex_rec->addr);

	checksum = bytearray[rec_len - 1];

	calc_checksum = mcu_bload_inv_checksum(bytearray, rec_len - 1);
	if (checksum != calc_checksum) {
		dev_err(&client->dev, " Invalid Checksum 0x%02x != 0x%02x !!\n",
				checksum, calc_checksum);
		return -1;
	}

	if ((ihex_rec->rectype == REC_TYPE_ELA)
			&& (ihex_rec->addr == 0x0000)
			&& (ihex_rec->datasize == 0x02)) {
		/*   Upper 32-bit configuration */
		g_bload_flashaddr = (g_bload_flashaddr & 0x00000000) |
			(ihex_rec->recdata[0] << 24) |
			(ihex_rec->recdata[1] << 16);

		pr_info("Updated Flash Addr = 0x%08x\n",
				g_bload_flashaddr);

	} else if (ihex_rec->rectype == REC_TYPE_DATA) {
		/*   Flash Data into Flashaddr */

		g_bload_flashaddr =
			(g_bload_flashaddr & 0xFFFF0000) | (ihex_rec->addr);
		g_bload_crc16 ^=
			mcu_bload_calc_crc16(ihex_rec->recdata, ihex_rec->datasize);

		/*   Write Erase Pages CMD */
		g_bload_buf[0] = BL_WRITE_MEM_NS;
		g_bload_buf[1] = ~(BL_WRITE_MEM_NS);

		ret = cam_write(client, g_bload_buf, 2);
		if (ret < 0) {
			dev_err(&client->dev, "Write Failed\n");
			return -1;
		}

		/*   Wait for ACK or NACK */
		ret = cam_read(client, g_bload_buf, 1);
		if (ret < 0) {
			dev_err(&client->dev, "Read Failed\n");
			return -1;
		}

		if (g_bload_buf[0] != RESP_ACK) {
			/*   NACK Received */
			dev_err(&client->dev, " NACK Received... exiting..\n");
			return -1;
		}

		g_bload_buf[0] = (g_bload_flashaddr & 0xFF000000) >> 24;
		g_bload_buf[1] = (g_bload_flashaddr & 0x00FF0000) >> 16;
		g_bload_buf[2] = (g_bload_flashaddr & 0x0000FF00) >> 8;
		g_bload_buf[3] = (g_bload_flashaddr & 0x000000FF);
		g_bload_buf[4] =
			g_bload_buf[0] ^ g_bload_buf[1] ^ g_bload_buf[2] ^
			g_bload_buf[3];

		ret = cam_write(client, g_bload_buf, 5);
		if (ret < 0) {
			dev_err(&client->dev, "Write Failed\n");
			return -1;
		}

		/*   Wait for ACK or NACK */
		ret = cam_read(client, g_bload_buf, 1);
		if (ret < 0) {
			dev_err(&client->dev, "Read Failed\n");
			return -1;
		}

		if (g_bload_buf[0] != RESP_ACK) {
			/*   NACK Received */
			dev_err(&client->dev, " NACK Received... exiting..\n");
			return -1;
		}

		g_bload_buf[0] = ihex_rec->datasize - 1;
		checksum = g_bload_buf[0];
		for (i = 0; i < ihex_rec->datasize; i++) {
			g_bload_buf[i + 1] = ihex_rec->recdata[i];
			checksum ^= g_bload_buf[i + 1];
		}

		g_bload_buf[i + 1] = checksum;

		ret = cam_write(client, g_bload_buf, i + 2);
		if (ret < 0) {
			dev_err(&client->dev, "Write Failed\n");
			return -1;
		}

poll_busy:
		/*   Wait for ACK or NACK */
		ret = cam_read(client, g_bload_buf, 1);
		if (ret < 0) {
			dev_err(&client->dev, "Read Failed\n");
			return -1;
		}

		if (g_bload_buf[0] == RESP_BUSY)
			goto poll_busy;

		if (g_bload_buf[0] != RESP_ACK) {
			/*   NACK Received */
			dev_err(&client->dev, " NACK Received... exiting..\n");
			return -1;
		}

	} else if (ihex_rec->rectype == REC_TYPE_SLA) {
		/*   Update Instruction pointer to this address */

	} else if (ihex_rec->rectype == REC_TYPE_EOF) {
		/*   End of File - Issue I2C Go Command */
		return 0;
	} else {
		/*   Unhandled Type */
		dev_err(&client->dev, "Unhandled Command Type\n");
		return -1;
	}

	return 0;
}

static int mcu_fw_update(struct i2c_client *client, unsigned char *mcu_fw_version)
{
	int ret = 0;

	g_bload_crc16 = 0;

	/* Read Firmware version from bootloader MCU */
	ret = mcu_bload_get_version(client);
	if (ret < 0) {
		dev_err(&client->dev, " Error in Get Version\n");
		goto exit;
	}

	pr_info(" Get Version SUCCESS !!\n");
#ifdef PRODE_STATUS
	probe_status_array[total_num_cam].mcu_fw_status_percentage = 20;
#endif
	/* Erase firmware present in the MCU and flash new firmware*/
	ret = mcu_bload_erase_flash(client);
	if (ret < 0) {
		dev_err(&client->dev, " Error in Erase Flash\n");
		goto exit;
	}

	pr_info("Erase Flash Success !!\n");
#ifdef PROBE_STATUS
	probe_status_array[total_num_cam].mcu_fw_status_percentage = 40;
#endif
	/* Read the firmware present in the text file */
	ret = mcu_bload_update_fw(client);
	if (ret < 0) {
		dev_err(&client->dev, " Write Flash FAIL !!\n");
		goto exit;
	}
	/* Verify the checksum for the update firmware */
	ret = mcu_bload_verify_flash(client, g_bload_crc16);
	if (ret < 0) {
		dev_err(&client->dev, " verify_flash FAIL !!\n");
		goto exit;
	}

	/* Reverting from bootloader mode */
	/* I2C GO Command */
	ret = mcu_bload_go(client);
	if (ret < 0) {
		dev_err(&client->dev, " i2c_bload_go FAIL !!\n");
		goto exit;
	}

	if (mcu_fw_version) {
		dev_dbg(&client->dev, "(%s) - Firmware Updated - (%.32s)\n",
				__func__, mcu_fw_version);
	}
exit:
	return ret;
}

unsigned short int mcu_bload_calc_crc16(unsigned char *buf, int len)
{
	unsigned short int crc = 0;
	int i = 0;

	if (!buf || !(buf + len))
		return 0;

	for (i = 0; i < len; i++)
		crc ^= buf[i];

	return crc;
}

unsigned char mcu_bload_inv_checksum(unsigned char *buf, int len)
{
	unsigned int checksum = 0x00;
	int i = 0;

	if (!buf || !(buf + len))
		return 0;

	for (i = 0; i < len; i++)
		checksum = (checksum + buf[i]);

	checksum &= (0xFF);
	return (~(checksum) + 1);
}

int mcu_bload_ascii2hex(unsigned char ascii)
{
	if (ascii <= '9')
		return (ascii - '0');
	else if ((ascii >= 'a') && (ascii <= 'f'))
		return (0xA + (ascii - 'a'));
	else if ((ascii >= 'A') && (ascii <= 'F'))
		return (0xA + (ascii - 'A'));
	return -EIO;
}

int mcu_bload_get_version(struct i2c_client *client)
{
	int ret = 0;

	/*----------------------------- GET VERSION -------------------- */

	/*   Write Get Version CMD */
	g_bload_buf[0] = BL_GET_VERSION;
	g_bload_buf[1] = ~(BL_GET_VERSION);


	ret = cam_write(client, g_bload_buf, 2);
	if (ret < 0) {
		dev_err(&client->dev, "Write Failed\n");
		return -1;
	}
	/*   Wait for ACK or NACK */
	ret = cam_read(client, g_bload_buf, 1);
	if (ret < 0) {
		dev_err(&client->dev, "Read Failed\n");
		return -1;
	}
	if (g_bload_buf[0] != 'y') {
		/*   NACK Received */
		dev_err(&client->dev, " NACK Received... exiting..\n");
		return -1;
	}
	ret = cam_read(client, g_bload_buf, 1);
	if (ret < 0) {
		dev_err(&client->dev, "Read Failed\n");
		return -1;
	}
	ret = cam_read(client, g_bload_buf, 1);
	if (ret < 0) {
		dev_err(&client->dev, "Read Failed\n");
		return -1;
	}
	/* ---------------- GET VERSION END ------------------- */

	return 0;
}

int mcu_bload_go(struct i2c_client *client)
{
	int ret = 0;

	g_bload_buf[0] = BL_GO;
	g_bload_buf[1] = ~(BL_GO);

	ret = cam_write(client, g_bload_buf, 2);
	if (ret < 0) {
		dev_err(&client->dev, "Write Failed\n");
		return -1;
	}

	ret = cam_read(client, g_bload_buf, 1);
	if (ret < 0) {
		dev_err(&client->dev, "Failed Read 1\n");
		return -1;
	}

	/*   Start Address */
	g_bload_buf[0] = (FLASH_START_ADDRESS & 0xFF000000) >> 24;
	g_bload_buf[1] = (FLASH_START_ADDRESS & 0x00FF0000) >> 16;
	g_bload_buf[2] = (FLASH_START_ADDRESS & 0x0000FF00) >> 8;
	g_bload_buf[3] = (FLASH_START_ADDRESS & 0x000000FF);
	g_bload_buf[4] =
		g_bload_buf[0] ^ g_bload_buf[1] ^ g_bload_buf[2] ^ g_bload_buf[3];

	ret = cam_write(client, g_bload_buf, 5);
	if (ret < 0) {
		dev_err(&client->dev, "Write Failed\n");
		return -1;
	}

	ret = cam_read(client, g_bload_buf, 1);
	if (ret < 0) {
		dev_err(&client->dev, "Failed Read 1\n");
		return -1;
	}

	if (g_bload_buf[0] != RESP_ACK) {
		/*   NACK Received */
		dev_err(&client->dev, " NACK Received... exiting..\n");
		return -1;
	}

	return 0;
}

int mcu_bload_update_fw(struct i2c_client *client)
{
	unsigned long hex_file_size = strlen(mcu_fw_buf);
	unsigned char *wbuf;
	int i = 0, recindex = 0, ret = 0;

	// Dynamically allocate memory for wbuf
	wbuf = kmalloc(MAX_BUF_LEN, GFP_KERNEL);
	if (!wbuf)
		return -ENOMEM;

	for (i = 0; i < hex_file_size; i++) {
		if ((recindex == 0) && (mcu_fw_buf[i] == ':')) {
			/*  debug_printk("Start of a Record\n"); */
		} else if (mcu_fw_buf[i] == CR) {
			/*   No Implementation */
		} else if (mcu_fw_buf[i] == LF) {
			if (recindex == 0) {
				/*   Parsing Complete */
				break;
			}

			/*   Analyze Packet and Send Commands */
			ret = mcu_bload_parse_send_cmd(client, wbuf, recindex);
			if (ret < 0) {
				dev_err(&client->dev, "Error in Processing Commands\n");
				break;
			}

			recindex = 0;

		} else {
			/*   Parse Rec Data */
			ret = mcu_bload_ascii2hex(mcu_fw_buf[i]);
			if (ret < 0) {
				dev_err(&client->dev,
						"Invalid Character - 0x%02x !!\n",
						mcu_fw_buf[i]);
				break;
			}

			wbuf[recindex] = (0xF0 & (ret << 4));
			i++;

			ret = mcu_bload_ascii2hex(mcu_fw_buf[i]);
			if (ret < 0) {
				dev_err(&client->dev,
						"Invalid Character - 0x%02x !!!!\n",
						mcu_fw_buf[i]);
				break;
			}

			wbuf[recindex] |= (0x0F & ret);
			recindex++;
		}
	}

	pr_info("Program FLASH Success !! - CRC = 0x%04x\n",
			g_bload_crc16);

	/* ------------ PROGRAM FLASH END ----------------------- */

	// Free the dynamically allocated memory for wbuf
	kfree(wbuf);

	return ret;
}

int mcu_bload_read(struct i2c_client *client,
		unsigned int g_bload_flashaddr, char *bytearray,
		unsigned int len)
{
	int ret = 0;

	g_bload_buf[0] = BL_READ_MEM;
	g_bload_buf[1] = ~(BL_READ_MEM);

	ret = cam_write(client, g_bload_buf, 2);
	if (ret < 0) {
		dev_err(&client->dev, "Write Failed\n");
		return -1;
	}

	/*   Wait for ACK or NACK */
	ret = cam_read(client, g_bload_buf, 1);
	if (ret < 0) {
		dev_err(&client->dev, "Read Failed\n");
		return -1;
	}

	if (g_bload_buf[0] != RESP_ACK) {
		/*   NACK Received */
		dev_err(&client->dev, " NACK Received... exiting..\n");
		return -1;
	}

	g_bload_buf[0] = (g_bload_flashaddr & 0xFF000000) >> 24;
	g_bload_buf[1] = (g_bload_flashaddr & 0x00FF0000) >> 16;
	g_bload_buf[2] = (g_bload_flashaddr & 0x0000FF00) >> 8;
	g_bload_buf[3] = (g_bload_flashaddr & 0x000000FF);
	g_bload_buf[4] =
		g_bload_buf[0] ^ g_bload_buf[1] ^ g_bload_buf[2] ^ g_bload_buf[3];

	ret = cam_write(client, g_bload_buf, 5);
	if (ret < 0) {
		dev_err(&client->dev, "Write Failed\n");
		return -1;
	}

	/*   Wait for ACK or NACK */
	ret = cam_read(client, g_bload_buf, 1);
	if (ret < 0) {
		dev_err(&client->dev, "Read Failed\n");
		return -1;
	}

	if (g_bload_buf[0] != RESP_ACK) {
		/*   NACK Received */
		dev_err(&client->dev, " NACK Received... exiting..\n");
		return -1;
	}

	g_bload_buf[0] = len - 1;
	g_bload_buf[1] = ~(len - 1);

	ret = cam_write(client, g_bload_buf, 2);
	if (ret < 0) {
		dev_err(&client->dev, "Write Failed\n");
		return -1;
	}

	/*   Wait for ACK or NACK */
	ret = cam_read(client, g_bload_buf, 1);
	if (ret < 0) {
		dev_err(&client->dev, "Read Failed\n");
		return -1;
	}

	if (g_bload_buf[0] != RESP_ACK) {
		/*   NACK Received */
		dev_err(&client->dev, " NACK Received... exiting..\n");
		return -1;
	}

	ret = cam_read(client, bytearray, len);
	if (ret < 0) {
		dev_err(&client->dev, "Read Failed\n");
		return -1;
	}

	return 0;
}


int mcu_bload_write(struct i2c_client *client,
		   unsigned int g_bload_flashaddr)
{
	int ret = 0, i = 0;
	unsigned char option_byte[4], checksum = 0;

	g_bload_buf[0] = BL_WRITE_MEM;
	g_bload_buf[1] = ~(BL_WRITE_MEM);

	ret = cam_write(client, g_bload_buf, 2);
	if (ret < 0) {
		dev_err(&client->dev, "Write Failed\n");
		return -1;
	}

	/*   Wait for ACK or NACK */
	ret = cam_read(client, g_bload_buf, 1);
	if (ret < 0) {
		dev_err(&client->dev, "Read Failed\n");
		return -1;
	}

	if (g_bload_buf[0] != RESP_ACK) {
		/*   NACK Received */
		dev_err(&client->dev, " NACK Received... exiting..\n");
		return -1;
	}

	g_bload_buf[0] = (g_bload_flashaddr & 0xFF000000) >> 24;
	g_bload_buf[1] = (g_bload_flashaddr & 0x00FF0000) >> 16;
	g_bload_buf[2] = (g_bload_flashaddr & 0x0000FF00) >> 8;
	g_bload_buf[3] = (g_bload_flashaddr & 0x000000FF);
	g_bload_buf[4] =
	    g_bload_buf[0] ^ g_bload_buf[1] ^ g_bload_buf[2] ^ g_bload_buf[3];

	ret = cam_write(client, g_bload_buf, 5);
	if (ret < 0) {
		dev_err(&client->dev, "Write Failed\n");
		return -1;
	}

	/*   Wait for ACK or NACK */
	ret = cam_read(client, g_bload_buf, 1);
	if (ret < 0) {
		dev_err(&client->dev, "Read Failed\n");
		return -1;
	}

	if (g_bload_buf[0] != RESP_ACK) {
		/*   NACK Received */
		dev_err(&client->dev, " NACK Received... exiting..\n");
		return -1;
	}

	option_byte[0] = 0xAA;
	option_byte[1] = 0xFE;
	option_byte[2] = 0xFF;
	option_byte[3] = 0xFA;

	g_bload_buf[0] = 4 - 1;
	checksum = g_bload_buf[0];
	for (i = 0; i < 4; i++) {
		g_bload_buf[i + 1] = option_byte[i];
		checksum ^= g_bload_buf[i + 1];
	}
	g_bload_buf[i + 1] = checksum;

	ret = cam_write(client, g_bload_buf, 6);
	if (ret < 0) {
		dev_err(&client->dev, "Write Failed\n");
		return -1;
	}

	/*   Wait for ACK or NACK */
	ret = cam_read(client, g_bload_buf, 1);
	if (ret < 0) {
		dev_err(&client->dev, "Read Failed\n");
		return -1;
	}

	if (g_bload_buf[0] != RESP_ACK) {
		/*   NACK Received */
		dev_err(&client->dev, " NACK Received... exiting..\n");
		return -1;
	}

	return 0;
}



int mcu_bload_verify_flash(struct i2c_client *client,
		unsigned short int orig_crc)
{
	char bytearray[FLASH_READ_LEN];
	unsigned short int calc_crc = 0;
	unsigned int flash_addr = FLASH_START_ADDRESS, i = 0;

	while ((i + FLASH_READ_LEN) <= FLASH_SIZE) {
		memset(bytearray, 0x0, FLASH_READ_LEN);

		if (mcu_bload_read
				(client, flash_addr + i, bytearray, FLASH_READ_LEN) < 0) {
			dev_err(&client->dev, " i2c_bload_read FAIL !!\n");
			return -1;
		}

		calc_crc ^= mcu_bload_calc_crc16(bytearray, FLASH_READ_LEN);
		i += FLASH_READ_LEN;
	}

	if ((FLASH_SIZE - i) > 0) {
		memset(bytearray, 0x0, FLASH_READ_LEN);

		if (mcu_bload_read
				(client, flash_addr + i, bytearray, (FLASH_SIZE - i))
				< 0) {
			dev_err(&client->dev, " i2c_bload_read FAIL !!\n");
			return -1;
		}

		calc_crc ^= mcu_bload_calc_crc16(bytearray, FLASH_READ_LEN);
	}

	if (orig_crc != calc_crc) {
#ifdef DEBUG
		dev_err(&client->dev, " CRC verification fail !! 0x%04x != 0x%04x\n",
				orig_crc, calc_crc);
#endif
		goto exit;
	}

	pr_info(" CRC Verification Success 0x%04x == 0x%04x\n",
			orig_crc, calc_crc);
exit:
	return 0;
}

static int mcu_jump_bload(struct i2c_client *client)
{
	uint32_t payload_len = 0;
	int err = 0;
	uint8_t mc_data[512];

	/*lock semaphore */
	mutex_lock(&g_i2c_mutex);
	/* First Txn Payload length = 0 */
	payload_len = 0;

	mc_data[0] = CMD_SIGNATURE;
	mc_data[1] = CMD_ID_FW_UPDT;
	mc_data[2] = payload_len >> 8;
	mc_data[3] = payload_len & 0xFF;
	mc_data[4] = errorcheck(&mc_data[2], 2);

	err = cam_write(client, mc_data, TX_LEN_PKT);
	if (err != 0) {
		dev_err(&client->dev, " %s(%d) Error - %d\n",
				__func__, __LINE__, err);
		goto exit;
	}
	mc_data[0] = CMD_SIGNATURE;
	mc_data[1] = CMD_ID_FW_UPDT;
	err = cam_write(client, mc_data, 2);
	if (err != 0) {
		dev_err(&client->dev, " %s(%d) Error - %d\n",
				__func__, __LINE__, err);
		goto exit;
	}
exit:
	/* unlock semaphore */
	mutex_unlock(&g_i2c_mutex);
	return err;

}

int check_ecam_mcu_fw_status(struct i2c_client *client, struct cam *priv, int spi_flash_bootdata_changed)
{
	uint8_t ret, loop, retry, err;
	unsigned char fw_version[32] = {0}, txt_fw_version[32] = {0};

	cam_trace(&client->dev, "check_ecam_mcu_fw_status: ENTER\n");

	ret = mcu_get_fw_version(client, fw_version, txt_fw_version);
	cam_trace(&client->dev,
		"check_ecam_mcu_fw_status: mcu_get_fw_version() rc=%d, bootdata_changed=%d\n",
		ret, spi_flash_bootdata_changed);

	if (ret != 0 || spi_flash_bootdata_changed == 0x01) {

#ifdef PROBE_STATUS
		sprintf(probe_status_array[total_num_cam].mcu_fw_status, "Updating MCU FW...");
		probe_status_array[total_num_cam].mcu_fw_status_percentage = 5;
		probe_status_array[total_num_cam].percentage = 20;
#endif

		/*In 0x08b40 sensor product using STM32L476JGY6TR MCU,
		 * There is no option for mcu_jump_bload mode in this MCU
		 */
		if ((ret > 0) & (priv->cam_id != Ox08B40)) {
			err = mcu_jump_bload(client);
			if (err < 0) {
				dev_err(&client->dev, " Cannot go into bootloader mode\n");
				err = -EIO;
				goto fw_update_err;
			}
			msleep(1000);
		} else {
			dev_info(&client->dev, "Using Boot pin for firmware update\n");

			retry = 10;
			while (retry-- > 0) {
				/*NEED TO CHECK*/
				if (toggle_mcu_boot_pin(client, priv) < 0) {
					msleep(100);
					dev_info(&client->dev, "Retry Boot pin toggle\n");
					continue;
				}
				break;
			}
			if (retry <= 0) {
				dev_err(&client->dev, " Cannot go into bootloader mode\n");
				err = -EIO;
				goto fw_update_err;
			}

		}
		dev_info(&client->dev, " Trying to Detect Bootloader mode\n");
		msleep(300);

		for (loop = 0; loop < 11; loop++) {
			err = mcu_bload_get_version(client);
			if (err != 0) {
				/* Trial and Error for 1 second (100ms * 10) */
				dev_err(&client->dev, " MCU INIT = %d\n", loop);
				/*NEED TO CHECK*/
				if (toggle_mcu_boot_pin(client, priv) < 0)
					dev_info(&client->dev, "Retry Boot pin toggle\n");

				if (loop != 10)
					msleep(300+(100*loop));
				else
					msleep(3000);
				continue;
			} else {
				dev_info(&client->dev, " Get Bload Version Success\n");
				break;
			}
		}

		if (loop == 10) {
			dev_err(&client->dev, "Error updating firmware\n");
			err = -EINVAL;
			goto fw_update_err;
		}

#ifdef PROBE_STATUS
		probe_status_array[total_num_cam].mcu_fw_status_percentage = 10;
#endif
		for (loop = 0; loop < 10; loop++) {
			err = mcu_fw_update(client, NULL);
			if (err < 0) {
				dev_err(&client->dev,
						"%s(%d) Error updating firmware.. Retry..\n",
						__func__, __LINE__);

				continue;
			} else {
				dev_err(&client->dev, "Firmware Updated Successfully\n");
#ifdef PROBE_STATUS
				probe_status_array[total_num_cam].percentage = 70;
#endif
				break;
			}

		}
		if (loop == 10) {
			dev_err(&client->dev, "Error Updating Firmware\n");
			err = -EFAULT;
			goto fw_update_err;
		}


		/*NEED TO CHECK*/ /*reset and boot pin handle*/
		if ((serdes_write_16b_reg(client, priv->ser_reassigned_addr,
						priv->boot_reg, MCU_BOOT_LOW)) < 0) {
			dev_err(&client->dev, "%s(%d): Failed\n", __func__, __LINE__);
			err = -EIO;
			goto fw_update_err;
		}
#ifdef DEBUG
		pr_info("Boot pin(%x) : %x ", priv->boot_reg, MCU_BOOT_LOW);
#endif
		if ((serdes_write_16b_reg(client, priv->ser_reassigned_addr,
						priv->reset_reg, MCU_RST_LOW)) < 0) {
			dev_err(&client->dev, "%s(%d): Failed\n", __func__, __LINE__);
			err = -EIO;
			goto fw_update_err;
		}
#ifdef DEBUG
		pr_info("RESET pin(%x) : %x ", priv->reset_reg, MCU_RST_LOW);
#endif
		if ((serdes_write_16b_reg(client, priv->ser_reassigned_addr,
						priv->reset_reg, MCU_RST_HIGH)) < 0) {
			dev_err(&client->dev, "%s(%d): Failed\n", __func__, __LINE__);
			err = -EIO;
			goto fw_update_err;
		}
#ifdef DEBUG
		pr_info("RESET pin(%x) : %x ", priv->reset_reg, MCU_RST_HIGH);
#endif
		/* Allow FW Updated Driver to reboot */
		/*NEED TO CHECK*//*delay*/
		msleep(100);
		/*Maintaining GMSL1 firmware update compatability*/
		for (loop = 0; loop < 10; loop++) {
			err = mcu_get_fw_version(client, fw_version, txt_fw_version);
			if (err < 0) {
				msleep(1000);

				/* See if it is a empty MCU */
				err = mcu_bload_get_version(client);
				if (err < 0) {
					dev_err(&client->dev, " Get Bload Version Fail\n");
				} else {
					dev_err(&client->dev, " Get Bload Version Success\n");

					/* Re-issue GO command to get into user mode */
					if (mcu_bload_go(client) < 0)
						dev_err(&client->dev, " i2c_bload_go FAIL !!\n");
					msleep(1000);
				}

				continue;
			} else {
				dev_err(&client->dev, " Get FW Version Success\n");
				break;
			}
		}
		if (loop == 10) {
			dev_err(&client->dev, "Error updating firmware\n");
			err = -EINVAL;
			goto fw_update_err;
		}

		pr_info("Current Firmware Version - (%.32s).",
				fw_version);

	} else {
		/* Same firmware version in MCU and Text File */
		pr_info("Current Firmware Version - (%.32s)", fw_version);
	}

#ifdef PROBE_STATUS
	sprintf(probe_status_array[total_num_cam].mcu_fw_status, "Completed");
	probe_status_array[total_num_cam].mcu_fw_status_percentage = 100;
	probe_status_array[total_num_cam].percentage = 50;
#endif
	cam_trace(&client->dev,
		"check_ecam_mcu_fw_status: EXIT OK (fw_version=%.32s)\n", fw_version);
	return 0;

fw_update_err:
#ifdef PROBE_STATUS
	sprintf(probe_status_array[total_num_cam].mcu_fw_status, "Error In Updating MCU");
	probe_status_array[total_num_cam].mcu_fw_status_percentage = -1;
#endif
	cam_trace(&client->dev,
		"check_ecam_mcu_fw_status: EXIT FAIL rc=%d\n", err);
	return err;

}

int serdes_configuration(struct cam *priv, struct i2c_client *client)
{

	cam_trace(&client->dev,
		"serdes_configuration: ENTER (phy=%c)\n", priv->phy);

	/* Configuring SIOA Serializer */
	if (priv->phy == PHY_A) {
		if (serdes_parse_regdata(client, SER1_CONF,
			ARRAY_SIZE(SER1_CONF), priv->ser_reassigned_addr) < 0) {
			dev_err(&client->dev,
				"%s: Failed to configure SIOA Serializer\n",
								__func__);
			return -EIO;
		}
		pr_info("configuring SIOA serializer successful\n");
	}

	/* Configuring SIOB Serializer */
	if (priv->phy == PHY_B) {
		if (serdes_parse_regdata(client, SER2_CONF,
			ARRAY_SIZE(SER2_CONF), priv->ser_reassigned_addr) < 0) {
			dev_err(&client->dev,
				"%s: Failed to configure SIOB Serializer\n",
								__func__);
			return -EIO;
		}
		pr_info("configuring SIOB serializer successful\n");
	}


	if (priv->cam_id == Ox08B40) {
		pr_info("*****DESER CONFIG*****");
		if (serdes_parse_regdata(client, DSER_CONF_8CAM,
			ARRAY_SIZE(DSER_CONF_8CAM), priv->dser_addr) < 0) {
			dev_err(&client->dev,
				"%s: Failed to configure DESER Serializer\n",
							__func__);
			return -EIO;
		}

	}

	pr_info("configuring Deserializer Successful\n");

	cam_trace(&client->dev, "serdes_configuration: EXIT OK\n");
	return ERRCODE_SUCCESS;

}

static s32 serdes_config_init(struct i2c_client *client, struct cam *priv)
{
	uint8_t slave_addr = 0;
	int err = 0;

	cam_trace(&client->dev,
		"serdes_config_init: ENTER (phy=%c)\n", priv->phy);

	if (priv->phy == PHY_A) {
		dev_info(&client->dev,
				" Issuing CHIP reset for Deserializer ...\n");
		if ((serdes_write_16b_reg(client, priv->dser_addr,
						0x0010, 0x80)) < 0) {
			dev_err(&client->dev, "%s(%d): Failed\n",
					__func__, __LINE__);
			err = -EIO;
			goto error;
		}
		msleep(100);

		if ((serdes_write_16b_reg(client, priv->dser_addr,
						0x0010, 0x21)) < 0) {
			dev_err(&client->dev, "%s(%d): Failed\n",
					__func__, __LINE__);
			err = -EIO;
			goto error;
		}
		msleep(100);

		/* Checking Whether SIOA serializer I2C Reassignment is Already Done*/
		serdes_read_16b_reg(client,
				priv->ser_reassigned_addr, 0x0000, &slave_addr);
		if (slave_addr == priv->ser_reassigned_addr << 1) {
			dev_info(&client->dev,
				"I2C Reassignment detected.. Skip i2c reassignment...\n");
			goto skip_i2c_reassignment;
		}

		/*I2C Reassignement for SIOA port Serializer*/
		if ((serdes_write_16b_reg(client,
				priv->ser_def_addr, 0x0000, priv->ser_reassigned_addr<<1)) < 0) {
			dev_err(&client->dev, "%s(%d): Failed\n",
					__func__, __LINE__);
			dev_err(&client->dev, " No serializer found on SIOA\n");
			dev_err(&client->dev, " Exiting probe\n");
			return -ENODEV;
		}
		msleep(100);

		dev_info(&client->dev,
			"SIOA Port I2C Reassignment successful\n");
skip_i2c_reassignment:

		/* SIOA port I2C address translation */
		if (serdes_parse_regdata(client, SER1_I2C_CONF,
			ARRAY_SIZE(SER1_I2C_CONF), priv->ser_reassigned_addr) < 0) {
			dev_err(&client->dev,
				"%s: Failed to configure SIOA SerializerI2C translation\n",
						__func__);
			return -EIO;
		}
		dev_info(&client->dev,
			"SIOA Port I2C translated successfully\n");
					/*Change GMSL2 Packet header*/
		if (serdes_parse_regdata(client,	SER1_PKT_HEADER_CHANGE,
					ARRAY_SIZE(SER1_PKT_HEADER_CHANGE),
					priv->ser_reassigned_addr) < 0) {
			dev_err(&client->dev, "%s: Failed to configure SIOA SerializerI2C translation\n",
					__func__);
			return -EIO;
		}
		/* Updating Serializer availabilty */
		ser_status = 0x21;
	} else if (priv->phy == PHY_B) {

		if ((serdes_write_16b_reg(client, priv->dser_addr, 0x0010, 0x22)) < 0) {
			dev_err(&client->dev, "%s(%d): Failed\n",
					__func__, __LINE__);
			err = -EIO;
			goto error;
		}
		msleep(150);

		/* Checking Whether SIOB serializer I2C Reassignment is Already Done*/
		serdes_read_16b_reg(client,
				priv->ser_reassigned_addr, 0x0000, &slave_addr);
		if (slave_addr == priv->ser_reassigned_addr << 1) {
			dev_info(&client->dev,
				"I2C Reassignment detected.. Skip i2c reassignment...\n");
			goto skip_i2c_reassignment_1;
		}
		pr_info("slave_addr = 0x%02x , ser_def = 0x%02x , ser_reassign = 0x%02x\n", slave_addr, priv->ser_def_addr, priv->ser_reassigned_addr);


		/*I2C Reassignement for SIOB port Serializer*/
		if ((serdes_write_16b_reg(client,
				priv->ser_def_addr, 0x0000, priv->ser_reassigned_addr<<1)) < 0) {
			dev_err(&client->dev, "%s(%d): Failed\n",
					__func__, __LINE__);
			dev_err(&client->dev, " No serializer found on SIOB\n");
			dev_err(&client->dev, " Exiting probe\n");
			return -ENODEV;
		}
		msleep(100);

		dev_info(&client->dev,
			"SIOB Port I2C Reassignment successful\n");
skip_i2c_reassignment_1:

		/*Change GMSL2 Packet header*/
		if (serdes_parse_regdata(client,	SER2_PKT_HEADER_CHANGE,
					ARRAY_SIZE(SER2_PKT_HEADER_CHANGE),
					priv->ser_reassigned_addr) < 0) {
			dev_err(&client->dev, "%s: Failed to configure SIOB SerializerI2C translation\n",
					__func__);
			return -EIO;
		}

		/* SIOB I2C Translation */
		if (serdes_parse_regdata(client,
				SER2_I2C_CONF, ARRAY_SIZE(SER2_I2C_CONF),
					priv->ser_reassigned_addr) < 0) {
			dev_err(&client->dev,
				"%s: Failed to configure SIOA SerializerI2C translation\n",
					__func__);
			return -EIO;
		}
		pr_info("SIOB Port I2C translated successfully\n");
		msleep(100);

		/* Updating Serializer availabilty */
		if (ser_status == 0x21)
			ser_status = 0x23;
		else
			ser_status = 0x22;
	} else {
		dev_err(&client->dev,
			"Device tree SIO ports Parse Unsuccessful\n");
		return -EINVAL;
	}

	/*Set Boot pin low*/
	if ((serdes_write_16b_reg(client,
			priv->ser_reassigned_addr, 0x02BE, MCU_BOOT_LOW)) < 0) {
		dev_err(&client->dev, "%s(%d): Failed\n",
				__func__, __LINE__);
		return -EIO;
	}

	/*Trigger Pin mapping*/
	if ((serdes_write_16b_reg(client,
			priv->ser_reassigned_addr, 0x02D3, 0xC4)) < 0) {
		dev_err(&client->dev, "%s(%d): Failed\n",
				__func__, __LINE__);
		return -EIO;
	}

	/* Setting CAM to MASTER_MODE */
	if ((serdes_write_16b_reg(client,
			priv->ser_reassigned_addr, 0x02D6, 0x40)) < 0) {
		dev_err(&client->dev, "%s(%d): Failed\n",
				__func__, __LINE__);
		return -EIO;
	}

	cam_trace(&client->dev, "serdes_config_init: EXIT OK\n");
	return ERRCODE_SUCCESS;
error:
	return err;
}

static int cam_power_on(struct camera_common_data *s_data)
{
	int errcode = 0;
	struct cam *priv = (struct cam *)s_data->priv;
	struct camera_common_power_rail *pw = &priv->power;

	if (!priv || !priv->pdata)
		return -EINVAL;
	dev_dbg(&priv->i2c_client->dev, "%s: power on\n", __func__);
	cam_trace(&priv->i2c_client->dev, "cam_power_on: ENTER\n");

	if (priv->pdata && priv->pdata->power_on) {
		errcode = priv->pdata->power_on(pw);
		if (errcode)
			dev_err(&priv->i2c_client->dev, "%s failed.\n",
							__func__);
		else
			pw->state = SWITCH_ON;
		cam_trace(&priv->i2c_client->dev,
			"cam_power_on: EXIT rc=%d (board power_on)\n", errcode);
		return errcode;
	}

	usleep_range(1350, 1360);

	pw->state = SWITCH_ON;
	cam_trace(&priv->i2c_client->dev, "cam_power_on: EXIT OK\n");
	return ERRCODE_SUCCESS;

}

static int cam_power_put(struct cam *priv)
{
	struct camera_common_power_rail *pw = &priv->power;

	if (!priv || !priv->pdata)
		return -EINVAL;

	if (unlikely(!pw))
		return -EFAULT;

	pw->avdd = NULL;
	pw->iovdd = NULL;

	return ERRCODE_SUCCESS;
}


static int cam_power_get(struct cam *priv)
{
	struct camera_common_power_rail *pw = &priv->power;
	struct camera_common_pdata *pdata = priv->pdata;
	const char *mclk_name;
	const char *parentclk_name;
	struct clk *parent;
	int errcode = 0;

	if (!priv || !priv->pdata)
		return -EINVAL;
	cam_trace(&priv->i2c_client->dev, "cam_power_get: ENTER\n");

	mclk_name = priv->pdata->mclk_name ? priv->pdata->mclk_name : "cam_mclk1";

	if (priv->pdata->mclk_name) {
		pw->mclk = devm_clk_get(&priv->i2c_client->dev, mclk_name);
		if (IS_ERR(pw->mclk)) {
			dev_err(&priv->i2c_client->dev,
				"unable to get clock %s\n", mclk_name);
			return PTR_ERR(pw->mclk);
		}

		parentclk_name = priv->pdata->parentclk_name;
		if (parentclk_name) {
			parent = devm_clk_get(&priv->i2c_client->dev,
							parentclk_name);
			if (IS_ERR(parent))
				dev_err(&priv->i2c_client->dev,
					"unable to get parent clcok %s",
							parentclk_name);
			else
				clk_set_parent(pw->mclk, parent);
		}
	}

	if (pdata->regulators.avdd)
		errcode |=
			camera_common_regulator_get(&priv->i2c_client->dev, &pw->avdd,
				pdata->regulators.avdd);

	if (pdata->regulators.iovdd)
		errcode |=
			camera_common_regulator_get(&priv->i2c_client->dev, &pw->iovdd,
				pdata->regulators.iovdd);

	if (errcode) {
		dev_err(&priv->i2c_client->dev,
				"%s: unable to get regulator(s)\n", __func__);
		goto done;
	}

done:
	pw->state = SWITCH_OFF;
	cam_trace(&priv->i2c_client->dev, "cam_power_get: EXIT rc=%d\n", errcode);
	return errcode;
}

static int ecam_mcu_firmware_load(struct i2c_client *client)
{
	uint8_t errcode = ERRCODE_SUCCESS;
	unsigned long txt_fw_pos = 0;

	cam_trace(&client->dev,
		"ecam_mcu_firmware_load: ENTER (fw_name=%s)\n", mcu_fw_name);

	/* Request firmware from the rootfs */
	errcode = request_firmware(&mcu_fw, mcu_fw_name, &client->dev);
	if (errcode < 0) {
		errcode = -ENOENT;
		goto mcu_fw_load_fail;
	}

	txt_fw_pos = mcu_fw->size - VERSION_FILE_OFFSET;
	mcu_fw_buf = kmalloc(mcu_fw->size+1, GFP_KERNEL);
	mcu_fw_buf[mcu_fw->size] = '\0';
	memcpy(mcu_fw_buf, mcu_fw->data, mcu_fw->size);

	cam_trace(&client->dev,
		"ecam_mcu_firmware_load: EXIT OK (size=%zu bytes)\n",
		mcu_fw->size);

mcu_fw_load_fail:
	return errcode;
}

/*
 * This function gets the e-con specific data from DT
 */
int8_t	ecam_specific_dt_parse(struct i2c_client *client, struct cam *priv)
{
	uint32_t mipi_lane = 0, mipi_clk = 0;
	int8_t errcode = ERRCODE_SUCCESS;
	const char *str;
	struct device_node *node = client->dev.of_node;
	struct device_node *ser_node;

	cam_trace(&client->dev, "ecam_specific_dt_parse: ENTER\n");

	errcode = of_property_read_u32(node, "camera_mipi_lanes", &mipi_lane);
	if (errcode < ERRCODE_SUCCESS) {
		dev_err(&client->dev, "Can not get Camera MIPI Lanes\n");
		goto error;
	}
	priv->mipi_lane_config = mipi_lane;

	errcode = of_property_read_u32(node, "camera-mipi-clk", &mipi_clk);
	if (errcode) {
		dev_err(&client->dev,
			"camera mipi clk is missing or invalid\n");
		goto error;
	}
	priv->mipi_clk_config = mipi_clk;

	errcode = of_property_read_string(node, "mcu_fw_name", &mcu_fw_name);
	if (!errcode) {
		pr_info("Firmware name from the device tree is > %s\n",
							mcu_fw_name);
	} else {
		pr_info("Unable to get mcu firmware name form Device tree\n");
		goto error;
	}

	errcode = of_property_read_string(node, "dev_name", &dev_name_comp[strm_thrd_num]);
	if (!errcode)
		pr_info("Dev_Name from the Device tree is %s\n",
				dev_name_comp[strm_thrd_num]);
	else {
		pr_info("Unable to get Dev name form Device tree\n");
		goto error;
	}

	/*Identifying Deserializer SIO port for
	 * I2C Address Reassignment and Translation
	 */
	errcode = of_property_read_string(node, "sio-port", &str);
	if (!errcode) {
		if (!strcmp(str, "A")) {
			priv->phy = PHY_A;
			pr_info("Current SIO ports is %c\n", priv->phy);
			/* RESET status if PHYA */
			ser_status = 0x20;
		} else if (!strcmp(str, "B")) {
			priv->phy = PHY_B;
			pr_info("Current SIO ports is %c\n", priv->phy);
		} else {
			dev_err(&client->dev, "Invalid SIO Port");
			errcode = -EINVAL;
			goto error;
		}
	} else {
		dev_err(&client->dev,
			"No SIO port mentioned in device tree\n");
		goto error;
	}

	/* read SER properties from DT */
	ser_node = of_parse_phandle(node, "nvidia,gmsl-ser-device", 0);
	if (ser_node == NULL) {
		dev_err(&client->dev,
			"missing %s handle\n",
				"nvidia,gmsl-ser-device");
		errcode = -EINVAL;
		goto error;
	}
	errcode = of_property_read_u32(ser_node, "reg", &priv->ser_reassigned_addr);
	if (errcode < 0) {
		dev_err(&client->dev, "serializer reg not found\n");
		goto error;
	}
#ifdef DEBUG
	pr_info("priv->ser_reassigned_addr = %x", priv->ser_reassigned_addr);
#endif
	errcode = of_property_read_u32(ser_node, "def-addr", &priv->ser_def_addr);
	if (errcode < 0) {
		dev_err(&client->dev, "serializer def-addr not found in DTB\n");
		goto error;
	}

	errcode = of_property_read_u32(ser_node, "reset_reg", &priv->reset_reg);
	if (errcode < 0) {
		dev_err(&client->dev, "serializer reset_reg not found in DTB\n");
		goto error;
	}

	errcode = of_property_read_u32(ser_node, "boot_reg", &priv->boot_reg);
	if (errcode < 0) {
		dev_err(&client->dev, "serializer boot_reg not found in DTB\n");
		goto error;
	}
	priv->dser_addr = DES_ADDR;

	if (priv->cam_id == Ox08B40)
		g_max_cam = 4;
	else
		g_max_cam = 4;

	cam_trace(&client->dev,
		"ecam_specific_dt_parse: EXIT OK (mipi_lane=%d, mipi_clk=%d, sio=%c)\n",
		priv->mipi_lane_config, priv->mipi_clk_config, priv->phy);

error:
	return errcode;
}


/*
 * This function gets the camera specific data from DT
 */
static struct camera_common_pdata *cam_parse_dt(struct i2c_client *client)
{
	struct device_node *node = client->dev.of_node;
	struct camera_common_pdata *board_priv_pdata;
	const struct of_device_id *match;
	int err;

	cam_trace(&client->dev, "cam_parse_dt: ENTER\n");

	if (!node)
		return NULL;

	match = of_match_device(cam_of_match, &client->dev);
	if (!match) {
		dev_err(&client->dev, "Failed to find matching dt id\n");
		return NULL;
	}

	board_priv_pdata =
		devm_kzalloc(&client->dev, sizeof(*board_priv_pdata), GFP_KERNEL);
	if (!board_priv_pdata)
		return NULL;

	err = camera_common_parse_clocks(&client->dev, board_priv_pdata);
	if (err) {
		dev_err(&client->dev, "Failed to find clocks\n");
		goto error;
	}

	board_priv_pdata->use_cam_gpio =
		of_property_read_bool(node, "cam,use-cam-gpio");

	err =
		of_property_read_string(node, "avdd-reg",
				&board_priv_pdata->regulators.avdd);
	if (err) {
		dev_err(&client->dev, "avdd-reg not in DT\n");
		goto error;
	}
	err =
		of_property_read_string(node, "iovdd-reg",
				&board_priv_pdata->regulators.iovdd);
	if (err) {
		dev_err(&client->dev, "iovdd-reg not in DT\n");
		goto error;
	}

	board_priv_pdata->has_eeprom =
		of_property_read_bool(node, "has-eeprom");

	cam_trace(&client->dev, "cam_parse_dt: EXIT OK\n");
	return board_priv_pdata;

error:
	devm_kfree(&client->dev, board_priv_pdata);
	return NULL;
}

static int mcu_get_serial_num(struct i2c_client *client,
		unsigned char *serial_num)
{
	uint32_t payload_len = 0;
	uint8_t errcode = ERRCODE_SUCCESS, orig_crc = 0, calc_crc = 0;
	int ret = 0, err = 0, loop, retry = 5;
	uint8_t mc_data[512], mc_ret_data[512];

	/* lock semaphore */
	mutex_lock(&g_i2c_mutex);

	while (retry-- > 0) {
		/* Query firmware version from MCU */
		payload_len = 0;

		mc_data[0] = CMD_SIGNATURE;
		mc_data[1] = CMD_ID_READ_SERIAL_NUMBER;
		mc_data[2] = payload_len >> 8;
		mc_data[3] = payload_len & 0xFF;
		mc_data[4] = errorcheck(&mc_data[2], 2);
		err = cam_write(client, mc_data, TX_LEN_PKT);

		mc_data[0] = CMD_SIGNATURE;
		mc_data[1] = CMD_ID_READ_SERIAL_NUMBER;

		err = cam_write(client, mc_data, 2);
		if (err != 0) {
			dev_err(&client->dev,
					" %s(%d) MCU CMD ID Write PKT SerNum Error - %d",
					__func__, __LINE__, ret);
			ret = -EIO;
			continue;
		}

		err = cam_read(client, mc_ret_data, RX_LEN_PKT);

		if (err != 0) {
			dev_err(&client->dev,
					" %s(%d) MCU CMD ID Read PKT SerNum Error - %d",
					__func__, __LINE__, ret);
			ret = -EIO;
			continue;
		}

		/* Verify CRC */
		orig_crc = mc_ret_data[4];
		calc_crc = errorcheck(&mc_ret_data[2], 2);
		if (orig_crc != calc_crc) {
			dev_err(&client->dev,
					" %s(%d) MCU CMD ID SerNum Error CRC 0x%02x != 0x%02x",
					__func__, __LINE__, orig_crc, calc_crc);
			ret = -EINVAL;
			continue;
		}
		errcode = mc_ret_data[5];
		if (errcode != ERRCODE_SUCCESS) {
			dev_err(&client->dev,
					" %s(%d) MCU CMD ID SerNum Errcode - 0x%02x", __func__,
					__LINE__, errcode);
			ret = -EIO;
			continue;
		}
		/* Read the serial num from MCU*/
		payload_len =
			((mc_ret_data[2] << 8) | mc_ret_data[3]) + HEADER_FOOTER_SIZE;
		memset(mc_ret_data, 0x00, payload_len);
		err = cam_read(client, mc_ret_data, payload_len);
		if (err != 0) {
			dev_err(&client->dev,
					" %s(%d) MCU fw CMD ID Read SerNum Error - %d", __func__,
					__LINE__, ret);
			ret = -EIO;
			continue;
		}
		/* Verify Errcode */
		errcode = mc_ret_data[payload_len - 1];
		if (errcode != ERRCODE_SUCCESS) {
			dev_err(&client->dev,
					" %s(%d) MCU SerNum CMD ID Read Payload Error - 0x%02x", __func__,
					__LINE__, errcode);
			ret = -EIO;
			continue;
		}
		if (ret == ERRCODE_SUCCESS)
			break;
	}


	if (retry < 0 && ret != ERRCODE_SUCCESS) {
		pr_info(" %s with exit code = %d %d", __func__, ret, __LINE__);
		goto exit;
	}
	for (loop = 0 ; loop < 15 ; loop++)
		*(serial_num+loop) = mc_ret_data[2+loop];

	serial_num[loop] = '\0';

	ret = ERRCODE_SUCCESS;

exit:
	/* unlock semaphore */
	mutex_unlock(&g_i2c_mutex);

	return ret;
}


static int ecam_lens_config_load(struct i2c_client *client)
{
	int ret = 0;
	/* Request lens configuration file from rootfs */
	ret = request_firmware(&lens_cfg_file, lens_config_filename, &client->dev);
	if (ret < 0)
		return -ENOENT;

	lens_config_buf = kmalloc(lens_cfg_file->size + 1, GFP_KERNEL);
	lens_config_buf[lens_cfg_file->size] = '\0';
	memcpy(lens_config_buf, lens_cfg_file->data, lens_cfg_file->size);
	return 0;
}

static void parse_csv_line(char *line)
{
	char *token = NULL;
	char *newline = NULL;
	char *file_lens_id = NULL;

	while ((newline = strsep(&line, "\n")) != NULL) {

		token = strsep(&newline, ",");
		file_lens_id = token;

		token = strsep(&newline, ",");
		if (strcmp(file_lens_id, lens_identifier) == 0) {
			isp_bootdata_name = kmalloc(strlen(token) + 1, GFP_KERNEL);
			strcpy(isp_bootdata_name, token);
			if (strcmp(isp_bootdata_name, "NARROW") == 0) {
				boot_identified = NARROW_ANGLE;
				pr_info("Lens identifier - NARROW: %s\n", lens_identifier);
			} else {
				boot_identified = WIDE_ANGLE;
				pr_info("Lens identifier - WIDE: %s\n", lens_identifier);
			}
			break;
		}
	}
}

static int mcu_lens_config_load(struct i2c_client *client)
{
	uint32_t payload_len = 0;
	int retry = 5, err = 0;
	uint8_t mc_data[512];

	while (retry-- > 0) {
		payload_len = 2;

		mc_data[0] = CMD_SIGNATURE;
		mc_data[1] = CMD_ID_SHARE_LENS_ID;
		mc_data[2] = payload_len >> 8;
		mc_data[3] = payload_len & 0xFF;
		mc_data[4] = errorcheck(&mc_data[2], 2);

		err = cam_write(client, mc_data, TX_LEN_PKT);
		if (err != 0) {
			dev_err(&client->dev,
					" %s(%d) MCU Stream On Write Error - %d\n",
					__func__, __LINE__, err);
			continue;
		}

		mc_data[0] = CMD_SIGNATURE;
		mc_data[1] = CMD_ID_SHARE_LENS_ID;
		mc_data[2] = boot_identified >> 8;
		mc_data[3] = boot_identified & 0xFF;
		mc_data[4] = errorcheck(&mc_data[2], 2);

		err = cam_write(client, mc_data, 5);
		if (err != 0) {
			dev_err(&client->dev,
					" %s(%d) MCU Stream On Write Error - %d\n",
					__func__, __LINE__, err);
			continue;
		}
	}
	msleep(20);
	/* unlock semaphore */

	return ERRCODE_SUCCESS;
}


int cam_get_serial_number(struct i2c_client *client)
{
	int err = ERRCODE_SUCCESS;
	uint8_t serial_num_len = 0, loop = 0;

	cam_trace(&client->dev, "cam_get_serial_number: ENTER\n");

	memset(serial_num, 0x00, sizeof(serial_num));
	memset(lens_identifier, 0x00, sizeof(lens_identifier));

	msleep(20);

	/* Load lens configuration file */
	err = ecam_lens_config_load(client);
	if (err < 0) {
		dev_err(&client->dev, "Unable to read lens configuration file\n");
		return err;
	}

	msleep(20);

	err = mcu_get_serial_num(client, serial_num);
	if (err < 0) {
		dev_err(&client->dev, "Get Serial Num Fail\n");
		return err;
	}

	if (serial_num[0] != '\0')
		dev_info(&client->dev, "Camera Serial Number: %.16s\n", serial_num);
	serial_num_len = strlen(serial_num);

	if (serial_num_len < 8) {
		dev_err(&client->dev, "Invalid Serial Number\n");
		return err;
	}

	for (loop = 3; loop < 8; loop++)
		lens_identifier[loop-3] = serial_num[loop];

	lens_identifier[loop] = '\0';

	/* Get Patch ISP Boot data file */
	parse_csv_line(lens_config_buf);

	msleep(20);

	err = mcu_lens_config_load(client);
	if (err < 0) {
		dev_err(&client->dev, "Unable to read lens configuration file\n");
		return err;
	}

	cam_trace(&client->dev, "cam_get_serial_number: EXIT OK\n");
	return ERRCODE_SUCCESS;
}


/*
 * This function is called while Loading the Driver
 */
static int ecam_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int8_t errcode = ERRCODE_SUCCESS;
	struct camera_common_data *common_data;
	struct device_node *node = client->dev.of_node;
	struct cam *priv;
	uint8_t loop;
	int8_t retry;
	ktime_t cam_probe_t_start = ktime_get();

	uint8_t spi_flash_bootdata_changed = 0;

	cam_trace(&client->dev,
		"ecam_probe: ENTER (i2c addr=0x%02x, cam_num_so_far=%d)\n",
		client->addr, num_cam_probed);

	/*
	 * Handling Load Switch to enable power to all modules
	 */

	if (!IS_ENABLED(CONFIG_OF) || !node) {
		errcode = -EINVAL;
		goto exit;
	}
	cam_trace(&client->dev, "step 1/17: device-tree node OK\n");

	/* Allocating memories for required Data */
	common_data = devm_kzalloc(&client->dev,
				sizeof(struct camera_common_data),
							GFP_KERNEL);
	if (!common_data) {
		errcode = -ENOMEM;
		goto exit;
	}

	priv = devm_kzalloc(&client->dev, sizeof(struct cam) +
		sizeof(struct v4l2_ctrl *) * MAX_NUM_CONTROLS,
			GFP_KERNEL);

	if (!priv) {
		errcode = -ENOMEM;
		goto exit;
	}
	cam_trace(&client->dev, "step 2/17: common_data/priv allocated\n");

	/* Identify the connected Device */
	priv->cam_num = num_cam_probed;
#ifdef DEBUG
	pr_warn("num_cam_probed = %d\n", num_cam_probed);
#endif
	num_cam_probed++;
	priv->cam_id = id->driver_data;
	cam_trace(&client->dev,
		"step 3/17: cam_num=%d cam_id=%d\n", priv->cam_num, priv->cam_id);

#ifdef PROBE_STATUS
	errcode = create_probe_sysfs_entries();
	if (errcode) {
		pr_info("Unable to create probe sysfs entries");
		/*
		 * Avoiding return to continue probning even if the
		 * creation of sysfs fails
		 */
	}
#endif
	priv->pwm_dev = devm_pwm_get(&client->dev, NULL);
	if (IS_ERR(priv->pwm_dev)) {
		errcode = PTR_ERR(priv->pwm_dev);
		if (errcode != -EPROBE_DEFER)
			dev_err(&client->dev,
				"Unable to request PWM for trigger\n");
		priv->got_pwm_handle = false;
	} else {
		priv->got_pwm_handle = true;
		dev_info(&client->dev, "Got PWM for trigger\n");
	}
	cam_trace(&client->dev, "step 4/17: PWM trigger handle acquired=%d\n",
		priv->got_pwm_handle);

	priv->pdata = cam_parse_dt(client);
	if (!priv->pdata) {
		dev_err(&client->dev, "Unable to parse data from dtb\n");
		errcode = -EFAULT;
		goto exit;
	}
	cam_trace(&client->dev, "step 5/17: cam_parse_dt() OK\n");

	/* Gets e-con specific data from DTB */
	errcode = ecam_specific_dt_parse(client, priv);
	if (errcode < 0) {
		dev_err(&client->dev,
			"Unable tp parse ecam specific data from dtb");
		errcode = -EFAULT;
		goto exit;
	}
	cam_trace(&client->dev,
		"step 6/17: ecam_specific_dt_parse() OK (phy=%c)\n", priv->phy);

	if (g_prev_connected_cam_id != priv->cam_id)
		is_fw_loaded = 0;

	/* Loading Fw from rootfs */
	if (!is_fw_loaded) {

		if (ecam_mcu_firmware_load(client) < ERRCODE_SUCCESS) {
			dev_err(&client->dev, "failed to load mcu firmware\n");
			errcode = -ENOENT;
			goto exit;
		} else {
			pr_info("Firmware Successfully read by driver\n");
			is_fw_loaded = 1;
		}
	}
	cam_trace(&client->dev,
		"step 7/17: MCU firmware image ready (is_fw_loaded=%d)\n",
		is_fw_loaded);

	mutex_init(&g_i2c_mutex);

#ifdef DEBUG
	pr_info("phy-A read %x(err_cam:%d)-resetregL:%x,seradd:%x ",
			priv->phy, total_num_cam, priv->reset_reg, priv->ser_reassigned_addr);
#endif

	/* Create's Sysfs Directory */
	errcode = create_sysfs_dir(client, priv);
	strm_thrd_num++;

	priv->i2c_client = client;
	priv->s_data = common_data;
	priv->subdev = &common_data->subdev;
	priv->subdev->dev = &client->dev;
	priv->s_data->dev = &client->dev;
	common_data->priv = (void *)priv;

	errcode = cam_power_get(priv);
	if (errcode)
		goto exit;
	cam_trace(&client->dev, "step 8/17: cam_power_get() OK\n");

	errcode = cam_power_on(common_data);
	if (errcode)
		goto exit;
	cam_trace(&client->dev, "step 9/17: cam_power_on() OK\n");

	errcode = serdes_config_init(client, priv);
	if (errcode < ERRCODE_SUCCESS) {
		dev_err(&client->dev, "serdes_config_init_failed\n");
		errcode = -EIO;
		goto exit;
	}
	cam_trace(&client->dev,
		"step 10/17: serdes_config_init() OK (ser_reassigned_addr=0x%02x)\n",
		priv->ser_reassigned_addr);



#ifdef PROBE_STATUS
	if (sysfs_probe_dir)
		create_cam_specific_sysfs_entries(total_num_cam, priv->cam_id);
#endif

	if (serdes_write_16b_reg(client, priv->ser_reassigned_addr,
		priv->boot_reg, MCU_BOOT_LOW) < 0) {
		dev_err(&client->dev, "%s(%d): Failed\n",
				__func__, __LINE__);
		return -EIO;
	}
#ifdef DEBUG
	pr_info("RESET pin-(%x) : %x ", priv->reset_reg, MCU_RST_LOW);
#endif
	msleep(20);
	/* Toggeling MCU RESET */
	errcode = toggle_mcu_reset_pin(client, priv);
	if (errcode)
		goto exit;
	cam_trace(&client->dev, "step 11/17: toggle_mcu_reset_pin() OK\n");

	msleep(100);

#ifdef PROBE_STATUS
	probe_status_array[total_num_cam].percentage = 10;
#endif
	/* Gets FW version and Update FW if required */
	errcode = check_ecam_mcu_fw_status(client, priv, spi_flash_bootdata_changed);
	if (errcode < 0) {
		dev_err(&client->dev, "%s(%d):ecam_mcu_fw_status Failed\n",
				__func__, __LINE__);
		goto exit;

	}
	cam_trace(&client->dev, "step 12/17: check_ecam_mcu_fw_status() OK\n");

	/* ISP Boot data update config , by read the serial number*/
	errcode = cam_get_serial_number(client);
	if (errcode != ERRCODE_SUCCESS) {
		dev_err(&client->dev, "%s(%d):Camera serial number read failed\n",
				__func__, __LINE__);
	}
	cam_trace(&client->dev,
		"step 13/17: cam_get_serial_number() rc=%d\n", errcode);

#ifdef PROBE_STATUS
	probe_status_array[total_num_cam].percentage = 90;
#endif

	/* MCU Related Configurations */
	errcode = mcu_configurations(client, priv);
	if (errcode < 0) {
		dev_err(&client->dev, "Error in mcu Configurations(%d)",
			__LINE__);
	}
	cam_trace(&client->dev,
		"step 14/17: mcu_configurations() rc=%d\n", errcode);

	errcode = ecam_mcu_core_initialize(client, priv);
	if (errcode < 0) {
		dev_err(&client->dev, "%s(%d):ecam MCU Init Failed\n",
				__func__, __LINE__);
		goto exit;
	}
	cam_trace(&client->dev, "step 15/17: ecam_mcu_core_initialize() OK\n");

	if (serdes_configuration(priv, client) < 0) {
		msleep(100);
		dev_info(&client->dev, "serdes configuration\n");
	}
	cam_trace(&client->dev, "step 16/17: serdes_configuration() done\n");

	/* SYSFS call for IMU read*/
	if (priv->cam_id == Ox08B40) {

		errcode = create_value_read_sysfs_entries(priv->cam_num, priv->cam_id);
		if (errcode) {
			pr_info("Unable to create IMU Read sysfs entries");
			/*
			 * Avoiding return to continue probning even if the
			 * creation of sysfs fails
			 */
		}
	}

	for (loop = 0; loop < frm_fmt_size; loop++) {
		priv->mcu_cam_frmfmt[loop].framerates = devm_kzalloc(&client->dev,
					sizeof(int) * MAX_NUM_FRATES, GFP_KERNEL);
		if (!priv->mcu_cam_frmfmt[loop].framerates) {
			errcode = -ENOMEM;
			goto exit;
		}
	}

	/* Enumerate Formats */
	retry = 5;
	while (--retry >= 0) {
		if (mcu_list_fmts(client, priv->stream_info,
					&frm_fmt_size, priv) < 0) {
			dev_err(&client->dev, "List Fmts failure.retrying\n");
			continue;
		} else {
			break;
		}
	}

	if (retry < 0) {
		dev_err(&client->dev, "Unable to List Fmts.\n");
		errcode = -EFAULT;
		goto exit;
	}
	cam_trace(&client->dev,
		"step 17/17: mcu_list_fmts() OK (frm_fmt_size=%d)\n", frm_fmt_size);

	common_data->ops = NULL;
	common_data->ctrl_handler = &priv->ctrl_handler;
	common_data->frmfmt = priv->mcu_cam_frmfmt;
	common_data->colorfmt =
	camera_common_find_datafmt(DEFAULT_DATAFMT);
	common_data->power = &priv->power;
	common_data->ctrls = priv->ctrls;
	common_data->priv = (void *)priv;
	common_data->numctrls = priv->num_ctrls;
	common_data->numfmts = frm_fmt_size;
	common_data->def_mode = 0;
	if (priv->cam_id == Ox08B40) {
		common_data->def_width = Ox08B40_DEFAULT_WIDTH;
		common_data->def_height = Ox08B40_DEFAULT_HEIGHT;
	}
	common_data->fmt_width = common_data->def_width;
	common_data->fmt_height = common_data->def_height;
	common_data->def_clk_freq = 24000000;

	priv->i2c_client = client;
	priv->s_data = common_data;
	priv->subdev = &common_data->subdev;
	priv->subdev->dev = &client->dev;
	priv->s_data->dev = &client->dev;
	priv->prev_index = 0xFFFE;

	errcode = camera_common_initialize(common_data,
			"ecam_yuv");
	if (errcode) {
		dev_err(&client->dev, "Failed to initialize ecam_yuv.\n");
		goto exit;
	}
	cam_trace(&client->dev, "camera_common_initialize() OK\n");

	v4l2_i2c_subdev_init(priv->subdev, client, &cam_subdev_ops);
	cam_trace(&client->dev, "v4l2_i2c_subdev_init() done\n");

	errcode = cam_ctrls_init(priv, priv->mcu_ctrl_info);
	if (errcode)
		goto exit;
	cam_trace(&client->dev, "cam_ctrls_init() OK (num_ctrls=%d)\n",
		priv->num_ctrls);

	priv->subdev->internal_ops = &cam_subdev_internal_ops;
	priv->subdev->flags |=
		V4L2_SUBDEV_FL_HAS_DEVNODE | V4L2_SUBDEV_FL_HAS_EVENTS;

	/*
	 * To unload the module driver module,
	 * Set (struct v4l2_subdev *)priv->subdev->sd to NULL.
	 * Refer tegracam_v4l2subdev_register() in tegracam_v4l2.c
	 */
	if (priv->subdev->owner == THIS_MODULE) {
		common_data->owner = priv->subdev->owner;
		priv->subdev->owner = NULL;
	} else {
		// It shouldn't come here in probe();
		;
	}

#if defined(CONFIG_MEDIA_CONTROLLER)
	priv->pad.flags = MEDIA_PAD_FL_SOURCE;
	priv->subdev->entity.ops = &cam_media_ops;
	errcode = tegra_media_entity_init(&priv->subdev->entity, 1,
						&priv->pad, true, true);
	if (errcode < ERRCODE_SUCCESS) {
		dev_err(&client->dev, "unable to init media entity\n");
		goto exit;
	}
	cam_trace(&client->dev, "tegra_media_entity_init() OK\n");
#endif

	errcode = v4l2_async_register_subdev(priv->subdev);
	if (errcode)
		goto exit;
	cam_trace(&client->dev, "v4l2_async_register_subdev() OK\n");

	if (priv->cam_id == Ox08B40)
		dev_info(&client->dev, "Detected Ox08B40 Sensor\n");

#ifdef PROBE_STATUS
	sprintf(probe_status_array[total_num_cam].status, "Probed");
	probe_status_array[total_num_cam].percentage = 100;
#endif
	total_num_cam++;
	total_cam_probed++;

	if (priv->got_pwm_handle) {
		// Set PWM Trigger to default
		pwm_disable(priv->pwm_dev);
		pwm_config(priv->pwm_dev, PWM_5HZ_DUTY, PWM_5HZ_PERIOD);
		pwm_enable(priv->pwm_dev);
		dev_info(&client->dev, "Setting PWM to 5HZ\n");
	}

exit:
	dev_info(&client->dev, " ser_status=%x\n", ser_status);
	if (serdes_write_16b_reg(client,
			priv->dser_addr, 0x0010, ser_status) < 0) {
		dev_err(&client->dev, "%s(%d): Failed\n",
				__func__, __LINE__);
		errcode = -EIO;
	}

	/* Storing previous cam_id */
	g_prev_connected_cam_id = priv->cam_id;

	if (num_cam_probed == 4) {
#ifdef PROBE_STATUS
		sprintf(probe_status_str, "Completed");
#endif

	}

#ifdef PROBE_STATUS
	if (errcode != ERRCODE_SUCCESS) {
		sprintf(probe_status_array[total_num_cam].status, "Error Probing Camera");
		probe_status_array[total_num_cam].percentage = -1;
	}
#endif

	cam_trace(&client->dev,
		"ecam_probe: EXIT rc=%d, elapsed=%lld ms\n",
		errcode,
		ktime_to_ms(ktime_sub(ktime_get(), cam_probe_t_start)));

	return errcode;
}

static int ecam_remove(struct i2c_client *client)
{
	uint8_t errcode = ERRCODE_SUCCESS;

	struct camera_common_data *s_data = to_camera_common_data(&client->dev);
	struct cam *priv = (struct cam *)s_data->priv;
	int loop = 0;

	if (!priv || !priv->pdata)
		return -1;

	v4l2_async_unregister_subdev(priv->subdev);
#if defined(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&priv->subdev->entity);
#endif

	if (priv->got_pwm_handle)
		pwm_disable(priv->pwm_dev);

	num_cam_probed--;
	v4l2_ctrl_handler_free(&priv->ctrl_handler);
	cam_power_put(priv);
	camera_common_remove_debugfs(s_data);

	mutex_destroy(&g_i2c_mutex);

	remove_sysfs_dir();

	if (is_fw_loaded == 1) {

		release_firmware(mcu_fw);

		if (mcu_fw_buf != NULL) {
			pr_info("Freeing mcu_fw_buf");
			kfree(mcu_fw_buf);
		}
		is_fw_loaded = 0;
		pr_info("Firmware Released by the Driver");
	}

	if (sysfs_probe_dir) {
		pr_info("Removing Probe Sysfs files");
		kobject_put(kobj_ref_probe);
		for (loop = 0; loop < total_num_cam; loop++)
			kobject_put(kobj_ref_probe_cam[loop]);
		sysfs_probe_dir = 0;
	}

	if (priv->cam_id == Ox08B40)
		remove_value_read_sysfs_entries();

	/* Free up memory */
	for (loop = 0;
		loop < priv->mcu_ctrl_info->ctrl_ui_data.ctrl_menu_info.num_menu_elem
			; loop++) {
		FREE_SAFE(&client->dev,
			priv->mcu_ctrl_info->ctrl_ui_data.ctrl_menu_info.menu[loop]);
	}

	FREE_SAFE(&client->dev,
		priv->mcu_ctrl_info->ctrl_ui_data.ctrl_menu_info.menu);

	FREE_SAFE(&client->dev, priv->mcu_ctrl_info);

	for (loop = 0; loop < s_data->numfmts; loop++) {
		FREE_SAFE(&client->dev,
			(void *)priv->mcu_cam_frmfmt[loop].framerates);
	}

	FREE_SAFE(&client->dev, priv->mcu_cam_frmfmt);

	FREE_SAFE(&client->dev, priv->ctrldb);
	FREE_SAFE(&client->dev, priv->streamdb);

	FREE_SAFE(&client->dev, priv->stream_info);
	FREE_SAFE(&client->dev, fw_version);
	FREE_SAFE(&client->dev, priv->pdata);
	FREE_SAFE(&client->dev, priv->s_data);
	FREE_SAFE(&client->dev, priv);


	dev_info(&client->dev, "rmmod success for YUV GMSL Cameras\n");
	return errcode;
}


static const struct i2c_device_id cam_id[] = {
	{"ox08b40", Ox08B40},
	{} /* Sentinal entry */
};
MODULE_DEVICE_TABLE(i2c, cam_id);

static struct i2c_driver cam_i2c_driver = {
	.driver = {
		   .name = "ecam_yuv",
		   .owner = THIS_MODULE,
		   .of_match_table = of_match_ptr(cam_of_match),
	},
	.probe = ecam_probe,
	.remove = ecam_remove,
	.id_table = cam_id,
};
module_i2c_driver(cam_i2c_driver);

/* Meta Information */
MODULE_AUTHOR("e-con Systems");
MODULE_DESCRIPTION("e-con's Device Driver for STURDeCAM88_CUOAGX");
MODULE_VERSION("2.4");
MODULE_INFO(cam_fw_version, "12OX08B40XXXX041202147f05XXXXXXX");
MODULE_LICENSE("GPL v2");
