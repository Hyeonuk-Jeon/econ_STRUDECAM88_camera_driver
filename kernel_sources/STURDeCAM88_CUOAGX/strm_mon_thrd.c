#include "strm_mon_thrd.h"

int mipi_sensor_frame_count_read(struct econ_stream_monitor *err_cam)
{
	uint16_t reg_addr = 0;
	uint8_t reg_len = 0, frm_chk_cnt = 0, is_frm_cnt_increasing = 0;
	int ret_val = 0, prev_frm_cnt = 0;

	if (err_cam->cam->cam_id == Ox08B40)
		reg_addr = OX08B40_SENSOR_FRM_CNT_REG;

	reg_len = TWO_BYTE;

	for (frm_chk_cnt = 0; frm_chk_cnt < 5 ; frm_chk_cnt++) {
		ret_val = ox08b40_sensor_read(err_cam->client, reg_addr, reg_len);
		if (ret_val < 0)
			pr_err("In %s sensor_write_failed\n", __func__);

		dev_dbg(&err_cam->client->dev, "Sensor frame count = %d\n", ret_val);
		pr_info("Sensor frame count = %d\n", ret_val);

		if (frm_chk_cnt == 0) {
			prev_frm_cnt = ret_val;
			continue;
		}
		if (prev_frm_cnt != ret_val)
			is_frm_cnt_increasing++;
		else
			dev_dbg(&err_cam->client->dev, "Retrying Bcaz Sensor Frame count is not increasing\n");
		msleep(20);
	}
	if (is_frm_cnt_increasing > 3) {
		pr_info("Sensor Outputs the Frames Properly\n");
		return is_frm_cnt_increasing;
	}
	pr_err("Sensor frame count read Failed\n");
	return -EIO;
}

int get_sensor_frame_count(struct econ_stream_monitor *err_cam)
{
	uint8_t retry = 0;

	while (retry++ < 5) {
		if (mipi_sensor_frame_count_read(err_cam) < 0) {
			msleep(50);
			continue;
		}
		break;
	}
	if (retry > 5) {
		pr_info("Error in reading Sensor Frame Count");
		return -EIO;
	}

	return 0;
}

int get_mipi_sensor_standby_state(struct econ_stream_monitor *err_cam)
{
	int ret_val = 0;
	uint16_t reg_addr = 0;
	uint8_t reg_len = 0;

	if (err_cam->cam->cam_id == Ox08B40)
		reg_addr = OX08B40_SENSOR_STANDBY_REG;

	reg_len = TWO_BYTE;
	if (err_cam->cam->cam_id == Ox08B40) {
		ret_val = ox08b40_sensor_read(err_cam->client, reg_addr, reg_len);
		if (ret_val < 0) {
			pr_err("In %s sensor_write_failed\n", __func__);
			return -EIO;
		}
	} else {
		if (ret_val < 0) {
			pr_err("In %s sensor_write_failed\n", __func__);
			return -EIO;
		}
	}


	if (((ret_val & 0x01) == OX08B40_SENSOR_STREAMING_STATE) & (err_cam->cam->cam_id == Ox08B40))
		pr_info("Sensor is in Streaming mode\n");
	else
		pr_info("sensor is in standby mode\n");
	return 0;

}

int isp_frame_count_read(struct econ_stream_monitor *err_cam)
{
	uint16_t reg_addr = 0;
	uint8_t reg_len = 0, frm_chk_cnt = 0;
	int ret_val = 0, prev_frm_cnt = 0;

	if (err_cam->cam->cam_id == Ox08B40)
		reg_addr = OX08B40_ISP_FRM_CNT_REG;

	reg_len = TWO_BYTE;


	while (frm_chk_cnt++ < 10) {
		ret_val = sensor_isp_read(err_cam->client, reg_addr, reg_len);
		if (ret_val < ERRCODE_SUCCESS) {
			pr_info("In %s sensor_write_failed\n", __func__);
			continue;
		}
		msleep(100);

		dev_dbg(&err_cam->client->dev, "ISP frame count = %d\n", ret_val);
		pr_info("ISP frame count = %d\n", ret_val);

		if (!prev_frm_cnt) {
			prev_frm_cnt = ret_val;
			continue;
		}

		if (prev_frm_cnt != ret_val) {
			pr_info("ISP Outputs the Frames Properly\n");
			return ERRCODE_SUCCESS;
		}
		continue;
	}
	pr_info("ISP frame count read Failed\n");
	return -EIO;
}

int get_isp_sensor_standby_state(struct econ_stream_monitor *err_cam)
{
	int ret_val = 0;
	uint16_t reg_addr = 0;
	uint8_t reg_len = 0, retry = 0;

	if (err_cam->cam->cam_id == Ox08B40)
		reg_addr = OX08B40_ISP_STANDBY_REG;

	pr_warn("cam_id = %d,reg_addr = 0x%04x\n", err_cam->cam->cam_id, reg_addr);
	reg_len = TWO_BYTE;

	while (retry++ < 5) {
		ret_val = sensor_isp_read(err_cam->client, reg_addr, reg_len);
		if (ret_val < ERRCODE_SUCCESS) {
			pr_info("ISP state read fail--retry[%d]: %x\n", retry, ret_val);
			msleep(50);
			continue;
		}

		if ((ret_val & OX08B40_ISP_STREAMING_STATE) & (err_cam->cam->cam_id == Ox08B40)) {
			pr_info("ISP is in Streaming mode\n");
			return ERRCODE_SUCCESS;
		} else if (ret_val & ISP_STREAMING_STATE) {
			pr_info("ISP is in Streaming mode\n");
			return ERRCODE_SUCCESS;
		}
		pr_info("ISP is in standby mode: %x\n", ret_val);
		break;
	}
	if (retry >= 5)
		return -EIO;

	if (err_cam->cam->cam_id == Ox08B40) {
		retry = 0;
		while (retry++ < 5) {
			if (get_mipi_sensor_standby_state(err_cam) < 0) {
				pr_info("sensor sTATE read fail--retry[%d]: %x\n", retry, ret_val);
				msleep(50);
				continue;
			}
			break;
		}
	}
	if (retry >= 5)
		return -EIO;

	return 0;

}

int get_isp_sensor_chip_id(struct econ_stream_monitor *err_cam)
{
	int ret_val = 0;
	uint16_t reg_addr = 0;
	uint8_t reg_len = 0;

	int retry = 0;

	if (err_cam->cam->cam_id == Ox08B40)
		reg_addr = OX08b40_ISP_CHIP_ID_REG;

	reg_len = TWO_BYTE;

	while (retry++ < 5) {
		ret_val = sensor_isp_read(err_cam->client, reg_addr, reg_len);
		if (ret_val > 0) {
			dev_info(&err_cam->client->dev, "ISP Chip ID = 0x%02x\n", ret_val);
			break;
		}
		continue;
		msleep(100);
	}

	/*NEED TO CHECK*/
	/*Sensor ID read*/
	reg_addr = OX08b40_SENSOR_CHIP_ID_REG;
	reg_len = 3;
	if (err_cam->cam->cam_id == Ox08B40) {
		pr_info("SENSOR ID READ for Ox08B40 sensor\n");
		retry = 0;
		while (retry++ < 5) {
			ret_val = ox08b40_sensor_read(err_cam->client, reg_addr, reg_len);
			if (ret_val < 0) {
				pr_err("In %s sensor_write_failed\n", __func__);
				msleep(50);
				continue;
			}
			pr_info("Sensor Chip ID = 0x%x\n", ret_val);
			break;
		}

	}
	if (retry > 5) {
		pr_info("ISP/Sensor Chip is not detected =0x%2x\n", ret_val);
		return -EIO;
	}

	return ERRCODE_SUCCESS;
}

int isp_sensor_err_handle(struct econ_stream_monitor *err_cam, uint8_t err_handle)
{
	int ret_val = 0;

	msleep(20);
	if (get_isp_sensor_chip_id(err_cam) < ERRCODE_SUCCESS) {
		pr_info("Error in ISP/Sensor Chip ID!\nso skipping frame count check\n");
		return -EIO;
	}

	msleep(20);
	if (get_isp_sensor_standby_state(err_cam) < ERRCODE_SUCCESS) {
		pr_info("ISP/Sensor is in standby mode\nso skipping frame count check\n");
		return -EIO;
	}
	if (err_handle) {
		msleep(20);
		ret_val = isp_frame_count_read(err_cam);
		if (ret_val < ERRCODE_SUCCESS) {
			pr_info("Error in ISP, Need to reset\n");
			return -EIO;
		}
		msleep(20);
		if (err_cam->cam->cam_id == Ox08B40) {
			/* Check the Sensor Frame count */
			if (get_sensor_frame_count(err_cam) < 0)
				return -EIO;
		}
	}

	return ERRCODE_SUCCESS;
}

int fsync_set(struct econ_stream_monitor *err_cam)
{
	uint8_t num = 0;

	num = err_cam->cam->cam_num;

	if ((mcu_set_ctrl(err_cam->client, V4L2_CID_FRAME_SYNC,
					CTRL_STANDARD, curr_fsync[num])) < ERRCODE_SUCCESS){
		pr_info("\nError in control configure - sync\n");
		return -EIO;
	}
	return 0;
}
int mcu_err_handle(struct econ_stream_monitor *err_cam,
		uint8_t err_handle)
{
	int err = 0, ret = 0, loop = 0;
	uint16_t sensor_id = 0;
	uint8_t retry = 0, num = 0;
	unsigned char fw_version[32] = {0}, txt_fw_version[32] = {0};

	if (err_handle) {
		if (serdes_write_16b_reg(err_cam->client, err_cam->cam->ser_reassigned_addr,
			err_cam->cam->boot_reg, 0x40) < 0) {
			dev_err(&err_cam->client->dev, "%s(%d): Failed\n",
				__func__, __LINE__);
			return -EIO;
		}

		/* MCU Reset */
		while (retry++ < 10 && err_handle) {
			pr_info("Performing MCU Reset");
			if (toggle_mcu_reset_pin(err_cam->client,
				     err_cam->cam) <  ERRCODE_SUCCESS) {
				msleep(100);
				continue;
			}
			break;
		}
		while (retry >= 10)
			return -EIO;

		/*Delay After ISP reset*/
		msleep(250);
	}


	retry = 0;
	while (retry++ < 5) {
		ret = mcu_get_fw_version(err_cam->client, fw_version, txt_fw_version);
		msleep(100);
		if (ret == 0)
			break;
	}
	if (ret != ERRCODE_SUCCESS) {
		pr_info("MCU Chip Not detected\n");
		return -EIO;
	}
	pr_info("Detected MCU and Current FW Version - (%.32s)",
						fw_version);
	msleep(20);

	if (err_handle) {
		pr_info("Going to get Sensor ID\n");
		if (mcu_get_sensor_id(err_cam->client, &sensor_id) < 0) {
			dev_err(&err_cam->client->dev,
				"Unable to get MCU Sensor ID\n");
			if (!err_handle)
				return -EIO;
		}
		dev_info(&err_cam->client->dev, "ISP ID = 0x%x\n", sensor_id);
	}
	if (!err_handle)
		return 0;



	if ((cam_get_serial_number(err_cam->client)) != ERRCODE_SUCCESS) {
		dev_err(&err_cam->client->dev, "%s(%d):Camera serial number read failed\n",
				__func__, __LINE__);
	}


	pr_info("Re-Initializing the MCU\n");
	retry = 10;
	while (retry-- > 0) {
		if (mcu_isp_init(err_cam->client) < 0) {
			dev_err(&err_cam->client->dev,
				  "Unable to INIT ISP, retry = %d\n", retry);
			continue;
		} else {
			break;
		}
	}

	/* Stream Config */
	err = gen_mcu_stream_config(err_cam->client, err_cam->cam);
	if (err < ERRCODE_SUCCESS) {
		pr_info("\nError in stream configure\n");
		return -EIO;
	}
	pr_info("gen_mcu_stream_config success\n");

	num = err_cam->cam->cam_num;
	if (err_cam->cam->cam_id == Ox08B40) {
#ifdef DEBUG
		pr_warn("available_ctrls_count = %d\n", available_ctrls_count);
#endif
		for (loop = 0; loop < available_ctrls_count; loop++) {
#ifdef DEBUG
			pr_info("cam_set_num: %d, value: %d\n", num, available_ctrls[loop].curr_array[num]);
#endif
			ret = mcu_set_ctrl(err_cam->client, available_ctrls[loop].ctrl_id, CTRL_STANDARD, available_ctrls[loop].curr_array[num]);
			if (ret < ERRCODE_SUCCESS) {
				pr_info("\nError in control configure - %s\n", available_ctrls[loop].name);
				return -EIO;
			}
		}
	}
	return 0;
}

int des_gmsl_link_lock_status(struct econ_stream_monitor *err_cam)
{
	uint8_t retry = 0;
	uint8_t ret_val = 0;
	uint8_t reg_val = 0;

	while (retry++ < 5) {
		/*****Detecting the Availability of DeSerializer*****/
		dev_info(&err_cam->client->dev, "\n*****Detecting the Availability of DeSerializer*****\n");

		if ((serdes_read_16b_reg(err_cam->client, err_cam->cam->dser_addr,
						DEV_ADDR_REG, &reg_val)) < 0) {
			dev_err(&err_cam->client->dev,
					"%s(%d):Deserializer not detected Check it's connection\n",
					__func__, __LINE__);
			continue;
		}
		dev_info(&err_cam->client->dev, "Deserializer Chip is present @addr = %02x\n", reg_val >> 1);
		err_cam->ecam_state->des_slave_addr = reg_val >> 1;

		if (err_cam->cam->phy == PHY_A) {
			ret_val = ser1_detect(err_cam);
			if (ret_val < 0) {
				dev_err(&err_cam->client->dev,
					"%s(%d): Failed\n",
					__func__, __LINE__);
				pr_err("Could not communicate with Serializer 1...\n Check the connection\n");
				return -ENODEV;
			}
		} else {
			ret_val = ser2_detect(err_cam);
			if (ret_val < 0) {
				dev_err(&err_cam->client->dev,
					"%s(%d): Failed\n",
					__func__, __LINE__);
				pr_err("Could not communicate with Serializer 2...\n Check the connection\n");
				return -ENODEV;
			}
		}
		break;
	}
	return ERRCODE_SUCCESS;
}

int re_write_i2c_trans_settings(struct econ_stream_monitor *err_cam,
		SERDES_PARSE *ser_conf, u32 reg_cnt)
{
	if (serdes_parse_regdata(err_cam->client, ser_conf,
				reg_cnt,
				err_cam->cam->ser_reassigned_addr) < 0) {
		dev_err(&err_cam->client->dev,
				"%s: Failed to configure SIOA SerializerI2C translation\n",
				__func__);
		return -EIO;
	} else
		return 0;
}

int i2c_trans_reg_read(struct econ_stream_monitor *err_cam,
		uint16_t reg_addr)
{
	uint8_t reg_val = 0;

	if ((serdes_read_16b_reg(err_cam->client, err_cam->cam->ser_reassigned_addr, reg_addr,
	    &reg_val)) < 0) {
		dev_err(&err_cam->client->dev, "%s(%d): Failed\n",
				__func__, __LINE__);
	}
	pr_info("reg val = 0x%2x", reg_val >> 1);
	if (reg_val >> 1 == SER1_SRC_MCU_ADDR) {
		pr_info("Serializer 1 I2C translation detected\n");
		return 0;
	} else if (reg_val >> 1 == SER2_SRC_MCU_ADDR) {
		pr_info("Serializer 2 I2C translation detected\n");
		return 0;
	}
	pr_info("I2C translation not detected re-write the settings\n");

return -EIO;
}

int check_ser_i2c_translation(struct econ_stream_monitor *err_cam)
{
	int ret_val = 0;
	uint8_t retry = 0;

	while (retry++ < 5) {
		if (err_cam->cam->phy == PHY_A) {
			ret_val = i2c_trans_reg_read(err_cam, SER_I2C_SRC_A_REG);
			if (ret_val == 0)
				return 0;

			pr_info("Writing I2C translation settings for SER1\n");
			ret_val = re_write_i2c_trans_settings(
				err_cam, SER1_I2C_CONF,
				ser1_i2c_array_size);
			err_cam->ecam_state->is_i2c_trans = 1;
		}
		if (err_cam->cam->phy == PHY_B) {
			ret_val = i2c_trans_reg_read(err_cam, SER_I2C_SRC_A_REG);
			if (ret_val == 0)
				return 0;
			pr_info("Writing I2C translation settings for SER2\n");

			ret_val = re_write_i2c_trans_settings(
				err_cam, SER2_I2C_CONF,
				ser2_i2c_array_size);
			err_cam->ecam_state->is_i2c_trans = 1;
		}
	}
	return ret_val;
}

int ser1_i2c_reassignment(struct econ_stream_monitor *err_cam)
{
	uint8_t retry = 0, reg_val =  0, ser_addr = 0;

	ser_addr = SER1_ADDR;

	while (retry++ < 5) {
		if ((serdes_write_16b_reg(err_cam->client,
						0x0040, DEV_ADDR_REG,
						ser_addr << 1)) < 0) {
			dev_err(&err_cam->client->dev,
					"%s(%d): Failed\n",
					__func__, __LINE__);
			continue;
		} else {
			if ((serdes_read_16b_reg(err_cam->client,
							ser_addr, DEV_ADDR_REG,
							&reg_val)) < 0) {
				dev_err(&err_cam->client->dev,
						"%s(%d): Failed\n",
						__func__, __LINE__);
				continue;
			}
		}
		if (reg_val >> 1 == ser_addr) {
			dev_info(&err_cam->client->dev,
					"SER1 I2C re-assigned with addr = 0x%02x\n"
					, reg_val >> 1);
			return ser_addr;
		}
	}
	dev_info(&err_cam->client->dev, "I2C Re-assignment Failed\n");
	return -EIO;
}

int ser2_i2c_reassignment(struct econ_stream_monitor *err_cam)
{
	uint8_t retry = 0, reg_val =  0, ser_addr = 0;

	ser_addr = SER2_ADDR;

	while (retry++ < 5) {
		if ((serdes_write_16b_reg(err_cam->client,
						0x0040, DEV_ADDR_REG,
						ser_addr << 1)) < 0) {
			dev_err(&err_cam->client->dev,
					"%s(%d): Failed\n",
					__func__, __LINE__);
			continue;
		} else {
			if ((serdes_read_16b_reg(err_cam->client,
							ser_addr, DEV_ADDR_REG,
							&reg_val)) < 0) {
				dev_err(&err_cam->client->dev,
						"%s(%d): Failed\n",
						__func__, __LINE__);
				continue;
			}
		}
		if (reg_val >> 1 == ser_addr) {
			dev_info(&err_cam->client->dev,
					"SER2 I2C re-assigned with addr = 0x%02x\n"
					, reg_val >> 1);
			return ser_addr;
		}
	}
	dev_info(&err_cam->client->dev, "I2C Re-assignment Failed\n");
	return -EIO;
}

int check_ser1_for_ser2_error(struct econ_stream_monitor *err_cam)
{
	uint8_t reg_val = 0, retry = 0, ser_addr = 0;
	int8_t ret_val = 0;

	ser_addr = SER1_ADDR;

	while (retry++ < 5) {
		if ((serdes_write_16b_reg(err_cam->client,
					    err_cam->cam->dser_addr,
						RESET_REG, EN_LINK_A)) < 0) {
			dev_err(&err_cam->client->dev, "%s(%d): Failed\n",
					__func__, __LINE__);
			continue;
		}
		msleep(100);
		if ((serdes_read_16b_reg(err_cam->client,
					  ser_addr, DEV_ADDR_REG, &reg_val)) < 0) {
			dev_err(&err_cam->client->dev, "%s(%d): Failed\n",	__func__, __LINE__);
			goto check_def_addr;
		} else if (reg_val >> 1 == ser_addr) {
			dev_info(&err_cam->client->dev,
					"LINK B serializer i2c re-assigned\n");
			ret_val = reg_val >> 1;
			goto success;
		}
check_def_addr:
		if ((serdes_read_16b_reg(err_cam->client,
					0x40,
					DEV_ADDR_REG, &reg_val)) < 0) {
			dev_err(&err_cam->client->dev,
					"%s(%d): Failed\n",
					__func__, __LINE__);
			continue;
		}
		if (reg_val >> 1 == 0x40) {
			dev_info(&err_cam->client->dev,
					"Reassign the LINK B serializeri2c address\n");
			goto i2c_reassign;
		} else {
			dev_info(&err_cam->client->dev,
					"Serializer not present in LINK B\n");
			goto err;
		}
i2c_reassign:

		ret_val = ser1_i2c_reassignment(err_cam);
	}
success:
	return ret_val;
err:
	return -ENODEV;
}

int check_ser2_for_ser1_error(struct econ_stream_monitor *err_cam)
{
	uint8_t reg_val = 0, retry = 0, ser_addr = 0;
	int8_t ret_val = 0;

	ser_addr = SER2_ADDR;

	while (retry++ < 5) {
		if ((serdes_write_16b_reg(err_cam->client,
					    err_cam->cam->dser_addr,
						RESET_REG, EN_LINK_B)) < 0) {
			dev_err(&err_cam->client->dev, "%s(%d): Failed\n",
					__func__, __LINE__);
			continue;
		}
		msleep(100);
		if ((serdes_read_16b_reg(err_cam->client,
					  ser_addr, DEV_ADDR_REG, &reg_val)) < 0) {
			dev_err(&err_cam->client->dev, "%s(%d): Failed\n",	__func__, __LINE__);
			goto check_def_addr;
		} else if (reg_val >> 1 == ser_addr) {
			dev_info(&err_cam->client->dev,
					"LINK B serializer i2c re-assigned\n");
			ret_val = reg_val >> 1;
			goto success;
		}
check_def_addr:
		if ((serdes_read_16b_reg(err_cam->client,
					 0x40,
					   DEV_ADDR_REG, &reg_val)) < 0) {
			dev_err(&err_cam->client->dev,
					"%s(%d): Failed\n",
					__func__, __LINE__);
			continue;
		}
		if (reg_val >> 1 == 0x40) {
			dev_info(&err_cam->client->dev,
					"Reassign the LINK B serializeri2c address\n");
			goto i2c_reassign;
		} else {
			dev_info(&err_cam->client->dev,
					"Serializer not present in LINK B\n");
			goto err;
		}
i2c_reassign:

		ret_val = ser2_i2c_reassignment(err_cam);
	}
success:
	return ret_val;
err:
	return -ENODEV;
}

int ser1_detect(struct econ_stream_monitor *err_cam)
{
	uint8_t retry = 0, reg_val = 0, ser_addr = 0;
	int8_t ret_val = 0;

	ser_addr = SER1_ADDR;

	/* Enable only Link A in Deserializer */
	while (retry++ < 5) {
		pr_warn("dser_addr = 0x%02x,reset reg = 0x%02x, link A enable = 0x%02x\n", err_cam->cam->dser_addr, RESET_REG, EN_LINK_A);
		if ((serdes_write_16b_reg(err_cam->client, err_cam->cam->dser_addr,
						RESET_REG, EN_LINK_A)) < 0) {
			dev_err(&err_cam->client->dev,
					"%s(%d): Failed\n",
					__func__, __LINE__);
			continue;
		}
		msleep(100);
		if ((serdes_read_16b_reg(err_cam->client, err_cam->cam->ser_reassigned_addr,
						DEV_ADDR_REG, &reg_val)) < 0) {
			dev_err(&err_cam->client->dev,
					"%s(%d): Could not Detect the\n"
					"Serializer Chip @addr = 0x%02x\n",
					__func__, __LINE__, err_cam->cam->ser_reassigned_addr);

			dev_info(&err_cam->client->dev, "Check with default addr\n");

			if ((serdes_read_16b_reg(err_cam->client, 0x40,
						DEV_ADDR_REG, &reg_val)) < 0) {
				dev_err(&err_cam->client->dev,
					"%s(%d): Could not Detect the\n"
					"Serializer Chip @addr = 0x40\n",
					__func__, __LINE__);
				continue;
			}
		}
		if (reg_val >> 1 == ser_addr) {
			pr_info("Serializer Chip is present @addr = %02x\n", reg_val >> 1);
			if (serdes_parse_regdata(err_cam->client,
						SER1_PKT_HEADER_CHANGE,
						ser1_pkt_header_array_size,
						ser_addr) < 0) {
				dev_err(&err_cam->client->dev, "%s: Failed to LINKA Serializer header change I2C translation\n", __func__);
				continue;
			} else {
				dev_info(&err_cam->client->dev, "%s: SUCCESS LINKA Serializer header change I2C translation\n", __func__);
			}
		} else {
			pr_info("I2C Re-assignment required for LINK A Serializer\n");

			ret_val = ser1_i2c_reassignment(err_cam);
			pr_info("ret_val = 0x%02x\n", ret_val);
			if (ret_val == SER1_ADDR) {

				pr_info("Done I2C Re-assignment for LINK A Serializer\n");
			} else {
				pr_err("error %d\n", __LINE__);
				return -EIO;
			}
		}
		ret_val = check_ser2_for_ser1_error(err_cam);
		if (ret_val == SER2_ADDR) {
			strm_mon_ser_status = EN_LINK_AB;
			return ret_val;
		}
		strm_mon_ser_status = EN_LINK_A;
		return 0;
	}
	return -ENODEV;
}

int ser2_detect(struct econ_stream_monitor *err_cam)
{
	uint8_t retry = 0, reg_val = 0, ser_addr = 0;
	int8_t ret_val = 0;

	ser_addr = SER2_ADDR;

	/* Enable only Link B in Deserializer */
	while (retry++ < 5) {
		if ((serdes_write_16b_reg(err_cam->client, err_cam->cam->dser_addr,
						RESET_REG, EN_LINK_B)) < 0) {
			dev_err(&err_cam->client->dev,
					"%s(%d): Failed\n",
					__func__, __LINE__);
			continue;
		}
		msleep(100);
		if ((serdes_read_16b_reg(err_cam->client, err_cam->cam->ser_reassigned_addr,
						DEV_ADDR_REG, &reg_val)) < 0) {
			dev_err(&err_cam->client->dev,
					"%s(%d): Could not Detect the\n"
					"Serializer Chip @addr = 0x%02x\n",
					__func__, __LINE__, err_cam->cam->ser_reassigned_addr);

			dev_info(&err_cam->client->dev, "Check with default addr\n");

			if ((serdes_read_16b_reg(err_cam->client, 0x40,
						DEV_ADDR_REG, &reg_val)) < 0) {
				dev_err(&err_cam->client->dev,
					"%s(%d): Could not Detect the\n"
					"Serializer Chip @addr = 0x40\n",
					__func__, __LINE__);
			continue;
			}
		}
		if (reg_val >> 1 == ser_addr) {
			pr_info("Serializer Chip is present @addr = %02x\n", reg_val >> 1);
			if (serdes_parse_regdata(err_cam->client,
						SER2_PKT_HEADER_CHANGE,
						ser2_pkt_header_array_size,
						ser_addr) < 0) {
				dev_err(&err_cam->client->dev, "%s: Failed to LINKB Serializer header change I2C translation\n", __func__);
				continue;
			} else {
				dev_info(&err_cam->client->dev, "%s: SUCCESS LINKB Serializer header change I2C translation\n", __func__);
			}
		} else {
			pr_info("I2C Re-assignment required for LINK B Serializer\n");

			ret_val = ser2_i2c_reassignment(err_cam);
			if (ret_val == SER2_ADDR)
				pr_info("Done I2C Re-assignment for LINK B Serializer\n");
			else
				return -EIO;
		}

		ret_val = check_ser1_for_ser2_error(err_cam);
		if (ret_val == SER1_ADDR) {
			strm_mon_ser_status = EN_LINK_AB;
			return ret_val;
		}
		strm_mon_ser_status = EN_LINK_B;
		return 0;
	}
	return -ENODEV;
}

int ser_video_status(struct econ_stream_monitor *err_cam)
{

	uint8_t retry = 0, reg_val = 0;
	int ret_val = 0;

	/*****Detect the Presence of LINK A Serializer*****/
	if (err_cam->cam->phy == PHY_A) {
		ret_val = ser1_detect(err_cam);
		if (ret_val < 0) {

			dev_err(&err_cam->client->dev,
					"%s(%d): Failed\n",
					__func__, __LINE__);
			return -ENODEV;

		} else {
			if ((serdes_write_16b_reg(err_cam->client, err_cam->cam->dser_addr,
							RESET_REG, strm_mon_ser_status)) < 0) {
				dev_err(&err_cam->client->dev,
						"%s(%d): Failed\n",
						__func__, __LINE__);
			}
			msleep(100);
			goto pclkdet;
		}
	}

	/* Check I2C re-assignment and Presence of LINK B Serializer */
	if (err_cam->cam->phy == PHY_B) {
		ret_val = ser2_detect(err_cam);
		if (ret_val < 0) {

			dev_err(&err_cam->client->dev,
					"%s(%d): Failed\n",
					__func__, __LINE__);
			return -ENODEV;

		} else {
			if ((serdes_write_16b_reg(err_cam->client,
							err_cam->cam->dser_addr,
							RESET_REG, strm_mon_ser_status)) < 0) {
				dev_err(&err_cam->client->dev,
						"%s(%d): Failed\n",
						__func__, __LINE__);
			}
			msleep(100);
			goto pclkdet;
		}
	}

pclkdet:
	pr_info("\n*****Checking Serializer's PCLKDET*****\n");

	/*****Checking Serializer's PCLKDET*****
	 * Serializer Video Pipe Using is Z
	 * Check the sensor and re-configure If this bit is not set
	 */
	while (retry++ < 5) {
		if ((serdes_read_16b_reg(err_cam->client, err_cam->cam->ser_reassigned_addr,
						SER_Z_PIPE_PCLKDET_REG, &reg_val)) < 0) {
			dev_err(&err_cam->client->dev,
					"%s:Error in Serializer read\n", __func__);
		}
		if (reg_val & 0x80) {
			dev_info(&err_cam->client->dev,
					" Detected PCLKDET in Serializer\n");
			err_cam->ecam_state->ser_pclkdet = 1;
			return 0;

		} else {
			continue;
		}
	}
	dev_err(&err_cam->client->dev,
			"Pixclock not detected in Serializer\n Need to reconfigure the MCU/Sensor...!\n");
	return -EREMOTEIO;

}

int dser_video_status(struct econ_stream_monitor *err_cam)
{

	uint8_t retry = 0, reg_val = 0;

	if (err_cam->cam->phy == PHY_A) {
		while (retry++ < 5) {
			dev_info(&err_cam->client->dev,
					"Checking Link A Ser and Dser video lock\n");

			if ((serdes_read_16b_reg(err_cam->client,
						    err_cam->cam->dser_addr,
							DSER_Y_PIPE_REG,
							   &reg_val)) < 0) {
				dev_err(&err_cam->client->dev,
						"%s(%d): Read Failed\n",
						__func__, __LINE__);
				continue;
			}
			if (reg_val & 0x01) {
				dev_info(&err_cam->client->dev,
						"Link A Ser and Dser video channel locked\n");
				return 0;
			}
			dev_err(&err_cam->client->dev,
					"Link A Ser and Dser video channel not locked\n");
			continue;

		}
		return -EIO;
	} else if (err_cam->cam->phy == PHY_B) {
		while (retry++ < 5) {
			dev_info(&err_cam->client->dev,
					"Checking Link B Ser and Dser video lock\n");

			if ((serdes_read_16b_reg(err_cam->client,
						    err_cam->cam->dser_addr,
							DSER_Z_PIPE_REG,
							   &reg_val)) < 0) {
				dev_err(&err_cam->client->dev,
						"%s(%d): Read Failed\n",
						__func__, __LINE__);
				continue;
			}
			if (reg_val & 0x01) {
				dev_info(&err_cam->client->dev,
						"Link B Ser and Dser video channel locked\n");
				return 0;
			}
			dev_err(&err_cam->client->dev,
					"Link B Ser and Dser video channel not locked\n");
			continue;

		}
		return -EIO;
	}
	dev_err(&err_cam->client->dev, "Invalid PHY\n");
	return -EINVAL;

}

int serdes_link_status(struct econ_stream_monitor *err_cam)
{
	int ret_val = 0;

	/* Detect Deserializer chip presence
	 * and GMSL link lock status
	 */
	ret_val = des_gmsl_link_lock_status(err_cam);
	if (ret_val < 0) {
		pr_info("Error in %s(%d)\n", __func__, __LINE__);
		return ret_val;
	}
	/* Detect Serializer chip presence
	 * Check frames from the camera using PCLKDET bit
	 */
	ret_val = ser_video_status(err_cam);
	if (ret_val < 0) {
		pr_info("Error in %s(%d)\n", __func__, __LINE__);
		return ret_val;
	}
	ret_val = dser_video_status(err_cam);
	if (ret_val < 0) {
		pr_info("Error in %s(%d)\n", __func__, __LINE__);
		return ret_val;
	}
	return 0;
}

int serdes_err_handle(struct econ_stream_monitor *err_cam)
{
	int ret_val = 0;
	uint8_t temp = 0, serdes_speed = 0, dser_rev = 0;

	/* Detect Presence of  Deserializer/Serializer Chip
	 *  and Check GMSL link lock and Frames from the Camera
	 */
	ret_val = serdes_link_status(err_cam);
	if (ret_val == -ENODEV) {
		return -ENODEV;
	} else if (ret_val == -EREMOTEIO) {
		pr_info("Re-Configure the Serializer/MCU to recover the stream\n");
		goto serdes_reconf;
	}

serdes_reconf:

	/*
	 *Check and Handle power disconnect to Serializer
	 */
	ret_val = check_ser_i2c_translation(err_cam);
	if (ret_val < 0) {
		pr_info("Serializer I2C translation Failed exiting stream recovery\n");
		return -EIO;
	}
	/* Check Ser,Dser and TB re-configure requirement */
	if (err_cam->ecam_state->is_i2c_trans) {
		pr_info("\n*****Going to re-configure the SerDes*****\n");

	/* Configuring SIOA Serializer */
	if (err_cam->cam->phy == PHY_A) {
		if (serdes_parse_regdata(err_cam->client, SER1_CONF,
					ser1_array_size,
					  err_cam->cam->ser_reassigned_addr) < 0) {
			dev_err(&err_cam->client->dev,
					"%s: Failed to configure SIOA Ser\n",
						__func__);
			return -EIO;
		}
		pr_info("configuring SIOA serializer successful\n");
	}
	if (err_cam->cam->phy == PHY_B) {
		if (serdes_parse_regdata(err_cam->client, SER2_CONF,
					ser2_array_size,
					  err_cam->cam->ser_reassigned_addr) < 0) {
			dev_err(&err_cam->client->dev,
					"%s: Failed to configure SIOA Ser\n",
						__func__);
			return -EIO;
		}
		pr_info("configuring SIOB serializer successful\n");
	}

	/* Configuring Deserializer */
	if (serdes_parse_regdata(err_cam->client, DSER_CONF_8CAM,
			dser8_array_size, err_cam->cam->dser_addr) < 0) {
		dev_err(&err_cam->client->dev,
			"%s: Failed to configure DESER Serializer\n", __func__);
		return -EIO;
	}

	/* Setting MIPI clock based on Deser Rev*/

	serdes_read_16b_reg(err_cam->client,
			err_cam->cam->dser_addr, 0x000E, &temp);

	dser_rev = temp & 0x0F;

	if (dser_rev == 0x0C) {
		dev_info(&err_cam->client->dev, "DSER REV %d setting clock 0x34\n", dser_rev);
		serdes_speed = 0x34;
	} else {
		dev_info(&err_cam->client->dev, "DSER REV %d setting clock 0x37\n", dser_rev);
		serdes_speed = 0x37;
	}

	if ((serdes_write_16b_reg(err_cam->client, err_cam->cam->dser_addr,
						0x0320, serdes_speed)) < 0) {
		dev_err(&err_cam->client->dev, "%s(%d): Failed\n",
					__func__, __LINE__);
		return -EIO;
	}
	if ((serdes_write_16b_reg(err_cam->client, err_cam->cam->dser_addr,
						0x0323, serdes_speed)) < 0) {
		dev_err(&err_cam->client->dev, "%s(%d): Failed\n",
					__func__, __LINE__);
		return -EIO;
	}

	pr_info("configuring Deserializer Successful\n");
	msleep(20);
	}
	/*Enabling LINKS based on Serializer Availablity*/
	dev_err(&err_cam->client->dev, " strm_mon_ser_status=%x\n",
			strm_mon_ser_status);
	if (serdes_write_16b_reg(err_cam->client,
				err_cam->cam->dser_addr, RESET_REG,
						strm_mon_ser_status) < 0) {
		dev_err(&err_cam->client->dev, "%s(%d): Failed\n",
				__func__, __LINE__);
		return -EIO;
	}
	msleep(100);
	pr_info("Ser_Des Re-configuration Successfull\n");
	return 0;

}

int find_err_cam(char *err_cam_name)
{
	uint8_t loop = 0;

	for (loop = 0; loop < MAX_CAMERAS; loop++) {
		if (dev_name_comp[loop] != NULL) {
			if (!(strcmp(err_cam_name,
				dev_name_comp[loop]))) {
				goto cam_find_exit;
			} else {
				continue;
			}
		} else {
			continue;
		}
	}

	pr_info("unable to identify the cam loop =%d\n", loop);
	return -ENODEV;

cam_find_exit:

	pr_info("err_cam_name  >>> %s\n", err_cam_name);
	return loop;

}


int stream_monitor_thread(void *strm_dev)
{
	int8_t ret_val = 0, reset = 0, i = 0;
	uint8_t err_cam = 0;
	uint8_t is_err_handle_in_progress = 0;
	uint8_t is_isp_sensor_err_handle_in_progress = 0;

	for (i = 0; i < 4; i++)
		thread_handle[i] = ENABLE;

	while (1) {
		msleep(1000);
		wait_event_interruptible(econ_err_hand_q,
			((econ_frame_err_track == 1
				|| is_stop_strm_mon_thread)
					&& ecam_status_check));
		{
			if (is_stop_strm_mon_thread) {
				pr_info("Thread returned");
				is_stop_strm_mon_thread = 0;
				return 0;
			}

			/* Getting err_cam number from vi */
			err_cam = find_err_cam(econ_dev_name);
			pr_info("Error Causing Camera is CAM-%d loop(%d)", err_cam, thread_handle[err_cam]);

		if (thread_handle[err_cam] == ENABLE) {
			thread_handle[err_cam] = DISABLE;
			if (!is_isp_sensor_err_handle_in_progress) {
				/*
				 * Checks Link Lock & Video Lock Status
				 * in Deserializer
				 */

				/* Detect and handling of Ser/Deser Errors */
				if (serdes_err_handle(strm_mon[err_cam]) < 0)
					goto reset_flags;
			}

			/*
			 * Detects MCU and Re-init
			 */
			ret_val = mcu_err_handle(strm_mon[err_cam], 1);
			if (ret_val < 0) {
				pr_info("Error in MCU Skipping Error Recovery");
				goto reset_flags;
			}
			is_isp_sensor_err_handle_in_progress++;

			/*
			 * Checks ISP and Sensor's Streaming State
			 * and frame count
			 */
			ret_val = isp_sensor_err_handle(strm_mon[err_cam], 1);
			if (ret_val) {
				pr_info("Error in isp/sensor frame count");
				if (reset) {
					pr_info("Error in ISP/Sensor");
					goto reset_flags;
				} else if (is_isp_sensor_err_handle_in_progress == 3) {
					reset = 1;
					pr_info("Error in ISP/Sensor Recovery");
					goto reset_flags;
				} else
					continue;
			}

			if (strm_mon[err_cam]->cam->cam_id == Ox08B40) {
				/*Trigger Pin mapping*/
				ret_val = fsync_set(strm_mon[err_cam]);
				if (ret_val < ERRCODE_SUCCESS)
					pr_info("\nError in fsync configure\n");
			}


reset_flags:
			pr_info("reset_flags");

			/* Enabling Link Status of the Deserializer */
			pr_info("resetting dser status , dser_ser_status = 0x%02x, dser_addr = 0x%02x\n", strm_mon_ser_status, strm_mon[err_cam]->cam->dser_addr);
			if ((serdes_write_16b_reg(strm_mon[err_cam]->client,
							strm_mon[err_cam]->cam->dser_addr,
							LINK_REG, strm_mon_ser_status)) < 0) {
				dev_err(&strm_mon[err_cam]->client->dev, "%s(%d): Failed\n",
						__func__, __LINE__);
			}
			msleep(100);

			pr_info("Reset both the Pipe");
			if ((serdes_write_16b_reg(strm_mon[err_cam]->client, strm_mon[err_cam]->cam->dser_addr, 0x0002, 0x03)) < 0)
				dev_err(&strm_mon[err_cam]->client->dev, "%s(%d): Failed\n", __func__, __LINE__);
			msleep(20);
			if ((serdes_write_16b_reg(strm_mon[err_cam]->client, strm_mon[err_cam]->cam->dser_addr, 0x0002, 0x33)) < 0)
				dev_err(&strm_mon[err_cam]->client->dev, "%s(%d): Failed\n", __func__, __LINE__);

			reset = 0;
			econ_num_uncorr_err = 0;
			econ_frame_err_track = 0;
			is_err_handle_in_progress = 0;
			is_isp_sensor_err_handle_in_progress = 0;
			thread_handle[err_cam] = ENABLE;
			} else
				break;
		}
	}

	pr_info("Streaming Monitor thread is stopped\n");
	return 0;
}

uint8_t create_stream_monitor_thread(void)
{
	strm_mon_thrd = kthread_create(stream_monitor_thread,
				&strm_mon, "strm_mon_thrd");
	if (strm_mon_thrd == NULL)
		return -ENOMEM;

	wake_up_process(strm_mon_thrd);
	is_stream_monitor_thrd = 1;

	/* Initialize Econ wait queue */
	if (check_queue_init == 0) {
		init_waitqueue_head(&econ_err_hand_q);
		pr_info("Initialized E-con wait queue\n");
		check_queue_init = 1;
	}

	return ERRCODE_SUCCESS;
}

uint8_t stop_stream_monitor_thread(void)
{
	pr_info("Going to Stop the thread in STREAM OFF\n");
	if (is_stream_monitor_thrd == 1) {
		is_stop_strm_mon_thread = 1;
		check_queue_init = 0;
		msleep(20);
		kthread_stop(strm_mon_thrd);
		pr_info("Streaming Monitor Thread Stopped\n");
		is_stream_monitor_thrd = 0;
	};

	return ERRCODE_SUCCESS;
}

int create_sysfs_dir(struct i2c_client *client, struct cam *priv)
{
	uint8_t loop = 0;

	dev_info(&client->dev, "phy-A read [%x], reset-reg:%x, ser-add:%x",
		priv->phy, priv->reset_reg, priv->ser_reassigned_addr);

	if (!is_strm_mon_mem) {
		for (loop = 0; loop < g_max_cam; loop++) {
			/* Allocate Memory for streaming monitor priv structures */
			strm_mon[loop] = kzalloc(sizeof(struct econ_stream_monitor),
								GFP_KERNEL);
			strm_mon[loop]->ecam_state = kzalloc(sizeof(struct econ_cam),
								GFP_KERNEL);
		}
		is_strm_mon_mem = 1;
	}

	pr_info("*************client copy, total_cam:%d,************", total_num_cam);
#ifdef DEBUG
	pr_warn("total_num_cam = %d, g_max_cam = %d, strm_thrd_num = %d\n", total_num_cam, g_max_cam, strm_thrd_num);
#endif
	strm_mon[strm_thrd_num]->client = client;
	strm_mon[strm_thrd_num]->cam = priv;
#ifdef DEBUG
	pr_info("phy read %c, [%x]-reg:%x, ser-add:%x ", strm_mon[strm_thrd_num]->cam->phy,
		priv->phy, strm_mon[strm_thrd_num]->cam->reset_reg, priv->ser_reassigned_addr);
#endif
	return 0;

}

int remove_sysfs_dir(void)
{
	uint8_t loop = 0;

	if (is_strm_mon_mem == 1) {
		pr_info("Freeing streaming monitor thread memory\n");
		for (loop = 0; loop < g_max_cam; loop++) {
			kfree(strm_mon[loop]->ecam_state);
			kfree(strm_mon[loop]);
		}
		is_strm_mon_mem = 0;
		pr_info("Freed streaming monitor thread memory\n");

	}

	return ERRCODE_SUCCESS;
}





static ssize_t  sysfs_ecam_cam_0_imu_value_read(struct kobject *kobj,
				struct kobj_attribute *attr, char *buf);

static ssize_t  sysfs_ecam_cam_1_imu_value_read(struct kobject *kobj,
				struct kobj_attribute *attr, char *buf);

static ssize_t  sysfs_ecam_cam_2_imu_value_read(struct kobject *kobj,
				struct kobj_attribute *attr, char *buf);

static ssize_t  sysfs_ecam_cam_3_imu_value_read(struct kobject *kobj,
				struct kobj_attribute *attr, char *buf);

static ssize_t  sysfs_ecam_cam_0_temp_value_read(struct kobject *kobj,
				struct kobj_attribute *attr, char *buf);

static ssize_t  sysfs_ecam_cam_1_temp_value_read(struct kobject *kobj,
				struct kobj_attribute *attr, char *buf);

static ssize_t  sysfs_ecam_cam_2_temp_value_read(struct kobject *kobj,
				struct kobj_attribute *attr, char *buf);

static ssize_t  sysfs_ecam_cam_3_temp_value_read(struct kobject *kobj,
				struct kobj_attribute *attr, char *buf);







struct kobj_attribute cam0_imu_read_attr = __ATTR(imu_read, 0440,
		sysfs_ecam_cam_0_imu_value_read, NULL);

struct kobj_attribute cam1_imu_read_attr = __ATTR(imu_read, 0440,
		sysfs_ecam_cam_1_imu_value_read, NULL);

struct kobj_attribute cam2_imu_read_attr = __ATTR(imu_read, 0440,
		sysfs_ecam_cam_2_imu_value_read, NULL);

struct kobj_attribute cam3_imu_read_attr = __ATTR(imu_read, 0440,
		sysfs_ecam_cam_3_imu_value_read, NULL);



struct kobj_attribute cam0_temp_read_attr = __ATTR(temp_read, 0440,
		sysfs_ecam_cam_0_temp_value_read, NULL);

struct kobj_attribute cam1_temp_read_attr = __ATTR(temp_read, 0440,
		sysfs_ecam_cam_1_temp_value_read, NULL);

struct kobj_attribute cam2_temp_read_attr = __ATTR(temp_read, 0440,
		sysfs_ecam_cam_2_temp_value_read, NULL);

struct kobj_attribute cam3_temp_read_attr = __ATTR(temp_read, 0440,
		sysfs_ecam_cam_3_temp_value_read, NULL);








static ssize_t sysfs_ecam_cam_0_imu_value_read(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	int ret = 0;

	ret = mcu_get_imu_angle(strm_mon[0]->client);
	if (ret < 0)
		return ret;

	ret = snprintf(buf, PAGE_SIZE, // Use PAGE_SIZE to limit the buffer size
			"accX = %d.%d m/s²\n"
			"accY = %d.%d m/s²\n"
			"accZ = %d.%d m/s²\n"
			"gyroX = %d.%d °/s\n"
			"gyroY = %d.%d °/s\n"
			"gyroZ = %d.%d °/s\n", accX_whole, accX_frac, accY_whole, accY_frac,
				accZ_whole, accZ_frac, gyroX_whole, gyroX_frac, gyroY_whole,
				gyroY_frac, gyroZ_whole, gyroZ_frac);
	return ret;
}

static ssize_t sysfs_ecam_cam_1_imu_value_read(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	int ret = 0;

	ret = mcu_get_imu_angle(strm_mon[1]->client);
	if (ret < 0)
		return ret;

	ret = snprintf(buf, PAGE_SIZE, // Use PAGE_SIZE to limit the buffer size
			"accX = %d.%d m/s²\n"
			"accY = %d.%d m/s²\n"
			"accZ = %d.%d m/s²\n"
			"gyroX = %d.%d °/s\n"
			"gyroY = %d.%d °/s\n"
			"gyroZ = %d.%d °/s\n", accX_whole, accX_frac, accY_whole, accY_frac,
				accZ_whole, accZ_frac, gyroX_whole, gyroX_frac, gyroY_whole,
				gyroY_frac, gyroZ_whole, gyroZ_frac);
	return ret;
}

static ssize_t sysfs_ecam_cam_2_imu_value_read(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	int ret = 0;

	ret = mcu_get_imu_angle(strm_mon[2]->client);
	if (ret < 0)
		return ret;

	ret = snprintf(buf, PAGE_SIZE, // Use PAGE_SIZE to limit the buffer size
			"accX = %d.%d m/s²\n"
			"accY = %d.%d m/s²\n"
			"accZ = %d.%d m/s²\n"
			"gyroX = %d.%d °/s\n"
			"gyroY = %d.%d °/s\n"
			"gyroZ = %d.%d °/s\n", accX_whole, accX_frac, accY_whole, accY_frac,
				accZ_whole, accZ_frac, gyroX_whole, gyroX_frac, gyroY_whole,
				gyroY_frac, gyroZ_whole, gyroZ_frac);
	return ret;
}

static ssize_t sysfs_ecam_cam_3_imu_value_read(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	int ret = 0;

	ret = mcu_get_imu_angle(strm_mon[3]->client);
	if (ret < 0)
		return ret;

	ret = snprintf(buf, PAGE_SIZE, // Use PAGE_SIZE to limit the buffer size
			"accX = %d.%d m/s²\n"
			"accY = %d.%d m/s²\n"
			"accZ = %d.%d m/s²\n"
			"gyroX = %d.%d °/s\n"
			"gyroY = %d.%d °/s\n"
			"gyroZ = %d.%d °/s\n", accX_whole, accX_frac, accY_whole, accY_frac,
				accZ_whole, accZ_frac, gyroX_whole, gyroX_frac, gyroY_whole,
				gyroY_frac, gyroZ_whole, gyroZ_frac);
	return ret;
}

static ssize_t sysfs_ecam_cam_0_temp_value_read(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	int ret = 0;

	ret = mcu_get_isp_sensor_temperature(strm_mon[0]->client);
	if (ret < 0)
		return ret;

	ret = snprintf(buf, PAGE_SIZE, // Use PAGE_SIZE to limit the buffer size
			"isp_temperature = %d.%d°C\n"
			"sensor_temperature = %d.%d°C\n", isp_temperature_integer, isp_temperature_decimal, sensor_temperature_integer, sensor_temperature_decimal);
	return ret;
}

static ssize_t sysfs_ecam_cam_1_temp_value_read(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	int ret = 0;

	ret = mcu_get_isp_sensor_temperature(strm_mon[1]->client);
	if (ret < 0)
		return ret;

	ret = snprintf(buf, PAGE_SIZE, // Use PAGE_SIZE to limit the buffer size
			"isp_temperature = %d.%d°C\n"
			"sensor_temperature = %d.%d°C\n", isp_temperature_integer, isp_temperature_decimal, sensor_temperature_integer, sensor_temperature_decimal);

	return ret;
}

static ssize_t sysfs_ecam_cam_2_temp_value_read(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	int ret = 0;

	ret = mcu_get_isp_sensor_temperature(strm_mon[2]->client);
	if (ret < 0)
		return ret;

	ret = snprintf(buf, PAGE_SIZE, // Use PAGE_SIZE to limit the buffer size
			"isp_temperature = %d.%d°C\n"
			"sensor_temperature = %d.%d°C\n", isp_temperature_integer, isp_temperature_decimal, sensor_temperature_integer, sensor_temperature_decimal);
	return ret;
}

static ssize_t sysfs_ecam_cam_3_temp_value_read(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	int ret = 0;

	ret = mcu_get_isp_sensor_temperature(strm_mon[3]->client);
	if (ret < 0)
		return ret;

	ret = snprintf(buf, PAGE_SIZE, // Use PAGE_SIZE to limit the buffer size
			"isp_temperature = %d.%d°C\n"
			"sensor_temperature = %d.%d°C\n", isp_temperature_integer, isp_temperature_decimal, sensor_temperature_integer, sensor_temperature_decimal);
	return ret;
}




/*
 * Creates camera specific IMU Read sysfs entries
 */
uint8_t create_value_read_sysfs_entries(uint8_t num_cam, uint8_t cam_id)
{
	char buf[10];

	if (!is_value_sysfs_dir) {
		/*Creating a directory in /sys/kernel/ */
		pr_info("********Creating a directory in /sys/kernel/*********");
		kobj_ref_value_read = kobject_create_and_add("ecam_value_read_sysfs", kernel_kobj);

		is_value_sysfs_dir = 1;
	}

	sprintf(buf, "cam_%d", num_cam);
	kobj_ref_value_read_cam[num_cam] = kobject_create_and_add(buf, kobj_ref_value_read);

	if (num_cam == 0) {
		if (sysfs_create_file(kobj_ref_value_read_cam[num_cam], &cam0_imu_read_attr.attr))
			pr_err("Cannot create sysfs file......\n");

		if (sysfs_create_file(kobj_ref_value_read_cam[num_cam], &cam0_temp_read_attr.attr))
			pr_err("Cannot create sysfs file......\n");

	}
	if (num_cam == 1) {
		if (sysfs_create_file(kobj_ref_value_read_cam[num_cam], &cam1_imu_read_attr.attr))
			pr_err("Cannot create sysfs file......\n");

		if (sysfs_create_file(kobj_ref_value_read_cam[num_cam], &cam1_temp_read_attr.attr))
			pr_err("Cannot create sysfs file......\n");

	}
	if (num_cam == 2) {
		if (sysfs_create_file(kobj_ref_value_read_cam[num_cam], &cam2_imu_read_attr.attr))
			pr_err("Cannot create sysfs file......\n");

		if (sysfs_create_file(kobj_ref_value_read_cam[num_cam], &cam2_temp_read_attr.attr))
			pr_err("Cannot create sysfs file......\n");

	}
	if (num_cam == 3) {
		if (sysfs_create_file(kobj_ref_value_read_cam[num_cam], &cam3_imu_read_attr.attr))
			pr_err("Cannot create sysfs file......\n");

		if (sysfs_create_file(kobj_ref_value_read_cam[num_cam], &cam3_temp_read_attr.attr))
			pr_err("Cannot create sysfs file......\n");

	}

	return 0;
}

int remove_value_read_sysfs_entries(void)
{
	uint8_t loop = 0;

	if (is_value_sysfs_dir) {
		pr_info("Removing Probe Sysfs files");
		kobject_put(kobj_ref_value_read);
		for (loop = 0; loop < MAX_CAMERAS; loop++) {
			if (kobj_ref_value_read_cam[loop])
				kobject_put(kobj_ref_value_read_cam[loop]);
		}
		is_value_sysfs_dir = 0;
	}
	return ERRCODE_SUCCESS;
}




