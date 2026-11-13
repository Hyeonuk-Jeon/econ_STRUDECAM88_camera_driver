/*
 * serdes.c - GMSL driver
 * Copyright (c) 2017-2018, e-con Systems.  All rights reserved.
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

#include <linux/types.h>
#include <linux/i2c.h>

typedef struct serdes_parse {
uint16_t reg;
uint8_t val;
} SERDES_PARSE;

int serdes_write_i2c(struct i2c_client *client, u16 sladdr,  u8 *val, u32 count)
{
	int ret;

	struct i2c_msg msg = {
		.addr = sladdr,
		.flags = 0,
		.len = count,
		.buf = val,
	};

	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret < 0) {
		dev_err(&client->dev, "Failed writing register ret = %d!\n",
			ret);
		return ret;
	}
	return 0;
}

int serdes_read_i2c(struct i2c_client *client, u16 sladdr, u8 *val, u32 count)
{
	int ret;
	struct i2c_msg msg = {
		.addr = sladdr,
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

s32 serdes_read_8b_reg(struct i2c_client *client, u16 sladdr, u8 reg, u8 *val)
{
	u8 bcount = 1;
	u8 au8RegBuf[1] = { 0 };
	u8 au8RdVal[1] = { 0 };

	au8RegBuf[0] = reg;

	if (serdes_write_i2c(client, sladdr, au8RegBuf, bcount) < 0) {
		dev_err(&client->dev, "%s:write reg error:reg=0x%x\n", __func__, reg);
		return -EIO;
	}

	if (serdes_read_i2c(client, sladdr, au8RdVal, bcount) < 0) {
		dev_err(&client->dev, "%s:read reg error:reg=0x%x\n", __func__, reg);
		return -EIO;
	}

	*val = au8RdVal[0];

	return 0;
}

s32 serdes_write_8b_reg(struct i2c_client *client, u16 sladdr, u8 reg, u8 val)
{
	u8 bcount = 2;
	u8 au8Buf[2] = { 0 };

	au8Buf[0] = reg;
	au8Buf[1] = val;

	if (serdes_write_i2c(client, sladdr, au8Buf, bcount) < 0) {
		dev_err(&client->dev,
			"%s:write reg error: reg = 0x%x,val = 0x%x\n", __func__,
			reg, val);
		return -EIO;
	}

	return 0;
}
s32 serdes_read_16b_reg(struct i2c_client *client, u16 sladdr, u16 reg, u8 *val)
{
	u8 bcount;
	u8 au8RegBuf[2] = { 0 };
	u8 au8RdVal[1] = { 0 };

	au8RegBuf[0] = reg >> 8;
	au8RegBuf[1] = reg & 0xff;
	bcount = 2;

	if (serdes_write_i2c(client, sladdr, au8RegBuf, bcount) < 0) {
		dev_err(&client->dev, "%s:write reg error:reg=0x%x\n", __func__, reg);
		return -EIO;
	}

	bcount = 1;
	if (serdes_read_i2c(client, sladdr, au8RdVal, bcount) < 0) {
		dev_err(&client->dev, "%s:read reg error:reg=0x%x\n", __func__, reg);
		return -EIO;
	}

	*val = au8RdVal[0];

	return 0;
}

s32 serdes_write_16b_reg(struct i2c_client *client, u16 sladdr, u16 reg, u8 val)
{
	u8 bcount = 3;
	u8 au8Buf[3] = { 0 };

	au8Buf[0] = reg >> 8;
	au8Buf[1] = reg & 0xff;
	au8Buf[2] = val;

	if (serdes_write_i2c(client, sladdr, au8Buf, bcount) < 0) {
		dev_err(&client->dev,
			"%s:write reg error: reg = 0x%x,val = 0x%x\n", __func__,
			reg, val);
		return -EIO;
	}

	return 0;
}

s32 serdes_parse_regdata(struct i2c_client *client,
	SERDES_PARSE *regdata, u32 reg_cnt, u8 serdes_id)
{
	int i = 0;

	for (i = 0; i < reg_cnt; i++) {
		if (regdata[i].reg == 0xFFFF)
			continue;

		if ((serdes_write_16b_reg(client, serdes_id, regdata[i].reg,
			regdata[i].val)) < 0) {
			dev_err(&client->dev, "%s(%d): Failed\n",
					__func__, __LINE__);
			return -EIO;
		}

	}

	return 0;
}
