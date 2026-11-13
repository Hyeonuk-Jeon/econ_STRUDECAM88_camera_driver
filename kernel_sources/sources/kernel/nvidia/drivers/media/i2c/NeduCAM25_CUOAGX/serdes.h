/*
 * Copyright (C) 2018 e-con Systems Pvt Ltd, Inc. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __SERDES_H
#define __SERDES_H

#define SER_ADDR 0x18
#define SENSOR_ADDR 0x42
#define GPIO_EXPANDER 0x20
#define DES_ADDR 0x30

#define UB960_SR_RESET				0x01
#define UB960_SR_REV_MASK			0x03

#define UB960_SR_GPIO_INPUT_CTL			0x18

#define UB960_REG_FS_CTL			0x18
#define UB960_REG_FS_HIGH_TIME_1 		0x19
#define	UB960_REG_FS_HIGH_TIME_0		0x1A
#define	UB960_REG_FS_LOW_TIME_1			0x1B
#define	UB960_REG_FS_LOW_TIME_0			0x1C

#define UB960_SR_CSI_PLL_CTL			0x1F
#define UB960_SR_FWD_CTL1			0x20

#define UB960_SR_CSI_PORT_SEL			0x32
#define UB960_TR_CSI_CTL			0x33

#define UB960_SR_FPD3_PORT_SEL			0x4C

#define UB960_RR_BCC_CONFIG			0x58
#define UB960_RR_BCC_CONFIG_I2C_PASS_THROUGH	BIT(6)

#define UB960_RR_SER_ID				0x5B
#define UB960_RR_SER_ALIAS_ID			0x5C

#define UB960_RR_SLAVE_ID(n)			(0x5D + (n))
#define UB960_RR_SLAVE_ALIAS(n)			(0x65 + (n))

#define UB960_RR_PORT_CONFIG			0x6D
#define UB960_RR_PORT_CONFIG2			0x7C
#define UB960_RR_BC_GPIO_CTL1			0x6F
#define UB960_RR_CSI_VC_MAP			0x72

#define UB960_RR_PORT_ICR_HI			0xD8
#define UB960_RR_PORT_ICR_LO			0xD9

#define UB953_REG_RESET_CTL			0x01
#define UB953_REG_RESET_CTL_DIGITAL_RESET_1	BIT(1)
#define UB953_REG_GENERAL_CFG			0x02
#define UB953_REG_CLKOUT_CTRL0			0x06
#define UB953_REG_CLKOUT_CTRL1			0x07
#define UB953_REG_LOCAL_GPIO_DATA		0x0D
#define UB953_REG_GPIO_INPUT_CTRL		0x0E

#define UB953_REG_REV_MASK_ID			0x50

#define UB953_REG_SCL_HIGH_TIME			0x0B
#define UB953_REG_SCL_LOW_TIME			0x0C

typedef struct fpdlink3_parse{
uint8_t reg;
uint8_t val;
}SERDES_PARSE;

SERDES_PARSE PATTERN_GEN[] = {
	{0xB0, 0x00},
	{0xB1, 0x01},
	{0xB2, 0x01},
	{0xB1, 0x02},
	{0xB2, 0xB3},
	{0xB1, 0x03},
	{0xB2, 0x24},
	{0xB1, 0x04},
	{0xB2, 0x16},
	{0xB1, 0x05},
	{0xB2, 0x80},
	{0xB1, 0x06},
	{0xB2, 0x02},
	{0xB1, 0x07},
	{0xB2, 0xD0},
	{0xB1, 0x08},
	{0xB2, 0x04},
	{0xB1, 0x09},
	{0xB2, 0x38},
	{0xB1, 0x0A},
	{0xB2, 0x04},
	{0xB1, 0x0B},
	{0xB2, 0x65},
	{0xB1, 0x0C},
	{0xB2, 0x0B},
	{0xB1, 0x0D},
	{0xB2, 0x93},
	{0xB1, 0x0E},
	{0xB2, 0x21},
	{0xB1, 0x0F},
	{0xB2, 0x0A},
	{0xB1, 0x10},
	{0xB2, 0x00},
	{0xB1, 0x11},
	{0xB2, 0x00},
	{0xB1, 0x12},
	{0xB2, 0xFF},
	{0xB1, 0x13},
	{0xB2, 0xFF},
	{0xB1, 0x14},
	{0xB2, 0xFF},
	{0xB1, 0x15},
	{0xB2, 0x00},
	{0xB1, 0x16},
	{0xB2, 0x00},
	{0xB1, 0x17},
	{0xB2, 0x0F},
	{0xB1, 0x18},
	{0xB2, 0xF0},
	{0xB1, 0x19},
	{0xB2, 0x00},
	{0xB1, 0x1A},
	{0xB2, 0x00},
	{0xB1, 0x1B},
	{0xB2, 0x3F},
	{0xB1, 0x1C},
	{0xB2, 0xC0},
	{0xB1, 0x1D},
	{0xB2, 0x00},
	{0xB1, 0x1E},
	{0xB2, 0x00},
	{0xB1, 0x1F},
	{0xB2, 0x00},
};

int fpdlink3_write_i2c(struct i2c_client *client, u16 sladdr,  u8 * val, u32 count);
int fpdlink3_read_i2c(struct i2c_client *client, u16 sladdr, u8 * val, u32 count);
s32 fpdlink3_read_8b_reg(struct i2c_client *client, u16 sladdr, u8 reg, u8 * val);
s32 fpdlink3_write_8b_reg(struct i2c_client *client, u16 sladdr, u8 reg, u8 val);
s32 fpdlink3_read_16b_reg(struct i2c_client *client, u16 sladdr, u16 reg, u8 * val);
s32 fpdlink3_write_16b_reg(struct i2c_client *client, u16 sladdr, u16 reg, u8 val);
s32 fpdlink3_update_8b_reg(struct i2c_client *client, u16 sladdr, u8 reg, u8 mask, u8 val);
//static int config_regulator(struct i2c_client *client,struct cam *priv, u8 val);
//static int toggle_reset(struct i2c_client *client,struct cam *priv, u8 val);
//static int toggle_boot(struct i2c_client *client,struct cam *priv, u8 val);
#endif /* __SERDES_H */
