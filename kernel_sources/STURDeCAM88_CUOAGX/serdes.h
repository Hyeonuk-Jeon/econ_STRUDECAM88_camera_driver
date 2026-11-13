#ifndef __SERDES_H
#define __SERDES_H

typedef struct serdes_parse {
uint16_t reg;
uint8_t val;
} SERDES_PARSE;

#define PHY_A 'A'
#define PHY_B 'B'
#define PHY_C 'C'
#define PHY_D 'D'

#define SER1_ADDR 0x40
#define SER2_ADDR 0x41
#define DES_ADDR 0x48
#define RESET_REG	0x0010

#define DEV_ADDR_REG			0x0000
#define SER_X_PIPE_PCLKDET_REG		0x0102
#define SER_Z_PIPE_PCLKDET_REG		0x0112
#define DSER_Y_PIPE_REG 		0x01FC
#define DSER_Z_PIPE_REG 		0x021C
#define SER1_SRC_MCU_ADDR		0x43
#define SER2_SRC_MCU_ADDR		0x44
#define SER3_SRC_MCU_ADDR		0x45
#define SER4_SRC_MCU_ADDR		0x46
#define SER_I2C_SRC_A_REG		0x0043
#define EN_LINK_NONE	0x20
#define EN_LINK_A	0x21
#define EN_LINK_B	0x22
#define EN_LINK_AB	0x23
#define LINK_REG	0x0010

#define TRIG_REG 0x0316

/* Serdes function declerations */
s32 serdes_write_16b_reg(struct i2c_client *client, u16 sladdr, u16 reg, u8 val);
s32 serdes_read_16b_reg(struct i2c_client *client, u16 sladdr, u16 reg, u8 *val);
s32 serdes_write_8b_reg(struct i2c_client *client, u16 sladdr, u8 reg, u8 val);
s32 serdes_read_8b_reg(struct i2c_client *client, u16 sladdr, u8 reg, u8 *val);
int serdes_read_i2c(struct i2c_client *client, u16 sladdr, u8 *val, u32 count);
int serdes_write_i2c(struct i2c_client *client, u16 sladdr,  u8 *val, u32 count);
s32 serdes_parse_regdata(struct i2c_client *client,
	SERDES_PARSE *regdata, u32 reg_cnt, u8 serdes_id);
int serdes_configuration(struct cam *priv, struct i2c_client *client);

#endif /* __SERDES_H */
