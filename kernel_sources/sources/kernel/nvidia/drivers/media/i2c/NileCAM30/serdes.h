#ifndef __NILECAM30_SERDES_H__
#define __NILECAM30_SERDES_H__

#define SER_ADDR1 0x40
#define DES_ADDR1 0x48

#define SER_ADDR2 0x41
#define SER_ADDR3 0x50

int serdes_ncam30_write_i2c(struct i2c_client *client,u16 sladdr,u8* val,u32 count);
int serdes_ncam30_read_i2c(struct i2c_client *client,u16 sladdr,u8* val,u32 count);

s32 serdes_ncam30_read_8b_reg(struct i2c_client *client,u16 sladdr,u8 reg, u8* val);
s32 serdes_ncam30_write_8b_reg(struct i2c_client *client,u16 sladdr,u8 reg,u8 val);
s32 serdes_ncam30_read_16b_reg(struct i2c_client *client,u16 sladdr,u16 reg,u8* val);
s32 serdes_ncam30_write_16b_reg(struct i2c_client *client,u16 sladdr,u16 reg,u8 val);

#endif
