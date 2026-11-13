
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/i2c.h>
#include <linux/delay.h>

#include <linux/of_irq.h>
#include <linux/irqdomain.h>
#include <linux/irq.h>
#include <linux/interrupt.h>


#define CS4223_LINE_TO_HOST_OFFSET 0x0800

#define CS4223_NUM_PORTS 2

#define CS4223_GLOBAL_BROADCAST 	0x0017
#define CS4223_GLOBAL_MDIO_CFG 		0x000f

#define CS4223_GLOBAL_INT		0x0026
#define CS4223_GLOBAL_INTE		0x0027
#define CS4223_GLOBAL_PIN_STATUS 	0x0016
#define CS4223_GLOBAL_DATAPATH_INT 	0x0028
#define CS4223_GLOBAL_DATAPATH_INTE 	0x0029


#define CS4223_GPIO_INTERR		0x0100
#define CS4223_GPIO_INTERR_OUTPUT_CFG	0x0101

#define CS4223_EFUSE_GENERAL_STATUS 0x0181


#define CS4223_GLOBAL_MDIO_CFG_PREF 0x78 //prefered value

#define CS4223_PP_LINE_LINEMISC_SOFT_RESET(x) (0x1000 + (x*0x1000))
#define CS4223_PP_LINE_LINEMISC_MPIF_RESET_DOTREG(x) (0x1001 + (x*0x1000))

#define CS4223_PP_LINE_LINEMISC_OVERRIDE_EN(x) (0x1010 + (x*0x1000))
#define CS4223_PP_LINE_LINEMISC_OVERRIDE_EN_DFT 0

#define CS4223_PP_HOST_HOSTMISC_SOFT_RESET(x) (0x1800 + (x*0x1000))
#define CS4223_PP_HOST_HOSTMISC_MPIF_RESET_DOTREG(x) (0x1801 + (x*0x1000)) 

#define CS4223_PP_LINE_LINEMISC_GIGEPCS_SOFT_RESET(x) (0x1002 + (x*0x1000))

#define CS4223_PP_LINE_LINEMISC_CLKEN(x) (0x1004 + (x*0x1000))
#define CS4223_PP_LINE_LINEMISC_MSEQCLKCTRL(x) (0x1007 + (x*0x1000))

#define CS4223_PP_LINE_LINEMISC_AN_MODE_SEL(x) (0x100d + (x*0x1000)) 

#define CS4223_PP_LINE_LINEMISC_INT(x)	(0x1011 + (x*0x1000))
#define CS4223_PP_LINE_LINEMISC_INTEN(x) (0x1012 + (x*0x1000))

#define CS4223_PP_LINE_SDS_COMMON_RX0_CONFIG(x) (0x1020 + (x*0x1000))

#define CS4223_PP_LINE_SDS_COMMON_SRX0_RX_CONFIG 0x1021

#define CS4223_PP_LINE_SDS_COMMON_SRX0_RX_CPA(x) (0x1027 + (x*0x1000))

#define CS4223_PP_LINE_SDS_COMMON_SRX0_AGC_RTUNE(x) (0x10a5 + (x*0x1000))

#define CS4223_PP_LINE_SDS_COMMON_SRX0_DFE_BIAS1(x) (0x10aa + (x*0x1000))
#define CS4223_PP_LINE_SDS_COMMON_SRX0_DFE_BIAS1_DFT 0x5555

#define CS4223_PP_LINE_SDS_COMMON_TX0_CONFIG(x) (0x108e + (x*0x1000))

#define CS4223_PP_LINE_SDS_COMMON_STX0_SQUELCH(x) (0x1091 + (x*0x1000))

#define CS4223_PP_HOST_SDS_COMMON_STX0_SQUELCH(x) (0x1891 + (x*0x1000))

#define CS4223_PP_LINE_SDS_COMMON_RXLOCKED0_INTSTAT(x) (0x105c + (x*0x1000))
#define CS4223_PP_LINE_SDS_COMMON_RXLOCKED0_INTEN(x) (0x105d + (x*0x1000))

#define CS4223_PP_LINE_SDS_COMMON_RXLOCKED0_RESOLUTION(x) (0x1057 + (x*0x1000))

#define CS4223_PP_LINE_SDS_COMMON_SRX0_AGC_CONFIG1(x) (0x10a1 + (x*0x1000))
#define CS4223_PP_LINE_SDS_COMMON_SRX0_AGC_CONFIG1_DFT 0x04
#define CS4223_PP_LINE_SDS_COMMON_SRX0_AGC_CONFIG4(x) (0x10a4 + (x*0x1000))

#define CS4223_PP_LINE_SDS_COMMON_SRX0_DFE_CONFIG(x) (0x10ae + (x*0x1000))
#define CS4223_PP_LINE_SDS_COMMON_SRX0_DFE_DFT 0x441

#define CS4223_PP_LINE_SDS_COMMON_SRX0_DAC_ENB_LSB(x) (0x10a7 + (x*0x1000))
#define CS4223_PP_LINE_SDS_COMMON_SRX0_DAC_ENB_MSB(x) (0x10a8 + (x*0x1000))


#define CS4223_PP_LINE_SDS_COMMON_INT(x) (0x10ba + (x*0x1000))
#define CS4223_PP_LINE_SDS_COMMON_INTEN(x) (0x10bb + (x*0x1000))

#define CS4223_PP_LINE_GIGEPCS_INT_LINE_PCS1GBE_INT(x) (0x1480 + (x*0x1000))
#define CS4223_PP_LINE_GIGEPCS_INT_LINE_PCS1GBE_INTEN(x) (0x1481 + (x*0x1000))

#define CS4223_PP_LINE_EGPCS_RX_MODE(x) (0x14a0 + (x*0x1000))

#define CS4223_PP_HOST_HOSTMISC_FUNCEN(x) (0x1803 + (x*0x1000))
#define CS4223_PP_HOST_HOSTMISC_GIGEPCS_SOFT_RESET(x) (0x1802 + (x*0x1000))

#define CS4223_PP_HOST_HOSTMISC_CLKEN(x) (0x1804 + (x*0x1000))
#define CS4223_PP_HOST_SDS_COMMON_RX0_CONFIG(x) (0x1820 + (x*0x1000))
#define CS4223_PP_HOST_SDS_COMMON_TX0_CONFIG(x) (0x188e + (x*0x1000))

#define CS4223_PP_LINE_SDS_DSP_MSEQ_COEF_DSP_FLOAT(x) (0x124c + (x*0x1000))
#define CS4223_PP_LINE_SDS_DSP_MSEQ_ENABLE(x) (0x1234 + (x*0x1000))

#define CS4223_PP_LINE_SDS_DSP_MSEQ_GRAM_CR(x) (0x1220 + (x*0x1000))
#define CS4223_PP_LINE_SDS_DSP_MSEQ_GRAM_D1(x) (0x1221 + (x*0x1000))
#define CS4223_PP_LINE_SDS_DSP_MSEQ_GRAM_D0(x) (0x1222 + (x*0x1000))
#define CS4223_PP_LINE_SDS_DSP_MSEQ_OPTIONS(x) (0x1225 + (x*0x1000))
#define CS4223_PP_LINE_SDS_DSP_MSEQ_PC(x) (0x1228 + (x*0x1000))
#define CS4223_PP_LINE_SDS_DSP_MSEQ_POWER_DOWN_LSB(x) (0x1235 + (x*0x1000))
#define CS4223_PP_LINE_SDS_DSP_MSEQ_POWER_DOWN_MSB(x) (0x1236 + (x*0x1000))
#define CS4223_PP_LINE_SDS_DSP_MSEQ_BANK_SELECTOR(x) (0x1238 + (x*0x1000))
#define CS4223_PP_LINE_SDS_DSP_MSEQ_MAIL_IN_LSB(x) (0x123a + (x*0x1000))
#define CS4223_PP_LINE_SDS_DSP_MSEQ_MAIL_IN_MSB(x) (0x123b + (x*0x1000))
#define CS4223_PP_LINE_SDS_DSP_MSEQ_MAIL_OUT_LSB(x) (0x123c + (x*0x1000))
#define CS4223_PP_LINE_SDS_DSP_MSEQ_MAIL_OUT_MSB(x) (0x123d + (x*0x1000))

#define CS4223_PP_LINE_SDS_DSPi_MSEQ_CAL_RX_PHSEL(x) (0x1265 + (x*0x1000))
#define CS4223_PP_LINE_SDS_DSP_MSEQ_CAL_RX_PHSEL_DFT 0x20
#define CS4223_PP_LINE_SDS_DSP_MSEQ_CAL_RX_EQADJ1(x) (0x125e + (x*0x1000))
#define CS4223_PP_LINE_SDS_DSP_MSEQ_CAL_RX_EQADJ2(x) (0x125f + (x*0x1000))
#define CS4223_PP_LINE_SDS_DSP_MSEQ_CAL_RX_SLICER(x) (0x1266 + (x*0x1000))


#define CS4223_PP_HOST_SDS_DSP_MSEQ_GRAM_CR(x) (0x1a20 + (x*0x1000))
#define CS4223_PP_HOST_SDS_DSP_MSEQ_GRAM_D1(x) (0x1a21 + (x*0x1000))
#define CS4223_PP_HOST_SDS_DSP_MSEQ_GRAM_D0(x) (0x1a22 + (x*0x1000))
#define CS4223_PP_HOST_SDS_DSP_MSEQ_PC(x) (0x1a28 + (x*0x1000))
#define CS4223_PP_HOST_SDS_DSP_MSEQ_POWER_DOWN_LSB(x) (0x1a35 + (x*0x1000))
#define CS4223_PP_HOST_SDS_DSP_MSEQ_BANK_SELECTOR(x) (0x1a38 + (x*0x1000))
#define CS4223_PP_HOST_SDS_DSP_MSEQ_MAIL_IN_LSB(x) (0x1a3a + (x*0x1000))
#define CS4223_PP_HOST_SDS_DSP_MSEQ_MAIL_IN_MSB(x) (0x1a3b + (x*0x1000))
#define CS4223_PP_HOST_SDS_DSP_MSEQ_MAIL_OUT_LSB(x) (0x1a3c + (x*0x1000))
#define CS4223_PP_HOST_SDS_DSP_MSEQ_MAIL_OUT_MSB(x) (0x1a3d + (x*0x1000))

#define CS4223_PP_HOST_SDS_DSP_MSEQ_SERDES(x) (0x1a75 + (x*0x1000))
//config change for host side to put in propper mode

#define CS4223_PP_LINE_SDS_DSP_MSEQ_SNR_CTRL(x) (0x126c + (x*0x1000))

#define CS4223_PP_LINE_SDS_DSP_MSEQ_SERDES(x) (0x1275 + (x*0x1000))

#define CS4223_PP_LINE_SDS_DSP_MSEQ_SPARE0_LSB(x) (0x1280 + (x*0x1000))
#define CS4223_PP_LINE_SDS_DSP_MSEQ_SPARE0_MSB(x) (0x1281 + (x*0x1000))
#define CS4223_PP_LINE_SDS_DSP_MSEQ_SPARE1_LSB(x) (0x1282 + (x*0x1000))
#define CS4223_PP_LINE_SDS_DSP_MSEQ_SPARE1_MSB(x) (0x1283 + (x*0x1000))

#define CS4223_PP_LINE_SDS_DSP_MSEQ_SPARE2_LSB(x) (0x1284 + (x*0x1000))
#define CS4223_PP_LINE_SDS_DSP_MSEQ_SPARE2_MSB(x) (0x1285 + (x*0x1000))

#define CS4223_PP_LINE_SDS_DSP_MSEQ_SPARE4_LSB(x) (0x1288 + (x*0x1000))
#define CS4223_PP_LINE_SDS_DSP_MSEQ_SPARE4_MSB(x) (0x1289 + (x*0x1000))

#define CS4223_PP_LINE_SDS_DSP_MSEQ_SPARE6_LSB(x)  (0x128c + (x*0x1000))
#define CS4223_PP_LINE_SDS_DSP_MSEQ_SPARE6_MSB(x)  (0x128d + (x*0x1000))
#define CS4223_PP_LINE_SDS_DSP_MSEQ_SPARE7_LSB(x)  (0x128e + (x*0x1000))
#define CS4223_PP_LINE_SDS_DSP_MSEQ_SPARE7_MSB(x) (0x128f + (x*0x1000))
#define CS4223_PP_LINE_SDS_DSP_MSEQ_SPARE9_LSB(x) (0x1292 + (x*0x1000))
#define CS4223_PP_LINE_SDS_DSP_MSEQ_SPARE9_MSB(x) (0x1293 + (x*0x1000))

#define CS4223_PP_LINE_SDS_DSP_MSEQ_SPARE11_LSB(x) (0x1296 + (x*0x1000))
#define CS4223_PP_LINE_SDS_DSP_MSEQ_SPARE11_MSB(x) (0x1297 + (x*0x1000))
#define CS4223_PP_LINE_SDS_DSP_MSEQ_SPARE12_LSB(x) (0x1298 + (x*0x1000))
#define CS4223_PP_LINE_SDS_DSP_MSEQ_SPARE12_MSB(x) (0x1299 + (x*0x1000))
#define CS4223_PP_LINE_SDS_DSP_MSEQ_SPARE13_LSB(x) (0x129a + (x*0x1000))
#define CS4223_PP_LINE_SDS_DSP_MSEQ_SPARE13_MSB(x) (0x129b + (x*0x1000))

#define CS4223_PP_LINE_SDS_DSP_MSEQ_SPARE18_LSB(x) (0x12a4 + (x*0x1000))
#define CS4223_PP_LINE_SDS_DSP_MSEQ_SPARE18_MSB(x) (0x12a5 + (x*0x1000))

#define CS4223_PP_LINE_SDS_DSP_MSEQ_SPARE21_LSB(x) (0x12aa + (x*0x1000))
#define CS4223_PP_LINE_SDS_DSP_MSEQ_SPARE21_MSB(x) (0x12ab + (x*0x1000))

#define CS4223_PP_LINE_SDS_DSP_MSEQ_SPARE24_LSB(x) (0x12b0 + (x*0x1000))
#define CS4223_PP_LINE_SDS_DSP_MSEQ_SPARE24_MSB(x) (0x12b1 + (x*0x1000))
#define CS4223_PP_LINE_SDS_DSP_MSEQ_SPARE25_LSB(x) (0x12b2 + (x*0x1000))
#define CS4223_PP_LINE_SDS_DSP_MSEQ_SPARE25_MSB(x) (0x12b3 + (x*0x1000))
#define CS4223_PP_LINE_SDS_DSP_MSEQ_SPARE26_LSB(x) (0x12b4 + (x*0x1000))
#define CS4223_PP_LINE_SDS_DSP_MSEQ_SPARE26_MSB(x) (0x12b5 + (x*0x1000))
#define CS4223_PP_LINE_SDS_DSP_MSEQ_SPARE27_LSB(x) (0x12b6 + (x*0x1000))
#define CS4223_PP_LINE_SDS_DSP_MSEQ_SPARE27_MSB(x) (0x12b7 + (x*0x1000))
#define CS4223_PP_LINE_SDS_DSP_MSEQ_SPARE28_LSB(x) (0x12b8 + (x*0x1000))
#define CS4223_PP_LINE_SDS_DSP_MSEQ_SPARE28_MSB(x) (0x12b9 + (x*0x1000))

#define CS4223_PP_HOST_SDS_DSP_MSEQ_SPARE6_LSB(x)  (0x1a8c + (x*0x1000))
#define CS4223_PP_HOST_SDS_DSP_MSEQ_SPARE6_MSB(x)  (0x1a8d + (x*0x1000))
#define CS4223_PP_HOST_SDS_DSP_MSEQ_SPARE12_LSB(x) (0x1a98 + (x*0x1000))
#define CS4223_PP_HOST_SDS_DSP_MSEQ_SPARE12_MSB(x) (0x1a99 + (x*0x1000))
#define CS4223_PP_HOST_SDS_DSP_MSEQ_SPARE20_LSB(x) (0x1aa8 + (x*0x1000))
#define CS4223_PP_HOST_SDS_DSP_MSEQ_SPARE20_MSB(x) (0x1aa9 + (x*0x1000))
#define CS4223_PP_HOST_SDS_DSP_MSEQ_SPARE24_LSB(x) (0x1ab0 + (x*0x1000))
#define CS4223_PP_HOST_SDS_DSP_MSEQ_SPARE24_MSB(x) (0x1ab1 + (x*0x1000))
#define CS4223_PP_HOST_SDS_DSP_MSEQ_SPARE25_LSB(x) (0x1ab2 + (x*0x1000))
#define CS4223_PP_HOST_SDS_DSP_MSEQ_SPARE25_MSB(x) (0x1ab3 + (x*0x1000))
#define CS4223_PP_HOST_SDS_DSP_MSEQ_SPARE26_LSB(x) (0x1ab4 + (x*0x1000))
#define CS4223_PP_HOST_SDS_DSP_MSEQ_SPARE26_MSB(x) (0x1ab5 + (x*0x1000))
#define CS4223_PP_HOST_SDS_DSP_MSEQ_SPARE28_LSB(x) (0x1ab8 + (x*0x1000))
#define CS4223_PP_HOST_SDS_DSP_MSEQ_SPARE28_MSB(x) (0x1ab9 + (x*0x1000))


#define CS4223_PP_LINE_SDS_MSEQ_SPARE24_MODE_CX1 0x04
#define CS4223_PP_LINE_SDS_MSEQ_SPARE24_MODE_SR 0x08
#define CS4223_PP_LINE_SDS_MSEQ_SPARE24_MODE_ZR 0x10
#define CS4223_PP_LINE_SDS_MSEQ_SPARE24_MODE_DWDM 0x20


#define CS4223_HOST_SDS_DSP_MSEQ_SPARE26(x) (0x1ab4 + (x*0x1000))
#define CS4223_HOST_SDS_COMMON_STX0_TX_OUTPUT_CTRLA(x) (0x188f + (x*0x1000))

#define CS4223_HOST_SDS_COMMON_STX0_TX_OUTPUT_CTRLB(x) (0x1890 + (x*0x1000))

#define CS4223_PP_LINE_AN_TX_ENABLE(x) (0x1500 + (x*0x1000))
#define CS4223_PP_LINE_AN_TX_EN 1

#define CS4223_PP_LINE_AN_TX_LINK_FAIL_TIMER_THRES0(x) (0x1525 + (x*0x1000))
#define CS4223_PP_LINE_AN_TX_LINK_FAIL_TIMER_THRES1(x) (0x1524 + (x*0x1000))

#define CS4223_PP_LINE_TP_TX_MAX_WAIT_TIMEOUT1(x) (0x158c + (x*0x1000))
#define CS4223_PP_LINE_TP_TX_MAX_WAIT_TIMEOUT0(x) (0x158d + (x*0x1000))

#define CS4223_PP_LINE_TP_TX_PRESET_COEFF_OVERRIDE(x) (0x1599 + (x*0x1000))

#define CS4223_PP_LINE_TP_TX_WAIT_TIMEOUT(x) (0x158e + (x*0x1000))

#define CS4223_PP_LINE_AN_TX_BP_STATUS(x) (0x151b + (x*0x1000))

#define CS4223_PP_LINE_AN_TX_RESET(x) (0x1501 + (x*0x1000))
#define CS4223_PP_LINE_AN_TX_RESET_TX 1
#define CS4223_PP_LINE_AN_TX_RESET_RX 2

#define CS4223_PP_LINE_AN_TX_RESTART(x) (0x1502 + (x*0x1000))
#define CS4223_PP_LINE_AN_TX_RESTART_EN 1

#define CS4223_PP_LINE_AN_TX_COMPLETE_STATUS(x) (0x1505 + (x*0x1000))

#define CS4223_PP_LINE_AN_TX_AN_COMPLETE_STATUS_INT(x) (0x1506 + (x*0x1000))
#define CS4223_PP_LINE_AN_TX_AN_COMPLETE_STATUS_INTE(x) (0x1507 + (x*0x1000))

#define CS4223_PP_LINE_AN_TX_ADV_ABILITY(x) (0x150c + (x*0x1000))
#define CS4223_PP_LINE_AN_TX_TECH_ABILITY_1(x) (0x150d + (x*0x1000))

#define CS4223_PP_LINE_AN_TX_MAIN_INT(x) (0x152d + (x*0x1000))

#define CS4223_PP_LINE_AN_TX_MAIN_INTE(x) (0x152e + (x*0x1000))

#define CS4223_PP_HOST_HOSTMISC_MSEQCLKCTRL(x) (0x1807 + (x*0x1000))
#define CS4223_PP_HOST_SDS_DSP_MSEQ_OPTIONS(x) (0x1A25 + (x*0x1000))

#define CS4223_PP_HOST_SDS_DSP_MSEQ_ENABLE(x) (0x1a34 + (x*0x1000))

#define CS4223_EEPROM_LOADER_STATUS 0x5001

#define CS4223_PP_HOST_SDS_COMMON_RXLOCKD0_INT(x)	(0x185c + (x*0x1000))
#define CS4223_PP_HOST_SDS_COMMON_RXLOCKD0_INTEN(x) 	(0x185d + (x*0x1000))


#define CS4223_PP_HOST_GIGEPCS_HOST_CTRL(x) (0x1c60 + (x*0x1000))

#define CS4223_GIGEPCS_RESET 0x8000
#define CS4223_GIGEPCS_AN_EN 0x1000

#define CS4223_PP_HOST_GIGEPCS_HOST_STATUS(x) (0x1c61 + (x*0x1000))

#define CS4223_PP_LINE_EGPCS_RX_MODE(x) (0x14a0 + (x*0x1000))


#define CS4223_PP_LINE_XGPCS_TX_TXCNTRL(x) (0x14c0 + (x*0x1000))

#define CS4223_PP_LINE_XGPCS_RX_RXCNTRL(x) (0x14e0 + (x*0x1000))

#define CS4223_PP_LINE_MCAN_RESET(x) (0x1680 + (x*0x1000))


#define CS4223_PP_LINE_AN_TX_CSTAT_AN_GOOD_CHK 2
#define CS4223_PP_LINE_AN_TX_CSTAT_AN_COMPLETE 1


#define CS4223_AN_MODE_10G 0
#define CS4223_AN_MODE_40G 1

//#define DEBUG 1

struct cti_cs4223e {
	u8 addr;

	u32 chip_id;
	u16 ucode_ver_sr;
	u16 ucode_ver_cx1;
	u16 ucode_ver_kr;
	u16 ucode_ver_zr;
	u16 ucode_ver_fc;

	u16 ucode_timestamp0; //month/date
	u16 ucode_timestamp1; //year
	u16 ucode_timestamp2; //timestamp

	struct i2c_client* client;
	struct device* dev;
};


static int cti_cs4223e_reg_write(struct cti_cs4223e* phy,u16 addr,u16 val){
	int ret;
	u8 wbuf[5];
	u8 rbuf[2];
	u16 tmp;
	u8 addr_lo,addr_hi,data_hi,data_lo;
	struct i2c_msg msg[2];
	
	memset(&msg[0],0,sizeof(struct i2c_msg)*2);

	addr_lo = (u8)(addr & 0xff);
	addr_hi = (u8)((addr & 0xff00) >> 8);

	data_lo = (u8)(val & 0xff);
	data_hi = (u8)((val & 0xff00) >> 8);

	wbuf[0] = (addr_hi << 1) | 0x0; //combined write
	wbuf[1] = addr_lo;
	wbuf[2] = data_hi;
	wbuf[3] = data_lo;

	//8-bit one's complement checksum, carry bit is re-added to the sum
	tmp = wbuf[0] + wbuf[1] + wbuf[2] + wbuf[3];
	wbuf[4] = (tmp & 0xff00) >> 8;
	wbuf[4] += tmp & 0xff;	
	wbuf[4] = ~(wbuf[4]);

	msg[0].len = 5;
	msg[0].addr = phy->addr;
	msg[0].flags = 0;
	msg[0].buf = wbuf;
	
	msg[1].flags = I2C_M_RD;
	msg[1].len = 1;
	msg[1].addr = phy->addr;
	msg[1].buf = rbuf;

	ret = i2c_transfer(phy->client->adapter,msg,2);

	if(ret < 0)
		goto err;

	dev_dbg(phy->dev,"Checksum of write: %.2x\n",rbuf[0]);
	
	if(rbuf[0] == 0xBD){ //bad write
		dev_err(phy->dev,"Checksum fail: bad write (%.4x = %.4x)!\n",addr,val);
		return -EIO;
	}
	

	return 0;
err:
	dev_err(phy->dev,"Failed reading register(%.4x) ret = %d!\n",addr,ret);
	return ret;
}

static int cti_cs4223e_reg_read(struct cti_cs4223e* phy,u16 addr,u16* val){
	int ret;
	u8 wbuf[4];
	u8 rbuf[4];
	u16 tmp;
	u8 addr_lo,addr_hi;
	struct i2c_msg msg[2];
	
	memset(&msg[0],0,sizeof(struct i2c_msg)*2);

	addr_lo = (u8)(addr & 0xff);
	addr_hi = (u8)((addr & 0xff00) >> 8);

	
	wbuf[0] = (addr_hi << 1) | 0x1;
	wbuf[1] = addr_lo;
	
	tmp = wbuf[0] + wbuf[1];

	wbuf[2] = ~((tmp & 0xff) + ((tmp & 0xff00) >> 8)); //~(addr_hi + addr_lo);
	
	
	msg[0].len = 3;
	msg[0].addr = phy->addr;
	msg[0].flags = 0;
	msg[0].buf = wbuf;
	
	msg[1].flags = I2C_M_RD;
	msg[1].len = 3;
	msg[1].addr = phy->addr;
	msg[1].buf = rbuf;

	ret = i2c_transfer(phy->client->adapter,msg,2);

	if(ret < 0)
		goto err;

	dev_dbg(phy->dev,"Checksum of read: %.2x\n",rbuf[2]);
	
	if(rbuf[2] == 0xBD){ //bad read
		if(rbuf[0] == 0xBA && rbuf[1] == 0xDA) 
			dev_err(phy->dev,"Checksum fail: bad address!\n");
		return -EIO;
	}
	
	*val = (((u16)rbuf[0]) << 8) | (((u16)rbuf[1]));

	return 0;
err:
	dev_err(phy->dev,"Failed reading register(%.4x) ret = %d!\n",addr,ret);
	return ret;
}

__maybe_unused static int cti_cs4223e_regdump(struct cti_cs4223e* phy){
	int i;
	int ret;
	u16 read;
	for(i=0; i < 32; i++){
		ret = cti_cs4223e_reg_read(phy,i,&read);
		dev_info(phy->dev,"%.4x: %.4x\n",i,read);
	}
	return 0;
}

static bool cti_cs4223e_eeprom_loaded(struct cti_cs4223e* phy){
	u16 pin_stat;
	u16 efuse_stat;
	u16 eeprom_stat;
	int ret;
	

	ret = cti_cs4223e_reg_read(phy,CS4223_EFUSE_GENERAL_STATUS,&efuse_stat);

	//hasn't finished loading efuse
	if(efuse_stat != 1)
		return false;


	ret = cti_cs4223e_reg_read(phy,CS4223_GLOBAL_PIN_STATUS,&pin_stat);

	//f00d = loader is still running
	//ffff = refclk not stable
	if(pin_stat == 0xf00d || pin_stat == 0xffff || pin_stat == 0xbada){
		return false;
	}else if(!(pin_stat & 0x4000)){
		dev_warn(phy->dev,"No EEPROM present.\n");
		return true; //No eeprom present
	}

	ret = cti_cs4223e_reg_read(phy,CS4223_EEPROM_LOADER_STATUS,&eeprom_stat);

	if(eeprom_stat == 0xbada)
		return false;

	
	if(eeprom_stat & 1) //load done
		return true;
	
	if(eeprom_stat & 2){ //load failed
		dev_err(phy->dev,"EEPROM Load failed.\n");
		return true; 
	}

	if(eeprom_stat & 4){
		dev_err(phy->dev,"Load Abort: %.4x\n",eeprom_stat);
		return true;
	}
	return false;
}

static bool cti_cs4223e_wait_eeprom_finished(struct cti_cs4223e* phy,u32 max_tries,u32 delay){
	u32 attempts = 0;
	bool finished = false;

	if(!delay)
		delay = 1;

	do {

		finished = cti_cs4223e_eeprom_loaded(phy);

		if(!finished)
			mdelay(delay);
		
	}while((attempts++ < max_tries) && !finished);

	return finished;
}


static int cti_cs4223e_gepcs_int(struct cti_cs4223e* phy,int line){
	int ret;
	u16 read;

	ret = cti_cs4223e_reg_read(phy,
			CS4223_PP_LINE_GIGEPCS_INT_LINE_PCS1GBE_INT(line),&read);

	ret = cti_cs4223e_reg_write(phy,
			CS4223_PP_LINE_GIGEPCS_INT_LINE_PCS1GBE_INT(line),read);

	//dev_info(phy->dev,"GIGEPCS Int: %.4x\n",read);

	return 0;
}

static int cti_cs4223e_kran_tx_int(struct cti_cs4223e* phy,int line){
	int ret;
	u16 read;

	ret = cti_cs4223e_reg_read(phy,
			CS4223_PP_LINE_AN_TX_MAIN_INT(line),
			&read);

	
	if(read & 0x20){
		ret = cti_cs4223e_reg_read(phy,
				CS4223_PP_LINE_AN_TX_AN_COMPLETE_STATUS_INT(line)
				,&read);

		if(read & 1){
			dev_info(phy->dev,"AUTONEG Complete.\n");
		}
		if(read & 2){
			dev_info(phy->dev,"AUTONEG good check.\n");
		}
		ret = cti_cs4223e_reg_write(phy,
				CS4223_PP_LINE_AN_TX_AN_COMPLETE_STATUS_INT(line),
				read);
	}
	
	return 0;
}

static int cti_cs4223e_hostx_int(struct cti_cs4223e* phy,int host){
	//int ret;

	dev_info(phy->dev,"Hostmisc interrupt.\n");

	return 0;
}

static int cti_cs4223e_linex_int(struct cti_cs4223e* phy,int line){
	int ret;
	u16 read;

	//ret = cti_cs4223e_reg_read(phy,CS4223_PP_LINE_SDS_COMMON_INT(line),&read);

	ret = cti_cs4223e_reg_read(phy,CS4223_PP_LINE_LINEMISC_INT(line),&read);

	ret = 0;
	if(read & 0x04){ //GEPCS Interrupt
		ret |= cti_cs4223e_gepcs_int(phy,line);
	}

	if(read & 0x200){
		ret |= cti_cs4223e_kran_tx_int(phy,line);
	}
	//dev_info(phy->dev,"SDS_COMMON_INT: %.4x\n",read);

	return ret;
}

static int cti_cs4223e_datapath_int(struct cti_cs4223e* phy){
	int ret;
	int i;
	u16 read;
	u16 tmp;

	ret = cti_cs4223e_reg_read(phy,CS4223_GLOBAL_DATAPATH_INT,&read);

	ret = cti_cs4223e_reg_write(phy,CS4223_GLOBAL_DATAPATH_INT,read);

	//dev_info(phy->dev,"Datapath interrupt: %.4x\n",read);
	ret = 0;

	tmp = read & 0xf; //grab lines
	for(i=0;i<4;i++){
		if(tmp & 1)
			ret |= cti_cs4223e_linex_int(phy,i);
		tmp = tmp >> 1;
	}

	tmp = (read & 0xf0) >> 4;
	for(i=0;i<4;i++){
		if(tmp & 1)
			ret |= cti_cs4223e_hostx_int(phy,i);
		tmp = tmp >> 1;
	}

	return ret;
}

static irqreturn_t cti_cs4223e_irq_handler(int irq,void* data){
	__maybe_unused struct cti_cs4223e* phy;
	int ret;
	//int i =0;
	u16 glbl_int;

	phy = (struct cti_cs4223e*)data;

	ret = cti_cs4223e_reg_read(phy,CS4223_GLOBAL_INT,&glbl_int);
	if(!glbl_int)
		return IRQ_NONE;

	if(glbl_int & 1){

		cti_cs4223e_datapath_int(phy);
		
	}

	return IRQ_HANDLED;
}

static inline int cti_cs4223e_get_irq(struct cti_cs4223e* phy){
	if(!phy || !phy->client){
		return -1;
	}

	return phy->client->irq;
}

static int cti_cs4223e_setup_irq(struct cti_cs4223e* phy){
	int ret;
	int irq;

	if(!phy || !phy->dev){
		return -EINVAL;
	}

	irq = cti_cs4223e_get_irq(phy);

	if(irq < 0){
		dev_err(phy->dev,"Failed to get IRQ!\n");
		return -ENXIO;
	}

	dev_info(phy->dev,"Registering irq (%d)\n",irq);
	ret = devm_request_threaded_irq(phy->dev,irq,NULL,
			cti_cs4223e_irq_handler, IRQF_ONESHOT,
			dev_name(phy->dev),phy);

	if(ret != 0){
		dev_err(phy->dev,"IRQ Request failed err: %d\n",ret);
	}else{
		dev_info(phy->dev,"IRQ registered: %d\n",irq);
	}

	return ret;
}

#define LINE_USEC_STARTING_BANK 	2
#define HOST_USEC_STARTING_BANK 	7
#define DATA_STORE_PCCR_ADDR 		0x0b
#define DATA_STORE_BANK_JUMP1_ADDR 	(0x62 - 0x40)
#define DATA_STORE_BANK_JUMP2_ADDR 	(0x63 - 0x40)
#define DATA_STORE_FUNCTION_NUMBER_ADDR 	(0x4e - 0x40)
#define DATA_STORE_WAIT_VCO_ADDR   	(0x46 - 0x40)
#define DATA_STORE_WAIT_CAL_ADDR	(0x47 - 0x40)
#define DATA_STORE_WAIT_TAPS_ADDR	(0x48 - 0x40)
#define DATA_STORE_WAIT_MISSION_ADDR	(0x49 - 0x40)
#define DATA_STORE_WAIT_ARBITRARY_ADDR	(0x4a - 0x40)
#define DATA_STORE_TX_MUX_ADDR		(0x50 - 0x40)
#define DATA_STORE_TX_EYE_OR_2X2_ADDR	(0x51 - 0x40)
#define DATA_STORE_WAIT_VCO_VAL		0x00020000
#define DATA_STORE_WAIT_CAL_VAL		0x00100000
#define DATA_STORE_WAIT_TAPS_VAL	0x00010000
#define DATA_STORE_WAIT_MISSION_VAL	0x00000100
#define DATA_STORE_WAIT_ARBITRARY_VAL	0x00020000

__maybe_unused static int cti_cs4223e_mseq_stall(struct cti_cs4223e* phy,int host,int line,bool stall){
	int ret;
	u16 spare12;
	u16 data;
	u16 bank;

	u16 offset;

	if(host)
		offset = CS4223_LINE_TO_HOST_OFFSET;
	else
		offset = 0;

	ret = cti_cs4223e_reg_read(phy,CS4223_PP_LINE_SDS_DSP_MSEQ_SPARE12_LSB(line)+offset,&spare12);

	if(stall){
		ret |= cti_cs4223e_reg_write(phy,CS4223_PP_LINE_LINEMISC_MSEQCLKCTRL(line)+offset,
				0x8004);
		ret |= cti_cs4223e_reg_write(phy,CS4223_PP_LINE_SDS_DSP_MSEQ_OPTIONS(line)+offset,
				0x0008);
		mdelay(150);

		ret |= cti_cs4223e_reg_write(phy,CS4223_PP_LINE_SDS_DSP_MSEQ_ENABLE(line)+offset,
				0x0000);

		ret |= cti_cs4223e_reg_read(phy,CS4223_PP_LINE_SDS_DSP_MSEQ_SERDES(line)+offset,&data);
		data &= ~0xf;
		ret |= cti_cs4223e_reg_write(phy,CS4223_PP_LINE_SDS_DSP_MSEQ_SERDES(line)+offset,data);
	}else{
		ret |= cti_cs4223e_reg_write(phy,CS4223_PP_LINE_SDS_DSP_MSEQ_SNR_CTRL(line)+offset,
				0x0000);
		
		ret |= cti_cs4223e_reg_read(phy,CS4223_PP_LINE_SDS_DSP_MSEQ_BANK_SELECTOR(line)+offset,
				&bank);

		ret |= cti_cs4223e_reg_write(phy,CS4223_PP_LINE_SDS_DSP_MSEQ_BANK_SELECTOR(line)+offset,
				0x0007);
		ret |= cti_cs4223e_reg_write(phy,CS4223_PP_LINE_SDS_DSP_MSEQ_BANK_SELECTOR(line)+offset,
				0x0000);

		if(spare12 & 0x0020){
			ret |= cti_cs4223e_reg_write(phy,
					CS4223_PP_LINE_SDS_DSP_MSEQ_BANK_SELECTOR(line)+offset,0x0007);
		}else{
			if(bank == 3){
				ret |= cti_cs4223e_reg_write(phy,
					CS4223_PP_LINE_SDS_DSP_MSEQ_BANK_SELECTOR(line)+offset,0x0003);
			}else{
				ret |= cti_cs4223e_reg_write(phy,
					CS4223_PP_LINE_SDS_DSP_MSEQ_BANK_SELECTOR(line)+offset,0x0000);
			}

		}

		ret |= cti_cs4223e_reg_read(phy,
				CS4223_PP_LINE_SDS_DSP_MSEQ_SPARE26_MSB(line)+offset,
				&data);

		if(data & 0x8000){
			ret |= cti_cs4223e_reg_write(phy,
					CS4223_PP_LINE_SDS_DSP_MSEQ_BANK_SELECTOR(line)+offset,
					0x0007);

		}

		ret |= cti_cs4223e_reg_write(phy,
				CS4223_PP_LINE_LINEMISC_MSEQCLKCTRL(line)+offset,
				0x0004);

		ret |= cti_cs4223e_reg_write(phy,
				CS4223_PP_LINE_SDS_DSP_MSEQ_GRAM_CR(line)+offset+1,0x0000);
		ret |= cti_cs4223e_reg_write(phy,
				CS4223_PP_LINE_SDS_DSP_MSEQ_GRAM_CR(line)+offset+2,0x0000);
		ret |= cti_cs4223e_reg_write(phy,
				CS4223_PP_LINE_SDS_DSP_MSEQ_GRAM_CR(line)+offset,0x803c);

		ret |= cti_cs4223e_reg_write(phy,
				CS4223_PP_LINE_SDS_DSP_MSEQ_PC(line)+offset,0x0000);
		ret |= cti_cs4223e_reg_write(phy,
				CS4223_PP_LINE_SDS_DSP_MSEQ_OPTIONS(line)+offset,0x0007);

	}
	
	return ret;
}

__maybe_unused static int cti_cs4223e_enter_post_kran(struct cti_cs4223e* phy,int line){
	int ret;
	



	return ret;
}


__maybe_unused static int cti_cs4223e_kran_status(struct cti_cs4223e* phy,int line){
	int ret;
	u16 complete_status;
	u16 main_ints;
	u16 bp_status;

	ret = cti_cs4223e_reg_read(phy,CS4223_PP_LINE_AN_TX_COMPLETE_STATUS(line),&complete_status);
	ret |= cti_cs4223e_reg_read(phy,CS4223_PP_LINE_AN_TX_MAIN_INT(line),&main_ints);
	ret |= cti_cs4223e_reg_read(phy,CS4223_PP_LINE_AN_TX_BP_STATUS(line),&bp_status); 
	
	dev_info(phy->dev,"AN Status word: %.4x Main ints: %.4x\n",complete_status,main_ints);
	dev_info(phy->dev,"BP Status word: %.4x\n",bp_status);
	

	return ret;
}

__maybe_unused static int cti_cs4223e_init_edc_mode(struct cti_cs4223e* phy,int line,int host,int mode){
	int ret;
	u16 offset;

	if(host)
		offset = CS4223_LINE_TO_HOST_OFFSET;
	else
		offset = 0;

	//reset mseq datastore between mode switches
	ret = cti_cs4223e_reg_write(phy,CS4223_PP_LINE_SDS_DSP_MSEQ_GRAM_D0(line) + offset,
			0x0000);
	ret |= cti_cs4223e_reg_write(phy,CS4223_PP_LINE_SDS_DSP_MSEQ_GRAM_D1(line) + offset,
			0x0000);
	ret |= cti_cs4223e_reg_write(phy,CS4223_PP_LINE_SDS_DSP_MSEQ_GRAM_CR(line) + offset,
			0x803c);

	//func number
	ret |= cti_cs4223e_reg_write(phy,CS4223_PP_LINE_SDS_DSP_MSEQ_GRAM_D0(line) + offset,
			0x0000);
	ret |= cti_cs4223e_reg_write(phy,CS4223_PP_LINE_SDS_DSP_MSEQ_GRAM_D1(line) + offset,
			0x0000);
	ret |= cti_cs4223e_reg_write(phy,CS4223_PP_LINE_SDS_DSP_MSEQ_GRAM_CR(line) + offset,
			0x800E);

	//reset MSEQ regs to defaults between mode switches
	
	ret |= cti_cs4223e_reg_write(phy,CS4223_PP_LINE_SDS_COMMON_SRX0_AGC_CONFIG4(line) + offset,
			0x81f0);
	ret |= cti_cs4223e_reg_write(phy,CS4223_PP_LINE_SDS_DSP_MSEQ_SPARE12_MSB(line) + offset,
			0x0000);
	ret |= cti_cs4223e_reg_write(phy,CS4223_PP_LINE_SDS_DSP_MSEQ_SPARE4_LSB(line) + offset,
			0x3000);
	ret |= cti_cs4223e_reg_write(phy,CS4223_PP_LINE_SDS_DSP_MSEQ_SERDES(line) + offset,
			0x0030);
	ret |= cti_cs4223e_reg_write(phy,CS4223_PP_LINE_SDS_COMMON_RXLOCKED0_RESOLUTION(line)+offset,
			0x0001);
	ret |= cti_cs4223e_reg_write(phy,CS4223_PP_LINE_SDS_DSP_MSEQ_BANK_SELECTOR(line)+offset,
			0x0000);
	ret |= cti_cs4223e_reg_write(phy,CS4223_PP_LINE_SDS_DSP_MSEQ_SPARE21_LSB(line)+offset,
			0x0000);
	ret |= cti_cs4223e_reg_write(phy,CS4223_PP_LINE_SDS_DSP_MSEQ_SPARE21_MSB(line)+offset,
			0x0000);
	ret |= cti_cs4223e_reg_write(phy,CS4223_PP_LINE_SDS_DSP_MSEQ_SPARE13_LSB(line)+offset,
			0x0000);
	ret |= cti_cs4223e_reg_write(phy,CS4223_PP_LINE_SDS_DSP_MSEQ_SPARE13_MSB(line)+offset,
			0x0000);
	ret |= cti_cs4223e_reg_write(phy,CS4223_PP_LINE_SDS_COMMON_SRX0_DFE_BIAS1(line)+offset,
			CS4223_PP_LINE_SDS_COMMON_SRX0_DFE_BIAS1_DFT);
	
	if(mode){ //SR Mode
		//only set SR bit
		ret |= cti_cs4223e_reg_write(phy,
				CS4223_PP_LINE_SDS_DSP_MSEQ_SPARE12_LSB(line)+offset,
				0x0001);
		//Spare1[0] = 1 to speed up locking
		ret |= cti_cs4223e_reg_write(phy,
				CS4223_PP_LINE_SDS_DSP_MSEQ_SPARE1_LSB(line)+offset,
				0x0001);

		ret |= cti_cs4223e_reg_write(phy,
				CS4223_PP_LINE_SDS_DSP_MSEQ_POWER_DOWN_LSB(line)+offset,
				0x0065);
		ret |= cti_cs4223e_reg_write(phy,
				CS4223_PP_LINE_SDS_DSP_MSEQ_POWER_DOWN_MSB(line)+offset,
				0x0000);
		ret |= cti_cs4223e_reg_write(phy,
				CS4223_PP_LINE_SDS_DSP_MSEQ_BANK_SELECTOR(line)+offset,
				0x0003);
		ret |= cti_cs4223e_reg_write(phy,
				CS4223_PP_LINE_SDS_COMMON_SRX0_RX_CPA(line)+offset,
				0x0099);
		ret |= cti_cs4223e_reg_write(phy,
				CS4223_PP_LINE_SDS_COMMON_SRX0_AGC_CONFIG1(line)+offset,
				0x0004);
		ret |= cti_cs4223e_reg_write(phy,
				CS4223_PP_LINE_SDS_COMMON_SRX0_DFE_CONFIG(line)+offset,
				0x0441);
		ret |= cti_cs4223e_reg_write(phy,
				CS4223_PP_LINE_SDS_COMMON_SRX0_DAC_ENB_LSB(line)+offset,
				0xDFFF);
		ret |= cti_cs4223e_reg_write(phy,
				CS4223_PP_LINE_SDS_COMMON_SRX0_DAC_ENB_MSB(line)+offset,
				0x000d);
		ret |= cti_cs4223e_reg_write(phy,
				CS4223_PP_LINE_LINEMISC_OVERRIDE_EN(line)+offset,
				0x0000);

		//function num set to 1 for SR only
		ret |= cti_cs4223e_reg_write(phy,
				CS4223_PP_LINE_SDS_DSP_MSEQ_GRAM_D0(line)+offset,
				0x0001);
		ret |= cti_cs4223e_reg_write(phy,
				CS4223_PP_LINE_SDS_DSP_MSEQ_GRAM_D1(line)+offset,
				0x0000);
		ret |= cti_cs4223e_reg_write(phy,
				CS4223_PP_LINE_SDS_DSP_MSEQ_GRAM_CR(line)+offset,
				0x800e);

		//cs4223_update_pre_equalizer
		ret |= cti_cs4223e_reg_write(phy,
				CS4223_PP_LINE_SDS_COMMON_SRX0_RX_CPA(line)+offset,
				0xdd);
	}else{ //10G BP mode
		dev_info(phy->dev,"Setting %s to 10G BP mode.\n",host ? "HOST" : "LINE");
		ret |= cti_cs4223e_reg_write(phy,
				CS4223_PP_LINE_SDS_DSP_MSEQ_SPARE12_LSB(line)+offset,
				0x0b00);
		ret |= cti_cs4223e_reg_write(phy,
				CS4223_PP_LINE_SDS_DSP_MSEQ_SPARE12_MSB(line)+offset,
				0x0c09);
		ret |= cti_cs4223e_reg_write(phy,
				CS4223_PP_LINE_SDS_DSP_MSEQ_POWER_DOWN_LSB(line)+offset,
				0x001f);
		ret |= cti_cs4223e_reg_write(phy,
				CS4223_PP_LINE_SDS_COMMON_SRX0_RX_CPA(line)+offset,
				0x0099);
		ret |= cti_cs4223e_reg_write(phy,
				CS4223_PP_LINE_SDS_COMMON_SRX0_AGC_CONFIG1(line)+offset,
				0x0005);
		ret |= cti_cs4223e_reg_write(phy,
				CS4223_PP_LINE_SDS_COMMON_SRX0_DAC_ENB_LSB(line)+offset,
				0x8f00);
		ret |= cti_cs4223e_reg_write(phy,
				CS4223_PP_LINE_SDS_COMMON_SRX0_DAC_ENB_MSB(line)+offset,
				0x000c);
		ret |= cti_cs4223e_reg_write(phy,
				CS4223_PP_LINE_SDS_DSP_MSEQ_POWER_DOWN_LSB(line)+offset,
				0x0000);
		ret |= cti_cs4223e_reg_write(phy,
				CS4223_PP_LINE_SDS_DSP_MSEQ_CAL_RX_EQADJ1(line)+offset,
				0x0008);
		ret |= cti_cs4223e_reg_write(phy,
				CS4223_PP_LINE_SDS_DSP_MSEQ_CAL_RX_EQADJ2(line)+offset,
				0x000B);
		ret |= cti_cs4223e_reg_write(phy,
				CS4223_PP_LINE_SDS_DSP_MSEQ_BANK_SELECTOR(line)+offset,
				0x0000);
		ret |= cti_cs4223e_reg_write(phy,
				CS4223_PP_LINE_SDS_DSP_MSEQ_SPARE9_LSB(line)+offset,
				0x00ff);
		ret |= cti_cs4223e_reg_write(phy,
				CS4223_PP_LINE_SDS_DSP_MSEQ_SPARE9_MSB(line)+offset,
				0x0030);
		ret |= cti_cs4223e_reg_write(phy,
				CS4223_PP_LINE_SDS_DSP_MSEQ_SPARE6_LSB(line)+offset,
				0x000f);
		ret |= cti_cs4223e_reg_write(phy,
				CS4223_PP_LINE_SDS_DSP_MSEQ_SPARE6_MSB(line)+offset,
				0x0000);
		ret |= cti_cs4223e_reg_write(phy,
				CS4223_PP_LINE_SDS_DSP_MSEQ_SPARE2_LSB(line)+offset,
				0x2350);
		ret |= cti_cs4223e_reg_write(phy,
				CS4223_PP_LINE_SDS_DSP_MSEQ_SPARE2_MSB(line)+offset,
				0x1b50);
		ret |= cti_cs4223e_reg_write(phy,
				CS4223_PP_LINE_SDS_DSP_MSEQ_SPARE18_MSB(line)+offset,
				0x0000);
		ret |= cti_cs4223e_reg_write(phy,
				CS4223_PP_LINE_SDS_DSP_MSEQ_SPARE18_LSB(line)+offset,
				0x0000);
		ret |= cti_cs4223e_reg_write(phy,
				CS4223_PP_LINE_SDS_DSP_MSEQ_SPARE11_MSB(line)+offset,
				0x007f);
		ret |= cti_cs4223e_reg_write(phy,
				CS4223_PP_LINE_SDS_DSP_MSEQ_SPARE11_LSB(line)+offset,
				0xfffe);
		ret |= cti_cs4223e_reg_write(phy,
				CS4223_PP_LINE_SDS_DSP_MSEQ_SPARE7_LSB(line)+offset,
				0x0010);
		ret |= cti_cs4223e_reg_write(phy,
				CS4223_PP_LINE_SDS_DSP_MSEQ_SPARE7_MSB(line)+offset,
				0x0030);
		ret |= cti_cs4223e_reg_write(phy,
				CS4223_PP_LINE_SDS_DSP_MSEQ_COEF_DSP_FLOAT(line)+offset,
				0x8001);
		ret |= cti_cs4223e_reg_write(phy,
				CS4223_PP_LINE_SDS_DSP_MSEQ_SPARE0_LSB(line)+offset,
				0x001D);
		ret |= cti_cs4223e_reg_write(phy,
				CS4223_PP_LINE_SDS_DSP_MSEQ_SPARE0_MSB(line)+offset,
				0x000a);
		ret |= cti_cs4223e_reg_write(phy,
				CS4223_PP_LINE_SDS_DSP_MSEQ_SPARE4_LSB(line)+offset,
				0x2500);
		ret |= cti_cs4223e_reg_write(phy,
				CS4223_PP_LINE_SDS_DSP_MSEQ_SPARE4_MSB(line)+offset,
				0x0000);
		ret |= cti_cs4223e_reg_write(phy,
				CS4223_PP_LINE_SDS_DSP_MSEQ_SPARE21_LSB(line)+offset,
				0x000a);
		ret |= cti_cs4223e_reg_write(phy,
				CS4223_PP_LINE_SDS_DSP_MSEQ_SPARE21_MSB(line)+offset,
				0x0023);
		ret |= cti_cs4223e_reg_write(phy,
				CS4223_PP_LINE_SDS_DSP_MSEQ_CAL_RX_SLICER(line)+offset,
				0x0080);
		//cti_cs4223e_pgm_reg_from_efuse(phy,i,CFG_HOST_SIDE/LINE_SIDE);
		
		ret |= cti_cs4223e_reg_write(phy,
				CS4223_PP_LINE_SDS_COMMON_SRX0_AGC_RTUNE(line)+offset,
				0x0000);

	}

	return ret;
}

__maybe_unused static int cti_cs4223e_enter_op_state(struct cti_cs4223e* phy,int line,int mode){
	int ret;
	ret = cti_cs4223e_mseq_stall(phy,0,line,true); //stall host and line
	ret |= cti_cs4223e_mseq_stall(phy,1,line,true); 

	//squelch host and line
		
	ret |= cti_cs4223e_reg_write(phy,CS4223_PP_HOST_SDS_COMMON_STX0_SQUELCH(line),0x0001);

	ret |= cti_cs4223e_reg_write(phy,CS4223_PP_LINE_SDS_COMMON_STX0_SQUELCH(line),0x0001);


	//Reset MSEQ datastore
	ret = cti_cs4223e_init_edc_mode(phy,line,0,0); //10g mode
	ret |= cti_cs4223e_init_edc_mode(phy,line,1,0); //10g mode on host
	//ret |= cti_cs4223e_reg_write(phy,CS4223_PP_LINE_SDS_COMMON_STX0_SQUELCH(line),def);
	if(mode){
		ret |= cti_cs4223e_reg_write(phy,CS4223_PP_HOST_SDS_COMMON_STX0_SQUELCH(line),0x0000);
		ret |= cti_cs4223e_reg_write(phy,CS4223_PP_LINE_SDS_COMMON_STX0_SQUELCH(line),0x0000);
		ret |= cti_cs4223e_mseq_stall(phy,0,line,false);
		ret |= cti_cs4223e_mseq_stall(phy,1,line,false); //unstall
	}



	return ret;
}


/*
 * Procedure for renegotiating the link
 */

__maybe_unused static int cti_cs4223e_kran_config(struct cti_cs4223e* phy,int line){
	u16 data;
	__maybe_unused u16 tech;
	__maybe_unused u16 adv_ability;
	__maybe_unused u16 spare26_msb;
	__maybe_unused u16 rx_ready;
	__maybe_unused u16 spare27_lsb,spare27_msb,lfi_thresh;
	__maybe_unused u16 val_low, val_high;
	int ret;


	ret = cti_cs4223e_enter_op_state(phy,line,0);

	ret = cti_cs4223e_reg_write(phy,
			CS4223_PP_HOST_SDS_DSP_MSEQ_SPARE20_MSB(line),0x4000);

	//Startup clocks
	ret |= cti_cs4223e_reg_write(phy,
			CS4223_PP_LINE_SDS_DSP_MSEQ_POWER_DOWN_LSB(line),
			0x0000);
	ret |= cti_cs4223e_reg_write(phy,
			CS4223_PP_HOST_SDS_DSP_MSEQ_POWER_DOWN_LSB(line),
			0x0000);
	ret |= cti_cs4223e_reg_write(phy,
			CS4223_PP_LINE_LINEMISC_CLKEN(line), 0xffff);
	ret |= cti_cs4223e_reg_write(phy,
			CS4223_PP_HOST_HOSTMISC_CLKEN(line),0xffff);


	//Release digital registers from reset
	
	ret |= cti_cs4223e_reg_write(phy,
			CS4223_PP_LINE_LINEMISC_SOFT_RESET(line),0x0000);
	ret |= cti_cs4223e_reg_write(phy,
			CS4223_PP_HOST_HOSTMISC_SOFT_RESET(line),0x0000);
	ret |= cti_cs4223e_reg_write(phy,
			CS4223_PP_LINE_LINEMISC_MPIF_RESET_DOTREG(line),0x0000);
	ret |= cti_cs4223e_reg_write(phy,
			CS4223_PP_HOST_HOSTMISC_MPIF_RESET_DOTREG(line),0x0000);

	//Power up microsequencer
	ret |= cti_cs4223e_reg_write(phy,
			CS4223_PP_LINE_LINEMISC_MSEQCLKCTRL(line),0x8004);
	ret |= cti_cs4223e_reg_write(phy,
			CS4223_PP_HOST_HOSTMISC_MSEQCLKCTRL(line),0x8004);

	//Release transmitter squelching done in pre_setup?
	ret |= cti_cs4223e_reg_write(phy,
			CS4223_PP_LINE_SDS_COMMON_STX0_SQUELCH(line),0x0000);
	
	ret |= cti_cs4223e_reg_write(phy,
			CS4223_PP_HOST_SDS_COMMON_STX0_SQUELCH(line),0x0000);

	//under host microsequence ctrl of lock
	ret |= cti_cs4223e_reg_write(phy,
			CS4223_PP_HOST_SDS_DSP_MSEQ_ENABLE(line), 0x0000);

	//Force host and line usec to the correct bank, stall them out first
	ret |= cti_cs4223e_reg_write(phy,
			CS4223_PP_LINE_SDS_DSP_MSEQ_OPTIONS(line),0x0008);
	ret |= cti_cs4223e_reg_write(phy,
			CS4223_PP_HOST_SDS_DSP_MSEQ_OPTIONS(line),0x0008);

	//Set usec to correct banks
	ret |= cti_cs4223e_reg_write(phy,
			CS4223_PP_LINE_SDS_DSP_MSEQ_BANK_SELECTOR(line),
			LINE_USEC_STARTING_BANK);
	ret |= cti_cs4223e_reg_write(phy,
			CS4223_PP_HOST_SDS_DSP_MSEQ_BANK_SELECTOR(line),
			HOST_USEC_STARTING_BANK);

	//reset program counters
	ret |= cti_cs4223e_reg_write(phy,
			CS4223_PP_LINE_SDS_DSP_MSEQ_PC(line),
			0x0000);
	ret |= cti_cs4223e_reg_write(phy,
			CS4223_PP_HOST_SDS_DSP_MSEQ_PC(line),
			0x0000);

	//reset the line-side bank_jump data store registers
	ret |= cti_cs4223e_reg_write(phy,
			CS4223_PP_LINE_SDS_DSP_MSEQ_GRAM_D1(line),0x0000);
	ret |= cti_cs4223e_reg_write(phy,
			CS4223_PP_LINE_SDS_DSP_MSEQ_GRAM_D0(line),0x0000);
	ret |= cti_cs4223e_reg_write(phy,
			CS4223_PP_LINE_SDS_DSP_MSEQ_GRAM_CR(line),
			0x8000 | DATA_STORE_BANK_JUMP1_ADDR);
	ret |= cti_cs4223e_reg_write(phy,
			CS4223_PP_LINE_SDS_DSP_MSEQ_GRAM_CR(line),
			0x8000 | DATA_STORE_BANK_JUMP2_ADDR);
	ret |= cti_cs4223e_reg_write(phy,
			CS4223_PP_LINE_SDS_DSP_MSEQ_GRAM_CR(line),
			0x8000 | DATA_STORE_FUNCTION_NUMBER_ADDR);

	//Init PCCR data store register
	
	ret |= cti_cs4223e_reg_write(phy,
			CS4223_PP_LINE_SDS_DSP_MSEQ_GRAM_D1(line),
			0x0000);

	//two-pass mode
	ret |= cti_cs4223e_reg_write(phy,
	 		CS4223_PP_LINE_SDS_DSP_MSEQ_GRAM_D0(line),0x0002);
	ret |= cti_cs4223e_reg_write(phy,
			CS4223_PP_LINE_SDS_DSP_MSEQ_GRAM_D0(line),0x0000);

	ret |= cti_cs4223e_reg_write(phy,
			CS4223_PP_LINE_SDS_DSP_MSEQ_GRAM_CR(line),
			0x8000 | DATA_STORE_PCCR_ADDR);

	val_low = (u16)(DATA_STORE_WAIT_VCO_VAL & 0xffff);
	val_high = (u16)((DATA_STORE_WAIT_VCO_VAL & 0xffff0000) >> 16);
	ret |= cti_cs4223e_reg_write(phy,
			CS4223_PP_LINE_SDS_DSP_MSEQ_GRAM_D1(line),
			0x8000 | val_high);
	ret |= cti_cs4223e_reg_write(phy,
			CS4223_PP_LINE_SDS_DSP_MSEQ_GRAM_D0(line),
			val_low);
	ret |= cti_cs4223e_reg_write(phy,
			CS4223_PP_LINE_SDS_DSP_MSEQ_GRAM_CR(line),
			0x8000 | DATA_STORE_WAIT_VCO_ADDR);

	val_low = (u16)(DATA_STORE_WAIT_CAL_VAL & 0xffff);
	val_high = (u16) ((DATA_STORE_WAIT_CAL_VAL & 0xffff0000) >> 16);
	ret |= cti_cs4223e_reg_write(phy,
			CS4223_PP_LINE_SDS_DSP_MSEQ_GRAM_D1(line), val_high);
	ret |= cti_cs4223e_reg_write(phy,
			CS4223_PP_LINE_SDS_DSP_MSEQ_GRAM_D0(line), val_low);
	ret |= cti_cs4223e_reg_write(phy,
			CS4223_PP_LINE_SDS_DSP_MSEQ_GRAM_CR(line), 
			0x8000 | DATA_STORE_WAIT_CAL_ADDR);

	val_low = (u16)(DATA_STORE_WAIT_TAPS_VAL & 0xffff);
	val_high = (u16)((DATA_STORE_WAIT_TAPS_VAL & 0xffff0000) >> 16);
	ret |= cti_cs4223e_reg_write(phy,
			CS4223_PP_LINE_SDS_DSP_MSEQ_GRAM_D1(line),val_high);
	ret |= cti_cs4223e_reg_write(phy,
			CS4223_PP_LINE_SDS_DSP_MSEQ_GRAM_D0(line),val_low);
	ret |= cti_cs4223e_reg_write(phy,
			CS4223_PP_LINE_SDS_DSP_MSEQ_GRAM_CR(line),
			0x8000 | DATA_STORE_WAIT_TAPS_ADDR);

	val_low = (u16)(DATA_STORE_WAIT_MISSION_VAL & 0xffff);
	val_high = (u16)((DATA_STORE_WAIT_MISSION_VAL & 0xffff0000) >> 16);
	ret |= cti_cs4223e_reg_write(phy,
			CS4223_PP_LINE_SDS_DSP_MSEQ_GRAM_D1(line),
			0x8000 | val_high);
	ret |= cti_cs4223e_reg_write(phy,
			CS4223_PP_LINE_SDS_DSP_MSEQ_GRAM_D0(line),
			val_low);
	ret |= cti_cs4223e_reg_write(phy,
			CS4223_PP_LINE_SDS_DSP_MSEQ_GRAM_CR(line),
			0x8000 | DATA_STORE_WAIT_MISSION_ADDR);

	val_low = (u16)(DATA_STORE_WAIT_ARBITRARY_VAL & 0xffff);
	val_high = (u16)((DATA_STORE_WAIT_ARBITRARY_VAL & 0xffff0000) >> 16);
	ret |= cti_cs4223e_reg_write(phy,
			CS4223_PP_LINE_SDS_DSP_MSEQ_GRAM_D1(line),
			0x8000 | val_high);
	ret |= cti_cs4223e_reg_write(phy,
			CS4223_PP_LINE_SDS_DSP_MSEQ_GRAM_D0(line),
			val_low);
	ret |= cti_cs4223e_reg_write(phy,
			CS4223_PP_LINE_SDS_DSP_MSEQ_GRAM_CR(line),
			0x8000 | DATA_STORE_WAIT_ARBITRARY_ADDR);

	//Init STX0_MISC data store registers, line and host
	
	ret |= cti_cs4223e_reg_write(phy,
			CS4223_PP_LINE_SDS_DSP_MSEQ_GRAM_D1(line),
			0x0000);
	ret |= cti_cs4223e_reg_write(phy,
			CS4223_PP_HOST_SDS_DSP_MSEQ_GRAM_D1(line),
			0x0000);
	ret |= cti_cs4223e_reg_write(phy,
			CS4223_PP_LINE_SDS_DSP_MSEQ_GRAM_D0(line),
			0x0022);
	ret |= cti_cs4223e_reg_write(phy,
			CS4223_PP_LINE_SDS_DSP_MSEQ_GRAM_CR(line),
			0x8000 | DATA_STORE_TX_MUX_ADDR);
	ret |= cti_cs4223e_reg_write(phy,
			CS4223_PP_HOST_SDS_DSP_MSEQ_GRAM_D0(line),
			0x0022);
	ret |= cti_cs4223e_reg_write(phy,
			CS4223_PP_HOST_SDS_DSP_MSEQ_GRAM_CR(line),
			0x8000 | DATA_STORE_TX_MUX_ADDR);
	ret |= cti_cs4223e_reg_write(phy,
			CS4223_PP_LINE_SDS_DSP_MSEQ_GRAM_D0(line),
			0x0033);
	ret |= cti_cs4223e_reg_write(phy,
			CS4223_PP_LINE_SDS_DSP_MSEQ_GRAM_CR(line),
			0x8000 | DATA_STORE_TX_EYE_OR_2X2_ADDR);
	ret |= cti_cs4223e_reg_write(phy,
			CS4223_PP_HOST_SDS_DSP_MSEQ_GRAM_D0(line),
			0x0033);
	ret |= cti_cs4223e_reg_write(phy,
			CS4223_PP_HOST_SDS_DSP_MSEQ_GRAM_CR(line),
			0x8000 | DATA_STORE_TX_EYE_OR_2X2_ADDR);

	//init line and host useq mailboxes 

	ret |= cti_cs4223e_reg_write(phy,
			CS4223_PP_LINE_SDS_DSP_MSEQ_MAIL_OUT_LSB(line),
			0x0000);
	ret |= cti_cs4223e_reg_write(phy,
			CS4223_PP_LINE_SDS_DSP_MSEQ_MAIL_OUT_MSB(line),
			0x0000);
	ret |= cti_cs4223e_reg_write(phy,
			CS4223_PP_LINE_SDS_DSP_MSEQ_MAIL_IN_LSB(line),
			0x0000);
	ret |= cti_cs4223e_reg_write(phy,
			CS4223_PP_LINE_SDS_DSP_MSEQ_MAIL_IN_MSB(line),
			0x0000);
	ret |= cti_cs4223e_reg_write(phy,
			CS4223_PP_HOST_SDS_DSP_MSEQ_MAIL_OUT_LSB(line),
			0x0000);
	ret |= cti_cs4223e_reg_write(phy,
			CS4223_PP_HOST_SDS_DSP_MSEQ_MAIL_OUT_MSB(line),
			0x0000);
	ret |= cti_cs4223e_reg_write(phy,
			CS4223_PP_HOST_SDS_DSP_MSEQ_MAIL_IN_LSB(line),
			0x0000);
	ret |= cti_cs4223e_reg_write(phy,
			CS4223_PP_HOST_SDS_DSP_MSEQ_MAIL_IN_MSB(line),
			0x0000);

	//init line mseq spare26 to 0
	ret |= cti_cs4223e_reg_write(phy,
			CS4223_PP_LINE_SDS_DSP_MSEQ_SPARE26_LSB(line),
			0x0000);

	// for host, init spare24 bit 31 high so dfe/sr ucode does not run
	ret |= cti_cs4223e_reg_write(phy,
			CS4223_PP_HOST_SDS_DSP_MSEQ_SPARE24_MSB(line),
			0x8000);

	//for host, init spare28 30 high
	ret |= cti_cs4223e_reg_write(phy,
			CS4223_PP_HOST_SDS_DSP_MSEQ_SPARE28_MSB(line),
			0x4000);

	//Release mseq from stall
	ret |= cti_cs4223e_reg_write(phy,
			CS4223_PP_LINE_SDS_DSP_MSEQ_OPTIONS(line),
			0x0007);
	ret |= cti_cs4223e_reg_write(phy,
			CS4223_PP_HOST_SDS_DSP_MSEQ_OPTIONS(line),
			0x0007);

	//Disable bit-swap
	
	ret |= cti_cs4223e_reg_read(phy,
			CS4223_PP_LINE_SDS_COMMON_RX0_CONFIG(line),
			&data);
	data &= ~(1 << 3);
	ret |= cti_cs4223e_reg_write(phy,
			CS4223_PP_LINE_SDS_COMMON_RX0_CONFIG(line),
			data);

	ret |= cti_cs4223e_reg_read(phy,
			CS4223_PP_LINE_SDS_COMMON_TX0_CONFIG(line),
			&data);
	data &= ~(1 << 3);
	ret |= cti_cs4223e_reg_write(phy,
			CS4223_PP_LINE_SDS_COMMON_TX0_CONFIG(line),
			data);

	ret |= cti_cs4223e_reg_read(phy,
			CS4223_PP_HOST_SDS_COMMON_RX0_CONFIG(line),
			&data);
	data &= ~(1 << 3);
	ret |= cti_cs4223e_reg_write(phy,
			CS4223_PP_HOST_SDS_COMMON_RX0_CONFIG(line),
			data);

	ret |= cti_cs4223e_reg_read(phy,
			CS4223_PP_HOST_SDS_COMMON_TX0_CONFIG(line),
			&data);
	data &= ~(1 << 3);
	ret |= cti_cs4223e_reg_write(phy,
			CS4223_PP_HOST_SDS_COMMON_TX0_CONFIG(line),
			data);

	//Release MCAN reset
	ret |= cti_cs4223e_reg_write(phy,
			CS4223_PP_LINE_MCAN_RESET(line),
			0x0000);
	
	ret |= cti_cs4223e_reg_write(phy,
			CS4223_PP_LINE_LINEMISC_AN_MODE_SEL(line),
			0x0000); //force 10g

	//Enable 10GE PCS transmitter
	ret |= cti_cs4223e_reg_write(phy,
			CS4223_PP_LINE_XGPCS_TX_TXCNTRL(line),
			0x0002);
	//Release GIGEPCS Rx and Tx and EGPCS Rx from reset
	ret |= cti_cs4223e_reg_write(phy,
			CS4223_PP_LINE_LINEMISC_GIGEPCS_SOFT_RESET(line),
			0x0000);
	ret |= cti_cs4223e_reg_write(phy,
			CS4223_PP_LINE_EGPCS_RX_MODE(line),
			0x0003);

	//Enable host-side 1GE PCS unit
	ret |= cti_cs4223e_reg_write(phy,
			CS4223_PP_HOST_HOSTMISC_FUNCEN(line),
			0x0001);
	ret |= cti_cs4223e_reg_write(phy,
			CS4223_PP_HOST_HOSTMISC_GIGEPCS_SOFT_RESET(line),
			0x0000);
		
	/*
	 * SPARE12[30] tells dfe mission code KR is being used and to jump back to KR code
	 * SPARE12[3] = 1 causes the CMODE_RES algo to use CR4 table
	 * SPARE12[26] = 1 enable SNR monitor
	 */
	ret |= cti_cs4223e_reg_read(phy,
			CS4223_PP_LINE_SDS_DSP_MSEQ_SPARE12_MSB(line),&data);
	data |= 0x4000;
	data &= ~0x0400;

	ret |= cti_cs4223e_reg_write(phy,
			CS4223_PP_LINE_SDS_DSP_MSEQ_SPARE12_MSB(line),data);
	
	ret |= cti_cs4223e_reg_read(phy,
			CS4223_PP_LINE_SDS_DSP_MSEQ_SPARE12_LSB(line),&data);

	// data |= 0x0008; //tech_cr4 en
	// data |= 0x0004; //sqlch_ing 
	data &= ~0x0004;

	ret |= cti_cs4223e_reg_write(phy,
			CS4223_PP_LINE_SDS_DSP_MSEQ_SPARE12_LSB(line),data);

	//Set KR Bit in SPARE12[30] on host side
	ret |= cti_cs4223e_reg_read(phy,
			CS4223_PP_HOST_SDS_DSP_MSEQ_SPARE12_MSB(line),&data);
	data |= 0x4000;
	//hphase_or --> data |= 0x2000;
	ret |= cti_cs4223e_reg_write(phy,
			CS4223_PP_HOST_SDS_DSP_MSEQ_SPARE12_MSB(line),data);

	//Clear host Tx squelch bit SPARE12[2] and power savings bit SPARE12[1] for host useq.
	ret |= cti_cs4223e_reg_read(phy,
			CS4223_PP_HOST_SDS_DSP_MSEQ_SPARE12_LSB(line),&data);

	data &= ~0x0002;

	data &= ~0x0004;

	data |= 0x0040;

	ret |= cti_cs4223e_reg_write(phy,
			CS4223_PP_HOST_SDS_DSP_MSEQ_SPARE12_LSB(line),
			data);

	//SPARE4 tells DFE how many mission.asm loops to run before starting phase calibration
	//For KR, phase calib is manually triggered
	ret |= cti_cs4223e_reg_write(phy,
			CS4223_PP_LINE_SDS_DSP_MSEQ_SPARE4_LSB(line),
			0x0000);
	ret |= cti_cs4223e_reg_write(phy,
			CS4223_PP_LINE_SDS_DSP_MSEQ_SPARE4_MSB(line),
			0x0000);

	ret |= cti_cs4223e_reg_write(phy,
			CS4223_PP_LINE_SDS_DSP_MSEQ_SPARE26_MSB(line),
			0x0000);
	ret |= cti_cs4223e_reg_write(phy,
			CS4223_PP_HOST_SDS_DSP_MSEQ_SPARE26_MSB(line),
			0x1f);


	ret |= cti_cs4223e_reg_write(phy,
			CS4223_PP_LINE_SDS_DSP_MSEQ_SPARE27_LSB(line),
			0x4b5); //timeout for 10g
	ret |= cti_cs4223e_reg_write(phy,
			CS4223_PP_LINE_SDS_DSP_MSEQ_SPARE27_MSB(line),
			0x6c); //1G LFI timeout

	ret |= cti_cs4223e_reg_write(phy,
			CS4223_PP_LINE_AN_TX_LINK_FAIL_TIMER_THRES1(line),
			0x449);

	ret |= cti_cs4223e_reg_write(phy,
			CS4223_PP_LINE_AN_TX_LINK_FAIL_TIMER_THRES0(line),
			0x0000);

	ret |= cti_cs4223e_reg_write(phy,
			CS4223_PP_LINE_SDS_DSP_MSEQ_SPARE28_MSB(line),
			1 << (28-16));

	ret |= cti_cs4223e_reg_write(phy,
			CS4223_PP_LINE_SDS_DSP_MSEQ_SPARE28_LSB(line),
			0x0000); 


	//setup time to EDC converge in DFE mode, line and host
	ret |= cti_cs4223e_reg_write(phy,
			CS4223_PP_LINE_SDS_DSP_MSEQ_SPARE6_MSB(line),
			0x0180);

	ret |= cti_cs4223e_reg_write(phy,
			CS4223_PP_HOST_SDS_DSP_MSEQ_SPARE6_MSB(line),
			0x240);

	ret |= cti_cs4223e_reg_write(phy,
			CS4223_PP_LINE_TP_TX_MAX_WAIT_TIMEOUT1(line),
			0x99a);
	ret |= cti_cs4223e_reg_write(phy,
			CS4223_PP_LINE_TP_TX_MAX_WAIT_TIMEOUT0(line),
			0xb10d);

	ret |= cti_cs4223e_reg_write(phy,
			CS4223_PP_LINE_TP_TX_MAX_WAIT_TIMEOUT1(line),
			0x99a);

	ret |= cti_cs4223e_reg_write(phy,
			CS4223_PP_LINE_TP_TX_WAIT_TIMEOUT(line),
			200);

	ret |= cti_cs4223e_reg_write(phy,
			CS4223_PP_LINE_AN_TX_TECH_ABILITY_1(line),0x0005); //force 1000BASE-KX | 10GBASE-KR
	
	ret |= cti_cs4223e_reg_write(phy,
			CS4223_PP_LINE_AN_TX_ADV_ABILITY(line),
			(1 << 5) | 1);


	ret |= cti_cs4223e_reg_write(phy,
			CS4223_PP_LINE_TP_TX_PRESET_COEFF_OVERRIDE(line),0x0001);

	//Tell host-side micro seq, what traffic rate, bit0 = 1.2500 gbaud, bit1 = 10.3125, bit2 3.1250 gbaud
	ret |= cti_cs4223e_reg_write(phy,
			CS4223_PP_HOST_SDS_DSP_MSEQ_SPARE25_LSB(line),0x0002); 	

	ret |= cti_cs4223e_reg_write(phy,
			CS4223_PP_LINE_LINEMISC_CLKEN(line),0x1010);
	ret |= cti_cs4223e_reg_write(phy,
			CS4223_PP_LINE_SDS_DSP_MSEQ_POWER_DOWN_LSB(line),
			0x00e0);
	ret |= cti_cs4223e_reg_write(phy,
			CS4223_PP_LINE_SDS_DSP_MSEQ_POWER_DOWN_LSB(line),
			0x0000);
	ret |= cti_cs4223e_reg_write(phy,
			CS4223_PP_LINE_LINEMISC_MSEQCLKCTRL(line),
			0x0004);
	ret |= cti_cs4223e_reg_write(phy,
			CS4223_PP_HOST_HOSTMISC_CLKEN(line),
			0x0000);
	ret |= cti_cs4223e_reg_write(phy,
			CS4223_PP_HOST_SDS_DSP_MSEQ_POWER_DOWN_LSB(line),
			0x01f7);
	ret |= cti_cs4223e_reg_write(phy,
			CS4223_PP_HOST_SDS_DSP_MSEQ_POWER_DOWN_LSB(line),
			0x01e0);

	ret |= cti_cs4223e_reg_write(phy,
			CS4223_PP_HOST_HOSTMISC_MSEQCLKCTRL(line),0x0004);

	//cti_cs4223e_mseq_stall(phy,0,line,false);
	//cti_cs4223e_mseq_stall(phy,1,line,false);
	//start autoneg
	
	ret |= cti_cs4223e_reg_write(phy,
			CS4223_PP_LINE_SDS_DSP_MSEQ_SPARE26_LSB(line),
			0x0001); //AN trigger

	mdelay(100);
	return ret;	

}



/*
 * TODO: Understand more about what exactly is going on here
 */

__maybe_unused static int cti_cs4223e_hard_reset(struct cti_cs4223e* phy){
	int ret;
	u32 stride;
	int i;

	ret = cti_cs4223e_wait_eeprom_finished(phy,2500,1);

	
	ret = cti_cs4223e_reg_write(phy,CS4223_GLOBAL_MDIO_CFG,CS4223_GLOBAL_MDIO_CFG_PREF);
	
	for(stride = 0; stride < 0x4000; stride += 0x1000){
		ret = cti_cs4223e_reg_write(phy,CS4223_PP_LINE_LINEMISC_MSEQCLKCTRL(0) + stride,0x8004);
		ret = cti_cs4223e_reg_write(phy,CS4223_PP_LINE_SDS_DSP_MSEQ_OPTIONS(0) + stride,0x0008);
		ret = cti_cs4223e_reg_write(phy,CS4223_PP_HOST_HOSTMISC_MSEQCLKCTRL(0) + stride,0x8004);
		ret = cti_cs4223e_reg_write(phy,CS4223_PP_HOST_SDS_DSP_MSEQ_OPTIONS(0) + stride,0x0008);
	}

	ret = cti_cs4223e_reg_write(phy,CS4223_GLOBAL_BROADCAST,0x80);

	ret = cti_cs4223e_reg_write(phy,CS4223_PP_LINE_LINEMISC_CLKEN(0),0xffff);
	ret = cti_cs4223e_reg_write(phy,CS4223_PP_LINE_SDS_DSP_MSEQ_POWER_DOWN_LSB(0),0x0000);
	ret = cti_cs4223e_reg_write(phy,CS4223_PP_LINE_LINEMISC_SOFT_RESET(0),0x0001);
	ret = cti_cs4223e_reg_write(phy,CS4223_PP_LINE_LINEMISC_MPIF_RESET_DOTREG(0),0x0007);
	ret = cti_cs4223e_reg_write(phy,CS4223_PP_LINE_LINEMISC_SOFT_RESET(0),0x0000);
	ret = cti_cs4223e_reg_write(phy,CS4223_PP_LINE_LINEMISC_CLKEN(0),0x0000);

	ret = cti_cs4223e_reg_write(phy,CS4223_GLOBAL_BROADCAST,0x0000);


	ret = cti_cs4223e_reg_write(phy,0x5000,0x01);

	ret = cti_cs4223e_wait_eeprom_finished(phy,2500,1);

	for(i = 0; i < 2; i++){
		ret = cti_cs4223e_reg_write(phy,CS4223_PP_LINE_AN_TX_TECH_ABILITY_1(i),0x0005);
		ret = cti_cs4223e_reg_write(phy,CS4223_PP_LINE_AN_TX_RESET(i),0x0000);
		ret = cti_cs4223e_reg_write(phy,CS4223_PP_LINE_AN_TX_ENABLE(i),1);
		msleep(200);

				
	}


	ret = cti_cs4223e_reg_write(phy,CS4223_GLOBAL_BROADCAST,0x0080);

	ret = cti_cs4223e_reg_write(phy,CS4223_PP_LINE_SDS_COMMON_STX0_SQUELCH(0), 1);
	ret = cti_cs4223e_reg_write(phy,CS4223_PP_LINE_SDS_COMMON_STX0_SQUELCH(1), 1);

	//ret = cti_cs4223e_reg_write(phy,CS4223_PP_LINE_SDS_COMMON_SRX0_RX_CONFIG,0x0024);

	ret = cti_cs4223e_reg_write(phy,CS4223_GLOBAL_BROADCAST,0x0000);


	dev_info(phy->dev,"Hard reset done.\n");
	msleep(1000);
	return ret;
}
static int cti_cs4223e_probe(struct i2c_client* client,const struct i2c_device_id* id){
	struct cti_cs4223e* phy;
	u16 read;
	int ret;
	int i;
	__maybe_unused int timeout = 5;
	phy = devm_kzalloc(&client->dev,sizeof(struct cti_cs4223e),GFP_KERNEL);
	if(!phy)
		return -ENOMEM;

	phy->client = client;
	phy->dev = &client->dev;
	phy->addr = client->addr;

	dev_set_drvdata(phy->dev,phy);

	i2c_set_clientdata(client,phy);


	ret = cti_cs4223e_setup_irq(phy);

	if(ret < 0){
		dev_warn(phy->dev,"Couldn't setup irq!\n");
	}

	ret = cti_cs4223e_hard_reset(phy);
	msleep(1000);

	ret = cti_cs4223e_reg_read(phy,0x0000,&read);
	
	if(ret < 0)
		return ret;
	
	phy->chip_id = read;
	
	ret = cti_cs4223e_reg_read(phy,0x0001,&read);
	phy->chip_id |= (((u32)read) << 16);

	dev_info(phy->dev,"Chip id: %.8x\n",phy->chip_id);

	ret = cti_cs4223e_reg_read(phy,0x0003,&phy->ucode_ver_sr);
	ret = cti_cs4223e_reg_read(phy,0x0004,&phy->ucode_ver_cx1);
	ret = cti_cs4223e_reg_read(phy,0x0005,&phy->ucode_ver_kr);

	dev_info(phy->dev,"Ucode version SR: %.4x CX1: %.4x KR: %.4x\n",
			phy->ucode_ver_sr,phy->ucode_ver_cx1,phy->ucode_ver_kr);


	
	ret = cti_cs4223e_reg_read(phy,0x0023,&read); //read scratch 

	if(!(read & 0x2)){
		dev_info(phy->dev,"Hard reset hasn't been applied yet!\n");
	}
	
	//We must configure the INTERR gpio to be active low, with a 10k pad pullup
	//We must also set its port to common and its function to GLOBAL_INT
	ret = cti_cs4223e_reg_write(phy,CS4223_GPIO_INTERR,0x0721); //active low
	ret = cti_cs4223e_reg_write(phy,CS4223_GPIO_INTERR_OUTPUT_CFG,0x0086);
	//turn on global interrupts
	ret = cti_cs4223e_reg_write(phy,CS4223_GLOBAL_INTE,0x0037);
	//turn on datapath interrupts
	ret = cti_cs4223e_reg_write(phy,CS4223_GLOBAL_DATAPATH_INTE,0x00ff);	

	//enable interrupts	
	for(i=0;i<2;i++){
		
		ret = cti_cs4223e_reg_write(phy,
				CS4223_PP_LINE_SDS_COMMON_RXLOCKED0_INTEN(i),0xff);

		ret = cti_cs4223e_reg_write(phy,
				CS4223_PP_LINE_SDS_COMMON_INTEN(i),0xc0f);

		ret = cti_cs4223e_reg_write(phy,
				CS4223_PP_LINE_LINEMISC_AN_MODE_SEL(i),0x00);


		ret = cti_cs4223e_reg_write(phy,
				CS4223_PP_LINE_GIGEPCS_INT_LINE_PCS1GBE_INTEN(i),0x001f);

	      	ret = cti_cs4223e_reg_read(phy,
				CS4223_PP_LINE_GIGEPCS_INT_LINE_PCS1GBE_INT(i),&read);
		ret = cti_cs4223e_reg_write(phy,
				CS4223_PP_LINE_GIGEPCS_INT_LINE_PCS1GBE_INT(i),read);
		ret = cti_cs4223e_reg_write(phy,
				CS4223_PP_LINE_AN_TX_AN_COMPLETE_STATUS_INTE(i),0x3);
		//0x3f3d
		ret = cti_cs4223e_reg_write(phy,CS4223_PP_LINE_LINEMISC_INTEN(i),0x3f3d);

		ret = cti_cs4223e_reg_write(phy,CS4223_PP_HOST_SDS_COMMON_RXLOCKD0_INTEN(i)
				,0xff);

		ret = cti_cs4223e_reg_write(phy,CS4223_PP_LINE_AN_TX_MAIN_INTE(i),0x20);

		ret = cti_cs4223e_reg_write(phy,CS4223_PP_LINE_AN_TX_AN_COMPLETE_STATUS_INTE(i)
				,0x0001);

	}

	
	for(i=0;i<2;i++){
		
		msleep(250);
		dev_info(phy->dev,"=====Slice %d=====\n",i);
		ret = cti_cs4223e_kran_status(phy,i);
	}
	

	return 0;
}

static int cti_cs4223e_remove(struct i2c_client* client){
	return 0;
}

static void cti_cs4223e_shutdown(struct i2c_client* client){
	return;
}

static const struct i2c_device_id cti_cs4223e_id[] = {
	{"cti_cs4223e",0},
	{},
};

MODULE_DEVICE_TABLE(i2c,cti_cs4223e_id);

const struct of_device_id cti_cs4223e_of_match[] = {
	{.compatible = "cti,cs4223e",},
	{},
};

MODULE_DEVICE_TABLE(of,cti_cs4223e_of_match);

static struct i2c_driver cti_cs4223e_i2c_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "cti_cs4223e",
		.of_match_table = cti_cs4223e_of_match,
	},
	.probe = cti_cs4223e_probe,
	.remove = cti_cs4223e_remove,
	.shutdown = cti_cs4223e_shutdown,
	.id_table = cti_cs4223e_id
};

module_i2c_driver(cti_cs4223e_i2c_driver);

MODULE_DESCRIPTION("Inphi CS4223e KR PHY Driver");
MODULE_AUTHOR("Liam Ewasko <lewasko@connecttech.com");
MODULE_AUTHOR("Connect Tech Inc");
MODULE_LICENSE("GPL v2");
