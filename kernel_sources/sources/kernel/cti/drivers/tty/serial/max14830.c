/*
 * Maxim 14830 Quad Serial UART 128-word FIFO
 * Copyright (C) 2024 Connect Tech Inc.
 *
 * Based on max310x.c, this is an i2c version of it for Kernel-5.10
 *
 * Author: Liam Ewasko
 *
 */

#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/gpio/driver.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/serial_core.h>
#include <linux/serial.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/uaccess.h>

#define MAX14830_MAJOR 204
//#define MAX14830_MAJOR 0
//#define MAX14830_MINOR 0
#define MAX14830_MINOR 209
#define MAX14830_UART_NRMAX 16

#define MAX14830_RHR_REG 		(0x00)
#define MAX14830_THR_REG 		(0x00)
#define MAX14830_IRQEN_REG 		(0x01)
#define MAX14830_IRQSTS_REG		(0x02)
#define MAX14830_LSR_IRQEN_REG  	(0x03)
#define MAX14830_LSR_IRQSTS_REG 	(0x04)
#define MAX14830_REG_05			(0x05)
#define MAX14830_SPCHR_IRQEN_REG 	(MAX14830_REG_05)
#define MAX14830_SPCHR_IRQSTS_REG	(0x06)
#define MAX14830_STS_IRQEN_REG		(0x07)
#define MAX14830_STS_IRQSTS_REG		(0x08)
#define MAX14830_MODE1_REG		(0x09)
#define MAX14830_MODE2_REG		(0x0A)
#define MAX14830_LCR_REG 		(0x0B)
#define MAX14830_RXTO_REG		(0x0C)
#define MAX14830_HDPIXDELAY_REG		(0x0D)
#define MAX14830_IRDA_REG		(0x0E)
#define MAX14830_FLOWLVL_REG		(0x0F)
#define MAX14830_FIFOTRIGLVL_REG	(0x10)
#define MAX14830_TXFIFOLVL_REG		(0x11)
#define MAX14830_RXFIFOLVL_REG		(0x12)
#define MAX14830_FLOWCTRL_REG		(0x13)
#define MAX14830_XON1_REG		(0x14)
#define MAX14830_XON2_REG		(0x15)
#define MAX14830_XOFF1_REG		(0x16)
#define MAX14830_XOFF2_REG		(0x17)

#define MAX14830_GPIOCFG_REG		(0x18)
#define MAX14830_GPIODATA_REG		(0x19)
#define MAX14830_PLLCFG_REG		(0x1A)
#define MAX14830_BRGCFG_REG		(0x1B)
#define MAX14830_BRGDIVLSB_REG		(0x1C)
#define MAX14830_BRGDIVMSB_REG		(0x1D)
#define MAX14830_CLKSRC_REG		(0x1E)


#define MAX14830_REG_1F 0x1F

#define MAX14830_REVID_REG 0x1F
#define MAX14830_GLOBALIRQ_REG 0x1F
#define MAX14830_GLOBALCMD_REG 0x1F

#define MAX14830_REVID_EXTREG (0x25)

#define MAX14830_IRQ_LSR_BIT (1 << 0)
#define MAX14830_IRQ_SPCHR_BIT (1 << 1)
#define MAX14830_IRQ_STS_BIT (1 << 2)
#define MAX14830_IRQ_RXFIFO_BIT (1 << 3)
#define MAX14830_IRQ_TXFIFO_BIT (1 << 4)
#define MAX14830_IRQ_TXEMPTY_BIT (1 << 5)
#define MAX14830_IRQ_RXEMPTY_BIT (1 << 6)
#define MAX14830_IRQ_CTS_BIT (1 << 7)

#define MAX14830_LSR_RXTO_BIT ( 1 << 0)
#define MAX14830_LSR_RXOVR_BIT (1 << 1)
#define MAX14830_LSR_RXPAR_BIT (1 << 2)
#define MAX14830_LSR_FRERR_BIT (1 << 3)
#define MAX14830_LSR_RXBRK_BIT (1 << 4)
#define MAX14830_LSR_RXNOISE_BIT (1 << 5)
#define MAX14830_LSR_CTS_BIT (1 << 7)

#define MAX14830_SPCHR_XON1_BIT (1 << 0)
#define MAX14830_SPCHR_XON2_BIT (1 << 1)
#define MAX14830_SPCHR_XOFF1_BIT (1 << 2)
#define MAX14830_SPCHR_XOFF2_BIT (1 << 3)
#define MAX14830_SPCHR_BREAK_BIT (1 << 4)
#define MAX14830_SPCHR_MULTIDROP_BIT (1 << 5)

#define MAX14830_STS_GPIO0_BIT (1 << 0)
#define MAX14830_STS_GPIO1_BIT (1 << 1)
#define MAX14830_STS_GPIO2_BIT (1 << 2)
#define MAX14830_STS_GPIO3_BIT (1 << 3)
#define MAX14830_STS_CLKREADY_BIT (1 << 5)
#define MAX14830_STS_SLEEP_BIT (1 << 6)

#define MAX14830_MODE1_RXDIS_BIT (1 << 0)
#define MAX14830_MODE1_TXDIS_BIT (1 << 1)
#define MAX14830_MODE1_TXHIZ_BIT (1 << 2)
#define MAX14830_MODE1_RTSHIZ_BIT (1 << 3)
#define MAX14830_MODE1_TRNSCVCTRL_BIT (1 << 4)
#define MAX14830_MODE1_FORCESLEEP_BIT (1 << 5)
#define MAX14830_MODE1_AUTOSLEEP_BIT (1 << 6)
#define MAX14830_MODE1_IRQSEL_BIT (1 << 7)

#define MAX14830_MODE2_RST_BIT	(1 << 0)
#define MAX14830_MODE2_FIFORST_BIT (1 << 1)
#define MAX14830_MODE2_RXTRIGINV_BIT (1 << 2)
#define MAX14830_MODE2_RXEMPTINV_BIT (1 << 3)
#define MAX14830_MODE2_SPCHR_BIT (1 << 4)
#define MAX14830_MODE2_LOOPBACK_BIT (1 << 5)
#define MAX14830_MODE2_MULTIDROP_BIT (1 << 6)
#define MAX14830_MODE2_ECHOSUPR_BIT (1 << 7)

#define MAX14830_LCR_LENGTH0_BIT (1 << 0)
#define MAX14830_LCR_LENGTH1_BIT (1 << 1)
#define MAX14830_LCR_STOPLEN_BIT (1 << 2)
#define MAX14830_LCR_PARITY_BIT (1 << 3)
#define MAX14830_LCR_EVENPARITY_BIT (1 << 4)
#define MAX14830_LCR_FORCEPARITY_BIT (1 << 5)
#define MAX14830_LCR_TXBREAK_BIT (1 << 6)
#define MAX14830_LCR_RTS_BIT (1 << 7)

#define MAX14830_IRDA_IRDAEN_BIT (1 << 0)
#define MAX14830_IRDA_SIR_BIT (1 << 1)

#define MAX14830_FLOWLVL_HALT_MASK (0x000f)
#define MAX14830_FLOWLVL_RES_MASK (0x00f0)
#define MAX14830_FLOWLVL_HALT(words) ((words / 8) & 0xf)
#define MAX14830_FLOWLVL_RES(words) (((words/8) & 0x0f) << 4)

#define MAX14830_FIFOTRIGLVL_TX_MASK (0x0f)
#define MAX14830_FIFOTRIGLVL_RX_MASK (0xf0)
#define MAX14830_FIFOTRIGLVL_TX(words) ((words /8 ) & 0xf)
#define MAX14830_FIFOTRIGLVL_RX(words) (((words / 8) & 0xf) << 4)

#define MAX14830_FLOWCTRL_AUTORTS_BIT (1 << 0)
#define MAX14830_FLOWCTRL_AUTOCTS_BIT (1 << 1)
#define MAX14830_FLOWCTRL_GPIADDR_BIT (1 << 2)
#define MAX14830_FLOWCTRL_SWFLOWEN_BIT (1 << 3)
#define MAX14830_FLOWCTRL_SWFLOW0_BIT (1 << 4)
#define MAX14830_FLOWCTRL_SWFLOW1_BIT (1 << 5)
#define MAX14830_FLOWCTRL_SWFLOW2_BIT (1 << 6)
#define MAX14830_FLOWCTRL_SWFLOW3_BIT (1 << 7)

#define MAX14830_PLLCFG_PREDIV_MASK (0x3f)
#define MAX14830_PLLCFG_PLLFACTOR_MASK (0xc0)

#define MAX14830_BRGCFG_2XMODE_BIT (1 << 4)
#define MAX14830_BRGCFG_4XMODE_BIT (1 << 5)

#define MAX14830_CLKSRC_CRYST_BIT (1 << 1)
#define MAX14830_CLKSRC_PLL_BIT (1 << 2)
#define MAX14830_CLKSRC_PLLBYP_BIT (1 << 3)
#define MAX14830_CLKSRC_EXTCLK_BIT (1 << 4)
#define MAX14830_CLKSRC_CLK2RTS_BIT (1 << 7)

#define MAX14830_EXTREG_ENBL 0xCE
#define MAX14830_EXTREG_DSBL 0xCD

#define MAX14830_BRGCFG_CLKDIS_BIT (1 << 6)
#define MAX14830_REV_ID (0xB0)

#define MAX14830_REV_MASK (0xF8)

#define MAX14830_FIFO_SIZE 128

static DECLARE_BITMAP(max14830_lines,MAX14830_UART_NRMAX);

struct max14830_if_cfg {
	int (*extended_reg_enable)(struct device* dev, bool enable);
	unsigned int rev_id_reg;
};

struct max14830_devtype {
	struct slave_addr {
		u8 min;
		u8 max;
	} slave_addr;
	char name[9];
	int nr;
	u8 mode1;
	int (*detect)(struct device* );
	void (*power)(struct uart_port*,int);
};

struct max14830 {
	struct uart_port port;
	struct work_struct tx_work;
	struct work_struct md_work;
	struct work_struct rs_work;
	struct regmap* regmap;

	u8 wr_header;
	u8 rd_header;
	u8 rx_buf[MAX14830_FIFO_SIZE];
};

#define to_max14830_port(_port) \
	container_of(_port, struct max14830, port)

struct max14830_port {
	struct max14830_devtype* devtype;
	struct max14830_if_cfg* if_cfg;
	struct regmap* regmap;
	struct clk* clk;
	struct max14830 p[0];
};

static u8 max14830_port_read(struct uart_port* uport,u8 reg){
	struct max14830* max = to_max14830_port(uport);
	unsigned int val = 0;
	
	regmap_read(max->regmap,reg,&val);
	return val;	
}

static void max14830_port_write(struct uart_port* uport,u8 reg, u8 val){
	struct max14830* max = to_max14830_port(uport);
	regmap_write(max->regmap,reg, val);
}

static void max14830_port_update(struct uart_port* uport, u8 reg,u8 mask,u8 val){
	struct max14830* max = to_max14830_port(uport);
	regmap_update_bits(max->regmap,reg,mask,val);
}

static void max14830_batch_write(struct uart_port* port, u8* txbuf, unsigned int len){
	struct max14830* max = to_max14830_port(port);
	regmap_raw_write(max->regmap,MAX14830_THR_REG,txbuf,len);
}

static void max14830_batch_read(struct uart_port* port,u8* rxbuf,unsigned int len){
	struct max14830* max = to_max14830_port(port);
	regmap_raw_read(max->regmap,MAX14830_RHR_REG,rxbuf,len);
}

static void max14830_power(struct uart_port *port,int on){
	max14830_port_update(port,MAX14830_MODE1_REG, \
			MAX14830_MODE1_FORCESLEEP_BIT,
			on ? 0 : MAX14830_MODE1_FORCESLEEP_BIT);
	if(on)
		msleep(50);
}

static int max14830_detect(struct device* dev){
	struct max14830_port* port = dev_get_drvdata(dev);
	unsigned int val = 0;
	int ret;

	ret = regmap_write(port->regmap, MAX14830_GLOBALCMD_REG,
		       MAX14830_EXTREG_ENBL);

	if(ret)
		return ret;

	regmap_read(port->regmap, MAX14830_REVID_EXTREG,&val);
	regmap_write(port->regmap, MAX14830_GLOBALCMD_REG, MAX14830_EXTREG_DSBL);

	if(((val & MAX14830_REV_MASK) != MAX14830_REV_ID)){
		dev_err(dev, "%s ID 0x%02x does not match\n", port->devtype->name, val);
		return -ENODEV;
	}		

	return 0;
}

static const struct max14830_devtype devtype = {
	.name = "max14830",
	.nr = 4,
	.mode1 = MAX14830_MODE1_IRQSEL_BIT,
	.power = max14830_power,
	.detect = max14830_detect,
	.slave_addr = {
		.min = 0x60,
		.max = 0x6f,
	},
};

static struct uart_driver max14830_uart = {
	.owner = THIS_MODULE,
	.driver_name = "max14830",
	.dev_name = "ttyMAX",
	.major = MAX14830_MAJOR,
	.minor = MAX14830_MINOR,
	.nr = MAX14830_UART_NRMAX,
};

static int max14830_set_baud(struct uart_port* port,int baud){
	unsigned int mode =0, div = 0, frac =0, c = 0, F = 0;

	div = port->uartclk / baud;
	if(div < 8){
		c = 4;
		mode = MAX14830_BRGCFG_4XMODE_BIT;
	}else if(div < 16){
		c = 8;
		mode = MAX14830_BRGCFG_2XMODE_BIT;
	}else{
		c = 16;
	}

	div /= c;
	F = c*baud;

	if(div > 0)
		frac = (16*(port->uartclk % F )) / F;
	else
		div = 1;

	max14830_port_write(port,MAX14830_BRGDIVMSB_REG, div >> 8);
	max14830_port_write(port,MAX14830_BRGDIVLSB_REG, div);
	max14830_port_write(port,MAX14830_BRGCFG_REG, frac | mode);

	return (16* port->uartclk) / (c*(16*div + frac));
}

static int max14830_update_best_err(unsigned long f, long* besterr){
	long err = f % (460800 * 16);

	if((*besterr < 0) || (*besterr > err)){
		*besterr = err;
		return 0;
	}
	return 1;
}

static u32 max14830_set_ref_clk(struct device* dev,struct max14830_port* p,unsigned long freq,bool xtal){
	unsigned int div,clksrc,pllcfg = 0;
	long besterr = -1;
	unsigned long fdiv,fmul,bestfreq = freq;

	max14830_update_best_err(freq,&besterr);

	for(div = 1; (div <= 63) && besterr;div++){
		fdiv = DIV_ROUND_CLOSEST(freq,div);
		fmul = fdiv * 6;
		if((fdiv >= 500000) && (fdiv <= 800000)){
			if(!max14830_update_best_err(fmul,&besterr)){
				pllcfg = (0 << 6) | div;
				bestfreq = fmul;
			}
		}
		fmul = fdiv*48;
		if((fdiv >= 850000) && (fdiv <= 1200000)){
			if(!max14830_update_best_err(fmul,&besterr)){
				pllcfg = (1 << 6) | div;
				bestfreq = fmul;
			}
		}

		fmul = fdiv* 96;
		if((fdiv >= 425000) && (fdiv <= 1000000)){
			if(!max14830_update_best_err(fmul,&besterr)){
				pllcfg = (2 << 6) | div;
				bestfreq = fmul;
			}
		}
		fmul = fdiv * 144;
		if((fdiv >= 390000) && (fdiv <= 667000)){
			if(!max14830_update_best_err(fmul,&besterr)){
				pllcfg = (3 << 6) | div;
				bestfreq = fmul;
			}
		}
		clksrc = MAX14830_CLKSRC_EXTCLK_BIT | (xtal ? MAX14830_CLKSRC_CRYST_BIT : 0);

		if(pllcfg){
			clksrc |= MAX14830_CLKSRC_PLL_BIT;
			regmap_write(p->regmap,MAX14830_PLLCFG_REG,pllcfg);
		}else
			clksrc |= MAX14830_CLKSRC_PLLBYP_BIT;

		regmap_write(p->regmap,MAX14830_CLKSRC_REG,clksrc);

		if(xtal){
			unsigned int val;
			msleep(10);
			regmap_read(p->regmap,MAX14830_STS_IRQSTS_REG,&val);
			if(!(val & MAX14830_STS_CLKREADY_BIT)){
				dev_warn(dev,"clock is not stable yet!\n");
			}
		}
	}
	return bestfreq;
}

static void max14830_start_tx(struct uart_port* port){
	struct max14830* max = to_max14830_port(port);
	schedule_work(&max->tx_work);
}


static void max14830_handle_rx(struct uart_port* port,unsigned int rxlen){
	struct max14830* max = to_max14830_port(port);
	unsigned int sts, ch, flag, i;

	if(port->read_status_mask == MAX14830_LSR_RXOVR_BIT){
		sts = max14830_port_read(port,MAX14830_LSR_IRQSTS_REG);
		max14830_batch_read(port,max->rx_buf,rxlen);

		port->icount.rx += rxlen;

		flag = TTY_NORMAL;
		sts &= port->read_status_mask;

		if(sts & MAX14830_LSR_RXOVR_BIT){
			dev_warn_ratelimited(port->dev,"Hardware RX FIFO overrun\n");
			port->icount.overrun++;
		}

		for(i=0;i<(rxlen-1);i++){
			uart_insert_char(port,sts,0,max->rx_buf[i],flag);
		}

		uart_insert_char(port,sts,MAX14830_LSR_RXOVR_BIT, max->rx_buf[rxlen-1],flag);
	}else{
		if(unlikely(rxlen >= port->fifosize)){
			dev_warn_ratelimited(port->dev,"Possible RX FIFO overrun\n");
			port->icount.buf_overrun++;
			rxlen = port->fifosize;
		}

		while(rxlen--){
			ch = max14830_port_read(port,MAX14830_RHR_REG);
			sts = max14830_port_read(port,MAX14830_LSR_IRQSTS_REG);

			sts &= MAX14830_LSR_RXPAR_BIT | MAX14830_LSR_FRERR_BIT | MAX14830_LSR_RXOVR_BIT | MAX14830_LSR_RXBRK_BIT;

			port->icount.rx++;
			flag = TTY_NORMAL;

			if(unlikely(sts)){
				if(sts & MAX14830_LSR_RXBRK_BIT){
					port->icount.brk++;
					if(uart_handle_break(port))
						continue;
				}else if( sts & MAX14830_LSR_RXPAR_BIT){
					port->icount.parity++;
				}else if(sts & MAX14830_LSR_FRERR_BIT){
					port->icount.frame++;
				}else if(sts & MAX14830_LSR_RXOVR_BIT){
					port->icount.overrun++;
				}

				sts &= port->read_status_mask;

				if(sts & MAX14830_LSR_RXBRK_BIT){
					flag = TTY_BREAK;
				}else if(sts & MAX14830_LSR_RXPAR_BIT){
					flag = TTY_PARITY;
				}else if(sts & MAX14830_LSR_FRERR_BIT){
					flag = TTY_FRAME;
				}else if(sts & MAX14830_LSR_RXOVR_BIT){
					flag = TTY_OVERRUN;
				}
			}
			if(uart_handle_sysrq_char(port,ch))
				continue;

			if(sts & port->ignore_status_mask)
				continue;

			uart_insert_char(port,sts, MAX14830_LSR_RXOVR_BIT,ch,flag);
		}
	}
	tty_flip_buffer_push(&port->state->port);
}

static void max14830_handle_tx(struct uart_port* port){
	struct circ_buf *xmit = &port->state->xmit;
	unsigned int txlen, to_send,until_end;

	if(unlikely(port->x_char)){
		max14830_port_write(port,MAX14830_THR_REG,port->x_char);
		port->icount.tx++;
		port->x_char = 0;
		return;
	}

	if(uart_circ_empty(xmit) || uart_tx_stopped(port))
		return;

	to_send = uart_circ_chars_pending(xmit);
	until_end = CIRC_CNT_TO_END(xmit->head,xmit->tail,UART_XMIT_SIZE);

	if(likely(to_send)){
		txlen = max14830_port_read(port,MAX14830_TXFIFOLVL_REG);
		txlen = port->fifosize - txlen;
		to_send = (to_send > txlen) ? txlen : to_send;

		if(until_end < to_send){
			max14830_batch_write(port,xmit->buf + xmit->tail,until_end);
			max14830_batch_write(port,xmit->buf,to_send - until_end);
		}else{
			max14830_batch_write(port,xmit->buf + xmit->tail,to_send);
		}
		port->icount.tx += to_send;
		xmit->tail = (xmit->tail + to_send) & (UART_XMIT_SIZE -1);
	}

	if(uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
		uart_write_wakeup(port);
}

static irqreturn_t max14830_port_irq(struct max14830_port* port,int portno){
	struct uart_port* uport = &port->p[portno].port;
	irqreturn_t res = IRQ_NONE;
	
	do {
		unsigned int ists, lsr,rxlen;
		ists = max14830_port_read(uport,MAX14830_IRQSTS_REG);
		rxlen = max14830_port_read(uport,MAX14830_RXFIFOLVL_REG);
		
		if(!ists && !rxlen)
			break;
		res = IRQ_HANDLED;
		dev_dbg(uport->dev, "port%d: IRQ Status: %.2x Rxlen: %d\n",portno,ists,rxlen);

		if(ists & MAX14830_IRQ_CTS_BIT){
			lsr = max14830_port_read(uport,MAX14830_LSR_IRQSTS_REG);
			uart_handle_cts_change(uport,!!(lsr & MAX14830_LSR_CTS_BIT));
		}

		if(rxlen){
			max14830_handle_rx(uport,rxlen);
		}
		if(ists & MAX14830_IRQ_TXEMPTY_BIT){
			max14830_start_tx(uport);
		}
	}while(1);

	return res;
}

static irqreturn_t max14830_ist(int irq,void* dev_id){
	struct max14830_port* port = (struct max14830_port*)dev_id;
	bool handled = false;
	if(port->devtype->nr > 1){
		do {
			unsigned int val = ~0;
			WARN_ON_ONCE(regmap_read(port->regmap, MAX14830_GLOBALIRQ_REG, &val));
			val = ((1 << port->devtype->nr) -1 ) & ~val;
			
			if(!val)
				break;
			
			if(max14830_port_irq(port,fls(val) - 1) == IRQ_HANDLED)
				handled = true;
		}while(1);
	}else{
		if(max14830_port_irq(port,0) == IRQ_HANDLED)
			handled = true;
	}

	return IRQ_RETVAL(handled);
}

static void max14830_tx_proc(struct work_struct* ws){
	struct max14830* max = container_of(ws,struct max14830,tx_work);
	max14830_handle_tx(&max->port);
}

static void max14830_md_proc(struct work_struct* ws){
	struct max14830* max = container_of(ws,struct max14830,md_work);
	max14830_port_update(&max->port,MAX14830_MODE2_REG, MAX14830_MODE2_LOOPBACK_BIT,
			(max->port.mctrl & TIOCM_LOOP) ? MAX14830_MODE2_LOOPBACK_BIT : 0);
}

static void max14830_rs_proc(struct work_struct* ws){
	struct max14830* max = container_of(ws,struct max14830,rs_work);
	unsigned int delay,mode1 = 0, mode2 = 0;

	delay = (max->port.rs485.delay_rts_before_send << 4) |
		max->port.rs485.delay_rts_after_send;
	max14830_port_write(&max->port,MAX14830_HDPIXDELAY_REG,delay);

	if(max->port.rs485.flags & SER_RS485_ENABLED){
		mode1 = MAX14830_MODE1_TRNSCVCTRL_BIT;
		if(!(max->port.rs485.flags & SER_RS485_RX_DURING_TX)){
			mode2 = MAX14830_MODE2_ECHOSUPR_BIT;
		}
	}

	max14830_port_update(&max->port, MAX14830_MODE1_REG,
			MAX14830_MODE1_TRNSCVCTRL_BIT, mode1);
	max14830_port_update(&max->port, MAX14830_MODE2_REG,
			MAX14830_MODE2_ECHOSUPR_BIT,mode2);
}

static unsigned int max14830_tx_empty(struct uart_port* port){
	u8 lvl = max14830_port_read(port,MAX14830_TXFIFOLVL_REG);
	return lvl ? 0 : TIOCSER_TEMT;
}

static unsigned int max14830_get_mctrl(struct uart_port* port){
	return TIOCM_DSR | TIOCM_CAR;
}


static void  max14830_set_mctrl(struct uart_port* port,unsigned int mctrl){
	struct max14830* max = to_max14830_port(port);
	schedule_work(&max->md_work);
}

static void max14830_break_ctl(struct uart_port* port,int break_state){
	max14830_port_update(port,MAX14830_LCR_REG,MAX14830_LCR_TXBREAK_BIT,
			break_state ? MAX14830_LCR_TXBREAK_BIT : 0);
}

static void max14830_set_termios(struct uart_port* port,struct ktermios* termios,struct ktermios* old){
	unsigned int lcr = 0, flow = 0;
	int baud;

	termios->c_cflag &= ~CMSPAR;

	switch(termios->c_cflag & CSIZE){

		case CS5:
			break;
		case CS6:
			lcr = MAX14830_LCR_LENGTH0_BIT;
			break;
		case CS7:
			lcr = MAX14830_LCR_LENGTH1_BIT;
			break;
		case CS8:
		default:
			lcr = MAX14830_LCR_LENGTH1_BIT | MAX14830_LCR_LENGTH0_BIT;
			break;
	}

	if(termios->c_cflag & PARENB){
		lcr |= MAX14830_LCR_PARITY_BIT;
		if(!(termios->c_cflag & PARODD))
			lcr |= MAX14830_LCR_EVENPARITY_BIT;
	}

	if(termios->c_cflag & CSTOPB){
		lcr |= MAX14830_LCR_STOPLEN_BIT; //2 stops
	}
	
	max14830_port_write(port,MAX14830_LCR_REG,lcr);

	port->read_status_mask = MAX14830_LSR_RXOVR_BIT;

	if(termios->c_iflag & INPCK){
		port->read_status_mask |= MAX14830_LSR_RXPAR_BIT |
					MAX14830_LSR_FRERR_BIT;

	}

	if(termios->c_iflag & (IGNBRK | BRKINT | PARMRK))
		port->read_status_mask |= MAX14830_LSR_RXBRK_BIT;

	port->ignore_status_mask = 0;

	if(termios->c_iflag & IGNBRK)
		port->ignore_status_mask |= MAX14830_LSR_RXBRK_BIT;
	if(!(termios->c_cflag & CREAD)){
		port->ignore_status_mask |= MAX14830_LSR_RXPAR_BIT |
					MAX14830_LSR_RXOVR_BIT |
					MAX14830_LSR_FRERR_BIT |
					MAX14830_LSR_RXBRK_BIT;
	}

	max14830_port_write(port,MAX14830_XON1_REG,termios->c_cc[VSTART]);
	max14830_port_write(port,MAX14830_XOFF1_REG,termios->c_cc[VSTOP]);

	if(termios->c_cflag & CRTSCTS || termios->c_iflag & IXOFF){
		max14830_port_update(port,MAX14830_MODE1_REG,MAX14830_MODE1_TXDIS_BIT,MAX14830_MODE1_TXDIS_BIT);
	}

	port->status &= ~(UPSTAT_AUTOCTS | UPSTAT_AUTORTS | UPSTAT_AUTOXOFF);

	if(termios->c_cflag & CRTSCTS){
		port->status |= UPSTAT_AUTOCTS | UPSTAT_AUTORTS;
		flow |= MAX14830_FLOWCTRL_AUTOCTS_BIT | MAX14830_FLOWCTRL_AUTORTS_BIT;
	}

	if(termios->c_iflag & IXON){
		flow |= MAX14830_FLOWCTRL_SWFLOW3_BIT | MAX14830_FLOWCTRL_SWFLOWEN_BIT;
	}

	if(termios->c_iflag & IXOFF){
		port->status |= UPSTAT_AUTOXOFF;
		flow |= MAX14830_FLOWCTRL_SWFLOW1_BIT | MAX14830_FLOWCTRL_SWFLOWEN_BIT;
	}
	max14830_port_write(port,MAX14830_FLOWCTRL_REG,flow);

	if(!(termios->c_cflag & CRTSCTS) && (termios->c_iflag & IXOFF)){
		max14830_port_update(port,MAX14830_MODE1_REG,MAX14830_MODE1_TXDIS_BIT,0);
	}

	baud = uart_get_baud_rate(port,termios,old,port->uartclk/16 / 0xffff,
			port->uartclk/4);

	baud = max14830_set_baud(port,baud);

	uart_update_timeout(port,termios->c_cflag,baud);

}

static int max14830_startup(struct uart_port* port){
	struct max14830_port* p = dev_get_drvdata(port->dev);
	unsigned int val;

	p->devtype->power(port,1);
	dev_dbg(port->dev,"Started port %lu\n",port->iobase);
	//setup mode1
	max14830_port_update(port,MAX14830_MODE1_REG,MAX14830_MODE1_TRNSCVCTRL_BIT,0);

	//setup mode2, reset fifos
	val = MAX14830_MODE2_RXEMPTINV_BIT | MAX14830_MODE2_FIFORST_BIT;
	max14830_port_write(port,MAX14830_MODE2_REG,val);
	max14830_port_update(port,MAX14830_MODE2_REG,MAX14830_MODE2_FIFORST_BIT,0);

	val = (clamp(port->rs485.delay_rts_before_send,0U, 15U) << 4) | clamp(port->rs485.delay_rts_after_send,0U,15U);
	max14830_port_write(port,MAX14830_HDPIXDELAY_REG,val);

	if(port->rs485.flags & SER_RS485_ENABLED){
		max14830_port_update(port,MAX14830_MODE1_REG, \
				MAX14830_MODE1_TRNSCVCTRL_BIT, \
				MAX14830_MODE1_TRNSCVCTRL_BIT);
		if(!(port->rs485.flags & SER_RS485_RX_DURING_TX)){
			max14830_port_update(port,MAX14830_MODE2_REG, \
					MAX14830_MODE2_ECHOSUPR_BIT, \
					MAX14830_MODE2_ECHOSUPR_BIT);
		}
	}
	max14830_port_write(port,MAX14830_FLOWLVL_REG, \
			MAX14830_FLOWLVL_RES(48) | MAX14830_FLOWLVL_HALT(96));

	max14830_port_read(port,MAX14830_IRQSTS_REG);

	val = MAX14830_IRQ_RXEMPTY_BIT | MAX14830_IRQ_TXEMPTY_BIT;
	max14830_port_write(port,MAX14830_IRQEN_REG,val | MAX14830_IRQ_CTS_BIT);
	return 0;
}

static void max14830_shutdown(struct uart_port* port){
	struct max14830_port* p = dev_get_drvdata(port->dev);

	max14830_port_write(port,MAX14830_IRQEN_REG,0); //turn off ints
	p->devtype->power(port,0);
}

static const char* max14830_type(struct uart_port* port){
	struct max14830_port* p = dev_get_drvdata(port->dev);
	return (port->type == PORT_MAX310X) ? p->devtype->name : NULL;
}

static int max14830_verify_port(struct uart_port* port,struct serial_struct* s){
	if((s->type != PORT_UNKNOWN) && (s->type != PORT_MAX310X))
		return -EINVAL;
	if(s->irq != port->irq)
		return -EINVAL;
	return 0;
}

static void max14830_config_port(struct uart_port* port,int flags){
	if(flags & UART_CONFIG_TYPE)
		port->type = PORT_MAX310X;
}

static void max14830_dummy_op(struct uart_port* port){
	
}

static int max14830_request_port(struct uart_port* port){
	return 0;
}

static int max14830_rs485_config(struct uart_port* port,struct serial_rs485 *rs485){
	struct max14830* max = to_max14830_port(port);

	if((rs485->delay_rts_before_send > 0x0f) || (rs485->delay_rts_after_send > 0x0f)){
		return -ERANGE;
	}

	rs485->flags &= SER_RS485_RTS_ON_SEND | SER_RS485_RX_DURING_TX | SER_RS485_ENABLED;
	memset(rs485->padding,0,sizeof(rs485->padding));
	port->rs485 = *rs485;
	schedule_work(&max->rs_work);
	return 0;
}

static const struct uart_ops max14830_ops = {
	.tx_empty = max14830_tx_empty,
	.set_mctrl = max14830_set_mctrl,
	.get_mctrl = max14830_get_mctrl,
	.set_termios = max14830_set_termios,
	.break_ctl = max14830_break_ctl,
	.stop_tx = max14830_dummy_op,
	.start_tx = max14830_start_tx,
	.stop_rx = max14830_dummy_op,
	.release_port = max14830_dummy_op,
	.request_port = max14830_request_port,
	.verify_port = max14830_verify_port,
	.config_port = max14830_config_port,
	.type = max14830_type,
	.shutdown = max14830_shutdown,
	.startup = max14830_startup,
};

static int max14830_probe(struct device* dev, const struct max14830_devtype* devtype, 
		struct max14830_if_cfg* if_cfg,
		struct regmap *regmaps[],int irq){

	int i,ret,fmin,fmax,freq;
	struct max14830_port* port;

	u32 uartclk = 0;
	bool xtal;

	for(i=0;i< devtype->nr;i++){
		if(IS_ERR(regmaps[i]))
			return PTR_ERR(regmaps[i]);
	}

	port = devm_kzalloc(dev,struct_size(port,p,devtype->nr),GFP_KERNEL);
	if(!port){
		dev_err(dev,"Error allocating port struct!\n");
		return -ENOMEM;
	}

	device_property_read_u32(dev, "clock-frequency",&uartclk);

	xtal = device_property_match_string(dev,"clock-names","osc") < 0;

	if(xtal){
		port->clk = devm_clk_get_optional(dev, "xtal");
	}else{
		port->clk = devm_clk_get_optional(dev,"osc");
	}

	dev_info(dev,"Clock frequency = %d Type: %s\n",uartclk,xtal ? "XTAL" : "OSC");

	if(IS_ERR(port->clk))
		return PTR_ERR(port->clk);

	ret = clk_prepare_enable(port->clk);
	if(ret)
		return ret;

	freq = clk_get_rate(port->clk);

	if(!freq)
		freq = uartclk;
	if(!freq){
		dev_err(dev, "Unable to source clock rate\n");
		ret = -EINVAL;
		goto exit_clk;
	}


	if(xtal){
		fmin = 1000000;
		fmax = 4000000;
	}else {
		fmin = 500000;
		fmax = 35000000;
	}

	if(freq < fmin || freq > fmax){
		ret = -ERANGE;
		goto exit_clk;
	}

	port->regmap = regmaps[0];
	port->devtype = devtype;
	port->if_cfg = if_cfg;
	dev_set_drvdata(dev,port);

	ret = devtype->detect(dev);
	if(ret)
		goto exit_clk;

	for(i=0;i<devtype->nr;i++){
		//reset port
		regmap_write(regmaps[i], MAX14830_MODE2_REG, MAX14830_MODE2_RST_BIT);
		//clear reset
		regmap_write(regmaps[i], MAX14830_MODE2_REG, 0);
		do { //spin for port startup
			regmap_read(regmaps[i],MAX14830_BRGDIVLSB_REG,&ret);
		}while(ret != 0x01);
		regmap_write(regmaps[i],MAX14830_MODE1_REG,devtype->mode1);

	}


	uartclk = max14830_set_ref_clk(dev,port,freq,xtal);
	dev_dbg(dev,"Reference clock set to %i Hz\n",uartclk);

	for(i=0;i< devtype->nr;i++){
		unsigned int line;
		struct uart_port* up;
		line = find_first_zero_bit(max14830_lines,MAX14830_UART_NRMAX);
		if(line == MAX14830_UART_NRMAX){
			ret = -ERANGE;
			goto exit_uart;
		}
		up = &port->p[i].port;

		up->line = line;
		up->dev = dev;
		up->irq = irq;
		up->type = PORT_MAX310X;
		up->fifosize = MAX14830_FIFO_SIZE;
	       	up->flags = UPF_FIXED_TYPE | UPF_LOW_LATENCY;
		up->iotype = UPIO_PORT;
		up->iobase = i;
		up->membase = (void __iomem*)~0;
		up->uartclk = uartclk;
		up->ops = &max14830_ops;
		port->p[i].regmap = regmaps[i];
		up->rs485_config = max14830_rs485_config;
		
		//Disable ints
		max14830_port_write(up,MAX14830_IRQEN_REG,0);
		//clear irq status
		max14830_port_read(up,MAX14830_IRQSTS_REG);
		
		INIT_WORK(&port->p[i].tx_work, max14830_tx_proc);
		INIT_WORK(&port->p[i].md_work, max14830_md_proc);
		INIT_WORK(&port->p[i].rs_work, max14830_rs_proc);
		
		ret = uart_add_one_port(&max14830_uart,up);
		if(ret){
			up->dev = NULL;
			goto exit_uart;
		}
		set_bit(line,max14830_lines);

		devtype->power(up,0);
	}

	ret = devm_request_threaded_irq(dev,irq,NULL, max14830_ist,IRQF_ONESHOT | IRQF_SHARED, dev_name(dev),port);

	if(!ret){
		dev_info(dev,"Probe successful\n");
		return 0;
	}

	dev_err(dev, "Unable to request IRQ %i\n",irq);

exit_uart:
	for(i=0; i < devtype->nr;i++){
		if(port->p[i].port.dev){
			uart_remove_one_port(&max14830_uart,&port->p[i].port);
			clear_bit(port->p[i].port.line,max14830_lines);
		}
	}

exit_clk:
	clk_disable_unprepare(port->clk);

	return ret;
}

static void max14830_remove(struct device* dev){
	struct max14830_port* port = dev_get_drvdata(dev);
	int i;

	for(i=0; i < port->devtype->nr;i++){
		cancel_work_sync(&port->p[i].tx_work);
		cancel_work_sync(&port->p[i].md_work);
		cancel_work_sync(&port->p[i].rs_work);
		uart_remove_one_port(&max14830_uart,&port->p[i].port);
		clear_bit(port->p[i].port.line, max14830_lines);
		port->devtype->power(&port->p[i].port,0);
	}

	clk_disable_unprepare(port->clk);
}

static const struct of_device_id max14830_dt_ids[] = {
	{ .compatible ="cti,max14830", .data = &devtype},
	{}
};
MODULE_DEVICE_TABLE(of, max14830_dt_ids);



static int __maybe_unused max14830_suspend(struct device* dev){
	struct max14830_port *p = dev_get_drvdata(dev);
	int i;

	for(i=0;i<p->devtype->nr;i++){
		uart_suspend_port(&max14830_uart,&p->p[i].port);
		p->devtype->power(&p->p[i].port,0);
	}
	return 0;
}

static int __maybe_unused max14830_resume(struct device* dev){
	struct max14830_port* p = dev_get_drvdata(dev);
	int i;

	for(i=0;i<p->devtype->nr;i++){
		p->devtype->power(&p->p[i].port,1);
		uart_resume_port(&max14830_uart,&p->p[i].port);
	}
	return 0;
}

static SIMPLE_DEV_PM_OPS(max14830_pm_ops, max14830_suspend, max14830_resume);


static int max14380_extended_reg_enable(struct device* dev,bool enable){
	return 0;
}


static bool max14830_reg_writeable(struct device* dev,unsigned int reg){
	switch(reg){
		case MAX14830_IRQSTS_REG:
		case MAX14830_LSR_IRQSTS_REG:
		case MAX14830_SPCHR_IRQSTS_REG:
		case MAX14830_STS_IRQSTS_REG:
		case MAX14830_TXFIFOLVL_REG:
		case MAX14830_RXFIFOLVL_REG:
			return false;
		default:
			break;
	}
	return true;
}

static bool max14830_reg_volatile(struct device* dev,unsigned int reg){
	switch(reg){
		case MAX14830_RHR_REG:
		case MAX14830_IRQSTS_REG:
		case MAX14830_LSR_IRQSTS_REG:
		case MAX14830_SPCHR_IRQSTS_REG:
		case MAX14830_STS_IRQSTS_REG:
		case MAX14830_TXFIFOLVL_REG:
		case MAX14830_RXFIFOLVL_REG:
		case MAX14830_GPIODATA_REG:
		case MAX14830_BRGDIVLSB_REG:
		case MAX14830_REG_05:
		case MAX14830_REG_1F:
			return true;
		default:
			break;
	}
	return false;
}

static bool max14830_reg_precious(struct device* dev,unsigned int reg){

	switch(reg){
		case MAX14830_RHR_REG:
		case MAX14830_IRQSTS_REG:
		case MAX14830_SPCHR_IRQSTS_REG:
		case MAX14830_STS_IRQSTS_REG:
			return true;
		default:
			break;
	}

	return false;
}

static bool max14830_reg_noinc(struct device* dev,unsigned int reg){
	return reg == MAX14830_RHR_REG;
}

static struct regmap_config regcfg_i2c = {
	.reg_bits = 8,
	.val_bits = 8,
	.cache_type = REGCACHE_RBTREE,
	.writeable_reg = max14830_reg_writeable,
	.volatile_reg = max14830_reg_volatile,
	.precious_reg = max14830_reg_precious,
	.max_register = MAX14830_REVID_EXTREG,
	.writeable_noinc_reg = max14830_reg_noinc,
	.readable_noinc_reg = max14830_reg_noinc,
	//.max_raw_read = MAX14830_FIFO_SIZE,
	//.max_raw_write = MAX14830_FIFO_SIZE,
};

static struct max14830_if_cfg max14830_i2c_if_cfg = {
	.extended_reg_enable = max14380_extended_reg_enable,
	.rev_id_reg = MAX14830_REVID_EXTREG
};

static unsigned short max14830_i2c_slave_addr(unsigned short addr,unsigned int nr){
	addr -= nr * 0x10;
	if(nr >= 2)
		addr -= 0x20;
	return addr;
}

static int max14830_i2c_probe(struct i2c_client* client){
	
	struct max14830_devtype *devtype = device_get_match_data(&client->dev);
	struct i2c_client* port_client;

	struct regmap* regmaps[4];
	unsigned int i;
	u8 port_addr;

	dev_info(&client->dev,"Probing max14830...\n");
	if(client->addr < devtype->slave_addr.min || client->addr > devtype->slave_addr.max){
		return dev_err_probe(&client->dev, -EINVAL, 
				"Slave addr 0x%x outside of range (0x%x, 0x%x)\n",
				client->addr, devtype->slave_addr.min,
				devtype->slave_addr.max);
	}

	regmaps[0] = devm_regmap_init_i2c(client, &regcfg_i2c);

	for(i=1; i < devtype->nr;i++){
		port_addr = max14830_i2c_slave_addr(client->addr, i);
		port_client = devm_i2c_new_dummy_device(&client->dev, client->adapter, port_addr);
		port_client->irq = client->irq;
		regmaps[i] = devm_regmap_init_i2c(port_client, &regcfg_i2c);
	}

	return max14830_probe(&client->dev,devtype, &max14830_i2c_if_cfg,regmaps,client->irq);
}

static int max14830_i2c_remove(struct i2c_client* client){
	max14830_remove(&client->dev);
	return 0;
}

struct i2c_device_id max14830_id[] = {
	{"max14830",0},
	{},
};

static struct i2c_driver max14830_i2c_driver = {
	.driver = {
		.name = "max14830",
		.of_match_table = max14830_dt_ids,
		.pm		= &max14830_pm_ops,
	},
	.probe_new = max14830_i2c_probe,
	.remove = max14830_i2c_remove,
	.id_table = max14830_id,
};

static int __init max14830_uart_init(void){
	int ret; 
	bitmap_zero(max14830_lines,MAX14830_UART_NRMAX);
	ret = 0;
	ret = uart_register_driver(&max14830_uart);
	if(ret){
		printk(KERN_INFO "%s: uart_register_driver failed (%d)\n",__func__,ret);
		return ret;
	}
	ret = i2c_add_driver(&max14830_i2c_driver);
	if(ret){
		printk(KERN_INFO "%s: i2c_add_driver failed (%d)\n",__func__,ret);
		uart_unregister_driver(&max14830_uart);
	}
	return ret;
}

module_init(max14830_uart_init);

static void __exit max14830_uart_exit(void){
	i2c_del_driver(&max14830_i2c_driver);
	uart_unregister_driver(&max14830_uart);
}

module_exit(max14830_uart_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Liam Ewasko");
MODULE_DESCRIPTION("MAX14830 i2c quad uart driver");

