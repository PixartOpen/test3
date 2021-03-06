#ifndef _PIXART_PLATFORM_
#define _PIXART_PLATFORM_

#include <linux/input.h>	/* BUS_SPI */
#include <linux/pm.h>
#include <linux/spi/spi.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/irq.h> 
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include  <linux/delay.h>  
#include  <linux/types.h>

/* extern functions */
extern unsigned char ReadData(unsigned char addr);
extern void WriteData(unsigned char addr, unsigned char data);
extern void delay_ms(int ms);
extern void nCS_Low_1ms(void);

//define downscale factor for rotation of shaft  
#define EXPECTED_COUNT_PER_ROUND 	360
#define REAL_AVG_COUNT_PER_ROUND 	446	//base on: sensor Reg0x0d=0x07, shaft diameter=2mm, sensor-to-shaft distance=2mm
#define debug_print printk
#endif