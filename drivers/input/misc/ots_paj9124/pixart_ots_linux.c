
#include <linux/input.h>	/* BUS_SPI */
#include <linux/pm.h>
#include <linux/spi/spi.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/irq.h> 
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include  <linux/delay.h>  
#include "pixart_ots.h"

struct pixart_ots_s {	
	struct device		*dev;
	struct input_dev	*input; 	
	int		irq_gpio; 
	int		irq; 
	u32	irq_flags;
	u8 sysfs_read_addr ;
	u32 last_jiffies;
}; 

#define MAX_SPI_FREQ_HZ     2000000
#define WRITECMD(reg) (reg|0x80)
#define READCMD(reg)  (reg)
#define DS_FACTOR 1

struct device		*g_dev = NULL;
static int pixart_ots_read(struct device *dev, u8 reg);
static int pixart_ots_write(struct device *dev, u8 reg, u8 val);
static int pixart_ots_read_cs_delay_us(struct device *dev, u8 reg, u16 delay_usecs);
extern unsigned char ReadData(unsigned char addr)
{
	if(g_dev != NULL)
	{
		return (unsigned char)pixart_ots_read(g_dev, addr);
	}
	
	return 0 ;
}
extern void WriteData(unsigned char addr, unsigned char data)
{
	if(g_dev != NULL)
	{
		pixart_ots_write(g_dev, addr, data);
	}
}
extern void delay_ms(int ms)
{
	mdelay(ms);
}

extern void nCS_Low_1ms(void)
{
	if(g_dev != NULL)
	{
		int dummy_read ;
		printk(">>> %s (%d) \n", __func__, __LINE__);
		dummy_read = pixart_ots_read_cs_delay_us(g_dev, 0, 1000 /*1ms*/);
		printk("<<< %s (%d) \n", __func__, __LINE__);
	}
}
#ifdef CONFIG_PM_SLEEP
static int pixart_ots_suspend(struct device *dev)
{
//	struct spi_device *spi = to_spi_device(dev);
//	struct pixart_ots_s *ots = spi_get_drvdata(spi);
	printk(">>> %s (%d) \n", __func__, __LINE__);

	return 0;
}

static int pixart_ots_resume(struct device *dev)
{
//	struct spi_device *spi = to_spi_device(dev);
//	struct pixart_ots_s *ots = spi_get_drvdata(spi);
	printk(">>> %s (%d) \n", __func__, __LINE__);

	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(pixart_ots_pm, pixart_ots_suspend, pixart_ots_resume);
static int pixart_spi_read_cs_delay_us(struct spi_device *spi, u8 addr, u8 *data, u16 delay_usecs)
{
	int retval;
	unsigned char txbuf[2];
	unsigned char rxbuf[2];
	struct spi_message msg;
	struct spi_transfer xfers ;

	spi_message_init(&msg);

	memset(&xfers, 0, sizeof(xfers));
	txbuf[0] = addr ;
	txbuf[1] = 0 ;

	xfers.tx_buf = txbuf;
	xfers.rx_buf = rxbuf;
	xfers.delay_usecs = delay_usecs ;
	xfers.len = 2;
	spi_message_add_tail(&xfers, &msg);
	retval = spi_sync(spi, &msg);
	if (retval == 0) { 
		*data = rxbuf[1] ; 
	} else {
		printk("%s: Failed to complete SPI transfer, error = %d\n",
				__func__, retval);
	}

	return retval;
}

static int pixart_spi_read(struct spi_device *spi, u8 addr, u8 *data)
{
	int retval;
	unsigned char txbuf[2];
	unsigned char rxbuf[2];
	struct spi_message msg;
	struct spi_transfer xfers ;

	spi_message_init(&msg);

	memset(&xfers, 0, sizeof(xfers));
	txbuf[0] = addr ;
	txbuf[1] = 0 ;

	xfers.tx_buf = txbuf;
	xfers.rx_buf = rxbuf;
	xfers.len = 2;
	spi_message_add_tail(&xfers, &msg);
	retval = spi_sync(spi, &msg);
	if (retval == 0) { 
		*data = rxbuf[1] ; 
	} else {
		printk("%s: Failed to complete SPI transfer, error = %d\n",
				__func__, retval);
	}

	return retval;
}

static int pixart_spi_write(struct spi_device *spi, u8 addr, u8 data)
{
	int retval;
	unsigned char txbuf[2];
	struct spi_message msg;
	struct spi_transfer xfers ;

	spi_message_init(&msg);

	memset(&xfers, 0, sizeof(xfers));
	txbuf[0] = addr ;
	txbuf[1] = data ;

	xfers.tx_buf = txbuf;
	xfers.len = 2;

	spi_message_add_tail(&xfers, &msg);
	retval = spi_sync(spi, &msg);
	if (retval == 0) { 
	} else {
		printk("%s: Failed to complete SPI transfer, error = %d\n",
				__func__, retval);
	}

	return retval;
}
static int pixart_ots_read(struct device *dev, u8 reg)
{
	struct spi_device *spi = to_spi_device(dev);
	u8 cmd;
	u8 data = 0;
	int ret ;

	cmd = READCMD(reg);
	ret = pixart_spi_read(spi, cmd, &data);//spi_w8r8(spi, cmd) ;
	
	if(ret != 0)
		return ret ;
	else
		return data;
}
static int pixart_ots_read_cs_delay_us(struct device *dev, u8 reg, u16 delay_usecs)
{
	struct spi_device *spi = to_spi_device(dev);
	u8 cmd;
	u8 data = 0;
	int ret ;

	cmd = READCMD(reg);
	ret = pixart_spi_read_cs_delay_us(spi, cmd, &data, delay_usecs);//spi_w8r8(spi, cmd) ;
	
	if(ret != 0)
		return ret ;
	else
		return data;
}
static int pixart_ots_write(struct device *dev, u8 reg, u8 val)
{
	struct spi_device *spi = to_spi_device(dev);
	unsigned char buf[2];

	buf[0] = WRITECMD(reg);
	buf[1] = val;

	return pixart_spi_write(spi, buf[0], buf[1]); // spi_write(spi, buf, sizeof(buf));
}
/*
static int pixart_ots_write_read(struct device *dev, u8 reg, u8 val)
{
	int rdata;
	int ret ;
	ret = pixart_ots_write(dev, reg, val);
	
	if((ret >=0) && (reg != 0x7f) )
	{
		rdata = pixart_ots_read(dev, reg) ;
		if(rdata != val)
		{
			ret = (-1);
			printk("%s (%d) : pixart_ots_write_read error, addr 0x%x, data 0x%x\n", __func__, __LINE__, reg, rdata);
		}
	}
	
	return ret;
}
*/
static ssize_t write_reg_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	char s[256];  
	char *p = s ;
	struct spi_device *spi = to_spi_device(dev);
	struct pixart_ots_s *ots = spi_get_drvdata(spi);
	
	printk("%s (%d) : write_reg_store\n", __func__, __LINE__);

	memcpy(s, buf, size);
	
	*(s+1)='\0';
	*(s+4)='\0';
	*(s+7)='\0';
	//example(in console): echo w 12 34 > rw_reg
	if(*p == 'w')
	{
		long write_addr, write_data ;

		p += 2;
		if(!kstrtol(p, 16, &write_addr))
		{
			p += 3 ;
			if(!kstrtol(p, 16, &write_data))
			{		
				printk("w 0x%x 0x%x\n", (u8)write_addr, (u8)write_data);
				pixart_ots_write(dev, (u8)write_addr, (u8)write_data);
			}
		}
	}
	//example(in console): echo r 12 > rw_reg
	else if(*p == 'r')
	{
		long read_addr ;
		p+=2;
		
		if(!kstrtol(p, 16, &read_addr))
		{
			int data = 0;
			ots->sysfs_read_addr = read_addr;
			data = pixart_ots_read(dev, (u8)read_addr);
			printk("r 0x%x 0x%x\n", (unsigned int)read_addr, data);
		}
	}	
	else if(*p == 'c')
	{
		nCS_Low_1ms();
	}
	else if(*p == 't')
	{
		int i=0; 
		for(i=0; i<1000000 ; i++)
		{
			if(pixart_ots_read(dev, 0) != 0x30)
			{
				printk("Check ID fail %d\n", i);
				break;
			}
		}
	}
	return size;
}
 
static ssize_t read_reg_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	
	//cat rw_reg
	int ret = -1;
	char *s= buf;  
	struct spi_device *spi = to_spi_device(dev);
	struct pixart_ots_s *ots = spi_get_drvdata(spi);
	
	printk("%s (%d) : read_reg_show\n", __func__, __LINE__);
	ret = pixart_ots_read(dev, ots->sysfs_read_addr); 
	
	if(ret < 0)
		s += sprintf(s,"Error\n");  
	else
  	s += sprintf(s,"Addr 0x%x, Data 0x%x\n",ots->sysfs_read_addr, ret);  		
	
	return (s - buf); 
}

//static DEVICE_ATTR(rw_reg, S_IRUGO | S_IWUSR | S_IWGRP, read_reg_show, write_reg_store);


static DEVICE_ATTR(rw_reg, 0666, read_reg_show, write_reg_store);



static struct attribute *rw_reg_sysfs_attrs[] = {
	&dev_attr_rw_reg.attr,
	NULL
};


static struct attribute_group rw_reg_attribute_group = {
	.attrs = rw_reg_sysfs_attrs,
};


static void time_interval_check(struct pixart_ots_s *ots)
{
	#define TIME_INTERVAL_RESET_TH 2000 //2 sec

	int interval_ms ;	
	if(jiffies >= ots->last_jiffies)
		interval_ms = jiffies_to_msecs(jiffies - ots->last_jiffies);
	else
		//jiffies overflow
		interval_ms = jiffies_to_msecs( 0xffffffff - ots->last_jiffies + jiffies);
	
	ots->last_jiffies = jiffies ;
	if(interval_ms > TIME_INTERVAL_RESET_TH)
	{
		printk("Reset Variables %d ms\n", interval_ms);
		OTS_Reset_Variables();
	}

}
static irqreturn_t pixart_ots_irq(int irq, void *handle)
{
//"cat /proc/kmsg" to see kernel message

//	int8_t dx=0, dy=0;
	int16_t deltaX, deltaY;
	//unsigned char OutBtnState;
	unsigned char OutOtsState;
	struct pixart_ots_s *ots = handle; 

	time_interval_check(ots);
//	printk(">>> %s (%d) \n", __func__, __LINE__);
	OTS_Sensor_ReadMotion(&deltaX,&deltaY);
	//printk("delta(%d, %d)\n", deltaX, deltaY);
	OutOtsState = OTS_Detect_Rotation(deltaX,deltaY);
	//OutBtnState = OTS_Detect_Pressing(deltaX,deltaY); 
	//printk("status(%d, %d)\n", OutOtsState, OutBtnState);

	if(OutOtsState == OTS_ROT_UP)		
	{
		//Right Key
		input_event(ots->input, EV_KEY, KEY_RIGHT, 1);
		input_sync(ots->input); 
		input_event(ots->input, EV_KEY, KEY_RIGHT, 0);
		input_sync(ots->input); 
		printk("Right\n");
	}
	else if(OutOtsState == OTS_ROT_DOWN)
	{
		//Left Key
		input_event(ots->input, EV_KEY, KEY_LEFT, 1);
		input_sync(ots->input); 
		input_event(ots->input, EV_KEY, KEY_LEFT, 0);
		input_sync(ots->input); 
		printk("Left\n");
	}
	/*
	if(	OutBtnState == OTS_BTN_RELEASE )
	{
		input_event(ots->input, EV_KEY, KEY_ENTER, 1);
		input_sync(ots->input); 
		input_event(ots->input, EV_KEY, KEY_ENTER, 0);
		input_sync(ots->input); 
		printk("Enter Released\n");		
	}
	*/
	return IRQ_HANDLED;
}

static int pixart_ots_open(struct input_dev *dev)
{
	struct pixart_ots_s *ots = input_get_drvdata(dev);
	int err = 0; 	
	printk(">>> %s (%d) \n", __func__, __LINE__);

	ots->last_jiffies = jiffies ;
	err = request_threaded_irq(ots->irq, NULL, pixart_ots_irq,
				   ots->irq_flags,
				   "pixart_ots_irq", //dev_name(ots->dev), 
				   ots);
	if (err) {
		printk("irq %d busy?\n", ots->irq);
	} 	
	return err;	
}

static void pixart_ots_close(struct input_dev *dev)
{
	printk(">>> %s (%d) \n", __func__, __LINE__);
} 

static int pixart_ots_probe(struct spi_device *spi)
{
	struct device_node *np ; 
	struct pixart_ots_s *ots;
	struct input_dev *input_dev; 
	int err;
	//u8	id ;

	printk(">>> %s (%d) \n", __func__, __LINE__);

	/* don't exceed max specified SPI CLK frequency */
//	if (spi->max_speed_hz > MAX_SPI_FREQ_HZ) {
//		dev_err(&spi->dev, "SPI CLK %d Hz?\n", spi->max_speed_hz);
//		return -EINVAL;
//	} 

	printk("bit_per_word: %d, mode: %d, speed: %d\n", spi->bits_per_word, spi->mode, spi->max_speed_hz);
//	spi->bits_per_word = 8;
//	spi->mode = (SPI_MODE_3 | SPI_CS_HIGH );
	err = spi_setup(spi);
	if (err) {
	        dev_dbg(&spi->dev, "spi master doesn't support 8 bits/word\n");
	        return err;
	}
	ots = kzalloc(sizeof(*ots), GFP_KERNEL); 
	if (!ots)
	{
		err = -ENOMEM;
		goto err_free_mem; 
	}
	
	input_dev = input_allocate_device();
	if (!input_dev) {
		err = -ENOMEM;
		goto err_free_mem; 
	} 
	
	ots->dev = &spi->dev;
	g_dev = &spi->dev;
	
	if(OTS_Sensor_Init() == 0)	
	{
		printk("SPI FAIL!!\n");
		err = -1;
//		goto err_free_mem;
	}
	
	np = ots->dev->of_node	;
	ots->irq_gpio = of_get_named_gpio_flags(np,
			"pixart_ots,irq-gpio", 0, &ots->irq_flags);  
	
	printk("irq_gpio: %d, irq_flags: 0x%x\n",ots->irq_gpio, ots->irq_flags);			
	
	if (!gpio_is_valid(ots->irq_gpio)){
		printk("invalid irq_gpio: %d\n",ots->irq_gpio);
		err = -1;
		goto err_free_mem;
	} 
	
	/* configure touchscreen irq gpio */
	if((err = gpio_request(ots->irq_gpio, "pixart_ots_irq_gpio")))
	{
		printk("unable to request gpio [%d], [%d]\n",ots->irq_gpio,err);
		err = -1;
		goto err_free_mem;
	} 

	if((err = gpio_direction_input(ots->irq_gpio)))
	{
		printk("unable to set dir for gpio[%d], [%d]\n",ots->irq_gpio, err);
		err = -1;
		goto err_free_mem;
	}
	
	ots->irq = gpio_to_irq(ots->irq_gpio);
	ots->input = input_dev;
	input_set_drvdata(input_dev, ots);

	__set_bit(EV_KEY, input_dev->evbit);
	__set_bit(KEY_LEFT, input_dev->keybit);  
	__set_bit(KEY_RIGHT, input_dev->keybit);  
	__set_bit(KEY_ENTER, input_dev->keybit);  
	
	input_dev->name = "pixart_ots";	
	input_dev->open = pixart_ots_open;
	input_dev->close = pixart_ots_close;	
	
	err = input_register_device(input_dev);
	if (err) {
		printk("Unable to register input device, error: %d\n", err);
		goto err_free_irq;
	}
   
  ///sys/bus/spi/drivers/pixart_ots/spi8.0
  ots->sysfs_read_addr = 0 ;
 	err = sysfs_create_group(&ots->dev->kobj, &rw_reg_attribute_group);
 	if(err)
 	{
 		kobject_put(&ots->dev->kobj);
 		printk("Unable to create group %d\n", err);
		goto err_free_irq;
	} 
	spi_set_drvdata(spi, ots);
	
	return 0;
	
err_free_irq:
	free_irq(ots->irq, ots); 
err_free_mem:
	input_free_device(input_dev);
	kfree(ots);
	return err;
}

static int pixart_ots_remove(struct spi_device *spi)
{
	struct pixart_ots_s *ots = spi_get_drvdata(spi);
	if (!ots)
	{
		sysfs_remove_group(&ots->dev->kobj, &rw_reg_attribute_group); 
		free_irq(ots->irq, ots);
		input_unregister_device(ots->input); 
		kfree(ots);
	}
	spi_set_drvdata(spi, NULL);

	return 0;
}
static struct of_device_id pixart_ots_match_table[] = {
	{ .compatible = "pixart,ots",},
	{ },
}; 
static struct spi_driver pixart_ots_driver = {
	.driver = {
		.name	= "pixart_ots",
		.bus	= &spi_bus_type,
		.owner	= THIS_MODULE,
		.pm	= &pixart_ots_pm,
		.of_match_table = pixart_ots_match_table, 
	},
	.probe		= pixart_ots_probe,
	.remove		= pixart_ots_remove,
};

static int __init pixart_ots_init(void)
{
	printk(">>> %s (%d) \n", __func__, __LINE__);
	return spi_register_driver(&pixart_ots_driver);
}
module_init(pixart_ots_init);

static void __exit pixart_ots_exit(void)
{
	printk(">>> %s (%d) \n", __func__, __LINE__);
	spi_unregister_driver(&pixart_ots_driver);
}
module_exit(pixart_ots_exit);

MODULE_AUTHOR("Pixart");
MODULE_DESCRIPTION("Pixart OTS driver");
MODULE_LICENSE("GPL");
