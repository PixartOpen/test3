
#include <linux/kernel.h>
#include <linux/input.h>
#include <linux/pm.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/irq.h> 
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/miscdevice.h>

#include "pixart_ots.h"
#include "pixart_platform.h"

static int pat9125_init_input_data(void);

#define pat9125_name "pixart_pat9125"

#define pat9125_DEV_NAME     pat9125_name
//#define MTK_PLATFORM
#ifdef MTK_PLATFORM
#include <cust_eint.h>
#include <mach/mt_gpio.h>
#define pat9125_I2C_ADDRESS	0x15
static struct i2c_board_info __initdata i2c_pixart={ I2C_BOARD_INFO(pat9125_DEV_NAME, pat9125_I2C_ADDRESS)};
static u8 *MTKI2CBuf_va = NULL;
static u64 MTKI2CBuf_pa = NULL; 	
#define I2C_DMA_MAX_TRANSACTION_LENGTH 255
#endif

typedef struct {
	struct i2c_client	*client;
	struct input_dev *pat9125_input_dev;	
	struct device		*i2c_dev;	
	int		irq_gpio; 
	int		irq; 	
	u32	irq_flags;
	struct device *pat9125_device; 
#ifdef MTK_PLATFORM	
	struct work_struct  eint_work;
#endif	
	u32 last_jiffies;
}pat9125_linux_data_t;


static pat9125_linux_data_t pat9125data;

static int pat9125_i2c_write(u8 reg, u8 *data, int len);
static int pat9125_i2c_read(u8 reg, u8 *data);

/**************************************/

extern unsigned char ReadData(unsigned char addr)
{
	u8 data = 0xff;
	pat9125_i2c_read(addr, &data);
	return data ;
}
extern void WriteData(unsigned char addr, unsigned char data)
{
	pat9125_i2c_write(addr, &data, 1);
}
extern void delay_ms(int ms)
{
	mdelay(ms);
}
/**************************************/
static int pat9125_i2c_write(u8 reg, u8 *data, int len)
{
	u8  buf[20];
	int rc;
	int ret = 0;
	int i;

	buf[0] = reg;
	if (len >= 20) {
		printk("%s (%d) : FAILED: buffer size is limitted(20) %d\n", __func__, __LINE__, len);
		dev_err(&pat9125data.client->dev, "pat9125_i2c_write FAILED: buffer size is limitted(20)\n");
		return -1;
	}

	for( i=0 ; i<len; i++ ) {
		buf[i+1] = data[i];
	}
 
	rc = i2c_master_send(pat9125data.client, buf, len+1);  // Returns negative errno, or else the number of bytes written.

	if (rc != len+1) {
		printk("%s (%d) : FAILED: writing to reg 0x%x\n", __func__, __LINE__, reg);

		ret = -1;
	}

	return ret;
}

static int pat9125_i2c_read(u8 reg, u8 *data)
{   

	u8  buf[20];
	int rc;		
	
	buf[0] = reg;
	
	rc = i2c_master_send(pat9125data.client, buf, 1);  //If everything went ok (i.e. 1 msg transmitted), return #bytes  transmitted, else error code.   thus if transmit is ok  return value 1
	if (rc != 1) {
		printk("%s (%d) : FAILED: writing to address 0x%x\n", __func__, __LINE__, reg);
		return -1;
	}	
	
	rc = i2c_master_recv(pat9125data.client, buf, 1);   // returns negative errno, or else the number of bytes read
	if (rc != 1) {
		printk("%s (%d) : FAILED: reading data\n", __func__, __LINE__);
		return -1;
	}	
		
	*data = buf[0] ;		
	return 0;
}

static void time_interval_check(pat9125_linux_data_t *ots)
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

void pixart_pat9125_ist(void) 
{
//"cat /proc/kmsg" to see kernel message

	int16_t deltaX, deltaY;
	unsigned char OutBtnState;
	unsigned char OutOtsState;
	printk(">>> %s (%d) \n", __func__, __LINE__);  

	time_interval_check(&pat9125data);
	OTS_Sensor_ReadMotion(&deltaX,&deltaY);
	//printk("delta(%d, %d)\n", deltaX, deltaY);
	OutOtsState = OTS_Detect_Rotation(deltaX,deltaY);
	OutBtnState = OTS_Detect_Pressing(deltaX,deltaY); 
	//printk("status(%d, %d)\n", OutOtsState, OutBtnState);

	if(OutOtsState == OTS_ROT_UP)		
	{
		//Right Key
		input_event(pat9125data.pat9125_input_dev, EV_KEY, KEY_RIGHT, 1);
		input_sync(pat9125data.pat9125_input_dev); 
		input_event(pat9125data.pat9125_input_dev, EV_KEY, KEY_RIGHT, 0);
		input_sync(pat9125data.pat9125_input_dev); 
		printk("Right\n");
	}
	else if(OutOtsState == OTS_ROT_DOWN)
	{
		//Left Key
		input_event(pat9125data.pat9125_input_dev, EV_KEY, KEY_LEFT, 1);
		input_sync(pat9125data.pat9125_input_dev); 
		input_event(pat9125data.pat9125_input_dev, EV_KEY, KEY_LEFT, 0);
		input_sync(pat9125data.pat9125_input_dev); 
		printk("Left\n");
	}
	
	if(	OutBtnState == OTS_BTN_RELEASE )
	{
		input_event(pat9125data.pat9125_input_dev, EV_KEY, KEY_ENTER, 1);
		input_sync(pat9125data.pat9125_input_dev); 
		input_event(pat9125data.pat9125_input_dev, EV_KEY, KEY_ENTER, 0);
		input_sync(pat9125data.pat9125_input_dev); 
		printk("Enter Released\n");		
	}		
	
}

#ifdef MTK_PLATFORM

extern void mt_eint_mask(unsigned int eint_num);
extern void mt_eint_unmask(unsigned int eint_num);
extern void mt_eint_set_hw_debounce(unsigned int eint_num, unsigned int ms);
extern void mt_eint_set_polarity(unsigned int eint_num, unsigned int pol);
extern unsigned int mt_eint_set_sens(unsigned int eint_num, unsigned int sens);
extern void mt_eint_registration(unsigned int eint_num, unsigned int flow, void (EINT_FUNC_PTR)(void), unsigned int is_auto_umask); 

static void pixart_eint_work(struct work_struct *work)
{	
	printk("<<< %s (%d)\n", __func__, __LINE__);
	pixart_pat9125_ist();
} 

static void pixart_eint_func(void)
{
	printk(">>> %s (%d) \n", __func__, __LINE__);  
	schedule_work(&pat9125data.eint_work);
} 

static int pixart_setup_eint(void)
{
	int level; 
	static u8 init = 0 ;
	
	if(init) return 0 ;
	mt_set_gpio_mode(GPIO_ALS_EINT_PIN, GPIO_ALS_EINT_PIN_M_EINT);
	//mt_set_gpio_mode(GPIO_ALS_EINT_PIN, GPIO_ALS_EINT_PIN_M_MDEINT);

	mt_set_gpio_dir(GPIO_ALS_EINT_PIN, GPIO_DIR_IN);
	level = mt_get_gpio_in(GPIO_ALS_EINT_PIN); 
	printk(">>> %s (%d) INT %d\n", __func__, __LINE__, level); 
	mt_set_gpio_pull_enable(GPIO_ALS_EINT_PIN, TRUE);
	mt_set_gpio_pull_select(GPIO_ALS_EINT_PIN, GPIO_PULL_DOWN);
	//mt_eint_set_hw_debounce(CUST_EINT_ALS_NUM, CUST_EINT_ALS_DEBOUNCE_CN);
	mt_eint_registration(CUST_EINT_ALS_NUM, CUST_EINTF_TRIGGER_RISING, pixart_eint_func, true);
	mt_eint_mask(CUST_EINT_ALS_NUM);  
	INIT_WORK(&pat9125data.eint_work, pixart_eint_work);	
	mt_eint_unmask(CUST_EINT_ALS_NUM);  
	init = 1 ;
  return 0;
}

#else

static irqreturn_t pixart_pat9125_irq(int irq, void *handle)
{
//"cat /proc/kmsg" to see kernel message
	pixart_pat9125_ist();
	return IRQ_HANDLED;
}

#endif

static int pat9125_start(void)
{
	int err = (-1); 	
	printk(">>> %s (%d) \n", __func__, __LINE__);

#ifdef MTK_PLATFORM		
	err = pixart_setup_eint();
#else
	err = request_threaded_irq(pat9125data.irq, NULL, pixart_pat9125_irq,
				   pat9125data.irq_flags,
				   "pixart_pat9125_irq", 
				   &pat9125data);
	if (err) {
		printk("irq %d busy?\n", pat9125data.irq);
	} 	
#endif
	pat9125data.last_jiffies = jiffies ;
	
	return err;	
}

static void pat9125_stop(void)
{
	printk(">>> %s (%d) \n", __func__, __LINE__);
#ifdef MTK_PLATFORM
	//TODO
#else
  free_irq(pat9125data.irq, &pat9125data);
#endif 
}

static ssize_t pat9125_fops_read(struct file *filp,char *buf,size_t count,loff_t *l)
{
	printk(">>> %s (%d) \n", __func__, __LINE__);			
	return 0;
}

static ssize_t pat9125_fops_write(struct file *filp,const char *buf,size_t count,loff_t *f_ops)
{
	printk(">>> %s (%d) \n", __func__, __LINE__); 
	return 0;
}

static long pat9125_fops_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
//static int pat9125_fops_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
	printk(">>> %s (%d) \n", __func__, __LINE__);	
	return 0;
}

static int pat9125_fops_open(struct inode *inode, struct file *filp)
{
	printk(">>> %s (%d) \n", __func__, __LINE__);	
	return 0;
}

static int pat9125_fops_release(struct inode *inode, struct file *filp)
{
	printk(">>> %s (%d) \n", __func__, __LINE__);
	return 0;
}
static struct file_operations pat9125_fops = 
{
	owner	:	THIS_MODULE,
	read	:	pat9125_fops_read,
	write	:	pat9125_fops_write,
	//ioctl	:	pat9125_fops_ioctl,
	unlocked_ioctl	:	pat9125_fops_ioctl,
	open	:	pat9125_fops_open,
	release	:	pat9125_fops_release,
};

/*----------------------------------------------------------------------------*/
struct miscdevice pat9125_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = pat9125_name,
	.fops = &pat9125_fops,
};
static ssize_t pat9125_test_store(struct device* dev, 
                                   struct device_attribute *attr, const char *buf, size_t count)
{
	char s[256];  
	char *p = s ;
	
	printk("%s (%d) : write_reg_store\n", __func__, __LINE__);

	memcpy(s, buf, sizeof(s));
	
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
				WriteData((u8)write_addr, (u8)write_data);
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
			data = ReadData((u8)read_addr) ;
			printk("r 0x%x 0x%x\n", (unsigned int)read_addr, data);
		}
	}	
	return count;
}                       

static ssize_t pat9125_test_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	
	//cat 	
	printk("%s (%d) : \n", __func__, __LINE__);
	
	return 0; 
}
static DEVICE_ATTR(test, S_IRUGO | S_IWUGO , pat9125_test_show, pat9125_test_store);
static struct device_attribute *pat9125_attr_list[] =
{
	&dev_attr_test,
};


/*----------------------------------------------------------------------------*/
static int pat9125_create_attr(struct device *dev) 
{
	int idx, err = 0;
	int num = (int)(sizeof(pat9125_attr_list)/sizeof(pat9125_attr_list[0]));
	if(!dev)
	{
		return -EINVAL;
	}	

	for(idx = 0; idx < num; idx++)
	{
		if((err = device_create_file(dev, pat9125_attr_list[idx])))
		{            
			printk("device_create_file (%s) = %d\n", pat9125_attr_list[idx]->attr.name, err);        
			break;
		}
	}

	return err;
}
/*----------------------------------------------------------------------------*/
static int pat9125_delete_attr(struct device *dev)
{
	
	int idx ,err = 0;
	int num = (int)(sizeof(pat9125_attr_list)/sizeof(pat9125_attr_list[0]));
	if (!dev)
	{
		return -EINVAL;
	}
	

	for (idx = 0; idx < num; idx++)
	{
		device_remove_file(dev, pat9125_attr_list[idx]);
	}	

	return err;
}

static int pat9125_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int err = 0;
#ifndef MTK_PLATFORM		
	struct device_node *np ; 
#endif	
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);

	printk("%s (%d) : probe module....\n", __func__, __LINE__);
	
	memset (&pat9125data, 0, sizeof(pat9125data));
	err = i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE);	
	if(err < 0){
		goto error_return;
	}	

	pat9125data.client = client;
	if((err = misc_register(&pat9125_device)))
	{
		printk("pat9125_device register failed\n");
		goto error_return;
	}
	
	pat9125data.pat9125_device = pat9125_device.this_device;
	if( (err = pat9125_create_attr(pat9125data.pat9125_device)) )
	{
		printk("create attribute err = %d\n", err);
		goto error_return;
	} 
	
	if( pat9125_init_input_data() < 0){
     goto error_return;
	}
			
	//interrupt initialization
	pat9125data.i2c_dev = &client->dev;
#ifndef MTK_PLATFORM		
	np = pat9125data.i2c_dev->of_node	;
	pat9125data.irq_gpio = of_get_named_gpio_flags(np,
			"pixart_pat9125,irq-gpio", 0, &pat9125data.irq_flags);  
	
	printk("irq_gpio: %d, irq_flags: 0x%x\n",pat9125data.irq_gpio, pat9125data.irq_flags);			
	
	if (!gpio_is_valid(pat9125data.irq_gpio)){
		err = (-1);
		printk("invalid irq_gpio: %d\n",pat9125data.irq_gpio);
		goto error_return;
	} 
	
	if((err = gpio_request(pat9125data.irq_gpio, "pixart_pat9125_irq_gpio")))
	{
		printk("unable to request gpio [%d], [%d]\n",pat9125data.irq_gpio,err);
		goto error_return;
	} 

	if((err = gpio_direction_input(pat9125data.irq_gpio)))
	{
		printk("unable to set dir for gpio[%d], [%d]\n",pat9125data.irq_gpio, err);
		goto error_return;
	}
	
	pat9125data.irq = gpio_to_irq(pat9125data.irq_gpio);
#endif

	if( !OTS_Sensor_Init() )
		goto error_return;
		
	if( !pat9125_start() )
			goto error_return;

	return 0;
			
error_return:

	return err;	

}


static int pat9125_i2c_remove(struct i2c_client *client)
{

	return 0;
}

static int pat9125_suspend(struct device *dev)
{   printk("%s (%d) : pat9125 suspend \n", __func__, __LINE__);
	return 0;
}

static int pat9125_resume(struct device *dev)
{
	printk("%s (%d) : pat9125 resume \n", __func__, __LINE__);
	return 0;
}

static const struct i2c_device_id pat9125_device_id[] = {
	{pat9125_DEV_NAME, 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, pat9125_device_id);

static const struct dev_pm_ops pat9125_pm_ops = {
	.suspend = pat9125_suspend,
	.resume = pat9125_resume
};

static struct of_device_id pixart_pat9125_match_table[] = {
	{ .compatible = "pixart,pat9125",},
	{ },
}; 

static struct i2c_driver pat9125_i2c_driver = {
	.driver = {
		   .name = pat9125_DEV_NAME,
		   .owner = THIS_MODULE,
		   .pm = &pat9125_pm_ops,
		   .of_match_table = pixart_pat9125_match_table,
		   },
	.probe = pat9125_i2c_probe,
	.remove = pat9125_i2c_remove,
	.id_table = pat9125_device_id,
};
static int pat9125_open(struct input_dev *dev)
{
	printk(">>> %s (%d) \n", __func__, __LINE__);
	return 0 ;
}

static void pat9125_close(struct input_dev *dev)
{
	printk(">>> %s (%d) \n", __func__, __LINE__);
}

static int pat9125_init_input_data(void)
{
	int ret = 0;

	printk("%s (%d) : initialize data\n", __func__, __LINE__);
	
	pat9125data.pat9125_input_dev = input_allocate_device();
	
	if (!pat9125data.pat9125_input_dev) {
		printk("%s (%d) : could not allocate mouse input device\n", __func__, __LINE__);
		return -ENOMEM;
	}

	__set_bit(EV_KEY, pat9125data.pat9125_input_dev->evbit);
	__set_bit(KEY_LEFT, pat9125data.pat9125_input_dev->keybit);  
	__set_bit(KEY_RIGHT, pat9125data.pat9125_input_dev->keybit);  
	__set_bit(KEY_ENTER, pat9125data.pat9125_input_dev->keybit);
	
	input_set_drvdata(pat9125data.pat9125_input_dev, &pat9125data);
	pat9125data.pat9125_input_dev->name = "Pixart pat9125";	

	pat9125data.pat9125_input_dev->open = pat9125_open;
	pat9125data.pat9125_input_dev->close = pat9125_close;
	
	ret = input_register_device(pat9125data.pat9125_input_dev);
	if (ret < 0) {
		input_free_device(pat9125data.pat9125_input_dev);
		printk("%s (%d) : could not register input device\n", __func__, __LINE__);	
		return ret;
	}
	
	return 0;	
}

static int __init pat9125_linux_init(void)
{
	printk("%s (%d) :init module\n", __func__, __LINE__);
	printk("Date : %s\n", __DATE__);
  printk("Time : %s\n", __TIME__);
#ifdef MTK_PLATFORM	
	MTKI2CBuf_va = (u8 *)dma_alloc_coherent(NULL, I2C_DMA_MAX_TRANSACTION_LENGTH, &MTKI2CBuf_pa, GFP_KERNEL); 
	i2c_register_board_info(2, &i2c_pixart, 1); 	
#endif
	return i2c_add_driver(&pat9125_i2c_driver);
}




static void __exit pat9125_linux_exit(void)
{
	printk("%s (%d) : exit module\n", __func__, __LINE__);	
	pat9125_stop();
	misc_register(&pat9125_device);
	pat9125_delete_attr(pat9125data.pat9125_device);
	i2c_del_driver(&pat9125_i2c_driver);
}


module_init(pat9125_linux_init);
module_exit(pat9125_linux_exit);
MODULE_AUTHOR("pixart");
MODULE_DESCRIPTION("pixart pat9125 driver");
MODULE_LICENSE("GPL");
  


