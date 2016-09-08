#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/poll.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/cdev.h>
#include <linux/errno.h>
#include <linux/time.h>
#include "kd_flashlight.h"
#include <asm/io.h>
#include <asm/uaccess.h>
#include "kd_camera_hw.h"
#include <cust_gpio_usage.h>
#include <cust_i2c.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/xlog.h>
#include <linux/version.h>

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,37))
#include <linux/mutex.h>
#else
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,27)
#include <linux/semaphore.h>
#else
#include <asm/semaphore.h>
#endif
#endif

#include <linux/i2c.h>
#include <linux/leds.h>



/******************************************************************************
 * Debug configuration
******************************************************************************/
// availible parameter
// ANDROID_LOG_ASSERT
// ANDROID_LOG_ERROR
// ANDROID_LOG_WARNING
// ANDROID_LOG_INFO
// ANDROID_LOG_DEBUG
// ANDROID_LOG_VERBOSE

#define TAG_NAME "[leds_strobe.c]"
#define PK_DBG_NONE(fmt, arg...)    do {} while (0)
#define PK_DBG_FUNC(fmt, arg...)    pr_debug(TAG_NAME "%s: " fmt, __FUNCTION__ ,##arg)
#define PK_WARN(fmt, arg...)        pr_warning(TAG_NAME "%s: " fmt, __FUNCTION__ ,##arg)
#define PK_NOTICE(fmt, arg...)      pr_notice(TAG_NAME "%s: " fmt, __FUNCTION__ ,##arg)
#define PK_INFO(fmt, arg...)        pr_info(TAG_NAME "%s: " fmt, __FUNCTION__ ,##arg)
#define PK_TRC_FUNC(f)              pr_debug(TAG_NAME "<%s>\n", __FUNCTION__)
#define PK_TRC_VERBOSE(fmt, arg...) pr_debug(TAG_NAME fmt, ##arg)
#define PK_ERROR(fmt, arg...)       pr_err(TAG_NAME "%s: " fmt, __FUNCTION__ ,##arg)


#define DEBUG_LEDS_STROBE
#ifdef  DEBUG_LEDS_STROBE
	#define PK_DBG PK_DBG_FUNC
	#define PK_VER PK_TRC_VERBOSE
	#define PK_ERR PK_ERROR
#else
	#define PK_DBG(a,...)
	#define PK_VER(a,...)
	#define PK_ERR(a,...)
#endif

/******************************************************************************
 * local variables
******************************************************************************/

static DEFINE_SPINLOCK(g_strobeSMPLock); /* cotta-- SMP proection */


static u32 strobe_Res = 0;
static u32 strobe_Timeus = 0;
static BOOL g_strobe_On = 0;

static int g_duty=-1;
static int g_timeOutTimeMs=0;

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,37))
static DEFINE_MUTEX(g_strobeSem);
#else
static DECLARE_MUTEX(g_strobeSem);
#endif


#define STROBE_DEVICE_ID 0xC6


static struct work_struct workTimeOut;

#define GPIO_FLASH_STROBE    	(GPIO93 | 0x80000000)
#define GPIO_FLASH_EN			(GPIO94 | 0x80000000)
#define GPIO_FLASH_TX          		(GPIO95 | 0x80000000)
//#define FLASH_GPIO_ENF GPIO12
//#define FLASH_GPIO_ENT GPIO13
/* registers definitions */
#define REG_ENABLE		0x01
#define REG_FLASH_LED0_BR	0x03
#define REG_FLASH_LED1_BR	0x04
#define REG_TORCH_LED0_BR	0x05
#define REG_TORCH_LED1_BR	0x06
#define REG_FLASH_TOUT		0x08
#define REG_FLAG0		0x0a
#define REG_FLAG1		0x0b
/*****************************************************************************
Functions
*****************************************************************************/
static void work_timeOutFunc(struct work_struct *data);

static struct i2c_client *LM3643_i2c_client = NULL;




struct LM3643_platform_data {
	u8 torch_pin_enable;    // 1:  TX1/TORCH pin isa hardware TORCH enable
	u8 pam_sync_pin_enable; // 1:  TX2 Mode The ENVM/TX2 is a PAM Sync. on input
	u8 thermal_comp_mode_enable;// 1: LEDI/NTC pin in Thermal Comparator Mode
	u8 strobe_pin_disable;  // 1 : STROBE Input disabled
	u8 vout_mode_enable;  // 1 : Voltage Out Mode enable
};

struct LM3643_chip_data {
	struct i2c_client *client;

	//struct led_classdev cdev_flash;
	//struct led_classdev cdev_torch;
	//struct led_classdev cdev_indicator;

	struct LM3643_platform_data *pdata;
	struct mutex lock;

	u8 last_flag;
	u8 no_pdata;
};

/* i2c access*/
/*
static int LM3643_read_reg(struct i2c_client *client, u8 reg,u8 *val)
{
	int ret;
	struct LM3643_chip_data *chip = i2c_get_clientdata(client);

	mutex_lock(&chip->lock);
	ret = i2c_smbus_read_byte_data(client, reg);
	mutex_unlock(&chip->lock);

	if (ret < 0) {
		PK_ERR("failed reading at 0x%02x error %d\n",reg, ret);
		return ret;
	}
	*val = ret&0xff;

	return 0;
}*/

static int LM3643_write_reg(struct i2c_client *client, u8 reg, u8 val)
{
	int ret=0;
	struct LM3643_chip_data *chip = i2c_get_clientdata(client);

	mutex_lock(&chip->lock);
	ret =  i2c_smbus_write_byte_data(client, reg, val);
	mutex_unlock(&chip->lock);

	if (ret < 0)
		PK_ERR("failed writting at 0x%02x\n", reg);
	return ret;
}

static int LM3643_read_reg(struct i2c_client *client, u8 reg)
{
	int val=0;
	struct LM3643_chip_data *chip = i2c_get_clientdata(client);

	mutex_lock(&chip->lock);
	val =  i2c_smbus_read_byte_data(client, reg);
	mutex_unlock(&chip->lock);


	return val;
}




static int LM3643_chip_init(struct LM3643_chip_data *chip)
{

	
	return 0;
}

static int LM3643_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct LM3643_chip_data *chip;
	struct LM3643_platform_data *pdata = client->dev.platform_data;

	int err = -1;

	printk("LM3643_probe start--->.\n");

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		err = -ENODEV;
		printk(KERN_ERR  "LM3643 i2c functionality check fail.\n");
		return err;
	}

	chip = kzalloc(sizeof(struct LM3643_chip_data), GFP_KERNEL);
	chip->client = client;

	mutex_init(&chip->lock);
	i2c_set_clientdata(client, chip);

	if(pdata == NULL){ //values are set to Zero.
		printk("LM3643 Platform data does not exist\n");
		pdata = kzalloc(sizeof(struct LM3643_platform_data),GFP_KERNEL);
		chip->pdata  = pdata;
		chip->no_pdata = 1;
	}

	chip->pdata  = pdata;
	if(LM3643_chip_init(chip)<0)
		goto err_chip_init;

	LM3643_i2c_client = client;
	printk("LM3643 Initializing is done \n");

	return 0;

err_chip_init:
	i2c_set_clientdata(client, NULL);
	kfree(chip);
	printk("LM3643 probe is failed \n");
	return -ENODEV;
}

static int LM3643_remove(struct i2c_client *client)
{
	struct LM3643_chip_data *chip = i2c_get_clientdata(client);

    if(chip->no_pdata)
		kfree(chip->pdata);
	kfree(chip);
	return 0;
}


#define LM3643_NAME "leds-LM3643"
#define LM3643_ADDR 0xc6
#define LM3643_I2C_BUSNUM 3
static const struct i2c_device_id LM3643_id[] = {
	{LM3643_NAME, 0},
	{}
};

static struct i2c_driver LM3643_i2c_driver = {
	.driver = {
		.name  = LM3643_NAME,
	},
	.probe	= LM3643_probe,
	.remove   = LM3643_remove,
	.id_table = LM3643_id,
};

struct LM3643_platform_data LM3643_pdata = {0, 0, 0, 0, 0};
static struct i2c_board_info __initdata i2c_LM3643={ I2C_BOARD_INFO(LM3643_NAME, LM3643_ADDR>>1), \
													.platform_data = &LM3643_pdata,};

static int __init LM3643_init(void)
{
	printk("LM3643_init\n");
	//i2c_register_board_info(2, &i2c_LM3643, 1);
	i2c_register_board_info(LM3643_I2C_BUSNUM, &i2c_LM3643, 1);


	return i2c_add_driver(&LM3643_i2c_driver);
}

static void __exit LM3643_exit(void)
{
	i2c_del_driver(&LM3643_i2c_driver);
}


module_init(LM3643_init);
module_exit(LM3643_exit);

MODULE_DESCRIPTION("Flash driver for LM3643");
MODULE_AUTHOR("pw <pengwei@mediatek.com>");
MODULE_LICENSE("GPL v2");

int readReg(int reg)
{

    int val;
    val = LM3643_read_reg(LM3643_i2c_client, reg);
    return (int)val;
}

int FL_Enable(void)
{
	char buf[2];
//	char bufR[2];
    if(g_duty<0)
        g_duty=0;
    else if(g_duty>16)
        g_duty=16;
  if(g_duty==0)
  {
    int val;
    val = (g_duty-1);
    buf[0]=REG_ENABLE;
#if defined(LYCONFIG_DUAL_COLORTEMPERATURE_FLASHLIGHT_SUPPORT)
    buf[1]=0x0b;	//torch mode
#else 
    buf[1]=0x09;	//torch mode
#endif
    LM3643_write_reg(LM3643_i2c_client, buf[0], buf[1]);
  }
  else
  {
        int val;
        if(g_duty==0)
            val=1;
        else if(g_duty==1)
            val=2;
        else //if(g_duty==2)
            val=3;
        buf[0]=REG_ENABLE;
#if defined(LYCONFIG_DUAL_COLORTEMPERATURE_FLASHLIGHT_SUPPORT)
        buf[1]=0x0f;			//flash mode
#else
        buf[1]=0x0d;			//flash mode
#endif
        LM3643_write_reg(LM3643_i2c_client, buf[0], buf[1]);
  }
    PK_DBG(" FL_Enable line=%d\n",__LINE__);

    return 0;
}



int FL_Disable(void)
{
		char buf[2];

///////////////////////
	buf[0]=REG_ENABLE;
	buf[1]=0x00;

	LM3643_write_reg(LM3643_i2c_client, buf[0], buf[1]);
	PK_DBG(" FL_Disable line=%d\n",__LINE__);
    return 0;
}

int FL_dim_duty(kal_uint32 duty)
{
	PK_DBG(" FL_dim_duty line=%d\n",__LINE__);
	g_duty = duty;
    return 0;
}




int FL_Init(void)
{
	int regVal0;
	char buf[2];
	mt_set_gpio_mode(GPIO_FLASH_EN, GPIO_MODE_00);                                                                                                                   
	mt_set_gpio_dir(GPIO_FLASH_EN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_FLASH_EN, GPIO_OUT_ONE);
	buf[0]=REG_FLASH_LED0_BR;		
	buf[1]=0x3f;		//flash0 729mA
	LM3643_write_reg(LM3643_i2c_client, buf[0], buf[1]);
	
	buf[0]=REG_FLASH_LED1_BR;
	buf[1]=0x3f;		//flash1 729mA
	LM3643_write_reg(LM3643_i2c_client, buf[0], buf[1]);

	buf[0]=REG_TORCH_LED0_BR;
	buf[1]=0x3f;		//torch0 89.3mA
	LM3643_write_reg(LM3643_i2c_client, buf[0], buf[1]);

	buf[0]=REG_TORCH_LED1_BR;
	buf[1]=0x3f;		//torch1 89.3mA
	LM3643_write_reg(LM3643_i2c_client, buf[0], buf[1]);

	buf[0]=REG_FLASH_TOUT;
	buf[1]=0x1a;		//ramp time 1mS,timeout 150mS
	LM3643_write_reg(LM3643_i2c_client, buf[0], buf[1]);
/*
	buf[0]=0x07;
	buf[1] = LM3643_read_reg(LM3643_i2c_client, buf[0]);
	buf[1] &= 0xf7;
	LM3643_write_reg(LM3643_i2c_client, buf[0], buf[1]);
*/
/*
	if(mt_set_gpio_mode(FLASH_GPIO_ENT,GPIO_MODE_00)){PK_DBG("[constant_flashlight] set gpio mode failed!! \n");}
    if(mt_set_gpio_dir(FLASH_GPIO_ENT,GPIO_DIR_OUT)){PK_DBG("[constant_flashlight] set gpio dir failed!! \n");}
    if(mt_set_gpio_out(FLASH_GPIO_ENT,GPIO_OUT_ZERO)){PK_DBG("[constant_flashlight] set gpio failed!! \n");}

    	if(mt_set_gpio_mode(FLASH_GPIO_ENF,GPIO_MODE_00)){PK_DBG("[constant_flashlight] set gpio mode failed!! \n");}
    if(mt_set_gpio_dir(FLASH_GPIO_ENF,GPIO_DIR_OUT)){PK_DBG("[constant_flashlight] set gpio dir failed!! \n");}
    if(mt_set_gpio_out(FLASH_GPIO_ENF,GPIO_OUT_ZERO)){PK_DBG("[constant_flashlight] set gpio failed!! \n");}
    */




    PK_DBG(" FL_Init line=%d\n",__LINE__);
    return 0;
}


int FL_Uninit(void)
{
	FL_Disable();
	mt_set_gpio_mode(GPIO_FLASH_EN, GPIO_MODE_00);                                                                                                                   
	mt_set_gpio_dir(GPIO_FLASH_EN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_FLASH_EN, GPIO_OUT_ZERO);
    return 0;
}

/*****************************************************************************
User interface
*****************************************************************************/

static void work_timeOutFunc(struct work_struct *data)
{
    FL_Disable();
    PK_DBG("ledTimeOut_callback\n");
    //printk(KERN_ALERT "work handler function./n");
}



enum hrtimer_restart ledTimeOutCallback(struct hrtimer *timer)
{
    schedule_work(&workTimeOut);
    return HRTIMER_NORESTART;
}
static struct hrtimer g_timeOutTimer;
void timerInit(void)
{
  INIT_WORK(&workTimeOut, work_timeOutFunc);
	g_timeOutTimeMs=1000; //1s
	hrtimer_init( &g_timeOutTimer, CLOCK_MONOTONIC, HRTIMER_MODE_REL );
	g_timeOutTimer.function=ledTimeOutCallback;

}



static int constant_flashlight_ioctl(unsigned int cmd, unsigned long arg)
{
	int i4RetValue = 0;
	int ior_shift;
	int iow_shift;
	int iowr_shift;
	ior_shift = cmd - (_IOR(FLASHLIGHT_MAGIC,0, int));
	iow_shift = cmd - (_IOW(FLASHLIGHT_MAGIC,0, int));
	iowr_shift = cmd - (_IOWR(FLASHLIGHT_MAGIC,0, int));
	PK_DBG("LM3643 constant_flashlight_ioctl() line=%d ior_shift=%d, iow_shift=%d iowr_shift=%d arg=%d\n",__LINE__, ior_shift, iow_shift, iowr_shift,(int)arg);
    switch(cmd)
    {

		case FLASH_IOC_SET_TIME_OUT_TIME_MS:
			PK_DBG("FLASH_IOC_SET_TIME_OUT_TIME_MS: %d\n",(int)arg);
			g_timeOutTimeMs=arg;
		break;


    	case FLASH_IOC_SET_DUTY :
    		PK_DBG("FLASHLIGHT_DUTY: %d\n",(int)arg);
    		FL_dim_duty(arg);
    		break;


    	case FLASH_IOC_SET_STEP:
    		PK_DBG("FLASH_IOC_SET_STEP: %d\n",(int)arg);

    		break;

    	case FLASH_IOC_SET_ONOFF :
    		PK_DBG("FLASHLIGHT_ONOFF: %d\n",(int)arg);
    		if(arg==1)
    		{

    		    int s;
    		    int ms;
    		    if(g_timeOutTimeMs>1000)
            	{
            		s = g_timeOutTimeMs/1000;
            		ms = g_timeOutTimeMs - s*1000;
            	}
            	else
            	{
            		s = 0;
            		ms = g_timeOutTimeMs;
            	}

				if(g_timeOutTimeMs!=0)
	            {
	            	ktime_t ktime;
					ktime = ktime_set( s, ms*1000000 );
					hrtimer_start( &g_timeOutTimer, ktime, HRTIMER_MODE_REL );
	            }
    			FL_Enable();
    		}
    		else
    		{
    			FL_Disable();
				hrtimer_cancel( &g_timeOutTimer );
    		}
    		break;
		default :
    		PK_DBG(" No such command \n");
    		i4RetValue = -EPERM;
    		break;
    }
    return i4RetValue;
}




static int constant_flashlight_open(void *pArg)
{
    int i4RetValue = 0;
    PK_DBG("constant_flashlight_open line=%d\n", __LINE__);

	if (0 == strobe_Res)
	{
	    FL_Init();
		timerInit();
	}
	PK_DBG("constant_flashlight_open line=%d\n", __LINE__);
	spin_lock_irq(&g_strobeSMPLock);


    if(strobe_Res)
    {
        PK_ERR(" busy!\n");
        i4RetValue = -EBUSY;
    }
    else
    {
        strobe_Res += 1;
    }


    spin_unlock_irq(&g_strobeSMPLock);
    PK_DBG("constant_flashlight_open line=%d\n", __LINE__);

    return i4RetValue;

}


static int constant_flashlight_release(void *pArg)
{
    PK_DBG(" constant_flashlight_release\n");

    if (strobe_Res)
    {
        spin_lock_irq(&g_strobeSMPLock);

        strobe_Res = 0;
        strobe_Timeus = 0;

        /* LED On Status */
        g_strobe_On = FALSE;

        spin_unlock_irq(&g_strobeSMPLock);

    	FL_Uninit();
    }

    PK_DBG(" Done\n");

    return 0;

}


FLASHLIGHT_FUNCTION_STRUCT	constantFlashlightFunc=
{
	constant_flashlight_open,
	constant_flashlight_release,
	constant_flashlight_ioctl
};


MUINT32 constantFlashlightInit(PFLASHLIGHT_FUNCTION_STRUCT *pfFunc)
{
    if (pfFunc != NULL)
    {
        *pfFunc = &constantFlashlightFunc;
    }
    return 0;
}



/* LED flash control for high current capture mode*/
ssize_t strobe_VDIrq(void)
{

    return 0;
}

EXPORT_SYMBOL(strobe_VDIrq);


