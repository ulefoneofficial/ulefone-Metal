#include <linux/kernel.h> //constant xx
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
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/xlog.h>
#include <linux/version.h>
#include "../../../../power/mt6735/sm5701.h"

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,37))
#include <linux/mutex.h>
#else
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,27)
#include <linux/semaphore.h>
#else
#include <asm/semaphore.h>
#endif
#endif



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
#define TAG_NAME "leds_strobe.c"
#define PK_DBG_NONE(fmt, arg...)    do {} while (0)
#define PK_DBG_FUNC(fmt, arg...)    xlog_printk(ANDROID_LOG_DEBUG  , TAG_NAME, KERN_INFO  "%s: " fmt, __FUNCTION__ ,##arg)
#define PK_WARN(fmt, arg...)        xlog_printk(ANDROID_LOG_WARNING, TAG_NAME, KERN_WARNING  "%s: " fmt, __FUNCTION__ ,##arg)
#define PK_NOTICE(fmt, arg...)      xlog_printk(ANDROID_LOG_DEBUG  , TAG_NAME, KERN_NOTICE  "%s: " fmt, __FUNCTION__ ,##arg)
#define PK_INFO(fmt, arg...)        xlog_printk(ANDROID_LOG_INFO   , TAG_NAME, KERN_INFO  "%s: " fmt, __FUNCTION__ ,##arg)
#define PK_TRC_FUNC(f)              xlog_printk(ANDROID_LOG_DEBUG  , TAG_NAME,  "<%s>\n", __FUNCTION__);
#define PK_TRC_VERBOSE(fmt, arg...) xlog_printk(ANDROID_LOG_VERBOSE, TAG_NAME,  fmt, ##arg)
#define PK_ERROR(fmt, arg...)       xlog_printk(ANDROID_LOG_ERROR  , TAG_NAME, KERN_ERR "%s: " fmt, __FUNCTION__ ,##arg)


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

static u32 current_fled_mode = 0;

#define STROBE_DEVICE_ID 0x92


static struct work_struct workTimeOut;

/*****************************************************************************
Functions
*****************************************************************************/
#define GPIO_ENF GPIO_CAMERA_FLASH_EN_PIN
#define GPIO_ENT GPIO_CAMERA_FLASH_MODE_PIN


    /*CAMERA-FLASH-EN */
BOOL flashlight_onoff_status = FALSE;
extern int flashlight_get_battery_vol(void);
static void work_timeOutFunc(struct work_struct *data);


int readReg(int reg)
{
    char buf[2];
    char bufR[2];
    buf[0]=reg;
    sm5701_chg_read_interface(buf[0] , bufR,0xff, 0);
    //PK_DBG("qq reg=%d val=%d qq\n", buf[0],bufR[0]);
    return (int)bufR[0];
}


int sm5701_fled_get_mode(void)
{
    PK_DBG("%s led_status = %d\n",__func__,current_fled_mode);

    // led_status == 0 : LED_DISABLE
    // led_status == 1 : LED_MOVIE
    // led_status == 2 : LED_FLASH

    return current_fled_mode;
}

static int sm5701_fled_set_mode(int mode)
{
    current_fled_mode = mode;

	switch (mode) {
		case FLASHLIGHT_MODE_OFF:
			PK_DBG("FLASHLIGHT_MODE_OFF\n");
			break;
		case FLASHLIGHT_MODE_MOVIE:
			PK_DBG("FLASHLIGHT_MODE_MOVIE\n");
			break;
		case FLASHLIGHT_MODE_FLASH:
			PK_DBG("FLASHLIGHT_MODE_FLASH\n");           
			break;  
		default:
			PK_DBG("Not FLASH MODE ERROR\n");
			return -EINVAL;
	}
	return 0;
}

static int sm5701_fled_flash(int turn_way)
{
    int mode = sm5701_fled_get_mode();
    int regval;
	char buf[2];
	int ret = 0;

    regval = readReg(0x0f);//FLEDCNTL1
    regval &= ~SM5701_FLEDCNTL1_FLEDEN_MASK;

    if (turn_way == TURN_WAY_I2C)
    {
    	switch (mode) 
        {
    		case FLASHLIGHT_MODE_FLASH:
                regval |= mode;
                buf[0]=0x0f;//FLEDCNTL1
                buf[1]=regval;
                sm5701_chg_config_interface(buf[0] , buf[1], 0xff,0);
    			break;
    		case FLASHLIGHT_MODE_MOVIE:
                regval |= mode;
                buf[0]=0x0f;//FLEDCNTL1
                buf[1]=regval;
                sm5701_chg_config_interface(buf[0] , buf[1], 0xff,0);
    			break;
            case FLASHLIGHT_MODE_OFF: 
                regval |= mode;
                buf[0]=0x0f;//FLEDCNTL1
                buf[1]=regval;
                sm5701_chg_config_interface(buf[0] , buf[1], 0xff,0); 
                break;
    		default:
                regval |= mode;
                buf[0]=0x0f;//FLEDCNTL1
                buf[1]=regval;
                sm5701_chg_config_interface(buf[0] , buf[1], 0xff,0);
    			PK_DBG("Error : not flash / mode\n");
    			ret = -EINVAL;
    	}
    }
    else if (turn_way == TURN_WAY_GPIO)
    {
    	switch (mode) {
    		case FLASHLIGHT_MODE_FLASH:
                regval |= SM5701_FLEDEN_EXTERNAL;
                buf[0]=0x0f;//FLEDCNTL1
                buf[1]=regval;
                sm5701_chg_config_interface(buf[0] , buf[1], 0xff,0); 
    			break;
    		case FLASHLIGHT_MODE_MOVIE:
                regval |= SM5701_FLEDEN_EXTERNAL;
                buf[0]=0x0f;//FLEDCNTL1
                buf[1]=regval;
                sm5701_chg_config_interface(buf[0] , buf[1], 0xff,0); 
    			break;
            case FLASHLIGHT_MODE_OFF:
                regval |= SM5701_FLEDEN_DISABLE;
                buf[0]=0x0f;//FLEDCNTL1
                buf[1]=regval;
                sm5701_chg_config_interface(buf[0] , buf[1], 0xff,0); 
                break;
    		default:
                regval |= SM5701_FLEDEN_DISABLE;
                buf[0]=0x0f;//FLEDCNTL1
                buf[1]=regval;
                sm5701_chg_config_interface(buf[0] , buf[1], 0xff,0); 
    			PK_DBG("Error : not flash / mode\n");
    			ret = -EINVAL;
    	}   
    }
    else
    {
        PK_DBG("Error : not flash / mode\n");
        ret = -EINVAL;
    }
    
	return ret;
}

static int sm5701_fled_notification(void)
{
    int vbus_valid = readReg(SM5701_STATUS1);
    int regval;
	char buf[2];

    int boost = sm5701_charger_get_otgmode();
    int chgon = sm5701_charger_get_chgmode();
    if(boost) chgon = 0;
    regval = readReg(0x09);//CNTL
    regval &= ~SM5701_CNTL_OPERATIONMODE_MASK;

	if (chgon) {
		if (current_fled_mode == FLASHLIGHT_MODE_MOVIE )			
        {
            sm5701_chg_set_bstout(SM5701_BSTOUT_4P5);
            
            regval |= SM5701_OPERATIONMODE_CHARGER_ON_FLASH_ON;
            buf[0]=0x09;//CNTL
            buf[1]=regval;
            sm5701_chg_config_interface(buf[0] , buf[1], 0xff,0);
        }
        else if (current_fled_mode == FLASHLIGHT_MODE_FLASH )
        { 
            sm5701_chg_set_bstout(SM5701_BSTOUT_4P5);
            
            regval |= SM5701_OPERATIONMODE_FLASH_ON;
            buf[0]=0x09;//CNTL
            buf[1]=regval;
            sm5701_chg_config_interface(buf[0] , buf[1], 0xff,0);
        }
        else
        {      
            sm5701_chg_set_bstout(SM5701_BSTOUT_4P5);
            
            regval |= SM5701_OPERATIONMODE_CHARGER_ON;
            buf[0]=0x09;//CNTL
            buf[1]=regval;
            sm5701_chg_config_interface(buf[0] , buf[1], 0xff,0);
        }
	}else {
		if (current_fled_mode == FLASHLIGHT_MODE_MOVIE )			
        {
            sm5701_chg_set_bstout(SM5701_BSTOUT_4P5);
            
            //regval |= SM5701_OPERATIONMODE_FLASH_ON;
            regval |= SM5701_OPERATIONMODE_OTG_ON_FLASH_ON;
            buf[0]=0x09;//CNTL
            buf[1]=regval;
            sm5701_chg_config_interface(buf[0] , buf[1], 0xff,0);
        }
        else if (current_fled_mode == FLASHLIGHT_MODE_FLASH )
        {      
            sm5701_chg_set_bstout(SM5701_BSTOUT_4P5);
            
            //regval |= SM5701_OPERATIONMODE_FLASH_ON;
            regval |= SM5701_OPERATIONMODE_OTG_ON_FLASH_ON;
            buf[0]=0x09;//CNTL
            buf[1]=regval;
            sm5701_chg_config_interface(buf[0] , buf[1], 0xff,0);
        }
	 else if (current_fled_mode == FLASHLIGHT_MODE_OFF )
        {      
            sm5701_chg_set_bstout(SM5701_BSTOUT_4P5);
            
            //regval |= SM5701_OPERATIONMODE_FLASH_ON;
            regval |= SM5701_OPERATIONMODE_OTG_ON;
            buf[0]=0x09;//CNTL
            buf[1]=regval;
            sm5701_chg_config_interface(buf[0] , buf[1], 0xff,0);
        }
        else
        {
            if(boost)
            {
                sm5701_chg_set_bstout(SM5701_BSTOUT_5P0);
                
                regval |= SM5701_OPERATIONMODE_OTG_ON;
                buf[0]=0x09;//CNTL
                buf[1]=regval;
                sm5701_chg_config_interface(buf[0] , buf[1], 0xff,0);
            }
            else
            {
                sm5701_chg_set_bstout(SM5701_BSTOUT_4P5);
                
                regval |= SM5701_OPERATIONMODE_CHARGER_ON;
                buf[0]=0x09;//CNTL
                buf[1]=regval;
                sm5701_chg_config_interface(buf[0] , buf[1], 0xff,0);
            }
        }
    }

	return 0;
}

#define GPIO_CAM_FLASH_SWITCH          (GPIO91 | 0x80000000)
int FL_Enable(void)
{
    char buf[2];
    int temp_bat_vol;
    temp_bat_vol = flashlight_get_battery_vol();
    //printk("%s, zhangchongyong temp_bat_vol = %d\n", __func__, temp_bat_vol);
    if(temp_bat_vol>3750)
    {
	    buf[0]=0x11; //FLEDCNTL3
	    //buf[1]=0xc; //IFLED : 600mA
	    #if defined(LYCONFIG_COMB_CUST_PROJECT_NAME_F9B_HXXD) || defined(LYCONFIG_COMB_CUST_PROJECT_NAME_F9B_BASE)	
	     buf[1]=0x8; //IFLED : 500mA
	    #else
             buf[1]=0xf; //IFLED : 700mA
	    #endif		
	    sm5701_chg_config_interface(buf[0] , buf[1], 0xff,0);
	    buf[0]=0x12; //FLEDCNTL4
	    buf[1]=0x09; //IMLED : 100mA
	    //buf[1]=0x13; //IMLED : 200mA
	    sm5701_chg_config_interface(buf[0] , buf[1], 0xff,0);     
    }
    else
    {
	    buf[0]=0x11; //FLEDCNTL3
	    buf[1]=0x4; //IFLED : 400mA
	    //buf[1]=0xc; //IFLED : 600mA
	    //buf[1]=0xf; //IFLED : 700mA
	    sm5701_chg_config_interface(buf[0] , buf[1], 0xff,0);
	    buf[0]=0x12; //FLEDCNTL4
	    buf[1]=0x09; //IMLED : 100mA
	   // buf[1]=0x13; //IMLED : 200mA
	    sm5701_chg_config_interface(buf[0] , buf[1], 0xff,0); 
    }
	mt_set_gpio_mode(GPIO_CAM_FLASH_SWITCH, GPIO_MODE_00);                                                                                                                   
       mt_set_gpio_dir(GPIO_CAM_FLASH_SWITCH, GPIO_DIR_OUT);
       mt_set_gpio_out(GPIO_CAM_FLASH_SWITCH, GPIO_OUT_ZERO);
	flashlight_onoff_status = TRUE;
 	PK_DBG(" <xhf>FL_Enable g_duty=%d\n",g_duty);
	if(g_duty==0)
	{
	//TORCH mode
     sm5701_fled_set_mode(FLASHLIGHT_MODE_MOVIE);
	 sm5701_fled_notification();
     sm5701_fled_flash(TURN_WAY_GPIO);
     mt_set_gpio_out(GPIO_ENT,GPIO_OUT_ZERO);
	 mt_set_gpio_out(GPIO_ENF,GPIO_OUT_ONE);
	 PK_DBG(" FL_Enable line=%d\n",__LINE__);
	}
	else
	{
	//FLASH mode
     sm5701_fled_set_mode(FLASHLIGHT_MODE_FLASH);
	 sm5701_fled_notification();
     sm5701_fled_flash(TURN_WAY_GPIO);
	 mt_set_gpio_out(GPIO_ENT,GPIO_OUT_ONE);
	 mt_set_gpio_out(GPIO_ENF,GPIO_OUT_ZERO);
	 PK_DBG(" FL_Enable line=%d\n",__LINE__);
	}

    return 0;
}

int FL_Disable(void)
{

	mt_set_gpio_out(GPIO_ENT,GPIO_OUT_ZERO);
	mt_set_gpio_out(GPIO_ENF,GPIO_OUT_ZERO);
	sm5701_fled_set_mode(FLASHLIGHT_MODE_OFF);    
    sm5701_fled_flash(TURN_WAY_GPIO);
    sm5701_fled_notification();
	PK_DBG(" FL_Disable line=%d\n",__LINE__);
	flashlight_onoff_status = FALSE;
    return 0;
}

int FL_dim_duty(kal_uint32 duty)
{
	g_duty=duty;
	PK_DBG(" FL_dim_duty line=%d\n",__LINE__);
    return 0;
}


int FL_Init(void)
{
    char buf[2];

    if(flashlight_get_battery_vol()>3750)
    {
	    buf[0]=0x11; //FLEDCNTL3
	    //buf[1]=0xc; //IFLED : 600mA
	    buf[1]=0xf; //IFLED : 700mA
	    sm5701_chg_config_interface(buf[0] , buf[1], 0xff,0);
	    buf[0]=0x12; //FLEDCNTL4
	    //buf[1]=0x09; //IMLED : 100mA
	    buf[1]=0x13; //IMLED : 200mA
	    sm5701_chg_config_interface(buf[0] , buf[1], 0xff,0);     
    }
    else
    {
	    buf[0]=0x11; //FLEDCNTL3
	    buf[1]=0x9; //IFLED : 500mA
	    //buf[1]=0xc; //IFLED : 600mA
	    //buf[1]=0xf; //IFLED : 700mA
	    sm5701_chg_config_interface(buf[0] , buf[1], 0xff,0);
	    buf[0]=0x12; //FLEDCNTL4
	    buf[1]=0x09; //IMLED : 100mA
	   // buf[1]=0x13; //IMLED : 200mA
	    sm5701_chg_config_interface(buf[0] , buf[1], 0xff,0); 
    }
    if(mt_set_gpio_mode(GPIO_ENF,GPIO_MODE_00)){PK_DBG("[constant_flashlight] set gpio mode failed!! \n");}
    if(mt_set_gpio_dir(GPIO_ENF,GPIO_DIR_OUT)){PK_DBG("[constant_flashlight] set gpio dir failed!! \n");}
    if(mt_set_gpio_out(GPIO_ENF,GPIO_OUT_ZERO)){PK_DBG("[constant_flashlight] set gpio failed!! \n");}
    /*Init. to disable*/
    if(mt_set_gpio_mode(GPIO_ENT,GPIO_MODE_00)){PK_DBG("[constant_flashlight] set gpio mode failed!! \n");}
    if(mt_set_gpio_dir(GPIO_ENT,GPIO_DIR_OUT)){PK_DBG("[constant_flashlight] set gpio dir failed!! \n");}
    if(mt_set_gpio_out(GPIO_ENT,GPIO_OUT_ZERO)){PK_DBG("[constant_flashlight] set gpio failed!! \n");}

    INIT_WORK(&workTimeOut, work_timeOutFunc);
    PK_DBG(" FL_Init line=%d\n",__LINE__);
    return 0;
}


int FL_Uninit(void)
{
	FL_Disable();
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
	g_timeOutTimeMs=1000; //1s
	hrtimer_init( &g_timeOutTimer, CLOCK_MONOTONIC, HRTIMER_MODE_REL );
	g_timeOutTimer.function=ledTimeOutCallback;

}



static int constant_flashlight_ioctl(MUINT32 cmd, MUINT32 arg)
{
	int i4RetValue = 0;
	int ior_shift;
	int iow_shift;
	int iowr_shift;
	ior_shift = cmd - (_IOR(FLASHLIGHT_MAGIC,0, int));
	iow_shift = cmd - (_IOW(FLASHLIGHT_MAGIC,0, int));
	iowr_shift = cmd - (_IOWR(FLASHLIGHT_MAGIC,0, int));
	PK_DBG("constant_flashlight_ioctl() line=%d ior_shift=%d, iow_shift=%d iowr_shift=%d arg=%d\n",__LINE__, ior_shift, iow_shift, iowr_shift, arg);
    switch(cmd)
    {

		case FLASH_IOC_SET_TIME_OUT_TIME_MS:
			PK_DBG("FLASH_IOC_SET_TIME_OUT_TIME_MS: %d\n",arg);
			g_timeOutTimeMs=arg;
		break;


    	case FLASH_IOC_SET_DUTY :
    		PK_DBG("FLASHLIGHT_DUTY: %d\n",arg);
    		FL_dim_duty(arg);
    		break;


    	case FLASH_IOC_SET_STEP:
    		PK_DBG("FLASH_IOC_SET_STEP: %d\n",arg);

    		break;

    	case FLASH_IOC_SET_ONOFF :
    		PK_DBG("FLASHLIGHT_ONOFF: %d\n",arg);
    		if(arg==1)
    		{
				if(g_timeOutTimeMs!=0)
	            {
	            	ktime_t ktime;
					ktime = ktime_set( 0, g_timeOutTimeMs*1000000 );
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


