/* drivers/hwmon/mt6516/amit/em3071.c - em3071 ALS/PS driver
 * 
 * Author: 
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/gpio.h>

#include "cust_alsps.h"
#include "em30718.h"
#include "alsps.h"

/******************************************************************************
 * configuration
*******************************************************************************/
/*----------------------------------------------------------------------------*/

#define EM3071_DEV_NAME     "EM30718"
/*----------------------------------------------------------------------------*/
#define APS_TAG                  "[EM30718] "
#define APS_FUN(f)               printk(APS_TAG"%s %d \n", __FUNCTION__ , __LINE__)
#define APS_ERR(fmt, args...)    printk(APS_TAG"%s %d : "fmt, __FUNCTION__, __LINE__, ##args)
#define APS_LOG(fmt, args...)    printk(APS_TAG fmt, ##args)
#define APS_DBG(fmt, args...)    printk(APS_TAG fmt, ##args)                 
/******************************************************************************
 * extern functions
*******************************************************************************/
/*for interrup work mode support --*/
#define mt65xx_eint_unmask mt_eint_unmask
#define mt65xx_eint_mask mt_eint_mask

#define PS_DRIVE 0Xb8  //0XA0 25MA //0xA8 50MA  //0XB8 200MA  0XB0 100MA  //Psensor Çý¶¯Á¦
#define PS_LTHD_VAL 20
/*----------------------------------------------------------------------------*/
static struct i2c_client *em3071_i2c_client = NULL;
/*----------------------------------------------------------------------------*/
static const struct i2c_device_id em3071_i2c_id[] = {{EM3071_DEV_NAME,0},{}};
/*the adapter id & i2c address will be available in customization*/
//static unsigned short em3071_force[] = {0x00, 0x48, I2C_CLIENT_END, I2C_CLIENT_END};
//static const unsigned short *const em3071_forces[] = { em3071_force, NULL };
//static struct i2c_client_address_data em3071_addr_data = { .forces = em3071_forces,};
/*----------------------------------------------------------------------------*/

static int em3071_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id); 
static int em3071_i2c_remove(struct i2c_client *client);
/*----------------------------------------------------------------------------*/
static int em3071_i2c_suspend(struct i2c_client *client, pm_message_t msg);
static int em3071_i2c_resume(struct i2c_client *client);
int em3071_setup_eint(struct i2c_client *client);
int em3071_read_ps(struct i2c_client *client, u16 *data);

static struct em3071_priv *g_em3071_ptr = NULL;
	
static int  em3071_local_init(void);
static int em3071_remove(void);

static int em3071_init_flag =-1; // 0<==>OK -1 <==> fail

/* Maintain alsps cust info here */
static struct alsps_hw em3071_cust = {
    .i2c_num    = 2,
    .polling_mode_ps = 1,
    .polling_mode_als = 1,
    //.power_id   = MT65XX_POWER_NONE,    /*LDO is not used*/
    //.power_vol  = VOL_DEFAULT,          /*LDO is not used*/
    .i2c_addr   = {0x48>>1},
    .als_level  = {15, 40 , 60, 90, 112, 132, 205, 273, 500, 845, 1136, 1545, 2364, 4655, 4095}, /* als_code */
   // .als_value  = {0, 10, 40, 65, 90, 145, 225, 300, 550, 930, 1250, 1700, 2600, 5120, 7680, 10240},    /* lux */
    .als_value  = {0, 50, 100, 150, 300, 600, 800, 1000, 1550, 1930, 2250, 2700, 3600, 5120, 7680, 10240},    /* lux */
    .ps_threshold = 80,
    .ps_threshold_high = 100,
    .ps_threshold_low = 60,
};
static struct alsps_hw *hw = &em3071_cust;
static struct platform_device *alspsPltFmDev;


/* For alsp driver get cust info */
struct alsps_hw *em3071_get_cust_alsps(void)
{
	return &em3071_cust;
}


static struct alsps_init_info em3071_init_info = {
		.name = EM3071_DEV_NAME,
		.init = em3071_local_init,
		.uninit = em3071_remove,
};
static int intr_flag_value = 0;

static DEFINE_MUTEX(sensor_lock);
/*----------------------------------------------------------------------------*/
typedef enum {
    CMC_BIT_ALS    = 1,
    CMC_BIT_PS     = 2,
} CMC_BIT;
/*----------------------------------------------------------------------------*/
struct em3071_i2c_addr {    /*define a series of i2c slave address*/
    u8  write_addr;  
    u8  ps_thd;     /*PS INT threshold*/
};
/*----------------------------------------------------------------------------*/
struct em3071_priv {
    struct alsps_hw  *hw;
    struct i2c_client *client;
    struct work_struct  eint_work;

    /*i2c address group*/
    struct em3071_i2c_addr  addr;
    
    /*misc*/
//    u16		    als_modulus;
    atomic_t    i2c_retry;
    atomic_t    als_suspend;
    atomic_t    als_debounce;   /*debounce time after enabling als*/
    atomic_t    als_deb_on;     /*indicates if the debounce is on*/
    atomic_t    als_deb_end;    /*the jiffies representing the end of debounce*/
    atomic_t    ps_mask;        /*mask ps: always return far away*/
    atomic_t    ps_debounce;    /*debounce time after enabling ps*/
    atomic_t    ps_deb_on;      /*indicates if the debounce is on*/
    atomic_t    ps_deb_end;     /*the jiffies representing the end of debounce*/
    atomic_t    ps_suspend;
	struct device_node *irq_node;
	int		irq;

    /*data*/
    u16         als;
    u16          ps;
    u8          _align;
    u16         als_level_num;
    u16         als_value_num;
    u32         als_level[C_CUST_ALS_LEVEL-1];
    u32         als_value[C_CUST_ALS_LEVEL];

    atomic_t    als_cmd_val;    /*the cmd value can't be read, stored in ram*/
    atomic_t    ps_cmd_val;     /*the cmd value can't be read, stored in ram*/
    atomic_t    ps_thd_val;     /*the cmd value can't be read, stored in ram*/
    atomic_t    ps_thd_val_h;   /*the cmd value can't be read, stored in ram*/
    atomic_t    ps_thd_val_l;   /*the cmd value can't be read, stored in ram*/
    ulong       enable;         /*enable mask*/
    ulong       pending_intr;   /*pending interrupt*/

    /*early suspend*/
#if defined(CONFIG_HAS_EARLYSUSPEND)
    struct early_suspend    early_drv;
#endif     
};

#ifdef CONFIG_OF
static const struct of_device_id alsps_of_match[] = {
	{.compatible = "mediatek,alsps"},
	{},
};
#endif

/*----------------------------------------------------------------------------*/
static struct i2c_driver em3071_i2c_driver = {	
	.probe      = em3071_i2c_probe,
	.remove     = em3071_i2c_remove,
	.suspend    = em3071_i2c_suspend,
	.resume     = em3071_i2c_resume,
	.id_table   = em3071_i2c_id,
	.driver = {
		.owner          = THIS_MODULE,
		.name           = EM3071_DEV_NAME,
#ifdef CONFIG_OF
		.of_match_table = alsps_of_match,
#endif
		
	},
};

static int ps_enabled=0;

static struct em3071_priv *em3071_obj = NULL;


static int em30713_read(struct i2c_client *client,u8 reg)
{
	int res = 0;
	u8 buffer[1];
	
	buffer[0] = reg;
	res = i2c_master_send(client, buffer, 0x1);
	if(res <= 0)
	{
		return 0;
	}
	res = i2c_master_recv(client, buffer, 0x1);
	if(res <= 0)
	{
		return 0;
	}
	return buffer[0];
}

#if 0
static int em30713_write(struct i2c_client *client,u8 reg,u8 data)
{
	int res = 0;
	u8 buffer[2];
	buffer[0] = reg;
	buffer[1] = data;
	res = i2c_master_send(client, buffer, 0x2);
	if(res <= 0)
	{
		return 0;
	}
	return 1;
}

#endif
/******************************************************************************
*********************DEVICE ATTER**********************************************
******************************************************************************/

#if 0
typedef enum {
    SHOW = 0,
    STORE  ,
} DEV_S;


static ssize_t show_em_readps_value(struct device_driver *ddri, char *buf)
{	
	u8 res=0;
	res=dev_attr_config(EM3071_CMM_PDATA_L,PS_DRIVE,SHOW,buf);
	return snprintf(buf, PAGE_SIZE, "%x\n", res);

}

static ssize_t show_em_readals_value(struct device_driver *ddri, char *buf)
{
	u8 als_l,als_h,als_value,config;
	config = PS_DRIVE | 0x06;
	als_l = dev_attr_config(EM3071_CMM_C0DATA_L,config,SHOW,buf);
	als_h = dev_attr_config(EM3071_CMM_C0DATA_H,config,SHOW,buf);
	als_value = als_l | (als_h<<8);
	return snprintf(buf, PAGE_SIZE, "%x\n", als_value); 
}
static ssize_t show_em_readid_value(struct device_driver *ddri, char *buf)
{
	u8 res=0;
	res=dev_attr_config(EM3071_CMM_ID,PS_DRIVE,SHOW,buf);
	return snprintf(buf, PAGE_SIZE, "%x\n", res); 
}
static ssize_t show_em_ps_int_state(struct device_driver *ddri, char *buf)
{
	u8 res=0;
	res=dev_attr_config(EM3071_CMM_STATUS,PS_DRIVE,SHOW,buf);
	return snprintf(buf, PAGE_SIZE, "%x\n", res); 
}

static ssize_t show_em_set_ps_config(struct device_driver *ddri, char *buf)
{
	u8 res=0;
	res=dev_attr_config(EM3071_CMM_ENABLE,PS_DRIVE,SHOW,buf);
	return snprintf(buf, PAGE_SIZE, "%x\n", res); 
}
static ssize_t store_em_set_ps_config(struct device_driver *ddri, const char *buf, size_t count)
{
	u8 res=0,ret;
	int config;
	ret = sscanf(buf, "%x", &config);
	res = dev_attr_config(EM3071_CMM_ENABLE,config,STORE,config);
	return count; 
}
static ssize_t show_em_psint_low_thd(struct device_driver *ddri, char *buf)
{
	u8 res=0;
	res = dev_attr_config(EM3071_CMM_INT_PS_LT,PS_DRIVE,SHOW,buf);
	return snprintf(buf, PAGE_SIZE, "%x\n", res);
}
static ssize_t store_em_psint_low_thd(struct device_driver *ddri, const char *buf, size_t count)
{
	u8 res = 0;
	int value;
	res = sscanf(buf, "%x", &value);
	res = dev_attr_config(EM3071_CMM_INT_PS_LT,PS_DRIVE,STORE,value);
	return count;
}
static ssize_t show_em_psint_high_thd(struct device_driver *ddri, char *buf)
{
	u8 res=0;
	res=dev_attr_config(EM3071_CMM_INT_PS_HT,PS_DRIVE,SHOW,buf);
	return snprintf(buf, PAGE_SIZE, "%x\n", res);
}
static ssize_t store_em_psint_high_thd(struct device_driver *ddri, char *buf)
{
	u8 res=0;
	int h_value;
	res = sscanf(buf, "%x", &h_value);
	res=dev_attr_config(EM3071_CMM_INT_PS_HT,PS_DRIVE,STORE,h_value);
	return snprintf(buf, PAGE_SIZE, "%x\n", res);	
}

static ssize_t show_em_offset(struct device_driver *ddri, char *buf)
{
	u8 res=0;
	res=dev_attr_config(EM3071_CMM_OFFSET,PS_DRIVE,SHOW,buf);
	return snprintf(buf, PAGE_SIZE, "%x\n", res);
}
static ssize_t store_em_offset(struct device_driver *ddri, char *buf)
{
	u8 res=0;
	int o_value;
	res = sscanf(buf, "%x", &o_value);
	res = dev_attr_config(EM3071_CMM_OFFSET,PS_DRIVE,STORE,o_value);
	return snprintf(buf, PAGE_SIZE, "%x\n", res);	
}
static int dev_attr_config(u8 reg,int config,int dev_s,int data)
{
	struct i2c_client *client = em3071_i2c_client;
	struct em3071_priv *obj;
	u8 res=0;
	u8 databuf[2],ps_config[2];
	if(client == NULL)
	{
		printk("i2c_client is null \n\r");
		return 0;
	}
	obj = i2c_get_clientdata(client);
	if(dev_s)
	{
		res=0;
		res = em30713_write(client,reg,data);
		return res;
	}else{
		res=0;
		res = em30713_read(client,reg);
		return res;
	}
}

static DRIVER_ATTR(em_ps,S_IWUSR | S_IRUGO, show_em_readps_value, NULL);
static DRIVER_ATTR(em_als,S_IWUSR | S_IRUGO, show_em_readals_value, NULL);
static DRIVER_ATTR(em_id,S_IWUSR | S_IRUGO, show_em_readid_value, NULL);
static DRIVER_ATTR(em_int_state,S_IWUSR | S_IRUGO, show_em_ps_int_state, NULL);
static DRIVER_ATTR(em_config,S_IWUSR | S_IRUGO, show_em_set_ps_config, store_em_set_ps_config);
static DRIVER_ATTR(em_ps_low_thd,S_IWUSR | S_IRUGO, show_em_psint_low_thd, store_em_psint_low_thd);
//static DRIVER_ATTR(em_ps_high_thd,S_IWUSR | S_IRUGO, show_em_psint_high_thd, store_em_psint_high_thd);
//static DRIVER_ATTR(em_offset,S_IWUSR | S_IRUGO, show_em_offset, store_em_offset);

static struct driver_attribute *em30713_attrs[]={
	&driver_attr_em_ps,
	&driver_attr_em_als,
	&driver_attr_em_id,
	&driver_attr_em_int_state,
	&driver_attr_em_config,
	&driver_attr_em_ps_low_thd,
	//&driver_attr_em_ps_high_thd,
	//&driver_attr_em_offset,
};


static int em30713_create_attr(struct device_driver *driver) 
{
	int idx, err = 0;
	int num = (int)(sizeof(em30713_attrs)/sizeof(em30713_attrs[0]));

	if(driver == NULL)
	{
		return -EINVAL;
	}
	

	for(idx = 0; idx < num; idx++)
	{
		err = driver_create_file(driver, em30713_attrs[idx]);
		if (err) {
			APS_ERR("driver_create_file (%s) = %d\n", em30713_attrs[idx]->attr.name, err);
			break;
		}
	}
	
	return err;
}

static int em30713_delete_attr(struct device_driver *driver)
{
	int idx ,err = 0;
	int num = (int)(sizeof(em30713_attrs)/sizeof(em30713_attrs[0]));

	if(driver == NULL)
	{
		return -EINVAL;
	}
	

	for(idx = 0; idx < num; idx++)
	{
		driver_remove_file(driver, em30713_attrs[idx]);
	}
	

	return err;
}

#endif

/*----------------------------------------------------------------------------*/
int em3071_get_addr(struct alsps_hw *hw, struct em3071_i2c_addr *addr)
{
	if(!hw || !addr)
	{
		return -EFAULT;
	}
	addr->write_addr= hw->i2c_addr[0];
	return 0;
}
/*----------------------------------------------------------------------------*/
static void em3071_power(struct alsps_hw *hw, unsigned int on) 
{
}

/*----------------------------------------------------------------------------*/
static int em3071_enable_als(struct i2c_client *client, int enable)
{
        struct em3071_priv *obj = i2c_get_clientdata(client);
        u8 databuf[2];      
        int res = 0;
        //u8 reg_value[1];
        //u8 buffer[2];
        if(client == NULL)
        {
            APS_DBG("CLIENT CANN'T EQUL NULL\n");
            return -1;
        }

	      databuf[0] = EM3071_CMM_ENABLE;    
            res = i2c_master_send(client, databuf, 0x1);
            if(res <= 0)
            {
                goto EXIT_ERR;
            }
            res = i2c_master_recv(client, databuf, 0x1);
            if(res <= 0)
            {
                goto EXIT_ERR;
            }

        if(enable)
        {			
            databuf[1] = ((databuf[0] & 0xF8) | 0x06);
    
            databuf[0] = EM3071_CMM_ENABLE;    
            //databuf[1] = 0xBE;
            res = i2c_master_send(client, databuf, 0x2);
            if(res <= 0)
            {
                goto EXIT_ERR;
            }
           
            atomic_set(&obj->als_deb_on, 1);
            atomic_set(&obj->als_deb_end, jiffies+atomic_read(&obj->als_debounce)/(1000/HZ));
            APS_DBG("em3071 power on\n");
        }
        else
        {
            
            databuf[1] = (databuf[0] & 0xF8);
                     
            databuf[0] = EM3071_CMM_ENABLE;    
        //    databuf[1] = 0xB8;    
            res = i2c_master_send(client, databuf, 0x2);
            if(res <= 0)
            {
                goto EXIT_ERR;
            }
            atomic_set(&obj->als_deb_on, 0);
            APS_DBG("EM3071 power off\n");
        }
		
        return 0;
        
    EXIT_ERR:
        APS_ERR("EM3071_enable_als fail\n");
        return res;
}


/*----------------------------------------------------------------------------*/
static int em3071_enable_ps(struct i2c_client *client, int enable)
{
	struct em3071_priv *obj = i2c_get_clientdata(client);
	u8 databuf[2];    
	int res = 0,report_val = 3,ps_data;
	//u8 buffer[2];
	struct hwm_sensor_data sensor_data;

	if(client == NULL)
	{
		APS_DBG("CLIENT CANN'T EQUL NULL\n");
		return -1;
	}

	
	APS_DBG("em3071_enable_ps, enable = %d\n", enable);

	databuf[0] = EM3071_CMM_ENABLE;	
	res = i2c_master_send(client, databuf, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	res = i2c_master_recv(client, databuf, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}

	if(enable)
	{
		databuf[1] = ((databuf[0] & 0x07) | PS_DRIVE);  //0xb8 :200MA	 0XB6:100MA 
			
		databuf[0] = EM3071_CMM_ENABLE;    

		res = i2c_master_send(client, databuf, 0x2);
		if(res <= 0)
		{
			goto EXIT_ERR;
		}
		atomic_set(&obj->ps_deb_on, 1);
		atomic_set(&obj->ps_deb_end, jiffies+atomic_read(&obj->ps_debounce)/(1000/HZ));
		APS_DBG("em3071 power on\n");

		/*for interrup work mode support */
		if(0 == obj->hw->polling_mode_ps)
		{
				ps_data = em30713_read(client,EM3071_CMM_PDATA_L);
				if(ps_data > atomic_read(&obj->ps_thd_val))
				{
					report_val = 0;
				}	
				if(ps_data < (atomic_read(&obj->ps_thd_val) - PS_LTHD_VAL))
				{
					report_val = 1;
				}
				sensor_data.values[0] = report_val;
				sensor_data.value_divide = 1;
				sensor_data.status = SENSOR_STATUS_ACCURACY_MEDIUM;
			
				if((res = hwmsen_get_interrupt_data(ID_PROXIMITY, &sensor_data)))
				{
				 APS_ERR("call hwmsen_get_interrupt_data fail = %d\n", res);
				}

				databuf[0] = EM3071_CMM_INT_PS_LT;	
				databuf[1] = (u8)(((atomic_read(&obj->ps_thd_val))-PS_LTHD_VAL) & 0x00FF);
				res = i2c_master_send(client, databuf, 0x2);
				if(res <= 0)
				{
					goto EXIT_ERR;
					return EM3071_ERR_I2C;
				}
				databuf[0] = EM3071_CMM_INT_PS_HT;	
				databuf[1] = (u8)((atomic_read(&obj->ps_thd_val)) & 0x00FF);
				res = i2c_master_send(client, databuf, 0x2);
				if(res <= 0)
				{
					goto EXIT_ERR;
					return EM3071_ERR_I2C;
				}
				enable_irq(em3071_obj->irq);
		}
	}
	else
	{
		databuf[1] = (databuf[0] & 0x07);
		
		databuf[0] = EM3071_CMM_ENABLE;    
                       
		res = i2c_master_send(client, databuf, 0x2);
		if(res <= 0)
		{
			goto EXIT_ERR;
		}
		atomic_set(&obj->ps_deb_on, 0);
		APS_DBG("em3071 power off\n");

		/*for interrup work mode support */
		if(0 == obj->hw->polling_mode_ps)
		{
			cancel_work_sync(&obj->eint_work);
			disable_irq(em3071_obj->irq);
		}
	}
	ps_enabled=enable;
	return 0;
	
EXIT_ERR:
	APS_ERR("em3071_enable_ps fail\n");
	return res;
}

/*for interrup work mode support --*/
static int em3071_check_and_clear_intr(struct i2c_client *client) 
{
	//struct em3071_priv *obj = i2c_get_clientdata(client);
	int res;
	u8 buffer[2];

	//if (mt_get_gpio_in(GPIO_ALS_EINT_PIN) == 1) /*skip if no interrupt*/  
	//    return 0;
	
	buffer[0] = EM3071_CMM_STATUS;
	res = i2c_master_send(client, buffer, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	res = i2c_master_recv(client, buffer, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	//APS_ERR("em3071_check_and_clear_intr status=0x%x\n", buffer[0]);
	if(0 != (buffer[0] & 0x80))
	{
		buffer[0] = EM3071_CMM_STATUS;
		buffer[1] = 0x00;
		res = i2c_master_send(client, buffer, 0x2);
		if(res <= 0)
		{
			goto EXIT_ERR;
		}
		res = 0;
	}

	return res;

EXIT_ERR:
	APS_ERR("em3071_check_and_clear_intr fail\n");
	return 1;
}

/*----------------------------------------------------------------------------*/
void em3071_eint_func(void)
{
	struct em3071_priv *obj = g_em3071_ptr;
	if(!obj)
	{
		return;
	}
	
	schedule_work(&obj->eint_work);
}

#if defined(CONFIG_OF)
static irqreturn_t em3071_eint_handler(int irq, void *desc)
{
	em3071_eint_func();
	disable_irq_nosync(em3071_obj->irq);

	return IRQ_HANDLED;
}
#endif

#ifdef CUSTOM_KERNEL_SENSORHUB
static int em3071_irq_handler(void *data, uint len)
{
	struct em3071_priv *obj = em3071_obj;
	SCP_SENSOR_HUB_DATA_P rsp = (SCP_SENSOR_HUB_DATA_P)data;

	if (!obj)
		return -1;

	APS_ERR("len = %d, type = %d, action = %d, errCode = %d\n",
		len, rsp->rsp.sensorType, rsp->rsp.action, rsp->rsp.errCode);

	switch (rsp->rsp.action) {
	case SENSOR_HUB_NOTIFY:
		switch (rsp->notify_rsp.event) {
		case SCP_INIT_DONE:
			schedule_work(&obj->init_done_work);
			/* schedule_delayed_work(&obj->init_done_work, HZ); */
			break;
		case SCP_NOTIFY:
			if (EM3071_NOTIFY_PROXIMITY_CHANGE == rsp->notify_rsp.data[0]) {
				intr_flag_value = rsp->notify_rsp.data[1];
				em3071_eint_func();
			} else
				APS_ERR("Unknown notify");
			break;
		default:
			APS_ERR("Error sensor hub notify");
			break;
		}
		break;
	default:
		APS_ERR("Error sensor hub action");
		break;
	}

	return 0;
}
#endif /* #ifdef CUSTOM_KERNEL_SENSORHUB */


/*----------------------------------------------------------------------------*/
int em3071_setup_eint(struct i2c_client *client)
{
#ifdef CUSTOM_KERNEL_SENSORHUB
	int err = 0;

	err = SCP_sensorHub_rsp_registration(ID_PROXIMITY, em3071_irq_handler);
#else /* #ifdef CUSTOM_KERNEL_SENSORHUB */

	int ret;
	struct pinctrl *pinctrl;
	struct pinctrl_state *pins_default;
	struct pinctrl_state *pins_cfg;
	u32 ints[2] = {0, 0};

	alspsPltFmDev = get_alsps_platformdev();
/* gpio setting */
	pinctrl = devm_pinctrl_get(&alspsPltFmDev->dev);
	if (IS_ERR(pinctrl)) {
		ret = PTR_ERR(pinctrl);
		APS_ERR("Cannot find alsps pinctrl!\n");
	}
	pins_default = pinctrl_lookup_state(pinctrl, "pin_default");
	if (IS_ERR(pins_default)) {
		ret = PTR_ERR(pins_default);
		APS_ERR("Cannot find alsps pinctrl default!\n");

	}

	pins_cfg = pinctrl_lookup_state(pinctrl, "pin_cfg");
	if (IS_ERR(pins_cfg)) {
		ret = PTR_ERR(pins_cfg);
		APS_ERR("Cannot find alsps pinctrl pin_cfg!\n");

	}
/* eint request */
	if (em3071_obj->irq_node) {
		of_property_read_u32_array(em3071_obj->irq_node, "debounce", ints, ARRAY_SIZE(ints));
		gpio_request(ints[0], "p-sensor");
		gpio_set_debounce(ints[0], ints[1]);
		pinctrl_select_state(pinctrl, pins_cfg);
		APS_LOG("ints[0] = %d, ints[1] = %d!!\n", ints[0], ints[1]);

		em3071_obj->irq = irq_of_parse_and_map(em3071_obj->irq_node, 0);
		APS_LOG("em3071_obj->irq = %d\n", em3071_obj->irq);
		if (!em3071_obj->irq) {
			APS_ERR("irq_of_parse_and_map fail!!\n");
			return -EINVAL;
		}
		if (request_irq(em3071_obj->irq, em3071_eint_handler, IRQF_TRIGGER_LOW, "ALS-eint", NULL)) {
			APS_ERR("IRQ LINE NOT AVAILABLE!!\n");
			return -EINVAL;
		}
		//enable_irq(em3071_obj->irq);//by xiehaifei 20160427
	} else {
		APS_ERR("null irq node!!\n");
		return -EINVAL;
	}

#endif /* #ifdef CUSTOM_KERNEL_SENSORHUB */
	return 0;
}

/****************************************************************************** 
**
*****************************************************************************/
int em3071_read_als(struct i2c_client *client, u16 *data)
{
	//struct em3071_priv *obj = i2c_get_clientdata(client);	 
	u16 als_value;	 
	u8 als_value_low[1], als_value_high[1];
	u8 buffer[1];
	int res = 0;
	
	if(client == NULL)
	{
		APS_DBG("CLIENT CANN'T EQUL NULL\n");
		return -1;
	}
//get adc channel 0 value
	buffer[0]=EM3071_CMM_C0DATA_L;
	res = i2c_master_send(client, buffer, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	res = i2c_master_recv(client, als_value_low, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	
	buffer[0]=EM3071_CMM_C0DATA_H;
	res = i2c_master_send(client, buffer, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	res = i2c_master_recv(client, als_value_high, 0x01);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	
	als_value = als_value_low[0] | ((0X0F&als_value_high[0])<<8);
	*data = als_value;
	printk("em30718 als value = %d\t\n",als_value);
	return 0;	 

	
	
EXIT_ERR:
	APS_ERR("em3071_read_ps fail\n");
	return res;
}
/*----------------------------------------------------------------------------*/

static int em3071_get_als_value(struct em3071_priv *obj, u16 als)
{
	int idx;
	int invalid = 0;
	for(idx = 0; idx < obj->als_level_num; idx++)
	{
		if(als < obj->hw->als_level[idx])
		{
			break;
		}
	}
	
	if(idx >= obj->als_value_num)
	{
		APS_ERR("exceed range\n"); 
		idx = obj->als_value_num - 1;
	}
	
	if(1 == atomic_read(&obj->als_deb_on))
	{
		unsigned long endt = atomic_read(&obj->als_deb_end);
		if(time_after(jiffies, endt))
		{
			atomic_set(&obj->als_deb_on, 0);
		}
		
		if(1 == atomic_read(&obj->als_deb_on))
		{
			invalid = 1;
		}
	}

	if(!invalid)
	{
		//APS_DBG("ALS: %05d => %05d\n", als, obj->hw->als_value[idx]);	
		return obj->hw->als_value[idx];
	}
	else
	{
		//APS_ERR("ALS: %05d => %05d (-1)\n", als, obj->hw->als_value[idx]);    
		return -1;
	}
}
/*----------------------------------------------------------------------------*/
int em3071_read_ps(struct i2c_client *client, u16 *data)
{
	//struct em3071_priv *obj = i2c_get_clientdata(client);       
	u8 ps_value[1];
	u8 buffer[1];
	int res = 0;

	if(client == NULL)
	{
		APS_DBG("CLIENT CANN'T EQUL NULL\n");
		return -1;
	}

	buffer[0] = EM3071_CMM_PDATA_L;
	res = i2c_master_send(client, buffer, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	res = i2c_master_recv(client, ps_value, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}

	*data = ps_value[0];
	printk("bingo *******************ps_data=%d\n", *data);
	return 0;    

EXIT_ERR:
	APS_ERR("em3071_read_ps fail\n");
	return res;
}
/*----------------------------------------------------------------------------*/
static int em3071_get_ps_value(struct em3071_priv *obj, u16 ps)
{
	int val;
	int invalid = 0;
	static int val_temp=1;

	printk("em3071  get ps %d\n",ps);
	if((ps > atomic_read(&obj->ps_thd_val)))
	{
		val = 0;  /*close*/
		val_temp = 0;
		intr_flag_value = 1;
	}
	else if((ps < (atomic_read(&obj->ps_thd_val)-PS_LTHD_VAL)))
	{
		val = 1;  /*far away*/
		val_temp = 1;
		intr_flag_value = 0;
	}
	else
	       val = val_temp;	
				
	if(atomic_read(&obj->ps_suspend))
	{
		invalid = 1;
	}
	else if(1 == atomic_read(&obj->ps_deb_on))
	{
		unsigned long endt = atomic_read(&obj->ps_deb_end);
		if(time_after(jiffies, endt))
		{
			atomic_set(&obj->ps_deb_on, 0);
		}
		
		if (1 == atomic_read(&obj->ps_deb_on))
		{
			invalid = 1;
		}
	}

	if(!invalid)
	{
		//APS_DBG("PS:  %05d => %05d\n", ps, val);
		return val;
	}	
	else
	{
		return -1;
	}	
}

/*for interrup work mode support --*/
static int em3071_init_client(struct i2c_client *client)
{
	struct em3071_priv *obj = i2c_get_clientdata(client);
	u8 databuf[2];    
	int res = 0;
	//struct PS_CALI_DATA_STRUCT ps_cali_temp;
	u8 offset_reg = 0;
	
	   databuf[0] = EM3071_CMM_ENABLE;	 
	   databuf[1] = PS_DRIVE;//0XBE 200MA
	   res = i2c_master_send(client, databuf, 0x2);
	   if(res <= 0)
	   {
			APS_FUN();
		   goto EXIT_ERR;
	   }
	 	mdelay(100);
	      em3071_read_ps( client, &obj->ps);
		if(obj->ps <=32)
		{
			offset_reg = 0x00;
		}else if(obj->ps > 32 && obj->ps <= 64)
			{
			offset_reg = 0x02;
		}else if(obj->ps > 64 && obj->ps <= 96)
			{
			offset_reg = 0x04;
		}else if(obj->ps > 96 && obj->ps <= 128)
			{
			offset_reg = 0x06;
		}else if(obj->ps > 128 && obj->ps <= 160)
			{
			offset_reg = 0x08;
		}else if(obj->ps > 160 && obj->ps <= 256)
			{
			offset_reg = 0xa0;
		}
	   databuf[0] = 0x0F;	 
       databuf[1] = offset_reg;
		  
	   res = i2c_master_send(client, databuf, 0x2);
	   if(res <= 0)
	   {
		   goto EXIT_ERR;
	   }
	   	/*for interrup work mode support*/
		if(0 == obj->hw->polling_mode_ps)
		{
			databuf[0] = EM3071_CMM_INT_PS_LT;	
			databuf[1] = (u8)(((atomic_read(&obj->ps_thd_val))-PS_LTHD_VAL) & 0x00FF);
			res = i2c_master_send(client, databuf, 0x2);
			if(res <= 0)
			{	    
				goto EXIT_ERR;
			}
			databuf[0] = EM3071_CMM_INT_PS_HT;	
			databuf[1] = (u8)((atomic_read(&obj->ps_thd_val)) & 0x00FF);
			res = i2c_master_send(client, databuf, 0x2);
			if(res <= 0)
			{
				goto EXIT_ERR;
			}
		}

	/*for interrup work mode support */
	res = em3071_setup_eint(client);
	if(res)
	{
		APS_ERR("setup eint: %d\n", res);
		return res;
	}
	res = em3071_check_and_clear_intr(client);
	if(res)
	{
		APS_ERR("check/clear intr: %d\n", res);
		//    return res;
	}
	
	return EM3071_SUCCESS;

EXIT_ERR:
	APS_ERR("init dev: %d\n", res);
	return res;
}


/*----------------------------------------------------------------------------*/
static void em3071_eint_work(struct work_struct *work)
{
#ifdef CUSTOM_KERNEL_SENSORHUB
	int res = 0;

	res = ps_report_interrupt_data(intr_flag_value);
	if (res != 0)
		APS_ERR("em3071_eint_work err: %d\n", res);
#else /* #ifdef CUSTOM_KERNEL_SENSORHUB */
	struct em3071_priv *obj = (struct em3071_priv *)container_of(work, struct em3071_priv, eint_work);
	int res = 0;

	res = em3071_check_and_clear_intr(obj->client);
	if (res != 0)
		goto EXIT_INTR_ERR;
	else {
		APS_LOG("em3071 interrupt value = %d\n", intr_flag_value);
		res = ps_report_interrupt_data(intr_flag_value);
	}

	enable_irq(obj->irq);

	return;
EXIT_INTR_ERR:
	enable_irq(obj->irq);
	APS_ERR("em3071_eint_work err: %d\n", res);
#endif /* #ifdef CUSTOM_KERNEL_SENSORHUB */
}


/****************************************************************************** 
 * Function Configuration
******************************************************************************/
static int em3071_open(struct inode *inode, struct file *file)
{
	file->private_data = em3071_i2c_client;

	if (!file->private_data)
	{
		APS_ERR("null pointer!!\n");
		return -EINVAL;
	}
	
	return nonseekable_open(inode, file);
}
/*----------------------------------------------------------------------------*/
static int em3071_release(struct inode *inode, struct file *file)
{
	file->private_data = NULL;
	return 0;
}


/*************************************************************/

static long em3071_ioctl( struct file *file, unsigned int cmd, unsigned long arg)
{
	struct i2c_client *client = (struct i2c_client*)file->private_data;
	struct em3071_priv *obj = i2c_get_clientdata(client);  
	int err = 0;
	void __user *ptr = (void __user*) arg;
	int dat;
	uint32_t enable;
	int threshold[2];
	int ps_result;

	switch (cmd)
	{
		case ALSPS_SET_PS_MODE:
			if(copy_from_user(&enable, ptr, sizeof(enable)))
			{
				err = -EFAULT;
				goto err_out;
			}
			if(enable)
			{
				err = em3071_enable_ps(obj->client, 1);
				if(err)
				{
					APS_ERR("enable ps fail: %d\n", err); 
					goto err_out;
				}
				
				set_bit(CMC_BIT_PS, &obj->enable);
			}
			else
			{
				err = em3071_enable_ps(obj->client, 0);
				if(err)
				{
					APS_ERR("disable ps fail: %d\n", err); 
					goto err_out;
				}
				
				clear_bit(CMC_BIT_PS, &obj->enable);
			}
			break;

		case ALSPS_GET_PS_MODE:
			enable = test_bit(CMC_BIT_PS, &obj->enable) ? (1) : (0);
			if(copy_to_user(ptr, &enable, sizeof(enable)))
			{
				err = -EFAULT;
				goto err_out;
			}
			break;

		case ALSPS_GET_PS_DATA:    
			err = em3071_read_ps(obj->client, &obj->ps);
			if(err)
			{
				goto err_out;
			}
			
			dat = em3071_get_ps_value(obj, obj->ps);
			if(dat == -1)
			{
				err = -EFAULT;
				goto err_out;
			}
			
			if(copy_to_user(ptr, &dat, sizeof(dat)))
			{
				err = -EFAULT;
				goto err_out;
			}  
			break;

		case ALSPS_GET_PS_RAW_DATA:    
			err = em3071_read_ps(obj->client, &obj->ps);
			if(err)
			{
				goto err_out;
			}
			
			dat = obj->ps;
			if(copy_to_user(ptr, &dat, sizeof(dat)))
			{
				err = -EFAULT;
				goto err_out;
			}  
			break;    

		case ALSPS_SET_PS_THRESHOLD:
			if(copy_from_user(threshold, ptr, sizeof(threshold)))
			{
				err = -EFAULT;
				goto err_out;
			}
			APS_ERR("%s set threshold high: 0x%x, low: 0x%x\n", __func__, threshold[0],threshold[1]); 
			atomic_set(&obj->ps_thd_val_h,  threshold[0]);
			atomic_set(&obj->ps_thd_val_l,  threshold[1]);//need to confirm

 	              obj->hw->ps_threshold = (threshold[0] + threshold[1])/2;
			atomic_set(&obj->ps_thd_val,  obj->hw->ps_threshold);		
			break;
		case ALSPS_GET_PS_THRESHOLD_HIGH:
			dat = atomic_read(&obj->ps_thd_val_h);
			printk("%s:dat:%d\n",__func__,dat);
			if(copy_to_user(ptr, &dat, sizeof(dat)))
			{
				err = -EFAULT;
				goto err_out;
			}  			
			break;
		case ALSPS_GET_PS_THRESHOLD_LOW:
			dat = atomic_read(&obj->ps_thd_val_l);
			printk("%s:dat:%d\n",__func__,dat);			
			if(copy_to_user(ptr, &dat, sizeof(dat)))
			{
				err = -EFAULT;
				goto err_out;
			}  						
			break;	

		case ALSPS_SET_ALS_MODE:
			if(copy_from_user(&enable, ptr, sizeof(enable)))
			{
				err = -EFAULT;
				goto err_out;
			}
			if(enable)
			{
				err = em3071_enable_als(obj->client, 1);
				if(err)
				{
					APS_ERR("enable als fail: %d\n", err); 
					goto err_out;
				}
				set_bit(CMC_BIT_ALS, &obj->enable);
			}
			else
			{
				err = em3071_enable_als(obj->client, 0);
				if(err)
				{
					APS_ERR("disable als fail: %d\n", err); 
					goto err_out;
				}
				clear_bit(CMC_BIT_ALS, &obj->enable);
			}
			break;

		case ALSPS_GET_ALS_MODE:
			enable = test_bit(CMC_BIT_ALS, &obj->enable) ? (1) : (0);
			if(copy_to_user(ptr, &enable, sizeof(enable)))
			{
				err = -EFAULT;
				goto err_out;
			}
			break;

		case ALSPS_GET_ALS_DATA: 
			err = em3071_read_als(obj->client, &obj->als);
			if(err)
			{
				goto err_out;
			}

			dat = em3071_get_als_value(obj, obj->als);
			if(copy_to_user(ptr, &dat, sizeof(dat)))
			{
				err = -EFAULT;
				goto err_out;
			}              
			break;

		case ALSPS_GET_ALS_RAW_DATA:
			err = em3071_read_als(obj->client, &obj->als);
			if(err)
			{
				goto err_out;
			}

			dat = obj->als;
			if(copy_to_user(ptr, &dat, sizeof(dat)))
			{
				err = -EFAULT;
				goto err_out;
			}              
			break;
		case ALSPS_GET_PS_TEST_RESULT:
			err = em3071_read_als(obj->client, &obj->ps);
			if (err)
				goto err_out;

			//if (obj->ps > atomic_read(&obj->ps_thd_val_h))
			//	ps_result = 0;
			//else
				ps_result = 1;
			if (copy_to_user(ptr, &ps_result, sizeof(ps_result))) {
				err = -EFAULT;
				goto err_out;
			}
			break;

		default:
			APS_ERR("%s not supported = 0x%04x", __FUNCTION__, cmd);
			err = -ENOIOCTLCMD;
			break;
	}

	err_out:
	return err;    
}
/*----------------------------------------------------------------------------*/
static const struct file_operations em3071_fops = {
	.owner = THIS_MODULE,
	.open = em3071_open,
	.release = em3071_release,
	.unlocked_ioctl = em3071_ioctl,
};
/*----------------------------------------------------------------------------*/
static struct miscdevice em3071_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "als_ps",
	.fops = &em3071_fops,
};
/*----------------------------------------------------------------------------*/
static int em3071_i2c_suspend(struct i2c_client *client, pm_message_t msg) 
{
//	struct em3071_priv *obj = i2c_get_clientdata(client);    
//	int err;
	APS_FUN(); 
	if(ps_enabled)
	{
		return -EACCES;
	}
	
	return 0;
}
/*----------------------------------------------------------------------------*/
static int em3071_i2c_resume(struct i2c_client *client)
{
//	struct em3071_priv *obj = i2c_get_clientdata(client);        
	APS_FUN();
	return 0;
}
#if defined(CONFIG_HAS_EARLYSUSPEND)
static void em3071_early_suspend(struct early_suspend *h) 
{   /*early_suspend is only applied for ALS*/
	struct em3071_priv *obj = container_of(h, struct em3071_priv, early_drv);   
	int err;
	APS_FUN();    

	if(!obj)
	{
		APS_ERR("null pointer!!\n");
		return;
	}

	#if 1
	
	atomic_set(&obj->als_suspend, 1);
	
	mutex_lock(&sensor_lock);
		if(err = em3071_enable_als(obj->client, 0))
		{
			APS_ERR("disable als fail: %d\n", err); 
		}
	mutex_unlock(&sensor_lock);
		
	#endif
}
/*----------------------------------------------------------------------------*/
static void em3071_late_resume(struct early_suspend *h)
{   /*late_resume is only applied for ALS*/
	struct em3071_priv *obj = container_of(h, struct em3071_priv, early_drv);         
	int err;
	APS_FUN();

	if(!obj)
	{
		APS_ERR("null pointer!!\n");
		return;
	}

        #if 1
		
	atomic_set(&obj->als_suspend, 0);
	if(test_bit(CMC_BIT_ALS, &obj->enable))
	{
		mutex_lock(&sensor_lock);		
		if(err = em3071_enable_als(obj->client, 1))
		{
			APS_ERR("enable als fail: %d\n", err);        

		}
		
		mutex_unlock(&sensor_lock);
	}
	#endif
}
#endif


static int als_open_report_data(int open)
{
	return 0;
}

static int als_enable_nodata(int en)
{
	int res = 0;
#ifdef CUSTOM_KERNEL_SENSORHUB
	SCP_SENSOR_HUB_DATA req;
	int len;
#endif /* #ifdef CUSTOM_KERNEL_SENSORHUB */

	APS_LOG("em3071_obj als enable value = %d\n", en);

#ifdef CUSTOM_KERNEL_SENSORHUB
	if (atomic_read(&em3071_obj->init_done)) {
		req.activate_req.sensorType = ID_LIGHT;
		req.activate_req.action = SENSOR_HUB_ACTIVATE;
		req.activate_req.enable = en;
		len = sizeof(req.activate_req);
		res = SCP_sensorHub_req_send(&req, &len, 1);
	} else
		APS_ERR("sensor hub has not been ready!!\n");

	mutex_lock(&sensor_lock);
	if (en)
		set_bit(CMC_BIT_ALS, &em3071_obj->enable);
	else
		clear_bit(CMC_BIT_ALS, &em3071_obj->enable);
	mutex_unlock(&sensor_lock);
#else /* #ifdef CUSTOM_KERNEL_SENSORHUB */
	mutex_lock(&sensor_lock);
	if (en)
		set_bit(CMC_BIT_ALS, &em3071_obj->enable);
	else
		clear_bit(CMC_BIT_ALS, &em3071_obj->enable);
	mutex_unlock(&sensor_lock);
	if (!em3071_obj) {
		APS_ERR("em3071_obj is null!!\n");
		return -1;
	}
	res = em3071_enable_als(em3071_obj->client, en);
#endif /* #ifdef CUSTOM_KERNEL_SENSORHUB */
	if (res) {
		APS_ERR("als_enable_nodata is failed!!\n");
		return -1;
	}
	return 0;
}

static int als_set_delay(u64 ns)
{
	return 0;
}

static int als_get_data(int *value, int *status)
{
	int err = 0;
#ifdef CUSTOM_KERNEL_SENSORHUB
	SCP_SENSOR_HUB_DATA req;
	int len;
#else
	struct em3071_priv *obj = NULL;
#endif /* #ifdef CUSTOM_KERNEL_SENSORHUB */

#ifdef CUSTOM_KERNEL_SENSORHUB
	if (atomic_read(&em3071_obj->init_done)) {
		req.get_data_req.sensorType = ID_LIGHT;
		req.get_data_req.action = SENSOR_HUB_GET_DATA;
		len = sizeof(req.get_data_req);
		err = SCP_sensorHub_req_send(&req, &len, 1);
	if (err)
		APS_ERR("SCP_sensorHub_req_send fail!\n");
	else {
		*value = req.get_data_rsp.int16_Data[0];
		*status = SENSOR_STATUS_ACCURACY_MEDIUM;
	}

	if (atomic_read(&em3071_obj->trace) & CMC_TRC_PS_DATA)
		APS_LOG("value = %d\n", *value);
	else {
		APS_ERR("sensor hub hat not been ready!!\n");
		err = -1;
	}
#else /* #ifdef CUSTOM_KERNEL_SENSORHUB */
	if (!em3071_obj) {
		APS_ERR("em3071_obj is null!!\n");
		return -1;
	}
	obj = em3071_obj;
	err = em3071_read_als(obj->client, &obj->als);
	if (err)
		err = -1;
	else {
		*value = em3071_get_als_value(obj, obj->als);
		if (*value < 0)
			err = -1;
		*status = SENSOR_STATUS_ACCURACY_MEDIUM;
	}
#endif /* #ifdef CUSTOM_KERNEL_SENSORHUB */

	return err;
}


static int ps_open_report_data(int open)
{
	return 0;
}


static int ps_enable_nodata(int en)
{
	int res = 0;
#ifdef CUSTOM_KERNEL_SENSORHUB
	SCP_SENSOR_HUB_DATA req;
	int len;
#endif /* #ifdef CUSTOM_KERNEL_SENSORHUB */

	APS_LOG("em3071_obj als enable value = %d\n", en);

#ifdef CUSTOM_KERNEL_SENSORHUB
	if (atomic_read(&em3071_obj->init_done)) {
		req.activate_req.sensorType = ID_PROXIMITY;
		req.activate_req.action = SENSOR_HUB_ACTIVATE;
		req.activate_req.enable = en;
		len = sizeof(req.activate_req);
		res = SCP_sensorHub_req_send(&req, &len, 1);
	} else
		APS_ERR("sensor hub has not been ready!!\n");

	mutex_lock(&sensor_lock);
	if (en)
		set_bit(CMC_BIT_PS, &em3071_obj->enable);
	else
		clear_bit(CMC_BIT_PS, &em3071_obj->enable);
	mutex_unlock(&sensor_lock);
#else /* #ifdef CUSTOM_KERNEL_SENSORHUB */
	mutex_lock(&sensor_lock);
	if (en)
		set_bit(CMC_BIT_PS, &em3071_obj->enable);

	else
		clear_bit(CMC_BIT_PS, &em3071_obj->enable);

	mutex_unlock(&sensor_lock);
	if (!em3071_obj) {
		APS_ERR("em3071_obj is null!!\n");
		return -1;
	}
	res = em3071_enable_ps(em3071_obj->client, en);
#endif /* #ifdef CUSTOM_KERNEL_SENSORHUB */

	if (res) {
		APS_ERR("als_enable_nodata is failed!!\n");
		return -1;
	}
	return 0;

}


static int ps_set_delay(u64 ns)
{
	return 0;
}



static int ps_get_data(int *value, int *status)
{
	int err = 0;
#ifdef CUSTOM_KERNEL_SENSORHUB
	SCP_SENSOR_HUB_DATA req;
	int len;
#endif /* #ifdef CUSTOM_KERNEL_SENSORHUB */

#ifdef CUSTOM_KERNEL_SENSORHUB
	if (atomic_read(&em3071_obj->init_done)) {
		req.get_data_req.sensorType = ID_PROXIMITY;
		req.get_data_req.action = SENSOR_HUB_GET_DATA;
		len = sizeof(req.get_data_req);
		err = SCP_sensorHub_req_send(&req, &len, 1);
	if (err) {
		APS_ERR("SCP_sensorHub_req_send fail!\n");
		*value = -1;
		err = -1;
	} else {
		*value = req.get_data_rsp.int16_Data[0];
		*status = SENSOR_STATUS_ACCURACY_MEDIUM;
	}

	if (atomic_read(&em3071_obj->trace) & CMC_TRC_PS_DATA)
		APS_LOG("value = %d\n", *value)
	else {
		APS_ERR("sensor hub has not been ready!!\n");
		err = -1;
	}
#else /* #ifdef CUSTOM_KERNEL_SENSORHUB */
	if (!em3071_obj) {
		APS_ERR("em3071_obj is null!!\n");
		return -1;
	}

	err = em3071_read_ps(em3071_obj->client, &em3071_obj->ps);
	if (err)
		err = -1;
	else {
		*value = em3071_get_ps_value(em3071_obj, em3071_obj->ps);
		if (*value < 0)
			err = -1;
		*status = SENSOR_STATUS_ACCURACY_MEDIUM;
	}
#endif /* #ifdef CUSTOM_KERNEL_SENSORHUB */

	return err;
}



/*----------------------------------------------------------------------------*/
static int em3071_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct em3071_priv *obj;
	struct als_control_path als_ctl = {0};
	struct als_data_path als_data = {0};
	struct ps_control_path ps_ctl = {0};
	struct ps_data_path ps_data = {0};
	int err = 0;

	if(!(obj = kzalloc(sizeof(*obj), GFP_KERNEL)))
	{
		err = -ENOMEM;
		goto exit;
	}
	memset(obj, 0, sizeof(*obj));
	em3071_obj = obj;

	APS_LOG("em3071_init_client() in!\n");
	obj->hw = em3071_get_cust_alsps();
	em3071_get_addr(obj->hw, &obj->addr);

	/*for interrup work mode support --*/
	INIT_WORK(&obj->eint_work, em3071_eint_work);
	
	obj->client = client;
	i2c_set_clientdata(client, obj);	
	atomic_set(&obj->als_debounce, 300);
	atomic_set(&obj->als_deb_on, 0);
	atomic_set(&obj->als_deb_end, 0);
	atomic_set(&obj->ps_debounce, 300);
	atomic_set(&obj->ps_deb_on, 0);
	atomic_set(&obj->ps_deb_end, 0);
	atomic_set(&obj->ps_mask, 0);
	atomic_set(&obj->als_suspend, 0);
	atomic_set(&obj->als_cmd_val, 0xDF);
	atomic_set(&obj->ps_cmd_val,  0xC1);
	obj->hw->ps_threshold = (obj->hw->ps_threshold_high + obj->hw->ps_threshold_low)/2;
	atomic_set(&obj->ps_thd_val,  obj->hw->ps_threshold);
	atomic_set(&obj->ps_thd_val_h,  obj->hw->ps_threshold_high);
	atomic_set(&obj->ps_thd_val_l,  obj->hw->ps_threshold_low);
	obj->enable = 0;
	obj->pending_intr = 0;
	obj->als_level_num = sizeof(obj->hw->als_level)/sizeof(obj->hw->als_level[0]);
	obj->als_value_num = sizeof(obj->hw->als_value)/sizeof(obj->hw->als_value[0]); 
	obj->irq_node = of_find_compatible_node(NULL, NULL, "mediatek, als-eint");
	
	BUG_ON(sizeof(obj->als_level) != sizeof(obj->hw->als_level));
	memcpy(obj->als_level, obj->hw->als_level, sizeof(obj->als_level));
	BUG_ON(sizeof(obj->als_value) != sizeof(obj->hw->als_value));
	memcpy(obj->als_value, obj->hw->als_value, sizeof(obj->als_value));
	atomic_set(&obj->i2c_retry, 3);
	set_bit(CMC_BIT_ALS, &obj->enable);
	set_bit(CMC_BIT_PS, &obj->enable);

	
	em3071_i2c_client = client;

	//client->timing = 20;

	err = em3071_init_client(client);
	if(err)
	{
		goto exit_init_failed;
	}
	APS_LOG("em3071_init_client() OK!\n");

	err = misc_register(&em3071_device);
	if(err)
	{
		APS_ERR("em3071_device register failed\n");
		goto exit_misc_device_register_failed;
	}

#if 0
	err = em30713_create_attr(&(em3071_init_info.platform_diver_addr->driver));
	if(err)
	{
		goto exit_create_attr_failed;
	}
#endif	

	als_ctl.open_report_data = als_open_report_data;
	als_ctl.enable_nodata = als_enable_nodata;
	als_ctl.set_delay  = als_set_delay;
	als_ctl.is_report_input_direct = false;
#ifdef CUSTOM_KERNEL_SENSORHUB
	als_ctl.is_support_batch = obj->hw->is_batch_supported_als;
#else
	als_ctl.is_support_batch = false;
#endif

	err = als_register_control_path(&als_ctl);
	if (err) {
		APS_ERR("register fail = %d\n", err);
		goto exit_sensor_obj_attach_fail;
	}

	als_data.get_data = als_get_data;
	als_data.vender_div = 100;
	err = als_register_data_path(&als_data);
	if (err) {
		APS_ERR("tregister fail = %d\n", err);
		goto exit_sensor_obj_attach_fail;
	}

	ps_ctl.open_report_data = ps_open_report_data;
	ps_ctl.enable_nodata = ps_enable_nodata;
	ps_ctl.set_delay  = ps_set_delay;
	ps_ctl.is_report_input_direct = false;
#ifdef CUSTOM_KERNEL_SENSORHUB
	ps_ctl.is_support_batch = obj->hw->is_batch_supported_ps;
#else
	ps_ctl.is_support_batch = false;
#endif
	err = ps_register_control_path(&ps_ctl);
	if (err) {
		APS_ERR("register fail = %d\n", err);
		goto exit_sensor_obj_attach_fail;
	}

	ps_data.get_data = ps_get_data;
	ps_data.vender_div = 100;
	err = ps_register_data_path(&ps_data);
	if (err) {
		APS_ERR("tregister fail = %d\n", err);
		goto exit_sensor_obj_attach_fail;
	}

	err = batch_register_support_info(ID_LIGHT, als_ctl.is_support_batch, 1, 0);
	if (err)
		APS_ERR("register light batch support err = %d\n", err);

	err = batch_register_support_info(ID_PROXIMITY, ps_ctl.is_support_batch, 1, 0);
	if (err)
		APS_ERR("register proximity batch support err = %d\n", err);
	


#if defined(CONFIG_HAS_EARLYSUSPEND)
	obj->early_drv.level    = EARLY_SUSPEND_LEVEL_DISABLE_FB - 1,
	obj->early_drv.suspend  = em3071_early_suspend,
	obj->early_drv.resume   = em3071_late_resume,    
	register_early_suspend(&obj->early_drv);
#endif

	APS_LOG("%s: OK\n", __func__);
	em3071_init_flag = 0;
	return 0;
	
exit_sensor_obj_attach_fail:
//exit_create_attr_failed:
	misc_deregister(&em3071_device);
exit_misc_device_register_failed:
exit_init_failed:
	//i2c_detach_client(client);
	kfree(obj);
exit:
	em3071_i2c_client = NULL;           
	
	APS_ERR("%s: err = %d\n", __func__, err);
	em3071_init_flag = -1;

	return err;
}
/*----------------------------------------------------------------------------*/
static int em3071_i2c_remove(struct i2c_client *client)
{
	int err;	

#if 0
	err = em30713_delete_attr(&(em3071_init_info.platform_diver_addr->driver));
	if (err)
		APS_ERR("em30713_delete_attr fail: %d\n", err);
#endif

	err = misc_deregister(&em3071_device);
	if(err)
	{
		APS_ERR("misc_deregister fail: %d\n", err);    
	}
	
	em3071_i2c_client = NULL;
	i2c_unregister_device(client);
	kfree(i2c_get_clientdata(client));

	return 0;
}
/*----------------------------------------------------------------------------*/

static int em3071_local_init(void)
{
    struct alsps_hw *hw = em3071_get_cust_alsps();

	em3071_power(hw, 1);
	if(i2c_add_driver(&em3071_i2c_driver))
	{
		APS_ERR("add driver error\n");
		return -1;
	}
	if(-1 == em3071_init_flag)
	{
	   return -1;
	}

	return 0;
}

static int em3071_remove(void)
{
	em3071_power(hw, 0);

	i2c_del_driver(&em3071_i2c_driver);
	return 0;
}

static int __init em3071_init(void)
{
	const char *name = "mediatek,EM30718";

	hw = get_alsps_dts_func(name, hw);
	if (!hw)
		APS_ERR("em30718 get dts info fail\n");

	alsps_driver_add(&em3071_init_info);

	return 0;
}
/*----------------------------------------------------------------------------*/
static void __exit em3071_exit(void)
{
}
/*----------------------------------------------------------------------------*/
module_init(em3071_init);
module_exit(em3071_exit);
/*----------------------------------------------------------------------------*/
MODULE_AUTHOR("binghua chen");
MODULE_DESCRIPTION("em30718 driver");
MODULE_LICENSE("GPL");
