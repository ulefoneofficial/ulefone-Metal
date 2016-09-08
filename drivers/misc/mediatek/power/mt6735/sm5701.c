#include <linux/types.h>
#include <linux/init.h>		/* For init/exit macros */
#include <linux/module.h>	/* For MODULE_ marcros  */
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#endif

#include "sm5701.h"
#include <mach/mt_charging.h>
#include <mt-plat/charging.h>

/**********************************************************
  *
  *   [I2C Slave Setting] 
  *
  *********************************************************/
#define SM5701_CHG_SLAVE_ADDR_WRITE   0x92
#define SM5701_CHG_SLAVE_ADDR_Read    0x93

static struct i2c_client *new_client = NULL;
static const struct i2c_device_id sm5701_i2c_id[] = {{"sm5701",0},{}};   
kal_bool chargin_hw_init_done = KAL_FALSE; 
static int sm5701_chg_driver_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int sm5701_driver_remove(struct i2c_client *client);
static unsigned int current_chg_mode = 0;
static unsigned int current_otg_mode = 0;

#if !defined(CONFIG_MTK_LEGACY)
static const struct of_device_id sm5701_of_match[] = {
	{.compatible = "mediatek,SWITHING_CHARGER"},
	{},
};
#endif

static struct i2c_driver sm5701_chg_driver = {
    .id_table    = sm5701_i2c_id,
    .probe       = sm5701_chg_driver_probe,
    .remove    = sm5701_driver_remove,
    .driver = {
        .owner = THIS_MODULE,
        .name    = "sm5701",
#if !defined(CONFIG_MTK_LEGACY)
        .of_match_table = sm5701_of_match,
#endif
    	},			
};
/**********************************************************
  *
  *   [Global Variable] 
  *
  *********************************************************/
unsigned char sm5701_chg_reg[SM5701_REG_NUM] = {0};

static DEFINE_MUTEX(sm5701_chg_i2c_access);
/**********************************************************
  *
  *   [I2C Function For Read/Write sm5701_chg] 
  *
  *********************************************************/
int sm5701_chg_read_byte(unsigned char cmd, unsigned char *returnData)
{
    char     cmd_buf[1]={0x00};
    char     readData = 0;
    int      ret=0;

    mutex_lock(&sm5701_chg_i2c_access);
    
    //new_client->addr = ((new_client->addr) & I2C_MASK_FLAG) | I2C_WR_FLAG;    
    new_client->ext_flag=((new_client->ext_flag ) & I2C_MASK_FLAG ) | I2C_WR_FLAG | I2C_DIRECTION_FLAG;

    cmd_buf[0] = cmd;
    ret = i2c_master_send(new_client, &cmd_buf[0], (1<<8 | 1));
    if (ret < 0) 
    {    
        //new_client->addr = new_client->addr & I2C_MASK_FLAG;
        new_client->ext_flag=0;

        mutex_unlock(&sm5701_chg_i2c_access);
        return 0;
    }
    
    readData = cmd_buf[0];
    *returnData = readData;

    // new_client->addr = new_client->addr & I2C_MASK_FLAG;
    new_client->ext_flag=0;
    
    mutex_unlock(&sm5701_chg_i2c_access);    
    return 1;
}

int sm5701_chg_write_byte(unsigned char cmd, unsigned char writeData)
{
    char    write_data[2] = {0};
    int     ret=0;
    
    mutex_lock(&sm5701_chg_i2c_access);
    
    write_data[0] = cmd;
    write_data[1] = writeData;
    
    new_client->ext_flag=((new_client->ext_flag ) & I2C_MASK_FLAG ) | I2C_DIRECTION_FLAG;
    
    ret = i2c_master_send(new_client, write_data, 2);
    if (ret < 0) 
    {
       
        new_client->ext_flag=0;
        mutex_unlock(&sm5701_chg_i2c_access);
        return 0;
    }
    
    new_client->ext_flag=0;
    mutex_unlock(&sm5701_chg_i2c_access);
    return 1;
}

/**********************************************************
  *
  *   [Read / Write Function] 
  *
  *********************************************************/
unsigned int sm5701_chg_read_interface (unsigned char RegNum, unsigned char *val, unsigned char MASK, unsigned char SHIFT)
{
    unsigned char sm5701_chg_reg = 0;
    int ret = 0;

   battery_xlog_printk(BAT_LOG_FULL,"--------------------------------------------------\n");

    ret = sm5701_chg_read_byte(RegNum, &sm5701_chg_reg);

	battery_xlog_printk(BAT_LOG_FULL,"[sm5701_chg_read_interface] Reg[%x]=0x%x\n", RegNum, sm5701_chg_reg);
	
    sm5701_chg_reg &= (MASK << SHIFT);
    *val = (sm5701_chg_reg >> SHIFT);
	
	battery_xlog_printk(BAT_LOG_FULL,"[sm5701_chg_read_interface] val=0x%x\n", *val);
	
    return ret;
}

unsigned int sm5701_chg_config_interface (unsigned char RegNum, unsigned char val, unsigned char MASK, unsigned char SHIFT)
{
    unsigned char sm5701_chg_reg = 0;
    int ret = 0;

    battery_xlog_printk(BAT_LOG_FULL,"--------------------------------------------------\n");

    ret = sm5701_chg_read_byte(RegNum, &sm5701_chg_reg);
    battery_xlog_printk(BAT_LOG_FULL,"[sm5701_chg_config_interface] Reg[%x]=0x%x\n", RegNum, sm5701_chg_reg);
    
    sm5701_chg_reg &= ~(MASK << SHIFT);
    sm5701_chg_reg |= (val << SHIFT);

	if(RegNum == SM5701_CNTL && val == 1 && MASK ==SM5701_CNTL_RESET_MASK && SHIFT == SM5701_CNTL_RESET_SHIFT)
	{
		// RESET bit
	}
	else if(RegNum == SM5701_CNTL)
	{
		sm5701_chg_reg &= ~0x08;	//RESET bit read returs 1, so clear it
	}
	 

    ret = sm5701_chg_write_byte(RegNum, sm5701_chg_reg);
    battery_xlog_printk(BAT_LOG_FULL,"[sm5701_chg_config_interface] write Reg[%x]=0x%x\n", RegNum, sm5701_chg_reg);

    // Check
    //sm5701_chg_read_byte(RegNum, &sm5701_chg_reg);
    //printk("[sm5701_chg_config_interface] Check Reg[%x]=0x%x\n", RegNum, sm5701_chg_reg);

    return ret;
}

//write one register directly
unsigned int sm5701_chg_reg_config_interface (unsigned char RegNum, unsigned char val)
{   
    int ret = 0;
    
    ret = sm5701_chg_write_byte(RegNum, val);

    return ret;
}

/**********************************************************
  *
  *   [Internal Function] 
  *
  *********************************************************/
//STATUS2----------------------------------------------------
unsigned int sm5701_chg_get_topoff_status(void)
{
    unsigned char val=0;
    unsigned int ret=0; 
 
    ret=sm5701_chg_read_interface(     (unsigned char)(SM5701_STATUS2), 
                                    (&val),
                                    (unsigned char)(SM5701_STATUS2_TOPOFF_MASK),
                                    (unsigned char)(SM5701_STATUS2_TOPOFF_SHIFT)
                                    );
    return val;
   // return 0;
}


//CNTL----------------------------------------------------
void sm5701_chg_set_operationmode(unsigned int val)
{
    unsigned int ret=0;    

    ret=sm5701_chg_config_interface(   (unsigned char)(SM5701_CNTL), 
                                    (unsigned char)(val),
                                    (unsigned char)(SM5701_CNTL_OPERATIONMODE_MASK),
                                    (unsigned char)(SM5701_CNTL_OPERATIONMODE_SHIFT)
                                    );
}

void sm5701_chg_set_reset(unsigned int val)
{
    unsigned int ret=0;    

    ret=sm5701_chg_config_interface(   (unsigned char)(SM5701_CNTL), 
                                    (unsigned char)(val),
                                    (unsigned char)(SM5701_CNTL_RESET_MASK),
                                    (unsigned char)(SM5701_CNTL_RESET_SHIFT)
                                    );
}

void sm5701_chg_set_ovpsel(unsigned int val)
{
    unsigned int ret=0;    

    ret=sm5701_chg_config_interface(   (unsigned char)(SM5701_CNTL), 
                                    (unsigned char)(val),
                                    (unsigned char)(SM5701_CNTL_OVPSEL_MASK),
                                    (unsigned char)(SM5701_CNTL_OVPSEL_SHIFT)
                                    );
}

void sm5701_chg_set_freqsel(unsigned int val)
{
    unsigned int ret=0;    

    ret=sm5701_chg_config_interface(   (unsigned char)(SM5701_CNTL), 
                                    (unsigned char)(val),
                                    (unsigned char)(SM5701_CNTL_FREQSEL_MASK),
                                    (unsigned char)(SM5701_CNTL_FREQSEL_SHIFT)
                                    );
}


//VBUSCNTL----------------------------------------------------
void sm5701_chg_set_vbuslimit(unsigned int val)
{
    unsigned int ret=0;    

    ret=sm5701_chg_config_interface(   (unsigned char)(SM5701_VBUSCNTL), 
                                    (unsigned char)(val),
                                    (unsigned char)(SM5701_VBUSCNTL_VBUSLIMIT_MASK),
                                    (unsigned char)(SM5701_VBUSCNTL_VBUSLIMIT_SHIFT)
                                    );
}

void sm5701_chg_set_aiclth(unsigned int val)
{
    unsigned int ret=0;    

    ret=sm5701_chg_config_interface(   (unsigned char)(SM5701_VBUSCNTL), 
                                    (unsigned char)(val),
                                    (unsigned char)(SM5701_VBUSCNTL_AICLTH_MASK),
                                    (unsigned char)(SM5701_VBUSCNTL_AICLTH_SHIFT)
                                    );
}

void sm5701_chg_set_aiclen(unsigned int val)
{
    unsigned int ret=0;    

    ret=sm5701_chg_config_interface(   (unsigned char)(SM5701_VBUSCNTL), 
                                    (unsigned char)(val),
                                    (unsigned char)(SM5701_VBUSCNTL_AICLEN_MASK),
                                    (unsigned char)(SM5701_VBUSCNTL_AICLEN_SHIFT)
                                    );
}


//CHGCNTL1----------------------------------------------------
void sm5701_chg_set_topoff(unsigned int val)
{
    unsigned int ret=0;    

    ret=sm5701_chg_config_interface(   (unsigned char)(SM5701_CHGCNTL1), 
                                    (unsigned char)(val),
                                    (unsigned char)(SM5701_CHGCNTL1_TOPOFF_MASK),
                                    (unsigned char)(SM5701_CHGCNTL1_TOPOFF_SHIFT)
                                    );
}

void sm5701_chg_set_autostop(unsigned int val)
{
    unsigned int ret=0;    

    ret=sm5701_chg_config_interface(   (unsigned char)(SM5701_CHGCNTL1), 
                                    (unsigned char)(val),
                                    (unsigned char)(SM5701_CHGCNTL1_AUTOSTOP_MASK),
                                    (unsigned char)(SM5701_CHGCNTL1_AUTOSTOP_SHIFT)
                                    );
}

void sm5701_chg_set_hsslpctrl(unsigned int val)
{
    unsigned int ret=0;    

    ret=sm5701_chg_config_interface(   (unsigned char)(SM5701_CHGCNTL1), 
                                    (unsigned char)(val),
                                    (unsigned char)(SM5701_CHGCNTL1_HSSLPCTRL_MASK),
                                    (unsigned char)(SM5701_CHGCNTL1_HSSLPCTRL_SHIFT)
                                    );
}


//CHGCNTL2----------------------------------------------------
void sm5701_chg_set_fastchg(unsigned int val)
{
    unsigned int ret=0;

    ret=sm5701_chg_config_interface(     (unsigned char)(SM5701_CHGCNTL2), 
                                    (unsigned char)(val),
                                    (unsigned char)(SM5701_CHGCNTL2_FASTCHG_MASK),
                                    (unsigned char)(SM5701_CHGCNTL2_FASTCHG_SHIFT)
                                    );
}


//CHGCNTL3----------------------------------------------------
void sm5701_chg_set_batreg(unsigned int val)
{
    unsigned int ret=0;    

    ret=sm5701_chg_config_interface(   (unsigned char)(SM5701_CHGCNTL3), 
                                    (unsigned char)(val),
                                    (unsigned char)(SM5701_CHGCNTL3_BATREG_MASK),
                                    (unsigned char)(SM5701_CHGCNTL3_BATREG_SHIFT)
                                    );
}


//CHGCNTL4----------------------------------------------------
void sm5701_chg_set_topofftimer(unsigned int val)
{
    unsigned int ret=0;    

    ret=sm5701_chg_config_interface(   (unsigned char)(SM5701_CHGCNTL4), 
                                    (unsigned char)(val),
                                    (unsigned char)(SM5701_CHGCNTL4_TOPOFFTIMER_MASK),
                                    (unsigned char)(SM5701_CHGCNTL4_TOPOFFTIMER_SHIFT)
                                    );
}

void sm5701_chg_set_fasttimer(unsigned int val)
{
    unsigned int ret=0;    

    ret=sm5701_chg_config_interface(   (unsigned char)(SM5701_CHGCNTL4), 
                                    (unsigned char)(val),
                                    (unsigned char)(SM5701_CHGCNTL4_FASTTIMER_MASK),
                                    (unsigned char)(SM5701_CHGCNTL4_FASTTIMER_SHIFT)
                                    );
}

//FLEDCNTL6----------------------------------------------------
void sm5701_chg_set_bstout(unsigned int val)
{
    unsigned int ret=0;    

    ret=sm5701_chg_config_interface(   (unsigned char)(SM5701_FLEDCNTL6), 
                                    (unsigned char)(val),
                                    (unsigned char)(SM5701_FLEDCNTL6_BSTOUT_MASK),
                                    (unsigned char)(SM5701_FLEDCNTL6_BSTOUT_SHIFT)
                                    );
}

int sm5701_charger_get_chgmode(void)
{
    battery_xlog_printk(BAT_LOG_FULL,"%s current_chg_mode = %d\n",__func__,current_chg_mode);

    return current_chg_mode;
}

int sm5701_charger_get_otgmode(void)
{
    battery_xlog_printk(BAT_LOG_FULL,"%s current_otg_mode = %d\n",__func__,current_otg_mode);

    return current_otg_mode;
}

int sm5701_charger_notification(int on)
{
    unsigned char vbus_valid = 0;
	int mode = 0;//sm5701_fled_get_mode();

    sm5701_chg_read_byte(SM5701_STATUS1, &vbus_valid);

    printk("%s on = %d, mode = %d\n",__func__,on, mode);
    
	if (on == 1) {
		if (mode == FLASHLIGHT_MODE_MOVIE )
        {
            sm5701_chg_set_bstout(SM5701_BSTOUT_4P5);
			sm5701_chg_set_operationmode(SM5701_OPERATIONMODE_CHARGER_ON_FLASH_ON);
        }
        else if (mode == FLASHLIGHT_MODE_FLASH )
        {            
            sm5701_chg_set_bstout(SM5701_BSTOUT_4P5);
			sm5701_chg_set_operationmode(SM5701_OPERATIONMODE_FLASH_ON);
        }
        else
        {        
            sm5701_chg_set_bstout(SM5701_BSTOUT_4P5);
			sm5701_chg_set_operationmode(SM5701_OPERATIONMODE_CHARGER_ON);
        }
	}else {
		if (mode == FLASHLIGHT_MODE_MOVIE )
        {        
            sm5701_chg_set_bstout(SM5701_BSTOUT_4P5);
			sm5701_chg_set_operationmode(SM5701_OPERATIONMODE_FLASH_ON);
        }
        else if (mode == FLASHLIGHT_MODE_FLASH )
        {        
            sm5701_chg_set_bstout(SM5701_BSTOUT_4P5);
			sm5701_chg_set_operationmode(SM5701_OPERATIONMODE_FLASH_ON);
        }
        else
        {        
            sm5701_chg_set_bstout(SM5701_BSTOUT_4P5);
			sm5701_chg_set_operationmode(SM5701_OPERATIONMODE_CHARGER_ON);
        }
    }

    current_chg_mode = on;
    
	return 0;
}

int sm5701_boost_notification(int on)
{
    int mode = 0;//sm5701_fled_get_mode();

    printk("%s on = %d, mode = %d\n",__func__,on, mode);

	if (on == 1) {
		if (mode == FLASHLIGHT_MODE_MOVIE )
        {            
            sm5701_chg_set_bstout(SM5701_BSTOUT_4P5);
			sm5701_chg_set_operationmode(SM5701_OPERATIONMODE_CHARGER_ON_FLASH_ON);
        }
        else if (mode == FLASHLIGHT_MODE_FLASH )
        {    
            sm5701_chg_set_bstout(SM5701_BSTOUT_4P5);
			sm5701_chg_set_operationmode(SM5701_OPERATIONMODE_CHARGER_ON_FLASH_ON);
        }
        else
        {        
            sm5701_chg_set_bstout(SM5701_BSTOUT_5P0);
			sm5701_chg_set_operationmode(SM5701_OPERATIONMODE_OTG_ON);
        }
	}else if(on == 0) {
		if (mode == FLASHLIGHT_MODE_MOVIE )			
        {            
            sm5701_chg_set_bstout(SM5701_BSTOUT_4P5);
            sm5701_chg_set_operationmode(SM5701_OPERATIONMODE_CHARGER_ON_FLASH_ON);
        }
        else if (mode == FLASHLIGHT_MODE_FLASH )
        {            
            sm5701_chg_set_bstout(SM5701_BSTOUT_4P5);
            sm5701_chg_set_operationmode(SM5701_OPERATIONMODE_CHARGER_ON_FLASH_ON);
        }
        else
        {            
            sm5701_chg_set_bstout(SM5701_BSTOUT_4P5);
			sm5701_chg_set_operationmode(SM5701_OPERATIONMODE_CHARGER_ON);
        }
    }

    current_otg_mode = on;
    
	return 0;
}


/**********************************************************
  *
  *   [Internal Function] 
  *
  *********************************************************/
void sm5701_chg_dump_register(void)
{
    int i=0;
    printk("[sm5701_chg] ");
    for (i=SM5701_INTMASK1;i<SM5701_REG_NUM;i++)
    {
        sm5701_chg_read_byte(i, &sm5701_chg_reg[i]);
        printk("[0x%x]=0x%x ", i, sm5701_chg_reg[i]);        
    }
    printk("\n");
}

#if 0
extern int g_enable_high_vbat_spec;
extern int g_pmic_cid;

void sm5701_chg_hw_init(void)
{    
    if(g_enable_high_vbat_spec == 1)
    {
        if(g_pmic_cid == 0x1020)
        {
            printk("[sm5701_chg_hw_init] (0x06,0x70) because 0x1020\n");
            sm5701_chg_reg_config_interface(0x06,0x70); // set ISAFE
        }
        else
        {
            printk("[sm5701_chg_hw_init] (0x06,0x77)\n");
            sm5701_chg_reg_config_interface(0x06,0x77); // set ISAFE and HW CV point (4.34)
        }
    }
    else
    {
        printk("[sm5701_chg_hw_init] (0x06,0x70) \n");
        sm5701_chg_reg_config_interface(0x06,0x70); // set ISAFE
    }
}
#endif
#if defined(LYCONFIG_FACTORY_BATTERY_TEST)
int sm5701_test_flag = 0;

int sm5701_flag_handler(void)
{
  return sm5701_test_flag;
}

module_param(sm5701_test_flag, int, 00664);
#endif
static int sm5701_chg_driver_probe(struct i2c_client *client, const struct i2c_device_id *id) 
{             
    int err=0; 
    #if defined(LYCONFIG_FACTORY_BATTERY_TEST)
    	int flag=0;
    #endif
    unsigned char vbus_valid_new = 0;

    battery_xlog_printk(BAT_LOG_CRTI,"[sm5701_chg_driver_probe] \n");

    if (!(new_client = kmalloc(sizeof(struct i2c_client), GFP_KERNEL))) {
        err = -ENOMEM;
        goto exit;
    }    
    memset(new_client, 0, sizeof(struct i2c_client));

    new_client = client;    

    //---------------------
  //  sm5701_chg_hw_init();
    sm5701_chg_dump_register();
    #if defined(LYCONFIG_FACTORY_BATTERY_TEST)
	    flag = sm5701_chg_read_byte(0x00,&vbus_valid_new);
	    sm5701_test_flag = flag ;
    #endif
    chargin_hw_init_done = KAL_TRUE;
	
    return 0;                                                                                       

exit:
    return err;

}

static int sm5701_driver_remove(struct i2c_client *client)
{
	//printk("zhangchongyong1 %s:\n", __func__);
	new_client = NULL;
	i2c_unregister_device(client);
	return 0;
}
/**********************************************************
  *
  *   [platform_driver API] 
  *
  *********************************************************/
unsigned char g_reg_value_sm5701_chg=0;
static ssize_t show_sm5701_chg_access(struct device *dev,struct device_attribute *attr, char *buf)
{
    battery_xlog_printk(BAT_LOG_FULL,"[show_sm5701_chg_access] 0x%x\n", g_reg_value_sm5701_chg);
    return sprintf(buf, "%u\n", g_reg_value_sm5701_chg);
}
static ssize_t store_sm5701_chg_access(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    int ret=0;
    char *pvalue = NULL;
    unsigned int reg_value = 0;
    unsigned int reg_address = 0;
    
    battery_xlog_printk(BAT_LOG_FULL,"[store_sm5701_chg_access] \n");
    
    if(buf != NULL && size != 0)
    {
        battery_xlog_printk(BAT_LOG_FULL,"[store_sm5701_chg_access] buf is %s and size is %d \n",buf,(int)size);
        reg_address = simple_strtoul(buf,&pvalue,16);
        
        if(size > 3)
        {        
            reg_value = simple_strtoul((pvalue+1),NULL,16);        
            battery_xlog_printk(BAT_LOG_FULL,"[store_sm5701_chg_access] write sm5701_chg reg 0x%x with value 0x%x !\n",reg_address,reg_value);
            ret=sm5701_chg_config_interface(reg_address, reg_value, 0xFF, 0x0);
        }
        else
        {    
            ret=sm5701_chg_read_interface(reg_address, &g_reg_value_sm5701_chg, 0xFF, 0x0);
            battery_xlog_printk(BAT_LOG_FULL,"[store_sm5701_chg_access] read sm5701_chg reg 0x%x with value 0x%x !\n",reg_address,g_reg_value_sm5701_chg);
            battery_xlog_printk(BAT_LOG_FULL,"[store_sm5701_chg_access] Please use \"cat sm5701_chg_access\" to get value\r\n");
        }        
    }    
    return size;
}
static DEVICE_ATTR(sm5701_chg_access, 0664, show_sm5701_chg_access, store_sm5701_chg_access); //664

static int sm5701_chg_user_space_probe(struct platform_device *dev)    
{    
    int ret_device_file = 0;

    battery_xlog_printk(BAT_LOG_CRTI,"******** sm5701_chg_user_space_probe!! ********\n" );
    
    ret_device_file = device_create_file(&(dev->dev), &dev_attr_sm5701_chg_access);
    
    return 0;
}

struct platform_device sm5701_chg_user_space_device = {
    .name   = "sm5701-chg-user",
    .id     = -1,
};

static struct platform_driver sm5701_chg_user_space_driver = {
    .probe      = sm5701_chg_user_space_probe,
    .driver     = {
        .name = "sm5701-chg-user",
    },
};


//static struct i2c_board_info __initdata i2c_sm5701_chg = { I2C_BOARD_INFO("sm5701_chg", (SM5701_CHG_SLAVE_ADDR_WRITE>>1))};

static int __init sm5701_chg_init(void)
{    
    int ret=0;
    
    battery_xlog_printk(BAT_LOG_CRTI,"[sm5701_chg_init] init start\n");
    
    //i2c_register_board_info(SM5701_BUSNUM, &i2c_sm5701_chg, 1);//Siliconmitus : SM5701_CHG_BUSNUM setting

    if(i2c_add_driver(&sm5701_chg_driver)!=0)
    {
        battery_xlog_printk(BAT_LOG_CRTI,"[sm5701_chg_init] failed to register sm5701_chg i2c driver.\n");
    }
    else
    {
        battery_xlog_printk(BAT_LOG_CRTI,"[sm5701_chg_init] Success to register sm5701_chg i2c driver.\n");
    }

    // sm5701_chg user space access interface
    ret = platform_device_register(&sm5701_chg_user_space_device);
    if (ret) {
        battery_xlog_printk(BAT_LOG_CRTI,"****[sm5701_chg_init] Unable to device register(%d)\n", ret);
        return ret;
    }    
    ret = platform_driver_register(&sm5701_chg_user_space_driver);
    if (ret) {
        battery_xlog_printk(BAT_LOG_CRTI,"****[sm5701_chg_init] Unable to register driver (%d)\n", ret);
        return ret;
    }
    
    return 0;        
}

static void __exit sm5701_chg_exit(void)
{
    i2c_del_driver(&sm5701_chg_driver);
}

module_init(sm5701_chg_init);
module_exit(sm5701_chg_exit);
   
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("I2C sm5701_chg Driver");
