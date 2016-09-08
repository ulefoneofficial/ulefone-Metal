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

#include "sm5414.h"
#include <mach/mt_charging.h>
#include <mt-plat/charging.h>


/**********************************************************
  *
  *   [I2C Slave Setting] 
  *
  *********************************************************/
#define SM5414_CHG_SLAVE_ADDR_WRITE   0x92
#define SM5414_CHG_SLAVE_ADDR_Read    0x93
#define SM5414_BUSNUM   3

static struct i2c_client *new_client = NULL;
static const struct i2c_device_id sm5414_i2c_id[] = {{"sm5414",0},{}};   
kal_bool chargin_hw_init_done = KAL_FALSE; 
static int sm5414_driver_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int sm5414_driver_remove(struct i2c_client *client);
//static u32 current_chg_mode = 0;
static u32 current_otg_mode = 0;

#if !defined(CONFIG_MTK_LEGACY)
static const struct of_device_id sm5414_of_match[] = {
	{.compatible = "mediatek,SWITHING_CHARGER"},
	{},
};
#endif

static struct i2c_driver sm5414_driver = {
    .id_table    = sm5414_i2c_id,
    .probe       = sm5414_driver_probe,
    .remove    = sm5414_driver_remove,
    .driver = {
        .owner = THIS_MODULE,
        .name    = "sm5414",
#if !defined(CONFIG_MTK_LEGACY)
        .of_match_table = sm5414_of_match,
#endif
    	},			
};

/**********************************************************
  *
  *   [Global Variable] 
  *
  *********************************************************/
unsigned char sm5414_reg[SM5414_REG_NUM] = {0};

static DEFINE_MUTEX(sm5414_i2c_access);
/**********************************************************
  *
  *   [I2C Function For Read/Write sm5414] 
  *
  *********************************************************/
int sm5414_read_byte(unsigned char cmd, unsigned char *returnData)
{
    char     cmd_buf[1]={0x00};
    char     readData = 0;
    int      ret=0;

    mutex_lock(&sm5414_i2c_access);
    
    //new_client->addr = ((new_client->addr) & I2C_MASK_FLAG) | I2C_WR_FLAG;    
    new_client->ext_flag=((new_client->ext_flag ) & I2C_MASK_FLAG ) | I2C_WR_FLAG | I2C_DIRECTION_FLAG;
    //new_client->addr = 0x49;
    cmd_buf[0] = cmd;
    ret = i2c_master_send(new_client, &cmd_buf[0], (1<<8 | 1));
    if (ret < 0) 
    {    
        //new_client->addr = new_client->addr & I2C_MASK_FLAG;
        new_client->ext_flag=0;

        mutex_unlock(&sm5414_i2c_access);
        return 0;
    }
    
    readData = cmd_buf[0];
    *returnData = readData;

    // new_client->addr = new_client->addr & I2C_MASK_FLAG;
    new_client->ext_flag=0;
    
    mutex_unlock(&sm5414_i2c_access);    
    return 1;
}

int sm5414_write_byte(unsigned char cmd, unsigned char writeData)
{
    char    write_data[2] = {0};
    int     ret=0;
    
    mutex_lock(&sm5414_i2c_access);
    
    write_data[0] = cmd;
    write_data[1] = writeData;
    
    new_client->ext_flag=((new_client->ext_flag ) & I2C_MASK_FLAG ) | I2C_DIRECTION_FLAG;
    //new_client->addr = 0x49;
    ret = i2c_master_send(new_client, write_data, 2);
    if (ret < 0) 
    {
       
        new_client->ext_flag=0;
        mutex_unlock(&sm5414_i2c_access);
        return 0;
    }
    
    new_client->ext_flag=0;
    mutex_unlock(&sm5414_i2c_access);
    return 1;
}

/**********************************************************
  *
  *   [Read / Write Function] 
  *
  *********************************************************/
unsigned int sm5414_read_interface (unsigned char RegNum, unsigned char *val, unsigned char MASK, unsigned char SHIFT)
{
    unsigned char sm5414_reg = 0;
    int ret = 0;

   battery_xlog_printk(BAT_LOG_FULL,"--------------------------------------------------\n");

    ret = sm5414_read_byte(RegNum, &sm5414_reg);

	battery_xlog_printk(BAT_LOG_FULL,"[sm5414_read_interface] Reg[%x]=0x%x\n", RegNum, sm5414_reg);
	
    sm5414_reg &= (MASK << SHIFT);
    *val = (sm5414_reg >> SHIFT);
	
	battery_xlog_printk(BAT_LOG_FULL,"[sm5414_read_interface] val=0x%x\n", *val);
	
    return ret;
}

unsigned int sm5414_config_interface (unsigned char RegNum, unsigned char val, unsigned char MASK, unsigned char SHIFT)
{
    unsigned char sm5414_reg = 0;
    int ret = 0;

    battery_xlog_printk(BAT_LOG_FULL,"--------------------------------------------------\n");

    ret = sm5414_read_byte(RegNum, &sm5414_reg);
    //printk("[sm5414_config_interface] Reg[%x]=0x%x\n", RegNum, sm5414_reg);
    
    sm5414_reg &= ~(MASK << SHIFT);
    sm5414_reg |= (val << SHIFT);	 

    ret = sm5414_write_byte(RegNum, sm5414_reg);
    //printk("[sm5414_config_interface] write Reg[%x]=0x%x\n", RegNum, sm5414_reg);

    // Check
    //sm5414_read_byte(RegNum, &sm5414_reg);
    //printk("[sm5414_config_interface] Check Reg[%x]=0x%x\n", RegNum, sm5414_reg);

    return ret;
}

//write one register directly
unsigned int sm5414_reg_config_interface (unsigned char RegNum, unsigned char val)
{   
    int ret = 0;
    
    ret = sm5414_write_byte(RegNum, val);

    return ret;
}

/**********************************************************
  *
  *   [Internal Function] 
  *
  *********************************************************/
//STATUS2----------------------------------------------------
unsigned int sm5414_get_topoff_status(void)
{
    unsigned char val=0;
    unsigned int ret=0; 
 
    ret=sm5414_read_interface(     (unsigned char)(SM5414_STATUS), 
                                    (&val),
                                    (unsigned char)(SM5414_STATUS_TOPOFF_MASK),
                                    (unsigned char)(SM5414_STATUS_TOPOFF_SHIFT)
                                    );
    return val;
   // return 0;
}

//CTRL----------------------------------------------------
void sm5414_set_enboost(unsigned int val)
{
    unsigned int ret=0;    

    ret=sm5414_config_interface(   (unsigned char)(SM5414_CTRL), 
                                    (unsigned char)(val),
                                    (unsigned char)(SM5414_CTRL_ENBOOST_MASK),
                                    (unsigned char)(SM5414_CTRL_ENBOOST_SHIFT)
                                    );
}

void sm5414_set_chgen(unsigned int val)
{
    unsigned int ret=0;    

    ret=sm5414_config_interface(   (unsigned char)(SM5414_CTRL), 
                                    (unsigned char)(val),
                                    (unsigned char)(SM5414_CTRL_CHGEN_MASK),
                                    (unsigned char)(SM5414_CTRL_CHGEN_SHIFT)
                                    );
}

void sm5414_set_suspen(unsigned int val)
{
    unsigned int ret=0;    

    ret=sm5414_config_interface(   (unsigned char)(SM5414_CTRL), 
                                    (unsigned char)(val),
                                    (unsigned char)(SM5414_CTRL_SUSPEN_MASK),
                                    (unsigned char)(SM5414_CTRL_SUSPEN_SHIFT)
                                    );
}

void sm5414_set_reset(unsigned int val)
{
    unsigned int ret=0;    

    ret=sm5414_config_interface(   (unsigned char)(SM5414_CTRL), 
                                    (unsigned char)(val),
                                    (unsigned char)(SM5414_CTRL_RESET_MASK),
                                    (unsigned char)(SM5414_CTRL_RESET_SHIFT)
                                    );
}

void sm5414_set_encomparator(unsigned int val)
{
    unsigned int ret=0;    

    ret=sm5414_config_interface(   (unsigned char)(SM5414_CTRL), 
                                    (unsigned char)(val),
                                    (unsigned char)(SM5414_CTRL_ENCOMPARATOR_MASK),
                                    (unsigned char)(SM5414_CTRL_ENCOMPARATOR_SHIFT)
                                    );
}

//vbusctrl----------------------------------------------------
void sm5414_set_vbuslimit(unsigned int val)
{
    unsigned int ret=0;    

    ret=sm5414_config_interface(   (unsigned char)(SM5414_VBUSCTRL), 
                                    (unsigned char)(val),
                                    (unsigned char)(SM5414_VBUSCTRL_VBUSLIMIT_MASK),
                                    (unsigned char)(SM5414_VBUSCTRL_VBUSLIMIT_SHIFT)
                                    );
}

//chgctrl1----------------------------------------------------
void sm5414_set_prechg(unsigned int val)
{
    unsigned int ret=0;    

    ret=sm5414_config_interface(   (unsigned char)(SM5414_CHGCTRL1), 
                                    (unsigned char)(val),
                                    (unsigned char)(SM5414_CHGCTRL1_PRECHG_MASK),
                                    (unsigned char)(SM5414_CHGCTRL1_PRECHG_SHIFT)
                                    );
}
void sm5414_set_aiclen(unsigned int val)
{
    unsigned int ret=0;    

    ret=sm5414_config_interface(   (unsigned char)(SM5414_CHGCTRL1), 
                                    (unsigned char)(val),
                                    (unsigned char)(SM5414_CHGCTRL1_AICLEN_MASK),
                                    (unsigned char)(SM5414_CHGCTRL1_AICLEN_SHIFT)
                                    );
}
void sm5414_set_autostop(unsigned int val)
{
    unsigned int ret=0;    

    ret=sm5414_config_interface(   (unsigned char)(SM5414_CHGCTRL1), 
                                    (unsigned char)(val),
                                    (unsigned char)(SM5414_CHGCTRL1_AUTOSTOP_MASK),
                                    (unsigned char)(SM5414_CHGCTRL1_AUTOSTOP_SHIFT)
                                    );
}
void sm5414_set_aiclth(unsigned int val)
{
    unsigned int ret=0;    

    ret=sm5414_config_interface(   (unsigned char)(SM5414_CHGCTRL1), 
                                    (unsigned char)(val),
                                    (unsigned char)(SM5414_CHGCTRL1_AICLTH_MASK),
                                    (unsigned char)(SM5414_CHGCTRL1_AICLTH_SHIFT)
                                    );
}

//chgctrl2----------------------------------------------------
void sm5414_set_fastchg(unsigned int val)
{
    unsigned int ret=0;    

    ret=sm5414_config_interface(   (unsigned char)(SM5414_CHGCTRL2), 
                                    (unsigned char)(val),
                                    (unsigned char)(SM5414_CHGCTRL2_FASTCHG_MASK),
                                    (unsigned char)(SM5414_CHGCTRL2_FASTCHG_SHIFT)
                                    );
}

//chgctrl3----------------------------------------------------
void sm5414_set_weakbat(unsigned int val)
{
    unsigned int ret=0;    

    ret=sm5414_config_interface(   (unsigned char)(SM5414_CHGCTRL3), 
                                    (unsigned char)(val),
                                    (unsigned char)(SM5414_CHGCTRL3_WEAKBAT_MASK),
                                    (unsigned char)(SM5414_CHGCTRL3_WEAKBAT_SHIFT)
                                    );
}
void sm5414_set_batreg(unsigned int val)
{
    unsigned int ret=0;    

    ret=sm5414_config_interface(   (unsigned char)(SM5414_CHGCTRL3), 
                                    (unsigned char)(val),
                                    (unsigned char)(SM5414_CHGCTRL3_BATREG_MASK),
                                    (unsigned char)(SM5414_CHGCTRL3_BATREG_SHIFT)
                                    );
}

//chgctrl4----------------------------------------------------
void sm5414_set_dislimit(unsigned int val)
{
    unsigned int ret=0;    

    ret=sm5414_config_interface(   (unsigned char)(SM5414_CHGCTRL4), 
                                    (unsigned char)(val),
                                    (unsigned char)(SM5414_CHGCTRL4_DISLIMIT_MASK),
                                    (unsigned char)(SM5414_CHGCTRL4_DISLIMIT_SHIFT)
                                    );
}
void sm5414_set_topoff(unsigned int val)
{
    unsigned int ret=0;    

    ret=sm5414_config_interface(   (unsigned char)(SM5414_CHGCTRL4), 
                                    (unsigned char)(val),
                                    (unsigned char)(SM5414_CHGCTRL4_TOPOFF_MASK),
                                    (unsigned char)(SM5414_CHGCTRL4_TOPOFF_SHIFT)
                                    );
}

//chgctrl5----------------------------------------------------
void sm5414_set_topofftimer(unsigned int val)
{
    unsigned int ret=0;    

    ret=sm5414_config_interface(   (unsigned char)(SM5414_CHGCTRL5), 
                                    (unsigned char)(val),
                                    (unsigned char)(SM5414_CHGCTRL5_TOPOFFTIMER_MASK),
                                    (unsigned char)(SM5414_CHGCTRL5_TOPOFFTIMER_SHIFT)
                                    );
}
void sm5414_set_fasttimer(unsigned int val)
{
    unsigned int ret=0;    

    ret=sm5414_config_interface(   (unsigned char)(SM5414_CHGCTRL5), 
                                    (unsigned char)(val),
                                    (unsigned char)(SM5414_CHGCTRL5_FASTTIMER_MASK),
                                    (unsigned char)(SM5414_CHGCTRL5_FASTTIMER_SHIFT)
                                    );
}
void sm5414_set_votg(unsigned int val)
{
    unsigned int ret=0;    

    ret=sm5414_config_interface(   (unsigned char)(SM5414_CHGCTRL5), 
                                    (unsigned char)(val),
                                    (unsigned char)(SM5414_CHGCTRL5_VOTG_MASK),
                                    (unsigned char)(SM5414_CHGCTRL5_VOTG_SHIFT)
                                    );
}  

int sm5414_charger_get_otgmode(void)
{
    battery_xlog_printk(BAT_LOG_FULL,"%s current_otg_mode = %d\n",__func__,current_otg_mode);

    return current_otg_mode;
}

void sm5414_cnt_nshdn_pin(unsigned int enable)
{
    if(KAL_TRUE == enable)
    {
    //Siliconmitus : To use i2c without vbus, set nSHDN PIN to 1
#if defined(GPIO_SM5414_SHDN_PIN)
        mt_set_gpio_mode(GPIO_SM5414_SHDN_PIN,GPIO_MODE_GPIO);  
        mt_set_gpio_dir(GPIO_SM5414_SHDN_PIN,GPIO_DIR_OUT);
        mt_set_gpio_out(GPIO_SM5414_SHDN_PIN,GPIO_OUT_ONE);
#else
        printk("[sm5414_cnt_nshdn_pin] Not Support nSHDN PIN\n");
#endif        
    }
    else
    {   
    //Siliconmitus : To disable i2c without vbus, set nSHDN PIN to 0
#if defined(GPIO_SM5414_SHDN_PIN)
        mt_set_gpio_mode(GPIO_SM5414_SHDN_PIN,GPIO_MODE_GPIO);  
        mt_set_gpio_dir(GPIO_SM5414_SHDN_PIN,GPIO_DIR_OUT);        
        mt_set_gpio_out(GPIO_SM5414_SHDN_PIN,GPIO_OUT_ZERO);
#else
        printk("[sm5414_cnt_nshdn_pin] Not Support nSHDN PIN\n");
#endif 
    }
}
extern void charger_en_gpio_output( int level);
void sm5414_otg_enable(unsigned int enable)
{
    if(KAL_TRUE == enable)
    {
        //Before turning on OTG, system must turn off charing function.
        charger_en_gpio_output(1);
        //mt_set_gpio_mode(GPIO_SWCHARGER_EN_PIN,GPIO_SWCHARGER_EN_PIN_M_GPIO);  
        //mt_set_gpio_dir(GPIO_SWCHARGER_EN_PIN,GPIO_DIR_OUT);
        //mt_set_gpio_out(GPIO_SWCHARGER_EN_PIN,GPIO_OUT_ONE);
        sm5414_set_enboost(ENBOOST_EN);
    }
    else
    {   
	 charger_en_gpio_output(0);
        sm5414_set_enboost(ENBOOST_DIS);
    }
    current_otg_mode = enable;
}

int sm5414_boost_notification(int on)
{
	sm5414_otg_enable(on);	
	return 0;
}
/**********************************************************
  *
  *   [Internal Function] 
  *
  *********************************************************/
void sm5414_dump_register(void)
{
    int i=0;
    printk("[sm5414] ");
    for (i=SM5414_INTMASK1;i<SM5414_REG_NUM;i++)
    {
        sm5414_read_byte(i, &sm5414_reg[i]);
        printk("[0x%x]=0x%x ", i, sm5414_reg[i]);        
    }
    printk("\n");
}

#if 0
extern int g_enable_high_vbat_spec;
extern int g_pmic_cid;

void sm5414_hw_init(void)
{    
    if(g_enable_high_vbat_spec == 1)
    {
        if(g_pmic_cid == 0x1020)
        {
            printk("[sm5414_hw_init] (0x06,0x70) because 0x1020\n");
            sm5414_reg_config_interface(0x06,0x70); // set ISAFE
        }
        else
        {
            printk("[sm5414_hw_init] (0x06,0x77)\n");
            sm5414_reg_config_interface(0x06,0x77); // set ISAFE and HW CV point (4.34)
        }
    }
    else
    {
        printk("[sm5414_hw_init] (0x06,0x70) \n");
        sm5414_reg_config_interface(0x06,0x70); // set ISAFE
    }
}
#endif

void sm5414_reg_init(void)
{
   //INT MASK 1/2/3
   //printk("zhangchongyong %s:\n", __func__);
    sm5414_write_byte(SM5414_INTMASK1, 0xFF);
    sm5414_write_byte(SM5414_INTMASK2, 0xFF);
    sm5414_write_byte(SM5414_INTMASK3, 0xFF);

	sm5414_set_encomparator(ENCOMPARATOR_EN);
    sm5414_set_topoff(TOPOFF_150mA);
#if defined(HIGH_BATTERY_VOLTAGE_SUPPORT)
#if defined(LYCONFIG_HIGH_BATTERY_VOLTAGE_44_SUPPORT)
	sm5414_set_batreg(BATREG_4_4_0_0_V); //VREG 4.400V
#else
	sm5414_set_batreg(BATREG_4_3_5_0_V); //VREG 4.352V
#endif	
#else
	sm5414_set_batreg(BATREG_4_2_0_0_V); //VREG 4.208V
#endif 

#if defined(SM5414_TOPOFF_TIMER_SUPPORT)
    sm5414_set_autostop(AUTOSTOP_EN);
    sm5414_set_topofftimer(TOPOFFTIMER_10MIN);
#else
    sm5414_set_autostop(AUTOSTOP_DIS);
#endif
}
#if defined(LYCONFIG_FACTORY_BATTERY_TEST)	
int sm5414_test_flag = 0;

int sm5414_flag_handler(void)
{
  return sm5414_test_flag;
}

module_param(sm5414_test_flag, int, 00664);
#endif

static int sm5414_driver_probe(struct i2c_client *client, const struct i2c_device_id *id) 
{             
    int err=0; 

    //printk("zhangchongyong1 %s:\n", __func__);
    battery_xlog_printk(BAT_LOG_CRTI,"[sm5414_driver_probe] START\n");

    if (!(new_client = kmalloc(sizeof(struct i2c_client), GFP_KERNEL))) {
        err = -ENOMEM;
        goto exit;
    }    
    memset(new_client, 0, sizeof(struct i2c_client));

    new_client = client;    

    sm5414_cnt_nshdn_pin(1); //Set nSHDN pin to high for i2c

    //---------------------
    sm5414_reg_init();
    sm5414_dump_register();
	

    chargin_hw_init_done = KAL_TRUE;
    #if defined(LYCONFIG_FACTORY_BATTERY_TEST)	
    	sm5414_test_flag = 1 ;
    #endif
    battery_xlog_printk(BAT_LOG_CRTI,"[sm5414_driver_probe] DONE\n");
    
    return 0;                                                                                       

exit:
    return err;

}

static int sm5414_driver_remove(struct i2c_client *client)
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
 /*
unsigned char g_reg_value_sm5414=0;
static ssize_t show_sm5414_access(struct device *dev,struct device_attribute *attr, char *buf)
{
    battery_xlog_printk(BAT_LOG_FULL,"[show_sm5414_access] 0x%x\n", g_reg_value_sm5414);
    return sprintf(buf, "%u\n", g_reg_value_sm5414);
}
static ssize_t store_sm5414_access(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    int ret=0;
    char *pvalue = NULL;
    unsigned int reg_value = 0;
    unsigned int reg_address = 0;
    
    battery_xlog_printk(BAT_LOG_FULL,"[store_sm5414_access] \n");
    
    if(buf != NULL && size != 0)
    {
        battery_xlog_printk(BAT_LOG_FULL,"[store_sm5414_access] buf is %s and size is %d \n",buf,(int)size);
        reg_address = simple_strtoul(buf,&pvalue,16);
        
        if(size > 3)
        {        
            reg_value = simple_strtoul((pvalue+1),NULL,16);        
            battery_xlog_printk(BAT_LOG_FULL,"[store_sm5414_access] write sm5414 reg 0x%x with value 0x%x !\n",reg_address,reg_value);
            ret=sm5414_config_interface(reg_address, reg_value, 0xFF, 0x0);
        }
        else
        {    
            ret=sm5414_read_interface(reg_address, &g_reg_value_sm5414, 0xFF, 0x0);
            battery_xlog_printk(BAT_LOG_FULL,"[store_sm5414_access] read sm5414 reg 0x%x with value 0x%x !\n",reg_address,g_reg_value_sm5414);
            battery_xlog_printk(BAT_LOG_FULL,"[store_sm5414_access] Please use \"cat sm5414_access\" to get value\r\n");
        }        
    }    
    return size;
}
static DEVICE_ATTR(sm5414_access, 0664, show_sm5414_access, store_sm5414_access); //664

static int sm5414_user_space_probe(struct platform_device *dev)    
{    
    int ret_device_file = 0;
	
    printk("zhangchongyong1 %s:\n", __func__);
    battery_xlog_printk(BAT_LOG_CRTI,"******** sm5414_user_space_probe!! ********\n" );
    
    ret_device_file = device_create_file(&(dev->dev), &dev_attr_sm5414_access);
    
    return 0;
}

struct platform_device sm5414_user_space_device = {
    .name   = "sm5414-user",
    .id     = -1,
};

static struct platform_driver sm5414_user_space_driver = {
    .probe      = sm5414_user_space_probe,
    .driver     = {
        .name = "sm5414-user",
    },
};
*/

//static struct i2c_board_info /*__initdata*/ i2c_sm5414 = { I2C_BOARD_INFO("sm5414", (SM5414_CHG_SLAVE_ADDR_WRITE>>1))};

static int __init sm5414_init(void)
{    
    int ret=0;
    
    battery_xlog_printk(BAT_LOG_CRTI,"[sm5414_init] init start\n");
    
    //i2c_register_board_info(3, &i2c_sm5414, 1);//Siliconmitus : SM5414_CHG_BUSNUM setting

    if(i2c_add_driver(&sm5414_driver)!=0)
    {
        battery_xlog_printk(BAT_LOG_CRTI,"[sm5414_init] failed to register sm5414 i2c driver.\n");
	 //printk("zhangchongyong1 %s:\n", __func__);
    }
    else
    {
        battery_xlog_printk(BAT_LOG_CRTI,"[sm5414_init] Success to register sm5414 i2c driver.\n");
	 //printk("zhangchongyong2 %s:\n", __func__);
    }
/*
    // sm5414 user space access interface
    ret = platform_device_register(&sm5414_user_space_device);
    if (ret) {
        battery_xlog_printk(BAT_LOG_CRTI,"****[sm5414_init] Unable to device register(%d)\n", ret);
        return ret;
    }    
    ret = platform_driver_register(&sm5414_user_space_driver);
    if (ret) {
        battery_xlog_printk(BAT_LOG_CRTI,"****[sm5414_init] Unable to register driver (%d)\n", ret);
        return ret;
    }
*/    
    //printk("zhangchongyong3 %s:\n", __func__);
    return ret;        
}

static void __exit sm5414_exit(void)
{
    i2c_del_driver(&sm5414_driver);
}

module_init(sm5414_init);
module_exit(sm5414_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("I2C sm5414 Driver");
