#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/kobject.h>
#include <linux/platform_device.h>
#include <asm/atomic.h>

#ifndef BUILD_LK
#include <linux/string.h>
#include <linux/kernel.h>
#endif

#include "lcm_drv.h"

#if 0
#include <mach/mt_typedefs.h>
#include <mach/mt_gpio.h>
#include <mach/mt_pm_ldo.h>
#endif
#include "nt50358_dcdc.h"

/*static struct i2c_client *nt50358_i2c_client;*/
struct i2c_client *nt50358_i2c_client;
int Enable_NT50358_DCDC_LOG = 1;
#define nt50358_dcdc_xlog_printk(fmt, args...) \
  do { \
    if (Enable_NT50358_DCDC_LOG) { \
      pr_notice(fmt, ##args); \
    } \
  } while (0)
/**********************************************************
  *
  *   [I2C Slave Setting] 
  *
  *********************************************************/
#define NT50358_DCDC_SLAVE_ADDR            0x3E  
#define NT50358_DCDC_SLAVE_ADDR_WRITE   0x7C
#define NT50358_DCDC_SLAVE_ADDR_READ    0x7D

#define NT50358_DCDC_I2C_BUSNUM 1


/**********************************************************
  *
  *   [Global Variable] 
  *
  *********************************************************/

static DEFINE_MUTEX(nt50358_dcdc_i2c_access);

/**********************************************************
  *
  *   [I2C Function For Read/Write nt50358_dcdc] 
  *
  *********************************************************/
int nt50358_dcdc_read_byte(char cmd, char *returnData)
{
    char     cmd_buf[1]={0x00};
    char     readData = 0;
    int      ret=0;

    mutex_lock(&nt50358_dcdc_i2c_access);
    
    //new_client->addr = ((new_client->addr) & I2C_MASK_FLAG) | I2C_WR_FLAG;    
    nt50358_i2c_client->ext_flag=((nt50358_i2c_client->ext_flag ) & I2C_MASK_FLAG ) | I2C_WR_FLAG | I2C_DIRECTION_FLAG;

    cmd_buf[0] = cmd;
    ret = i2c_master_send(nt50358_i2c_client, &cmd_buf[0], (1<<8 | 1));
    if (ret < 0) 
    {    
        //new_client->addr = new_client->addr & I2C_MASK_FLAG;
        nt50358_i2c_client->ext_flag=0;

        mutex_unlock(&nt50358_dcdc_i2c_access);
        return 0;
    }
    
    readData = cmd_buf[0];
    *returnData = readData;

    // new_client->addr = new_client->addr & I2C_MASK_FLAG;
    nt50358_i2c_client->ext_flag=0;
    
    mutex_unlock(&nt50358_dcdc_i2c_access);    
    return 1;
}

int nt50358_dcdc_write_byte(char cmd, char writeData)
{
    char    write_data[2] = {0};
    int     ret=0;
    
    mutex_lock(&nt50358_dcdc_i2c_access);
    
    write_data[0] = cmd;
    write_data[1] = writeData;
    
    nt50358_i2c_client->ext_flag=((nt50358_i2c_client->ext_flag ) & I2C_MASK_FLAG ) | I2C_DIRECTION_FLAG;
    
    ret = i2c_master_send(nt50358_i2c_client, write_data, 2);
    if (ret < 0) 
    {
       
        nt50358_i2c_client->ext_flag=0;
        mutex_unlock(&nt50358_dcdc_i2c_access);
        return 0;
    }
    
    nt50358_i2c_client->ext_flag=0;
    mutex_unlock(&nt50358_dcdc_i2c_access);
    return 1;
}

int nt50358_dcdc_set_avdd (char addr, char value)
{
	char tmp;

	nt50358_dcdc_read_byte(addr, &tmp);
	nt50358_dcdc_xlog_printk("[nt50358_dcdc_read_byte] Reg[%x]=0x%x\n",addr, tmp);

	tmp &= 0xE0;
	tmp|=value;

	nt50358_dcdc_write_byte(addr, tmp);
	nt50358_dcdc_xlog_printk("[nt50358_dcdc_write_byte] Reg[%x]=0x%x\n",addr, tmp);
       return 0;
}

int nt50358_dcdc_set_avee (char addr, char value)
{
	char tmp;

	nt50358_dcdc_read_byte(addr, &tmp);
	nt50358_dcdc_xlog_printk("[nt50358_dcdc_read_byte] Reg[%x]=0x%x\n",addr, tmp);

	tmp &= 0xE0;
	tmp|=value;

	nt50358_dcdc_write_byte(addr, tmp);
	nt50358_dcdc_xlog_printk("[nt50358_dcdc_write_byte] Reg[%x]=0x%x\n",addr, tmp);
       return 0;
}

int nt50358_dcdc_set_bost (char addr, char value)
{
	char tmp=0;

	nt50358_dcdc_xlog_printk("[nt50358_dcdc_set_bost:\n");
	nt50358_dcdc_read_byte(addr, &tmp);
	nt50358_dcdc_xlog_printk("[nt50358_dcdc_read_byte] Reg[%x]=0x%x\n",addr, tmp);

	tmp &= 0xc0;
	tmp|=value;

	nt50358_dcdc_write_byte(addr, tmp);
	nt50358_dcdc_xlog_printk("[nt50358_dcdc_write_byte] Reg[%x]=0x%x\n",addr, tmp);
	return 0;
}


int dw8769_set_discharge_status (char addr, char value)
{
	char tmp=0;

	nt50358_dcdc_xlog_printk("[dw8769_set_discharge_status:\n");
	nt50358_dcdc_read_byte(addr, &tmp);
	nt50358_dcdc_xlog_printk("[nt50358_dcdc_read_byte] Reg[%x]=0x%x\n",addr, tmp);

	tmp &= 0x80;
	tmp|=value;

	nt50358_dcdc_write_byte(addr, tmp);
	nt50358_dcdc_xlog_printk("[nt50358_dcdc_write_byte] Reg[%x]=0x%x\n",addr, tmp);
	return 0;
}

int get_dcdc_type (void)
{
	char tmp=0;

	char dcdc_type=0;

	//NT50358 0x03=0x03
	//DW8769 0x03=0x83
	nt50358_dcdc_read_byte(0x03, &tmp);

	nt50358_dcdc_xlog_printk("[otm1282,dcdc_type,REG0x03=%x\n",tmp);
	if ((tmp&0x80)!=0)
		dcdc_type=1;//DW8769
	else 
		dcdc_type=0;//NT50358

	nt50358_dcdc_xlog_printk("[otm1282,dcdc_type(0:NT50358,1:DW8769)=%d\n",dcdc_type);
	return dcdc_type;
}

#define TPS_I2C_BUSNUM  NT50358_DCDC_I2C_BUSNUM	/* for I2C channel 0 */
#define I2C_ID_NAME "nt50358"
#define TPS_ADDR 0x3E

#if defined(CONFIG_MTK_LEGACY)
static struct i2c_board_info nt50358_board_info __initdata = { I2C_BOARD_INFO(I2C_ID_NAME, TPS_ADDR) };
#endif
#if !defined(CONFIG_MTK_LEGACY)
static const struct of_device_id lcm_of_match[] = {
		{.compatible = "mediatek,I2C_LCD_BIAS"},
		{},
};
#endif



/*****************************************************************************
 * Function Prototype
 *****************************************************************************/
static int nt50358_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int nt50358_remove(struct i2c_client *client);
/*****************************************************************************
 * Data Structure
 *****************************************************************************/

struct nt50358_dev {
	struct i2c_client *client;

};

static const struct i2c_device_id nt50358_id[] = {
	{I2C_ID_NAME, 0},
	{}
};

/* #if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,36)) */
/* static struct i2c_client_address_data addr_data = { .forces = forces,}; */
/* #endif */
static struct i2c_driver nt50358_dcdc_i2c_driver = {
	.id_table = nt50358_id,
	.probe = nt50358_probe,
	.remove = nt50358_remove,
	/* .detect               = mt6605_detect, */
	.driver = {
		   .owner = THIS_MODULE,
		   .name = "nt50358",
#if !defined(CONFIG_MTK_LEGACY)
			.of_match_table = lcm_of_match,
#endif
		   },
};

static int nt50358_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	nt50358_dcdc_xlog_printk("nt50358_iic_probe\n");
	nt50358_dcdc_xlog_printk("TPS: info==>name=%s addr=0x%x\n", client->name, client->addr);
	nt50358_i2c_client = client;
	return 0;
}

static int nt50358_remove(struct i2c_client *client)
{
	nt50358_dcdc_xlog_printk("nt50358_remove\n");
	nt50358_i2c_client = NULL;
	i2c_unregister_device(client);
	return 0;
}

void flashlight_flash_en_gpio_output( int level);
void flashlight_mode_select_gpio_output( int level);
void ext_spkamp_en_gpio_output( int level );
void msensor_rst_output( int level);
struct pinctrl *lcm_pinctrllcm;
struct pinctrl_state *lcm_pins_default;
struct pinctrl_state *lcm_rst_output0, *lcm_rst_output1,  *lcm_enp_output0, *lcm_enp_output1, *lcm_enn_output0, *lcm_enn_output1, *chg_en_output0, *chg_en_output1,*smartpa_reset_output0,*smartpa_reset_output1;
struct pinctrl_state *flashlight_mode_select_output0,*flashlight_mode_select_output1,*flashlight_flash_en_output0,*flashlight_flash_en_output1;
struct pinctrl_state *msensor_rst_out0, *msensor_rst_out1;
struct pinctrl_state  *modem_bpi_bus_19_cfg,*modem_bpi_bus_20_cfg,*modem_bpi_bus_21_cfg, *modem_bpi_bus_18_cfg;
struct pinctrl_state *ext_spkamp_en_output0,*ext_spkamp_en_output1;
struct pinctrl_state *fan5405_chg_en_output0, *fan5405_chg_en_output1;
static int lcm_pinctrl_probe(struct platform_device *pdev)
{
	int retval = 0;

	lcm_pinctrllcm = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(lcm_pinctrllcm)) {
		nt50358_dcdc_xlog_printk("Cannot find usb pinctrl!\n");
	} else {
		nt50358_dcdc_xlog_printk("find usb pinctrllcm ok!\n");
	}
	/********************************************************/
	lcm_pins_default = pinctrl_lookup_state(lcm_pinctrllcm, "default");
	if (IS_ERR(lcm_pins_default))
		nt50358_dcdc_xlog_printk( "Can *NOT* find default\n");
	else
		nt50358_dcdc_xlog_printk("Find default\n");
	lcm_rst_output0 = pinctrl_lookup_state(lcm_pinctrllcm, "state_lcm_rst_output0");
	if (IS_ERR(lcm_rst_output0))
		nt50358_dcdc_xlog_printk( "Can *NOT* find state_lcm_rst_en\n");
	else
		nt50358_dcdc_xlog_printk("Find state_lcm_rst_en\n");
	lcm_rst_output1 = pinctrl_lookup_state(lcm_pinctrllcm, "state_lcm_rst_output1");
	if (IS_ERR(lcm_rst_output1))
		nt50358_dcdc_xlog_printk( "Can *NOT* find state_lcm_rst_en\n");
	else
		nt50358_dcdc_xlog_printk("Find state_lcm_rst_en\n");
	
	lcm_enp_output0 = pinctrl_lookup_state(lcm_pinctrllcm, "state_lcm_enp_output0");
	if (IS_ERR(lcm_enp_output0))
		nt50358_dcdc_xlog_printk( "Can *NOT* find lcm_enp_en\n");
	else
		nt50358_dcdc_xlog_printk("Find lcm_enp_en\n");
	lcm_enp_output1 = pinctrl_lookup_state(lcm_pinctrllcm, "state_lcm_enp_output1");
	if (IS_ERR(lcm_enp_output1))
		nt50358_dcdc_xlog_printk( "Can *NOT* find lcm_enp_en\n");
	else
		nt50358_dcdc_xlog_printk("Find lcm_enp_en\n");
	
	lcm_enn_output0 = pinctrl_lookup_state(lcm_pinctrllcm, "state_lcm_enn_output0");
	if (IS_ERR(lcm_enn_output0))
		nt50358_dcdc_xlog_printk( "Can *NOT* find lcm_enp_en\n");
	else
		nt50358_dcdc_xlog_printk("Find lcm_enp_en\n");
	lcm_enn_output1 = pinctrl_lookup_state(lcm_pinctrllcm, "state_lcm_enn_output1");
	if (IS_ERR(lcm_enn_output1))
		nt50358_dcdc_xlog_printk( "Can *NOT* find lcm_enp_en\n");
	else
		nt50358_dcdc_xlog_printk("Find lcm_enp_en\n");
	chg_en_output0 = pinctrl_lookup_state(lcm_pinctrllcm, "state_charger_en_output0");
	if (IS_ERR(chg_en_output0))
		nt50358_dcdc_xlog_printk( "Can *NOT* find exe_charger_en\n");
	else
		nt50358_dcdc_xlog_printk("Find exe_charger_en\n");
	chg_en_output1 = pinctrl_lookup_state(lcm_pinctrllcm, "state_charger_en_output1");
	if (IS_ERR(chg_en_output1))
		nt50358_dcdc_xlog_printk( "Can *NOT* find exe_charger_en\n");
	else
		nt50358_dcdc_xlog_printk("Find exe_charger_en\n");

	smartpa_reset_output0 = pinctrl_lookup_state(lcm_pinctrllcm, "state_smartpa_reset_output0");
	if (IS_ERR(smartpa_reset_output0))
		nt50358_dcdc_xlog_printk( "Can *NOT* find smartpa_reset_output0\n");
	else
		nt50358_dcdc_xlog_printk("Find smartpa_reset_output0\n");
	smartpa_reset_output1 = pinctrl_lookup_state(lcm_pinctrllcm, "state_smartpa_reset_output1");
	if (IS_ERR(smartpa_reset_output1))
		nt50358_dcdc_xlog_printk( "Can *NOT* find smartpa_reset_output1\n");
	else
		nt50358_dcdc_xlog_printk("Find smartpa_reset_output1\n");
	
	#if 1//added by xiehaifei for flashlight
	flashlight_mode_select_output0 = pinctrl_lookup_state(lcm_pinctrllcm, "state_flashlight_mode_select_output0");
	if (IS_ERR(flashlight_mode_select_output0))
		nt50358_dcdc_xlog_printk( "Can *NOT* find flashlight_mode_select_output0!\n");
	else
	{
		nt50358_dcdc_xlog_printk("Find flashlight_mode_select_output0!\n");
		flashlight_mode_select_gpio_output(0);
	}
	flashlight_mode_select_output1 = pinctrl_lookup_state(lcm_pinctrllcm, "state_flashlight_mode_select_output1");
	if (IS_ERR(flashlight_mode_select_output1))
		nt50358_dcdc_xlog_printk( "Can *NOT* find flashlight_mode_select_output1!\n");
	else
		nt50358_dcdc_xlog_printk("Find flashlight_mode_select_output1!\n");

	flashlight_flash_en_output0 = pinctrl_lookup_state(lcm_pinctrllcm, "state_flashlight_flash_en_output0");
	if (IS_ERR(flashlight_flash_en_output0))
		nt50358_dcdc_xlog_printk( "Can *NOT* find flashlight_flash_en_output0!\n");
	else
	{
		nt50358_dcdc_xlog_printk("Find flashlight_flash_en_output0!\n");
		flashlight_flash_en_gpio_output(0);
	}
	flashlight_flash_en_output1 = pinctrl_lookup_state(lcm_pinctrllcm, "state_flashlight_flash_en_output1");
	if (IS_ERR(flashlight_flash_en_output1))
		nt50358_dcdc_xlog_printk( "Can *NOT* find flashlight_flash_en_output1!\n");
	else
		nt50358_dcdc_xlog_printk("Find flashlight_flash_en_output1!\n");
	#endif//ended by xiehaifei  for flashlight
	
	msensor_rst_out0 = pinctrl_lookup_state(lcm_pinctrllcm, "state_msensor_rst_output0");
	if (IS_ERR(msensor_rst_out0))
		nt50358_dcdc_xlog_printk( "Can *NOT* find msensor_rst_out0\n");
	else
		nt50358_dcdc_xlog_printk("Find msensor_rst_out0\n");
	msensor_rst_out1 = pinctrl_lookup_state(lcm_pinctrllcm, "state_msensor_rst_output1");
	if (IS_ERR(msensor_rst_out1))
		nt50358_dcdc_xlog_printk( "Can *NOT* find msensor_rst_out1\n");
	else
	{
		nt50358_dcdc_xlog_printk("Find msensor_rst_out1\n");
		msensor_rst_output(1);
	}	

	modem_bpi_bus_18_cfg= pinctrl_lookup_state(lcm_pinctrllcm, "state_modem_bpi_bus_18_cfg");
	if (IS_ERR(modem_bpi_bus_18_cfg))
		printk( "Can *NOT* find modem_bpi_bus_18_cfg!\n");
	else
	{
		pinctrl_select_state(lcm_pinctrllcm, modem_bpi_bus_18_cfg);
		printk("Find modem_bpi_bus_18_cfg!\n");
	}	
	
	modem_bpi_bus_19_cfg= pinctrl_lookup_state(lcm_pinctrllcm, "state_modem_bpi_bus_19_cfg");
	if (IS_ERR(modem_bpi_bus_19_cfg))
		printk( "Can *NOT* find modem_bpi_bus_19_cfg!\n");
	else
	{
		pinctrl_select_state(lcm_pinctrllcm, modem_bpi_bus_19_cfg);
		printk("Find modem_bpi_bus_19_cfg!\n");
	}	

	modem_bpi_bus_20_cfg= pinctrl_lookup_state(lcm_pinctrllcm, "state_modem_bpi_bus_20_cfg");
	if (IS_ERR(modem_bpi_bus_20_cfg))
		printk( "Can *NOT* find modem_bpi_bus_20_cfg!\n");
	else
	{
		pinctrl_select_state(lcm_pinctrllcm, modem_bpi_bus_20_cfg);
		printk("Find modem_bpi_bus_20_cfg!\n");
	}

	modem_bpi_bus_21_cfg= pinctrl_lookup_state(lcm_pinctrllcm, "state_modem_bpi_bus_21_cfg");
	if (IS_ERR(modem_bpi_bus_21_cfg))
		printk( "Can *NOT* find modem_bpi_bus_21_cfg!\n");
	else
	{
		pinctrl_select_state(lcm_pinctrllcm, modem_bpi_bus_21_cfg);
		printk("Find modem_bpi_bus_21_cfg!\n");
	}

	#if 1//defined(CLASS_K)　||　defined(LYCONFIG_COMB_SPEAKER_SUPPORT_CLASS_K)
	ext_spkamp_en_output0 = pinctrl_lookup_state(lcm_pinctrllcm, "state_ext_spkamp_en_output0");
	if (IS_ERR(ext_spkamp_en_output0))
		nt50358_dcdc_xlog_printk( "Can *NOT* find ext_spkamp_en_output0!\n");
	else
	{
		nt50358_dcdc_xlog_printk("Find ext_spkamp_en_output0!\n");
		ext_spkamp_en_gpio_output(0);
	}
	ext_spkamp_en_output1 = pinctrl_lookup_state(lcm_pinctrllcm, "state_ext_spkamp_en_output1");
	if (IS_ERR(ext_spkamp_en_output1))
		nt50358_dcdc_xlog_printk( "Can *NOT* find ext_spkamp_en_output1!\n");
	else
		nt50358_dcdc_xlog_printk("Find ext_spkamp_en_output1!\n");
	#endif

//#if defined(LYCONFIG_AUTO_PLATFORM_NAME_F5C)
	fan5405_chg_en_output0 = pinctrl_lookup_state(lcm_pinctrllcm, "state_fan5405_charger_en_output0");
	if (IS_ERR(fan5405_chg_en_output0))
		nt50358_dcdc_xlog_printk( "Can *NOT* find state_fan5405_charger_en_output0\n");
	else
		nt50358_dcdc_xlog_printk("Find state_fan5405_charger_en_output0\n");
	fan5405_chg_en_output1 = pinctrl_lookup_state(lcm_pinctrllcm, "state_fan5405_charger_en_output1");
	if (IS_ERR(fan5405_chg_en_output1))
		nt50358_dcdc_xlog_printk( "Can *NOT* find state_fan5405_charger_en_output1\n");
	else
		nt50358_dcdc_xlog_printk("Find state_fan5405_charger_en_output1\n");
//#endif

	return retval;
}

void lcm_gpio_output(int pin, int level)
{
	nt50358_dcdc_xlog_printk("[lcm]lcm_gpio_output pin = %d, level = %d\n", pin, level);
	if (pin == 0) {
		if (level)
			pinctrl_select_state(lcm_pinctrllcm, lcm_rst_output1);
		else
			pinctrl_select_state(lcm_pinctrllcm, lcm_rst_output0);
	} else if (pin == 1) {
		if (level)
			pinctrl_select_state(lcm_pinctrllcm, lcm_enp_output1);
		else
			pinctrl_select_state(lcm_pinctrllcm, lcm_enp_output0);
	} else {
	       if (level)
			pinctrl_select_state(lcm_pinctrllcm, lcm_enn_output1);
		else
			pinctrl_select_state(lcm_pinctrllcm, lcm_enn_output0);
	}
}

void charger_en_gpio_output( int level)
{
	nt50358_dcdc_xlog_printk("[lcm]charger_en_gpio_output pin , level = %d\n",  level);
#if defined(LYCONFIG_AUTO_PLATFORM_NAME_F5C)&& defined(LYCONFIG_COMB_CHARGER_IC_MTK_FAN5405_SUPPORT) 	
  if (level)
		pinctrl_select_state(lcm_pinctrllcm, fan5405_chg_en_output1);
	else
		pinctrl_select_state(lcm_pinctrllcm, fan5405_chg_en_output0);
#else
       if (level)
		pinctrl_select_state(lcm_pinctrllcm, chg_en_output1);
	else
		pinctrl_select_state(lcm_pinctrllcm, chg_en_output0);
#endif	
}

void smartpa_reset_gpio_output( int level )
{
	nt50358_dcdc_xlog_printk("[lcm]smartpa_reset_gpio_output pin , level = %d\n",  level);
	if (level)
		pinctrl_select_state(lcm_pinctrllcm, smartpa_reset_output1);
	else
		pinctrl_select_state(lcm_pinctrllcm, smartpa_reset_output0);
}

void flashlight_mode_select_gpio_output( int level)
{
	nt50358_dcdc_xlog_printk("[lcm]flashlight_mode_select_gpio_output pin , level = %d\n",  level);
       if (level)
		pinctrl_select_state(lcm_pinctrllcm, flashlight_mode_select_output1);
	else
		pinctrl_select_state(lcm_pinctrllcm, flashlight_mode_select_output0);
}

void flashlight_flash_en_gpio_output( int level)
{
	nt50358_dcdc_xlog_printk("[lcm]flashlight_flash_en_gpio_output pin , level = %d\n",  level);
       if (level)
		pinctrl_select_state(lcm_pinctrllcm, flashlight_flash_en_output1);
	else
		pinctrl_select_state(lcm_pinctrllcm, flashlight_flash_en_output0);
}

void msensor_rst_output( int level)
{
	nt50358_dcdc_xlog_printk("[lcm]msensor_rst_output pin , level = %d\n",  level);
       if (level)
		pinctrl_select_state(lcm_pinctrllcm, msensor_rst_out1);
	else
		pinctrl_select_state(lcm_pinctrllcm, msensor_rst_out0);
}


void ext_spkamp_en_gpio_output( int level )
{
#if 1//defined(LYCONFIG_COMB_SPEAKER_SUPPORT_CLASS_K)
	nt50358_dcdc_xlog_printk("[lcm]ext_spkamp_en_gpio_output pin , level = %d\n",  level);
	if (level)
		pinctrl_select_state(lcm_pinctrllcm, ext_spkamp_en_output1);
	else
		pinctrl_select_state(lcm_pinctrllcm, ext_spkamp_en_output0);
#endif	
}



static int lcm_pinctrl_remove(struct platform_device *pdev)
{
	return 0;
}

static const struct of_device_id lcm_pinctrl_ids[] = {
	{.compatible = "mediatek,lcm_pinctrl",},
	{},
};

static struct platform_driver lcm_pinctrl_driver = {
	.probe = lcm_pinctrl_probe,
	.remove = lcm_pinctrl_remove,
	.driver = {
		.name = "lcm_pinctrl",
#ifdef CONFIG_OF
		.of_match_table = lcm_pinctrl_ids,
#endif
	},
};


static int __init nt50358_dcdc_init(void)
{    
    int ret=0;
    
   printk("[nt50358_dcdc_init] init start\n");

    if(i2c_add_driver(&nt50358_dcdc_i2c_driver)!=0)
    {
        nt50358_dcdc_xlog_printk("[nt50358_dcdc_init] failed to register nt50358_dcdc i2c driver.\n");
	 ret = -1;
    }
    else
    {
        nt50358_dcdc_xlog_printk("[nt50358_dcdc_init] Success to register nt50358_dcdc i2c driver.\n");
    }
    if (!platform_driver_register(&lcm_pinctrl_driver))
			nt50358_dcdc_xlog_printk("register lcm pinctrl succeed!!\n");
    else {
			nt50358_dcdc_xlog_printk("register lcm pinctrl fail!!\n");
			ret = -1;
    }
    return ret;        
}

static void __exit nt50358_dcdc_exit(void)
{
    i2c_del_driver(&nt50358_dcdc_i2c_driver);
}

module_init(nt50358_dcdc_init);
module_exit(nt50358_dcdc_exit);
