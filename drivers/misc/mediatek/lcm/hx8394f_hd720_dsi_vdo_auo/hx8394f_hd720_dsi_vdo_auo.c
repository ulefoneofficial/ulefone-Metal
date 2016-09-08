#ifndef BUILD_LK
#include <linux/string.h>
#endif
#include "lcm_drv.h"
#ifdef BUILD_LK
	#include <platform/mt_gpio.h>
    #include <platform/mt_i2c.h> 
    #include <platform/mt_pmic.h>
#elif defined(BUILD_UBOOT)
	#include <asm/arch/mt_gpio.h>
#else
//	#include <mach/mt_gpio.h>
//	#include <mach/mt_pm_ldo.h>
	#include <mt-plat/upmu_common.h>
//    #include <linux/i2c.h>

#endif
#include "../nt50358_dcdc.h"

#define HX8394_ID (0x8394)
// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------
#define FRAME_WIDTH  (720)
#define FRAME_HEIGHT (1280)

#ifndef TRUE
    #define TRUE 1
#endif
#ifndef FALSE
    #define FALSE 0
#endif

#define REGFLAG_DELAY                                       0xFE
#define REGFLAG_END_OF_TABLE                                0xFF

// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------
static LCM_UTIL_FUNCS lcm_util = {0};

#define SET_RESET_PIN(v)    				(lcm_util.set_reset_pin((v)))

#define UDELAY(n)                           (lcm_util.udelay(n))
#define MDELAY(n)                           (lcm_util.mdelay(n))

// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------
#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)        lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)           lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)                                          lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)                      lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)										lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size)                   lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)
#define   LCM_DSI_CMD_MODE							0

extern unsigned int pmic_config_interface(unsigned int RegNum, unsigned int val, unsigned int MASK, unsigned int SHIFT);

struct LCM_setting_table {
	unsigned cmd;
	unsigned char count;
	unsigned char para_list[64];
};


static struct LCM_setting_table lcm_initialization_setting[] = {
	{0xB9,3,{0xFF,0x83,0x94}}, 
	{0xBA,6,{0x63,0x03,0x68,0x6B,0xB2,0xC0}},
	{0xB1,10,{0x50,0x15,0x75,0x09,0x32,0x44,0x71,0x31,0x55,0x2F}},//VGH FS1=4->1
	{0xD2,1,{0x88}},
	{0xB2,5,{0x00,0x80,0x64,0x10,0x07}}, 
	{0xB4,21,{0x01,0x74,0x01,0x70,0x01,0x74,0x05,0x05,0x86,0x77, 
                 0x00,0x3F,0x01,0x74,0x01,0x74,0x01,0x74,0x01,0x0c,0x86}}, 	

	{0xD3,33,{0x00,0x00,0x07,0x07,
0x40,0x1E,0x08,0x00,0x32,
0x10,0x08,0x00,0x08,0x54,
0x15,0x10,0x05,0x04,0x02,
0x12,0x10,0x05,0x07,0x23,
0x23,0x0C,0x0C,0x27,0x10,
0x07,0x07,0x10,0x40}},

	{0xD5,44,{0x19,0x19,0x18,0x18,
0x1B,0x1B,0x1A,0x1A,0x04,
0x05,0x06,0x07,0x00,0x01,
0x02,0x03,0x20,0x21,0x18,
0x18,0x22,0x23,0x18,0x18,
0x18,0x18,0x18,0x18,0x18,
0x18,0x18,0x18,0x18,0x18,
0x18,0x18,0x18,0x18,0x18,
0x18,0x18,0x18,0x18,0x18}},

	{0xD6,44,{0x18,0x18,0x19,0x19,
0x1B,0x1B,0x1A,0x1A,0x03,
0x02,0x01,0x00,0x07,0x06,
0x05,0x04,0x23,0x22,0x18,
0x18,0x21,0x20,0x18,0x18,
0x18,0x18,0X18,0x18,0x18,
0x18,0x18,0x18,0x18,0x18,
0x18,0x18,0x18,0x18,0x18,
0x18,0x18,0x18,0x18,0x18}},

	{0xCC,1,{0x0B}},
	{0xC0,2,{0x1F,0x31}},//1F  73
	{0xE0,58,{0x00,0x0B,0x17,0x1E,
0x20,0x24,0x28,0x26,0x4E,
0x5D,0x6D,0x6B,0x74,0x84,
0x89,0x8E,0x9A,0x9B,0x96,
0xA4,0xB2,0x58,0x55,0x59,
0x5B,0x5D,0x60,0x64,0x7F,
0x00,0x0B,0x17,0x1D,0x20,
0x24,0x28,0x26,0x4E,0x5D,
0x6D,0x6B,0x74,0x85,0x8A,
0x8E,0x9A,0x9B,0x97,0xA5,
0xB2,0x58,0x55,0x58,0x5B,
0x5D,0x61,0x65,0x7F}},
			
	{REGFLAG_DELAY, 5, {}},
	
	{0xB6,2,{0x83,0x83}},

	{0xD4,1,{0x02}},
	{0xBD,1,{0x01}},
	{0xB1,1,{0x00}},
	{0xBD,1,{0x00}},


	{0xBF,7,{0x40,0x81,0x50,0x00,0x1A,0xFC,0x01}},
	{REGFLAG_DELAY, 10,{}},
 
	{0x11,0, {}},
	{REGFLAG_DELAY, 120, {}},
	{0xB2,12,{0x00,0x80,0x64,0x10,0x07,0x2f,0x00,0x00,0x00,0x00,0xc0,0x18}},
	{0x29,0, {}},
	{REGFLAG_DELAY, 10, {}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
	unsigned int i;
	unsigned cmd;

	for (i = 0; i < count; i++) {

		cmd = table[i].cmd;
		switch (cmd) {

			case REGFLAG_DELAY :
				MDELAY(table[i].count);
				break;

			case REGFLAG_END_OF_TABLE :
				break;

			default:
				dsi_set_cmdq_V2(cmd, table[i].count, table[i].para_list, force_update);
		}
	}
}

#if 0
static struct LCM_setting_table lcm_sleep_out_setting[] = {
	// Sleep Out
	{0x11, 1, {0x00}},
	{REGFLAG_DELAY, 120, {}},
	// Display ON
	{0x29, 1, {0x00}},
	{REGFLAG_DELAY, 100, {}},

	{REGFLAG_END_OF_TABLE, 0x00, {}}
};
#endif

static struct LCM_setting_table lcm_deep_sleep_mode_in_setting[] = {
	// Display off sequence
	{0x28, 1, {0x00}},
	{REGFLAG_DELAY, 100, {}},
	// Sleep Mode On
	{0x10, 1, {0x00}},
	{REGFLAG_DELAY, 120, {}},

	{REGFLAG_END_OF_TABLE, 0x00, {}}
};


// ---------------------------------------------------------------------------
//  LCM Driver Implementations
// ---------------------------------------------------------------------------
static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
	memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}

static void lcm_get_params(LCM_PARAMS *params)
{
	memset(params, 0, sizeof(LCM_PARAMS));

	params->type     = LCM_TYPE_DSI;
	params->width    = FRAME_WIDTH;
	params->height   = FRAME_HEIGHT;

    params->physical_width = 65;
    params->physical_height = 109;

	params->dsi.mode = SYNC_PULSE_VDO_MODE;

	// DSI
	/* Command mode setting */
	params->dsi.LANE_NUM                = LCM_FOUR_LANE;
	//The following defined the fomat for data coming from LCD engine.
	params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;

	// Video mode setting
	params->dsi.PS = LCM_PACKED_PS_24BIT_RGB888;

	params->dsi.vertical_sync_active                = 2;
	params->dsi.vertical_backporch                  = 16;//3;//4
	params->dsi.vertical_frontporch                 = 9;
	params->dsi.vertical_active_line                = FRAME_HEIGHT;//1280+2+16+9=1307

	params->dsi.horizontal_sync_active              = 60;//12;//20;
	params->dsi.horizontal_backporch                = 60;//32;//48; //10 OK
	params->dsi.horizontal_frontporch               = 60;
	params->dsi.horizontal_active_pixel             = FRAME_WIDTH;

	params->dsi.PLL_CLOCK = 220;

	params->dsi.clk_lp_per_line_enable = 0;
	params->dsi.esd_check_enable = 1;
	params->dsi.customization_esd_check_enable = 1;

       params->dsi.lcm_esd_check_table[0].cmd          = 0xd9;
       params->dsi.lcm_esd_check_table[0].count        = 1;
       params->dsi.lcm_esd_check_table[0].para_list[0] = 0x80;

       params->dsi.lcm_esd_check_table[1].cmd          = 0x09;
       params->dsi.lcm_esd_check_table[1].count        = 2;
       params->dsi.lcm_esd_check_table[1].para_list[0] = 0x80;
       params->dsi.lcm_esd_check_table[1].para_list[1] = 0x73;
       //params->dsi.lcm_esd_check_table[1].para_list[2] = 0x04;

       params->dsi.lcm_esd_check_table[2].cmd          = 0x45;
       params->dsi.lcm_esd_check_table[2].count        = 2;
       params->dsi.lcm_esd_check_table[2].para_list[0] = 0x05;
       params->dsi.lcm_esd_check_table[2].para_list[1] = 0x19;
	

}



#if 1
static void lcd_power_en(unsigned char enabled)
{
    if (enabled)
    {
		//pmic_config_interface(MT6328_PMIC_RG_VGP1_VOSEL_ADDR, 0x6, MT6328_PMIC_RG_VGP1_VOSEL_MASK, MT6328_PMIC_RG_VGP1_VOSEL_SHIFT);
		//pmic_config_interface(MT6328_PMIC_RG_VGP1_EN_ADDR, 0x1, MT6328_PMIC_RG_VGP1_EN_MASK, MT6328_PMIC_RG_VGP1_EN_SHIFT);

		//MDELAY(5);
		#ifdef LCM_USE_NT50358_DCDC
		lcm_gpio_output(1,1);
		/*
		if (get_dcdc_type())
		{
			dw8769_set_discharge_status(0x03,0x80);// REG0x03: 0x83:disbale ;0x80: enbale
		}
		else
		{
			nt50358_dcdc_set_bost(0x05,0x30);//bost volatge:5.6V
		}
		//MDELAY(2);
		nt50358_dcdc_set_avdd(0x00,0x0e);//+5.5V
		MDELAY(5);
		lcm_gpio_output(2,1);
		nt50358_dcdc_set_avee(0x01,0x0e);//-5.5V
		MDELAY(5);
		*/
		#endif
	
    }
    else
    {
		//pmic_config_interface(MT6328_PMIC_RG_VGP1_VOSEL_ADDR, 0x0, MT6328_PMIC_RG_VGP1_VOSEL_MASK, MT6328_PMIC_RG_VGP1_VOSEL_SHIFT);
		//pmic_config_interface(MT6328_PMIC_RG_VGP1_EN_ADDR, 0x0, MT6328_PMIC_RG_VGP1_EN_MASK, MT6328_PMIC_RG_VGP1_EN_SHIFT);
		#ifdef LCM_USE_NT50358_DCDC
		lcm_gpio_output(1,0);
		lcm_gpio_output(2,0);
		#endif
    }
}
#endif

	

static void lcm_init(void)
{
	//pmic_config_interface(MT6328_PMIC_RG_VGP1_VOSEL_ADDR, 0x0, MT6328_PMIC_RG_VGP1_VOSEL_MASK, MT6328_PMIC_RG_VGP1_VOSEL_SHIFT);
	//pmic_config_interface(MT6328_PMIC_RG_VGP1_EN_ADDR, 0x0, MT6328_PMIC_RG_VGP1_EN_MASK, MT6328_PMIC_RG_VGP1_EN_SHIFT);
	//lcd_power_en(0);
	/* MDELAY(10000); */
	//MDELAY(10);
	//MDELAY(2000); 
	//lcm_gpio_output(0,0);
	//MDELAY(10);
	pmic_config_interface(MT6328_PMIC_RG_VGP1_VOSEL_ADDR, 0x6, MT6328_PMIC_RG_VGP1_VOSEL_MASK, MT6328_PMIC_RG_VGP1_VOSEL_SHIFT);
	pmic_config_interface(MT6328_PMIC_RG_VGP1_EN_ADDR, 0x1, MT6328_PMIC_RG_VGP1_EN_MASK, MT6328_PMIC_RG_VGP1_EN_SHIFT);
	lcd_power_en(1);
	MDELAY(50);
	lcm_gpio_output(0,0);
	MDELAY(10);
	lcm_gpio_output(0,1);
	MDELAY(50);      
	push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);  
}


static void lcm_suspend(void)
{
	push_table(lcm_deep_sleep_mode_in_setting, sizeof(lcm_deep_sleep_mode_in_setting) / sizeof(struct LCM_setting_table), 1);
	MDELAY(10);
}

static void lcm_resume(void)
{
	//push_table(lcm_sleep_out_setting, sizeof(lcm_sleep_out_setting) / sizeof(struct LCM_setting_table), 1);
	lcm_init();
}

static unsigned int lcm_compare_id(void)
{
	unsigned int id = 0;
	unsigned char buffer[3];

	unsigned int data_array[16];

	SET_RESET_PIN(1);  //NOTE:should reset LCM firstly
	MDELAY(10);

	SET_RESET_PIN(0);
	MDELAY(10);

	SET_RESET_PIN(1);
	MDELAY(10);
	MDELAY(20);

	data_array[0]=0x00043902;
	data_array[1]=0x9483FFB9;
	dsi_set_cmdq(data_array, 2, 1);
	MDELAY(10);

	data_array[0] = 0x00033700;
	dsi_set_cmdq(data_array, 1, 1);

	read_reg_v2(0x04, buffer, 3);
	id = (buffer[0] << 8) | buffer [1]; //we only need ID


#ifdef BUILD_LK
	printf("read id, buf:0x%02x ,0x%02x,0x%02x, id=0X%X", buffer[0], buffer[1], buffer[2], id);
#else
    printk("read id, buf:0x%02x ,0x%02x,0x%02x, id=0X%X", buffer[0], buffer[1], buffer[2], id);
#endif

	return (HX8394_ID == id)?1:0;
}

LCM_DRIVER hx8394f_dsi_vdo_lcm_drv = {
	.name           = "hx8394f_dsi_vdo_auo",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init           = lcm_init,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,	
	.compare_id     = lcm_compare_id,
};
