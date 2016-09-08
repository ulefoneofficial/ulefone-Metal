#ifndef BUILD_LK
#include <linux/string.h>
#endif
#include "lcm_drv.h"

#ifdef BUILD_LK
	#include <platform/mt_gpio.h>
#include <platform/mt_pmic.h>
#elif defined(BUILD_UBOOT)
	#include <asm/arch/mt_gpio.h>
#else
//	#include <mach/mt_gpio.h>
//	#include <mach/mt_pm_ldo.h>
//	#include <mach/upmu_common.h>

#endif
#include "../nt50358_dcdc.h"
// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------

#define FRAME_WIDTH  										(720)
#define FRAME_HEIGHT 										(1280)

#define REGFLAG_DELAY             							0XFEFF
#define REGFLAG_END_OF_TABLE      							0xFFFF   // END OF REGISTERS MARKER

#define LCM_DSI_CMD_MODE									0

#define LCM_ID_OTM1283 										0x40

// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

static LCM_UTIL_FUNCS lcm_util = {0};
static int esd_test_new = 0;

#define GPIO_LCD_RST_EN      								(GPIO146 | 0x80000000)
#define SET_RESET_PIN(v)    								(lcm_util.set_reset_pin((v)))

#define UDELAY(n) 											(lcm_util.udelay(n))
#define MDELAY(n) 											(lcm_util.mdelay(n))


// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)										lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)					lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)											lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size)   				lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)   

struct LCM_setting_table {
    unsigned cmd;
    unsigned char count;
    unsigned char para_list[64];
};


static struct LCM_setting_table lcm_initialization_setting[] = {
	{0x00,1,{0x00}}, 
	{0xff,3,{0x12,0x83,0x01}}, 
	 
	{0x00,1,{0x80}}, 
	{0xff,2,{0x12,0x83}}, 
	 
	{0x00,1,{0x92}}, 
	{0xff,2,{0x30,0x02}}, ///3lane :0x20 4lane: 0x30
	 
	{0x00,1,{0xc6}}, 
	{0xb0,1,{0x03}}, 
	 
	{0x00,1,{0xb9}}, 
	{0xb0,1,{0x51}}, 
	 
	{0x00,1,{0x80}}, 
	{0xc0,9,{0x00,0x64,0x00,0x0f,0x11,0x00,0x64,0x0f,0x11}}, 
	 
	{0x00,1,{0x90}}, 
	{0xc0,6,{0x00,0x55,0x00,0x01,0x00,0x04}}, 
	 
	{0x00,1,{0xa4}}, 
	{0xc0,1,{0x00}}, 
	 
	{0x00,1,{0xb3}}, 
	{0xc0,3,{0x00,0x50,0x48}},//50 
	 
	{0x00,1,{0x81}}, 
	{0xc1,1,{0x55}}, 
	 
	//{0x00,1,{0xa0}},//**esd 
	//{0xc1,1,{0x80}}, 
	 
	//{0x00,1,{0x90}},//**BEST MODE 
	//{0xf6,1,{0x16}}, 
	 
	{0x00,1,{0x80}}, 
	{0xc4,1,{0x30}}, 
	 
	{0x00,1,{0x81}}, 
	{0xc4,1,{0x82}}, 
	 
	{0x00,1,{0x82}}, 
	{0xc4,1,{0x02}}, 
	 
	{0x00,1,{0x87}}, 
	{0xc4,1,{0x18}}, 
	 
	{0x00,1,{0x8a}}, 
	{0xc4,1,{0x40}}, 
	 
	{0x00,1,{0x8b}}, 
	{0xc4,1,{0x00}}, 
	 
	{0x00,1,{0x90}}, 
	{0xc4,1,{0x49}}, 
	 
	{0x00,1,{0xa0}}, 
	{0xc4,14,{0x05,0x10,0x06,0x02,0x05,0x15,0x10,0x05,0x10,0x07,0x02,0x05,0x15,0x10}}, 
	 
	{0x00,1,{0xb0}}, 
	{0xc4,2,{0x00,0x00}}, 
	 
	{0x00,1,{0x90}}, 
	{0xc5,1,{0x50}}, 
	 
	{0x00,1,{0x91}}, 
	{0xc5,2,{0x19,0x50}}, 
	 
	{0x00,1,{0x94}}, 
	{0xc5,1,{0x66}}, 
	 
	{0x00,1,{0xb0}}, 
	{0xc5,2,{0x04,0x38}}, 
	 
	{0x00,1,{0xb4}}, 
	{0xc5,1,{0xc0}}, 
	 
	{0x00,1,{0xbb}}, 
	{0xc5,1,{0x80}}, 
	 
	{0x00,1,{0xb5}}, 
	{0xc5,6,{0x3b,0xed,0x30,0x3b,0xed,0x30}}, 
	 
	{0x00,1,{0xb1}}, 
	{0xc6,1,{0x05}}, 
	 
	{0x00,1,{0xa0}}, 
	{0xc1,1,{0x02}}, 
	 
	{0x00,1,{0x90}}, 
	{0xf5,4,{0x02,0x11,0x02,0x11}}, 
	 
	{0x00,1,{0xb2}}, 
	{0xf5,2,{0x00,0x00}}, 
	 
	{0x00,1,{0xb4}}, 
	{0xf5,2,{0x00,0x00}}, 
	 
	{0x00,1,{0xb6}}, 
	{0xf5,2,{0x00,0x00}}, 
	 
	{0x00,1,{0xb8}}, 
	{0xf5,2,{0x00,0x00}}, 
	 
	{0x00,1,{0x94}}, 
	{0xf5,1,{0x02}}, 
	 
	{0x00,1,{0xba}}, 
	{0xf5,1,{0x03}}, 
	 
	{0x00,1,{0xc3}}, 
	{0xf5,1,{0x81}}, 
	 
	{0x00,1,{0xb1}}, 
	{0xc6,1,{0x05}}, 
	 
	{0x00,1,{0x00}}, 
	{0xd0,1,{0x40}}, 
	 
	{0x00,1,{0x00}}, 
	{0xd1,2,{0x00,0x00}}, 
	 
	{0x00,1,{0x80}}, 
	{0xcb,11,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}}, 
	 
	{0x00,1,{0x90}}, 
	{0xcb,15,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xff,0x00,0xff,0x00}}, 
	 
	{0x00,1,{0xa0}}, 
	{0xcb,15,{0xff,0x00,0xff,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}}, 
	 
	{0x00,1,{0xb0}}, 
	{0xcb,15,{0x00,0x00,0x00,0xff,0x00,0xff,0x00,0xff,0x00,0xff,0x00,0x00,0x00,0x00,0x00}}, 
	 
	{0x00,1,{0xc0}}, 
	{0xcb,15,{0x00,0x00,0x00,0x00,0x05,0x05,0x00,0x00,0x05,0x00,0x05,0x05,0x00,0x05,0x05}}, 
	 
	{0x00,1,{0xd0}}, 
	{0xcb,15,{0x05,0x05,0x05,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x05,0x05,0x00,0x00}}, 
	 
	{0x00,1,{0xe0}}, 
	{0xcb,14,{0x05,0x00,0x05,0x05,0x00,0x05,0x05,0x05,0x05,0x05,0x00,0x00,0x00,0x00}}, 
	 
	{0x00,1,{0xf0}}, 
	{0xcb,11,{0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff}}, 
	 
	{0x00,1,{0x80}}, 
	{0xcc,15,{0x00,0x00,0x00,0x00,0x2e,0x2d,0x00,0x00,0x29,0x00,0x2a,0x0c,0x00,0x0a,0x10}}, 
	 
	{0x00,1,{0x90}}, 
	{0xcc,15,{0x0e,0x02,0x04,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x2e,0x2d,0x00,0x00}}, 
	 
	{0x00,1,{0xa0}}, 
	{0xcc,14,{0x29,0x00,0x2a,0x0b,0x00,0x09,0x0f,0x0d,0x01,0x03,0x00,0x00,0x00,0x00}}, 
	 
	{0x00,1,{0xb0}}, 
	{0xcc,15,{0x00,0x00,0x00,0x00,0x2d,0x2e,0x00,0x00,0x29,0x00,0x2a,0x0d,0x00,0x0f,0x09}}, 
	 
	{0x00,1,{0xc0}}, 
	{0xcc,15,{0x0b,0x03,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x2d,0x2e,0x00,0x00}}, 
	 
	{0x00,1,{0xd0}}, 
	{0xcc,14,{0x29,0x00,0x2a,0x0e,0x00,0x10,0x0a,0x0c,0x04,0x02,0x00,0x00,0x00,0x00}}, 
	 
	{0x00,1,{0x80}}, 
	{0xce,12,{0x8b,0x03,0x10,0x8a,0x03,0x10,0x89,0x03,0x10,0x88,0x03,0x10}}, 
	 
	{0x00,1,{0x90}}, 
	{0xce,14,{0xf0,0x00,0x00,0xf0,0x00,0x00,0xf0,0x00,0x00,0xf0,0x00,0x00,0x00,0x00}}, 
	 
	{0x00,1,{0xa0}}, 
	{0xce,14,{0x38,0x07,0x84,0xfe,0x00,0x10,0x00,0x38,0x06,0x84,0xff,0x00,0x10,0x00}}, 
	 
	{0x00,1,{0xb0}}, 
	{0xce,14,{0x38,0x05,0x85,0x00,0x00,0x10,0x00,0x38,0x04,0x85,0x01,0x00,0x10,0x00}}, 
	 
	{0x00,1,{0xc0}}, 
	{0xce,14,{0x38,0x03,0x85,0x02,0x00,0x10,0x00,0x38,0x02,0x85,0x03,0x00,0x10,0x00}}, 
	 
	{0x00,1,{0xd0}}, 
	{0xce,14,{0x38,0x01,0x85,0x04,0x00,0x10,0x00,0x38,0x00,0x85,0x05,0x00,0x10,0x00}}, 
	 
	{0x00,1,{0x80}}, 
	{0xcf,14,{0x70,0x00,0x00,0x10,0x00,0x00,0x00,0x70,0x00,0x00,0x10,0x00,0x00,0x00}}, 
	 
	{0x00,1,{0x90}}, 
	{0xcf,14,{0x70,0x00,0x00,0x10,0x00,0x00,0x00,0x70,0x00,0x00,0x10,0x00,0x00,0x00}}, 
	 
	{0x00,1,{0xa0}}, 
	{0xcf,14,{0x70,0x00,0x00,0x10,0x00,0x00,0x00,0x70,0x00,0x00,0x10,0x00,0x00,0x00}}, 
	 
	{0x00,1,{0xb0}}, 
	{0xcf,14,{0x70,0x00,0x00,0x10,0x00,0x00,0x00,0x70,0x00,0x00,0x10,0x00,0x00,0x00}}, 
	 
	{0x00,1,{0xc0}}, 
	{0xcf,11,{0x04,0x01,0x20,0x20,0x00,0x00,0x01,0x80,0x00,0x03,0x08}}, 
	 
	{0x00,1,{0x00}}, 
	{0xd8,2,{0xbc,0xbc}}, 
	 
	{0x00,1,{0x00}}, 
	{0xd9,1,{0xaf}},//b6 b0  a9 
	 
	{0x00,1,{0x00}}, 
	{0xe1,16,{0x0b,0x0D,0x12,0x0D,0x06,0x0C,0x0A,0x09,0x04,0x07,0x10,0x08,0x10,0x11,0x0C,0x0d}}, 
	 
	{0x00,1,{0x00}}, 
	{0xe2,16,{0x01,0x0D,0x11,0x0D,0x06,0x0C,0x0A,0x09,0x05,0x08,0x10,0x08,0x0F,0x10,0x0C,0x03}}, 
	 
	{0x00,1,{0x00}}, 
	{0xff,3,{0xff,0xff,0xff}}, 
	 
	{0x11,0,{}},
	    
	{REGFLAG_DELAY,150,{}},
	    
	{0x29,0,{}},
	    
	{REGFLAG_DELAY,20,{}},
};

#if 0
static struct LCM_setting_table lcm_set_window[] = {
	{0x2A,	4,	{0x00, 0x00, (FRAME_WIDTH>>8), (FRAME_WIDTH&0xFF)}},
	{0x2B,	4,	{0x00, 0x00, (FRAME_HEIGHT>>8), (FRAME_HEIGHT&0xFF)}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};


static struct LCM_setting_table lcm_sleep_out_setting[] = {
    // Sleep Out
	{0x11, 1, {0x00}},
    {REGFLAG_DELAY, 100, {}},

    // Display ON
	{0x29, 1, {0x00}},
	{REGFLAG_DELAY, 10, {}},
	
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

#endif
static struct LCM_setting_table lcm_sleep_in_setting[] = {
	// Display off sequence
	{0x28, 1, {0x00}},

    // Sleep Mode On
	{0x10, 1, {0x00}},

	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static void lcd_power_en(unsigned char enabled)
{
    if (enabled)
    {
	#ifdef BUILD_LK
	#else
		//pmic_config_interface(MT6328_PMIC_RG_VGP1_VOSEL_ADDR, 0x6, MT6328_PMIC_RG_VGP1_VOSEL_MASK, MT6328_PMIC_RG_VGP1_VOSEL_SHIFT);
		//pmic_config_interface(MT6328_PMIC_RG_VGP1_EN_ADDR, 0x1, MT6328_PMIC_RG_VGP1_EN_MASK, MT6328_PMIC_RG_VGP1_EN_SHIFT);
	#endif
    }
    else
    {
	#ifdef BUILD_LK
	#else
		//pmic_config_interface(MT6328_PMIC_RG_VGP1_VOSEL_ADDR, 0x0, MT6328_PMIC_RG_VGP1_VOSEL_MASK, MT6328_PMIC_RG_VGP1_VOSEL_SHIFT);
		//pmic_config_interface(MT6328_PMIC_RG_VGP1_EN_ADDR, 0x0, MT6328_PMIC_RG_VGP1_EN_MASK, MT6328_PMIC_RG_VGP1_EN_SHIFT);
	#endif
    }
}

static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
	unsigned int i;

    for(i = 0; i < count; i++) {
		
        unsigned cmd;
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
	
		params->type   = LCM_TYPE_DSI;

		params->width  = FRAME_WIDTH;
		params->height = FRAME_HEIGHT;

#if (LCM_DSI_CMD_MODE)
		params->dsi.mode   = CMD_MODE;
#else
		params->dsi.mode   = BURST_VDO_MODE;
#endif
	
		// DSI
		/* Command mode setting */
		params->dsi.LANE_NUM				= LCM_FOUR_LANE;
		//The following defined the fomat for data coming from LCD engine.
		params->dsi.data_format.color_order     = LCM_COLOR_ORDER_RGB;
		params->dsi.data_format.trans_seq       = LCM_DSI_TRANS_SEQ_MSB_FIRST;
		params->dsi.data_format.padding         = LCM_DSI_PADDING_ON_LSB;
		params->dsi.data_format.format              = LCM_DSI_FORMAT_RGB888;

		// Highly depends on LCD driver capability.
		params->dsi.packet_size=256;

		// Video mode setting		
		params->dsi.intermediat_buffer_num = 0;

		params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;

		params->dsi.vertical_sync_active				= 4;//2;
		params->dsi.vertical_backporch					= 16;//14;
		params->dsi.vertical_frontporch					= 16;
		params->dsi.vertical_active_line				= FRAME_HEIGHT; //1280+16+16+4=1316

		params->dsi.horizontal_sync_active				= 6;//2;
		params->dsi.horizontal_backporch				= 44;//44;//42;
		params->dsi.horizontal_frontporch				= 44;
		params->dsi.horizontal_active_pixel				= FRAME_WIDTH;//720+44+44+6=814

    	params->dsi.PLL_CLOCK = 208;//215//156;
		// Bit rate calculation
		//params->dsi.pll_div1=0;		// fref=26MHz, fvco=fref*(div1+1)	(div1=0~63, fvco=500MHZ~1GHz)
		//params->dsi.pll_div2=1; 		// div2=0~15: fout=fvo/(2*div2)
		//params->dsi.fbk_div =11;    //fref=26MHz, fvco=fref*(fbk_div+1)*2/(div1_real*div2_real)

             //FPS=

		#if 1
		params->dsi.clk_lp_per_line_enable = 0;
		params->dsi.esd_check_enable = 1;
		params->dsi.customization_esd_check_enable = 1;
		params->dsi.lcm_esd_check_table[0].cmd          = 0x0a;
		params->dsi.lcm_esd_check_table[0].count        = 1;
		params->dsi.lcm_esd_check_table[0].para_list[0] = 0x9C;
		params->dsi.vertical_vfp_lp = 100;
		#endif

}

static unsigned int lcm_compare_id(void)
{
	unsigned int id = 0;
	unsigned char buffer[2];
	unsigned int array[16];
	
	lcd_power_en(1);
	MDELAY(50);
	SET_RESET_PIN(1);
	SET_RESET_PIN(0);
	MDELAY(10);

	SET_RESET_PIN(1);
	MDELAY(20);
	
	//push_table(lcm_compare_id_setting, sizeof(lcm_compare_id_setting) / sizeof(struct LCM_setting_table), 1);
	
	array[0] = 0x00023700;// read id return two byte,version and id
	dsi_set_cmdq(array, 1, 1);
	
	read_reg_v2(0xDA, buffer, 1);
	id = buffer[0]; //we only need ID
	
#ifdef BUILD_LK
	printf("%s, LK OTM1283A ID = 0x%08x\n", __func__, id);
#else
	printk("%s, kernel OTM1283A ID = 0x%08x\n",__func__, id);
#endif	
		return (LCM_ID_OTM1283 == id)?1:0;
}


static void lcm_init(void)
{
	lcd_power_en(1);
	MDELAY(50);
	//mt_set_gpio_mode(GPIO_LCD_RST_EN, GPIO_MODE_00);                                                                                                          
	//mt_set_gpio_dir(GPIO_LCD_RST_EN, GPIO_DIR_OUT);
	//mt_set_gpio_out(GPIO_LCD_RST_EN, GPIO_OUT_ONE);
	lcm_gpio_output(0,1);
	MDELAY(10);
	//mt_set_gpio_out(GPIO_LCD_RST_EN, GPIO_OUT_ZERO);
	lcm_gpio_output(0,0);
	MDELAY(10);
	//mt_set_gpio_out(GPIO_LCD_RST_EN, GPIO_OUT_ONE);
	lcm_gpio_output(0,1);
	MDELAY(100);

	push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
	//push_table(lcm_sleep_out_setting, sizeof(lcm_sleep_out_setting) / sizeof(struct LCM_setting_table), 1);
}


static void lcm_suspend(void)
{
	//mt_set_gpio_out(GPIO_LCD_RST_EN, GPIO_OUT_ZERO);
	lcm_gpio_output(0,0);
	MDELAY(5);
	//mt_set_gpio_out(GPIO_LCD_RST_EN, GPIO_OUT_ONE);
	lcm_gpio_output(0,1);
	MDELAY(5);
	
	push_table(lcm_sleep_in_setting, sizeof(lcm_sleep_in_setting) / sizeof(struct LCM_setting_table), 1);
	//mt_set_gpio_out(GPIO_LCD_RST_EN, GPIO_OUT_ZERO);
	lcm_gpio_output(0,0);
}



static void lcm_resume(void)
{
	//mt_set_gpio_out(GPIO_LCD_RST_EN, GPIO_OUT_ONE);
	lcm_gpio_output(0,0);
	MDELAY(10);
	lcm_gpio_output(0,1);
	MDELAY(50);  
	lcm_init();
	//push_table(lcm_sleep_out_setting, sizeof(lcm_sleep_out_setting) / sizeof(struct LCM_setting_table), 1);
}

#if (LCM_DSI_CMD_MODE)
static void lcm_update(unsigned int x, unsigned int y,
                       unsigned int width, unsigned int height)
{
	unsigned int x0 = x;
	unsigned int y0 = y;
	unsigned int x1 = x0 + width - 1;
	unsigned int y1 = y0 + height - 1;

	unsigned char x0_MSB = ((x0>>8)&0xFF);
	unsigned char x0_LSB = (x0&0xFF);
	unsigned char x1_MSB = ((x1>>8)&0xFF);
	unsigned char x1_LSB = (x1&0xFF);
	unsigned char y0_MSB = ((y0>>8)&0xFF);
	unsigned char y0_LSB = (y0&0xFF);
	unsigned char y1_MSB = ((y1>>8)&0xFF);
	unsigned char y1_LSB = (y1&0xFF);

	unsigned int data_array[16];

	data_array[0]= 0x00053902;
	data_array[1]= (x1_MSB<<24)|(x0_LSB<<16)|(x0_MSB<<8)|0x2a;
	data_array[2]= (x1_LSB);
	dsi_set_cmdq(data_array, 3, 1);
	
	data_array[0]= 0x00053902;
	data_array[1]= (y1_MSB<<24)|(y0_LSB<<16)|(y0_MSB<<8)|0x2b;
	data_array[2]= (y1_LSB);
	dsi_set_cmdq(data_array, 3, 1);
	
	data_array[0]= 0x002c3909;
	dsi_set_cmdq(data_array, 1, 0);	
}
#endif

#if 1
static unsigned int lcm_esd_check(void)
{
#ifndef BUILD_LK
	char  buffer[3];
	int   array[4];

	array[0] = 0x00013700;// read id return two byte,version and id
	dsi_set_cmdq(array, 1, 1);
	esd_test_new++;
	read_reg_v2(0x0a, buffer, 1);
	printk("%s, id = 0x%08x\n", __func__, buffer[0]);
	if(buffer[0]==0x9c)
	{	
		if((esd_test_new%15) == 0) return TRUE;		//every 30S ESD fail a time
		return FALSE;
	}
	else
	{			 
		return TRUE;
	}
#endif

}

static unsigned int lcm_esd_recover(void)
{
	lcd_power_en(0);
	MDELAY(2000);
	lcm_init();
	printk("%s\n", __func__);
	return TRUE;
}
#endif

static unsigned int lcm_ata_check(unsigned char *buffer)
{
    extern int lcm_ata_check_flag;

    return (lcm_ata_check_flag > 0) ? 1 : 0;
}

LCM_DRIVER otm1283a_hd720_dsi_vdo_lg_lcm_drv = 
{
    .name			= "otm1283a_dsi_vdo_lg",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init           = lcm_init,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
	.compare_id     = lcm_compare_id,
	.esd_check 		= lcm_esd_check,
	.esd_recover 	= lcm_esd_recover,	
	.ata_check		= lcm_ata_check,
#if (LCM_DSI_CMD_MODE)
    .update         = lcm_update,
#endif
};
