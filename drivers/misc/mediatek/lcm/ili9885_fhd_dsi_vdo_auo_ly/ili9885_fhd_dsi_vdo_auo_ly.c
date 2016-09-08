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
    //#include <mach/mt_gpio.h>
    //#include <mach/mt_pm_ldo.h>
    #include <mt-plat/upmu_common.h>
    //#include <linux/i2c.h>
#endif
#include "../nt50358_dcdc.h"
// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------

#define FRAME_WIDTH  (1080)
#define FRAME_HEIGHT (1920)

#define SET_RESET_PIN(v)    								(lcm_util.set_reset_pin((v)))
#define UDELAY(n) 	(lcm_util.udelay(n))
#define MDELAY(n) 	(lcm_util.mdelay(n))

#define GPIO_LCD_RST_EN      (GPIO146| 0x80000000)
#define GPIO_LCD_ENP_EN      (GPIO82 | 0x80000000)
#define GPIO_LCD_ENN_EN      (GPIO88 | 0x80000000)

#ifndef TRUE
    #define TRUE 1
#endif

#ifndef FALSE
    #define FALSE 0
#endif
#define REGFLAG_DELAY             							0XFE
#define REGFLAG_END_OF_TABLE      							0xFF   // END OF REGISTERS MARKER
// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util = {0};

#define SET_RESET_PIN(v)    (lcm_util.set_reset_pin((v)))

#define UDELAY(n) (lcm_util.udelay(n))
#define MDELAY(n) (lcm_util.mdelay(n))

// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	        lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)										lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)					lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)											lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size)   				lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)   


#define   LCM_DSI_CMD_MODE							0


struct LCM_setting_table {
    unsigned cmd;
    unsigned char count;
    unsigned char para_list[128];
};
static struct LCM_setting_table lcm_initialization_setting[] = {

{0xB0,3,{0x98,0x85,0x0A}},
{REGFLAG_DELAY,20,{}}, 

{0xC4,7,{0x70,0x19,0x23,0x00,0x00F,0x0F,0x00}}, //set SDT
{0xD0,6,{0x55,0x05,0x34,0x6B,0x0EC,0xC0}}, //VGH=3AVDD-AVEE=16V(single), VGL=2AVEE-AVDD=-14V(dual)
{0xD3,9,{0x33,0x33,0x05,0x03,0x059,0x59,0x22,0x26,0x22}}, //GVDDP=4.3V GVDDN=-4.3V VGHO=15 VGLO=-12 AVDDR=4.7V AVEER=-4.7V
//{0xD5,10,{0x8B,0x00,0x00,0x00,0x01,0x8A,0x01,0x8A,0x00,0xFF }},//set Vcom




{0xEC,07,{0x76,0x1E,0x32,0x00,0x46,0x00,0x02}}, //black display while video stop
{0xEF,01,{0x8F}}, //power saving
//set LVD sequence
{0xEB,35,{0xA3,0xC7,0x73,0x00,0x58,0x55,0x55,0x55,0x55,0x50,0x00,0x00,0x00,0x00,0x00,0x25,0xCD,0x0F,0xFF,0xFF,0xFF,0xFF,0xFF,0x55,0x55,0x55,0x55,0x32,0x77,0x55,0x43,0x45,0x5E,0xFF,0x55}}, //VCOM tie GND during sleep in & source GND for 2 frames


//GIP setting
{0xE5,73,{0x36,0x36,0xA1,0xF6,0xF6,0x47,0x07,0x55,0x15,0x63,0x23,0x71,0x31,0x3E,0x37,0x85,0x36,0x36,0x36,0x36,0x36,0x36,0xA8,0xF6,0xF6,0x4E,0x0E,0x5C,0x1C,0x6A,0x2A,0x78,0x38,0x3E,0x37,
	0x8C,0x36,0x36,0x36,0x36,0x18,0x70,0x61,0x00,0x4E,0xBB,0x70,0x80,0x00,0x4E,0xBB,0xF7,0x00,0x4E,0xBB,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x07}},
{0xEA,66,{0x51,0x00,0x00,0x00,0x00,0x00,0x00,0x40,0x00,0x00,0x00,0x00,0x00,0x00,0x0F,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x70,0x01,0x10,0x00,0x40,0x80,
	0xC0,0x00,0x00,0x01,0x23,0x45,0x67,0x89,0xAB,0xCD,0xEF,0xCC,0xCC,0x22,0x33,0x33,0x00,0x11,0x00,0x11,0x00,0x11,0x00,0x11,0xCC,0xCC,0x22,0xCC,0xCC,0xCC,0xCC}},
{0xED,23,{0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x40}}, //set reg_dispon_wait_vdo                                                                                                                                                                                    



//gamma setting
{0xC7,122,{0x00,0x19,0x00,0x68,0x00,0x96,0x00,0xB0,0x00,0xD0,0x00,0xE2,0x00,0xF2,0x01,0x04,0x01,0x14,0x01,0x43,0x01,0x66,0x01,0x9F,0x01,0xCB,0x02,0x10,0x02,0x45,0x02,0x47,0x02,0x7A,0x02,
	0xB1,0x02,0xD5,0x03,0x06,0x03,0x24,0x03,0x48,0x03,0x58,0x03,0x61,0x03,0x73,0x03,0x7D,0x03,0x93,0x03,0xA8,0x03,0xC8,0x03,0xE8,0x00,0x19,0x00,0x68,0x00,0x96,0x00,0xB0,0x00,0xD0,0x00,
	0xE2,0x00,0xF2,0x01,0x04,0x01,0x14,0x01,0x43,0x01,0x66,0x01,0x9F,0x01,0xCB,0x02,0x10,0x02,0x45,0x02,0x47,0x02,0x7A,0x02,0xB1,0x02,0xD5,0x03,0x06,0x03,0x24,0x03,0x48,0x03,0x58,0x03,
	0x61,0x03,0x73,0x03,0x7D,0x03,0x93,0x03,0xA8,0x03,0xC8,0x03,0xE8,0x01,0x01}},

{0xD1,03,{0x09,0x09,0xC2}},   

{0xEE,14,{0x22,0x10,0x02,0x02,0x0F,0x40,0x00,0xA7,0x00,0x04,0x00,0x00,0x40,0xB9}},    

{0x11,1,{0x00}},//SLEEP OUT      
{REGFLAG_DELAY,120,{}},     
                                
{0x29,1,{0x00}},//Display ON
{REGFLAG_DELAY,20,{}}, 
};

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
static void lcd_power_en(unsigned char enabled)
{
    if (enabled)
    {
        //pmic_config_interface(MT6328_PMIC_RG_VGP1_VOSEL_ADDR, 0x6, MT6328_PMIC_RG_VGP1_VOSEL_MASK, MT6328_PMIC_RG_VGP1_VOSEL_SHIFT);
        //pmic_config_interface(MT6328_PMIC_RG_VGP1_EN_ADDR, 0x1, MT6328_PMIC_RG_VGP1_EN_MASK, MT6328_PMIC_RG_VGP1_EN_SHIFT);

        //MDELAY(5);
    #ifdef LCM_USE_NT50358_DCDC
        lcm_gpio_output(1,1);

        if (get_dcdc_type()) {
            dw8769_set_discharge_status(0x03,0x80);// REG0x03: 0x83:disbale ;0x80: enbale
        } else {
            nt50358_dcdc_set_bost(0x05,0x30);//bost volatge:5.6V
        }
        //MDELAY(2);
        nt50358_dcdc_set_avdd(0x00,0x0e);//+5.5V
        MDELAY(5);
        lcm_gpio_output(2,1);
        nt50358_dcdc_set_avee(0x01,0x0e);//-5.5V
        MDELAY(5);
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

	
static void init_lcm_registers(void)
{
    push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);

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
	
    params->physical_width = 73;
    params->physical_height = 119;

#if (LCM_DSI_CMD_MODE)
	params->dsi.mode   = CMD_MODE;
	params->dsi.switch_mode = SYNC_PULSE_VDO_MODE;
#else
    	params->dsi.mode   = BURST_VDO_MODE;
	params->dsi.switch_mode = CMD_MODE;
#endif
	params->dsi.switch_mode_enable = 0;

	// DSI
	/* Command mode setting */
	params->dsi.LANE_NUM				= LCM_FOUR_LANE;
	//The following defined the fomat for data coming from LCD engine.
	params->dsi.data_format.color_order 	= LCM_COLOR_ORDER_RGB;
	params->dsi.data_format.trans_seq   	= LCM_DSI_TRANS_SEQ_MSB_FIRST;
	params->dsi.data_format.padding     	= LCM_DSI_PADDING_ON_LSB;
	params->dsi.data_format.format      		= LCM_DSI_FORMAT_RGB888;

	// Highly depends on LCD driver capability.
	params->dsi.packet_size=256;
	//video mode timing

	params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;
/*
	params->dsi.vertical_sync_active				= 1;      //2
	params->dsi.vertical_backporch					= 4;      //8 
	params->dsi.vertical_frontporch					= 5;     //10
	params->dsi.vertical_active_line					= FRAME_HEIGHT;

	params->dsi.horizontal_sync_active				= 5;     //10
	params->dsi.horizontal_backporch				= 40;     //20   
	params->dsi.horizontal_frontporch				= 96;     //40    
	params->dsi.horizontal_active_pixel				= FRAME_WIDTH;
    	//params->dsi.ssc_disable							= 1;
    	*/
       params->dsi.vertical_sync_active				= 6;      //2
	params->dsi.vertical_backporch					= 18;      //8 
	params->dsi.vertical_frontporch					= 20;     //10
	params->dsi.vertical_active_line					= FRAME_HEIGHT;

	params->dsi.horizontal_sync_active				= 20;     //10
	params->dsi.horizontal_backporch				= 80;     //20   
	params->dsi.horizontal_frontporch				= 80;     //40    
	params->dsi.horizontal_active_pixel				= FRAME_WIDTH;
    	params->dsi.ssc_disable							= 1;
#if (LCM_DSI_CMD_MODE)
	params->dsi.PLL_CLOCK = 500; //this value must be in MTK suggested table
#else
	params->dsi.PLL_CLOCK = 473; //this value must be in MTK suggested table
#endif

	params->dsi.clk_lp_per_line_enable = 0;
	params->dsi.esd_check_enable = 1;
	params->dsi.customization_esd_check_enable = 0;
	params->dsi.lcm_esd_check_table[0].cmd          = 0x0a;
	params->dsi.lcm_esd_check_table[0].count        = 1;
	params->dsi.lcm_esd_check_table[0].para_list[0] = 0x9C;
	
}

static void lcm_init_power(void)
{
	//printk("otm1282c %s", __func__);
	lcd_power_en(1);
}

static void lcm_suspend_power(void)
{
		//printk("otm1282c %s", __func__);
}

static void lcm_resume_power(void)
{
	//printk("otm1282c %s", __func__);
}

static void lcm_init(void)
{
    pmic_config_interface(MT6328_PMIC_RG_VGP1_VOSEL_ADDR, 0x0, MT6328_PMIC_RG_VGP1_VOSEL_MASK, MT6328_PMIC_RG_VGP1_VOSEL_SHIFT);
    pmic_config_interface(MT6328_PMIC_RG_VGP1_EN_ADDR, 0x0, MT6328_PMIC_RG_VGP1_EN_MASK, MT6328_PMIC_RG_VGP1_EN_SHIFT);
    lcd_power_en(0);
    /* MDELAY(10000); */
    //MDELAY(10);
    MDELAY(2000);
    lcm_gpio_output(0,0);
    MDELAY(10);
    pmic_config_interface(MT6328_PMIC_RG_VGP1_VOSEL_ADDR, 0x6, MT6328_PMIC_RG_VGP1_VOSEL_MASK, MT6328_PMIC_RG_VGP1_VOSEL_SHIFT);
    pmic_config_interface(MT6328_PMIC_RG_VGP1_EN_ADDR, 0x1, MT6328_PMIC_RG_VGP1_EN_MASK, MT6328_PMIC_RG_VGP1_EN_SHIFT);
    lcd_power_en(1);
    MDELAY(50);
    lcm_gpio_output(0,0);
    MDELAY(10);
    lcm_gpio_output(0,1);
    MDELAY(50);
    init_lcm_registers();
}

static void lcm_suspend(void)
{
    unsigned int data_array[16];

    data_array[0] = 0x00280500; // Display Off
    dsi_set_cmdq(data_array, 1, 1); 
    MDELAY(120);
    data_array[0] = 0x00100500;  // Sleep In
    dsi_set_cmdq(data_array, 1, 1);
    MDELAY(120);

    lcm_gpio_output(0,0);
    lcd_power_en(0);
}


static void lcm_resume(void)
{
    //1 do lcm init again to solve some display issue
    //_set_gpio_mode(GPIO_LCD_RST_EN, GPIO_MODE_00);
    //_set_gpio_dir(GPIO_LCD_RST_EN, GPIO_DIR_OUT);
    //_set_gpio_out(GPIO_LCD_RST_EN, GPIO_OUT_ONE);
    //it_lcm_registers();
    //SET_RESET_PIN(0);
    //MDELAY(50);
    lcd_power_en(1);
    MDELAY(50);
    lcm_gpio_output(0,0);
    MDELAY(10);
    lcm_gpio_output(0,1);
    MDELAY(50);  
    init_lcm_registers();
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

	data_array[0]= 0x00290508; //HW bug, so need send one HS packet
	dsi_set_cmdq(data_array, 1, 1);
	
	data_array[0]= 0x002c3909;
	dsi_set_cmdq(data_array, 1, 0);

}
#endif

static unsigned int lcm_compare_id(void)
{
  unsigned int id=0;
	unsigned char buffer[4];
	unsigned int array[16];
#if 0
    mt_set_gpio_mode(GPIO_LCD_RST_EN, GPIO_MODE_00);
    mt_set_gpio_dir(GPIO_LCD_RST_EN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_LCD_RST_EN, GPIO_OUT_ZERO);
    lcd_power_en(1);
    MDELAY(10);
    mt_set_gpio_out(GPIO_LCD_RST_EN, GPIO_OUT_ZERO);
    MDELAY(50);
    mt_set_gpio_out(GPIO_LCD_RST_EN, GPIO_OUT_ONE);
    MDELAY(500);
#else
    SET_RESET_PIN(1);
    SET_RESET_PIN(0);
    MDELAY(1);

    SET_RESET_PIN(1);
    MDELAY(20);

#endif
  array[0] = 0x00022902;
  array[1] = 0x000000b0;
  dsi_set_cmdq(array, 2, 1);

  array[0] = 0x00043700;// read id return two byte,version and id
  dsi_set_cmdq(array, 1, 1);

  read_reg_v2(0xbf, buffer, 4);
  id = buffer[2]<<8 | buffer[3]; //we only need ID

#ifdef BUILD_LK
	printf("%s id= x%08x,id0 = 0x%08x,id1 = 0x%08x,id2 = 0x%08x,id3 = 0x%08x\n", __func__, id,buffer[0],buffer[1],buffer[2],buffer[3]);
#else
    printk("%s id= x%08x,id0 = 0x%08x,id1 = 0x%08x,id2 = 0x%08x,id3 = 0x%08x\n", __func__, id,buffer[0],buffer[1],buffer[2],buffer[3]);
#endif

    return (0x9885 == id) ? 1 : 0;

}

LCM_DRIVER ili9885_fhd_dsi_vdo_auo_ly_lcm_drv = 
{
    .name			= "ili9885_fhd_dsi_vdo_auo_ly",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init           = lcm_init,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
	.compare_id    = lcm_compare_id,

	.init_power		= lcm_init_power,
    .resume_power = lcm_resume_power,
    .suspend_power = lcm_suspend_power,
	//.esd_check = lcm_esd_check,
 	//.esd_recover = lcm_esd_recover,
    #if (LCM_DSI_CMD_MODE)
    .update         = lcm_update,
    #endif
    };
