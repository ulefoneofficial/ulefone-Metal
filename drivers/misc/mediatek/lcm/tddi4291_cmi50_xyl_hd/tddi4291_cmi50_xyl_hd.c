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

#define FRAME_WIDTH  (720)
#define FRAME_HEIGHT (1280)

#ifndef TRUE
    #define TRUE 1
#endif

#ifndef FALSE
    #define FALSE 0
#endif

// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util = {0};

extern void synaptics_rmi4_irq_enable_lcd(bool enable);
//extern void dsi0_mipi_low_power(bool enable);
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

#if 0
static void lcd_mipi_mode(unsigned char mode)
{
	if(mode == GPIO_MODE_00)
  	{/*
  		mt_set_gpio_mode(LCM_TDP0_GPIO, GPIO_MODE_00);                                                                                                                      
		mt_set_gpio_dir(LCM_TDP0_GPIO, GPIO_DIR_OUT);
		mt_set_gpio_out(LCM_TDP0_GPIO, GPIO_OUT_ZERO);

		mt_set_gpio_mode(LCM_TDN0_GPIO, GPIO_MODE_00);                                                                                                                      
		mt_set_gpio_dir(LCM_TDN0_GPIO, GPIO_DIR_OUT);
		mt_set_gpio_out(LCM_TDN0_GPIO, GPIO_OUT_ZERO);

		mt_set_gpio_mode(LCM_TDP1_GPIO, GPIO_MODE_00);                                                                                                                      
		mt_set_gpio_dir(LCM_TDP1_GPIO, GPIO_DIR_OUT);
		mt_set_gpio_out(LCM_TDP1_GPIO, GPIO_OUT_ZERO);

		mt_set_gpio_mode(LCM_TDN1_GPIO, GPIO_MODE_00);                                                                                                                      
		mt_set_gpio_dir(LCM_TDN1_GPIO, GPIO_DIR_OUT);
		mt_set_gpio_out(LCM_TDN1_GPIO, GPIO_OUT_ZERO);

		mt_set_gpio_mode(LCM_TCP_GPIO, GPIO_MODE_00);                                                                                                                      
		mt_set_gpio_dir(LCM_TCP_GPIO, GPIO_DIR_OUT);
		mt_set_gpio_out(LCM_TCP_GPIO, GPIO_OUT_ZERO);

		mt_set_gpio_mode(LCM_TCN_GPIO, GPIO_MODE_00);                                                                                                                      
		mt_set_gpio_dir(LCM_TCN_GPIO, GPIO_DIR_OUT);
		mt_set_gpio_out(LCM_TCN_GPIO, GPIO_OUT_ZERO);

		mt_set_gpio_mode(LCM_TDP2_GPIO, GPIO_MODE_00);                                                                                                                      
		mt_set_gpio_dir(LCM_TDP2_GPIO, GPIO_DIR_OUT);
		mt_set_gpio_out(LCM_TDP2_GPIO, GPIO_OUT_ZERO);

		mt_set_gpio_mode(LCM_TDN2_GPIO, GPIO_MODE_00);                                                                                                                      
		mt_set_gpio_dir(LCM_TDN2_GPIO, GPIO_DIR_OUT);
		mt_set_gpio_out(LCM_TDN2_GPIO, GPIO_OUT_ZERO);

		mt_set_gpio_mode(LCM_TDP3_GPIO, GPIO_MODE_00);                                                                                                                      
		mt_set_gpio_dir(LCM_TDP3_GPIO, GPIO_DIR_OUT);
		mt_set_gpio_out(LCM_TDP3_GPIO, GPIO_OUT_ZERO);

		mt_set_gpio_mode(LCM_TDN3_GPIO, GPIO_MODE_00);                                                                                                                      
		mt_set_gpio_dir(LCM_TDN3_GPIO, GPIO_DIR_OUT);
		mt_set_gpio_out(LCM_TDN3_GPIO, GPIO_OUT_ZERO);
*/

		mt_set_gpio_mode(GPIO_LCD_RST_EN, GPIO_MODE_00);                                                                                                                      
		mt_set_gpio_dir(GPIO_LCD_RST_EN, GPIO_DIR_OUT);
		mt_set_gpio_out(GPIO_LCD_RST_EN, GPIO_OUT_ZERO);

		//mt_set_gpio_mode(LCM_TP_INT_GPIO, GPIO_MODE_00);                                                                                                                      
		//mt_set_gpio_dir(LCM_TP_INT_GPIO, GPIO_DIR_OUT);
		//mt_set_gpio_out(LCM_TP_INT_GPIO, GPIO_OUT_ZERO);
	}
	else
	{
/*		mt_set_gpio_mode(LCM_TDP0_GPIO, GPIO_MODE_01); 
		mt_set_gpio_mode(LCM_TDN0_GPIO, GPIO_MODE_01);  
		mt_set_gpio_mode(LCM_TDP1_GPIO, GPIO_MODE_01);
		mt_set_gpio_mode(LCM_TDN1_GPIO, GPIO_MODE_01);
		mt_set_gpio_mode(LCM_TCP_GPIO, GPIO_MODE_01);
		mt_set_gpio_mode(LCM_TCN_GPIO, GPIO_MODE_01); 
		mt_set_gpio_mode(LCM_TDP2_GPIO, GPIO_MODE_01);  
		mt_set_gpio_mode(LCM_TDN2_GPIO, GPIO_MODE_01); 
		mt_set_gpio_mode(LCM_TDP3_GPIO, GPIO_MODE_01); 
		mt_set_gpio_mode(LCM_TDN3_GPIO, GPIO_MODE_01);
*/

		//mt_set_gpio_out(GPIO_LCD_RST_EN, GPIO_OUT_ONE);
		//mt_set_gpio_mode(LCM_TP_INT_GPIO, GPIO_MODE_04);                                                                                                                      
	}
}

static void lcd_power_en(unsigned char enabled)
{
    if (enabled)
    {
#ifdef BUILD_LK
	 /* VGP1_PMU 3V */
      //pmic_config_interface(MT6328_PMIC_RG_VGP1_VOSEL_ADDR, 0x6, MT6328_PMIC_RG_VGP1_VOSEL_MASK, MT6328_PMIC_RG_VGP1_VOSEL_SHIFT);
      //pmic_config_interface(MT6328_PMIC_RG_VGP1_EN_ADDR, 0x1, MT6328_PMIC_RG_VGP1_EN_MASK, MT6328_PMIC_RG_VGP1_EN_SHIFT);
      //pmic_config_interface(PMIC_RG_VGP1_EN, 0x1, MT6328_PMIC_RG_VGP1_EN_MASK, MT6328_PMIC_RG_VGP1_EN_SHIFT); 
	//pmic_config_interface(MT6328_PMIC_RG_VGP1_NDIS_EN_ADDR, 0x1, MT6328_PMIC_RG_VGP1_NDIS_EN_MASK, MT6328_PMIC_RG_VGP1_NDIS_EN_SHIFT); 
	//MDELAY(200);
	//printf("%s, lcd_power_en = %d\n", __func__, enabled);

	/* VGP3_PMU 1.8V */
	//pmic_config_interface(MT6328_PMIC_RG_VGP1_VOSEL_ADDR, 0x3, MT6328_PMIC_RG_VGP1_VOSEL_MASK, MT6328_PMIC_RG_VGP1_VOSEL_SHIFT);
	//pmic_config_interface(MT6328_PMIC_RG_VGP1_EN_ADDR, 0x1, MT6328_PMIC_RG_VGP1_EN_MASK, MT6328_PMIC_RG_VGP1_EN_SHIFT); 
#else
       lcd_mipi_mode(GPIO_MODE_00);
	mt_set_gpio_mode(GPIO_LCD_FLASH_POWER_EN, GPIO_MODE_00);                                                                                                                      
	mt_set_gpio_dir(GPIO_LCD_FLASH_POWER_EN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_LCD_FLASH_POWER_EN, GPIO_OUT_ONE);
    	pmic_config_interface(MT6328_PMIC_RG_VGP1_VOSEL_ADDR, 0x6, MT6328_PMIC_RG_VGP1_VOSEL_MASK, MT6328_PMIC_RG_VGP1_VOSEL_SHIFT);
       pmic_config_interface(MT6328_PMIC_RG_VGP1_EN_ADDR, 0x1, MT6328_PMIC_RG_VGP1_EN_MASK, MT6328_PMIC_RG_VGP1_EN_SHIFT);
#endif
        //        mt_set_gpio_out(GPIO_LCM_PWR, GPIO_OUT_ONE);
    }
    else
    {
#ifdef BUILD_LK
	/* VGP3_PMU 1.8V */
        //pmic_config_interface(DIGLDO_CON9, 0x0, PMIC_RG_VGP3_EN_MASK, PMIC_RG_VGP3_EN_SHIFT);
	 //pmic_config_interface(DIGLDO_CON30, 0x0, PMIC_RG_VGP3_VOSEL_MASK, PMIC_RG_VGP3_VOSEL_SHIFT);
	 //MDELAY(200);
        /* VGP1_PMU 3V */
	//pmic_config_interface(MT6328_PMIC_RG_VGP1_VOSEL_ADDR, 0x0, MT6328_PMIC_RG_VGP1_VOSEL_MASK, MT6328_PMIC_RG_VGP1_VOSEL_SHIFT);
	//pmic_config_interface(MT6328_PMIC_RG_VGP1_EN_ADDR, 0x0, MT6328_PMIC_RG_VGP1_EN_MASK, MT6328_PMIC_RG_VGP1_EN_SHIFT); 
#else
	mt_set_gpio_mode(GPIO_LCD_FLASH_POWER_EN, GPIO_MODE_00);                                                                                                                      
	mt_set_gpio_dir(GPIO_LCD_FLASH_POWER_EN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_LCD_FLASH_POWER_EN, GPIO_OUT_ZERO);
	pmic_config_interface(MT6328_PMIC_RG_VGP1_VOSEL_ADDR, 0x0, MT6328_PMIC_RG_VGP1_VOSEL_MASK, MT6328_PMIC_RG_VGP1_VOSEL_SHIFT);
	pmic_config_interface(MT6328_PMIC_RG_VGP1_EN_ADDR, 0x0, MT6328_PMIC_RG_VGP1_EN_MASK, MT6328_PMIC_RG_VGP1_EN_SHIFT);
	lcd_mipi_mode(GPIO_MODE_01);
#endif
        //       mt_set_gpio_out(GPIO_LCM_PWR, GPIO_OUT_ZERO);
    }
}
#endif
#if 1
static void init_lcm_registers(void)
{
	unsigned int data_array[16];
	
	//SET_RESET_PIN(1);
    	//SET_RESET_PIN(0);
    	//MDELAY(20);
    	//SET_RESET_PIN(1);
    	MDELAY(200);
	//dsi0_mipi_low_power(0);
	//SET_RESET_PIN(1);
#if 0
	 data_array[0] = 0x00de0500; 
	dsi_set_cmdq(data_array, 1, 1); 

	 data_array[0] = 0x00210500; 
	dsi_set_cmdq(data_array, 1, 1); 

	 data_array[0] = 0x00df0500; 
	dsi_set_cmdq(data_array, 1, 1); 

	data_array[0] = 0x00110500; // Sleep Out
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(120);
	
	data_array[0] = 0x00290500; // Display On
	dsi_set_cmdq(data_array, 1, 1); 
	MDELAY(120);
#endif
	//data_array[0] = 0x00010500; // 
	//dsi_set_cmdq(data_array, 1, 1); //soft reset
	//MDELAY(120);
	data_array[0] = 0x00DE0500; // 
	dsi_set_cmdq(data_array, 1, 1); //enter register mode


	data_array[0] = 0x32B41500; // 
	dsi_set_cmdq(data_array, 1, 1);


	data_array[0] = 0x70B31500; // 
	dsi_set_cmdq(data_array, 1, 1);


	data_array[0] = 0x10211500; // 
	dsi_set_cmdq(data_array, 1, 1);


	data_array[0] = 0x00DF0500; // 
	dsi_set_cmdq(data_array, 1, 1);


	data_array[0] = 0x00110500; // 
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(120);


	data_array[0] = 0x00290500; // Display On
	dsi_set_cmdq(data_array, 1, 1); 
	MDELAY(120);
	//synaptics_rmi4_irq_enable_lcd(1);
}
#endif

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

       //1 SSD2075 has no TE Pin
		// enable tearing-free
		params->dbi.te_mode 				= LCM_DBI_TE_MODE_DISABLED;
		params->dbi.te_edge_polarity		= LCM_POLARITY_RISING;

        #if (LCM_DSI_CMD_MODE)
		params->dsi.mode   = CMD_MODE;
        #else
		//params->dsi.mode   = SYNC_PULSE_VDO_MODE;
		params->dsi.mode   = BURST_VDO_MODE;
		//params->dsi.mode   = SYNC_EVENT_VDO_MODE; 
		
        #endif
	
		// DSI
		/* Command mode setting */
		//1 Three lane or Four lane
		params->dsi.LANE_NUM				= LCM_FOUR_LANE;
		//The following defined the fomat for data coming from LCD engine.
		params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
		params->dsi.data_format.trans_seq   = LCM_DSI_TRANS_SEQ_MSB_FIRST;
		params->dsi.data_format.padding     = LCM_DSI_PADDING_ON_LSB;
		params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;

		// Highly depends on LCD driver capability.
		// Not support in MT6573
		params->dsi.packet_size=256;

		// Video mode setting		
		params->dsi.intermediat_buffer_num = 0;//because DSI/DPI HW design change, this parameters should be 0 when video mode in MT658X; or memory leakage

		params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;
		params->dsi.word_count=720*3;	
		params->dsi.PLL_CLOCK = 260; //this value must be in MTK suggested table
		
		/* params->dsi.vertical_sync_active				= 3;  //---3 */
		/* params->dsi.vertical_backporch					= 12; //---14 */
		/* params->dsi.vertical_frontporch					= 8;  //----8 */
		/* params->dsi.vertical_active_line				= FRAME_HEIGHT; */

		/* params->dsi.horizontal_sync_active				= 2;  //----2 */
		/* params->dsi.horizontal_backporch				= 28; //----28 */
		/* params->dsi.horizontal_frontporch				= 50; //----50 */
		/* params->dsi.horizontal_active_pixel				= FRAME_WIDTH; */

	params->dsi.vertical_sync_active				= 2;
	params->dsi.vertical_backporch					= 10;
	params->dsi.vertical_frontporch					= 10;
	params->dsi.vertical_active_line				= FRAME_HEIGHT;

	params->dsi.horizontal_sync_active				= 10;
	params->dsi.horizontal_backporch				= 118;
	params->dsi.horizontal_frontporch				= 118;
	params->dsi.horizontal_active_pixel				= FRAME_WIDTH;


		params->dsi.HS_PRPR=3;
		params->dsi.CLK_HS_POST=22;
		params->dsi.DA_HS_EXIT=20;


		// Bit rate calculation
		//1 Every lane speed
		params->dsi.pll_div1=0;		// div1=0,1,2,3;div1_real=1,2,4,4 ----0: 546Mbps  1:273Mbps
		params->dsi.pll_div2=1;		// div2=0,1,2,3;div1_real=1,2,4,4	
		params->dsi.fbk_div =19;    // fref=26MHz, fvco=fref*(fbk_div+1)*2/(div1_real*div2_real)	

	params->dsi.clk_lp_per_line_enable = 0;
	params->dsi.esd_check_enable = 0;
	params->dsi.customization_esd_check_enable = 0;
	params->dsi.lcm_esd_check_table[0].cmd          = 0x0a;
	params->dsi.lcm_esd_check_table[0].count        = 1;
	params->dsi.lcm_esd_check_table[0].para_list[0] = 0x9c;

}

static void lcm_init(void)
{
#if 0
	//dsi0_mipi_low_power(1);
	SET_RESET_PIN(0);
	//synaptics_rmi4_irq_enable_lcd(0);
//	lcd_power_en(0);
	MDELAY(2000);
	//lcd_power_en(0);
	/* MDELAY(10000); */
	//MDELAY(10);
//	lcd_power_en(1);
	/*MDELAY(50);
	mt_set_gpio_mode(GPIO_LCD_RST_EN, GPIO_MODE_00);                                                                                                                      
	mt_set_gpio_dir(GPIO_LCD_RST_EN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_LCD_RST_EN, GPIO_OUT_ONE);
	MDELAY(10);
	mt_set_gpio_out(GPIO_LCD_RST_EN, GPIO_OUT_ZERO);*/
	//MDELAY(10);
	SET_RESET_PIN(1);
	init_lcm_registers();
#endif
}

void lcm_suspend_hs(void)
{
	unsigned int data_array[16];

	data_array[0] = 0x00280508; // Display Off
	dsi_set_cmdq(data_array, 1, 1); 
	MDELAY(10);
	data_array[0] = 0x00100508; ; // Sleep In
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(100);
}

static void lcm_suspend(void)
{
#if 1
	unsigned int data_array[16];
	data_array[0] = 0x00280500; // Display Off
	dsi_set_cmdq(data_array, 1, 1); 
	MDELAY(10);
	data_array[0] = 0x00100500; ; // Sleep In
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(10);
	//synaptics_rmi4_irq_enable_lcd(1);
	lcm_gpio_output(0,0);
#endif
}

static void lcm_resume(void)
{
#if 1
   //1 do lcm init again to solve some display issue
	//_set_gpio_mode(GPIO_LCD_RST_EN, GPIO_MODE_00);                                                                                                                      
	//_set_gpio_dir(GPIO_LCD_RST_EN, GPIO_DIR_OUT);
	//_set_gpio_out(GPIO_LCD_RST_EN, GPIO_OUT_ONE);
	//it_lcm_registers();
	//SET_RESET_PIN(0);
	//MDELAY(50);
	//SET_RESET_PIN(1);
	lcm_gpio_output(0,1);
	init_lcm_registers();
#endif
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
	unsigned char buffer[2];
	unsigned int array[16];  

//    	lcd_power_en(1);
	MDELAY(50);
	SET_RESET_PIN(1);
	MDELAY(10);
	SET_RESET_PIN(0);
	MDELAY(50);
	SET_RESET_PIN(1);
	MDELAY(500);      

	array[0]=0x00DE0500;
    	dsi_set_cmdq(array, 1, 1);

    	array[0]=0x32B41500; 
    	dsi_set_cmdq(array, 1, 1);

    	array[0]=0x00DF0500;
    	dsi_set_cmdq(array, 1, 1);

	array[0] = 0x00013700;// read id return two byte,version and id
	dsi_set_cmdq(array, 1, 1);

	read_reg_v2(0x0a, buffer, 1);
	
    #ifdef BUILD_LK
	printf("%s, LK ssd2075 id = 0x%08x\n", __func__, buffer[0]);
   #else
	printk("%s, Kernel ssd2075 id = 0x%08x\n", __func__, buffer[0]);
   #endif

   return (0x9c == buffer[0])?1:0; 
	//return 1;


}

#if 0
static unsigned int lcm_esd_check(void)
{
#ifndef BUILD_LK
	char  buffer[3];
	int   array[4];

	/// please notice: the max return packet size is 1
	/// if you want to change it, you can refer to the following marked code
	/// but read_reg currently only support read no more than 4 bytes....
	/// if you need to read more, please let BinHan knows.
	/*
			unsigned int data_array[16];
			unsigned int max_return_size = 1;
			
			data_array[0]= 0x00003700 | (max_return_size << 16);	
			
			dsi_set_cmdq(&data_array, 1, 1);
	*/
/*	
	array[0]=0x00DE0500;
    	dsi_set_cmdq(array, 1, 1);

    	array[0]=0x32B41500; 
    	dsi_set_cmdq(array, 1, 1);

    	array[0]=0x00DF0500;
    	dsi_set_cmdq(array, 1, 1);
*/
	array[0] = 0x00013700;// read id return two byte,version and id
	dsi_set_cmdq(array, 1, 1);
	esd_test++;
	read_reg_v2(0x0a, buffer, 1);
	printk("%s, id = 0x%08x\n", __func__, buffer[0]);
	if(buffer[0]==0x9c)
	{	
		//if((esd_test%15) == 0) return TRUE;		//every 30S ESD fail a time
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

LCM_DRIVER tddi4291_cmi50_xyl_hd_lcm_drv = 
{
    .name			= "tddi4291_cmi50_xyl_hd",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init           = lcm_init,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
	.compare_id    = lcm_compare_id,
//	.ata_check		= lcm_ata_check,
//	.init_power		= lcm_init_power,
//       .resume_power = lcm_resume_power,
//       .suspend_power = lcm_suspend_power,
	//.esd_check = lcm_esd_check,
	//.esd_recover = lcm_esd_recover,
    #if (LCM_DSI_CMD_MODE)
    .update         = lcm_update,
    #endif
    };
