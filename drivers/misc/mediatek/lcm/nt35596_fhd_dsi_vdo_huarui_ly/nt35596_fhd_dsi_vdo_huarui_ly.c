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
//  #include <mach/mt_gpio.h>
//  #include <mach/mt_pm_ldo.h>
//  #include <mach/upmu_common.h>
//    #include <linux/i2c.h>
#endif
#include "../nt50358_dcdc.h"
// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------

#define FRAME_WIDTH  (1080)
#define FRAME_HEIGHT (1920)


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

#define SET_RESET_PIN(v)    (lcm_util.set_reset_pin((v)))

#define UDELAY(n) (lcm_util.udelay(n))
#define MDELAY(n) (lcm_util.mdelay(n))

// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)        lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)           lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)                                          lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)                      lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)                                           lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size)                   lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)   


#define LCM_DSI_CMD_MODE                                        0

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
    } else {
        //pmic_config_interface(MT6328_PMIC_RG_VGP1_VOSEL_ADDR, 0x0, MT6328_PMIC_RG_VGP1_VOSEL_MASK, MT6328_PMIC_RG_VGP1_VOSEL_SHIFT);
        //pmic_config_interface(MT6328_PMIC_RG_VGP1_EN_ADDR, 0x0, MT6328_PMIC_RG_VGP1_EN_MASK, MT6328_PMIC_RG_VGP1_EN_SHIFT);
    #ifdef LCM_USE_NT50358_DCDC
        lcm_gpio_output(1,0);
        lcm_gpio_output(2,0);
    #endif
    }
}


void TC358768_DCS_write_1A_1P(unsigned char cmd, unsigned char para)
{
    unsigned int data_array[16];
    //unsigned char buffer;

#if 0//ndef BUILD_LK

    do {
        data_array[0] =(0x00001500 | (para<<24) | (cmd<<16));
        dsi_set_cmdq(data_array, 1, 1);

        if (cmd == 0xFF)
            break;

        read_reg_v2(cmd, &buffer, 1);

        if(buffer != para)
            printk("%s, data_array = 0x%08x, (cmd, para, back) = (0x%02x, 0x%02x, 0x%02x)\n", __func__, data_array[0], cmd, para, buffer);  

        MDELAY(1);

    } while (buffer != para);

#else

    data_array[0] =(0x00001500 | (para<<24) | (cmd<<16));
    dsi_set_cmdq(data_array, 1, 1);

    //MDELAY(1);

#endif

}

#define TC358768_DCS_write_1A_0P(cmd)                           data_array[0]=(0x00000500 | (cmd<<16)); \
                                                                dsi_set_cmdq(data_array, 1, 1);

static void init_lcm_registers(void)
{
    unsigned int data_array[16];
    //unsigned char buffer[8];

    TC358768_DCS_write_1A_1P(0xFF,0x05);
    MDELAY(10);
    TC358768_DCS_write_1A_1P(0xFB,0x01);
    TC358768_DCS_write_1A_1P(0xC5,0x31);
    TC358768_DCS_write_1A_1P(0xFF,0x05);
    MDELAY(10);
    TC358768_DCS_write_1A_1P(0x90,0x00);
    TC358768_DCS_write_1A_1P(0x93,0x04);
    TC358768_DCS_write_1A_1P(0x94,0x04);

    TC358768_DCS_write_1A_1P(0x9B,0x0F);

    TC358768_DCS_write_1A_1P(0xA4,0x0F);
    TC358768_DCS_write_1A_1P(0x91,0x44);
    TC358768_DCS_write_1A_1P(0x92,0x79);

    TC358768_DCS_write_1A_1P(0x00,0x0F);
    TC358768_DCS_write_1A_1P(0x01,0x00);
    TC358768_DCS_write_1A_1P(0x02,0x00);
    TC358768_DCS_write_1A_1P(0x03,0x00);
    TC358768_DCS_write_1A_1P(0x04,0x0B);
    TC358768_DCS_write_1A_1P(0x05,0x0C);
    TC358768_DCS_write_1A_1P(0x06,0x00);
    TC358768_DCS_write_1A_1P(0x07,0x00);
    TC358768_DCS_write_1A_1P(0x08,0x00);
    TC358768_DCS_write_1A_1P(0x09,0x00);
    TC358768_DCS_write_1A_1P(0x0A,0x03);
    TC358768_DCS_write_1A_1P(0x0B,0x04);
    TC358768_DCS_write_1A_1P(0x0C,0x01);
    TC358768_DCS_write_1A_1P(0x0D,0x13);
    TC358768_DCS_write_1A_1P(0x0E,0x15);
    TC358768_DCS_write_1A_1P(0x0F,0x17);

    TC358768_DCS_write_1A_1P(0x10,0x0F);
    TC358768_DCS_write_1A_1P(0x11,0x00);
    TC358768_DCS_write_1A_1P(0x12,0x00);
    TC358768_DCS_write_1A_1P(0x13,0x00);
    TC358768_DCS_write_1A_1P(0x14,0x0B);
    TC358768_DCS_write_1A_1P(0x15,0x0C);
    TC358768_DCS_write_1A_1P(0x16,0x00);
    TC358768_DCS_write_1A_1P(0x17,0x00);
    TC358768_DCS_write_1A_1P(0x18,0x00);
    TC358768_DCS_write_1A_1P(0x19,0x00);
    TC358768_DCS_write_1A_1P(0x1A,0x03);
    TC358768_DCS_write_1A_1P(0x1B,0x04);
    TC358768_DCS_write_1A_1P(0x1C,0x01);
    TC358768_DCS_write_1A_1P(0x1D,0x13);
    TC358768_DCS_write_1A_1P(0x1E,0x15);
    TC358768_DCS_write_1A_1P(0x1F,0x17);

    TC358768_DCS_write_1A_1P(0x20,0x09);
    TC358768_DCS_write_1A_1P(0x21,0x01);
    TC358768_DCS_write_1A_1P(0x22,0x00);
    TC358768_DCS_write_1A_1P(0x23,0x00);
    TC358768_DCS_write_1A_1P(0x24,0x00);
    TC358768_DCS_write_1A_1P(0x25,0x6D); 

    TC358768_DCS_write_1A_1P(0x2F,0x02);
    TC358768_DCS_write_1A_1P(0x30,0x04);
    TC358768_DCS_write_1A_1P(0x31,0x49);
    TC358768_DCS_write_1A_1P(0x32,0x23);
    TC358768_DCS_write_1A_1P(0x33,0x01);
    TC358768_DCS_write_1A_1P(0x34,0x00);
    TC358768_DCS_write_1A_1P(0x35,0x69);
    TC358768_DCS_write_1A_1P(0x36,0x00);
    TC358768_DCS_write_1A_1P(0x37,0x2D);
    TC358768_DCS_write_1A_1P(0x38,0x08); 

    TC358768_DCS_write_1A_1P(0x29,0x58);
    TC358768_DCS_write_1A_1P(0x2A,0x16);
    TC358768_DCS_write_1A_1P(0x2B,0x0A);

    TC358768_DCS_write_1A_1P(0x5B,0x00);
    TC358768_DCS_write_1A_1P(0x5F,0x75);
    TC358768_DCS_write_1A_1P(0x63,0x00);
    TC358768_DCS_write_1A_1P(0x67,0x04);
    TC358768_DCS_write_1A_1P(0x6C,0x00);
       
    TC358768_DCS_write_1A_1P(0x5C,0x2C);
    TC358768_DCS_write_1A_1P(0x60,0x75);
    TC358768_DCS_write_1A_1P(0x64,0x00);
    TC358768_DCS_write_1A_1P(0x68,0x04);
    TC358768_DCS_write_1A_1P(0x6C,0x10);
       
    TC358768_DCS_write_1A_1P(0x5D,0x00);
    TC358768_DCS_write_1A_1P(0x61,0x75);
    TC358768_DCS_write_1A_1P(0x65,0x00);
    TC358768_DCS_write_1A_1P(0x69,0x04);
    TC358768_DCS_write_1A_1P(0x6C,0x00);
       
    TC358768_DCS_write_1A_1P(0x6C,0x10);
      
    TC358768_DCS_write_1A_1P(0x7A,0x01);

    TC358768_DCS_write_1A_1P(0x7B,0x80);

    TC358768_DCS_write_1A_1P(0x7C,0xD8);
    TC358768_DCS_write_1A_1P(0x7D,0x60);
    TC358768_DCS_write_1A_1P(0x7E,0x07);
    TC358768_DCS_write_1A_1P(0x7F,0x17);
    TC358768_DCS_write_1A_1P(0x80,0x00);
    TC358768_DCS_write_1A_1P(0x81,0x05);
    TC358768_DCS_write_1A_1P(0x82,0x02);
    TC358768_DCS_write_1A_1P(0x83,0x00);
    TC358768_DCS_write_1A_1P(0x84,0x05);
    TC358768_DCS_write_1A_1P(0x85,0x05);
    TC358768_DCS_write_1A_1P(0x86,0x1B);
    TC358768_DCS_write_1A_1P(0x87,0x1B);
    TC358768_DCS_write_1A_1P(0x88,0x1B);
    TC358768_DCS_write_1A_1P(0x89,0x1B);
    TC358768_DCS_write_1A_1P(0x8A,0x33);
    TC358768_DCS_write_1A_1P(0x8B,0x00);
    TC358768_DCS_write_1A_1P(0x8C,0x00);

    TC358768_DCS_write_1A_1P(0x73,0xD0);
    TC358768_DCS_write_1A_1P(0x74,0x0D);
    TC358768_DCS_write_1A_1P(0x75,0x03);
    TC358768_DCS_write_1A_1P(0x76,0x16);

    TC358768_DCS_write_1A_1P(0x99,0x33);
    TC358768_DCS_write_1A_1P(0x98,0x00);

    TC358768_DCS_write_1A_1P(0xFF,0x01);
    MDELAY(500);

    TC358768_DCS_write_1A_1P(0x00,0x01);

    TC358768_DCS_write_1A_1P(0x05,0x50);

    TC358768_DCS_write_1A_1P(0x06,0x4A);

    TC358768_DCS_write_1A_1P(0x14,0xA8);
    TC358768_DCS_write_1A_1P(0x07,0xB2);

    TC358768_DCS_write_1A_1P(0x0E,0xB5);
    TC358768_DCS_write_1A_1P(0x0F,0xB8);

    TC358768_DCS_write_1A_1P(0x0B,0x69);
    TC358768_DCS_write_1A_1P(0x0C,0x69);

    TC358768_DCS_write_1A_1P(0x11,0x13);
    TC358768_DCS_write_1A_1P(0x12,0x13);

    TC358768_DCS_write_1A_1P(0x08,0x0C);
    TC358768_DCS_write_1A_1P(0x15,0x13);
    TC358768_DCS_write_1A_1P(0x16,0x13);

    TC358768_DCS_write_1A_1P(0xFF,0xE0);
    MDELAY(1);

    TC358768_DCS_write_1A_1P(0x46,0x63);

    TC358768_DCS_write_1A_1P(0xFB,0x01);

    TC358768_DCS_write_1A_1P(0xFF,0x00);
    MDELAY(1);
    TC358768_DCS_write_1A_1P(0xD3,0x07);
    TC358768_DCS_write_1A_1P(0xD4,0x08);

    TC358768_DCS_write_1A_0P( 0x11);
    MDELAY(120);
    TC358768_DCS_write_1A_0P( 0x29);
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
    //params->dsi.switch_mode = CMD_MODE;
#endif
    params->dsi.switch_mode_enable = 0;

    // DSI
    /* Command mode setting */
    params->dsi.LANE_NUM                    = LCM_FOUR_LANE;
    //The following defined the fomat for data coming from LCD engine.
    params->dsi.data_format.color_order     = LCM_COLOR_ORDER_RGB;
    params->dsi.data_format.trans_seq       = LCM_DSI_TRANS_SEQ_MSB_FIRST;
    params->dsi.data_format.padding         = LCM_DSI_PADDING_ON_LSB;
    params->dsi.data_format.format              = LCM_DSI_FORMAT_RGB888;

    // Highly depends on LCD driver capability.
    params->dsi.packet_size=256;
    //video mode timing

    params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;

    params->dsi.vertical_sync_active                = 1;      //2
    params->dsi.vertical_backporch                  = 4;      //8 
    params->dsi.vertical_frontporch                 = 5;     //10
    params->dsi.vertical_active_line                    = FRAME_HEIGHT;

    params->dsi.horizontal_sync_active              = 5;     //10
    params->dsi.horizontal_backporch                = 40;     //20   
    params->dsi.horizontal_frontporch               = 96;     //40    
    params->dsi.horizontal_active_pixel             = FRAME_WIDTH;

#if (LCM_DSI_CMD_MODE)
    params->dsi.PLL_CLOCK = 500; //this value must be in MTK suggested table
#else
    params->dsi.PLL_CLOCK = 450; //this value must be in MTK suggested table
#endif

    params->dsi.clk_lp_per_line_enable = 0;
    params->dsi.esd_check_enable = 1;
    params->dsi.customization_esd_check_enable = 0;
    params->dsi.lcm_esd_check_table[0].cmd          = 0x0a;
    params->dsi.lcm_esd_check_table[0].count        = 1;
    params->dsi.lcm_esd_check_table[0].para_list[0] = 0x9c;

}

#if 0
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
#endif

static void lcm_init(void)
{
    //unsigned int data_array[16];
    printk("nt35596 %s", __func__);
    lcd_power_en(0);
    MDELAY(1000); 
    //MDELAY(10);
    lcm_gpio_output(0,0);
    MDELAY(10);
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
    //unsigned char buffer[2];

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
    lcd_power_en(1);
    MDELAY(50);
    lcm_gpio_output(0,0);
    MDELAY(10);
    lcm_gpio_output(0,1);
    MDELAY(50);  
    init_lcm_registers();
    //printk("%s, zhangchongyong\n", __func__);
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
    //unsigned int id0,id1,id=0;
    unsigned char buffer[2];
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
    return 1;
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

    read_reg_v2(0xF4, buffer, 2);
    
#ifdef BUILD_LK
    printf("%s, LK huarui ly nt35596 id = 0x%08x\n", __func__, buffer[0]);
#else
    printk("%s, Kernel huarui ly nt35596 id = 0x%08x\n", __func__, buffer[0]);
#endif

    return (0x96 == buffer[0])?1:0;
}

#if 0
static unsigned int lcm_esd_check(void)
{
#ifndef BUILD_LK
    char buffer[3];
    int array[4];

    /// please notice: the max return packet size is 1
    /// if you want to change it, you can refer to the following marked code
    /// but read_reg currently only support read no more than 4 bytes
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

    read_reg_v2(0x0a, buffer, 1);
    if (buffer[0]==0x9c) {
        return FALSE;
    } else {
        return TRUE;
    }
#endif
}

static unsigned int lcm_esd_recover(void)
{
    lcm_init();
    lcm_resume();

    return TRUE;
}
#endif

static unsigned int lcm_ata_check(unsigned char *buffer)
{
    extern int lcm_ata_check_flag;

    return (lcm_ata_check_flag > 0) ? 1 : 0;
}

LCM_DRIVER nt35596_fhd_dsi_vdo_huarui_ly_lcm_drv = 
{
    .name           = "nt35596_fhd_dsi_vdo_huarui_ly",
    .set_util_funcs = lcm_set_util_funcs,
    .get_params     = lcm_get_params,
    .init           = lcm_init,
    .suspend        = lcm_suspend,
    .resume         = lcm_resume,
    .compare_id     = lcm_compare_id,
    .ata_check      = lcm_ata_check,
    //.init_power     = lcm_init_power,
    //.resume_power   = lcm_resume_power,
    //.suspend_power  = lcm_suspend_power,
    //.esd_check      = lcm_esd_check,
    //.esd_recover    = lcm_esd_recover,
#if (LCM_DSI_CMD_MODE)
    .update         = lcm_update,
#endif
};
