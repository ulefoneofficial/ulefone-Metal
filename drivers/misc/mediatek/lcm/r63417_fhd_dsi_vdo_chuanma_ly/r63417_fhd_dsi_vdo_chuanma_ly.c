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

#define LCM_ID_RM63417          0x3417

#ifndef TRUE
    #define TRUE 1
#endif

#ifndef FALSE
    #define FALSE 0
#endif
#define REGFLAG_DELAY                                       0XFE
#define REGFLAG_END_OF_TABLE                                0xFF   // END OF REGISTERS MARKER
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

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)    lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)       lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)                                      lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)                  lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)                                       lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size)               lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)


#define LCM_DSI_CMD_MODE                                    0

/*
static struct LCM_setting_table {
    unsigned char cmd;
    unsigned char count;
    unsigned char para_list[64];
};
static struct LCM_setting_table lcm_initialization_setting[] = {

    {REGFLAG_DELAY, 100, {}},

    {0xB0,1,{0x04}},

    {0xd6,1,{0x01}},

    {0xB3,6,{0x1C,0x00,0x00,0x00,0x00,0x0}},

    {0xB4,2,{0x0C,0x00}},

    {0xB6,2,{0x3A,0xD3}},

    {0xC0,1,{0x22}},

    {0xC1,34,{0x84,0x60,0x00,0xcf,0x1c,0x94,0x82,0x52,0xEF,0xBD,0xF7,0x58,0x73,0xAE,0xB1,0xF7,0xDE,0x7B,0x4A,0x81,0x42,0x86,0x78,0x00,0x00,0x00,0x00,0x00,0x62,0x04,0x06,0x2A,0x00,0x01}},

    {0xC2,7,{0x31,0xF7,0x80,0x08,0x08,0x00,0x00}},

    {0xC4,22,{0x70,0x00,0x00,0x11,0x03,0x00,0x00,0x00,0x00,0x00,0x03,0x00,0x00,0x00,0x11,0x03,0x00,0x00,0x00,0x00,0x00,0x03}},

    {0xC6,40,{0x73,0x03,0x69,0x2A,0x63,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x08,0x15,0x0A,0x73,0x03,0x69,0x2A,0x63,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x08,0x15,0x0A}},

    {0xC7,30,{0x00,0x09,0x10,0x18,0x26,0x33,0x3D,0x4C,0x31,0x39,0x45,0x55,0x5F,0x69,0x7F,0x00,0x09,0x10,0x18,0x26,0x33,0x3D,0x4C,0x31,0x39,0x45,0x55,0x5F,0x69,0x7F}},

    {0xCB,9,{0xFE,0xE0,0x07,0x7F,0x00,0x00,0x00,0x00,0xC0}},

    {0xCC,1,{0x03}},

    {0xD0,10,{0x2A,0x82,0xBB,0x28,0x11,0x4C,0x19,0x19,0x0C,0x00}},

    {0xD3,25,{0x1B,0x33,0xBB,0xBB,0xB3,0x33,0x33,0x33,0x00,0x01,0x00,0xA0,0x88,0xA0,0x08,0x39,0x39,0x33,0x3B,0x37,0x72,0x07,0x3D,0xBF,0x55}},

    {0xD5,7,{0x06,0x00,0x00,0x01,0x39,0x01,0x39}},

    {0x11, 0,{0x00}},
    {REGFLAG_DELAY, 200, {}},
    {0x29, 0,{0x00}},
    {REGFLAG_DELAY, 200, {}},
    {REGFLAG_END_OF_TABLE, 0x00, {}},
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
    
}*/
static void lcd_power_en(unsigned char enabled)
{
    if (enabled) {
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

static void init_lcm_registers(void)
{
    unsigned int data_array[16];
    data_array[0] = 0x00110500;
    dsi_set_cmdq(data_array, 1, 1);
    MDELAY(120);
    data_array[0] = 0x00022902;
    data_array[1] = 0x000004b0;
    dsi_set_cmdq(data_array, 2, 1);
    MDELAY(10);
    /*data_array[0] = 0x00000500;
    dsi_set_cmdq(data_array, 1, 1);
    data_array[0] = 0x00000500;
    dsi_set_cmdq(data_array, 1, 1); */

    data_array[0] = 0x00022902;
    data_array[1] = 0x000001d6;
    dsi_set_cmdq(data_array, 2, 1);
    MDELAY(10);

    data_array[0] = 0x00072902;
    data_array[1] = 0x00001cb3;
    data_array[2] = 0x00000000;
    dsi_set_cmdq(data_array, 3, 1);
    MDELAY(10);

    data_array[0] = 0x00032902;
    data_array[1] = 0x00000cb4;
    dsi_set_cmdq(data_array, 2, 1);
    MDELAY(10);

    data_array[0] = 0x00032902;
    data_array[1] = 0x00d33ab6;
    dsi_set_cmdq(data_array, 2, 1);
    MDELAY(10);

    data_array[0] = 0x00022902;
    data_array[1] = 0x000022c0;
    dsi_set_cmdq(data_array, 2, 1);
    MDELAY(10);

    data_array[0] = 0x00232902;
    data_array[1] = 0x006084c1;
    data_array[2] = 0x82941ccf;
    data_array[3] = 0xf7bdef52;
    data_array[4] = 0xb1ae7358;
    data_array[5] = 0x4a7bdef7;
    data_array[6] = 0x78864281;
    data_array[7] = 0x00000000;
    data_array[8] = 0x06046200;
    data_array[9] = 0x0001002a;
    dsi_set_cmdq(data_array, 10, 1);
    MDELAY(10);

    data_array[0] = 0x00082902;
    data_array[1] = 0x80f731c2;
    data_array[2] = 0x00000808;
    dsi_set_cmdq(data_array, 3, 1);
    MDELAY(10);

    data_array[0] = 0x00172902;
    data_array[1] = 0x000070c4;
    data_array[2] = 0x00000311;
    data_array[3] = 0x03000000;
    data_array[4] = 0x11000000;
    data_array[5] = 0x00000003;
    data_array[6] = 0x00030000;
    dsi_set_cmdq(data_array, 7, 1);
    MDELAY(10);

    data_array[0] = 0x00292902;
    data_array[1] = 0x690373c6;
    data_array[2] = 0x0000632a;
    data_array[3] = 0x00000000;
    data_array[4] = 0x00000000;
    data_array[5] = 0x15080000;
    data_array[6] = 0x6903730a;
    data_array[7] = 0x0000632a;
    data_array[8] = 0x00000000;
    data_array[9] = 0x00000000;
    data_array[10] = 0x15080000;
    data_array[11] = 0x0000000a;
    dsi_set_cmdq(data_array, 12, 1);
    MDELAY(10);

    data_array[0] = 0x001f2902;
    data_array[1] = 0x100900c7;
    data_array[2] = 0x3d332618;
    data_array[3] = 0x4539314c;
    data_array[4] = 0x7f695f55;
    data_array[5] = 0x18100900;
    data_array[6] = 0x4c3d3326;
    data_array[7] = 0x55453931;
    data_array[8] = 0x007f695f;
    dsi_set_cmdq(data_array, 9, 1);
    MDELAY(10);

    data_array[0] = 0x00212902;
    data_array[1] = 0x838001ca;
    data_array[2] = 0xdcd2c8a5;
    data_array[3] = 0x802008dc;
    data_array[4] = 0x374a0aff;
    data_array[5] = 0x0cf855a0;
    data_array[6] = 0x3f10200c;
    data_array[7] = 0x1000003f;
    data_array[8] = 0x3f3f3f10;
    data_array[9] = 0x0000003f;
    dsi_set_cmdq(data_array, 10, 1);
    MDELAY(10);

    data_array[0] = 0x000a2902;
    data_array[1] = 0x07e0fecb;
    data_array[2] = 0x0000007f;
    data_array[3] = 0x0000c000;
    dsi_set_cmdq(data_array, 4, 1);
    MDELAY(10);

    data_array[0] = 0x00022902;
    data_array[1] = 0x000003cc;
    dsi_set_cmdq(data_array, 2, 1);
    MDELAY(10);

    data_array[0] = 0x000b2902;
    data_array[1] = 0xbb822ad0;
    data_array[2] = 0x194c1128;
    data_array[3] = 0x00000c19;
    dsi_set_cmdq(data_array, 4, 1);
    MDELAY(10);

    data_array[0] = 0x001a2902;
    data_array[1] = 0xbb331bd3;
    data_array[2] = 0x3333b3bb;
    data_array[3] = 0x00010033;
    data_array[4] = 0x08a088a0;
    data_array[5] = 0x3b333939;
    data_array[6] = 0x3d077237;
    data_array[7] = 0x000055bf;
    dsi_set_cmdq(data_array, 8, 1);
    MDELAY(10);

    data_array[0] = 0x00082902;
    data_array[1] = 0x000006d5;
    data_array[2] = 0x26012601;//39 34 2c
    dsi_set_cmdq(data_array, 3, 1);
    MDELAY(10);

    data_array[0] = 0x00351500;
    dsi_set_cmdq(data_array, 1, 1);
    MDELAY(10);
    /*data_array[0] = 0x00110500;
    dsi_set_cmdq(data_array, 1, 1);
    MDELAY(200);
    */
    data_array[0] = 0x00290500;
    dsi_set_cmdq(data_array, 1, 1);
    MDELAY(50);

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
    params->dsi.LANE_NUM                = LCM_FOUR_LANE;
    //The following defined the fomat for data coming from LCD engine.
    params->dsi.data_format.color_order     = LCM_COLOR_ORDER_RGB;
    params->dsi.data_format.trans_seq       = LCM_DSI_TRANS_SEQ_MSB_FIRST;
    params->dsi.data_format.padding         = LCM_DSI_PADDING_ON_LSB;
    params->dsi.data_format.format              = LCM_DSI_FORMAT_RGB888;

    // Highly depends on LCD driver capability.
    params->dsi.packet_size=256;
    //video mode timing

    params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;
    /*
    params->dsi.vertical_sync_active                = 1;      //2
    params->dsi.vertical_backporch                  = 4;      //8 
    params->dsi.vertical_frontporch                 = 5;     //10
    params->dsi.vertical_active_line                    = FRAME_HEIGHT;

    params->dsi.horizontal_sync_active              = 5;     //10
    params->dsi.horizontal_backporch                = 40;     //20
    params->dsi.horizontal_frontporch               = 96;     //40
    params->dsi.horizontal_active_pixel             = FRAME_WIDTH;
        //params->dsi.ssc_disable                           = 1;
        */
    params->dsi.vertical_sync_active                = 2;      //2
    params->dsi.vertical_backporch                  = 8;      //8 
    params->dsi.vertical_frontporch                 = 8;     //10
    params->dsi.vertical_active_line                    = FRAME_HEIGHT;

    params->dsi.horizontal_sync_active              = 8;     //10
    params->dsi.horizontal_backporch                = 40;     //20
    params->dsi.horizontal_frontporch               = 80;     //40
    params->dsi.horizontal_active_pixel             = FRAME_WIDTH;

#if (LCM_DSI_CMD_MODE)
    params->dsi.PLL_CLOCK = 500; //this value must be in MTK suggested table
#else
    params->dsi.PLL_CLOCK = 450; //this value must be in MTK suggested table
#endif

    params->dsi.clk_lp_per_line_enable = 0;
    params->dsi.esd_check_enable = 1;
    params->dsi.customization_esd_check_enable = 0;
    params->dsi.lcm_esd_check_table[0].cmd = 0x0a;
    params->dsi.lcm_esd_check_table[0].count = 1;
    params->dsi.lcm_esd_check_table[0].para_list[0] = 0x1C;
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

    return (LCM_ID_RM63417 == id) ? 1 : 0;
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

    read_reg_v2(0x0a, buffer, 1);
    if (buffer[0]==0x1C) {
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

LCM_DRIVER r63417_fhd_dsi_vdo_chuanma_ly_lcm_drv = 
{
    .name           = "r63417_fhd_dsi_vdo_chuanma_ly",
    .set_util_funcs = lcm_set_util_funcs,
    .get_params     = lcm_get_params,
    .init           = lcm_init,
    .suspend        = lcm_suspend,
    .resume         = lcm_resume,
    .compare_id     = lcm_compare_id,
    .ata_check      = lcm_ata_check,
    .init_power     = lcm_init_power,
    .resume_power   = lcm_resume_power,
    .suspend_power  = lcm_suspend_power,
    //.esd_check      = lcm_esd_check,
    //.esd_recover    = lcm_esd_recover,
#if (LCM_DSI_CMD_MODE)
    .update         = lcm_update,
#endif
};
