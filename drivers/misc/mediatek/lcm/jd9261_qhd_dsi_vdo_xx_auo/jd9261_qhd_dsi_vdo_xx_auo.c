#ifndef BUILD_LK
#include <linux/string.h>
#include <linux/wait.h>
#include <mt-plat/upmu_common.h>
#endif
#include "lcm_drv.h"

#include <mach/gpio_const.h>
#include <mt-plat/mt_gpio.h>
// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------

#define FRAME_WIDTH                                         (540)
#define FRAME_HEIGHT                                        (960)

#define REGFLAG_DELAY                                       0XFFE
#define REGFLAG_END_OF_TABLE                                0xFFF   // END OF REGISTERS MARKER

#define LCM_DSI_CMD_MODE                                    0

// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util = {0};

#define SET_RESET_PIN(v)                                    (lcm_util.set_reset_pin((v)))

#define UDELAY(n)                                           (lcm_util.udelay(n))
#define MDELAY(n)                                           (lcm_util.mdelay(n))


// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)    lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)       lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)                                      lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)                  lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg                                            lcm_util.dsi_read_reg()
#define read_reg_v2(cmd, buffer, buffer_size)               lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)

struct LCM_setting_table {
    unsigned cmd;
    unsigned char count;
    unsigned char para_list[64];
};

// zhangchongyong  add for lcm detect ,read adc voltage
extern int IMM_GetOneChannelValue(int dwChannel, int data[4], int* rawdata);

static struct LCM_setting_table lcm_initialization_setting[] = {
    //JD9261+AUO5.5 IPS JIN PING 20160527
    {0xBF,3,{0x92,0x61,0xF3}},
    {REGFLAG_DELAY,1,{}},
    {0xB3,2,{0x00,0x97}},
    {REGFLAG_DELAY,1,{}},
    {0xB4,2,{0x00,0x9F}},
    {REGFLAG_DELAY,1,{}},
    {0xB8,6,{0x00,0xDF,0x01,0x00,0xDF,0x01}},
    {REGFLAG_DELAY,1,{}},
    {0xC1,1,{0x10}},
    {REGFLAG_DELAY,1,{}},
    {0xC3,1,{0x05}},
    {REGFLAG_DELAY,1,{}},
    {0xC4,2,{0x00,0x78}},
    {REGFLAG_DELAY,1,{}},
    {0xC7,9,{0x00,0x01,0x32,0x0D,0x6C,0x24,0x11,0xA5,0xA5}},
    {REGFLAG_DELAY,1,{}},
    {0xC8,38,{0x7F,0x6A,0x5C,0x4F,0x4B,0x3B,0x3E,0x25,0x3A,0x36,0x32,0x4D,0x38,0x3E,0x2D,0x29,0x1C,0x0B,0x02,0x7F,0x6A,0x5C,0x4F,0x4B,0x3B,0x3E,0x25,0x3A,0x36,0x32,0x4D,0x38,0x3E,0x2D,0x29,0x1C,0x0B,0x02}},
    {REGFLAG_DELAY,1,{}},
    {0xD4,19,{0x1F,0x1F,0x01,0x0D,0x0F,0x07,0x05,0x11,0x1F,0x1F,0x1F,0x1F,0x1F,0x1F,0x1F,0x1F,0x1F,0x1F,0x1F}},
    {REGFLAG_DELAY,1,{}},
    {0xD5,19,{0x1F,0x1F,0x00,0x0C,0x0E,0x06,0x04,0x10,0x1F,0x1F,0x1F,0x1F,0x1F,0x1F,0x1F,0x1F,0x1F,0x1F,0x1F}},
    {REGFLAG_DELAY,1,{}},
    {0xD6,19,{0x1F,0x1F,0x10,0x0C,0x0E,0x04,0x06,0x00,0x1F,0x0F,0x1F,0x1F,0x1F,0x1F,0x1F,0x1F,0x1F,0x1F,0x1F}},
    {REGFLAG_DELAY,1,{}},
    {0xD7,19,{0x1F,0x1F,0x11,0x0D,0x0F,0x05,0x07,0x01,0x1F,0x1D,0x1F,0x1F,0x1F,0x1F,0x1F,0x1F,0x1F,0x1F,0x1F}},
    {REGFLAG_DELAY,1,{}},
    {0xD8,20,{0x60,0x00,0x00,0x10,0x06,0x10,0x00,0x00,0x10,0x00,0x00,0x05,0x73,0x53,0xC8,0x31,0x04,0x05,0x73,0x00}},
    {REGFLAG_DELAY,1,{}},
    {0xD9,21,{0x00,0x06,0x06,0x8F,0x33,0x04,0x72,0x02,0x00,0xBC,0x00,0x3B,0xCB,0x1F,0x00,0x00,0x00,0x06,0x70,0x01,0xE0}},
    {REGFLAG_DELAY,1,{}},
    {0xBE,1,{0x01}},
    {REGFLAG_DELAY,1,{}},
    {0xC1,1,{0x10}},
    {REGFLAG_DELAY,1,{}},
    {0xCC,10,{0x34,0x20,0x38,0x60,0x11,0x91,0x00,0x50,0x00,0x00}},
    {REGFLAG_DELAY,1,{}},
    {0xBE,1,{0x00}},
    {REGFLAG_DELAY,1,{}},
    {0xCF,2,{0x02,0x05}},
    {REGFLAG_DELAY,1,{}},
    {0x35,1,{0x00}}, 
    {REGFLAG_DELAY,1,{}},
    {0x11,1,{0x00}},                 // Sleep-Out
    {REGFLAG_DELAY,120,{}},
    {0x29,1,{0x00}},                 // Display On
    {REGFLAG_DELAY,20,{}},

    {REGFLAG_END_OF_TABLE,0x00,{}}
};

/*
static void init_lcm_registers(void){
    
    //JD9261+AUO5.5 IPS JIN PING 20160527
    unsigned int data_array[16];

    data_array[0]=0x00043902;
    data_array[1]=0xF36192BF;
    dsi_set_cmdq(data_array,2, 1);

    data_array[0]=0x00033902;
    data_array[1]=0x009700B3;
    dsi_set_cmdq(data_array,2, 1);

    data_array[0]=0x00033902;
    data_array[1]=0x009F00B4;
    dsi_set_cmdq(data_array,2, 1);

    data_array[0]=0x00073902;
    data_array[1]=0x01DF00B8;
    data_array[2]=0x0001DF00;
    dsi_set_cmdq(data_array,3, 1);

    data_array[0]=0x10C11500;
    dsi_set_cmdq(data_array,1, 1);

    data_array[0]=0x05C31500;
    dsi_set_cmdq(data_array,1, 1);

    data_array[0]=0x00033902;
    data_array[1]=0x007800C4;
    dsi_set_cmdq(data_array,2, 1);

    data_array[0]=0x000A3902;
    data_array[1]=0x320100C7;
    data_array[2]=0x11246C0D;
    data_array[3]=0x0000A5A5;
    dsi_set_cmdq(data_array,4, 1);

    data_array[0]=0x00273902;
    data_array[1]=0x5C6A7FC8;
    data_array[2]=0x3E3B4B4F;
    data_array[3]=0x32363A25;
    data_array[4]=0x2D3E384D;
    data_array[5]=0x020B1C29;
    data_array[6]=0x4F5C6A7F;
    data_array[7]=0x253E3B4B;
    data_array[8]=0x4D32363A;
    data_array[9]=0x292D3E38;
    data_array[10]=0x00020B1C;
    dsi_set_cmdq(data_array,11, 1);
    
    data_array[0]=0x001143902;
    data_array[1]=0x011F1FD4;
    data_array[2]=0x05070F0D;
    data_array[3]=0x1F1F1F11;
    data_array[4]=0x1F1F1F1F;
    data_array[5]=0x1F1F1F1F;
    dsi_set_cmdq(data_array,6, 1);
    
    data_array[0]=0x001143902;
    data_array[1]=0x001F1FD5;
    data_array[2]=0x04060E0C;
    data_array[3]=0x1F1F1F10;
    data_array[4]=0x1F1F1F1F;
    data_array[5]=0x1F1F1F1F;
    dsi_set_cmdq(data_array,6, 1);
    
    data_array[0]=0x00153902;
    data_array[1]=0x000060D8;
    data_array[2]=0x00100610;
    data_array[3]=0x00001000;
    data_array[4]=0xC8537305;
    data_array[5]=0x73050431;
    data_array[6]=0x00000000;
    dsi_set_cmdq(data_array,7, 1);

    data_array[0]=0x00163902;
    data_array[1]=0x060600D9;
    data_array[2]=0x7204338F;
    data_array[3]=0x00BC0002;
    data_array[4]=0x001FCB3B;
    data_array[5]=0x70060000;
    data_array[6]=0x0000E001;
    dsi_set_cmdq(data_array,7, 1);

    data_array[0]=0x01BE1500;
    dsi_set_cmdq(data_array,1, 1);
    
    
    data_array[0]=0x10C11500;
    dsi_set_cmdq(data_array,1, 1);

    data_array[0]=0x000B3902;
    data_array[1]=0x382034CC;
    data_array[2]=0x00911160;
    data_array[3]=0x00000040;
    dsi_set_cmdq(data_array,4, 1);

    data_array[0]=0x00BE1500;
    dsi_set_cmdq(data_array,1, 1);


    data_array[0]=0x00033902;
    data_array[1]=0x000502CF;
    dsi_set_cmdq(data_array,2, 1);

    data_array[0]=0x00110500;
    dsi_set_cmdq(data_array,1, 1);
    MDELAY(120);

    data_array[0]=0x00290500;
    dsi_set_cmdq(data_array,1, 1);
    MDELAY(20);

};*/

/*
static struct LCM_setting_table lcm_sleep_out_setting[] = {
    // Sleep Out
    {0x11, 1, {0x00}},
    {REGFLAG_DELAY, 120, {}},

    // Display ON
    {0x29, 1, {0x00}},
    {REGFLAG_DELAY, 50, {}},

    {REGFLAG_END_OF_TABLE, 0x00, {}}
};


static struct LCM_setting_table lcm_deep_sleep_mode_in_setting[] = {
    // Display off sequence
    {0x28, 1, {0x00}},
    {REGFLAG_DELAY, 20, {}},
    // Sleep Mode On
    {0x10, 1, {0x00}},
    {REGFLAG_DELAY, 120, {}},
    {REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_backlight_level_setting[] = {
    {0x51, 1, {0xFF}},
    {REGFLAG_END_OF_TABLE, 0x00, {}}
};
*/

static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
    unsigned int i;

    for(i = 0; i < count; i++) {
        unsigned cmd;

        cmd = table[i].cmd;
        switch (cmd) {
            case REGFLAG_DELAY:
                MDELAY(table[i].count);
                break;
                
            case REGFLAG_END_OF_TABLE:
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

    params->type = LCM_TYPE_DSI;

    params->width = FRAME_WIDTH;
    params->height = FRAME_HEIGHT;

    //add by kevin for actual size of w & h 20160411 start
    params->physical_width  = 68;
    params->physical_height = 120;
    //add by kevin for actual size of w & h 20160411 end
    params->dsi.mode   = SYNC_PULSE_VDO_MODE;

    // DSI
    /* Command mode setting */
    params->dsi.LANE_NUM                = LCM_TWO_LANE;
    //The following defined the fomat for data coming from LCD engine.
    params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
    params->dsi.data_format.trans_seq   = LCM_DSI_TRANS_SEQ_MSB_FIRST;
    params->dsi.data_format.padding     = LCM_DSI_PADDING_ON_LSB;
    params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;

    // Highly depends on LCD driver capability.
    // Not support in MT6573
    params->dsi.packet_size=256;

    // Video mode setting
    params->dsi.intermediat_buffer_num = 2;

    params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;

    params->dsi.vertical_sync_active = 2;//2
    params->dsi.vertical_backporch = 7;//4
    params->dsi.vertical_frontporch = 9;//4
    params->dsi.vertical_active_line = FRAME_HEIGHT;

    params->dsi.horizontal_sync_active = 10;//8
    params->dsi.horizontal_backporch = 50;//64
    params->dsi.horizontal_frontporch = 50;//64
    params->dsi.horizontal_active_pixel = FRAME_WIDTH;

    // Bit rate calculation
    //params->dsi.pll_div1 =37;// fref=26MHz, fvco=fref*(div1+1)   (div1=0~63, fvco=500MHZ~1GHz)
    //params->dsi.pll_div2 =1;// div2=0~15: fout=fvo/(2*div2)
    params->dsi.PLL_CLOCK = 220;
    //esd CHECK
    params->dsi.esd_check_enable = 1; 
    params->dsi.customization_esd_check_enable = 1;
    params->dsi.clk_lp_per_line_enable =1;
    params->dsi.lcm_esd_check_table[0].cmd = 0x0a;
    params->dsi.lcm_esd_check_table[0].count = 1;
    params->dsi.lcm_esd_check_table[0].para_list[0] = 0x9C;
    params->dsi.noncont_clock = 1; 
    params->dsi.noncont_clock_period = 2; 

    // Bit rate calculation
    //params->dsi.PLL_CLOCK =270;//240
    //params->dsi.ssc_disable = 1;
}

static void lcd_power_en(unsigned char enabled)
{
    if (enabled) {
    #ifdef BUILD_LK
    #else
        //ly_xinchao add start
        pmic_config_interface(MT6328_PMIC_RG_VGP1_VOSEL_ADDR, 0x6, MT6328_PMIC_RG_VGP1_VOSEL_MASK, MT6328_PMIC_RG_VGP1_VOSEL_SHIFT);
        pmic_config_interface(MT6328_PMIC_RG_VGP1_EN_ADDR, 0x1, MT6328_PMIC_RG_VGP1_EN_MASK, MT6328_PMIC_RG_VGP1_EN_SHIFT);
    //#ifdef LYCONFIG_AUTO_PLATFORM_NAME_Z3
        //MDELAY(50);
        //pmic_config_interface(MT6328_PMIC_RG_VGP3_VOSEL_ADDR, 0x3, MT6328_PMIC_RG_VGP3_VOSEL_MASK, MT6328_PMIC_RG_VGP3_VOSEL_SHIFT);
        //pmic_config_interface(MT6328_PMIC_RG_VGP3_EN_ADDR, 0x1, MT6328_PMIC_RG_VGP3_EN_MASK, MT6328_PMIC_RG_VGP3_EN_SHIFT);
    //#endif
        //ly_xinchao add end
#endif
    } else {
    #ifdef BUILD_LK
    #else
        //ly_xinchao add start
        pmic_config_interface(MT6328_PMIC_RG_VGP1_VOSEL_ADDR, 0x0, MT6328_PMIC_RG_VGP1_VOSEL_MASK, MT6328_PMIC_RG_VGP1_VOSEL_SHIFT);
        pmic_config_interface(MT6328_PMIC_RG_VGP1_EN_ADDR, 0x0, MT6328_PMIC_RG_VGP1_EN_MASK, MT6328_PMIC_RG_VGP1_EN_SHIFT);
        
    //#ifdef LYCONFIG_AUTO_PLATFORM_NAME_Z3
        //MDELAY(50);
        //pmic_config_interface(MT6328_PMIC_RG_VGP3_VOSEL_ADDR, 0x0, MT6328_PMIC_RG_VGP3_VOSEL_MASK, MT6328_PMIC_RG_VGP3_VOSEL_SHIFT);
        //pmic_config_interface(MT6328_PMIC_RG_VGP3_EN_ADDR, 0x0, MT6328_PMIC_RG_VGP3_EN_MASK, MT6328_PMIC_RG_VGP3_EN_SHIFT);
    //#endif
        //ly_xinchao add end
    #endif
    }
}

static unsigned int lcm_compare_id(void);

static void lcm_init(void)
{
    lcd_power_en(1);
    MDELAY(20);
    SET_RESET_PIN(1);
    MDELAY(20);
    SET_RESET_PIN(0);
    MDELAY(120);
    SET_RESET_PIN(1);
    MDELAY(120);
    // init_lcm_registers();
    push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
}

static void lcm_suspend(void)
{
    SET_RESET_PIN(0);
    MDELAY(120);
    
    /*
    unsigned int data_array[16];

    data_array[0]=0x00280500; // Display Off
    dsi_set_cmdq(data_array, 1, 1);
    
    data_array[0] = 0x00100500; // Sleep In
    dsi_set_cmdq(data_array, 1, 1);
    */

    //push_table(lcm_deep_sleep_mode_in_setting, sizeof(lcm_deep_sleep_mode_in_setting) / sizeof(struct LCM_setting_table), 1);
}

static void lcm_resume(void)
{
    lcm_init();
    
    //push_table(lcm_sleep_out_setting, sizeof(lcm_sleep_out_setting) / sizeof(struct LCM_setting_table), 1);
}

static unsigned int lcm_compare_id(void)
{
    unsigned int id = 0;
    //unsigned int id_midd;
    //unsigned int id_low;
    unsigned char buffer[5];
    //unsigned char buffer1[5];
    unsigned int array[16];

    //return 1;
    SET_RESET_PIN(1);
    MDELAY(10);
    SET_RESET_PIN(0);
    MDELAY(50);
    SET_RESET_PIN(1);
    MDELAY(50);

    array[0] = 0x00043902;
    array[1] = 0xF36192BF;
    dsi_set_cmdq(array, 2, 1);

    array[0] = 0x00BE1500;
    dsi_set_cmdq(array, 1, 1);

    array[0] = 0x00023700;
    dsi_set_cmdq(array, 1, 1);

    read_reg_v2(0x04, buffer, 2);

    id = (buffer[0] << 8 | buffer[1]);//we only need ID
#if defined(BUILD_LK)
    printf(" [jd9261]id = [0x%x],buffer[0] = [0x%x],buffer[1] = [0x%x] \n", id, buffer[0], buffer[1]);
#endif
    return (0x9261 == id) ? 1 : 0;

    //return 1;
}

static unsigned int lcm_ata_check(unsigned char *buffer)
{
    extern int lcm_ata_check_flag;

    return (lcm_ata_check_flag > 0) ? 1 : 0;
}

LCM_DRIVER jd9216_dsi_vdo_hz_auo_lcm_drv =
{
    .name           = "jd9261_qhd_dsi_vdo_xx_auo",
    .set_util_funcs = lcm_set_util_funcs,
    .get_params     = lcm_get_params,
    .init           = lcm_init,
    .suspend        = lcm_suspend,
    .resume         = lcm_resume,
    .compare_id     = lcm_compare_id,
    .ata_check      = lcm_ata_check,
};

