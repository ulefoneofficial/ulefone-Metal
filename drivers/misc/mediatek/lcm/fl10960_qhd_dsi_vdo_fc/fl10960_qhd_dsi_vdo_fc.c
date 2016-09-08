#ifndef BUILD_LK
#include <linux/string.h>
#include <linux/wait.h>
#include <mt-plat/upmu_common.h>
#endif
#include "lcm_drv.h"

// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------

#define FRAME_WIDTH                                         (540)
#define FRAME_HEIGHT                                        (960)

#define REGFLAG_DELAY                                       0XFE
#define REGFLAG_END_OF_TABLE                                0xFF   // END OF REGISTERS MARKER

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


static struct LCM_setting_table lcm_initialization_setting[] = {
    
    /*
    Note :

    Data ID will depends on the following rule.
    
    count of parameters > 1 => Data ID = 0x39
    count of parameters = 1 => Data ID = 0x15
    count of parameters = 0 => Data ID = 0x05

    Structure Format :

    {DCS command, count of parameters, {parameter list}}
    {REGFLAG_DELAY, milliseconds of time, {}},

    ...

    Setting ending by predefined flag
    
    {REGFLAG_END_OF_TABLE, 0x00, {}}
    */
    {0xB9,3,{0xF1,0x09,0x60}},
    {0xBA,27,{0x11,0x81,0x06,0xF9,0x0A,0x0A,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x44,0x25,0x00,0x91,0x0A,0x00,0x00,0x02,0x4F,0x11,0x00,0x00,0x37}},
    {0xB1,7,{0x21,0x53,0xE3,0x1A,0x1A,0x33,0x77}},
    {0xB3,8,{0x02,0x00,0x06,0x06,0x28,0x28,0x28,0x28}},
    {0xB4,1,{0x20}},
    {0xB5,2,{0x05,0x05}},
    {0xB6,2,{0xA0,0xA0}},
    {0xB8,2,{0x64,0x22}},
    {0xE3,10,{0x03,0x03,0x03,0x03,0x03,0x03,0x03,0x03,0x00,0x14}},
    {0xCC,1,{0x02}},
    {0xBC,1,{0x44}},
    {0xBF,2,{0x02,0x10}},

    {0xE9,63,{0x84,0x10,0x02,0x03,0xBF,0x35,0x6A,0x12,0x31,0x23,0x18,0x00,0x35,0x6A,0x13,0x3C,0x00,0x00,0x84,0x00,0x00,0x00,0x00,0x00,0x84,0x00,0x00,0x00,0x88,0x13,0x13,0x15,0x88,
     0x88,0x88,0x88,0x88,0x88,0x88,0x88,0x02,0x02,0x04,0x88,0x88,0x88,0x88,0x88,0x88,0x88,0x00,0x00,0x00,0x00,0x83,0x6A,0x35,0x33,0x00,0x00,0x18,0x00,0x00}},

    {0xEA,48,{0x02,0x1A,0x00,0x00,0x88,0x42,0x00,0x20,0x88,0x88,0x88,0x88,0x88,0x88,0x88,0x88,0x53,0x11,0x31,0x88,0x88,0x88,0x88,0x88,0x88,0x88,0x00,0x0F,0x00,0xFF,0x00,0x04,0x00,
     0x00,0x3C,0x00,0x00,0x00,0x18,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},

    {0xE0,34,{0x00,0x00,0x00,0x31,0x37,0x3F,0x22,0x28,0x03,0x07,0x0B,0x0E,0x0E,0x0E,0x11,0x11,0x18,0x00,0x00,0x00,0x31,0x37,0x3F,0x22,0x28,0x03,0x07,0x0B,0x0E,0x0E,0x0E,0x11,0x11,0x18}},

    {0x11, 0,{0x00}},
    {REGFLAG_DELAY, 200, {}},
    {0x29, 0,{0x00}},
    {REGFLAG_DELAY, 20, {}},

};


/*
static struct LCM_setting_table lcm_sleep_out_setting[] = {
    // Sleep Out
    {0x11, 0, {0x00}},
    {REGFLAG_DELAY, 120, {}},

    // Display ON
    {0x29, 0, {0x00}},
    {REGFLAG_DELAY, 100, {}},
    {REGFLAG_END_OF_TABLE, 0x00, {}}
};
*/

static struct LCM_setting_table lcm_deep_sleep_mode_in_setting[] = {
    // Display off sequence
    {0x28, 0, {0x00}},
    {REGFLAG_DELAY, 120, {}},
    // Sleep Mode On
    {0x10, 0, {0x00}},
    {REGFLAG_DELAY, 100, {}},
    {REGFLAG_END_OF_TABLE, 0x00, {}}
};

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
                //UDELAY(5);//soso add or it will fail to send register
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

    //add by kevin for actual size of w & h 20160411 start
    params->physical_width  = 68;
    params->physical_height = 120;
    //add by kevin for actual size of w & h 20160411 end
    params->dsi.mode   = SYNC_EVENT_VDO_MODE;

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

    params->dsi.vertical_sync_active                = 4;        //2
    params->dsi.vertical_backporch                  = 16;        //4
    params->dsi.vertical_frontporch                 = 20;        //4
    params->dsi.vertical_active_line                = FRAME_HEIGHT; 

    params->dsi.horizontal_sync_active              = 8;    //8
    params->dsi.horizontal_backporch                = 54;   //64
    params->dsi.horizontal_frontporch               = 54;   //64
    params->dsi.horizontal_active_pixel             = FRAME_WIDTH;

    // Bit rate calculation
    //params->dsi.pll_div1=37;      // fref=26MHz, fvco=fref*(div1+1)   (div1=0~63, fvco=500MHZ~1GHz)
    //params->dsi.pll_div2=1;       // div2=0~15: fout=fvo/(2*div2)
    params->dsi.PLL_CLOCK               = 240;
    params->dsi.clk_lp_per_line_enable  = 0;
    params->dsi.esd_check_enable        = 0;// 1;
    params->dsi.customization_esd_check_enable      = 0;
    params->dsi.lcm_esd_check_table[0].cmd          = 0;//0x53;
    params->dsi.lcm_esd_check_table[0].count        = 0;// 1;
    params->dsi.lcm_esd_check_table[0].para_list[0] = 0x24;
}

static void lcd_power_en(unsigned char enabled)
{
    if (enabled) {
    #ifdef BUILD_LK
    #else
        pmic_config_interface(MT6328_PMIC_RG_VGP1_VOSEL_ADDR, 0x6, MT6328_PMIC_RG_VGP1_VOSEL_MASK, MT6328_PMIC_RG_VGP1_VOSEL_SHIFT);
        pmic_config_interface(MT6328_PMIC_RG_VGP1_EN_ADDR, 0x1, MT6328_PMIC_RG_VGP1_EN_MASK, MT6328_PMIC_RG_VGP1_EN_SHIFT);

        //MDELAY(50);
        //pmic_config_interface(MT6328_PMIC_RG_VGP3_VOSEL_ADDR, 0x3, MT6328_PMIC_RG_VGP3_VOSEL_MASK, MT6328_PMIC_RG_VGP3_VOSEL_SHIFT);
        //pmic_config_interface(MT6328_PMIC_RG_VGP3_EN_ADDR, 0x1, MT6328_PMIC_RG_VGP3_EN_MASK, MT6328_PMIC_RG_VGP3_EN_SHIFT);
    #endif
    } else {
    #ifdef BUILD_LK
    #else
        pmic_config_interface(MT6328_PMIC_RG_VGP1_VOSEL_ADDR, 0x0, MT6328_PMIC_RG_VGP1_VOSEL_MASK, MT6328_PMIC_RG_VGP1_VOSEL_SHIFT);
        pmic_config_interface(MT6328_PMIC_RG_VGP1_EN_ADDR, 0x0, MT6328_PMIC_RG_VGP1_EN_MASK, MT6328_PMIC_RG_VGP1_EN_SHIFT);

        //MDELAY(50);
        //pmic_config_interface(MT6328_PMIC_RG_VGP3_VOSEL_ADDR, 0x0, MT6328_PMIC_RG_VGP3_VOSEL_MASK, MT6328_PMIC_RG_VGP3_VOSEL_SHIFT);
        //pmic_config_interface(MT6328_PMIC_RG_VGP3_EN_ADDR, 0x0, MT6328_PMIC_RG_VGP3_EN_MASK, MT6328_PMIC_RG_VGP3_EN_SHIFT);
    #endif
    }
}

static void lcm_init(void)
{
    lcd_power_en(1);
    MDELAY(50);
    SET_RESET_PIN(1);
    MDELAY(10);
    SET_RESET_PIN(0);
    MDELAY(10);//Must > 5ms
    SET_RESET_PIN(1);
    MDELAY(120);//Must > 50ms

    push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
}


static void lcm_suspend(void)
{
    push_table(lcm_deep_sleep_mode_in_setting, sizeof(lcm_deep_sleep_mode_in_setting) / sizeof(struct LCM_setting_table), 1);
    SET_RESET_PIN(0);
    MDELAY(10);
}


static void lcm_resume(void)
{
    lcm_init();
//  push_table(lcm_sleep_out_setting, sizeof(lcm_sleep_out_setting) / sizeof(struct LCM_setting_table), 1);
}

static unsigned int lcm_compare_id(void)
{
    unsigned int id = 0;
    unsigned char buffer[3];
    unsigned int array[16];

    SET_RESET_PIN(1);//NOTE:should reset LCM firstly
    SET_RESET_PIN(0);
    MDELAY(6);
    SET_RESET_PIN(1);
    MDELAY(50);

    array[0] = 0x00033700;// read id return two byte,version and id
    dsi_set_cmdq(array, 1, 1);
    read_reg_v2(0x04, buffer, 3);
    id = buffer[0] | buffer[1] << 8; //we only need ID

#if defined(BUILD_UBOOT)
    printf("\n\n\n\n[soso]%s, id1 = 0x%08x\n", __func__, id);
#endif
    return (id == 0x9006)?1:0;
}

static unsigned int lcm_ata_check(unsigned char *buffer)
{
    extern int lcm_ata_check_flag;

    return (lcm_ata_check_flag > 0) ? 1 : 0;
}

LCM_DRIVER fl10960_dsi_fc_lcm_drv = 
{
    .name           = "fl10960_dsi_fc_lcm_drv",
    .set_util_funcs = lcm_set_util_funcs,
    .get_params     = lcm_get_params,
    .init           = lcm_init,
    .suspend        = lcm_suspend,
    .resume         = lcm_resume,
    .compare_id     = lcm_compare_id,
    .ata_check      = lcm_ata_check,
};

