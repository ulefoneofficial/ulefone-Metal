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

#define FRAME_WIDTH                                         (720)
#define FRAME_HEIGHT                                        (1280)

#define REGFLAG_DELAY                                       0xFFE
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


static struct LCM_setting_table lcm_initialization_setting[] = {
    //****************************** Page 1 Command******************************//
    {0x00,1,{0x00}},
    {0xff,3,{0x12,0x85,0x01}},

    {0x00,1,{0x80}},
    {0xff,2,{0x12,0x85}},

    {0x00,1,{0x91}},
    {0xb3,2,{0x08,0x10}},//  MIPI lane Setting ,08 is 3lane,0c is 4lane

    {0x00,1,{0x81}},
    {0xc0,4,{0x83,0x00,0x2c,0x2c}},

    {0x00,1,{0x80}},
    {0xc1,4,{0x15,0x15,0x15,0x15}},

    {0x00,1,{0x90}},
    {0xc1,1,{0x55}},

    {0x00,1,{0xB3}},
    {0xC0,2,{0x00,0x48}},

    {0x00,1,{0xc3}},
    {0xCb,2,{0x05,0x05}},///cbc3,cbc4

    {0x00,1,{0x00}},
    {0xD8,2,{0x26,0x26}},

    {0x00,1,{0x00}},
    {0xD9,1,{0x76}},

    {0x00,1,{0x80}},
    {0xc2,4,{0x82,0x01,0x40,0xc4}},//change

    {0x00,1,{0xf0}},
    {0xc2,2,{0x00,0x00}},

    {0x00,1,{0xf8}},
    {0xc2,1,{0x40}},

    {0x00,1,{0x90}},
    {0xc2,15,{0x2d,0x2d,0x00,0x00,0x8A,0x2c,0x2d,0x00,0x00,0x8A,0x00,0x00,0x01,0x06,0x86}},

    {0x00,1,{0xa0}},
    {0xc2,5,{0x01,0x00,0x01,0x06,0x86}},

    {0x00,1,{0xec}},
    {0xc2,2,{0x10,0x00}},

    {0x00,1,{0xf2}},
    {0xc2,2,{0x00,0x00}},

    {0x00,1,{0xc0}},
    {0xc2,5,{0x00,0x00,0x00,0x00,0x00}},

    {0x00,1,{0xa0}},
    {0xc0,4,{0x02,0x00,0x0B,0x02}},

    {0x00,1,{0xa5}},
    {0xc0,2,{0x1c,0x07}},

    {0x00,1,{0x80}},
    {0xa5,1,{0x0c}},

    {0x00,1,{0xea}},
    {0xc2,2,{0x11,0x00}},

    {0x00,1,{0xfb}},
    {0xc2,1,{0x0a}},

    {0x00,1,{0x80}},
    {0xcc,12,{0x01,0x15,0x15,0x03,0x04,0x0b,0x0b,0x0b,0x0b,0x0b,0x0b,0x0b}},

    {0x00,1,{0xb0}},
    {0xcc,12,{0x01,0x15,0x15,0x04,0x03,0x0b,0x0b,0x0b,0x0b,0x0b,0x0b,0x0b}},

    {0x00,1,{0x80}},
    {0xcd,15,{0x0b,0x0b,0x0b,0x0b,0x0b,0x0b,0x0b,0x0b,0x0b,0x0b,0x0b,0x0b,0x12,0x13,0x14}},

    {0x00,1,{0x80}},
    {0xcb,7,{0x00,0x00,0x00,0x00,0x00,0x00,0x00}},

    {0x00,1,{0xf0}},
    {0xcb,7,{0x24,0xf3,0xf3,0x00,0x00,0xff,0xff}},

    {0x00,1,{0xc8}},
    {0xf5,1,{0x2e}},

    {0x00,1,{0xc0}},
    {0xcb,15,{0x05,0x05,0x0a,0x0a,0x0a,0xFF,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},//change

    {0x00,1,{0xd0}},
    {0xcb,12,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x05,0x05,0x05}},

    {0x00,1,{0x90}},
    {0xcb,15,{0x00,0x00,0x00,0x00,0x00,0xFF,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},

    {0x00,1,{0xa0}},
    {0xcb,12,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},

    {0x00,1,{0xe0}},
    {0xcc,4,{0x80,0x0f,0xf0,0x00}},

    {0x00,1,{0xd0}},
    {0xcd,15,{0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x0a,0x0b,0x0c,0x0d,0x0e,0x0f}},

    {0x00,1,{0xe0}},
    {0xcd,12,{0x10,0x11,0x12,0x13,0x14,0x15,0x16,0x17,0x18,0x19,0x1a,0x1b}},

    {0x00,1,{0xa0}},
    {0xcd,15,{0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x0a,0x0b,0x0c,0x0d,0x0e,0x0f}},

    {0x00,1,{0xb0}},
    {0xcd,12,{0x10,0x11,0x12,0x13,0x14,0x15,0x16,0x17,0x18,0x19,0x1a,0x1b}},

    {0x00,1,{0x00}},
    {0xE1,24,{0x1A,0x1E,0x23,0x2b,0x32,0x38,0x42,0x53,0x5c,0x6f,0x7b,0x83,0x76,0x72,0x6d,0x5e,0x4d,0x3b,0x31,0x28,0x04,0x04,0x05,0x05}},

    {0x00,1,{0x00}},
    {0xE2,24,{0x1A,0x1E,0x23,0x2b,0x32,0x38,0x42,0x53,0x5c,0x6f,0x7b,0x83,0x76,0x72,0x6d,0x5e,0x4d,0x3b,0x31,0x28,0x04,0x04,0x05,0x05}},

    {0x00,1,{0x00}},
    {0xE3,24,{0x1A,0x1E,0x23,0x2b,0x32,0x38,0x42,0x53,0x5c,0x6f,0x7b,0x83,0x76,0x72,0x6d,0x5e,0x4d,0x3b,0x31,0x28,0x04,0x04,0x05,0x05}},

    {0x00,1,{0x00}},
    {0xE4,24,{0x1A,0x1E,0x23,0x2b,0x32,0x38,0x42,0x53,0x5c,0x6f,0x7b,0x83,0x76,0x72,0x6d,0x5e,0x4d,0x3b,0x31,0x28,0x04,0x04,0x05,0x05}},

    {0x00,1,{0x00}},
    {0xE5,24,{0x1A,0x1E,0x23,0x2b,0x32,0x38,0x42,0x53,0x5c,0x6f,0x7b,0x83,0x76,0x72,0x6d,0x5e,0x4d,0x3b,0x31,0x28,0x04,0x04,0x05,0x05}},

    {0x00,1,{0x00}},
    {0xE6,24,{0x1A,0x1E,0x23,0x2b,0x32,0x38,0x42,0x53,0x5c,0x6f,0x7b,0x83,0x76,0x72,0x6d,0x5e,0x4d,0x3b,0x31,0x28,0x04,0x04,0x05,0x05}},

    {0x00,1,{0x83}},
    {0xC4,1,{0x02}},

    {0x00,1,{0x80}},
    {0xC4,1,{0x04}},

    {0x00,1,{0xF8}},
    {0xC2,1,{0x40}},

    {0x00,1,{0x80}},
    {0xA5,1,{0x0C}},

    {0x00,1,{0x81}},
    {0xA5,1,{0x04}},

    {0x00,1,{0x84}},
    {0xC1,2,{0xE0,0xE8}},

    {0x00,1,{0x9A}},
    {0xC5,3,{0x22,0x22,0x20}},

    {0x00,1,{0xC8}},
    {0xF5,1,{0x2E}},

    {0x00,1,{0xCD}},
    {0xF5,1,{0x25}},

    {0x00,1,{0x96}},
    {0xC5,1,{0x80}},

    {0x00,1,{0x98}},
    {0xC5,1,{0x80}},

    {0x00,1,{0xE1}},
    {0xCC,1,{0x0F}},

    {0x00,1,{0xE2}},
    {0xCC,1,{0xF0}},

    {0x00,1,{0x80}},
    {0xFF,3,{0xFF,0xFF,0xFF}},

    {0x11,1,{0x00}},//SLEEP OUT
    {REGFLAG_DELAY,120,{}},

    {0x29,1,{0x00}},//Display ON
    {REGFLAG_DELAY,20,{}},

    // Setting ending by predefined flag
    {REGFLAG_END_OF_TABLE, 0x00, {}}
};

#if 1
static struct LCM_setting_table lcm_sleep_out_setting[] = {
    // Sleep Out
    {0x11, 1, {0x00}},
    {REGFLAG_DELAY, 120, {}},
    // Display ON
    {0x29, 1, {0x00}},
    {REGFLAG_DELAY, 20, {}},

    {REGFLAG_END_OF_TABLE, 0x00, {}}
};
#endif

static struct LCM_setting_table lcm_deep_sleep_mode_in_setting[] = {
    // Display off sequence
    {0x28, 1, {0x00}},
    {REGFLAG_DELAY, 40, {}},
    // Sleep Mode On
    {0x10, 1, {0x00}},
    {REGFLAG_DELAY, 120, {}},

    {REGFLAG_END_OF_TABLE, 0x00, {}}
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

    params->physical_width = 65;
    params->physical_height = 109;

    params->dsi.mode   = SYNC_PULSE_VDO_MODE;

    // DSI
    /* Command mode setting */
    params->dsi.LANE_NUM                = LCM_THREE_LANE;
    //The following defined the fomat for data coming from LCD engine.
    params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
    params->dsi.data_format.trans_seq   = LCM_DSI_TRANS_SEQ_MSB_FIRST;
    params->dsi.data_format.padding     = LCM_DSI_PADDING_ON_LSB;
    params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;

    // Highly depends on LCD driver capability.
    // Not support in MT6573
    params->dsi.packet_size=256;

    // Video mode setting
    params->dsi.intermediat_buffer_num = 0;

    params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;

    params->dsi.vertical_sync_active                = 4;        //2
    params->dsi.vertical_backporch                  = 6;        //4
    params->dsi.vertical_frontporch                 = 6;        //4
    params->dsi.vertical_active_line                = FRAME_HEIGHT;

    params->dsi.horizontal_sync_active              = 10;    //8
    params->dsi.horizontal_backporch                = 10;   //64
    params->dsi.horizontal_frontporch               = 10;   //64
    params->dsi.horizontal_active_pixel             = FRAME_WIDTH;

    params->dsi.PLL_CLOCK                           = 160;

    params->dsi.clk_lp_per_line_enable              = 0;
    params->dsi.esd_check_enable                    = 1;// 1;
    params->dsi.customization_esd_check_enable      = 1;
    params->dsi.lcm_esd_check_table[0].cmd          = 0xa;//0x53;
    params->dsi.lcm_esd_check_table[0].count        = 0;// 1;
    params->dsi.lcm_esd_check_table[0].para_list[0] = 0x9c;
}

static void lcd_power_en(unsigned char enabled)
{
    if (enabled) {
#ifdef BUILD_LK
#else
        pmic_config_interface(MT6328_PMIC_RG_VGP1_VOSEL_ADDR, 0x6, MT6328_PMIC_RG_VGP1_VOSEL_MASK, MT6328_PMIC_RG_VGP1_VOSEL_SHIFT);
        pmic_config_interface(MT6328_PMIC_RG_VGP1_EN_ADDR, 0x1, MT6328_PMIC_RG_VGP1_EN_MASK, MT6328_PMIC_RG_VGP1_EN_SHIFT);
        MDELAY(50);
#endif
    } else {
#ifdef BUILD_LK
#else
        pmic_config_interface(MT6328_PMIC_RG_VGP1_VOSEL_ADDR, 0x0, MT6328_PMIC_RG_VGP1_VOSEL_MASK, MT6328_PMIC_RG_VGP1_VOSEL_SHIFT);
        pmic_config_interface(MT6328_PMIC_RG_VGP1_EN_ADDR, 0x0, MT6328_PMIC_RG_VGP1_EN_MASK, MT6328_PMIC_RG_VGP1_EN_SHIFT);
        MDELAY(50);
#endif
    }
}

static void lcm_init(void)
{
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
}

static void lcm_resume(void)
{
    //lcm_init();
    printk("otm1285a lcm_resume\n");
    push_table(lcm_sleep_out_setting, sizeof(lcm_sleep_out_setting) / sizeof(struct LCM_setting_table), 1);
}

static unsigned int lcm_compare_id(void)
{
    int   array[4];
    char  buffer[10];
    unsigned int id=0;
    char  id0=0;
    char  id1=0;
    char  id2=0;

    //NOTE:should reset LCM firstly
    SET_RESET_PIN(1);
    MDELAY(10);
    SET_RESET_PIN(0);
    MDELAY(10);
    SET_RESET_PIN(1);
    MDELAY(120);

    array[0] = 0x00033700;
    dsi_set_cmdq(array, 1, 1);
    read_reg_v2(0xDA,buffer, 1);

    array[0] = 0x00033700;
    dsi_set_cmdq(array, 1, 1);
    read_reg_v2(0xDB,buffer+1, 1);

    array[0] = 0x00033700;
    dsi_set_cmdq(array, 1, 1);
    read_reg_v2(0xDC,buffer+2, 1);

    id0 = buffer[0];  //should be 0x10
    id1 = buffer[1];  //should be 0x80
    id2 = buffer[2];  //should be 0x1a

    printk("%s, id0 = 0x%x\n", __func__, id0);//should be 0x10
    printk("%s, id1 = 0x%x\n", __func__, id1);//should be 0x80
    printk("%s, id2 = 0x%x\n", __func__, id2);//should be 0x1a


    //id  = (unsigned int)((id0 << 8) | id1);
    id=id0;
    printk("%s, id=0x%x\n", __func__,id);//should be 0x1080

    return (id == 0x40)?1:0;
}

static unsigned int lcm_ata_check(unsigned char *buffer)
{
    extern int lcm_ata_check_flag;

    return (lcm_ata_check_flag > 0) ? 1 : 0;
}

LCM_DRIVER otm1285a_hd_dsi_vdo_ykl_lcm_drv =
{
    .name           = "otm1285a_hd_dsi_vdo_ykl",
    .set_util_funcs = lcm_set_util_funcs,
    .get_params     = lcm_get_params,
    .init           = lcm_init,
    .suspend        = lcm_suspend,
    .resume         = lcm_resume,
    .compare_id     = lcm_compare_id,
    .ata_check      = lcm_ata_check,
};

