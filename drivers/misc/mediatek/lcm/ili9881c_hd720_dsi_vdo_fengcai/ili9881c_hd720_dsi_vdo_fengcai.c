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
//  #include <mach/mt_gpio.h>
//  #include <mach/mt_pm_ldo.h>
//  #include <mach/upmu_common.h>

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

int esd_test = 0;

extern void synaptics_rmi4_irq_enable_lcd(bool enable);

//static unsigned int lcm_esd_test = FALSE;      ///only for ESD test
static LCM_UTIL_FUNCS lcm_util = {0};

extern int lcm_ata_check_flag;

#define SET_RESET_PIN(v)            (lcm_util.set_reset_pin((v)))

#define UDELAY(n)   (lcm_util.udelay(n))
#define MDELAY(n)   (lcm_util.mdelay(n))

#define REGFLAG_DELAY           0XFE
#define REGFLAG_END_OF_TABLE    0xFFF   // END OF REGISTERS MARKER

#define LCM_ID_ILI9881C         (0x9881)

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

#if 0
static void lcd_mipi_mode(unsigned char mode)
{
    if(mode == GPIO_MODE_00)
    {
        /*
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
    } else {
        /*
        mt_set_gpio_mode(LCM_TDP0_GPIO, GPIO_MODE_01);
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
        //mt_set_gpio_out(GPIO_LCM_PWR, GPIO_OUT_ZERO);
    }
}
#endif

struct LCM_setting_table {
    unsigned cmd;
    unsigned char count;
    unsigned char para_list[64];
};

static struct LCM_setting_table lcm_initialization_setting[] = {
    {0xFF,3,{0x98,0x81,0x08}},
    {0xA3,1,{0x40}},//3line
    //{0xA3,1,{0xC0}},//4line

    {0xFF,3,{0x98,0x81,0x07}},
    {0x03,1,{0x20}},
    {0x04,1,{0x06}},
    {0x05,1,{0x00}},
    {0x06,1,{0x01}},
    {0x07,1,{0x00}},
    {0x08,1,{0x00}},
    {0x09,1,{0x00}},
    {0x0a,1,{0x01}},
    {0x0b,1,{0x2F}},
    {0x0c,1,{0x00}},
    {0x0d,1,{0x00}},
    {0x0e,1,{0x00}},
    {0x0F,1,{0x00}},
    {0x10,1,{0x44}},
    {0x11,1,{0x02}},
    {0x12,1,{0x03}},
    {0x13,1,{0x00}},
    {0x14,1,{0x00}},
    {0x15,1,{0x00}},
    {0x16,1,{0x2F}},
    {0x17,1,{0x2F}},
    {0x18,1,{0x00}},
    {0x19,1,{0x00}},
    {0x1a,1,{0x00}},
    {0x1b,1,{0x50}},
    {0x1c,1,{0xBB}},
    {0x1d,1,{0x0C}},
    {0x1e,1,{0x00}},
    {0x1F,1,{0x00}},
    {0x20,1,{0x00}},
    {0x21,1,{0x00}},
    {0x22,1,{0x00}},
    {0x23,1,{0xC0}},
    {0x24,1,{0x30}},
    {0x25,1,{0x00}},
    {0x26,1,{0x00}},
    {0x27,1,{0x03}},

    {0x30,1,{0x01}},
    {0x31,1,{0x23}},
    {0x32,1,{0x45}},
    {0x33,1,{0x67}},
    {0x34,1,{0x89}},
    {0x35,1,{0xAB}},
    {0x36,1,{0x01}},
    {0x37,1,{0x23}},
    {0x38,1,{0x45}},
    {0x39,1,{0x67}},
    {0x3a,1,{0x89}},
    {0x3b,1,{0xAB}},
    {0x3c,1,{0xCD}},
    {0x3d,1,{0xEF}},

    {0x50,1,{0x11}},
    {0x51,1,{0x06}},
    {0x52,1,{0x0C}},
    {0x53,1,{0x0D}},
    {0x54,1,{0x0E}},
    {0x55,1,{0x0F}},
    {0x56,1,{0x02}},
    {0x57,1,{0x02}},
    {0x58,1,{0x02}},
    {0x59,1,{0x02}},
    {0x5a,1,{0x02}},
    {0x5b,1,{0x02}},
    {0x5c,1,{0x02}},
    {0x5d,1,{0x02}},
    {0x5e,1,{0x02}},
    {0x5F,1,{0x02}},
    {0x60,1,{0x05}},
    {0x61,1,{0x05}},
    {0x62,1,{0x05}},
    {0x63,1,{0x02}},
    {0x64,1,{0x01}},
    {0x65,1,{0x00}},
    {0x66,1,{0x08}},
    {0x67,1,{0x08}},
    {0x68,1,{0x0C}},
    {0x69,1,{0x0D}},
    {0x6a,1,{0x0E}},
    {0x6b,1,{0x0F}},
    {0x6c,1,{0x02}},
    {0x6d,1,{0x02}},
    {0x6e,1,{0x02}},
    {0x6F,1,{0x02}},
    {0x70,1,{0x02}},
    {0x71,1,{0x02}},
    {0x72,1,{0x02}},
    {0x73,1,{0x02}},
    {0x74,1,{0x02}},
    {0x75,1,{0x02}},
    {0x76,1,{0x05}},
    {0x77,1,{0x05}},
    {0x78,1,{0x05}},
    {0x79,1,{0x02}},
    {0x7a,1,{0x01}},
    {0x7b,1,{0x00}},
    {0x7c,1,{0x06}},

    {0xFF,3,{0x98,0x81,0x08}},
    //{0xA3,1,{0x80}},
    {0x76,1,{0xB4}},
    {0x78,1,{0x02}},
    {0x74,1,{0x37}},       //CLAMP_VGH==16V
    {0x8E,1,{0x15}},       //CLAMP_VGL=-10V
    {0x40,1,{0x01}},
    {0x84,1,{0x81}},
    {0x72,1,{0x25}},
    {0xE3,1,{0x75}},
    {0x7D,1,{0xCB}},
    {0x76,1,{0xD4}},     //3.5*VSP
    {0x78,1,{0x06}},     //-3*VSP   VGL=-10V
    {0x7E,1,{0x49}},
    {0x49,1,{0x10}},
    {0x32,1,{0x05}},
    {0x3C,1,{0x05}},

    {0xFF,3,{0x98,0x81,0x01}},
    {0x22,1,{0x0A}},
    {0x43,1,{0x01}},
    {0x53,1,{0x4D}},
    {0x55,1,{0x68}},
    {0x50,1,{0xB9}},
    {0x51,1,{0xBA}},
    {0x31,1,{0x00}},

    {0xA0,1,{0x08}},
    {0xA1,1,{0x10}},
    {0xA2,1,{0x17}},
    {0xA3,1,{0x0D}},
    {0xA4,1,{0x0D}},
    {0xA5,1,{0x1C}},
    {0xA6,1,{0x10}},
    {0xA7,1,{0x14}},
    {0xA8,1,{0x4D}},
    {0xA9,1,{0x1A}},
    {0xAA,1,{0x26}},
    {0xAB,1,{0x53}},
    {0xAC,1,{0x1C}},
    {0xAD,1,{0x1A}},
    {0xAE,1,{0x4F}},
    {0xAF,1,{0x23}},
    {0xB0,1,{0x27}},
    {0xB1,1,{0x4C}},
    {0xB2,1,{0x62}},
    {0xB3,1,{0x39}},

    {0xC0,1,{0x08}},
    {0xC1,1,{0x10}},
    {0xC2,1,{0x17}},
    {0xC3,1,{0x0D}},
    {0xC4,1,{0x0D}},
    {0xC5,1,{0x1C}},
    {0xC6,1,{0x10}},
    {0xC7,1,{0x14}},
    {0xC8,1,{0x4D}},
    {0xC9,1,{0x1A}},
    {0xCA,1,{0x26}},
    {0xCB,1,{0x53}},
    {0xCC,1,{0x1C}},
    {0xCD,1,{0x1A}},
    {0xCE,1,{0x4F}},
    {0xCF,1,{0x23}},
    {0xD0,1,{0x27}},
    {0xD1,1,{0x4C}},
    {0xD2,1,{0x62}},
    {0xD3,1,{0x39}},

    {0xFF,3,{0x98,0x81,0x00}},
    {0x35,1,{0x00}},
    {0x11,1,{0x00}},
    {REGFLAG_DELAY,200,{0x00}},
    {0x29,1,{0x00}},
    {REGFLAG_DELAY,20,{0x00}},
    {REGFLAG_END_OF_TABLE, 0x00, {}}
};

//static struct LCM_setting_table lcm_sleep_out_setting[] = {
//  // Sleep Out
//  {0x11, 1, {0x00}},
//  {REGFLAG_DELAY, 120, {}},
//  // Display ON
//  {0x29, 1, {0x00}},
//  {REGFLAG_DELAY, 100, {}},

//  {REGFLAG_END_OF_TABLE, 0x00, {}}
//};


static struct LCM_setting_table lcm_deep_sleep_mode_in_setting[] = {
    // Display off sequence
    {0x28, 1, {0x00}},
    {REGFLAG_DELAY, 10, {}},
    // Sleep Mode On
    {0x10, 1, {0x00}},
    {REGFLAG_DELAY, 120, {}},

    {REGFLAG_END_OF_TABLE, 0x00, {}}
};

static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
    memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}

static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
    unsigned int i;
    unsigned cmd;

    for(i = 0; i < count; i++) {
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

static void lcm_get_params(LCM_PARAMS *params)
{
    memset(params, 0, sizeof(LCM_PARAMS));

    params->type   = LCM_TYPE_DSI;

    params->width  = FRAME_WIDTH;
    params->height = FRAME_HEIGHT;

#if (LCM_DSI_CMD_MODE)
    params->dsi.mode   = CMD_MODE;
#else
    params->dsi.mode   = SYNC_PULSE_VDO_MODE; //SYNC_PULSE_VDO_MODE;//BURST_VDO_MODE; 
#endif

    params->physical_width = 73;
    params->physical_height = 119;

    // DSI
    /* Command mode setting */
    //1 Three lane or Four lane
    params->dsi.LANE_NUM                = LCM_THREE_LANE;
    //The following defined the fomat for data coming from LCD engine.
    params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
    params->dsi.data_format.trans_seq   = LCM_DSI_TRANS_SEQ_MSB_FIRST;
    params->dsi.data_format.padding     = LCM_DSI_PADDING_ON_LSB;
    params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;

    // Highly depends on LCD driver capability.
    params->dsi.packet_size=256;
    //video mode timing
    params->dsi.intermediat_buffer_num = 0;//because DSI/DPI HW design change, this parameters should be 0 when video mode in MT658X; or memory leakage

    params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;

    params->dsi.vertical_sync_active = 2;// 3    2
    params->dsi.vertical_backporch = 14;// 20   1
    params->dsi.vertical_frontporch = 8; // 1  12
    params->dsi.vertical_active_line = FRAME_HEIGHT; 

    params->dsi.horizontal_sync_active = 20;// 50  2
    params->dsi.horizontal_backporch = 120;
    params->dsi.horizontal_frontporch = 120;
    params->dsi.horizontal_active_pixel = FRAME_WIDTH;

    params->dsi.PLL_CLOCK=280;
    params->dsi.clk_lp_per_line_enable = 0;
    params->dsi.esd_check_enable = 1;
    params->dsi.customization_esd_check_enable = 0;
    params->dsi.lcm_esd_check_table[0].cmd          = 0x0a;
    params->dsi.lcm_esd_check_table[0].count        = 1;
    params->dsi.lcm_esd_check_table[0].para_list[0] = 0x9c;
}



static void lcm_init(void)
{
    MDELAY(50);
    SET_RESET_PIN(1);
    MDELAY(20);
    SET_RESET_PIN(0);
    MDELAY(20);
    SET_RESET_PIN(1);
    MDELAY(120);
    push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);  
}

static void lcm_suspend(void)
{
    unsigned int data_array[16];
    data_array[0] = 0x00280500; // Display Off
    dsi_set_cmdq(data_array, 1, 1); 
    MDELAY(10);
    data_array[0] = 0x00100500; ; // Sleep In
    dsi_set_cmdq(data_array, 1, 1);
    MDELAY(10);
    //synaptics_rmi4_irq_enable_lcd(1);
    lcm_gpio_output(0,0);
    push_table(lcm_deep_sleep_mode_in_setting, sizeof(lcm_deep_sleep_mode_in_setting) / sizeof(struct LCM_setting_table), 1);
    MDELAY(10);
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
    //SET_RESET_PIN(1);
    lcm_gpio_output(0,1);

    //push_table(lcm_sleep_out_setting, sizeof(lcm_sleep_out_setting) / sizeof(struct LCM_setting_table), 1);
    lcm_init();
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

static unsigned int lcm_compare_id(void)
{
    unsigned int id = 0;
    unsigned char buffer[3];
    unsigned int data_array[16];

    SET_RESET_PIN(1);
    SET_RESET_PIN(0);
    MDELAY(10);
    SET_RESET_PIN(1);
    MDELAY(120);

    data_array[0]=0x00043902; 
    data_array[1]=0x018198FF;
    dsi_set_cmdq(data_array, 2, 1); 
    MDELAY(10);

    data_array[0] = 0x00013700;
    dsi_set_cmdq(data_array, 1, 1);
    read_reg_v2(0x00, &buffer[0], 1);

    data_array[0] = 0x00013700;
    dsi_set_cmdq(data_array, 1, 1);
    read_reg_v2(0x01, &buffer[1], 1);

    id = (buffer[0] << 8) | buffer[1];

#ifdef BUILD_LK
    printf("read id, buf0:0x%02x, buf1=0x%x, id=0x%x", buffer[0], buffer[1], id);
#else
    printk("read id, buf0:0x%02x, buf1=0x%x, id=0x%x", buffer[0], buffer[1], id);
#endif

    return (LCM_ID_ILI9881C == id)?1:0;
}

static unsigned int lcm_ata_check(unsigned char *buffer)
{
    extern int lcm_ata_check_flag;

    return (lcm_ata_check_flag > 0) ? 1 : 0;
}

LCM_DRIVER ili9881c_hd720_dsi_vdo_fengcai_lcm_drv =
{
    .name           = "ili9881c_hd720_dsi_vdo_fengcai",
    .set_util_funcs = lcm_set_util_funcs,
    .get_params     = lcm_get_params,
    .init           = lcm_init,
    .suspend        = lcm_suspend,
    .resume         = lcm_resume,
    .compare_id     = lcm_compare_id,
    .ata_check      = lcm_ata_check,
    //.esd_check      = lcm_esd_check,
    //.esd_recover    = lcm_esd_recover,
#if (LCM_DSI_CMD_MODE)
    .update         = lcm_update,
#endif
};

