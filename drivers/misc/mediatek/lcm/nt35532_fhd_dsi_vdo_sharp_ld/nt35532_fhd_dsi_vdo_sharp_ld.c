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

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)    lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)       lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)                                      lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)                  lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)                                       lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size)               lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)


#define LCM_DSI_CMD_MODE    0

extern unsigned int pmic_config_interface(unsigned int RegNum, unsigned int val, unsigned int MASK, unsigned int SHIFT);

#if 1
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
#endif

static void init_lcm_registers(void)
{
    unsigned int data_array[16];

    data_array[0] = 0x01FF1500;
    dsi_set_cmdq(data_array, 1, 1);
    data_array[0] = 0x01FB1500;
    dsi_set_cmdq(data_array, 1, 1);
    data_array[0] = 0x01001500;
    dsi_set_cmdq(data_array, 1, 1);
    data_array[0] = 0x55011500;
    dsi_set_cmdq(data_array, 1, 1);
    data_array[0] = 0x59021500;
    dsi_set_cmdq(data_array, 1, 1);
    data_array[0] = 0x0C041500;
    dsi_set_cmdq(data_array, 1, 1);
    data_array[0] = 0x2A051500;
    dsi_set_cmdq(data_array, 1, 1);
    data_array[0] = 0x64061500;
    dsi_set_cmdq(data_array, 1, 1);
    data_array[0] = 0xC6071500;
    dsi_set_cmdq(data_array, 1, 1);
    data_array[0] = 0x890D1500;
    dsi_set_cmdq(data_array, 1, 1);
    data_array[0] = 0x890E1500;
    dsi_set_cmdq(data_array, 1, 1);
    data_array[0] = 0xE00F1500;
    dsi_set_cmdq(data_array, 1, 1);
    data_array[0] = 0x03101500;
    dsi_set_cmdq(data_array, 1, 1);
    data_array[0] = 0x5A111500;
    dsi_set_cmdq(data_array, 1, 1);
    data_array[0] = 0x5A121500;
    dsi_set_cmdq(data_array, 1, 1);
    data_array[0] = 0x75131500;
    dsi_set_cmdq(data_array, 1, 1);
    data_array[0] = 0x75141500;
    dsi_set_cmdq(data_array, 1, 1);
    data_array[0] = 0x60151500;
    dsi_set_cmdq(data_array, 1, 1);
    data_array[0] = 0x13161500;
    dsi_set_cmdq(data_array, 1, 1);
    data_array[0] = 0x13171500;
    dsi_set_cmdq(data_array, 1, 1);
    data_array[0] = 0x77601500;
    dsi_set_cmdq(data_array, 1, 1);
    data_array[0] = 0xA31C1500;
    dsi_set_cmdq(data_array, 1, 1);
    data_array[0] = 0x05FF1500;
    dsi_set_cmdq(data_array, 1, 1);
    data_array[0] = 0x01FB1500;
    dsi_set_cmdq(data_array, 1, 1);
    data_array[0] = 0x38001500;
    dsi_set_cmdq(data_array, 1, 1);
    data_array[0] = 0x38011500;
    dsi_set_cmdq(data_array, 1, 1);
    data_array[0] = 0x07021500;
    dsi_set_cmdq(data_array, 1, 1);
    data_array[0] = 0x40031500;
    dsi_set_cmdq(data_array, 1, 1);
    data_array[0] = 0x40041500;
    dsi_set_cmdq(data_array, 1, 1);
    data_array[0] = 0x25051500;
    dsi_set_cmdq(data_array, 1, 1);
    data_array[0] = 0x1D061500;
    dsi_set_cmdq(data_array, 1, 1);
    data_array[0] = 0x23071500;
    dsi_set_cmdq(data_array, 1, 1);
    data_array[0] = 0x1B081500;
    dsi_set_cmdq(data_array, 1, 1);
    data_array[0] = 0x21091500;
    dsi_set_cmdq(data_array, 1, 1);
    data_array[0] = 0x190A1500;
    dsi_set_cmdq(data_array, 1, 1);
    data_array[0] = 0x1F0B1500;
    dsi_set_cmdq(data_array, 1, 1);
    data_array[0] = 0x170C1500;
    dsi_set_cmdq(data_array, 1, 1);
    data_array[0] = 0x050D1500;
    dsi_set_cmdq(data_array, 1, 1);
    data_array[0] = 0x040E1500;
    dsi_set_cmdq(data_array, 1, 1);
    data_array[0] = 0x0F0F1500;
    dsi_set_cmdq(data_array, 1, 1);
    data_array[0] = 0x38101500;
    dsi_set_cmdq(data_array, 1, 1);
    data_array[0] = 0x38111500;
    dsi_set_cmdq(data_array, 1, 1);
    data_array[0] = 0x38121500;
    dsi_set_cmdq(data_array, 1, 1);
    data_array[0] = 0x38131500;
    dsi_set_cmdq(data_array, 1, 1);
    data_array[0] = 0x38141500;
    dsi_set_cmdq(data_array, 1, 1);
    data_array[0] = 0x38151500;
    dsi_set_cmdq(data_array, 1, 1);
    data_array[0] = 0x06161500;
    dsi_set_cmdq(data_array, 1, 1);
    data_array[0] = 0x40171500;
    dsi_set_cmdq(data_array, 1, 1);
    data_array[0] = 0x40181500;
    dsi_set_cmdq(data_array, 1, 1);
    data_array[0] = 0x24191500;
    dsi_set_cmdq(data_array, 1, 1);
    data_array[0] = 0x1C1A1500;
    dsi_set_cmdq(data_array, 1, 1);
    data_array[0] = 0x221B1500;
    dsi_set_cmdq(data_array, 1, 1);
    data_array[0] = 0x1A1C1500;
    dsi_set_cmdq(data_array, 1, 1);
    data_array[0] = 0x201D1500;
    dsi_set_cmdq(data_array, 1, 1);
    data_array[0] = 0x181E1500;
    dsi_set_cmdq(data_array, 1, 1);
    data_array[0] = 0x1E1F1500;
    dsi_set_cmdq(data_array, 1, 1);
    data_array[0] = 0x16201500;
    dsi_set_cmdq(data_array, 1, 1);
    data_array[0] = 0x05211500;
    dsi_set_cmdq(data_array, 1, 1);
    data_array[0] = 0x04221500;
    dsi_set_cmdq(data_array, 1, 1);
    data_array[0] = 0x0E231500;
    dsi_set_cmdq(data_array, 1, 1);
    data_array[0] = 0x38241500;
    dsi_set_cmdq(data_array, 1, 1);
    data_array[0] = 0x38251500;
    dsi_set_cmdq(data_array, 1, 1);
    data_array[0] = 0x38261500;
    dsi_set_cmdq(data_array, 1, 1);
    data_array[0] = 0x38271500;
    dsi_set_cmdq(data_array, 1, 1);
    data_array[0] = 0x38281500;
    dsi_set_cmdq(data_array, 1, 1);
    data_array[0] = 0x38291500;
    dsi_set_cmdq(data_array, 1, 1);
    data_array[0] = 0x0E2A1500;
    dsi_set_cmdq(data_array, 1, 1);
    data_array[0] = 0x402B1500;
    dsi_set_cmdq(data_array, 1, 1);
    data_array[0] = 0x402D1500;
    dsi_set_cmdq(data_array, 1, 1);
    data_array[0] = 0x162F1500;
    dsi_set_cmdq(data_array, 1, 1);
    data_array[0] = 0x1E301500;
    dsi_set_cmdq(data_array, 1, 1);
    data_array[0] = 0x18311500;
    dsi_set_cmdq(data_array, 1, 1);
    data_array[0] = 0x20321500;
    dsi_set_cmdq(data_array, 1, 1);
    data_array[0] = 0x1A331500;
    dsi_set_cmdq(data_array, 1, 1);
    data_array[0] = 0x22341500;
    dsi_set_cmdq(data_array, 1, 1);
    data_array[0] = 0x1C351500;
    dsi_set_cmdq(data_array, 1, 1);
    data_array[0] = 0x24361500;
    dsi_set_cmdq(data_array, 1, 1);
    data_array[0] = 0x05371500;
    dsi_set_cmdq(data_array, 1, 1);
    data_array[0] = 0x04381500;
    dsi_set_cmdq(data_array, 1, 1);
    data_array[0] = 0x06391500;
    dsi_set_cmdq(data_array, 1, 1);
    data_array[0] = 0x383A1500;
    dsi_set_cmdq(data_array, 1, 1);
    data_array[0] = 0x383B1500;
    dsi_set_cmdq(data_array, 1, 1);
    data_array[0] = 0x383D1500;
    dsi_set_cmdq(data_array, 1, 1);
    data_array[0] = 0x383F1500;
    dsi_set_cmdq(data_array, 1, 1);
    data_array[0] = 0x38401500;
    dsi_set_cmdq(data_array, 1, 1);
    data_array[0] = 0x38411500;
    dsi_set_cmdq(data_array, 1, 1);
    data_array[0] = 0x0F421500;
    dsi_set_cmdq(data_array, 1, 1);
    data_array[0] = 0x40431500;
    dsi_set_cmdq(data_array, 1, 1);
    data_array[0] = 0x40441500;
    dsi_set_cmdq(data_array, 1, 1);
    data_array[0] = 0x17451500;
    dsi_set_cmdq(data_array, 1, 1);
    data_array[0] = 0x1F461500;
    dsi_set_cmdq(data_array, 1, 1);
    data_array[0] = 0x19471500;
    dsi_set_cmdq(data_array, 1, 1);
    data_array[0] = 0x21481500;
    dsi_set_cmdq(data_array, 1, 1);
    data_array[0] = 0x1B491500;
    dsi_set_cmdq(data_array, 1, 1);
    data_array[0] = 0x234A1500;
    dsi_set_cmdq(data_array, 1, 1);
    data_array[0] = 0x1D4B1500;
    dsi_set_cmdq(data_array, 1, 1);
    data_array[0] = 0x254C1500;
    dsi_set_cmdq(data_array, 1, 1);
    data_array[0] = 0x054D1500;
    dsi_set_cmdq(data_array, 1, 1);
    data_array[0] = 0x044E1500;
    dsi_set_cmdq(data_array, 1, 1);
    data_array[0] = 0x074F1500;
    dsi_set_cmdq(data_array, 1, 1);
    data_array[0] = 0x38501500;
    dsi_set_cmdq(data_array, 1, 1);
    data_array[0] = 0x38511500;
    dsi_set_cmdq(data_array, 1, 1);
    data_array[0] = 0x38521500;
    dsi_set_cmdq(data_array, 1, 1);
    data_array[0] = 0x38531500;
    dsi_set_cmdq(data_array, 1, 1);
    data_array[0] = 0x0A5B1500;
    dsi_set_cmdq(data_array, 1, 1);
    data_array[0] = 0x0A5C1500;
    dsi_set_cmdq(data_array, 1, 1);
    data_array[0] = 0x0A631500;
    dsi_set_cmdq(data_array, 1, 1);
    data_array[0] = 0x15641500;
    dsi_set_cmdq(data_array, 1, 1);
    data_array[0] = 0x24681500;
    dsi_set_cmdq(data_array, 1, 1);
    data_array[0] = 0x24691500;
    dsi_set_cmdq(data_array, 1, 1);
    data_array[0] = 0x7B901500;
    dsi_set_cmdq(data_array, 1, 1);
    data_array[0] = 0x11911500;
    dsi_set_cmdq(data_array, 1, 1);
    data_array[0] = 0x14921500;
    dsi_set_cmdq(data_array, 1, 1);
    data_array[0] = 0x107E1500;
    dsi_set_cmdq(data_array, 1, 1);
    data_array[0] = 0x107F1500;
    dsi_set_cmdq(data_array, 1, 1);
    data_array[0] = 0x00801500;
    dsi_set_cmdq(data_array, 1, 1);
    data_array[0] = 0x00981500;
    dsi_set_cmdq(data_array, 1, 1);
    data_array[0] = 0x00991500;
    dsi_set_cmdq(data_array, 1, 1);
    data_array[0] = 0x2E541500;
    dsi_set_cmdq(data_array, 1, 1);
    data_array[0] = 0x38591500;
    dsi_set_cmdq(data_array, 1, 1);
    data_array[0] = 0x015D1500;
    dsi_set_cmdq(data_array, 1, 1);
    data_array[0] = 0x275E1500;
    dsi_set_cmdq(data_array, 1, 1);
    data_array[0] = 0x39621500;
    dsi_set_cmdq(data_array, 1, 1);
    data_array[0] = 0x88661500;
    dsi_set_cmdq(data_array, 1, 1);
    data_array[0] = 0x11671500;
    dsi_set_cmdq(data_array, 1, 1);
    data_array[0] = 0x0E6A1500;
    dsi_set_cmdq(data_array, 1, 1);
    data_array[0] = 0x206B1500;
    dsi_set_cmdq(data_array, 1, 1);
    data_array[0] = 0x086C1500;
    dsi_set_cmdq(data_array, 1, 1);
    data_array[0] = 0x006D1500;
    dsi_set_cmdq(data_array, 1, 1);
    data_array[0] = 0x017D1500;
    dsi_set_cmdq(data_array, 1, 1);
    data_array[0] = 0x01B71500;
    dsi_set_cmdq(data_array, 1, 1);
    data_array[0] = 0x0AB81500;
    dsi_set_cmdq(data_array, 1, 1);
    data_array[0] = 0x13BA1500;
    dsi_set_cmdq(data_array, 1, 1);
    data_array[0] = 0x01BC1500;
    dsi_set_cmdq(data_array, 1, 1);
    data_array[0] = 0x55BD1500;
    dsi_set_cmdq(data_array, 1, 1);
    data_array[0] = 0x38BE1500;
    dsi_set_cmdq(data_array, 1, 1);
    data_array[0] = 0x44BF1500;
    dsi_set_cmdq(data_array, 1, 1);
    data_array[0] = 0x77CF1500;
    dsi_set_cmdq(data_array, 1, 1);
    data_array[0] = 0x00C81500;
    dsi_set_cmdq(data_array, 1, 1);
    data_array[0] = 0x55C91500;
    dsi_set_cmdq(data_array, 1, 1);
    data_array[0] = 0x00CA1500;
    dsi_set_cmdq(data_array, 1, 1);
    data_array[0] = 0x55CB1500;
    dsi_set_cmdq(data_array, 1, 1);
    data_array[0] = 0xA2CC1500;
    dsi_set_cmdq(data_array, 1, 1);
    data_array[0] = 0x88CE1500;
    dsi_set_cmdq(data_array, 1, 1);
    data_array[0] = 0x88CF1500;
    dsi_set_cmdq(data_array, 1, 1);
    data_array[0] = 0x00D01500;
    dsi_set_cmdq(data_array, 1, 1);
    data_array[0] = 0x00D11500;
    dsi_set_cmdq(data_array, 1, 1);
    data_array[0] = 0x00D31500;
    dsi_set_cmdq(data_array, 1, 1);
    data_array[0] = 0x00D51500;
    dsi_set_cmdq(data_array, 1, 1);
    data_array[0] = 0x22D61500;
    dsi_set_cmdq(data_array, 1, 1);
    data_array[0] = 0x31D71500;
    dsi_set_cmdq(data_array, 1, 1);
    data_array[0] = 0x7ED81500;
    dsi_set_cmdq(data_array, 1, 1);
    MDELAY(10);
    data_array[0] = 0x00FF1500;
    dsi_set_cmdq(data_array, 1, 1);
    data_array[0] = 0x01FB1500;
    dsi_set_cmdq(data_array, 1, 1);
    data_array[0] = 0x15D31500;
    dsi_set_cmdq(data_array, 1, 1);
    data_array[0] = 0x10D41500;
    dsi_set_cmdq(data_array, 1, 1);
    data_array[0] = 0x18D51500;
    dsi_set_cmdq(data_array, 1, 1);
    data_array[0] = 0xB8D61500;
    dsi_set_cmdq(data_array, 1, 1);
    data_array[0] = 0x00D71500;
    dsi_set_cmdq(data_array, 1, 1);
    data_array[0] = 0x00351500;
    dsi_set_cmdq(data_array, 1, 1);
    data_array[0] = 0x00110500; 
    dsi_set_cmdq(data_array, 1, 1);
    MDELAY(120);

    data_array[0] = 0x00290500; 
    dsi_set_cmdq(data_array, 1, 1); 
    MDELAY(10);
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
    params->dsi.switch_mode = SYNC_PULSE_VDO_MODE;
#else
    params->dsi.mode   = SYNC_PULSE_VDO_MODE;
    //params->dsi.switch_mode = CMD_MODE;
#endif
    params->dsi.switch_mode_enable = 0;

    // DSI
    /* Command mode setting */
    //1 Three lane or Four lane
    params->dsi.LANE_NUM                = LCM_FOUR_LANE;
    //The following defined the fomat for data coming from LCD engine.
    params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
    params->dsi.data_format.trans_seq   = LCM_DSI_TRANS_SEQ_MSB_FIRST;
    params->dsi.data_format.padding     = LCM_DSI_PADDING_ON_LSB;
    params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;

    // Highly depends on LCD driver capability.
    // Not support in MT6573
    params->dsi.packet_size=256;

    // Video mode setting

    params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;

    params->dsi.vertical_sync_active    = 5;
    params->dsi.vertical_backporch      = 17;
    params->dsi.vertical_frontporch     = 20;
    params->dsi.vertical_active_line    = FRAME_HEIGHT;

    params->dsi.horizontal_sync_active  = 10;
    params->dsi.horizontal_backporch    = 118;
    params->dsi.horizontal_frontporch   = 118;
    params->dsi.horizontal_active_pixel = FRAME_WIDTH;

#if (LCM_DSI_CMD_MODE)
    params->dsi.PLL_CLOCK = 500; //this value must be in MTK suggested table
#else
    params->dsi.PLL_CLOCK = 500; //this value must be in MTK suggested table
#endif

    params->dsi.clk_lp_per_line_enable = 0;
    params->dsi.esd_check_enable = 1;
    params->dsi.lcm_ext_te_enable = 1;
    params->dsi.customization_esd_check_enable = 0;
    params->dsi.lcm_esd_check_table[0].cmd          = 0x0a;
    params->dsi.lcm_esd_check_table[0].count        = 1;
    params->dsi.lcm_esd_check_table[0].para_list[0] = 0x9c;

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

    read_reg_v2(0x0a, buffer, 1);

#ifdef BUILD_LK
    printf("%s, LK otm1282c id = 0x%08x\n", __func__, buffer[0]);
#else
    printk("%s, Kernel otm1282c id = 0x%08x\n", __func__, buffer[0]);
#endif

    return (0x08 == buffer[0])?1:0; 
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

LCM_DRIVER nt35532_fhd_dsi_vdo_sharp_ld_lcm_drv =
{
    .name           = "nt35532_fhd_dsi_vdo_sharp_ld",
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
