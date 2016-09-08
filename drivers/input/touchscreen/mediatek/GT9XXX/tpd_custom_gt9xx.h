/* Copyright Statement:
 *
 * This software/firmware and related documentation ("MediaTek Software") are
 * protected under relevant copyright laws. The information contained herein
 * is confidential and proprietary to MediaTek Inc. and/or its licensors.
 * Without the prior written permission of MediaTek inc. and/or its licensors,
 * any reproduction, modification, use or disclosure of MediaTek Software,
 * and information contained herein, in whole or in part, shall be strictly prohibited.
 */ 
/* MediaTek Inc. (C) 2010. All rights reserved.
 * 
 * BY OPENING THIS FILE, RECEIVER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
 * THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE") 
 * RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO RECEIVER ON
 * AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
 * NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
 * SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
 * SUPPLIED WITH THE MEDIATEK SOFTWARE, AND RECEIVER AGREES TO LOOK ONLY TO SUCH
 * THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. RECEIVER EXPRESSLY ACKNOWLEDGES
 * THAT IT IS RECEIVER'S SOLE RESPONSIBILITY TO OBTAIN FROM ANY THIRD PARTY ALL PROPER LICENSES
 * CONTAINED IN MEDIATEK SOFTWARE. MEDIATEK SHALL ALSO NOT BE RESPONSIBLE FOR ANY MEDIATEK
 * SOFTWARE RELEASES MADE TO RECEIVER'S SPECIFICATION OR TO CONFORM TO A PARTICULAR
 * STANDARD OR OPEN FORUM. RECEIVER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND
 * CUMULATIVE LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
 * AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
 * OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY RECEIVER TO
 * MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
 *
 * The following software/firmware and/or related documentation ("MediaTek Software")
 * have been modified by MediaTek Inc. All revisions are subject to any receiver's
 * applicable license agreements with MediaTek Inc.
 */
 /*
  * Version: V2.5
  * Release Date: 2015/01/21
  */

#ifndef TPD_CUSTOM_GT9XX_H__
#define TPD_CUSTOM_GT9XX_H__

#include <linux/hrtimer.h>
#include <linux/string.h>
#include <linux/vmalloc.h>
//#include <linux/io.h>

#include <linux/init.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/bitops.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/byteorder/generic.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
#include <linux/interrupt.h>
#include <linux/time.h>
#include <linux/rtpm_prio.h>

#include <linux/proc_fs.h>
#include <asm/uaccess.h>
#include <linux/jiffies.h>

#include "tpd.h"

/* Pre-defined definition */

#define TPD_KEY_COUNT   3
#define key_1           245,2000              //auto define  
#define key_2           540,2000
#define key_3           834,2000

#ifdef LYCONFIG_COMB_CUST_PROJECT_NAME_F1_YP
#define TPD_KEYS        {KEY_BACK, KEY_HOMEPAGE,KEY_MENU}
#else
#define TPD_KEYS        {KEY_MENU, KEY_HOMEPAGE,KEY_BACK}
#endif
#define TPD_KEYS_DIM    {{key_1,50,30},{key_2,50,30},{key_3,50,30}}

extern u16 show_len;
extern u16 total_len;
extern u8 gtp_rawdiff_mode;
extern unsigned int touch_irq;

extern int tpd_halt;
extern s32 gtp_send_cfg(struct i2c_client *client);
extern void gtp_reset_guitar(struct i2c_client *client, s32 ms);
extern void gtp_int_sync(s32 ms);       
extern u8 gup_init_update_proc(struct i2c_client *client);
extern u8 gup_init_fw_proc(struct i2c_client *client);
extern s32 gtp_i2c_read(struct i2c_client *client, u8 *buf, s32 len);
extern s32 gtp_i2c_write(struct i2c_client *client,u8 *buf,s32 len);
extern int i2c_write_bytes(struct i2c_client *client, u16 addr, u8 *txbuf, int len);
extern int i2c_read_bytes(struct i2c_client *client, u16 addr, u8 *rxbuf, int len);
extern s32 i2c_read_dbl_check(struct i2c_client *client, u16 addr, u8 *rxbuf, int len);
extern s32 gtp_i2c_read_dbl_check(struct i2c_client *client, u16 addr, u8 *rxbuf, int len);

//***************************PART1:ON/OFF define*******************************
#define GTP_CUSTOM_CFG        0
#define GTP_DRIVER_SEND_CFG   0       // driver send config to TP in intilization
#define GTP_HAVE_TOUCH_KEY    0
#define GTP_POWER_CTRL_SLEEP  0       // turn off/on power on suspend/resume

#define GTP_AUTO_UPDATE       0       // auto updated fw by .bin file
#define GTP_HEADER_FW_UPDATE  0       // auto updated fw by gtp_default_FW in gt9xx_firmware.h, function together with GTP_AUTO_UDPATE
#define GTP_AUTO_UPDATE_CFG   0       // auto update config by .cfg file, function together with GTP_AUTO_UPDATE

#define GTP_SUPPORT_I2C_DMA   1       // if gt9xxf, better enable it if hardware platform supported
#define GTP_COMPATIBLE_MODE   0       // compatible with GT9XXF

#define GTP_CREATE_WR_NODE    1
#define GTP_ESD_PROTECT       1       // esd protection with a cycle of 2 seconds
#define GTP_CHARGER_SWITCH    0       // charger plugin & plugout detect

#define GTP_WITH_PEN          0      
#define GTP_PEN_HAVE_BUTTON   0       // active pen has buttons, functions together with GTP_WITH_PEN

#if defined(LYCONFIG_FEA_TP_WAKEUP_SUPPORT)
#define GTP_GESTURE_WAKEUP    1 
#else
#define GTP_GESTURE_WAKEUP    0 
#endif
//#define TPD_PROXIMITY
#define TPD_HAVE_BUTTON             // report key as coordinate,Vibration feedback
//#define TPD_WARP_X                  // mirrored x coordinate
//#define TPD_WARP_Y                  // mirrored y coordinate
#define GTP_DEBUG_ON          1
#define GTP_DEBUG_ARRAY_ON    0
#define GTP_DEBUG_FUNC_ON     0

//***************************PART2:TODO define**********************************
//STEP_1(REQUIRED):Change config table.
// Sensor_ID Map:
/* sensor_opt1 sensor_opt2 Sensor_ID 
    GND         GND         0 
    VDDIO       GND         1 
    NC          GND         2 
    GND         NC/300K     3 
    VDDIO       NC/300K     4 
    NC          NC/300K     5 
*/
// TODO: define your own default or for Sensor_ID == 0 config here. 
// The predefined one is just a sample config, which is not suitable for your tp in most cases.
#define CTP_CFG_GROUP0 {\
0x58,0x38,0x04,0x80,0x07,0x05,0x34,0x01,0x01,0x4C,\
0x28,0x0F,0x50,0x3C,0x03,0x05,0x00,0x00,0xFF,0x7F,\
0x01,0x00,0x05,0x16,0x16,0x20,0x14,0x8A,0x2A,0x0B,\
0x37,0x39,0x0C,0x08,0x03,0x06,0x00,0x02,0x33,0x1D,\
0x3C,0x03,0x00,0x00,0x00,0x0C,0x00,0x00,0x00,0x00,\
0x2B,0x28,0x55,0x94,0xD5,0x02,0x00,0x00,0x00,0x04,\
0x98,0x2B,0x00,0x87,0x32,0x00,0x79,0x3A,0x00,0x6D,\
0x43,0x00,0x62,0x4F,0x00,0x62,0x18,0x30,0x48,0x00,\
0xF0,0x4A,0x3A,0xBB,0xBB,0x29,0x00,0x00,0x00,0x00,\
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x0F,\
0x14,0x03,0x04,0x10,0x42,0xF8,0x0A,0x19,0x01,0x00,\
0x46,0x32,0x02,0x04,0x06,0x08,0x0A,0x0C,0x0E,0x10,\
0x12,0x14,0x16,0xFF,0xFF,0xFF,0x00,0x00,0x00,0x00,\
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,\
0x00,0x00,0x12,0x10,0x0F,0x0C,0x0A,0x08,0x06,0x04,\
0x02,0x00,0x16,0x18,0x1C,0x1D,0x1E,0x1F,0x20,0x21,\
0x22,0x24,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0x00,0x00,\
0x00,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,\
0xFF,0xFF,0xFF,0xFF,0x16,0x01\
    }

#define GTP_CFG_GROUP0_CHARGER {\
	0x47,0xD0,0x02,0x00,0x05,0x05,0x34,0x00,0x01,0x8C,\
0x1E,0x0C,0x50,0x3C,0x03,0x07,0x01,0x01,0x00,0x00,\
0x00,0x00,0x00,0x18,0x1A,0x1E,0x14,0x8B,0x2B,0x0C,\
0x50,0x52,0xD6,0x09,0x00,0x00,0x00,0x9C,0x32,0x1D,\
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,\
0xF4,0x4A,0x64,0x9E,0xE5,0x01,0x14,0x00,0x00,0x04,\
0x74,0x4C,0x00,0x70,0x50,0x00,0x69,0x55,0x00,0x63,\
0x5B,0x00,0x5E,0x61,0x00,0x5E,0x00,0x00,0x00,0x00,\
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,\
0x00,0x01,0x1B,0x14,0x0D,0x14,0x03,0x0F,0x0A,0x03,\
0x07,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,\
0x00,0x00,0x02,0x04,0x06,0x08,0x0A,0x0C,0x0E,0x10,\
0x12,0x14,0x16,0x18,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,\
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,\
0xFF,0xFF,0x00,0x01,0x02,0x04,0x06,0x07,0x08,0x09,\
0x0A,0x0C,0x0E,0x1D,0x1E,0x1F,0x20,0x22,0x24,0x25,\
0x26,0x28,0x29,0x2A,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,\
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,\
0xFF,0xFF,0xFF,0xFF,0xB5,0x01\
}

// TODO: define your config for Sensor_ID == 1 here, if needed
#define CTP_CFG_GROUP1 {\
    }

#define GTP_CFG_GROUP1_CHARGER {\
}

// TODO: define your config for Sensor_ID == 2 here, if needed
#define CTP_CFG_GROUP2 {\
    }

#define GTP_CFG_GROUP2_CHARGER {\
}


// TODO: define your config for Sensor_ID == 3 here, if needed
#define CTP_CFG_GROUP3 {\
    }

#define GTP_CFG_GROUP3_CHARGER {\
}


// TODO: define your config for Sensor_ID == 4 here, if needed
#define CTP_CFG_GROUP4 {\
    }

#define GTP_CFG_GROUP4_CHARGER {\
}

// TODO: define your config for Sensor_ID == 5 here, if needed
#define CTP_CFG_GROUP5 {\
    }

#define GTP_CFG_GROUP5_CHARGER {\
}


// STEP_2(REQUIRED): Customize your I/O ports & I/O operations here
//#define TPD_POWER_SOURCE_CUSTOM     MT65XX_POWER_LDO_VGP4      // define your power source for tp if needed

#define GTP_GPIO_AS_INT(pin) tpd_gpio_as_int(pin)
#define GTP_GPIO_OUTPUT(pin, level) tpd_gpio_output(pin, level)

#define GTP_IRQ_TAB                     {IRQ_TYPE_EDGE_RISING, IRQ_TYPE_EDGE_FALLING, IRQ_TYPE_LEVEL_LOW, IRQ_TYPE_LEVEL_HIGH}

// STEP_3(optional):Custom set some config by themself,if need.
#if GTP_CUSTOM_CFG
  #define GTP_MAX_HEIGHT   800           
  #define GTP_MAX_WIDTH    480
  #define GTP_INT_TRIGGER  0    //0:Rising 1:Falling
#else 
  #define GTP_MAX_HEIGHT   4096
  #define GTP_MAX_WIDTH    4096
  #define GTP_INT_TRIGGER  1
#endif
#define GTP_MAX_TOUCH      5
#define VELOCITY_CUSTOM
#define TPD_VELOCITY_CUSTOM_X 15
#define TPD_VELOCITY_CUSTOM_Y 15

//STEP_4(optional):If this project have touch key,Set touch key config.                                    
#if GTP_HAVE_TOUCH_KEY
    #define GTP_KEY_TAB  {KEY_MENU, KEY_HOMEPAGE, KEY_BACK, KEY_SEND}
#endif
#if GTP_GESTURE_WAKEUP
    #define TPD_GESTURE_TAB {KEY_RIGHT,KEY_DOWN,KEY_UP,KEY_LEFT,KEY_O,KEY_M,KEY_E,KEY_C,KEY_W,KEY_S,KEY_Z,KEY_V,KEY_P}
#endif
//***************************PART3:OTHER define*********************************
#define GTP_DRIVER_VERSION          "V2.5<2015/01/21>"
#define GTP_I2C_NAME                "Goodix-TS"
#define GT91XX_CONFIG_PROC_FILE     "gt9xx_config"
#define GTP_POLL_TIME               10
#define GTP_ADDR_LENGTH             2
#define GTP_CONFIG_MIN_LENGTH       186
#define GTP_CONFIG_MAX_LENGTH       240
#define FAIL                        0
#define SUCCESS                     1
#define SWITCH_OFF                  0
#define SWITCH_ON                   1

#define CFG_GROUP_LEN(p_cfg_grp)  (sizeof(p_cfg_grp) / sizeof(p_cfg_grp[0]))

//******************** For GT9XXF Start **********************//
#if GTP_COMPATIBLE_MODE
typedef enum
{
    CHIP_TYPE_GT9  = 0,
    CHIP_TYPE_GT9F = 1,
} CHIP_TYPE_T;
#endif

#define GTP_REG_MATRIX_DRVNUM           0x8069
#define GTP_REG_MATRIX_SENNUM           0x806A
#define GTP_REG_RQST                    0x8043
#define GTP_REG_BAK_REF                 0x99D0
#define GTP_REG_MAIN_CLK                0x8020
#define GTP_REG_CHIP_TYPE               0x8000
#define GTP_REG_HAVE_KEY                0x804E

#define GTP_FL_FW_BURN              0x00
#define GTP_FL_ESD_RECOVERY         0x01
#define GTP_FL_READ_REPAIR          0x02

#define GTP_BAK_REF_SEND                0
#define GTP_BAK_REF_STORE               1
#define CFG_LOC_DRVA_NUM                29
#define CFG_LOC_DRVB_NUM                30
#define CFG_LOC_SENS_NUM                31

#define GTP_CHK_FW_MAX                  1000
#define GTP_CHK_FS_MNT_MAX              300
#define GTP_BAK_REF_PATH                "/data/gtp_ref.bin"
#define GTP_MAIN_CLK_PATH               "/data/gtp_clk.bin"
#define GTP_RQST_CONFIG                 0x01
#define GTP_RQST_BAK_REF                0x02
#define GTP_RQST_RESET                  0x03
#define GTP_RQST_MAIN_CLOCK             0x04
#define GTP_RQST_RESPONDED              0x00
#define GTP_RQST_IDLE                   0xFF

//******************** For GT9XXF End **********************//

//Register define
#define GTP_READ_COOR_ADDR          0x814E
#define GTP_REG_SLEEP               0x8040
#define GTP_REG_SENSOR_ID           0x814A
#define GTP_REG_CONFIG_DATA         0x8047
#define GTP_REG_VERSION             0x8140
#define GTP_REG_HW_INFO             0x4220

#define RESOLUTION_LOC              3
#define TRIGGER_LOC                 8

#define I2C_MASTER_CLOCK                300
#define I2C_BUS_NUMBER                  1     // I2C Bus for TP, mt6572
#define GTP_DMA_MAX_TRANSACTION_LENGTH  255   // for DMA mode
#define GTP_DMA_MAX_I2C_TRANSFER_SIZE   (GTP_DMA_MAX_TRANSACTION_LENGTH - GTP_ADDR_LENGTH)
#define MAX_TRANSACTION_LENGTH          8
#define MAX_I2C_TRANSFER_SIZE           (MAX_TRANSACTION_LENGTH - GTP_ADDR_LENGTH)
#define TPD_MAX_RESET_COUNT             3
#define TPD_CALIBRATION_MATRIX          {962,0,0,0,1600,0,0,0};


#define TPD_RESET_ISSUE_WORKAROUND
#define TPD_HAVE_CALIBRATION
#define TPD_NO_GPIO
#define TPD_RESET_ISSUE_WORKAROUND

#ifdef TPD_WARP_X
#undef TPD_WARP_X
#define TPD_WARP_X(x_max, x) ( x_max - 1 - x )
#else
#define TPD_WARP_X(x_max, x) x
#endif

#ifdef TPD_WARP_Y
#undef TPD_WARP_Y
#define TPD_WARP_Y(y_max, y) ( y_max - 1 - y )
#else
#define TPD_WARP_Y(y_max, y) y
#endif

//Log define
#define GTP_INFO(fmt,arg...)           printk("<<-GTP-INFO->> "fmt"\n",##arg)
#define GTP_ERROR(fmt,arg...)          printk("<<-GTP-ERROR->> "fmt"\n",##arg)
#define GTP_DEBUG(fmt,arg...)          do{\
                                         if(GTP_DEBUG_ON)\
                                         printk("<<-GTP-DEBUG->> [%d]"fmt"\n",__LINE__, ##arg);\
                                       }while(0)
#define GTP_DEBUG_ARRAY(array, num)    do{\
                                         s32 i;\
                                         u8* a = array;\
                                         if(GTP_DEBUG_ARRAY_ON)\
                                         {\
                                            printk("<<-GTP-DEBUG-ARRAY->>\n");\
                                            for (i = 0; i < (num); i++)\
                                            {\
                                                printk("%02x   ", (a)[i]);\
                                                if ((i + 1 ) %10 == 0)\
                                                {\
                                                    printk("\n");\
                                                }\
                                            }\
                                            printk("\n");\
                                        }\
                                       }while(0)
#define GTP_DEBUG_FUNC()               do{\
                                         if(GTP_DEBUG_FUNC_ON)\
                                         printk("<<-GTP-FUNC->> Func:%s@Line:%d\n",__func__,__LINE__);\
                                       }while(0)
#define GTP_SWAP(x, y)                 do{\
                                         typeof(x) z = x;\
                                         x = y;\
                                         y = z;\
                                       }while (0)


//*****************************End of Part III********************************

#endif /* TPD_CUSTOM_GT9XX_H__ */
