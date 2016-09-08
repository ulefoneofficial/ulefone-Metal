/*****************************************************************************
*
* Filename:
* ---------
*   sm5701_core.h
*
* Project:
* --------
*   Android
*
* Description:
* ------------
*   sm5701_core header file
*
* Author:
* -------
*
****************************************************************************/

#ifndef _SM5701_SW_H_
#define _SM5701_SW_H_

//#define HIGH_BATTERY_VOLTAGE_SUPPORT

/* SM5701 Registers. */
#define SM5701_INT1			0x00
#define SM5701_INT2			0x01
#define SM5701_INT3			0x02
#define SM5701_INTMASK1		0x03
#define SM5701_INTMASK2		0x04
#define SM5701_INTMASK3		0x05
#define SM5701_STATUS1		0x06
#define SM5701_STATUS2		0x07
#define SM5701_STATUS3		0x08
#define SM5701_CNTL			0x09
#define SM5701_VBUSCNTL		0x0A
#define SM5701_CHGCNTL1		0x0B
#define SM5701_CHGCNTL2		0x0C
#define SM5701_CHGCNTL3		0x0D
#define SM5701_CHGCNTL4		0x0E

#define SM5701_FLEDCNTL1	0x0F
#define SM5701_FLEDCNTL2	0x10
#define SM5701_FLEDCNTL3	0x11
#define SM5701_FLEDCNTL4	0x12
#define SM5701_FLEDCNTL5	0x13
#define SM5701_FLEDCNTL6	0x14
#define SM5701_DEVICE_ID    0x15

#define SM5701_REG_NUM 0x16 


/**********************************************************
  *
  *   [MASK/SHIFT] 
  *
  *********************************************************/
#define BATREG_MASK			0x00

//INT1
#define SM5701_INT1_AICL		0x01
#define SM5701_INT1_BATOVP		0x02
#define SM5701_INT1_VBUSOK		0x04
#define SM5701_INT1_VBUSUVLO	0x08
#define SM5701_INT1_VBUSOVP		0x10
#define SM5701_INT1_VBUSLIMIT	0x20
#define SM5701_INT1_CHGRSTF		0x40
#define SM5701_INT1_NOBAT		0x80
//INT2
#define SM5701_INT2_TOPOFF		0x01
#define SM5701_INT2_DONE		0x02
#define SM5701_INT2_FASTTMROFF	0x04
#define SM5701_INT2_CHGON		0x08
#define SM5701_INT2_OTGOCP		0x10
//INT3
#define SM5701_INT3_THEMSHDN	0x01
#define SM5701_INT3_THEMREG		0x02
#define SM5701_INT3_FLEDSHORT	0x04
#define SM5701_INT3_FLEDOPEN	0x08
#define SM5701_INT3_BOOSTPOK_NG	0x10
#define SM5701_INT3_BOOSTPOK	0x20
#define SM5701_INT3_AMSTMROFF	0x40
#define SM5701_INT3_LOWBATT		0x80
//INTMSK1
#define SM5701_INTMSK1_AICLM		0x01
#define SM5701_INTMSK1_BATOVPM		0x02
#define SM5701_INTMSK1_VBUSOKM		0x04
#define SM5701_INTMSK1_VBUSUVLOM	0x08
#define SM5701_INTMSK1_VBUSOVPM		0x10
#define SM5701_INTMSK1_VBUSLIMITM	0x20
#define SM5701_INTMSK1_CHGRSTFM		0x40
#define SM5701_INTMSK1_NOBATM		0x80
//INTMSK2
#define SM5701_INTMSK2_TOPOFFM		0x01
#define SM5701_INTMSK2_DONEM		0x02
#define SM5701_INTMSK2_FASTTMROFFM	0x04
#define SM5701_INTMSK2_CHGONM		0x08
#define SM5701_INTMSK2_OTGOCP		0x10
//INTMSK3
#define SM5701_INTMSK3_THEMSHDNM	0x01
#define SM5701_INTMSK3_THEMREGM		0x02
#define SM5701_INTMSK3_FLEDSHORTM	0x04
#define SM5701_INTMSK3_FLEDOPENM	0x08
#define SM5701_INTMSK3_BOOSTPOK_NGM	0x10
#define SM5701_INTMSK3_BOOSTPOKM	0x20
#define SM5701_INTMSK3_AMSTMROFFM	0x40
#define SM5701_INTMSK3_LOWBATTM		0x80
//STATUS1
#define SM5701_STATUS1_AICL_MASK			0x01
#define SM5701_STATUS1_AICL_SHIFT			0

#define SM5701_STATUS1_BATOVP_MASK			0x01
#define SM5701_STATUS1_BATOVP_SHIFT			1

#define SM5701_STATUS1_VBUSOK_MASK			0x01
#define SM5701_STATUS1_VBUSOK_SHIFT			2

#define SM5701_STATUS1_VBUSUVLO_MASK		0x01
#define SM5701_STATUS1_VBUSUVLO_SHIFT		3

#define SM5701_STATUS1_VBUSOVP_MASK			0x01
#define SM5701_STATUS1_VBUSOVP_SHIFT		4

#define SM5701_STATUS1_VBUSLIMIT_MASK		0x01
#define SM5701_STATUS1_VBUSLIMIT_SHIFT		5

#define SM5701_STATUS1_CHGRSTF_MASK			0x01
#define SM5701_STATUS1_CHGRSTF_SHIFT		6

#define SM5701_STATUS1_NOBAT_MASK			0x01
#define SM5701_STATUS1_NOBAT_SHIFT			7

//STATUS2
#define SM5701_STATUS2_TOPOFF_MASK			0x01
#define SM5701_STATUS2_TOPOFF_SHIFT			0

#define SM5701_STATUS2_DONE_MASK			0x01
#define SM5701_STATUS2_DONE_SHIFT			1

#define SM5701_STATUS2_FASTTMROFF_MASK		0x01
#define SM5701_STATUS2_FASTTMROFF_SHIFT		2

#define SM5701_STATUS2_CHGON_MASK			0x01
#define SM5701_STATUS2_CHGON_SHIFT			3

#define SM5701_STATUS2_OTGOCP_MASK			0x01
#define SM5701_STATUS2_OTGOCP_SHIFT			4

//STATUS3
#define SM5701_STATUS3_THEMSHDN_MASK	    0x01
#define SM5701_STATUS3_THEMSHDN_SHIFT	    0

#define SM5701_STATUS3_THEMREG_MASK		    0x02
#define SM5701_STATUS3_THEMREG_SHIFT		1

#define SM5701_STATUS3_FLEDSHORT_MASK		0x04
#define SM5701_STATUS3_FLEDSHORT_SHIFT		2

#define SM5701_STATUS3_FLEDOPEN_MASK		0x08
#define SM5701_STATUS3_FLEDOPEN_SHIFT		3

#define SM5701_STATUS3_BOOSTPOK_NG_MASK	    0x10
#define SM5701_STATUS3_BOOSTPOK_NG_SHIFT	4

#define SM5701_STATUS3_BOOSTPOK_MASK		0x20
#define SM5701_STATUS3_BOOSTPOK_SHIFT		5

#define SM5701_STATUS3_ABSTMROFF_MASK		0x40
#define SM5701_STATUS3_ABSTMROFF_SHIFT		6

#define SM5701_STATUS3_LOWBATT_MASK			0x80
#define SM5701_STATUS3_LOWBATT_SHIFT		7

//CNTL
#define SM5701_CNTL_OPERATIONMODE_MASK		0x07
#define SM5701_CNTL_OPERATIONMODE_SHIFT		0

#define SM5701_CNTL_RESET_MASK				0x01
#define SM5701_CNTL_RESET_SHIFT				3

#define SM5701_CNTL_OVPSEL_MASK				0x03
#define SM5701_CNTL_OVPSEL_SHIFT			4

#define SM5701_CNTL_FREQSEL_MASK			0x03
#define SM5701_CNTL_FREQSEL_SHIFT			6

//VBUSCNTL
#define SM5701_VBUSCNTL_VBUSLIMIT_MASK		0x07
#define SM5701_VBUSCNTL_VBUSLIMIT_SHIFT		0

#define SM5701_VBUSCNTL_AICLTH_MASK			0x07
#define SM5701_VBUSCNTL_AICLTH_SHIFT		3

#define SM5701_VBUSCNTL_AICLEN_MASK			0x01
#define SM5701_VBUSCNTL_AICLEN_SHIFT		6


//CHGCNTL1
#define SM5701_CHGCNTL1_TOPOFF_MASK         0x0F
#define SM5701_CHGCNTL1_TOPOFF_SHIFT        0

#define SM5701_CHGCNTL1_AUTOSTOP_MASK       0x01
#define SM5701_CHGCNTL1_AUTOSTOP_SHIFT      4

#define SM5701_CHGCNTL1_HSSLPCTRL_MASK      0x03
#define SM5701_CHGCNTL1_HSSLPCTRL_SHIFT     5


//CHGCNTL2
#define SM5701_CHGCNTL2_FASTCHG_MASK		0x3F
#define SM5701_CHGCNTL2_FASTCHG_SHIFT		0


//CHGCNTL3
#define SM5701_CHGCNTL3_BATREG_MASK			0x1F
#define SM5701_CHGCNTL3_BATREG_SHIFT		0


//CHGCNTL4
#define SM5701_CHGCNTL4_TOPOFFTIMER_MASK	0x03
#define SM5701_CHGCNTL4_TOPOFFTIMER_SHIFT	0

#define SM5701_CHGCNTL4_FASTTIMER_MASK		0x03
#define SM5701_CHGCNTL4_FASTTIMER_SHIFT		2


//FLEDCNTL1
#define SM5701_FLEDCNTL1_FLEDEN			0x03
#define SM5701_FLEDCNTL1_FLEDEN_MASK    0x03

#define SM5701_FLEDCNTL1_ABSTMR			0x0B
#define SM5701_FLEDCNTL1_ENABSTMR		0x10
//FLEDCNTL2
#define SM5701_FLEDCNTL2_ONETIMER		0x0F
#define SM5701_FLEDCNTL2_nONESHOT		0x10
#define SM5701_FLEDCNTL2_SAFET			0x60
#define SM5701_FLEDCNTL2_nENSAFET		0x80
//FLEDCNTL3
#define SM5701_FLEDCNTL3_IFLED			0x1F
//FLEDCNTL4
#define SM5701_FLEDCNTL4_IMLED			0x1F
//FLEDCNTL5
#define SM5701_FLEDCNTL5_LBDHYS			0x03
#define SM5701_FLEDCNTL5_LOWBATT		0x1B
#define SM5701_FLEDCNTL5_LBRSTIMER		0x60
#define SM5701_FLEDCNTL5_ENLOWBATT		0x80
//FLEDCNTL6
#define SM5701_FLEDCNTL6_BSTOUT			0x0F
#define SM5701_FLEDCNTL6_BSTOUT_MASK	0x0F
#define SM5701_FLEDCNTL6_BSTOUT_SHIFT	0
#ifdef LYCONFIG_COMB_SUB_FLASHLIGHT_TYPE_SM5701
#define SM5701_BSTOUT_4P5               0x00
#define SM5701_BSTOUT_5P0               0x00
#else
#define SM5701_BSTOUT_4P5               0x05
#define SM5701_BSTOUT_5P0               0x0A
#endif
enum fled_mode {
    FLASHLIGHT_MODE_OFF = 0,
    FLASHLIGHT_MODE_MOVIE,
    FLASHLIGHT_MODE_FLASH,
    FLASHLIGHT_MODE_MAX,
};

enum {
    TURN_WAY_I2C = 0,
    TURN_WAY_GPIO = 1,
};

#define SM5701_FLEDEN_DISABLE       0x00
#define SM5701_FLEDEN_MOVIE_MODE    0x01
#define SM5701_FLEDEN_FLASH_MODE    0x02
#define SM5701_FLEDEN_EXTERNAL      0x03


//CNTL
#define SM5701_OPERATIONMODE_SUSPEND                0x0 // 000 : Suspend (charger-OFF) MODE
#define SM5701_OPERATIONMODE_FLASH_ON               0x1 // 001 : Flash LED Driver=ON Ready in Charger & OTG OFF Mode
#define SM5701_OPERATIONMODE_OTG_ON                 0x2 // 010 : OTG=ON in Charger & Flash OFF Mode
#define SM5701_OPERATIONMODE_OTG_ON_FLASH_ON        0x3 // 011 : OTG=ON & Flash LED Driver=ON Ready in charger OFF Mode
#define SM5701_OPERATIONMODE_CHARGER_ON             0x4 // 100 : Charger=ON in OTG & Flash OFF Mode. Same as 0x6(110)
#define SM5701_OPERATIONMODE_CHARGER_ON_FLASH_ON    0x5 // 101 : Charger=ON & Flash LED Driver=ON Ready in OTG OFF Mode. Same as 0x7(111)

/**********************************************************
  *
  *   [Extern Function] 
  *
  *********************************************************/
//STATUS2----------------------------------------------------
extern unsigned int sm5701_chg_get_topoff_status(void);

//CNTL----------------------------------------------------
extern void sm5701_chg_set_operationmode(unsigned int val);
extern void sm5701_chg_set_reset(unsigned int val);
extern void sm5701_chg_set_ovpsel(unsigned int val);
extern void sm5701_chg_set_freqsel(unsigned int val);

//VBUSCNTL----------------------------------------------------
extern void sm5701_chg_set_vbuslimit(unsigned int val);
extern void sm5701_chg_set_aiclth(unsigned int val);
extern void sm5701_chg_set_aiclen(unsigned int val);

//CHGCNTL1----------------------------------------------------
extern void sm5701_chg_set_topoff(unsigned int val);
extern void sm5701_chg_set_autostop(unsigned int val);
extern void sm5701_chg_set_hsslpctrl(unsigned int val);

//CHGCNTL2----------------------------------------------------
extern void sm5701_chg_set_fastchg(unsigned int val);

//CHGCNTL3----------------------------------------------------
extern void sm5701_chg_set_batreg(unsigned int val);

//CHGCNTL4----------------------------------------------------
extern void sm5701_chg_set_topofftimer(unsigned int val);
extern void sm5701_chg_set_fasttimer(unsigned int val);

extern void sm5701_chg_set_bstout(unsigned int val);
//---------------------------------------------------------
extern int sm5701_fled_get_mode(void);
extern int sm5701_charger_get_chgmode(void);
extern int sm5701_charger_get_otgmode(void);

extern int sm5701_charger_notification(int on);
extern int sm5701_boost_notification(int on);

extern void sm5701_chg_dump_register(void);
extern unsigned int sm5701_chg_reg_config_interface (unsigned char RegNum, unsigned char val);

extern unsigned int sm5701_chg_read_interface (unsigned char RegNum, unsigned char *val, unsigned char MASK, unsigned char SHIFT);
extern unsigned int sm5701_chg_config_interface (unsigned char RegNum, unsigned char val, unsigned char MASK, unsigned char SHIFT);

#endif // _SM5701_SW_H_

