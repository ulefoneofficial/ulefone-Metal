#ifndef _CUST_BATTERY_METER_H
#define _CUST_BATTERY_METER_H

#define MTK_SM5414_SUPPORT

#if defined(MTK_SM5414_SUPPORT)
#define LYCONFIG_COMB_CHARGER_IC_MTK_SM5414_SUPPORT
#elif defined(MTK_SM5701_SUPPORT)
#define LYCONFIG_COMB_CHARGER_IC_MTK_SM5701_SUPPORT
#elif defined(MTK_FAN5405_SUPPORT)
#define LYCONFIG_COMB_CHARGER_IC_MTK_FAN5405_SUPPORT
#endif

#if defined(LYCONFIG_COMB_CUST_PROJECT_NAME_F5B_JF)
	#include "mt_battery_meter_f5b_jf.h"
#elif defined(LYCONFIG_COMB_CUST_PROJECT_NAME_F5B_JH)
#if defined(LYCONFIG_COMB_CUST_PROJECT_SUB_NAME_F5B_JH_A)
	#include "mt_battery_meter_default_f5b_jh.h"
#else
	#include "mt_battery_meter_default_f5b_jh_c.h"
#endif	
#elif defined(LYCONFIG_COMB_CUST_PROJECT_NAME_F5C_JH)
#if defined(LYCONFIG_COMB_CUST_PROJECT_SUB_NAME_F5C_JH_D)
	#include "mt_battery_meter_default_f5b_jh.h"
#else
	#include "mt_battery_meter_default_f5b_jh_c.h"
#endif
#elif defined(LYCONFIG_COMB_CUST_PROJECT_NAME_F5B_FM) || defined(LYCONFIG_COMB_CUST_PROJECT_NAME_F5C_FM)
	#include "mt_battery_meter_default_f5b_fm.h"
#elif 1//defined(LYCONFIG_COMB_CUST_PROJECT_NAME_F5B_GQ_F) || defined(LYCONFIG_COMB_CUST_PROJECT_NAME_F5C_GQ)
	#include "mt_battery_meter_default_f5bp_gq.h"
#elif defined(LYCONFIG_COMB_CUST_PROJECT_NAME_F1C_GQ)
	#include "mt_battery_meter_default_f1c_gq.h"
#elif defined(LYCONFIG_COMB_CUST_PROJECT_NAME_F1_YP)
	#if defined(LYCONFIG_COMB_CUST_PROJECT_SUB_NAME_F1_YP_B1)
	#include "mt_battery_meter_default_f1_yp_b1.h"
	#else
	#include "mt_battery_meter_default_f1_yp.h"
	#endif
#elif defined(LYCONFIG_COMB_CUST_PROJECT_NAME_FACTORY)
	#include "mt_battery_meter_default.h"	
#else
	#include "mt_battery_meter_default.h"
	//#error "PLEASE CONFIG LYCONFIG_COMB_CUST_PROJECT_NAME RIGHT!"
#endif

#if defined(CAR_TUNE_VALUE)
#undef CAR_TUNE_VALUE
#endif

#if defined(LYCONFIG_AUTO_PLATFORM_NAME_F5B)  || defined(LYCONFIG_AUTO_PLATFORM_NAME_F5C) 
	#if defined(LYCONFIG_COMB_CUST_PROJECT_NAME_F5B_FM) 
	#define CAR_TUNE_VALUE		75
	#else
	#define CAR_TUNE_VALUE		85
	#endif
#elif defined(LYCONFIG_AUTO_PLATFORM_NAME_F1C)
	#define CAR_TUNE_VALUE		88
#else
	#define CAR_TUNE_VALUE		85
#endif

#define DISABLE_RFG_EXIST_CHECK

#endif	/*#ifndef _CUST_BATTERY_METER_H*/
