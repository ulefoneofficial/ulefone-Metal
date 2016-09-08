#ifndef _CUST_BATTERY_METER_TABLE_H
#define _CUST_BATTERY_METER_TABLE_H

#if defined(LYCONFIG_COMB_CUST_PROJECT_NAME_F5B_JF)
	#include "mt_battery_meter_table_f5b_jf.h"
#elif defined(LYCONFIG_COMB_CUST_PROJECT_NAME_F5B_JH)
#if defined(LYCONFIG_COMB_CUST_PROJECT_SUB_NAME_F5B_JH_A)
	#include "mt_battery_meter_table_default_f5b_jh.h"
#else
	#include "mt_battery_meter_table_default_f5b_jh_c.h"
#endif	
#elif defined(LYCONFIG_COMB_CUST_PROJECT_NAME_F5C_JH)
#if defined(LYCONFIG_COMB_CUST_PROJECT_SUB_NAME_F5C_JH_D)
	#include "mt_battery_meter_table_default_f5b_jh.h"
#else
	#include "mt_battery_meter_table_default_f5b_jh_c.h"
#endif
#elif 1//defined(LYCONFIG_COMB_CUST_PROJECT_NAME_F5B_GQ_F)
	#include "mt_battery_meter_table_default_f5bp_gq.h"
#elif defined(LYCONFIG_COMB_CUST_PROJECT_NAME_F5B_FM) || defined(LYCONFIG_COMB_CUST_PROJECT_NAME_F5C_FM)
	#include "mt_battery_meter_table_default_f5b_fm.h"	
#elif defined(LYCONFIG_COMB_CUST_PROJECT_NAME_F1_YP)
	#if defined(LYCONFIG_COMB_CUST_PROJECT_SUB_NAME_F1_YP_B1) || defined(LYCONFIG_COMB_CUST_PROJECT_SUB_NAME_F1_YP_B2)
	#include "mt_battery_meter_table_default_f1_yp_b1.h"
	#else
	#include "mt_battery_meter_table_default_f1_yp.h"	
	#endif
#else
	#include "mt_battery_meter_table_default.h"
#endif

#endif	/*#ifndef _CUST_BATTERY_METER_TABLE_H*/

