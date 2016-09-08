/*****************************************************************************
*
* Filename:
* ---------
*   nt30358_dcdc.h
*
* Project:
* --------
*   Android
*
* Description:
* ------------
*   nt30358_dcdc header file
*
* Author:added by xiehaifei 150623 for lcm power supply
* -------
*
****************************************************************************/
#ifndef _NT50358_DCDC_H_
#define _NT50358_DCDC_H_


#define LCM_USE_NT50358_DCDC
#define LCM_USE_DW8769




extern int nt50358_dcdc_set_avdd (char addr, char value);
extern int nt50358_dcdc_set_avee (char addr, char value);
extern int nt50358_dcdc_set_bost (char addr, char value);
extern int dw8769_set_discharge_status (char addr, char value);
extern int get_dcdc_type (void);
extern void lcm_gpio_output(int pin, int level);
#endif