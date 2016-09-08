/* 
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
/*
 * Definitions for em3071 als/ps sensor chip.
 */
#ifndef __EM3071_H__
#define __EM3071_H__

#include <linux/ioctl.h>

#define EM3071_CMM_ID		0X00
#define EM3071_CMM_ENABLE 	0X01
#define EM3071_CMM_STATUS	0x02
#define EM3071_CMM_INT_PS_LT  	0X03
#define EM3071_CMM_INT_PS_HT  	0X04
#define EM3071_CMM_PDATA_L 	0X08
#define EM3071_CMM_C0DATA_L 	0X09
#define EM3071_CMM_C0DATA_H 	0X0A
#define EM3071_CMM_OFFSET	0X0F

#define EM3071_SUCCESS						0
#define EM3071_ERR_I2C						-1


enum EM3071_NOTIFY_TYPE {
	EM3071_NOTIFY_PROXIMITY_CHANGE = 1,
};

//static int em30713_read(struct i2c_client *client,u8 reg);
//static int em30713_write(struct i2c_client *client,u8 reg,u8 data);
//static int dev_attr_config(u8 reg,int config,int dev_s,int data);
#endif
