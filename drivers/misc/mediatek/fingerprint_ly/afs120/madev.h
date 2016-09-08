#ifndef MADEV_H
#define MADEV_H

#define MTK // MTK平台
#include "../../../../../../resource/LyRoConfigUtils.h"
#ifdef MTK

#include <linux/delay.h>
#include <linux/spi/spi.h>
#include <linux/cdev.h>
#include <linux/types.h>

//#include <mach/mt_pm_ldo.h>
#include <mach/mt_spi.h>
#include "cust_gpio_usage.h"
#include <mach/mt_gpio.h>

	#define FBUF  (1024*15)	// 帧大小
#else 	// 三星、高通平台
	#define W 		120		// 宽
	#define H 		120		// 高
	#define FBUF  	((W+1)*(H+1)+1) // 帧大小
#endif


#define MA_READ_FRAME_REG 	0x78

#define IOCTL_DEBUG	0x100	// 驱动信息
#define IOCTL_IRQEN	0x101	// 驱动使能
#define IOCTL_SPEED	0x102	// SPI速度
#define IOCTL_RDLEN	0x103	// 读长度
#define IOCTL_LINKD	0x104	// 连接设备
#define IOCTL_STNUM	0x105   // 材料编号
#define IOCTL_CPARA	0x106	// 检查参数
#define IOCTL_VDATE	0x107	// 版本日期

#define STUFF_NUM  LYCONFIG_FINGERPRINT_SNUM  // 材料编号: 1陶瓷，2塑封

#ifndef msleep
#define msleep(n) 	\
do { 	\
long timeout = (n) * HZ / 1000; \
while(timeout > 0) \
{ 	\
timeout = schedule_timeout(timeout); \
} 	\
}while(0);
#endif

#endif /* SPIDEV_H */
