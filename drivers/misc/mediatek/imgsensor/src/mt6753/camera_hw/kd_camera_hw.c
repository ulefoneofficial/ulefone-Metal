#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>

#include "kd_camera_typedef.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_camera_feature.h"
#include "kd_camera_hw.h"
#include "upmu_common.h"
/******************************************************************************
 * Debug configuration
******************************************************************************/
#define PFX "[kd_camera_hw]"
#define PK_DBG_NONE(fmt, arg...)    do {} while (0)
#define PK_DBG_FUNC(fmt, args...)    pr_debug(PFX  fmt, ##args)

#define DEBUG_CAMERA_HW_K
#ifdef DEBUG_CAMERA_HW_K
#define PK_DBG(fmt, args...) pr_debug(PFX  fmt, ##args)
#define PK_ERR(fmt, arg...)         pr_err(fmt, ##arg)
#define PK_XLOG_INFO(fmt, args...)  pr_debug(PFX  fmt, ##args);
#else
#define PK_DBG(a, ...)
#define PK_ERR(a, ...)
#define PK_XLOG_INFO(fmt, args...)
#endif
//ly_xinchao add for camera start

#define VOL_2800 2800000
#define VOL_1800 1800000
#define VOL_1500 1500000
#define VOL_1200 1200000
#define VOL_1000 1000000

#define IDX_PS_CMRST 0
#define IDX_PS_CMPDN 4
#define IDX_PS_MODE 1
#define IDX_PS_ON   2
#define IDX_PS_OFF  3

#define GPIO_HIGH_LEVEL 1
#define GPIO_LOW_LEVEL 	0

#define LYCONFIG_DUAL_CAMERA_BOTH_WORK_SUPPORT
#define LYCONFIG_EXCHANGE_BACK_FRONT_CAMERA_SUPPORT
//#define LYCONFIG_DUAL_CAMERA_USE_CLK1_SUPPORT
#define LYCONFIG_DUAL_CAMERA_USE_VCAMA_SUPPORT


CAMERA_POWER_INFO camera_d_vol_table[] = 
{
	//{SENSOR_DRVNAME_IMX145_MIPI_RAW, VOL_1200, GPIO_HIGH_LEVEL},
	{SENSOR_DRVNAME_IMX149_MIPI_RAW, VOL_1200, GPIO_HIGH_LEVEL},
	//{SENSOR_DRVNAME_IMX135_MIPI_RAW, VOL_1000, GPIO_HIGH_LEVEL},
	//{SENSOR_DRVNAME_IMX164_MIPI_RAW, VOL_1200, GPIO_HIGH_LEVEL},
	{SENSOR_DRVNAME_IMX214_MIPI_RAW, VOL_1000, GPIO_HIGH_LEVEL},
	{SENSOR_DRVNAME_IMX258_MIPI_RAW, VOL_1200, GPIO_HIGH_LEVEL},
	//{SENSOR_DRVNAME_IMX220_MIPI_RAW, VOL_1000, GPIO_HIGH_LEVEL},
	{SENSOR_DRVNAME_IMX219_MIPI_RAW, VOL_1200, GPIO_HIGH_LEVEL},
	{SENSOR_DRVNAME_IMX166_MIPI_RAW, VOL_1200, GPIO_HIGH_LEVEL},
	{SENSOR_DRVNAME_OV5648_MIPI_RAW, VOL_1500, GPIO_HIGH_LEVEL},
	{SENSOR_DRVNAME_OV5670_MIPI_RAW, VOL_1500, GPIO_HIGH_LEVEL},
	{SENSOR_DRVNAME_OV13850_MIPI_RAW, VOL_1200, GPIO_HIGH_LEVEL},
	{SENSOR_DRVNAME_OV9760_MIPI_RAW, VOL_1200, GPIO_HIGH_LEVEL},
	{SENSOR_DRVNAME_OV9762_MIPI_RAW, VOL_1200, GPIO_HIGH_LEVEL},
	{SENSOR_DRVNAME_HI551_MIPI_RAW, VOL_1800, GPIO_HIGH_LEVEL},
	//{SENSOR_DRVNAME_HI841_MIPI_RAW, VOL_1500, GPIO_HIGH_LEVEL},
	{SENSOR_DRVNAME_MN34152_MIPI_RAW, VOL_1200, GPIO_HIGH_LEVEL},
	//{SENSOR_DRVNAME_SP8408_MIPI_RAW, VOL_1200, GPIO_HIGH_LEVEL},	
	//{SENSOR_DRVNAME_GC2355_MIPI_RAW, VOL_1800, GPIO_LOW_LEVEL},
	{SENSOR_DRVNAME_GC2755_MIPI_RAW, VOL_1800, GPIO_LOW_LEVEL},
	{SENSOR_DRVNAME_GC5024_MIPI_RAW, VOL_1500, GPIO_LOW_LEVEL},
	//{SENSOR_DRVNAME_GC5005_MIPI_RAW, VOL_1200, GPIO_LOW_LEVEL},
	//{SENSOR_DRVNAME_GC8003_MIPI_RAW, VOL_1500, GPIO_LOW_LEVEL},
	{SENSOR_DRVNAME_GC8024_MIPI_RAW, VOL_1200, GPIO_LOW_LEVEL},
	//{SENSOR_DRVNAME_S5K2P8_MIPI_RAW, VOL_1200, GPIO_HIGH_LEVEL},
	{SENSOR_DRVNAME_S5K3L2_MIPI_RAW, VOL_1200, GPIO_HIGH_LEVEL},
	{SENSOR_DRVNAME_S5K4H5YC_MIPI_RAW, VOL_1200, GPIO_HIGH_LEVEL},
	{SENSOR_DRVNAME_S5K5E2YA_MIPI_RAW, VOL_1200, GPIO_HIGH_LEVEL},
	{SENSOR_DRVNAME_S5K3H5XA_MIPI_RAW, VOL_1200, GPIO_HIGH_LEVEL},
};

static int kd_find_camera_digit_vol(char *sensor_name)
{
	int i = 0;

	for (i=0; i<sizeof(camera_d_vol_table)/sizeof(camera_d_vol_table[0]); i++)
	{
		if (!strcmp(camera_d_vol_table[i].camera_name, sensor_name))
		{
			return camera_d_vol_table[i].digit_vol;
		}
	}
	//defautl 1.5V
	return VOL_1500;
}

static int kd_find_camera_pwd_valid_state(char *sensor_name)
{
	int i = 0;

	int pwd_state = IDX_PS_OFF;

	for (i=0; i<sizeof(camera_d_vol_table)/sizeof(camera_d_vol_table[0]); i++)
	{
		if (!strcmp(camera_d_vol_table[i].camera_name, sensor_name))
		{
			if (camera_d_vol_table[i].pwd_valid_state == GPIO_HIGH_LEVEL)
			{
				pwd_state = IDX_PS_ON;
			}
			else
			{
				pwd_state = IDX_PS_OFF;
			}
			break;
		}
	}
	
	return pwd_state;
}


static int kd_find_camera_pwd_off_state(char *sensor_name)
{
	int i = 0;

	int pwd_off_state = IDX_PS_OFF;

	for (i=0; i<sizeof(camera_d_vol_table)/sizeof(camera_d_vol_table[0]); i++)
	{
		if (!strcmp(camera_d_vol_table[i].camera_name, sensor_name))
		{
			if (camera_d_vol_table[i].pwd_valid_state == GPIO_HIGH_LEVEL)
			{
				pwd_off_state = IDX_PS_OFF;
			}
			else
			{
				pwd_off_state = IDX_PS_ON;
			}
			break;
		}
	}
	
	return pwd_off_state;
}
//ly_xinchao add for camera end


#if !defined(CONFIG_MTK_LEGACY)
/* GPIO Pin control*/
struct platform_device *cam_plt_dev = NULL;
struct pinctrl *camctrl = NULL;
struct pinctrl_state *cam0_pnd_h = NULL;
struct pinctrl_state *cam0_pnd_l = NULL;
struct pinctrl_state *cam0_rst_h = NULL;
struct pinctrl_state *cam0_rst_l = NULL;
struct pinctrl_state *cam1_pnd_h = NULL;
struct pinctrl_state *cam1_pnd_l = NULL;
struct pinctrl_state *cam1_rst_h = NULL;
struct pinctrl_state *cam1_rst_l = NULL;
struct pinctrl_state *cam_ldo0_h = NULL;
struct pinctrl_state *cam_ldo0_l = NULL;
struct pinctrl_state *cam1_avdd_h = NULL;
struct pinctrl_state *cam1_avdd_l = NULL;
struct pinctrl_state *cam1_pnd1_h = NULL;
struct pinctrl_state *cam1_pnd1_l = NULL;

int mtkcam_gpio_init(struct platform_device *pdev)
{
	int ret = 0;

	camctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(camctrl)) {
		dev_err(&pdev->dev, "Cannot find camera pinctrl!");
		ret = PTR_ERR(camctrl);
	}
    /*Cam0 Power/Rst Ping initialization*/
	cam0_pnd_h = pinctrl_lookup_state(camctrl, "cam0_pnd1");
	if (IS_ERR(cam0_pnd_h)) {
		ret = PTR_ERR(cam0_pnd_h);
		pr_debug("%s : pinctrl err, cam0_pnd_h\n", __func__);
	}

	cam0_pnd_l = pinctrl_lookup_state(camctrl, "cam0_pnd0");
	if (IS_ERR(cam0_pnd_l)) {
		ret = PTR_ERR(cam0_pnd_l);
		pr_debug("%s : pinctrl err, cam0_pnd_l\n", __func__);
	}


	cam0_rst_h = pinctrl_lookup_state(camctrl, "cam0_rst1");
	if (IS_ERR(cam0_rst_h)) {
		ret = PTR_ERR(cam0_rst_h);
		pr_debug("%s : pinctrl err, cam0_rst_h\n", __func__);
	}

	cam0_rst_l = pinctrl_lookup_state(camctrl, "cam0_rst0");
	if (IS_ERR(cam0_rst_l)) {
		ret = PTR_ERR(cam0_rst_l);
		pr_debug("%s : pinctrl err, cam0_rst_l\n", __func__);
	}

    /*Cam1 Power/Rst Ping initialization*/
	cam1_pnd_h = pinctrl_lookup_state(camctrl, "cam1_pnd1");
	if (IS_ERR(cam1_pnd_h)) {
		ret = PTR_ERR(cam1_pnd_h);
		pr_debug("%s : pinctrl err, cam1_pnd_h\n", __func__);
	}

	cam1_pnd_l = pinctrl_lookup_state(camctrl, "cam1_pnd0");
	if (IS_ERR(cam1_pnd_l )) {
		ret = PTR_ERR(cam1_pnd_l );
		pr_debug("%s : pinctrl err, cam1_pnd_l\n", __func__);
	}


	cam1_rst_h = pinctrl_lookup_state(camctrl, "cam1_rst1");
	if (IS_ERR(cam1_rst_h)) {
		ret = PTR_ERR(cam1_rst_h);
		pr_debug("%s : pinctrl err, cam1_rst_h\n", __func__);
	}


	cam1_rst_l = pinctrl_lookup_state(camctrl, "cam1_rst0");
	if (IS_ERR(cam1_rst_l)) {
		ret = PTR_ERR(cam1_rst_l);
		pr_debug("%s : pinctrl err, cam1_rst_l\n", __func__);
	}
	/*externel LDO enable */
	cam_ldo0_h = pinctrl_lookup_state(camctrl, "cam_ldo0_1");
	if (IS_ERR(cam_ldo0_h)) {
		ret = PTR_ERR(cam_ldo0_h);
		pr_debug("%s : pinctrl err, cam_ldo0_h\n", __func__);
	}


	cam_ldo0_l = pinctrl_lookup_state(camctrl, "cam_ldo0_0");
	if (IS_ERR(cam_ldo0_l)) {
		ret = PTR_ERR(cam_ldo0_l);
		pr_debug("%s : pinctrl err, cam_ldo0_l\n", __func__);
	}

	/*sub camera dvdd */
	cam1_avdd_h = pinctrl_lookup_state(camctrl, "cam1_avdd_1");
	if (IS_ERR(cam1_avdd_h)) {
		ret = PTR_ERR(cam1_avdd_h);
		pr_debug("%s : pinctrl err, cam1_avdd_h\n", __func__);
	}


	cam1_avdd_l = pinctrl_lookup_state(camctrl, "cam1_avdd_0");
	if (IS_ERR(cam1_avdd_l)) {
		ret = PTR_ERR(cam1_avdd_l);
		pr_debug("%s : pinctrl err, cam1_avdd_l\n", __func__);
	}

	  /*Cam1 Pnd1 Ping initialization*/
	cam1_pnd1_h = pinctrl_lookup_state(camctrl, "cam1_pnd1_1");
	if (IS_ERR(cam1_pnd1_h)) {
		ret = PTR_ERR(cam1_pnd1_h);
		pr_debug("%s : pinctrl err, cam1_pnd1_h\n", __func__);
	}

	cam1_pnd1_l = pinctrl_lookup_state(camctrl, "cam1_pnd1_0");
	if (IS_ERR(cam1_pnd1_l )) {
		ret = PTR_ERR(cam1_pnd1_l );
		pr_debug("%s : pinctrl err, cam1_pnd1_l\n", __func__);
	}
	
	return ret;
}

int mtkcam_gpio_set(int PinIdx, int PwrType, int Val)
{
	int ret = 0;
	switch (PwrType) {
	case CAMRST:
		#if defined(LYCONFIG_EXCHANGE_BACK_FRONT_CAMERA_SUPPORT)
		if (PinIdx == 0) {
			if (Val == 0)
				pinctrl_select_state(camctrl, cam1_rst_l);
			else
				pinctrl_select_state(camctrl, cam1_rst_h);
		} else {
			if (Val == 0)
				pinctrl_select_state(camctrl, cam0_rst_l);
			else
				pinctrl_select_state(camctrl, cam0_rst_h);
		}
		#else
		if (PinIdx == 0) {
			if (Val == 0)
				pinctrl_select_state(camctrl, cam0_rst_l);
			else
				pinctrl_select_state(camctrl, cam0_rst_h);
		} else {
			if (Val == 0)
				pinctrl_select_state(camctrl, cam1_rst_l);
			else
				pinctrl_select_state(camctrl, cam1_rst_h);
		}
		#endif
		break;
	case CAMPDN:
		#if defined(LYCONFIG_EXCHANGE_BACK_FRONT_CAMERA_SUPPORT)
		if (PinIdx == 0) {
			if (Val == 0)
			{
				#ifdef LYCONFIG_AUTO_PLATFORM_NAME_F9B
				pinctrl_select_state(camctrl, cam1_pnd1_l);
				#else
				pinctrl_select_state(camctrl, cam1_pnd_l);
				#endif
			}
			else
			{
				#ifdef LYCONFIG_AUTO_PLATFORM_NAME_F9B
				pinctrl_select_state(camctrl, cam1_pnd1_h);
				#else
				pinctrl_select_state(camctrl, cam1_pnd_h);
				#endif
			}
		} else {
			if (Val == 0)
				pinctrl_select_state(camctrl, cam0_pnd_l);
			else
				pinctrl_select_state(camctrl, cam0_pnd_h);
		}
		#else
		if (PinIdx == 0) {
			if (Val == 0)
				pinctrl_select_state(camctrl, cam0_pnd_l);
			else
				pinctrl_select_state(camctrl, cam0_pnd_h);
		} else {
			if (Val == 0)
			{
				#ifdef LYCONFIG_AUTO_PLATFORM_NAME_F9B
				pinctrl_select_state(camctrl, cam1_pnd1_l);
				#else
				pinctrl_select_state(camctrl, cam1_pnd_l);
				#endif
			}
			else
			{
				#ifdef LYCONFIG_AUTO_PLATFORM_NAME_F9B
				pinctrl_select_state(camctrl, cam1_pnd1_h);
				#else
				pinctrl_select_state(camctrl, cam1_pnd_h);
				#endif
			}
		}
		#endif
		break;
	case CAMLDO:
		if (Val == 0)
			pinctrl_select_state(camctrl, cam_ldo0_l);
		else
			pinctrl_select_state(camctrl, cam_ldo0_h);
		break;
	case CAM1AVDD:
		if (Val == 0)
			pinctrl_select_state(camctrl, cam1_avdd_l);
		else
			pinctrl_select_state(camctrl, cam1_avdd_h);
		break;	
	default:
		PK_DBG("PwrType(%d) is invalid !!\n", PwrType);
		break;
	};

	PK_DBG("PinIdx(%d) PwrType(%d) val(%d)\n", PinIdx, PwrType, Val);

	return ret;
}

/*added by xiehaifei for camera pip config*/
/********LINGYANG**CAMERA**CONFIG**PIP**************/
/*LDO*//*MAIN*//*SUB*/
/*DVDD*//*VCAMD*//*VRF18_1*/
/*AVDD*//*VCAMA*//*VSUBCAM*/
/*IOVDD*//*VCAMIO*//*VCAMIO*/
/*AFVDD*//*VCAMAF*//*VCAMAF*/
/*CLK*//*CLK1*//*CLK2*/
/*LYCONFIG_DUAL_CAMERA_USE_CLK1_SUPPORT=no*/
/*LYCONFIG_DUAL_CAMERA_USE_VCAMA_SUPPORT=no*/
/********LINGYANG**CAMERA**CONFIG**NONPIP***********/
/*LDO*//*MAIN*//*SUB*/
/*DVDD*//*VCAMD*//*VRF18_1*/
/*AVDD*//*VCAMA*//*VCAMA*/
/*IOVDD*//*VCAMIO*//*VCAMIO*/
/*AFVDD*//*VCAMAF*//*VCAMAF*/
/*CLK*//*CLK1*//*CLK2*/
/*LYCONFIG_DUAL_CAMERA_USE_CLK1_SUPPORT=no*/
/*LYCONFIG_DUAL_CAMERA_USE_VCAMA_SUPPORT=yes*/
/*********F5B_GQ F5B13E**CAMERA**CONFIG**NONPIP***********/
/*LDO*//*MAIN*//*SUB*/
/*DVDD*//*VCAMD*//*VRF18_1*/
/*AVDD*//*VCAMA*//*VCAMA*/
/*IOVDD*//*VCAMIO*//*VCAMIO*/
/*AFVDD*//*VCAMAF*//*VCAMAF*/
/*CLK*//*CLK1*//*CLK1*/
/*LYCONFIG_DUAL_CAMERA_USE_CLK1_SUPPORT=yes*/
/*LYCONFIG_DUAL_CAMERA_USE_VCAMA_SUPPORT=yes*/
/********LINGYANG**CAMERA**CONFIG**END*************/

int kdCISModulePowerOn(CAMERA_DUAL_CAMERA_SENSOR_ENUM SensorIdx, char *currSensorName, bool On, char *mode_name)
{

	u32 pinSetIdx = 0;/* default main sensor */

	int d_vol = VOL_1500;
	int ps_state = IDX_PS_ON;
	int ps_off_state = IDX_PS_OFF;


	u32 pinSet[3][8] = {
		/* for main sensor */
		{/* The reset pin of main sensor uses GPIO10 of mt6306, please call mt6306 API to set */
			CAMERA_CMRST_PIN,
			CAMERA_CMRST_PIN_M_GPIO,   /* mode */
			GPIO_OUT_ONE,              /* ON state */
			GPIO_OUT_ZERO,             /* OFF state */
			CAMERA_CMPDN_PIN,
			CAMERA_CMPDN_PIN_M_GPIO,
			GPIO_OUT_ONE,
			GPIO_OUT_ZERO,
		},
		/* for sub sensor */
		{
			CAMERA_CMRST1_PIN,
			CAMERA_CMRST1_PIN_M_GPIO,
			GPIO_OUT_ONE,
			GPIO_OUT_ZERO,
			CAMERA_CMPDN1_PIN,
			CAMERA_CMPDN1_PIN_M_GPIO,
			GPIO_OUT_ONE,
			GPIO_OUT_ZERO,
		},
		/* for main_2 sensor */
		{
			GPIO_CAMERA_INVALID,
			GPIO_CAMERA_INVALID,   /* mode */
			GPIO_OUT_ONE,               /* ON state */
			GPIO_OUT_ZERO,              /* OFF state */
			GPIO_CAMERA_INVALID,
			GPIO_CAMERA_INVALID,
			GPIO_OUT_ONE,
			GPIO_OUT_ZERO,
		}
	};

	if (DUAL_CAMERA_MAIN_SENSOR == SensorIdx)
		pinSetIdx = 0;
	else if (DUAL_CAMERA_SUB_SENSOR == SensorIdx)
		pinSetIdx = 1;
	else if (DUAL_CAMERA_MAIN_2_SENSOR == SensorIdx)
		pinSetIdx = 2;

	//ly_xinchao add for camera start
	d_vol = kd_find_camera_digit_vol(currSensorName);

	ps_state = kd_find_camera_pwd_valid_state(currSensorName);

	ps_off_state = kd_find_camera_pwd_off_state(currSensorName);

	PK_DBG("kdCISModulePowerOn SensorIdx =%d currSensorName=%s d_vol=%d ps_state=%d \n", SensorIdx,currSensorName, d_vol, ps_state);
	printk("kdCISModulePowerOn SensorIdx =%d currSensorName=%s d_vol=%d ps_state=%d  On=%d\n", SensorIdx,currSensorName, d_vol, ps_state,On);
	//ly_xinchao add for camera end

	switch (SensorIdx)
	{
		case DUAL_CAMERA_MAIN_SENSOR:
			PK_DBG("[DUAL_CAMERA_MAIN_SENSOR] on = %d\n", On);
			
		 	if (On)
			{
				#if defined(LYCONFIG_DUAL_CAMERA_BOTH_WORK_SUPPORT)
					#if defined(LYCONFIG_DUAL_CAMERA_USE_CLK1_SUPPORT)
					ISP_MCLK1_EN(1);
					#else	
						#if defined(LYCONFIG_EXCHANGE_BACK_FRONT_CAMERA_SUPPORT)
						ISP_MCLK2_EN(1);
						#else
						ISP_MCLK1_EN(1);
						#endif
					#endif	
				#else
				ISP_MCLK1_EN(1);
				#endif

				/* First Power Pin low and Reset Pin Low */
				if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMPDN]) {
					mtkcam_gpio_set(pinSetIdx, CAMPDN,
							pinSet[pinSetIdx][IDX_PS_CMPDN + IDX_PS_OFF]);
				}

				if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST]) {
					mtkcam_gpio_set(pinSetIdx, CAMRST,
							pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_OFF]);
				}

				/* VCAM_IO */
				if (TRUE != _hwPowerOn(VCAMIO, VOL_1800)) {
					PK_DBG("[CAMERA SENSOR] Fail to enable IO power (VCAM_IO), power id = %d\n", VCAMIO);
					goto _kdCISModulePowerOn_exit_;
				}
				mdelay(1);

				//VCAM_A
				#if defined(LYCONFIG_DUAL_CAMERA_USE_VCAMA_SUPPORT)
				if (TRUE != _hwPowerOn(VCAMA, VOL_2800)) {
					PK_DBG("[CAMERA SENSOR] Fail to enable analog power (VCAM_A), power id = %d\n", VCAMA);
					goto _kdCISModulePowerOn_exit_;
				}
				#else
					#if defined(LYCONFIG_EXCHANGE_BACK_FRONT_CAMERA_SUPPORT)
					mtkcam_gpio_set(pinSetIdx, CAM1AVDD,1);
					#else
					if (TRUE != _hwPowerOn(VCAMA, VOL_2800)) {
						PK_DBG("[CAMERA SENSOR] Fail to enable analog power (VCAM_A), power id = %d\n", VCAMA);
						goto _kdCISModulePowerOn_exit_;
					}
					#endif
				#endif
				mdelay(1);

				//VCAM_D
				#if defined(LYCONFIG_EXCHANGE_BACK_FRONT_CAMERA_SUPPORT)
				 if (d_vol == VOL_1000){
                                       d_vol=VOL_1200;
                                       pmic_set_register_value(PMIC_RG_VRF18_1_CAL, 7);//-120mv
                               }else {
                                       pmic_set_register_value(PMIC_RG_VRF18_1_CAL, 0);
                               }

				if (TRUE != _hwPowerOn(VSUBCAMD, d_vol)) {
						PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
						goto _kdCISModulePowerOn_exit_;
					}
				#else
				if (TRUE != _hwPowerOn(VCAMD, d_vol)) {
					PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
					goto _kdCISModulePowerOn_exit_;
				}
				#endif
				mdelay(5);
				
				/* AF_VCC */
				if (TRUE != _hwPowerOn(VCAMAF, VOL_2800)) {
					PK_DBG("[CAMERA SENSOR] Fail to enable AF power (VCAM_AF), power id = %d\n", VCAMAF);
					goto _kdCISModulePowerOn_exit_;
				}
				
				/* enable active sensor */
				if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMPDN])
					mtkcam_gpio_set(pinSetIdx, CAMPDN, pinSet[pinSetIdx][IDX_PS_CMPDN + ps_state]);
				 mdelay(2);
				 
				if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST])
				mtkcam_gpio_set(pinSetIdx, CAMRST, pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_ON]);

			    	mdelay(20);

			}
			else	//off
			{
				#if defined(LYCONFIG_DUAL_CAMERA_BOTH_WORK_SUPPORT)
					#if defined(LYCONFIG_DUAL_CAMERA_USE_CLK1_SUPPORT)
					ISP_MCLK1_EN(0);
					#else
						#if defined(LYCONFIG_EXCHANGE_BACK_FRONT_CAMERA_SUPPORT)
						ISP_MCLK2_EN(0);
						#else
						ISP_MCLK1_EN(0);
						#endif
					#endif
				#else
				ISP_MCLK1_EN(0);
				#endif

					
				/* Set Power Pin low and Reset Pin Low */
				if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMPDN])
					mtkcam_gpio_set(pinSetIdx, CAMPDN, pinSet[pinSetIdx][IDX_PS_CMPDN + ps_off_state]);

				/* Set Reset Pin Low */
				if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST])
					mtkcam_gpio_set(pinSetIdx, CAMRST, pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_OFF]);

			
			    	/* AF_VCC */
				//added by xiehaifei 20160623 for imx149 i2c case the switch charge ic i2c error!	
				#if defined(IMX149_MIPI_RAW_GQ)
				if (currSensorName && (0 == strcmp(SENSOR_DRVNAME_IMX149_MIPI_RAW, currSensorName))){
					printk("IMX149 VCAMAF don't powerdown!\n");
				}else
				#endif
				//ended by xiehaifei
				if (TRUE != _hwPowerDown(VCAMAF)) {
						PK_DBG("[CAMERA SENSOR] Fail to OFF AF power (VCAM_AF),power id = %d\n", VCAMAF);
						/* return -EIO; */
						goto _kdCISModulePowerOn_exit_;
				}
			       
				
				/* VCAM_IO */
				if (TRUE != _hwPowerDown(VCAMIO)) {
					PK_DBG("[CAMERA SENSOR] Fail to OFF digital power (VCAM_IO), power id = %d\n", VCAMIO);
					/* return -EIO; */
					goto _kdCISModulePowerOn_exit_;
				}
				
				
				//VCAM_D
				#if defined(LYCONFIG_EXCHANGE_BACK_FRONT_CAMERA_SUPPORT)
				if(TRUE != _hwPowerDown(VSUBCAMD))
					{
						PK_DBG("[CAMERA SENSOR] Fail to OFF core power (VCAM_D), power id = %d \n",VCAMD);
						goto _kdCISModulePowerOn_exit_;
					}
				#else
					if (TRUE != _hwPowerDown(VCAMD)) {
					PK_DBG("[CAMERA SENSOR] Fail to OFF core power (VCAM_D),power id = %d\n", VCAMD);
					goto _kdCISModulePowerOn_exit_;
				}
				#endif

			   	 //VCAM_A
				#if defined(LYCONFIG_DUAL_CAMERA_USE_VCAMA_SUPPORT)
				if (TRUE != _hwPowerDown(VCAMA)) {
					PK_DBG("[CAMERA SENSOR] Fail to OFF analog power (VCAM_A),power id= (%d)\n", VCAMA);
					/* return -EIO; */
					goto _kdCISModulePowerOn_exit_;
				}
				#else
					#if defined(LYCONFIG_EXCHANGE_BACK_FRONT_CAMERA_SUPPORT)
					mtkcam_gpio_set(pinSetIdx, CAM1AVDD,0);
					#else
					if (TRUE != _hwPowerDown(VCAMA)) {
						PK_DBG("[CAMERA SENSOR] Fail to OFF analog power (VCAM_A),power id= (%d)\n", VCAMA);
						/* return -EIO; */
						goto _kdCISModulePowerOn_exit_;
					}
					#endif
				#endif
			}
		
			break;
		case DUAL_CAMERA_SUB_SENSOR:
			if (On)
			{
				#if defined(LYCONFIG_DUAL_CAMERA_BOTH_WORK_SUPPORT)
					#if defined(LYCONFIG_DUAL_CAMERA_USE_CLK1_SUPPORT)
					    ISP_MCLK1_EN(1);
                        		#else  
					    #if defined(LYCONFIG_EXCHANGE_BACK_FRONT_CAMERA_SUPPORT)
						ISP_MCLK1_EN(1);
					    #else
						ISP_MCLK2_EN(1);
					    #endif
					#endif
				#else
					ISP_MCLK1_EN(1);
				#endif
				//mtkcam_gpio_set(pinSetIdx, CAMLDO, 1);
				
				/* First Power Pin low and Reset Pin Low */
				if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMPDN]) {
					mtkcam_gpio_set(pinSetIdx, CAMPDN,
							pinSet[pinSetIdx][IDX_PS_CMPDN + IDX_PS_OFF]);
				}

				if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST]) {
					mtkcam_gpio_set(pinSetIdx, CAMRST,
							pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_OFF]);
				}

				/* VCAM_IO */
				if (TRUE != _hwPowerOn(VCAMIO, VOL_1800)) {
					PK_DBG("[CAMERA SENSOR] Fail to enable IO power (VCAM_IO), power id = %d\n", VCAMIO);
					goto _kdCISModulePowerOn_exit_;
				}
				mdelay(1);
				
				//VCAM_A
				#if defined(LYCONFIG_DUAL_CAMERA_USE_VCAMA_SUPPORT)
				if(TRUE != _hwPowerOn(VCAMA, VOL_2800))
				{
					PK_DBG("[CAMERA SENSOR] Fail to enable analog power (VCAM_A), power id = %d\n", VCAMA);
					goto _kdCISModulePowerOn_exit_;
				}
				#else
					#if defined(LYCONFIG_EXCHANGE_BACK_FRONT_CAMERA_SUPPORT) 
					if(TRUE != _hwPowerOn(VCAMA, VOL_2800))
					{
						PK_DBG("[CAMERA SENSOR] Fail to enable analog power (VCAM_A), power id = %d\n", VCAMA);
						goto _kdCISModulePowerOn_exit_;
					}
					#else    
					mtkcam_gpio_set(pinSetIdx, CAM1AVDD,1);
					#endif 
				#endif
				mdelay(1);
				
				#if defined(LYCONFIG_EXCHANGE_BACK_FRONT_CAMERA_SUPPORT)
				if(TRUE != _hwPowerOn(VCAMD, d_vol))
				{
					PK_DBG("[CAMERA SENSOR] Fail to enable digital power (VCAM_D), power id = %d \n", VCAMD);
					goto _kdCISModulePowerOn_exit_;
				}
				#else
				 if (d_vol == VOL_1000){
                                       d_vol=VOL_1200;
                                       pmic_set_register_value(PMIC_RG_VRF18_1_CAL, 7);//-120mv
                               }else {
                                       pmic_set_register_value(PMIC_RG_VRF18_1_CAL, 0);
                            }

				if(TRUE != _hwPowerOn(VSUBCAMD, d_vol))
					{
						PK_DBG("[CAMERA SENSOR] Fail to enable digital power (VCAM_D), power id = %d \n", VCAMD);
						goto _kdCISModulePowerOn_exit_;
					}
				#endif
			    	mdelay(5);

				//AF_VCC
				if(TRUE != _hwPowerOn(VCAMAF, VOL_2800))
				{
					PK_DBG("[CAMERA SENSOR] Fail to enable analog power (VCAM_AF), power id = %d \n", VCAMAF);
					goto _kdCISModulePowerOn_exit_;
				 }

				/* enable active sensor */
				if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMPDN])
					mtkcam_gpio_set(pinSetIdx, CAMPDN, pinSet[pinSetIdx][IDX_PS_CMPDN + ps_state]);
				mdelay(2);

				if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST])
					mtkcam_gpio_set(pinSetIdx, CAMRST, pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_ON]);
			
				mdelay(20);
			
			}
			else
			{
				#if  defined(LYCONFIG_DUAL_CAMERA_BOTH_WORK_SUPPORT)
					#if defined(LYCONFIG_DUAL_CAMERA_USE_CLK1_SUPPORT)
					ISP_MCLK1_EN(0);
                        		#else
					    #if defined(LYCONFIG_EXCHANGE_BACK_FRONT_CAMERA_SUPPORT)
					    ISP_MCLK1_EN(0);
					    #else
					    ISP_MCLK2_EN(0);
					    #endif
					#endif
				#else
				ISP_MCLK1_EN(0);
				#endif

				//mtkcam_gpio_set(pinSetIdx, CAMLDO, 0);
				
				/* First Power Pin low and Reset Pin Low */
				if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMPDN]) {
					mtkcam_gpio_set(pinSetIdx, CAMPDN,
							pinSet[pinSetIdx][IDX_PS_CMPDN + ps_off_state]);
				}

				if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST]) {
					mtkcam_gpio_set(pinSetIdx, CAMRST,
							pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_OFF]);
				}

				#if defined(LYCONFIG_EXCHANGE_BACK_FRONT_CAMERA_SUPPORT)
				if(TRUE != _hwPowerDown(VCAMD))
					{
						PK_DBG("[CAMERA SENSOR] Fail to OFF core power (VCAM_D), power id = %d \n",VCAMD);
						goto _kdCISModulePowerOn_exit_;
					}
				#else
				if(TRUE != _hwPowerDown(VSUBCAMD))
				{
					PK_DBG("[CAMERA SENSOR] Fail to OFF core power (VCAM_D), power id = %d \n",VCAMD);
					goto _kdCISModulePowerOn_exit_;
				}
				#endif

				//VCAM_A
				#if defined(LYCONFIG_DUAL_CAMERA_USE_VCAMA_SUPPORT)	
				if(TRUE != _hwPowerDown(VCAMA)) {
					PK_DBG("[CAMERA SENSOR] Fail to OFF analog power (VCAM_A), power id= (%d) \n", VCAMA);
					goto _kdCISModulePowerOn_exit_;
				}
				#else
					#if defined(LYCONFIG_EXCHANGE_BACK_FRONT_CAMERA_SUPPORT)
					if(TRUE != _hwPowerDown(VCAMA)) {
						PK_DBG("[CAMERA SENSOR] Fail to OFF analog power (VCAM_A), power id= (%d) \n", VCAMA);
						goto _kdCISModulePowerOn_exit_;
					}				
					#else
					mtkcam_gpio_set(pinSetIdx, CAM1AVDD,0);
					#endif
				#endif

				//VCAM_IO
				if(TRUE != _hwPowerDown(VCAMIO)) {
					PK_DBG("[CAMERA SENSOR] Fail to OFF digital power (VCAM_IO), power id = %d \n", VCAMIO);
					goto _kdCISModulePowerOn_exit_;
				}

				//AF_VCC
				if(TRUE != _hwPowerDown(VCAMAF))
				{
					PK_DBG("[CAMERA SENSOR] Fail to OFF AF power (VCAM_AF), power id = %d \n", VCAMAF);
					goto _kdCISModulePowerOn_exit_;
				}
			}
			break;
		default:
			break;
			
		}
	

	return 0;

_kdCISModulePowerOn_exit_:
	return -EIO;
	

}
#else
/*Legacy define*/
int kdCISModulePowerOn(CAMERA_DUAL_CAMERA_SENSOR_ENUM SensorIdx, char *currSensorName, BOOL On, char *mode_name)
{

	u32 pinSetIdx = 0;/*default main sensor*/

#define IDX_PS_CMRST 0
#define IDX_PS_CMPDN 4
#define IDX_PS_MODE 1
#define IDX_PS_ON   2
#define IDX_PS_OFF  3


	u32 pinSet[3][8] = {
		/* for main sensor */
		{
			/* The reset pin of main sensor uses GPIO10 of mt6306, please call mt6306 API to set */
			CAMERA_CMRST_PIN,
			CAMERA_CMRST_PIN_M_GPIO,   /* mode */
			GPIO_OUT_ONE,              /* ON state */
			GPIO_OUT_ZERO,             /* OFF state */
			CAMERA_CMPDN_PIN,
			CAMERA_CMPDN_PIN_M_GPIO,
			GPIO_OUT_ONE,
			GPIO_OUT_ZERO,
		},
		/* for sub sensor */
		{
			CAMERA_CMRST1_PIN,
			CAMERA_CMRST1_PIN_M_GPIO,
			GPIO_OUT_ONE,
			GPIO_OUT_ZERO,
			CAMERA_CMPDN1_PIN,
			CAMERA_CMPDN1_PIN_M_GPIO,
			GPIO_OUT_ONE,
			GPIO_OUT_ZERO,
		},
		/* for main_2 sensor */
		{
			GPIO_CAMERA_INVALID,
			GPIO_CAMERA_INVALID,   /* mode */
			GPIO_OUT_ONE,               /* ON state */
			GPIO_OUT_ZERO,              /* OFF state */
			GPIO_CAMERA_INVALID,
			GPIO_CAMERA_INVALID,
			GPIO_OUT_ONE,
			GPIO_OUT_ZERO,
		}
	};



	if (DUAL_CAMERA_MAIN_SENSOR == SensorIdx)
		pinSetIdx = 0;
	else if (DUAL_CAMERA_SUB_SENSOR == SensorIdx)
		pinSetIdx = 1;
	else if (DUAL_CAMERA_MAIN_2_SENSOR == SensorIdx)
		pinSetIdx = 2;


	/* power ON */
	if (On) {

#if 0
		ISP_MCLK1_EN(1);
		ISP_MCLK2_EN(1);
		ISP_MCLK3_EN(1);
#else
		if (pinSetIdx == 0)
			ISP_MCLK1_EN(1);
		else if (pinSetIdx == 1)
			ISP_MCLK2_EN(1);
#endif

		PK_DBG("[PowerON]pinSetIdx:%d, currSensorName: %s\n", pinSetIdx, currSensorName);

		if ((currSensorName && (0 == strcmp(currSensorName, "imx135mipiraw"))) ||
		    (currSensorName && (0 == strcmp(currSensorName, "imx220mipiraw")))) {
			/* First Power Pin low and Reset Pin Low */
			if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMPDN]) {
				if (mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMPDN],
					pinSet[pinSetIdx][IDX_PS_CMPDN + IDX_PS_MODE]))
					PK_DBG("[CAMERA LENS] set gpio mode failed!! (CMPDN)\n");
				if (mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMPDN], GPIO_DIR_OUT))
					PK_DBG("[CAMERA LENS] set gpio dir failed!! (CMPDN)\n");
				if (mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],
					pinSet[pinSetIdx][IDX_PS_CMPDN + IDX_PS_OFF]))
					PK_DBG("[CAMERA LENS] set gpio failed!! (CMPDN)\n");
			}


			if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST]) {
				if (mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMRST],
					pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_MODE]))
					PK_DBG("[CAMERA SENSOR] set gpio mode failed!! (CMRST)\n");
				if (mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST], GPIO_DIR_OUT))
					PK_DBG("[CAMERA SENSOR] set gpio dir failed!! (CMRST)\n");
				if (mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],
					pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_OFF]))
					PK_DBG("[CAMERA SENSOR] set gpio failed!! (CMRST)\n");
			}

			/* AF_VCC */
			if (TRUE != hwPowerOn(CAMERA_POWER_VCAM_AF, VOL_2800, mode_name)) {
				PK_DBG("[CAMERA SENSOR] Fail to enable analog power (VCAM_AF),power id = %d\n",
					CAMERA_POWER_VCAM_AF);
				goto _kdCISModulePowerOn_exit_;
			}

			mdelay(1);

			/* VCAM_A */
			if (TRUE != hwPowerOn(CAMERA_POWER_VCAM_A, VOL_2800, mode_name)) {
				PK_DBG("[CAMERA SENSOR] Fail to enable analog power (VCAM_A),power id = %d\n",
					CAMERA_POWER_VCAM_A);
				goto _kdCISModulePowerOn_exit_;
			}

			mdelay(1);

			if (TRUE != hwPowerOn(CAMERA_POWER_VCAM_D, VOL_1000, mode_name)) {
				PK_DBG("[CAMERA SENSOR] Fail to enable digital power (VCAM_D),power id = %d\n",
					CAMERA_POWER_VCAM_D);
				goto _kdCISModulePowerOn_exit_;
			}

			mdelay(1);

			/* VCAM_IO */
			if (TRUE != hwPowerOn(CAMERA_POWER_VCAM_IO, VOL_1800, mode_name)) {
				PK_DBG("[CAMERA SENSOR] Fail to enable digital power (VCAM_IO),power id = %d\n",
					CAMERA_POWER_VCAM_IO);
				goto _kdCISModulePowerOn_exit_;
			}

			mdelay(2);


			/* enable active sensor */
			if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST]) {
				if (mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMRST],
					pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_MODE]))
					PK_DBG("[CAMERA SENSOR] set gpio mode failed!! (CMRST)\n");
				if (mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST],
					GPIO_DIR_OUT))
					PK_DBG("[CAMERA SENSOR] set gpio dir failed!! (CMRST)\n");
				if (mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],
					pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_ON]))
					PK_DBG("[CAMERA SENSOR] set gpio failed!! (CMRST)\n");
			}

			if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMPDN]) {
				if (mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],
					pinSet[pinSetIdx][IDX_PS_CMPDN + IDX_PS_ON]))
					PK_DBG("[CAMERA LENS] set gpio failed!! (CMPDN)\n");
			}

		} else if (currSensorName && (0 == strcmp(SENSOR_DRVNAME_OV5648_MIPI_RAW,
			currSensorName))) {
			mt_set_gpio_mode(GPIO_SPI_MOSI_PIN, GPIO_MODE_00);
			mt_set_gpio_dir(GPIO_SPI_MOSI_PIN, GPIO_DIR_OUT);
			mt_set_gpio_out(GPIO_SPI_MOSI_PIN, GPIO_OUT_ONE);
			/* First Power Pin low and Reset Pin Low */
			if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMPDN]) {
				if (mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMPDN],
					pinSet[pinSetIdx][IDX_PS_CMPDN + IDX_PS_MODE]))
					PK_DBG("[CAMERA LENS] set gpio mode failed!! (CMPDN)\n");
				if (mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMPDN], GPIO_DIR_OUT))
					PK_DBG("[CAMERA LENS] set gpio dir failed!! (CMPDN)\n");
				if (mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],
					pinSet[pinSetIdx][IDX_PS_CMPDN + IDX_PS_OFF]))
					PK_DBG("[CAMERA LENS] set gpio failed!! (CMPDN)\n");
			}

			if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST]) {
				if (mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMRST],
					pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_MODE]))
					PK_DBG("[CAMERA SENSOR] set gpio mode failed!! (CMRST)\n");
				if (mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST], GPIO_DIR_OUT))
					PK_DBG("[CAMERA SENSOR] set gpio dir failed!! (CMRST)\n");
				if (mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],
					pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_OFF]))
					PK_DBG("[CAMERA SENSOR] set gpio failed!! (CMRST)\n");
			}


			/* VCAM_IO */
			if (TRUE != hwPowerOn(CAMERA_POWER_VCAM_IO, VOL_1800, mode_name)) {
				PK_DBG("[CAMERA SENSOR] Fail to enable digital power (VCAM_IO),power id = %d\n", CAMERA_POWER_VCAM_IO);
				goto _kdCISModulePowerOn_exit_;
			}

			mdelay(1);

			/* VCAM_A */
			if (TRUE != hwPowerOn(CAMERA_POWER_VCAM_A, VOL_2800, mode_name)) {
				PK_DBG("[CAMERA SENSOR] Fail to enable analog power (VCAM_A),power id = %d\n",
					CAMERA_POWER_VCAM_A);
				goto _kdCISModulePowerOn_exit_;
			}

			mdelay(1);

			if (TRUE != hwPowerOn(SUB_CAMERA_POWER_VCAM_D, VOL_1500, mode_name)) {
				PK_DBG("[CAMERA SENSOR] Fail to enable digital power (VCAM_D),power id = %d\n",
					CAMERA_POWER_VCAM_D);
				goto _kdCISModulePowerOn_exit_;
			}

			mdelay(5);

			/* AF_VCC */
			if (TRUE != hwPowerOn(CAMERA_POWER_VCAM_AF, VOL_2800, mode_name)) {
				PK_DBG("[CAMERA SENSOR] Fail to enable analog power (VCAM_AF),power id = %d\n",
					CAMERA_POWER_VCAM_AF);
				goto _kdCISModulePowerOn_exit_;
			}


			mdelay(1);


			/* enable active sensor */
			if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMPDN]) {
				if (mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMPDN],
					pinSet[pinSetIdx][IDX_PS_CMPDN + IDX_PS_MODE]))
					PK_DBG("[CAMERA LENS] set gpio mode failed!! (CMPDN)\n");
				if (mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMPDN],
					GPIO_DIR_OUT))
					PK_DBG("[CAMERA LENS] set gpio dir failed!! (CMPDN)\n");
				if (mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],
					pinSet[pinSetIdx][IDX_PS_CMPDN + IDX_PS_ON]))
					PK_DBG("[CAMERA LENS] set gpio failed!! (CMPDN)\n");
			}


			mdelay(2);


			if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST]) {
				if (mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMRST],
					pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_MODE]))
					PK_DBG("[CAMERA SENSOR] set gpio mode failed!! (CMRST)\n");
				if (mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST], GPIO_DIR_OUT))
					PK_DBG("[CAMERA SENSOR] set gpio dir failed!! (CMRST)\n");
				if (mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],
					pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_ON]))
					PK_DBG("[CAMERA SENSOR] set gpio failed!! (CMRST)\n");
			}

			mdelay(20);
		} else  if (currSensorName && (0 == strcmp(SENSOR_DRVNAME_GC2355_MIPI_RAW, currSensorName))) {
			mt_set_gpio_mode(GPIO_SPI_MOSI_PIN, GPIO_MODE_00);
			mt_set_gpio_dir(GPIO_SPI_MOSI_PIN, GPIO_DIR_OUT);
			mt_set_gpio_out(GPIO_SPI_MOSI_PIN, GPIO_OUT_ONE);
			/* First Power Pin low and Reset Pin Low */
			if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMPDN]) {
				if (mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMPDN],
					pinSet[pinSetIdx][IDX_PS_CMPDN + IDX_PS_MODE]))
					PK_DBG("[CAMERA LENS] set gpio mode failed!! (CMPDN)\n");
				if (mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMPDN], GPIO_DIR_OUT))
					PK_DBG("[CAMERA LENS] set gpio dir failed!! (CMPDN)\n");
				if (mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],
					pinSet[pinSetIdx][IDX_PS_CMPDN + IDX_PS_OFF]))
					PK_DBG("[CAMERA LENS] set gpio failed!! (CMPDN)\n");
			}

			if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST]) {
				if (mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMRST],
					pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_MODE]))
					PK_DBG("[CAMERA SENSOR] set gpio mode failed!! (CMRST)\n");
				if (mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST], GPIO_DIR_OUT))
					PK_DBG("[CAMERA SENSOR] set gpio dir failed!! (CMRST)\n");
				if (mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],
					pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_OFF]))
					PK_DBG("[CAMERA SENSOR] set gpio failed!! (CMRST)\n");
			}

			mdelay(50);

			/* VCAM_A */
			if (TRUE != hwPowerOn(CAMERA_POWER_VCAM_A, VOL_2800, mode_name)) {
				PK_DBG("[CAMERA SENSOR] Fail to enable analog power (VCAM_A),power id = %d\n",
					CAMERA_POWER_VCAM_A);
				goto _kdCISModulePowerOn_exit_;
			}

			mdelay(10);

			/* VCAM_IO */
			if (TRUE != hwPowerOn(CAMERA_POWER_VCAM_IO, VOL_1800, mode_name)) {
				PK_DBG("[CAMERA SENSOR] Fail to enable digital power (VCAM_IO),power id = %d\n",
					CAMERA_POWER_VCAM_IO);
				goto _kdCISModulePowerOn_exit_;
			}

			mdelay(10);

			if (TRUE != hwPowerOn(SUB_CAMERA_POWER_VCAM_D, VOL_1500, mode_name)) {
				PK_DBG("[CAMERA SENSOR] Fail to enable digital power (VCAM_D),power id = %d\n",
					CAMERA_POWER_VCAM_D);
				goto _kdCISModulePowerOn_exit_;
			}

			mdelay(10);

			/* AF_VCC */
			if (TRUE != hwPowerOn(CAMERA_POWER_VCAM_AF, VOL_2800, mode_name)) {
				PK_DBG("[CAMERA SENSOR] Fail to enable analog power (VCAM_AF),power id = %d\n",
					CAMERA_POWER_VCAM_AF);
				goto _kdCISModulePowerOn_exit_;
			}


			mdelay(50);

			if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST]) {
				if (mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMRST],
					pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_MODE]))
					PK_DBG("[CAMERA SENSOR] set gpio mode failed!! (CMRST)\n");
				if (mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST], GPIO_DIR_OUT))
					PK_DBG("[CAMERA SENSOR] set gpio dir failed!! (CMRST)\n");
				if (mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],
					pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_OFF]))
					PK_DBG("[CAMERA SENSOR] set gpio failed!! (CMRST)\n");
				mdelay(5);
				if (mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],
					pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_ON]))
					PK_DBG("[CAMERA SENSOR] set gpio failed!! (CMRST)\n");

			}
			mdelay(5);
			/* enable active sensor */
			if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMPDN]) {
				if (mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMPDN],
					pinSet[pinSetIdx][IDX_PS_CMPDN + IDX_PS_MODE]))
					PK_DBG("[CAMERA LENS] set gpio mode failed!! (CMPDN)\n"); }
				if (mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMPDN], GPIO_DIR_OUT))
					PK_DBG("[CAMERA LENS] set gpio dir failed!! (CMPDN)\n");
				if (mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],
					pinSet[pinSetIdx][IDX_PS_CMPDN + IDX_PS_ON]))
					PK_DBG("[CAMERA LENS] set gpio failed!! (CMPDN)\n");
				mdelay(5);
				if (mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],
					pinSet[pinSetIdx][IDX_PS_CMPDN + IDX_PS_OFF]))
					PK_DBG("[CAMERA LENS] set gpio failed!! (CMPDN)\n");
			} else {
			/* First Power Pin low and Reset Pin Low */
			if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMPDN]) {
				if (mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMPDN],
					pinSet[pinSetIdx][IDX_PS_CMPDN + IDX_PS_MODE]))
					PK_DBG("[CAMERA LENS] set gpio mode failed!! (CMPDN)\n");
				if (mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMPDN], GPIO_DIR_OUT))
					PK_DBG("[CAMERA LENS] set gpio dir failed!! (CMPDN)\n");
				if (mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],
					pinSet[pinSetIdx][IDX_PS_CMPDN + IDX_PS_OFF]))
					PK_DBG("[CAMERA LENS] set gpio failed!! (CMPDN)\n");
			}

			if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST]) {
				if (mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMRST],
					pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_MODE]))
					PK_DBG("[CAMERA SENSOR] set gpio mode failed!! (CMRST)\n");
				if (mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST], GPIO_DIR_OUT))
					PK_DBG("[CAMERA SENSOR] set gpio dir failed!! (CMRST)\n");
				if (mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],
					pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_OFF]))
					PK_DBG("[CAMERA SENSOR] set gpio failed!! (CMRST)\n");
			}

			/* VCAM_IO */
			if (TRUE != hwPowerOn(CAMERA_POWER_VCAM_IO, VOL_1800, mode_name)) {
				PK_DBG("[CAMERA SENSOR] Fail to enable digital power (VCAM_IO),power id = %d\n",
					CAMERA_POWER_VCAM_IO);
				goto _kdCISModulePowerOn_exit_;
			}

			/* VCAM_A */
			if (TRUE != hwPowerOn(CAMERA_POWER_VCAM_A, VOL_2800, mode_name)) {
				PK_DBG("[CAMERA SENSOR] Fail to enable analog power (VCAM_A),power id = %d\n",
					CAMERA_POWER_VCAM_A);
				goto _kdCISModulePowerOn_exit_;
			}
			/* VCAM_D */
			if (currSensorName && (0 == strcmp(SENSOR_DRVNAME_S5K2P8_MIPI_RAW, currSensorName))) {
				if (TRUE != hwPowerOn(CAMERA_POWER_VCAM_D, VOL_1200, mode_name)) {
					PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
					goto _kdCISModulePowerOn_exit_;
				}
			} else if (currSensorName && (0 == strcmp(SENSOR_DRVNAME_IMX219_MIPI_RAW, currSensorName))) {
				if (TRUE != hwPowerOn(CAMERA_POWER_VCAM_D, VOL_1200, mode_name)) {
					PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
					goto _kdCISModulePowerOn_exit_;
				}
			} else { /* Main VCAMD max 1.5V */
				if (TRUE != hwPowerOn(CAMERA_POWER_VCAM_D, VOL_1500, mode_name)) {
					PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
					goto _kdCISModulePowerOn_exit_;
				}

			}


			/* AF_VCC */
			if (TRUE != hwPowerOn(CAMERA_POWER_VCAM_AF, VOL_2800, mode_name)) {
				PK_DBG("[CAMERA SENSOR] Fail to enable analog power (VCAM_AF),power id = %d\n",
					CAMERA_POWER_VCAM_AF);
				goto _kdCISModulePowerOn_exit_;
			}

			mdelay(5);

			/* enable active sensor */
			if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMPDN]) {
				if (mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMPDN],
					pinSet[pinSetIdx][IDX_PS_CMPDN + IDX_PS_MODE]))
					PK_DBG("[CAMERA LENS] set gpio mode failed!! (CMPDN)\n");
				if (mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMPDN], GPIO_DIR_OUT))
					PK_DBG("[CAMERA LENS] set gpio dir failed!! (CMPDN)\n");
				if (mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],
					pinSet[pinSetIdx][IDX_PS_CMPDN + IDX_PS_ON]))
					PK_DBG("[CAMERA LENS] set gpio failed!! (CMPDN)\n");
			}

			mdelay(1);

			if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST]) {
				if (mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMRST],
					pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_MODE]))
					PK_DBG("[CAMERA SENSOR] set gpio mode failed!! (CMRST)\n");
				if (mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST], GPIO_DIR_OUT))
					PK_DBG("[CAMERA SENSOR] set gpio dir failed!! (CMRST)\n");
				if (mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],
					pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_ON]))
					PK_DBG("[CAMERA SENSOR] set gpio failed!! (CMRST)\n");
			}
		}
	} else { /* power OFF */

		PK_DBG("[PowerOFF]pinSetIdx:%d\n", pinSetIdx);
		if (pinSetIdx == 0)
			ISP_MCLK1_EN(0);
		else if (pinSetIdx == 1)
			ISP_MCLK2_EN(0);

		if ((currSensorName && (0 == strcmp(currSensorName, "imx135mipiraw"))) ||
		    (currSensorName && (0 == strcmp(currSensorName, "imx220mipiraw")))) {
			/* Set Power Pin low and Reset Pin Low */
			if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMPDN]) {
				if (mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMPDN],
					pinSet[pinSetIdx][IDX_PS_CMPDN + IDX_PS_MODE]))
					PK_DBG("[CAMERA LENS] set gpio mode failed!! (CMPDN)\n");
				if (mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMPDN], GPIO_DIR_OUT))
					PK_DBG("[CAMERA LENS] set gpio dir failed!! (CMPDN)\n");
				if (mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],
					pinSet[pinSetIdx][IDX_PS_CMPDN + IDX_PS_OFF]))
					PK_DBG("[CAMERA LENS] set gpio failed!! (CMPDN)\n");
			}

			/* Set Reset Pin Low */
			if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST]) {
				if (mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMRST],
					pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_MODE]))
					PK_DBG("[CAMERA SENSOR] set gpio mode failed!! (CMRST)\n");
				if (mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST],
					GPIO_DIR_OUT))
					PK_DBG("[CAMERA SENSOR] set gpio dir failed!! (CMRST)\n");
				if (mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],
					pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_OFF]))
					PK_DBG("[CAMERA SENSOR] set gpio failed!! (CMRST)\n");
			}

			/* AF_VCC */
			if (TRUE != hwPowerDown(CAMERA_POWER_VCAM_AF, mode_name)) {
				PK_DBG("[CAMERA SENSOR] Fail to OFF AF power (VCAM_AF),power id = %d\n",
					CAMERA_POWER_VCAM_AF);
				/* return -EIO; */
				goto _kdCISModulePowerOn_exit_;
			}

			/* VCAM_IO */
			if (TRUE != hwPowerDown(CAMERA_POWER_VCAM_IO, mode_name)) {
				PK_DBG("[CAMERA SENSOR] Fail to OFF digital power (VCAM_IO),power id = %d\n",
					CAMERA_POWER_VCAM_IO);
				/* return -EIO; */
				goto _kdCISModulePowerOn_exit_;
			}

			if (TRUE != hwPowerDown(CAMERA_POWER_VCAM_D, mode_name)) {
				PK_DBG("[CAMERA SENSOR] Fail to OFF core power (VCAM_D),power id = %d\n",
					CAMERA_POWER_VCAM_D);
				goto _kdCISModulePowerOn_exit_;
			}

			/* VCAM_A */
			if (TRUE != hwPowerDown(CAMERA_POWER_VCAM_A, mode_name)) {
				PK_DBG("[CAMERA SENSOR] Fail to OFF analog power (VCAM_A),power id= (%d)\n",
					CAMERA_POWER_VCAM_A);
				/* return -EIO; */
				goto _kdCISModulePowerOn_exit_;
			}

		} else if (currSensorName && (0 == strcmp(SENSOR_DRVNAME_OV5648_MIPI_RAW, currSensorName))) {
			mt_set_gpio_out(GPIO_SPI_MOSI_PIN, GPIO_OUT_ZERO);
			/* Set Power Pin low and Reset Pin Low */
			if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMPDN]) {
				if (mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMPDN],
					pinSet[pinSetIdx][IDX_PS_CMPDN + IDX_PS_MODE]))
					PK_DBG("[CAMERA LENS] set gpio mode failed!! (CMPDN)\n");
				if (mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMPDN], GPIO_DIR_OUT))
					PK_DBG("[CAMERA LENS] set gpio dir failed!! (CMPDN)\n");
				if (mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],
					pinSet[pinSetIdx][IDX_PS_CMPDN + IDX_PS_OFF]))
					PK_DBG("[CAMERA LENS] set gpio failed!! (CMPDN)\n");
			}

			if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST]) {
				if (mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMRST],
					pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_MODE]))
					PK_DBG("[CAMERA SENSOR] set gpio mode failed!! (CMRST)\n");
				if (mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST],
					GPIO_DIR_OUT))
					PK_DBG("[CAMERA SENSOR] set gpio dir failed!! (CMRST)\n");
				if (mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],
					pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_OFF]))
					PK_DBG("[CAMERA SENSOR] set gpio failed!! (CMRST)\n");
			}

			if (TRUE != hwPowerDown(SUB_CAMERA_POWER_VCAM_D, mode_name)) {
				PK_DBG("[CAMERA SENSOR] Fail to OFF core power (VCAM_D),power id = %d\n",
					SUB_CAMERA_POWER_VCAM_D);
				goto _kdCISModulePowerOn_exit_;
			}

			/* VCAM_A */
			if (TRUE != hwPowerDown(CAMERA_POWER_VCAM_A, mode_name)) {
				PK_DBG("[CAMERA SENSOR] Fail to OFF analog power (VCAM_A),power id= (%d)\n",
					CAMERA_POWER_VCAM_A);
				/* return -EIO; */
				goto _kdCISModulePowerOn_exit_;
			}

			/* VCAM_IO */
			if (TRUE != hwPowerDown(CAMERA_POWER_VCAM_IO, mode_name)) {
				PK_DBG("[CAMERA SENSOR] Fail to OFF digital power (VCAM_IO),power id = %d\n",
					CAMERA_POWER_VCAM_IO);
				/* return -EIO; */
				goto _kdCISModulePowerOn_exit_;
			}

			/* AF_VCC */
			if (TRUE != hwPowerDown(CAMERA_POWER_VCAM_AF, mode_name)) {
				PK_DBG("[CAMERA SENSOR] Fail to OFF AF power (VCAM_AF),power id = %d\n",
					CAMERA_POWER_VCAM_AF);
				/* return -EIO; */
				goto _kdCISModulePowerOn_exit_;
			}

		} else if (currSensorName && (0 == strcmp(SENSOR_DRVNAME_GC2355_MIPI_RAW, currSensorName))) {
			mt_set_gpio_out(GPIO_SPI_MOSI_PIN, GPIO_OUT_ZERO);
			/* Set Power Pin low and Reset Pin Low */
			if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMPDN]) {
				if (mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMPDN],
					pinSet[pinSetIdx][IDX_PS_CMPDN + IDX_PS_MODE]))
					PK_DBG("[CAMERA LENS] set gpio mode failed!! (CMPDN)\n");
				if (mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMPDN], GPIO_DIR_OUT))
					PK_DBG("[CAMERA LENS] set gpio dir failed!! (CMPDN)\n");
				if (mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],
					pinSet[pinSetIdx][IDX_PS_CMPDN + IDX_PS_ON]))
					PK_DBG("[CAMERA LENS] set gpio failed!! (CMPDN)\n");
			}

			if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST]) {
				if (mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMRST],
					pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_MODE]))
					PK_DBG("[CAMERA SENSOR] set gpio mode failed!! (CMRST)\n");
				if (mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST],
					GPIO_DIR_OUT))
					PK_DBG("[CAMERA SENSOR] set gpio dir failed!! (CMRST)\n");
				if (mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],
					pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_OFF]))
					PK_DBG("[CAMERA SENSOR] set gpio failed!! (CMRST)\n");
			}


			if (TRUE != hwPowerDown(SUB_CAMERA_POWER_VCAM_D, mode_name)) {
				PK_DBG("[CAMERA SENSOR] Fail to OFF core power (VCAM_D),power id = %d\n",
					SUB_CAMERA_POWER_VCAM_D);
				goto _kdCISModulePowerOn_exit_;
			}

			/* VCAM_A */
			if (TRUE != hwPowerDown(CAMERA_POWER_VCAM_A, mode_name)) {
				PK_DBG("[CAMERA SENSOR] Fail to OFF analog power (VCAM_A),power id= (%d)\n",
					CAMERA_POWER_VCAM_A);
				/* return -EIO; */
				goto _kdCISModulePowerOn_exit_;
			}

			/* VCAM_IO */
			if (TRUE != hwPowerDown(CAMERA_POWER_VCAM_IO, mode_name)) {
				PK_DBG("[CAMERA SENSOR] Fail to OFF digital power (VCAM_IO),power id = %d\n",
					CAMERA_POWER_VCAM_IO);
				/* return -EIO; */
				goto _kdCISModulePowerOn_exit_;
			}

			/* AF_VCC */
			if (TRUE != hwPowerDown(CAMERA_POWER_VCAM_AF, mode_name)) {
				PK_DBG("[CAMERA SENSOR] Fail to OFF AF power (VCAM_AF),power id = %d\n",
					CAMERA_POWER_VCAM_AF);
				/* return -EIO; */
				goto _kdCISModulePowerOn_exit_;
			}

		} else {
			/* Set Power Pin low and Reset Pin Low */
			if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMPDN]) {
				if (mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMPDN],
					pinSet[pinSetIdx][IDX_PS_CMPDN + IDX_PS_MODE]))
					PK_DBG("[CAMERA LENS] set gpio mode failed!! (CMPDN)\n");
				if (mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMPDN], GPIO_DIR_OUT))
					PK_DBG("[CAMERA LENS] set gpio dir failed!! (CMPDN)\n");
				if (mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],
					pinSet[pinSetIdx][IDX_PS_CMPDN + IDX_PS_OFF]))
					PK_DBG("[CAMERA LENS] set gpio failed!! (CMPDN)\n");
			}


			if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST]) {
				if (mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMRST],
					pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_MODE]))
					PK_DBG("[CAMERA SENSOR] set gpio mode failed!! (CMRST)\n");
				if (mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST],
					GPIO_DIR_OUT))
					PK_DBG("[CAMERA SENSOR] set gpio dir failed!! (CMRST)\n");
				if (mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],
					pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_OFF]))
					PK_DBG("[CAMERA SENSOR] set gpio failed!! (CMRST)\n");
			}


			if (TRUE != hwPowerDown(SUB_CAMERA_POWER_VCAM_D, mode_name)) {
				PK_DBG("[CAMERA SENSOR] Fail to OFF core power (VCAM_D),power id = %d\n",
					SUB_CAMERA_POWER_VCAM_D);
				goto _kdCISModulePowerOn_exit_;
			}

			/* VCAM_A */
			if (TRUE != hwPowerDown(CAMERA_POWER_VCAM_A, mode_name)) {
				PK_DBG("[CAMERA SENSOR] Fail to OFF analog power (VCAM_A),power id= (%d)\n",
					CAMERA_POWER_VCAM_A);
				/* return -EIO; */
				goto _kdCISModulePowerOn_exit_;
			}

			/* VCAM_IO */
			if (TRUE != hwPowerDown(CAMERA_POWER_VCAM_IO, mode_name)) {
				PK_DBG("[CAMERA SENSOR] Fail to OFF digital power (VCAM_IO),power id = %d\n",
					CAMERA_POWER_VCAM_IO);
				/* return -EIO; */
				goto _kdCISModulePowerOn_exit_;
			}

			/* AF_VCC */
			if (TRUE != hwPowerDown(CAMERA_POWER_VCAM_AF, mode_name)) {
				PK_DBG("[CAMERA SENSOR] Fail to OFF AF power (VCAM_AF),power id = %d\n",
					CAMERA_POWER_VCAM_AF);
				/* return -EIO; */
				goto _kdCISModulePowerOn_exit_;
			}

		}

	}

	return 0;

_kdCISModulePowerOn_exit_:
	return -EIO;
}

#endif
EXPORT_SYMBOL(kdCISModulePowerOn);

/* !-- */
/*  */


