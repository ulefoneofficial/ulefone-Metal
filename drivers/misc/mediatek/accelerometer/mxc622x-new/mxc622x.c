/* MXC622X motion sensor driver
*
*
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
#include "cust_acc.h"
#include "accel.h"
#include "mxc622x.h"
/*----------------------------------------------------------------------------*/
#define I2C_DRIVERID_MXC622X 150
/*----------------------------------------------------------------------------*/
#define DEBUG_SWITCH 0
/*----------------------------------------------------------------------------*/
//#define CONFIG_MXC622X_LOWPASS   /*apply low pass filter on output*/		 
#define SW_CALIBRATION

#if DEBUG_SWITCH
#define GSE_TAG					"[mxc622x] "
#define GSE_ERR(fmt, args...)	pr_err(GSE_TAG fmt, ##args)
#define GSE_LOG(fmt, args...)	pr_debug(GSE_TAG fmt, ##args)
#else
#define GSE_TAG
#define GSE_ERR(fmt, args...)	do {} while (0)
#define GSE_LOG(fmt, args...)	do {} while (0)
#endif


#ifdef CONFIG_COMPAT
#undef CONFIG_COMPAT
#endif
/*----------------------------------------------------------------------------*/
#define MXC622X_AXIS_X          0
#define MXC622X_AXIS_Y          1
#define MXC622X_AXIS_Z          2
#define MXC622X_AXES_NUM        2
#define MXC622X_DATA_LEN        2
#define MXC622X_DEV_NAME        "MXC622X"
/*----------------------------------------------------------------------------*/

#define MXC622X_REG_CHIP_ID 0x08
#define MXC622X_ID 0x05
#define MXC622X_REG_DETECTION 0x04
/*----------------------------------------------------------------------------*/
static int mxc622x_local_init(void);
static int mxc622x_remove(void);
static int mxc622x_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id); 
static int mxc622x_i2c_remove(struct i2c_client *client);
static int mxc622x_suspend(struct i2c_client *client, pm_message_t msg);
static int mxc622x_resume(struct i2c_client *client);

/*----------------------------------------------------------------------------*/
typedef enum {
	ADX_TRC_FILTER  = 0x01,
	ADX_TRC_RAWDATA = 0x02,
	ADX_TRC_IOCTL   = 0x04,
	ADX_TRC_CALI    = 0X08,
	ADX_TRC_INFO    = 0X10,
} ADX_TRC;
/*----------------------------------------------------------------------------*/
struct scale_factor {
	u8 whole;
	u8 fraction;
};
/*----------------------------------------------------------------------------*/
struct data_resolution {
	struct scale_factor scalefactor;
	int sensitivity;
};
/*----------------------------------------------------------------------------*/
#define C_MAX_FIR_LENGTH (32)
/*----------------------------------------------------------------------------*/
struct data_filter {
	s16 raw[C_MAX_FIR_LENGTH][MXC622X_AXES_NUM];
	int sum[MXC622X_AXES_NUM];
	int num;
	int idx;
};
/*----------------------------------------------------------------------------*/
struct mxc622x_i2c_data {
	struct i2c_client *client;
	struct acc_hw *hw;
	struct hwmsen_convert cvt;
	atomic_t layout;
	/*misc */
	struct data_resolution *reso;
	atomic_t trace;
	atomic_t suspend;
	atomic_t selftest;
	atomic_t filter;
	s16 cali_sw[MXC622X_AXES_NUM+1];

	/*data */
	s8 offset[MXC622X_AXES_NUM+1];  /*+1: for 4-byte alignment*/
	s16 data[MXC622X_AXES_NUM+1];

#if defined(CONFIG_MXC622X_LOWPASS)
	atomic_t firlen;
	atomic_t fir_en;
	struct data_filter fir;
#endif
};
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
static struct acc_init_info mxc622x_init_info = {
	.name = "mxc622x",
	.init = mxc622x_local_init,
	.uninit = mxc622x_remove,
};

#ifdef CONFIG_OF
static const struct of_device_id mxc622x_of_match[] = {
	{.compatible = "mediatek,mt6580-mxc622x"},
	{},
};
MODULE_DEVICE_TABLE(of, mxc622x_of_match);
#endif

/*----------------------------------------------------------------------------*/
static const struct i2c_device_id mxc622x_i2c_id[] = {{MXC622X_DEV_NAME,0},{}};

static int mxc622x_init_flag =-1;// 0<==>OK -1 <==> fail
/* Maintain  cust info here */
struct acc_hw mxc622x_accel_cust;
static struct acc_hw *hw = &mxc622x_accel_cust;

/* For  driver get cust info */
struct acc_hw *mxc622x_get_cust_acc(void)
{
	return &mxc622x_accel_cust;//i2c addr:0x15
}

static struct i2c_driver mxc622x_i2c_driver = {
	.driver = {
		.name = MXC622X_DEV_NAME,
#ifdef CONFIG_OF
		.of_match_table = mxc622x_of_match,
#endif
	},
	.probe = mxc622x_i2c_probe,
	.remove	= mxc622x_i2c_remove,
	.suspend = mxc622x_suspend,
	.resume = mxc622x_resume,
	.id_table = mxc622x_i2c_id,
};

/*----------------------------------------------------------------------------*/
static struct i2c_client *mxc622x_i2c_client = NULL;
static struct mxc622x_i2c_data *obj_i2c_data = NULL;

static bool sensor_power = true;
static struct GSENSOR_VECTOR3D gsensor_gain;
static char selftestRes[10] = {0};

//static struct mutex mxc622x_mutex;

/*----------------------------------------------------------------------------*/
static struct data_resolution mxc622x_data_resolution[1] = {
	/* combination by {FULL_RES,RANGE}*/
	{{ 15, 6}, 64},   // dataformat +/-2g	in 8-bit resolution;  { 15, 6} = 15.6 = (2*2*1000)/(2^8);  64 = (2^8)/(2*2)
};
/*----------------------------------------------------------------------------*/
static struct data_resolution mxc622x_offset_resolution = {{15, 6}, 64};


/*----------------------------------------------------------------------------*/
static int mxc622x_i2c_read_block(struct i2c_client *client, u8 addr, u8 *data, u8 len)
{
	int err;
	u8 beg = addr;
	struct i2c_msg msgs[2] = {
		{
			.addr = client->addr,	.flags = 0,
			.len = 1,	.buf = &beg
		},
		{
			.addr = client->addr,	.flags = I2C_M_RD,
			.len = len,	.buf = data,
		}
	};

	if (!client) {
		return -EINVAL;
	} else if (len > C_I2C_FIFO_SIZE) {
		GSE_ERR(" length %d exceeds %d\n", len, C_I2C_FIFO_SIZE);
		return -EINVAL;
	}

	err = i2c_transfer(client->adapter, msgs, sizeof(msgs)/sizeof(msgs[0]));
	if (err != 2) {
		GSE_ERR("i2c_transfer error: (%d %p %d) %d\n",
			addr, data, len, err);
		err = -EIO;
	} else {
		err = 0;
	}
	return err;

}

/*--------------------MXC622X power control function----------------------------------*/
static void MXC622X_power(struct acc_hw *hw, unsigned int on) 
{
}

/*----------------------------------------------------------------------------*/
static int MXC622X_SetPowerMode(struct i2c_client *client, bool enable)
{
	u8 databuf[2];
	int res = 0;
	struct mxc622x_i2c_data *obj = i2c_get_clientdata(client);


	if (enable == sensor_power) {
		GSE_LOG("Sensor power status is newest!\n");
		return MXC622X_SUCCESS;
	}
#if 0
	if (hwmsen_read_block(client, addr, databuf, 0x01)) {
		GSE_ERR("read power ctl register err!\n");
		return MXC622X_ERR_I2C;
	}
#endif

	if (enable) {
		databuf[1] = 0x00;
	} else {
		databuf[1] = 0x01<<7;
	}

	databuf[0] = MXC622X_REG_DETECTION;

	res = i2c_master_send(client, databuf, 0x2);

	if (res <= 0) {
		GSE_LOG("set power mode failed!\n");
		return MXC622X_ERR_I2C;
	} else if (atomic_read(&obj->trace) & ADX_TRC_INFO) {
		GSE_LOG("set power mode ok %d!\n", databuf[1]);
	}

	sensor_power = enable;

	mdelay(20);

	return MXC622X_SUCCESS;
}

/*----------------------------------------------------------------------------*/
static int MXC622X_SetDataResolution(struct mxc622x_i2c_data *obj)
{
	/*set g sensor dataresolution here*/

	/*MXC622X only can set to 8-bit dataresolution, so do nothing in mxc622x driver here*/

	/*end of set dataresolution*/



	/*we set measure range from -2g to +2g in MXC622X_SetDataFormat(client, MXC622X_RANGE_2G),
	and set 10-bit dataresolution MXC622X_SetDataResolution()*/
	/*so mxc622x_data_resolution[0] set value as {{ 15, 6},64} when declaration, and assign the value to obj->reso here*/

	obj->reso = &mxc622x_data_resolution[0];
	return 0;
	
	/*if you changed the measure range, for example call: MXC622X_SetDataFormat(client, MXC622X_RANGE_4G), 
	you must set the right value to mxc622x_data_resolution*/

}

/*----------------------------------------------------------------------------*/
static int MXC622X_ReadData(struct i2c_client *client, s16 data[MXC622X_AXES_NUM])
{
	struct mxc622x_i2c_data *priv = i2c_get_clientdata(client);
	u8 addr = MXC622X_REG_DATAX0;
	u8 buf[MXC622X_DATA_LEN] = {0};
	int err = 0;
	int i;

	if (NULL == client) {
		err = -EINVAL;
		return err;
	}

	err = mxc622x_i2c_read_block(client, addr, buf, 0x02);
	if (err) {
		GSE_ERR("error: %d\n", err);
		return err;
	}

	data[MXC622X_AXIS_X] = (s16)buf[0];
	data[MXC622X_AXIS_Y] = (s16)buf[1];
	GSE_ERR("MXC622X_AXIS_X: %d MXC622X_AXIS_Y:%d \n", data[MXC622X_AXIS_X], data[MXC622X_AXIS_Y]);
	/*
	if (data[MXC622X_AXIS_X]&0x80) {
		data[MXC622X_AXIS_X] = ~data[MXC622X_AXIS_X];
		data[MXC622X_AXIS_X] &= 0xff;
		data[MXC622X_AXIS_X]+=1;
		data[MXC622X_AXIS_X] = -data[MXC622X_AXIS_X];
	}
	if (data[MXC622X_AXIS_Y]&0x80) {
		data[MXC622X_AXIS_Y] = ~data[MXC622X_AXIS_Y];
		data[MXC622X_AXIS_Y] &= 0xff;
		data[MXC622X_AXIS_Y]+=1;
		data[MXC622X_AXIS_Y] = -data[MXC622X_AXIS_Y];
	}
	*/
	for (i = 0; i < 2; i ++) {
		if (data[i] == 0x80 )
			data[i]= -128;
		else if (data[i] & 0x080) {
			data[i] -= 0x1;
			data[i] = ~data[i];
			data[i] &= 0xff;
			data[i] = -data[i];
		}
	}

	if (atomic_read(&priv->trace) & ADX_TRC_RAWDATA) {
		GSE_LOG("[%08X %08X] => [%5d %5d]\n", data[MXC622X_AXIS_X],
			data[MXC622X_AXIS_Y],
			data[MXC622X_AXIS_X], data[MXC622X_AXIS_Y]);
	}
	GSE_LOG("0x02=%x\n", mxc622x_i2c_read_block(client, 0x02, buf, 0x01));
	GSE_LOG("0x04=%x\n", mxc622x_i2c_read_block(client, 0x04, buf, 0x01));
#ifdef CONFIG_MXC622X_LOWPASS
	if (atomic_read(&priv->filter)) {
		if (atomic_read(&priv->fir_en) && !atomic_read(&priv->suspend)) {
			int idx, firlen = atomic_read(&priv->firlen);

			if (priv->fir.num < firlen) {
				priv->fir.raw[priv->fir.num][MXC622X_AXIS_X] = data[MXC622X_AXIS_X];
				priv->fir.raw[priv->fir.num][MXC622X_AXIS_Y] = data[MXC622X_AXIS_Y];
				//priv->fir.raw[priv->fir.num][MXC622X_AXIS_Z] = data[MPU6050_AXIS_Z];
				priv->fir.sum[MXC622X_AXIS_X] += data[MXC622X_AXIS_X];
				priv->fir.sum[MXC622X_AXIS_Y] += data[MXC622X_AXIS_Y];
				//priv->fir.sum[MXC622X_AXIS_Z] += data[MPU6050_AXIS_Z];
				if (atomic_read(&priv->trace) & ADX_TRC_FILTER) {
					GSE_LOG("add [%2d] [%5d %5d] => [%5d %5d]\n", priv->fir.num,
						priv->fir.raw[priv->fir.num][MXC622X_AXIS_X],
						priv->fir.raw[priv->fir.num][MXC622X_AXIS_Y],
						//priv->fir.raw[priv->fir.num][MXC622X_AXIS_Z],
						priv->fir.sum[MXC622X_AXIS_X],priv->fir.sum[MXC622X_AXIS_Y]
						/*priv->fir.sum[MXC622X_AXIS_Z]*/);
				}
				priv->fir.num++;
				priv->fir.idx++;
			} else {
				idx = priv->fir.idx % firlen;
				priv->fir.sum[MXC622X_AXIS_X] -= priv->fir.raw[idx][MXC622X_AXIS_X];
				priv->fir.sum[MXC622X_AXIS_Y] -= priv->fir.raw[idx][MXC622X_AXIS_Y];
				//priv->fir.sum[MXC622X_AXIS_Z] -= priv->fir.raw[idx][MXC622X_AXIS_Z];
				priv->fir.raw[idx][MXC622X_AXIS_X] = data[MXC622X_AXIS_X];
				priv->fir.raw[idx][MXC622X_AXIS_Y] = data[MXC622X_AXIS_Y];
				//priv->fir.raw[idx][MXC622X_AXIS_Z] = data[MXC622X_AXIS_Z];
				priv->fir.sum[MXC622X_AXIS_X] += data[MXC622X_AXIS_X];
				priv->fir.sum[MXC622X_AXIS_Y] += data[MXC622X_AXIS_Y];
				//priv->fir.sum[MXC622X_AXIS_Z] += data[MXC622X_AXIS_Z];
				priv->fir.idx++;
				data[MXC622X_AXIS_X] = priv->fir.sum[MXC622X_AXIS_X]/firlen;
				data[MXC622X_AXIS_Y] = priv->fir.sum[MXC622X_AXIS_Y]/firlen;
				//data[MXC622X_AXIS_Z] = priv->fir.sum[MXC622X_AXIS_Z]/firlen;
				if (atomic_read(&priv->trace) & ADX_TRC_FILTER) {
					GSE_LOG("add [%2d] [%5d %5d] => [%5d %5d] : [%5d %5d]\n",
					idx,
					priv->fir.raw[idx][MXC622X_AXIS_X], priv->fir.raw[idx][MXC622X_AXIS_Y],
					priv->fir.sum[MXC622X_AXIS_X], priv->fir.sum[MXC622X_AXIS_Y],
					data[MXC622X_AXIS_X], data[MXC622X_AXIS_Y]);
				}
			}
		}
	}
#endif

	return err;
}

/*----------------------------------------------------------------------------*/
static int MXC622X_ReadOffset(struct i2c_client *client, s8 ofs[MXC622X_AXES_NUM])
{
	int err = 0;
#ifdef SW_CALIBRATION
	ofs[0] = ofs[1] = ofs[2] = 0x0;
#else
	/*
	err = hwmsen_read_block(client, MXC622X_REG_OFSX, ofs, MXC622X_AXES_NUM);
	if (err)
		GSE_ERR("error: %d\n", err);
	*/
#endif
	/* GSE_LOG("offesx=%x, y=%x, z=%x",ofs[0],ofs[1],ofs[2]); */

	return err;
}

/*----------------------------------------------------------------------------*/
static int MXC622X_ResetCalibration(struct i2c_client *client)
{
	struct mxc622x_i2c_data *obj = i2c_get_clientdata(client);

	memset(obj->cali_sw, 0x00, sizeof(obj->cali_sw));
	memset(obj->offset, 0x00, sizeof(obj->offset));

	return 0;
}

/*----------------------------------------------------------------------------*/
static int MXC622X_ReadCalibration(struct i2c_client *client, int dat[MXC622X_AXES_NUM])
{
	struct mxc622x_i2c_data *obj = i2c_get_clientdata(client);
#ifdef SW_CALIBRATION
	int mul;
#else
	int err;
#endif
#ifdef SW_CALIBRATION
	mul = 0;/* only SW Calibration, disable HW Calibration */
#else

	err = MXC622X_ReadOffset(client, obj->offset);
	if (err) {
		GSE_ERR("read offset fail, %d\n", err);
		return err;
	}
	mul = obj->reso->sensitivity/mxc622x_offset_resolution.sensitivity;
#endif
	dat[obj->cvt.map[MXC622X_AXIS_X]] =
		obj->cvt.sign[MXC622X_AXIS_X]*(obj->offset[MXC622X_AXIS_X]*mul + obj->cali_sw[MXC622X_AXIS_X]);
	dat[obj->cvt.map[MXC622X_AXIS_Y]] =
		obj->cvt.sign[MXC622X_AXIS_Y]*(obj->offset[MXC622X_AXIS_Y]*mul + obj->cali_sw[MXC622X_AXIS_Y]);
	//dat[obj->cvt.map[MXC622X_AXIS_Z]] =
	//	obj->cvt.sign[MXC622X_AXIS_Z]*(obj->offset[MPU6050_AXIS_Z]*mul + obj->cali_sw[MXC622X_AXIS_Z]);

	return 0;
}
/*----------------------------------------------------------------------------*/
static int MXC622X_ReadCalibrationEx(struct i2c_client *client, int act[MXC622X_AXES_NUM], int raw[MXC622X_AXES_NUM])
{
	/*raw: the raw calibration data; act: the actual calibration data */
	struct mxc622x_i2c_data *obj = i2c_get_clientdata(client);
#ifdef SW_CALIBRATION
	int mul;
#else
	int err;
#endif
#ifdef SW_CALIBRATION
	mul = 0;/* only SW Calibration, disable HW Calibration */
#else

	err = MXC622X_ReadOffset(client, obj->offset);
	if (err) {
		GSE_ERR("read offset fail, %d\n", err);
		return err;
	}
	mul = obj->reso->sensitivity / mxc622x_offset_resolution.sensitivity;
#endif

	raw[MXC622X_AXIS_X] = obj->offset[MXC622X_AXIS_X]*mul + obj->cali_sw[MXC622X_AXIS_X];
	raw[MXC622X_AXIS_Y] = obj->offset[MXC622X_AXIS_Y]*mul + obj->cali_sw[MXC622X_AXIS_Y];
	//raw[MXC622X_AXIS_Z] = obj->offset[MPU6050_AXIS_Z]*mul + obj->cali_sw[MXC622X_AXIS_Z];

	act[obj->cvt.map[MXC622X_AXIS_X]] = obj->cvt.sign[MXC622X_AXIS_X] * raw[MXC622X_AXIS_X];
	act[obj->cvt.map[MXC622X_AXIS_Y]] = obj->cvt.sign[MXC622X_AXIS_Y] * raw[MXC622X_AXIS_Y];
	//act[obj->cvt.map[MXC622X_AXIS_Z]] = obj->cvt.sign[MPU6050_AXIS_Z] * raw[MXC622X_AXIS_Z];

	return 0;
}

/*----------------------------------------------------------------------------*/
static int MXC622X_WriteCalibration(struct i2c_client *client, int dat[MXC622X_AXES_NUM])
{
	struct mxc622x_i2c_data *obj = i2c_get_clientdata(client);
	int err;
	int cali[MXC622X_AXES_NUM], raw[MXC622X_AXES_NUM];

	err = MXC622X_ReadCalibrationEx(client, cali, raw);
	if (err) {/*offset will be updated in obj->offset */
		GSE_ERR("read offset fail, %d\n", err);
		return err;
	}

	GSE_LOG("OLDOFF: (%+3d %+3d): (%+3d %+3d) / (%+3d %+3d)\n",
		raw[MXC622X_AXIS_X], raw[MXC622X_AXIS_Y],
		obj->offset[MXC622X_AXIS_X], obj->offset[MXC622X_AXIS_Y],
		obj->cali_sw[MXC622X_AXIS_X], obj->cali_sw[MXC622X_AXIS_Y]);

	/*calculate the real offset expected by caller */
	cali[MXC622X_AXIS_X] += dat[MXC622X_AXIS_X];
	cali[MXC622X_AXIS_Y] += dat[MXC622X_AXIS_Y];
	//cali[MXC622X_AXIS_Z] += dat[MXC622X_AXIS_Z];

	GSE_LOG("UPDATE: (%+3d %+3d)\n", dat[MXC622X_AXIS_X], dat[MXC622X_AXIS_Y]);
#ifdef SW_CALIBRATION
	obj->cali_sw[MXC622X_AXIS_X] = obj->cvt.sign[MXC622X_AXIS_X]*(cali[obj->cvt.map[MXC622X_AXIS_X]]);
	obj->cali_sw[MXC622X_AXIS_Y] = obj->cvt.sign[MXC622X_AXIS_Y]*(cali[obj->cvt.map[MXC622X_AXIS_Y]]);
	//obj->cali_sw[MXC622X_AXIS_Z] = obj->cvt.sign[MXC622X_AXIS_Z]*(cali[obj->cvt.map[MXC622X_AXIS_Z]]);
#else
#if 0
	obj->offset[MXC622X_AXIS_X] =
		(s8)(obj->cvt.sign[MXC622X_AXIS_X]*(cali[obj->cvt.map[MXC622X_AXIS_X]])/(divisor));
	obj->offset[MXC622X_AXIS_Y] =
		(s8)(obj->cvt.sign[MXC622X_AXIS_Y]*(cali[obj->cvt.map[MXC622X_AXIS_Y]])/(divisor));
	obj->offset[MXC622X_AXIS_Z] =
		(s8)(obj->cvt.sign[MXC622X_AXIS_Z]*(cali[obj->cvt.map[MXC622X_AXIS_Z]])/(divisor));

	/*convert software calibration using standard calibration*/
	obj->cali_sw[MXC622X_AXIS_X] = obj->cvt.sign[MXC622X_AXIS_X]*(cali[obj->cvt.map[MXC622X_AXIS_X]])%(divisor);
	obj->cali_sw[MXC622X_AXIS_Y] = obj->cvt.sign[MXC622X_AXIS_Y]*(cali[obj->cvt.map[MXC622X_AXIS_Y]])%(divisor);
	//obj->cali_sw[MXC622X_AXIS_Z] = obj->cvt.sign[MXC622X_AXIS_Z]*(cali[obj->cvt.map[MXC622X_AXIS_Z]])%(divisor);

	GSE_LOG("NEWOFF: (%+3d %+3d %+3d): (%+3d %+3d %+3d) / (%+3d %+3d %+3d)\n",
		obj->offset[MXC622X_AXIS_X]*divisor + obj->cali_sw[MXC622X_AXIS_X],
		obj->offset[MXC622X_AXIS_Y]*divisor + obj->cali_sw[MXC622X_AXIS_Y],
		obj->offset[MXC622X_AXIS_Z]*divisor + obj->cali_sw[MXC622X_AXIS_Z],
		obj->offset[MXC622X_AXIS_X], obj->offset[MXC622X_AXIS_Y], obj->offset[MXC622X_AXIS_Z],
		obj->cali_sw[MXC622X_AXIS_X], obj->cali_sw[MXC622X_AXIS_Y], obj->cali_sw[MXC622X_AXIS_Z]);

	err = hwmsen_write_block(obj->client, MXC622X_REG_OFSX, obj->offset, MXC622X_AXES_NUM);
	if (err) {
		GSE_ERR("write offset fail: %d\n", err);
		return err;
	}
#endif
#endif

	return err;
}

/*----------------------------------------------------------------------------*/
static int MXC622X_CheckDeviceID(struct i2c_client *client)
{
	u8 databuf[10];
	int res = 0;

	memset(databuf, 0, sizeof(u8)*10);

	databuf[0] = MXC622X_REG_CHIP_ID;

	res = i2c_master_send(client, databuf, 0x01);
	if (res <= 0) {
		GSE_ERR("i2c_master_send failed!\n");
		return res;
	}
	udelay(500);

	GSE_LOG("MXC622X_CheckDeviceID 0x%x\n", databuf[0]);
	databuf[0] = 0x0;
	
	res = i2c_master_recv(client, databuf, 0x01);
	if (res <= 0) {
		GSE_ERR("i2c_master_recv failed!\n");
		return res;
	}

	databuf[0] = (databuf[0]&0x3f);

	if (databuf[0]!= MXC622X_ID) {
		GSE_ERR("MXC622X_CheckDeviceID %d failed!\n", databuf[0]);
		return res;
	}

	GSE_LOG("MXC622X_CheckDeviceID %d done!\n ", databuf[0]);

	return MXC622X_SUCCESS;
}

/*----------------------------------------------------------------------------*/
static int MXC622X_SetDataFormat(struct i2c_client *client, u8 dataformat)
{
	struct mxc622x_i2c_data *obj = i2c_get_clientdata(client);
#if 0
	memset(databuf, 0, sizeof(u8)*10);

	if (hwmsen_read_block(client, MXC622X_REG_DATA_FORMAT, databuf, 0x01)) {
		GSE_LOG("mxc622x read Dataformat failt \n");
		return MXC622X_ERR_I2C;
	}

	databuf[0] &= ~MXC622X_RANGE_MASK;
	databuf[0] |= dataformat;
	databuf[1] = databuf[0];
	databuf[0] = MXC622X_REG_DATA_FORMAT;
	res = i2c_master_send(client, databuf, 0x2);

	if (res <= 0) {
		return MXC622X_ERR_I2C;
	}
#endif

	return MXC622X_SetDataResolution(obj);
}

/*----------------------------------------------------------------------------*/
static int MXC622X_SetBWRate(struct i2c_client *client, u8 bwrate)
{
#if 0
	u8 databuf[10];
	int res = 0;

	memset(databuf, 0, sizeof(u8) * 10);

	if (hwmsen_read_block(client, MXC622X_REG_BW_RATE, databuf, 0x01)) {
		GSE_LOG("mxc622x read rate failt \n");
		return MXC622X_ERR_I2C;
	}

	databuf[0] &= ~MXC622X_BW_MASK;
	databuf[0] |= bwrate;
	databuf[1] = databuf[0];
	databuf[0] = MXC622X_REG_BW_RATE;

	res = i2c_master_send(client, databuf, 0x2);

	if (res <= 0) {
		return MXC622X_ERR_I2C;
	}

	GSE_LOG("MXC622X_SetBWRate OK! \n");
#endif
	return MXC622X_SUCCESS;
}

/*----------------------------------------------------------------------------*/
static int MXC622X_SetIntEnable(struct i2c_client *client, u8 intenable)
{
#if 0
	u8 databuf[10];
	int res = 0;

	res = hwmsen_write_byte(client, MXC622X_INT_REG, 0x00);
	if (res != MXC622X_SUCCESS) {
		return res;
	}
	GSE_LOG("MXC622X disable interrupt ...\n");

	/*for disable interrupt function*/
#endif

	return MXC622X_SUCCESS;
}

/*----------------------------------------------------------------------------*/
static int mxc622x_init_client(struct i2c_client *client, int reset_cali)
{
	struct mxc622x_i2c_data *obj = i2c_get_clientdata(client);
	int res = 0;

	res = MXC622X_CheckDeviceID(client);
	if (res != MXC622X_SUCCESS) {
		GSE_ERR("MXC622X_CheckDeviceID failed!\n");
		return res;
	}
	res = MXC622X_SetPowerMode(client, 0);
	if (res != MXC622X_SUCCESS) {
		GSE_ERR("MXC622X_SetPowerMode failed!\n");
		return res;
	}

	res = MXC622X_SetBWRate(client, MXC622X_BW_100HZ);
	if (res != MXC622X_SUCCESS ) {
		GSE_ERR("MXC622X_SetBWRate failed!\n");
		return res;
	}

	res = MXC622X_SetDataFormat(client, MXC622X_RANGE_2G);
	if (res != MXC622X_SUCCESS) {
		GSE_ERR("MXC622X_SetDataFormat failed!\n");
		return res;
	}

	gsensor_gain.x = gsensor_gain.y = gsensor_gain.z = obj->reso->sensitivity;

	res = MXC622X_SetIntEnable(client, 0x00);
	if (res != MXC622X_SUCCESS) {
		GSE_ERR("MXC622X_SetIntEnable failed!\n");
		return res;
	}

	if (0 != reset_cali) {
		/*reset calibration only in power on */
		res = MXC622X_ResetCalibration(client);
		if (res != MXC622X_SUCCESS) {
			GSE_ERR("MXC622X_ResetCalibration failed!\n");
			return res;
		}
	}
#ifdef CONFIG_MXC622X_LOWPASS
	memset(&obj->fir, 0x00, sizeof(obj->fir));
#endif

	mdelay(20);

	return MXC622X_SUCCESS;
}

/*----------------------------------------------------------------------------*/
static int MXC622X_ReadChipInfo(struct i2c_client *client, char *buf, int bufsize)
{
	u8 databuf[10];

	memset(databuf, 0, sizeof(u8) * 10);

	if ((NULL == buf) || (bufsize <= 30))
		return -1;


	if (NULL == client) {
		*buf = 0;
		return -2;
	}

	sprintf(buf, "MXC622X Chip");
	return 0;
}

/*----------------------------------------------------------------------------*/
static int MXC622X_ReadSensorData(struct i2c_client *client, char *buf, int bufsize)
{
	struct mxc622x_i2c_data *obj = (struct mxc622x_i2c_data*)i2c_get_clientdata(client);
	u8 databuf[20];
	int acc[MXC622X_AXES_NUM];
	int res = 0;

	memset(databuf, 0, sizeof(u8)*10);


	if (NULL == buf)
		return -1;

	if (NULL == client) {
		*buf = 0;
		return -2;
	}

	if (sensor_power == false) {
		res = MXC622X_SetPowerMode(client, 1);
		if (res)
			GSE_ERR("Power on mxc622x error %d!\n", res);
	}

	res = MXC622X_ReadData(client, obj->data);
	if (res) {
		GSE_ERR("I2C error: ret value=%d", res);
		return -3;
	}
	obj->data[MXC622X_AXIS_X] += obj->cali_sw[MXC622X_AXIS_X];
	obj->data[MXC622X_AXIS_Y] += obj->cali_sw[MXC622X_AXIS_Y];
	//obj->data[MXC622X_AXIS_Z] += obj->cali_sw[MXC622X_AXIS_Z];

	/*remap coordinate*/
	acc[obj->cvt.map[MXC622X_AXIS_X]] = obj->cvt.sign[MXC622X_AXIS_X]*obj->data[MXC622X_AXIS_X];
	acc[obj->cvt.map[MXC622X_AXIS_Y]] = obj->cvt.sign[MXC622X_AXIS_Y]*obj->data[MXC622X_AXIS_Y];
	//acc[obj->cvt.map[MXC622X_AXIS_Z]] = obj->cvt.sign[MXC622X_AXIS_Z]*obj->data[MXC622X_AXIS_Z];

	if (atomic_read(&obj->trace) & ADX_TRC_RAWDATA) {
		GSE_LOG("Mapped gsensor data: %d, %d!\n", acc[MXC622X_AXIS_X], acc[MXC622X_AXIS_Y]);
	}
	/* Out put the mg */
	acc[MXC622X_AXIS_X] = acc[MXC622X_AXIS_X] * GRAVITY_EARTH_1000 / obj->reso->sensitivity;
	acc[MXC622X_AXIS_Y] = acc[MXC622X_AXIS_Y] * GRAVITY_EARTH_1000 / obj->reso->sensitivity;
	//acc[MXC622X_AXIS_Z] = acc[MXC622X_AXIS_Z] * GRAVITY_EARTH_1000 / obj->reso->sensitivity;

	if (atomic_read(&obj->trace) & ADX_TRC_RAWDATA) {
		GSE_LOG("after * GRAVITY_EARTH_1000 / obj->reso->sensitivity: %d, %d!\n", acc[MXC622X_AXIS_X], acc[MXC622X_AXIS_Y]);
	}

#if defined(LYCONFIG_COMB_CUST_PROJECT_NAME_FACTORY)
	sprintf(buf, "%04x %04x %4x", acc[MXC622X_AXIS_X], acc[MXC622X_AXIS_Y], acc[MXC622X_AXIS_Y]);
#else
	sprintf(buf, "%04x %04x", acc[MXC622X_AXIS_X], acc[MXC622X_AXIS_Y]);
#endif
	if(atomic_read(&obj->trace) & ADX_TRC_IOCTL)
		GSE_LOG("gsensor data: %s!\n", buf);

	return 0;
}

/*----------------------------------------------------------------------------*/
static int MXC622X_ReadRawData(struct i2c_client *client, char *buf)
{
	struct mxc622x_i2c_data *obj = (struct mxc622x_i2c_data*)i2c_get_clientdata(client);
	int res = 0;

	if (!buf || !client)
		return -EINVAL;

	res = MXC622X_ReadData(client, obj->data);
	if (res) {
		GSE_ERR("I2C error: ret value=%d", res);
		return EIO;
	}
	sprintf(buf, "MXC622X_ReadRawData %04x %04x", obj->data[MXC622X_AXIS_X],
		obj->data[MXC622X_AXIS_Y]);

	return 0;
}

/*----------------------------------------------------------------------------*/
static int MXC622X_InitSelfTest(struct i2c_client *client)
{
	int res = 0;
	u8 data;

	res = MXC622X_SetBWRate(client, MXC622X_BW_100HZ);
	if (res != MXC622X_SUCCESS)	{ /* 0x2C->BW=100Hz */
		return res;
	}

	res = mxc622x_i2c_read_block(client, MXC622X_REG_DATA_FORMAT, &data, 1);

	if (res != MXC622X_SUCCESS)
		return res;


	return MXC622X_SUCCESS;
}

/*----------------------------------------------------------------------------*/
static int mxc622x_JudgeTestResult(struct i2c_client *client, s32 prv[MXC622X_AXES_NUM], s32 nxt[MXC622X_AXES_NUM])
{
	struct criteria {
		int min;
		int max;
	};

	struct criteria self[4][3] = {
		{{0, 540}, {0, 540}, {0, 875} },
		{{0, 270}, {0, 270}, {0, 438} },
		{{0, 135}, {0, 135}, {0, 219} },
		{{0, 67}, {0, 67}, {0, 110} },
	};
	struct criteria (*ptr)[3] = NULL;
	u8 format;
	int res;

	res = mxc622x_i2c_read_block(client, MXC622X_REG_DATA_FORMAT, &format, 1);
	if (res)
		return res;

	format = format & MXC622X_RANGE_16G;

	switch (format) {
	case MXC622X_RANGE_2G:
		GSE_LOG("format use self[0]\n");
		ptr = &self[0];
		break;

	case MXC622X_RANGE_4G:
		GSE_LOG("format use self[1]\n");
		ptr = &self[1];
		break;

	case MXC622X_RANGE_8G:
		GSE_LOG("format use self[2]\n");
		ptr = &self[2];
		break;

	case MXC622X_RANGE_16G:
		GSE_LOG("format use self[3]\n");
		ptr = &self[3];
		break;

	default:
		GSE_LOG("format unknown use\n");
		break;
	}

	if (!ptr) {
		GSE_ERR("null pointer\n");
		return -EINVAL;
	}
	GSE_LOG("format=0x%x\n", format);

	GSE_LOG("X diff is %ld\n", abs(nxt[MXC622X_AXIS_X] - prv[MXC622X_AXIS_X]));
	GSE_LOG("Y diff is %ld\n", abs(nxt[MXC622X_AXIS_Y] - prv[MXC622X_AXIS_Y]));


	if ((abs(nxt[MXC622X_AXIS_X] - prv[MXC622X_AXIS_X]) > (*ptr)[MXC622X_AXIS_X].max) ||
		(abs(nxt[MXC622X_AXIS_X] - prv[MXC622X_AXIS_X]) < (*ptr)[MXC622X_AXIS_X].min)) {
		GSE_ERR("X is over range\n");
		res = -EINVAL;
	}
	if ((abs(nxt[MXC622X_AXIS_Y] - prv[MXC622X_AXIS_Y]) > (*ptr)[MXC622X_AXIS_Y].max) ||
		(abs(nxt[MXC622X_AXIS_Y] - prv[MXC622X_AXIS_Y]) < (*ptr)[MXC622X_AXIS_Y].min)) {
		GSE_ERR("Y is over range\n");
		res = -EINVAL;
	}
	/*
	if ((abs(nxt[MXC622X_AXIS_Z] - prv[MXC622X_AXIS_Z]) > (*ptr)[MXC622X_AXIS_Z].max) ||
			(abs(nxt[MXC622X_AXIS_Z] - prv[MXC622X_AXIS_Z]) < (*ptr)[MXC622X_AXIS_Z].min)) {
		GSE_ERR("Z is over range\n");
		res = -EINVAL;
	}
	*/
	return res;
}
/*----------------------------------------------------------------------------*/
static ssize_t show_chipinfo_value(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = mxc622x_i2c_client;
	char strbuf[MXC622X_BUFSIZE];

	if (NULL == client) {
		GSE_ERR("i2c client is null!!\n");
		return 0;
	}

	MXC622X_ReadChipInfo(client, strbuf, MXC622X_BUFSIZE);
	return snprintf(buf, PAGE_SIZE, "%s\n", strbuf);
}

/*----------------------------------------------------------------------------*/
static ssize_t show_sensordata_value(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = mxc622x_i2c_client;
	char strbuf[MXC622X_BUFSIZE];
	if (NULL == client) {
		GSE_ERR("i2c client is null!!\n");
		return 0;
	}
	MXC622X_ReadSensorData(client, strbuf, MXC622X_BUFSIZE);
	//MXC622X_ReadRawData(client, strbuf);
	return snprintf(buf, PAGE_SIZE, "%s\n", strbuf);
}

/*----------------------------------------------------------------------------*/
static ssize_t show_cali_value(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = mxc622x_i2c_client;
	struct mxc622x_i2c_data *obj;
	int err, len = 0, mul;
	int tmp[MXC622X_AXES_NUM];

	if (NULL == client) {
		GSE_ERR("i2c client is null!!\n");
		return 0;
	}

	obj = (struct mxc622x_i2c_data *)i2c_get_clientdata(client);
	err = MXC622X_ReadOffset(client, obj->offset);
	if (err)
		return -EINVAL;
	err = MXC622X_ReadCalibration(client, tmp);
	if (err)
		return -EINVAL;

	mul = obj->reso->sensitivity/mxc622x_offset_resolution.sensitivity;
	len += snprintf(buf+len, PAGE_SIZE-len, "[HW ][%d] (%+3d, %+3d) : (0x%02X, 0x%02X)\n", mul,
		obj->offset[MXC622X_AXIS_X], obj->offset[MXC622X_AXIS_Y],
		obj->offset[MXC622X_AXIS_X], obj->offset[MXC622X_AXIS_Y]);
	len += snprintf(buf+len, PAGE_SIZE-len, "[SW ][%d] (%+3d, %+3d)\n", 1,
		obj->cali_sw[MXC622X_AXIS_X], obj->cali_sw[MXC622X_AXIS_Y]);

	len += snprintf(buf+len, PAGE_SIZE-len, "[ALL](%+3d, %+3d) : (%+3d, %+3d)\n",
		obj->offset[MXC622X_AXIS_X]*mul + obj->cali_sw[MXC622X_AXIS_X],
		obj->offset[MXC622X_AXIS_Y]*mul + obj->cali_sw[MXC622X_AXIS_Y],
		tmp[MXC622X_AXIS_X], tmp[MXC622X_AXIS_Y]);
	return len;
}

/*----------------------------------------------------------------------------*/
static ssize_t store_cali_value(struct device_driver *ddri, const char *buf, size_t count)
{
	struct i2c_client *client = mxc622x_i2c_client;
	int err, x, y;
	int dat[MXC622X_AXES_NUM];

	if (!strncmp(buf, "rst", 3)) {
		err = MXC622X_ResetCalibration(client);
		if (err)
			GSE_ERR("reset offset err = %d\n", err);

	} else if (3 == sscanf(buf, "0x%02X 0x%02X", &x, &y)) {
		dat[MXC622X_AXIS_X] = x;
		dat[MXC622X_AXIS_Y] = y;

		err = MXC622X_WriteCalibration(client, dat);
		if (err)
			GSE_ERR("write calibration err = %d\n", err);

	} else {
		GSE_ERR("invalid format\n");
	}

	return count;
}

/*----------------------------------------------------------------------------*/
static ssize_t show_self_value(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = mxc622x_i2c_client;

	if (NULL == client) {
		GSE_ERR("i2c client is null!!\n");
		return 0;
	}
	return snprintf(buf, 8, "%s\n", selftestRes);
}
/*----------------------------------------------------------------------------*/
static ssize_t store_self_value(struct device_driver *ddri, const char *buf, size_t count)
{/*write anything to this register will trigger the process */
	struct item {
		s16 raw[MXC622X_AXES_NUM];
	};

	struct i2c_client *client = mxc622x_i2c_client;
	int idx, res, num;
	long prv_len, nxt_len;
	struct item *prv = NULL, *nxt = NULL;
	s32 avg_prv[MXC622X_AXES_NUM] = { 0, 0 };
	s32 avg_nxt[MXC622X_AXES_NUM] = { 0, 0 };


	res = kstrtoint(buf, 10, &num);
	if (res != 0) {
		GSE_ERR("parse number fail\n");
		return count;
	} else if (num == 0) {
		GSE_ERR("invalid data count\n");
		return count;
	}
	prv_len = sizeof(*prv) * num;
	nxt_len = sizeof(*nxt) * num;
	prv = kzalloc(prv_len, GFP_KERNEL);
	nxt = kzalloc(prv_len, GFP_KERNEL);
	if (!prv || !nxt)
		goto exit;



	GSE_LOG("NORMAL:\n");
	MXC622X_SetPowerMode(client, true);
	msleep(50);

	for (idx = 0; idx < num; idx++) {
		res = MXC622X_ReadData(client, prv[idx].raw);
		if (res) {
			GSE_ERR("read data fail: %d\n", res);
			goto exit;
		}

		avg_prv[MXC622X_AXIS_X] += prv[idx].raw[MXC622X_AXIS_X];
		avg_prv[MXC622X_AXIS_Y] += prv[idx].raw[MXC622X_AXIS_Y];
		//avg_prv[MXC622X_AXIS_Z] += prv[idx].raw[MXC622X_AXIS_Y];
		GSE_LOG("[%5d %5d]\n", prv[idx].raw[MXC622X_AXIS_X],
			prv[idx].raw[MXC622X_AXIS_Y]/*, prv[idx].raw[MXC622X_AXIS_Z]*/);
	}

	avg_prv[MXC622X_AXIS_X] /= num;
	avg_prv[MXC622X_AXIS_Y] /= num;
	//avg_prv[MXC622X_AXIS_Z] /= num;

	/*initial setting for self test */
	GSE_LOG("SELFTEST:\n");
	for (idx = 0; idx < num; idx++) {
		res = MXC622X_ReadData(client, nxt[idx].raw);
		if (res) {
			GSE_ERR("read data fail: %d\n", res);
			goto exit;
		}
		avg_nxt[MXC622X_AXIS_X] += nxt[idx].raw[MXC622X_AXIS_X];
		avg_nxt[MXC622X_AXIS_Y] += nxt[idx].raw[MXC622X_AXIS_Y];
		//avg_nxt[MXC622X_AXIS_Z] += nxt[idx].raw[MXC622X_AXIS_Y];
		//GSE_LOG("[%5d %5d]\n", nxt[idx].raw[MXC622X_AXIS_X],
		//	nxt[idx].raw[MXC622X_AXIS_Y], nxt[idx].raw[MXC622X_AXIS_Z]);
	}

	avg_nxt[MXC622X_AXIS_X] /= num;
	avg_nxt[MXC622X_AXIS_Y] /= num;
	//avg_nxt[MXC622X_AXIS_Z] /= num;

	GSE_LOG("X: %5d - %5d = %5d\n", avg_nxt[MXC622X_AXIS_X], avg_prv[MXC622X_AXIS_X],
		avg_nxt[MXC622X_AXIS_X] - avg_prv[MXC622X_AXIS_X]);
	GSE_LOG("Y: %5d - %5d = %5d\n", avg_nxt[MXC622X_AXIS_Y], avg_prv[MXC622X_AXIS_Y],
		avg_nxt[MXC622X_AXIS_Y] - avg_prv[MXC622X_AXIS_Y]);
	//GSE_LOG("Z: %5d - %5d = %5d\n", avg_nxt[MXC622X_AXIS_Z], avg_prv[MXC622X_AXIS_Z],
	//	avg_nxt[MXC622X_AXIS_Z] - avg_prv[MXC622X_AXIS_Z]);

	if (!mxc622x_JudgeTestResult(client, avg_prv, avg_nxt)) {
		GSE_LOG("SELFTEST : PASS\n");
		strcpy(selftestRes, "y");
	} else {
		GSE_LOG("SELFTEST : FAIL\n");
		strcpy(selftestRes, "n");
	}

exit:
	/*restore the setting */
	mxc622x_init_client(client, 0);
	kfree(prv);
	kfree(nxt);
	return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t show_selftest_value(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = mxc622x_i2c_client;
	struct mxc622x_i2c_data *obj;

	if (NULL == client) {
		GSE_ERR("i2c client is null!!\n");
		return 0;
	}

	obj = i2c_get_clientdata(client);
	return snprintf(buf, PAGE_SIZE, "%d\n", atomic_read(&obj->selftest));
}

/*----------------------------------------------------------------------------*/
static ssize_t store_selftest_value(struct device_driver *ddri, const char *buf, size_t count)
{
	struct mxc622x_i2c_data *obj = obj_i2c_data;
	int tmp;

	if (NULL == obj) {
		GSE_ERR("i2c data obj is null!!\n");
		return 0;
	}


	if (0 == kstrtoint(buf, 10, &tmp)) {
		if (atomic_read(&obj->selftest) && !tmp) {
			/*enable -> disable */
			mxc622x_init_client(obj->client, 0);
		} else if (!atomic_read(&obj->selftest) && tmp) {
			/*disable -> enable */
			MXC622X_InitSelfTest(obj->client);
		}

		GSE_LOG("selftest: %d => %d\n", atomic_read(&obj->selftest), tmp);
		atomic_set(&obj->selftest, tmp);
	} else {
		GSE_ERR("invalid content: '%s', length = %zu\n", buf, count);
	}
	return count;
}

/*----------------------------------------------------------------------------*/
static ssize_t show_firlen_value(struct device_driver *ddri, char *buf)
{
#ifdef CONFIG_MXC622X_LOWPASS
	struct i2c_client *client = mxc622x_i2c_client;
	struct mxc622x_i2c_data *obj = i2c_get_clientdata(client);

	if (atomic_read(&obj->firlen)) {
		int idx, len = atomic_read(&obj->firlen);

		GSE_LOG("len = %2d, idx = %2d\n", obj->fir.num, obj->fir.idx);

		for (idx = 0; idx < len; idx++) {
			 GSE_LOG("[%5d %5d]\n", obj->fir.raw[idx][MXC622X_AXIS_X], obj->fir.raw[idx][MXC622X_AXIS_Y]);
		}
		GSE_LOG("sum = [%5d %5d]\n", obj->fir.sum[MXC622X_AXIS_X],
			obj->fir.sum[MXC622X_AXIS_Y]);
		GSE_LOG("avg = [%5d %5d]\n", obj->fir.sum[MXC622X_AXIS_X]/len,
			obj->fir.sum[MXC622X_AXIS_Y]/len);
	}
	return snprintf(buf, PAGE_SIZE, "%d\n", atomic_read(&obj->firlen));
#else
	return snprintf(buf, PAGE_SIZE, "not support\n");
#endif
}

/*----------------------------------------------------------------------------*/
static ssize_t store_firlen_value(struct device_driver *ddri, const char *buf, size_t count)
{
#ifdef CONFIG_MXC622X_LOWPASS
	struct i2c_client *client = mxc622x_i2c_client;
	struct mxc622x_i2c_data *obj = i2c_get_clientdata(client);
	int firlen;

	if(1 != sscanf(buf, "%d", &firlen))
		GSE_ERR("invallid format\n");
	} else if (firlen > C_MAX_FIR_LENGTH) {
		GSE_ERR("exceeds maximum filter length\n");
	} else {
		atomic_set(&obj->firlen, firlen);
		if (0 == firlen) {
			atomic_set(&obj->fir_en, 0);
		} else {
			memset(&obj->fir, 0x00, sizeof(obj->fir));
			atomic_set(&obj->fir_en, 1);
		}
	}
#endif
	return count;
}

/*----------------------------------------------------------------------------*/
static ssize_t show_trace_value(struct device_driver *ddri, char *buf)
{
	ssize_t res;
	struct mxc622x_i2c_data *obj = obj_i2c_data;

	if (obj == NULL) {
		GSE_ERR("i2c_data obj is null!!\n");
		return 0;
	}

	res = snprintf(buf, PAGE_SIZE, "0x%04X\n", atomic_read(&obj->trace));
	return res;
}

/*----------------------------------------------------------------------------*/
static ssize_t store_trace_value(struct device_driver *ddri, const char *buf, size_t count)
{
	struct mxc622x_i2c_data *obj = obj_i2c_data;
	int trace;

	if (obj == NULL) {
		GSE_ERR("i2c_data obj is null!!\n");
		return 0;
	}

	if(1 == sscanf(buf, "0x%x", &trace))
		atomic_set(&obj->trace, trace);
	else
		GSE_ERR("invalid content: '%s', length = %d\n", buf, (int)count);


	return count;
}

/*----------------------------------------------------------------------------*/
static ssize_t show_status_value(struct device_driver *ddri, char *buf)
{
	ssize_t len = 0;
	struct mxc622x_i2c_data *obj = obj_i2c_data;

	if (obj == NULL) {
		GSE_ERR("i2c_data obj is null!!\n");
		return 0;
	}

	if (obj->hw)
		len += snprintf(buf+len, PAGE_SIZE-len, "CUST: %d %d (%d %d)\n",
			obj->hw->i2c_num, obj->hw->direction, obj->hw->power_id, obj->hw->power_vol);
	else
		len += snprintf(buf + len, PAGE_SIZE - len, "CUST: NULL\n");

	return len;
}

/*----------------------------------------------------------------------------*/
static ssize_t show_power_status_value(struct device_driver *ddri, char *buf)
{
	if (sensor_power)
		GSE_LOG("G sensor is in work mode, sensor_power = %d\n", sensor_power);
	else
		GSE_LOG("G sensor is in standby mode, sensor_power = %d\n", sensor_power);

	return 0;
}

/*----------------------------------------------------------------------------*/
static ssize_t show_layout_value(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = mxc622x_i2c_client;  
	struct mxc622x_i2c_data *data = i2c_get_clientdata(client);

	return sprintf(buf, "(%d, %d)\n[%+2d %+2d %+2d]\n[%+2d %+2d %+2d]\n",
	data->hw->direction,atomic_read(&data->layout),	data->cvt.sign[0], data->cvt.sign[1],
	data->cvt.sign[2],data->cvt.map[0], data->cvt.map[1], data->cvt.map[2]);
}
/*----------------------------------------------------------------------------*/
static ssize_t store_layout_value(struct device_driver *ddri, const char *buf, size_t count)
{
	struct i2c_client *client = mxc622x_i2c_client;  
	struct mxc622x_i2c_data *data = i2c_get_clientdata(client);
	int layout = 0;

	if (1 == sscanf(buf, "%d", &layout)) {
		atomic_set(&data->layout, layout);
		if (!hwmsen_get_convert(layout, &data->cvt))
			GSE_LOG(KERN_ERR "HWMSEN_GET_CONVERT function error!\r\n");
		else if(!hwmsen_get_convert(data->hw->direction, &data->cvt))
			GSE_LOG(KERN_ERR "invalid layout: %d, restore to %d\n", layout, data->hw->direction);
		else {
			GSE_LOG(KERN_ERR "invalid layout: (%d, %d)\n", layout, data->hw->direction);
			hwmsen_get_convert(0, &data->cvt);
		}
	} else {
		GSE_LOG(KERN_ERR "invalid format = '%s'\n", buf);
	}

	return count;
}

/*----------------------------------------------------------------------------*/
static DRIVER_ATTR(chipinfo, S_IWUSR | S_IRUGO, show_chipinfo_value, NULL);
static DRIVER_ATTR(sensordata, S_IWUSR | S_IRUGO, show_sensordata_value, NULL);
static DRIVER_ATTR(cali, S_IWUSR | S_IRUGO, show_cali_value, store_cali_value);
static DRIVER_ATTR(self, S_IWUSR | S_IRUGO, show_selftest_value, store_selftest_value);
static DRIVER_ATTR(selftest, S_IWUSR | S_IRUGO, show_self_value, store_self_value);
static DRIVER_ATTR(firlen, S_IWUSR | S_IRUGO, show_firlen_value, store_firlen_value);
static DRIVER_ATTR(trace, S_IWUSR | S_IRUGO, show_trace_value, store_trace_value);
static DRIVER_ATTR(layout, S_IRUGO | S_IWUSR, show_layout_value, store_layout_value);
static DRIVER_ATTR(status, S_IRUGO, show_status_value, NULL);
static DRIVER_ATTR(powerstatus, S_IRUGO, show_power_status_value, NULL);
/*----------------------------------------------------------------------------*/
static struct driver_attribute *mxc622x_attr_list[] = {
	&driver_attr_chipinfo,	/*chip information */
	&driver_attr_sensordata,	/*dump sensor data */
	&driver_attr_cali,	/*show calibration data */
	&driver_attr_self,	/*self test demo */
	&driver_attr_selftest,	/*self control: 0: disable, 1: enable */
	&driver_attr_firlen,	/*filter length: 0: disable, others: enable */
	&driver_attr_trace,	/*trace log */
	&driver_attr_layout,
	&driver_attr_status,
	&driver_attr_powerstatus,
};

/*----------------------------------------------------------------------------*/
static int mxc622x_create_attr(struct device_driver *driver) 
{
	int idx, err = 0;
	int num = (int)(sizeof(mxc622x_attr_list)/sizeof(mxc622x_attr_list[0]));

	if (driver == NULL)
		return -EINVAL;


	for (idx = 0; idx < num; idx++) {
		err = driver_create_file(driver, mxc622x_attr_list[idx]);
		if (0 != err) {
			GSE_ERR("driver_create_file (%s) = %d\n", mxc622x_attr_list[idx]->attr.name,
				err);
			break;
		}
	}
	return err;
}

/*----------------------------------------------------------------------------*/
static int mxc622x_delete_attr(struct device_driver *driver)
{
	int idx, err = 0;
	int num = (int)(sizeof(mxc622x_attr_list)/sizeof(mxc622x_attr_list[0]));

	if (driver == NULL)
		return -EINVAL;


	for (idx = 0; idx < num; idx++)
		driver_remove_file(driver, mxc622x_attr_list[idx]);


	return err;
}

/*----------------------------------------------------------------------------*/
static int mxc622x_open(struct inode *inode, struct file *file)
{
	file->private_data = mxc622x_i2c_client;

	if (file->private_data == NULL) {
		GSE_ERR("null pointer!!\n");
		return -EINVAL;
	}
	return nonseekable_open(inode, file);
}

/*----------------------------------------------------------------------------*/
static int mxc622x_release(struct inode *inode, struct file *file)
{
	file->private_data = NULL;
	return 0;
}

/*----------------------------------------------------------------------------*/
//static int mxc622x_ioctl(struct inode *inode, struct file *file, unsigned int cmd,
 //   unsigned long arg)
static long mxc622x_unlocked_ioctl(struct file *file, unsigned int cmd,unsigned long arg)		 
{
	struct i2c_client *client = (struct i2c_client *)file->private_data;
	struct mxc622x_i2c_data *obj = (struct mxc622x_i2c_data*)i2c_get_clientdata(client);  
	char strbuf[MXC622X_BUFSIZE];
	void __user *data;
	struct SENSOR_DATA sensor_data;
	int err = 0;
	int cali[3];

	if (_IOC_DIR(cmd) & _IOC_READ) {
		err = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));
	} else if (_IOC_DIR(cmd) & _IOC_WRITE) {
		err = !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));
	}

	if (err) {
		GSE_ERR("access error: %08X, (%2d, %2d)\n", cmd, _IOC_DIR(cmd), _IOC_SIZE(cmd));
		return -EFAULT;
	}

	switch (cmd) {
		case GSENSOR_IOCTL_INIT:
			mxc622x_init_client(client, 0);
			break;

		case GSENSOR_IOCTL_READ_CHIPINFO:
			data = (void __user *) arg;
			if (data == NULL) {
				err = -EINVAL;
				break;
			}

			MXC622X_ReadChipInfo(client, strbuf, MXC622X_BUFSIZE);
			if (copy_to_user(data, strbuf, strlen(strbuf)+1)) {
				err = -EFAULT;
				break;
			}
			break;

		case GSENSOR_IOCTL_READ_SENSORDATA:
			data = (void __user *) arg;
			if (data == NULL) {
				err = -EINVAL;
				break;
			}

			MXC622X_ReadSensorData(client, strbuf, MXC622X_BUFSIZE);
			if (copy_to_user(data, strbuf, strlen(strbuf)+1)) {
				err = -EFAULT;
				break;
			}
			break;

		case GSENSOR_IOCTL_READ_GAIN:
			data = (void __user *) arg;
			if (data == NULL) {
				err = -EINVAL;
				break;
			}
			if (copy_to_user(data, &gsensor_gain, sizeof(struct GSENSOR_VECTOR3D))) {
				err = -EFAULT;
				break;
			}
			break;

		case GSENSOR_IOCTL_READ_RAW_DATA:
			data = (void __user *) arg;
			if (data == NULL) {
				err = -EINVAL;
				break;
			}
			MXC622X_ReadRawData(client, strbuf);
			if (copy_to_user(data, &strbuf, strlen(strbuf)+1)) {
				err = -EFAULT;
				break;
			}
			break;

		case GSENSOR_IOCTL_SET_CALI:
			data = (void __user*)arg;
			if (data == NULL) {
				err = -EINVAL;
				break;
			}
			if (copy_from_user(&sensor_data, data, sizeof(sensor_data))) {
				 err = -EFAULT;
				 break;
			}
			if (atomic_read(&obj->suspend)) {
				GSE_ERR("Perform calibration in suspend state!!\n");
				err = -EINVAL;
			} else {
				cali[MXC622X_AXIS_X] = sensor_data.x * obj->reso->sensitivity / GRAVITY_EARTH_1000;
				cali[MXC622X_AXIS_Y] = sensor_data.y * obj->reso->sensitivity / GRAVITY_EARTH_1000;
				err = MXC622X_WriteCalibration(client, cali);
			}
			break;

		case GSENSOR_IOCTL_CLR_CALI:
			err = MXC622X_ResetCalibration(client);
			break;

		case GSENSOR_IOCTL_GET_CALI:
			data = (void __user*)arg;
			if (data == NULL) {
				err = -EINVAL;
				break;
			}
			err = MXC622X_ReadCalibration(client, cali);
			if (err) {
				break;
			}

			sensor_data.x = cali[MXC622X_AXIS_X] * GRAVITY_EARTH_1000 / obj->reso->sensitivity;
			sensor_data.y = cali[MXC622X_AXIS_Y] * GRAVITY_EARTH_1000 / obj->reso->sensitivity;

			if (copy_to_user(data, &sensor_data, sizeof(sensor_data))) {
				err = -EFAULT;
				break;
			}
			break;
#if 0
		//begin
		case GSENSOR_IOCTL_GET_DELAY:
			GSE_LOG("Gsensor get delay!!\n");
			break;
		case GSENSOR_IOCTL_GET_STATUS:
			data = (void __user *) arg;
			if (data == NULL) {
				err = -EINVAL;
				break;
			}
			mutex_lock(&mxc622x_mutex);
			if (copy_to_user(data, &enable_status, sizeof(enable_status))) {
				err = -EFAULT;
				mutex_unlock(&mxc622x_mutex);
				break;
			}
			mutex_unlock(&mxc622x_mutex);
			break;

		case GSENSOR_IOCTL_GET_DATA:
			data = (void __user *) arg;
			if (data == NULL) {
				err = -EINVAL;
				break;
			}

			//MXC400X_RxData(client, vec);
			err = MXC622X_ReadSensorData(client, vec,MXC622X_AXES_NUM);

			if (copy_to_user(data, vec, sizeof(vec))) {
				err = -EFAULT;
				break;
			}
			break;

		case GSENSOR_IOCTL_SET_DATA:
			data = (void __user *) arg;
			if (data == NULL) {
				err = -EINVAL;
				break;
			}

			if (copy_from_user(value, data, sizeof(value))) {
				err = -EFAULT;
				break;
			}

			MXC400X_SaveData(value);
			break;

		case GSENSOR_IOCTL_GET_TEMP:
			data = (void __user *) arg;
			if (data == NULL) {
				err = -EINVAL;
				break;
			}
			
			err = MXC400X_ReadTemp(client, &temp);

			if (copy_to_user(data, &temp, sizeof(temp))) {
				err = -EFAULT;
				break;
			}
			break;

		case GSENSOR_IOCTL_GET_DANT:
			data = (void __user *) arg;
			if (data == NULL) {
				err = -EINVAL;
				break;
			}
			err = MXC400X_ReadDant(client, dant);
			//GSE_LOG("dant 2 = %d %d %d %d\n", dant[0], dant[1], dant[2], dant[3]);
			if (copy_to_user(data, dant, sizeof(dant))) {
				err = -EFAULT;
				break;
			}
			break;

		case GSENSOR_IOCTL_SET_TEST1:
			data = (void __user *) arg;
			if (data == NULL) {
				err = -EINVAL;
				break;
			}
			if (copy_from_user(&test, data, sizeof(test))) {
				err = -EFAULT;
				break;
			}

			err = MXC400X_Test1(client, test);
			break;

		case GSENSOR_IOCTL_SET_TEST2:
			data = (void __user *) arg;
			if (data == NULL) {
				err = -EINVAL;
				break;
			}
			if (copy_from_user(&test, data, sizeof(test))) {
				err = -EFAULT;
				break;
			}

			err = MXC400X_Test2(client, test);
			break;

		case GSENSOR_IOCTL_SET_TEST3:
			data = (void __user *) arg;
			if (data == NULL) {
				err = -EINVAL;
				break;
			}
			if (copy_from_user(&test, data, sizeof(test))) {
				err = -EFAULT;
				break;
			}

			err = MXC400X_Test3(client, test);
			break;

		case GSENSOR_IOCTL_READ_REG:
			data = (void __user *) arg;
			if (data == NULL) {
				err = -EINVAL;
				break;
			}
			if (copy_from_user(&test, data, sizeof(test))) {
				err = -EFAULT;
				break;
			}
			if (err = mxc622x_i2c_read_block(client, test, &buf, 1)); {
				GSE_ERR("error: %d\n", err);
			}
			if (copy_to_user(data, &buf, 1)) {
				err = -EFAULT;
				break;
			}
			break;

		case GSENSOR_IOCTL_WRITE_REG:
			data = (void __user *) arg;
			if (data == NULL) {
				err = -EINVAL;
				break;
			}
			if (copy_from_user(reg, data, sizeof(reg))) {
				err = -EFAULT;
				break;
			}
			err = i2c_master_send(client, reg, 0x2);
			if (err <= 0) {
				GSE_LOG("write reg failed!\n");
				err = -EFAULT;
				break;
			}
			break;
#endif
		//end


		default:
			GSE_ERR("unknown IOCTL: 0x%08x\n", cmd);
			err = -ENOIOCTLCMD;
			break;

	}

	return err;
}

/*----------------------------------------------------------------------------*/
#ifdef CONFIG_COMPAT
static long mxc622x_compat_ioctl(struct file *file, unsigned int cmd,unsigned long arg)
{
	long err = 0;
	void __user *arg64 = compat_ptr(arg);

	if (!file->f_op || !file->f_op->unlocked_ioctl)
		return -ENOTTY;

	switch (cmd) {
	case COMPAT_GSENSOR_IOCTL_INIT:
		err = file->f_op->unlocked_ioctl(file, GSENSOR_IOCTL_INIT, (unsigned long)arg64);
		if (err < 0) {
			GSE_LOG(KERN_ERR "COMPAT_MMC31XX_IOC_TM is failed!\n");
		}
		break;

	case COMPAT_GSENSOR_IOCTL_READ_CHIPINFO:
		err = file->f_op->unlocked_ioctl(file, GSENSOR_IOCTL_READ_CHIPINFO, (unsigned long)arg64);
		if(err < 0)
		{
			GSE_LOG(KERN_ERR "COMPAT_GSENSOR_IOCTL_READ_CHIPINFO is failed!\n");
		}
		break;

	case COMPAT_GSENSOR_IOCTL_READ_SENSORDATA:
		err = file->f_op->unlocked_ioctl(file, GSENSOR_IOCTL_READ_SENSORDATA, (unsigned long)arg64);
		if (err) {
			GSE_ERR("GSENSOR_IOCTL_READ_SENSORDATA unlocked_ioctl failed.");
			return err;
		}
		break;
	case COMPAT_GSENSOR_IOCTL_READ_GAIN:
		err = file->f_op->unlocked_ioctl(file, GSENSOR_IOCTL_READ_GAIN, (unsigned long)arg64);
		if (err < 0) {
			GSE_LOG(KERN_ERR "COMPAT_GSENSOR_IOCTL_READ_GAIN is failed!\n");
		}
		break;
	case COMPAT_GSENSOR_IOCTL_READ_RAW_DATA:
		err = file->f_op->unlocked_ioctl(file, GSENSOR_IOCTL_READ_RAW_DATA, (unsigned long)arg64);
		if (err < 0) {
			GSE_LOG(KERN_ERR "COMPAT_GSENSOR_IOCTL_READ_RAW_DATA is failed!\n");
		}
		break;

	case COMPAT_GSENSOR_IOCTL_SET_CALI:
		err = file->f_op->unlocked_ioctl(file, GSENSOR_IOCTL_SET_CALI, (unsigned long)arg64);
		if (err) {
			GSE_ERR("GSENSOR_IOCTL_SET_CALI unlocked_ioctl failed.");
			return err;
		}
		break;

	case COMPAT_GSENSOR_IOCTL_GET_CALI:
		err = file->f_op->unlocked_ioctl(file, GSENSOR_IOCTL_GET_CALI, (unsigned long)arg64);
		if (err) {
			GSE_ERR("GSENSOR_IOCTL_GET_CALI unlocked_ioctl failed.");
			return err;
		}
		break;

	case COMPAT_GSENSOR_IOCTL_CLR_CALI:
		err = file->f_op->unlocked_ioctl(file, GSENSOR_IOCTL_CLR_CALI, (unsigned long)arg64);
		if (err) {
			GSE_ERR("GSENSOR_IOCTL_CLR_CALI unlocked_ioctl failed.");
			return err;
		}
		break;
	case COMPAT_GSENSOR_IOCTL_GET_DELAY:
		err = file->f_op->unlocked_ioctl(file, GSENSOR_IOCTL_GET_DELAY, (unsigned long)arg64);
		if (err < 0) {
			GSE_LOG(KERN_ERR "COMPAT_GSENSOR_IOCTL_GET_DELAY is failed!\n");
		}
		break;

	case COMPAT_GSENSOR_IOCTL_GET_STATUS:
		err = file->f_op->unlocked_ioctl(file, GSENSOR_IOCTL_GET_STATUS, (unsigned long)arg64);
		if (err < 0) {
			GSE_LOG(KERN_ERR "COMPAT_GSENSOR_IOCTL_GET_STATUS is failed!\n");
		}
		break;
	case COMPAT_GSENSOR_IOCTL_GET_DATA:
		err = file->f_op->unlocked_ioctl(file, GSENSOR_IOCTL_GET_DATA, (unsigned long)arg64);
		if (err < 0) {
			GSE_LOG(KERN_ERR "COMPAT_GSENSOR_IOCTL_GET_DATA is failed!\n");
		}
		break;

	case COMPAT_GSENSOR_IOCTL_SET_DATA:
		err = file->f_op->unlocked_ioctl(file, GSENSOR_IOCTL_SET_DATA, (unsigned long)arg64);
		if (err < 0) {
			GSE_LOG(KERN_ERR "COMPAT_GSENSOR_IOCTL_SET_DATA is failed!\n");
		}
		break;
	case COMPAT_GSENSOR_IOCTL_GET_TEMP:
		err = file->f_op->unlocked_ioctl(file, GSENSOR_IOCTL_GET_TEMP, (unsigned long)arg64);
		if (err < 0) {
			GSE_LOG(KERN_ERR "COMPAT_GSENSOR_IOCTL_GET_TEMP is failed!\n");
		}
		break;

	case COMPAT_GSENSOR_IOCTL_GET_DANT:
		err = file->f_op->unlocked_ioctl(file, GSENSOR_IOCTL_GET_DANT, (unsigned long)arg64);
		if (err < 0)
		{
			GSE_LOG(KERN_ERR "COMPAT_GSENSOR_IOCTL_GET_DANT is failed!\n");
		}
		break;
	case COMPAT_GSENSOR_IOCTL_READ_REG:
		err = file->f_op->unlocked_ioctl(file, GSENSOR_IOCTL_READ_REG, (unsigned long)arg64);
		if (err < 0) {
			GSE_LOG(KERN_ERR "COMPAT_GSENSOR_IOCTL_READ_REG is failed!\n");
		}
		break;

	case COMPAT_GSENSOR_IOCTL_WRITE_REG:
		err = file->f_op->unlocked_ioctl(file, GSENSOR_IOCTL_WRITE_REG, (unsigned long)arg64);
		if (err < 0) {
			GSE_LOG(KERN_ERR "COMPAT_GSENSOR_IOCTL_WRITE_REG is failed!\n");
		}
		break;

	default:
		GSE_ERR("unknown IOCTL: 0x%08x\n", cmd);
		err = -ENOIOCTLCMD;
		break;
	}

	return err;
}
#endif

/*----------------------------------------------------------------------------*/
static struct file_operations mxc622x_fops = {
	.owner = THIS_MODULE,
	.open = mxc622x_open,
	.release = mxc622x_release,
	.unlocked_ioctl = mxc622x_unlocked_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = mxc622x_compat_ioctl,
#endif
};

/*----------------------------------------------------------------------------*/
static struct miscdevice mxc622x_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "gsensor",
	.fops = &mxc622x_fops,
};
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
static int mxc622x_suspend(struct i2c_client *client, pm_message_t msg) 
{
	struct mxc622x_i2c_data *obj = i2c_get_clientdata(client);
	int err = 0;

	if (msg.event == PM_EVENT_SUSPEND) {
		if (obj == NULL) {
			GSE_ERR("null pointer!!\n");
			return -EINVAL;
		}
		atomic_set(&obj->suspend, 1);

		err = MXC622X_SetPowerMode(obj->client, 0);
		if (err) {
			GSE_ERR("write power control fail!!\n");
			return err;
		}
		MXC622X_power(obj->hw, 0);
	}
	return err;
}

/*----------------------------------------------------------------------------*/
static int mxc622x_resume(struct i2c_client *client)
{
	struct mxc622x_i2c_data *obj = i2c_get_clientdata(client);
	int err;

	if (obj == NULL) {
		GSE_ERR("null pointer!!\n");
		return -EINVAL;
	}

	MXC622X_power(obj->hw, 1);

	err = mxc622x_init_client(client, 0);
	if (err) {
		GSE_ERR("initialize client fail!!\n");
		return err;
	}
	atomic_set(&obj->suspend, 0);
	GSE_LOG("mxc622x_resume ok\n");

	return 0;
}


/* if use  this typ of enable , Gsensor should report inputEvent(x, y, z ,stats, div) to HAL */
static int mxc622x_open_report_data(int open)
{
	/*should queuq work to report event if  is_report_input_direct=1*/
	return 0;
}

/* if use  this typ of enable , Gsensor only enabled but not report inputEvent to HAL*/

static int mxc622x_enable_nodata(int en)
{
	int res = 0;
	int retry = 0;
	bool power = false;

	if (1 == en)
		power = true;
	if (0 == en)
		power = false;

	for (retry = 0; retry < 3; retry++) {
		res = MXC622X_SetPowerMode(mxc622x_i2c_client, power);
		if (res == 0) {
			GSE_LOG("MXC622X_SetPowerMode done\n");
			break;
		}
		GSE_LOG("MXC622X_SetPowerMode fail\n");
	}

	if (res != 0) {
		GSE_LOG("MXC622X_SetPowerMode fail!\n");
		return -1;
	}
	GSE_LOG("MXC622X_SetPowerMode OK!\n");
	return 0;
}

static int mxc622x_set_delay(u64 ns)
{
	int value = 0;

	value = (int)ns/1000/1000;
	GSE_LOG("mc3xxx_set_delay (%d), chip only use 1024HZ\n", value);
	return 0;
}

static int mxc622x_get_data(int *x ,int *y,int *z, int *status)
{
	char buff[MXC622X_BUFSIZE];

	MXC622X_ReadSensorData(obj_i2c_data->client, buff, MXC622X_BUFSIZE);
	//*x = buff[0];
	//*y = buff[1];
	*z = 512;
	sscanf(buff, "%x %x %x", x, y, z);

	GSE_ERR("mxc622x_get_data x = %d, y = %d!\n", buff[0], buff[1]);
	*status = SENSOR_STATUS_ACCURACY_MEDIUM;

	return 0;
}

/*----------------------------------------------------------------------------*/
static int mxc622x_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct i2c_client *new_client;
	struct mxc622x_i2c_data *obj;
	int err = 0;
	struct acc_control_path ctl = {0};
	struct acc_data_path data = {0};


	obj = kzalloc(sizeof(*obj), GFP_KERNEL);
	if (!(obj)) {
		err = -ENOMEM;
		goto exit;
	}

	memset(obj, 0, sizeof(struct mxc622x_i2c_data));

	obj->hw = mxc622x_get_cust_acc();

#if defined(LYCONFIG_AUTO_PLATFORM_NAME_Z100) || defined(LYCONFIG_AUTO_PLATFORM_NAME_Z100B)
	obj->hw->direction = 1;
#elif defined(LYCONFIG_AUTO_PLATFORM_NAME_Z300) || defined(LYCONFIG_AUTO_PLATFORM_NAME_Z300A)
	obj->hw->direction = 1;
#elif defined(LYCONFIG_AUTO_PLATFORM_NAME_Z5)
	obj->hw->direction = 4;
#endif

	atomic_set(&obj->layout, obj->hw->direction);

	err = hwmsen_get_convert(obj->hw->direction, &obj->cvt);
	if (err) {
		GSE_ERR("invalid direction: %d\n", obj->hw->direction);
		 goto exit_kfree;
	}

	obj_i2c_data = obj;
	obj->client = client;
	/* obj->client->timing = 400; */

	new_client = obj->client;
	i2c_set_clientdata(new_client, obj);

	atomic_set(&obj->trace, 0);
	atomic_set(&obj->suspend, 0);

#ifdef CONFIG_MXC622X_LOWPASS
	if (obj->hw->firlen > C_MAX_FIR_LENGTH)
		atomic_set(&obj->firlen, C_MAX_FIR_LENGTH);
	else
		atomic_set(&obj->firlen, obj->hw->firlen);


	if (atomic_read(&obj->firlen) > 0)
		atomic_set(&obj->fir_en, 1);

#endif

	mxc622x_i2c_client = new_client; 

	err = mxc622x_init_client(new_client, 1);
	if (err)
		goto exit_init_failed;


	err = misc_register(&mxc622x_device);
	if (err) {
		GSE_ERR("mxc622x_device register failed\n");
		goto exit_misc_device_register_failed;
	}

	ctl.is_use_common_factory = 0;
	err = mxc622x_create_attr(&mxc622x_init_info.platform_diver_addr->driver);
	if (err) {
		GSE_ERR("create attribute err = %d\n", err);
		goto exit_create_attr_failed;
	}
	ctl.open_report_data = mxc622x_open_report_data;
	ctl.enable_nodata = mxc622x_enable_nodata;
	ctl.set_delay  = mxc622x_set_delay;
	ctl.is_report_input_direct = false;
	ctl.is_support_batch = obj->hw->is_batch_supported;

	err = acc_register_control_path(&ctl);
	if (err) {
		GSE_ERR("register acc control path err\n");
		goto exit_register_control_path_failed;
	}

	data.get_data = mxc622x_get_data;
	data.vender_div = 1000;
	err = acc_register_data_path(&data);
	if (err) {
		GSE_ERR("register acc data path err= %d\n", err);
		goto exit_register_data_path_failed;
	}
	mxc622x_init_flag = 0;
	GSE_LOG("%s: OK\n", __func__);
	return 0;
exit_register_data_path_failed:
exit_register_control_path_failed:
exit_create_attr_failed:
	misc_deregister(&mxc622x_device);
exit_misc_device_register_failed:
exit_init_failed:
	/* i2c_detach_client(new_client); */
exit_kfree:
	kfree(obj);
exit:
	GSE_ERR("%s: err = %d\n", __func__, err);
	mxc622x_init_flag = -1;
	return -1;
}

/*----------------------------------------------------------------------------*/
static int mxc622x_i2c_remove(struct i2c_client *client)
{
	int err = 0;
	err = mxc622x_delete_attr(&mxc622x_init_info.platform_diver_addr->driver);
	if (err)
		GSE_ERR("mxc622x_delete_attr fail: %d\n", err);

	err = misc_deregister(&mxc622x_device);
	if (err)
		GSE_ERR("misc_deregister fail: %d\n", err);


	mxc622x_i2c_client = NULL;
	obj_i2c_data = NULL;
	i2c_unregister_device(client);
	kfree(i2c_get_clientdata(client));
	return 0;
}
/*----------------------------------------------------------------------------*/

static int  mxc622x_local_init(void)
{
	MXC622X_power(hw, 1);

	if (i2c_add_driver(&mxc622x_i2c_driver)) {
		GSE_ERR("add driver error\n");
		return -1;
	}
	if (-1 == mxc622x_init_flag) {
		return -1;
	}
	return 0;
}

static int mxc622x_remove(void)
{
	MXC622X_power(hw, 0);
	i2c_del_driver(&mxc622x_i2c_driver);
	return 0;
}
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
static int __init mxc622x_init(void)
{
	const char *name = "mediatek,MXC622X";

	hw = get_accel_dts_func(name, hw);
	if (!hw)
		GSE_ERR("mxc622x get dts info fail\n");

	acc_driver_add(&mxc622x_init_info);

	return 0;
}
/*----------------------------------------------------------------------------*/
static void __exit mxc622x_exit(void)
{

}
/*----------------------------------------------------------------------------*/
module_init(mxc622x_init);
module_exit(mxc622x_exit);
/*----------------------------------------------------------------------------*/
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("MXC622X I2C driver");

