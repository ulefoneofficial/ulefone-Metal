/*
 * drivers/misc/tfa98xx.c
 *
 * Copyright (c) 2014, WPI (World Peace Industrial Group).  All rights reserved.
 * Author: Nick Li
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

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/uaccess.h>
#include <linux/sysfs.h>
#include <linux/miscdevice.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <sound/initval.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>

#include <mt-plat/mt_gpio.h>
#include <mach/gpio_const.h>
//#include <tfa98xx/tfa98xx.h>

//#include <cust_gpio_usage.h>
//#include <mach/mt_gpio.h>

/***************** DEBUG ****************/
//#define pr_debug printk
//#define pr_warning printk

/* For MTK platform */
#define USING_MTK_PLATFORM

/* print the bytes of I2C read and write, should be closed when debugging is finished.. */
//#define TEST_DEBUG

/* print the log message, should be closed when debugging is finished.. */
#define TFA98XX_DEBUG

/* Check the chip version and test I2C's operation is OK or not */
#define CHECK_CHIP_VERSION

//for SPRD/ROCKCHIP
//#define I2C_BUS_NUM_STATIC_ALLOC

/***************** DEBUG ****************/

#ifdef USING_MTK_PLATFORM

#include <linux/dma-mapping.h>

/* for MTK I2C burst */
#ifdef CONFIG_MTK_I2C_EXTENSION
#define I2C_USE_DMA
#endif

#ifdef I2C_USE_DMA
static u8 *I2CDMABuf_va;
static dma_addr_t I2CDMABuf_pa;
#endif
#endif

#ifdef TFA98XX_DEBUG
#define PRINT_LOG printk
#else
#define PRINT_LOG(...) 
#endif

struct tfa98xx_dev	
{
	struct mutex		lock;
	struct i2c_client	*client;
	struct miscdevice	tfa98xx_device;
	bool deviceInit;
};

static struct tfa98xx_dev *tfa98xx;

/* tfa98xx I2C defination ++ */
#define TFA98XX_I2C_NAME   "i2c_smartpa"
#define TFA_I2CSLAVEBASE        0x34
#define TFA_I2CSLAVEBASE_R      0x36
#define I2C_STATIC_BUS_NUM        (2)

#define MAX_BUFFER_SIZE	255 
/* tfa98xx I2C defination -- */

#define TFA98XX_WHOAMI 	0x03
#define TFA9890_REV 					0x80
#define TFA9887_REV 					0x12

#define GPIO_SMARTPA_RST_PIN 	(GPIO87 | 0x80000000)	

//static struct i2c_board_info  tfa98xx_i2c_boardinfo = {
//	I2C_BOARD_INFO(TFA98XX_I2C_NAME, TFA_I2CSLAVEBASE), 
//};

static void SetGpioI2s0Mode(void)
{
/*	mt_set_gpio_mode(GPIO78 | 0x80000000, GPIO_MODE_04);
	mt_set_gpio_dir(GPIO78 | 0x80000000, GPIO_DIR_OUT); 

	mt_set_gpio_mode(GPIO79 | 0x80000000, GPIO_MODE_04);
	mt_set_gpio_dir(GPIO79 | 0x80000000, GPIO_DIR_OUT); 

	mt_set_gpio_mode(GPIO80 | 0x80000000, GPIO_MODE_04);
	mt_set_gpio_dir(GPIO80 | 0x80000000, GPIO_DIR_OUT); 
*/
}


#ifdef I2C_BUS_NUM_STATIC_ALLOC
int i2c_static_add_device(struct i2c_board_info *info)
{
	struct i2c_adapter *adapter;
	struct i2c_client *client;
	int err;

	adapter = i2c_get_adapter(I2C_STATIC_BUS_NUM);
	if (!adapter) {
		printk("tfa98xx %s: can't get i2c adapter\n", __FUNCTION__);
		err = -ENODEV;
		goto i2c_err;
	}

	client = i2c_new_device(adapter, info);
	if (!client) {
		pr_err("tfa98xx %s:  can't add i2c device at 0x%x\n",
			__FUNCTION__, (unsigned int)info->addr);
		err = -ENODEV;
		goto i2c_err;
	}

	i2c_put_adapter(adapter);

	return 0;

i2c_err:
	return err;
}

#endif /*I2C_BUS_NUM_STATIC_ALLOC*/

//////////////////////// i2c R/W ////////////////////////////

#ifdef I2C_USE_DMA
static int tfa_i2c_write(struct i2c_client *client, const uint8_t *buf, int len)
{
	int i = 0;
	for(i = 0 ; i < len; i++)
	{
		I2CDMABuf_va[i] = buf[i];
	}

	if(len < 8)
	{
		client->addr = client->addr & I2C_MASK_FLAG;
		client->ext_flag = ((client->ext_flag & I2C_MASK_FLAG ) | I2C_ENEXT_FLAG);
		client->timing = 400;
		return i2c_master_send(client, buf, len);
	}
	else
	{
		client->addr = client->addr & I2C_MASK_FLAG;
		client->ext_flag = ((client->ext_flag & I2C_MASK_FLAG ) | I2C_ENEXT_FLAG | I2C_DMA_FLAG);
		client->timing = 400;
		return i2c_master_send(client, (unsigned char *)I2CDMABuf_pa, len);
	}    
}

static int tfa_i2c_read(struct i2c_client *client, uint8_t *buf, int len)
{
    int i = 0, ret = 0;
    
    if(len < 8)
    {
        client->addr = client->addr & I2C_MASK_FLAG;
		client->ext_flag = ((client->ext_flag & I2C_MASK_FLAG )| I2C_ENEXT_FLAG);
		client->timing = 400;
        return i2c_master_recv(client, buf, len);
    }
    else
    {
        client->addr = client->addr & I2C_MASK_FLAG;
		client->ext_flag = ((client->ext_flag & I2C_MASK_FLAG )| I2C_ENEXT_FLAG | I2C_DMA_FLAG);
		client->timing = 400;
        ret = i2c_master_recv(client, (unsigned char *)I2CDMABuf_pa, len);
    
        if(ret < 0)
        {
            printk("%s: i2c_master_recv(len = %d) returned %d.\n", __func__, len, ret);
            return ret;
        }
    
        for(i = 0; i < len; i++)
        {
            buf[i] = I2CDMABuf_va[i];
        }
    }
    client->addr = client->addr & I2C_MASK_FLAG;

    return ret;
}
#endif

static ssize_t tfa98xx_dev_read(struct file *filp, char __user *buf, size_t count, loff_t *offset)
{
	char tmp[MAX_BUFFER_SIZE];
	int ret=0;
#ifdef TEST_DEBUG
	int i;
#endif
//SetGpioI2s0Mode();

	if (count > MAX_BUFFER_SIZE)
	{
		count = MAX_BUFFER_SIZE;
		printk("tfa98xx_dev_read====qianzhibin===count > MAX_BUFFER_SIZE");
	}

	//PRINT_LOG("%s : reading %zu bytes.\n", __func__, count);

#ifdef I2C_USE_DMA
    //tfa98xx->client->addr = tfa98xx->client->addr & I2C_MASK_FLAG | I2C_DMA_FLAG | I2C_ENEXT_FLAG;
    mutex_lock(&tfa98xx->lock);
    ret = tfa_i2c_read(tfa98xx->client, tmp, count);
    mutex_unlock(&tfa98xx->lock);
    tfa98xx->client->addr = tfa98xx->client->addr & I2C_MASK_FLAG;
    if (ret <0) 
    {
        printk("%s: tfa_i2c_read returned %d\n", __func__, ret);
        return ret;
    }
#else
	/* Read data */
	int pos=0;
	mutex_lock(&tfa98xx->lock);
	while(pos!=count)
	{
		if((count-pos)>8)
		{
			ret = i2c_master_recv(tfa98xx->client, &tmp[pos], 8);
			pos += 8;
		}
		else
		{
			ret = i2c_master_recv(tfa98xx->client, tmp, count-pos);
			pos += (count-pos);
		}
		if (ret < 0) 
		{
			printk("%s: i2c_master_recv returned %d\n", __func__, ret);
			break;
		}
	}
	mutex_unlock(&tfa98xx->lock);

	if (ret < 0) 
	{
		printk("%s: i2c_master_recv returned %d\n", __func__, ret);
		return ret;
	}
#endif
	if (ret > count) 
	{
		printk("%s: received too many bytes from i2c (%d)\n", __func__, ret);
		return -EIO;
	}
	if (copy_to_user(buf, tmp, ret)) 
	{
		pr_warning("%s : failed to copy to user space\n", __func__);
		return -EFAULT;
	}
	
#ifdef TEST_DEBUG
	PRINT_LOG("Read from tfa98xx:");
	for(i = 0; i < ret; i++)
	{
		PRINT_LOG(" %02X", tmp[i]);
	}
	PRINT_LOG("\n");
#endif
	
	return ret;
}

static ssize_t tfa98xx_dev_write(struct file *filp, const char __user *buf, size_t count, loff_t *offset)
{
	struct tfa98xx_dev  *tfa98xx_dev;
	char tmp[MAX_BUFFER_SIZE];
	int ret;
#ifdef TEST_DEBUG
	int i;
#endif
//SetGpioI2s0Mode();

	tfa98xx_dev = filp->private_data;

	if (count > MAX_BUFFER_SIZE)
	{
		count = MAX_BUFFER_SIZE;
		printk("tfa98xx_dev_write====qianzhibin===count > MAX_BUFFER_SIZE");
	}
	if (copy_from_user(tmp, buf, count)) 
	{
		printk("%s : failed to copy from user space\n", __func__);
		return -EFAULT;
	}

	//PRINT_LOG("%s : writing %zu bytes.\n", __func__, count);
#ifdef I2C_USE_DMA
    //tfa98xx->client->addr = tfa98xx->client->addr & I2C_MASK_FLAG | I2C_DMA_FLAG | I2C_ENEXT_FLAG;
	mutex_lock(&tfa98xx->lock);
    ret = tfa_i2c_write(tfa98xx->client, tmp, count);
	mutex_unlock(&tfa98xx->lock);
    tfa98xx->client->addr = tfa98xx->client->addr & I2C_MASK_FLAG;
	if (ret <0) 
	{
		printk("%s: tfa_i2c_write returned %d\n", __func__, ret);
		return ret;
	}

#else
	/* Write data */
	int pos = 0;
	mutex_lock(&tfa98xx->lock);
	while(pos!=count)
	{
		if((count-pos)>8)
		{
			ret = i2c_master_send(tfa98xx->client, &tmp[pos], 8);
			pos += 8;
		}
		else
		{
			ret = i2c_master_send(tfa98xx->client, &tmp[pos], count-pos);
			pos += (count-pos);
		}
		if (ret < 0) 
		{
			printk("%s: i2c_master_send returned %d\n", __func__, ret);
			break;
		}
	}

	mutex_unlock(&tfa98xx->lock);
	if (ret < 0) 
	{
		printk("%s : i2c_master_send returned %d\n", __func__, ret);
		return ret;
	}
#endif


#ifdef TEST_DEBUG
	PRINT_LOG("Write to tfa98xx:");
	for(i = 0; i < count; i++)
	{
		PRINT_LOG(" %02X", tmp[i]);
	}
	PRINT_LOG("\n");
#endif	
	return ret;
}
//////////////////////// i2c R/W ////////////////////////////

static int tfa98xx_dev_open(struct inode *inode, struct file *filp)
{

	struct tfa98xx_dev *tfa98xx_dev = container_of(filp->private_data, struct tfa98xx_dev, tfa98xx_device);
SetGpioI2s0Mode();

	filp->private_data = tfa98xx_dev;

	pr_debug("%s : %d,%d\n", __func__, imajor(inode), iminor(inode));

	return 0;
}

#ifdef LYCONFIG_KERNEL_SMARTPA_ESD_CHECK
#define I2C_NXP_RESET	0x0730	/* reset the nxp chip */
extern void smartpa_reset_gpio_output( int level );
#endif
static long tfa98xx_dev_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	//void __user *argp = (void __user *)arg;
	//int mode = 0;

	switch (cmd) 
	{
		/* This part do not work ok now , it is used for stereo application. */
		case I2C_SLAVE:
		case I2C_SLAVE_FORCE:
			if((arg == TFA_I2CSLAVEBASE) ||(arg == TFA_I2CSLAVEBASE_R))
			{
			    tfa98xx->client->addr = arg;
			    return 0;
			}
			else
			    return -1;
	#ifdef LYCONFIG_KERNEL_SMARTPA_ESD_CHECK
		case I2C_NXP_RESET:
 			printk("%s ioctl ==I2C_NXP_RESET \n", __func__);
			smartpa_reset_gpio_output( 1 );
			msleep(100);
			smartpa_reset_gpio_output( 0 );
			msleep(100);
			return 0;
	#endif
		default:
			printk("%s bad ioctl %u\n", __func__, cmd);
			return -EINVAL;
	}
	return 0;
}

static const struct file_operations tfa98xx_dev_fops = 
{
	.owner	= THIS_MODULE,
	.open	= tfa98xx_dev_open,
	.unlocked_ioctl = tfa98xx_dev_ioctl,
	.llseek	= no_llseek,
	.read	= tfa98xx_dev_read,
	.write	= tfa98xx_dev_write,
};

static int tfa98xx_i2c_probe(struct i2c_client *client,
				      const struct i2c_device_id *id)
{
	int ret = 0;
	int read_id = 0;
	int rev_value = 0;
	
	printk("%s +\n", __func__);
SetGpioI2s0Mode();
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) 
	{
		printk("%s : need I2C_FUNC_I2C\n", __func__);
		return  -ENODEV;
	}

	tfa98xx = kzalloc(sizeof(*tfa98xx), GFP_KERNEL);
	if (tfa98xx == NULL) 
	{
		dev_err(&client->dev,
				"failed to allocate memory for module data\n");
		ret = -ENOMEM;
		goto err_exit;
	}
	
	i2c_set_clientdata(client, tfa98xx);
	tfa98xx->client   = client;
	
	/* init mutex and queues */
	mutex_init(&tfa98xx->lock);

#ifdef CHECK_CHIP_VERSION
	mutex_lock(&tfa98xx->lock);
	read_id = i2c_smbus_read_word_data(client, TFA98XX_WHOAMI);
	rev_value = ((read_id & 0x00FF)<< 8) | ((read_id & 0xFF00)>> 8);
	rev_value = rev_value & 0xFFFF;
	mutex_unlock(&tfa98xx->lock);

	PRINT_LOG("tfa98xx_i2c_probe:rev_value=0x%x\n", rev_value);

	if (rev_value == TFA9887_REV) 
	{
 		printk("NXP Device detected!\nTFA9887 registered I2C driver!\n");
	}
	else if(rev_value == TFA9890_REV)
	{
 		printk("NXP Device detected!\nTFA9890 registered I2C driver!\n");
	}
	else
	{
		printk("NXP Device not found, i2c error %d \n", rev_value);
		ret = -1;
		goto i2c_error;
	}
#endif

	tfa98xx->tfa98xx_device.minor = MISC_DYNAMIC_MINOR;
	tfa98xx->tfa98xx_device.name = TFA98XX_I2C_NAME;
	tfa98xx->tfa98xx_device.fops = &tfa98xx_dev_fops;

	ret = misc_register(&tfa98xx->tfa98xx_device);
	if (ret) 
	{
		printk("%s : misc_register failed\n", __FILE__);
		ret = -1;
		goto err_misc_register;
	}

#ifdef I2C_USE_DMA
    	//msleep(50);
        tfa98xx->tfa98xx_device.this_device->coherent_dma_mask = DMA_BIT_MASK(32);
    	I2CDMABuf_va = (u8 *)dma_alloc_coherent(tfa98xx->tfa98xx_device.this_device, MAX_BUFFER_SIZE, &I2CDMABuf_pa, GFP_KERNEL);

    	if(!I2CDMABuf_va)
    	{
			printk("Allocate TFA98XX DMA I2C Buffer failed!\n");
			ret = -1;
			goto err_misc_register;

    	}
	memset(I2CDMABuf_va, 0, MAX_BUFFER_SIZE);

#endif


	if (tfa98xx) {
		mutex_lock(&tfa98xx->lock);
		tfa98xx->deviceInit = true;
		mutex_unlock(&tfa98xx->lock);
	}

	printk("%s is OK! -\n", __func__);
	return 0;

err_misc_register:
	misc_deregister(&tfa98xx->tfa98xx_device);
i2c_error:
	mutex_destroy(&tfa98xx->lock);
	kfree(tfa98xx);
	tfa98xx = NULL;
err_exit:
	return ret;
}


static int tfa98xx_i2c_remove(struct i2c_client *client)
{
	struct tfa98xx_dev *tfa98xx_dev;

	PRINT_LOG("Enter %s.  %d\n", __FUNCTION__, __LINE__);
#ifdef I2C_USE_DMA
    if(I2CDMABuf_va)
    {
		dma_free_coherent(NULL, MAX_BUFFER_SIZE, I2CDMABuf_va, I2CDMABuf_pa);
		I2CDMABuf_va = NULL;
		I2CDMABuf_pa = 0;
    }
#endif
	tfa98xx_dev = i2c_get_clientdata(client);
	misc_deregister(&tfa98xx_dev->tfa98xx_device);
	mutex_destroy(&tfa98xx_dev->lock);
	kfree(tfa98xx_dev);
	
	return 0;
}

static void tfa98xx_i2c_shutdown(struct i2c_client *i2c)
{
	PRINT_LOG("Enter %s. +  %4d\n", __FUNCTION__, __LINE__);
}

#ifdef CONFIG_OF
static const struct of_device_id tfa98xx_of_match[] = {
//	{ .compatible = "nxp,i2c_smartpa", },
	{ .compatible = "mediatek,i2c_smartpa_ly", },
	{},
};
#endif
MODULE_DEVICE_TABLE(of, tfa98xx_of_match);

static const struct i2c_device_id tfa98xx_i2c_id[] = {
	{ TFA98XX_I2C_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, tfa98xx_i2c_id);

static struct i2c_driver tfa98xx_i2c_driver = {
	.driver = {
		.name = TFA98XX_I2C_NAME,
		.owner = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = tfa98xx_of_match,
#endif
	},
	.probe =    tfa98xx_i2c_probe,
	.remove =   tfa98xx_i2c_remove,
	.id_table = tfa98xx_i2c_id,
	.shutdown = tfa98xx_i2c_shutdown,
};

static int __init tfa98xx_modinit(void)
{
	int ret = 0;
	mt_set_gpio_mode(GPIO_SMARTPA_RST_PIN, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_SMARTPA_RST_PIN, GPIO_DIR_OUT); 
	mt_set_gpio_out(GPIO_SMARTPA_RST_PIN,GPIO_OUT_ONE);
	msleep(100);
	mt_set_gpio_out(GPIO_SMARTPA_RST_PIN,GPIO_OUT_ZERO);
	msleep(100);

	PRINT_LOG("Loading tfa98xx driver\n");
#if 0
	//for mtk platform...
	ret = i2c_register_board_info(I2C_STATIC_BUS_NUM, &tfa98xx_i2c_boardinfo, 1 /* ARRAYSIZE(tfa98xx_i2c_boardinfo) */);
	if (ret < 0) {
		printk("i2c_register_board_info error %d\n", ret);
	}
#endif
	ret = i2c_add_driver(&tfa98xx_i2c_driver);
	if (ret != 0) {
	printk("Failed to register tfa98xx I2C driver: %d\n",
	    ret);
	}
	return ret;
}

subsys_initcall(tfa98xx_modinit);

static void __exit tfa98xx_exit(void)
{
	i2c_del_driver(&tfa98xx_i2c_driver);
}
module_exit(tfa98xx_exit);

MODULE_AUTHOR("Nick Li <nick.li@wpi-group.com>");
MODULE_DESCRIPTION("TFA98xx Smart Audio I2C driver");
MODULE_LICENSE("GPL");

