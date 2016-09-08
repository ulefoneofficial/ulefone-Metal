/*
 * Simple synchronous userspace interface to SPI devices
 *
 * Copyright (C) 2006 SWAPP
 *	Andrea Paterniani <a.paterniani@swapp-eng.it>
 * Copyright (C) 2007 David Brownell (simplification, cleanup)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/spi/spi.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/wait.h>
#include <linux/kthread.h>
#include <linux/ioctl.h>
#include <linux/input.h>   
#include <linux/device.h>
#include <linux/err.h>
#include <linux/list.h>
#include <linux/errno.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/compat.h>
#include <linux/poll.h>
#include <linux/sort.h>
#include <linux/types.h>
#include <linux/version.h>
#include <linux/spi/spidev.h>
#include <linux/spi/spi.h>
#include <linux/usb.h>
#include <linux/usb/ulpi.h>
#include <linux/usb/ch9.h>
#include <linux/usb/gadget.h>
#include <linux/power_supply.h>
#include <asm/uaccess.h>
#include <linux/ktime.h>
#include <linux/interrupt.h>
#include <linux/of_irq.h>
#include <linux/completion.h>
#include <linux/proc_fs.h>  
#include <linux/seq_file.h>
#include <linux/kobject.h>

#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#endif  
#include <mach/gpio_const.h>
#include <mt-plat/mt_gpio.h>
#include "et310.h"

#define  TRANSFER_MODE_DMA
#ifndef SPI_PACKET_SIZE
#define SPI_PACKET_SIZE 	 0x400
#define SPI_PACKET_COUNT    0x100
#define SPI_FIFO_SIZE 32
#endif


#define FP_SPI_DEBUG
#ifdef FP_SPI_DEBUG
//#define DEBUG_PRINT(fmt, args...)	pr_err(fmt, ## args)
#define DEBUG_PRINT(fmt, args...)	printk(fmt, ## args)
#else
/* Don't do anything in release builds */
#define DEBUG_PRINT(fmt, args...)
#endif

#define LGE_TEST 1
#define FPC_BTP_SPI_CLOCK_SPEED			8*1000*1000
#define MASS_READ_TRANSMITION_SIZE 2304

#define EGIS_MASS_READ_SEGMENT_COUNT 10

static DECLARE_BITMAP(minors, N_SPI_MINORS);
static LIST_HEAD(device_list);
static DEFINE_MUTEX(device_list_lock);
static unsigned bufsiz = 4096;
module_param(bufsiz, uint, S_IRUGO);
MODULE_PARM_DESC(bufsiz, "data bytes in biggest supported SPI message");


extern void fingerprint_power_en_gpio_output( int level);
extern void fingerprint_reset_gpio_output( int level);
extern void fingerprint_spi_set_gpio_mode( int spi_pin_mode);

unsigned int fingerpirnt_irq = 0;
#ifdef CONFIG_OF
//static const struct of_device_id fp_of_match[] = {
//	{.compatible = "mediatek, fingerprint",},
//	{},
//};
#endif
/*-----------------MTK spi configure-----------------*/
static struct mt_chip_conf spi_conf =
{
	.setuptime = 1,
	.holdtime = 1,
	.high_time = 4,  //  4
	.low_time = 4,   //  4
	.cs_idletime = 2,
	.ulthgh_thrsh = 0,
	.cpol = 1,
	.cpha = 1,
	.rx_mlsb = SPI_MSB, 
	.tx_mlsb = SPI_MSB,
	.tx_endian = 0,
	.rx_endian = 0,
#ifdef TRANSFER_MODE_DMA
	.com_mod = DMA_TRANSFER,
#else
	.com_mod = FIFO_TRANSFER,
#endif
	.pause = 0,
	.finish_intr = 1,
	.deassert = 0,
	.ulthigh = 0,
	.tckdly = 0,
};

static struct mt_chip_conf spi_conf_fifo =
{
	.setuptime = 1,
	.holdtime = 1,
	.high_time = 4,
	.low_time = 4,
	.cs_idletime = 2,
	.ulthgh_thrsh = 0,
	.cpol = 1,
	.cpha = 1,
	.rx_mlsb = SPI_MSB, 
	.tx_mlsb = SPI_MSB,
	.tx_endian = 0,
	.rx_endian = 0,
	.com_mod = FIFO_TRANSFER,
	.pause = 0,
	.finish_intr = 1,
	.deassert = 0,
	.ulthigh = 0,
	.tckdly = 0,
};

static struct mt_chip_conf spi_conf_dma =
{
	.setuptime = 1,
	.holdtime = 1,
	.high_time = 4,  // 4
	.low_time = 4,   // 4
	.cs_idletime = 2,
	.ulthgh_thrsh = 0,
	.cpol = 1,
	.cpha = 1,
	.rx_mlsb = SPI_MSB, 
	.tx_mlsb = SPI_MSB,
	.tx_endian = 0,
	.rx_endian = 0,
	.com_mod = DMA_TRANSFER,
	.pause = 0,
	.finish_intr = 1,
	.deassert = 0,
	.ulthigh = 0,
	.tckdly = 0,
};

/*--------------------------- Data Transfer -----------------------------*/

//static struct spi_transfer msg_xfer[EGIS_MASS_READ_SEGMENT_COUNT];
int fp_mass_read(struct fp_data *fp, u8 addr, u8 *buf, int read_len)
{
#if 1
	ssize_t status;
	struct spi_device *spi;
	struct spi_message m;
	u8 *write_addr;
	u32 spi_transfer_len = (read_len + 3) > SPI_PACKET_SIZE ? (read_len + 3 + SPI_PACKET_SIZE - 1) / SPI_PACKET_SIZE * SPI_PACKET_SIZE : read_len + 3;
	/* Set start address */
	u8 *read_data = kzalloc(spi_transfer_len, GFP_KERNEL);
	struct spi_transfer t_set_addr = {
		.tx_buf = NULL,
		.len = 2,
	};

	/* Write and read data in one data query section */
	struct spi_transfer t_read_data = {
		.tx_buf = NULL,
		.rx_buf = NULL,
		.len = spi_transfer_len,
	};

	DEBUG_PRINT("%s", __func__);

	if (read_data == NULL)
		return -ENOMEM;

	write_addr = kzalloc(2, GFP_KERNEL);
	write_addr[0] = FP_WRITE_ADDRESS;
	write_addr[1] = addr;

	DEBUG_PRINT("%s read_len = %d\n", __func__, read_len);

	read_data[0] = FP_READ_DATA;

	t_set_addr.tx_buf = write_addr;
	t_read_data.tx_buf = t_read_data.rx_buf = read_data;

	spi = fp->spi;
	spi_message_init(&m);
	spi_message_add_tail(&t_set_addr, &m);
	spi_message_add_tail(&t_read_data, &m);
	status = spi_sync(spi, &m);

	kfree(write_addr);

	if (status == 0)
		memcpy(buf, read_data + 3, read_len);
	else
		pr_err("%s read data error status = %ld\n"
				, __func__, status);
	kfree(read_data);

	return status;
#else

	ssize_t status = 0;
	struct spi_device *spi;
	struct spi_message m;
	u8 write_addr[] = {FP_WRITE_ADDRESS, addr};
	struct spi_transfer t_set_addr = {
		.tx_buf = write_addr,
		.len = 2,
	};
	u8 read_data[] = {FP_READ_DATA, 0x0, 0x0};
	struct spi_transfer t_read_data = {
		.tx_buf = read_data,
		.len = 3
	};
	int const msg_count = read_len / MASS_READ_TRANSMITION_SIZE;
	int const msg_remainder = read_len % MASS_READ_TRANSMITION_SIZE;
	int msg_index;

	spi = fp->spi;
	spi_message_init(&m);
	spi_message_add_tail(&t_set_addr, &m);
	status = spi_sync(spi, &m);
	if (status < 0) {
//		pr_err("%s read data error status = %d", __func__, status);
		goto out;
	}

	/*Set read data*/

	spi_message_init(&m);
	spi_message_add_tail(&t_read_data, &m);

	/*Seperate msgs into transmition size*/
	//memset(msg_xfer, 0x00, sizeof(msg_xfer));
	memset(msg_xfer, 0x00, sizeof(struct spi_transfer)*EGIS_MASS_READ_SEGMENT_COUNT);
	for (msg_index = 0; msg_index < msg_count; msg_index++) {
		msg_xfer[msg_index].tx_buf = buf + msg_index * MASS_READ_TRANSMITION_SIZE;
		msg_xfer[msg_index].rx_buf = buf + msg_index * MASS_READ_TRANSMITION_SIZE;
		msg_xfer[msg_index].len = MASS_READ_TRANSMITION_SIZE;
		spi_message_add_tail(&msg_xfer[msg_index], &m);
	}
	if (msg_remainder) {
		msg_xfer[msg_index].tx_buf = buf + msg_index * MASS_READ_TRANSMITION_SIZE;
		msg_xfer[msg_index].rx_buf = buf + msg_index * MASS_READ_TRANSMITION_SIZE;
		msg_xfer[msg_index].len = msg_remainder;
		spi_message_add_tail(&msg_xfer[msg_index], &m);
	}
	status = spi_sync(spi, &m);
	if (status < 0) {
//		pr_err("%s read data error status = %d", __func__, status);
		goto out;
	}
out:
#endif	
	return status;
}

/*
 * Read io register
 */
int fp_io_read_register(struct fp_data *fp, u8 *addr, u8 *buf)
{
	ssize_t status = 0;
	struct spi_device *spi;
	struct spi_message m;
	int read_len = 1;

	u8 write_addr[] = {FP_WRITE_ADDRESS, 0x00};
	u8 read_value[] = {FP_READ_DATA, 0x00};
	u8 val, addrval;

	u8 result[] = {0xFF, 0xFF};
	struct spi_transfer t_set_addr = {
	
		.speed_hz = FPC_BTP_SPI_CLOCK_SPEED,
		.tx_buf = write_addr,
		.len = 2,
	};
	struct spi_transfer t = {
		.speed_hz = FPC_BTP_SPI_CLOCK_SPEED,
		.tx_buf = read_value,
		.rx_buf = result,
		.len = 2,
	};

	DEBUG_PRINT("%s", __func__);

	if (copy_from_user(&addrval, (const u8 __user *) (uintptr_t) addr
		, read_len)) {
		pr_err("%s buffer copy_from_user fail", __func__);
		status = -EFAULT;
		return status;
	}

	DEBUG_PRINT("%s read_len = %d\n", __func__, read_len);

	spi = fp->spi;

	/*Set address*/
	write_addr[1] = addrval;

	/*Start to read*/
	spi_message_init(&m);
	spi_message_add_tail(&t_set_addr, &m);
	#if 1//deleted by xhf
	status = spi_sync(spi, &m);
	if (status < 0) {
		pr_err("%s read data error status = %ld\n"
				, __func__, status);
		return status;
	}
	spi_message_init(&m);
	#endif
	spi_message_add_tail(&t, &m);
	status = spi_sync(spi, &m);

	if (status < 0) {
		pr_err("%s read data error status = %ld\n"
				, __func__, status);
		return status;
	}

	val = result[1];

	DEBUG_PRINT("%s Read add_val = %x buf = 0x%x\n", __func__,
			addrval, val);

	if (copy_to_user((u8 __user *) (uintptr_t) buf, &val, read_len)) {
		pr_err("%s buffer copy_to_user fail status", __func__);
		status = -EFAULT;
		return status;
	}

	return status;
}

/*
 * Write data to register
 */
int fp_io_write_register(struct fp_data *fp, u8 *buf)
{
	ssize_t status = 0;
	struct spi_device *spi;
	int write_len = 2;
	struct spi_message m;

	u8 write_addr[] = {FP_WRITE_ADDRESS, 0x00};
	u8 write_value[] = {FP_WRITE_DATA, 0x00};
	u8 val[2];

	struct spi_transfer t1 = {
		.speed_hz = FPC_BTP_SPI_CLOCK_SPEED,
		.tx_buf = write_addr,
		.len = 2,
	};
	struct spi_transfer t2 = {
		.speed_hz = FPC_BTP_SPI_CLOCK_SPEED,
		.tx_buf = write_value,
		.len = 2,
	};

	DEBUG_PRINT("%s", __func__);

	if (copy_from_user(val, (const u8 __user *) (uintptr_t) buf
		, write_len)) {
		pr_err("%s buffer copy_from_user fail", __func__);
		status = -EFAULT;
		return status;
	}

	DEBUG_PRINT("%s write_len = %d\n", __func__, write_len);
	DEBUG_PRINT("%s address = %x data = %x\n", __func__, val[0], val[1]);

	spi = fp->spi;
	
	/*Set address*/
	write_addr[1] = val[0];
	/*Set value*/
	write_value[1] = val[1];

	spi_message_init(&m);
	spi_message_add_tail(&t1, &m);
	#if 1//deleted by xhf
	status = spi_sync(spi, &m);
	if (status < 0) {
		pr_err("%s write data error status = %ld\n",
				__func__, status);
		return status;
	}

	spi_message_init(&m);
	#endif
	spi_message_add_tail(&t2, &m);
	status = spi_sync(spi, &m);
	if (status < 0) {
		pr_err("%s write data error status = %ld\n",
				__func__, status);
		return status;
	}

	return status;
}

int fp_read_register(struct fp_data *fp, u8 addr, u8 *buf)
{
	ssize_t status;
	struct spi_device *spi;
	struct spi_message m;

	/*Set address*/
	u8 write_addr[] = {FP_WRITE_ADDRESS, addr};
	u8 read_value[] = {FP_READ_DATA, 0x00};
	u8 result[] = {0xFF, 0xFF};

	struct spi_transfer t1 = {
		.speed_hz = FPC_BTP_SPI_CLOCK_SPEED,
		.tx_buf = write_addr,
		.len = 2,
	};
	struct spi_transfer t2 = {
		.speed_hz = FPC_BTP_SPI_CLOCK_SPEED,
		.tx_buf = read_value,
		.rx_buf = result,
		.len = 2,
	};

	DEBUG_PRINT("%s", __func__);

	spi = fp->spi;

	spi_message_init(&m);
	spi_message_add_tail(&t1, &m);
	#if 1//deleted by xhf
	status = spi_sync(spi, &m);

	if (status == 0) {
		*buf = result[1];
		printk("fp_read_register address = %x result = %x %x\n"
					, addr, result[0], result[1]);
	} else
		pr_err("%s read data error status = %ld\n"
				, __func__, status);
	spi_message_init(&m);
	#endif
	spi_message_add_tail(&t2, &m);
	status = spi_sync(spi, &m);

	if (status == 0) {
		*buf = result[1];
		DEBUG_PRINT("fp_read_register address = %x result = %x %x\n"
					, addr, result[0], result[1]);
	} else
		pr_err("%s read data error status = %ld\n"
				, __func__, status);

	return status;
}

int fp_set_single_register(struct fp_data *fp, u8 addr, u8 val)
{
	ssize_t status = 0;
	struct spi_device *spi;
	struct spi_message m;

	u8 write_addr[] = {FP_WRITE_ADDRESS, 0x00};
	u8 write_value[] = {FP_WRITE_DATA, 0x00};

	struct spi_transfer t1 = {
		.tx_buf = write_addr,
		.len = 2,
	};
	struct spi_transfer t2 = {
		.tx_buf = write_value,
		.len = 2,
	};

	/*Set address*/
	write_addr[1] = addr;
	/*Set value*/
	write_value[1] = val;

	spi = fp->spi;

	spi_message_init(&m);
	spi_message_add_tail(&t1, &m);
	spi_message_add_tail(&t2, &m);
	status = spi_sync(spi, &m);
	if (status < 0) {
		pr_err("%s write data error status = %ld\n",
				__func__, status);
		return status;
	}

	return status;
}

int fp_io_get_one_image(
	struct fp_data *fp,
	u8 *buf,
	u8 *image_buf
	)
{
	uint8_t read_val,
			*tx_buf = (uint8_t *)buf,
			*work_buf = NULL,
			*val = kzalloc(6, GFP_KERNEL);
	int32_t status;
	uint32_t frame_not_ready_count = 0, read_count;

	DEBUG_PRINT("%s\n", __func__);
	if (val == NULL)
	{
		return -ENOMEM;
	}

	if (copy_from_user(val, (const u8 __user *) (uintptr_t) tx_buf, 6)) {
		pr_err("%s buffer copy_from_user fail\n", __func__);
		status = -EFAULT;
		goto end;
	}

	/* total pixel , width * hight */
	read_count = val[0] * val[1];
	printk("fp_io_get_one_image, read_count=%d !\n",read_count);

	while (1) {
		status = fp_read_register
				(fp, FSTATUS_FP_ADDR, &read_val);
		if (status < 0)
		{
			goto end;
		}

		if (read_val & FRAME_READY_MASK)
			break;

		if (frame_not_ready_count >= 250) {
			pr_err("frame_not_ready_count = %d\n",
					frame_not_ready_count);
			break;
		}
		frame_not_ready_count++;
	}

	work_buf = kzalloc(read_count, GFP_KERNEL);
	if (work_buf == NULL) {
		status = -ENOMEM;
		goto end;
	}
	status = fp_mass_read(fp, FDATA_FP_ADDR, work_buf, read_count);
	if (status < 0) {
		pr_err("%s call fp_mass_read error status = %d\n"
				, __func__, status);
		goto end;
	}

	if (copy_to_user((u8 __user *) (uintptr_t) image_buf,
		work_buf, read_count)) {
		pr_err("buffer copy_to_user fail status = %d\n", status);
		status = -EFAULT;
	}
end:
	kfree(val);
	kfree(work_buf);
	return status;
}

/*----------------------- EEPROM ------------------------*/

int fp_eeprom_wren(struct fp_data *fp)
{
	ssize_t status = 0;
	struct spi_device *spi;
	struct spi_message m;

	u8 write_data[] = {FP_EEPROM_WREN_OP};
	struct spi_transfer t = {
		.speed_hz = FPC_BTP_SPI_CLOCK_SPEED,
		.tx_buf = write_data,
		.len = 1,
	};

	DEBUG_PRINT("%s opcode = %x\n", __func__, write_data[0]);

	spi = fp->spi;

	spi_message_init(&m);
	spi_message_add_tail(&t, &m);
	status = spi_sync(spi, &m);
	if (status < 0) {
		pr_err("%s spi_sync error status = %ld\n",
				__func__, status);
		return status;
	}

	return status;
}

int fp_eeprom_wrdi(struct fp_data *fp)
{
	ssize_t status = 0;
	struct spi_device *spi;
	struct spi_message m;

	u8 write_data[] = {FP_EEPROM_WRDI_OP};
	struct spi_transfer t = {
		.speed_hz = FPC_BTP_SPI_CLOCK_SPEED,
		.tx_buf = write_data,
		.len = 1,
	};

	DEBUG_PRINT("%s opcode = %x\n", __func__, write_data[0]);

	spi = fp->spi;

	spi_message_init(&m);
	spi_message_add_tail(&t, &m);
	status = spi_sync(spi, &m);
	if (status < 0) {
		pr_err("%s spi_sync error status = %ld\n",
				__func__, status);
		return status;
	}

	return status;
}

int fp_eeprom_rdsr(struct fp_data *fp, u8 *buf)
{
	ssize_t status;
	struct spi_device *spi;
	struct spi_message m;
	u8 val,
	   read_value[] = {FP_EEPROM_RDSR_OP, 0x00},
	   result[] = {0xFF, 0xFF};

	struct spi_transfer t = {
		.speed_hz = FPC_BTP_SPI_CLOCK_SPEED,
		.tx_buf = read_value,
		.rx_buf = result,
		.len = 2,
	};

	spi = fp->spi;
	spi_message_init(&m);
	spi_message_add_tail(&t, &m);
	status = spi_sync(spi, &m);
	if (status < 0) {
		pr_err("%s spi_sync error status = %ld\n",
				__func__, status);
		return status;
	}

	val = result[1];

	DEBUG_PRINT("%s address = %x buf = %x\n", __func__,
			FP_EEPROM_RDSR_OP, val);

	if (copy_to_user((u8 __user *) (uintptr_t) buf, &val, 1)) {
		pr_err("%s buffer copy_to_user fail status\n", __func__);
		status = -EFAULT;
		return status;
	}

	return status;
}

int fp_eeprom_wrsr(struct fp_data *fp, u8 *buf)
{
	ssize_t status;
	struct spi_device *spi;
	struct spi_message m;
	u8 val;

	u8 write_data[] = {FP_EEPROM_WRSR_OP, 0x00};

	struct spi_transfer t = {
		.speed_hz = FPC_BTP_SPI_CLOCK_SPEED,
		.tx_buf = write_data,
		.len = 2,
	};

	if (copy_from_user(&val, (const u8 __user *) (uintptr_t) buf
		, 1)) {
		pr_err("%s buffer copy_from_user fail\n", __func__);
		status = -EFAULT;
		return status;
	}

	DEBUG_PRINT("%s data = %x\n", __func__, val);

	spi = fp->spi;

	write_data[1] = val;

	spi_message_init(&m);
	spi_message_add_tail(&t, &m);
	status = spi_sync(spi, &m);
	if (status < 0) {
		pr_err("%s spi_sync error status = %ld\n",
				__func__, status);
		return status;
	}

	return status;
}

int fp_eeprom_read(struct fp_data *fp, u8 *addr, u8 *buf, int read_len)
{
	ssize_t status = 0;
	struct spi_device *spi;
	struct spi_message m;
	u8 addrval, *read_value = kzalloc(read_len + 2, GFP_KERNEL);

	struct spi_transfer t = {
		.speed_hz = FPC_BTP_SPI_CLOCK_SPEED,
		.tx_buf = NULL,
		.rx_buf = NULL,
		.len = read_len + 2,
	};

	if (read_value == NULL)
		return -ENOMEM;

	if (copy_from_user(&addrval, (const u8 __user *) (uintptr_t) addr
		, 1)) {
		pr_err("%s buffer copy_from_user fail\n", __func__);
		status = -EFAULT;
		goto exit;
	}

	DEBUG_PRINT("%s read_len = %d\n", __func__, read_len);
	DEBUG_PRINT("%s addrval = %x\n", __func__, addrval);

	spi = fp->spi;

	read_value[0] = FP_EEPROM_READ_OP;
	read_value[1] = addrval;

	t.tx_buf = t.rx_buf = read_value;

	spi_message_init(&m);
	spi_message_add_tail(&t, &m);
	status = spi_sync(spi, &m);
	if (status < 0) {
		pr_err("%s spi_sync error status = %ld\n"
				, __func__, status);
		goto exit;
	}

	if (copy_to_user((u8 __user *) (uintptr_t) buf,
				read_value + 2, read_len)) {
		pr_err("%s buffer copy_to_user fail status\n", __func__);
		status = -EFAULT;
		goto exit;
	}

exit:
	kfree(read_value);

	return status;
}

/*
 * buf - the data wrote to sensor with address info
 * write_len - the length of the data write to memory without address
 */
int fp_eeprom_write(struct fp_data *fp, u8 *buf, int write_len)
{
	ssize_t status = 0;
	struct spi_device *spi;
	struct spi_message m;

	u8 *write_value = kzalloc(write_len + 2, GFP_KERNEL);

	struct spi_transfer t = {
		.speed_hz = FPC_BTP_SPI_CLOCK_SPEED,
		.tx_buf = NULL,
		.len = write_len + 2,
	};

	if (write_value == NULL)
		return -ENOMEM;

	write_value[0] = FP_EEPROM_WRITE_OP;

	if (copy_from_user(write_value + 1, (const u8 __user *) (uintptr_t) buf
		, write_len + 1)) {
		pr_err("%s buffer copy_from_user fail\n", __func__);
		status = -EFAULT;
		goto exit;
	}

	DEBUG_PRINT("%s write_len = %d\n", __func__, write_len);
	DEBUG_PRINT("%s address = %x\n", __func__, write_value[1]);

	spi = fp->spi;

	t.tx_buf = write_value;

	spi_message_init(&m);
	spi_message_add_tail(&t, &m);
	status = spi_sync(spi, &m);
	if (status < 0) {
		pr_err("%s read data error status = %ld\n",
				__func__, status);
		goto exit;
	}

exit:
	kfree(write_value);

	return status;
}

/* ------------------------------ Interrupt -----------------------------*/

/*
 * Interrupt description
 */

#define FP_INT_DETECTION_PERIOD  10
#define FP_DETECTION_THRESHOLD	10

/*
 * FPS interrupt table
 */
struct interrupt_desc fps_ints={9 , 0, "BUT0" , 0};
// struct interrupt_desc fps_ints[] = {
// #if LGE_TEST
// 	{GPIO8 , 0, "BUT0" , 0} /* TINY4412CON15 XEINT12 pin */
// #else
// 	{GPIO8 , 0, "BUT0" , 0} /* TINY4412CON15 XEINT12 pin */
// #endif
// };
//int fps_ints_size = ARRAY_SIZE(fps_ints);

static DECLARE_WAIT_QUEUE_HEAD(interrupt_waitq);

/*
 *	FUNCTION NAME.
 *		interrupt_timer_routine
 *
 *	FUNCTIONAL DESCRIPTION.
 *		basic interrupt timer inital routine
 *
 *	ENTRY PARAMETERS.
 *		gpio - gpio address
 *
 *	EXIT PARAMETERS.
 *		Function Return
 */

void interrupt_timer_routine(
	unsigned long _data
)
{
	struct interrupt_desc *bdata = (struct interrupt_desc *)_data;
	pr_info("FPS interrupt count = %d" , bdata->int_count);
	if (bdata->int_count >= bdata->detect_threshold) {
		bdata->finger_on = 1;
		printk("<xhf>FPS triggered !!!!!!!\n");
	} else
		printk("<xhf>FPS not triggered !!!!!!!\n");

	printk("zq interrupt_timer_routine  int_count = %d   \n", bdata->int_count);
	bdata->int_count = 0;
	wake_up_interruptible(&interrupt_waitq);
}

/*
 *	FUNCTION NAME.
 *		fingerprint_interrupt
 *
 *	FUNCTIONAL DESCRIPTION.
 *		finger print interrupt callback routine
 *
 *	ENTRY PARAMETERS.
 *		irq
 *		dev_id
 *
 *	EXIT PARAMETERS.
 *		Function Return
 */

// static irqreturn_t fingerprint_interrupt(
// 	int irq,
// 	void *dev_id
// )
// {
// 	struct interrupt_desc *bdata = (struct interrupt_desc *)dev_id;
// 	if (!bdata->int_count)
// 		mod_timer(&bdata->timer,
// 			jiffies + msecs_to_jiffies(bdata->detect_period));
// 	bdata->int_count++;
// 	return IRQ_HANDLED;
// }
#if 1
static irqreturn_t fp_eint_func(int irq, void *dev_id)
{
	if (!fps_ints.int_count)
		mod_timer(&fps_ints.timer,jiffies + msecs_to_jiffies(fps_ints.detect_period));
	fps_ints.int_count++;
	
	//mt_eint_unmask(CUST_EINT_FPS_EINT_NUM);
	//enable_irq(fingerpirnt_irq);
	 printk("<xhf>fp_eint_func, fps_ints.int_count=%d",fps_ints.int_count); 
	
	return IRQ_HANDLED;
}

/*
 *	FUNCTION NAME.
 *		Interrupt_Init
 *
 *	FUNCTIONAL DESCRIPTION.
 *		button initial routine
 *
 *	ENTRY PARAMETERS.
 *		gpio - gpio address
 *
 *	EXIT PARAMETERS.
 *		Function Return int
 */
 static int fingerprint_irq_registration(void)
{
	int ret = 0;

	struct device_node *node = NULL;


	//node = of_find_matching_node(node, fp_of_match);
	node = of_find_compatible_node(NULL, NULL, "mediatek,fingerprint");
	printk("node.name %s full name %s\n",node->name,node->full_name);
	
	if (node){
		fingerpirnt_irq = irq_of_parse_and_map(node, 0);
		printk("fingerpirnt_irq number %d\n", fingerpirnt_irq);
		ret = request_irq(fingerpirnt_irq, fp_eint_func, IRQF_TRIGGER_RISING,"fingerprint", NULL);
		if (ret > 0)
			printk("fingerprint request_irq IRQ LINE NOT AVAILABLE!.");
	}else{
		printk("fingerprint request_irq can not find fp eint device node!.");
	}

	return ret;
}
#endif
int Interrupt_Init(int int_num, int detect_period, int detect_threshold)
{
	//int i, irq;

	pr_info("FP %s int_num = %d detect_period = %d detect_threshold = %d\n",
				__func__,
				int_num,
				detect_period,
				detect_threshold);
#if 0
	for (i = 0; i < ARRAY_SIZE(fps_ints); i++) {
		fps_ints[i].int_count = 0;
		fps_ints[i].finger_on = 0;
		if (i == int_num) {
			fps_ints[i].detect_period = detect_period;
			fps_ints[i].detect_threshold = detect_threshold;
		} else {
			fps_ints[i].detect_period = FP_INT_DETECTION_PERIOD;
			fps_ints[i].detect_threshold = FP_DETECTION_THRESHOLD;
		}
		if (!fps_ints[i].gpio)
			continue;

		/*
		 * set the IRQ function and GPIO.
		 * then setting the interrupt trigger type
		 */
		// irq = gpio_to_irq(fps_ints[i].gpio);
		// err = request_irq(
		// 		irq, fingerprint_interrupt,
		// 		IRQ_TYPE_EDGE_RISING,
		// 		fps_ints[i].name, (void *)&fps_ints[i]);
		if (err)
			break;

	}
#else
	if((detect_period>0) && (detect_threshold>0)){
		fps_ints.detect_period = detect_period;
		fps_ints.detect_threshold = detect_threshold;
	}
	else{
		fps_ints.detect_period = detect_period;
		fps_ints.detect_threshold = detect_threshold;
	}
	fps_ints.int_count = 0;
	fps_ints.finger_on = 0;

	fingerprint_irq_registration();
	//enable_irq(fingerpirnt_irq);

	
#endif
	return 0;
}

/*
 *	FUNCTION NAME.
 *		Interrupt_Free
 *
 *	FUNCTIONAL DESCRIPTION.
 *		free all interrupt resource
 *
 *	ENTRY PARAMETERS.
 *		gpio - gpio address
 *
 *	EXIT PARAMETERS.
 *		Function Return int
 */

int Interrupt_Free(void)
{
	//int irq, i;

	// for (i = 0; i < ARRAY_SIZE(fps_ints); i++) {
	// 	if (!fps_ints[i].gpio)
	// 		continue;
	// 	// irq = gpio_to_irq(fps_ints[i].gpio);
	// 	// free_irq(irq, (void *)&fps_ints[i]);
	// 	mt_eint_mask(fps_ints[i].gpio);	//Daita MTK interrupt

	// 	del_timer_sync(&fps_ints[i].timer);
	// }
	//disable_irq(fingerpirnt_irq);
	if (fingerpirnt_irq>0)
	free_irq(fingerpirnt_irq, NULL);
	del_timer_sync(&fps_ints.timer);
	return 0;
}

/*
 *	FUNCTION NAME.
 *		fps_interrupt_re d
 *
 *	FUNCTIONAL DESCRIPTION.
 *		FPS interrupt read status
 *
 *	ENTRY PARAMETERS.
 *		wait poll table structure
 *
 *	EXIT PARAMETERS.
 *		Function Return int
 */

unsigned int fps_interrupt_poll(
	struct file *file,
	struct poll_table_struct *wait
)
{
	unsigned int mask = 0;
	//int i = 0;
	//pr_info("%s\n", __func__);

	// for (i = 0; i < fps_ints_size; i++)
	// 	fps_ints[i].int_count = 0;
	// poll_wait(file, &interrupt_waitq, wait);
	// for (i = 0; i < fps_ints_size; i++) {
	// 	if (fps_ints[i].finger_on) {
	// 		mask |= POLLIN | POLLRDNORM;
	// 		fps_ints[i].finger_on = 0;
	// 	}
	// }
	fps_ints.int_count = 0;
	poll_wait(file, &interrupt_waitq, wait);
	//DEBUG_PRINT("%s finger_on = %d\n", __func__, fp_ints.finger_on);
	
	if (fps_ints.finger_on) {
			mask |= POLLIN | POLLRDNORM;
			fps_ints.finger_on = 0;
	}

	return mask;
}

void fps_interrupt_abort(void)
{
	//int i = 0;
//	for (i = 0; i < fps_ints_size; i++)
		fps_ints.finger_on = 0;
	wake_up_interruptible(&interrupt_waitq);
}

/*-------------------------------------------------------------------------*/

static inline void fp_reset(void)
{
	fingerprint_reset_gpio_output(0);
	msleep(30);
	fingerprint_reset_gpio_output(1);
	msleep(20);
}

static inline void fp_reset_set(int enable)
{
	DEBUG_PRINT("%s(%s)\n", __func__, enable ? "true" : "false");
	if (enable == 0) {
		fingerprint_reset_gpio_output(0);
		msleep(30);
	} else {
		fingerprint_reset_gpio_output(1);
		msleep(20);
	}
}

static ssize_t fp_read(struct file *filp,
						char __user *buf,
						size_t count,
						loff_t *f_pos)
{
	/*Implement by vendor if needed*/
	return 0;
}

static ssize_t fp_write(struct file *filp,
						const char __user *buf,
						size_t count,
						loff_t *f_pos)
{
	/*Implement by vendor if needed*/
	return 0;
}

static long fp_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int err = 0;
	int retval = 0;
	struct fp_data *fp;
	struct spi_device *spi;
	u32 tmp;
	struct egis_ioc_transfer *ioc = NULL;

	/* Check type and command number */
	if (_IOC_TYPE(cmd) != EGIS_IOC_MAGIC) {
		pr_err("%s _IOC_TYPE(cmd) != EGIS_IOC_MAGIC", __func__);
		return -ENOTTY;
	}

	/*
	 * Check access direction once here; don't repeat below.
	 * IOC_DIR is from the user perspective, while access_ok is
	 * from the kernel perspective; so they look reversed.
	 */
	if (_IOC_DIR(cmd) & _IOC_READ)
		err = !access_ok(VERIFY_WRITE,
						(void __user *)arg,
						_IOC_SIZE(cmd));
	if (err == 0 && _IOC_DIR(cmd) & _IOC_WRITE)
		err = !access_ok(VERIFY_READ,
						(void __user *)arg,
						_IOC_SIZE(cmd));
	if (err) {
		pr_err("%s err", __func__);
		return -EFAULT;
	}

	/*
	 * guard against device removal before, or while,
	 * we issue this ioctl.
	 */
	fp = filp->private_data;
	spin_lock_irq(&fp->spi_lock);
	spi = spi_dev_get(fp->spi);
	spin_unlock_irq(&fp->spi_lock);

	if (spi == NULL) {
		pr_err("%s spi == NULL", __func__);
		return -ESHUTDOWN;
	}

	mutex_lock(&fp->buf_lock);

	/* segmented and/or full-duplex I/O request */
	if (_IOC_NR(cmd) != _IOC_NR(EGIS_IOC_MESSAGE(0))
					|| _IOC_DIR(cmd) != _IOC_WRITE) {
		retval = -ENOTTY;
		goto out;
	}

	tmp = _IOC_SIZE(cmd);
	if ((tmp == 0) || (tmp % sizeof(struct egis_ioc_transfer)) != 0) {
		retval = -EINVAL;
		goto out;
	}

	/* copy into scratch area */
	ioc = kmalloc(tmp, GFP_KERNEL);
	if (ioc == NULL) {
		retval = -ENOMEM;
		goto out;
	}
	if (__copy_from_user(ioc, (void __user *)arg, tmp)) {
		retval = -EFAULT;
		goto out;
	}

	DEBUG_PRINT("<xhf>%s ioc->opcode = %d\n", __func__, ioc->opcode);
	/*
        * FACTORY Read register or Write register data
        * tx_buf include register address will be read
        * added by xiehaifei for factory test
        */
       if (ioc->opcode == FP_REGISTER_FACTORY_TEST_READ) {
               u8 *address = ioc->tx_buf;
               u8 *result = ioc->rx_buf;
               //DEBUG_PRINT("fp FP_REGISTER_READ\n");

               //added by xhf
               spi->controller_data = (void *) &spi_conf_fifo;          // mtk configuration
               spi_setup(spi);
               //end

               retval = fp_io_read_register(fp, address, result);
               if (retval < 0) {
               DEBUG_PRINT("%s FP_REGISTER_READ error retval = %d\n"
               , __func__, retval);
               goto out;
               }

               printk("FP_REGISTER_FACTORY_TEST_READ, result[0]=%x/n",result[0]);

               if (result[0]==FP_REGISTER_TEST_READ_DATA)
               {
                       printk("ET310 SPI TEST SUCCESS!\n");
                       retval=1;
               }
               else printk("ET310 SPI TEST FAIL!\n");
       }


	/*
	 * Read register
	 * tx_buf include register address will be read
	 */
	if (ioc->opcode == FP_REGISTER_READ) {
		u8 *address = ioc->tx_buf;
		u8 *result = ioc->rx_buf;
		DEBUG_PRINT("fp FP_REGISTER_READ\n");

		//added by xhf
        spi->controller_data = (void *) &spi_conf_fifo;          // mtk configuration
        spi_setup(spi);
		//end

		retval = fp_io_read_register(fp, address, result);
		
		
		if (retval < 0)	{
			pr_err("%s FP_REGISTER_READ error retval = %d\n"
			, __func__, retval);
			goto out;
		}
	}

	/*
	 * Write data to register
	 * tx_buf includes address and value will be wrote
	 */
	if (ioc->opcode == FP_REGISTER_WRITE) {
		u8 *buf = ioc->tx_buf;
		DEBUG_PRINT("fp FP_REGISTER_WRITE");

		//added by xhf
        spi->controller_data = (void *) &spi_conf_fifo;          // mtk configuration
        spi_setup(spi);
		//end

		retval = fp_io_write_register(fp, buf);

		if (retval < 0) {
			pr_err("%s FP_REGISTER_WRITE error retval = %d\n"
			, __func__, retval);
			goto out;
		}
	}

	/*
	 * Get one frame data from sensor
	 */
	if (ioc->opcode == FP_GET_ONE_IMG) {
		u8 *buf = ioc->tx_buf;
		u8 *image_buf = ioc->rx_buf;
		DEBUG_PRINT("fp FP_GET_ONE_IMG\n");

		spi->controller_data = (void *) &spi_conf_dma;          // mtk configuration
		spi_setup(spi);

		retval = fp_io_get_one_image(fp, buf, image_buf);

		if (retval < 0) {
			pr_err("%s FP_GET_ONE_IMG error retval = %d\n"
			, __func__, retval);
			goto out;
		}
	}

	if (ioc->opcode == FP_SENSOR_RESET)
		fp_reset();

	if (ioc->opcode == FP_RESET_SET) {
		pr_info("%s FP_SENSOR_RESET\n", __func__);
		pr_info("%s status = %d\n", __func__, ioc->len);
		fp_reset_set(ioc->len);
	}

	if (ioc->opcode == FP_SET_SPI_CLOCK) {
		__u32 current_speed = spi->max_speed_hz;
		pr_info("%s FP_SET_SPI_CLOCK\n", __func__);
		pr_info("%s speed_hz = %d\n", __func__, ioc->speed_hz);
		pr_info("%s current_speed = %d\n", __func__, current_speed);

		spi->max_speed_hz = ioc->speed_hz;
		retval = spi_setup(spi);
		if (retval < 0) {
			pr_err("%s spi_setup error %d\n", __func__, retval);
			spi->max_speed_hz = current_speed;
		}
		pr_info("%s spi speed_hz = %d\n", __func__, spi->max_speed_hz);
	}

	if (ioc->opcode == FP_EEPROM_WREN) {
		pr_info("%s FP_EEPROM_WREN\n", __func__);
		fp_reset_set(0);
		fp_eeprom_wren(fp);
		fp_reset_set(1);
	}

	if (ioc->opcode == FP_EEPROM_WRDI) {
		pr_info("%s FP_EEPROM_WRDI\n", __func__);
		fp_reset_set(0);
		fp_eeprom_wrdi(fp);
		fp_reset_set(1);
	}

	if (ioc->opcode == FP_EEPROM_RDSR) {
		u8 *result = ioc->rx_buf;
		pr_info("%s FP_EEPROM_RDSR\n", __func__);
		fp_reset_set(0);
		fp_eeprom_rdsr(fp, result);
		fp_reset_set(1);
	}

	if (ioc->opcode == FP_EEPROM_WRSR) {
		u8 *buf = ioc->tx_buf;
		pr_info("%s FP_EEPROM_WRSR\n", __func__);
		fp_reset_set(0);
		fp_eeprom_wrsr(fp, buf);
		fp_reset_set(1);
	}

	if (ioc->opcode == FP_EEPROM_READ) {
		u8 *buf = ioc->tx_buf;
		u8 *result = ioc->rx_buf;
		pr_info("%s FP_EEPROM_READ\n", __func__);
		fp_reset_set(0);
		fp_eeprom_read(fp, buf, result, ioc->len);
		fp_reset_set(1);
	}

	if (ioc->opcode == FP_EEPROM_WRITE) {
		u8 *buf = ioc->tx_buf;
		pr_info("%s FP_EEPROM_WRITE\n", __func__);
		fp_reset_set(0);
		fp_eeprom_write(fp, buf, ioc->len);
		fp_reset_set(1);
	}

	if (ioc->opcode == FP_POWER_ONOFF)
		pr_info("power control status = %d\n", ioc->len);

	/*
	 * Trigger inital routine
	 */
	if (ioc->opcode == INT_TRIGGER_INIT) {
		pr_info(">>> fp Trigger function init\n");
		retval = Interrupt_Init(
				(int)ioc->pad[0],
				(int)ioc->pad[1],
				(int)ioc->pad[2]);
		pr_info("trigger init = %x\n", retval);
	}

	/*
	 * trigger
	 */
	if (ioc->opcode == INT_TRIGGER_CLOSE) {
		pr_info("<<< fp Trigger function close\n");
		retval = Interrupt_Free();
		pr_info("trigger close = %x\n", retval);
	}

	/*
	 * read interrupt status
	 */
	if (ioc->opcode == INT_TRIGGER_ABORT)
	{
		pr_info("<<< fp Trigger function close\n");
		fps_interrupt_abort();
	}
	
out:
	if (ioc != NULL)
		kfree(ioc);

	mutex_unlock(&fp->buf_lock);
	spi_dev_put(spi);
	if (retval < 0)
		pr_err("%s retval = %d", __func__, retval);
	return retval;
}

#ifdef CONFIG_COMPAT
static long fp_compat_ioctl(struct file *filp,
	unsigned int cmd,
	unsigned long arg)
{
	return fp_ioctl(filp, cmd, (unsigned long)compat_ptr(arg));
}
#else
#define fp_compat_ioctl NULL
#endif /* CONFIG_COMPAT */

static int fp_open(struct inode *inode, struct file *filp)
{
	struct fp_data *fp;
	int			status = -ENXIO;

	DEBUG_PRINT("%s\n", __func__);
	mutex_lock(&device_list_lock);

	list_for_each_entry(fp, &device_list, device_entry)
	{
		if (fp->devt == inode->i_rdev) {
			status = 0;
			break;
		}
	}
	if (status == 0) {
		if (fp->buffer == NULL) {
			fp->buffer = kmalloc(bufsiz, GFP_KERNEL);
			if (fp->buffer == NULL) {
				dev_dbg(&fp->spi->dev, "open/ENOMEM\n");
				status = -ENOMEM;
			}
		}
		if (status == 0) {
			fp->users++;
			filp->private_data = fp;
			nonseekable_open(inode, filp);
		}
	} else
		pr_debug("fp: nothing for minor %d\n", iminor(inode));

	mutex_unlock(&device_list_lock);
	return status;
}

static int fp_release(struct inode *inode, struct file *filp)
{
	struct fp_data *fp;

	DEBUG_PRINT("%s\n", __func__);
	mutex_lock(&device_list_lock);
	fp = filp->private_data;
	filp->private_data = NULL;

	/* last close? */
	fp->users--;
	if (fp->users == 0) {
		int	dofree;

		kfree(fp->buffer);
		fp->buffer = NULL;

		/* ... after we unbound from the underlying device? */
		spin_lock_irq(&fp->spi_lock);
		dofree = (fp->spi == NULL);
		spin_unlock_irq(&fp->spi_lock);

		if (dofree)
			kfree(fp);
	}
	mutex_unlock(&device_list_lock);

	return 0;
}

static const struct file_operations fp_fops = {
	.owner = THIS_MODULE,
	.write = fp_write,
	.read = fp_read,
	.unlocked_ioctl = fp_ioctl,
	.compat_ioctl = fp_compat_ioctl,
	.open = fp_open,
	.release = fp_release,
	.llseek = no_llseek,
	.poll = fps_interrupt_poll
};

/*-------------------------------------------------------------------------*/

static struct class *fp_class;

/*-------------------------------------------------------------------------*/

static int __init fp_probe(struct spi_device *spi)
{
	struct fp_data *fp;
	int status;
	unsigned long minor;
	//int i;
	struct spi_device *spi_initialize;

	DEBUG_PRINT("%s initial\n", __func__);

	/* Allocate driver data */
	fp = kzalloc(sizeof(*fp), GFP_KERNEL);
	if (fp == NULL)
		return -ENOMEM;

	/* Initialize the driver data */
	fp->spi = spi;

	//hwPowerOn(MT6331_POWER_LDO_VMCH, VOL_3300,"ET310_3V3");	//mtk 3.3V power-on
	fingerprint_power_en_gpio_output(1);
	spi->controller_data = (void *) &spi_conf;          // mtk configuration
    spi->max_speed_hz = FPC_BTP_SPI_CLOCK_SPEED;
    spi_setup(spi);
	spin_lock_init(&fp->spi_lock);
	mutex_init(&fp->buf_lock);

	INIT_LIST_HEAD(&fp->device_entry);

	/*
	 * If we can allocate a minor number, hook up this device.
	 * Reusing minors is fine so long as udev or mdev is working.
	 */
	mutex_lock(&device_list_lock);
	minor = find_first_zero_bit(minors, N_SPI_MINORS);
	if (minor < N_SPI_MINORS) {
		struct device *dev;
		fp->devt = MKDEV(FP_MAJOR, minor);
		DEBUG_PRINT("%s: create dev %d,%d\n", __func__, FP_MAJOR, (int)minor);
		dev = device_create(fp_class, &spi->dev, fp->devt,
					fp, "esfp0");
		status = IS_ERR(dev) ? PTR_ERR(dev) : 0;
	} else{
		dev_dbg(&spi->dev, "no minor number available!\n");
		status = -ENODEV;
	}
	if (status == 0) {
		set_bit(minor, minors);
		list_add(&fp->device_entry, &device_list);
	}
	mutex_unlock(&device_list_lock);
	

	spin_lock_irq(&fp->spi_lock);
	spi_initialize= spi_dev_get(fp->spi);
	spin_unlock_irq(&fp->spi_lock);	

	spi_initialize->mode = SPI_MODE_3;
	spi_initialize->bits_per_word = 8;

	spi_setup(spi_initialize);

	fp_reset();
	
	if (status == 0)
		spi_set_drvdata(spi, fp);
	else
		kfree(fp);


	setup_timer(&fps_ints.timer, interrupt_timer_routine, (unsigned long)&fps_ints);
	add_timer(&fps_ints.timer);
	// for (i = 0; i < fps_ints_size; i++) {
	// 	setup_timer(&fps_ints[0].timer, interrupt_timer_routine,
	// 				(unsigned long)fps_ints);
	// }
	//Interrupt_Init(0, 20, 10);	
	
	DEBUG_PRINT("%s : initialize success %d\n", __func__, status);

	return status;
}

static int fp_remove(struct spi_device *spi)
{
	struct fp_data *fp = spi_get_drvdata(spi);
	DEBUG_PRINT("%s\n", __func__);

	//hwPowerDown(MT6331_POWER_LDO_VMCH,"ET310_3V3");	// mtk 3.3v power-off

	/* make sure ops on existing fds can abort cleanly */
	spin_lock_irq(&fp->spi_lock);
	fp->spi = NULL;
	spi_set_drvdata(spi, NULL);
	spin_unlock_irq(&fp->spi_lock);

	/* prevent new opens */
	mutex_lock(&device_list_lock);
	list_del(&fp->device_entry);
	device_destroy(fp_class, fp->devt);
	clear_bit(MINOR(fp->devt), minors);
	if (fp->users == 0)
		kfree(fp);
	mutex_unlock(&device_list_lock);

	return 0;
}

static struct spi_device_id spi_id_table = {"et310", 0};
static struct spi_driver fp_spi_driver = {

	.driver = {
		.name = "et310",
		.bus	= &spi_bus_type,
		.owner = THIS_MODULE,
	},
	.probe = fp_probe,
	.remove = fp_remove,
	.id_table = &spi_id_table,

};
/////////////////////Daita ET310 MTK//////////////////////////////////
static struct spi_board_info spi_board_devs[] __initdata = {
	[0] = {
			.modalias="et310",
			.bus_num = 0,
			.chip_select=0,
			.mode = SPI_MODE_3,
		},
};

/*-------------------------------------------------------------------------*/

static int __init fp_init(void)
{
	int status;
	DEBUG_PRINT("%s\n", __func__);

	/*
	 * Claim our 256 reserved device numbers.  Then register a class
	 * that will key udev/mdev to add/remove /dev nodes.  Last, register
	 * the driver which manages those device numbers.
	 */

	BUILD_BUG_ON(N_SPI_MINORS > 256);
	status = register_chrdev(FP_MAJOR, "et310", &fp_fops);
	if (status < 0) {
		DEBUG_PRINT("%s : cannot register chrdev : %d\n", __func__, status);
		return status;
	}
	
	fp_class = class_create(THIS_MODULE, "fp");
	if (IS_ERR(fp_class)) {
		DEBUG_PRINT("%s : cannot create class\n", __func__);
		unregister_chrdev(FP_MAJOR, fp_spi_driver.driver.name);
		return PTR_ERR(fp_class);
	}

	spi_register_board_info(spi_board_devs, ARRAY_SIZE(spi_board_devs));
	status = spi_register_driver(&fp_spi_driver);
	if (status < 0) {
		DEBUG_PRINT("%s : cannot spi register : %d\n", __func__, status);
		class_destroy(fp_class);
		unregister_chrdev(FP_MAJOR, fp_spi_driver.driver.name);
	}
	
	DEBUG_PRINT("%s status=%d\n", __func__, status);
	return status;
}
module_init(fp_init);

static void __exit fp_exit(void)
{
	DEBUG_PRINT("%s\n", __func__);
	spi_unregister_driver(&fp_spi_driver);
	class_destroy(fp_class);
	unregister_chrdev(FP_MAJOR, fp_spi_driver.driver.name);
}
module_exit(fp_exit);

MODULE_AUTHOR("Wang YuWei, <robert.wang@egistec.com>");
MODULE_DESCRIPTION("SPI Interface for ET310");
MODULE_LICENSE("GPL");
MODULE_ALIAS("spi:spidev");
