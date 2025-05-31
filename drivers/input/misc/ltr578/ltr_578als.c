/*
 * File:ltr_578als.c
 * Author:Shi Zhigang <Zhigang.Shi@liteon.com>
 * Created:2018-08-17
 * Description:LTR-578ALS Driver
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
 */

//#define USE_WAKELOCK_PS
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/gpio.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/miscdevice.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/sysctl.h>
#include <linux/uaccess.h>
#ifdef USE_WAKELOCK_PS
#include <linux/wakelock.h>
#endif
#include <linux/kthread.h>
#include "ltr_578als.h"

#define LTR578_DBG
#ifdef LTR578_DBG
#define ENTER printk(KERN_INFO "[LTR578_DBG] func: %s  line: %04d  ", __func__, __LINE__)
#define PRINT_DBG(x...)  printk(KERN_INFO "[LTR578_DBG] " x)
#define PRINT_INFO(x...)  printk(KERN_INFO "[LTR578_INFO] " x)
#define PRINT_WARN(x...)  printk(KERN_INFO "[LTR578_WARN] " x)
#define PRINT_ERR(format, x...)  printk(KERN_ERR "[LTR578_ERR] func: %s  line: %04d  info: " format, __func__, __LINE__, ## x)
#else
#define ENTER
#define PRINT_DBG(x...)
#define PRINT_INFO(x...)  printk(KERN_INFO "[LTR578_INFO] " x)
#define PRINT_WARN(x...)  printk(KERN_INFO "[LTR578_WARN] " x)
#define PRINT_ERR(format, x...)  printk(KERN_ERR "[LTR578_ERR] func: %s  line: %04d  info: " format, __func__, __LINE__, ## x)
#endif

typedef struct tag_ltr578 {
	struct input_dev *input;
	struct i2c_client *client;
	struct work_struct work;
	struct workqueue_struct *ltr_work_queue;
} ltr578_t, *ltr578_p;

static int ps_threshold_high = 600;
static int ps_threshold_low = 500;
/* prox data max range */
static int dyna_cali = 2047;

static int p_flag = -1;
static int l_flag = -1;
//static u8 l_gainrange = ALS_RANGE_3;
static struct i2c_client *this_client = NULL;

static int ltr578_reg_init(void);

#ifdef USE_INTERRUPT
#ifdef USE_WAKELOCK_PS
static struct wake_lock psensor_timeout_wakelock;
#define WAKELOCK_TIMEOUT_INT_MS 50
#define WAKELOCK_TIMEOUT_WORK_MS 500
#endif
#endif

ltr578_t *g_ltr578_data = NULL;

#ifdef USE_POLLING
static int als_poll_flag = 0;
static int als_polling_function(void* arg);
static struct completion als_thread_completion;
static struct task_struct *als_polling_tsk = NULL;
static void als_ps_report_init(void);
#endif

static int ltr578_i2c_read_bytes(u8 index, u8 *rx_buff, u8 length)
{
	int ret = -1;
	struct i2c_msg msgs[] = {
		{
			.addr = this_client->addr,/* chip address, 7bit */
			.flags = 0,/* write */
			.len = 1,
			.buf = &index,
		},
		{
			.addr = this_client->addr,
			.flags = I2C_M_RD,
			.len = length,
			.buf = rx_buff,
		},
	};

	ret = i2c_transfer(this_client->adapter, msgs, 2);
	if (ret != 2)
		PRINT_ERR("READ ERROR!ret=%d\n", ret);
	return ret;
}

static int ltr578_i2c_write_bytes(u8 *tx_buff, u8 length)
{
	int ret = -1;
	struct i2c_msg msgs[1];

	msgs[0].addr = this_client->addr;
	msgs[0].flags = 0;
	msgs[0].len = length;
	msgs[0].buf = tx_buff;

	ret = i2c_transfer(this_client->adapter, msgs, 1);
	if (ret != 1)
		PRINT_ERR("WRITE ERROR!ret=%d\n", ret);
	return ret;
}

static int ltr578_i2c_read_2_bytes(u8 reg)
{
	int ret = 0;
	u8 data[2] = {0};

	ret = ltr578_i2c_read_bytes(reg, data, 2);
	if (ret != 2) {
		PRINT_ERR("READ ERROR!ret=%d\n", ret);
		return -1;
	}

	ret = data[1];
	ret = (ret<<8) | data[0];

	return ret;
}

static int ltr578_i2c_read_1_byte(u8 reg)
{
	int ret = 0;
	u8 data[1] = {0};

	ret = ltr578_i2c_read_bytes(reg, data, 1);
	if (ret != 2) {
		PRINT_ERR("READ ERROR!ret=%d\n", ret);
		return -1;
	}

	ret = data[0];
	return ret;
}

#ifdef USE_INTERRUPT
static int ltr578_i2c_write_2_bytes(u8 reg, u16 value)
{
	int ret = 0;
	u8 data[3] = {0};

	data[0] = reg;
	data[1] = value & 0x00FF;
	data[2] = value >> 8;

	ret = ltr578_i2c_write_bytes(data, 3);
	if (ret != 1) {
		PRINT_ERR("WRITE ERROR!ret=%d\n", ret);
		return -1;
	}
	return 1;
}
#endif

static int ltr578_i2c_write_1_byte(u8 reg, u8 value)
{
	int ret = 0;
	u8 data[2] = {0};

	data[0] = reg;
	data[1] = value;

	ret = ltr578_i2c_write_bytes(data, 2);
	if (ret != 1) {
		PRINT_ERR("WRITE ERROR!ret=%d\n", ret);
		return -1;
	}
	return 1;
}

static void dynamic_calibrate(void)
{
	int i = 0;
	int val = 0;
	int data_total = 0;
	int noise = 0;

	for (i = 0; i < 3; i++) {
		msleep(20);
		val = ltr578_i2c_read_2_bytes(LTR578_PS_DATA_0);
		data_total += val;
	}
	noise = data_total/3;

	if(noise < (dyna_cali + 500))
	{
		dyna_cali = noise;

		if(noise < 1500)
		{
			ps_threshold_high = noise + 250;
			ps_threshold_low = noise + 220;
		}else{
			ps_threshold_high = 1800;
			ps_threshold_low = 1600;
		}
	}
    
	PRINT_INFO("ps_threshold_high=%d, ps_threshold_low=%d\n", ps_threshold_high, ps_threshold_low);
#ifdef USE_INTERRUPT
	ltr578_i2c_write_1_byte(LTR578_PS_THRES_UP_0, ps_threshold_high & 0xff);
	ltr578_i2c_write_1_byte(LTR578_PS_THRES_UP_1, (ps_threshold_high>>8) & 0x07);
	ltr578_i2c_write_1_byte(LTR578_PS_THRES_LOW_0, ps_threshold_low & 0xff);
	ltr578_i2c_write_1_byte(LTR578_PS_THRES_LOW_1, (ps_threshold_low>>8) & 0x07);
#endif
}

static int ltr578_ps_enable(void)
{
	int ret = -1;
	u8 regdata;
	ltr578_t *ltr_578als = (ltr578_t *)i2c_get_clientdata(this_client);
	printk("zxs %s\n", __func__);

	if (p_flag == 1)
	{
		PRINT_INFO("PS: Already enabled \n");
		return 0;
	}

	regdata = ltr578_i2c_read_1_byte(LTR578_MAIN_CTRL);
	regdata |= 0x01;	
	
	ret = ltr578_i2c_write_1_byte(LTR578_MAIN_CTRL, regdata);
	if (ret < 0)
	{
		PRINT_INFO("PS: Enable failed \n");
		return ret;
	}
	mdelay(WAKEUP_DELAY);
	
	dynamic_calibrate();
	
	/*input report init*/
	input_report_abs(ltr_578als->input, ABS_DISTANCE, 1);
	input_sync(ltr_578als->input);

#ifdef USE_POLLING
	als_ps_report_init();
#endif
	
	if (ret >= 0)
		return 0;
	else
		return ret;
}

static int ltr578_ps_disable(void)
{
	int ret = -1;
	u8 regdata;
	ltr578_t *ltr_578als = (ltr578_t *)i2c_get_clientdata(this_client);
	printk("zxs %s\n", __func__);

	if (p_flag == 0)
	{
		PRINT_INFO("PS: Already disabled \n");
		return 0;
	}

	regdata = ltr578_i2c_read_1_byte(LTR578_MAIN_CTRL);
	regdata &= 0xFE;

	ret = ltr578_i2c_write_1_byte(LTR578_MAIN_CTRL, regdata);
	if (ret < 0)
	{
		PRINT_INFO("PS: Disable failed \n");
		return ret;
	}

	/*input report init*/
	input_report_abs(ltr_578als->input, ABS_DISTANCE, 1);
	input_sync(ltr_578als->input);

#ifdef USE_POLLING
	if (l_flag == 0)
		als_poll_flag = 0;
#endif
	
	if (ret >= 0)
		return 0;
	else
		return ret;
}

static int ltr578_als_enable(void)
{
	int ret = -1;
	u8 regdata;
	printk("zxs %s\n", __func__);

	if (l_flag == 1)
	{
		PRINT_INFO("ALS: Already enabled \n");
		return 0;
	}

	regdata = ltr578_i2c_read_1_byte(LTR578_MAIN_CTRL);
	regdata |= 0x02;

	ret = ltr578_i2c_write_1_byte(LTR578_MAIN_CTRL, regdata);
	if (ret < 0)
	{
		PRINT_INFO("ALS: Enable failed \n");
		return ret;
	}
	
	mdelay(WAKEUP_DELAY);

#ifdef USE_POLLING
	als_ps_report_init();
#endif
	
	if (ret >= 0)
		return 0;
	else
		return ret;
}

// Put ALS into Standby mode
static int ltr578_als_disable(void)
{
    int ret = -1;
	u8 regdata;
	printk("zxs %s\n", __func__);

	if (l_flag == 0)
	{
		PRINT_INFO("ALS: Already disabled \n");
		return 0;
	}

	regdata = ltr578_i2c_read_1_byte(LTR578_MAIN_CTRL);
	regdata &= 0xFD;

	ret = ltr578_i2c_write_1_byte(LTR578_MAIN_CTRL, regdata);
	if (ret < 0)
	{
		PRINT_INFO("ALS: Disable failed \n");
		return ret;
	}

#ifdef USE_POLLING
	if (p_flag == 0)
		als_poll_flag = 0;
#endif

    if(ret >= 0)
        return 0;
    else
        return ret;
}

static int ltr578_als_read(void)
{
	int alsval_0, alsval_1, alsval_2, alsval;
	int clearval_0, clearval_1, clearval_2, clearval;
	int luxdata_int;
	printk("zxs %s\n", __func__);

	alsval_0 = ltr578_i2c_read_1_byte(LTR578_ALS_DATA_0);
	alsval_1 = ltr578_i2c_read_1_byte(LTR578_ALS_DATA_1);
	alsval_2 = ltr578_i2c_read_1_byte(LTR578_ALS_DATA_2);
	alsval = (alsval_2 * 256 * 256) + (alsval_1 * 256) + alsval_0;
	PRINT_INFO("alsval_0 = %d,alsval_1=%d,alsval_2=%d,alsval=%d\n", alsval_0, alsval_1, alsval_2, alsval);

	clearval_0 = ltr578_i2c_read_1_byte(LTR578_CLEAR_DATA_0);
	clearval_1 = ltr578_i2c_read_1_byte(LTR578_CLEAR_DATA_1);
	clearval_2 = ltr578_i2c_read_1_byte(LTR578_CLEAR_DATA_2);
	clearval = (clearval_2 * 256 * 256) + (clearval_1 * 256) + clearval_0;
	PRINT_INFO("clearval_0 = %d,clearval_1=%d,clearval_2=%d,clearval=%d\n", clearval_0, clearval_1, clearval_2, clearval);

	if (alsval == 0)
	{
		luxdata_int = 0;
		goto out;
	}

	if (ALS_USE_CLEAR_DATA == 1)
	{
		// ALS_Lux = ALS_DATA * 0.8 * WINFAC1 * (1 ? WINFAC2 * CLEAR_DATA / ALS_DATA /1000) / ALS_GAIN / ALS_INT
#if 0
		luxdata_int = alsval * 8 * ALS_WIN_FACTOR * (1 - ALS_WIN_FACTOR2 * clearval / alsval / 1000) / l_gainrange / 10;
#else	
		luxdata_int = alsval * 8 * ALS_WIN_FACTOR * (1 - ALS_WIN_FACTOR2 * clearval / alsval / 1000) / 10;
#endif
	}
	else
	{
#if 0
		luxdata_int = alsval * 8 * ALS_WIN_FACTOR / l_gainrange / 10;//formula: ALS counts * 0.8/gain/int , int=1	
#else	
		luxdata_int = alsval * 8 * ALS_WIN_FACTOR / 10;
#endif
	}
	PRINT_INFO("ltr578_als_read: als_value_lux = %d\n", luxdata_int);
out:	
	return luxdata_int;
}

static int ltr578_open(struct inode *inode, struct file *file)
{
	PRINT_INFO("ltr578_open\n");
	return 0;
}

static int ltr578_release(struct inode *inode, struct file *file)
{
	PRINT_INFO("ltr578_release\n");
	return 0;
}

static long ltr578_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	int flag;
	
    PRINT_INFO("cmd = %d, %d\n", _IOC_NR(cmd), cmd);
	printk("zxs %s cmd = %d\n", __func__, _IOC_NR(cmd));
	switch (cmd) {
	case LTR_IOCTL_SET_PFLAG: {
		printk("zxs LTR_IOCTL_SET_PFLAG\n");
		  if (copy_from_user(&flag, argp, sizeof(flag)))
			  return -EFAULT;
		  PRINT_INFO("SET_PFLAG = %d\n", flag);
		  if (flag == 1) {
			  //ltr578_reg_init();
			  //msleep(20);
			  if (ltr578_ps_enable())
				  return -EIO;
		  } else if (flag == 0) {
			  if (ltr578_ps_disable())
				  return -EIO;
		  } else
			  return -EINVAL;
		  p_flag = flag;
	  }
	  break;
	case LTR_IOCTL_SET_LFLAG: {
		printk("zxs LTR_IOCTL_SET_LFLAG\n");
		  if (copy_from_user(&flag, argp, sizeof(flag)))
			  return -EFAULT;
		  PRINT_INFO("SET_LFLAG = %d\n", flag);
		  if (flag == 1) {
			  //ltr578_reg_init();
			  //msleep(20);
			  if (ltr578_als_enable())
				  return -EIO;
		  } else if (flag == 0) {
			  if (ltr578_als_disable())
				  return -EIO;
		  } else
			  return -EINVAL;
		  l_flag = flag;
	  }
	  break;	
	case LTR_IOCTL_GET_PFLAG: {
		printk("zxs LTR_IOCTL_GET_PFLAG\n");
		  flag = p_flag;
		  PRINT_INFO("PFLAG = %d\n", flag);
		  if (copy_to_user(argp, &flag, sizeof(flag)))
			  return -EFAULT;
	  }
	  break;
	case LTR_IOCTL_GET_LFLAG: {
		printk("zxs LTR_IOCTL_GET_LFLAG\n");
		  flag = l_flag;
		  PRINT_INFO("LFLAG=%d\n", flag);
		  if (copy_to_user(argp, &flag, sizeof(flag)))
			  return -EFAULT;
	  }
	  break;
	case LTR_IOCTL_GET_CHIPINFO:{
		if (copy_to_user(argp, "LTR578", sizeof("LTR578")))
		{
			printk(KERN_ERR "%s: get delay copy error!\n", __func__);
			return -EFAULT;
		}
	  }
	  break;
	default:
	  PRINT_ERR("unknown cmd:0x%08X(%d)\n", cmd, cmd);
	  break;
	}

	return 0;
}

static struct file_operations ltr578_fops = {
	 .owner = THIS_MODULE,
	 .open = ltr578_open,
	 .release = ltr578_release,
	 .unlocked_ioctl = ltr578_ioctl,
};

static struct miscdevice ltr578_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = LTR578_I2C_NAME,
	.fops = &ltr578_fops,
};

static void ltr578_process_data(ltr578_t *pls)
{
	int status = 0;
	int value = 0;

	status = ltr578_i2c_read_1_byte(LTR578_MAIN_STATUS);
	PRINT_DBG("LTR578_MAIN_STATUS = 0x%02X, p_flag = %d, l_flag = %d\n", status, p_flag, l_flag);

	if (p_flag)
	{
		if ((0x01 == (status & 0x01))) {/*is 578 PS*/
			value = ltr578_i2c_read_2_bytes(LTR578_PS_DATA_0);
			/* modified by hongguang@wecorp for dynamic calibrate */
			if (value >= ps_threshold_high) {	  /* 3cm high */
#ifdef USE_INTERRUPT
				ltr578_i2c_write_2_bytes(LTR578_PS_THRES_UP_0, 0x07FF);
				ltr578_i2c_write_2_bytes(LTR578_PS_THRES_LOW_0, ps_threshold_low);
#endif
				input_report_abs(pls->input, ABS_DISTANCE, 0);
				input_sync(pls->input);
			}
			else if (value <= ps_threshold_low) {		/* 5cm low */
				if (dyna_cali > 20 && value < (dyna_cali - 100))
				{
					if (value < 1500)
					{
						ps_threshold_high = value + 250;
						ps_threshold_low = value + 220;
					}
					else {
						ps_threshold_high = 1800;
						ps_threshold_low = 1600;
					}
				}
#ifdef USE_INTERRUPT
#ifdef USE_WAKELOCK_PS
				/* wake lock only when report 1 */
				wake_lock_timeout(&psensor_timeout_wakelock, msecs_to_jiffies(WAKELOCK_TIMEOUT_WORK_MS));
#endif
				ltr578_i2c_write_2_bytes(LTR578_PS_THRES_UP_0, ps_threshold_high);
				ltr578_i2c_write_2_bytes(LTR578_PS_THRES_LOW_0, 0x0000);
#endif
				input_report_abs(pls->input, ABS_DISTANCE, 1);
				input_sync(pls->input);
			}
		}
	}

	if (l_flag)
	{
		if (0x08 == (status & 0x08)) {/*is ALS*/
			value = ltr578_als_read();
			PRINT_DBG("ALS INT: ALS_DATA_VAL = 0x%04X(%4d)\n", value, value);
			input_report_abs(pls->input, ABS_MISC, value);
			input_sync(pls->input);
		}
	}
}

#ifdef USE_POLLING
static void als_ps_report_init(void)
{
	printk("zxs %s\n", __func__);

	if (als_poll_flag == 0)
	{
		als_poll_flag = 1;
		als_polling_tsk = kthread_run(als_polling_function, NULL, "als_polling");
	}
}

static int als_polling_function(void* arg)
{
	printk("zxs %s\n", __func__);
	init_completion(&als_thread_completion);
	while (1)
	{
		mdelay(5);
		ltr578_process_data(g_ltr578_data);

		if (als_poll_flag == 0)
			break;
	};

	complete(&als_thread_completion);
	return 0;
}
#endif

#ifdef USE_INTERRUPT
static void ltr578_work(struct work_struct *work)
{
	ltr578_t *pls = container_of(work, ltr578_t, work);
	printk("zxs %s\n", __func__);

	ltr578_process_data(pls);
    enable_irq(pls->client->irq);
}

static irqreturn_t ltr578_irq_handler(int irq, void *dev_id)
{
	ltr578_t *pls = (ltr578_t *) dev_id;
	printk("zxs %s\n", __func__);

#ifdef USE_WAKELOCK_PS
	wake_lock_timeout(&psensor_timeout_wakelock, msecs_to_jiffies(WAKELOCK_TIMEOUT_INT_MS));
#endif
	disable_irq_nosync(pls->client->irq);
	queue_work(pls->ltr_work_queue, &pls->work);
	return IRQ_HANDLED;
}
#endif

static int ltr578_reg_init(void)
{
	int res;
			
	res = ltr578_i2c_write_1_byte(LTR578_PS_PULSES, 32); //32pulses 
	if (res < 0)
	{
		PRINT_DBG("ltr578_reg_init() PS Pulses error...\n");
		goto EXIT_ERR;
	}
	res = ltr578_i2c_write_1_byte(LTR578_PS_LED, 0x36); // 60khz & 100mA 
	if (res < 0)
	{
		PRINT_DBG("ltr578_reg_init() PS LED error...\n");
		goto EXIT_ERR;
	}
	res = ltr578_i2c_write_1_byte(LTR578_PS_MEAS_RATE, 0x5C); // 11bits & 50ms time 
	if (res < 0)
	{
		PRINT_DBG("ltr578_reg_init() PS time error...\n");
		goto EXIT_ERR;
	}

	res = ltr578_i2c_write_1_byte(LTR578_INT_PST, 0x11);
	if (res < 0)
	{
		PRINT_DBG("ltr578_reg_init() PS persist error...\n");
		goto EXIT_ERR;
	}
	res = ltr578_i2c_write_1_byte(LTR578_INT_CFG, 0x15);
	if (res < 0)
	{
		PRINT_DBG("ltr578_reg_init() PS config error...\n");
		goto EXIT_ERR;
	}

	res = ltr578_i2c_write_1_byte(LTR578_ALS_GAIN, MODE_ALS_Range3);
	if (res < 0)
	{
		PRINT_DBG("ltr578_reg_init() ALS gain error...\n");
		goto EXIT_ERR;
	}
	res = ltr578_i2c_write_1_byte(LTR578_ALS_MEAS_RATE, ALS_RESO_MEAS);// 18 bit & 100ms measurement rate
	if (res < 0)
	{
		PRINT_DBG("ltr578_reg_init() ALS meas rate error...\n");
		goto EXIT_ERR;
	}
#ifdef USE_INTERRUPT  
    // als
    ltr578_i2c_write_1_byte(LTR578_ALS_THRES_UP_0, 0x00);
    ltr578_i2c_write_1_byte(LTR578_ALS_THRES_UP_1, 0x00);
	ltr578_i2c_write_1_byte(LTR578_ALS_THRES_UP_2, 0x00);
    ltr578_i2c_write_1_byte(LTR578_ALS_THRES_LOW_0, 0xff);
    ltr578_i2c_write_1_byte(LTR578_ALS_THRES_LOW_1, 0xff);
	ltr578_i2c_write_1_byte(LTR578_ALS_THRES_LOW_2, 0xff);
#endif
    msleep(WAKEUP_DELAY);

	PRINT_INFO("ltr578_reg_init success!\n");
	return 0;
EXIT_ERR:
	PRINT_INFO("ltr578_reg_init fail!\n");
	return -1;
}

static ssize_t ltr578_register_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int address, value;
	int len;

	/* echo "00 01" > regs */
	/* 0x00: 0x01 */
	len = sscanf(buf, "%3x %3x", &address, &value);
	PRINT_INFO("address=0x%02x, value=0x%02x\n", address, value);
	ltr578_i2c_write_1_byte(address, value);

	return count;
}

static ssize_t ltr578_register_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	size_t count = 0;
	u8 reg[0x40];
	int i;

	for (i = 0x00; i < 0x26; i++) {
		reg[i] = ltr578_i2c_read_1_byte(i);
		count += sprintf(&buf[count], "0x%02x: 0x%02x\n", i, reg[i]);
	}
	return count;
}

static DEVICE_ATTR(regs, 0644, ltr578_register_show, ltr578_register_store);

static struct attribute *ltr578_attributes[] = {
	&dev_attr_regs.attr,	
	NULL
};

static struct attribute_group ltr578_attribute_group = {
	.attrs = ltr578_attributes
};

static int ltr_578als_sysfs_init(struct input_dev *input_dev)
{
	struct kobject *ltr_578als_kobj;
	int ret = -1;

	ltr_578als_kobj = kobject_create_and_add("ltr_578als", &(input_dev->dev.kobj));
	if (ltr_578als_kobj == NULL) {
		ret = -ENOMEM;
		PRINT_ERR("register sysfs failed. ret = %d\n", ret);
		return ret;
	}

	ret = sysfs_create_group(ltr_578als_kobj,
			&ltr578_attribute_group);
	if (ret) {
		PRINT_ERR("create sysfs failed. ret = %d\n", ret);
		return ret;
	}
	return ret;
}

static int ltr578_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret = 0;
	ltr578_t *ltr_578als = NULL;
	struct input_dev *input_dev = NULL;
#ifdef USE_INTERRUPT
	struct ltr578_pls_platform_data *pdata = client->dev.platform_data;
#endif
	int chip_id = 0;

#ifdef USE_INTERRUPT
#ifdef CONFIG_OF
	struct device_node *np = client->dev.of_node;
	if (np && !pdata) {
		pdata = kzalloc(sizeof(*pdata), GFP_KERNEL);
		if (!pdata) {
			dev_err(&client->dev, "Could not allocate struct ltr578_pls_platform_data");
			goto exit_allocate_pdata_failed;
		}
        pdata->irq_gpio_number = of_get_named_gpio(np, "ltr578,irq-gpio", 0);
		if (pdata->irq_gpio_number < 0) {
			dev_err(&client->dev, "fail to get irq_gpio_number\n");
			kfree(pdata);
			goto exit_irq_gpio_read_fail;
		}
		client->dev.platform_data = pdata;
	}
#endif
#endif

#ifdef USE_INTERRUPT
	ret = gpio_request(pdata->irq_gpio_number, LTR578_PLS_IRQ_PIN);
	if (ret) {
		PRINT_ERR("gpio_request failed!\n");
		goto exit_gpio_request_failed;
	}
	gpio_direction_input(pdata->irq_gpio_number);
	client->irq = gpio_to_irq(pdata->irq_gpio_number);
	PRINT_INFO("client->irq = %d\n", client->irq);
#endif
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		PRINT_ERR("i2c_check_functionality failed!\n");
		ret = -ENODEV;
		goto exit_i2c_check_functionality_failed;
	}
	ltr_578als = kzalloc(sizeof(ltr578_t), GFP_KERNEL);
	if (!ltr_578als) {
		PRINT_ERR("kzalloc failed!\n");
		ret = -ENOMEM;
		goto exit_kzalloc_failed;
	}
	g_ltr578_data = ltr_578als;

	i2c_set_clientdata(client, ltr_578als);
	ltr_578als->client = client;
	this_client = client;

	chip_id = ltr578_i2c_read_1_byte(LTR578_PART_ID);
	if (chip_id < 0 || chip_id != LTR578_CHIP_ID) {
		PRINT_ERR("ltr578 read chip_id failed!\n");
		ret = -ENOMEM;
		goto exit_read_chip_id_failed;
	}
	
	input_dev = input_allocate_device();
	if (!input_dev) {
		PRINT_ERR("input_allocate_device failed!\n");
		ret = -ENOMEM;
		goto exit_input_allocate_device_failed;
	}

	input_dev->name = LTR578_INPUT_DEV;
	input_dev->phys = LTR578_INPUT_DEV;
	input_dev->id.bustype = BUS_I2C;
	input_dev->dev.parent = &client->dev;
	input_dev->id.vendor = 0x0001;
	input_dev->id.product = 0x0001;
	input_dev->id.version = 0x0010;
	ltr_578als->input = input_dev;

	__set_bit(EV_ABS, input_dev->evbit);
	input_set_abs_params(input_dev, ABS_DISTANCE, 0, 1, 0, 0);
	input_set_abs_params(input_dev, ABS_MISC, 0, 100001, 0, 0);
	ret = input_register_device(input_dev);
	if (ret < 0) {
		PRINT_ERR("input_register_device failed!\n");
		input_free_device(input_dev);
		input_dev = NULL;
		goto exit_input_register_device_failed;
	}
	ret = misc_register(&ltr578_device);
	if (ret) {
		PRINT_ERR("misc_register failed!\n");
		goto exit_misc_register_failed;
	}
	if (ltr578_reg_init() < 0) {
		PRINT_ERR("ltr578_reg_init failed!\n");
		ret = -1;
		goto exit_ltr578_reg_init_failed;
	}
#ifdef USE_INTERRUPT
	INIT_WORK(&ltr_578als->work, ltr578_work);
	ltr_578als->ltr_work_queue = create_singlethread_workqueue(LTR578_I2C_NAME);
	if (!ltr_578als->ltr_work_queue) {
		PRINT_ERR("create_singlethread_workqueue failed!\n");
		goto exit_create_singlethread_workqueue_failed;
	}
#ifdef USE_WAKELOCK_PS
	wake_lock_init(&psensor_timeout_wakelock, WAKE_LOCK_SUSPEND, "psensor timeout wakelock");
#endif
	if (client->irq > 0) {
		ret = request_irq(client->irq, ltr578_irq_handler, IRQ_TYPE_EDGE_FALLING | IRQF_NO_SUSPEND,	client->name, ltr_578als);
		if (ret < 0) {
			PRINT_ERR("request_irq failed!\n");
			goto exit_request_irq_failed;
		}
	}
#endif
	ret = ltr_578als_sysfs_init(input_dev);
	if (ret) {
		PRINT_ERR("ltr_578als_sysfs_init failed!\n");
		goto exit_ltr_578als_sysfs_init_failed;
	}

#ifdef USE_POLLING
	als_poll_flag = 0;
	als_ps_report_init();
#endif

	PRINT_INFO("probe success!\n");
	return 0;

exit_ltr_578als_sysfs_init_failed:
#ifdef USE_INTERRUPT
	free_irq(ltr_578als->client->irq, ltr_578als);
exit_request_irq_failed:
	destroy_workqueue(ltr_578als->ltr_work_queue);
	ltr_578als->ltr_work_queue = NULL;
#ifdef USE_WAKELOCK_PS
	wake_lock_destroy(&psensor_timeout_wakelock);
#endif
exit_create_singlethread_workqueue_failed:
#endif
exit_ltr578_reg_init_failed:
	misc_deregister(&ltr578_device);
exit_misc_register_failed:
	input_unregister_device(input_dev);
exit_input_register_device_failed:
exit_input_allocate_device_failed:
exit_read_chip_id_failed:
	kfree(ltr_578als);
	ltr_578als = NULL;
exit_kzalloc_failed:
exit_i2c_check_functionality_failed:
#ifdef USE_INTERRUPT
	gpio_free(pdata->irq_gpio_number);
exit_gpio_request_failed:
exit_irq_gpio_read_fail:
exit_allocate_pdata_failed:
#endif
	PRINT_ERR("probe failed!\n");
	return ret;
}

static int ltr578_remove(struct i2c_client *client)
{
	ltr578_t *ltr_578als = i2c_get_clientdata(client);

#ifdef USE_INTERRUPT
#ifdef USE_WAKELOCK_PS
	wake_lock_destroy(&psensor_timeout_wakelock);
#endif
	flush_workqueue(ltr_578als->ltr_work_queue);
	destroy_workqueue(ltr_578als->ltr_work_queue);
	ltr_578als->ltr_work_queue = NULL;
#endif
	misc_deregister(&ltr578_device);
	input_unregister_device(ltr_578als->input);
	ltr_578als->input = NULL;
#ifdef USE_INTERRUPT
	free_irq(ltr_578als->client->irq, ltr_578als);
#endif
	kfree(ltr_578als);
	ltr_578als = NULL;
	this_client = NULL;

	PRINT_INFO("ltr578_remove\n");
	return 0;
}

static const struct i2c_device_id ltr578_id[] = {
	{LTR578_I2C_NAME, 0},
	{}
};

static const struct of_device_id ltr578_of_match[] = {
	{ .compatible = "LITEON,ltr_578als", },
	{}
};

MODULE_DEVICE_TABLE(of, ltr578_of_match);

static struct i2c_driver ltr578_driver = {
	.driver = {
		.name = LTR578_I2C_NAME,
		.of_match_table = ltr578_of_match,
	},
	.probe = ltr578_probe,
	.remove = ltr578_remove,
	.id_table = ltr578_id,
};

static int __init ltr578_init(void)
{
	int ret = -1;

	ret = i2c_add_driver(&ltr578_driver);
	if (ret) {
		PRINT_ERR("i2c_add_driver failed!\n");
		return ret;
	}
	return ret;
}

static void __exit ltr578_exit(void)
{
	i2c_del_driver(&ltr578_driver);
}

late_initcall(ltr578_init);
module_exit(ltr578_exit);

MODULE_AUTHOR("Shi Zhigang <Zhigang.Shi@liteon.com>");
MODULE_DESCRIPTION("Proximity&Light Sensor LTR578ALS DRIVER");
MODULE_LICENSE("GPL");

