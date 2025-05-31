#ifndef _LINUX_LTR_578ALS_H
#define _LINUX_LTR_578ALS_H

#include <linux/types.h>
#include <linux/ioctl.h>

#define LTR578_I2C_NAME      "ltr578"
#define LTR578_INPUT_DEV     "ltr578_pls"
#define LTR578_I2C_ADDR      0x53
#define LTR578_PLS_IRQ_PIN   "ltr578_irq_pin"
#define LTR578_PS_THRESHOLD  0x03B6

/*LTR-578ALS IO Control*/
#define LTR_IOCTL_MAGIC         0x1C
#define LTR_IOCTL_GET_PFLAG     _IOR(LTR_IOCTL_MAGIC, 1, int)
#define LTR_IOCTL_GET_LFLAG     _IOR(LTR_IOCTL_MAGIC, 2, int)
#define LTR_IOCTL_SET_PFLAG     _IOW(LTR_IOCTL_MAGIC, 3, int)
#define LTR_IOCTL_SET_LFLAG     _IOW(LTR_IOCTL_MAGIC, 4, int)
#define LTR_IOCTL_GET_DATA      _IOW(LTR_IOCTL_MAGIC, 5, unsigned char)
#define LTR_IOCTL_GET_CHIPINFO  _IOW(LTR_IOCTL_MAGIC, 6, unsigned char)

/*LTR-578ALS Registers*/
#define LTR578_MAIN_CTRL		0x00
#define LTR578_PS_LED			0x01
#define LTR578_PS_PULSES		0x02
#define LTR578_PS_MEAS_RATE		0x03
#define LTR578_ALS_MEAS_RATE	0x04
#define LTR578_ALS_GAIN			0x05

#define LTR578_INT_CFG			0x19
#define LTR578_INT_PST 			0x1A
#define LTR578_PS_THRES_UP_0	0x1B
#define LTR578_PS_THRES_UP_1	0x1C
#define LTR578_PS_THRES_LOW_0	0x1D
#define LTR578_PS_THRES_LOW_1	0x1E
#define LTR578_PS_CAN_0			0x1F
#define LTR578_PS_CAN_1			0x20
#define LTR578_ALS_THRES_UP_0	0x21
#define LTR578_ALS_THRES_UP_1	0x22
#define LTR578_ALS_THRES_UP_2	0x23
#define LTR578_ALS_THRES_LOW_0	0x24
#define LTR578_ALS_THRES_LOW_1	0x25
#define LTR578_ALS_THRES_LOW_2	0x26

/* 578's Read Only Registers */
#define LTR578_PART_ID			0x06
#define LTR578_MAIN_STATUS		0x07
#define LTR578_PS_DATA_0		0x08
#define LTR578_PS_DATA_1		0x09
#define LTR578_CLEAR_DATA_0		0x0A
#define LTR578_CLEAR_DATA_1		0x0B
#define LTR578_CLEAR_DATA_2		0x0C
#define LTR578_ALS_DATA_0		0x0D
#define LTR578_ALS_DATA_1		0x0E
#define LTR578_ALS_DATA_2		0x0F

/*Basic Operating Modes*/
#define MODE_ALS_Range1			0x00  ///for als gain x1
#define MODE_ALS_Range3			0x01  ///for als gain x3
#define MODE_ALS_Range6			0x02  ///for als gain x6
#define MODE_ALS_Range9			0x03  ///for als gain x9
#define MODE_ALS_Range18		0x04  ///for als gain x18

#define ALS_RANGE_1				1
#define ALS_RANGE_3 			3
#define ALS_RANGE_6 			6
#define ALS_RANGE_9 			9
#define ALS_RANGE_18			18

#define ALS_RESO_MEAS			0x22

#define ALS_WIN_FACTOR			1
#define ALS_WIN_FACTOR2			5
#define ALS_USE_CLEAR_DATA		0

#define PON_DELAY				600
#define WAKEUP_DELAY			10

#define LTR578_CHIP_ID			0xB1

//#define USE_INTERRUPT
#define USE_POLLING

struct ltr578_pls_platform_data {
        int irq_gpio_number;
};

#endif
