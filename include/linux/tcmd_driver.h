

#ifndef __TCMD_H__
#define __TCMD_H__

#include <linux/mfd/pm8xxx/mpp.h>
#include <linux/mfd/pm8xxx/gpio.h>
#include <linux/mfd/pm8xxx/misc.h>
#include <linux/mfd/pm8xxx/vibrator.h>
#include <linux/mfd/pm8xxx/pm8xxx-adc.h>
#include <linux/qpnp/qpnp-adc.h>

typedef struct{
	char name[16];
	char cmd;
	int param;
}tcmd_reg_arg ;

typedef struct{
	char cmd;
	struct pm8xxx_adc_chan_result adc_config;
}TCMD_PM8XXX_ADC_CHAN_PARAM;

typedef struct{
	char cmd;
	struct qpnp_vadc_result adc_config;
}TCMD_QPNP_VADC_CHAN_PARAM;


typedef struct{
	char cmd;
	struct pm8xxx_mpp_config_data mpp_config;
}TCMD_PM8XXX_MPP_PARAM;

typedef struct{
	char cmd;
	struct pm_gpio gpio_config;
}TCMD_PM8XXX_GPIO_PARAM;

typedef struct qpnp_coincell_chg TCMD_QPNP_CC_CHG_PARAM;

typedef struct pm8xxx_vib_config TCMD_PM8XXX_VIB_PARAM;

typedef struct{
	uint8_t bus_id;
	uint8_t slv_addr_idx;
	uint8_t reg_offset_buf;
}TCMD_MHL_PARAM;
typedef unsigned char        UINT8;      /**< Unsigned 8 bit integer */

typedef struct{
        int usbPresentCurrent;
        int batPresentCurrent;
}TCMD_REG_USB_BAT_CURRENT;

typedef struct{
	int chip_id;
}TCMD_SLIMPORT_PARAM;

#define TCMD_PMIC_ADC_BATTERY_CURRENT 0x08
#define TCMD_IOCTL_BASE 0x0a
#define TCMD_IOCTL_MASK_INT	_IOW(TCMD_IOCTL_BASE, 0x01, int)
#define TCMD_IOCTL_UNMASK_INT _IOW(TCMD_IOCTL_BASE, 0x02, int)
#define TCMD_IOCTL_READ_INT	_IOWR(TCMD_IOCTL_BASE, 0x03, int)
#define TCMD_IOCTL_SET_INT _IOW(TCMD_IOCTL_BASE, 0x04, char)

#define TCMD_IOCTL_SET_REG _IOW(TCMD_IOCTL_BASE, 0x05, tcmd_reg_arg *)
#define TCMD_IOCTL_GET_REG _IOWR(TCMD_IOCTL_BASE, 0x06, tcmd_reg_arg *)
#define TCMD_IOCTL_CHARGER _IOW(TCMD_IOCTL_BASE, 0x07, unsigned char)

#define TCMD_IOCTL_POWER_DOWN _IOW(TCMD_IOCTL_BASE, 0x08, bool)
//for ADC
#define TCMD_IOCTL_SET_ADC _IOW(TCMD_IOCTL_BASE, 0x09, TCMD_PM8XXX_ADC_CHAN_PARAM *)
#define TCMD_IOCTL_GET_ADC _IOWR(TCMD_IOCTL_BASE, 0x0a, TCMD_QPNP_VADC_CHAN_PARAM *)

//for MPP
#define TCMD_IOCTL_SET_MPP _IOW(TCMD_IOCTL_BASE, 0x0b, TCMD_PM8XXX_MPP_PARAM *)
#define TCMD_IOCTL_GET_MPP _IOWR(TCMD_IOCTL_BASE, 0x0c, TCMD_PM8XXX_MPP_PARAM *)

//for GPIO
#define TCMD_IOCTL_SET_GPIO _IOW(TCMD_IOCTL_BASE, 0x0d, TCMD_PM8XXX_GPIO_PARAM *)
#define TCMD_IOCTL_GET_GPIO _IOWR(TCMD_IOCTL_BASE, 0x0e, TCMD_PM8XXX_GPIO_PARAM *)

//for coincell
#define TCMD_IOCTL_SET_COINCELL _IOW(TCMD_IOCTL_BASE, 0x0f, TCMD_QPNP_CC_CHG_PARAM *)

//for vibrator
#define TCMD_IOCTL_SET_VIBRATOR _IOW(TCMD_IOCTL_BASE, 0x10, TCMD_PM8XXX_VIB_PARAM *)

//for MHL
#define TCMD_IOCTL_READ_MHL _IOWR(TCMD_IOCTL_BASE, 0x12, TCMD_MHL_PARAM *)

//for flash_lm3559 Torch Mode
#define TCMD_IOCTL_SET_FLASH_LM3559 _IOW(TCMD_IOCTL_BASE, 0x11, int *)

// for emmc
#define TCMD_IOCTL_READ_MMC_EXT_CSD _IOWR(TCMD_IOCTL_BASE, 0x15, UINT8 *)

//For USB charging Disable/Enable
#define TCMD_IOCTL_USB_CHRGING _IOW(TCMD_IOCTL_BASE, 0x16, unsigned char)

// For reading present USB and BATTERY current
#define TCMD_IOCTL_GET_USB_BAT_CURRENT _IOWR(TCMD_IOCTL_BASE, 0x17, TCMD_REG_USB_BAT_CURRENT*)

// To set USB and BAT current
#define TCMD_IOCTL_SET_USB_BAT_CURRENT _IOW(TCMD_IOCTL_BASE, 0x18, TCMD_REG_USB_BAT_CURRENT*)

//for Coincall enable disable
#define TCMD_IOCTL_COINCELL_ENABLE_DISABLE _IOW(TCMD_IOCTL_BASE, 0x19, int *)

//for charging path priority
#define TCMD_IOCTL_CHGPTH_PRIORITY _IOW(TCMD_IOCTL_BASE, 0x20, unsigned char)

#define TCMD_IOCTL_GET_SLIMPORT_CHIPID _IOWR(TCMD_IOCTL_BASE, 0x21, TCMD_SLIMPORT_PARAM*)

#define GPIO_MAP_NAME_SIZE 20

struct gpio_mapping {
	u32 used;
	u32 pin_num;
	char name[GPIO_MAP_NAME_SIZE];
};

extern void __init gpio_mapping_init(struct gpio_mapping *table, int size);

enum tcmd_gpio_enum {
	TCMD_GPIO_ISL29030_INT = 0,
	TCMD_GPIO_KXTF9_INT,
	TCMD_GPIO_MMC_DETECT,
	TCMD_GPIO_WC_STAT_0,
	TCMD_GPIO_WC_STAT_1,
	TCMD_GPIO_WC_CTRL_0,
	TCMD_GPIO_WC_CTRL_1,
	TCMD_GPIO_WC_CHG_COMPLETE,
	TCMD_GPIO_WC_CHG_TERMINATE,
	TCMD_GPIO_INT_MAX_NUM
};

/** Regulator action parameters */
typedef enum
{
	TCMD_REG_ENABLE = 0x00,
	TCMD_REG_DISABLE = 0x01,
	TCMD_REG_SET_VOLTAGE = 0x02,
	TCMD_REG_GET_VOLTAGE = 0x03,
	TCMD_REG_SET_POWER_MODE = 0x04,
	TCMD_REG_GET_POWER_MODE = 0x05

} TCMD_REG_ACTION_T;

struct tcmd_gpio_set_arg {
	unsigned char gpio;
	unsigned char gpio_state;
};

struct tcmd_driver_platform_data {
	int size;
	int gpio_list[TCMD_GPIO_INT_MAX_NUM+1];
};

#endif /* __TCMD_H__ */
