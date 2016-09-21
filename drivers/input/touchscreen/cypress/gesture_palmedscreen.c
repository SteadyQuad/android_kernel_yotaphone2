/*
 * Gesture recognition driver: palmed screen recognition
 *
 * Copyright (c) 2013,2014,2015 YotaDevices.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 */

#include <linux/kernel.h>
#include <linux/cyttsp5_core.h>
#include <linux/cyttsp5_bus.h>
#include "cyttsp5_device_access.h"
#include "cyttsp5_regs.h"
#include "gesture_driver.h"

#define PALMED_ALGORITHM_MULTICAP 1
#define PALMED_ALGORITHM_SELFCAP_LOGISTIC_REGRESSION 2
#define PALMED_ALGORITHM_SELFCAP_LOGISTIC_REGRESSION_FRONT 3
#define PALMED_ALGORITHM_SELFCAP_LOGISTIC_REGRESSION_BACK 4

#define PALMED_LR_DATA_SIZE (1 + CY_FS_SELFCAP_SIZE + CY_BS_SELFCAP_SIZE)

extern u8 fs_sensor_data[CY_MAX_PRBUF_SIZE/2];
extern u8 bs_sensor_data[CY_MAX_PRBUF_SIZE/2];

static int32_t selfcap_lr_data[PALMED_LR_DATA_SIZE];

// Both screens are active
static int32_t calib_vector[PALMED_LR_DATA_SIZE] = {
	-6129,
	-375,
	-326,
	-137,
	-2,
	73,
	89,
	108,
	84,
	31,
	-15,
	-95,
	-7,
	217,
	96,
	-509,
	-763,
	1511,
	246,
	331,
	741,
	-117,
	98,
	-22,
	127,
	57,
	29,
	16,
	14,
	31,
	1,
	136,
	109,
	276,
	232,
	207,
	279,
	-251,
	210
};

// Use value only from the back screen
static int32_t calib_vector_back[PALMED_LR_DATA_SIZE] = {
	-45675,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	2457,
	-227,
	126,
	571,
	-39,
	178,
	-83,
	122,
	25,
	1,
	-9,
	13,
	-2,
	9,
	48,
	57,
	331,
	140,
	-40,
	227,
	-273,
	132
};

// Use values only from the front screen
static int32_t calib_vector_front[PALMED_LR_DATA_SIZE] = {
	32087,
	-195,
	-175,
	195,
	-383,
	318,
	133,
	-120,
	257,
	-205,
	461,
	-458,
	88,
	171,
	-56,
	-65,
	-802,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0
};
// Constants to determine front/back screen is palmed
#define FRONT_LR_THRESHOLD 138629
#define BACK_LR_THRESHOLD (-138629)

static inline int sensor_threshold(s16 data, int threshold)
{
	return data > threshold ? data : 0;
}

static int sensor_calc_fs_sum(s16 *pdata16, int len, int threshold)
{
	int row;
	s16 *p;
	int sum = 0;

	for (row = 14; row < CY_FS_ROW-5; row++) {
		p = pdata16+row*CY_FS_COLUMN;
		sum += sensor_threshold(*p, threshold);
		*p= -5;

		p = pdata16+row*CY_FS_COLUMN + 1;
		sum += sensor_threshold(*p, threshold);
		*p = -5;

		p = pdata16+row*CY_FS_COLUMN + CY_FS_COLUMN-1;
		sum += sensor_threshold(*p, threshold);
		*p = -5;

		p = pdata16+row*CY_FS_COLUMN + CY_FS_COLUMN-2;
		sum += sensor_threshold(*p, threshold);
		*p= -5;
	}
	return sum;
}

static int sensor_calc_bs_sum(s16 *pdata16, int len, int threshold)
{
	int sum = 0;
	int offset;
	s16 *p;

	for (p = pdata16 + CY_BS_COLUMN, offset = 9; offset < CY_BS_COLUMN; offset++) {
		sum += sensor_threshold(*(p+offset), threshold);
		*(p+offset) = 0;
	}

	for (p = pdata16 + CY_BS_COLUMN*(CY_BS_ROW - 2), offset = 7; offset < CY_BS_COLUMN; offset++) {
		sum += sensor_threshold(*(p+offset), threshold);
		*(p+offset) = -1;
		sum += sensor_threshold(*(p+offset + CY_BS_COLUMN), threshold);
		*(p+offset + CY_BS_COLUMN) = -2;
	}

	return sum;
}

static int sensor_calc_fs_middle_area(s16 *pdata16, int len, int threshold)
{
	int sum = 0;
	int row, column;
	s16 *p = pdata16;

	for (row = 2; row < CY_FS_ROW/2; row++) {
		p = pdata16 + row*CY_FS_COLUMN;
		for (column = 2; column < CY_FS_COLUMN-2; column++) {
			sum += sensor_threshold(*(p + column), threshold);
			*(p+column) = -7;
		}
	}
	return sum;
}

static int sensor_calc_bs_middle_area(s16 *pdata16, int len, int threshold)
{
	int sum = 0;
	int row, column;
	s16 *p = pdata16;

	for (row = 2; row < CY_BS_ROW - 2; row++) {
		p = pdata16 + row*CY_BS_COLUMN;
		for (column = 2; column < CY_BS_COLUMN/2; column++) {
			sum += sensor_threshold(*(p + column), threshold);
			*(p+column) = -5;
		}
	}
	return sum;
}

#if 0
static int sensor_dump(s16 *pdata16, int rows, int cols)
{
	int i;
	printk("\nrows=%d, cols=%d\n", rows, cols);
	for (i = 0; i < cols*rows; i++) {
		if ((i % cols) == 0)
			printk("\n");
		printk("%03d ", pdata16[i]);
	}
	return 0;
}
#endif

static void sensor_normalize_sum(int *front_sum, int *back_sum, int fs_len, int bs_len)
{
	//printk("\nfront_sum=%d, back_sum=%d\n", *front_sum, *back_sum);
	// Back touch has less touch sensors -> need to correct ratio
	*back_sum = *back_sum*fs_len/bs_len;
	// Normalisation because of non-equal sensor data for front and back:
        // max bs data~2300, fs data=600, ratio=4
	*front_sum = *front_sum * 3;
}

static int palmed_screen(int front_sum, int back_sum) {
	if (abs(front_sum - back_sum) < 500)
		return -1;
	if (abs(front_sum) >= abs(back_sum))
		return 0;
	else
		return 1;
	return -1;
}

static int palmed_multicap(void)
{
	u16 read_offset, read_count;
	u8 config;
	u16 actual_read_len;
	int screen = -1;
	int front_sum = 0, back_sum = 0;
	const int fs_data_len = CY_FS_ROW*CY_FS_COLUMN;
	const int bs_data_len = CY_BS_ROW*CY_BS_COLUMN;

	read_offset = 0;
	read_count = fs_data_len;
	cyttsp5_read_sensor_data(CYTTSP5_DEVICE_ACCESS_NAME, CY_FRONT_CORE_ID,
				 1, read_offset, read_count, CY_MUT_DIFF, NULL,
				 &config, &actual_read_len, fs_sensor_data);

	//	sensor_dump((s16 *)fs_sensor_data, CY_FS_ROW, CY_FS_COLUMN);
	front_sum = sensor_calc_fs_sum((s16 *)fs_sensor_data, fs_data_len, 50);
	//	sensor_dump((s16 *)fs_sensor_data, CY_FS_ROW, CY_FS_COLUMN);

	read_offset = 0;
	read_count = bs_data_len;
	cyttsp5_read_sensor_data(CYTTSP5_DEVICE_ACCESS_NAME, CY_BACK_CORE_ID,
				 1, read_offset, read_count, CY_MUT_DIFF, NULL,
				 &config, &actual_read_len, bs_sensor_data);

	//	sensor_dump((s16 *)bs_sensor_data, CY_BS_ROW, CY_BS_COLUMN);
	back_sum = sensor_calc_bs_sum((s16 *)bs_sensor_data, bs_data_len, 150);
	//	sensor_dump((s16 *)bs_sensor_data, CY_BS_ROW, CY_BS_COLUMN);

	sensor_normalize_sum(&front_sum, &back_sum, fs_data_len, bs_data_len);
	screen = palmed_screen(front_sum, back_sum);

	//printk(KERN_INFO "%s: Palmed screen - 1st alg=%d\n", __func__, screen);

	if (screen == -1) {
		front_sum = sensor_calc_fs_middle_area((s16 *)fs_sensor_data, fs_data_len, 50);
		back_sum = sensor_calc_bs_middle_area((s16 *)bs_sensor_data, bs_data_len, 150);

		//sensor_dump((s16 *)fs_sensor_data, CY_FS_ROW, CY_FS_COLUMN);
		//sensor_dump((s16 *)bs_sensor_data, CY_BS_ROW, CY_BS_COLUMN);

		sensor_normalize_sum(&front_sum, &back_sum, fs_data_len, bs_data_len);
		screen = palmed_screen(front_sum, back_sum);

		//printk(KERN_INFO "%s: Palmed screen - 2nd alg=%d\n", __func__, screen);
	}
	printk(KERN_INFO "%s: Palmed multicap, screen=%d\n", __func__, screen);
	return screen;
}

static int palmed_logistic_regression(int algorithm)
{
	int screen = -1;
	u16 read_offset = 0, read_count = 0;
	u16 actual_read_len;
	u8 config = 0;
	int i, data_offset = 0;
	int estimate = 0;
        int32_t *cl_vec;

        if (algorithm == PALMED_ALGORITHM_SELFCAP_LOGISTIC_REGRESSION_FRONT)
		cl_vec = calib_vector_front;
	else if (algorithm == PALMED_ALGORITHM_SELFCAP_LOGISTIC_REGRESSION_BACK)
		cl_vec = calib_vector_back;
	else
		cl_vec = calib_vector;

	// Initialize selfcap_data array.
	memset(selfcap_lr_data, 0, sizeof(selfcap_lr_data));
	// selfcap_lr_data[0] should be 1 for algorithm consistency
	selfcap_lr_data[0] = 1;
	data_offset++;

	if (algorithm == PALMED_ALGORITHM_SELFCAP_LOGISTIC_REGRESSION_BACK) {
		// Only back data are used
		data_offset += CY_FS_SELFCAP_SIZE;
	}
	else {
		config = 0;
		read_offset = 0;
		read_count = CY_FS_SELFCAP_SIZE;
		cyttsp5_read_sensor_data(CYTTSP5_DEVICE_ACCESS_NAME, CY_FRONT_CORE_ID,
					 1, read_offset, read_count, CY_SELF_DIFF, NULL,
					 &config, &actual_read_len, fs_sensor_data);
		for (i = 0; i < CY_FS_SELFCAP_SIZE; i++) {
			selfcap_lr_data[data_offset++] = *(((s16*)fs_sensor_data)+i);
		}
	}

	if (algorithm == PALMED_ALGORITHM_SELFCAP_LOGISTIC_REGRESSION_FRONT) {
		// Only front data are used
		data_offset += CY_BS_SELFCAP_SIZE;
	}
	else {
		config = 0;
		read_offset = 0;
		read_count = CY_BS_SELFCAP_SIZE;
		cyttsp5_read_sensor_data(CYTTSP5_DEVICE_ACCESS_NAME, CY_BACK_CORE_ID,
					 1, read_offset, read_count, CY_SELF_DIFF, NULL,
					 &config, &actual_read_len, bs_sensor_data);
		for (i = 0; i < CY_BS_SELFCAP_SIZE; i++) {
			selfcap_lr_data[data_offset++] = *(((s16*)bs_sensor_data)+i);
		}
	}

	for (i = 0; i < PALMED_LR_DATA_SIZE; i++) {
		estimate += selfcap_lr_data[i]*cl_vec[i];
	}

	if (estimate > FRONT_LR_THRESHOLD)
		screen = 1;
	else if (estimate < BACK_LR_THRESHOLD)
		screen = 0;
	else screen = -1;
	printk(KERN_INFO "%s: Palmed selfcap logistic regression (alg %d), estimate: %d, screen=%d\n", __func__, algorithm, estimate, screen);
	return screen;
}

int get_palmed_screen(int algorithm)
{
	int front_suspended = cyttsp5_is_suspended(CYTTSP5_DEVICE_ACCESS_NAME, CY_FRONT_CORE_ID);
	int back_suspended = cyttsp5_is_suspended(CYTTSP5_DEVICE_ACCESS_NAME, CY_BACK_CORE_ID);
	if (algorithm == PALMED_ALGORITHM_MULTICAP) {
		if (front_suspended || back_suspended) {
			printk(KERN_ERR "%s: Multicap algorithm is not supported for suspended touch\n", __func__);
			return -1;
		}
		return palmed_multicap();
	}
	else {
		// use logistic regression algorithm by default
		if (front_suspended && back_suspended) {
			printk(KERN_ERR "%s: Can not use logistic regression algorithm\n", __func__);
			return -1;
		}
		else if (front_suspended)
			algorithm = PALMED_ALGORITHM_SELFCAP_LOGISTIC_REGRESSION_BACK;
		else if (back_suspended)
			algorithm = PALMED_ALGORITHM_SELFCAP_LOGISTIC_REGRESSION_FRONT;
		else
			algorithm = PALMED_ALGORITHM_SELFCAP_LOGISTIC_REGRESSION;
		return palmed_logistic_regression(algorithm);
	}
}

