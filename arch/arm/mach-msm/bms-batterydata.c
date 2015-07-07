/* Copyright (c) 2011-2013, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/batterydata-lib.h>

static struct single_row_lut fcc_temp = {
	.x		= {-20, 0, 25, 40, 60},
	.y		= {2544,2535,2536,2533,2526},
	.cols		= 5
};

static struct pc_temp_ocv_lut pc_temp_ocv = {
	.rows		= 31,
	.cols		= 5,
	.temp		= {-20,0,25,40,60},
	.percent	= {100,95,90,85,80,75,70,65,60,55,50,45,40,35,30,25,20,16,13,11,10,9,8,7,6,5,4,3,2,1,0},
	.ocv		= {
				{4317,4318,4312,4304,4291},
				{4200,4227,4232,4230,4225},
				{4127,4168,4175,4173,4170},
				{4066,4115,4122,4120,4116},
				{3998,4064,4072,4070,4067},
				{3938,4010,4026,4024,4021},
				{3892,3958,3984,3982,3979},
				{3851,3914,3944,3945,3942},
				{3814,3878,3902,3907,3906},
				{3799,3846,3856,3860,3860},
				{3786,3820,3828,3828,3828},
				{3772,3798,3808,3808,3806},
				{3758,3778,3790,3791,3789},
				{3742,3766,3777,3777,3775},
				{3724,3752,3766,3762,3759},
				{3702,3730,3752,3746,3734},
				{3676,3706,3728,3724,3712},
				{3645,3694,3699,3697,3686},
				{3612,3684,3682,3676,3664},
				{3580,3678,3679,3672,3660},
				{3559,3672,3677,3670,3659},
				{3536,3663,3674,3668,3658},
				{3514,3649,3672,3666,3654},
				{3488,3630,3667,3663,3652},
				{3464,3607,3656,3656,3642},
				{3436,3582,3632,3630,3611},
				{3406,3550,3594,3584,3562},
				{3368,3510,3536,3522,3500},
				{3326,3452,3454,3438,3416},
				{3258,3342,3326,3308,3286},
				{3000,3000,3000,3000,3000}
	}
};

static struct sf_lut rbatt_sf = {
	.rows		= 30,
	.cols		= 5,
	.row_entries	= {-20,0,25,40,60},
	.percent	= {100,95,90,85,80,75,70,65,60,55,50,45,40,35,30,25,20,16,13,11,10,9,8,7,6,5,4,3,2,1},
	.sf		= {
				{968,227,100,90,91},
				{968,227,100,90,91},
				{931,234,102,91,92},
				{904,243,105,93,92},
				{861,251,108,95,93},
				{824,252,113,98,94},
				{814,243,120,101,96},
				{810,237,130,107,99},
				{811,232,128,114,105},
				{862,233,107,100,98},
				{922,233,104,93,92},
				{986,234,105,94,92},
				{1047,234,107,95,94},
				{1105,250,110,99,96},
				{1169,275,110,98,96},
				{1244,300,113,95,93},
				{1351,331,113,95,93},
				{1540,378,114,95,93},
				{1761,430,111,95,92},
				{1813,437,113,94,93},
				{2126,453,114,95,93},
				{2492,473,118,98,95},
				{2913,497,122,99,96},
				{3411,525,128,101,98},
				{4053,564,137,105,98},
				{4853,614,143,105,96},
				{6086,679,148,104,96},
				{7887,773,147,107,100},
				{10401,922,154,116,108},
				{14533,1305,195,146,130}
	}
};

struct bms_battery_data palladium_2500_data = {
	.fcc			= 2500,
	.fcc_temp_lut		= &fcc_temp,
	.pc_temp_ocv_lut	= &pc_temp_ocv,
	.rbatt_sf_lut		= &rbatt_sf,
	.default_rbatt_mohm	= 166,
	.rbatt_capacitive_mohm	= 50,
	.flat_ocv_threshold_uv	= 3800000,
};
struct bms_battery_data palladium_1500_data = {
        .fcc                    = 1500,
        .fcc_temp_lut           = &fcc_temp,
        .pc_temp_ocv_lut        = &pc_temp_ocv,
        .rbatt_sf_lut           = &rbatt_sf,
        .default_rbatt_mohm     = 236,
        .rbatt_capacitive_mohm  = 50,
        .flat_ocv_threshold_uv  = 3800000,
};
