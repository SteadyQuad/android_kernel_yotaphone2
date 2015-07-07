/*
 * cyttsp5_loader.c
 * Cypress TrueTouch(TM) Standard Product V5 FW Loader Module.
 * For use with Cypress Txx5xx parts.
 * Supported parts include:
 * TMA5XX
 *
 * Copyright (C) 2012-2013 Cypress Semiconductor, Inc.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2, and only version 2, as published by the
 * Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * Contact Cypress Semiconductor at www.cypress.com <ttdrivers@cypress.com>
 *
 */

#include <asm/unaligned.h>
#include <linux/delay.h>
#include <linux/pm_runtime.h>
#include <linux/slab.h>
#include <linux/firmware.h>
#include <linux/cyttsp5_bus.h>
#include <linux/cyttsp5_core.h>
#include "cyttsp5_regs.h"

#define CYTTSP5_LOADER_NAME "cyttsp5_loader"
#define CYTTSP5_AUTO_LOAD_FOR_CORRUPTED_FW 1
#define CYTTSP5_LOADER_FW_UPGRADE_RETRY_COUNT 3

#define CYTTSP5_FW_UPGRADE \
	(defined(CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP5_PLATFORM_FW_UPGRADE) \
	|| defined(CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP5_BINARY_FW_UPGRADE))

#define CYTTSP5_TTCONFIG_UPGRADE \
	(defined(CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP5_PLATFORM_TTCONFIG_UPGRADE) \
	|| defined(CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP5_MANUAL_TTCONFIG_UPGRADE))

static const u8 cyttsp5_security_key[] = {
	0xA5, 0x01, 0x02, 0x03, 0xFF, 0xFE, 0xFD, 0x5A
};

/* Timeout values in ms. */
#define CY_LDR_REQUEST_EXCLUSIVE_TIMEOUT		500
#define CY_LDR_SWITCH_TO_APP_MODE_TIMEOUT		300

#define CY_MAX_STATUS_SIZE				32

#define CY_DATA_MAX_ROW_SIZE				256
#define CY_DATA_ROW_SIZE				128

#define CY_ARRAY_ID_OFFSET				0
#define CY_ROW_NUM_OFFSET				1
#define CY_ROW_SIZE_OFFSET				3
#define CY_ROW_DATA_OFFSET				5

#define CY_POST_TT_CFG_CRC_MASK				0x2

struct cyttsp5_loader_data {
	struct cyttsp5_device *ttsp;
	struct cyttsp5_sysinfo *si;
	u8 status_buf[CY_MAX_STATUS_SIZE];
	struct completion int_running;
#ifdef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP5_BINARY_FW_UPGRADE
	struct completion builtin_bin_fw_complete;
	int builtin_bin_fw_status;
	bool is_manual_upgrade_enabled;
#endif
	struct work_struct fw_and_config_upgrade;
	struct work_struct calibration_work;
	struct cyttsp5_loader_platform_data *loader_pdata;
#ifdef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP5_MANUAL_TTCONFIG_UPGRADE
	struct mutex config_lock;
	u8 *config_data;
	int config_size;
	bool config_loading;
#endif
};

struct cyttsp5_dev_id {
	u32 silicon_id;
	u8 rev_id;
	u32 bl_ver;
};

struct cyttsp5_hex_image {
	u8 array_id;
	u16 row_num;
	u16 row_size;
	u8 row_data[CY_DATA_ROW_SIZE];
} __packed;

#if CYTTSP5_FW_UPGRADE || CYTTSP5_TTCONFIG_UPGRADE
/*
 * return code:
 * -1: Do not upgrade firmware
 *  0: Version info same, let caller decide
 *  1: Do a firmware upgrade
 */
static int cyttsp5_check_firmware_version(struct cyttsp5_device *ttsp,
		u32 fw_ver_new, u32 fw_revctrl_new)
{
	struct device *dev = &ttsp->dev;
	struct cyttsp5_loader_data *data = dev_get_drvdata(dev);
	u32 fw_ver_img;
	u32 fw_revctrl_img;

	fw_ver_img = data->si->cydata.fw_ver_major << 8;
	fw_ver_img += data->si->cydata.fw_ver_minor;

	dev_dbg(dev, "%s: img vers:0x%04X new vers:0x%04X\n", __func__,
			fw_ver_img, fw_ver_new);

	if (fw_ver_new > fw_ver_img) {
		dev_dbg(dev, "%s: Image is newer, will upgrade\n",
				__func__);
		return 1;
	}

	if (fw_ver_new < fw_ver_img) {
		dev_dbg(dev, "%s: Image is older, will NOT upgrade\n",
				__func__);
		return -1;
	}

	fw_revctrl_img = data->si->cydata.revctrl;

	dev_dbg(dev, "%s: img revctrl:0x%04X new revctrl:0x%04X\n",
			__func__, fw_revctrl_img, fw_revctrl_new);

	if (fw_revctrl_new > fw_revctrl_img) {
		dev_dbg(dev, "%s: Image is newer, will upgrade\n",
				__func__);
		return 1;
	}

	if (fw_revctrl_new < fw_revctrl_img) {
		dev_dbg(dev, "%s: Image is older, will NOT upgrade\n",
				__func__);
		return -1;
	}

	return 0;
}
#endif /* CYTTSP5_FW_UPGRADE || CYTTSP5_TTCONFIG_UPGRADE */


#if CYTTSP5_FW_UPGRADE
static u8 *cyttsp5_get_row_(struct cyttsp5_device *ttsp, u8 *row_buf,
		u8 *image_buf, int size)
{
	memcpy(row_buf, image_buf, size);
	return image_buf + size;
}

static int cyttsp5_ldr_enter_(struct cyttsp5_device *ttsp,
		struct cyttsp5_dev_id *dev_id)
{
	int rc;
	u8 return_data[8];
	u8 mode;

	dev_id->silicon_id = 0;
	dev_id->rev_id = 0;
	dev_id->bl_ver = 0;

	cyttsp5_request_reset(ttsp);
	msleep(CY_LDR_SWITCH_TO_APP_MODE_TIMEOUT);

	rc = cyttsp5_request_get_mode(ttsp, 0, &mode);
	if (rc < 0)
		return rc;

	if (mode == CY_MODE_UNKNOWN)
		return -EINVAL;

	if (mode == CY_MODE_OPERATIONAL) {
		rc = cyttsp5_request_nonhid_start_bl(ttsp, 0);
		if (rc < 0)
			return rc;
	}

	rc = cyttsp5_request_nonhid_get_bl_info(ttsp, 0, return_data);
	if (rc < 0)
		return rc;

	dev_id->silicon_id = get_unaligned_le32(&return_data[0]);
	dev_id->rev_id = return_data[4];
	dev_id->bl_ver = return_data[5] + (return_data[6] << 8)
		+ (return_data[7] << 16);

	return 0;
}

static int cyttsp5_ldr_init_(struct cyttsp5_device *ttsp,
		struct cyttsp5_hex_image *row_image)
{
	return cyttsp5_request_nonhid_initiate_bl(ttsp, 0, 8,
			(u8 *)cyttsp5_security_key, row_image->row_size,
			row_image->row_data);
}

static int cyttsp5_ldr_parse_row_(struct cyttsp5_device *ttsp, u8 *row_buf,
	struct cyttsp5_hex_image *row_image)
{
	int rc = 0;

	row_image->array_id = row_buf[CY_ARRAY_ID_OFFSET];
	row_image->row_num = get_unaligned_be16(&row_buf[CY_ROW_NUM_OFFSET]);
	row_image->row_size = get_unaligned_be16(&row_buf[CY_ROW_SIZE_OFFSET]);

	if (row_image->row_size > ARRAY_SIZE(row_image->row_data)) {
		dev_err(&ttsp->dev,
			"%s: row data buffer overflow\n", __func__);
		rc = -EOVERFLOW;
		goto cyttsp5_ldr_parse_row_exit;
	}

	memcpy(row_image->row_data, &row_buf[CY_ROW_DATA_OFFSET],
	       row_image->row_size);
cyttsp5_ldr_parse_row_exit:
	return rc;
}

static int cyttsp5_ldr_prog_row_(struct cyttsp5_device *ttsp,
				 struct cyttsp5_hex_image *row_image)
{
	u16 length = row_image->row_size + 3;
	u8 data[3 + row_image->row_size];
	u8 offset = 0;

	data[offset++] = row_image->array_id;
	data[offset++] = LOW_BYTE(row_image->row_num);
	data[offset++] = HI_BYTE(row_image->row_num);
	memcpy(data + 3, row_image->row_data, row_image->row_size);
	return cyttsp5_request_nonhid_prog_and_verify(ttsp, 0, length, data);
}

static int cyttsp5_ldr_verify_chksum_(struct cyttsp5_device *ttsp)
{
	u8 result;
	int rc;

	rc = cyttsp5_request_nonhid_verify_app_integrity(ttsp, 0, &result);
	if (rc)
		return rc;

	/* fail */
	if (result == 0)
		return -EINVAL;

	return 0;
}

static int cyttsp5_ldr_exit_(struct cyttsp5_device *ttsp)
{
	return cyttsp5_request_nonhid_launch_app(ttsp, 0);
}

static int cyttsp5_load_app_(struct cyttsp5_device *ttsp, const u8 *fw,
			     int fw_size)
{
	struct device *dev = &ttsp->dev;
	struct cyttsp5_dev_id *dev_id;
	struct cyttsp5_hex_image *row_image;
	u8 *row_buf;
	size_t image_rec_size;
	size_t row_buf_size = CY_DATA_MAX_ROW_SIZE;
	int row_count = 0;
	u8 *p;
	u8 *last_row;
	int rc;
	int rc_tmp;

	image_rec_size = sizeof(struct cyttsp5_hex_image);
	if (fw_size % image_rec_size != 0) {
		dev_err(dev, "%s: Firmware image is misaligned\n", __func__);
		rc = -EINVAL;
		goto _cyttsp5_load_app_error;
	}

	dev_info(dev, "%s: start load app\n", __func__);
#ifdef TTHE_TUNER_SUPPORT
	cyttsp5_request_tthe_print(ttsp, NULL, 0, "start load app");
#endif

	row_buf = kzalloc(row_buf_size, GFP_KERNEL);
	row_image = kzalloc(sizeof(struct cyttsp5_hex_image), GFP_KERNEL);
	dev_id = kzalloc(sizeof(struct cyttsp5_dev_id), GFP_KERNEL);
	if (row_buf == NULL || row_image == NULL || dev_id == NULL) {
		dev_err(dev, "%s: Unable to alloc row buffers(%p %p %p)\n",
			__func__, row_buf, row_image, dev_id);
		rc = -ENOMEM;
		goto _cyttsp5_load_app_exit;
	}

	cyttsp5_request_stop_wd(ttsp);

	dev_info(dev, "%s: Send BL Loader Enter\n", __func__);
#ifdef TTHE_TUNER_SUPPORT
	cyttsp5_request_tthe_print(ttsp, NULL, 0, "Send BL Loader Enter");
#endif
	rc = cyttsp5_ldr_enter_(ttsp, dev_id);
	if (rc) {
		dev_err(dev, "%s: Error cannot start Loader (ret=%d)\n",
			__func__, rc);
		goto _cyttsp5_load_app_exit;
	}
	dev_vdbg(dev, "%s: dev: silicon id=%08X rev=%02X bl=%08X\n",
		__func__, dev_id->silicon_id,
		dev_id->rev_id, dev_id->bl_ver);

	/* get last row */
	last_row = (u8 *)fw + fw_size - image_rec_size;
	cyttsp5_get_row_(ttsp, row_buf, last_row, image_rec_size);
	cyttsp5_ldr_parse_row_(ttsp, row_buf, row_image);

	/* initialise bootloader */
	rc = cyttsp5_ldr_init_(ttsp, row_image);
	if (rc) {
		dev_err(dev, "%s: Error cannot init Loader (ret=%d)\n",
			__func__, rc);
		goto _cyttsp5_load_app_exit;
	}

	dev_info(dev, "%s: Send BL Loader Blocks\n", __func__);
#ifdef TTHE_TUNER_SUPPORT
	cyttsp5_request_tthe_print(ttsp, NULL, 0, "Send BL Loader Blocks");
#endif
	p = (u8 *)fw;
	while (p < last_row) {
		/* Get row */
		dev_dbg(dev, "%s: read row=%d\n", __func__, ++row_count);
		memset(row_buf, 0, row_buf_size);
		p = cyttsp5_get_row_(ttsp, row_buf, p, image_rec_size);

		/* Parse row */
		dev_vdbg(dev, "%s: p=%p buf=%p buf[0]=%02X\n",
			__func__, p, row_buf, row_buf[0]);
		rc = cyttsp5_ldr_parse_row_(ttsp, row_buf, row_image);
		dev_vdbg(dev,
			"%s: array_id=%02X row_num=%04X(%d) row_size=%04X(%d)\n",
			__func__, row_image->array_id,
			row_image->row_num, row_image->row_num,
			row_image->row_size, row_image->row_size);
		if (rc) {
			dev_err(dev, "%s: Parse Row Error (a=%d r=%d ret=%d\n",
				__func__, row_image->array_id,
				row_image->row_num, rc);
			goto _cyttsp5_load_app_exit;
		} else {
			dev_vdbg(dev, "%s: Parse Row (a=%d r=%d ret=%d\n",
				__func__, row_image->array_id,
				row_image->row_num, rc);
		}

		/* program row */
		rc = cyttsp5_ldr_prog_row_(ttsp, row_image);
		if (rc) {
			dev_err(dev,
				"%s: Program Row Error (array=%d row=%d ret=%d)\n",
				__func__, row_image->array_id,
				row_image->row_num, rc);
			goto _cyttsp5_load_app_exit;
		}

		dev_vdbg(dev, "%s: array=%d row_cnt=%d row_num=%04X\n",
			__func__, row_image->array_id, row_count,
			row_image->row_num);
	}

	/* exit loader */
	dev_info(dev, "%s: Send BL Loader Terminate\n", __func__);
#ifdef TTHE_TUNER_SUPPORT
	cyttsp5_request_tthe_print(ttsp, NULL, 0, "Send BL Loader Terminate");
#endif
	rc = cyttsp5_ldr_exit_(ttsp);
	if (rc) {
		dev_err(dev, "%s: Error on exit Loader (ret=%d)\n",
			__func__, rc);

		/* verify app checksum */
		rc_tmp = cyttsp5_ldr_verify_chksum_(ttsp);
		if (rc_tmp)
			dev_err(dev, "%s: ldr_verify_chksum fail r=%d\n",
				__func__, rc_tmp);
		else
			dev_info(dev, "%s: APP Checksum Verified\n", __func__);
	}

_cyttsp5_load_app_exit:
	kfree(row_buf);
	kfree(row_image);
	kfree(dev_id);
_cyttsp5_load_app_error:
	return rc;
}

static void cyttsp5_fw_calibrate(struct work_struct *calibration_work)
{
	struct cyttsp5_loader_data *data = container_of(calibration_work,
			struct cyttsp5_loader_data, calibration_work);
	struct cyttsp5_device *ttsp = data->ttsp;
	struct device *dev = &ttsp->dev;
	u8 mode;
	int rc;

	pm_runtime_get_sync(dev);

	rc = cyttsp5_request_exclusive(ttsp, CY_LDR_REQUEST_EXCLUSIVE_TIMEOUT);
	if (rc < 0)
		goto exit;

	rc = cyttsp5_request_nonhid_suspend_scanning(ttsp, 0);
	if (rc < 0)
		goto release;

	for (mode = 0; mode < 3; mode++) {
		rc = cyttsp5_request_nonhid_calibrate_idacs(ttsp, 0, mode);
		if (rc < 0)
			goto release;
	}

	rc = cyttsp5_request_nonhid_resume_scanning(ttsp, 0);
	if (rc < 0)
		goto release;

	dev_dbg(dev, "%s: Calibration Done\n", __func__);

release:
	cyttsp5_release_exclusive(ttsp);
exit:
	pm_runtime_put(dev);
}

static int cyttsp5_fw_calibration_attention(struct cyttsp5_device *ttsp)
{
	struct device *dev = &ttsp->dev;
	struct cyttsp5_loader_data *data = dev_get_drvdata(dev);
	int rc = 0;

	schedule_work(&data->calibration_work);

	cyttsp5_unsubscribe_attention(ttsp, CY_ATTEN_STARTUP,
		cyttsp5_fw_calibration_attention, 0);

	return rc;
}

static int cyttsp5_upgrade_firmware(struct cyttsp5_device *ttsp,
		const u8 *fw_img, int fw_size)
{
	struct device *dev = &ttsp->dev;
	struct cyttsp5_loader_data *data = dev_get_drvdata(dev);
	int retry = CYTTSP5_LOADER_FW_UPGRADE_RETRY_COUNT;
	int rc;

	pm_runtime_get_sync(dev);

	rc = cyttsp5_request_exclusive(ttsp, CY_LDR_REQUEST_EXCLUSIVE_TIMEOUT);
	if (rc < 0) {
		pm_runtime_put(dev);
		goto exit;
	}

	while (retry--) {
		rc = cyttsp5_load_app_(ttsp, fw_img, fw_size);
		if (rc < 0)
			dev_err(dev, "%s: Firmware update failed rc=%d, retry:%d\n",
				__func__, rc, retry);
		else
			break;
		msleep(20);
	}
	if (rc < 0) {
		dev_err(dev, "%s: Firmware update failed with error code %d\n",
			__func__, rc);
	} else if (data->loader_pdata &&
			(data->loader_pdata->flags
			 & CY_LOADER_FLAG_CALIBRATE_AFTER_FW_UPGRADE)) {
		/* set up call back for startup */
		dev_vdbg(dev, "%s: Adding callback for calibration\n",
			__func__);
		rc = cyttsp5_subscribe_attention(ttsp, CY_ATTEN_STARTUP,
				cyttsp5_fw_calibration_attention, 0);
		if (rc) {
			dev_err(dev, "%s: Failed adding callback for calibration\n",
				__func__);
			dev_err(dev, "%s: No calibration will be performed\n",
				__func__);
			rc = 0;
		}
	}

	cyttsp5_release_exclusive(ttsp);
	pm_runtime_put_sync(dev);

	cyttsp5_request_restart(ttsp);
exit:
	return rc;
}

static int cyttsp5_loader_attention(struct cyttsp5_device *ttsp)
{
	struct cyttsp5_loader_data *data = dev_get_drvdata(&ttsp->dev);
	complete(&data->int_running);
	return 0;
}
#endif /* CYTTSP5_FW_UPGRADE */

#ifdef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP5_PLATFORM_FW_UPGRADE
static int cyttsp5_check_firmware_version_platform(struct cyttsp5_device *ttsp,
		struct cyttsp5_touch_firmware *fw)
{
	struct device *dev = &ttsp->dev;
	struct cyttsp5_loader_data *data = dev_get_drvdata(dev);
	u32 fw_ver_new;
	u32 fw_revctrl_new;
	int upgrade;

	if (!data->si) {
		dev_info(dev, "%s: No firmware infomation found, device FW may be corrupted\n",
			__func__);
		return CYTTSP5_AUTO_LOAD_FOR_CORRUPTED_FW;
	}

	fw_ver_new = get_unaligned_be16(fw->ver + 2);
	/* 4 middle bytes are not used */
	fw_revctrl_new = get_unaligned_be32(fw->ver + 8);

	upgrade = cyttsp5_check_firmware_version(ttsp, fw_ver_new,
		fw_revctrl_new);

	if (upgrade > 0)
		return 1;

	return 0;
}

static int upgrade_firmware_from_platform(struct cyttsp5_device *ttsp)
{
	struct device *dev = &ttsp->dev;
	struct cyttsp5_loader_data *data = dev_get_drvdata(dev);
	struct cyttsp5_touch_firmware *fw;
	int rc = -ENOSYS;
	int upgrade;

	if (data->loader_pdata == NULL) {
		dev_err(dev, "%s: No loader platform data\n", __func__);
		return rc;
	}

	fw = data->loader_pdata->fw;
	if (fw == NULL || fw->img == NULL || fw->size == 0) {
		dev_err(dev, "%s: No platform firmware\n", __func__);
		return rc;
	}

	if (fw->ver == NULL || fw->vsize == 0) {
		dev_err(dev, "%s: No platform firmware version\n",
			__func__);
		return rc;
	}

	upgrade = cyttsp5_check_firmware_version_platform(ttsp, fw);
	if (upgrade)
		return cyttsp5_upgrade_firmware(ttsp, fw->img, fw->size);

	return rc;
}
#endif /* CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP5_PLATFORM_FW_UPGRADE */

#ifdef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP5_BINARY_FW_UPGRADE
static void _cyttsp5_firmware_cont(const struct firmware *fw, void *context)
{
	struct cyttsp5_device *ttsp = context;
	struct device *dev = &ttsp->dev;
	struct cyttsp5_loader_data *data = dev_get_drvdata(dev);
	u8 header_size = 0;

	if (fw == NULL)
		goto cyttsp5_firmware_cont_exit;

	if (fw->data == NULL || fw->size == 0) {
		dev_err(dev,
			"%s: No firmware received\n", __func__);
		goto cyttsp5_firmware_cont_release_exit;
	}

	header_size = fw->data[0];
	if (header_size >= (fw->size + 1)) {
		dev_err(dev,
			"%s: Firmware format is invalid\n", __func__);
		goto cyttsp5_firmware_cont_release_exit;
	}

	cyttsp5_upgrade_firmware(ttsp, &(fw->data[header_size + 1]),
		fw->size - (header_size + 1));

cyttsp5_firmware_cont_release_exit:
	release_firmware(fw);

cyttsp5_firmware_cont_exit:
	data->is_manual_upgrade_enabled = 0;
	return;
}

static int cyttsp5_check_firmware_version_builtin(struct cyttsp5_device *ttsp,
		const struct firmware *fw)
{
	struct device *dev = &ttsp->dev;
	struct cyttsp5_loader_data *data = dev_get_drvdata(dev);
	u32 fw_ver_new;
	u32 fw_revctrl_new;
	int upgrade;

	if (!data->si) {
		dev_info(dev, "%s: No firmware infomation found, device FW may be corrupted\n",
			__func__);
		return CYTTSP5_AUTO_LOAD_FOR_CORRUPTED_FW;
	}

	fw_ver_new = get_unaligned_be16(fw->data + 3);
	/* 4 middle bytes are not used */
	fw_revctrl_new = get_unaligned_be32(fw->data + 9);

	upgrade = cyttsp5_check_firmware_version(ttsp, fw_ver_new,
			fw_revctrl_new);

	if (upgrade > 0)
		return 1;

	return 0;
}

static void _cyttsp5_firmware_cont_builtin(const struct firmware *fw,
		void *context)
{
	struct cyttsp5_device *ttsp = context;
	struct device *dev = &ttsp->dev;
	struct cyttsp5_loader_data *data = dev_get_drvdata(dev);
	int upgrade;

	if (fw == NULL) {
		dev_info(dev, "%s: No builtin firmware\n", __func__);
		goto _cyttsp5_firmware_cont_builtin_exit;
	}

	if (fw->data == NULL || fw->size == 0) {
		dev_err(dev, "%s: Invalid builtin firmware\n", __func__);
		goto _cyttsp5_firmware_cont_builtin_exit;
	}

	dev_dbg(dev, "%s: Found firmware\n", __func__);

	upgrade = cyttsp5_check_firmware_version_builtin(ttsp, fw);
	if (upgrade) {
		_cyttsp5_firmware_cont(fw, ttsp);
		data->builtin_bin_fw_status = 0;
		complete(&data->builtin_bin_fw_complete);
		return;
	}

_cyttsp5_firmware_cont_builtin_exit:
	release_firmware(fw);

	data->builtin_bin_fw_status = -EINVAL;
	complete(&data->builtin_bin_fw_complete);
}

static int upgrade_firmware_from_class(struct cyttsp5_device *ttsp)
{
	int retval;

	dev_vdbg(&ttsp->dev, "%s: Enabling firmware class loader\n", __func__);

	retval = request_firmware_nowait(THIS_MODULE, FW_ACTION_NOHOTPLUG, "",
			&ttsp->dev, GFP_KERNEL, ttsp, _cyttsp5_firmware_cont);
	if (retval < 0) {
		dev_err(&ttsp->dev, "%s: Fail request firmware class file load\n",
			__func__);
		return retval;
	}

	return 0;
}

static int upgrade_firmware_from_builtin(struct cyttsp5_device *ttsp)
{
	struct device *dev = &ttsp->dev;
	struct cyttsp5_loader_data *data = dev_get_drvdata(dev);
	int retval = -1;

	dev_vdbg(dev, "%s: Enabling firmware class loader built-in\n",
		__func__);

	if (!strcmp(ttsp->core_id,CY_BACK_CORE_ID)){
		retval = request_firmware_nowait(THIS_MODULE, FW_ACTION_HOTPLUG,
			CY_FW_BACK_FILE_NAME, dev, GFP_KERNEL, ttsp,
			_cyttsp5_firmware_cont_builtin);
	}
	else if(!strcmp(ttsp->core_id,CY_FRONT_CORE_ID)){
		retval = request_firmware_nowait(THIS_MODULE, FW_ACTION_HOTPLUG,
                        CY_FW_FRONT_FILE_NAME, dev, GFP_KERNEL, ttsp,
                        _cyttsp5_firmware_cont_builtin);
	}

	if (retval < 0) {
		dev_err(dev, "%s: Fail request firmware class file load\n",
			__func__);
		return retval;
	}

	/* wait until FW binary upgrade finishes */
	wait_for_completion(&data->builtin_bin_fw_complete);

	return data->builtin_bin_fw_status;
}
#endif /* CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP5_BINARY_FW_UPGRADE */

#if CYTTSP5_TTCONFIG_UPGRADE
static int cyttsp5_write_config_row_(struct cyttsp5_device *ttsp, u8 ebid,
		u16 row_number, u16 row_size, u8 *data)
{
	int rc;
	u16 actual_write_len;

	rc = cyttsp5_request_nonhid_write_conf_block(ttsp, 0, row_number,
			row_size, ebid, data, (u8 *)cyttsp5_security_key,
			&actual_write_len);
	if (rc) {
		dev_err(&ttsp->dev,
			"%s: Fail Put EBID=%d row=%d cmd fail r=%d\n",
			__func__, ebid, row_number, rc);
		return rc;
	}

	if (actual_write_len != row_size) {
		dev_err(&ttsp->dev,
			"%s: Fail Put EBID=%d row=%d wrong write size=%d\n",
			__func__, ebid, row_number, actual_write_len);
		rc = -EINVAL;
	}

	return rc;
}

static int cyttsp5_upgrade_ttconfig(struct cyttsp5_device *ttsp,
		const u8 *ttconfig_data, int ttconfig_size)
{
	struct device *dev = &ttsp->dev;
	u8 ebid = CY_TCH_PARM_EBID;
	u16 row_size = CY_DATA_ROW_SIZE;
	u16 table_size;
	u16 row_count;
	u16 residue;
	u8 *row_buf;
	u8 verify_crc_status;
	u16 calculated_crc;
	u16 stored_crc;
	int rc = 0;
	int i;

	table_size = ttconfig_size;
	row_count = table_size / row_size;
	row_buf = (u8 *)ttconfig_data;
	dev_dbg(dev, "%s: size:%d row_size=%d row_count=%d\n",
		__func__, table_size, row_size, row_count);

	pm_runtime_get_sync(dev);

	rc = cyttsp5_request_exclusive(ttsp, CY_LDR_REQUEST_EXCLUSIVE_TIMEOUT);
	if (rc < 0) {
		pm_runtime_put(dev);
		goto exit;
	}

	rc = cyttsp5_request_nonhid_suspend_scanning(ttsp, 0);
	if (rc < 0)
		goto release;

	for (i = 0; i < row_count; i++) {
		dev_dbg(dev, "%s: row=%d size=%d\n", __func__, i, row_size);
		rc = cyttsp5_write_config_row_(ttsp, ebid, i, row_size,
				row_buf);
		if (rc) {
			dev_err(dev, "%s: Fail put row=%d r=%d\n",
				__func__, i, rc);
			break;
		}
		row_buf += row_size;
	}
	if (!rc) {
		residue = table_size % row_size;
		dev_dbg(dev, "%s: row=%d size=%d\n", __func__, i, residue);
		rc = cyttsp5_write_config_row_(ttsp, ebid, i, residue,
				row_buf);
		row_count++;
		if (rc)
			dev_err(dev, "%s: Fail put row=%d r=%d\n",
				__func__, i, rc);
	}

	if (!rc)
		dev_dbg(dev, "%s: TT_CFG updated: rows:%d bytes:%d\n",
			__func__, row_count, table_size);

	rc = cyttsp5_request_nonhid_verify_config_block_crc(ttsp, 0, ebid,
			&verify_crc_status, &calculated_crc, &stored_crc);
	if (rc || verify_crc_status)
		dev_err(dev,
			"%s: CRC Failed, ebid=%d, status=%d, scrc=%X ccrc=%X\n",
			__func__, ebid, verify_crc_status,
			calculated_crc, stored_crc);
	else
		dev_dbg(dev,
			"%s: CRC PASS, ebid=%d, status=%d, scrc=%X ccrc=%X\n",
			__func__, ebid, verify_crc_status,
			calculated_crc, stored_crc);

	rc = cyttsp5_request_nonhid_resume_scanning(ttsp, 0);
	if (rc < 0)
		goto release;


release:
	cyttsp5_release_exclusive(ttsp);
	pm_runtime_put_sync(dev);
	if (!rc)
		cyttsp5_request_restart(ttsp);
exit:
	return rc;
}
#endif /* CYTTSP5_TTCONFIG_UPGRADE */

#ifdef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP5_PLATFORM_TTCONFIG_UPGRADE
static int cyttsp5_get_ttconfig_crc(struct cyttsp5_device *ttsp,
		const u8 *ttconfig_data, int ttconfig_size, u16 *crc)
{
	u16 crc_loc;

	crc_loc = get_unaligned_le16(&ttconfig_data[2]);
	if (ttconfig_size < crc_loc + 2)
		return -EINVAL;

	*crc = get_unaligned_le16(&ttconfig_data[crc_loc]);

	return 0;
}

static int cyttsp5_check_ttconfig_version(struct cyttsp5_device *ttsp,
		const u8 *ttconfig_data, int ttconfig_size)
{
	struct device *dev = &ttsp->dev;
	struct cyttsp5_loader_data *data = dev_get_drvdata(dev);
	u16 cfg_crc_new;
	int rc;

	/* Check if config CRC different. */
	rc = cyttsp5_get_ttconfig_crc(ttsp, ttconfig_data, ttconfig_size,
			&cfg_crc_new);
	if (rc)
		return 0;

	if (!data->si)
		return 0;

	dev_dbg(dev, "%s: img_crc:0x%04X new_crc:0x%04X\n",
		__func__, data->si->ttconfig.crc, cfg_crc_new);

	if (cfg_crc_new != data->si->ttconfig.crc) {
		dev_dbg(dev, "%s: Config CRC different, will upgrade\n",
			__func__);
		return 1;
	}

	dev_dbg(dev, "%s: Config CRC equal, will NOT upgrade\n", __func__);
	return 0;
}

static int cyttsp5_check_ttconfig_version_platform(struct cyttsp5_device *ttsp,
		struct cyttsp5_touch_config *ttconfig)
{
	struct device *dev = &ttsp->dev;
	struct cyttsp5_loader_data *data = dev_get_drvdata(dev);
	u32 fw_ver_config;
	u32 fw_revctrl_config;

	if (!data->si) {
		dev_info(dev, "%s: No firmware infomation found, device FW may be corrupted\n",
			__func__);
		return 0;
	}

	fw_ver_config = get_unaligned_be16(ttconfig->fw_ver + 2);
	/* 4 middle bytes are not used */
	fw_revctrl_config = get_unaligned_be32(ttconfig->fw_ver + 8);

	/* FW versions should match */
	if (cyttsp5_check_firmware_version(ttsp, fw_ver_config,
			fw_revctrl_config)) {
		dev_err(dev, "%s: FW versions mismatch\n", __func__);
		return 0;
	}

	/* Check PowerOn Self Test, TT_CFG CRC bit */
	if ((data->si->cydata.post_code & CY_POST_TT_CFG_CRC_MASK) == 0) {
		dev_dbg(dev, "%s: POST, TT_CFG failed (%X), will upgrade\n",
			__func__, data->si->cydata.post_code);
		return 1;
	}

	return cyttsp5_check_ttconfig_version(ttsp, ttconfig->param_regs->data,
			ttconfig->param_regs->size);
}

static int upgrade_ttconfig_from_platform(struct cyttsp5_device *ttsp)
{
	struct device *dev = &ttsp->dev;
	struct cyttsp5_loader_data *data = dev_get_drvdata(dev);
	struct cyttsp5_touch_config *ttconfig;
	struct touch_settings *param_regs;
	struct cyttsp5_touch_fw;
	int rc = -ENOSYS;
	int upgrade;

	if (data->loader_pdata == NULL) {
		dev_info(dev, "%s: No loader platform data\n", __func__);
		return rc;
	}

	ttconfig = data->loader_pdata->ttconfig;
	if (ttconfig == NULL) {
		dev_info(dev, "%s: No ttconfig data\n", __func__);
		return rc;
	}

	param_regs = ttconfig->param_regs;
	if (param_regs == NULL) {
		dev_info(dev, "%s: No touch parameters\n", __func__);
		return rc;
	}

	if (param_regs->data == NULL || param_regs->size == 0) {
		dev_info(dev, "%s: Invalid touch parameters\n", __func__);
		return rc;
	}

	if (ttconfig->fw_ver == NULL || ttconfig->fw_vsize == 0) {
		dev_info(dev, "%s: Invalid FW version for touch parameters\n",
			__func__);
		return rc;
	}

	upgrade = cyttsp5_check_ttconfig_version_platform(ttsp, ttconfig);
	if (upgrade)
		return cyttsp5_upgrade_ttconfig(ttsp, param_regs->data,
				param_regs->size);

	return rc;
}
#endif /* CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP5_PLATFORM_TTCONFIG_UPGRADE */

#ifdef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP5_MANUAL_TTCONFIG_UPGRADE
static ssize_t cyttsp5_config_data_write(struct file *filp,
		struct kobject *kobj, struct bin_attribute *bin_attr,
		char *buf, loff_t offset, size_t count)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct cyttsp5_loader_data *data = dev_get_drvdata(dev);
	u8 *p;

	dev_vdbg(dev, "%s: offset:%lld count:%d\n", __func__, offset, count);

	mutex_lock(&data->config_lock);

	if (!data->config_loading) {
		mutex_unlock(&data->config_lock);
		return -ENODEV;
	}

	p = krealloc(data->config_data, offset + count, GFP_KERNEL);
	if (!p) {
		kfree(data->config_data);
		data->config_data = NULL;
		mutex_unlock(&data->config_lock);
		return -ENOMEM;
	}
	data->config_data = p;

	memcpy(&data->config_data[offset], buf, count);
	data->config_size += count;

	mutex_unlock(&data->config_lock);

	return count;
}

static struct bin_attribute bin_attr_config_data = {
	.attr = {
		.name = "config_data",
		.mode = S_IWUSR,
	},
	.size = 0,
	.write = cyttsp5_config_data_write,
};

static ssize_t cyttsp5_config_loading_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cyttsp5_loader_data *data = dev_get_drvdata(dev);
	bool config_loading;

	mutex_lock(&data->config_lock);
	config_loading = data->config_loading;
	mutex_unlock(&data->config_lock);

	return sprintf(buf, "%d\n", config_loading);
}

static int cyttsp5_verify_ttconfig_binary(struct cyttsp5_device *ttsp,
		u8 *bin_config_data, int bin_config_size, u8 **start, int *len)
{
	struct device *dev = &ttsp->dev;
	struct cyttsp5_loader_data *data = dev_get_drvdata(dev);
	int header_size;
	u16 config_size;
	u32 fw_ver_config;
	u32 fw_revctrl_config;

	if (!data->si) {
		dev_err(dev, "%s: No firmware infomation found, device FW may be corrupted\n",
			__func__);
		return -ENODEV;
	}

	/*
	 * We need 11 bytes for FW version control info and at
	 * least 6 bytes in config (Length + Max Length + CRC)
	 */
	header_size = bin_config_data[0] + 1;
	if (header_size < 11 || header_size >= bin_config_size - 6) {
		dev_err(dev, "%s: Invalid header size %d\n", __func__,
			header_size);
		return -EINVAL;
	}

	fw_ver_config = get_unaligned_be16(&bin_config_data[1]);
	/* 4 middle bytes are not used */
	fw_revctrl_config = get_unaligned_be32(&bin_config_data[7]);

	/* FW versions should match */
	if (cyttsp5_check_firmware_version(ttsp, fw_ver_config,
			fw_revctrl_config)) {
		dev_err(dev, "%s: FW versions mismatch\n", __func__);
		return -EINVAL;
	}

	config_size = get_unaligned_le16(&bin_config_data[header_size]);
	/* Perform a simple size check (2 bytes for CRC) */
	if (config_size != bin_config_size - header_size - 2) {
		dev_err(dev, "%s: Config size invalid\n", __func__);
		return -EINVAL;
	}

	*start = &bin_config_data[header_size];
	*len = bin_config_size - header_size;

	return 0;
}

/*
 * 1: Start loading TT Config
 * 0: End loading TT Config and perform upgrade
 *-1: Exit loading
 */
static ssize_t cyttsp5_config_loading_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct cyttsp5_loader_data *data = dev_get_drvdata(dev);
	long value;
	u8 *start;
	int length;
	int rc;

	rc = kstrtol(buf, 10, &value);
	if (rc < 0 || value < -1 || value > 1) {
		dev_err(dev, "%s: Invalid value\n", __func__);
		return size;
	}

	mutex_lock(&data->config_lock);

	if (value == 1)
		data->config_loading = true;
	else if (value == -1)
		data->config_loading = false;
	else if (value == 0 && data->config_loading) {
		data->config_loading = false;
		if (data->config_size == 0) {
			dev_err(dev, "%s: No config data\n", __func__);
			goto exit_free;
		}

		rc = cyttsp5_verify_ttconfig_binary(data->ttsp,
				data->config_data, data->config_size,
				&start, &length);
		if (rc)
			goto exit_free;

		rc = cyttsp5_upgrade_ttconfig(data->ttsp, start, length);
	}

exit_free:
	kfree(data->config_data);
	data->config_data = NULL;
	data->config_size = 0;

	mutex_unlock(&data->config_lock);

	if (rc)
		return rc;

	return size;
}

static DEVICE_ATTR(config_loading, S_IRUSR | S_IWUSR,
	cyttsp5_config_loading_show, cyttsp5_config_loading_store);
#endif /* CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP5_MANUAL_TTCONFIG_UPGRADE */

static void cyttsp5_fw_and_config_upgrade(
		struct work_struct *fw_and_config_upgrade)
{
	struct cyttsp5_loader_data *data = container_of(fw_and_config_upgrade,
			struct cyttsp5_loader_data, fw_and_config_upgrade);
	struct cyttsp5_device *ttsp = data->ttsp;
	struct device *dev = &ttsp->dev;

	data->si = cyttsp5_request_sysinfo(ttsp);
	if (data->si == NULL)
		dev_err(dev, "%s: Fail get sysinfo pointer from core\n",
			__func__);
#if !CYTTSP5_FW_UPGRADE
	dev_info(dev, "%s: No FW upgrade method selected!\n", __func__);
#endif

#ifdef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP5_PLATFORM_FW_UPGRADE
	if (!upgrade_firmware_from_platform(ttsp))
		return;
#endif

#ifdef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP5_BINARY_FW_UPGRADE
	if (!upgrade_firmware_from_builtin(ttsp))
		return;
#endif

#ifdef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP5_PLATFORM_TTCONFIG_UPGRADE
	if (!upgrade_ttconfig_from_platform(ttsp))
		return;
#endif
}

#ifdef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP5_BINARY_FW_UPGRADE
static ssize_t cyttsp5_manual_upgrade_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct cyttsp5_loader_data *data = dev_get_drvdata(dev);
	int rc;

	if (data->is_manual_upgrade_enabled)
		return -EBUSY;
	else
		data->is_manual_upgrade_enabled = 1;

	rc = upgrade_firmware_from_class(data->ttsp);

	if (rc < 0)
		data->is_manual_upgrade_enabled = 0;

	return size;
}

static DEVICE_ATTR(manual_upgrade, S_IRUSR | S_IWUSR,
	NULL, cyttsp5_manual_upgrade_store);
#endif

static int cyttsp5_loader_probe(struct cyttsp5_device *ttsp)
{
	struct cyttsp5_loader_data *data;
	struct device *dev = &ttsp->dev;
	int rc;

	data = kzalloc(sizeof(*data), GFP_KERNEL);
	if (data == NULL) {
		dev_err(dev, "%s: Error, kzalloc\n", __func__);
		rc = -ENOMEM;
		goto error_alloc_data_failed;
	}

#ifdef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP5_BINARY_FW_UPGRADE
	rc = device_create_file(dev, &dev_attr_manual_upgrade);
	if (rc) {
		dev_err(dev, "%s: Error, could not create manual_upgrade\n",
				__func__);
		goto error_create_manual_upgrade;
	}
#endif

#ifdef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP5_MANUAL_TTCONFIG_UPGRADE
	rc = device_create_file(dev, &dev_attr_config_loading);
	if (rc) {
		dev_err(dev, "%s: Error, could not create config_loading\n",
				__func__);
		goto error_create_config_loading;
	}

	rc = device_create_bin_file(dev, &bin_attr_config_data);
	if (rc) {
		dev_err(dev, "%s: Error, could not create config_data\n",
				__func__);
		goto error_create_config_data;
	}
#endif

	data->loader_pdata = cyttsp5_request_loader_pdata(ttsp);
	data->ttsp = ttsp;
	dev_set_drvdata(dev, data);

	pm_runtime_enable(dev);

#if CYTTSP5_FW_UPGRADE
	init_completion(&data->int_running);
#ifdef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP5_BINARY_FW_UPGRADE
	init_completion(&data->builtin_bin_fw_complete);
#endif
	cyttsp5_subscribe_attention(ttsp, CY_ATTEN_IRQ,
		cyttsp5_loader_attention, CY_MODE_BOOTLOADER);

	INIT_WORK(&data->calibration_work, cyttsp5_fw_calibrate);
#endif

#ifdef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP5_MANUAL_TTCONFIG_UPGRADE
	mutex_init(&data->config_lock);
#endif
	INIT_WORK(&data->fw_and_config_upgrade, cyttsp5_fw_and_config_upgrade);
	schedule_work(&data->fw_and_config_upgrade);

	dev_info(dev, "%s: Successful probe %s\n", __func__, ttsp->name);
	return 0;


#ifdef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP5_MANUAL_TTCONFIG_UPGRADE
error_create_config_data:
	device_remove_file(dev, &dev_attr_config_loading);
error_create_config_loading:
#ifdef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP5_BINARY_FW_UPGRADE
	device_remove_file(dev, &dev_attr_manual_upgrade);
#endif
#endif
#ifdef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP5_BINARY_FW_UPGRADE
error_create_manual_upgrade:
#endif
	kfree(data);
error_alloc_data_failed:
	dev_err(dev, "%s failed.\n", __func__);
	return rc;
}

static int cyttsp5_loader_release(struct cyttsp5_device *ttsp)
{
	struct device *dev = &ttsp->dev;
	struct cyttsp5_loader_data *data = dev_get_drvdata(dev);
	int retval = 0;

#if CYTTSP5_FW_UPGRADE
	retval = cyttsp5_unsubscribe_attention(ttsp, CY_ATTEN_IRQ,
		cyttsp5_loader_attention, CY_MODE_BOOTLOADER);
	if (retval < 0) {
		dev_err(dev,
			"%s: Failed to restart IC with error code %d\n",
			__func__, retval);
	}
#endif
	pm_runtime_suspend(dev);
	pm_runtime_disable(dev);
#ifdef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP5_MANUAL_TTCONFIG_UPGRADE
	device_remove_bin_file(dev, &bin_attr_config_data);
	device_remove_file(dev, &dev_attr_config_loading);
	kfree(data->config_data);
#endif
#ifdef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP5_BINARY_FW_UPGRADE
	device_remove_file(dev, &dev_attr_manual_upgrade);
#endif
	dev_set_drvdata(dev, NULL);
	kfree(data);
	return retval;
}

static struct cyttsp5_driver cyttsp5_loader_driver = {
	.probe = cyttsp5_loader_probe,
	.remove = cyttsp5_loader_release,
	.driver = {
		.name = CYTTSP5_LOADER_NAME,
		.bus = &cyttsp5_bus_type,
		.owner = THIS_MODULE,
	},
};

static const char cyttsp5_loader_name[] = CYTTSP5_LOADER_NAME;
static struct cyttsp5_device_info cyttsp5_loader_infos[CY_MAX_NUM_CORE_DEVS];

static char *core_ids[CY_MAX_NUM_CORE_DEVS] = {
	CY_FRONT_CORE_ID,
	CY_BACK_CORE_ID,
	NULL,
	NULL,
	NULL
};

static int num_core_ids = 2;

module_param_array(core_ids, charp, &num_core_ids, 0);
MODULE_PARM_DESC(core_ids,
	"Core id list of cyttsp5 core devices for loader module");

static int __init cyttsp5_loader_init(void)
{
	int rc = 0;
	int i, j;

	/* Check for invalid or duplicate core_ids */
	for (i = 0; i < num_core_ids; i++) {
		if (!strlen(core_ids[i])) {
			pr_err("%s: core_id %d is empty\n",
				__func__, i+1);
			return -EINVAL;
		}
		for (j = i+1; j < num_core_ids; j++)
			if (!strcmp(core_ids[i], core_ids[j])) {
				pr_err("%s: core_ids %d and %d are same\n",
					__func__, i+1, j+1);
				return -EINVAL;
			}
	}

	for (i = 0; i < num_core_ids; i++) {
		cyttsp5_loader_infos[i].name = cyttsp5_loader_name;
		cyttsp5_loader_infos[i].core_id = core_ids[i];
		pr_info("%s: Registering loader device for core_id: %s\n",
			__func__, cyttsp5_loader_infos[i].core_id);
		rc = cyttsp5_register_device(&cyttsp5_loader_infos[i]);
		if (rc < 0) {
			pr_err("%s: Error, failed registering device\n",
				__func__);
			goto fail_unregister_devices;
		}
	}
	rc = cyttsp5_register_driver(&cyttsp5_loader_driver);
	if (rc) {
		pr_err("%s: Error, failed registering driver\n", __func__);
		goto fail_unregister_devices;
	}

	pr_info("%s: Cypress TTSP FW Loader Driver (Built %s) rc=%d\n",
		 __func__, CY_DRIVER_DATE, rc);
	return 0;

fail_unregister_devices:
	for (i--; i <= 0; i--) {
		cyttsp5_unregister_device(cyttsp5_loader_infos[i].name,
			cyttsp5_loader_infos[i].core_id);
		pr_info("%s: Unregistering loader device for core_id: %s\n",
			__func__, cyttsp5_loader_infos[i].core_id);
	}
	return rc;
}
//module_init(cyttsp5_loader_init);
deferred_module_init_0(cyttsp5_loader_init);

static void __exit cyttsp5_loader_exit(void)
{
	int i;

	cyttsp5_unregister_driver(&cyttsp5_loader_driver);
	for (i = 0; i < num_core_ids; i++) {
		cyttsp5_unregister_device(cyttsp5_loader_infos[i].name,
			cyttsp5_loader_infos[i].core_id);
		pr_info("%s: Unregistering loader device for core_id: %s\n",
			__func__, cyttsp5_loader_infos[i].core_id);
	}
}
module_exit(cyttsp5_loader_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Cypress TrueTouch(R) Standard Product FW Loader Driver");
MODULE_AUTHOR("Cypress Semiconductor <ttdrivers@cypress.com>");
