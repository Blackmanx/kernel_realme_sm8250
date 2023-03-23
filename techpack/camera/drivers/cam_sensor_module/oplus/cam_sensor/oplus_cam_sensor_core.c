// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2017-2020, The Linux Foundation. All rights reserved.
 * Copyright (c) 2020, Oplus. All rights reserved.
 */

#include <linux/module.h>
#include <cam_sensor_cmn_header.h>
#include "cam_sensor_util.h"
#include "cam_soc_util.h"
#include "cam_trace.h"
#include "cam_common_util.h"
#include "cam_packet_util.h"
#include "oplus_cam_sensor_core.h"
#include "cam_sensor_core.h"
#include "oplus_cam_sensor_dev.h"
#ifdef OPLUS_FEATURE_CAMERA_COMMON
#include <linux/firmware.h>
#include <linux/dma-contiguous.h>


struct cam_sensor_i2c_reg_setting_array {
	struct cam_sensor_i2c_reg_array reg_setting[4600];
	unsigned short size;
	enum camera_sensor_i2c_type addr_type;
	enum camera_sensor_i2c_type data_type;
	unsigned short delay;
};


struct cam_sensor_init_settings {
	struct cam_sensor_i2c_reg_setting_array imx689_setting;
	struct cam_sensor_i2c_reg_setting_array imx689_setting1;
	struct cam_sensor_i2c_reg_setting_array imx789_setting;
	struct cam_sensor_i2c_reg_setting_array imx789_settingMP;
	struct cam_sensor_i2c_reg_setting_array imx766_init_setting1;
	struct cam_sensor_i2c_reg_setting_array imx766_init_setting2;

};

struct cam_sensor_init_settings sensor_init_settings = {
#include "cam_sensor_initsettings.h"
};


#define MAX_LENGTH 128

static int RamWriteByte(struct camera_io_master *cci_master_info,
                        uint32_t addr, uint32_t data, unsigned short mdelay)
{
	int32_t rc = 0;
	int retry = 1;
	int i = 0;
	struct cam_sensor_i2c_reg_array i2c_write_setting = {
		.reg_addr = addr,
		.reg_data = data,
		.delay = mdelay,
		.data_mask = 0x00,
	};
	struct cam_sensor_i2c_reg_setting i2c_write = {
		.reg_setting = &i2c_write_setting,
		.size = 1,
		.addr_type = CAMERA_SENSOR_I2C_TYPE_WORD,
		.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE,
		.delay = mdelay,
	};
	if (cci_master_info == NULL) {
		CAM_ERR(CAM_SENSOR, "Invalid Args");
		return -EINVAL;
	}

	for( i = 0; i < retry; i++)
	{
		rc = camera_io_dev_write(cci_master_info, &i2c_write);
		if (rc < 0) {
			CAM_ERR(CAM_SENSOR, "write 0x%04x failed, retry:%d", addr, i + 1);
		} else {
			return rc;
		}
	}
	return rc;
}


static int RamWriteWord(struct camera_io_master *cci_master_info,
                        uint32_t addr, uint32_t data)
{
	int32_t rc = 0;
	int retry = 1;
	int i = 0;
	struct cam_sensor_i2c_reg_array i2c_write_setting = {
		.reg_addr = addr,
		.reg_data = data,
		.delay = 0x00,
		.data_mask = 0x00,
	};
	struct cam_sensor_i2c_reg_setting i2c_write = {
		.reg_setting = &i2c_write_setting,
		.size = 1,
		.addr_type = CAMERA_SENSOR_I2C_TYPE_WORD,
		.data_type = CAMERA_SENSOR_I2C_TYPE_WORD,
		.delay = 0x00,
	};
	if (addr == 0x8c) {
		i2c_write .addr_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
		i2c_write .data_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
	}
	if (cci_master_info == NULL) {
		CAM_ERR(CAM_SENSOR, "Invalid Args");
		return -EINVAL;
	}

	for( i = 0; i < retry; i++)
	{
		rc = camera_io_dev_write(cci_master_info, &i2c_write);
		if (rc < 0) {
			CAM_ERR(CAM_SENSOR, "write 0x%04x failed, retry:%d", addr, i + 1);
		} else {
			return rc;
		}
	}
	return rc;
}

#define SEM1815S_FW_VERSION_OFFSET           0x7FF4
#define SEM1815S_FW_UPDATE_VERSION           0x1240
#define SEM1815S_SEC_FW_UPDATE_VERSION       0x12d4
#define ABNORMAL_FW_UPDATE_VERSION           0x0000
#define SEM1815S_FW_UPDATE_NAME              "ois_sem1215s_19066_dvt.bin"

static int cam_sem1815s_ois_fw_download(struct cam_sensor_ctrl_t *o_ctrl)
{
	uint16_t                           total_bytes = 0;
	uint8_t                            *ptr = NULL;
	int32_t                            rc = 0, cnt;
	uint32_t                           fw_size, data;
	uint16_t                           check_sum = 0x0;
	const struct firmware              *fw = NULL;
	const char                         *fw_name_bin = SEM1815S_FW_UPDATE_NAME;

	struct device                      *dev = &(o_ctrl->pdev->dev);
	struct cam_sensor_i2c_reg_setting  i2c_reg_setting;
	struct page                        *page = NULL;
	int                                reg_double = 1;
	uint8_t                            fw_ver[4] = {0};
	uint32_t                           ois_fw_version = SEM1815S_FW_UPDATE_VERSION;
	uint16_t tmp_slave_addr = 0x00;
	uint16_t ois_prog_addr = 0x1100;
	int i = 0;


	struct cam_camera_slave_info *slave_info;

	if (!o_ctrl) {
		CAM_ERR(CAM_SENSOR, "Invalid Args");
		return -EINVAL;
	}

	CAM_INFO(CAM_SENSOR, "entry:%s ", __func__);

	slave_info = &(o_ctrl->sensordata->slave_info);
	if (!slave_info) {
		CAM_ERR(CAM_SENSOR, " failed: %pK",
		        slave_info);
		return -EINVAL;
	}

	/* Load FW first , if firmware not exist , return */
	rc = request_firmware(&fw, fw_name_bin, dev);
	if (rc) {
		CAM_ERR(CAM_SENSOR, "Failed to locate %s", fw_name_bin);
		o_ctrl->io_master_info.cci_client->sid = tmp_slave_addr;
		return 0;
	}

	tmp_slave_addr = o_ctrl->io_master_info.cci_client->sid;
	o_ctrl->io_master_info.cci_client->sid = (0x68 >> 1);

	rc = camera_io_dev_read(&(o_ctrl->io_master_info), 0x040b, &data,
	                        CAMERA_SENSOR_I2C_TYPE_WORD, CAMERA_SENSOR_I2C_TYPE_BYTE);

	if (rc < 0) {
		CAM_ERR(CAM_SENSOR, "read 0x040b fail");
		o_ctrl->io_master_info.cci_client->sid = tmp_slave_addr;
		return 0;
	}

	CAM_INFO(CAM_SENSOR, "read data 0x040b is 0x%x ", data);

	rc = camera_io_dev_read(&(o_ctrl->io_master_info), 0x1008, &data,
	                        CAMERA_SENSOR_I2C_TYPE_WORD, CAMERA_SENSOR_I2C_TYPE_WORD);
	if (rc < 0) {
		CAM_ERR(CAM_SENSOR, "read 0x1008 fail");
		o_ctrl->io_master_info.cci_client->sid = tmp_slave_addr;
		return 0;
	}
	data = ((data >> 8) & 0x0FF) | ((data & 0xFF) << 8);

	if (data == ois_fw_version || data == SEM1815S_SEC_FW_UPDATE_VERSION || data == ABNORMAL_FW_UPDATE_VERSION) {
		CAM_INFO(CAM_SENSOR, "fw version:0x%0x need to update ois_fw_version:0x%0x !!!", data, ois_fw_version);
	} else {
		CAM_INFO(CAM_SENSOR, "fw version:0x%0x ois_fw_version:0x%0x no need to update !!!", data, ois_fw_version);
		o_ctrl->io_master_info.cci_client->sid = tmp_slave_addr;
		return 0;
	}

	rc = camera_io_dev_read(&(o_ctrl->io_master_info), 0x1020, &data,
	                        CAMERA_SENSOR_I2C_TYPE_WORD, CAMERA_SENSOR_I2C_TYPE_WORD);
	if (rc < 0) {
		CAM_INFO(CAM_SENSOR, "read 0x1020 fail");
		o_ctrl->io_master_info.cci_client->sid = tmp_slave_addr;
		return 0;
	}

	CAM_INFO(CAM_SENSOR, "read data 0x1020 is 0x%x ", data);

	rc = camera_io_dev_read(&(o_ctrl->io_master_info), 0x0001, &data,
	                        CAMERA_SENSOR_I2C_TYPE_WORD, CAMERA_SENSOR_I2C_TYPE_BYTE);
	if (rc < 0) {
		CAM_INFO(CAM_SENSOR, "read 0x0001 fail");
	}
	if (data != 0x01) {
		RamWriteByte(&(o_ctrl->io_master_info), 0x0000, 0x0, 50);
	}

	rc = camera_io_dev_read(&(o_ctrl->io_master_info), 0x0201, &data,
	                        CAMERA_SENSOR_I2C_TYPE_WORD, CAMERA_SENSOR_I2C_TYPE_BYTE);
	if (rc < 0) {
		CAM_ERR(CAM_SENSOR, "read 0x0201 fail");
	}
	if (data != 0x01) {
		RamWriteByte(&(o_ctrl->io_master_info), 0x0200, 0x0, 10);
		rc = camera_io_dev_read(&(o_ctrl->io_master_info), 0x0201, &data,
		                        CAMERA_SENSOR_I2C_TYPE_WORD, CAMERA_SENSOR_I2C_TYPE_BYTE);
		if (rc < 0) {
			CAM_ERR(CAM_SENSOR, "read 0x0201 fail");
		}
	}

	RamWriteByte(&(o_ctrl->io_master_info), 0x1000, 0x05, 60);

	rc = camera_io_dev_read(&(o_ctrl->io_master_info), 0x0001, &data,
	                        CAMERA_SENSOR_I2C_TYPE_WORD, CAMERA_SENSOR_I2C_TYPE_BYTE);
	if (rc < 0) {
		CAM_ERR(CAM_SENSOR, "read 0x0001 fail");
	}
	if (data != 0x02) {
		o_ctrl->io_master_info.cci_client->sid = tmp_slave_addr;
		return 0;
	}

	total_bytes = fw->size;
	i2c_reg_setting.addr_type = CAMERA_SENSOR_I2C_TYPE_WORD;
	i2c_reg_setting.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
	i2c_reg_setting.size = total_bytes;
	i2c_reg_setting.delay = 0;
	fw_size = PAGE_ALIGN(sizeof(struct cam_sensor_i2c_reg_array) *
	                     total_bytes) >> PAGE_SHIFT;
	page = cma_alloc(dev_get_cma_area((o_ctrl->soc_info.dev)),
	                 fw_size, 0, GFP_KERNEL);
	if (!page) {
		CAM_ERR(CAM_SENSOR, "Failed in allocating i2c_array");
		release_firmware(fw);
		o_ctrl->io_master_info.cci_client->sid = tmp_slave_addr;
		return -ENOMEM;
	}

	i2c_reg_setting.reg_setting = (struct cam_sensor_i2c_reg_array *) (
	                                  page_address(page));

	CAM_INFO(CAM_SENSOR, "total_bytes:%d", total_bytes);

	for (cnt = 0, ptr = (uint8_t *)fw->data; cnt < total_bytes;) {
		i2c_reg_setting.size = 0;
		for (i = 0; (i < MAX_LENGTH && cnt < total_bytes); i++, ptr++) {
			if (cnt >= SEM1815S_FW_VERSION_OFFSET && cnt < (SEM1815S_FW_VERSION_OFFSET + 4)) {
				fw_ver[cnt - SEM1815S_FW_VERSION_OFFSET] = *ptr;
				CAM_ERR(CAM_SENSOR, "get fw version:0x%0x", fw_ver[cnt - SEM1815S_FW_VERSION_OFFSET]);
			}
			i2c_reg_setting.reg_setting[i].reg_addr = ois_prog_addr;
			i2c_reg_setting.reg_setting[i].reg_data = *ptr;
			i2c_reg_setting.reg_setting[i].delay = 0;
			i2c_reg_setting.reg_setting[i].data_mask = 0;
			i2c_reg_setting.size++;
			cnt++;
			if (reg_double == 0) {
				reg_double = 1;
			} else {
				check_sum += ((*(ptr + 1) << 8) | *ptr) & 0xFFFF;
				reg_double = 0;
			}
		}
		i2c_reg_setting.delay = 0;

		if (i2c_reg_setting.size > 0) {
			rc = camera_io_dev_write_continuous(&(o_ctrl->io_master_info),
			                                    &i2c_reg_setting, 1);
			msleep(1);
		}
	}
	if (rc < 0) {
		CAM_ERR(CAM_SENSOR, "OIS FW download failed %d", rc);
		goto release_firmware;
	}
	CAM_INFO(CAM_SENSOR, "check sum:0x%0x", check_sum);

	RamWriteWord(&(o_ctrl->io_master_info), 0x1002, ((check_sum & 0x0FF) << 8) | ((check_sum & 0xFF00) >> 8));
	msleep(10);

	rc = camera_io_dev_read(&(o_ctrl->io_master_info), 0x1001, &data,
	                        CAMERA_SENSOR_I2C_TYPE_WORD, CAMERA_SENSOR_I2C_TYPE_BYTE);
	if (rc < 0) {
		CAM_ERR(CAM_SENSOR, "read 0x1001 fail");
	} else {
		CAM_INFO(CAM_SENSOR, "get 0x1001 = 0x%0x", data);
	}

	RamWriteByte(&(o_ctrl->io_master_info), 0x1000, 0x80, 200);
	if (rc < 0) {
		CAM_ERR(CAM_SENSOR, "write 0x1000 fail");
	}

	rc = camera_io_dev_read(&(o_ctrl->io_master_info), 0x1008, &data,
	                        CAMERA_SENSOR_I2C_TYPE_WORD, CAMERA_SENSOR_I2C_TYPE_WORD);
	if (rc < 0) {
		CAM_ERR(CAM_SENSOR, "read 0x1008 fail");
	}
	CAM_INFO(CAM_SENSOR, "get 0x1008 = 0x%0x", data);

	rc = camera_io_dev_read(&(o_ctrl->io_master_info), 0x100A, &data,
	                        CAMERA_SENSOR_I2C_TYPE_WORD, CAMERA_SENSOR_I2C_TYPE_WORD);
	if (rc < 0) {
		CAM_ERR(CAM_SENSOR, "read 0x100A fail");
	}
	CAM_INFO(CAM_SENSOR, "get 0x100A = 0x%0x", data);

release_firmware:
	o_ctrl->io_master_info.cci_client->sid = tmp_slave_addr;
	cma_release(dev_get_cma_area((o_ctrl->soc_info.dev)),
	            page, fw_size);
	release_firmware(fw);

	return rc;
}
#endif
#ifdef OPLUS_FEATURE_CAMERA_COMMON
#define FD_DFCT_NUM_ADDR 0x7678
#define SG_DFCT_NUM_ADDR 0x767A
#define FD_DFCT_ADDR 0x8B00
#define SG_DFCT_ADDR 0x8B10

#define V_ADDR_SHIFT 12
#define H_DATA_MASK 0xFFF80000
#define V_DATA_MASK 0x0007FF80

struct sony_dfct_tbl_t imx471_dfct_tbl;

static int sensor_imx471_get_dpc_data(struct cam_sensor_ctrl_t *s_ctrl)
{
	int i = 0, j = 0;
	int rc = 0;
	int check_reg_val, dfct_data_h, dfct_data_l;
	int dfct_data = 0;
	int fd_dfct_num = 0, sg_dfct_num = 0;
	int retry_cnt = 5;
	int data_h = 0, data_v = 0;
	int fd_dfct_addr = FD_DFCT_ADDR;
	int sg_dfct_addr = SG_DFCT_ADDR;

	CAM_INFO(CAM_SENSOR, "sensor_imx471_get_dpc_data enter");
	if (s_ctrl == NULL) {
		CAM_ERR(CAM_SENSOR, "Invalid Args");
		return -EINVAL;
	}

	memset(&imx471_dfct_tbl, 0, sizeof(struct sony_dfct_tbl_t));

	for (i = 0; i < retry_cnt; i++) {
		check_reg_val = 0;
		rc = camera_io_dev_read(&(s_ctrl->io_master_info),
		                        FD_DFCT_NUM_ADDR, &check_reg_val,
		                        CAMERA_SENSOR_I2C_TYPE_WORD,
		                        CAMERA_SENSOR_I2C_TYPE_BYTE);

		if (0 == rc) {
			fd_dfct_num = check_reg_val & 0x07;
			if (fd_dfct_num > FD_DFCT_MAX_NUM)
				fd_dfct_num = FD_DFCT_MAX_NUM;
			break;
		}
	}

	for (i = 0; i < retry_cnt; i++) {
		check_reg_val = 0;
		rc = camera_io_dev_read(&(s_ctrl->io_master_info),
		                        SG_DFCT_NUM_ADDR, &check_reg_val,
		                        CAMERA_SENSOR_I2C_TYPE_WORD,
		                        CAMERA_SENSOR_I2C_TYPE_WORD);

		if (0 == rc) {
			sg_dfct_num = check_reg_val & 0x01FF;
			if (sg_dfct_num > SG_DFCT_MAX_NUM)
				sg_dfct_num = SG_DFCT_MAX_NUM;
			break;
		}
	}

	CAM_INFO(CAM_SENSOR, " fd_dfct_num = %d, sg_dfct_num = %d", fd_dfct_num, sg_dfct_num);
	imx471_dfct_tbl.fd_dfct_num = fd_dfct_num;
	imx471_dfct_tbl.sg_dfct_num = sg_dfct_num;

	if (fd_dfct_num > 0) {
		for (j = 0; j < fd_dfct_num; j++) {
			dfct_data = 0;
			for (i = 0; i < retry_cnt; i++) {
				dfct_data_h = 0;
				rc = camera_io_dev_read(&(s_ctrl->io_master_info),
				                        fd_dfct_addr, &dfct_data_h,
				                        CAMERA_SENSOR_I2C_TYPE_WORD,
				                        CAMERA_SENSOR_I2C_TYPE_WORD);
				if (0 == rc) {
					break;
				}
			}
			for (i = 0; i < retry_cnt; i++) {
				dfct_data_l = 0;
				rc = camera_io_dev_read(&(s_ctrl->io_master_info),
				                        fd_dfct_addr + 2, &dfct_data_l,
				                        CAMERA_SENSOR_I2C_TYPE_WORD,
				                        CAMERA_SENSOR_I2C_TYPE_WORD);
				if (0 == rc) {
					break;
				}
			}
			CAM_DBG(CAM_SENSOR, " dfct_data_h = 0x%x, dfct_data_l = 0x%x", dfct_data_h, dfct_data_l);
			dfct_data = (dfct_data_h << 16) | dfct_data_l;
			data_h = 0;
			data_v = 0;
			data_h = (dfct_data & (H_DATA_MASK >> j % 8)) >> (19 - j % 8); //19 = 32 -13;
			data_v = (dfct_data & (V_DATA_MASK >> j % 8)) >> (7 - j % 8); // 7 = 32 -13 -12;
			CAM_DBG(CAM_SENSOR, "j = %d, H = %d, V = %d", j, data_h, data_v);
			imx471_dfct_tbl.fd_dfct_addr[j] = ((data_h & 0x1FFF) << V_ADDR_SHIFT) | (data_v & 0x0FFF);
			CAM_DBG(CAM_SENSOR, "fd_dfct_data[%d] = 0x%08x", j, imx471_dfct_tbl.fd_dfct_addr[j]);
			fd_dfct_addr = fd_dfct_addr + 3 + ((j + 1) % 8 == 0);
		}
	}
	if (sg_dfct_num > 0) {
		for (j = 0; j < sg_dfct_num; j++) {
			dfct_data = 0;
			for (i = 0; i < retry_cnt; i++) {
				dfct_data_h = 0;
				rc = camera_io_dev_read(&(s_ctrl->io_master_info),
				                        sg_dfct_addr, &dfct_data_h,
				                        CAMERA_SENSOR_I2C_TYPE_WORD,
				                        CAMERA_SENSOR_I2C_TYPE_WORD);
				if (0 == rc) {
					break;
				}
			}
			for (i = 0; i < retry_cnt; i++) {
				dfct_data_l = 0;
				rc = camera_io_dev_read(&(s_ctrl->io_master_info),
				                        sg_dfct_addr + 2, &dfct_data_l,
				                        CAMERA_SENSOR_I2C_TYPE_WORD,
				                        CAMERA_SENSOR_I2C_TYPE_WORD);
				if (0 == rc) {
					break;
				}
			}
			CAM_DBG(CAM_SENSOR, " dfct_data_h = 0x%x, dfct_data_l = 0x%x", dfct_data_h, dfct_data_l);
			dfct_data = (dfct_data_h << 16) | dfct_data_l;
			data_h = 0;
			data_v = 0;
			data_h = (dfct_data & (H_DATA_MASK >> j % 8)) >> (19 - j % 8); //19 = 32 -13;
			data_v = (dfct_data & (V_DATA_MASK >> j % 8)) >> (7 - j % 8); // 7 = 32 -13 -12;
			CAM_DBG(CAM_SENSOR, "j = %d, H = %d, V = %d", j, data_h, data_v);
			imx471_dfct_tbl.sg_dfct_addr[j] = ((data_h & 0x1FFF) << V_ADDR_SHIFT) | (data_v & 0x0FFF);
			CAM_DBG(CAM_SENSOR, "sg_dfct_data[%d] = 0x%08x", j, imx471_dfct_tbl.sg_dfct_addr[j]);
			sg_dfct_addr = sg_dfct_addr + 3 + ((j + 1) % 8 == 0);
		}
	}

	CAM_INFO(CAM_SENSOR, "exit");
	return rc;
}
#endif

#ifdef OPLUS_FEATURE_CAMERA_COMMON
#define LASER_ENABLE_PATH  "/sys/class/input/input6/enable_ps_sensor"
#define LASER_CROSSTALK_ENABLE "/sys/class/input/input6/crosstalk_enable"
#define LASER_CORRECTION_MODE  "/sys/class/input/input6/smudge_correction_mode"

static int writefileData(char *filename, int data)
{
	struct file *mfile = NULL;
	ssize_t size = 0;
	loff_t offsize = 0;
	mm_segment_t old_fs;
	char fdata[2] = {0};

	memset(fdata, 0, sizeof(fdata));
	mfile = filp_open(filename, O_RDWR, 0660);
	if (IS_ERR(mfile)) {
		CAM_ERR(CAM_SENSOR, "fopen file %s failed !", filename);
		return (-1);
	}
	else {
		CAM_INFO(CAM_SENSOR, "fopen file %s succeeded", filename);
	}
	snprintf(fdata, sizeof(fdata), "%d", data);
	old_fs = get_fs();
	set_fs(KERNEL_DS);
	offsize = 0;
	size = vfs_write(mfile, fdata, sizeof(fdata), &offsize);
	if (size < 0) {
		CAM_ERR(CAM_SENSOR, "write file:%s data:%d error size:%d", filename, data, size);
		set_fs(old_fs);
		filp_close(mfile, NULL);
		return (-1);
	}
	else {
		CAM_INFO(CAM_SENSOR, "write file:%s data:%d correct size:%d", filename, data, size);
	}
	set_fs(old_fs);
	filp_close(mfile, NULL);

	return 0;
}
#endif

#ifdef OPLUS_FEATURE_CAMERA_COMMON
#define SONY_SENSOR_MP1 (0x02)  //imx708 cut1.0

#define SONY_SENSOR_CUT0_9 (0x00)  //imx766 cut0.9
#define SONY_SENSOR_CUT0_9_1 (0x01)  //imx766 cut0.91
#define SONY_SENSOR_CUT1_0 (0x09)  //imx766 cut1.0
#define SONY_SENSOR_CUTMP (0x10)  //imx766 MP
#endif

uint32_t oplus_cam_sensor_addr_is_byte_type(struct cam_sensor_ctrl_t *s_ctrl, struct cam_camera_slave_info *slave_info)
{
	int rc = 0;
	uint32_t chipid = 0;

	uint32_t gc02m0_high = 0;
	uint32_t gc02m0_low = 0;
	uint32_t chipid_high = 0;
	uint32_t chipid_low = 0;
	uint32_t sensor_version = 0;
	uint16_t sensor_version_reg = 0x0018;
	const uint32_t IMX708_0_9_SOURCE_CHIPID = 0xF708;
	const uint32_t IMX708_1_0_SOURCE_CHIPID = 0xE708;
	struct cam_sensor_cci_client ee_cci_client;
	uint32_t ee_vcmid = 0;
	const uint8_t IMX766_EEPROM_SID = (0xA2 >> 1);
	const uint8_t IMX766_EEPROM_VCMID_ADDR = 0x0A;
	const uint8_t IMX766_SECOND_SOURCE_VCMID = 0xE9;
	const uint32_t IMX766_FIRST_SOURCE_CHIPID = 0x766F;
	const uint32_t IMX766_SECOND_SOURCE_CHIPID = 0x766E;

	const uint8_t OV64B_EEPROM_SID = (0xA0 >> 1);
	const uint8_t OV64B_EEPROM_VCMID_ADDR = 0x0A;
	const uint8_t OV64B_EEPROM_VCMID_VCMID = 0xEA;
	const uint32_t OV64B_FIRST_CHIPID = 0x6B64;
	const uint32_t OV64B_SECOND_CHIPID = 0x5A64;

	if (slave_info->sensor_id == 0x02d0 || slave_info->sensor_id == 0x25 || slave_info->sensor_id == 0x02e0 ||
	    slave_info->sensor_id == 0x5035) {
		gc02m0_high = slave_info->sensor_id_reg_addr & 0xff00;
		gc02m0_high = gc02m0_high >> 8;
		gc02m0_low = slave_info->sensor_id_reg_addr & 0x00ff;
		rc = camera_io_dev_read(
		         &(s_ctrl->io_master_info),
		         gc02m0_high,
		         &chipid_high, CAMERA_SENSOR_I2C_TYPE_BYTE,
		         CAMERA_SENSOR_I2C_TYPE_BYTE);

		CAM_ERR(CAM_SENSOR, "gc02m0_high: 0x%x chipid_high id 0x%x:",
		        gc02m0_high, chipid_high);

		rc = camera_io_dev_read(
		         &(s_ctrl->io_master_info),
		         gc02m0_low,
		         &chipid_low, CAMERA_SENSOR_I2C_TYPE_BYTE,
		         CAMERA_SENSOR_I2C_TYPE_BYTE);

		CAM_ERR(CAM_SENSOR, "gc02m0_low: 0x%x chipid_low id 0x%x:",
		        gc02m0_low, chipid_low);

		chipid = ((chipid_high << 8) & 0xff00) | (chipid_low & 0x00ff);
	}
	CAM_WARN(CAM_SENSOR, "slave_info->sensor_id is %x", slave_info->sensor_id);
	if (slave_info->sensor_id == IMX708_0_9_SOURCE_CHIPID || slave_info->sensor_id == IMX708_1_0_SOURCE_CHIPID) {
		rc = camera_io_dev_read(
		         &(s_ctrl->io_master_info),
		         sensor_version_reg,
		         &sensor_version, CAMERA_SENSOR_I2C_TYPE_WORD,
		         CAMERA_SENSOR_I2C_TYPE_WORD);
		CAM_WARN(CAM_SENSOR, "imx708 sensor_version: 0x%x",
		         sensor_version >> 8);
		if ((sensor_version >> 8) >= SONY_SENSOR_MP1) {
			chipid = IMX708_1_0_SOURCE_CHIPID;
			CAM_WARN(CAM_SENSOR, "set chipid : 0x%x",
			         IMX708_1_0_SOURCE_CHIPID);
			if ((sensor_version >> 8) > SONY_SENSOR_MP1) {
				s_ctrl->sensordata->slave_info.sensor_version = 1;
			} else {
				s_ctrl->sensordata->slave_info.sensor_version = 0;
			}
			CAM_WARN(CAM_SENSOR, "imx708 slave_info.sensor_version: %d:",
			         s_ctrl->sensordata->slave_info.sensor_version );
		} else {
			chipid = IMX708_0_9_SOURCE_CHIPID;
			CAM_WARN(CAM_SENSOR, "set chipid : 0x%x",
			         IMX708_0_9_SOURCE_CHIPID);
		}
	}

	if (slave_info->sensor_id == 0x0689) {
		rc = camera_io_dev_read(
		         &(s_ctrl->io_master_info),
		         sensor_version_reg,
		         &sensor_version, CAMERA_SENSOR_I2C_TYPE_WORD,
		         CAMERA_SENSOR_I2C_TYPE_WORD);
		CAM_WARN(CAM_SENSOR, "imx689 sensor_version: 0x%x",
		         sensor_version >> 8);
		if ((sensor_version >> 8) > SONY_SENSOR_MP1) {
			s_ctrl->sensordata->slave_info.sensor_version = 1;
			CAM_WARN(CAM_SENSOR, "imx689 slave_info.sensor_version: %d:",
			         s_ctrl->sensordata->slave_info.sensor_version );
		} else {
			s_ctrl->sensordata->slave_info.sensor_version = 0;
			CAM_WARN(CAM_SENSOR, "imx689 slave_info.sensor_version: %d:",
			         s_ctrl->sensordata->slave_info.sensor_version );
		}
		chipid = 0x689;
	}
	if (slave_info->sensor_id == 0x0766 ||
		slave_info->sensor_id == IMX766_FIRST_SOURCE_CHIPID ||
	    slave_info->sensor_id == IMX766_SECOND_SOURCE_CHIPID) {
		rc = camera_io_dev_read(
			&(s_ctrl->io_master_info),
			sensor_version_reg,
			&sensor_version, CAMERA_SENSOR_I2C_TYPE_WORD,
			CAMERA_SENSOR_I2C_TYPE_WORD);


		CAM_WARN(CAM_SENSOR, "imx766 sensor_version: 0x%x",
			sensor_version);
		if ((sensor_version >> 8) ==  SONY_SENSOR_CUT0_9) {
			s_ctrl->sensordata->slave_info.sensor_version = 0;
			CAM_WARN(CAM_SENSOR, "imx766 slave_info.sensor_version: %d:",
				s_ctrl->sensordata->slave_info.sensor_version );
		} else if ((sensor_version >> 8) ==  SONY_SENSOR_CUT0_9_1){
			s_ctrl->sensordata->slave_info.sensor_version = 1;
			CAM_WARN(CAM_SENSOR, "imx766 slave_info.sensor_version: %d:",
				s_ctrl->sensordata->slave_info.sensor_version );
		} else if ((sensor_version >> 8) ==  SONY_SENSOR_CUT1_0) {
			s_ctrl->sensordata->slave_info.sensor_version = 2;
			CAM_WARN(CAM_SENSOR, "imx766 slave_info.sensor_version: %d:",
				s_ctrl->sensordata->slave_info.sensor_version );
		} else if ((sensor_version >> 8) >=  SONY_SENSOR_CUTMP) {
			s_ctrl->sensordata->slave_info.sensor_version = 3;
			CAM_WARN(CAM_SENSOR, "imx766 slave_info.sensor_version: %d:",
				s_ctrl->sensordata->slave_info.sensor_version );
		}

		rc = camera_io_dev_read(&(s_ctrl->io_master_info),
		       slave_info->sensor_id_reg_addr,
		       &chipid,slave_info->addr_type,
		       CAMERA_SENSOR_I2C_TYPE_WORD);
	}
	if (slave_info->sensor_id == IMX766_FIRST_SOURCE_CHIPID ||
		  slave_info->sensor_id == IMX766_SECOND_SOURCE_CHIPID) {
		memcpy(&ee_cci_client, s_ctrl->io_master_info.cci_client,
						sizeof(struct cam_sensor_cci_client));
		ee_cci_client.sid = IMX766_EEPROM_SID;
		rc = cam_cci_i2c_read(&ee_cci_client,
			 IMX766_EEPROM_VCMID_ADDR,
			 &ee_vcmid, CAMERA_SENSOR_I2C_TYPE_WORD,
			 CAMERA_SENSOR_I2C_TYPE_BYTE);

		CAM_WARN(CAM_SENSOR, "distinguish imx766 camera module, vcm id : 0x%x ", ee_vcmid);
        if (IMX766_SECOND_SOURCE_VCMID == ee_vcmid) {
			chipid = IMX766_SECOND_SOURCE_CHIPID;
		} else {
			chipid = IMX766_FIRST_SOURCE_CHIPID;
		}
	}

	if (slave_info->sensor_id == 0x2b03) {
		rc = camera_io_dev_read(
		         &(s_ctrl->io_master_info),
		         slave_info->sensor_id_reg_addr,
		         &chipid, CAMERA_SENSOR_I2C_TYPE_WORD,
		         CAMERA_SENSOR_I2C_TYPE_WORD);

		CAM_WARN(CAM_SENSOR, "ov02b chipid: 0x%x",
		         chipid);
	}

	if ((slave_info->sensor_id == OV64B_FIRST_CHIPID) || (slave_info->sensor_id == OV64B_SECOND_CHIPID)) {
		memset(&ee_cci_client, 0x0, sizeof(struct cam_sensor_cci_client));
		memcpy(&ee_cci_client, s_ctrl->io_master_info.cci_client,
							sizeof(struct cam_sensor_cci_client));
		ee_cci_client.sid = OV64B_EEPROM_SID;
		rc = cam_cci_i2c_read(&ee_cci_client,
					OV64B_EEPROM_VCMID_ADDR,
					&ee_vcmid, CAMERA_SENSOR_I2C_TYPE_WORD,
					CAMERA_SENSOR_I2C_TYPE_BYTE);
		CAM_WARN(CAM_SENSOR, "OV64B read eeprom vcm id is 0x%x:", ee_vcmid);
		if (OV64B_EEPROM_VCMID_VCMID == ee_vcmid) {
			chipid = OV64B_SECOND_CHIPID;
		} else {
			chipid = OV64B_FIRST_CHIPID;
		}
	}

	return chipid;
}

void oplus_cam_sensor_for_laser_ois(struct cam_sensor_ctrl_t *s_ctrl, struct cam_camera_slave_info *slave_info)
{
	if (s_ctrl->laser_support) {
		writefileData(LASER_ENABLE_PATH, 0);
		writefileData(LASER_CROSSTALK_ENABLE, 1);
		writefileData(LASER_CORRECTION_MODE, 1);
	}
	if (s_ctrl->sem1815s_ois_support) {
		cam_sem1815s_ois_fw_download(s_ctrl);
	}
	if (slave_info->sensor_id == 0x0471) {
		sensor_imx471_get_dpc_data(s_ctrl);
	}
}

int32_t oplus_cam_sensor_driver_cmd(struct cam_sensor_ctrl_t *s_ctrl,
                                    void *arg)
{
	int rc = 0;
	struct cam_control *cmd = (struct cam_control *)arg;
#ifdef OPLUS_FEATURE_CAMERA_COMMON
	CAM_ERR(CAM_SENSOR, "linnta oplus_cam_sensor_driver_cmd");
	switch (cmd->op_code) {
	case CAM_OEM_IO_CMD: {
		struct cam_oem_rw_ctl oem_ctl;
		struct camera_io_master oem_io_master_info;
		struct cam_sensor_cci_client oem_cci_client;
		struct cam_oem_i2c_reg_array *cam_regs = NULL;
		if (copy_from_user(&oem_ctl, (void __user *)cmd->handle,
		                   sizeof(struct cam_oem_rw_ctl))) {
			CAM_ERR(CAM_SENSOR,
			        "Fail in copy oem control infomation form user data");
			rc = -ENOMEM;
			return rc;
		}
		if (oem_ctl.num_bytes > 0) {
			cam_regs = (struct cam_oem_i2c_reg_array *)kzalloc(
			               sizeof(struct cam_oem_i2c_reg_array) * oem_ctl.num_bytes, GFP_KERNEL);
			if (!cam_regs) {
				rc = -ENOMEM;
				CAM_ERR(CAM_SENSOR, "failed alloc cam_regs");
				return rc;
			}

			if (copy_from_user(cam_regs, u64_to_user_ptr(oem_ctl.cam_regs_ptr),
			                   sizeof(struct cam_oem_i2c_reg_array)*oem_ctl.num_bytes)) {
				CAM_INFO(CAM_SENSOR, "copy_from_user error!!!", oem_ctl.num_bytes);
				rc = -EFAULT;
				goto free_cam_regs;
			}
		}
		memcpy(&oem_io_master_info, &(s_ctrl->io_master_info), sizeof(struct camera_io_master));
		memcpy(&oem_cci_client, s_ctrl->io_master_info.cci_client, sizeof(struct cam_sensor_cci_client));
		oem_io_master_info.cci_client = &oem_cci_client;
		if (oem_ctl.slave_addr != 0) {
			oem_io_master_info.cci_client->sid = (oem_ctl.slave_addr >> 1);
		}

		switch (oem_ctl.cmd_code) {
		case CAM_OEM_CMD_READ_DEV: {
			int i = 0;
			for (; i < oem_ctl.num_bytes; i++)
			{
				rc |= cam_cci_i2c_read(
				          oem_io_master_info.cci_client,
				          cam_regs[i].reg_addr,
				          &(cam_regs[i].reg_data),
				          oem_ctl.reg_addr_type,
				          oem_ctl.reg_data_type);
				CAM_INFO(CAM_SENSOR,
				         "read addr:0x%x  Data:0x%x ",
				         cam_regs[i].reg_addr, cam_regs[i].reg_data);
			}

			if (rc < 0) {
				CAM_ERR(CAM_SENSOR,
				        "Fail oem ctl data ,slave sensor id is 0x%x", s_ctrl->sensordata->slave_info.sensor_id);
				goto free_cam_regs;;
			}

			if (copy_to_user(u64_to_user_ptr(oem_ctl.cam_regs_ptr), cam_regs,
			                 sizeof(struct cam_oem_i2c_reg_array)*oem_ctl.num_bytes)) {
				CAM_ERR(CAM_SENSOR,
				        "Fail oem ctl data ,slave sensor id is 0x%x", s_ctrl->sensordata->slave_info.sensor_id);
				goto free_cam_regs;

			}
			break;
		}
		case CAM_OEM_CMD_WRITE_DEV: {
			struct cam_sensor_i2c_reg_setting write_setting;
			int i = 0;
			for (;i < oem_ctl.num_bytes; i++)
			{
				CAM_DBG(CAM_SENSOR,"Get from OEM addr: 0x%x data: 0x%x ",
								cam_regs[i].reg_addr, cam_regs[i].reg_data);
			}

			write_setting.addr_type = oem_ctl.reg_addr_type;
			write_setting.data_type = oem_ctl.reg_data_type;
			write_setting.size = oem_ctl.num_bytes;
			write_setting.reg_setting = (struct cam_sensor_i2c_reg_array *)cam_regs;

			rc = cam_cci_i2c_write_table(&oem_io_master_info, &write_setting);

			if (rc < 0) {
				CAM_ERR(CAM_SENSOR,
				        "Fail oem write data ,slave sensor id is 0x%x", s_ctrl->sensordata->slave_info.sensor_id);
				goto free_cam_regs;
			}

			break;
		}

		case CAM_OEM_OIS_CALIB : {
			rc = cam_ois_sem1215s_calibration(&oem_io_master_info);
			CAM_ERR(CAM_SENSOR, "ois calib failed rc:%d", rc);
			break;
		}

		default:
			CAM_ERR(CAM_SENSOR,
			        "Unknow OEM cmd ,slave sensor id is 0x%x", s_ctrl->sensordata->slave_info.sensor_id);
			break ;
		}

free_cam_regs:
		if (cam_regs != NULL) {
			kfree(cam_regs);
			cam_regs = NULL;
		}
		mutex_unlock(&(s_ctrl->cam_sensor_mutex));
		CAM_ERR(CAM_SENSOR, "(rc error");
		return rc;
	}

	case CAM_OEM_GET_ID : {
		if (copy_to_user((void __user *)cmd->handle, &s_ctrl->soc_info.index,
		                 sizeof(uint32_t))) {
			CAM_ERR(CAM_SENSOR,
			        "copy camera id to user fail ");
		}
		break;
	}
	case CAM_GET_DPC_DATA: {
		if (0x0471 != s_ctrl->sensordata->slave_info.sensor_id) {
		    CAM_ERR(CAM_SENSOR, "(0x0471 != s_ctrl->sensordata->slave_info.sensor_id");
			rc = -EFAULT;
			return rc;
		}
		CAM_INFO(CAM_SENSOR, "imx471_dfct_tbl: fd_dfct_num=%d, sg_dfct_num=%d",
		         imx471_dfct_tbl.fd_dfct_num, imx471_dfct_tbl.sg_dfct_num);
		if (copy_to_user((void __user *) cmd->handle, &imx471_dfct_tbl,
		                 sizeof(struct  sony_dfct_tbl_t))) {
			CAM_ERR(CAM_SENSOR, "Failed Copy to User");
			rc = -EFAULT;
			return rc;
		}
	}
	break;
	default:
		CAM_ERR(CAM_SENSOR, "Invalid Opcode: %d", cmd->op_code);
		rc = -EINVAL;
		break;
	}
#endif
    if(rc < 0)
	CAM_ERR(CAM_SENSOR, "error rc < 0");

	return rc;
}
void oplus_cam_sensor_power_up(struct cam_sensor_ctrl_t *s_ctrl)
{
	/* Add by Fangyan @ Camera.Drv 2020/12/23 for different qsc version update only for imx766 */
	int rc;
	struct cam_sensor_cci_client ee_cci_client;
	uint32_t sensor_version = 0;
	uint32_t flag_addr      = 0;
	struct cam_camera_slave_info *slave_info;
	slave_info = &(s_ctrl->sensordata->slave_info);

	if (slave_info->sensor_id == 0x0766 ||
		slave_info->sensor_id == 0x0766E ||
		slave_info->sensor_id == 0x0766F) {
		memcpy(&ee_cci_client, s_ctrl->io_master_info.cci_client,
		       sizeof(struct cam_sensor_cci_client));
		if (slave_info->sensor_slave_addr == 0x20) {
			ee_cci_client.sid = 0xA0 >> 1;
			flag_addr = 0x2A32;
		} else if (slave_info->sensor_slave_addr == 0x34) {
			ee_cci_client.sid = 0xA2 >> 1;
			flag_addr = 0x2BD0;
		}
		rc = cam_cci_i2c_read(&ee_cci_client,
			 flag_addr,
			 &sensor_version, CAMERA_SENSOR_I2C_TYPE_WORD,
			 CAMERA_SENSOR_I2C_TYPE_BYTE);
		CAM_WARN(CAM_SENSOR, "QSC tool version is %x",
		         sensor_version);
		if (sensor_version == 0x03) {
			struct cam_sensor_i2c_reg_array qsc_tool = {
				.reg_addr = 0x86A9,
				.reg_data = 0x4E,
				.delay = 0x00,
				.data_mask = 0x00,
			};
			struct cam_sensor_i2c_reg_setting qsc_tool_write = {
				.reg_setting = &qsc_tool,
				.size = 1,
				.addr_type = CAMERA_SENSOR_I2C_TYPE_WORD,
				.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE,
				.delay = 0x00,
			};
			rc = camera_io_dev_write(&(s_ctrl->io_master_info), &qsc_tool_write);
			CAM_WARN(CAM_SENSOR, "update the qsc tool version %d", rc);
		}
	}
}

int cam_ois_sem1215s_calibration(struct camera_io_master *ois_master_info)
{
	int32_t                            rc = 0;
	uint32_t                           data;
	uint32_t                           calib_data = 0x0;
	int                                calib_ret = 0;
	//uint32_t                           gyro_offset = 0;
	int                                i = 0;
	if (!ois_master_info) {
		CAM_ERR(CAM_OIS, "Invalid Args");
		return -EINVAL;
	}

	rc = camera_io_dev_read(ois_master_info, 0x0001, &data,
	                        CAMERA_SENSOR_I2C_TYPE_WORD, CAMERA_SENSOR_I2C_TYPE_BYTE);
	if (rc < 0) {
		CAM_ERR(CAM_OIS, "read 0x0001 fail");
	}
	if (data != 0x01) {
		RamWriteByte(ois_master_info, 0x0000, 0x0, 50);
	}

	rc = camera_io_dev_read(ois_master_info, 0x0201, &data,
	                        CAMERA_SENSOR_I2C_TYPE_WORD, CAMERA_SENSOR_I2C_TYPE_BYTE);
	if (rc < 0) {
		CAM_ERR(CAM_OIS, "read 0x0201 fail");
	}
	if (data != 0x01) {
		RamWriteByte(ois_master_info, 0x0200, 0x0, 10);
	}

	RamWriteByte(ois_master_info, 0x0600, 0x1, 100);
	for (i = 0; i < 5; i++) {
		rc = camera_io_dev_read(ois_master_info, 0x0600, &data,
		                        CAMERA_SENSOR_I2C_TYPE_WORD, CAMERA_SENSOR_I2C_TYPE_BYTE);
		if (data == 0x00) {
			break;
		}

		if((data != 0x00) && (i >= 5))
		{
			CAM_ERR(CAM_OIS, "Gyro Offset Cal FAIL ");

		}
	}

	//cam_ois_read_gyrodata(ois_master_info, 0x0604, 0x0606, &gyro_offset, 1);

	rc = camera_io_dev_read(ois_master_info, 0x0004, &calib_data,
	                        CAMERA_SENSOR_I2C_TYPE_WORD, CAMERA_SENSOR_I2C_TYPE_WORD);
	if ((calib_data & 0x0100) == 0x0100) {
		CAM_ERR(CAM_OIS, "Gyro X Axis Offset Cal ERROR ");
		calib_ret = (0x1 << 16);
	}
	if ((calib_data & 0x0200) == 0x0200) {
		CAM_ERR(CAM_OIS, "Gyro Y Axis Offset Cal ERROR ");
		calib_ret |= (0x1);
	}

	if ((calib_data & (0x0100 | 0x0200)) == 0x0000)
	{
		RamWriteByte(ois_master_info, 0x300, 0x1, 100);
		for (i = 0; i < 5; i++) {
			rc = camera_io_dev_read(ois_master_info, 0x0300, &data,
			                        CAMERA_SENSOR_I2C_TYPE_WORD, CAMERA_SENSOR_I2C_TYPE_BYTE);
			if (data == 0x00) {
				break;
			} else if((data != 0x00) && (i >= 5)) {
				CAM_ERR(CAM_OIS, "Flash Save FAIL ");
			}
		}

		rc = camera_io_dev_read(ois_master_info, 0x0004, &calib_data,
			CAMERA_SENSOR_I2C_TYPE_WORD, CAMERA_SENSOR_I2C_TYPE_WORD);
		if ((calib_data & 0x0040) != 0x00)
		{
			CAM_ERR(CAM_OIS, "Gyro Offset Cal ERROR ");
			calib_ret = (0x1 << 15);
		}
	}

	return calib_ret;
}
struct cam_sensor_dpc_reg_setting_array {
	struct cam_sensor_i2c_reg_array reg_setting[25];
	unsigned short size;
	enum camera_sensor_i2c_type addr_type;
	enum camera_sensor_i2c_type data_type;
	unsigned short delay;
};

struct cam_sensor_dpc_reg_setting_array gc5035OTPWrite_setting[7] = {
#include "CAM_GC5035_SPC_SENSOR_SETTINGS.h"
};

uint32_t totalDpcNum = 0;
uint32_t totalDpcFlag = 0;
uint32_t gc5035_chipversion_buffer[26] = {0};

int sensor_gc5035_get_dpc_data(struct cam_sensor_ctrl_t *s_ctrl)
{
	int rc = 0;
	uint32_t gc5035_dpcinfo[3] = {0};
	uint32_t i;
	uint32_t dpcinfoOffet = 0xcd;
	uint32_t chipPage8Offet = 0xd0;
	uint32_t chipPage9Offet = 0xc0;

	struct cam_sensor_i2c_reg_setting sensor_setting;
	/*write otp read init settings*/
	sensor_setting.reg_setting = gc5035OTPWrite_setting[0].reg_setting;
	sensor_setting.addr_type = gc5035OTPWrite_setting[0].addr_type;
	sensor_setting.data_type = gc5035OTPWrite_setting[0].data_type;
	sensor_setting.size = gc5035OTPWrite_setting[0].size;
	sensor_setting.delay = gc5035OTPWrite_setting[0].delay;

	rc = camera_io_dev_write(&(s_ctrl->io_master_info), &sensor_setting);

	if (rc < 0) {
		CAM_ERR(CAM_SENSOR, "gc5035SpcWrite_setting Failed to write sensor setting");
		return rc;
	}
	/*write dpc page0 setting*/
	sensor_setting.reg_setting = gc5035OTPWrite_setting[1].reg_setting;
	sensor_setting.addr_type = gc5035OTPWrite_setting[1].addr_type;
	sensor_setting.data_type = gc5035OTPWrite_setting[1].data_type;
	sensor_setting.size = gc5035OTPWrite_setting[1].size;
	sensor_setting.delay = gc5035OTPWrite_setting[1].delay;

	rc = camera_io_dev_write(&(s_ctrl->io_master_info), &sensor_setting);

	if (rc < 0) {
		CAM_ERR(CAM_SENSOR, "gc5035SpcWrite_setting Failed to write sensor setting");
		return rc;
	}
	/*read dpc data*/
	for (i = 0; i < 3; i++) {
		rc = camera_io_dev_read(
		         &(s_ctrl->io_master_info),
		         dpcinfoOffet + i,
		         &gc5035_dpcinfo[i], CAMERA_SENSOR_I2C_TYPE_BYTE,
		         CAMERA_SENSOR_I2C_TYPE_BYTE);
		if (rc < 0) {
			CAM_ERR(CAM_SENSOR, "gc5035SpcWrite_setting Failed to read dpc info sensor setting");
			break;
		}
	}

	if (rc < 0)
		return rc;
	/*close read data*/
	sensor_setting.reg_setting = gc5035OTPWrite_setting[2].reg_setting;
	sensor_setting.addr_type = gc5035OTPWrite_setting[2].addr_type;
	sensor_setting.data_type = gc5035OTPWrite_setting[2].data_type;
	sensor_setting.size = gc5035OTPWrite_setting[2].size;
	sensor_setting.delay = gc5035OTPWrite_setting[2].delay;

	rc = camera_io_dev_write(&(s_ctrl->io_master_info), &sensor_setting);
	if (rc < 0) {
		CAM_ERR(CAM_SENSOR, "gc5035SpcWrite_setting Failed to write sensor setting");
		return rc;
	}

	if (gc5035_dpcinfo[0] == 1) {
		totalDpcFlag = 1;
		totalDpcNum = gc5035_dpcinfo[1] + gc5035_dpcinfo[2] ;
		CAM_INFO(CAM_SENSOR, "gc5035SpcWrite_setting gc5035_dpcinfo[1] = %d", gc5035_dpcinfo[1]);
		CAM_INFO(CAM_SENSOR, "gc5035SpcWrite_setting gc5035_dpcinfo[2] = %d", gc5035_dpcinfo[2]);
		CAM_INFO(CAM_SENSOR, "gc5035SpcWrite_setting totalDpcNum = %d", totalDpcNum);

	}
	//write for update reg for page 8
	sensor_setting.reg_setting = gc5035OTPWrite_setting[5].reg_setting;
	sensor_setting.addr_type = gc5035OTPWrite_setting[5].addr_type;
	sensor_setting.data_type = gc5035OTPWrite_setting[5].data_type;
	sensor_setting.size = gc5035OTPWrite_setting[5].size;
	sensor_setting.delay = gc5035OTPWrite_setting[5].delay;

	rc = camera_io_dev_write(&(s_ctrl->io_master_info), &sensor_setting);
	if (rc < 0) {
		CAM_ERR(CAM_SENSOR, "gc5035SpcWrite_setting Failed to write sensor setting");
		return rc;
	}
	for (i = 0; i < 0x10; i++) {
		rc = camera_io_dev_read(
		         &(s_ctrl->io_master_info),
		         chipPage8Offet + i,
		         &gc5035_chipversion_buffer[i], CAMERA_SENSOR_I2C_TYPE_BYTE,
		         CAMERA_SENSOR_I2C_TYPE_BYTE);
		if (rc < 0) {
			CAM_ERR(CAM_SENSOR, "gc5035SpcWrite_setting Failed to read dpc info sensor setting");
			break;
		}
	}
	/*close read data*/
	sensor_setting.reg_setting = gc5035OTPWrite_setting[2].reg_setting;
	sensor_setting.addr_type = gc5035OTPWrite_setting[2].addr_type;
	sensor_setting.data_type = gc5035OTPWrite_setting[2].data_type;
	sensor_setting.size = gc5035OTPWrite_setting[2].size;
	sensor_setting.delay = gc5035OTPWrite_setting[2].delay;

	rc = camera_io_dev_write(&(s_ctrl->io_master_info), &sensor_setting);
	if (rc < 0) {
		CAM_ERR(CAM_SENSOR, "gc5035SpcWrite_setting Failed to write sensor setting");
		return rc;
	}
	//write for update reg for page 9
	sensor_setting.reg_setting = gc5035OTPWrite_setting[6].reg_setting;
	sensor_setting.addr_type = gc5035OTPWrite_setting[6].addr_type;
	sensor_setting.data_type = gc5035OTPWrite_setting[6].data_type;
	sensor_setting.size = gc5035OTPWrite_setting[6].size;
	sensor_setting.delay = gc5035OTPWrite_setting[6].delay;

	rc = camera_io_dev_write(&(s_ctrl->io_master_info), &sensor_setting);
	if (rc < 0) {
		CAM_ERR(CAM_SENSOR, "gc5035SpcWrite_setting Failed to write sensor setting");
		return rc;
	}
	for (i = 0x00; i < 0x0a; i++) {
		rc = camera_io_dev_read(
		         &(s_ctrl->io_master_info),
		         chipPage9Offet + i,
		         &gc5035_chipversion_buffer[0x10 + i], CAMERA_SENSOR_I2C_TYPE_BYTE,
		         CAMERA_SENSOR_I2C_TYPE_BYTE);
		if (rc < 0) {
			CAM_ERR(CAM_SENSOR, "gc5035SpcWrite_setting Failed to read dpc info sensor setting");
			break;
		}
	}
	/*close read data*/
	sensor_setting.reg_setting = gc5035OTPWrite_setting[2].reg_setting;
	sensor_setting.addr_type = gc5035OTPWrite_setting[2].addr_type;
	sensor_setting.data_type = gc5035OTPWrite_setting[2].data_type;
	sensor_setting.size = gc5035OTPWrite_setting[2].size;
	sensor_setting.delay = gc5035OTPWrite_setting[2].delay;

	rc = camera_io_dev_write(&(s_ctrl->io_master_info), &sensor_setting);
	if (rc < 0) {
		CAM_ERR(CAM_SENSOR, "gc5035SpcWrite_setting Failed to write sensor setting");
		return rc;
	}
	return rc;

}

int sensor_gc5035_write_dpc_data(struct cam_sensor_ctrl_t *s_ctrl)
{
	int rc = 0;
	struct cam_sensor_i2c_reg_array gc5035SpcTotalNum_setting[2];
	struct cam_sensor_i2c_reg_setting sensor_setting;
	//for test
	struct cam_sensor_i2c_reg_array gc5035SRAM_setting;
	uint32_t temp_val[4];
	int j, i;

	if (totalDpcFlag == 0)
		return 0;

	sensor_setting.reg_setting = gc5035OTPWrite_setting[3].reg_setting;
	sensor_setting.addr_type = gc5035OTPWrite_setting[3].addr_type;
	sensor_setting.data_type = gc5035OTPWrite_setting[3].data_type;
	sensor_setting.size = gc5035OTPWrite_setting[3].size;
	sensor_setting.delay = gc5035OTPWrite_setting[3].delay;

	rc = camera_io_dev_write(&(s_ctrl->io_master_info), &sensor_setting);

	if (rc < 0) {
		CAM_ERR(CAM_SENSOR, "gc5035SpcWrite_setting Failed to write sensor setting");
		return rc;
	}
	gc5035SpcTotalNum_setting[0].reg_addr = 0x01;
	gc5035SpcTotalNum_setting[0].reg_data = (totalDpcNum >> 8) & 0x07;
	gc5035SpcTotalNum_setting[0].delay = gc5035SpcTotalNum_setting[0].data_mask = 0;

	gc5035SpcTotalNum_setting[1].reg_addr = 0x02;
	gc5035SpcTotalNum_setting[1].reg_data = totalDpcNum & 0xff;
	gc5035SpcTotalNum_setting[1].delay = gc5035SpcTotalNum_setting[1].data_mask = 0;

	sensor_setting.reg_setting = gc5035SpcTotalNum_setting;
	sensor_setting.addr_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
	sensor_setting.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
	sensor_setting.size = 2;
	sensor_setting.delay = 0;

	rc = camera_io_dev_write(&(s_ctrl->io_master_info), &sensor_setting);

	if (rc < 0) {
		CAM_ERR(CAM_SENSOR, "gc5035SpcWrite_setting Failed to write sensor setting");
		return rc;
	}

	sensor_setting.reg_setting = gc5035OTPWrite_setting[4].reg_setting;
	sensor_setting.addr_type = gc5035OTPWrite_setting[4].addr_type;
	sensor_setting.data_type = gc5035OTPWrite_setting[4].data_type;
	sensor_setting.size = gc5035OTPWrite_setting[4].size;
	sensor_setting.delay = gc5035OTPWrite_setting[4].delay;

	rc = camera_io_dev_write(&(s_ctrl->io_master_info), &sensor_setting);

	if (rc < 0) {
		CAM_ERR(CAM_SENSOR, "gc5035SpcWrite_setting Failed to write sensor setting");
		return rc;
	}
	gc5035SpcTotalNum_setting[0].reg_addr = 0xfe;
	gc5035SpcTotalNum_setting[0].reg_data = 0x02;
	gc5035SpcTotalNum_setting[0].delay = gc5035SpcTotalNum_setting[0].data_mask = 0;

	gc5035SpcTotalNum_setting[1].reg_addr = 0xbe;
	gc5035SpcTotalNum_setting[1].reg_data = 0x00;
	gc5035SpcTotalNum_setting[1].delay = gc5035SpcTotalNum_setting[1].data_mask = 0;
	sensor_setting.reg_setting = gc5035SpcTotalNum_setting;
	sensor_setting.addr_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
	sensor_setting.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
	sensor_setting.size = 2;
	sensor_setting.delay = 0;
	rc = camera_io_dev_write(&(s_ctrl->io_master_info), &sensor_setting);

	if (rc < 0) {
		CAM_ERR(CAM_SENSOR, "gc5035SpcWrite_setting Failed to write sensor setting");
		return rc;
	}
	for (i = 0; i < totalDpcNum * 4; i++) {
		gc5035SRAM_setting.reg_addr = 0xaa;
		gc5035SRAM_setting.reg_data = i;
		gc5035SRAM_setting.delay = gc5035SRAM_setting.data_mask = 0;
		sensor_setting.reg_setting = &gc5035SRAM_setting;
		sensor_setting.addr_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
		sensor_setting.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
		sensor_setting.size = 1;
		sensor_setting.delay = 0;
		rc = camera_io_dev_write(&(s_ctrl->io_master_info), &sensor_setting);
		for (j = 0; j < 4; j++) {
			rc = camera_io_dev_read(
			         &(s_ctrl->io_master_info),
			         0xac,
			         &temp_val[j], CAMERA_SENSOR_I2C_TYPE_BYTE,
			         CAMERA_SENSOR_I2C_TYPE_BYTE);
			if (rc < 0) {
				CAM_ERR(CAM_SENSOR, "gc5035SpcWrite_setting Failed to read dpc info sensor setting");
				break;
			}
		}
		CAM_INFO(CAM_SENSOR, "GC5035_OTP_GC val0 = 0x%x , val1 = 0x%x , val2 = 0x%x,val3 = 0x%x \n",
		         temp_val[0], temp_val[1], temp_val[2], temp_val[3]);
		CAM_INFO(CAM_SENSOR, "GC5035_OTP_GC x = %d , y = %d ,type = %d \n",
		         ((temp_val[1] & 0x0f) << 8) + temp_val[0], ((temp_val[2] & 0x7f) << 4) + ((temp_val[1] & 0xf0) >> 4), (((temp_val[3] & 0x01) << 1) + ((temp_val[2] & 0x80) >> 7)));
	}

	gc5035SpcTotalNum_setting[0].reg_addr = 0xbe;
	gc5035SpcTotalNum_setting[0].reg_data = 0x01;
	gc5035SpcTotalNum_setting[0].delay = gc5035SpcTotalNum_setting[0].data_mask = 0;

	gc5035SpcTotalNum_setting[1].reg_addr = 0xfe;
	gc5035SpcTotalNum_setting[1].reg_data = 0x00;
	gc5035SpcTotalNum_setting[1].delay = gc5035SpcTotalNum_setting[1].data_mask = 0;

	sensor_setting.reg_setting = gc5035SpcTotalNum_setting;
	sensor_setting.addr_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
	sensor_setting.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
	sensor_setting.size = 2;
	sensor_setting.delay = 0;
	rc = camera_io_dev_write(&(s_ctrl->io_master_info), &sensor_setting);

	if (rc < 0) {
		CAM_ERR(CAM_SENSOR, "gc5035SpcWrite_setting Failed to write sensor setting");
		return rc;
	}
	return rc;
}

int sensor_gc5035_update_reg(struct cam_sensor_ctrl_t *s_ctrl)
{
	int rc = -1;
	uint8_t flag_chipv = 0;
	int i = 0;
	uint8_t VALID_FLAG = 0x01;
	uint8_t CHIPV_FLAG_OFFSET = 0x0;
	uint8_t CHIPV_OFFSET = 0x01;
	uint8_t reg_setting_size = 0;
	struct cam_sensor_i2c_reg_array gc5035_update_reg_setting[20];
	struct cam_sensor_i2c_reg_setting sensor_setting;
	CAM_DBG(CAM_SENSOR, "Enter");

	flag_chipv = gc5035_chipversion_buffer[CHIPV_FLAG_OFFSET];
	CAM_DBG(CAM_SENSOR, "gc5035 otp chipv flag_chipv: 0x%x", flag_chipv);
	if (VALID_FLAG != (flag_chipv & 0x03)) {
		CAM_ERR(CAM_SENSOR, "gc5035 otp chip regs data is Empty/Invalid!");
		return rc;
	}

	for (i = 0; i < 5; i++) {
		if (VALID_FLAG == ((gc5035_chipversion_buffer[CHIPV_OFFSET + 5 * i] >> 3) & 0x01)) {
			gc5035_update_reg_setting[reg_setting_size].reg_addr = 0xfe;
			gc5035_update_reg_setting[reg_setting_size].reg_data = gc5035_chipversion_buffer[CHIPV_OFFSET + 5 * i] & 0x07;
			gc5035_update_reg_setting[reg_setting_size].delay = gc5035_update_reg_setting[reg_setting_size].data_mask = 0;
			reg_setting_size++;
			gc5035_update_reg_setting[reg_setting_size].reg_addr = gc5035_chipversion_buffer[CHIPV_OFFSET + 5 * i + 1];
			gc5035_update_reg_setting[reg_setting_size].reg_data = gc5035_chipversion_buffer[CHIPV_OFFSET + 5 * i + 2];
			gc5035_update_reg_setting[reg_setting_size].delay = gc5035_update_reg_setting[reg_setting_size].data_mask = 0;
			reg_setting_size++;

			CAM_DBG(CAM_SENSOR, "gc5035 otp chipv : 0xfe=0x%x, addr[%d]=0x%x, value[%d]=0x%x", gc5035_chipversion_buffer[CHIPV_OFFSET +  5 * i] & 0x07, i * 2,
			        gc5035_chipversion_buffer[CHIPV_OFFSET + 5 * i + 1], i * 2, gc5035_chipversion_buffer[CHIPV_OFFSET + 5 * i + 2]);
		}
		if (VALID_FLAG == ((gc5035_chipversion_buffer[CHIPV_OFFSET + 5 * i] >> 7) & 0x01)) {
			gc5035_update_reg_setting[reg_setting_size].reg_addr = 0xfe;
			gc5035_update_reg_setting[reg_setting_size].reg_data = (gc5035_chipversion_buffer[CHIPV_OFFSET + 5 * i] & 0x70) >> 4;
			gc5035_update_reg_setting[reg_setting_size].delay = gc5035_update_reg_setting[reg_setting_size].data_mask = 0;
			reg_setting_size++;
			gc5035_update_reg_setting[reg_setting_size].reg_addr = gc5035_chipversion_buffer[CHIPV_OFFSET + 5 * i + 3];
			gc5035_update_reg_setting[reg_setting_size].reg_data = gc5035_chipversion_buffer[CHIPV_OFFSET + 5 * i + 4];
			gc5035_update_reg_setting[reg_setting_size].delay = gc5035_update_reg_setting[reg_setting_size].data_mask = 0;
			reg_setting_size++;

			CAM_DBG(CAM_SENSOR, "gc5035 otp chipv : 0xfe=0x%x, addr[%d]=0x%x, value[%d]=0x%x", (gc5035_chipversion_buffer[CHIPV_OFFSET + 5 * i] & 0x70) >> 4, i * 2 + 1,
			        gc5035_chipversion_buffer[CHIPV_OFFSET + 5 * i + 3], i * 2 + 1, gc5035_chipversion_buffer[CHIPV_OFFSET + 5 * i + 4]);
		}
	}
	sensor_setting.reg_setting = gc5035_update_reg_setting;
	sensor_setting.addr_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
	sensor_setting.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
	sensor_setting.size = reg_setting_size;
	sensor_setting.delay = 0;

	rc = camera_io_dev_write(&(s_ctrl->io_master_info), &sensor_setting);

	if (rc < 0) {
		CAM_ERR(CAM_SENSOR, "gc5035SpcWrite_setting Failed to write sensor setting");
		return rc;
	}
	rc = 0;
	CAM_DBG(CAM_SENSOR, "Exit");
	return rc;

}

int sensor_start_thread(void *arg)
{
	struct cam_sensor_ctrl_t *s_ctrl = (struct cam_sensor_ctrl_t *)arg;
	int rc = 0;
	struct cam_sensor_i2c_reg_setting sensor_init_setting;

	CAM_ERR(CAM_SENSOR, "sensor start thread run ---");

	if (!s_ctrl) {
		CAM_ERR(CAM_SENSOR, "s_ctrl is NULL");
		return -1;
	}
	mutex_lock(&(s_ctrl->cam_sensor_mutex));

	//power up for sensor
	mutex_lock(&(s_ctrl->sensor_power_state_mutex));
	if(s_ctrl->sensor_power_state == CAM_SENSOR_POWER_OFF) {
		rc = cam_sensor_power_up(s_ctrl);
		if(rc < 0) {
			CAM_ERR(CAM_SENSOR, "sensor power up faild!");
		} else {
			CAM_INFO(CAM_SENSOR, "sensor power up success sensor id 0x%x", s_ctrl->sensordata->slave_info.sensor_id);
			s_ctrl->sensor_power_state = CAM_SENSOR_POWER_ON;
		}
	} else {
		CAM_INFO(CAM_SENSOR, "sensor have power up!");
	}
	mutex_unlock(&(s_ctrl->sensor_power_state_mutex));

	//write initsetting for sensor
	if (rc == 0) {
		mutex_lock(&(s_ctrl->sensor_initsetting_mutex));
		if (s_ctrl->sensor_initsetting_state == CAM_SENSOR_SETTING_WRITE_INVALID) {
			CAM_INFO(CAM_SENSOR, "start write init setting %x , sensorVersion %d ",
			         s_ctrl->sensordata->slave_info.sensor_id,
			         s_ctrl->sensordata->slave_info.sensor_version);
			if (s_ctrl->sensordata->slave_info.sensor_id == 0x689) {

				sensor_init_setting.reg_setting = sensor_init_settings.imx689_setting.reg_setting;
				sensor_init_setting.addr_type = CAMERA_SENSOR_I2C_TYPE_WORD;
				sensor_init_setting.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
				sensor_init_setting.size = sensor_init_settings.imx689_setting.size;
				sensor_init_setting.delay = sensor_init_settings.imx689_setting.delay;
				rc = camera_io_dev_write(&(s_ctrl->io_master_info), &sensor_init_setting);
				if(rc < 0) {
					CAM_ERR(CAM_SENSOR, "write setting failed!");
				} else {
					CAM_INFO(CAM_SENSOR, "write setting1 success!");
					s_ctrl->sensor_initsetting_state = CAM_SENSOR_SETTING_WRITE_SUCCESS;
				}
				sensor_init_setting.reg_setting = sensor_init_settings.imx689_setting1.reg_setting;
				sensor_init_setting.addr_type = CAMERA_SENSOR_I2C_TYPE_WORD;
				sensor_init_setting.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
				sensor_init_setting.size = sensor_init_settings.imx689_setting1.size;
				sensor_init_setting.delay = sensor_init_settings.imx689_setting1.delay;
				rc = camera_io_dev_write(&(s_ctrl->io_master_info), &sensor_init_setting);
				if(rc < 0) {
					CAM_ERR(CAM_SENSOR, "write setting failed!");
				} else {
					CAM_INFO(CAM_SENSOR, "write setting success!");
					s_ctrl->sensor_initsetting_state = CAM_SENSOR_SETTING_WRITE_SUCCESS;
				}

			} else if (s_ctrl->sensordata->slave_info.sensor_id == 0x766) {
				// imx766 senson veriosn 0 used for sensor module before dvt
				if(s_ctrl->sensordata->slave_info.sensor_version == 0) {
					sensor_init_setting.reg_setting = sensor_init_settings.imx766_init_setting1.reg_setting;
					sensor_init_setting.addr_type = CAMERA_SENSOR_I2C_TYPE_WORD;
					sensor_init_setting.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
					sensor_init_setting.size = sensor_init_settings.imx766_init_setting1.size;
					sensor_init_setting.delay = sensor_init_settings.imx766_init_setting1.delay;
				} else {
					sensor_init_setting.reg_setting = sensor_init_settings.imx766_init_setting2.reg_setting;
					sensor_init_setting.addr_type = CAMERA_SENSOR_I2C_TYPE_WORD;
					sensor_init_setting.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
					sensor_init_setting.size = sensor_init_settings.imx766_init_setting2.size;
					sensor_init_setting.delay = sensor_init_settings.imx766_init_setting2.delay;
				}

				rc = camera_io_dev_write(&(s_ctrl->io_master_info), &sensor_init_setting);
				if(rc < 0) {
					CAM_ERR(CAM_SENSOR, "write setting failed!");
				} else {
					CAM_INFO(CAM_SENSOR, "write setting success!");
					s_ctrl->sensor_initsetting_state = CAM_SENSOR_SETTING_WRITE_SUCCESS;
				}

			} else {
				CAM_ERR(CAM_SENSOR, "cant find oem advance init setting !");
			}
		} else {
			CAM_INFO(CAM_SENSOR, "sensor setting alread wrote or do not need to write it !");
		}
		mutex_unlock(&(s_ctrl->sensor_initsetting_mutex));
	}
	CAM_ERR(CAM_SENSOR, "sensor start thread run end ---");

	mutex_unlock(&(s_ctrl->cam_sensor_mutex));
	return rc;

}

int cam_sensor_start(struct cam_sensor_ctrl_t *s_ctrl)
{
	int rc = 0;

	if(s_ctrl == NULL) {
		CAM_ERR(CAM_SENSOR, "s_ctrl is null ");
		return -1;
	}

	mutex_lock(&(s_ctrl->cam_sensor_mutex));

	mutex_lock(&(s_ctrl->sensor_power_state_mutex));
	if(s_ctrl->sensor_power_state == CAM_SENSOR_POWER_OFF) {
		s_ctrl->sensor_open_thread = kthread_run(sensor_start_thread, s_ctrl, s_ctrl->device_name);
		if (!s_ctrl->sensor_open_thread) {
			CAM_ERR(CAM_SENSOR, "create sensor start thread failed");
			rc = -1;
		} else {
			CAM_INFO(CAM_SENSOR, "create sensor start thread success");
		}
	} else {
		CAM_INFO(CAM_SENSOR, "sensor have power up");
	}
	mutex_unlock(&(s_ctrl->sensor_power_state_mutex));

	mutex_unlock(&(s_ctrl->cam_sensor_mutex));
	return rc;
}

int cam_sensor_stop(struct cam_sensor_ctrl_t *s_ctrl)
{
	int rc = 0;
	CAM_ERR(CAM_SENSOR, "sensor do stop");
	mutex_lock(&(s_ctrl->cam_sensor_mutex));

	//power off for sensor
	mutex_lock(&(s_ctrl->sensor_power_state_mutex));
	if(s_ctrl->sensor_power_state == CAM_SENSOR_POWER_ON) {
		rc = cam_sensor_power_down(s_ctrl);
		if(rc < 0) {
			CAM_ERR(CAM_SENSOR, "sensor power down faild!");
		} else {
			CAM_INFO(CAM_SENSOR, "sensor power down success sensor id 0x%x", s_ctrl->sensordata->slave_info.sensor_id);
			s_ctrl->sensor_power_state = CAM_SENSOR_POWER_OFF;
			mutex_lock(&(s_ctrl->sensor_initsetting_mutex));
			s_ctrl->sensor_initsetting_state = CAM_SENSOR_SETTING_WRITE_INVALID;
			mutex_unlock(&(s_ctrl->sensor_initsetting_mutex));
		}
	} else {
		CAM_INFO(CAM_SENSOR, "sensor have power down!");
	}
	mutex_unlock(&(s_ctrl->sensor_power_state_mutex));

	mutex_unlock(&(s_ctrl->cam_sensor_mutex));
	return rc;
}
