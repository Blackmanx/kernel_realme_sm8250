// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2017-2020, The Linux Foundation. All rights reserved.
 * Copyright (c) 2020, Oplus. All rights reserved.
 */

#include <linux/module.h>
#include <linux/firmware.h>
#include <linux/dma-contiguous.h>
#include <cam_sensor_cmn_header.h>
#include "cam_ois_core.h"
#include "cam_ois_soc.h"
#include "cam_sensor_util.h"
#include "cam_debug_util.h"
#include "cam_res_mgr_api.h"
#include "cam_common_util.h"
#include "cam_packet_util.h"
#include "oplus_cam_ois_core.h"
#include "onsemi_fw/fw_download_interface.h"

#ifndef OPLUS_FEATURE_CAMERA_COMMON
#define OPLUS_FEATURE_CAMERA_COMMON
#endif
#ifdef OPLUS_FEATURE_CAMERA_COMMON

#define MODE_NOCONTINUE 1
#define MODE_CONTINUE 0
#define MAX_LENGTH 160
#define CAMX_HALL_MAX_NUMBER 100

struct cam_sensor_i2c_reg_setting_array bu63169_pll_settings = {
    .reg_setting =
	{
		{.reg_addr = 0x8262, .reg_data = 0xFF02, .delay = 0x00, .data_mask = 0x00}, \
		{.reg_addr = 0x8263, .reg_data = 0x9F05, .delay = 0x01, .data_mask = 0x00}, \
		{.reg_addr = 0x8264, .reg_data = 0x6040, .delay = 0x00, .data_mask = 0x00}, \
		{.reg_addr = 0x8260, .reg_data = 0x1130, .delay = 0x00, .data_mask = 0X00}, \
		{.reg_addr = 0x8265, .reg_data = 0x8000, .delay = 0x00, .data_mask = 0x00}, \
		{.reg_addr = 0x8261, .reg_data = 0x0280, .delay = 0x00, .data_mask = 0x00}, \
		{.reg_addr = 0x8261, .reg_data = 0x0380, .delay = 0x00, .data_mask = 0x00}, \
		{.reg_addr = 0x8261, .reg_data = 0x0988, .delay = 0x00, .data_mask = 0X00}, \
	},
    .size = 8,
    .addr_type = CAMERA_SENSOR_I2C_TYPE_WORD,
    .data_type = CAMERA_SENSOR_I2C_TYPE_WORD,
    .delay = 1,
};

static int RamWriteWord(struct cam_ois_ctrl_t *o_ctrl,
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
		i2c_write.addr_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
		i2c_write.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
	}

	if (o_ctrl == NULL) {
		CAM_ERR(CAM_OIS, "Invalid Args");
		return -EINVAL;
	}

	for(i = 0; i < retry; i++)
	{
		rc = camera_io_dev_write(&(o_ctrl->io_master_info), &i2c_write);
		if (rc < 0) {
			CAM_ERR(CAM_OIS, "write 0x%x = 0x%0x failed, retry:%d !!!", addr, data, i+1);
		} else {
			CAM_DBG(CAM_OIS, "write 0x%x = 0x%0x", addr,data);
			return rc;
		}
	}
	return rc;
}
static int RamMultiWrite(struct cam_ois_ctrl_t *o_ctrl,
	struct cam_sensor_i2c_reg_setting *write_setting) {
	int rc = 0;
	int i = 0;
	for (i = 0; i < write_setting->size; i++) {
		rc = RamWriteWord(o_ctrl, write_setting->reg_setting[i].reg_addr,
			write_setting->reg_setting[i].reg_data);
	}
	return rc;
}

#endif

int cam_ois_apply_settings_oem(struct cam_ois_ctrl_t *o_ctrl,
	struct i2c_settings_list *i2c_list)
{
	int rc = 0;
#ifdef OPLUS_FEATURE_CAMERA_COMMON
	int mode = MODE_CONTINUE;
#endif
#ifdef OPLUS_FEATURE_CAMERA_COMMON
        if (strstr(o_ctrl->ois_name, "bu63169")) {
            mode = MODE_NOCONTINUE;
        }
#endif

    if (mode == MODE_CONTINUE) {
        rc = camera_io_dev_write(&(o_ctrl->io_master_info),
            &(i2c_list->i2c_settings));
    } else {
        rc = RamMultiWrite(o_ctrl, &(i2c_list->i2c_settings));
    }

	return rc;
}

static int oplus_cam_ois_fw_download(struct cam_ois_ctrl_t *o_ctrl)
{
	uint16_t                           total_bytes = 0;
	uint8_t                           *ptr = NULL;
	int32_t                            cnt;
	int                                i = 0;
	int32_t                            rc = 0;
	uint32_t                           fw_size;
	const struct firmware             *fw = NULL;
	const char                        *fw_name_prog = NULL;
	const char                        *fw_name_coeff = NULL;
	char                               name_prog[32] = {0};
	char                               name_coeff[32] = {0};
	struct device                     *dev = &(o_ctrl->pdev->dev);
	struct cam_sensor_i2c_reg_setting  i2c_reg_setting;
	struct page                       *page = NULL;

	if (!o_ctrl) {
		CAM_ERR(CAM_OIS, "Invalid Args");
		return -EINVAL;
	}

	snprintf(name_coeff, 32, "%s.coeff", o_ctrl->ois_name);

	snprintf(name_prog, 32, "%s.prog", o_ctrl->ois_name);

	/* cast pointer as const pointer*/
	fw_name_prog = name_prog;
	fw_name_coeff = name_coeff;

	/* Load FW */
	rc = request_firmware(&fw, fw_name_prog, dev);
	if (rc) {
		CAM_ERR(CAM_OIS, "Failed to locate %s", fw_name_prog);
		return rc;
	}

	total_bytes = fw->size;
	i2c_reg_setting.addr_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
	i2c_reg_setting.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
	i2c_reg_setting.size = total_bytes;
	i2c_reg_setting.delay = 0;
	fw_size = PAGE_ALIGN(sizeof(struct cam_sensor_i2c_reg_array) *
		total_bytes) >> PAGE_SHIFT;

	if (o_ctrl->m_ois_fw_mode.ois_fw_prog_ptr == NULL) {
		page = cma_alloc(dev_get_cma_area((o_ctrl->soc_info.dev)),
			fw_size, 0, GFP_KERNEL);
		if (!page) {
			CAM_ERR(CAM_OIS, "Failed in allocating i2c_array");
			release_firmware(fw);
			return -ENOMEM;
		}
		o_ctrl->m_ois_fw_mode.ois_fw_prog_ptr = (struct cam_sensor_i2c_reg_array *) (
			page_address(page));
		o_ctrl->m_ois_fw_mode.ois_fw_prog_size = fw_size;
	}

	i2c_reg_setting.reg_setting = (struct cam_sensor_i2c_reg_array *)o_ctrl->m_ois_fw_mode.ois_fw_prog_ptr;

	for (cnt = 0, ptr = (uint8_t *)fw->data; cnt < total_bytes;) {
		i2c_reg_setting.size = 0;
		for (i = 0; (i < MAX_LENGTH && cnt < total_bytes); i++,ptr++) {
			i2c_reg_setting.reg_setting[i].reg_addr =
				o_ctrl->opcode.prog;
			i2c_reg_setting.reg_setting[i].reg_data = *ptr;
			i2c_reg_setting.reg_setting[i].delay = 0;
			i2c_reg_setting.reg_setting[i].data_mask = 0;
			i2c_reg_setting.size++;
			cnt++;
		}
		i2c_reg_setting.delay = 0;
		if (i2c_reg_setting.size > 0) {
			rc = camera_io_dev_write_continuous(&(o_ctrl->io_master_info),
				&i2c_reg_setting, 1);
		}
	}

	if (rc < 0) {
		CAM_ERR(CAM_OIS, "OIS FW download failed %d", rc);
		goto release_firmware;
	}

	page = NULL;
	fw_size = 0;
	release_firmware(fw);

	rc = request_firmware(&fw, fw_name_coeff, dev);
	if (rc) {
		CAM_ERR(CAM_OIS, "Failed to locate %s", fw_name_coeff);
		return rc;
	}

	total_bytes = fw->size;
	i2c_reg_setting.addr_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
	i2c_reg_setting.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
	i2c_reg_setting.size = total_bytes;
	i2c_reg_setting.delay = 0;

	fw_size = PAGE_ALIGN(sizeof(struct cam_sensor_i2c_reg_array) *
		total_bytes) >> PAGE_SHIFT;

	if (o_ctrl->m_ois_fw_mode.ois_fw_coeff_ptr == NULL) {
		page = cma_alloc(dev_get_cma_area((o_ctrl->soc_info.dev)),
		fw_size, 0, GFP_KERNEL);
		if (!page) {
			CAM_ERR(CAM_OIS, "Failed in allocating i2c_array");
			release_firmware(fw);
			return -ENOMEM;
		}

		o_ctrl->m_ois_fw_mode.ois_fw_coeff_ptr = (struct cam_sensor_i2c_reg_array *) (
			page_address(page));
		o_ctrl->m_ois_fw_mode.ois_fw_coeff_size = fw_size;
	}
	i2c_reg_setting.reg_setting = (struct cam_sensor_i2c_reg_array *)o_ctrl->m_ois_fw_mode.ois_fw_coeff_ptr;

	for (cnt = 0, ptr = (uint8_t *)fw->data; cnt < total_bytes;) {
		i2c_reg_setting.size = 0;
		for (i = 0; (i < MAX_LENGTH && cnt < total_bytes); i++,ptr++) {
			i2c_reg_setting.reg_setting[i].reg_addr =
				o_ctrl->opcode.coeff;
				i2c_reg_setting.reg_setting[i].reg_data = *ptr;
				i2c_reg_setting.reg_setting[i].delay = 0;
				i2c_reg_setting.reg_setting[i].data_mask = 0;
				i2c_reg_setting.size++;
				cnt++;
		}
		i2c_reg_setting.delay = 0;
		if (i2c_reg_setting.size > 0) {
			rc = camera_io_dev_write_continuous(&(o_ctrl->io_master_info),
				&i2c_reg_setting, 1);
		}
	}

	if (rc < 0)
		CAM_ERR(CAM_OIS, "OIS FW download failed %d", rc);

release_firmware:
	release_firmware(fw);

	return rc;
}


#ifdef OPLUS_FEATURE_CAMERA_COMMON
#define OIS_HALL_DATA_SIZE   52
int cam_ois_bu63169_getmultiHall(
	struct cam_ois_ctrl_t *o_ctrl,
	OISHALL2EIS *hall_data)
{
	int32_t        rc = 0;
	uint8_t        data[OIS_HALL_DATA_SIZE] = {0x00};
	uint8_t        dataNum = 0;
	int            offset = 0;
	int32_t        i = 0;
	uint32_t       timeStamp = 0;
	uint32_t       mHalldata_X, mHalldata_Y, mdata[OIS_HALL_DATA_SIZE];
	struct timeval endTime, startTime;

	if (!o_ctrl) {
		CAM_ERR(CAM_OIS, "Invalid Args");
		return -EINVAL;
	}

	memset(hall_data, 0x00, sizeof(OISHALL2EIS));
	do_gettimeofday(&endTime);
	rc = camera_io_dev_read_seq(&(o_ctrl->io_master_info), 0x8A, data,
		CAMERA_SENSOR_I2C_TYPE_BYTE, CAMERA_SENSOR_I2C_TYPE_BYTE, OIS_HALL_DATA_SIZE);
	if (rc < 0) {
		CAM_ERR(CAM_OIS, "get mutil hall data fail");
		return -EINVAL;
	}

	dataNum = data[0];
	offset++;
	if (dataNum <= 0 || dataNum > HALL_MAX_NUMBER) {
		CAM_ERR(CAM_OIS, "get a wrong number of hall data:%d", dataNum);
		return -EINVAL;
	}

	for (i = 0; i < dataNum; i++) {
		mdata[i] = ((data[offset+3] & 0x00FF)
			| ((data[offset+2] << 8) & 0xFF00)
			| ((data[offset+1] << 16) & 0x00FF0000)
			| ((data[offset] << 24) & 0xFF000000));
		offset += 4;
	}

	timeStamp = ((uint32_t)(data[offset] << 8) | data[offset+1]);

	if ((long)(timeStamp * 1778) / 100 > endTime.tv_usec) {
		endTime.tv_sec = endTime.tv_sec - 1;
		endTime.tv_usec = (1000 * 1000 + endTime.tv_usec) - (timeStamp * 1778) / 100;
	} else {
		endTime.tv_usec = endTime.tv_usec - (timeStamp * 1778) / 100;
	}

	if (endTime.tv_usec < (long)(4000 * dataNum)) {
		startTime.tv_sec = endTime.tv_sec - 1;
		startTime.tv_usec = (1000 * 1000 + endTime.tv_usec) - (4000 * dataNum);
	} else {
		startTime.tv_sec = endTime.tv_sec;
		startTime.tv_usec = endTime.tv_usec - (4000 * dataNum);
	}

	for (i = 0; i < dataNum; i++) {
		mHalldata_X = ((mdata[i] >> 16) & 0xFFFF);
		mHalldata_Y = (mdata[i] & 0xFFFF);
		if ((startTime.tv_usec + 4000) / (1000 * 1000) >= 1) {
			startTime.tv_sec = startTime.tv_sec + 1;
			startTime.tv_usec = ((startTime.tv_usec + 4000) - 1000 * 1000);
		} else {
			startTime.tv_usec += 4000;
		}
		hall_data->datainfo[i].mHalldata = mdata[i];
		hall_data->datainfo[i].timeStampSec = startTime.tv_sec;
		hall_data->datainfo[i].timeStampUsec = startTime.tv_usec;
		CAM_DBG(CAM_OIS, "camxhalldata[%d] X:0x%04x  halldataY:0x%04x  sec = %d, us = %d", i,
			mHalldata_X,
			mHalldata_Y,
			hall_data->datainfo[i].timeStampSec,
			hall_data->datainfo[i].timeStampUsec);
	}

	return rc;
}


int32_t cam_lc898128_write_data(struct cam_ois_ctrl_t * o_ctrl,void * arg)
{
	int32_t  rc = 0;

    struct cam_control    *cmd = (struct cam_control *)arg;
	struct cam_write_eeprom_t cam_write_eeprom;

	forceExitpoll(o_ctrl);

	memset(&cam_write_eeprom, 0, sizeof(struct cam_write_eeprom_t));
	if (copy_from_user(&cam_write_eeprom, (void __user *) cmd->handle, sizeof(struct cam_write_eeprom_t))) {
		CAM_ERR(CAM_OIS, "Failed Copy from User");
		return -EFAULT;
	}

	//disable write protection
	if (cam_write_eeprom.isWRP == 0x01) {
		WriteEEpromData(&cam_write_eeprom);
	}
	//ReadEEpromData(&cam_write_eeprom);

	return rc;
}

#endif

void set_ois_thread(struct cam_control *ioctl_ctrl)
{
#ifdef OPLUS_FEATURE_CAMERA_COMMON
#define OIS_SET_THREAD_STATUS_MASK              0x1000
#define OIS_SET_MIAN_GET_HALL_DATA_THREAD_MASK  0x01
#define OIS_SET_TELE_GET_HALL_DATA_THREAD_MASK  0x02
   //add by hongbo.dai@camera, for read ois reg data
   int							   is_enable_tele_ois_thread = 0;
   int							   is_enable_main_ois_thread = 0;

   // send a dummy configure informaintion for set ois get hall thread
   if (ioctl_ctrl->reserved & OIS_SET_THREAD_STATUS_MASK) {
	   if (ioctl_ctrl->reserved & OIS_SET_MIAN_GET_HALL_DATA_THREAD_MASK){
		   is_enable_main_ois_thread = true;
	   }
	   if (ioctl_ctrl->reserved & OIS_SET_TELE_GET_HALL_DATA_THREAD_MASK){
		   is_enable_tele_ois_thread = true;
	   }
	   set_ois_thread_status(is_enable_main_ois_thread,is_enable_tele_ois_thread);
	   CAM_INFO(CAM_OIS,"set ois get hall data thread status : main:[%d] tele: [%d]",
		   is_enable_main_ois_thread,is_enable_tele_ois_thread);
   }
#endif
}

int oplus_cam_ois_apply_settings(struct cam_ois_ctrl_t *o_ctrl,
	struct i2c_settings_array *i2c_set)
{
	struct i2c_settings_list *i2c_list;
	int32_t rc = 0;
	uint32_t i, size;

	if (o_ctrl == NULL || i2c_set == NULL) {
		CAM_ERR(CAM_OIS, "Invalid Args");
		return -EINVAL;
	}

	if (i2c_set->is_settings_valid != 1) {
		CAM_ERR(CAM_OIS, " Invalid settings");
		return -EINVAL;
	}

	list_for_each_entry(i2c_list,
		&(i2c_set->list_head), list) {
		if (i2c_list->op_code ==  CAM_SENSOR_I2C_WRITE_RANDOM) {
#ifdef OPLUS_FEATURE_CAMERA_COMMON
			rc = cam_ois_apply_settings_oem(o_ctrl,i2c_list);
#else
			rc = camera_io_dev_write(&(o_ctrl->io_master_info),
				&(i2c_list->i2c_settings));
#endif
			if (rc < 0) {
				CAM_ERR(CAM_OIS,
					"Failed in Applying i2c wrt settings");
				return rc;
			}
		} else if (i2c_list->op_code == CAM_SENSOR_I2C_POLL) {
			size = i2c_list->i2c_settings.size;
			for (i = 0; i < size; i++) {
				rc = camera_io_dev_poll(
				&(o_ctrl->io_master_info),
				i2c_list->i2c_settings.reg_setting[i].reg_addr,
				i2c_list->i2c_settings.reg_setting[i].reg_data,
				i2c_list->i2c_settings.reg_setting[i].data_mask,
				i2c_list->i2c_settings.addr_type,
				i2c_list->i2c_settings.data_type,
				i2c_list->i2c_settings.reg_setting[i].delay);
				if (rc < 0) {
					CAM_ERR(CAM_OIS,
						"i2c poll apply setting Fail");
					return rc;
				}
			}
		}
	}

	return rc;
}

int oplus_cam_ois_pkt_parse(struct cam_ois_ctrl_t *o_ctrl)
{
	uint32_t 					   reg_val;
	int                            rc;
	struct cam_sensor_i2c_reg_setting sensor_setting;

		if (strstr(o_ctrl->ois_name, "sem1215")) {
			OISRead(o_ctrl, 0x1008, &reg_val);
			CAM_ERR(CAM_OIS, "read OIS fw Version:0x%0x", reg_val);
		}
		if (strstr(o_ctrl->ois_name, "bu63169")) {
			CAM_ERR(CAM_OIS, "need to write pll0 settings");
			sensor_setting.addr_type = CAMERA_SENSOR_I2C_TYPE_WORD;
			sensor_setting.data_type = CAMERA_SENSOR_I2C_TYPE_WORD;
			sensor_setting.size = bu63169_pll_settings.size;
			sensor_setting.delay = bu63169_pll_settings.delay;
			sensor_setting.reg_setting = bu63169_pll_settings.reg_setting;

			CAM_ERR(CAM_OIS, "need to write pll0 settings");
			rc = RamMultiWrite(o_ctrl, &sensor_setting);
			if (rc < 0) {
				CAM_ERR(CAM_OIS, "write pll settings error");
				return rc;
			}
		}
		if (o_ctrl->ois_fw_flag) {
			if (strstr(o_ctrl->ois_name, "lc898")) {
				o_ctrl->ois_module_vendor = (o_ctrl->opcode.pheripheral & 0xFF00) >> 8;
				o_ctrl->ois_actuator_vendor = o_ctrl->opcode.pheripheral & 0xFF;
				rc = DownloadFW(o_ctrl);
			} else {
				rc = oplus_cam_ois_fw_download(o_ctrl);
			}

			if (rc) {
				CAM_ERR(CAM_OIS, "Failed OIS FW Download");
				return rc;
			}
		}
#ifdef OPLUS_FEATURE_CAMERA_COMMON
		if (strstr(o_ctrl->ois_name, "lc898") != NULL) {
			if (o_ctrl->is_ois_calib) {
				rc = oplus_cam_ois_apply_settings(o_ctrl,
					&o_ctrl->i2c_calib_data);
				if (rc) {
					CAM_ERR(CAM_OIS, "Cannot apply calib data");
					return rc;
				}
			}
			Initcheck128(o_ctrl);
		}
#endif
		if (strstr(o_ctrl->ois_name, "bu63169")) {
			uint32_t sum_check = 0;
			rc = camera_io_dev_read(&(o_ctrl->io_master_info), 0x84F7, &sum_check,
			    CAMERA_SENSOR_I2C_TYPE_WORD, CAMERA_SENSOR_I2C_TYPE_WORD);
			if (rc < 0) {
				CAM_ERR(CAM_OIS, "read 0x84F7 fail");
			} else {
				CAM_ERR(CAM_OIS, "0x84F7 = 0x%x", sum_check);
			}
			rc = camera_io_dev_read(&(o_ctrl->io_master_info), 0x84F6, &sum_check,
				CAMERA_SENSOR_I2C_TYPE_WORD, CAMERA_SENSOR_I2C_TYPE_WORD);
			if (rc < 0) {
				CAM_ERR(CAM_OIS, "read 0x84F6 fail");
			} else {
				CAM_ERR(CAM_OIS, "0x84F6 = 0x%x", sum_check);
			}
		}


	return 0;
}


/**
 * cam_ois_driver_cmd - Handle ois cmds
 * @e_ctrl:     ctrl structure
 * @arg:        Camera control command argument
 *
 * Returns success or failure
 */
int oplus_cam_ois_driver_cmd(struct cam_ois_ctrl_t *o_ctrl, void *arg)
{
	int                              rc = 0;
	struct cam_control              *cmd = (struct cam_control *)arg;
	switch (cmd->op_code) {
#ifdef OPLUS_FEATURE_CAMERA_COMMON
case CAM_GET_OIS_EIS_HALL: {
        int get_hall_version;
        get_hall_version = cmd->reserved ;
        if (o_ctrl->cam_ois_state == CAM_OIS_START
            &&(strstr(o_ctrl->ois_name, "lc898") != NULL || strstr(o_ctrl->ois_name, "bu63169") != NULL)) {
            if (get_hall_version == GET_HALL_DATA_VERSION_V2){
                ReadOISHALLDataV2(o_ctrl, u64_to_user_ptr(cmd->handle));
            } else if (get_hall_version == GET_HALL_DATA_VERSION_V3){
                ReadOISHALLDataV3(o_ctrl, u64_to_user_ptr(cmd->handle));
            } else {
                ReadOISHALLData(o_ctrl, u64_to_user_ptr(cmd->handle));
            }
        } else if (o_ctrl->cam_ois_state == CAM_OIS_START
                   && strstr(o_ctrl->ois_name, "sem1215s_ois") != NULL) {
            if (get_hall_version == GET_HALL_DATA_VERSION_V2){
                Sem1215sReadOISHALLDataV2(o_ctrl, u64_to_user_ptr(cmd->handle));
            }
            else {
                Sem1215sReadOISHALLData(o_ctrl, u64_to_user_ptr(cmd->handle));
            }
        }else {
            CAM_DBG(CAM_OIS, "OIS in wrong state %d", o_ctrl->cam_ois_state);
        }
        break;
    }
case CAM_WRITE_CALIBRATION_DATA:
case CAM_WRITE_AE_SYNC_DATA:
    CAM_DBG(CAM_OIS, "CAM_WRITE_DATA");
    if (strstr(o_ctrl->ois_name, "lc898") != NULL) {
        rc = cam_lc898128_write_data(o_ctrl, arg);
        if (rc) {
            CAM_ERR(CAM_EEPROM, "Failed in write AE sync data");
	     return rc;
        }
    }
    break;
#endif
    default:
	       CAM_ERR(CAM_SENSOR, "Invalid Opcode: %d", cmd->op_code);
		rc = -EINVAL;
		break;
	}
	return rc;
}
