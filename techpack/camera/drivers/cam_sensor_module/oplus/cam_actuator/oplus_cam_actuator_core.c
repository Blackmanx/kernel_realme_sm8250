// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2020, Oplus. All rights reserved.
 */

#include <linux/module.h>
#include "cam_sensor_cmn_header.h"
#include "cam_actuator_core.h"
#include "cam_sensor_util.h"
#include "cam_trace.h"
#include "cam_common_util.h"
#include "cam_packet_util.h"

#include "oplus_cam_actuator_core.h"

#ifdef OPLUS_FEATURE_CAMERA_COMMON
static uint32_t update_reg_arr[19][4] = {
	{0xAE, 0x3B, 0x00, 0x0},
	{0x10, 0x30, 0x00, 0x0},
	{0x11, 0x3A, 0x00, 0x0},
	{0x12, 0xF8, 0x00, 0x0},
	{0x13, 0x7C, 0x00, 0x0},
	{0x14, 0x66, 0x00, 0x0},
	{0x15, 0x22, 0x00, 0x0},
	{0x16, 0x15, 0x00, 0x0},
	{0x17, 0x62, 0x00, 0x0},
	{0x18, 0xDB, 0x00, 0x0},
	{0x19, 0x00, 0x00, 0x0},
	{0x1A, 0x0A, 0x00, 0x0},
	{0x20, 0x36, 0x00, 0x0},
	{0x21, 0x20, 0x00, 0x0},
	{0x22, 0x00, 0x00, 0x0},
	{0x23, 0x15, 0x00, 0x0},
	{0x03, 0x01, 0x5A, 0x0},
	{0x03, 0x02, 0x5A, 0x0},
	{0x03, 0x08, 0x24, 0x0},
};

static uint32_t pid_r5_reg[29][3] = {
/*   addr  val   delay */
	{0x0A, 0x89, 0x00},
	{0x0B, 0x88, 0x00},
	{0x10, 0x24, 0x00},
	{0x11, 0x3C, 0x00},
	{0x12, 0x54, 0x00},
	{0x13, 0x56, 0x00},
	{0x14, 0x21, 0x00},
	{0x15, 0x00, 0x00},
	{0x16, 0x21, 0x00},
	{0x17, 0x40, 0x00},
	{0x18, 0xDB, 0x00},
	{0x1A, 0x00, 0x00},
	{0x1B, 0x5C, 0x00},
	{0x1C, 0xB0, 0x00},
	{0x1D, 0x92, 0x00},
	{0x1E, 0x50, 0x00},
	{0x1F, 0x52, 0x00},
	{0x20, 0x01, 0x00},
	{0x21, 0x02, 0x00},
	{0x22, 0x04, 0x00},
	{0x23, 0x28, 0x00},
	{0x24, 0xFF, 0x00},
	{0x25, 0x1E, 0x00},
	{0x26, 0x58, 0x00},
	{0xAE, 0x3B, 0x00},
	{0x03, 0x02, 0xFF},
	{0x4B, 0x00, 0x00},
	{0xAE, 0x00, 0x00},
	{0xFF, 0x05, 0x64}, /* pid version r5 */
};

static int RamWriteByte(struct cam_actuator_ctrl_t *a_ctrl,
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
		.addr_type = CAMERA_SENSOR_I2C_TYPE_BYTE,
		.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE,
		.delay = mdelay,
	};
	if (a_ctrl == NULL) {
		CAM_ERR(CAM_ACTUATOR, "Invalid Args");
		return -EINVAL;
	}

	CAM_ERR(CAM_ACTUATOR, "addr is %x,data is %x", addr, data);

	for(i = 0; i < retry; i++) {
		rc = camera_io_dev_write(&(a_ctrl->io_master_info), &i2c_write);
		if (rc < 0) {
			CAM_ERR(CAM_ACTUATOR, "write 0x%04x failed, retry:%d", addr, i+1);
		} else {
			return rc;
		}
	}
	return rc;
}

static int32_t cam_actuator_update_pid(struct cam_actuator_ctrl_t *a_ctrl)
{
	int32_t UPDATE_REG_SIZE = 19;
	int32_t cnt = 0;
	int32_t rc = 0;
	int32_t re = 0;
	int32_t retry = 1;
	uint32_t reg_data = 0;

	CAM_INFO(CAM_ACTUATOR, "entry imx708 new actuator.");
	if (0x18 >> 1 == a_ctrl->io_master_info.cci_client->sid) {
		CAM_INFO(CAM_ACTUATOR, "actuator_addr = 0x%x",
			a_ctrl->io_master_info.cci_client->sid);

		/*wait 10 ms*/
		msleep(10);
		for (cnt = 0; cnt < UPDATE_REG_SIZE; cnt++) {
			rc = RamWriteByte(a_ctrl,
				update_reg_arr[cnt][0],
				update_reg_arr[cnt][1],
				update_reg_arr[cnt][2]);
			if (rc < 0) {
				CAM_ERR(CAM_ACTUATOR, "write imx708 new PID data error in step %d:rc %d", cnt, rc);
			}
		}
		for (re = 0; re < retry; re++) {
			rc = camera_io_dev_read(
				&(a_ctrl->io_master_info),
				0x4B, &reg_data,
				CAMERA_SENSOR_I2C_TYPE_BYTE, CAMERA_SENSOR_I2C_TYPE_BYTE);
			if (rc < 0 || ((reg_data & 0x04) != 0)) {
				CAM_ERR(CAM_ACTUATOR, "4Bh's data is 0x%x or the read io is error ,rc :%d", reg_data, rc);
				rc = RamWriteByte(a_ctrl, update_reg_arr[16][0], update_reg_arr[16][1], update_reg_arr[16][2]);
				rc = RamWriteByte(a_ctrl, update_reg_arr[17][0], update_reg_arr[17][1], update_reg_arr[17][2]);
			} else {
				break;
			}
		}
		rc = RamWriteByte(a_ctrl, 0xAE, 0x00, 0);
		if (rc < 0) {
			CAM_ERR(CAM_ACTUATOR, "Release Setting failed");
		}
	}
	return rc;
}

static int32_t cam_actuator_update_pid_to_r5(struct cam_actuator_ctrl_t *a_ctrl)
{
	int32_t UPDATE_REG_SIZE = 24;
	int32_t cnt = 2;
	int32_t rc = 0;
	uint32_t reg_data = 0;

	CAM_DBG(CAM_ACTUATOR, "start to update pid prarm to r5");
	if (0xE8 >> 1 == a_ctrl->io_master_info.cci_client->sid) {
		CAM_INFO(CAM_ACTUATOR, "actuator_addr = 0x%x",
			a_ctrl->io_master_info.cci_client->sid);

		/* change setting mode to write */
		rc = RamWriteByte(a_ctrl, pid_r5_reg[24][0], pid_r5_reg[24][1], pid_r5_reg[24][2]);
		if (rc < 0) {
			CAM_ERR(CAM_ACTUATOR, "write change setting mode fail rc %d", rc);
			return rc;
		}

		/* write register data */
		for (cnt = 2; cnt < UPDATE_REG_SIZE; cnt++) {
			rc = RamWriteByte(a_ctrl, pid_r5_reg[cnt][0], pid_r5_reg[cnt][1], pid_r5_reg[cnt][2]);
			if (rc < 0) {
				CAM_ERR(CAM_ACTUATOR, "write ov64b new PID data error in step %d:rc %d", cnt, rc);
			}
		}

		/* store operation */
		rc = RamWriteByte(a_ctrl, pid_r5_reg[25][0], pid_r5_reg[25][1], pid_r5_reg[25][2]);
		if (rc < 0) {
			CAM_ERR(CAM_ACTUATOR, "write store register fail rc %d", rc);
		}

		/* msleep(0xD8);  */

		/* write pid version to r5 */
		rc = RamWriteByte(a_ctrl, pid_r5_reg[28][0], pid_r5_reg[28][1], pid_r5_reg[28][2]);
		if (rc < 0) {
			CAM_ERR(CAM_ACTUATOR, "write store register fail rc %d", rc);
		}

		a_ctrl->io_master_info.cci_client->sid = 0xE9 >> 1;

		/* read register */
		rc = camera_io_dev_read(
			&(a_ctrl->io_master_info),
			pid_r5_reg[26][0], &reg_data,
			CAMERA_SENSOR_I2C_TYPE_BYTE, CAMERA_SENSOR_I2C_TYPE_BYTE);
		if (rc < 0 || ((reg_data & 0x04) != 0)) {
			CAM_ERR(CAM_ACTUATOR, "4Bh's data is 0x%x or the read io is error ,rc :%d", reg_data, rc);
		}

		a_ctrl->io_master_info.cci_client->sid = 0xE8 >> 1;

		/* release setting mode */
		rc = RamWriteByte(a_ctrl, pid_r5_reg[27][0], pid_r5_reg[27][1], pid_r5_reg[27][2]);
		if (rc < 0) {
			CAM_ERR(CAM_ACTUATOR, "release setting mode fail rc %d", rc);
		}

		/* read and check all register data */
		for (cnt = 0; cnt < UPDATE_REG_SIZE; cnt++) {
			rc = camera_io_dev_read(
				&(a_ctrl->io_master_info),
				pid_r5_reg[cnt][0], &reg_data,
				CAMERA_SENSOR_I2C_TYPE_BYTE, CAMERA_SENSOR_I2C_TYPE_BYTE);
			if (rc < 0) {
				CAM_ERR(CAM_ACTUATOR, "read reg 0x%x fail rc %d", pid_r5_reg[cnt][0], rc);
				break;
			}
			CAM_DBG(CAM_ACTUATOR, "read reg 0x%x 0x%x", pid_r5_reg[cnt][0], reg_data);
			if (reg_data != pid_r5_reg[cnt][1]) {
				CAM_ERR(CAM_ACTUATOR, "pid data wrong reg 0x%x val 0x%x acquire 0x%x rc %d",
					pid_r5_reg[cnt][0], reg_data, pid_r5_reg[cnt][1], rc);
				rc = -1;
				break;
			}
		}
	}
	return rc;
}

static int32_t cam_actuator_check_firmware(struct cam_actuator_ctrl_t *a_ctrl)
{
	int32_t UPDATE_REG_SIZE = 16;
	int32_t cnt = 0;
	int32_t rc = 0;
	uint32_t reg_data = 0;
	uint32_t pid_version_reg = 0xFF;
	uint32_t PID_R2 = 0;

	CAM_INFO(CAM_ACTUATOR, "start check actuator  firmware.");
	if (0x18 >> 1 == a_ctrl->io_master_info.cci_client->sid) {
		CAM_INFO(CAM_ACTUATOR, "actuator_addr = 0x%x",
			a_ctrl->io_master_info.cci_client->sid);

		/* wait 10 ms */
		msleep(10);
		for (cnt = 1; cnt < UPDATE_REG_SIZE; cnt++) {
			rc = camera_io_dev_read(
				&(a_ctrl->io_master_info),
				update_reg_arr[cnt][0], &reg_data,
				CAMERA_SENSOR_I2C_TYPE_BYTE, CAMERA_SENSOR_I2C_TYPE_BYTE);
			if (rc < 0) {
				CAM_ERR(CAM_ACTUATOR, "read imx708 new PID data error in step %d:rc %d", cnt, rc);
				break;
			}
			if (reg_data != update_reg_arr[cnt][1]) {
				CAM_ERR(CAM_ACTUATOR, "imx708 new PID data wrong in step %d:rc %d, reg_data is %x", cnt, rc, reg_data);
				rc = -1;
				break;
			}
		}
	}

	if (0xE8 >> 1 == a_ctrl->io_master_info.cci_client->sid ||
		0xE9 >> 1 == a_ctrl->io_master_info.cci_client->sid) {
		CAM_INFO(CAM_ACTUATOR, "actuator_addr = 0x%x",
			a_ctrl->io_master_info.cci_client->sid);

		msleep(10);
		rc = camera_io_dev_read(
			&(a_ctrl->io_master_info),
			pid_version_reg, &reg_data,
			CAMERA_SENSOR_I2C_TYPE_BYTE, CAMERA_SENSOR_I2C_TYPE_BYTE);
		if (rc < 0) {
			CAM_ERR(CAM_ACTUATOR, "read pid version err rc %d", rc);
		}
		if (reg_data == PID_R2) {
			rc = -1;
			CAM_ERR(CAM_ACTUATOR, "pid version PID_R2");
		} else {
			CAM_INFO(CAM_ACTUATOR, "verify pid r5 register data");
			/* read and check all register data */
			for (cnt = 0; cnt < 24; cnt++) {
				rc = camera_io_dev_read(
					&(a_ctrl->io_master_info),
					pid_r5_reg[cnt][0], &reg_data,
					CAMERA_SENSOR_I2C_TYPE_BYTE, CAMERA_SENSOR_I2C_TYPE_BYTE);
				if (rc < 0) {
					CAM_ERR(CAM_ACTUATOR, "read reg 0x%x fail rc %d", pid_r5_reg[cnt][0], rc);
					break;
				}
				if (reg_data != pid_r5_reg[cnt][1]) {
					CAM_ERR(CAM_ACTUATOR, "reg 0x%2x val 0x%2x different to r5 param val 0x%2x",
						pid_r5_reg[cnt][0], reg_data, pid_r5_reg[cnt][1]);
					a_ctrl->is_actuator_pid_updated = false;
					break;
				}
				else {
					a_ctrl->is_actuator_pid_updated = true;
				}
				CAM_INFO(CAM_ACTUATOR, "read reg 0x%x 0x%x", pid_r5_reg[cnt][0], reg_data);
			}
		}
	}

	return rc;
}

#endif

int32_t oplus_cam_actuator_power_up(struct cam_actuator_ctrl_t *a_ctrl)
{
	int retry = 2;
	int re = 0;
	int rc = 0;

	if (a_ctrl->need_check_actuator_data
		&& (1 == a_ctrl->io_master_info.cci_client->cci_i2c_master)
		&& (0 == a_ctrl->io_master_info.cci_client->cci_device)) {
		for (re = 0;re < retry; re++) {
			rc = cam_actuator_check_firmware(a_ctrl);
			if (rc < 0) {
				/*if rc is error ,update the pid eeprom*/
				CAM_INFO(CAM_ACTUATOR, "check the pid data is empty ,will store the pid data!");
				rc = cam_actuator_update_pid(a_ctrl);
			}
			if (rc < 0) {
				CAM_ERR(CAM_ACTUATOR, "update the pid data error,check the io ctrl!");
			} else {
				break;
			}
		}
	}
	CAM_INFO(CAM_ACTUATOR, "is_actuator_pid_updated %d,need_check_actuator_data:%d", a_ctrl->is_actuator_pid_updated,
	a_ctrl->need_check_actuator_data);

	if (a_ctrl->need_check_actuator_data
		&& (0 == a_ctrl->io_master_info.cci_client->cci_i2c_master)
		&& (0 == a_ctrl->io_master_info.cci_client->cci_device)
		&& !(a_ctrl->is_actuator_pid_updated)) {
		for (re = 0;re < retry; re++) {
			rc = cam_actuator_check_firmware(a_ctrl);
			CAM_INFO(CAM_ACTUATOR, "cam_actuator_check_firmware rc:%d", rc);

			if (rc < 0) {
				CAM_ERR(CAM_ACTUATOR, "update pid version to r5");
				rc = cam_actuator_update_pid_to_r5(a_ctrl);
				if (rc < 0) {
					a_ctrl->is_actuator_pid_updated = false;
					CAM_ERR(CAM_ACTUATOR, "actuator updated pid to r5 version failed! check the io ctrl! is_actuator_pid_updated %d",
					a_ctrl->is_actuator_pid_updated);
				} else {
					a_ctrl->is_actuator_pid_updated = true;
					CAM_INFO(CAM_ACTUATOR, "actuator successfully updated pid to r5 version  %d", a_ctrl->is_actuator_pid_updated);
					break;
				}
			} else {
				a_ctrl->is_actuator_pid_updated = true;
				rc = 0;
			}

			if (a_ctrl->is_actuator_pid_updated == true) {
				break;
			}
		}

		if (a_ctrl->is_actuator_pid_updated == false) {
			CAM_ERR(CAM_ACTUATOR, "actuator pid update fail addr:0x%x", a_ctrl->io_master_info.cci_client->sid);
		}
	}
	else {
		CAM_INFO(CAM_ACTUATOR, "Update conditions are not met !");
	}

	return rc;
}

void oplus_cam_actuator_i2c_modes_util(
	struct camera_io_master *io_master_info,
	struct i2c_settings_list *i2c_list)
{
	uint32_t value;
	if (i2c_list->i2c_settings.reg_setting[0].reg_addr == 0x0204) {
		value = (i2c_list->i2c_settings.reg_setting[0].reg_data & 0xFF00) >> 8;
		i2c_list->i2c_settings.reg_setting[0].reg_data =
		((i2c_list->i2c_settings.reg_setting[0].reg_data & 0xFF) << 8) | value;
		CAM_DBG(CAM_ACTUATOR, "new value %d", i2c_list->i2c_settings.reg_setting[0].reg_data);
	}
}
