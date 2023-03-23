
/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2017-2019, The Linux Foundation. All rights reserved.
 * Copyright (c) 2020, Oplus. All rights reserved.
 */
#ifndef _OPLUS_CAM_OIS_CORE_H_
#define _OPLUS_CAM_OIS_CORE_H_

#include <linux/cma.h>
#include "cam_ois_dev.h"
struct cam_sensor_i2c_reg_setting_array {
	struct cam_sensor_i2c_reg_array reg_setting[512];
	unsigned short size;
	enum camera_sensor_i2c_type addr_type;
	enum camera_sensor_i2c_type data_type;
	unsigned short delay;
};

int cam_ois_apply_settings_oem(struct cam_ois_ctrl_t *o_ctrl,
	struct i2c_settings_list *i2c_list);

int32_t cam_lc898128_write_data(struct cam_ois_ctrl_t * o_ctrl,void * arg);

int oplus_cam_ois_driver_cmd(struct cam_ois_ctrl_t *o_ctrl, void *arg);

int cam_ois_bu63169_getmultiHall(
	struct cam_ois_ctrl_t *o_ctrl,
	OISHALL2EIS *hall_data);
int oplus_cam_ois_pkt_parse(struct cam_ois_ctrl_t *o_ctrl);
void set_ois_thread(struct cam_control *ioctl_ctrl);
#endif


