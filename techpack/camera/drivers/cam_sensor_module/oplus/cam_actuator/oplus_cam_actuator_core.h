/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2017-2018, The Linux Foundation. All rights reserved.
 * Copyright (c) 2020, Oplus. All rights reserved.
 */

#ifndef _OPLUS_CAM_ACTUATOR_CORE_H_
#define _OPLUS_CAM_ACTUATOR_CORE_H_

#include "cam_actuator_dev.h"

void oplus_cam_actuator_i2c_modes_util(
	struct camera_io_master *io_master_info,
	struct i2c_settings_list *i2c_list);
int32_t oplus_cam_actuator_power_up(struct cam_actuator_ctrl_t *a_ctrl);

#endif /* _CAM_ACTUATOR_CORE_H_ */
