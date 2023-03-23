/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2017-2019, The Linux Foundation. All rights reserved.
 * Copyright (c) 2020, Oplus. All rights reserved.
 */

#ifndef _OPLUS_CAM_SENSOR_DEV_H_
#define _OPLUS_CAM_SENSOR_DEV_H_
#include "cam_sensor_dev.h"

long oplus_cam_sensor_subdev_ioctl(struct v4l2_subdev *sd,
	unsigned int cmd, void *arg, unsigned int *is_ftm_current_test);

#endif /* _OPLUS_CAM_SENSOR_DEV_H_ */
