/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2017-2019, The Linux Foundation. All rights reserved.
 * Copyright (c) 2020, Oplus. All rights reserved.
 */


#ifndef _OPLUS_CAM_ACTUATOR_DEV_H_
#define _OPLUS_CAM_ACTUATOR_DEV_H_

#include <cam_sensor_io.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/module.h>
#include <linux/iommu.h>
#include <linux/timer.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-subdev.h>
#include <cam_cci_dev.h>
#include <cam_sensor_cmn_header.h>
#include <cam_subdev.h>
#include "cam_sensor_util.h"
#include "cam_soc_util.h"
#include "cam_debug_util.h"
#include "cam_context.h"

void oplus_cam_actuator_fswitch_dev_file_create(struct cam_actuator_ctrl_t *a_ctrl,
    struct platform_device *pdev);


#endif /* _OPLUS_CAM_ACTUATOR_DEV_H_ */
