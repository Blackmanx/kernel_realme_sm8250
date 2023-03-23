// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2017-2018, The Linux Foundation. All rights reserved.
 * Copyright (c) 2020, Oplus. All rights reserved.
 */

#include "cam_actuator_dev.h"
#include "cam_req_mgr_dev.h"
#include "cam_actuator_soc.h"
#include "cam_actuator_core.h"
#include "cam_trace.h"

#ifdef OPLUS_FEATURE_CAMERA_COMMON
static int32_t cam_actuator_power(struct cam_actuator_ctrl_t *a_ctrl, int enable)
{
	int rc = 0;
	struct cam_hw_soc_info  *soc_info =
		&a_ctrl->soc_info;
	struct cam_actuator_soc_private  *soc_private;
	struct cam_sensor_power_ctrl_t *power_info;

	soc_private =
		(struct cam_actuator_soc_private *)a_ctrl->soc_info.soc_private;
	power_info = &soc_private->power_info;

	if ((power_info->power_setting == NULL) &&
		(power_info->power_down_setting == NULL)) {
		CAM_INFO(CAM_ACTUATOR,
			"Using default power settings");
		rc = cam_actuator_construct_default_power_setting(power_info);
		if (rc < 0) {
			CAM_ERR(CAM_ACTUATOR,
				"Construct default actuator power setting failed.");
			return rc;
		}
	}

	/* Parse and fill vreg params for power up settings */
	rc = msm_camera_fill_vreg_params(
		&a_ctrl->soc_info,
		power_info->power_setting,
		power_info->power_setting_size);
	if (rc) {
		CAM_ERR(CAM_ACTUATOR,
			"failed to fill vreg params for power up rc:%d", rc);
		return rc;
	}

	/* Parse and fill vreg params for power down settings*/
	rc = msm_camera_fill_vreg_params(
		&a_ctrl->soc_info,
		power_info->power_down_setting,
		power_info->power_down_setting_size);
	if (rc) {
		CAM_ERR(CAM_ACTUATOR,
			"failed to fill vreg params power down rc:%d", rc);
		return rc;
	}

	power_info->dev = soc_info->dev;
	if (enable) {
		rc = cam_sensor_core_power_up(power_info, soc_info);
		if (rc) {
			CAM_ERR(CAM_ACTUATOR,
				"failed in actuator power up rc %d", rc);
			return rc;
		}
	} else {
		rc = cam_sensor_util_power_down(power_info, soc_info);
		if (rc) {
			CAM_ERR(CAM_ACTUATOR, "power down the core is failed:%d", rc);
			return rc;
		}
	}
	return rc;
}

static ssize_t cam_actuator_switch_store(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t count)
{
	int rc = 0;
 	struct cam_actuator_ctrl_t *data = dev_get_drvdata(dev);
	int enable = 0;
	if (data == NULL){
		pr_err("get actuator data NULL.\n");
		rc = -EINVAL;
	}
	if (kstrtoint(buf, 0, &enable)) {
		pr_err("get val error.\n");
		rc = -EINVAL;
	}
	CAM_ERR(CAM_ACTUATOR, "echo data = %d ", enable);
	mutex_lock(&(data->actuator_mutex));
	if (data->actuator_power_enable != enable) {
		data->actuator_power_enable = enable;
		cam_actuator_power(data, enable);
	}
	mutex_unlock(&(data->actuator_mutex));

	return count;
}

static ssize_t cam_actuator_switch_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
 	struct cam_actuator_ctrl_t *data = dev_get_drvdata(dev);
	if (data == NULL) {
		pr_err("get actuator data NULL.\n");
		return -EINVAL;
	} else {
		return snprintf(buf, 5, "%d\n", data->actuator_power_enable);
	}
}

static DEVICE_ATTR(fswitch, 0660, cam_actuator_switch_show,cam_actuator_switch_store);
void oplus_cam_actuator_fswitch_dev_file_create(struct cam_actuator_ctrl_t *a_ctrl,
    struct platform_device *pdev)
{
    a_ctrl->actuator_power_enable = 0;
    device_create_file(&pdev->dev, &dev_attr_fswitch);
}

#endif

