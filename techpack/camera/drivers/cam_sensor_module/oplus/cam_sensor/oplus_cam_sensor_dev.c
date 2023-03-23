// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2017-2020, The Linux Foundation. All rights reserved.
 * Copyright (c) 2020, Oplus. All rights reserved.
 */

#include "cam_sensor_dev.h"
#include "cam_req_mgr_dev.h"
#include "cam_sensor_soc.h"
#include "cam_sensor_core.h"
#include "oplus_cam_sensor_dev.h"
#include "oplus_cam_sensor_core.h"

struct cam_sensor_i2c_reg_setting_array {
	struct cam_sensor_i2c_reg_array reg_setting[4600];
	unsigned short size;
	enum camera_sensor_i2c_type addr_type;
	enum camera_sensor_i2c_type data_type;
	unsigned short delay;
};

struct cam_sensor_settings {
    struct cam_sensor_i2c_reg_setting_array imx766_setting;
    struct cam_sensor_i2c_reg_setting_array imx686_setting_part1;
    struct cam_sensor_i2c_reg_setting_array imx686_setting_part2;
    struct cam_sensor_i2c_reg_setting_array imx586_setting0;
    struct cam_sensor_i2c_reg_setting_array imx586_setting1;
    struct cam_sensor_i2c_reg_setting_array streamoff;
    struct cam_sensor_i2c_reg_setting_array s5k3m5_setting;
    struct cam_sensor_i2c_reg_setting_array imx471_setting;
    struct cam_sensor_i2c_reg_setting_array imx481_setting;
    struct cam_sensor_i2c_reg_setting_array gc5035_setting;
    struct cam_sensor_i2c_reg_setting_array imx689_setting;
    struct cam_sensor_i2c_reg_setting_array gc2375_setting;
    struct cam_sensor_i2c_reg_setting_array imx615_setting_part1;
    struct cam_sensor_i2c_reg_setting_array imx615_setting_part2;
    struct cam_sensor_i2c_reg_setting_array imx616_setting;
    struct cam_sensor_i2c_reg_setting_array imx708_setting;
    struct cam_sensor_i2c_reg_setting_array imx708_setting_v1;
    struct cam_sensor_i2c_reg_setting_array gc02m0b_setting;
    struct cam_sensor_i2c_reg_setting_array ov02b10_setting;
    struct cam_sensor_i2c_reg_setting_array hi846_setting;
    struct cam_sensor_i2c_reg_setting_array ov64b_setting;
	struct cam_sensor_i2c_reg_setting_array s5kgw1_setting;
    struct cam_sensor_i2c_reg_setting_array gc02m1b_setting;
};

struct cam_sensor_settings sensor_settings = {
#include "CAM_SENSOR_SETTINGS.h"
};

/* Add for AT camera test */
long oplus_cam_sensor_subdev_ioctl(struct v4l2_subdev *sd,
	unsigned int cmd, void *arg, unsigned int *is_ftm_current_test)
{
	int rc = 0;
	struct cam_sensor_ctrl_t *s_ctrl =
		v4l2_get_subdevdata(sd);

    struct cam_sensor_i2c_reg_setting sensor_setting;
	switch (cmd) {

	case VIDIOC_CAM_SENSOR_STATR:
		rc = cam_sensor_start(s_ctrl);
		break;
	case VIDIOC_CAM_SENSOR_STOP:
		rc = cam_sensor_stop(s_ctrl);
		break;

	case VIDIOC_CAM_FTM_POWNER_DOWN:

		CAM_ERR(CAM_SENSOR, "FTM stream off");
		rc = cam_sensor_power_down(s_ctrl);
        CAM_ERR(CAM_SENSOR, "FTM power down.rc=%d, sensorid is %x",rc,s_ctrl->sensordata->slave_info.sensor_id);
		break;

	case VIDIOC_CAM_FTM_POWNER_UP:

		rc = cam_sensor_power_up(s_ctrl);
		CAM_ERR(CAM_SENSOR, "FTM power up sensor id 0x%x,result %d",s_ctrl->sensordata->slave_info.sensor_id,rc);
		if (rc < 0) {
			cam_sensor_power_down(s_ctrl);
			CAM_ERR(CAM_SENSOR, "FTM power up failed! Need to power down");
			break;
		}
        *is_ftm_current_test = 1;
		if (s_ctrl->sensordata->slave_info.sensor_id == 0x586) {
		    CAM_ERR(CAM_SENSOR, "FTM sensor setting 0x%x",s_ctrl->sensordata->slave_info.sensor_id);
		    sensor_setting.reg_setting = sensor_settings.imx586_setting0.reg_setting;
		    sensor_setting.addr_type = CAMERA_SENSOR_I2C_TYPE_WORD;
	            sensor_setting.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
		    sensor_setting.size = sensor_settings.imx586_setting0.size;
	            sensor_setting.delay = sensor_settings.imx586_setting0.delay;
                    rc = camera_io_dev_write(&(s_ctrl->io_master_info), &sensor_setting);
                    if (rc < 0) {
    		    /* If the I2C reg write failed for the first section reg, send
                    the result instead of keeping writing the next section of reg. */
                        CAM_ERR(CAM_SENSOR, "FTM Failed to write sensor setting 1/2");
                        break;
                    } else {
                        CAM_ERR(CAM_SENSOR, "FTM successfully to write sensor setting 1/2");
                    }
                    sensor_setting.reg_setting = sensor_settings.imx586_setting1.reg_setting;
                    sensor_setting.addr_type = CAMERA_SENSOR_I2C_TYPE_WORD;
                    sensor_setting.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
                    sensor_setting.size = sensor_settings.imx586_setting1.size;
                    sensor_setting.delay = sensor_settings.imx586_setting1.delay;
                    rc = camera_io_dev_write(&(s_ctrl->io_master_info), &sensor_setting);
                    if (rc < 0) {
                        CAM_ERR(CAM_SENSOR, "FTM Failed to write sensor setting 2/2");
                    } else {
                        CAM_ERR(CAM_SENSOR, "FTM successfully to write sensor setting 2/2");
                    }
		} else {
			if (s_ctrl->sensordata->slave_info.sensor_id == 0x686) {
			CAM_ERR(CAM_SENSOR, "FTM sensor setting 0x%x",s_ctrl->sensordata->slave_info.sensor_id);
				sensor_setting.reg_setting = sensor_settings.imx686_setting_part1.reg_setting;
				sensor_setting.addr_type = CAMERA_SENSOR_I2C_TYPE_WORD;
				sensor_setting.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
				sensor_setting.size = sensor_settings.imx686_setting_part1.size;
				sensor_setting.delay = sensor_settings.imx686_setting_part1.delay;
				rc = camera_io_dev_write(&(s_ctrl->io_master_info), &sensor_setting);
				if (rc < 0) {
					CAM_ERR(CAM_SENSOR, "FTM Failed to write sensor imx686 setting 1/2");
					break;
				} else {
					CAM_ERR(CAM_SENSOR, "FTM successfully to write sensor imx686 setting 1/2");
				}
				sensor_setting.reg_setting = sensor_settings.imx686_setting_part2.reg_setting;
				sensor_setting.addr_type = CAMERA_SENSOR_I2C_TYPE_WORD;
				sensor_setting.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
				sensor_setting.size = sensor_settings.imx686_setting_part2.size;
				sensor_setting.delay = sensor_settings.imx686_setting_part2.delay;
				rc = camera_io_dev_write(&(s_ctrl->io_master_info), &sensor_setting);
				if (rc < 0){
					CAM_ERR(CAM_SENSOR, "FTM Failed to write sensor imx686 setting 2/2");
					break;
				} else {
					CAM_ERR(CAM_SENSOR, "FTM successfully to write sensor imx686 setting 2/2");
				}
			} else if (s_ctrl->sensordata->slave_info.sensor_id == 0x766 ||
					s_ctrl->sensordata->slave_info.sensor_id == 0x766E ||
					s_ctrl->sensordata->slave_info.sensor_id == 0x766F) {
				CAM_ERR(CAM_SENSOR, "FTM sensor setting 0x%x",s_ctrl->sensordata->slave_info.sensor_id);
				sensor_setting.reg_setting = sensor_settings.imx766_setting.reg_setting;
				sensor_setting.addr_type = CAMERA_SENSOR_I2C_TYPE_WORD;
				sensor_setting.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
				sensor_setting.size = sensor_settings.imx766_setting.size;
				sensor_setting.delay = sensor_settings.imx766_setting.delay;
                                rc = camera_io_dev_write(&(s_ctrl->io_master_info), &sensor_setting);
			}else if (s_ctrl->sensordata->slave_info.sensor_id == 0x30d5) {
				CAM_ERR(CAM_SENSOR, "FTM sensor setting 0x%x",s_ctrl->sensordata->slave_info.sensor_id);
				sensor_setting.reg_setting = sensor_settings.s5k3m5_setting.reg_setting;
				sensor_setting.addr_type = CAMERA_SENSOR_I2C_TYPE_WORD;
				sensor_setting.data_type = CAMERA_SENSOR_I2C_TYPE_WORD;
				sensor_setting.size = sensor_settings.s5k3m5_setting.size;
				sensor_setting.delay = sensor_settings.s5k3m5_setting.delay;
                                rc = camera_io_dev_write(&(s_ctrl->io_master_info), &sensor_setting);
			} else if (s_ctrl->sensordata->slave_info.sensor_id == 0x5035) {
				CAM_ERR(CAM_SENSOR, "FTM sensor setting 0x%x",s_ctrl->sensordata->slave_info.sensor_id);
				sensor_setting.reg_setting = sensor_settings.gc5035_setting.reg_setting;
				sensor_setting.addr_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
				sensor_setting.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
				sensor_setting.size = sensor_settings.gc5035_setting.size;
				sensor_setting.delay = sensor_settings.gc5035_setting.delay;
                                rc = camera_io_dev_write(&(s_ctrl->io_master_info), &sensor_setting);
			} else if (s_ctrl->sensordata->slave_info.sensor_id == 0x471) {
				CAM_ERR(CAM_SENSOR, "FTM sensor setting 0x%x",s_ctrl->sensordata->slave_info.sensor_id);
				sensor_setting.reg_setting = sensor_settings.imx471_setting.reg_setting;
				sensor_setting.addr_type = CAMERA_SENSOR_I2C_TYPE_WORD;
				sensor_setting.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
				sensor_setting.size = sensor_settings.imx471_setting.size;
				sensor_setting.delay = sensor_settings.imx471_setting.delay;
                                rc = camera_io_dev_write(&(s_ctrl->io_master_info), &sensor_setting);
			} else if (s_ctrl->sensordata->slave_info.sensor_id == 0x481) {
				CAM_ERR(CAM_SENSOR, "FTM sensor setting 0x%x",s_ctrl->sensordata->slave_info.sensor_id);
				sensor_setting.reg_setting = sensor_settings.imx481_setting.reg_setting;
				sensor_setting.addr_type = CAMERA_SENSOR_I2C_TYPE_WORD;
				sensor_setting.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
				sensor_setting.size = sensor_settings.imx481_setting.size;
				sensor_setting.delay = sensor_settings.imx481_setting.delay;
                                rc = camera_io_dev_write(&(s_ctrl->io_master_info), &sensor_setting);
			} else if (s_ctrl->sensordata->slave_info.sensor_id == 0x689) {
				CAM_ERR(CAM_SENSOR, "FTM sensor setting 0x%x",s_ctrl->sensordata->slave_info.sensor_id);
				sensor_setting.reg_setting = sensor_settings.imx689_setting.reg_setting;
				sensor_setting.addr_type = CAMERA_SENSOR_I2C_TYPE_WORD;
				sensor_setting.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
				sensor_setting.size = sensor_settings.imx689_setting.size;
				sensor_setting.delay = sensor_settings.imx689_setting.delay;
                                rc = camera_io_dev_write(&(s_ctrl->io_master_info), &sensor_setting);
			} else if (s_ctrl->sensordata->slave_info.sensor_id == 0x2375) {
				CAM_ERR(CAM_SENSOR, "FTM sensor setting 0x%x",s_ctrl->sensordata->slave_info.sensor_id);
				sensor_setting.reg_setting = sensor_settings.gc2375_setting.reg_setting;
				sensor_setting.addr_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
				sensor_setting.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
				sensor_setting.size = sensor_settings.gc2375_setting.size;
				sensor_setting.delay = sensor_settings.gc2375_setting.delay;
                                rc = camera_io_dev_write(&(s_ctrl->io_master_info), &sensor_setting);
			} else if (s_ctrl->sensordata->slave_info.sensor_id == 0x0615) {
				CAM_ERR(CAM_SENSOR, "FTM sensor setting 0x%x",s_ctrl->sensordata->slave_info.sensor_id);
					sensor_setting.reg_setting = sensor_settings.imx615_setting_part1.reg_setting;
					sensor_setting.addr_type = CAMERA_SENSOR_I2C_TYPE_WORD;
					sensor_setting.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
					sensor_setting.size = sensor_settings.imx615_setting_part1.size;
					sensor_setting.delay = sensor_settings.imx615_setting_part1.delay;
					rc = camera_io_dev_write(&(s_ctrl->io_master_info), &sensor_setting);
					if (rc < 0) {
						CAM_ERR(CAM_SENSOR, "FTM Failed to write sensor imx615 setting 1/2");
						break;
					} else {
						CAM_ERR(CAM_SENSOR, "FTM successfully to write sensor imx615 setting 1/2");
					}
					sensor_setting.reg_setting = sensor_settings.imx615_setting_part2.reg_setting;
					sensor_setting.addr_type = CAMERA_SENSOR_I2C_TYPE_WORD;
					sensor_setting.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
					sensor_setting.size = sensor_settings.imx615_setting_part2.size;
					sensor_setting.delay = sensor_settings.imx615_setting_part2.delay;
					rc = camera_io_dev_write(&(s_ctrl->io_master_info), &sensor_setting);
					if (rc < 0){
						CAM_ERR(CAM_SENSOR, "FTM Failed to write sensor imx615 setting 2/2");
						break;
					} else {
						CAM_ERR(CAM_SENSOR, "FTM successfully to write sensor imx615 setting 2/2");
					}
			} else if (s_ctrl->sensordata->slave_info.sensor_id == 0x0616) {
				sensor_setting.reg_setting = sensor_settings.imx616_setting.reg_setting;
				sensor_setting.addr_type = CAMERA_SENSOR_I2C_TYPE_WORD;
				sensor_setting.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
				sensor_setting.size = sensor_settings.imx616_setting.size;
				sensor_setting.delay = sensor_settings.imx616_setting.delay;
				rc = camera_io_dev_write(&(s_ctrl->io_master_info), &sensor_setting);
				CAM_ERR(CAM_SENSOR,"FTM GET imx616 setting");
			} else if (s_ctrl->sensordata->slave_info.sensor_id == 0xf708
					|| s_ctrl->sensordata->slave_info.sensor_id == 0x0708) {
				sensor_setting.reg_setting = sensor_settings.imx708_setting.reg_setting;
				sensor_setting.addr_type = CAMERA_SENSOR_I2C_TYPE_WORD;
				sensor_setting.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
				sensor_setting.size = sensor_settings.imx708_setting.size;
				sensor_setting.delay = sensor_settings.imx708_setting.delay;
				CAM_ERR(CAM_SENSOR,"FTM GET imx708 setting_v0");
				rc = camera_io_dev_write(&(s_ctrl->io_master_info), &sensor_setting);
			} else if (s_ctrl->sensordata->slave_info.sensor_id == 0xe708) {
				sensor_setting.reg_setting = sensor_settings.imx708_setting_v1.reg_setting;
				sensor_setting.addr_type = CAMERA_SENSOR_I2C_TYPE_WORD;
				sensor_setting.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
				sensor_setting.size = sensor_settings.imx708_setting_v1.size;
				sensor_setting.delay = sensor_settings.imx708_setting_v1.delay;
				CAM_ERR(CAM_SENSOR,"FTM GET imx708 setting_v1");
				rc = camera_io_dev_write(&(s_ctrl->io_master_info), &sensor_setting);
			} else if(s_ctrl->sensordata->slave_info.sensor_id == 0x02d0) {
				sensor_setting.reg_setting = sensor_settings.gc02m0b_setting.reg_setting;
				sensor_setting.addr_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
				sensor_setting.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
				sensor_setting.size = sensor_settings.gc02m0b_setting.size;
				sensor_setting.delay = sensor_settings.gc02m0b_setting.delay;
				rc = camera_io_dev_write(&(s_ctrl->io_master_info), &sensor_setting);
				CAM_ERR(CAM_SENSOR,"FTM GET gc02m0b setting");
			} else if(s_ctrl->sensordata->slave_info.sensor_id == 0x4608) {
				sensor_setting.reg_setting = sensor_settings.hi846_setting.reg_setting;
				sensor_setting.addr_type = CAMERA_SENSOR_I2C_TYPE_WORD;
				sensor_setting.data_type = CAMERA_SENSOR_I2C_TYPE_WORD;
				sensor_setting.size = sensor_settings.hi846_setting.size;
				sensor_setting.delay = sensor_settings.hi846_setting.delay;
				rc = camera_io_dev_write(&(s_ctrl->io_master_info), &sensor_setting);
				CAM_ERR(CAM_SENSOR,"FTM GET HI846 setting");
			} else if (s_ctrl->sensordata->slave_info.sensor_id == 0x2b ||
				s_ctrl->sensordata->slave_info.sensor_id == 0x2b03) {
				sensor_setting.reg_setting = sensor_settings.ov02b10_setting.reg_setting;
				sensor_setting.addr_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
				sensor_setting.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
				sensor_setting.size = sensor_settings.ov02b10_setting.size;
				sensor_setting.delay = sensor_settings.ov02b10_setting.delay;
				rc = camera_io_dev_write(&(s_ctrl->io_master_info), &sensor_setting);
				CAM_ERR(CAM_SENSOR,"FTM GET gc02m0b setting");
			} else if (s_ctrl->sensordata->slave_info.sensor_id == 0x5664 ||
				s_ctrl->sensordata->slave_info.sensor_id == 0x6b64 ||
				s_ctrl->sensordata->slave_info.sensor_id == 0x5a64) {
				sensor_setting.reg_setting = sensor_settings.ov64b_setting.reg_setting;
				sensor_setting.addr_type = CAMERA_SENSOR_I2C_TYPE_WORD;
				sensor_setting.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
				sensor_setting.size = sensor_settings.ov64b_setting.size;
				sensor_setting.delay = sensor_settings.ov64b_setting.delay;
				rc = camera_io_dev_write(&(s_ctrl->io_master_info), &sensor_setting);
				CAM_ERR(CAM_SENSOR,"FTM GET ov64b setting");
			} else if (s_ctrl->sensordata->slave_info.sensor_id == 0x971) {
				sensor_setting.reg_setting = sensor_settings.s5kgw1_setting.reg_setting;
				sensor_setting.addr_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
				sensor_setting.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
				sensor_setting.size = sensor_settings.s5kgw1_setting.size;
				sensor_setting.delay = sensor_settings.s5kgw1_setting.delay;
				rc = camera_io_dev_write(&(s_ctrl->io_master_info), &sensor_setting);
				CAM_ERR(CAM_SENSOR, "FTM GET s5kgw1 setting");
			} else if (s_ctrl->sensordata->slave_info.sensor_id == 0x02e0) {
				sensor_setting.reg_setting = sensor_settings.gc02m1b_setting.reg_setting;
				sensor_setting.addr_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
				sensor_setting.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
				sensor_setting.size = sensor_settings.gc02m1b_setting.size;
				sensor_setting.delay = sensor_settings.gc02m1b_setting.delay;
				rc = camera_io_dev_write(&(s_ctrl->io_master_info), &sensor_setting);
				CAM_ERR(CAM_SENSOR,"FTM GET gc02m1b setting");
			} else {
				CAM_ERR(CAM_SENSOR, "FTM unknown sensor id 0x%x",s_ctrl->sensordata->slave_info.sensor_id);
				rc = -1;
			}
			if (rc < 0) {
				cam_sensor_power_down(s_ctrl);
				CAM_ERR(CAM_SENSOR, "FTM Failed to write sensor setting, and need to power down");
			} else {
				CAM_ERR(CAM_SENSOR, "FTM successfully to write sensor setting");
			}
		}
		break;
	default:
		CAM_ERR(CAM_SENSOR, "Invalid ioctl cmd: %d", cmd);
		break;
	}
	return rc;
}

