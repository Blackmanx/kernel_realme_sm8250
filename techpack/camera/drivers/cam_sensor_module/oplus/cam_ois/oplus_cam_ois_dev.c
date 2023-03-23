// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2017-2020, The Linux Foundation. All rights reserved.
 * Copyright (c) 2020, Oplus. All rights reserved.
 */

#include "cam_ois_dev.h"
#include "cam_req_mgr_dev.h"
#include "cam_ois_soc.h"
#include "cam_ois_core.h"
#include "cam_debug_util.h"
#include <linux/dma-contiguous.h>

#ifndef OPLUS_FEATURE_CAMERA_COMMON
#define OPLUS_FEATURE_CAMERA_COMMON
#endif

#ifdef OPLUS_FEATURE_CAMERA_COMMON
#include "onsemi_fw/fw_download_interface.h"
#endif


void init_ois_hall_data(struct cam_ois_ctrl_t *o_ctrl)
{
	mutex_init(&(o_ctrl->ois_read_mutex));
	mutex_init(&(o_ctrl->ois_hall_data_mutex));
	mutex_init(&(o_ctrl->ois_poll_thread_mutex));

	o_ctrl->ois_poll_thread_control_cmd = 0;
	if (kfifo_alloc(&o_ctrl->ois_hall_data_fifo, SAMPLE_COUNT_IN_DRIVER*SAMPLE_SIZE_IN_DRIVER, GFP_KERNEL)) {
		CAM_ERR(CAM_OIS, "failed to init ois_hall_data_fifo");
	}

	if (kfifo_alloc(&o_ctrl->ois_hall_data_fifoV2, SAMPLE_COUNT_IN_DRIVER*SAMPLE_SIZE_IN_DRIVER, GFP_KERNEL)) {
		CAM_ERR(CAM_OIS, "failed to init ois_hall_data_fifoV2");
	}
	InitOISResource(o_ctrl);
}

void oplus_cam_ois_fw_init(struct cam_ois_ctrl_t *o_ctrl)
{
	o_ctrl->m_ois_fw_mode.ois_fw_prog_ptr = NULL;
	o_ctrl->m_ois_fw_mode.ois_fw_coeff_ptr = NULL;
	o_ctrl->m_ois_fw_mode.ois_fw_coeff_size = 0;
	o_ctrl->m_ois_fw_mode.ois_fw_prog_size = 0;
}

int oplus_cam_ois_deinit(struct cam_ois_ctrl_t *o_ctrl) {
	int ret = 0;
	if(o_ctrl->m_ois_fw_mode.ois_fw_prog_ptr != NULL) {
		cma_release(dev_get_cma_area((o_ctrl->soc_info.dev)),
			(struct page *)o_ctrl->m_ois_fw_mode.ois_fw_prog_ptr, o_ctrl->m_ois_fw_mode.ois_fw_prog_size);
		o_ctrl->m_ois_fw_mode.ois_fw_prog_ptr = NULL;
	}
	if(o_ctrl->m_ois_fw_mode.ois_fw_coeff_ptr != NULL) {
		cma_release(dev_get_cma_area((o_ctrl->soc_info.dev)),
			(struct page *)o_ctrl->m_ois_fw_mode.ois_fw_coeff_ptr, o_ctrl->m_ois_fw_mode.ois_fw_coeff_size);
		o_ctrl->m_ois_fw_mode.ois_fw_coeff_ptr = NULL;
	}
	return ret;
}

