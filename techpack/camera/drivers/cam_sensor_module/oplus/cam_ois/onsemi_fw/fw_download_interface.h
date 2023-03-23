/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2020, Oplus. All rights reserved.
 */

#ifndef _DOWNLOAD_OIS_FW_H_
#define _DOWNLOAD_OIS_FW_H_

#include <linux/module.h>
#include <linux/firmware.h>
#include <cam_sensor_cmn_header.h>
#include "cam_ois_dev.h"
#include "cam_ois_core.h"
#include "cam_ois_soc.h"
#include "cam_sensor_util.h"
#include "cam_debug_util.h"
#include "cam_res_mgr_api.h"
#include "cam_common_util.h"

#include <linux/string.h>
#include <linux/time.h>
#include <linux/types.h>

//int RamWrite32A(uint32_t addr, uint32_t data);
//int RamRead32A(uint32_t addr, uint32_t* data);
int RamWrite32A_oplus(struct cam_ois_ctrl_t *o_ctrl, uint32_t addr, uint32_t data);
int RamRead32A_oplus(struct cam_ois_ctrl_t *o_ctrl, uint32_t addr, uint32_t* data);
int RamRead16A_oplus(struct cam_ois_ctrl_t *o_ctrl, uint32_t addr, uint32_t* data);

int DownloadFW(struct cam_ois_ctrl_t *o_ctrl);
void OISControl(struct cam_ois_ctrl_t *o_ctrl, void *arg);
void ReadOISHALLData(struct cam_ois_ctrl_t *o_ctrl, void *data);
void ReadOISHALLDataV2(struct cam_ois_ctrl_t *o_ctrl, void *data);
void ReadOISHALLDataV3(struct cam_ois_ctrl_t *o_ctrl, void *data);

bool IsOISReady(struct cam_ois_ctrl_t *o_ctrl);
void InitOIS(struct cam_ois_ctrl_t *o_ctrl);
void DeinitOIS(struct cam_ois_ctrl_t *o_ctrl);
void InitOISResource(struct cam_ois_ctrl_t *o_ctrl);
void CheckOISdata(void);
void CheckOISfwVersion(void);
int OISRamWriteWord(struct cam_ois_ctrl_t *ois_ctrl, uint32_t addr, uint32_t data);
int OISRead(struct cam_ois_ctrl_t *o_ctrl, uint32_t addr, uint32_t* data);
void forceExitpoll(struct cam_ois_ctrl_t *o_ctrl);
void WriteEEpromData(struct cam_write_eeprom_t *cam_write_eeprom);
void ReadEEpromData(struct cam_write_eeprom_t *cam_write_eeprom);
void Sem1215sReadOISHALLData(struct cam_ois_ctrl_t *o_ctrl, void *data);
void Sem1215sReadOISHALLDataV2(struct cam_ois_ctrl_t *o_ctrl, void *data);
int Initcheck128(struct cam_ois_ctrl_t *o_ctrl);
void set_ois_thread_status(int main_thread_status ,int tele_thread_status);


typedef struct {
	uint32_t			Index;
	uint8_t	            FWType;    // 1: Normal OIS FW, 2: Servo ON FW
	const uint8_t*		UpdataCode;
	uint32_t			SizeUpdataCode;
	uint64_t			SizeUpdataCodeCksm;
	const uint8_t*		FromCode;
	uint32_t			SizeFromCode;
	uint64_t			SizeFromCodeCksm;
	uint32_t			SizeFromCodeValid;
}	DOWNLOAD_TBL_EXT;

#endif
/* _DOWNLOAD_OIS_FW_H_ */

