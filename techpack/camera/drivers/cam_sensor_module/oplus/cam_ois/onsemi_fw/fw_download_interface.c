// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2020, Oplus. All rights reserved.
 */

#include <linux/kfifo.h>
#include <asm/arch_timer.h>
#include "fw_download_interface.h"
#include "LC898124/Ois.h"
#include "linux/proc_fs.h"

extern unsigned char SelectDownload(uint8_t GyroSelect, uint8_t ActSelect, uint8_t MasterSlave, uint8_t FWType);
extern uint8_t FlashDownload128( uint8_t ModuleVendor, uint8_t ActVer, uint8_t MasterSlave, uint8_t FWType);
extern uint8_t	LoadUserAreaToPM( void );
extern uint8_t	RdBurstUareaFromPm( uint32_t UlAddress, uint8_t *PucData , uint8_t UcLength , uint8_t mode );
extern uint8_t	RdSingleUareaFromPm( uint32_t UlAddress, uint8_t *PucData , uint8_t UcLength , uint8_t mode );
extern uint8_t	WrUareaToPm( uint32_t UlAddress, uint8_t *PucData , uint8_t UcLength , uint8_t mode );
extern uint8_t	WrUareaToPm( uint32_t UlAddress, uint8_t *PucData , uint8_t UcLength , uint8_t mode );
extern uint8_t	WrUareaToFlash(void);

#define MAX_DATA_NUM 64

static char ic_name_a[] = "lc898";
static char ic_name_b[] = "LC898";

struct mutex ois_mutex;
struct cam_ois_ctrl_t *ois_ctrl = NULL;
struct cam_ois_ctrl_t *ois_ctrls[CAM_OIS_TYPE_MAX] = {NULL};
enum cam_ois_state_vendor ois_state[CAM_OIS_TYPE_MAX] = {0};

#define OIS_REGISTER_SIZE 100
#define OIS_READ_REGISTER_DELAY 100
#define COMMAND_SIZE 255

struct dentry *ois_dentry = NULL;
bool dump_ois_registers = false;
static volatile int g_is_enable_main_ois_thread = 0;
static volatile int g_is_enable_tele_ois_thread = 0;


uint32_t ois_registers_124[OIS_REGISTER_SIZE][2] = {
	{0xF010, 0x0000},//Servo On/Off
	{0xF012, 0x0000},//Enable/Disable OIS
	{0xF013, 0x0000},//OIS Mode
	{0xF015, 0x0000},//Select Gyro vendor
	{0x82B8, 0x0000},//Gyro Gain X
	{0x8318, 0x0000},//Gyro Gain Y
	{0x0338, 0x0000},//Gyro Offset X
	{0x033c, 0x0000},//Gyro Offset Y
	{0x01C0, 0x0000},//Hall Offset X
	{0x0214, 0x0000},//Hall Offset Y
	{0x0310, 0x0000},//Gyro Raw Data X
	{0x0314, 0x0000},//Gyro Raw Data Y
	{0x0268, 0x0000},//Hall Raw Data X
	{0x026C, 0x0000},//Hall Raw Data Y
	{0xF100, 0x0000},//OIS status
	{0x0000, 0x0000},
};

uint32_t ois_registers_128[OIS_REGISTER_SIZE][2] = {
	{0xF010, 0x0000},//Servo On/Off
	{0xF012, 0x0000},//Enable/Disable OIS
	{0xF013, 0x0000},//OIS Mode
	{0xF015, 0x0000},//Select Gyro vendor
	{0x82B8, 0x0000},//Gyro Gain X
	{0x8318, 0x0000},//Gyro Gain Y
	{0x0240, 0x0000},//Gyro Offset X
	{0x0244, 0x0000},//Gyro Offset Y
	{0x00D8, 0x0000},//Hall Offset X
	{0x0128, 0x0000},//Hall Offset Y
	{0x0220, 0x0000},//Gyro Raw Data X
	{0x0224, 0x0000},//Gyro Raw Data Y
	{0x0178, 0x0000},//Hall Raw Data X
	{0x017C, 0x0000},//Hall Raw Data Y
	{0xF01D, 0x0000},//SPI IF read access command
	{0xF01E, 0x0000},//SPI IF Write access command
	{0xF100, 0x0000},//OIS status
	{0x0000, 0x0000},
};

static struct task_struct * g_ois_read_gyro_data_thread = NULL;
static bool g_enable_ois_read_gyro_data = 1;

int ois_read_gyro_data_thread(void *arg)
{
	int result;
	UINT16 addr_ary[] = {0x87F0,0x87F4,0x01F8,0x021C,0x0220,0x0224,0x0230,0x234,0x1F0,0x214,0x1E0,0x204,0x1D8,0x1FC,0x1F4,0x218,0x82A0,0x8300,0x8294,0x82F4,0x8370,0x8374,0x82B4,0x8314};
	UINT32 addr,value;
	int i =0;
	int arry_size =0;
	arry_size = sizeof(addr_ary) / sizeof(addr_ary[0]);
	CAM_ERR(CAM_OIS, "ois_read_gyro_data_thread start ");
	while(1) {
		if (ois_ctrls[CAM_OIS_MASTER]->cam_ois_state >= CAM_OIS_CONFIG) {
			for (i = 0; i < arry_size; i++) {
				addr = addr_ary[i] ;
				result = RamRead16A_oplus(ois_ctrls[CAM_OIS_MASTER], addr , &value);
				if (result < 0) {
					CAM_ERR(CAM_OIS, "read addr = 0x%x, value = 0x%x fail", addr, value);
				} else {
					CAM_ERR(CAM_OIS, "read addr = 0x%x, value = 0x%x success", addr, value);
				}
			}
		}
		usleep_range(20000-10, 20000);
	}
	CAM_ERR(CAM_OIS, "ois_read_gyro_data_thread end ");
}

static ssize_t ois_read(struct file *p_file,
	char __user *puser_buf, size_t count, loff_t *p_offset)
{

    return 0;
}

static ssize_t ois_write(struct file *p_file,
	const char __user *puser_buf,
	size_t count, loff_t *p_offset)
{
	char data[COMMAND_SIZE] = {0};
	char* const delim = " ";
	int iIndex = 0;
	char *token = NULL, *cur = NULL;
	uint32_t addr =0, value = 0;
	int result = 0;
	uint32_t read_data;
	if(puser_buf) {
		if(copy_from_user(&data, puser_buf, count)){
			CAM_ERR(CAM_OIS, "copy_from_user failed ,please check input parameter ");
		}
	}

	cur = data;
	while ((token = strsep(&cur, delim))) {
		//CAM_ERR(CAM_OIS, "string = %s iIndex = %d, count = %d", token, iIndex, count);
		if (iIndex  == 0) {
			if(kstrtoint(token, 16, &addr)){
				CAM_ERR(CAM_OIS, "check input parameter [addr] ");
				return 0;
			}
		} else if (iIndex == 1) {
		    if(kstrtoint(token, 16, &value)){
				CAM_ERR(CAM_OIS, "check input  parameter [value]");
				return 0;
			}
		}
		iIndex++;
	}
	if (ois_ctrls[CAM_OIS_MASTER] && addr != 0) {
		if (value == 0xFFFF){
			RamRead32A_oplus(ois_ctrls[CAM_OIS_MASTER], addr , &read_data);
			CAM_ERR(CAM_OIS, "read ois data addr = 0x%x, value = 0x%x ", addr, read_data);
		}else if (addr == 0xFFFF){
			g_enable_ois_read_gyro_data = value;
			CAM_ERR(CAM_OIS, "set thead status  addr = 0x%x, value = 0x%x ", addr, value);
		} else {
			result = RamWrite32A_oplus(ois_ctrls[CAM_OIS_MASTER], addr, value);
		}
		if (result < 0) {
			CAM_ERR(CAM_OIS, "write addr = 0x%x, value = 0x%x fail", addr, value);
		} else {
			CAM_INFO(CAM_OIS, "write addr = 0x%x, value = 0x%x success", addr, value);
		}
	}

	if (g_ois_read_gyro_data_thread == NULL )
	{
		g_ois_read_gyro_data_thread =
			 kthread_run(ois_read_gyro_data_thread, NULL, "ois_read_gyro_data_thread");

		CAM_ERR(CAM_OIS, "ois thread create success ");
	}
	return count;
}



static const struct file_operations proc_file_fops = {
	.owner = THIS_MODULE,
	.read  = ois_read,
	.write = ois_write,
};


int ois_start_read(void *arg, bool start)
{
	struct cam_ois_ctrl_t *o_ctrl = (struct cam_ois_ctrl_t *)arg;

	if (!o_ctrl) {
		CAM_ERR(CAM_OIS, "failed: o_ctrl %pK", o_ctrl);
		return -EINVAL;
	}

	mutex_lock(&(o_ctrl->ois_read_mutex));
	o_ctrl->ois_read_thread_start_to_read = start;
	mutex_unlock(&(o_ctrl->ois_read_mutex));

	msleep(OIS_READ_REGISTER_DELAY);

	return 0;
}

int ois_read_thread(void *arg)
{
	int rc = 0;
	int i;
	int data_size = 15;
	char buf[OIS_REGISTER_SIZE*16] = {0};

	struct cam_ois_ctrl_t *o_ctrl = (struct cam_ois_ctrl_t *)arg;

	if (!o_ctrl) {
		CAM_ERR(CAM_OIS, "failed: o_ctrl %pK", o_ctrl);
		return -EINVAL;
	}

	CAM_ERR(CAM_OIS, "ois_read_thread created");

	while (!kthread_should_stop()) {
		memset(buf, 0, sizeof(buf));
		mutex_lock(&(o_ctrl->ois_read_mutex));
		if (o_ctrl->ois_read_thread_start_to_read) {
			if (strstr(o_ctrl->ois_name, "124")) {
				for (i = 0; i < OIS_REGISTER_SIZE; i++) {
					if (ois_registers_124[i][0]) {
						ois_registers_124[i][1] = 0;
						camera_io_dev_read(&(o_ctrl->io_master_info), (uint32_t)ois_registers_124[i][0], (uint32_t *)&ois_registers_124[i][1],
						                   CAMERA_SENSOR_I2C_TYPE_WORD, CAMERA_SENSOR_I2C_TYPE_DWORD);
					}
				}

				for (i = 0; i < OIS_REGISTER_SIZE; i++) {
					if (ois_registers_124[i][0]) {
						snprintf(buf+(i*data_size), data_size, "0x%04x,0x%04x,", ois_registers_124[i][0], ois_registers_124[i][1]);
					}
				}
			} else if (strstr(o_ctrl->ois_name, "128")) {
				for (i = 0; i < OIS_REGISTER_SIZE; i++) {
					if (ois_registers_128[i][0]) {
						ois_registers_128[i][1] = 0;
						camera_io_dev_read(&(o_ctrl->io_master_info), (uint32_t)ois_registers_128[i][0], (uint32_t *)&ois_registers_128[i][1],
						                   CAMERA_SENSOR_I2C_TYPE_WORD, CAMERA_SENSOR_I2C_TYPE_DWORD);
					}
				}

				for (i = 0; i < OIS_REGISTER_SIZE; i++) {
					if (ois_registers_128[i][0]) {
						snprintf(buf+(i*data_size), data_size, "0x%04x,0x%04x,", ois_registers_128[i][0], ois_registers_128[i][1]);
					}
				}
			}
			CAM_ERR(CAM_OIS, "%s OIS register data: %s", o_ctrl->ois_name, buf);
		}
		mutex_unlock(&(o_ctrl->ois_read_mutex));

		msleep(OIS_READ_REGISTER_DELAY);
	}

	CAM_ERR(CAM_OIS, "ois_read_thread exist");

	return rc;
}

int ois_start_read_thread(void *arg, bool start)
{
	struct cam_ois_ctrl_t *o_ctrl = (struct cam_ois_ctrl_t *)arg;

	if (!o_ctrl) {
		CAM_ERR(CAM_OIS, "o_ctrl is NULL");
		return -1;
	}

	mutex_lock(&(o_ctrl->ois_read_mutex));
	if (start) {
		if (o_ctrl->ois_read_thread) {
			CAM_ERR(CAM_OIS, "ois_read_thread is already created, no need to create again.");
		} else {
			o_ctrl->ois_read_thread = kthread_run(ois_read_thread, o_ctrl, o_ctrl->ois_name);
			if (!o_ctrl->ois_read_thread) {
				CAM_ERR(CAM_OIS, "create ois read thread failed");
				mutex_unlock(&(o_ctrl->ois_read_mutex));
				return -2;
			}
		}
	} else {
		if (o_ctrl->ois_read_thread) {
			o_ctrl->ois_read_thread_start_to_read = 0;
			kthread_stop(o_ctrl->ois_read_thread);
			o_ctrl->ois_read_thread = NULL;
		} else {
			CAM_ERR(CAM_OIS, "ois_read_thread is already stopped, no need to stop again.");
		}
	}
	mutex_unlock(&(o_ctrl->ois_read_mutex));

	return 0;
}

void WitTim( uint16_t time)
{
	msleep(time);
}

void CntRd(uint32_t addr, void *data, uint16_t size)
{
	int i = 0;
	int32_t rc = 0;
	int retry = 3;
	struct cam_ois_ctrl_t *o_ctrl = ois_ctrl;

	for(i = 0; i < retry; i++) {
		rc = camera_io_dev_read_seq(&(o_ctrl->io_master_info), addr, (uint8_t *)data,
		                            CAMERA_SENSOR_I2C_TYPE_WORD,
		                            CAMERA_SENSOR_I2C_TYPE_BYTE,
		                            size);
		if (rc < 0) {
			CAM_ERR(CAM_OIS, "Continue read failed, rc:%d, retry:%d", rc, i+1);
		} else {
			break;
		}
	}
}

void CntWrt(  void *register_data, uint16_t size)
{
	uint8_t *data = (uint8_t *)register_data;
	int32_t rc = 0;
	int i = 0;
	int reg_data_cnt = size - 1;
	int continue_cnt = 0;
	int retry = 3;
	static struct cam_sensor_i2c_reg_array *i2c_write_setting_gl = NULL;

	struct cam_ois_ctrl_t *o_ctrl = ois_ctrl;

	struct cam_sensor_i2c_reg_setting i2c_write;

	if (o_ctrl == NULL) {
		CAM_ERR(CAM_OIS, "Invalid Args");
		return;
	}

	if (i2c_write_setting_gl == NULL) {
		i2c_write_setting_gl = (struct cam_sensor_i2c_reg_array *)kzalloc(
		                           sizeof(struct cam_sensor_i2c_reg_array)*reg_data_cnt, GFP_KERNEL);
		if(!i2c_write_setting_gl) {
			CAM_ERR(CAM_OIS, "Alloc i2c_write_setting_gl failed");
			return;
		}
	}

	memset(i2c_write_setting_gl, 0, sizeof(struct cam_sensor_i2c_reg_array)*reg_data_cnt);

	for(i = 0; i< reg_data_cnt; i++) {
		if (i == 0) {
			i2c_write_setting_gl[continue_cnt].reg_addr = data[0];
			i2c_write_setting_gl[continue_cnt].reg_data = data[1];
			i2c_write_setting_gl[continue_cnt].delay = 0x00;
			i2c_write_setting_gl[continue_cnt].data_mask = 0x00;
		} else {
			i2c_write_setting_gl[continue_cnt].reg_data = data[i+1];
			i2c_write_setting_gl[continue_cnt].delay = 0x00;
			i2c_write_setting_gl[continue_cnt].data_mask = 0x00;
		}
		continue_cnt++;
	}
	i2c_write.reg_setting = i2c_write_setting_gl;
	i2c_write.size = continue_cnt;
	i2c_write.addr_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
	i2c_write.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
	i2c_write.delay = 0x00;

	for(i = 0; i < retry; i++) {
		rc = camera_io_dev_write_continuous(&(o_ctrl->io_master_info),
		                                    &i2c_write, 1);
		if (rc < 0) {
			CAM_ERR(CAM_OIS, "Continue write failed, rc:%d, retry:%d", rc, i+1);
		} else {
			break;
		}
	}

	if (i2c_write_setting_gl != NULL) {
		kfree(i2c_write_setting_gl);
		i2c_write_setting_gl = NULL;
	}
}


int RamWrite32A(    uint32_t addr, uint32_t data)
{
	int32_t rc = 0;
	int retry = 3;
	int i;

	struct cam_ois_ctrl_t *o_ctrl = ois_ctrl;

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
		.data_type = CAMERA_SENSOR_I2C_TYPE_DWORD,
		.delay = 0x00,
	};

	if (o_ctrl == NULL) {
		CAM_ERR(CAM_OIS, "Invalid Args");
		return -EINVAL;
	}

	for(i = 0; i < retry; i++) {
		rc = camera_io_dev_write(&(o_ctrl->io_master_info), &i2c_write);
		if (rc < 0) {
			CAM_ERR(CAM_OIS, "write 0x%04x failed, retry:%d", addr, i+1);
		} else {
			return rc;
		}
	}
	return rc;
}

int RamRead32A(    uint32_t addr, uint32_t* data)
{
	int32_t rc = 0;
	int retry = 3;
	int i;

	struct cam_ois_ctrl_t *o_ctrl = ois_ctrl;

	if (o_ctrl == NULL) {
		CAM_ERR(CAM_OIS, "Invalid Args");
		return -EINVAL;
	}
	for(i = 0; i < retry; i++) {
		rc = camera_io_dev_read(&(o_ctrl->io_master_info), (uint32_t)addr, (uint32_t *)data,
		                        CAMERA_SENSOR_I2C_TYPE_WORD, CAMERA_SENSOR_I2C_TYPE_DWORD);
		if (rc < 0) {
			CAM_ERR(CAM_OIS, "read 0x%04x failed, retry:%d", addr, i+1);
		} else {
			return rc;
		}
	}
	return rc;
}

int RamWrite32A_oplus(struct cam_ois_ctrl_t *o_ctrl, uint32_t addr, uint32_t data)
{
	int32_t rc = 0;
	int retry = 3;
	int i;

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
		.data_type = CAMERA_SENSOR_I2C_TYPE_DWORD,
		.delay = 0x00,
	};

	if (o_ctrl == NULL) {
		CAM_ERR(CAM_OIS, "Invalid Args");
		return -EINVAL;
	}

	for(i = 0; i < retry; i++) {
		rc = camera_io_dev_write(&(o_ctrl->io_master_info), &i2c_write);
		if (rc < 0) {
			CAM_ERR(CAM_OIS, "write 0x%04x failed, retry:%d", addr, i+1);
		} else {
			return rc;
		}
	}
	return rc;
}

int RamRead32A_oplus(struct cam_ois_ctrl_t *o_ctrl, uint32_t addr, uint32_t* data)
{
	int32_t rc = 0;
	int retry = 3;
	int i;

	if (o_ctrl == NULL) {
		CAM_ERR(CAM_OIS, "Invalid Args");
		return -EINVAL;
	}
	for(i = 0; i < retry; i++) {
		rc = camera_io_dev_read(&(o_ctrl->io_master_info), (uint32_t)addr, (uint32_t *)data,
		                        CAMERA_SENSOR_I2C_TYPE_WORD, CAMERA_SENSOR_I2C_TYPE_DWORD);
		if (rc < 0) {
			CAM_ERR(CAM_OIS, "read 0x%04x failed, retry:%d", addr, i+1);
		} else {
			return rc;
		}
	}
	return rc;
}


int RamRead16A_oplus(struct cam_ois_ctrl_t *o_ctrl, uint32_t addr, uint32_t* data)
{
	int32_t rc = 0;
	int retry = 3;
	int i;

	if (o_ctrl == NULL) {
		CAM_ERR(CAM_OIS, "Invalid Args");
		return -EINVAL;
	}
	for(i = 0; i < retry; i++) {
		rc = camera_io_dev_read(&(o_ctrl->io_master_info), (uint32_t)addr, (uint32_t *)data,
		                        CAMERA_SENSOR_I2C_TYPE_WORD, CAMERA_SENSOR_I2C_TYPE_WORD);
		if (rc < 0) {
			CAM_ERR(CAM_OIS, "read 0x%04x failed, retry:%d", addr, i+1);
		} else {
			return rc;
		}
	}
	return rc;
}


void OISCountinueRead(struct cam_ois_ctrl_t *o_ctrl, uint32_t addr, void *data, uint16_t size)
{
	int i = 0;
	int32_t rc = 0;
	int retry = 3;

	for(i = 0; i < retry; i++) {
		rc = camera_io_dev_read_seq(&(o_ctrl->io_master_info), addr, (uint8_t *)data,
		                            CAMERA_SENSOR_I2C_TYPE_WORD,
		                            CAMERA_SENSOR_I2C_TYPE_WORD,
		                            size);
		if (rc < 0) {
			CAM_ERR(CAM_OIS, "Continue read failed, rc:%d, retry:%d", rc, i+1);
		} else {
			break;
		}
	}
}

void OISCountinueWrite(  struct cam_ois_ctrl_t *o_ctrl, void *register_data, uint16_t size)
{
	uint32_t *data = (uint32_t *)register_data;
	int32_t rc = 0;
	int i = 0;
	int reg_data_cnt = size - 1;
	int continue_cnt = 0;
	int retry = 3;
	static struct cam_sensor_i2c_reg_array *i2c_write_setting_gl = NULL;

	struct cam_sensor_i2c_reg_setting i2c_write;

	if (o_ctrl == NULL) {
		CAM_ERR(CAM_OIS, "Invalid Args");
		return;
	}

	if (i2c_write_setting_gl == NULL) {
		i2c_write_setting_gl = (struct cam_sensor_i2c_reg_array *)kzalloc(
		                           sizeof(struct cam_sensor_i2c_reg_array)*MAX_DATA_NUM*2, GFP_KERNEL);
		if(!i2c_write_setting_gl) {
			CAM_ERR(CAM_OIS, "Alloc i2c_write_setting_gl failed");
			return;
		}
	}

	memset(i2c_write_setting_gl, 0, sizeof(struct cam_sensor_i2c_reg_array)*MAX_DATA_NUM*2);

	for(i = 0; i< reg_data_cnt; i++) {
		if (i == 0) {
			i2c_write_setting_gl[continue_cnt].reg_addr = data[0];
			i2c_write_setting_gl[continue_cnt].reg_data = data[1];
			i2c_write_setting_gl[continue_cnt].delay = 0x00;
			i2c_write_setting_gl[continue_cnt].data_mask = 0x00;
		} else {
			i2c_write_setting_gl[continue_cnt].reg_data = data[i+1];
			i2c_write_setting_gl[continue_cnt].delay = 0x00;
			i2c_write_setting_gl[continue_cnt].data_mask = 0x00;
		}
		continue_cnt++;
	}
	i2c_write.reg_setting = i2c_write_setting_gl;
	i2c_write.size = continue_cnt;
	i2c_write.addr_type = CAMERA_SENSOR_I2C_TYPE_WORD;
	i2c_write.data_type = CAMERA_SENSOR_I2C_TYPE_DWORD;
	i2c_write.delay = 0x00;

	for(i = 0; i < retry; i++) {
		rc = camera_io_dev_write_continuous(&(o_ctrl->io_master_info),
		                                    &i2c_write, 1);
		if (rc < 0) {
			CAM_ERR(CAM_OIS, "Continue write failed, rc:%d, retry:%d", rc, i+1);
		} else {
			break;
		}
	}

	if (i2c_write_setting_gl != NULL) {
		kfree(i2c_write_setting_gl);
		i2c_write_setting_gl = NULL;
	}
}

int OISWrite(struct cam_ois_ctrl_t *o_ctrl, uint32_t addr, uint32_t data)
{
	int32_t rc = 0;
	int retry = 3;
	int i;

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
		.data_type = CAMERA_SENSOR_I2C_TYPE_DWORD,
		.delay = 0x00,
	};

	if (o_ctrl == NULL) {
		CAM_ERR(CAM_OIS, "Invalid Args");
		return -EINVAL;
	}

	for(i = 0; i < retry; i++) {
		rc = camera_io_dev_write(&(o_ctrl->io_master_info), &i2c_write);
		if (rc < 0) {
			CAM_ERR(CAM_OIS, "write 0x%04x failed, retry:%d", addr, i+1);
		} else {
			return rc;
		}
	}
	return rc;
}

int OISRead(struct cam_ois_ctrl_t *o_ctrl, uint32_t addr, uint32_t* data)
{
	int32_t rc = 0;
	int retry = 3;
	int i;

	if (o_ctrl == NULL) {
		CAM_ERR(CAM_OIS, "Invalid Args");
		return -EINVAL;
	}
	for(i = 0; i < retry; i++) {
		rc = camera_io_dev_read(&(o_ctrl->io_master_info), (uint32_t)addr, (uint32_t *)data,
		                        CAMERA_SENSOR_I2C_TYPE_WORD, CAMERA_SENSOR_I2C_TYPE_DWORD);
		if (rc < 0) {
			CAM_ERR(CAM_OIS, "read 0x%04x failed, retry:%d", addr, i+1);
		} else {
			return rc;
		}
	}
	return rc;
}

void Set124Or128GyroAccelCoef(struct cam_ois_ctrl_t *o_ctrl)
{
	CAM_ERR(CAM_OIS, "SetGyroAccelCoef SelectAct 0x%x GyroPostion 0x%x\n", o_ctrl->ois_actuator_vendor, o_ctrl->ois_gyro_position);

	if (strstr(o_ctrl->ois_name, "124")) {
		if(o_ctrl->ois_gyro_position==3) {
			RamWrite32A( GCNV_XX, (UINT32) 0x00000000);
			RamWrite32A( GCNV_XY, (UINT32) 0x80000001);
			RamWrite32A( GCNV_YY, (UINT32) 0x00000000);
			RamWrite32A( GCNV_YX, (UINT32) 0x7FFFFFFF);
			RamWrite32A( GCNV_ZP, (UINT32) 0x7FFFFFFF);

			RamWrite32A( ACNV_XX, (UINT32) 0x00000000);
			RamWrite32A( ACNV_XY, (UINT32) 0x7FFFFFFF);
			RamWrite32A( ACNV_YY, (UINT32) 0x00000000);
			RamWrite32A( ACNV_YX, (UINT32) 0x7FFFFFFF);
			RamWrite32A( ACNV_ZP, (UINT32) 0x80000001);
		} else if(o_ctrl->ois_gyro_position==2) {
			RamWrite32A( GCNV_XX, (UINT32) 0x00000000);
			RamWrite32A( GCNV_XY, (UINT32) 0x7FFFFFFF);
			RamWrite32A( GCNV_YY, (UINT32) 0x00000000);
			RamWrite32A( GCNV_YX, (UINT32) 0x7FFFFFFF);
			RamWrite32A( GCNV_ZP, (UINT32) 0x7FFFFFFF);

			RamWrite32A( ACNV_XX, (UINT32) 0x00000000);
			RamWrite32A( ACNV_XY, (UINT32) 0x7FFFFFFF);
			RamWrite32A( ACNV_YY, (UINT32) 0x00000000);
			RamWrite32A( ACNV_YX, (UINT32) 0x7FFFFFFF);
			RamWrite32A( ACNV_ZP, (UINT32) 0x80000001);
		}else if(o_ctrl->ois_gyro_position==4) {
			RamWrite32A( GCNV_XX, (UINT32) 0x00000000);
			RamWrite32A( GCNV_XY, (UINT32) 0x80000001);
			RamWrite32A( GCNV_YY, (UINT32) 0x00000000);
			RamWrite32A( GCNV_YX, (UINT32) 0x80000001);
			RamWrite32A( GCNV_ZP, (UINT32) 0x7FFFFFFF);

			RamWrite32A( ACNV_XX, (UINT32) 0x00000000);
			RamWrite32A( ACNV_XY, (UINT32) 0x7FFFFFFF);
			RamWrite32A( ACNV_YY, (UINT32) 0x00000000);
			RamWrite32A( ACNV_YX, (UINT32) 0x7FFFFFFF);
			RamWrite32A( ACNV_ZP, (UINT32) 0x80000001);
		}
	} else if (strstr(o_ctrl->ois_name, "128")) {

	}
}


static int Download124Or128FW(struct cam_ois_ctrl_t *o_ctrl)
{
	uint32_t UlReadValX, UlReadValY;
	uint32_t spi_type;
	unsigned char rc = 0;
	struct timespec mStartTime, mEndTime, diff;
	uint64_t mSpendTime = 0;

	if (!o_ctrl) {
		CAM_ERR(CAM_OIS, "Invalid Args");
		return -EINVAL;
	}

	ois_ctrl = o_ctrl;

	getnstimeofday(&mStartTime);

	CAM_INFO(CAM_OIS, "MasterSlave 0x%x, GyroVendor 0x%x, GyroPosition 0x%x, ModuleVendor 0x%x, ActVer 0x%x, FWType 0x%x\n",
	         o_ctrl->ois_type, o_ctrl->ois_gyro_vendor, o_ctrl->ois_gyro_position, o_ctrl->ois_module_vendor, o_ctrl->ois_actuator_vendor, o_ctrl->ois_fw_flag);

	if (strstr(o_ctrl->ois_name, "124")) {
		rc = SelectDownload(o_ctrl->ois_gyro_vendor, o_ctrl->ois_actuator_vendor, o_ctrl->ois_type, o_ctrl->ois_fw_flag);

		if (0 == rc) {
			Set124Or128GyroAccelCoef(ois_ctrl);

			//remap
			RamWrite32A(0xF000, 0x00000000 );
			//msleep(120);

			//SPI-Master ( Act1 )  Check gyro signal
			RamRead32A(0x061C, & UlReadValX );
			RamRead32A(0x0620, & UlReadValY );
			CAM_INFO(CAM_OIS, "Gyro_X:0x%x, Gyro_Y:0x%x", UlReadValX, UlReadValY);

			spi_type = 0;
			RamRead32A(0xf112, & spi_type );
			CAM_INFO(CAM_OIS, "spi_type:0x%x", spi_type);

			//SPI-Master ( Act1 )  Check gyro gain
			RamRead32A(0x82b8, & UlReadValX );
			RamRead32A(0x8318, & UlReadValY );
			CAM_INFO(CAM_OIS, "Gyro_gain_X:0x%x, Gyro_gain_Y:0x%x", UlReadValX, UlReadValY);

			//SPI-Master ( Act1 )  start gyro signal transfer. ( from Master to slave. )
			if (CAM_OIS_MASTER == o_ctrl->ois_type) {
				RamWrite32A(0x8970, 0x00000001 );
				//msleep(5);
				RamWrite32A(0xf111, 0x00000001 );
				//msleep(5);
			}
		} else {
			switch (rc) {
			case 0x01:
				CAM_ERR(CAM_OIS, "H/W error");
				break;
			case 0x02:
				CAM_ERR(CAM_OIS, "Table Data & Program download verify error");
				break;
			case 0xF0:
				CAM_ERR(CAM_OIS, "Download code select error");
				break;
			case 0xF1:
				CAM_ERR(CAM_OIS, "Download code information read error");
				break;
			case 0xF2:
				CAM_ERR(CAM_OIS, "Download code information disagreement");
				break;
			case 0xF3:
				CAM_ERR(CAM_OIS, "Download code version error");
				break;
			default:
				CAM_ERR(CAM_OIS, "Unkown error code");
				break;
			}
		}
	} else if (strstr(o_ctrl->ois_name, "128")) {
		rc = FlashDownload128(o_ctrl->ois_module_vendor, o_ctrl->ois_actuator_vendor, o_ctrl->ois_type, o_ctrl->ois_fw_flag);

		if (0 == rc) {
			Set124Or128GyroAccelCoef(ois_ctrl);

			//LC898128 don't need to do remap
			//RamWrite32A(0xF000, 0x00000000 );
			//msleep(120);
#if 0
			//select gyro vendor
			RamWrite32A(0xF015, o_ctrl->ois_gyro_vendor);
			msleep(10);

			//SPI-Master ( Act1 )  Check gyro signal
			RamRead32A(0x0220, & UlReadValX );
			RamRead32A(0x0224, & UlReadValY );
			CAM_INFO(CAM_OIS, "Gyro_X:0x%x, Gyro_Y:0x%x", UlReadValX, UlReadValY);

			spi_type = 0;
			RamRead32A(0xf112, & spi_type );
			CAM_INFO(CAM_OIS, "spi_type:0x%x", spi_type);

			//SPI-Master ( Act1 )  Check gyro gain
			RamRead32A(0x82b8, & UlReadValX );
			RamRead32A(0x8318, & UlReadValY );
			CAM_INFO(CAM_OIS, "Gyro_gain_X:0x%x, Gyro_gain_Y:0x%x", UlReadValX, UlReadValY);

			//SPI-Master ( Act1 )  start gyro signal transfer. ( from Master to slave. )
			//if (CAM_OIS_MASTER == o_ctrl->ois_type) {
			//	RamWrite32A(0xF017, 0x01);
			//}
#endif
		} else {
			switch (rc&0xF0) {
			case 0x00:
				CAM_ERR(CAM_OIS, "Error ; during the rom boot changing. Also including 128 power off issue.");
				break;
			case 0x20:
				CAM_ERR(CAM_OIS, "Error ; during Initial program for updating to program memory.");
				break;
			case 0x30:
				CAM_ERR(CAM_OIS, "Error ; during User Mat area erasing.");
				break;
			case 0x40:
				CAM_ERR(CAM_OIS, "Error ; during User Mat area programing.");
				break;
			case 0x50:
				CAM_ERR(CAM_OIS, "Error ; during the verification.");
				break;
			case 0x90:
				CAM_ERR(CAM_OIS, "Error ; during the drive offset confirmation.");
				break;
			case 0xA0:
				CAM_ERR(CAM_OIS, "Error ; during the MAT2 re-write process.");
				break;
			case 0xF0:
				if (rc == 0xF0)
					CAM_ERR(CAM_OIS, "mistake of module vendor designation.");
				else if (rc == 0xF1)
					CAM_ERR(CAM_OIS, "mistake size of From Code.");
				break;
			default:
				CAM_ERR(CAM_OIS, "Unkown error code");
				break;
			}
		}
	} else {
		CAM_ERR(CAM_OIS, "Unsupported OIS");
	}
	getnstimeofday(&mEndTime);
	diff = timespec_sub(mEndTime, mStartTime);
	mSpendTime = (timespec_to_ns(&diff))/1000000;

	CAM_INFO(CAM_OIS, "cam_ois_fw_download rc=%d, (Spend: %d ms)", rc, mSpendTime);

	return 0;
}


int Initcheck128(struct cam_ois_ctrl_t *o_ctrl)
{
	uint32_t UlReadValX, UlReadValY;
	uint32_t spi_type;

	//select gyro vendor
	RamWrite32A(0xF015, o_ctrl->ois_gyro_vendor);
	msleep(20);

	//SPI-Master ( Act1 )  Check gyro signal
	RamRead32A(0x0220, &UlReadValX );
	RamRead32A(0x0224, &UlReadValY );
	CAM_INFO(CAM_OIS, "Gyro_X:0x%x, Gyro_Y:0x%x", UlReadValX, UlReadValY);

	spi_type = 0;
	RamRead32A(0xf112, &spi_type );
	CAM_INFO(CAM_OIS, "spi_type:0x%x", spi_type);

	//SPI-Master ( Act1 )  Check gyro gain
	RamRead32A(0x82b8, &UlReadValX );
	RamRead32A(0x8318, &UlReadValY );
	CAM_INFO(CAM_OIS, "Gyro_gain_X:0x%x, Gyro_gain_Y:0x%x", UlReadValX, UlReadValY);
	return 0;
}


int DownloadFW(struct cam_ois_ctrl_t *o_ctrl)
{
	uint8_t rc = 0;

	if (o_ctrl) {
		if (strstr(o_ctrl->ois_name, ic_name_a) == NULL
			&& strstr(o_ctrl->ois_name, ic_name_b) == NULL) {
			return 0;
		}
		mutex_lock(&ois_mutex);

		if (CAM_OIS_INVALID == ois_state[o_ctrl->ois_type]) {

			if (CAM_OIS_MASTER == o_ctrl->ois_type) {
				rc = Download124Or128FW(ois_ctrls[CAM_OIS_MASTER]);
				if (rc) {
					CAM_ERR(CAM_OIS, "Download %s FW failed", o_ctrl->ois_name);
				} else {
					if (dump_ois_registers && !ois_start_read_thread(ois_ctrls[CAM_OIS_MASTER], 1)) {
						ois_start_read(ois_ctrls[CAM_OIS_MASTER], 1);
					}
				}
			} else if (CAM_OIS_SLAVE == o_ctrl->ois_type) {
				if (CAM_OIS_INVALID == ois_state[CAM_OIS_MASTER]) {
					rc = Download124Or128FW(ois_ctrls[CAM_OIS_MASTER]);
					if (!rc&&dump_ois_registers&&!ois_start_read_thread(ois_ctrls[CAM_OIS_MASTER], 1)) {
						ois_start_read(ois_ctrls[CAM_OIS_MASTER], 1);
					}
					msleep(120);// Need to check whether we need a long time delay
				}
				if (rc) {
					CAM_ERR(CAM_OIS, "Download %s FW failed", ois_ctrls[CAM_OIS_MASTER]->ois_name);
				} else {
					rc = Download124Or128FW(ois_ctrls[CAM_OIS_SLAVE]);
					if (rc) {
						CAM_ERR(CAM_OIS, "Download %s FW failed", o_ctrl->ois_name);
					} else {
						if (dump_ois_registers&&!ois_start_read_thread(ois_ctrls[CAM_OIS_SLAVE], 1)) {
							ois_start_read(ois_ctrls[CAM_OIS_SLAVE], 1);
						}
					}
				}
			}
			ois_state[o_ctrl->ois_type] = CAM_OIS_FW_DOWNLOADED;
		} else {
			CAM_ERR(CAM_OIS, "OIS state 0x%x is wrong", ois_state[o_ctrl->ois_type]);
		}
		mutex_unlock(&ois_mutex);
	} else {
		CAM_ERR(CAM_OIS, "o_ctrl is NULL");
	}

	return rc;
}

int OISPollThread124(void *arg)
{
#define SAMPLE_COUNT_IN_OIS_124 7
#define SAMPLE_INTERVAL     4000
	int32_t i = 0;
	uint32_t *data = NULL;
	uint32_t kfifo_in_len = 0;
	uint32_t fifo_size_in_ois = SAMPLE_COUNT_IN_OIS_124*SAMPLE_SIZE_IN_DRIVER;
	uint32_t fifo_size_in_driver = SAMPLE_COUNT_IN_DRIVER*SAMPLE_SIZE_IN_DRIVER;
	unsigned long long timestampQ = 0;

	struct cam_ois_ctrl_t *o_ctrl = (struct cam_ois_ctrl_t *)arg;
	uint32_t ois_hall_registers[SAMPLE_COUNT_IN_OIS_124] = {0x89C4, 0x89C0, 0x89BC, 0x89B8, 0x89B4, 0x89B0, 0x89AC};

	mutex_lock(&(o_ctrl->ois_hall_data_mutex));
	kfifo_reset(&(o_ctrl->ois_hall_data_fifo));
	mutex_unlock(&(o_ctrl->ois_hall_data_mutex));

	data = kzalloc(fifo_size_in_ois, GFP_KERNEL);
	if (!data) {
		CAM_ERR(CAM_OIS, "failed to kzalloc");
		return -1;
	}

	CAM_DBG(CAM_OIS, "OISPollThread124 creat");

	while(1) {
		mutex_lock(&(o_ctrl->ois_poll_thread_mutex));
		if (o_ctrl->ois_poll_thread_exit) {
			mutex_unlock(&(o_ctrl->ois_poll_thread_mutex));
			goto exit;
		}
		mutex_unlock(&(o_ctrl->ois_poll_thread_mutex));
		timestampQ = arch_counter_get_cntvct();
		//CAM_ERR(CAM_OIS, "trace timestamp:%lld in Qtime", timestampQ);

		memset(data, 0, fifo_size_in_ois);
		//Read OIS HALL data
		for (i = 0; i < SAMPLE_COUNT_IN_OIS_124; i++) {
			data[3*i] = timestampQ >> 32;
			data[3*i+1] = timestampQ & 0xFFFFFFFF;
			OISRead(o_ctrl, ois_hall_registers[i], &(data[3*i+2]));
			timestampQ -= 2*CLOCK_TICKCOUNT_MS;
		}

		for (i = SAMPLE_COUNT_IN_OIS_124 - 1; i >= 0; i--) {
			CAM_DBG(CAM_OIS, "OIS HALL data %lld (0x%x 0x%x)", ((uint64_t)data[3*i] << 32)+(uint64_t)data[3*i+1], data[3*i+2]&0xFFFF0000>>16, data[3*i+2]&0xFFFF);
		}

		mutex_lock(&(o_ctrl->ois_hall_data_mutex));
		if ((kfifo_len(&(o_ctrl->ois_hall_data_fifo)) + fifo_size_in_ois) > fifo_size_in_driver) {
			CAM_DBG(CAM_OIS, "ois_hall_data_fifo is full, fifo size %d, file len %d, will reset FIFO", kfifo_size(&(o_ctrl->ois_hall_data_fifo)), kfifo_len(&(o_ctrl->ois_hall_data_fifo)));
			kfifo_reset(&(o_ctrl->ois_hall_data_fifo));
		}

		if ((kfifo_len(&(o_ctrl->ois_hall_data_fifo)) + fifo_size_in_ois) <= fifo_size_in_driver) {
			kfifo_in_len = kfifo_in(&(o_ctrl->ois_hall_data_fifo), data, fifo_size_in_ois);
			if (kfifo_in_len != fifo_size_in_ois) {
				CAM_DBG(CAM_OIS, "kfifo_in %d Bytes, FIFO maybe full, some OIS Hall sample maybe dropped.", kfifo_in_len);
			} else {
				CAM_DBG(CAM_OIS, "kfifo_in %d Bytes", fifo_size_in_ois);
			}
		}
		mutex_unlock(&(o_ctrl->ois_hall_data_mutex));

		usleep_range(SAMPLE_COUNT_IN_OIS_124*SAMPLE_INTERVAL-5, SAMPLE_COUNT_IN_OIS_124*SAMPLE_INTERVAL);
	}

exit:
	kfree(data);
	CAM_DBG(CAM_OIS, "OISPollThread124 exit");
	return 0;
}

void forceExitpoll(struct cam_ois_ctrl_t *o_ctrl)
{
	if (o_ctrl) {
		if (strstr(o_ctrl->ois_name, ic_name_a) == NULL
			&& strstr(o_ctrl->ois_name, ic_name_b) == NULL
			&& strstr(o_ctrl->ois_name, "sem1215") == NULL
			&& strstr(o_ctrl->ois_name, "bu63169") == NULL) {
			return;
		}
	}
	g_enable_ois_read_gyro_data = 0;
	CAM_INFO(CAM_OIS, "++++:%s ois-name %s", __func__,o_ctrl->ois_name);
	o_ctrl->ois_poll_thread_exit = true;
	msleep(1);
}

int OISPollThread128(void *arg)
{
	uint32_t i = 0;
	uint32_t j = 0;

	uint32_t kfifo_in_len = 0;
	uint32_t fifo_size_in_ois = SAMPLE_COUNT_IN_OIS*SAMPLE_SIZE_IN_OIS;
	uint32_t fifo_size_in_ois_aligned = SAMPLE_COUNT_IN_OIS*SAMPLE_SIZE_IN_OIS_ALIGNED;
	uint32_t fifo_size_in_driver = SAMPLE_COUNT_IN_DRIVER*SAMPLE_SIZE_IN_DRIVER;
	uint16_t *p_hall_data_in_ois = NULL;
	struct cam_ois_hall_data_in_ois_aligned *p_hall_data_in_ois_aligned = NULL;
	struct cam_ois_hall_data_in_driver *p_hall_data_in_driver = NULL;
	struct cam_ois_ctrl_t *o_ctrl = (struct cam_ois_ctrl_t *)arg;

	uint64_t first_QTimer = 0;      // This will be used for the start QTimer to calculate the QTimer interval
	uint64_t prev_QTimer = 0;       // This is the last QTimer in the last CCI read
	uint64_t current_QTimer = 0;    // This will be used for the end QTimer to calculate the QTimer interval
	uint64_t interval_QTimer = 0;   // This is the QTimer interval between two sample

	uint64_t sample_offset = 0;     // This is timestamp offset between IC and system
	uint64_t readout_time = (((1+2+1+SAMPLE_SIZE_IN_OIS*SAMPLE_COUNT_IN_OIS)*8)/1000)*CLOCK_TICKCOUNT_MS;      // This is the time of CCI read
	uint16_t sample_count = 0;      // This is the sample count for one CCI read
	uint16_t sample_num = 0;        // This will be used to detect whehter some HALL data was dropped
	uint32_t total_sample_count = 0;// This will be used to calculate the QTimer interval
	uint16_t threshold = 2;         // This is the threshold to trigger Timestamp calibration, this means 2ms
	uint16_t tmp = 0;
	uint64_t real_QTimer;
	uint64_t real_QTimer_after;
	uint64_t i2c_read_offset;
	static uint64_t pre_real_QTimer = 0;
	static uint64_t pre_QTimer_offset =0 ;
	uint64_t estimate_QTimer = 0;	// This is the QTimer interval between two sample
	uint32_t vaild_cnt = 0;
	uint32_t is_add_Offset = 0;
	uint32_t offset_cnt;
	mutex_lock(&(o_ctrl->ois_hall_data_mutex));
	kfifo_reset(&(o_ctrl->ois_hall_data_fifo));
	kfifo_reset(&(o_ctrl->ois_hall_data_fifoV2));
	mutex_unlock(&(o_ctrl->ois_hall_data_mutex));

	p_hall_data_in_ois = kzalloc(fifo_size_in_ois, GFP_KERNEL);
	if (!p_hall_data_in_ois) {
		CAM_ERR(CAM_OIS, "failed to kzalloc p_hall_data_in_ois");
		return -1;
	}

	p_hall_data_in_ois_aligned = kzalloc(fifo_size_in_ois_aligned, GFP_KERNEL);
	if (!p_hall_data_in_ois_aligned) {
		CAM_ERR(CAM_OIS, "failed to kzalloc p_hall_data_in_ois_aligned");
		kfree(p_hall_data_in_ois);
		return -1;
	}

	p_hall_data_in_driver = kzalloc(SAMPLE_COUNT_IN_OIS*SAMPLE_SIZE_IN_DRIVER, GFP_KERNEL);
	if (!p_hall_data_in_driver) {
		CAM_ERR(CAM_OIS, "failed to kzalloc p_hall_data_in_driver");
		kfree(p_hall_data_in_ois);
		kfree(p_hall_data_in_ois_aligned);
		return -1;
	}

	CAM_INFO(CAM_OIS, "OISPollThread128 creat");

	RamWrite32A_oplus(o_ctrl,0xF110, 0x0);//Clear buffer to all "0" & enable buffer update function.

	while(1) {

		sample_count = 0;
		tmp = sample_num;
		memset(p_hall_data_in_ois, 0, fifo_size_in_ois);
		memset(p_hall_data_in_ois_aligned, 0, fifo_size_in_ois_aligned);
		memset(p_hall_data_in_driver, 0, SAMPLE_COUNT_IN_OIS*SAMPLE_SIZE_IN_DRIVER);

		usleep_range(13995, 14000);
		real_QTimer = arch_counter_get_cntvct();
		//Read OIS HALL data
		mutex_lock(&(o_ctrl->ois_poll_thread_mutex));

		if (o_ctrl->ois_poll_thread_exit) {
			mutex_unlock(&(o_ctrl->ois_poll_thread_mutex));
			goto exit;
		}
		OISCountinueRead(o_ctrl, 0xF111, (void *)p_hall_data_in_ois, fifo_size_in_ois);

		mutex_unlock(&(o_ctrl->ois_poll_thread_mutex));

		real_QTimer_after = arch_counter_get_cntvct();
		i2c_read_offset = real_QTimer_after - real_QTimer;
		//Covert the data from unaligned to aligned
		for(i = 0, j = 0; i < SAMPLE_COUNT_IN_OIS; i++) {
			if(((p_hall_data_in_ois[3*i] == 0) && (p_hall_data_in_ois[3*i+1] == 0) && (p_hall_data_in_ois[3*i+2] == 0)) || \
				(p_hall_data_in_ois[3*i] == OIS_MAGIC_NUMBER && p_hall_data_in_ois[3*i+1] == OIS_MAGIC_NUMBER)) {
				CAM_DBG(CAM_OIS, "OIS HALL RAW data %d %d (0x%x 0x%x)", i,
				        p_hall_data_in_ois[3*i],
				        p_hall_data_in_ois[3*i+1],
				        p_hall_data_in_ois[3*i+2]);
			} else {
				p_hall_data_in_ois_aligned[j].hall_data_cnt = p_hall_data_in_ois[3*i];
				p_hall_data_in_ois_aligned[j].hall_data = ((uint32_t)p_hall_data_in_ois[3*i+1] << 16) + p_hall_data_in_ois[3*i+2];
				CAM_DBG(CAM_OIS, "OIS HALL RAW data %d %d (0x%x 0x%x)", i,
				        p_hall_data_in_ois_aligned[j].hall_data_cnt,
				        p_hall_data_in_ois_aligned[j].hall_data&0xFFFF0000>>16,
				        p_hall_data_in_ois_aligned[j].hall_data&0xFFFF);
				j++;
			}
		}

		sample_offset = (uint64_t)((p_hall_data_in_ois[3*(SAMPLE_COUNT_IN_OIS-1)+2] & 0xFF) * CLOCK_TICKCOUNT_MS * 2 / OIS_MAX_COUNTER);

		if(first_QTimer == 0) {
			//Init some parameters
			for(i = 0; i < SAMPLE_COUNT_IN_OIS; i++) {
				if((p_hall_data_in_ois_aligned[i].hall_data == 0) && (p_hall_data_in_ois_aligned[i].hall_data_cnt == 0)) {
					break;
				}
			}
			if ((i >= 1) && (i <= SAMPLE_COUNT_IN_OIS)) {
				first_QTimer = arch_counter_get_cntvct() - readout_time - sample_offset;
				prev_QTimer = first_QTimer;
				sample_num = p_hall_data_in_ois_aligned[i-1].hall_data_cnt;
			}
			continue;
		} else {
			vaild_cnt = 0;
			current_QTimer = arch_counter_get_cntvct() - readout_time - sample_offset;
			//calculate sample_count and total_sample_count, and detect whether some hall data was dropped.
			for(i = 0; i < SAMPLE_COUNT_IN_OIS; i++) {
				if((p_hall_data_in_ois_aligned[i].hall_data != 0) || (p_hall_data_in_ois_aligned[i].hall_data_cnt != 0)) {
					total_sample_count++;
					sample_count++;
					while (++tmp != p_hall_data_in_ois_aligned[i].hall_data_cnt) {
						total_sample_count++;
						CAM_DBG(CAM_OIS, "One sample was droped, %d %d %d", i, tmp, p_hall_data_in_ois_aligned[i].hall_data_cnt);
					}
				}
			}
			if(sample_count > 0) {
				if (total_sample_count > 1) {
					interval_QTimer = (current_QTimer - first_QTimer)/(total_sample_count - 1);
				} else if(total_sample_count == 1) {
					interval_QTimer = threshold*CLOCK_TICKCOUNT_MS;
				}

				//Calculate the TS for every sample, if some sample were dropped, the TS of this sample will still be calculated, but will not report to UMD.
				for(i = 0; i < SAMPLE_COUNT_IN_OIS; i++) {
					if((p_hall_data_in_ois_aligned[i].hall_data != 0) || (p_hall_data_in_ois_aligned[i].hall_data_cnt != 0)) {
						if (i == 0) {
							//p_hall_data_in_driver[i].timestamp = prev_QTimer;
							estimate_QTimer = prev_QTimer;
							while (++sample_num != p_hall_data_in_ois_aligned[i].hall_data_cnt) {
								//p_hall_data_in_driver[i].timestamp += interval_QTimer;
								estimate_QTimer += interval_QTimer;
							}
							//p_hall_data_in_driver[i].timestamp += interval_QTimer;
							estimate_QTimer += interval_QTimer;

							p_hall_data_in_driver[i].high_dword = estimate_QTimer >> 32;
							p_hall_data_in_driver[i].low_dword  = estimate_QTimer & 0xFFFFFFFF;
							p_hall_data_in_driver[i].hall_data  = p_hall_data_in_ois_aligned[i].hall_data;
						} else {

							estimate_QTimer = ((uint64_t)p_hall_data_in_driver[i-1].high_dword << 32) + (uint64_t)p_hall_data_in_driver[i-1].low_dword;
							while (++sample_num != p_hall_data_in_ois_aligned[i].hall_data_cnt) {
								//p_hall_data_in_driver[i].timestamp += interval_QTimer;
								estimate_QTimer += interval_QTimer;
							}

							//p_hall_data_in_driver[i].timestamp += interval_QTimer;
							estimate_QTimer += interval_QTimer;
							p_hall_data_in_driver[i].high_dword = estimate_QTimer >> 32;
							p_hall_data_in_driver[i].low_dword  = estimate_QTimer & 0xFFFFFFFF;
							p_hall_data_in_driver[i].hall_data  = p_hall_data_in_ois_aligned[i].hall_data;
						}
						vaild_cnt ++ ;
					} else {
						break;
					}
				}

				if ((i >= 1) && (i <= SAMPLE_COUNT_IN_OIS)) {
					prev_QTimer = ((uint64_t)p_hall_data_in_driver[i-1].high_dword << 32) + (uint64_t)p_hall_data_in_driver[i-1].low_dword;
				}
				real_QTimer -= sample_offset;


				CAM_DBG(CAM_OIS, "OIS HALL data before %lld %lld",
							real_QTimer - pre_real_QTimer - (vaild_cnt-1) * interval_QTimer,
							pre_real_QTimer - (real_QTimer - (vaild_cnt-1) * interval_QTimer) );

				if ( pre_real_QTimer != 0 &&  vaild_cnt > 0 &&
					((real_QTimer - pre_real_QTimer - (vaild_cnt-1) * interval_QTimer <= CLOCK_TICKCOUNT_MS) ||
					 (pre_real_QTimer - (real_QTimer - (vaild_cnt-1) * interval_QTimer) <= CLOCK_TICKCOUNT_MS))) {
					real_QTimer += interval_QTimer;
				}

				for (i =0; i < 5; i++){
					if ( pre_real_QTimer != 0 &&  vaild_cnt > 0 &&
						((int64_t)(real_QTimer - pre_real_QTimer - (vaild_cnt-1) * interval_QTimer )< 0)) {
						real_QTimer += interval_QTimer;
						is_add_Offset = 1;
					}
				}

				if ( pre_real_QTimer != 0 &&  vaild_cnt > 0 &&
					((real_QTimer - pre_real_QTimer - (vaild_cnt-1) * interval_QTimer <= CLOCK_TICKCOUNT_MS) ||
					 (pre_real_QTimer - (real_QTimer - (vaild_cnt-1) * interval_QTimer) <= CLOCK_TICKCOUNT_MS))) {
					real_QTimer += interval_QTimer;

				}

				if ((pre_real_QTimer != 0)
					&& ((int64_t)(real_QTimer - pre_real_QTimer - (vaild_cnt-1) * interval_QTimer) > 42000
					|| (int64_t)(real_QTimer - pre_real_QTimer - (vaild_cnt-1) * interval_QTimer) < 34000)) {

					if (total_sample_count > 100 ) {
						real_QTimer =  pre_real_QTimer + vaild_cnt * interval_QTimer;
						CAM_ERR(CAM_OIS, "OIS HALL data force calate  %d ",offset_cnt);
						offset_cnt ++ ;
						if (offset_cnt > 3) {
							is_add_Offset = 1;
						}
					}
				} else {
						offset_cnt = 0;
				}

				CAM_DBG(CAM_OIS, "OIS HALL data after %lld  %lld",
							real_QTimer - pre_real_QTimer - (vaild_cnt-1) * interval_QTimer,
							pre_real_QTimer - (real_QTimer - (vaild_cnt-1) * interval_QTimer));

				pre_QTimer_offset = real_QTimer - pre_real_QTimer - (vaild_cnt-1) * interval_QTimer;

				for (i = 0; i < vaild_cnt ;i++){
					p_hall_data_in_driver[vaild_cnt - i -1].high_dword = real_QTimer >> 32;
					p_hall_data_in_driver[vaild_cnt - i -1].low_dword  = real_QTimer & 0xFFFFFFFF;
					real_QTimer -= interval_QTimer;
				}

				for ( i = 0; i < vaild_cnt;i++){
					CAM_DBG(CAM_OIS, "OIS HALL data %lld (0x%x 0x%x) pre :%lld offset reg:%d i2c_read_offset %lld",
						        ((uint64_t)p_hall_data_in_driver[i].high_dword << 32) + (uint64_t)p_hall_data_in_driver[i].low_dword,
						        (p_hall_data_in_driver[i].hall_data&0xFFFF0000)>>16,
						        p_hall_data_in_driver[i].hall_data&0xFFFF,
						        pre_real_QTimer,
						        (p_hall_data_in_ois[3*(SAMPLE_COUNT_IN_OIS-1)+2] & 0xFF),
						        i2c_read_offset);

				}
				if (!is_add_Offset){
					pre_real_QTimer = ((uint64_t)p_hall_data_in_driver[vaild_cnt -1].high_dword << 32) |
													(uint64_t)p_hall_data_in_driver[vaild_cnt -1].low_dword;
				} else {
					pre_real_QTimer = 0;
					is_add_Offset = 0;
				}
				//Do Timestamp calibration
				//Put the HALL data into the FIFO
				mutex_lock(&(o_ctrl->ois_hall_data_mutex));
				if ((kfifo_len(&(o_ctrl->ois_hall_data_fifo)) + vaild_cnt*SAMPLE_SIZE_IN_DRIVER) > fifo_size_in_driver) {
					CAM_DBG(CAM_OIS, "ois_hall_data_fifo is full, fifo size %d, file len %d, will reset FIFO",
					        kfifo_size(&(o_ctrl->ois_hall_data_fifo)),
					        kfifo_len(&(o_ctrl->ois_hall_data_fifo)));
					kfifo_reset(&(o_ctrl->ois_hall_data_fifo));
				}

				if ((kfifo_len(&(o_ctrl->ois_hall_data_fifoV2)) + vaild_cnt*SAMPLE_SIZE_IN_DRIVER) > fifo_size_in_driver) {
					CAM_DBG(CAM_OIS, "ois type=%d,ois_hall_data_fifoV2 is full, fifo size %d, file len %d, will reset FIFO",o_ctrl->ois_type,
					        kfifo_size(&(o_ctrl->ois_hall_data_fifoV2)),
					        kfifo_len(&(o_ctrl->ois_hall_data_fifoV2)));
					kfifo_reset(&(o_ctrl->ois_hall_data_fifoV2));
				}


				if ((kfifo_len(&(o_ctrl->ois_hall_data_fifo)) + vaild_cnt*SAMPLE_SIZE_IN_DRIVER) <= fifo_size_in_driver) {
					kfifo_in_len = kfifo_in(&(o_ctrl->ois_hall_data_fifo), p_hall_data_in_driver, vaild_cnt*SAMPLE_SIZE_IN_DRIVER);
					if (kfifo_in_len != vaild_cnt*SAMPLE_SIZE_IN_DRIVER) {
						CAM_DBG(CAM_OIS, "kfifo_in %d Bytes, FIFO maybe full, some OIS Hall sample maybe dropped.", vaild_cnt);
					} else {
						CAM_DBG(CAM_OIS, "kfifo_in %d Bytes", vaild_cnt*SAMPLE_SIZE_IN_DRIVER);
					}
				}

				//Store ois data for EISv2
				if ((kfifo_len(&(o_ctrl->ois_hall_data_fifoV2)) + vaild_cnt*SAMPLE_SIZE_IN_DRIVER) <= fifo_size_in_driver) {
					kfifo_in_len = kfifo_in(&(o_ctrl->ois_hall_data_fifoV2), p_hall_data_in_driver, vaild_cnt*SAMPLE_SIZE_IN_DRIVER);

					if (kfifo_in_len != vaild_cnt*SAMPLE_SIZE_IN_DRIVER) {
						CAM_DBG(CAM_OIS, "ois type=%d,kfifo_in %d Bytes, FIFOV2 maybe full, some OIS Hall sample maybe dropped.",o_ctrl->ois_type, kfifo_in_len);
					} else {
						CAM_DBG(CAM_OIS, "ois type=%d,kfifo_inV2 %d Bytes",o_ctrl->ois_type, vaild_cnt*SAMPLE_SIZE_IN_DRIVER);
					}
				}else{
					kfifo_out(&(o_ctrl->ois_hall_data_fifoV2), p_hall_data_in_driver, vaild_cnt*SAMPLE_SIZE_IN_DRIVER);
					kfifo_in_len = kfifo_in(&(o_ctrl->ois_hall_data_fifoV2), p_hall_data_in_driver, vaild_cnt*SAMPLE_SIZE_IN_DRIVER);
					if (kfifo_in_len != vaild_cnt*SAMPLE_SIZE_IN_DRIVER) {
						CAM_DBG(CAM_OIS, "ois type=%d,kfifo_in %d Bytes, FIFOV2 maybe full, some OIS Hall sample maybe dropped.",o_ctrl->ois_type, kfifo_in_len);
					} else {
						CAM_DBG(CAM_OIS, "ois type=%d,kfifo_inV2 %d Bytes",o_ctrl->ois_type, vaild_cnt*SAMPLE_SIZE_IN_DRIVER);
					}
					CAM_DBG(CAM_OIS, "ois type=%d,kfifo_in %d Bytes, FIFOV2 maybe full, some OIS Hall sample maybe dropped.",o_ctrl->ois_type, kfifo_in_len);

				}

				mutex_unlock(&(o_ctrl->ois_hall_data_mutex));
			}
		}
	}

exit:
	pre_real_QTimer = 0;
	is_add_Offset = 0;
	total_sample_count = 0;
	kfree(p_hall_data_in_ois);
	kfree(p_hall_data_in_ois_aligned);
	kfree(p_hall_data_in_driver);
	g_is_enable_tele_ois_thread = false;
	CAM_INFO(CAM_OIS, "OISPollThread128 exit");
	return 0;
}

int Sem1215sOISPollThread(void *arg)
{
#define SEM1215S_SAMPLE_COUNT_IN_OIS               1
#define SEM1215S_SAMPLE_INTERVAL                   4000

	int32_t i = 0;
	uint32_t *data = NULL;
	uint32_t data_x = 0;
	uint32_t data_y = 0;
	uint32_t kfifo_in_len = 0;
	uint32_t fifo_size_in_ois = SEM1215S_SAMPLE_COUNT_IN_OIS*OIS_HALL_SAMPLE_BYTE;
	uint32_t fifo_size_in_ois_driver = OIS_HALL_SAMPLE_COUNT*OIS_HALL_SAMPLE_BYTE;
	unsigned long long timestampQ = 0;

	struct cam_ois_ctrl_t *o_ctrl = (struct cam_ois_ctrl_t *)arg;
	uint32_t ois_hall_registers[SEM1215S_SAMPLE_COUNT_IN_OIS] = {0x1100};

	mutex_lock(&(o_ctrl->ois_hall_data_mutex));
	kfifo_reset(&(o_ctrl->ois_hall_data_fifo));
	mutex_unlock(&(o_ctrl->ois_hall_data_mutex));

	data = kzalloc(fifo_size_in_ois, GFP_KERNEL);
	if (!data) {
		CAM_ERR(CAM_OIS, "failed to kzalloc");
		return -1;
	}

	CAM_INFO(CAM_OIS, "Sem1215sOISPollThread creat");

	while(1) {
		mutex_lock(&(o_ctrl->ois_poll_thread_mutex));
		if (o_ctrl->ois_poll_thread_exit
            || false == g_is_enable_tele_ois_thread) {
			mutex_unlock(&(o_ctrl->ois_poll_thread_mutex));
			goto exit;
		}
		mutex_unlock(&(o_ctrl->ois_poll_thread_mutex));
		timestampQ = arch_counter_get_cntvct();
		//CAM_ERR(CAM_OIS, "trace timestamp:%lld in Qtime", timestampQ);

		memset(data, 0, fifo_size_in_ois);
		//Read OIS HALL data
		for (i = 0; i < 1; i++) {
			data[3*i] = timestampQ >> 32;
			data[3*i+1] = timestampQ & 0xFFFFFFFF;
			camera_io_dev_read(&(o_ctrl->io_master_info), (uint32_t)ois_hall_registers[i], &data_x,
		                        CAMERA_SENSOR_I2C_TYPE_WORD, CAMERA_SENSOR_I2C_TYPE_WORD);
			camera_io_dev_read(&(o_ctrl->io_master_info), (uint32_t)ois_hall_registers[i]+2, &data_y,
								CAMERA_SENSOR_I2C_TYPE_WORD, CAMERA_SENSOR_I2C_TYPE_WORD);
			data[3*i+2] = (data_x & 0xFFFF) | ((data_y & 0xFFFF) << 16);
			timestampQ += CLOCK_TICKCOUNT_MS * 4;
		}

		for (i = 0; i < 1 ; i++) {
			CAM_DBG(CAM_OIS, "OIS HALL data %lld (0x%x 0x%x)", ((uint64_t)data[3*i] << 32)+(uint64_t)data[3*i+1], (data[3*i+2]&0xFFFF0000)>>16, data[3*i+2]&0xFFFF);
		}

		mutex_lock(&(o_ctrl->ois_hall_data_mutex));
		if ((kfifo_len(&(o_ctrl->ois_hall_data_fifo)) + fifo_size_in_ois) > fifo_size_in_ois_driver) {
			CAM_DBG(CAM_OIS, "ois_hall_data_fifo is full, fifo size %d, file len %d, will reset FIFO", kfifo_size(&(o_ctrl->ois_hall_data_fifo)), kfifo_len(&(o_ctrl->ois_hall_data_fifo)));
			kfifo_reset(&(o_ctrl->ois_hall_data_fifo));
		}

		if ((kfifo_len(&(o_ctrl->ois_hall_data_fifo)) + fifo_size_in_ois) <= fifo_size_in_ois_driver) {
			kfifo_in_len = kfifo_in(&(o_ctrl->ois_hall_data_fifo), data, fifo_size_in_ois);
			if (kfifo_in_len != fifo_size_in_ois) {
				CAM_DBG(CAM_OIS, "kfifo_in %d Bytes, FIFO maybe full, some OIS Hall sample maybe dropped.", kfifo_in_len);
			} else {
				CAM_DBG(CAM_OIS, "kfifo_in %d Bytes", fifo_size_in_ois);
			}
		}

		mutex_unlock(&(o_ctrl->ois_hall_data_mutex));
		usleep_range(SEM1215S_SAMPLE_INTERVAL-5, SEM1215S_SAMPLE_INTERVAL);
	}

exit:
	kfree(data);
	o_ctrl->ois_poll_thread_exit = true;
	CAM_INFO(CAM_OIS, "Sem1215sOISPollThread exit");
	return 0;
}

#define OIS_HALL_DATA_SIZE   52
void timeval_add(struct timeval *tv,int32_t usec)
{
	if (usec > 0) {
		if (tv->tv_usec + usec >= 1000*1000) {
			tv->tv_sec += 1;
			tv->tv_usec = tv->tv_usec - 1000 * 1000 + usec;
		} else {
			tv->tv_usec += usec;
		}
	} else {
		if (tv->tv_usec < abs(usec)) {
			tv->tv_sec -= 1;
			tv->tv_usec = tv->tv_usec + 1000 * 1000 + usec;
		} else {
			tv->tv_usec += usec;
		}
	}
}
int OISPollThread169(void *arg)
{
	struct cam_ois_ctrl_t *o_ctrl                   = (struct cam_ois_ctrl_t *)arg;
	int32_t                i;
	int32_t                rc;
	uint8_t                data[OIS_HALL_DATA_SIZE];
	struct hall_info      *pHalldata;
	uint32_t               mHalldata_X, mHalldata_Y;
	uint8_t                dataNum;
	int                    offset;
	uint32_t               delayCount;
	uint32_t               delayTime;//usec
	struct timeval         endTime;
	struct timeval         estimateLastEndTime;
	struct timeval         LastPacketEndTime;
	int32_t                gap_usec;
	uint32_t               fifo_size_in_driver;
	uint32_t               kfifo_in_len;
	uint32_t               validSize;
	bool                   isFirstFrame;

	mutex_lock(&(o_ctrl->ois_hall_data_mutex));
	kfifo_reset(&(o_ctrl->ois_hall_data_fifo));
	kfifo_reset(&(o_ctrl->ois_hall_data_fifoV2));
	fifo_size_in_driver = kfifo_size(&(o_ctrl->ois_hall_data_fifo));
	mutex_unlock(&(o_ctrl->ois_hall_data_mutex));

	pHalldata = kzalloc(OIS_HALL_DATA_SIZE*sizeof(struct hall_info), GFP_KERNEL);
	if (!pHalldata) {
		CAM_ERR(CAM_OIS, "failed to kzalloc pHalldata");
		return -1;
	}
	isFirstFrame = true;
	do_gettimeofday(&LastPacketEndTime);
	CAM_INFO(CAM_OIS, "OISPollThread169 creat");
	while(1) {
		rc          = 0;
		offset      = 0;
		memset(data, 0, OIS_HALL_DATA_SIZE);
		memset(pHalldata,0,OIS_HALL_DATA_SIZE*sizeof(struct hall_info));

		usleep_range(15995, 16000);

		//Read OIS HALL data
		mutex_lock(&(o_ctrl->ois_poll_thread_mutex));
		if (o_ctrl->ois_poll_thread_exit) {
			mutex_unlock(&(o_ctrl->ois_poll_thread_mutex));
			goto exit;
		}
		rc = camera_io_dev_read_seq(&(o_ctrl->io_master_info), 0x8A, data,
		CAMERA_SENSOR_I2C_TYPE_BYTE, CAMERA_SENSOR_I2C_TYPE_BYTE, OIS_HALL_DATA_SIZE);
		mutex_unlock(&(o_ctrl->ois_poll_thread_mutex));
		do_gettimeofday(&endTime);

		if (rc < 0) {
			CAM_ERR(CAM_OIS, "get mutil hall data fail");
			continue;
		}


		//Covert the data from unaligned to aligned
		dataNum = data[0];
		offset++;
		if (dataNum <= 0 || dataNum > HALL_MAX_NUMBER) {
			CAM_ERR(CAM_OIS, "get a wrong number of hall data:%d", dataNum);
			continue;
		}
		for (i = 0; i < dataNum; i++) {
			pHalldata[i].mHalldata = ((data[offset+3] & 0x00FF)
				| ((data[offset+2] << 8) & 0xFF00)
				| ((data[offset+1] << 16) & 0x00FF0000)
				| ((data[offset] << 24) & 0xFF000000));
			offset += 4;
		}
		delayCount = ((uint32_t)(data[offset] << 8) | data[offset+1]);
		delayTime = (long)(delayCount * 1778) / 100;//usec


		//Do Timestamp calibration
		timeval_add(&endTime,-(delayTime));
		CAM_DBG(CAM_OIS,"end time afters: sec = %d, us = %d,dataNum=%d delayCount=%d,delayTime=%d",endTime.tv_sec,endTime.tv_usec,dataNum,delayCount,delayTime);
		//1.get gap_usec
		estimateLastEndTime.tv_sec = endTime.tv_sec;
		estimateLastEndTime.tv_usec = endTime.tv_usec;
		timeval_add(&estimateLastEndTime,-(long)(4000 * dataNum));
		if (dataNum == HALL_MAX_NUMBER || isFirstFrame) {
			LastPacketEndTime.tv_sec = estimateLastEndTime.tv_sec;
			LastPacketEndTime.tv_usec = estimateLastEndTime.tv_usec;
			isFirstFrame = false;
			CAM_DBG(CAM_OIS,"Data drop!!,reset LastPacketEndTime: sec = %d, us = %d",LastPacketEndTime.tv_sec,LastPacketEndTime.tv_usec);
		}
		gap_usec = 1000 * 1000 * (estimateLastEndTime.tv_sec - LastPacketEndTime.tv_sec) + estimateLastEndTime.tv_usec - LastPacketEndTime.tv_usec;
		CAM_DBG(CAM_OIS,"ois packet gap(usec)=%d Calculated last tv_sec=%d tv_usec=%d",gap_usec,estimateLastEndTime.tv_sec,estimateLastEndTime.tv_usec);
		//2.estimate LastEndTime
		estimateLastEndTime.tv_sec = LastPacketEndTime.tv_sec;
		estimateLastEndTime.tv_usec = LastPacketEndTime.tv_usec;
		if (abs(gap_usec) < 8000) {
			timeval_add(&estimateLastEndTime,(gap_usec >> 5));
		} else {
			timeval_add(&estimateLastEndTime,(gap_usec >> 2));
		}
		CAM_DBG(CAM_OIS,"Estimated last tv_sec=%d tv_usec=%d",estimateLastEndTime.tv_sec,estimateLastEndTime.tv_usec);


		//add Timestamp
		for (i = 0; i < dataNum; i++) {
			mHalldata_X = ((pHalldata[i].mHalldata >> 16) & 0xFFFF);
			mHalldata_Y = (pHalldata[i].mHalldata & 0xFFFF);
			timeval_add(&estimateLastEndTime,4000);
			pHalldata[i].timeStampSec = estimateLastEndTime.tv_sec;
			pHalldata[i].timeStampUsec = estimateLastEndTime.tv_usec;
			CAM_DBG(CAM_OIS, "camxhalldata[%d] X:0x%04x  halldataY:0x%04x  sec = %d, us = %d", i,
				mHalldata_X,
				mHalldata_Y,
				pHalldata[i].timeStampSec,
				pHalldata[i].timeStampUsec);
		}
		LastPacketEndTime.tv_sec = estimateLastEndTime.tv_sec;
		LastPacketEndTime.tv_usec = estimateLastEndTime.tv_usec;


		//Put the HALL data into the FIFO
		validSize = dataNum*sizeof(struct hall_info);
		mutex_lock(&(o_ctrl->ois_hall_data_mutex));
		if ((kfifo_len(&(o_ctrl->ois_hall_data_fifo)) + validSize) > fifo_size_in_driver) {
			CAM_DBG(CAM_OIS, "ois_hall_data_fifo is full, fifo size %d, file len %d, will reset FIFO",
			        kfifo_size(&(o_ctrl->ois_hall_data_fifo)),
			        kfifo_len(&(o_ctrl->ois_hall_data_fifo)));
			kfifo_reset(&(o_ctrl->ois_hall_data_fifo));
		}
		if ((kfifo_len(&(o_ctrl->ois_hall_data_fifoV2)) + validSize) > fifo_size_in_driver) {
			CAM_DBG(CAM_OIS, "ois type=%d,ois_hall_data_fifoV2 is full, fifo size %d, file len %d, will reset FIFO",o_ctrl->ois_type,
			        kfifo_size(&(o_ctrl->ois_hall_data_fifoV2)),
			        kfifo_len(&(o_ctrl->ois_hall_data_fifoV2)));
			kfifo_reset(&(o_ctrl->ois_hall_data_fifoV2));
		}
		if ((kfifo_len(&(o_ctrl->ois_hall_data_fifo)) + validSize) <= fifo_size_in_driver) {
			kfifo_in_len = kfifo_in(&(o_ctrl->ois_hall_data_fifo), pHalldata, validSize);
			if (kfifo_in_len != validSize) {
				CAM_DBG(CAM_OIS, "kfifo_in %d Bytes, FIFO maybe full, some OIS Hall sample maybe dropped.", kfifo_in_len);
			} else {
				CAM_DBG(CAM_OIS, "kfifo_in %d Bytes", validSize);
			}
		}
		if ((kfifo_len(&(o_ctrl->ois_hall_data_fifoV2)) + validSize) <= fifo_size_in_driver) {
			kfifo_in_len = kfifo_in(&(o_ctrl->ois_hall_data_fifoV2), pHalldata, validSize);

			if (kfifo_in_len != validSize) {
				CAM_DBG(CAM_OIS, "ois type=%d,kfifo_in %d Bytes, FIFOV2 maybe full, some OIS Hall sample maybe dropped.",o_ctrl->ois_type, kfifo_in_len);
			} else {
				CAM_DBG(CAM_OIS, "ois type=%d,kfifo_inV2 %d Bytes",o_ctrl->ois_type, validSize);
			}
		}
		mutex_unlock(&(o_ctrl->ois_hall_data_mutex));
	}
exit:
	kfree(pHalldata);
	o_ctrl->ois_poll_thread_exit = true;
	CAM_INFO(CAM_OIS, "OISPollThread169 exit");
	return 0;
}

void ReadOISHALLData(struct cam_ois_ctrl_t *o_ctrl, void *data)
{
	uint32_t fifo_len_in_ois_driver;
	uint32_t data_size = 0;

	mutex_lock(&(o_ctrl->ois_hall_data_mutex));
	fifo_len_in_ois_driver = kfifo_len(&(o_ctrl->ois_hall_data_fifo));
	if (fifo_len_in_ois_driver > 0) {
		if (fifo_len_in_ois_driver > SAMPLE_SIZE_IN_DRIVER*SAMPLE_COUNT_IN_DRIVER) {
			fifo_len_in_ois_driver = SAMPLE_SIZE_IN_DRIVER*SAMPLE_COUNT_IN_DRIVER;
		}
		kfifo_to_user(&(o_ctrl->ois_hall_data_fifo), data, fifo_len_in_ois_driver, &data_size);
		CAM_DBG(CAM_OIS, "ReadOISHALLData Copied %d Bytes to UMD", data_size);
	} else {
		CAM_DBG(CAM_OIS, "ReadOISHALLData fifo_len is %d, no need copy to UMD", fifo_len_in_ois_driver);
	}

	mutex_unlock(&(o_ctrl->ois_hall_data_mutex));
}

void ReadOISHALLDataV2(struct cam_ois_ctrl_t *o_ctrl, void *data)
{
	uint32_t data_size = 0;
	uint32_t fifo_len_in_ois_driver;

	mutex_lock(&(o_ctrl->ois_hall_data_mutex));
	fifo_len_in_ois_driver = kfifo_len(&(o_ctrl->ois_hall_data_fifoV2));
	if (fifo_len_in_ois_driver > 0) {
		if (fifo_len_in_ois_driver > SAMPLE_SIZE_IN_DRIVER*SAMPLE_COUNT_IN_DRIVER) {
			fifo_len_in_ois_driver = SAMPLE_SIZE_IN_DRIVER*SAMPLE_COUNT_IN_DRIVER;
		}
		kfifo_to_user(&(o_ctrl->ois_hall_data_fifoV2), data, fifo_len_in_ois_driver, &data_size);
		CAM_DBG(CAM_OIS, "ois type=%d,Copied %d Bytes to UMD EISv2",o_ctrl->ois_type, data_size);
	} else {
		CAM_DBG(CAM_OIS, "ois type=%d,fifo_len is %d, no need copy to UMD EISv2",o_ctrl->ois_type, fifo_len_in_ois_driver);
	}


	mutex_unlock(&(o_ctrl->ois_hall_data_mutex));
}

void ReadOISHALLDataV3(struct cam_ois_ctrl_t *o_ctrl, void *data)
{

	mutex_lock(&(o_ctrl->ois_hall_data_mutex));
	//reserved
	mutex_unlock(&(o_ctrl->ois_hall_data_mutex));

}

void Sem1215sReadOISHALLData(struct cam_ois_ctrl_t *o_ctrl, void *data)
{
	uint32_t data_size = 0;
	uint32_t fifo_len_in_ois_driver;

	mutex_lock(&(o_ctrl->ois_hall_data_mutex));
	fifo_len_in_ois_driver = kfifo_len(&(o_ctrl->ois_hall_data_fifo));
	if (fifo_len_in_ois_driver > 0) {
		if (fifo_len_in_ois_driver > OIS_HALL_SAMPLE_COUNT*OIS_HALL_SAMPLE_BYTE) {
			fifo_len_in_ois_driver = OIS_HALL_SAMPLE_COUNT*OIS_HALL_SAMPLE_BYTE;
		}
		kfifo_to_user(&(o_ctrl->ois_hall_data_fifo), data, fifo_len_in_ois_driver, &data_size);
		CAM_INFO(CAM_OIS, "Sem1215sReadOISHALLData Copied %d Bytes to UMD", data_size);
	} else {
		CAM_INFO(CAM_OIS, "Sem1215sReadOISHALLData fifo_len is %d, no need copy to UMD", fifo_len_in_ois_driver);
	}
	mutex_unlock(&(o_ctrl->ois_hall_data_mutex));
}

void Sem1215sReadOISHALLDataV2(struct cam_ois_ctrl_t *o_ctrl, void *data)
{
	mutex_lock(&(o_ctrl->ois_hall_data_mutex));
	mutex_unlock(&(o_ctrl->ois_hall_data_mutex));

}


void OISControl(struct cam_ois_ctrl_t *o_ctrl, void *arg)
{

}

void set_ois_thread_status(int main_thread_status ,int tele_thread_status)
{
		g_is_enable_main_ois_thread = main_thread_status;
		g_is_enable_tele_ois_thread = tele_thread_status;

}


bool IsOISReady(struct cam_ois_ctrl_t *o_ctrl)
{
	uint32_t temp, retry_cnt;
	retry_cnt = 3;

	if (o_ctrl) {
		if (strstr(o_ctrl->ois_name, "sem1215") != NULL) {
			mutex_lock(&(o_ctrl->ois_poll_thread_mutex));
			if (o_ctrl->ois_poll_thread
				|| false == g_is_enable_tele_ois_thread) {
				CAM_ERR(CAM_OIS, "tele camera ois_poll_thread is already created, no need to create again.");
			} else {
				o_ctrl->ois_poll_thread_exit = false;
				o_ctrl->ois_poll_thread = kthread_run(Sem1215sOISPollThread, o_ctrl, o_ctrl->ois_name);
			}
			mutex_unlock(&(o_ctrl->ois_poll_thread_mutex));
			if (!o_ctrl->ois_poll_thread) {
				o_ctrl->ois_poll_thread_exit = true;
				CAM_ERR(CAM_OIS, "create ois poll thread failed");
				return false;
			}
			return true;
		}
		if (strstr(o_ctrl->ois_name, ic_name_a) == NULL
			&& strstr(o_ctrl->ois_name, ic_name_b) == NULL
			&& strstr(o_ctrl->ois_name, "bu63169") == NULL) {
			CAM_ERR(CAM_OIS, "return");
			return true;
		}
		if (CAM_OIS_READY == ois_state[o_ctrl->ois_type]) {
			CAM_INFO(CAM_OIS, "%s OIS %d is ready ",o_ctrl->ois_name,o_ctrl->ois_type);

			mutex_lock(&(o_ctrl->ois_poll_thread_mutex));
			if (o_ctrl->ois_poll_thread
					|| false == g_is_enable_main_ois_thread ) {
				CAM_ERR(CAM_OIS, "main camera ois_poll_thread is already created or don't need to create..");
			} else {
				o_ctrl->ois_poll_thread_exit = false;
				if (strstr(o_ctrl->ois_name, "128")) {
					o_ctrl->ois_poll_thread = kthread_run(OISPollThread128, o_ctrl, o_ctrl->ois_name);
				} else if (strstr(o_ctrl->ois_name, "124")) {
					o_ctrl->ois_poll_thread = kthread_run(OISPollThread124, o_ctrl, o_ctrl->ois_name);
				} else if (strstr(o_ctrl->ois_name, "bu63169")) {
					o_ctrl->ois_poll_thread = kthread_run(OISPollThread169, o_ctrl, o_ctrl->ois_name);
					set_user_nice(o_ctrl->ois_poll_thread,-2);
				}
				if (!o_ctrl->ois_poll_thread) {
					o_ctrl->ois_poll_thread_exit = true;
					CAM_ERR(CAM_OIS, "create ois poll thread failed");
				}
			}
			mutex_unlock(&(o_ctrl->ois_poll_thread_mutex));
			return true;
		} else {
			do {
				RamRead32A_oplus(o_ctrl,0xF100, &temp);
				CAM_ERR(CAM_OIS, "OIS %d 0xF100 = 0x%x", o_ctrl->ois_type, temp);
				if (temp == 0) {
					ois_state[o_ctrl->ois_type] = CAM_OIS_READY;
					return true;
				}
				retry_cnt--;
				msleep(10);
			} while(retry_cnt);
			return false;
		}
	} else {
		CAM_ERR(CAM_OIS, "o_ctrl is NULL");
		return false;
	}
}

void InitOIS(struct cam_ois_ctrl_t *o_ctrl)
{
	if (o_ctrl) {
		if (strstr(o_ctrl->ois_name, ic_name_a) == NULL
			&& strstr(o_ctrl->ois_name, ic_name_b) == NULL) {
			return;
		}
		if (o_ctrl->ois_type == CAM_OIS_MASTER) {
			ois_state[CAM_OIS_MASTER] = CAM_OIS_INVALID;
		} else if (o_ctrl->ois_type == CAM_OIS_SLAVE) {
			ois_state[CAM_OIS_SLAVE] = CAM_OIS_INVALID;
			if (ois_ctrls[CAM_OIS_MASTER]) {
				if (camera_io_init(&(ois_ctrls[CAM_OIS_MASTER]->io_master_info))) {
					CAM_ERR(CAM_OIS, "cci_init failed");
				}
			}
		} else {
			CAM_ERR(CAM_OIS, "ois_type 0x%x is wrong", o_ctrl->ois_type);
		}
	} else {
		CAM_ERR(CAM_OIS, "o_ctrl is NULL");
	}
}

void DeinitOIS(struct cam_ois_ctrl_t *o_ctrl)
{
	set_ois_thread_status(0,0);
	if (o_ctrl) {
		if (strstr(o_ctrl->ois_name, "sem1215") != NULL
		    ||strstr(o_ctrl->ois_name, "bu63169") != NULL)
		{
			mutex_lock(&(o_ctrl->ois_poll_thread_mutex));
			if (o_ctrl->ois_poll_thread) {
				o_ctrl->ois_poll_thread_exit = true;
				o_ctrl->ois_poll_thread = NULL;
			}
			mutex_unlock(&(o_ctrl->ois_poll_thread_mutex));
		}

		if (strstr(o_ctrl->ois_name, ic_name_a) == NULL
			&& strstr(o_ctrl->ois_name, ic_name_b) == NULL) {
			return;
		}

		mutex_lock(&(o_ctrl->ois_poll_thread_mutex));
		if (o_ctrl->ois_poll_thread) {
			o_ctrl->ois_poll_thread_exit = true;
			o_ctrl->ois_poll_thread = NULL;
		}
		mutex_unlock(&(o_ctrl->ois_poll_thread_mutex));

		if (o_ctrl->ois_type == CAM_OIS_MASTER) {
			if (dump_ois_registers&&ois_ctrls[CAM_OIS_MASTER]) {
				ois_start_read_thread(ois_ctrls[CAM_OIS_MASTER], 0);
			}
			ois_state[CAM_OIS_MASTER] = CAM_OIS_INVALID;
		} else if (o_ctrl->ois_type == CAM_OIS_SLAVE) {
			if (ois_ctrls[CAM_OIS_MASTER]) {
				if(dump_ois_registers) {
					ois_start_read_thread(ois_ctrls[CAM_OIS_MASTER], 0);
				}
				if (camera_io_release(&(ois_ctrls[CAM_OIS_MASTER]->io_master_info))) {
					CAM_ERR(CAM_OIS, "cci_deinit failed");
				}
			}

			if (ois_ctrls[CAM_OIS_SLAVE]) {
				if(dump_ois_registers) {
					ois_start_read_thread(ois_ctrls[CAM_OIS_SLAVE], 0);
				}
			}
			ois_state[CAM_OIS_SLAVE] = CAM_OIS_INVALID;

		} else {
			CAM_ERR(CAM_OIS, "ois_type 0x%x is wrong", o_ctrl->ois_type);
		}
	} else {
		CAM_ERR(CAM_OIS, "o_ctrl is NULL");
	}
}

void InitOISResource(struct cam_ois_ctrl_t *o_ctrl)
{
	struct proc_dir_entry *face_common_dir = NULL;
	struct proc_dir_entry *proc_file_entry = NULL;

	if (o_ctrl) {
		if (strstr(o_ctrl->ois_name, ic_name_a) == NULL
			&& strstr(o_ctrl->ois_name, ic_name_b) == NULL) {
			return;
		}
		mutex_init(&ois_mutex);
		if (o_ctrl->ois_type == CAM_OIS_MASTER) {
			ois_ctrls[CAM_OIS_MASTER] = o_ctrl;
			//Hardcode the parameters of main OIS, and those parameters will be overrided when open main camera
			o_ctrl->io_master_info.cci_client->sid = 0x24;
			o_ctrl->io_master_info.cci_client->i2c_freq_mode = I2C_FAST_PLUS_MODE;
			o_ctrl->io_master_info.cci_client->retries = 3;
			o_ctrl->io_master_info.cci_client->id_map = 0;
			CAM_INFO(CAM_OIS, "ois_ctrls[%d] = %p", CAM_OIS_MASTER, ois_ctrls[CAM_OIS_MASTER]);
		} else if (o_ctrl->ois_type == CAM_OIS_SLAVE) {
			ois_ctrls[CAM_OIS_SLAVE] = o_ctrl;
			CAM_INFO(CAM_OIS, "ois_ctrls[%d] = %p", CAM_OIS_SLAVE, ois_ctrls[CAM_OIS_SLAVE]);
		} else {
			CAM_ERR(CAM_OIS, "ois_type 0x%x is wrong", o_ctrl->ois_type);
		}

		if (!ois_dentry) {
			ois_dentry = debugfs_create_dir("camera_ois", NULL);
			if (ois_dentry) {
				debugfs_create_bool("dump_registers", 0644, ois_dentry, &dump_ois_registers);
			} else {
				CAM_ERR(CAM_OIS, "failed to create dump_registers node");
			}
		} else {
			CAM_ERR(CAM_OIS, "dump_registers node exist");
		}
	} else {
		CAM_ERR(CAM_OIS, "o_ctrl is NULL");
	}
	//Create OIS control node
	face_common_dir =  proc_mkdir("OIS", NULL);
	if(!face_common_dir) {
		CAM_ERR(CAM_OIS, "create dir fail CAM_ERROR API");
		//return FACE_ERROR_GENERAL;
	}

	proc_file_entry = proc_create("OISControl", 0777, face_common_dir, &proc_file_fops);
	if(proc_file_entry == NULL) {
		CAM_ERR(CAM_OIS, "Create fail");
	} else {
		CAM_INFO(CAM_OIS, "Create successs");
	}

}

void CheckOISdata(void) {
	uint32_t temp = 0x0;
	RamRead32A(0XF010, &temp);
	CAM_INFO(CAM_OIS, "0XF010 = 0x%0x", temp);
	RamRead32A(0XF011, &temp);
	CAM_INFO(CAM_OIS, "0XF011 = 0x%0x", temp);
	RamRead32A(0XF012, &temp);
	CAM_INFO(CAM_OIS, "0XF012 = 0x%0x", temp);
	RamRead32A(0XF013, &temp);
	CAM_INFO(CAM_OIS, "0XF013 = 0x%0x", temp);
	RamRead32A(0XF015, &temp);
	CAM_INFO(CAM_OIS, "0XF015 = 0x%0x", temp);
	RamRead32A(0XF017, &temp);
	CAM_INFO(CAM_OIS, "0XF017 = 0x%0x", temp);
	RamRead32A(0X0178, &temp);
	CAM_INFO(CAM_OIS, "0X0178 = 0x%0x", temp);
	RamRead32A(0X017C, &temp);
	CAM_INFO(CAM_OIS, "0X017C = 0x%0x", temp);
	RamRead32A(0X00D8, &temp);
	CAM_INFO(CAM_OIS, "0X00D8 = 0x%0x", temp);
	RamRead32A(0X0128, &temp);
	CAM_INFO(CAM_OIS, "0X0128 = 0x%0x", temp);
	return;
}

void CheckOISfwVersion(void) {
	uint32_t temp = 0x0;
	RamRead32A(0x8000, &temp);
	CAM_INFO(CAM_OIS, "OIS Version 0X8000 = 0x%0x", temp);
	RamRead32A(0x8004, &temp);
	CAM_INFO(CAM_OIS, "OIS Version 0X8004 = 0x%0x", temp);
	return;
}

#define MAX_EEPROM_PACK_SIZE 252
#define EEPROM_UNIT_SIZE 4

void WriteEEpromData(struct cam_write_eeprom_t *cam_write_eeprom) {
	UINT8 mdata[MAX_EEPROM_PACK_SIZE];
	UINT32 i = 0;
	UINT32 addr_offset = 0x00;
	memset(mdata, 0xFF, MAX_EEPROM_PACK_SIZE);
	LoadUserAreaToPM();

	/*
	for (i = 0; i < (cam_write_eeprom->calibDataSize / MAX_EEPROM_PACK_SIZE); i++) {
		RdBurstUareaFromPm((cam_write_eeprom->baseAddr
			+ (i * MAX_EEPROM_PACK_SIZE / EEPROM_UNIT_SIZE)),
			mdata, MAX_EEPROM_PACK_SIZE, 0);
	}

	addr_offset = cam_write_eeprom->baseAddr + (i * MAX_EEPROM_PACK_SIZE);
	RdBurstUareaFromPm(addr_offset, mdata,
		(cam_write_eeprom->calibDataSize % MAX_EEPROM_PACK_SIZE), 0);
	*/
	CAM_ERR(CAM_OIS, "entry write eeprom!!!");
	for (i = 0; i < (cam_write_eeprom->calibDataSize / MAX_EEPROM_PACK_SIZE); i++) {
		WrUareaToPm((cam_write_eeprom->baseAddr
			+ (i * MAX_EEPROM_PACK_SIZE / EEPROM_UNIT_SIZE)),
			&cam_write_eeprom->calibData[0], MAX_EEPROM_PACK_SIZE, 0);
	}
	addr_offset = cam_write_eeprom->baseAddr + (i * MAX_EEPROM_PACK_SIZE);
	WrUareaToPm(addr_offset, &cam_write_eeprom->calibData[i*MAX_EEPROM_PACK_SIZE],
		(cam_write_eeprom->calibDataSize % MAX_EEPROM_PACK_SIZE), 0);

	WrUareaToFlash();
}

void ReadEEpromData(struct cam_write_eeprom_t *cam_write_eeprom) {
	UINT8 mdata[MAX_EEPROM_PACK_SIZE];
	UINT32 i = 0;
	UINT32 addr_offset = 0x00;
	cam_write_eeprom->calibDataSize;
	LoadUserAreaToPM();
	for (i = 0; i < (cam_write_eeprom->calibDataSize / MAX_EEPROM_PACK_SIZE); i++) {
		RdBurstUareaFromPm((cam_write_eeprom->baseAddr
			+ (i * MAX_EEPROM_PACK_SIZE / EEPROM_UNIT_SIZE)),
			mdata, MAX_EEPROM_PACK_SIZE, 0);
	}
	addr_offset = cam_write_eeprom->baseAddr + (i * MAX_EEPROM_PACK_SIZE);

	RdBurstUareaFromPm(addr_offset, mdata,
		(cam_write_eeprom->calibDataSize % MAX_EEPROM_PACK_SIZE), 0);
}

