/* Copyright (c) 2017-2019, The Linux Foundation. All rights reserved.
 * Copyright (c) 2021 Qualcomm Innovation Center, Inc. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/firmware.h>
#include <linux/string.h>
#include <linux/time.h>
#include <cam_sensor_cmn_header.h>
#include "cam_ois_core.h"
#include "cam_ois_soc.h"
#include "cam_sensor_util.h"
#include "cam_debug_util.h"
#include "cam_res_mgr_api.h"
#include "cam_common_util.h"
#include "cam_packet_util.h"
#include "cam_ois_oplus.h"

#define MASTER_CCI_ADDR (0x7C >> 1)
#define SLAVE_CCI_ADDR (0x74 >> 1)

static bool imx586_ois_initialized = false;
static bool s5k3m5_ois_initialized = false;
static bool imx586_ois_ready = false;
static bool s5k3m5_ois_ready = false;

extern struct cam_ois_ctrl_t *ctrl_wide;
extern struct cam_ois_ctrl_t *ctrl_tele;
enum cci_i2c_master_t imx586_cci_master = MASTER_MAX;

#ifdef ENABLE_OIS_DELAY_POWER_DOWN
static int cam_ois_power_down(struct cam_ois_ctrl_t *o_ctrl);

int ois_power_down_thread(void *arg)
{
	int rc = 0;
	int i;
	struct cam_ois_ctrl_t *o_ctrl = (struct cam_ois_ctrl_t *)arg;
	struct cam_ois_soc_private *soc_private =
			(struct cam_ois_soc_private *)o_ctrl->soc_info.soc_private;
	struct cam_sensor_power_ctrl_t *power_info = &soc_private->power_info;

		if (!o_ctrl || !soc_private || !power_info) {
			CAM_ERR(CAM_OIS, "failed: o_ctrl %pK, soc_private %pK, power_info %pK", o_ctrl, soc_private, power_info);
			return -EINVAL;
		}

	mutex_lock(&(o_ctrl->ois_power_down_mutex));
	o_ctrl->ois_power_down_thread_state = CAM_OIS_POWER_DOWN_THREAD_RUNNING;
	mutex_unlock(&(o_ctrl->ois_power_down_mutex));

	for (i = 0; i < (OIS_POWER_DOWN_DELAY/50); i++) {
		msleep(50);// sleep 50ms every time, and sleep OIS_POWER_DOWN_DELAY/50 times.

		mutex_lock(&(o_ctrl->ois_power_down_mutex));
		if (o_ctrl->ois_power_down_thread_exit) {
		mutex_unlock(&(o_ctrl->ois_power_down_mutex));
		break;
		}
		mutex_unlock(&(o_ctrl->ois_power_down_mutex));
	}

	mutex_lock(&(o_ctrl->ois_power_down_mutex));
	if ((!o_ctrl->ois_power_down_thread_exit) && (o_ctrl->ois_power_state == CAM_OIS_POWER_ON)) {
			rc = cam_ois_power_down(o_ctrl);
			if (!rc) {
				kfree(power_info->power_setting);
				kfree(power_info->power_down_setting);
				power_info->power_setting = NULL;
				power_info->power_down_setting = NULL;
				power_info->power_down_setting_size = 0;
				power_info->power_setting_size = 0;
				CAM_ERR(CAM_OIS, "cam_ois_power_down successfully");
			} else {
				CAM_ERR(CAM_OIS, "cam_ois_power_down failed");
			}
			o_ctrl->ois_power_state = CAM_OIS_POWER_OFF;
	} else {
			CAM_ERR(CAM_OIS, "No need to do power down, ois_power_down_thread_exit %d, ois_power_state %d", o_ctrl->ois_power_down_thread_exit, o_ctrl->ois_power_state);
	}
	o_ctrl->ois_power_down_thread_state = CAM_OIS_POWER_DOWN_THREAD_STOPPED;
	mutex_unlock(&(o_ctrl->ois_power_down_mutex));

	return rc;
}
#endif

int32_t cam_ois_construct_default_power_setting(
	struct cam_sensor_power_ctrl_t *power_info)
{
	int rc = 0;

	power_info->power_setting_size = 1;
	power_info->power_setting =
		(struct cam_sensor_power_setting *)
		kzalloc(sizeof(struct cam_sensor_power_setting),
			GFP_KERNEL);
	if (!power_info->power_setting)
		return -ENOMEM;

	power_info->power_setting[0].seq_type = SENSOR_VAF;
	power_info->power_setting[0].seq_val = CAM_VAF;
	power_info->power_setting[0].config_val = 1;
	power_info->power_setting[0].delay = 2;

	power_info->power_down_setting_size = 1;
	power_info->power_down_setting =
		(struct cam_sensor_power_setting *)
		kzalloc(sizeof(struct cam_sensor_power_setting),
			GFP_KERNEL);
	if (!power_info->power_down_setting) {
		rc = -ENOMEM;
		goto free_power_settings;
	}

	power_info->power_down_setting[0].seq_type = SENSOR_VAF;
	power_info->power_down_setting[0].seq_val = CAM_VAF;
	power_info->power_down_setting[0].config_val = 0;

	return rc;

free_power_settings:
	kfree(power_info->power_setting);
	power_info->power_setting = NULL;
	power_info->power_setting_size = 0;
	return rc;
}


/**
 * cam_ois_get_dev_handle - get device handle
 * @o_ctrl:     ctrl structure
 * @arg:        Camera control command argument
 *
 * Returns success or failure
 */
static int cam_ois_get_dev_handle(struct cam_ois_ctrl_t *o_ctrl,
	void *arg)
{
	struct cam_sensor_acquire_dev    ois_acq_dev;
	struct cam_create_dev_hdl        bridge_params;
	struct cam_control              *cmd = (struct cam_control *)arg;

	if (o_ctrl->bridge_intf.device_hdl != -1) {
		CAM_ERR(CAM_OIS, "Device is already acquired");
		return -EFAULT;
	}
	if (copy_from_user(&ois_acq_dev, u64_to_user_ptr(cmd->handle),
		sizeof(ois_acq_dev)))
		return -EFAULT;

	bridge_params.session_hdl = ois_acq_dev.session_handle;
	bridge_params.ops = &o_ctrl->bridge_intf.ops;
	bridge_params.v4l2_sub_dev_flag = 0;
	bridge_params.media_entity_flag = 0;
	bridge_params.priv = o_ctrl;
	bridge_params.dev_id = CAM_OIS;
	ois_acq_dev.device_handle =
		cam_create_device_hdl(&bridge_params);
	if (ois_acq_dev.device_handle <= 0) {
		CAM_ERR(CAM_OIS, "Can not create device handle");
		return -EFAULT;
	}
	o_ctrl->bridge_intf.device_hdl = ois_acq_dev.device_handle;
	o_ctrl->bridge_intf.session_hdl = ois_acq_dev.session_handle;

	CAM_DBG(CAM_OIS, "Device Handle: %d", ois_acq_dev.device_handle);
	if (copy_to_user(u64_to_user_ptr(cmd->handle), &ois_acq_dev,
		sizeof(struct cam_sensor_acquire_dev))) {
		CAM_ERR(CAM_OIS, "ACQUIRE_DEV: copy to user failed");
		return -EFAULT;
	}
	return 0;
}

static int cam_ois_power_up(struct cam_ois_ctrl_t *o_ctrl)
{
	int                             rc = 0;
	struct cam_hw_soc_info          *soc_info =
		&o_ctrl->soc_info;
	struct cam_ois_soc_private *soc_private;
	struct cam_sensor_power_ctrl_t  *power_info;

	soc_private =
		(struct cam_ois_soc_private *)o_ctrl->soc_info.soc_private;
	power_info = &soc_private->power_info;

	if ((power_info->power_setting == NULL) &&
		(power_info->power_down_setting == NULL)) {
		CAM_INFO(CAM_OIS,
			"Using default power settings");
		rc = cam_ois_construct_default_power_setting(power_info);
		if (rc < 0) {
			CAM_ERR(CAM_OIS,
				"Construct default ois power setting failed.");
			return rc;
		}
	}

	ctrl_wide = NULL;
	ctrl_tele = NULL;

	/* Parse and fill vreg params for power up settings */
	rc = msm_camera_fill_vreg_params(
		soc_info,
		power_info->power_setting,
		power_info->power_setting_size);
	if (rc) {
		CAM_ERR(CAM_OIS,
			"failed to fill vreg params for power up rc:%d", rc);
		return rc;
	}

	/* Parse and fill vreg params for power down settings*/
	rc = msm_camera_fill_vreg_params(
		soc_info,
		power_info->power_down_setting,
		power_info->power_down_setting_size);
	if (rc) {
		CAM_ERR(CAM_OIS,
			"failed to fill vreg params for power down rc:%d", rc);
		return rc;
	}

	power_info->dev = soc_info->dev;

	rc = cam_sensor_core_power_up(power_info, soc_info);
	if (rc) {
		CAM_ERR(CAM_OIS, "failed in ois power up rc %d", rc);
		return rc;
	}

	rc = camera_io_init(&o_ctrl->io_master_info);
	if (rc)
		CAM_ERR(CAM_OIS, "cci_init failed: rc: %d", rc);
	else
		CAM_INFO(CAM_OIS,"camera_io_init");

	return rc;
}

/**
 * cam_ois_power_down - power down OIS device
 * @o_ctrl:     ctrl structure
 *
 * Returns success or failure
 */
static int cam_ois_power_down(struct cam_ois_ctrl_t *o_ctrl)
{
	int32_t                         rc = 0;
	struct cam_sensor_power_ctrl_t  *power_info;
	struct cam_hw_soc_info          *soc_info =
		&o_ctrl->soc_info;
	struct cam_ois_soc_private *soc_private;

	if (!o_ctrl) {
		CAM_ERR(CAM_OIS, "failed: o_ctrl %pK", o_ctrl);
		return -EINVAL;
	}

	soc_private =
		(struct cam_ois_soc_private *)o_ctrl->soc_info.soc_private;
	power_info = &soc_private->power_info;
	soc_info = &o_ctrl->soc_info;

	if (!power_info) {
		CAM_ERR(CAM_OIS, "failed: power_info %pK", power_info);
		return -EINVAL;
	}

	rc = cam_sensor_util_power_down(power_info, soc_info);
	if (rc) {
		CAM_ERR(CAM_OIS, "power down the core is failed:%d", rc);
		return rc;
	}

	camera_io_release(&o_ctrl->io_master_info);
	CAM_INFO(CAM_OIS, "cam_io_release");

	if (strstr(o_ctrl->ois_name, "imx586")) {
		imx586_ois_initialized = false;
		imx586_ois_ready = false;
	} else if (strstr(o_ctrl->ois_name, "s5k3m5")) {
		s5k3m5_ois_initialized = false;
		s5k3m5_ois_ready = false;
	}

	return rc;
}

static int cam_ois_apply_settings(struct cam_ois_ctrl_t *o_ctrl,
	struct i2c_settings_array *i2c_set)
{
	struct i2c_settings_list *i2c_list;
	int32_t rc = 0;
	uint32_t i, size;

	if (o_ctrl == NULL || i2c_set == NULL) {
		CAM_ERR(CAM_OIS, "Invalid Args");
		return -EINVAL;
	}

	if (i2c_set->is_settings_valid != 1) {
		CAM_ERR(CAM_OIS, " Invalid settings");
		return -EINVAL;
	}

	list_for_each_entry(i2c_list,
		&(i2c_set->list_head), list) {
		if (i2c_list->op_code ==  CAM_SENSOR_I2C_WRITE_RANDOM) {
			rc = camera_io_dev_write(&(o_ctrl->io_master_info),
				&(i2c_list->i2c_settings));
			if (rc < 0) {
				CAM_ERR(CAM_OIS,
					"Failed in Applying i2c wrt settings");
				return rc;
			}
		} else if (i2c_list->op_code == CAM_SENSOR_I2C_POLL) {
			size = i2c_list->i2c_settings.size;
			for (i = 0; i < size; i++) {
				rc = camera_io_dev_poll(
				&(o_ctrl->io_master_info),
				i2c_list->i2c_settings.reg_setting[i].reg_addr,
				i2c_list->i2c_settings.reg_setting[i].reg_data,
				i2c_list->i2c_settings.reg_setting[i].data_mask,
				i2c_list->i2c_settings.addr_type,
				i2c_list->i2c_settings.data_type,
				i2c_list->i2c_settings.reg_setting[i].delay);
				if (rc < 0) {
					CAM_ERR(CAM_OIS,
						"i2c poll apply setting Fail");
					return rc;
				}
			}
		}
	}

	return rc;
}

static int cam_ois_slaveInfo_pkt_parser(struct cam_ois_ctrl_t *o_ctrl,
	uint32_t *cmd_buf, size_t len)
{
	int32_t rc = 0;
	struct cam_cmd_ois_info *ois_info;

	if (!o_ctrl || !cmd_buf || len < sizeof(struct cam_cmd_ois_info)) {
		CAM_ERR(CAM_OIS, "Invalid Args");
		return -EINVAL;
	}

	ois_info = (struct cam_cmd_ois_info *)cmd_buf;
	if (o_ctrl->io_master_info.master_type == CCI_MASTER) {
		o_ctrl->io_master_info.cci_client->i2c_freq_mode =
			ois_info->i2c_freq_mode;
		o_ctrl->io_master_info.cci_client->sid =
			ois_info->slave_addr >> 1;
		o_ctrl->ois_fw_flag = ois_info->ois_fw_flag;
		o_ctrl->is_ois_calib = ois_info->is_ois_calib;
		memcpy(o_ctrl->ois_name, ois_info->ois_name, OIS_NAME_LEN);
		o_ctrl->ois_name[OIS_NAME_LEN - 1] = '\0';
		o_ctrl->io_master_info.cci_client->retries = 3;
		o_ctrl->io_master_info.cci_client->id_map = 0;
		memcpy(&(o_ctrl->opcode), &(ois_info->opcode),
			sizeof(struct cam_ois_opcode));
		CAM_DBG(CAM_OIS, "Slave addr: 0x%x Freq Mode: %d",
			ois_info->slave_addr, ois_info->i2c_freq_mode);
	} else if (o_ctrl->io_master_info.master_type == I2C_MASTER) {
		o_ctrl->io_master_info.client->addr = ois_info->slave_addr;
		CAM_DBG(CAM_OIS, "Slave addr: 0x%x", ois_info->slave_addr);
	} else {
		CAM_ERR(CAM_OIS, "Invalid Master type : %d",
			o_ctrl->io_master_info.master_type);
		rc = -EINVAL;
	}

	return rc;
}

static int cam_ois_fw_download(struct cam_ois_ctrl_t *o_ctrl)
{
	UINT32 UlReadValX, UlReadValY;
	UINT32 spi_type;
	UINT32 UlReadVal;
	unsigned char rc = 0;
	struct timespec mStartTime, mEndTime, diff;
	UINT64 mSpendTime = 0;
	enum cci_i2c_master_t entry_cci_master;

	if (!o_ctrl) {
		CAM_ERR(CAM_OIS, "Invalid Args");
		return -EINVAL;
	}
	entry_cci_master = o_ctrl->io_master_info.cci_client->cci_i2c_master;
	getnstimeofday(&mStartTime);

	//Master_1:imx586 Master_0:s5k3m5
	CAM_INFO(CAM_OIS, "sid:0x%02x, Master:%d, ois_gyro_id:%d cci_master_id:%d",
		o_ctrl->io_master_info.cci_client->sid,
		o_ctrl->io_master_info.cci_client->cci_i2c_master,
		o_ctrl->ois_gyro_id,o_ctrl->cci_master_id);

	//identify if hotdogb else fallback
	imx586_cci_master = (enum cci_i2c_master_t)o_ctrl->cci_master_id;
	if (imx586_cci_master == o_ctrl->io_master_info.cci_client->cci_i2c_master &&
		false == imx586_ois_initialized && strstr(o_ctrl->ois_name, "imx586")) {
		if (o_ctrl->ois_gyro_id == 3) {
			if (strcmp(o_ctrl->ois_name, "ofilm_imx586_lc898124ep3_ois") == 0) {
				rc = SelectDownload(o_ctrl, 0x02, 0x01, 0x00, o_ctrl->ois_fw_flag);
			} else {
				rc = SelectDownload(o_ctrl, 0x02, 0x00, 0x00, o_ctrl->ois_fw_flag);
			}
			} else {
			if (strcmp(o_ctrl->ois_name, "ofilm_imx586_lc898124ep3_ois") == 0) {
				rc = SelectDownload(o_ctrl, 0x02, 0x06, 0x00, o_ctrl->ois_fw_flag);
			} else {
				rc = SelectDownload(o_ctrl, 0x02, 0x02, 0x00, o_ctrl->ois_fw_flag);
			}
		}

		if (0 == rc) {
			imx586_ois_initialized = true;
			RamRead32A(o_ctrl,(SiVerNum + 4), &UlReadVal);
			SetGyroAccelCoef(o_ctrl,(UINT8)(UlReadVal>>8));
			CAM_INFO(CAM_OIS, "SetGyroAccelCoef : %02x\n", (UINT8)(UlReadVal>>8));

			//remap master
			RamWrite32A(o_ctrl, 0xF000, 0x00000000);

			//SPI-Master ( Act1 )  Check gyro signal
			RamRead32A(o_ctrl, 0x061C, & UlReadValX);
			RamRead32A(o_ctrl, 0x0620, & UlReadValY);
			CAM_INFO(CAM_OIS, "Gyro_X:0x%x, Gyro_Y:0x%x", UlReadValX, UlReadValY);

			spi_type = 0;
			RamRead32A(o_ctrl, 0xf112, & spi_type);
			CAM_INFO(CAM_OIS, "spi_type:0x%x", spi_type);

			//SPI-Master ( Act1 )  Check gyro gain
			RamRead32A(o_ctrl, 0x82b8, & UlReadValX);
			RamRead32A(o_ctrl, 0x8318, & UlReadValY);
			CAM_INFO(CAM_OIS, "Gyro_gain_X:0x%x, Gyro_gain_Y:0x%x", UlReadValX, UlReadValY);

			//SPI-Master ( Act1 )  start gyro signal transfer. ( from Master to slave. )
			RamWrite32A(o_ctrl, 0x8970, 0x00000001);
			//msleep(5);
			RamWrite32A(o_ctrl, 0xf111, 0x00000001);
			//msleep(5);
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
	} else if (MASTER_0 == o_ctrl->io_master_info.cci_client->cci_i2c_master &&
		false == s5k3m5_ois_initialized && strstr(o_ctrl->ois_name, "s5k3m5")) {
		o_ctrl->io_master_info.cci_client->cci_i2c_master = MASTER_0;
		o_ctrl->io_master_info.cci_client->sid = SLAVE_CCI_ADDR;
		if (strcmp(o_ctrl->ois_name, "ofilm_s5k3m5_lc898124ep3_ois") == 0 ) {
			rc = SelectDownload(o_ctrl, 0x02, 0x07, 0x01, o_ctrl->ois_fw_flag);
		} else {
			rc = SelectDownload(o_ctrl, 0x02, 0x03, 0x01, o_ctrl->ois_fw_flag);
		}
		if (0 == rc) {
			RamRead32A( o_ctrl,(SiVerNum + 4), &UlReadVal );
			//SetGyroAccelCoef(o_ctrl, 0x02);
			SetGyroAccelCoef( o_ctrl,(UINT8)(UlReadVal>>8) );
			CAM_INFO(CAM_OIS, "SetGyroAccelCoef : %02x\n", (UINT8)(UlReadVal>>8));
			//remap slave
			RamWrite32A(o_ctrl, 0xF000, 0x00000000 );
			//msleep(120);
			//SPI-Master ( Act1 )  Check gyro signal
			RamRead32A(o_ctrl, 0x061C, & UlReadValX );
			RamRead32A(o_ctrl, 0x0620, & UlReadValY );
			CAM_INFO(CAM_OIS, "Gyro_X:0x%x, Gyro_Y:0x%x", UlReadValX, UlReadValY);

			spi_type = 0;
			RamRead32A(o_ctrl, 0xf112, & spi_type );
			CAM_INFO(CAM_OIS, "spi_type:0x%x", spi_type);

			//SPI-Master ( Act1 )  Check gyro gain
			RamRead32A(o_ctrl, 0x82b8, & UlReadValX );
			RamRead32A(o_ctrl, 0x8318, & UlReadValY );
			CAM_INFO(CAM_OIS, "Gyro_gain_X:0x%x, Gyro_gain_Y:0x%x", UlReadValX, UlReadValY);
			s5k3m5_ois_initialized = true;
		} else {
			switch (rc) {
			case 0x01:
				CAM_ERR(CAM_OIS, "Slave H/W error");
				break;
			case 0x02:
				CAM_ERR(CAM_OIS, "Slave Table Data & Program download verify error");
				break;
			case 0xF0:
				CAM_ERR(CAM_OIS, "Slave Download code select error");
				break;
			case 0xF1:
				CAM_ERR(CAM_OIS, "Slave Download code information read error");
				break;
			case 0xF2:
				CAM_ERR(CAM_OIS, "Slave Download code information disagreement");
				break;
			case 0xF3:
				CAM_ERR(CAM_OIS, "Slave Download code version error");
				break;
			default:
				CAM_ERR(CAM_OIS, "Slave Unkown error code");
				break;
			}
		}
	
		if (false == imx586_ois_initialized) {
			o_ctrl->io_master_info.cci_client->cci_i2c_master = imx586_cci_master;
			o_ctrl->io_master_info.cci_client->sid = MASTER_CCI_ADDR;
			if (strcmp(o_ctrl->ois_name, "ofilm_imx586_lc898124ep3_ois") == 0 ) {
				rc = SelectDownload(o_ctrl, 0x02, 0x06, 0x00, o_ctrl->ois_fw_flag);
			} else {
				rc = SelectDownload(o_ctrl, 0x02, 0x02, 0x00, o_ctrl->ois_fw_flag);
			}
			if (0 == rc) {
				//remap master
				RamWrite32A(o_ctrl, 0xF000, 0x00000000 );
				msleep(120);
				//SPI-Master ( Act1 )  start gyro signal transfer. ( from Master to slave. )
				RamWrite32A(o_ctrl, 0x8970, 0x00000001 );
				msleep(5);
				RamWrite32A(o_ctrl, 0xf111, 0x00000001 );
				o_ctrl->io_master_info.cci_client->cci_i2c_master = MASTER_0;
				o_ctrl->io_master_info.cci_client->sid = SLAVE_CCI_ADDR;
				RamRead32A(o_ctrl, 0x061C, & UlReadValX );
				RamRead32A(o_ctrl, 0x0620, & UlReadValY );
				CAM_INFO(CAM_OIS, "Slave Gyro_X:0x%x, Gyro_Y:0x%x", UlReadValX, UlReadValY);
				spi_type = 0;
				RamRead32A(o_ctrl, 0xf112, & spi_type );
				CAM_INFO(CAM_OIS, "spi_type:0x%x", spi_type);
				//imx586_ois_initialized = true;
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
		}
	}
	getnstimeofday(&mEndTime);
	diff = timespec_sub(mEndTime, mStartTime);
	mSpendTime = (timespec_to_ns(&diff))/1000000;
	o_ctrl->io_master_info.cci_client->cci_i2c_master = entry_cci_master;
	CAM_INFO(CAM_OIS, "cam_ois_fw_download rc=%d, (Spend: %d ms)", rc, mSpendTime);

	return 0;
}

/**
 * cam_ois_pkt_parse - Parse csl packet
 * @o_ctrl:     ctrl structure
 * @arg:        Camera control command argument
 *
 * Returns success or failure
 */
static int cam_ois_pkt_parse(struct cam_ois_ctrl_t *o_ctrl, void *arg)
{
	int32_t                         rc = 0;
	int32_t                         i = 0;
	uint32_t                        total_cmd_buf_in_bytes = 0;
	struct common_header           *cmm_hdr = NULL;
	uintptr_t                       generic_ptr;
	struct cam_control             *ioctl_ctrl = NULL;
	struct cam_config_dev_cmd       dev_config;
	struct i2c_settings_array      *i2c_reg_settings = NULL;
	struct cam_cmd_buf_desc        *cmd_desc = NULL;
	uintptr_t                       generic_pkt_addr;
	size_t                          pkt_len;
	size_t                          remain_len = 0;
	struct cam_packet              *csl_packet = NULL;
	size_t                          len_of_buff = 0;
	uint32_t                       *offset = NULL, *cmd_buf;
	struct cam_ois_soc_private     *soc_private =
		(struct cam_ois_soc_private *)o_ctrl->soc_info.soc_private;
	struct cam_sensor_power_ctrl_t  *power_info = &soc_private->power_info;
	uint32_t temp, retry_cnt;

	ioctl_ctrl = (struct cam_control *)arg;
	if (copy_from_user(&dev_config,
		u64_to_user_ptr(ioctl_ctrl->handle),
		sizeof(dev_config)))
		return -EFAULT;
	rc = cam_mem_get_cpu_buf(dev_config.packet_handle,
		&generic_pkt_addr, &pkt_len);
	if (rc) {
		CAM_ERR(CAM_OIS,
			"error in converting command Handle Error: %d", rc);
		return rc;
	}

	remain_len = pkt_len;
	if ((sizeof(struct cam_packet) > pkt_len) ||
		((size_t)dev_config.offset >= pkt_len -
		sizeof(struct cam_packet))) {
		CAM_ERR(CAM_OIS,
			"Inval cam_packet strut size: %zu, len_of_buff: %zu",
			 sizeof(struct cam_packet), pkt_len);
		rc = -EINVAL;
		goto rel_pkt;
	}

	remain_len -= (size_t)dev_config.offset;
	csl_packet = (struct cam_packet *)
		(generic_pkt_addr + (uint32_t)dev_config.offset);

	if (cam_packet_util_validate_packet(csl_packet,
		remain_len)) {
		CAM_ERR(CAM_OIS, "Invalid packet params");
		rc = -EINVAL;
		goto rel_pkt;
	}


	switch (csl_packet->header.op_code & 0xFFFFFF) {
	case CAM_OIS_PACKET_OPCODE_INIT:
		offset = (uint32_t *)&csl_packet->payload;
		offset += (csl_packet->cmd_buf_offset / sizeof(uint32_t));
		cmd_desc = (struct cam_cmd_buf_desc *)(offset);

		/* Loop through multiple command buffers */
		for (i = 0; i < csl_packet->num_cmd_buf; i++) {
			total_cmd_buf_in_bytes = cmd_desc[i].length;
			if (!total_cmd_buf_in_bytes)
				continue;

			rc = cam_mem_get_cpu_buf(cmd_desc[i].mem_handle,
				&generic_ptr, &len_of_buff);
			if (rc < 0) {
				CAM_ERR(CAM_OIS, "Failed to get cpu buf : 0x%x",
					cmd_desc[i].mem_handle);
				goto rel_pkt;
			}
			cmd_buf = (uint32_t *)generic_ptr;
			if (!cmd_buf) {
				CAM_ERR(CAM_OIS, "invalid cmd buf");
				rc = -EINVAL;
				goto rel_cmd_buf;
			}

			if ((len_of_buff < sizeof(struct common_header)) ||
				(cmd_desc[i].offset > (len_of_buff -
				sizeof(struct common_header)))) {
				CAM_ERR(CAM_OIS,
					"Invalid length for sensor cmd");
				rc = -EINVAL;
				goto rel_cmd_buf;
			}
			remain_len = len_of_buff - cmd_desc[i].offset;
			cmd_buf += cmd_desc[i].offset / sizeof(uint32_t);
			cmm_hdr = (struct common_header *)cmd_buf;

			switch (cmm_hdr->cmd_type) {
			case CAMERA_SENSOR_CMD_TYPE_I2C_INFO:
				rc = cam_ois_slaveInfo_pkt_parser(
					o_ctrl, cmd_buf, remain_len);
				if (rc < 0) {
					CAM_ERR(CAM_OIS,
					"Failed in parsing slave info");
					goto rel_cmd_buf;
				}
				break;
			case CAMERA_SENSOR_CMD_TYPE_PWR_UP:
			case CAMERA_SENSOR_CMD_TYPE_PWR_DOWN:
				CAM_DBG(CAM_OIS,
					"Received power settings buffer");
#ifdef ENABLE_OIS_DELAY_POWER_DOWN
				mutex_lock(&(o_ctrl->ois_power_down_mutex));
				if (o_ctrl->ois_power_state == CAM_OIS_POWER_OFF) {
				rc = cam_sensor_update_power_settings(
						cmd_buf,
						total_cmd_buf_in_bytes,
						power_info, remain_len);
					if (!rc){
					    CAM_ERR(CAM_OIS, "cam_sensor_update_power_settings successfully");
					} else {
					    CAM_ERR(CAM_OIS, "cam_sensor_update_power_settings failed");
					    goto rel_cmd_buf;
					}
				} else {
				    CAM_ERR(CAM_OIS, "OIS already power on, no need to update power setting");
				}
				mutex_unlock(&(o_ctrl->ois_power_down_mutex));
#else
				rc = cam_sensor_update_power_settings(
					cmd_buf,
					total_cmd_buf_in_bytes,
					power_info, remain_len);
#endif
				if (rc) {
					CAM_ERR(CAM_OIS,
					"Failed: parse power settings");
					goto rel_cmd_buf;
				}
				break;
			default:
			if (o_ctrl->i2c_init_data.is_settings_valid == 0) {
				CAM_DBG(CAM_OIS,
				"Received init settings");
				i2c_reg_settings =
					&(o_ctrl->i2c_init_data);
				i2c_reg_settings->is_settings_valid = 1;
				i2c_reg_settings->request_id = 0;
				rc = cam_sensor_i2c_command_parser(
					&o_ctrl->io_master_info,
					i2c_reg_settings,
					&cmd_desc[i], 1);
				if (rc < 0) {
					CAM_ERR(CAM_OIS,
					"init parsing failed: %d", rc);
					goto rel_cmd_buf;
				}
			} else if ((o_ctrl->is_ois_calib != 0) &&
				(o_ctrl->i2c_calib_data.is_settings_valid ==
				0)) {
				CAM_DBG(CAM_OIS,
					"Received calib settings");
				i2c_reg_settings = &(o_ctrl->i2c_calib_data);
				i2c_reg_settings->is_settings_valid = 1;
				i2c_reg_settings->request_id = 0;
				rc = cam_sensor_i2c_command_parser(
					&o_ctrl->io_master_info,
					i2c_reg_settings,
					&cmd_desc[i], 1);
				if (rc < 0) {
					CAM_ERR(CAM_OIS,
						"Calib parsing failed: %d", rc);
					goto rel_cmd_buf;
				}
			}
			break;
			}
			if (cam_mem_put_cpu_buf(cmd_desc[i].mem_handle))
				CAM_WARN(CAM_OIS, "Failed to put cpu buf: 0x%x",
					cmd_desc[i].mem_handle);
		}

		if (o_ctrl->cam_ois_state != CAM_OIS_CONFIG) {
#ifdef ENABLE_OIS_DELAY_POWER_DOWN
			mutex_lock(&(o_ctrl->ois_power_down_mutex));
			o_ctrl->ois_power_down_thread_exit = true;
			if (o_ctrl->ois_power_state == CAM_OIS_POWER_OFF){
				rc = cam_ois_power_up(o_ctrl);
				if (!rc){
					o_ctrl->ois_power_state = CAM_OIS_POWER_ON;
					CAM_ERR(CAM_OIS, "cam_ois_power_up successfully");
				} else {
					CAM_ERR(CAM_OIS, "cam_ois_power_up failed");
					goto rel_pkt;
				}
			} else {
				CAM_ERR(CAM_OIS, "OIS already power on, no need to power on again");
			}
			mutex_unlock(&(o_ctrl->ois_power_down_mutex));
#else
			rc = cam_ois_power_up(o_ctrl);
#endif
			if (rc) {
				CAM_ERR(CAM_OIS, " OIS Power up failed");
				goto rel_pkt;
			}
			o_ctrl->cam_ois_state = CAM_OIS_CONFIG;
		}

		if (o_ctrl->ois_fw_flag) {
			rc = cam_ois_fw_download(o_ctrl);
			if (rc) {
				CAM_ERR(CAM_OIS, "Failed OIS FW Download");
				goto pwr_dwn;
			}
		}

		rc = cam_ois_apply_settings(o_ctrl, &o_ctrl->i2c_init_data);
		if (rc < 0) {
			CAM_ERR(CAM_OIS, "Cannot apply Init settings");
			goto pwr_dwn;
		}

		if (o_ctrl->is_ois_calib) {
			rc = cam_ois_apply_settings(o_ctrl,
				&o_ctrl->i2c_calib_data);
			if (rc) {
				CAM_ERR(CAM_OIS, "Cannot apply calib data");
				goto pwr_dwn;
			}
		}

		rc = delete_request(&o_ctrl->i2c_init_data);
		if (rc < 0) {
			CAM_WARN(CAM_OIS,
				"Fail deleting Init data: rc: %d", rc);
			rc = 0;
		}
		rc = delete_request(&o_ctrl->i2c_calib_data);
		if (rc < 0) {
			CAM_WARN(CAM_OIS,
				"Fail deleting Calibration data: rc: %d", rc);
			rc = 0;
		}
		break;
	case CAM_OIS_PACKET_OPCODE_OIS_CONTROL:
		if (o_ctrl->cam_ois_state < CAM_OIS_CONFIG) {
			rc = -EINVAL;
			CAM_WARN(CAM_OIS,
				"Not in right state to control OIS: %d",
				o_ctrl->cam_ois_state);
			goto rel_pkt;
		}
		offset = (uint32_t *)&csl_packet->payload;
		offset += (csl_packet->cmd_buf_offset / sizeof(uint32_t));
		cmd_desc = (struct cam_cmd_buf_desc *)(offset);
		i2c_reg_settings = &(o_ctrl->i2c_mode_data);
		i2c_reg_settings->is_settings_valid = 1;
		i2c_reg_settings->request_id = 0;
		rc = cam_sensor_i2c_command_parser(&o_ctrl->io_master_info,
			i2c_reg_settings,
			cmd_desc, 1);
		if (rc < 0) {
			CAM_ERR(CAM_OIS, "OIS pkt parsing failed: %d", rc);
			goto rel_pkt;
		}
		imx586_cci_master = (enum cci_i2c_master_t)o_ctrl->cci_master_id;
		if (imx586_cci_master == o_ctrl->io_master_info.cci_client->cci_i2c_master && !imx586_ois_ready) {
			retry_cnt = 10;
			do {
				RamRead32A(o_ctrl, 0xF100, &temp);
				CAM_ERR(CAM_OIS, "read imx586 0xF100 = 0x%x", temp);
				if (temp == 0)
				{
					imx586_ois_ready = true;
					break;
				}
				retry_cnt--;
				msleep(10);
			} while(retry_cnt);
		}
		if (MASTER_0 == o_ctrl->io_master_info.cci_client->cci_i2c_master && !s5k3m5_ois_ready) {
			retry_cnt = 10;
			do {
				RamRead32A(o_ctrl, 0xF100, &temp);
				CAM_ERR(CAM_OIS, "read s5k3m5 0xF100 = 0x%x", temp);
				if (temp == 0)
				{
					s5k3m5_ois_ready = true;
					break;
				}
				retry_cnt--;
				msleep(10);
			} while(retry_cnt);
		}

		rc = cam_ois_apply_settings(o_ctrl, i2c_reg_settings);
		if (rc < 0) {
			CAM_ERR(CAM_OIS, "Cannot apply mode settings");
			goto rel_pkt;
		}

		rc = delete_request(i2c_reg_settings);
		if (rc < 0) {
			CAM_ERR(CAM_OIS,
				"Fail deleting Mode data: rc: %d", rc);
			goto rel_pkt;
		}
		break;
	default:
		CAM_ERR(CAM_OIS, "Invalid Opcode: %d",
			(csl_packet->header.op_code & 0xFFFFFF));
		rc = -EINVAL;
		goto rel_pkt;
	}

	if (!rc)
		goto rel_pkt;

rel_cmd_buf:
	if (cam_mem_put_cpu_buf(cmd_desc[i].mem_handle))
		CAM_WARN(CAM_OIS, "Failed to put cpu buf: 0x%x",
			cmd_desc[i].mem_handle);
pwr_dwn:
	cam_ois_power_down(o_ctrl);
rel_pkt:
	if (cam_mem_put_cpu_buf(dev_config.packet_handle))
		CAM_WARN(CAM_OIS, "Fail in put buffer: 0x%x",
			dev_config.packet_handle);

	return rc;
}

void cam_ois_shutdown(struct cam_ois_ctrl_t *o_ctrl)
{
	int rc = 0;
#ifndef ENABLE_OIS_DELAY_POWER_DOWN
	struct cam_ois_soc_private *soc_private =
		(struct cam_ois_soc_private *)o_ctrl->soc_info.soc_private;
	struct cam_sensor_power_ctrl_t *power_info = &soc_private->power_info;
#endif

	if (o_ctrl->cam_ois_state == CAM_OIS_INIT)
		return;

	if (o_ctrl->cam_ois_state >= CAM_OIS_CONFIG) {
#ifdef ENABLE_OIS_DELAY_POWER_DOWN
		mutex_lock(&(o_ctrl->ois_power_down_mutex));
		if (o_ctrl->ois_power_state == CAM_OIS_POWER_ON && o_ctrl->ois_power_down_thread_state == CAM_OIS_POWER_DOWN_THREAD_STOPPED) {
			o_ctrl->ois_power_down_thread_exit = false;
			kthread_run(ois_power_down_thread, o_ctrl, "ois_power_down_thread");
			CAM_ERR(CAM_OIS, "ois_power_down_thread created");
		} else {
			CAM_ERR(CAM_OIS, "no need to create ois_power_down_thread, ois_power_state %d, ois_power_down_thread_state %d", o_ctrl->ois_power_state, o_ctrl->ois_power_down_thread_state);
		}
		mutex_unlock(&(o_ctrl->ois_power_down_mutex));
#else
		rc = cam_ois_power_down(o_ctrl);
#endif
		if (rc < 0)
			CAM_ERR(CAM_OIS, "OIS Power down failed");
		o_ctrl->cam_ois_state = CAM_OIS_ACQUIRE;
	}

	if (o_ctrl->cam_ois_state >= CAM_OIS_ACQUIRE) {
		rc = cam_destroy_device_hdl(o_ctrl->bridge_intf.device_hdl);
		if (rc < 0)
			CAM_ERR(CAM_OIS, "destroying the device hdl");
		o_ctrl->bridge_intf.device_hdl = -1;
		o_ctrl->bridge_intf.link_hdl = -1;
		o_ctrl->bridge_intf.session_hdl = -1;
	}

	if (o_ctrl->i2c_mode_data.is_settings_valid == 1)
		delete_request(&o_ctrl->i2c_mode_data);

	if (o_ctrl->i2c_calib_data.is_settings_valid == 1)
		delete_request(&o_ctrl->i2c_calib_data);

	if (o_ctrl->i2c_init_data.is_settings_valid == 1)
		delete_request(&o_ctrl->i2c_init_data);

#ifndef ENABLE_OIS_DELAY_POWER_DOWN
	kfree(power_info->power_setting);
	kfree(power_info->power_down_setting);
	power_info->power_setting = NULL;
	power_info->power_down_setting = NULL;
	power_info->power_down_setting_size = 0;
	power_info->power_setting_size = 0;
#endif

	o_ctrl->cam_ois_state = CAM_OIS_INIT;
}

/**
 * cam_ois_driver_cmd - Handle ois cmds
 * @e_ctrl:     ctrl structure
 * @arg:        Camera control command argument
 *
 * Returns success or failure
 */
int cam_ois_driver_cmd(struct cam_ois_ctrl_t *o_ctrl, void *arg)
{
	int                              rc = 0;
	struct cam_ois_query_cap_t       ois_cap = {0};
	struct cam_control              *cmd = (struct cam_control *)arg;
#ifndef ENABLE_OIS_DELAY_POWER_DOWN
	struct cam_ois_soc_private      *soc_private = NULL;
	struct cam_sensor_power_ctrl_t  *power_info = NULL;
#endif

	uint32_t testAddr  = 61458;
	uint32_t testData  = 0;

	if (!o_ctrl || !cmd) {
		CAM_ERR(CAM_OIS, "Invalid arguments");
		return -EINVAL;
	}

	if (cmd->handle_type != CAM_HANDLE_USER_POINTER) {
		CAM_ERR(CAM_OIS, "Invalid handle type: %d",
			cmd->handle_type);
		return -EINVAL;
	}

	if(MASTER_1 == o_ctrl->io_master_info.cci_client->cci_i2c_master && ctrl_wide == NULL)
	{
		ctrl_wide = o_ctrl; //record wide device info
	}
	else if(MASTER_0 == o_ctrl->io_master_info.cci_client->cci_i2c_master && ctrl_tele == NULL)
	{
		ctrl_tele = o_ctrl; //record tele device info
	}

#ifndef ENABLE_OIS_DELAY_POWER_DOWN
	soc_private =
		(struct cam_ois_soc_private *)o_ctrl->soc_info.soc_private;
	power_info = &soc_private->power_info;
#endif

	mutex_lock(&(o_ctrl->ois_mutex));
	switch (cmd->op_code) {
	case CAM_QUERY_CAP:
		ois_cap.slot_info = o_ctrl->soc_info.index;

		if (copy_to_user(u64_to_user_ptr(cmd->handle),
			&ois_cap,
			sizeof(struct cam_ois_query_cap_t))) {
			CAM_ERR(CAM_OIS, "Failed Copy to User");
			rc = -EFAULT;
			goto release_mutex;
		}
		CAM_DBG(CAM_OIS, "ois_cap: ID: %d", ois_cap.slot_info);
		break;
	case CAM_ACQUIRE_DEV:
		rc = cam_ois_get_dev_handle(o_ctrl, arg);
		if (rc) {
			CAM_ERR(CAM_OIS, "Failed to acquire dev");
			goto release_mutex;
		}

		o_ctrl->cam_ois_state = CAM_OIS_ACQUIRE;
		break;
	case CAM_START_DEV:
		if (o_ctrl->cam_ois_state != CAM_OIS_CONFIG) {
			rc = -EINVAL;
			CAM_WARN(CAM_OIS,
			"Not in right state for start : %d",
			o_ctrl->cam_ois_state);
			goto release_mutex;
		}
		o_ctrl->cam_ois_state = CAM_OIS_START;
		break;
	case CAM_CONFIG_DEV:
		CAM_INFO(CAM_OIS, "CAM_CONFIG_DEV for %s", o_ctrl->ois_name);
		rc = cam_ois_pkt_parse(o_ctrl, arg);

		camera_io_dev_read((&o_ctrl->io_master_info),
		testAddr, &testData, 2, 4);
		CAM_INFO(CAM_OIS, "OISAddr 0x%x, value read %d for %s",
		testAddr, testData, o_ctrl->ois_name);

		if (rc) {
			CAM_ERR(CAM_OIS, "Failed in ois pkt Parsing");
			goto release_mutex;
		}
		break;
	case CAM_RELEASE_DEV:
		if (o_ctrl->cam_ois_state == CAM_OIS_START) {
			rc = -EINVAL;
			CAM_WARN(CAM_OIS,
				"Cant release ois: in start state");
			goto release_mutex;
		}

		if (o_ctrl->cam_ois_state == CAM_OIS_CONFIG) {
#ifdef ENABLE_OIS_DELAY_POWER_DOWN
			mutex_lock(&(o_ctrl->ois_power_down_mutex));
			if (o_ctrl->ois_power_state == CAM_OIS_POWER_ON && o_ctrl->ois_power_down_thread_state == CAM_OIS_POWER_DOWN_THREAD_STOPPED) {
				o_ctrl->ois_power_down_thread_exit = false;
				kthread_run(ois_power_down_thread, o_ctrl, "ois_power_down_thread");
				CAM_ERR(CAM_OIS, "ois_power_down_thread created");
			} else {
				CAM_ERR(CAM_OIS, "no need to create ois_power_down_thread, ois_power_state %d, ois_power_down_thread_state %d", o_ctrl->ois_power_state, o_ctrl->ois_power_down_thread_state);
			}
			mutex_unlock(&(o_ctrl->ois_power_down_mutex));
#else
			rc = cam_ois_power_down(o_ctrl);
#endif
			if (rc < 0) {
				CAM_ERR(CAM_OIS, "OIS Power down failed");
				goto release_mutex;
			}
		}

		if (o_ctrl->bridge_intf.device_hdl == -1) {
			CAM_ERR(CAM_OIS, "link hdl: %d device hdl: %d",
				o_ctrl->bridge_intf.device_hdl,
				o_ctrl->bridge_intf.link_hdl);
			rc = -EINVAL;
			goto release_mutex;
		}
		rc = cam_destroy_device_hdl(o_ctrl->bridge_intf.device_hdl);
		if (rc < 0)
			CAM_ERR(CAM_OIS, "destroying the device hdl");
		o_ctrl->bridge_intf.device_hdl = -1;
		o_ctrl->bridge_intf.link_hdl = -1;
		o_ctrl->bridge_intf.session_hdl = -1;
		o_ctrl->cam_ois_state = CAM_OIS_INIT;

#ifndef ENABLE_OIS_DELAY_POWER_DOWN
		kfree(power_info->power_setting);
		kfree(power_info->power_down_setting);
		power_info->power_setting = NULL;
		power_info->power_down_setting = NULL;
		power_info->power_down_setting_size = 0;
		power_info->power_setting_size = 0;
#endif

		if (o_ctrl->i2c_mode_data.is_settings_valid == 1)
			delete_request(&o_ctrl->i2c_mode_data);

		if (o_ctrl->i2c_calib_data.is_settings_valid == 1)
			delete_request(&o_ctrl->i2c_calib_data);

		if (o_ctrl->i2c_init_data.is_settings_valid == 1)
			delete_request(&o_ctrl->i2c_init_data);

		break;
	case CAM_STOP_DEV:
		if (o_ctrl->cam_ois_state != CAM_OIS_START) {
			rc = -EINVAL;
			CAM_WARN(CAM_OIS,
			"Not in right state for stop : %d",
			o_ctrl->cam_ois_state);
		}
		o_ctrl->cam_ois_state = CAM_OIS_CONFIG;
		break;
	default:
		CAM_ERR(CAM_OIS, "invalid opcode");
		goto release_mutex;
	}
release_mutex:
	mutex_unlock(&(o_ctrl->ois_mutex));
	return rc;
}
