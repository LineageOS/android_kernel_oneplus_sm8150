#ifndef _CAM_OIS_OPLUS_H_
#define _CAM_OIS_OPLUS_H_

#include "ois_fw/Ois.h"

//18821 master semco
#include "ois_fw/LC898124EP3_Code_0_1_2_2_0_0.h"		// Gyro=LSM6DSM,	SPI Maste	SO2820_T1
#include "ois_fw/LC898124EP3_Code_0_1_3_2_1_0.h"		// Gyro=LSM6DSM,	SPI Slave	SO3600_T1

//18821 master ofilm
#include "ois_fw/LC898124EP3_Code_0_2_6_2_0_0.h"		// Gyro=LSM6DSM,	SPI Maste	SO2820_T1
#include "ois_fw/LC898124EP3_Code_0_2_6_2_0_1.h"		// Gyro=LSM6DSM,	SPI Slave	SO3600_T1

//18821 slave semco
#include "ois_fw/LC898124EP3_Code_0_1_2_3_0_0.h"		// Gyro=BMI160,		SPI Maste	SO2820_T1
#include "ois_fw/LC898124EP3_Code_0_1_3_3_1_0.h"		// Gyro=BMI160,		SPI Slave	SO3600_T1

//18821 slave ofilm
#include "ois_fw/LC898124EP3_Code_0_2_7_2_1_0.h"		// Gyro=BMI160,		SPI Maste	SO2820_T1
#include "ois_fw/LC898124EP3_Code_0_2_7_2_1_1.h"		// Gyro=BMI160,		SPI Slave	SO3600_T1

//18857 semco
#include "ois_fw/LC898124EP3_Code_0_1_0_2_2_0.h"		// Gyro=LSM6DSM,	SO2821
#include "ois_fw/LC898124EP3_Code_0_1_0_2_2_1.h"		// Gyro=LSM6DSM,	FRA, SO2821

//18821 Servo on master semco
#include "ois_fw/LC898124EP3_Servo_On_Code_0_1_2_2_0_0.h"		// Gyro=LSM6DSM,	SPI Maste	SO2820_T1
#include "ois_fw/LC898124EP3_Servo_On_Code_0_1_3_2_1_0.h"		// Gyro=LSM6DSM,	SPI Slave	SO3600_T1

//18821 Servo on master ofilm
#include "ois_fw/LC898124EP3_Servo_On_Code_0_2_6_2_0_0.h"		// Gyro=LSM6DSM,	SPI Maste	SO2820_T1

//18821 Servo on slave semco
#include "ois_fw/LC898124EP3_Servo_On_Code_0_1_2_3_0_0.h"		// Gyro=BMI160,		SPI Maste	SO2820_T1
#include "ois_fw/LC898124EP3_Servo_On_Code_0_1_3_3_1_0.h"		// Gyro=BMI160,		SPI Slave	SO3600_T1

//18821 Servo on slave ofilm
#include "ois_fw/LC898124EP3_Servo_On_Code_0_2_7_2_1_0.h"		// Gyro=BMI160,		SPI Maste	SO2820_T1

//18857 Servo on semco
#include "ois_fw/LC898124EP3_Servo_On_Code_0_1_0_2_2_0.h"		// Gyro=LSM6DSM,	SO2821
#include "ois_fw/LC898124EP3_Servo_On_Code_0_1_0_2_2_1.h"		// Gyro=LSM6DSM,	FRA, SO2821


//19801 Master semco
#include "ois_fw/LC898124EP3_Code_1_1_2_2_0_0.h"        // Gyro back
#include "ois_fw/LC898124EP3_Code_2_1_2_2_0_0.h"        // Gyro front

//19801 Slave semco
#include "ois_fw/LC898124EP3_Code_1_1_3_2_1_0.h"        // Gyro back
#include "ois_fw/LC898124EP3_Code_2_1_3_2_1_0.h"        // Gyro front

//19801 Master ofilm
#include "ois_fw/LC898124EP3_Code_1_2_6_2_0_0.h"                 // Gyro=LSM6DSM,        SPI Maste            M12337        Gyro back
#include "ois_fw/LC898124EP3_Code_2_2_6_2_0_0.h"                 // Gyro=LSM6DSM,        SPI Maste            M12337        Gyro front

//19801 Slave ofilm
#include "ois_fw/LC898124EP3_Code_1_2_7_2_1_0.h"                 // Gyro=LSM6DSM,        SPI Slave            M10235        Gyro back
#include "ois_fw/LC898124EP3_Code_2_2_7_2_1_0.h"                 // Gyro=LSM6DSM,        SPI Slave            M10235        Gyro front

//18865 semco
#include "ois_fw/LC898124EP3_Code_1_1_0_2_2_0.h"		// Gyro=LSM6DSM,	SO2823

//18865 ofilm
#include "ois_fw/LC898124EP3_Code_1_2_1_2_2_0.h"		 // Gyro=LSM6DSM,	                      M12337

static struct cam_sensor_i2c_reg_array *i2c_write_setting_gl = NULL;

typedef struct {
	INT32			SiSampleNum;			// Measure Sample Number
	INT32			SiSampleMax;			// Measure Sample Number Max

	struct {
		INT32		SiMax1;				// Max Measure Result
		INT32		SiMin1;				// Min Measure Result
		UINT32  	UiAmp1;				// Amplitude Measure Result
		INT64		LLiIntegral1;			// Integration Measure Result
		INT64		LLiAbsInteg1;			// Absolute Integration Measure Result
		INT32		PiMeasureRam1;			// Measure Delay RAM Address
	} MeasureFilterA;

	struct {
		INT32		SiMax2;				// Max Measure Result
		INT32		SiMin2;				// Min Measure Result
		UINT32	        UiAmp2;				// Amplitude Measure Result
		INT64		LLiIntegral2;			// Integration Measure Result
		INT64		LLiAbsInteg2;			// Absolute Integration Measure Result
		INT32		PiMeasureRam2;			// Measure Delay RAM Address
	} MeasureFilterB;
} MeasureFunction_Type;

//**************************
//	define
//**************************

#define BURST_LENGTH_PM (12*5)
#define BURST_LENGTH_DM (10*6)
#define BURST_LENGTH BURST_LENGTH_PM

#define	GYROF_NUM	2048	// 2048times

#define MAX_DATA_NUM	64
#define ONE_MSEC_COUNT	18	// 18.0288kHz * 18 ¨P 1ms

#define LOOPGAIN	1
#define THROUGH		2
#define NOISE		3
#define OSCCHK		4

#define MODEL_0		1
#define MODEL_0_SERVO	2
#define MODEL_1		3
#define MODEL_2		4


//18857
const DOWNLOAD_TBL DTbl[] = {
	{0x0002, MODEL_0      , LC898124EP3_PM_0_1_0_2_2_0, LC898124EP3_PMSize_0_1_0_2_2_0, (UINT32)((UINT32)LC898124EP3_PMCheckSum_0_1_0_2_2_0 + (UINT32)LC898124EP3_DMA_CheckSum_0_1_0_2_2_0 + (UINT32)LC898124EP3_DMB_CheckSum_0_1_0_2_2_0), LC898124EP3_DM_0_1_0_2_2_0, LC898124EP3_DMA_ByteSize_0_1_0_2_2_0, LC898124EP3_DMB_ByteSize_0_1_0_2_2_0 },
	{0x0082, MODEL_0      , LC898124EP3_PM_0_1_0_2_2_1, LC898124EP3_PMSize_0_1_0_2_2_1, (UINT32)((UINT32)LC898124EP3_PMCheckSum_0_1_0_2_2_1 + (UINT32)LC898124EP3_DMA_CheckSum_0_1_0_2_2_1 + (UINT32)LC898124EP3_DMB_CheckSum_0_1_0_2_2_1), LC898124EP3_DM_0_1_0_2_2_1, LC898124EP3_DMA_ByteSize_0_1_0_2_2_1, LC898124EP3_DMB_ByteSize_0_1_0_2_2_1 },
	{0x0002, MODEL_0_SERVO, LC898124EP3_SERVO_ON_PM_0_1_0_2_2_0, LC898124EP3_SERVO_ON_PMSize_0_1_0_2_2_0, (UINT32)((UINT32)LC898124EP3_SERVO_ON_PMCheckSum_0_1_0_2_2_0 + (UINT32)LC898124EP3_SERVO_ON_DMA_CheckSum_0_1_2_2_0_0 + (UINT32)LC898124EP3_SERVO_ON_DMB_CheckSum_0_1_0_2_2_0), LC898124EP3_SERVO_ON_DM_0_1_0_2_2_0, LC898124EP3_SERVO_ON_DMA_ByteSize_0_1_0_2_2_0, LC898124EP3_SERVO_ON_DMB_ByteSize_0_1_0_2_2_0 },
	{0x0002, MODEL_1      , LC898124EP3_PM_1_1_0_2_2_0, LC898124EP3_PMSize_1_1_0_2_2_0, (UINT32)((UINT32)LC898124EP3_PMCheckSum_1_1_0_2_2_0 + (UINT32)LC898124EP3_DMA_CheckSum_1_1_0_2_2_0 + (UINT32)LC898124EP3_DMB_CheckSum_1_1_0_2_2_0), LC898124EP3_DM_1_1_0_2_2_0, LC898124EP3_DMA_ByteSize_1_1_0_2_2_0, LC898124EP3_DMB_ByteSize_1_1_0_2_2_0 },
	{0x0102, MODEL_1      , LC898124EP3_PM_1_2_1_2_2_0, LC898124EP3_PMSize_1_2_1_2_2_0, (UINT32)((UINT32)LC898124EP3_PMCheckSum_1_2_1_2_2_0 + (UINT32)LC898124EP3_DMA_CheckSum_1_2_1_2_2_0 + (UINT32)LC898124EP3_DMB_CheckSum_1_2_1_2_2_0), LC898124EP3_DM_1_2_1_2_2_0, LC898124EP3_DMA_ByteSize_1_2_1_2_2_0, LC898124EP3_DMB_ByteSize_1_2_1_2_2_0 },
	{0xFFFF, MODEL_0      , (void*)0, 0, 0, (void*)0,0,0 }
};

//18821 master
const DOWNLOAD_TBL DTbl_M[] = {
	{0x0202, MODEL_0      , LC898124EP3_PM_0_1_2_2_0_0, LC898124EP3_PMSize_0_1_2_2_0_0, (UINT32)((UINT32)LC898124EP3_PMCheckSum_0_1_2_2_0_0 + (UINT32)LC898124EP3_DMA_CheckSum_0_1_2_2_0_0 + (UINT32)LC898124EP3_DMB_CheckSum_0_1_2_2_0_0), LC898124EP3_DM_0_1_2_2_0_0, LC898124EP3_DMA_ByteSize_0_1_2_2_0_0, LC898124EP3_DMB_ByteSize_0_1_2_2_0_0 },
	{0x0203, MODEL_0      , LC898124EP3_PM_0_1_2_3_0_0, LC898124EP3_PMSize_0_1_2_3_0_0, (UINT32)((UINT32)LC898124EP3_PMCheckSum_0_1_2_3_0_0 + (UINT32)LC898124EP3_DMA_CheckSum_0_1_2_3_0_0 + (UINT32)LC898124EP3_DMB_CheckSum_0_1_2_3_0_0), LC898124EP3_DM_0_1_2_3_0_0, LC898124EP3_DMA_ByteSize_0_1_2_3_0_0, LC898124EP3_DMB_ByteSize_0_1_2_3_0_0 },
	{0x0602, MODEL_0      , LC898124EP3_PM_0_2_6_2_0_0, LC898124EP3_PMSize_0_2_6_2_0_0, (UINT32)((UINT32)LC898124EP3_PMCheckSum_0_2_6_2_0_0 + (UINT32)LC898124EP3_DMA_CheckSum_0_2_6_2_0_0 + (UINT32)LC898124EP3_DMB_CheckSum_0_2_6_2_0_0), LC898124EP3_DM_0_2_6_2_0_0, LC898124EP3_DMA_ByteSize_0_2_6_2_0_0, LC898124EP3_DMB_ByteSize_0_2_6_2_0_0 },
	{0x0202, MODEL_0_SERVO, LC898124EP3_SERVO_ON_PM_0_1_2_2_0_0, LC898124EP3_SERVO_ON_PMSize_0_1_2_2_0_0, (UINT32)((UINT32)LC898124EP3_SERVO_ON_PMCheckSum_0_1_2_2_0_0 + (UINT32)LC898124EP3_SERVO_ON_DMA_CheckSum_0_1_2_2_0_0 + (UINT32)LC898124EP3_SERVO_ON_DMB_CheckSum_0_1_2_2_0_0), LC898124EP3_SERVO_ON_DM_0_1_2_2_0_0, LC898124EP3_SERVO_ON_DMA_ByteSize_0_1_2_2_0_0, LC898124EP3_SERVO_ON_DMB_ByteSize_0_1_2_2_0_0 },
	{0x0602, MODEL_0_SERVO, LC898124EP3_SERVO_ON_PM_0_2_6_2_0_0, LC898124EP3_SERVO_ON_PMSize_0_2_6_2_0_0, (UINT32)((UINT32)LC898124EP3_SERVO_ON_PMCheckSum_0_2_6_2_0_0 + (UINT32)LC898124EP3_SERVO_ON_DMA_CheckSum_0_2_6_2_0_0 + (UINT32)LC898124EP3_SERVO_ON_DMB_CheckSum_0_2_6_2_0_0), LC898124EP3_SERVO_ON_DM_0_2_6_2_0_0, LC898124EP3_SERVO_ON_DMA_ByteSize_0_2_6_2_0_0, LC898124EP3_SERVO_ON_DMB_ByteSize_0_2_6_2_0_0 },
	{0x0202, MODEL_1      , LC898124EP3_PM_1_1_2_2_0_0, LC898124EP3_PMSize_1_1_2_2_0_0, (UINT32)((UINT32)LC898124EP3_PMCheckSum_1_1_2_2_0_0 + (UINT32)LC898124EP3_DMA_CheckSum_1_1_2_2_0_0 + (UINT32)LC898124EP3_DMB_CheckSum_1_1_2_2_0_0), LC898124EP3_DM_1_1_2_2_0_0, LC898124EP3_DMA_ByteSize_1_1_2_2_0_0, LC898124EP3_DMB_ByteSize_1_1_2_2_0_0 },
	{0x0202, MODEL_2      , LC898124EP3_PM_2_1_2_2_0_0, LC898124EP3_PMSize_2_1_2_2_0_0, (UINT32)((UINT32)LC898124EP3_PMCheckSum_2_1_2_2_0_0 + (UINT32)LC898124EP3_DMA_CheckSum_2_1_2_2_0_0 + (UINT32)LC898124EP3_DMB_CheckSum_2_1_2_2_0_0), LC898124EP3_DM_2_1_2_2_0_0, LC898124EP3_DMA_ByteSize_2_1_2_2_0_0, LC898124EP3_DMB_ByteSize_2_1_2_2_0_0 },
	{0xFFFF, 1, (void*)0, 0, 0, (void*)0,0,0 }
};

//18821 slave
const DOWNLOAD_TBL DTbl_S[] = {
	{0x0302, MODEL_0      , LC898124EP3_PM_0_1_3_2_1_0, LC898124EP3_PMSize_0_1_3_2_1_0, (UINT32)((UINT32)LC898124EP3_PMCheckSum_0_1_3_2_1_0 + (UINT32)LC898124EP3_DMA_CheckSum_0_1_3_2_1_0 + (UINT32)LC898124EP3_DMB_CheckSum_0_1_3_2_1_0), LC898124EP3_DM_0_1_3_2_1_0, LC898124EP3_DMA_ByteSize_0_1_3_2_1_0, LC898124EP3_DMB_ByteSize_0_1_3_2_1_0 },
	{0x0303, MODEL_0      , LC898124EP3_PM_0_1_3_3_1_0, LC898124EP3_PMSize_0_1_3_3_1_0, (UINT32)((UINT32)LC898124EP3_PMCheckSum_0_1_3_3_1_0 + (UINT32)LC898124EP3_DMA_CheckSum_0_1_3_3_1_0 + (UINT32)LC898124EP3_DMB_CheckSum_0_1_3_3_1_0), LC898124EP3_DM_0_1_3_3_1_0, LC898124EP3_DMA_ByteSize_0_1_3_3_1_0, LC898124EP3_DMB_ByteSize_0_1_3_3_1_0 },
	{0x0702, MODEL_0      , LC898124EP3_PM_0_2_7_2_1_0, LC898124EP3_PMSize_0_2_7_2_1_0, (UINT32)((UINT32)LC898124EP3_PMCheckSum_0_2_7_2_1_0 + (UINT32)LC898124EP3_DMA_CheckSum_0_2_7_2_1_0 + (UINT32)LC898124EP3_DMB_CheckSum_0_2_7_2_1_0), LC898124EP3_DM_0_2_7_2_1_0, LC898124EP3_DMA_ByteSize_0_2_7_2_1_0, LC898124EP3_DMB_ByteSize_0_2_7_2_1_0 },
	{0x0302, MODEL_0_SERVO, LC898124EP3_SERVO_ON_PM_0_1_3_2_1_0, LC898124EP3_SERVO_ON_PMSize_0_1_3_2_1_0, (UINT32)((UINT32)LC898124EP3_SERVO_ON_PMCheckSum_0_1_3_2_1_0 + (UINT32)LC898124EP3_SERVO_ON_DMA_CheckSum_0_1_3_2_1_0 + (UINT32)LC898124EP3_SERVO_ON_DMB_CheckSum_0_1_3_2_1_0), LC898124EP3_SERVO_ON_DM_0_1_3_2_1_0, LC898124EP3_SERVO_ON_DMA_ByteSize_0_1_3_2_1_0, LC898124EP3_SERVO_ON_DMB_ByteSize_0_1_3_2_1_0 },
	{0x0702, MODEL_0_SERVO, LC898124EP3_SERVO_ON_PM_0_2_7_2_1_0, LC898124EP3_SERVO_ON_PMSize_0_2_7_2_1_0, (UINT32)((UINT32)LC898124EP3_SERVO_ON_PMCheckSum_0_2_7_2_1_0 + (UINT32)LC898124EP3_SERVO_ON_DMA_CheckSum_0_2_7_2_1_0 + (UINT32)LC898124EP3_SERVO_ON_DMB_CheckSum_0_2_7_2_1_0), LC898124EP3_SERVO_ON_DM_0_2_7_2_1_0, LC898124EP3_SERVO_ON_DMA_ByteSize_0_2_7_2_1_0, LC898124EP3_SERVO_ON_DMB_ByteSize_0_2_7_2_1_0 },
	{0x0302, MODEL_1      , LC898124EP3_PM_1_1_3_2_1_0, LC898124EP3_PMSize_1_1_3_2_1_0, (UINT32)((UINT32)LC898124EP3_PMCheckSum_1_1_3_2_1_0 + (UINT32)LC898124EP3_DMA_CheckSum_1_1_3_2_1_0 + (UINT32)LC898124EP3_DMB_CheckSum_1_1_3_2_1_0), LC898124EP3_DM_1_1_3_2_1_0, LC898124EP3_DMA_ByteSize_1_1_3_2_1_0, LC898124EP3_DMB_ByteSize_1_1_3_2_1_0 },
	{0x0302, MODEL_2      , LC898124EP3_PM_2_1_3_2_1_0, LC898124EP3_PMSize_2_1_3_2_1_0, (UINT32)((UINT32)LC898124EP3_PMCheckSum_2_1_3_2_1_0 + (UINT32)LC898124EP3_DMA_CheckSum_2_1_3_2_1_0 + (UINT32)LC898124EP3_DMB_CheckSum_2_1_3_2_1_0), LC898124EP3_DM_2_1_3_2_1_0, LC898124EP3_DMA_ByteSize_2_1_3_2_1_0, LC898124EP3_DMB_ByteSize_2_1_3_2_1_0 },
	{0xFFFF, 1, (void*)0, 0, 0, (void*)0,0,0 }
};

static int RamWrite32A(struct cam_ois_ctrl_t *o_ctrl,
	UINT32 addr, UINT32 data)
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

	for (i = 0; i < retry; i++)
	{
		rc = camera_io_dev_write(&(o_ctrl->io_master_info), &i2c_write);
		if (rc < 0) {
			CAM_ERR(CAM_OIS, "write 0x%04x failed, retry:%d", addr, i+1);
		} else {
			return rc;
		}
	}
	return rc;
}

static int RamRead32A(struct cam_ois_ctrl_t *o_ctrl,
	UINT32 addr, UINT32* data)
{
	int32_t rc = 0;
	int retry = 3;
	int i;

	if (o_ctrl == NULL) {
		CAM_ERR(CAM_OIS, "Invalid Args");
		return -EINVAL;
	}
	for (i = 0; i < retry; i++)
	{
		rc = camera_io_dev_read(&(o_ctrl->io_master_info), 
			(uint32_t)addr, (uint32_t *)data,
			CAMERA_SENSOR_I2C_TYPE_WORD, CAMERA_SENSOR_I2C_TYPE_DWORD);
		if (rc < 0) {
			CAM_ERR(CAM_OIS, "read 0x%04x failed, retry:%d", addr, i+1);
		} else {
			return rc;
		}
	}
	return rc;
}

static void CntWrt(struct cam_ois_ctrl_t *o_ctrl,
	UINT8 *data, UINT16 size)
{
	int32_t rc = 0;
	int i = 0;
	int reg_data_cnt = size - 1;
	int continue_cnt = 0;
	int retry = 3;
	struct cam_sensor_i2c_reg_setting i2c_write;

	if (o_ctrl == NULL) {
		CAM_ERR(CAM_OIS, "Invalid Args");
		return;
	}

	if (i2c_write_setting_gl == NULL) {
		i2c_write_setting_gl = (struct cam_sensor_i2c_reg_array *)kzalloc(
		sizeof(struct cam_sensor_i2c_reg_array)*MAX_DATA_NUM, GFP_KERNEL);
		if (!i2c_write_setting_gl) {
			CAM_ERR(CAM_OIS, "Alloc i2c_write_setting_gl failed");
			return;
		}
	}

	for (i = 0; i< reg_data_cnt; i++) {
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

	for (i = 0; i < retry; i++)
	{
		rc = camera_io_dev_write_continuous(&(o_ctrl->io_master_info),
		&i2c_write, 1);
		if (rc < 0) {
			CAM_ERR(CAM_OIS, "Continue write failed, rc:%d, retry:%d", rc, i+1);
		} else {
			break;
		}
	}
}

static void DMIOWrite32(struct cam_ois_ctrl_t *o_ctrl, UINT32 IOadrs, UINT32 IOdata)
{
	UINT8 data[10];
	data[0] = 0xC0;		// Pmem address set
	data[1] = 0x00;		// Command High
	data[2] = (UINT8)(IOadrs >>24);		// IOadres
	data[3] = (UINT8)(IOadrs >>16);		// Command High
	data[4] = (UINT8)(IOadrs >> 8);		// Command High
	data[5] = (UINT8)(IOadrs >> 0);		// Command High
	data[6] = (UINT8)(IOdata >>24);		// IOadres
	data[7] = (UINT8)(IOdata >>16);		// Command High
	data[8] = (UINT8)(IOdata >> 8);		// Command High
	data[9] = (UINT8)(IOdata >> 0);		// Command High
	CntWrt(o_ctrl, data, 10); 	// I2C 1Byte address.
};

//********************************************************************************
// Function Name 	: MonitorInfo
// Retun Value		: NON
// Argment Value	: NON
// Explanation		:
// History			: Second edition
//********************************************************************************
void MonitorInfo(DSPVER* Dspcode)
{
	CAM_INFO(CAM_OIS, "Vendor : %02x \n", Dspcode->Vendor);
	CAM_INFO(CAM_OIS, "User : %02x \n", Dspcode->User);
	CAM_INFO(CAM_OIS, "Model : %02x \n", Dspcode->Model);
	CAM_INFO(CAM_OIS, "Version : %02x \n", Dspcode->Version);

	if (Dspcode->SpiMode == SPI_MST)
		CAM_INFO(CAM_OIS, "spi mode : Master\n");
	if (Dspcode->SpiMode == SPI_SLV)
		CAM_INFO(CAM_OIS, "spi mode : Slave\n");
	if (Dspcode->SpiMode == SPI_SNGL)
		CAM_INFO(CAM_OIS, "spi mode : only master\n");

	if (Dspcode->ActType == ACT_SO2820) {
		CAM_INFO(CAM_OIS, "actuator type : SO2820\n");
	} else if (Dspcode->ActType == ACT_SO3600) {
		CAM_INFO(CAM_OIS, "actuator type : SO3600\n");
	} else {
		CAM_INFO(CAM_OIS, "actuator type : SOXXXX\n");
	}

	if (Dspcode->GyroType == GYRO_ICM20690)
		CAM_INFO(CAM_OIS, "gyro type : INVEN ICM20690 \n");
	if (Dspcode->GyroType == GYRO_LSM6DSM)
		CAM_INFO(CAM_OIS, "gyro type : ST LSM6DSM \n");
}

//********************************************************************************
// Function Name 	: GetInfomationBeforeDownlaod
// Retun Value		: True(0) / Fail(1)
// Argment Value	: NON
// Explanation		: <Pmem Memory> Write Data
// History			: First edition
//********************************************************************************
UINT8 GetInfomationBeforeDownload(DSPVER* Info, const UINT8* DataDM,  UINT32 LengthDM)
{
	UINT32 i;
	Info->ActType = 0;
	Info->GyroType = 0;

	for (i = 0; i < LengthDM; i += 6)
	{
		if ((DataDM[0+i] == 0xA0) && (DataDM[1+i] == 0x00))
		{
			Info->Vendor = DataDM[2+i];
			Info->User = DataDM[3+i];
			Info->Model = DataDM[4+i];
			Info->Version = DataDM[5+i];
			if ((DataDM[6+i] == 0xA0) && (DataDM[7+i] == 0x04))
			{
				Info->SpiMode = DataDM[8+i];
				Info->ActType = DataDM[10+i];
				Info->GyroType = DataDM[11+i];
			}
			MonitorInfo(Info);
			return 0;
		}
	}
	return(1);
}

//********************************************************************************
// Function Name 	: DownloadToEP3
// Retun Value		: NON
// Argment Value	: PMlength: 5byte unit, DMlength : 1Byte unit
// Explanation		: <Pmem Memory> Write Data
// History			: First edition
//********************************************************************************
unsigned char DownloadToEP3(struct cam_ois_ctrl_t *o_ctrl, const UINT8* DataPM,
	UINT32 LengthPM, UINT32 Parity, const UINT8* DataDM, UINT32 LengthDMA,
	UINT32 LengthDMB)
{
	UINT32 i, j;
	UINT8 data[MAX_DATA_NUM];		// work fifo buffer max size 64 byte
	UINT8 Remainder;
	UINT32 UlReadVal, UlCnt;
	UINT32 ReadVerifyPM = 0, ReadVerifyDMA = 0, ReadVerifyDMB = 0;	// Checksum
	UINT32 VerifySUM = 0;

//*******************************************************************************//
//*   pre-check ROM code version 												*//
//*******************************************************************************//
	RamRead32A(o_ctrl, CMD_ROMVER, &UlReadVal);
	if (UlReadVal == OLD_VER)
		return 3;		/* ROM code version error */

//--------------------------------------------------------------------------------
// 0. Start up to boot exection
//--------------------------------------------------------------------------------
	RamWrite32A(o_ctrl, CMD_IO_ADR_ACCESS, ROMINFO);
	RamRead32A(o_ctrl, CMD_IO_DAT_ACCESS, &UlReadVal);
	switch ((UINT8)UlReadVal) {
	case 0x0A:	/* Normal Rom program execution */
		break;
	case 0x01:	/* Normal Ram program execution */
		DMIOWrite32(o_ctrl, SYSDSP_REMAP, 0x00001000); 	// CORE_RST
		msleep(6);						// Boot 6msec
		break;
	default:
		return 1;
	}
//--------------------------------------------------------------------------------
// 1. Download Program
//--------------------------------------------------------------------------------
	data[0] = 0x30;		// Pmem address set
	data[1] = 0x00;		// Command High
	data[2] = 0x10;		// Command High
	data[3] = 0x00;		// Command High
	data[4] = 0x00;		// Command High
	CntWrt(o_ctrl, data, 5); 	// I2C 1Byte address.

	// program start
	data[0] = 0x40;		// Pmem address set
	Remainder = ((LengthPM*5) / BURST_LENGTH_PM);

	for (i = 0; i < Remainder; i++)
	{
		UlCnt = 1;
		for (j=0; j < BURST_LENGTH_PM; j++)
			data[UlCnt++] = *DataPM++;
		CntWrt(o_ctrl, data, BURST_LENGTH_PM+1);  // I2Caddresss 1Byte.
	}
	Remainder = ((LengthPM*5) % BURST_LENGTH_PM);
	if (Remainder != 0)
	{
		UlCnt = 1;
		for (j=0; j < Remainder; j++)	data[UlCnt++] = *DataPM++;
		CntWrt(o_ctrl, data, UlCnt);  // I2C 1Byte address.
	}

	// Checksum start
	data[0] = 0xF0;						// Pmem address set
	data[1] = 0x0A;						// Command High
	data[2] = (unsigned char)((LengthPM & 0xFF00) >> 8);	// Size High
	data[3] = (unsigned char)((LengthPM & 0x00FF) >> 0);	// Size Low
	CntWrt(o_ctrl, data, 4); 	// I2C 2Byte addresss.

//--------------------------------------------------------------------------------
// 2. Download Table Data
//--------------------------------------------------------------------------------
	RamWrite32A(o_ctrl, DmCheck_CheckSumDMA, 0);		// DMA Parity Clear
	RamWrite32A(o_ctrl, DmCheck_CheckSumDMB, 0);		// DMB Parity Clear

	/***** DMA Data Send *****/
	Remainder = ((LengthDMA*6/4) / BURST_LENGTH_DM);
	for (i=0; i< Remainder; i++)
	{
		CntWrt(o_ctrl, (UINT8*)DataDM, BURST_LENGTH_DM);	// I2Caddresss 1Byte.
		DataDM += BURST_LENGTH_DM;
	}
	Remainder = ((LengthDMA*6/4) % BURST_LENGTH_DM);
	if (Remainder != 0)
	{
	CntWrt(o_ctrl, (UINT8*)DataDM, (UINT8)Remainder);	// I2Caddresss 1Byte.
	}
	DataDM += Remainder;

	/***** DMB Data Send *****/
	Remainder = ((LengthDMB*6/4) / BURST_LENGTH_DM);
	for (i=0; i< Remainder; i++)
	{
		CntWrt(o_ctrl, (UINT8*)DataDM, BURST_LENGTH_DM);	// I2Caddresss 1Byte.
		DataDM += BURST_LENGTH_DM;
	}
	Remainder = ((LengthDMB*6/4) % BURST_LENGTH_DM);
	if (Remainder != 0)
	{
		CntWrt(o_ctrl, (UINT8*)DataDM, (UINT8)Remainder);	// I2Caddresss 1Byte.
	}

//--------------------------------------------------------------------------------
// 3. Verify
//--------------------------------------------------------------------------------
	RamRead32A(o_ctrl, PmCheck_CheckSum, &ReadVerifyPM);
	RamRead32A(o_ctrl, DmCheck_CheckSumDMA, &ReadVerifyDMA);
	RamRead32A(o_ctrl, DmCheck_CheckSumDMB, &ReadVerifyDMB);
	VerifySUM = ReadVerifyPM + ReadVerifyDMA + ReadVerifyDMB;
	if (VerifySUM == Parity) {
		CAM_ERR(CAM_OIS, "verify success. ReadVerifyPM=0x%x, ReadVerifyDMA=0x%x, ReadVerifyDMB=0x%x, VerifySUM=0x%x, Parity=0x%x",
		ReadVerifyPM, ReadVerifyDMA, ReadVerifyDMB, VerifySUM, Parity);
	} else {
		CAM_ERR(CAM_OIS, "verify fail. ReadVerifyPM=0x%x, ReadVerifyDMA=0x%x, ReadVerifyDMB=0x%x, VerifySUM=0x%x, Parity=0x%x",
		ReadVerifyPM, ReadVerifyDMA, ReadVerifyDMB, VerifySUM, Parity);
		return 2;
	}
	return 0;
}

unsigned char SelectDownload(struct cam_ois_ctrl_t *o_ctrl, UINT8 GyroSelect,
	UINT8 ActSelect, UINT8 MasterSlave, UINT8 FWType)
{
	DSPVER Dspcode;
	DOWNLOAD_TBL *ptr;
	CAM_INFO(CAM_OIS, "ois_name:%s, GyroSelect:0x%x, ActSelect:0x%x, MasterSlave:0x%x, FWType:%d\n",
		o_ctrl->ois_name, GyroSelect, ActSelect, MasterSlave, FWType);

	if (o_ctrl->ois_gyro_id==3) {
		ptr = (DOWNLOAD_TBL *)DTbl;
	} else {
		if (MasterSlave == 0x00) {
			ptr = (DOWNLOAD_TBL *)DTbl_M;
		} else {
			ptr = (DOWNLOAD_TBL *)DTbl_S;
		}
	}

	while (ptr->Cmd != 0xFFFF) {
		if ((ptr->Cmd == (((UINT16)ActSelect<<8) + GyroSelect)) && (ptr->FWType == FWType)) break;
		ptr++;
	}

	if (ptr->Cmd == 0xFFFF)
		return 0xF0;

	if (GetInfomationBeforeDownload(&Dspcode, ptr->DataDM, (ptr->LengthDMA +  ptr->LengthDMB)) != 0)
		return 0xF1;

	if ((ActSelect != Dspcode.ActType) || ((GyroSelect&0x7f) != Dspcode.GyroType))
		return 0xF2;

	return DownloadToEP3(o_ctrl, ptr->DataPM, ptr->LengthPM, ptr->Parity, ptr->DataDM, ptr->LengthDMA, ptr->LengthDMB);
}

//********************************************************************************
// Function Name 	: SetGyroAccelCoef
// Retun Value		: non
// Argment Value	:
// Explanation		: Set Gyro Coef and Accel Coef
// History			: First edition
//********************************************************************************
void SetGyroAccelCoef(struct cam_ois_ctrl_t *o_ctrl, UINT8 SelectAct) {
	CAM_INFO(CAM_OIS, "SetGyroAccelCoef SelectAct: %d", SelectAct);
	switch(SelectAct) {
	case ACT_SO2820 :
		if (MODEL_1 == o_ctrl->ois_fw_flag)
		{
			CAM_INFO(CAM_OIS, "SetGyroAccelCoef ACT_SO2820 MODEL_1: %d", MODEL_1);
			RamWrite32A(o_ctrl, GCNV_XX, (UINT32) 0x80000001);
			RamWrite32A(o_ctrl, GCNV_XY, (UINT32) 0x00000000);
			RamWrite32A(o_ctrl, GCNV_YY, (UINT32) 0x7FFFFFFF);
			RamWrite32A(o_ctrl, GCNV_YX, (UINT32) 0x00000000);
			RamWrite32A(o_ctrl, GCNV_ZP, (UINT32) 0x7FFFFFFF);

			RamWrite32A(o_ctrl, ACNV_XX, (UINT32) 0x7FFFFFFF);
			RamWrite32A(o_ctrl, ACNV_XY, (UINT32) 0x00000000);
			RamWrite32A(o_ctrl, ACNV_YY, (UINT32) 0x7FFFFFFF);
			RamWrite32A(o_ctrl, ACNV_YX, (UINT32) 0x00000000);
			RamWrite32A(o_ctrl, ACNV_ZP, (UINT32) 0x80000001);

			break;
		}
		if (MODEL_2 == o_ctrl->ois_fw_flag)
		{
			CAM_INFO(CAM_OIS, "SetGyroAccelCoef ACT_SO2820 MODEL_2: %d", MODEL_1);
			RamWrite32A(o_ctrl, GCNV_XX, (UINT32) 0x80000001);
			RamWrite32A(o_ctrl, GCNV_XY, (UINT32) 0x00000000);
			RamWrite32A(o_ctrl, GCNV_YY, (UINT32) 0x80000001);
			RamWrite32A(o_ctrl, GCNV_YX, (UINT32) 0x00000000);
			RamWrite32A(o_ctrl, GCNV_ZP, (UINT32) 0x80000001);

			RamWrite32A(o_ctrl, ACNV_XX, (UINT32) 0x7FFFFFFF);
			RamWrite32A(o_ctrl, ACNV_XY, (UINT32) 0x00000000);
			RamWrite32A(o_ctrl, ACNV_YY, (UINT32) 0x7FFFFFFF);
			RamWrite32A(o_ctrl, ACNV_YX, (UINT32) 0x00000000);
			RamWrite32A(o_ctrl, ACNV_ZP, (UINT32) 0x7FFFFFFF);
			break;
		}
		if (1 == o_ctrl->ois_gyro_id)//18821 rear
		{
			CAM_INFO(CAM_OIS, "SetGyroAccelCoef : gyro %d 821 rear\n", o_ctrl->ois_gyro_id);
			RamWrite32A(o_ctrl, GCNV_XX, (UINT32)0x00000000);
			RamWrite32A(o_ctrl, GCNV_XY, (UINT32)0x7FFFFFFF);
			RamWrite32A(o_ctrl, GCNV_YY, (UINT32)0x00000000);
			RamWrite32A(o_ctrl, GCNV_YX, (UINT32)0x7FFFFFFF);
			RamWrite32A(o_ctrl, GCNV_ZP, (UINT32)0x7FFFFFFF);

			RamWrite32A(o_ctrl, ACNV_XX, (UINT32)0x00000000);
			RamWrite32A(o_ctrl, ACNV_XY, (UINT32)0x7FFFFFFF);
			RamWrite32A(o_ctrl, ACNV_YY, (UINT32)0x00000000);
			RamWrite32A(o_ctrl, ACNV_YX, (UINT32)0x7FFFFFFF);
			RamWrite32A(o_ctrl, ACNV_ZP, (UINT32)0x7FFFFFFF);
		}
		if (2 == o_ctrl->ois_gyro_id)//18827 rear
		{
			CAM_INFO(CAM_OIS, "SetGyroAccelCoef : gyro %d 827 rear\n", o_ctrl->ois_gyro_id);
			RamWrite32A(o_ctrl, GCNV_XX, (UINT32)0x00000000);
			RamWrite32A(o_ctrl, GCNV_XY, (UINT32)0x7FFFFFFF);
			RamWrite32A(o_ctrl, GCNV_YY, (UINT32)0x00000000);
			RamWrite32A(o_ctrl, GCNV_YX, (UINT32)0x80000001);
			RamWrite32A(o_ctrl, GCNV_ZP, (UINT32)0x80000001);

			RamWrite32A(o_ctrl, ACNV_XX, (UINT32)0x00000000);
			RamWrite32A(o_ctrl, ACNV_XY, (UINT32)0x7FFFFFFF);
			RamWrite32A(o_ctrl, ACNV_YY, (UINT32)0x00000000);
			RamWrite32A(o_ctrl, ACNV_YX, (UINT32)0x80000001);
			RamWrite32A(o_ctrl, ACNV_ZP, (UINT32)0x80000001);
		}
		break;
	case ACT_SO3600:
		if (MODEL_1 == o_ctrl->ois_fw_flag)
		{
			CAM_INFO(CAM_OIS, "SetGyroAccelCoef ACT_SO3600 MODEL_1: %d", MODEL_1);
			RamWrite32A(o_ctrl, GCNV_XX, (UINT32) 0x00000000);
			RamWrite32A(o_ctrl, GCNV_XY, (UINT32) 0x80000001);
			RamWrite32A(o_ctrl, GCNV_YY, (UINT32) 0x00000000);
			RamWrite32A(o_ctrl, GCNV_YX, (UINT32) 0x7FFFFFFF);
			RamWrite32A(o_ctrl, GCNV_ZP, (UINT32) 0x7FFFFFFF);

			RamWrite32A(o_ctrl, ACNV_XX, (UINT32) 0x00000000);
			RamWrite32A(o_ctrl, ACNV_XY, (UINT32) 0x7FFFFFFF);
			RamWrite32A(o_ctrl, ACNV_YY, (UINT32) 0x00000000);
			RamWrite32A(o_ctrl, ACNV_YX, (UINT32) 0x7FFFFFFF);
			RamWrite32A(o_ctrl, ACNV_ZP, (UINT32) 0x80000001);
			break;
		}
		if (MODEL_2 == o_ctrl->ois_fw_flag)
		{
			CAM_INFO(CAM_OIS, "SetGyroAccelCoef ACT_SO3600 MODEL_2: %d", MODEL_2);
			RamWrite32A(o_ctrl, GCNV_XX, (UINT32) 0x00000000);
			RamWrite32A(o_ctrl, GCNV_XY, (UINT32) 0x7FFFFFFF);
			RamWrite32A(o_ctrl, GCNV_YY, (UINT32) 0x00000000);
			RamWrite32A(o_ctrl, GCNV_YX, (UINT32) 0x7FFFFFFF);
			RamWrite32A(o_ctrl, GCNV_ZP, (UINT32) 0x80000001);

			RamWrite32A(o_ctrl, ACNV_XX, (UINT32) 0x00000000);
			RamWrite32A(o_ctrl, ACNV_XY, (UINT32) 0x7FFFFFFF);
			RamWrite32A(o_ctrl, ACNV_YY, (UINT32) 0x00000000);
			RamWrite32A(o_ctrl, ACNV_YX, (UINT32) 0x7FFFFFFF);
			RamWrite32A(o_ctrl, ACNV_ZP, (UINT32) 0x7FFFFFFF);

			break;
		}
		if (1 == o_ctrl->ois_gyro_id)//18821 rear
		{
			CAM_INFO(CAM_OIS, "SetGyroAccelCoef tele: gyro %d 821 rear\n", o_ctrl->ois_gyro_id);
			RamWrite32A(o_ctrl, GCNV_XX, (UINT32)0x80000001);
			RamWrite32A(o_ctrl, GCNV_XY, (UINT32)0x00000000);
			RamWrite32A(o_ctrl, GCNV_YY, (UINT32)0x80000001);
			RamWrite32A(o_ctrl, GCNV_YX, (UINT32)0x00000000);
			RamWrite32A(o_ctrl, GCNV_ZP, (UINT32)0x7FFFFFFF);

			RamWrite32A(o_ctrl, ACNV_XX, (UINT32)0x7FFFFFFF);
			RamWrite32A(o_ctrl, ACNV_XY, (UINT32)0x00000000);
			RamWrite32A(o_ctrl, ACNV_YY, (UINT32)0x80000001);
			RamWrite32A(o_ctrl, ACNV_YX, (UINT32)0x00000000);
			RamWrite32A(o_ctrl, ACNV_ZP, (UINT32)0x7FFFFFFF);
		}
		if (2 == o_ctrl->ois_gyro_id)//18827 rear
		{
			CAM_INFO(CAM_OIS, "SetGyroAccelCoef tele: gyro %d 827 rear\n", o_ctrl->ois_gyro_id);
			RamWrite32A(o_ctrl, GCNV_XX, (UINT32)0x7FFFFFFF);
			RamWrite32A(o_ctrl, GCNV_XY, (UINT32)0x00000000);
			RamWrite32A(o_ctrl, GCNV_YY, (UINT32)0x80000001);
			RamWrite32A(o_ctrl, GCNV_YX, (UINT32)0x00000000);
			RamWrite32A(o_ctrl, GCNV_ZP, (UINT32)0x7FFFFFFF);

			RamWrite32A(o_ctrl, ACNV_XX, (UINT32)0x80000001);
			RamWrite32A(o_ctrl, ACNV_XY, (UINT32)0x00000000);
			RamWrite32A(o_ctrl, ACNV_YY, (UINT32)0x80000001);
			RamWrite32A(o_ctrl, ACNV_YX, (UINT32)0x00000000);
			RamWrite32A(o_ctrl, ACNV_ZP, (UINT32)0x7FFFFFFF);
		}
		break;
	case ACT_SO2821:
		if (3 == o_ctrl->ois_gyro_id) //semco 18857 rear
		{
			switch (o_ctrl->ois_fw_flag) {
			case MODEL_0: //18857
				CAM_INFO(CAM_OIS, "SetGyroAccelCoef 857main : gyro %d 857 rear\n", o_ctrl->ois_gyro_id);
				RamWrite32A(o_ctrl, GCNV_XX, (UINT32) 0x00000000);
				RamWrite32A(o_ctrl, GCNV_XY, (UINT32) 0x7FFFFFFF);
				RamWrite32A(o_ctrl, GCNV_YY, (UINT32) 0x00000000);
				RamWrite32A(o_ctrl, GCNV_YX, (UINT32) 0x7FFFFFFF);
				RamWrite32A(o_ctrl, GCNV_ZP, (UINT32) 0x7FFFFFFF);

				RamWrite32A(o_ctrl, ACNV_XX, (UINT32) 0x00000000);
				RamWrite32A(o_ctrl, ACNV_XY, (UINT32) 0x7FFFFFFF);
				RamWrite32A(o_ctrl, ACNV_YY, (UINT32) 0x00000000);
				RamWrite32A(o_ctrl, ACNV_YX, (UINT32) 0x7FFFFFFF);
				RamWrite32A(o_ctrl, ACNV_ZP, (UINT32) 0x7FFFFFFF);
				break;
			case MODEL_1: //18865/18863
				CAM_INFO(CAM_OIS, "SetGyroAccelCoef 865main : gyro %d 865 rear\n", o_ctrl->ois_gyro_id);
				RamWrite32A(o_ctrl, GCNV_XX, (UINT32) 0x00000000);
				RamWrite32A(o_ctrl, GCNV_XY, (UINT32) 0x7FFFFFFF);
				RamWrite32A(o_ctrl, GCNV_YY, (UINT32) 0x00000000);
				RamWrite32A(o_ctrl, GCNV_YX, (UINT32) 0x7FFFFFFF);
				RamWrite32A(o_ctrl, GCNV_ZP, (UINT32) 0x7FFFFFFF);

				RamWrite32A(o_ctrl, ACNV_XX, (UINT32) 0x00000000);
				RamWrite32A(o_ctrl, ACNV_XY, (UINT32) 0x7FFFFFFF);
				RamWrite32A(o_ctrl, ACNV_YY, (UINT32) 0x00000000);
				RamWrite32A(o_ctrl, ACNV_YX, (UINT32) 0x80000001);
				RamWrite32A(o_ctrl, ACNV_ZP, (UINT32) 0x80000001);
			}
		}
		break;
	case ACT_M12337: //second master 199865/19863
		if (3 == o_ctrl->ois_gyro_id)
		{
			if (MODEL_1 == o_ctrl->ois_fw_flag) //master 18865 only
			{
			CAM_INFO(CAM_OIS, "SetGyroAccelCoef ACT_M12337 MODEL_1: %d", MODEL_1);
			//todo: get from onsemi for ofilm model 1, currently copied from ofilm 18821
			RamWrite32A(o_ctrl, GCNV_XX, (UINT32)0x00000000);
			RamWrite32A(o_ctrl, GCNV_XY, (UINT32)0x7FFFFFFF);
			RamWrite32A(o_ctrl, GCNV_YY, (UINT32)0x00000000);
			RamWrite32A(o_ctrl, GCNV_YX, (UINT32)0x80000001);
			RamWrite32A(o_ctrl, GCNV_ZP, (UINT32)0x7FFFFFFF);

			RamWrite32A(o_ctrl, ACNV_XX, (UINT32)0x00000000);
			RamWrite32A(o_ctrl, ACNV_XY, (UINT32)0x7FFFFFFF);
			RamWrite32A(o_ctrl, ACNV_YY, (UINT32)0x00000000);
			RamWrite32A(o_ctrl, ACNV_YX, (UINT32)0x7FFFFFFF);
			RamWrite32A(o_ctrl, ACNV_ZP, (UINT32)0x7FFFFFFF);
			}
		}
		break;
	case ACT_M12337_A1 : //second master 19801/19861
	case ACT_M10235_A1 : //second slave 19801/19861
		if (MODEL_1 == o_ctrl->ois_fw_flag) //master & slave 19801/19861
		{
			CAM_INFO(CAM_OIS, "SetGyroAccelCoef ACT_M12337_A1/ACT_M10235_A1 MODEL_1: %d", MODEL_1);
			RamWrite32A(o_ctrl,GCNV_XX, (UINT32) 0x80000001);
			RamWrite32A(o_ctrl,GCNV_XY, (UINT32) 0x00000000);
			RamWrite32A(o_ctrl,GCNV_YY, (UINT32) 0x80000001);
			RamWrite32A(o_ctrl,GCNV_YX, (UINT32) 0x00000000);
			RamWrite32A(o_ctrl,GCNV_ZP, (UINT32) 0x7FFFFFFF);

			RamWrite32A(o_ctrl,ACNV_XX, (UINT32) 0x80000001);
			RamWrite32A(o_ctrl,ACNV_XY, (UINT32) 0x00000000);
			RamWrite32A(o_ctrl,ACNV_YY, (UINT32) 0x80000001);
			RamWrite32A(o_ctrl,ACNV_YX, (UINT32) 0x00000000);
			RamWrite32A(o_ctrl,ACNV_ZP, (UINT32) 0x7FFFFFFF);
			break;
		}
		if (MODEL_2 == o_ctrl->ois_fw_flag) //master & slave 19801/19861
		{
			CAM_INFO(CAM_OIS, "SetGyroAccelCoef ACT_M12337_A1/ACT_M10235_A1 MODEL_2: %d", MODEL_2);
			RamWrite32A(o_ctrl,GCNV_XX, (UINT32) 0x80000001);
			RamWrite32A(o_ctrl,GCNV_XY, (UINT32) 0x00000000);
			RamWrite32A(o_ctrl,GCNV_YY, (UINT32) 0x7FFFFFFF);
			RamWrite32A(o_ctrl,GCNV_YX, (UINT32) 0x00000000);
			RamWrite32A(o_ctrl,GCNV_ZP, (UINT32) 0x7FFFFFFF);

			RamWrite32A(o_ctrl,ACNV_XX, (UINT32) 0x80000001);
			RamWrite32A(o_ctrl,ACNV_XY, (UINT32) 0x00000000);
			RamWrite32A(o_ctrl,ACNV_YY, (UINT32) 0x80000001);
			RamWrite32A(o_ctrl,ACNV_YX, (UINT32) 0x00000000);
			RamWrite32A(o_ctrl,ACNV_ZP, (UINT32) 0x7FFFFFFF);
			break;
		}
		if (1 == o_ctrl->ois_gyro_id)//18821 second rear
		{
			CAM_INFO(CAM_OIS, "SetGyroAccelCoef : gyro %d 821 rear\n", o_ctrl->ois_gyro_id);
			RamWrite32A(o_ctrl, GCNV_XX, (UINT32)0x00000000);
			RamWrite32A(o_ctrl, GCNV_XY, (UINT32)0x7FFFFFFF);
			RamWrite32A(o_ctrl, GCNV_YY, (UINT32)0x00000000);
			RamWrite32A(o_ctrl, GCNV_YX, (UINT32)0x80000001);
			RamWrite32A(o_ctrl, GCNV_ZP, (UINT32)0x7FFFFFFF);

			RamWrite32A(o_ctrl, ACNV_XX, (UINT32)0x00000000);
			RamWrite32A(o_ctrl, ACNV_XY, (UINT32)0x7FFFFFFF);
			RamWrite32A(o_ctrl, ACNV_YY, (UINT32)0x00000000);
			RamWrite32A(o_ctrl, ACNV_YX, (UINT32)0x7FFFFFFF);
			RamWrite32A(o_ctrl, ACNV_ZP, (UINT32)0x7FFFFFFF);
		}
		if (2 == o_ctrl->ois_gyro_id)//18827 rear
		{
			CAM_INFO(CAM_OIS, "SetGyroAccelCoef : gyro %d 827 rear\n", o_ctrl->ois_gyro_id);
			RamWrite32A(o_ctrl, GCNV_XX, (UINT32)0x00000000);
			RamWrite32A(o_ctrl, GCNV_XY, (UINT32)0x7FFFFFFF);
			RamWrite32A(o_ctrl, GCNV_YY, (UINT32)0x00000000);
			RamWrite32A(o_ctrl, GCNV_YX, (UINT32)0x7FFFFFFF);
			RamWrite32A(o_ctrl, GCNV_ZP, (UINT32)0x80000001);

			RamWrite32A(o_ctrl, ACNV_XX, (UINT32)0x00000000);
			RamWrite32A(o_ctrl, ACNV_XY, (UINT32)0x7FFFFFFF);
			RamWrite32A(o_ctrl, ACNV_YY, (UINT32)0x00000000);
			RamWrite32A(o_ctrl, ACNV_YX, (UINT32)0x7FFFFFFF);
			RamWrite32A(o_ctrl, ACNV_ZP, (UINT32)0x80000001);
		}
		break;
	default:
		CAM_INFO(CAM_OIS, "SetGyroAccelCoef : default");
	}

}

//********************************************************************************
// Function Name 	: MesFil
// Retun Value		: NON
// Argment Value	: Measure Filter Mode
// Explanation		: Measure Filter Setting Function
// History			: First edition
//********************************************************************************
void MesFil(struct cam_ois_ctrl_t *o_ctrl, UINT8	UcMesMod)	// 20.019kHz
{
	UINT32	UlMeasFilaA, UlMeasFilaB, UlMeasFilaC;
	UINT32	UlMeasFilbA, UlMeasFilbB, UlMeasFilbC;

        UlMeasFilaA = 0x00000000;
        UlMeasFilaB = 0x00000000;
        UlMeasFilaC = 0x00000000;
        UlMeasFilbA = 0x00000000;
        UlMeasFilbB = 0x00000000;
        UlMeasFilbC = 0x00000000;

	if (!UcMesMod) {	// Hall Bias&Offset Adjust
		UlMeasFilaA = 0x0342AD4D;	// LPF 150Hz
		UlMeasFilaB = 0x0342AD4D;
		UlMeasFilaC = 0x797AA565;
		UlMeasFilbA = 0x7FFFFFFF;	// Through
		UlMeasFilbB = 0x00000000;
		UlMeasFilbC = 0x00000000;
	} else if (UcMesMod == LOOPGAIN) {	// Loop Gain Adjust
		UlMeasFilaA = 0x12FEA055;	// LPF1000Hz
		UlMeasFilaB = 0x12FEA055;
		UlMeasFilaC = 0x5A02BF55;
		UlMeasFilbA = 0x7F559791;	// HPF30Hz
		UlMeasFilbB = 0x80AA686F;
		UlMeasFilbC = 0x7EAB2F23;
	} else if (UcMesMod == THROUGH) {	// for Through
		UlMeasFilaA = 0x7FFFFFFF;	// Through
		UlMeasFilaB = 0x00000000;
		UlMeasFilaC = 0x00000000;
		UlMeasFilbA = 0x7FFFFFFF;	// Through
		UlMeasFilbB = 0x00000000;
		UlMeasFilbC = 0x00000000;
	} else if (UcMesMod == NOISE) {		// SINE WAVE TEST for NOISE
		UlMeasFilaA = 0x0342AD4D;	// LPF150Hz
		UlMeasFilaB = 0x0342AD4D;
		UlMeasFilaC = 0x797AA565;
		UlMeasFilbA = 0x0342AD4D;	// LPF150Hz
		UlMeasFilbB = 0x0342AD4D;
		UlMeasFilbC = 0x797AA565;
	} else if (UcMesMod == OSCCHK) {
		UlMeasFilaA = 0x065BE349;	// LPF300Hz
		UlMeasFilaB = 0x065BE349;
		UlMeasFilaC = 0x7348396D;
		UlMeasFilbA = 0x065BE349;	// LPF300Hz
		UlMeasFilbB = 0x065BE349;
		UlMeasFilbC = 0x7348396D;
	}

	RamWrite32A (o_ctrl, MeasureFilterA_Coeff_a1, UlMeasFilaA);
	RamWrite32A (o_ctrl, MeasureFilterA_Coeff_b1, UlMeasFilaB);
	RamWrite32A (o_ctrl, MeasureFilterA_Coeff_c1, UlMeasFilaC);

	RamWrite32A (o_ctrl, MeasureFilterA_Coeff_a2, UlMeasFilbA);
	RamWrite32A (o_ctrl, MeasureFilterA_Coeff_b2, UlMeasFilbB);
	RamWrite32A (o_ctrl, MeasureFilterA_Coeff_c2, UlMeasFilbC);

	RamWrite32A (o_ctrl, MeasureFilterB_Coeff_a1, UlMeasFilaA);
	RamWrite32A (o_ctrl, MeasureFilterB_Coeff_b1, UlMeasFilaB);
	RamWrite32A (o_ctrl, MeasureFilterB_Coeff_c1, UlMeasFilaC);

	RamWrite32A (o_ctrl, MeasureFilterB_Coeff_a2, UlMeasFilbA);
	RamWrite32A (o_ctrl, MeasureFilterB_Coeff_b2, UlMeasFilbB);
	RamWrite32A (o_ctrl, MeasureFilterB_Coeff_c2, UlMeasFilbC);
}

//********************************************************************************
// Function Name 	: ClrMesFil
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Clear Measure Filter Function
// History			: First edition
//********************************************************************************
void ClrMesFil(struct cam_ois_ctrl_t *o_ctrl)
{
	RamWrite32A (o_ctrl, MeasureFilterA_Delay_z11, 0);
	RamWrite32A (o_ctrl, MeasureFilterA_Delay_z12, 0);

	RamWrite32A (o_ctrl, MeasureFilterA_Delay_z21, 0);
	RamWrite32A (o_ctrl, MeasureFilterA_Delay_z22, 0);

	RamWrite32A (o_ctrl, MeasureFilterB_Delay_z11, 0);
	RamWrite32A (o_ctrl, MeasureFilterB_Delay_z12, 0);

	RamWrite32A (o_ctrl, MeasureFilterB_Delay_z21, 0);
	RamWrite32A (o_ctrl, MeasureFilterB_Delay_z22, 0);
}


//********************************************************************************
// Function Name 	: SetWaitTime
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Set Timer wait Function
// History			: First edition
//********************************************************************************
void SetWaitTime(struct cam_ois_ctrl_t *o_ctrl, UINT16 UsWaitTime)
{
	RamWrite32A(o_ctrl, WaitTimerData_UiWaitCounter, 0);
	RamWrite32A(o_ctrl, WaitTimerData_UiTargetCount, (UINT32)(ONE_MSEC_COUNT * UsWaitTime));
}

//********************************************************************************
// Function Name 	: SetTransDataAdr
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Trans Address for Data Function
// History			: First edition
//********************************************************************************
void SetTransDataAdr(struct cam_ois_ctrl_t *o_ctrl, UINT16 UsLowAddress,
	UINT32 UlLowAdrBeforeTrans)
{
	UnDwdVal	StTrsVal;

	if (UlLowAdrBeforeTrans < 0x00009000) {
		StTrsVal.UlDwdVal = UlLowAdrBeforeTrans;
	} else {
		StTrsVal.StDwdVal.UsHigVal = (UINT16)((UlLowAdrBeforeTrans & 0x0000F000) >> 8);
		StTrsVal.StDwdVal.UsLowVal = (UINT16)(UlLowAdrBeforeTrans & 0x00000FFF);
	}
//TRACE(" TRANS  ADR = %04xh, DAT = %08xh \n",UsLowAddress, StTrsVal.UlDwdVal);
	RamWrite32A(o_ctrl, UsLowAddress,	StTrsVal.UlDwdVal);

}

//********************************************************************************
// Function Name 	: MemoryClear
// Retun Value		: NON
// Argment Value	: Top pointer, Size
// Explanation		: Memory Clear Function
// History			: First edition
//********************************************************************************
void MemoryClear(struct cam_ois_ctrl_t *o_ctrl, UINT16 UsSourceAddress, UINT16 UsClearSize)
{
	UINT16	UsLoopIndex;

	for (UsLoopIndex = 0; UsLoopIndex < UsClearSize; UsLoopIndex += 4) {
		RamWrite32A(o_ctrl, UsSourceAddress + UsLoopIndex, 0x00000000);	// 4Byte
	}
}

//********************************************************************************
// Function Name 	: MeasureStart
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Measure start setting Function
// History			: First edition
//********************************************************************************
void MeasureStart(struct cam_ois_ctrl_t *o_ctrl, INT32 SlMeasureParameterNum,
	UINT32 SlMeasureParameterA, UINT32 SlMeasureParameterB)
{
	MemoryClear(o_ctrl, StMeasFunc_SiSampleNum, sizeof(MeasureFunction_Type));
	RamWrite32A(o_ctrl, StMeasFunc_MFA_SiMax1, 0x80000000);	// Set Min
	RamWrite32A(o_ctrl, StMeasFunc_MFB_SiMax2, 0x80000000);	// Set Min
	RamWrite32A(o_ctrl, StMeasFunc_MFA_SiMin1, 0x7FFFFFFF);	// Set Max
	RamWrite32A(o_ctrl, StMeasFunc_MFB_SiMin2, 0x7FFFFFFF);	// Set Max

	SetTransDataAdr(o_ctrl, StMeasFunc_MFA_PiMeasureRam1, SlMeasureParameterA);	// Set Measure Filter A Ram Address
	SetTransDataAdr(o_ctrl, StMeasFunc_MFB_PiMeasureRam2, SlMeasureParameterB);	// Set Measure Filter B Ram Address
	RamWrite32A(o_ctrl, StMeasFunc_SiSampleNum, 0);		// Clear Measure Counter
	ClrMesFil(o_ctrl);					// Clear Delay Ram
	SetWaitTime(o_ctrl, 1);
	RamWrite32A(o_ctrl, StMeasFunc_SiSampleMax, SlMeasureParameterNum);	// Set Measure Max Number

}

//********************************************************************************
// Function Name 	: MeasureWait
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Wait complete of Measure Function
// History			: First edition
//********************************************************************************
void MeasureWait(struct cam_ois_ctrl_t *o_ctrl)
{
	UINT32 SlWaitTimerSt;

	SlWaitTimerSt = 1;
	while(SlWaitTimerSt) {
		RamRead32A(o_ctrl, StMeasFunc_SiSampleMax, &SlWaitTimerSt);
	}
}

//********************************************************************************
// Function Name 	: SetGyroOffset
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: set the gyro offset data. before do this before Remapmain.
// History			: First edition
//********************************************************************************
void SetGyroOffset(struct cam_ois_ctrl_t *o_ctrl, UINT16 GyroOffsetX, UINT16 GyroOffsetY, UINT16 GyroOffsetZ)
{
	RamWrite32A(o_ctrl, GYRO_RAM_GXOFFZ, ((GyroOffsetX << 16) & 0xFFFF0000));	// X axis Gyro offset
	RamWrite32A(o_ctrl, GYRO_RAM_GYOFFZ, ((GyroOffsetY << 16) & 0xFFFF0000));	// Y axis Gyro offset
	RamWrite32A(o_ctrl, GYRO_RAM_GZOFFZ, ((GyroOffsetZ << 16) & 0xFFFF0000));	// Y axis Gyro offset
}

//********************************************************************************
// Function Name 	:GyroOffsetMeasureStart
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: start the gyro offset adjustment
// History			: First edition
//********************************************************************************
void GyroOffsetMeasureStart(struct cam_ois_ctrl_t *o_ctrl)
{
	MesFil(o_ctrl, THROUGH);	// Set Measure Filter
	MeasureStart(o_ctrl, GYROF_NUM, GYRO_RAM_GX_ADIDAT, GYRO_RAM_GY_ADIDAT);	// Start measure
}

//********************************************************************************
// Function Name 	:GyroOffsetMeasureStartZ
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: start the gyro offset adjustment
// History			: First edition
//********************************************************************************
void GyroOffsetMeasureStartZ(struct cam_ois_ctrl_t *o_ctrl)
{
	MesFil(o_ctrl, THROUGH);	// Set Measure Filter
	MeasureStart(o_ctrl, GYROF_NUM, GYRO_RAM_GZ_ADIDAT, GYRO_RAM_GZ_ADIDAT);	// Start measure
}

//********************************************************************************
// Function Name 	: GetGyroOffset
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: get the gyro offset adjustment result
// History			: First edition
//********************************************************************************
UINT8 GetGyroOffset(struct cam_ois_ctrl_t *o_ctrl, UINT16* GyroOffsetX,
	UINT16* GyroOffsetY, INT16 GYROF_UPPER, INT16 GYROF_LOWER)
{
	UnllnVal StMeasValueA, StMeasValueB;
	INT32 SlMeasureAveValueA, SlMeasureAveValueB;
	INT32 SlMeasureMaxValue, SlMeasureMinValue;
	UINT32 UlReadVal, UlCnt=0;
	UINT8 ans=0;

	// Wait complete of measurement
	do {
		if (UlCnt++ > 100) {
			/* timeout error */
			*GyroOffsetX = 0;
			*GyroOffsetY = 0;
			CAM_ERR(CAM_OIS, "UlCnt up to 100+ return 3.");		//byron
			return 3;
		}
		RamRead32A(o_ctrl, StMeasFunc_SiSampleMax, &UlReadVal);
	} while (UlReadVal != 0);

	RamRead32A(o_ctrl, StMeasFunc_MFA_SiMax1, (UINT32 *)&SlMeasureMaxValue);	// Max value
	RamRead32A(o_ctrl, StMeasFunc_MFA_SiMin1, (UINT32 *)&SlMeasureMinValue);	// Min value
	if (SlMeasureMaxValue == SlMeasureMinValue)
	{
		CAM_ERR(CAM_OIS, "SlMeasureMaxValue == SlMeasureMinValue return 3.");	//byron2
		return 3;
	}

	RamRead32A(o_ctrl, StMeasFunc_MFA_LLiIntegral1, &StMeasValueA.StUllnVal.UlLowVal);	// X axis
	RamRead32A(o_ctrl, StMeasFunc_MFA_LLiIntegral1 + 4, &StMeasValueA.StUllnVal.UlHigVal);
	RamRead32A(o_ctrl, StMeasFunc_MFB_LLiIntegral2, &StMeasValueB.StUllnVal.UlLowVal);	// Y axis
	RamRead32A(o_ctrl, StMeasFunc_MFB_LLiIntegral2 + 4, &StMeasValueB.StUllnVal.UlHigVal);

	SlMeasureAveValueA = (INT32)((INT64)StMeasValueA.UllnValue / GYROF_NUM);
	SlMeasureAveValueB = (INT32)((INT64)StMeasValueB.UllnValue / GYROF_NUM);

	SlMeasureAveValueA = (SlMeasureAveValueA >> 16) & 0x0000FFFF;
	SlMeasureAveValueB = (SlMeasureAveValueB >> 16) & 0x0000FFFF;

	*GyroOffsetX = (UINT16)(SlMeasureAveValueA & 0x0000FFFF);		//Measure Result Store
	*GyroOffsetY = (UINT16)(SlMeasureAveValueB & 0x0000FFFF);		//Measure Result Store

	if (((INT16)(*GyroOffsetX) > GYROF_UPPER) || ((INT16)(*GyroOffsetX) < GYROF_LOWER)) {
		ans |= 1;
	}
	if (((INT16)(*GyroOffsetY) > GYROF_UPPER) || ((INT16)(*GyroOffsetY) < GYROF_LOWER)) {
		ans |= 2;
	}
	CAM_ERR(CAM_OIS, "func finish return.");	//byron3
	return ans;
}

#endif
