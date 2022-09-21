/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2018-2020 Oplus. All rights reserved.
 */
/*==============================================================================

FILE:      oplus_pmic_info.h

DESCRIPTION:

=============================================================================*/
#ifndef __OPLUS_PMIC_INFO_H__
#define __OPLUS_PMIC_INFO_H__

#define PMIC_INFO_MAGIC 0x43494D504F50504F
#define DATA_VALID_FLAG 0xCC
#define MAX_HISTORY_COUNT 4

//xbl struct
struct PMICRegStruct {
	u64 pon_reason;
	u64 fault_reason;
	u32 ldo_ocp_status;
	u8  spms_ocp_status;
	u8  bob_ocp_status;
	u16  data_is_valid;
};

struct PMICRecordStruct {
	struct PMICRegStruct pmic_pon_poff_reason[8]; //PM_MAX_NUM_PMICS is 8
};

struct PMICHistoryStruct {
	u64 pmic_magic;
	u64  log_count;
	struct PMICRecordStruct pmic_record[MAX_HISTORY_COUNT];
};

//kernel struct
struct PMICRegKernelStruct {

	//u64 pon_reason;
	u8 PON_PON_REASON1;				//0x8C0
	u8 PON_RESERVE1;				//0x8C1
	u8 PON_WARM_RESET_REASON1;		//0x8C2
	u8 PON_RESERVE2;				//0x8C3
	u8 PON_ON_REASON;				//0x8C4	
	u8 PON_POFF_REASON1;			//0x8C5	
	u8 PON_RESERVE3;				//0x8C6	
	u8 PON_OFF_REASON;				//0x8C7	

	//u64 fault_reason;
	u8 PON_FAULT_REASON1;			//0x8C8
	u8 PON_FAULT_REASON2;			//0x8C9
	u8 PON_S3_RESET_REASON;			//0x8CA
	u8 PON_SOFT_RESET_REASON1;		//0x8CB
	u8 PON_RESERVE4;				//0x8CC
	u8 PON_RESERVE5;				//0x8CD
	u8 PON_RESERVE6;				//0x8CE	
	u8 PON_RESERVE7;				//0x8DF	


	u32 ldo_ocp_status;
	u8  spms_ocp_status;
	u8  bob_ocp_status;
	u16  data_is_valid;
};

struct PMICRecordKernelStruct {
	struct PMICRegKernelStruct pmic_pon_poff_reason[8]; //PM_MAX_NUM_PMICS is 8
};

struct PMICHistoryKernelStruct {
	char pmic_magic[8];
	u64  log_count;
	struct PMICRecordKernelStruct pmic_record[MAX_HISTORY_COUNT];
};


#define MAX_RECORD_LDO_NUM 32
#define MAX_RECORD_SPMS_NUM 8
#define MAX_RECORD_BOB_NUM 8

#define pmic_info_attr(_name) \
static struct kobj_attribute _name##_attr = {	\
	.attr	= {				\
		.name = __stringify(_name),	\
		.mode = 0644,			\
	},					\
	.show	= _name##_show,			\
	.store	= _name##_store,		\
}

#define pmic_info_attr_ro(_name) \
static struct kobj_attribute _name##_attr = {	\
	.attr	= {				\
		.name = __stringify(_name),	\
		.mode = S_IRUGO,		\
	},					\
	.show	= _name##_show,			\
}

//PON_OFF_REASON |0x8C7
#define INDEX_RAW_XVDD_RB_OCCURED 		3
#define INDEX_RAW_DVDD_RB_OCCURED 		4
#define INDEX_IMMEDIATE_XVDD_SHUTDOWN	5
#define INDEX_S3_RESET					6
#define INDEX_FAULT_SEQ					7
#define INDEX_POFF_SEQ					8

struct PMICHistoryStruct *get_pmic_history(void);

#endif