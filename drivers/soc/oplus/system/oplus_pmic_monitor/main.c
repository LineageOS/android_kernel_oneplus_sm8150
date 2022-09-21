// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2018-2020 Oplus. All rights reserved.
 */
#include <linux/kobject.h>
#include <linux/string.h>
#include <linux/err.h>
#include <linux/module.h>
#include "oplus_pmic_info.h"

/**********************************************/
static ssize_t pmic_history_magic_show(struct kobject *kobj,
			struct kobj_attribute *attr, char *buf)
{
    char page[16] = {0};
    int len = 0;
	struct PMICHistoryKernelStruct *pmic_history_ptr = NULL;

	pmic_history_ptr = (struct PMICHistoryKernelStruct *)get_pmic_history();
	if (NULL == pmic_history_ptr) {
		len += snprintf(&page[len], 16-len, "NULL\n");
	} else {
		len += snprintf(&page[len], 16-len, "%s\n",pmic_history_ptr->pmic_magic);
	}
    memcpy(buf,page,len);
	return len;
}
pmic_info_attr_ro(pmic_history_magic);

/**********************************************/

/**********************************************/
static ssize_t pmic_history_count_show(struct kobject *kobj,
			struct kobj_attribute *attr, char *buf)
{
    char page[16] = {0};
    int len = 0;
	struct PMICHistoryKernelStruct *pmic_history_ptr = NULL;

	pmic_history_ptr = (struct PMICHistoryKernelStruct *)get_pmic_history();
	if (NULL == pmic_history_ptr) {
		len += snprintf(&page[len],16-len, "NULL\n");
	} else {
		len += snprintf(&page[len],16-len, "%ld\n",pmic_history_ptr->log_count);
	}
    memcpy(buf,page,len);
	return len;
}
pmic_info_attr_ro(pmic_history_count);

/**********************************************/
static char * const pon_poff_reason_str[] = {
	[0] = "POWER OFF by N/A 0 ",
	[1] = "POWER OFF by N/A 1 ",
	[2] = "POWER OFF by RAW_XVDD_RB_OCCURED ",
	[3] = "POWER OFF by RAW_DVDD_RB_OCCURED ",
	[4] = "POWER OFF by IMMEDIATE_XVDD_SHUTDOWN ",
	[5] = "POWER OFF by S3_RESET ",
	[6] = "POWER OFF by FAULT_SEQ ",
	[7] = "POWER OFF by POFF_SEQ ",
};

static char * const pon_poff_reason1_str[] = {
	[0] = ":SOFT (Software)",
	[1] = ":PS_HOLD (PS_HOLD/MSM Controlled Shutdown)",
	[2] = ":PMIC_WD (PMIC Watchdog)",
	[3] = ":GP1 (Keypad_Reset1)",
	[4] = ":GP2 (Keypad_Reset2)",
	[5] = ":KPDPWR_AND_RESIN (Power Key and Reset Line)",
	[6] = ":RESIN_N (Reset Line/Volume Down Key)",
	[7] = ":KPDPWR_N (Long Power Key Hold)",
};

static char * const pon_fault_reason_str[] = {
	[0] = ":GP_FAULT0",
	[1] = ":GP_FAULT1",
	[2] = ":GP_FAULT2",
	[3] = ":GP_FAULT3",
	[4] = ":MBG_FAULT",
	[5] = ":OVLO",
	[6] = ":UVLO",
	[7] = ":AVDD_RB",
	[8] = ":N/A 8",
	[9] = ":N/A 9",
	[10] = ":N/A 10",
	[11] = ":FAULT_N",
	[12] = ":PBS_WATCHDOG_TO",
	[13] = ":PBS_NACK",
	[14] = ":RESTART_PON",
	[15] = ":OTST3",
};

static char * const pon_s3_reset_reason[] = {
	[0] = ":N/A 0",
	[1] = ":N/A 1",
	[2] = ":N/A 2",
	[3] = ":N/A 3",
	[4] = ":FAULT_N",
	[5] = ":PBS_WATCHDOG_TO",
	[6] = ":PBS_NACK",
	[7] = ":KPDPWR_ANDOR_RESIN",
};

static char * const unknow_reason_str = ":UNKNOW REASON";
static char * const no_L2_reason_str = ":don't have L2 reason";

static ssize_t poff_reason_show(struct kobject *kobj,
			struct kobj_attribute *attr, char *buf) {
	char page[512] = {0};
    int len = 0;
	u8  L1_poff_index=0,L2_poff_index=0;
	char * L1_str_ptr=unknow_reason_str;
	char * L2_str_ptr=unknow_reason_str;
	struct PMICHistoryKernelStruct *pmic_history_ptr = NULL;
	struct PMICRecordKernelStruct pmic_first_record;
	u8 pmic_device_index=0;
	u64 pmic_history_count=0;

	pmic_history_ptr = (struct PMICHistoryKernelStruct *)get_pmic_history();

	if (NULL == pmic_history_ptr) {
		len += snprintf(&page[len],512-len, "PMIC|0|0x00|0x0000|NULL\n");
		memcpy(buf,page,len);
		return len;
	}

	pmic_first_record = pmic_history_ptr->pmic_record[0];
	pmic_history_count = pmic_history_ptr->log_count;

	for (pmic_device_index = 0;pmic_device_index<8;pmic_device_index++) {
		if (DATA_VALID_FLAG != pmic_first_record.pmic_pon_poff_reason[pmic_device_index].data_is_valid) {
			continue;
		}

		L1_poff_index = ffs(pmic_first_record.pmic_pon_poff_reason[pmic_device_index].PON_OFF_REASON);
		switch (L1_poff_index) {
			case INDEX_RAW_XVDD_RB_OCCURED:
			case INDEX_RAW_DVDD_RB_OCCURED:
			case INDEX_IMMEDIATE_XVDD_SHUTDOWN:
					L1_str_ptr = pon_poff_reason_str[L1_poff_index-1];
					L2_poff_index = 0;
					L2_str_ptr = no_L2_reason_str;
				break;

			case INDEX_S3_RESET:
					L1_str_ptr = pon_poff_reason_str[L1_poff_index-1];
					L2_poff_index = ffs(pmic_first_record.pmic_pon_poff_reason[pmic_device_index].PON_S3_RESET_REASON);
					if (L2_poff_index <= 0 || L2_poff_index > 8 ) {
						L2_str_ptr = unknow_reason_str;
					} else {
						L2_str_ptr = pon_s3_reset_reason[L2_poff_index-1];
					}
				break;

			case INDEX_FAULT_SEQ:
					L1_str_ptr = pon_poff_reason_str[L1_poff_index-1];
					L2_poff_index = ffs( (pmic_first_record.pmic_pon_poff_reason[pmic_device_index].PON_FAULT_REASON1 << 8)
										| (pmic_first_record.pmic_pon_poff_reason[pmic_device_index].PON_FAULT_REASON2)
										);
					if (L2_poff_index <= 0 || L2_poff_index > 16 ) {
						L2_str_ptr = unknow_reason_str;
					} else {
						L2_str_ptr = pon_fault_reason_str[L2_poff_index-1];
					}
				break;
			case INDEX_POFF_SEQ:
					L1_str_ptr = pon_poff_reason_str[L1_poff_index-1];
					L2_poff_index = ffs(pmic_first_record.pmic_pon_poff_reason[pmic_device_index].PON_POFF_REASON1);
					if (L2_poff_index <= 0 || L2_poff_index > 8 ) {
						L2_str_ptr = unknow_reason_str;
					} else {
						L2_str_ptr = pon_poff_reason1_str[L2_poff_index-1];
					}
				break;
			default:
					L1_poff_index = 0;
					L2_poff_index = 0;
					L1_str_ptr = unknow_reason_str;
					L2_str_ptr = unknow_reason_str;
				break;
		}

		len += snprintf(&page[len], 512-len, "PMIC|%d|0x%02X|0x%04X|%s %s (1/%ld)\n",
						pmic_device_index,
						L1_poff_index==0?0:0x1<<(L1_poff_index-1),
						L2_poff_index==0?0:0x1<<(L2_poff_index-1),
						L1_str_ptr,
						L2_str_ptr,
						pmic_history_count);
	}

	memcpy(buf,page,len);
	return len;

}
pmic_info_attr_ro(poff_reason);
/**********************************************/

/**********************************************/
static char * const pon_pon_reason1_str[] = {
	[0] = "POWER ON by HARD_RESET",
	[1] = "POWER ON by SMPL",
	[2] = "POWER ON by RTC",
	[3] = "POWER ON by DC_CHG",
	[4] = "POWER ON by USB_CHG",
	[5] = "POWER ON by PON1",
	[6] = "POWER ON by CBLPWR_N",
	[7] = "POWER ON by KPDPWR_N",
};

static char * const pon_warm_reset_reason1_str[] = {
	[0] = ":SOFT",
	[1] = ":PS_HOLD",
	[2] = ":PMIC_WD",
	[3] = ":GP1",
	[4] = ":GP2",
	[5] = ":KPDPWR_AND_RESIN",
	[6] = ":RESIN_N",
	[7] = ":KPDPWR_N",
};

#define QPNP_WARM_SEQ BIT(6)

static ssize_t pon_reason_show(struct kobject *kobj,
			struct kobj_attribute *attr, char *buf) {
    char page[512] = {0};
    int len = 0;
	struct PMICHistoryKernelStruct *pmic_history_ptr = NULL;
	struct PMICRecordKernelStruct pmic_first_record;
	u8 pmic_device_index=0;
	u8 pon_index=0,pon_warm_reset_index=0;
	u8 pon_pon_reason1_reg_vaule=0,pon_warm_reset_reason1=0;
	u64 pmic_history_count=0;

	pmic_history_ptr = (struct PMICHistoryKernelStruct *)get_pmic_history();

	if (NULL == pmic_history_ptr) {
		len += snprintf(&page[len],512-len, "PMIC|0|0x000000000000\n");
		memcpy(buf,page,len);
		return len;
	}

	pmic_first_record = pmic_history_ptr->pmic_record[0];
	pmic_history_count = pmic_history_ptr->log_count;

	for (pmic_device_index = 0;pmic_device_index<8;pmic_device_index++) {
		if (DATA_VALID_FLAG != pmic_first_record.pmic_pon_poff_reason[pmic_device_index].data_is_valid) {
			continue;
		}

		pon_pon_reason1_reg_vaule = pmic_first_record.pmic_pon_poff_reason[pmic_device_index].PON_PON_REASON1;
		pon_index = ffs(pon_pon_reason1_reg_vaule);
		if (0 == pon_index) {
			len += snprintf(&page[len],512-len, "PMIC|%d|0x%02X|NKNOW PON REASON\n",pmic_device_index,0x0);
			continue;
		}

		if (QPNP_WARM_SEQ & pmic_first_record.pmic_pon_poff_reason[pmic_device_index].PON_ON_REASON) {
			pon_warm_reset_reason1 = pmic_first_record.pmic_pon_poff_reason[pmic_device_index].PON_WARM_RESET_REASON1;
			pon_warm_reset_index = ffs(pon_pon_reason1_reg_vaule);
			len += snprintf(&page[len],512-len, "PMIC|%d|0x%02X|WARM_SEQ:%s %s (1/%ld)\n",
													pmic_device_index,
													pon_pon_reason1_reg_vaule,
													pon_pon_reason1_str[pon_index-1],
													0==pon_warm_reset_index?"NULL":pon_warm_reset_reason1_str[pon_warm_reset_index-1],
													pmic_history_count);
		} else {
			len += snprintf(&page[len],512-len, "PMIC|%d|0x%02X|PON_SEQ:%s (1/%ld)\n",
													pmic_device_index,
													pon_pon_reason1_reg_vaule,
													pon_pon_reason1_str[pon_index-1],
													pmic_history_count);
		}
	}

	memcpy(buf,page,len);
	return len;
}
pmic_info_attr_ro(pon_reason);
/**********************************************/

/**********************************************/
static ssize_t ocp_status_show(struct kobject *kobj,
			struct kobj_attribute *attr, char *buf)
{
    char page[256] = {0};
    int len = 0;
	struct PMICHistoryKernelStruct *pmic_history_ptr = NULL;
	struct PMICRecordKernelStruct pmic_first_record;
	u8 pmic_device_index=0;

	pmic_history_ptr = (struct PMICHistoryKernelStruct *)get_pmic_history();

	if (NULL == pmic_history_ptr) {
		len += snprintf(&page[len],256-len, "PMIC|0|0x000000000000\n");
		memcpy(buf,page,len);
		return len;
	}

	pmic_first_record = pmic_history_ptr->pmic_record[0];
	for (pmic_device_index = 0;pmic_device_index<8;pmic_device_index++) {
		if (DATA_VALID_FLAG != pmic_first_record.pmic_pon_poff_reason[pmic_device_index].data_is_valid) {
			continue;
		}
		len += snprintf(&page[len],256-len, "PMIC|%d|0x%08X%02X%02X\n",
				pmic_device_index,
				pmic_first_record.pmic_pon_poff_reason[pmic_device_index].ldo_ocp_status,
				pmic_first_record.pmic_pon_poff_reason[pmic_device_index].spms_ocp_status,
				pmic_first_record.pmic_pon_poff_reason[pmic_device_index].bob_ocp_status);
	}

    memcpy(buf,page,len);
	return len;
}
pmic_info_attr_ro(ocp_status);

/**********************************************/

static struct attribute * g[] = {
    &pmic_history_magic_attr.attr,
	&pmic_history_count_attr.attr,
	&poff_reason_attr.attr,
	&pon_reason_attr.attr,
	&ocp_status_attr.attr,
	NULL,
};

static struct attribute_group attr_group = {
	.attrs = g,
};

struct kobject *pmic_info_kobj;

static int __init pmic_info_init(void)
{
	int error;

	pmic_info_kobj = kobject_create_and_add("pmic_info", NULL);
	if (!pmic_info_kobj)
		return -ENOMEM;
	error = sysfs_create_group(pmic_info_kobj, &attr_group);
	if (error)
		return error;
	return 0;
}

core_initcall(pmic_info_init);

MODULE_LICENSE("GPL v2");
