# Copyright (C), 2008-2030, OPLUS Mobile Comm Corp., Ltd
### All rights reserved.
###
### File: - OplusKernelEnvConfig.mk
### Description:
###     you can get the oplus feature variables set in android side in this file
###     this file will add global macro for common oplus added feature
###     BSP team can do customzation by referring the feature variables
### Version: 1.0
### Date: 2020-03-18
### Author: Liang.Sun
###
### ------------------------------- Revision History: ----------------------------
### <author>                        <date>       <version>   <desc>
### ------------------------------------------------------------------------------
##################################################################################

ALLOWED_MCROS := \
OP_8150_ADAPT \
OPLUS_ARCH_EXTENDS \
OPLUS_BUG_STABILITY \
OPLUS_FEATURE_AEC \
OPLUS_FEATURE_CAMERA_COMMON \
OPLUS_FEATURE_CHG_BASIC \
OPLUS_FEATURE_KTV \
OPLUS_FEATURE_QCOM_PMICWD \
OPLUS_FEATURE_TP_BASIC \
VENDOR_EDIT

$(foreach myfeature,$(ALLOWED_MCROS),\
         $(warning make $(myfeature) to be a macro here) \
         $(eval KBUILD_CFLAGS += -D$(myfeature)) \
         $(eval KBUILD_CPPFLAGS += -D$(myfeature)) \
         $(eval CFLAGS_KERNEL += -D$(myfeature)) \
         $(eval CFLAGS_MODULE += -D$(myfeature)) \
)

# OPLUS_CUSTOM_OP_DEF
KBUILD_CFLAGS += -DOPLUS_CUSTOM_OP_DEF

# OPLUS_FEATURE_FSA4480
KBUILD_CFLAGS += -DOPLUS_FEATURE_FSA4480

# OPLUS_FEATURE_MM_ULTRASOUND
export OPLUS_FEATURE_MM_ULTRASOUND=y
KBUILD_CFLAGS += -DOPLUS_FEATURE_MM_ULTRASOUND

# CONFIG_ONEPLUS_MOTOR_OP7
export CONFIG_ONEPLUS_MOTOR_OP7=OP7
KBUILD_CFLAGS += -DCONFIG_ONEPLUS_MOTOR_OP7
