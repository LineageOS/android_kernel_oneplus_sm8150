KBUILD_CFLAGS +=   -DVENDOR_EDIT
KBUILD_CPPFLAGS += -DVENDOR_EDIT
CFLAGS_KERNEL +=   -DVENDOR_EDIT
CFLAGS_MODULE +=   -DVENDOR_EDIT
obj-$(CONFIG_POWER_AVS)		+= avs/
obj-$(CONFIG_POWER_RESET)	+= reset/
obj-$(CONFIG_POWER_SUPPLY)	+= supply/

#ifdef OPLUS_FEATURE_CHG_BASIC
obj-$(CONFIG_OPLUS_SM8150R_CHARGER)	+= oplus/
#endif
