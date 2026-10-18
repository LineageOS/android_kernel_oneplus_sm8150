// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2020-2022 Oplus. All rights reserved.
 */

#include <linux/configfs.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/nls.h>
#include <linux/kdev_t.h>

#include "oplus_charger.h"
#include "oplus_gauge.h"
#include "oplus_vooc.h"
#include "oplus_short.h"
#include "oplus_adapter.h"
#include "oplus_wireless.h"
#include "charger_ic/oplus_short_ic.h"
#include "oplus_debug_info.h"

static struct class *oplus_chg_class;
static struct device *oplus_usb_dir;
static struct device *oplus_battery_dir;
static struct device *oplus_wireless_dir;

/**********************************************************************
* usb device nodes
**********************************************************************/
int __attribute__((weak)) oplus_get_otg_online_status(void)
{
	return 0;
}

static ssize_t otg_online_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct oplus_chg_chip *chip = NULL;
	int otg_online = 0;

	chip = (struct oplus_chg_chip *)dev_get_drvdata(oplus_usb_dir);
	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	otg_online = oplus_get_otg_online_status();

	return sprintf(buf, "%d\n", otg_online);
}
static DEVICE_ATTR_RO(otg_online);

bool __attribute__((weak)) oplus_get_otg_switch_status(void)
{
	return false;
}

static ssize_t otg_switch_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct oplus_chg_chip *chip = NULL;

	chip = (struct oplus_chg_chip *)dev_get_drvdata(oplus_usb_dir);
	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	oplus_get_otg_switch_status();
	return sprintf(buf, "%d\n", chip->otg_switch);
}

void __attribute__((weak)) oplus_set_otg_switch_status(bool value)
{
	return;
}

static ssize_t otg_switch_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int val = 0;
	struct oplus_chg_chip *chip = NULL;

	chip = (struct oplus_chg_chip *)dev_get_drvdata(oplus_usb_dir);
	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	if (kstrtos32(buf, 0, &val)) {
		chg_err("buf error\n");
		return -EINVAL;
	}

	if (val == 1) {
		chip->otg_switch = true;
		oplus_set_otg_switch_status(true);
	} else {
		chip->otg_switch = false;
		chip->otg_online = false;
		oplus_set_otg_switch_status(false);
	}

	return count;
}
static DEVICE_ATTR_RW(otg_switch);

int __attribute__((weak)) oplus_get_usb_status(void)
{
	return 0;
}

static ssize_t usb_status_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int status = 0;

	status = oplus_get_usb_status();
	return sprintf(buf, "%d\n", status);
}
static DEVICE_ATTR_RO(usb_status);

int __attribute__((weak)) oplus_get_usbtemp_volt_l(void)
{
	return 0;
}

static ssize_t usbtemp_volt_l_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int volt = 0;

	volt = oplus_get_usbtemp_volt_l();
	return sprintf(buf, "%d\n", volt);
}
static DEVICE_ATTR_RO(usbtemp_volt_l);

int __attribute__((weak)) oplus_get_usbtemp_volt_r(void)
{
	return 0;
}

static ssize_t usbtemp_volt_r_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int volt = 0;

	volt = oplus_get_usbtemp_volt_r();
	return sprintf(buf, "%d\n", volt);
}
static DEVICE_ATTR_RO(usbtemp_volt_r);

static ssize_t fast_chg_type_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct oplus_chg_chip *chip = NULL;
	int type = oplus_chg_get_fast_chg_type();

	chip = (struct oplus_chg_chip *)dev_get_drvdata(oplus_usb_dir);
	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	return sprintf(buf, "%d\n", type);
}
static DEVICE_ATTR_RO(fast_chg_type);

static struct device_attribute *oplus_usb_attributes[] = {
	&dev_attr_otg_online,	&dev_attr_otg_switch,	  &dev_attr_usb_status,
	&dev_attr_fast_chg_type, &dev_attr_usbtemp_volt_l, &dev_attr_usbtemp_volt_r,
	NULL
};

/**********************************************************************
* battery device nodes
**********************************************************************/
static ssize_t authenticate_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct oplus_chg_chip *chip = NULL;

	chip = (struct oplus_chg_chip *)dev_get_drvdata(oplus_battery_dir);
	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	return sprintf(buf, "%d\n", chip->authenticate);
}
static DEVICE_ATTR_RO(authenticate);

static ssize_t battery_cc_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct oplus_chg_chip *chip = NULL;

	chip = (struct oplus_chg_chip *)dev_get_drvdata(oplus_battery_dir);
	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	return sprintf(buf, "%d\n", chip->batt_cc);
}
static DEVICE_ATTR_RO(battery_cc);

static ssize_t battery_fcc_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct oplus_chg_chip *chip = NULL;

	chip = (struct oplus_chg_chip *)dev_get_drvdata(oplus_battery_dir);
	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	return sprintf(buf, "%d\n", chip->batt_fcc);
}
static DEVICE_ATTR_RO(battery_fcc);

static ssize_t battery_rm_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct oplus_chg_chip *chip = NULL;

	chip = (struct oplus_chg_chip *)dev_get_drvdata(oplus_battery_dir);
	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	if (oplus_vooc_get_fastchg_started() == true) {
		chip->batt_rm = oplus_gauge_get_prev_remaining_capacity() * chip->vbatt_num;
	} else {
		chip->batt_rm = oplus_gauge_get_remaining_capacity() * chip->vbatt_num;
	}

	return sprintf(buf, "%d\n", chip->batt_rm);
}
static DEVICE_ATTR_RO(battery_rm);

static ssize_t battery_soh_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct oplus_chg_chip *chip = NULL;

	chip = (struct oplus_chg_chip *)dev_get_drvdata(oplus_battery_dir);
	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	return sprintf(buf, "%d\n", chip->batt_soh);
}
static DEVICE_ATTR_RO(battery_soh);

static ssize_t charge_technology_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct oplus_chg_chip *chip = NULL;

	chip = (struct oplus_chg_chip *)dev_get_drvdata(oplus_battery_dir);
	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	return sprintf(buf, "%d\n", chip->vooc_project);
}
static DEVICE_ATTR_RO(charge_technology);

#ifdef CONFIG_OPLUS_CHIP_SOC_NODE
static ssize_t chip_soc_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct oplus_chg_chip *chip = NULL;

	chip = (struct oplus_chg_chip *)dev_get_drvdata(oplus_battery_dir);
	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	return sprintf(buf, "%d\n", chip->soc);
}
static DEVICE_ATTR_RO(chip_soc);
#endif /*CONFIG_OPLUS_CHIP_SOC_NODE*/

#ifdef CONFIG_OPLUS_SMART_CHARGER_SUPPORT
static ssize_t cool_down_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct oplus_chg_chip *chip = NULL;

	chip = (struct oplus_chg_chip *)dev_get_drvdata(oplus_battery_dir);
	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	return sprintf(buf, "%d\n", chip->cool_down);
}

static ssize_t cool_down_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int val = 0;
	struct oplus_chg_chip *chip = NULL;

	chip = (struct oplus_chg_chip *)dev_get_drvdata(oplus_battery_dir);
	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	if (kstrtos32(buf, 0, &val)) {
		chg_err("buf error\n");
		return -EINVAL;
	}
	oplus_smart_charge_by_cool_down(chip, val);

	return count;
}
static DEVICE_ATTR_RW(cool_down);
#endif /*CONFIG_OPLUS_SMART_CHARGER_SUPPORT*/

static ssize_t fast_charge_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int val = 0;
	struct oplus_chg_chip *chip = NULL;

	chip = (struct oplus_chg_chip *)dev_get_drvdata(oplus_battery_dir);
	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}
	val = oplus_chg_show_vooc_logo_ornot();

	return sprintf(buf, "%d\n", val);
}
static DEVICE_ATTR_RO(fast_charge);

static ssize_t mmi_charging_enable_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct oplus_chg_chip *chip = NULL;

	chip = (struct oplus_chg_chip *)dev_get_drvdata(oplus_battery_dir);
	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	return sprintf(buf, "%d\n", chip->mmi_chg);
}

static ssize_t mmi_charging_enable_store(struct device *dev, struct device_attribute *attr, const char *buf,
					 size_t count)
{
	int val = 0;
	int ret = 0;
	struct oplus_chg_chip *chip = NULL;

	chip = (struct oplus_chg_chip *)dev_get_drvdata(oplus_battery_dir);
	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	if (kstrtos32(buf, 0, &val)) {
		chg_err("buf error\n");
		return -EINVAL;
	}

	chg_err("set mmi_chg = [%d].\n", val);
	if (val == 0) {
		if (chip->unwakelock_chg == 1) {
			ret = -EINVAL;
			chg_err("unwakelock testing , this test not allowed.\n");
		} else {
			chip->prop_status = POWER_SUPPLY_STATUS_NOT_CHARGING;
			chip->mmi_chg = 0;
			oplus_chg_turn_off_charging(chip);
			if (oplus_vooc_get_fastchg_started() == true) {
				oplus_vooc_turn_off_fastchg();
				chip->mmi_fastchg = 0;
			}
		}
	} else {
		if (chip->unwakelock_chg == 1) {
			ret = -EINVAL;
			chg_err("unwakelock testing , this test not allowed.\n");
		} else {
			chip->mmi_chg = 1;
			chip->prop_status = POWER_SUPPLY_STATUS_CHARGING;
			if (oplus_vooc_get_fastchg_started() == false) {
				oplus_chg_clear_chargerid_info();
				reset_mcu_delay = 0;
				oplus_vooc_reset_fastchg_after_usbout();
			}
			chip->mmi_fastchg = 1;
			oplus_chg_turn_on_charging(chip);
		}
	}

	return ret < 0 ? ret : count;
}
static DEVICE_ATTR_RW(mmi_charging_enable);

static ssize_t battery_notify_code_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct oplus_chg_chip *chip = NULL;

	chip = (struct oplus_chg_chip *)dev_get_drvdata(oplus_battery_dir);
	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	return sprintf(buf, "%d\n", chip->notify_code);
}
static DEVICE_ATTR_RO(battery_notify_code);

static ssize_t charge_timeout_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct oplus_chg_chip *chip = NULL;

	chip = (struct oplus_chg_chip *)dev_get_drvdata(oplus_battery_dir);
	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	return sprintf(buf, "%d\n", chip->chging_over_time);
}
static DEVICE_ATTR_RO(charge_timeout);

static ssize_t adapter_fw_update_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct oplus_chg_chip *chip = NULL;

	chip = (struct oplus_chg_chip *)dev_get_drvdata(oplus_battery_dir);
	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	return sprintf(buf, "%d\n", oplus_vooc_get_adapter_update_status());
}
static DEVICE_ATTR_RO(adapter_fw_update);

#ifdef CONFIG_OPLUS_CHECK_CHARGERID_VOLT
static ssize_t chargerid_volt_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct oplus_chg_chip *chip = NULL;

	chip = (struct oplus_chg_chip *)dev_get_drvdata(oplus_battery_dir);
	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	return sprintf(buf, "%d\n", chip->chargerid_volt);
}
static DEVICE_ATTR_RO(chargerid_volt);
#endif /*CONFIG_OPLUS_CHECK_CHARGERID_VOLT*/

#ifdef CONFIG_OPLUS_SHIP_MODE_SUPPORT
static ssize_t ship_mode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct oplus_chg_chip *chip = NULL;

	chip = (struct oplus_chg_chip *)dev_get_drvdata(oplus_battery_dir);
	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	return sprintf(buf, "%d\n", chip->enable_shipmode);
}

static ssize_t ship_mode_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int val = 0;
	struct oplus_chg_chip *chip = NULL;

	chip = (struct oplus_chg_chip *)dev_get_drvdata(oplus_battery_dir);
	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	if (kstrtos32(buf, 0, &val)) {
		chg_err("buf error\n");
		return -EINVAL;
	}
	chip->enable_shipmode = val;
	oplus_gauge_update_soc_smooth_parameter();

	return count;
}
static DEVICE_ATTR_RW(ship_mode);
#endif /*CONFIG_OPLUS_SHIP_MODE_SUPPORT*/

#ifdef CONFIG_OPLUS_SHORT_C_BATT_CHECK
#ifdef CONFIG_OPLUS_SHORT_USERSPACE
static ssize_t short_c_limit_chg_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct oplus_chg_chip *chip = NULL;

	chip = (struct oplus_chg_chip *)dev_get_drvdata(oplus_battery_dir);
	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	return sprintf(buf, "%d\n", (int)chip->short_c_batt.limit_chg);
}

static ssize_t short_c_limit_chg_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int val = 0;
	struct oplus_chg_chip *chip = NULL;

	chip = (struct oplus_chg_chip *)dev_get_drvdata(oplus_battery_dir);
	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	if (kstrtos32(buf, 0, &val)) {
		chg_err("buf error\n");
		return -EINVAL;
	}

	printk(KERN_ERR "[OPLUS_CHG] [short_c_bat] set limit chg[%d]\n", !!val);
	chip->short_c_batt.limit_chg = !!val;
	/* for userspace logic */
	if (!!val == 0) {
		chip->short_c_batt.is_switch_on = 0;
	}

	return count;
}
static DEVICE_ATTR_RW(short_c_limit_chg);

static ssize_t short_c_limit_rechg_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct oplus_chg_chip *chip = NULL;

	chip = (struct oplus_chg_chip *)dev_get_drvdata(oplus_battery_dir);
	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	return sprintf(buf, "%d\n", (int)chip->short_c_batt.limit_rechg);
}

static ssize_t short_c_limit_rechg_store(struct device *dev, struct device_attribute *attr, const char *buf,
					 size_t count)
{
	int val = 0;
	struct oplus_chg_chip *chip = NULL;

	chip = (struct oplus_chg_chip *)dev_get_drvdata(oplus_battery_dir);
	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	if (kstrtos32(buf, 0, &val)) {
		chg_err("buf error\n");
		return -EINVAL;
	}

	printk(KERN_ERR "[OPLUS_CHG] [short_c_bat] set limit rechg[%d]\n", !!val);
	chip->short_c_batt.limit_rechg = !!val;

	return count;
}
static DEVICE_ATTR_RW(short_c_limit_rechg);

static ssize_t charge_term_current_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct oplus_chg_chip *chip = NULL;

	chip = (struct oplus_chg_chip *)dev_get_drvdata(oplus_battery_dir);
	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	return sprintf(buf, "%d\n", chip->limits.iterm_ma);
}
static DEVICE_ATTR_RO(charge_term_current);

static ssize_t input_current_settled_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int val = 0;
	struct oplus_chg_chip *chip = NULL;

	chip = (struct oplus_chg_chip *)dev_get_drvdata(oplus_battery_dir);
	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	val = 2000;
	if (chip && chip->chg_ops->get_dyna_aicl_result) {
		val = chip->chg_ops->get_dyna_aicl_result();
	}

	return sprintf(buf, "%d\n", val);
}
static DEVICE_ATTR_RO(input_current_settled);
#endif /*CONFIG_OPLUS_SHORT_USERSPACE*/
#endif /*CONFIG_OPLUS_SHORT_C_BATT_CHECK*/

#ifdef CONFIG_OPLUS_SHORT_HW_CHECK
static ssize_t short_c_hw_feature_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct oplus_chg_chip *chip = NULL;

	chip = (struct oplus_chg_chip *)dev_get_drvdata(oplus_battery_dir);
	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	return sprintf(buf, "%d\n", chip->short_c_batt.is_feature_hw_on);
}

static ssize_t short_c_hw_feature_store(struct device *dev, struct device_attribute *attr, const char *buf,
					size_t count)
{
	int val = 0;
	struct oplus_chg_chip *chip = NULL;

	chip = (struct oplus_chg_chip *)dev_get_drvdata(oplus_battery_dir);
	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	if (kstrtos32(buf, 0, &val)) {
		chg_err("buf error\n");
		return -EINVAL;
	}

	printk(KERN_ERR "[OPLUS_CHG] [short_c_hw_check]: set is_feature_hw_on [%d]\n", val);
	chip->short_c_batt.is_feature_hw_on = val;

	return count;
}
static DEVICE_ATTR_RW(short_c_hw_feature);

static ssize_t short_c_hw_status_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct oplus_chg_chip *chip = NULL;

	chip = (struct oplus_chg_chip *)dev_get_drvdata(oplus_battery_dir);
	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	return sprintf(buf, "%d\n", chip->short_c_batt.shortc_gpio_status);
}
static DEVICE_ATTR_RO(short_c_hw_status);
#endif /*CONFIG_OPLUS_SHORT_HW_CHECK*/

#ifdef CONFIG_OPLUS_SHORT_IC_CHECK
static ssize_t short_ic_otp_status_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct oplus_chg_chip *chip = NULL;

	chip = (struct oplus_chg_chip *)dev_get_drvdata(oplus_battery_dir);
	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	return sprintf(buf, "%d\n", chip->short_c_batt.ic_short_otp_st);
}
static DEVICE_ATTR_RO(short_ic_otp_status);

static ssize_t short_ic_volt_thresh_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct oplus_chg_chip *chip = NULL;

	chip = (struct oplus_chg_chip *)dev_get_drvdata(oplus_battery_dir);
	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	return sprintf(buf, "%d\n", chip->short_c_batt.ic_volt_threshold);
}

static ssize_t short_ic_volt_thresh_store(struct device *dev, struct device_attribute *attr, const char *buf,
					  size_t count)
{
	int val = 0;
	struct oplus_chg_chip *chip = NULL;

	chip = (struct oplus_chg_chip *)dev_get_drvdata(oplus_battery_dir);
	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	if (kstrtos32(buf, 0, &val)) {
		chg_err("buf error\n");
		return -EINVAL;
	}

	chip->short_c_batt.ic_volt_threshold = val;
	oplus_short_ic_set_volt_threshold(chip);

	return count;
}
static DEVICE_ATTR_RW(short_ic_volt_thresh);

static ssize_t short_ic_otp_value_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct oplus_chg_chip *chip = NULL;

	chip = (struct oplus_chg_chip *)dev_get_drvdata(oplus_battery_dir);
	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	return sprintf(buf, "%d\n", oplus_short_ic_get_otp_error_value(chip));
}
static DEVICE_ATTR_RO(short_ic_otp_value);
#endif /*CONFIG_OPLUS_SHORT_IC_CHECK*/

static ssize_t voocchg_ing_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int val = 0;
	struct oplus_chg_chip *chip = NULL;

	chip = (struct oplus_chg_chip *)dev_get_drvdata(oplus_battery_dir);
	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	val = oplus_vooc_get_fastchg_ing();
#ifndef WPC_NEW_INTERFACE
	if (!val && chip->wireless_support) {
		val = oplus_wpc_get_fast_charging();
	}
#else
	if (!val && chip->wireless_support) {
		val = oplus_wpc_get_status();
	}
#endif

	return sprintf(buf, "%d\n", val);
}
static DEVICE_ATTR_RO(voocchg_ing);

static struct device_attribute *oplus_battery_attributes[] = { &dev_attr_authenticate,
							       &dev_attr_battery_cc,
							       &dev_attr_battery_fcc,
							       &dev_attr_battery_rm,
							       &dev_attr_battery_soh,
							       &dev_attr_charge_technology,
#ifdef CONFIG_OPLUS_CHIP_SOC_NODE
							       &dev_attr_chip_soc,
#endif
#ifdef CONFIG_OPLUS_SMART_CHARGER_SUPPORT
							       &dev_attr_cool_down,
#endif
							       &dev_attr_fast_charge,
							       &dev_attr_mmi_charging_enable,
							       &dev_attr_battery_notify_code,
							       &dev_attr_charge_timeout,
							       &dev_attr_adapter_fw_update,
#ifdef CONFIG_OPLUS_CHECK_CHARGERID_VOLT
							       &dev_attr_chargerid_volt,
#endif
#ifdef CONFIG_OPLUS_SHIP_MODE_SUPPORT
							       &dev_attr_ship_mode,
#endif
#ifdef CONFIG_OPLUS_SHORT_C_BATT_CHECK
#ifdef CONFIG_OPLUS_SHORT_USERSPACE
							       &dev_attr_short_c_limit_chg,
							       &dev_attr_short_c_limit_rechg,
							       &dev_attr_charge_term_current,
							       &dev_attr_input_current_settled,
#endif
#endif
#ifdef CONFIG_OPLUS_SHORT_HW_CHECK
							       &dev_attr_short_c_hw_feature,
							       &dev_attr_short_c_hw_status,
#endif
#ifdef CONFIG_OPLUS_SHORT_IC_CHECK
							       &dev_attr_short_ic_otp_status,
							       &dev_attr_short_ic_volt_thresh,
							       &dev_attr_short_ic_otp_value,
#endif
							       &dev_attr_voocchg_ing,
							       NULL };

/**********************************************************************
* wireless device nodes
**********************************************************************/
static ssize_t max_w_power_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct oplus_chg_chip *chip = NULL;

	chip = (struct oplus_chg_chip *)dev_get_drvdata(oplus_wireless_dir);
	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	return sprintf(buf, "%d\n", oplus_wpc_get_max_wireless_power());
}
static DEVICE_ATTR_RO(max_w_power);

static struct device_attribute *oplus_wireless_attributes[] = {
	&dev_attr_max_w_power,
	NULL
};

static int oplus_usb_dir_create(struct oplus_chg_chip *chip)
{
	int status = 0;
	dev_t devt;
	struct device_attribute **attrs;
	struct device_attribute *attr;

	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	status = alloc_chrdev_region(&devt, 0, 1, "usb");
	if (status < 0) {
		chg_err("alloc_chrdev_region usb fail!\n");
		return -ENOMEM;
	}
	oplus_usb_dir = device_create(oplus_chg_class, NULL, devt, NULL, "%s", "usb");
	oplus_usb_dir->devt = devt;
	dev_set_drvdata(oplus_usb_dir, chip);

	attrs = oplus_usb_attributes;
	for (attr = *attrs; attr != NULL; attr = *(++attrs)) {
		int err;

		err = device_create_file(oplus_usb_dir, attr);
		if (err) {
			chg_err("device_create_file fail!\n");
			device_destroy(oplus_usb_dir->class, oplus_usb_dir->devt);
			return err;
		}
	}

	return 0;
}

static void oplus_usb_dir_destroy(void)
{
	struct device_attribute **attrs;
	struct device_attribute *attr;

	attrs = oplus_usb_attributes;
	for (attr = *attrs; attr != NULL; attr = *(++attrs))
		device_remove_file(oplus_usb_dir, attr);
	device_destroy(oplus_usb_dir->class, oplus_usb_dir->devt);
	unregister_chrdev_region(oplus_usb_dir->devt, 1);
}

static int oplus_battery_dir_create(struct oplus_chg_chip *chip)
{
	int status = 0;
	dev_t devt;
	struct device_attribute **attrs;
	struct device_attribute *attr;

	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	status = alloc_chrdev_region(&devt, 0, 1, "battery");
	if (status < 0) {
		chg_err("alloc_chrdev_region battery fail!\n");
		return -ENOMEM;
	}

	oplus_battery_dir = device_create(oplus_chg_class, NULL, devt, NULL, "%s", "battery");
	oplus_battery_dir->devt = devt;
	dev_set_drvdata(oplus_battery_dir, chip);

	attrs = oplus_battery_attributes;
	for (attr = *attrs; attr != NULL; attr = *(++attrs)) {
		int err;

		err = device_create_file(oplus_battery_dir, attr);
		if (err) {
			chg_err("device_create_file fail!\n");
			device_destroy(oplus_battery_dir->class, oplus_battery_dir->devt);
			return err;
		}
	}

	return 0;
}

static void oplus_battery_dir_destroy(void)
{
	struct device_attribute **attrs;
	struct device_attribute *attr;

	attrs = oplus_battery_attributes;
	for (attr = *attrs; attr != NULL; attr = *(++attrs))
		device_remove_file(oplus_battery_dir, attr);
	device_destroy(oplus_battery_dir->class, oplus_battery_dir->devt);
	unregister_chrdev_region(oplus_battery_dir->devt, 1);
}

static void oplus_wireless_dir_destroy(void)
{
	struct device_attribute **attrs;
	struct device_attribute *attr;

	attrs = oplus_wireless_attributes;
	while ((attr = *attrs++))
		device_remove_file(oplus_wireless_dir, attr);
	device_destroy(oplus_wireless_dir->class, oplus_wireless_dir->devt);
	unregister_chrdev_region(oplus_wireless_dir->devt, 1);
}

int oplus_usb_node_add(struct device_attribute **usb_attributes)
{
	struct device_attribute **attrs;
	struct device_attribute *attr;

	attrs = usb_attributes;
	for (attr = *attrs; attr != NULL; attr = *(++attrs)) {
		int err;

		err = device_create_file(oplus_usb_dir, attr);
		if (err) {
			chg_err("device_create_file fail!\n");
			device_destroy(oplus_usb_dir->class, oplus_usb_dir->devt);
			return err;
		}
	}

	return 0;
}

void oplus_usb_node_delete(struct device_attribute **usb_attributes)
{
	struct device_attribute **attrs;
	struct device_attribute *attr;

	attrs = usb_attributes;
	for (attr = *attrs; attr != NULL; attr = *(++attrs))
		device_remove_file(oplus_usb_dir, attr);
}

int oplus_battery_node_add(struct device_attribute **battery_attributes)
{
	struct device_attribute **attrs;
	struct device_attribute *attr;

	attrs = battery_attributes;
	for (attr = *attrs; attr != NULL; attr = *(++attrs)) {
		int err;

		err = device_create_file(oplus_battery_dir, attr);
		if (err) {
			chg_err("device_create_file fail!\n");
			device_destroy(oplus_battery_dir->class, oplus_battery_dir->devt);
			return err;
		}
	}

	return 0;
}

void oplus_battery_node_delete(struct device_attribute **battery_attributes)
{
	struct device_attribute **attrs;
	struct device_attribute *attr;

	attrs = battery_attributes;
	for (attr = *attrs; attr != NULL; attr = *(++attrs))
		device_remove_file(oplus_battery_dir, attr);
}

int oplus_wireless_node_add(struct device_attribute **wireless_attributes)
{
	struct device_attribute **attrs;
	struct device_attribute *attr;

	attrs = wireless_attributes;
	while ((attr = *attrs++)) {
		int err;

		err = device_create_file(oplus_wireless_dir, attr);
		if (err) {
			chg_err("device_create_file fail!\n");
			device_destroy(oplus_wireless_dir->class, oplus_wireless_dir->devt);
			return err;
		}
	}

	return 0;
}


static int oplus_wireless_dir_create(struct oplus_chg_chip *chip)
{
	int status = 0;
	dev_t devt;
	struct device_attribute **attrs;
	struct device_attribute *attr;

	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	status = alloc_chrdev_region(&devt, 0, 1, "wireless");
	if (status < 0) {
		chg_err("alloc_chrdev_region wireless fail!\n");
		return -ENOMEM;
	}

	oplus_wireless_dir = device_create(oplus_chg_class, NULL,
			devt, NULL, "%s", "wireless");
	oplus_wireless_dir->devt = devt;
	dev_set_drvdata(oplus_wireless_dir, chip);

	attrs = oplus_wireless_attributes;
	while ((attr = *attrs++)) {
		int err;

		err = device_create_file(oplus_wireless_dir, attr);
		if (err) {
			chg_err("device_create_file fail!\n");
			device_destroy(oplus_wireless_dir->class, oplus_wireless_dir->devt);
			return err;
		}
	}

	return 0;
}

void oplus_wireless_node_delete(struct device_attribute **wireless_attributes)
{
	struct device_attribute **attrs;
	struct device_attribute *attr;

	attrs = wireless_attributes;
	while ((attr = *attrs++))
		device_remove_file(oplus_wireless_dir, attr);
}

int oplus_chg_configfs_init(struct oplus_chg_chip *chip)
{
	int status = 0;

	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	oplus_chg_class = class_create(THIS_MODULE, "oplus_chg");
	if (IS_ERR(oplus_chg_class)) {
		chg_err("oplus_chg_configfs_init fail!\n");
		return PTR_ERR(oplus_chg_class);
	}

	status = oplus_usb_dir_create(chip);
	if (status < 0)
		chg_err("oplus_usb_dir_create fail!\n");

	status = oplus_battery_dir_create(chip);
	if (status < 0)
		chg_err("oplus_battery_dir_create fail!\n");

	status = oplus_wireless_dir_create(chip);
	if (status < 0)
		chg_err("oplus_wireless_dir_create fail!\n");

	return 0;
}
EXPORT_SYMBOL(oplus_chg_configfs_init);

int oplus_chg_configfs_exit(void)
{
	oplus_wireless_dir_destroy();
	oplus_battery_dir_destroy();
	oplus_usb_dir_destroy();

	if (!IS_ERR(oplus_chg_class))
		class_destroy(oplus_chg_class);
	return 0;
}
EXPORT_SYMBOL(oplus_chg_configfs_exit);

