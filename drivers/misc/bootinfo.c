/*
 * bootinfo.c
 *
 * Copyright (C) 2011 Xiaomi Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/string.h>
#include <linux/bootinfo.h>
#include <linux/bitops.h>
//#include <linux/input/qpnp-power-on.h>
#include <linux/kobject.h>
#include <linux/slab.h>

static const char * const powerup_reasons[PU_REASON_MAX] = {
	[PU_REASON_EVENT_KPD]		= "keypad",
	[PU_REASON_EVENT_RTC]		= "rtc",
	[PU_REASON_EVENT_CABLE]		= "cable",
	[PU_REASON_EVENT_SMPL]		= "smpl",
	[PU_REASON_EVENT_PON1]		= "pon1",
	[PU_REASON_EVENT_USB_CHG]	= "usb_chg",
	[PU_REASON_EVENT_DC_CHG]	= "dc_chg",
	[PU_REASON_EVENT_HWRST]		= "hw_reset",
	[PU_REASON_EVENT_LPK]		= "long_power_key",
};

static const char * const powerdown_reasons[PD_REASON_MAX] = {
	[PD_REASON_EVENT_SOFT]		= "soft",
	[PD_REASON_EVENT_PS_HOLD]	= "ps_hold",
	[PD_REASON_EVENT_PMIC_WD]	= "pmic_wd",
	[PD_REASON_EVENT_GP1]		= "gp2",
	[PD_REASON_EVENT_GP2]		= "gp1",
	[PD_REASON_EVENT_KPDPWR_AND_RESIN]	= "kpd_and_resin",
	[PD_REASON_EVENT_RESIN]	        = "resin",
	[PD_REASON_EVENT_KPDPWR]	= "long_power_key",
};

static const char * const reset_reasons[RS_REASON_MAX] = {
	[RS_REASON_EVENT_WDOG]		= "wdog",
	[RS_REASON_EVENT_KPANIC]	= "kpanic",
	[RS_REASON_EVENT_NORMAL]	= "reboot",
	[RS_REASON_EVENT_OTHER]		= "other",
	[RS_REASON_EVENT_DVE]		= "dm_verity_enforcing",
	[RS_REASON_EVENT_DVL]		= "dm_verity_logging",
	[RS_REASON_EVENT_DVK]		= "dm_verity_keysclear",
	[RS_REASON_EVENT_FASTBOOT]	= "fastboot_reboot",
};

#define MAX_CMDLINE_PARAM_LEN 64
static struct kobject *bootinfo_kobj;
static char powerup_reason[MAX_CMDLINE_PARAM_LEN];
static char powerdown_reason[MAX_CMDLINE_PARAM_LEN];
u32 pu_reason = 0;
u32 pd_reason = 0;

#define bootinfo_attr(_name) \
static struct kobj_attribute _name##_attr = {	\
	.attr	= {				\
		.name = __stringify(_name),	\
		.mode = 0644,			\
	},					\
	.show	= _name##_show,			\
	.store	= NULL,				\
}

int is_abnormal_powerup(void)
{
	return (pu_reason & (RESTART_EVENT_KPANIC | RESTART_EVENT_WDOG)) |
		(pu_reason & BIT(PU_REASON_EVENT_HWRST) & RESTART_EVENT_OTHER);
}

static int get_powerup_reason(char *buf)
{
	int ret;
	if (!buf || !buf[0]) {
		pr_err("get_powerup_reason: pu_reason is null, powerup_reason:%s\n",powerup_reason);
		return -EINVAL;
	}
	ret = kstrtou32(buf, 16, &pu_reason);
	return 0;
}

static int get_powerdown_reason(char *buf)
{
	int ret;
	if (!buf || !buf[0]) {
		pr_err("get_powerdown_reason: pd_reason is null, powerdown_reason:%s\n", powerdown_reason);
		return -EINVAL;
	}
	ret = kstrtou32(buf, 16, &pd_reason);
	return 0;
}

static ssize_t powerup_reason_show(struct kobject *kobj,
			struct kobj_attribute *attr, char *buf)
{
	char *s = buf;
	int pu_reason_index = PU_REASON_MAX;
	u32 reset_reason;
	int reset_reason_index = RS_REASON_MAX;
	get_powerup_reason(powerup_reason);
	get_powerdown_reason(powerdown_reason);
	if (((pu_reason & BIT(PU_REASON_EVENT_HWRST))
		&& (pd_reason & BIT(PD_REASON_EVENT_PS_HOLD))) ||
		(pu_reason & BIT(PU_REASON_EVENT_WARMRST))) {
		reset_reason = pu_reason >> 16;
		reset_reason_index =
		find_first_bit((unsigned long *)&reset_reason,
				sizeof(reset_reason)*BITS_PER_BYTE);
		if (reset_reason_index < RS_REASON_MAX
			&& reset_reason_index >= 0) {
			if (reset_reason_index == RS_REASON_EVENT_FASTBOOT)
				reset_reason_index = RS_REASON_EVENT_NORMAL;
			s += snprintf(s,
				strlen(reset_reasons[reset_reason_index]) + 2,
				"%s\n", reset_reasons[reset_reason_index]);
			pr_err("%s: rs_reason [0x%x], first non-zero bit %d\n",
				__func__, reset_reason, reset_reason_index);
			goto out;
		};
	}
	if (pd_reason & BIT(PD_REASON_EVENT_KPDPWR))
		pu_reason_index = PU_REASON_EVENT_LPK;
	else if (pu_reason & BIT(PU_REASON_EVENT_HWRST))
		pu_reason_index = PU_REASON_EVENT_HWRST;
	else if (pu_reason & BIT(PU_REASON_EVENT_SMPL))
		pu_reason_index = PU_REASON_EVENT_SMPL;
	else if (pu_reason & BIT(PU_REASON_EVENT_RTC))
		pu_reason_index = PU_REASON_EVENT_RTC;
	else if (pu_reason & BIT(PU_REASON_EVENT_USB_CHG))
		pu_reason_index = PU_REASON_EVENT_USB_CHG;
	else if (pu_reason & BIT(PU_REASON_EVENT_DC_CHG))
		pu_reason_index = PU_REASON_EVENT_DC_CHG;
	else if (pu_reason & BIT(PU_REASON_EVENT_KPD))
		pu_reason_index = PU_REASON_EVENT_KPD;
	else if (pu_reason & BIT(PU_REASON_EVENT_PON1))
		pu_reason_index = PU_REASON_EVENT_PON1;
	if (pu_reason_index < PU_REASON_MAX && pu_reason_index >= 0) {
		s += snprintf(s, strlen(powerup_reasons[pu_reason_index]) + 2,
				"%s\n", powerup_reasons[pu_reason_index]);
		pr_err("%s: pu_reason [0x%x] index %d\n",
			__func__, pu_reason, pu_reason_index);
		goto out;
	}
	s += snprintf(s, 15, "unknown reboot\n");
out:
	return (s - buf);
}

static ssize_t powerup_reason_details_show(struct kobject *kobj,
			struct kobj_attribute *attr, char *buf)
{
	get_powerup_reason(powerup_reason);
  	pr_err("%s: pu_reason = 0x%x\n",
			__func__, pu_reason);
	return snprintf(buf, 11, "0x%x\n", pu_reason);
}

static ssize_t poweroff_reason_show(struct kobject *kobj,
			struct kobj_attribute *attr, char *buf)
{
	char *s = buf;
	int pd_reason_index = PD_REASON_MAX;

	get_powerdown_reason(powerdown_reason);
	if (pd_reason & BIT(PD_REASON_EVENT_SOFT))
		pd_reason_index = PD_REASON_EVENT_SOFT;
	else if (pd_reason & BIT(PD_REASON_EVENT_PS_HOLD))
		pd_reason_index = PD_REASON_EVENT_PS_HOLD;
	else if (pd_reason & BIT(PD_REASON_EVENT_PMIC_WD))
		pd_reason_index = PD_REASON_EVENT_PMIC_WD;
	else if (pd_reason & BIT(PD_REASON_EVENT_GP1))
		pd_reason_index = PD_REASON_EVENT_GP1;
	else if (pd_reason & BIT(PD_REASON_EVENT_GP2))
		pd_reason_index = PD_REASON_EVENT_GP2;
	else if (pd_reason & BIT(PD_REASON_EVENT_KPDPWR_AND_RESIN))
		pd_reason_index = PD_REASON_EVENT_KPDPWR_AND_RESIN;
	else if (pd_reason & BIT(PD_REASON_EVENT_RESIN))
		pd_reason_index = PD_REASON_EVENT_RESIN;
	else if (pd_reason & BIT(PD_REASON_EVENT_KPDPWR))
		pd_reason_index = PD_REASON_EVENT_KPDPWR;
	if (pd_reason_index < PD_REASON_MAX && pd_reason_index >= 0) {
		s += snprintf(s, strlen(powerdown_reasons[pd_reason_index]) + 2,
				"%s\n", powerdown_reasons[pd_reason_index]);
		pr_debug("%s: pd_reason [0x%x] index %d\n",
			__func__, pd_reason, pd_reason_index);
		goto out;
	}
	s += snprintf(s, 15, "unknown reboot\n");
out:
	return (s - buf);
}

static ssize_t poweroff_reason_details_show(struct kobject *kobj,
			struct kobj_attribute *attr, char *buf)
{
	get_powerdown_reason(powerdown_reason);
	return snprintf(buf, 11, "0x%x\n", pd_reason);
}

bootinfo_attr(powerup_reason);
bootinfo_attr(powerup_reason_details);
bootinfo_attr(poweroff_reason);
bootinfo_attr(poweroff_reason_details);

static struct attribute *g[] = {
	&powerup_reason_attr.attr,
	&powerup_reason_details_attr.attr,
	&poweroff_reason_attr.attr,
	&poweroff_reason_details_attr.attr,
	NULL,
};

static struct attribute_group attr_group = {
	.attrs = g,
};

static int __init bootinfo_init(void)
{
	int ret = -ENOMEM;

	bootinfo_kobj = kobject_create_and_add("bootinfo", NULL);
	if (bootinfo_kobj == NULL) {
		pr_err("bootinfo_init: subsystem_register failed\n");
		goto fail;
	}

	ret = sysfs_create_group(bootinfo_kobj, &attr_group);
	if (ret) {
		pr_err("bootinfo_init: subsystem_register failed\n");
		goto sys_fail;
	}

	return ret;

sys_fail:
	kobject_del(bootinfo_kobj);
fail:
	return ret;

}

static void __exit bootinfo_exit(void)
{
	if (bootinfo_kobj) {
		sysfs_remove_group(bootinfo_kobj, &attr_group);
		kobject_del(bootinfo_kobj);
	}
}

MODULE_LICENSE("GPL");
core_initcall(bootinfo_init);
module_exit(bootinfo_exit);

module_param_string(pureason, powerup_reason, MAX_CMDLINE_PARAM_LEN,0644);
module_param_string(pdreason, powerdown_reason, MAX_CMDLINE_PARAM_LEN,0644);
