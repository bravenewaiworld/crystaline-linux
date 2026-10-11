// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * X-Powers AXP2202 battery charger and fuel gauge driver.
 *
 * Based on Allwinner's BSP driver (github.com/radxa/allwinner-bsp).
 * Copyright(c) 2020 - 2023 Allwinner Technology Co.,Ltd. All rights reserved.
 */

#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/mfd/axp2101.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_wakeup.h>
#include <linux/power_supply.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/workqueue.h>

/* Battery voltage readings outside this range are reported as 0 */
#define AXP2202_VBAT_MIN_MV		2000
#define AXP2202_VBAT_MAX_MV		8000
#define AXP2202_ICHG_MAX_MA		3072
#define AXP2202_ICHG_STEP_MA		64
#define AXP2202_MODEL_MAX_LEN		512	/* battery model blob size */
#define AXP2202_NTC_POINTS		16
#define AXP2202_MONITOR_INTERVAL	msecs_to_jiffies(10 * 1000)
#define AXP2202_SMOOTH_INTERVAL		msecs_to_jiffies(30 * 1000)
/* Reported temperature limit when there is no NTC, in 0.1 degC */
#define AXP2202_NO_NTC_LIMIT		200000

/*
 * SOC smoothing: after a gauge reset the driver reports a SOC that climbs
 * slowly towards the gauge's, and keeps it in AXP2202_CURVE_CHECK so that it
 * survives reboots. Bit 7 = smoothing active, bits 6:0 = SOC to report.
 */
#define AXP2202_CURVE_ACTIVE		BIT(7)
#define AXP2202_CURVE_SOC		GENMASK(6, 0)

/*
 * Gauge memory is read indirectly: write an address to AXP2202_GAUGE_FG_ADDR,
 * then read AXP2202_GAUGE_FG_DATA_H/L. This one holds the high 16 bits of
 * the coulomb counter.
 */
#define AXP2202_FG_COULOMB_HIGH		0x2e

/* Temperatures in 0.1 degC at the DT points pmu_bat_temp_para1..16 */
static const int axp2202_ntc_temps[AXP2202_NTC_POINTS] = {
	-250, -150, -100, -50, 0, 50, 100, 200,
	300, 400, 450, 500, 550, 600, 700, 800,
};

/* Fields are named after the DT properties they are read from. */
struct axp2202_bat_config {
	u32 pmu_battery_cap;			/* mAh */
	u32 pmu_runtime_chgcur;			/* mA */
	u32 pmu_suspend_chgcur;			/* mA */
	u32 pmu_shutdown_chgcur;		/* mA */
	u32 pmu_terminal_chgcur;		/* mA */
	u32 pmu_init_chgvol;			/* mV */
	u32 pmu_battery_warning_level1;		/* % */
	u32 pmu_battery_warning_level2;		/* % */
	u32 pmu_chgled_func;			/* PMIC drives the charge LED */
	u32 pmu_chgled_type;			/* raw LED mode code */
	u32 pmu_bat_temp_enable;		/* battery has an NTC */
	u32 pmu_bat_ts_current;			/* NTC bias current, uA */
	/*
	 * TS pin voltages (mV) bounding the charging window and the
	 * operating window. "jetia" is the vendor DT's spelling of JEITA.
	 */
	u32 pmu_bat_charge_ltf;
	u32 pmu_bat_charge_htf;
	u32 pmu_bat_shutdown_ltf;
	u32 pmu_bat_shutdown_htf;
	u32 pmu_jetia_en;
	u32 pmu_jetia_cool;
	u32 pmu_jetia_warm;
	/* raw 2-bit charge current reduction codes for the JEITA bands */
	u32 pmu_jcool_ifall;
	u32 pmu_jwarm_ifall;
	/* TS pin voltage (mV) at each of axp2202_ntc_temps, descending */
	u32 ntc_mv[AXP2202_NTC_POINTS];
};

/* IRQ resource name, and the DT flag that keeps it enabled during suspend */
static const struct {
	const char *name;
	const char *wakeup_prop;
} axp2202_bat_irqs[] = {
	{ "battery_insert",		"wakeup_bat_in" },
	{ "battery_remove",		"wakeup_bat_out" },
	{ "charge_start",		"wakeup_bat_charging" },
	{ "charge_done",		"wakeup_bat_charge_over" },
	{ "SOC_low_warning1",		"wakeup_low_warning1" },
	{ "SOC_low_warning2",		"wakeup_low_warning2" },
	{ "battery_under_temp_work",	"wakeup_bat_untemp_work" },
	{ "battery_over_temp_work",	"wakeup_bat_ovtemp_work" },
	{ "battery_under_temp_chg",	"wakeup_bat_untemp_chg" },
	{ "battery_over_temp_chg",	"wakeup_bat_ovtemp_chg" },
	{ "battery_over_voltage",	"wakeup_bat_ov" },
	{ "gauge_new_soc",		"wakeup_new_soc" },
};

struct axp2202_bat_power {
	struct device *dev;
	struct regmap *regmap;
	struct power_supply *bat_supply;
	struct axp2202_bat_config cfg;
	struct delayed_work monitor_work;
	struct delayed_work curve_work;
	struct wakeup_source *curve_ws;
	/* virqs to disable during suspend, 0 for the others */
	int sleep_irqs[ARRAY_SIZE(axp2202_bat_irqs)];
};

static enum power_supply_property axp2202_bat_props[] = {
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_CHARGE_COUNTER,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CAPACITY_LEVEL,
	POWER_SUPPLY_PROP_CAPACITY_ALERT_MIN,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_TEMP_ALERT_MIN,
	POWER_SUPPLY_PROP_TEMP_ALERT_MAX,
	POWER_SUPPLY_PROP_TEMP_AMBIENT_ALERT_MIN,
	POWER_SUPPLY_PROP_TEMP_AMBIENT_ALERT_MAX,
	POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW,
	POWER_SUPPLY_PROP_TIME_TO_FULL_NOW,
	POWER_SUPPLY_PROP_MANUFACTURER,
};

/* Reads a big-endian register pair, masking the high byte with @hi_mask. */
static int axp2202_read_u16(struct regmap *regmap, unsigned int reg,
			    u8 hi_mask, unsigned int *val)
{
	u8 data[2];
	int ret;

	ret = regmap_bulk_read(regmap, reg, data, 2);
	if (ret < 0)
		return ret;

	*val = ((data[0] & hi_mask) << 8) | data[1];
	return 0;
}

/* Median of three reads, to filter out single noisy samples. */
static int axp2202_read_u16_median(struct regmap *regmap, unsigned int reg,
				   u8 hi_mask, unsigned int *val)
{
	unsigned int v[3];
	int i, ret;

	for (i = 0; i < 3; i++) {
		ret = axp2202_read_u16(regmap, reg, hi_mask, &v[i]);
		if (ret < 0)
			return ret;
	}

	if (v[0] > v[1])
		swap(v[0], v[1]);
	if (v[1] > v[2])
		swap(v[1], v[2]);
	if (v[0] > v[1])
		swap(v[0], v[1]);

	*val = v[1];
	return 0;
}

/* Converts a TS pin voltage (mV) to 0.1 degC using the DT NTC curve. */
static int axp2202_ntc_to_temp(const struct axp2202_bat_config *cfg, int mv)
{
	const u32 *p = cfg->ntc_mv;
	const int *t = axp2202_ntc_temps;
	int i;

	/* without an NTC, report a fixed 30.0 degC */
	if (!cfg->pmu_bat_temp_enable)
		return 300;

	/* Clamp outside the table, interpolate linearly inside it. */

	if (mv < p[AXP2202_NTC_POINTS - 1])
		return t[AXP2202_NTC_POINTS - 1];

	for (i = AXP2202_NTC_POINTS - 2; i >= 0; i--) {
		if (mv > p[i])
			continue;
		if (p[i] == p[i + 1])
			return t[i];
		return t[i] + ((int)p[i] - mv) * (t[i + 1] - t[i]) /
			      ((int)p[i] - (int)p[i + 1]);
	}

	return t[0];
}

/*
 * Converts a DT threshold to 0.1 degC; @sign picks which end of the scale
 * stands for "no limit" when there is no NTC.
 */
static int axp2202_ntc_limit(const struct axp2202_bat_config *cfg, u32 mv,
			     int sign)
{
	if (!cfg->pmu_bat_temp_enable)
		return sign * AXP2202_NO_NTC_LIMIT;
	return axp2202_ntc_to_temp(cfg, mv);
}

static int axp2202_read_ts_temp(struct axp2202_bat_power *bat_power, int *temp)
{
	struct regmap *regmap = bat_power->regmap;
	unsigned int raw;
	int i, ret, prev;

	ret = regmap_update_bits(regmap, AXP2202_ADC_DATA_SEL, AXP2202_ADC_SEL,
				 AXP2202_ADC_SEL_TS);
	if (ret < 0)
		return ret;
	usleep_range(1000, 2000);	/* let the ADC settle on the channel */

	/* Re-read until two samples agree within 10 degC, at most 11 reads. */
	for (i = 0; i <= 10; i++) {
		ret = axp2202_read_u16(regmap, AXP2202_ADC_DATA_H,
				       GENMASK(5, 0), &raw);
		if (ret < 0)
			return ret;

		prev = *temp;
		/* the ADC counts in 0.5 mV steps */
		*temp = axp2202_ntc_to_temp(&bat_power->cfg, raw / 2);
		if (i > 0 && abs(*temp - prev) <= 100)
			break;
	}

	return 0;
}

static int axp2202_get_bat_present(struct axp2202_bat_power *bat_power,
				   int *present)
{
	unsigned int data;
	int ret;

	ret = regmap_read(bat_power->regmap, AXP2202_COMM_STAT0, &data);
	if (ret < 0)
		return ret;

	*present = !!(data & AXP2202_BAT_PRESENT);
	return 0;
}

static int axp2202_get_bat_health(struct axp2202_bat_power *bat_power,
				  int *health)
{
	unsigned int stat, fault;
	int ret;

	ret = regmap_read(bat_power->regmap, AXP2202_COMM_STAT0, &stat);
	if (ret < 0)
		return ret;
	ret = regmap_read(bat_power->regmap, AXP2202_COMM_FAULT, &fault);
	if (ret < 0)
		return ret;

	if (!(stat & AXP2202_BAT_PRESENT))
		*health = POWER_SUPPLY_HEALTH_UNKNOWN;
	else if (stat & AXP2202_BAT_ACTIVATING)
		*health = POWER_SUPPLY_HEALTH_DEAD;
	else if (fault & AXP2202_BAT_OVER_TEMP)
		*health = POWER_SUPPLY_HEALTH_OVERHEAT;
	else if (fault & AXP2202_BAT_UNDER_TEMP)
		*health = POWER_SUPPLY_HEALTH_COLD;
	else
		*health = POWER_SUPPLY_HEALTH_GOOD;

	return 0;
}

static int axp2202_get_bat_status(struct axp2202_bat_power *bat_power,
				  int *status)
{
	struct regmap *regmap = bat_power->regmap;
	unsigned int data;
	int ret;

	ret = regmap_read(regmap, AXP2202_COMM_STAT1, &data);
	if (ret < 0)
		return ret;

	switch (data & AXP2202_CHG_STATE) {
	/*
	 * Trickle charging only revives an over-discharged cell; it is
	 * reported as discharging, as Allwinner's driver does.
	 */
	case AXP2202_CHG_STATE_TRICKLE:
	case AXP2202_CHG_STATE_NONE:
		*status = POWER_SUPPLY_STATUS_DISCHARGING;
		break;
	case AXP2202_CHG_STATE_PRE:
	case AXP2202_CHG_STATE_CC:
	case AXP2202_CHG_STATE_CV:
		*status = POWER_SUPPLY_STATUS_CHARGING;
		break;
	case AXP2202_CHG_STATE_DONE:
		*status = POWER_SUPPLY_STATUS_FULL;
		break;
	default:
		*status = POWER_SUPPLY_STATUS_UNKNOWN;
		break;
	}

	/*
	 * With VBUS present, report full once the gauge reads 100%, even if
	 * the charger is still in CV. This ignores SOC smoothing.
	 */
	ret = regmap_read(regmap, AXP2202_COMM_STAT0, &data);
	if (ret < 0)
		return ret;

	if (data & AXP2202_VBUS_GOOD) {
		ret = regmap_read(regmap, AXP2202_GAUGE_SOC, &data);
		if (ret < 0)
			return ret;
		if (data == 100)
			*status = POWER_SUPPLY_STATUS_FULL;
	}

	return 0;
}

static int axp2202_get_soc(struct axp2202_bat_power *bat_power, int *soc)
{
	struct regmap *regmap = bat_power->regmap;
	unsigned int data;
	int ret, status;

	ret = regmap_read(regmap, AXP2202_CURVE_CHECK, &data);
	if (ret < 0)
		return ret;

	/* report the smoothed SOC while smoothing is active */
	if (data & AXP2202_CURVE_ACTIVE)
		data &= AXP2202_CURVE_SOC;
	else
		ret = regmap_read(regmap, AXP2202_GAUGE_SOC, &data);
	if (ret < 0)
		return ret;

	ret = axp2202_get_bat_status(bat_power, &status);
	if (ret < 0)
		return ret;
	/* the charger is done but the gauge stopped at 99% */
	if (status == POWER_SUPPLY_STATUS_FULL && data == 99)
		data = 100;

	*soc = min(data, 100U);
	return 0;
}

static int axp2202_get_capacity_level(struct axp2202_bat_power *bat_power,
				      int *level)
{
	const struct axp2202_bat_config *cfg = &bat_power->cfg;
	int ret, present, soc;

	ret = axp2202_get_bat_present(bat_power, &present);
	if (ret < 0)
		return ret;

	if (!present) {
		*level = POWER_SUPPLY_CAPACITY_LEVEL_UNKNOWN;
		return 0;
	}

	ret = axp2202_get_soc(bat_power, &soc);
	if (ret < 0)
		return ret;

	if (soc == 100)
		*level = POWER_SUPPLY_CAPACITY_LEVEL_FULL;
	else if (soc > 80)
		*level = POWER_SUPPLY_CAPACITY_LEVEL_HIGH;
	else if (soc > cfg->pmu_battery_warning_level1)
		*level = POWER_SUPPLY_CAPACITY_LEVEL_NORMAL;
	else if (soc > cfg->pmu_battery_warning_level2)
		*level = POWER_SUPPLY_CAPACITY_LEVEL_LOW;
	else
		*level = POWER_SUPPLY_CAPACITY_LEVEL_CRITICAL;

	return 0;
}

/* Returns the configured constant charge current in mA. */
static int axp2202_get_ichg(struct axp2202_bat_power *bat_power, int *mA)
{
	unsigned int data;
	int ret;

	ret = regmap_read(bat_power->regmap, AXP2202_ICC_CFG, &data);
	if (ret < 0)
		return ret;

	*mA = (data & GENMASK(5, 0)) * AXP2202_ICHG_STEP_MA;
	return 0;
}

static u32 axp2202_ichg_round(u32 mA)
{
	return rounddown(min_t(u32, mA, AXP2202_ICHG_MAX_MA),
			 AXP2202_ICHG_STEP_MA);
}

static int axp2202_set_ichg(struct axp2202_bat_power *bat_power, int mA)
{
	mA = clamp(mA, 0, AXP2202_ICHG_MAX_MA);

	return regmap_update_bits(bat_power->regmap, AXP2202_ICC_CFG,
				  GENMASK(5, 0), mA / AXP2202_ICHG_STEP_MA);
}

static int axp2202_bat_get_property(struct power_supply *psy,
				    enum power_supply_property psp,
				    union power_supply_propval *val)
{
	struct axp2202_bat_power *bat_power = power_supply_get_drvdata(psy);
	const struct axp2202_bat_config *cfg = &bat_power->cfg;
	struct regmap *regmap = bat_power->regmap;
	unsigned int data;
	int ret = 0;

	switch (psp) {
	case POWER_SUPPLY_PROP_PRESENT:
		ret = axp2202_get_bat_present(bat_power, &val->intval);
		break;
	case POWER_SUPPLY_PROP_STATUS:
		ret = axp2202_get_bat_status(bat_power, &val->intval);
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		ret = axp2202_get_bat_health(bat_power, &val->intval);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		ret = axp2202_read_u16_median(regmap, AXP2202_VBAT_H,
					      GENMASK(5, 0), &data);
		if (ret < 0)
			break;
		if (data < AXP2202_VBAT_MIN_MV || data > AXP2202_VBAT_MAX_MV)
			data = 0;
		val->intval = data * 1000;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		/* only charge current is measured (0 on battery), 0.5 mA/LSB */
		ret = axp2202_read_u16(regmap, AXP2202_ICHG_H, GENMASK(5, 0),
				       &data);
		val->intval = data * 500;
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
		ret = axp2202_get_ichg(bat_power, &val->intval);
		val->intval *= 1000;
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		val->intval = cfg->pmu_battery_cap * 1000;
		break;
	case POWER_SUPPLY_PROP_CHARGE_COUNTER:
		/* estimated from SOC and design capacity, in uAh */
		ret = axp2202_get_soc(bat_power, &val->intval);
		val->intval = val->intval * cfg->pmu_battery_cap * 10;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		ret = axp2202_get_soc(bat_power, &val->intval);
		break;
	case POWER_SUPPLY_PROP_CAPACITY_LEVEL:
		ret = axp2202_get_capacity_level(bat_power, &val->intval);
		break;
	case POWER_SUPPLY_PROP_CAPACITY_ALERT_MIN:
		/* see the AXP2202_GAUGE_THLD encoding in axp2202_init_chip() */
		ret = regmap_read(regmap, AXP2202_GAUGE_THLD, &data);
		val->intval = (data >> 4) + 5;
		break;
	case POWER_SUPPLY_PROP_TEMP:
		ret = axp2202_read_ts_temp(bat_power, &val->intval);
		break;
	/*
	 * TEMP_ALERT_* report the operating window (the PMIC stops running
	 * from the battery outside it); TEMP_AMBIENT_ALERT_* are used for the
	 * charging window (charging pauses outside it).
	 */
	case POWER_SUPPLY_PROP_TEMP_ALERT_MIN:
		val->intval = axp2202_ntc_limit(cfg, cfg->pmu_bat_shutdown_ltf,
						-1);
		break;
	case POWER_SUPPLY_PROP_TEMP_ALERT_MAX:
		val->intval = axp2202_ntc_limit(cfg, cfg->pmu_bat_shutdown_htf,
						1);
		break;
	case POWER_SUPPLY_PROP_TEMP_AMBIENT_ALERT_MIN:
		val->intval = axp2202_ntc_limit(cfg, cfg->pmu_bat_charge_ltf,
						-1);
		break;
	case POWER_SUPPLY_PROP_TEMP_AMBIENT_ALERT_MAX:
		val->intval = axp2202_ntc_limit(cfg, cfg->pmu_bat_charge_htf,
						1);
		break;
	/* the gauge counts in minutes */
	case POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW:
		ret = axp2202_read_u16_median(regmap,
					      AXP2202_GAUGE_TIME2EMPTY_H,
					      0xff, &data);
		val->intval = data * 60;
		break;
	case POWER_SUPPLY_PROP_TIME_TO_FULL_NOW:
		ret = axp2202_read_u16_median(regmap, AXP2202_GAUGE_TIME2FULL_H,
					      0xff, &data);
		val->intval = data * 60;
		break;
	case POWER_SUPPLY_PROP_MANUFACTURER:
		val->strval = "xpower,axp2202";
		break;
	default:
		return -EINVAL;
	}

	return ret;
}

static const struct power_supply_desc axp2202_bat_desc = {
	.name = "axp2202-battery",
	.type = POWER_SUPPLY_TYPE_BATTERY,
	.get_property = axp2202_bat_get_property,
	.properties = axp2202_bat_props,
	.num_properties = ARRAY_SIZE(axp2202_bat_props),
};

/*
 * Restarts the fuel gauge's MCU, with charging paused for 500 ms on each
 * side. Sleeps about a second. Always leaves charging enabled.
 */
static int axp2202_reset_gauge(struct axp2202_bat_power *bat_power)
{
	struct regmap *regmap = bat_power->regmap;
	int ret;

	regmap_update_bits(regmap, AXP2202_MODULE_EN, AXP2202_CHG_EN, 0);
	msleep(500);

	ret = regmap_update_bits(regmap, AXP2202_RESET_CFG,
				 AXP2202_RESET_GAUGE_MCU,
				 AXP2202_RESET_GAUGE_MCU);
	if (!ret)
		ret = regmap_update_bits(regmap, AXP2202_RESET_CFG,
					 AXP2202_RESET_GAUGE_MCU, 0);

	msleep(500);
	regmap_update_bits(regmap, AXP2202_MODULE_EN, AXP2202_CHG_EN,
			   AXP2202_CHG_EN);

	return ret;
}

/*
 * Reads the battery model blob: the "param" phandle points at a node whose
 * "select" string names the child holding the "parameter" byte array.
 */
static int axp2202_get_model(struct axp2202_bat_power *bat_power, u8 *model,
			     size_t max_len)
{
	struct device_node *params, *selected;
	const char *name;
	int len = -ENODATA;

	params = of_parse_phandle(bat_power->dev->of_node, "param", 0);
	if (!params)
		return -ENODATA;

	if (!of_property_read_string(params, "select", &name)) {
		selected = of_get_child_by_name(params, name);
		if (selected) {
			len = of_property_read_variable_u8_array(selected,
					"parameter", model, 1, max_len);
			of_node_put(selected);
		}
	}

	of_node_put(params);
	return len;
}

/* Opens the battery model memory with its address pointer at the start. */
static int axp2202_rewind_model(struct regmap *regmap)
{
	int ret;

	ret = regmap_update_bits(regmap, AXP2202_GAUGE_CONFIG,
				 AXP2202_GAUGE_BROM_EN, 0);
	if (ret < 0)
		return ret;
	return regmap_update_bits(regmap, AXP2202_GAUGE_CONFIG,
				  AXP2202_GAUGE_BROM_EN,
				  AXP2202_GAUGE_BROM_EN);
}

/* Uploads the battery model from DT into the fuel gauge, then restarts it. */
static int axp2202_load_model(struct axp2202_bat_power *bat_power)
{
	struct regmap *regmap = bat_power->regmap;
	unsigned int data;
	u8 *model;
	int i, len, ret;

	model = kzalloc(AXP2202_MODEL_MAX_LEN, GFP_KERNEL);
	if (!model)
		return -ENOMEM;

	len = axp2202_get_model(bat_power, model, AXP2202_MODEL_MAX_LEN);
	if (len < 0) {
		ret = len;
		goto out;
	}

	/* cancel SOC smoothing; it tracked the previous model */
	regmap_write(regmap, AXP2202_CURVE_CHECK, 0);

	ret = axp2202_rewind_model(regmap);
	if (ret < 0)
		goto out_reset;

	for (i = 0; i < len; i++) {
		ret = regmap_write(regmap, AXP2202_GAUGE_BROM, model[i]);
		if (ret < 0)
			goto out_reset;
	}

	/* rewind again to read back and verify */
	ret = axp2202_rewind_model(regmap);
	if (ret < 0)
		goto out_reset;

	for (i = 0; i < len; i++) {
		ret = regmap_read(regmap, AXP2202_GAUGE_BROM, &data);
		if (ret < 0)
			goto out_reset;
		if (data != model[i]) {
			dev_err(bat_power->dev,
				"battery model mismatch at byte %d\n", i);
			ret = -EIO;
			goto out_reset;
		}
	}

	ret = regmap_update_bits(regmap, AXP2202_GAUGE_CONFIG,
				 AXP2202_GAUGE_BROM_EN, 0);
	if (ret < 0)
		goto out_reset;
	ret = regmap_update_bits(regmap, AXP2202_GAUGE_CONFIG,
				 AXP2202_GAUGE_MODEL_LOADED,
				 AXP2202_GAUGE_MODEL_LOADED);
	if (ret < 0)
		goto out_reset;

	ret = axp2202_reset_gauge(bat_power);
	goto out;

out_reset:
	/* close model memory and keep the gauge running on its old model */
	regmap_update_bits(regmap, AXP2202_GAUGE_CONFIG,
			   AXP2202_GAUGE_BROM_EN, 0);
	axp2202_reset_gauge(bat_power);
out:
	kfree(model);
	return ret;
}

/* AXP2202_TS_CURRENT code: 0 = 20 uA, 1 = 40 uA, 2 = 50 uA, 3 = 60 uA */
static u8 axp2202_ntc_current_bits(u32 uA)
{
	if (uA < 40)
		return 0;
	if (uA < 50)
		return 1;
	if (uA < 60)
		return 2;
	return 3;
}

/* Rounds down to a supported charge target voltage. */
static u8 axp2202_vterm_bits(u32 mV)
{
	if (mV < 4100)
		return AXP2202_VTERM_4_0V;
	if (mV < 4200)
		return AXP2202_VTERM_4_1V;
	if (mV < 4350)
		return AXP2202_VTERM_4_2V;
	if (mV < 4400)
		return AXP2202_VTERM_4_35V;
	if (mV < 5000)
		return AXP2202_VTERM_4_4V;
	return AXP2202_VTERM_5_0V;
}

/* Programs the NTC thresholds; the vendor DT gives them in mV. */
static void axp2202_init_ntc(struct axp2202_bat_power *bat_power)
{
	struct axp2202_bat_config *cfg = &bat_power->cfg;
	struct regmap *regmap = bat_power->regmap;
	unsigned int val;

	regmap_update_bits(regmap, AXP2202_TS_CFG,
			   AXP2202_TS_NOT_NTC | AXP2202_TS_CURRENT,
			   axp2202_ntc_current_bits(cfg->pmu_bat_ts_current));

	/*
	 * Colder means a higher TS voltage. Keep charge_ltf >= charge_htf and
	 * make the operating window enclose the charging window. A DT value
	 * of 0 keeps the chip default. Register steps: VLTF 32 mV, VHTF 2 mV
	 * (8 bits, so at most 510 mV).
	 */
	if (cfg->pmu_bat_charge_ltf) {
		cfg->pmu_bat_charge_ltf = max(cfg->pmu_bat_charge_ltf,
					      cfg->pmu_bat_charge_htf);
		regmap_write(regmap, AXP2202_VLTF_CHG,
			     cfg->pmu_bat_charge_ltf / 32);
	}
	if (cfg->pmu_bat_charge_htf) {
		cfg->pmu_bat_charge_htf = min(cfg->pmu_bat_charge_htf, 510U);
		regmap_write(regmap, AXP2202_VHTF_CHG,
			     cfg->pmu_bat_charge_htf / 2);
	}
	if (cfg->pmu_bat_shutdown_ltf) {
		cfg->pmu_bat_shutdown_ltf = max(cfg->pmu_bat_shutdown_ltf,
						cfg->pmu_bat_charge_ltf);
		regmap_write(regmap, AXP2202_VLTF_WORK,
			     cfg->pmu_bat_shutdown_ltf / 32);
	}
	if (cfg->pmu_bat_shutdown_htf) {
		cfg->pmu_bat_shutdown_htf = min(cfg->pmu_bat_shutdown_htf,
						cfg->pmu_bat_charge_htf);
		regmap_write(regmap, AXP2202_VHTF_WORK,
			     cfg->pmu_bat_shutdown_htf / 2);
	}

	if (!cfg->pmu_jetia_en) {
		regmap_update_bits(regmap, AXP2202_JEITA_CFG,
				   AXP2202_JEITA_EN, 0);
		return;
	}

	/* register steps: JEITA_COOL 16 mV, JEITA_WARM 8 mV */
	regmap_update_bits(regmap, AXP2202_JEITA_CFG, AXP2202_JEITA_EN,
			   AXP2202_JEITA_EN);
	if (cfg->pmu_jetia_cool) {
		cfg->pmu_jetia_cool = max(cfg->pmu_jetia_cool,
					  cfg->pmu_jetia_warm);
		regmap_write(regmap, AXP2202_JEITA_COOL,
			     cfg->pmu_jetia_cool / 16);
	}
	if (cfg->pmu_jetia_warm) {
		cfg->pmu_jetia_warm = min(cfg->pmu_jetia_warm, 2040U);
		regmap_write(regmap, AXP2202_JEITA_WARM,
			     cfg->pmu_jetia_warm / 8);
	}

	/* Charge current reduction when warm (bits 7:6) and cool (bits 5:4) */
	val = (cfg->pmu_jwarm_ifall << 6) | (cfg->pmu_jcool_ifall << 4);
	regmap_update_bits(regmap, AXP2202_JEITA_CV_CFG, GENMASK(7, 4), val);
}

static int axp2202_init_chip(struct axp2202_bat_power *bat_power)
{
	struct axp2202_bat_config *cfg = &bat_power->cfg;
	struct regmap *regmap = bat_power->regmap;
	unsigned int val;
	int ret, present, status;

	ret = axp2202_get_bat_present(bat_power, &present);
	if (ret < 0)
		return ret;

	/*
	 * The model and its loaded flag survive reboots while the PMU stays
	 * powered, so it is only uploaded when the flag is clear.
	 */
	ret = regmap_read(regmap, AXP2202_GAUGE_CONFIG, &val);
	if (ret < 0)
		return ret;
	if (present && !(val & AXP2202_GAUGE_MODEL_LOADED)) {
		dev_info(bat_power->dev, "loading battery model\n");
		ret = axp2202_load_model(bat_power);
		if (ret < 0)
			dev_err(bat_power->dev,
				"failed to load battery model: %d\n", ret);
	}

	/* pre-charge and termination currents, 64 mA steps */
	regmap_update_bits(regmap, AXP2202_IPRECHG_CFG, GENMASK(3, 0), 2);
	regmap_update_bits(regmap, AXP2202_ITERM_CFG, GENMASK(3, 0),
			   clamp(cfg->pmu_terminal_chgcur / 64, 1U, 15U));

	/* without a battery there is no NTC to read */
	if (!present)
		cfg->pmu_bat_temp_enable = 0;

	if (cfg->pmu_bat_temp_enable) {
		axp2202_init_ntc(bat_power);
	} else {
		regmap_update_bits(regmap, AXP2202_TS_CFG, AXP2202_TS_NOT_NTC,
				   AXP2202_TS_NOT_NTC);
		regmap_update_bits(regmap, AXP2202_JEITA_CFG,
				   AXP2202_JEITA_EN, 0);
	}

	if (cfg->pmu_chgled_func)
		regmap_update_bits(regmap, AXP2202_CHGLED_CFG, GENMASK(2, 0),
				   cfg->pmu_chgled_type);

	regmap_update_bits(regmap, AXP2202_VTERM_CFG, AXP2202_VTERM,
			   axp2202_vterm_bits(cfg->pmu_init_chgvol));

	/* Round to what the charger can do, so readback compares equal. */
	cfg->pmu_runtime_chgcur = axp2202_ichg_round(cfg->pmu_runtime_chgcur);
	cfg->pmu_suspend_chgcur = axp2202_ichg_round(cfg->pmu_suspend_chgcur);
	cfg->pmu_shutdown_chgcur =
		axp2202_ichg_round(cfg->pmu_shutdown_chgcur);
	axp2202_set_ichg(bat_power, cfg->pmu_runtime_chgcur);

	/*
	 * Low SOC warning levels: bits 7:4 = level1 - 5 (5..20%),
	 * bits 3:0 = level2 (0..15%).
	 */
	val = clamp((int)cfg->pmu_battery_warning_level1 - 5, 0, 15) << 4;
	val |= min(cfg->pmu_battery_warning_level2, 15U);
	regmap_write(regmap, AXP2202_GAUGE_THLD, val);

	/*
	 * Gauge SOC above 60% but VBAT below 3.5 V while charging means the
	 * gauge is wrong: restart it and report a smoothed SOC that climbs
	 * from 1% (see axp2202_curve_work). With VBAT >= 3.5 V the gauge is
	 * plausible, so cancel any smoothing left from a previous boot. At
	 * 60% or below, smoothing state is left alone.
	 */
	ret = regmap_read(regmap, AXP2202_GAUGE_SOC, &val);
	if (ret < 0)
		return ret;
	if (val > 60) {
		ret = axp2202_read_u16(regmap, AXP2202_VBAT_H, GENMASK(5, 0),
				       &val);
		if (ret < 0)
			return ret;
		if (val >= 3500) {
			regmap_write(regmap, AXP2202_CURVE_CHECK, 0);
		} else if (!axp2202_get_bat_status(bat_power, &status) &&
			   status == POWER_SUPPLY_STATUS_CHARGING) {
			dev_warn(bat_power->dev,
				 "SOC too high for %u mV, resetting gauge\n",
				 val);
			axp2202_reset_gauge(bat_power);
			regmap_write(regmap, AXP2202_CURVE_CHECK,
				     AXP2202_CURVE_ACTIVE | 1);
		}
	}

	return 0;
}

#define AXP2202_DT_U32(np, cfg, name, def)			\
	do {							\
		if (of_property_read_u32(np, #name, &(cfg)->name)) \
			(cfg)->name = def;			\
	} while (0)

static void axp2202_bat_parse_dt(struct axp2202_bat_power *bat_power)
{
	struct device_node *np = bat_power->dev->of_node;
	struct axp2202_bat_config *cfg = &bat_power->cfg;
	char name[32];
	int i;

	AXP2202_DT_U32(np, cfg, pmu_battery_cap, 4000);
	AXP2202_DT_U32(np, cfg, pmu_runtime_chgcur, 500);
	AXP2202_DT_U32(np, cfg, pmu_suspend_chgcur, 1200);
	AXP2202_DT_U32(np, cfg, pmu_shutdown_chgcur, 1200);
	AXP2202_DT_U32(np, cfg, pmu_terminal_chgcur, 128);
	AXP2202_DT_U32(np, cfg, pmu_init_chgvol, 4200);
	AXP2202_DT_U32(np, cfg, pmu_battery_warning_level1, 15);
	AXP2202_DT_U32(np, cfg, pmu_battery_warning_level2, 0);
	AXP2202_DT_U32(np, cfg, pmu_chgled_func, 1);
	AXP2202_DT_U32(np, cfg, pmu_chgled_type, 0);

	AXP2202_DT_U32(np, cfg, pmu_bat_temp_enable, 0);
	AXP2202_DT_U32(np, cfg, pmu_bat_ts_current, 50);
	AXP2202_DT_U32(np, cfg, pmu_bat_charge_ltf, 1312);
	AXP2202_DT_U32(np, cfg, pmu_bat_charge_htf, 176);
	AXP2202_DT_U32(np, cfg, pmu_bat_shutdown_ltf, 1984);
	AXP2202_DT_U32(np, cfg, pmu_bat_shutdown_htf, 152);

	AXP2202_DT_U32(np, cfg, pmu_jetia_en, 0);
	AXP2202_DT_U32(np, cfg, pmu_jetia_cool, 880);
	AXP2202_DT_U32(np, cfg, pmu_jetia_warm, 240);
	AXP2202_DT_U32(np, cfg, pmu_jcool_ifall, 1);
	AXP2202_DT_U32(np, cfg, pmu_jwarm_ifall, 0);

	for (i = 0; i < AXP2202_NTC_POINTS; i++) {
		snprintf(name, sizeof(name), "pmu_bat_temp_para%d", i + 1);
		of_property_read_u32(np, name, &cfg->ntc_mv[i]);
	}
}

static irqreturn_t axp2202_bat_irq(int irq, void *data)
{
	struct axp2202_bat_power *bat_power = data;

	power_supply_changed(bat_power->bat_supply);

	return IRQ_HANDLED;
}

static void axp2202_monitor_work(struct work_struct *work)
{
	struct axp2202_bat_power *bat_power =
		container_of(work, struct axp2202_bat_power, monitor_work.work);
	struct regmap *regmap = bat_power->regmap;
	unsigned int val;

	power_supply_changed(bat_power->bat_supply);

	/*
	 * The coulomb counter is signed and normally fits in 16 bits, so its
	 * high word is 0 or 0xffff; anything else is an overflow and the
	 * gauge is restarted. Reading FG_ADDR back checks that the gauge
	 * accepted the address.
	 */
	regmap_write(regmap, AXP2202_GAUGE_FG_ADDR, AXP2202_FG_COULOMB_HIGH);
	if (!regmap_read(regmap, AXP2202_GAUGE_CONFIG, &val) &&
	    (val & AXP2202_GAUGE_MODEL_LOADED) &&
	    !regmap_read(regmap, AXP2202_GAUGE_FG_ADDR, &val) &&
	    val == AXP2202_FG_COULOMB_HIGH &&
	    !axp2202_read_u16(regmap, AXP2202_GAUGE_FG_DATA_H, 0xff, &val) &&
	    val != 0xffff && val != 0) {
		dev_warn(bat_power->dev,
			 "coulomb counter overflow, resetting gauge\n");
		axp2202_reset_gauge(bat_power);
	}

	schedule_delayed_work(&bat_power->monitor_work,
			      AXP2202_MONITOR_INTERVAL);
}

/*
 * Raises the smoothed SOC by 1% per interval while VBUS is present. Ends
 * once the gauge SOC is no longer above it, at which point the reported
 * SOC jumps to the gauge's. On battery the smoothed SOC stays frozen.
 * A wakeup source is held throughout.
 */
static void axp2202_curve_work(struct work_struct *work)
{
	struct axp2202_bat_power *bat_power =
		container_of(work, struct axp2202_bat_power, curve_work.work);
	struct regmap *regmap = bat_power->regmap;
	unsigned int curve, soc, stat0;

	power_supply_changed(bat_power->bat_supply);

	if (regmap_read(regmap, AXP2202_CURVE_CHECK, &curve) ||
	    regmap_read(regmap, AXP2202_GAUGE_SOC, &soc) ||
	    !(curve & AXP2202_CURVE_ACTIVE) ||
	    min(soc, 100U) <= (curve & AXP2202_CURVE_SOC)) {
		dev_dbg(bat_power->dev, "SOC smoothing done\n");
		regmap_write(regmap, AXP2202_CURVE_CHECK, 0);
		__pm_relax(bat_power->curve_ws);
		return;
	}

	if (!regmap_read(regmap, AXP2202_COMM_STAT0, &stat0) &&
	    (stat0 & AXP2202_VBUS_GOOD))
		regmap_update_bits(regmap, AXP2202_CURVE_CHECK,
				   AXP2202_CURVE_SOC,
				   (curve & AXP2202_CURVE_SOC) + 1);

	schedule_delayed_work(&bat_power->curve_work, AXP2202_SMOOTH_INTERVAL);
}

static int axp2202_battery_probe(struct platform_device *pdev)
{
	struct axp20x_dev *axp_dev = dev_get_drvdata(pdev->dev.parent);
	struct power_supply_config psy_cfg = {};
	struct axp2202_bat_power *bat_power;
	unsigned int curve;
	int i, irq, ret;

	/* The MFD core creates this device even if its DT node is disabled. */
	if (!of_device_is_available(pdev->dev.of_node))
		return -ENODEV;

	if (!axp_dev->irq) {
		dev_err(&pdev->dev, "PMIC has no IRQ\n");
		return -EINVAL;
	}

	bat_power = devm_kzalloc(&pdev->dev, sizeof(*bat_power), GFP_KERNEL);
	if (!bat_power)
		return -ENOMEM;

	bat_power->dev = &pdev->dev;
	bat_power->regmap = axp_dev->regmap;
	platform_set_drvdata(pdev, bat_power);
	INIT_DELAYED_WORK(&bat_power->monitor_work, axp2202_monitor_work);
	INIT_DELAYED_WORK(&bat_power->curve_work, axp2202_curve_work);

	axp2202_bat_parse_dt(bat_power);

	ret = axp2202_init_chip(bat_power);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to initialise charger: %d\n", ret);
		return ret;
	}

	psy_cfg.of_node = pdev->dev.of_node;
	psy_cfg.drv_data = bat_power;
	bat_power->bat_supply = devm_power_supply_register(&pdev->dev,
					&axp2202_bat_desc, &psy_cfg);
	if (IS_ERR(bat_power->bat_supply)) {
		dev_err(&pdev->dev, "failed to register power supply\n");
		return PTR_ERR(bat_power->bat_supply);
	}

	for (i = 0; i < ARRAY_SIZE(axp2202_bat_irqs); i++) {
		irq = platform_get_irq_byname(pdev, axp2202_bat_irqs[i].name);
		if (irq < 0)
			continue;

		irq = regmap_irq_get_virq(axp_dev->regmap_irqc, irq);
		if (irq < 0)
			return irq;

		ret = devm_request_any_context_irq(&pdev->dev, irq,
						   axp2202_bat_irq, 0,
						   axp2202_bat_irqs[i].name,
						   bat_power);
		if (ret < 0) {
			dev_err(&pdev->dev, "failed to request %s IRQ: %d\n",
				axp2202_bat_irqs[i].name, ret);
			return ret;
		}
		if (!of_property_read_bool(pdev->dev.of_node,
					   axp2202_bat_irqs[i].wakeup_prop))
			bat_power->sleep_irqs[i] = irq;
	}

	/* smoothing state survives reboots; resume it if active */
	ret = regmap_read(bat_power->regmap, AXP2202_CURVE_CHECK, &curve);
	if (ret < 0)
		return ret;
	if (curve & AXP2202_CURVE_ACTIVE) {
		bat_power->curve_ws =
			wakeup_source_register("bat_curve_smooth");
		__pm_stay_awake(bat_power->curve_ws);
		schedule_delayed_work(&bat_power->curve_work,
				      AXP2202_SMOOTH_INTERVAL);
	}

	schedule_delayed_work(&bat_power->monitor_work, msecs_to_jiffies(500));

	return 0;
}

static int axp2202_battery_remove(struct platform_device *pdev)
{
	struct axp2202_bat_power *bat_power = platform_get_drvdata(pdev);

	cancel_delayed_work_sync(&bat_power->monitor_work);
	cancel_delayed_work_sync(&bat_power->curve_work);
	wakeup_source_unregister(bat_power->curve_ws);

	return 0;
}

/* Sets the charge current used after power-off; the PMIC keeps charging. */
static void axp2202_battery_shutdown(struct platform_device *pdev)
{
	struct axp2202_bat_power *bat_power = platform_get_drvdata(pdev);

	cancel_delayed_work_sync(&bat_power->monitor_work);
	cancel_delayed_work_sync(&bat_power->curve_work);
	axp2202_set_ichg(bat_power, bat_power->cfg.pmu_shutdown_chgcur);
}

#ifdef CONFIG_PM_SLEEP
static void axp2202_bat_set_irqs(struct axp2202_bat_power *bat_power,
				 bool enable)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(axp2202_bat_irqs); i++) {
		if (!bat_power->sleep_irqs[i])
			continue;
		if (enable)
			enable_irq(bat_power->sleep_irqs[i]);
		else
			disable_irq(bat_power->sleep_irqs[i]);
	}
}

static int axp2202_battery_suspend(struct device *dev)
{
	struct axp2202_bat_power *bat_power = dev_get_drvdata(dev);

	cancel_delayed_work_sync(&bat_power->monitor_work);
	cancel_delayed_work_sync(&bat_power->curve_work);
	axp2202_set_ichg(bat_power, bat_power->cfg.pmu_suspend_chgcur);
	axp2202_bat_set_irqs(bat_power, false);

	return 0;
}

static int axp2202_battery_resume(struct device *dev)
{
	struct axp2202_bat_power *bat_power = dev_get_drvdata(dev);
	unsigned int curve;

	axp2202_bat_set_irqs(bat_power, true);
	axp2202_set_ichg(bat_power, bat_power->cfg.pmu_runtime_chgcur);
	schedule_delayed_work(&bat_power->monitor_work, 0);
	if (bat_power->curve_ws &&
	    !regmap_read(bat_power->regmap, AXP2202_CURVE_CHECK, &curve) &&
	    (curve & AXP2202_CURVE_ACTIVE))
		schedule_delayed_work(&bat_power->curve_work, 0);

	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(axp2202_battery_pm_ops, axp2202_battery_suspend,
			 axp2202_battery_resume);

static const struct of_device_id axp2202_bat_power_match[] = {
	{ .compatible = "x-powers,axp2202-bat-power-supply" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, axp2202_bat_power_match);

static struct platform_driver axp2202_bat_power_driver = {
	.driver = {
		.name = "axp2202-bat-power-supply",
		.of_match_table = axp2202_bat_power_match,
		.pm = &axp2202_battery_pm_ops,
	},
	.probe = axp2202_battery_probe,
	.remove = axp2202_battery_remove,
	.shutdown = axp2202_battery_shutdown,
};
module_platform_driver(axp2202_bat_power_driver);

MODULE_AUTHOR("wangxiaoliang <wangxiaoliang@x-powers.com>");
MODULE_DESCRIPTION("AXP2202 battery driver");
MODULE_LICENSE("GPL");
