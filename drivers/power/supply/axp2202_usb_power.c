// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * X-Powers AXP2202 USB (VBUS) power supply driver.
 *
 * Also reports the USB cable state as extcon cables, for the USB port's role switching:
 * EXTCON_USB while powered by a computer or charger, EXTCON_USB_HOST while a USB device that
 * needs power is attached (from the chip's USB-C detection), and the charger type found by the
 * chip's BC1.2 detection (EXTCON_CHG_USB_*), which also sets the input current limit itself.
 *
 * Based on Allwinner's BSP driver (github.com/radxa/allwinner-bsp).
 * Copyright(c) 2020 - 2023 Allwinner Technology Co.,Ltd. All rights reserved.
 */

#include <linux/bitfield.h>
#include <linux/delay.h>
#include <linux/extcon.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/mfd/axp2101.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/regmap.h>

/* IRQ resource name, and the DT flag that keeps it enabled during suspend */
static const struct {
	const char *name;
	const char *wakeup_prop;
} axp2202_usb_irqs[] = {
	{ "vbus_insert", "wakeup_usb_in" },
	{ "vbus_remove", "wakeup_usb_out" },
	{ "cable_insert", "wakeup_typec_in" },	/* USB-C attach and detach */
	{ "cable_remove", "wakeup_typec_out" },
	{ "bc_done", "wakeup_bc_done" },	/* BC1.2 charger detection finished */
};

static const unsigned int axp2202_usb_cables[] = {
	EXTCON_USB,
	EXTCON_USB_HOST,
	EXTCON_CHG_USB_SDP,
	EXTCON_CHG_USB_CDP,
	EXTCON_CHG_USB_DCP,
	EXTCON_NONE,
};

/* USB-C state (AXP2202_CC_STAT0 bits 3:0), the standard USB-C state machine's attached states */
#define AXP2202_CC_STATE		GENMASK(3, 0)
#define AXP2202_CC_ATTACHED_SINK	3	/* powered by the other end */
#define AXP2202_CC_ATTACHED_SOURCE	6	/* the other end needs power */
/* BC1.2 result (AXP2202_BC_DECT bits 7:5): 1 SDP, 2 CDP, 3 DCP, as mainline's axp20x drivers */
#define AXP2202_BC_RESULT		GENMASK(7, 5)

struct axp2202_usb_power {
	struct regmap *regmap;
	struct power_supply *usb_supply;
	struct extcon_dev *edev;
	/* virqs to disable during suspend, 0 for the others */
	int sleep_irqs[ARRAY_SIZE(axp2202_usb_irqs)];
};

static enum power_supply_property axp2202_usb_props[] = {
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_MIN,
	POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_MANUFACTURER,
};

/*
 * Returns the PMIC die temperature in 0.1 degC; there is no USB-specific
 * sensor. The conversion T = (753 - raw) / 1.73 degC comes from the vendor.
 */
static int axp2202_get_die_temp(struct regmap *regmap, int *temp)
{
	u8 data[2];
	int i, ret, prev;

	ret = regmap_update_bits(regmap, AXP2202_ADC_DATA_SEL, AXP2202_ADC_SEL,
				 AXP2202_ADC_SEL_TDIE);
	if (ret < 0)
		return ret;
	usleep_range(1000, 2000);	/* let the ADC settle on the channel */

	/* Re-read until two samples agree within 10 degC, at most 11 reads. */
	for (i = 0; i <= 10; i++) {
		ret = regmap_bulk_read(regmap, AXP2202_ADC_DATA_H, data, 2);
		if (ret < 0)
			return ret;

		prev = *temp;
		*temp = (753 - ((data[0] << 8) | data[1])) * 1000 / 173;
		if (i > 0 && abs(*temp - prev) <= 100)
			break;
	}

	return 0;
}

static int axp2202_usb_get_property(struct power_supply *psy,
				    enum power_supply_property psp,
				    union power_supply_propval *val)
{
	struct axp2202_usb_power *usb_power = power_supply_get_drvdata(psy);
	struct regmap *regmap = usb_power->regmap;
	unsigned int data;
	u8 vbus[2];
	int ret = 0;

	switch (psp) {
	case POWER_SUPPLY_PROP_PRESENT:
	case POWER_SUPPLY_PROP_ONLINE:
		ret = regmap_read(regmap, AXP2202_COMM_STAT0, &data);
		val->intval = !!(data & AXP2202_VBUS_GOOD);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		/* 1 mV per LSB */
		ret = regmap_bulk_read(regmap, AXP2202_VBUS_H, vbus, 2);
		val->intval = (((vbus[0] & GENMASK(5, 0)) << 8) | vbus[1]) *
			      1000;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MIN:
		/* input voltage regulation (VINDPM) threshold */
		ret = regmap_read(regmap, AXP2202_VINDPM_CFG, &data);
		val->intval = ((data & GENMASK(3, 0)) * 80 + 3880) * 1000;
		break;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		ret = regmap_read(regmap, AXP2202_IIN_LIM, &data);
		val->intval = ((data & GENMASK(5, 0)) * 50 + 100) * 1000;
		break;
	case POWER_SUPPLY_PROP_TEMP:
		ret = axp2202_get_die_temp(regmap, &val->intval);
		break;
	case POWER_SUPPLY_PROP_MANUFACTURER:
		val->strval = "xpower,axp2202";
		break;
	default:
		return -EINVAL;
	}

	return ret;
}

static const struct power_supply_desc axp2202_usb_desc = {
	.name = "axp2202-usb",
	.type = POWER_SUPPLY_TYPE_USB,
	.get_property = axp2202_usb_get_property,
	.properties = axp2202_usb_props,
	.num_properties = ARRAY_SIZE(axp2202_usb_props),
};

/* Read the cable state from the chip and report changes. */
static void axp2202_usb_update(struct axp2202_usb_power *usb_power)
{
	static const unsigned int chargers[] = {
		EXTCON_CHG_USB_SDP, EXTCON_CHG_USB_CDP, EXTCON_CHG_USB_DCP,
	};
	unsigned int stat, cc, bc;
	bool vbus, host;
	int i;

	if (regmap_read(usb_power->regmap, AXP2202_COMM_STAT0, &stat) ||
	    regmap_read(usb_power->regmap, AXP2202_CC_STAT0, &cc) ||
	    regmap_read(usb_power->regmap, AXP2202_BC_DECT, &bc))
		return;
	host = (cc & AXP2202_CC_STATE) == AXP2202_CC_ATTACHED_SOURCE;
	/* while host, VBUS is our own output */
	vbus = (stat & AXP2202_VBUS_GOOD) && !host;
	bc = vbus ? FIELD_GET(AXP2202_BC_RESULT, bc) : 0;

	extcon_set_state_sync(usb_power->edev, EXTCON_USB_HOST, host);
	extcon_set_state_sync(usb_power->edev, EXTCON_USB, vbus);
	for (i = 0; i < ARRAY_SIZE(chargers); i++)
		extcon_set_state_sync(usb_power->edev, chargers[i], bc == i + 1);
	power_supply_changed(usb_power->usb_supply);
}

static irqreturn_t axp2202_usb_irq(int irq, void *data)
{
	/*
	 * Runs in a thread (nested regmap-irq), so it may sleep. Let VBUS
	 * settle before reporting the change.
	 */
	msleep(20);
	axp2202_usb_update(data);

	return IRQ_HANDLED;
}

static void axp2202_usb_init(struct axp2202_usb_power *usb_power,
			     struct device_node *np)
{
	struct regmap *regmap = usb_power->regmap;
	u32 vindpm_mv, bc12_en, boost_mv;

	/* VBUS voltage below which the PMIC reduces input current (VINDPM) */
	if (of_property_read_u32(np, "pmu_usbad_vol", &vindpm_mv))
		vindpm_mv = 4600;
	if (of_property_read_u32(np, "pmu_bc12_en", &bc12_en))
		bc12_en = 0;
	if (of_property_read_u32(np, "pmu_boost_vol", &boost_mv))
		boost_mv = 5126;

	/* the VBUS voltage reading (AXP2202_VBUS_H) needs its ADC channel */
	regmap_update_bits(regmap, AXP2202_ADC_CH_EN0, BIT(2), BIT(2));

	vindpm_mv = clamp(vindpm_mv, 3880U, 5080U);
	regmap_update_bits(regmap, AXP2202_VINDPM_CFG, GENMASK(3, 0),
			   (vindpm_mv - 3880) / 80);

	/*
	 * BC1.2 charger detection (AXP2202_CLK_EN bit 4); bit 7 of
	 * AXP2202_IIN_LIM is set whenever detection is off.
	 */
	if (bc12_en) {
		regmap_update_bits(regmap, AXP2202_CLK_EN, BIT(4), BIT(4));
		regmap_update_bits(regmap, AXP2202_IIN_LIM, BIT(7), 0);
	} else {
		regmap_update_bits(regmap, AXP2202_CLK_EN, BIT(4), 0);
		regmap_update_bits(regmap, AXP2202_IIN_LIM, BIT(7), BIT(7));
	}

	/* OTG boost output voltage: 4550 mV + 64 mV steps in bits 7:4 */
	boost_mv = clamp(boost_mv, 4550U, 4550U + 15 * 64);
	regmap_update_bits(regmap, AXP2202_BST_CFG0, GENMASK(7, 4),
			   ((boost_mv - 4550) / 64) << 4);
	/* vendor value, meaning undocumented */
	regmap_write(regmap, AXP2202_BST_CFG1, 0x03);
}

static int axp2202_usb_probe(struct platform_device *pdev)
{
	struct axp20x_dev *axp_dev = dev_get_drvdata(pdev->dev.parent);
	struct device_node *np = pdev->dev.of_node;
	struct power_supply_config psy_cfg = {};
	struct axp2202_usb_power *usb_power;
	int i, irq, ret;

	/* The MFD core creates this device even if its DT node is disabled. */
	if (!of_device_is_available(np))
		return -ENODEV;

	if (!axp_dev->irq) {
		dev_err(&pdev->dev, "PMIC has no IRQ\n");
		return -EINVAL;
	}

	usb_power = devm_kzalloc(&pdev->dev, sizeof(*usb_power), GFP_KERNEL);
	if (!usb_power)
		return -ENOMEM;

	usb_power->regmap = axp_dev->regmap;
	platform_set_drvdata(pdev, usb_power);

	axp2202_usb_init(usb_power, np);

	psy_cfg.of_node = np;
	psy_cfg.drv_data = usb_power;
	usb_power->usb_supply = devm_power_supply_register(&pdev->dev,
					&axp2202_usb_desc, &psy_cfg);
	if (IS_ERR(usb_power->usb_supply)) {
		dev_err(&pdev->dev, "failed to register power supply\n");
		return PTR_ERR(usb_power->usb_supply);
	}

	usb_power->edev = devm_extcon_dev_allocate(&pdev->dev, axp2202_usb_cables);
	if (IS_ERR(usb_power->edev))
		return PTR_ERR(usb_power->edev);
	ret = devm_extcon_dev_register(&pdev->dev, usb_power->edev);
	if (ret)
		return ret;
	axp2202_usb_update(usb_power);

	for (i = 0; i < ARRAY_SIZE(axp2202_usb_irqs); i++) {
		irq = platform_get_irq_byname(pdev, axp2202_usb_irqs[i].name);
		if (irq < 0)
			continue;

		irq = regmap_irq_get_virq(axp_dev->regmap_irqc, irq);
		if (irq < 0)
			return irq;

		ret = devm_request_any_context_irq(&pdev->dev, irq,
						   axp2202_usb_irq, 0,
						   axp2202_usb_irqs[i].name,
						   usb_power);
		if (ret < 0) {
			dev_err(&pdev->dev, "failed to request %s IRQ: %d\n",
				axp2202_usb_irqs[i].name, ret);
			return ret;
		}
		if (!of_property_read_bool(np, axp2202_usb_irqs[i].wakeup_prop))
			usb_power->sleep_irqs[i] = irq;
	}

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static void axp2202_usb_set_irqs(struct axp2202_usb_power *usb_power,
				 bool enable)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(axp2202_usb_irqs); i++) {
		if (!usb_power->sleep_irqs[i])
			continue;
		if (enable)
			enable_irq(usb_power->sleep_irqs[i]);
		else
			disable_irq(usb_power->sleep_irqs[i]);
	}
}

static int axp2202_usb_suspend(struct device *dev)
{
	axp2202_usb_set_irqs(dev_get_drvdata(dev), false);
	return 0;
}

static int axp2202_usb_resume(struct device *dev)
{
	struct axp2202_usb_power *usb_power = dev_get_drvdata(dev);

	axp2202_usb_set_irqs(usb_power, true);
	axp2202_usb_update(usb_power);	/* cables may have changed while asleep */
	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(axp2202_usb_pm_ops, axp2202_usb_suspend,
			 axp2202_usb_resume);

static const struct of_device_id axp2202_usb_power_match[] = {
	{ .compatible = "x-powers,axp2202-usb-power-supply" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, axp2202_usb_power_match);

static struct platform_driver axp2202_usb_power_driver = {
	.driver = {
		.name = "axp2202-usb-power-supply",
		.of_match_table = axp2202_usb_power_match,
		.pm = &axp2202_usb_pm_ops,
	},
	.probe = axp2202_usb_probe,
};
module_platform_driver(axp2202_usb_power_driver);

MODULE_AUTHOR("wangxiaoliang <wangxiaoliang@x-powers.com>");
MODULE_DESCRIPTION("AXP2202 USB power supply driver");
MODULE_LICENSE("GPL");
