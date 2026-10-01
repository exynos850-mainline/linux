// SPDX-License-Identifier: GPL-2.0-only
/*
 * Power key driver for the Samsung S2MPU12 PMIC
 */

#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/pm_wakeirq.h>
#include <linux/pm_wakeup.h>

struct s2mpu12_pwrkey {
	struct input_dev *input;
	int irq_press;
};

static irqreturn_t s2mpu12_pwrkey_irq(int irq, void *data)
{
	struct s2mpu12_pwrkey *pk = data;

	input_report_key(pk->input, KEY_POWER, irq == pk->irq_press);
	input_sync(pk->input);

	return IRQ_HANDLED;
}

static int s2mpu12_pwrkey_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct s2mpu12_pwrkey *pk;
	int irq_release, ret;

	pk = devm_kzalloc(dev, sizeof(*pk), GFP_KERNEL);
	if (!pk)
		return -ENOMEM;

	pk->irq_press = platform_get_irq_byname(pdev, "pwronr");
	if (pk->irq_press < 0)
		return pk->irq_press;

	irq_release = platform_get_irq_byname(pdev, "pwronf");
	if (irq_release < 0)
		return irq_release;

	pk->input = devm_input_allocate_device(dev);
	if (!pk->input)
		return -ENOMEM;

	pk->input->name = "s2mpu12-pwrkey";
	pk->input->phys = "s2mpu12-pwrkey/input0";
	input_set_capability(pk->input, EV_KEY, KEY_POWER);

	ret = input_register_device(pk->input);
	if (ret)
		return dev_err_probe(dev, ret, "failed to register input device\n");

	ret = devm_request_threaded_irq(dev, pk->irq_press, NULL,
					s2mpu12_pwrkey_irq, 0,
					"pwrkey-press", pk);
	if (ret)
		return dev_err_probe(dev, ret, "failed to request press IRQ\n");

	ret = devm_request_threaded_irq(dev, irq_release, NULL,
					s2mpu12_pwrkey_irq, 0,
					"pwrkey-release", pk);
	if (ret)
		return dev_err_probe(dev, ret, "failed to request release IRQ\n");

	/* A key press wakes the system from suspend */
	ret = devm_device_init_wakeup(dev);
	if (ret)
		return dev_err_probe(dev, ret, "failed to init wakeup\n");

	return devm_pm_set_wake_irq(dev, pk->irq_press);
}

static const struct platform_device_id s2mpu12_pwrkey_id[] = {
	{ .name = "s2mpu12-power-keys" },
	{ }
};
MODULE_DEVICE_TABLE(platform, s2mpu12_pwrkey_id);

static struct platform_driver s2mpu12_pwrkey_driver = {
	.driver = {
		.name = "s2mpu12-power-keys",
	},
	.probe = s2mpu12_pwrkey_probe,
	.id_table = s2mpu12_pwrkey_id,
};
module_platform_driver(s2mpu12_pwrkey_driver);

MODULE_DESCRIPTION("Samsung S2MPU12 PMIC power key driver");
MODULE_LICENSE("GPL");
