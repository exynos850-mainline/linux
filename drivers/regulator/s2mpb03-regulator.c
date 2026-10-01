// SPDX-License-Identifier: GPL-2.0+
/*
 * s2mpb03.c - Regulator driver for the Samsung s2mpb03
 *
 * Copyright (C) 2016 Samsung Electronics
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include <linux/bug.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/regmap.h>
#include <linux/platform_device.h>
#include <linux/unaligned.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/of_regulator.h>
#include <linux/mfd/samsung/core.h>
#include <linux/regulator/s2mpb03.h>

struct s2mpb03_data {
	struct regmap *regmap;
	struct device *dev;
};

int s2mpb03_read_reg(struct regmap *regmap, u8 reg, u8 *dest)
{
	unsigned int val;
	int ret;

	ret = regmap_read(regmap, reg, &val);
	if (ret < 0) {
		dev_info(regmap_get_device(regmap), "%s reg(0x%02hhx), ret(%d)\n",
			 __func__, reg, ret);
		return ret;
	}

	*dest = val & 0xff;
	return 0;
}
EXPORT_SYMBOL_GPL(s2mpb03_read_reg);

int s2mpb03_bulk_read(struct regmap *regmap, u8 reg, int count, u8 *buf)
{
	return regmap_bulk_read(regmap, reg, buf, count);
}
EXPORT_SYMBOL_GPL(s2mpb03_bulk_read);

int s2mpb03_read_word(struct regmap *regmap, u8 reg)
{
	u8 buf[2];
	int ret;

	ret = regmap_bulk_read(regmap, reg, buf, sizeof(buf));
	if (ret < 0)
		return ret;

	return get_unaligned_le16(buf);
}
EXPORT_SYMBOL_GPL(s2mpb03_read_word);

int s2mpb03_write_reg(struct regmap *regmap, u8 reg, u8 value)
{
	int ret;

	ret = regmap_write(regmap, reg, value);
	if (ret < 0)
		dev_info(regmap_get_device(regmap), "%s reg(0x%02hhx), ret(%d)\n",
			 __func__, reg, ret);

	return ret;
}
EXPORT_SYMBOL_GPL(s2mpb03_write_reg);

int s2mpb03_bulk_write(struct regmap *regmap, u8 reg, int count, u8 *buf)
{
	return regmap_bulk_write(regmap, reg, buf, count);
}
EXPORT_SYMBOL_GPL(s2mpb03_bulk_write);

int s2mpb03_update_reg(struct regmap *regmap, u8 reg, u8 val, u8 mask)
{
	return regmap_update_bits(regmap, reg, mask, val & mask);
}
EXPORT_SYMBOL_GPL(s2mpb03_update_reg);

static int s2m_enable(struct regulator_dev *rdev)
{
	struct s2mpb03_data *info = rdev_get_drvdata(rdev);
	struct regmap *regmap = info->regmap;

	return s2mpb03_update_reg(regmap, rdev->desc->enable_reg,
					rdev->desc->enable_mask,
					rdev->desc->enable_mask);
}

static int s2m_disable_regmap(struct regulator_dev *rdev)
{
	struct s2mpb03_data *info = rdev_get_drvdata(rdev);
	struct regmap *regmap = info->regmap;
	u8 val;

	if (rdev->desc->enable_is_inverted)
		val = rdev->desc->enable_mask;
	else
		val = 0;

	return s2mpb03_update_reg(regmap, rdev->desc->enable_reg,
				   val, rdev->desc->enable_mask);
}

static int s2m_is_enabled_regmap(struct regulator_dev *rdev)
{
	struct s2mpb03_data *info = rdev_get_drvdata(rdev);
	struct regmap *regmap = info->regmap;
	int ret;
	u8 val;

	ret = s2mpb03_read_reg(regmap, rdev->desc->enable_reg, &val);
	if (ret < 0)
		return ret;

	if (rdev->desc->enable_is_inverted)
		return (val & rdev->desc->enable_mask) == 0;
	else
		return (val & rdev->desc->enable_mask) != 0;
}

static int s2m_get_voltage_sel_regmap(struct regulator_dev *rdev)
{
	struct s2mpb03_data *info = rdev_get_drvdata(rdev);
	struct regmap *regmap = info->regmap;
	int ret;
	u8 val;

	ret = s2mpb03_read_reg(regmap, rdev->desc->vsel_reg, &val);
	if (ret < 0)
		return ret;

	val &= rdev->desc->vsel_mask;

	return val;
}

static int s2m_set_voltage_sel_regmap(struct regulator_dev *rdev, unsigned sel)
{
	struct s2mpb03_data *info = rdev_get_drvdata(rdev);
	struct regmap *regmap = info->regmap;
	int ret;

	ret = s2mpb03_update_reg(regmap, rdev->desc->vsel_reg,
				sel, rdev->desc->vsel_mask);
	if (ret < 0)
		goto out;

	if (rdev->desc->apply_bit)
		ret = s2mpb03_update_reg(regmap, rdev->desc->apply_reg,
					 rdev->desc->apply_bit,
					 rdev->desc->apply_bit);
	return ret;
out:
	pr_warn("%s: failed to set voltage_sel_regmap\n", rdev->desc->name);
	return ret;
}

static int s2m_set_voltage_time_sel(struct regulator_dev *rdev,
				   unsigned int old_selector,
				   unsigned int new_selector)
{
	int old_volt, new_volt;

	/* sanity check */
	if (!rdev->desc->ops->list_voltage)
		return -EINVAL;

	old_volt = rdev->desc->ops->list_voltage(rdev, old_selector);
	new_volt = rdev->desc->ops->list_voltage(rdev, new_selector);

	if (old_selector < new_selector)
		return DIV_ROUND_UP(new_volt - old_volt, S2MPB03_RAMP_DELAY);

	return 0;
}

static const struct regulator_ops s2mpb03_ldo_ops = {
	.list_voltage		= regulator_list_voltage_linear,
	.map_voltage		= regulator_map_voltage_linear,
	.is_enabled		= s2m_is_enabled_regmap,
	.enable			= s2m_enable,
	.disable		= s2m_disable_regmap,
	.get_voltage_sel	= s2m_get_voltage_sel_regmap,
	.set_voltage_sel	= s2m_set_voltage_sel_regmap,
	.set_voltage_time_sel	= s2m_set_voltage_time_sel,
};

#define _LDO(macro)	S2MPB03_LDO##macro
#define _REG(ctrl)	S2MPB03_REG##ctrl
#define _ldo_ops(num)	s2mpb03_ldo_ops##num
#define _TIME(macro)	S2MPB03_ENABLE_TIME##macro

#define LDO_DESC(_name, _id, _ops, m, s, v, e, t)	{	\
	.name		= _name,				\
	.id		= _id,					\
	.ops		= _ops,					\
	.of_match	= of_match_ptr(_name),			\
	.of_match_full_name = true,				\
	.regulators_node = of_match_ptr("regulators"),		\
	.type		= REGULATOR_VOLTAGE,			\
	.owner		= THIS_MODULE,				\
	.min_uV		= m,					\
	.uV_step	= s,					\
	.n_voltages	= S2MPB03_LDO_N_VOLTAGES,		\
	.vsel_reg	= v,					\
	.vsel_mask	= S2MPB03_LDO_VSEL_MASK,		\
	.enable_reg	= e,					\
	.enable_mask	= S2MPB03_LDO_ENABLE_MASK,		\
	.enable_time	= t					\
}

static const struct regulator_desc regulators[S2MPB03_REGULATOR_MAX] = {
	/* name, id, ops, min_uv, uV_step, vsel_reg, enable_reg */
	LDO_DESC("s2mpb03-ldo1", _LDO(1), &_ldo_ops(), _LDO(_MIN1),
		_LDO(_STEP2), _REG(_LDO1_CTRL),
		_REG(_LDO1_CTRL), _TIME(_LDO)),
	LDO_DESC("s2mpb03-ldo2", _LDO(2), &_ldo_ops(), _LDO(_MIN1),
		_LDO(_STEP2), _REG(_LDO2_CTRL),
		_REG(_LDO2_CTRL),  _TIME(_LDO)),
	LDO_DESC("s2mpb03-ldo3", _LDO(3), &_ldo_ops(), _LDO(_MIN1),
		_LDO(_STEP1), _REG(_LDO3_CTRL),
		_REG(_LDO3_CTRL), _TIME(_LDO)),
	LDO_DESC("s2mpb03-ldo4", _LDO(4), &_ldo_ops(), _LDO(_MIN1),
		_LDO(_STEP2), _REG(_LDO4_CTRL),
		_REG(_LDO4_CTRL), _TIME(_LDO)),
	LDO_DESC("s2mpb03-ldo5", _LDO(5), &_ldo_ops(), _LDO(_MIN2),
		_LDO(_STEP1), _REG(_LDO5_CTRL),
		_REG(_LDO5_CTRL), _TIME(_LDO)),
	LDO_DESC("s2mpb03-ldo6", _LDO(6), &_ldo_ops(), _LDO(_MIN2),
		_LDO(_STEP1), _REG(_LDO6_CTRL),
		_REG(_LDO6_CTRL), _TIME(_LDO)),
	LDO_DESC("s2mpb03-ldo7", _LDO(7), &_ldo_ops(), _LDO(_MIN2),
		_LDO(_STEP1), _REG(_LDO7_CTRL),
		_REG(_LDO7_CTRL), _TIME(_LDO))
};

static int s2mpb03_pmic_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct sec_pmic_dev *iodev = dev_get_drvdata(pdev->dev.parent);
	struct s2mpb03_data *s2mpb03;
	struct regulator_config config = { };
	unsigned int rdev_num = ARRAY_SIZE(regulators);

	s2mpb03 = devm_kzalloc(dev, sizeof(*s2mpb03), GFP_KERNEL);
	if (!s2mpb03)
		return -ENOMEM;

	platform_set_drvdata(pdev, s2mpb03);

	s2mpb03->regmap = iodev->regmap_pmic;
	s2mpb03->dev = dev;
	if (!dev->of_node)
		device_set_of_node_from_dev(dev, dev->parent);

	config.dev = dev;
	config.driver_data = s2mpb03;

	for (int i = 0; i < rdev_num; i++) {
		struct regulator_dev *regulator;

		regulator = devm_regulator_register(&pdev->dev,
						&regulators[i], &config);
		if (IS_ERR(regulator)) {
			return dev_err_probe(&pdev->dev, PTR_ERR(regulator),
					"regulator init failed for %d\n", i);
		}
	}

	return 0;
}

static const struct platform_device_id s2mpb03_pmic_id[] = {
	{ .name = "s2mpb03-regulator" },
	{ }
};
MODULE_DEVICE_TABLE(platform, s2mpb03_pmic_id);

static struct platform_driver s2mpb03_platform_driver = {
	.driver = {
		.name = "s2mpb03",
	},
	.probe = s2mpb03_pmic_probe,
	.id_table = s2mpb03_pmic_id,
};
module_platform_driver(s2mpb03_platform_driver);

MODULE_DESCRIPTION("SAMSUNG S2MPB03 Regulator Driver");
MODULE_LICENSE("GPL");
