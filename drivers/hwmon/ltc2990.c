/*
 * Driver for Linear Technology LTC2990 power monitor
 *
 * Copyright (C) 2014 Topic Embedded Products
 * Author: Mike Looijmans <mike.looijmans@topic.nl>
 *
 * License: GPLv2
 *
 * Value conversion refactored and support for all measurement modes added
 * by Tom Levens <tom.levens@cern.ch>
 */

#include <linux/bitops.h>
#include <linux/err.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>

#define LTC2990_STATUS	0x00
#define LTC2990_CONTROL	0x01
#define LTC2990_TRIGGER	0x02
#define LTC2990_TINT_MSB	0x04
#define LTC2990_V1_MSB	0x06
#define LTC2990_V2_MSB	0x08
#define LTC2990_V3_MSB	0x0A
#define LTC2990_V4_MSB	0x0C
#define LTC2990_VCC_MSB	0x0E

#define LTC2990_CONTROL_KELVIN		BIT(7)
#define LTC2990_CONTROL_SINGLE		BIT(6)
#define LTC2990_CONTROL_MEASURE_ALL	(0x3 << 3)
#define LTC2990_CONTROL_MODE_DEFAULT	0x06
#define LTC2990_CONTROL_MODE_MAX	0x07

#define LTC2990_IN0	BIT(0)
#define LTC2990_IN1	BIT(1)
#define LTC2990_IN2	BIT(2)
#define LTC2990_IN3	BIT(3)
#define LTC2990_IN4	BIT(4)
#define LTC2990_CURR1	BIT(5)
#define LTC2990_CURR2	BIT(6)
#define LTC2990_TEMP1	BIT(7)
#define LTC2990_TEMP2	BIT(8)
#define LTC2990_TEMP3	BIT(9)

static const int ltc2990_attrs_ena[] = {
	LTC2990_IN1 | LTC2990_IN2 | LTC2990_TEMP3,
	LTC2990_CURR1 | LTC2990_TEMP3,
	LTC2990_CURR1 | LTC2990_IN3 | LTC2990_IN4,
	LTC2990_TEMP2 | LTC2990_IN3 | LTC2990_IN4,
	LTC2990_TEMP2 | LTC2990_CURR2,
	LTC2990_TEMP2 | LTC2990_TEMP3,
	LTC2990_CURR1 | LTC2990_CURR2,
	LTC2990_IN1 | LTC2990_IN2 | LTC2990_IN3 | LTC2990_IN4
};

struct ltc2990_data {
	struct i2c_client *i2c;
	u32 mode;
};

/* Return the converted value from the given register in uV or mC */
static int ltc2990_get_value(struct i2c_client *i2c, int index, s32 *result)
{
	s32 val;
	u8 reg;

	switch (index) {
	case LTC2990_IN0:
		reg = LTC2990_VCC_MSB;
		break;
	case LTC2990_IN1:
	case LTC2990_CURR1:
	case LTC2990_TEMP2:
		reg = LTC2990_V1_MSB;
		break;
	case LTC2990_IN2:
		reg = LTC2990_V2_MSB;
		break;
	case LTC2990_IN3:
	case LTC2990_CURR2:
	case LTC2990_TEMP3:
		reg = LTC2990_V3_MSB;
		break;
	case LTC2990_IN4:
		reg = LTC2990_V4_MSB;
		break;
	case LTC2990_TEMP1:
		reg = LTC2990_TINT_MSB;
		break;
	default:
		return -EINVAL;
	}

	val = i2c_smbus_read_word_swapped(i2c, reg);
	if (unlikely(val < 0))
		return val;

	switch (index) {
	case LTC2990_TEMP1:
	case LTC2990_TEMP2:
	case LTC2990_TEMP3:
		/* temp, 0.0625 degrees/LSB, 13-bit  */
		*result = sign_extend32(val, 12) * 1000 / 16;
		break;
	case LTC2990_CURR1:
	case LTC2990_CURR2:
		 /* Vx-Vy, 19.42uV/LSB */
		*result = sign_extend32(val, 14) * 1942 / 100;
		break;
	case LTC2990_IN0:
		/* Vcc, 305.18uV/LSB, 2.5V offset */
		*result = sign_extend32(val, 14) * 30518 / (100 * 1000) + 2500;
		break;
	case LTC2990_IN1:
	case LTC2990_IN2:
	case LTC2990_IN3:
	case LTC2990_IN4:
		/* Vx: 305.18uV/LSB */
		*result = sign_extend32(val, 14) * 30518 / (100 * 1000);
		break;
	default:
		return -EINVAL; /* won't happen, keep compiler happy */
	}

	return 0;
}

static ssize_t ltc2990_show_value(struct device *dev,
				  struct device_attribute *da, char *buf)
{
	struct sensor_device_attribute *attr = to_sensor_dev_attr(da);
	struct ltc2990_data *data = dev_get_drvdata(dev);
	s32 value;
	int ret;

	ret = ltc2990_get_value(data->i2c, attr->index, &value);
	if (unlikely(ret < 0))
		return ret;

	return snprintf(buf, PAGE_SIZE, "%d\n", value);
}

static umode_t ltc2990_attrs_visible(struct kobject *kobj,
				     struct attribute *a, int n)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct ltc2990_data *data = dev_get_drvdata(dev);
	struct device_attribute *da =
			container_of(a, struct device_attribute, attr);
	struct sensor_device_attribute *attr = to_sensor_dev_attr(da);

	if (attr->index == LTC2990_TEMP1 ||
	    attr->index == LTC2990_IN0 ||
	    attr->index & ltc2990_attrs_ena[data->mode])
		return a->mode;
	else
		return 0;
}

static SENSOR_DEVICE_ATTR(temp1_input, S_IRUGO, ltc2990_show_value, NULL,
			  LTC2990_TEMP1);
static SENSOR_DEVICE_ATTR(temp2_input, S_IRUGO, ltc2990_show_value, NULL,
			  LTC2990_TEMP2);
static SENSOR_DEVICE_ATTR(temp3_input, S_IRUGO, ltc2990_show_value, NULL,
			  LTC2990_TEMP3);
static SENSOR_DEVICE_ATTR(curr1_input, S_IRUGO, ltc2990_show_value, NULL,
			  LTC2990_CURR1);
static SENSOR_DEVICE_ATTR(curr2_input, S_IRUGO, ltc2990_show_value, NULL,
			  LTC2990_CURR2);
static SENSOR_DEVICE_ATTR(in0_input, S_IRUGO, ltc2990_show_value, NULL,
			  LTC2990_IN0);
static SENSOR_DEVICE_ATTR(in1_input, S_IRUGO, ltc2990_show_value, NULL,
			  LTC2990_IN1);
static SENSOR_DEVICE_ATTR(in2_input, S_IRUGO, ltc2990_show_value, NULL,
			  LTC2990_IN2);
static SENSOR_DEVICE_ATTR(in3_input, S_IRUGO, ltc2990_show_value, NULL,
			  LTC2990_IN3);
static SENSOR_DEVICE_ATTR(in4_input, S_IRUGO, ltc2990_show_value, NULL,
			  LTC2990_IN4);

static struct attribute *ltc2990_attrs[] = {
	&sensor_dev_attr_temp1_input.dev_attr.attr,
	&sensor_dev_attr_temp2_input.dev_attr.attr,
	&sensor_dev_attr_temp3_input.dev_attr.attr,
	&sensor_dev_attr_curr1_input.dev_attr.attr,
	&sensor_dev_attr_curr2_input.dev_attr.attr,
	&sensor_dev_attr_in0_input.dev_attr.attr,
	&sensor_dev_attr_in1_input.dev_attr.attr,
	&sensor_dev_attr_in2_input.dev_attr.attr,
	&sensor_dev_attr_in3_input.dev_attr.attr,
	&sensor_dev_attr_in4_input.dev_attr.attr,
	NULL,
};

static const struct attribute_group ltc2990_group = {
	.attrs = ltc2990_attrs,
	.is_visible = ltc2990_attrs_visible,
};
__ATTRIBUTE_GROUPS(ltc2990);

static int ltc2990_i2c_probe(struct i2c_client *i2c,
			     const struct i2c_device_id *id)
{
	int ret;
	struct device *hwmon_dev;
	struct ltc2990_data *data;
	struct device_node *of_node = i2c->dev.of_node;

	if (!i2c_check_functionality(i2c->adapter, I2C_FUNC_SMBUS_BYTE_DATA |
				     I2C_FUNC_SMBUS_WORD_DATA))
		return -ENODEV;

	data = devm_kzalloc(&i2c->dev, sizeof(struct ltc2990_data), GFP_KERNEL);
	if (unlikely(!data))
		return -ENOMEM;
	data->i2c = i2c;

	if (!of_node || of_property_read_u32(of_node, "lltc,mode", &data->mode))
		data->mode = LTC2990_CONTROL_MODE_DEFAULT;

	if (data->mode > LTC2990_CONTROL_MODE_MAX) {
		dev_err(&i2c->dev, "Error: Invalid mode %d.\n", data->mode);
		return -EINVAL;
	}

	/* Setup continuous mode */
	ret = i2c_smbus_write_byte_data(i2c, LTC2990_CONTROL,
					LTC2990_CONTROL_MEASURE_ALL |
					data->mode);
	if (ret < 0) {
		dev_err(&i2c->dev, "Error: Failed to set control mode.\n");
		return ret;
	}
	/* Trigger once to start continuous conversion */
	ret = i2c_smbus_write_byte_data(i2c, LTC2990_TRIGGER, 1);
	if (ret < 0) {
		dev_err(&i2c->dev, "Error: Failed to start acquisition.\n");
		return ret;
	}

	hwmon_dev = devm_hwmon_device_register_with_groups(&i2c->dev,
							   i2c->name,
							   data,
							   ltc2990_groups);

	return PTR_ERR_OR_ZERO(hwmon_dev);
}

static const struct i2c_device_id ltc2990_i2c_id[] = {
	{ "ltc2990", 0 },
	{}
};
MODULE_DEVICE_TABLE(i2c, ltc2990_i2c_id);

static struct i2c_driver ltc2990_i2c_driver = {
	.driver = {
		.name = "ltc2990",
	},
	.probe    = ltc2990_i2c_probe,
	.id_table = ltc2990_i2c_id,
};

module_i2c_driver(ltc2990_i2c_driver);

MODULE_DESCRIPTION("LTC2990 Sensor Driver");
MODULE_AUTHOR("Topic Embedded Products");
MODULE_LICENSE("GPL v2");
