/*
 * max25614.c - max25614 LED flash driver
 *
 * Copyright (c) 2018-2022, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <media/camera_common.h>
#include <linux/module.h>
#include <media/max25614.h>



struct max25614 {
	struct i2c_client *i2c_client;
	struct regmap *regmap;
	struct mutex lock;
};

int max25614_write_reg(struct device *dev, u8 addr, u8 val)
{
	struct max25614 *priv = dev_get_drvdata(dev);
	int err;

	err = regmap_write(priv->regmap, addr, val);
	if (err)
		dev_err(dev, "%s:i2c write failed, 0x%x = %x\n",
			__func__, addr, val);

	usleep_range(100, 110);

	return err;
}
EXPORT_SYMBOL(max25614_write_reg);

int max25614_read_reg(struct device *dev, u8 addr, u8 *val)
{
	struct max25614 *priv = dev_get_drvdata(dev);
	int err;
	u32 reg_val = 0;

	err = regmap_read(priv->regmap, addr, &reg_val);

	if (err)
		dev_err(dev, "%s:i2c read failed, 0x%x = %x\n",
			__func__, addr, reg_val);

	*val = reg_val & 0xFF;

	return err;
}
EXPORT_SYMBOL(max25614_read_reg);

static  struct regmap_config max25614_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.cache_type = REGCACHE_RBTREE,
};

static int max25614_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	struct max25614 *priv;
	int err = 0;

	dev_info(&client->dev, "[MAX25614]: probing LED flash\n");

	priv = devm_kzalloc(&client->dev, sizeof(*priv), GFP_KERNEL);
	priv->i2c_client = client;
	priv->regmap = devm_regmap_init_i2c(priv->i2c_client,
				&max25614_regmap_config);
	if (IS_ERR(priv->regmap)) {
		dev_err(&client->dev,
			"regmap init failed: %ld\n", PTR_ERR(priv->regmap));
		return -ENODEV;
	}

	mutex_init(&priv->lock);

	dev_set_drvdata(&client->dev, priv);

	dev_info(&client->dev, "%s:  success\n", __func__);

	return err;
}

static int max25614_remove(struct i2c_client *client)
{
	struct max25614 *priv;

	if (client != NULL) {
		priv = dev_get_drvdata(&client->dev);
		mutex_destroy(&priv->lock);
		i2c_unregister_device(client);
		client = NULL;
	}

	return 0;
}

static const struct i2c_device_id max25614_id[] = {
	{ "max25614", 0 },
	{ },
};

static const struct of_device_id max25614_of_match[] = {
	{ .compatible = "adi,max25614", },
	{ },
};
MODULE_DEVICE_TABLE(of, max25614_of_match);
MODULE_DEVICE_TABLE(i2c, max25614_id);

static struct i2c_driver max25614_i2c_driver = {
	.driver = {
		.name = "max25614",
		.owner = THIS_MODULE,
	},
	.probe = max25614_probe,
	.remove = max25614_remove,
	.id_table = max25614_id,
};

static int __init max25614_init(void)
{
	return i2c_add_driver(&max25614_i2c_driver);
}

static void __exit max25614_exit(void)
{
	i2c_del_driver(&max25614_i2c_driver);
}

module_init(max25614_init);
module_exit(max25614_exit);

MODULE_DESCRIPTION("LED flash driver max25614");
MODULE_AUTHOR("Software Supreme");
MODULE_LICENSE("GPL v2");
