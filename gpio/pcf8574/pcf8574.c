/*
 * A simple gpio_chip driver for pcf8574
 *
 * Author: IotaHydrae <writeforever@foxmail.com>
 *
 * 2022 (c) IotaHydrae.  This file is licensed under the terms of the GNU
 * General Public License version 2.  This program is licensed "as is" without
 * any warranty of any kind, whether express or implied.
 */
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/irqdomain.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/slab.h>
#include <linux/spinlock.h>

struct pcf8574{
    u32 base;
    u32 ngpio;

    struct mutex lock;
    struct gpio_chip gc;
};

static inline int pcf8574_write_byte(struct gpio_chip *gc, u8 byte)
{
    int rc;
    const char buf[1] = { byte };
    struct i2c_client *client = to_i2c_client(gc->parent);

    rc = i2c_master_send(client, buf, 1);

    return rc;
}

static inline u8 pcf8574_read_byte(struct gpio_chip *gc)
{
    int rc;
    u8 buf[1];
    struct i2c_client *client = to_i2c_client(gc->parent);

    rc = i2c_master_recv(client, buf, 1);

    return buf[0];
}

static int pcf8574_of_config(struct pcf8574 *pcf8574_dev)
{
    int rc;
    struct i2c_client *client = to_i2c_client(pcf8574_dev->gc.parent);

    rc = of_property_read_u32(client->dev.of_node, "base", &pcf8574_dev->base);
    pcf8574_dev->gc.base = -1;

    rc = of_property_read_u32(client->dev.of_node, "ngpios", &pcf8574_dev->ngpio);
    pcf8574_dev->gc.ngpio = pcf8574_dev->ngpio;

    return 0;
}

static int pcf8574_direction_output(struct gpio_chip *gc,
                                      unsigned offset, int value)
{
    printk("set pin %d as output %s\n", offset, value ? "high" : "low");

    return 0;
}

static int pcf8574_direction_input(struct gpio_chip *chip,
                                     unsigned offset)
{
    printk("set pin %d as input\n", offset);

    return 0;
}

static int pcf8574_get_value(struct gpio_chip *chip, unsigned offset)
{
    u8 reg = pcf8574_read_byte(chip);
    printk("%s, val: 0x%02x\n", __func__, reg);

    return reg & (1 << offset);
}

static void pcf8574_set_value(struct gpio_chip *chip,
                               unsigned offset, int value)
{
    u8 reg = pcf8574_read_byte(chip);
    printk("set pin %d as %d\n", offset, value);

    if (value)
        reg |= (1 << offset);
    else
        reg &= ~(1 << offset);

    printk("%s, val: 0x%02x\n", __func__, reg);
    pcf8574_write_byte(chip, reg);
}

static int pcf8574_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    int rc;
    struct pcf8574 *pcf8574_dev;
    struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);

    printk("%s, %d\n", __func__, __LINE__);

    /* Check Adapter's functionality */
    if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE))
        return -EIO;

    /* Alloc a gpio_chip structure */
    pcf8574_dev = devm_kzalloc(&client->dev, sizeof(struct pcf8574), 
                               GFP_KERNEL);

    if (!pcf8574_dev)
        return -ENOMEM;

    /* Setting gpio_chip */
    printk("%s, setting gpio_chip\n", __func__);

    /* Setting functions of gpio_chip */
    pcf8574_dev->gc.label = client->name;
    pcf8574_dev->gc.direction_output = pcf8574_direction_output;
    pcf8574_dev->gc.direction_input = pcf8574_direction_input;
    pcf8574_dev->gc.get = pcf8574_get_value;
    pcf8574_dev->gc.set = pcf8574_set_value;

    pcf8574_dev->gc.parent = &client->dev;
    pcf8574_dev->gc.owner = THIS_MODULE;

    /* Setting base, ngpio of gpio_chip */
    pcf8574_of_config(pcf8574_dev);

    /* TODO: chip irq support */

    /* Register gpio_chip */
    rc = devm_gpiochip_add_data(&client->dev, &pcf8574_dev->gc, NULL);

    return 0;
}

static int pcf8574_remove(struct i2c_client *client)
{
    printk("%s, %d\n", __func__, __LINE__);
    return 0;
}

static const struct i2c_device_id pcf8547_id_table[] = {
    { "pcf8574", 0 },
    { /* KEEP THIS */ },
};
MODULE_DEVICE_TABLE(i2c, pcf8547_id_table);

static const struct of_device_id pcf8574_of_match_table[] = {
    {.compatible = "hamsterbear,pcf8574" },
    { /* KEEP THIS */ },
};
MODULE_DEVICE_TABLE(of, pcf8574_of_match_table);

static struct i2c_driver pcf8574_driver = {
    .probe  = pcf8574_probe,
    .remove = pcf8574_remove,
    .driver = {
        .name = "hamsterbear,pcf8574",
        .of_match_table = of_match_ptr(pcf8574_of_match_table),
    },
    .id_table = pcf8547_id_table,
};

module_i2c_driver(pcf8574_driver);

MODULE_AUTHOR("IotaHydrae <writeforever@foxmail.com>");
MODULE_DESCRIPTION("A virtual gpio_chip driver example");
MODULE_LICENSE("GPL");
