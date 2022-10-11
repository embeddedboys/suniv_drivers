/*
 * A virtual gpio_chip driver example
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

struct gpio_virt{
    u32 base;
    u32 ngpio;

    struct gpio_chip gc;
};

static int g_gpio_val = 0;

static int gpio_virt_of_config(struct gpio_virt *vt_gpio_dev)
{
    int rc;
    struct platform_device *pdev = container_of(vt_gpio_dev->gc.parent, 
                                                struct platform_device,
                                                dev);

    rc = of_property_read_u32(pdev->dev.of_node, "base", &vt_gpio_dev->base);
    rc = of_property_read_u32(pdev->dev.of_node, "ngpios", &vt_gpio_dev->ngpio);

    return 0;
}

static int gpio_virt_direction_output(struct gpio_chip *gc,
                                      unsigned offset, int value)
{
    printk("set pin %d as output %s\n", offset, value ? "high" : "low");
    return 0;
}

static int gpio_virt_direction_input(struct gpio_chip *chip,
                                     unsigned offset)
{
    printk("set pin %d as input\n", offset);
    return 0;
}

static int gpio_virt_get_value(struct gpio_chip *chip, unsigned offset)
{
    int val;
    val = (g_gpio_val & (1 << offset)) ? 1: 0;
    printk("get pin %d, it's val = %d\n", offset, val);
    return val;
}

static void gpio_virt_set_value(struct gpio_chip *chip,
                               unsigned offset, int value)
{
    printk("set pin %d as %d\n", offset, value);
    if (value)
        g_gpio_val |= (1 << offset);
    else
        g_gpio_val &= ~(1 << offset);
}

static int gpio_virt_probe(struct platform_device *pdev)
{
    int rc;
    struct gpio_virt *vt_gpio_dev;

    printk("%s, %d\n", __func__, __LINE__);

    /* Alloc a gpio_chip structure */
    vt_gpio_dev = devm_kzalloc(&pdev->dev, sizeof(struct gpio_virt), 
                               GFP_KERNEL);

    if (!vt_gpio_dev)
        return -ENOMEM;

    /* Setting gpio_chip */
    printk("%s, setting gpio_chip\n", __func__);

    /* Setting functions of gpio_chip */
    vt_gpio_dev->gc.label = pdev->name;
    vt_gpio_dev->gc.direction_output = gpio_virt_direction_output;
    vt_gpio_dev->gc.direction_input = gpio_virt_direction_input;
    vt_gpio_dev->gc.get = gpio_virt_get_value;
    vt_gpio_dev->gc.set = gpio_virt_set_value;

    vt_gpio_dev->gc.parent = &pdev->dev;
    vt_gpio_dev->gc.owner = THIS_MODULE;

    /* Setting base, ngpio of gpio_chip */
    vt_gpio_dev->gc.base = -1;
    rc = of_property_read_u32(pdev->dev.of_node, "ngpios", &vt_gpio_dev->ngpio);
    vt_gpio_dev->gc.ngpio = vt_gpio_dev->ngpio;

    /* Register gpio_chip */
    rc = devm_gpiochip_add_data(&pdev->dev, &vt_gpio_dev->gc, NULL);

    return 0;
}

static int gpio_virt_remove(struct platform_device *pdev)
{
    printk("%s, %d\n", __func__, __LINE__);
    return 0;
}

static const struct of_device_id gpio_virt_of_match_table[] = {
    {.compatible = "hamsterbear,gpio_virt" },
    { /* KEEP THIS */ },
};
MODULE_DEVICE_TABLE(of, gpio_virt_of_match_table);

static struct platform_driver gpio_virt_driver = {
    .probe  = gpio_virt_probe,
    .remove = gpio_virt_remove,
    .driver = {
        .name = "hamsterbear,gpio_virt",
        .of_match_table = of_match_ptr(gpio_virt_of_match_table),
    },
};

module_platform_driver(gpio_virt_driver);

MODULE_AUTHOR("IotaHydrae <writeforever@foxmail.com>");
MODULE_DESCRIPTION("A virtual gpio_chip driver example");
MODULE_LICENSE("GPL");
