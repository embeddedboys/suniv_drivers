/**
 * @file spi-suniv.c
 * @author IotaHydrae (writeforever@foxmail.com)
 * @brief 
 * @version 0.1
 * @date 2023-03-10
 * 
 * MIT License
 * 
 * Copyright 2022 IotaHydrae(writeforever@foxmail.com)
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * 
 */

#include <linux/slab.h>
#include <linux/wait.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/pinctrl/consumer.h>
#include <linux/pm_runtime.h>
#include <linux/reset.h>
#include <linux/io.h>
#include <linux/spi/spi.h>
#include <linux/spi/spi_bitbang.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_irq.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/delay.h>

#include <linux/kernel.h>
#include <linux/module.h>

#define DRV_NAME "spi_suniv"

struct suniv_spi {
    struct spi_bitbang      bitbang;
    struct device           *dev;

    void __iomem            *base;
    unsigned long           base_phys;

    int                     irq;

    struct spi_master       *master;
    struct clk              *hclk;
    struct clk              *mclk;

    struct completion       xfer_done;
    struct reset_control    *rstc;
};

static int spi_suniv_transfer(struct spi_device *spi, struct spi_message *mesg)
{
    return 0;
}

static int spi_suniv_probe(struct platform_device *pdev)
{
    struct spi_master *master;
    struct suniv_spi  *sspi;

    master = devm_spi_alloc_master(&pdev->dev, sizeof(struct suniv_spi));
    if (!master) {
        dev_err(&pdev->dev, "Unable to allocate SPI Master\n");
        return -ENOMEM;
    }

    platform_set_drvdata(pdev, master);
    sspi = spi_master_get_devdata(master);

    sspi->base = devm_platform_ioremap_resource(pdev, 0);
    if (IS_ERR(sspi->base))
        return PTR_ERR(sspi->base);

    /* clock */

    /* locks */

    /* alloc spi master */
    sspi->master = devm_spi_alloc_master(&pdev->dev, 0);
    if (!sspi->master) {
        dev_err(&pdev->dev, "devm_spi_alloc_master error!\n");
        return -ENOMEM;
    }

    return 0;
}

static int spi_suniv_remove(struct platform_device *pdev)
{
    return 0;
}

static const struct of_device_id spi_suniv_dt_ids[] = {
    { .compatible = "allwinner,suniv-f1c100s-spi" },
    { .compatible = "allwinner,suniv-f1c200s-spi" },
    { /* KEEP THIS */ },
};
MODULE_DEVICE_TABLE(of, spi_suniv_dt_ids);

static struct platform_driver spi_suniv_driver = {
    .probe = spi_suniv_probe,
    .remove = spi_suniv_remove,
    .driver = {
        .name = DRV_NAME,
        .of_match_table = of_match_ptr(spi_suniv_dt_ids),
    },
};

module_platform_driver(spi_suniv_driver);

MODULE_AUTHOR("Iota Hydrae <writeforever@foxmail.com>");
MODULE_DESCRIPTION("Allwinner suniv SoC family SPI controller driver");
MODULE_LICENSE("GPL");