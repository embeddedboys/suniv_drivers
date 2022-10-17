/*
 * A simple DMA Engine driver for Allwinner F series SoC
 *
 * Author: IotaHydrae <writeforever@foxmail.com>
 *
 * 2022 (c) IotaHydrae.  This file is licensed under the terms of the GNU
 * General Public License version 2.  This program is licensed "as is" without
 * any warranty of any kind, whether express or implied.
 */

#include <linux/clk.h>
#include <linux/dmaengine.h>
#include <linux/dmapool.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of_dma.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/reset.h>

#include "virt-dma.h"

/* So, Let's take a look at the DMA system of Suniv SoC,
 * 
 * First, There are two kinds of system DMA in the Soc.
 * One is Normal DMA (NDMA) with 4 channels;
 * the other is Dedicated DMA (DDMA) with 4 channels;
 * 
 * Normal DMA master interface support single and INCR4 
 * operation (may be early terminated), and will treat any 
 * response from AHB bus as OK response. 
 * Dedicated DMA master interface support single and INCR4 
 * operation (may be early terminated), and will treat any 
 * response from DMA bus as OK response. 
 */

#define DRV_NAME "suniv-dma"

#define SUNIV_DMA_NDMA_NR_CHANNELS  4
#define SUNIV_DMA_DDMA_NR_CHANNELS  4
#define SUNIV_DMA_MAX_CHANNELS  (SUNIV_DMA_NDMA_NR_CHANNELS + \
                                 SUNIV_DMA_DDMA_NR_CHANNELS)

#define SUNIV_DMA_MAX_BURST         4

/* Suniv DMA Register offsets */
#define SUNIV_DMA_NDMA_BASE_OFFSET      0x100
#define SUNIV_DMA_DDMA_BASE_OFFSET      0x300
#define SUNIV_DMA_COMMON_OFFSET(n)      (n*0x20)

/* Registers Map of Suniv DMA */
#define SUNIV_DMA_INT_CTRL_REG      0x00
#define SUNIV_DMA_INT_STA_REG       0x04


/* Suniv DMA direction */
#define SUNIV_DMA_MEM_TO_MEM        0x00
#define SUNIV_DMA_MEM_TO_DEV        0x01
#define SUNIV_DMA_DEV_TO_MEM        0x02
#define SUNIV_DMA_DEV_TO_DEV        0x03

struct suniv_dma_chan {
    u32 id;
    bool is_dedicated;

    struct virt_dma_chan    vchan; /* current virtual channel servicing */
    struct dma_slave_config cfg;
};

struct suniv_dma {
    void __iomem            *base;      /* Memory address base of DMA controller */
    int                     irq;

    spinlock_t              lock;

    struct dma_device       ddev;
    struct device           *dev;
    
    struct clk              *clk;
    struct reset_control    *rstc;

    struct suniv_dma_chan   chan[SUNIV_DMA_MAX_CHANNELS];
    u32                     nr_channels;
};

static irqreturn_t suniv_dma_isr(int irq, void *devid)
{
    return IRQ_HANDLED;
}

static struct dma_chan *suniv_dma_of_xlate(struct of_phandle_args *dma_spec,
                                           struct of_dma *ofdma)
{
    return NULL;
}

static int suniv_dma_alloc_chan_resources(struct dma_chan *chan)
{
    return 0;
}

static void suniv_dma_free_chan_reosuces(struct dma_chan *chan)
{

}

static int suniv_dma_terminate_all(struct dma_chan *chan)
{
    return 0;
}

static enum dma_status suniv_dma_tx_status(struct dma_chan *chan,
                               dma_cookie_t cookie,
                               struct dma_tx_state *txstate)
{
    return 0;
}

static void suniv_dma_issue_pending(struct dma_chan *chan)
{

}

static struct dma_async_tx_descriptor *suniv_dma_prep_dma_memcpy(
		                struct dma_chan *chan, dma_addr_t dst, dma_addr_t src,
                        size_t len, unsigned long flags)
{
    return NULL;
}

static int suniv_dma_probe(struct platform_device *pdev)
{
    int i, rc;
    struct suniv_dma *dma_dev;
    struct dma_device *dd;
    struct suniv_dma_chan *chan;

    printk("%s\n", __func__);

    printk("%s, alloc memory of dma_dev\n", __func__);
    dma_dev = devm_kzalloc(&pdev->dev, sizeof(struct suniv_dma),
                           GFP_KERNEL);

    if (!dma_dev)
        return -ENOMEM;

    dd = &dma_dev->ddev;

    printk("%s, ioremap the base addr of DMA registers\n", __func__);
    dma_dev->base = devm_platform_ioremap_resource(pdev, 0);
    if (IS_ERR(dma_dev->base))
        return PTR_ERR(dma_dev->base);

    printk("%s, DMA register base: %p\n", __func__, dma_dev->base);

    /* Initialize locks */
    spin_lock_init(&dma_dev->lock);

    /* Acquire clocks */
    dma_dev->clk = devm_clk_get(&pdev->dev, "ahb");
    if (IS_ERR(dma_dev->clk)) {
        dev_err(&pdev->dev, "Unable to acquire AHB clock\n");
        return PTR_ERR(dma_dev->clk);
    }

    dma_dev->rstc = devm_reset_control_array_get_exclusive(&pdev->dev);
    if (IS_ERR(dma_dev->rstc)) {
        dev_err(&pdev->dev, "Unable to get reset controller\n");
        return PTR_ERR(dma_dev->rstc);
    }

    /* set private data */
    platform_set_drvdata(pdev, dma_dev);

    dma_cap_set(DMA_MEMCPY, dd->cap_mask);
    dma_cap_set(DMA_SLAVE, dd->cap_mask);
    dd->device_alloc_chan_resources = suniv_dma_alloc_chan_resources;
    dd->device_free_chan_resources  = suniv_dma_free_chan_reosuces;
    dd->device_terminate_all        = suniv_dma_terminate_all;
    dd->device_tx_status            = suniv_dma_tx_status;
    dd->device_issue_pending        = suniv_dma_issue_pending;
    dd->device_prep_dma_memcpy      = suniv_dma_prep_dma_memcpy;

    dd->src_addr_widths = BIT(DMA_SLAVE_BUSWIDTH_1_BYTE) |
                          BIT(DMA_SLAVE_BUSWIDTH_2_BYTES) |
                          BIT(DMA_SLAVE_BUSWIDTH_4_BYTES);
    dd->dst_addr_widths = BIT(DMA_SLAVE_BUSWIDTH_1_BYTE) |
                          BIT(DMA_SLAVE_BUSWIDTH_2_BYTES) |
                          BIT(DMA_SLAVE_BUSWIDTH_4_BYTES);
    dd->directions = BIT(DMA_MEM_TO_MEM) | BIT(DMA_MEM_TO_DEV) |
                     BIT(DMA_DEV_TO_MEM) | BIT(DMA_DEV_TO_DEV);
    dd->max_burst = SUNIV_DMA_MAX_BURST;
    dd->dev = &pdev->dev;
    INIT_LIST_HEAD(&dd->channels);

    for(i=0; i < SUNIV_DMA_MAX_CHANNELS; i++) {
        chan = &dma_dev->chan[i];
        chan->id = i;
        vchan_init(&chan->vchan, dd);
    }

    rc = dma_async_device_register(dd);
    if (rc)
        return -EAGAIN;

    dma_dev->irq = platform_get_irq(pdev, 0);
    if (dma_dev->irq < 0) {
        printk("%s, can't get irq\n", __func__);
        return dma_dev->irq;
    }
    printk("%s, Got irq number %d from device\n", __func__, dma_dev->irq);


    rc = devm_request_irq(&pdev->dev, dma_dev->irq,
                          suniv_dma_isr, 0,
                          DRV_NAME " device", dma_dev);
    if (rc) {
        dev_err(&pdev->dev, "request irq failed with err %d\n", rc);
        return rc;
    }

    rc = of_dma_controller_register(pdev->dev.of_node,
                                    suniv_dma_of_xlate, dma_dev);
    if (rc) {
        dev_err(&pdev->dev, "Suniv DMA OF registration failed %d\n", rc);
        goto err_unregister;
    }

    return 0;

err_unregister:
    dma_async_device_unregister(dd);

    return rc;
}

static int suniv_dma_remove(struct platform_device *pdev)
{
    printk("%s\n", __func__);
    return 0;
}

static struct of_device_id suniv_dma_of_match_table[] = {
    {.compatible = "allwinner,suniv-f1c100s-dma" },
    { /* KEEP THIS */ }
};
MODULE_DEVICE_TABLE(of, suniv_dma_of_match_table);

static struct platform_driver suniv_dma_driver = {
    .probe  = suniv_dma_probe,
    .remove = suniv_dma_remove,
    .driver = {
        .name = DRV_NAME,
        .of_match_table = of_match_ptr(suniv_dma_of_match_table),
    },
};

module_platform_driver(suniv_dma_driver);

MODULE_AUTHOR("IotaHydrae <writeforever@foxmail.com>");
MODULE_DESCRIPTION("A simple DMA Engine driver for Allwinner F series SoC");
MODULE_LICENSE("GPL");
