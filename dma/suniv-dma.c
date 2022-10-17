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

/* Suniv DMA Register offsets */
#define SUNIV_DMA_NDMA_BASE         0x100
#define SUNIV_DMA_DDMA_BASE         0x300
#define SUNIV_DMA_CHN_OFFSET(n) (n*0x20)
#define SUNIV_NDMA_CHN_REG_OFFSET(v,n)   (v + SUNIV_DMA_NDMA_BASE + \
                                              SUNIV_DMA_CHN_OFFSET(n))
#define SUNIV_DDMA_CHN_REG_OFFSET(v,n)   (v + SUNIV_DMA_DDMA_BASE + \
                                              SUNIV_DMA_CHN_OFFSET(n))
/* Global Registers Map of Suniv DMA */
#define SUNIV_DMA_INT_CTRL_REG      0x00
#define SUNIV_DMA_INT_STA_REG       0x04
#define SUNIV_DMA_PTY_CFG_REG       0x08

/* Channel Registers Map of Suniv DMA */
#define SUNIV_NDMA_CFG_REG(n)       SUNIV_NDMA_CHN_REG_OFFSET(0x00, n)
#define SUNIV_NDMA_SRC_ADR_REG(n)   SUNIV_NDMA_CHN_REG_OFFSET(0x04, n)
#define SUNIV_NDMA_DES_ADR_REG(n)   SUNIV_NDMA_CHN_REG_OFFSET(0x08, n)
#define SUNIV_NDMA_BYTE_CNT_REG(n)  SUNIV_NDMA_CHN_REG_OFFSET(0x0c, n)

#define SUNIV_DDMA_CFG_REG(n)       SUNIV_DDMA_CHN_REG_OFFSET(0x00, n)
#define SUNIV_DDMA_SRC_ADR_REG(n)   SUNIV_DDMA_CHN_REG_OFFSET(0x04, n)
#define SUNIV_DDMA_DES_ADR_REG(n)   SUNIV_DDMA_CHN_REG_OFFSET(0x08, n)
#define SUNIV_DDMA_BYTE_CNT_REG(n)  SUNIV_DDMA_CHN_REG_OFFSET(0x0c, n)
#define SUNIV_DDMA_PAR_REG(n)       SUNIV_DDMA_CHN_REG_OFFSET(0x18, n)
#define SUNIV_DDMA_GEN_DATA(n)      SUNIV_DDMA_CHN_REG_OFFSET(0x1c, n)

/* Suniv DMA direction */
#define SUNIV_DMA_MEM_TO_MEM        0x00
#define SUNIV_DMA_MEM_TO_DEV        0x01
#define SUNIV_DMA_DEV_TO_MEM        0x02
#define SUNIV_DMA_DEV_TO_DEV        0x03

#define SUNIV_DMA_NDMA_NR_CHANNELS  4
#define SUNIV_DMA_DDMA_NR_CHANNELS  4
#define SUNIV_DMA_MAX_CHANNELS      (SUNIV_DMA_NDMA_NR_CHANNELS + \
                                     SUNIV_DMA_DDMA_NR_CHANNELS)

#define SUNIV_DMA_MAX_BURST         4

enum suniv_dma_width {
    SUNIV_DMA_1_BYTE,
    SUNIV_DMA_2_BYTE,
    SUNIV_DMA_4_BYTE,
};

enum suniv_dma_busrt_size {
    SUNIV_DMA_BURST_SINGLE,
    SUNIV_DMA_BURST_INCR4,
};

struct suniv_dma_desc {
    struct virt_dma_desc vdesc;
    enum dma_transfer_direction dir;
};

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
    
    struct clk              *clk;
    struct reset_control    *rstc;

    struct suniv_dma_chan   chan[SUNIV_DMA_MAX_CHANNELS];
    u32                     nr_channels;
};

static inline u32 suniv_dma_read(struct suniv_dma *dma_dev, u32 reg)
{
    return readl_relaxed(dma_dev->base + reg);
}

static inline void suniv_dma_write(struct suniv_dma *dma_dev, u32 reg, u32 val)
{
    writel_relaxed(val, dma_dev->base + reg);
}

static inline struct suniv_dma_chan *to_suniv_dma_chan(struct dma_chan *c)
{
    return container_of(c, struct suniv_dma_chan, vchan.chan);
}

static inline struct suniv_dma *to_suniv_dma(struct dma_chan *c)
{
    struct suniv_dma_chan *sc = to_suniv_dma_chan(c);

    return container_of(sc, struct suniv_dma, chan[c->chan_id]);
}

static inline struct suniv_dma_desc *to_suniv_dma_desc(struct virt_dma_desc *vd)
{
    return container_of(vd, struct suniv_dma_desc, vdesc);
}

static int suniv_dma_enable(struct suniv_dma *dma_dev)
{
    int rc;

    rc = clk_prepare_enable(dma_dev->clk);
    if (rc)
        return rc;

    rc = reset_control_deassert(dma_dev->rstc);
    if (rc)
        return rc;

    return rc;
}

static void suniv_dma_disable(struct suniv_dma *dma_dev)
{
    clk_disable_unprepare(dma_dev->clk);

    reset_control_assert(dma_dev->rstc);
}

static irqreturn_t suniv_dma_isr(int irq, void *devid)
{
    return IRQ_HANDLED;
}

static struct dma_chan *suniv_dma_of_xlate(struct of_phandle_args *dma_spec,
                                           struct of_dma *ofdma)
{
    printk("%s, %s\n", __func__, dma_spec->np->name);
    return NULL;
}

static int suniv_dma_alloc_chan_resources(struct dma_chan *chan)
{
    printk("%s\n", __func__);
    return 0;
}

static void suniv_dma_free_chan_reosuces(struct dma_chan *chan)
{
    printk("%s\n", __func__);
}

static int suniv_dma_terminate_all(struct dma_chan *chan)
{
    printk("%s\n", __func__);
    return 0;
}

static enum dma_status suniv_dma_tx_status(struct dma_chan *chan,
                               dma_cookie_t cookie,
                               struct dma_tx_state *txstate)
{
    printk("%s\n", __func__);
    return 0;
}

static void suniv_dma_issue_pending(struct dma_chan *chan)
{
    printk("%s\n", __func__);
}

static struct dma_async_tx_descriptor *suniv_dma_prep_dma_memcpy(
		                struct dma_chan *c, dma_addr_t dst, dma_addr_t src,
                        size_t len, unsigned long flags)
{
    struct suniv_dma_chan *chan = to_suniv_dma_chan(c);
    struct suniv_dma_desc *desc;

    desc = kzalloc(sizeof(struct suniv_dma_desc), GFP_NOWAIT);
    if (!desc)
        return NULL;


    printk("%s\n", __func__);
    return vchan_tx_prep(&chan->vchan, &desc->vdesc, flags);
}

static void suniv_dma_desc_free(struct virt_dma_desc *vdesc)
{
    kfree(container_of(vdesc, struct suniv_dma_desc, vdesc));
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
        chan->vchan.desc_free = suniv_dma_desc_free;
        vchan_init(&chan->vchan, dd);
    }

    /* set private data */
    platform_set_drvdata(pdev, dma_dev);

    rc = suniv_dma_enable(dma_dev);
    if (rc) {
        dev_err(&pdev->dev, "enable suniv dma failed:%d", rc);
        return rc;
    }

    rc = dma_async_device_register(dd);
    if (rc) {
        dev_err(&pdev->dev, "register dma device failed:%d", rc);
        return -EAGAIN;
    }

    rc = of_dma_controller_register(pdev->dev.of_node,
                                    suniv_dma_of_xlate, dma_dev);
    if (rc) {
        dev_err(&pdev->dev, "suniv DMA OF registration failed:%d\n", rc);
        goto err_unregister;
    }

    return 0;

err_unregister:
    dma_async_device_unregister(dd);
    return rc;
}

static int suniv_dma_remove(struct platform_device *pdev)
{
    struct suniv_dma *dma_dev = platform_get_drvdata(pdev);

    printk("%s\n", __func__);
    if (dma_dev->irq > 0)
        devm_free_irq(&pdev->dev, dma_dev->irq, dma_dev);
    
    /* channels clean */

    of_dma_controller_free(pdev->dev.of_node);
    dma_async_device_unregister(&dma_dev->ddev);
    suniv_dma_disable(dma_dev);

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
