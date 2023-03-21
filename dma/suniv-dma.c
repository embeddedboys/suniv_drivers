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
#define SUNIV_DMA_CHN_OFFSET(n)     (n*0x20)
#define SUNIV_NDMA_CHN_REG_OFFSET(v,n)   (v + SUNIV_DMA_NDMA_BASE + \
                                              SUNIV_DMA_CHN_OFFSET(n))
#define SUNIV_DDMA_CHN_REG_OFFSET(v,n)   (v + SUNIV_DMA_DDMA_BASE + \
                                              SUNIV_DMA_CHN_OFFSET(n))
/* Global Registers Map of Suniv DMA */
#define SUNIV_DMA_INT_CTRL_REG      0x00
/* Bits of Interrupt control register */
#define SUNIV_DDMA_FULL_TRANS_INT_EN(n)    BIT(16+2*n+1)
#define SUNIV_DDMA_HALF_TRANS_INT_EN(n)    BIT(16+2*n)
#define SUNIV_NDMA_FULL_TRANS_INT_EN(n)    BIT(2*n+1)
#define SUNIV_NDMA_HALF_TRANS_INT_EN(n)    BIT(2*n)

#define SUNIV_DMA_INT_STA_REG       0x04
/* Bits of Interrupt status register */
#define SUNIV_DDMA_FULL_TRANS_INT_PENDING(n)   BIT(16+2*n+1)
#define SUNIV_DDMA_HALF_TRANS_INT_PENDING(n)   BIT(16+2*n)
#define SUNIV_NDMA_FULL_TRANS_INT_PENDING(n)   BIT(2*n+1)
#define SUNIV_NDMA_HALF_TRANS_INT_PENDING(n)   BIT(2*n)

#define SUNIV_DMA_PTY_CFG_REG       0x08
/* Bits of Priority configure register */
#define SUNIV_DMA_AUTO_LOCK_GATING  BIT(16)
#define SUNIV_DMA_NDMA_PRIORITY_COUNTER_OFFSET   7
#define SUNIV_DMA_AC320_PRIORITY_COUNTER_OFFSET  4


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

/* Bits of DMA Configure Register */
#define SUNIV_DMA_LOADING               BIT(31)
#define SUNIV_DMA_BUSY_OFFSET           30
#define SUNIV_DMA_CONTINUOUS_MODE_EN    BIT(29)
#define SUNIV_DMA_WAIT_STATE_OFFSET     26
#define SUNIV_DMA_DES_DATAWIDTH_OFFSET  24
#define SUNIV_DMA_DES_BRUST_LEN_OFFSET  23
#define SUNIV_DMA_DES_ADDR_TYPE_OFFSET  21
#define SUNIV_DMA_DES_DRQ_TYPE_OFFSET   16
#define SUNIV_DMA_REMAIN_BYTE_CNT_READ_EN  BIT(15)
#define SUNIV_DMA_SRC_DATAWIDTH_OFFSET  8
#define SUNIV_DMA_SRC_BRUST_LEN_OFFSET  7
#define SUNIV_DMA_SRC_ADDR_TYPE_OFFSET  5
#define SUNIV_DMA_SRC_DRQ_TYPE_OFFSET   0



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

#define SUNIV_NDMA_BYET_CNT_MASK     GENMASK(17, 0)
#define SUNIV_DDMA_BYET_CNT_MASK     GENMASK(24, 0)
#define SUNIV_DDMA_DRQ_TYPE_SDRAM   0x1
#define SUNIV_NDMA_DRQ_TYPE_SDRAM   0x11

enum suniv_dma_width {
    SUNIV_DMA_1_BYTE,
    SUNIV_DMA_2_BYTE,
    SUNIV_DMA_4_BYTE,
};

enum suniv_dma_busrt_len {
    SUNIV_DMA_BURST_1,
    SUNIV_DMA_BURST_4,
};

struct suniv_dma_chn_hw {
    u32 busy;
    dma_addr_t src_addr;
    dma_addr_t des_addr;
    u32 datawidth;
    u32 brust_len;
    u32 byte_cnt;

    u32 cfg;
};

struct suniv_dma_desc {
    struct suniv_dma_chn_hw      chan_hw;
    struct virt_dma_desc         vdesc;
    enum dma_transfer_direction  dir;
};

struct suniv_dma_chan {
    u32 id;
    bool is_dedicated;

    struct virt_dma_chan    vchan; /* current virtual channel servicing */
    struct suniv_dma_desc   *desc;
    struct dma_slave_config cfg;
};

struct suniv_dma {
    void __iomem            *base;      /* Memory address base of DMA controller */
    int                     irq;

    spinlock_t              lock;

    struct dma_device       ddev;

    struct clk              *hclk;
    struct clk              *mclk;
    struct reset_control    *rstc;

    struct suniv_dma_chan   chan[SUNIV_DMA_MAX_CHANNELS];
    u32                     nr_channels;
};

static inline u32 suniv_dma_read(struct suniv_dma *dma_dev, u32 reg)
{
    return readl(dma_dev->base + reg);
}

static inline void suniv_dma_write(struct suniv_dma *dma_dev, u32 reg, u32 val)
{
    writel(val, dma_dev->base + reg);
}

static inline void suniv_dma_update(struct suniv_dma *dma_dev, u32 reg, 
                                    u32 mask, u32 val)
{
    u32 dump_val = readl(dma_dev->base + reg);
    u32 tmp;

    tmp = (dump_val & ~mask) | val;
    writel(tmp, dma_dev->base + reg);
}

static inline struct suniv_dma_chan *to_suniv_dma_chan(struct dma_chan *c)
{
    return container_of(c, struct suniv_dma_chan, vchan.chan);
}

static inline struct suniv_dma *to_suniv_dma_by_dc(struct dma_chan *c)
{
    return container_of(c->device, struct suniv_dma, ddev);
}

static inline struct suniv_dma *to_suniv_dma_by_sdc(struct suniv_dma_chan *c)
{
    return container_of(c->vchan.chan.device, struct suniv_dma, ddev);
}

static inline struct suniv_dma_desc *to_suniv_dma_desc(struct virt_dma_desc *vd)
{
    return container_of(vd, struct suniv_dma_desc, vdesc);
}

static int suniv_dma_enable(struct suniv_dma *dma_dev)
{
    int rc;

    rc = clk_prepare_enable(dma_dev->hclk);
    if (rc)
        return rc;

    rc = clk_prepare_enable(dma_dev->mclk);
    if (rc)
        return rc;

    rc = reset_control_deassert(dma_dev->rstc);
    if (rc)
        return rc;

    suniv_dma_write(dma_dev, SUNIV_DMA_INT_CTRL_REG, 0);
    suniv_dma_write(dma_dev, SUNIV_DMA_INT_STA_REG, 0xFFFFFFFF);

    return rc;
}

static void suniv_dma_disable(struct suniv_dma *dma_dev)
{
    clk_disable_unprepare(dma_dev->hclk);
    clk_disable_unprepare(dma_dev->mclk);

    reset_control_assert(dma_dev->rstc);
}

static void suniv_dma_set_interurpt(struct suniv_dma *dma_dev,
                                    struct suniv_dma_chan *chan,
                                    u32 end)
{
    u32 reg;
	unsigned long flags;

    printk("%s\n", __func__);
	spin_lock_irqsave(&dma_dev->lock, flags);

    reg = suniv_dma_read(dma_dev, SUNIV_DMA_INT_CTRL_REG);

    if (end && chan->is_dedicated)
        reg |= SUNIV_DDMA_FULL_TRANS_INT_EN(chan->id);
    else if (end && !chan->is_dedicated)
        reg |= SUNIV_NDMA_FULL_TRANS_INT_EN(chan->id);
    else if (!end && chan->is_dedicated)
        reg &= ~SUNIV_DDMA_FULL_TRANS_INT_EN(chan->id);
    else // (!end && !chan->is_dedicated)
        reg &= ~SUNIV_NDMA_FULL_TRANS_INT_EN(chan->id);

    suniv_dma_write(dma_dev, SUNIV_DMA_INT_CTRL_REG, reg);

    spin_unlock_irqrestore(&dma_dev->lock, flags);
}

static irqreturn_t suniv_dma_isr(int irq, void *devid)
{
    struct suniv_dma *dma_dev = (struct suniv_dma *)devid;
    struct suniv_dma_chan *chan;
    int bit;
    //u32 remain_bytes;
    unsigned long irq_pending = suniv_dma_read(dma_dev, SUNIV_DMA_INT_STA_REG);

    printk("%s, irq_pending: 0x%lx\n", __func__, irq_pending);
    for_each_set_bit(bit, &irq_pending, 32) {
        chan = &dma_dev->chan[bit >> 1];

        if (bit & 1) {
            /* TODO: Disable chann IRQ */
            spin_lock(&dma_dev->lock);
            suniv_dma_set_interurpt(dma_dev, chan, 0);


            /* TODO: Clear chann Interrupt Pending */
         	suniv_dma_update(dma_dev, SUNIV_DMA_INT_STA_REG, 
            				chan->is_dedicated?
            				SUNIV_DDMA_FULL_TRANS_INT_PENDING(chan->id): 
            				SUNIV_NDMA_FULL_TRANS_INT_PENDING(chan->id), 
                            chan->is_dedicated?
            				SUNIV_DDMA_FULL_TRANS_INT_PENDING(chan->id): 
            				SUNIV_NDMA_FULL_TRANS_INT_PENDING(chan->id));

            
            spin_unlock(&dma_dev->lock);
        }
    }
    
    return IRQ_HANDLED;
}

static void suniv_dma_set_chn_config(struct suniv_dma_chan *schan, struct suniv_dma_desc *sdesc)
{
    struct suniv_dma *dma_dev = to_suniv_dma_by_sdc(schan);
    struct suniv_dma_chn_hw *hw = &sdesc->chan_hw;
    
    /* A bunch of `suniv_dma_write` here, config current dma channel */
    if (schan->is_dedicated) {
        printk("%s, Configuring DDMA chann: %d \n", __func__, schan->id);
        suniv_dma_write(dma_dev, SUNIV_DDMA_SRC_ADR_REG(schan->id), hw->src_addr);
        suniv_dma_write(dma_dev, SUNIV_DDMA_DES_ADR_REG(schan->id), hw->des_addr);
        suniv_dma_write(dma_dev, SUNIV_DDMA_BYTE_CNT_REG(schan->id), hw->byte_cnt);
        suniv_dma_write(dma_dev, SUNIV_DDMA_CFG_REG(schan->id), hw->byte_cnt);

    } else {
        printk("%s, Configuring NDMA chann: %d \n", __func__, schan->id);
        suniv_dma_write(dma_dev, SUNIV_NDMA_SRC_ADR_REG(schan->id), hw->src_addr);
        suniv_dma_write(dma_dev, SUNIV_NDMA_DES_ADR_REG(schan->id), hw->des_addr);
        suniv_dma_write(dma_dev, SUNIV_NDMA_BYTE_CNT_REG(schan->id), hw->byte_cnt);
        suniv_dma_write(dma_dev, SUNIV_NDMA_CFG_REG(schan->id), hw->cfg);
    }

    printk("%s, chnn %d cfg reg dump: 0x%x", schan->is_dedicated?"DDMA":"NDMA", 
           schan->id, suniv_dma_read(dma_dev, SUNIV_NDMA_CFG_REG(schan->id)));
}

static void suniv_dma_start(struct suniv_dma_chan *chan)
{
    struct suniv_dma *dma_dev = to_suniv_dma_by_sdc(chan);
    struct virt_dma_desc *vdesc;

    printk("%s\n", __func__);
    if(!chan->desc) {
        vdesc = vchan_next_desc(&chan->vchan);
        if (!vdesc)
            return;

        chan->desc = to_suniv_dma_desc(vdesc);
    }

    /* Hardware Configuration */
    suniv_dma_set_interurpt(dma_dev, chan, 1);
    suniv_dma_set_chn_config(chan, chan->desc);
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

static void suniv_dma_issue_pending(struct dma_chan *c)
{
    struct suniv_dma_chan *chan = to_suniv_dma_chan(c);
    unsigned long flags;

    printk("%s\n", __func__);
    spin_lock_irqsave(&chan->vchan.lock, flags);
    if (vchan_issue_pending(&chan->vchan) && !chan->desc)
        suniv_dma_start(chan);
    spin_unlock_irqrestore(&chan->vchan.lock, flags);
}

static struct dma_async_tx_descriptor *suniv_dma_prep_dma_memcpy(
		                struct dma_chan *c, dma_addr_t dst, dma_addr_t src,
                        size_t len, unsigned long flags)
{
    struct suniv_dma_chn_hw *hw;
    struct suniv_dma_desc *desc;
    struct suniv_dma_chan *chan = to_suniv_dma_chan(c);

    printk("%s, src: %u, dst: %u, len: %d", __func__, src, dst, len);

    desc = kzalloc(sizeof(struct suniv_dma_desc), GFP_NOWAIT);
    if (!desc)
        return NULL;

    /* Save hardware params */
    hw = &desc->chan_hw;

    hw->des_addr = dst;
    hw->src_addr = src;
/*
    if (IS_ALIGNED(len, 4)) {
        hw->datawidth = SUNIV_DMA_4_BYTE;
        hw->brust_len = SUNIV_DMA_BURST_4;
    } else if (IS_ALIGNED(len, 2)) {
        hw->datawidth = SUNIV_DMA_2_BYTE;
        hw->brust_len = SUNIV_DMA_BURST_1;
    } else {
        hw->datawidth = SUNIV_DMA_1_BYTE;
        hw->brust_len = SUNIV_DMA_BURST_1;
    }
*/
    hw->datawidth = SUNIV_DMA_4_BYTE;
    hw->brust_len = SUNIV_DMA_BURST_1;
    if (chan->is_dedicated) {
        hw->byte_cnt = len & SUNIV_DDMA_BYET_CNT_MASK;

        hw->cfg |= SUNIV_DDMA_DRQ_TYPE_SDRAM << SUNIV_DMA_DES_DRQ_TYPE_OFFSET;
        hw->cfg |= SUNIV_DDMA_DRQ_TYPE_SDRAM << SUNIV_DMA_SRC_DRQ_TYPE_OFFSET;
    } else {
        hw->byte_cnt = len & SUNIV_NDMA_BYET_CNT_MASK;


        hw->cfg |= SUNIV_NDMA_DRQ_TYPE_SDRAM << SUNIV_DMA_DES_DRQ_TYPE_OFFSET;
        hw->cfg |= SUNIV_NDMA_DRQ_TYPE_SDRAM << SUNIV_DMA_SRC_DRQ_TYPE_OFFSET;
    }

    /* Common configuration */
    hw->cfg |= SUNIV_DMA_LOADING;

    return vchan_tx_prep(&chan->vchan, &desc->vdesc, flags);
}

static void suniv_dma_desc_free(struct virt_dma_desc *vdesc)
{
    kfree(container_of(vdesc, struct suniv_dma_desc, vdesc));
}

static int suniv_dma_probe(struct platform_device *pdev)
{
    int i,j, rc;
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
    dma_dev->hclk = devm_clk_get(&pdev->dev, "ahb");
    if (IS_ERR(dma_dev->hclk)) {
        dev_err(&pdev->dev, "Unable to acquire AHB clock\n");
        return PTR_ERR(dma_dev->hclk);
    }

    dma_dev->mclk = devm_clk_get(&pdev->dev, "mod");
    if (IS_ERR(dma_dev->mclk)) {
        dev_err(&pdev->dev, "Unable to acquire module clock\n");
        return PTR_ERR(dma_dev->mclk);
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

    for (i = 0; i < SUNIV_DMA_NDMA_NR_CHANNELS; i++) {
        chan = &dma_dev->chan[i];
        chan->id = i;
        chan->vchan.desc_free = suniv_dma_desc_free;
        chan->is_dedicated = 0;
        vchan_init(&chan->vchan, dd);
    }

    for (j = 0; i < SUNIV_DMA_MAX_CHANNELS; j++, i++) {
        chan = &dma_dev->chan[i];
        chan->id = j;
        chan->vchan.desc_free = suniv_dma_desc_free;
        chan->is_dedicated = 1;
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
