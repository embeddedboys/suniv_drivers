/*
 * Driver for the i2c controller on the suniv SoC family.
 * This driver is based on i2c-mv64xxx.c
 *
 * Author: IotaHydrae <writeforever@foxmail.com>
 *
 * 2022 (c) IotaHydrae.  This file is licensed under the terms of the GNU 
 * General Public License version 2.  This program is licensed "as is" without
 * any warranty of any kind, whether express or implied.
 */
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/pinctrl/consumer.h>
#include <linux/pm_runtime.h>
#include <linux/reset.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_irq.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/delay.h>

#include <linux/kernel.h>
#include <linux/module.h>

#define SUNIV_I2C_BUS_CLOCK_DEFAULT 100000
#define SUNIV_I2C_BUS_CLOCK_FAST	400000

#define SUNIV_I2C_REG_CLOCK_M(clkm) ((0xf & clkm) << 3)
#define SUNIV_I2C_REG_CLOCK_N(clkn) (0x7 & clkn)

/* suniv i2c control register bits */
#define SUNIV_I2C_REG_CONTROL_INT_EN BIT(7)
#define SUNIV_I2C_REG_CONTROL_BUS_EN BIT(6)
#define SUNIV_I2C_REG_CONTROL_M_STA  BIT(5)
#define SUNIV_I2C_REG_CONTROL_M_STP  BIT(4)
#define SUNIV_I2C_REG_CONTROL_INT_FLAG BIT(3)
#define SUNIV_I2C_REG_CONTROL_A_ACK  BIT(2)

/* suniv i2c status register vals */
#define SUNIV_I2C_BUS_STATUS_ERROR         0x00
#define SUNIV_I2C_BUS_STATUS_START         0x08
#define SUNIV_I2C_BUS_STATUS_REPEAT_START  0x10
#define SUNIV_I2C_BUS_STATUS_ADDR_WR_ACK   0x18
#define SUNIV_I2C_BUS_STATUS_ADDR_WR_NOACK 0x20
#define SUNIV_I2C_BUS_STATUS_MASTER_DATA_ACK 0x28

enum {
	SUNIV_I2C_BUS_DIR_RD = 0x00,
	SUNIV_I2C_BUS_DIR_WR = 0x01,
};

#define SUNIV_CONTLR_NAME "suniv_i2c"

struct suniv_i2c_regs {
	u8 addr;	/* TWI Slave Address Register */
	u8 xaddr;	/* TWI Extend Address Register */
	u8 data;	/* TWI Data Register */
	u8 cntr;	/* TWI Control Register */
	u8 stat;	/* TWI Status Register */
	u8 ccr;		/* TWI Clock Register */
	u8 srst;	/* TWI Soft Reset Register */
	u8 efr;		/* TWI Enhance Feature Register */
	u8 lcr;		/* TWI Line Control Register */
};

struct suniv_i2c_data {
	void   __iomem		  *base;
	int                    irq;
	struct suniv_i2c_regs  reg_offsets;
	struct i2c_msg		   *msgs;
	int			           num_msgs;
	struct i2c_adapter	   adapter;
	struct clk              *clk;

	wait_queue_head_t      wait_queue;
	spinlock_t             lock;

	struct reset_control	*rstc;
};

struct suniv_i2c_regs suniv_i2c_regs_f1c100s = {
	.addr  = 0x00,
	.xaddr = 0x04,
	.data  = 0x08,
	.cntr  = 0x0c,
	.stat  = 0x10,
	.ccr   = 0x14,
	.srst  = 0x18,
	.efr   = 0x1c,
	.lcr   = 0x20,
};

static int suniv_i2c_of_config(struct suniv_i2c_data *i2c_data, struct device *dev)
{
	int rc = 0;
	struct device_node *np = dev->of_node;
	__u32 bus_freq;

	rc = of_property_read_u32(np, "clock-frequency", &bus_freq);
	if(rc)
		bus_freq = SUNIV_I2C_BUS_CLOCK_DEFAULT;
	
	return rc;
}

static void suniv_i2c_hw_init(struct suniv_i2c_data *i2c_data)
{
	/* set the bus clock, temporarily set to 100Kbit/s */
	writel(SUNIV_I2C_REG_CLOCK_N(2) | SUNIV_I2C_REG_CLOCK_M(11),
		   i2c_data->base + i2c_data->reg_offsets.ccr);

	/* clear addr,xaddr registers */
	writel(0, i2c_data->base + i2c_data->reg_offsets.addr);
	writel(0, i2c_data->base + i2c_data->reg_offsets.xaddr);

	/* enable bus */
	writel(SUNIV_I2C_REG_CONTROL_BUS_EN | SUNIV_I2C_REG_CONTROL_M_STP,
		   i2c_data->base + i2c_data->reg_offsets.cntr);
}

/* send i2c bus start signal */
static void suniv_i2c_send_start(struct suniv_i2c_data *i2c_data)
{
	printk(KERN_WARNING "%s, sending start signal\n", __func__);
}

/* suniv i2c bus intr handler */
static irqreturn_t suniv_i2c_isr(int irq, void *dev_id)
{
	printk("%s\n", __func__);
	return IRQ_HANDLED;
}

/*
 * master_xfer should return the number of messages successfully
 * processed, or a negative value on error
 */
static int suniv_i2c_xfer(struct i2c_adapter *adap, struct i2c_msg *msgs,
			   				  int num)
{
	int index;
	struct suniv_i2c_data *i2c_data = i2c_get_adapdata(adap);
	
	/* test i2c start signal */
	suniv_i2c_send_start(i2c_data);


	/* do simple i2c msg loop */
	for(index = 0; index<msgs->len;index++) {
		printk(KERN_WARNING "prepared send to %d\n", msgs[index].addr);
	}
	return 0;
}
							  
/* To determine what the adapter supports */
static u32 suniv_i2c_functionality(struct i2c_adapter *adap)
{
	/* we don't need 10bit slave address now */
	return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL;
}

/* the smbus function could simulate by the i2c master_xfer func */
static const struct i2c_algorithm suniv_i2c_algo = {
	.master_xfer   = suniv_i2c_xfer,
	.functionality = suniv_i2c_functionality,
};

static int suniv_i2c_probe(struct platform_device *pdev)
{
	struct suniv_i2c_data *i2c_data;
	int rc;

	printk("%s\n", __func__);

	if(!pdev->dev.of_node)
		return -ENODEV;

	i2c_data = devm_kzalloc(&pdev->dev, sizeof(struct suniv_i2c_data), 
							GFP_KERNEL);
	if(!i2c_data)
		return -ENOMEM;

	/* ioremap the bus register base addr */
	i2c_data->base = devm_platform_ioremap_resource(pdev, 0);
	if(IS_ERR(i2c_data->base)){
		return PTR_ERR(i2c_data->base);
	}

	/* get irq number from device */
	i2c_data->irq = platform_get_irq(pdev, 0);
	if(i2c_data->irq < 0){
		printk("%s, can't get irq\n", __func__);
		return i2c_data->irq;
	}

	/* init locks */
	init_waitqueue_head(&i2c_data->wait_queue);
	spin_lock_init(&i2c_data->lock);

	/* get clock */
	i2c_data->clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(i2c_data->clk)) {
		dev_err(&pdev->dev, "No clock specified\n");
		return PTR_ERR(i2c_data->clk);
	}


	/* copy regs offset to self data */
	memcpy(&i2c_data->reg_offsets, &suniv_i2c_regs_f1c100s, sizeof(struct suniv_i2c_regs));

	/* setting i2c adapter structure */
	i2c_data->adapter.owner       = THIS_MODULE;
	i2c_data->adapter.algo        = &suniv_i2c_algo;
	i2c_data->adapter.algo_data   = NULL;
	i2c_data->adapter.dev.parent  = &pdev->dev;
	i2c_data->adapter.retries     = 3;
	i2c_data->adapter.timeout     = HZ;
	i2c_data->adapter.nr          = pdev->id;
	i2c_data->adapter.dev.of_node = pdev->dev.of_node;
	strlcpy(i2c_data->adapter.name, SUNIV_CONTLR_NAME " adapter", sizeof(i2c_data->adapter.name));

	/* set privte data */
	platform_set_drvdata(pdev, i2c_data);
	i2c_set_adapdata(&i2c_data->adapter, i2c_data);

	/* clks and reset */
	clk_prepare_enable(i2c_data->clk);
	reset_control_reset(i2c_data->rstc);

	/* i2c bus hardware init */
	suniv_i2c_hw_init(i2c_data);

	/* configure properties from dt */
	rc = suniv_i2c_of_config(i2c_data, &pdev->dev);
	if(rc)
		return rc;

	/* request irq */
	rc = devm_request_irq(&pdev->dev, i2c_data->irq, suniv_i2c_isr, 0, 
						   SUNIV_CONTLR_NAME "adapter", i2c_data);

	/* do last work, add adapter to system */
	if(rc){
		dev_err(&i2c_data->adapter.dev,
			"suniv: can't register intr handler irq%d: %d\n", i2c_data->irq, rc);
		return rc;
	}
	
	rc = i2c_add_numbered_adapter(&i2c_data->adapter);
	if(rc !=0) {
		dev_err(&pdev->dev, "failed to add adapter\n");
		goto err_free_irq;
	}

err_free_irq:
	free_irq(i2c_data->irq, i2c_data);

	return rc;
}

static int suniv_i2c_remove(struct platform_device *pdev)
{
	return 0;
}

static const struct of_device_id suniv_i2c_of_match_table[] = {
	{.compatible = "allwinner,suniv-f1c100s-i2c", .data = &suniv_i2c_regs_f1c100s},
	{.compatible = "allwinner,suniv-f1c200s-i2c", .data = &suniv_i2c_regs_f1c100s},
	{},
};
MODULE_DEVICE_TABLE(of, suniv_i2c_of_match_table);

static struct platform_driver suniv_i2c_driver = {
	.probe  = suniv_i2c_probe,
	.remove = suniv_i2c_remove,
	.driver = {
		.name = "suniv_i2c",
		.of_match_table = of_match_ptr(suniv_i2c_of_match_table),
	},
};

module_platform_driver(suniv_i2c_driver);

MODULE_AUTHOR("Mark A. Greer <mgreer@mvista.com>");
MODULE_AUTHOR("IotaHydrae writeforever@foxmail.com");
MODULE_DESCRIPTION("Suniv SoC family host bridge i2c adapter driver");
MODULE_LICENSE("GPL");
