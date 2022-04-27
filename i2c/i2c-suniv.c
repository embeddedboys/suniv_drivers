/*
 * Driver for the i2c controller on the suniv SoC family.
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

#define SUNIV_CONTLR_NAME "suniv_i2c"

struct suniv_i2c_regs {
	u8 addr;
	u8 xaddr;
	u8 data;
	u8 cntr;
	u8 stat;
	u8 ccr;
	u8 srst;
	u8 efr;
	u8 lcr;
};

struct suniv_i2c_data {
	void   __iomem		  *base;
	int                    irq;
	struct suniv_i2c_regs  reg_offsets;
	struct i2c_msg		   *msgs;
	int			           num_msgs;
	struct i2c_adapter	   adapter;

	wait_queue_head_t      wait_queue;
	spinlock_t             lock;
};

struct suniv_i2c_regs suniv_i2c_regs_f1c100s = {
	.addr  = 0x00,	/* TWI Slave Address Register */
	.xaddr = 0x04,	/* TWI Extend Address Register */
	.data  = 0x08,	/* TWI Data Register */
	.cntr  = 0x0c,	/* TWI Control Register */
	.stat  = 0x10,	/* TWI Status Register */
	.ccr   = 0x14,	/*  */
	.srst  = 0x18,
	.efr   = 0x1c,
	.lcr   = 0x20,
};

static void suniv_i2c_send_start(struct suniv_i2c_data *i2c_data)
{
	
}

static irqreturn_t suniv_i2c_isr(int irq, void *dev_id)
{
	return IRQ_HANDLED;
}

/*
 * master_xfer should return the number of messages successfully
 * processed, or a negative value on error
 */
static int suniv_i2c_xfer(struct i2c_adapter *adap, struct i2c_msg *msgs,
			   				  int num)
{
	for(int i = 0; i<msgs->len;i++) {
		
	}
	return 0;
}
							  
/* To determine what the adapter supports */
static u32 suniv_i2c_functionality(struct i2c_adapter *adap)
{
	return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL;
}

static const struct i2c_algorithm suniv_i2c_algo = {
	.master_xfer   = suniv_i2c_xfer,
	.functionality = suniv_i2c_functionality,
};

static int suniv_i2c_probe(struct platform_device *pdev)
{
	struct suniv_i2c_data *i2c_data;
	int ret;

	i2c_data = devm_kzalloc(&pdev->dev, sizeof(struct suniv_i2c_data), 
							GFP_KERNEL);
	if(!i2c_data){
		return -ENOMEM;
	}

	i2c_data->base = devm_platform_ioremap_resource(pdev, 0);
	if(IS_ERR(i2c_data->base)){
		return PTR_ERR(i2c_data->base);
	}

	init_waitqueue_head(&i2c_data->wait_queue);
	spin_lock_init(&i2c_data->lock);

	i2c_data->irq = platform_get_irq(pdev, 0);
	if(i2c_data->irq < 0){
		return -EINVAL;
	}

	memcpy(&i2c_data->reg_offsets, &suniv_i2c_regs_f1c100s, sizeof(struct suniv_i2c_regs));

	i2c_data->adapter.owner       = THIS_MODULE;
	i2c_data->adapter.dev.parent  = &pdev->dev;
	i2c_data->adapter.algo        = &suniv_i2c_algo;
	i2c_data->adapter.nr          = pdev->id;
	i2c_data->adapter.dev.of_node = pdev->dev.of_node;

	platform_set_drvdata(pdev, i2c_data);
	i2c_set_adapdata(&i2c_data->adapter, i2c_data);

	ret = devm_request_irq(&pdev->dev, i2c_data->irq, suniv_i2c_isr, 0, 
						   SUNIV_CONTLR_NAME "adapter", i2c_data);
	
	if(ret) {
		dev_err(&pdev->dev, "Failed to request irq %d\n", i2c_data->irq);
		return ret;
	} else if (ret = i2c_add_numbered_adapter(&i2c_data->adapter) != 0) {
		dev_err(&pdev->dev, "Failed to add adapter\n");
		goto err_free_irq;
	}

err_free_irq:
	free_irq(drv_data->irq, i2c_data);

	return ret;
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

MODULE_AUTHOR("IotaHydrae writeforever@foxmail.com");
MODULE_DESCRIPTION("Suniv SoC family host bridge i2c adapter driver");
MODULE_LICENSE("GPL");
