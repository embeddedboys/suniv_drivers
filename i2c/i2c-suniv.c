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
#include <linux/wait.h>
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

#define SUNIV_I2C_BUS_CLOCK_DEFAULT                     100000
#define SUNIV_I2C_BUS_CLOCK_FAST                        400000

#define SUNIV_I2C_ADDR(addr) ((0xff & addr) << 1)
#define SUNIV_I2C_REG_CLOCK_M(clkm) ((0xf & clkm) << 3)
#define SUNIV_I2C_REG_CLOCK_N(clkn) (0x7 & clkn)

/* suniv i2c control register bits */
#define SUNIV_I2C_REG_CONTROL_INT_EN                    BIT(7)
#define SUNIV_I2C_REG_CONTROL_BUS_EN                    BIT(6)
#define SUNIV_I2C_REG_CONTROL_M_STA                     BIT(5)
#define SUNIV_I2C_REG_CONTROL_M_STP                     BIT(4)
#define SUNIV_I2C_REG_CONTROL_INT_FLAG                  BIT(3)
#define SUNIV_I2C_REG_CONTROL_A_ACK                     BIT(2)

/* suniv i2c status register vals */
#define SUNIV_I2C_BUS_STATUS_ERROR                              0x00
#define SUNIV_I2C_BUS_STATUS_START                              0x08
#define SUNIV_I2C_BUS_STATUS_REPEAT_START                       0x10
#define SUNIV_I2C_BUS_STATUS_ADDR_WR_ACK                        0x18
#define SUNIV_I2C_BUS_STATUS_ADDR_WR_NOACK                      0x20
#define SUNIV_I2C_BUS_STATUS_MASTER_DATA_SEND_ACK               0x28
#define SUNIV_I2C_BUS_STATUS_MASTER_DATA_SEND_NOACK             0x30
#define SUNIV_I2C_BUS_STATUS_ADDR_RD_ACK                        0x40
#define SUNIV_I2C_BUS_STATUS_ADDR_RD_NOACK                      0x48
#define SUNIV_I2C_BUS_STATUS_MASTER_DATA_RECV_ACK               0x50
#define SUNIV_I2C_BUS_STATUS_MASTER_DATA_RECV_NOACK             0x58
#define SUNIV_I2C_BUS_STATUS_SEC_ADDR_WR_ACK                    0xd0

enum {
        SUNIV_I2C_BUS_DIR_WR = 0x00,
        SUNIV_I2C_BUS_DIR_RD = 0x01,
};

#define SUNIV_CONTLR_NAME "suniv_i2c"

struct suniv_i2c_regs {
        u8 addr;    /* TWI Slave Address Register */
        u8 xaddr;   /* TWI Extend Address Register */
        u8 data;    /* TWI Data Register */
        u8 cntr;    /* TWI Control Register */
        u8 stat;    /* TWI Status Register */
        u8 ccr;     /* TWI Clock Register */
        u8 srst;    /* TWI Soft Reset Register */
        u8 efr;     /* TWI Enhance Feature Register */
        u8 lcr;     /* TWI Line Control Register */
};

struct suniv_i2c_data {
        void __iomem            *base;
        int                     irq;
        
        u32                                             rc;
        u32                     dir;
        u32                     cntr_bits;
        u32                     addr;
        u32                     xaddr;
        u32                     byte_left;
        u32                     byte_pos;
        struct suniv_i2c_regs   reg_offsets;
        
        struct i2c_msg                  *msg;
        struct i2c_msg          *msgs;
        int                     num_msgs;
        struct i2c_adapter      adapter;
        struct clk              *hclk;
        struct clk              *mclk;
        
        struct completion       complete;
        wait_queue_head_t       wait_queue;
        spinlock_t              lock;
        
        u32                     sleep;
        
        struct reset_control    *rstc;
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

static inline u32 suniv_i2c_read(struct suniv_i2c_data *i2c_data, u32 reg)
{
        return readl(i2c_data->base + reg);
}

static inline void suniv_i2c_write(struct suniv_i2c_data *i2c_data, u32 reg,
                                   u32 val)
{
        writel(val, i2c_data->base + reg);
}

static ssize_t suniv_i2c_dump_register(struct device *dev,
                                       struct device_attribute *attr, char *buf)
{
        struct suniv_i2c_data *i2c_data = dev->driver_data;
        
        printk("---------------------------> %s <---------------------------\n",
               __func__);

        printk("addr  : 0x%02x \t\t\t xaddr  : 0x%02x \t\t\t data  : 0x%02x\n",
               suniv_i2c_read(i2c_data, i2c_data->reg_offsets.addr),
               suniv_i2c_read(i2c_data, i2c_data->reg_offsets.xaddr),
               suniv_i2c_read(i2c_data, i2c_data->reg_offsets.data));

        printk("cntr  : 0x%02x \t\t\t stat	: 0x%02x \t\t\t ccr   : 0x%02x \n",
               suniv_i2c_read(i2c_data, i2c_data->reg_offsets.cntr),
               suniv_i2c_read(i2c_data, i2c_data->reg_offsets.stat),
               suniv_i2c_read(i2c_data, i2c_data->reg_offsets.ccr));

        printk("srst  : 0x%02x \t\t\t efr	: 0x%02x \t\t\t lcr   : 0x%02x \n",
               suniv_i2c_read(i2c_data, i2c_data->reg_offsets.srst),
               suniv_i2c_read(i2c_data, i2c_data->reg_offsets.efr),
               suniv_i2c_read(i2c_data, i2c_data->reg_offsets.lcr));

        return 0;
}

static DEVICE_ATTR(dump_register, S_IRUSR, suniv_i2c_dump_register, NULL);

static struct attribute *suniv_i2c_sysfs_attrs[] = {
        &dev_attr_dump_register.attr,
        NULL
};

static struct attribute_group suniv_i2c_attribute_group = {
        .attrs = suniv_i2c_sysfs_attrs,
};

static int suniv_i2c_create_sysfs(struct device  *dev)
{
        int rc = 0;
        
        pr_debug("%s\n", __func__);
        
        rc = sysfs_create_group(&dev->kobj, &suniv_i2c_attribute_group);
        if (rc) {
                pr_debug("%s, create sysfs group failed!\n", __func__);
                sysfs_remove_group(&dev->kobj, &suniv_i2c_attribute_group);
                rc = -ENOMEM;
        }
        
        return rc;
}

static int suniv_i2c_of_config(struct suniv_i2c_data *i2c_data,
                               struct device *dev)
{
        int rc = 0;
        struct device_node *np = dev->of_node;
        u32 bus_freq;
        
        rc = of_property_read_u32(np, "clock-frequency", &bus_freq);
        
        if (rc)
                bus_freq = SUNIV_I2C_BUS_CLOCK_DEFAULT;
                
        return rc;
}

static inline void suniv_i2c_hw_init(struct suniv_i2c_data *i2c_data)
{
        //int i2c_speed;
        /*
         * we don't need to reset bus here, because
         * the reset system should do this work
         */
        suniv_i2c_write(i2c_data, i2c_data->reg_offsets.srst, 1);
        
        /* set the bus clock, temporarily set to 100Kbit/s */
        /*
        i2c_speed = SUNIV_I2C_REG_CLOCK_N(2) | SUNIV_I2C_REG_CLOCK_M(11);
        pr_debug("%s, i2c speed: 0x%x", __func__, i2c_speed);
        */
        suniv_i2c_write(i2c_data, i2c_data->reg_offsets.ccr,
                        SUNIV_I2C_REG_CLOCK_N(2) | SUNIV_I2C_REG_CLOCK_M(11));
                        
        /* clear registers */
        suniv_i2c_write(i2c_data, i2c_data->reg_offsets.addr, 0);
        suniv_i2c_write(i2c_data, i2c_data->reg_offsets.xaddr, 0);
        suniv_i2c_write(i2c_data, i2c_data->reg_offsets.cntr, 0);
        
        /* enable bus */
        suniv_i2c_write(i2c_data, i2c_data->reg_offsets.cntr,
                        SUNIV_I2C_REG_CONTROL_BUS_EN |
                        SUNIV_I2C_REG_CONTROL_M_STP);
}

/* send i2c bus start signal */
static void suniv_i2c_send_start(struct suniv_i2c_data *i2c_data,
                                 struct i2c_msg *msg)
{
        /* Store msg info here, used in isr */
        i2c_data->msg = msg;
        i2c_data->byte_left = msg->len;
        i2c_data->byte_pos = 0;
        
        pr_debug("%s, addr:0x%x\n", __func__, i2c_data->msgs->addr);
        pr_debug("%s, byte left:%d\n", __func__, i2c_data->byte_left);
        
        /* Transfer direction set */
        if (msg->flags & I2C_M_RD)
                i2c_data->dir = SUNIV_I2C_BUS_DIR_RD;
        else
                i2c_data->dir = SUNIV_I2C_BUS_DIR_WR;
                
        /* Set cntr register bits, like enable bus, intr etc. */
        i2c_data->cntr_bits = SUNIV_I2C_REG_CONTROL_BUS_EN
                              | SUNIV_I2C_REG_CONTROL_A_ACK
                              | SUNIV_I2C_REG_CONTROL_INT_EN;
        //pr_debug("%s, i2c_data->cntr_bits : 0x%x\n", __func__, i2c_data->cntr_bits);
        
        /* If it's a 10 bit address */
        if (msg->flags & I2C_M_TEN) {
                i2c_data->addr = SUNIV_I2C_ADDR(msg->addr) | i2c_data->dir;
                i2c_data->xaddr = (u32)msg->addr & 0xff;
        } else {
                i2c_data->addr = SUNIV_I2C_ADDR(msg->addr) | i2c_data->dir;
                i2c_data->xaddr = 0;
        }
        
        /* Write into cntr register */
        pr_debug("%s, sending start signal\n", __func__);
        suniv_i2c_write(i2c_data, i2c_data->reg_offsets.cntr,
                        i2c_data->cntr_bits | SUNIV_I2C_REG_CONTROL_M_STA);
}

/* suniv i2c adapter intr handler */
static irqreturn_t suniv_i2c_isr(int irq, void *dev_id)
{
        pr_debug("%s\n", __func__);
        
        return IRQ_WAKE_THREAD;
}

static irqreturn_t suniv_i2c_isr_thread_fn(int irq, void *dev_id)
{
        u32                                             status_stat;
        struct suniv_i2c_data           *i2c_data = dev_id;
        
        /* If the int flag have been set */
        while (suniv_i2c_read(i2c_data, i2c_data->reg_offsets.cntr) &
               SUNIV_I2C_REG_CONTROL_INT_FLAG) {
               
                /* Check the status register and do action */
                status_stat = suniv_i2c_read(i2c_data, i2c_data->reg_offsets.stat);
                //suniv_i2c_dump_register(&i2c_data->adapter.dev, NULL, NULL);
                switch (status_stat) {
                
                /* Error interrupt */
                case SUNIV_I2C_BUS_STATUS_ERROR: /* 0x00 */
                        pr_debug("%s, 0x%02x, SUNIV_I2C_BUS_STATUS_ERROR\n", __func__,
                                 SUNIV_I2C_BUS_STATUS_ERROR);
                        break;
                        
                /* Start condition interrupt */
                case SUNIV_I2C_BUS_STATUS_START: /* 0x08 */
                        pr_debug("%s, 0x%02x : SUNIV_I2C_BUS_STATUS_START\n", __func__,
                                 SUNIV_I2C_BUS_STATUS_START);
                        fallthrough;
                case SUNIV_I2C_BUS_STATUS_REPEAT_START: /* 0x10 */
                        pr_debug("%s, 0x%02x : SUNIV_I2C_BUS_STATUS_REPEAT_START\n", __func__,
                                 SUNIV_I2C_BUS_STATUS_REPEAT_START);
                        suniv_i2c_write(i2c_data, i2c_data->reg_offsets.data, i2c_data->addr);
                        suniv_i2c_write(i2c_data, i2c_data->reg_offsets.cntr, i2c_data->cntr_bits);
                        
                        break;
                        
                /* Write to slave */
                case SUNIV_I2C_BUS_STATUS_ADDR_WR_ACK: /* 0x18 */
                        pr_debug("%s, 0x%02x : SUNIV_I2C_BUS_STATUS_ADDR_WR_ACK : Address byte has been sent\n",
                                 __func__, SUNIV_I2C_BUS_STATUS_ADDR_WR_ACK);
                        /* TODO: check if it's a 10 bit addr */
                        fallthrough;
                case SUNIV_I2C_BUS_STATUS_SEC_ADDR_WR_ACK: /* 0xd0 */
                        pr_debug("%s, 0x%02x : SUNIV_I2C_BUS_STATUS_SEC_ADDR_WR_ACK : Second address byte has been sent\n",
                                 __func__, SUNIV_I2C_BUS_STATUS_SEC_ADDR_WR_ACK);
                        fallthrough;
                case SUNIV_I2C_BUS_STATUS_MASTER_DATA_SEND_ACK: /* 0x28 */
                        pr_debug("%s, 0x%02x : SUNIV_I2C_BUS_STATUS_MASTER_DATA_ACK : Data byte has been sent, %d left",
                                 __func__, SUNIV_I2C_BUS_STATUS_MASTER_DATA_SEND_ACK, i2c_data->byte_left);
                                 
                        if (i2c_data->byte_left == 0) {
                                pr_debug("%s, sending a stop signal\n", __func__);
                                
                                i2c_data->cntr_bits &= ~SUNIV_I2C_REG_CONTROL_INT_EN;
                                suniv_i2c_write(i2c_data, i2c_data->reg_offsets.cntr,
                                                i2c_data->cntr_bits |
                                                SUNIV_I2C_REG_CONTROL_M_STP);
                                i2c_data->rc++;
                                complete(&i2c_data->complete);
                        } else {
                                pr_debug("%s, writing 0x%02x to slave\n", __func__, *(i2c_data->msg->buf));
                                suniv_i2c_write(i2c_data, i2c_data->reg_offsets.data,
                                                i2c_data->msg->buf[i2c_data->byte_pos++]);
                                suniv_i2c_write(i2c_data, i2c_data->reg_offsets.cntr, i2c_data->cntr_bits);
                                i2c_data->byte_left--;
                        }
                        
                        break;
                        
                /* Read from slave */
                case SUNIV_I2C_BUS_STATUS_ADDR_RD_ACK: /* 0x40 */
                        pr_debug("%s, 0x%02x : SUNIV_I2C_BUS_STATUS_ADDR_RD_ACK", __func__,
                                 SUNIV_I2C_BUS_STATUS_ADDR_RD_ACK);
                        fallthrough;
                case SUNIV_I2C_BUS_STATUS_MASTER_DATA_RECV_ACK: /*         0x50 */
                        pr_debug("%s, 0x%02x : SUNIV_I2C_BUS_STATUS_MASTER_DATA_RECV_ACK", __func__,
                                 SUNIV_I2C_BUS_STATUS_MASTER_DATA_RECV_ACK);
                        /* copy received byte in register into i2c msg */
                        if (i2c_data->byte_left == 0) {
                                pr_debug("%s, sending a stop signal\n", __func__);
                                
                                i2c_data->cntr_bits &= ~SUNIV_I2C_REG_CONTROL_INT_EN;
                                suniv_i2c_write(i2c_data, i2c_data->reg_offsets.cntr,
                                                i2c_data->cntr_bits |
                                                SUNIV_I2C_REG_CONTROL_M_STP);
                                                
                                i2c_data->rc++;
                                complete(&i2c_data->complete);
                        } /*else if(i2c_data->byte_pos == 0) {
                                                        suniv_i2c_read(i2c_data,i2c_data->reg_offsets.data);
                                        }*/ else {
                                printk("%s, byte_pos : %d\n", __func__, i2c_data->byte_pos);
                                i2c_data->msg->buf[i2c_data->byte_pos++] = suniv_i2c_read(i2c_data,
                                                                                          i2c_data->reg_offsets.data);
                                pr_debug("%s, read 0x%02x from slave\n", __func__,
                                         i2c_data->msg->buf[i2c_data->byte_pos]);
                                suniv_i2c_write(i2c_data, i2c_data->reg_offsets.cntr, i2c_data->cntr_bits);
                                i2c_data->byte_left--;
                        }
                        
                        break;
                        
                /* Non device responsed in read mode */
                case SUNIV_I2C_BUS_STATUS_MASTER_DATA_RECV_NOACK: /* 0x58 */
                        pr_debug("%s, 0x%02x : SUNIV_I2C_BUS_STATUS_MASTER_DATA_RECV_NOACK", __func__,
                                 SUNIV_I2C_BUS_STATUS_MASTER_DATA_RECV_NOACK);
                        i2c_data->cntr_bits &= ~SUNIV_I2C_REG_CONTROL_INT_EN;
                        suniv_i2c_write(i2c_data, i2c_data->reg_offsets.cntr,
                                        i2c_data->cntr_bits |
                                        SUNIV_I2C_REG_CONTROL_M_STP);
                        break;
                        
                /* Non device responsed */
                case SUNIV_I2C_BUS_STATUS_ADDR_WR_NOACK:        /* 0x20 */
                        pr_debug("%s, 0x%02x : SUNIV_I2C_BUS_STATUS_ADDR_WR_NOACK", __func__,
                                 SUNIV_I2C_BUS_STATUS_ADDR_WR_NOACK);
                        fallthrough;
                case SUNIV_I2C_BUS_STATUS_MASTER_DATA_SEND_NOACK:               /* 0x30 */
                        pr_debug("%s, 0x%02x : SUNIV_I2C_BUS_STATUS_DATA_NOACK", __func__,
                                 SUNIV_I2C_BUS_STATUS_MASTER_DATA_SEND_NOACK);
                        fallthrough;
                case SUNIV_I2C_BUS_STATUS_ADDR_RD_NOACK:        /* 0x48 */
                        pr_debug("%s, 0x%02x : SUNIV_I2C_BUS_STATUS_ADDR_RD_NOACK", __func__,
                                 SUNIV_I2C_BUS_STATUS_ADDR_RD_NOACK);
                        i2c_data->cntr_bits &= ~SUNIV_I2C_REG_CONTROL_INT_EN;
                        suniv_i2c_write(i2c_data, i2c_data->reg_offsets.cntr,
                                        i2c_data->cntr_bits |
                                        SUNIV_I2C_REG_CONTROL_M_STP);
                        break;
                        
                default:
                        suniv_i2c_write(i2c_data, i2c_data->reg_offsets.cntr,
                                        i2c_data->cntr_bits | SUNIV_I2C_REG_CONTROL_M_STP);
                        suniv_i2c_hw_init(i2c_data);
                        i2c_recover_bus(&i2c_data->adapter);
                        break;
                }
                
                //complete(&i2c_data->complete);
        }
        
        return IRQ_HANDLED;
}


/* For each message,  */
static int suniv_i2c_do_msgs(struct suniv_i2c_data *i2c_data)
{
        int                     i;
        long                            time_left = 0;
        int                     num = i2c_data->num_msgs;
        struct i2c_msg                  *msgs = i2c_data->msgs;
        
        i2c_data->rc = 0;
        
        for (i = 0; i < num; i++) {
                /* send start signal and waiting for interrupt occured */
                pr_debug("%s, sending msg : %d", __func__, i);
                
                reinit_completion(&i2c_data->complete);
                suniv_i2c_send_start(i2c_data, &msgs[i]);
                
                /*
                time_left = wait_event_timeout(i2c_data->wait_queue, !i2c_data->sleep,
                                               i2c_data->adapter.timeout);
                                pr_debug("%s, time left : %d", __func__, (int)time_left);
                
                if (time_left == 0) {
                        pr_debug("%s, i2c bus time out:%d\n", __func__, (int)time_left);
                                                return time_left;
                }
                */
                
                /* After call this, if controller can't receive ack, this will timeout */
                time_left = wait_for_completion_timeout(&i2c_data->complete,
                                                        i2c_data->adapter.timeout);
                                                        
                pr_debug("%s, time left : %d", __func__, (int)time_left);
                
                if (!time_left) {
                        pr_debug("%s, i2c msg time out : %d", __func__, (int)time_left);
                        return -ETIMEDOUT;
                }
                
        }
        
        return i2c_data->rc;
}

/*
 * master_xfer should return the number of messages successfully
 * processed, or a negative value on error
 */
static int suniv_i2c_xfer(struct i2c_adapter *adap, struct i2c_msg *msgs,
                          int num)
{
        int                     rc = false;
        unsigned long                   flags;
        
        struct suniv_i2c_data *i2c_data = i2c_get_adapdata(adap);
        i2c_data->msgs = msgs;
        i2c_data->num_msgs = num;
        
        /* When the CPU host wants to start a bus transfer,
         * it initiates a bus START to enter the master mode by setting IM_STA bit
         * in the 2WIRE_CNTR register to high (before it must be low).
         * The TWI will assert INT line and INT_FLAG to indicate a
         * completion for the START condition and each consequent byte transfer.
         * At each interrupt, the micro-processor needs to check the 2WIRE_STAT register for current status.
         * A transfer has to be concluded with STOP condition by setting M_STP
         * bit high.
        */
        pr_debug("%s,  %d msg need to be transfer", __func__, i2c_data->num_msgs);
        
        //spin_lock_irqsave(&i2c_data->lock, flags);
        /* i2c_data->cntr_bits = suniv_i2c_read(i2c_data, i2c_data->reg_offsets.cntr); */
        pr_debug("%s, i2c_data->cntr_bits : 0x%x\n", __func__, i2c_data->cntr_bits);
        suniv_i2c_hw_init(i2c_data);
        
        /* do single i2c msg whatever read or write */
        if (num == 1)
                rc = true;
                
        /* if it's a write and read ops */
        if (num == 2 && !(msgs[0].flags & I2C_M_RD)
            && (msgs[1].flags & I2C_M_RD))
                rc = true;
                
        /* handle msgs */
        if (rc)
                rc = suniv_i2c_do_msgs(i2c_data);
        else
                pr_debug(KERN_WARNING "%s, required ops not supported!\n", __func__);
                
        //spin_unlock_irqrestore(&i2c_data->lock, flags);
        
        pr_debug("%s, successed msg count : %d", __func__, i2c_data->rc);
        return rc;
}

/* To determine what the adapter supports */
static u32 suniv_i2c_functionality(struct i2c_adapter *adap)
{
        /* we don't need 10bit slave address now */
        return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL;
}

/* some smbus function could simulate by the i2c master_xfer func */
static const struct i2c_algorithm suniv_i2c_algo = {
        .master_xfer   = suniv_i2c_xfer,
        .functionality = suniv_i2c_functionality,
};

static int suniv_i2c_probe(struct platform_device *pdev)
{
        struct suniv_i2c_data *i2c_data;
        int rc;
        
        printk("%s\n", __func__);
        
        if (!pdev->dev.of_node)
                return -ENODEV;
                
        printk("%s: alloc memory of i2c_data\n", __func__);
        i2c_data = devm_kzalloc(&pdev->dev, sizeof(struct suniv_i2c_data),
                                GFP_KERNEL);
                                
        if (!i2c_data)
                return -ENOMEM;
                
        printk("%s: ioremap the bus register base addr\n", __func__);
        /* ioremap the bus register base addr */
        i2c_data->base = devm_platform_ioremap_resource(pdev, 0);
        
        if (IS_ERR(i2c_data->base)) {
                return PTR_ERR(i2c_data->base);
        }
        
        dev_dbg(&pdev->dev, "i2c reg base: %p\n", i2c_data->base);
        
        printk("%s: get irq number from device\n", __func__);
        /* get irq number from device */
        i2c_data->irq = platform_get_irq(pdev, 0);
        
        if (i2c_data->irq < 0) {
                printk("%s, can't get irq\n", __func__);
                return i2c_data->irq;
        }
        
        /* init locks */
        init_waitqueue_head(&i2c_data->wait_queue);
        spin_lock_init(&i2c_data->lock);
        
        init_completion(&i2c_data->complete);
        
        /* get clocks */
        i2c_data->hclk = devm_clk_get(&pdev->dev, "ahb");
        
        if (IS_ERR(i2c_data->hclk)) {
                dev_err(&pdev->dev, "Unable to acquire AHB clock\n");
                return PTR_ERR(i2c_data->hclk);
        }
        
        i2c_data->mclk = devm_clk_get(&pdev->dev, "mod");
        
        if (IS_ERR(i2c_data->mclk)) {
                dev_err(&pdev->dev, "Unable to acquire module clock\n");
                return PTR_ERR(i2c_data->mclk);
        }
        
        i2c_data->rstc = devm_reset_control_get_exclusive(&pdev->dev, NULL);
        
        if (IS_ERR(i2c_data->rstc)) {
                dev_err(&pdev->dev, "can't get reset controller\n");
                return PTR_ERR(i2c_data->rstc);
        }
        
        /* copy regs offset to self data */
        memcpy(&i2c_data->reg_offsets, &suniv_i2c_regs_f1c100s,
               sizeof(struct suniv_i2c_regs));
               
        printk("%s: setting i2c adapter structure\n", __func__);
        /* setting i2c adapter structure */
        i2c_data->adapter.owner       = THIS_MODULE;
        i2c_data->adapter.algo        = &suniv_i2c_algo;
        i2c_data->adapter.algo_data   = NULL;
        i2c_data->adapter.dev.parent  = &pdev->dev;
        i2c_data->adapter.retries     = 3;
        i2c_data->adapter.timeout     = msecs_to_jiffies(20);
        i2c_data->adapter.nr          = pdev->id;
        i2c_data->adapter.dev.of_node = pdev->dev.of_node;
        strlcpy(i2c_data->adapter.name, SUNIV_CONTLR_NAME " adapter",
                sizeof(i2c_data->adapter.name));
                
        /* set privte data */
        printk("%s: set privte data\n", __func__);
        platform_set_drvdata(pdev, i2c_data);
        i2c_set_adapdata(&i2c_data->adapter, i2c_data);
        
        /* clks and reset */
        rc = clk_prepare_enable(i2c_data->hclk);
        
        if (rc) {
                dev_err(&pdev->dev, "can't enable AHB clock\n");
                return rc;
        }
        
        rc = clk_prepare_enable(i2c_data->mclk);
        
        if (rc) {
                dev_err(&pdev->dev, "can't enable module clock\n");
                return rc;
        }
        
        rc = reset_control_assert(i2c_data->rstc);
        
        if (rc) {
                dev_err(&pdev->dev, "can't assert the device from device");
                return rc;
        };
        
        /* A reset is inneed */
        printk("%s: reset the i2c controller\n", __func__);
        reset_control_reset(i2c_data->rstc);
        
        /* Get configs from device tree */
        rc = suniv_i2c_of_config(i2c_data, &pdev->dev);
        
        /* I2C bus hardware init */
        suniv_i2c_hw_init(i2c_data);
        
        /* Configure properties from dt */
        /*
        rc = suniv_i2c_of_config(i2c_data, &pdev->dev);
        if(rc)
            return rc;
        */
        
        /* request threaded irq */
        rc = devm_request_threaded_irq(&pdev->dev, i2c_data->irq,
                                       suniv_i2c_isr, suniv_i2c_isr_thread_fn,
                                       IRQF_NO_SUSPEND | IRQF_ONESHOT,
                                       SUNIV_CONTLR_NAME "adapter", i2c_data);
                                       
        if (rc) {
                dev_err(&i2c_data->adapter.dev,
                        "suniv: can't register intr handler irq%d: %d\n", i2c_data->irq, rc);
                return rc;
        }
        
        /* Add this adapter to system */
        printk("%s: adding adapter to system \n", __func__);
        rc = i2c_add_numbered_adapter(&i2c_data->adapter);
        if (rc != 0) {
                dev_err(&pdev->dev, "failed to add adapter\n");
                i2c_del_adapter(&i2c_data->adapter);
        }
        
        /* Create a sysfs interface */
        printk("%s: createing sysfs interface \n", __func__);
        rc = suniv_i2c_create_sysfs(&i2c_data->adapter.dev);
        
        return rc;
}

static int suniv_i2c_remove(struct platform_device *pdev)
{
        struct suniv_i2c_data *i2c_data = platform_get_drvdata(pdev);
        
        sysfs_remove_group(&i2c_data->adapter.dev.kobj, &suniv_i2c_attribute_group);
        i2c_del_adapter(&i2c_data->adapter);
        
        reset_control_deassert(i2c_data->rstc);
        clk_disable_unprepare(i2c_data->hclk);
        clk_disable_unprepare(i2c_data->mclk);
        
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
MODULE_DESCRIPTION("Allwinner suniv SoC family host bridge i2c adapter driver");
MODULE_LICENSE("GPL");
