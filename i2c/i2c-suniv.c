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

#define SUNIV_I2C_BUS_CLOCK_NORMAL                      100000
#define SUNIV_I2C_BUS_CLOCK_FAST                        400000
#define SUNIV_I2C_BUS_CLOCK_DEFAULT                     SUNIV_I2C_BUS_CLOCK_NORMAL

#define SUNIV_I2C_ADDR(addr) ((0xff & addr) << 1)
#define SUNIV_I2C_REG_CLOCK_M(clkm) ((0xf & clkm) << 3)
#define SUNIV_I2C_REG_CLOCK_N(clkn) (0x7 & clkn)

#define SUNIV_I2C_SPEED_100K    (SUNIV_I2C_REG_CLOCK_M(2) | SUNIV_I2C_REG_CLOCK_N(11))
#define SUNIV_I2C_SPEED_400K    0

/* Suniv I2C control register bits */
#define SUNIV_I2C_REG_CONTROL_INT_EN                    BIT(7)  /* I2C Interrupt enable */
#define SUNIV_I2C_REG_CONTROL_BUS_EN                    BIT(6)  /* I2C Bus enable */
#define SUNIV_I2C_REG_CONTROL_M_STA                     BIT(5)  /* I2C Start signal flag */
#define SUNIV_I2C_REG_CONTROL_M_STP                     BIT(4)  /* I2C Stop singal flag */
#define SUNIV_I2C_REG_CONTROL_INT_FLAG                  BIT(3)  /* I2C Interrupt flag */
#define SUNIV_I2C_REG_CONTROL_A_ACK                     BIT(2)  /* I2C ACK singal flag */

#define SUNIV_I2C_REG_SRST_SOFT_RESET                   BIT(0)

/* Suniv I2C status register values
 */
#define SUNIV_I2C_BUS_STATUS_ERROR                      0x00    /* Bus error */
#define SUNIV_I2C_BUS_STATUS_START                      0x08    /* Start signal sent */
#define SUNIV_I2C_BUS_STATUS_REPEAT_START               0x10    /* Restart signal sent */
#define SUNIV_I2C_BUS_STATUS_ADDR_WR_ACK                0x18    /* Address + WR sent, ACK received */
#define SUNIV_I2C_BUS_STATUS_ADDR_WR_NOACK              0x20    /* Address + WR sent, ACK not received */
#define SUNIV_I2C_BUS_STATUS_MASTER_DATA_SEND_ACK       0x28    /* Data sent in master mode, ACK received */
#define SUNIV_I2C_BUS_STATUS_MASTER_DATA_SEND_NOACK     0x30    /* Data sent in master mode, ACK not received */
#define SUNIV_I2C_BUS_STATUS_ADDR_RD_ACK                0x40    /* Address + RD sent, ACK received */
#define SUNIV_I2C_BUS_STATUS_ADDR_RD_NOACK              0x48    /* Address + RD sent, ACK not received */
#define SUNIV_I2C_BUS_STATUS_MASTER_DATA_RECV_ACK       0x50    /* Data received in master mode, ACK transmitted */
#define SUNIV_I2C_BUS_STATUS_MASTER_DATA_RECV_NOACK     0x58    /* Data received in master mode, not ACK transmitted */
#define SUNIV_I2C_BUS_STATUS_SEC_ADDR_WR_ACK            0xd0

/* Suniv I2C message direction */
#define SUNIV_I2C_BUS_DIR_WR    0x00
#define SUNIV_I2C_BUS_DIR_RD    0x01

#define SUNIV_CONTLR_NAME "suniv_i2c"

enum {
        SUNIV_I2C_STATE_IDLE  = 0x00,
        SUNIV_I2C_STATE_START = 0x01,
        SUNIV_I2C_STATE_WRITE = 0x02,
        SUNIV_I2C_STATE_READ  = 0x04,
        SUNIV_I2C_STATE_STOP  = 0x08,

        /* special condition */
        SUNIV_I2C_STATE_READ_NOACK  = 0x10,
        SUNIV_I2C_STATE_NO_DEVICE   = 0x20,
        SUNIV_I2C_STATE_ERROR       = 0x40,
};

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

struct suniv_i2c {
        void __iomem            *base;          /* Memory address base of I2C controller */
        int                     irq;

        u32                     dir;            /* Direction of I2C msg */
        u32                     state;          /* current state of fsm */
        u32                     cntr_bits;      /* Control register value */
        u32                     addr;           /* Addr of current device */
        u32                     xaddr;          /* Extend addr of current device */
        u32                     byte_left;      /* Left byte need to be execute */
        u32                     byte_pos;       /* Position of current I2C msg buf */
        u32                     dummy_read;     /* Dummy read flag */
        u32                     bus_freq;       /* Current Bus Speed */

        atomic_t                rc;             /* Suceessed I2C msg count */

        struct suniv_i2c_regs   reg_offsets;

        struct i2c_msg          *msg;           /* Current I2C msg */
        struct i2c_msg          *msgs;          /* I2C msgs from userspace */
        int                     num_msgs;       /* Number of I2C msgs */
        struct i2c_adapter      adapter;
        struct clk              *hclk;
        struct clk              *mclk;

        struct completion       complete;

        struct reset_control    *rstc;
};

struct suniv_i2c_regs suniv_i2c_regs_f1c100s = {
        .addr  = 0x00,  /* addr and xaddr is used in slave mode */
        .xaddr = 0x04,
        .data  = 0x08,
        .cntr  = 0x0c,
        .stat  = 0x10,
        .ccr   = 0x14,
        .srst  = 0x18,
        .efr   = 0x1c,
        .lcr   = 0x20,
};

static inline u32 suniv_i2c_read(struct suniv_i2c *i2c_dev, u32 reg)
{
        return readl(i2c_dev->base + reg);
}

static inline void suniv_i2c_write(struct suniv_i2c *i2c_dev, u32 reg, u32 val)
{
        writel(val, i2c_dev->base + reg);
}

static inline void suniv_i2c_soft_reset(struct suniv_i2c *i2c_dev)
{
        suniv_i2c_write(i2c_dev, i2c_dev->reg_offsets.srst,
                        SUNIV_I2C_REG_SRST_SOFT_RESET);
}


static ssize_t suniv_i2c_sysfs_dump_register(struct device *dev,
                                             struct device_attribute *attr, char *buf)
{
        struct suniv_i2c *i2c_dev = dev->driver_data;

        printk("-------------------> %s <-------------------\n", __func__);

        printk("addr  : 0x%02x \t\t xaddr  : 0x%02x \t\t data  : 0x%02x\n",
               suniv_i2c_read(i2c_dev, i2c_dev->reg_offsets.addr),
               suniv_i2c_read(i2c_dev, i2c_dev->reg_offsets.xaddr),
               suniv_i2c_read(i2c_dev, i2c_dev->reg_offsets.data));

        printk("cntr  : 0x%02x \t\t stat	: 0x%02x \t\t ccr   : 0x%02x \n",
               suniv_i2c_read(i2c_dev, i2c_dev->reg_offsets.cntr),
               suniv_i2c_read(i2c_dev, i2c_dev->reg_offsets.stat),
               suniv_i2c_read(i2c_dev, i2c_dev->reg_offsets.ccr));

        printk("srst  : 0x%02x \t\t efr	: 0x%02x \t\t lcr   : 0x%02x \n",
               suniv_i2c_read(i2c_dev, i2c_dev->reg_offsets.srst),
               suniv_i2c_read(i2c_dev, i2c_dev->reg_offsets.efr),
               suniv_i2c_read(i2c_dev, i2c_dev->reg_offsets.lcr));

        return 0;
}
static DEVICE_ATTR(dump_register, S_IRUSR, suniv_i2c_sysfs_dump_register, NULL);

static inline void suniv_i2c_set_bus_freq(struct suniv_i2c *i2c_dev, u32 bus_freq)
{
        switch (bus_freq) {
        case SUNIV_I2C_BUS_CLOCK_NORMAL:
                i2c_dev->bus_freq = SUNIV_I2C_SPEED_100K;
                break;
        case SUNIV_I2C_BUS_CLOCK_FAST:
                i2c_dev->bus_freq = SUNIV_I2C_SPEED_400K;
                break;
        default:
                dev_warn(&i2c_dev->adapter.dev, "given freq not supported!");
                i2c_dev->bus_freq = SUNIV_I2C_SPEED_100K;
                break;
        }
}

static ssize_t suniv_i2c_sysfs_set_bus_freq(struct device *dev,
                                            struct device_attribute *attr, char *buf)
{
        struct suniv_i2c *i2c_dev = dev->driver_data;
        printk("bus freq : %d\n", i2c_dev->bus_freq);
        return 0;
}
static DEVICE_ATTR(set_bus_freq, S_IRUSR, suniv_i2c_sysfs_set_bus_freq, NULL);

static struct attribute *suniv_i2c_sysfs_attrs[] = {
        &dev_attr_dump_register.attr,
        &dev_attr_set_bus_freq.attr,
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

        return rc;
}

static int __maybe_unused suniv_i2c_of_config(struct suniv_i2c *i2c_dev,
                                              struct device *dev)
{
        int rc = 0;
        struct device_node *np = dev->of_node;
        u32 bus_freq;

        rc = of_property_read_u32(np, "clock-frequency", &bus_freq);
        if (rc)
                bus_freq = SUNIV_I2C_BUS_CLOCK_DEFAULT;

        suniv_i2c_set_bus_freq(i2c_dev, bus_freq);

        return 0;
}

static inline void suniv_i2c_hw_init(struct suniv_i2c *i2c_dev)
{
        pr_debug("%s, software resetting i2c adapter ...\n", __func__);
        suniv_i2c_soft_reset(i2c_dev);

        pr_debug("%s, setting bus clock ...\n", __func__);
        /* set the bus clock, temporarily set to 100Kbit/s */
        suniv_i2c_write(i2c_dev, i2c_dev->reg_offsets.ccr, i2c_dev->bus_freq);

        /* clear registers */
        suniv_i2c_write(i2c_dev, i2c_dev->reg_offsets.addr, 0);
        suniv_i2c_write(i2c_dev, i2c_dev->reg_offsets.xaddr, 0);
        suniv_i2c_write(i2c_dev, i2c_dev->reg_offsets.cntr, 0);

        pr_debug("%s, enable i2c bus ...\n", __func__);
        /* enable bus */
        i2c_dev->cntr_bits |= SUNIV_I2C_REG_CONTROL_BUS_EN;
        suniv_i2c_write(i2c_dev, i2c_dev->reg_offsets.cntr,
                        i2c_dev->cntr_bits | SUNIV_I2C_REG_CONTROL_M_STP);
}

static void suniv_i2c_send_start(struct suniv_i2c *i2c_dev,
                                 struct i2c_msg *msg)
{
        /* Store msg info here, used in isr */
        i2c_dev->msg        = msg;
        i2c_dev->byte_left  = msg->len;
        i2c_dev->byte_pos   = 0;
        i2c_dev->dummy_read = 1;

        pr_debug("%s, addr:0x%x\n", __func__, i2c_dev->msgs->addr);
        pr_debug("%s, byte left:%d\n", __func__, i2c_dev->byte_left);

        /* Transfer direction set */
        if (msg->flags & I2C_M_RD)
                i2c_dev->dir = SUNIV_I2C_BUS_DIR_RD;
        else
                i2c_dev->dir = SUNIV_I2C_BUS_DIR_WR;

        /* Set cntr register bits, like enable intr, etc. */
        i2c_dev->cntr_bits |= SUNIV_I2C_REG_CONTROL_INT_EN;

        /* If it's a 10 bit address */
        if (msg->flags & I2C_M_TEN) {
                i2c_dev->addr = SUNIV_I2C_ADDR(msg->addr) | i2c_dev->dir;
                i2c_dev->xaddr = (u32)msg->addr & 0xff;
        } else {
                i2c_dev->addr = SUNIV_I2C_ADDR(msg->addr) | i2c_dev->dir;
                i2c_dev->xaddr = 0;
        }

        /* Write into cntr register */
        pr_debug("%s, sending a i2c start signal\n", __func__);
        suniv_i2c_write(i2c_dev, i2c_dev->reg_offsets.cntr,
                        i2c_dev->cntr_bits | SUNIV_I2C_REG_CONTROL_M_STA);
}

static inline void suniv_i2c_send_stop(struct suniv_i2c *i2c_dev)
{
        suniv_i2c_write(i2c_dev, i2c_dev->reg_offsets.cntr,
                        i2c_dev->cntr_bits |
                        SUNIV_I2C_REG_CONTROL_M_STP);

        complete(&i2c_dev->complete);
}

static inline void suniv_i2c_clear_irq(struct suniv_i2c *i2c_dev)
{
        i2c_dev->cntr_bits &= ~(SUNIV_I2C_REG_CONTROL_INT_FLAG);
        suniv_i2c_write(i2c_dev, i2c_dev->reg_offsets.cntr, i2c_dev->cntr_bits);
}

static void suniv_i2c_do_action(struct suniv_i2c *i2c_dev)
{
        pr_debug("%s\n", __func__);
        switch (i2c_dev->state) {
        case SUNIV_I2C_STATE_START:
                suniv_i2c_write(i2c_dev, i2c_dev->reg_offsets.data, i2c_dev->addr);
                suniv_i2c_clear_irq(i2c_dev);
                break;
        case SUNIV_I2C_STATE_WRITE:
                pr_debug("%s, 0x%02x : SUNIV_I2C_BUS_STATUS_MASTER_DATA_ACK : Data byte has been sent, %d left",
                         __func__, SUNIV_I2C_BUS_STATUS_MASTER_DATA_SEND_ACK, i2c_dev->byte_left);
                if (i2c_dev->byte_left == 0) {
                        atomic_inc(&i2c_dev->rc);
                        suniv_i2c_send_stop(i2c_dev);
                } else {
                        pr_debug("%s, writing 0x%02x to slave\n", __func__, i2c_dev->msg->buf[i2c_dev->byte_pos]);
                        suniv_i2c_write(i2c_dev, i2c_dev->reg_offsets.data,
                                        i2c_dev->msg->buf[i2c_dev->byte_pos++]);
                        suniv_i2c_clear_irq(i2c_dev);
                        i2c_dev->byte_left--;
                }
                break;
        case SUNIV_I2C_STATE_READ:
                /* copy received byte in register into i2c msg */
                if (i2c_dev->dummy_read) {
                        suniv_i2c_write(i2c_dev, i2c_dev->reg_offsets.cntr,
                                        i2c_dev->cntr_bits | SUNIV_I2C_REG_CONTROL_A_ACK);
                        i2c_dev->dummy_read = 0;
                        break;
                }

                if (i2c_dev->byte_left == 0) {
                        pr_debug("%s, sending a stop signal\n", __func__);

                        //i2c_dev->cntr_bits &= ~SUNIV_I2C_REG_CONTROL_INT_EN;
                        atomic_inc(&i2c_dev->rc);
                        suniv_i2c_send_stop(i2c_dev);
                } else {
                        i2c_dev->msg->buf[i2c_dev->byte_pos] = suniv_i2c_read(i2c_dev,
                                                                              i2c_dev->reg_offsets.data);
                        pr_debug("%s, read 0x%02x from slave\n", __func__,
                                 i2c_dev->msg->buf[i2c_dev->byte_pos]);
                        suniv_i2c_clear_irq(i2c_dev);
                        i2c_dev->byte_left--;
                        i2c_dev->byte_pos++;
                }
                break;

        /* Handle special condition */
        case SUNIV_I2C_STATE_READ_NOACK:
                atomic_inc(&i2c_dev->rc);
                suniv_i2c_send_stop(i2c_dev);
                break;

        case SUNIV_I2C_STATE_STOP:
        case SUNIV_I2C_STATE_NO_DEVICE:
                suniv_i2c_send_stop(i2c_dev);
                break;

        case SUNIV_I2C_STATE_ERROR:
        default:
                suniv_i2c_hw_init(i2c_dev);
                suniv_i2c_send_stop(i2c_dev);
                i2c_recover_bus(&i2c_dev->adapter);
                break;
        }

        i2c_dev->state = SUNIV_I2C_STATE_IDLE;
}

static void suniv_i2c_fsm(struct suniv_i2c *i2c_dev)
{
        unsigned long              status_stat;

        status_stat = suniv_i2c_read(i2c_dev, i2c_dev->reg_offsets.stat);
        switch (status_stat) {
        /* Error interrupt */
        case SUNIV_I2C_BUS_STATUS_ERROR:                /* 0x00 */
                i2c_dev->state = SUNIV_I2C_STATE_ERROR;
                break;

        /* Start condition interrupt */
        case SUNIV_I2C_BUS_STATUS_START:                 /* 0x08 */
        case SUNIV_I2C_BUS_STATUS_REPEAT_START:          /* 0x10 */
                i2c_dev->state = SUNIV_I2C_STATE_START;
                break;

        /* Write to slave */
        case SUNIV_I2C_BUS_STATUS_ADDR_WR_ACK:           /* 0x18 */
        /* TODO: check if it's a 10 bit addr */
        case SUNIV_I2C_BUS_STATUS_SEC_ADDR_WR_ACK:       /* 0xd0 */
        case SUNIV_I2C_BUS_STATUS_MASTER_DATA_SEND_ACK:  /* 0x28 */
                i2c_dev->state = SUNIV_I2C_STATE_WRITE;
                break;

        /* Read from slave */
        case SUNIV_I2C_BUS_STATUS_ADDR_RD_ACK:            /* 0x40 */
        case SUNIV_I2C_BUS_STATUS_MASTER_DATA_RECV_ACK:   /* 0x50 */
                i2c_dev->state = SUNIV_I2C_STATE_READ;
                break;

        /* Data byte received in master mode, not ACK transmitted */
        case SUNIV_I2C_BUS_STATUS_MASTER_DATA_RECV_NOACK: /* 0x58 */
                i2c_dev->state = SUNIV_I2C_STATE_READ_NOACK;
                break;

        /* Non device responsed */
        case SUNIV_I2C_BUS_STATUS_ADDR_WR_NOACK:           /* 0x20 */
        case SUNIV_I2C_BUS_STATUS_MASTER_DATA_SEND_NOACK:  /* 0x30 */
        case SUNIV_I2C_BUS_STATUS_ADDR_RD_NOACK:           /* 0x48 */
                i2c_dev->state = SUNIV_I2C_STATE_NO_DEVICE;
                break;

        default:
                i2c_dev->state = SUNIV_I2C_STATE_ERROR;
                break;
        }
}

static irqreturn_t suniv_i2c_isr(int irq, void *dev_id)
{
        struct suniv_i2c           *i2c_dev = dev_id;

        while (!(suniv_i2c_read(i2c_dev, i2c_dev->reg_offsets.cntr) &
                 SUNIV_I2C_REG_CONTROL_INT_FLAG));

        suniv_i2c_fsm(i2c_dev);
        suniv_i2c_do_action(i2c_dev);

        return IRQ_HANDLED;
}

static int suniv_i2c_do_msgs(struct suniv_i2c *i2c_dev)
{
        int                     i;
        long                    time_left = 0;
        int                     num = i2c_dev->num_msgs;
        struct i2c_msg          *msgs = i2c_dev->msgs;

        atomic_set(&i2c_dev->rc, 0);
        i2c_dev->state = SUNIV_I2C_STATE_IDLE;

        for (i = 0; i < num; i++) {
                pr_debug("%s, sending msg : %d", __func__, i);

                /* reinit the completion be ready for next */
                reinit_completion(&i2c_dev->complete);

                /* Send a start signal and waiting for interrupt occured */
                suniv_i2c_send_start(i2c_dev, &msgs[i]);

                /* After call this, if controller can't receive ack or error occured,
                 * here will be timeout, and a negative value returned
                 */
                time_left = wait_for_completion_timeout(&i2c_dev->complete,
                                                        i2c_dev->adapter.timeout);

                pr_debug("%s, time left : %d ms", __func__, jiffies_to_msecs(time_left));

                if (!time_left) {
                        /* controller may died here */
                        pr_debug("%s, i2c msg time out : %d", __func__, (int)time_left);
                        suniv_i2c_hw_init(i2c_dev);
                        atomic_set(&i2c_dev->rc, -ETIMEDOUT);
                }

        }

        return atomic_read(&i2c_dev->rc);
}

/*
 * master_xfer should return the number of messages successfully
 * processed, or a negative value on error
 */
static int suniv_i2c_xfer(struct i2c_adapter *adap, struct i2c_msg *msgs, int num)
{
        struct suniv_i2c *i2c_dev = i2c_get_adapdata(adap);
        int                     rc = false;

        i2c_dev->msgs = msgs;
        i2c_dev->num_msgs = num;

        /* When the CPU host wants to start a bus transfer,
         * it initiates a bus START to enter the master mode by setting IM_STA bit
         * in the 2WIRE_CNTR register to high (before it must be low).
         * The TWI will assert INT line and INT_FLAG to indicate a
         * completion for the START condition and each consequent byte transfer.
         * At each interrupt, the micro-processor needs to check the 2WIRE_STAT register for current status.
         * A transfer has to be concluded with STOP condition by setting M_STP
         * bit high.
         *
         * Actually it works like a state machine, when you
         * a start signal sent, the INT_FLAG in control register
         * will be set and go into the isr, then read the value
         * in status register and according to it do the corresponding service action.
         */

        pr_debug("%s,  %d msg need to be transfer", __func__, i2c_dev->num_msgs);

        /* i2c_dev->cntr_bits = suniv_i2c_read(i2c_dev, i2c_dev->reg_offsets.cntr); */
        pr_debug("%s, i2c_dev->cntr_bits : 0x%x\n", __func__, i2c_dev->cntr_bits);
        suniv_i2c_hw_init(i2c_dev);
        /* do single i2c msg whatever read or write */
        if (num == 1)
                rc = true;

        /* if it's a write and read ops */
        if (num == 2 && !(msgs[0].flags & I2C_M_RD)
            && (msgs[1].flags & I2C_M_RD))
                rc = true;

        /* handle msgs */
        if (rc)
                rc = suniv_i2c_do_msgs(i2c_dev);
        else
                pr_debug(KERN_WARNING "%s, required ops not supported!\n", __func__);


        pr_debug("%s, successed msg count : %d", __func__, atomic_read(&i2c_dev->rc));
        return rc;
}

/* To determine what the adapter supports */
static u32 suniv_i2c_functionality(struct i2c_adapter *adap)
{
        return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL;
}

/* Some smbus function could simulate by the i2c master_xfer func */
static const struct i2c_algorithm suniv_i2c_algo = {
        .master_xfer   = suniv_i2c_xfer,
        .functionality = suniv_i2c_functionality,
};

static int suniv_i2c_probe(struct platform_device *pdev)
{
        int rc;
        struct suniv_i2c *i2c_dev;

        pr_debug("%s\n", __func__);

        if (!pdev->dev.of_node)
                return -ENODEV;

        pr_debug("%s: alloc memory of i2c_dev\n", __func__);
        i2c_dev = devm_kzalloc(&pdev->dev, sizeof(struct suniv_i2c),
                               GFP_KERNEL);

        if (!i2c_dev)
                return -ENOMEM;

        pr_debug("%s: ioremap the bus register base addr\n", __func__);
        i2c_dev->base = devm_platform_ioremap_resource(pdev, 0);
        if (IS_ERR(i2c_dev->base))
                return PTR_ERR(i2c_dev->base);

        pr_debug("%s, i2c reg base: %p\n", __func__, i2c_dev->base);

        /* get clocks */
        i2c_dev->hclk = devm_clk_get(&pdev->dev, "ahb");
        if (IS_ERR(i2c_dev->hclk)) {
                dev_err(&pdev->dev, "Unable to acquire AHB clock\n");
                return PTR_ERR(i2c_dev->hclk);
        }

        i2c_dev->mclk = devm_clk_get(&pdev->dev, "mod");
        if (IS_ERR(i2c_dev->mclk)) {
                dev_err(&pdev->dev, "Unable to acquire module clock\n");
                return PTR_ERR(i2c_dev->mclk);
        }

        i2c_dev->rstc = devm_reset_control_get_exclusive(&pdev->dev, NULL);
        if (IS_ERR(i2c_dev->rstc)) {
                dev_err(&pdev->dev, "can't get reset controller\n");
                return PTR_ERR(i2c_dev->rstc);
        }

        /* copy regs offset to self data */
        memcpy(&i2c_dev->reg_offsets, &suniv_i2c_regs_f1c100s,
               sizeof(struct suniv_i2c_regs));

        pr_debug("%s: setting i2c adapter structure\n", __func__);
        /* setting i2c adapter structure */
        i2c_dev->adapter.owner       = THIS_MODULE;
        i2c_dev->adapter.algo        = &suniv_i2c_algo;
        i2c_dev->adapter.algo_data   = NULL;
        i2c_dev->adapter.dev.parent  = &pdev->dev;
        i2c_dev->adapter.retries     = 3;
        i2c_dev->adapter.timeout     = msecs_to_jiffies(250);
        i2c_dev->adapter.nr          = pdev->id;
        i2c_dev->adapter.dev.of_node = pdev->dev.of_node;
        snprintf(i2c_dev->adapter.name, sizeof(i2c_dev->adapter.name),
                 SUNIV_CONTLR_NAME " bus%d", i2c_dev->adapter.nr);

        /* set private data */
        pr_debug("%s: set privte data\n", __func__);
        platform_set_drvdata(pdev, i2c_dev);
        i2c_set_adapdata(&i2c_dev->adapter, i2c_dev);

        /* other initialze ops */
        init_completion(&i2c_dev->complete);

        /* clks and reset */
        rc = clk_prepare_enable(i2c_dev->hclk);
        if (rc) {
                dev_err(&pdev->dev, "can't enable AHB clock!\n");
                return rc;
        }

        rc = clk_prepare_enable(i2c_dev->mclk);
        if (rc) {
                dev_err(&pdev->dev, "can't enable module clock!\n");
                return rc;
        }

        rc = reset_control_assert(i2c_dev->rstc);
        if (rc) {
                dev_err(&pdev->dev, "can't assert the device from device!\n");
                return rc;
        };

        /* A reset is in need */
        pr_debug("%s: reset the i2c controller\n", __func__);
        reset_control_reset(i2c_dev->rstc);

        /* get device tree configs */
        rc = suniv_i2c_of_config(i2c_dev, &pdev->dev);
        if (rc) {
                dev_err(&pdev->dev, "of config failed!\n");
                return rc;
        }

        /* I2C bus hardware init */
        suniv_i2c_hw_init(i2c_dev);

        /* Add this adapter to system */
        pr_debug("%s: adding adapter to system \n", __func__);
        rc = i2c_add_numbered_adapter(&i2c_dev->adapter);
        if (rc != 0) {
                dev_err(&pdev->dev, "failed to add adapter\n");
                i2c_del_adapter(&i2c_dev->adapter);
        }

        i2c_dev->irq = platform_get_irq(pdev, 0);
        if (i2c_dev->irq < 0) {
                printk("%s, can't get irq\n", __func__);
                return i2c_dev->irq;
        }
        pr_debug("%s: Got irq number %d from device\n", __func__, i2c_dev->irq);

        rc = devm_request_irq(&pdev->dev, i2c_dev->irq, suniv_i2c_isr,
                              IRQF_NO_SUSPEND | IRQF_ONESHOT,
                              SUNIV_CONTLR_NAME "adapter", i2c_dev);

        if (rc) {
                dev_err(&i2c_dev->adapter.dev,
                        "suniv: can't register intr handler irq : %d\n", i2c_dev->irq);
                return rc;
        }

        /* Create a sysfs interface */
        pr_debug("%s: createing sysfs interface \n", __func__);
        rc = suniv_i2c_create_sysfs(&i2c_dev->adapter.dev);
        if (rc) {
                dev_err(&pdev->dev, "%s, create sysfs group failed!\n", __func__);
                sysfs_remove_group(&i2c_dev->adapter.dev.kobj, &suniv_i2c_attribute_group);
                rc = -ENOMEM;
        }

        return rc;
}

static int suniv_i2c_remove(struct platform_device *pdev)
{
        struct suniv_i2c *i2c_dev = platform_get_drvdata(pdev);

        sysfs_remove_group(&i2c_dev->adapter.dev.kobj, &suniv_i2c_attribute_group);
        i2c_del_adapter(&i2c_dev->adapter);

        reset_control_deassert(i2c_dev->rstc);
        clk_disable_unprepare(i2c_dev->hclk);
        clk_disable_unprepare(i2c_dev->mclk);

        return 0;
}

static int __maybe_unused suniv_i2c_runtime_suspend(struct device *dev)
{
        struct suniv_i2c *i2c_dev = dev_get_drvdata(dev);

        clk_disable_unprepare(i2c_dev->mclk);
        clk_disable_unprepare(i2c_dev->hclk);

        /* TODO: select gpio pinctrl function */

        return 0;
}

static int __maybe_unused suniv_i2c_runtime_resume(struct device *dev)
{
        struct suniv_i2c *i2c_dev = dev_get_drvdata(dev);
        int rc;

        rc = clk_prepare_enable(i2c_dev->mclk);
        if (rc)
                return rc;

        rc = clk_prepare_enable(i2c_dev->hclk);
        if (rc)
                return rc;

        /* TODO: select i2c pinctrl function */

        suniv_i2c_hw_init(i2c_dev);

        return 0;
}

#if CONFIG_PM
static const struct dev_pm_ops suniv_i2c_pm_ops = {
        SET_RUNTIME_PM_OPS(suniv_i2c_runtime_suspend,
                           suniv_i2c_runtime_resume, NULL)
};
#else
static const struct dev_pm_ops suniv_i2c_pm_ops = {
        SET_RUNTIME_PM_OPS(NULL, NULL, NULL)
};
#endif

#if CONFIG_OF
static const struct of_device_id suniv_i2c_of_match_table[] = {
        {.compatible = "allwinner,suniv-f1c100s-i2c", .data = &suniv_i2c_regs_f1c100s},
        {.compatible = "allwinner,suniv-f1c200s-i2c", .data = &suniv_i2c_regs_f1c100s},
        { /* KEEP THIS */ },
};
MODULE_DEVICE_TABLE(of, suniv_i2c_of_match_table);
#endif

static struct platform_driver suniv_i2c_driver = {
        .probe  = suniv_i2c_probe,
        .remove = suniv_i2c_remove,
        .driver = {
                .name           = "suniv_i2c",
                .of_match_table = of_match_ptr(suniv_i2c_of_match_table),
                .pm             = &suniv_i2c_pm_ops,
        },
};

module_platform_driver(suniv_i2c_driver);

MODULE_AUTHOR("Mark A. Greer <mgreer@mvista.com>");
MODULE_AUTHOR("IotaHydrae <writeforever@foxmail.com>");
MODULE_DESCRIPTION("Allwinner suniv SoC family host bridge i2c adapter driver");
MODULE_LICENSE("GPL");
