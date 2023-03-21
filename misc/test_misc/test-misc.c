/**
 * @file test-misc.c
 * @author IotaHydrae (writeforever@foxmail.com)
 * @brief
 * @version 0.1
 * @date 2023-02-10
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

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/fs.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/miscdevice.h>
#include <linux/list.h>
#include <linux/kfifo.h>
#include <linux/idr.h>
#include <linux/rbtree.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>

/*
struct miscdevice {
  int minor;
  const char *name;
  struct file_operations *fops;
  struct miscdevice *next, *prev;
};
*/
struct fox {
    char *name;
    unsigned long       tail_length;
    unsigned long       weight;
    bool                is_fantastic;
    struct list_head    list;
} fox = {
    .name = "peter",
    .tail_length = 40,
    .weight = 6,
    .is_fantastic = true,
    .list = LIST_HEAD_INIT(fox.list)
}, fox_brother = {
    .name = "red",
    .tail_length = 50,
    .weight = 8,
    .is_fantastic = false,
    .list = LIST_HEAD_INIT(fox_brother.list)
};

#define DRV_NAME "test_misc"

static void wonderful_people_tasklet_handler(struct tasklet_struct *t)
{
    pr_info("%s, Hello, I'm a wonderful people!", __func__);
}
DECLARE_TASKLET(wonderful_people, wonderful_people_tasklet_handler);

static void bad_people_work_handler(struct work_struct *w)
{
    printk("%s, Hello, I'm a bad people!", __func__);
}
DECLARE_WORK(bad_people, bad_people_work_handler);

static void mad_people_work_handler(struct work_struct *w)
{
    printk("%s, Hello, I'm a mad people!", __func__);
}
DECLARE_WORK(mad_people, mad_people_work_handler);

static int test_misc_open(struct inode *inode, struct file *file)
{
    pr_info("test misc device open\n");
    return 0;
}

static ssize_t test_misc_read(struct file *filep, char __user *buf,
                              size_t count, loff_t *f_ops)
{
    int rc;
    const char *hello_msg = "Hello, world!\n";
    pr_info("test misc device read\n");
    rc = copy_to_user(buf, hello_msg, strlen(hello_msg));
    return 0;
}

static ssize_t test_misc_write(struct file *filep, const char __user *buf,
                               size_t len, loff_t *ppos)
{
    pr_info("test misc device write\n");
    return len;
}
/*
** This function will be called when we close the Misc Device file
*/
static int test_misc_release(struct inode *inodep, struct file *filp)
{
    pr_info("test misc device release\n");
    return 0;
}

static const struct file_operations test_misc_dev_fops = {
    .owner   = THIS_MODULE,
    .open    = test_misc_open,
    .read    = test_misc_read,
    .write   = test_misc_write,
    .release = test_misc_release,
};

static struct miscdevice test_misc_dev = {
    .name = DRV_NAME,
    .fops = &test_misc_dev_fops,
};

static int __init test_misc_init(void)
{
    int i, rc;
    struct fox *f;
    
    struct kfifo fifo;
    char buf[16] = {0};
    char data[9] = {1, 1, 1, 1};
    
    /* test list */
    list_add(&fox_brother.list, &fox.list);
    list_for_each_entry(f, &fox.list, list) {
        printk("I'm %s, nice to meet you!\n", f->name);
        if (f->is_fantastic)
            printk("I'm also fantastic!\n\n");
    }
    
    /* test fifo */
    rc = kfifo_alloc(&fifo, PAGE_SIZE, GFP_KERNEL);
    
    rc = kfifo_init(&fifo, buf, ARRAY_SIZE(buf));
    rc = kfifo_in(&fifo, data, ARRAY_SIZE(data));
    if (rc != ARRAY_SIZE(data)) {
        printk("error! data may lose. byte overflowed: %d\n", ARRAY_SIZE(data) - rc);
    }
    
    rc = kfifo_in(&fifo, data, ARRAY_SIZE(data));
    if (rc != ARRAY_SIZE(data)) {
        printk("error! data may lose. byte overflowed: %d\n", ARRAY_SIZE(data) - rc);
    }
    
    for (i = 0; i < ARRAY_SIZE(buf); i++) {
        printk("%02d, %d", i, buf[i]);
    }
    
    while (kfifo_avail(&fifo)) {
        unsigned int val;
        int ret;
        
        ret = kfifo_out(&fifo, &val, sizeof(val));
        if (ret != sizeof(val))
            return -EINVAL;
            
        printk(KERN_INFO "%u\n", val);
    }
    
    /* test idr */
    struct idr id_huh;
    idr_init(&id_huh);
    
    /* test rbtree */
    struct rb_root root = RB_ROOT;
    
    pr_info("scheduling my tasklet!");
    tasklet_schedule(&wonderful_people);
    
    pr_info("scheduling my work!");
    schedule_work(&bad_people);

    struct workqueue_struct *keventd_wq = create_workqueue("events");
    queue_work(keventd_wq, &mad_people);

    flush_workqueue(keventd_wq);
    // return -1;
    rc = misc_register(&test_misc_dev);
    if (rc) {
        printk("misc register failed!\n");
        return rc;
    }
    
    printk("misc device has been registered!\n");
    
    return 0;
}

static void __exit test_misc_exit(void)
{
    misc_deregister(&test_misc_dev);
}

module_init(test_misc_init);
module_exit(test_misc_exit);

MODULE_AUTHOR("IotaHydrae <writeforever@foxmail.com>");
MODULE_DESCRIPTION("A virtual gpio_chip driver example");
MODULE_LICENSE("GPL");
