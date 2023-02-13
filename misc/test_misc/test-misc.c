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
#include <linux/fs.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/miscdevice.h>

/*
struct miscdevice {
  int minor;
  const char *name;
  struct file_operations *fops;
  struct miscdevice *next, *prev;
};
*/

#define DRV_NAME "test_misc"

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
    .owner = THIS_MODULE,
    .open = test_misc_open,
    .read = test_misc_read,
    .write = test_misc_write,
    .release = test_misc_release,
};

static struct miscdevice test_misc_dev = {
    .name = DRV_NAME,
    .fops = &test_misc_dev_fops,
};

static int __init test_misc_init(void)
{
    int rc;

    rc = misc_register(&test_misc_dev);
    if (rc) {
        pr_err("misc register failed!\n");
        return rc;
    }

    pr_info("misc device has been registered!\n");

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
