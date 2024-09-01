/*
 Author: ZyZy

 This driver power on the ap64350 for arcturus board
 
*/

#include <linux/module.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/gpio.h>
#include <linux/fs.h>
#include <asm/uaccess.h>

#define IMX_GPIO_NR(port, index)		((((port)-1)*32)+((index)&31))


/* GPIOs related to AP64350 LTE connected in Arcturus*/
#define AP64350_VBAT       IMX_GPIO_NR(1,18) //PIN -120

#define DEVICE_NAME "ap64350"



static int ap64350_control_open(struct inode *inode, struct file *file)
{
    return 0;
}

static int ap64350_control_release(struct inode *inode, struct file *file)
{
    return 0;
}

static ssize_t ap64350_control_write(struct file *file, const char *buf, size_t count, loff_t *ppos)
{
    char action;
    int ret;
    
    if (count != 1)
        return -EINVAL;
    
    ret = copy_from_user(&action, buf, 1);
    if (ret)
        return -EFAULT;
    
    // Simulating reset button
    if (action == '1') {
        gpio_set_value(AP64350_VBAT, 1); // Set GPIO High 
    } else if (action == '0') {
        gpio_set_value(AP64350_VBAT, 0); // Set GPIO low
    } else {
        return -EINVAL;
    }

    return count;
}

static struct file_operations ap64350_gpio_control_fops = {
    .owner = THIS_MODULE,
    .open = ap64350_control_open,
    .release = ap64350_control_release,
    .write = ap64350_control_write,
};

static int __init ap64350_control_init(void)
{
    int ret;

    printk(KERN_INFO "AP64350 GPIO Control Module Start\n");

    ret = gpio_request(AP64350_VBAT, "AP64350_vbat_gpio_control");
    if (ret) {
        printk(KERN_ERR "Unable to request GPIO %d - AP64350_vbat_gpio_control\n", AP64350_VBAT);
        return ret;
    }

    ret = gpio_direction_output(AP64350_VBAT, 0);
    if (ret) {
        printk(KERN_ERR "Unable to set GPIO %d direction - AP64350_vbat_gpio_control\n", AP64350_VBAT);
        gpio_free(AP64350_VBAT);
        return ret;
    }

    msleep(30);

    printk(KERN_INFO "AP64350 GPIO AP64350_vbat_gpio_control configured\n");

    ret = register_chrdev(0, DEVICE_NAME, &ap64350_gpio_control_fops);
    if (ret < 0) {
        printk(KERN_ERR "Unable to register AP64350 Control device\n");
        gpio_free(AP64350_VBAT);
        return ret;
    }

    printk(KERN_INFO "AP64350 GPIO Control Module loaded\n");
    return 0;
}

static void __exit ap64350_control_exit(void)
{
    unregister_chrdev(0, DEVICE_NAME);
    gpio_free(AP64350_VBAT);
    printk(KERN_INFO "AP64350 GPIO Control Module unloaded\n");
}

module_init(ap64350_control_init);
module_exit(ap64350_control_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Aluisio Leonello Victal");
MODULE_DESCRIPTION("A simple AP64350 soc init/control module");