/*
 Author: ZyZy

 This driver power on the EG91 for canopus board
 
*/

#include <linux/module.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/gpio.h>
#include <linux/fs.h>
#include <asm/uaccess.h>

#define IMX_GPIO_NR(port, index)		((((port)-1)*32)+((index)&31))


/* GPIOs related to EG91 LTE connected in Arcturus*/
#define EG91_VBAT       IMX_GPIO_NR(4,14) //PIN -120
#define EG91_WDISABLE   IMX_GPIO_NR(3,23) //PIN 88-107
#define EG91_PWR        IMX_GPIO_NR(3,24) //PIN 89-108
#define EG91_RST        IMX_GPIO_NR(3,25) //PIN 90-109

#define DEVICE_NAME "eg91-control"



static int eg91_control_open(struct inode *inode, struct file *file)
{
    return 0;
}

static int eg91_control_release(struct inode *inode, struct file *file)
{
    return 0;
}

static ssize_t eg91_control_write(struct file *file, const char *buf, size_t count, loff_t *ppos)
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
        gpio_set_value(EG91_RST, 1); // Set GPIO High 
        msleep(700);
        gpio_set_value(EG91_RST, 0); // Set GPIO Low

    } else if (action == '0') {
        gpio_set_value(EG91_RST, 0); // Set GPIO high
    } else {
        return -EINVAL;
    }

    return count;
}

static struct file_operations eg91_gpio_control_fops = {
    .owner = THIS_MODULE,
    .open = eg91_control_open,
    .release = eg91_control_release,
    .write = eg91_control_write,
};

static int __init eg91_control_init(void)
{
    int ret;

    printk(KERN_INFO "EG91 GPIO Control Module Start\n");

    ret = gpio_request(EG91_VBAT, "eg91_vbat_gpio_control");
    if (ret) {
        printk(KERN_ERR "Unable to request GPIO %d - eg91_vbat_gpio_control\n", EG91_VBAT);
        return ret;
    }

    ret = gpio_direction_output(EG91_VBAT, 0);
    if (ret) {
        printk(KERN_ERR "Unable to set GPIO %d direction - eg91_vbat_gpio_control\n", EG91_VBAT);
        gpio_free(EG91_VBAT);
        return ret;
    }

    msleep(30);

    printk(KERN_INFO "EG91 GPIO eg91_vbat_gpio_control configured\n");

    ret = gpio_request(EG91_PWR, "eg91_pwr_gpio_control");
    if (ret) {
        printk(KERN_ERR "Unable to request GPIO %d - eg91_pwr_gpio_control\n", EG91_PWR);
        return ret;
    }

    ret = gpio_direction_output(EG91_PWR, 0);
    if (ret) {
        printk(KERN_ERR "Unable to set GPIO %d direction - eg91_pwr_gpio_control\n", EG91_PWR);
        gpio_free(EG91_VBAT);
        return ret;
    }
    printk(KERN_INFO "EG91 GPIO eg91_pwr_gpio_control configured\n");


    ret = gpio_request(EG91_RST, "eg91_rst_gpio_control");
    if (ret) {
        printk(KERN_ERR "Unable to request GPIO %d - eg91_rst_gpio_control\n", EG91_RST);
        return ret;
    }

    ret = gpio_direction_output(EG91_RST, 0);
    if (ret) {
        printk(KERN_ERR "Unable to set GPIO %d direction - eg91_rst_gpio_control\n", EG91_RST);
        gpio_free(EG91_PWR);
        gpio_free(EG91_VBAT);
        return ret;
    }
    printk(KERN_INFO "EG91 GPIO eg91_rst_gpio_control configured\n");

    // Simulating pwr button
    gpio_set_value(EG91_PWR, 1); // Set GPIO High
    msleep(550);
    gpio_set_value(EG91_PWR, 0); // Set GPIO Low


    ret = register_chrdev(0, DEVICE_NAME, &eg91_gpio_control_fops);
    if (ret < 0) {
        printk(KERN_ERR "Unable to register EG91 Control device\n");
        //gpio_free(EG91_VBAT);
        gpio_free(EG91_PWR);
        gpio_free(EG91_RST);
        return ret;
    }

    printk(KERN_INFO "EG91 GPIO Control Module loaded\n");
    return 0;
}

static void __exit eg91_control_exit(void)
{
    unregister_chrdev(0, DEVICE_NAME);
    //gpio_free(EG91_VBAT);
    gpio_free(EG91_PWR);
    gpio_free(EG91_RST);
    printk(KERN_INFO "EG91 GPIO Control Module unloaded\n");
}

module_init(eg91_control_init);
module_exit(eg91_control_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Aluisio Leonello Victal");
MODULE_DESCRIPTION("A simple EG91 soc init/control module");
