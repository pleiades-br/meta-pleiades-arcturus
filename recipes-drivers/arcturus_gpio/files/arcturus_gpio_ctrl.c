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

#define FILE_CMD_SIZE 32
#define FILE_IC_NAME 12
#define FILE_IC_CMD 3

#define INPUT_DIRECTION 0
#define OUTPUT_DIRECTION 1

#define IC_OFF 0
#define IC_ON 1


#define RETURN_OK 0
#define RETURN_ERROR -1

/* GPIOs related to EG91 LTE connected in Arcturus*/
#define EG91_VBAT       IMX_GPIO_NR(4,14) //PIN -120
#define EG91_WDISABLE   IMX_GPIO_NR(3,23) //PIN 88-107
#define EG91_PWR        IMX_GPIO_NR(3,24) //PIN 89-108
#define EG91_RST        IMX_GPIO_NR(3,25) //PIN 90-109
#define EG91_GPS        IMX_GPIO_NR(1,5) //PIN 45
#define AP64350_VBAT    IMX_GPIO_NR(1,18) //PIN 54
#define PT100_RST       IMX_GPIO_NR(5,2) //PIN 13

#define DEVICE_NAME "arcturus-gpio-control"


static unsigned short int _eg91_state = IC_OFF;
static unsigned short int _pt100_state = IC_OFF;
static unsigned short int _ap64350_state = IC_OFF;


static void arcturus_reset_ic(const int gpio, 
                                const int start_value, 
                                const int end_value, 
                                const int sleep_time)
{
    gpio_set_value(gpio, start_value); 
    msleep(sleep_time);
    gpio_set_value(gpio, end_value); 
}


static int arcturus_control_open(struct inode *inode, struct file *file)
{
    return 0;
}

static int arcturus_control_release(struct inode *inode, struct file *file)
{
    return 0;
}

static ssize_t arcturus_control_write(struct file *file, const char *userbuf, size_t count, loff_t *offset)
{
    char icname[FILE_IC_NAME + 1]  = {0};
    char cmd[FILE_IC_CMD + 1] = {0};
    char buf[FILE_CMD_SIZE] = {0};
    int ret;

    count = min_t(size_t, count, (sizeof(buf)-1));
	if (copy_from_user(buf, userbuf, count))
		return -EFAULT;

	buf[count] = 0;

    ret = sscanf(buf, "%s %s", icname, cmd);
    if (ret != 2) {
        return -EFAULT;
    }

    if (!strcmp(icname, "EG91")) {
        if(!strcmp(cmd, "PWR")) {
            printk(KERN_INFO "EG91 Power %s requested", _eg91_state == IC_OFF? "on" : "off");
            _eg91_state == IC_OFF? gpio_set_value(EG91_PWR, 0) : gpio_set_value(EG91_PWR, 1);
            _eg91_state = (_eg91_state == IC_OFF? IC_ON : IC_OFF);
        } else if(!strcmp(cmd, "RST")) {
            printk(KERN_INFO "EG91 reset operation requested");
            /* 
                If the power is off and a reset requested by user, 
                lets turn on the EG91 
            */
            if (_eg91_state == IC_OFF) {
                gpio_set_value(EG91_PWR, 0);
                _eg91_state = IC_ON;
            }
            arcturus_reset_ic(EG91_PWR, 1, 0, 550);
        } else if(!strcmp(cmd, "GPS_ON")) {
            printk(KERN_INFO "EG91 GPS ON operation requested");
            gpio_set_value(EG91_GPS, 0);
        } else if(!strcmp(cmd, "GPS_OFF")) {
            printk(KERN_INFO "EG91 GPS OFF operation requested");
            gpio_set_value(EG91_GPS, 1);
        } else {
            printk(KERN_ERR "EG91 operation requested not valid %s", cmd);
        }
    } else if(!strcmp(icname, "PT100")) {
        if(!strcmp(cmd, "RST")) {
            arcturus_reset_ic(PT100_RST, 1, 0, 200);
        } else {
            printk(KERN_ERR "PT100 operation requested not valid %s", cmd);
        }
    } else if(!strcmp(icname, "AP64350")) {
        if(!strcmp(cmd, "PWR")) {
            printk(KERN_INFO "AP64350 Power %s requested", _ap64350_state == IC_OFF? "on" : "off");
            _ap64350_state == IC_OFF? gpio_set_value(AP64350_VBAT, 1) : gpio_set_value(AP64350_VBAT, 0);
            _ap64350_state = (_ap64350_state == IC_OFF? IC_ON : IC_OFF);
        } else {
            printk(KERN_ERR "AP64350 operation requested not valid %s", cmd);
        }
    } else {
        printk(KERN_ERR "No IC name %s found", icname);
    }

    return count;
}

static struct file_operations arcturus_gpio_control_fops = {
    .owner = THIS_MODULE,
    .open = arcturus_control_open,
    .release = arcturus_control_release,
    .write = arcturus_control_write,
};

static int arcturus_gpio_conf(const int gpio, 
                                const char *name, 
                                const unsigned int direction, 
                                const unsigned int value)
{
    int ret = RETURN_OK;
    
    ret = gpio_request(gpio, "name");
    if (ret) {
        printk(KERN_ERR "Unable to request GPIO %d - %s\n", gpio, name);
        return ret;
    }

    if (direction == INPUT_DIRECTION) {
        ret = gpio_direction_input(gpio);
    } else {
        ret = gpio_direction_output(gpio, value);
    }

    if (ret) {
            printk(KERN_ERR "Unable to set GPIO %d to %s direction - %s\n", 
                    gpio, (direction == INPUT_DIRECTION ? "Input" : "Output"), name);
            gpio_free(gpio);
    }

    return ret;
}



/* ADS122C04 */
static int arcturus_pt100_init(void)
{
    if (arcturus_gpio_conf(PT100_RST, "PT100_rst_gpio_control", OUTPUT_DIRECTION, 0)) {
        goto err;
    }

    msleep(30);

    // Simulating pwr button
    arcturus_reset_ic(PT100_RST, 1, 0, 200);
   
    printk(KERN_INFO "PT100 GPIO PT100_rst_gpio_control configured\n");
    _pt100_state = IC_ON;
    return 0;
    
    err:
        gpio_free(PT100_RST);
        return RETURN_ERROR;
}


static int arcturus_ap64350_init(void) 
{

    if (arcturus_gpio_conf(AP64350_VBAT, "AP64350_vbat_gpio_control", OUTPUT_DIRECTION, 0)) {
        goto err;
    }

    msleep(30);
    
    printk(KERN_INFO "AP64350 GPIO AP64350_vbat_gpio_control configured\n");
    _ap64350_state = IC_ON;
    return 0;
    
    err:
        gpio_free(AP64350_VBAT);
        return RETURN_ERROR;
}

static int arcturus_eg91_init(void)
{
    if (arcturus_gpio_conf(EG91_VBAT, "eg91_vbat_gpio_control", OUTPUT_DIRECTION, 0)) {
        goto err;
    }

    msleep(30);

    if (arcturus_gpio_conf(EG91_PWR, "eg91_pwr_gpio_control", OUTPUT_DIRECTION, 0)) {
        goto err;
    }

    if (arcturus_gpio_conf(EG91_RST, "eg91_rst_gpio_control", OUTPUT_DIRECTION, 0)) {
        goto err;
    }

    // Simulating pwr button
    arcturus_reset_ic(EG91_PWR, 1, 0, 550);

    printk(KERN_INFO "EG91 GPIO Control Module loaded\n");
    _eg91_state = IC_ON;

    if (arcturus_gpio_conf(EG91_GPS, "eg91_rst_gpio_control", OUTPUT_DIRECTION, 0)) {
        goto err;
    }
    return 0;

    err:
        gpio_free(EG91_VBAT);
        gpio_free(EG91_PWR);
        gpio_free(EG91_VBAT);
        gpio_free(EG91_GPS);
        return RETURN_ERROR;
}


static int __init arcturus_control_init(void)
{
    int ret = RETURN_OK;

    if (arcturus_eg91_init()) {
        printk(KERN_INFO "EG91 GPIO Control failed to initialize\n");
    }

    if (arcturus_ap64350_init()) {
        printk(KERN_INFO "AP64350 GPIO Control failed to initialize\n");
    }

    if (arcturus_pt100_init()) {
        printk(KERN_INFO "PT100 GPIO Control failed to initialize\n");
    }

    ret = register_chrdev(0, DEVICE_NAME, &arcturus_gpio_control_fops);
    if (ret < 0) {
        printk(KERN_ERR "Unable to register Arcturus Gpio Control device\n");
        gpio_free(EG91_VBAT);
        gpio_free(EG91_PWR);
        gpio_free(EG91_RST);
        gpio_free(EG91_GPS);
        gpio_free(AP64350_VBAT);
        gpio_free(PT100_RST);
        return ret;
    }

    printk(KERN_INFO "Arcturus GPIO Control module loaded\n");
    return 0;
}

static void __exit arcturus_control_exit(void)
{
    unregister_chrdev(0, DEVICE_NAME);
    gpio_free(EG91_VBAT);
    gpio_free(EG91_PWR);
    gpio_free(EG91_RST);
    gpio_free(EG91_GPS);
    gpio_free(AP64350_VBAT);
    gpio_free(PT100_RST);
    printk(KERN_INFO "Arcturus GPIO Control module unloaded\n");
}

module_init(arcturus_control_init);
module_exit(arcturus_control_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Aluisio Leonello Victal");
MODULE_DESCRIPTION("A simple gpio init/control module for Arcturus board");
