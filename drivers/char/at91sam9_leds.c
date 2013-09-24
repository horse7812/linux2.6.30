#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/init.h>
//#include <linux/delay.h>
//#include <asm/irq.h>
//#include <asm/arch/regs-gpio.h>
//#include <asm/hardware.h>
#include <linux/platform_device.h>
#include <mach/board.h>
#include <mach/gpio.h>
#include <asm/gpio.h>

#define DEVICE_NAME     "at91sam9_leds"  /* 加载模式后，执行”cat /proc/devices”命令看到的设备名称 */
#define LED_MAJOR       0     /* 主设备号 */

/* 应用程序执行ioctl(fd, cmd, arg)时的第2个参数 */
#define IOCTL_LED_ON    0
#define IOCTL_LED_OFF   1

static struct gpio_led at91sam9_leds[] = {
	{	/* XLED0 */
		.name			= "XLED0",
		.gpio			= AT91_PIN_PC27,
		.active_low		= 0,			// XLED0 is off
		.default_trigger	= "none",
	},
	{	/* XLED1 */
		.name			= "XLED1",
		.gpio			= AT91_PIN_PC28,
		.active_low		= 0,			// XLED1 is off
		.default_trigger	= "none",
	},
	{	/* XLED2 */
		.name			= "XLED2",
		.gpio			= AT91_PIN_PC29,
		.active_low		= 0,			// XLED2 is off
		.default_trigger	= "none",
	},
	{	/* XLED3 */
		.name			= "XLED3",
		.gpio			= AT91_PIN_PC30,
		.active_low		= 0,			// XLED3 is off
		.default_trigger	= "none",
	},
};


/* 用来指定LED所用的GPIO引脚 */
/*
static unsigned long led_table [] = {
    S3C2410_GPB5,
    S3C2410_GPB6,
    S3C2410_GPB7,
    S3C2410_GPB8,
};
*/
/* 用来指定GPIO引脚的功能：输出 */
/*
static unsigned int led_cfg_table [] = {
    S3C2410_GPB5_OUTP,
    S3C2410_GPB6_OUTP,
    S3C2410_GPB7_OUTP,
    S3C2410_GPB8_OUTP,
};
*/

/* 应用程序对设备文件/dev/leds执行open(...)时，
 * 就会调用at91sam9_leds_open函数
 */
static int at91sam9_leds_open(struct inode *inode, struct file *file)
{
    int i;
    
    for (i = 0; i < 4; i++) {
        // 设置GPIO引脚的功能：本驱动中LED所涉及的GPIO引脚设为输出功能
        //s3c2410_gpio_cfgpin(led_table[i], led_cfg_table[i]);
        at91_set_gpio_output(at91sam9_leds[i].gpio, at91sam9_leds[i].active_low);
    }
    return 0;
}

/* 应用程序对设备文件/dev/leds执行ioclt(...)时，
 * 就会调用at91sam9_leds_ioctl函数
 */
static int at91sam9_leds_ioctl(
    struct inode *inode, 
    struct file *file, 
    unsigned int cmd, 
    unsigned long arg)
{
    if (arg > 4) {
        return -EINVAL;
    }
    
    switch(cmd) {
    case IOCTL_LED_ON:
        // 设置指定引脚的输出电平为0
        //s3c2410_gpio_setpin(led_table[arg], 0);
        at91_set_gpio_value(at91sam9_leds[arg].gpio, 1);
        return 0;

    case IOCTL_LED_OFF:
        // 设置指定引脚的输出电平为1
        //s3c2410_gpio_setpin(led_table[arg], 1);
        at91_set_gpio_value(at91sam9_leds[arg].gpio, 0);
        return 0;

    default:
        return -EINVAL;
    }
}

/* 这个结构是字符设备驱动程序的核心
 * 当应用程序操作设备文件时所调用的open、read、write等函数，
 * 最终会调用这个结构中指定的对应函数
 */
static struct file_operations at91sam9_leds_fops = {
    .owner  =   THIS_MODULE,    /* 这是一个宏，推向编译模块时自动创建的__this_module变量 */
    .open   =   at91sam9_leds_open,     
    .ioctl  =   at91sam9_leds_ioctl,
};

/*
 * 执行“insmod at91sam9_leds.ko”命令时就会调用这个函数
 */
static int __init at91sam9_leds_init(void)
{
    int ret;

    /* 注册字符设备驱动程序
     * 参数为主设备号、设备名字、file_operations结构；
     * 这样，主设备号就和具体的file_operations结构联系起来了，
     * 操作主设备为LED_MAJOR的设备文件时，就会调用at91sam9_leds_fops中的相关成员函数
     * LED_MAJOR可以设为0，表示由内核自动分配主设备号
     */
    ret = register_chrdev(LED_MAJOR, DEVICE_NAME, &at91sam9_leds_fops);
    if (ret < 0) {
      printk(DEVICE_NAME " can't register major number\n");
      return ret;
    }
    
    printk(DEVICE_NAME " initialized\n");
    return 0;
}

/*
 * 执行”rmmod at91sam9_leds.ko”命令时就会调用这个函数 
 */
static void __exit at91sam9_leds_exit(void)
{
    /* 卸载驱动程序 */
    unregister_chrdev(LED_MAJOR, DEVICE_NAME);
}

/* 这两行指定驱动程序的初始化函数和卸载函数 */
module_init(at91sam9_leds_init);
module_exit(at91sam9_leds_exit);

/* 描述驱动程序的一些信息，不是必须的 */
MODULE_AUTHOR("http://www.wsn.net");             // 驱动程序的作者
MODULE_DESCRIPTION("at91sam9 LED Driver");   // 一些描述信息
MODULE_LICENSE("GPL");                              // 遵循的协议

