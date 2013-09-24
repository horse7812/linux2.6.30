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

#define DEVICE_NAME     "at91sam9_leds"  /* ����ģʽ��ִ�С�cat /proc/devices����������豸���� */
#define LED_MAJOR       0     /* ���豸�� */

/* Ӧ�ó���ִ��ioctl(fd, cmd, arg)ʱ�ĵ�2������ */
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


/* ����ָ��LED���õ�GPIO���� */
/*
static unsigned long led_table [] = {
    S3C2410_GPB5,
    S3C2410_GPB6,
    S3C2410_GPB7,
    S3C2410_GPB8,
};
*/
/* ����ָ��GPIO���ŵĹ��ܣ���� */
/*
static unsigned int led_cfg_table [] = {
    S3C2410_GPB5_OUTP,
    S3C2410_GPB6_OUTP,
    S3C2410_GPB7_OUTP,
    S3C2410_GPB8_OUTP,
};
*/

/* Ӧ�ó�����豸�ļ�/dev/ledsִ��open(...)ʱ��
 * �ͻ����at91sam9_leds_open����
 */
static int at91sam9_leds_open(struct inode *inode, struct file *file)
{
    int i;
    
    for (i = 0; i < 4; i++) {
        // ����GPIO���ŵĹ��ܣ���������LED���漰��GPIO������Ϊ�������
        //s3c2410_gpio_cfgpin(led_table[i], led_cfg_table[i]);
        at91_set_gpio_output(at91sam9_leds[i].gpio, at91sam9_leds[i].active_low);
    }
    return 0;
}

/* Ӧ�ó�����豸�ļ�/dev/ledsִ��ioclt(...)ʱ��
 * �ͻ����at91sam9_leds_ioctl����
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
        // ����ָ�����ŵ������ƽΪ0
        //s3c2410_gpio_setpin(led_table[arg], 0);
        at91_set_gpio_value(at91sam9_leds[arg].gpio, 1);
        return 0;

    case IOCTL_LED_OFF:
        // ����ָ�����ŵ������ƽΪ1
        //s3c2410_gpio_setpin(led_table[arg], 1);
        at91_set_gpio_value(at91sam9_leds[arg].gpio, 0);
        return 0;

    default:
        return -EINVAL;
    }
}

/* ����ṹ���ַ��豸��������ĺ���
 * ��Ӧ�ó�������豸�ļ�ʱ�����õ�open��read��write�Ⱥ�����
 * ���ջ��������ṹ��ָ���Ķ�Ӧ����
 */
static struct file_operations at91sam9_leds_fops = {
    .owner  =   THIS_MODULE,    /* ����һ���꣬�������ģ��ʱ�Զ�������__this_module���� */
    .open   =   at91sam9_leds_open,     
    .ioctl  =   at91sam9_leds_ioctl,
};

/*
 * ִ�С�insmod at91sam9_leds.ko������ʱ�ͻ�����������
 */
static int __init at91sam9_leds_init(void)
{
    int ret;

    /* ע���ַ��豸��������
     * ����Ϊ���豸�š��豸���֡�file_operations�ṹ��
     * ���������豸�žͺ;����file_operations�ṹ��ϵ�����ˣ�
     * �������豸ΪLED_MAJOR���豸�ļ�ʱ���ͻ����at91sam9_leds_fops�е���س�Ա����
     * LED_MAJOR������Ϊ0����ʾ���ں��Զ��������豸��
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
 * ִ�С�rmmod at91sam9_leds.ko������ʱ�ͻ����������� 
 */
static void __exit at91sam9_leds_exit(void)
{
    /* ж���������� */
    unregister_chrdev(LED_MAJOR, DEVICE_NAME);
}

/* ������ָ����������ĳ�ʼ��������ж�غ��� */
module_init(at91sam9_leds_init);
module_exit(at91sam9_leds_exit);

/* �������������һЩ��Ϣ�����Ǳ���� */
MODULE_AUTHOR("http://www.wsn.net");             // �������������
MODULE_DESCRIPTION("at91sam9 LED Driver");   // һЩ������Ϣ
MODULE_LICENSE("GPL");                              // ��ѭ��Э��

