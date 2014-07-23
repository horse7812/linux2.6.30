/*
 *  Board-specific setup code for the AT91SAM9M10G45 Evaluation Kit family
 *
 *  Covers: * AT91SAM9G45-EKES  board
 *          * AT91SAM9M10-EKES  board
 *          * AT91SAM9M10G45-EK board
 *
 *  Copyright (C) 2009 Atmel Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */

#include <linux/types.h>
#include <linux/init.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <linux/fb.h>
#include <linux/gpio_keys.h>
#include <linux/input.h>
#include <linux/leds.h>
#include <linux/clk.h>
#include <linux/atmel-mci.h>
#include <linux/i2c/at24.h>
#include <linux/dm9000.h>
#include <linux/serial_sc16is7x2.h>
#include <linux/can/mcp251x.h>

#include <linux/mtd/physmap.h>

#include <mach/hardware.h>
#include <video/atmel_lcdc.h>

#include <asm/setup.h>
#include <asm/mach-types.h>
#include <asm/irq.h>

#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/irq.h>

#include <mach/hardware.h>
#include <mach/board.h>
#include <mach/gpio.h>
#include <mach/at91sam9_smc.h>
#include <mach/at91_shdwc.h>

#include "sam9_smc.h"
#include "generic.h"


static void __init ek_map_io(void)
{
	/* Initialize processor: 12.000 MHz crystal */
	at91sam9g45_initialize(12000000);

	/* DGBU on ttyS0. (Rx & Tx only) */
	at91_register_uart(0, 0, 0);

	/* USART0 not connected on the -EK board */
	at91_register_uart(AT91SAM9G45_ID_US0, 1, ATMEL_UART_RTS);
	//at91_register_uart(AT91SAM9G45_ID_US0, 1, 0);
	/* USART1 on ttyS2. (Rx, Tx) */
	at91_register_uart(AT91SAM9G45_ID_US1, 2, 0); //ATMEL_UART_RTS);
	/* USART2 on ttyS3. (Rx, Tx) */
	at91_register_uart(AT91SAM9G45_ID_US2, 3, 0); //ATMEL_UART_RTS);
	/* USART3 on ttyS4. (Rx, Tx) */
	at91_register_uart(AT91SAM9G45_ID_US3, 4,  0); //ATMEL_UART_RTS);

	/* set serial console to ttyS0 (ie, DBGU) */
	at91_set_serial_console(0);
}

static void __init ek_init_irq(void)
{
	at91sam9g45_init_interrupts(NULL);
}

/*
static struct at91_cf_data at91sam9g45ek_cf_data = {
    .det_pin    = AT91_PIN_PC7,
    .rst_pin    = AT91_PIN_PC17,
    .irq_pin    = AT91_PIN_PC25,
    // .vcc_pin = ... always powered
    .chipselect = 4,
};
*/

/*
 * DM9000 ethernet device
 */
//#if defined(CONFIG_DM9000)
#if 1
static struct resource dm9000_resource0[] = {
    [0] = {
        .start  = AT91_CHIPSELECT_0,	// A5=0
        .end    = AT91_CHIPSELECT_0 + 3,
        .flags  = IORESOURCE_MEM
    },
    [1] = {
        .start  = AT91_CHIPSELECT_0 + 0x4,
        .end    = AT91_CHIPSELECT_0 + 0x7,
        .flags  = IORESOURCE_MEM
    },
    [2] = {
        .start  = AT91_PIN_PD0,	// change at 2012-05-03 by dongking
        .end    = AT91_PIN_PD0,
        .flags  = IORESOURCE_IRQ
    }
};

static struct resource dm9000_resource1[] = {
    [0] = {
        .start  = AT91_CHIPSELECT_0 + 0x20,	// A5=1
        .end    = AT91_CHIPSELECT_0 + 0x20 + 3,
        .flags  = IORESOURCE_MEM
    },
    [1] = {
        .start  = AT91_CHIPSELECT_0 + 0x20 + 0x4,
        .end    = AT91_CHIPSELECT_0 + 0x20 + 0x7,
        .flags  = IORESOURCE_MEM
    },
    [2] = {
        .start  = AT91_PIN_PD1,	// change at 2012-05-03 by dongking
        .end    = AT91_PIN_PD1,
        .flags  = IORESOURCE_IRQ
    }
};


static struct dm9000_plat_data dm9000_platdata0 = {
    .flags      = DM9000_PLATF_16BITONLY|DM9000_PLATF_NO_EEPROM,
    .dev_addr	= {0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F}
};
//dm9000 with fiber-optic
static struct dm9000_plat_data dm9000_platdata1 = {
    .flags      = DM9000_PLATF_16BITONLY|DM9000_PLATF_NO_EEPROM|DM9000_PLATF_FIBER_OPTIC,
    .dev_addr	= {0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x1F}
};

static struct platform_device dm9000_device0 = {
    .name       = "dm9000",
    .id     = 0,
    .num_resources  = ARRAY_SIZE(dm9000_resource0),
    .resource   = dm9000_resource0,
    .dev        = {
        .platform_data  = &dm9000_platdata0,
    }
};

static struct platform_device dm9000_device1 = {
    .name       = "dm9000",
    .id     = 1,
    .num_resources  = ARRAY_SIZE(dm9000_resource1),
    .resource   = dm9000_resource1,
    .dev        = {
        .platform_data  = &dm9000_platdata1,
    }
};

/*
 * SMC timings for the DM9000.
 * Note: These timings were calculated for MASTER_CLOCK = 100000000 according to the DM9000 timings.
 */

static struct sam9_smc_config __initdata dm9000_smc_config = {
    .ncs_read_setup     = 1,
    .nrd_setup      = 3,
    .ncs_write_setup    = 1,
    .nwe_setup      = 3,

    .ncs_read_pulse     = 11,
    .nrd_pulse      = 6,
    .ncs_write_pulse    = 11,
    .nwe_pulse      = 6,

    .read_cycle     = 22,
    .write_cycle        = 22,

    .mode           = AT91_SMC_READMODE | AT91_SMC_WRITEMODE | AT91_SMC_EXNWMODE_DISABLE | AT91_SMC_BAT_WRITE | AT91_SMC_DBW_16,
    .tdf_cycles     = 2,
};

static void __init ek_add_device_dm9000(void)
{
    /* Configure chip-select 0 (DM9000) */
    sam9_smc_configure(0, &dm9000_smc_config);	//NCS0

    /* Configure Reset signal as output */
    //at91_set_A_periph(AT91_PIN_PC13, 0);	//NCS2

    /* Configure Interrupt pin as input, no pull-up */
    at91_set_gpio_input(AT91_PIN_PD0, 0);	// change at 2012-05-03 by dongking
    at91_set_gpio_input(AT91_PIN_PD1, 0);	// change at 2012-05-03 by dongking
    
    platform_device_register(&dm9000_device0);
    platform_device_register(&dm9000_device1);
}
#else
static void __init ek_add_device_dm9000(void) {}
#endif /* CONFIG_DM9000 */


/*
 * USB HS Host port (common to OHCI & EHCI)
 */
static struct at91_usbh_data __initdata ek_usbh_hs_data = {
	.ports		= 2,
	.vbus_pin	= {AT91_PIN_PD9, AT91_PIN_PD18},
};


/*
 * USB HS Device port
 */
static struct usba_platform_data __initdata ek_usba_udc_data = {
	.vbus_pin	= AT91_PIN_PD7,
};


/*
 * SPI devices.
 */
/* 
static struct spi_board_info ek_spi_devices[] = {
	{	// DataFlash chip 
		.modalias	= "mtd_dataflash",
		.chip_select	= 0,
		.max_speed_hz	= 15 * 1000 * 1000,
		.bus_num	= 0,
	},
};*/

/*
  *   SPI uart
  */
const char *sc16is7x2_gpio_names0[] = {
	"spi_gpio_0", 
	"spi_gpio_1",
	"spi_gpio_2", 
	"spi_gpio_3",
	"spi_gpio_4", 
	"spi_gpio_5", 
	"spi_gpio_6", 
	"spi_gpio_7"
};
const char *sc16is7x2_gpio_names1[] = {
	"spi_gpio_8", 
	"spi_gpio_9",
	"spi_gpio_10", 
	"spi_gpio_11",
	"spi_gpio_12", 
	"spi_gpio_13", 
	"spi_gpio_14", 
	"spi_gpio_15"
};

static struct sc16is7x2_platform_data sc16is7x2_SERIALPORT_data0= {
	.uartclk = 5529600,
	.uart_base = 5,
	.gpio_base=180,
	.label = "spi_uart",
	.names = sc16is7x2_gpio_names0,
};
static struct sc16is7x2_platform_data sc16is7x2_SERIALPORT_data1= {
	.uartclk = 5529600,
	.uart_base = 7,
	.gpio_base=188,
	.label = "spi_uart",
	.names = sc16is7x2_gpio_names1,
};

static struct mcp251x_platform_data mcp251x_CAN_data0 = {
	 .oscillator_frequency = 25*1000*1000,
	 .board_specific_setup = NULL,
	 .model = CAN_MCP251X_MCP2515,
	 .power_enable = NULL,
	 .transceiver_enable = NULL,
};
static struct mcp251x_platform_data mcp251x_CAN_data1 = {
	 .oscillator_frequency = 25*1000*1000,
	 .board_specific_setup = NULL,
	 .model = CAN_MCP251X_MCP2515,
	 .power_enable = NULL,
	 .transceiver_enable = NULL,
};


static struct spi_board_info __initdata ek_spi0_devices[] = {
	[0]={
		.modalias      = "mcp251x",
		.bus_num       = 0,
		.chip_select   = 0,
		.irq           = AT91_PIN_PD20,
		.max_speed_hz  = 2*1000*1000,
		.mode = SPI_MODE_0,
		.platform_data = &mcp251x_CAN_data0,
		},
	[1]={
		.modalias      = "mcp251x",
		.bus_num       = 0,
		.chip_select   = 1,
		.irq           = AT91_PIN_PD21,
		.max_speed_hz  = 2*1000*1000, 
		.mode = SPI_MODE_0,
		.platform_data = &mcp251x_CAN_data1,
		},    
	[2]={
		.modalias = "sc16is7x2",
		.bus_num = 0,		
		.chip_select =2,
		.irq = AT91_PIN_PD22,
		.max_speed_hz = 2*1000*1000,
		.mode = SPI_MODE_0,
		.platform_data = &sc16is7x2_SERIALPORT_data0,
		//.controller_data
		},
	[3]={
		.modalias = "sc16is7x2",
		.bus_num = 0,
		.chip_select =3,
		.irq = AT91_PIN_PD23,		
		.max_speed_hz = 2*1000*1000,
		.mode = SPI_MODE_0,
		.platform_data = &sc16is7x2_SERIALPORT_data1,
		//.controller_data
		},		
};

/*
  *   I2C DS3231
  */
  /*
static struct at24_platform_data at24c16 = {
    .byte_len   = 2 * SZ_1K / 8,
    .page_size  = 256,
    .flags      = 0,
};*/
static struct i2c_board_info __initdata ek_i2c_devices[] = {
	#if 0
    {
        I2C_BOARD_INFO("24c02", 0x50),
        .platform_data = &at24c16,
    },
	#endif
    {
        I2C_BOARD_INFO("ds3231", 0xd0 >> 1),
    },
};


/*
 * MCI (SD/MMC)
 */
 /*
static struct mci_platform_data __initdata mci0_data = {
	.slot[0] = {
		.bus_width	= 4,
		.detect_pin	= AT91_PIN_PD10,
		.wp_pin		= -1,
	},
};

static struct mci_platform_data __initdata mci1_data = {
	.slot[0] = {
		.bus_width	= 4,
		.detect_pin	= AT91_PIN_PD11,
		.wp_pin		= AT91_PIN_PD29,
	},
};
*/

/*
 * MACB Ethernet device
 */
static struct at91_eth_data __initdata ek_macb_data = {
	.phy_irq_pin	= AT91_PIN_PD5,
	.is_rmii	= 1,
};


/*
 * NAND flash
 */
static struct mtd_partition __initdata ek_nand_partition[] = {
	{
		.name	= "Bootstrap",
		.offset	= 0,
		.size	= 0x20000,
	},
	{
		.name	= "U-Boot",
		.offset	= 0x20000,
		.size	= 0x60000,
	},
	{
		.name	= "Para1",
		.offset	= 0x60000,
		.size	= 0x20000,
	},
	{
		.name	= "Para2",
		.offset	= 0x80000,
		.size	= 0x20000,
	},
	{
		.name	= "Logo",
		.offset	= 0xa0000,
		.size	= 0x100000,
	},
	{
		.name	= "Kernel",
		.offset	= 0x1a0000,
		.size	= 0x200000,
	},
	{
		.name	= "Rootfs",
		.offset	= MTDPART_OFS_NXTBLK,
		.size	= MTDPART_SIZ_FULL,
	},
};

static struct mtd_partition * __init nand_partitions(int size, int *num_partitions)
{
	*num_partitions = ARRAY_SIZE(ek_nand_partition);
	return ek_nand_partition;
}

/* det_pin is not connected */
static struct atmel_nand_data __initdata ek_nand_data = {
	.ale		= 21,
	.cle		= 22,
	.rdy_pin	= AT91_PIN_PC8,	
	.enable_pin	= AT91_PIN_PC14,
	.partition_info	= nand_partitions,
#if defined(CONFIG_MTD_NAND_AT91_BUSWIDTH_16)
	.bus_width_16	= 1,
#else
	.bus_width_16	= 0,
#endif
};

static struct sam9_smc_config __initdata ek_nand_smc_config = {
	.ncs_read_setup		= 1,	// modify by dongking at 2011-06-22
	.nrd_setup		= 2,
	.ncs_write_setup	= 1,	// modify by dongking at 2011-06-22
	.nwe_setup		= 2,

	.ncs_read_pulse		= 4,
	.nrd_pulse		= 4,
	.ncs_write_pulse	= 4,
	.nwe_pulse		= 4,

	.read_cycle		= 7,
	.write_cycle		= 7,

	.mode			= AT91_SMC_READMODE | AT91_SMC_WRITEMODE | AT91_SMC_EXNWMODE_DISABLE,
	.tdf_cycles		= 3,
};

static void __init ek_add_device_nand(void)
{
	/* setup bus-width (8 or 16) */
	if (ek_nand_data.bus_width_16)
		ek_nand_smc_config.mode |= AT91_SMC_DBW_16;
	else
		ek_nand_smc_config.mode |= AT91_SMC_DBW_8;

	/* configure chip-select 3 (NAND) */
	sam9_smc_configure(3, &ek_nand_smc_config);

	at91_add_device_nand(&ek_nand_data);
}


/*
 * LCD Controller
 */
//#if defined(CONFIG_FB_ATMEL) || defined(CONFIG_FB_ATMEL_MODULE)
#if 0
static struct fb_videomode at91_tft_vga_modes;
static struct fb_videomode __initdata at91_tft_vga_modes_arrary[] = {
	{
		.name           = "TX09D50VM1CCA @ 70",
		.refresh	= 60,
		.xres		= 480,		.yres		= 272,
		.pixclock	= KHZ2PICOS(10000),
		.left_margin	= 1,		.right_margin	= 1,
		.upper_margin	= 40,		.lower_margin	= 1,
		.hsync_len	= 45,		.vsync_len	= 1,
		.sync		= 0,
		.vmode		= FB_VMODE_NONINTERLACED,
	},
	{
		.name           = "TX09D50VM1CCA @ 70",
		.refresh	= 60,
		.xres		= 800,		.yres		= 480,
		.pixclock	= KHZ2PICOS(26000),
		.left_margin	= 17,		.right_margin	= 11,
		.upper_margin	= 4,		.lower_margin	= 7,
		.hsync_len	= 5,		.vsync_len	= 1,
		.sync		= 0,
		.vmode		= FB_VMODE_NONINTERLACED,
	},
	{
		.name           = "TX09D50VM1CCA @ 70",
		.refresh	= 60,
		.xres		= 800,		.yres		= 600,
		.pixclock	= KHZ2PICOS(30000),

		.left_margin	= 1,		.right_margin	= 1,
		.upper_margin	= 40,		.lower_margin	= 1,
		.hsync_len	= 45,		.vsync_len	= 1,

		.sync		= 0,
		.vmode		= FB_VMODE_NONINTERLACED,
	},
};

static struct fb_monspecs at91fb_default_monspecs = {
	.manufacturer	= "HIT",
	.monitor        = "TX09D70VM1CCA",

	.modedb		=  &at91_tft_vga_modes,
	.modedb_len	=  1,
	.hfmin		= 15000,
	.hfmax		= 17640,
	.vfmin		= 57,
	.vfmax		= 67,
};

#define AT91SAM9G45_DEFAULT_LCDCON2 	(ATMEL_LCDC_MEMOR_LITTLE \
					| ATMEL_LCDC_DISTYPE_TFT \
					| ATMEL_LCDC_IFWIDTH_16 \
					| ATMEL_LCDC_CLKMOD_ALWAYSACTIVE)

/* Driver datas */
static struct atmel_lcdfb_info __initdata ek_lcdc_data = {
	.lcdcon_is_backlight		= true,
	.default_bpp			= 16,
	.default_dmacon			= ATMEL_LCDC_DMAEN,
	.default_lcdcon2		= AT91SAM9G45_DEFAULT_LCDCON2,
	.default_monspecs		= &at91fb_default_monspecs,
	.guard_time			= 9,
	.lcd_wiring_mode		= ATMEL_LCDC_WIRING_RGB,
};

void __init ek_fb_set_platdata(int default_display)
{
	memcpy(&at91_tft_vga_modes, &at91_tft_vga_modes_arrary[default_display], sizeof(at91_tft_vga_modes));
}
#else
//static struct atmel_lcdfb_info __initdata ek_lcdc_data;
void __init ek_fb_set_platdata(int default_display){}
#endif
	

/*
 * Touchscreen
 */
 /*
static struct at91_tsadcc_data ek_tsadcc_data = {
	.adc_clock		= 300000,
	.pendet_debounce	= 0x0d,
	.ts_sample_hold_time	= 0x0a,
};
*/

/*
 * GPIO Buttons
 */
#if defined(CONFIG_KEYBOARD_GPIO) || defined(CONFIG_KEYBOARD_GPIO_MODULE)
static struct gpio_keys_button ek_buttons[] = {
	{	/* BP1, "XKEY0" */
		.code		= BTN_LEFT,
		.gpio		= AT91_PIN_PD4,
		.active_low	= 1,
		.desc		= "XKEY0",
		//.wakeup		= 1,
	},
	{	/* BP2, "XKEY1" */
		.code		= BTN_RIGHT,
		.gpio		= AT91_PIN_PD6,
		.active_low	= 1,
		.desc		= "XKEY1",
		//.wakeup		= 1,
	}
};

static struct gpio_keys_platform_data ek_button_data = {
	.buttons	= ek_buttons,
	.nbuttons	= ARRAY_SIZE(ek_buttons),
};

static struct platform_device ek_button_device = {
	.name		= "gpio-keys",
	.id		= -1,
	.num_resources	= 0,
	.dev		= {
		.platform_data	= &ek_button_data,
	}
};

static void __init ek_add_device_buttons(void)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(ek_buttons); i++) {
		at91_set_GPIO_periph(ek_buttons[i].gpio, 1);
		at91_set_deglitch(ek_buttons[i].gpio, 1);
	}

	platform_device_register(&ek_button_device);
}
#else
static void __init ek_add_device_buttons(void) {}
#endif


/*
 * AC97
 * reset_pin is not connected: NRST
 */
 /*
static struct ac97c_platform_data ek_ac97_data = {
};
*/

/*
 * LEDs ... these could all be PWM-driven, for variable brightness
 */

static struct gpio_led ek_leds[] = {
	{	// XLED0 ALARM
		.name			= "XLED0",
		.gpio			= AT91_PIN_PD2,
		.active_low		= 1,			// XLED0 is off
		.default_trigger	= "none",
	},
	{	// XLED1 FAULT
		.name			= "XLED1",
		.gpio			= AT91_PIN_PD3,
		.active_low		= 1,			// XLED1 is off
		.default_trigger	= "none",
	}
#if 0
	{	// XLED2 RUN
		.name			= "XLED2",
		.gpio			= AT91_PIN_PD2,
		.active_low		= 1,			// XLED0 is off
		.default_trigger	= "none",
	},
#endif
};

/*
 * PWM Leds
 */
static struct gpio_led ek_pwm_led[] = {
#if defined(CONFIG_LEDS_ATMEL_PWM) || defined(CONFIG_LEDS_ATMEL_PWM_MODULE)
	{	/* "right" led, green, userled1, pwm1 */
		.name			= "BELL",
		.gpio			= AT91_PWM2,	/* is PWM channel number */
		.active_low		= 0,
		.default_trigger	= "none",
	},
#endif
};
/*
static char tft_type = 's';

static int __init ek_tft_setup(char *str)
{
    tft_type = str[0];
    return 1;
}

__setup("tft=", ek_tft_setup);
*/
static void __init ek_board_init(void)
{/*
	int default_display = 0;
	switch (tft_type) {
    case 's': // small or production 
        default_display = 0;
        break;
    case 'm': // middle 
        default_display = 1;
        break;
    case 'b': // big 
    default:
        default_display = 2;
        break;
    }
    ek_fb_set_platdata(default_display);*/
	/* Serial */
	at91_add_device_serial();
	/* USB HS Host */
	at91_add_device_usbh_ohci(&ek_usbh_hs_data);
	at91_add_device_usbh_ehci(&ek_usbh_hs_data);
	/* USB HS Device */
	at91_add_device_usba(&ek_usba_udc_data);	
	/* MMC */
	#if 0
	at91_add_device_mci(0, &mci0_data);
	at91_add_device_mci(1, &mci1_data);
	#endif
	/* Ethernet */
	at91_add_device_eth(&ek_macb_data);
	#if 1
	/* DM9000 ethernet */
	ek_add_device_dm9000();
	#endif
	/* NAND */
	ek_add_device_nand();
	/* SPI */
	at91_add_device_spi(ek_spi0_devices, ARRAY_SIZE(ek_spi0_devices));
	/* I2C */
	at91_add_device_i2c(0, ek_i2c_devices, ARRAY_SIZE(ek_i2c_devices));
	/* LCD Controller */
	//at91_add_device_lcdc(&ek_lcdc_data);
	/* Touch Screen */
	//at91_add_device_tsadcc(&ek_tsadcc_data);
	/* Push Buttons */
	ek_add_device_buttons();
	/* AC97 */
	//at91_add_device_ac97(&ek_ac97_data);
	/* LEDs */
	at91_gpio_leds(ek_leds, ARRAY_SIZE(ek_leds));
	at91_pwm_leds(ek_pwm_led, ARRAY_SIZE(ek_pwm_led));
	#if 0
	/* CF */
	at91_add_device_cf(&at91sam9g45ek_cf_data);
	#endif
}


#if defined(CONFIG_MACH_AT91SAM9G45EKES)
MACHINE_START(AT91SAM9G45EKES, "Atmel AT91SAM9G45-EKES")
#elif defined(CONFIG_MACH_AT91SAM9M10EKES)
MACHINE_START(AT91SAM9M10EKES, "Atmel AT91SAM9M10-EKES")
#else
MACHINE_START(AT91SAM9M10G45EK, "Atmel AT91SAM9M10G45-EK")
#endif
	/* Maintainer: Atmel */
	.phys_io	= AT91_BASE_SYS,
	.io_pg_offst	= (AT91_VA_BASE_SYS >> 18) & 0xfffc,
	.boot_params	= AT91_SDRAM_BASE + 0x100,
	.timer		= &at91sam926x_timer,
	.map_io		= ek_map_io,
	.init_irq	= ek_init_irq,
	.init_machine	= ek_board_init,
MACHINE_END
