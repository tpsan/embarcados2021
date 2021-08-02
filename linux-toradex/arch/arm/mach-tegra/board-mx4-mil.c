/*
 * arch/arm/mach-tegra/board-colibri_t20.c
 *
 * Copyright (C) 2011-2012 Toradex, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <asm/mach/arch.h>
#include <asm/mach-types.h>

#include <linux/can/platform/mcp251x.h>
#include <linux/can/platform/sja1000.h>
#include <linux/clk.h>
#include <linux/colibri_usb.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/i2c-tegra.h>
#include <linux/input.h>
#include <linux/io.h>
#include <linux/leds_pwm.h>
#include <linux/lm95245.h>
#include <linux/memblock.h>
#include <linux/platform_data/tegra_usb.h>
#include <linux/platform_device.h>
#include <linux/reboot.h>
#include <linux/serial_8250.h>
#include <linux/spi/spi.h>
#if defined(CONFIG_SPI_GPIO) || defined(CONFIG_SPI_GPIO_MODULE)
#include <linux/spi/spi_gpio.h>
#endif
#include <linux/tegra_uart.h>
#include <linux/usb/gpio_vbus.h>
#include <linux/wm97xx.h>
#include <linux/mma845x.h>

#include <mach/gpio.h>
#include <mach/mx4_mil_iomap.h>
#include <mach/nand.h>
#include <mach/sdhci.h>
#include <mach/usb_phy.h>
#include <mach/w1.h>

#include "board-mx4-mil.h"
#include "board.h"
#include "clock.h"
#include "cpu-tegra.h"
#include "devices.h"
#include "gpio-names.h"
//#include "../../../drivers/mtd/maps/tegra_nor.h"
#include <linux/platform_data/tegra_nor.h>
#include "pm.h"
#include "pm-irq.h"
#include "wakeups-t2.h"

/* Legacy defines from previous tegra_nor.h (changed) */
#define __BITMASK0(len)					((1 << (len)) - 1)
#define __BITMASK(start, len)			(__BITMASK0(len) << (start))
#define REG_BIT(bit)					(1 << (bit))
#define REG_FIELD(val, start, len)		(((val) & __BITMASK0(len)) << (start))
#define TEGRA_GMI_PHYS					0x70009000
#define TEGRA_GMI_BASE					IO_TO_VIRT(TEGRA_GMI_PHYS)
#define SNOR_CONFIG_REG					(TEGRA_GMI_BASE + 0x00)
#define SNOR_STATUS_REG					(TEGRA_GMI_BASE + 0x04)
#define SNOR_CONFIG_SNOR_CS(val) 		REG_FIELD((val), 4, 3)
#define SNOR_CONFIG_GO					REG_BIT(31)
#define SNOR_CONFIG_32BIT				REG_BIT(30)
#define SNOR_CONFIG_MUX					REG_BIT(28)
#define SNOR_CONFIG_ADV_POL				REG_BIT(22)


/* We define a no export flag. We dont want to export the gpio
for a NC pin By: Mirza */
#define GPIOF_NO_EXPORT		(1 << 6)

/* Active LOW flag - Supplement to gpio.h flags in line with the
   GPIOF_NO_EXPORT flag */
#define GPIOF_ACT_LOW		(1 << 7)

#if defined(CONFIG_CAN_SJA1000) || defined(CONFIG_CAN_SJA1000_MODULE)
static struct resource colibri_can_resource[] = {
	[0] =   {
		.flags	= IORESOURCE_MEM,
	},
	[1] =   {
		/* interrupt assigned during initialisation */
		.flags	= IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHEDGE,
	}
};

static struct sja1000_platform_data colibri_can_platdata = {
	.osc_freq	= 24000000,
	.ocr		= (OCR_MODE_NORMAL | OCR_TX0_PUSHPULL),
	.cdr		= CDR_CLKOUT_MASK |  /* Set CLKOUT to Fosc */
			  CDR_CBP, /* CAN input comparator bypass */
};

static struct platform_device colibri_can_device = {
	.name		= "sja1000_platform",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(colibri_can_resource),
	.resource	= colibri_can_resource,
	.dev            = {
		.platform_data = &colibri_can_platdata,
	}
};

static struct resource colibri_can_resource2[] = {
	[0] =   {
		.flags	= IORESOURCE_MEM,
	},
	[1] =   {
		/* interrupt assigned during initialisation */
		.flags	= IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHEDGE,
	}
};

static struct sja1000_platform_data colibri_can_platdata2 = {
	.osc_freq	= 24000000,
	.ocr		= (OCR_MODE_NORMAL | OCR_TX0_PUSHPULL),
	.cdr		= CDR_CLK_OFF | /* Clock off (CLKOUT pin) */
			  CDR_CBP, /* CAN input comparator bypass */
};

static struct platform_device colibri_can_device2 = {
	.name		= "sja1000_platform",
	.id		= 1,
	.num_resources	= ARRAY_SIZE(colibri_can_resource2),
	.resource	= colibri_can_resource2,
	.dev            = {
		.platform_data = &colibri_can_platdata2,
	}
};

static struct resource colibri_can_resource3[] = {
	[0] =   {
		.flags	= IORESOURCE_MEM,
	},
	[1] =   {
		/* interrupt assigned during initialisation */
		.flags	= IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHEDGE,
	}
};

static struct sja1000_platform_data colibri_can_platdata3 = {
	.osc_freq	= 24000000,
	.ocr		= (OCR_MODE_NORMAL | OCR_TX0_PUSHPULL),
	.cdr		= CDR_CLK_OFF | /* Clock off (CLKOUT pin) */
			  CDR_CBP, /* CAN input comparator bypass */
};

static struct platform_device colibri_can_device3 = {
	.name		= "sja1000_platform",
	.id		= 2,
	.num_resources	= ARRAY_SIZE(colibri_can_resource3),
	.resource	= colibri_can_resource3,
	.dev            = {
		.platform_data = &colibri_can_platdata3,
	}
};

#endif /* CONFIG_CAN_SJA1000 || CONFIG_CAN_SJA1000_MODULE */

/* 	First appearence is in P1E revision. This revision also added a third can
	so we use the same switch to add the third CAN
*/
static uint8_t vcb_muxed_can = 0;

static int __init vcb_can_mode(char *options)
{
	if (!strcmp(options, "1"))
		vcb_muxed_can = 1;

	return 0;
}
__setup("vcb_muxed_can=", vcb_can_mode);

static void vcb_sja1000_init(void)
{

	u32 snor_config;

	snor_config = readl(SNOR_CONFIG_REG);

	printk("VCB CAN mode is \"%s\"\n", vcb_muxed_can ? "muxed" : "not muxed");

	if (vcb_muxed_can)
		snor_config |= (SNOR_CONFIG_ADV_POL | SNOR_CONFIG_MUX);

	snor_config |= (SNOR_CONFIG_GO | SNOR_CONFIG_SNOR_CS(SNOR_CS_PIN));
	writel(snor_config, SNOR_CONFIG_REG);

	tegra_gpio_enable(TEGRA_CAN_INT);
	tegra_gpio_enable(TEGRA_CAN2_INT);
	tegra_gpio_enable(TEGRA_CAN3_INT);

	gpio_request_one(TEGRA_CAN_INT, GPIOF_DIR_IN, "CAN1-INT");
	gpio_request_one(TEGRA_CAN2_INT, GPIOF_DIR_IN, "CAN2-INT");
	gpio_request_one(TEGRA_CAN3_INT, GPIOF_DIR_IN, "CAN3-INT");


	if (vcb_muxed_can) {
		colibri_can_resource[0].start = CAN_ADDRESS_TO_HW(CAN_OFFSET_MUXED);
		colibri_can_resource[0].end = CAN_ADDRESS_TO_HW(CAN_OFFSET_MUXED)
			+ CAN_ADDRESS_SIZE;

		colibri_can_resource[0].flags |= IORESOURCE_MEM_16BIT;

		colibri_can_resource2[0].start = CAN_ADDRESS_TO_HW(CAN2_OFFSET_MUXED);
		colibri_can_resource2[0].end = CAN_ADDRESS_TO_HW(CAN2_OFFSET_MUXED)
			+ CAN_ADDRESS_SIZE;

		colibri_can_resource2[0].flags |= IORESOURCE_MEM_16BIT;

		colibri_can_resource3[0].start = CAN_ADDRESS_TO_HW(CAN3_OFFSET_MUXED);
		colibri_can_resource3[0].end = CAN_ADDRESS_TO_HW(CAN3_OFFSET_MUXED)
			+ CAN_ADDRESS_SIZE;

		colibri_can_resource3[0].flags |= IORESOURCE_MEM_16BIT;
	} else {
		colibri_can_resource[0].start =
			CAN_ADDRESS_TO_HW(CAN_OFFSET_NON_MUXED);

		colibri_can_resource[0].end =
			CAN_ADDRESS_TO_HW(CAN_OFFSET_NON_MUXED) + CAN_ADDRESS_SIZE;

		colibri_can_resource2[0].start =
			CAN_ADDRESS_TO_HW(CAN2_OFFSET_NON_MUXED);

		colibri_can_resource2[0].end =
			CAN_ADDRESS_TO_HW(CAN2_OFFSET_NON_MUXED) + CAN_ADDRESS_SIZE;
	}

	colibri_can_resource[1].start	= TEGRA_GPIO_TO_IRQ(TEGRA_CAN_INT);
	colibri_can_resource[1].end	= TEGRA_GPIO_TO_IRQ(TEGRA_CAN_INT);

	colibri_can_resource2[1].start	= TEGRA_GPIO_TO_IRQ(TEGRA_CAN2_INT);
	colibri_can_resource2[1].end	= TEGRA_GPIO_TO_IRQ(TEGRA_CAN2_INT);

	colibri_can_resource3[1].start	= TEGRA_GPIO_TO_IRQ(TEGRA_CAN3_INT);
	colibri_can_resource3[1].end	= TEGRA_GPIO_TO_IRQ(TEGRA_CAN3_INT);

	platform_device_register(&colibri_can_device);
	platform_device_register(&colibri_can_device2);


	if (vcb_muxed_can)
		platform_device_register(&colibri_can_device3);
}


/* Clocks */
static struct tegra_clk_init_table colibri_t20_clk_init_table[] __initdata = {
	/* name		parent		rate		enabled */
	{"blink",	"clk_32k",	32768,		false},
	/* SMSC3340 REFCLK 24 MHz */
	{"pll_p_out4",	"pll_p",	24000000,	true},
	{"pwm",		"clk_m",	0,		false},
	{"spdif_out",	"pll_a_out0",	0,		false},

	/* uarta disabled by "Disabling clocks left on by bootloader" stage,
	   uarte will be used for debug console */
	{"uarte",	"pll_p",	216000000,	true},

//required otherwise uses pll_p_out4 as parent and changing its rate to 72 MHz
	{"sclk",	"pll_p_out3",	108000000,	true},

	/* AC97 incl. touch (note: unfortunately no clk source mux exists) */
	{"ac97",	"pll_a_out0",	24576000,	true},

	//External bus
	{"nor",		"pll_p",	86500000,	true},

	/* WM9715L XTL_IN 24.576 MHz */
//[    0.372722] Unable to set parent pll_a_out0 of clock cdev1: -38
//	{"cdev1",	"pll_a_out0",	24576000,	true},
//	{"pll_a_out0",	"pll_a",	24576000,	true},

	{"vde",		"pll_c",	240000000,	false},

	{"ndflash",	"pll_p",	108000000,	false},

//[    2.284308] kernel BUG at drivers/spi/spi-tegra.c:254!
//[    2.289454] Unable to handle kernel NULL pointer dereference at virtual address 00000000
	{"sbc4",	"pll_p",	12000000,	false},

	{NULL,		NULL,		0,		0},
};

/* GPIO */

#define FF_DCD		TEGRA_GPIO_PC6	/* SODIMM 31 */
#define FF_DSR		TEGRA_GPIO_PC1	/* SODIMM 29 */

#define I2C_SCL		TEGRA_GPIO_PC4	/* SODIMM 196 */
#define I2C_SDA		TEGRA_GPIO_PC5	/* SODIMM 194 */

#define LAN_EXT_WAKEUP	TEGRA GPIO_PV5
#define LAN_PME		TEGRA_GPIO_PV6
#define LAN_RESET	TEGRA_GPIO_PV4
#define LAN_V_BUS	TEGRA_GPIO_PBB1

#define MMC_CD		TEGRA_GPIO_PT4	/* SODIMM 43 */

#define NAND_WP_N	TEGRA_GPIO_PS0

#define PWR_I2C_SCL	TEGRA_GPIO_PZ6
#define PWR_I2C_SDA	TEGRA_GPIO_PZ7

#define MECS_USB_HUB_RESET	TEGRA_GPIO_PBB3	/* SODIMM 127 */

#define THERMD_ALERT	TEGRA_GPIO_PV7

#define TOUCH_PEN_INT	TEGRA_GPIO_PV2

#define USB3340_RESETB	TEGRA_GPIO_PV1

///TBC
#define USBC_DET	TEGRA_GPIO_PK5	/* SODIMM 137 */
#define USBH_OC		TEGRA_GPIO_PW3	/* SODIMM 131 */
#define USBH_PEN	TEGRA_GPIO_PW2	/* SODIMM 129 */

static struct gpio colibri_t20_gpios[] = {
	/* Not connected pins */
	//Might conflict with Volume up key
	{TEGRA_GPIO_PBB4,	(GPIOF_IN | GPIOF_NO_EXPORT),	"P22 - NC"},
	//Might conflict with Volume down key
	{TEGRA_GPIO_PBB5,	(GPIOF_IN | GPIOF_NO_EXPORT),		"P24 - NC"},
	{TEGRA_GPIO_PL7,	(GPIOF_IN | GPIOF_NO_EXPORT),		"P65 - NC"},
	{TEGRA_GPIO_PK4,	(GPIOF_IN | GPIOF_NO_EXPORT),		"P106 - NC"},
	{TEGRA_GPIO_PK1,	(GPIOF_IN | GPIOF_NO_EXPORT),		"P152 - NC"},
	{TEGRA_GPIO_PU5,	(GPIOF_IN | GPIOF_NO_EXPORT),		"P116 - NC"}, // Wake up
	{TEGRA_GPIO_PU6,	(GPIOF_IN | GPIOF_NO_EXPORT),		"P118 - NC"}, //Wake up
	// Used by BL_ON (see board-colibri_t20-panel.c)
	//{TEGRA_GPIO_PP4,	(GPIOF_IN | GPIOF_NO_EXPORT),		"P120 - NC"},
	//Might conflict with ADDRESS14
	{TEGRA_GPIO_PP5,	(GPIOF_IN | GPIOF_NO_EXPORT),		"P122 - NC"},
	//Might conflict with ADDRESS15
	{TEGRA_GPIO_PP6,	(GPIOF_IN | GPIOF_NO_EXPORT),		"P124 - NC"},
	{TEGRA_GPIO_PP7,	(GPIOF_IN | GPIOF_NO_EXPORT),		"P188 - NC"},
	{TEGRA_GPIO_PJ0,	(GPIOF_IN | GPIOF_NO_EXPORT),		"P126 - NC"},
	{TEGRA_GPIO_PJ2,	(GPIOF_IN | GPIOF_NO_EXPORT),		"P128 - NC"},
	{TEGRA_GPIO_PI3,	(GPIOF_IN | GPIOF_NO_EXPORT),		"P130 - NC"},
	{TEGRA_GPIO_PI6,	(GPIOF_IN | GPIOF_NO_EXPORT),		"P132 - NC"},
    /* GMI_IORDY multiplexed GMI_WAIT/GMI_IORDY in pinmux - not used */
	//{TEGRA_GPIO_PI7,	(GPIOF_IN | GPIOF_NO_EXPORT),		"P95 - NC"},
	{TEGRA_GPIO_PI5,	(GPIOF_IN | GPIOF_NO_EXPORT),		"P95 - NC"},
	{TEGRA_GPIO_PX4,	(GPIOF_IN | GPIOF_NO_EXPORT),		"P134 - NC"},
	//Pin 136, 138, 140, 142 Muxed to PM2 et al in pinmux (SPI2). Currently not used
	{TEGRA_GPIO_PX3,	(GPIOF_IN | GPIOF_NO_EXPORT),		"P136 - NC"},
	{TEGRA_GPIO_PX2,	(GPIOF_IN | GPIOF_NO_EXPORT),		"P138 - NC"},
	{TEGRA_GPIO_PX1,	(GPIOF_IN | GPIOF_NO_EXPORT),		"P140 - NC"},
	{TEGRA_GPIO_PX0,	(GPIOF_IN | GPIOF_NO_EXPORT),		"P142 - NC"},
	{TEGRA_GPIO_PB2,	(GPIOF_IN | GPIOF_NO_EXPORT),		"P154 - NC"},
	{TEGRA_GPIO_PN5,	(GPIOF_IN | GPIOF_NO_EXPORT),		"P158 - NC"},
	{TEGRA_GPIO_PN4,	(GPIOF_IN | GPIOF_NO_EXPORT),		"P160 - NC"},
	{TEGRA_GPIO_PN6,	(GPIOF_IN | GPIOF_NO_EXPORT),		"P162 - NC"},
	{TEGRA_GPIO_PZ2,	(GPIOF_IN | GPIOF_NO_EXPORT),		"P156 - NC"},
	{TEGRA_GPIO_PZ4,	(GPIOF_IN | GPIOF_NO_EXPORT),		"P164 - NC"},
	{TEGRA_GPIO_PAA4,	(GPIOF_IN | GPIOF_NO_EXPORT),		"P166 - NC"},
	{TEGRA_GPIO_PAA5,	(GPIOF_IN | GPIOF_NO_EXPORT),		"P168 - NC"},
	{TEGRA_GPIO_PAA6,	(GPIOF_IN | GPIOF_NO_EXPORT),		"P170 - NC"},
	{TEGRA_GPIO_PAA7,	(GPIOF_IN | GPIOF_NO_EXPORT),		"P172 - NC"},
	{TEGRA_GPIO_PA5,	(GPIOF_IN | GPIOF_NO_EXPORT),		"P144 - NC"},
	{TEGRA_GPIO_PA4,	(GPIOF_IN | GPIOF_NO_EXPORT),		"P146 - NC"},

	/* Digital inputs */
	// P45 is used for CF in PXA. Consider change.

	//{TEGRA_GPIO_PY6,	(GPIOF_IN | GPIOF_NO_EXPORT),	"P37 - WAKE-UP-CPU"},
	{TEGRA_GPIO_PK6,	(GPIOF_IN ),                	"P135 - MODEM-WAKEUP"},
	{TEGRA_GPIO_PC6,	(GPIOF_IN ),                	"P31 - XANTSHORT"},

	/* Compact flash - These pins are not connected on T20*/
	{TEGRA_GPIO_PC1,	(GPIOF_IN | GPIOF_NO_EXPORT),		"P29 - CF-READY"},
	{TEGRA_GPIO_PT1,	(GPIOF_IN | GPIOF_NO_EXPORT),		"P75 - CF-RESET"},
	{TEGRA_GPIO_PT3,	(GPIOF_IN | GPIOF_NO_EXPORT),		"P77 - CF-BVD2/MM_PXA300_CLK"},
/*
	{TEGRA_GPIO_PD6,	(GPIOF_IN | GPIOF_NO_EXPORT),		"P81 - CF-nCD1+2"},
*/
	{TEGRA_GPIO_PL6,	(GPIOF_IN | GPIOF_NO_EXPORT),		"P85 - CF-nPPEN"},
	{TEGRA_GPIO_PD7,	(GPIOF_IN | GPIOF_NO_EXPORT),		"P94 - CF-nPCE1"},
	{TEGRA_GPIO_PX5,	(GPIOF_IN | GPIOF_NO_EXPORT),		"P100 - CF-nPSKTSEL"},

	{TEGRA_GPIO_PX6,	(GPIOF_IN | GPIOF_NO_EXPORT),		"P102 - CF-nPWAIT/MM_PXA300_DAT3"},

	{TEGRA_GPIO_PX7,	(GPIOF_IN | GPIOF_NO_EXPORT),		"P104 - CF-nIOIS16/MM_PXA300_DAT2"},

	/* Wakeup of external ethernet interface */
	{TEGRA_GPIO_PAA2,	(GPIOF_DIR_OUT | GPIOF_INIT_LOW | GPIOF_ACT_LOW),		"P51 - OPT1-WAKE"},
	{TEGRA_GPIO_PAA3,	(GPIOF_DIR_OUT | GPIOF_INIT_LOW | GPIOF_ACT_LOW),		"P53 - OPT2-WAKE"},

	/* Gyro Interrupts */
	/* Our gyro driver does not support interrupts though */
 	{TEGRA_GPIO_PA0,	(GPIOF_IN | GPIOF_NO_EXPORT),		"P73 - GYRO-INT1"},
 	{TEGRA_GPIO_PA7,	(GPIOF_IN | GPIOF_NO_EXPORT),		"P67 - GYRO-INT2"},

	/* These are enabled in respective driver. Only have them here for reference */

	/* MMC Interface*/
	//{TEGRA_GPIO_PT0,	(GPIOF_IN | GPIOF_NO_EXPORT),		"P96 - MMC-CLK"},
	//{TEGRA_GPIO_PD5,	(GPIOF_IN | GPIOF_NO_EXPORT),		"P98 - MMC-CMD"},
	//{TEGRA_GPIO_PL0,	(GPIOF_IN | GPIOF_NO_EXPORT),		"P101 - MMC-DAT0"},
	//{TEGRA_GPIO_PL1,	(GPIOF_IN | GPIOF_NO_EXPORT),		"P103 - MMC-DAT1"},
	//{TEGRA_GPIO_PL2,	(GPIOF_IN | GPIOF_NO_EXPORT),		"P79 - MMC-DAT2"},
	//{TEGRA_GPIO_PL3,	(GPIOF_IN | GPIOF_NO_EXPORT),		"P97 - MMC-DAT3"},

	/* MM CARD DETECT*/
	//{TEGRA_GPIO_PT4,	(GPIOF_IN | GPIOF_NO_EXPORT),		"P71 - MM_CD"},

	/* Accelometer Interrupts */
 	//{TEGRA_GPIO_PB7,	GPIOF_IN,		"P63 - ACC-INT1"},
 	//{TEGRA_GPIO_PK4,	GPIOF_IN,		"P131 - ACC-INT2"},

	/* CAN Interrupts*/
	//{TEGRA_GPIO_PB5,	GPIOF_IN,		"P28 - CAN1-INT"},
	//{TEGRA_GPIO_PA6,	GPIOF_IN,		"P30 - CAN2-INT"},
};

static void colibri_t20_gpio_init(void)
{
	int i = 0;
	int length = sizeof(colibri_t20_gpios) / sizeof(struct gpio);
	int err = 0;

	for (i = 0; i < length; i++) {
		err = gpio_request_one(colibri_t20_gpios[i].gpio,
				       colibri_t20_gpios[i].flags,
				       colibri_t20_gpios[i].label);

		if (err) {
			pr_warning("gpio_request(%s) failed, err = %d",
				   colibri_t20_gpios[i].label, err);
		} else {
			tegra_gpio_enable(colibri_t20_gpios[i].gpio);

			if(!(colibri_t20_gpios[i].flags & GPIOF_NO_EXPORT)) {
				gpio_export(colibri_t20_gpios[i].gpio, true);
			}
			if(colibri_t20_gpios[i].flags & GPIOF_ACT_LOW) {
				gpio_sysfs_set_active_low(colibri_t20_gpios[i].gpio, true);
			}
		}
	}

	/* Enable wake up on selected gpio (see mx4_iomap.h) - set edge in sysfs from userspace to enable */
	err = enable_irq_wake(gpio_to_irq(GPIO_WAKEUP_PIN));
	if (err) {
		pr_err("Failed to enable wakeup for irq %d\n", gpio_to_irq(GPIO_WAKEUP_PIN));
	}
	else {
		pr_info("Enabling gpio wakeup on irq %d\n", gpio_to_irq(GPIO_WAKEUP_PIN));
	}
	err = tegra_pm_irq_set_wake_type(gpio_to_irq(GPIO_WAKEUP_PIN), IRQF_TRIGGER_RISING);
	if (err) {
		pr_err("Failed to set wake type for irq %d\n", gpio_to_irq(GPIO_WAKEUP_PIN));
	}
	else {
		pr_info("Setting wake type to rising\n");
	}
}



/* I2C */
#ifdef CONFIG_MXC_MMA845X
static struct mxc_mma845x_platform_data mma845x_data = {
	.gpio_pin_get = NULL,
	.gpio_pin_put = NULL,
	.int1 = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PB6),
	.int2 = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PU5),
};
#endif

/* GEN1_I2C: I2C_SDA/SCL on SODIMM pin 194/196 (e.g. RTC on carrier board) */
static struct i2c_board_info colibri_t20_i2c_bus1_board_info[] __initdata = {
#ifdef CONFIG_MXC_MMA845X
	{
		I2C_BOARD_INFO("mma845x", 0x1C),
			.platform_data = (void *)&mma845x_data,
	},
#endif
};

static struct tegra_i2c_platform_data colibri_t20_i2c1_platform_data = {
	.adapter_nr	= 0,
	.arb_recovery	= arb_lost_recovery,
	.bus_count	= 1,
	.bus_clk_rate	= {400000, 0},
	.scl_gpio	= {I2C_SCL, 0},
	.sda_gpio	= {I2C_SDA, 0},
	.slave_addr	= 0x00FC,
};

/* GEN2_I2C: unused */

/* DDC_CLOCK/DATA on X3 pin 15/16 (e.g. display EDID) */
static const struct tegra_pingroup_config i2c2_ddc = {
	.pingroup	= TEGRA_PINGROUP_DDC,
	.func		= TEGRA_MUX_I2C2,
};

static struct tegra_i2c_platform_data colibri_t20_i2c2_platform_data = {
	.adapter_nr	= 1,
	.arb_recovery	= arb_lost_recovery,
	.bus_clk_rate	= {10000, 10000},
	.bus_count	= 1,
	.slave_addr	= 0x00FC,
};

/* PWR_I2C: power I2C to PMIC and temperature sensor */

static void lm95245_probe_callback(struct device *dev);

static struct lm95245_platform_data colibri_t20_lm95245_pdata = {
	.enable_os_pin	= true,
	.probe_callback	= lm95245_probe_callback,
};

static struct i2c_board_info colibri_t20_i2c_bus4_board_info[] __initdata = {
	{
		/* LM95245 temperature sensor */
		I2C_BOARD_INFO("lm95245", 0x4c),
			.platform_data = &colibri_t20_lm95245_pdata,
	},
};

static struct tegra_i2c_platform_data colibri_t20_dvc_platform_data = {
	.adapter_nr	= 4,
	.arb_recovery	= arb_lost_recovery,
	.bus_clk_rate	= {400000, 0},
	.bus_count	= 1,
	.is_dvc		= true,
	.scl_gpio	= {PWR_I2C_SCL, 0},
	.sda_gpio	= {PWR_I2C_SDA, 0},
};

static void colibri_t20_i2c_init(void)
{
	tegra_i2c_device1.dev.platform_data = &colibri_t20_i2c1_platform_data;
	tegra_i2c_device2.dev.platform_data = &colibri_t20_i2c2_platform_data;
	tegra_i2c_device4.dev.platform_data = &colibri_t20_dvc_platform_data;

	platform_device_register(&tegra_i2c_device1);
	platform_device_register(&tegra_i2c_device2);
	platform_device_register(&tegra_i2c_device4);

	i2c_register_board_info(0, colibri_t20_i2c_bus1_board_info, ARRAY_SIZE(colibri_t20_i2c_bus1_board_info));
	i2c_register_board_info(4, colibri_t20_i2c_bus4_board_info, ARRAY_SIZE(colibri_t20_i2c_bus4_board_info));
}

/* NAND */
static struct tegra_nand_chip_parms nand_chip_parms[] = {
	/* Micron MT29F4G08ABBDAH4 */
	[0] = {
		.vendor_id		= 0x2C,
		.device_id		= 0xAC,
		.read_id_fourth_byte	= 0x15,
		.capacity		= 512,
		.timing = {
			.trp		= 12,
			.trh		= 10,	/* tREH */
			.twp		= 12,
			.twh		= 10,
			.tcs		= 20,	/* Max(tCS, tCH, tALS, tALH) */
			.twhr		= 80,
			.tcr_tar_trr	= 20,	/* Max(tCR, tAR, tRR) */
			.twb		= 100,
			.trp_resp	= 12,	/* tRP */
			.tadl		= 70,
		},
	},
	/* Micron MT29F4G08ABBEAH4 */
	[1] = {
		.vendor_id		= 0x2C,
		.device_id		= 0xAC,
		.read_id_fourth_byte	= 0x26,
		.capacity		= 512,
		.timing = {
			.trp		= 15,
			.trh		= 10,	/* tREH */
			.twp		= 15,
			.twh		= 10,
			.tcs		= 25,	/* Max(tCS, tCH, tALS, tALH) */
			.twhr		= 80,
			.tcr_tar_trr	= 20,	/* Max(tCR, tAR, tRR) */
			.twb		= 100,
			.trp_resp	= 15,	/* tRP */
			.tadl		= 100,
		},
	},
	/* Micron MT29F8G08ABCBB on Colibri T20 before V1.2 */
	[2] = {
		.vendor_id		= 0x2C,
		.device_id		= 0x38,
		.read_id_fourth_byte	= 0x26,
		.capacity		= 1024,
		.timing = {
			/* timing mode 4 */
			.trp		= 12,
			.trh		= 10,	/* tREH */
			.twp		= 12,
			.twh		= 10,
			.tcs		= 20,	/* Max(tCS, tCH, tALS, tALH) */
			.twhr		= 60,
			.tcr_tar_trr	= 20,	/* Max(tCR, tAR, tRR) */
			.twb		= 100,
			.trp_resp	= 12,	/* tRP */
			.tadl		= 70,
		},
	},
	/* Micron MT29F8G08ADBDAH4 */
	[3] = {
		.vendor_id		= 0x2C,
		.device_id		= 0xA3,
		.read_id_fourth_byte	= 0x15,
		.capacity		= 1024,
		.timing = {
			.trp		= 12,
			.trh		= 10,	/* tREH */
			.twp		= 12,
			.twh		= 10,
			.tcs		= 20,	/* Max(tCS, tCH, tALS, tALH) */
			.twhr		= 80,
			.tcr_tar_trr	= 20,	/* Max(tCR, tAR, tRR) */
			.twb		= 100,
			.trp_resp	= 12,	/* tRP */
			.tadl		= 70,
		},
	},
	/* Micron MT29F8G08ABBCA */
	[4] = {
		.vendor_id		= 0x2C,
		.device_id		= 0xA3,
		.read_id_fourth_byte	= 0x26,
		.capacity		= 1024,
		.timing = {
			.trp		= 15,
			.trh		= 10,	/* tREH */
			.twp		= 15,
			.twh		= 10,
			.tcs		= 25,	/* Max(tCS, tCH, tALS, tALH) */
			.twhr		= 80,
			.tcr_tar_trr	= 20,	/* Max(tCR, tAR, tRR) */
			.twb		= 100,
			.trp_resp	= 15,	/* tRP */
			.tadl		= 100,
		},
	},
	/* Samsung K9K8G08U0B */
	[5] = {
		.vendor_id		= 0xec,
		.device_id		= 0xd3,
		.read_id_fourth_byte	= 0x95,
		.capacity		= 1024,
		.timing			= {
			.trp		= 12,	/* tRP, ND_nRE pulse width */
			.trh		= 100,	/* tRHZ, ND_nRE high duration */
			.twp		= 12,	/* tWP, ND_nWE pulse time */
			.twh		= 10,	/* tWH, ND_nWE high duration */
			.tcs		= 20,	/* Max(tCS, tCH, tALS, tALH) */
			.twhr		= 60,	/* tWHR, ND_nWE high to ND_nRE low delay for
		                                   status read */
			.tcr_tar_trr	= 20,	/* Max(tCR, tAR, tRR) */
			.twb		= 100,
			.trp_resp	= 12,	/* tRP */
			.tadl		= 70,
		},
	},
};

struct tegra_nand_platform colibri_t20_nand_data = {
	.max_chips	= 8,
	.chip_parms	= nand_chip_parms,
	.nr_chip_parms	= ARRAY_SIZE(nand_chip_parms),
	.wp_gpio	= NAND_WP_N,
};

static struct resource resources_nand[] = {
	[0] = {
		.start	= INT_NANDFLASH,
		.end	= INT_NANDFLASH,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct platform_device tegra_nand_device = {
	.name		= "tegra_nand",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(resources_nand),
	.resource	= resources_nand,
	.dev = {
		.platform_data = &colibri_t20_nand_data,
	},
};

/* VCB LED:s */
static struct gpio_led status_leds[] = {
	[0] =  {
		/* Global on switch for LEDs */
		.name = "led-on-2",
		.gpio = (TEGRA_GPIO_PAA1),
		.active_low = 0,
		.default_state = LEDS_GPIO_DEFSTATE_ON,
	},
	[1] =  {
		.name = "led-vehicle",
		.gpio = (TEGRA_GPIO_PD6),
		.active_low = 0,
		.default_state = LEDS_GPIO_DEFSTATE_OFF,
	},
	[2] =  {
		.name = "led-usb",
		.gpio = (TEGRA_GPIO_PL2),
		.active_low = 0,
		.default_state = LEDS_GPIO_DEFSTATE_OFF,
	},
};

static struct gpio_led_platform_data status_led_data = {
	.num_leds	= ARRAY_SIZE(status_leds),
	.leds		= status_leds
};

static struct platform_device status_led_dev = {
	.name		= "leds-gpio",
	.id		= -1,
	.dev		= {
		.platform_data	= &status_led_data,
	},
};

/* RTC */
#ifdef CONFIG_RTC_DRV_TEGRA
static struct resource tegra_rtc_resources[] = {
	[0] = {
		.start	= TEGRA_RTC_BASE,
		.end	= TEGRA_RTC_BASE + TEGRA_RTC_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= INT_RTC,
		.end	= INT_RTC,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct platform_device tegra_rtc_device = {
	.name		= "tegra_rtc",
	.id		= -1,
	.resource	= tegra_rtc_resources,
	.num_resources	= ARRAY_SIZE(tegra_rtc_resources),
};
#endif /* CONFIG_RTC_DRV_TEGRA */

/* SPI */

#if defined(CONFIG_SPI_TEGRA) && defined(CONFIG_SPI_SPIDEV)

static struct mx4_io_platform_data mx4_io_pdata = {
	.event_rdy = -1,
};

static struct spi_board_info tegra_spi_devices[] __initdata = {
	{
		.bus_num	= 3,		/* SPI4 */
		.chip_select	= 0,
		.irq		= TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PY6),
		.max_speed_hz	= 5000000,
		.modalias	= "mx4_io_spi",
		.mode		= SPI_MODE_1,
		.platform_data	= &mx4_io_pdata,
	},
	{
		.bus_num	= 3,
		.chip_select	= 1,
		.irq		= 0,
		.max_speed_hz	= 50000000,
		.modalias	= "spidev",
		.mode		= SPI_MODE_1,
		.platform_data	= NULL,
	},
};

static void __init colibri_t20_register_spidev(void)
{
	spi_register_board_info(tegra_spi_devices,
				ARRAY_SIZE(tegra_spi_devices));
}
#else /* CONFIG_SPI_TEGRA & CONFIG_SPI_SPIDEV */
#define colibri_t20_register_spidev() do {} while (0)
#endif /* CONFIG_SPI_TEGRA & CONFIG_SPI_SPIDEV */

/* Thermal throttling
   Note: As our hardware only allows triggering an interrupt on
	 over-temperature shutdown we first use it to catch entering throttle
	 and only then set it up to catch an actual over-temperature shutdown.
	 While throttling we setup a workqueue to catch leaving it again. */

static int colibri_t20_shutdown_temp = 115000;
static int colibri_t20_throttle_hysteresis = 3000;
static int colibri_t20_throttle_temp = 90000;
static struct device *lm95245_device = NULL;
static int thermd_alert_irq_disabled = 0;
struct work_struct thermd_alert_work;
struct workqueue_struct *thermd_alert_workqueue;

/* Over-temperature shutdown OS pin GPIO interrupt handler */
static irqreturn_t thermd_alert_irq(int irq, void *data)
{
	disable_irq_nosync(irq);
	thermd_alert_irq_disabled = 1;
	queue_work(thermd_alert_workqueue, &thermd_alert_work);

	return IRQ_HANDLED;
}

/* Gets both entered by THERMD_ALERT GPIO interrupt as well as re-scheduled
   while throttling. */
static void thermd_alert_work_func(struct work_struct *work)
{
	int temp = 0;

	lm95245_get_remote_temp(lm95245_device, &temp);

	if (temp > colibri_t20_shutdown_temp) {
		/* First check for hardware over-temperature condition mandating
		   immediate shutdown */
		pr_err("over-temperature condition %d degC reached, initiating "
				"immediate shutdown", temp);
		kernel_power_off();
	} else if (temp < colibri_t20_throttle_temp - colibri_t20_throttle_hysteresis) {
		/* Make sure throttling gets disabled again */
		if (tegra_is_throttling()) {
			tegra_throttling_enable(false);
			lm95245_set_remote_os_limit(lm95245_device, colibri_t20_throttle_temp);
		}
	} else if (temp < colibri_t20_throttle_temp) {
		/* Operating within hysteresis so keep re-scheduling to catch
		   leaving below throttle again */
		if (tegra_is_throttling()) {
			msleep(100);
			queue_work(thermd_alert_workqueue, &thermd_alert_work);
		}
	} else if (temp >= colibri_t20_throttle_temp) {
		/* Make sure throttling gets enabled and set shutdown limit */
		if (!tegra_is_throttling()) {
			tegra_throttling_enable(true);
			lm95245_set_remote_os_limit(lm95245_device, colibri_t20_shutdown_temp);
		}
		/* And re-schedule again */
		msleep(100);
		queue_work(thermd_alert_workqueue, &thermd_alert_work);
	}

	/* Avoid unbalanced enable for IRQ 367 */
	if (thermd_alert_irq_disabled) {
		thermd_alert_irq_disabled = 0;
		enable_irq(TEGRA_GPIO_TO_IRQ(THERMD_ALERT));
	}
}

static void colibri_t20_thermd_alert_init(void)
{
	gpio_request(THERMD_ALERT, "THERMD_ALERT");
	gpio_direction_input(THERMD_ALERT);

	thermd_alert_workqueue = create_singlethread_workqueue("THERMD_ALERT");

	INIT_WORK(&thermd_alert_work, thermd_alert_work_func);
}

static void lm95245_probe_callback(struct device *dev)
{
	lm95245_device = dev;

	lm95245_set_remote_os_limit(lm95245_device, colibri_t20_throttle_temp);

	if (request_irq(TEGRA_GPIO_TO_IRQ(THERMD_ALERT), thermd_alert_irq,
			IRQF_TRIGGER_LOW, "THERMD_ALERT", NULL))
		pr_err("%s: unable to register THERMD_ALERT interrupt\n", __func__);
}

#ifdef CONFIG_DEBUG_FS
static int colibri_t20_thermal_get_throttle_temp(void *data, u64 *val)
{
	*val = (u64)colibri_t20_throttle_temp;
	return 0;
}

static int colibri_t20_thermal_set_throttle_temp(void *data, u64 val)
{
	colibri_t20_throttle_temp = val;
	if (!tegra_is_throttling() && lm95245_device)
		lm95245_set_remote_os_limit(lm95245_device, colibri_t20_throttle_temp);

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(throttle_fops,
			colibri_t20_thermal_get_throttle_temp,
			colibri_t20_thermal_set_throttle_temp,
			"%llu\n");

static int colibri_t20_thermal_get_shutdown_temp(void *data, u64 *val)
{
	*val = (u64)colibri_t20_shutdown_temp;
	return 0;
}

static int colibri_t20_thermal_set_shutdown_temp(void *data, u64 val)
{
	colibri_t20_shutdown_temp = val;
	if (tegra_is_throttling() && lm95245_device)
		lm95245_set_remote_os_limit(lm95245_device, colibri_t20_shutdown_temp);

	/* Carefull as we can only actively monitor one temperatur limit and
	   assumption is throttling is lower than shutdown one. */
	if (colibri_t20_shutdown_temp < colibri_t20_throttle_temp)
		colibri_t20_thermal_set_throttle_temp(NULL, colibri_t20_shutdown_temp);

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(shutdown_fops,
			colibri_t20_thermal_get_shutdown_temp,
			colibri_t20_thermal_set_shutdown_temp,
			"%llu\n");

static int __init colibri_t20_thermal_debug_init(void)
{
	struct dentry *thermal_debugfs_root;

	thermal_debugfs_root = debugfs_create_dir("thermal", 0);

	if (!debugfs_create_file("throttle", 0644, thermal_debugfs_root,
					NULL, &throttle_fops))
		return -ENOMEM;

	if (!debugfs_create_file("shutdown", 0644, thermal_debugfs_root,
					NULL, &shutdown_fops))
		return -ENOMEM;

	return 0;
}
late_initcall(colibri_t20_thermal_debug_init);
#endif /* CONFIG_DEBUG_FS */

static struct platform_device *colibri_t20_uart_devices[] __initdata = {
/*
	MX-4
	---
	UARTA(ttyHS0) - J1708
	UARTB(ttyHS1) - RS485
	UARTC(ttyHS2) - LIN
	UARTD(ttyHS3) - RS232/CTS/RTS

	VCB
	---
	UARTA - Not used
	UARTB(ttyHS1) - J1708
	UARTC(ttyHS2) - K-Line
	UARTD(ttyHS3) - RS232/RS485

*/
	&tegra_uarte_device,
	&tegra_uartb_device,
	&tegra_uartc_device,
	&tegra_uartd_device,
};

static struct uart_clk_parent uart_parent_clk[] = {
	[0] = {.name = "pll_p"},
	[1] = {.name = "pll_m"},
	[2] = {.name = "pll_p"},
	[3] = {.name = "pll_p"},
	[4] = {.name = "clk_m"},
};

static struct tegra_uart_platform_data colibri_t20_uart_pdata;

static void __init uart_debug_init(void)
{
	unsigned long rate;
	struct clk *c;

	/* UARTE is the debug port. */
	pr_info("Selecting UARTE as the debug console\n");
	colibri_t20_uart_devices[0] = &debug_uarte_device;
	debug_uart_port_base = ((struct plat_serial8250_port *)(
			debug_uarte_device.dev.platform_data))->mapbase;
	debug_uart_clk = clk_get_sys("serial8250.0", "uarte");

	/* Clock enable for the debug channel */
	if (!IS_ERR_OR_NULL(debug_uart_clk)) {
		rate = ((struct plat_serial8250_port *)(
			debug_uarte_device.dev.platform_data))->uartclk;
		pr_info("The debug console clock name is %s\n",
						debug_uart_clk->name);
		c = tegra_get_clock_by_name("pll_p");
		if (IS_ERR_OR_NULL(c))
			pr_err("Not getting the parent clock pll_p\n");
		else
			clk_set_parent(debug_uart_clk, c);

		clk_enable(debug_uart_clk);
		clk_set_rate(debug_uart_clk, rate);
	} else {
		pr_err("Not getting the clock %s for debug console\n",
					debug_uart_clk->name);
	}
}

static void __init colibri_t20_uart_init(void)
{
	int i;
	struct clk *c;

	for (i = 0; i < ARRAY_SIZE(uart_parent_clk); ++i) {
		c = tegra_get_clock_by_name(uart_parent_clk[i].name);
		if (IS_ERR_OR_NULL(c)) {
			pr_err("Not able to get the clock for %s\n",
						uart_parent_clk[i].name);
			continue;
		}
		uart_parent_clk[i].parent_clk = c;
		uart_parent_clk[i].fixed_clk_rate = clk_get_rate(c);
	}
	colibri_t20_uart_pdata.parent_clk_list = uart_parent_clk;
	colibri_t20_uart_pdata.parent_clk_count = ARRAY_SIZE(uart_parent_clk);
	tegra_uarte_device.dev.platform_data = &colibri_t20_uart_pdata;
	tegra_uarta_device.dev.platform_data = &colibri_t20_uart_pdata;
	tegra_uartb_device.dev.platform_data = &colibri_t20_uart_pdata;
	tegra_uartc_device.dev.platform_data = &colibri_t20_uart_pdata;
	tegra_uartd_device.dev.platform_data = &colibri_t20_uart_pdata;

	/* Register low speed only if it is selected */
	if (!is_tegra_debug_uartport_hs())
		uart_debug_init();

	platform_add_devices(colibri_t20_uart_devices,
				ARRAY_SIZE(colibri_t20_uart_devices));
}

/* USB */

//overcurrent?

//TODO: overcurrent
#ifdef CONFIG_USB_GADGET
static struct tegra_usb_platform_data tegra_udc_pdata = {
	.has_hostpc	= false,
	.op_mode	= TEGRA_USB_OPMODE_DEVICE,
	.phy_intf	= TEGRA_USB_PHY_INTF_UTMI,
	.port_otg	= false,
	.u_cfg.utmi = {
		.elastic_limit		= 16,
		.hssync_start_delay	= 0,
		.idle_wait_delay	= 17,
		.term_range_adj		= 6,
		.xcvr_lsfslew		= 2,
		.xcvr_lsrslew		= 2,
		.xcvr_setup		= 8,
		.xcvr_setup_offset	= 0,
		.xcvr_use_fuses		= 1,
	},
	.u_data.dev = {
		.charging_supported		= false,
		.remote_wakeup_supported	= false,
		.vbus_gpio			= -1,
		.vbus_pmu_irq			= 0,
	},
};
#endif /* CONFIG_USB_GADGET */

static void ulpi_link_platform_open(void)
{
	int reset_gpio = USB3340_RESETB;

	gpio_request(reset_gpio, "ulpi_phy_reset");
	gpio_direction_output(reset_gpio, 0);
	msleep(5);
	gpio_direction_output(reset_gpio, 1);
}

static void ulpi_link_platform_post_phy_on(void)
{
	/* enable VBUS */
	gpio_set_value(LAN_V_BUS, 1);

	/* reset */
	gpio_set_value(LAN_RESET, 0);

	udelay(5);

	/* unreset */
	gpio_set_value(LAN_RESET, 1);
}

static void ulpi_link_platform_pre_phy_off(void)
{
	/* disable VBUS */
	gpio_set_value(LAN_V_BUS, 0);
}

static struct tegra_usb_phy_platform_ops ulpi_link_plat_ops = {
	.open = ulpi_link_platform_open,
	.post_phy_on = ulpi_link_platform_post_phy_on,
	.pre_phy_off = ulpi_link_platform_pre_phy_off,
};

static struct tegra_usb_platform_data tegra_ehci2_ulpi_link_pdata = {
	.has_hostpc	= false,
	.op_mode	= TEGRA_USB_OPMODE_HOST,
	.ops		= &ulpi_link_plat_ops,
	.phy_intf	= TEGRA_USB_PHY_INTF_ULPI_LINK,
	.port_otg	= false,
	.u_cfg.ulpi = {
		.clk			= "cdev2",
		.clock_out_delay	= 1,
		.data_trimmer		= 4,
		.dir_trimmer		= 4,
		.shadow_clk_delay	= 10,
		.stpdirnxt_trimmer	= 4,
	},
	.u_data.host = {
		.hot_plug			= false,
		.power_off_on_suspend		= true,
		.remote_wakeup_supported	= false,
		.vbus_gpio			= -1,
		.vbus_reg			= NULL,
	},
};

static struct tegra_usb_platform_data tegra_ehci3_utmi_pdata = {
	.has_hostpc	= false,
	.op_mode	= TEGRA_USB_OPMODE_HOST,
	.phy_intf	= TEGRA_USB_PHY_INTF_UTMI,
	.port_otg	= false,
	.u_cfg.utmi = {
		.elastic_limit		= 16,
		.hssync_start_delay	= 9,
		.idle_wait_delay	= 17,
		.term_range_adj		= 6,
		.xcvr_lsfslew		= 2,
		.xcvr_lsrslew		= 2,
		.xcvr_setup		= 8,
	},
	.u_data.host = {
		.hot_plug			= true,
		.power_off_on_suspend		= false,
		.remote_wakeup_supported	= false,
		.vbus_gpio			= USBH_PEN,
		.vbus_gpio_inverted		= 0,
		.vbus_reg			= NULL,
	},
};

static void colibri_t20_usb_init(void)
{
	gpio_request(LAN_V_BUS, "LAN_V_BUS");
	gpio_direction_output(LAN_V_BUS, 0);
	gpio_export(LAN_V_BUS, false);

	gpio_request(LAN_RESET, "LAN_RESET");
	gpio_direction_output(LAN_RESET, 0);
	gpio_export(LAN_RESET, false);

#ifdef CONFIG_USB_GADGET
	/* OTG should be the first to be registered
	   EHCI instance 0: USB1_DP/N -> USBOTG_P/N */
	/* setup the udc platform data */
	tegra_udc_device.dev.platform_data = &tegra_udc_pdata;
	platform_device_register(&tegra_udc_device);
#endif /* CONFIG_USB_GADGET */
	/* EHCI instance 1: ULPI PHY -> ASIX ETH */
	tegra_ehci2_device.dev.platform_data = &tegra_ehci2_ulpi_link_pdata;
	platform_device_register(&tegra_ehci2_device);

	/* EHCI instance 2: USB3_DP/N -> USBH1_P/N */
	tegra_ehci3_device.dev.platform_data = &tegra_ehci3_utmi_pdata;
	platform_device_register(&tegra_ehci3_device);

}

static struct platform_device *colibri_t20_devices[] __initdata = {
#ifdef CONFIG_RTC_DRV_TEGRA
	&tegra_rtc_device,
#endif
	&tegra_nand_device,
	&tegra_pmu_device,
	&tegra_wdt_device,
	&spdif_dit_device,
	&tegra_spi_device4,
	&status_led_dev,
};

static void __init colibri_t20_init(void)
{
#if defined(CONFIG_CAN_SJA1000) || defined(CONFIG_CAN_SJA1000_MODULE)
	vcb_sja1000_init();
#endif /* CONFIG_CAN_SJA1000 || CONFIG_CAN_SJA1000_MODULE */

	tegra_clk_init_from_table(colibri_t20_clk_init_table);
	colibri_t20_pinmux_init();
	colibri_t20_thermd_alert_init();
	colibri_t20_i2c_init();
	colibri_t20_uart_init();

	platform_add_devices(colibri_t20_devices,
			     ARRAY_SIZE(colibri_t20_devices));
	tegra_ram_console_debug_init();
	colibri_t20_regulator_init();

	colibri_t20_usb_init();

	/* Note: V1.1c modules require proper BCT setting 666 rather than
	   721.5 MHz EMC clock */
	colibri_t20_emc_init();

	colibri_t20_gpio_init();
	colibri_t20_register_spidev();

	tegra_release_bootloader_fb();

	/* Show platform version */
	pr_info("Host Mobility VCB\n");
}

int __init tegra_colibri_t20_protected_aperture_init(void)
{
	if (!machine_is_colibri_t20())
		return 0;

	tegra_protected_aperture_init(tegra_grhost_aperture);
	return 0;
}
late_initcall(tegra_colibri_t20_protected_aperture_init);

void __init colibri_t20_reserve(void)
{
	if (memblock_reserve(0x0, 4096) < 0)
		pr_warn("Cannot reserve first 4K of memory for safety\n");

	/* we specify zero for special handling due to already reserved
	   fbmem/nvmem (U-Boot 2011.06 compatibility from our V1.x images) */
	tegra_reserve(0, 0, 0);
	tegra_ram_console_debug_reserve(SZ_1M);
}

static const char *colibri_t20_dt_board_compat[] = {
	"toradex,colibri_t20",
	NULL
};

#ifdef CONFIG_ANDROID
MACHINE_START(COLIBRI_T20, "ventana")
#else
MACHINE_START(COLIBRI_T20, "Toradex Colibri T20")
#endif
	.boot_params	= 0x00000100,
	.dt_compat	= colibri_t20_dt_board_compat,
	.init_early	= tegra_init_early,
	.init_irq	= tegra_init_irq,
	.init_machine	= colibri_t20_init,
	.map_io		= tegra_map_common_io,
	.reserve	= colibri_t20_reserve,
	.timer		= &tegra_timer,
MACHINE_END
