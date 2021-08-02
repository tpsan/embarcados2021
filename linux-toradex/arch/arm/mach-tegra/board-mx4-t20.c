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
#include <linux/irq.h>
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
#include <linux/wm97xx.h>
#include <linux/mma845x.h>

#include <mach/gpio.h>
#include <mach/mx4_iomap.h>
#include <mach/nand.h>
#include <mach/sdhci.h>
#include <mach/usb_phy.h>
#include <mach/w1.h>

#include "board-mx4-t20.h"
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
#include "wakeups.h"

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

 #define L3G4200D_DRDY_GPIO      TEGRA_GPIO_PA7

static struct wm97xx_batt_pdata colibri_t20_adc_pdata = {
	.batt_aux	= WM97XX_AUX_ID1,	/* AD0 - ANALOG_IN0 */
	.temp_aux	= WM97XX_AUX_ID2,	/* AD1 - ANALOG_IN1 */
	.charge_gpio	= -1,
	.batt_div	= 1,
	.batt_mult	= 1,
	.temp_div	= 1,
	.temp_mult	= 1,
	.batt_name	= "colibri_t20-analog_inputs",
};

static struct wm97xx_pdata colibri_t20_wm97xx_pdata = {
	.batt_pdata = &colibri_t20_adc_pdata,
};

/* Audio */
#ifdef CONFIG_SOUND
static struct platform_device colibri_t20_audio_device = {
	.name	= "colibri_t20-snd-wm9715l",
	.id	= 0,
};

void *get_colibri_t20_audio_platform_data(void)
{
	return &colibri_t20_wm97xx_pdata;
}
EXPORT_SYMBOL(get_colibri_t20_audio_platform_data);
#endif /* CONFIG_SOUND */

#ifdef COLIBRI_T20_VI
/* Camera */
static struct platform_device tegra_camera = {
	.name	= "tegra_camera",
	.id	= -1,
};
#endif /* COLIBRI_T20_VI */

/* CAN */

#if defined(CONFIG_CAN_SJA1000) || defined(CONFIG_CAN_SJA1000_MODULE)
static struct resource colibri_can_resource[] = {
	[0] =   {
		.start	= TEGRA_CAN_BASE,		/* address */
		.end	= TEGRA_CAN_BASE + TEGRA_CAN_SIZE,/* data */
#ifndef CONFIG_HM_GMI_MUX
		.flags	= IORESOURCE_MEM,
#else
		.flags	= IORESOURCE_MEM | IORESOURCE_MEM_32BIT,
#endif /* !CONFIG_HM_GMI_MUX*/
	},
	[1] =   {
		/* interrupt assigned during initialisation */
		.flags	= IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHEDGE,
	}
};

static struct sja1000_platform_data colibri_can_platdata = {
	.osc_freq	= 24000000,
	.ocr		= (OCR_MODE_NORMAL | OCR_TX0_PUSHPULL),
#ifdef CONFIG_HM_DAISY_CHAIN_CAN
	.cdr		= CDR_CLKOUT_MASK |  /* Set CLKOUT to Fosc */
			  CDR_CBP, /* CAN input comparator bypass */
#else
	.cdr		= CDR_CLK_OFF | /* Clock off (CLKOUT pin) */
			  CDR_CBP, /* CAN input comparator bypass */
#endif /* CONFIG_HM_DAISY_CHAIN_CAN */
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

/* ********************************CAN DEVICE 2*******************************/
static struct resource colibri_can_resource2[] = {
	[0] =   {
		.start	= TEGRA_CAN2_BASE, 	/* address */
		.end	= TEGRA_CAN2_BASE + TEGRA_CAN_SIZE, /* data */
#ifndef CONFIG_HM_GMI_MUX
		.flags	= IORESOURCE_MEM,
#else
		.flags	= IORESOURCE_MEM | IORESOURCE_MEM_32BIT,
#endif /* !CONFIG_HM_GMI_MUX*/
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


#ifdef CONFIG_HM_GTT_CAN /* CAN 3-6 */
/* ********************************CAN DEVICE 3*******************************/
static struct resource colibri_can_resource3[] = {
	[0] =   {
		.start	= TEGRA_CAN3_BASE, 	/* address */
		.end	= TEGRA_CAN3_BASE + TEGRA_CAN_SIZE, /* data */
#ifndef CONFIG_HM_GMI_MUX
		.flags	= IORESOURCE_MEM,
#else
		.flags	= IORESOURCE_MEM | IORESOURCE_MEM_32BIT,
#endif /* !CONFIG_HM_GMI_MUX*/
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

/* ********************************CAN DEVICE 4*******************************/
static struct resource colibri_can_resource4[] = {
	[0] =   {
		.start	= TEGRA_CAN4_BASE, 	/* address */
		.end	= TEGRA_CAN4_BASE + TEGRA_CAN_SIZE, /* data */
#ifndef CONFIG_HM_GMI_MUX
		.flags	= IORESOURCE_MEM,
#else
		.flags	= IORESOURCE_MEM | IORESOURCE_MEM_32BIT,
#endif /* !CONFIG_HM_GMI_MUX*/
	},
	[1] =   {
		/* interrupt assigned during initialisation */
		.flags	= IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHEDGE,
	}
};

static struct sja1000_platform_data colibri_can_platdata4 = {
	.osc_freq	= 24000000,
	.ocr		= (OCR_MODE_NORMAL | OCR_TX0_PUSHPULL),
	.cdr		= CDR_CLK_OFF | /* Clock off (CLKOUT pin) */
			  CDR_CBP, /* CAN input comparator bypass */
};

static struct platform_device colibri_can_device4 = {
	.name		= "sja1000_platform",
	.id		= 3,
	.num_resources	= ARRAY_SIZE(colibri_can_resource4),
	.resource	= colibri_can_resource4,
	.dev            = {
		.platform_data = &colibri_can_platdata4,
	}
};

/* ********************************CAN DEVICE 5*******************************/
static struct resource colibri_can_resource5[] = {
	[0] =   {
		.start	= TEGRA_CAN5_BASE, 	/* address */
		.end	= TEGRA_CAN5_BASE + TEGRA_CAN_SIZE, /* data */
#ifndef CONFIG_HM_GMI_MUX
		.flags	= IORESOURCE_MEM,
#else
		.flags	= IORESOURCE_MEM | IORESOURCE_MEM_32BIT,
#endif /* !CONFIG_HM_GMI_MUX*/
	},
	[1] =   {
		/* interrupt assigned during initialisation */
		.flags	= IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHEDGE,
	}
};

static struct sja1000_platform_data colibri_can_platdata5 = {
	.osc_freq	= 24000000,
	.ocr		= (OCR_MODE_NORMAL | OCR_TX0_PUSHPULL),
	.cdr		= CDR_CLK_OFF | /* Clock off (CLKOUT pin) */
			  CDR_CBP, /* CAN input comparator bypass */
};

static struct platform_device colibri_can_device5 = {
	.name		= "sja1000_platform",
	.id		= 4,
	.num_resources	= ARRAY_SIZE(colibri_can_resource5),
	.resource	= colibri_can_resource5,
	.dev            = {
		.platform_data = &colibri_can_platdata5,
	}
};

static struct resource colibri_can_resource6[] = {
	[0] =   {
		.start	= TEGRA_CAN6_BASE, 	/* address */
		.end	= TEGRA_CAN6_BASE + TEGRA_CAN_SIZE, /* data */
#ifndef CONFIG_HM_GMI_MUX
		.flags	= IORESOURCE_MEM,
#else
		.flags	= IORESOURCE_MEM | IORESOURCE_MEM_32BIT,
#endif /* !CONFIG_HM_GMI_MUX*/
	},
	[1] =   {
		/* interrupt assigned during initialisation */
		.flags	= IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHEDGE,
	}
};

static struct sja1000_platform_data colibri_can_platdata6 = {
	.osc_freq	= 24000000,
	.ocr		= (OCR_MODE_NORMAL | OCR_TX0_PUSHPULL),
	.cdr		= CDR_CLK_OFF | /* Clock off (CLKOUT pin) */
			  CDR_CBP, /* CAN input comparator bypass */
};

static struct platform_device colibri_can_device6 = {
	.name		= "sja1000_platform",
	.id		= 5,
	.num_resources	= ARRAY_SIZE(colibri_can_resource6),
	.resource	= colibri_can_resource6,
	.dev            = {
		.platform_data = &colibri_can_platdata6,
	}
};

#endif /* CONFIG_HM_GTT_CAN */

#endif /* CONFIG_CAN_SJA1000 || CONFIG_CAN_SJA1000_MODULE */


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

static struct gpio colibri_t20_gpios[] = {
	/* Not connected pins */

#ifndef CONFIG_HM_GMI_MUX
	//Might conflict with Volume up key
	{TEGRA_GPIO_PBB4,	(GPIOF_IN | GPIOF_NO_EXPORT),	"P22 - NC"},
	//Might conflict with Volume down key
	{TEGRA_GPIO_PBB5,	(GPIOF_IN | GPIOF_NO_EXPORT),		"P24 - NC"},
#endif

	{TEGRA_GPIO_PL7,	(GPIOF_IN | GPIOF_NO_EXPORT),		"P65 - NC"},
	//Used as ACC int
	//{TEGRA_GPIO_PK4,	(GPIOF_IN | GPIOF_NO_EXPORT),		"P106 - NC"},
	{TEGRA_GPIO_PK1,	(GPIOF_IN | GPIOF_NO_EXPORT),		"P152 - NC"},

#ifndef CONFIG_HM_GMI_MUX
	// Used as ACC int
	{TEGRA_GPIO_PU5,	(GPIOF_IN | GPIOF_NO_EXPORT),		"P116 - NC"}, // Wake up
#endif

	{TEGRA_GPIO_PU6,	(GPIOF_IN | GPIOF_NO_EXPORT),		"P118 - NC"}, //Wake up
	// Used by BL_ON (see board-colibri_t20-panel.c)
	//{TEGRA_GPIO_PP4,	(GPIOF_IN | GPIOF_NO_EXPORT),		"P120 - NC"},
	//Might conflict with ADDRESS14
	{TEGRA_GPIO_PP5,	(GPIOF_IN | GPIOF_NO_EXPORT),		"P122 - NC"},
	//Might conflict with ADDRESS15
	{TEGRA_GPIO_PP6,	(GPIOF_IN | GPIOF_NO_EXPORT),		"P124 - NC"},
	{TEGRA_GPIO_PJ0,	(GPIOF_IN | GPIOF_NO_EXPORT),		"P126 - NC"},
	{TEGRA_GPIO_PJ2,	(GPIOF_IN | GPIOF_NO_EXPORT),		"P128 - NC"},
	{TEGRA_GPIO_PI3,	(GPIOF_IN | GPIOF_NO_EXPORT),		"P130 - NC"},
	{TEGRA_GPIO_PI6,	(GPIOF_IN | GPIOF_NO_EXPORT),		"P132 - NC"},
    /* GMI_IORDY multiplexed GMI_WAIT/GMI_IORDY in pinmux - not used */
	//{TEGRA_GPIO_PI7,	(GPIOF_IN | GPIOF_NO_EXPORT),		"P95 - NC"},
	{TEGRA_GPIO_PI5,	(GPIOF_IN | GPIOF_NO_EXPORT),		"P95 - NC"},
	//Pin 136, 138, 140, 142 Muxed to PM2 et al in pinmux (SPI2). Currently not used
	{TEGRA_GPIO_PX3,	(GPIOF_IN | GPIOF_NO_EXPORT),		"P136 - NC"},
	{TEGRA_GPIO_PX2,	(GPIOF_IN | GPIOF_NO_EXPORT),		"P138 - NC"},
	{TEGRA_GPIO_PN5,	(GPIOF_IN | GPIOF_NO_EXPORT),		"P158 - NC"},
	{TEGRA_GPIO_PN4,	(GPIOF_IN | GPIOF_NO_EXPORT),		"P160 - NC"},
	{TEGRA_GPIO_PZ2,	(GPIOF_IN | GPIOF_NO_EXPORT),		"P156 - NC"},
	{TEGRA_GPIO_PZ4,	(GPIOF_IN | GPIOF_NO_EXPORT),		"P164 - NC"},
	{TEGRA_GPIO_PAA4,	(GPIOF_IN | GPIOF_NO_EXPORT),		"P166 - NC"},
	{TEGRA_GPIO_PAA5,	(GPIOF_IN | GPIOF_NO_EXPORT),		"P168 - NC"},
	{TEGRA_GPIO_PAA7,	(GPIOF_IN | GPIOF_NO_EXPORT),		"P172 - NC"},

#ifndef CONFIG_HM_GMI_MUX
	{TEGRA_GPIO_PP7,	(GPIOF_IN | GPIOF_NO_EXPORT),		"P188 - NC"},
	{TEGRA_GPIO_PA3,	(GPIOF_IN | GPIOF_NO_EXPORT),		"P184 - NC"},
	{TEGRA_GPIO_PA2,	(GPIOF_IN | GPIOF_NO_EXPORT),		"P186 - NC"},
	{TEGRA_GPIO_PA5,	(GPIOF_IN | GPIOF_NO_EXPORT),		"P144 - NC"},
	{TEGRA_GPIO_PA4,	(GPIOF_IN | GPIOF_NO_EXPORT),		"P146 - NC"},
	{TEGRA_GPIO_PX1,	(GPIOF_IN | GPIOF_NO_EXPORT),		"P140 - NC"},
	{TEGRA_GPIO_PX0,	(GPIOF_IN | GPIOF_NO_EXPORT),		"P142 - NC"},
#endif
	/* Digital inputs */
	// P45 is used for CF in PXA. Consider change.

#ifdef CONFIG_HM_DIGITAL_INPUTS
	#ifndef CONFIG_HM_GMI_MUX
		{TEGRA_GPIO_PC1,	(GPIOF_IN | GPIOF_ACT_LOW),		"P29 - DIGITAL-IN-1"},
		{TEGRA_GPIO_PC7,	(GPIOF_IN | GPIOF_ACT_LOW),		"P43 - DIGITAL-IN-2"},
		{TEGRA_GPIO_PB6,	(GPIOF_IN | GPIOF_ACT_LOW),		"P55 - DIGITAL-IN-3"},
		{TEGRA_GPIO_PB4,	(GPIOF_IN | GPIOF_ACT_LOW),		"P59 - DIGITAL-IN-4"},
		{TEGRA_GPIO_PZ0,	(GPIOF_IN | GPIOF_ACT_LOW),		"P23 - DIGITAL-IN-5"},
		{TEGRA_GPIO_PZ1,	(GPIOF_IN | GPIOF_ACT_LOW),		"P25 - DIGITAL-IN-6"},
	#else
		{TEGRA_GPIO_PU2,	(GPIOF_IN | GPIOF_ACT_LOW),		"P110 - DIGITAL-IN-1"},
		{TEGRA_GPIO_PC7,	(GPIOF_IN | GPIOF_ACT_LOW),		"P43 - DIGITAL-IN-2"},
		{TEGRA_GPIO_PV3,	(GPIOF_IN | GPIOF_ACT_LOW),		"P45 - DIGITAL-IN-3"},
		{TEGRA_GPIO_PBB5,	(GPIOF_IN | GPIOF_ACT_LOW),		"P24 - DIGITAL-IN-4"},
		{TEGRA_GPIO_PBB4,	(GPIOF_IN | GPIOF_ACT_LOW),		"P22 - DIGITAL-IN-5"},
		{TEGRA_GPIO_PZ1,	(GPIOF_IN | GPIOF_ACT_LOW),		"P25 - DIGITAL-IN-6"},
	#endif
#endif /* CONFIG_HM_DIGITAL_INPUTS */

#ifdef CONFIG_MACH_HM_MX4_GTT
	{TEGRA_GPIO_PK6,	(GPIOF_IN ),                	"P135 - MODEM-WAKEUP"},
#else
	{TEGRA_GPIO_PW2,	(GPIOF_IN ),                	"P129 - MODEM-WAKEUP"},
#endif /* CONFIG_MACH_HM_MX4_GTT */

	{TEGRA_GPIO_PC6,	(GPIOF_IN ),                	"P31 - XANTSHORT"},

#ifndef CONFIG_HM_GMI_MUX
	{TEGRA_GPIO_PK0,	(GPIOF_IN | GPIOF_NO_EXPORT),		"P150 - MM_PXA300_CMD"},
#endif /* !CONFIG_HM_GMI_MUX */

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

	/* Extern UART Interrupts*/
#ifdef CONFIG_HM_EXT_8250_UART
	{TEGRA_GPIO_PT2,	(GPIOF_IN | GPIOF_NO_EXPORT),		"P69 - UART-INTA"},
	{TEGRA_GPIO_PBB2,	(GPIOF_IN | GPIOF_NO_EXPORT),		"P133 - UART-INTB"},
#ifdef CONFIG_MACH_HM_MX4_VCC
	{TEGRA_GPIO_PU3,	(GPIOF_IN | GPIOF_NO_EXPORT),		"P112 - UART-INTC"},
#else
	{TEGRA_GPIO_PK5,	(GPIOF_IN | GPIOF_NO_EXPORT),		"P137 - UART-INTC"},
	{TEGRA_GPIO_PX4,	(GPIOF_IN | GPIOF_NO_EXPORT),		"P134 - NC"},
#endif /* CONFIG_MACH_HM_MX4_VCC */
#endif /* CONFIG_HM_EXT_8250_UART */

/* Wakeup of external ethernet interface */
#ifdef CONFIG_HM_EXT_AX88772B
	{TEGRA_GPIO_PAA2,	(GPIOF_DIR_OUT | GPIOF_INIT_LOW | GPIOF_ACT_LOW),		"P51 - OPT1-WAKE"},
	{TEGRA_GPIO_PAA3,	(GPIOF_DIR_OUT | GPIOF_INIT_LOW | GPIOF_ACT_LOW),		"P53 - OPT2-WAKE"},
#endif /* CONFIG_HM_EXT_AX88772B */

	/* Gyro Interrupts */
	/* Our gyro driver does not support interrupts though */
#ifndef CONFIG_HM_GMI_MUX
 	{TEGRA_GPIO_PA0,	(GPIOF_IN | GPIOF_NO_EXPORT),		"P73 - GYRO-INT1"},
 	{TEGRA_GPIO_PA7,	(GPIOF_IN | GPIOF_NO_EXPORT),		"P67 - GYRO-INT2"},
#else
 	{TEGRA_GPIO_PA0,	(GPIOF_IN | GPIOF_NO_EXPORT),		"P73 - GYRO-INT1"},
 	{TEGRA_GPIO_PAA7,	(GPIOF_IN | GPIOF_NO_EXPORT),		"P172 - GYRO-INT2"},
#endif

    /* CAN wake up. */
#ifdef CONFIG_HM_WAKE_ON_CAN /* GPIO_PB6 might be used for digital in. */
    /* CAN wakeup pin. */
    {TEGRA_GPIO_PB6,	(GPIOF_IN),		                "P55 - CAN-WAKEUP"},

    /* Enable wakeup on each interface. */
    {TEGRA_GPIO_PH2,	(GPIOF_DIR_OUT | GPIOF_INIT_LOW),"P169 - CAN0-WAKEUP"},
    {TEGRA_GPIO_PH3,	(GPIOF_DIR_OUT | GPIOF_INIT_LOW),"P171 - CAN1-WAKEUP"},
    {TEGRA_GPIO_PH4,	(GPIOF_DIR_OUT | GPIOF_INIT_LOW),"P173 - CAN2-WAKEUP"},
    {TEGRA_GPIO_PH5,	(GPIOF_DIR_OUT | GPIOF_INIT_LOW),"P175 - CAN3-WAKEUP"},
    {TEGRA_GPIO_PH6,	(GPIOF_DIR_OUT | GPIOF_INIT_LOW),"P177 - CAN4-WAKEUP"},
    {TEGRA_GPIO_PH7,	(GPIOF_DIR_OUT | GPIOF_INIT_LOW),"P179 - CAN5-WAKEUP"},
#endif /* HM_WAKE_ON_CAN */

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
}

#ifdef CONFIG_DEBUG_FS

static irqreturn_t wake_up_irq(int irq, void *data)
{
	return IRQ_HANDLED;
}

static int setup_wake_source(unsigned int gpio, unsigned long flags,
	const char *name)
{
	int irq = gpio_to_irq(gpio);
	int err = 0;

	if((err = request_irq(irq, wake_up_irq, flags, name, NULL))){
		printk(KERN_ERR "Failed to request irq: %d\n", irq);
		goto error;
	}

	if((err = enable_irq_wake(irq))){
		printk(KERN_ERR "Failed to enable wake irq: %d\n", irq);
		free_irq(irq, NULL);
		goto error;
	}

error:
	return err;
}

#define WAKE_SOURCE_DIG_IN_2	(1 << 0)
#define WAKE_SOURCE_CAN			(1 << 1)
static u64 wake_sources;

static int colibri_t20_get_wakeup_source(void *data, u64 *val)
{
	*val = (u64)wake_sources;
	return 0;
}

static int colibri_t20_set_wakeup_source(void *data, u64 val)
{
	int err = 0;

	if(val & WAKE_SOURCE_DIG_IN_2){
		err = setup_wake_source(GPIO_WAKEUP_PIN, IRQF_TRIGGER_RISING
			| IRQF_NO_SUSPEND, "GPIO-WAKE");

	} else {
		if(wake_sources & WAKE_SOURCE_DIG_IN_2) {
			int irq = gpio_to_irq(GPIO_WAKEUP_PIN);

			if((err = disable_irq_wake(irq)))
				printk(KERN_ERR "Failed to disable irq wake: %d", irq);

			free_irq(irq, NULL);
		}
	}

	if(err)
		goto error;

	if(val & WAKE_SOURCE_CAN){
		err = setup_wake_source(CAN_WAKEUP_PIN, IRQF_TRIGGER_FALLING
			| IRQF_NO_SUSPEND, "CAN-WAKE");
	} else {
		if(wake_sources & WAKE_SOURCE_CAN) {
			int irq = gpio_to_irq(CAN_WAKEUP_PIN);

			if((err = disable_irq_wake(irq)))
				printk(KERN_ERR "Failed to disable irq wake: %d", irq);

			free_irq(irq, NULL);
		}
	}

error:
	wake_sources = val;

	return err;
}

DEFINE_SIMPLE_ATTRIBUTE(wake_source_fops, colibri_t20_get_wakeup_source,
	colibri_t20_set_wakeup_source, "%llu\n");
#endif /* CONFIG_DEBUG_FS */

static int colibri_t20_wakeup_source_init(void)
{
	struct dentry *wake_source_debugfs_root;

	wake_source_debugfs_root = debugfs_create_dir("t20_wake_source", 0);

	if (!debugfs_create_file("wake_source", 0644, wake_source_debugfs_root,
		NULL, &wake_source_fops))
		return -ENOMEM;

	colibri_t20_set_wakeup_source(NULL, (u64)0);
	return 0;
}

/* I2C */
#ifdef CONFIG_SENSORS_L3G4200D
static int colibri_l3g4200d_init(void)
{
        tegra_gpio_enable(L3G4200D_DRDY_GPIO);
        gpio_request(L3G4200D_DRDY_GPIO, "l3g4200d_irq");
        gpio_direction_input(L3G4200D_DRDY_GPIO);
        return 0;
}

struct l3g4200d_platform_data colibri_gyro_pdata = {
        .poll_interval = 10,
        .gpio_drdy = L3G4200D_DRDY_GPIO,

        .ctrl_reg1 = 0x1f,      /* ODR100 */
        .ctrl_reg2 = 0x00,
        .ctrl_reg3 = 0x08,      /* Enable DRDY interrupt */
        .ctrl_reg4 = 0xA0,      /* BDU enable, 2000 dps */
        .ctrl_reg5 = 0x00,
        .reference = 0x00,
        .fifo_ctrl_reg = 0x00,
        .int1_cfg = 0x00,
        .int1_tsh_xh = 0x00,
        .int1_tsh_xl = 0x00,
        .int1_tsh_yh = 0x00,
        .int1_tsh_yl = 0x00,
        .int1_tsh_zh = 0x00,
        .int1_tsh_zl = 0x00,
        .int1_duration = 0x00,
};
#else
static int colibri_l3g4200d_init(void){ return 0; }
#endif

#ifdef CONFIG_MXC_MMA845X
static struct mxc_mma845x_platform_data mma845x_data = {
	.gpio_pin_get = NULL,
	.gpio_pin_put = NULL,
#ifndef CONFIG_HM_GMI_MUX
	.int1 = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PB7), //P63
	.int2 = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PK4), //P106
#else
	.int1 = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PK4), //P106
	.int2 = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PU5), //P116
#endif
};
#endif

/* GEN1_I2C: I2C_SDA/SCL on SODIMM pin 194/196 (e.g. RTC on carrier board) */
static struct i2c_board_info colibri_t20_i2c_bus1_board_info[] __initdata = {
//#ifdef CONFIG_SENSORS_L3G4200D
#if 0
{
        I2C_BOARD_INFO(L3G4200D_NAME, 0x68),
        .platform_data = &colibri_gyro_pdata,
        .irq = TEGRA_GPIO_TO_IRQ(L3G4200D_DRDY_GPIO),
    },
#endif

#ifdef CONFIG_MXC_MMA845X
	{
		I2C_BOARD_INFO("mma845x", 0x1C),
			.platform_data = (void *)&mma845x_data,
	},
#endif
#ifdef CONFIG_VIDEO_ADV7180
	{
		I2C_BOARD_INFO("adv7180", 0x21),
	},
#endif /* CONFIG_VIDEO_ADV7180 */
#ifdef CONFIG_VIDEO_MT9V111
	{
		I2C_BOARD_INFO("mt9v111", 0x5c),
			.platform_data = (void *)&camera_mt9v111_data,
	},
#endif /* CONFIG_VIDEO_MT9V111 */

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

/* MMC/SD */

#ifdef CONFIG_HM_REDPINE_WIFI
static struct tegra_sdhci_platform_data colibri_t20_sdhci_wifi_platform_data = {
	/* We dont have a card detect pin for wifi. I is always connected. */
	.is_8bit	= 0,
	.cd_gpio	= -1,
	.power_gpio	= -1,
	.wp_gpio	= -1,
};
#endif /* CONFIG_HM_REDPINE_WIFI */

static struct tegra_sdhci_platform_data colibri_t20_sdhci_mmc_platform_data = {
	.cd_gpio		= MMC_CD,
	.cd_gpio_wake	= 0,
	.is_8bit		= 0,
	.power_gpio		= -1,
	.wp_gpio		= -1,
};

int __init colibri_t20_sdhci_init(void)
{
#if !defined(CONFIG_HM_REDPINE_WIFI)
	tegra_sdhci_device4.dev.platform_data =
			&colibri_t20_sdhci_mmc_platform_data;

	platform_device_register(&tegra_sdhci_device4);
#else
	tegra_sdhci_device2.dev.platform_data =
			&colibri_t20_sdhci_mmc_platform_data;

	tegra_sdhci_device4.dev.platform_data =
			&colibri_t20_sdhci_wifi_platform_data;

	platform_device_register(&tegra_sdhci_device2);
	platform_device_register(&tegra_sdhci_device4);
#endif /* CONFIG_HM_REDPINE_WIFI */

	return 0;
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


#ifdef CONFIG_HM_WIFI_NETDEV_LED
static struct gpio_led status_leds[] = {
	[0] =  {
		/* Global on switch for LEDs */
		.name = "mx4-ct-wifi",
		.default_trigger = "netdev",
		.gpio = (MX4_CT_WIFI_LED),
		.active_low = 0,
		.default_state = LEDS_GPIO_DEFSTATE_OFF,
	},
	[1] =  {
		/* Global on switch for LEDs */
		.name = "mx4-wifi",
		.default_trigger = "netdev",
		.gpio = (MX4_WIFI_LED),
		.active_low = 0,
		.default_state = LEDS_GPIO_DEFSTATE_OFF,
	},
	[2] =  {
		/* WIFI-RED on MX-4 T20 */
		.name = "mx4-wifi-red",
		.default_trigger = "none",
		.gpio = (MX4_WIFI_RED),
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
#endif /* CONFIG_HM_WIFI_NETDEV_LED */


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
#ifdef CONFIG_HM_EVENT_IRQ
	.event_rdy = TEGRA_GPIO_TO_IRQ(MX4_DATA_READY_PIC),
#else
	.event_rdy = -1,
#endif
};


static struct spi_board_info tegra_spi_devices[] __initdata = {
	{
		.bus_num	= 3,		/* SPI4 */
		.chip_select	= 0,
		.irq		= TEGRA_GPIO_TO_IRQ(MX4_WAKE_UP_CPU),
		.max_speed_hz	= 12000000,
		.modalias	= "mx4_io_spi",
		.mode		= SPI_MODE_1,
		.platform_data	= &mx4_io_pdata,
	},
	{
		.bus_num	= 3,
//working with spidev_test
//		.chip_select	= 0,
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
		enable_irq(gpio_to_irq(THERMD_ALERT));
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

	if (request_irq(gpio_to_irq(THERMD_ALERT), thermd_alert_irq,
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

/* UART */
#ifdef CONFIG_HM_EXT_8250_UART
#define SERIAL_FLAGS (UPF_BOOT_AUTOCONF | UPF_IOREMAP | UPF_SKIP_TEST)
#define SERIAL_CLK   (24000000)

static struct plat_serial8250_port extern_uart_platform_data[] = {
	[0] = { 	/* Extern uart C (RS232)*/
		.mapbase	= TEGRA_EXT_UARTC_BASE,
		.irq		= TEGRA_GPIO_TO_IRQ(TEGRA_EXT_UARTC_INT),
		.irqflags 	= IRQF_TRIGGER_RISING,
		.flags		= SERIAL_FLAGS,
		.iotype		= UPIO_MEM,
#ifndef CONFIG_HM_GMI_MUX
		.regshift	= 5,
#else
		.regshift	= 18,
#endif /* !CONFIG_HM_GMI_MUX */
		.uartclk	= SERIAL_CLK,
	},
	{
		.flags = 0
	},
};

static struct platform_device extern_uart = {
	.name = "serial8250",
	.id = PLAT8250_DEV_PLATFORM1,
	.dev = {
		.platform_data = extern_uart_platform_data,
	},
};
#endif /* ifdef CONFIG_HM_EXT_8250_UART */

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
#ifdef CONFIG_MACH_HM_MX4
	&tegra_uarta_device,
#endif /* CONFIG_MACH_HM_MX4 */
	&tegra_uartb_device,
	&tegra_uartc_device,
	&tegra_uartd_device,
#ifdef CONFIG_HM_EXT_8250_UART
	&extern_uart,
#endif /* CONFIG_HM_EXT_8250_UART */
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
	.port_otg	= true,
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

static struct tegra_usb_platform_data tegra_ehci1_utmi_pdata = {
	.has_hostpc	= false,
	.op_mode	= TEGRA_USB_OPMODE_HOST,
	.phy_intf	= TEGRA_USB_PHY_INTF_UTMI,
	.port_otg	= true,
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
		.vbus_gpio			= -1, // TODO: Set a power enable?
		.vbus_reg			= NULL,
	},
};

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
	return;
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
		.power_off_on_suspend		= false,
		.remote_wakeup_supported	= false,
		.vbus_gpio			= -1,
		.vbus_reg			= NULL,
	},
};


static void modem_link_platform_open(void)
{
	int vbus_gpio = USBH_PEN;
	printk( KERN_INFO "USB HUB/Modem link platform open");

	gpio_request(vbus_gpio, "usb-hub/modem_vbus_gpio");
	gpio_direction_output(vbus_gpio, 1);
}

static void modem_link_platform_post_phy_on(void)
{
	printk( KERN_INFO "USB HUB/Modem link platform on");
	/* enable VBUS */
	gpio_set_value(USBH_PEN, 1);
}

static void modem_link_platform_pre_phy_off(void)
{
	printk( KERN_INFO "USB HUB/Modem link platform off");
	/* disable VBUS */
	gpio_set_value(USBH_PEN, 0);
}

static struct tegra_usb_phy_platform_ops modem_link_plat_ops = {
	.open = modem_link_platform_open,
	.post_phy_on = modem_link_platform_post_phy_on,
	.pre_phy_off = modem_link_platform_pre_phy_off,
};

static struct tegra_usb_platform_data tegra_ehci3_utmi_pdata = {
	.has_hostpc	= false,
	.ops 		= &modem_link_plat_ops,
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
		.power_off_on_suspend		= true,
		.remote_wakeup_supported	= false,
		.vbus_gpio			= -1,
		.vbus_gpio_inverted		= 0,
		.vbus_reg			= NULL,
	},
};

#ifndef CONFIG_USB_TEGRA_OTG
static struct platform_device *tegra_usb_otg_host_register(void)
{
	struct platform_device *pdev;
	void *platform_data;
	int val;

	pdev = platform_device_alloc(tegra_ehci1_device.name,
				     tegra_ehci1_device.id);
	if (!pdev)
		return NULL;

	val = platform_device_add_resources(pdev, tegra_ehci1_device.resource,
					    tegra_ehci1_device.num_resources);
	if (val)
		goto error;

	pdev->dev.dma_mask = tegra_ehci1_device.dev.dma_mask;
	pdev->dev.coherent_dma_mask = tegra_ehci1_device.dev.coherent_dma_mask;

	platform_data = kmalloc(sizeof(struct tegra_usb_platform_data),
				GFP_KERNEL);
	if (!platform_data)
		goto error;

	memcpy(platform_data, &tegra_ehci1_utmi_pdata,
	       sizeof(struct tegra_usb_platform_data));
	pdev->dev.platform_data = platform_data;

	val = platform_device_add(pdev);
	if (val)
		goto error_add;

	return pdev;

error_add:
	kfree(platform_data);
error:
	pr_err("%s: failed to add the host controller device\n", __func__);
	platform_device_put(pdev);
	return NULL;
}

static void tegra_usb_otg_host_unregister(struct platform_device *pdev)
{
	platform_device_unregister(pdev);
}

static struct colibri_otg_platform_data colibri_otg_pdata = {
	.cable_detect_gpio	= USBC_DET,
	.host_register		= &tegra_usb_otg_host_register,
	.host_unregister	= &tegra_usb_otg_host_unregister,
};
#else /* !CONFIG_USB_TEGRA_OTG */
static struct tegra_usb_otg_data tegra_otg_pdata = {
	.ehci_device = &tegra_ehci1_device,
	.ehci_pdata = &tegra_ehci1_utmi_pdata,
};
#endif /* !CONFIG_USB_TEGRA_OTG */

#ifndef CONFIG_USB_TEGRA_OTG
struct platform_device colibri_otg_device = {
	.name	= "colibri-otg",
	.id	= -1,
	.dev = {
		.platform_data = &colibri_otg_pdata,
	},
};
#endif /* !CONFIG_USB_TEGRA_OTG */

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
#ifndef CONFIG_USB_TEGRA_OTG
	platform_device_register(&colibri_otg_device);
#else /* !CONFIG_USB_TEGRA_OTG */
	tegra_otg_device.dev.platform_data = &tegra_otg_pdata;
	platform_device_register(&tegra_otg_device);
#endif /* !CONFIG_USB_TEGRA_OTG */

	/* setup the udc platform data */
	tegra_udc_device.dev.platform_data = &tegra_udc_pdata;
	platform_device_register(&tegra_udc_device);
#else
	/* MX-4 HOST PORT -> USB HUB */
	tegra_ehci1_device.dev.platform_data = &tegra_ehci1_utmi_pdata;
	platform_device_register(&tegra_ehci1_device);
#endif /* CONFIG_USB_GADGET */
	/* EHCI instance 1: ULPI PHY -> ASIX ETH */
	tegra_ehci2_device.dev.platform_data = &tegra_ehci2_ulpi_link_pdata;
	platform_device_register(&tegra_ehci2_device);

	/* EHCI instance 2: USB3_DP/N -> USBH1_P/N */
	tegra_ehci3_device.dev.platform_data = &tegra_ehci3_utmi_pdata;
	platform_device_register(&tegra_ehci3_device);

}

/* W1, aka OWR, aka OneWire */

#ifdef CONFIG_W1_MASTER_TEGRA
struct tegra_w1_timings colibri_t20_w1_timings = {
		.tsu		= 1,
		.trelease	= 0xf,
		.trdv		= 0xf,
		.tlow0		= 0x3c,
		.tlow1		= 1,
		.tslot		= 0x77,

		.tpdl		= 0x78,
		.tpdh		= 0x1e,
		.trstl		= 0x1df,
		.trsth		= 0x1df,
		.rdsclk		= 0x7,
		.psclk		= 0x50,
};

struct tegra_w1_platform_data colibri_t20_w1_platform_data = {
	.clk_id		= "tegra_w1",
	.timings	= &colibri_t20_w1_timings,
};
#endif /* CONFIG_W1_MASTER_TEGRA */

static struct platform_device *colibri_t20_devices[] __initdata = {
#ifdef CONFIG_RTC_DRV_TEGRA
	&tegra_rtc_device,
#endif
	&tegra_nand_device,

	&tegra_pmu_device,
	&tegra_gart_device,
	&tegra_aes_device,
#ifdef CONFIG_KEYBOARD_GPIO
	&colibri_t20_keys_device,
#endif
	&tegra_wdt_device,
	&tegra_avp_device,
#ifdef CONFIG_TEGRA_CAMERA
	&tegra_camera,
#endif
	&tegra_ac97_device,
	&tegra_spdif_device,
	&tegra_das_device,
	&spdif_dit_device,
//bluetooth
	&tegra_pcm_device,
#ifdef CONFIG_SOUND
	&colibri_t20_audio_device,
#endif
	&tegra_spi_device4,
#ifdef CONFIG_HM_WIFI_NETDEV_LED
	&status_led_dev,
#endif /* CONFIG_HM_WIFI_NETDEV_LED */
	&tegra_pwfm1_device,
	&tegra_pwfm2_device,
	&tegra_pwfm3_device,
#ifdef CONFIG_W1_MASTER_TEGRA
	&tegra_w1_device,
#endif
};

static void __init colibri_t20_init(void)
{

#if defined(CONFIG_CAN_SJA1000) || defined(CONFIG_CAN_SJA1000_MODULE)

#ifdef CONFIG_HM_GMI_MUX
	writel(SNOR_CONFIG_SNOR_CS(SNOR_CS_PIN) | SNOR_CONFIG_MUX | SNOR_CONFIG_ADV_POL
		| SNOR_CONFIG_32BIT,
		SNOR_CONFIG_REG);
	writel(SNOR_CONFIG_GO | SNOR_CONFIG_SNOR_CS(SNOR_CS_PIN) | SNOR_CONFIG_MUX |
		SNOR_CONFIG_ADV_POL | SNOR_CONFIG_32BIT, SNOR_CONFIG_REG);
#else
	writel(SNOR_CONFIG_SNOR_CS(SNOR_CS_PIN), SNOR_CONFIG_REG);
	writel(SNOR_CONFIG_GO | SNOR_CONFIG_SNOR_CS(SNOR_CS_PIN), SNOR_CONFIG_REG);
#endif /* CONFIG_HM_GMI_MUX */

	tegra_gpio_enable(TEGRA_CAN_INT);
	tegra_gpio_enable(TEGRA_CAN2_INT);
	gpio_request_one(TEGRA_CAN_INT, GPIOF_DIR_IN, "CAN1-INT");
	gpio_request_one(TEGRA_CAN2_INT, GPIOF_DIR_IN, "CAN2-INT");

	colibri_can_resource[1].start	= TEGRA_GPIO_TO_IRQ(TEGRA_CAN_INT);
	colibri_can_resource[1].end	= TEGRA_GPIO_TO_IRQ(TEGRA_CAN_INT);

	colibri_can_resource2[1].start	= TEGRA_GPIO_TO_IRQ(TEGRA_CAN2_INT);
	colibri_can_resource2[1].end	= TEGRA_GPIO_TO_IRQ(TEGRA_CAN2_INT);

	platform_device_register(&colibri_can_device);
	platform_device_register(&colibri_can_device2);

#if defined CONFIG_HM_GTT_CAN
	tegra_gpio_enable(TEGRA_CAN3_INT);
	tegra_gpio_enable(TEGRA_CAN4_INT);
	tegra_gpio_enable(TEGRA_CAN5_INT);
	tegra_gpio_enable(TEGRA_CAN6_INT);
	gpio_request_one(TEGRA_CAN3_INT, GPIOF_DIR_IN, "CAN3-INT");
	gpio_request_one(TEGRA_CAN4_INT, GPIOF_DIR_IN, "CAN4-INT");
	gpio_request_one(TEGRA_CAN5_INT, GPIOF_DIR_IN, "CAN5-INT");
	gpio_request_one(TEGRA_CAN6_INT, GPIOF_DIR_IN, "CAN6-INT");


	colibri_can_resource3[1].start	= TEGRA_GPIO_TO_IRQ(TEGRA_CAN3_INT);
	colibri_can_resource3[1].end	= TEGRA_GPIO_TO_IRQ(TEGRA_CAN3_INT);

	colibri_can_resource4[1].start	= TEGRA_GPIO_TO_IRQ(TEGRA_CAN4_INT);
	colibri_can_resource4[1].end	= TEGRA_GPIO_TO_IRQ(TEGRA_CAN4_INT);

	colibri_can_resource5[1].start	= TEGRA_GPIO_TO_IRQ(TEGRA_CAN5_INT);
	colibri_can_resource5[1].end	= TEGRA_GPIO_TO_IRQ(TEGRA_CAN5_INT);

	colibri_can_resource6[1].start	= TEGRA_GPIO_TO_IRQ(TEGRA_CAN6_INT);
	colibri_can_resource6[1].end	= TEGRA_GPIO_TO_IRQ(TEGRA_CAN6_INT);

	platform_device_register(&colibri_can_device3);
	platform_device_register(&colibri_can_device4);
	platform_device_register(&colibri_can_device5);
	platform_device_register(&colibri_can_device6);
#endif /* CONFIG_HM_GTT_CAN */

#endif /* CONFIG_CAN_SJA1000 || CONFIG_CAN_SJA1000_MODULE */

	colibri_l3g4200d_init();

	colibri_t20_wakeup_source_init();

	tegra_clk_init_from_table(colibri_t20_clk_init_table);
	colibri_t20_pinmux_init();
	colibri_t20_thermd_alert_init();
	colibri_t20_i2c_init();
	colibri_t20_uart_init();
//
	tegra_ac97_device.dev.platform_data = &colibri_t20_wm97xx_pdata;
//
#ifdef CONFIG_W1_MASTER_TEGRA
	tegra_w1_device.dev.platform_data = &colibri_t20_w1_platform_data;
#endif
	platform_add_devices(colibri_t20_devices,
			     ARRAY_SIZE(colibri_t20_devices));
	tegra_ram_console_debug_init();
#ifndef CONFIG_MACH_HM_VCB
	colibri_t20_sdhci_init();
#endif /* CONFIG_MACH_HM_VCB */
	colibri_t20_regulator_init();

//	tegra_das_device.dev.platform_data = &tegra_das_pdata;
//	tegra_ac97_device.dev.platform_data = &tegra_audio_pdata;
//	tegra_spdif_input_device.name = "spdif";
//	tegra_spdif_input_device.dev.platform_data = &tegra_spdif_audio_pdata;

	colibri_t20_usb_init();
	colibri_t20_panel_init();
//sensors

	/* Note: V1.1c modules require proper BCT setting 666 rather than
	   721.5 MHz EMC clock */
	colibri_t20_emc_init();

	colibri_t20_gpio_init();
	colibri_t20_register_spidev();

	tegra_release_bootloader_fb();

	/* Show platform version */
#ifdef CONFIG_MACH_HM_VCB
	pr_info("Host Mobility VCB\n");
#endif
#ifdef CONFIG_MACH_HM_MX4
	pr_info("Host Mobility MX4\n");
#endif
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
	tegra_reserve(0, SZ_8M + SZ_1M, SZ_16M);
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
