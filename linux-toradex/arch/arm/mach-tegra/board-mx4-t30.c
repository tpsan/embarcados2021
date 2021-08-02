/*
 * arch/arm/mach-tegra/board-mx4-t30.c
 *
 * Copyright (c) 2012-2014 Host Mobility AB
 *
 * This source code is licensed under the GNU General Public License,
 * Version 2. See the file COPYING for more details.
 */

#include <asm/mach/arch.h>
#include <asm/mach-types.h>

/* sja1000 and gpio_keys require this include */
#include <linux/types.h>

#include <linux/can/platform/sja1000.h>
#include <linux/clk.h>
#include <linux/colibri_usb.h>
#include <linux/gpio_keys.h>
#include <linux/i2c.h>
#include <linux/i2c-tegra.h>
#include <linux/input.h>
#include <linux/io.h>
#include <linux/leds_pwm.h>
#include <linux/lm95245.h>
#include <linux/mfd/stmpe.h>
#include <linux/mma845x.h>
#include <linux/platform_data/tegra_usb.h>
#include <linux/platform_device.h>
#include <linux/pps_gen_gpio.h>
#include <linux/serial_8250.h>
#include <linux/spi-tegra.h>
#include <linux/spi/spi.h>
#include <linux/tegra_uart.h>

#include <mach/io_dpd.h>
#include <mach/mx4_t30_iomap.h>
#include <mach/sdhci.h>
#include <mach/tegra_asoc_pdata.h>
#include <mach/tegra_fiq_debugger.h>
#include <mach/thermal.h>
#include <mach/usb_phy.h>

#include "board-mx4-t30.h"
#include "board.h"
#include "clock.h"
#include "devices.h"
#include "gpio-names.h"
#include "pm.h"

//from former drivers/mtd/maps/tegra_nor.h
#define TEGRA_GMI_PHYS			0x70009000
#define TEGRA_GMI_BASE			IO_TO_VIRT(TEGRA_GMI_PHYS)
#define TEGRA_SNOR_CONFIG_REG		(TEGRA_GMI_BASE + 0x00)

//from drivers/mtd/maps/tegra_nor.c
#define __BITMASK0(len)			(BIT(len) - 1)
#define REG_FIELD(val, start, len)	(((val) & __BITMASK0(len)) << (start))
#define REG_BIT(bit)					(1 << (bit))
#define TEGRA_SNOR_CONFIG_GO		BIT(31)
#define TEGRA_SNOR_CONFIG_SNOR_CS(val)	REG_FIELD((val), 4, 3)
#define TEGRA_SNOR_CONFIG_32BIT				REG_BIT(30)
#define TEGRA_SNOR_CONFIG_MUX					REG_BIT(28)
#define TEGRA_SNOR_CONFIG_ADV_POL				REG_BIT(22)

/* Audio */

static struct tegra_asoc_platform_data colibri_t30_audio_sgtl5000_pdata = {
	.gpio_ext_mic_en	= -1,
	.gpio_hp_det		= -1,
	.gpio_hp_mute		= -1,
	.gpio_int_mic_en	= -1,
	.gpio_spkr_en		= -1,
	.i2s_param[BASEBAND] = {
		.audio_port_id	= -1,
	},
	.i2s_param[BT_SCO] = {
		.audio_port_id	= -1,
	},
	.i2s_param[HIFI_CODEC] = {
		.audio_port_id	= 1, /* index of below registered
					tegra_i2s_device plus one if HDA codec
					is activated as well */
		.i2s_mode	= TEGRA_DAIFMT_I2S,
		.is_i2s_master	= 1, /* meaning T30 SoC is I2S master */
		.sample_size	= 16,
	},
};

static struct platform_device colibri_t30_audio_sgtl5000_device = {
	.name	= "tegra-snd-colibri_t30-sgtl5000",
	.id	= 0,
	.dev = {
		.platform_data = &colibri_t30_audio_sgtl5000_pdata,
	},
};

#if defined(CONFIG_CAN_SJA1000) || defined(CONFIG_CAN_SJA1000_MODULE)
static struct resource colibri_can_resource[] = {
	[0] =   {
		.start	= TEGRA_CAN_BASE,		/* address */
		.end	= TEGRA_CAN_BASE + TEGRA_CAN_SIZE,/* data */
		.flags	= IORESOURCE_MEM | IORESOURCE_MEM_16BIT,
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

/* ********************************CAN DEVICE 2*******************************/
static struct resource colibri_can_resource2[] = {
	[0] =   {
		.start	= TEGRA_CAN2_BASE, 	/* address */
		.end	= TEGRA_CAN2_BASE + TEGRA_CAN_SIZE, /* data */
		.flags	= IORESOURCE_MEM | IORESOURCE_MEM_16BIT,
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

/* ********************************CAN DEVICE 3*******************************/
static struct resource colibri_can_resource3[] = {
	[0] =   {
		.start	= TEGRA_CAN3_BASE, 	/* address */
		.end	= TEGRA_CAN3_BASE + TEGRA_CAN_SIZE, /* data */
		.flags	= IORESOURCE_MEM | IORESOURCE_MEM_16BIT,
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
		.flags	= IORESOURCE_MEM | IORESOURCE_MEM_16BIT,
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
		.flags	= IORESOURCE_MEM | IORESOURCE_MEM_16BIT,
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
		.flags	= IORESOURCE_MEM | IORESOURCE_MEM_16BIT,
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


static void __init sja1000_init(void)
{
	u32 snor_cfg = 0;

	snor_cfg |= TEGRA_SNOR_CONFIG_SNOR_CS(TEGRA_SNOR_CS_PIN)
		| TEGRA_SNOR_CONFIG_MUX | TEGRA_SNOR_CONFIG_ADV_POL;

	writel(snor_cfg, TEGRA_SNOR_CONFIG_REG);

	snor_cfg |= TEGRA_SNOR_CONFIG_GO;
	writel(snor_cfg, TEGRA_SNOR_CONFIG_REG);

	tegra_gpio_enable(TEGRA_CAN_INT);
	tegra_gpio_enable(TEGRA_CAN2_INT);
	tegra_gpio_enable(TEGRA_CAN3_INT);
	tegra_gpio_enable(TEGRA_CAN4_INT);
	tegra_gpio_enable(TEGRA_CAN5_INT);
	tegra_gpio_enable(TEGRA_CAN6_INT);

	gpio_request_one(TEGRA_CAN_INT, GPIOF_DIR_IN, "CAN1-INT");
	gpio_request_one(TEGRA_CAN2_INT, GPIOF_DIR_IN, "CAN2-INT");
	gpio_request_one(TEGRA_CAN3_INT, GPIOF_DIR_IN, "CAN3-INT");
	gpio_request_one(TEGRA_CAN4_INT, GPIOF_DIR_IN, "CAN4-INT");
	gpio_request_one(TEGRA_CAN5_INT, GPIOF_DIR_IN, "CAN5-INT");
	gpio_request_one(TEGRA_CAN6_INT, GPIOF_DIR_IN, "CAN6-INT");

	colibri_can_resource[1].start	= TEGRA_GPIO_TO_IRQ(TEGRA_CAN_INT);
	colibri_can_resource[1].end		= TEGRA_GPIO_TO_IRQ(TEGRA_CAN_INT);

	colibri_can_resource2[1].start	= TEGRA_GPIO_TO_IRQ(TEGRA_CAN2_INT);
	colibri_can_resource2[1].end	= TEGRA_GPIO_TO_IRQ(TEGRA_CAN2_INT);

	colibri_can_resource3[1].start	= TEGRA_GPIO_TO_IRQ(TEGRA_CAN3_INT);
	colibri_can_resource3[1].end	= TEGRA_GPIO_TO_IRQ(TEGRA_CAN3_INT);

	colibri_can_resource4[1].start	= TEGRA_GPIO_TO_IRQ(TEGRA_CAN4_INT);
	colibri_can_resource4[1].end	= TEGRA_GPIO_TO_IRQ(TEGRA_CAN4_INT);

	colibri_can_resource5[1].start	= TEGRA_GPIO_TO_IRQ(TEGRA_CAN5_INT);
	colibri_can_resource5[1].end	= TEGRA_GPIO_TO_IRQ(TEGRA_CAN5_INT);

	colibri_can_resource6[1].start	= TEGRA_GPIO_TO_IRQ(TEGRA_CAN6_INT);
	colibri_can_resource6[1].end	= TEGRA_GPIO_TO_IRQ(TEGRA_CAN6_INT);

	platform_device_register(&colibri_can_device);
	platform_device_register(&colibri_can_device2);
	platform_device_register(&colibri_can_device3);
	platform_device_register(&colibri_can_device4);
	platform_device_register(&colibri_can_device5);
	platform_device_register(&colibri_can_device6);
}

#endif /* CONFIG_CAN_SJA1000 || CONFIG_CAN_SJA1000_MODULE */

/* Clocks */
static struct tegra_clk_init_table colibri_t30_clk_init_table[] __initdata = {
	/* name		parent		rate		enabled */
	{"apbif",	"clk_m",	12000000,	false},
	{"audio0",	"i2s0_sync",	0,		false},
	{"audio1",	"i2s1_sync",	0,		false},
	{"audio2",	"i2s2_sync",	0,		false},
	{"audio3",	"i2s3_sync",	0,		false},
	{"audio4",	"i2s4_sync",	0,		false},
	{"blink",	"clk_32k",	32768,		true},
	/* optional camera clock */
	{"clk_out_2",	"extern2",	24000000,	false},
	{"d_audio",	"clk_m",	12000000,	false},
	{"dam0",	"clk_m",	12000000,	false},
	{"dam1",	"clk_m",	12000000,	false},
	{"dam2",	"clk_m",	12000000,	false},
	{"extern2",	"clk_m",	24000000,	false},
	{"hda",		"pll_p",	108000000,	false},
	{"hda2codec_2x","pll_p",	48000000,	false},
	{"i2c1",	"pll_p",	3200000,	false},
	{"i2c2",	"pll_p",	3200000,	false},
	{"i2c3",	"pll_p",	3200000,	false},
	{"i2c4",	"pll_p",	3200000,	false},
	{"i2c5",	"pll_p",	3200000,	false},
	{"i2s0",	"pll_a_out0",	0,		false},
	{"i2s1",	"pll_a_out0",	0,		false},
	{"i2s2",	"pll_a_out0",	0,		false},
	{"i2s3",	"pll_a_out0",	0,		false},
	{"i2s4",	"pll_a_out0",	0,		false},
	{"nor",		"pll_p",	86500000,	true},
	{"pll_a",	NULL,		564480000,	true},
	{"pll_m",	NULL,		0,		false},
	{"pwm",		"pll_p",	3187500,	false},
	{"spdif_out",	"pll_a_out0",	0,		false},
	{"vi",		"pll_p",	0,		false},
	{NULL,		NULL,		0,		0},
};

/* GPIO
   Pins in the following struct are configured as GPIO inputs and are
   accessible from userspace through /sys/class/gpio.
   Pins which likely are used with one of their alternate functions
   are commented out.
   Refer to the TRM, chapters 'Multi-Prupose Io Pins' and 'GPIO Controller'. */
static struct gpio colibri_t30_gpios[] = {
//	{TEGRA_GPIO_PA2,	GPIOF_IN,	"SODIMM pin 186"},
//	{TEGRA_GPIO_PA3,	GPIOF_IN,	"SODIMM pin 184"},
//	{TEGRA_GPIO_PB2,	GPIOF_IN,	"SODIMM pin 154"},
#ifndef COLIBRI_T30_VI
	{TEGRA_GPIO_PC1,	GPIOF_IN,	"SODIMM pin 81"},
#endif
	{TEGRA_GPIO_PI3,	GPIOF_IN,	"SODIMM pin 130"},
	{TEGRA_GPIO_PI6,	GPIOF_IN,	"SODIMM pin 132"},
	//{TEGRA_GPIO_PJ0,	GPIOF_IN,	"SODIMM pin 126"},
	//{TEGRA_GPIO_PJ2,	GPIOF_IN,	"SODIMM pin 128"},
//conflicts with GMI_ADV_N used for multiplexed address/data bus
//	{TEGRA_GPIO_PK0,	GPIOF_IN,	"SODIMM pin 150"},
//	{TEGRA_GPIO_PK1,	GPIOF_IN,	"SODIMM pin 152"},
#ifndef CONFIG_KEYBOARD_GPIO
//conflicts with menu key
//	{TEGRA_GPIO_PK6,	GPIOF_IN,	"SODIMM pin 135"},
#endif
//	{TEGRA_GPIO_PN0,	GPIOF_IN,	"SODIMM pin 174"},
//	{TEGRA_GPIO_PN1,	GPIOF_IN,	"SODIMM pin 176"},
//	{TEGRA_GPIO_PN2,	GPIOF_IN,	"SODIMM pin 178"},
//	{TEGRA_GPIO_PN3,	GPIOF_IN,	"SODIMM pin 180"},
//	{TEGRA_GPIO_PN4,	GPIOF_IN,	"SODIMM pin 160"},
//	{TEGRA_GPIO_PN5,	GPIOF_IN,	"SODIMM pin 158"},
//	{TEGRA_GPIO_PN6,	GPIOF_IN,	"SODIMM pin 162"},
//conflicts with ADDRESS13
//	{TEGRA_GPIO_PP4,	GPIOF_IN,	"SODIMM pin 120"},
//conflicts with ADDRESS14
//	{TEGRA_GPIO_PP5,	GPIOF_IN,	"SODIMM pin 122"},
//conflicts with ADDRESS15
//	{TEGRA_GPIO_PP6,	GPIOF_IN,	"SODIMM pin 124"},
//	{TEGRA_GPIO_PP7,	GPIOF_IN,	"SODIMM pin 188"},
#if !defined(IRIS) && !defined(CONFIG_CAN_MCP251X) && \
		!defined(CONFIG_CAN_MCP251X_MODULE) && \
		!defined(CONFIG_CAN_SJA1000) && \
		!defined(CONFIG_CAN_SJA1000_MODULE)
//conflicts with CAN interrupt on Colibri Evaluation Board
//conflicts with DAC_PSAVE# on Iris
	{TEGRA_GPIO_PS0,	GPIOF_IN,	"SODIMM pin 73"},
#endif
#ifndef CONFIG_KEYBOARD_GPIO
//conflicts with back key
	{TEGRA_GPIO_PT5,	GPIOF_IN,	"SOD-133, Iris X16-14"},
//conflicts with home key
	{TEGRA_GPIO_PT6,	GPIOF_IN,	"SODIMM pin 127"},
//conflicts with power key (WAKE1)
	//{TEGRA_GPIO_PV1,	GPIOF_IN,	"SODI-45, Iris X16-20"},
#endif
#ifndef COLIBRI_T30_VI
	{TEGRA_GPIO_PW5,	GPIOF_IN,	"SODIMM pin 75"},
	//WLAN_CD
	//{TEGRA_GPIO_PV2,	GPIOF_IN,	"SODIMM pin 71"},
	{TEGRA_GPIO_PV3,	GPIOF_IN,	"SODI-85, Iris X16-18"},
#endif
//conflicts with ADDRESS12
//	{TEGRA_GPIO_PU6,	GPIOF_IN,	"SODIMM pin 118"},
//multiplexed LCD_D21
//	{TEGRA_GPIO_PX0,	GPIOF_IN,	"SODIMM pin 142"},
//multiplexed LCD_D20
//	{TEGRA_GPIO_PX1,	GPIOF_IN,	"SODIMM pin 140"},
//multiplexed LCD_D19
//	{TEGRA_GPIO_PX2,	GPIOF_IN,	"SODIMM pin 138"},
//multiplexed LCD_D18
//	{TEGRA_GPIO_PX3,	GPIOF_IN,	"SODIMM pin 136"},
	{TEGRA_GPIO_PX4,	GPIOF_IN,	"SODIMM pin 134"},
	{TEGRA_GPIO_PX6,	GPIOF_IN,	"102, I X13 ForceOFF#"},
	{TEGRA_GPIO_PX7,	GPIOF_IN,	"104, I X14 ForceOFF#"},
#ifndef COLIBRI_T30_VI
//	{TEGRA_GPIO_PY4,	GPIOF_IN,	"SODI-97, Iris X16-17"},
//	{TEGRA_GPIO_PY5,	GPIOF_IN,	"SODI-79, Iris X16-19"},
//	{TEGRA_GPIO_PY6,	GPIOF_IN,	"SODI-103, Iris X16-15"},
//	{TEGRA_GPIO_PY7,	GPIOF_IN,	"SODI-101, Iris X16-16"},
//	{TEGRA_GPIO_PZ0,	GPIOF_IN,	"SODI-96"},
//	{TEGRA_GPIO_PZ1,	GPIOF_IN,	"SODI-98, Iris X16-13"},
#endif
//	{TEGRA_GPIO_PZ2,	GPIOF_IN,	"SODIMM pin 156"},
//	{TEGRA_GPIO_PZ4,	GPIOF_IN,	"SODIMM pin 164"},
	{TEGRA_GPIO_PBB4,	GPIOF_IN,	"SODIMM pin 166"},
	{TEGRA_GPIO_PBB5,	GPIOF_IN,	"SODIMM pin 168"},
	{TEGRA_GPIO_PBB6,	GPIOF_IN,	"SODIMM pin 170"},
	{TEGRA_GPIO_PBB7,	GPIOF_IN,	"SODIMM pin 172"},
#ifndef COLIBRI_T30_VI
//	{TEGRA_GPIO_PCC2,	GPIOF_IN,	"SODIMM pin 77"},
	{TEGRA_GPIO_PCC7,	GPIOF_IN,	"SODIMM pin 94"},
#endif
#ifndef CONFIG_KEYBOARD_GPIO
//conflicts with volume down key
	//{TEGRA_GPIO_PCC6,	GPIOF_IN,	"SODIMM pin 24"},
//conflicts with volume up key
	//{TEGRA_GPIO_PDD7,	GPIOF_IN,	"SODIMM pin 22"},
#endif
#ifndef COLIBRI_T30_VI
	{TEGRA_GPIO_PDD5,	GPIOF_IN,	"SODIMM pin 69"},
	{TEGRA_GPIO_PDD6,	GPIOF_IN,	"SODIMM pin 65"},
#endif
	{TEGRA_GPIO_PM6,	GPIOF_OUT_INIT_HIGH,	"FR-MCU-IN"},
	{TEGRA_GPIO_PH0,	GPIOF_OUT_INIT_HIGH,	"FR-ETH2-WAKE"},
	{TEGRA_GPIO_PH6,	GPIOF_OUT_INIT_HIGH,	"FR-ETH-WAKE"},
	{TEGRA_GPIO_PH7,	GPIOF_OUT_INIT_LOW,	"nFR-RST"},
	{TEGRA_GPIO_PJ1,	GPIOF_OUT_INIT_LOW,	"SSP-CAN-CS-D0"},
	{TEGRA_GPIO_PE7,	GPIOF_OUT_INIT_LOW,	"SSP-CAN-CS-D1"},
	{TEGRA_GPIO_PF1,	GPIOF_OUT_INIT_LOW,	"SSP-CAN-CS-D2"},
};

static void colibri_t30_gpio_init(void)
{
	int i = 0;
	int length = sizeof(colibri_t30_gpios) / sizeof(struct gpio);
	int err = 0;

	for (i = 0; i < length; i++) {
		err = gpio_request_one(colibri_t30_gpios[i].gpio,
				       colibri_t30_gpios[i].flags,
				       colibri_t30_gpios[i].label);

		if (err) {
			pr_warning("gpio_request(%s) failed, err = %d",
				   colibri_t30_gpios[i].label, err);
		} else {
			gpio_export(colibri_t30_gpios[i].gpio, true);
		}
	}
}

/* I2C */
#ifdef CONFIG_MXC_MMA845X
static struct mxc_mma845x_platform_data mma845x_data = {
	.gpio_pin_get = NULL,
	.gpio_pin_put = NULL,
	.int1 = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PK4), //P106
	.int2 = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PU5), //P116
};
#endif

/* Make sure that the pinmuxing enable the 'open drain' feature for pins used
   for I2C */

/* GEN1_I2C: I2C_SDA/SCL on SODIMM pin 194/196 (e.g. RTC on carrier board) */
static struct i2c_board_info colibri_t30_i2c_bus1_board_info[] __initdata = {
#ifdef CONFIG_RTC_DRV_PCF85063
    	{
		I2C_BOARD_INFO("pcf85063", 0x51),
	},
#endif
#ifdef CONFIG_MXC_MMA845X
	{
		I2C_BOARD_INFO("mma845x", 0x1C),
			.platform_data = (void *)&mma845x_data,
	},
#endif
};

static struct tegra_i2c_platform_data colibri_t30_i2c1_platform_data = {
	.adapter_nr	= 0,
	.arb_recovery	= arb_lost_recovery,
	.bus_clk_rate	= {400000, 0},
	.bus_count	= 1,
	.scl_gpio	= {I2C_SCL, 0},
	.sda_gpio	= {I2C_SDA, 0},
	.slave_addr	= 0x00FC,
};

/* GEN2_I2C: unused */

/* DDC_CLOCK/DATA on X3 pin 15/16 (e.g. display EDID) */
static struct tegra_i2c_platform_data colibri_t30_i2c4_platform_data = {
	.adapter_nr	= 3,
	.arb_recovery	= arb_lost_recovery,
	.bus_clk_rate	= {10000, 10000},
	.bus_count	= 1,
	.scl_gpio	= {DDC_SCL, 0},
	.sda_gpio	= {DDC_SDA, 0},
	.slave_addr	= 0x00FC,
};

/* PWR_I2C: power I2C to audio codec, PMIC, temperature sensor and touch screen
	    controller */

/* STMPE811 touch screen controller */
static struct stmpe_ts_platform_data stmpe811_ts_data = {
	.adc_freq		= 1, /* 3.25 MHz ADC clock speed */
	.ave_ctrl		= 3, /* 8 sample average control */
	.fraction_z		= 7, /* 7 length fractional part in z */
	.i_drive		= 1, /* 50 mA typical 80 mA max touchscreen
					drivers current limit value */
	.mod_12b		= 1, /* 12-bit ADC */
	.ref_sel		= 0, /* internal ADC reference */
	.sample_time		= 4, /* ADC converstion time: 80 clocks */
	.settling		= 3, /* 1 ms panel driver settling time */
	.touch_det_delay	= 5, /* 5 ms touch detect interrupt delay */
};

/* STMPE811 ADC controller */
static struct stmpe_adc_platform_data stmpe811_adc_data = {
	.sample_time		= 4, /* ADC converstion time: 80 clocks */
	.mod_12b		= 1, /* 12-bit ADC */
	.ref_sel		= 0, /* internal ADC reference */
	.adc_freq		= 1, /* 3.25 MHz ADC clock speed */
};

static struct stmpe_platform_data stmpe811_data = {
	.blocks		= STMPE_BLOCK_TOUCHSCREEN | STMPE_BLOCK_ADC,
	.id		= 1,
	.irq_base	= STMPE811_IRQ_BASE,
	.irq_trigger	= IRQF_TRIGGER_FALLING,
	.ts		= &stmpe811_ts_data,
	.adc		= &stmpe811_adc_data,
};

static void lm95245_probe_callback(struct device *dev);

static struct lm95245_platform_data colibri_t30_lm95245_pdata = {
	.enable_os_pin	= true,
	.probe_callback	= lm95245_probe_callback,
};

static struct i2c_board_info colibri_t30_i2c_bus5_board_info[] __initdata = {
	{
		/* SGTL5000 audio codec */
		I2C_BOARD_INFO("sgtl5000", 0x0a),
	},
	{
		/* STMPE811 touch screen controller */
		I2C_BOARD_INFO("stmpe", 0x41),
			.flags = I2C_CLIENT_WAKE,
			.platform_data = &stmpe811_data,
			.type = "stmpe811",
	},
	{
		/* LM95245 temperature sensor
		   Note: OVERT_N directly connected to PMIC PWRDN */
		I2C_BOARD_INFO("lm95245", 0x4c),
			.platform_data = &colibri_t30_lm95245_pdata,
	},
};

static struct tegra_i2c_platform_data colibri_t30_i2c5_platform_data = {
	.adapter_nr	= 4,
	.arb_recovery	= arb_lost_recovery,
	.bus_clk_rate	= {400000, 0},
	.bus_count	= 1,
	.scl_gpio	= {PWR_I2C_SCL, 0},
	.sda_gpio	= {PWR_I2C_SDA, 0},
};

static void __init colibri_t30_i2c_init(void)
{
	tegra_i2c_device1.dev.platform_data = &colibri_t30_i2c1_platform_data;
	tegra_i2c_device4.dev.platform_data = &colibri_t30_i2c4_platform_data;
	tegra_i2c_device5.dev.platform_data = &colibri_t30_i2c5_platform_data;

	platform_device_register(&tegra_i2c_device1);
	platform_device_register(&tegra_i2c_device4);
	platform_device_register(&tegra_i2c_device5);

	i2c_register_board_info(0, colibri_t30_i2c_bus1_board_info,
				ARRAY_SIZE(colibri_t30_i2c_bus1_board_info));

	/* enable touch interrupt GPIO */
	gpio_request(TOUCH_PEN_INT, "TOUCH_PEN_INT");
	gpio_direction_input(TOUCH_PEN_INT);

	colibri_t30_i2c_bus5_board_info[1].irq = gpio_to_irq(TOUCH_PEN_INT);
	i2c_register_board_info(4, colibri_t30_i2c_bus5_board_info,
				ARRAY_SIZE(colibri_t30_i2c_bus5_board_info));
}



#ifdef CONFIG_KEYBOARD_GPIO
#define GPIO_KEY(_id, _gpio, _lowactive, _iswake)	\
	{						\
		.code = _id,				\
		.gpio = TEGRA_GPIO_##_gpio,		\
		.active_low = _lowactive,		\
		.desc = #_id,				\
		.type = EV_KEY,				\
		.wakeup = _iswake,			\
		.debounce_interval = 10,		\
	}

/* Note: Only wake-able gpios can be used as wakeup keys */
static struct gpio_keys_button colibri_t30_keys[] = {

	GPIO_KEY(KEY_POWER, PT5, 1, 0),		/* SODIMM 133 INPUT-VOLTAGE-LOW */
};

static struct gpio_keys_platform_data colibri_t30_keys_platform_data = {
	.buttons	= colibri_t30_keys,
	.nbuttons	= ARRAY_SIZE(colibri_t30_keys),
};

static struct platform_device colibri_t30_keys_device = {
	.name	= "gpio-keys",
	.id	= 0,
	.dev = {
		.platform_data = &colibri_t30_keys_platform_data,
	},
};
#endif /* CONFIG_KEYBOARD_GPIO */


static struct pps_gen_gpio_platform_data pps_gpio_pdata = {
	.gpio = TEGRA_GPIO_PM7,
};

static struct platform_device pps_gpio_device = {
	.name 		= "pps_gen_gpio",
	.id 		= -1,
	.dev		= {
		.platform_data = &pps_gpio_pdata,
	}
};

static struct gpio_led status_leds[] = {
	[0] =  {
		/* WIFI-GREEN on MX-4 T20/T30 */
		.name = "mx4-wifi",
		.default_trigger = "netdev",
		.gpio = TEGRA_GPIO_PN6,
		.active_low = 0,
		.default_state = LEDS_GPIO_DEFSTATE_OFF,
	},
	[1] =  {
		/* WIFI-RED on MX-4 T20/T30 */
		.name = "mx4-wifi-red",
		.default_trigger = "none",
		.gpio = TEGRA_GPIO_PB2,
		.active_low = 0,
		.default_state = LEDS_GPIO_DEFSTATE_OFF,
	},
};

static struct gpio_led_platform_data status_led_data = {
	.num_leds	= ARRAY_SIZE(status_leds),
	.leds		= status_leds
};

static struct platform_device colibri_t30_wifi_leds = {
	.name		= "leds-gpio",
	.id		= -1,
	.dev		= {
		.platform_data	= &status_led_data,
	},
};


/* MMC/SD */

#ifndef COLIBRI_T30_SDMMC4B
static struct tegra_sdhci_platform_data colibri_t30_emmc_platform_data = {
	.cd_gpio	= -1,
	.ddr_clk_limit	= 52000000,
	.is_8bit	= 1,
	.mmc_data = {
		.built_in = 1,
	},
	.power_gpio	= -1,
	.tap_delay	= 0x0f,
	.wp_gpio	= -1,
};
#endif /* COLIBRI_T30_SDMMC4B */

static struct tegra_sdhci_platform_data colibri_t30_sdcard_platform_data = {
	.cd_gpio	= -1,
	.ddr_clk_limit	= 52000000,
	.is_8bit	= 0,
	.power_gpio	= -1,
	.tap_delay	= 0x0f,
	.wp_gpio	= -1,
	.no_1v8		= 1,
};

#define WLAN_EN		TEGRA_GPIO_PO0
#define WLAN_CD		TEGRA_GPIO_PV2
#define BT_EN		TEGRA_GPIO_PCC6

static struct tegra_sdhci_platform_data colibri_t30_wifi_platform_data = {
	.cd_gpio	= -1,
	.ddr_clk_limit	= 52000000,
	.is_8bit	= 0,
	.power_gpio	= -1,
	.tap_delay	= 0x0f,
	.wp_gpio	= -1,
	.no_1v8		= 1,
};

static void __init colibri_t30_sdhci_init(void)
{
	gpio_request(WLAN_EN, "WLAN_EN");
	gpio_export(WLAN_EN, true);
	gpio_direction_output(WLAN_EN, 1);

	gpio_request(BT_EN, "BT_EN");
	gpio_export(BT_EN, true);
	gpio_direction_output(BT_EN, 0);

	/* register eMMC first */
	tegra_sdhci_device4.dev.platform_data =
#ifdef COLIBRI_T30_SDMMC4B
			&colibri_t30_sdcard_platform_data;
#else
			&colibri_t30_emmc_platform_data;
#endif
	platform_device_register(&tegra_sdhci_device4);

#ifndef COLIBRI_T30_SDMMC4B
	tegra_sdhci_device2.dev.platform_data =
			&colibri_t30_sdcard_platform_data;
	platform_device_register(&tegra_sdhci_device2);
#endif

	tegra_sdhci_device1.dev.platform_data =
			&colibri_t30_wifi_platform_data;
	platform_device_register(&tegra_sdhci_device1);
}



/* SPI */

#if defined(CONFIG_SPI_TEGRA) && defined(CONFIG_SPI_SPIDEV)
static struct tegra_spi_device_controller_data spidev_controller_data = {
	.cs_hold_clk_count	= 1,
	.cs_setup_clk_count	= 1,
	.is_hw_based_cs		= 1,
};

static struct mx4_io_platform_data mx4_io_pdata = {
	.event_rdy = TEGRA_GPIO_TO_IRQ(MX4_DATA_READY_PIC),
};

static struct spi_board_info tegra_spi_devices[] __initdata = {
	{
		.bus_num	= 0,		/* SPI1: Colibri SSP */
		.chip_select	= 0,
		.controller_data	= &spidev_controller_data,
		.irq		= TEGRA_GPIO_TO_IRQ(MX4_WAKE_UP_CPU),
		.max_speed_hz	= 12000000,
		.modalias	= "mx4_io_spi",
		.mode		= SPI_MODE_1,
		.platform_data	= &mx4_io_pdata,
	},
	{
		.bus_num		= 0,		/* SPI1: Colibri SSP */
		.chip_select		= 1,
		.controller_data	= &spidev_controller_data,
		.irq			= 0,
		.max_speed_hz		= 50000000,
		.modalias		= "spidev",
		.mode			= SPI_MODE_1,
		.platform_data		= NULL,
	},
	{
		.bus_num		= 1,		/* SPI2 */
		.chip_select		= 0,
		.controller_data	= &spidev_controller_data,
		.irq			= 0,
		.max_speed_hz		= 50000000,
		.modalias		= "spidev",
		.mode			= SPI_MODE_1,
		.platform_data		= NULL,
	},
};

static void __init colibri_t30_register_spidev(void)
{
	spi_register_board_info(tegra_spi_devices,
				ARRAY_SIZE(tegra_spi_devices));
}
#else /* CONFIG_SPI_TEGRA & CONFIG_SPI_SPIDEV */
#define colibri_t30_register_spidev() do {} while (0)
#endif /* CONFIG_SPI_TEGRA & CONFIG_SPI_SPIDEV */

static struct platform_device *colibri_t30_spi_devices[] __initdata = {
	&tegra_spi_device1,
	&tegra_spi_device2,
};

static struct spi_clk_parent spi_parent_clk[] = {
	[0] = {.name = "pll_p"},
#ifndef CONFIG_TEGRA_PLLM_RESTRICTED
	[1] = {.name = "pll_m"},
	[2] = {.name = "clk_m"},
#else /* !CONFIG_TEGRA_PLLM_RESTRICTED */
	[1] = {.name = "clk_m"},
#endif /* !CONFIG_TEGRA_PLLM_RESTRICTED */
};

static struct tegra_spi_platform_data colibri_t30_spi_pdata = {
	.is_dma_based		= true,
	.max_dma_buffer		= 16 * 1024,
	.is_clkon_always	= false,
	.max_rate		= 100000000,
};

static void __init colibri_t30_spi_init(void)
{
	int i;
	struct clk *c;

	for (i = 0; i < ARRAY_SIZE(spi_parent_clk); ++i) {
		c = tegra_get_clock_by_name(spi_parent_clk[i].name);
		if (IS_ERR_OR_NULL(c)) {
			pr_err("Not able to get the clock for %s\n",
						spi_parent_clk[i].name);
			continue;
		}
		spi_parent_clk[i].parent_clk = c;
		spi_parent_clk[i].fixed_clk_rate = clk_get_rate(c);
	}
	colibri_t30_spi_pdata.parent_clk_list = spi_parent_clk;
	colibri_t30_spi_pdata.parent_clk_count = ARRAY_SIZE(spi_parent_clk);
	tegra_spi_device1.dev.platform_data = &colibri_t30_spi_pdata;
	tegra_spi_device2.dev.platform_data = &colibri_t30_spi_pdata;
	platform_add_devices(colibri_t30_spi_devices,
				ARRAY_SIZE(colibri_t30_spi_devices));
}

/* Thermal throttling */

static void *colibri_t30_alert_data;
static void (*colibri_t30_alert_func)(void *);
static int colibri_t30_low_edge = 0;
static int colibri_t30_low_hysteresis = 3000;
static int colibri_t30_low_limit = 0;
static struct device *lm95245_device = NULL;
static int thermd_alert_irq_disabled = 0;
struct work_struct thermd_alert_work;
struct workqueue_struct *thermd_alert_workqueue;

static struct balanced_throttle throttle_list[] = {
#ifdef CONFIG_TEGRA_THERMAL_THROTTLE
	{
		.id = BALANCED_THROTTLE_ID_TJ,
		.throt_tab_size = 10,
		.throt_tab = {
			{      0, 1000 },
			{ 640000, 1000 },
			{ 640000, 1000 },
			{ 640000, 1000 },
			{ 640000, 1000 },
			{ 640000, 1000 },
			{ 760000, 1000 },
			{ 760000, 1050 },
			{1000000, 1050 },
			{1000000, 1100 },
		},
	},
#endif /* CONFIG_TEGRA_THERMAL_THROTTLE */
#ifdef CONFIG_TEGRA_SKIN_THROTTLE
	{
		.id = BALANCED_THROTTLE_ID_SKIN,
		.throt_tab_size = 6,
		.throt_tab = {
			{ 640000, 1200 },
			{ 640000, 1200 },
			{ 760000, 1200 },
			{ 760000, 1200 },
			{1000000, 1200 },
			{1000000, 1200 },
		},
	},
#endif /* CONFIG_TEGRA_SKIN_THROTTLE */
};

/* All units are in millicelsius */
static struct tegra_thermal_data thermal_data = {
	.shutdown_device_id = THERMAL_DEVICE_ID_NCT_EXT,
	.temp_shutdown = 115000,

#if defined(CONFIG_TEGRA_EDP_LIMITS) || defined(CONFIG_TEGRA_THERMAL_THROTTLE)
	.throttle_edp_device_id = THERMAL_DEVICE_ID_NCT_EXT,
#endif
#ifdef CONFIG_TEGRA_EDP_LIMITS
	.edp_offset = TDIODE_OFFSET, /* edp based on tdiode */
	.hysteresis_edp = 3000,
#endif
#ifdef CONFIG_TEGRA_THERMAL_THROTTLE
	.temp_throttle = 85000,
	.tc1 = 0,
	.tc2 = 1,
	.passive_delay = 2000,
#endif /* CONFIG_TEGRA_THERMAL_THROTTLE */
#ifdef CONFIG_TEGRA_SKIN_THROTTLE
	.skin_device_id = THERMAL_DEVICE_ID_SKIN,
	.temp_throttle_skin = 43000,
	.tc1_skin = 0,
	.tc2_skin = 1,
	.passive_delay_skin = 5000,

	.skin_temp_offset = 9793,
	.skin_period = 1100,
	.skin_devs_size = 2,
	.skin_devs = {
		{
			THERMAL_DEVICE_ID_NCT_EXT,
			{
				2, 1, 1, 1,
				1, 1, 1, 1,
				1, 1, 1, 0,
				1, 1, 0, 0,
				0, 0, -1, -7
			}
		},
		{
			THERMAL_DEVICE_ID_NCT_INT,
			{
				-11, -7, -5, -3,
				-3, -2, -1, 0,
				0, 0, 1, 1,
				1, 2, 2, 3,
				4, 6, 11, 18
			}
		},
	},
#endif /* CONFIG_TEGRA_SKIN_THROTTLE */
};

/* Over-temperature shutdown OS aka high limit GPIO pin interrupt handler */
static irqreturn_t thermd_alert_irq(int irq, void *data)
{
	disable_irq_nosync(irq);
	thermd_alert_irq_disabled = 1;
	queue_work(thermd_alert_workqueue, &thermd_alert_work);

	return IRQ_HANDLED;
}

/* Gets both entered by THERMD_ALERT GPIO interrupt as well as re-scheduled. */
static void thermd_alert_work_func(struct work_struct *work)
{
	int temp = 0;

	lm95245_get_remote_temp(lm95245_device, &temp);

	/* This emulates NCT1008 low limit behaviour */
	if (!colibri_t30_low_edge && temp <= colibri_t30_low_limit) {
		colibri_t30_alert_func(colibri_t30_alert_data);
		colibri_t30_low_edge = 1;
	} else if (colibri_t30_low_edge && temp > colibri_t30_low_limit +
						  colibri_t30_low_hysteresis) {
		colibri_t30_low_edge = 0;
	}

	/* Avoid unbalanced enable for IRQ 367 */
	if (thermd_alert_irq_disabled) {
		colibri_t30_alert_func(colibri_t30_alert_data);
		thermd_alert_irq_disabled = 0;
		enable_irq(gpio_to_irq(THERMD_ALERT));
	}

	/* Keep re-scheduling */
	msleep(2000);
	queue_work(thermd_alert_workqueue, &thermd_alert_work);
}

static int lm95245_get_temp(void *_data, long *temp)
{
	struct device *lm95245_device = _data;
	int lm95245_temp = 0;
	lm95245_get_remote_temp(lm95245_device, &lm95245_temp);
	*temp = lm95245_temp;
	return 0;
}

static int lm95245_get_temp_low(void *_data, long *temp)
{
	*temp = 0;
	return 0;
}

/* Our temperature sensor only allows triggering an interrupt on over-
   temperature shutdown aka the high limit we therefore need to setup a
   workqueue to catch leaving the low limit. */
static int lm95245_set_limits(void *_data,
			long lo_limit_milli,
			long hi_limit_milli)
{
	struct device *lm95245_device = _data;
	colibri_t30_low_limit = lo_limit_milli;
	if (lm95245_device) lm95245_set_remote_os_limit(lm95245_device,
							hi_limit_milli);
	return 0;
}

static int lm95245_set_alert(void *_data,
				void (*alert_func)(void *),
				void *alert_data)
{
	lm95245_device = _data;
	colibri_t30_alert_func = alert_func;
	colibri_t30_alert_data = alert_data;
	return 0;
}

static int lm95245_set_shutdown_temp(void *_data, long shutdown_temp)
{
	struct device *lm95245_device = _data;
	if (lm95245_device) lm95245_set_remote_critical_limit(lm95245_device,
							      shutdown_temp);
	return 0;
}

#ifdef CONFIG_TEGRA_SKIN_THROTTLE
/* Internal aka local board/case temp */
static int lm95245_get_itemp(void *dev_data, long *temp)
{
	struct device *lm95245_device = dev_data;
	int lm95245_temp = 0;
	lm95245_get_local_temp(lm95245_device, &lm95245_temp);
	*temp = lm95245_temp;
	return 0;
}
#endif /* CONFIG_TEGRA_SKIN_THROTTLE */

static void lm95245_probe_callback(struct device *dev)
{
	struct tegra_thermal_device *lm95245_remote;

	lm95245_remote = kzalloc(sizeof(struct tegra_thermal_device),
					GFP_KERNEL);
	if (!lm95245_remote) {
		pr_err("unable to allocate thermal device\n");
		return;
	}

	lm95245_remote->name = "lm95245_remote";
	lm95245_remote->id = THERMAL_DEVICE_ID_NCT_EXT;
	lm95245_remote->data = dev;
	lm95245_remote->offset = TDIODE_OFFSET;
	lm95245_remote->get_temp = lm95245_get_temp;
	lm95245_remote->get_temp_low = lm95245_get_temp_low;
	lm95245_remote->set_limits = lm95245_set_limits;
	lm95245_remote->set_alert = lm95245_set_alert;
	lm95245_remote->set_shutdown_temp = lm95245_set_shutdown_temp;

	tegra_thermal_device_register(lm95245_remote);

#ifdef CONFIG_TEGRA_SKIN_THROTTLE
	{
		struct tegra_thermal_device *lm95245_local;
		lm95245_local = kzalloc(sizeof(struct tegra_thermal_device),
						GFP_KERNEL);
		if (!lm95245_local) {
			kfree(lm95245_local);
			pr_err("unable to allocate thermal device\n");
			return;
		}

		lm95245_local->name = "lm95245_local";
		lm95245_local->id = THERMAL_DEVICE_ID_NCT_INT;
		lm95245_local->data = dev;
		lm95245_local->get_temp = lm95245_get_itemp;

		tegra_thermal_device_register(lm95245_local);
	}
#endif /* CONFIG_TEGRA_SKIN_THROTTLE */

	if (request_irq(gpio_to_irq(THERMD_ALERT), thermd_alert_irq,
			IRQF_TRIGGER_LOW, "THERMD_ALERT", NULL))
		pr_err("%s: unable to register THERMD_ALERT interrupt\n",
		       __func__);

	//initalize the local temp limit
	if(dev)
		lm95245_set_local_shared_os__critical_limit(dev, TCRIT_LOCAL);
}

static void colibri_t30_thermd_alert_init(void)
{
	gpio_request(THERMD_ALERT, "THERMD_ALERT");
	gpio_direction_input(THERMD_ALERT);

	thermd_alert_workqueue = create_singlethread_workqueue("THERMD_ALERT");

	INIT_WORK(&thermd_alert_work, thermd_alert_work_func);
}

/* UART */

static struct platform_device *colibri_t30_uart_devices[] __initdata = {
	&tegra_uarta_device, /* Colibri UART_A (formerly FFUART) */
	&tegra_uartd_device, /* Colibri UART_B (formerly BTUART) */
	&tegra_uartb_device, /* Colibri UART_C (formerly STDUART) */
	&tegra_uartc_device, /* UART 3 -> FR-MCU */
};

static struct uart_clk_parent uart_parent_clk[] = {
	[0] = {.name = "clk_m"},
	[1] = {.name = "pll_p"},
#ifndef CONFIG_TEGRA_PLLM_RESTRICTED
	[2] = {.name = "pll_m"},
#endif
};

static struct tegra_uart_platform_data colibri_t30_uart_pdata;

static void __init uart_debug_init(void)
{
	int debug_port_id;

	debug_port_id = get_tegra_uart_debug_port_id();
	if (debug_port_id < 0) {
		debug_port_id = 0;
	}

	switch (debug_port_id) {
	case 0:
		/* UARTA is the debug port. */
		pr_info("Selecting UARTA as the debug console\n");
		colibri_t30_uart_devices[0] = &debug_uarta_device;
		debug_uart_clk = clk_get_sys("serial8250.0", "uarta");
		debug_uart_port_base = ((struct plat_serial8250_port *)(
			debug_uarta_device.dev.platform_data))->mapbase;
		break;

	case 1:
		/* UARTB is the debug port. */
		pr_info("Selecting UARTB as the debug console\n");
		colibri_t30_uart_devices[2] = &debug_uartb_device;
		debug_uart_clk = clk_get_sys("serial8250.0", "uartb");
		debug_uart_port_base = ((struct plat_serial8250_port *)(
			debug_uartb_device.dev.platform_data))->mapbase;
		break;

	case 3:
		/* UARTD is the debug port. */
		pr_info("Selecting UARTD as the debug console\n");
		colibri_t30_uart_devices[1] = &debug_uartd_device;
		debug_uart_clk = clk_get_sys("serial8250.0", "uartd");
		debug_uart_port_base = ((struct plat_serial8250_port *)(
			debug_uartd_device.dev.platform_data))->mapbase;
		break;

	default:
		pr_info("The debug console id %d is invalid, Assuming UARTA",
			debug_port_id);
		colibri_t30_uart_devices[0] = &debug_uarta_device;
		debug_uart_clk = clk_get_sys("serial8250.0", "uarta");
		debug_uart_port_base = ((struct plat_serial8250_port *)(
			debug_uarta_device.dev.platform_data))->mapbase;
		break;
	}
}

static void __init colibri_t30_uart_init(void)
{
	struct clk *c;
	int i;

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
	colibri_t30_uart_pdata.parent_clk_list = uart_parent_clk;
	colibri_t30_uart_pdata.parent_clk_count = ARRAY_SIZE(uart_parent_clk);
	tegra_uarta_device.dev.platform_data = &colibri_t30_uart_pdata;
	tegra_uartb_device.dev.platform_data = &colibri_t30_uart_pdata;
	tegra_uartc_device.dev.platform_data = &colibri_t30_uart_pdata;
	tegra_uartd_device.dev.platform_data = &colibri_t30_uart_pdata;

	/* Register low speed only if it is selected */
	if (!is_tegra_debug_uartport_hs()) {
		uart_debug_init();
		/* Clock enable for the debug channel */
		if (!IS_ERR_OR_NULL(debug_uart_clk)) {
			pr_info("The debug console clock name is %s\n",
						debug_uart_clk->name);
			c = tegra_get_clock_by_name("pll_p");
			if (IS_ERR_OR_NULL(c))
				pr_err("Not getting the parent clock pll_p\n");
			else
				clk_set_parent(debug_uart_clk, c);

			clk_enable(debug_uart_clk);
			clk_set_rate(debug_uart_clk, clk_get_rate(c));
		} else {
			pr_err("Not getting the clock %s for debug console\n",
					debug_uart_clk->name);
		}
	}

	platform_add_devices(colibri_t30_uart_devices,
				ARRAY_SIZE(colibri_t30_uart_devices));
}

/* USB */

//TODO: overcurrent

static struct tegra_usb_platform_data tegra_udc_pdata = {
	.has_hostpc	= true,
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

static struct tegra_usb_platform_data tegra_ehci1_utmi_pdata = {
	.has_hostpc	= true,
	.op_mode	= TEGRA_USB_OPMODE_HOST,
	.phy_intf	= TEGRA_USB_PHY_INTF_UTMI,
	.port_otg	= true,
	.u_cfg.utmi = {
		.elastic_limit		= 16,
		.hssync_start_delay	= 0,
		.idle_wait_delay	= 17,
		.term_range_adj		= 6,
		.xcvr_lsfslew		= 2,
		.xcvr_lsrslew		= 2,
		.xcvr_setup		= 15,
		.xcvr_setup_offset	= 0,
		.xcvr_use_fuses		= 1,
	},
	.u_data.host = {
		.hot_plug			= true,
		.power_off_on_suspend		= true,
		.remote_wakeup_supported	= true,
		.vbus_gpio			= -1,
		.vbus_reg			= NULL,
	},
};

static void ehci2_utmi_platform_post_phy_on(void)
{
	/* enable VBUS */
	gpio_set_value(LAN_V_BUS, 1);

	/* reset */
	gpio_set_value(LAN_RESET, 0);

	udelay(5);

	/* unreset */
	gpio_set_value(LAN_RESET, 1);
}

static void ehci2_utmi_platform_pre_phy_off(void)
{
	/* disable VBUS */
	gpio_set_value(LAN_V_BUS, 0);
}

static struct tegra_usb_phy_platform_ops ehci2_utmi_plat_ops = {
	.post_phy_on = ehci2_utmi_platform_post_phy_on,
	.pre_phy_off = ehci2_utmi_platform_pre_phy_off,
};

static struct tegra_usb_platform_data tegra_ehci2_utmi_pdata = {
	.has_hostpc	= true,
	.op_mode	= TEGRA_USB_OPMODE_HOST,
	.ops		= &ehci2_utmi_plat_ops,
	.phy_intf	= TEGRA_USB_PHY_INTF_UTMI,
	.port_otg	= false,
	.u_cfg.utmi = {
		.elastic_limit		= 16,
		.hssync_start_delay	= 0,
		.idle_wait_delay	= 17,
		.term_range_adj		= 6,
		.xcvr_lsfslew		= 2,
		.xcvr_lsrslew		= 2,
		.xcvr_setup		= 15,
		.xcvr_setup_offset	= 0,
		.xcvr_use_fuses		= 1,
	},
	.u_data.host = {
		.hot_plug			= false,
		.power_off_on_suspend		= true,
		.remote_wakeup_supported	= true,
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
	.has_hostpc	= true,
	.op_mode	= TEGRA_USB_OPMODE_HOST,
	.ops 		= &modem_link_plat_ops,
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
	.u_data.host = {
		.hot_plug			= true,
		.power_off_on_suspend		= true,
		.remote_wakeup_supported	= true,
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

static void colibri_t30_usb_init(void)
{
	gpio_request(LAN_V_BUS, "LAN_V_BUS");
	gpio_direction_output(LAN_V_BUS, 0);
	gpio_export(LAN_V_BUS, false);

	gpio_request(LAN_RESET, "LAN_RESET");
	gpio_direction_output(LAN_RESET, 0);
	gpio_export(LAN_RESET, false);

	/* OTG should be the first to be registered
	   EHCI instance 0: USB1_DP/N -> USBC_P/N */
#ifndef CONFIG_USB_TEGRA_OTG
	platform_device_register(&colibri_otg_device);
#else /* !CONFIG_USB_TEGRA_OTG */
	tegra_otg_device.dev.platform_data = &tegra_otg_pdata;
	platform_device_register(&tegra_otg_device);
#endif /* !CONFIG_USB_TEGRA_OTG */

	/* setup the udc platform data */
	tegra_udc_device.dev.platform_data = &tegra_udc_pdata;
	platform_device_register(&tegra_udc_device);

	/* EHCI instance 1: USB2_DP/N -> AX88772B */
	tegra_ehci2_device.dev.platform_data = &tegra_ehci2_utmi_pdata;
	platform_device_register(&tegra_ehci2_device);

	/* EHCI instance 2: USB3_DP/N -> USBH_P/N */
	tegra_ehci3_device.dev.platform_data = &tegra_ehci3_utmi_pdata;
	platform_device_register(&tegra_ehci3_device);
}

static struct platform_device *colibri_t30_devices[] __initdata = {
	&tegra_pmu_device,
#if defined(CONFIG_TEGRA_IOVMM_SMMU) || defined(CONFIG_TEGRA_IOMMU_SMMU)
	&tegra_smmu_device,
#endif
	&tegra_wdt0_device,
	&tegra_wdt1_device,
	&tegra_wdt2_device,
#if defined(CONFIG_TEGRA_AVP)
	&tegra_avp_device,
#endif
#ifdef CONFIG_TEGRA_CAMERA
	&tegra_camera,
#endif
#if defined(CONFIG_CRYPTO_DEV_TEGRA_SE)
	&tegra_se_device,
#endif
#if defined(CONFIG_CRYPTO_DEV_TEGRA_AES)
	&tegra_aes_device,
#endif
#ifdef CONFIG_KEYBOARD_GPIO
	&colibri_t30_keys_device,
#endif
	&colibri_t30_wifi_leds,
	&pps_gpio_device,
	&tegra_ahub_device,
	&tegra_dam_device0,
	&tegra_dam_device1,
	&tegra_dam_device2,
	&tegra_i2s_device2,
	&tegra_pcm_device,
	&colibri_t30_audio_sgtl5000_device,
	&tegra_hda_device,
	&tegra_cec_device,
	&tegra_pwfm1_device,
	&tegra_pwfm2_device,
	&tegra_pwfm3_device,
};

static void __init colibri_t30_init(void)
{
	tegra_thermal_init(&thermal_data,
				throttle_list,
				ARRAY_SIZE(throttle_list));
	tegra_clk_init_from_table(colibri_t30_clk_init_table);
	colibri_t30_pinmux_init();
#if defined(CONFIG_CAN_SJA1000) || defined(CONFIG_CAN_SJA1000_MODULE)
	sja1000_init();
#endif /* CONFIG_CAN_SJA1000 | CONFIG_CAN_SJA1000_MODULE */
	colibri_t30_thermd_alert_init();
	colibri_t30_i2c_init();
	colibri_t30_spi_init();
	colibri_t30_usb_init();
#ifdef CONFIG_TEGRA_EDP_LIMITS
	colibri_t30_edp_init();
#endif
	colibri_t30_uart_init();

	platform_add_devices(colibri_t30_devices,
			     ARRAY_SIZE(colibri_t30_devices));
	tegra_ram_console_debug_init();
	tegra_io_dpd_init();
	colibri_t30_sdhci_init();
	colibri_t30_regulator_init();
	colibri_t30_suspend_init();
	colibri_t30_panel_init();
//	colibri_t30_sensors_init();
	colibri_t30_emc_init();
	colibri_t30_register_spidev();
	colibri_t30_gpio_init();

	tegra_release_bootloader_fb();
#ifdef CONFIG_TEGRA_WDT_RECOVERY
	tegra_wdt_recovery_init();
#endif
	tegra_serial_debug_init(TEGRA_UARTD_BASE, INT_WDT_CPU, NULL, -1, -1);

	/* Activate Mic Bias */
	gpio_request(EN_MIC_GND, "EN_MIC_GND");
	gpio_direction_output(EN_MIC_GND, 1);
}

static void __init colibri_t30_reserve(void)
{
#if defined(CONFIG_NVMAP_CONVERT_CARVEOUT_TO_IOVMM)
	/* Support 1920X1080 32bpp,double buffered on HDMI*/
	tegra_reserve(0, SZ_8M + SZ_1M, SZ_16M);
#else
	tegra_reserve(SZ_128M, SZ_8M, SZ_8M);
#endif
	tegra_ram_console_debug_reserve(SZ_1M);
}

static const char *colibri_t30_dt_board_compat[] = {
	"toradex,colibri_t30",
	NULL
};

#ifdef CONFIG_ANDROID
MACHINE_START(COLIBRI_T30, "cardhu")
#else
MACHINE_START(COLIBRI_T30, "Toradex Colibri T30 on MX-4 T20/T30 board")
#endif
	.boot_params	= 0x80000100,
	.dt_compat	= colibri_t30_dt_board_compat,
	.init_early	= tegra_init_early,
	.init_irq	= tegra_init_irq,
	.init_machine	= colibri_t30_init,
	.map_io		= tegra_map_common_io,
	.reserve	= colibri_t30_reserve,
	.timer		= &tegra_timer,
MACHINE_END
