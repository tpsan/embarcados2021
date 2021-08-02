/*
 * arch/arm/mach-tegra/mx4-pm.c
 *
 * Custom suspend action for mx4 board.
 *
 * Copyright (c) 2013-2016, Host Mobility AB. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */
#include <asm/mach/arch.h>
#include <linux/io.h>
#include <asm/mach-types.h>

#include <linux/kernel.h>
#include <linux/syscore_ops.h>

#include <mach/gpio.h>

#include "gpio-names.h"

#define __BITMASK0(len)				((1 << (len)) - 1)
#define __BITMASK(start, len)		(__BITMASK0(len) << (start))
#define REG_BIT(bit)				(1 << (bit))
#define REG_FIELD(val, start, len)	(((val) & __BITMASK0(len)) << (start))
#define REG_FIELD_MASK(start, len)	(~(__BITMASK((start), (len))))
#define REG_GET_FIELD(val, start, len)	(((val) >> (start)) & __BITMASK0(len))
#define TEGRA_GMI_PHYS				0x70009000
#define TEGRA_GMI_BASE				IO_TO_VIRT(TEGRA_GMI_PHYS)
#define CONFIG_REG					(TEGRA_GMI_BASE + 0x00)
#define STATUS_REG					(TEGRA_GMI_BASE + 0x04)
#define CONFIG_SNOR_CS(val) 		REG_FIELD((val), 4, 3)
#define CONFIG_GO					REG_BIT(31)
#define POWER_DOWN					REG_BIT(19)


static struct gpio gpios_to_handle[] = {
	{TEGRA_GPIO_PY4,	GPIOF_OUT_INIT_LOW,	"P35 - FF-TXD"},
	{TEGRA_GPIO_PY5,	GPIOF_OUT_INIT_LOW,	"P33 - FF-RXD"},
	{TEGRA_GPIO_PC2,	GPIOF_OUT_INIT_LOW,	"P21 - STD-TXD"},
	{TEGRA_GPIO_PK7,	GPIOF_OUT_INIT_LOW,	"P32 - BT-RTS"},
	{TEGRA_GPIO_PJ7,	GPIOF_OUT_INIT_LOW,	"P38 - BT-TXD"},


	{TEGRA_GPIO_PI0,	GPIOF_OUT_INIT_LOW,	"P89 - nWE"},
	{TEGRA_GPIO_PI1,	GPIOF_OUT_INIT_LOW,	"P91 - nOE"},
	{TEGRA_GPIO_PW0,	GPIOF_OUT_INIT_LOW,	"P93 - RDnWR"},
	{TEGRA_GPIO_PK2,	GPIOF_OUT_INIT_LOW,	"P105 - nCS"},
#ifdef CONFIG_MACH_COLIBRI_T30
	{TEGRA_GPIO_PS2,	GPIOF_OUT_INIT_LOW,	"P47 - SDIO_CLK"},
	{TEGRA_GPIO_PS3,	GPIOF_OUT_INIT_LOW,	"P190 - SDIO_CMD"},
	{TEGRA_GPIO_PS4,	GPIOF_OUT_INIT_LOW,	"P192 - SDIO_DAT0"},
	{TEGRA_GPIO_PS5,	GPIOF_OUT_INIT_LOW,	"P49 - SDIO_DAT1"},
	{TEGRA_GPIO_PS6,	GPIOF_OUT_INIT_LOW,	"P51 - SDIO_DAT2"},
	{TEGRA_GPIO_PS7,	GPIOF_OUT_INIT_LOW,	"P53 - SDIO_DAT3"},
#endif
};

#if 0
static struct tegra_pingroup_config mx4_pinconfig[] = {
	{TEGRA_PINGROUP_SDIO1,	TEGRA_MUX_UARTE,	TEGRA_PUPD_PULL_UP,	TEGRA_TRI_NORMAL},
	{TEGRA_PINGROUP_UAD,	TEGRA_MUX_IRDA,		TEGRA_PUPD_NORMAL,	TEGRA_TRI_NORMAL},
	{TEGRA_PINGROUP_GMC,	TEGRA_MUX_GMI, 		TEGRA_PUPD_NORMAL, 	TEGRA_TRI_NORMAL},
};
#endif

static int tegra_mx4_custom_suspend(void)
{
	int length = sizeof(gpios_to_handle) / sizeof(struct gpio);
	int err = 0, i;
	unsigned long flags;

	printk( KERN_INFO "Entering custom mx4 suspend rutine!");

	local_irq_save(flags);
	for (i = 0; i < length; i++) {
		err = gpio_request_one(gpios_to_handle[i].gpio, gpios_to_handle[i].flags, "function tri-stated");
		if (err) {
			pr_warning("gpio_request(%d) failed, err = %d",
				   gpios_to_handle[i].gpio, err);
		}
		tegra_gpio_enable(gpios_to_handle[i].gpio);
	}
	local_irq_restore(flags);
	return 0;
}

static void tegra_mx4_custom_resume(void)
{
	int length = sizeof(gpios_to_handle) / sizeof(struct gpio);
	int i;
	unsigned long flags;


	local_irq_save(flags);
	printk( KERN_INFO "Entering custom mx4 resume rutine!");

	for (i = 0; i < length; i++) {
		tegra_gpio_disable(gpios_to_handle[i].gpio);
		gpio_free(gpios_to_handle[i].gpio);
	}

	local_irq_restore(flags);
}

static struct syscore_ops tegra_mx4_custom_syscore_ops = {
	.suspend = tegra_mx4_custom_suspend,
	.resume = tegra_mx4_custom_resume,
};

static int tegra_mx4_custom_syscore_init(void){
	register_syscore_ops(&tegra_mx4_custom_syscore_ops);
	return 0;
}
arch_initcall(tegra_mx4_custom_syscore_init);
