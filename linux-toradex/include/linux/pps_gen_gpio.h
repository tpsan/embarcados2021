/*
 * Driver model for GPIO PPS generator
 *
 * Copyright (C) 2016 Host Mobility AB
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */
#ifndef __LINUX_PPS_GEN_GPIO_H_INCLUDED
#define __LINUX_PPS_GEN_GPIO_H_INCLUDED

struct pps_gen_gpio_platform_data {
	unsigned gpio;
};

#endif
