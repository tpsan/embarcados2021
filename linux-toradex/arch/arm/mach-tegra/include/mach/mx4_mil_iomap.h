#ifndef __MACH_MX4_IOMAP_H
#define __MACH_MX4_IOMAP_H

#include <asm/sizes.h>
#include <../gpio-names.h>

/* Host Mobility MX-4 MAP*/
#define GMI_CS_BASE_TEGRA 0xd0000000 /* Nor base addr*/

#define SNOR_CS_PIN				4

/* In 16bit mode the external address pin A[0] correlates to the internal memory address bit 1,
   external address pin A[1] to internal memory address bit 2, and so on. Found in Colibri_T20_Datasheet 5.6.1*/
#define SNOR_ADDR_HW_TO_VIRTUAL(x) (x << 1)

#define CAN_ADDRESS_TO_HW(x) (GMI_CS_BASE_TEGRA + SNOR_ADDR_HW_TO_VIRTUAL(x))

#define CAN_ADDRESS_SIZE			0x1f

#define CAN_OFFSET_MUXED			0x00000
#define CAN2_OFFSET_MUXED			0x20000
#define CAN3_OFFSET_MUXED			0x40000

#define CAN_OFFSET_NON_MUXED			0x180
#define CAN2_OFFSET_NON_MUXED			0x1A0

#define TEGRA_CAN_INT			TEGRA_GPIO_PB5
#define TEGRA_CAN2_INT			TEGRA_GPIO_PA6
#define TEGRA_CAN3_INT			TEGRA_GPIO_PN0

#define GPIO_WAKEUP_PIN			TEGRA_GPIO_PC7

struct mx4_io_platform_data {
	int event_rdy;
};

/* End Host Mobility MX-4 MAP*/
#endif
