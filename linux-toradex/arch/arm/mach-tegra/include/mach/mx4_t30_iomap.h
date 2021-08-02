#ifndef __MACH_MX4_T30_IOMAP_H
#define __MACH_MX4_T30_IOMAP_H

#include <asm/sizes.h>
#include <../gpio-names.h>

/* Host Mobility MX-4 MAP*/
#define GMI_CS_BASE_TEGRA 0x48000000 /* Nor base addr*/

#define TEGRA_SNOR_CS_PIN				4

/* In 16bit mode the external address pin A[0] correlates to the internal memory address bit 1,
   external address pin A[1] to internal memory address bit 2, and so on. Found in Colibri_T20_Datasheet 5.6.1*/
#define PXA_ADDR_TO_T20_ADDR(x) (x << 1)

#define TEGRA_CAN_SIZE				0xff

#define CAN_OFFSET			0x00000
#define CAN2_OFFSET			0x20000
#define CAN3_OFFSET			0x40000
#define CAN4_OFFSET			0x50000
#define CAN5_OFFSET			0x60000
#define CAN6_OFFSET			0x70000

#define TEGRA_CAN_BASE 			GMI_CS_BASE_TEGRA + PXA_ADDR_TO_T20_ADDR(CAN_OFFSET)
#define TEGRA_CAN_INT			TEGRA_GPIO_PB5

#define TEGRA_CAN2_BASE 		(GMI_CS_BASE_TEGRA + PXA_ADDR_TO_T20_ADDR(CAN2_OFFSET))
#define TEGRA_CAN2_INT			TEGRA_GPIO_PA6

#define TEGRA_CAN3_BASE         (GMI_CS_BASE_TEGRA + PXA_ADDR_TO_T20_ADDR(CAN3_OFFSET))
#define TEGRA_CAN3_INT			TEGRA_GPIO_PN0

#define TEGRA_CAN4_BASE         (GMI_CS_BASE_TEGRA + PXA_ADDR_TO_T20_ADDR(CAN4_OFFSET))
#define TEGRA_CAN4_INT			TEGRA_GPIO_PN1

#define TEGRA_CAN5_BASE         (GMI_CS_BASE_TEGRA + PXA_ADDR_TO_T20_ADDR(CAN5_OFFSET))
#define TEGRA_CAN5_INT			TEGRA_GPIO_PN2

#define TEGRA_CAN6_BASE         (GMI_CS_BASE_TEGRA + PXA_ADDR_TO_T20_ADDR(CAN6_OFFSET))
#define TEGRA_CAN6_INT			TEGRA_GPIO_PN3

#define MX4_WIFI_LED				TEGRA_GPIO_PN6
#define MX4_CT_WIFI_LED				TEGRA_GPIO_PAA6
#define MX4_WIFI_RED				TEGRA_GPIO_PB2

#define MX4_WAKE_UP_CPU				TEGRA_GPIO_PO5
#define MX4_DATA_READY_PIC			TEGRA_GPIO_PV1

#define MMC_CD		-1 /* Not connected */

struct mx4_io_platform_data {
	int event_rdy;
};

#endif /* __MACH_MX4_T30_IOMAP_H */
