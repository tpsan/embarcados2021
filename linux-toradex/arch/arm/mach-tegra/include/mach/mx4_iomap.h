#ifndef __MACH_MX4_IOMAP_H
#define __MACH_MX4_IOMAP_H

#include <asm/sizes.h>
#include <../gpio-names.h>

/* Host Mobility MX-4 MAP*/
#define GMI_CS_BASE_TEGRA 0xd0000000 /* Nor base addr*/

#define SNOR_CS_PIN				4

/* In 16bit mode the external address pin A[0] correlates to the internal memory address bit 1,
   external address pin A[1] to internal memory address bit 2, and so on. Found in Colibri_T20_Datasheet 5.6.1*/
#ifndef CONFIG_HM_GMI_MUX
#define PXA_ADDR_TO_T20_ADDR(x) (x << 1)
#else
#define PXA_ADDR_TO_T20_ADDR(x) (x << 2)
#endif /* !CONFIG_HM_GMI_MUX */

#ifndef CONFIG_HM_GMI_MUX
#define TEGRA_CAN_SIZE				0x3f
#else
#define TEGRA_CAN_SIZE				0xff
#endif /* !CONFIG_HM_GMI_MUX */

#ifdef CONFIG_HM_GTT_CAN
	#ifndef CONFIG_HM_GMI_MUX
		#define CAN_OFFSET			0x000
		#define CAN2_OFFSET			0x020
		#define CAN3_OFFSET			0x040
		#define CAN4_OFFSET			0x060
		#define CAN5_OFFSET			0x080
		#define CAN6_OFFSET			0x0A0
	#else
		#define CAN_OFFSET			0x00000
		#define CAN2_OFFSET			0x20000
		#define CAN3_OFFSET			0x40000
		#define CAN4_OFFSET			0x50000
		#define CAN5_OFFSET			0x60000
		#define CAN6_OFFSET			0x70000
	#endif /* !CONFIG_HM_GMI_MUX */
#else
	#define CAN_OFFSET			0x180
	#define CAN2_OFFSET			0x1A0
#endif /* CONFIG_HM_GTT_CAN */

#define TEGRA_CAN_BASE 			GMI_CS_BASE_TEGRA + PXA_ADDR_TO_T20_ADDR(CAN_OFFSET)
#define TEGRA_CAN_INT			TEGRA_GPIO_PB5

#define TEGRA_CAN2_BASE 		(GMI_CS_BASE_TEGRA + PXA_ADDR_TO_T20_ADDR(CAN2_OFFSET))
#define TEGRA_CAN2_INT			TEGRA_GPIO_PA6

#ifdef CONFIG_HM_GTT_CAN
	#define TEGRA_CAN3_BASE         (GMI_CS_BASE_TEGRA + PXA_ADDR_TO_T20_ADDR(CAN3_OFFSET))
	#define TEGRA_CAN3_INT			TEGRA_GPIO_PN0

	#define TEGRA_CAN4_BASE         (GMI_CS_BASE_TEGRA + PXA_ADDR_TO_T20_ADDR(CAN4_OFFSET))
	#define TEGRA_CAN4_INT			TEGRA_GPIO_PN1

	#define TEGRA_CAN5_BASE         (GMI_CS_BASE_TEGRA + PXA_ADDR_TO_T20_ADDR(CAN5_OFFSET))
	#define TEGRA_CAN5_INT			TEGRA_GPIO_PN2

	#define TEGRA_CAN6_BASE         (GMI_CS_BASE_TEGRA + PXA_ADDR_TO_T20_ADDR(CAN6_OFFSET))
	#define TEGRA_CAN6_INT			TEGRA_GPIO_PN3
#endif /* CONFIG_HM_GTT_CAN */


#ifdef CONFIG_HM_GMI_MUX
	#define UARTA_OFFSET				0x000
	#define UARTB_OFFSET				0x000
#ifndef CONFIG_MACH_HM_MX4_VCC
	#define UARTC_OFFSET				0x80000 /* MA19 */
#else
	#define UARTC_OFFSET				0x2000000 /* MA25 */
#endif /* !CONFIG_MACH_HM_MX4_VCC */
#else
	#define UARTA_OFFSET				0x000
	#define UARTB_OFFSET				0x80
	#define UARTC_OFFSET				0x100
#endif /* CONFIG_HM_GMI_MUX*/


#define TEGRA_EXT_UARTA_BASE 		(GMI_CS_BASE_TEGRA + PXA_ADDR_TO_T20_ADDR(UARTA_OFFSET))
#define TEGRA_EXT_UARTA_INT			TEGRA_GPIO_PT2

#define TEGRA_EXT_UARTB_BASE 		(GMI_CS_BASE_TEGRA + PXA_ADDR_TO_T20_ADDR(UARTB_OFFSET))
#define TEGRA_EXT_UARTB_INT			TEGRA_GPIO_PBB2

#define TEGRA_EXT_UARTC_BASE 		(GMI_CS_BASE_TEGRA + PXA_ADDR_TO_T20_ADDR(UARTC_OFFSET))
#ifdef CONFIG_MACH_HM_MX4_VCC
#define TEGRA_EXT_UARTC_INT			TEGRA_GPIO_PU3
#else
#define TEGRA_EXT_UARTC_INT			TEGRA_GPIO_PK5
#endif /* CONFIG_MACH_HM_MX4_VCC */

#define GPIO_WAKEUP_PIN				TEGRA_GPIO_PC7
#define CAN_WAKEUP_PIN				TEGRA_GPIO_PB6

#define MX4_WIFI_LED				TEGRA_GPIO_PN6
#define MX4_CT_WIFI_LED				TEGRA_GPIO_PAA6
#define MX4_WIFI_RED				TEGRA_GPIO_PB2


#define MX4_WAKE_UP_CPU				TEGRA_GPIO_PY6
#define MX4_DATA_READY_PIC			TEGRA_GPIO_PV3

#if defined(MACH_HM_MX4_GTT)
#define MMC_CD		TEGRA_GPIO_PT4	/* SODIMM 71 */
#elif defined(CONFIG_MACH_HM_MX4_VCC_T20)
#define MMC_CD		-1 /* Not connected */
#else
#define MMC_CD		TEGRA_GPIO_PBB3	/* SODIMM 127 */
#endif /* CONFIG_MACH_HM_MX4_VCC */


struct mx4_io_platform_data {
	int event_rdy;
};


/* End Host Mobility MX-4 MAP*/
#endif
