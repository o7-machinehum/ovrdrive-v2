/********************************** (C) COPYRIGHT *******************************
* File Name          : Main.c
* Description        : USB Mass Storage Device (MSC/BOT) using HydraUSB3 BSP
*                      USB2 High Speed and USB3 Super Speed
*                      eMMC/SD storage via CH569 EMMC controller
* SPDX-License-Identifier: Apache-2.0
*******************************************************************************/
#include "CH56x_common.h"
#include "CH56x_debug_log.h"

#include "CH56x_usb30_devbulk.h"
#include "CH56x_usb20_devbulk.h"
#include "CH56x_usb30_devbulk_LIB.h"
#include "CH56x_usb_devbulk_desc_cmd.h"

#include "sw_udisk.h"
#include "sd.h"

#undef FREQ_SYS
/* System clock / MCU frequency in Hz */
#define FREQ_SYS (120000000)

#if(defined DEBUG)
#define UART1_BAUD (115200)
#endif

/* Blink time in ms */
#define BLINK_FAST (50)
#define BLINK_USB3 (250)
#define BLINK_USB2 (500)

int blink_ms = BLINK_USB2;

debug_log_buf_t log_buf;

/* FLASH_ROMA Read Unique ID (8bytes/64bits) */
#define FLASH_ROMA_UID_ADDR (0x77fe4)
usb_descriptor_serial_number_t unique_id;

/* USB VID PID - use default from BSP */
usb_descriptor_usb_vid_pid_t vid_pid =
{
	.vid = { .id_16b = 0x16C0 },
	.pid = { .id_16b = 0x05DC }
};

/*********************************************************************
 * @fn      main
 *
 * @brief   Main program.
 *
 * @return  none
 */
int main()
{
	/* HydraUSB3 configure GPIO In/Out */
	bsp_gpio_init();

	/* Init BSP (MCU Frequency & SysTick) */
	bsp_init(FREQ_SYS);
	log_init(&log_buf);

#if(defined DEBUG)
	/* Configure serial debugging for printf()/log_printf()... */
	UART1_init(UART1_BAUD, FREQ_SYS);
#endif
	log_printf("HydraUSB3_MSC Start\r\n");
	log_printf("ChipID(Hex)=%02X\r\n", R8_CHIP_ID);

	memset(&unique_id, 0, 8);
	FLASH_ROMA_READ(FLASH_ROMA_UID_ADDR, (uint32_t*)&unique_id, 8);

	log_printf("HydraUSB3_MSC FW v0.1 (CPr Freq=%d MHz)\r\n", (FREQ_SYS/1000000));

	// USB2 & USB3 Init
	R32_USB_CONTROL = 0;
	PFIC_EnableIRQ(USBSS_IRQn);
	PFIC_EnableIRQ(LINK_IRQn);

	PFIC_EnableIRQ(TMR0_IRQn);
	R8_TMR0_INTER_EN = RB_TMR_IE_CYC_END;
	TMR0_TimerInit(67000000); // USB3.0 connection failure timeout about 0.56 seconds

	/* USB Descriptor set String Serial Number with CH569 Unique ID */
	usb_descriptor_set_string_serial_number(&unique_id);

	/* USB Descriptor set USB VID/PID */
	usb_descriptor_set_usb_vid_pid(&vid_pid);

	/* USB3.0 initialization */
	USB30D_init(ENABLE);

	/* SD card init */
	log_printf("Initializing SD card...\r\n");

	uint8_t sta = SDCardInit(&TF_EMMCParam);

	/* Enable EMMC interrupt AFTER init (matches WCH reference) */
	PFIC_EnableIRQ(EMMC_IRQn);
	TF_EMMCParam.EMMCOpErr = 0;

	if(sta == OP_SUCCESS)
	{
		Udisk_Capability = TF_EMMCParam.EMMCSecNum;
		Udisk_Status |= DEF_UDISK_EN_FLAG;
		log_printf("SD OK: %d sectors (%d MB)\r\n",
			TF_EMMCParam.EMMCSecNum,
			TF_EMMCParam.EMMCSecNum / 2048);
	}
	else
	{
		log_printf("SD init FAILED (sta=%d)\r\n", sta);
	}

	log_printf("MSC ready, entering main loop\r\n");

	// Infinite loop - MSC protocol handled via interrupts + main loop polling
	while(1)
	{
		UDISK_onePack_Deal();
	}
}

/*********************************************************************
 * @fn      HardFault_Handler
 *
 * @brief   HardFault Handler
 *
 * @return  none
 */
__attribute__((interrupt("WCH-Interrupt-fast"))) void HardFault_Handler(void)
{
	printf("HardFault\r\n");
	printf(" SP=0x%08X\r\n", __get_SP());
	printf(" MIE=0x%08X\r\n", __get_MIE());
	printf(" MSTATUS=0x%08X\r\n", __get_MSTATUS());
	printf(" MCAUSE=0x%08X\r\n", __get_MCAUSE());
	bsp_wait_ms_delay(1);
}
