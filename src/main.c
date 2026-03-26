/********************************** (C) COPYRIGHT *******************************
* File Name          : Main.c
* Description        : USB Mass Storage Device (MSC/BOT) using HydraUSB3 BSP
*                      USB2 High Speed and USB3 Super Speed
*                      eMMC/SD storage via CH569 EMMC controller
*
* Architecture Overview:
*   This firmware turns a CH569W microcontroller into a USB flash drive.
*   The host PC sees a standard USB Mass Storage device. Internally,
*   data is stored on an SD card connected to the CH569's eMMC controller.
*
*   Data flow:
*     Host PC  <--USB-->  CH569 (this firmware)  <--SD bus-->  SD Card
*
*   Protocol stack (top to bottom):
*     USB MSC Bulk-Only Transport  [ref/USB_MSC_Bulk_Only_Transport_v1.0.pdf]
*     SCSI Block Commands          [ref/SCSI_Block_Commands_SBC3_r25.pdf]
*     SD Physical Layer            [ref/SD_Physical_Layer_Spec_v6.00.pdf]
*     CH569 Hardware Registers     [CH569DS1.PDF datasheet]
*
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
#include "ovrdrive.h"

#undef FREQ_SYS
/* System clock / MCU frequency in Hz (CH569 max = 120MHz)
 * [CH569DS1.PDF: System Clock Configuration] */
#define FREQ_SYS (120000000)

#if(defined DEBUG)
/* UART1 baud rate for debug serial output via log_printf()
 * Connected to UART1 on the HydraUSB3 board (PA2=TX, PA3=RX) */
#define UART1_BAUD (115200)
#endif

/* LED blink intervals to visually indicate USB connection speed */
#define BLINK_FAST (50)
#define BLINK_USB3 (250)   /* Fast blink = USB 3.0 SuperSpeed (5Gbps) */
#define BLINK_USB2 (500)   /* Slow blink = USB 2.0 High-Speed (480Mbps) */

int blink_ms = BLINK_USB2;

/* Debug log ring buffer - stores log_printf() output for UART1 transmission */
debug_log_buf_t log_buf;

/* CH569 has a 64-bit unique ID burned into flash ROM at this address.
 * Used as USB serial number descriptor so each board is uniquely identified.
 * [CH569DS1.PDF: FLASH_ROMA Unique ID] */
#define FLASH_ROMA_UID_ADDR (0x77fe4)
usb_descriptor_serial_number_t unique_id;

/* USB Vendor/Product ID
 * VID 0x16C0 = Van Ooijen Technische Informatica (shared/community VID)
 * PID 0x05DC = generic vendor-class device with libusb
 * [ref/USB_MSC_Overview_v1.4.pdf: USB descriptors] */
usb_descriptor_usb_vid_pid_t vid_pid =
{
	.vid = { .id_16b = 0x16C0 },
	.pid = { .id_16b = 0x05DC }
};

/*********************************************************************
 * @fn      main
 *
 * @brief   Main program entry point.
 *
 *          Initialization order:
 *            1. GPIO and BSP (clock tree, SysTick)
 *            2. UART1 for debug output
 *            3. USB3/USB2 controller and descriptors
 *            4. SD card (via CH569 eMMC controller)
 *            5. Enter infinite main loop polling for MSC transfers
 *
 *          The main loop calls UDISK_onePack_Deal() which checks
 *          flags set by USB interrupt handlers. When the host sends
 *          a SCSI READ10 or WRITE10 command, the ISR sets a flag
 *          and the main loop handles the bulk data transfer with
 *          interrupts disabled for deterministic timing.
 *
 * @return  none (never returns)
 */
int main()
{
	bsp_gpio_init();
	bsp_init(FREQ_SYS);
	log_init(&log_buf);

#if(defined DEBUG)
	UART1_init(UART1_BAUD, FREQ_SYS);
#endif
	log_printf("HydraUSB3_MSC Start\r\n");
	log_printf("ChipID(Hex)=%02X\r\n", R8_CHIP_ID);

	/* Read 64-bit unique chip ID from flash ROM for USB serial number */
	memset(&unique_id, 0, 8);
	FLASH_ROMA_READ(FLASH_ROMA_UID_ADDR, (uint32_t*)&unique_id, 8);

	log_printf("HydraUSB3_MSC FW v0.1 (CPr Freq=%d MHz)\r\n", (FREQ_SYS/1000000));

	/* ── USB Controller Initialization ──
	 * The CH569 has separate USB2.0 HS and USB3.0 SS controllers.
	 * Both share endpoint buffers (endp1Rbuff, endp1Tbuff in RAMX).
	 * g_DeviceUsbType is set by the BSP to indicate which speed connected.
	 * [CH569DS1.PDF: USB2.0/USB3.0 Controller] */
	R32_USB_CONTROL = 0;
	PFIC_EnableIRQ(USBSS_IRQn);   /* USB3.0 SuperSpeed interrupt */
	PFIC_EnableIRQ(LINK_IRQn);    /* USB3.0 link-layer interrupt (LTSSM) */

	/* Timer0 used as USB3.0 connection timeout (~0.56 seconds).
	 * If USB3 link training fails, firmware falls back to USB2.
	 * [CH569DS1.PDF: Timer0 Configuration] */
	PFIC_EnableIRQ(TMR0_IRQn);
	R8_TMR0_INTER_EN = RB_TMR_IE_CYC_END;
	TMR0_TimerInit(67000000);

	/* Set USB descriptor strings from chip unique ID
	 * [ref/USB_MSC_Overview_v1.4.pdf: String Descriptors] */
	usb_descriptor_set_string_serial_number(&unique_id);
	usb_descriptor_set_usb_vid_pid(&vid_pid);

	/* Start USB3.0 link training. If the host only supports USB2,
	 * the Timer0 timeout triggers fallback to USB2 HS mode.
	 * [USB 3.0 Spec: Link Training and Status State Machine (LTSSM)] */
	USB30D_init(ENABLE);

	/* ── SD Card Initialization ──
	 * Uses the CH569's built-in eMMC controller to talk to an SD card.
	 * Init sequence follows SD Physical Layer spec Section 4.2:
	 *   CMD0 → CMD8 → ACMD41 → CMD2 → CMD3 → CMD9 → CMD7 → ACMD6 → ACMD51
	 * [ref/SD_Physical_Layer_Spec_v6.00.pdf: Card Initialization] */
	log_printf("Initializing SD card...\r\n");

	uint8_t sta = SDCardInit(&TF_EMMCParam);

	/* Enable eMMC error interrupt AFTER init completes.
	 * The ISR (EMMC_IRQHandler in sw_udisk.c) sets EMMCOpErr flag on
	 * any error interrupt. Only error conditions are enabled (not BKGAP
	 * or TRANDONE), which are polled during data transfers.
	 * [CH569DS1.PDF: EMMC Interrupt Enable Register R16_EMMC_INT_EN] */
	PFIC_EnableIRQ(EMMC_IRQn);
	TF_EMMCParam.EMMCOpErr = 0;

	if(sta == OP_SUCCESS)
	{
		Udisk_Status |= DEF_UDISK_EN_FLAG;  /* Mark device as ready */
		log_printf("SD OK: %d sectors (%d MB)\r\n",
			TF_EMMCParam.EMMCSecNum,
			TF_EMMCParam.EMMCSecNum / 2048);

		/* Initialize OVRDrive encrypt-in-place system.
		 * Sets locked capacity (8GB) and scans for unlock.txt.
		 * If found, derives key, wipes file, and re-enumerates USB. */
		ovrd_init();
	}
	else
	{
		/* SD init failed - device will respond NOT READY to TEST UNIT READY
		 * and return Sense Key 0x02 (NOT READY), ASC 0x3A (MEDIUM NOT PRESENT)
		 * [ref/SCSI_Block_Commands_SBC3_r25.pdf: Sense Keys] */
		log_printf("SD init FAILED (sta=%d)\r\n", sta);
	}

	log_printf("MSC ready, entering main loop\r\n");

	/* ── Main Loop ──
	 * USB interrupts handle CBW reception and small responses.
	 * Large data transfers (READ10/WRITE10) are deferred to the main
	 * loop via UDISK_InPackflag / UDISK_OutPackflag to avoid running
	 * lengthy DMA operations inside an ISR.
	 * [ref/USB_MSC_Bulk_Only_Transport_v1.0.pdf: Command/Data/Status flow] */
	while(1)
	{
		UDISK_onePack_Deal();
	}
}

/*********************************************************************
 * @fn      HardFault_Handler
 *
 * @brief   RISC-V hard fault exception handler.
 *          Prints diagnostic registers for debugging.
 *          Uses WCH's fast interrupt attribute for minimal latency.
 *          [QingKe V3 Processor Manual: Exception Handling]
 *
 * @return  none (should not return; system is in undefined state)
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
