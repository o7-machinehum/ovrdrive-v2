/********************************** (C) COPYRIGHT *******************************
 * File Name          : sw_udisk.h
 * Description        : USB Mass Storage (MSC/BOT) protocol layer header
 *                      Ported from WCH CH569 example to HydraUSB3 BSP
 *
 * This header defines the BOT (Bulk-Only Transport) protocol structures
 * and SCSI command opcodes for the USB Mass Storage device.
 *
 * Key specs:
 *   [ref/USB_MSC_Bulk_Only_Transport_v1.0.pdf] - CBW/CSW structures
 *   [ref/SCSI_Block_Commands_SBC3_r25.pdf]     - SCSI command opcodes
 *   [ref/USB_MSC_Overview_v1.4.pdf]             - Class/SubClass/Protocol
 *
 * SPDX-License-Identifier: Apache-2.0
 *******************************************************************************/
#ifndef SW_UDISK_H_
#define SW_UDISK_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "CH56x_common.h"
#include "CH56x_usb30_devbulk.h"
#include "CH56x_usb20_devbulk.h"
#include "CH56x_usb30_devbulk_LIB.h"
#include "CH56x_emmc.h"

/******************************************************************************/
/* eMMC/SD DMA buffer size.
 * 40KB = 80 sectors of 512 bytes = 10 x 4KB blocks.
 * This buffer lives in RAMX (.DMADATA section) which is DMA-accessible.
 * Two buffers exist: UDisk_In_Buf (read path) and UDisk_Out_Buf (write path).
 * For reads, this acts as a circular buffer between eMMC DMA and USB sends.
 * For writes, sectors are received into this buffer then written via BSP.
 * [CH569DS1.PDF: RAMX memory region, DMA constraints] */
#define UDISKSIZE (1024*4*10)

/******************************************************************************/
/* BulkOnly Mass Storage class-specific requests
 * These are USB control requests (bRequest values) specific to the
 * Mass Storage class, sent on the default control pipe (EP0).
 * [ref/USB_MSC_Bulk_Only_Transport_v1.0.pdf: Section 3 - Requests] */
#define CMD_UDISK_RESET                 0xFF  /* Bulk-Only Mass Storage Reset */
#define CMD_UDISK_GET_MAX_LUN           0xFE  /* Get Max LUN (we return 0) */

/******************************************************************************/
/* SCSI Command opcodes
 * These appear in byte 0 of the CBW's Command Block (mCBW_CB_Buf[0]).
 * The host sends these inside CBW packets to control the storage device.
 * [ref/SCSI_Block_Commands_SBC3_r25.pdf: Command opcodes] */
#define CMD_U_TEST_READY                0x00  /* TEST UNIT READY - check if device ready */
#define CMD_U_REZERO_UNIT               0x01  /* REZERO UNIT - seek to track 0 (legacy) */
#define CMD_U_REQUEST_SENSE             0x03  /* REQUEST SENSE - get error details after failure */
#define CMD_U_FORMAT_UNIT               0x04  /* FORMAT UNIT - low-level format (not supported) */
#define CMD_U_INQUIRY                   0x12  /* INQUIRY - identify device (vendor, product) */
#define CMD_U_MODE_SELECT               0x15  /* MODE SELECT(6) - set device parameters */
#define CMD_U_RELEASE                   0x17  /* RELEASE - release reserved device */
#define CMD_U_MODE_SENSE                0x1A  /* MODE SENSE(6) - get device parameters */
#define CMD_U_START_STOP                0x1B  /* START STOP UNIT - eject/load media */
#define CMD_U_SEND_DIAG                 0x1D  /* SEND DIAGNOSTIC - self-test */
#define CMD_U_PREVT_REMOVE              0x1E  /* PREVENT ALLOW MEDIUM REMOVAL */
#define CMD_U_READ_FORMAT_CAPACITY      0x23  /* READ FORMAT CAPACITIES - MMC/UFI specific */
#define CMD_U_READ_CAPACITY             0x25  /* READ CAPACITY(10) - get size and block length */
#define CMD_U_READ10                    0x28  /* READ(10) - read sectors from device */
#define CMD_U_WRITE10                   0x2A  /* WRITE(10) - write sectors to device */
#define CMD_U_SEEK10                    0x2B  /* SEEK(10) - seek to LBA (legacy) */
#define CMD_U_WR_VERIFY10               0x2E  /* WRITE AND VERIFY(10) - write with verify */
#define CMD_U_VERIFY10                  0x2F  /* VERIFY(10) - verify sectors */
#define CMD_U_SYNC_CACHE                0x35  /* SYNCHRONIZE CACHE(10) - flush write cache */
#define CMD_U_READ_TOC                  0x43  /* READ TOC - CD-ROM table of contents (N/A) */
#define CMD_U_MODE_SENSE2               0x5A  /* MODE SENSE(10) - get device parameters (long) */
#define CMD_U_READ12                    0xA8  /* READ(12) - read sectors (extended) */
#define CMD_U_WRITE12                   0xAA  /* WRITE(12) - write sectors (extended) */

/******************************************************************************/
/* Default disk size placeholder (overridden at runtime by Udisk_Capability).
 * The actual size comes from SD card CSD register parsing in SDReadCSD().
 * [ref/SD_Physical_Layer_Spec_v6.00.pdf: Section 5.3 - CSD Register] */
#define MY_UDISK_SIZE  0x00000040

/******************************************************************************/
/* BulkOnly Command/Status/Sense union
 *
 * This union overlays three structures on the same 31-byte buffer:
 *   mCBW      - Command Block Wrapper (host→device, 31 bytes)
 *   mCSW      - Command Status Wrapper (device→host, 13 bytes)
 *   ReqSense  - REQUEST SENSE response data (18 bytes)
 *
 * CBW layout [ref/USB_MSC_Bulk_Only_Transport_v1.0.pdf: Section 5.1]:
 *   Bytes 0-3:   Signature 'USBC' (0x55534243)
 *   Bytes 4-7:   Tag (echoed back in CSW to match command/response)
 *   Bytes 8-11:  Data Transfer Length (bytes host expects to transfer)
 *   Byte 12:     Flags (bit 7: 0=OUT/write, 1=IN/read)
 *   Byte 13:     LUN (always 0 for single-LUN devices)
 *   Byte 14:     CB Length (length of command block, 1-16)
 *   Bytes 15-30: Command Block (SCSI CDB, up to 16 bytes)
 *
 * CSW layout [ref/USB_MSC_Bulk_Only_Transport_v1.0.pdf: Section 5.2]:
 *   Bytes 0-3:   Signature 'USBS' (0x55534253)
 *   Bytes 4-7:   Tag (must match CBW tag)
 *   Bytes 8-11:  Data Residue (bytes NOT transferred)
 *   Byte 12:     Status (0=Passed, 1=Failed, 2=Phase Error)
 */
typedef union _BULK_ONLY_CMD
{
	uint8_t buf[ 31 ];
	struct
	{
		uint8_t mCBW_Sig[ 4 ];      /* Signature: must be 'USBC' */
		uint8_t mCBW_Tag[ 4 ];      /* Tag: echoed in CSW */
		uint8_t mCBW_DataLen[ 4 ];  /* Total data transfer length (LE) */
		uint8_t mCBW_Flag;          /* Bit 7: direction (1=IN, 0=OUT) */
		uint8_t mCBW_LUN;           /* Logical Unit Number */
		uint8_t mCBW_CB_Len;        /* Command Block length */
		uint8_t mCBW_CB_Buf[ 16 ];  /* SCSI Command Descriptor Block (CDB) */
	} mCBW;
	struct
	{
		uint8_t mCSW_Sig[ 4 ];      /* Signature: must be 'USBS' */
		uint8_t mCSW_Tag[ 4 ];      /* Tag: copied from CBW */
		uint8_t mCSW_Residue[ 4 ];  /* Data residue (untransferred bytes) */
		uint8_t mCSW_Status;        /* 0=Passed, 1=Failed, 2=Phase Error */
	} mCSW;
	struct
	{
		/* Fixed-format sense data [SBC-3: Section 4.5]
		 * Returned in response to REQUEST SENSE after a command failure */
		uint8_t ErrorCode;           /* 0x70 = current errors */
		uint8_t Reserved1;
		uint8_t SenseKey;            /* Category of error (see below) */
		uint8_t Information[ 4 ];
		uint8_t SenseLength;         /* Additional sense length (0x0A) */
		uint8_t Reserved2[ 4 ];
		uint8_t SenseCode;           /* Additional Sense Code (ASC) */
		uint8_t SenseCodeQua;        /* ASC Qualifier (ASCQ) */
		uint8_t Reserved3[ 4 ];
		/* Common Sense Key / ASC combinations used in this firmware:
		 *   0x00/0x00 = No Sense (success)
		 *   0x02/0x3A = Not Ready / Medium Not Present
		 *   0x03/0x0C = Medium Error / Write Error
		 *   0x05/0x20 = Illegal Request / Invalid Command */
	} ReqSense;
} BULK_ONLY_CMD;

/******************************************************************************/
/* Disk configuration constants */
#define DEF_CFG_DISK_SEC_SIZE      512   /* Sector size in bytes (always 512 for SD) */
#define DEF_UDISK_SECTOR_SIZE      DEF_CFG_DISK_SEC_SIZE
#define DEF_UDISK_PACK_512         512   /* USB2 max packet size for bulk (HS) */
#define DEF_UDISK_PACK_64          64    /* USB1.1 max packet size (not used) */

/******************************************************************************/
/* Udisk_Status flags */
#define DEF_UDISK_EN_FLAG              0x01  /* SD card initialized and ready */

/******************************************************************************/
/* Udisk_Transfer_Status flags - track BOT protocol state machine
 * [ref/USB_MSC_Bulk_Only_Transport_v1.0.pdf: Command/Data/Status Protocol]
 *
 * BLUCK_UP_FLAG:   Data-IN phase pending (device→host, e.g., READ10 data)
 * BLUCK_DOWN_FLAG: Data-OUT phase pending (host→device, e.g., WRITE10 data)
 * CSW_UP_FLAG:     CSW needs to be sent after data phase completes */
#define DEF_UDISK_BLUCK_UP_FLAG        0x01
#define DEF_UDISK_BLUCK_DOWN_FLAG      0x02
#define DEF_UDISK_CSW_UP_FLAG          0x04

/******************************************************************************/
/* External variables */
extern volatile uint8_t  Udisk_CBW_Tag_Save[ 4 ];  /* Saved CBW tag for CSW */
extern volatile uint8_t  Udisk_Sense_Key;           /* SCSI sense key for REQUEST SENSE */
extern volatile uint8_t  Udisk_Sense_ASC;           /* SCSI ASC for REQUEST SENSE */
extern volatile uint8_t  Udisk_CSW_Status;          /* CSW status: 0=pass, 1=fail */

extern volatile uint32_t UDISK_Transfer_DataLen;    /* Remaining bytes in data phase */
extern volatile uint32_t UDISK_Cur_Sec_Lba;         /* Current sector LBA for READ/WRITE */
extern volatile uint16_t UDISK_Sec_Pack_Count;      /* Sectors transferred so far */
extern volatile uint16_t UDISK_Pack_Size;            /* USB packet size (512 for HS) */

extern BULK_ONLY_CMD     mBOC;                      /* CBW/CSW/Sense shared buffer */

extern volatile uint8_t  Udisk_Status;              /* DEF_UDISK_EN_FLAG if SD ready */
extern volatile uint8_t  Udisk_Transfer_Status;     /* BOT state flags */
extern volatile uint32_t Udisk_Capability;           /* SD card sector count */

/* Flags set by USB ISR, consumed by main loop.
 * This deferred-processing pattern avoids running DMA transfers inside ISRs.
 * InPackflag=1: host requested READ10 data, main loop calls UDISK_Up_OnePack()
 * OutPackflag=1: host sent WRITE10 data, main loop calls UDISK_Down_OnePack() */
extern volatile uint8_t  UDISK_OutPackflag;
extern volatile uint8_t  UDISK_InPackflag;

/* eMMC/SD card parameter structure (init state, capacity, sector size, RCA)
 * [ref/SD_Physical_Layer_Spec_v6.00.pdf: Card registers] */
extern EMMC_PARAMETER TF_EMMCParam;

/* DMA buffers in RAMX (.DMADATA linker section). 16-byte aligned for DMA.
 * UDisk_In_Buf:  eMMC reads into here, USB sends from here (read path)
 * UDisk_Out_Buf: USB receives into here, eMMC writes from here (write path)
 * [CH569DS1.PDF: RAMX is the only DMA-capable memory region] */
extern __attribute__((aligned(16))) uint8_t UDisk_In_Buf[UDISKSIZE] __attribute__((section(".DMADATA")));
extern __attribute__((aligned(16))) uint8_t UDisk_Out_Buf[UDISKSIZE] __attribute__((section(".DMADATA")));

/* SCSI response tables (fixed data returned for INQUIRY, CAPACITY, etc.) */
extern uint8_t  UDISK_Inquity_Tab[ ];
extern uint8_t  const  UDISK_Rd_Format_Capacity[ ];
extern uint8_t  const  UDISK_Rd_Capacity[ ];
extern uint8_t  const  UDISK_Mode_Sense_1A[ ];
extern uint8_t  const  UDISK_Mode_Senese_5A[ ];

/******************************************************************************/
/* Function declarations */
extern void UDISK_CMD_Deal_Status( uint8_t key, uint8_t asc, uint8_t status );
extern void UDISK_CMD_Deal_Fail( void );
extern void UDISK_SCSI_CMD_Deal( void );
extern void UDISK_Bulk_UpData( void );
extern void UDISK_Up_CSW( void );
extern void UDISK_Up_OnePack( void );
extern void UDISK_Out_EP_Deal( uint8_t *pbuf, uint16_t packlen );
extern void UDISK_In_EP_Deal( void );
extern void UDISK_Down_OnePack( void );
extern void UDISK_onePack_Deal( void );

#ifdef __cplusplus
}
#endif

#endif /* SW_UDISK_H_ */
