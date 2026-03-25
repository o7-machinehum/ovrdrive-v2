/********************************** (C) COPYRIGHT *******************************
 * File Name          : sw_udisk.c
 * Description        : USB Mass Storage (MSC/BOT) protocol layer
 *                      Ported from WCH CH569 example to HydraUSB3 BSP
 *
 * This module implements the complete USB Mass Storage Class using Bulk-Only
 * Transport (BOT). It sits between the USB endpoint handlers (ISR-level) and
 * the eMMC/SD card driver (BSP), translating SCSI block commands into SD card
 * read/write operations.
 *
 * Protocol stack (this file implements the middle two layers):
 *   Host (Linux/Windows)
 *     |  USB MSC BOT protocol  [ref/USB_MSC_Bulk_Only_Transport_v1.0.pdf]
 *   CBW/CSW handling (UDISK_Out_EP_Deal, UDISK_SCSI_CMD_Deal, UDISK_Up_CSW)
 *     |  SCSI Block Commands   [ref/SCSI_Block_Commands_SBC3_r25.pdf]
 *   SCSI command dispatch (CMD_U_READ10 → UDISK_Up_OnePack, etc.)
 *     |  eMMC/SD driver         [wch-ch56x-bsp/drv/CH56x_emmc.c]
 *   SD card hardware            [ref/SD_Physical_Layer_Spec_v6.00.pdf]
 *
 * Data flow:
 *   READ10:  SD card --DMA--> UDisk_In_Buf --USB--> Host
 *   WRITE10: Host --USB--> UDisk_Out_Buf --BSP--> SD card
 *
 * The BOT protocol state machine:
 *   1. Host sends CBW (31 bytes) on bulk OUT endpoint
 *      [ref/USB_MSC_Bulk_Only_Transport_v1.0.pdf: Section 5.1 - CBW]
 *   2. Optional data phase (IN or OUT depending on command)
 *   3. Device sends CSW (13 bytes) on bulk IN endpoint
 *      [ref/USB_MSC_Bulk_Only_Transport_v1.0.pdf: Section 5.2 - CSW]
 *
 * Uses EP1 IN (0x81) / EP1 OUT (0x01) for bulk transfers.
 * Supports both USB2 High-Speed (480Mbps) and USB3 SuperSpeed (5Gbps).
 *
 * SPDX-License-Identifier: Apache-2.0
 *******************************************************************************/

#include "sw_udisk.h"
#include "CH56x_debug_log.h"

/******************************************************************************/
/* eMMC interrupt handler declaration and card parameter structure.
 * The IRQ handler fires on eMMC errors (FIFO overflow, transfer error,
 * data timeout, response index error, response CRC error, response timeout).
 * Error conditions are enabled in EMMCIO0Init() via R16_EMMC_INT_EN.
 * [CH569DS1.PDF: EMMC Interrupt Enable Register R16_EMMC_INT_EN] */
void EMMC_IRQHandler(void) __attribute__((interrupt));
EMMC_PARAMETER TF_EMMCParam;

/* eMMC/SD DMA buffers in RAMX (.DMADATA linker section).
 * RAMX (96KB starting at 0x20020000) is the only DMA-capable memory on CH569.
 * Both buffers are 40KB (80 sectors) and 16-byte aligned for DMA requirements.
 *
 * UDisk_In_Buf:  Read path  - eMMC DMA reads into this, USB sends from this.
 *                Used as a circular buffer in UDISK_Up_OnePack() where eMMC
 *                fills sectors while USB simultaneously drains them.
 * UDisk_Out_Buf: Write path - USB receives into this, then BSP writes to eMMC.
 *                Used as a linear buffer in UDISK_Down_OnePack().
 * [CH569DS1.PDF: RAMX memory region, DMA address constraints] */
__attribute__((aligned(16))) uint8_t UDisk_In_Buf[UDISKSIZE] __attribute__((section(".DMADATA")));
__attribute__((aligned(16))) uint8_t UDisk_Out_Buf[UDISKSIZE] __attribute__((section(".DMADATA")));

/******************************************************************************/
/* SCSI INQUIRY response data (36 bytes).
 * Returned when host sends INQUIRY command (opcode 0x12).
 * This identifies the device to the operating system.
 * [ref/SCSI_Block_Commands_SBC3_r25.pdf: INQUIRY command]
 *
 * Layout:
 *   Byte 0:    Peripheral Device Type (0x00 = direct-access block device)
 *   Byte 1:    RMB (0x80 = removable media)
 *   Byte 2:    Version (0x02 = SCSI-2 compliance)
 *   Byte 3:    Response Data Format (0x02 = standard)
 *   Byte 4:    Additional Length (0x1F = 31 bytes follow)
 *   Bytes 8-15:  Vendor Identification ("HydraUSB")
 *   Bytes 16-31: Product Identification ("USB3 MSC    ")
 *   Bytes 32-35: Product Revision ("Disk WCH") */
uint8_t UDISK_Inquity_Tab[ ] =
{
    0x00,                                                /* Peripheral Device Type: direct-access (disk) */
    0x80,                                                /* Removable media bit set */
    0x02,                                                /* SCSI-2 compliance version */
    0x02,                                                /* Response data format */
    0x1F,                                                /* Additional Length (31 bytes follow) */
    0x00,
    0x00,
    0x00,
    'H',                                                 /* Vendor Information (8 bytes) */
    'y',
    'd',
    'r',
    'a',
    'U',
    'S',
    'B',
    'U',                                                 /* Product Identification (16 bytes) */
    'S',
    'B',
    '3',
    ' ',
    'M',
    'S',
    'C',
    ' ',
    ' ',
    ' ',
    ' ',
    'D',                                                 /* Product Revision (4 bytes) */
    'i',
    's',
    'k',
    ' ',
    'W',
    'C',
    'H'
};

/******************************************************************************/
/* READ FORMAT CAPACITIES response (12 bytes).
 * Returned for SCSI opcode 0x23 (READ FORMAT CAPACITIES).
 * This is an MMC/UFI command used by Windows during device enumeration.
 * Values here are placeholders - bytes 4-7 are overridden at runtime with
 * actual SD card sector count from Udisk_Capability.
 *
 * Layout:
 *   Bytes 0-3: Capacity List Header (length = 8)
 *   Bytes 4-7: Number of Blocks (big-endian, overridden at runtime)
 *   Byte 8:    Descriptor Code (0x02 = formatted media)
 *   Bytes 9-11: Block Length (512 bytes, big-endian)
 * [ref/SCSI_Block_Commands_SBC3_r25.pdf: READ FORMAT CAPACITIES] */
uint8_t  const  UDISK_Rd_Format_Capacity[ ] =
{
    0x00,
    0x00,
    0x00,
    0x08,
    ( MY_UDISK_SIZE >> 24 ) & 0xFF,
    ( MY_UDISK_SIZE >> 16 ) & 0xFF,
    ( MY_UDISK_SIZE >> 8 ) & 0xFF,
    ( MY_UDISK_SIZE ) & 0xFF,
    0x02,
    ( DEF_CFG_DISK_SEC_SIZE >> 16 ) & 0xFF,
    ( DEF_CFG_DISK_SEC_SIZE >> 8 ) & 0xFF,
    ( DEF_CFG_DISK_SEC_SIZE ) & 0xFF,
};

/******************************************************************************/
/* READ CAPACITY(10) response (8 bytes).
 * Returned for SCSI opcode 0x25 (READ CAPACITY).
 * Reports the last valid LBA (total sectors - 1) and block size.
 * Values here are placeholders - bytes 0-3 are overridden at runtime
 * with (Udisk_Capability - 1) from the SD card's CSD register.
 *
 * Layout:
 *   Bytes 0-3: Last Logical Block Address (big-endian, overridden at runtime)
 *   Bytes 4-7: Block Length in bytes (512, big-endian)
 * [ref/SCSI_Block_Commands_SBC3_r25.pdf: READ CAPACITY(10)]
 * [ref/SD_Physical_Layer_Spec_v6.00.pdf: Section 5.3 - CSD Register] */
uint8_t  const  UDISK_Rd_Capacity[ ] =
{
    ( ( MY_UDISK_SIZE - 1 ) >> 24 ) & 0xFF,
    ( ( MY_UDISK_SIZE - 1 ) >> 16 ) & 0xFF,
    ( ( MY_UDISK_SIZE - 1 ) >> 8 ) & 0xFF,
    ( ( MY_UDISK_SIZE - 1 ) ) & 0xFF,
    ( DEF_CFG_DISK_SEC_SIZE >> 24 ) & 0xFF,
    ( DEF_CFG_DISK_SEC_SIZE >> 16 ) & 0xFF,
    ( DEF_CFG_DISK_SEC_SIZE >> 8 ) & 0xFF,
    ( DEF_CFG_DISK_SEC_SIZE ) & 0xFF,
};

/******************************************************************************/
/* MODE SENSE(6) response (12 bytes) for SCSI opcode 0x1A.
 * Returns device parameters including write-protect status and capacity.
 * Byte 2 = 0x00 means write-unprotected (read-write).
 * Bytes 4-7 are overridden at runtime with actual capacity.
 *
 * Layout:
 *   Byte 0:    Mode Data Length (0x0B = 11 bytes follow)
 *   Byte 1:    Medium Type (0x00)
 *   Byte 2:    Device-Specific Parameter (0x00 = not write-protected)
 *   Byte 3:    Block Descriptor Length (0x08)
 *   Bytes 4-7: Number of Blocks (big-endian, overridden at runtime)
 *   Bytes 8-11: Block Length (512 bytes, big-endian)
 * [ref/SCSI_Block_Commands_SBC3_r25.pdf: MODE SENSE(6)] */
uint8_t  const  UDISK_Mode_Sense_1A[ ] =
{
    0x0B,
    0x00,
    0x00,                                               /* 0x00:write-unprotected */
    0x08,
    ( ( MY_UDISK_SIZE - 1 ) >> 24 ) & 0xFF,
    ( ( MY_UDISK_SIZE - 1 )  >> 16 ) & 0xFF,
    ( ( MY_UDISK_SIZE - 1 )  >> 8 ) & 0xFF,
    ( ( MY_UDISK_SIZE - 1 )  ) & 0xFF,
    ( DEF_CFG_DISK_SEC_SIZE >> 24 ) & 0xFF,
    ( DEF_CFG_DISK_SEC_SIZE >> 16 ) & 0xFF,
    ( DEF_CFG_DISK_SEC_SIZE >> 8 ) & 0xFF,
    ( DEF_CFG_DISK_SEC_SIZE ) & 0xFF,
};

/******************************************************************************/
/* MODE SENSE(10) response (16 bytes) for SCSI opcode 0x5A.
 * Same information as MODE SENSE(6) but with a longer header.
 * Byte 3 = 0x00 means write-unprotected.
 * Bytes 8-11 are overridden at runtime with actual capacity.
 * [ref/SCSI_Block_Commands_SBC3_r25.pdf: MODE SENSE(10)] */
uint8_t  const  UDISK_Mode_Senese_5A[ ] =
{
    0x00,
    0x0E,
    0x00,
    0x00,                                              /* 0x00:write-unprotected */
    0x00,
    0x00,
    0x00,
    0x08,
    ( ( MY_UDISK_SIZE - 1 ) >> 24 ) & 0xFF,
    ( ( MY_UDISK_SIZE - 1 )  >> 16 ) & 0xFF,
    ( ( MY_UDISK_SIZE - 1 )  >> 8 ) & 0xFF,
    ( ( MY_UDISK_SIZE - 1 )  ) & 0xFF,
    ( DEF_CFG_DISK_SEC_SIZE >> 24 ) & 0xFF,
    ( DEF_CFG_DISK_SEC_SIZE >> 16 ) & 0xFF,
    ( DEF_CFG_DISK_SEC_SIZE >> 8 ) & 0xFF,
    ( DEF_CFG_DISK_SEC_SIZE ) & 0xFF,
};


/******************************************************************************/
/* BOT protocol state variables.
 *
 * Udisk_Status:          Bit flags indicating device readiness.
 *                        DEF_UDISK_EN_FLAG (0x01) = SD card initialized.
 *                        Checked by TEST UNIT READY and all data commands.
 *
 * Udisk_Transfer_Status: Bit flags tracking current BOT state machine phase.
 *                        DEF_UDISK_BLUCK_UP_FLAG   (0x01) = data-IN pending
 *                        DEF_UDISK_BLUCK_DOWN_FLAG (0x02) = data-OUT pending
 *                        DEF_UDISK_CSW_UP_FLAG     (0x04) = CSW send pending
 *                        [ref/USB_MSC_Bulk_Only_Transport_v1.0.pdf: Section 5]
 *
 * Udisk_Capability:      Total sector count from SD card CSD register.
 *                        Set in main.c after SDCardInit() succeeds.
 *                        Used by READ CAPACITY, MODE SENSE, FORMAT CAPACITY.
 *                        [ref/SD_Physical_Layer_Spec_v6.00.pdf: Section 5.3] */
volatile uint8_t  Udisk_Status = 0x00;
volatile uint8_t  Udisk_Transfer_Status = 0x00;
volatile uint32_t Udisk_Capability = 0x00;

/* CBW tag is saved from the incoming CBW and echoed back in the CSW.
 * This allows the host to match responses to commands.
 * [ref/USB_MSC_Bulk_Only_Transport_v1.0.pdf: Section 5.1 - dCBWTag] */
volatile uint8_t  Udisk_CBW_Tag_Save[ 4 ];

/* SCSI sense data for REQUEST SENSE response after command failure.
 * Common combinations used in this firmware:
 *   Key=0x00, ASC=0x00: No Sense (success)
 *   Key=0x02, ASC=0x3A: Not Ready / Medium Not Present (SD not initialized)
 *   Key=0x03, ASC=0x0C: Medium Error / Write Error (eMMC write failed)
 *   Key=0x05, ASC=0x20: Illegal Request / Invalid Command
 * [ref/SCSI_Block_Commands_SBC3_r25.pdf: Sense Keys and ASC codes] */
volatile uint8_t  Udisk_Sense_Key = 0x00;
volatile uint8_t  Udisk_Sense_ASC = 0x00;

/* CSW status byte: 0=Command Passed, 1=Command Failed, 2=Phase Error.
 * [ref/USB_MSC_Bulk_Only_Transport_v1.0.pdf: Section 5.2 - bCSWStatus] */
volatile uint8_t  Udisk_CSW_Status = 0x00;

/* Data transfer tracking for READ10/WRITE10 commands.
 * UDISK_Transfer_DataLen:  Remaining bytes in current data phase.
 * UDISK_Cur_Sec_Lba:       Starting LBA (Logical Block Address) from CDB.
 * UDISK_Sec_Pack_Count:    Sectors transferred so far in current command.
 * UDISK_Pack_Size:         USB packet size (512 for USB2 HS, 1024 for USB3 SS).
 * [ref/SCSI_Block_Commands_SBC3_r25.pdf: READ(10)/WRITE(10) CDB layout] */
volatile uint32_t UDISK_Transfer_DataLen = 0x00;
volatile uint32_t UDISK_Cur_Sec_Lba = 0x00;
volatile uint16_t UDISK_Sec_Pack_Count = 0x00;
volatile uint16_t UDISK_Pack_Size = DEF_UDISK_PACK_512;

/* Flags set by USB ISR, consumed by main loop in UDISK_onePack_Deal().
 * This deferred-processing pattern avoids running lengthy eMMC DMA transfers
 * inside interrupt handlers, which would block other interrupts.
 *
 * UDISK_InPackflag=1:  Host requested READ10 data (set in UDISK_In_EP_Deal
 *                      or UDISK_Out_EP_Deal when CBW contains READ10).
 *                      Main loop calls UDISK_Up_OnePack() to read from SD
 *                      and send via USB.
 *
 * UDISK_OutPackflag=1: Host sent first WRITE10 data packet (set in
 *                      UDISK_Out_EP_Deal when data-OUT phase begins).
 *                      Main loop calls UDISK_Down_OnePack() to receive
 *                      remaining packets and write to SD. */
volatile uint8_t UDISK_OutPackflag = 0;
volatile uint8_t UDISK_InPackflag = 0;

/* Shared CBW/CSW/Sense buffer. Uses the BULK_ONLY_CMD union defined in
 * sw_udisk.h which overlays three structures on the same 31 bytes:
 *   mCBW     - Command Block Wrapper (received from host, 31 bytes)
 *   mCSW     - Command Status Wrapper (sent to host, 13 bytes)
 *   ReqSense - REQUEST SENSE response data (18 bytes)
 * [ref/USB_MSC_Bulk_Only_Transport_v1.0.pdf: Section 5] */
BULK_ONLY_CMD mBOC;

/* Pointer to data buffer for non-READ10 IN transfers (INQUIRY, CAPACITY, etc.).
 * Set by UDISK_SCSI_CMD_Deal() to point to the response table, then used by
 * UDISK_Bulk_UpData() to memcpy into the USB TX endpoint buffer. */
uint8_t   *pEndp2_Buf;


/*******************************************************************************
 * @fn      UDISK_CMD_Deal_Status
 *
 * @brief   Set SCSI sense data and CSW status for the current command.
 *          Called after processing each SCSI command to record the result.
 *          These values are used by:
 *            - UDISK_Up_CSW() to set CSW bCSWStatus byte
 *            - CMD_U_REQUEST_SENSE handler to fill sense response
 *
 * @param   key    - SCSI Sense Key (error category)
 * @param   asc    - Additional Sense Code (specific error)
 * @param   status - CSW status (0=pass, 1=fail, 2=phase error)
 *
 * [ref/SCSI_Block_Commands_SBC3_r25.pdf: Sense Keys]
 * [ref/USB_MSC_Bulk_Only_Transport_v1.0.pdf: Section 5.2 - bCSWStatus]
 ******************************************************************************/
void UDISK_CMD_Deal_Status( uint8_t key, uint8_t asc, uint8_t status )
{
    Udisk_Sense_Key  = key;
    Udisk_Sense_ASC  = asc;
    Udisk_CSW_Status = status;
}

/*******************************************************************************
 * @fn      UDISK_CMD_Deal_Fail
 *
 * @brief   Handle SCSI command failure by STALLing the appropriate bulk
 *          endpoint(s). The host detects the STALL and issues a clear-feature
 *          followed by REQUEST SENSE to retrieve error details.
 *
 *          If data-IN was pending (BLUCK_UP_FLAG), STALL EP1 IN.
 *          If data-OUT was pending (BLUCK_DOWN_FLAG), STALL EP1 OUT.
 *
 *          USB2: Set STALL via R8_UEP1_TX_CTRL / R8_UEP1_RX_CTRL registers.
 *          USB3: Set STALL via USB30_IN_set() / USB30_OUT_set() BSP functions.
 *
 * [ref/USB_MSC_Bulk_Only_Transport_v1.0.pdf: Section 6.6.1 - Error Handling]
 * [CH569DS1.PDF: USB2.0 Endpoint Control Registers]
 ******************************************************************************/
void UDISK_CMD_Deal_Fail( void )
{
    if( Udisk_Transfer_Status & DEF_UDISK_BLUCK_UP_FLAG )
    {
        /* STALL EP1 IN (bulk IN) - device refuses to send data */
        if( g_DeviceUsbType == USB_U20_SPEED )
        {
            R8_UEP1_TX_CTRL = (R8_UEP1_TX_CTRL & ~RB_UEP_TRES_MASK) | UEP_T_RES_STALL;
        }
        else
        {
            USB30_IN_set(ENDP_1, ENABLE, STALL, 0, 0);
        }
        Udisk_Transfer_Status &= ~DEF_UDISK_BLUCK_UP_FLAG;
    }
    if( Udisk_Transfer_Status & DEF_UDISK_BLUCK_DOWN_FLAG )
    {
        /* STALL EP1 OUT (bulk OUT) - device refuses to receive data */
        if( g_DeviceUsbType == USB_U20_SPEED )
        {
            R8_UEP1_RX_CTRL = (R8_UEP1_RX_CTRL & ~RB_UEP_RRES_MASK) | UEP_R_RES_STALL;
        }
        else
        {
            USB30_OUT_set(ENDP_1, STALL, 0);
        }
        Udisk_Transfer_Status &= ~DEF_UDISK_BLUCK_DOWN_FLAG;
    }
}

/*******************************************************************************
 * @fn      CMD_RD_WR_Deal_Pre
 *
 * @brief   Parse the SCSI CDB (Command Descriptor Block) for READ(10) or
 *          WRITE(10) commands. Extracts the starting LBA and transfer length.
 *
 *          READ(10) / WRITE(10) CDB layout (10 bytes):
 *            Byte 0:    Opcode (0x28 for READ, 0x2A for WRITE)
 *            Byte 1:    Flags (unused here)
 *            Bytes 2-5: Logical Block Address (big-endian, 32-bit)
 *            Byte 6:    Group Number (unused)
 *            Bytes 7-8: Transfer Length in sectors (big-endian, 16-bit)
 *            Byte 9:    Control
 *
 *          The CDB bytes are accessed from mBOC.mCBW.mCBW_CB_Buf[] which
 *          contains the 16-byte SCSI command block from the CBW.
 *
 * [ref/SCSI_Block_Commands_SBC3_r25.pdf: READ(10) and WRITE(10) commands]
 ******************************************************************************/
void CMD_RD_WR_Deal_Pre( void )
{
    /* Extract 32-bit LBA from CDB bytes 2-5 (big-endian) */
    UDISK_Cur_Sec_Lba = (uint32_t)mBOC.mCBW.mCBW_CB_Buf[ 2 ] << 24;
    UDISK_Cur_Sec_Lba = UDISK_Cur_Sec_Lba + ( (uint32_t)mBOC.mCBW.mCBW_CB_Buf[ 3 ] << 16 );
    UDISK_Cur_Sec_Lba = UDISK_Cur_Sec_Lba + ( (uint32_t)mBOC.mCBW.mCBW_CB_Buf[ 4 ] << 8 );
    UDISK_Cur_Sec_Lba = UDISK_Cur_Sec_Lba + ( (uint32_t)mBOC.mCBW.mCBW_CB_Buf[ 5 ] );

    /* Extract 16-bit transfer length (in sectors) from CDB bytes 7-8,
     * then convert to bytes by multiplying by sector size (512) */
    UDISK_Transfer_DataLen = ( (uint32_t)mBOC.mCBW.mCBW_CB_Buf[ 7 ] << 8 );
    UDISK_Transfer_DataLen = UDISK_Transfer_DataLen + ( (uint32_t)mBOC.mCBW.mCBW_CB_Buf[ 8 ] );
    UDISK_Transfer_DataLen = UDISK_Transfer_DataLen * DEF_UDISK_SECTOR_SIZE;

    UDISK_Sec_Pack_Count = 0x00;
    UDISK_CMD_Deal_Status( 0x00, 0x00, 0x00 );
}

/*******************************************************************************
 * @fn      UDISK_SCSI_CMD_Deal
 *
 * @brief   Main SCSI command dispatcher. Called when a complete CBW is received
 *          from the host. Validates the CBW signature ('USBC'), saves the tag,
 *          determines data phase direction, then dispatches to the appropriate
 *          command handler.
 *
 *          CBW validation [ref/USB_MSC_Bulk_Only_Transport_v1.0.pdf: 5.1]:
 *            - Signature must be 'USBC' (0x55534243)
 *            - Tag is saved for echoing in CSW
 *            - DataTransferLength determines if there's a data phase
 *            - Flags bit 7 determines direction (1=IN, 0=OUT)
 *
 *          Supported SCSI commands:
 *            0x00 TEST UNIT READY    - Check if SD card is ready
 *            0x03 REQUEST SENSE      - Return error details after failure
 *            0x12 INQUIRY            - Return device identification
 *            0x1A MODE SENSE(6)      - Return device parameters
 *            0x1B START STOP UNIT    - Eject/load (acknowledged, no action)
 *            0x1E PREVENT REMOVAL    - Prevent media removal (acknowledged)
 *            0x23 READ FORMAT CAP    - Return formatted capacity
 *            0x25 READ CAPACITY(10)  - Return last LBA and block size
 *            0x28 READ(10)           - Read sectors from SD card
 *            0x2A WRITE(10)          - Write sectors to SD card
 *            0x2E WRITE AND VERIFY   - Write with verify (treated as WRITE)
 *            0x2F VERIFY(10)         - Verify sectors (acknowledged)
 *            0x35 SYNCHRONIZE CACHE  - Flush cache (no-op, SD writes are sync)
 *            0x5A MODE SENSE(10)     - Return device parameters (long form)
 *
 * [ref/SCSI_Block_Commands_SBC3_r25.pdf: Command opcodes]
 * [ref/USB_MSC_Bulk_Only_Transport_v1.0.pdf: CBW processing]
 ******************************************************************************/
void UDISK_SCSI_CMD_Deal( void )
{
    uint8_t i;

    /* Validate CBW signature: must be 'USBC' (0x55534243)
     * [ref/USB_MSC_Bulk_Only_Transport_v1.0.pdf: Section 5.1 - dCBWSignature] */
    if( ( mBOC.mCBW.mCBW_Sig[ 0 ] == 'U' ) && ( mBOC.mCBW.mCBW_Sig[ 1 ] == 'S' )
      &&( mBOC.mCBW.mCBW_Sig[ 2 ] == 'B' ) && ( mBOC.mCBW.mCBW_Sig[ 3 ] == 'C' ) )
    {
        /* Save the CBW tag for echoing in the CSW response.
         * The host uses this to match CSW responses to CBW commands.
         * [ref/USB_MSC_Bulk_Only_Transport_v1.0.pdf: Section 5.1 - dCBWTag] */
        Udisk_CBW_Tag_Save[ 0 ] = mBOC.mCBW.mCBW_Tag[ 0 ];
        Udisk_CBW_Tag_Save[ 1 ] = mBOC.mCBW.mCBW_Tag[ 1 ];
        Udisk_CBW_Tag_Save[ 2 ] = mBOC.mCBW.mCBW_Tag[ 2 ];
        Udisk_CBW_Tag_Save[ 3 ] = mBOC.mCBW.mCBW_Tag[ 3 ];

        /* Extract data transfer length from CBW (little-endian 32-bit).
         * dCBWDataTransferLength: number of bytes host expects to transfer.
         * If zero, no data phase - go directly to CSW.
         * [ref/USB_MSC_Bulk_Only_Transport_v1.0.pdf: Section 5.1] */
        UDISK_Transfer_DataLen = ( uint32_t )mBOC.mCBW.mCBW_DataLen[ 3 ] << 24;
        UDISK_Transfer_DataLen += ( ( uint32_t )mBOC.mCBW.mCBW_DataLen[ 2 ] << 16 );
        UDISK_Transfer_DataLen += ( ( uint32_t )mBOC.mCBW.mCBW_DataLen[ 1 ] << 8 );
        UDISK_Transfer_DataLen += ( ( uint32_t )mBOC.mCBW.mCBW_DataLen[ 0 ] );

        /* Determine data phase direction from bmCBWFlags bit 7.
         *   1 = Data-IN  (device→host, e.g., READ10, INQUIRY)
         *   0 = Data-OUT (host→device, e.g., WRITE10)
         * [ref/USB_MSC_Bulk_Only_Transport_v1.0.pdf: Section 5.1 - bmCBWFlags] */
        if( UDISK_Transfer_DataLen )
        {
            if( mBOC.mCBW.mCBW_Flag & 0x80 )
            {
                Udisk_Transfer_Status |= DEF_UDISK_BLUCK_UP_FLAG;
            }
            else
            {
                Udisk_Transfer_Status |= DEF_UDISK_BLUCK_DOWN_FLAG;
            }
        }
        /* Always set CSW pending - every CBW requires a CSW response */
        Udisk_Transfer_Status |= DEF_UDISK_CSW_UP_FLAG;

        log_printf("CBW: cmd=0x%02X len=%d\r\n", mBOC.mCBW.mCBW_CB_Buf[ 0 ], UDISK_Transfer_DataLen);

        /* Dispatch based on SCSI opcode (byte 0 of the Command Block).
         * The Command Block is at CBW bytes 15-30, mapped to mCBW_CB_Buf[0..15].
         * [ref/SCSI_Block_Commands_SBC3_r25.pdf: Command opcodes] */
        switch( mBOC.mCBW.mCBW_CB_Buf[ 0 ] )
        {
            case  CMD_U_INQUIRY:
                /* INQUIRY (0x12): Return device identification data.
                 * Host uses this to identify vendor, product, and device type.
                 * Response capped at 0x24 (36) bytes per SCSI spec.
                 * Byte 0 is set to 0x00 (direct-access device, connected).
                 * [ref/SCSI_Block_Commands_SBC3_r25.pdf: INQUIRY command] */
                if( UDISK_Transfer_DataLen > 0x24 )
                {
                    UDISK_Transfer_DataLen = 0x24;
                }
                UDISK_Inquity_Tab[ 0 ] = 0x00;
                pEndp2_Buf = (uint8_t *)UDISK_Inquity_Tab;
                UDISK_CMD_Deal_Status( 0x00, 0x00, 0x00 );
                break;

            case  CMD_U_READ_FORMAT_CAPACITY:
                /* READ FORMAT CAPACITIES (0x23): Return formatted capacity.
                 * Windows sends this during device enumeration.
                 * If SD card is ready, return capacity from Udisk_Capability.
                 * If not ready, fail with Sense Key 0x02 (NOT READY),
                 * ASC 0x3A (MEDIUM NOT PRESENT).
                 * [ref/SCSI_Block_Commands_SBC3_r25.pdf: READ FORMAT CAPACITIES] */
                if( ( Udisk_Status & DEF_UDISK_EN_FLAG ) )
                {
                    if( UDISK_Transfer_DataLen > 0x0C )
                    {
                        UDISK_Transfer_DataLen = 0x0C;
                    }

                    /* Copy template then override sector count with actual value */
                    for( i = 0x00; i < UDISK_Transfer_DataLen; i++ )
                    {
                        mBOC.buf[ i ] = UDISK_Rd_Format_Capacity[ i ];
                    }
                    mBOC.buf[ 4 ] = ( ( Udisk_Capability >> 24 ) & 0xFF );
                    mBOC.buf[ 5 ] = ( ( Udisk_Capability >> 16 ) & 0xFF );
                    mBOC.buf[ 6 ] = ( ( Udisk_Capability >> 8  ) & 0xFF );
                    mBOC.buf[ 7 ] = ( ( Udisk_Capability       ) & 0xFF );
                    pEndp2_Buf = mBOC.buf;
                    UDISK_CMD_Deal_Status( 0x00, 0x00, 0x00 );
                }
                else
                {
                    /* SD card not initialized: NOT READY / MEDIUM NOT PRESENT */
                    UDISK_CMD_Deal_Status( 0x02, 0x3A, 0x01 );
                    UDISK_CMD_Deal_Fail( );
                }
                break;

            case  CMD_U_READ_CAPACITY:
                /* READ CAPACITY(10) (0x25): Return last LBA and block size.
                 * The host needs this to know the disk size. Returns:
                 *   Bytes 0-3: Last LBA = (Udisk_Capability - 1), big-endian
                 *   Bytes 4-7: Block size = 512, big-endian
                 * [ref/SCSI_Block_Commands_SBC3_r25.pdf: READ CAPACITY(10)] */
                if( ( Udisk_Status & DEF_UDISK_EN_FLAG ) )
                {
                    if( UDISK_Transfer_DataLen > 0x08 )
                    {
                        UDISK_Transfer_DataLen = 0x08;
                    }

                    /* Copy template then override last LBA with actual value */
                    for( i = 0x00; i < UDISK_Transfer_DataLen; i++ )
                    {
                        mBOC.buf[ i ] = UDISK_Rd_Capacity[ i ];
                    }
                    mBOC.buf[ 0 ] = ( ( Udisk_Capability - 1 ) >> 24 ) & 0xFF;
                    mBOC.buf[ 1 ] = ( ( Udisk_Capability - 1 ) >> 16 ) & 0xFF;
                    mBOC.buf[ 2 ] = ( ( Udisk_Capability - 1 ) >> 8  ) & 0xFF;
                    mBOC.buf[ 3 ] = ( ( Udisk_Capability - 1 )       ) & 0xFF;

                    pEndp2_Buf = mBOC.buf;
                    UDISK_CMD_Deal_Status( 0x00, 0x00, 0x00 );
                }
                else
                {
                    UDISK_CMD_Deal_Status( 0x02, 0x3A, 0x01 );
                    UDISK_CMD_Deal_Fail( );
                }
                break;

            case  CMD_U_READ10:
                /* READ(10) (0x28): Read sectors from SD card.
                 * CDB contains LBA and sector count (parsed by CMD_RD_WR_Deal_Pre).
                 * Actual data transfer is deferred to UDISK_Up_OnePack() in the
                 * main loop via UDISK_InPackflag.
                 * [ref/SCSI_Block_Commands_SBC3_r25.pdf: READ(10)] */
                if( ( Udisk_Status & DEF_UDISK_EN_FLAG ) )
                {
                    CMD_RD_WR_Deal_Pre( );
                }
                else
                {
                    UDISK_CMD_Deal_Status( 0x02, 0x3A, 0x01 );
                    UDISK_CMD_Deal_Fail( );
                }
                break;

            case  CMD_U_WR_VERIFY10:
                /* WRITE AND VERIFY(10) (0x2E): Write with verify.
                 * Treated the same as WRITE(10) - we don't do post-write verify.
                 * Falls through to WRITE10 handler. */
            case  CMD_U_WRITE10:
                /* WRITE(10) (0x2A): Write sectors to SD card.
                 * CDB contains LBA and sector count (parsed by CMD_RD_WR_Deal_Pre).
                 * Data-OUT phase is deferred to UDISK_Down_OnePack() in the
                 * main loop via UDISK_OutPackflag.
                 * [ref/SCSI_Block_Commands_SBC3_r25.pdf: WRITE(10)] */
                if( Udisk_Status & DEF_UDISK_EN_FLAG )
                {
                    CMD_RD_WR_Deal_Pre( );
                }
                else
                {
                    UDISK_CMD_Deal_Status( 0x02, 0x3A, 0x01 );
                    UDISK_CMD_Deal_Fail( );
                }
                break;

            case  CMD_U_MODE_SENSE:
                /* MODE SENSE(6) (0x1A): Return device mode parameters.
                 * Includes write-protect status (byte 2 = 0x00 = read-write)
                 * and block descriptor with capacity.
                 * [ref/SCSI_Block_Commands_SBC3_r25.pdf: MODE SENSE(6)] */
                if( ( Udisk_Status & DEF_UDISK_EN_FLAG ) )
                {
                    if( UDISK_Transfer_DataLen > 0x0C )
                    {
                        UDISK_Transfer_DataLen = 0x0C;
                    }
                    for( i = 0x00; i < UDISK_Transfer_DataLen; i++ )
                    {
                        mBOC.buf[ i ] = UDISK_Mode_Sense_1A[ i ];
                    }
                    /* Override capacity fields with actual SD card size */
                    mBOC.buf[ 4 ] = ( Udisk_Capability >> 24 ) & 0xFF;
                    mBOC.buf[ 5 ] = ( Udisk_Capability >> 16 ) & 0xFF;
                    mBOC.buf[ 6 ] = ( Udisk_Capability >> 8  ) & 0xFF;
                    mBOC.buf[ 7 ] = ( Udisk_Capability       ) & 0xFF;
                    pEndp2_Buf = mBOC.buf;
                }
                else
                {
                    UDISK_CMD_Deal_Status( 0x02, 0x3A, 0x01 );
                    UDISK_CMD_Deal_Fail( );
                }
                break;

            case  CMD_U_MODE_SENSE2:
                /* MODE SENSE(10) (0x5A): Return device mode parameters (long).
                 * Only supports page code 0x3F (return all pages).
                 * [ref/SCSI_Block_Commands_SBC3_r25.pdf: MODE SENSE(10)] */
                if( mBOC.mCBW.mCBW_CB_Buf[ 2 ] == 0x3F )
                {
                    if( UDISK_Transfer_DataLen > 0x10 )
                    {
                        UDISK_Transfer_DataLen = 0x10;
                    }

                    for( i = 0x00; i < UDISK_Transfer_DataLen; i++ )
                    {
                        mBOC.buf[ i ] = UDISK_Mode_Senese_5A[ i ];
                    }
                    /* Override capacity fields with actual SD card size */
                    mBOC.buf[ 8 ]  = ( Udisk_Capability >> 24 ) & 0xFF;
                    mBOC.buf[ 9 ]  = ( Udisk_Capability >> 16 ) & 0xFF;
                    mBOC.buf[ 10 ] = ( Udisk_Capability >> 8  ) & 0xFF;
                    mBOC.buf[ 11 ] = ( Udisk_Capability       ) & 0xFF;
                    pEndp2_Buf = mBOC.buf;
                }
                else
                {
                    /* Unsupported page code: ILLEGAL REQUEST / INVALID COMMAND */
                    UDISK_CMD_Deal_Status( 0x05, 0x20, 0x01 );
                    UDISK_CMD_Deal_Fail( );
                }
                break;

            case  CMD_U_REQUEST_SENSE:
                /* REQUEST SENSE (0x03): Return error details after a failure.
                 * Host sends this after receiving a CSW with status=1 (failed).
                 * We return fixed-format sense data (18 bytes):
                 *   ErrorCode=0x70: current errors, fixed format
                 *   SenseKey:       error category (from Udisk_Sense_Key)
                 *   SenseCode:      specific error (from Udisk_Sense_ASC)
                 * [ref/SCSI_Block_Commands_SBC3_r25.pdf: REQUEST SENSE] */
                mBOC.ReqSense.ErrorCode = 0x70;
                mBOC.ReqSense.Reserved1 = 0x00;
                mBOC.ReqSense.SenseKey  = Udisk_Sense_Key;
                mBOC.ReqSense.Information[ 0 ] = 0x00;
                mBOC.ReqSense.Information[ 1 ] = 0x00;
                mBOC.ReqSense.Information[ 2 ] = 0x00;
                mBOC.ReqSense.Information[ 3 ] = 0x00;
                mBOC.ReqSense.SenseLength = 0x0A;
                mBOC.ReqSense.Reserved2[ 0 ] = 0x00;
                mBOC.ReqSense.Reserved2[ 1 ] = 0x00;
                mBOC.ReqSense.Reserved2[ 2 ] = 0x00;
                mBOC.ReqSense.Reserved2[ 3 ] = 0x00;
                mBOC.ReqSense.SenseCode = Udisk_Sense_ASC;
                mBOC.ReqSense.SenseCodeQua = 0x00;
                mBOC.ReqSense.Reserved3[ 0 ] = 0x00;
                mBOC.ReqSense.Reserved3[ 1 ] = 0x00;
                mBOC.ReqSense.Reserved3[ 2 ] = 0x00;
                mBOC.ReqSense.Reserved3[ 3 ] = 0x00;
                pEndp2_Buf = mBOC.buf;
                Udisk_CSW_Status = 0x00;
                break;

            case  CMD_U_TEST_READY:
                /* TEST UNIT READY (0x00): Host polls if device is ready.
                 * Returns success if SD card is initialized (DEF_UDISK_EN_FLAG),
                 * otherwise returns NOT READY / MEDIUM NOT PRESENT.
                 * Linux/Windows poll this during mount and periodically.
                 * [ref/SCSI_Block_Commands_SBC3_r25.pdf: TEST UNIT READY] */
                if( Udisk_Status & DEF_UDISK_EN_FLAG )
                {
                    UDISK_CMD_Deal_Status( 0x00, 0x00, 0x00 );
                }
                else
                {
                    UDISK_CMD_Deal_Status( 0x02, 0x3A, 0x01 );
                    Udisk_Transfer_Status |= DEF_UDISK_BLUCK_UP_FLAG;
                    UDISK_CMD_Deal_Fail( );
                }
                break;

            case  CMD_U_PREVT_REMOVE:
                /* PREVENT ALLOW MEDIUM REMOVAL (0x1E): Prevent/allow eject.
                 * Acknowledged with success - we don't support locking.
                 * [ref/SCSI_Block_Commands_SBC3_r25.pdf: PREVENT ALLOW MEDIUM REMOVAL] */
                UDISK_CMD_Deal_Status( 0x00, 0x00, 0x00 );
                break;

            case  CMD_U_VERIFY10:
                /* VERIFY(10) (0x2F): Verify sectors on media.
                 * Acknowledged with success - no actual verification done.
                 * [ref/SCSI_Block_Commands_SBC3_r25.pdf: VERIFY(10)] */
                UDISK_CMD_Deal_Status( 0x00, 0x00, 0x00 );
                break;

            case  CMD_U_START_STOP:
                /* START STOP UNIT (0x1B): Start/stop the device or eject media.
                 * Acknowledged with success - SD card can't be ejected.
                 * [ref/SCSI_Block_Commands_SBC3_r25.pdf: START STOP UNIT] */
                UDISK_CMD_Deal_Status( 0x00, 0x00, 0x00 );
                break;

            case  CMD_U_SYNC_CACHE:
                /* SYNCHRONIZE CACHE(10) (0x35): Flush write cache to media.
                 * Acknowledged immediately - SD writes via BSP are synchronous,
                 * so there's nothing to flush.
                 * [ref/SCSI_Block_Commands_SBC3_r25.pdf: SYNCHRONIZE CACHE(10)] */
                UDISK_CMD_Deal_Status( 0x00, 0x00, 0x00 );
                break;

            default:
                /* Unsupported SCSI command - return ILLEGAL REQUEST (0x05)
                 * with ASC 0x20 (INVALID COMMAND OPERATION CODE).
                 * [ref/SCSI_Block_Commands_SBC3_r25.pdf: Sense Keys] */
                log_printf("SCSI: unsupported cmd 0x%02X\r\n", mBOC.mCBW.mCBW_CB_Buf[ 0 ]);
                UDISK_CMD_Deal_Status( 0x05, 0x20, 0x01 );
                Udisk_Transfer_Status |= DEF_UDISK_BLUCK_UP_FLAG;
                UDISK_CMD_Deal_Fail( );
                break;
        }
    }
    else
    {   /* CBW signature invalid - not a valid BOT command.
         * Return Phase Error (status=2) and STALL both endpoints.
         * [ref/USB_MSC_Bulk_Only_Transport_v1.0.pdf: Section 6.6.1] */
        UDISK_CMD_Deal_Status( 0x05, 0x20, 0x02 );
        Udisk_Transfer_Status |= DEF_UDISK_BLUCK_UP_FLAG;
        Udisk_Transfer_Status |= DEF_UDISK_BLUCK_DOWN_FLAG;
        UDISK_CMD_Deal_Fail(  );
    }
}

/*******************************************************************************
 * @fn      UDISK_In_EP_Deal
 *
 * @brief   Bulk IN endpoint completion handler. Called from USB ISR when an
 *          IN transfer on EP1 completes (host acknowledged our data).
 *
 *          Two cases:
 *          1. Data-IN phase pending (BLUCK_UP_FLAG):
 *             - If READ10: defer to main loop via UDISK_InPackflag
 *               (eMMC DMA is too slow for ISR context)
 *             - Otherwise: send next chunk via UDISK_Bulk_UpData()
 *               (INQUIRY, CAPACITY, etc. are small enough for ISR)
 *
 *          2. CSW pending (CSW_UP_FLAG):
 *             - Send CSW to complete the BOT transaction
 *
 * [ref/USB_MSC_Bulk_Only_Transport_v1.0.pdf: Command/Data/Status Protocol]
 ******************************************************************************/
void UDISK_In_EP_Deal( void )
{
    if( Udisk_Transfer_Status & DEF_UDISK_BLUCK_UP_FLAG )
    {
        if( mBOC.mCBW.mCBW_CB_Buf[ 0 ] == CMD_U_READ10 )
        {
            /* READ10 data phase: defer eMMC read to main loop.
             * The main loop (UDISK_onePack_Deal) will call UDISK_Up_OnePack()
             * which disables interrupts and runs a tight DMA+USB polling loop. */
            UDISK_InPackflag = 1;
        }
        else
        {
            /* Non-READ10 data (INQUIRY, CAPACITY, etc.): small enough to
             * handle directly. Send next packet of response data. */
            UDISK_Bulk_UpData( );
        }
    }
    else if( Udisk_Transfer_Status & DEF_UDISK_CSW_UP_FLAG )
    {
        /* All data sent, now send the CSW to complete the transaction */
        UDISK_Up_CSW( );
    }
}

/*******************************************************************************
 * @fn      UDISK_Out_EP_Deal
 *
 * @brief   Bulk OUT endpoint handler. Called from USB ISR when data arrives
 *          on EP1 OUT. Handles two scenarios:
 *
 *          1. Data-OUT phase in progress (WRITE10 data arriving):
 *             Set UDISK_OutPackflag to defer processing to main loop.
 *             The main loop calls UDISK_Down_OnePack() for the eMMC write.
 *
 *          2. New CBW arriving (packlen == 31 bytes):
 *             Copy the 31-byte CBW into mBOC union, then call
 *             UDISK_SCSI_CMD_Deal() to parse and dispatch the command.
 *             If the command has a data-IN phase (e.g., READ10), start it.
 *             If the command has no data phase, send CSW immediately.
 *
 * @param   pbuf    - pointer to received data (typically endp1Rbuff)
 * @param   packlen - number of bytes received
 *
 * [ref/USB_MSC_Bulk_Only_Transport_v1.0.pdf: Section 5.1 - CBW]
 * The CBW is exactly 31 (0x1F) bytes. We detect it by checking packlen.
 ******************************************************************************/
void UDISK_Out_EP_Deal( uint8_t *pbuf, uint16_t packlen )
{
    uint32_t i;

    if( Udisk_Transfer_Status & DEF_UDISK_BLUCK_DOWN_FLAG )
    {
        /* Data-OUT phase of WRITE command - first data packet arrived.
         * Defer to main loop for bulk eMMC write operation. */
        UDISK_OutPackflag = 1;
    }
    else
    {
        /* Check if this is a CBW (exactly 31 bytes).
         * CBW is always 31 bytes: 4 (sig) + 4 (tag) + 4 (len) +
         * 1 (flags) + 1 (LUN) + 1 (CB len) + 16 (CB) = 31
         * [ref/USB_MSC_Bulk_Only_Transport_v1.0.pdf: Section 5.1] */
        if( packlen == 0x1F )
        {
            /* Copy CBW data into mBOC union for parsing */
            for( i = 0; i < packlen; i++ )
            {
                mBOC.buf[ i ] = *pbuf++;
            }

            /* Parse the SCSI command from the CBW */
            UDISK_SCSI_CMD_Deal( );

            /* After parsing, determine what to do next:
             * - If data-OUT pending: wait for data (handled by OutPackflag)
             * - If data-IN pending: start sending data
             * - If no data phase: send CSW immediately */
            if( ( Udisk_Transfer_Status & DEF_UDISK_BLUCK_DOWN_FLAG ) == 0x00 )
            {
                if( Udisk_Transfer_Status & DEF_UDISK_BLUCK_UP_FLAG )
                {
                    if( mBOC.mCBW.mCBW_CB_Buf[ 0 ] == CMD_U_READ10 )
                    {
                        /* READ10: defer to main loop for eMMC DMA */
                        UDISK_InPackflag = 1;
                    }
                    else
                    {
                        /* Small response (INQUIRY etc.): send now from ISR */
                        UDISK_Bulk_UpData( );
                    }
                }
                else if( Udisk_CSW_Status == 0x00 )
                {
                    /* No data phase, command succeeded: send CSW */
                    UDISK_Up_CSW( );
                }
            }
        }
    }
}

/*******************************************************************************
 * @fn      UDISK_Bulk_UpData
 *
 * @brief   Send bulk data to host via EP1 IN. Used for small SCSI responses
 *          (INQUIRY, READ CAPACITY, MODE SENSE, REQUEST SENSE, etc.).
 *          NOT used for READ10 data - that goes through UDISK_Up_OnePack().
 *
 *          Sends up to UDISK_Pack_Size (512) bytes per call. If the response
 *          is larger, it will be called again from UDISK_In_EP_Deal() when
 *          the host ACKs the previous packet.
 *
 *          USB2: Copy data to endp1Tbuff, set DMA address and length, ACK.
 *          USB3: Copy data to endp1Tbuff, set DMA, configure endpoint, ERDY.
 *                ERDY tells the host the device has data ready to send.
 *                [USB 3.0 Spec: ERDY Transaction Packet]
 *
 * [CH569DS1.PDF: USB2.0/USB3.0 Endpoint TX Control Registers]
 ******************************************************************************/
void UDISK_Bulk_UpData( void )
{
    uint32_t  len;

    if( UDISK_Transfer_DataLen > UDISK_Pack_Size )
    {
        len = UDISK_Pack_Size;
        UDISK_Transfer_DataLen -= UDISK_Pack_Size;
    }
    else
    {
        len = UDISK_Transfer_DataLen;
        UDISK_Transfer_DataLen = 0x00;
        Udisk_Transfer_Status &= ~DEF_UDISK_BLUCK_UP_FLAG;
    }

    if( g_DeviceUsbType == USB_U20_SPEED )
    {
        /* USB2 High-Speed path:
         * Copy response data to endpoint TX buffer, set DMA address,
         * set transfer length, then ACK to allow the host to IN the data.
         * [CH569DS1.PDF: R32_UEP1_TX_DMA, R16_UEP1_T_LEN, R8_UEP1_TX_CTRL] */
        memcpy(endp1Tbuff, pEndp2_Buf, len);
        R16_UEP1_T_LEN = len;
        R32_UEP1_TX_DMA = (uint32_t)(uint8_t *)endp1Tbuff;
        R8_UEP1_TX_CTRL = (R8_UEP1_TX_CTRL & ~RB_UEP_TRES_MASK) | UEP_T_RES_ACK;
    }
    else
    {
        /* USB3 SuperSpeed path:
         * Copy data to TX buffer, configure endpoint for IN transfer with
         * burst level from DEF_ENDP1_IN_BURST_LEVEL, then send ERDY to
         * notify host that data is available.
         * [CH569DS1.PDF: USBSS Endpoint registers]
         * [USB 3.0 Spec: ERDY flow control] */
        memcpy(endp1Tbuff, pEndp2_Buf, len);
        USBSS->UEP1_TX_DMA = (uint32_t)(uint8_t *)endp1Tbuff;
        USB30_IN_set(ENDP_1, ENABLE, ACK, DEF_ENDP1_IN_BURST_LEVEL, len);
        USB30_send_ERDY(ENDP_1 | IN, DEF_ENDP1_IN_BURST_LEVEL);
    }
}

/*******************************************************************************
 * @fn      UDISK_Up_CSW
 *
 * @brief   Construct and send the 13-byte Command Status Wrapper (CSW) on
 *          EP1 IN. This completes the BOT transaction.
 *
 *          CSW layout (13 bytes):
 *            Bytes 0-3:  Signature 'USBS' (0x55534253)
 *            Bytes 4-7:  Tag (copied from CBW, must match)
 *            Bytes 8-11: Data Residue (untransferred bytes, set to 0)
 *            Byte 12:    Status (0=Passed, 1=Failed, 2=Phase Error)
 *
 * [ref/USB_MSC_Bulk_Only_Transport_v1.0.pdf: Section 5.2 - CSW]
 ******************************************************************************/
void UDISK_Up_CSW( void )
{
    Udisk_Transfer_Status = 0x00;
    log_printf("CSW: sta=%d\r\n", Udisk_CSW_Status);

    /* Build CSW in mBOC union (overlays the same buffer as CBW) */
    mBOC.mCSW.mCSW_Sig[ 0 ] = 'U';
    mBOC.mCSW.mCSW_Sig[ 1 ] = 'S';
    mBOC.mCSW.mCSW_Sig[ 2 ] = 'B';
    mBOC.mCSW.mCSW_Sig[ 3 ] = 'S';
    mBOC.mCSW.mCSW_Tag[ 0 ] = Udisk_CBW_Tag_Save[ 0 ];
    mBOC.mCSW.mCSW_Tag[ 1 ] = Udisk_CBW_Tag_Save[ 1 ];
    mBOC.mCSW.mCSW_Tag[ 2 ] = Udisk_CBW_Tag_Save[ 2 ];
    mBOC.mCSW.mCSW_Tag[ 3 ] = Udisk_CBW_Tag_Save[ 3 ];
    mBOC.mCSW.mCSW_Residue[ 0 ] = 0x00;
    mBOC.mCSW.mCSW_Residue[ 1 ] = 0x00;
    mBOC.mCSW.mCSW_Residue[ 2 ] = 0x00;
    mBOC.mCSW.mCSW_Residue[ 3 ] = 0x00;
    mBOC.mCSW.mCSW_Status = Udisk_CSW_Status;

    /* Send 13-byte CSW via EP1 IN */
    if( g_DeviceUsbType == USB_U20_SPEED )
    {
        memcpy(endp1Tbuff, (uint8_t *)mBOC.buf, 0x0D);
        R16_UEP1_T_LEN = 0x0D;
        R32_UEP1_TX_DMA = (uint32_t)(uint8_t *)endp1Tbuff;
        R8_UEP1_TX_CTRL = (R8_UEP1_TX_CTRL & ~RB_UEP_TRES_MASK) | UEP_T_RES_ACK;
    }
    else
    {
        memcpy(endp1Tbuff, (uint8_t *)mBOC.buf, 0x0D);
        USBSS->UEP1_TX_DMA = (uint32_t)(uint8_t *)endp1Tbuff;
        USB30_IN_set(ENDP_1, ENABLE, ACK, DEF_ENDP1_IN_BURST_LEVEL, 0x0D);
        USB30_send_ERDY(ENDP_1 | IN, DEF_ENDP1_IN_BURST_LEVEL);
    }
}

/*******************************************************************************
 * @fn      UDISK_onePack_Deal
 *
 * @brief   Main loop handler for deferred READ10/WRITE10 data transfers.
 *          Called continuously from main() while(1) loop.
 *
 *          USB ISR handlers (UDISK_In_EP_Deal / UDISK_Out_EP_Deal) set flags
 *          to defer large data transfers to the main loop. This avoids running
 *          lengthy eMMC DMA operations inside interrupt context, which would
 *          block other interrupts and cause USB timeouts.
 *
 *          UDISK_InPackflag=1:  Host wants to read data → UDISK_Up_OnePack()
 *          UDISK_OutPackflag=1: Host sent write data   → UDISK_Down_OnePack()
 *
 *          After a write completes, re-enable EP1 OUT to accept the next CBW.
 *          USB2: Set ACK response on RX control register.
 *          USB3: Set ACK + burst level and send ERDY.
 *
 * [ref/USB_MSC_Bulk_Only_Transport_v1.0.pdf: Command/Data/Status flow]
 ******************************************************************************/
void UDISK_onePack_Deal( void )
{
    if( UDISK_InPackflag == 1 )
    {
        UDISK_InPackflag = 0;
        UDISK_Up_OnePack();
    }

    if( UDISK_OutPackflag == 1 )
    {
        UDISK_OutPackflag = 0;
        UDISK_Down_OnePack();

        /* Re-enable EP1 OUT to accept the next CBW from host */
        if( g_DeviceUsbType == USB_U20_SPEED )
        {
            R8_UEP1_RX_CTRL = (R8_UEP1_RX_CTRL & ~RB_UEP_RRES_MASK) | UEP_R_RES_ACK;
        }
        else
        {
            USB30_OUT_set(ENDP_1, ACK, DEF_ENDP1_OUT_BURST_LEVEL);
            USB30_send_ERDY(ENDP_1 | OUT, DEF_ENDP1_OUT_BURST_LEVEL);
        }
    }
}

/*******************************************************************************
 * @fn      UDISK_Up_OnePack
 *
 * @brief   READ10 data transfer: read sectors from SD card via eMMC DMA and
 *          simultaneously send them to the host via USB.
 *
 *          This implements a streaming circular buffer approach:
 *            - eMMC DMA reads sectors into UDisk_In_Buf (40KB circular)
 *            - USB sends sectors from UDisk_In_Buf to host
 *            - Both run concurrently for maximum throughput
 *
 *          eMMC AUTOGAPSTOP mechanism:
 *            TRAN_MODE bits: AUTOGAPSTOP (bit4) + GAP_STOP (bit1)
 *            The eMMC controller pauses DMA after each block, signaled by
 *            the BKGAP interrupt flag. We release the pause by clearing
 *            GAP_STOP: R32_EMMC_TRAN_MODE = (1<<4) [AUTOGAPSTOP only].
 *            If the USB consumer falls behind (buffer nearly full), we hold
 *            the gap stop (lock=1) until USB catches up, preventing overflow.
 *            [CH569DS1.PDF: R32_EMMC_TRAN_MODE - AUTOGAPSTOP, GAP_STOP bits]
 *
 *          Transfer termination:
 *            When TRANDONE fires, all requested blocks have been read.
 *            We drain remaining USB sends, then issue CMD12 (STOP_TRANSMISSION)
 *            to end the multi-block read.
 *            [ref/SD_Physical_Layer_Spec_v6.00.pdf: CMD12, CMD18]
 *
 *          Interrupts are disabled during the transfer for deterministic timing:
 *            - USB IRQ disabled to prevent ISR from interfering with polling
 *            - EMMC IRQ disabled to prevent ISR from clearing transfer flags
 *            - EP0 NAK'd to prevent control transfers during data phase
 *
 * @note    USB2 path sends 512 bytes per packet (1 sector).
 *          USB3 path sends 1024 bytes per burst (2 sectors) via ERDY flow.
 *
 * [ref/SCSI_Block_Commands_SBC3_r25.pdf: READ(10)]
 * [ref/SD_Physical_Layer_Spec_v6.00.pdf: CMD18 READ_MULTIPLE_BLOCK]
 * [CH569DS1.PDF: EMMC DMA and interrupt registers]
 ******************************************************************************/
void UDISK_Up_OnePack( void )
{
    uint16_t preqnum;
    uint8_t s;
    uint32_t cmd_arg_val;
    uint16_t cmd_set_val;
    uint16_t sdtran = 0, usbtran = 0;  /* sector counters: SD read vs USB sent */
    uint8_t sdstep = 0, usbstep = 0;   /* circular buffer position (in sectors) */
    uint8_t lock = 0, flag = 1;         /* lock=1: hold gap stop; flag=1: first iter */

    preqnum = UDISK_Transfer_DataLen / 512;
    UDISK_Transfer_DataLen = 0;

    cprintf("R lba=%lu n=%u\r\n", UDISK_Cur_Sec_Lba, preqnum);

    if( g_DeviceUsbType == USB_U20_SPEED )
    {
        /* ── USB2 High-Speed READ path ──
         * Disable USB HS interrupt to prevent ISR interference.
         * Save and NAK EP0 to block control transfers during data phase.
         * Enable auto-toggle for DATA0/DATA1 PID alternation in polling loop.
         * [CH569DS1.PDF: USB2.0 Endpoint Control Registers] */
        uint8_t uep0rxsave = R8_UEP0_RX_CTRL;
        uint8_t uep0txsave = R8_UEP0_TX_CTRL;
        PFIC_DisableIRQ(USBHS_IRQn);
        R8_UEP0_TX_CTRL = UEP_T_RES_NAK;
        R8_UEP0_RX_CTRL = UEP_R_RES_NAK;

        /* Enable hardware auto-toggle: automatically alternates DATA0/DATA1
         * PID for each packet. Required when polling (ISR normally handles this).
         * [CH569DS1.PDF: RB_UEP_T_AUTOTOG] */
        R8_UEP1_TX_CTRL |= RB_UEP_T_AUTOTOG;

        /* Diagnostic: verify eMMC and USB buffer integrity during read.
         * Checks first uint32 of each sector (should equal LBA with test pattern). */
        uint32_t start_lba = UDISK_Cur_Sec_Lba;
        uint16_t sd_err = 0, usb_err = 0;
        uint32_t sd_first_exp = 0, sd_first_got = 0;
        uint32_t usb_first_exp = 0, usb_first_got = 0;

        /* Start eMMC multi-block read (CMD18 READ_MULTIPLE_BLOCK).
         * DMA starts at UDisk_In_Buf with AUTOGAPSTOP+GAP_STOP enabled.
         * The controller will pause after each block, giving us time to
         * update the DMA address for the next circular buffer position.
         * [ref/SD_Physical_Layer_Spec_v6.00.pdf: CMD18]
         * [CH569DS1.PDF: R32_EMMC_TRAN_MODE, R32_EMMC_BLOCK_CFG] */
        PFIC_DisableIRQ(EMMC_IRQn);
        R16_EMMC_INT_FG = 0xffff;
        R32_EMMC_DMA_BEG1 = (uint32_t)UDisk_In_Buf;
        R32_EMMC_TRAN_MODE = (1<<4)|(1<<1); /* AUTOGAPSTOP | GAP_STOP */
        R32_EMMC_BLOCK_CFG = (512)<<16 | preqnum;

        cmd_arg_val = UDISK_Cur_Sec_Lba;
        cmd_set_val = RB_EMMC_CKIDX | RB_EMMC_CKCRC | RESP_TYPE_48 | EMMC_CMD18;
        EMMCSendCmd(cmd_arg_val, cmd_set_val);

        /* ── Simultaneous eMMC read + USB2 send loop ──
         * Two interleaved state machines running in one loop:
         *   1. USB TX: poll RB_USB_IF_TRANSFER to detect IN-ACK from host,
         *      then queue the next sector for transmission.
         *   2. eMMC DMA: poll BKGAP flag for each completed block,
         *      advance circular buffer pointer, release gap stop.
         * The loop exits when TRANDONE fires (all blocks read by eMMC).
         *
         * CRITICAL: NAK EP1 TX immediately when TRANSFER fires.
         * Without this, if eMMC guard isn't ready (sdtran too low),
         * the endpoint stays armed and the host's next IN token gets
         * an auto-response from the OLD DMA address (= shift -1). */
        uint16_t xfer_count = 0;
        while( 1 )
        {
            /* Immediately NAK EP1 TX when any transfer completes.
             * This closes the race window where the host could send
             * an IN token before we update the DMA address. */
            if( R8_USB_INT_FG & RB_USB_IF_TRANSFER )
            {
                R8_UEP1_TX_CTRL = (R8_UEP1_TX_CTRL & ~RB_UEP_TRES_MASK) | UEP_T_RES_NAK;
            }

            /* USB TX side: set up next sector when eMMC has data ready.
             * flag=1 on first iteration (no previous ACK to wait for). */
            if( ( sdtran > 1 && usbtran < sdtran - 1 ) && ( (R8_USB_INT_FG & RB_USB_IF_TRANSFER) || flag ) )
            {
                if( R8_USB_INT_FG & RB_USB_IF_TRANSFER ) xfer_count++;
                R8_USB_INT_FG = RB_USB_IF_TRANSFER;
                flag = 0;
                /* Diag: verify buffer contains expected sector before sending */
                {
                    uint32_t exp = start_lba + usbtran;
                    uint32_t got = *(volatile uint32_t*)(UDisk_In_Buf + usbstep * 512);
                    if (got != exp) { if(!usb_err) { usb_first_exp=exp; usb_first_got=got; } usb_err++; }
                }
                R32_UEP1_TX_DMA = (uint32_t)(uint8_t *)(UDisk_In_Buf + usbstep * 512);
                R16_UEP1_T_LEN = 512;
                __asm__ volatile("fence" ::: "memory");
                R8_UEP1_TX_CTRL = (R8_UEP1_TX_CTRL & ~RB_UEP_TRES_MASK) | UEP_T_RES_ACK;

                usbtran++;
                usbstep++;
                if( usbstep == UDISKSIZE/512 ) usbstep = 0; /* wrap circular buffer */
                if( lock )
                {
                    lock = 0;
                    __asm__ volatile("fence" ::: "memory");
                    R32_EMMC_TRAN_MODE = (uint32_t)(1<<4);
                }
            }

            /* Dismiss spurious SETUP activity during data transfer */
            if( R8_USB_INT_FG & RB_USB_IF_SETUOACT )
            {
                R8_USB_INT_FG = RB_USB_IF_SETUOACT;
            }

            /* eMMC DMA side: check for completed block (BKGAP) or
             * completed transfer (TRANDONE) */
            if( R16_EMMC_INT_FG & RB_EMMC_IF_BKGAP )
            {
                /* Diag: verify eMMC delivered correct sector before advancing */
                {
                    uint32_t exp = start_lba + sdtran;
                    uint32_t got = *(volatile uint32_t*)(UDisk_In_Buf + sdstep * 512);
                    if (got != exp) { if(!sd_err) { sd_first_exp=exp; sd_first_got=got; } sd_err++; }
                }
                R16_EMMC_INT_FG = RB_EMMC_IF_BKGAP;
                sdtran++;
                sdstep++;
                if( sdstep == UDISKSIZE/512 ) sdstep = 0; /* wrap */
                R32_EMMC_DMA_BEG1 = (uint32_t)(uint8_t *)(UDisk_In_Buf + sdstep*512);

                if( (sdtran-usbtran) < ((UDISKSIZE/512)-2) )
                {
                    __asm__ volatile("fence" ::: "memory");
                    R32_EMMC_TRAN_MODE = (uint32_t)(1<<4); /* release gap stop */
                }
                else
                    lock = 1; /* buffer nearly full, hold gap stop */
            }
            else if( R16_EMMC_INT_FG & RB_EMMC_IF_TRANDONE )
            {
                /* Diag: verify last eMMC sector */
                {
                    uint32_t exp = start_lba + sdtran;
                    uint32_t got = *(volatile uint32_t*)(UDisk_In_Buf + sdstep * 512);
                    if (got != exp) { if(!sd_err) { sd_first_exp=exp; sd_first_got=got; } sd_err++; }
                }
                R16_EMMC_INT_FG = RB_EMMC_IF_TRANDONE | RB_EMMC_IF_CMDDONE;
                sdtran++;
                sdstep++;
                break;
            }
        }

        /* ── Drain remaining USB sends ──
         * eMMC read is complete (sdtran sectors read), but USB may still
         * have unsent sectors in the buffer. Send them all. */
        while( 1 )
        {
            /* Immediately NAK on transfer complete (same as main loop) */
            if( R8_USB_INT_FG & RB_USB_IF_TRANSFER )
            {
                R8_UEP1_TX_CTRL = (R8_UEP1_TX_CTRL & ~RB_UEP_TRES_MASK) | UEP_T_RES_NAK;
            }

            if( (R8_USB_INT_FG & RB_USB_IF_TRANSFER) || flag )
            {
                xfer_count++;
                R8_USB_INT_FG = RB_USB_IF_TRANSFER;
                flag = 0;
                /* Diag: verify buffer contains expected sector before sending */
                {
                    uint32_t exp = start_lba + usbtran;
                    uint32_t got = *(volatile uint32_t*)(UDisk_In_Buf + usbstep * 512);
                    if (got != exp) { if(!usb_err) { usb_first_exp=exp; usb_first_got=got; } usb_err++; }
                }
                R32_UEP1_TX_DMA = (uint32_t)(uint8_t *)(UDisk_In_Buf + usbstep * 512);
                R16_UEP1_T_LEN = 512;
                __asm__ volatile("fence" ::: "memory");
                R8_UEP1_TX_CTRL = (R8_UEP1_TX_CTRL & ~RB_UEP_TRES_MASK) | UEP_T_RES_ACK;

                usbtran++;
                usbstep++;
                if( usbstep == UDISKSIZE/512 ) usbstep = 0;
                if( usbtran == sdtran )
                {
                    /* All sectors sent - wait for final ACK then NAK */
                    while( !(R8_USB_INT_FG & RB_USB_IF_TRANSFER) );
                    R8_USB_INT_FG = RB_USB_IF_TRANSFER;
                    R8_UEP1_TX_CTRL = (R8_UEP1_TX_CTRL & ~RB_UEP_TRES_MASK) | UEP_T_RES_NAK;
                    break;
                }
            }

            if( R8_USB_INT_FG & RB_USB_IF_SETUOACT )
            {
                R8_USB_INT_FG = RB_USB_IF_SETUOACT;
            }
        }

        /* ── Stop eMMC multi-block transfer (CMD12 STOP_TRANSMISSION) ──
         * Required after CMD18 to end the multi-block read operation.
         * Uses R1b response type (48-bit with busy signaling on DAT0).
         * [ref/SD_Physical_Layer_Spec_v6.00.pdf: CMD12] */
        R32_EMMC_TRAN_MODE = 0;
        cmd_arg_val = 0;
        cmd_set_val = RB_EMMC_CKIDX | RB_EMMC_CKCRC | RESP_TYPE_R1b | EMMC_CMD12;
        EMMCSendCmd(cmd_arg_val, cmd_set_val);
        while(1)
        {
            s = CheckCMDComp(&TF_EMMCParam);
            if( s != CMD_NULL ) break;
        }
        R16_EMMC_INT_FG = 0xffff;
        TF_EMMCParam.EMMCOpErr = 0;
        PFIC_EnableIRQ(EMMC_IRQn);

        /* Print read-path diagnostics:
         *   sd_err:  sectors where eMMC DMA delivered wrong data
         *   usb_err: sectors where buffer had wrong data at USB TX time
         *   xfer:    TRANSFER flags seen (should == usbtran-1 if no auto-sends)
         * If sd_err=0 and usb_err=0 but host sees corruption → USB HW race */
        if (sd_err || usb_err)
            cprintf("R ERR sd=%u usb=%u sdE=%08lx/%08lx usbE=%08lx/%08lx\r\n",
                    sd_err, usb_err, sd_first_exp, sd_first_got, usb_first_exp, usb_first_got);
        else
            cprintf("R OK n=%u xfer=%u usbtran=%u\r\n", preqnum, xfer_count, usbtran);

        /* Disable auto-toggle before re-enabling USB ISR.
         * The ISR manages toggle manually; leaving auto-toggle on would
         * cause DATA PID mismatches. */
        R8_UEP1_TX_CTRL &= ~RB_UEP_T_AUTOTOG;

        PFIC_EnableIRQ(USBHS_IRQn);
        R8_UEP0_TX_CTRL = uep0txsave;
        R8_UEP0_RX_CTRL = uep0rxsave;
    }
    else
    {
        /* ── USB3 SuperSpeed READ path ──
         * Same streaming circular buffer approach as USB2 but:
         *   - Sends 1024 bytes (2 sectors) per burst via ERDY flow control
         *   - Polls bit 31 of USBSS->UEP1_TX_CTRL for TX completion
         *   - Uses USB30_IN_set() / USB30_send_ERDY() instead of register writes
         * [USB 3.0 Spec: ERDY Transaction Packet, Burst Transactions] */
        PFIC_DisableIRQ(USBSS_IRQn);

        /* Start eMMC multi-block read (CMD18) - same as USB2 path */
        PFIC_DisableIRQ(EMMC_IRQn);
        R16_EMMC_INT_FG = 0xffff;
        R32_EMMC_DMA_BEG1 = (uint32_t)UDisk_In_Buf;
        R32_EMMC_TRAN_MODE = (1<<4)|(1<<1); /* AUTOGAPSTOP | GAP_STOP */
        R32_EMMC_BLOCK_CFG = (512)<<16 | preqnum;

        cmd_arg_val = UDISK_Cur_Sec_Lba;
        cmd_set_val = RB_EMMC_CKIDX | RB_EMMC_CKCRC | RESP_TYPE_48 | EMMC_CMD18;
        EMMCSendCmd(cmd_arg_val, cmd_set_val);

        /* Simultaneous eMMC read + USB3 send loop.
         * USB3 sends 2 sectors (1024 bytes) per ERDY burst. */
        while( 1 )
        {
            /* USB3 TX: check bit 31 of UEP1_TX_CTRL for completion */
            if( ( sdtran > 1 && usbtran < sdtran - 1 ) && ( (USBSS->UEP1_TX_CTRL & ((uint32_t)1<<31)) || flag ) )
            {
                USB30_IN_clearIT(ENDP_1);
                USBSS->UEP1_TX_DMA = (uint32_t)(uint8_t *)(UDisk_In_Buf + usbstep * 512);
                USB30_IN_set(ENDP_1, ENABLE, ACK, 1, 1024);
                USB30_send_ERDY(ENDP_1 | IN, 1);
                usbtran += 2;  /* 2 sectors per burst */
                usbstep += 2;
                if( usbstep == UDISKSIZE/512 ) usbstep = 0;
                flag = 0;
                if( lock )
                {
                    lock = 0;
                    R32_EMMC_TRAN_MODE = (uint32_t)(1<<4);
                }
            }

            /* eMMC DMA: same BKGAP/TRANDONE handling as USB2 path */
            if( R16_EMMC_INT_FG & RB_EMMC_IF_BKGAP )
            {
                R16_EMMC_INT_FG = RB_EMMC_IF_BKGAP;
                sdtran++;
                sdstep++;
                if( sdstep == UDISKSIZE/512 ) sdstep = 0;
                R32_EMMC_DMA_BEG1 = (uint32_t)(uint8_t *)(UDisk_In_Buf + sdstep*512);
                if( (sdtran-usbtran) < ((UDISKSIZE/512)-2) )
                    R32_EMMC_TRAN_MODE = (uint32_t)(1<<4);
                else
                    lock = 1;
            }
            else if( R16_EMMC_INT_FG & RB_EMMC_IF_TRANDONE )
            {
                R16_EMMC_INT_FG = RB_EMMC_IF_TRANDONE | RB_EMMC_IF_CMDDONE;
                sdtran++;
                sdstep++;
                break;
            }
        }

        /* Drain remaining USB3 sends.
         * Handle last burst specially: if only 1 sector remains, send 512
         * instead of 1024 bytes. */
        while( 1 )
        {
            if( (USBSS->UEP1_TX_CTRL & ((uint32_t)1<<31)) || flag )
            {
                flag = 0;
                USB30_IN_clearIT(ENDP_1);
                if( (sdtran - usbtran) > 1 )
                {
                    /* 2+ sectors remain: send full 1024-byte burst */
                    USBSS->UEP1_TX_DMA = (uint32_t)(uint8_t *)(UDisk_In_Buf + usbstep * 512);
                    USB30_IN_set(ENDP_1, ENABLE, ACK, 1, 1024);
                    USB30_send_ERDY(ENDP_1 | IN, 1);
                    usbtran += 2;
                    usbstep += 2;
                }
                else
                {
                    /* 1 sector remains: send 512-byte short burst */
                    USBSS->UEP1_TX_DMA = (uint32_t)(uint8_t *)(UDisk_In_Buf + usbstep * 512);
                    USB30_IN_set(ENDP_1, ENABLE, ACK, 1, 512);
                    USB30_send_ERDY(ENDP_1 | IN, 1);
                    usbtran++;
                    usbstep++;
                }
                if( usbstep == UDISKSIZE/512 ) usbstep = 0;
                if( usbtran == sdtran )
                {
                    /* All sectors sent - wait for final completion */
                    while( !(USBSS->UEP1_TX_CTRL & ((uint32_t)1<<31)) );
                    USB30_IN_clearIT(ENDP_1);
                    break;
                }
            }
        }

        /* Stop eMMC transfer (CMD12) - same as USB2 path */
        R32_EMMC_TRAN_MODE = 0;
        cmd_arg_val = 0;
        cmd_set_val = RB_EMMC_CKIDX | RB_EMMC_CKCRC | RESP_TYPE_R1b | EMMC_CMD12;
        EMMCSendCmd(cmd_arg_val, cmd_set_val);
        while(1)
        {
            s = CheckCMDComp(&TF_EMMCParam);
            if( s != CMD_NULL ) break;
        }
        R16_EMMC_INT_FG = 0xffff;
        TF_EMMCParam.EMMCOpErr = 0;
        PFIC_EnableIRQ(EMMC_IRQn);

        PFIC_EnableIRQ(USBSS_IRQn);
    }

    UDISK_Cur_Sec_Lba += preqnum;

    if( UDISK_Transfer_DataLen == 0 )
    {
        UDISK_Up_CSW( );
    }

    if( UDISK_Transfer_DataLen == 0x00 )
    {
        Udisk_Transfer_Status &= ~DEF_UDISK_BLUCK_UP_FLAG;
    }
}

/*******************************************************************************
 * @fn      UDISK_Down_OnePack
 *
 * @brief   WRITE10 data transfer: receive sectors from USB into buffer, then
 *          write to SD card using BSP's EMMCCardWriteMulSec().
 *
 *          Unlike the READ path (streaming circular buffer), the WRITE path
 *          uses a two-phase approach:
 *            Phase 1: Receive all USB data for a chunk into UDisk_Out_Buf
 *            Phase 2: Write the chunk to SD via EMMCCardWriteMulSec()
 *
 *          This approach was chosen over streaming because the eMMC write
 *          path requires a different gap stop release mechanism than reads
 *          (R32_EMMC_RESPONSE3=0 vs clearing GAP_STOP bit), and the BSP
 *          function handles this correctly.
 *
 *          For transfers larger than the buffer (80 sectors / 40KB), the
 *          transfer is chunked: receive 80 sectors, write them, repeat.
 *
 *          The first sector (USB2) or first 2 sectors (USB3) are already
 *          in endp1Rbuff when this function is called, having been received
 *          by the USB ISR that triggered UDISK_OutPackflag.
 *
 * @note    EMMCCardWriteMulSec() uses CMD25 (WRITE_MULTIPLE_BLOCK) internally.
 *          It handles gap stop release via R32_EMMC_RESPONSE3=0 and issues
 *          CMD12 (STOP_TRANSMISSION) when done.
 *          [wch-ch56x-bsp/drv/CH56x_emmc.c: EMMCCardWriteMulSec()]
 *
 * [ref/SCSI_Block_Commands_SBC3_r25.pdf: WRITE(10)]
 * [ref/SD_Physical_Layer_Spec_v6.00.pdf: CMD25 WRITE_MULTIPLE_BLOCK]
 ******************************************************************************/
void UDISK_Down_OnePack( void )
{
    uint8_t s;
    uint16_t total_sectors;
    uint16_t sectors_left;
    uint16_t chunk_sectors;
    uint16_t sectors_received;
    uint16_t reqnum;
    uint32_t lba;
    uint16_t buf_sectors = UDISKSIZE / 512;  /* 80 sectors per buffer (40KB) */
    uint8_t first = 1;

    total_sectors = UDISK_Transfer_DataLen / 512;
    UDISK_Transfer_DataLen = 0;
    lba = UDISK_Cur_Sec_Lba;
    sectors_left = total_sectors;

    if( g_DeviceUsbType == USB_U20_SPEED )
    {
        /* ── USB2 High-Speed WRITE path ──
         * Disable USB HS interrupt, NAK EP0, enable auto-toggle for polling.
         * [CH569DS1.PDF: USB2.0 Endpoint Control Registers] */
        uint8_t uep0rxsave = R8_UEP0_RX_CTRL;
        uint8_t uep0txsave = R8_UEP0_TX_CTRL;
        PFIC_DisableIRQ(USBHS_IRQn);
        R8_UEP0_TX_CTRL = UEP_T_RES_NAK;
        R8_UEP0_RX_CTRL = UEP_R_RES_NAK;
        R8_UEP1_RX_CTRL |= RB_UEP_R_AUTOTOG;
        R8_USB_INT_FG = RB_USB_IF_TRANSFER;  /* Clear any stale flag from EP0 */

        while( sectors_left > 0 )
        {
            /* Determine chunk size: min(sectors_left, buffer_capacity) */
            chunk_sectors = (sectors_left > buf_sectors) ? buf_sectors : sectors_left;
            sectors_received = 0;

            /* Clear any stale transfer flag from previous eMMC write phase.
             * Without this, a flag set by EP0 SETUP during eMMC I/O causes
             * the first poll iteration to "succeed" with no new data. */
            R8_USB_INT_FG = RB_USB_IF_TRANSFER;

            cprintf("W lba=%lu n=%u\r\n", lba, chunk_sectors);

            /* First sector of first chunk is already in endp1Rbuff.
             * The USB ISR received it before setting UDISK_OutPackflag.
             * Copy it to the beginning of UDisk_Out_Buf. */
            if( first )
            {
                memcpy(UDisk_Out_Buf, endp1Rbuff, 512);
                sectors_received = 1;
                first = 0;
            }

            /* ── Phase 1: Receive remaining sectors via USB polling ──
             * For each sector:
             *   1. Point RX DMA to next slot in UDisk_Out_Buf
             *   2. ACK to allow host to send next OUT packet
             *   3. Poll RB_USB_IF_TRANSFER until packet arrives
             *   4. NAK to hold off next packet while we update DMA
             * [CH569DS1.PDF: R32_UEP1_RX_DMA, R8_UEP1_RX_CTRL] */
            while( sectors_received < chunk_sectors )
            {
                R32_UEP1_RX_DMA = (uint32_t)(UDisk_Out_Buf + sectors_received * 512);
                __asm__ volatile("fence" ::: "memory");  /* Ensure DMA addr visible before ACK */
                R8_UEP1_RX_CTRL = (R8_UEP1_RX_CTRL & ~RB_UEP_RRES_MASK) | UEP_R_RES_ACK;

                while( !(R8_USB_INT_FG & RB_USB_IF_TRANSFER) )
                {
                    /* Dismiss any SETUP activity during data phase */
                    if( R8_USB_INT_FG & RB_USB_IF_SETUOACT )
                        R8_USB_INT_FG = RB_USB_IF_SETUOACT;
                }
                {
                    uint8_t ist = R8_USB_INT_ST;
                    if((ist & 0x0F) != 1)  /* Not EP1 */
                        cprintf("W !EP%u tok=%u rx=%u\r\n", ist & 0x0F, (ist>>4)&3, sectors_received);
                }
                R8_USB_INT_FG = RB_USB_IF_TRANSFER;
                R8_UEP1_RX_CTRL = (R8_UEP1_RX_CTRL & ~RB_UEP_RRES_MASK) | UEP_R_RES_NAK;
                sectors_received++;
            }

            /* Checksum of first two and last sector in chunk for host cross-reference.
             * With the 127-word test pattern, XOR checksum = LBA value. */
            {
                uint32_t ckfirst = 0, cksecond = 0, cklast = 0;
                int i;
                for(i = 0; i < 128; i++) {
                    ckfirst ^= ((uint32_t*)UDisk_Out_Buf)[i];
                    if( chunk_sectors > 1 )
                        cksecond ^= ((uint32_t*)(UDisk_Out_Buf + 512))[i];
                    cklast ^= ((uint32_t*)(UDisk_Out_Buf + (chunk_sectors-1)*512))[i];
                }
                cprintf("W ck0=%08lx ck1=%08lx ck%u=%08lx\r\n",
                        ckfirst, cksecond, chunk_sectors-1, cklast);
            }

            /* ── Phase 2: Write chunk to SD card via BSP ──
             * EMMCCardWriteMulSec() handles CMD25, gap stop release via
             * R32_EMMC_RESPONSE3=0, and CMD12 internally.
             * Disable EMMC IRQ during transfer to prevent ISR from
             * interfering with the BSP's polling loop.
             * [wch-ch56x-bsp/drv/CH56x_emmc.c: EMMCCardWriteMulSec()] */
            PFIC_DisableIRQ(EMMC_IRQn);
            R16_EMMC_INT_FG = 0xffff;
            TF_EMMCParam.EMMCOpErr = 0;
            TF_EMMCParam.EMMCSecSize = 512;  /* Always use 512-byte blocks */
            reqnum = chunk_sectors;
            s = EMMCCardWriteMulSec(&TF_EMMCParam, &reqnum, UDisk_Out_Buf, lba);
            R16_EMMC_INT_FG = 0xffff;
            TF_EMMCParam.EMMCOpErr = 0;
            PFIC_EnableIRQ(EMMC_IRQn);

            cprintf("W s=%u req=%u act=%u\r\n", s, chunk_sectors, reqnum);

            if( s != CMD_SUCCESS )
            {
                /* Write failed: set MEDIUM ERROR / WRITE ERROR sense data.
                 * CSW will be sent with status=1 (failed).
                 * [ref/SCSI_Block_Commands_SBC3_r25.pdf: Sense Key 0x03] */
                UDISK_CMD_Deal_Status( 0x03, 0x0C, 0x01 );
                break;
            }

            /* Wait for card to finish programming (DAT0 goes high) */
            while( !EMMCDat0Sta );

            lba += chunk_sectors;
            sectors_left -= chunk_sectors;
        }

        /* Restore EP1 RX DMA to default endpoint buffer and disable auto-toggle.
         * Re-enable USB ISR to resume normal CBW reception. */
        R32_UEP1_RX_DMA = (uint32_t)(uint8_t *)endp1Rbuff;
        R8_UEP1_RX_CTRL &= ~RB_UEP_R_AUTOTOG;
        PFIC_EnableIRQ(USBHS_IRQn);
        R8_UEP0_TX_CTRL = uep0txsave;
        R8_UEP0_RX_CTRL = uep0rxsave;
    }
    else
    {
        /* ── USB3 SuperSpeed WRITE path ──
         * Same two-phase approach as USB2 but:
         *   - Receives 1024 bytes (2 sectors) per ERDY burst
         *   - Polls bit 26 of USBSS->UEP1_RX_CTRL for RX completion
         *   - Uses USB30_OUT_set() / USB30_send_ERDY() for flow control
         * [USB 3.0 Spec: ERDY Transaction Packet] */
        PFIC_DisableIRQ(USBSS_IRQn);

        while( sectors_left > 0 )
        {
            chunk_sectors = (sectors_left > buf_sectors) ? buf_sectors : sectors_left;
            sectors_received = 0;

            cprintf("W lba=%lu n=%u\r\n", lba, chunk_sectors);

            /* First burst (up to 2 sectors) already in endp1Rbuff */
            if( first )
            {
                uint16_t first_count = (chunk_sectors >= 2) ? 2 : 1;
                memcpy(UDisk_Out_Buf, endp1Rbuff, first_count * 512);
                sectors_received = first_count;
                first = 0;
            }

            /* Phase 1: Receive remaining sectors in 1024-byte bursts */
            while( sectors_received < chunk_sectors )
            {
                uint16_t this_burst;

                USBSS->UEP1_RX_DMA = (uint32_t)(UDisk_Out_Buf + sectors_received * 512);
                USB30_OUT_set(ENDP_1, ACK, 1);
                USB30_send_ERDY(ENDP_1 | OUT, 1);

                /* Poll bit 26 of UEP1_RX_CTRL for RX completion */
                while( !(USBSS->UEP1_RX_CTRL & ((uint32_t)1 << 26)) );
                USB30_OUT_clearIT(ENDP_1);
                USB30_OUT_set(ENDP_1, NRDY, 0);

                this_burst = 2;
                if( sectors_received + this_burst > chunk_sectors )
                    this_burst = chunk_sectors - sectors_received;
                sectors_received += this_burst;
            }

            /* Checksum of first and last sector in chunk for host cross-reference */
            {
                uint32_t ckfirst = 0, cklast = 0;
                int i;
                for(i = 0; i < 128; i++) {
                    ckfirst ^= ((uint32_t*)UDisk_Out_Buf)[i];
                    cklast ^= ((uint32_t*)(UDisk_Out_Buf + (chunk_sectors-1)*512))[i];
                }
                cprintf("W ck0=%08lx ck%u=%08lx\r\n", ckfirst, chunk_sectors-1, cklast);
            }

            /* Phase 2: Write chunk to SD card via BSP */
            PFIC_DisableIRQ(EMMC_IRQn);
            R16_EMMC_INT_FG = 0xffff;
            TF_EMMCParam.EMMCOpErr = 0;
            TF_EMMCParam.EMMCSecSize = 512;  /* Always use 512-byte blocks */
            reqnum = chunk_sectors;
            s = EMMCCardWriteMulSec(&TF_EMMCParam, &reqnum, UDisk_Out_Buf, lba);
            R16_EMMC_INT_FG = 0xffff;
            TF_EMMCParam.EMMCOpErr = 0;
            PFIC_EnableIRQ(EMMC_IRQn);

            cprintf("W s=%u req=%u act=%u\r\n", s, chunk_sectors, reqnum);

            if( s != CMD_SUCCESS )
            {
                UDISK_CMD_Deal_Status( 0x03, 0x0C, 0x01 );
                break;
            }

            /* Wait for card to finish programming (DAT0 goes high) */
            while( !EMMCDat0Sta );

            lba += chunk_sectors;
            sectors_left -= chunk_sectors;
        }

        /* Restore EP1 RX DMA to default endpoint buffer */
        USBSS->UEP1_RX_DMA = (uint32_t)(uint8_t *)endp1Rbuff;
        USB30_OUT_clearIT(ENDP_1);
        PFIC_EnableIRQ(USBSS_IRQn);
    }

    /* Send CSW to complete the WRITE transaction */
    if( UDISK_Transfer_DataLen == 0 )
    {
        Udisk_Transfer_Status &= ~DEF_UDISK_BLUCK_DOWN_FLAG;
        UDISK_Up_CSW( );
    }
}

/*******************************************************************************
 * @fn      EMMC_IRQHandler
 *
 * @brief   eMMC controller interrupt handler.
 *          Fires on any error condition enabled in R16_EMMC_INT_EN:
 *            - RB_EMMC_IE_FIFO_OV:  FIFO overflow
 *            - RB_EMMC_IE_TRANERR:  Transfer error
 *            - RB_EMMC_IE_DATTMO:   Data timeout
 *            - RB_EMMC_IE_REIDX_ER: Response index error
 *            - RB_EMMC_IE_RECRC_WR: Response CRC error
 *            - RB_EMMC_IE_RE_TMOUT: Response timeout
 *          (Enabled in EMMCIO0Init() [wch-ch56x-bsp/drv/CH56x_emmc.c])
 *
 *          Sets TF_EMMCParam.EMMCOpErr=1 which is checked by:
 *            - CheckCMDComp() in the BSP (returns CMD_FAILED)
 *            - EMMCCardWriteMulSec() (returns CMD_FAILED)
 *            - EMMCCardReadMulSec() (returns CMD_FAILED)
 *
 *          Note: BKGAP and TRANDONE are NOT enabled as interrupts - they
 *          are polled in the transfer loops. Only error conditions generate
 *          interrupts.
 *
 *          Uses WCH-Interrupt-fast attribute for minimal ISR entry/exit
 *          latency on the QingKe RISC-V core.
 *          [QingKe V3 Processor Manual: Fast Interrupt Handling]
 *
 * [CH569DS1.PDF: R16_EMMC_INT_FG, R16_EMMC_INT_EN]
 ******************************************************************************/
void EMMC_IRQHandler(void)
{
    uint16_t t = R16_EMMC_INT_FG;
    if(t)
    {
        TF_EMMCParam.EMMCOpErr = 1;
        R16_EMMC_INT_FG = t;  /* Clear interrupt flags by writing 1s */
    }
}
