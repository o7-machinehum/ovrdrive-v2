/********************************** (C) COPYRIGHT *******************************
* File Name          : sw_udisk.c
* Description        : USB Mass Storage (MSC/BOT) protocol layer
*                      Ported from WCH CH569 example to HydraUSB3 BSP
*                      Dummy RAM-backed storage (no eMMC/SD)
*                      Uses EP1 IN (0x81) / EP1 OUT (0x01) for bulk transfers
* SPDX-License-Identifier: Apache-2.0
*******************************************************************************/

#include "sw_udisk.h"

/******************************************************************************/
/* INQUIRY response data */
uint8_t UDISK_Inquity_Tab[ ] =
{
    0x00,                                                /* Peripheral Device Type: UDISK */
    0x80,                                                /* Removable */
    0x02,                                                /* ISO/ECMA version */
    0x02,
    0x1F,                                                /* Additional Length */
    0x00,
    0x00,
    0x00,
    'H',                                                 /* Vendor Information */
    'y',
    'd',
    'r',
    'a',
    'U',
    'S',
    'B',
    'U',                                                 /* Product Identification */
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
    'D',                                                 /* Product Revision */
    'i',
    's',
    'k',
    ' ',
    'W',
    'C',
    'H'
};

/******************************************************************************/
/* READ FORMAT CAPACITY response */
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
/* READ CAPACITY response */
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
/* MODE_SENSE response for CMD 0x1A */
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
/* MODE_SENSE response for CMD 0x5A */
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


volatile uint8_t  Udisk_Status = 0x00;
volatile uint8_t  Udisk_Transfer_Status = 0x00;
volatile uint32_t Udisk_Capability = 0x00;
volatile uint8_t  Udisk_CBW_Tag_Save[ 4 ];
volatile uint8_t  Udisk_Sense_Key = 0x00;
volatile uint8_t  Udisk_Sense_ASC = 0x00;
volatile uint8_t  Udisk_CSW_Status = 0x00;

volatile uint32_t UDISK_Transfer_DataLen = 0x00;
volatile uint32_t UDISK_Cur_Sec_Lba = 0x00;
volatile uint16_t UDISK_Sec_Pack_Count = 0x00;
volatile uint16_t UDISK_Pack_Size = DEF_UDISK_PACK_512;

volatile uint8_t UDISK_OutPackflag = 0;
volatile uint8_t UDISK_InPackflag = 0;

BULK_ONLY_CMD mBOC;
uint8_t   *pEndp2_Buf;


/*******************************************************************************
* Function Name  : UDISK_CMD_Deal_Status
* Description    : Set current command execution status
*******************************************************************************/
void UDISK_CMD_Deal_Status( uint8_t key, uint8_t asc, uint8_t status )
{
    Udisk_Sense_Key  = key;
    Udisk_Sense_ASC  = asc;
    Udisk_CSW_Status = status;
}

/*******************************************************************************
* Function Name  : UDISK_CMD_Deal_Fail
* Description    : Handle command failure - STALL appropriate endpoints
*                  Uses EP1 IN/OUT (mapped from original EP2 TX / EP3 RX)
*******************************************************************************/
void UDISK_CMD_Deal_Fail( void )
{
    if( Udisk_Transfer_Status & DEF_UDISK_BLUCK_UP_FLAG )
    {
        /* STALL EP1 IN (bulk IN) */
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
        /* STALL EP1 OUT (bulk OUT) */
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
* Function Name  : CMD_RD_WR_Deal_Pre
* Description    : Prepare for READ10/WRITE10 command
*******************************************************************************/
void CMD_RD_WR_Deal_Pre( void )
{
    /* Extract LBA */
    UDISK_Cur_Sec_Lba = (uint32_t)mBOC.mCBW.mCBW_CB_Buf[ 2 ] << 24;
    UDISK_Cur_Sec_Lba = UDISK_Cur_Sec_Lba + ( (uint32_t)mBOC.mCBW.mCBW_CB_Buf[ 3 ] << 16 );
    UDISK_Cur_Sec_Lba = UDISK_Cur_Sec_Lba + ( (uint32_t)mBOC.mCBW.mCBW_CB_Buf[ 4 ] << 8 );
    UDISK_Cur_Sec_Lba = UDISK_Cur_Sec_Lba + ( (uint32_t)mBOC.mCBW.mCBW_CB_Buf[ 5 ] );

    /* Extract transfer length in bytes */
    UDISK_Transfer_DataLen = ( (uint32_t)mBOC.mCBW.mCBW_CB_Buf[ 7 ] << 8 );
    UDISK_Transfer_DataLen = UDISK_Transfer_DataLen + ( (uint32_t)mBOC.mCBW.mCBW_CB_Buf[ 8 ] );
    UDISK_Transfer_DataLen = UDISK_Transfer_DataLen * DEF_UDISK_SECTOR_SIZE;

    UDISK_Sec_Pack_Count = 0x00;
    UDISK_CMD_Deal_Status( 0x00, 0x00, 0x00 );
}

/*******************************************************************************
* Function Name  : UDISK_SCSI_CMD_Deal
* Description    : Process SCSI commands from CBW
*******************************************************************************/
void UDISK_SCSI_CMD_Deal( void )
{
    uint8_t i;

    if( ( mBOC.mCBW.mCBW_Sig[ 0 ] == 'U' ) && ( mBOC.mCBW.mCBW_Sig[ 1 ] == 'S' )
      &&( mBOC.mCBW.mCBW_Sig[ 2 ] == 'B' ) && ( mBOC.mCBW.mCBW_Sig[ 3 ] == 'C' ) )
    {
        Udisk_CBW_Tag_Save[ 0 ] = mBOC.mCBW.mCBW_Tag[ 0 ];
        Udisk_CBW_Tag_Save[ 1 ] = mBOC.mCBW.mCBW_Tag[ 1 ];
        Udisk_CBW_Tag_Save[ 2 ] = mBOC.mCBW.mCBW_Tag[ 2 ];
        Udisk_CBW_Tag_Save[ 3 ] = mBOC.mCBW.mCBW_Tag[ 3 ];

        UDISK_Transfer_DataLen = ( uint32_t )mBOC.mCBW.mCBW_DataLen[ 3 ] << 24;
        UDISK_Transfer_DataLen += ( ( uint32_t )mBOC.mCBW.mCBW_DataLen[ 2 ] << 16 );
        UDISK_Transfer_DataLen += ( ( uint32_t )mBOC.mCBW.mCBW_DataLen[ 1 ] << 8 );
        UDISK_Transfer_DataLen += ( ( uint32_t )mBOC.mCBW.mCBW_DataLen[ 0 ] );

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
        Udisk_Transfer_Status |= DEF_UDISK_CSW_UP_FLAG;

        /* Process SCSI command */
        switch( mBOC.mCBW.mCBW_CB_Buf[ 0 ] )
        {
            case  CMD_U_INQUIRY:
                /* CMD: 0x12 */
                if( UDISK_Transfer_DataLen > 0x24 )
                {
                    UDISK_Transfer_DataLen = 0x24;
                }
                UDISK_Inquity_Tab[ 0 ] = 0x00;
                pEndp2_Buf = (uint8_t *)UDISK_Inquity_Tab;
                UDISK_CMD_Deal_Status( 0x00, 0x00, 0x00 );
                break;

            case  CMD_U_READ_FORMAT_CAPACITY:
                /* CMD: 0x23 */
                if( ( Udisk_Status & DEF_UDISK_EN_FLAG ) )
                {
                    if( UDISK_Transfer_DataLen > 0x0C )
                    {
                        UDISK_Transfer_DataLen = 0x0C;
                    }

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
                    UDISK_CMD_Deal_Status( 0x02, 0x3A, 0x01 );
                    UDISK_CMD_Deal_Fail( );
                }
                break;

            case  CMD_U_READ_CAPACITY:
                /* CMD: 0x25 */
                if( ( Udisk_Status & DEF_UDISK_EN_FLAG ) )
                {
                    if( UDISK_Transfer_DataLen > 0x08 )
                    {
                        UDISK_Transfer_DataLen = 0x08;
                    }

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
                /* CMD: 0x28 */
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
                /* CMD: 0x2E */
            case  CMD_U_WRITE10:
                /* CMD: 0x2A */
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
                /* CMD: 0x1A */
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
                /* CMD: 0x5A */
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
                    mBOC.buf[ 8 ]  = ( Udisk_Capability >> 24 ) & 0xFF;
                    mBOC.buf[ 9 ]  = ( Udisk_Capability >> 16 ) & 0xFF;
                    mBOC.buf[ 10 ] = ( Udisk_Capability >> 8  ) & 0xFF;
                    mBOC.buf[ 11 ] = ( Udisk_Capability       ) & 0xFF;
                    pEndp2_Buf = mBOC.buf;
                }
                else
                {
                    UDISK_CMD_Deal_Status( 0x05, 0x20, 0x01 );
                    UDISK_CMD_Deal_Fail( );
                }
                break;

            case  CMD_U_REQUEST_SENSE:
                /* CMD: 0x03 */
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
                /* CMD: 0x1E */
                UDISK_CMD_Deal_Status( 0x00, 0x00, 0x00 );
                break;

            case  CMD_U_VERIFY10:
                /* CMD: 0x1F */
                UDISK_CMD_Deal_Status( 0x00, 0x00, 0x00 );
                break;

            case  CMD_U_START_STOP:
                /* CMD: 0x1B */
                UDISK_CMD_Deal_Status( 0x00, 0x00, 0x00 );
                break;

            default:
                UDISK_CMD_Deal_Status( 0x05, 0x20, 0x01 );
                Udisk_Transfer_Status |= DEF_UDISK_BLUCK_UP_FLAG;
                UDISK_CMD_Deal_Fail( );
                break;
        }
    }
    else
    {   /* CBW signature invalid */
        UDISK_CMD_Deal_Status( 0x05, 0x20, 0x02 );
        Udisk_Transfer_Status |= DEF_UDISK_BLUCK_UP_FLAG;
        Udisk_Transfer_Status |= DEF_UDISK_BLUCK_DOWN_FLAG;
        UDISK_CMD_Deal_Fail(  );
    }
}

/*******************************************************************************
* Function Name  : UDISK_In_EP_Deal
* Description    : Bulk IN endpoint handler (EP1 IN transfer complete)
*******************************************************************************/
void UDISK_In_EP_Deal( void )
{
    if( Udisk_Transfer_Status & DEF_UDISK_BLUCK_UP_FLAG )
    {
        if( mBOC.mCBW.mCBW_CB_Buf[ 0 ] == CMD_U_READ10 )
        {
            /* Defer to main loop for READ10 data transfer */
            UDISK_InPackflag = 1;
        }
        else
        {
            UDISK_Bulk_UpData( );
        }
    }
    else if( Udisk_Transfer_Status & DEF_UDISK_CSW_UP_FLAG )
    {
        UDISK_Up_CSW( );
    }
}

/*******************************************************************************
* Function Name  : UDISK_Out_EP_Deal
* Description    : Bulk OUT endpoint handler (EP1 OUT data received)
*******************************************************************************/
void UDISK_Out_EP_Deal( uint8_t *pbuf, uint16_t packlen )
{
    uint32_t i;

    if( Udisk_Transfer_Status & DEF_UDISK_BLUCK_DOWN_FLAG )
    {
        /* Data phase of WRITE command - defer to main loop */
        UDISK_OutPackflag = 1;
    }
    else
    {
        if( packlen == 0x1F )
        {
            for( i = 0; i < packlen; i++ )
            {
                mBOC.buf[ i ] = *pbuf++;
            }

            /* Process SCSI command from CBW */
            UDISK_SCSI_CMD_Deal( );
            if( ( Udisk_Transfer_Status & DEF_UDISK_BLUCK_DOWN_FLAG ) == 0x00 )
            {
                if( Udisk_Transfer_Status & DEF_UDISK_BLUCK_UP_FLAG )
                {
                    if( mBOC.mCBW.mCBW_CB_Buf[ 0 ] == CMD_U_READ10 )
                    {
                        /* Defer READ10 to main loop */
                        UDISK_InPackflag = 1;
                    }
                    else
                    {
                        UDISK_Bulk_UpData( );
                    }
                }
                else if( Udisk_CSW_Status == 0x00 )
                {
                    UDISK_Up_CSW( );
                }
            }
        }
    }
}

/*******************************************************************************
* Function Name  : UDISK_Bulk_UpData
* Description    : Send bulk data to host via EP1 IN
*                  Used for non-READ10 responses (INQUIRY, CAPACITY, etc.)
*******************************************************************************/
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
        memcpy(endp1Tbuff, pEndp2_Buf, len);
        R16_UEP1_T_LEN = len;
        R32_UEP1_TX_DMA = (uint32_t)(uint8_t *)endp1Tbuff;
        R8_UEP1_TX_CTRL = (R8_UEP1_TX_CTRL & ~RB_UEP_TRES_MASK) | UEP_T_RES_ACK;
    }
    else
    {
        memcpy(endp1Tbuff, pEndp2_Buf, len);
        USBSS->UEP1_TX_DMA = (uint32_t)(uint8_t *)endp1Tbuff;
        USB30_IN_set(ENDP_1, ENABLE, ACK, DEF_ENDP1_IN_BURST_LEVEL, len);
        USB30_send_ERDY(ENDP_1 | IN, DEF_ENDP1_IN_BURST_LEVEL);
    }
}

/*******************************************************************************
* Function Name  : UDISK_Up_CSW
* Description    : Send Command Status Wrapper via EP1 IN
*******************************************************************************/
void UDISK_Up_CSW( void )
{
    Udisk_Transfer_Status = 0x00;

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
* Function Name  : UDISK_onePack_Deal
* Description    : Main loop handler for deferred READ10/WRITE10 data transfers
*******************************************************************************/
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
* Function Name  : UDISK_Up_OnePack
* Description    : READ10 data transfer - dummy version sends zeros
*                  Polls USB hardware directly with interrupts disabled
*******************************************************************************/
void UDISK_Up_OnePack( void )
{
    uint16_t preqnum;

    preqnum = UDISK_Transfer_DataLen / 512;
    UDISK_Transfer_DataLen = 0;

    /* Fill TX buffer with zeros (dummy storage) */
    memset(endp1Tbuff, 0, 512);

    if( g_DeviceUsbType == USB_U20_SPEED )
    {
        uint8_t uep0rxsave = R8_UEP0_RX_CTRL;
        uint8_t uep0txsave = R8_UEP0_TX_CTRL;
        PFIC_DisableIRQ(USBHS_IRQn);
        R8_UEP0_TX_CTRL = UEP_T_RES_NAK;
        R8_UEP0_RX_CTRL = UEP_R_RES_NAK;

        uint8_t first = 1;
        uint16_t i;
        for( i = 0; i < preqnum; i++ )
        {
            if( !first )
            {
                while( !(R8_USB_INT_FG & RB_USB_IF_TRANSFER) )
                {
                    if( R8_USB_INT_FG & RB_USB_IF_SETUOACT )
                        R8_USB_INT_FG = RB_USB_IF_SETUOACT;
                }
                R8_USB_INT_FG = RB_USB_IF_TRANSFER;
            }
            first = 0;

            R8_UEP1_TX_CTRL = (R8_UEP1_TX_CTRL & ~RB_UEP_TRES_MASK) | UEP_T_RES_NAK;
            R32_UEP1_TX_DMA = (uint32_t)(uint8_t *)endp1Tbuff;
            R16_UEP1_T_LEN = 512;
            R8_UEP1_TX_CTRL = (R8_UEP1_TX_CTRL & ~RB_UEP_TRES_MASK) | UEP_T_RES_ACK;
        }

        /* Wait for last packet to complete */
        if( preqnum > 0 )
        {
            while( !(R8_USB_INT_FG & RB_USB_IF_TRANSFER) )
            {
                if( R8_USB_INT_FG & RB_USB_IF_SETUOACT )
                    R8_USB_INT_FG = RB_USB_IF_SETUOACT;
            }
            R8_UEP1_TX_CTRL = (R8_UEP1_TX_CTRL & ~RB_UEP_TRES_MASK) | UEP_T_RES_NAK;
            R8_USB_INT_FG = RB_USB_IF_TRANSFER;
        }

        PFIC_EnableIRQ(USBHS_IRQn);
        R8_UEP0_TX_CTRL = uep0txsave;
        R8_UEP0_RX_CTRL = uep0rxsave;
    }
    else
    {
        /* USB3 path: send sectors using burst transfers */
        PFIC_DisableIRQ(USBSS_IRQn);

        memset(endp1Tbuff, 0, 1024);

        uint16_t sent = 0;
        uint8_t first = 1;

        while( sent < preqnum )
        {
            if( (USBSS->UEP1_TX_CTRL & ((uint32_t)1 << 31)) || first )
            {
                first = 0;
                USB30_IN_clearIT(ENDP_1);

                uint16_t remaining = preqnum - sent;
                uint16_t tx_len;
                uint16_t tx_sectors;

                if( remaining >= 2 )
                {
                    tx_sectors = 2;
                    tx_len = 1024;
                }
                else
                {
                    tx_sectors = 1;
                    tx_len = 512;
                }

                USBSS->UEP1_TX_DMA = (uint32_t)(uint8_t *)endp1Tbuff;
                USB30_IN_set(ENDP_1, ENABLE, ACK, 1, tx_len);
                USB30_send_ERDY(ENDP_1 | IN, 1);

                sent += tx_sectors;
            }
        }

        /* Wait for last transfer to complete */
        while( !(USBSS->UEP1_TX_CTRL & ((uint32_t)1 << 31)) );
        USB30_IN_clearIT(ENDP_1);

        PFIC_EnableIRQ(USBSS_IRQn);
    }

    UDISK_Cur_Sec_Lba += preqnum;

    if( UDISK_Transfer_DataLen == 0x00 )
    {
        Udisk_Transfer_Status &= ~DEF_UDISK_BLUCK_UP_FLAG;
    }

    UDISK_Up_CSW( );
}

/*******************************************************************************
* Function Name  : UDISK_Down_OnePack
* Description    : WRITE10 data transfer - dummy version receives and discards
*                  Polls USB hardware directly with interrupts disabled
*******************************************************************************/
void UDISK_Down_OnePack( void )
{
    uint16_t preqnum;

    preqnum = UDISK_Transfer_DataLen / 512;
    UDISK_Transfer_DataLen = 0;

    if( g_DeviceUsbType == USB_U20_SPEED )
    {
        uint8_t uep0rxsave = R8_UEP0_RX_CTRL;
        uint8_t uep0txsave = R8_UEP0_TX_CTRL;
        PFIC_DisableIRQ(USBHS_IRQn);
        R8_UEP0_TX_CTRL = UEP_T_RES_NAK;
        R8_UEP0_RX_CTRL = UEP_R_RES_NAK;

        /* First sector already received (triggered UDISK_OutPackflag) */
        uint16_t received = 1;

        while( received < preqnum )
        {
            /* Re-arm EP1 OUT for next packet */
            R32_UEP1_RX_DMA = (uint32_t)(uint8_t *)endp1Rbuff;
            R8_UEP1_RX_CTRL = (R8_UEP1_RX_CTRL & ~RB_UEP_RRES_MASK) | UEP_R_RES_ACK;

            while( !(R8_USB_INT_FG & RB_USB_IF_TRANSFER) )
            {
                if( R8_USB_INT_FG & RB_USB_IF_SETUOACT )
                    R8_USB_INT_FG = RB_USB_IF_SETUOACT;
            }
            R8_USB_INT_FG = RB_USB_IF_TRANSFER;
            received++;
        }

        R8_UEP1_RX_CTRL = (R8_UEP1_RX_CTRL & ~RB_UEP_RRES_MASK) | UEP_R_RES_NAK;
        R32_UEP1_RX_DMA = (uint32_t)(uint8_t *)endp1Rbuff;

        PFIC_EnableIRQ(USBHS_IRQn);
        R8_UEP0_TX_CTRL = uep0txsave;
        R8_UEP0_RX_CTRL = uep0rxsave;
    }
    else
    {
        /* USB3 path: receive sectors and discard */
        PFIC_DisableIRQ(USBSS_IRQn);

        /* First burst already received (2 sectors = 1 USB3 packet) */
        uint16_t received = 2;

        while( received < preqnum )
        {
            USB30_OUT_clearIT(ENDP_1);
            USBSS->UEP1_RX_DMA = (uint32_t)(uint8_t *)endp1Rbuff;
            USB30_OUT_set(ENDP_1, ACK, 1);
            USB30_send_ERDY(ENDP_1 | OUT, 1);

            /* Wait for transfer complete (bit 31 of RX_CTRL) */
            while( !(USBSS->UEP1_RX_CTRL & ((uint32_t)1 << 31)) );

            received += 2; /* Each USB3 packet = 1024 bytes = 2 sectors */
        }

        USB30_OUT_clearIT(ENDP_1);
        USBSS->UEP1_RX_DMA = (uint32_t)(uint8_t *)endp1Rbuff;

        PFIC_EnableIRQ(USBSS_IRQn);
    }

    Udisk_Transfer_Status &= ~DEF_UDISK_BLUCK_DOWN_FLAG;
    UDISK_Up_CSW( );
}
