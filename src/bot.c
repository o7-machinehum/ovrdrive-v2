/* bot.c - BOT (Bulk-Only Transport) protocol layer
 *
 * Host PC <--USB--> BOT (this file) <--SCSI--> msc_read/msc_write
 *
 * [ref/USB_MSC_Bulk_Only_Transport_v1.0.pdf]
 * [ref/SCSI_Block_Commands_SBC3_r25.pdf]
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "bot.h"
#include "scsi_tables.h"
#include "msc_read.h"
#include "msc_write.h"
#include "ovrdrive.h"
#include "CH56x_usb30_devbulk.h"
#include "CH56x_usb20_devbulk.h"
#include "CH56x_usb30_devbulk_LIB.h"
#include "CH56x_debug_log.h"
#include <string.h>

void bot_set_sense(uint8_t key, uint8_t asc, uint8_t status)
{
    Udisk_Sense_Key  = key;
    Udisk_Sense_ASC  = asc;
    Udisk_CSW_Status = status;
}

void bot_stall_endpoints(void)
{
    if (Udisk_Transfer_Status & BOT_FLAG_DATA_IN) {
        if (g_DeviceUsbType == USB_U20_SPEED)
            R8_UEP1_TX_CTRL = (R8_UEP1_TX_CTRL & ~RB_UEP_TRES_MASK) | UEP_T_RES_STALL;
        else
            USB30_IN_set(ENDP_1, ENABLE, STALL, 0, 0);
        Udisk_Transfer_Status &= ~BOT_FLAG_DATA_IN;
    }
    if (Udisk_Transfer_Status & BOT_FLAG_DATA_OUT) {
        if (g_DeviceUsbType == USB_U20_SPEED)
            R8_UEP1_RX_CTRL = (R8_UEP1_RX_CTRL & ~RB_UEP_RRES_MASK) | UEP_R_RES_STALL;
        else
            USB30_OUT_set(ENDP_1, STALL, 0);
        Udisk_Transfer_Status &= ~BOT_FLAG_DATA_OUT;
    }
}

void scsi_parse_rw10_cdb(void)
{
    UDISK_Cur_Sec_Lba = (uint32_t)mBOC.mCBW.mCBW_CB_Buf[2] << 24;
    UDISK_Cur_Sec_Lba += (uint32_t)mBOC.mCBW.mCBW_CB_Buf[3] << 16;
    UDISK_Cur_Sec_Lba += (uint32_t)mBOC.mCBW.mCBW_CB_Buf[4] << 8;
    UDISK_Cur_Sec_Lba += (uint32_t)mBOC.mCBW.mCBW_CB_Buf[5];

    UDISK_Transfer_DataLen = (uint32_t)mBOC.mCBW.mCBW_CB_Buf[7] << 8;
    UDISK_Transfer_DataLen += (uint32_t)mBOC.mCBW.mCBW_CB_Buf[8];
    UDISK_Transfer_DataLen *= SECTOR_SIZE;

    UDISK_Sec_Pack_Count = 0x00;
    bot_set_sense(SENSE_KEY_NO_ERROR, SENSE_ASC_NO_ERROR, CSW_STATUS_PASSED);
}

void bot_dispatch_scsi(void)
{
    uint8_t i;

    if ((mBOC.mCBW.mCBW_Sig[0] == 'U') && (mBOC.mCBW.mCBW_Sig[1] == 'S') &&
        (mBOC.mCBW.mCBW_Sig[2] == 'B') && (mBOC.mCBW.mCBW_Sig[3] == 'C'))
    {
        Udisk_CBW_Tag_Save[0] = mBOC.mCBW.mCBW_Tag[0];
        Udisk_CBW_Tag_Save[1] = mBOC.mCBW.mCBW_Tag[1];
        Udisk_CBW_Tag_Save[2] = mBOC.mCBW.mCBW_Tag[2];
        Udisk_CBW_Tag_Save[3] = mBOC.mCBW.mCBW_Tag[3];

        UDISK_Transfer_DataLen  = (uint32_t)mBOC.mCBW.mCBW_DataLen[3] << 24;
        UDISK_Transfer_DataLen += (uint32_t)mBOC.mCBW.mCBW_DataLen[2] << 16;
        UDISK_Transfer_DataLen += (uint32_t)mBOC.mCBW.mCBW_DataLen[1] << 8;
        UDISK_Transfer_DataLen += (uint32_t)mBOC.mCBW.mCBW_DataLen[0];

        if (UDISK_Transfer_DataLen) {
            if (mBOC.mCBW.mCBW_Flag & 0x80)
                Udisk_Transfer_Status |= BOT_FLAG_DATA_IN;
            else
                Udisk_Transfer_Status |= BOT_FLAG_DATA_OUT;
        }
        Udisk_Transfer_Status |= BOT_FLAG_CSW_PENDING;

#ifdef DEBUG_USB
        log_printf("CBW: cmd=0x%02X len=%d\r\n", mBOC.mCBW.mCBW_CB_Buf[0], UDISK_Transfer_DataLen);
#endif

        switch (mBOC.mCBW.mCBW_CB_Buf[0])
        {
            case CMD_U_INQUIRY:
                if (UDISK_Transfer_DataLen > INQUIRY_RESPONSE_SIZE)
                    UDISK_Transfer_DataLen = INQUIRY_RESPONSE_SIZE;
                g_inquiry_response[0] = 0x00;
                pEndp2_Buf = (uint8_t *)g_inquiry_response;
                bot_set_sense(SENSE_KEY_NO_ERROR, SENSE_ASC_NO_ERROR, CSW_STATUS_PASSED);
                break;

            case CMD_U_READ_FORMAT_CAPACITY:
                if (Udisk_Status & BOT_FLAG_DEVICE_READY) {
                    if (UDISK_Transfer_DataLen > 0x0C)
                        UDISK_Transfer_DataLen = 0x0C;
                    for (i = 0; i < UDISK_Transfer_DataLen; i++)
                        mBOC.buf[i] = g_format_capacity_response[i];
                    mBOC.buf[4] = (Udisk_Capability >> 24) & 0xFF;
                    mBOC.buf[5] = (Udisk_Capability >> 16) & 0xFF;
                    mBOC.buf[6] = (Udisk_Capability >> 8) & 0xFF;
                    mBOC.buf[7] = (Udisk_Capability) & 0xFF;
                    pEndp2_Buf = mBOC.buf;
                    bot_set_sense(SENSE_KEY_NO_ERROR, SENSE_ASC_NO_ERROR, CSW_STATUS_PASSED);
                } else {
                    bot_set_sense(SENSE_KEY_NOT_READY, SENSE_ASC_MEDIUM_NOT_PRESENT, CSW_STATUS_FAILED);
                    bot_stall_endpoints();
                }
                break;

            case CMD_U_READ_CAPACITY:
                if (Udisk_Status & BOT_FLAG_DEVICE_READY) {
                    if (UDISK_Transfer_DataLen > 0x08)
                        UDISK_Transfer_DataLen = 0x08;
                    for (i = 0; i < UDISK_Transfer_DataLen; i++)
                        mBOC.buf[i] = g_read_capacity_response[i];
                    mBOC.buf[0] = ((Udisk_Capability - 1) >> 24) & 0xFF;
                    mBOC.buf[1] = ((Udisk_Capability - 1) >> 16) & 0xFF;
                    mBOC.buf[2] = ((Udisk_Capability - 1) >> 8) & 0xFF;
                    mBOC.buf[3] = ((Udisk_Capability - 1)) & 0xFF;
                    pEndp2_Buf = mBOC.buf;
                    bot_set_sense(SENSE_KEY_NO_ERROR, SENSE_ASC_NO_ERROR, CSW_STATUS_PASSED);
                } else {
                    bot_set_sense(SENSE_KEY_NOT_READY, SENSE_ASC_MEDIUM_NOT_PRESENT, CSW_STATUS_FAILED);
                    bot_stall_endpoints();
                }
                break;

            case CMD_U_READ10:
                if (Udisk_Status & BOT_FLAG_DEVICE_READY)
                    scsi_parse_rw10_cdb();
                else {
                    bot_set_sense(SENSE_KEY_NOT_READY, SENSE_ASC_MEDIUM_NOT_PRESENT, CSW_STATUS_FAILED);
                    bot_stall_endpoints();
                }
                break;

            case CMD_U_WR_VERIFY10:
            case CMD_U_WRITE10:
                if (Udisk_Status & BOT_FLAG_DEVICE_READY)
                    scsi_parse_rw10_cdb();
                else {
                    bot_set_sense(SENSE_KEY_NOT_READY, SENSE_ASC_MEDIUM_NOT_PRESENT, CSW_STATUS_FAILED);
                    bot_stall_endpoints();
                }
                break;

            case CMD_U_MODE_SENSE:
                if (Udisk_Status & BOT_FLAG_DEVICE_READY) {
                    if (UDISK_Transfer_DataLen > 0x0C)
                        UDISK_Transfer_DataLen = 0x0C;
                    for (i = 0; i < UDISK_Transfer_DataLen; i++)
                        mBOC.buf[i] = g_mode_sense6_response[i];
                    mBOC.buf[4] = (Udisk_Capability >> 24) & 0xFF;
                    mBOC.buf[5] = (Udisk_Capability >> 16) & 0xFF;
                    mBOC.buf[6] = (Udisk_Capability >> 8) & 0xFF;
                    mBOC.buf[7] = (Udisk_Capability) & 0xFF;
                    pEndp2_Buf = mBOC.buf;
                } else {
                    bot_set_sense(SENSE_KEY_NOT_READY, SENSE_ASC_MEDIUM_NOT_PRESENT, CSW_STATUS_FAILED);
                    bot_stall_endpoints();
                }
                break;

            case CMD_U_MODE_SENSE2:
                if (mBOC.mCBW.mCBW_CB_Buf[2] == 0x3F) {
                    if (UDISK_Transfer_DataLen > 0x10)
                        UDISK_Transfer_DataLen = 0x10;
                    for (i = 0; i < UDISK_Transfer_DataLen; i++)
                        mBOC.buf[i] = g_mode_sense10_response[i];
                    mBOC.buf[8]  = (Udisk_Capability >> 24) & 0xFF;
                    mBOC.buf[9]  = (Udisk_Capability >> 16) & 0xFF;
                    mBOC.buf[10] = (Udisk_Capability >> 8) & 0xFF;
                    mBOC.buf[11] = (Udisk_Capability) & 0xFF;
                    pEndp2_Buf = mBOC.buf;
                } else {
                    bot_set_sense(SENSE_KEY_ILLEGAL_REQUEST, SENSE_ASC_INVALID_COMMAND, CSW_STATUS_FAILED);
                    bot_stall_endpoints();
                }
                break;

            case CMD_U_REQUEST_SENSE:
                mBOC.ReqSense.ErrorCode = 0x70;
                mBOC.ReqSense.Reserved1 = 0x00;
                mBOC.ReqSense.SenseKey = Udisk_Sense_Key;
                mBOC.ReqSense.Information[0] = 0x00;
                mBOC.ReqSense.Information[1] = 0x00;
                mBOC.ReqSense.Information[2] = 0x00;
                mBOC.ReqSense.Information[3] = 0x00;
                mBOC.ReqSense.SenseLength = 0x0A;
                mBOC.ReqSense.Reserved2[0] = 0x00;
                mBOC.ReqSense.Reserved2[1] = 0x00;
                mBOC.ReqSense.Reserved2[2] = 0x00;
                mBOC.ReqSense.Reserved2[3] = 0x00;
                mBOC.ReqSense.SenseCode = Udisk_Sense_ASC;
                mBOC.ReqSense.SenseCodeQua = 0x00;
                mBOC.ReqSense.Reserved3[0] = 0x00;
                mBOC.ReqSense.Reserved3[1] = 0x00;
                mBOC.ReqSense.Reserved3[2] = 0x00;
                mBOC.ReqSense.Reserved3[3] = 0x00;
                pEndp2_Buf = mBOC.buf;
                Udisk_CSW_Status = CSW_STATUS_PASSED;
                break;

            case CMD_U_TEST_READY:
                if (Udisk_Status & BOT_FLAG_DEVICE_READY)
                    bot_set_sense(SENSE_KEY_NO_ERROR, SENSE_ASC_NO_ERROR, CSW_STATUS_PASSED);
                else {
                    bot_set_sense(SENSE_KEY_NOT_READY, SENSE_ASC_MEDIUM_NOT_PRESENT, CSW_STATUS_FAILED);
                    Udisk_Transfer_Status |= BOT_FLAG_DATA_IN;
                    bot_stall_endpoints();
                }
                break;

            case CMD_U_PREVT_REMOVE:
            case CMD_U_VERIFY10:
            case CMD_U_START_STOP:
            case CMD_U_SYNC_CACHE:
                bot_set_sense(SENSE_KEY_NO_ERROR, SENSE_ASC_NO_ERROR, CSW_STATUS_PASSED);
                break;

            default:
#ifdef DEBUG_USB
                log_printf("SCSI: unsupported cmd 0x%02X\r\n", mBOC.mCBW.mCBW_CB_Buf[0]);
#endif
                bot_set_sense(SENSE_KEY_ILLEGAL_REQUEST, SENSE_ASC_INVALID_COMMAND, CSW_STATUS_FAILED);
                Udisk_Transfer_Status |= BOT_FLAG_DATA_IN;
                bot_stall_endpoints();
                break;
        }
    }
    else
    {
        bot_set_sense(SENSE_KEY_ILLEGAL_REQUEST, SENSE_ASC_INVALID_COMMAND, CSW_STATUS_PHASE_ERROR);
        Udisk_Transfer_Status |= BOT_FLAG_DATA_IN;
        Udisk_Transfer_Status |= BOT_FLAG_DATA_OUT;
        bot_stall_endpoints();
    }
}

void bot_handle_bulk_in(void)
{
    if (Udisk_Transfer_Status & BOT_FLAG_DATA_IN) {
        if (mBOC.mCBW.mCBW_CB_Buf[0] == CMD_U_READ10)
            UDISK_InPackflag = 1;
        else
            bot_send_response_data();
    } else if (Udisk_Transfer_Status & BOT_FLAG_CSW_PENDING) {
        bot_send_csw();
    }
}

void bot_handle_bulk_out(uint8_t *pbuf, uint16_t packlen)
{
    uint32_t i;

    if (Udisk_Transfer_Status & BOT_FLAG_DATA_OUT) {
        UDISK_OutPackflag = 1;
    } else {
        if (packlen == CBW_SIZE) {
            for (i = 0; i < packlen; i++)
                mBOC.buf[i] = *pbuf++;

            bot_dispatch_scsi();

            if ((Udisk_Transfer_Status & BOT_FLAG_DATA_OUT) == 0x00) {
                if (Udisk_Transfer_Status & BOT_FLAG_DATA_IN) {
                    if (mBOC.mCBW.mCBW_CB_Buf[0] == CMD_U_READ10)
                        UDISK_InPackflag = 1;
                    else
                        bot_send_response_data();
                } else if (Udisk_CSW_Status == CSW_STATUS_PASSED) {
                    bot_send_csw();
                }
            }
        }
    }
}

void bot_send_response_data(void)
{
    uint32_t len;

    if (UDISK_Transfer_DataLen > UDISK_Pack_Size) {
        len = UDISK_Pack_Size;
        UDISK_Transfer_DataLen -= UDISK_Pack_Size;
    } else {
        len = UDISK_Transfer_DataLen;
        UDISK_Transfer_DataLen = 0x00;
        Udisk_Transfer_Status &= ~BOT_FLAG_DATA_IN;
    }

    if (g_DeviceUsbType == USB_U20_SPEED) {
        memcpy(endp1Tbuff, pEndp2_Buf, len);
        R16_UEP1_T_LEN = len;
        R32_UEP1_TX_DMA = (uint32_t)(uint8_t *)endp1Tbuff;
        R8_UEP1_TX_CTRL = (R8_UEP1_TX_CTRL & ~RB_UEP_TRES_MASK) | UEP_T_RES_ACK;
    } else {
        memcpy(endp1Tbuff, pEndp2_Buf, len);
        USBSS->UEP1_TX_DMA = (uint32_t)(uint8_t *)endp1Tbuff;
        USB30_IN_set(ENDP_1, ENABLE, ACK, DEF_ENDP1_IN_BURST_LEVEL, len);
        USB30_send_ERDY(ENDP_1 | IN, DEF_ENDP1_IN_BURST_LEVEL);
    }
}

void bot_send_csw(void)
{
    Udisk_Transfer_Status = 0x00;
#ifdef DEBUG_USB
    log_printf("CSW: sta=%d\r\n", Udisk_CSW_Status);
#endif

    mBOC.mCSW.mCSW_Sig[0] = 'U';
    mBOC.mCSW.mCSW_Sig[1] = 'S';
    mBOC.mCSW.mCSW_Sig[2] = 'B';
    mBOC.mCSW.mCSW_Sig[3] = 'S';
    mBOC.mCSW.mCSW_Tag[0] = Udisk_CBW_Tag_Save[0];
    mBOC.mCSW.mCSW_Tag[1] = Udisk_CBW_Tag_Save[1];
    mBOC.mCSW.mCSW_Tag[2] = Udisk_CBW_Tag_Save[2];
    mBOC.mCSW.mCSW_Tag[3] = Udisk_CBW_Tag_Save[3];
    mBOC.mCSW.mCSW_Residue[0] = 0x00;
    mBOC.mCSW.mCSW_Residue[1] = 0x00;
    mBOC.mCSW.mCSW_Residue[2] = 0x00;
    mBOC.mCSW.mCSW_Residue[3] = 0x00;
    mBOC.mCSW.mCSW_Status = Udisk_CSW_Status;

    if (g_DeviceUsbType == USB_U20_SPEED) {
        memcpy(endp1Tbuff, (uint8_t *)mBOC.buf, CSW_SIZE);
        R16_UEP1_T_LEN = CSW_SIZE;
        R32_UEP1_TX_DMA = (uint32_t)(uint8_t *)endp1Tbuff;
        R8_UEP1_TX_CTRL = (R8_UEP1_TX_CTRL & ~RB_UEP_TRES_MASK) | UEP_T_RES_ACK;
    } else {
        memcpy(endp1Tbuff, (uint8_t *)mBOC.buf, CSW_SIZE);
        USBSS->UEP1_TX_DMA = (uint32_t)(uint8_t *)endp1Tbuff;
        USB30_IN_set(ENDP_1, ENABLE, ACK, DEF_ENDP1_IN_BURST_LEVEL, CSW_SIZE);
        USB30_send_ERDY(ENDP_1 | IN, DEF_ENDP1_IN_BURST_LEVEL);
    }
}

void bot_poll(void)
{
    if (UDISK_InPackflag == 1) {
        UDISK_InPackflag = 0;
        msc_read_sectors();
    }

    if (UDISK_OutPackflag == 1) {
        UDISK_OutPackflag = 0;
        msc_write_sectors();

        if (g_DeviceUsbType == USB_U20_SPEED) {
            R8_UEP1_RX_CTRL = (R8_UEP1_RX_CTRL & ~RB_UEP_RRES_MASK) | UEP_R_RES_ACK;
        } else {
            USB30_OUT_set(ENDP_1, ACK, DEF_ENDP1_OUT_BURST_LEVEL);
            USB30_send_ERDY(ENDP_1 | OUT, DEF_ENDP1_OUT_BURST_LEVEL);
        }
    }

    ovrd_poll();
}
