/* msc_write.c - MSC WRITE10: two-phase USB receive then SD write
 * SPDX-License-Identifier: Apache-2.0
 */
#include "msc_write.h"
#include "emmc_ops.h"
#include "msc_diag.h"
#include "CH56x_usb30_devbulk.h"
#include "CH56x_usb20_devbulk.h"
#include "CH56x_usb30_devbulk_LIB.h"
#include <string.h>

extern void bot_set_sense(uint8_t key, uint8_t asc, uint8_t status);
extern void bot_send_csw(void);

static uint8_t write_chunk_to_sd(uint32_t write_lba, uint16_t chunk_sectors)
{
    uint16_t reqnum;
    uint8_t s;

    if (ovrd_state == STATE_UNLOCKED)
        ovrd_crypt_buf(UDisk_Out_Buf, write_lba, chunk_sectors);

    PFIC_DisableIRQ(EMMC_IRQn);
    R16_EMMC_INT_FG = 0xffff;
    TF_EMMCParam.EMMCOpErr = 0;
    TF_EMMCParam.EMMCSecSize = SECTOR_SIZE;
    reqnum = chunk_sectors;
    s = EMMCCardWriteMulSec(&TF_EMMCParam, &reqnum, UDisk_Out_Buf, write_lba);
    R16_EMMC_INT_FG = 0xffff;
    TF_EMMCParam.EMMCOpErr = 0;
    PFIC_EnableIRQ(EMMC_IRQn);

    cprintf("W s=%u req=%u act=%u\r\n", s, chunk_sectors, reqnum);
    return s;
}

static void write_stream_usb2(uint16_t total_sectors, uint32_t lba)
{
    uint16_t sectors_left = total_sectors;
    uint16_t buf_sectors = UDISK_BUF_SIZE / SECTOR_SIZE;
    uint8_t first = 1;

    uint8_t uep0rxsave = R8_UEP0_RX_CTRL;
    uint8_t uep0txsave = R8_UEP0_TX_CTRL;
    PFIC_DisableIRQ(USBHS_IRQn);
    R8_UEP0_TX_CTRL = UEP_T_RES_NAK;
    R8_UEP0_RX_CTRL = UEP_R_RES_NAK;
    R8_UEP1_RX_CTRL |= RB_UEP_R_AUTOTOG;
    R8_USB_INT_FG = RB_USB_IF_TRANSFER;

    while (sectors_left > 0)
    {
        uint16_t chunk_sectors = (sectors_left > buf_sectors) ? buf_sectors : sectors_left;
        uint16_t sectors_received = 0;

        R8_USB_INT_FG = RB_USB_IF_TRANSFER;
#ifdef DEBUG_USB
        cprintf("W lba=%lu n=%u\r\n", lba, chunk_sectors);
#endif

        /* First sector already in endp1Rbuff from USB ISR */
        if (first) {
            memcpy(UDisk_Out_Buf, endp1Rbuff, SECTOR_SIZE);
            sectors_received = 1;
            first = 0;
        }

        /* Receive remaining sectors via USB polling */
        while (sectors_received < chunk_sectors)
        {
            R32_UEP1_RX_DMA = (uint32_t)(UDisk_Out_Buf + sectors_received * SECTOR_SIZE);
            __asm__ volatile("fence" ::: "memory");
            R8_UEP1_RX_CTRL = (R8_UEP1_RX_CTRL & ~RB_UEP_RRES_MASK) | UEP_R_RES_ACK;

            while (!(R8_USB_INT_FG & RB_USB_IF_TRANSFER)) {
                if (R8_USB_INT_FG & RB_USB_IF_SETUOACT)
                    R8_USB_INT_FG = RB_USB_IF_SETUOACT;
            }
            {
                uint8_t ist = R8_USB_INT_ST;
                if ((ist & 0x0F) != 1)
                    cprintf("W !EP%u tok=%u rx=%u\r\n", ist & 0x0F, (ist >> 4) & 3, sectors_received);
            }
            R8_USB_INT_FG = RB_USB_IF_TRANSFER;
            R8_UEP1_RX_CTRL = (R8_UEP1_RX_CTRL & ~RB_UEP_RRES_MASK) | UEP_R_RES_NAK;
            sectors_received++;
        }

        diag_write_checksums_usb2(UDisk_Out_Buf, chunk_sectors);
        ovrd_snoop_write(UDisk_Out_Buf, (uint32_t)chunk_sectors * SECTOR_SIZE);

        {
            uint32_t write_lba = lba;
            if (ovrd_state == STATE_UNLOCKED)
                write_lba = lba + LOCKED_SECTORS;

            uint8_t s = write_chunk_to_sd(write_lba, chunk_sectors);
            if (s != CMD_SUCCESS) {
                bot_set_sense(SENSE_KEY_MEDIUM_ERROR, SENSE_ASC_WRITE_ERROR, CSW_STATUS_FAILED);
                break;
            }
        }

        while (!EMMCDat0Sta);

        lba += chunk_sectors;
        sectors_left -= chunk_sectors;
    }

    R32_UEP1_RX_DMA = (uint32_t)(uint8_t *)endp1Rbuff;
    R8_UEP1_RX_CTRL &= ~RB_UEP_R_AUTOTOG;
    PFIC_EnableIRQ(USBHS_IRQn);
    R8_UEP0_TX_CTRL = uep0txsave;
    R8_UEP0_RX_CTRL = uep0rxsave;
}

static void write_stream_usb3(uint16_t total_sectors, uint32_t lba)
{
    uint16_t sectors_left = total_sectors;
    uint16_t buf_sectors = UDISK_BUF_SIZE / SECTOR_SIZE;
    uint8_t first = 1;

    PFIC_DisableIRQ(USBSS_IRQn);

    while (sectors_left > 0)
    {
        uint16_t chunk_sectors = (sectors_left > buf_sectors) ? buf_sectors : sectors_left;
        uint16_t sectors_received = 0;

#ifdef DEBUG_USB
        cprintf("W lba=%lu n=%u\r\n", lba, chunk_sectors);
#endif

        /* First burst (up to 2 sectors) already in endp1Rbuff */
        if (first) {
            uint16_t first_count = (chunk_sectors >= 2) ? 2 : 1;
            memcpy(UDisk_Out_Buf, endp1Rbuff, first_count * SECTOR_SIZE);
            sectors_received = first_count;
            first = 0;
        }

        /* Receive remaining sectors in 1024-byte bursts */
        while (sectors_received < chunk_sectors)
        {
            uint16_t this_burst;

            USBSS->UEP1_RX_DMA = (uint32_t)(UDisk_Out_Buf + sectors_received * SECTOR_SIZE);
            USB30_OUT_set(ENDP_1, ACK, 1);
            USB30_send_ERDY(ENDP_1 | OUT, 1);

            while (!(USBSS->UEP1_RX_CTRL & ((uint32_t)1 << 26)));
            USB30_OUT_clearIT(ENDP_1);
            USB30_OUT_set(ENDP_1, NRDY, 0);

            this_burst = 2;
            if (sectors_received + this_burst > chunk_sectors)
                this_burst = chunk_sectors - sectors_received;
            sectors_received += this_burst;
        }

        diag_write_checksums_usb3(UDisk_Out_Buf, chunk_sectors);
        ovrd_snoop_write(UDisk_Out_Buf, (uint32_t)chunk_sectors * SECTOR_SIZE);

        {
            uint32_t write_lba = lba;
            if (ovrd_state == STATE_UNLOCKED)
                write_lba = lba + LOCKED_SECTORS;

            uint8_t s = write_chunk_to_sd(write_lba, chunk_sectors);
            if (s != CMD_SUCCESS) {
                bot_set_sense(SENSE_KEY_MEDIUM_ERROR, SENSE_ASC_WRITE_ERROR, CSW_STATUS_FAILED);
                break;
            }
        }

        while (!EMMCDat0Sta);

        lba += chunk_sectors;
        sectors_left -= chunk_sectors;
    }

    USBSS->UEP1_RX_DMA = (uint32_t)(uint8_t *)endp1Rbuff;
    USB30_OUT_clearIT(ENDP_1);
    PFIC_EnableIRQ(USBSS_IRQn);
}

void msc_write_sectors(void)
{
    uint16_t total_sectors = UDISK_Transfer_DataLen / SECTOR_SIZE;
    UDISK_Transfer_DataLen = 0;
    uint32_t lba = UDISK_Cur_Sec_Lba;

    if (g_DeviceUsbType == USB_U20_SPEED)
        write_stream_usb2(total_sectors, lba);
    else
        write_stream_usb3(total_sectors, lba);

    if (UDISK_Transfer_DataLen == 0) {
        Udisk_Transfer_Status &= ~BOT_FLAG_DATA_OUT;
        bot_send_csw();
    }
}
