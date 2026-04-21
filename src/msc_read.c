/* msc_read.c - MSC READ10: streaming circular buffer from eMMC to USB
 * SPDX-License-Identifier: Apache-2.0
 */
#include "msc_read.h"
#include "emmc_ops.h"
#include "msc_diag.h"
#include "CH56x_usb30_devbulk.h"
#include "CH56x_usb20_devbulk.h"
#include "CH56x_usb30_devbulk_LIB.h"

extern void bot_send_csw(void);

static void read_stream_usb2(uint32_t actual_lba, uint16_t preqnum)
{
    uint16_t sdtran = 0, usbtran = 0;
    uint8_t sdstep = 0, usbstep = 0;
    uint8_t lock = 0, flag = 1;
    read_diag_t diag;

    uint8_t uep0rxsave = R8_UEP0_RX_CTRL;
    uint8_t uep0txsave = R8_UEP0_TX_CTRL;
    PFIC_DisableIRQ(USBHS_IRQn);
    R8_UEP0_TX_CTRL = UEP_T_RES_NAK;
    R8_UEP0_RX_CTRL = UEP_R_RES_NAK;
    R8_UEP1_TX_CTRL |= RB_UEP_T_AUTOTOG;

    diag_read_init(&diag, actual_lba);
    emmc_start_multiblock_read(UDisk_In_Buf, actual_lba, preqnum);

    /* Interleaved eMMC DMA read + USB2 TX polling loop */
    uint16_t xfer_count = 0;
    while (1)
    {
        /* NAK immediately on transfer complete to prevent stale DMA race */
        if (R8_USB_INT_FG & RB_USB_IF_TRANSFER)
            R8_UEP1_TX_CTRL = (R8_UEP1_TX_CTRL & ~RB_UEP_TRES_MASK) | UEP_T_RES_NAK;

        if ((sdtran > 1 && usbtran < sdtran - 1) &&
            ((R8_USB_INT_FG & RB_USB_IF_TRANSFER) || flag))
        {
            if (R8_USB_INT_FG & RB_USB_IF_TRANSFER) xfer_count++;
            R8_USB_INT_FG = RB_USB_IF_TRANSFER;
            flag = 0;

            diag_check_usb_sector(&diag, UDisk_In_Buf + usbstep * SECTOR_SIZE, usbtran);

            R32_UEP1_TX_DMA = (uint32_t)(uint8_t *)(UDisk_In_Buf + usbstep * SECTOR_SIZE);
            R16_UEP1_T_LEN = SECTOR_SIZE;
            __asm__ volatile("fence" ::: "memory");
            R8_UEP1_TX_CTRL = (R8_UEP1_TX_CTRL & ~RB_UEP_TRES_MASK) | UEP_T_RES_ACK;

            usbtran++;
            usbstep++;
            if (usbstep == UDISK_BUF_SIZE / SECTOR_SIZE) usbstep = 0;
            if (lock) {
                lock = 0;
                emmc_release_gap_stop();
            }
        }

        if (R8_USB_INT_FG & RB_USB_IF_SETUOACT)
            R8_USB_INT_FG = RB_USB_IF_SETUOACT;

        if (R16_EMMC_INT_FG & RB_EMMC_IF_BKGAP)
        {
            if (ovrd_state == STATE_UNLOCKED)
                ovrd_crypt_buf(UDisk_In_Buf + sdstep * SECTOR_SIZE, actual_lba + sdtran, 1);

            diag_check_sd_sector(&diag, UDisk_In_Buf + sdstep * SECTOR_SIZE, sdtran);

            R16_EMMC_INT_FG = RB_EMMC_IF_BKGAP;
            sdtran++;
            sdstep++;
            if (sdstep == UDISK_BUF_SIZE / SECTOR_SIZE) sdstep = 0;
            R32_EMMC_DMA_BEG1 = (uint32_t)(uint8_t *)(UDisk_In_Buf + sdstep * SECTOR_SIZE);

            if ((sdtran - usbtran) < ((UDISK_BUF_SIZE / SECTOR_SIZE) - 2))
                emmc_release_gap_stop();
            else
                lock = 1;
        }
        else if (R16_EMMC_INT_FG & RB_EMMC_IF_TRANDONE)
        {
            if (ovrd_state == STATE_UNLOCKED)
                ovrd_crypt_buf(UDisk_In_Buf + sdstep * SECTOR_SIZE, actual_lba + sdtran, 1);

            diag_check_sd_sector(&diag, UDisk_In_Buf + sdstep * SECTOR_SIZE, sdtran);

            R16_EMMC_INT_FG = RB_EMMC_IF_TRANDONE | RB_EMMC_IF_CMDDONE;
            sdtran++;
            sdstep++;
            break;
        }
    }

    /* Drain remaining USB sends after eMMC is done */
    while (1)
    {
        if (R8_USB_INT_FG & RB_USB_IF_TRANSFER)
            R8_UEP1_TX_CTRL = (R8_UEP1_TX_CTRL & ~RB_UEP_TRES_MASK) | UEP_T_RES_NAK;

        if ((R8_USB_INT_FG & RB_USB_IF_TRANSFER) || flag)
        {
            xfer_count++;
            R8_USB_INT_FG = RB_USB_IF_TRANSFER;
            flag = 0;

            diag_check_usb_sector(&diag, UDisk_In_Buf + usbstep * SECTOR_SIZE, usbtran);

            R32_UEP1_TX_DMA = (uint32_t)(uint8_t *)(UDisk_In_Buf + usbstep * SECTOR_SIZE);
            R16_UEP1_T_LEN = SECTOR_SIZE;
            __asm__ volatile("fence" ::: "memory");
            R8_UEP1_TX_CTRL = (R8_UEP1_TX_CTRL & ~RB_UEP_TRES_MASK) | UEP_T_RES_ACK;

            usbtran++;
            usbstep++;
            if (usbstep == UDISK_BUF_SIZE / SECTOR_SIZE) usbstep = 0;
            if (usbtran == sdtran)
            {
                while (!(R8_USB_INT_FG & RB_USB_IF_TRANSFER));
                R8_USB_INT_FG = RB_USB_IF_TRANSFER;
                R8_UEP1_TX_CTRL = (R8_UEP1_TX_CTRL & ~RB_UEP_TRES_MASK) | UEP_T_RES_NAK;
                break;
            }
        }

        if (R8_USB_INT_FG & RB_USB_IF_SETUOACT)
            R8_USB_INT_FG = RB_USB_IF_SETUOACT;
    }

    emmc_stop_multiblock_read();
    diag_read_summary(&diag, preqnum, xfer_count, usbtran);

    R8_UEP1_TX_CTRL &= ~RB_UEP_T_AUTOTOG;
    PFIC_EnableIRQ(USBHS_IRQn);
    R8_UEP0_TX_CTRL = uep0txsave;
    R8_UEP0_RX_CTRL = uep0rxsave;
}

static void read_stream_usb3(uint32_t actual_lba, uint16_t preqnum)
{
    uint16_t sdtran = 0, usbtran = 0;
    uint8_t sdstep = 0, usbstep = 0;
    uint8_t lock = 0, flag = 1;

    PFIC_DisableIRQ(USBSS_IRQn);
    emmc_start_multiblock_read(UDisk_In_Buf, actual_lba, preqnum);

    while (1)
    {
        if ((sdtran > 1 && usbtran < sdtran - 1) &&
            ((USBSS->UEP1_TX_CTRL & ((uint32_t)1 << 31)) || flag))
        {
            USB30_IN_clearIT(ENDP_1);
            USBSS->UEP1_TX_DMA = (uint32_t)(uint8_t *)(UDisk_In_Buf + usbstep * SECTOR_SIZE);
            USB30_IN_set(ENDP_1, ENABLE, ACK, 1, 1024);
            USB30_send_ERDY(ENDP_1 | IN, 1);
            usbtran += 2;
            usbstep += 2;
            if (usbstep == UDISK_BUF_SIZE / SECTOR_SIZE) usbstep = 0;
            flag = 0;
            if (lock) {
                lock = 0;
                emmc_release_gap_stop();
            }
        }

        if (R16_EMMC_INT_FG & RB_EMMC_IF_BKGAP)
        {
            R16_EMMC_INT_FG = RB_EMMC_IF_BKGAP;
            if (ovrd_state == STATE_UNLOCKED)
                ovrd_crypt_buf(UDisk_In_Buf + sdstep * SECTOR_SIZE, actual_lba + sdtran, 1);
            sdtran++;
            sdstep++;
            if (sdstep == UDISK_BUF_SIZE / SECTOR_SIZE) sdstep = 0;
            R32_EMMC_DMA_BEG1 = (uint32_t)(uint8_t *)(UDisk_In_Buf + sdstep * SECTOR_SIZE);
            if ((sdtran - usbtran) < ((UDISK_BUF_SIZE / SECTOR_SIZE) - 2))
                emmc_release_gap_stop();
            else
                lock = 1;
        }
        else if (R16_EMMC_INT_FG & RB_EMMC_IF_TRANDONE)
        {
            R16_EMMC_INT_FG = RB_EMMC_IF_TRANDONE | RB_EMMC_IF_CMDDONE;
            if (ovrd_state == STATE_UNLOCKED)
                ovrd_crypt_buf(UDisk_In_Buf + sdstep * SECTOR_SIZE, actual_lba + sdtran, 1);
            sdtran++;
            sdstep++;
            break;
        }
    }

    /* Drain remaining USB3 sends */
    while (1)
    {
        if ((USBSS->UEP1_TX_CTRL & ((uint32_t)1 << 31)) || flag)
        {
            flag = 0;
            USB30_IN_clearIT(ENDP_1);
            if ((sdtran - usbtran) > 1)
            {
                USBSS->UEP1_TX_DMA = (uint32_t)(uint8_t *)(UDisk_In_Buf + usbstep * SECTOR_SIZE);
                USB30_IN_set(ENDP_1, ENABLE, ACK, 1, 1024);
                USB30_send_ERDY(ENDP_1 | IN, 1);
                usbtran += 2;
                usbstep += 2;
            }
            else
            {
                USBSS->UEP1_TX_DMA = (uint32_t)(uint8_t *)(UDisk_In_Buf + usbstep * SECTOR_SIZE);
                USB30_IN_set(ENDP_1, ENABLE, ACK, 1, SECTOR_SIZE);
                USB30_send_ERDY(ENDP_1 | IN, 1);
                usbtran++;
                usbstep++;
            }
            if (usbstep == UDISK_BUF_SIZE / SECTOR_SIZE) usbstep = 0;
            if (usbtran == sdtran)
            {
                while (!(USBSS->UEP1_TX_CTRL & ((uint32_t)1 << 31)));
                USB30_IN_clearIT(ENDP_1);
                break;
            }
        }
    }

    emmc_stop_multiblock_read();
    PFIC_EnableIRQ(USBSS_IRQn);
}

void msc_read_sectors(void)
{
    uint16_t preqnum = UDISK_Transfer_DataLen / SECTOR_SIZE;
    UDISK_Transfer_DataLen = 0;

    uint32_t actual_lba = compute_physical_lba(UDISK_Cur_Sec_Lba);

    cprintf("R lba=%lu n=%u\r\n", actual_lba, preqnum);

    if (g_DeviceUsbType == USB_U20_SPEED)
        read_stream_usb2(actual_lba, preqnum);
    else
        read_stream_usb3(actual_lba, preqnum);

    UDISK_Cur_Sec_Lba += preqnum;

    if (UDISK_Transfer_DataLen == 0)
        bot_send_csw();

    if (UDISK_Transfer_DataLen == 0x00)
        Udisk_Transfer_Status &= ~BOT_FLAG_DATA_IN;
}
