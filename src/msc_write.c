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

#ifdef DEBUG
typedef struct {
    uint64_t usb_cycles;
    uint64_t snoop_cycles;
    uint64_t crypt_cycles;
    uint64_t sd_cycles;
    uint64_t dat0_cycles;
    uint32_t total_start;
    uint32_t chunks;
    uint16_t sectors;
} write_profile_t;

static write_profile_t write_prof;

static inline uint32_t write_prof_now(void)
{
    /* SysTick counts down, so invert the low word to get an increasing counter. */
    return ~bsp_get_SysTickCNT_LSB();
}

static inline uint32_t write_prof_elapsed(uint32_t start)
{
    return write_prof_now() - start;
}

static uint32_t write_prof_us(uint64_t cycles)
{
    uint64_t cycles_per_us = bsp_get_nbtick_1us();
    if (cycles_per_us == 0)
        return 0;
    return (uint32_t)(cycles / cycles_per_us);
}

static void write_prof_reset(uint16_t sectors)
{
    memset(&write_prof, 0, sizeof(write_prof));
    EMMCWriteProfileReset();
    write_prof.sectors = sectors;
    write_prof.total_start = write_prof_now();
}

static void write_prof_log(void)
{
    uint32_t total_cycles = write_prof_elapsed(write_prof.total_start);

    cprintf("Wprof u%u sec=%u ch=%lu total=%luus usb=%luus snoop=%luus crypt=%luus sd=%luus dat0=%luus\r\n",
            (g_DeviceUsbType == USB_U20_SPEED) ? 2 : 3,
            write_prof.sectors,
            (unsigned long)write_prof.chunks,
            (unsigned long)write_prof_us(total_cycles),
            (unsigned long)write_prof_us(write_prof.usb_cycles),
            (unsigned long)write_prof_us(write_prof.snoop_cycles),
            (unsigned long)write_prof_us(write_prof.crypt_cycles),
            (unsigned long)write_prof_us(write_prof.sd_cycles),
            (unsigned long)write_prof_us(write_prof.dat0_cycles));

    cprintf("Eprof calls=%lu req=%lu done=%lu bkgap=%lu cmd24=%lu/%luus cmd25=%lu/%luus data=%lu/%luus cmd12=%lu/%luus err=%lu/%lu/%lu/%lu/%lu\r\n",
            (unsigned long)emmc_write_prof.calls,
            (unsigned long)emmc_write_prof.req_sectors,
            (unsigned long)emmc_write_prof.done_sectors,
            (unsigned long)emmc_write_prof.bkgaps,
            (unsigned long)write_prof_us(emmc_write_prof.cmd24_cycles),
            (unsigned long)write_prof_us(emmc_write_prof.cmd24_max_cycles),
            (unsigned long)write_prof_us(emmc_write_prof.cmd25_cycles),
            (unsigned long)write_prof_us(emmc_write_prof.cmd25_max_cycles),
            (unsigned long)write_prof_us(emmc_write_prof.data_cycles),
            (unsigned long)write_prof_us(emmc_write_prof.data_max_cycles),
            (unsigned long)write_prof_us(emmc_write_prof.cmd12_cycles),
            (unsigned long)write_prof_us(emmc_write_prof.cmd12_max_cycles),
            (unsigned long)emmc_write_prof.invalid_addr_errors,
            (unsigned long)emmc_write_prof.cmd24_errors,
            (unsigned long)emmc_write_prof.cmd25_errors,
            (unsigned long)emmc_write_prof.data_errors,
            (unsigned long)emmc_write_prof.cmd12_errors);

    cprintf("Ewait bkgap=%lu/%luus trdone=%lu/%luus\r\n",
            (unsigned long)write_prof_us(emmc_write_prof.bkgap_wait_cycles),
            (unsigned long)write_prof_us(emmc_write_prof.bkgap_wait_max_cycles),
            (unsigned long)write_prof_us(emmc_write_prof.trandone_wait_cycles),
            (unsigned long)write_prof_us(emmc_write_prof.trandone_wait_max_cycles));
}
#endif

static uint8_t write_chunk_to_sd(uint8_t *buf, uint32_t write_lba, uint16_t chunk_sectors)
{
    uint16_t reqnum;
    uint8_t s;

    if (ovrd_state == STATE_UNLOCKED) {
#ifdef DEBUG
        uint32_t prof_start = write_prof_now();
#endif
        ovrd_crypt_buf(buf, write_lba, chunk_sectors);
#ifdef DEBUG
        write_prof.crypt_cycles += write_prof_elapsed(prof_start);
#endif
    }

#ifdef DEBUG
    uint32_t prof_start = write_prof_now();
#endif
    PFIC_DisableIRQ(EMMC_IRQn);
    R16_EMMC_INT_FG = 0xffff;
    TF_EMMCParam.EMMCOpErr = 0;
    TF_EMMCParam.EMMCSecSize = SECTOR_SIZE;
    reqnum = chunk_sectors;
    if (ovrd_state == STATE_UNLOCKED && chunk_sectors == 1 && write_lba >= LOCKED_SECTORS)
        s = EMMCCardWriteOneSec(&TF_EMMCParam, &reqnum, buf, write_lba);
    else
        s = EMMCCardWriteMulSec(&TF_EMMCParam, &reqnum, buf, write_lba);
    R16_EMMC_INT_FG = 0xffff;
    TF_EMMCParam.EMMCOpErr = 0;
    PFIC_EnableIRQ(EMMC_IRQn);
#ifdef DEBUG
    write_prof.sd_cycles += write_prof_elapsed(prof_start);
#endif

#ifdef DEBUG_USB
    cprintf("W s=%u req=%u act=%u\r\n", s, chunk_sectors, reqnum);
#endif
    return s;
}

static uint8_t *alternate_write_buf(uint8_t *buf)
{
    return (buf == UDisk_Out_Buf) ? UDisk_In_Buf : UDisk_Out_Buf;
}

static void receive_chunk_usb2(uint8_t *buf, uint16_t chunk_sectors, uint32_t lba, uint8_t *first)
{
    uint16_t sectors_received = 0;
#ifdef DEBUG
    uint32_t prof_start = write_prof_now();
#endif

    R8_USB_INT_FG = RB_USB_IF_TRANSFER;
#ifdef DEBUG_USB
    cprintf("W lba=%lu n=%u\r\n", lba, chunk_sectors);
#endif

    /* First sector already in endp1Rbuff from USB ISR */
    if (*first) {
        memcpy(buf, endp1Rbuff, SECTOR_SIZE);
        sectors_received = 1;
        *first = 0;
    }

    /* Receive remaining sectors via USB polling */
    while (sectors_received < chunk_sectors)
    {
        R32_UEP1_RX_DMA = (uint32_t)(buf + sectors_received * SECTOR_SIZE);
        __asm__ volatile("fence" ::: "memory");
        R8_UEP1_RX_CTRL = (R8_UEP1_RX_CTRL & ~RB_UEP_RRES_MASK) | UEP_R_RES_ACK;

        while (!(R8_USB_INT_FG & RB_USB_IF_TRANSFER)) {
            if (R8_USB_INT_FG & RB_USB_IF_SETUOACT)
                R8_USB_INT_FG = RB_USB_IF_SETUOACT;
        }
#ifdef DEBUG_USB
        {
            uint8_t ist = R8_USB_INT_ST;
            if ((ist & 0x0F) != 1)
                cprintf("W !EP%u tok=%u rx=%u\r\n", ist & 0x0F, (ist >> 4) & 3, sectors_received);
        }
#endif
        R8_USB_INT_FG = RB_USB_IF_TRANSFER;
        R8_UEP1_RX_CTRL = (R8_UEP1_RX_CTRL & ~RB_UEP_RRES_MASK) | UEP_R_RES_NAK;
        sectors_received++;
    }

#ifdef DEBUG
    write_prof.usb_cycles += write_prof_elapsed(prof_start);
#endif
#ifdef DEBUG_USB
    diag_write_checksums_usb2(buf, chunk_sectors);
#endif
}

static void receive_chunk_usb3(uint8_t *buf, uint16_t chunk_sectors, uint32_t lba, uint8_t *first)
{
    uint16_t sectors_received = 0;
#ifdef DEBUG
    uint32_t prof_start = write_prof_now();
#endif

#ifdef DEBUG_USB
    cprintf("W lba=%lu n=%u\r\n", lba, chunk_sectors);
#endif

    /* First burst (up to 2 sectors) already in endp1Rbuff */
    if (*first) {
        uint16_t first_count = (chunk_sectors >= 2) ? 2 : 1;
        memcpy(buf, endp1Rbuff, first_count * SECTOR_SIZE);
        sectors_received = first_count;
        *first = 0;
    }

    /* Receive remaining sectors in 1024-byte bursts */
    while (sectors_received < chunk_sectors)
    {
        uint16_t this_burst;

        USBSS->UEP1_RX_DMA = (uint32_t)(buf + sectors_received * SECTOR_SIZE);
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

#ifdef DEBUG
    write_prof.usb_cycles += write_prof_elapsed(prof_start);
#endif
#ifdef DEBUG_USB
    diag_write_checksums_usb3(buf, chunk_sectors);
#endif
}

static uint8_t write_received_chunk(uint8_t *buf, uint32_t lba, uint16_t chunk_sectors)
{
#ifdef DEBUG
    uint32_t prof_start = write_prof_now();
    write_prof.chunks++;
#endif
    ovrd_snoop_write(buf, (uint32_t)chunk_sectors * SECTOR_SIZE);
#ifdef DEBUG
    write_prof.snoop_cycles += write_prof_elapsed(prof_start);
#endif
    return write_chunk_to_sd(buf, compute_physical_lba(lba), chunk_sectors);
}

static void wait_dat0_ready(void)
{
#ifdef DEBUG
    uint32_t prof_start = write_prof_now();
#endif
    while (!EMMCDat0Sta);
#ifdef DEBUG
    write_prof.dat0_cycles += write_prof_elapsed(prof_start);
#endif
}

static void write_stream_usb2(uint16_t total_sectors, uint32_t lba)
{
    uint16_t sectors_left = total_sectors;
    uint16_t buf_sectors = UDISK_BUF_SIZE / SECTOR_SIZE;
    uint16_t chunk_sectors;
    uint8_t *buf = UDisk_Out_Buf;
    uint8_t first = 1;

    uint8_t uep0rxsave = R8_UEP0_RX_CTRL;
    uint8_t uep0txsave = R8_UEP0_TX_CTRL;
    PFIC_DisableIRQ(USBHS_IRQn);
    R8_UEP0_TX_CTRL = UEP_T_RES_NAK;
    R8_UEP0_RX_CTRL = UEP_R_RES_NAK;
    R8_UEP1_RX_CTRL |= RB_UEP_R_AUTOTOG;
    R8_USB_INT_FG = RB_USB_IF_TRANSFER;

    if (sectors_left > 0)
    {
        chunk_sectors = (sectors_left > buf_sectors) ? buf_sectors : sectors_left;
        receive_chunk_usb2(buf, chunk_sectors, lba, &first);

        while (sectors_left > 0)
        {
            uint8_t s = write_received_chunk(buf, lba, chunk_sectors);
            if (s != CMD_SUCCESS) {
                bot_set_sense(SENSE_KEY_MEDIUM_ERROR, SENSE_ASC_WRITE_ERROR, CSW_STATUS_FAILED);
                break;
            }

            lba += chunk_sectors;
            sectors_left -= chunk_sectors;
            if (sectors_left == 0) {
                wait_dat0_ready();
                break;
            }

            buf = alternate_write_buf(buf);
            chunk_sectors = (sectors_left > buf_sectors) ? buf_sectors : sectors_left;
            receive_chunk_usb2(buf, chunk_sectors, lba, &first);

            wait_dat0_ready();
        }
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
    uint16_t chunk_sectors;
    uint8_t *buf = UDisk_Out_Buf;
    uint8_t first = 1;

    PFIC_DisableIRQ(USBSS_IRQn);

    if (sectors_left > 0)
    {
        chunk_sectors = (sectors_left > buf_sectors) ? buf_sectors : sectors_left;
        receive_chunk_usb3(buf, chunk_sectors, lba, &first);

        while (sectors_left > 0)
        {
            uint8_t s = write_received_chunk(buf, lba, chunk_sectors);
            if (s != CMD_SUCCESS) {
                bot_set_sense(SENSE_KEY_MEDIUM_ERROR, SENSE_ASC_WRITE_ERROR, CSW_STATUS_FAILED);
                break;
            }

            lba += chunk_sectors;
            sectors_left -= chunk_sectors;
            if (sectors_left == 0) {
                wait_dat0_ready();
                break;
            }

            buf = alternate_write_buf(buf);
            chunk_sectors = (sectors_left > buf_sectors) ? buf_sectors : sectors_left;
            receive_chunk_usb3(buf, chunk_sectors, lba, &first);

            wait_dat0_ready();
        }
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

#ifdef DEBUG
    write_prof_reset(total_sectors);
#endif

    if (g_DeviceUsbType == USB_U20_SPEED)
        write_stream_usb2(total_sectors, lba);
    else
        write_stream_usb3(total_sectors, lba);

#ifdef DEBUG
    write_prof_log();
#endif

    if (UDISK_Transfer_DataLen == 0) {
        Udisk_Transfer_Status &= ~BOT_FLAG_DATA_OUT;
        bot_send_csw();
    }
}
