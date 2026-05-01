/* msc_diag.h - Read/write path diagnostic helpers
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef MSC_DIAG_H_
#define MSC_DIAG_H_

#include "bot_state.h"
#include "CH56x_debug_log.h"

typedef struct {
    uint32_t start_lba;
    uint16_t sd_err, usb_err;
    uint32_t sd_first_exp, sd_first_got;
    uint32_t usb_first_exp, usb_first_got;
} read_diag_t;

static inline void diag_read_init(read_diag_t *d, uint32_t start_lba)
{
    d->start_lba = start_lba;
    d->sd_err = 0;
    d->usb_err = 0;
    d->sd_first_exp = 0;
    d->sd_first_got = 0;
    d->usb_first_exp = 0;
    d->usb_first_got = 0;
}

static inline void diag_check_sd_sector(read_diag_t *d, uint8_t *buf,
                                         uint16_t sdtran)
{
    uint32_t exp = d->start_lba + sdtran;
    uint32_t got = *(volatile uint32_t*)buf;
    if (got != exp) {
        if (!d->sd_err) { d->sd_first_exp = exp; d->sd_first_got = got; }
        d->sd_err++;
    }
}

static inline void diag_check_usb_sector(read_diag_t *d, uint8_t *buf,
                                          uint16_t usbtran)
{
    uint32_t exp = d->start_lba + usbtran;
    uint32_t got = *(volatile uint32_t*)buf;
    if (got != exp) {
        if (!d->usb_err) { d->usb_first_exp = exp; d->usb_first_got = got; }
        d->usb_err++;
    }
}

static inline void diag_log_read_summary(read_diag_t *d, uint16_t preqnum,
                                      uint16_t xfer_count, uint16_t usbtran)
{
    if (d->sd_err || d->usb_err)
        cprintf("R ERR sd=%u usb=%u sdE=%08lx/%08lx usbE=%08lx/%08lx\r\n",
                d->sd_err, d->usb_err,
                d->sd_first_exp, d->sd_first_got,
                d->usb_first_exp, d->usb_first_got);
    else
        cprintf("R OK n=%u xfer=%u usbtran=%u\r\n", preqnum, xfer_count, usbtran);
}

static inline uint32_t diag_sector_checksum(uint8_t *sector_buf)
{
    uint32_t ck = 0;
    int i;
    for (i = 0; i < 128; i++)
        ck ^= ((uint32_t*)sector_buf)[i];
    return ck;
}

static inline void diag_write_checksums_usb2(uint8_t *buf, uint16_t chunk_sectors)
{
    uint32_t ckfirst = diag_sector_checksum(buf);
    uint32_t cksecond = 0;
    if (chunk_sectors > 1)
        cksecond = diag_sector_checksum(buf + SECTOR_SIZE);
    uint32_t cklast = diag_sector_checksum(buf + (chunk_sectors - 1) * SECTOR_SIZE);
    cprintf("W ck0=%08lx ck1=%08lx ck%u=%08lx\r\n",
            ckfirst, cksecond, chunk_sectors - 1, cklast);
}

static inline void diag_write_checksums_usb3(uint8_t *buf, uint16_t chunk_sectors)
{
    uint32_t ckfirst = diag_sector_checksum(buf);
    uint32_t cklast = diag_sector_checksum(buf + (chunk_sectors - 1) * SECTOR_SIZE);
    cprintf("W ck0=%08lx ck%u=%08lx\r\n", ckfirst, chunk_sectors - 1, cklast);
}

#endif /* MSC_DIAG_H_ */
