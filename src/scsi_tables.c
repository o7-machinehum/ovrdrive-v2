/* scsi_tables.c - SCSI response data tables
 * Capacity fields are overridden at runtime with actual SD card values.
 * SPDX-License-Identifier: Apache-2.0
 */
#include "scsi_tables.h"

uint8_t g_inquiry_response[] =
{
    0x00, 0x80, 0x02, 0x02, 0x1F, 0x00, 0x00, 0x00,
    'H', 'y', 'd', 'r', 'a', 'U', 'S', 'B',   /* Vendor */
    'U', 'S', 'B', '3', ' ', 'M', 'S', 'C',   /* Product */
    ' ', ' ', ' ', ' ',
    'D', 'i', 's', 'k',                         /* Revision */
    ' ', 'W', 'C', 'H'
};

/* Bytes 4-7 overridden at runtime with actual capacity */
uint8_t const g_format_capacity_response[] =
{
    0x00, 0x00, 0x00, 0x08,
    (MY_UDISK_SIZE >> 24) & 0xFF,
    (MY_UDISK_SIZE >> 16) & 0xFF,
    (MY_UDISK_SIZE >> 8) & 0xFF,
    (MY_UDISK_SIZE) & 0xFF,
    0x02,
    (SECTOR_SIZE >> 16) & 0xFF,
    (SECTOR_SIZE >> 8) & 0xFF,
    (SECTOR_SIZE) & 0xFF,
};

/* Bytes 0-3 overridden at runtime with (capacity - 1) */
uint8_t const g_read_capacity_response[] =
{
    ((MY_UDISK_SIZE - 1) >> 24) & 0xFF,
    ((MY_UDISK_SIZE - 1) >> 16) & 0xFF,
    ((MY_UDISK_SIZE - 1) >> 8) & 0xFF,
    ((MY_UDISK_SIZE - 1)) & 0xFF,
    (SECTOR_SIZE >> 24) & 0xFF,
    (SECTOR_SIZE >> 16) & 0xFF,
    (SECTOR_SIZE >> 8) & 0xFF,
    (SECTOR_SIZE) & 0xFF,
};

/* Bytes 4-7 overridden at runtime */
uint8_t const g_mode_sense6_response[] =
{
    0x0B, 0x00,
    0x00,  /* write-unprotected */
    0x08,
    ((MY_UDISK_SIZE - 1) >> 24) & 0xFF,
    ((MY_UDISK_SIZE - 1) >> 16) & 0xFF,
    ((MY_UDISK_SIZE - 1) >> 8) & 0xFF,
    ((MY_UDISK_SIZE - 1)) & 0xFF,
    (SECTOR_SIZE >> 24) & 0xFF,
    (SECTOR_SIZE >> 16) & 0xFF,
    (SECTOR_SIZE >> 8) & 0xFF,
    (SECTOR_SIZE) & 0xFF,
};

/* Bytes 8-11 overridden at runtime */
uint8_t const g_mode_sense10_response[] =
{
    0x00, 0x0E, 0x00,
    0x00,  /* write-unprotected */
    0x00, 0x00, 0x00, 0x08,
    ((MY_UDISK_SIZE - 1) >> 24) & 0xFF,
    ((MY_UDISK_SIZE - 1) >> 16) & 0xFF,
    ((MY_UDISK_SIZE - 1) >> 8) & 0xFF,
    ((MY_UDISK_SIZE - 1)) & 0xFF,
    (SECTOR_SIZE >> 24) & 0xFF,
    (SECTOR_SIZE >> 16) & 0xFF,
    (SECTOR_SIZE >> 8) & 0xFF,
    (SECTOR_SIZE) & 0xFF,
};
