/* bot_state.h - BOT protocol state, constants, and DMA buffers
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef BOT_STATE_H_
#define BOT_STATE_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "CH56x_common.h"
#include "CH56x_emmc.h"

#define SECTOR_SIZE             512
#define CBW_SIZE                31
#define CSW_SIZE                13
#define INQUIRY_RESPONSE_SIZE   36
#define UDISK_BUF_SIZE          (1024 * 4 * 10)  /* 40KB = 80 sectors */

/* SCSI Sense Keys / ASC */
#define SENSE_KEY_NO_ERROR              0x00
#define SENSE_ASC_NO_ERROR              0x00
#define SENSE_KEY_NOT_READY             0x02
#define SENSE_ASC_MEDIUM_NOT_PRESENT    0x3A
#define SENSE_KEY_MEDIUM_ERROR          0x03
#define SENSE_ASC_WRITE_ERROR           0x0C
#define SENSE_KEY_ILLEGAL_REQUEST       0x05
#define SENSE_ASC_INVALID_COMMAND       0x20

/* CSW Status */
#define CSW_STATUS_PASSED       0x00
#define CSW_STATUS_FAILED       0x01
#define CSW_STATUS_PHASE_ERROR  0x02

/* BOT transfer flags */
#define BOT_FLAG_DATA_IN        0x01
#define BOT_FLAG_DATA_OUT       0x02
#define BOT_FLAG_CSW_PENDING    0x04
#define BOT_FLAG_DEVICE_READY   0x01

/* eMMC transfer mode bits [CH569DS1.PDF: R32_EMMC_TRAN_MODE] */
#define EMMC_TRAN_AUTOGAPSTOP   (1 << 4)
#define EMMC_TRAN_GAP_STOP      (1 << 1)

/* MSC class requests */
#define CMD_UDISK_RESET         0xFF
#define CMD_UDISK_GET_MAX_LUN   0xFE

/* SCSI opcodes */
#define CMD_U_TEST_READY            0x00
#define CMD_U_REZERO_UNIT           0x01
#define CMD_U_REQUEST_SENSE         0x03
#define CMD_U_FORMAT_UNIT           0x04
#define CMD_U_INQUIRY               0x12
#define CMD_U_MODE_SELECT           0x15
#define CMD_U_RELEASE               0x17
#define CMD_U_MODE_SENSE            0x1A
#define CMD_U_START_STOP            0x1B
#define CMD_U_SEND_DIAG             0x1D
#define CMD_U_PREVT_REMOVE          0x1E
#define CMD_U_READ_FORMAT_CAPACITY  0x23
#define CMD_U_READ_CAPACITY         0x25
#define CMD_U_READ10                0x28
#define CMD_U_WRITE10               0x2A
#define CMD_U_SEEK10                0x2B
#define CMD_U_WR_VERIFY10           0x2E
#define CMD_U_VERIFY10              0x2F
#define CMD_U_SYNC_CACHE            0x35
#define CMD_U_READ_TOC              0x43
#define CMD_U_MODE_SENSE2           0x5A
#define CMD_U_READ12                0xA8
#define CMD_U_WRITE12               0xAA

/* Placeholder capacity (overridden at runtime) */
#define MY_UDISK_SIZE  0x00000040

/* CBW/CSW/Sense overlay union */
typedef union _BULK_ONLY_CMD {
    uint8_t buf[CBW_SIZE];
    struct {
        uint8_t mCBW_Sig[4];
        uint8_t mCBW_Tag[4];
        uint8_t mCBW_DataLen[4];
        uint8_t mCBW_Flag;
        uint8_t mCBW_LUN;
        uint8_t mCBW_CB_Len;
        uint8_t mCBW_CB_Buf[16];
    } mCBW;
    struct {
        uint8_t mCSW_Sig[4];
        uint8_t mCSW_Tag[4];
        uint8_t mCSW_Residue[4];
        uint8_t mCSW_Status;
    } mCSW;
    struct {
        uint8_t ErrorCode;
        uint8_t Reserved1;
        uint8_t SenseKey;
        uint8_t Information[4];
        uint8_t SenseLength;
        uint8_t Reserved2[4];
        uint8_t SenseCode;
        uint8_t SenseCodeQua;
        uint8_t Reserved3[4];
    } ReqSense;
} BULK_ONLY_CMD;

typedef struct {
    volatile uint8_t  device_ready;
    volatile uint8_t  transfer_flags;
    volatile uint32_t capacity;
    volatile uint8_t  cbw_tag[4];
    volatile uint8_t  sense_key;
    volatile uint8_t  sense_asc;
    volatile uint8_t  csw_status;
    volatile uint32_t transfer_bytes_left;
    volatile uint32_t current_lba;
    volatile uint16_t sectors_done;
    volatile uint16_t pack_size;
    volatile uint8_t  read_pending;      /* ISR -> main loop flag */
    volatile uint8_t  write_pending;     /* ISR -> main loop flag */
} bot_state_t;

extern bot_state_t   g_bot;
extern BULK_ONLY_CMD g_cbw_csw;
extern uint8_t      *g_response_ptr;
extern EMMC_PARAMETER TF_EMMCParam;

extern __attribute__((aligned(16))) uint8_t UDisk_In_Buf[UDISK_BUF_SIZE]
    __attribute__((section(".DMADATA")));
extern __attribute__((aligned(16))) uint8_t UDisk_Out_Buf[UDISK_BUF_SIZE]
    __attribute__((section(".DMADATA")));

void EMMC_IRQHandler(void) __attribute__((interrupt));

/* Backward-compat aliases for BSP ISRs and internal modules */
#define UDISKSIZE                   UDISK_BUF_SIZE
#define DEF_CFG_DISK_SEC_SIZE       SECTOR_SIZE
#define DEF_UDISK_SECTOR_SIZE       SECTOR_SIZE
#define DEF_UDISK_PACK_512          SECTOR_SIZE
#define DEF_UDISK_PACK_64           64

#define DEF_UDISK_EN_FLAG           BOT_FLAG_DEVICE_READY
#define DEF_UDISK_BLUCK_UP_FLAG     BOT_FLAG_DATA_IN
#define DEF_UDISK_BLUCK_DOWN_FLAG   BOT_FLAG_DATA_OUT
#define DEF_UDISK_CSW_UP_FLAG       BOT_FLAG_CSW_PENDING

#define Udisk_Status                g_bot.device_ready
#define Udisk_Transfer_Status       g_bot.transfer_flags
#define Udisk_Capability            g_bot.capacity
#define Udisk_CBW_Tag_Save          g_bot.cbw_tag
#define Udisk_Sense_Key             g_bot.sense_key
#define Udisk_Sense_ASC             g_bot.sense_asc
#define Udisk_CSW_Status            g_bot.csw_status
#define UDISK_Transfer_DataLen      g_bot.transfer_bytes_left
#define UDISK_Cur_Sec_Lba           g_bot.current_lba
#define UDISK_Sec_Pack_Count        g_bot.sectors_done
#define UDISK_Pack_Size             g_bot.pack_size
#define UDISK_InPackflag            g_bot.read_pending
#define UDISK_OutPackflag           g_bot.write_pending
#define mBOC                        g_cbw_csw
#define pEndp2_Buf                  g_response_ptr

#ifdef __cplusplus
}
#endif

#endif /* BOT_STATE_H_ */
