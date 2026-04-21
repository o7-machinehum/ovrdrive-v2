/* emmc_ops.h - Shared eMMC DMA inline helpers
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef EMMC_OPS_H_
#define EMMC_OPS_H_

#include "bot_state.h"
#include "ovrdrive.h"

static inline uint32_t compute_physical_lba(uint32_t host_lba)
{
    if (ovrd_state == STATE_UNLOCKED)
        return host_lba + LOCKED_SECTORS;
    return host_lba;
}

/* Start CMD18 READ_MULTIPLE_BLOCK with AUTOGAPSTOP */
static inline void emmc_start_multiblock_read(uint8_t *buf_start,
                                               uint32_t lba,
                                               uint16_t num_sectors)
{
    PFIC_DisableIRQ(EMMC_IRQn);
    R16_EMMC_INT_FG = 0xffff;
    R32_EMMC_DMA_BEG1 = (uint32_t)buf_start;
    R32_EMMC_TRAN_MODE = EMMC_TRAN_AUTOGAPSTOP | EMMC_TRAN_GAP_STOP;
    R32_EMMC_BLOCK_CFG = (SECTOR_SIZE) << 16 | num_sectors;

    uint32_t cmd_arg = lba;
    uint16_t cmd_set = RB_EMMC_CKIDX | RB_EMMC_CKCRC | RESP_TYPE_48 | EMMC_CMD18;
    EMMCSendCmd(cmd_arg, cmd_set);
}

/* CMD12 STOP_TRANSMISSION — required after CMD18 */
static inline void emmc_stop_multiblock_read(void)
{
    uint8_t s;
    R32_EMMC_TRAN_MODE = 0;
    EMMCSendCmd(0, RB_EMMC_CKIDX | RB_EMMC_CKCRC | RESP_TYPE_R1b | EMMC_CMD12);
    while (1) {
        s = CheckCMDComp(&TF_EMMCParam);
        if (s != CMD_NULL) break;
    }
    R16_EMMC_INT_FG = 0xffff;
    TF_EMMCParam.EMMCOpErr = 0;
    PFIC_EnableIRQ(EMMC_IRQn);
}

/* Release gap stop so eMMC DMA continues to next block */
static inline void emmc_release_gap_stop(void)
{
    __asm__ volatile("fence" ::: "memory");
    R32_EMMC_TRAN_MODE = (uint32_t)EMMC_TRAN_AUTOGAPSTOP;
}

/* Advance circular buffer DMA pointer. Returns 1 if buffer nearly full. */
static inline int emmc_advance_dma(uint8_t *buf_base,
                                    uint8_t *sdstep_ptr,
                                    uint16_t sdtran,
                                    uint16_t usbtran)
{
    uint8_t sdstep = *sdstep_ptr;
    sdstep++;
    if (sdstep == UDISK_BUF_SIZE / SECTOR_SIZE) sdstep = 0;
    *sdstep_ptr = sdstep;
    R32_EMMC_DMA_BEG1 = (uint32_t)(buf_base + sdstep * SECTOR_SIZE);

    if ((sdtran - usbtran) < ((UDISK_BUF_SIZE / SECTOR_SIZE) - 2)) {
        emmc_release_gap_stop();
        return 0;
    }
    return 1;
}

#endif /* EMMC_OPS_H_ */
