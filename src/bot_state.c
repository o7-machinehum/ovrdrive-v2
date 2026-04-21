/* bot_state.c - BOT state definitions and eMMC IRQ handler
 * SPDX-License-Identifier: Apache-2.0
 */
#include "bot_state.h"

bot_state_t g_bot = { .pack_size = SECTOR_SIZE };
BULK_ONLY_CMD g_cbw_csw;
uint8_t *g_response_ptr;
EMMC_PARAMETER TF_EMMCParam;

__attribute__((aligned(16))) uint8_t UDisk_In_Buf[UDISK_BUF_SIZE]
    __attribute__((section(".DMADATA")));
__attribute__((aligned(16))) uint8_t UDisk_Out_Buf[UDISK_BUF_SIZE]
    __attribute__((section(".DMADATA")));

void EMMC_IRQHandler(void)
{
    uint16_t t = R16_EMMC_INT_FG;
    if(t)
    {
        TF_EMMCParam.EMMCOpErr = 1;
        R16_EMMC_INT_FG = t;
    }
}
