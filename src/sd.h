/********************************** (C) COPYRIGHT *******************************
 * File Name          : sd.h
 * Description        : SD card initialization driver header
 *
 * This module handles SD card initialization using the CH569's eMMC controller.
 * It implements the SD-specific init sequence (as opposed to the eMMC sequence
 * in CH56x_emmc.c) because SD cards use a different command flow:
 *   - CMD55+ACMD41 instead of CMD1 for OCR (SD vs eMMC distinction)
 *   - CMD3 returns RCA (SD assigns) vs CMD3 sets RCA (eMMC host assigns)
 *   - ACMD6 for bus width (SD) vs CMD6 with different args (eMMC)
 *   - ACMD51 for SCR register (SD-specific)
 *
 * [ref/SD_Physical_Layer_Spec_v6.00.pdf: Section 4.2 - Card Initialization]
 *******************************************************************************/
#ifndef __SD_H__
#define __SD_H__

#include "CH56x_common.h"
#include "CH56x_emmc.h"

/* Full SD card init: tries 8 IO modes (clock phase/edge/drive combinations),
 * returns OP_SUCCESS if any mode initializes the card successfully. */
uint8_t SDCardInit(PSD_PARAMETER pEMMCPara);

/* Individual init steps (called internally by SDCardInit): */
uint8_t SDReadOCR(PSD_PARAMETER pEMMCPara);       /* ACMD41 - negotiate voltage/capacity */
uint8_t SDSetRCA(PSD_PARAMETER pEMMCPara);         /* CMD3 - get relative card address */
uint8_t SDReadCSD(PSD_PARAMETER pEMMCPara);        /* CMD9 - read Card Specific Data */
uint8_t SDSetBusWidth(PSD_PARAMETER pEMMCPara, uint8_t bus_mode);  /* ACMD6 - set 1/4-bit */
uint8_t SD_ReadSCR(PSD_PARAMETER pEMMCPara, uint8_t *pRdatbuf);   /* ACMD51 - read SCR */

#endif
