#ifndef __SD_H__
#define __SD_H__

#include "CH56x_common.h"
#include "CH56x_emmc.h"

uint8_t SDCardInit(PSD_PARAMETER pEMMCPara);
uint8_t SDReadOCR(PSD_PARAMETER pEMMCPara);
uint8_t SDSetRCA(PSD_PARAMETER pEMMCPara);
uint8_t SDReadCSD(PSD_PARAMETER pEMMCPara);
uint8_t SDSetBusWidth(PSD_PARAMETER pEMMCPara, uint8_t bus_mode);
uint8_t SD_ReadSCR(PSD_PARAMETER pEMMCPara, uint8_t *pRdatbuf);

#endif
