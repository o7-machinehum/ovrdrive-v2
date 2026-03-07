#include "sd.h"

static __attribute__((aligned(8))) uint8_t scr_buf[512] __attribute__((section(".DMADATA")));

uint8_t SDReadOCR(PSD_PARAMETER pEMMCPara)
{
	uint8_t  i;
	uint32_t cmd_arg_val;
	uint16_t cmd_set_val;
	uint8_t  sta = 0;
	uint32_t cmd_rsp_val;

	for(i = 0; i < 100; i++)
	{
		/* CMD55 - APP_CMD prefix */
		cmd_arg_val = 0;
		cmd_set_val = RB_EMMC_CKIDX | RB_EMMC_CKCRC | RESP_TYPE_48 | 55;
		EMMCSendCmd(cmd_arg_val, cmd_set_val);
		mDelayuS(2);
		while(1)
		{
			sta = CheckCMDComp(pEMMCPara);
			if(sta != CMD_NULL) break;
		}
		if(sta == CMD_FAILED)
		{
			mDelaymS(20);
			continue;
		}

		/* ACMD41 - SD_SEND_OP_COND */
		cmd_arg_val = 0x40FF8000;
		cmd_set_val = RESP_TYPE_48 | 41;
		EMMCSendCmd(cmd_arg_val, cmd_set_val);
		mDelayuS(2);
		while(1)
		{
			sta = CheckCMDComp(pEMMCPara);
			if(sta != CMD_NULL) break;
		}
		if(sta == CMD_SUCCESS)
		{
			cmd_rsp_val = R32_EMMC_RESPONSE3;
			if(cmd_rsp_val & (1<<31))
			{
				if(cmd_rsp_val & (1<<30))
					pEMMCPara->EMMCType = EMMCIO_HIGH_CAPACITY_SD_CARD;
				else
					pEMMCPara->EMMCType = EMMCIO_CAPACITY_SD_CARD_V2_0;
				break;
			}
		}
		mDelaymS(20);
	}
	if(i == 100) return OP_FAILED;
	return sta;
}

uint8_t SDSetRCA(PSD_PARAMETER pEMMCPara)
{
	uint32_t cmd_arg_val;
	uint16_t cmd_set_val;
	uint8_t  sta;

	cmd_arg_val = 0;
	cmd_set_val = RB_EMMC_CKIDX | RB_EMMC_CKCRC | RESP_TYPE_48 | EMMC_CMD3;
	EMMCSendCmd(cmd_arg_val, cmd_set_val);
	while(1)
	{
		sta = CheckCMDComp(pEMMCPara);
		if(sta != CMD_NULL) break;
	}
	if(sta == CMD_SUCCESS)
	{
		pEMMCPara->EMMC_RCA = R32_EMMC_RESPONSE3 >> 16;
	}
	return sta;
}

/* Read the CSD (Card specific data)*/
uint8_t SDReadCSD(PSD_PARAMETER pEMMCPara)
{
	uint32_t cmd_arg_val;
	uint16_t cmd_set_val;
	uint8_t  sta;
	uint32_t disk_block_num = 0;

	cmd_arg_val = pEMMCPara->EMMC_RCA << 16;
	cmd_set_val = RESP_TYPE_136 | EMMC_CMD9;
	EMMCSendCmd(cmd_arg_val, cmd_set_val);
	while(1)
	{
		sta = CheckCMDComp(pEMMCPara);
		if(sta != CMD_NULL) break;
	}

	if(sta == CMD_SUCCESS)
	{
		pEMMCPara->EMMC_CSD[0] = R32_EMMC_RESPONSE0;
		pEMMCPara->EMMC_CSD[1] = R32_EMMC_RESPONSE1;
		pEMMCPara->EMMC_CSD[2] = R32_EMMC_RESPONSE2;
		pEMMCPara->EMMC_CSD[3] = R32_EMMC_RESPONSE3;

		if(pEMMCPara->EMMC_CSD[3] >> 30) /* SDHC/SDXC */
		{
			pEMMCPara->EMMCType = EMMCIO_HIGH_CAPACITY_SD_CARD;
			disk_block_num = ((((pEMMCPara->EMMC_CSD[2] & 0x0ff) << 16) |
				((pEMMCPara->EMMC_CSD[1] & 0xffff0000) >> 16)) + 1) << 10;
		}
		else /* SDSC */
		{
			pEMMCPara->EMMCType = EMMCIO_CAPACITY_SD_CARD_V2_0;
			disk_block_num = (((pEMMCPara->EMMC_CSD[2] & 0x3ff) << 2) |
				(pEMMCPara->EMMC_CSD[1] >> 30)) + 1;
			disk_block_num = disk_block_num << (((pEMMCPara->EMMC_CSD[1] >> 15) & 0x07) + 2);
		}
	}
	pEMMCPara->EMMCSecNum = disk_block_num;
	pEMMCPara->EMMCSecSize = 1 << ((pEMMCPara->EMMC_CSD[2] >> 16) & 0x000f);
	return sta;
}

uint8_t SDSetBusWidth(PSD_PARAMETER pEMMCPara, uint8_t bus_mode)
{
	uint32_t cmd_arg_val;
	uint16_t cmd_set_val;
	uint8_t  sta;

	/* CMD55 */
	cmd_arg_val = (pEMMCPara->EMMC_RCA) << 16;
	cmd_set_val = RB_EMMC_CKIDX | RB_EMMC_CKCRC | RESP_TYPE_48 | 55;
	EMMCSendCmd(cmd_arg_val, cmd_set_val);
	while(1)
	{
		sta = CheckCMDComp(pEMMCPara);
		if(sta != CMD_NULL) break;
	}
	if(sta == CMD_SUCCESS)
	{
		/* ACMD6 - SET_BUS_WIDTH */
		if(bus_mode == 0)
			cmd_arg_val = 0x0; /* 1-bit */
		else
			cmd_arg_val = 0x2; /* 4-bit */

		cmd_set_val = RB_EMMC_CKIDX | RB_EMMC_CKCRC | RESP_TYPE_48 | EMMC_CMD6;
		EMMCSendCmd(cmd_arg_val, cmd_set_val);
		while(1)
		{
			sta = CheckCMDComp(pEMMCPara);
			if(sta != CMD_NULL) break;
		}
	}
	return sta;
}

uint8_t SD_ReadSCR(PSD_PARAMETER pEMMCPara, uint8_t *pRdatbuf)
{
	uint32_t cmd_arg_val;
	uint16_t cmd_set_val, t;
	uint8_t  sta;

	/* CMD55 */
	cmd_arg_val = (pEMMCPara->EMMC_RCA) << 16;
	cmd_set_val = RB_EMMC_CKIDX | RB_EMMC_CKCRC | RESP_TYPE_48 | 55;
	EMMCSendCmd(cmd_arg_val, cmd_set_val);
	while(1)
	{
		sta = CheckCMDComp(pEMMCPara);
		if(sta != CMD_NULL) break;
	}

	if(sta == CMD_SUCCESS)
	{
		R32_EMMC_DMA_BEG1 = (uint32_t)pRdatbuf;
		R32_EMMC_TRAN_MODE = 0;
		R32_EMMC_BLOCK_CFG = 8 << 16 | 1;

		/* ACMD51 - SEND_SCR */
		cmd_arg_val = 0;
		cmd_set_val = RB_EMMC_CKIDX | RB_EMMC_CKCRC | RESP_TYPE_48 | 51;
		EMMCSendCmd(cmd_arg_val, cmd_set_val);
		while(1)
		{
			sta = CheckCMDComp(pEMMCPara);
			if(sta != CMD_NULL) break;
		}
		while(1)
		{
			if(R16_EMMC_INT_FG & RB_EMMC_IF_TRANDONE) break;
		}
		t = R16_EMMC_INT_FG;
		R16_EMMC_INT_FG = t;
	}

	return sta;
}

/*******************************************************************************
 * SDCardInit - Full SD card initialization sequence
 *   CMD0 -> CMD8 -> ACMD41 -> CMD2 -> CMD3 -> CMD9 -> CMD7 -> ACMD6 -> ACMD51
 *   Returns OP_SUCCESS or OP_FAILED
 ******************************************************************************/
/*******************************************************************************
 * SD_IO_init - Configure EMMC controller for SD card with given IO mode
 *   Tries different sampling/phase/drive combinations
 ******************************************************************************/
static void SD_IO_init(uint8_t mode)
{
	/* GPIO configuration */
	R32_PB_PU  |= bSDCMD;
	R32_PB_PU  |= (0x1f<<17);
	R32_PA_PU  |= (7<<0);
	R32_PB_DIR |= bSDCK;

	if(mode >= 1 && mode <= 5)
	{
		R32_PA_DRV |= (7<<0);
		R32_PB_DRV |= (0x1f<<17);
		R32_PB_DRV |= bSDCMD;
		R32_PB_DRV |= bSDCK;
	}

	/* Controller reset */
	R8_EMMC_CONTROL = RB_EMMC_ALL_CLR | RB_EMMC_RST_LGC;

	if(mode == 0 || mode == 1 || mode == 6 || mode == 7)
		R8_EMMC_CONTROL = RB_EMMC_DMAEN;
	else
		R8_EMMC_CONTROL = RB_EMMC_DMAEN | RB_EMMC_NEGSMP;

	/* 1-bit mode for init */
	R8_EMMC_CONTROL = (R8_EMMC_CONTROL & ~RB_EMMC_LW_MASK) | bLW_OP_DAT0;

	if(mode < 4)
		R16_EMMC_CLK_DIV = RB_EMMC_CLKOE | RB_EMMC_PHASEINV | LOWEMMCCLK;
	else
		R16_EMMC_CLK_DIV = RB_EMMC_CLKOE | LOWEMMCCLK;

	/* Enable error interrupts */
	R16_EMMC_INT_FG = 0xffff;
	R16_EMMC_INT_EN = RB_EMMC_IE_FIFO_OV |
					  RB_EMMC_IE_TRANERR |
					  RB_EMMC_IE_DATTMO |
					  RB_EMMC_IE_REIDX_ER |
					  RB_EMMC_IE_RECRC_WR |
					  RB_EMMC_IE_RE_TMOUT;

	R8_EMMC_TIMEOUT = 14;
}

/*******************************************************************************
 * SD_do_init - SD card init sequence for a given IO mode
 ******************************************************************************/
static uint8_t SD_do_init(PSD_PARAMETER pEMMCPara, uint8_t mode)
{
	uint8_t sta;
	uint8_t i;

	/* CMD0 - GO_IDLE_STATE */
	EMMCResetIdle(pEMMCPara);
	mDelaymS(30);
	EMMCResetIdle(pEMMCPara);
	mDelaymS(30);

	/* CMD8 - SEND_IF_COND (voltage check) */
	for(i = 0; i < 3; i++)
	{
		pEMMCPara->EMMCOpErr = 0;
		EMMCSendCmd(0x01AA, RB_EMMC_CKIDX | RB_EMMC_CKCRC | RESP_TYPE_48 | EMMC_CMD8);
		while(1)
		{
			sta = CheckCMDComp(pEMMCPara);
			if(sta != CMD_NULL) break;
		}
		if(sta == CMD_SUCCESS) break;
		mDelaymS(30);
	}
	if(sta != CMD_SUCCESS) return OP_FAILED;

	/* ACMD41 - SD_SEND_OP_COND */
	sta = SDReadOCR(pEMMCPara);
	if(sta != CMD_SUCCESS) return OP_FAILED;

	/* CMD2 - ALL_SEND_CID */
	sta = EMMCReadCID(pEMMCPara);
	if(sta != CMD_SUCCESS) return OP_FAILED;

	/* CMD3 - SEND_RELATIVE_ADDR */
	sta = SDSetRCA(pEMMCPara);
	if(sta != CMD_SUCCESS) return OP_FAILED;

	/* CMD9 - SEND_CSD */
	sta = SDReadCSD(pEMMCPara);
	if(sta != CMD_SUCCESS) return OP_FAILED;

	/* CMD7 - SELECT_CARD */
	mDelaymS(5);
	sta = SelectEMMCCard(pEMMCPara);
	if(sta != CMD_SUCCESS) return OP_FAILED;

	/* ACMD6 - SET_BUS_WIDTH to 4-bit */
	sta = SDSetBusWidth(pEMMCPara, 1);
	if(sta != CMD_SUCCESS) return OP_FAILED;
	R8_EMMC_CONTROL = (R8_EMMC_CONTROL & ~RB_EMMC_LW_MASK) | bLW_OP_DAT4;

	/* ACMD51 - Read SCR */
	sta = SD_ReadSCR(pEMMCPara, scr_buf);
	if(sta != CMD_SUCCESS) return OP_FAILED;

	/* Switch to high-speed clock (divider: lower = faster, 10 = ~12MHz) */
	if(mode < 4)
		R16_EMMC_CLK_DIV = RB_EMMC_CLKMode | RB_EMMC_PHASEINV | RB_EMMC_CLKOE | 10;
		// R16_EMMC_CLK_DIV = RB_EMMC_CLKMode | RB_EMMC_PHASEINV | RB_EMMC_CLKOE | LOWEMMCCLK; /* Slow */
	else
		R16_EMMC_CLK_DIV = RB_EMMC_CLKMode | RB_EMMC_CLKOE | 10;
		// R16_EMMC_CLK_DIV = RB_EMMC_CLKMode | RB_EMMC_CLKOE | LOWEMMCCLK; /* Slow */

	return OP_SUCCESS;
}

/*******************************************************************************
 * SDCardInit - Try all 8 IO modes until one works
 ******************************************************************************/
uint8_t SDCardInit(PSD_PARAMETER pEMMCPara)
{
	uint8_t i, sta;

	for(i = 0; i < 8; i++)
	{
		log_printf("SD: trying IO mode %d\r\n", i);
		SD_IO_init(i);
		pEMMCPara->EMMCOpErr = 0;
		sta = SD_do_init(pEMMCPara, i);
		if(sta == OP_SUCCESS)
		{
			log_printf("SD: mode %d OK\r\n", i);
			return OP_SUCCESS;
		}
	}

	return OP_FAILED;
}
