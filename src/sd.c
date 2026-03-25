/********************************** (C) COPYRIGHT *******************************
 * File Name          : sd.c
 * Description        : SD card initialization driver
 *
 * This file implements SD card initialization using the CH569's eMMC controller.
 * The CH569 has a single "eMMC" controller that handles both eMMC and SD cards.
 * SD cards use a different init sequence than eMMC, so this file provides
 * SD-specific versions of OCR reading, RCA assignment, CSD parsing, etc.
 *
 * The init sequence follows the SD Physical Layer Spec (Section 4.2):
 *   CMD0 (reset) → CMD8 (voltage check) → ACMD41 (capacity negotiation) →
 *   CMD2 (read CID) → CMD3 (get RCA) → CMD9 (read CSD) → CMD7 (select) →
 *   ACMD6 (set 4-bit bus) → ACMD51 (read SCR)
 *
 * After init, the card is in Transfer State and ready for data commands
 * (CMD17/18 for read, CMD24/25 for write).
 *
 * Hardware interface:
 *   CLK  = PB15 (bSDCK)          - SD clock output
 *   CMD  = PB14 (bSDCMD)         - SD command line (bidirectional)
 *   DAT0 = PB17                  - SD data line 0
 *   DAT1 = PB18                  - SD data line 1
 *   DAT2 = PB19                  - SD data line 2
 *   DAT3 = PB20                  - SD data line 3
 *   [CH569DS1.PDF: GPIO Pin Multiplexing, eMMC Controller]
 *
 * Clock configuration:
 *   Base clock = 480MHz (from PLL)
 *   LOWEMMCCLK divider = 0x1F (31) → 480/31 ≈ 15MHz
 *   Used during init (slow) and currently for data transfers too
 *   (conservative speed for long-wire setups)
 *   [ref/SD_Physical_Layer_Spec_v6.00.pdf: Section 6 - Clock Control]
 *
 * SPDX-License-Identifier: Apache-2.0
 *******************************************************************************/
#include "sd.h"

/* Scratch buffer for SCR register read (8 bytes needed, 512 allocated for
 * DMA alignment requirements). Must be in RAMX for DMA access.
 * [CH569DS1.PDF: DMA buffers must be in RAMX (.DMADATA section)] */
static __attribute__((aligned(8))) uint8_t scr_buf[512] __attribute__((section(".DMADATA")));

/*******************************************************************************
 * SDReadOCR - Read SD card Operating Conditions Register via ACMD41
 *
 * SD cards use ACMD41 (CMD55 prefix + CMD41) instead of CMD1 (eMMC).
 * This negotiates voltage range and determines card capacity type.
 *
 * The host sends:
 *   CMD55 (APP_CMD prefix, tells card next command is application-specific)
 *   ACMD41 with arg 0x40FF8000:
 *     Bit 30 (HCS=1): Host supports SDHC/SDXC
 *     Bits 23-0: Voltage window (3.2-3.4V range)
 *
 * Card responds with OCR register:
 *   Bit 31: Power-up complete (busy bit, polled until set)
 *   Bit 30: Card Capacity Status (CCS): 1=SDHC/SDXC, 0=SDSC
 *
 * [ref/SD_Physical_Layer_Spec_v6.00.pdf: Section 4.2.3 - ACMD41]
 ******************************************************************************/
uint8_t SDReadOCR(PSD_PARAMETER pEMMCPara)
{
	uint8_t  i;
	uint32_t cmd_arg_val;
	uint16_t cmd_set_val;
	uint8_t  sta = 0;
	uint32_t cmd_rsp_val;

	/* Poll ACMD41 up to 100 times (~2 seconds total with delays).
	 * SD spec requires at most 1 second for power-up. */
	for(i = 0; i < 100; i++)
	{
		/* CMD55 - APP_CMD prefix (required before every ACMD)
		 * [ref/SD_Physical_Layer_Spec_v6.00.pdf: Section 4.3.9 - CMD55] */
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

		/* ACMD41 - SD_SEND_OP_COND
		 * Arg: 0x40FF8000 = HCS bit set + voltage window
		 * No CRC/index check on response (R3 response type) */
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
			/* Read OCR from response register
			 * [CH569DS1.PDF: R32_EMMC_RESPONSE3 holds R3/R1 response] */
			cmd_rsp_val = R32_EMMC_RESPONSE3;
			if(cmd_rsp_val & (1<<31))  /* Bit 31: power-up complete */
			{
				if(cmd_rsp_val & (1<<30))  /* Bit 30: CCS - SDHC/SDXC */
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

/*******************************************************************************
 * SDSetRCA - Get Relative Card Address (CMD3)
 *
 * For SD cards, CMD3 (SEND_RELATIVE_ADDR) causes the card to publish its
 * own RCA. This differs from eMMC where the host assigns the RCA.
 * The RCA is in bits [31:16] of the R6 response.
 *
 * [ref/SD_Physical_Layer_Spec_v6.00.pdf: Section 4.3.1 - CMD3]
 ******************************************************************************/
uint8_t SDSetRCA(PSD_PARAMETER pEMMCPara)
{
	uint32_t cmd_arg_val;
	uint16_t cmd_set_val;
	uint8_t  sta;

	/* For SD cards, CMD3 arg is 0 (card chooses its own RCA) */
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
		/* Extract RCA from upper 16 bits of R6 response */
		pEMMCPara->EMMC_RCA = R32_EMMC_RESPONSE3 >> 16;
	}
	return sta;
}

/*******************************************************************************
 * SDReadCSD - Read Card Specific Data register (CMD9)
 *
 * The CSD register contains card capacity, block sizes, and timing info.
 * It's a 128-bit register returned as an R2 response (RESP_TYPE_136).
 *
 * Two CSD versions exist:
 *   CSD v1.0 (SDSC): Complex capacity calculation using C_SIZE, C_SIZE_MULT,
 *     and READ_BL_LEN fields. Max capacity 2GB.
 *   CSD v2.0 (SDHC/SDXC): Simplified - capacity = (C_SIZE + 1) * 512KB.
 *     C_SIZE is in a different location than v1.0.
 *
 * [ref/SD_Physical_Layer_Spec_v6.00.pdf: Section 5.3 - CSD Register]
 ******************************************************************************/
uint8_t SDReadCSD(PSD_PARAMETER pEMMCPara)
{
	uint32_t cmd_arg_val;
	uint16_t cmd_set_val;
	uint8_t  sta;
	uint32_t disk_block_num = 0;

	/* CMD9 requires RCA in argument bits [31:16] */
	cmd_arg_val = pEMMCPara->EMMC_RCA << 16;
	cmd_set_val = RESP_TYPE_136 | EMMC_CMD9;  /* 136-bit response (R2) */
	EMMCSendCmd(cmd_arg_val, cmd_set_val);
	while(1)
	{
		sta = CheckCMDComp(pEMMCPara);
		if(sta != CMD_NULL) break;
	}

	if(sta == CMD_SUCCESS)
	{
		/* Read 128-bit CSD from four 32-bit response registers
		 * [CH569DS1.PDF: R32_EMMC_RESPONSE0-3 hold R2 response] */
		pEMMCPara->EMMC_CSD[0] = R32_EMMC_RESPONSE0;
		pEMMCPara->EMMC_CSD[1] = R32_EMMC_RESPONSE1;
		pEMMCPara->EMMC_CSD[2] = R32_EMMC_RESPONSE2;
		pEMMCPara->EMMC_CSD[3] = R32_EMMC_RESPONSE3;

		if(pEMMCPara->EMMC_CSD[3] >> 30) /* CSD_STRUCTURE: bits [127:126] */
		{
			/* CSD v2.0 (SDHC/SDXC)
			 * C_SIZE is in bits [69:48], capacity = (C_SIZE+1) * 1024 sectors
			 * Each sector = 512 bytes, so capacity = (C_SIZE+1) * 512KB */
			pEMMCPara->EMMCType = EMMCIO_HIGH_CAPACITY_SD_CARD;
			disk_block_num = ((((pEMMCPara->EMMC_CSD[2] & 0x0ff) << 16) |
				((pEMMCPara->EMMC_CSD[1] & 0xffff0000) >> 16)) + 1) << 10;
		}
		else
		{
			/* CSD v1.0 (SDSC)
			 * Capacity = (C_SIZE+1) * 2^(C_SIZE_MULT+2) * 2^(READ_BL_LEN)
			 * C_SIZE: bits [73:62], C_SIZE_MULT: bits [49:47]
			 * READ_BL_LEN: bits [83:80] */
			pEMMCPara->EMMCType = EMMCIO_CAPACITY_SD_CARD_V2_0;
			disk_block_num = (((pEMMCPara->EMMC_CSD[2] & 0x3ff) << 2) |
				(pEMMCPara->EMMC_CSD[1] >> 30)) + 1;
			disk_block_num = disk_block_num << (((pEMMCPara->EMMC_CSD[1] >> 15) & 0x07) + 2);
		}
	}
	pEMMCPara->EMMCSecNum = disk_block_num;
	/* READ_BL_LEN: bits [83:80] in CSD → sector size (should be 512) */
	pEMMCPara->EMMCSecSize = 1 << ((pEMMCPara->EMMC_CSD[2] >> 16) & 0x000f);
	return sta;
}

/*******************************************************************************
 * SDSetBusWidth - Set SD card data bus width via ACMD6
 *
 * SD cards start in 1-bit mode (DAT0 only). After init, we switch to
 * 4-bit mode (DAT0-DAT3) for 4x data throughput.
 *
 * ACMD6 arg: 0x0 = 1-bit, 0x2 = 4-bit
 * [ref/SD_Physical_Layer_Spec_v6.00.pdf: Section 4.3.10 - ACMD6]
 ******************************************************************************/
uint8_t SDSetBusWidth(PSD_PARAMETER pEMMCPara, uint8_t bus_mode)
{
	uint32_t cmd_arg_val;
	uint16_t cmd_set_val;
	uint8_t  sta;

	/* CMD55 - APP_CMD prefix (required before ACMD6)
	 * Must include RCA in argument for addressed commands */
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
			cmd_arg_val = 0x0; /* 1-bit mode */
		else
			cmd_arg_val = 0x2; /* 4-bit mode */

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

/*******************************************************************************
 * SD_ReadSCR - Read SD Configuration Register via ACMD51
 *
 * The SCR is an SD-specific 64-bit register that contains:
 *   - SD spec version supported
 *   - Data bus widths supported
 *   - Security features
 *
 * Read as a single 8-byte data block via DMA.
 * [ref/SD_Physical_Layer_Spec_v6.00.pdf: Section 5.6 - SCR Register]
 ******************************************************************************/
uint8_t SD_ReadSCR(PSD_PARAMETER pEMMCPara, uint8_t *pRdatbuf)
{
	uint32_t cmd_arg_val;
	uint16_t cmd_set_val, t;
	uint8_t  sta;

	/* CMD55 - APP_CMD prefix */
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
		/* Set up DMA for 8-byte read
		 * [CH569DS1.PDF: R32_EMMC_DMA_BEG1, R32_EMMC_BLOCK_CFG] */
		R32_EMMC_DMA_BEG1 = (uint32_t)pRdatbuf;
		R32_EMMC_TRAN_MODE = 0;              /* Read direction (eMMC→memory) */
		R32_EMMC_BLOCK_CFG = 8 << 16 | 1;    /* Block size=8, count=1 */

		/* ACMD51 - SEND_SCR */
		cmd_arg_val = 0;
		cmd_set_val = RB_EMMC_CKIDX | RB_EMMC_CKCRC | RESP_TYPE_48 | 51;
		EMMCSendCmd(cmd_arg_val, cmd_set_val);
		while(1)
		{
			sta = CheckCMDComp(pEMMCPara);
			if(sta != CMD_NULL) break;
		}
		/* Wait for DMA transfer to complete */
		while(1)
		{
			if(R16_EMMC_INT_FG & RB_EMMC_IF_TRANDONE) break;
		}
		/* Clear all interrupt flags */
		t = R16_EMMC_INT_FG;
		R16_EMMC_INT_FG = t;
	}

	return sta;
}

/*******************************************************************************
 * SD_IO_init - Configure eMMC controller GPIO and registers for SD card
 *
 * The CH569's eMMC controller needs specific GPIO and register setup.
 * Multiple IO modes are tried (different phase/edge/drive combinations)
 * because signal integrity varies with PCB layout and wire length.
 *
 * IO modes 0-7 cycle through combinations of:
 *   - NEGSMP: Sample on negative clock edge (modes 2,3,4,5)
 *   - PHASEINV: Invert clock phase (modes 0,1,2,3)
 *   - Drive strength: High drive for modes 1-5
 *
 * [CH569DS1.PDF: eMMC Controller Registers, GPIO Configuration]
 ******************************************************************************/
static void SD_IO_init(uint8_t mode)
{
	/* GPIO: Enable pull-ups on CMD and DAT lines for proper idle state
	 * SD spec requires pull-ups on CMD, DAT0-DAT3 */
	R32_PB_PU  |= bSDCMD;        /* CMD line pull-up */
	R32_PB_PU  |= (0x1f<<17);    /* DAT0-DAT4 pull-ups (PB17-PB21) */
	R32_PA_PU  |= (7<<0);        /* Additional data lines (PA0-PA2) */
	R32_PB_DIR |= bSDCK;         /* CLK line as output */

	/* High drive strength for modes 1-5 (helps with long wires) */
	if(mode >= 1 && mode <= 5)
	{
		R32_PA_DRV |= (7<<0);
		R32_PB_DRV |= (0x1f<<17);
		R32_PB_DRV |= bSDCMD;
		R32_PB_DRV |= bSDCK;
	}

	/* Reset eMMC controller (clears all internal state)
	 * [CH569DS1.PDF: R8_EMMC_CONTROL - RB_EMMC_ALL_CLR, RB_EMMC_RST_LGC] */
	R8_EMMC_CONTROL = RB_EMMC_ALL_CLR | RB_EMMC_RST_LGC;

	/* Enable DMA engine. NEGSMP (negative-edge sampling) helps with
	 * signal integrity on some configurations. */
	if(mode == 0 || mode == 1 || mode == 6 || mode == 7)
		R8_EMMC_CONTROL = RB_EMMC_DMAEN;
	else
		R8_EMMC_CONTROL = RB_EMMC_DMAEN | RB_EMMC_NEGSMP;

	/* Start in 1-bit mode (DAT0 only) per SD spec init requirements
	 * [ref/SD_Physical_Layer_Spec_v6.00.pdf: Section 4.2 - 1-bit init] */
	R8_EMMC_CONTROL = (R8_EMMC_CONTROL & ~RB_EMMC_LW_MASK) | bLW_OP_DAT0;

	/* Set slow clock for initialization (LOWEMMCCLK = divider 31)
	 * 480MHz / 31 ≈ 15MHz. SD spec requires ≤400kHz for CMD0-ACMD41,
	 * but many cards tolerate higher speeds during init.
	 * PHASEINV inverts the clock phase for some modes. */
	if(mode < 4)
		R16_EMMC_CLK_DIV = RB_EMMC_CLKOE | RB_EMMC_PHASEINV | LOWEMMCCLK;
	else
		R16_EMMC_CLK_DIV = RB_EMMC_CLKOE | LOWEMMCCLK;

	/* Enable error interrupts only (NOT data/command complete).
	 * BKGAP and TRANDONE are polled, not interrupt-driven.
	 * Error interrupts trigger EMMC_IRQHandler which sets EMMCOpErr flag.
	 * [CH569DS1.PDF: R16_EMMC_INT_EN register bits] */
	R16_EMMC_INT_FG = 0xffff;  /* Clear all pending flags first */
	R16_EMMC_INT_EN = RB_EMMC_IE_FIFO_OV |    /* FIFO overflow */
					  RB_EMMC_IE_TRANERR |    /* Transfer error */
					  RB_EMMC_IE_DATTMO |     /* Data timeout */
					  RB_EMMC_IE_REIDX_ER |   /* Response index error */
					  RB_EMMC_IE_RECRC_WR |   /* Response CRC error */
					  RB_EMMC_IE_RE_TMOUT;    /* Response timeout */

	/* Data timeout value (2^14 clock cycles)
	 * [CH569DS1.PDF: R8_EMMC_TIMEOUT] */
	R8_EMMC_TIMEOUT = 14;
}

/*******************************************************************************
 * SD_do_init - Execute full SD card init sequence for a given IO mode
 *
 * Init sequence per SD Physical Layer Spec Section 4.2:
 *
 *   CMD0  (GO_IDLE_STATE)    → Reset card to idle state
 *   CMD8  (SEND_IF_COND)    → Voltage check + SD v2.0 detection
 *   ACMD41(SD_SEND_OP_COND) → Negotiate voltage, detect SDHC/SDXC
 *   CMD2  (ALL_SEND_CID)    → Read Card Identification (128-bit)
 *   CMD3  (SEND_RELATIVE_ADDR) → Get card's Relative Card Address
 *   CMD9  (SEND_CSD)        → Read Card Specific Data (capacity, etc.)
 *   CMD7  (SELECT_CARD)     → Move card from Standby to Transfer state
 *   ACMD6 (SET_BUS_WIDTH)   → Switch from 1-bit to 4-bit data bus
 *   ACMD51(SEND_SCR)        → Read SD Configuration Register
 *
 * After this sequence, the card is in Transfer State and ready for
 * CMD17/18 (read) and CMD24/25 (write) data transfers.
 ******************************************************************************/
static uint8_t SD_do_init(PSD_PARAMETER pEMMCPara, uint8_t mode)
{
	uint8_t sta;
	uint8_t i;

	/* CMD0 - GO_IDLE_STATE (no response expected)
	 * Sent twice with delays for robust reset.
	 * [ref/SD_Physical_Layer_Spec_v6.00.pdf: Section 4.2.1] */
	EMMCResetIdle(pEMMCPara);
	mDelaymS(30);
	EMMCResetIdle(pEMMCPara);
	mDelaymS(30);

	/* CMD8 - SEND_IF_COND
	 * Arg 0x01AA: VHS=0x01 (2.7-3.6V), check pattern=0xAA
	 * Only SD v2.0+ cards respond. Cards that don't respond
	 * are SD v1.x or not SD cards at all.
	 * [ref/SD_Physical_Layer_Spec_v6.00.pdf: Section 4.3.13 - CMD8] */
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

	/* ACMD41 - SD_SEND_OP_COND (polls until card is ready) */
	sta = SDReadOCR(pEMMCPara);
	if(sta != CMD_SUCCESS) return OP_FAILED;

	/* CMD2 - ALL_SEND_CID (read 128-bit Card Identification)
	 * [ref/SD_Physical_Layer_Spec_v6.00.pdf: Section 5.2 - CID Register] */
	sta = EMMCReadCID(pEMMCPara);
	if(sta != CMD_SUCCESS) return OP_FAILED;

	/* CMD3 - SEND_RELATIVE_ADDR
	 * Card publishes its RCA (used for all subsequent addressed commands) */
	sta = SDSetRCA(pEMMCPara);
	if(sta != CMD_SUCCESS) return OP_FAILED;

	/* CMD9 - SEND_CSD (read 128-bit Card Specific Data)
	 * This gives us the card capacity (sector count) */
	sta = SDReadCSD(pEMMCPara);
	if(sta != CMD_SUCCESS) return OP_FAILED;

	/* CMD7 - SELECT_CARD (transitions card from Standby → Transfer state)
	 * Card must be selected before data transfer commands work */
	mDelaymS(5);
	sta = SelectEMMCCard(pEMMCPara);
	if(sta != CMD_SUCCESS) return OP_FAILED;

	/* ACMD6 - SET_BUS_WIDTH to 4-bit mode
	 * Then update the CH569 eMMC controller to match (bLW_OP_DAT4)
	 * [CH569DS1.PDF: R8_EMMC_CONTROL - RB_EMMC_LW_MASK bits] */
	sta = SDSetBusWidth(pEMMCPara, 1);
	if(sta != CMD_SUCCESS) return OP_FAILED;
	R8_EMMC_CONTROL = (R8_EMMC_CONTROL & ~RB_EMMC_LW_MASK) | bLW_OP_DAT4;

	/* ACMD51 - SEND_SCR (read SD Configuration Register)
	 * Verifies the card is responding correctly in 4-bit mode */
	sta = SD_ReadSCR(pEMMCPara, scr_buf);
	if(sta != CMD_SUCCESS) return OP_FAILED;

	/* Switch to operational clock speed.
	 * CLKMode bit enables the high-speed clock divider.
	 * Currently using LOWEMMCCLK (~15MHz) for reliability with long wires.
	 * For shorter connections, EMMCCLK_48 (divider 10, ~48MHz) is typical.
	 * [CH569DS1.PDF: R16_EMMC_CLK_DIV register] */
	if(mode < 4)
		R16_EMMC_CLK_DIV = RB_EMMC_CLKMode | RB_EMMC_PHASEINV | RB_EMMC_CLKOE | LOWEMMCCLK;
	else
		R16_EMMC_CLK_DIV = RB_EMMC_CLKMode | RB_EMMC_CLKOE | LOWEMMCCLK;

	return OP_SUCCESS;
}

/*******************************************************************************
 * SDCardInit - Initialize SD card, trying all 8 IO modes
 *
 * Different SD cards and PCB layouts require different clock phase/edge/drive
 * configurations. This function tries all 8 combinations until one works.
 *
 * Returns OP_SUCCESS if any mode successfully initializes the card.
 * The card remains configured with the first working mode.
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
