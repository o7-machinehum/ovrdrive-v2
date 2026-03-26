#include "ovrdrive.h"
#include "crypto.h"
#include "sw_udisk.h"
#include "CH56x_ecdc.h"
#include "CH56x_debug_log.h"
#include <string.h>

volatile uint8_t ovrd_state = STATE_LOCKED;
volatile uint8_t ovrd_unlock_pending = 0;

/* AES-256 key in RAMX for DMA access by ECDC */
__attribute__((aligned(16))) uint32_t aes_key[8] __attribute__((section(".DMADATA")));

/* Small test buffer in RAMX for ECDC self-test */
__attribute__((aligned(16))) static uint8_t ecdc_test_buf[16] __attribute__((section(".DMADATA")));

/* Pending password extracted from write buffer */
static uint8_t pending_pw[128];
static size_t pending_pw_len;

void ovrd_init(void)
{
	/* Ensure ECDC is fully disabled — prevent stale state from
	 * corrupting eMMC-to-RAMX DMA in locked mode */
	R16_ECEC_CTRL = 0;
	R8_ECDC_INT_FG = 0xFF;

	Udisk_Capability = LOCKED_SECTORS;
	ovrd_state = STATE_LOCKED;
	log_printf("ovrd: locked, %lu sectors\r\n", LOCKED_SECTORS);
}

/* Run ECDC self-test: encrypt 16 bytes of zeros, verify non-zero,
 * encrypt again (CTR is involutory), verify zeros restored. */
static void ecdc_selftest(void)
{
	uint32_t ctr[4] = {0, 0x12345678, 0, 0};

	/* Fill test buffer with zeros */
	memset(ecdc_test_buf, 0, 16);
	uint32_t before = *(uint32_t*)ecdc_test_buf;

	/* Encrypt: SRAM_LEN is in 128-bit (16-byte) units per CH569 datasheet Ch15 */
	ECDC_SetCount((puint32_t)ctr);
	ECDC_Excute(SELFDMA_ENCRY, MODE_LITTLE_ENDIAN);
	ECDC_SelfDMA((uint32_t)ecdc_test_buf, 1);  /* 1 × 16 bytes = 16 bytes */
	uint32_t after_enc = *(uint32_t*)ecdc_test_buf;

	/* Decrypt (same as encrypt for CTR) */
	ECDC_SetCount((puint32_t)ctr);
	ECDC_Excute(SELFDMA_ENCRY, MODE_LITTLE_ENDIAN);
	ECDC_SelfDMA((uint32_t)ecdc_test_buf, 1);
	uint32_t after_dec = *(uint32_t*)ecdc_test_buf;

	log_printf("ovrd: ECDC test: before=%08lx enc=%08lx dec=%08lx %s\r\n",
	           before, after_enc, after_dec,
	           (after_enc != 0 && after_dec == 0) ? "PASS" : "FAIL");

	/* Also test with SELFDMA_DECRY to see if it differs */
	memset(ecdc_test_buf, 0, 16);
	ECDC_SetCount((puint32_t)ctr);
	ECDC_Excute(SELFDMA_ENCRY, MODE_LITTLE_ENDIAN);
	ECDC_SelfDMA((uint32_t)ecdc_test_buf, 1);
	uint32_t enc2 = *(uint32_t*)ecdc_test_buf;

	ECDC_SetCount((puint32_t)ctr);
	ECDC_Excute(SELFDMA_DECRY, MODE_LITTLE_ENDIAN);
	ECDC_SelfDMA((uint32_t)ecdc_test_buf, 1);
	uint32_t dec2 = *(uint32_t*)ecdc_test_buf;

	log_printf("ovrd: ECDC enc=%08lx decry=%08lx %s\r\n",
	           enc2, dec2,
	           (dec2 == 0) ? "DECRY=ENCRY" : "DECRY!=ENCRY");
}

static void ovrd_do_unlock(void)
{
	log_printf("ovrd: deriving key (%u bytes)...\r\n", (unsigned)pending_pw_len);

	uint8_t key_bytes[32];
	derive_key(pending_pw, pending_pw_len, key_bytes);
	memcpy(aes_key, key_bytes, 32);
	memset(key_bytes, 0, sizeof(key_bytes));
	memset(pending_pw, 0, sizeof(pending_pw));
	pending_pw_len = 0;

	/* Initialize ECDC for AES-256-CTR */
	uint32_t initial_ctr[4] = {0, 0, 0, 0};
	ECDC_Init(MODE_AES_CTR, ECDCCLK_240MHZ, KEYLENGTH_256BIT,
	          (puint32_t)aes_key, (puint32_t)initial_ctr);

	/* Verify ECDC hardware is working */
	ecdc_selftest();

	Udisk_Capability = TF_EMMCParam.EMMCSecNum - LOCKED_SECTORS;
	ovrd_state = STATE_UNLOCKED;

	log_printf("ovrd: unlocked, %lu sectors\r\n", Udisk_Capability);

	/* Reset BOT state and re-enumerate USB */
	Udisk_Transfer_Status = 0;
	UDISK_InPackflag = 0;
	UDISK_OutPackflag = 0;
	USB3_force();

	log_printf("ovrd: re-enumerated\r\n");
}

void ovrd_snoop_write(const uint8_t *buf, uint32_t len)
{
	if (ovrd_state != STATE_LOCKED || ovrd_unlock_pending)
		return;

	const char *prefix = "password:";
	const size_t prefix_len = 9;
	uint32_t i;

	for (i = 0; i + prefix_len <= len; i++) {
		if (buf[i] != 'p')
			continue;
		if (memcmp(buf + i, prefix, prefix_len) != 0)
			continue;

		/* Found "password:" — extract until newline/CR/null */
		size_t pw_start = i + prefix_len;
		size_t pw_end = pw_start;
		while (pw_end < len && (pw_end - pw_start) < sizeof(pending_pw) &&
		       buf[pw_end] != '\n' && buf[pw_end] != '\r' && buf[pw_end] != '\0')
			pw_end++;

		size_t pw_len = pw_end - pw_start;
		if (pw_len > 0) {
			memcpy(pending_pw, buf + pw_start, pw_len);
			pending_pw_len = pw_len;
			ovrd_unlock_pending = 1;
			log_printf("ovrd: password snooped (%u bytes)\r\n", (unsigned)pw_len);
		}
		return;
	}
}

void ovrd_poll(void)
{
	if (!ovrd_unlock_pending)
		return;
	ovrd_unlock_pending = 0;
	ovrd_do_unlock();
}

static void ecdc_set_sector_nonce(uint32_t sd_lba)
{
	uint32_t ctr[4];
	ctr[0] = 0;
	ctr[1] = sd_lba;
	ctr[2] = 0;
	ctr[3] = 0;
	ECDC_SetCount((puint32_t)ctr);
}

void ovrd_encrypt_buf(uint8_t *buf, uint32_t sd_lba, uint16_t num_sectors)
{
	static uint8_t elog = 0;
	uint32_t pre = *(volatile uint32_t*)buf;

	uint16_t i;
	for (i = 0; i < num_sectors; i++) {
		ecdc_set_sector_nonce(sd_lba + i);
		ECDC_Excute(SELFDMA_ENCRY, MODE_LITTLE_ENDIAN);
		/* SRAM_LEN is in 128-bit units: 512 bytes / 16 = 32 */
		ECDC_SelfDMA((uint32_t)(buf + i * 512), 32);
	}
	/* Disable ECDC operational bits so subsequent eMMC DMA is not
	 * intercepted. WRPERI_EN stays set after ECDC_Excute and would
	 * cause eMMC DMA (SRAM→peripheral) to be encrypted a second time. */
	R16_ECEC_CTRL &= ~(RB_ECDC_WRSRAM_EN | RB_ECDC_WRPERI_EN |
	                    RB_ECDC_RDPERI_EN | RB_ECDC_MODE_SEL);

	if (elog < 5) {
		uint32_t post = *(volatile uint32_t*)buf;
		log_printf("E lba=%lu n=%u %08lx->%08lx ctrl=%04x\r\n",
		           sd_lba, num_sectors, pre, post, R16_ECEC_CTRL);
		elog++;
	}
}

void ovrd_decrypt_buf(uint8_t *buf, uint32_t sd_lba, uint16_t num_sectors)
{
	static uint8_t dlog = 0;
	uint32_t pre = *(volatile uint32_t*)buf;

	uint16_t i;
	for (i = 0; i < num_sectors; i++) {
		ecdc_set_sector_nonce(sd_lba + i);
		ECDC_Excute(SELFDMA_ENCRY, MODE_LITTLE_ENDIAN);
		/* SRAM_LEN is in 128-bit units: 512 bytes / 16 = 32 */
		ECDC_SelfDMA((uint32_t)(buf + i * 512), 32);
	}
	/* Disable ECDC so eMMC DMA for next sector isn't intercepted */
	R16_ECEC_CTRL &= ~(RB_ECDC_WRSRAM_EN | RB_ECDC_WRPERI_EN |
	                    RB_ECDC_RDPERI_EN | RB_ECDC_MODE_SEL);

	if (dlog < 5) {
		uint32_t post = *(volatile uint32_t*)buf;
		log_printf("D lba=%lu n=%u %08lx->%08lx ctrl=%04x\r\n",
		           sd_lba, num_sectors, pre, post, R16_ECEC_CTRL);
		dlog++;
	}
}
