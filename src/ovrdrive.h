#ifndef OVRDRIVE_H_
#define OVRDRIVE_H_

#include <stdint.h>
#include <stddef.h>

/* State machine */
#define STATE_LOCKED    0
#define STATE_UNLOCKED  1

/* First 8GB visible when locked: 8*1024*1024*1024/512 = 16777216 sectors */
#define LOCKED_SECTORS  (16777216UL)

extern volatile uint8_t ovrd_state;
extern volatile uint8_t ovrd_unlock_pending;
extern uint32_t aes_key[8] __attribute__((section(".DMADATA")));

/* Called from main() after SD init. Sets locked capacity. */
void ovrd_init(void);

/* Snoop write buffer for "password:" trigger. Called before eMMC write.
 * Scrubs the password from buf so plaintext never reaches SD. */
void ovrd_snoop_write(uint8_t *buf, uint32_t len);

/* Called from main loop — performs deferred unlock if password was found. */
void ovrd_poll(void);

/* Encrypt sectors in-place in RAMX using ECDC AES-256-CTR. */
void ovrd_encrypt_buf(uint8_t *buf, uint32_t sd_lba, uint16_t num_sectors);

/* Decrypt sectors in-place in RAMX using ECDC AES-256-CTR. */
void ovrd_decrypt_buf(uint8_t *buf, uint32_t sd_lba, uint16_t num_sectors);

#endif /* OVRDRIVE_H_ */
