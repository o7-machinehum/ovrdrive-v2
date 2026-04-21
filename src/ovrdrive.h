#ifndef OVRDRIVE_H_
#define OVRDRIVE_H_

#include <stdint.h>
#include <stddef.h>

#define STATE_LOCKED    0
#define STATE_UNLOCKED  1
#define LOCKED_SECTORS  (16777216UL)  /* 8GB in sectors */

extern volatile uint8_t ovrd_state;
extern volatile uint8_t ovrd_unlock_pending;
extern uint32_t aes_key[8] __attribute__((section(".DMADATA")));

void ovrd_init(void);
void ovrd_snoop_write(uint8_t *buf, uint32_t len);
void ovrd_poll(void);
void ovrd_crypt_buf(uint8_t *buf, uint32_t sd_lba, uint16_t num_sectors);

#define ovrd_encrypt_buf ovrd_crypt_buf
#define ovrd_decrypt_buf ovrd_crypt_buf

#endif /* OVRDRIVE_H_ */
