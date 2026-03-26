#include "crypto.h"
#include <string.h>

/*****************************************************************************
 * SHA-256 implementation (FIPS 180-4)
 * https://nvlpubs.nist.gov/nistpubs/FIPS/NIST.FIPS.180-4.pdf
 *****************************************************************************/

static const uint32_t K[64] = {
	0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5,
	0x3956c25b, 0x59f111f1, 0x923f82a4, 0xab1c5ed5,
	0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3,
	0x72be5d74, 0x80deb1fe, 0x9bdc06a7, 0xc19bf174,
	0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc,
	0x2de92c6f, 0x4a7484aa, 0x5cb0a9dc, 0x76f988da,
	0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7,
	0xc6e00bf3, 0xd5a79147, 0x06ca6351, 0x14292967,
	0x27b70a85, 0x2e1b2138, 0x4d2c6dfc, 0x53380d13,
	0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85,
	0xa2bfe8a1, 0xa81a664b, 0xc24b8b70, 0xc76c51a3,
	0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070,
	0x19a4c116, 0x1e376c08, 0x2748774c, 0x34b0bcb5,
	0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f, 0x682e6ff3,
	0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208,
	0x90befffa, 0xa4506ceb, 0xbef9a3f7, 0xc67178f2
};

// Rotate right by n bits
#define ROTR(x,n) (((x)>>(n)) | ((x)<<(32-(n))))

// Choice. For each bit position, if x is 1 take bit from y, if x 0 take bit from z
#define CH(x,y,z)  (((x)&(y)) ^ (~(x)&(z)))

// Majority. For each bit position, if x+y+z > 2, output 1 else 0.
#define MAJ(x,y,z) (((x)&(y)) ^ ((x)&(z)) ^ ((y)&(z)))

// Mixing functions, diffusion
#define EP0(x) (ROTR(x,2) ^ ROTR(x,13) ^ ROTR(x,22))
#define EP1(x) (ROTR(x,6) ^ ROTR(x,11) ^ ROTR(x,25))
#define SIG0(x) (ROTR(x,7) ^ ROTR(x,18) ^ ((x)>>3))
#define SIG1(x) (ROTR(x,17) ^ ROTR(x,19) ^ ((x)>>10))

typedef struct {
	uint32_t state[8];
	uint8_t  buf[64];
	uint32_t buf_len;
	uint64_t total_len;
} sha256_ctx_t;

static void sha256_transform(sha256_ctx_t *ctx, const uint8_t block[64])
{
	uint32_t W[64];
	uint32_t a, b, c, d, e, f, g, h;
	uint32_t t1, t2;
	int i;

	for (i = 0; i < 16; i++) {
		W[i] = ((uint32_t)block[i*4] << 24) |
		        ((uint32_t)block[i*4+1] << 16) |
		        ((uint32_t)block[i*4+2] << 8) |
		        ((uint32_t)block[i*4+3]);
	}
	for (i = 16; i < 64; i++)
		W[i] = SIG1(W[i-2]) + W[i-7] + SIG0(W[i-15]) + W[i-16];

	a = ctx->state[0]; b = ctx->state[1];
	c = ctx->state[2]; d = ctx->state[3];
	e = ctx->state[4]; f = ctx->state[5];
	g = ctx->state[6]; h = ctx->state[7];

	for (i = 0; i < 64; i++) {
		t1 = h + EP1(e) + CH(e,f,g) + K[i] + W[i];
		t2 = EP0(a) + MAJ(a,b,c);
		h = g; g = f; f = e;
		e = d + t1;
		d = c; c = b; b = a;
		a = t1 + t2;
	}

	ctx->state[0] += a; ctx->state[1] += b;
	ctx->state[2] += c; ctx->state[3] += d;
	ctx->state[4] += e; ctx->state[5] += f;
	ctx->state[6] += g; ctx->state[7] += h;
}

static void sha256_init(sha256_ctx_t *ctx)
{
	ctx->state[0] = 0x6a09e667; ctx->state[1] = 0xbb67ae85;
	ctx->state[2] = 0x3c6ef372; ctx->state[3] = 0xa54ff53a;
	ctx->state[4] = 0x510e527f; ctx->state[5] = 0x9b05688c;
	ctx->state[6] = 0x1f83d9ab; ctx->state[7] = 0x5be0cd19;
	ctx->buf_len = 0;
	ctx->total_len = 0;
}

static void sha256_update(sha256_ctx_t *ctx, const uint8_t *data, size_t len)
{
	size_t i;
	ctx->total_len += len;

	/* Fill partial block first */
	if (ctx->buf_len > 0) {
		size_t space = 64 - ctx->buf_len;
		size_t copy = (len < space) ? len : space;
		memcpy(ctx->buf + ctx->buf_len, data, copy);
		ctx->buf_len += copy;
		data += copy;
		len -= copy;
		if (ctx->buf_len == 64) {
			sha256_transform(ctx, ctx->buf);
			ctx->buf_len = 0;
		}
	}

	/* Process full blocks */
	for (i = 0; i + 64 <= len; i += 64)
		sha256_transform(ctx, data + i);

	/* Store remainder */
	if (i < len) {
		memcpy(ctx->buf, data + i, len - i);
		ctx->buf_len = len - i;
	}
}

static void sha256_final(sha256_ctx_t *ctx, uint8_t hash[32])
{
	uint64_t bits = ctx->total_len * 8;
	uint8_t pad = 0x80;
	uint8_t zero = 0x00;
	int i;

	/* Append 0x80 */
	sha256_update(ctx, &pad, 1);

	/* Pad with zeros until 56 bytes mod 64 */
	while (ctx->buf_len != 56)
		sha256_update(ctx, &zero, 1);

	/* Append length in bits (big-endian 64-bit) */
	for (i = 7; i >= 0; i--) {
		uint8_t b = (uint8_t)(bits >> (i * 8));
		sha256_update(ctx, &b, 1);
	}

	/* Write hash output (big-endian) */
	for (i = 0; i < 8; i++) {
		hash[i*4]   = (uint8_t)(ctx->state[i] >> 24);
		hash[i*4+1] = (uint8_t)(ctx->state[i] >> 16);
		hash[i*4+2] = (uint8_t)(ctx->state[i] >> 8);
		hash[i*4+3] = (uint8_t)(ctx->state[i]);
	}
}

void sha256(const uint8_t *data, size_t len, uint8_t hash_out[32])
{
	sha256_ctx_t ctx;
	sha256_init(&ctx);
	sha256_update(&ctx, data, len);
	sha256_final(&ctx, hash_out);
}

/*****************************************************************************
 * KDF: Iterated SHA-256 key derivation
 * key = SHA256(salt || password), then 1000x key = SHA256(key || password)
 *****************************************************************************/

static const uint8_t kdf_salt[16] = "OVRDrive-v2-salt";
#define KDF_ROUNDS 1000

void derive_key(const uint8_t *password, size_t pw_len, uint8_t key_out[32])
{
	sha256_ctx_t ctx;
	int i;

	/* Round 0: key = SHA256(salt || password) */
	sha256_init(&ctx);
	sha256_update(&ctx, kdf_salt, sizeof(kdf_salt));
	sha256_update(&ctx, password, pw_len);
	sha256_final(&ctx, key_out);

	/* Rounds 1..KDF_ROUNDS: key = SHA256(key || password) */
	for (i = 0; i < KDF_ROUNDS; i++) {
		sha256_init(&ctx);
		sha256_update(&ctx, key_out, 32);
		sha256_update(&ctx, password, pw_len);
		sha256_final(&ctx, key_out);
	}
}
