/* Compact SHA-256 implementation for 8051. Adapted from FIPS 180-4 reference,
 * public-domain style. State and message schedule live in XDATA to fit in
 * the small XRAM/IDATA the chip provides. */

#include "sha256.h"

static __code const uint32_t K[64] = {
    0x428a2f98UL, 0x71374491UL, 0xb5c0fbcfUL, 0xe9b5dba5UL,
    0x3956c25bUL, 0x59f111f1UL, 0x923f82a4UL, 0xab1c5ed5UL,
    0xd807aa98UL, 0x12835b01UL, 0x243185beUL, 0x550c7dc3UL,
    0x72be5d74UL, 0x80deb1feUL, 0x9bdc06a7UL, 0xc19bf174UL,
    0xe49b69c1UL, 0xefbe4786UL, 0x0fc19dc6UL, 0x240ca1ccUL,
    0x2de92c6fUL, 0x4a7484aaUL, 0x5cb0a9dcUL, 0x76f988daUL,
    0x983e5152UL, 0xa831c66dUL, 0xb00327c8UL, 0xbf597fc7UL,
    0xc6e00bf3UL, 0xd5a79147UL, 0x06ca6351UL, 0x14292967UL,
    0x27b70a85UL, 0x2e1b2138UL, 0x4d2c6dfcUL, 0x53380d13UL,
    0x650a7354UL, 0x766a0abbUL, 0x81c2c92eUL, 0x92722c85UL,
    0xa2bfe8a1UL, 0xa81a664bUL, 0xc24b8b70UL, 0xc76c51a3UL,
    0xd192e819UL, 0xd6990624UL, 0xf40e3585UL, 0x106aa070UL,
    0x19a4c116UL, 0x1e376c08UL, 0x2748774cUL, 0x34b0bcb5UL,
    0x391c0cb3UL, 0x4ed8aa4aUL, 0x5b9cca4fUL, 0x682e6ff3UL,
    0x748f82eeUL, 0x78a5636fUL, 0x84c87814UL, 0x8cc70208UL,
    0x90befffaUL, 0xa4506cebUL, 0xbef9a3f7UL, 0xc67178f2UL,
};

#define ROR(x, n)  (((x) >> (n)) | ((x) << (32 - (n))))
#define CH(x,y,z)  (((x) & (y)) ^ (~(x) & (z)))
#define MAJ(x,y,z) (((x) & (y)) ^ ((x) & (z)) ^ ((y) & (z)))
#define BSIG0(x)   (ROR(x, 2)  ^ ROR(x, 13) ^ ROR(x, 22))
#define BSIG1(x)   (ROR(x, 6)  ^ ROR(x, 11) ^ ROR(x, 25))
#define SSIG0(x)   (ROR(x, 7)  ^ ROR(x, 18) ^ ((x) >> 3))
#define SSIG1(x)   (ROR(x, 17) ^ ROR(x, 19) ^ ((x) >> 10))

/* Working state for sha256_compress kept in XDATA to avoid OSEG pressure
 * on the small 8051 IRAM. compress is non-reentrant. */
__xdata static uint32_t W[64];
__xdata static uint32_t s_a, s_b, s_c, s_d, s_e, s_f, s_g, s_h, s_t1, s_t2;

static void sha256_compress(sha256_t *ctx)
{
    uint8_t  i;

    /* Load 16 big-endian message words then expand to 64. */
    for (i = 0; i < 16; i++) {
        uint8_t off = i << 2;
        W[i] = ((uint32_t)ctx->buf[off]     << 24) |
               ((uint32_t)ctx->buf[off + 1] << 16) |
               ((uint32_t)ctx->buf[off + 2] <<  8) |
               ((uint32_t)ctx->buf[off + 3]);
    }
    for (i = 16; i < 64; i++) {
        W[i] = SSIG1(W[i - 2]) + W[i - 7] + SSIG0(W[i - 15]) + W[i - 16];
    }

    s_a = ctx->h[0]; s_b = ctx->h[1]; s_c = ctx->h[2]; s_d = ctx->h[3];
    s_e = ctx->h[4]; s_f = ctx->h[5]; s_g = ctx->h[6]; s_h = ctx->h[7];

    for (i = 0; i < 64; i++) {
        s_t1 = s_h + BSIG1(s_e) + CH(s_e, s_f, s_g) + K[i] + W[i];
        s_t2 = BSIG0(s_a) + MAJ(s_a, s_b, s_c);
        s_h = s_g; s_g = s_f; s_f = s_e;
        s_e = s_d + s_t1;
        s_d = s_c; s_c = s_b; s_b = s_a;
        s_a = s_t1 + s_t2;
    }

    ctx->h[0] += s_a; ctx->h[1] += s_b; ctx->h[2] += s_c; ctx->h[3] += s_d;
    ctx->h[4] += s_e; ctx->h[5] += s_f; ctx->h[6] += s_g; ctx->h[7] += s_h;
}

void sha256_init(sha256_t *ctx)
{
    ctx->h[0] = 0x6a09e667UL; ctx->h[1] = 0xbb67ae85UL;
    ctx->h[2] = 0x3c6ef372UL; ctx->h[3] = 0xa54ff53aUL;
    ctx->h[4] = 0x510e527fUL; ctx->h[5] = 0x9b05688cUL;
    ctx->h[6] = 0x1f83d9abUL; ctx->h[7] = 0x5be0cd19UL;
    ctx->bitlen  = 0;
    ctx->buf_used = 0;
}

void sha256_update(sha256_t *ctx, const uint8_t *data, uint16_t len)
{
    while (len) {
        uint8_t take = 64 - ctx->buf_used;
        if (take > len) take = (uint8_t)len;
        for (uint8_t i = 0; i < take; i++) ctx->buf[ctx->buf_used + i] = data[i];
        ctx->buf_used += take;
        ctx->bitlen   += (uint32_t)take << 3;
        data          += take;
        len           -= take;
        if (ctx->buf_used == 64) {
            sha256_compress(ctx);
            ctx->buf_used = 0;
        }
    }
}

void sha256_final(sha256_t *ctx, uint8_t out[32])
{
    uint8_t i;
    uint32_t bitlen = ctx->bitlen;

    /* Append 0x80 padding bit. */
    ctx->buf[ctx->buf_used++] = 0x80;
    /* Pad with zeros until 56 mod 64. */
    if (ctx->buf_used > 56) {
        while (ctx->buf_used < 64) ctx->buf[ctx->buf_used++] = 0;
        sha256_compress(ctx);
        ctx->buf_used = 0;
    }
    while (ctx->buf_used < 56) ctx->buf[ctx->buf_used++] = 0;

    /* Append length in bits as big-endian 64-bit. We only support up to
     * 4 GB messages so high 32 bits are zero. */
    ctx->buf[56] = 0; ctx->buf[57] = 0; ctx->buf[58] = 0; ctx->buf[59] = 0;
    ctx->buf[60] = (uint8_t)(bitlen >> 24);
    ctx->buf[61] = (uint8_t)(bitlen >> 16);
    ctx->buf[62] = (uint8_t)(bitlen >>  8);
    ctx->buf[63] = (uint8_t)(bitlen);
    sha256_compress(ctx);

    /* Emit big-endian. */
    for (i = 0; i < 8; i++) {
        out[i * 4 + 0] = (uint8_t)(ctx->h[i] >> 24);
        out[i * 4 + 1] = (uint8_t)(ctx->h[i] >> 16);
        out[i * 4 + 2] = (uint8_t)(ctx->h[i] >>  8);
        out[i * 4 + 3] = (uint8_t)(ctx->h[i]);
    }
}
