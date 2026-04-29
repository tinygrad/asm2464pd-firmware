#ifndef BOOTSTUB_SHA256_H
#define BOOTSTUB_SHA256_H

#include "types.h"

/* Streaming SHA-256. Callers should place their context in __xdata so it
 * doesn't bloat IDATA. */
typedef struct {
    uint32_t h[8];
    uint32_t bitlen;     /* low 32 bits — bootstub never hashes more than 4 GB */
    uint8_t  buf[64];
    uint8_t  buf_used;
} sha256_t;

void sha256_init(sha256_t *ctx);
void sha256_update(sha256_t *ctx, const uint8_t *data, uint16_t len);
void sha256_final(sha256_t *ctx, uint8_t out[32]);

#endif
