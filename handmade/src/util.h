#ifndef UTIL_H
#define UTIL_H

#include "types.h"

/* IE register bits (8051 interrupt enable). */
#define IE_EA   0x80

#define CRITICAL_ENTER()  (IE &= (uint8_t)~IE_EA)
#define CRITICAL_EXIT()   (IE |= IE_EA)

/* Memory primitives for generic (banked) pointers.  SDCC inherits the 3-byte
 * generic pointer type from the void * assignment, so these handle __code,
 * __xdata, and __data sources uniformly.  Length is uint8_t (max 255 bytes). */

static void mem_copy(void *dst, const void *src, uint8_t n) {
  uint8_t *d = dst;
  const uint8_t *s = src;
  while (n--) *d++ = *s++;
}

static void mem_set(void *dst, uint8_t val, uint8_t n) {
  uint8_t *d = dst;
  while (n--) *d++ = val;
}

#endif /* UTIL_H */
