#ifndef BOOTSTUB_TYPES_H
#define BOOTSTUB_TYPES_H

typedef unsigned char uint8_t;
typedef unsigned short uint16_t;
typedef unsigned long uint32_t;
typedef uint8_t bool;

#define true 1
#define false 0

#define SFR(addr) __sfr __at(addr)

#endif
