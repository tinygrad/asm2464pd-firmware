#ifndef __TYPES_H__
#define __TYPES_H__

/*
 * ASM2464PD Firmware - Type Definitions
 *
 * 8051 architecture types for SDCC compiler
 */

/* Standard integer types */
typedef unsigned char       uint8_t;
typedef signed char         int8_t;
typedef unsigned short      uint16_t;
typedef signed short        int16_t;
typedef unsigned long       uint32_t;
typedef signed long         int32_t;

/* Boolean type */
typedef uint8_t             bool;
#define true                1
#define false               0

/* NULL definition */
#ifndef NULL
#define NULL                ((void *)0)
#endif

/* 8051 memory space qualifiers (SDCC specific) */
#ifndef __SDCC
/* Fallback for non-SDCC compilers (for IDE support) */
#define __data
#define __idata
#define __xdata
#define __pdata
#define __code
#define __sfr
#define __sbit
#define __at(x)
#define __interrupt(x)
#define __using(x)
#define __naked
#define __reentrant
#endif

/* Helper macros for register access */
#define SFR(addr)           __sfr __at(addr)
#define SBIT(addr, bit)     __sbit __at(addr + bit)

/* Byte access macros */
#define LOBYTE(w)           ((uint8_t)((w) & 0xFF))
#define HIBYTE(w)           ((uint8_t)(((w) >> 8) & 0xFF))
#define MAKEWORD(lo, hi)    ((uint16_t)(((uint16_t)(hi) << 8) | (uint8_t)(lo)))

/* Bit manipulation macros */
#define BIT(n)              (1 << (n))
#define SETBIT(v, n)        ((v) |= BIT(n))
#define CLRBIT(v, n)        ((v) &= ~BIT(n))
#define TSTBIT(v, n)        (((v) & BIT(n)) != 0)

/* Memory access macros */
#define XDATA8(addr)        (*(__xdata uint8_t *)(addr))
#define XDATA16(addr)       (*(__xdata uint16_t *)(addr))
#define XDATA32(addr)       (*(__xdata uint32_t *)(addr))

#define CODE8(addr)         (*(__code uint8_t *)(addr))
#define CODE16(addr)        (*(__code uint16_t *)(addr))

#endif /* __TYPES_H__ */
