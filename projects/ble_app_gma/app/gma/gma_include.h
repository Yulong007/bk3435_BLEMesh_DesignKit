
#ifndef __GMA_INCLUDE__
#define __GMA_INCLUDE__

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stddef.h>
#include "user_config.h"

typedef unsigned int u32;
typedef unsigned short u16;
typedef unsigned char u8;

extern signed int uart_printf(const char *fmt, ...);

#define GMA_DEBUG    uart_printf
#define GMA_PRINTF   uart_printf
#define GMA_MALLOC()
#define GMA_FREE()

#endif /* __GMA_INCLUDE__ */
