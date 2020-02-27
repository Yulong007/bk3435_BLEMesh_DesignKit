#include <stdio.h>
#include <stdlib.h>

#include "mesh_log.h"

#ifndef MAX
#define MAX(a, b) ((a) > (b) ? (a) : (b))
#endif /* MAX */

#ifndef MIN
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#endif /* MIN */


const char *mesh_buffer_to_hex(const void *buf, unsigned int len)
{
    static const char hex[] = "0123456789abcdef";
    static char str[129];
    const uint8_t *b = buf;
    int i;

    len = MIN(len, (sizeof(str) - 1) / 2);

    for (i = 0; i < len; i++)
    {
        str[i * 2]     = hex[b[i] >> 4];
        str[i * 2 + 1] = hex[b[i] & 0xf];
    }

    str[i * 2] = '\0';

    return str;
}

void mem_rcopy(uint8_t *dst, uint8_t const *src, uint16_t len)
{
	src += len;
	while (len--) 
	{
		*dst++ = *--src;
	}
}


