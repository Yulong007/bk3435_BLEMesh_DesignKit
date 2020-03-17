

#ifndef __SHA256_H__
#define __SHA256_H__

#include "gma_include.h"

#if GMA_AES
typedef unsigned char BYTE_T;             // 8-bit byte
typedef unsigned int  WORD_T;             // 32-bit word, change to "long" for 16-bit machines

/****************************** MACROS ******************************/
#define SHA256_BLOCK_SIZE 32            // SHA256 outputs a 32 byte digest

typedef struct {
    BYTE_T data[64];
    WORD_T datalen;
    unsigned long long bitlen;
    WORD_T state[8];
} SHA256_CTX;

/*********************** FUNCTION DECLARATIONS **********************/
void ali_sha256_init(SHA256_CTX *ctx);
void ali_sha256_update(SHA256_CTX *ctx, const BYTE_T data[], size_t len);
void ali_sha256_final(SHA256_CTX *ctx, BYTE_T hash[]);
#endif

#endif /* __SHA256_H__ */

