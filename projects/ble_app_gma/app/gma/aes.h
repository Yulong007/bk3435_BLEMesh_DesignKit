
#ifndef __AES_H__
#define __AES_H__

#include "gma_include.h"

#if GMA_AES
#define GETU32(pt) (((u32)(pt)[0] << 24) ^ ((u32)(pt)[1] << 16) ^ ((u32)(pt)[2] <<  8) ^ ((u32)(pt)[3]))
#define PUTU32(ct, st) { (ct)[0] = (u8)((st) >> 24); (ct)[1] = (u8)((st) >> 16); (ct)[2] = (u8)((st) >>  8); (ct)[3] = (u8)(st); }

#define AES_MAXNR 14
#define AES_BLOCK_SIZE 16
#define MAXKC   (256/32)
#define MAXKB   (256/8)
#define MAXNR   14

struct aes_key_st {
    unsigned int rd_key[4 *(AES_MAXNR + 1)];
    int rounds;
};
typedef struct aes_key_st AES_KEY;

typedef struct
{
	AES_KEY key;
	uint8_t iv[16];
} AES_CTX;

/*!
    @brief AES Event type
*/
enum
{
	AES_COMPLETE,
    AES_INPUT_DATA_INCORRECT,
    AES_SET_ENCRYPT_KEY_FAIL,
	AES_SET_DECRYPT_KEY_FAIL,
    AES_INPUT_OUTPUT_USE_SAME_BUFFER,   
};

/*!
    @brief  AES CBC mode pkcs7 Encryption Function.
    @param AES key definition Parameter.
    @param AES key 16-bytes.
    @param Want to be encrypt data.
    @param Want to be encrypt data length.
    @param After encryption Data point.
    @param After encryption Data length.
    @return Encryption Status according "AES Event type"(0:AES_COMPLETE).
*/
uint8_t aes_cbc_encrypt_pkcs7(AES_CTX *c, const uint8_t key[16], const uint8_t *in, uint8_t in_len, uint8_t *out, int *out_len);

/*!
    @brief  AES CBC mode pkcs7 Decryption Function.
    @param AES key definition Parameter.
    @param AES key 16-bytes.
    @param Want to be decrypt data.
    @param Want to be decrypt data length.
    @param After decryption Data point.
    @param After decryption Data length.
    @return Decryption Status according "AES Event type"(0:6AES_COMPLETE).
*/
uint8_t aes_cbc_decrypt_pkcs7(AES_CTX *c, const uint8_t key[16],const uint8_t *in, uint8_t in_len, uint8_t *out, int *out_len);
#endif

#endif /* __AES_H__ */

