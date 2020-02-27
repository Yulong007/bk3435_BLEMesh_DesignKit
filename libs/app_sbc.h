/**
 *************************************************************************************
 * @file    sbc_decoder.h
 * @brief   An implementation of SBC decoder for bluetooth.
 *
 * @author  Aixing.Li
 * @version V1.0.0
 *
 * &copy; 2017 BEKEN Corporation Ltd. All rights reserved.
 *
 *************************************************************************************
 */
#ifndef __APP_SBC_H__
#define __APP_SBC_H__

//#include "sbc_common.h"
#include <stdint.h>

#define EN_ENCODER
//#define   EN_DECODER


#define SBC_MAX_FRAME_SIZE              513         /**< SBC max frame size 513 in bytes = 4+(4*8*2)/8+(8+16*250)/8      */


/**
 * @brief SBC frame header context
 */
typedef struct _SbcFrameHeader
{
#if defined(__BIG_ENDIAN__)
    //big endianness
    uint32_t crc_check          : 8;
    uint32_t bitpool            : 8;
    uint32_t subband_mode       : 1;
    uint32_t allocation_method  : 1;
    uint32_t channel_mode       : 2;
    uint32_t block_mode         : 2;
    uint32_t sample_rate_index  : 2;
    uint32_t syncword           : 8;
#else
    //little endianness
    uint32_t syncword           : 8;
    uint32_t subband_mode       : 1;
    uint32_t allocation_method  : 1;
    uint32_t channel_mode       : 2;
    uint32_t block_mode         : 2;
    uint32_t sample_rate_index  : 2;
    uint32_t bitpool            : 8;
    uint32_t crc_check          : 8;
#endif
} SbcFrameHeader;




/**
 * @brief SBC decoder context
 */
typedef struct _SbcCommonContext
{
    int8_t   blocks;                    /**< block number       */
    int8_t   subbands;                  /**< subbands number    */
    uint8_t  join;                      /**< bit number x set means joint stereo has been used in sub-band x */
    uint8_t  bitpool;                   /**< indicate the size of the bit allocation pool that has been used for encoding the stream */

    int8_t   channel_mode;              /**< channel mode       */
    int8_t   sample_rate_index;         /**< sample rate index, 0:16000, 1:32000, 2:44100, 3:48000 */
    int8_t   allocation_method;         /**< allocation method  */
    int8_t   reserved8;                 /**< dummy, reserved for byte align */

    int8_t   bits[2][8];                /**< calculate result by bit allocation. */

    int8_t   scale_factor[2][8];        /**< only the lower 4 bits of every element are to be used */

    int32_t  mem[2][8];                 /**< Memory used as bit need and levels */

} SbcCommonContext;




#ifdef  EN_ENCODER

/**
 * @brief SBC encoder context
 */
typedef struct _SbcEncoderContext
{
    SbcCommonContext frame;
    SbcFrameHeader   header;

    int8_t   num_channels;              /**< channels number    */
    uint8_t  pcm_length;                /**< PCM length         */
    uint16_t sample_rate;               /**< sample rate        */

    int32_t  sb_sample_f[2][16][8];     /**< subband sample     */

#if 1
    uint8_t  reserved;
    uint8_t  frame_index;
    uint8_t  frame_id[2];
#endif

    uint8_t  stream[256];     // 512          /**< encoded buffer     */
    int32_t  position[2];
    int32_t  xfifo[2][160];

} SbcEncoderContext;




/**
 * @brief  SBC encoder initialzie
 * @param  sbc          SBC encoder context pointer
 * @param  sample_rate  sample rate
 * @param  num_channels number of channels
 * @return error code, @see SBC_ENCODER_ERROR_CODE
 */
int32_t sbc_encoder_init(SbcEncoderContext *sbc, int32_t sample_rate, int32_t num_channels);

/**
 * @brief  SBC encoder parameters config
 * @param  sbc SBC encoder context pointer
 * @param  cmd @see SBC_ENCODER_IOCTRL_CMD
 * @param  arg the argument or result adress for the cmd
 * @return error code, @see SBC_ENCODER_ERROR_CODE
 */
int32_t sbc_encoder_ioctrl(SbcEncoderContext *sbc, uint32_t cmd, uint32_t arg);

/**
 * @brief  SBC encoder encode one frame
 * @param  sbc SBC encoder context pointer
 * @param  pcm input PCM samples to be encoded,
 *         the number of input PCM samples MUST be sbc->pcm_length !!!
 * @return encoded buffer length by encoder if no error ocurs,
 *         else error code (always small than 0) will be return, @see SBC_ENCODER_ERROR_CODE
 *         the output encoded buffer refer to sbc->stream.
 */
int32_t sbc_encoder_frame_encode(SbcEncoderContext *sbc, const int16_t *pcm);

#endif

//==================================================
//==================================================
//  DECODER DEFINE
//==================================================
//==================================================
#ifdef  EN_DECODER

/**
 * @brief SBC decoder context
 */
typedef struct _SbcDecoderContext
{
    SbcCommonContext frame;

    int8_t   num_channels;              /**< channels number    */
    uint8_t  pcm_length;                /**< PCM length         */
    uint16_t sample_rate;               /**< sample rate        */

    int32_t  pcm_sample[2][128];        /**< PCM frame buffer   */

    int32_t  vfifo[2][170];             /**< FIFO V for subbands synthesis calculation. */

    int32_t  offset[2][16];

} SbcDecoderContext;


/**
 * @brief  SBC decoder initialize
 * @param  sbc SBC decoder context pointer
 * @return error code, @see SBC_DECODER_ERROR_CODE
 */
int32_t sbc_decoder_init(SbcDecoderContext *sbc);

/**
 * @brief  SBC decoder decode one frame
 * @param  sbc    SBC decoder context pointer
 * @param  data   buffer to be decoded
 * @param  length input buffer legth
 * @return consumed buffer length by decoder if no error ocurs,
 *         else error code (always small than 0) will be return, @see SBC_DECODER_ERROR_CODE
 *         the output PCM data please refer to the follows variables:
 *         sbc->pcm_sample   means output PCM data address
 *         sbc->pcm_length   means output PCM data length in sample
 *           sbc->num_channels means output PCM data channels
 */
int32_t sbc_decoder_frame_decode(SbcDecoderContext *sbc, const uint8_t *data, int32_t length);

#endif

#endif
