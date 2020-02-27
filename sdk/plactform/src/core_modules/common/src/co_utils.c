/**
****************************************************************************************
*
* @file co_utils.c
*
* @brief Common Utility functions
*
* Copyright (C) RivieraWaves 2009-2015
*
*
****************************************************************************************
*/

/**
****************************************************************************************
* @addtogroup  CO_UTILS
* @{
****************************************************************************************
*/

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"      // SW configuration

#include <string.h>           // for mem* functions
#include "co_bt.h"            // common bt definitions
#include "co_utils.h"         // common utility definitions
#include "co_endian.h"        // Endianess
#include "co_math.h"          // Mathematics functions



/*
 * DEFINES
 ****************************************************************************************
 */

/// Minimum time to do a kernel timer action
#define CO_MIN_NB_SLOT_DUR       32

/*
 * GLOBAL VARIABLES
 ******* *********************************************************************************
 */


/*
 * CONSTANT DEFINITIONS
 ****************************************************************************************
 */

/// Number of '1' bits in values from 0 to 15, used to fasten bit counting
const unsigned char one_bits[] = {0, 1, 1, 2, 1, 2, 2, 3, 1, 2, 2, 3, 2, 3, 3, 4};

/// SCA to PPM
const uint16_t co_sca2ppm[] =
{
    [SCA_500PPM] = 500,
    [SCA_250PPM] = 250,
    [SCA_150PPM] = 150,
    [SCA_100PPM] = 100,
    [SCA_75PPM] = 75,
    [SCA_50PPM] = 50,
    [SCA_30PPM] = 30,
    [SCA_20PPM] = 20
};

/// NULL BD address
const struct bd_addr co_null_bdaddr = {{0x00, 0x00, 0x00, 0x00, 0x00, 0x00}};


/*
 * LOCAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */


/*
 * EXPORTED FUNCTION DEFINITIONS
 ****************************************************************************************
 */

#if RW_DEBUG
void co_bytes_to_string(char *dest, uint8_t *src, uint8_t nb_bytes)
{
    for (int i = 0 ; i < nb_bytes ; i++)
    {
        char digit;
        uint8_t byte = *(src + nb_bytes - 1 - i);

        // MSbs
        digit = (byte & 0xF0) >> 4;
        digit += (digit < 10) ? 48 : 55;
        *(dest + 2 * i) = (char)digit;

        // LSbs
        digit = (byte & 0x0F) >> 0;
        digit += (digit < 10) ? 48 : 55;
        *(dest + 2 * i + 1) = (char)digit;
    }
}
#endif //RW_DEBUG

bool co_bdaddr_compare(struct bd_addr const *bd_address1,
                       struct bd_addr const *bd_address2)
{

    for (uint8_t idx = 0; idx < BD_ADDR_LEN; idx++)
    {
        /// checks if the addresses are similar
        if (bd_address1->addr[idx] != bd_address2->addr[idx])
        {
            return (false);
        }
    }
    return (true);
}

#if (BT_EMB_PRESENT)
uint32_t co_slot_to_duration(uint16_t slot_cnt)
{
    uint32_t duration = 0;
    /* Avoid Slot number < 32                                                           */
    if (slot_cnt < CO_MIN_NB_SLOT_DUR)
    {
        slot_cnt = CO_MIN_NB_SLOT_DUR;
    }
    /* Duration contains a number of Slot and a tick is 10 ms                           */
    duration = slot_cnt >> 4;
    return (duration);
}

uint8_t co_nb_good_channels(const struct chnl_map *map)
{
    uint8_t nb_good_channels = 0;

    // Count number of good channels
    for (int i = (CHNL_MAP_LEN - 1) ; i >= 0 ; i--)
    {
        uint8_t byte = map->map[i];
        nb_good_channels += NB_ONE_BITS(byte);
    }

    return nb_good_channels;
}
#endif //BT_EMB_PRESENT

#if 1//(BLE_MESH) 

/// Extract length of an array from a format string
static uint16_t co_util_read_array_size(char **fmt_cursor)
{
    // Read size
    uint16_t size = 0;

    // Sanity check
    ASSERT_ERR(fmt_cursor);

    // Convert unit
    size = (*(*fmt_cursor)++) - '0';

    while (((*(*fmt_cursor)) >= '0') && ((*(*fmt_cursor)) <= '9'))
    {
        // Convert tens
        size = 10 * size + ((*(*fmt_cursor)++) - '0');
    }

    // Return the read size
    return (size);
}


uint8_t co_util_pack(uint8_t *out, uint8_t *in, uint16_t *out_len, uint16_t in_len, const char *format)
{
    uint8_t status = CO_UTIL_PACK_OK;
    uint8_t *p_in = in;
    uint8_t *p_out = out;
    uint8_t *p_in_end = in + in_len;
    char *cursor = (char *) format;
    bool b_copy = (in != NULL) && (out != NULL);

    ASSERT_ERR(format != NULL);

    bool b_lsb = true;
    bool big_number = false;

    // Check if forced to little endian
    if (*cursor == '<')
    {
        b_lsb = true;
        cursor++;
    }
    else if (*cursor == '>')
    {
        b_lsb = false;
        cursor++;
    }

    while ((*cursor != '\0') && (status == CO_UTIL_PACK_OK))
    {
        uint16_t nb = 0;

        // Check if the new field is an array (starting with a number)
        if ((*cursor >= '0') && (*cursor <= '9'))
        {
            nb = co_util_read_array_size(&cursor);
        }

        // Parse the format string
        switch (*cursor++)
        {
            case ('G'):   // Big Number
            {
                big_number = true;
            } // No break
            case ('B'):   // Byte
            {
                // For arrays only
                if (nb > 1)
                {
                    if (b_copy)
                    {
                        // Check if enough space in input buffer to read
                        if ((p_in + nb) > p_in_end)
                        {
                            status = CO_UTIL_PACK_IN_BUF_OVFLW;
                            break;
                        }

                        // Copy bytes
                        if (!big_number || (b_lsb == CPU_LE))
                        {
                            memcpy(p_out, p_in, nb);
                        }
                        // Swap bytes
                        else
                        {
                            co_bswap(p_out, p_in, nb);
                        }
                    }

                    // Move pointers
                    p_out += nb;
                    p_in += nb;
                }
                else
                {
                    if (b_copy)
                    {
                        // Check if enough space in input buffer to read
                        if ((p_in + 1) > p_in_end)
                        {
                            status = CO_UTIL_PACK_IN_BUF_OVFLW;
                            break;
                        }

                        // Copy data
                        *p_out = *p_in;
                    }

                    // Move pointers
                    p_out++;
                    p_in++;
                }

                big_number = false;
            }
            break;

            case ('H'):   // Short Word
            {
                // Align data buffer to a 16-bits address
                uint16_t *short_word = (uint16_t *)CO_ALIGN2_HI((uint32_t)p_in);

                if (b_copy)
                {
                    // Check if enough space in input buffer to read
                    if (((uint8_t *)(short_word + 1)) > p_in_end)
                    {
                        status = CO_UTIL_PACK_IN_BUF_OVFLW;
                        break;
                    }

                    // Copy data
                    if (b_lsb)
                    {
                        co_write16p(p_out, co_htobs(*short_word));
                    }
                    else
                    {
                        co_write16p(p_out, co_htons(*short_word));
                    }
                }

                // Move pointers
                p_in = (uint8_t *)(short_word + 1);
                p_out += 2;
            }
            break;

            case ('D'):   // 24 bits integer
            {
                // Align data buffer to a 32-bits address
                uint32_t *long_word = (uint32_t *)CO_ALIGN4_HI((uint32_t)p_in);

                if (b_copy)
                {
                    // Check if enough space in input buffer to read
                    if (((uint8_t *)(long_word + 1)) > p_in_end)
                    {
                        status = CO_UTIL_PACK_IN_BUF_OVFLW;
                        break;
                    }

                    // Copy data
                    if (b_lsb)
                    {
                        co_write24p(p_out, co_htob24(*long_word));
                    }
                    else
                    {
                        co_write24p(p_out, co_hton24(*long_word));
                    }
                }

                // Move pointers
                p_in = (uint8_t *)(long_word + 1);
                p_out += 3;
            }
            break;

            case ('L'):   // Long Word
            {
                // Align data buffer to a 32-bits address
                uint32_t *long_word = (uint32_t *)CO_ALIGN4_HI((uint32_t)p_in);

                if (b_copy)
                {
                    // Check if enough space in input buffer to read
                    if (((uint8_t *)(long_word + 1)) > p_in_end)
                    {
                        status = CO_UTIL_PACK_IN_BUF_OVFLW;
                        break;
                    }

                    // Copy data
                    if (b_lsb)
                    {
                        co_write32p(p_out, co_htobl(*long_word));
                    }
                    else
                    {
                        co_write32p(p_out, co_htonl(*long_word));
                    }
                }

                // Move pointers
                p_in = (uint8_t *)(long_word + 1);
                p_out += 4;
            }
            break;

            default:
            {
                // data format error
                status = CO_UTIL_PACK_WRONG_FORMAT;
            }
            break;
        }
    }

    if (status == CO_UTIL_PACK_OK)
    {
        *out_len = (uint16_t)(p_out - out);
    }

    return (status);
}

uint8_t co_util_unpack(uint8_t *out, uint8_t *in, uint16_t *out_len, uint16_t in_len, const char *format)
{
    uint8_t status = CO_UTIL_PACK_OK;
    uint8_t *p_in = in;
    uint8_t *p_out = out;
    uint8_t *p_in_end = in + in_len;
    uint8_t *p_out_end = out + *out_len;
    char *cursor = (char *) format;
    bool b_copy = ((out != NULL) && (in != NULL));

    bool b_lsb = true;
    bool big_number = false;

    // Check if forced to little endian
    if (*cursor == '<')
    {
        b_lsb = true;
        cursor++;
    }
    else if (*cursor == '>')
    {
        b_lsb = false;
        cursor++;
    }

    ASSERT_ERR(format != NULL);

    while ((*cursor != '\0') && (status == CO_UTIL_PACK_OK))
    {
        uint16_t nb = 0;

        // Check if the new field is an array (starting with a number)
        if ((*cursor >= '0') && (*cursor <= '9'))
        {
            nb = co_util_read_array_size(&cursor);
        }

        // Parse the format string
        switch (*cursor++)
        {
            case ('G'):   // Big Number
            {
                big_number = true;
            } // No break
            case ('B'):   // Byte
            {
                // For arrays only
                if (nb > 1)
                {
                    if (b_copy)
                    {
                        // Check if enough space in input buffer to read
                        if ((p_in + nb) > p_in_end)
                        {
                            status = CO_UTIL_PACK_IN_BUF_OVFLW;
                            break;
                        }

                        // Check if enough space in out buffer to write
                        if ((p_out + nb) > p_out_end)
                        {
                            status = CO_UTIL_PACK_OUT_BUF_OVFLW;
                            break;
                        }

                        // Copy bytes
                        if (!big_number || (b_lsb == CPU_LE))
                        {
                            memcpy(p_out, p_in, nb);
                        }
                        // Swap bytes
                        else
                        {
                            co_bswap(p_out, p_in, nb);
                        }
                    }

                    // Move pointers
                    p_out += nb;
                    p_in += nb;
                }
                else
                {
                    if (b_copy)
                    {
                        // Check if enough space in input buffer to read
                        if ((p_in + 1) > p_in_end)
                        {
                            status = CO_UTIL_PACK_IN_BUF_OVFLW;
                            break;
                        }

                        // Check if enough space in out buffer to write
                        if ((p_out + 1) > p_out_end)
                        {
                            status = CO_UTIL_PACK_OUT_BUF_OVFLW;
                            break;
                        }

                        // Copy data
                        *p_out = *p_in;
                    }

                    // Move pointers
                    p_out++;
                    p_in++;
                }
                big_number = false;
            }
            break;

            case ('H'):   // Short Word
            {
                // Align data buffer to a 16-bits address
                uint16_t *short_word = (uint16_t *)CO_ALIGN2_HI((uint32_t)p_out);

                if (b_copy)
                {
                    // Check if enough space in input buffer to read
                    if ((p_in + 2) > p_in_end)
                    {
                        status = CO_UTIL_PACK_IN_BUF_OVFLW;
                        break;
                    }

                    // Check if enough space in out buffer to write
                    if (((uint8_t *)(short_word + 1)) > p_out_end)
                    {
                        status = CO_UTIL_PACK_OUT_BUF_OVFLW;
                        break;
                    }

                    // Copy data
                    if (b_lsb)
                    {
                        *short_word = co_btohs(co_read16p(p_in));
                    }
                    else
                    {
                        *short_word = co_ntohs(co_read16p(p_in));
                    }
                }

                // Move pointers
                p_out = (uint8_t *)(short_word + 1);
                p_in += 2;
            }
            break;

            case ('D'):   // 24 bits integer
            {
                // Align data buffer to a 32-bits address
                uint32_t *long_word = (uint32_t *)CO_ALIGN4_HI((uint32_t)p_out);

                if (b_copy)
                {
                    // Check if enough space in input buffer to read
                    if ((p_in + 3) > p_in_end)
                    {
                        status = CO_UTIL_PACK_IN_BUF_OVFLW;
                        break;
                    }

                    // Check if enough space in out buffer to write
                    if (((uint8_t *)(long_word + 1)) > p_out_end)
                    {
                        status = CO_UTIL_PACK_OUT_BUF_OVFLW;
                        break;
                    }

                    // Copy data
                    if (b_lsb)
                    {
                        *long_word = co_btoh24(co_read24p(p_in));
                    }
                    else
                    {
                        *long_word = co_ntoh24(co_read24p(p_in));
                    }
                }

                // Move pointers
                p_out = (uint8_t *)(long_word + 1);
                p_in += 3;
            }
            break;

            case ('L'):   // Long Word
            {
                // Align data buffer to a 32-bits address
                uint32_t *long_word = (uint32_t *)CO_ALIGN4_HI((uint32_t)p_out);

                if (b_copy)
                {
                    // Check if enough space in input buffer to read
                    if ((p_in + 4) > p_in_end)
                    {
                        status = CO_UTIL_PACK_IN_BUF_OVFLW;
                        break;
                    }

                    // Check if enough space in out buffer to write
                    if (((uint8_t *)(long_word + 1)) > p_out_end)
                    {
                        status = CO_UTIL_PACK_OUT_BUF_OVFLW;
                        break;
                    }

                    // Copy data
                    if (b_lsb)
                    {
                        *long_word = co_btohl(co_read32p(p_in));
                    }
                    else
                    {
                        *long_word = co_ntohl(co_read32p(p_in));
                    }
                }

                // Move pointers
                p_out = (uint8_t *)(long_word + 1);
                p_in += 4;
            }
            break;

            default:
            {
                // data format error
                status = CO_UTIL_PACK_WRONG_FORMAT;
            }
            break;
        }
    }

    // Check a potential mismatch between the theoretical (measured) input length and the given input length
    if (p_in > p_in_end)
    {
        status = CO_UTIL_PACK_IN_BUF_OVFLW;
    }

    // Return the total size needed for unpacked parameters
    *out_len = (uint16_t)(p_out - out);

    return (status);
}
#endif // (BLE_MESH)


/// @} CO_UTILS
