/**
*****************************************************************************************
*     Copyright(c) 2015, Beken. All rights reserved.
*****************************************************************************************
  * @file     ali_config.c
  * @brief    Source file for user data.
  * @details  Load and store user data to flash.
  * @author   gang.cheng
  * @date     2018-12-20
  * @version  v1.0
  * *************************************************************************************
  */





#ifndef _USER_CONFIG_H
#define _USER_CONFIG_H

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief alibaba data storage address
 * @note this address must in flash map and
 *  must be 4k alignment
 */
#define FLASH_ALI_DATA_ADDRESS    0x7E000

#define FLASH_ALI_MAC_ADDR_OFFSET 0
#define FLASH_ALI_PRODUCT_ID_ADDR_OFFSET 6
#define FLASH_ALI_SECRET_ADDR_OFFSET 10


#define FLASH_ALI_MAC_ADDR_LEN 6
#define FLASH_ALI_PRODUCT_ID_LEN 4
#define FLASH_ALI_SECRET_LEN 16



#pragma pack(1)
typedef struct
{
    uint16_t cid;
    struct
    {
        uint8_t adv_ver: 4;
        uint8_t sec: 1;
        uint8_t ota: 1;
        uint8_t bt_ver: 2; //!< 0 bt4.0, 1 bt4.2, 2 bt5.0, 3 higher
    } pid;
    uint32_t product_id;
    uint8_t mac_addr[6];

    uint8_t feature_flag;
    uint8_t rfu[2];
} ali_uuid_t;

#pragma pack()

/**
 * @brief read alibaba secret key from flash
 * @param[out] key: alibaba specified secret key
 * @retval TRUE: read data from flash success
 * @retval FALSE: read data from flash failed
 */
bool user_data_read_ali_secret_key(uint8_t *key);

/**
 * @brief read alibaba product id from flash
 * @param[out] key: alibaba specified secret key
 * @retval TRUE: read data from flash success
 * @retval FALSE: read data from flash failed
 */
uint32_t user_data_read_ali_product_id(void);

/**
 * @brief read alibaba mac_addr from flash
 * @param[out] addr: alibaba specified mac_addr
 * @retval TRUE: read data from flash success
 * @retval FALSE: read data from flash failed
 */

bool user_data_read_ali_mac(uint8_t *addr, uint8_t mode);

/**
 * @brief write alibaba specified data to flash
 * @param[in] id: alibaba specified product id
 * @param[in] key: alibaba specified secret key
 * @retval TRUE: write data to flash success
 * @retval FALSE: write data to flash failed
 */
bool user_data_write_ali_data(uint32_t id, const uint8_t *key);

/**
 * @brief check whether flash contains alibaba private date or not
 * @retval TRUE: flash contains alibaba data
 * @retval FALSE: flash has no alibaba data
 */
bool user_data_contains_ali_data(void);


#endif


