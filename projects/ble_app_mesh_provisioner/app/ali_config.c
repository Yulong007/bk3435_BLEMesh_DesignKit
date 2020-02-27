/**
*****************************************************************************************
*     Copyright(c) 2015, Beken. All rights reserved.
*****************************************************************************************
  * @file     user_config.c
  * @brief    Source file for user data.
  * @details  Load and store user data to flash.
  * @author   gang.cheng
  * @date     2018-12-20
  * @version  v1.0
  * *************************************************************************************
  */


#include "ali_config.h"
#include <string.h>
#include "flash.h"
#include "uart.h"
#include "mesh_log.h"

bool user_data_read_ali_secret_key(uint8_t *key)
{
    uint8_t l_key[FLASH_ALI_SECRET_LEN];
    flash_read_data(l_key, FLASH_ALI_DATA_ADDRESS + FLASH_ALI_SECRET_ADDR_OFFSET, FLASH_ALI_SECRET_LEN);
    for (int i = 0; i < FLASH_ALI_SECRET_LEN; i++)
    {
        key[i] = l_key[FLASH_ALI_SECRET_LEN - 1 - i];
    }

    MESH_APP_PRINT_INFO("%s key = %s\n", __func__, mesh_buffer_to_hex(key, FLASH_ALI_SECRET_LEN));
    return true;
}




uint32_t user_data_read_ali_product_id(void)
{
    uint32_t product_id;

    flash_read_data((uint8_t *)&product_id, FLASH_ALI_DATA_ADDRESS + FLASH_ALI_PRODUCT_ID_ADDR_OFFSET, FLASH_ALI_PRODUCT_ID_LEN);
    MESH_APP_PRINT_INFO("%s product_id = 0x%08x\n", __func__, product_id);


    return product_id;
}

bool user_data_read_ali_mac(uint8_t *addr, uint8_t mode)
{

    uint8_t l_addr[FLASH_ALI_MAC_ADDR_LEN];
    flash_read_data(l_addr, FLASH_ALI_DATA_ADDRESS + FLASH_ALI_MAC_ADDR_OFFSET, FLASH_ALI_MAC_ADDR_LEN);

    if (mode)
    {
        memcpy(addr, l_addr, FLASH_ALI_MAC_ADDR_LEN);

    }
    else
    {
        for (int i = 0; i < FLASH_ALI_MAC_ADDR_LEN; i++)
        {
            addr[i] = l_addr[FLASH_ALI_MAC_ADDR_LEN - 1 - i];
        }
    }
    MESH_APP_PRINT_INFO("%s addr = %s\n", __func__, mesh_buffer_to_hex(addr, FLASH_ALI_MAC_ADDR_LEN));
    return true;
}


bool user_data_contains_ali_data(void)
{
    uint8_t l_addr[FLASH_ALI_MAC_ADDR_LEN];
    flash_read_data(l_addr, FLASH_ALI_DATA_ADDRESS + FLASH_ALI_MAC_ADDR_OFFSET, FLASH_ALI_MAC_ADDR_LEN);

    MESH_APP_PRINT_INFO("l_addr %02x %02x %02x %02x %02x %02x\r\n", l_addr[0], l_addr[1], l_addr[2], l_addr[3], l_addr[4], l_addr[5]);
    if (l_addr[0]!=0xff ||l_addr[1]!=0xff||
            l_addr[2]!=0xff||l_addr[3]!=0xff||
            l_addr[4]!=0xff||l_addr[5]!=0xff )
    {
        return true;

    }
    else
    {
        return false;
    }

}






