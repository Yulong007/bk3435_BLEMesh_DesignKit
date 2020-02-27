/**
 ****************************************************************************************
 *
 * @file oad_common.h
 *
 * @brief Header file - OTA Profile Server Role
 *
 * Copyright (C) beken 2009-2015
 *
 *
 ****************************************************************************************
 */
#ifndef _OTA_COMMON_H_
#define _OTA_COMMON_H_

#include <string.h>
#include "rwip_config.h"
#include "attm.h"
#include "oads.h"
#include "oads_task.h"
#include "ke_mem.h"
#include "prf_utils.h"
#include "prf.h"
#include "co_utils.h"
#include "flash.h"
#include "uart.h"
#include "ke.h"
#include "icu.h"
#include "wdt.h"                 // Platform Definitions

/*
 * DEFINES
 ****************************************************************************************
 */


#define FLASH_SEC_SIZE                       (0X1000)
#define FLASH_HALF_BLOCK_SIZE                (0X8000) // 32KB
#define FLASH_ONE_BLOCK_SIZE                 (0X10000) // 64KB

#define SEC_MAX_FSIZE_4K                         (180) //KB 
#define SEC_MAX_FSIZE_APP_BLOCK              (0x2C00) // 176 * 1024 / 16
#define SEC_MAX_FSIZE_STACK_APP_BLOCK            (0x4F00) // 316 * 1024 / 16 (stack + app)

#define SEC_MAX_FSIZE_BYTE                   (0X2C000) //176 * 1024


#define SEC_IMAGE_STACK_RUN_CADDR            (0x2F00)
#define SEC_IMAGE_STACK_RUN_FADDR            (0x31F0)
#define SEC_IMAGE_STACK_OAD_HEADER_FADDR     (0x31E0)

#define SEC_IMAGE_APP_RUN_CADDR              (0x23E00)
#define SEC_IMAGE_APP_RUN_FADDR              (0x261E0)
#define SEC_IMAGE_APP_OAD_HEADER_FADDR       (0x261D0)


#define SEC_IMAGE_STACK_ALLOC_START_FADDR    (0x3000)  //(12KB)
#define SEC_IMAGE_ALLOC_END_FADDR                   (0x52000) //(324KB)


#define SEC_IMAGE_APP_ALLOC_START_FADDR      (0x26000)
#define SEC_IMAGE_APP_ALLOC_END_FADDR        (SEC_IMAGE_ALLOC_END_FADDR)

#define SEC_IMAGE_BACKUP_OAD_HEADER_FADDR            (0x52000) //328kb * 1024
#define SEC_IMAGE_BACKUP_OAD_IMAGE_FADDR             (0x52010) //328kb * 1024 + 0X10
#define SEC_IMAGE_BACKUP_ALLOC_START_FADDR       (0x52000) //(328KB)
#define SEC_IMAGE_BACKUP_ALLOC_END_FADDR             (0x7E000) //(504KB)


#define IMAGE_TOTAL_LEN_64K                 0x4000
#define IMAGE_TOTAL_LEN_128K                0x8000
#define IMAGE_TOTAL_LEN_192K                0xC000


#define OAD_APP_PART_UID                    (0x42424242)
#define OAD_APP_STACK_UID                   (0x53535353)


//OADS.C USED

// The Image is transporte in 16-byte blocks in order to avoid using blob operations.
//需要根据实际的值修改
#define OAD_BLOCK_SIZE        16
#define HAL_FLASH_PAGE_SIZE   256

#define HAL_FLASH_WORD_SIZE   4
#define OAD_BLOCKS_PER_PAGE  (HAL_FLASH_PAGE_SIZE / OAD_BLOCK_SIZE)

#define OAD_BLOCK_APP_MAX       (0x2d00)  //(128K *1024*4) /64
#define OAD_BLOCK_STACK_MAX   (0x4f00)


/*

            |------------------|
            |    NVDS(4KB)     |
            |                                    |
0x0007F000  |------------------| F_ADDR = 0x0007F000
            |                                    |
            |   USER_CONF(4KB) |
0x0007E000  |------------------| F_ADDR = 0x0007E000
            |   BACK_UP(176KB) |
            |                                    |
0x00052000  |------------------| F_ADDR = 0x00052000
            |                                    |
            |   APP(176KB)     |
            |                                    |
0x00026000  |------------------| C_ADDR = 0x23E00,F_ADDR = 0x261E0 (APP)
            |                                    |
            |  STACK(140KB)    |
            |                                    |
0x00003000  |------------------| C_ADDR = 0x2F00,F_ADDR = 0x31F0 (STACK)
            |                                    |
            |    BIM(12KB)     |
            |                                    |
0x00000000  |------------------|

*/

/*********************************************************************
 * TYPEDEFS
 */
// The Image Header will not be encrypted, but it will be included in a Signature.
typedef struct
{
    // Secure OAD uses the Signature for image validation instead of calculating a CRC, but the use
    // of CRC==CRC-Shadow for quick boot-up determination of a validated image is still used.
    uint32_t crc;       // CRC must not be 0x0000 or 0xFFFF.
    // User-defined Image Version Number - default logic uses simple a '!=' comparison to start an OAD.
    uint16_t ver;

    uint16_t len;        // Image length in 4-byte blocks (i.e. HAL_FLASH_WORD_SIZE blocks).

    uint32_t  uid;       // User-defined Image Identification bytes.
    uint8_t  crc_status;     // cur image crc status
    uint8_t  sec_status;     // cur image sec status
    uint16_t  rom_ver;     // Rom ver.
} img_hdr_t;



#define  BLOCK_SIZE         0X10

#define CRC_UNCHECK         0xFF
#define CRC_CHECK_OK        0xAA
#define CRC_CHECK_FAIL      0x55

#define SECT_UNKNOW         0xFF
#define SECT_NORMAL         0xAA
#define SECT_ABNORMAL       0x55

enum
{
    SSTATUS_SECT_NORMAL = 0,
    SSTATUS_SECT_ERASED,
    SSTATUS_SECT_ABNORMAL,
    SSTATUS_SECT_UNKOWN,
    SSTATUS_SECT_DIFF_ROM_VER,
};


extern img_hdr_t hdr_back;

uint32_t calc_backup_sec_crc(void);
uint32_t oad_get_psec_backup_header(void);
int make_crc32_table(uint32_t init);
uint32_t make_crc32(uint32_t crc, unsigned char *string, uint32_t size);
void oads_erase_backup_sec(void);

#endif


