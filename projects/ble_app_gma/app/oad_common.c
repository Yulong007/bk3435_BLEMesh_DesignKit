#include "rwip_config.h"
#include "oad_common.h"
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
#include "wdt.h"
#include "mesh_log.h"

uint32_t crc32_table[256];
img_hdr_t hdr_back;


int make_crc32_table(uint32_t init)
{
    uint32_t c;
    int i = 0;
    int bit = 0;
    for (i = 0; i < 256; i++)
    {
        c = (uint32_t)i;
        for (bit = 0; bit < 8; bit++)
        {
            if (c&1)
            {
                c = (c>>1)^(init);//0xEDB88320
            }
            else
            {
                c = c >> 1;
            }
        }
        crc32_table[i] = c;

    }

    return 0;
}
uint32_t make_crc32(uint32_t crc, unsigned char *string, uint32_t size)
{
    while (size--)
    {
        crc = (crc >> 8)^(crc32_table[(crc^*string++)&0xff]);
    }
    return crc;
}




void oads_erase_backup_sec(void)
{
    uint32_t addr;
    MESH_APP_PRINT_INFO("oads_erase_backup_sec\r\n");

    //176k
    {
        MESH_APP_PRINT_INFO("APP_PART_UID bim_erase_backup_sec \r\n");
        uint32_t addr = SEC_IMAGE_BACKUP_ALLOC_START_FADDR ;

        MESH_APP_PRINT_INFO("START addr = 0x%x", addr);

		
        for (; addr < SEC_IMAGE_BACKUP_ALLOC_END_FADDR; addr+= FLASH_SEC_SIZE )
        {
			wdt_feed(0x7FFF);
            flash_erase(FLASH_MAIN_BASE_ADDR, addr, FLASH_SEC_SIZE, NULL);
        }

        MESH_APP_PRINT_INFO("END addr = 0x%x", addr);

    }

    return;
}

uint32_t oad_get_psec_backup_header(void)
{
    MESH_APP_PRINT_INFO("udi_get_psec_backup_header addr = 0x%x", SEC_IMAGE_BACKUP_OAD_HEADER_FADDR);
    flash_read(0, SEC_IMAGE_BACKUP_OAD_HEADER_FADDR, sizeof(img_hdr_t), (uint8_t *)&hdr_back, NULL);

    MESH_APP_PRINT_INFO("hdr_back.crc = 0x%x\r\n", hdr_back.crc);

    MESH_APP_PRINT_INFO("hdr_back.crc_status = 0x%x\r\n", hdr_back.crc_status);

    MESH_APP_PRINT_INFO("hdr_back.len = 0x%x\r\n", hdr_back.len);

    MESH_APP_PRINT_INFO("hdr_back.rom_ver = 0x%x\r\n", hdr_back.rom_ver);

    MESH_APP_PRINT_INFO("hdr_back.sec_status = 0x%x\r\n", hdr_back.sec_status);

    MESH_APP_PRINT_INFO("hdr_back.ver = 0x%x\r\n", hdr_back.ver);

    MESH_APP_PRINT_INFO("hdr_back.uid = 0x%x\r\n", hdr_back.uid);

    return 0;
}
 
uint32_t calc_backup_sec_crc(void)
{
    MESH_APP_PRINT_INFO("%s\r\n", __func__);
    uint8_t data[BLOCK_SIZE];
    uint8_t tmp_data[BLOCK_SIZE];
    uint16_t block_total;
    uint32_t read_addr;
    uint32_t calcuCrc = 0xffffffff;
    oad_get_psec_backup_header();
    make_crc32_table(0xEDB88320);
    block_total = hdr_back.len / 4 - 1;
    read_addr = SEC_IMAGE_BACKUP_OAD_IMAGE_FADDR;
    MESH_APP_PRINT_INFO("read start addr = 0x%x\r\n", read_addr);
    MESH_APP_PRINT_INFO("block_total= 0x%x\r\n", block_total);

    for (uint32_t i = 0; i < block_total; i++)
    {
        flash_read(0, read_addr, BLOCK_SIZE, data, NULL);
        flash_read(0, read_addr, BLOCK_SIZE, tmp_data, NULL);
        //for (int a=0; a<BLOCK_SIZE; a++)
        //{
        //    MESH_APP_PRINT_INFO("%02x ", tmp_data[a]);
        //}
        //MESH_APP_PRINT_INFO("\r\n");
        calcuCrc = make_crc32(calcuCrc, data, BLOCK_SIZE);
        read_addr+= BLOCK_SIZE;

        if (memcmp(data, tmp_data, BLOCK_SIZE) != 0)
        {
            MESH_APP_PRINT_INFO("read_addr error = 0x08%x\r\n", read_addr);
            for (int a=0; a<BLOCK_SIZE; a++)
            {
                MESH_APP_PRINT_INFO("tmp_data = %02x,data = %02x \r\n", tmp_data[a], data[a]);
            }
        }
    }
    MESH_APP_PRINT_INFO("read end addr = 0x%x,calcuCrc = 0x%08x\r\n", read_addr, calcuCrc);

    MESH_APP_PRINT_INFO("return crc = 0x%x\r\n", calcuCrc);
    if (hdr_back.crc == calcuCrc)
    {
        hdr_back.crc_status = CRC_CHECK_OK;
        flash_write(0, SEC_IMAGE_BACKUP_OAD_HEADER_FADDR, sizeof(img_hdr_t), (uint8_t *)&hdr_back, NULL);

    }

    return calcuCrc;
}




