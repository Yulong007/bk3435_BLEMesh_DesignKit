
#include <stdint.h>
#include <string.h>
#include "att.h"

#include "co_error.h"
#include "co_utils.h"
#include "co_math.h"

#include "gattm.h"
#include "attm_db.h"

#include "ke_timer.h"
#include "ke_mem.h"
#include "ke_msg.h"
#include "ke_task.h"
#include "uart.h"

#include "../gattm/gattm_int.h" // Access to the internal variable required



#define USER_SERV_UUID_128  {0xFB, 0x34, 0x9B, 0x5F, 0x80, 0x00, 0x00, 0x80, \
                                  0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}

#define USER_CHAR_UUID_128  {0x16, 0x0A, 0x10, 0x40, 0xD1, 0x9F, 0x4C, 0x6C, \
                                  0xB4, 0x55, 0xE3, 0xF7, 0XCF, 0X84, 0x00, 0x00}




void attm_serv_convert_to128(uint8_t *uuid128, const uint8_t *uuid, uint8_t uuid_len)
{
    uint8_t auc_128UUIDBase[ATT_UUID_128_LEN] = USER_SERV_UUID_128;
    uint8_t cursor = 0;

    if ((uuid_len == ATT_UUID_32_LEN) || (uuid_len == ATT_UUID_16_LEN))
    {
        /* place the UUID on 12th to 15th location of UUID */
        cursor = 12;
    }
    else
    {
        /* we consider it's 128 bits UUID */
        uuid_len  = ATT_UUID_128_LEN;
    }

    /* place the UUID on 12th to 15th location of UUID */
    memcpy(&(auc_128UUIDBase[cursor]), uuid, uuid_len);

    /* update value */
    memcpy(&uuid128[0], &auc_128UUIDBase[0], ATT_UUID_128_LEN);
}


void attm_char_convert_to128(uint8_t *uuid128, const uint8_t *uuid, uint8_t uuid_len)
{
    uint8_t auc_128UUIDBase[ATT_UUID_128_LEN] = USER_CHAR_UUID_128;
    uint8_t cursor = 0;

    if ((uuid_len == ATT_UUID_32_LEN) || (uuid_len == ATT_UUID_16_LEN))
    {
        /* place the UUID on 12th to 15th location of UUID */
        cursor = 14;
    }
    else
    {
        /* we consider it's 128 bits UUID */
        uuid_len  = ATT_UUID_128_LEN;
    }

    /* place the UUID on 12th to 15th location of UUID */
    memcpy(&(auc_128UUIDBase[cursor]), uuid, uuid_len);

    /* update value */
    memcpy(&uuid128[0], &auc_128UUIDBase[0], ATT_UUID_128_LEN);
}


uint8_t attm_util_svc_create_db128(uint16_t *shdl, uint16_t uuid, uint8_t *cfg_flag, uint8_t max_nb_att, uint8_t *att_tbl, ke_task_id_t const dest_id, const struct attm_desc *att_db, uint8_t svc_perm)
{
    uint8_t nb_att = 0;
    uint8_t i;
    uint8_t status = ATT_ERR_NO_ERROR;
    struct gattm_svc_desc *svc_desc;



    //Compute number of attributes and maximal payload size
    for (i = 1; i < max_nb_att; i++)
    {
        // check within db_cfg flag if attribute is enabled or not
        if ((cfg_flag == NULL) || (((cfg_flag[i / 8] >> (i % 8)) & 1) == 1))
        {
            // increment number of attribute to add
            nb_att++;
        }
    }


    // Allocate service information
    svc_desc = (struct gattm_svc_desc *) ke_malloc(
                   sizeof(struct gattm_svc_desc) + (sizeof(struct gattm_att_desc) * (nb_att)), KE_MEM_NON_RETENTION);

    // Initialize service info
    svc_desc->start_hdl = *shdl;
    svc_desc->nb_att = nb_att;
    svc_desc->task_id = dest_id;
    // ensure that service has a 128 bits UUID
    svc_desc->perm = (svc_perm & ~( PERM_MASK_SVC_MI));
    svc_desc->perm |= (0x02 << 5);

    // copy UUID
    attm_serv_convert_to128(svc_desc->uuid, (uint8_t *)&uuid, ATT_UUID_16_LEN);

    // Set Attribute parameters
    nb_att = 0;
    for (i = 1; i < max_nb_att; i++)
    {
        // check within db_cfg flag if attribute is enabled or not
        if ((cfg_flag == NULL) || (((cfg_flag[i / 8] >> (i % 8)) & 1) == 1))
        {
            // ensure that service has a 128 bits UUID
            svc_desc->atts[nb_att].max_len  = att_db[i].max_size;
            svc_desc->atts[nb_att].ext_perm = (att_db[i].ext_perm);
            svc_desc->atts[nb_att].perm     = att_db[i].perm;

            if ( (ATT_UUID_LEN(PERM_GET(svc_desc->atts[nb_att].ext_perm, UUID_LEN))) ==  16)
            {
                attm_char_convert_to128(svc_desc->atts[nb_att].uuid, (uint8_t *) & (att_db[i].uuid), ATT_UUID_16_LEN);
            }
            else
            {
                memcpy(svc_desc->atts[nb_att].uuid, &(att_db[i].uuid), ATT_UUID_16_LEN);
            }

            // increment number of attributes
            nb_att++;
        }
    }


    // add service in database
    status = attmdb_add_service(svc_desc);

    // if service added
    if (status == ATT_ERR_NO_ERROR)
    {
        // return start handle
        *shdl = svc_desc->start_hdl;

        // update attribute table mapping
        nb_att = 0;
        for (i = 0; (i < max_nb_att) && (att_tbl != NULL); i++)
        {
            // check within db_cfg flag if attribute is enabled or not
            if ((cfg_flag == NULL) || (((cfg_flag[i / 8] >> (i % 8)) & 1) == 1))
            {
                //Save handle offset in attribute table
                att_tbl[i] = *shdl + nb_att;
                // increment number of attributes
                nb_att++;
            }
        }
    }

    // free service description
    ke_free(svc_desc);

    return (status);
}
