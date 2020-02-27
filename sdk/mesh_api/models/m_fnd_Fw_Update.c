

/**
 ****************************************************************************************
 * @file m_fnd_Fw_Update.c
 *
 * @brief Mesh  Firmware Update / Distribution Server Model
 *
 * Copyright (C) Beken 2018-2019
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup M_FND_FIRMWARE
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "m_fnd_int.h"      // Internal defines
#include <string.h>         // For use of memcpy function
#include "co_math.h"        // For use of CO_BIT macros
#include "m_lay.h"
#include "m_fnd_Fw_Update.h"
#include "flash.h"
#include "app.h"
#include "m_fnd_BLOB_Transfer.h"
#include "oad_common.h"
#include "mesh_log.h"


uint32_t oad_calcuCrc;


/*
 * DEFINES
 ****************************************************************************************
 */

#define M_FND_FW_UPDATE_MODEL_N    (3)


/// Firmware Update Server SIG Model ID
#define M_FND_FW_MODEL_UPDATE_ID                        (0xFE00)



/// Length of Firmware Update Model Info status message
#define M_FND_FW_UPDATE_MODEL_INFO_STATUS_LEN                       (2 + 4)//Min 6

/// Length of  Firmware Update Model Update status message
#define M_FND_FW_UPDATE_MODEL_UPDATE_STATUS_LEN          (16)


/*
 * MACROS
 ****************************************************************************************
 */



/*
 * MESSAGE STRUCTURES
 ****************************************************************************************
 */




/*
 * STRUCTURES
 ****************************************************************************************
 */

// Foundation model layer environment structure
typedef struct m_fnd_fw_update_env
{
    // List of buffers containing messages to process
    co_list_t   process_queue;
    // Delayed job structure
    mal_djob_t djob;
    // Opcode actually handled
    uint8_t     opcode;
    // Model local index
    m_lid_t     model_lid;

    uint8_t *update_url; //!< set by upper
    uint16_t url_len;

    uint8_t Phase;

    uint16_t Company_ID;
    // Unique firmware identifier ,local version
    uint8_t Firmware_ID[FW_UPDATE_FW_ID_LEN];
    uint8_t Firmware_ID_new[FW_UPDATE_FW_ID_LEN];
    // Unique object identifier
    uint8_t Object_ID[8];

} m_fnd_fw_update_env_t;


/*
 * PRIVATE VARIABLES
 ****************************************************************************************
 */

// Foundation model layer environment
__STATIC m_fnd_fw_update_env_t *p_m_fnd_fw_update_env;


/*
 * STATIC FUNCTION DEFINITIONS
 ****************************************************************************************
 */

__STATIC void m_fnd_fw_update_process_next(void);



/*
 * STATIC FUNCTIONS
 ****************************************************************************************
 */


/**
 ****************************************************************************************
 * @brief Allocate a buffer.
 *
 * @param[out] pp_buf
 * @param[in]  data_len
 *
 * @return M_ERR_NO_ERR if buffer has been successfully allocated, else another error code.
 ****************************************************************************************
 */
__STATIC uint16_t m_fnd_fw_update_buf_alloc(mesh_tb_buf_t **pp_buf, uint16_t data_len)
{
    // Try to allocate a buffer and return the status
    return (mesh_tb_buf_alloc(pp_buf, M_FND_BUF_HEAD_LEN, data_len, 0));
}


/**
 ****************************************************************************************
 * @brief Push a buffer for transmission (response).
 *
 * @param[in] p_buf     Pointer to the buffer containing the message to send.
 * @param[in] opcode    Operation code
 ****************************************************************************************
 */
__STATIC uint16_t m_fnd_fw_update_send(mesh_tb_buf_t *p_buf, uint16_t opcode)
{
    uint16_t status;
    // Retrieve buffer containing the message for which a response is sent
    mesh_tb_buf_t *p_buf_req = (mesh_tb_buf_t *)co_list_pick(&p_m_fnd_fw_update_env->process_queue);
    // Get buffer environment
    m_lay_buf_env_t *p_env = (m_lay_buf_env_t *)&p_buf_req->env[0];
    // Use request source address as destination address
    uint16_t dst = p_env->src;

    MESH_MODEL_PRINT_DEBUG("%s\r\n", __func__);
    // Send the provided message
    status = m_api_model_rsp_send(p_m_fnd_fw_update_env->model_lid, (uint32_t)opcode, 0,
                                  p_buf, p_env->app_lid, dst, false, false);

    return status;
}

/*
// Firmware Update Info Status message structure
typedef struct m_fnd_fw_update_info_status
{
    // Company identifier
    uint16_t Company_ID;
    // Unique firmware identifier
    uint8_t Firmware_ID[1 + M_FND_FW_UPDATE_MODEL_N];
    // URL for update source (optional)
    uint8_t Update_URL[M_FND_FW_UPDATE_MODEL_N];
} m_fnd_fw_update_info_status_t;
*/
/*
__STATIC uint8_t m_fnd_fw_update_get_info_status(m_fnd_fw_update_info_status_t *status)
{
    MESH_MODEL_PRINT_DEBUG("%s\r\n",__func__);
    status->Company_ID = p_m_fnd_fw_update_env->Company_ID;

    memcpy(status->Firmware_ID,p_m_fnd_fw_update_env->Firmware_ID,FW_UPDATE_FW_ID_LEN);
    status->Update_URL[0] = 0x00;
   // status->Update_URL[1] = 0x00;
   // status->Update_URL[2] = 0x00;
    return 0;
}
*/
/*

// Firmware Update Status message structure
typedef struct m_fnd_fw_update_status
{
    // Status code of the operation
    uint8_t Status;
    // Phase of the update
    uint8_t Phase_Add_Info; //
    // Additional Information
    //  uint8_t Additional_Information :5;  // 5 bits
    // Company identifier
    uint16_t Company_ID;
    // Unique firmware identifier
    uint8_t Firmware_ID[1 + M_FND_FW_UPDATE_MODEL_N];
    // Unique object identifier
    uint8_t Object_ID[8];
} m_fnd_fw_update_status_t;

*/


/*enum fw_update_phase
{
    //No DFU update in progress
    UPDATE_PHASE_IDLE = 0x00,
    //DFU update is prepared and awaiting start
    UPDATE_PHASE_PREPARED = 0x01,
    //DFU update is in progress
    UPDATE_PHASE_IN_PROGRESS = 0x02,
    //DFU upload is finished and waiting to be apply
    UPDATE_PHASE_WAITING_APPLY = 0x03,

};
*/

/*
__STATIC uint8_t m_fnd_fw_update_get_update_status(m_fnd_fw_update_status_t *status)
{
    MESH_MODEL_PRINT_DEBUG("%s\r\n",__func__);
    status->Status = FW_UPDATE_STAT_SUCCESS;
    status->Phase = FW_UPDATE_PHASE_PREPARED;//0x01;
    status->Additional_Information  = 0;

    status->Company_ID = p_m_fnd_fw_update_env->Company_ID;
    memcpy(status->Firmware_ID,p_m_fnd_fw_update_env->Firmware_ID,FW_UPDATE_FW_ID_LEN);

    memcpy(status->Object_ID,p_m_fnd_fw_update_env->Object_ID,8);

    return 0;
}
*/

/*
__STATIC uint8_t m_fnd_fw_update_get_update_get_status(m_fnd_fw_update_status_t *status)
{
    MESH_MODEL_PRINT_DEBUG("%s\r\n",__func__);
    status->Status = FW_UPDATE_STAT_SUCCESS;
    status->Phase  = FW_UPDATE_PHASE_WAITING_APPLY;//0x03;
    status->Additional_Information = 0;
    status->Company_ID = 0x01a8;
    memcpy(status->Firmware_ID,p_m_fnd_fw_update_env->Firmware_ID,FW_UPDATE_FW_ID_LEN);

    memcpy(status->Object_ID,p_m_fnd_fw_update_env->Object_ID,8);
    return 0;
}
__STATIC uint8_t m_fnd_fw_update_get_update_start_status(m_fnd_fw_update_status_t *status)
{
    MESH_MODEL_PRINT_DEBUG("%s\r\n",__func__);
    status->Status = 0;
    status->Phase = FW_UPDATE_PHASE_IN_PROGRESS;//0x02;
    status->Additional_Information = 0;
    status->Company_ID = 0x01a8;
    memcpy(status->Firmware_ID,p_m_fnd_fw_update_env->Firmware_ID,FW_UPDATE_FW_ID_LEN);

    memcpy(status->Object_ID,p_m_fnd_fw_update_env->Object_ID,8);
    return 0;
}
*/


/*
__STATIC uint8_t m_fnd_fw_update_get_update_apply_status(m_fnd_fw_update_status_t *status)
{
    MESH_MODEL_PRINT_DEBUG("%s\r\n",__func__);
    if(hdr_back.crc == oad_calcuCrc)
    {
        status->Status = FW_UPDATE_STAT_SUCCESS;//0x00;
    }else
    {
    status->Status  = FW_UPDATE_STAT_NEWER_FW_VERSION_PRESENT; // 0x04
    }

    status->Phase = FW_UPDATE_PHASE_IDLE;//0x00;
    status->Additional_Information = 0;
    status->Company_ID = p_m_fnd_fw_update_env->Company_ID;
    memcpy(status->Firmware_ID,p_m_fnd_fw_update_env->Firmware_ID,FW_UPDATE_FW_ID_LEN);

    memcpy(status->Object_ID,p_m_fnd_fw_update_env->Object_ID,8);
    return 0;
}

*/
/**
 ****************************************************************************************
 * @brief Prepare and send a Firmware Update Server Model Info Status message.
 *
 ****************************************************************************************
 */
/*
// Firmware Update Info Status message structure
typedef struct m_fnd_fw_update_info_status
{
    // Company identifier
    uint16_t Company_ID;
    // Unique firmware identifier
    uint8_t Firmware_ID[FW_UPDATE_FW_ID_LEN];
    // URL for update source (optional)
    uint8_t Update_URL[1];
} m_fnd_fw_update_info_status_t;
*/
__STATIC void m_fnd_fw_update_send_model_info_status(uint16_t company_id, uint8_t *firmware_id, uint8_t *update_url, uint8_t url_len)
{
    MESH_MODEL_PRINT_DEBUG("%s\r\n", __func__);
    // Pointer to the buffer that will contain the message
    mesh_tb_buf_t *p_buf_status;
    // Data length
    uint16_t data_length = M_FND_FW_UPDATE_MODEL_INFO_STATUS_LEN + url_len;

    MESH_MODEL_PRINT_DEBUG("data_length = %d\r\n", data_length);
    if (m_fnd_fw_update_buf_alloc(&p_buf_status, data_length) == MESH_ERR_NO_ERROR)
    {
        // Get pointer to data

        m_fnd_fw_update_info_status_t *status = (m_fnd_fw_update_info_status_t *)MESH_TB_BUF_DATA(p_buf_status);
        // Write status
        status->Company_ID = company_id;


        memcpy(status->Firmware_ID, p_m_fnd_fw_update_env->Firmware_ID, FW_UPDATE_FW_ID_LEN);
        if (url_len)
        {
            memcpy(status->Update_URL, update_url, url_len);
        }
        // if (0 != memcmp(p_m_fnd_fw_update_env->Firmware_ID_new, p_m_fnd_fw_update_env->Firmware_ID, FW_UPDATE_FW_ID_LEN))
        {
            //  memcpy(status->Firmware_ID,p_m_fnd_fw_update_env->Firmware_ID_new,FW_UPDATE_FW_ID_LEN);
        }

        MESH_MODEL_PRINT_DEBUG("Company_ID = 0x%02x\r\n", status->Company_ID);
        MESH_MODEL_PRINT_DEBUG("Firmware_ID = 0x%02x%02x%02x%02x\r\n", status->Firmware_ID[0], status->Firmware_ID[1], status->Firmware_ID[2], status->Firmware_ID[3]);
        if (p_m_fnd_fw_update_env->url_len)
        {
            for (int i = 0; i < url_len; i++)
            {
                MESH_MODEL_PRINT_DEBUG("Update_URL[%d] = 0x%x\r\n", i, status->Update_URL[i]);
            }
        }

        // Send the message
        m_fnd_fw_update_send(p_buf_status, M_FND_FW_UPDATE_2B_OPCODE(M_FND_FW_UPDATE_OPCODE_INFO_STATUS));
    }
    else
    {
        MESH_MODEL_PRINT_DEBUG("m_fnd_fw_update_buf_alloc fail 0\r\n");
    }

}



/**
 ****************************************************************************************
 * @brief Prepare and send a Firmware Update Server Model Info Status message.
 *
 ****************************************************************************************
 */

/*
// Firmware Update Status message structure
typedef struct m_fnd_fw_update_status
{
    // Status code of the operation
    uint8_t Status;
    // Phase of the update
    uint8_t Phase : 3;  // 3 bits
    // Additional Information
    uint8_t Additional_Information :5;  // 5 bits
    // Company identifier
    uint16_t Company_ID;
    // Unique firmware identifier
    uint8_t Firmware_ID[FW_UPDATE_FW_ID_LEN];
    // Unique object identifier
    uint8_t Object_ID[8];
} m_fnd_fw_update_status_t;*/

__STATIC void m_fnd_fw_update_send_model_update_status(uint8_t stat, uint8_t phase, uint8_t addi_info, uint16_t company_id, uint8_t *firmware_id, uint8_t *object_id)
{
    MESH_MODEL_PRINT_DEBUG("%s\r\n", __func__);
    // Pointer to the buffer that will contain the message
    mesh_tb_buf_t *p_buf_status;
    // Data length
    uint16_t data_length = M_FND_FW_UPDATE_MODEL_UPDATE_STATUS_LEN;


    if (m_fnd_fw_update_buf_alloc(&p_buf_status, data_length) == MESH_ERR_NO_ERROR)
    {
        // Get pointer to data
        m_fnd_fw_update_status_t *status = (m_fnd_fw_update_status_t *)MESH_TB_BUF_DATA(p_buf_status);
        // Write status
        status->Status = stat;//FW_UPDATE_STAT_SUCCESS;
        status->Phase  = phase;//FW_UPDATE_PHASE_WAITING_APPLY;//0x03;
        status->Additional_Information = addi_info;
        status->Company_ID = company_id;//p_m_fnd_fw_update_env->Company_ID;
        memcpy(status->Firmware_ID, firmware_id, FW_UPDATE_FW_ID_LEN);

        memcpy(status->Object_ID, object_id, 8);

        MESH_MODEL_PRINT_DEBUG("Status = 0x%x\r\n", status->Status);
        MESH_MODEL_PRINT_DEBUG("Phase_Add_Info = 0x%x\r\n", status->Phase);
        MESH_MODEL_PRINT_DEBUG("Company_ID = 0x%x\r\n", status->Company_ID);
        MESH_MODEL_PRINT_DEBUG("Firmware_ID = 0x%02x,%02x,%02x,%02x\r\n", status->Firmware_ID[0], status->Firmware_ID[1], status->Firmware_ID[2], status->Firmware_ID[3]);

        MESH_MODEL_PRINT_DEBUG("Object_ID = 0x%x,%x,%x,%x,", status->Object_ID[0], status->Object_ID[1], status->Object_ID[2], status->Object_ID[3]);
        MESH_MODEL_PRINT_DEBUG("%x,%x,%x,%x \r\n", status->Object_ID[4], status->Object_ID[5], status->Object_ID[6], status->Object_ID[7]);

        // Send the message
        m_fnd_fw_update_send(p_buf_status, M_FND_FW_UPDATE_2B_OPCODE(M_FND_FW_UPDATE_OPCODE_UPDATE_STATUS));
    }
    else
    {
        MESH_MODEL_PRINT_DEBUG("m_fnd_fw_update_buf_alloc fail 1\r\n");
    }
}
/*
__STATIC void m_fnd_fw_update_send_model_update_prepare_status()
{
    MESH_MODEL_PRINT_DEBUG("%s\r\n",__func__);
    // Pointer to the buffer that will contain the message
    mesh_tb_buf_t *p_buf_status;
    // Data length
    uint16_t data_length = M_FND_FW_UPDATE_MODEL_UPDATE_STATUS_LEN;

    MESH_MODEL_PRINT_DEBUG("data_length = %d\r\n",data_length);
    if (m_fnd_fw_update_buf_alloc(&p_buf_status, data_length) == MESH_ERR_NO_ERROR)
    {
        // Get pointer to data
        m_fnd_fw_update_status_t *status = (m_fnd_fw_update_status_t *)MESH_TB_BUF_DATA(p_buf_status);
        // Write status

        status->Status = FW_UPDATE_STAT_SUCCESS;
        status->Phase = FW_UPDATE_PHASE_PREPARED;//0x01;
        status->Additional_Information  = 0;

        status->Company_ID = p_m_fnd_fw_update_env->Company_ID;
        memcpy(status->Firmware_ID,p_m_fnd_fw_update_env->Firmware_ID,FW_UPDATE_FW_ID_LEN);

        memcpy(status->Object_ID,p_m_fnd_fw_update_env->Object_ID,8);

        MESH_MODEL_PRINT_DEBUG("Status = 0x%x\r\n",status->Status);
        MESH_MODEL_PRINT_DEBUG("Phase_Add_Info = 0x%x\r\n",status->Phase);
        MESH_MODEL_PRINT_DEBUG("Company_ID = 0x%x\r\n",status->Company_ID);
        MESH_MODEL_PRINT_DEBUG("Firmware_ID = 0x%x%x%x%x\r\n",status->Firmware_ID[0],status->Firmware_ID[1],status->Firmware_ID[2],status->Firmware_ID[3]);

        MESH_MODEL_PRINT_DEBUG("Object_ID = 0x%x,%x,%x,%x,",status->Object_ID[0],status->Object_ID[1],status->Object_ID[2],status->Object_ID[3]);
        MESH_MODEL_PRINT_DEBUG("%x,%x,%x,%x \r\n",status->Object_ID[4],status->Object_ID[5],status->Object_ID[6],status->Object_ID[7]);
        // Send the message
        m_fnd_fw_update_send(p_buf_status, M_FND_FW_UPDATE_2B_OPCODE(M_FND_FW_UPDATE_OPCODE_UPDATE_STATUS));
    }
}


__STATIC void m_fnd_fw_update_send_model_update_start_status( m_fnd_fw_update_start_t *param)
{
    MESH_MODEL_PRINT_DEBUG("%s\r\n",__func__);
    // Pointer to the buffer that will contain the message
    mesh_tb_buf_t *p_buf_status;
    // Data length
    uint16_t data_length = M_FND_FW_UPDATE_MODEL_UPDATE_STATUS_LEN;


    if (m_fnd_fw_update_buf_alloc(&p_buf_status, data_length) == MESH_ERR_NO_ERROR)
    {
        // Get pointer to data
        m_fnd_fw_update_status_t *status = (m_fnd_fw_update_status_t *)MESH_TB_BUF_DATA(p_buf_status);

        status->Status = FW_UPDATE_STAT_SUCCESS;
        // Write status
        if(param->Company_ID != p_m_fnd_fw_update_env->Company_ID)
        {
            //status->Status = FW_UPDATE_STAT_WRONG_COMPANY_FIRMWARE_COMBINATION; 0x01//
        }

        status->Phase = FW_UPDATE_PHASE_IN_PROGRESS;//0x02;
        status->Additional_Information = 0;
        status->Company_ID = p_m_fnd_fw_update_env->Company_ID;
        memcpy(status->Firmware_ID,p_m_fnd_fw_update_env->Firmware_ID,FW_UPDATE_FW_ID_LEN);

        memcpy(status->Object_ID,p_m_fnd_fw_update_env->Object_ID,8);

        MESH_MODEL_PRINT_DEBUG("Status = 0x%x\r\n",status->Status);
        MESH_MODEL_PRINT_DEBUG("Phase_Add_Info = 0x%x\r\n",status->Phase);
        MESH_MODEL_PRINT_DEBUG("Company_ID = 0x%x\r\n",status->Company_ID);
        MESH_MODEL_PRINT_DEBUG("Firmware_ID = 0x%02x,%02x,%02x,%02x\r\n",status->Firmware_ID[0],status->Firmware_ID[1],status->Firmware_ID[2],status->Firmware_ID[3]);

        MESH_MODEL_PRINT_DEBUG("Object_ID = 0x%x,%x,%x,%x,",status->Object_ID[0],status->Object_ID[1],status->Object_ID[2],status->Object_ID[3]);
        MESH_MODEL_PRINT_DEBUG("%x,%x,%x,%x \r\n",status->Object_ID[4],status->Object_ID[5],status->Object_ID[6],status->Object_ID[7]);

        // Send the message
        m_fnd_fw_update_send(p_buf_status, M_FND_FW_UPDATE_2B_OPCODE(M_FND_FW_UPDATE_OPCODE_UPDATE_STATUS));
    }
}

__STATIC void m_fnd_fw_update_send_model_update_abort_status()
{
    MESH_MODEL_PRINT_DEBUG("%s\r\n",__func__);
    // Pointer to the buffer that will contain the message
    mesh_tb_buf_t *p_buf_status;
    // Data length
    uint16_t data_length = M_FND_FW_UPDATE_MODEL_UPDATE_STATUS_LEN;


    if (m_fnd_fw_update_buf_alloc(&p_buf_status, data_length) == MESH_ERR_NO_ERROR)
    {
        // Get pointer to data
        m_fnd_fw_update_status_t *status = (m_fnd_fw_update_status_t *)MESH_TB_BUF_DATA(p_buf_status);

        status->Status = FW_UPDATE_STAT_SUCCESS;
        // Write status


        status->Phase = FW_UPDATE_PHASE_IN_PROGRESS;//0x02;
        status->Additional_Information = 0;
        status->Company_ID = p_m_fnd_fw_update_env->Company_ID;
        memcpy(status->Firmware_ID,p_m_fnd_fw_update_env->Firmware_ID,FW_UPDATE_FW_ID_LEN);

        memcpy(status->Object_ID,p_m_fnd_fw_update_env->Object_ID,8);

        MESH_MODEL_PRINT_DEBUG("Status = 0x%x\r\n",status->Status);
        MESH_MODEL_PRINT_DEBUG("Phase_Add_Info = 0x%x\r\n",status->Phase);
        MESH_MODEL_PRINT_DEBUG("Company_ID = 0x%x\r\n",status->Company_ID);
        MESH_MODEL_PRINT_DEBUG("Firmware_ID = 0x%02x,%02x,%02x,%02x\r\n",status->Firmware_ID[0],status->Firmware_ID[1],status->Firmware_ID[2],status->Firmware_ID[3]);

        MESH_MODEL_PRINT_DEBUG("Object_ID = 0x%x,%x,%x,%x,",status->Object_ID[0],status->Object_ID[1],status->Object_ID[2],status->Object_ID[3]);
        MESH_MODEL_PRINT_DEBUG("%x,%x,%x,%x \r\n",status->Object_ID[4],status->Object_ID[5],status->Object_ID[6],status->Object_ID[7]);

        // Send the message
        m_fnd_fw_update_send(p_buf_status, M_FND_FW_UPDATE_2B_OPCODE(M_FND_FW_UPDATE_OPCODE_UPDATE_STATUS));
    }
}


__STATIC void m_fnd_fw_update_send_model_update_apply_status()
{
    MESH_MODEL_PRINT_DEBUG("%s\r\n",__func__);
    // Pointer to the buffer that will contain the message
    mesh_tb_buf_t *p_buf_status;
    // Data length
    uint16_t data_length = M_FND_FW_UPDATE_MODEL_UPDATE_STATUS_LEN;


    if (m_fnd_fw_update_buf_alloc(&p_buf_status, data_length) == MESH_ERR_NO_ERROR)
    {
        // Get pointer to data
        m_fnd_fw_update_status_t *status = (m_fnd_fw_update_status_t *)MESH_TB_BUF_DATA(p_buf_status);
        // Write status

        if(hdr_back.crc == oad_calcuCrc)
        {
            status->Status = FW_UPDATE_STAT_SUCCESS;//0x00;
        }else
        {
            status->Status  = FW_UPDATE_STAT_NEWER_FW_VERSION_PRESENT; // 0x04
        }

        status->Phase = FW_UPDATE_PHASE_IDLE;//0x00;
        status->Additional_Information = 0;
        status->Company_ID = p_m_fnd_fw_update_env->Company_ID;
        memcpy(status->Firmware_ID,p_m_fnd_fw_update_env->Firmware_ID,FW_UPDATE_FW_ID_LEN);

        memcpy(status->Object_ID,p_m_fnd_fw_update_env->Object_ID,8);


        MESH_MODEL_PRINT_DEBUG("Status = 0x%x\r\n",status->Status);
        MESH_MODEL_PRINT_DEBUG("Phase_Add_Info = 0x%x\r\n",status->Phase);
        MESH_MODEL_PRINT_DEBUG("Company_ID = 0x%x\r\n",status->Company_ID);
        MESH_MODEL_PRINT_DEBUG("Firmware_ID = 0x%02x,%02x,%02x,%02x\r\n",status->Firmware_ID[0],status->Firmware_ID[1],status->Firmware_ID[2],status->Firmware_ID[3]);

        MESH_MODEL_PRINT_DEBUG("Object_ID = 0x%x,%x,%x,%x,",status->Object_ID[0],status->Object_ID[1],status->Object_ID[2],status->Object_ID[3]);
        MESH_MODEL_PRINT_DEBUG("%x,%x,%x,%x \r\n",status->Object_ID[4],status->Object_ID[5],status->Object_ID[6],status->Object_ID[7]);

        // Send the message
        m_fnd_fw_update_send(p_buf_status, M_FND_FW_UPDATE_2B_OPCODE(M_FND_FW_UPDATE_OPCODE_UPDATE_STATUS));
    }

}
*/
/**
 ****************************************************************************************
 * @brief Handle Firmware Update Information Get
 * Unacknowledged message received from provisioner.
 *
 * @param[in] p_buf     Pointer to the buffer containing the message.
 * @param[in] opcode    Operation code
 ****************************************************************************************
 */


__STATIC void m_fnd_fw_update_handler_information(mesh_tb_buf_t *p_buf, uint8_t opcode)
{
    MESH_MODEL_PRINT_DEBUG("%s,opcode = %x\r\n", __func__, opcode);
    if (opcode == M_FND_FW_UPDATE_OPCODE_INFO_GET)
    {
        MESH_MODEL_PRINT_DEBUG("opcode:M_FND_FW_UPDATE_OPCODE_INFO_GET step: 1\r\n");

        m_fnd_fw_update_send_model_info_status(p_m_fnd_fw_update_env->Company_ID, p_m_fnd_fw_update_env->Firmware_ID, p_m_fnd_fw_update_env->update_url, p_m_fnd_fw_update_env->url_len);

    }
    // Process next received message
    m_fnd_fw_update_process_next();
}



/**
 ****************************************************************************************
 * @brief Handle Firmware Update Get / Prepare / Start / Abort / Apply
 * Unacknowledged message received from provisioner.
 *
 * @param[in] p_buf     Pointer to the buffer containing the message.
 * @param[in] opcode    Operation code
 ****************************************************************************************
 */
/*
// Firmware Update Prepare message structure
typedef struct m_fnd_fw_update_prepare
{
    // Company identifier
    uint16_t Company_ID;
    // Unique firmware identifier
    uint8_t Firmware_ID[1 + M_FND_FW_UPDATE_MODEL_N];
    // Unique object identifier
    uint8_t Object_ID[8];
    // Vendor specific validation data for update (optional)
    uint8_t Vendor_validation_data[256];

} m_fnd_fw_update_prepare_t;
*/

/*
// Firmware Update Start message structure
typedef struct m_fnd_fw_update_start
{
    // Firmware update policy
    uint8_t Update_Policy;
    // Company identifier
    uint16_t Company_ID;
    // Unique firmware identifier
    uint8_t Firmware_ID[1 + M_FND_FW_UPDATE_MODEL_N];

} m_fnd_fw_update_start_t;
*/

/*
typedef struct m_fnd_fw_update_get
{
    // Company identifier
    uint16_t Company_ID;
    // Unique firmware identifier
    uint8_t Firmware_ID[1 + M_FND_FW_UPDATE_MODEL_N];

} m_fnd_fw_update_get_t;
*/

extern uint32_t calc_backup_sec_crc(void);
__STATIC void m_fnd_fw_update_handler_update(mesh_tb_buf_t *p_buf, uint8_t opcode)
{
    MESH_MODEL_PRINT_DEBUG("%s,opcode = %x\r\n", __func__, opcode);

    switch (opcode)
    {
        case M_FND_FW_UPDATE_OPCODE_UPDATE_GET:
        {
            uint8_t stat = FW_UPDATE_STAT_SUCCESS;
            uint8_t phase = FW_UPDATE_PHASE_WAITING_APPLY;
            uint8_t addi_info = 0;
            MESH_MODEL_PRINT_DEBUG("opcode:M_FND_FW_UPDATE_OPCODE_UPDATE_GET\r\n");
            m_fnd_fw_update_get_t *param = (m_fnd_fw_update_get_t *) MESH_TB_BUF_DATA(p_buf);
            MESH_MODEL_PRINT_DEBUG("param->Company_ID = 0x%x\r\n", param->Company_ID);
            MESH_MODEL_PRINT_DEBUG("param->Firmware_ID = 0x%02x,%02x,%02x,%02x\r\n", param->Firmware_ID[0], param->Firmware_ID[1], param->Firmware_ID[2], param->Firmware_ID[3]);
            m_fnd_fw_update_send_model_update_status( stat, phase, addi_info, p_m_fnd_fw_update_env->Company_ID, p_m_fnd_fw_update_env->Firmware_ID, p_m_fnd_fw_update_env->Object_ID);
        } break;

        case M_FND_FW_UPDATE_OPCODE_UPDATE_PREPARE:
        {

            uint8_t stat = FW_UPDATE_STAT_SUCCESS;
            uint8_t phase = FW_UPDATE_PHASE_PREPARED;
            uint8_t addi_info = 0;

            MESH_MODEL_PRINT_DEBUG("opcode:M_FND_FW_UPDATE_OPCODE_UPDATE_PREPARE step: 3\r\n");
            m_fnd_fw_update_prepare_t *param = (m_fnd_fw_update_prepare_t  *) MESH_TB_BUF_DATA(p_buf);
            MESH_MODEL_PRINT_DEBUG("param->Company_ID = 0x%x\r\n", param->Company_ID);
            MESH_MODEL_PRINT_DEBUG("param->Firmware_ID = 0x%02x,%02x,%02x,%02x\r\n", param->Firmware_ID[0], param->Firmware_ID[1], param->Firmware_ID[2], param->Firmware_ID[3]);
            for (int i = 0; i < 8; i++)
            {
                MESH_MODEL_PRINT_DEBUG("param->Object_ID = 0x%x\r\n", param->Object_ID[i]);
                p_m_fnd_fw_update_env->Object_ID[i] = param->Object_ID[i];
            }
            //  memcpy(p_m_fnd_fw_update_env->Firmware_ID_new, param->Firmware_ID,FW_UPDATE_FW_ID_LEN);

            // m_fnd_fw_update_send_model_update_prepare_status();
            m_fnd_fw_update_send_model_update_status( stat, phase, addi_info, p_m_fnd_fw_update_env->Company_ID, p_m_fnd_fw_update_env->Firmware_ID, p_m_fnd_fw_update_env->Object_ID);



        } break ;

        case M_FND_FW_UPDATE_OPCODE_UPDATE_START:
        {

            uint8_t stat = FW_UPDATE_STAT_SUCCESS;
            uint8_t phase = FW_UPDATE_PHASE_IN_PROGRESS;
            uint8_t addi_info = 0;
            MESH_MODEL_PRINT_DEBUG("opcode:M_FND_FW_UPDATE_OPCODE_UPDATE_START step: 4\r\n");
            m_fnd_fw_update_start_t *param = (m_fnd_fw_update_start_t *) MESH_TB_BUF_DATA(p_buf);
            MESH_MODEL_PRINT_DEBUG("param->Update_Policy = 0x%x\r\n", param->Update_Policy);
            MESH_MODEL_PRINT_DEBUG("param->Company_ID = 0x%x\r\n", param->Company_ID);
            MESH_MODEL_PRINT_DEBUG("param->Firmware_ID = 0x%02x,%02x,%02x,%02x\r\n", param->Firmware_ID[0], param->Firmware_ID[1], param->Firmware_ID[2], param->Firmware_ID[3]);

            //  m_fnd_fw_update_send_model_update_start_status(param);

            m_fnd_fw_update_send_model_update_status( stat, phase, addi_info, p_m_fnd_fw_update_env->Company_ID, p_m_fnd_fw_update_env->Firmware_ID, p_m_fnd_fw_update_env->Object_ID);


        } break;

        case M_FND_FW_UPDATE_OPCODE_UPDATE_ABORT:
        {
            uint8_t stat = FW_UPDATE_STAT_SUCCESS;
            uint8_t phase = FW_UPDATE_PHASE_IDLE;
            uint8_t addi_info = 0;

            MESH_MODEL_PRINT_DEBUG("opcode:M_FND_FW_UPDATE_OPCODE_UPDATE_ABORT\r\n");
            // m_fnd_fw_update_send_model_update_abort_status();

            m_fnd_fw_update_send_model_update_status( stat, phase, addi_info, p_m_fnd_fw_update_env->Company_ID, p_m_fnd_fw_update_env->Firmware_ID, p_m_fnd_fw_update_env->Object_ID);

        } break;


        case M_FND_FW_UPDATE_OPCODE_UPDATE_APPLY:
        {
            uint8_t stat = FW_UPDATE_STAT_SUCCESS;
            uint8_t phase = FW_UPDATE_PHASE_IDLE;
            uint8_t addi_info = 0;

            oad_calcuCrc = calc_backup_sec_crc();
            if (hdr_back.crc == oad_calcuCrc)
            {
                stat = FW_UPDATE_STAT_SUCCESS;//0x00;
            }
            else
            {
                stat  = FW_UPDATE_STAT_COMPANY_FIRMWARE_APPLY_FAILED; // 0x03
            }


            MESH_MODEL_PRINT_DEBUG("opcode:M_FND_FW_UPDATE_OPCODE_UPDATE_APPLY\r\n");
            m_fnd_fw_update_apply_t *param = (m_fnd_fw_update_apply_t *) MESH_TB_BUF_DATA(p_buf);
            MESH_MODEL_PRINT_DEBUG("param->Company_ID = 0x%x\r\n", param->Company_ID);
            MESH_MODEL_PRINT_DEBUG("param->Firmware_ID = 0x%02x,%02x,%02x,%02x\r\n", param->Firmware_ID[0], param->Firmware_ID[1], param->Firmware_ID[2], param->Firmware_ID[3]);


            //     m_fnd_fw_update_send_model_update_apply_status();
            m_fnd_fw_update_send_model_update_status( stat, phase, addi_info, p_m_fnd_fw_update_env->Company_ID, p_m_fnd_fw_update_env->Firmware_ID, p_m_fnd_fw_update_env->Object_ID);

            nvds_del(NVDS_TAG_MESH_OTA_INFO);
            wdt_reset(0xa000);
        } break;
        default:break;
    }

    // Process next received message
    m_fnd_fw_update_process_next();

}

/**
 ****************************************************************************************
 * @brief Call the appropriate handler for a given buffer.
 *
 * @param[in] p_buf     Buffer to handle.
 ****************************************************************************************
 */
__STATIC void m_fnd_fw_update_process(m_api_buf_t *p_buf)
{

    MESH_MODEL_PRINT_DEBUG("%s\r\n", __func__);
    // Get buffer environment
    m_lay_buf_env_t *p_env = (m_lay_buf_env_t *)(&((mesh_tb_buf_t *)p_buf)->env[0]);

    // Check that opcode is a 2-byte opcode
    if (MESH_IS_2_OCT_OPCODE(p_env->u5.opcode))
    {
        // Get only second byte of the opcode
        uint8_t opcode_2b = (uint8_t)(p_env->u5.opcode >> 8);

        switch (opcode_2b)
        {
            case (M_FND_FW_UPDATE_OPCODE_INFO_GET)          : m_fnd_fw_update_handler_information(p_buf, opcode_2b); break;

            case (M_FND_FW_UPDATE_OPCODE_UPDATE_GET)        :
            case (M_FND_FW_UPDATE_OPCODE_UPDATE_PREPARE)    :
            case (M_FND_FW_UPDATE_OPCODE_UPDATE_START)      :
            case (M_FND_FW_UPDATE_OPCODE_UPDATE_ABORT)      :
            case (M_FND_FW_UPDATE_OPCODE_UPDATE_APPLY)      : m_fnd_fw_update_handler_update(p_buf, opcode_2b); break;

            default                                         : m_fnd_fw_update_process_next(); break;
        }
    }
}




/**
 ****************************************************************************************
 * @brief Pop first buffer from the processing queue, release it and start processing of
 * next one in the processing queue.
 ****************************************************************************************
 */
__STATIC void m_fnd_fw_update_process_next(void)
{

    MESH_MODEL_PRINT_DEBUG("%s\r\n", __func__);
    // Remove first buffer in the process queue
    mesh_tb_buf_t *p_buf = (mesh_tb_buf_t *)co_list_pop_front(&p_m_fnd_fw_update_env->process_queue);

    // Release the buffer
    mesh_tb_buf_release(p_buf);

    // Process next buffer
    p_buf = (mesh_tb_buf_t *)co_list_pick(&p_m_fnd_fw_update_env->process_queue);

    if (p_buf != NULL)
    {
        m_fnd_fw_update_process(p_buf);
    }
}

/**
 ****************************************************************************************
 * @brief Callback function called upon reception of a message.
 *
 * @param[in] model_lid       Model local index.
 * @param[in] opcode          Operation code.
 * @param[in] p_api_buf       Pointer to the buffer containing the message PDU.
 * @param[in] app_key_lid     Application key local index (Required for a response).
 * @param[in] src             Source address of the message (Required for a response).
 * @param[in] rssi            Measured RSSI level for the received PDU.
 * @param[in] not_relayed     True if message have been received by an immediate peer;
 *                            False, it can have been relayed.
 ****************************************************************************************
 */
__STATIC void m_fnd_fw_update_cb_rx(m_lid_t model_lid, uint32_t opcode, m_api_buf_t *p_api_buf, m_lid_t app_key_lid,
                                    uint16_t src, int8_t rssi, bool not_relayed)
{

    MESH_MODEL_PRINT_DEBUG("%s\r\n", __func__);
    // Data length
    uint16_t data_len = ((mesh_tb_buf_t *)p_api_buf)->data_len;
    // Allocate a buffer
    mesh_tb_buf_t *p_buf;

    mesh_tb_buf_alloc(&p_buf, 0, data_len, 0);

    // Copy content of the received buffer in the allocated one
    mesh_tb_buf_copy((mesh_tb_buf_t *)p_api_buf, p_buf, data_len, true);

    // Insert the buffer in the process queue
    co_list_push_back(&p_m_fnd_fw_update_env->process_queue, &p_buf->hdr);

    // Check if buffer can be processed now or if another buffer is currently processed
    if ((mesh_tb_buf_t *)co_list_pick(&p_m_fnd_fw_update_env->process_queue) == p_buf)
    {
        m_fnd_fw_update_process(p_buf);
    }
}
/**
 ****************************************************************************************
 * @brief Callback function called upon reception of a message in order to know if the model
 * supports the received operation code.
 *
 * @param[in] model_lid     Model local index
 * @param[in] opcode        Operation code to check
 ****************************************************************************************
 */
__STATIC void m_fnd_fw_update_cb_opcode_check(m_lid_t model_lid, uint32_t opcode)
{

    // Status
    uint16_t status = MESH_ERR_NOT_SUPPORTED;

    // Check that opcode is a 2-byte opcode
    if (MESH_IS_2_OCT_OPCODE(opcode) && M_IS_UPDATE_OPCODE(opcode))
    {
        // Get only second byte of the opcode
        uint8_t opcode_2b = (uint8_t)(opcode >> 8);

        switch (opcode_2b)
        {
            case M_FND_FW_UPDATE_OPCODE_INFO_GET:
            case M_FND_FW_UPDATE_OPCODE_INFO_STATUS:
            case M_FND_FW_UPDATE_OPCODE_UPDATE_GET:
            case M_FND_FW_UPDATE_OPCODE_UPDATE_PREPARE:
            case M_FND_FW_UPDATE_OPCODE_UPDATE_START:
            case M_FND_FW_UPDATE_OPCODE_UPDATE_ABORT:
            case M_FND_FW_UPDATE_OPCODE_UPDATE_APPLY:
            case M_FND_FW_UPDATE_OPCODE_UPDATE_STATUS:
            {
                status = MESH_ERR_NO_ERROR;
            } break;

            default:break;

        }
    }
    MESH_MODEL_PRINT_DEBUG("%s,opcode = 0x%x,status = %x\r\n", __func__, opcode, status);
    // Indicate if provided operation code is supported or not
    m_api_model_opcode_status(model_lid, opcode, status);
}


/**
 ****************************************************************************************
 * @brief Callback function called once buffer pushed by a model has been transmitted or if
 * an issue has been raised during processing.
 *
 * @param[in] model_lid     Model local index.
 * @param[in] tx_hdl        Handle value configured by model when message has been pushed for transmission.
 * @param[in] p_api_buf     Pointer to the buffer containing the transmitted PDU.
 * @param[in] status        Transmission status.
 ****************************************************************************************
 */
__STATIC void m_fnd_fw_update_cb_sent(m_lid_t model_lid, uint8_t tx_hdl, m_api_buf_t *p_api_buf, uint16_t status)
{
    MESH_MODEL_PRINT_DEBUG("%s\r\n", __func__);

    // Release the buffer
    mesh_tb_buf_release((mesh_tb_buf_t *)p_api_buf);
}



/**
 ****************************************************************************************
 * @brief Callback function called upon reception of new publication parameters in order to inform
 * the model about a new publish period.
 *
 * @param[in] model_lid     Model local index
 * @param[in] period_ms     Publish period in milliseconds
 ****************************************************************************************
 */
__STATIC void m_fnd_fw_update_cb_publish_period(m_lid_t model_lid, uint16_t addr, uint32_t period_ms)
{
    // Inform the application about the new publication period
    // m_api_send_fault_period_ind(period_ms, period_ms >> p_m_fnd_hlths_env->fast_period_divisor);
}


/*
 * CALLBACK FUNCTIONS
 ****************************************************************************************
 */

/// Health Client Model callback functions
const m_api_model_cb_t m_fnd_fw_update_cb =
{
    .cb_rx             = m_fnd_fw_update_cb_rx,
    .cb_sent           = m_fnd_fw_update_cb_sent,
    .cb_opcode_check   = m_fnd_fw_update_cb_opcode_check,
    .cb_publish_param = m_fnd_fw_update_cb_publish_period,
};



/*
 * GLOBAL FUNCTIONS
 ****************************************************************************************
 */
__STATIC nvds_mesh_ota_tag_t fw_ota_info;
//uint16_t m_fnd_fw_update_init(bool reset, void *p_env, const m_cfg_t* p_cfg)
uint16_t m_fnd_fw_update_init(bool reset)
{
    MESH_MODEL_PRINT_DEBUG("%s\r\n", __func__);
    if (!reset)
    {
        uint8_t oad_header[16];
        uint8_t length = sizeof(nvds_mesh_ota_tag_t);
        // Get environment for Health Client model
        //p_m_fnd_fw_update_env = (m_fnd_fw_update_env_t *)p_env;
        p_m_fnd_fw_update_env = mal_malloc(sizeof(m_fnd_fw_update_env_t));

        // Initialize environment
        memset(p_m_fnd_fw_update_env, 0, sizeof(m_fnd_fw_update_env_t));

        flash_read(0, SEC_IMAGE_APP_OAD_HEADER_FADDR, 16, oad_header, NULL);

        p_m_fnd_fw_update_env->Firmware_ID[0] = oad_header[4];
        p_m_fnd_fw_update_env->Firmware_ID[1] = oad_header[5];
        p_m_fnd_fw_update_env->Firmware_ID[2] = oad_header[14];
        p_m_fnd_fw_update_env->Firmware_ID[3] = oad_header[15];

        memset(&fw_ota_info, 0, sizeof(nvds_mesh_ota_tag_t));
        if (NVDS_OK == nvds_get(NVDS_TAG_MESH_OTA_INFO, &length, (uint8_t *)&fw_ota_info))
        {
            memcpy(p_m_fnd_fw_update_env->Object_ID, fw_ota_info.Object_ID, 8);
        }

        MESH_MODEL_PRINT_DEBUG("%s, Object_ID: %s\n",
                               __func__,
                               mesh_buffer_to_hex(p_m_fnd_fw_update_env->Object_ID, 8));

        p_m_fnd_fw_update_env->Company_ID = 0x01A8;

        p_m_fnd_fw_update_env->url_len = 0;

        MESH_MODEL_PRINT_DEBUG("oad_header = %s\n", mesh_buffer_to_hex(oad_header, 16));

        // Register the model
        m_api_register_model(M_FND_FW_MODEL_UPDATE_ID, 0, false, &m_fnd_fw_update_cb,
                             &p_m_fnd_fw_update_env->model_lid);
    }
    else
    {
        p_m_fnd_fw_update_env = NULL;
    }

    // Return environment size
    return (sizeof(m_fnd_fw_update_env_t));
}


uint16_t m_fnd_fw_update_get_env_size(const m_cfg_t* p_cfg)
{
    // Return environment size
    return (sizeof(m_fnd_fw_update_env_t));
}




/// @} end of group
