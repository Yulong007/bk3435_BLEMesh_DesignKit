

/**
 ****************************************************************************************
 * @file m_fnd_BLOB_Transfer.c
 *
 * @brief Mesh BLOB Transfer Server Model
 *
 * Copyright (C) Beken 2018-2019
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup M_FND_BLOB_TRANSFER
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
#include "m_fnd_BLOB_Transfer.h"
#include "flash.h"         // flash definitio
#include "pwm.h"
#include "nvds.h"
#include "app.h"
#include "ll.h"
#include "oad_common.h"
#include "mesh_log.h"

/*
 * DEFINES
 ****************************************************************************************
 */
/// BLOB Transfer Server SIG Model ID
#define M_FND_BLOB_TRANSFER_MODEL_ID                        (0xFF00)

/// Length of  BLOB Transfer Model Object Trans Phase Status message
#define M_FND_BLOB_MODEL_OBJ_TRANS_PHASE_STATUS_LEN                     (9)

/// Length of  BLOB Transfer Model Object Trans Get message
#define M_FND_BLOB_MODEL_OBJ_TRANS_GET_LEN                      (8)

/// Length of  BLOB Transfer Model Object Trans Start message
#define M_FND_BLOB_MODEL_OBJ_TRANS_START_LEN                        (13)

/// Length of BLOB Transfer Model Object Trans Abort message
#define M_FND_BLOB_MODEL_OBJ_TRANS_ABORT_LEN           (8)

/// Length of BLOB Transfer Model Object  Trans Status message
#define M_FND_BLOB_MODEL_OBJ_TRANS_STATUS_LEN              (14)

/// Length of  BLOB Transfer Model Object Block Trans Start message
#define M_FND_BLOB_MODEL_OBJ_BLK_TRANS_START_MIN_LEN                        (15)//Min length


/// Length of BLOB Transfer Model Object Block Trans Status message
#define M_FND_BLOB_MODEL_OBJ_BLK_TRANS_STATUS_LEN              (1)

/// Length of BLOB Transfer Model Object Chunk Trans message
#define M_FND_BLOB_MODEL_OBJ_CHUNK_TRANS_LEN           (2)//Min lenght

/// Length of BLOB Transfer Model Object Block Get message
#define M_FND_BLOB_MODEL_OBJ_BLK_GET_LEN           (10)

/// Length of BLOB Transfer Model Object Block Status message
#define M_FND_BLOB_MODEL_OBJ_BLK_STATUS_MIN_LEN           (1)//Min lenght

/// Length of BLOB Transfer Model Object Info Status message
#define M_FND_BLOB_MODEL_OBJ_INFO_STATUS_LEN           (4)

/*
 * MACROS
 ****************************************************************************************
 */

//Minimum block size:  2 ^ Max Block Size Log
#define MIN_BLOCK_SIZE_LOG  12
//Maximum block size:  2 ^ Max Block Size Log
#define MAX_BLOCK_SIZE_LOG  12
//Supported maximum number of chunks in block
#define MAX_CHUNKS_NUMBER   16


/*
 * MESSAGE STRUCTURES
 ****************************************************************************************
 */







/*
 * STRUCTURES
 ****************************************************************************************
 */
/// Foundation model layer environment structure
typedef struct m_fnd_blob_env
{
    /// List of buffers containing messages to process
    co_list_t   process_queue;
    /// Delayed job structure
    mal_djob_t djob;
    /// Opcode actually handled
    uint8_t     opcode;
    /// Model local index
    m_lid_t     model_lid;

    uint16_t Block_Number;
    uint16_t Current_Block_Size;
    uint16_t Receive_Chunk_Mask;
    uint16_t Current_Chunk_Mask;

    uint32_t Block_Checksum_Value;
    uint32_t Object_size;
    uint8_t block_size_log;

    uint8_t Object_ID[8];
    //  m_fnd_blob_obj_trans_status_t  trans_status;
    m_fnd_blob_obj_blk_trans_status_t blk_trans_status;
    m_fnd_blob_obj_blk_status_t blk_status;
    m_fnd_blob_obj_info_status_t info_status;

} m_fnd_blob_env_t;



/*
 * PRIVATE VARIABLES
 ****************************************************************************************
 */

/// Foundation model layer environment
__STATIC m_fnd_blob_env_t *p_m_fnd_blob_env;

__STATIC nvds_mesh_ota_tag_t ota_info;


uint8_t  crc_buf[4096];


/*
 * STATIC FUNCTION DEFINITIONS
 ****************************************************************************************
 */

__STATIC void m_fnd_blob_process_next(void);



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
__STATIC uint16_t m_fnd_blob_buf_alloc(mesh_tb_buf_t **pp_buf, uint16_t data_len)
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
__STATIC void m_fnd_blob_send(mesh_tb_buf_t *p_buf, uint16_t opcode)
{
    // Retrieve buffer containing the message for which a response is sent
    mesh_tb_buf_t *p_buf_req = (mesh_tb_buf_t *)co_list_pick(&p_m_fnd_blob_env->process_queue);
    // Get buffer environment
    m_lay_buf_env_t *p_env = (m_lay_buf_env_t *)&p_buf_req->env[0];
    // Use request source address as destination address
    uint16_t dst = p_env->src;

    MESH_MODEL_PRINT_DEBUG("%s\r\n", __func__);
    // Send the provided message
    m_api_model_rsp_send(p_m_fnd_blob_env->model_lid, (uint32_t)opcode, 0,
                         p_buf, p_env->app_lid, dst, false, false);
}


/**
 ****************************************************************************************
 * @brief Push a buffer for transmission (publication).
 *
 * @param[in] p_buf     Pointer to the buffer containing the message to send.
 * @param[in] opcode    Operation code
 ****************************************************************************************
 */
__STATIC void m_fnd_blob_publish(mesh_tb_buf_t *p_buf, uint16_t opcode)
{
    uint16_t status;
    // Send the provided message
    status = m_api_model_publish(p_m_fnd_blob_env->model_lid, (uint32_t)opcode, 0, p_buf, false);
    if (status != MESH_ERR_NO_ERROR)
    {
        m_api_buf_release(p_buf);
    }
}


/**
 ****************************************************************************************
 * @brief
 *
 * @param[in]
 * @param[in]
 ****************************************************************************************
 */


/**
 ****************************************************************************************
 * @brief
 *
 * @param[in]
 * @param[in]
 ****************************************************************************************
 */


/*
__STATIC m_fnd_blob_obj_trans_status_t m_fnd_blob_get_obj_trans_status()
{
    MESH_MODEL_PRINT_DEBUG("%s\r\n",__func__);
    m_fnd_blob_obj_trans_status_t status;
    status.Status = BLOB_TRANS_STATUS_BUSY;//;//BLOB_TRANS_STATUS_READY;

    memcpy(status.Object_ID, p_m_fnd_blob_env->trans_status.Object_ID,8);
    status.Object_Size = p_m_fnd_blob_env->Object_size;
    status.Block_Size_Log = 12; // 2^12 = 4096
    return status;
}
*/
/**
 ****************************************************************************************
 * @brief
 *
 * @param[in]
 * @param[in]
 ****************************************************************************************
 */

/*

/// BLOB Object Block Status message  structure
typedef struct m_fnd_blob_obj_blk_status
{
    //Status of operation
    uint8_t Status;
    //Missing chunks list
    uint8_t Missing_Chunks_List[2 * 16];

}m_fnd_blob_obj_blk_status_t;

*/

__STATIC uint8_t  m_fnd_blob_get_obj_blk_status(m_fnd_blob_obj_blk_status_t *status)
{
    MESH_MODEL_PRINT_DEBUG("%s\r\n", __func__);
    uint8_t len = 0;
    uint8_t miss_chunk = 0;
    status->Status = BLOB_BLK_STATUS_ALL_CHUNK_RECEVICED;
    MESH_MODEL_PRINT_DEBUG("p_m_fnd_blob_env->Blk_Chunk_Mask = 0x%04x\r\n", p_m_fnd_blob_env->Receive_Chunk_Mask);
    MESH_MODEL_PRINT_DEBUG("p_m_fnd_blob_env->Current_Chunk_Mask = 0x%04x\r\n", p_m_fnd_blob_env->Current_Chunk_Mask);
    for (int i = 0 ; i < 16; i++)
    {
        if (((p_m_fnd_blob_env->Receive_Chunk_Mask >> i) & 0x01) != 1 &&(((p_m_fnd_blob_env->Current_Chunk_Mask >> i) & 0x01) == 1 ))
        {
            MESH_MODEL_PRINT_DEBUG("chunk_mask >> %d !=  1\r\n", i);
            status->Missing_Chunks_List[miss_chunk++] = i;
            status->Status = BLOB_BLK_STATUS_N0T_ALL_CHUNK_RECEVICED;
            len+=2;
        }
        else
        {
            MESH_MODEL_PRINT_DEBUG("chunk_mask >> %d == 1\r\n", i);
        }
    }

    return len;
}

/**
 ****************************************************************************************
 * @brief
 *
 * @param[in]
 * @param[in]
 ****************************************************************************************
 */

/*
__STATIC m_fnd_blob_obj_info_status_t m_fnd_blob_get_obj_info_status()
{
    MESH_MODEL_PRINT_DEBUG("%s\r\n",__func__);
    m_fnd_blob_obj_info_status_t status;

    status.Min_Block_Size_Log = 12; // 2^12 = 4096
    status.Max_Block_Size_Log = 12; // 2^12 = 4096
    status.Max_Chunks_Number = 16;
    return status;
}
*/
/**
 ****************************************************************************************
 * @brief Prepare and send a Blob Transfer Model Object Transfer Status message.
 *
 * @param[in] status        Handling status for the command that triggered sending of this message.
 * @param[in] p_msg         Pointer to the genonoffs_onoff_status message.
 * @param[in] vendor        True if model identifier is a vendor model identifier.
 ****************************************************************************************
 */

/*

/// BLOB Transfer phase status message  structure
typedef struct m_fnd_blob_obj_trans_status
{
    //Status of operation
    uint8_t Status;
    //Unique object identifier
    uint8_t Object_ID[8];
    //Object size in bytes
    uint32_t Object_Size;
    //Size of the block during current transfer
    uint8_t Block_Size_Log;

}m_fnd_blob_obj_trans_status_t;
*/

__STATIC void m_fnd_blob_send_model_obj_trans_status(uint8_t stat, uint8_t *object_id, uint32_t object_size, uint8_t block_size_log)
{
    MESH_MODEL_PRINT_DEBUG("%s\r\n", __func__);
    // Pointer to the buffer that will contain the message
    mesh_tb_buf_t *p_buf_status;
    // Data length
    uint16_t data_length = M_FND_BLOB_MODEL_OBJ_TRANS_STATUS_LEN;//14

    m_fnd_blob_obj_trans_status_t status;

    if (m_fnd_blob_buf_alloc(&p_buf_status, data_length) == MESH_ERR_NO_ERROR)
    {
        // Get pointer to data
        uint8_t *p_data = MESH_TB_BUF_DATA(p_buf_status);
        // Write status
        status.Status  = stat ; //BLOB_TRANS_STATUS_BUSY;
        memcpy(status.Object_ID, object_id, 8);
        status.Object_Size = object_size;
        status.Block_Size_Log = block_size_log; // 2^12 = 4096

        MESH_MODEL_PRINT_DEBUG("status = 0x%x\r\n", status.Status);
        MESH_MODEL_PRINT_DEBUG("Object_Size = 0x%x\r\n", status.Object_Size);
        MESH_MODEL_PRINT_DEBUG("Object_ID = 0x%x,%x,%x,%x,", status.Object_ID[0], status.Object_ID[1], status.Object_ID[2], status.Object_ID[3]);
        MESH_MODEL_PRINT_DEBUG("%x,%x,%x,%x \r\n", status.Object_ID[4], status.Object_ID[5], status.Object_ID[6], status.Object_ID[7]);

        memcpy(p_data, (uint8_t *)&status, data_length);

        for (int i = 0; i <sizeof(m_fnd_blob_obj_trans_status_t); i++ ) // len 14
        {
            MESH_MODEL_PRINT_DEBUG("data[%d]= 0x%02x\r\n", i, p_data[i]);
        } MESH_MODEL_PRINT_DEBUG("\r\n");

        // Send the message
        m_fnd_blob_send(p_buf_status, M_FND_BLOB_TRANSFER_2B_OPCODE(M_FND_BLOB_OPCODE_OBJ_TRANS_STATUS));
    }

}

/**
 ****************************************************************************************
 * @brief Prepare and send a Blob Transfer Model Object Transfer Status message.
 *
 * @param[in] status        Handling status for the command that triggered sending of this message.
 * @param[in] p_msg         Pointer to the genonoffs_onoff_status message.
 * @param[in] vendor        True if model identifier is a vendor model identifier.
 ****************************************************************************************
 */

__STATIC void m_fnd_blob_send_model_obj_blk_trans_status(uint8_t stat)
{
    MESH_MODEL_PRINT_DEBUG("%s\r\n", __func__);
    // Pointer to the buffer that will contain the message
    mesh_tb_buf_t *p_buf_status;
    // Data length
    uint16_t data_length = M_FND_BLOB_MODEL_OBJ_BLK_TRANS_STATUS_LEN;

    m_fnd_blob_obj_blk_trans_status_t status;

    if (m_fnd_blob_buf_alloc(&p_buf_status, data_length) == MESH_ERR_NO_ERROR)
    {
        // Get pointer to data
        uint8_t *p_data = MESH_TB_BUF_DATA(p_buf_status);
        // Write status
        status.Status = stat;//m_fnd_blob_get_obj_blk_trans_status(stat);

        MESH_MODEL_PRINT_DEBUG("status = 0x%x\r\n", status.Status);
        memcpy(p_data, (uint8_t *)&status, sizeof(m_fnd_blob_obj_blk_trans_status_t));


        // Send the message
        m_fnd_blob_send(p_buf_status, M_FND_BLOB_TRANSFER_2B_OPCODE(M_FND_BLOB_OPCODE_OBJ_BLK_TRANS_STATUS));
    }
}



/**
 ****************************************************************************************
 * @brief Prepare and send a Blob Transfer Model Object Block Status message.
 *
 * @param[in] status        Handling status for the command that triggered sending of this message.
 * @param[in] p_msg         Pointer to the genonoffs_onoff_status message.
 * @param[in] vendor        True if model identifier is a vendor model identifier.
 ****************************************************************************************
 */

__STATIC void m_fnd_blob_send_model_obj_blk_status()
{
    MESH_MODEL_PRINT_DEBUG("%s\r\n", __func__);
    // Pointer to the buffer that will contain the message
    mesh_tb_buf_t *p_buf_status;
    // Data length
    uint16_t data_length = M_FND_BLOB_MODEL_OBJ_BLK_STATUS_MIN_LEN;

    m_fnd_blob_obj_blk_status_t status;

    uint8_t len = m_fnd_blob_get_obj_blk_status(&status);


    MESH_MODEL_PRINT_DEBUG("size m_fnd_blob_obj_blk_status_t = %d\r\n", sizeof(m_fnd_blob_obj_blk_status_t));
    MESH_MODEL_PRINT_DEBUG("len = %d\r\n", len);

    data_length+= len;

    MESH_MODEL_PRINT_DEBUG("status.Status = 0x%x,data_length = %d\r\n", status.Status, data_length);
    MESH_MODEL_PRINT_DEBUG("status.Missing_Chunks_List :");
    for (int i = 0; i < (len - 1) /2 ; i++ )
        MESH_MODEL_PRINT_DEBUG("0x%x ", status.Missing_Chunks_List[i]);

    MESH_MODEL_PRINT_DEBUG("\r\n");
    if (m_fnd_blob_buf_alloc(&p_buf_status, data_length) == MESH_ERR_NO_ERROR)
    {
        // Get pointer to data
        uint8_t *p_data = MESH_TB_BUF_DATA(p_buf_status);

        // Write status
        memcpy(p_data, (uint8_t *)&status, data_length);
        MESH_MODEL_PRINT_DEBUG("p_data = 0x%x,%x,%x\r\n", p_data[0], p_data[1], p_data[2]);
        // Send the message
        m_fnd_blob_send(p_buf_status, M_FND_BLOB_TRANSFER_2B_OPCODE(M_FND_BLOB_OPCODE_OBJ_BLK_STATUS));
    }
    else
    {
        MESH_MODEL_PRINT_DEBUG("m_fnd_blob_buf_alloc fail!!!\r\n");
    }
}


/**
 ****************************************************************************************
 * @brief Prepare and send a Blob Transfer Model Object Info Status message.
 *
 * @param[in] status        Handling status for the command that triggered sending of this message.
 * @param[in] p_msg         Pointer to the genonoffs_onoff_status message.
 * @param[in] vendor        True if model identifier is a vendor model identifier.
 ****************************************************************************************
 */


__STATIC void m_fnd_blob_send_model_obj_info_status(uint8_t min_block_size_log, uint8_t max_block_size_log, uint16_t max_chunks_number)
{
    MESH_MODEL_PRINT_DEBUG("%s\r\n", __func__);
    // Pointer to the buffer that will contain the message
    mesh_tb_buf_t *p_buf_status;
    // Data length
    uint16_t data_length = M_FND_BLOB_MODEL_OBJ_INFO_STATUS_LEN;

    m_fnd_blob_obj_info_status_t status;
    //m_fnd_blob_obj_info_status_t *p_status;
    MESH_MODEL_PRINT_DEBUG("data_length = %d\r\n", data_length);
    if (m_fnd_blob_buf_alloc(&p_buf_status, data_length) == MESH_ERR_NO_ERROR)
    {
        // Get pointer to data
        uint8_t *p_data = MESH_TB_BUF_DATA(p_buf_status);
        // Write status
        status.Min_Block_Size_Log = min_block_size_log;
        status.Max_Block_Size_Log = max_block_size_log;
        status.Max_Chunks_Number = max_chunks_number;
        memcpy(p_data, (uint8_t *)&status, data_length);
        MESH_MODEL_PRINT_DEBUG("Min_Block_Size_Log = 0x%02x\r\n", status.Min_Block_Size_Log);
        MESH_MODEL_PRINT_DEBUG("Max_Block_Size_Log = 0x%02x\r\n", status.Max_Block_Size_Log);
        MESH_MODEL_PRINT_DEBUG("Max_Chunks_Number = 0x%02x\r\n", status.Max_Chunks_Number);

        // Send the message
        m_fnd_blob_send(p_buf_status, M_FND_BLOB_TRANSFER_2B_OPCODE(M_FND_BLOB_OPCODE_OBJ_INFO_STATUS));
    }
    else
    {
        MESH_MODEL_PRINT_DEBUG("m_fnd_blob_buf_alloc fail!!!\r\n");
    }
}




/**
 ****************************************************************************************
 * @brief Handle Blob Transfer Model Object Get ,Start or Abort
 * Unacknowledged message received from provisioner.
 *
 * @param[in] p_buf     Pointer to the buffer containing the message.
 * @param[in] opcode    Operation code
 ****************************************************************************************
 */

__STATIC void m_fnd_blob_handler_obj_obj(mesh_tb_buf_t *p_buf, uint8_t opcode)
{
    MESH_MODEL_PRINT_DEBUG("%s,opcode = %x\r\n", __func__, opcode);

    switch (opcode)
    {
        case M_FND_BLOB_OPCODE_OBJ_TRANS_GET:
        {
            MESH_MODEL_PRINT_DEBUG("M_FND_BLOB_OPCODE_OBJ_TRANS_GET\r\n");

            uint8_t stat = BLOB_TRANS_STATUS_BUSY;
            // Read value of the  object trans get
            m_fnd_blob_obj_trans_get_t  *get = (m_fnd_blob_obj_trans_get_t *) MESH_TB_BUF_DATA(p_buf);
            for (int i = 0; i < 8; i++)
            {
                MESH_MODEL_PRINT_DEBUG("Object ID[%d] = 0x%x\r\n", i, get->Object_ID[i]);
            }
            m_fnd_blob_send_model_obj_trans_status(stat, p_m_fnd_blob_env->Object_ID, p_m_fnd_blob_env->Object_size, p_m_fnd_blob_env->block_size_log);

            //  m_fnd_blob_send_model_obj_trans_status();
        } break;

        case M_FND_BLOB_OPCODE_OBJ_TRANS_START:
        {
            MESH_MODEL_PRINT_DEBUG("opcode:M_FND_BLOB_OPCODE_OBJ_TRANS_START step: 5\r\n");
            // Read value of the  object trans start
            m_fnd_blob_obj_trans_start_t  *start = (m_fnd_blob_obj_trans_start_t *) MESH_TB_BUF_DATA(p_buf);
            uint8_t stat = BLOB_TRANS_STATUS_BUSY;
            for (int i = 0; i < 8; i++)
            {
                MESH_MODEL_PRINT_DEBUG("Object ID[%d] = 0x%x\r\n", i, start->Object_ID[i]);
                p_m_fnd_blob_env->Object_ID[i] = start->Object_ID[i];
            }
            MESH_MODEL_PRINT_DEBUG("Object Size = 0x%x\r\n", start->Object_Size);
            p_m_fnd_blob_env->Object_size = start->Object_Size;
            p_m_fnd_blob_env->block_size_log = start->Block_Size_Log;
            MESH_MODEL_PRINT_DEBUG("Current Block Size Log = 0x%x\r\n", start->Block_Size_Log);
            // m_fnd_blob_send_model_obj_trans_status();

            m_fnd_blob_send_model_obj_trans_status(stat, p_m_fnd_blob_env->Object_ID, p_m_fnd_blob_env->Object_size, start->Block_Size_Log);
        } break;

        case M_FND_BLOB_OPCODE_OBJ_TRANS_ABORT:
        {
            MESH_MODEL_PRINT_DEBUG("M_FND_BLOB_OPCODE_OBJ_TRANS_ABORT\r\n");
            uint8_t stat = BLOB_TRANS_STATUS_BUSY;
            // Read value of the  object trans abort
            m_fnd_blob_obj_trans_abort_t  *abort = (m_fnd_blob_obj_trans_abort_t *) MESH_TB_BUF_DATA(p_buf);
            for (int i = 0; i < 8; i++)
            {
                MESH_MODEL_PRINT_DEBUG("Object ID[%d] = 0x%x\r\n", i, abort->Object_ID[i]);
            }

            m_fnd_blob_send_model_obj_trans_status(stat, p_m_fnd_blob_env->Object_ID, p_m_fnd_blob_env->Object_size, p_m_fnd_blob_env->block_size_log );
            // m_fnd_blob_send_model_obj_trans_status();
        } break;

        default:break;

    }



    // Process next received message
    m_fnd_blob_process_next();
}

/**
 ****************************************************************************************
 * @brief Handle Blob Transfer Model Block Transfer Start or Block Get
 * Unacknowledged message received from provisioner.
 *
 * @param[in] p_buf     Pointer to the buffer containing the message.
 * @param[in] opcode    Operation code
 ****************************************************************************************
 */


__STATIC void m_fnd_blob_handler_obj_blk(mesh_tb_buf_t *p_buf, uint8_t opcode)
{
    MESH_MODEL_PRINT_DEBUG("%s,opcode = %x\r\n", __func__, opcode);
    // M_FND_BLOB_OPCODE_OBJ_BLK_TRANS_START
    // M_FND_BLOB_OPCODE_OBJ_BLK_GET
    if (opcode == M_FND_BLOB_OPCODE_OBJ_BLK_TRANS_START)
    {
        MESH_MODEL_PRINT_DEBUG("opcode:M_FND_BLOB_OPCODE_OBJ_BLK_TRANS_START step: 6\r\n");

        nvds_tag_len_t length = sizeof(nvds_mesh_ota_tag_t);
        p_m_fnd_blob_env->Receive_Chunk_Mask = 0;
        p_m_fnd_blob_env->Current_Chunk_Mask = 0;
        p_m_fnd_blob_env->Current_Block_Size = 0;

        // Read value of the object blk trans start
        m_fnd_blob_obj_blk_trans_start_t  *start = (m_fnd_blob_obj_blk_trans_start_t *) MESH_TB_BUF_DATA(p_buf);
        MESH_MODEL_PRINT_DEBUG("p_buf ->head_len = 0x%x\r\n", p_buf ->head_len);
        MESH_MODEL_PRINT_DEBUG("p_buf ->tail_len = 0x%x\r\n", p_buf ->tail_len);
        MESH_MODEL_PRINT_DEBUG("p_buf ->data_len = 0x%x\r\n", p_buf ->data_len);

        if (p_buf ->data_len == sizeof(m_fnd_blob_obj_blk_trans_start_t))
        {
            p_m_fnd_blob_env->Current_Block_Size = start->Current_Block_Size;

            for (int i = 0; i < ((p_m_fnd_blob_env->Current_Block_Size - 1) / 0x100) + 1; i++)
            {
                p_m_fnd_blob_env->Current_Chunk_Mask |= (0x01 << i);
            }
        }
        else
        {
            p_m_fnd_blob_env->Current_Block_Size = 0x1000;
            p_m_fnd_blob_env->Current_Chunk_Mask = 0xFFFF;
        }
        MESH_MODEL_PRINT_DEBUG("Current_Block_Size = 0x%x\r\n", p_m_fnd_blob_env->Current_Block_Size);
        MESH_MODEL_PRINT_DEBUG("Current_chunk_Mask = 0x%x\r\n", p_m_fnd_blob_env->Current_Chunk_Mask);
        for (int i = 0; i < 8; i++)
        {
            MESH_MODEL_PRINT_DEBUG("Object ID[%d] = 0x%x\r\n", i, start->Object_ID[i]);
            p_m_fnd_blob_env->Object_ID[i] =  start->Object_ID[i];
        }
        MESH_MODEL_PRINT_DEBUG("m_fnd_blob_obj_blk_trans_start_t size = %d\r\n", sizeof(m_fnd_blob_obj_blk_trans_start_t));

        p_m_fnd_blob_env->Block_Number = start->Block_Number;
        p_m_fnd_blob_env->Block_Checksum_Value = start->Block_Checksum_Value;
        MESH_MODEL_PRINT_DEBUG("Block_Number = 0x%x\r\n", start->Block_Number);
        MESH_MODEL_PRINT_DEBUG("Chunk_Size = 0x%x\r\n", start->Chunk_Size);
        MESH_MODEL_PRINT_DEBUG("Checksum_Algorithm = 0x%x\r\n", start->Block_Checksum_Algorithm);
        MESH_MODEL_PRINT_DEBUG("Checksum_Value = 0x%x\r\n", start->Block_Checksum_Value);

        memset(&ota_info, 0, sizeof(nvds_mesh_ota_tag_t));
        if (NVDS_OK == nvds_get(NVDS_TAG_MESH_OTA_INFO, &length, (uint8_t *)&ota_info))
        {
            if (p_m_fnd_blob_env->Block_Number < ota_info.Block_Num)
            {
                // Send Status message
                m_fnd_blob_send_model_obj_blk_trans_status(BLOB_BLK_TRANS_STATUS_DUPLICATIN_BLOCK);
            }
            else
            {
                ota_info.Block_Num = p_m_fnd_blob_env->Block_Number;
                ota_info.Current_Block_Size = p_m_fnd_blob_env->Current_Block_Size;
                ota_info.Receive_Chunk_Mask = p_m_fnd_blob_env->Receive_Chunk_Mask;
                ota_info.Current_Chunk_Mask =  p_m_fnd_blob_env->Current_Chunk_Mask;
                ota_info.Block_Checksum_Value = p_m_fnd_blob_env->Block_Checksum_Value;
                memcpy(ota_info.Object_ID, p_m_fnd_blob_env->Object_ID, 8);

                // Send Status message
                m_fnd_blob_send_model_obj_blk_trans_status(BLOB_BLK_TRANS_STATUS_ACCEPTED);
                if (NVDS_OK == nvds_put(NVDS_TAG_MESH_OTA_INFO, length, (uint8_t*)&ota_info))
                {
                    MESH_MODEL_PRINT_DEBUG("nvds_put---NVDS_TAG_MESH_OTA_INFO success 1\r\n");
                }
                else
                {
                    MESH_MODEL_PRINT_DEBUG("nvds_put---NVDS_TAG_MESH_OTA_INFO fail 1 !!!!\r\n");
                }
            }

        }
        else
        {
            ota_info.Block_Num = p_m_fnd_blob_env->Block_Number;
            ota_info.Current_Block_Size = p_m_fnd_blob_env->Current_Block_Size;
            ota_info.Receive_Chunk_Mask = p_m_fnd_blob_env->Receive_Chunk_Mask;
            ota_info.Current_Chunk_Mask =  p_m_fnd_blob_env->Current_Chunk_Mask;
            ota_info.Block_Checksum_Value = p_m_fnd_blob_env->Block_Checksum_Value;
            memcpy(ota_info.Object_ID, p_m_fnd_blob_env->Object_ID, 8);
            // Send Status message
            m_fnd_blob_send_model_obj_blk_trans_status(BLOB_BLK_TRANS_STATUS_ACCEPTED);
            if (NVDS_OK == nvds_put(NVDS_TAG_MESH_OTA_INFO, length, (uint8_t*)&ota_info))
            {
                MESH_MODEL_PRINT_DEBUG("nvds_put---NVDS_TAG_MESH_OTA_INFO success 2\r\n");
            }
            else
            {
                MESH_MODEL_PRINT_DEBUG("nvds_put---NVDS_TAG_MESH_OTA_INFO fail 2!!!!\r\n");
            }

        }


    }
    else if (opcode == M_FND_BLOB_OPCODE_OBJ_BLK_GET)
    {
        MESH_MODEL_PRINT_DEBUG("opcode:M_FND_BLOB_OPCODE_OBJ_BLK_GET step: 8\r\n");

        m_fnd_blob_obj_blk_get_t  *get = (m_fnd_blob_obj_blk_get_t *) MESH_TB_BUF_DATA(p_buf);

        for (int i = 0; i < 8; i++)
        {
            MESH_MODEL_PRINT_DEBUG("Object ID[%d] = 0x%x\r\n", i, get->Object_ID[i]);
        }
        MESH_MODEL_PRINT_DEBUG("m_fnd_blob_obj_blk_trans_start_t size = %d\r\n", sizeof(m_fnd_blob_obj_blk_get_t));
        MESH_MODEL_PRINT_DEBUG("Block_Number = 0x%x\r\n", get->Block_Number);
        // Send Status message
        m_fnd_blob_send_model_obj_blk_status();

    }
    // Process next received message
    m_fnd_blob_process_next();

}

/**
 ****************************************************************************************
 * @brief Handle Blob Transfer Model chunk Transfer
 * Unacknowledged message received from provisioner.
 *
 * @param[in] p_buf     Pointer to the buffer containing the message.
 * @param[in] opcode    Operation code
 ****************************************************************************************
 */


__STATIC void m_fnd_blob_handler_obj_chunk(mesh_tb_buf_t *p_buf, uint8_t opcode)
{
    MESH_MODEL_PRINT_DEBUG("%s,opcode = %x\r\n", __func__, opcode);

    if (opcode == M_FND_BLOB_OPCODE_OBJ_CHUNK_TRANS)
    {
        m_fnd_blob_obj_chunk_trans_t  *chunk = (m_fnd_blob_obj_chunk_trans_t *) MESH_TB_BUF_DATA(p_buf);

        MESH_MODEL_PRINT_DEBUG("opcode:M_FND_BLOB_OPCODE_OBJ_CHUNK_TRANS step: 7\r\n");
        uint32_t write_addr;
        uint32_t crc_value ;
        nvds_tag_len_t length = sizeof(nvds_mesh_ota_tag_t);

        MESH_MODEL_PRINT_DEBUG("Block_Number = 0x%x\r\n", p_m_fnd_blob_env->Block_Number);
        MESH_MODEL_PRINT_DEBUG("Chunk_Number = 0x%x\r\n", chunk->Chunk_Number);
        MESH_MODEL_PRINT_DEBUG("Current_Block_Size = 0x%x\r\n", p_m_fnd_blob_env->Current_Block_Size);

        p_m_fnd_blob_env->Receive_Chunk_Mask |= (0x01 << chunk->Chunk_Number);
        MESH_MODEL_PRINT_DEBUG("chunk_mask = 0x%04x\r\n", p_m_fnd_blob_env->Receive_Chunk_Mask);

        write_addr = SEC_IMAGE_BACKUP_OAD_HEADER_FADDR + p_m_fnd_blob_env->Block_Number * 0x1000 + chunk->Chunk_Number * 0x100;

        GLOBAL_INT_DISABLE();
        if ((write_addr % 0x1000) == 0)
        {
            MESH_MODEL_PRINT_DEBUG("flash_erase addr = 0x%08x\r\n", write_addr);
            flash_erase(FLASH_MAIN_BASE_ADDR, write_addr, 0x1000, NULL);
        }
        MESH_MODEL_PRINT_DEBUG("flash_write_addr = 0x%08x\r\n", write_addr);
        flash_write(FLASH_MAIN_BASE_ADDR, write_addr, p_buf->data_len - 2, chunk->Chunk_Data, NULL);
        MESH_MODEL_PRINT_DEBUG("p_buf->data_len = %d,\r\n", p_buf->data_len);
        MESH_MODEL_PRINT_DEBUG("Chunk_data = \r\n");
        GLOBAL_INT_RESTORE();
        for (int i = 0; i < p_buf->data_len - 2; i++)
        {

            MESH_MODEL_PRINT_DEBUG("%02x ", chunk->Chunk_Data[i]);
            if (i % 16 == 15)
            {
                MESH_MODEL_PRINT_DEBUG("\r\n");
            }
            crc_buf[chunk->Chunk_Number*256 + i] = chunk->Chunk_Data[i];

        } MESH_MODEL_PRINT_DEBUG("\r\n");
        //
        ota_info.Receive_Chunk_Mask = p_m_fnd_blob_env->Receive_Chunk_Mask;
        if (NVDS_OK == nvds_put(NVDS_TAG_MESH_OTA_INFO, length, (uint8_t*)&ota_info))
        {

        }
        else
        {

            MESH_MODEL_PRINT_DEBUG("nvds_put(NVDS_TAG_MESH_OTA_INFO fail!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\r\n");
        }


        if ( (p_m_fnd_blob_env->Receive_Chunk_Mask == p_m_fnd_blob_env->Current_Chunk_Mask))
        {
            GLOBAL_INT_DISABLE();
            crc_value = 0xFFFFFFFF;
            make_crc32_table(0xedb88320);
            crc_value  = make_crc32(crc_value, crc_buf, p_m_fnd_blob_env->Current_Block_Size);
            GLOBAL_INT_RESTORE();
            MESH_MODEL_PRINT_DEBUG("calcuCrc crc_value = 0x%x\r\n", crc_value);
            MESH_MODEL_PRINT_DEBUG("Block_Checksum_Value = 0x%x\r\n", p_m_fnd_blob_env->Block_Checksum_Value);
            if (p_m_fnd_blob_env->Block_Checksum_Value == crc_value)
            {
                MESH_MODEL_PRINT_DEBUG("calcuCrc Success!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\r\n");
                //  while(1);
            }
            else
            {
                MESH_MODEL_PRINT_DEBUG("calcuCrc fail!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\r\n");
            }


        }
    }
    // Process next received message
    m_fnd_blob_process_next();

}

/**
 ****************************************************************************************
 * @brief Handle Blob Transfer Model Block Transfer Start or Block Get
 * Unacknowledged message received from provisioner.
 *
 * @param[in] p_buf     Pointer to the buffer containing the message.
 * @param[in] opcode    Operation code
 ****************************************************************************************
 */


__STATIC void m_fnd_blob_handler_obj_capabilities(mesh_tb_buf_t *p_buf, uint8_t opcode)
{
    MESH_MODEL_PRINT_DEBUG("%s,opcode = %x\r\n", __func__, opcode);
    // M_FND_BLOB_OPCODE_OBJ_INFO_GET
    if (opcode == M_FND_BLOB_OPCODE_OBJ_INFO_GET)
    {
        MESH_MODEL_PRINT_DEBUG("opcode:M_FND_BLOB_OPCODE_OBJ_INFO_GET step: 2\r\n");

        // Send Status message
        m_fnd_blob_send_model_obj_info_status(MIN_BLOCK_SIZE_LOG, MAX_BLOCK_SIZE_LOG, MAX_CHUNKS_NUMBER);
    }
    // Process next received message
    m_fnd_blob_process_next();
}




/**
 ****************************************************************************************
 * @brief Call the appropriate handler for a given buffer.
 *
 * @param[in] p_buf     Buffer to handle.
 ****************************************************************************************
 */
__STATIC void m_fnd_blob_process(m_api_buf_t *p_buf)
{

    MESH_MODEL_PRINT_DEBUG("%s\r\n", __func__);
    // Get buffer environment
    m_lay_buf_env_t *p_env = (m_lay_buf_env_t *)(&((mesh_tb_buf_t *)p_buf)->env[0]);

    // Check that opcode is a 1-byte opcode
    if (MESH_IS_1_OCT_OPCODE(p_env->u5.opcode))
    {
        // Get only first byte of the opcode
        uint8_t opcode_1b = (uint8_t)(p_env->u5.opcode);
        switch (opcode_1b)
        {
            // states Block
            case (M_FND_BLOB_OPCODE_OBJ_BLK_GET)        : m_fnd_blob_handler_obj_blk(p_buf, opcode_1b); break;
            // states Chunk
            case (M_FND_BLOB_OPCODE_OBJ_CHUNK_TRANS)    : m_fnd_blob_handler_obj_chunk(p_buf, opcode_1b); break;
            default                                     : m_fnd_blob_process_next(); break;
        }

    }
    else
    {
        // Get only second byte of the opcode
        uint8_t opcode_2b = (uint8_t)(p_env->u5.opcode >> 8);

        switch (opcode_2b)
        {
            // states Object
            case (M_FND_BLOB_OPCODE_OBJ_TRANS_GET)          :
            case (M_FND_BLOB_OPCODE_OBJ_TRANS_START)        :
            case (M_FND_BLOB_OPCODE_OBJ_TRANS_ABORT)        : m_fnd_blob_handler_obj_obj(p_buf, opcode_2b);  break;
            // states Block
            case (M_FND_BLOB_OPCODE_OBJ_BLK_TRANS_START)    : m_fnd_blob_handler_obj_blk(p_buf, opcode_2b);  break;
            // states Capabilities
            case (M_FND_BLOB_OPCODE_OBJ_INFO_GET)           : m_fnd_blob_handler_obj_capabilities(p_buf, opcode_2b);  break;
            default                                         : m_fnd_blob_process_next();                       break;
        }
    }
}




/**
 ****************************************************************************************
 * @brief Pop first buffer from the processing queue, release it and start processing of
 * next one in the processing queue.
 ****************************************************************************************
 */
__STATIC void m_fnd_blob_process_next(void)
{

    MESH_MODEL_PRINT_DEBUG("%s\r\n", __func__);
    // Remove first buffer in the process queue
    mesh_tb_buf_t *p_buf = (mesh_tb_buf_t *)co_list_pop_front(&p_m_fnd_blob_env->process_queue);

    // Release the buffer
    mesh_tb_buf_release(p_buf);

    // Process next buffer
    p_buf = (mesh_tb_buf_t *)co_list_pick(&p_m_fnd_blob_env->process_queue);

    if (p_buf != NULL)
    {
        m_fnd_blob_process(p_buf);
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
__STATIC void m_fnd_blob_cb_rx(m_lid_t model_lid, uint32_t opcode, m_api_buf_t *p_api_buf, m_lid_t app_key_lid,
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
    co_list_push_back(&p_m_fnd_blob_env->process_queue, &p_buf->hdr);

    // Check if buffer can be processed now or if another buffer is currently processed
    if ((mesh_tb_buf_t *)co_list_pick(&p_m_fnd_blob_env->process_queue) == p_buf)
    {
        m_fnd_blob_process(p_buf);
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
__STATIC void m_fnd_blob_cb_opcode_check(m_lid_t model_lid, uint32_t opcode)
{

    // Status
    uint16_t status = MESH_ERR_NOT_SUPPORTED;

    // Check that opcode is a 1-byte opcode
    if (MESH_IS_1_OCT_OPCODE(opcode))
    {
        // Get only second byte of the opcode
        uint8_t opcode_1b = (uint8_t)(opcode);

        switch (opcode_1b)
        {
            case M_FND_BLOB_OPCODE_OBJ_CHUNK_TRANS:
            case M_FND_BLOB_OPCODE_OBJ_BLK_GET:
            {
                status = MESH_ERR_NO_ERROR;
            } break;
            default:break;
        }
    }
    else if (MESH_IS_2_OCT_OPCODE(opcode) && M_IS_TRANSFER_OPCODE(opcode))
    {
        // Get only second byte of the opcode
        uint8_t opcode_2b = (uint8_t)(opcode >> 8);

        switch (opcode_2b)
        {
            case M_FND_BLOB_OPCODE_OBJ_TRANS_GET:
            case M_FND_BLOB_OPCODE_OBJ_TRANS_START:
            case M_FND_BLOB_OPCODE_OBJ_TRANS_ABORT:
            case M_FND_BLOB_OPCODE_OBJ_TRANS_STATUS:
            case M_FND_BLOB_OPCODE_OBJ_BLK_TRANS_START:
            case M_FND_BLOB_OPCODE_OBJ_BLK_TRANS_STATUS:
            case M_FND_BLOB_OPCODE_OBJ_BLK_STATUS:
            case M_FND_BLOB_OPCODE_OBJ_INFO_GET:
            case M_FND_BLOB_OPCODE_OBJ_INFO_STATUS:
            {
                status = MESH_ERR_NO_ERROR;
            } break;
            default:break;
        }

    }




    MESH_MODEL_PRINT_DEBUG("%s,opcode = 0x%x,status = 0x%x\r\n", __func__, opcode, status);
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
__STATIC void m_fnd_blob_cb_sent(m_lid_t model_lid, uint8_t tx_hdl, m_api_buf_t *p_api_buf, uint16_t status)
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
__STATIC void m_fnd_blob_cb_publish_period(m_lid_t model_lid, uint16_t addr, uint32_t period_ms)
{
    // Inform the application about the new publication period
    // m_api_send_fault_period_ind(period_ms, period_ms >> p_m_fnd_hlths_env->fast_period_divisor);
}


/*
 * CALLBACK FUNCTIONS
 ****************************************************************************************
 */

/// Health Client Model callback functions
const m_api_model_cb_t m_fnd_blob_cb =
{
    .cb_rx             = m_fnd_blob_cb_rx,
    .cb_sent           = m_fnd_blob_cb_sent,
    .cb_opcode_check   = m_fnd_blob_cb_opcode_check,
    .cb_publish_param = m_fnd_blob_cb_publish_period,
};



/*
 * GLOBAL FUNCTIONS
 ****************************************************************************************
 */

/*
typedef struct nvds_mesh_ota_tag
{
    uint8_t Object_ID[8];
    uint16_t Block_Num; //  正常进行trans 的Block Num

    uint16_t Current_Block_Size; //  正常进行trans 的 Block size

    uint16_t Receive_Chunk_Mask;
    uint16_t Chunk_Total_Num;
//  uint16_t Blk_Chunk_Mask;
    uint32_t Block_Checksum_Value;// Block Checksum_Value


}nvds_mesh_ota_tag_t;
*/

//uint16_t m_fnd_blob_init(bool reset, void *p_env, const m_cfg_t* p_cfg)
uint16_t m_fnd_blob_init(bool reset)
{
    MESH_MODEL_PRINT_DEBUG("m_fnd_blob_init\r\n");
    if (!reset)
    {
        // Get environment for Blob Transfer Client model
        //p_m_fnd_blob_env = (m_fnd_blob_env_t *)p_env;
        p_m_fnd_blob_env = mal_malloc(sizeof(m_fnd_blob_env_t));

        uint8_t length = sizeof(nvds_mesh_ota_tag_t);
        // Initialize environment
        memset(p_m_fnd_blob_env, 0, sizeof(m_fnd_blob_env_t));
        memset(&ota_info, 0, sizeof(nvds_mesh_ota_tag_t));
        if (NVDS_OK == nvds_get(NVDS_TAG_MESH_OTA_INFO, &length, (uint8_t *)&ota_info))
        {
            MESH_MODEL_PRINT_DEBUG("NVDS_OK\r\n");
            MESH_MODEL_PRINT_DEBUG("Block_Num = 0x%x\r\n", ota_info.Block_Num);
            MESH_MODEL_PRINT_DEBUG("Current_Block_Size = 0x%x\r\n", ota_info.Current_Block_Size);
            MESH_MODEL_PRINT_DEBUG("Receive_Chunk_Mask = 0x%x\r\n", ota_info.Receive_Chunk_Mask);
            MESH_MODEL_PRINT_DEBUG("Current_Chunk_Mask = 0x%x\r\n", ota_info.Current_Chunk_Mask);
            MESH_MODEL_PRINT_DEBUG("Block_Checksum_Value = 0x%x\r\n", ota_info.Block_Checksum_Value);
            MESH_MODEL_PRINT_DEBUG("Object_ID = 0x");
            for (int i = 0; i < 8; i++)
            {
                MESH_MODEL_PRINT_DEBUG("%02x ", ota_info.Object_ID[i]);
            } MESH_MODEL_PRINT_DEBUG("\r\n");
            memcpy(p_m_fnd_blob_env->Object_ID, ota_info.Object_ID, 8);
            p_m_fnd_blob_env->Block_Number = ota_info.Block_Num;
            p_m_fnd_blob_env->Current_Block_Size = ota_info.Current_Block_Size ;
            p_m_fnd_blob_env->Receive_Chunk_Mask = ota_info.Receive_Chunk_Mask ;
            p_m_fnd_blob_env->Current_Chunk_Mask = ota_info.Current_Chunk_Mask;
            p_m_fnd_blob_env->Block_Checksum_Value = ota_info.Block_Checksum_Value;

        }
        else
        {
            MESH_MODEL_PRINT_DEBUG("NVDS_Fail\r\n");
            MESH_MODEL_PRINT_DEBUG("Block_Num = 0x%x\r\n", ota_info.Block_Num);
            MESH_MODEL_PRINT_DEBUG("Current_Block_Size = 0x%x\r\n", ota_info.Current_Block_Size);
            MESH_MODEL_PRINT_DEBUG("Receive_Chunk_Mask = 0x%x\r\n", ota_info.Receive_Chunk_Mask);
            MESH_MODEL_PRINT_DEBUG("Current_Chunk_Mask = 0x%x\r\n", ota_info.Current_Chunk_Mask);
            MESH_MODEL_PRINT_DEBUG("Block_Checksum_Value = 0x%x\r\n", ota_info.Block_Checksum_Value);
        }




        // Register the model
        m_api_register_model(M_FND_BLOB_TRANSFER_MODEL_ID, 0, false, &m_fnd_blob_cb,
                             &p_m_fnd_blob_env->model_lid);

    }
    else
    {
        p_m_fnd_blob_env = NULL;
    }

    // Return environment size
    return (sizeof(m_fnd_blob_env_t));
}


uint16_t m_fnd_blob_get_env_size(const m_cfg_t* p_cfg)
{
    // Return environment size
    return (sizeof(m_fnd_blob_env_t));
}



/// @} end of group
