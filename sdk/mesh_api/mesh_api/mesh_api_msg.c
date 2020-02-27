/**
 ****************************************************************************************
 *
 * @file mesh_api_msg.c
 *
 * @brief Mesh Stack Message Application Programming Interface
 *
 * Copyright (C) RivieraWaves 2017-2019
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup MESH_API_MSG
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "mesh_inc.h"           // Mesh Stack Include Files

#if (BLE_MESH_MSG_API)

/*
 * LOCAL FUNCTIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Send a basic command complete message to the application
 *
 * @param[in] cmd_code  Debug Command Operation code (@see enum mesh_dbg_cmd_code)
 * @param[in] src_id    Source identifier of task that request command execution
 * @param[in] status    Status error code of the command execution (@see enum mesh_err)
 ****************************************************************************************
 */
__STATIC void mesh_api_msg_basic_cmp_evt_send(uint32_t cmd_code, uint16_t src_id, uint16_t status)
{
    // Allocate the command complete event message
    mesh_api_cmp_evt_t *p_cmp_evt = MAL_MSG_ALLOC(MESH_COMMON_API_CMP_EVT, mesh_api_cmp_evt_t);

    // Fill the content
    p_cmp_evt->cmd_code = cmd_code;
    p_cmp_evt->status = status;

    // Send the message
    mal_msg_send(src_id, p_cmp_evt);
}

/*
 * CALLBACKS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Callback function called by once a block has been freed.
 *
 * @param[in] block_id      Index of the block that has been freed.
 ****************************************************************************************
 */
__STATIC void mesh_api_msg_buf_block_freed_cb(uint8_t block_id)
{
    // Allocate command complete event message
    mesh_api_buf_free_block_cmp_evt_t *p_cmp_evt
        = MAL_MSG_ALLOC(MESH_COMMON_API_CMP_EVT, mesh_api_buf_free_block_cmp_evt_t);

    // Fill the indication message
    p_cmp_evt->cmd_code = MESH_API_BUF_FREE_BLOCK;
    p_cmp_evt->status = MESH_ERR_NO_ERROR;
    p_cmp_evt->block_id = block_id;

    // Fill the indication message
    mal_msg_send(mal_app_id_get(), p_cmp_evt);
}

/*
 * GLOBAL VARIABLES
 ****************************************************************************************
 */

/// Callbacks structure for Mesh Stack
__STATIC const mesh_api_cb_t mesh_api_msg_cb =
{
    .cb_buf_block_freed = mesh_api_msg_buf_block_freed_cb,
};

/*
 * COMMAND HANDLERS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Handle MESH_API_SET_RUN_TIME command
 *
 * @param[in] src_id    Source identifier of task that request command execution
 * @param[in] p_cmd     Pointer to command parameters
 ****************************************************************************************
 */
__STATIC void mesh_api_set_run_time_cmd_handler(uint16_t src_id, const mesh_api_set_run_time_cmd_t *p_cmd)
{
    // Set clock information
    uint16_t status = mesh_api_set_run_time(p_cmd->clock_ms, p_cmd->nb_wrap);

    // Send the command complete message
    mesh_api_msg_basic_cmp_evt_send(MESH_API_SET_RUN_TIME, src_id, status);
}

/**
 ****************************************************************************************
 * @brief Handle MESH_API_SET_RUN_TIME command
 *
 * @param[in] src_id    Source identifier of task that request command execution
 ****************************************************************************************
 */
__STATIC void mesh_api_get_run_time_cmd_handler(uint16_t src_id)
{
    // Allocate command complete message
    mesh_api_get_run_time_cmp_evt_t *p_cmp_evt
        = MAL_MSG_ALLOC(MESH_COMMON_API_CMP_EVT, mesh_api_get_run_time_cmp_evt_t);

    // Fill the command complete message
    p_cmp_evt->cmd_code = MESH_API_GET_RUN_TIME;
    p_cmp_evt->status = MESH_ERR_NO_ERROR;

    // Get clock information
    mesh_api_get_run_time(&p_cmp_evt->clock_ms, &p_cmp_evt->nb_wrap);

    // Send the command complete message
    mal_msg_send(src_id, p_cmp_evt);
}

/**
 ****************************************************************************************
 * @brief Handle MESH_API_GET_VERSION command
 *
 * @param[in] src_id    Source identifier of task that request command execution
 ****************************************************************************************
 */
__STATIC void mesh_api_get_version_cmd_handler(uint16_t src_id)
{
    // Allocate command complete message
    mesh_api_get_version_cmp_evt_t *p_cmp_evt
        = MAL_MSG_ALLOC(MESH_COMMON_API_CMP_EVT, mesh_api_get_version_cmp_evt_t);

    // Fill the command complete message
    p_cmp_evt->cmd_code = MESH_API_GET_VERSION;
    p_cmp_evt->status = MESH_ERR_NO_ERROR;

    // Get version
    mesh_api_get_version(&p_cmp_evt->version);

    // Send the command complete message
    mal_msg_send(src_id, p_cmp_evt);
}

/**
 ****************************************************************************************
 * @brief Handle MESH_API_BUF_ALLOC_BLOCK command
 *
 * @param[in] src_id    Source identifier of task that request command execution
 * @param[in] p_cmd     Pointer to command parameters
 ****************************************************************************************
 */
__STATIC void mesh_api_buf_alloc_block_cmd_handler(uint16_t src_id, const mesh_api_buf_alloc_block_cmd_t *p_cmd)
{
    // Block ID
    uint8_t block_id;
    // Add required block
    uint16_t status = mesh_api_buf_alloc_block(&block_id, p_cmd->nb_bufs, p_cmd->small);
    // Allocate command complete message
    mesh_api_buf_alloc_block_cmp_evt_t *p_cmp_evt
        = MAL_MSG_ALLOC(MESH_COMMON_API_CMP_EVT, mesh_api_buf_alloc_block_cmp_evt_t);

    // Fill the command complete message
    p_cmp_evt->cmd_code = MESH_API_BUF_ALLOC_BLOCK;
    p_cmp_evt->status = status;
    p_cmp_evt->block_id = block_id;

    // Send the command complete message
    mal_msg_send(src_id, p_cmp_evt);
}

/**
 ****************************************************************************************
 * @brief Handle MESH_API_BUF_FREE_BLOCK command
 *
 * @param[in] src_id    Source identifier of task that request command execution
 * @param[in] p_cmd     Pointer to command parameters
 ****************************************************************************************
 */
__STATIC void mesh_api_buf_free_block_cmd_handler(uint16_t src_id, const mesh_api_buf_free_block_cmd_t *p_cmd)
{
    // Free required block
    uint16_t status = mesh_api_buf_free_block(p_cmd->block_id);

    if (status != MESH_ERR_NO_ERROR)
    {
        // Send the command complete message
        mesh_api_msg_basic_cmp_evt_send(MESH_API_BUF_FREE_BLOCK, src_id, status);
    }
}

/*
 * MESSAGE HANDLERS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief General handler for MESH_COMMON_API_CMD message
 *
 * @param[in] src_id    Identifier of the task that issue message
 * @param[in] p_cmd     Pointer to command parameters
 *
 * @return Status of message after execution of the handler (@see enum mal_msg_status)
 ****************************************************************************************
 */
__STATIC uint8_t mesh_api_msg_handler_cmd(uint16_t src_id, const mesh_api_cmd_t *p_cmd)
{
    // Call the appropriate handler
    switch (p_cmd->cmd_code)
    {
        case (MESH_API_GET_RUN_TIME):
        {
            mesh_api_get_run_time_cmd_handler(src_id);
        } break;

        case (MESH_API_SET_RUN_TIME):
        {
            mesh_api_set_run_time_cmd_handler(src_id, (const mesh_api_set_run_time_cmd_t *)p_cmd);
        } break;

        case (MESH_API_GET_VERSION):
        {
            mesh_api_get_version_cmd_handler(src_id);
        } break;

        case (MESH_API_BUF_ALLOC_BLOCK):
        {
            mesh_api_buf_alloc_block_cmd_handler(src_id, (const mesh_api_buf_alloc_block_cmd_t *)p_cmd);
        } break;

        case (MESH_API_BUF_FREE_BLOCK):
        {
            mesh_api_buf_free_block_cmd_handler(src_id, (const mesh_api_buf_free_block_cmd_t *)p_cmd);
        } break;

        default:
        {
            // Inform application about command handling
            mesh_api_msg_basic_cmp_evt_send(p_cmd->cmd_code, src_id, MESH_ERR_COMMAND_DISALLOWED);
        } break;
    }

    return (MAL_MSG_FREE);
}

/*
 * GLOBAL FUNCTIONS
 ****************************************************************************************
 */

uint16_t mesh_api_msg_init(bool reset, void *p_env, const mesh_cfg_t *p_cfg)
{
    // Set the callbacks for asynchronous events
    mesh_api_set(&mesh_api_msg_cb);

    // No environment
    return (0);
}

uint16_t mesh_api_msg_get_env_size(const mesh_cfg_t *p_cfg)
{
    // No environment
    return (0);
}

uint8_t mesh_api_msg_handler(uint16_t msg_id, uint16_t src_id, const void *p_param)
{
    uint8_t msg_status = MAL_MSG_FREE;

    switch (msg_id)
    {
        // Mesh command
        case (MESH_COMMON_API_CMD):
        {
            msg_status = mesh_api_msg_handler_cmd(src_id, (const mesh_api_cmd_t *)p_param);
        } break;

        // Drop the message
        default:
        {
        } break;
    }

    return (msg_status);
}

#endif //(BLE_MESH_MSG_API)

/// @} MESH_API_MSG
