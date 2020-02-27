/**
 ****************************************************************************************
 *
 * @file mesh_api.c
 *
 * @brief Mesh Stack Application Programming Interface
 *
 * Copyright (C) RivieraWaves 2017-2019
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup MESH_API
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "mesh_inc.h"        // Mesh Stack Include Files
//#include "mesh_dbg.h"        // Mesh Stack Debug Module Definitions
#include "mesh_tb.h"         // Mesh Stack Toolbox Definitions
#include "mesh_tb_buf.h"     // Mesh Buffer Manager Definitions
#include "mesh_version.h"    // Mesh Version Definitions

/*
 * EXTERNAL FUNCTIONS DECLARATION
 ****************************************************************************************
 */

#if (BLE_MESH_MSG_API)
extern uint16_t mesh_api_msg_init(bool reset, void *p_env, const mesh_cfg_t *p_cfg);
extern uint16_t mesh_api_msg_get_env_size(const mesh_cfg_t *p_cfg);
extern uint8_t mesh_api_msg_handler(uint16_t msg_id, uint16_t src_id, const void *p_param);
#endif //(BLE_MESH_MSG_API)

/*
 * LOCAL VARIABLES
 ****************************************************************************************
 */

/// Callback used to inform upper application about asynchronous events
const mesh_api_cb_t *p_mesh_api_cb;

/*
 * GLOBAL FUNCTIONS
 ****************************************************************************************
 */

uint16_t mesh_api_init(bool reset, void *p_env, const mesh_cfg_t *p_cfg)
{
    // Get address of memory block reserved for Mesh Stack environment
    uint8_t *p_env_cursor = (uint8_t *)p_env;

    p_env_cursor += CO_ALIGN4_HI(mesh_tb_init(reset, (void *)p_env_cursor, NULL));
#if (BLE_MESH_DBG)
    p_env_cursor += CO_ALIGN4_HI(mesh_dbg_init(reset, (void *)p_env_cursor, NULL));
#endif //(BLE_MESH_DBG)
#if (BLE_MESH_MSG_API)
    p_env_cursor += CO_ALIGN4_HI(mesh_api_msg_init(reset, (void *)p_env_cursor, NULL));
#endif //(BLE_MESH_MSG_API)

    // Return size of used environment
    return (((uint32_t)p_env_cursor) - ((uint32_t)p_env));
}

uint16_t mesh_api_get_env_size(const mesh_cfg_t *p_cfg)
{
    uint16_t total_env_size = 0;

    total_env_size += CO_ALIGN4_HI(mesh_tb_get_env_size(NULL));
#if (BLE_MESH_DBG)
    total_env_size += CO_ALIGN4_HI(mesh_dbg_get_env_size(NULL));
#endif //(BLE_MESH_DBG)
#if (BLE_MESH_MSG_API)
    total_env_size += CO_ALIGN4_HI(mesh_api_msg_get_env_size(NULL));
#endif //(BLE_MESH_MSG_API)

    // Return size of environment
    return (total_env_size);
}

uint8_t mesh_api_handler(uint16_t msg_id, uint16_t src_id, const void *p_param)
{
    uint8_t msg_status = MAL_MSG_FREE;

    // Mesh Common API
    if (msg_id < MESH_COMMON_DBG_FIRST)
    {
#if (BLE_MESH_MSG_API)
        msg_status = mesh_api_msg_handler(msg_id, src_id, p_param);
#else //(BLE_MESH_MSG_API)
        ASSERT_WARN(0, msg_id, src_id);
#endif //(BLE_MESH_MSG_API)
    }
    // Mesh Common API
    else
    {
#if (BLE_MESH_DBG)
        msg_status = mesh_dbg_handler(msg_id, src_id, p_param);
#else //(BLE_MESH_DBG)
        ASSERT_WARN(0, msg_id, src_id);
#endif //(BLE_MESH_DBG)
    }

    return (msg_status);
}

uint16_t mesh_api_set(const mesh_api_cb_t *p_cb_api)
{
    uint16_t status = MESH_ERR_INVALID_PARAM;

    // Check that all callbacks are configured
    if ((p_cb_api->cb_buf_block_freed != NULL))
    {
        p_mesh_api_cb = p_cb_api;
        status = MESH_ERR_NO_ERROR;
    }
    else
    {
        ASSERT_ERR(0);
    }

    return (status);
}

void mesh_api_get_run_time(uint32_t *p_clock_ms, uint16_t *p_nb_wrap)
{
    // Get clock information
    mal_timer_get_clock(p_clock_ms, p_nb_wrap);
}

void mesh_api_get_version(mesh_version_t *p_version)
{
    /// Mesh Specification version (X.Y.Z)
    p_version->mesh_version_x = MESH_VERSION_X;
    p_version->mesh_version_y = MESH_VERSION_Y;
    p_version->mesh_version_z = MESH_VERSION_Z;

    /// Mesh Software version (X.Y)
    p_version->mesh_sw_version_x = MESH_SW_VERSION_X;
    p_version->mesh_sw_version_y = MESH_SW_VERSION_Y;
}

uint16_t mesh_api_set_run_time(uint32_t clock_ms, uint16_t nb_wrap)
{
    // Set time
    mal_timer_set_clock(clock_ms, nb_wrap);

    return (MESH_ERR_NO_ERROR);
}

uint16_t mesh_api_buf_alloc_block(uint8_t *p_block_id, uint8_t nb_bufs, bool small)
{
    // Allocate a block of buffer
    return (mesh_tb_buf_block_alloc(p_block_id, nb_bufs, small));
}

uint16_t mesh_api_buf_free_block(uint8_t block_id)
{
    uint16_t status = MESH_ERR_COMMAND_DISALLOWED;

    if (p_mesh_api_cb != NULL)
    {
        // Free indicated block of buffer
        status = mesh_tb_buf_block_free(block_id, p_mesh_api_cb->cb_buf_block_freed);
    }

    return (status);
}

/// @} end of group
