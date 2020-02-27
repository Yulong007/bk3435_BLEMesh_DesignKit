/**
 ****************************************************************************************
 *
 * @file mm_genc_loc.c
 *
 * @brief Mesh Model Generic Location Client Module
 *
 * Copyright (C) RivieraWaves 2017-2019
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup MM_GENC_LOC
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "mm_genc_int.h"      // Mesh Model Generic Server Module Internal Definitions


#if (BLE_MESH_MDL_GENC_LOC)
/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

/// Structure for Generic Location Client model environment
typedef struct mm_genc_loc_env
{
    /// Basic model environment - Must be first element in the structure - DO NOT MOVE
    mm_tb_state_mdl_env_t env;
} mm_genc_loc_env_t;

/*
 * INTERNAL CALLBACK FUNCTIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Inform Generic Location Client model about reception of a message
 *
 * @param[in] p_env         Pointer to the environment allocated for the model
 * @param[in] p_buf         Pointer to buffer containing the received message
 * @param[in] p_route_env   Pointer to routing environment containing information about the
 * received message
 ****************************************************************************************
 */
__STATIC void mm_genc_loc_cb_rx(mm_tb_state_mdl_env_t *p_env, mesh_tb_buf_t *p_buf,
                                mm_route_buf_env_t *p_route_env)
{
    // Get pointer to data
    uint8_t *p_data = MESH_TB_BUF_DATA(p_buf);

    if (p_route_env->opcode == MM_MSG_GEN_LOCG_STATUS)
    {
        // Global latitude value
        int32_t latitude = (int32_t)co_read32p(p_data + MM_GEN_LOC_STATUS_GLOB_LAT_POS);
        // Global longitude value
        int32_t longitude = (int32_t)co_read32p(p_data + MM_GEN_LOC_STATUS_GLOB_LONG_POS);
        // Global altitude value
        int16_t altitude = (int16_t)co_read16p(p_data + MM_GEN_LOC_STATUS_GLOB_ALT_POS);

        // Inform the application about the received Generic Location state value (global part)
        mm_api_send_cli_locg_ind(p_route_env->u_addr.src, latitude, longitude, altitude);
    }
    else
    {
        // Local north value
        int16_t north = (int16_t)co_read16p(p_data + MM_GEN_LOC_STATUS_LOC_NORTH_POS);
        // Local east value
        int16_t east = (int16_t)co_read16p(p_data + MM_GEN_LOC_STATUS_LOC_EAST_POS);
        // Local altitude value
        int16_t altitude = (int16_t)co_read16p(p_data + MM_GEN_LOC_STATUS_LOC_ALT_POS);
        // Floor value
        uint8_t floor = *(p_data + MM_GEN_LOC_STATUS_LOC_FLOOR_POS);
        // Uncertainty value
        uint16_t uncertainty = co_read16p(p_data + MM_GEN_LOC_STATUS_LOC_UNCERT_POS);

        // Inform the application about the received Generic Location state value (local part)
        mm_api_send_cli_locl_ind(p_route_env->u_addr.src, north, east, altitude,
                                 floor, uncertainty);
    }
}

/**
 ****************************************************************************************
 * @brief Inform Generic Location Client model about a received opcode in order
 * to check if the model is authorized to handle the associated message
 *
 * @param[in] p_env         Pointer to the environment allocated for the model
 * @param[in] opcode        Opcode value to be checked
 *
 * @return An error status (@see enum mesh_err)
 ****************************************************************************************
 */
__STATIC uint16_t mm_genc_loc_cb_opcode_check(mm_tb_state_mdl_env_t *p_env, uint32_t opcode)
{
    uint16_t status;

    if ((opcode == MM_MSG_GEN_LOCG_STATUS)
            || (opcode == MM_MSG_GEN_LOCL_STATUS))
    {
        status = MESH_ERR_NO_ERROR;
    }
    else
    {
        MESH_MODEL_PRINT_INFO("%s, Invalid opcode (0x%x).\n", __func__, opcode);
        status = MESH_ERR_MDL_INVALID_OPCODE;
    }

    return (status);
}

/**
 ****************************************************************************************
 * @brief Send a Generic Location Get message in order to retrieve Generic Location state
 * value (either global or local part) of a given node's element.
 *
 * @param[in] p_env         Pointer to the environment allocated for the model
 * @param[in] dst           Address of node's element from which Generic Location state value
 * must be retrieved
 * @param[in] get_info      Get information
 *
 * @return An error status (@see enum mesh_err)
 ****************************************************************************************
 */
__STATIC uint16_t mm_genc_loc_cb_get(mm_tb_state_mdl_env_t *p_env, uint16_t dst, uint16_t get_info)
{
    // Pointer to the buffer that will contain the message
    mesh_tb_buf_t *p_buf_get;
    // Allocate a new buffer for the message
    uint16_t status = mm_route_buf_alloc(&p_buf_get, 0);

    if (status == MESH_ERR_NO_ERROR)
    {
        // Get pointer to environment
        mm_route_buf_env_t *p_buf_env = (mm_route_buf_env_t *)&p_buf_get->env;

        // Prepare environment
        p_buf_env->app_key_lid = 6; // TODO [LT]
        p_buf_env->u_addr.dst = dst;
        p_buf_env->info = 0;
        p_buf_env->mdl_lid = p_env->mdl_lid;
        p_buf_env->opcode = (get_info == MM_GET_TYPE_LOC_GLOBAL)
                            ? MM_MSG_GEN_LOCG_GET : MM_MSG_GEN_LOCL_GET;

        // Send the message
        mm_route_send(p_buf_get);
    }
    else
    {
        MESH_MODEL_PRINT_WARN("%s, Alloc buffer fail.\n", __func__);
    }

    return (status);
}

/*
 * GLOBAL FUNCTIONS
 ****************************************************************************************
 */

uint16_t mm_genc_loc_register(void)
{
    // Model local index
    m_lid_t mdl_lid;
    // Register the model
    uint16_t status = m_api_register_model(MM_ID_GENC_LOC, 0, M_MDL_CONFIG_PUBLI_AUTH_BIT,
                                           &mm_route_cb, &mdl_lid);

    if (status == MESH_ERR_NO_ERROR)
    {
        // Inform the Model State Manager about registered model
        status = mm_tb_state_register(0, MM_ID_GENC_LOC, mdl_lid,
                                      MM_TB_STATE_ROLE_CLI | MM_TB_STATE_CFG_CB_BIT,
                                      sizeof(mm_genc_loc_env_t));

        if (status == MESH_ERR_NO_ERROR)
        {
            // Get allocated environment
            mm_genc_loc_env_t *p_env_loc = (mm_genc_loc_env_t *)mm_tb_state_get_env(mdl_lid);
            // Get client-specific callback functions
            mm_cli_cb_t *p_cb_cli = p_env_loc->env.cb.u.p_cb_cli;

            // Set internal callback functions
            p_env_loc->env.cb.cb_rx = mm_genc_loc_cb_rx;
            p_env_loc->env.cb.cb_opcode_check = mm_genc_loc_cb_opcode_check;
            p_cb_cli->cb_get = mm_genc_loc_cb_get;

            // Inform application about registered model
            mm_api_send_register_ind(MM_ID_GENC_LOC, 0, mdl_lid);
        }
        else
        {
            MESH_MODEL_PRINT_WARN("%s, Model State Register fail.\n", __func__);
        }
    }
    else
    {
        MESH_MODEL_PRINT_WARN("%s, Register the model fail.\n", __func__);
    }

    return (status);
}

uint16_t mm_genc_loc_set_global(mm_tb_state_mdl_env_t *p_env, uint16_t dst, uint8_t set_info,
                                int32_t latitude, int32_t longitude, int16_t altitude)
{
    // Pointer to the buffer that will contain the message
    mesh_tb_buf_t *p_buf_set;
    // Allocate a new buffer for the publication
    uint8_t status = mm_route_buf_alloc(&p_buf_set, MM_GEN_LOC_SET_GLOB_LEN);

    if (status == MESH_ERR_NO_ERROR)
    {
        // Get pointer to data
        uint8_t *p_data = MESH_TB_BUF_DATA(p_buf_set);
        // Get pointer to environment
        mm_route_buf_env_t *p_buf_env = (mm_route_buf_env_t *)&p_buf_set->env;

        // Prepare environment
        p_buf_env->app_key_lid = 6; // TODO [LT]
        p_buf_env->u_addr.dst = dst;
        p_buf_env->info = 0;
        p_buf_env->mdl_lid = p_env->mdl_lid;
        p_buf_env->opcode = GETB(set_info, MM_SET_INFO_ACK)
                            ? MM_MSG_GEN_LOCG_SET : MM_MSG_GEN_LOCG_SET_UNACK;

        // Fill the message
        co_write32p(p_data + MM_GEN_LOC_SET_GLOB_LAT_POS, latitude);
        co_write32p(p_data + MM_GEN_LOC_SET_GLOB_LONG_POS, longitude);
        co_write16p(p_data + MM_GEN_LOC_SET_GLOB_ALT_POS, altitude);

        // Send the message
        mm_route_send(p_buf_set);
    }
    else
    {
        MESH_MODEL_PRINT_WARN("%s, buffer alloc fail.\n", __func__);
    }

    return (status);
}

uint16_t mm_genc_loc_set_local(mm_tb_state_mdl_env_t *p_env, uint16_t dst, uint8_t set_info,
                               int16_t north, int16_t east, int16_t altitude, uint8_t floor,
                               uint16_t uncertainty)
{
    // Pointer to the buffer that will contain the message
    mesh_tb_buf_t *p_buf_set;
    // Allocate a new buffer for the publication
    uint8_t status = mm_route_buf_alloc(&p_buf_set, MM_GEN_LOC_SET_LOC_LEN);

    if (status == MESH_ERR_NO_ERROR)
    {
        // Get pointer to data
        uint8_t *p_data = MESH_TB_BUF_DATA(p_buf_set);
        // Get pointer to environment
        mm_route_buf_env_t *p_buf_env = (mm_route_buf_env_t *)&p_buf_set->env;

        // Prepare environment
        p_buf_env->app_key_lid = 6; // TODO [LT]
        p_buf_env->u_addr.dst = dst;
        p_buf_env->info = 0;
        p_buf_env->mdl_lid = p_env->mdl_lid;
        p_buf_env->opcode = GETB(set_info, MM_SET_INFO_ACK)
                            ? MM_MSG_GEN_LOCG_SET : MM_MSG_GEN_LOCG_SET_UNACK;

        // Fill the message
        co_write16p(p_data + MM_GEN_LOC_SET_LOC_NORTH_POS, north);
        co_write16p(p_data + MM_GEN_LOC_SET_LOC_EAST_POS, east);
        co_write16p(p_data + MM_GEN_LOC_SET_LOC_ALT_POS, altitude);
        *(p_data + MM_GEN_LOC_SET_LOC_FLOOR_POS) = floor;
        co_write16p(p_data + MM_GEN_LOC_SET_LOC_UNCERT_POS, uncertainty);

        // Send the message
        mm_route_send(p_buf_set);
    }
    else
    {
        MESH_MODEL_PRINT_WARN("%s, buffer alloc fail.\n", __func__);
    }

    return (status);
}
#endif //(BLE_MESH_MDL_GENC_LOC)
/// @} end of group
