/**
 ****************************************************************************************
 *
 * @file mm_genc_dtt.c
 *
 * @brief Mesh Model Generic Default Transition Time Client Module
 *
 * Copyright (C) RivieraWaves 2017-2019
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup MM_GENC_DTT
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "mm_genc_int.h"      // Mesh Model Generic Server Module Internal Definitions

#if (BLE_MESH_MDL_GENC_DTT)
/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

/// Structure for Generic Default Transition Time Client model environment
typedef struct mm_genc_dtt_env
{
    /// Basic model environment - Must be first element in the structure - DO NOT MOVE
    mm_tb_state_mdl_env_t env;
} mm_genc_dtt_env_t;

/*
 * INTERNAL CALLBACK FUNCTIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Inform Generic Default Transition Time Client model about reception of a message
 *
 * @param[in] p_env         Pointer to the environment allocated for the model
 * @param[in] p_buf         Pointer to buffer containing the received message
 * @param[in] p_route_env   Pointer to routing environment containing information about the
 * received message
 ****************************************************************************************
 */
__STATIC void mm_genc_dtt_cb_rx(mm_tb_state_mdl_env_t *p_env, mesh_tb_buf_t *p_buf,
                                mm_route_buf_env_t *p_route_env)
{
    if (p_route_env->opcode == MM_MSG_GEN_DTT_STATUS)
    {
        // Get pointer to data
        uint8_t *p_data = MESH_TB_BUF_DATA(p_buf);
        // Generic Default Transition Time state value
        uint8_t dft_trans_time = *(p_data + MM_GEN_DDT_STATUS_DDT_POS);

        // Inform the application about the received Generic Default Transition Time state value
        mm_api_send_cli_state_ind(p_route_env->u_addr.src, MM_STATE_GEN_DTT, dft_trans_time, 0, 0);
    }
    else
    {
        MESH_MODEL_PRINT_WARN("%s, Invalid opcode (0x%x).\n", __func__, p_route_env->opcode);
    }
}

/**
 ****************************************************************************************
 * @brief Inform Generic Default Transition Time Client model about a received opcode in order
 * to check if the model is authorized to handle the associated message
 *
 * @param[in] p_env         Pointer to the environment allocated for the model
 * @param[in] opcode        Opcode value to be checked
 *
 * @return An error status (@see enum mesh_err)
 ****************************************************************************************
 */
__STATIC uint16_t mm_genc_dtt_cb_opcode_check(mm_tb_state_mdl_env_t *p_env, uint32_t opcode)
{
    uint16_t status;

    if (opcode == MM_MSG_GEN_DTT_STATUS)
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
 * @brief Send a Generic Default Transition Time Get message to a given node's element
 *
 * @param[in] p_env         Pointer to the environment allocated for the model
 * @param[in] dst           Address of node's element to which message will be sent
 *
 * @return An error status (@see enum mesh_err)
 ****************************************************************************************
 */
__STATIC uint16_t mm_genc_dtt_cb_get(mm_tb_state_mdl_env_t *p_env, uint16_t dst, uint16_t get_info)
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
        p_buf_env->opcode = MM_MSG_GEN_DTT_GET;

        // Send the message
        mm_route_send(p_buf_get);
    }
    else
    {
        MESH_MODEL_PRINT_WARN("%s, buffer alloc fail.\n", __func__);
    }

    return (status);
}

/**
 ****************************************************************************************
 * @brief Set Generic Default Transition Time state value on a given node's element by
 * sending either a Generic Default Transition Time Set or a Generic Default Transition
 * Time Set Unacknowledged message
 *
 * @param[in] p_env             Pointer to the environment allocated for the model
 * @param[in] dst               Address of node's element to which message will be sent
 * @param[in] state_1           Generic Default Transition Time state value (uint8_t)
 * @param[in] state_2           N\A
 * @param[in] set_info          Set information (@see enum mm_set_info)
 *
 * @return An error status (@see enum mesh_err)
 ****************************************************************************************
 */
__STATIC uint16_t mm_genc_dtt_cb_set(mm_tb_state_mdl_env_t *p_env, uint16_t dst,
                                     uint32_t state_1, uint32_t state_2,
                                     uint16_t set_info)
{
    // Pointer to the buffer that will contain the message
    mesh_tb_buf_t *p_buf_set;
    // Allocate a new buffer for the publication
    uint8_t status = mm_route_buf_alloc(&p_buf_set, MM_GEN_DTT_SET_LEN);

    if (status == MESH_ERR_NO_ERROR)
    {
        // Get pointer to data
        uint8_t *p_data = MESH_TB_BUF_DATA(p_buf_set);
        // Get pointer to environment
        mm_route_buf_env_t *p_buf_env = (mm_route_buf_env_t *)&p_buf_set->env;
        // Generic Default Transition Time state value
        uint8_t dft_trans_time = state_1;

        // Prepare environment
        p_buf_env->app_key_lid = 6; // TODO [LT]
        p_buf_env->u_addr.dst = dst;
        p_buf_env->info = 0;
        p_buf_env->mdl_lid = p_env->mdl_lid;
        p_buf_env->opcode = GETB(set_info, MM_SET_INFO_ACK) ? MM_MSG_GEN_DTT_SET : MM_MSG_GEN_DTT_SET_UNACK;;

        // Fill the message
        *(p_data + MM_GEN_DDT_STATUS_DDT_POS) = dft_trans_time;

        // Send the message
        mm_route_send(p_buf_set);
    }
    else
    {
        MESH_MODEL_PRINT_WARN("%s, buffer alloc fail.\n", __func__);
    }

    return (status);
}

/*
 * GLOBAL FUNCTIONS
 ****************************************************************************************
 */

uint16_t mm_genc_dtt_register(void)
{
    // Model local index
    m_lid_t mdl_lid;
    // Register the model
    uint16_t status = m_api_register_model(MM_ID_GENC_DTT, 0, M_MDL_CONFIG_PUBLI_AUTH_BIT,
                                           &mm_route_cb, &mdl_lid);

    if (status == MESH_ERR_NO_ERROR)
    {
        // Inform the Model State Manager about registered model
        status = mm_tb_state_register(0, MM_ID_GENC_DTT, mdl_lid,
                                      MM_TB_STATE_ROLE_CLI | MM_TB_STATE_CFG_CB_BIT,
                                      sizeof(mm_genc_dtt_env_t));

        if (status == MESH_ERR_NO_ERROR)
        {
            // Get allocated environment
            mm_genc_dtt_env_t *p_env_dtt = (mm_genc_dtt_env_t *)mm_tb_state_get_env(mdl_lid);
            // Get client-specific callback functions
            mm_cli_cb_t *p_cb_cli = p_env_dtt->env.cb.u.p_cb_cli;

            // Set internal callback functions
            p_env_dtt->env.cb.cb_rx = mm_genc_dtt_cb_rx;
            p_env_dtt->env.cb.cb_opcode_check = mm_genc_dtt_cb_opcode_check;
            p_cb_cli->cb_get = mm_genc_dtt_cb_get;
            p_cb_cli->cb_set = mm_genc_dtt_cb_set;

            // Inform application about registered model
            mm_api_send_register_ind(MM_ID_GENC_DTT, 0, mdl_lid);
        }
        else
        {
            MESH_MODEL_PRINT_WARN("%s, Model State fail.\n", __func__);
        }
    }
    else
    {
        MESH_MODEL_PRINT_WARN("%s, Register the model fail.\n", __func__);
    }

    return (status);
}
#endif //(BLE_MESH_MDL_GENC_DTT)
/// @} end of group
