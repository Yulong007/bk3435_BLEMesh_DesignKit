/**
 ****************************************************************************************
 *
 * @file mm_genc_oo.c
 *
 * @brief Mesh Model Generic OnOff Client Module
 *
 * Copyright (C) RivieraWaves 2017-2019
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup MM_GENC_OO
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "mm_genc_int.h"      // Mesh Model Generic Server Module Internal Definitions
#include "m_tb_key.h"


#if (BLE_MESH_MDL_GENC_OO)
/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

/// Structure for Generic OnOff Client model environment
typedef struct mm_genc_oo_env
{
    /// Basic model environment - Must be first element in the structure - DO NOT MOVE
    mm_tb_state_mdl_env_t env;
} mm_genc_oo_env_t;

/*
 * INTERNAL CALLBACK FUNCTIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Inform Generic OnOff Client model about reception of a message
 *
 * @param[in] p_env         Pointer to the environment allocated for the model
 * @param[in] p_buf         Pointer to buffer containing the received message
 * @param[in] p_route_env   Pointer to routing environment containing information about the
 * received message
 ****************************************************************************************
 */
__STATIC void mm_genc_oo_cb_rx(mm_tb_state_mdl_env_t *p_env, mesh_tb_buf_t *p_buf,
                               mm_route_buf_env_t *p_route_env)
{
    if (p_route_env->opcode == MM_MSG_GEN_OO_STATUS)
    {
        // Get pointer to data
        uint8_t *p_data = MESH_TB_BUF_DATA(p_buf);
        // Current state
        uint8_t cur_state = *(p_data + MM_GEN_OO_STATUS_OO_POS);
        // Target state
        uint8_t tgt_state = 0;
        // Remaining time
        uint8_t rem_time = 0;

        // Read optional parameters if presents
        if (p_buf->data_len == MM_GEN_OO_STATUS_LEN)
        {
            tgt_state = *(p_data + MM_GEN_OO_STATUS_TGT_OO_POS);
            rem_time = *(p_data + MM_GEN_OO_STATUS_REM_TIME_POS);
        }

        // Inform the application about the received Generic OnOff state value
        mm_api_send_cli_state_ind(p_route_env->u_addr.src, MM_STATE_GEN_ONOFF, cur_state,
                                  tgt_state, mm_tb_get_trans_time_ms(rem_time));
    }
    else
    {
        MESH_MODEL_PRINT_WARN("%s, Invalid opcode (0x%x).\n", __func__, p_route_env->opcode);
    }
}

/**
 ****************************************************************************************
 * @brief Inform Generic OnOff Client model about a received opcode in order to check if the
 * model is authorized to handle the associated message
 *
 * @param[in] p_env         Pointer to the environment allocated for the model
 * @param[in] opcode        Opcode value to be checked
 *
 * @return An error status (@see enum mesh_err)
 ****************************************************************************************
 */
__STATIC uint16_t mm_genc_oo_cb_opcode_check(mm_tb_state_mdl_env_t *p_env, uint32_t opcode)
{
    uint16_t status;

    if (opcode == MM_MSG_GEN_OO_STATUS)
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
 * @brief Send a Generic OnOff Get message to a given node's element
 *
 * @param[in] p_env         Pointer to the environment allocated for the model
 * @param[in] dst           Address of node's element to which message will be sent
 *
 * @return An error status (@see enum mesh_err)
 ****************************************************************************************
 */
__STATIC uint16_t mm_genc_oo_cb_get(mm_tb_state_mdl_env_t *p_env, uint16_t dst, uint16_t get_info)
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
        p_buf_env->opcode = MM_MSG_GEN_OO_GET;

        // Send the message
        mm_route_send(p_buf_get);
    }
    else
    {
        MESH_MODEL_PRINT_WARN("%s, Alloc buffer fail.\n", __func__);
    }

    return (status);
}

/**
 ****************************************************************************************
 * @brief Initiate transition of Generic OnOff state value on a given node's element by
 * sending either a Generic OnOff Set or a Generic OnOff Set Unacknowledged message
 *
 * @param[in] p_env             Pointer to the environment allocated for the model
 * @param[in] dst               Address of node's element to which message will be sent
 * @param[in] state_1           Target Generic OnOff state value
 * @param[in] state_2           N\A
 * @param[in] trans_time_ms     Transition time in milliseconds
 * @param[in] delay_ms          Delay in milliseconds
 * @param[in] trans_info        Transition information (@see enum mm_trans_info)
 *
 * @return An error status (@see enum mesh_err)
 ****************************************************************************************
 */
__STATIC uint16_t mm_genc_oo_cb_transition(mm_tb_state_mdl_env_t *p_env, uint16_t dst,
        uint32_t state_1, uint32_t state_2,
        uint32_t trans_time_ms, uint16_t delay_ms,
        uint16_t trans_info)
{
    // Get message length
    uint8_t length = (GETB(trans_info, MM_TRANS_INFO_LONG) || trans_time_ms || delay_ms)
                     ? MM_GEN_OO_SET_LEN : MM_GEN_OO_SET_MIN_LEN;
    // Pointer to the buffer that will contain the message
    mesh_tb_buf_t *p_buf_set;
    // Allocate a new buffer
    uint8_t status = mm_route_buf_alloc(&p_buf_set, length);

    if (status == MESH_ERR_NO_ERROR)
    {
        // Get pointer to data
        uint8_t *p_data = MESH_TB_BUF_DATA(p_buf_set);
        // Get pointer to environment
        mm_route_buf_env_t *p_buf_env = (mm_route_buf_env_t *)&p_buf_set->env;
        // Generic OnOff state value
        uint8_t onoff = state_1;

        // Prepare environment
        p_buf_env->app_key_lid = m_tb_model_bind_get_app_key_lid(p_env->mdl_lid); // TODO [LT]
        p_buf_env->u_addr.dst = dst;
        p_buf_env->info = 0;
        p_buf_env->mdl_lid = p_env->mdl_lid;
        p_buf_env->opcode = GETB(trans_info, MM_TRANS_INFO_ACK)
                            ? MM_MSG_GEN_OO_SET : MM_MSG_GEN_OO_SET_UNACK;

        // Fill the message
        *(p_data + MM_GEN_OO_SET_OO_POS) = onoff;
        *(p_data + MM_GEN_OO_SET_TID_POS) = GETF(trans_info, MM_TRANS_INFO_TID);

        if (length == MM_GEN_OO_SET_LEN)
        {
            *(p_data + MM_GEN_OO_SET_TRANS_TIME_POS) = mm_tb_get_trans_time(trans_time_ms);
            *(p_data + MM_GEN_OO_SET_DELAY_POS) = delay_ms / 5;
        }

        // Send the message
        mm_route_send(p_buf_set);
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

uint16_t mm_genc_oo_register(void)
{
    // Model local index
    m_lid_t mdl_lid;
    // Register the model
    uint16_t status = m_api_register_model(MM_ID_GENC_OO, 0, M_MDL_CONFIG_PUBLI_AUTH_BIT,
                                           &mm_route_cb, &mdl_lid);

    if (status == MESH_ERR_NO_ERROR)
    {
        // Inform the Model State Manager about registered model
        status = mm_tb_state_register(0, MM_ID_GENC_OO, mdl_lid,
                                      MM_TB_STATE_ROLE_CLI | MM_TB_STATE_CFG_CB_BIT,
                                      sizeof(mm_genc_oo_env_t));

        if (status == MESH_ERR_NO_ERROR)
        {
            // Get allocated environment
            mm_genc_oo_env_t *p_env_oo = (mm_genc_oo_env_t *)mm_tb_state_get_env(mdl_lid);
            // Get client-specific callback functions
            mm_cli_cb_t *p_cb_cli = p_env_oo->env.cb.u.p_cb_cli;

            // Set internal callback functions
            p_env_oo->env.cb.cb_rx = mm_genc_oo_cb_rx;
            p_env_oo->env.cb.cb_opcode_check = mm_genc_oo_cb_opcode_check;
            p_cb_cli->cb_get = mm_genc_oo_cb_get;
            p_cb_cli->cb_transition = mm_genc_oo_cb_transition;

            // Inform application about registered model
            mm_api_send_register_ind(MM_ID_GENC_OO, 0, mdl_lid);
        }
        else
        {
            MESH_MODEL_PRINT_WARN("%s, Register the Model State fail, status = (0x%x).\n", __func__, status);
        }
    }
    else
    {
        MESH_MODEL_PRINT_WARN("%s, Register the model fail, status = (0x%x).\n", __func__, status);
    }

    return (status);
}
#endif //(BLE_MESH_MDL_GENC_OO)
/// @} end of group
