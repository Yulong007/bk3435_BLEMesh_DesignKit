/**
 ****************************************************************************************
 *
 * @file mm_genc_lvl.c
 *
 * @brief Mesh Model Generic Level Client Module
 *
 * Copyright (C) RivieraWaves 2017-2019
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup MM_GENC_LVL
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "mm_genc_int.h"      // Mesh Model Generic Server Module Internal Definitions

#if (BLE_MESH_MDL_GENC_LVL)
/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

/// Structure for Generic Level Client model environment
typedef struct mm_genc_lvl_env
{
    /// Basic model environment - Must be first element in the structure - DO NOT MOVE
    mm_tb_state_mdl_env_t env;
} mm_genc_lvl_env_t;

/*
 * LOCAL FUNCTIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Compute length of set message to prepare:
 *          - Generic Level Set/Set Unacknowledged
 *          - Generic Delta Set/Set Unacknowledged
 *          - Generic Move Set/Set Unacknowledged
 *
 * @param[in] trans_type        Transition type (@see enum mm_trans_type)
 * @param[in] set_long          True if optional parameters are present, else false
 *
 * @return Length of the set message
 ****************************************************************************************
 */
__STATIC uint8_t mm_genc_lvl_get_length(uint8_t trans_type, bool set_long)
{
    uint8_t length;

    if (trans_type == MM_TRANS_TYPE_CLASSIC)
    {
        // Generic Level Set/Set Unacknowledged
        length = (set_long) ? MM_GEN_LVL_SET_LEN : MM_GEN_LVL_SET_MIN_LEN;
    }
    else if (trans_type == MM_TRANS_TYPE_DELTA)
    {
        // Generic Delta Set/Set Unacknowledged
        length = (set_long) ? MM_GEN_LVL_SET_DELTA_LEN : MM_GEN_LVL_SET_DELTA_MIN_LEN;
    }
    else     // (trans_type == MM_TRANS_TYPE_MOVE)
    {
        // Generic Move Set/Set Unacknowledged
        length = (set_long) ? MM_GEN_LVL_SET_MOVE_LEN : MM_GEN_LVL_SET_MOVE_MIN_LEN;
    }

    return (length);
}

/*
 * INTERNAL CALLBACK FUNCTIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Inform Generic Level Client model about reception of a message
 *
 * @param[in] p_env         Pointer to the environment allocated for the model
 * @param[in] p_buf         Pointer to buffer containing the received message
 * @param[in] p_route_env   Pointer to routing environment containing information about the
 * received message
 ****************************************************************************************
 */
__STATIC void mm_genc_lvl_cb_rx(mm_tb_state_mdl_env_t *p_env, mesh_tb_buf_t *p_buf,
                                mm_route_buf_env_t *p_route_env)
{
    if (p_route_env->opcode == MM_MSG_GEN_LVL_STATUS)
    {
        // Get pointer to data
        uint8_t *p_data = MESH_TB_BUF_DATA(p_buf);
        // Current state
        uint16_t cur_state = co_read16p(p_data + MM_GEN_LVL_STATUS_LVL_POS);
        // Target state
        uint16_t tgt_state;
        // Remaining time
        uint8_t rem_time;

        // Read optional parameters if presents
        if (p_buf->data_len == MM_GEN_LVL_STATUS_LEN)
        {
            tgt_state = co_read16p(p_data + MM_GEN_LVL_STATUS_TGT_LVL_POS);
            rem_time = *(p_data + MM_GEN_LVL_STATUS_REM_TIME_POS);
        }
        else
        {
            tgt_state = 0;
            rem_time = 0;
        }

        // Inform the application about the received Generic Level state value
        mm_api_send_cli_state_ind(p_route_env->u_addr.src, MM_STATE_GEN_LVL, cur_state,
                                  tgt_state, mm_tb_get_trans_time_ms(rem_time));
    }
}

/**
 ****************************************************************************************
 * @brief Inform Generic Level Client model about a received opcode in order to check if the
 * model is authorized to handle the associated message
 *
 * @param[in] p_env         Pointer to the environment allocated for the model
 * @param[in] opcode        Opcode value to be checked
 *
 * @return An error status (@see enum mesh_err)
 ****************************************************************************************
 */
__STATIC uint16_t mm_genc_lvl_cb_opcode_check(mm_tb_state_mdl_env_t *p_env, uint32_t opcode)
{
    uint16_t status;

    if (opcode == MM_MSG_GEN_LVL_STATUS)
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
 * @brief Send a Generic Level Get message to a given node's element
 *
 * @param[in] p_env         Pointer to the environment allocated for the model
 * @param[in] dst           Address of node's element to which message will be sent
 *
 * @return An error status (@see enum mesh_err)
 ****************************************************************************************
 */
__STATIC uint16_t mm_genc_lvl_cb_get(mm_tb_state_mdl_env_t *p_env, uint16_t dst, uint16_t get_info)
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
        p_buf_env->opcode = MM_MSG_GEN_LVL_GET;

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
 * @brief Initiate transition of Generic Level state value on a given node's element by
 * sending either a Generic Level Set or a Generic Level Set Unacknowledged message
 *
 * @param[in] p_env             Pointer to the environment allocated for the model
 * @param[in] dst               Address of node's element to which message will be sent
 * @param[in] state_1           Target Generic Level state value
 * @param[in] state_2           N\A
 * @param[in] trans_time_ms     Transition time in milliseconds
 * @param[in] delay_ms          Delay in milliseconds
 * @param[in] trans_info        Transition information (@see enum mm_trans_info)
 *
 * @return An error status (@see enum mesh_err)
 ****************************************************************************************
 */
__STATIC uint16_t mm_genc_lvl_cb_transition(mm_tb_state_mdl_env_t *p_env, uint16_t dst,
        uint32_t state_1, uint32_t state_2,
        uint32_t trans_time_ms, uint16_t delay_ms,
        uint16_t trans_info)
{
    // Transition type
    uint8_t trans_type = GETF(trans_info, MM_TRANS_INFO_TYPE);
    // Long message or not
    bool long_set = (GETB(trans_info, MM_TRANS_INFO_LONG) || trans_time_ms || delay_ms);
    // Get message length
    uint8_t length = mm_genc_lvl_get_length(trans_type, long_set);
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

        // Prepare environment
        p_buf_env->app_key_lid = 6; // TODO [LT]
        p_buf_env->u_addr.dst = dst;
        p_buf_env->info = 0;
        p_buf_env->mdl_lid = p_env->mdl_lid;

        if (trans_type == MM_TRANS_TYPE_CLASSIC)
        {
            p_buf_env->opcode = GETB(trans_info, MM_TRANS_INFO_ACK)
                                ? MM_MSG_GEN_LVL_SET : MM_MSG_GEN_LVL_SET_UNACK;

            // Fill the message
            co_write16p(p_data + MM_GEN_LVL_SET_LVL_POS, state_1);
            *(p_data + MM_GEN_LVL_SET_TID_POS) = GETF(trans_info, MM_TRANS_INFO_TID);
            p_data += MM_GEN_LVL_SET_TRANS_TIME_POS;
        }
        else if (trans_type == MM_TRANS_TYPE_DELTA)
        {
            p_buf_env->opcode = GETB(trans_info, MM_TRANS_INFO_ACK)
                                ? MM_MSG_GEN_DELTA_SET : MM_MSG_GEN_DELTA_SET_UNACK;

            // Fill the message
            co_write32p(p_data + MM_GEN_LVL_SET_DELTA_LVL_POS, state_1);
            *(p_data + MM_GEN_LVL_SET_DELTA_TID_POS) = GETF(trans_info, MM_TRANS_INFO_TID);
            p_data += MM_GEN_LVL_SET_DELTA_TRANS_TIME_POS;
        }
        else
        {
            p_buf_env->opcode = GETB(trans_info, MM_TRANS_INFO_ACK)
                                ? MM_MSG_GEN_MOVE_SET : MM_MSG_GEN_MOVE_SET_UNACK;

            // Fill the message
            co_write16p(p_data + MM_GEN_LVL_SET_MOVE_DELTA_LVL_POS, state_1);
            *(p_data + MM_GEN_LVL_SET_MOVE_TID_POS) = GETF(trans_info, MM_TRANS_INFO_TID);
            p_data += MM_GEN_LVL_SET_MOVE_TRANS_TIME_POS;
        }

        if (long_set)
        {
            // Set transition time and delay values
            *p_data = mm_tb_get_trans_time(trans_time_ms);
            *(p_data + 1) = delay_ms / 5;
        }

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

uint16_t mm_genc_lvl_register(void)
{
    // Model local index
    m_lid_t mdl_lid;
    // Register the model
    uint16_t status = m_api_register_model(MM_ID_GENC_LVL, 0, M_MDL_CONFIG_PUBLI_AUTH_BIT,
                                           &mm_route_cb, &mdl_lid);

    if (status == MESH_ERR_NO_ERROR)
    {
        // Inform the Model State Manager about registered model
        status = mm_tb_state_register(0, MM_ID_GENC_LVL, mdl_lid,
                                      MM_TB_STATE_ROLE_CLI | MM_TB_STATE_CFG_CB_BIT,
                                      sizeof(mm_genc_lvl_env_t));

        if (status == MESH_ERR_NO_ERROR)
        {
            // Get allocated environment
            mm_genc_lvl_env_t *p_env_lvl = (mm_genc_lvl_env_t *)mm_tb_state_get_env(mdl_lid);
            // Get client-specific callback functions
            mm_cli_cb_t *p_cb_cli = p_env_lvl->env.cb.u.p_cb_cli;

            // Set internal callback functions
            p_env_lvl->env.cb.cb_rx = mm_genc_lvl_cb_rx;
            p_env_lvl->env.cb.cb_opcode_check = mm_genc_lvl_cb_opcode_check;
            p_cb_cli->cb_get = mm_genc_lvl_cb_get;
            p_cb_cli->cb_transition = mm_genc_lvl_cb_transition;

            // Inform application about registered model
            mm_api_send_register_ind(MM_ID_GENC_LVL, 0, mdl_lid);
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
#endif //(BLE_MESH_MDL_GENC_LVL)
/// @} end of group
