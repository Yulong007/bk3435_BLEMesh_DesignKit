/**
 ****************************************************************************************
 *
 * @file mm_lightc_ctl.c
 *
 * @brief Mesh Model Light CTL Client Module
 *
 * Copyright (C) RivieraWaves 2017-2019
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup MM_LIGHTC_CTL
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "mm_lightc_int.h"      // Mesh Model Light Client Module Internal Definitions

#if (BLE_MESH_MDL_LIGHTC_CTL)
/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

/// Structure for Light CTL Client model environment
typedef struct mm_lightc_ctl_env
{
    /// Basic model environment - Must be first element in the structure - DO NOT MOVE
    mm_tb_state_mdl_env_t env;
} mm_lightc_ctl_env_t;

/*
 * MESSAGE HANDLERS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Handler for Light CTL Status message
 *
 * @param[in] p_buf     Pointer to buffer containing the received message
 * @param[in] src       Source address of the message
 ****************************************************************************************
 */
__STATIC void mm_lightc_ctl_handler_status(mesh_tb_buf_t *p_buf, uint16_t src)
{
    // Get pointer to data
    uint8_t *p_data = MESH_TB_BUF_DATA(p_buf);
    // Present CTL Lightness value
    uint16_t lightness = co_read16p(p_data + MM_LIGHT_CTL_STATUS_LIGHTNESS_POS);
    // Present CTL Temperature value
    uint16_t temp = co_read16p(p_data + MM_LIGHT_CTL_STATUS_TEMP_POS);
    // Target CTL Lightness value
    uint16_t lightness_tgt;
    // Target CTL Temperature value
    uint16_t temp_tgt;
    // Remaining time in milliseconds
    uint32_t rem_time_ms;

    // Check if optional parameters are provided
    if (p_buf->data_len == MM_LIGHT_CTL_STATUS_LEN)
    {
        lightness_tgt = co_read16p(p_data + MM_LIGHT_CTL_STATUS_TGT_LIGHTNESS_POS);
        temp_tgt = co_read16p(p_data + MM_LIGHT_CTL_STATUS_TGT_TEMP_POS);
        rem_time_ms = mm_tb_get_trans_time_ms(*(p_data + MM_LIGHT_CTL_STATUS_REM_TIME_POS));
    }
    else
    {
        lightness_tgt = 0;
        temp_tgt = 0;
        rem_time_ms = 0;
    }

    // Inform the application about the received Light CTL Lightness state value
    mm_api_send_cli_state_ind(src, MM_STATE_LIGHT_CTL_LN, lightness, lightness_tgt, rem_time_ms);
    // Inform the application about the received Light CTL Temperature state value
    mm_api_send_cli_state_ind(src, MM_STATE_LIGHT_CTL_TEMP, temp, temp_tgt, rem_time_ms);
}

/**
 ****************************************************************************************
 * @brief Handler for Light CTL Temperature Status message
 *
 * @param[in] p_buf     Pointer to buffer containing the received message
 * @param[in] src       Source address of the message
 ****************************************************************************************
 */
__STATIC void mm_lightc_ctl_handler_status_temp(mesh_tb_buf_t *p_buf, uint16_t src)
{
    // Get pointer to data
    uint8_t *p_data = MESH_TB_BUF_DATA(p_buf);
    // Present CTL Delta UV value
    int16_t delta_uv = (int16_t)co_read16p(p_data + MM_LIGHT_CTL_TEMP_STATUS_DELTA_UV_POS);
    // Present CTL Temperature value
    uint16_t temp = co_read16p(p_data + MM_LIGHT_CTL_TEMP_STATUS_TEMP_POS);
    // Target CTL Delta UV value
    int16_t delta_uv_tgt;
    // Target CTL Temperature value
    uint16_t temp_tgt;
    // Remaining time in milliseconds
    uint32_t rem_time_ms;

    // Check if optional parameters are provided
    if (p_buf->data_len == MM_LIGHT_CTL_TEMP_STATUS_LEN)
    {
        delta_uv_tgt = (int16_t)co_read16p(p_data + MM_LIGHT_CTL_TEMP_STATUS_TGT_DELTA_UV_POS);
        temp_tgt = co_read16p(p_data + MM_LIGHT_CTL_TEMP_STATUS_TGT_TEMP_POS);
        rem_time_ms = mm_tb_get_trans_time_ms(*(p_data + MM_LIGHT_CTL_TEMP_STATUS_REM_TIME_POS));
    }
    else
    {
        delta_uv_tgt = 0;
        temp_tgt = 0;
        rem_time_ms = 0;
    }

    // Inform the application about the received Light CTL Temperature state value
    mm_api_send_cli_state_ind(src, MM_STATE_LIGHT_CTL_TEMP, temp, temp_tgt, rem_time_ms);
    // Inform the application about the received Light CTL Delta UV state value
    mm_api_send_cli_state_ind(src, MM_STATE_LIGHT_CTL_DELTA_UV, delta_uv, delta_uv_tgt, rem_time_ms);
}

/**
 ****************************************************************************************
 * @brief Handler for Light CTL Default Status message
 *
 * @param[in] p_buf     Pointer to buffer containing the received message
 * @param[in] src       Source address of the message
 ****************************************************************************************
 */
__STATIC void mm_lightc_ctl_handler_status_dflt(mesh_tb_buf_t *p_buf, uint16_t src)
{
    // Get pointer to data
    uint8_t *p_data = MESH_TB_BUF_DATA(p_buf);
    // Light Lightness Default state value
    uint16_t lightness_dflt = co_read16p(p_data + MM_LIGHT_CTL_DFLT_STATUS_LIGHTNESS_POS);
    // Light CTL Temperature Default state value
    uint16_t temp_dflt = co_read16p(p_data + MM_LIGHT_CTL_DFLT_STATUS_TEMP_POS);
    // Light CTL Delta UV Default state value
    int16_t delta_uv_dflt = (int16_t)co_read16p(p_data + MM_LIGHT_CTL_DFLT_STATUS_DELTA_UV_POS);

    // Inform the application about the received Light Lightness Default state value
    mm_api_send_cli_state_ind(src, MM_STATE_LIGHT_LN_DFLT, lightness_dflt, 0, 0);
    // Inform the application about the received Light Lightness Default state value
    mm_api_send_cli_state_ind(src, MM_STATE_LIGHT_CTL_TEMP_DFLT, temp_dflt, 0, 0);
    // Inform the application about the received Light Lightness Default state value
    mm_api_send_cli_state_ind(src, MM_STATE_LIGHT_CTL_DELTA_UV_DFLT, delta_uv_dflt, 0, 0);
}

/**
 ****************************************************************************************
 * @brief Handler for Light CTL Temperature Range Status message
 *
 * @param[in] p_buf     Pointer to buffer containing the received message
 * @param[in] src       Source address of the message
 ****************************************************************************************
 */
__STATIC void mm_lightc_ctl_handler_status_temp_range(mesh_tb_buf_t *p_buf, uint16_t src)
{
    // Get pointer to data
    uint8_t *p_data = MESH_TB_BUF_DATA(p_buf);
    // Light CTL Temperature Range state value (minimum)
    uint16_t temp_min = co_read16p(p_data + MM_LIGHT_CTL_TEMP_RANGE_STATUS_MIN_POS);
    // Light CTL Temperature Range state value (maximum)
    uint16_t temp_max = co_read16p(p_data + MM_LIGHT_CTL_TEMP_RANGE_STATUS_MAX_POS);

    // Inform the application about the received Light Lightness Range state value
    mm_api_send_cli_state_ind(src, MM_STATE_LIGHT_CTL_TEMP_RANGE, temp_min, temp_max, 0);
}

/*
 * INTERNAL CALLBACK FUNCTIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Inform Light CTL Client model about reception of a message
 *
 * @param[in] p_env         Pointer to the environment allocated for the model
 * @param[in] p_buf         Pointer to buffer containing the received message
 * @param[in] p_route_env   Pointer to routing environment containing information about the
 * received message
 ****************************************************************************************
 */
__STATIC void mm_lightc_ctl_cb_rx(mm_tb_state_mdl_env_t *p_env, mesh_tb_buf_t *p_buf,
                                  mm_route_buf_env_t *p_route_env)
{
    // Call the appropriate handler for the received message
    switch (p_route_env->opcode)
    {
        case (MM_MSG_LIGHT_CTL_STATUS):
        {
            mm_lightc_ctl_handler_status(p_buf, p_route_env->u_addr.src);
        } break;

        case (MM_MSG_LIGHT_CTL_TEMP_STATUS):
        {
            mm_lightc_ctl_handler_status_temp(p_buf, p_route_env->u_addr.src);
        } break;

        case (MM_MSG_LIGHT_CTL_TEMP_RANGE_STATUS):
        {
            mm_lightc_ctl_handler_status_temp_range(p_buf, p_route_env->u_addr.src);
        } break;

        case (MM_MSG_LIGHT_CTL_DFLT_STATUS):
        {
            mm_lightc_ctl_handler_status_dflt(p_buf, p_route_env->u_addr.src);
        } break;

        default:
        {
        } break;
    }
}

/**
 ****************************************************************************************
 * @brief Inform Light CTL Client model about a received opcode in order
 * to check if the model is authorized to handle the associated message
 *
 * @param[in] p_env         Pointer to the environment allocated for the model
 * @param[in] opcode        Opcode value to be checked
 *
 * @return An error status (@see enum mesh_err)
 ****************************************************************************************
 */
__STATIC uint16_t mm_lightc_ctl_cb_opcode_check(mm_tb_state_mdl_env_t *p_env, uint32_t opcode)
{
    uint16_t status;

    if ((opcode == MM_MSG_LIGHT_CTL_STATUS)
            || (opcode == MM_MSG_LIGHT_CTL_TEMP_STATUS)
            || (opcode == MM_MSG_LIGHT_CTL_TEMP_RANGE_STATUS)
            || (opcode == MM_MSG_LIGHT_CTL_DFLT_STATUS))
    {
        status = MESH_ERR_NO_ERROR;
    }
    else
    {
        MESH_MODEL_PRINT_INFO("%s, Invalid opcode.\n", __func__);
        status = MESH_ERR_MDL_INVALID_OPCODE;
    }

    return (status);
}

/**
 ****************************************************************************************
 * @brief Send Light CTL Get or Light CTL Temperature Get or Light CTL Temperature Range Get
 * or Light CTL Default Get message
 *
 * @param[in] p_env         Pointer to the environment allocated for the model
 * @param[in] dst           Address of node's element from which state value must be retrieved
 * @param[in] get_option    Get option
 *
 * @return An error status (@see enum mesh_err)
 ****************************************************************************************
 */
__STATIC uint16_t mm_lightc_ctl_cb_get(mm_tb_state_mdl_env_t *p_env, uint16_t dst, uint16_t get_info)
{
    // Status
    uint16_t status;
    // Get type
    uint8_t get_type = get_info;

    // Check option value
    if (get_type <= MM_GET_TYPE_LIGHT_CTL_MAX)
    {
        // Pointer to the buffer that will contain the message
        mesh_tb_buf_t *p_buf_get;

        // Allocate a new buffer for the message
        status = mm_route_buf_alloc(&p_buf_get, 0);

        if (status == MESH_ERR_NO_ERROR)
        {
            // Get pointer to environment
            mm_route_buf_env_t *p_buf_env = (mm_route_buf_env_t *)&p_buf_get->env;

            // Prepare environment
            p_buf_env->app_key_lid = 6; // TODO [LT]
            p_buf_env->u_addr.dst = dst;
            p_buf_env->info = 0;
            p_buf_env->mdl_lid = p_env->mdl_lid;

            if (get_type == MM_GET_TYPE_LIGHT_CTL)
            {
                p_buf_env->opcode = MM_MSG_LIGHT_CTL_GET;
            }
            else if (get_type == MM_GET_TYPE_LIGHT_CTL_TEMP)
            {
                p_buf_env->opcode = MM_MSG_LIGHT_CTL_TEMP_GET;
            }
            else if (get_type == MM_GET_TYPE_LIGHT_CTL_TEMP_RANGE)
            {
                p_buf_env->opcode = MM_MSG_LIGHT_CTL_TEMP_RANGE_GET;
            }
            else     // (option == MM_GET_TYPE_LIGHT_CTL_DFLT)
            {
                p_buf_env->opcode = MM_MSG_LIGHT_CTL_DFLT_GET;
            }

            // Send the message
            mm_route_send(p_buf_get);
        }
        else
        {
            MESH_MODEL_PRINT_WARN("%s, Buffer alloc fail.\n", __func__);
        }
    }
    else
    {
        MESH_MODEL_PRINT_WARN("%s, Invalid parameters.\n", __func__);
        status = MESH_ERR_INVALID_PARAM;
    }

    return (status);
}

/**
 ****************************************************************************************
 * @brief Send Light CTL Set or Light CTL Set Unacknowledged message in order to set:
 *      - Light CTL Lightness state value
 *      - Light CTL Temperature state value
 *      - Light CTL Delta UV state value
 * or Light CTL Temperature Set or Light CTL Temperature Set Unacknowledged message in order
 * to set
 *      - Light CTL Temperature state value
 *      - Light CTL Delta UV state value
 *
 * @param[in] p_env             Pointer to the environment allocated for the model
 * @param[in] dst               Address of node's element on which state value must be set
 * @param[in] state_1           State value 1
 *      - Light CTL Lightness state value (uint16_t) (Bit 0-15) | Light CTL Temperature
 *      state value (uint16_t) (Bit 16-31) if Light CTL Set message
 * or
 *      - Light CTL Temperature state value (uint16_t) if Light CTL Temperature Set message
 * @param[in] state_2           State value 2
 *      - Light CTL Delta UV state value (int16_t)
 * @param[in] trans_time_ms     Transition time in milliseconds
 * @param[in] delay_ms          Delay in milliseconds
 * @param[in] trans_info        Transition information (@see enum mm_trans_info and
 * @see enum mm_trans_type_light_ctl)
 *
 * @return An error status (@see enum mesh_err)
 ****************************************************************************************
 */
__STATIC uint16_t mm_lightc_ctl_cb_transition(mm_tb_state_mdl_env_t *p_env, uint16_t dst,
        uint32_t state_1, uint32_t state_2,
        uint32_t trans_time_ms, uint16_t delay_ms,
        uint16_t trans_info)
{
    // Status
    uint16_t status;
    // Transition type
    uint8_t trans_type = GETF(trans_info, MM_TRANS_INFO_TYPE);

    if (trans_type <= MM_TRANS_TYPE_LIGHT_CTL_MAX)
    {
        // Long message or not
        bool long_set = (GETB(trans_info, MM_TRANS_INFO_LONG) || trans_time_ms || delay_ms);
        // Pointer to the buffer that will contain the message
        mesh_tb_buf_t *p_buf_set;
        // Message length
        uint8_t length;

        if (trans_type == MM_TRANS_TYPE_LIGHT_CTL_TEMP)
        {
            length = (long_set) ? MM_LIGHT_CTL_TEMP_SET_LEN : MM_LIGHT_CTL_TEMP_SET_MIN_LEN;
        }
        else     // (trans_type == MM_TRANS_TYPE_LIGHT_CTL)
        {
            length = (long_set) ? MM_LIGHT_CTL_SET_LEN : MM_LIGHT_CTL_SET_MIN_LEN;
        }

        // Allocate a new buffer
        status = mm_route_buf_alloc(&p_buf_set, length);

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

            // Set opcode and start to fill the message
            if (trans_type == MM_TRANS_TYPE_LIGHT_CTL_TEMP)
            {
                p_buf_env->opcode = (GETB(trans_info, MM_TRANS_INFO_ACK))
                                    ? MM_MSG_LIGHT_CTL_TEMP_SET : MM_MSG_LIGHT_CTL_TEMP_SET_UNACK;

                co_write16p(p_data + MM_LIGHT_CTL_TEMP_SET_TEMP_POS, state_1);
                co_write16p(p_data + MM_LIGHT_CTL_TEMP_SET_DELTA_UV_POS, (int16_t)state_2);

                p_data += MM_LIGHT_CTL_TEMP_SET_TID_POS;
            }
            else     // (trans_type == MM_TRANS_TYPE_LIGHT_CTL)
            {
                p_buf_env->opcode = (GETB(trans_info, MM_TRANS_INFO_ACK))
                                    ? MM_MSG_LIGHT_CTL_SET : MM_MSG_LIGHT_CTL_SET_UNACK;

                co_write16p(p_data + MM_LIGHT_CTL_SET_LIGHTNESS_POS, state_1);
                co_write16p(p_data + MM_LIGHT_CTL_SET_TEMP_POS, (state_1 >> 16));
                co_write16p(p_data + MM_LIGHT_CTL_SET_DELTA_UV_POS, (int16_t)state_2);

                p_data += MM_LIGHT_CTL_SET_TID_POS;
            }

            // Fill the message
            *(p_data++) = GETF(trans_info, MM_TRANS_INFO_TID);

            if (long_set)
            {
                // Set transition time and delay values
                *(p_data++) = mm_tb_get_trans_time(trans_time_ms);
                *(p_data++) = delay_ms / 5;
            }

            // Send the message
            mm_route_send(p_buf_set);
        }
        else
        {
            MESH_MODEL_PRINT_WARN("%s, Buffer alloc fail.\n", __func__);
        }
    }
    else
    {
        MESH_MODEL_PRINT_WARN("%s, Invalid parameter.\n", __func__);
        status = MESH_ERR_INVALID_PARAM;
    }

    return (status);
}

/**
 ****************************************************************************************
 * @brief Send Light CTL Default Set or Light CTL Default Set Unacknowledged message in
 * order to set:
 *      - Light Lightness Default state value
 *      - Light CTL Temperature Default state value
 *      - Light CTL Delta UV Default state value
 * or Light CTL Temperature Range Set or Light CTL Temperature Range Set Unacknowledged message
 * in order to set Light CTL Temperature Range state value
 *
 * @param[in] p_env             Pointer to the environment allocated for the model
 * @param[in] dst               Address of node's element on which state value must be set
 * @param[in] state_1           State value 1
 *      - Light Lightness Default state value (uint16_t) (Bit 0-15) | Light CTL Temperature
 *      Default state value (uint16_t) (Bit 16-31) if Light CTL Default Set message
 * or
 *      - Light CTL Temperature Range Min state value (uint16_t) if Light CTL Temperature Range
 *      Set message
 * @param[in] state_2           State value 2
 *      - Light CTL Delta UV Default state value (int16_t) if Light CTL Default Set message
 * or
 *      - Light CTL Temperature Range Max state value (uint16_t) if Light CTL Temperature Range
 *      Set message
 * @param[in] set_info          Set information (@see enum mm_set_info and
 * @see enum mm_set_type_light_ctl for set type value)
 *
 * @return An error status (@see enum mesh_err)
 ****************************************************************************************
 */
__STATIC uint16_t mm_lightc_ctl_cb_set(mm_tb_state_mdl_env_t *p_env, uint16_t dst,
                                       uint32_t state_1, uint32_t state_2,
                                       uint16_t set_info)
{
    // Set type
    uint8_t set_type = GETF(set_info, MM_SET_INFO_TYPE);
    // Status
    uint16_t status;

    // Check provided set type
    if (set_type <= MM_SET_TYPE_LIGHT_CTL_MAX)
    {
        // Pointer to the buffer that will contain the message
        mesh_tb_buf_t *p_buf_set;
        // Opcode
        uint32_t opcode;
        // Length
        uint8_t length;

        if (set_type == MM_SET_TYPE_LIGHT_CTL_TEMP_RANGE)
        {
            length = MM_LIGHT_CTL_TEMP_RANGE_SET_LEN;
            opcode = GETB(set_info, MM_SET_INFO_ACK)
                     ? MM_MSG_LIGHT_CTL_TEMP_RANGE_SET : MM_MSG_LIGHT_CTL_TEMP_RANGE_SET_UNACK;
        }
        else     // (set_type == MM_SET_TYPE_LIGHT_CTL_DFLT)
        {
            length = MM_LIGHT_CTL_DFLT_SET_LEN;
            opcode = GETB(set_info, MM_SET_INFO_ACK)
                     ? MM_MSG_LIGHT_CTL_DFLT_SET : MM_MSG_LIGHT_CTL_DFLT_SET_UNACK;
        }

        // Allocate a new buffer for the publication
        status = mm_route_buf_alloc(&p_buf_set, length);

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
            p_buf_env->opcode = opcode;

            // Fill the message
            if (set_type == MM_SET_TYPE_LIGHT_CTL_TEMP_RANGE)
            {
                co_write16p(p_data + MM_LIGHT_CTL_TEMP_RANGE_SET_MIN_POS, state_1);
                co_write16p(p_data + MM_LIGHT_CTL_TEMP_RANGE_SET_MAX_POS, state_2);
            }
            else     // (set_type == MM_SET_TYPE_LIGHT_CTL_DFLT)
            {
                co_write16p(p_data + MM_LIGHT_CTL_DFLT_SET_LIGHTNESS_POS, state_1);
                co_write16p(p_data + MM_LIGHT_CTL_DFLT_SET_TEMP_POS, (state_1 >> 16));
                co_write16p(p_data + MM_LIGHT_CTL_DFLT_SET_DELTA_UV_POS, (int16_t)state_2);
            }

            // Send the message
            mm_route_send(p_buf_set);
        }
        else
        {
            MESH_MODEL_PRINT_WARN("%s, Buffer alloc fail.\n", __func__);
        }
    }
    else
    {
        MESH_MODEL_PRINT_WARN("%s, Invalid Parameters.\n", __func__);
        status = MESH_ERR_INVALID_PARAM;
    }

    return (status);
}

/*
 * GLOBAL FUNCTIONS
 ****************************************************************************************
 */

uint16_t mm_lightc_ctl_register(void)
{
    // Model local index
    m_lid_t mdl_lid;
    // Register the model
    uint16_t status = m_api_register_model(MM_ID_LIGHTC_CTL, 0, M_MDL_CONFIG_PUBLI_AUTH_BIT,
                                           &mm_route_cb, &mdl_lid);

    if (status == MESH_ERR_NO_ERROR)
    {
        // Inform the Model State Manager about registered model
        status = mm_tb_state_register(0, MM_ID_LIGHTC_CTL, mdl_lid,
                                      MM_TB_STATE_ROLE_CLI | MM_TB_STATE_CFG_CB_BIT,
                                      sizeof(mm_lightc_ctl_env_t));

        if (status == MESH_ERR_NO_ERROR)
        {
            // Get allocated environment
            mm_lightc_ctl_env_t *p_env_ctl = (mm_lightc_ctl_env_t *)mm_tb_state_get_env(mdl_lid);
            // Get client-specific callback functions
            mm_cli_cb_t *p_cb_cli = p_env_ctl->env.cb.u.p_cb_cli;

            // Set internal callback functions
            p_env_ctl->env.cb.cb_rx = mm_lightc_ctl_cb_rx;
            p_env_ctl->env.cb.cb_opcode_check = mm_lightc_ctl_cb_opcode_check;
            p_cb_cli->cb_get = mm_lightc_ctl_cb_get;
            p_cb_cli->cb_set = mm_lightc_ctl_cb_set;
            p_cb_cli->cb_transition = mm_lightc_ctl_cb_transition;

            // Inform application about registered model
            mm_api_send_register_ind(MM_ID_LIGHTC_CTL, 0, mdl_lid);
        }
        else
        {
            MESH_MODEL_PRINT_WARN("%s, Model State register fail.\n", __func__);
        }
    }
    else
    {
        MESH_MODEL_PRINT_WARN("%s, Model register fail.\n", __func__);
    }

    return (status);
}
#endif //(BLE_MESH_MDL_LIGHTC_CTL)
/// @} end of group
