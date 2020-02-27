/**
 ****************************************************************************************
 *
 * @file mm_genc_prop.c
 *
 * @brief Mesh Model Generic Property Client Module
 *
 * Copyright (C) RivieraWaves 2017-2019
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup MM_GENC_PROP
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "mm_genc_int.h"      // Mesh Model Generic Server Module Internal Definitions

#if (BLE_MESH_MDL_GENC_PROP)
/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

/// Structure for Generic Property Client model environment
typedef struct mm_genc_prop_env
{
    /// Basic model environment - Must be first element in the structure - DO NOT MOVE
    mm_tb_state_mdl_env_t env;
} mm_genc_prop_env_t;

/*
 * INTERNAL CALLBACK FUNCTIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Inform Generic Property Client model about reception of a message
 *
 * @param[in] p_env         Pointer to the environment allocated for the model
 * @param[in] p_buf         Pointer to buffer containing the received message
 * @param[in] p_route_env   Pointer to routing environment containing information about the
 * received message
 ****************************************************************************************
 */
__STATIC void mm_genc_prop_cb_rx(mm_tb_state_mdl_env_t *p_env, mesh_tb_buf_t *p_buf,
                                 mm_route_buf_env_t *p_route_env)
{
    // Get pointer to data
    uint8_t *p_data = MESH_TB_BUF_DATA(p_buf);

    switch (p_route_env->opcode)
    {
        case (MM_MSG_GEN_UPROP_STATUS):
        case (MM_MSG_GEN_APROP_STATUS):
        case (MM_MSG_GEN_MPROP_STATUS):
        {
            // Extract Property ID
            uint16_t prop_id = co_read16p(p_data + MM_GEN_PROP_STATUS_ID_POS);
            // Extract User Access
            uint8_t user_access = *(p_data + MM_GEN_PROP_STATUS_ACCESS_POS);
            // Compute state value length
            uint16_t length = p_buf->data_len - MM_GEN_PROP_STATUS_MIN_LEN;

            // Send the received property state value to the application
            mm_api_send_cli_prop_ind(p_route_env->u_addr.src, prop_id, user_access, length,
                                     p_data + MM_GEN_PROP_STATUS_VALUE_POS);
        } break;

        default:
        {
            // Property type
            uint8_t prop_type;
            // Compute number of Property IDs
            uint8_t nb_prop = p_buf->data_len >> 1;

            switch (p_route_env->opcode)
            {
                case (MM_MSG_GEN_UPROPS_STATUS):
                {
                    prop_type = MM_PROP_TYPE_USER;
                } break;

                case (MM_MSG_GEN_APROPS_STATUS):
                {
                    prop_type = MM_PROP_TYPE_ADMIN;
                } break;

                case (MM_MSG_GEN_MPROPS_STATUS):
                {
                    prop_type = MM_PROP_TYPE_MANUF;
                } break;

                default:   // MM_MSG_GEN_CPROPS_STATUS
                {
                    prop_type = MM_PROP_TYPE_CLI;
                } break;
            }

            // Send the received list of properties to the application
            mm_api_send_cli_prop_list_ind(p_route_env->u_addr.src, prop_type,
                                          nb_prop, (uint16_t *)p_data);
        } break;
    }
}

/**
 ****************************************************************************************
 * @brief Inform Generic Property Client model about a received opcode in order
 * to check if the model is authorized to handle the associated message
 *
 * @param[in] p_env         Pointer to the environment allocated for the model
 * @param[in] opcode        Opcode value to be checked
 *
 * @return An error status (@see enum mesh_err)
 ****************************************************************************************
 */
__STATIC uint16_t mm_genc_prop_cb_opcode_check(mm_tb_state_mdl_env_t *p_env, uint32_t opcode)
{
    uint16_t status;

    if ((opcode == MM_MSG_GEN_UPROPS_STATUS)
            || (opcode == MM_MSG_GEN_UPROP_STATUS)
            || (opcode == MM_MSG_GEN_APROPS_STATUS)
            || (opcode == MM_MSG_GEN_APROP_STATUS)
            || (opcode == MM_MSG_GEN_MPROPS_STATUS)
            || (opcode == MM_MSG_GEN_MPROP_STATUS)
            || (opcode == MM_MSG_GEN_CPROPS_STATUS))
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

/*
 * GLOBAL FUNCTIONS
 ****************************************************************************************
 */

uint16_t mm_genc_prop_register(void)
{
    // Model local index
    m_lid_t mdl_lid;
    // Register the model
    uint16_t status = m_api_register_model(MM_ID_GENC_PROP, 0, M_MDL_CONFIG_PUBLI_AUTH_BIT,
                                           &mm_route_cb, &mdl_lid);

    if (status == MESH_ERR_NO_ERROR)
    {
        // Inform the Model State Manager about registered model
        status = mm_tb_state_register(0, MM_ID_GENC_PROP, mdl_lid, MM_TB_STATE_ROLE_CLI,
                                      sizeof(mm_genc_prop_env_t));

        if (status == MESH_ERR_NO_ERROR)
        {
            // Get allocated environment
            mm_genc_prop_env_t *p_env_prop = (mm_genc_prop_env_t *)mm_tb_state_get_env(mdl_lid);

            // Set internal callback functions
            p_env_prop->env.cb.cb_rx = mm_genc_prop_cb_rx;
            p_env_prop->env.cb.cb_opcode_check = mm_genc_prop_cb_opcode_check;

            // Inform application about registered model
            mm_api_send_register_ind(MM_ID_GENC_PROP, 0, mdl_lid);
        }
        else
        {
            MESH_MODEL_PRINT_WARN("%s, Register the model state fail.\n", __func__);
        }
    }
    else
    {
        MESH_MODEL_PRINT_WARN("%s, Register the model fail.\n", __func__);
    }

    return (status);
}

/**
 ****************************************************************************************
 * @brief Send a Generic User Properties Get or Generic User Property Get or Generic Admin
 * Properties Get or Generic Admin Property Get or Generic Manufacturer Properties Get
 * or Generic Manufacturer Property Get or Generic Client Properties Get message
 * to a given node's element
 *
 * @param[in] p_env         Pointer to the environment allocated for the model
 * @param[in] dst           Address of node's element to which message will be sent
 * @param[in] get_info      Get information
 *
 * @return An error status (@see enum mesh_err)
 ****************************************************************************************
 */
uint16_t mm_genc_prop_get(mm_tb_state_mdl_env_t *p_env, uint16_t dst, uint8_t get_type,
                          uint16_t prop_id)
{
    // Allocate a new buffer for the message
    uint16_t status;

    if (get_type <= MM_GET_TYPE_PROP_MAX)
    {
        // Pointer to the buffer that will contain the message
        mesh_tb_buf_t *p_buf_get;
        // Opcode
        uint32_t opcode = 0;
        // Length
        uint16_t length = 0;

        switch (get_type)
        {
            case (MM_GET_TYPE_PROP_UPROPS):
            {
                opcode = MM_MSG_GEN_UPROPS_GET;
            } break;

            case (MM_GET_TYPE_PROP_UPROP):
            {
                opcode = MM_MSG_GEN_UPROP_GET;
                length = 2;
            } break;

            case (MM_GET_TYPE_PROP_APROPS):
            {
                opcode = MM_MSG_GEN_APROPS_GET;
            } break;

            case (MM_GET_TYPE_PROP_APROP):
            {
                opcode = MM_MSG_GEN_APROP_GET;
                length = 2;
            } break;

            case (MM_GET_TYPE_PROP_MPROPS):
            {
                opcode = MM_MSG_GEN_MPROPS_GET;
            } break;

            case (MM_GET_TYPE_PROP_MPROP):
            {
                opcode = MM_MSG_GEN_MPROP_GET;
                length = 2;
            } break;

            case (MM_GET_TYPE_PROP_CPROPS):
            {
                opcode = MM_MSG_GEN_CPROPS_GET;
            } break;

            default:
            {
            } break;
        }

        status = mm_route_buf_alloc(&p_buf_get, length);

        if (status == MESH_ERR_NO_ERROR)
        {
            // Get pointer to environment
            mm_route_buf_env_t *p_buf_env = (mm_route_buf_env_t *)&p_buf_get->env;
            // Get pointer to data
            uint8_t *p_data = MESH_TB_BUF_DATA(p_buf_get);

            // Prepare environment
            p_buf_env->app_key_lid = 6; // TODO [LT]
            p_buf_env->u_addr.dst = dst;
            p_buf_env->info = 0;
            p_buf_env->mdl_lid = p_env->mdl_lid;
            p_buf_env->opcode = opcode;

            if (length != 0)
            {
                co_write16p(p_data, prop_id);
            }

            // Send the message
            mm_route_send(p_buf_get);
        }
        else
        {
            MESH_MODEL_PRINT_WARN("%s, buffer alloc fail.\n", __func__);
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
 * @brief Send a Generic User Property Set or Generic Admin Property Set or Generic
 * Manufacturer Property Set to a given node's element
 *
 * @param[in] p_env         Pointer to the environment allocated for the model
 * @param[in] dst             Address of node's element to which message will be sent
 * @param[in] get_info      Get information
 *
 * @return An error status (@see enum mesh_err)
 ****************************************************************************************
 */
uint16_t mm_genc_prop_set(mm_tb_state_mdl_env_t *p_env, uint16_t dst, uint8_t set_info,
                          uint16_t prop_id, uint8_t user_access, uint16_t length, const uint8_t *p_val)
{
    // Allocate a new buffer for the message
    uint16_t status;
    // Property type
    uint8_t prop_type = GETF(set_info, MM_SET_INFO_TYPE);

    if (prop_type < MM_PROP_TYPE_CLI)
    {
        // Pointer to the buffer that will contain the message
        mesh_tb_buf_t *p_buf_get;
        // Opcode
        uint32_t opcode = 0;
        // Length
        uint16_t set_length = 0;

        switch (prop_type)
        {
            case (MM_PROP_TYPE_USER):
            {
                opcode = (GETB(set_info, MM_SET_INFO_ACK))
                         ? MM_MSG_GEN_UPROP_SET : MM_MSG_GEN_UPROP_SET_UNACK;
                set_length = MM_GEN_PROP_USER_SET_MIN_LEN + length;
            } break;

            case (MM_PROP_TYPE_ADMIN):
            {
                opcode = (GETB(set_info, MM_SET_INFO_ACK))
                         ? MM_MSG_GEN_APROP_SET : MM_MSG_GEN_APROP_SET_UNACK;
                set_length = MM_GEN_PROP_ADMIN_SET_MIN_LEN + length;
            } break;

            case (MM_PROP_TYPE_MANUF):
            {
                opcode = (GETB(set_info, MM_SET_INFO_ACK))
                         ? MM_MSG_GEN_MPROP_SET : MM_MSG_GEN_MPROP_SET_UNACK;
                set_length = MM_GEN_PROP_MANUF_SET_LEN;
            } break;

            default:
            {
            } break;
        }

        status = mm_route_buf_alloc(&p_buf_get, set_length);

        if (status == MESH_ERR_NO_ERROR)
        {
            // Get pointer to environment
            mm_route_buf_env_t *p_buf_env = (mm_route_buf_env_t *)&p_buf_get->env;
            // Get pointer to data
            uint8_t *p_data = MESH_TB_BUF_DATA(p_buf_get);

            // Prepare environment
            p_buf_env->app_key_lid = 6; // TODO [LT]
            p_buf_env->u_addr.dst = dst;
            p_buf_env->info = 0;
            p_buf_env->mdl_lid = p_env->mdl_lid;
            p_buf_env->opcode = opcode;

            // Write Property ID
            co_write16p(p_data, prop_id);

            switch (prop_type)
            {
                case (MM_PROP_TYPE_USER):
                {
                    memcpy(p_data + MM_GEN_PROP_SET_USER_VAL_POS, p_val, length);
                } break;

                case (MM_PROP_TYPE_ADMIN):
                {
                    *(p_data + MM_GEN_PROP_SET_ADMIN_ACCESS_POS) = user_access;
                    memcpy(p_data + MM_GEN_PROP_SET_ADMIN_VAL_POS, p_val, length);
                } break;

                case (MM_PROP_TYPE_MANUF):
                {
                    *(p_data + MM_GEN_PROP_SET_MANUF_ACCESS_POS) = user_access;
                } break;

                default:
                {
                } break;
            }

            // Send the message
            mm_route_send(p_buf_get);
        }
    }
    else
    {
        status = MESH_ERR_INVALID_PARAM;
    }

    return (status);
}
#endif  //(BLE_MESH_MDL_GENC_PROP)
/// @} end of group
