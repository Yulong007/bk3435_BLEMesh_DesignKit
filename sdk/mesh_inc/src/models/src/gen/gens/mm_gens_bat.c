/**
 ****************************************************************************************
 * @file mm_gens_bat.c
 *
 * @brief Mesh Model Generic Battery Server Module
 *
 * Copyright (C) RivieraWaves 2017-2019
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup MM_GENS_BAT
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "mm_gens_int.h"      // Mesh Model Generic Server Module Internal Definitions


#if (BLE_MESH_MDL_GENS_BAT)
/*
 * ENUMERATIONS
 ****************************************************************************************
 */

/// Information bit field
/// 8               2          1       0
/// +---------------+----------+-------+
/// |       RFU     | Wait App | Publi |
/// +---------------+----------+-------+
enum mm_gens_bat_info
{
    /// Publication to be sent after confirmation received from application
    MM_GENS_BAT_INFO_PUBLI_POS = 0,
    MM_GENS_BAT_INFO_PUBLI_BIT = CO_BIT(MM_GENS_BAT_INFO_PUBLI_POS),

    /// Wait for confirmation from application
    MM_GENS_BAT_INFO_WAIT_APP_POS = 1,
    MM_GENS_BAT_INFO_WAIT_APP_BIT = CO_BIT(MM_GENS_BAT_INFO_WAIT_APP_POS),
};

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

/// Structure for Generic Battery Server model environment
typedef struct mm_gens_bat_env
{
    /// Basic model environment - Must be first element in the structure - DO NOT MOVE
    mm_tb_state_mdl_env_t env;
    /// Timer for sending of publications
    mesh_tb_timer_t tmr_publi;
    /// Publication period
    uint32_t publi_period_ms;

    /// List of prepared Generic Battery Status messages
    co_list_t list_status;

    /// Information (@see enum mm_gens_bat_info)
    uint8_t info;
} mm_gens_bat_env_t;

/*
 * LOCAL FUNCTIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Prepare and send a Generic Battery Status message
 *
 * @param[in] p_env_bat          Pointer to Generic Battery Server model environment
 ****************************************************************************************
 */
__STATIC void mm_gens_bat_send_status(mm_gens_bat_env_t *p_env_bat, mesh_tb_buf_t *p_buf,
                                      uint8_t bat_lvl, uint32_t time_discharge, uint32_t time_charge,
                                      uint8_t flags)
{
    // Status
    uint8_t status;

    if (p_buf)
    {
        status = MESH_ERR_NO_ERROR;
    }
    else
    {
        status = mm_route_buf_alloc(&p_buf, MM_GEN_BAT_STATUS_LEN);
    }

    if (status == MESH_ERR_NO_ERROR)
    {
        // Get pointer to data
        uint8_t *p_data = MESH_TB_BUF_DATA(p_buf);
        // Get pointer to environment
        mm_route_buf_env_t *p_env = (mm_route_buf_env_t *)&p_buf->env[0];

        p_env->opcode = MM_MSG_GEN_BAT_STATUS;
        SETB(p_env->info, MM_ROUTE_BUF_INFO_PUBLISH, (p_buf == NULL));
        SETB(p_env->info, MM_ROUTE_BUF_INFO_RX, 0);

        // Fill the message
        *(p_data + MM_GEN_BAT_STATUS_LEVEL_POS) = bat_lvl;
        co_write24p(p_data + MM_GEN_BAT_STATUS_TIME_DISCHARGE_POS, time_discharge);
        co_write24p(p_data + MM_GEN_BAT_STATUS_TIME_CHARGE_POS, time_charge);
        *(p_data + MM_GEN_BAT_STATUS_FLAGS_POS) = flags;

        // Send the message
        mm_route_send(p_buf);
    }
    else
    {
        MESH_MODEL_PRINT_WARN("%s, buffer alloc fail.\n", __func__);
    }
}

/**
 ****************************************************************************************
 * @brief Inform application that current Generic Battery State is required
 *
 * @param[in] p_env_bat          Pointer to Generic Battery Server model environment
 ****************************************************************************************
 */
__STATIC void mm_gens_bat_send_req_ind(mm_gens_bat_env_t *p_env_bat)
{
    // Check that a confirmation is not already expected from application
    if (!GETB(p_env_bat->info, MM_GENS_BAT_INFO_WAIT_APP))
    {
        SETB(p_env_bat->info, MM_GENS_BAT_INFO_WAIT_APP, 1);
        mm_api_send_srv_state_req_ind(MM_API_SRV_BAT_REQ_IND, p_env_bat->env.elmt_idx);
    }
}

/**
 ****************************************************************************************
 * @brief Look for buffer prepared for transmission of Generic Battery Status message to a given
 * node's element
 *
 * @param[in] p_env_bat     Pointer to environment allocated for the Generic Battery Server model
 * @param[in] src           Address of node's element
 ****************************************************************************************
 */
__STATIC bool mm_gens_bat_find_get(mm_gens_bat_env_t *p_env_bat, uint16_t src)
{
    // Buffer with same source address has been found
    bool found = false;
    // Get first buffer
    mesh_tb_buf_t *p_buf = (mesh_tb_buf_t *)co_list_pick(&p_env_bat->list_status);

    while (p_buf)
    {
        // Read environment
        mm_route_buf_env_t *p_env = (mm_route_buf_env_t *)&p_buf->env[0];

        if (p_env->u_addr.src == src)
        {
            found = true;
            break;
        }

        // Get next buffer
        p_buf = (mesh_tb_buf_t *)p_buf->hdr.next;
    }

    return (found);
}

/**
 ****************************************************************************************
 * @brief Prepare a buffer for transmission of Generic Battery Status message
 *
 * @param[in] p_env_bat          Pointer to Generic Battery Server model environment
 * @param[in] p_route_env        Information about received Generic Battery Get message
 ****************************************************************************************
 */
__STATIC void mm_gens_bat_prepare_status(mm_gens_bat_env_t *p_env_bat,
        mm_route_buf_env_t *p_route_env)
{
    // Check that a Generic Battery Get message from the same source address has not already
    // been received
    if (!mm_gens_bat_find_get(p_env_bat, p_route_env->u_addr.src))
    {
        // Buffer that will contain the Generic Battery Status message
        mesh_tb_buf_t *p_buf_status;
        // Allocate a new buffer
        uint16_t status = mm_route_buf_alloc(&p_buf_status, MM_GEN_BAT_STATUS_LEN);

        if (status == MESH_ERR_NO_ERROR)
        {
            // Copy the received environment
            memcpy(&p_buf_status->env[0], p_route_env, sizeof(mm_route_buf_env_t));

            // Insert the buffer in the list of received get messages
            co_list_push_back(&p_env_bat->list_status, &p_buf_status->hdr);
        }
    }
}

/*
 * INTERNAL CALLBACK FUNCTIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Callback function called when timer monitoring publication duration for
 * Generic Battery Server model expires
 *
 * @param[in] p_env     Pointer to model environment for Generic Battery Server model
 ****************************************************************************************
 */
__STATIC void mm_gens_bat_cb_tmr_publi(void *p_env)
{
    // Get allocated environment
    mm_gens_bat_env_t *p_env_bat = (mm_gens_bat_env_t *)p_env;

    if (p_env_bat->publi_period_ms)
    {
        // Keep in mind that a publication must be sent
        SETB(p_env_bat->info, MM_GENS_BAT_INFO_PUBLI, 1);

        // Retrieve current Generic Battery state from the application
        mm_gens_bat_send_req_ind(p_env_bat);

        // Restart the timer
        mesh_tb_timer_set(&p_env_bat->tmr_publi, p_env_bat->publi_period_ms);
    }
}

/**
 ****************************************************************************************
 * @brief Inform Generic Battery Server model about a received message
 *
 * @param[in] p_env         Pointer to the environment allocated for the Generic Battery
 * Server model
 * @param[in] p_buf         Pointer to the buffer containing the received message
 * @param[in] p_route_env   Pointer to structure containing reception parameters provided
 * by the Mesh Profile block
 ****************************************************************************************
 */
__STATIC void mm_gens_bat_cb_rx(mm_tb_state_mdl_env_t *p_env, mesh_tb_buf_t *p_buf,
                                mm_route_buf_env_t *p_route_env)
{
    do
    {
        // Get environment for Generic Battery Server model
        mm_gens_bat_env_t *p_env_bat = (mm_gens_bat_env_t *)p_env;

        // Prepare buffer for transmission of Generic Battery Status message
        mm_gens_bat_prepare_status(p_env_bat, p_route_env);

        // Retrieve current Generic Battery state from the application
        mm_gens_bat_send_req_ind(p_env_bat);
    }
    while (0);
}

/**
 ****************************************************************************************
 * @brief Check if a given opcode is handled by the Generic Battery Server model
 *
 * @param[in] p_env         Pointer to the environment allocated for the Generic Battery
 * Server model
 * @param[in] opcode        Opcode to check
 *
 * @return An error status (@see enum mesh_err)
 ****************************************************************************************
 */
__STATIC uint16_t mm_gens_bat_cb_opcode_check(mm_tb_state_mdl_env_t *p_env, uint32_t opcode)
{
    uint16_t status;

    if (opcode == MM_MSG_GEN_BAT_GET)
    {
        status = MESH_ERR_NO_ERROR;
    }
    else
    {
        MESH_MODEL_PRINT_INFO("%s,Invalid opcode (0x%x).\n", __func__, opcode);
        status = MESH_ERR_MDL_INVALID_OPCODE;
    }

    return (status);
}

/**
 ****************************************************************************************
 * @brief Inform Generic Battery Server model about received publication parameters
 *
 * @param[in] p_env         Pointer the the environment allocated for the Generic Battery
 * Server model
 * @param[in] addr          Publication address
 * @param[in] period_ms     Publication period in milliseconds
 ****************************************************************************************
 */
__STATIC void mm_gens_bat_cb_publish_param(mm_tb_state_mdl_env_t *p_env, uint16_t addr, uint32_t period_ms)
{
    mm_gens_bat_env_t *p_env_bat = (mm_gens_bat_env_t *)p_env;

    // Inform the state manager about received publication parameters
    mm_tb_state_publish_param_ind((mm_tb_state_mdl_publi_env_t *)p_env, addr, period_ms);

    if (p_env_bat->publi_period_ms == 0)
    {
        SETB(p_env_bat->info, MM_GENS_BAT_INFO_PUBLI, 0);
    }
}

/*
 * GLOBAL FUNCTIONS
 ****************************************************************************************
 */

uint16_t mm_gens_bat_register(uint8_t elmt_idx)
{
    // Allocated model local index
    m_lid_t mdl_lid;
    // Register the model
    uint16_t status = m_api_register_model(MM_ID_GENS_BAT, elmt_idx, M_MDL_CONFIG_PUBLI_AUTH_BIT,
                                           &mm_route_cb, &mdl_lid);

    if (status == MESH_ERR_NO_ERROR)
    {
        // Inform the Model State Manager about registered model
        status = mm_tb_state_register(elmt_idx, MM_ID_GENS_BAT, mdl_lid,
                                      MM_TB_STATE_ROLE_SRV, sizeof(mm_gens_bat_env_t));

        if (status == MESH_ERR_NO_ERROR)
        {
            // Get allocated environment
            mm_gens_bat_env_t *p_env_bat = (mm_gens_bat_env_t *)mm_tb_state_get_env(mdl_lid);

            // Prepare timer for publications
            p_env_bat->tmr_publi.cb = mm_gens_bat_cb_tmr_publi;
            p_env_bat->tmr_publi.p_env = (void *)p_env_bat;

            // Set internal callback functions
            p_env_bat->env.cb.cb_rx = mm_gens_bat_cb_rx;
            p_env_bat->env.cb.cb_opcode_check = mm_gens_bat_cb_opcode_check;
            p_env_bat->env.cb.cb_publish_param = mm_gens_bat_cb_publish_param;

            // Inform application about registered model
            mm_api_send_register_ind(MM_ID_GENS_BAT, elmt_idx, mdl_lid);
        }
        else
        {
            MESH_MODEL_PRINT_WARN("%s, Register the model state fail, status = 0x%x.\n", __func__, status);
        }
    }
    else
    {
        MESH_MODEL_PRINT_WARN("%s, Register the model fail, status = 0x%x.\n", __func__, status);
    }

    return (status);
}

void mm_gens_bat_cfm(mm_tb_state_mdl_env_t *p_env, uint16_t status, uint8_t bat_lvl,
                     uint32_t time_discharge, uint32_t time_charge, uint8_t flags)
{
    mm_gens_bat_env_t *p_env_bat = (mm_gens_bat_env_t *)p_env;

    // Check if confirmation from application was expected
    if (GETB(p_env_bat->info, MM_GENS_BAT_INFO_WAIT_APP))
    {
        SETB(p_env_bat->info, MM_GENS_BAT_INFO_WAIT_APP, 0);

        if (status != MESH_ERR_NO_ERROR)
        {
            bat_lvl = MM_BAT_LVL_UNKNOWN;
            time_discharge = MM_BAT_TIME_DISCHARGE_UNKNOWN;
            time_charge = MM_BAT_TIME_CHARGE_UNKNOWN;
            flags = MM_BAT_FLAGS_UNKNOWN;
        }
        else
        {
            // Check the provided values
            if (bat_lvl > MM_BAT_LVL_MAX)
            {
                bat_lvl = MM_BAT_LVL_UNKNOWN;
            }

            if (time_discharge > MM_BAT_TIME_DISCHARGE_UNKNOWN)
            {
                time_discharge = MM_BAT_TIME_DISCHARGE_UNKNOWN;
            }

            if (time_charge > MM_BAT_TIME_CHARGE_UNKNOWN)
            {
                time_charge = MM_BAT_TIME_CHARGE_UNKNOWN;
            }
        }

        // Send responses for received get requests
        while (!co_list_is_empty(&p_env_bat->list_status))
        {
            mesh_tb_buf_t *p_buf = (mesh_tb_buf_t *)co_list_pop_front(&p_env_bat->list_status);

            mm_gens_bat_send_status(p_env_bat, p_buf,
                                    bat_lvl, time_discharge, time_charge, flags);
        }

        // Send a publication if needed
        if (GETB(p_env_bat->info, MM_GENS_BAT_INFO_PUBLI))
        {
            SETB(p_env_bat->info, MM_GENS_BAT_INFO_PUBLI, 0);

            mm_gens_bat_send_status(p_env_bat, NULL,
                                    bat_lvl, time_discharge, time_charge, flags);
        }
    }
}
#endif  //(BLE_MESH_MDL_GENS_BAT)
/// @} end of group
