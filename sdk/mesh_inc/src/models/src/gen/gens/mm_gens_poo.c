/**
 ****************************************************************************************
 * @file mm_gens_poo.c
 *
 * @brief Mesh Model Generic Power OnOff Server Module
 *
 * Copyright (C) RivieraWaves 2017-2019
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup MM_GENS_POO
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "mm_gens_int.h"      // Mesh Model Generic Server Module Internal Definitions


#if (BLE_MESH_MDL_GENS_POO)
/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

/// Structure for Generic Power OnOff Server model environment
typedef struct mm_gens_poo_env
{
    /// Basic model environment - Must be first element in the structure - DO NOT MOVE
    mm_tb_state_mdl_env_t env;
    /// Timer for sending of publications
    mesh_tb_timer_t tmr_publi;
    /// Publication period
    uint32_t publi_period_ms;

    /// Generic OnPowerUp state value
    uint8_t on_power_up;
} mm_gens_poo_env_t;

/// Structure for Generic Power OnOff Setup Server model environment
typedef struct mm_gens_poos_env
{
    /// Basic model environment - Must be first element in the structure - DO NOT MOVE
    mm_tb_state_mdl_env_t env;
    /// Pointer to environment of associated Generic Power OnOff Server model
    mm_gens_poo_env_t *p_env_poo;
} mm_gens_poos_env_t;

/*
 * LOCAL FUNCTIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Prepare and send a Generic OnPowerUp Status
 *
 * @param[in] p_env_poo          Pointer to Generic Power OnOff Server model environment
 ****************************************************************************************
 */
__STATIC void mm_gens_poo_send_status(mm_gens_poo_env_t *p_env_poo, mm_route_buf_env_t *p_route_env)
{
    // Pointer to the buffer that will contain the message
    mesh_tb_buf_t *p_buf_status;
    // Status
    uint8_t status = mm_route_buf_alloc(&p_buf_status, MM_GEN_POO_STATUS_LEN);

    if (status == MESH_ERR_NO_ERROR)
    {
        // Get pointer to data
        uint8_t *p_data = MESH_TB_BUF_DATA(p_buf_status);
        // Get pointer to environment
        mm_route_buf_env_t *p_env = (mm_route_buf_env_t *)&p_buf_status->env[0];

        // Prepare environment
        if (p_route_env)
        {
            memcpy(p_env, p_route_env, sizeof(mm_route_buf_env_t));
        }

        p_env->opcode = MM_MSG_GEN_ONPUP_STATUS;
        SETB(p_env->info, MM_ROUTE_BUF_INFO_PUBLISH, (p_route_env == NULL));
        SETB(p_env->info, MM_ROUTE_BUF_INFO_RX, 0);

        // Fill the message
        *(p_data + MM_GEN_POO_STATUS_OPU_POS) = p_env_poo->on_power_up;

        // Send the message
        mm_route_send(p_buf_status);
    }
    else
    {
        MESH_MODEL_PRINT_WARN("%s, Buffer alloc fail.\n", __func__);
    }
}

/*
 * MESSAGE HANDLERS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Handler for Generic OnPowerUp Set/Set Unacknowledged message
 *
 * @param[in] p_env_poo     Pointer to environment for Generic OnPowerUp Server model
 * @param[in] p_buf         Pointer to received message content
 * @param[in] p_route_env   Pointer to structure containing reception parameters provided
 * by the Mesh Profile block
 ****************************************************************************************
 */
__STATIC void mm_gens_poo_handler_set(mm_gens_poo_env_t *p_env_poo, mesh_tb_buf_t *p_buf,
                                      mm_route_buf_env_t *p_route_env)
{
    // Set pointer to received message
    uint8_t *p_data = MESH_TB_BUF_DATA(p_buf);
    // Get received Generic OnPowerUp state value
    uint8_t on_power_up = *(p_data + MM_GEN_POO_SET_OPU_POS);

    // Check received value
    if (on_power_up <= MM_ON_POWER_UP_RESTORE)
    {
        // Check if state value has been changed
        bool updated = (on_power_up != p_env_poo->on_power_up);

        if (updated)
        {
            // Keep the provided value
            p_env_poo->on_power_up = on_power_up;

            // Inform application about received value
            mm_api_send_srv_state_upd_ind(MM_STATE_GEN_ONPOWERUP, p_env_poo->env.elmt_idx,
                                          on_power_up, 0);
        }

        // Send the response
        if (p_route_env->opcode == MM_MSG_GEN_ONPUP_SET)
        {
            mm_gens_poo_send_status(p_env_poo, p_route_env);
        }

        // Send a publication if authorized
        if (updated
                && GETB(p_env_poo->env.info, MM_TB_STATE_MDL_INFO_PUBLI))
        {
            mm_gens_poo_send_status(p_env_poo, NULL);
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
 * Generic Power OnOff Server model expires
 *
 * @param[in] p_env     Pointer to model environment for Generic Power OnOff Server
 * model
 ****************************************************************************************
 */
__STATIC void mm_gens_poo_cb_tmr_publi(void *p_env)
{
    // Get allocated environment
    mm_gens_poo_env_t *p_env_poo = (mm_gens_poo_env_t *)p_env;

    if (p_env_poo->publi_period_ms)
    {
        // Publish a Generic OnPowerUp Status message
        mm_gens_poo_send_status(p_env_poo, NULL);

        // Restart the timer
        mesh_tb_timer_set(&p_env_poo->tmr_publi, p_env_poo->publi_period_ms);
    }
}

/**
 ****************************************************************************************
 * @brief Inform Generic Power OnOff Server model about a received message
 *
 * @param[in] p_env         Pointer to the environment allocated for the Generic Power OnOff
 * Server model
 * @param[in] p_buf         Pointer to the buffer containing the received message
 * @param[in] p_route_env   Pointer to structure containing reception parameters provided
 * by the Mesh Profile block
 ****************************************************************************************
 */
__STATIC void mm_gens_poo_cb_rx(mm_tb_state_mdl_env_t *p_env, mesh_tb_buf_t *p_buf,
                                mm_route_buf_env_t *p_route_env)
{
    if (p_route_env->opcode != MM_MSG_GEN_ONPUP_GET)
    {
        // Environment for Generic Power OnOff Setup Server model
        mm_gens_poos_env_t *p_env_poos = (mm_gens_poos_env_t *)p_env;

        // Handle Generic OnPowerUp Set/Set Unacknowledged message
        mm_gens_poo_handler_set(p_env_poos->p_env_poo, p_buf, p_route_env);
    }
    else
    {
        // Send a Generic OnPowerUp Status message
        mm_gens_poo_send_status((mm_gens_poo_env_t *)p_env, p_route_env);
    }
}

/**
 ****************************************************************************************
 * @brief Check if a given opcode is handled by the Generic Power OnOff Server model
 *
 * @param[in] p_env         Pointer to the environment allocated for the Generic Power OnOff
 * Server model
 * @param[in] opcode        Opcode to check
 *
 * @return An error status (@see enum mesh_err)
 ****************************************************************************************
 */
__STATIC uint16_t mm_gens_poo_cb_opcode_check(mm_tb_state_mdl_env_t *p_env, uint32_t opcode)
{
    uint16_t status = MESH_ERR_MDL_INVALID_OPCODE;

    if (p_env->mdl_id == MM_ID_GENS_POO)
    {
        if (opcode == MM_MSG_GEN_ONPUP_GET)
        {
            status = MESH_ERR_NO_ERROR;
        }
    }
    else if (p_env->mdl_id == MM_ID_GENS_POOS)
    {
        if ((opcode == MM_MSG_GEN_ONPUP_SET)
                || (opcode == MM_MSG_GEN_ONPUP_SET_UNACK))
        {
            status = MESH_ERR_NO_ERROR;
        }
    }

    if (status != MESH_ERR_NO_ERROR)
    {
        MESH_MODEL_PRINT_INFO("%s, Invalid opcode (0x%x)\n", __func__, opcode);
    }

    return (status);
}

/**
 ****************************************************************************************
 * @brief Inform Generic Power OnOff Server model about received publication parameters
 *
 * @param[in] p_env         Pointer the the environment allocated for the Generic Power OnOff
 * Server model
 * @param[in] addr          Publication address
 * @param[in] period_ms     Publication period in milliseconds
 ****************************************************************************************
 */
__STATIC void mm_gens_poo_cb_publish_param(mm_tb_state_mdl_env_t *p_env, uint16_t addr, uint32_t period_ms)
{
    // Inform the state manager about received publication parameters
    mm_tb_state_publish_param_ind((mm_tb_state_mdl_publi_env_t *)p_env, addr, period_ms);
}

/**
 ****************************************************************************************
 * @brief Set Generic OnPowerUp state value
 *
 * @param[in] p_env         Pointer the the environment allocated for the Generic OnOff
 * Server model
 * @param[in] on_power_up   Generic OnPowerUp state value
 *
 * @return An error status (@see enum mesh_err)
 ****************************************************************************************
 */
__STATIC uint16_t mm_gens_poo_cb_set(mm_tb_state_mdl_env_t *p_env, uint16_t state_id,
                                     uint32_t state)
{
    // Returned status
    uint16_t status = MESH_ERR_NO_ERROR;
    // Generic OnPowerUp state value
    uint8_t on_power_up = state;

    // Check provided value
    if (on_power_up <= MM_ON_POWER_UP_RESTORE)
    {
        // Get environment for the Generic Power OnOff Server model
        mm_gens_poo_env_t *p_env_poo = (mm_gens_poo_env_t *)p_env;

        // Keep the provided value
        p_env_poo->on_power_up = on_power_up;
    }
    else
    {
        MESH_MODEL_PRINT_WARN("%s, Invalid parameters.\n", __func__);
        status = MESH_ERR_INVALID_PARAM;
    }

    return (status);
}

/*
 * GLOBAL FUNCTIONS
 ****************************************************************************************
 */

uint16_t mm_gens_poo_register(uint8_t elmt_idx)
{
    uint16_t status;

    do
    {
        // Allocated model local index
        m_lid_t mdl_lid;
        // Pointer to allocated environment for Generic Power OnOff Server model
        mm_gens_poo_env_t *p_env_poo;
        // Pointer to server-specific callback functions
        mm_srv_cb_t *p_cb_srv;

        // Register Generic Power OnOff Server model
        status = m_api_register_model(MM_ID_GENS_POO, elmt_idx, M_MDL_CONFIG_PUBLI_AUTH_BIT,
                                      &mm_route_cb, &mdl_lid);

        if (status != MESH_ERR_NO_ERROR)
        {
            MESH_MODEL_PRINT_WARN("%s, Register Generic Power OnOff Server model fail, status = 0x%x\n", __func__, status);
            break;
        }

        // Inform the Model State Manager about registered model
        status = mm_tb_state_register(elmt_idx, MM_ID_GENS_POO, mdl_lid,
                                      MM_TB_STATE_ROLE_SRV | MM_TB_STATE_CFG_CB_BIT,
                                      sizeof(mm_gens_poo_env_t));

        if (status != MESH_ERR_NO_ERROR)
        {
            MESH_MODEL_PRINT_WARN("%s, Register Generic Power OnOff Server model State fail, status = 0x%x\n", __func__, status);
            break;
        }

        // Get allocated environment
        p_env_poo = (mm_gens_poo_env_t *)mm_tb_state_get_env(mdl_lid);

        // Get server-specific callback functions
        p_cb_srv = p_env_poo->env.cb.u.p_cb_srv;

        // Prepare timer for publications
        p_env_poo->tmr_publi.cb = mm_gens_poo_cb_tmr_publi;
        p_env_poo->tmr_publi.p_env = (void *)p_env_poo;

        // Set internal callback functions
        p_env_poo->env.cb.cb_rx = mm_gens_poo_cb_rx;
        p_env_poo->env.cb.cb_opcode_check = mm_gens_poo_cb_opcode_check;
        p_env_poo->env.cb.cb_publish_param = mm_gens_poo_cb_publish_param;
        p_cb_srv->cb_set = mm_gens_poo_cb_set;

        // Inform application about registered model
        mm_api_send_register_ind(MM_ID_GENS_POO, elmt_idx, mdl_lid);

        // Register Generic Power OnOff Setup Server model
        status = m_api_register_model(MM_ID_GENS_POOS, elmt_idx, 0, &mm_route_cb, &mdl_lid);

        if (status != MESH_ERR_NO_ERROR)
        {
            MESH_MODEL_PRINT_WARN("%s, Register Generic Power OnOff Setup Server model fail, status = 0x%x\n", __func__, status);
            break;
        }

        // Inform the Model State Manager about registered model
        status = mm_tb_state_register(elmt_idx, MM_ID_GENS_POOS, mdl_lid,
                                      MM_TB_STATE_ROLE_SRV, sizeof(mm_gens_poos_env_t));

        if (status == MESH_ERR_NO_ERROR)
        {
            // Get allocated environment
            mm_gens_poos_env_t *p_env_poos = (mm_gens_poos_env_t *)mm_tb_state_get_env(mdl_lid);

            // Set internal callback functions
            p_env_poos->env.cb.cb_rx = mm_gens_poo_cb_rx;
            p_env_poos->env.cb.cb_opcode_check = mm_gens_poo_cb_opcode_check;

            // Link environment
            p_env_poos->p_env_poo = p_env_poo;

            // Inform application about registered model
            mm_api_send_register_ind(MM_ID_GENS_POOS, elmt_idx, mdl_lid);
        }
        else
        {
            MESH_MODEL_PRINT_WARN("%s, Register Generic Power OnOff Setup Server Model State fail, status = 0x%x\n", __func__, status);
        }
    }
    while (0);

    return (status);
}
#endif //(BLE_MESH_MDL_GENS_POO)
/// @} end of group
