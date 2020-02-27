/**
 ****************************************************************************************
 * @file mm_gens_dtt.c
 *
 * @brief Mesh Model Generic Default Transition Time Server Module
 *
 * Copyright (C) RivieraWaves 2017-2019
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup MM_GENS_DTT
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "mm_gens_int.h"      // Mesh Model Generic Server Module Internal Definitions


#if (BLE_MESH_MDL_GENS_DTT)
/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

/// Structure for Generic Default Transition Time server model environment
typedef struct mm_gens_dtt_env
{
    /// Basic model environment - Must be first element in the structure - DO NOT MOVE
    mm_tb_state_mdl_env_t env;
    /// Timer for sending of publications
    mesh_tb_timer_t tmr_publi;
    /// Publication period
    uint32_t publi_period_ms;

    /// Default Transition Time state value
    uint8_t dft_trans_time;
} mm_gens_dtt_env_t;

/*
 * LOCAL FUNCTIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Prepare and send a Generic Default Transition Time Status
 *
 * @param[in] p_env_dtt          Pointer to Generic Default Transition Time Server model
 * environment
 ****************************************************************************************
 */
__STATIC void mm_gens_dtt_send_status(mm_gens_dtt_env_t *p_env_dtt, mm_route_buf_env_t *p_route_env)
{
    // Pointer to the buffer that will contain the message
    mesh_tb_buf_t *p_buf_status;
    // Status
    uint8_t status = mm_route_buf_alloc(&p_buf_status, MM_GEN_DTT_STATUS_LEN);

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

        p_env->opcode = MM_MSG_GEN_DTT_STATUS;
        p_env->mdl_lid = p_env_dtt->env.mdl_lid;
        SETB(p_env->info, MM_ROUTE_BUF_INFO_PUBLISH, (p_route_env == NULL));
        SETB(p_env->info, MM_ROUTE_BUF_INFO_RX, 0);

        // Fill the message
        *(p_data + MM_GEN_DDT_STATUS_DDT_POS) = p_env_dtt->dft_trans_time;

        // Send the message
        mm_route_send(p_buf_status);
    }
    else
    {
        MESH_MODEL_PRINT_WARN("%s, Alloc buffer fail.\n", __func__);
    }
}

/*
 * MESSAGE HANDLERS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Handler for Generic Default Transition Time Set/Set Unacknowledged message
 *
 * @param[in] p_env_dtt     Pointer to environment for Generic Default Transition Time model
 * @param[in] p_buf         Pointer to received message content
 * @param[in] p_route_env   Pointer to structure containing reception parameters provided
 * by the Mesh Profile block
 ****************************************************************************************
 */
__STATIC void mm_gens_dtt_handler_set(mm_gens_dtt_env_t *p_env_dtt, mesh_tb_buf_t *p_buf,
                                      mm_route_buf_env_t *p_route_env)
{
    // Set pointer to received message
    uint8_t *p_data = MESH_TB_BUF_DATA(p_buf);
    // Get received default transition time
    uint8_t dft_trans_time = *(p_data + MM_GEN_DDT_SET_DDT_POS);

    // Check received value
    if (GETF(dft_trans_time, MM_TRANS_TIME_STEP_NB) <= MM_TRANS_TIME_NB_STEPS_MAX)
    {
        // Check if state value has been changed
        bool update = (dft_trans_time != p_env_dtt->dft_trans_time);

        if (update)
        {
            // Keep the provided value
            p_env_dtt->dft_trans_time = dft_trans_time;

            // Inform the Binding Manager
            mm_tb_bind_set_dft_trans_time(p_env_dtt->env.elmt_idx, dft_trans_time);

            // Inform application about received value
            mm_api_send_srv_state_upd_ind(MM_STATE_GEN_DTT, p_env_dtt->env.elmt_idx,
                                          dft_trans_time, 0);

        }

        // Send the response
        if (p_route_env->opcode == MM_MSG_GEN_DTT_SET)
        {
            mm_gens_dtt_send_status(p_env_dtt, p_route_env);
        }

        // Send a publication if authorized
        if (update
                && GETB(p_env_dtt->env.info, MM_TB_STATE_MDL_INFO_PUBLI))
        {
            mm_gens_dtt_send_status(p_env_dtt, NULL);
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
 * Generic Default Transition Time Server model expires
 *
 * @param[in] p_env     Pointer to model environment for Generic Default Transition Time Server
 * model
 ****************************************************************************************
 */
__STATIC void mm_gens_dtt_cb_tmr_publi(void *p_env)
{
    // Get allocated environment
    mm_gens_dtt_env_t *p_env_dtt = (mm_gens_dtt_env_t *)p_env;

    if (p_env_dtt->publi_period_ms)
    {
        // Publish a Generic Default Transition Time Status message
        mm_gens_dtt_send_status(p_env_dtt, NULL);

        // Restart the timer
        mesh_tb_timer_set(&p_env_dtt->tmr_publi, p_env_dtt->publi_period_ms);
    }
}

/**
 ****************************************************************************************
 * @brief Inform Generic Default Transition Time Server model about a received message
 *
 * @param[in] p_env         Pointer to the environment allocated for the Generic Default
 * Transition Time Server model
 * @param[in] p_buf         Pointer to the buffer containing the received message
 * @param[in] p_route_env   Pointer to structure containing reception parameters provided
 * by the Mesh Profile block
 ****************************************************************************************
 */
__STATIC void mm_gens_dtt_cb_rx(mm_tb_state_mdl_env_t *p_env, mesh_tb_buf_t *p_buf,
                                mm_route_buf_env_t *p_route_env)
{
    // Get environment for Generic Default Transition Time Server model
    mm_gens_dtt_env_t *p_env_dtt = (mm_gens_dtt_env_t *)p_env;

    if (p_route_env->opcode != MM_MSG_GEN_DTT_GET)
    {
        // Handle Generic Default Transition Time Set/Set Unacknowledged message
        mm_gens_dtt_handler_set(p_env_dtt, p_buf, p_route_env);
    }
    else
    {
        // Send a Generic Default Transition Time Status message
        mm_gens_dtt_send_status(p_env_dtt, p_route_env);
    }
}

/**
 ****************************************************************************************
 * @brief Check if a given opcode is handled by the Generic Default Transition Time Server model
 *
 * @param[in] p_env         Pointer to the environment allocated for the Generic Default Transition
 * Time Server model
 * @param[in] opcode        Opcode to check
 *
 * @return An error status (@see enum mesh_err)
 ****************************************************************************************
 */
__STATIC uint16_t mm_gens_dtt_cb_opcode_check(mm_tb_state_mdl_env_t *p_env, uint32_t opcode)
{
    uint16_t status;

    if ((opcode == MM_MSG_GEN_DTT_GET)
            || (opcode == MM_MSG_GEN_DTT_SET)
            || (opcode == MM_MSG_GEN_DTT_SET_UNACK))
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
 * @brief Inform Generic Default Transition Time Server model about received publication
 * parameters
 *
 * @param[in] p_env         Pointer the the environment allocated for the Generic Default
 * Transition Time Server model
 * @param[in] addr          Publication address
 * @param[in] period_ms     Publication period in milliseconds
 ****************************************************************************************
 */
__STATIC void mm_gens_dtt_cb_publish_param(mm_tb_state_mdl_env_t *p_env, uint16_t addr, uint32_t period_ms)
{
    // Inform the state manager about received publication parameters
    mm_tb_state_publish_param_ind((mm_tb_state_mdl_publi_env_t *)p_env, addr, period_ms);
}

/**
 ****************************************************************************************
 * @brief Set Generic Default Transition Time state value
 *
 * @param[in] p_env         Pointer the the environment allocated for the Generic OnOff
 * Server model
 * @param[in] dtt           Generic Default Transition Time state value
 *
 * @return An error status (@see enum mesh_err)
 ****************************************************************************************
 */
__STATIC uint16_t mm_gens_dtt_cb_set(mm_tb_state_mdl_env_t *p_env, uint16_t state_id, uint32_t state)
{
    // Returned status
    uint16_t status = MESH_ERR_NO_ERROR;
    // Generic Default Transition Time state value
    uint8_t dft_trans_time = (uint8_t)state;

    // Check provided value
    if (GETF(dft_trans_time, MM_TRANS_TIME_STEP_NB) <= MM_TRANS_TIME_NB_STEPS_MAX)
    {
        // Get environment for the Generic Default Transition Time Server model
        mm_gens_dtt_env_t *p_env_dtt = (mm_gens_dtt_env_t *)p_env;

        // Keep the provided value
        p_env_dtt->dft_trans_time = dft_trans_time;

        // Inform the Binding Manager
        mm_tb_bind_set_dft_trans_time(p_env_dtt->env.elmt_idx, dft_trans_time);
    }
    else
    {
        MESH_MODEL_PRINT_WARN("%s, Invalid parameter.\n", __func__);
        status = MESH_ERR_INVALID_PARAM;
    }

    return (status);
}

/*
 * GLOBAL FUNCTIONS
 ****************************************************************************************
 */

uint16_t mm_gens_dtt_register(uint8_t elmt_idx)
{
    // Allocated model local index
    m_lid_t mdl_lid;
    // Register the model
    uint16_t status = m_api_register_model(MM_ID_GENS_DTT, elmt_idx, M_MDL_CONFIG_PUBLI_AUTH_BIT,
                                           &mm_route_cb, &mdl_lid);

    if (status == MESH_ERR_NO_ERROR)
    {
        // Inform the Model State Manager about registered model
        status = mm_tb_state_register(elmt_idx, MM_ID_GENS_DTT, mdl_lid,
                                      MM_TB_STATE_ROLE_SRV | MM_TB_STATE_CFG_CB_BIT,
                                      sizeof(mm_gens_dtt_env_t));

        if (status == MESH_ERR_NO_ERROR)
        {
            // Get allocated environment
            mm_gens_dtt_env_t *p_env_dtt = (mm_gens_dtt_env_t *)mm_tb_state_get_env(mdl_lid);
            // Get server-specific callback functions
            mm_srv_cb_t *p_cb_srv = p_env_dtt->env.cb.u.p_cb_srv;

            // Prepare timer for publications
            p_env_dtt->tmr_publi.cb = mm_gens_dtt_cb_tmr_publi;
            p_env_dtt->tmr_publi.p_env = (void *)p_env_dtt;

            // Set internal callback functions
            p_env_dtt->env.cb.cb_rx = mm_gens_dtt_cb_rx;
            p_env_dtt->env.cb.cb_opcode_check = mm_gens_dtt_cb_opcode_check;
            p_env_dtt->env.cb.cb_publish_param = mm_gens_dtt_cb_publish_param;
            p_cb_srv->cb_set = mm_gens_dtt_cb_set;

            // Inform application about registered model
            mm_api_send_register_ind(MM_ID_GENS_DTT, elmt_idx, mdl_lid);
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
#endif //(BLE_MESH_MDL_GENS_DTT)
/// @} end of group
