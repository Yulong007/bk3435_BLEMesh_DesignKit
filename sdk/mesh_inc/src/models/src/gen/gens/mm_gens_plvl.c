/**
 ****************************************************************************************
 * @file mm_gens_plvl.c
 *
 * @brief Mesh Model Generic Power Level Server Module
 *
 * Copyright (C) RivieraWaves 2017-2019
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup MM_GENS_PLVL
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "mm_gens_int.h"      // Mesh Model Generic Server Module Internal Definitions


#if (BLE_MESH_MDL_GENS_PLVL)
/*
 * DEFINES
 ****************************************************************************************
 */

/// Validity of information provided to the Replay Manager
#define MM_GENS_PLVL_REPLAY_MS               (6000)

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

/// Structure for Generic Power Level server model environment
typedef struct mm_gens_plvl_env
{
    /// Basic model environment - Must be first element in the structure - DO NOT MOVE
    mm_tb_state_mdl_env_t env;
    /// Timer for sending of publications
    mesh_tb_timer_t tmr_publi;
    /// Publication period in milliseconds
    uint32_t publi_period_ms;

    /// Environment for replay protection mechanism
    mm_tb_replay_mdl_env_t replay_env;

    /// Delta value in case of move transition
    int16_t move_delta;
    /// Generic Power Actual state value
    uint16_t power_actual;
    /// Target value of Generic Power Actual state value
    uint16_t tgt_power_actual;
    /// Generic Power Last state value
    uint16_t power_last;
    /// Generic Power Default state value
    uint16_t power_dflt;
    /// Generic Power Range Min state value
    uint16_t power_min;
    /// Generic Power Range Max state value
    uint16_t power_max;

    /// Source address of set message that has triggered last or current transition
    uint16_t status_dst_addr;
    /// Application key local index to be used for transmission of Generic Level Status message
    m_lid_t status_app_key_lid;
    /// Relaying of sent Generic Level Status authorized
    bool status_relay;
} mm_gens_plvl_env_t;

/// Structure for Generic Power Level Setup Server model environment
typedef struct mm_gens_plvls_env
{
    /// Basic model environment - Must be first element in the structure - DO NOT MOVE
    mm_tb_state_mdl_env_t env;
    /// Pointer to environment of associated Generic Power Level Server model
    mm_gens_plvl_env_t *p_env_plvl;
} mm_gens_plvls_env_t;


/*
 * LOCAL FUNCTIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Prepare and send a Generic Power Level Status message
 *
 * @param[in] p_env_plvl        Pointer to Generic Power Level Server model environment
 * @param[in] p_route_env       Pointer to structure containing reception information provided
 * by Mesh Profile for received request message
 * @param[in] publish           Indicate if message is sent as a publication (true) or as
 * a response to a received request (false)
 ****************************************************************************************
 */
__STATIC void mm_gens_plvl_send_plvl_status(mm_gens_plvl_env_t *p_env_plvl,
        mm_route_buf_env_t *p_route_env, bool publish)
{
    // Pointer to the buffer that will contain the message
    mesh_tb_buf_t *p_buf_status;
    // Remaining time
    uint8_t rem_time;
    // Transition type
    uint8_t trans_type;
    // Data length
    uint8_t data_length;

    // Check if a transition has been started
    mm_tb_bind_get_trans_info(p_env_plvl->env.grp_lid, &trans_type, &rem_time);

    // Deduce deduce data length
    data_length = (trans_type != MM_TRANS_TYPE_NONE)
                  ? MM_GEN_PLVL_STATUS_LEN : MM_GEN_PLVL_STATUS_MIN_LEN;

    if (mm_route_buf_alloc(&p_buf_status, data_length) == MESH_ERR_NO_ERROR)
    {
        // Get pointer to data
        uint8_t *p_data = MESH_TB_BUF_DATA(p_buf_status);
        // Get pointer to environment
        mm_route_buf_env_t *p_buf_env = (mm_route_buf_env_t *)&p_buf_status->env;

        // Prepare environment
        if (p_route_env)
        {
            memcpy(p_buf_env, p_route_env, sizeof(mm_route_buf_env_t));
        }
        else if (!publish)
        {
            p_buf_env->app_key_lid = p_env_plvl->status_app_key_lid;
            p_buf_env->u_addr.dst = p_env_plvl->status_dst_addr;
            SETB(p_buf_env->info, MM_ROUTE_BUF_INFO_RELAY, p_env_plvl->status_relay);
        }

        SETB(p_buf_env->info, MM_ROUTE_BUF_INFO_RX, 0);
        SETB(p_buf_env->info, MM_ROUTE_BUF_INFO_PUBLISH, publish);
        p_buf_env->mdl_lid = p_env_plvl->env.mdl_lid;
        p_buf_env->opcode = MM_MSG_GEN_PLVL_STATUS;

        // Fill the message
        co_write16p(p_data + MM_GEN_PLVL_STATUS_PRES_POWER_POS, p_env_plvl->power_actual);

        if (data_length == MM_GEN_PLVL_STATUS_LEN)
        {
            // Targeted Generic Power Actual state value
            uint16_t tgt_power;

            if (trans_type == MM_TRANS_TYPE_MOVE)
            {
                tgt_power = (p_env_plvl->move_delta > 0) ? p_env_plvl->power_max
                            : p_env_plvl->power_min;
                rem_time = MM_TRANS_TIME_UNKNOWN;
            }
            else
            {
                tgt_power = p_env_plvl->tgt_power_actual;
            }

            co_write16p(p_data + MM_GEN_PLVL_STATUS_TGT_POWER_POS, tgt_power);
            *(p_data + MM_GEN_PLVL_STATUS_REM_TIME_POS) = rem_time;
        }

        // Send the message
        mm_route_send(p_buf_status);
    }
    else
    {
        MESH_MODEL_PRINT_WARN("%s, Buffer alloc fail. \n", __func__);
    }
}

/**
 ****************************************************************************************
 * @brief Prepare and send a Generic Power Last Status message
 *
 * @param[in] p_env_plvl        Pointer to Generic Power Level Server model environment
 * @param[in] p_route_env       Pointer to structure containing reception information provided
 * by Mesh Profile for received request message
 ****************************************************************************************
 */
__STATIC void mm_gens_plvl_send_plast_status(mm_gens_plvl_env_t *p_env_plvl,
        mm_route_buf_env_t *p_route_env)
{
    // Pointer to the buffer that will contain the message
    mesh_tb_buf_t *p_buf_status;
    // Allocate a new buffer for the publication
    uint8_t status = mm_route_buf_alloc_status(&p_buf_status,
                     MM_GEN_PLVL_LAST_STATUS_LEN, p_route_env);

    if (status == MESH_ERR_NO_ERROR)
    {
        // Get pointer to data
        uint8_t *p_data = MESH_TB_BUF_DATA(p_buf_status);
        // Get pointer to environment
        mm_route_buf_env_t *p_buf_env = (mm_route_buf_env_t *)&p_buf_status->env;

        // Prepare environment
        p_buf_env->mdl_lid = p_env_plvl->env.mdl_lid;
        p_buf_env->opcode = MM_MSG_GEN_PLAST_STATUS;

        // Fill the message
        co_write16p(p_data + MM_GEN_PLVL_LAST_STATUS_POWER_POS, p_env_plvl->power_last);

        // Send the message
        mm_route_send(p_buf_status);
    }
    else
    {
        MESH_MODEL_PRINT_WARN("%s, Buffer alloc fail. \n", __func__);
    }
}

/**
 ****************************************************************************************
 * @brief Prepare and send a Generic Power Default Status message
 *
 * @param[in] p_env_plvl        Pointer to Generic Power Level Server model environment
 * @param[in] p_route_env       Pointer to structure containing reception information provided
 * by Mesh Profile for received request message
 ****************************************************************************************
 */
__STATIC void mm_gens_plvl_send_pdflt_status(mm_gens_plvl_env_t *p_env_plvl,
        mm_route_buf_env_t *p_route_env)
{
    // Pointer to the buffer that will contain the message
    mesh_tb_buf_t *p_buf_status;
    // Allocate a new buffer for the publication
    uint8_t status = mm_route_buf_alloc_status(&p_buf_status,
                     MM_GEN_PLVL_DFLT_STATUS_LEN, p_route_env);

    if (status == MESH_ERR_NO_ERROR)
    {
        // Get pointer to data
        uint8_t *p_data = MESH_TB_BUF_DATA(p_buf_status);
        // Get pointer to environment
        mm_route_buf_env_t *p_buf_env = (mm_route_buf_env_t *)&p_buf_status->env;

        // Prepare environment
        p_buf_env->mdl_lid = p_env_plvl->env.mdl_lid;
        p_buf_env->opcode = MM_MSG_GEN_PDFLT_STATUS;

        // Fill the message
        co_write16p(p_data + MM_GEN_PLVL_DFLT_STATUS_POWER_POS, p_env_plvl->power_dflt);

        // Send the message
        mm_route_send(p_buf_status);
    }
    else
    {
        MESH_MODEL_PRINT_WARN("%s, Buffer alloc fail. \n", __func__);
    }
}

/**
 ****************************************************************************************
 * @brief Prepare and send a Generic Power Range Status message
 *
 * @param[in] p_env_plvl        Pointer to Generic Power Level Server model environment
 * @param[in] p_route_env       Pointer to structure containing reception information provided
 * by Mesh Profile for received request message
 * @param[in] status            Status sent in the Generic Power Range Status message
 * (@see enum mm_status)
 ****************************************************************************************
 */
__STATIC void mm_gens_plvl_send_prange_status(mm_gens_plvl_env_t *p_env_plvl,
        mm_route_buf_env_t *p_route_env,
        uint8_t status)
{
    // Pointer to the buffer that will contain the message
    mesh_tb_buf_t *p_buf_status;
    // Allocate a new buffer for the publication
    uint16_t alloc_status = mm_route_buf_alloc_status(&p_buf_status,
                            MM_GEN_PLVL_RANGE_STATUS_LEN, p_route_env);

    if (alloc_status == MESH_ERR_NO_ERROR)
    {
        // Get pointer to data
        uint8_t *p_data = MESH_TB_BUF_DATA(p_buf_status);
        // Get pointer to environment
        mm_route_buf_env_t *p_buf_env = (mm_route_buf_env_t *)&p_buf_status->env;

        // Prepare environment
        p_buf_env->mdl_lid = p_env_plvl->env.mdl_lid;
        p_buf_env->opcode = MM_MSG_GEN_PRANGE_STATUS;

        // Fill the message
        *(p_data + MM_GEN_PLVL_RANGE_STATUS_CODE_POS) = status;
        co_write16p(p_data + MM_GEN_PLVL_RANGE_STATUS_MIN_POS, p_env_plvl->power_min);
        co_write16p(p_data + MM_GEN_PLVL_RANGE_STATUS_MAX_POS, p_env_plvl->power_max);

        // Send the message
        mm_route_send(p_buf_status);
    }
    else
    {
        MESH_MODEL_PRINT_WARN("%s, Buffer alloc fail. \n", __func__);
    }
}

/**
 ****************************************************************************************
 * @brief Publish Generic Power Level state value if sending of publications is enabled
 *
 * @param[in] p_env_plvl         Pointer to Generic Power Level Server model environment
 ****************************************************************************************
 */
__STATIC void mm_gens_plvl_publish(mm_gens_plvl_env_t *p_env_plvl)
{
    // Check if sending of publication is enabled
    if (GETB(p_env_plvl->env.info, MM_TB_STATE_MDL_INFO_PUBLI))
    {
        mm_gens_plvl_send_plvl_status(p_env_plvl, NULL, true);
    }
}

/**
 ****************************************************************************************
 * @brief Check if a Generic Power Level Status message must be sent and send it if it
 * is the case
 *
 * @param[in] p_env_plvl        Pointer to Generic Power Level Server model environment
 ****************************************************************************************
 */
__STATIC void mm_gens_plvl_check_status_rsp(mm_gens_plvl_env_t *p_env_plvl)
{
    if (p_env_plvl->status_dst_addr != MESH_UNASSIGNED_ADDR)
    {
        // Send a response to the node that has required the transition
        mm_gens_plvl_send_plvl_status(p_env_plvl, NULL, false);

        p_env_plvl->status_dst_addr = MESH_UNASSIGNED_ADDR;
    }
}

/*
 * MESSAGE HANDLERS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Handler for Generic Power Level Set/Set Unacknowledged message
 *
 * @param[in] p_env         Pointer to environment of model for which message has been received
 * @param[in] p_buf         Pointer to buffer containing the received message
 * @param[in] p_route_env   Pointer to routing information for the received buffer
 ****************************************************************************************
 */
__STATIC void mm_gens_plvl_handler_set_plvl(mm_gens_plvl_env_t *p_env_plvl, mesh_tb_buf_t *p_buf,
        mm_route_buf_env_t *p_route_env)
{
    do
    {
        // Check if a Generic Power Level Status message must be sent
        bool send_status = (p_route_env->opcode == MM_MSG_GEN_PLVL_SET);
        // Get pointer to data
        uint8_t *p_data = MESH_TB_BUF_DATA(p_buf);
        // Extract power value
        uint16_t power = co_read16p(p_data + MM_GEN_PLVL_SET_POWER_POS);
        // Extract TID value
        uint8_t tid = *(p_data + MM_GEN_PLVL_SET_TID_POS);
        // Transition time
        uint8_t trans_time = MM_TRANS_TIME_UNKNOWN;
        // Delay
        uint8_t delay = 0;

        // Extract and check optional parameters if present
        if (p_buf->data_len == MM_GEN_PLVL_SET_LEN)
        {
            trans_time = (uint16_t)(*(p_data + MM_GEN_PLVL_SET_TRANS_TIME_POS));

            // Check received value
            if (GETF(trans_time, MM_TRANS_TIME_STEP_NB) > MM_TRANS_TIME_NB_STEPS_MAX)
            {
                // Drop the message
                break;
            }

            delay = *(p_data + MM_GEN_PLVL_SET_DELAY_POS);
        }

        // Check if Generic Power Actual state is modified
        if ((p_env_plvl->status_dst_addr != MESH_UNASSIGNED_ADDR)
                || mm_tb_replay_is_retx(&p_env_plvl->replay_env, p_route_env->u_addr.src, tid)
                || (power == p_env_plvl->power_actual))
        {
            // Send a Generic Power Level Status message
            if (send_status)
            {
                mm_gens_plvl_send_plvl_status(p_env_plvl, p_route_env, false);
            }
            break;
        };

        if (send_status)
        {
            // Keep information for transmission of status
            p_env_plvl->status_dst_addr = p_route_env->u_addr.src;
            p_env_plvl->status_app_key_lid = p_route_env->app_key_lid;
            p_env_plvl->status_relay = GETB(p_route_env->info, MM_ROUTE_BUF_INFO_RELAY);
        }

        if (power != 0)
        {
            // Ensure that Generic Power Actual state value is between Generic Power Range
            // Min and Max values
            if (power > p_env_plvl->power_max)
            {
                power = p_env_plvl->power_max;
            }
            else if (power < p_env_plvl->power_min)
            {
                power = p_env_plvl->power_min;
            }
        }

        // Update target state
        p_env_plvl->tgt_power_actual = power;

        // Inform the Binding Manager about new transition
        mm_tb_bind_trans_new(p_env_plvl->env.grp_lid, MM_TRANS_TYPE_CLASSIC,
                             trans_time, delay);
    }
    while (0);
}

/**
 ****************************************************************************************
 * @brief Handler for Generic Power Default Set/Set Unacknowledged message
 *
 * @param[in] p_env         Pointer to environment of model for which message has been received
 * @param[in] p_buf         Pointer to buffer containing the received message
 * @param[in] p_route_env   Pointer to routing information for the received buffer
 ****************************************************************************************
 */
__STATIC void mm_gens_plvl_handler_set_pdflt(mm_gens_plvl_env_t *p_env_plvl, mesh_tb_buf_t *p_buf,
        mm_route_buf_env_t *p_route_env)
{
    // Get pointer to data
    uint8_t *p_data = MESH_TB_BUF_DATA(p_buf);
    // Extract received state value
    uint16_t power_dflt = co_read16p(p_data + MM_GEN_PLVL_DFLT_SET_POWER_POS);

    if (power_dflt != p_env_plvl->power_dflt)
    {
        // Keep received value
        p_env_plvl->power_dflt = power_dflt;

        // Inform application about received value
        mm_api_send_srv_state_upd_ind(MM_STATE_GEN_POWER_DFLT, p_env_plvl->env.elmt_idx,
                                      p_env_plvl->power_dflt, 0);
    }

    // If needed, send a Generic Power Default Status message to the requester
    if (p_route_env->opcode == MM_MSG_GEN_PDFLT_SET)
    {
        mm_gens_plvl_send_pdflt_status(p_env_plvl, p_route_env);
    }
}

/**
 ****************************************************************************************
 * @brief Handler for Generic Power Range Set/Set Unacknowledged message
 *
 * @param[in] p_env         Pointer to environment of model for which message has been received
 * @param[in] p_buf         Pointer to buffer containing the received message
 * @param[in] p_route_env   Pointer to routing information for the received buffer
 ****************************************************************************************
 */
__STATIC void mm_gens_plvl_handler_set_prange(mm_gens_plvl_env_t *p_env_plvl, mesh_tb_buf_t *p_buf,
        mm_route_buf_env_t *p_route_env)
{
    do
    {
        // Get pointer to data
        uint8_t *p_data = MESH_TB_BUF_DATA(p_buf);
        // Status
        uint8_t status = MM_STATUS_SUCCESS;
        // Extract Generic Power Range state value
        uint16_t power_min = co_read16p(p_data + MM_GEN_PLVL_RANGE_SET_MIN_POS);
        // Extract Generic Power Range state value
        uint16_t power_max = co_read16p(p_data + MM_GEN_PLVL_RANGE_SET_MAX_POS);

        // Check provided values
        if (power_min == 0)
        {
            status = MM_STATUS_CANNOT_SET_RANGE_MIN;
        }
        else if (power_max == 0)
        {
            status = MM_STATUS_CANNOT_SET_RANGE_MAX;
        }
        else if (power_min > power_max)
        {
            // Drop the message
            break;
        }

        if ((status == MM_STATUS_SUCCESS)
                && ((p_env_plvl->power_min != power_min)
                    || (p_env_plvl->power_max != power_max)))
        {
            p_env_plvl->power_min = power_min;
            p_env_plvl->power_max = power_max;

            // Inform application about received value
            mm_api_send_srv_state_upd_ind(MM_STATE_GEN_POWER_RANGE, p_env_plvl->env.elmt_idx,
                                          power_min | ((uint32_t)power_max << 16), 0);
        }

        // If needed, send a Generic Power Range Status message to the requester
        if (p_route_env->opcode == MM_MSG_GEN_PRANGE_SET)
        {
            mm_gens_plvl_send_prange_status(p_env_plvl, p_route_env, status);
        }
    }
    while (0);
}

/*
 * INTERNAL CALLBACK FUNCTIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Callback function called when timer monitoring publication duration for
 * Generic Power Level Server model expires
 *
 * @param[in] p_env     Pointer to model environment for Generic Power Level Server model
 ****************************************************************************************
 */
__STATIC void mm_gens_plvl_cb_tmr_publi(void *p_env)
{
    // Get allocated environment
    mm_gens_plvl_env_t *p_env_plvl = (mm_gens_plvl_env_t *)p_env;

    if (p_env_plvl->publi_period_ms)
    {
        // Publish a Generic Power Level Status message
        mm_gens_plvl_publish(p_env_plvl);

        // Restart the timer
        mesh_tb_timer_set(&p_env_plvl->tmr_publi, p_env_plvl->publi_period_ms);
    }
}

/**
 ****************************************************************************************
 * @brief Inform Generic Power Level Server model about a received message
 *
 * @param[in] p_env         Pointer to the environment allocated for the Generic Power Level
 * Server model
 * @param[in] p_buf         Pointer to the buffer containing the received message
 * @param[in] p_route_env   Pointer to structure containing reception parameters provided
 * by the Mesh Profile block
 ****************************************************************************************
 */
__STATIC void mm_gens_plvl_cb_rx(mm_tb_state_mdl_env_t *p_env, mesh_tb_buf_t *p_buf,
                                 mm_route_buf_env_t *p_route_env)
{
    do
    {
        // Environment for Generic Power Level Server model
        mm_gens_plvl_env_t *p_env_plvl;

        if (p_env->mdl_id == MM_ID_GENS_PLVL)
        {
            p_env_plvl = (mm_gens_plvl_env_t *)p_env;

            switch (p_route_env->opcode)
            {
                case (MM_MSG_GEN_PLVL_GET):
                {
                    // Send a Generic Power Level Status message
                    mm_gens_plvl_send_plvl_status(p_env_plvl, p_route_env, false);
                } break;

                case (MM_MSG_GEN_PLVL_SET):
                case (MM_MSG_GEN_PLVL_SET_UNACK):
                {
                    // Handle the message
                    mm_gens_plvl_handler_set_plvl(p_env_plvl, p_buf, p_route_env);
                } break;

                case (MM_MSG_GEN_PLAST_GET):
                {
                    // Send a Generic Power Last Status message
                    mm_gens_plvl_send_plast_status(p_env_plvl, p_route_env);
                } break;

                case (MM_MSG_GEN_PDFLT_GET):
                {
                    // Send a Generic Power Default Status message
                    mm_gens_plvl_send_pdflt_status(p_env_plvl, p_route_env);
                } break;

                case (MM_MSG_GEN_PRANGE_GET):
                {
                    // Send a Generic Power Range Status message
                    mm_gens_plvl_send_prange_status(p_env_plvl, p_route_env, MM_STATUS_SUCCESS);
                } break;

                default:
                {
                } break;
            }
        }
        else
        {
            // Environment for Generic Power Level Setup Server model
            mm_gens_plvls_env_t *p_env_plvls = (mm_gens_plvls_env_t *)p_env;
            p_env_plvl = p_env_plvls->p_env_plvl;

            switch (p_route_env->opcode)
            {
                case (MM_MSG_GEN_PDFLT_SET):
                case (MM_MSG_GEN_PDFLT_SET_UNACK):
                {
                    // Handle the message
                    mm_gens_plvl_handler_set_pdflt(p_env_plvl, p_buf, p_route_env);
                } break;

                case (MM_MSG_GEN_PRANGE_SET):
                case (MM_MSG_GEN_PRANGE_SET_UNACK):
                {
                    // Handle the message
                    mm_gens_plvl_handler_set_prange(p_env_plvl, p_buf, p_route_env);
                } break;

                default:
                {
                } break;
            }
        }
    }
    while (0);
}

/**
 ****************************************************************************************
 * @brief Check if a given opcode is handled by the Generic Power Level Server model
 *
 * @param[in] p_env         Pointer to the environment allocated for the Generic Power Level
 * Server model
 * @param[in] opcode        Opcode to check
 *
 * @return An error status (@see enum mesh_err)
 ****************************************************************************************
 */
__STATIC uint16_t mm_gens_plvl_cb_opcode_check(mm_tb_state_mdl_env_t *p_env, uint32_t opcode)
{
    uint16_t status = MESH_ERR_MDL_INVALID_OPCODE;

    if (p_env->mdl_id == MM_ID_GENS_PLVL)
    {
        if ((opcode == MM_MSG_GEN_PLVL_GET)
                || (opcode == MM_MSG_GEN_PLVL_SET)
                || (opcode == MM_MSG_GEN_PLVL_SET_UNACK)
                || (opcode == MM_MSG_GEN_PLAST_GET)
                || (opcode == MM_MSG_GEN_PDFLT_GET)
                || (opcode == MM_MSG_GEN_PRANGE_GET))
        {
            status = MESH_ERR_NO_ERROR;
        }
    }
    else if (p_env->mdl_id == MM_ID_GENS_PLVLS)
    {
        if ((opcode == MM_MSG_GEN_PDFLT_SET)
                || (opcode == MM_MSG_GEN_PDFLT_SET_UNACK)
                || (opcode == MM_MSG_GEN_PRANGE_SET)
                || (opcode == MM_MSG_GEN_PRANGE_SET_UNACK))
        {
            status = MESH_ERR_NO_ERROR;
        }
    }

    if (status != MESH_ERR_NO_ERROR)
    {
        MESH_MODEL_PRINT_INFO("%s, opcode (0x%x)\n", __func__, opcode);
    }

    return (status);
}


/**
 ****************************************************************************************
 * @brief Inform Generic Power Level Server model about received publication parameters
 *
 * @param[in] p_env         Pointer the the environment allocated for the Generic Power Level
 * Server model
 * @param[in] addr          Publication address
 * @param[in] period_ms     Publication period in milliseconds
 ****************************************************************************************
 */
__STATIC void mm_gens_plvl_cb_publish_param(mm_tb_state_mdl_env_t *p_env, uint16_t addr, uint32_t period_ms)
{
    // Inform the state manager about received publication parameters
    mm_tb_state_publish_param_ind((mm_tb_state_mdl_publi_env_t *)p_env, addr, period_ms);
}

/**
 ****************************************************************************************
 * @brief Set Generic Power Actual or Generic Power Default or Generic Power Range state value
 *
 * @param[in] p_env         Pointer the the environment allocated for the Generic OnOff
 * Server model
 * @param[in] state_id      State identifier
 * @param[in] state         Generic Power Actual or Generic Power Default or Generic Power
 * Range state value
 *
 * @return An error status (@see enum mesh_err)
 ****************************************************************************************
 */
__STATIC uint16_t mm_gens_plvl_cb_set(mm_tb_state_mdl_env_t *p_env, uint16_t state_id, uint32_t state)
{
    // Returned status
    uint16_t status = MESH_ERR_NO_ERROR;
    // Get environment for the Generic Power Level Server model
    mm_gens_plvl_env_t *p_env_plvl = (mm_gens_plvl_env_t *)p_env;

    switch (state_id)
    {
        case (MM_STATE_GEN_POWER_ACTUAL):
        case (MM_STATE_GEN_POWER_DFLT):
        {
            uint16_t level = state;

            if ((level == 0)
                    || ((level >= p_env_plvl->power_min)
                        && (level <= p_env_plvl->power_max)))
            {
                if (state_id == MM_STATE_GEN_POWER_ACTUAL)
                {
                    p_env_plvl->power_actual = level;

                    if (level != 0)
                    {
                        p_env_plvl->power_last = level;
                    }

                    // Set the targeted Generic OnOff state value
                    mm_tb_bind_set_state(p_env_plvl->env.grp_lid, MM_STATE_TYPE_TARGET, MM_ID_GENS_OO,
                                         (p_env_plvl->power_actual == 0) ? 0 : 1);

                    // Set the targeted Generic Level state value
                    mm_tb_bind_set_state(p_env_plvl->env.grp_lid, MM_STATE_TYPE_TARGET, MM_ID_GENS_LVL,
                                         p_env_plvl->power_actual - 32768);
                }
                else
                {
                    p_env_plvl->power_dflt = level;
                }
            }
            else
            {
                MESH_MODEL_PRINT_WARN("%s, Invalid parameter, Line %d. \n", __func__, __LINE__);
                status = MESH_ERR_INVALID_PARAM;
            }
        } break;

        case (MM_STATE_GEN_POWER_RANGE):
        {

        } break;

        default:
        {
            MESH_MODEL_PRINT_WARN("%s, Invalid parameter, Line %d. \n", __func__, __LINE__);
            status = MESH_ERR_INVALID_PARAM;
        } break;
    }

    return (status);
}

/*
 * GLOBAL FUNCTIONS
 ****************************************************************************************
 */

uint16_t mm_gens_plvl_register(uint8_t elmt_idx, m_lid_t *p_mdl_lid)
{
    uint16_t status;

    do
    {
        // Pointer to allocated environment for Generic Power Level Server model
        mm_gens_plvl_env_t *p_env_plvl;
        // Model local index
        m_lid_t mdl_lid;
        // Pointer to server-specific callback functions
        mm_srv_cb_t *p_cb_srv;

        // Register Generic Power Level Server model
        status = m_api_register_model(MM_ID_GENS_PLVL, elmt_idx, M_MDL_CONFIG_PUBLI_AUTH_BIT,
                                      &mm_route_cb, p_mdl_lid);

        if (status != MESH_ERR_NO_ERROR)
        {
            MESH_MODEL_PRINT_WARN("%s, Register Generic Power Level Server model fail, status = 0x%x \n",
                                  __func__, status);
            break;
        }

        // Inform the Model State Manager about registered model
        status = mm_tb_state_register(elmt_idx, MM_ID_GENS_PLVL, *p_mdl_lid,
                                      MM_TB_STATE_ROLE_SRV | MM_TB_STATE_CFG_CB_BIT,
                                      sizeof(mm_gens_plvl_env_t));

        if (status != MESH_ERR_NO_ERROR)
        {
            MESH_MODEL_PRINT_WARN("%s, Model State register fail, status = 0x%x \n",
                                  __func__, status);
            break;
        }

        // Get allocated environment
        p_env_plvl = (mm_gens_plvl_env_t *)mm_tb_state_get_env(*p_mdl_lid);

        // Get server-specific callback functions
        p_cb_srv = p_env_plvl->env.cb.u.p_cb_srv;

        // Initiate states
        p_env_plvl->power_min = 1;
        p_env_plvl->power_max = 0xFFFF;

        // Prepare environment for Replay Manager
        p_env_plvl->replay_env.delay_ms = MM_GENS_PLVL_REPLAY_MS;

        // Prepare timer for publications
        p_env_plvl->tmr_publi.cb = mm_gens_plvl_cb_tmr_publi;
        p_env_plvl->tmr_publi.p_env = (void *)p_env_plvl;

        // Set internal callback functions
        p_env_plvl->env.cb.cb_rx = mm_gens_plvl_cb_rx;
        p_env_plvl->env.cb.cb_opcode_check = mm_gens_plvl_cb_opcode_check;
        p_env_plvl->env.cb.cb_publish_param = mm_gens_plvl_cb_publish_param;
        p_cb_srv->cb_set = mm_gens_plvl_cb_set;

        // Inform application about registered model
        mm_api_send_register_ind(MM_ID_GENS_PLVL, elmt_idx, *p_mdl_lid);

        // Register Generic Power Level Setup Server model
        status = m_api_register_model(MM_ID_GENS_PLVLS, elmt_idx, 0, &mm_route_cb, &mdl_lid);

        if (status != MESH_ERR_NO_ERROR)
        {
            break;
        }

        // Inform the Model State Manager about registered model
        status = mm_tb_state_register(elmt_idx, MM_ID_GENS_PLVLS, mdl_lid,
                                      MM_TB_STATE_ROLE_SRV, sizeof(mm_gens_plvls_env_t));

        if (status == MESH_ERR_NO_ERROR)
        {
            // Get allocated environment
            mm_gens_plvls_env_t *p_env_plvls = (mm_gens_plvls_env_t *)mm_tb_state_get_env(mdl_lid);

            // Set internal callback functions
            p_env_plvls->env.cb.cb_rx = mm_gens_plvl_cb_rx;
            p_env_plvls->env.cb.cb_opcode_check = mm_gens_plvl_cb_opcode_check;

            // Link environment
            p_env_plvls->p_env_plvl = p_env_plvl;

            // Inform application about registered model
            mm_api_send_register_ind(MM_ID_GENS_PLVLS, elmt_idx, mdl_lid);
        }
        else
        {
            MESH_MODEL_PRINT_WARN("%s, Model State register fail, Line %d, status = 0x%x \n",
                                  __func__, __LINE__, status);
        }
    }
    while (0);

    return (status);
}

/*
 * CALLBACK FUNCTIONS FOR BINDING MANAGER
 ****************************************************************************************
 */

void mm_gens_plvl_cb_grp_event(m_lid_t mdl_lid, uint8_t event, uint8_t info)
{
    // Get environment for Generic Power Server model
    mm_gens_plvl_env_t *p_env_plvl = (mm_gens_plvl_env_t *)mm_tb_state_get_env(mdl_lid);
    MESH_MODEL_PRINT_DEBUG("p_mdl_info->cb_grp_event \r\n");
    switch (event)
    {
        case (MESH_MDL_GRP_EVENT_TRANS_DELAY_EXPIRED):
        {
            // Set the targeted Generic OnOff state value
            mm_tb_bind_set_state(p_env_plvl->env.grp_lid, MM_STATE_TYPE_TARGET, MM_ID_GENS_OO,
                                 (p_env_plvl->tgt_power_actual == 0) ? 0 : 1);

            // Set the targeted Generic Level state value
            mm_tb_bind_set_state(p_env_plvl->env.grp_lid, MM_STATE_TYPE_TARGET, MM_ID_GENS_LVL,
                                 p_env_plvl->tgt_power_actual - 32768);

            // Start the transition
            mm_tb_bind_trans_start(p_env_plvl->env.grp_lid);
        } break;

        case (MESH_MDL_GRP_EVENT_TRANS_IMMEDIATE):
        {
            p_env_plvl->power_actual = p_env_plvl->tgt_power_actual;

            if (p_env_plvl->power_actual != 0)
            {
                p_env_plvl->power_last = p_env_plvl->power_actual;
            }
        } // no break;

        case (MESH_MDL_GRP_EVENT_TRANS_STARTED):
        {
            uint8_t trans_time = info;

            // Inform application about state update
            mm_api_send_srv_state_upd_ind(MM_STATE_GEN_POWER_ACTUAL, p_env_plvl->env.elmt_idx,
                                          p_env_plvl->tgt_power_actual,
                                          (trans_time) ? mm_tb_get_trans_time_ms(trans_time) : 0);

            // Check if a status message must be sent
            mm_gens_plvl_check_status_rsp(p_env_plvl);

            // Send a publication
            mm_gens_plvl_publish(p_env_plvl);
        } break;

        case (MESH_MDL_GRP_EVENT_TRANS_END):
        {
            p_env_plvl->power_actual = p_env_plvl->tgt_power_actual;

            // Inform application about state update
            mm_api_send_srv_state_upd_ind(MM_STATE_GEN_POWER_ACTUAL, p_env_plvl->env.elmt_idx,
                                          p_env_plvl->power_actual, 0);

            if (p_env_plvl->power_actual != 0)
            {
                p_env_plvl->power_last = p_env_plvl->power_actual;
            }

            // Check if a status message must be sent
            mm_gens_plvl_check_status_rsp(p_env_plvl);

            // Send a publication
            mm_gens_plvl_publish(p_env_plvl);
        } break;

        case (MESH_MDL_GRP_EVENT_GROUP_FULL):
        {
            // Set Generic Level state value
            mm_tb_bind_set_state(p_env_plvl->env.grp_lid, MM_STATE_TYPE_CURRENT,
                                 MM_ID_GENS_LVL, 0x8000);
        } break;

        //case (MESH_MDL_GRP_EVENT_TRANS_ABORTED):
        default:
        {
        } break;
    }
}

void mm_gens_plvl_cb_trans_req(m_lid_t main_mdl_lid, uint32_t req_model_id, uint8_t trans_type,
                               uint32_t state_delta)
{
    MESH_MODEL_PRINT_DEBUG("%s\r\n", __func__);
    // Get environment for Generic Power Server model
    mm_gens_plvl_env_t *p_env_plvl = (mm_gens_plvl_env_t *)mm_tb_state_get_env(main_mdl_lid);
    // Targeted Generic Power Actual state value
    uint16_t tgt_power;

    if (req_model_id == MM_ID_GENS_OO)
    {
        // Requested Generic OnOff state value
        uint8_t onoff = (uint8_t)state_delta;

        if (onoff == 0)
        {
            tgt_power = 0;
        }
        else
        {
            tgt_power =  (p_env_plvl->power_dflt == 0) ? p_env_plvl->power_last
                         : p_env_plvl->power_dflt;
        }
    }
    else     // req_model_id == MM_ID_GENS_LVL
    {
        if (trans_type == MM_TRANS_TYPE_CLASSIC)
        {
            // Requested Generic Level state value
            int16_t level = (int16_t)state_delta;

            // Generic Power Actual = Generic Level + 32768
            tgt_power = 32768 + level;
        }
        else     // ((trans_type == MM_TRANS_TYPE_DELTA) || trans_type == MM_TRANS_TYPE_MOVE))
        {
            // Delta value
            int32_t delta;

            if (trans_type == MM_TRANS_TYPE_MOVE)
            {
                delta = (int16_t)state_delta;

                // Keep the provided delta value
                p_env_plvl->move_delta = (int16_t)state_delta;
            }
            else
            {
                delta = (int32_t)state_delta;
            }

            // Add the Generic Power Actual state value to the received delta value
            delta += p_env_plvl->power_actual;

            // The Generic Power Actual state value cannot wrap
            if (delta < 0)
            {
                tgt_power = 0;
            }
            else
            {
                tgt_power = (delta > 0xFFFF) ? 0xFFFF : (uint16_t)delta;
            }
        }
    }

    // Check that new targeted Generic Power Actual state value is well within the set
    // Generic Power Range state min and max values
    if (tgt_power != 0)
    {
        // Ensure that Generic Power Actual state value is between Generic Power Range
        // Min and Max values
        if (tgt_power > p_env_plvl->power_max)
        {
            tgt_power = p_env_plvl->power_max;
        }
        else if (tgt_power < p_env_plvl->power_min)
        {
            tgt_power = p_env_plvl->power_min;
        }
    }

    // Check if Generic Power Actual state value is modified
    if (tgt_power != p_env_plvl->power_actual)
    {
        p_env_plvl->tgt_power_actual = tgt_power;

        // Start a new transition
        mm_tb_bind_trans_new(p_env_plvl->env.grp_lid, trans_type, 0, 0);
    }
    else
    {
        // Reject the transition
        mm_tb_bind_trans_reject(p_env_plvl->env.grp_lid);
    }
}
#endif //(BLE_MESH_MDL_GENS_PLVL)
/// @} end of group
