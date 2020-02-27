/**
 ****************************************************************************************
 * @file mm_gens_lvl.c
 *
 * @brief Mesh Model Generic Level Server Module
 *
 * Copyright (C) RivieraWaves 2017-2019
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup MM_GENS_LVL
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "mm_gens_int.h"      // Mesh Model Generic Server Module Internal Definitions


#if (BLE_MESH_MDL_GENS_LVL)
/*
 * DEFINES
 ****************************************************************************************
 */

/// Upper limit of Generic Level State when no model is bound (signed 16-bit integer)
#define MM_GENS_LVL_UPPER_LIMIT             (32767)
/// Lower limit of Generic Level State when no model is bound (signed 16-bit integer)
#define MM_GENS_LVL_LOWER_LIMIT             (-32768)
/// Validity of information provided to the Replay Manager
#define MM_GENS_LVL_REPLAY_MS               (6000)

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

/// Information for handling of move transition
typedef struct mm_gens_lvl_move_param
{
    /// Received delta value
    int16_t delta;
    /// Received transition time
    uint8_t trans_time;
} mm_gens_lvl_move_param_t;

/// Information for handling of delta transitions
typedef struct mm_gens_lvl_delta_param
{
    /// Source address
    uint16_t src;
    /// TID
    uint8_t tid;
} mm_gens_lvl_delta_param_t;

/// Structure for Generic Level server model environment
typedef struct mm_gens_lvl_env
{
    /// Basic model environment - Must be first element in the structure - DO NOT MOVE
    mm_tb_state_mdl_env_t env;
    /// Timer for sending of publications
    mesh_tb_timer_t tmr_publi;
    /// Publication period in milliseconds
    uint32_t publi_period_ms;

    /// Environment for replay protection mechanism
    mm_tb_replay_mdl_env_t replay_env;

    /// Parameters for a move transition
    mm_gens_lvl_move_param_t move_param;
    /// Parameters for a delta transition
    mm_gens_lvl_delta_param_t delta_param;
    union
    {
        /// Initial level in case of a delta transition
        int16_t init_level;
        /// Target level in case of move transition
        int16_t tgt_move_level;
    } u;
    /// Current level
    int16_t level;
    /// Target Level
    int16_t tgt_level;

    /// Source address of set message that has triggered last or current transition
    uint16_t status_dst_addr;
    /// Application key local index to be used for transmission of Generic Level Status message
    m_lid_t status_app_key_lid;
    /// Relaying of sent Generic Level Status authorized
    bool status_relay;
} mm_gens_lvl_env_t;

/*
 * LOCAL FUNCTIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Compute target level when delta value is applied for a delta transition
 *
 * @param[in] p_env_lvl     Pointer to Generic Level Server model environment
 * @param[in] delta         Delta value
 *
 * @return Computed target level
 ****************************************************************************************
 */
__STATIC int16_t mm_gens_lvl_compute_tgt_delta(mm_gens_lvl_env_t *p_env_lvl, int32_t delta)
{
    // Target level
    int32_t tgt_level = p_env_lvl->u.init_level;

    tgt_level += delta;

    if (tgt_level > MM_GENS_LVL_UPPER_LIMIT)
    {
        tgt_level = MM_GENS_LVL_UPPER_LIMIT;
    }
    else if (tgt_level < MM_GENS_LVL_LOWER_LIMIT)
    {
        tgt_level = MM_GENS_LVL_LOWER_LIMIT;
    }

    return ((int16_t)tgt_level);
}

/**
 ****************************************************************************************
 * @brief Compute target level when delta value is applied for a move transition
 *
 * @param[in] p_env_lvl     Pointer to Generic Level Server model environment
 * @param[in] delta         Delta value
 *
 * @return Computed target level
 ****************************************************************************************
 */
__STATIC int16_t mm_gens_lvl_compute_tgt_move(mm_gens_lvl_env_t *p_env_lvl)
{
    // Target level
    int32_t tgt_level = p_env_lvl->level;

    tgt_level += p_env_lvl->move_param.delta;

    if (tgt_level > MM_GENS_LVL_UPPER_LIMIT)
    {
        tgt_level = MM_GENS_LVL_UPPER_LIMIT;
    }
    else if (tgt_level < MM_GENS_LVL_LOWER_LIMIT)
    {
        tgt_level = MM_GENS_LVL_LOWER_LIMIT;
    }

    return ((int16_t)tgt_level);
}

/**
 ****************************************************************************************
 * @brief Prepare and send a Generic Level Status message
 *
 * @param[in] p_env_lvl         Pointer to Generic Level Server model environment
 * @param[in] p_route_env       Pointer to structure containing reception information provided
 * by Mesh Profile for received request message
 * @param[in] publish           Indicate if message is sent as a publication (true) or as
 * a response to a received request (false)
 ****************************************************************************************
 */
__STATIC void mm_gens_lvl_send_status(mm_gens_lvl_env_t *p_env_lvl, mm_route_buf_env_t *p_route_env,
                                      bool publish)
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
    mm_tb_bind_get_trans_info(p_env_lvl->env.grp_lid, &trans_type, &rem_time);

    // Deduce deduce data length
    data_length = (trans_type != MM_TRANS_TYPE_NONE)
                  ? MM_GEN_LVL_STATUS_LEN : MM_GEN_LVL_STATUS_MIN_LEN;

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
            p_buf_env->app_key_lid = p_env_lvl->status_app_key_lid;
            p_buf_env->u_addr.dst = p_env_lvl->status_dst_addr;
            SETB(p_buf_env->info, MM_ROUTE_BUF_INFO_RELAY, p_env_lvl->status_relay);
        }

        SETB(p_buf_env->info, MM_ROUTE_BUF_INFO_RX, 0);
        SETB(p_buf_env->info, MM_ROUTE_BUF_INFO_PUBLISH, publish);
        p_buf_env->mdl_lid = p_env_lvl->env.mdl_lid;
        p_buf_env->opcode = MM_MSG_GEN_LVL_STATUS;

        // Fill the message
        co_write16p(p_data + MM_GEN_LVL_STATUS_LVL_POS, p_env_lvl->level);

        if (data_length == MM_GEN_LVL_STATUS_LEN)
        {
            // Targeted level
            int16_t tgt_level;

            if (trans_type == MM_TRANS_TYPE_MOVE)
            {
                rem_time = MM_TRANS_TIME_UNKNOWN;
                tgt_level = p_env_lvl->u.tgt_move_level;
            }
            else
            {
                tgt_level = p_env_lvl->tgt_level;
            }

            co_write16p(p_data + MM_GEN_LVL_STATUS_TGT_LVL_POS, tgt_level);
            *(p_data + MM_GEN_LVL_STATUS_REM_TIME_POS) = rem_time;
        }

        // Send the message
        mm_route_send(p_buf_status);
    }
    else
    {
        MESH_MODEL_PRINT_WARN("%s, buffer alloc fail.\n", __func__);
    }
}

/**
 ****************************************************************************************
 * @brief Publish Generic Level state value if sending of publications is enabled
 *
 * @param[in] p_env_lvl          Pointer to Generic Level Server model environment
 ****************************************************************************************
 */
__STATIC void mm_gens_lvl_publish(mm_gens_lvl_env_t *p_env_lvl)
{
    // Check if sending of publication is enabled
    if (GETB(p_env_lvl->env.info, MM_TB_STATE_MDL_INFO_PUBLI))
    {
        mm_gens_lvl_send_status(p_env_lvl, NULL, true);
    }
}

/**
 ****************************************************************************************
 * @brief Check if a Generic Level Status message must be sent and send it if it
 * is the case
 *
 * @param[in] p_env_lvl        Pointer to Generic Level Server model environment
 ****************************************************************************************
 */
__STATIC void mm_gens_lvl_check_status_rsp(mm_gens_lvl_env_t *p_env_lvl)
{
    if (p_env_lvl->status_dst_addr)
    {
        // Send a response to the node that has required the transition
        mm_gens_lvl_send_status(p_env_lvl, NULL, false);

        p_env_lvl->status_dst_addr = MESH_UNASSIGNED_ADDR;
    }
}

/**
 ****************************************************************************************
 * @brief Check if a transaction initiated by a peer node is a new transaction or continuation
 * of an already started one
 *
 * @param[in] p_env_lvl        Pointer to Generic Level Server model environment
 * @param[in] src              Source address of received Generic Delta Set/Set Unacknowledged
 * message
 * @param[in] tid              Received transaction identifier
 ****************************************************************************************
 */
__STATIC bool mm_gens_lvl_is_new_transaction(mm_gens_lvl_env_t *p_env_lvl, uint16_t src, uint8_t tid)
{
    // New transaction or not
    bool new_transaction = true;

    // Check if the SRC/TID couple has already been received in the last 6 seconds
    if (mm_tb_replay_is_retx(&p_env_lvl->replay_env, src, tid))
    {
        if ((src == p_env_lvl->delta_param.src)
                && (tid == p_env_lvl->delta_param.tid))
        {
            new_transaction = false;
        }
    }

    return (new_transaction);
}

/*
 * MESSAGE HANDLERS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Handler for Generic Level Set/Set Unacknowledged message
 *
 * @param[in] p_env         Pointer to environment of model for which message has been received
 * @param[in] p_buf         Pointer to the buffer containing the received message
 * @param[in] p_route_env   Pointer to structure containing reception parameters provided
 * by the Mesh Profile block
 ****************************************************************************************
 */
__STATIC bool mm_gens_lvl_handler_set(mm_gens_lvl_env_t *p_env_lvl, mesh_tb_buf_t *p_buf,
                                      mm_route_buf_env_t *p_route_env)
{
    // Boolean indicating if Generic Level Status can be sent now
    bool send_status = (p_route_env->opcode == MM_MSG_GEN_LVL_SET);

    do
    {
        // Get pointer to data
        uint8_t *p_data = MESH_TB_BUF_DATA(p_buf);
        // Extract required Generic Level state value
        int16_t level = co_read16p(p_data + MM_GEN_LVL_SET_LVL_POS);
        // Extract TID value
        uint8_t tid = *(p_data + MM_GEN_LVL_SET_TID_POS);
        // Transition time
        uint8_t trans_time = MM_TRANS_TIME_UNKNOWN;
        // Delay
        uint8_t delay = 0;

        // Extract optional parameters if present
        if (p_buf->data_len == MM_GEN_LVL_SET_LEN)
        {
            trans_time = (uint16_t)(*(p_data + MM_GEN_LVL_SET_TRANS_TIME_POS));

            // Check received value
            if (GETF(trans_time, MM_TRANS_TIME_STEP_NB) > MM_TRANS_TIME_NB_STEPS_MAX)
            {
                // Drop the message
                send_status = false;
                break;
            }

            delay = *(p_data + MM_GEN_LVL_SET_DELAY_POS);
        }

        // Check if received message is a retransmitted one, if state is modified and if
        // a new transition can be started now
        if ((p_env_lvl->status_dst_addr != MESH_UNASSIGNED_ADDR)
                || mm_tb_replay_is_retx(&p_env_lvl->replay_env, p_route_env->u_addr.src, tid)
                || (level == p_env_lvl->level))
        {
            if (send_status)
            {
                // Send Generic Level status message
                mm_gens_lvl_send_status(p_env_lvl, p_route_env, false);
            }
            break;
        }

        // Keep information for transmission of status if needed
        if (send_status)
        {
            // Keep transaction information
            p_env_lvl->status_dst_addr = p_route_env->u_addr.src;
            p_env_lvl->status_app_key_lid = p_route_env->app_key_lid;
            p_env_lvl->status_relay = GETB(p_route_env->info, MM_ROUTE_BUF_INFO_RELAY);

            // Delay status transmission
            send_status = false;
        }

        if (GETB(p_env_lvl->env.info, MM_TB_STATE_MDL_INFO_MAIN))
        {
            // Update target state
            p_env_lvl->tgt_level = level;

            // Inform the Binding Manager about new transition
            mm_tb_bind_trans_new(p_env_lvl->env.grp_lid, MM_TRANS_TYPE_CLASSIC,
                                 trans_time, delay);
        }
        else
        {
            // Inform the Binding Manager
            mm_tb_bind_trans_req(p_env_lvl->env.grp_lid, p_env_lvl->env.mdl_lid,
                                 MM_TRANS_TYPE_CLASSIC, level, trans_time, delay);
        }
    }
    while (0);

    return (send_status);
}

/**
 ****************************************************************************************
 * @brief Handler for Generic Delta Set/Set Unacknowledged message
 *
 * @param[in] p_env         Pointer to environment of model for which message has been received
 * @param[in] p_buf         Pointer to the buffer containing the received message
 * @param[in] p_route_env   Pointer to structure containing reception parameters provided
 * by the Mesh Profile block
 ****************************************************************************************
 */
__STATIC bool mm_gens_lvl_handler_set_delta(mm_gens_lvl_env_t *p_env_lvl, mesh_tb_buf_t *p_buf,
        mm_route_buf_env_t *p_route_env)
{
    // Boolean indicating if Generic Level Status can be sent now
    bool send_status = (p_route_env->opcode == MM_MSG_GEN_DELTA_SET);

    do
    {
        // Get pointer to data
        uint8_t *p_data = MESH_TB_BUF_DATA(p_buf);
        // Extract required delta value
        int32_t delta = (int32_t)co_read32p(p_data + MM_GEN_LVL_SET_DELTA_LVL_POS);
        // Extract TID value
        uint8_t tid = *(p_data + MM_GEN_LVL_SET_DELTA_TID_POS);
        // Transition time
        uint8_t trans_time = MM_TRANS_TIME_UNKNOWN;
        // Delay
        uint8_t delay = 0;
        // Indicate if transaction is a new one
        uint8_t new_transaction;

        // Extract optional value if present
        if (p_buf->data_len == MM_GEN_LVL_SET_DELTA_LEN)
        {
            trans_time = (uint16_t)(*(p_data + MM_GEN_LVL_SET_DELTA_TRANS_TIME_POS));

            // Check received value
            if (GETF(trans_time, MM_TRANS_TIME_STEP_NB) > MM_TRANS_TIME_NB_STEPS_MAX)
            {
                // Drop the message
                send_status = false;
                break;
            }

            delay = *(p_data + MM_GEN_LVL_SET_DELTA_DELAY_POS);
        }

        if (delta == 0)
        {
            break;
        }

        // Check if transaction is a new transaction or if a new one must be started
        new_transaction = mm_gens_lvl_is_new_transaction(p_env_lvl, p_route_env->u_addr.src, tid);

        // Keep information for transmission of status if needed
        if (send_status)
        {
            p_env_lvl->status_dst_addr = p_route_env->u_addr.src;
            p_env_lvl->status_app_key_lid = p_route_env->app_key_lid;
            p_env_lvl->status_relay = GETB(p_route_env->info, MM_ROUTE_BUF_INFO_RELAY);

            send_status = false;
        }

        p_env_lvl->delta_param.src = p_route_env->u_addr.src;
        p_env_lvl->delta_param.tid = tid;

        if (GETB(p_env_lvl->env.info, MM_TB_STATE_MDL_INFO_MAIN))
        {
            // If transition is a new transition, keep the current level
            if (new_transaction)
            {
                p_env_lvl->u.init_level = p_env_lvl->level;
            }

            // Update target state
            p_env_lvl->tgt_level = mm_gens_lvl_compute_tgt_delta(p_env_lvl, delta);

            // Inform the Binding Manager about new transition
            mm_tb_bind_trans_new(p_env_lvl->env.grp_lid, MM_TRANS_TYPE_DELTA,
                                 trans_time, delay);
        }
        else
        {
            // Inform the Binding Manager
            mm_tb_bind_trans_req(p_env_lvl->env.grp_lid, p_env_lvl->env.mdl_lid,
                                 MM_TRANS_TYPE_DELTA, delta, trans_time, delay);
        }
    }
    while (0);

    return (send_status);
}

/**
 ****************************************************************************************
 * @brief Handler for Generic Move Set/Set Unacknowledged message
 *
 * @param[in] p_env         Pointer to environment of model for which message has been received
 * @param[in] p_buf         Pointer to the buffer containing the received message
 * @param[in] p_route_env   Pointer to structure containing reception parameters provided
 * by the Mesh Profile block
 ****************************************************************************************
 */
__STATIC bool mm_gens_lvl_handler_set_move(mm_gens_lvl_env_t *p_env_lvl, mesh_tb_buf_t *p_buf,
        mm_route_buf_env_t *p_route_env)
{
    // Boolean indicating if Generic Level Status can be sent now
    bool send_status = (p_route_env->opcode == MM_MSG_GEN_MOVE_SET);

    do
    {
        // Get pointer to data
        uint8_t *p_data = MESH_TB_BUF_DATA(p_buf);
        // Extract required delta value
        int16_t delta = (int16_t)co_read16p(p_data + MM_GEN_LVL_SET_MOVE_DELTA_LVL_POS);
        // Extract TID value
        uint8_t tid = *(p_data + MM_GEN_LVL_SET_MOVE_TID_POS);
        // Transition time
        uint8_t trans_time = MM_TRANS_TIME_UNKNOWN;
        // Delay
        uint8_t delay = 0;

        // Extract and check optional parameters if present
        if (p_buf->data_len == MM_GEN_LVL_SET_MOVE_LEN)
        {
            trans_time = (uint16_t)(*(p_data + MM_GEN_LVL_SET_MOVE_TRANS_TIME_POS));

            // Check received value
            if (GETF(trans_time, MM_TRANS_TIME_STEP_NB) > MM_TRANS_TIME_NB_STEPS_MAX)
            {
                // Drop the message
                send_status = false;
                break;
            }

            delay = *(p_data + MM_GEN_LVL_SET_MOVE_DELAY_POS);
        }

        // Check if received message is a retransmitted one, if state is modified and if
        // a new transition can be started now
        if ((p_env_lvl->status_dst_addr != MESH_UNASSIGNED_ADDR)
                || mm_tb_replay_is_retx(&p_env_lvl->replay_env, p_route_env->u_addr.src, tid))
        {
            if (send_status)
            {
                // Send Generic Level status message
                mm_gens_lvl_send_status(p_env_lvl, p_route_env, false);
            }
            break;
        }

        // Keep information for transmission of status if needed
        if (send_status)
        {
            p_env_lvl->status_dst_addr = p_route_env->u_addr.src;
            p_env_lvl->status_app_key_lid = p_route_env->app_key_lid;
            p_env_lvl->status_relay = GETB(p_route_env->info, MM_ROUTE_BUF_INFO_RELAY);

            send_status = false;
        }

        p_env_lvl->move_param.delta = delta;
        p_env_lvl->move_param.trans_time = trans_time;

        if (GETB(p_env_lvl->env.info, MM_TB_STATE_MDL_INFO_MAIN))
        {
            if (delta != 0)
            {
                // Move target level
                int16_t tgt_move_level = (delta > 0)
                                         ? MM_GENS_LVL_UPPER_LIMIT : MM_GENS_LVL_LOWER_LIMIT;

                if (p_env_lvl->level != p_env_lvl->u.tgt_move_level)
                {
                    // Update target state
                    p_env_lvl->u.tgt_move_level = tgt_move_level;
                    p_env_lvl->tgt_level = p_env_lvl->level + delta;

                    // Inform the Binding Manager about new transition
                    if (mm_tb_bind_trans_new(p_env_lvl->env.grp_lid, MM_TRANS_TYPE_MOVE,
                                             trans_time, delay) == MESH_ERR_NO_ERROR)
                    {
                        break;
                    }
                }

                send_status = (p_route_env->opcode == MM_MSG_GEN_MOVE_SET);
                p_env_lvl->status_dst_addr = MESH_UNASSIGNED_ADDR;
            }
        }
        else
        {
            // Inform the Binding Manager
            mm_tb_bind_trans_req(p_env_lvl->env.grp_lid, p_env_lvl->env.mdl_lid,
                                 MM_TRANS_TYPE_MOVE, delta, trans_time, delay);
        }
    }
    while (0);

    return (send_status);
}

/*
 * INTERNAL CALLBACK FUNCTIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Callback function called when timer monitoring publication duration for
 * Generic Level Server model expires
 *
 * @param[in] p_env     Pointer to model environment for Generic Level Server model
 ****************************************************************************************
 */
__STATIC void mm_gens_lvl_cb_tmr_publi(void *p_env)
{
    // Get allocated environment
    mm_gens_lvl_env_t *p_env_lvl = (mm_gens_lvl_env_t *)p_env;

    if (p_env_lvl->publi_period_ms)
    {
        // Publish a Generic Level Status message
        mm_gens_lvl_publish(p_env_lvl);

        // Restart the timer
        mesh_tb_timer_set(&p_env_lvl->tmr_publi, p_env_lvl->publi_period_ms);
    }
}

/**
 ****************************************************************************************
 * @brief Inform Generic Level Server model about a received message
 *
 * @param[in] p_env         Pointer to the environment allocated for the Generic Level
 * Server model
 * @param[in] p_buf         Pointer to the buffer containing the received message
 * @param[in] p_route_env   Pointer to structure containing reception parameters provided
 * by the Mesh Profile block
 ****************************************************************************************
 */
__STATIC void mm_gens_lvl_cb_rx(mm_tb_state_mdl_env_t *p_env, mesh_tb_buf_t *p_buf,
                                mm_route_buf_env_t *p_route_env)
{
    // Boolean indicating if Generic Level Status message can be sent now
    bool send_status;
    // Cast the provided environment structure
    mm_gens_lvl_env_t *p_env_lvl = (mm_gens_lvl_env_t *)p_env;

    switch (p_route_env->opcode)
    {
        case (MM_MSG_GEN_LVL_SET):
        case (MM_MSG_GEN_LVL_SET_UNACK):
        {
            send_status = mm_gens_lvl_handler_set(p_env_lvl, p_buf, p_route_env);
        } break;

        case (MM_MSG_GEN_DELTA_SET):
        case (MM_MSG_GEN_DELTA_SET_UNACK):
        {
            send_status = mm_gens_lvl_handler_set_delta(p_env_lvl, p_buf, p_route_env);
        } break;

        case (MM_MSG_GEN_MOVE_SET):
        case (MM_MSG_GEN_MOVE_SET_UNACK):
        {
            send_status = mm_gens_lvl_handler_set_move(p_env_lvl, p_buf, p_route_env);
        } break;

        case (MM_MSG_GEN_LVL_GET):
        {
            send_status = true;
        } break;

        default:
        {
            send_status = false;
        } break;
    }

    if (send_status)
    {
        // Send a Generic Level Status message
        mm_gens_lvl_send_status(p_env_lvl, p_route_env, false);
    }
}

/**
 ****************************************************************************************
 * @brief Check if a given opcode is handled by the Generic Level Server model
 *
 * @param[in] p_env         Pointer to the environment allocated for the Generic Level
 * Server model
 * @param[in] opcode        Opcode to check
 *
 * @return An error status (@see enum mesh_err)
 ****************************************************************************************
 */
__STATIC uint16_t mm_gens_lvl_cb_opcode_check(mm_tb_state_mdl_env_t *p_env, uint32_t opcode)
{
    uint16_t status;

    if ((opcode == MM_MSG_GEN_LVL_GET)
            || (opcode == MM_MSG_GEN_LVL_SET)
            || (opcode == MM_MSG_GEN_LVL_SET_UNACK)
            || (opcode == MM_MSG_GEN_DELTA_SET)
            || (opcode == MM_MSG_GEN_DELTA_SET_UNACK)
            || (opcode == MM_MSG_GEN_MOVE_SET)
            || (opcode == MM_MSG_GEN_MOVE_SET_UNACK)
       )
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
 * @brief Inform Generic Level Server model about received publication parametersd
 *
 * @param[in] p_env         Pointer the the environment allocated for the Generic Level
 * Server model
 * @param[in] addr          Publication address
 * @param[in] period_ms     Publication period in milliseconds
 ****************************************************************************************
 */
__STATIC void mm_gens_lvl_cb_publish_param(mm_tb_state_mdl_env_t *p_env, uint16_t addr, uint32_t period_ms)
{
    // Inform the state manager about received publication parameters
    mm_tb_state_publish_param_ind((mm_tb_state_mdl_publi_env_t *)p_env, addr, period_ms);
}

/**
 ****************************************************************************************
 * @brief Set Generic Level state value
 *
 * @param[in] p_env         Pointer the the environment allocated for the Generic Level
 * Server model
 * @param[in] level         Generic Level state value
 *
 * @return An error status (@see enum mesh_err)
 ****************************************************************************************
 */
__STATIC uint16_t mm_gens_lvl_cb_set(mm_tb_state_mdl_env_t *p_env, uint16_t state_id, uint32_t state)
{
    // Returned status
    uint16_t status = MESH_ERR_NO_ERROR;

    if (GETB(p_env->info, MM_TB_STATE_MDL_INFO_MAIN))
    {
        // Get environment for the Generic Level Server model
        mm_gens_lvl_env_t *p_env_lvl = (mm_gens_lvl_env_t *)p_env;

        // Set state value
        p_env_lvl->level = state;
    }
    else
    {
        MESH_MODEL_PRINT_WARN("%s, command disallowed.\n", __func__);
        status = MESH_ERR_COMMAND_DISALLOWED;
    }

    return (status);
}

/*
 * GLOBAL FUNCTIONS
 ****************************************************************************************
 */

uint16_t mm_gens_lvl_register(uint8_t elmt_idx, m_lid_t *p_mdl_lid)
{
    // Register the model
    uint16_t status = m_api_register_model(MM_ID_GENS_LVL, elmt_idx, M_MDL_CONFIG_PUBLI_AUTH_BIT,
                                           &mm_route_cb, p_mdl_lid);

    if (status == MESH_ERR_NO_ERROR)
    {
        // Inform the Model State Manager about registered model
        status = mm_tb_state_register(elmt_idx, MM_ID_GENS_LVL, *p_mdl_lid,
                                      MM_TB_STATE_ROLE_SRV | MM_TB_STATE_CFG_CB_BIT,
                                      sizeof(mm_gens_lvl_env_t));

        if (status == MESH_ERR_NO_ERROR)
        {
            // Get allocated environment
            mm_gens_lvl_env_t *p_env_lvl = (mm_gens_lvl_env_t *)mm_tb_state_get_env(*p_mdl_lid);
            // Get server-specific callback functions
            mm_srv_cb_t *p_cb_srv = p_env_lvl->env.cb.u.p_cb_srv;

            // Prepare timer for publications
            p_env_lvl->tmr_publi.cb = mm_gens_lvl_cb_tmr_publi;
            p_env_lvl->tmr_publi.p_env = (void *)p_env_lvl;

            // Prepare environment for Replay Manager
            p_env_lvl->replay_env.delay_ms = MM_GENS_LVL_REPLAY_MS;

            // Set internal callback functions
            p_env_lvl->env.cb.cb_rx = mm_gens_lvl_cb_rx;
            p_env_lvl->env.cb.cb_opcode_check = mm_gens_lvl_cb_opcode_check;
            p_env_lvl->env.cb.cb_publish_param = mm_gens_lvl_cb_publish_param;
            p_cb_srv->cb_set = mm_gens_lvl_cb_set;

            // Inform application about registered model
            mm_api_send_register_ind(MM_ID_GENS_LVL, elmt_idx, *p_mdl_lid);
        }
        else
        {
            MESH_MODEL_PRINT_WARN("%s, Register the model State fail.\n", __func__);
        }
    }
    else
    {
        MESH_MODEL_PRINT_WARN("%s, Register the model fail.\n", __func__);
    }

    return (status);
}

/*
 * CALLBACK FUNCTIONS FOR BINDING MANAGER
 ****************************************************************************************
 */

void mm_gens_lvl_cb_grp_event(m_lid_t mdl_lid, uint8_t event, uint8_t info)
{
    // Get environment for Generic Level Server model
    mm_gens_lvl_env_t *p_env_lvl = (mm_gens_lvl_env_t *)mm_tb_state_get_env(mdl_lid);
    switch (event)
    {
        case (MESH_MDL_GRP_EVENT_TRANS_REJECTED):
        {
            // Send Generic Level Status message
            mm_gens_lvl_check_status_rsp(p_env_lvl);
        } break;

        case (MESH_MDL_GRP_EVENT_TRANS_DELAY_EXPIRED):
        {
            // Start the transition
            mm_tb_bind_trans_start(p_env_lvl->env.grp_lid);
        } break;

        case (MESH_MDL_GRP_EVENT_TRANS_IMMEDIATE):
        {
            p_env_lvl->level = p_env_lvl->tgt_level;
        } // no break;

        case (MESH_MDL_GRP_EVENT_TRANS_STARTED):
        {
            // Inform application about the update
            if (GETB(p_env_lvl->env.info, MM_TB_STATE_MDL_INFO_MAIN))
            {
                uint8_t trans_time = info;

                // Inform application about state update
                mm_api_send_srv_state_upd_ind(MM_STATE_GEN_LVL, p_env_lvl->env.elmt_idx,
                                              (uint16_t)p_env_lvl->tgt_level,
                                              (trans_time) ? mm_tb_get_trans_time_ms(trans_time) : 0);
            }

            // Check if a Generic Level Status message must be sent
            mm_gens_lvl_check_status_rsp(p_env_lvl);

            // Send a publication
            mm_gens_lvl_publish(p_env_lvl);
        } break;

        case (MESH_MDL_GRP_EVENT_TRANS_END):
        {
            // Transition type
            uint8_t trans_type = info;

            // Update current level value
            p_env_lvl->level = p_env_lvl->tgt_level;

            // Inform application about the update
            if (GETB(p_env_lvl->env.info, MM_TB_STATE_MDL_INFO_MAIN))
            {
                // Inform application about state update
                mm_api_send_srv_state_upd_ind(MM_STATE_GEN_LVL, p_env_lvl->env.elmt_idx,
                                              (uint16_t)p_env_lvl->level, 0);;
            }

            // Send a publication
            mm_gens_lvl_publish(p_env_lvl);

            // Check transition type
            if (trans_type == MM_TRANS_TYPE_MOVE)
            {
                // Continue transition to the target state
                if (GETB(p_env_lvl->env.info, MM_TB_STATE_MDL_INFO_MAIN))
                {
                    // Check if target level has been reached
                    if (p_env_lvl->level != p_env_lvl->u.tgt_move_level)
                    {
                        // Update target level
                        p_env_lvl->tgt_level = mm_gens_lvl_compute_tgt_move(p_env_lvl);

                        // Inform the Binding Manager about new transition
                        mm_tb_bind_trans_new(p_env_lvl->env.grp_lid, MM_TRANS_TYPE_MOVE,
                                             p_env_lvl->move_param.trans_time, 0);
                    }
                }
                else
                {
                    // Inform the Binding Manager
                    mm_tb_bind_trans_req(p_env_lvl->env.grp_lid, p_env_lvl->env.mdl_lid,
                                         MM_TRANS_TYPE_MOVE, p_env_lvl->move_param.delta,
                                         p_env_lvl->move_param.trans_time, 0);
                }
            }
        } break;

        // case (MESH_MDL_GRP_EVENT_TRANS_ABORTED):
        // case (MESH_MDL_GRP_EVENT_GROUP_FULL):
        default:
        {
        } break;
    }
}

void mm_gens_lvl_cb_set_state(m_lid_t mdl_lid, uint8_t type, uint32_t state)
{
    // Get environment for Generic Level Server model
    mm_gens_lvl_env_t *p_env_lvl = (mm_gens_lvl_env_t *)mm_tb_state_get_env(mdl_lid);

    // Sanity check
    ASSERT_INFO(!GETB(p_env_lvl->env.info, MM_TB_STATE_MDL_INFO_MAIN), mdl_lid, 0);

    if (type == MM_STATE_TYPE_CURRENT)
    {
        p_env_lvl->level = (int16_t)state;
    }
    else if (type == MM_STATE_TYPE_TARGET)
    {
        p_env_lvl->tgt_level = (int16_t)state;
    }
    else     // (type == MM_STATE_TYPE_TARGET_MOVE)
    {
        p_env_lvl->u.tgt_move_level = (int16_t)state;
    }
}
#endif //(BLE_MESH_MDL_GENS_LVL)
/// @} end of group
