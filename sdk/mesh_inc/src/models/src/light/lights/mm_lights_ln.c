/**
 ****************************************************************************************
 * @file mm_lights_ln.c
 *
 * @brief Mesh Model Light Lightness Server Module
 *
 * Copyright (C) RivieraWaves 2017-2019
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup MM_LIGHTS_LN
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "mm_lights_int.h"      // Mesh Model Lighting Module Internal Definitions


#if (BLE_MESH_MDL_LIGHTS_LN)
/*
 * DEFINES
 ****************************************************************************************
 */

/// Validity of information provided to the Replay Manager
#define MM_LIGHTS_LN_REPLAY_MS               (6000)

/*
 * MACROS
 ****************************************************************************************
 */

/// Convert Light Lightness Linear value into Light Lightness Actual value
/// LA = 65535 * sqrt(LL / 65535)
///    = sqrt(65535 * 65535) * sqrt(LL / 65535)
///    = sqrt(65535 * LL)
///    = sqrt(65536 * LL - LL)
///    = sqrt(2^16 * LL - LL)
#define MM_LIGHTS_LN_ACTUAL(linear)                     \
        (mm_lights_isqrt(((uint32_t)linear << 16) - linear))

/// Convert Light Lightness Actual value into Light Lightness Linear value
/// LL = Ceil(65535 * (LA / 65535)^2)
///    = Ceil(LA^2 / 65535)
///    = Floor((LA^2 + 65534) + 65535)
#define MM_LIGHTS_LN_LINEAR(actual)                     \
        ((((uint32_t)actual * actual) + 65534) / 65535)

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

/// Structure for Lighting Lightness Server model environment
typedef struct mm_lights_ln_env
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
    /// Light Lightness Actual state value
    uint16_t ln;
    /// Light Lightness Linear
    uint16_t ln_linear;
    /// Target value of Light Lightness Actual state value
    uint16_t ln_tgt;
    /// Target value of Light Lightness Linear state value
    uint16_t ln_tgt_linear;
    /// Light Lightness Last state value
    uint16_t ln_last;
    /// Light Lightness Default state value
    uint16_t ln_dflt;
    /// Light Lightness Range Min state value
    uint16_t ln_min;
    /// Light Lightness Range Max state value
    uint16_t ln_max;

    /// Source address of set message that has triggered last or current transition
    uint16_t status_dst_addr;
    /// Application key local index to be used for transmission of Light Lightness Status
    /// or Light Lightness Linear Status message
    m_lid_t status_app_key_lid;
    /// Relaying of sent Light Lightness Status or Light Lightness Linear Status message
    /// authorized
    bool status_relay;
    /// Send a Light Lightness Status or Light Lightness Linear Status message
    bool status_linear;
} mm_lights_ln_env_t;

/// Structure for Light Lightness Setup Server model environment
typedef struct mm_lights_lns_env
{
    /// Basic model environment - Must be first element in the structure - DO NOT MOVE
    mm_tb_state_mdl_env_t env;
    /// Pointer to environment of associated Light Lightness Server model
    mm_lights_ln_env_t *p_env_ln;
} mm_lights_lns_env_t;

/*
 * LOCAL FUNCTIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Prepare and send a Light Lightness Status or a Light Lightness Linear Status message.
 * Note that both messages have the same content.
 *
 * @param[in] p_env_ln          Pointer to Light Lightness Server model environment
 * @param[in] p_route_env       Pointer to structure containing reception information provided
 * by Mesh Profile for received request message
 * @param[in] publish           Indicate if message is sent as a publication (true) or as
 * a response to a received request (false)
 * @param[in] linear            True if Light Lightness Status must be sent, else False
 ****************************************************************************************
 */
__STATIC void mm_lights_ln_send_status(mm_lights_ln_env_t *p_env_ln,
                                       mm_route_buf_env_t *p_route_env, bool publish, bool linear)
{

    // Pointer to the buffer that will contain the message
    mesh_tb_buf_t *p_buf_status;
    // Remaining time
    uint8_t rem_time;
    // Transition type
    uint8_t trans_type;
    // Data length
    uint8_t data_length;
    uint16_t status;
    UNUSED(status);
    // Check if a transition has been started
    mm_tb_bind_get_trans_info(p_env_ln->env.grp_lid, &trans_type, &rem_time);


    // Deduce deduce data length
    data_length = (trans_type != MM_TRANS_TYPE_NONE)
                  ? MM_LIGHT_LN_STATUS_LEN : MM_LIGHT_LN_STATUS_MIN_LEN;

    if ((status = mm_route_buf_alloc(&p_buf_status, data_length)) == MESH_ERR_NO_ERROR)
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
            p_buf_env->app_key_lid = p_env_ln->status_app_key_lid;
            p_buf_env->u_addr.dst = p_env_ln->status_dst_addr;
            SETB(p_buf_env->info, MM_ROUTE_BUF_INFO_RELAY, p_env_ln->status_relay);
        }

        SETB(p_buf_env->info, MM_ROUTE_BUF_INFO_RX, 0);
        SETB(p_buf_env->info, MM_ROUTE_BUF_INFO_PUBLISH, publish);
        p_buf_env->mdl_lid = p_env_ln->env.mdl_lid;
        p_buf_env->opcode = (linear) ? MM_MSG_LIGHT_LN_LINEAR_STATUS : MM_MSG_LIGHT_LN_STATUS;

        // Fill the message
        co_write16p(p_data + MM_LIGHT_LN_STATUS_LIGHTNESS_POS,
                    (linear) ? p_env_ln->ln_linear : p_env_ln->ln);

        if (data_length == MM_LIGHT_LN_STATUS_LEN)
        {
            // Target state value
            uint16_t tgt;

            if (trans_type == MM_TRANS_TYPE_MOVE)
            {
                tgt = (p_env_ln->move_delta > 0) ? p_env_ln->ln_max
                      : p_env_ln->ln_min;

                if (linear)
                {
                    tgt = MM_LIGHTS_LN_LINEAR(tgt);
                }

                rem_time = MM_TRANS_TIME_UNKNOWN;
            }
            else
            {
                tgt = (linear) ? p_env_ln->ln_tgt_linear : p_env_ln->ln_tgt;
            }

            co_write16p(p_data + MM_LIGHT_LN_STATUS_TGT_LIGHTNESS_POS, tgt);
            *(p_data + MM_LIGHT_LN_STATUS_REM_TIME_POS) = rem_time;
        }

        MESH_MODEL_PRINT_DEBUG("%s p_env_ln->ln:%d,p_env_ln->ln_tgt:%d\r\n", __func__, p_env_ln->ln, p_env_ln->ln_tgt);
        // Send the message
        mm_route_send(p_buf_status);
    }
    else
    {
        MESH_MODEL_PRINT_ERR("%s  mm_route_buf_alloc error!!, status = 0x%x\r\n", __func__, status);
    }

}

/**
 ****************************************************************************************
 * @brief Prepare and send a Light Lightness Last Status message
 *
 * @param[in] p_env_ln          Pointer to Light Lightness Server model environment
 * @param[in] p_route_env       Pointer to structure containing reception information provided
 * by Mesh Profile for received request message
 ****************************************************************************************
 */
__STATIC void mm_lights_ln_send_status_last(mm_lights_ln_env_t *p_env_ln,
        mm_route_buf_env_t *p_route_env)
{
    // Pointer to the buffer that will contain the message
    mesh_tb_buf_t *p_buf_status;
    // Allocate a new buffer for the publication
    uint8_t status = mm_route_buf_alloc_status(&p_buf_status,
                     MM_LIGHT_LN_LAST_STATUS_LEN, p_route_env);

    if (status == MESH_ERR_NO_ERROR)
    {
        // Get pointer to data
        uint8_t *p_data = MESH_TB_BUF_DATA(p_buf_status);
        // Get pointer to environment
        mm_route_buf_env_t *p_buf_env = (mm_route_buf_env_t *)&p_buf_status->env;

        // Complete environment
        p_buf_env->mdl_lid = p_env_ln->env.mdl_lid;
        p_buf_env->opcode = MM_MSG_LIGHT_LN_LAST_STATUS;

        // Fill the message
        co_write16p(p_data + MM_LIGHT_LN_LAST_STATUS_LIGHTNESS_POS, p_env_ln->ln_last);

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
 * @brief Prepare and send a Light Lightness Default Status message
 *
 * @param[in] p_env_ln          Pointer to Light Lightness Server model environment
 * @param[in] p_route_env       Pointer to structure containing reception information provided
 * by Mesh Profile for received request message
 ****************************************************************************************
 */
__STATIC void mm_lights_ln_send_status_dflt(mm_lights_ln_env_t *p_env_ln,
        mm_route_buf_env_t *p_route_env)
{
    // Pointer to the buffer that will contain the message
    mesh_tb_buf_t *p_buf_status;
    // Allocate a new buffer for the publication
    uint8_t status = mm_route_buf_alloc_status(&p_buf_status,
                     MM_LIGHT_LN_DFLT_STATUS_LEN, p_route_env);

    if (status == MESH_ERR_NO_ERROR)
    {
        // Get pointer to data
        uint8_t *p_data = MESH_TB_BUF_DATA(p_buf_status);
        // Get pointer to environment
        mm_route_buf_env_t *p_buf_env = (mm_route_buf_env_t *)&p_buf_status->env;

        // Complete environment
        p_buf_env->mdl_lid = p_env_ln->env.mdl_lid;
        p_buf_env->opcode = MM_MSG_LIGHT_LN_DFLT_STATUS;

        // Fill the message
        co_write16p(p_data + MM_LIGHT_LN_DFLT_STATUS_LIGHTNESS_POS, p_env_ln->ln_dflt);

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
 * @brief Prepare and send a Light Lightness Range Status message
 *
 * @param[in] p_env_ln          Pointer to Light Lightness Level Server model environment
 * @param[in] p_route_env       Pointer to structure containing reception information provided
 * by Mesh Profile for received request message
 * @param[in] status            Status sent in the Light Lightness Range Status message
 * (@see enum mm_status)
 ****************************************************************************************
 */
__STATIC void mm_lights_ln_send_status_range(mm_lights_ln_env_t *p_env_ln,
        mm_route_buf_env_t *p_route_env,
        uint8_t status)
{
    // Pointer to the buffer that will contain the message
    mesh_tb_buf_t *p_buf_status;
    // Allocate a new buffer for the publication
    uint16_t alloc_status = mm_route_buf_alloc_status(&p_buf_status,
                            MM_LIGHT_LN_RANGE_STATUS_LEN, p_route_env);

    if (alloc_status == MESH_ERR_NO_ERROR)
    {
        // Get pointer to data
        uint8_t *p_data = MESH_TB_BUF_DATA(p_buf_status);
        // Get pointer to environment
        mm_route_buf_env_t *p_buf_env = (mm_route_buf_env_t *)&p_buf_status->env;

        // Prepare environment
        p_buf_env->mdl_lid = p_env_ln->env.mdl_lid;
        p_buf_env->opcode = MM_MSG_LIGHT_LN_RANGE_STATUS;

        // Fill the message
        *(p_data + MM_LIGHT_LN_RANGE_STATUS_CODE_POS) = status;
        co_write16p(p_data + MM_LIGHT_LN_RANGE_STATUS_MIN_POS, p_env_ln->ln_min);
        co_write16p(p_data + MM_LIGHT_LN_RANGE_STATUS_MAX_POS, p_env_ln->ln_max);

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
 * @brief Publish Light Lightness Actual state value if sending of publications is enabled
 *
 * @param[in] p_env_ln         Pointer to Light Lightness Server model environment
 ****************************************************************************************
 */
__STATIC void mm_lights_ln_publish(mm_lights_ln_env_t *p_env_ln)
{
    // Check if sending of publication is enabled
    if (GETB(p_env_ln->env.info, MM_TB_STATE_MDL_INFO_PUBLI))
    {
        mm_lights_ln_send_status(p_env_ln, NULL, true, false);
    }
}

/**
 ****************************************************************************************
 * @brief Check if a Light Lightnes Status message must be sent and send it if it
 * is the case
 *
 * @param[in] p_env_ln        Pointer to Light Lightness Server model environment
 ****************************************************************************************
 */
__STATIC void mm_lights_ln_check_status_rsp(mm_lights_ln_env_t *p_env_ln)
{
    if (p_env_ln->status_dst_addr != MESH_UNASSIGNED_ADDR)
    {
        // Send a response to the node that has required the transition
        mm_lights_ln_send_status(p_env_ln, NULL, false, p_env_ln->status_linear);

        p_env_ln->status_dst_addr = MESH_UNASSIGNED_ADDR;
    }
}

/*
 * MESSAGE HANDLERS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Handler for Light Lightness Set/Set Unacknowledged and Light Lightness Linear
 * Set/Set Unacknowledged message.
 * Note that all messages have the same content.
 *
 * @param[in] p_env         Pointer to environment of model for which message has been received
 * @param[in] p_buf         Pointer to buffer containing the received message
 * @param[in] p_route_env   Pointer to routing information for the received buffer
 ****************************************************************************************
 */
__STATIC void mm_lights_ln_handler_set(mm_lights_ln_env_t *p_env_ln, mesh_tb_buf_t *p_buf,
                                       mm_route_buf_env_t *p_route_env)
{
    do
    {

        // Set Light Lightness Actual or Light Lightness Linear staet value
        bool linear = ((p_route_env->opcode == MM_MSG_LIGHT_LN_LINEAR_SET)
                       || (p_route_env->opcode == MM_MSG_LIGHT_LN_LINEAR_SET_UNACK));
        // Check if a status message must be sent
        bool send_status = ((p_route_env->opcode == MM_MSG_LIGHT_LN_LINEAR_SET)
                            || (p_route_env->opcode == MM_MSG_LIGHT_LN_SET));
        // Get pointer to data
        uint8_t *p_data = MESH_TB_BUF_DATA(p_buf);
        // Lightness Actual and Lightness Linear values
        uint16_t ln, ln_linear;
        // Extract TID value
        uint8_t tid = *(p_data + MM_LIGHT_LN_SET_TID_POS);
        // Transition time
        uint8_t trans_time = MM_TRANS_TIME_UNKNOWN;
        // Delay
        uint8_t delay = 0;


        // Extract and check optional parameters if present
        if (p_buf->data_len == MM_LIGHT_LN_SET_LEN)
        {
            trans_time = (uint16_t)(*(p_data + MM_LIGHT_LN_SET_TRANS_TIME_POS));

            // Check received value
            if (GETF(trans_time, MM_TRANS_TIME_STEP_NB) > MM_TRANS_TIME_NB_STEPS_MAX)
            {
                // Drop the message
                break;
            }

            delay = *(p_data + MM_LIGHT_LN_SET_DELAY_POS);
        }

        if (linear)
        {
            ln_linear = co_read16p(p_data + MM_LIGHT_LN_SET_LIGHTNESS_POS);
            ln = MM_LIGHTS_LN_ACTUAL(ln_linear);
        }
        else
        {
            ln = co_read16p(p_data + MM_LIGHT_LN_SET_LIGHTNESS_POS);
        }

        if (ln != 0)
        {
            // Ensure that Light Lightness Actual state value is between Light Lightness Range
            // Min and Max values
            if (ln > p_env_ln->ln_max)
            {
                ln = p_env_ln->ln_max;
            }
            else if (ln < p_env_ln->ln_min)
            {
                ln = p_env_ln->ln_min;
            }
        }


        // Check if Light Lightness Actual state is modified
        if ((MESH_UNASSIGNED_ADDR != p_env_ln->status_dst_addr)
                || mm_tb_replay_is_retx(&p_env_ln->replay_env, p_route_env->u_addr.src, tid)
                || (ln == p_env_ln->ln))
        {
            // Send a Light Lightness Status or a Light Lightness Linear Status message
            if (send_status)
            {
                mm_lights_ln_send_status(p_env_ln, p_route_env, false, linear);
            }
            break;
        }

        if (send_status)
        {
            // Keep information for transmission of status
            p_env_ln->status_dst_addr = p_route_env->u_addr.src;
            p_env_ln->status_app_key_lid = p_route_env->app_key_lid;
            p_env_ln->status_relay = GETB(p_route_env->info, MM_ROUTE_BUF_INFO_RELAY);
            p_env_ln->status_linear = linear;
        }

        // Update target state
        p_env_ln->ln_tgt = ln;
        p_env_ln->ln_tgt_linear = MM_LIGHTS_LN_LINEAR(ln);


        // Inform the Binding Manager about new transition
        mm_tb_bind_trans_new(p_env_ln->env.grp_lid, MM_TRANS_TYPE_CLASSIC, trans_time, delay);
    }
    while (0);
}

/**
 ****************************************************************************************
 * @brief Handler for Light Lightness Default Set/Set Unacknowledged message
 *
 * @param[in] p_env         Pointer to environment of model for which message has been received
 * @param[in] p_buf         Pointer to buffer containing the received message
 * @param[in] p_route_env   Pointer to routing information for the received buffer
 ****************************************************************************************
 */
__STATIC void mm_lights_ln_handler_set_dflt(mm_lights_ln_env_t *p_env_ln, mesh_tb_buf_t *p_buf,
        mm_route_buf_env_t *p_route_env)
{
    // Get pointer to data
    uint8_t *p_data = MESH_TB_BUF_DATA(p_buf);
    // Extract received state value
    uint16_t ln_dflt = co_read16p(p_data + MM_LIGHT_LN_DFLT_SET_LIGHTNESS_POS);

    if (ln_dflt != p_env_ln->ln_dflt)
    {
        // Keep received value
        p_env_ln->ln_dflt = ln_dflt;

        // Inform application about received value
        mm_api_send_srv_state_upd_ind(MM_STATE_LIGHT_LN_DFLT, p_env_ln->env.elmt_idx,
                                      p_env_ln->ln_dflt, 0);
    }

    // If needed, send a Light Lightness Default Status message to the requester
    if (p_route_env->opcode == MM_MSG_LIGHT_LN_DFLT_SET)
    {
        mm_lights_ln_send_status_dflt(p_env_ln, p_route_env);
    }
}

/**
 ****************************************************************************************
 * @brief Handler for Light Lightness Range Set/Set Unacknowledged message
 *
 * @param[in] p_env         Pointer to environment of model for which message has been received
 * @param[in] p_buf         Pointer to buffer containing the received message
 * @param[in] p_route_env   Pointer to routing information for the received buffer
 ****************************************************************************************
 */
__STATIC void mm_lights_ln_handler_set_range(mm_lights_ln_env_t *p_env_ln, mesh_tb_buf_t *p_buf,
        mm_route_buf_env_t *p_route_env)
{
    do
    {
        // Get pointer to data
        uint8_t *p_data = MESH_TB_BUF_DATA(p_buf);
        // Status
        uint8_t status = MM_STATUS_SUCCESS;
        // Extract Light Lightness Range state value
        uint16_t ln_min = co_read16p(p_data + MM_LIGHT_LN_RANGE_SET_MIN_POS);
        // Extract Light Lightness Range state value
        uint16_t ln_max = co_read16p(p_data + MM_LIGHT_LN_RANGE_SET_MAX_POS);

        // Check provided values
        if (ln_min == 0)
        {
            status = MM_STATUS_CANNOT_SET_RANGE_MIN;
        }
        else if (ln_max == 0)
        {
            status = MM_STATUS_CANNOT_SET_RANGE_MAX;
        }
        else if (ln_min > ln_max)
        {
            // Drop the message
            break;
        }

        if ((status == MM_STATUS_SUCCESS)
                && ((p_env_ln->ln_min != ln_min)
                    || (p_env_ln->ln_max != ln_max)))
        {
            p_env_ln->ln_min = ln_min;
            p_env_ln->ln_max = ln_max;

            // Inform application about received value
            mm_api_send_srv_state_upd_ind(MM_STATE_LIGHT_LN_RANGE, p_env_ln->env.elmt_idx,
                                          (uint32_t)ln_min | ((uint32_t)ln_max << 16), 0);
        }

        // If needed, send a Light Lightness Range Status message to the requester
        if (p_route_env->opcode == MM_MSG_LIGHT_LN_RANGE_SET)
        {
            mm_lights_ln_send_status_range(p_env_ln, p_route_env, status);
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
 * Light Lightness Server model expires
 *
 * @param[in] p_env     Pointer to model environment for Light Lightness Server model
 ****************************************************************************************
 */
__STATIC void mm_lights_ln_cb_tmr_publi(void *p_env)
{
    // Get allocated environment
    mm_lights_ln_env_t *p_env_ln = (mm_lights_ln_env_t *)p_env;

    if (p_env_ln->publi_period_ms)
    {
        // Publish a Light Lightness Status message
        mm_lights_ln_publish(p_env_ln);

        // Restart the timer
        mesh_tb_timer_set(&p_env_ln->tmr_publi, p_env_ln->publi_period_ms);
    }
}

/**
 ****************************************************************************************
 * @brief Inform Light Lightness Server model about a received message
 *
 * @param[in] p_env         Pointer to the environment allocated for the Light Lightness
 * Server model
 * @param[in] p_buf         Pointer to the buffer containing the received message
 * @param[in] p_route_env   Pointer to structure containing reception parameters provided
 * by the Mesh Profile block
 ****************************************************************************************
 */
__STATIC void mm_lights_ln_cb_rx(mm_tb_state_mdl_env_t *p_env, mesh_tb_buf_t *p_buf,
                                 mm_route_buf_env_t *p_route_env)
{
    do
    {
        {
            MESH_MODEL_PRINT_DEBUG("%s opcode :0x%x\r\n", __func__, p_route_env->opcode);
            uint8_t *p_data = MESH_TB_BUF_DATA(p_buf);
            MESH_MODEL_PRINT_DEBUG("###############################\r\n");
            MESH_MODEL_PRINT_DEBUG("data: %s \n", mesh_buffer_to_hex(p_data, p_buf->data_len));
            MESH_MODEL_PRINT_DEBUG("###############################\r\n");
        }
        // Environment for Light Lightness Server model
        mm_lights_ln_env_t *p_env_ln;

        if (p_env->mdl_id == MM_ID_LIGHTS_LN)
        {
            p_env_ln = (mm_lights_ln_env_t *)p_env;

            switch (p_route_env->opcode)
            {
                case (MM_MSG_LIGHT_LN_GET):
                case (MM_MSG_LIGHT_LN_LINEAR_GET):
                {
                    // Send a Light Lightness Status or a Light Lightness Linear Status message
                    mm_lights_ln_send_status(p_env_ln, p_route_env, false,
                                             (p_route_env->opcode == MM_MSG_LIGHT_LN_LINEAR_GET));
                } break;

                case (MM_MSG_LIGHT_LN_SET):
                case (MM_MSG_LIGHT_LN_SET_UNACK):
                case (MM_MSG_LIGHT_LN_LINEAR_SET):
                case (MM_MSG_LIGHT_LN_LINEAR_SET_UNACK):
                {
                    // Handle the message
                    mm_lights_ln_handler_set(p_env_ln, p_buf, p_route_env);
                } break;

                case (MM_MSG_LIGHT_LN_LAST_GET):
                {
                    // Send a Light Lightness Last Status message
                    mm_lights_ln_send_status_last(p_env_ln, p_route_env);
                } break;

                case (MM_MSG_LIGHT_LN_DFLT_GET):
                {
                    // Send a Light Lightness Default Status message
                    mm_lights_ln_send_status_dflt(p_env_ln, p_route_env);
                } break;

                case (MM_MSG_LIGHT_LN_RANGE_GET):
                {
                    // Send a Light Lightness Range Status message
                    mm_lights_ln_send_status_range(p_env_ln, p_route_env, MM_STATUS_SUCCESS);
                } break;

                default:
                {
                } break;
            }
        }
        else     // (p_env->mdl_id == MM_ID_LIGHTS_LNS)
        {
            // Environment for Light Lightness Setup Server model
            mm_lights_lns_env_t *p_env_lns = (mm_lights_lns_env_t *)p_env;
            p_env_ln = p_env_lns->p_env_ln;

            switch (p_route_env->opcode)
            {
                case (MM_MSG_LIGHT_LN_DFLT_SET):
                case (MM_MSG_LIGHT_LN_DFLT_SET_UNACK):
                {
                    // Handle the message
                    mm_lights_ln_handler_set_dflt(p_env_ln, p_buf, p_route_env);
                } break;

                case (MM_MSG_LIGHT_LN_RANGE_SET):
                case (MM_MSG_LIGHT_LN_RANGE_SET_UNACK):
                {
                    // Handle the message
                    mm_lights_ln_handler_set_range(p_env_ln, p_buf, p_route_env);
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
 * @brief Check if a given opcode is handled by the Light Lightness Server model
 *
 * @param[in] p_env         Pointer to the environment allocated for the Light Lightness
 * Server model
 * @param[in] opcode        Opcode to check
 *
 * @return An error status (@see enum mesh_err)
 ****************************************************************************************
 */
__STATIC uint16_t mm_lights_ln_cb_opcode_check(mm_tb_state_mdl_env_t *p_env, uint32_t opcode)
{
    uint16_t status = MESH_ERR_MDL_INVALID_OPCODE;

    if (p_env->mdl_id == MM_ID_LIGHTS_LN)
    {
        if ((opcode == MM_MSG_LIGHT_LN_GET)
                || (opcode == MM_MSG_LIGHT_LN_SET)
                || (opcode == MM_MSG_LIGHT_LN_SET_UNACK)
                || (opcode == MM_MSG_LIGHT_LN_LINEAR_GET)
                || (opcode == MM_MSG_LIGHT_LN_LINEAR_SET)
                || (opcode == MM_MSG_LIGHT_LN_LINEAR_SET_UNACK)
                || (opcode == MM_MSG_LIGHT_LN_LAST_GET)
                || (opcode == MM_MSG_LIGHT_LN_DFLT_GET)
                || (opcode == MM_MSG_LIGHT_LN_RANGE_GET))
        {
            status = MESH_ERR_NO_ERROR;
        }
    }
    else if (p_env->mdl_id == MM_ID_LIGHTS_LNS)
    {
        if ((opcode == MM_MSG_LIGHT_LN_DFLT_SET)
                || (opcode == MM_MSG_LIGHT_LN_DFLT_SET_UNACK)
                || (opcode == MM_MSG_LIGHT_LN_RANGE_SET)
                || (opcode == MM_MSG_LIGHT_LN_RANGE_SET_UNACK))
        {
            status = MESH_ERR_NO_ERROR;
        }
    }

    if (status != MESH_ERR_NO_ERROR)
    {
        MESH_MODEL_PRINT_INFO("%s, Invalid opcode (0x%x).\n", __func__, opcode);
    }

    return (status);
}

/**
 ****************************************************************************************
 * @brief Inform Light Lightness Server model about received publication parameters.
 *
 * @param[in] p_env         Pointer the the environment allocated for the Light Lightness
 * Server model
 * @param[in] addr          Publication address
 * @param[in] period_ms     Publication period in milliseconds
 ****************************************************************************************
 */
__STATIC void mm_lights_ln_cb_publish_param(mm_tb_state_mdl_env_t *p_env, uint16_t addr,
        uint32_t period_ms)
{
    // Inform the state manager about received publication parameters
    mm_tb_state_publish_param_ind((mm_tb_state_mdl_publi_env_t *)p_env, addr, period_ms);
}

/**
 ****************************************************************************************
 * @brief Set Light Lightness Actual or Light Lightness Default or Light Lightness Range
 * state value
 *
 * @param[in] p_env         Pointer the the environment allocated for the Generic OnOff
 * Server model
 * @param[in] state_id      State identifier
 * @param[in] state         Light Lightness Actual or Light Lightness Default or Light
 * Lightness Range state value
 *
 * @return An error status (@see enum mesh_err)
 ****************************************************************************************
 */
__STATIC uint16_t mm_lights_ln_cb_set(mm_tb_state_mdl_env_t *p_env, uint16_t state_id, uint32_t state)
{
    // Returned status
    uint16_t status = MESH_ERR_NO_ERROR;
    // Get environment for the Light Lightness Server model
    mm_lights_ln_env_t *p_env_ln = (mm_lights_ln_env_t *)p_env;

    switch (state_id)
    {
        case (MM_STATE_LIGHT_LN):
        {
            uint16_t ln = state;

            p_env_ln->ln = ln;
            p_env_ln->ln_linear = MM_LIGHTS_LN_LINEAR(ln);

            if (ln != 0)
            {
                p_env_ln->ln_last = ln;
            }

            // Set the targeted Generic OnOff state value
            mm_tb_bind_set_state(p_env_ln->env.grp_lid, MM_STATE_TYPE_TARGET, MM_ID_GENS_OO,
                                 (p_env_ln->ln == 0) ? 0 : 1);

            // Set the targeted Generic Level state value
            mm_tb_bind_set_state(p_env_ln->env.grp_lid, MM_STATE_TYPE_TARGET, MM_ID_GENS_LVL,
                                 p_env_ln->ln - 32768);
        } break;

        case (MM_STATE_LIGHT_LN_DFLT):
        {
            p_env_ln->ln_dflt = state;
        } break;

        case (MM_STATE_LIGHT_LN_RANGE):
        {
            p_env_ln->ln_min = state;
            p_env_ln->ln_max = (state >> 16);
        } break;

        default:
        {
            status = MESH_ERR_INVALID_PARAM;
        } break;
    }

    return (status);
}

/*
 * GLOBAL FUNCTIONS
 ****************************************************************************************
 */

uint16_t mm_lights_ln_register(uint8_t elmt_idx, m_lid_t *p_mdl_lid)
{
    uint16_t status;

    do
    {
        // Pointer to allocated environment for Light Lightness Server model
        mm_lights_ln_env_t *p_env_ln;
        // Model local index
        m_lid_t mdl_lid;
        // Pointer to server-specific callback functions
        mm_srv_cb_t *p_cb_srv;

        // Register Light Lightness Server model
        status = m_api_register_model(MM_ID_LIGHTS_LN, elmt_idx, M_MDL_CONFIG_PUBLI_AUTH_BIT,
                                      &mm_route_cb, p_mdl_lid);

        if (status != MESH_ERR_NO_ERROR)
        {
            MESH_MODEL_PRINT_WARN("%s, Register Light Lightness Server model fail. \n", __func__);
            break;
        }

        // Inform the Model State Manager about registered model
        status = mm_tb_state_register(elmt_idx, MM_ID_LIGHTS_LN, *p_mdl_lid,
                                      MM_TB_STATE_ROLE_SRV | MM_TB_STATE_CFG_CB_BIT,
                                      sizeof(mm_lights_ln_env_t));

        if (status != MESH_ERR_NO_ERROR)
        {
            MESH_MODEL_PRINT_WARN("%s, Register Light Lightness Server model State fail. \n", __func__);
            break;
        }

        // Get allocated environment
        p_env_ln = (mm_lights_ln_env_t *)mm_tb_state_get_env(*p_mdl_lid);

        // Get server-specific callback functions
        p_cb_srv = p_env_ln->env.cb.u.p_cb_srv;

        // Initiate states
        p_env_ln->ln_min = 1;
        p_env_ln->ln_max = 0xFFFF;

        // p_env_ln->ln_dflt = 0;

        p_env_ln->ln_last = 0x8000;

        p_env_ln->ln = 0x8000;

        // Prepare environment for Replay Manager
        p_env_ln->replay_env.delay_ms = MM_LIGHTS_LN_REPLAY_MS;

        // Prepare timer for publications
        p_env_ln->tmr_publi.cb = mm_lights_ln_cb_tmr_publi;
        p_env_ln->tmr_publi.p_env = (void *)p_env_ln;

        // Set internal callback functions
        p_env_ln->env.cb.cb_rx = mm_lights_ln_cb_rx;
        p_env_ln->env.cb.cb_opcode_check = mm_lights_ln_cb_opcode_check;
        p_env_ln->env.cb.cb_publish_param = mm_lights_ln_cb_publish_param;
        p_cb_srv->cb_set = mm_lights_ln_cb_set;

        // Inform application about registered model
        mm_api_send_register_ind(MM_ID_LIGHTS_LN, elmt_idx, *p_mdl_lid);

        // Register Light Lightness Setup Server model
        status = m_api_register_model(MM_ID_LIGHTS_LNS, elmt_idx, 0, &mm_route_cb, &mdl_lid);

        if (status != MESH_ERR_NO_ERROR)
        {
            MESH_MODEL_PRINT_WARN("%s, Register Light Lightness Setup Server model fail. \n", __func__);
            break;
        }

        // Inform the Model State Manager about registered model
        status = mm_tb_state_register(elmt_idx, MM_ID_LIGHTS_LNS, mdl_lid,
                                      MM_TB_STATE_ROLE_SRV, sizeof(mm_lights_lns_env_t));

        if (status == MESH_ERR_NO_ERROR)
        {
            // Get allocated environment
            mm_lights_lns_env_t *p_env_lns = (mm_lights_lns_env_t *)mm_tb_state_get_env(mdl_lid);

            // Set internal callback functions
            p_env_lns->env.cb.cb_rx = mm_lights_ln_cb_rx;
            p_env_lns->env.cb.cb_opcode_check = mm_lights_ln_cb_opcode_check;

            // Link environment
            p_env_lns->p_env_ln = p_env_ln;

            // Inform application about registered model
            mm_api_send_register_ind(MM_ID_LIGHTS_LNS, elmt_idx, mdl_lid);
        }
        else
        {
            MESH_MODEL_PRINT_WARN("%s, Register Light Lightness Setup Server model State fail. \n", __func__);
        }
    }
    while (0);

    return (status);
}

/*
 * CALLBACK FUNCTIONS FOR BINDING MANAGER
 ****************************************************************************************
 */


void mm_lights_ln_cb_grp_event(m_lid_t mdl_lid, uint8_t event, uint8_t info)
{
    // Get environment for Light Lightness Server model
    mm_lights_ln_env_t *p_env_ln = (mm_lights_ln_env_t *)mm_tb_state_get_env(mdl_lid);

    MESH_MODEL_PRINT_DEBUG("%s event:%x\r\n", __func__, event);
    switch (event)
    {
        case (MESH_MDL_GRP_EVENT_TRANS_DELAY_EXPIRED):
        {


            // Set the targeted Generic OnOff state value
            mm_tb_bind_set_state(p_env_ln->env.grp_lid, MM_STATE_TYPE_TARGET, MM_ID_GENS_OO,
                                 (p_env_ln->ln_tgt == 0) ? 0 : 1);

            // Set the targeted Generic Level state value
            mm_tb_bind_set_state(p_env_ln->env.grp_lid, MM_STATE_TYPE_TARGET, MM_ID_GENS_LVL,
                                 (int32_t)p_env_ln->ln_tgt - 32768);

            // Start the transition
            mm_tb_bind_trans_start(p_env_ln->env.grp_lid);
        } break;

        //case (MESH_MDL_GRP_EVENT_TRANS_STARTED):
        case (MESH_MDL_GRP_EVENT_TRANS_IMMEDIATE):
        {
            p_env_ln->ln = p_env_ln->ln_tgt;
            p_env_ln->ln_linear = p_env_ln->ln_tgt_linear;

            if (p_env_ln->ln != 0)
            {
                p_env_ln->ln_last = p_env_ln->ln;
            }
        } // no break;


        case (MESH_MDL_GRP_EVENT_TRANS_STARTED):
        {
            uint8_t trans_time = info;

            if (GETB(p_env_ln->env.info, MM_TB_STATE_MDL_INFO_MAIN))
            {
                // Inform application about state update
                mm_api_send_srv_state_upd_ind(MM_STATE_LIGHT_LN, p_env_ln->env.elmt_idx,
                                              p_env_ln->ln_tgt,
                                              (trans_time) ? mm_tb_get_trans_time_ms(trans_time) : 0);
            }

            // Check if a status message must be sent
            mm_lights_ln_check_status_rsp(p_env_ln);

            // Send a publication
            mm_lights_ln_publish(p_env_ln);
        } break;

        case (MESH_MDL_GRP_EVENT_TRANS_END):
        {
            p_env_ln->ln = p_env_ln->ln_tgt;
            p_env_ln->ln_linear = p_env_ln->ln_tgt_linear;

            if (GETB(p_env_ln->env.info, MM_TB_STATE_MDL_INFO_MAIN))
            {
                // Inform application about state update
                mm_api_send_srv_state_upd_ind(MM_STATE_LIGHT_LN, p_env_ln->env.elmt_idx, p_env_ln->ln, 0);
            }

            if (p_env_ln->ln != 0)
            {
                p_env_ln->ln_last = p_env_ln->ln;
            }

            // Check if a status message must be sent
            mm_lights_ln_check_status_rsp(p_env_ln);

            // Send a publication
            mm_lights_ln_publish(p_env_ln);
        } break;

        case (MESH_MDL_GRP_EVENT_GROUP_FULL):
        {
            // Set Generic Level state value
            mm_tb_bind_set_state(p_env_ln->env.grp_lid, MM_STATE_TYPE_CURRENT,
                                 MM_ID_GENS_LVL, 0x8000);
        } break;

        //case (MESH_MDL_GRP_EVENT_TRANS_ABORTED):
        default:
        {
        } break;
    }
}

void mm_lights_ln_cb_trans_req(m_lid_t main_mdl_lid, uint32_t req_model_id, uint8_t trans_type,
                               uint32_t state_delta)
{

    // Get environment for Light Lightness Server model
    mm_lights_ln_env_t *p_env_ln = (mm_lights_ln_env_t *)mm_tb_state_get_env(main_mdl_lid);
    // Targeted Light Lightness Actual state value
    uint16_t ln_tgt;

    MESH_MODEL_PRINT_DEBUG("%s main_mdl_lid:%d,req_model_id:0x%x\r\n", __func__, main_mdl_lid, req_model_id);

    if (req_model_id == MM_ID_GENS_OO)
    {
        // Requested Generic OnOff state value
        uint8_t onoff = (uint8_t)state_delta;

        if (onoff == 0)
        {
            ln_tgt = 0;
        }
        else
        {
            ln_tgt =  (p_env_ln->ln_dflt == 0) ? p_env_ln->ln_last
                      : p_env_ln->ln_dflt;
        }

        MESH_MODEL_PRINT_DEBUG("onoff:%d,ln_tgt:%d,p_env_ln->ln:%d\r\n", onoff, ln_tgt, p_env_ln->ln);
    }
    else     // req_model_id == MM_ID_GENS_LVL
    {
        if (trans_type == MM_TRANS_TYPE_CLASSIC)
        {
            // Requested Generic Level state value
            int16_t level = (int16_t)state_delta;

            // Light Lightness Actual = Generic Level + 32768
            ln_tgt = 32768 + level;
        }
        else     // ((trans_type == MM_TRANS_TYPE_DELTA) || trans_type == MM_TRANS_TYPE_MOVE))
        {
            // Delta value
            int32_t delta;

            if (trans_type == MM_TRANS_TYPE_MOVE)
            {
                delta = (int16_t)state_delta;

                // Keep the provided delta value
                p_env_ln->move_delta = (int16_t)state_delta;
            }
            else
            {
                delta = (int32_t)state_delta;
            }

            // Add the Light Lightness Actual state value to the received delta value
            delta += p_env_ln->ln;

            // The Light Lightness Actual state value cannot wrap
            if (delta < 0)
            {
                ln_tgt = 0;
            }
            else
            {
                ln_tgt = (delta > 0xFFFF) ? 0xFFFF : (uint16_t)delta;
            }
        }
    }

    if (ln_tgt != 0)
    {
        // Ensure that Light Lightness Actual state value is between Light Lightness Range
        // Min and Max values
        if (ln_tgt > p_env_ln->ln_max)
        {
            ln_tgt = p_env_ln->ln_max;
        }
        else if (ln_tgt < p_env_ln->ln_min)
        {
            ln_tgt = p_env_ln->ln_min;
        }
    }

    // Check if Light Lightness Actual state value is modified
    if (ln_tgt != p_env_ln->ln)
    {
        p_env_ln->ln_tgt = ln_tgt;
        p_env_ln->ln_tgt_linear = MM_LIGHTS_LN_LINEAR(ln_tgt);

        // Start a new transition
        mm_tb_bind_trans_new(p_env_ln->env.grp_lid, trans_type, 0, 0);
    }
    else
    {
        // Reject the transition
        mm_tb_bind_trans_reject(p_env_ln->env.grp_lid);
    }
}

void mm_lights_ln_cb_set_state(m_lid_t mdl_lid, uint8_t type, uint32_t state)
{
    // Get environment for Light Lightness Server model
    mm_lights_ln_env_t *p_env_ln = (mm_lights_ln_env_t *)mm_tb_state_get_env(mdl_lid);
    // Light Lightness state value
    uint16_t ln = state;
    // Generic Level = Light Lightness - 32768
    int16_t lvl = (int32_t)ln - 32768;
    // Sanity check
    ASSERT_INFO(!GETB(p_env_ln->env.info, MM_TB_STATE_MDL_INFO_MAIN), mdl_lid, 0);
    MESH_MODEL_PRINT_DEBUG("%s mdl_lid:%d,type:%d,state:%d\r\n", __func__, mdl_lid, type, state);
    if (type == MM_STATE_TYPE_CURRENT)
    {
        p_env_ln->ln = ln;
        p_env_ln->ln_linear = MM_LIGHTS_LN_LINEAR(ln);
    }
    else     // (type == MM_STATE_TYPE_TARGET)
    {
        p_env_ln->ln_tgt = ln;
        p_env_ln->ln_tgt_linear = MM_LIGHTS_LN_LINEAR(ln);
    }

    // Set the Generic Level state (current or target) value
    mm_tb_bind_set_state(p_env_ln->env.grp_lid, type, MM_ID_GENS_LVL, lvl);
}

uint16_t mm_lights_ln_get(uint8_t elmt_idx, uint32_t state_id)
{
    // Get model local index of Light Lightness Server model on given element
    m_lid_t mdl_lid = mm_tb_state_get_lid(elmt_idx, MM_ID_LIGHTS_LN);
    // State
    uint16_t state = 0;

    if (mdl_lid != MESH_INVALID_LID)
    {
        // Get environment for the model
        mm_lights_ln_env_t *p_env_ln = (mm_lights_ln_env_t *)mm_tb_state_get_env(mdl_lid);

        switch (state_id)
        {
            case (MM_STATE_LIGHT_LN_DFLT):
            {
                state = p_env_ln->ln_dflt;
            } break;

            case (MM_STATE_LIGHT_LN_RANGE_MIN):
            {
                state = p_env_ln->ln_min;
            } break;

            case (MM_STATE_LIGHT_LN_RANGE_MAX):
            {
                state = p_env_ln->ln_max;
            } break;

            case (MM_STATE_LIGHT_LN_LAST):
            {
                state = p_env_ln->ln_last;
            } break;

            default:
            {
            } break;
        }
    }

    return (state);
}

void mm_lights_ln_set_dflt(uint8_t elmt_idx, uint16_t ln_dflt)
{
    // Get model local index of Light Lightness Server model on given element
    m_lid_t mdl_lid = mm_tb_state_get_lid(elmt_idx, MM_ID_LIGHTS_LN);

    if (mdl_lid != MESH_INVALID_LID)
    {
        // Get environment for the model
        mm_lights_ln_env_t *p_env_ln = (mm_lights_ln_env_t *)mm_tb_state_get_env(mdl_lid);

        // Keep the provided value
        p_env_ln->ln_dflt = ln_dflt;

        // Inform application about received value
        mm_api_send_srv_state_upd_ind(MM_STATE_LIGHT_LN_DFLT, elmt_idx, ln_dflt, 0);
    }
}

#endif //(BLE_MESH_MDL_LIGHTS_LN)
/// @} end of group
