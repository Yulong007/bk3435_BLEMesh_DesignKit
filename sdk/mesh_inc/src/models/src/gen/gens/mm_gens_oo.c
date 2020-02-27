/**
 ****************************************************************************************
 * @file mm_gens_oo.c
 *
 * @brief Mesh Model Generic OnOff Server Module
 *
 * Copyright (C) RivieraWaves 2017-2019
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup MM_GENS_OO
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "mm_gens_int.h"      // Mesh Model Generic Server Module Internal Definitions
#include "gpio.h"
/*
 * DEFINES
 ****************************************************************************************
 */
#if(BLE_MESH_MDL_GENS_OO)
/// Validity of information provided to the Replay Manager
#define MM_GENS_OO_REPLAY_MS               (6000)
 
/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

/// Structure for Generic OnOff Server model environment
typedef struct mm_gens_oo_env
{
    /// Basic model environment - Must be first element in the structure - DO NOT MOVE
    mm_tb_state_mdl_env_t env;
    /// Timer for sending of publications
    mesh_tb_timer_t tmr_publi;
    /// Publication period in milliseconds
    uint32_t publi_period_ms;

    /// Environment for replay protection mechanism
    mm_tb_replay_mdl_env_t replay_env;

    /// Current OnOff state value
    uint8_t onoff;
    /// Target OnOff state value
    uint8_t tgt_onoff;

    /// Address to which a Generic OnOff Status message must be sent (unassigned address if no
    /// status must be sent)
    uint16_t status_dst_addr;
    /// Application key local index to be used for transmission of Generic OnOff Status message
    m_lid_t status_app_key_lid;
    /// Relaying of sent Generic OnOff Status authorized
    bool status_relay;
} mm_gens_oo_env_t;

/*
 * LOCAL FUNCTIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Prepare and send a Generic OnOff Status message
 *
 * @param[in] p_env_oo          Pointer to Generic OnOff Server model environment
 * @param[in] publish           True if status is a publication, false if state is a response
 ****************************************************************************************
 */
__STATIC void mm_gens_oo_send_status(mm_gens_oo_env_t *p_env_oo, mm_route_buf_env_t *p_route_env,
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
    uint16_t status;
    UNUSED(status);
    MESH_MODEL_PRINT_DEBUG("%s\r\n", __func__);

    MESH_MODEL_PRINT_DEBUG("publish :%d,p_env_oo->onoff:%d,p_env_oo->tgt_onoff:%d\r\n", publish, p_env_oo->onoff, p_env_oo->tgt_onoff);

    // Check if a transition has been started
    mm_tb_bind_get_trans_info(p_env_oo->env.grp_lid, &trans_type, &rem_time);

    // Deduce deduce data length
    data_length = (trans_type != MM_TRANS_TYPE_NONE) ? MM_GEN_OO_STATUS_LEN : MM_GEN_OO_STATUS_MIN_LEN;

    if ((status = mm_route_buf_alloc(&p_buf_status, data_length)) == MESH_ERR_NO_ERROR)
    {
        // Get pointer to data
        uint8_t *p_data = MESH_TB_BUF_DATA(p_buf_status);
        // Get pointer to environment
        mm_route_buf_env_t *p_buf_env = (mm_route_buf_env_t *)&p_buf_status->env;

        // Prepare environment
        if (p_route_env)
        {
            p_buf_env->app_key_lid = p_route_env->app_key_lid;
            p_buf_env->u_addr.dst = p_route_env->u_addr.src;
            p_buf_env->info = p_route_env->info;
        }
        else if (!publish)
        {
            p_buf_env->app_key_lid = p_env_oo->status_app_key_lid;
            p_buf_env->u_addr.dst = p_env_oo->status_dst_addr;
            SETB(p_buf_env->info, MM_ROUTE_BUF_INFO_RELAY, p_env_oo->status_relay);

            // Can accept new transitions
            p_env_oo->status_dst_addr = MESH_UNASSIGNED_ADDR;
        }

        SETB(p_buf_env->info, MM_ROUTE_BUF_INFO_RX, 0);
        SETB(p_buf_env->info, MM_ROUTE_BUF_INFO_PUBLISH, publish);
        p_buf_env->mdl_lid = p_env_oo->env.mdl_lid;
        p_buf_env->opcode = MM_MSG_GEN_OO_STATUS;

        // Fill the message
        *(p_data + MM_GEN_OO_STATUS_OO_POS) = p_env_oo->onoff;

        if (data_length == MM_GEN_OO_STATUS_LEN)
        {
            *(p_data + MM_GEN_OO_STATUS_TGT_OO_POS) = p_env_oo->tgt_onoff;
            *(p_data + MM_GEN_OO_STATUS_REM_TIME_POS) = rem_time;
        }

        // Send the message
        mm_route_send(p_buf_status);
    }
    else
    {
        MESH_MODEL_PRINT_ERR("%s, mm_route_buf_alloc fail, status = 0x%x\r\n", __func__, status);
    }
}

/**
 ****************************************************************************************
 * @brief Publish Generic OnOff state value if sending of publications is enabled
 *
 * @param[in] p_env_oo          Pointer to Generic OnOff Server model environment
 ****************************************************************************************
 */
__STATIC void mm_gens_oo_publish(mm_gens_oo_env_t *p_env_oo)
{
    // Check if sending of publication is enabled
    if (GETB(p_env_oo->env.info, MM_TB_STATE_MDL_INFO_PUBLI))
    {
        mm_gens_oo_send_status(p_env_oo, NULL, true);
    }
}

/**
 ****************************************************************************************
 * @brief Check if a Generic OnOff Status message must be sent and send it if it
 * is the case
 *
 * @param[in] p_env_oo        Pointer to Generic OnOff Server model environment
 ****************************************************************************************
 */
__STATIC void mm_gens_oo_check_status_rsp(mm_gens_oo_env_t *p_env_oo)
{
    if (p_env_oo->status_dst_addr)
    {
        // Send a response to the node that has required the transition
        mm_gens_oo_send_status(p_env_oo, NULL, false);

        p_env_oo->status_dst_addr = MESH_UNASSIGNED_ADDR;
    }
}

/*
 * MESSAGE HANDLERS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Handler for Generic OnOff Set/Set Unacknowledged message
 *
 * @param[in] p_env     Pointer to environment of model for which message has been received
 * @param[in] p_data    Pointer to received message
 ****************************************************************************************
 */
__STATIC void mm_gens_oo_handler_set(mm_gens_oo_env_t *p_env_oo, mesh_tb_buf_t *p_buf,
                                     mm_route_buf_env_t *p_route_env)
{

    do
    {
        // Get pointer to data
        uint8_t *p_data = MESH_TB_BUF_DATA(p_buf);
        // Extract required onoff state value
        uint8_t onoff = *(p_data + MM_GEN_OO_SET_OO_POS);
        // Extract TID value
        uint8_t tid = *(p_data + MM_GEN_OO_SET_TID_POS);
        // Transition time
        uint8_t trans_time = MM_TRANS_TIME_UNKNOWN;
        // Delay
        uint8_t delay = 0;

        gpio_set(0x33, onoff);
        MESH_MODEL_PRINT_DEBUG("%s,onoff:%d,p_env_oo->onoff:%d,p_env_oo->tgt_onoff:%d,tid:%d\r\n", __func__, onoff, p_env_oo->onoff, p_env_oo->tgt_onoff, tid);
        // Check received state value
        if (onoff > 1)
        {
            break;
        }

        // Extract and check optional parameters if present
        if (p_buf->data_len == MM_GEN_OO_SET_LEN)
        {
            trans_time = (uint16_t)(*(p_data + MM_GEN_OO_SET_TRANS_TIME_POS));

            // Check received value
            if (GETF(trans_time, MM_TRANS_TIME_STEP_NB) > MM_TRANS_TIME_NB_STEPS_MAX)
            {
                // Drop the message
                break;
            }

            delay = *(p_data + MM_GEN_OO_SET_DELAY_POS);
        }

        // Check if received message is a retransmitted one, if state is modified and if
        // a new transition can be started now
        if ((p_env_oo->status_dst_addr != MESH_UNASSIGNED_ADDR)
                || mm_tb_replay_is_retx(&p_env_oo->replay_env, p_route_env->u_addr.src, tid)
                || (onoff == p_env_oo->onoff))
        {

            if (p_route_env->opcode == MM_MSG_GEN_OO_SET)
            {
                MESH_MODEL_PRINT_DEBUG("gens_oo_send_status 1\r\n", onoff);
                // Send Generic OnOff status message
                mm_gens_oo_send_status(p_env_oo, p_route_env, false);
            }
            break;
        }

        // Keep information for transmission of status if needed
        if (p_route_env->opcode == MM_MSG_GEN_OO_SET)
        {
            p_env_oo->status_dst_addr = p_route_env->u_addr.src;
            p_env_oo->status_app_key_lid = p_route_env->app_key_lid;
            p_env_oo->status_relay = GETB(p_route_env->info, MM_ROUTE_BUF_INFO_RELAY);
        }

        if (GETB(p_env_oo->env.info, MM_TB_STATE_MDL_INFO_MAIN))
        {
            // Update target state
            p_env_oo->tgt_onoff = onoff;

            // Inform the Binding Manager about new transition
            mm_tb_bind_trans_new(p_env_oo->env.grp_lid, MM_TRANS_TYPE_CLASSIC,
                                 trans_time, delay);
        }
        else
        {
            // Inform the Binding Manager
            mm_tb_bind_trans_req(p_env_oo->env.grp_lid, p_env_oo->env.mdl_lid,
                                 MM_TRANS_TYPE_CLASSIC, onoff, trans_time, delay);
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
 * Generic OnOff Server model expires
 *
 * @param[in] p_env     Pointer to model environment for Generic OnOff Server model
 ****************************************************************************************
 */
__STATIC void mm_gens_oo_cb_tmr_publi(void *p_env)
{
    // Get allocated environment
    mm_gens_oo_env_t *p_env_oo = (mm_gens_oo_env_t *)p_env;

    if (p_env_oo->publi_period_ms)
    {
        // Publish a Generic OnOff Status message
        mm_gens_oo_publish(p_env_oo);

        // Restart the timer
        mesh_tb_timer_set(&p_env_oo->tmr_publi, p_env_oo->publi_period_ms);
    }
}

/**
 ****************************************************************************************
 * @brief Inform Generic OnOff Server model about a received message
 *
 * @param[in] p_env         Pointer to the environment allocated for the Generic OnOff
 * Server model
 * @param[in] p_buf         Pointer to the buffer containing the received message
 * @param[in] p_route_env   Pointer to structure containing reception parameters provided
 * by the Mesh Profile block
 ****************************************************************************************
 */
__STATIC void mm_gens_oo_cb_rx(mm_tb_state_mdl_env_t *p_env, mesh_tb_buf_t *p_buf,
                               mm_route_buf_env_t *p_route_env)
{
    // Get environment for Generic OnOff Server model
    mm_gens_oo_env_t *p_env_oo = (mm_gens_oo_env_t *)p_env;

    {
        MESH_MODEL_PRINT_DEBUG("mm_gens_oo_cb_rx,opcode = 0x%x\r\n", p_route_env->opcode);
        uint8_t *p_data = MESH_TB_BUF_DATA(p_buf);
        MESH_MODEL_PRINT_DEBUG("###############################\r\n");
        MESH_MODEL_PRINT_DEBUG("data: %s \n", mesh_buffer_to_hex(p_data, p_buf->data_len));
        MESH_MODEL_PRINT_DEBUG("###############################\r\n");
    }
    if (p_route_env->opcode != MM_MSG_GEN_OO_GET)
    {
        // Handle Generic OnOff Set/Set Unacknowledged message
        mm_gens_oo_handler_set(p_env_oo, p_buf, p_route_env);
    }
    else
    {

        // Send a Generic OnOff Status message
        mm_gens_oo_send_status(p_env_oo, p_route_env, false);
    }
}

/**
 ****************************************************************************************
 * @brief Check if a given opcode is handled by the Generic OnOff Server model
 *
 * @param[in] p_env         Pointer to the environment allocated for the Generic OnOff
 * Server model
 * @param[in] opcode        Opcode to check
 *
 * @return An error status (@see enum mesh_err)
 ****************************************************************************************
 */
__STATIC uint16_t mm_gens_oo_cb_opcode_check(mm_tb_state_mdl_env_t *p_env, uint32_t opcode)
{
    uint16_t status;

    if ((opcode == MM_MSG_GEN_OO_GET)
            || (opcode == MM_MSG_GEN_OO_SET)
            || (opcode == MM_MSG_GEN_OO_SET_UNACK))
    {
        status = MESH_ERR_NO_ERROR;
    }
    else
    {
        MESH_MODEL_PRINT_INFO("%s, Invalid opcode (0x%x)\n", __func__, opcode);
        status = MESH_ERR_MDL_INVALID_OPCODE;
    }

    return (status);
}

/**
 ****************************************************************************************
 * @brief Inform Generic OnOff Server model about received publication parameters
 *
 * @param[in] p_env         Pointer the the environment allocated for the Generic OnOff
 * Server model
 * @param[in] addr          Publication address
 * @param[in] period_ms     Publication period in milliseconds
 ****************************************************************************************
 */
__STATIC void mm_gens_oo_cb_publish_param(mm_tb_state_mdl_env_t *p_env, uint16_t addr,
        uint32_t period_ms)
{
    // Inform the state manager about received publication parameters
    mm_tb_state_publish_param_ind((mm_tb_state_mdl_publi_env_t *)p_env, addr, period_ms);
}

/**
 ****************************************************************************************
 * @brief Set Generic OnOff state value
 *
 * @param[in] p_env         Pointer the the environment allocated for the Generic OnOff
 * Server model
 * @param[in] onoff         Generic OnOff state value
 *
 * @return An error status (@see enum mesh_err)
 ****************************************************************************************
 */
__STATIC uint16_t mm_gens_oo_cb_set(mm_tb_state_mdl_env_t *p_env, uint16_t state_id, uint32_t state)
{
    // Returned status
    uint16_t status = MESH_ERR_NO_ERROR;

    MESH_MODEL_PRINT_DEBUG("%s\r\n", __func__);

    if (GETB(p_env->info, MM_TB_STATE_MDL_INFO_MAIN))
    {
        // Generic OnOff state
        uint8_t onoff = state;

        if (onoff < 1)
        {
            // Get environment for the Generic OnOff Server model
            mm_gens_oo_env_t *p_env_oo = (mm_gens_oo_env_t *)p_env;

            // Set state value
            p_env_oo->onoff = onoff;
        }
        else
        {
            MESH_MODEL_PRINT_WARN("%s, Invalid parameter.\n", __func__);
            status = MESH_ERR_INVALID_PARAM;
        }
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

uint16_t mm_gens_oo_register(uint8_t elmt_idx, m_lid_t *p_mdl_lid)
{
    // Register the model
    uint16_t status = m_api_register_model(MM_ID_GENS_OO, elmt_idx, M_MDL_CONFIG_PUBLI_AUTH_BIT,
                                           &mm_route_cb, p_mdl_lid);

    do
    {
        // Check if model has been properly registered
        if (status != MESH_ERR_NO_ERROR)
        {
            break;
        }

        // Inform the Model State Manager about registered model
        status = mm_tb_state_register(elmt_idx, MM_ID_GENS_OO, *p_mdl_lid,
                                      MM_TB_STATE_ROLE_SRV | MM_TB_STATE_CFG_CB_BIT,
                                      sizeof(mm_gens_oo_env_t));

        if (status == MESH_ERR_NO_ERROR)
        {
            // Get allocated environment
            mm_gens_oo_env_t *p_env_oo = (mm_gens_oo_env_t *)mm_tb_state_get_env(*p_mdl_lid);
            // Get server-specific callback functions
            mm_srv_cb_t *p_cb_srv = p_env_oo->env.cb.u.p_cb_srv;

            // Prepare timer for publication
            p_env_oo->tmr_publi.cb = mm_gens_oo_cb_tmr_publi;
            p_env_oo->tmr_publi.p_env = (void *)p_env_oo;


            p_env_oo->onoff = 1;//sean add for init
            // Prepare environment for Replay Manager
            p_env_oo->replay_env.delay_ms = MM_GENS_OO_REPLAY_MS;

            // Set internal callback functions
            p_env_oo->env.cb.cb_rx = mm_gens_oo_cb_rx;
            p_env_oo->env.cb.cb_opcode_check = mm_gens_oo_cb_opcode_check;
            p_env_oo->env.cb.cb_publish_param = mm_gens_oo_cb_publish_param;
            p_cb_srv->cb_set = mm_gens_oo_cb_set;

            // Inform application about registered model
            mm_api_send_register_ind(MM_ID_GENS_OO, elmt_idx, *p_mdl_lid);
        }
        else
        {
            MESH_MODEL_PRINT_WARN("%s, Model State register fail.\n", __func__);
        }
    }
    while (0);

    return (status);
}

/*
 * CALLBACK FUNCTIONS FOR BINDING MANAGER
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Callback function called by the Binding Manager when group event occurs
 *
 * @param[in] mdl_lid       Model local index
 * @param[in] event         Event
 * @param[in] info          Information
 ****************************************************************************************
 */
void mm_gens_oo_cb_grp_event(m_lid_t mdl_lid, uint8_t event, uint8_t info)
{
    // Get environment for Generic OnOff Server model
    mm_gens_oo_env_t *p_env_oo = (mm_gens_oo_env_t *)mm_tb_state_get_env(mdl_lid);
    MESH_MODEL_PRINT_DEBUG("%s event:%d\r\n", __func__, event);
    switch (event)
    {
        case (MESH_MDL_GRP_EVENT_TRANS_REJECTED):
        {

            // Send a response to the node that has required the transition
            mm_gens_oo_check_status_rsp(p_env_oo);
        } break;

        case (MESH_MDL_GRP_EVENT_TRANS_DELAY_EXPIRED):
        {
            // Start the transition
            mm_tb_bind_trans_start(p_env_oo->env.grp_lid);
        } break;

        case (MESH_MDL_GRP_EVENT_TRANS_IMMEDIATE):
        {
            p_env_oo->onoff = p_env_oo->tgt_onoff;
        } // no break;

        case (MESH_MDL_GRP_EVENT_TRANS_STARTED):
        {
            // Inform application about the update
            if (GETB(p_env_oo->env.info, MM_TB_STATE_MDL_INFO_MAIN))
            {
                uint8_t trans_time = info;



                // Inform application about state update
                mm_api_send_srv_state_upd_ind(MM_STATE_GEN_ONOFF, p_env_oo->env.elmt_idx,
                                              p_env_oo->tgt_onoff,
                                              (trans_time) ? mm_tb_get_trans_time_ms(trans_time) : 0);
            }

            // Send a response to the node that has required the transition
            mm_gens_oo_check_status_rsp(p_env_oo);

            // Send a publication
            mm_gens_oo_publish(p_env_oo);

            if (p_env_oo->tgt_onoff == 1)
            {
                p_env_oo->onoff = 1;
            }
        } break;

        case (MESH_MDL_GRP_EVENT_TRANS_END):
        {
            // New state is the target state
            p_env_oo->onoff = p_env_oo->tgt_onoff;

            // Inform application about the update
            if (GETB(p_env_oo->env.info, MM_TB_STATE_MDL_INFO_MAIN))
            {
                // Inform application about state update
                mm_api_send_srv_state_upd_ind(MM_STATE_GEN_ONOFF, p_env_oo->env.elmt_idx,
                                              p_env_oo->onoff, 0);
            }

            // Send a publication
            mm_gens_oo_publish(p_env_oo);
        } break;

        // case (MESH_MDL_GRP_EVENT_TRANS_ABORTED):
        // case (MESH_MDL_GRP_EVENT_GROUP_FULL):
        default:
        {
        } break;
    }
}

void mm_gens_oo_cb_trans_req(m_lid_t main_mdl_lid, uint32_t req_model_id, uint8_t trans_type,
                             uint32_t state_delta)
{
    MESH_MODEL_PRINT_DEBUG("mm_gens_oo_cb_trans_req,state_delta = 0x%x\r\n", state_delta);
    // Get environment for Light CTL Server model
    mm_gens_oo_env_t *p_env_oo = (mm_gens_oo_env_t *)mm_tb_state_get_env(main_mdl_lid);
    // Targeted Gen onoff Actual state value
    uint8_t onoff;
    if (req_model_id == MM_ID_GENS_OO)
    {
        // Requested Generic OnOff state value
        onoff = (uint8_t)state_delta;

    }

    // Check if Light CTL Actual state value is modified
    if (onoff != p_env_oo->onoff)
    {
        p_env_oo->onoff = onoff;

        // Start a new transition
        mm_tb_bind_trans_new(p_env_oo->env.grp_lid, trans_type, 0, 0);
    }
    else
    {
        // Reject the transition
        mm_tb_bind_trans_reject(p_env_oo->env.grp_lid);
    }
}

void mm_gens_oo_cb_set_state(m_lid_t mdl_lid, uint8_t type, uint32_t state)
{
    // Get environment for Generic Level Server model
    mm_gens_oo_env_t *p_env_oo = (mm_gens_oo_env_t *)mm_tb_state_get_env(mdl_lid);

    // Sanity check
    ASSERT_INFO(!GETB(p_env_oo->env.info, MM_TB_STATE_MDL_INFO_MAIN), mdl_lid, 0);

    MESH_MODEL_PRINT_DEBUG("%s type:%d,state:%d\r\n", __func__, type, state);
    if (type == MM_STATE_TYPE_CURRENT)
    {
        p_env_oo->onoff = (uint8_t)state;
    }
    else     // (type == MM_STATE_TYPE_TARGET) || (type == MM_STATE_TYPE_TARGET_MOVE)
    {
        p_env_oo->tgt_onoff = (int8_t)state;
    }
}
#endif //(BLE_MESH_MDL_GENS_OO)
/// @} end of group
