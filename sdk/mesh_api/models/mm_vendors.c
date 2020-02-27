/**
 ****************************************************************************************
 * @file mm_vendors.c
 *
 * @brief Mesh Model vendor Server Module
 *
 * Copyright (C) Beken 2019-2030
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup MM_VENDORS
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "mm_vendors.h"
#include "co_utils.h"
#include "m_tb_key.h"
#include "mesh_log.h"

/*
 * GLOBAL FUNCTIONS
 ****************************************************************************************
 */
#define MM_VENDOR_3B_OPCODE(opcode)                                                             \
    ((uint32_t)(opcode))



#define MM_IS_VENDOR_OPCODE(opcode)       ((((opcode) & 0xFFFF00) >> 8) == 0x01A8)
static bool is_cfm_msg_recved  = false;

static m_lid_t vdr_lid;


uint8_t * get_attr_info(mm_vendors_env_t *p_env_vdr,uint16_t attr_type,uint8_t *len)
{
    for (uint32_t i = 0; i< MM_VENDORS_ATTR_MAX_NUM; i++)
    {
        if ((p_env_vdr->attr[i].attr_type == attr_type)&&(p_env_vdr->attr[i].attr_type != 0))
        {
            *len = p_env_vdr->attr[i].attr_len;
            return p_env_vdr->attr[i].attr_param;
        }
    }
    *len = 0;
    return NULL;

}


uint8_t get_vendor_tid(mm_vendors_env_t *p_env_vdr)
{
    return p_env_vdr->tid++;
}


uint8_t find_index_by_type(mm_vendors_env_t *p_env_vdr)
{
    int i = 0;
    for (; i < MM_VENDORS_ATTR_MAX_NUM; i ++)
    {
        if (p_env_vdr->cur_attr_type == p_env_vdr->attr[i].attr_type)
        {
            return i;
        }
    }
    return i;
}
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
__STATIC void mm_vendors_send_status(mm_vendors_env_t *p_env_vdr,
                                     mm_route_buf_env_t *p_route_env, uint16_t publish, uint16_t attr_type)
{
    MESH_MODEL_PRINT_DEBUG("%s\r\n", __func__);

    // Pointer to the buffer that will contain the message
    mesh_tb_buf_t *p_buf_status;
    // Remaining time
    uint8_t rem_time;
    // Transition type
    uint8_t trans_type;
    // Data length
    uint8_t data_length;

    // Check if a transition has been started
    mm_tb_bind_get_trans_info(p_env_vdr->env.grp_lid, &trans_type, &rem_time);

    // Deduce deduce data length
    uint8_t *attr_param = NULL;
    attr_param = get_attr_info(p_env_vdr,attr_type,&data_length);

    MESH_MODEL_PRINT_DEBUG("attr_param :0x%x,attr_type:0x%x,data_length:%d\r\n",attr_param,attr_type,data_length);
    if (attr_param)
    {
        if (mm_route_buf_alloc(&p_buf_status, data_length + 3) == MESH_ERR_NO_ERROR)
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
                p_buf_env->app_key_lid = p_env_vdr->status_app_key_lid;
                p_buf_env->u_addr.dst = p_env_vdr->status_dst_addr;
                SETB(p_buf_env->info, MM_ROUTE_BUF_INFO_RELAY, p_env_vdr->status_relay);
            }
            SETB(p_buf_env->info, MM_ROUTE_BUF_INFO_RX, 0);
            SETB(p_buf_env->info, MM_ROUTE_BUF_INFO_PUBLISH, publish);
            p_buf_env->mdl_lid = p_env_vdr->env.mdl_lid;
            p_buf_env->opcode = MM_VENDOR_3B_OPCODE(MM_MSG_VENDOR_ATTR_ATTR_STATUS);
            // Fill the message
            *p_data = get_vendor_tid(p_env_vdr);
            co_write16p(p_data + MM_VENDORS_STATUS_ATTR_TYPE_POS, attr_type);
            memcpy(p_data + MM_VENDORS_STATUS_ATTR_PARAM_POS,attr_param,data_length);
            // Send the message
            mm_route_send(p_buf_status);

            MESH_MODEL_PRINT_DEBUG("mm_route_send satus\r\n");
        }
    }
}


/**
 ****************************************************************************************
 * @brief Publish vendors state value if sending of publications is enabled
 *
 * @param[in] p_env_oo          Pointer to vendors Server model environment
 ****************************************************************************************
 */
__STATIC void mm_vendors_publish(mm_vendors_env_t *p_env_vdr)
{
    MESH_MODEL_PRINT_DEBUG("mm_vendors_publish\r\n");
    // Check if sending of publication is enabled
    if (GETB(p_env_vdr->env.info, MM_TB_STATE_MDL_INFO_PUBLI))
    {
        mm_vendors_send_status(p_env_vdr, NULL, true, p_env_vdr->cur_attr_type);
    }
}

/*
 * MESSAGE HANDLERS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Check if a Generic OnOff Status message must be sent and send it if it
 * is the case
 *
 * @param[in] p_env_oo        Pointer to Generic OnOff Server model environment
 ****************************************************************************************
 */
__STATIC void mm_vendors_check_status_rsp(mm_vendors_env_t *p_env_vdr)
{
    if (p_env_vdr->status_dst_addr)
    {
        // Send a response to the node that has required the transition
        mm_vendors_send_status(p_env_vdr, NULL, false,p_env_vdr->cur_attr_type);

        p_env_vdr->status_dst_addr = MESH_UNASSIGNED_ADDR;
    }
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
__STATIC void mm_vendors_cb_publish_param(mm_tb_state_mdl_env_t *p_env, uint16_t addr,
        uint32_t period_ms)
{
    // Inform the state manager about received publication parameters
    MESH_MODEL_PRINT_DEBUG("mm_vendors_cb_publish_param addr:%x,period_ms:%d\r\n",addr,period_ms);
    //   mm_tb_state_publish_param_ind((mm_tb_state_mdl_publi_env_t *)p_env, addr, period_ms);
}
/**
 ****************************************************************************************
 * @brief Handler for vENDORS Set/Set Unacknowledged
 * Set/Set Unacknowledged message.
 * Note that all messages have the same content.
 *
 * @param[in] p_env         Pointer to environment of model for which message has been received
 * @param[in] p_buf         Pointer to buffer containing the received message
 * @param[in] p_route_env   Pointer to routing information for the received buffer
 ****************************************************************************************
 */
__STATIC void mm_vendors_handler_set(mm_vendors_env_t *p_env_vdr, mesh_tb_buf_t *p_buf,
                                     mm_route_buf_env_t *p_route_env)
{
    do
    {
        // Get pointer to data
        uint8_t *p_data = MESH_TB_BUF_DATA(p_buf);
        uint8_t data_len;
        uint8_t trans_time = 0;
        uint8_t delay = 0;

        // Extract TID value
        uint8_t tid = *(p_data + MM_VENDORS_STATUS_TID_POS);

        uint16_t attr_type =  co_read16(p_data + MM_VENDORS_STATUS_ATTR_TYPE_POS);
        MESH_MODEL_PRINT_DEBUG("vendor model,attr_type: 0x%x\r\n", attr_type);
        uint8_t * attr_param = get_attr_info(p_env_vdr,attr_type,&data_len);

        MESH_MODEL_PRINT_DEBUG("vendor model,p_buf->data_len: %d\r\n", p_buf->data_len);
        MESH_MODEL_PRINT_DEBUG("vendor model,p_buf->tail_len: %d\r\n", p_buf->tail_len);
        MESH_MODEL_PRINT_DEBUG("vendor model,p_buf->head_len: %d\r\n", p_buf->head_len);
        MESH_MODEL_PRINT_DEBUG("p_buf DATA:");
        for (int i=0; i<p_buf->data_len; i++)
        {
            MESH_MODEL_PRINT_DEBUG("0x%x ", p_data[i]);
        }
        MESH_MODEL_PRINT_DEBUG("\r\n");

        // Check if received message is a retransmitted one, if state is modified and if
        // a new transition can be started now
        if ((p_env_vdr->status_dst_addr != MESH_UNASSIGNED_ADDR)
                || mm_tb_replay_is_retx(&p_env_vdr->replay_env, p_route_env->u_addr.src, tid)
                /*|| (!memcmp(attr_param, p_data + MM_VENDORS_STATUS_ATTR_PARAM_POS,data_len))*/
                || (attr_param == NULL))
        {
            if (p_route_env->opcode == MM_MSG_VENDOR_ATTR_ATTR_SET)
            {
                // Send Generic OnOff status message
                mm_vendors_send_status(p_env_vdr, p_route_env, false,attr_type);
            }
            break;
        }

        // Keep information for transmission of status if needed
        if (p_route_env->opcode == MM_MSG_VENDOR_ATTR_ATTR_SET)
        {
            p_env_vdr->status_dst_addr = p_route_env->u_addr.src;
            p_env_vdr->status_app_key_lid = p_route_env->app_key_lid;
            p_env_vdr->status_relay = GETB(p_route_env->info, MM_ROUTE_BUF_INFO_RELAY);
        }

        p_env_vdr->cur_attr_type = attr_type;

        if (GETB(p_env_vdr->env.info, MM_TB_STATE_MDL_INFO_MAIN))
        {
            // Update state
            uint8_t index = find_index_by_type(p_env_vdr);
            memcpy(p_env_vdr->attr[index].tgt_attr_param,p_data + MM_VENDORS_STATUS_ATTR_PARAM_POS,data_len);

            // Inform the Binding Manager about new transition
            mm_tb_bind_trans_new(p_env_vdr->env.grp_lid, MM_TRANS_TYPE_CLASSIC,
                                 trans_time, delay);
            MESH_MODEL_PRINT_DEBUG("vendor model, mm_tb_bind_trans_new\r\n");
        }
        else
        {
            MESH_MODEL_PRINT_DEBUG("vendor model, mm_tb_bind_trans_array_req\r\n");
            // Inform the Binding Manager
            mm_tb_bind_trans_array_req(p_env_vdr->env.grp_lid, p_env_vdr->env.mdl_lid,
                                       MM_TRANS_TYPE_CLASSIC, p_data + MM_VENDORS_STATUS_ATTR_PARAM_POS, trans_time, delay);
        }
    }
    while (0);

}


/**
 ****************************************************************************************
 * @brief Callback function called when timer monitoring publication duration for
 * vendor  Server model expires
 *
 * @param[in] p_env     Pointer to model environment for vendor Server model
 ****************************************************************************************
 */
__STATIC void mm_vendor_cb_tmr_publi(void *p_env)
{
    // Get allocated environment
    mm_vendors_env_t *p_env_vdr = (mm_vendors_env_t *)p_env;
    MESH_MODEL_PRINT_DEBUG("mm_vendors_publish,publi_period_ms:%d\r\n",p_env_vdr->publi_period_ms);
    if (p_env_vdr->publi_period_ms)
    {
        // Publish a Generic OnOff Status message
        mm_vendors_publish(p_env_vdr);

        // Restart the timer
        mesh_tb_timer_set(&p_env_vdr->tmr_publi, p_env_vdr->publi_period_ms);
    }
}
/**
 ****************************************************************************************
 * @brief Inform Vendor  Server model about a received message
 *
 * @param[in] p_env         Pointer to the environment allocated for the Generic OnOff
 * Server model
 * @param[in] p_buf         Pointer to the buffer containing the received message
 * @param[in] p_route_env   Pointer to structure containing reception parameters provided
 * by the Mesh Profile block
 ****************************************************************************************
 */
__STATIC void mm_vendors_cb_rx(mm_tb_state_mdl_env_t *p_env, mesh_tb_buf_t *p_buf,
                               mm_route_buf_env_t *p_route_env)
{
    // Get environment for Generic OnOff Server model
    mm_vendors_env_t *p_env_vdr = (mm_vendors_env_t *)p_env;

    // Get only second byte of the opcode
    uint32_t opcode_3b = p_route_env->opcode;//(uint8_t)(p_route_env->opcode >> 16);

    MESH_MODEL_PRINT_DEBUG("mm_vendors_cb_rx, opcode = 0x%x\r\n", opcode_3b);
    switch (opcode_3b)
    {
        case MM_MSG_VENDOR_ATTR_GET:
        {
            uint8_t *p_data = MESH_TB_BUF_DATA(p_buf);
            uint16_t attr_type =  co_read16(p_data + MM_VENDORS_STATUS_ATTR_TYPE_POS);
            p_env_vdr->cur_attr_type = attr_type;
            MESH_MODEL_PRINT_DEBUG("vendor model,attr_type: 0x%x\r\n", attr_type);
            mm_vendors_send_status(p_env_vdr, p_route_env, false,attr_type);
        } break;

        case MM_MSG_VENDOR_ATTR_ATTR_SET:
        case MM_MSG_VENDOR_ATTR_ATTR_SET_UNACK:
        {
            mm_vendors_handler_set(p_env_vdr,p_buf,p_route_env);
        } break;

        case MM_MSG_VENDOR_ATTR_CONFIRMATION:
        {
            MESH_MODEL_PRINT_DEBUG("RECV vendors cfm msg\r\n");
            uint8_t *p_data = MESH_TB_BUF_DATA(p_buf);
            uint16_t attr_type =  co_read16(p_data + MM_VENDORS_STATUS_ATTR_TYPE_POS);
            p_env_vdr->cur_attr_type = attr_type;
            is_cfm_msg_recved = true;
            MESH_MODEL_PRINT_DEBUG("vendor model,attr_type: 0x%x\r\n", attr_type);
        } break;

        case MM_MSG_VENDOR_ATTR_TRANSPARENT:
        {

        } break;


        default:break;

    }

}


void mm_vendors_cfm_clear(void)
{
    is_cfm_msg_recved = false;
}

bool mm_vendors_cfm_recved(void)
{
    return is_cfm_msg_recved;
}

/**
 ****************************************************************************************
 * @brief Check if a given opcode is handled by the Vendor Server model
 *
 * @param[in] p_env         Pointer to the environment allocated for the Vendor
 * Server model
 * @param[in] opcode        Opcode to check
 *
 * @return An error status (@see enum mesh_err)
 ****************************************************************************************
 */
__STATIC uint16_t mm_vendors_cb_opcode_check(mm_tb_state_mdl_env_t *p_env, uint32_t opcode)
{
    uint16_t status = MESH_ERR_MDL_INVALID_OPCODE;

    MESH_MODEL_PRINT_DEBUG("mm_vendors_cb_opcode_check, opcode = 0x%x, p_env->mdl_id = 0x%x\r\n", opcode, p_env->mdl_id);
    if (p_env->mdl_id == MM_ID_VENDORS)
    {
        if (MM_IS_VENDOR_OPCODE(opcode))
        {
            // Get only second byte of the opcode
            uint32_t opcode_3b = opcode; //(uint8_t)(opcode >> 16);
            if ((opcode_3b == MM_MSG_VENDOR_ATTR_GET)
                    || (opcode_3b == MM_MSG_VENDOR_ATTR_ATTR_SET)
                    || (opcode_3b == MM_MSG_VENDOR_ATTR_ATTR_SET_UNACK)
                    || (opcode_3b == MM_MSG_VENDOR_ATTR_CONFIRMATION)
                    || (opcode_3b == MM_MSG_VENDOR_ATTR_TRANSPARENT))
            {
                status = MESH_ERR_NO_ERROR;
            }
        }

    }

    if (status != MESH_ERR_NO_ERROR)
    {
        MESH_MODEL_PRINT_INFO("%s, Invalid opcode (0x%x).\n", __func__, opcode);
    }

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
void mm_vendors_cb_grp_event(m_lid_t mdl_lid, uint8_t event, uint8_t info)
{
    // Get environment for Vendor Server model
    mm_vendors_env_t *p_env_vdr = (mm_vendors_env_t *)mm_tb_state_get_env(mdl_lid);

    MESH_MODEL_PRINT_DEBUG("mm_vendors_cb_grp_event, event: %d \r\n", event);
    switch (event)
    {
        case (MESH_MDL_GRP_EVENT_TRANS_REJECTED):
        {
            // Send a response to the node that has required the transition
            mm_vendors_check_status_rsp(p_env_vdr);
        } break;

        case (MESH_MDL_GRP_EVENT_TRANS_DELAY_EXPIRED):
        {
            // Start the transition
            mm_tb_bind_trans_start(p_env_vdr->env.grp_lid);
        } break;

        case (MESH_MDL_GRP_EVENT_TRANS_IMMEDIATE):
        {
            uint8_t index = 0;
            index = find_index_by_type(p_env_vdr);
            if (MM_VENDORS_ATTR_MAX_NUM  != index)
            {
                memcpy(p_env_vdr->attr[index].attr_param,p_env_vdr->attr[index].tgt_attr_param,p_env_vdr->attr[index].attr_len);
                MESH_MODEL_PRINT_DEBUG("mm_vendors_cb_grp_event, pDATA:");
                for (int i=0; i<p_env_vdr->attr[index].attr_len; i++)
                {
                    MESH_MODEL_PRINT_DEBUG("0x%x ", p_env_vdr->attr[index].attr_param[i]);
                }
                MESH_MODEL_PRINT_DEBUG("\r\n");
            }
        } // no break;

        case (MESH_MDL_GRP_EVENT_TRANS_STARTED):
        {
            // Inform application about the update
            if (GETB(p_env_vdr->env.info, MM_TB_STATE_MDL_INFO_MAIN))
            {
                uint8_t trans_time = info;

                uint8_t index = 0;
                index = find_index_by_type(p_env_vdr);

                if (p_env_vdr->cur_attr_type == LIGHT_COLOR)
                {
                    // Inform application about state update
                    mm_api_send_srv_array_state_upd_ind(MM_STATE_VENDORS_HSL, p_env_vdr->env.elmt_idx,
                                                        p_env_vdr->attr[index].attr_len,p_env_vdr->attr[index].tgt_attr_param,
                                                        (trans_time) ? mm_tb_get_trans_time_ms(trans_time) : 0);
                }
            }

            // Send a response to the node that has required the transition
            mm_vendors_check_status_rsp(p_env_vdr);

            // Send a publication
            mm_vendors_publish(p_env_vdr);

        } break;

        case (MESH_MDL_GRP_EVENT_TRANS_END):
        {
            // New state is the target state
            uint8_t index = 0;
            index = find_index_by_type(p_env_vdr);
            if (MM_VENDORS_ATTR_MAX_NUM  != index)
            {
                memcpy(p_env_vdr->attr[index].attr_param,p_env_vdr->attr[index].tgt_attr_param,p_env_vdr->attr[index].attr_len);
            }

            // Inform application about the update
            if (GETB(p_env_vdr->env.info, MM_TB_STATE_MDL_INFO_MAIN))
            {
                uint8_t index = 0;
                index = find_index_by_type(p_env_vdr);

                if (p_env_vdr->cur_attr_type == LIGHT_COLOR)
                {
                    // Inform application about state update
                    mm_api_send_srv_array_state_upd_ind(MM_STATE_VENDORS_HSL, p_env_vdr->env.elmt_idx,
                                                        p_env_vdr->attr[index].attr_len,p_env_vdr->attr[index].tgt_attr_param, 0);
                }
            }
            // Send a publication
            mm_vendors_publish(p_env_vdr);
        } break;

        // case (MESH_MDL_GRP_EVENT_TRANS_ABORTED):
        // case (MESH_MDL_GRP_EVENT_GROUP_FULL):
        default:
        {
        } break;
    }
}



/**
 ****************************************************************************************
 * @brief Set vendors state value
 *
 * @param[in] p_env         Pointer the the environment allocated for the Generic OnOff
 * Server model
 * @param[in] onoff         Generic OnOff state value
 *
 * @return An error status (@see enum mesh_err)
 ****************************************************************************************
 */
__STATIC uint16_t mm_vendors_cb_set(mm_tb_state_mdl_env_t *p_env, uint16_t state_id, uint32_t state)
{
    // Returned status
    uint16_t status = MESH_ERR_NO_ERROR;

    MESH_APP_PRINT_INFO("mm_vendors_cb_set\r\n");


    return (status);
}


uint16_t mm_vendors_register(uint8_t elmt_idx, m_lid_t *p_mdl_lid)
{
    // Register the model
    uint16_t status = m_api_register_model(MM_ID_VENDORS, elmt_idx, M_MDL_CONFIG_PUBLI_AUTH_BIT,
                                           &mm_route_cb, p_mdl_lid);

    do
    {
        // Check if model has been properly registered
        if (status != MESH_ERR_NO_ERROR)
        {
            break;
        }

        // Inform the Model State Manager about registered model
        status = mm_tb_state_register(elmt_idx, MM_ID_VENDORS, *p_mdl_lid,
                                      MM_TB_STATE_ROLE_SRV | MM_TB_STATE_CFG_CB_BIT,
                                      sizeof(mm_vendors_env_t));

        if (status == MESH_ERR_NO_ERROR)
        {
            // Get allocated environment
            mm_vendors_env_t *p_env_vdr = (mm_vendors_env_t *)mm_tb_state_get_env(*p_mdl_lid);
            // Get server-specific callback functions
            mm_srv_cb_t *p_cb_srv = p_env_vdr->env.cb.u.p_cb_srv;

            // Prepare timer for publication
            p_env_vdr->tmr_publi.cb = mm_vendor_cb_tmr_publi;
            p_env_vdr->tmr_publi.p_env = (void *)p_env_vdr;

            vdr_lid = *p_mdl_lid;
            p_env_vdr->tid = 0;

            // Prepare environment for Replay Manager
            p_env_vdr->replay_env.delay_ms = MM_VENDORS_REPLAY_MS;

            // Set internal callback functions
            p_env_vdr->env.cb.cb_rx = mm_vendors_cb_rx;
            p_env_vdr->env.cb.cb_opcode_check = mm_vendors_cb_opcode_check;
            p_env_vdr->env.cb.cb_publish_param = mm_vendors_cb_publish_param;
            p_cb_srv->cb_set = mm_vendors_cb_set;


            //light switch status type
            p_env_vdr->attr[0].attr_type = 0x0100;
            p_env_vdr->attr[0].attr_len = 0x01;
            //light color status type
            p_env_vdr->attr[1].attr_type = 0x0123;
            p_env_vdr->attr[1].attr_len = 0x06;
            p_env_vdr->cur_attr_type = 0x0100;


            // Group local index
            m_lid_t grp_lid;

            // Create group
            mm_tb_bind_add_group(0, elmt_idx, &grp_lid, *p_mdl_lid,
                                 mm_vendors_cb_grp_event, NULL);

            // Inform application about registered model
            mm_api_send_register_ind(MM_ID_VENDORS, elmt_idx, *p_mdl_lid);
        }
    }
    while (0);

    return (status);
}



void mm_vendors_cb_set_array_state(m_lid_t mdl_lid, uint8_t type, uint8_t* buf)
{
    // Get environment for Vendor Server model
    mm_vendors_env_t *p_env_vdr = (mm_vendors_env_t *)mm_tb_state_get_env(mdl_lid);

    // Extract TID value

    uint8_t data_len;
    uint8_t tid = *(buf + MM_VENDORS_STATUS_TID_POS);

    uint16_t attr_type =  co_read16(buf + MM_VENDORS_STATUS_ATTR_TYPE_POS);
    uint8_t * attr_param = get_attr_info(p_env_vdr,attr_type,&data_len);
    uint8_t index = find_index_by_type(p_env_vdr);

    if (attr_param)
    {
        if (type == MM_STATE_TYPE_CURRENT)
        {
            memcpy(attr_param,(buf + MM_VENDORS_STATUS_ATTR_PARAM_POS),data_len);
        }
        else
        {
            if (index != MM_VENDORS_ATTR_MAX_NUM)
            {
                memcpy(p_env_vdr->attr[index].tgt_attr_param, (buf + MM_VENDORS_STATUS_ATTR_PARAM_POS),data_len);
            }
        }
    }

}

void mm_vendor_attr_indication(uint16_t attr_type, uint8_t len, uint8_t *value)
{

    // Pointer to the buffer that will contain the message
    mesh_tb_buf_t *p_buf_status;
    if (mm_route_buf_alloc(&p_buf_status, len + 3) == MESH_ERR_NO_ERROR)
    {

        // Get allocated environment
        mm_vendors_env_t *p_env_vdr = (mm_vendors_env_t *)mm_tb_state_get_env(vdr_lid);

        MESH_APP_PRINT_INFO("indication mdl_id = 0x%x\r\n", p_env_vdr->env.mdl_id);
        // Get pointer to data
        uint8_t *p_data = MESH_TB_BUF_DATA(p_buf_status);
        // Get pointer to environment
        mm_route_buf_env_t *p_buf_env = (mm_route_buf_env_t *)&p_buf_status->env;

        // Prepare environment
        m_tb_key_app_find(0, &(p_buf_env->app_key_lid));
        SETB(p_buf_env->info, MM_ROUTE_BUF_INFO_RELAY, p_env_vdr->status_relay);


        SETB(p_buf_env->info, MM_ROUTE_BUF_INFO_RX, 0);
        SETB(p_buf_env->info, MM_ROUTE_BUF_INFO_PUBLISH, 1);
        p_buf_env->mdl_lid = p_env_vdr->env.mdl_lid;
        p_buf_env->opcode = MM_VENDOR_3B_OPCODE(MM_MSG_VENDOR_ATTR_INDICATION);
        p_env_vdr->cur_attr_type = attr_type;

        MESH_APP_PRINT_INFO("indication mdl_lid = %x,p_buf_env->opcode = 0x%x\r\n",p_env_vdr->env.mdl_lid,p_buf_env->opcode);
        // Fill the message
        *p_data = get_vendor_tid(p_env_vdr);
        co_write16p(p_data + MM_VENDORS_STATUS_ATTR_TYPE_POS, attr_type);
        memcpy(p_data + MM_VENDORS_STATUS_ATTR_PARAM_POS, value, len);

        mm_vendors_cb_set_array_state(p_env_vdr->env.mdl_lid, MM_STATE_TYPE_CURRENT, p_data);
        // Send the message
        mm_route_send(p_buf_status);

    }

}

