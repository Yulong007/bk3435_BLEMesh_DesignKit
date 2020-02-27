/**
 ****************************************************************************************
 *
 * @file m_api_msg.c
 *
 * @brief Mesh Message Application Program Interface
 *
 * Copyright (C) RivieraWaves 2017-2018
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup M_API_MSG
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "m_inc.h"          // Mesh Profile Include Files
#include "mesh_tb_buf.h"    // Mesh Buffer Manager
#include "m_api.h"          // Mesh Profile Application Programming Interface Definitions
#include "app_mesh.h"
#include "m_prov.h"         // Mesh Provisioning Layer
#include "m_fnd.h"
#if (BLE_MESH_MSG_API)

/*
 * LOCAL FUNCTIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Send a basic command complete message to the application
 *
 * @param[in] cmd_code  Debug Command Operation code (@see enum m_dbg_cmd_code)
 * @param[in] src_id    Source identifier of task that request command execution
 * @param[in] status    Status error code of the command execution (@see enum hl_err)
 ****************************************************************************************
 */
__STATIC void m_api_basic_cmp_evt_send(uint32_t cmd_code, uint16_t src_id, uint16_t status)
{
    // Allocate the command complete event message
    m_api_cmp_evt_t *p_cmp_evt = MAL_MSG_ALLOC(MESH_API_CMP_EVT, m_api_cmp_evt_t);

    // Fill the content
    p_cmp_evt->cmd_code = cmd_code;
    p_cmp_evt->status = status;

    // Send the message
    mal_msg_send(src_id, p_cmp_evt);
}
 
/**
 ****************************************************************************************
 * @brief Callback executed when mesh profile has been enabled
 ****************************************************************************************
 */
__STATIC void m_api_msg_enabled_cb(uint16_t status)
{
    m_api_basic_cmp_evt_send(M_API_ENABLE, mal_app_id_get(), status);
}

/**
 ****************************************************************************************
 * @brief Callback executed when mesh profile has been disabled
 ****************************************************************************************
 */
__STATIC void m_api_msg_disabled_cb(uint16_t status)
{
    m_api_basic_cmp_evt_send(M_API_DISABLE, mal_app_id_get(), status);
}

/**
 ****************************************************************************************
 * @brief Definition of callback function to call upon reception of a PDU for specific model identifier
 *
 * @param[in] model_lid   Model Local Identifier
 * @param[in] opcode      Operation code
 * @param[in] p_buf       Pointer to the buffer containing the message PDU. - No need to release buffer.
 * @param[in] app_key_lid Application Key Local identifier (Required for a response)
 * @param[in] src         Source address of the message (Required for a response)
 * @param[in] rssi        Measured RSSI level for the received PDU.
 * @param[in] not_relayed True if message have been received by an immediate peer; False, it can have been relayed
 ****************************************************************************************
 */
__STATIC void m_api_msg_model_rx_cb(m_lid_t model_lid, uint32_t opcode, m_api_buf_t* p_buf, m_lid_t app_key_lid,
                                    uint16_t src, int8_t rssi, bool not_relayed)
{
    // Retrieve buffer info
    uint16_t len = ((mesh_tb_buf_t *)p_buf)->data_len;
    uint8_t *p_data = MESH_TB_BUF_DATA((mesh_tb_buf_t *)p_buf);

    // Allocate message
    m_api_model_msg_ind_t *p_msg = MAL_MSG_ALLOC_DYN(MESH_API_MODEL_MSG_IND, m_api_model_msg_ind_t, len);

    // Fill the message
    p_msg->model_lid   = model_lid;
    p_msg->app_key_lid = app_key_lid;
    p_msg->rssi        = rssi;
    p_msg->not_relayed = not_relayed;
    p_msg->opcode      = opcode;
    p_msg->src         = src;
    p_msg->msg_len     = len;
    memcpy(p_msg->msg, p_data, len);

    // Send the message
    mal_msg_send(m_api_model_get_task_id(model_lid), p_msg);
}

/**
 ****************************************************************************************
 * @brief Definition of callback function to call upon reception of a PDU to check that model can handle it
 *
 * @note m_api_model_opcode_status function must be used to provide information about opcode support
 *
 * @param[in] model_lid Model Local Identifier
 * @param[in] opcode    Operation code to check
 ****************************************************************************************
 */
__STATIC void m_api_msg_model_opcode_check_cb(m_lid_t model_lid, uint32_t opcode)
{
    // Allocate message
    m_api_model_opcode_req_ind_t *p_opcode_req
        = MAL_MSG_ALLOC(MESH_API_MODEL_OPCODE_REQ_IND, m_api_model_opcode_req_ind_t);

    // Fill the message
    p_opcode_req->model_lid = model_lid;
    p_opcode_req->opcode    = opcode;

    // Send the message
    mal_msg_send(m_api_model_get_task_id(model_lid), p_opcode_req);
}

/**
 ****************************************************************************************
 * @brief Definition of callback function to call once PDU has been sent.
 *
 * @param[in] model_lid Model Local Identifier
 * @param[in] tx_hdl    Handle value configured by model when message has been requested to be sent
 * @param[in] p_buf     Pointer to the buffer containing the transmitted PDU. - Buffer must be released by model.
 * @param[in] status    Transmission status.
 ****************************************************************************************
 */
__STATIC void m_api_msg_model_sent_cb(m_lid_t model_lid, uint8_t tx_hdl, m_api_buf_t* p_buf, uint16_t status)
{
    // Allocate message
    m_api_model_msg_sent_ind_t *p_sent = MAL_MSG_ALLOC(MESH_API_MODEL_MSG_SENT_IND, m_api_model_msg_sent_ind_t);

    // Fill the message
    p_sent->model_lid = model_lid;
    p_sent->tx_hdl    = tx_hdl;
    p_sent->status    = status;

    // Send the message
    mal_msg_send(m_api_model_get_task_id(model_lid), p_sent);

    // release buffer
    m_api_buf_release(p_buf);
}

/**
 ****************************************************************************************
 * @brief Definition of callback function to call upon reception of a PDU to check that model can handle it
 *
 * @note m_api_model_opcode_status function must be used to provide information about opcode support
 *
 * @param[in] model_lid Model Local Identifier
 * @param[in] opcode    Operation code to check
 ****************************************************************************************
 */
__STATIC void m_api_msg_model_publish_param_cb(m_lid_t model_lid, uint16_t addr, uint32_t period_ms)
{
    // Allocate message
    m_api_model_publish_param_ind_t *p_ind
        = MAL_MSG_ALLOC(MESH_API_MODEL_PUBLISH_PARAM_IND, m_api_model_publish_param_ind_t);

    // Fill the message
    p_ind->model_lid = model_lid;
    p_ind->period_ms = period_ms;
    p_ind->addr = addr;

    // Send the message
    mal_msg_send(m_api_model_get_task_id(model_lid), p_ind);
}

/**
 ****************************************************************************************
 * Callback used to inform about a modification of the provisioning module state
 *
 * @param state        State of the provisioner // success or fail
 * @param status       Status of the current state
 ****************************************************************************************
 */
__STATIC void m_api_msg_prov_state_cb(uint8_t state, uint16_t status)
{
    // Allocate MESH_PROV_STATE_IND message and inform the application about the new
    // provisioning state

    m_api_prov_state_ind_t *p_ind  = MAL_MSG_ALLOC(MESH_API_PROV_STATE_IND, m_api_prov_state_ind_t);

    p_ind->state  = state;
    p_ind->status = status;


    mal_msg_send(mal_app_id_get(), p_ind);

}

/**
 ****************************************************************************************
 * Callback used to inform about end of stored information loading procedure
 *
 * @param state        State of the provisioner
 * @param status       Status of the current state
 ****************************************************************************************
 */
__STATIC void m_api_msg_loaded_cb(uint16_t status)
{
    // Inform the application
    m_api_basic_cmp_evt_send(M_API_STORAGE_LOAD, mal_app_id_get(), status);
}

/**
 ****************************************************************************************
 * Callback used to inform that provisioning parameters are required
 ****************************************************************************************
 */




__STATIC void m_api_msg_prov_param_req_cb(void)
{

    // Allocate message and send request indication to app
    void *p_req_ind = MAL_MSG_ALLOC_DEFAULT(MESH_API_PROV_PARAM_REQ_IND);
    mal_msg_send(mal_app_id_get(), p_req_ind);

}



/**
 ****************************************************************************************
 * Callback used to inform that Out Of Band Authentication Data is required for provisioning
 *
 * @note Authentication data must be provided using @see m_api_prov_oob_auth_rsp() function
 *
 *
 * @param auth_method  Authentication method (@see enum m_prov_auth_method)
 * @param auth_action  Authentication Action:
 *                     - M_PROV_AUTH_NO_OOB     = Prohibited
 *                     - M_PROV_AUTH_STATIC_OOB = 16 bytes LSB static out of band data required
 *                     - M_PROV_AUTH_OUTPUT_OOB = @see enum m_prov_out_oob, 1 bit set.
 *                     - M_PROV_AUTH_INPUT_OOB  = @see enum m_prov_in_oob, 1 bit set.
 * @param auth_size    expected authentication maximum data size
 ****************************************************************************************
 */
__STATIC void m_api_msg_prov_oob_auth_req_cb(uint8_t auth_method, uint16_t auth_action, uint8_t auth_size)
{
    m_api_prov_auth_data_req_ind_t *p_req_ind
        = MAL_MSG_ALLOC(MESH_API_PROV_AUTH_DATA_REQ_IND, m_api_prov_auth_data_req_ind_t);

    p_req_ind->auth_method  = auth_method;
    p_req_ind->auth_action  = auth_action;
    p_req_ind->auth_size    = auth_size;

    mal_msg_send(mal_app_id_get(), p_req_ind);

}

__STATIC void m_api_msg_attention_cb(uint8_t attention_state)
{
    // Allocate message
    m_api_attention_update_ind_t *p_ind
        = MAL_MSG_ALLOC(MESH_API_ATTENTION_UPDATE_IND, m_api_attention_update_ind_t);

    // Fill the message
    p_ind->attention_state = attention_state;

    // Send the message
    mal_msg_send(mal_app_id_get(), p_ind);
}
__STATIC void m_api_msg_compo_data_cb(uint8_t page)
{
    // Allocate message
    m_api_compo_data_req_ind_t *p_req_ind
        = MAL_MSG_ALLOC(MESH_API_COMPO_DATA_REQ_IND, m_api_compo_data_req_ind_t);

    // Fill the message
    p_req_ind->page = page;

    // Send the message
    mal_msg_send(mal_app_id_get(), p_req_ind);
}

__STATIC void m_api_msg_fault_get_cb(uint16_t comp_id)
{
    // Allocate message
    m_api_fault_get_req_ind_t *p_req_ind = MAL_MSG_ALLOC(MESH_API_FAULT_GET_REQ_IND, m_api_fault_get_req_ind_t);

    // Fill the message
    p_req_ind->comp_id = comp_id;

    // Send the message
    mal_msg_send(mal_app_id_get(), p_req_ind);
}

__STATIC void m_api_msg_fault_test_cb(uint16_t comp_id, uint8_t test_id, bool cfm_needed)
{
    // Allocate message
    m_api_fault_test_req_ind_t *p_req_ind = MAL_MSG_ALLOC(MESH_API_FAULT_TEST_REQ_IND, m_api_fault_test_req_ind_t);

    // Fill the message
    p_req_ind->comp_id    = comp_id;
    p_req_ind->test_id    = test_id;
    p_req_ind->cfm_needed = cfm_needed;

    // Send the message
    mal_msg_send(mal_app_id_get(), p_req_ind);
}

__STATIC void m_api_msg_fault_clear_cb(uint16_t comp_id)
{
    // Allocate message
    m_api_fault_clear_ind_t *p_ind = MAL_MSG_ALLOC(MESH_API_FAULT_CLEAR_IND, m_api_fault_clear_ind_t);

    // Fill the message
    p_ind->comp_id = comp_id;

    // Send the message
    mal_msg_send(mal_app_id_get(), p_ind);
}

__STATIC void m_api_msg_fault_period_cb(uint32_t period_ms, uint32_t period_fault_ms)
{
    // Allocate message
    m_api_fault_period_ind_t *p_ind = MAL_MSG_ALLOC(MESH_API_FAULT_PERIOD_IND, m_api_fault_period_ind_t);

    // Fill the message
    p_ind->period_ms = period_ms;
    p_ind->period_fault_ms = period_fault_ms;

    // Send the message
    mal_msg_send(mal_app_id_get(), p_ind);
}

__STATIC void m_api_msg_node_reset_cb(void)
{
    // Allocate message
    void *p_ind = MAL_MSG_ALLOC_DEFAULT(MESH_API_NODE_RESET_IND);

    // Send the message
    mal_msg_send(mal_app_id_get(), p_ind);
}

#if (BLE_MESH_LPN)
__STATIC void m_api_msg_lpn_status_cb(uint16_t status, uint16_t friend_addr)
{
    // Allocate message
    m_api_lpn_status_ind_t *p_ind = MAL_MSG_ALLOC(MESH_API_LPN_STATUS_IND, m_api_lpn_status_ind_t);

    // Fill the message
    p_ind->status = status;
    p_ind->friend_addr = friend_addr;

    // Send the message
    mal_msg_send(mal_app_id_get(), p_ind);
}

__STATIC void m_api_msg_lpn_offer_cb(uint16_t friend_addr, uint8_t rx_window,
                                     uint8_t queue_size, uint8_t subs_list_size, int8_t rssi)
{
    // Allocate message
    m_api_lpn_offer_ind_t *p_ind = MAL_MSG_ALLOC(MESH_API_LPN_OFFER_IND, m_api_lpn_offer_ind_t);

    // Fill the message
    p_ind->addr = friend_addr;
    p_ind->rx_window = rx_window;
    p_ind->queue_size = queue_size;
    p_ind->subs_list_size = subs_list_size;
    p_ind->rssi = rssi;

    // Send the message
    mal_msg_send(mal_app_id_get(), p_ind);
}
#endif //(BLE_MESH_LPN)

#if (BLE_MESH_GATT_PROXY)
__STATIC void m_api_msg_proxy_adv_update_cb(uint8_t state, uint8_t reason)
{
    // Allocate message
    m_api_proxy_adv_update_ind_t *p_ind
        = MAL_MSG_ALLOC(MESH_API_PROXY_ADV_UPDATE_IND, m_api_proxy_adv_update_ind_t);

    // Fill the message
    p_ind->state = state;
    p_ind->reason = reason;

    // Send the message
    mal_msg_send(mal_app_id_get(), p_ind);
}
#endif //(BLE_MESH_GATT_PROXY)

#if (BLE_MESH_PROVISIONER)
__STATIC void m_api_msg_unprov_beacon_cb(uint8_t* dev_uuid, uint16_t oob_info)
{
    m_api_unprov_beacon_ind_t *p_ind = MAL_MSG_ALLOC(MESH_API_PROV_UNPROV_BCN_IND, m_api_unprov_beacon_ind_t);

    memcpy(p_ind->dev_uuid, dev_uuid, MESH_DEV_UUID_LEN);
    p_ind->oob_info = oob_info;

    // Send the message
    mal_msg_send(mal_app_id_get(), p_ind);
}

__STATIC void m_api_msg_prov_capability_cb(uint16_t algorithms, uint8_t  pub_key_type, uint8_t  static_oob_type,
        uint8_t  out_oob_size, uint16_t out_oob_action, uint8_t  in_oob_size,
        uint16_t in_oob_action)
{
    m_api_prov_capabilities_ind_t *p_ind = MAL_MSG_ALLOC(MESH_API_PROV_CAPAVILITY_IND, m_api_prov_capabilities_ind_t);

    p_ind->algorithms = algorithms;
    p_ind->pub_key_type = pub_key_type;
    p_ind->static_oob_type = static_oob_type;
    p_ind->out_oob_size = out_oob_size;
    p_ind->out_oob_action = out_oob_action;
    p_ind->in_oob_size = in_oob_size;
    p_ind->in_oob_action = in_oob_action;

    // Send the message
    mal_msg_send(mal_app_id_get(), p_ind);
}

__STATIC void m_api_msg_dev_added_cb(m_lid_t dev_lid)
{
    uint8_t *p_ind = MAL_MSG_ALLOC(MESH_API_DEV_ADDED_IND, uint8_t);
    MESH_APP_PRINT_INFO("-----m_api_msg_dev_added_cb\r\n");

    *p_ind = dev_lid;

    // Send the message
    mal_msg_send(mal_app_id_get(), p_ind);

}

__STATIC void m_api_msg_confc_dev_compo_data_status_cb(m_lid_t dev_lid, uint8_t* data, uint8_t len)
{
    m_api_compo_data_status_ind_t *p_ind = MAL_MSG_ALLOC_DYN(MESH_API_DEV_COMPO_DATA_STS_IND, m_api_compo_data_status_ind_t, len - 1);

    p_ind->dev_lid = dev_lid;
    p_ind->data_len = len;
    memcpy(p_ind->data, data, len);
    // Send the message
    mal_msg_send(mal_app_id_get(), p_ind);
}


__STATIC void m_api_msg_confc_app_key_status_cb(m_lid_t dev_lid, uint8_t status, uint16_t netkey_id, uint16_t appkey_id)
{
    m_api_app_key_status_ind_t *p_ind = MAL_MSG_ALLOC(MESH_API_APP_KEY_STS_IND, m_api_app_key_status_ind_t);
    MESH_APP_PRINT_INFO("m_fnd_confc_handler_app_key_status,p=%x\r\n", p_ind);

    p_ind->dev_lid = dev_lid;
    p_ind->status= status;
    p_ind->nwk_key_id= netkey_id;
    p_ind->app_key_id= appkey_id;
    // Send the message
    mal_msg_send(mal_app_id_get(), p_ind);
}

__STATIC void m_api_msg_confc_model_app_status_cb(m_lid_t dev_lid, uint8_t status, uint16_t address, uint16_t appkey_id, uint16_t model_id)
{
    m_api_model_app_status_ind_t *p_ind = MAL_MSG_ALLOC(MESH_API_MODEL_APP_STS_IND, m_api_model_app_status_ind_t);

    p_ind->dev_lid = dev_lid;
    p_ind->status= status;
    p_ind->address= address;
    p_ind->app_key_id = appkey_id;
    p_ind->model_id = model_id;
    // Send the message
    mal_msg_send(mal_app_id_get(), p_ind);
}

__STATIC void m_api_msg_confc_model_subs_status_cb(m_lid_t dev_lid, uint8_t status, uint16_t elem_addr, uint16_t subs_addr, uint16_t model_id)
{
    m_api_model_subs_status_ind_t *p_ind = MAL_MSG_ALLOC(MESH_API_MODEL_SUBS_STS_IND, m_api_model_subs_status_ind_t);
    p_ind->dev_lid = dev_lid;
    p_ind->status = status;
    p_ind->elem_addr = elem_addr;
    p_ind->subs_addr = subs_addr;
    p_ind->model_id = model_id;
    // Send the message
    mal_msg_send(mal_app_id_get(), p_ind);
}

#endif

/*
 * GLOBAL VARIABLES
 ****************************************************************************************
 */

/// Mesh Profile Callback Structure
__STATIC const m_api_cb_t m_api_msg_cb =
{
    .cb_enabled = m_api_msg_enabled_cb,
    .cb_disabled = m_api_msg_disabled_cb,
    .cb_prov_state = m_api_msg_prov_state_cb,
    .cb_prov_param_req = m_api_msg_prov_param_req_cb,
    .cb_prov_auth_req = m_api_msg_prov_oob_auth_req_cb,
    .cb_loaded = m_api_msg_loaded_cb,
    .cb_attention = m_api_msg_attention_cb,
    .cb_compo_data = m_api_msg_compo_data_cb,
    .cb_node_reset = m_api_msg_node_reset_cb,
#if (BLE_MESH_LPN)
    .cb_lpn_status = m_api_msg_lpn_status_cb,
    .cb_lpn_offer = m_api_msg_lpn_offer_cb,
#endif //(BLE_MESH_LPN)
#if (BLE_MESH_GATT_PROXY)
    .cb_proxy_adv_update = m_api_msg_proxy_adv_update_cb,
#endif //(BLE_MESH_GATT_PROXY)
#if (BLE_MESH_PROVISIONER)
    .cb_unprov_beacon = m_api_msg_unprov_beacon_cb,
    .cb_m_api_prov_capability = m_api_msg_prov_capability_cb,
    .cb_m_api_dev_added = m_api_msg_dev_added_cb,
    .cb_m_api_confc_dev_compo_data_status = m_api_msg_confc_dev_compo_data_status_cb,
    .cb_m_api_confc_app_key_status = m_api_msg_confc_app_key_status_cb,
    .cb_m_api_confc_model_app_status = m_api_msg_confc_model_app_status_cb,
    .subs_status_cb = m_api_msg_confc_model_subs_status_cb,
#endif //(BLE_MESH_PROVISIONER) 
};

/// Callback Structure for Health Model for primary element
__STATIC const m_api_fault_cb_t m_api_msg_fault_cb =
{
    .cb_fault_get = m_api_msg_fault_get_cb,
    .cb_fault_test = m_api_msg_fault_test_cb,
    .cb_fault_clear = m_api_msg_fault_clear_cb,
    .cb_fault_period = m_api_msg_fault_period_cb,
};

/// model call-backs structure for Message API
__STATIC const m_api_model_cb_t m_api_msg_model_cb =
{
    .cb_rx = m_api_msg_model_rx_cb,
    .cb_sent = m_api_msg_model_sent_cb,
    .cb_opcode_check = m_api_msg_model_opcode_check_cb,
    .cb_publish_param = m_api_msg_model_publish_param_cb,
};

/*
 * COMMAND HANDLERS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Enable Mesh profile
 *
 * @param[in] src_id    Source identifier of task that request command execution
 ****************************************************************************************
 */
__STATIC uint16_t m_api_enable_cmd_handler(uint16_t src_id)
{
    // Enable the stack
    return (m_api_enable());
}

/**
 ****************************************************************************************
 * @brief Disable Mesh profile
 *
 * @param[in] src_id    Source identifier of task that request command execution
 ****************************************************************************************
 */
__STATIC uint16_t m_api_disable_cmd_handler(uint16_t src_id)
{
    // Disable the stack
    return (m_api_disable());
}

/**
 ****************************************************************************************
 * @brief Handle M_API_REGISTER_MODEL command
 *
 * @param[in] src_id    Source identifier of task that request command execution
 * @param[in] p_cmd     Command parameters
 ****************************************************************************************
 */
__STATIC uint16_t m_api_register_model_cmd_handler(uint16_t src_id, m_api_register_model_cmd_t *p_cmd)
{
    // Allocated model lid
    uint8_t model_lid;
    // Register the model instance
    uint16_t status = m_api_register_model(p_cmd->model_id, p_cmd->elmt_idx, p_cmd->config,
                                           &m_api_msg_model_cb, &model_lid);

    if (status == MESH_ERR_NO_ERROR)
    {
        // Allocate command complete message
        m_api_register_model_cmp_evt_t *p_cmp_evt
            = MAL_MSG_ALLOC(MESH_API_CMP_EVT, m_api_register_model_cmp_evt_t);

        // Set the task id for communication with this model
        m_api_model_set_task_id(model_lid, src_id);

        // Fill the command complete message
        p_cmp_evt->cmd_code = M_API_REGISTER_MODEL;
        p_cmp_evt->status = MESH_ERR_NO_ERROR;
        p_cmp_evt->model_lid = model_lid;

        // Send the command complete message
        mal_msg_send(src_id, p_cmp_evt);
    }

    return (status);
}

/**
 ****************************************************************************************
 * @brief Handle M_API_STORAGE_LOAD command
 *
 * @param[in] src_id    Source identifier of task that request command execution
 * @param[in] p_cmd     Command parameters
 ****************************************************************************************
 */
__STATIC uint16_t m_api_storage_load_cmd_handler(uint16_t src_id, m_api_storage_load_cmd_t *p_cmd)
{
    return (m_api_storage_load(p_cmd->length, &p_cmd->data[0]));
}

/**
 ****************************************************************************************
 * @brief Handle M_API_STORAGE_SAVE command
 *
 * @param[in] src_id    Source identifier of task that request command execution
 ****************************************************************************************
 */
__STATIC void m_api_storage_save_cmd_handler(uint16_t src_id)
{
    // Send the command complete message
    m_api_basic_cmp_evt_send(M_API_STORAGE_SAVE, src_id, m_api_storage_save());
}

/**
 ****************************************************************************************
 * @brief Handle M_API_STORAGE_CONFIG command
 *
 * @param[in] src_id    Source identifier of task that request command execution
 * @param[in] p_cmd     Command parameters
 ****************************************************************************************
 */
__STATIC void m_api_storage_config_cmd_handler(uint16_t src_id, m_api_storage_config_cmd_t *p_cmd)
{
    // Send the command complete message
    m_api_basic_cmp_evt_send(M_API_STORAGE_CONFIG, src_id, m_api_storage_config(p_cmd->config));
}

/**
 ****************************************************************************************
 * @brief Handle M_API_MODEL_MSG_PUBLISH command
 *
 * @param[in] src_id    Source identifier of task that request command execution
 * @param[in] p_cmd     Command parameters
 ****************************************************************************************
 */
__STATIC uint16_t m_api_model_publish_cmd_handler(uint16_t src_id, m_api_model_publish_cmd_t *p_cmd)
{
    uint16_t status;

    if (p_cmd->len > M_ACCESS_PAYLOAD_PARAM_LENGTH_MAX)
    {
        status = MESH_ERR_INVALID_PARAM;
    }
    else
    {
        mesh_tb_buf_t *p_buf;

        // allocate a buffer for message
        status = mesh_tb_buf_alloc(&p_buf, M_LAY_ACCESS_BUF_HEADER_LEN, p_cmd->len,
                                   M_LAY_ACCESS_BUF_TAIL_LEN);

        if (status == MESH_ERR_NO_ERROR)
        {
            // copy message into the buffer
            memcpy(MESH_TB_BUF_DATA(p_buf), p_cmd->msg, p_cmd->len);

            // request publication of the message
            status = m_api_model_publish(p_cmd->model_lid, p_cmd->opcode, p_cmd->tx_hdl, p_buf, p_cmd->trans_mic_64);

            // an error occurs, release the buffer
            if (status != MESH_ERR_NO_ERROR)
            {
                m_api_buf_release(p_buf);
            }
        }
    }

    // Send back command status
    m_api_basic_cmp_evt_send(p_cmd->cmd_code, src_id, status);

    return (MESH_ERR_NO_ERROR);
}

/**
 ****************************************************************************************
 * @brief Handle M_API_MODEL_MSG_RESPOND command
 *
 * @param[in] src_id    Source identifier of task that request command execution
 * @param[in] p_cmd     Command parameters
 ****************************************************************************************
 */
__STATIC uint16_t m_api_model_rsp_send_cmd_handler(uint16_t src_id, m_api_model_rsp_send_cmd_t *p_cmd)
{
    uint16_t status;

    if (p_cmd->len > M_ACCESS_PAYLOAD_PARAM_LENGTH_MAX)
    {
        status = MESH_ERR_INVALID_PARAM;
    }
    else
    {
        mesh_tb_buf_t *p_buf;

        // allocate a buffer for message
        status = mesh_tb_buf_alloc(&p_buf, M_LAY_ACCESS_BUF_HEADER_LEN, p_cmd->len,
                                   M_LAY_ACCESS_BUF_TAIL_LEN);

        if (status == MESH_ERR_NO_ERROR)
        {
            // copy message into the buffer
            memcpy(MESH_TB_BUF_DATA(p_buf), p_cmd->msg, p_cmd->len);

            // request message response to be send
            status = m_api_model_rsp_send(p_cmd->model_lid, p_cmd->opcode, p_cmd->tx_hdl, p_buf,
                                          p_cmd->key_lid, p_cmd->dst, p_cmd->trans_mic_64, p_cmd->not_relay);

            // an error occurs, release the buffer
            if (status != MESH_ERR_NO_ERROR)
            {
                m_api_buf_release(p_buf);
            }
        }
    }

    // Send back command status
    m_api_basic_cmp_evt_send(p_cmd->cmd_code, src_id, status);

    return (MESH_ERR_NO_ERROR);
}

/**
 ****************************************************************************************
 * @brief Handle M_API_HEALTH_STATUS_SEND_CMD command
 *
 * @param[in] src_id    Source identifier of task that request command execution
 * @param[in] p_cmd     Command parameters
 ****************************************************************************************
 */
__STATIC uint16_t m_api_health_status_send_cmd_handler(uint16_t src_id, m_api_health_status_send_cmd_t *p_cmd)
{
    // Propagate the received status
    m_api_health_status_send(p_cmd->comp_id, p_cmd->test_id, p_cmd->length, &p_cmd->fault_array[0]);

    // Send back command status
    m_api_basic_cmp_evt_send(p_cmd->cmd_code, src_id, MESH_ERR_NO_ERROR);

    return (MESH_ERR_NO_ERROR);
}

/**
 ****************************************************************************************
 * @brief Handle M_API_PROV_PUB_KEY_READ command
 *
 * @param[in] src_id    Source identifier of task that request command execution
 * @param[in] p_cmd     Command parameters
 ****************************************************************************************
 */
__STATIC uint16_t m_api_prov_pub_key_read_cmd_handler(uint16_t src_id, m_api_prov_pub_key_read_cmd_t *p_cmd)
{
    m_api_prov_pub_key_read_cmp_evt_t* p_evt = MAL_MSG_ALLOC(MESH_API_CMP_EVT, m_api_prov_pub_key_read_cmp_evt_t);

    // Read the public key
    p_evt->cmd_code = p_cmd->cmd_code;
    p_evt->status = m_api_prov_pub_key_read(p_evt->pub_key_x, p_evt->pub_key_y);

    mal_msg_send(src_id, p_evt);

    return (MESH_ERR_NO_ERROR);
}

/**
 ****************************************************************************************
 * @brief Handle M_API_IV_UPD_TEST_MODE command
 *
 * @param[in] src_id    Source identifier of task that request command execution
 * @param[in] p_cmd     Command parameters
 ****************************************************************************************
 */
__STATIC uint16_t m_api_iv_upd_test_mode_cmd_handler(uint16_t src_id, m_api_iv_upd_test_mode_cmd_t *p_cmd)
{
    m_api_iv_upd_test_mode(p_cmd->update);

    // Send back command status
    m_api_basic_cmp_evt_send(p_cmd->cmd_code, src_id, MESH_ERR_NO_ERROR);

    return (MESH_ERR_NO_ERROR);
}

#if (BLE_MESH_LPN)
/**
 ****************************************************************************************
 * @brief Handle M_API_LPN_START command
 *
 * @param[in] src_id    Source identifier of task that request command execution
 * @param[in] p_cmd     Command parameters
 ****************************************************************************************
 */
__STATIC uint16_t m_api_lpn_start_cmd_handler(uint16_t src_id, m_api_lpn_start_cmd_t *p_cmd)
{
    // Forward the command
    uint16_t status = m_api_lpn_start(p_cmd->poll_timeout, p_cmd->poll_intv_ms, p_cmd->prev_addr, p_cmd->rx_delay,
                                      p_cmd->rssi_factor, p_cmd->rx_window_factor, p_cmd->min_queue_size_log);

    // Send back command status
    m_api_basic_cmp_evt_send(p_cmd->cmd_code, src_id, status);

    return (MESH_ERR_NO_ERROR);
}

/**
 ****************************************************************************************
 * @brief Handle M_API_LPN_STOP command
 *
 * @param[in] src_id    Source identifier of task that request command execution
 * @param[in] p_cmd     Command parameters
 ****************************************************************************************
 */
__STATIC uint16_t m_api_lpn_stop_cmd_handler(uint16_t src_id)
{
    // Forward the command
    uint16_t status = m_api_lpn_stop();

    // Send back command status
    m_api_basic_cmp_evt_send(M_API_LPN_STOP, src_id, status);

    return (MESH_ERR_NO_ERROR);
}

/**
 ****************************************************************************************
 * @brief Handle M_API_LPN_SELECT_FRIEND command
 *
 * @param[in] src_id    Source identifier of task that request command execution
 * @param[in] p_cmd     Command parameters
 ****************************************************************************************
 */
__STATIC uint16_t m_api_lpn_select_friend_cmd_handler(uint16_t src_id, m_api_lpn_select_friend_cmd_t *p_cmd)
{
    // Forward the command
    uint16_t status = m_api_lpn_select_friend(p_cmd->friend_addr);

    // Send back command status
    m_api_basic_cmp_evt_send(p_cmd->cmd_code, src_id, status);

    return (MESH_ERR_NO_ERROR);
}
#endif //(BLE_MESH_LPN)

#if (BLE_MESH_GATT_PROXY)
/**
 ****************************************************************************************
 * @brief Handle M_API_PROXY_CTL command
 *
 * @param[in] src_id    Source identifier of task that request command execution
 * @param[in] p_cmd     Command parameters
 ****************************************************************************************
 */
__STATIC uint16_t m_api_proxy_ctl_cmd_handler(uint16_t src_id, m_api_proxy_ctl_cmd_t *p_cmd)
{
    // Forward the command
    uint16_t status = m_api_proxy_ctrl(p_cmd->enable);

    // Send back command status
    m_api_basic_cmp_evt_send(p_cmd->cmd_code, src_id, status);

    return (MESH_ERR_NO_ERROR);
}
#endif // (BLE_MESH_GATT_PROXY)

#if (BLE_MESH_PROVISIONER)
__STATIC uint8_t m_api_prov_add_new_device_cmd_handler(uint16_t src_id, m_api_prov_add_new_device_cmd_t *p_cmd)
{
    // Propagate the response
    m_prov_pro_add_new_dev(p_cmd->dev_uuid, p_cmd->oob_info, p_cmd->uri_hash);

    return (MESH_ERR_NO_ERROR);
}

__STATIC uint8_t m_api_prov_start_dev_prov_cmd_handler(uint16_t src_id, m_api_prov_start_dev_prov_cmd_t *p_cmd)
{
    // Propagate the response

    m_prov_start_dev_prov(p_cmd->algorithm, p_cmd->pub_key, p_cmd->auth_method,
                          p_cmd->auth_action, p_cmd->auth_size, p_cmd->dev_pub_key);

    return (MESH_ERR_NO_ERROR);
}

__STATIC uint8_t m_api_confc_dev_compo_data_get_cmd_handler(uint16_t src_id, m_api_confc_dev_compo_data_get_cmd_t *p_cmd)
{
    // Propagate the response
    MESH_APP_PRINT_INFO("m_api_confc_dev_compo_data_get_cmd_handler\r\n");

    m_fnd_confc_send_compo_data_get(p_cmd->dev_lid, p_cmd->page);

    return (MESH_ERR_NO_ERROR);
}

__STATIC uint8_t m_api_confc_dev_app_key_add_cmd_handler(uint16_t src_id, m_api_confc_dev_app_key_add_cmd_t *p_cmd)
{
    // Propagate the response
    MESH_APP_PRINT_INFO("m_api_confc_dev_app_key_add_cmd_handler\r\n");

    uint16_t status;

    status = m_fnd_confc_send_app_key_add(p_cmd->dev_lid);

    return (status);
}

__STATIC uint8_t m_api_confc_model_app_bind_cmd_handler(uint16_t src_id, m_api_confc_model_app_bind_cmd_t *p_cmd)
{
    // Propagate the response
    MESH_APP_PRINT_INFO("m_api_confc_model_app_bind_cmd_handler\r\n");

    m_fnd_confc_send_model_app_bind(p_cmd->dev_lid, p_cmd->isSIG, p_cmd->model_id);

    return (MESH_ERR_NO_ERROR);
}

__STATIC uint8_t m_api_confc_model_app_subs_add_cmd_handler(uint16_t src_id, 
                                                                       m_api_confc_model_app_subs_add_cmd_t *p_cmd)
{
    MESH_APP_PRINT_INFO("%s\n");

    m_fnd_confc_model_subs_add(p_cmd->dev_lid, p_cmd->is_sig, p_cmd->model_lid, p_cmd->addr);

    return (MESH_ERR_NO_ERROR);
}

__STATIC uint8_t m_api_confc_model_app_subs_del_cmd_handler(uint16_t src_id,
                                                                       m_api_confc_model_app_subs_del_cmd_t *p_cmd)
{
    MESH_APP_PRINT_INFO("%s\n");

    m_fnd_confc_model_subs_del(p_cmd->dev_lid, p_cmd->is_sig, p_cmd->model_lid, p_cmd->addr);

    return (MESH_ERR_NO_ERROR);
}

__STATIC uint8_t m_api_confc_model_app_reset_node_cmd_handler(uint16_t src_id,
                                                                          m_api_confc_model_app_node_reset_cmd_t *p_cmd)
{
    MESH_APP_PRINT_INFO("%s\n");

    m_fnd_confc_send_node_reset(p_cmd->dev_lid);

    return (MESH_ERR_NO_ERROR);
}
#endif
/**
 ****************************************************************************************
 * @brief Handler of Command message hander
 *
 * @param[in] src_id    Identifier of the task that issue message
 *
 * @return Status of message after execution of the handler (@see enum mal_msg_status)
 ****************************************************************************************
 */
__STATIC uint8_t m_api_msg_exec_cmd(uint16_t src_id, m_api_cmd_t *p_cmd)
{
    // Returned status
    uint16_t status = MESH_ERR_NO_ERROR;
    // Message status
    uint8_t msg_status = MAL_MSG_FREE;

    switch (p_cmd->cmd_code)
    {
        case (M_API_ENABLE):
        {
            status = m_api_enable_cmd_handler(src_id);
        } break;

        case (M_API_DISABLE):
        {
            status = m_api_disable_cmd_handler(src_id);
        } break;

        case (M_API_REGISTER_MODEL):
        {
            MESH_APP_PRINT_INFO("M_API_REGISTER_MODEL\r\n");
            status = m_api_register_model_cmd_handler(src_id, (m_api_register_model_cmd_t *)p_cmd);
        } break;

        case (M_API_STORAGE_LOAD):
        {
            MESH_APP_PRINT_INFO("M_API_STORAGE_LOAD\r\n");
            status = m_api_storage_load_cmd_handler(src_id, (m_api_storage_load_cmd_t *)p_cmd);
        } break;

        case (M_API_STORAGE_SAVE):
        {
            MESH_APP_PRINT_INFO("M_API_STORAGE_SAVE\r\n");
            m_api_storage_save_cmd_handler(src_id);
        } break;

        case (M_API_STORAGE_CONFIG):
        {
            MESH_APP_PRINT_INFO("M_API_STORAGE_CONFIG\r\n");
            m_api_storage_config_cmd_handler(src_id, (m_api_storage_config_cmd_t *)p_cmd);
        } break;

        case (M_API_MODEL_PUBLISH):
        {
            MESH_APP_PRINT_INFO("M_API_STORAGE_SAVE\r\n");
            status = m_api_model_publish_cmd_handler(src_id, (m_api_model_publish_cmd_t *)p_cmd);
        } break;

        case (M_API_MODEL_RSP_SEND):
        {
            status = m_api_model_rsp_send_cmd_handler(src_id, (m_api_model_rsp_send_cmd_t *)p_cmd);
        } break;

        case (M_API_HEALTH_STATUS_SEND):
        {
            status = m_api_health_status_send_cmd_handler(src_id, (m_api_health_status_send_cmd_t *)p_cmd);
        } break;

        case (M_API_PROV_PUB_KEY_READ):
        {
            status = m_api_prov_pub_key_read_cmd_handler(src_id, (m_api_prov_pub_key_read_cmd_t *)p_cmd);
        } break;
#if (BLE_MESH_PROVISIONER)
        case (M_API_PROV_ADD_NEW_DEV):
        {
            status = m_api_prov_add_new_device_cmd_handler(src_id, (m_api_prov_add_new_device_cmd_t *)p_cmd);
        } break;

        case (M_API_PROV_START_DEV_PROV):
        {
            status = m_api_prov_start_dev_prov_cmd_handler(src_id, (m_api_prov_start_dev_prov_cmd_t *)p_cmd);
        } break;

        case (M_API_CONFC_DEV_COMPO_DATA_GET):
        {
            status = m_api_confc_dev_compo_data_get_cmd_handler(src_id, (m_api_confc_dev_compo_data_get_cmd_t *)p_cmd);
        } break;

        case (M_API_CONFC_DEV_APP_KEY_ADD):
        {
            status = m_api_confc_dev_app_key_add_cmd_handler(src_id, (m_api_confc_dev_app_key_add_cmd_t *)p_cmd);
        } break;

        case (M_API_CONFC_DEV_APP_BIND):
        {
            status = m_api_confc_model_app_bind_cmd_handler(src_id, (m_api_confc_model_app_bind_cmd_t *)p_cmd);
        } break;
        case (M_API_CONFC_DEV_APP_SUBS_ADD):
        {
            status = m_api_confc_model_app_subs_add_cmd_handler(src_id, (m_api_confc_model_app_subs_add_cmd_t *)p_cmd);
        }
        break;
        case M_API_CONFC_DEV_APP_SUBS_DEL:
        {
            status = m_api_confc_model_app_subs_del_cmd_handler(src_id, (m_api_confc_model_app_subs_del_cmd_t *)p_cmd);
        }
        break;
        case (M_API_CONFC_DEV_APP_NODE_RESET):
        {
            status = m_api_confc_model_app_reset_node_cmd_handler(src_id, (m_api_confc_model_app_node_reset_cmd_t *)p_cmd);
        }
        break;
#endif
        case (M_API_IV_UPD_TEST_MODE):
        {
            status = m_api_iv_upd_test_mode_cmd_handler(src_id, (m_api_iv_upd_test_mode_cmd_t *)p_cmd);
        } break;

#if (BLE_MESH_LPN)
        case (M_API_LPN_START):
        {
            status = m_api_lpn_start_cmd_handler(src_id, (m_api_lpn_start_cmd_t *)p_cmd);
        } break;

        case (M_API_LPN_STOP):
        {
            status = m_api_lpn_stop_cmd_handler(src_id);
        } break;

        case (M_API_LPN_SELECT_FRIEND):
        {
            status = m_api_lpn_select_friend_cmd_handler(src_id, (m_api_lpn_select_friend_cmd_t *)p_cmd);
        } break;
#endif //(BLE_MESH_LPN)

#if (BLE_MESH_GATT_PROXY)
        case (M_API_PROXY_CTL):
        {
            status = m_api_proxy_ctl_cmd_handler(src_id, (m_api_proxy_ctl_cmd_t *)p_cmd);
        } break;
#endif //(BLE_MESH_GATT_PROXY)

        default:
        {
            // Unknown request
            status = MESH_ERR_NOT_SUPPORTED;
        } break;
    }

    // Check if command has been rejected
    if (status != MESH_ERR_NO_ERROR)
    {
        // Inform application that command is rejected.
        m_api_basic_cmp_evt_send(p_cmd->cmd_code, src_id, status);
    }

    return (msg_status);
}

/**
 ****************************************************************************************
 * @brief Handler of Model opcode support response
 *
 * @param[in] src_id    Identifier of the task that issue message
 * @param[in] p_cfm     Parameters of the message received
 *
 * @return Status of message after execution of the handler (@see enum mal_msg_status)
 ****************************************************************************************
 */
__STATIC uint8_t m_api_model_opcode_cfm_handler(uint16_t src_id, m_api_model_opcode_cfm_t *p_cfm)
{
    // Operation code result
    m_api_model_opcode_status(p_cfm->model_lid, p_cfm->opcode, p_cfm->status);

    return (MAL_MSG_FREE);
}

/**
 ****************************************************************************************
 * @brief Handler of get composition data response
 *
 * @param[in] src_id    Identifier of the task that issue message
 * @param[in] p_cfm     Parameters of the message received
 *
 * @return Status of message after execution of the handler (@see enum mal_msg_status)
 ****************************************************************************************
 */
__STATIC uint8_t m_api_compo_data_cfm_handler(m_api_compo_data_cfm_t *p_cfm)
{
    // Propagate the response
    m_api_compo_data_cfm(p_cfm->page, p_cfm->length, &p_cfm->data[0]);

    return (MAL_MSG_FREE);
}

/**
 ****************************************************************************************
 * @brief Handler of get composition data response
 *
 * @param[in] src_id    Identifier of the task that issue message
 * @param[in] p_cfm     Parameters of the message received
 *
 * @return Status of message after execution of the handler (@see enum m_al_msg_status)
 ****************************************************************************************
 */
__STATIC uint8_t m_api_fault_cfm_handler(m_api_fault_cfm_t *p_cfm)
{
    // Propagate the response
    m_api_health_cfm(p_cfm->accept, p_cfm->comp_id, p_cfm->test_id, p_cfm->length,
                     &p_cfm->fault_array[0]);

    return (MAL_MSG_FREE);
}

/**
 ****************************************************************************************
 * @brief Handler of MESH_PROV_PARAM_CFM message
 *
 * @param[in] src_id    Identifier of the task that issue message
 * @param[in] p_cfm     Parameters of the message received
 *
 * @return Status of message after execution of the handler (@see enum m_al_msg_status)
 ****************************************************************************************
 */
#include <string.h>

__STATIC uint8_t m_api_prov_param_cfm_handler(m_api_prov_param_cfm_t *p_cfm)
{

    MESH_APP_PRINT_INFO("m_api_prov_param_cfm_handler\r\n");
    // Propagate the response
    m_api_prov_param_rsp(p_cfm);

    return (MAL_MSG_FREE);
}

/**
 ****************************************************************************************
 * @brief Handler of  Authentication Data response
 *
 * @param[in] src_id    Identifier of the task that issue message
 * @param[in] p_cfm     Parameters of the message received
 *
 * @return Status of message after execution of the handler (@see enum m_al_msg_status)
 ****************************************************************************************
 */
__STATIC uint8_t m_api_prov_auth_data_cfm_handler(m_api_prov_auth_data_cfm_t *p_cfm)
{
    // Propagate the response
    m_api_prov_oob_auth_rsp(p_cfm->accept, p_cfm->auth_size, p_cfm->auth_data);

    return (MAL_MSG_FREE);
}

/*
 * EXPORTED FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */

uint16_t m_api_msg_init(bool reset, void *p_env, const m_cfg_t *p_cfg)
{
    // Set callback functions for asynchronous events
    m_api_set(&m_api_msg_cb, &m_api_msg_fault_cb);

    // No environment
    return (0);
}

uint16_t m_api_msg_get_env_size(const m_cfg_t *p_cfg)
{
    // No environment
    return (0);
}

uint8_t m_api_msg_handler(uint16_t msg_id, uint16_t src_id, const void *p_param)
{

    uint8_t msg_status = MAL_MSG_FREE;
    switch (msg_id)
    {
        // Mesh command
        case (MESH_API_CMD):
        {
            msg_status = m_api_msg_exec_cmd(src_id, (m_api_cmd_t *)p_param);
        } break;

        // Operation code confirmation
        case (MESH_API_MODEL_OPCODE_CFM):
        {
            msg_status = m_api_model_opcode_cfm_handler(src_id, (m_api_model_opcode_cfm_t *)p_param);
        } break;

        case (MESH_API_COMPO_DATA_CFM):
        {
            msg_status = m_api_compo_data_cfm_handler((m_api_compo_data_cfm_t *)p_param);
        } break;

        case (MESH_API_FAULT_CFM):
        {
            msg_status = m_api_fault_cfm_handler((m_api_fault_cfm_t *)p_param);
        } break;

        case (MESH_API_PROV_PARAM_CFM):
        {
            msg_status = m_api_prov_param_cfm_handler((m_api_prov_param_cfm_t *)p_param);
        } break;

        case (MESH_API_PROV_AUTH_DATA_CFM):
        {
            msg_status = m_api_prov_auth_data_cfm_handler((m_api_prov_auth_data_cfm_t *)p_param);
        } break;

        // Unknown message
        default:
        {
            ASSERT_WARN(0, msg_id, src_id);
        } break;
    }

    return (msg_status);
}


#endif // (BLE_MESH_MSG_API)

/// @} M_API_MSG
