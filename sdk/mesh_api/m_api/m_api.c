/**
 ****************************************************************************************
 *
 * @file m_api.c
 *
 * @brief Mesh Profile Application Programming Interface
 *
 * Copyright (C) RivieraWaves 2017-2019
 *
 ****************************************************************************************
 */


/**
 ****************************************************************************************
 * @addtogroup M_API
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "m_inc.h"          // Mesh Profile Include Files
#include "m_api.h"          // Mesh Application Programming Interface Internals
#include "m_tb.h"           // Mesh Toolboxes
#include "m_lay.h"          // Mesh Communication Layers
#include "m_fnd.h"          // Mesh Foundation Models
#include "m_prov.h"         // Mesh Provisioning Layer
#include "m_bcn.h"          // Mesh Beaconing Layer
#include "m_bearer.h"       // Mesh Bearer Layer
#include "m_tb_key.h"
#include "m_prov_int.h"

/// Callback used to discuss with a native application
const m_api_cb_t *p_m_api_cb;
/// Callback used to discuss with a native application
const m_api_fault_cb_t *p_m_api_fault_cb;

/*
 * EXTERNAL FUNCTIONS DECLARATION
 ****************************************************************************************
 */

#if (BLE_MESH_MSG_API)
uint16_t m_api_msg_init(bool reset, void *p_env, const m_cfg_t *p_cfg);
uint16_t m_api_msg_get_env_size(const m_cfg_t *p_cfg);
uint8_t m_api_msg_handler(uint16_t msg_id, uint16_t src_id, const void *p_param);
#endif // (BLE_MESH_MSG_API)

/*
 * LOCAL FUNCTIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief TODO [LT]
 ****************************************************************************************
 */
__STATIC void m_api_bearer_started_cb(uint16_t status)
{
    MESH_APP_PRINT_INFO("======== %s, status = %d \n", __func__, status);
#if (BLE_MESH_PROVISIONER)
    if (status == MESH_ERR_NO_ERROR)
    {
        if (m_tb_state_get_prov_state() == M_TB_STATE_PROV_STATE_UNPROV)
        {
            // Start provisioner
            status = m_prov_pro_start();
        }
#if 0
        else
        {
            status = MESH_ERR_PROV_PROHIBITED;
        }
#endif
    }
#else
    do
    {
        if (status == MESH_ERR_NO_ERROR)
        {
            if (m_tb_state_get_prov_state() == M_TB_STATE_PROV_STATE_UNPROV)
            {
                // Start provisioning
                status = m_prov_start();

                if (status == MESH_ERR_NO_ERROR)
                {
                    break;
                }
            }
        }

        if (status == MESH_ERR_NO_ERROR)
        {
            // update beacon state machine
            m_bcn_state_update();
        }
    }
    while (0);
#endif

    if (status == MESH_ERR_NO_ERROR)
    {
        m_tb_state_set_enabled(true);
    }

    p_m_api_cb->cb_enabled(status);
}

/**
 ****************************************************************************************
 * @brief TODO [LT]
 ****************************************************************************************
 */
__STATIC void m_api_bearer_stopped_cb(uint16_t status)
{
    MESH_APP_PRINT_INFO("m_api_bearer_stopped_cb stu :%d", status);
    if (status == MESH_ERR_NO_ERROR)
    {
        m_tb_state_set_enabled(false);
    }

    p_m_api_cb->cb_disabled(status);
}

/**
 ****************************************************************************************
 * @brief TODO [LT]
 ****************************************************************************************
 */
__STATIC void m_api_storage_load_cb(uint16_t status)
{
    p_m_api_cb->cb_loaded(status);
}

/*
 * GLOBAL FUNCTIONS
 ****************************************************************************************
 */

uint16_t m_api_set(const m_api_cb_t *p_cb_api, const m_api_fault_cb_t *p_fault_cb_api)
{
    uint16_t status = MESH_ERR_INVALID_PARAM;

    // Sanity check to ensure that all callbacks are configured
    if (p_cb_api
            && p_cb_api->cb_enabled
            && p_cb_api->cb_disabled
            && p_cb_api->cb_prov_state
            && p_cb_api->cb_prov_param_req
            && p_cb_api->cb_prov_auth_req
            && p_cb_api->cb_loaded
            && p_cb_api->cb_attention
            && p_cb_api->cb_compo_data
            && p_cb_api->cb_node_reset
#if (BLE_MESH_LPN)
            && p_cb_api->cb_lpn_status
            && p_cb_api->cb_lpn_offer
#endif //(BLE_MESH_LPN)
#if (BLE_MESH_GATT_PROXY)
            && p_cb_api->cb_proxy_adv_update
#endif //(BLE_MESH_GATT_PROXY)
            && p_fault_cb_api
            && p_fault_cb_api->cb_fault_get
            && p_fault_cb_api->cb_fault_test
            && p_fault_cb_api->cb_fault_clear
            && p_fault_cb_api->cb_fault_period
       )
    {
        p_m_api_cb = p_cb_api;
        p_m_api_fault_cb = p_fault_cb_api;
        status = MESH_ERR_NO_ERROR;
    }
    else
    {
        ASSERT_ERR(0);
    }

    return (status);
}

uint16_t m_api_enable(void)
{
    uint16_t status = MESH_ERR_COMMAND_DISALLOWED;
    MESH_APP_PRINT_INFO("m_api_enable,m_tb_state_is_enabled = 0x%x\r\n", m_tb_state_is_enabled());
    MESH_APP_PRINT_INFO("p_m_api_cb = 0x%x\r\n", p_m_api_cb);
    if ((p_m_api_cb != NULL) && !m_tb_state_is_enabled())
    {
        // Enable bearer layer and return the status
        status = m_bearer_start(M_BEARER_TYPE_ADV, 0, NULL, m_api_bearer_started_cb);
    }

    return (status);
}

uint16_t m_api_disable(void)
{
    uint16_t status = MESH_ERR_COMMAND_DISALLOWED;
    MESH_APP_PRINT_INFO("m_api_disable p_m_api_cb:%x,state_is_enable: 0x%x\r\n", p_m_api_cb, m_tb_state_is_enabled());
    // Check that profile is well enabled
    if ((p_m_api_cb != NULL) && m_tb_state_is_enabled())
    {
        // Disable bearer layer and return the status
        status = m_bearer_stop(M_BEARER_TYPE_ADV, m_api_bearer_stopped_cb);
    }

    return (status);
}

uint16_t m_api_buf_alloc(m_api_buf_t **pp_buf, uint16_t size)
{
    // Allocate a buffer
    return (mesh_tb_buf_alloc((mesh_tb_buf_t **)pp_buf, M_LAY_ACCESS_BUF_HEADER_LEN, size,
                              M_LAY_ACCESS_BUF_TAIL_LEN));
}

uint16_t m_api_buf_release(m_api_buf_t *p_buf)
{
    // Release provided buffer
    return (mesh_tb_buf_release((mesh_tb_buf_t *)p_buf));
}

uint16_t m_api_model_publish(m_lid_t model_lid, uint32_t opcode, uint8_t tx_hdl, m_api_buf_t *p_buf,
                             bool trans_mic_64)
{
    // Provide the buffer to Access Layer
    return (m_lay_access_publish(model_lid, opcode, tx_hdl, (mesh_tb_buf_t *)p_buf, trans_mic_64));
}

uint16_t m_api_model_rsp_send(m_lid_t model_lid, uint32_t opcode, uint8_t tx_hdl, m_api_buf_t *p_buf,
                              m_lid_t key_lid, uint16_t dst, bool trans_mic_64, bool not_relay)
{
    // Provide the buffer to Access Layer
    return (m_lay_access_rsp_send(model_lid, opcode, tx_hdl, (mesh_tb_buf_t *)p_buf, key_lid, dst,
                                  trans_mic_64, not_relay));
}

void m_api_model_opcode_status(m_lid_t model_lid, uint32_t opcode, uint16_t status)
{
    // Provide the status to Access Layer
    m_lay_access_opcode_status(model_lid, opcode, status);
}

uint16_t m_api_register_model(uint32_t model_id, uint8_t elmt_idx, uint8_t config,
                              const m_api_model_cb_t *p_cb, m_lid_t *p_model_lid)
{
    // Status
    uint16_t status = MESH_ERR_COMMAND_DISALLOWED;;

    // Reject the command if stack is enabled
    if (!m_tb_state_is_enabled())
    {
        // Register the model instance
        status = m_tb_mio_register_model(model_id, elmt_idx, config, p_cb, p_model_lid);
    }


    return (status);
}

uint16_t m_api_storage_load(uint16_t length, uint8_t *p_data)
{
    // Status
    uint16_t status = MESH_ERR_COMMAND_DISALLOWED;

    // Reject the command if stack is enabled
    if (!m_tb_state_is_enabled())
    {
        status = m_tb_store_load(length, p_data, m_api_storage_load_cb);
    }

    return (status);
}

uint16_t m_api_storage_save(void)
{
    MESH_APP_PRINT_INFO("%s\r\n", __func__);

    return (m_tb_store_save());
}

uint16_t m_api_storage_config(uint32_t config)
{
    MESH_APP_PRINT_INFO("%s\r\n", __func__);
    return (m_tb_store_config(config));
}

void m_api_iv_upd_test_mode(bool update)
{
    // Inform the key manager about the request
    m_tb_key_iv_upd_test_mode_ind(update);
}

void m_api_health_status_send(uint16_t comp_id, uint8_t test_id, uint8_t length, uint8_t *p_fault_array)
{
    m_fnd_hlths_status_ind(comp_id, test_id, length, p_fault_array);
}
 
void m_api_health_cfm(bool accept, uint16_t comp_id, uint8_t test_id, uint8_t length,
                      uint8_t *p_fault_array)
{
    m_fnd_hlths_cfm(accept, comp_id, test_id, length, p_fault_array);
}
 
void m_api_compo_data_cfm(uint8_t page, uint8_t length, uint8_t *p_data)
{
    // Inform the storage manager about received application response
    m_tb_store_rx_compo_data(page, length, p_data);
}

// -----------------------------------------------------------------------------
//                    Provisioning API
// -----------------------------------------------------------------------------

void m_api_prov_param_rsp(void *p_param)
{
    m_prov_param_rsp(p_param);
}

void m_api_prov_oob_auth_rsp(bool accept, uint8_t auth_size, const uint8_t *p_auth_data)
{
    m_prov_oob_auth_rsp(accept, auth_size, p_auth_data);
}

uint16_t m_api_prov_pub_key_read(uint8_t *p_pub_key_x, uint8_t *p_pub_key_y)
{
    MESH_APP_PRINT_INFO("m_api_prov_pub_key_read\r\n");
    return (m_prov_pub_key_get(p_pub_key_x, p_pub_key_y));
}
#if (BLE_MESH_LPN)
uint16_t m_api_lpn_start(uint32_t poll_timeout, uint32_t poll_intv_ms, uint16_t prev_addr, uint8_t rx_delay,
                         uint8_t rssi_factor, uint8_t rx_window_factor, uint8_t min_queue_size_log)
{
    return (m_lay_lpn_start(poll_timeout, poll_intv_ms, prev_addr, rx_delay, rssi_factor,
                            rx_window_factor, min_queue_size_log));
}

uint16_t m_api_lpn_stop(void)
{
    return (m_lay_lpn_stop());
}

uint16_t m_api_lpn_select_friend(uint16_t friend_addr)
{
    return (m_lay_lpn_select_friend(friend_addr));
}
#endif //(BLE_MESH_LPN)

#if (BLE_MESH_GATT_PROXY)
// -----------------------------------------------------------------------------
//                    GATT Proxy API
// -----------------------------------------------------------------------------

uint16_t m_api_proxy_ctrl(uint8_t enable)
{
    return (m_lay_proxy_bearer_adv_ctrl(enable, MESH_INVALID_LID, MESH_PROXY_ADV_UPD_REASON_USER));
}
#endif // (BLE_MESH_GATT_PROXY)


uint8_t m_api_handler(uint16_t msg_id, uint16_t src_id, const void *p_param)
{

    uint8_t msg_status = MAL_MSG_FREE;

    // Mesh Message API
    if (msg_id < MESH_DBG_FIRST)
    {
#if (BLE_MESH_MSG_API)
        msg_status = m_api_msg_handler(msg_id, src_id, p_param);
#else // !(BLE_MESH_MSG_API)
        ASSERT_WARN(0, msg_id, src_id);
#endif // (BLE_MESH_MSG_API)
    }
    // Mesh Debug API
    else
    {
#if (BLE_MESH_DBG)
        msg_status = m_dbg_msg_handler(msg_id, src_id, p_param);
#else // !(BLE_MESH_DBG)
        ASSERT_WARN(0, msg_id, src_id);
#endif // (BLE_MESH_DBG)
    }

    return (msg_status);
}


uint16_t m_api_init(bool reset, void *p_env, const m_cfg_t *p_cfg)
{
    // Stack initialization
    uint8_t *p_env_cursor = (uint8_t *)p_env;

    // Initialize callbacks
    p_m_api_cb = NULL;


    p_env_cursor += CO_ALIGN4_HI(m_tb_init(reset, (void *) p_env_cursor, p_cfg));

    p_env_cursor += CO_ALIGN4_HI(m_bearer_init(reset, (void *) p_env_cursor, p_cfg));

    p_env_cursor += CO_ALIGN4_HI(m_lay_init(reset, (void *) p_env_cursor, p_cfg));

    p_env_cursor += CO_ALIGN4_HI(m_prov_init(reset, (void *) p_env_cursor, p_cfg));

    p_env_cursor += CO_ALIGN4_HI(m_fnd_init(reset, (void *) p_env_cursor, p_cfg));

    p_env_cursor += CO_ALIGN4_HI(m_bcn_init(reset, (void *) p_env_cursor, p_cfg));


#if (BLE_MESH_DBG)
    p_env_cursor += CO_ALIGN4_HI(m_dbg_init(reset, (void *) p_env_cursor, p_cfg));
#endif // (BLE_MESH_DBG)

    if (!reset)
    {
#if (BLE_MESH_MSG_API)
        // Use the message-based callback if requested by application
        if (p_cfg->features & M_FEAT_MSG_API_SUP)
        {
            m_api_msg_init(reset, (void *)p_env_cursor, p_cfg);
        }
#endif //(BLE_MESH_MSG_API)
    }

    // Return size of the environment used
    return (((uint32_t)p_env_cursor) - ((uint32_t)p_env));
}

uint16_t m_api_get_env_size(const m_cfg_t *p_cfg)
{
    uint16_t total_env_size = 0;

    // Retrieve size of environment variable required for the mesh profile
    total_env_size += CO_ALIGN4_HI(m_tb_get_env_size(p_cfg));
    total_env_size += CO_ALIGN4_HI(m_bearer_get_env_size(p_cfg));
    total_env_size += CO_ALIGN4_HI(m_lay_get_env_size(p_cfg));
    total_env_size += CO_ALIGN4_HI(m_prov_get_env_size(p_cfg));
    total_env_size += CO_ALIGN4_HI(m_fnd_get_env_size(p_cfg));
    total_env_size += CO_ALIGN4_HI(m_bcn_get_env_size(p_cfg));


#if (BLE_MESH_DBG)
    total_env_size += CO_ALIGN4_HI(m_dbg_get_env_size(p_cfg));
#endif //(BLE_MESH_DBG)

#if (BLE_MESH_MSG_API)
    // Use the message-based callback if requested by application
    if (p_cfg->features & M_FEAT_MSG_API_SUP)
    {
        m_api_msg_get_env_size(p_cfg);
    }
#endif //(BLE_MESH_MSG_API)

    return (total_env_size);
}
void m_api_send_attention_update_ind(uint8_t attention_state)
{
    p_m_api_cb->cb_attention(attention_state);
}

void m_api_send_compo_data_req_ind(uint8_t page)
{
    p_m_api_cb->cb_compo_data(page);
}

void m_api_send_fault_get_req_ind(uint16_t comp_id)
{
    p_m_api_fault_cb->cb_fault_get(comp_id);
}

void m_api_send_fault_test_req_ind(uint16_t comp_id, uint8_t test_id, bool cfm_needed)
{
    p_m_api_fault_cb->cb_fault_test(comp_id, test_id, cfm_needed);
}

void m_api_send_fault_clear_ind(uint16_t comp_id)
{
    p_m_api_fault_cb->cb_fault_clear(comp_id);
}

void m_api_send_fault_period_ind(uint32_t period_ms, uint32_t period_fault_ms)
{
    p_m_api_fault_cb->cb_fault_period(period_ms, period_fault_ms);
}

void m_api_send_update_ind(uint8_t upd_type, uint16_t length, uint8_t *p_data)
{
    // Allocate message
    m_api_update_ind_t *p_ind = MAL_MSG_ALLOC_DYN(MESH_API_UPDATE_IND, m_api_update_ind_t, length);

    // Fill the message
    p_ind->upd_type = upd_type;
    p_ind->length = length;
    memcpy(&p_ind->data[0], p_data, length);

    // Send the message
    mal_msg_send(mal_app_id_get(), p_ind);
}




void m_api_send_node_reset_ind(void)
{
    p_m_api_cb->cb_node_reset();
}

#if (BLE_MESH_LPN)
void m_api_send_lpn_status_ind(uint16_t status, uint16_t friend_addr)
{
    p_m_api_cb->cb_lpn_status(status, friend_addr);
}

void m_api_send_lpn_offer_ind(uint16_t friend_addr, uint8_t rx_window,
                              uint8_t queue_size, uint8_t subs_list_size, int8_t rssi)
{
    p_m_api_cb->cb_lpn_offer(friend_addr, rx_window, queue_size, subs_list_size, rssi);
}
#endif //(BLE_MESH_LPN)

#if (BLE_MESH_GATT_PROXY)
void m_api_send_proxy_adv_update_ind(uint8_t state, uint8_t reason)
{
    p_m_api_cb->cb_proxy_adv_update(state, reason);
}
#endif //(BLE_MESH_GATT_PROXY)

#if (BLE_MESH_MSG_API)
void m_api_model_set_task_id(m_lid_t model_lid, uint16_t task_id)
{
    m_tb_mio_set_task_id(model_lid, task_id);
}

uint16_t m_api_model_get_task_id(m_lid_t model_lid)
{
    return (m_tb_mio_get_task_id(model_lid));
}
#endif //(BLE_MESH_MSG_API)

#if 0//(BLE_MESH_LPN)

// -----------------------------------------------------------------------------
//                    Low Power Node API
// -----------------------------------------------------------------------------

uint16_t m_api_lpn_start(uint32_t poll_timeout, uint32_t poll_intv_ms, uint8_t rx_delay, uint8_t rssi_factor,
                         uint8_t rx_window_factor, uint8_t min_queue_size_log)
{
    return (m_lay_lpn_start(poll_timeout, poll_intv_ms, rx_delay, rssi_factor,
                            rx_window_factor, min_queue_size_log));
}

uint16_t m_api_lpn_stop(void)
{
    return (m_lay_lpn_stop());
}

uint16_t m_api_lpn_select_friend(uint16_t friend_addr)
{
    return (m_lay_lpn_select_friend(friend_addr));
}
#endif //(BLE_MESH_LPN)




void m_api_send_model_app_bind_ind(uint8_t status, uint32_t model_id, uint16_t element_addr, m_lid_t model_lid, m_lid_t app_key_lid)
{
    MESH_APP_PRINT_INFO("%s\r\n", __func__);
    // Allocate message
    m_api_model_app_bind_ind_t *p_ind = MAL_MSG_ALLOC(MESH_API_MODEL_APP_BIND_IND, m_api_model_app_bind_ind_t);

    // Fill the message
    p_ind->status = status;
    p_ind->model_id = model_id;
    p_ind->element_addr = element_addr;
    p_ind->model_lid = model_lid;
    p_ind->app_key_lid = app_key_lid;
    // Send the message
    mal_msg_send(TASK_APP, p_ind);
}

void m_api_send_key_ind(uint8_t key_type, const void *key)
{
    MESH_APP_PRINT_INFO("%s\r\n", __func__);
    switch (key_type)
    {
        case M_TB_KEY_DEVICE:
        {
            m_tb_key_dev_t *p_key = MAL_MSG_ALLOC(MESH_API_KEY_IND, m_tb_key_dev_t);

            memcpy(p_key, key, sizeof(m_tb_key_dev_t));

            mal_msg_send(TASK_APP, p_key);
        } break;

        case M_TB_KEY_NETWORK:
        {
            m_tb_key_net_t *p_key = MAL_MSG_ALLOC(MESH_API_KEY_IND, m_tb_key_net_t);

            memcpy(p_key, key, sizeof(m_tb_key_net_t));

            mal_msg_send(TASK_APP, p_key);
        } break;

        case M_TB_KEY_APPLICATION:
        {
            m_tb_key_app_t *p_key = MAL_MSG_ALLOC(MESH_API_KEY_IND, m_tb_key_app_t);

            memcpy(p_key, key, sizeof(m_tb_key_app_t));

            mal_msg_send(TASK_APP, p_key);
        } break;


        default: break;

    }

}


void m_api_send_model_subs_ind(uint8_t status, uint32_t model_id, uint16_t element_addr, uint16_t subs_addr, m_lid_t model_lid)
{
    MESH_APP_PRINT_INFO("%s,subs_addr = 0x%x\r\n", __func__,subs_addr);
    // Allocate message
    m_api_model_subs_ind_t *p_ind = MAL_MSG_ALLOC(MESH_API_MODEL_SUBS_IND, m_api_model_subs_ind_t);


    // Fill the message
    p_ind->status = status;
    p_ind->model_id = model_id;
    p_ind->element_addr = element_addr;
    p_ind->subs_addr = subs_addr;
    p_ind->model_lid = model_lid;

    // Send the message
    mal_msg_send(TASK_APP, p_ind);
}

#if (BLE_MESH_PROVISIONER)
uint16_t m_api_add_network_key(uint16_t net_key_id, uint8_t *netkey)
{
    uint16_t status = MESH_ERR_NO_ERROR;
    m_prov_res_t *p_res = p_m_prov_env->p_res;
    MESH_APP_PRINT_INFO("%s, p_res = %p\n", __func__, p_res);

    if (!p_res)
    {
        return MESH_ERR_INVALID_PARAM;
        //m_prov_start();
    }

    m_lid_t net_key_lid;
    if (m_tb_key_net_find(net_key_id, &net_key_lid) == MESH_ERR_NO_ERROR)
    {
        // Set the network key id and lid
        //p_res->state == M_PROV_PSTATE_SCAN;
        p_res->nwk_key_id = net_key_id;
        p_res->nwk_key_lid = net_key_lid;
        memcpy(p_res->net_key, netkey, MESH_KEY_LEN);
        return status;
    }

    //if (netkey && p_res && (p_res->nwk_key_id == 0xff))
    if (netkey && p_res)
    {
        p_res->nwk_key_id = net_key_id;
        memcpy(p_res->net_key, netkey, MESH_KEY_LEN);
        status = m_prov_net_key_add(p_res->nwk_key_id, p_res->net_key, 0);
        MESH_APP_PRINT_INFO("%s, status = 0x%x\n", __func__, status);
    }
    else
    {
        MESH_APP_PRINT_INFO("%s, invalid netkey id. \n", __func__);
        status = MESH_ERR_INVALID_NETKEY_ID;
    }

    return status;
}

uint16_t m_api_add_app_key(uint16_t app_key_id, uint8_t *app_key, uint16_t net_key_id)
{
    uint16_t status = MESH_ERR_NO_ERROR;
    m_prov_res_t *p_res = p_m_prov_env->p_res;

    //if (app_key && p_res && (p_res->app_key_id == 0xff))
    if (app_key && p_res)
    {
        p_res->nwk_key_id = net_key_id;
        p_res->app_key_id = app_key_id;
        memcpy(p_res->app_key, app_key, MESH_KEY_LEN);
    }
    else
    {
        MESH_APP_PRINT_WARN("%s, invalid appkey id. \n", __func__);
        status = MESH_ERR_INVALID_APPKEY_ID;
    }

    return status;
    //m_tb_key_app_add(app_key_id, app_key, net_key_id,
    //                      m_tb_key_added_cb res_cb)
}
#endif //BLE_MESH_PROVISIONER

/// @} M_API
