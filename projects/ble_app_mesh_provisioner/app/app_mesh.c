/**
 ****************************************************************************************
 *
 * @file app_mesh.c
 *
 * @brief mesh Application Module entry point
 *
 * Copyright (C) Beken 2019-2020
 *
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup APP
 * @{
 ****************************************************************************************
 */

#include "rwip_config.h"     // SW configuration

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include <string.h>
#include "app_mesh.h"                // Bracese Application Module Definitions
#include "app_mm_msg.h"                // Bracese Application Module Definitions
#include "m_api.h"
#include "app.h"                     // Application Definitions
#include "app_task.h"                // application task definitions
#include "co_bt.h"
#include "prf_types.h"               // Profile common types definition
#include "arch.h"                    // Platform Definitions
#include "prf.h"
#include "lld_evt.h"
#include "uart.h"
#include "m_api.h"
#include "mesh_api_msg.h"
#include "mal.h"
#include "m_bcn.h"
#include "m_prov_int.h"     // Mesh Provisioning Internal Defines
#include "led.h"
#include "wdt.h"
#include "app_light_ali_server.h"
#include "ali_config.h"
#include "mm_vendors.h"
#include "m_fnd_Scenes.h"
#include "m_fnd_int.h"
#include "mesh_param_int.h"

#include "mesh_tb_timer.h"

#include "mm_defines.h"
#include "mm_tb_state.h"
#include "mesh_general_api.h"

#define UN_PROV_DEV_MAX_NUM 1


/// Invites a device to join a mesh network
typedef struct unprov_dev_info
{
    uint8_t fActive;
    /// Device UUID
    uint8_t dev_uuid[MESH_DEV_UUID_LEN];
    /// OOB information
    uint16_t oob_info;
    uint32_t  uri_hash;
    m_lid_t  dev_lid;

} unprov_dev_info_t;

/*
 * LOCATION FUN DEFINES
 ****************************************************************************************
 */



/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

/// braces Application Module Environment Structure
struct app_mesh_env_tag app_mesh_env;

unprov_dev_info_t app_mesh_unprov_dev[UN_PROV_DEV_MAX_NUM];
uint8_t current_prov_dev_index;
uint16_t app_subs_addr = 0xc000;
bool is_onoff_loop_start = false;

/*
 * GLOBAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */
static void app_mesh_on_onff_loop(m_lid_t dev_lid);

void app_mesh_add_new_dev(void)
{

    MESH_APP_PRINT_INFO("app_mesh_add_new_dev ,msgid:%x\n", MESH_API_CMD);

    m_api_prov_add_new_device_cmd_t *cmd = MAL_MSG_ALLOC(MESH_API_CMD, m_api_prov_add_new_device_cmd_t);
    cmd->cmd_code = M_API_PROV_ADD_NEW_DEV;

    memcpy(cmd->dev_uuid, app_mesh_unprov_dev[0].dev_uuid, MESH_DEV_UUID_LEN);
    cmd->oob_info = app_mesh_unprov_dev[0].oob_info;
    cmd->uri_hash = app_mesh_unprov_dev[0].uri_hash;

    mal_msg_send(prf_get_task_from_id(TASK_ID_MESH), cmd);
    //crshFlag = 1;
    current_prov_dev_index = 0;
}

//app layer need to choose supported auth method
void app_mesh_start_dev_prov(void)
{

    MESH_APP_PRINT_INFO("app_mesh_start_dev_prov\n");

    m_api_prov_start_dev_prov_cmd_t *cmd = MAL_MSG_ALLOC(MESH_API_CMD, m_api_prov_start_dev_prov_cmd_t);
    cmd->cmd_code = M_API_PROV_START_DEV_PROV;

    cmd->algorithm = 0;
    cmd->pub_key = 0; // no oob pub key
    cmd->auth_method = 1; // Static oob authentication is used
    cmd->auth_action = 0;
    cmd->auth_size = M_OOB_AUTH_DATA_LEN;
    cmd->dev_pub_key = NULL;

    mal_msg_send(prf_get_task_from_id(TASK_ID_MESH), cmd);
    //crshFlag = 1;
}

void app_mesh_dev_compo_data_get(m_lid_t dev_lid)
{

    MESH_APP_PRINT_INFO("app_mesh_dev_compo_data_get\n");

    m_api_confc_dev_compo_data_get_cmd_t *cmd = MAL_MSG_ALLOC(MESH_API_CMD, m_api_confc_dev_compo_data_get_cmd_t);

    cmd->cmd_code = M_API_CONFC_DEV_COMPO_DATA_GET;
    cmd->dev_lid = dev_lid;
    cmd->page = 0xFF;

    mal_msg_send(prf_get_task_from_id(TASK_ID_MESH), cmd);
}

void app_mesh_dev_add_app_key(m_lid_t dev_lid)
{
    MESH_APP_PRINT_INFO("app_mesh_dev_add_app_key\n");

    m_api_confc_dev_app_key_add_cmd_t *cmd = MAL_MSG_ALLOC(MESH_API_CMD, m_api_confc_dev_app_key_add_cmd_t);

    cmd->cmd_code = M_API_CONFC_DEV_APP_KEY_ADD;
    cmd->dev_lid = dev_lid;

    mal_msg_send(prf_get_task_from_id(TASK_ID_MESH), cmd);
}


void app_mesh_model_app_bind(m_lid_t dev_lid)
{
    MESH_APP_PRINT_INFO("app_mesh_model_app_bind\n");

    m_api_confc_model_app_bind_cmd_t *cmd = MAL_MSG_ALLOC(MESH_API_CMD, m_api_confc_model_app_bind_cmd_t);

    cmd->cmd_code = M_API_CONFC_DEV_APP_BIND;
    cmd->dev_lid = dev_lid;
    cmd->isSIG = true;
    cmd->model_id = MM_ID_GENS_OO;

    mal_msg_send(prf_get_task_from_id(TASK_ID_MESH), cmd);
}

extern m_lid_t g_vdr_lid;
void app_mesh_model_subs_add(m_lid_t dev_lid, bool is_sig, uint16_t addr)
{
    MESH_APP_PRINT_INFO("%s, oo model lid %d\n", __func__, g_oo_mdl_lid);
    m_api_confc_model_app_subs_add_cmd_t *cmd = MAL_MSG_ALLOC(MESH_API_CMD, m_api_confc_model_app_subs_add_cmd_t);

    if (cmd)
    {
        cmd->cmd_code = M_API_CONFC_DEV_APP_SUBS_ADD;
        cmd->dev_lid = dev_lid;
        cmd->is_sig = is_sig;
        cmd->model_lid = g_vdr_lid;//g_oo_mdl_lid;
        cmd->addr = addr;

        mal_msg_send(prf_get_task_from_id(TASK_ID_MESH), cmd);
    }

    return;
}

void app_mesh_model_subs_del(m_lid_t dev_lid, bool is_sig, uint16_t addr)
{
    MESH_APP_PRINT_INFO("%s, oo model lid %d\n", __func__, g_oo_mdl_lid);

    m_api_confc_model_app_subs_del_cmd_t *cmd = MAL_MSG_ALLOC(MESH_API_CMD, m_api_confc_model_app_subs_del_cmd_t);

    if (cmd)
    {
        cmd->cmd_code = M_API_CONFC_DEV_APP_SUBS_DEL;
        cmd->dev_lid = dev_lid;
        cmd->is_sig = is_sig;
        cmd->model_lid = g_oo_mdl_lid;
        cmd->addr = addr;

        mal_msg_send(prf_get_task_from_id(TASK_ID_MESH), cmd);
    }

    return;
}

void app_mesh_node_reset(m_lid_t dev_lid)
{
    MESH_APP_PRINT_INFO("%s, dev_lid %d\n", __func__, dev_lid);
    m_api_confc_model_app_node_reset_cmd_t *cmd = MAL_MSG_ALLOC(MESH_API_CMD, m_api_confc_model_app_node_reset_cmd_t);

    if (cmd)
    {
        cmd->cmd_code = M_API_CONFC_DEV_APP_NODE_RESET;
        cmd->dev_lid = dev_lid;

        mal_msg_send(prf_get_task_from_id(TASK_ID_MESH), cmd);
    }

    return;
}

void app_mesh_set_on_off(m_lid_t dev_lid, uint8_t onoff)
{
    MESH_APP_PRINT_INFO("app_mesh_set_on_off\n");

    mm_api_cli_transition_cmd_t *cmd = MAL_MSG_ALLOC(MESH_MDL_API_CMD, mm_api_cli_transition_cmd_t);
    m_tb_key_dev_info_t* p_dev_info;
    uint16_t status;
    uint16_t subs_nb_addr;
    m_lid_t sub_mdl_lid = mm_tb_state_get_lid(0, MM_ID_VENDORS);
    memset(cmd, 0, sizeof(mm_api_cli_transition_cmd_t));
    p_dev_info = m_tb_key_dev_info_get(dev_lid);

    cmd->cmd_code = MM_API_CLI_TRANSITION;
    cmd->state_1 = onoff;
    cmd->mdl_lid = mm_tb_state_get_lid(0, MM_ID_VENDORS);
    subs_nb_addr = m_tb_mio_get_subscription_list_size(sub_mdl_lid);
    if (0 == subs_nb_addr)
    {
        MESH_APP_PRINT_WARN("%s, the subscription list is NULL.\n", __func__);
        return;
    }
    
    uint8_t *subs_addr = mal_malloc(subs_nb_addr * M_ADDR_LEN);
    if (subs_addr)
    {
        m_tb_mio_get_subscription_list(sub_mdl_lid, subs_addr, false);
    }
    else
    {
        MESH_APP_PRINT_WARN("%s, buffer alloc fail.\n", __func__);
        return;
    }
    
    cmd->dst = co_read16p(subs_addr);
    MESH_APP_PRINT_INFO("subs_nb_addr = %d\n", subs_nb_addr);
    MESH_APP_PRINT_INFO("mdl_lid=%d\n", cmd->mdl_lid);
    MESH_APP_PRINT_INFO("dst = 0x%x\n", cmd->dst);
    MESH_APP_PRINT_INFO("app_key_lid = %d\n", p_dev_info->app_key_lid);
    // Bind the application key with the model
    status = m_tb_key_model_bind(p_dev_info->app_key_lid, cmd->mdl_lid);
    MESH_APP_PRINT_INFO("mdl_lid = %d, status = 0x%x\n", cmd->mdl_lid, status);
    if (status == MESH_ERR_NO_ERROR)
    {
        // Inform the Model IO manager
        m_tb_mio_bind(cmd->mdl_lid);
    }

    uint8_t send_data[2] = {0x01, 00};
    // mal_msg_send(prf_get_task_from_id(TASK_ID_MESH), cmd);
    mm_vendorc_transition(cmd->mdl_lid, p_dev_info->app_key_lid, cmd->dst, 
                          MM_MSG_VENDOR_ATTR_ATTR_SET, send_data, sizeof(send_data));
    mal_free(subs_addr);
}

void app_mesh_reg_on_off(void)
{
    MESH_APP_PRINT_INFO("app_mesh_reg_on_off\r\n");

    mm_api_register_client_cmd_t *cmd = MAL_MSG_ALLOC(MESH_MDL_API_CMD, mm_api_register_client_cmd_t);

    cmd->cmd_code = MM_API_REGISTER_CLIENT;
    cmd->cmdl_idx = MM_CMDL_IDX_GENC_ONOFF;

    mal_msg_send(prf_get_task_from_id(TASK_ID_MESH), cmd);
}

static void app_mesh_adv_report_cb(const struct adv_report* p_report)
{
#if 0
    MESH_APP_PRINT_INFO("evt_type = %x  :%s\n", p_report->evt_type, (p_report->evt_type == ADV_CONN_UNDIR) ? \
                        "ADV_CONN_UNDIR" :(p_report->evt_type == ADV_CONN_DIR)? \
                        "ADV_CONN_DIR" : (p_report->evt_type == ADV_DISC_UNDIR)? \
                        "ADV_DISC_UNDIR" : (p_report->evt_type == ADV_NONCONN_UNDIR)? \
                        "ADV_NONCONN_UNDIR": "Unknow");

    MESH_APP_PRINT_INFO("adv_addr = %02x:%02x:%02x:%02x:%02x:%02x\n",
                        p_report->adv_addr.addr[0], p_report->adv_addr.addr[1],
                        p_report->adv_addr.addr[2], p_report->adv_addr.addr[3],
                        p_report->adv_addr.addr[4], p_report->adv_addr.addr[5]);
   // app_relay_user_adv(30, 2,6, p_report->adv_addr.addr);
#endif // #if 0
}

void app_mesh_init(void)
{

    // Reset the environment
    memset(&app_mesh_env, 0, sizeof(struct app_mesh_env_tag));

    mesh_stack_param_init();
}

void app_mesh_add_mesh(void)
{

    MESH_APP_PRINT_INFO("app_mesh_add_mesh profile\r\n");
    // struct oads_db_cfg *db_cfg;
    mesh_cfg_t *db_cfg;
    struct gapm_profile_task_add_cmd *req = KE_MSG_ALLOC_DYN(GAPM_PROFILE_TASK_ADD_CMD,
                                            TASK_GAPM, TASK_APP,
                                            gapm_profile_task_add_cmd, sizeof( mesh_cfg_t));
    // Fill message
    req->operation = GAPM_PROFILE_TASK_ADD;
    req->sec_lvl = 0;//PERM(SVC_AUTH, ENABLE);
    req->prf_task_id = TASK_ID_MESH;
    req->app_task = TASK_APP;
    req->start_hdl = 0; //req->start_hdl = 0; dynamically allocated

    // Set parameters
    db_cfg = ( mesh_cfg_t* ) req->param;

    // Sending of notifications is supported
    db_cfg->prf_cfg.features = M_FEAT_RELAY_NODE_SUP | M_FEAT_PROXY_NODE_SUP | M_FEAT_FRIEND_NODE_SUP | M_FEAT_LOW_POWER_NODE_SUP | M_FEAT_MSG_API_SUP | M_FEAT_PB_GATT_SUP | M_FEAT_DYN_BCN_INTV_SUP;
    db_cfg->prf_cfg.cid = 0x5F0;
#if (BLE_MESH_MDL)
    db_cfg->model_cfg.nb_replay = 2;
#endif // (BLE_MESH_MDL)
    // Send the message
    ke_msg_send(req);
}

void user_models_bind_app_key(uint16_t app_key_id)
{
    m_lid_t app_key_lid;
    uint16_t status;

    status = m_tb_key_app_find(app_key_id, &app_key_lid); // 0 not change

    //MODEL_DBG_PRINTF("user_models_bind_app_key  app_key_lid = 0x%x,status:%x\r\n",app_key_lid,status);

    if (status == MESH_ERR_NO_ERROR)
    {
        for (int i = 2; i < m_tb_mio_get_nb_model(); i++)
        {
            status = m_tb_key_model_bind(app_key_lid, i);
            //MODEL_DBG_PRINTF("m_tb_key_model_bind  m_lid: %d,status:%x\r\n",i,status);
            if (status == MESH_ERR_NO_ERROR)
            {
                m_tb_mio_bind(i);
            }
        }

    }

}

void app_ali_set_unprov_adv_state(uint8_t state)
{
    uint8_t len = sizeof(state);
    //uint32_t ret;
    nvds_put(NVDS_TAG_MESH_PROV_STATE, len, (uint8_t*)&state);

}

uint8_t app_ali_get_unprov_adv_state(void)
{
    uint8_t state = 0;
    uint8_t len = sizeof(state);
    //uint32_t ret;

    nvds_get(NVDS_TAG_MESH_PROV_STATE, &len, (uint8_t*)&state);

    return state;

}

uint16_t user_models_subs_group_addr(m_lid_t m_lid, uint16_t addr)
{
    uint16_t status;

    status =  m_tb_mio_add_subscription(m_lid, addr);

    MESH_MODEL_PRINT_DEBUG("user_ m_tb_mio_add_subscription model_lid:%x, addr:%x,status:%x\r\n", m_lid, addr, status);
    return status;
}

/**
 **************************************************
 *
 * @param[in] m_lid      Index of model local id
 * @param[in] addr       model publish addr (Group addr)
 * @param[in] ttl        msg ttl
 * @param[in] period      period (bit 7-6 step resolution)00:100ms, 01:1s, 10:10s,11: 10min
 *                               (bit 5-0 step number) // total time = step resolution * step number
 * @return An error status
 ***************************************************
 */

uint16_t user_models_publish_set(uint16_t app_key_id, m_lid_t m_lid, uint16_t addr, uint8_t ttl, uint8_t period) //period
{
    uint16_t status;
    m_lid_t app_key_lid;

    status = m_tb_key_app_find(app_key_id, &app_key_lid); // 0 not change
    if (status == MESH_ERR_NO_ERROR)
    {
        status = m_tb_mio_set_publi_param(m_lid, addr, NULL,
                                          app_key_lid, M_TTL_DEFAULT, period,
                                          0,
                                          0);
    }

    return status;
}

m_lid_t g_vdr_mdl_lid;
void app_mesh_add_models_server(void)
{
    MESH_APP_PRINT_INFO("app_mesh_add_mesh_models_server\r\n");

    //mm_vendors_register(0, &g_vdr_mdl_lid);

    m_fnd_scenes_init(light_scene_server_data);

    app_ai_lights_models_init(0);
    MESH_APP_PRINT_INFO("g_vdr_mdl_lid = %x\r\n", g_vdr_mdl_lid);
}

__STATIC void app_unprov_adv_cb_timerout(void *p_env)
{
    MESH_APP_PRINT_INFO("%s end!!!\r\n", __func__);

#if 0
//   m_bcn_stop_tx_unprov_bcn();

    mesh_stack_param_unprov_bcn_intv_ms_set(1000);
    m_stack_param.m_adv_nb_tx = 5;
    m_stack_param.m_adv_interval = 10;
#if (BLE_MESH_GATT_PROV)
    m_prov_bearer_gatt_stop();

    m_prov_bearer_scan_stop();
#endif
    led_deinit();
    rwip_prevent_sleep_clear(BK_MESH_ACTIVE);
#endif


}

void app_unprov_adv_timeout_set(uint32_t timer)
{
    MESH_APP_PRINT_INFO("app_unprov_adv_timeout_set %d\r\n", timer);
    if (timer)
    {
        app_mesh_env.timer_upd.cb = app_unprov_adv_cb_timerout;
        app_mesh_env.timer_upd.period = timer;
        mesh_tb_timer_set(&app_mesh_env.timer_upd, timer);

        rwip_prevent_sleep_set(BK_MESH_ACTIVE);
    }
    else
    {
        mesh_tb_timer_clear(&app_mesh_env.timer_upd);
    }
}

void led_init(void)
{

}

__STATIC void app_model_bind_success_cb(void *p_env)
{
    MESH_APP_PRINT_INFO("%s end!!!\r\n", __func__);
    static uint8_t flag = 0;
    static uint8_t led_cnt = 0;
    if (flag == 0)
    {
        flag = 1;
        led_init();
    }

    if (led_cnt++ < 8)
    {
        if (led_cnt % 2 == 0)
        {
            pwm3.target_value  = 0;
        }
        else
        {
            pwm3.target_value  = 6000;
        }

        //  mesh_tb_timer_set(&app_mesh_env.timer_upd, app_mesh_env.timer_upd.period);
    }
}

__STATIC void app_model_bind_fail_cb(void *p_env)
{
    MESH_APP_PRINT_INFO("%s end!!!\r\n", __func__);
    static uint8_t flag = 0;
    static uint8_t led_cnt = 0;
    if (flag == 0)
    {
        flag = 1;
        led_init();
    }

    if (led_cnt++ < 8)
    {
        if (led_cnt % 2 == 0)
        {
            pwm4.target_value  = 0;
        }
        else
        {
            pwm4.target_value  = 6000;
        }

        mesh_tb_timer_set(&app_mesh_env.timer_upd, app_mesh_env.timer_upd.period);
    }
}

/**
 ****************************************************************************************
 * @brief
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int app_mesh_msg_dflt_handler(ke_msg_id_t const msgid,
                                     void const *param,
                                     ke_task_id_t const dest_id,
                                     ke_task_id_t const src_id)
{
    // Drop the message
    app_models_msg_pro_handler(msgid, param, dest_id, src_id);

    return (KE_MSG_CONSUMED);
}

extern uint8_t light_status;

static int app_mesh_msg_model_app_bind_handler(ke_msg_id_t const msgid,
        struct m_api_model_app_bind_ind const *param,
        ke_task_id_t const dest_id,
        ke_task_id_t const src_id)
{
    // Drop the message
    MESH_APP_PRINT_INFO("%s\r\n", __func__);

    static uint16_t config_model_id[6] = {0x1000, 0x1300, 0x1303, 0x1307, 0xfe00, 0xff00};
    static uint16_t config_num = 0;

    uint16_t status;
    {
        for (int i = 0; i < 4; i++)
        {
            if (param->model_id == config_model_id[i])
            {
                config_num++;
                break;
            }
        }
        MESH_APP_PRINT_INFO("config_num = 0x%x\r\n", config_num);
        if (config_num == 1)
        {
            config_num = 5;
            light_prov_complete();
            m_tb_state_set_relay_state(1, 1);
            app_unprov_adv_timeout_set(0);
        }
    }
    // Bind the application key with the model


    MESH_APP_PRINT_INFO("param->status = 0x%x\r\n", param->status);
    MESH_APP_PRINT_INFO("model_id = 0x%x,config_num = %d\r\n", param->model_id, config_num);

    return (KE_MSG_CONSUMED);
}
extern uint16_t quick_onoff_count;
static int app_mesh_msg_node_reset_handler(ke_msg_id_t const msgid,
        void const *param,
        ke_task_id_t const dest_id,
        ke_task_id_t const src_id)
{
    MESH_APP_PRINT_INFO("app_mesh_msg_node_reset_handler msgid = 0x%x\r\n", msgid);

    m_tb_store_config(0);

    light_unBind_complete();
    quick_onoff_count = 0;
    light_state_nv_store(FLASH_LIGHT_PARAM_TYPE_POWER_ON_COUNT);
    wdt_reset(0x1ff);

    return (KE_MSG_CONSUMED);
}
static int app_mesh_msg_key_ind_handler(ke_msg_id_t const msgid,
                                        struct m_tb_key const *param,
                                        ke_task_id_t const dest_id,
                                        ke_task_id_t const src_id)
{
    MESH_APP_PRINT_INFO("app_mesh_msg_key_ind_handler msgid = 0x%x\r\n", msgid);

    m_tb_key_t *key  = (m_tb_key_t *)param;
    uint8_t r_key[MESH_KEY_LEN];
    mem_rcopy(r_key, key->key, MESH_KEY_LEN);
    switch (key->key_type)
    {
        case M_TB_KEY_DEVICE:
        {
            MESH_APP_PRINT_INFO("DEVICE key: %s\n", mesh_buffer_to_hex(r_key, MESH_KEY_LEN));
        } break;
        case M_TB_KEY_NETWORK:
        {
            MESH_APP_PRINT_INFO("NETWORK key: %s\n", mesh_buffer_to_hex(r_key, MESH_KEY_LEN));
        } break;
        case M_TB_KEY_APPLICATION:
        {
            MESH_APP_PRINT_INFO("APPLICATION key: %s\n", mesh_buffer_to_hex(r_key, MESH_KEY_LEN));

            m_tb_key_app_t *app_key = (m_tb_key_app_t *)param;

            user_models_bind_app_key(app_key->app_key_id);

            user_models_subs_group_addr(g_ln_mdl_lid, 0xc000);

            user_models_subs_group_addr(g_ctl_mdl_lid, 0xc000);

            user_models_subs_group_addr(g_hsl_mdl_lid, 0xc000);

            //user_models_publish_set(app_key->app_key_id, g_vdr_mdl_lid, 0xF000, 5, 0x43); // period 1s * 3 == 3s
        } break;

        default:break;
    }


    return (KE_MSG_CONSUMED);
}

static int app_mesh_api_cmp_handler(ke_msg_id_t const msgid,
                                    struct m_api_cmp_evt const *param,
                                    ke_task_id_t const dest_id,
                                    ke_task_id_t const src_id)
{
    MESH_APP_PRINT_INFO("app_mesh_api_cmp_handler,cmd_code:0x%x,stu:%x\n", param->cmd_code, param->status);
    switch (param->cmd_code)
    {

        case M_API_STORAGE_LOAD://0x50
        {
            if (param->status == MESH_ERR_NO_ERROR)
            {

            }

            app_mesh_enable();

        } break;

        case M_API_ENABLE://0x0
        {
            if ((param->cmd_code == M_API_ENABLE) && (param->status == MESH_ERR_NO_ERROR))
            {
                MESH_APP_PRINT_INFO("prov_state = %s \n", (m_tb_state_get_prov_state() == M_TB_STATE_PROV_STATE_PROV) ? "M_TB_STATE_PROV_STATE_PROV":"M_TB_STATE_PROV_STATE_UNPROV" );
                if (m_tb_state_get_prov_state() == M_TB_STATE_PROV_STATE_PROV)
                {
                    if (!is_onoff_loop_start)
                    {
                        app_mesh_on_onff_loop(0);
                    }
#if (BLE_MESH_GATT_PROXY)
                    m_api_proxy_ctrl(2);
#endif
                }
                else
                {
                    //app_unprov_adv_timeout_set(MESH_UNPROV_ADV_TIME);// 10 minute
                }
            }

        } break;

        case M_API_DISABLE://0x1
        {
            // Check that can store the key info in the nvs flash or not.
            m_tb_store_nvs_after_stop_scan();
            app_mesh_enable();
            MESH_APP_PRINT_INFO("M_API_DISABLE param->status %x\n", param->status);
        }

        default:break;
    }

    return (KE_MSG_CONSUMED);
}
static int app_mesh_model_api_cmp_handler(ke_msg_id_t const msgid,
        struct mm_api_cmp_evt const *param,
        ke_task_id_t const dest_id,
        ke_task_id_t const src_id)
{
    MESH_APP_PRINT_INFO("app_mesh_model_api_cmp_handler,cmd_code:0x%x,stu:%x\n", param->cmd_code, param->status);
    switch (param->cmd_code)
    {
        case MM_API_REGISTER_SERVER://0x0
        {
            if (param->status == MESH_ERR_NO_ERROR)
            {
                MESH_APP_PRINT_INFO("%s, Start register the GENC ONOFF client.\n");
            }

        }
        break;

        case MM_API_SRV_SET://200
        {
            if (param->status == MESH_ERR_NO_ERROR)
            {

            }

        }
        break;

        default:break;
    }

    return (KE_MSG_CONSUMED);
}


static int app_mesh_api_prov_auth_data_req_ind_handler(ke_msg_id_t const msgid,
        struct m_api_prov_auth_data_req_ind const *param,
        ke_task_id_t const dest_id,
        ke_task_id_t const src_id)
{
    MESH_APP_PRINT_INFO("app_mesh_api_prov_auth_data_req_ind_handler\n");


    char *str[4] =    {"M_PROV_AUTH_NO_OOB",
                       /// Static OOB authentication is used
                       "M_PROV_AUTH_STATIC_OOB",
                       /// Output OOB authentication is used
                       "M_PROV_AUTH_OUTPUT_OOB",
                       /// Input OOB authentication is used
                       "M_PROV_AUTH_INPUT_OOB"
                      };

    /// No OOB authentication is used
    MESH_APP_PRINT_INFO("auth_method:%x %s,auth_action:%x,auth_size:%x\n", param->auth_method, str[param->auth_method], param->auth_action, param->auth_size);

    m_api_prov_auth_data_cfm_t *cfm = KE_MSG_ALLOC_DYN(MESH_API_PROV_AUTH_DATA_CFM, prf_get_task_from_id(TASK_ID_MESH), TASK_APP, m_api_prov_auth_data_cfm, 16);

    cfm->accept = 1;
    cfm->auth_size = param->auth_size;



#if MAC78da07bcd71b
// 78da07bcd71b; eddc0a4d10287aa2adce37866ad3f2e5
    cfm->auth_data[0] = 0xe5; cfm->auth_data[1] = 0xf2; cfm->auth_data[2] = 0xd3;
    cfm->auth_data[3] = 0x6a; cfm->auth_data[4] = 0x86; cfm->auth_data[5] = 0x37;
    cfm->auth_data[6] = 0xce; cfm->auth_data[7] = 0xad; cfm->auth_data[8] = 0xa2;
    cfm->auth_data[9] = 0x7a; cfm->auth_data[10] = 0x28; cfm->auth_data[11] = 0x10;
    cfm->auth_data[12] = 0x4d; cfm->auth_data[13] = 0x0a; cfm->auth_data[14] = 0xdc;
    cfm->auth_data[15] = 0xed;
#elif MAC78da07bcd71c
//78da07bcd71c;  a8a69e256c9b6898c267ef12a733673d
    cfm->auth_data[0] = 0x3d; cfm->auth_data[1] = 0x67; cfm->auth_data[2] = 0x33;
    cfm->auth_data[3] = 0xa7; cfm->auth_data[4] = 0x12; cfm->auth_data[5] = 0xef;
    cfm->auth_data[6] = 0x67; cfm->auth_data[7] = 0xc2; cfm->auth_data[8] = 0x98;
    cfm->auth_data[9] = 0x68; cfm->auth_data[10] = 0x9b; cfm->auth_data[11] = 0x6c;
    cfm->auth_data[12] = 0x25; cfm->auth_data[13] = 0x9e; cfm->auth_data[14] = 0xa6;
    cfm->auth_data[15] = 0xa8;

#elif MAC78da07bcd71d
    //78da07bcd71d;  c52ab4202a675f6fdd6f44c3ad942fa6
    cfm->auth_data[0] = 0xa6; cfm->auth_data[1] = 0x2f; cfm->auth_data[2] = 0x94;
    cfm->auth_data[3] = 0xad; cfm->auth_data[4] = 0xc3; cfm->auth_data[5] = 0x44;
    cfm->auth_data[6] = 0x6f; cfm->auth_data[7] = 0xdd; cfm->auth_data[8] = 0x6f;
    cfm->auth_data[9] = 0x5f; cfm->auth_data[10] = 0x67; cfm->auth_data[11] = 0x2a;
    cfm->auth_data[12] = 0x20; cfm->auth_data[13] = 0xb4; cfm->auth_data[14] = 0x2a;
    cfm->auth_data[15] = 0xc5;

#elif MAC38d2ca08ca20
    //75 d7 82 e3 b5 a7 0f 4a b7 e6 cc 60 7c 56 91 7a
    cfm->auth_data[0] = 0x75; cfm->auth_data[1] = 0xd7; cfm->auth_data[2] = 0x82;
    cfm->auth_data[3] = 0xe3; cfm->auth_data[4] = 0xb5; cfm->auth_data[5] = 0xa7;
    cfm->auth_data[6] = 0x0f; cfm->auth_data[7] = 0x4a; cfm->auth_data[8] = 0xb7;
    cfm->auth_data[9] = 0xe6; cfm->auth_data[10] = 0xcc; cfm->auth_data[11] = 0x60;
    cfm->auth_data[12] = 0x7c; cfm->auth_data[13] = 0x56; cfm->auth_data[14] = 0x91;
    cfm->auth_data[15] = 0x7a;

#elif MACf8a76324a49f
    //e2 70 c0 2a 2b 4e 5a c1 1e d0 09 1e a4 2f 6d 4e
    cfm->auth_data[0] = 0x4e; cfm->auth_data[1] = 0x6d; cfm->auth_data[2] = 0x2f;
    cfm->auth_data[3] = 0xa4; cfm->auth_data[4] = 0x1e; cfm->auth_data[5] = 0x09;
    cfm->auth_data[6] = 0xd0; cfm->auth_data[7] = 0x1e; cfm->auth_data[8] = 0xc1;
    cfm->auth_data[9] = 0x5a; cfm->auth_data[10] = 0x4e; cfm->auth_data[11] = 0x2b;
    cfm->auth_data[12] = 0x2a; cfm->auth_data[13] = 0xc0; cfm->auth_data[14] = 0x70;
    cfm->auth_data[15] = 0xe2;
#else
    // 78da07bcd71b; eddc0a4d10287aa2adce37866ad3f2e5
    cfm->auth_data[0] = 0xe5; cfm->auth_data[1] = 0xf2; cfm->auth_data[2] = 0xd3;
    cfm->auth_data[3] = 0x6a; cfm->auth_data[4] = 0x86; cfm->auth_data[5] = 0x37;
    cfm->auth_data[6] = 0xce; cfm->auth_data[7] = 0xad; cfm->auth_data[8] = 0xa2;
    cfm->auth_data[9] = 0x7a; cfm->auth_data[10] = 0x28; cfm->auth_data[11] = 0x10;
    cfm->auth_data[12] = 0x4d; cfm->auth_data[13] = 0x0a; cfm->auth_data[14] = 0xdc;
    cfm->auth_data[15] = 0xed;
#endif // 

#if TEST_MESH_OTA

    uint8_t auth_value[16] =
    {
        0x08, 0x07, 0x06, 0x05, 0x04, 0x03, 0x02, 0x01,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0a
    };
    memcpy(cfm->auth_data, auth_value, 16);
#endif // 


    //  00 00 00 00 00 00 00 00 01 02 03 04 05 06 07 08
#if ALI_MESH
    uint8_t ali_auth_value[16];
    if (user_data_contains_ali_data())
    {
        if (user_data_read_ali_secret_key(ali_auth_value))
        {
            memcpy(cfm->auth_data, ali_auth_value, 16);
        }

    }
#endif //          

    for (int i =0 ; i < 16; i++)
    {
        MESH_APP_PRINT_INFO("cfm->auth_data[%d] = 0x%02x\r\n", i, cfm->auth_data[i]);
    }


    ke_msg_send(cfm);

    return (KE_MSG_CONSUMED);
}

static int app_mesh_api_prov_attention_update_ind_handler(ke_msg_id_t const msgid,
        struct m_api_attention_update_ind const *param,
        ke_task_id_t const dest_id,
        ke_task_id_t const src_id)
{
    MESH_APP_PRINT_INFO("%s\n", __func__);

    MESH_APP_PRINT_INFO("param->attention_state :%d\n", param->attention_state);

    return (KE_MSG_CONSUMED);
}

static int app_mesh_api_prov_state_ind_handler(ke_msg_id_t const msgid,
        struct m_api_prov_state_ind const *param,
        ke_task_id_t const dest_id,
        ke_task_id_t const src_id)
{
    MESH_APP_PRINT_INFO("%s\n", __func__);

    char *str[3] = {"M_PROV_STARTED", "M_PROV_SUCCEED", "M_PROV_FAILED"};

    MESH_APP_PRINT_INFO("state :%s,status:%x\n", str[param->state], param->status);


    if (param->state == M_PROV_STARTED)
    {
        light_prov_start();
    }
    else if (param->state == M_PROV_FAILED)
    {
        for (int i = 0; i < UN_PROV_DEV_MAX_NUM; i++)
        {
            app_mesh_unprov_dev[i].fActive = 0;
            memset(app_mesh_unprov_dev[i].dev_uuid, 0, MESH_DEV_UUID_LEN);
            app_mesh_unprov_dev[i].oob_info = 0;
            app_mesh_unprov_dev[i].uri_hash = 0;
        }

    }
    else if (param->state == M_PROV_SUCCEED)
    {
        // m_tb_store_config(20);
        MESH_APP_PRINT_INFO("+++++++ M_PROV_SUCCEED +++++++ \n");
    }

    return (KE_MSG_CONSUMED);
}

static char temp_uuid[7] = {0xa9, 0x01, 0x71, 0x33, 0x02, 0x00, 0x00};

#include "m_prov.h"
static int app_mesh_unprov_beacon_handler(ke_msg_id_t const msgid,
        void const *param,
        ke_task_id_t const dest_id,
        ke_task_id_t const src_id)
{
    int i = 0;
    m_api_unprov_beacon_ind_t *p_ind = (m_api_unprov_beacon_ind_t *)param;

    if (!m_prov_get_start_state())
    {
        MESH_APP_PRINT_ERR("m_prov_get_start_state false.");
        // Check the provisioner in the stack layer ready to start or not.
        return KE_MSG_CONSUMED;
    }

    for (i = 0; i < UN_PROV_DEV_MAX_NUM; i++)
    {
        // MESH_APP_PRINT_ERR("************ Received the unprov beacon, start provisioning. fActive = %d **********************\n", app_mesh_unprov_dev[i].fActive);
        if (app_mesh_unprov_dev[i].fActive == 1)
        {
            if (0 == memcmp(app_mesh_unprov_dev[i].dev_uuid, p_ind->dev_uuid, MESH_DEV_UUID_LEN))
            {
                return KE_MSG_CONSUMED;
            }
        }
        if ((app_mesh_unprov_dev[i].fActive == 0) &&
                (memcmp(temp_uuid, p_ind->dev_uuid, sizeof(temp_uuid)) == 0))
        {
            app_mesh_unprov_dev[i].fActive = 1;
            memcpy(app_mesh_unprov_dev[i].dev_uuid, p_ind->dev_uuid, MESH_DEV_UUID_LEN);
            app_mesh_unprov_dev[i].oob_info = p_ind->oob_info;
            app_mesh_unprov_dev[i].uri_hash = p_ind->uri_hash;
            MESH_APP_PRINT_INFO("==Dev UUID==\n");
            MESH_APP_PRINT_INFO("%s\n", mesh_buffer_to_hex(p_ind->dev_uuid, MESH_DEV_UUID_LEN));
            MESH_APP_PRINT_INFO("OOB_INFO=%d\n", p_ind->oob_info);
            break;
        }
    }
 
    if (i == UN_PROV_DEV_MAX_NUM)
    {
        //MESH_APP_PRINT_INFO("app_mesh_unprov_dev is full\n");
        return (KE_MSG_CONSUMED);
    }

    if (memcmp(temp_uuid, p_ind->dev_uuid, sizeof(temp_uuid)) != 0)
    {
        app_mesh_unprov_dev[i].fActive = 0;
        return KE_MSG_CONSUMED;
    }

    // Should add the network key and App key before provisioning
    uint16_t net_key_id = 0x07;
    uint16_t app_key_id = 0;
    uint8_t net_key[MESH_KEY_LEN] = {0};
    uint8_t app_key[MESH_KEY_LEN] = {0};

    for (int i = 0; i < MESH_KEY_LEN; i++)
    {
        net_key[i] = (i+0x0A)*i - i^3 + 10;
    }

    for (int i = 0; i < MESH_KEY_LEN; i++)
    {
        app_key[i] = (i+0x0B)*i - i^3 + 17;
    }

    if (m_tb_state_get_prov_state() == M_TB_STATE_PROV_STATE_PROV)
    {
        // Stop Update the data to nvds when doing provisioning.
        m_tb_store_config(0);
    }

    m_api_add_network_key(net_key_id, &net_key[0]);
    m_api_add_app_key(app_key_id, &app_key[0], net_key_id);
    // Start provision with the remote device.
    app_mesh_add_new_dev();
    return (KE_MSG_CONSUMED);
}

static int app_mesh_prov_capability_handler(ke_msg_id_t const msgid,
        void const *param,
        ke_task_id_t const dest_id,
        ke_task_id_t const src_id)
{
    int i;
    m_api_prov_capabilities_ind_t *p_ind = (m_api_prov_capabilities_ind_t *)param;

    if (p_ind)
    {
        MESH_APP_PRINT_INFO("%s, algorithms %d, pub_key_type %d, static_oob_type %d, out_oob_size %d, out_oob_action %d, in_oob_size %d, in_oob_action %d\n",
                            __func__, p_ind->algorithms, p_ind->pub_key_type, p_ind->static_oob_type,
                            p_ind->out_oob_size, p_ind->out_oob_action, p_ind->in_oob_size, p_ind->in_oob_action);
    }
    // TODO: The APP should input the capablility data and
    // TODO: compare the provisioner OOB action and the device OOB action then select which OOB method should be used
    //simplified app, prov capability data is not used
    app_mesh_start_dev_prov();
    return (KE_MSG_CONSUMED);
}

mesh_tb_timer_t binding_timer;
m_lid_t dev_lid_binding;

mesh_tb_timer_t binding_timer_reset;
mesh_tb_timer_t reprov_timer;

void binding_timer_cb(void *timer)
{
    MESH_APP_PRINT_INFO("binding_timer_cb\n");

    app_mesh_dev_compo_data_get(dev_lid_binding);
}

void reprov_timer_cb(void *timer)
{
    MESH_APP_PRINT_INFO("reprov_timer_cb\n");
    for (int i = 0; i < UN_PROV_DEV_MAX_NUM; i++)
    {
        app_mesh_unprov_dev[i].fActive = 0;
        memset(app_mesh_unprov_dev[i].dev_uuid, 0, MESH_DEV_UUID_LEN);
        app_mesh_unprov_dev[i].oob_info = 0;
        app_mesh_unprov_dev[i].uri_hash = 0;
    }
}

static int app_mesh_dev_added_handler(ke_msg_id_t const msgid,
                                      void const *param,
                                      ke_task_id_t const dest_id,
                                      ke_task_id_t const src_id)
{
    MESH_APP_PRINT_INFO("app_mesh_dev_added_handler\n");

    binding_timer.cb = binding_timer_cb;
    binding_timer.period = 500;
    mesh_tb_timer_set(&binding_timer, binding_timer.period);

    dev_lid_binding = *(uint8_t *)param;

    return (KE_MSG_CONSUMED);
}

static int app_mesh_dev_compo_data_sts_handler(ke_msg_id_t const msgid,
        void const *param,
        ke_task_id_t const dest_id,
        ke_task_id_t const src_id)
{
    MESH_APP_PRINT_INFO("app_mesh_dev_compo_data_sts_handler\n");

    m_api_compo_data_status_ind_t *p_ind = (m_api_compo_data_status_ind_t *)param;

    app_mesh_dev_add_app_key(p_ind->dev_lid);

    return (KE_MSG_CONSUMED);
}

static int app_mesh_app_key_sts_handler(ke_msg_id_t const msgid,
                                        void const *param,
                                        ke_task_id_t const dest_id,
                                        ke_task_id_t const src_id)
{
    m_api_app_key_status_ind_t *p_ind = (m_api_app_key_status_ind_t *)param;

    MESH_APP_PRINT_INFO("app_mesh_app_key_sts_handler,nwkID=%x,appID=%x\n", p_ind->nwk_key_id, p_ind->app_key_id);

    if (p_ind->status == MESH_ERR_NO_ERROR)
    {
        app_mesh_model_app_bind(p_ind->dev_lid);
    }

    return (KE_MSG_CONSUMED);
}

mesh_tb_timer_t onoff_loop_timer;

void on_onff_loop_timer_cb(void *timer)
{
    MESH_APP_PRINT_INFO("on_onff_loop_timer_cb\n");
    static uint8_t onoff = 1;

    app_mesh_set_on_off(dev_lid_binding, onoff);

    onoff = (~onoff)&0x01;

    binding_timer.period = 5000;
    mesh_tb_timer_set(&binding_timer, binding_timer.period);
}

static void app_mesh_on_onff_loop(m_lid_t dev_lid)
{
    MESH_APP_PRINT_INFO("app_mesh_on_onff_loop\n");
    is_onoff_loop_start = true;
    dev_lid_binding = dev_lid;
    binding_timer.cb = on_onff_loop_timer_cb;
    binding_timer.period = 5000;
    mesh_tb_timer_set(&binding_timer, binding_timer.period);
}
 
static int app_mesh_model_app_sts_handler(ke_msg_id_t const msgid,
        void const *param,
        ke_task_id_t const dest_id,
        ke_task_id_t const src_id)
{
    m_api_model_app_status_ind_t *p_ind = (m_api_model_app_status_ind_t *)param;

    MESH_APP_PRINT_INFO("app_mesh_model_app_sts_handler, sts = 0x%x, address= 0x%x, key= 0x%x, model = 0x%x, status %d\r\n",
                        p_ind->status, p_ind->address, p_ind->app_key_id, p_ind->model_id, p_ind->status);

    if (p_ind && (p_ind->status == MESH_ERR_NO_ERROR))
    {
        app_mesh_model_subs_add(p_ind->dev_lid, false, app_subs_addr);
    }

    return (KE_MSG_CONSUMED);
}

static int app_mesh_model_subs_sts_handler(ke_msg_id_t const msgid,
                                                             void const *param,
                                                             ke_task_id_t const dest_id,
                                                             ke_task_id_t const src_id)
{
    m_api_model_subs_status_ind_t *p_ind = (m_api_model_subs_status_ind_t *)param;
    MESH_APP_PRINT_INFO("%s, dev_lid %d, status %d, elem_addr 0x%x, subs_addr 0x%x, model_id 0x%x\n",
                        __func__, p_ind->dev_lid, p_ind->status, p_ind->elem_addr,
                        p_ind->subs_addr, p_ind->model_id);

    reprov_timer.cb = reprov_timer_cb;
    reprov_timer.period = 1000;
    mesh_tb_timer_set(&reprov_timer, reprov_timer.period);
    m_tb_store_config(10);
    if (p_ind && (p_ind->status == MESH_ERR_NO_ERROR))
    {
        app_mesh_on_onff_loop(p_ind->dev_lid);
    }

    return (KE_MSG_CONSUMED);
}

/// Default State handlers definition
const struct ke_msg_handler app_mesh_msg_handler_list[] =
{
    // Note: first message is latest message checked by kernel so default is put on top.
    {KE_MSG_DEFAULT_HANDLER,                    (ke_msg_func_t)app_mesh_msg_dflt_handler},

    {MESH_API_MODEL_APP_BIND_IND,               (ke_msg_func_t)app_mesh_msg_model_app_bind_handler},

    {MESH_API_NODE_RESET_IND,                   (ke_msg_func_t)app_mesh_msg_node_reset_handler},

    {MESH_API_KEY_IND,                          (ke_msg_func_t)app_mesh_msg_key_ind_handler},

    {MESH_API_CMP_EVT,                          (ke_msg_func_t)app_mesh_api_cmp_handler},

    {MESH_MDL_API_CMP_EVT,                      (ke_msg_func_t)app_mesh_model_api_cmp_handler},

    {MESH_API_PROV_AUTH_DATA_REQ_IND,           (ke_msg_func_t)app_mesh_api_prov_auth_data_req_ind_handler},

    {MESH_API_ATTENTION_UPDATE_IND,            (ke_msg_func_t)app_mesh_api_prov_attention_update_ind_handler},

    {MESH_API_PROV_STATE_IND,                   (ke_msg_func_t)app_mesh_api_prov_state_ind_handler},

    {MESH_API_PROV_UNPROV_BCN_IND,          (ke_msg_func_t)app_mesh_unprov_beacon_handler},

    {MESH_API_PROV_CAPAVILITY_IND,            (ke_msg_func_t)app_mesh_prov_capability_handler},

    {MESH_API_DEV_ADDED_IND,                     (ke_msg_func_t)app_mesh_dev_added_handler},

    {MESH_API_DEV_COMPO_DATA_STS_IND,     (ke_msg_func_t)app_mesh_dev_compo_data_sts_handler},

    {MESH_API_APP_KEY_STS_IND,                  (ke_msg_func_t)app_mesh_app_key_sts_handler},

    {MESH_API_MODEL_APP_STS_IND,              (ke_msg_func_t)app_mesh_model_app_sts_handler},
    {MESH_API_MODEL_SUBS_STS_IND,             (ke_msg_func_t)app_mesh_model_subs_sts_handler},
};

const struct ke_state_handler app_mesh_table_handler =
{&app_mesh_msg_handler_list[0], (sizeof(app_mesh_msg_handler_list)/sizeof(struct ke_msg_handler))};

