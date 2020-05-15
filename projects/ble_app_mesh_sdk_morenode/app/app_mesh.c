/**
 ****************************************************************************************
 *
 * @file app_mesh.c
 *
 * @brief mesh Application Module entry point
 *
 * @auth  gang.cheng
 *
 * @date  2018.07.09
 *
 * Copyright (C) Beken 2009-2016
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
#include "mesh_general_api.h"
#include "lld_adv_test.h"
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
#include "app_light_server.h"
#include "mm_vendors_morenode.h"
#include "m_fnd_int.h"
#include "mesh_param_int.h"
#include "m_fnd_Fw_Update.h"
#include "mal_int.h"
#include "gpio.h"
#include "user_config.h"
#include "mesh_api_msg.h"

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

static void app_mesh_adv_report_cb(const struct adv_report* p_report);

/*
 * GLOBAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */

void app_mesh_init(void)
{
    // Reset the environment
    memset(&app_mesh_env, 0, sizeof(struct app_mesh_env_tag));

    mesh_stack_param_init();
  
    mal_adv_report_register(app_mesh_adv_report_cb);
#if (MESH_MEM_TB_BUF_DBG)
    mesh_mem_dbg_init();
#endif /* MESH_MEM_TB_BUF_DBG */
}

void app_mesh_add_mesh(void)
{

    MESH_APP_PRINT_INFO("app_mesh_add_mesh profile\n");
    mesh_cfg_t *db_cfg;
    struct gapm_profile_task_add_cmd *req = KE_MSG_ALLOC_DYN(GAPM_PROFILE_TASK_ADD_CMD,
                                            TASK_GAPM, TASK_APP,
                                            gapm_profile_task_add_cmd, sizeof(mesh_cfg_t));
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
    db_cfg->model_cfg.nb_replay = 5;
#endif // (BLE_MESH_MDL)
    // Send the message
    ke_msg_send(req);
}


static void app_mesh_adv_report_cb(const struct adv_report* p_report)
{
    //MESH_APP_PRINT_INFO("evt_type = %x  :%s\r\n", p_report->evt_type, (p_report->evt_type == ADV_CONN_UNDIR) ? "ADV_CONN_UNDIR" :(p_report->evt_type == ADV_CONN_DIR)? "ADV_CONN_DIR" : (p_report->evt_type == ADV_DISC_UNDIR)? "ADV_DISC_UNDIR" : (p_report->evt_type == ADV_NONCONN_UNDIR)? "ADV_NONCONN_UNDIR": "Unknow");
    //MESH_APP_PRINT_INFO("adv_addr = %02x:%02x:%02x:%02x:%02x:%02x\r\n", p_report->adv_addr.addr[0], p_report->adv_addr.addr[1], p_report->adv_addr.addr[2], p_report->adv_addr.addr[3], p_report->adv_addr.addr[4], p_report->adv_addr.addr[5]);
}

void user_models_bind_app_key(uint16_t app_key_id)
{
    m_lid_t app_key_lid;
    uint16_t status;

    status = m_tb_key_app_find(app_key_id, &app_key_lid); // 0 not change

    MESH_APP_PRINT_INFO("user_models_bind_app_key  app_key_lid = 0x%x,status:%x\n", app_key_lid, status);

    if (status == MESH_ERR_NO_ERROR)
    {
        for (int i = 0; i < m_tb_mio_get_nb_model(); i++)
        {
            status = m_tb_key_model_bind(app_key_lid, i);
            MESH_APP_PRINT_INFO("m_tb_key_model_bind  m_lid= 0x%x,status:%x\n", i, status);
            if (status == MESH_ERR_NO_ERROR)
            {
                m_tb_mio_bind(i);
            }
        }
    }

}

uint16_t user_models_subs_group_addr(m_lid_t m_lid, uint16_t addr)
{
    uint16_t status;

    status =  m_tb_mio_add_subscription(m_lid, addr);

    MESH_APP_PRINT_INFO("user_ m_tb_mio_add_subscription model_lid:%x, addr:%x,status:%x\n", m_lid, addr, status);
    return status;
}

uint16_t user_models_publish_set(uint16_t app_key_id, m_lid_t m_lid, uint16_t addr,uint8_t period)
{
    uint16_t status;
    m_lid_t app_key_lid;
    status = m_tb_key_app_find(app_key_id, &app_key_lid); // 0 not change
    status = m_tb_mio_set_publi_param(m_lid, addr, NULL,
                                      app_key_lid, M_TTL_DEFAULT, 
					period, ///change 191231
                                     // 10, ///1s???  ///add 191230
                                      ///54,  ///54 -> 5s
                                      0,
                                      0);

    MESH_APP_PRINT_DEBUG("user_ m_tb_mio_set_publi_param model_lid:%x, addr:%x,status:%x\n", m_lid, addr, status);
    return status;
}

uint16_t user_models_publish_set_keyid(uint16_t app_keyid,m_lid_t m_lid, uint16_t addr,uint8_t period)
{
    uint16_t status;
    m_lid_t app_key_lid;
    status = m_tb_key_app_find(app_keyid, &app_key_lid); // 0 not change
    status = m_tb_mio_set_publi_param(m_lid, addr, NULL,
                                      app_key_lid, M_TTL_DEFAULT, 
					period, ///change 191231
                                     // 10, ///1s???  ///add 191230
                                      ///54,  ///54 -> 5s
                                      0,
                                      0);

    MESH_APP_PRINT_DEBUG("user_ m_tb_mio_set_publi_param model_lid:%x, addr:%x,status:%x\n", m_lid, addr, status);
    return status;
}

uint16_t user_models_publish_set_vendor(m_lid_t m_lid, uint16_t addr)
{
    uint16_t status;
    m_lid_t app_key_lid;
    status = m_tb_key_app_find(0, &app_key_lid); // 0 not change
    status = m_tb_mio_set_publi_param(m_lid, addr, NULL,
                                      app_key_lid, M_TTL_DEFAULT, 
					  5, ///change 191231
                                     // 10, ///1s???  ///add 191230
                                      ///54,  ///54 -> 5s
                                      0,
                                      0);
    MESH_APP_PRINT_DEBUG("user_ m_tb_mio_set_publi_param model_lid:%x, addr:%x,status:%x\n", m_lid, addr, status);
    return status;
}

uint16_t user_models_publish_clear(uint16_t app_keyid,m_lid_t m_lid, uint16_t addr)
{
    uint16_t status;
    m_lid_t app_key_lid;
    status = m_tb_key_app_find(app_keyid, &app_key_lid); // 0 not change
    status = m_tb_mio_set_publi_param(m_lid, addr, NULL,
                                      app_key_lid, M_TTL_DEFAULT, 
                                      0, ///1s???  ///add 191230
                                      ///54,  ///54 -> 5s
                                      0,
                                      0);

    MESH_APP_PRINT_DEBUG("user_ m_tb_mio_set_publi_param model_lid:%x, addr:%x,status:%x\n", m_lid, addr, status);
    return status;
}




void app_mesh_add_models_server(void)
{
    MESH_APP_PRINT_INFO("app_mesh_add_mesh_models_server\n");
    app_lights_models_init(0);

}

extern void m_link_open_ack_dis(void);
static void app_get_prov_param(m_api_prov_param_cfm_t* cfm, uint8_t adv_type);
__STATIC void app_unprov_adv_cb_timerout(void *p_env)
{
    MESH_APP_PRINT_INFO("%s end!!!\n", __func__);

    m_bcn_stop_tx_unprov_bcn();
#if (BLE_MESH_GATT_PROV)
    m_prov_bearer_gatt_stop();

    m_prov_bearer_scan_stop();
#endif
    led_deinit();
    rwip_prevent_sleep_clear(BK_MESH_ACTIVE);

#if (UNPROV_TIMEOUT_ADV)
    rwip_prevent_sleep_set(BK_MESH_ACTIVE);
    m_api_prov_param_cfm_t *cfm = KE_MSG_ALLOC(MESH_API_PROV_PARAM_CFM, prf_get_task_from_id(TASK_ID_MESH), TASK_APP, m_api_prov_param_cfm);
    app_get_prov_param(cfm, 1);
    ke_msg_send(cfm);
    m_link_open_ack_dis();
    m_stack_param.m_adv_interval = 64;
    m_stack_param.m_bcn_default_unprov_bcn_intv_ms = 60000;
#endif
}

void app_unprov_adv_timeout_set(uint32_t timer)
{
    MESH_APP_PRINT_INFO("app_unprov_adv_timeout_set %d\n", timer);
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

extern uint16_t gdst;

///VENDOR_ONEGROUP_GET_ATTRTYPE
mesh_tb_timer_t onegroup_get_timer;
void app_onegroup_get(void *timer)
{
	MESH_APP_PRINT_DEBUG("%s\n",__func__);
	mm_vendor_attr_indication_publish(VENDOR_ONEGROUP_GET_ATTRTYPE,5,NULL,gdst); 
	onegroup_get_timer.period = 500;
	mesh_tb_timer_set(&onegroup_get_timer, onegroup_get_timer.period);
}

void app_onegroup_get_timer_set(uint32_t timer)
{
	if(timer)
	{
		onegroup_get_timer.cb = app_onegroup_get;
		onegroup_get_timer.period = timer;
		mesh_tb_timer_set(&onegroup_get_timer, timer);
	}
	else
	{
		mesh_tb_timer_clear(&onegroup_get_timer);
	}
}

///ONOFF_ONEGROUP_GET_ATTRTYPE
mesh_tb_timer_t ONOFF_onegroup_get_timer;
void app_ONOFF_onegroup_get(void *timer)
{
	MESH_APP_PRINT_DEBUG("%s\n",__func__);
	mm_vendor_attr_indication_publish(ONOFF_ONEGROUP_GET_ATTRTYPE,5,NULL,gdst); 
	ONOFF_onegroup_get_timer.period = 500;
	mesh_tb_timer_set(&ONOFF_onegroup_get_timer, ONOFF_onegroup_get_timer.period);
}

void app_ONOFF_onegroup_get_timer_set(uint32_t timer)
{
	if(timer)
	{
		ONOFF_onegroup_get_timer.cb = app_ONOFF_onegroup_get ;//app_onegroup_get;
		ONOFF_onegroup_get_timer.period = timer;
		mesh_tb_timer_set(&ONOFF_onegroup_get_timer, timer);
	}
	else
	{
		mesh_tb_timer_clear(&ONOFF_onegroup_get_timer);
	}
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
    }
}

__STATIC void app_model_bind_fail_cb(void *p_env)
{
    MESH_APP_PRINT_INFO("%s end!!!\n", __func__);
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

static int app_mesh_msg_model_app_bind_handler(ke_msg_id_t const msgid,
        struct m_api_model_app_bind_ind const *param,
        ke_task_id_t const dest_id,
        ke_task_id_t const src_id)
{
    // Drop the message
    MESH_APP_PRINT_INFO("%s\n", __func__);

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
        MESH_APP_PRINT_INFO("config_num = 0x%x\n", config_num);
        if (config_num == 1)
        {
            config_num = 5;

        }

    }

    MESH_APP_PRINT_INFO("param->status = 0x%x\n", param->status);
    MESH_APP_PRINT_INFO("model_id = 0x%x,config_num = %d\n", param->model_id, config_num);

    return (KE_MSG_CONSUMED);
}

extern uint16_t quick_onoff_count;
static int app_mesh_msg_node_reset_handler(ke_msg_id_t const msgid,
        void const *param,
        ke_task_id_t const dest_id,
        ke_task_id_t const src_id)
{
    MESH_APP_PRINT_INFO("app_mesh_msg_node_reset_handler msgid = 0x%x\r\n", msgid);
    MESH_APP_PRINT_INFO("device prov state = %d\n", m_tb_state_get_prov_state());

    m_tb_store_config(0);
    light_unBind_complete();
    quick_onoff_count = 0;
    light_state_nv_store(FLASH_LIGHT_PARAM_TYPE_POWER_ON_COUNT);
    wdt_reset(0x1ff);

    return (KE_MSG_CONSUMED);
}

extern uint16_t send_nb;
extern uint16_t g_app_key_id;
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
            MESH_APP_PRINT_INFO("******************DEVICE key************************\r\n\r\n");
        } break;
        case M_TB_KEY_NETWORK:
        {
            MESH_APP_PRINT_INFO("******************NETWORK key************************\r\n\r\n");

        } break;
        case M_TB_KEY_APPLICATION:
        {
            MESH_APP_PRINT_INFO("******************APPLICATION key********************\r\n\r\n");

            m_tb_key_app_t *app_key = (m_tb_key_app_t *)param;

			///add 0312      
			g_app_key_id = app_key->app_key_id;
			#if UART_CMD_PROV_EN
			
			user_models_bind_app_key(app_key->app_key_id);    
			#endif
			  
			//user_models_bind_app_key(app_key_id);        
			#if 0//UART_CMD_PROV_EN
				#if 0        
				MESH_APP_PRINT_INFO("****0312 0x03 vendor publish app_key->app_key_id = %d",app_key->app_key_id);        
				send_nb = 100;        
				
				//m_tb_mio_set_prim_addr(0x03);        
				//user_models_publish_set(g_vdr_lid, 0xc000,05);          
				//user_models_subs_group_addr(g_vdr_lid, 0xc00f);       
				
				user_models_publish_set_keyid(app_key->app_key_id,g_vdr_lid, 0xc000,05);    ///0313
				#else       
				MESH_APP_PRINT_INFO("****0312  0x05 vendor sub");        
				m_tb_mio_set_prim_addr(0x05);        
				//user_models_publish_set(g_vdr_lid, 0xc00f,05);                    
				user_models_subs_group_addr(g_vdr_lid, 0xc000);       
				#endif
			#endif

        } break;

        default:
            break;
    }

    MESH_APP_PRINT_INFO("%s \n", mesh_buffer_to_hex(r_key, MESH_KEY_LEN));

    return (KE_MSG_CONSUMED);
}

extern uint16_t send_nb;
extern uint16_t app_key_id ;
extern uint8_t nvds_ctlset_value;
extern uint8_t nvds_ctlset_value_len;
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
            app_mesh_enable();
        } break;

        case M_API_ENABLE://0x0
        {
            if ((param->cmd_code == M_API_ENABLE) && (param->status == MESH_ERR_NO_ERROR))
            {
                MESH_APP_PRINT_INFO("prov_state = %s\n", (m_tb_state_get_prov_state() == M_TB_STATE_PROV_STATE_PROV) ? "M_TB_STATE_PROV_STATE_PROV":"M_TB_STATE_PROV_STATE_UNPROV" );
                MESH_APP_PRINT_INFO("****uaddr = 0x%x\n",m_tb_mio_get_prim_addr());
		if(!nvds_get(NVDS_TAG_MESH_CTLSET_VALUE,&nvds_ctlset_value_len,&nvds_ctlset_value))  ///add 0305
		{
			MESH_APP_PRINT_INFO("****get nvds CTLSET value = 0x%x, = %d,*10 = %d\n",nvds_ctlset_value,nvds_ctlset_value,nvds_ctlset_value*10);
		}
		else
		{
			MESH_APP_PRINT_INFO("****get nvds CTLSET value Error\n");
		}
		if (m_tb_state_get_prov_state() == M_TB_STATE_PROV_STATE_PROV)
                {

			{

		   MESH_APP_PRINT_INFO("******************************************************************************* \r\n");
			MESH_APP_PRINT_INFO("******************************************************************************* \r\n");
			MESH_APP_PRINT_INFO("***************  PROV   OK    ************************************************* \r\n");

			MESH_APP_PRINT_INFO("*************** uaddr    is 0x%4x ******************************************** \r\n",m_tb_mio_get_prim_addr());
			//MESH_APP_PRINT_INFO("*************** gdst     is 0x%4x ******************************************** \r\n",gdst);
			//MESH_APP_PRINT_INFO("*************** sub_dst  is 0x%4x ******************************************** \r\n",sub_dst);
			MESH_APP_PRINT_INFO("*************** ctlparam is 0x%4x ******************************************** \r\n",nvds_ctlset_value);
		    MESH_APP_PRINT_INFO("******************************************************************************* \r\n");
			MESH_APP_PRINT_INFO("******************************************************************************* \r\n");
			
		
  		  }

				
#if (!TEST_MESH_OTA)
#if (BLE_MESH_GATT_PROXY)
                    //m_tb_state_set_gatt_proxy_state(M_CONF_GATT_PROXY_STATE_ENABLED);
                    m_api_proxy_ctrl(2);
#endif /* !TEST_MESH_OTA */
#endif /* BLE_MESH_GATT_PROXY */
                }
                else
                {
#if (UART_CMD_PROV_EN)
                    app_test_add_key();
			//m_tb_mio_set_prim_addr(0x03);        
		    
#else
                    app_unprov_adv_timeout_set(MESH_UNPROV_ADV_TIME);// 10 minute
#endif /* UART_CMD_PROV_EN */

                }
            }

        } break;

        case M_API_DISABLE://0x1
        {
            MESH_APP_PRINT_INFO("M_API_DISABLE param->status %x\n", param->status);
            m_tb_store_nvs_after_stop_scan();
            m_api_enable();
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
                MESH_APP_PRINT_INFO("model register success\n");
            }

        } break;

        case MM_API_SRV_SET://200
        {

        } break;

        default:
            break;
    }

    return (KE_MSG_CONSUMED);
}

static int app_mesh_api_prov_auth_data_req_ind_handler(ke_msg_id_t const msgid,
        struct m_api_prov_auth_data_req_ind const *param,
        ke_task_id_t const dest_id,
        ke_task_id_t const src_id)
{
    MESH_APP_PRINT_INFO("app_mesh_api_prov_auth_data_req_ind_handler\n");
    MESH_APP_PRINT_INFO("auth_method:%x,auth_action:%x,auth_size:%x\n", param->auth_method, param->auth_action, param->auth_size);

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
    //78da07bcd71d;c52ab4202a675f6fdd6f44c3ad942fa6
    cfm->auth_data[0] = 0xa6; cfm->auth_data[1] = 0x2f; cfm->auth_data[2] = 0x94;
    cfm->auth_data[3] = 0xad; cfm->auth_data[4] = 0xc3; cfm->auth_data[5] = 0x44;
    cfm->auth_data[6] = 0x6f; cfm->auth_data[7] = 0xdd; cfm->auth_data[8] = 0x6f;
    cfm->auth_data[9] = 0x5f; cfm->auth_data[10] = 0x67; cfm->auth_data[11] = 0x2a;
    cfm->auth_data[12] = 0x20; cfm->auth_data[13] = 0xb4; cfm->auth_data[14] = 0x2a;
    cfm->auth_data[15] = 0xc5;
#endif // MAC78da07bcd71b

#if TEST_MESH_OTA

    uint8_t auth_value[16] =
    {
        0x08, 0x07, 0x06, 0x05, 0x04, 0x03, 0x02, 0x01,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0a
    };
    memcpy(cfm->auth_data, auth_value, 16);
#endif // TEST_MESH_OTA

    //  00 00 00 00 00 00 00 00 01 02 03 04 05 06 07 08
    for (int i =0 ; i < 16; i++)
    {
        MESH_APP_PRINT_INFO("cfm->auth_data[%d] = 0x%02x\n", i, cfm->auth_data[i]);
    }

    ke_msg_send(cfm);

    return (KE_MSG_CONSUMED);
}

extern bool user_data_contains_ali_data(void);
extern bool user_data_read_ali_mac(uint8_t *addr, uint8_t mode);
static void app_get_prov_param(m_api_prov_param_cfm_t* cfm, uint8_t adv_type)
{
    cfm->dev_uuid[0] = 0xa8; cfm->dev_uuid[1] = 0x01; // CID
    cfm->dev_uuid[2] = 0x71;// PID

    cfm->dev_uuid[3] = 0x33; cfm->dev_uuid[4] = 0x02; cfm->dev_uuid[5] = 0x00; cfm->dev_uuid[6] = 0x00; // PRODUCT ID

#if MAC78da07bcd71b
    cfm->dev_uuid[7] = 0x1b; cfm->dev_uuid[8] = 0xd7;
    cfm->dev_uuid[9] = 0xbc; cfm->dev_uuid[10] = 0x07;
    cfm->dev_uuid[11] = 0xda; cfm->dev_uuid[12] = 0x78; //MAC
    cfm->dev_uuid[13] = 0x00; cfm->dev_uuid[14] = 0xaa; cfm->dev_uuid[15] = 0xaa; //RFU
    cfm->uri_hash = 0x0;
    cfm->oob_info = 0x0000;
#elif   MAC78da07bcd71c
    cfm->dev_uuid[7] = 0x1c; cfm->dev_uuid[8] = 0xd7;
    cfm->dev_uuid[9] = 0xbc; cfm->dev_uuid[10] = 0x07;
    cfm->dev_uuid[11] = 0xda; cfm->dev_uuid[12] = 0x78; //MAC
    cfm->dev_uuid[13] = 0x00; cfm->dev_uuid[14] = 0x00; cfm->dev_uuid[15] = 0x00; //RFU
    cfm->uri_hash = 0x0;
    cfm->oob_info = 0x0000;
#elif   MAC78da07bcd71d
    cfm->dev_uuid[7] = 0x1d; cfm->dev_uuid[8] = 0xd7;
    cfm->dev_uuid[9] = 0xbc; cfm->dev_uuid[10] = 0x07;
    cfm->dev_uuid[11] = 0xda; cfm->dev_uuid[12] = 0x78; //MAC
    cfm->dev_uuid[13] = 0x00; cfm->dev_uuid[14] = 0x00; cfm->dev_uuid[15] = 0x00; //RFU
    cfm->uri_hash = 0x0;
    cfm->oob_info = 0x0000;
#endif /* MAC78da07bcd71b */

#if 1 //ALI_MESH
    /** set device uuid */
//    ali_uuid_t dev_uuid;
    uint8_t bt_addr[6];
    uint32_t product_id;
    if (user_data_contains_ali_data())
    {
        //product_id = user_data_read_ali_product_id();
        if (user_data_read_ali_mac(bt_addr, 1))
        {
            memcpy(&cfm->dev_uuid[7] , bt_addr, sizeof(bt_addr));
        }
		
	#if 0
        dev_uuid.cid = 0x01A8; //!< taobao
        dev_uuid.pid.adv_ver = 1;
        dev_uuid.pid.sec = 1;
        dev_uuid.pid.ota = 0;
        dev_uuid.pid.bt_ver = 1;
        dev_uuid.product_id = product_id;// PRODUCT ID

        // gap_get_param(GAP_PARAM_BD_ADDR, bt_addr);
        memcpy(dev_uuid.mac_addr, bt_addr, sizeof(bt_addr));

        dev_uuid.feature_flag = 0x00;
        if (adv_type == 0x1)
        {
            dev_uuid.feature_flag = 0x01;
        }
        memset(dev_uuid.rfu, 0, sizeof(dev_uuid.rfu));
		
        memcpy(cfm->dev_uuid, (uint8_t *)&dev_uuid, 16);
	#endif

    }
#endif //ALI_MESH 

    for (int i =0 ; i < 16; i++)
    {
        MESH_APP_PRINT_INFO("cfm->dev_uuid[%d] = 0x%02x\n", i, cfm->dev_uuid[i]);
    }
    cfm->static_oob = 0; //M_PROV_STATIC_OOB_AVAILABLE;
    cfm->pub_key_oob = 0; //M_PROV_PUB_KEY_OOB_USED;
    cfm->out_oob_size = 0;
    cfm->in_oob_size = 0;
    cfm->out_oob_action =  0; //M_PROV_OUT_OOB_NUMERIC;
    cfm->in_oob_action = 0;
    cfm->nb_elt = 4;
    cfm->info = 0; //M_PROV_INFO_URI_HASH_PRESENT;
}

static int app_mesh_api_prov_param_req_ind_handler(ke_msg_id_t const msgid,
        void const *param,
        ke_task_id_t const dest_id,
        ke_task_id_t const src_id)
{
    MESH_APP_PRINT_INFO("app_mesh_api_prov_param_req_ind_handler\n");
    //sean add
    m_api_prov_param_cfm_t *cfm = KE_MSG_ALLOC(MESH_API_PROV_PARAM_CFM, prf_get_task_from_id(TASK_ID_MESH), TASK_APP, m_api_prov_param_cfm);

    app_get_prov_param(cfm, 0);
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

    MESH_APP_PRINT_INFO("state :%d,status:%x\n", param->state, param->status);
    
    if (param->state == M_PROV_STARTED)
    {
    	  MESH_APP_PRINT_INFO("light_prov_startl\n");
        //light_prov_start();
        //gpio_triger(0x11);
    }
    else if (param->state == M_PROV_FAILED)
    {

        MESH_APP_PRINT_INFO("light_prov_fail\n");
        //ight_prov_fail();
    }
    else if (param->state == M_PROV_SUCCEED)
    {
        //gpio_triger(0x11);
        //gpio_triger(0x11);
        //gpio_triger(0x11);
        MESH_APP_PRINT_INFO("light_prov_success\n");

        m_tb_store_config(10);
        m_tb_state_set_relay_state(1, 1);
        app_unprov_adv_timeout_set(0);

#if (!TEST_MESH_OTA)
        m_lid_t net_key_lid = MESH_INVALID_LID;
        // Get local identifier of added network key
        if (m_tb_key_net_next(&net_key_lid, NULL) == MESH_ERR_NO_ERROR)
        {
#if (BLE_MESH_GATT_PROXY)
            // Inform API if provisioned to inform that proxy could be started
            m_tb_state_set_gatt_proxy_state(M_CONF_GATT_PROXY_STATE_ENABLED);
            m_api_proxy_ctrl(2);
#endif /* BLE_MESH_GATT_PROXY */
        }
#endif /* !TEST_MESH_OTA */
    }

    return (KE_MSG_CONSUMED);
}

static int app_mesh_api_compo_data_ind_handler(
                    ke_msg_id_t const msgid,
                    void const *param,
                    ke_task_id_t const dest_id,
                    ke_task_id_t const src_id)
{
    m_tb_store_config(5);
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

    {MESH_API_PROV_PARAM_REQ_IND,               (ke_msg_func_t)app_mesh_api_prov_param_req_ind_handler},

    {MESH_API_ATTENTION_UPDATE_IND,             (ke_msg_func_t)app_mesh_api_prov_attention_update_ind_handler},

    {MESH_API_PROV_STATE_IND,                   (ke_msg_func_t)app_mesh_api_prov_state_ind_handler},
    {MESH_API_COMPO_DATA_REQ_IND,               (ke_msg_func_t)app_mesh_api_compo_data_ind_handler},
};

const struct ke_state_handler app_mesh_table_handler =
{&app_mesh_msg_handler_list[0], (sizeof(app_mesh_msg_handler_list)/sizeof(struct ke_msg_handler))};

