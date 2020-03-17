/**
 ****************************************************************************************
 *
 * @file appm_task.c
 *
 * @brief RW APP Task implementation
 *
 * Copyright (C) RivieraWaves 2009-2015
 *
 *
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"          // SW configuration

#if (BLE_APP_PRESENT)
#include <string.h>
#include "app_task.h"              // Application Manager Task API
#include "app.h"                      // Application Manager Definition
#include "gapc_task.h"            // GAP Controller Task API
#include "gapm_task.h"          // GAP Manager Task API
#include "gattc_task.h"
#include "arch.h"                    // Platform Definitions

#include "ke_timer.h"             // Kernel timer
#include "app_fff0.h"              // fff0 Module Definition
#include "fff0s_task.h"
#include "app_dis.h"              // Device Information Module Definition
#include "diss_task.h"
#include "app_batt.h"             // Battery Module Definition
#include "bass_task.h"
#include "app_oads.h"
#include "oads_task.h"
#include "gpio.h"
#include "audio.h"
#include "uart.h"
#include "BK3435_reg.h"
#include "icu.h"
#include "reg_ble_em_cs.h"
#include "lld.h"
#include "wdt.h"
#include "app_mesh.h"
#include "mesh_general_api.h"
#include "app_mm_msg.h"
#include "led.h"

#include "app_light_ali_server.h"
extern void light_ali_quick_onoff_timer(void);

#if GMA_SUPPORT
extern const struct ke_state_handler app_gmas_table_handler;
#endif

/*
 * LOCAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */

static uint8_t appm_get_handler(const struct ke_state_handler *handler_list,
                                ke_msg_id_t msgid,
                                void *param,
                                ke_task_id_t src_id)
{
    // Counter
    uint8_t counter;

    // Get the message handler function by parsing the message table
    for (counter = handler_list->msg_cnt; 0 < counter; counter--)
    {

        struct ke_msg_handler handler = (*(handler_list->msg_table + counter - 1));

        if ((handler.id == msgid) ||
                (handler.id == KE_MSG_DEFAULT_HANDLER))
        {
            // If handler is NULL, message should not have been received in this state
            ASSERT_ERR(handler.func);

            return (uint8_t)(handler.func(msgid, param, TASK_APP, src_id));
        }
    }

    // If we are here no handler has been found, drop the message
    return (KE_MSG_CONSUMED);
}

/*
 * MESSAGE HANDLERS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Handles ready indication from the GAP. - Reset the stack
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int gapm_device_ready_ind_handler(ke_msg_id_t const msgid,
        void const *param,
        ke_task_id_t const dest_id,
        ke_task_id_t const src_id)
{
    // Application has not been initialized
    ASSERT_ERR(ke_state_get(dest_id) == APPM_INIT);

    // Reset the stack
    struct gapm_reset_cmd* cmd = KE_MSG_ALLOC(GAPM_RESET_CMD,
                                 TASK_GAPM, TASK_APP,
                                 gapm_reset_cmd);

    cmd->operation = GAPM_RESET;

    ke_msg_send(cmd);

    return (KE_MSG_CONSUMED);
}


/**
 ****************************************************************************************
 * @brief Handles GAP manager command complete events.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int gapm_cmp_evt_handler(ke_msg_id_t const msgid,
                                struct gapm_cmp_evt const *param,
                                ke_task_id_t const dest_id,
                                ke_task_id_t const src_id)
{
    MESH_APP_PRINT_INFO("gapm cmp:param->operation = 0x%x, param->status = 0x%x \n", param->operation, param->status);
    switch (param->operation)
    {
        // Reset completed
        case (GAPM_RESET):
        {
            if (param->status == GAP_ERR_NO_ERROR)
            {
                // Set Device configuration
                struct gapm_set_dev_config_cmd* cmd = KE_MSG_ALLOC(GAPM_SET_DEV_CONFIG_CMD,
                                                      TASK_GAPM, TASK_APP,
                                                      gapm_set_dev_config_cmd);
                // Set the operation
                cmd->operation = GAPM_SET_DEV_CONFIG;
                // Set the device role - Peripheral
                cmd->role      = GAP_ROLE_PERIPHERAL;
                // Set Data length parameters
                cmd->sugg_max_tx_octets = BLE_MIN_OCTETS;
                cmd->sugg_max_tx_time   = BLE_MIN_TIME;

                cmd->max_mtu = 131;//BLE_MIN_OCTETS;
                //Do not support secure connections
                cmd->pairing_mode = GAPM_PAIRING_LEGACY;

                cmd->att_cfg |= GAPM_MASK_ATT_SVC_CHG_EN;

                //cmd->addr_type   = GAPM_CFG_ADDR_HOST_PRIVACY; //2017-10-24 by alen
                // load IRK
                memcpy(cmd->irk.key, app_env.loc_irk, KEY_LEN);

                // Send message
                ke_msg_send(cmd);
            }
            else
            {
                ASSERT_ERR(0);
            }
        }
        break;
        case (GAPM_PROFILE_TASK_ADD):// 0x1B
        {
            // Add the next requested service
            if (!appm_add_svc())
            {
                // Go to the ready state
                ke_state_set(TASK_APP, APPM_READY);

                //appm_set_dev_name(APP_DFLT_DEVICE_NAME, strlen(APP_DFLT_DEVICE_NAME));
                light_ali_app_init();

                app_mesh_add_models_server();

                app_store_mesh_info();

            }
        }
        break;
        // Device Configuration updated
        case (GAPM_SET_DEV_CONFIG):
        {
            ASSERT_INFO(param->status == GAP_ERR_NO_ERROR, param->operation, param->status);

            // Go to the create db state
            ke_state_set(TASK_APP, APPM_CREATE_DB);
            // Add the first required service in the database
            // and wait for the PROFILE_ADDED_IND
            appm_add_svc();
        }
        break;

        case (GAPM_ADV_NON_CONN):
        case (GAPM_ADV_UNDIRECT):
        case (GAPM_ADV_DIRECT):
        case (GAPM_UPDATE_ADVERTISE_DATA):
        case (GAPM_ADV_DIRECT_LDC):
        {
            if (param->status == GAP_ERR_TIMEOUT)
            {
                ke_state_set(TASK_APP, APPM_READY);

                //device not bonded, start general adv
                //appm_start_advertising();
            }
        }
        break;

        default:
        {
            // Drop the message
        }
        break;
    }

    return (KE_MSG_CONSUMED);
}

static int gapc_get_dev_info_req_ind_handler(ke_msg_id_t const msgid,
        struct gapc_get_dev_info_req_ind const *param,
        ke_task_id_t const dest_id,
        ke_task_id_t const src_id)
{
    switch (param->req)
    {
        case GAPC_DEV_NAME:
        {
            struct gapc_get_dev_info_cfm * cfm = KE_MSG_ALLOC_DYN(GAPC_GET_DEV_INFO_CFM,
                                                 src_id, dest_id,
                                                 gapc_get_dev_info_cfm, APP_DEVICE_NAME_MAX_LEN);
            cfm->req = param->req;
            cfm->info.name.length = appm_get_dev_name(cfm->info.name.value);

            // Send message
            ke_msg_send(cfm);
        } break;

        case GAPC_DEV_APPEARANCE:
        {
            // Allocate message
            struct gapc_get_dev_info_cfm *cfm = KE_MSG_ALLOC(GAPC_GET_DEV_INFO_CFM,
                                                src_id, dest_id,
                                                gapc_get_dev_info_cfm);
            cfm->req = param->req;

            // No appearance
            cfm->info.appearance = 0;

            // Send message
            ke_msg_send(cfm);
        } break;

        case GAPC_DEV_SLV_PREF_PARAMS:
        {
            // Allocate message
            struct gapc_get_dev_info_cfm *cfm = KE_MSG_ALLOC(GAPC_GET_DEV_INFO_CFM,
                                                src_id, dest_id,
                                                gapc_get_dev_info_cfm);
            cfm->req = param->req;
            // Slave preferred Connection interval Min
            cfm->info.slv_params.con_intv_min = 8;
            // Slave preferred Connection interval Max
            cfm->info.slv_params.con_intv_max = 10;
            // Slave preferred Connection latency
            cfm->info.slv_params.slave_latency = 0;
            // Slave preferred Link supervision timeout
            cfm->info.slv_params.conn_timeout  = 600;  // 6s (600*10ms)

            // Send message
            ke_msg_send(cfm);
        } break;

        default: /* Do Nothing */
            break;
    }


    return (KE_MSG_CONSUMED);
}
/**
 ****************************************************************************************
 * @brief Handles GAPC_SET_DEV_INFO_REQ_IND message.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int gapc_set_dev_info_req_ind_handler(ke_msg_id_t const msgid,
        struct gapc_set_dev_info_req_ind const *param,
        ke_task_id_t const dest_id,
        ke_task_id_t const src_id)
{
    // Set Device configuration
    struct gapc_set_dev_info_cfm* cfm = KE_MSG_ALLOC(GAPC_SET_DEV_INFO_CFM, src_id, dest_id,
                                        gapc_set_dev_info_cfm);
    // Reject to change parameters
    cfm->status = GAP_ERR_REJECTED;
    cfm->req = param->req;
    // Send message
    ke_msg_send(cfm);

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles connection complete event from the GAP. Enable all required profiles
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int gapc_connection_req_ind_handler(ke_msg_id_t const msgid,
        struct gapc_connection_req_ind const *param,
        ke_task_id_t const dest_id,
        ke_task_id_t const src_id)
{
    MESH_APP_PRINT_INFO("%s\n", __func__);

    app_env.conidx = KE_IDX_GET(src_id);
    // Check if the received Connection Handle was valid
    if (app_env.conidx != GAP_INVALID_CONIDX)
    {
        // Retrieve the connection info from the parameters
        app_env.conhdl = param->conhdl;

        //m_api_proxy_ctrl(0);

        // Send connection confirmation
        struct gapc_connection_cfm *cfm = KE_MSG_ALLOC(GAPC_CONNECTION_CFM,
                                          KE_BUILD_ID(TASK_GAPC, app_env.conidx), TASK_APP,
                                          gapc_connection_cfm);

        cfm->auth = GAP_AUTH_REQ_NO_MITM_NO_BOND;
        // Send the message
        ke_msg_send(cfm);

        /*--------------------------------------------------------------
         * ENABLE REQUIRED PROFILES
         *--------------------------------------------------------------*/

        // Enable Battery Service
        app_batt_enable_prf(app_env.conhdl);

        // We are now in connected State
        ke_state_set(dest_id, APPM_CONNECTED);

#if UPDATE_CONNENCT_PARAM
        //ke_timer_set(APP_PARAM_UPDATE_REQ_IND,TASK_APP,100);
#endif

        if (m_tb_state_get_prov_state() == M_TB_STATE_PROV_STATE_PROV)
        {
            MESH_APP_PRINT_INFO("M_TB_STATE_PROV_STATE_PROV\n");
        }
        else
        {
            MESH_APP_PRINT_INFO("M_TB_STATE_PROV_STATE_UNPROV\n");
        }

    }
    else
    {
        // No connection has been establish, restart advertising
        //appm_start_advertising();
    }

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles GAP controller command complete events.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int gapc_cmp_evt_handler(ke_msg_id_t const msgid,
                                struct gapc_cmp_evt const *param,
                                ke_task_id_t const dest_id,
                                ke_task_id_t const src_id)
{
    MESH_APP_PRINT_INFO("gapc_cmp_evt_handler operation = %x\n", param->operation);
    switch (param->operation)
    {
        case (GAPC_UPDATE_PARAMS):  //0x09
        {
            if (param->status != GAP_ERR_NO_ERROR)
            {
                MESH_APP_PRINT_INFO("gapc update params fail !\n");
            }
            else
            {
                MESH_APP_PRINT_INFO("gapc update params ok !\n");
            }

        } break;

        case (GAPC_SECURITY_REQ): //0x0c
        {
            if (param->status != GAP_ERR_NO_ERROR)
            {
                MESH_APP_PRINT_INFO("gapc security req fail !\n");
            }
            else
            {
                MESH_APP_PRINT_INFO("gapc security req ok !\n");
            }
        } break;
        case (GAPC_BOND): // 0xa
        {
            if (param->status != GAP_ERR_NO_ERROR)
            {
                MESH_APP_PRINT_INFO("gapc bond fail !\n");
            }
            else
            {
                MESH_APP_PRINT_INFO("gapc bond ok !\n");
            }
        } break;

        case (GAPC_ENCRYPT): // 0xb
        {
            if (param->status != GAP_ERR_NO_ERROR)
            {
                MESH_APP_PRINT_INFO("gapc encrypt start fail !\r\n");
            }
            else
            {
                MESH_APP_PRINT_INFO("gapc encrypt start ok !\r\n");
            }
        }
        break;


        default:
            break;
    }

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles disconnection complete event from the GAP.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int gapc_disconnect_ind_handler(ke_msg_id_t const msgid,
                                       struct gapc_disconnect_ind const *param,
                                       ke_task_id_t const dest_id,
                                       ke_task_id_t const src_id)
{
    MESH_APP_PRINT_INFO("disconnect link reason = 0x%x\n", param->reason);
#if UPDATE_CONNENCT_PARAM
    if (ke_timer_active(APP_PARAM_UPDATE_REQ_IND, TASK_APP))
    {
        ke_timer_clear(APP_PARAM_UPDATE_REQ_IND, TASK_APP);
    }
#endif

    // Go to the ready state
    ke_state_set(TASK_APP, APPM_READY);

    wdt_disable_flag = 1;

    // Restart Advertising
    //appm_start_advertising();

#if (BLE_MESH_GATT_PROXY)
    m_api_proxy_ctrl(2);
#endif


    return (KE_MSG_CONSUMED);
}


/**
 ****************************************************************************************
 * @brief Handles profile add indication from the GAP.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int gapm_profile_added_ind_handler(ke_msg_id_t const msgid,
        struct gapm_profile_added_ind *param,
        ke_task_id_t const dest_id,
        ke_task_id_t const src_id)
{
    // Current State
    uint8_t state = ke_state_get(dest_id);

    if (state == APPM_CREATE_DB)
    {
        switch (param->prf_task_id)
        {
            default:
                break;
        }
    }
    else
    {
        ASSERT_INFO(0, state, src_id);
    }

    return KE_MSG_CONSUMED;
}


/*******************************************************************************
 * Function: app_period_timer_handler
 * Description: app period timer process
 * Input: msgid -Id of the message received.
 *        param -Pointer to the parameters of the message.
 *        dest_id -ID of the receiving task instance (TASK_GAP).
 *        ID of the sending task instance.
 * Return: If the message was consumed or not.
 * Others: void
*******************************************************************************/
static int app_period_timer_handler(ke_msg_id_t const msgid,
                                    void *param,
                                    ke_task_id_t const dest_id,
                                    ke_task_id_t const src_id)
{
    MESH_APP_PRINT_INFO("%s\n", __func__);

    //  ke_timer_set(APP_PERIOD_TIMER,TASK_APP,100);
    return KE_MSG_CONSUMED;
}


/**
 ****************************************************************************************
 * @brief Handles reception of all messages sent from the lower layers to the application
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int appm_msg_handler(ke_msg_id_t const msgid,
                            void *param,
                            ke_task_id_t const dest_id,
                            ke_task_id_t const src_id)
{
    // Retrieve identifier of the task from received message
    ke_task_id_t src_task_id = MSG_T(msgid);
    // Message policy
    uint8_t msg_pol          = KE_MSG_CONSUMED;


    switch (src_task_id)
    {
        case (TASK_ID_GAPC):
        {
            // else drop the message
        } break;

        case (TASK_ID_GATTC):
        {
            // Service Changed - Drop
        } break;

        case (TASK_ID_FFF0S):
        {
            // Call the Health Thermometer Module
            msg_pol = appm_get_handler(&app_fff0_table_handler, msgid, param, src_id);
        } break;

        case (TASK_ID_DISS):
        {
            // Call the Device Information Module
            msg_pol = appm_get_handler(&app_dis_table_handler, msgid, param, src_id);
        } break;

        case (TASK_ID_BASS):
        {
            // Call the Battery Module
            msg_pol = appm_get_handler(&app_batt_table_handler, msgid, param, src_id);
        } break;

        case (TASK_ID_OADS):
        {
            // Call the Health Thermometer Module
            msg_pol = appm_get_handler(&app_oads_table_handler, msgid, param, src_id);
        } break;


        case (TASK_ID_MESH):
        {
            // Call the Health Thermometer Module
            msg_pol = appm_get_handler(&app_mesh_table_handler, msgid, param, src_id);//
        } break;
        
#if GMA_SUPPORT
        case (TASK_ID_GMAS):
        {
            // Call the GMA Module
            msg_pol = appm_get_handler(&app_gmas_table_handler, msgid, param, src_id);
        } break;
#endif

        default:
        {
        } break;
    }

    return (msg_pol);
}


/*******************************************************************************
 * Function: gapc_update_conn_param_req_ind_handler
 * Description: Update request command processing from slaver connection parameters
 * Input: msgid   -Id of the message received.
 *        param   -Pointer to the parameters of the message.
 *        dest_id -ID of the receiving task instance
 *        src_id  -ID of the sending task instance.
 * Return: If the message was consumed or not.
 * Others: void
*******************************************************************************/
static int gapc_update_conn_param_req_ind_handler (ke_msg_id_t const msgid,
        const struct gapc_param_update_req_ind  *param,
        ke_task_id_t const dest_id, ke_task_id_t const src_id)
{

    MESH_APP_PRINT_INFO("slave send param_update_req\n");
    struct gapc_conn_param  up_param;

    up_param.intv_min   = BLE_UAPDATA_MIN_INTVALUE;
    up_param.intv_max   = BLE_UAPDATA_MAX_INTVALUE;
    up_param.latency    = BLE_UAPDATA_LATENCY;
    up_param.time_out   = BLE_UAPDATA_TIMEOUT;

    appm_update_param(&up_param);

    return KE_MSG_CONSUMED;
}


/*******************************************************************************
 * Function: gapc_le_pkt_size_ind_handler
 * Description: GAPC_LE_PKT_SIZE_IND
 * Input: msgid   -Id of the message received.
 *        param   -Pointer to the parameters of the message.
 *        dest_id -ID of the receiving task instance
 *        src_id  -ID of the sending task instance.
 * Return: If the message was consumed or not.
 * Others: void
*******************************************************************************/
static int gapc_le_pkt_size_ind_handler (ke_msg_id_t const msgid,
        const struct gapc_le_pkt_size_ind  *param,
        ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    MESH_APP_PRINT_INFO("%s \n", __func__);
    MESH_APP_PRINT_INFO("1max_rx_octets = %d\n", param->max_rx_octets);
    MESH_APP_PRINT_INFO("1max_rx_time = %d\n", param->max_rx_time);
    MESH_APP_PRINT_INFO("1max_tx_octets = %d\n", param->max_tx_octets);
    MESH_APP_PRINT_INFO("1max_tx_time = %d\n", param->max_tx_time);

    return KE_MSG_CONSUMED;
}

/**
 ****************************************************************************************
 * @brief  GAPC_PARAM_UPDATED_IND
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int gapc_param_updated_ind_handler (ke_msg_id_t const msgid,
        const struct gapc_param_updated_ind  *param,
        ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    MESH_APP_PRINT_INFO("%s \n", __func__);
    MESH_APP_PRINT_INFO("con_interval = %d\n", param->con_interval);
    MESH_APP_PRINT_INFO("con_latency = %d\n", param->con_latency);
    MESH_APP_PRINT_INFO("sup_to = %d\n", param->sup_to);

    return KE_MSG_CONSUMED;
}


/**
 ****************************************************************************************
 * @brief  GATTC_MTU_CHANGED_IND
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int gattc_mtu_changed_ind_handler(ke_msg_id_t const msgid,
        struct gattc_mtu_changed_ind const *ind,
        ke_task_id_t const dest_id,
        ke_task_id_t const src_id)
{
    MESH_APP_PRINT_INFO("%s \n", __func__);
    MESH_APP_PRINT_INFO("ind->mtu = %d,seq = %d\n", ind->mtu, ind->seq_num);

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief   GAPC_PARAM_UPDATE_REQ_IND
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int gapc_param_update_req_ind_handler(ke_msg_id_t const msgid,
        struct gapc_param_update_req_ind const *param,
        ke_task_id_t const dest_id,
        ke_task_id_t const src_id)
{
    MESH_APP_PRINT_INFO("%s \n", __func__);
    // Prepare the GAPC_PARAM_UPDATE_CFM message
    struct gapc_param_update_cfm *cfm = KE_MSG_ALLOC(GAPC_PARAM_UPDATE_CFM,
                                        src_id, dest_id,
                                        gapc_param_update_cfm);

    cfm->ce_len_max = 0xffff;
    cfm->ce_len_min = 0xffff;
    cfm->accept = true;

    // Send message
    ke_msg_send(cfm);

    return (KE_MSG_CONSUMED);
}

/*
 * GLOBAL VARIABLES DEFINITION
 ****************************************************************************************
 */


static int hci_ahi_msg_process_handler(ke_msg_id_t const msgid,
                                       struct hci_ahi_msg_pro_t *param, ke_task_id_t const dest_id,
                                       ke_task_id_t const src_id)
{
    uint8_t states = 0;

    static uint8_t wait_cnt = 0;
    MESH_APP_PRINT_INFO("hci_ahi_msg_process_handler start\n");
    uint8_t (*callback) (void*, uint8_t) = NULL;
    void* data =NULL;


    callback = param->callback;
    data = param->dummy;
    if (callback != NULL)
    {
        // Call handler
        MESH_APP_PRINT_INFO("callback 0x%08x\n", callback);
        {
            states = callback(data, RWIP_EIF_STATUS_OK);
        }

    }

    wait_cnt = 0;
    MESH_APP_PRINT_INFO("CONSUMED hci_ahi_msg_process_handler end,states : %d,wait_cnt:%d\n", states, wait_cnt);
    return (KE_MSG_CONSUMED);
}

static int hci_ahi_msg_timer_handler(ke_msg_id_t const msgid,
                                     void *param, ke_task_id_t const dest_id,
                                     ke_task_id_t const src_id)
{
    MESH_APP_PRINT_INFO("hci_ahi_msg_timer_handler\n");

    ke_state_t states = ke_state_get(dest_id);
    ke_state_set(dest_id, APPM_PRO_HCI_AHI_MSG);
    ke_state_set(dest_id, states);

    return (KE_MSG_CONSUMED);
}

/* Default State handlers definition. */
const struct ke_msg_handler appm_default_state[] =
{
    // Note: first message is latest message checked by kernel so default is put on top.
    {KE_MSG_DEFAULT_HANDLER,        (ke_msg_func_t)appm_msg_handler},
    {GAPM_DEVICE_READY_IND,         (ke_msg_func_t)gapm_device_ready_ind_handler},
    {GAPM_CMP_EVT,                  (ke_msg_func_t)gapm_cmp_evt_handler},
    {GAPC_GET_DEV_INFO_REQ_IND,     (ke_msg_func_t)gapc_get_dev_info_req_ind_handler},
    {GAPC_SET_DEV_INFO_REQ_IND,     (ke_msg_func_t)gapc_set_dev_info_req_ind_handler},
    {GAPC_CONNECTION_REQ_IND,       (ke_msg_func_t)gapc_connection_req_ind_handler},
    {GAPC_CMP_EVT,                  (ke_msg_func_t)gapc_cmp_evt_handler},
    {GAPC_DISCONNECT_IND,           (ke_msg_func_t)gapc_disconnect_ind_handler},
    {GAPM_PROFILE_ADDED_IND,        (ke_msg_func_t)gapm_profile_added_ind_handler},
    {GAPC_LE_PKT_SIZE_IND,          (ke_msg_func_t)gapc_le_pkt_size_ind_handler},
    {GAPC_PARAM_UPDATED_IND,        (ke_msg_func_t)gapc_param_updated_ind_handler},
    {GATTC_MTU_CHANGED_IND,         (ke_msg_func_t)gattc_mtu_changed_ind_handler},
    {GAPC_PARAM_UPDATE_REQ_IND,     (ke_msg_func_t)gapc_param_update_req_ind_handler},
    {APP_PARAM_UPDATE_REQ_IND,      (ke_msg_func_t)gapc_update_conn_param_req_ind_handler},
    {APP_PERIOD_TIMER,              (ke_msg_func_t)app_period_timer_handler},

    {HCI_AHI_MSG_PRO_CMD,           (ke_msg_func_t)hci_ahi_msg_process_handler},

    {HCI_AHI_MSG_PRO_TMR,           (ke_msg_func_t)hci_ahi_msg_timer_handler},
};

/* Specifies the message handlers that are common to all states. */
const struct ke_state_handler appm_default_handler = KE_STATE_HANDLER(appm_default_state);

/* Defines the place holder for the states of all the task instances. */
ke_state_t appm_state[APP_IDX_MAX];

#endif //(BLE_APP_PRESENT)

/// @} APPTASK
