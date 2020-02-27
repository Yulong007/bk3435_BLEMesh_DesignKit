/**
 ****************************************************************************************
 *
 * @file   fff0s_task.c
 *
 * @brief FFF0 Server Role Task Implementation.
 *
 * Copyright (C) Beken 2009-2015
 *
 *
 ****************************************************************************************
 */


/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"

#if (BLE_WECHAT_SERVER)

#include "gap.h"
#include "gattc_task.h"
#include "attm.h"
#include "atts.h"
#include "co_utils.h"
#include "wechat.h"
#include "wechat_task.h"
#include "uart.h"
#include "prf_utils.h"




static int wechat_ind_upd_req_handler(ke_msg_id_t const msgid,
                                      struct wechat_ind_upd_req const *param,
                                      ke_task_id_t const dest_id,
                                      ke_task_id_t const src_id)
{
    int msg_status = KE_MSG_SAVED;
    uint8_t state = ke_state_get(dest_id);

    UART_PRINTF("len-> %d\r\n", param->length);

    // check state of the task
    if (state == WECHAT_IDLE)
    {
        struct wechat_env_tag *wechat_env = PRF_ENV_GET(WECHAT, wechat);
        {
            // put task in a busy state
            ke_state_set(dest_id, WECHAT_BUSY);
            wechat_indicate_lvl(wechat_env, param);
            ke_state_set(dest_id, WECHAT_IDLE);
            msg_status = KE_MSG_CONSUMED;
        }
    }

    return (msg_status);
}



static int gattc_att_info_req_ind_handler(ke_msg_id_t const msgid,
        struct gattc_att_info_req_ind *param,
        ke_task_id_t const dest_id,
        ke_task_id_t const src_id)
{

    struct gattc_att_info_cfm *cfm;
    uint8_t  att_idx = 0;
    // retrieve handle information
    uint8_t status = wechat_get_att_idx(param->handle, &att_idx);

    //Send write response
    cfm = KE_MSG_ALLOC(GATTC_ATT_INFO_CFM, src_id, dest_id, gattc_att_info_cfm);
    cfm->handle = param->handle;

    if (status == GAP_ERR_NO_ERROR)
    {
        // check if it's a client configuration char
        if (att_idx == WECHAT_IDX_INDICATE_CFG)
        {
            // CCC attribute length = 2
            cfm->length = 2;
        }
        // not expected request
        else
        {
            cfm->length = 0;
            status = ATT_ERR_WRITE_NOT_PERMITTED;
        }
    }

    cfm->status = status;
    ke_msg_send(cfm);

    return (KE_MSG_CONSUMED);
}



static int gattc_write_req_ind_handler(ke_msg_id_t const msgid, struct gattc_write_req_ind const *param,
                                       ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    struct gattc_write_cfm *cfm;
    uint8_t att_idx = 0;
    uint8_t conidx = KE_IDX_GET(src_id);
    // retrieve handle information
    uint8_t status = wechat_get_att_idx(param->handle,  &att_idx);
    // If the attribute has been found, status is GAP_ERR_NO_ERROR
    if (status == GAP_ERR_NO_ERROR)
    {
        struct wechat_env_tag *wechat_env = PRF_ENV_GET(WECHAT, wechat);
        // Extract value before check
        uint16_t indi_cfg = co_read16p(&param->value[0]);

        // Only update configuration if value for stop or notification enable
        if ((att_idx == WECHAT_IDX_INDICATE_CFG)
                && ((indi_cfg == PRF_CLI_STOP_NTFIND) || (indi_cfg == PRF_CLI_START_IND)))
        {
            // Conserve information in environment
            if (indi_cfg == PRF_CLI_START_IND)   //0x0002
            {
                // Ntf cfg bit set to 1
                wechat_env->indi_cfg[conidx] |= (WECHAT_INDICATE_SUP );
            }
            else
            {
                // Ntf cfg bit set to 0
                wechat_env->indi_cfg[conidx] &= ~(WECHAT_INDICATE_SUP );
            }

            // Inform APP of configuration change
            struct wechat_indi_cfg_ind *ind = KE_MSG_ALLOC(WECHAT_INDICATE_IND_CFG_IND,
                                              prf_dst_task_get(&(wechat_env->prf_env), conidx), dest_id,
                                              wechat_indi_cfg_ind);
            ind->conidx = conidx;
            ind->indi_cfg = wechat_env->indi_cfg[conidx];

            ke_msg_send(ind);
        }
        else if (att_idx == WECHAT_IDX_WRITE_VAL)
        {
            // Allocate the alert value change indication
            struct wechat_write_ind *ind = KE_MSG_ALLOC(WECHAT_WRITER_REQ_IND,
                                           prf_dst_task_get(&(wechat_env->prf_env), conidx),
                                           dest_id, wechat_write_ind);

            // Fill in the parameter structure
            memcpy(ind->write_buf, &param->value[0], param->length);
            ind->conidx = conidx;
            ind->length = param->length;

            // Send the message
            ke_msg_send(ind);
        }
        else
        {
            status = PRF_APP_ERROR;
        }

    }

    //Send write response
    cfm = KE_MSG_ALLOC(GATTC_WRITE_CFM, src_id, dest_id, gattc_write_cfm);
    cfm->handle = param->handle;
    cfm->status = status;
    ke_msg_send(cfm);

    return (KE_MSG_CONSUMED);
}



static int gattc_read_req_ind_handler(ke_msg_id_t const msgid, struct gattc_read_req_ind const *param,
                                      ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    struct gattc_read_cfm *cfm;
    uint8_t  att_idx = 0;
    uint8_t conidx = KE_IDX_GET(src_id);
    // retrieve handle information
    uint8_t status = wechat_get_att_idx(param->handle, &att_idx);
    uint16_t length = 0;
    struct wechat_env_tag *wechat_env = PRF_ENV_GET(WECHAT, wechat);

    // If the attribute has been found, status is GAP_ERR_NO_ERROR
    if (status == GAP_ERR_NO_ERROR)
    {
        // read notification information
        if (att_idx == WECHAT_IDX_INDICATE_VAL)
        {
            length = WECAHT_INDI_DATA_LEN * sizeof(uint8_t);
        }
        else if (att_idx == WECHAT_IDX_READ_VAL)
        {
            length = WECAHT_READ_DATA_LEN * sizeof(uint8_t);
        }
        // read notification information
        else if (att_idx == WECHAT_IDX_INDICATE_CFG)
        {
            length = sizeof(uint16_t);
        }

        else
        {
            status = PRF_APP_ERROR;
        }
    }

    //Send write response
    cfm = KE_MSG_ALLOC_DYN(GATTC_READ_CFM, src_id, dest_id, gattc_read_cfm, length);
    cfm->handle = param->handle;
    cfm->status = status;
    cfm->length = length;

    if (status == GAP_ERR_NO_ERROR)
    {
        // read notification information
        if (att_idx == WECHAT_IDX_INDICATE_VAL)
        {
            cfm->value[0] = wechat_env->indi_buffer[0];
        }
        else if (att_idx == WECHAT_IDX_READ_VAL)
        {
            cfm->value[0] = wechat_env->read_buffer[0];
        }
        // retrieve notification config
        else if (att_idx == WECHAT_IDX_INDICATE_CFG)
        {
            uint16_t indi_cfg = (wechat_env->indi_cfg[conidx] & WECHAT_INDICATE_SUP) ? PRF_CLI_START_IND : PRF_CLI_STOP_NTFIND;
            co_write16p(cfm->value, indi_cfg);
        }
        // retrieve battery level format
        else
        {
            /* Not Possible */
        }
    }

    ke_msg_send(cfm);

    return (KE_MSG_CONSUMED);
}


static int gattc_cmp_evt_handler(ke_msg_id_t const msgid,  struct gattc_cmp_evt const *param,
                                 ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    struct wechat_env_tag *wechat_env = PRF_ENV_GET(WECHAT, wechat);

    struct gattc_cmp_evt *evt = KE_MSG_ALLOC(WECHAT_GATTC_CMP_EVT,
                                prf_dst_task_get(&(wechat_env->prf_env), 0),
                                dest_id, gattc_cmp_evt);

    evt->operation = param->operation;
    evt->status = param->status;
    evt->seq_num = param->seq_num;

    ke_state_set(dest_id, WECHAT_IDLE);
    ke_msg_send(evt);

    return (KE_MSG_CONSUMED);
}


/// Default State handlers definition
const struct ke_msg_handler wechat_default_state[] =
{
    {WECHAT_INDICATE_UPD_REQ,       (ke_msg_func_t) wechat_ind_upd_req_handler},
    {GATTC_ATT_INFO_REQ_IND,        (ke_msg_func_t) gattc_att_info_req_ind_handler},
    {GATTC_WRITE_REQ_IND,           (ke_msg_func_t) gattc_write_req_ind_handler},
    {GATTC_READ_REQ_IND,            (ke_msg_func_t) gattc_read_req_ind_handler},
    {GATTC_CMP_EVT,                 (ke_msg_func_t) gattc_cmp_evt_handler},
};

/// Specifies the message handlers that are common to all states.
const struct ke_state_handler wechat_default_handler = KE_STATE_HANDLER(wechat_default_state);

#endif /* #if (BLE_FFF0_SERVER) */





