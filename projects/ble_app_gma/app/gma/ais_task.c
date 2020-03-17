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

#if (BLE_GMA_SERVER)

#include "gap.h"
#include "gattc_task.h"
#include "attm.h"
#include "atts.h"
#include "co_utils.h"
#include "ais.h"
#include "ais_task.h"
#include "uart.h"
#include "prf_utils.h"

uint8_t gma_conidx = 0;

static int gma_data_send_handler(ke_msg_id_t const msgid,
                                          struct gma_tx_ind const *param,
                                          ke_task_id_t const dest_id,
                                          ke_task_id_t const src_id)
{
    int msg_status = KE_MSG_SAVED;
    uint8_t state = ke_state_get(dest_id);
	
    // check state of the task
    if(state == GMAS_IDLE)
    {
        struct gmas_env_tag* gmas_env = PRF_ENV_GET(GMAS, gmas);

        // put task in a busy state
        ke_state_set(dest_id, GMAS_BUSY);	
        gmas_env->conidx = gma_conidx;
		gmas_data_send(gmas_env, param);
		ke_state_set(dest_id, GMAS_IDLE);   
		msg_status = KE_MSG_CONSUMED;	
    }

    return (msg_status);
    
}

static int gattc_att_info_req_ind_handler(ke_msg_id_t const msgid,
                                                   struct gattc_att_info_req_ind *param,
                                                   ke_task_id_t const dest_id,
                                                   ke_task_id_t const src_id)
{

    struct gattc_att_info_cfm * cfm;
    uint8_t  att_idx = 0;
    // retrieve handle information
    uint8_t status = gmas_get_att_idx(param->handle, &att_idx);

    //Send write response
    cfm = KE_MSG_ALLOC(GATTC_ATT_INFO_CFM, src_id, dest_id, gattc_att_info_cfm);
    cfm->handle = param->handle;

    if(status == GAP_ERR_NO_ERROR)
    {
        // check if it's a client configuration char
        if(att_idx == GMAS_IDX_NOTIFY_CFG)
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
    struct gattc_write_cfm * cfm;
    uint8_t att_idx = 0;
    uint8_t conidx = KE_IDX_GET(src_id);
    // retrieve handle information
    uint8_t status = gmas_get_att_idx(param->handle,  &att_idx);
		
    // If the attribute has been found, status is GAP_ERR_NO_ERROR
    if (status == GAP_ERR_NO_ERROR)
    {
        struct gmas_env_tag* gmas_env = PRF_ENV_GET(GMAS, gmas);
        // Extract value before check
        uint16_t ntf_cfg = co_read16p(&param->value[0]);

        // Only update configuration if value for stop or notification enable
        if ((att_idx == GMAS_IDX_NOTIFY_CFG)
              && ((ntf_cfg == PRF_CLI_STOP_NTFIND) || (ntf_cfg == PRF_CLI_START_NTF)))
        {
            // Conserve information in environment
            if (ntf_cfg == PRF_CLI_START_NTF)
            {
                // Ntf cfg bit set to 1
                gmas_env->tx_ntf_cfg[conidx] |= (GMA_NTF_SUP );
            }
            else
            {
                // Ntf cfg bit set to 0
                gmas_env->tx_ntf_cfg[conidx] &= ~(GMA_NTF_SUP );
            }		
        }
        else if ((att_idx == GMAS_IDX_INDICATE_CFG)
              && ((ntf_cfg == PRF_CLI_STOP_NTFIND) || (ntf_cfg == PRF_CLI_START_IND)))
        {
            // Conserve information in environment
            if (ntf_cfg == PRF_CLI_START_IND)
            {
                // Ntf cfg bit set to 1
                gmas_env->tx_ntf_cfg[conidx] |= (GMA_IND_SUP );
            }
            else
            {
                // Ntf cfg bit set to 0
                gmas_env->tx_ntf_cfg[conidx] &= ~(GMA_IND_SUP );
            }		
        }
		else if ((att_idx == GMAS_IDX_WRITE_VAL) || (att_idx == GMAS_IDX_WRITEWITHNORESP_VAL))
		{
			// Allocate the alert value change indication
			struct gma_rx_ind *ind = KE_MSG_ALLOC(GMA_ID_RX,
                                			      prf_dst_task_get(&(gmas_env->prf_env), conidx),
                                			      dest_id, gma_rx_ind);
			
			// Fill in the parameter structure	
			memcpy(ind->data, &param->value[0], param->length);
			ind->length = param->length;
			
			// Send the message
			ke_msg_send(ind);
		}
        else
        {
            status = PRF_APP_ERROR;
        }
        gma_conidx = conidx;
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
    struct gattc_read_cfm * cfm;
    uint8_t  att_idx = 0;
    uint8_t conidx = KE_IDX_GET(src_id);
    // retrieve handle information
    uint8_t status = gmas_get_att_idx(param->handle, &att_idx);
    uint16_t length = 0;
    struct gmas_env_tag* gmas_env = PRF_ENV_GET(GMAS, gmas);

    // If the attribute has been found, status is GAP_ERR_NO_ERROR
    if (status == GAP_ERR_NO_ERROR)
    {
        if((att_idx == GMAS_IDX_INDICATE_CFG) || (att_idx == GMAS_IDX_NOTIFY_CFG))
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
        if (att_idx == GMAS_IDX_READ_VAL)
        {
            cfm->value[0] = gmas_env->tx_ntf_cfg[0];
        }
        // retrieve notification config
        else if (att_idx == GMAS_IDX_NOTIFY_CFG)
        {
            co_write16p(cfm->value, gmas_env->tx_ntf_cfg[conidx]);
        }  
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
    GMA_PRINTF("gattc_cmp operation:%x,status:%x\r\n",param->operation,param->status);
    if(param->operation == GATTC_NOTIFY)
    {	
    }
	// go back in to idle mode
    ke_state_set(dest_id, ke_state_get(dest_id) & ~GMAS_BUSY);

    return (KE_MSG_CONSUMED);
}

/// Default State handlers definition
const struct ke_msg_handler gmas_default_state[] =
{
    {GMA_ID_TX,                     (ke_msg_func_t) gma_data_send_handler},
    {GATTC_ATT_INFO_REQ_IND,        (ke_msg_func_t) gattc_att_info_req_ind_handler},
    {GATTC_WRITE_REQ_IND,           (ke_msg_func_t) gattc_write_req_ind_handler},
    {GATTC_READ_REQ_IND,            (ke_msg_func_t) gattc_read_req_ind_handler},
    {GATTC_CMP_EVT,                 (ke_msg_func_t) gattc_cmp_evt_handler},
    
};

/// Specifies the message handlers that are common to all states.
const struct ke_state_handler gmas_default_handler = KE_STATE_HANDLER(gmas_default_state);

#endif /* #if (BLE_FFF0_SERVER) */
