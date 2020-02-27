/**
 ****************************************************************************************
 *
 * @file wechat.c
 *
 * @brief wechat Server Implementation.
 *
 * Copyright (C) beken 2009-2015
 *
 *
 ****************************************************************************************
 */

#include "rwip_config.h"

#if (BLE_WECHAT_SERVER)
#include "attm.h"
#include "wechat.h"
#include "wechat_task.h"
#include "prf_utils.h"
#include "prf.h"
#include "ke_mem.h"

#include "uart.h"



/*
 * FFF0 ATTRIBUTES DEFINITION
 ****************************************************************************************
 */

/// Full FFF0 Database Description - Used to add attributes into the database
const struct attm_desc wechat_att_db[WECHAT_IDX_NB] =
{
// Wechat Service Declaration
    [WECHAT_IDX_SVC]         = {ATT_DECL_PRIMARY_SERVICE, PERM(RD, ENABLE), 0, 0},
// Wechat Indicate Char Declaration
    [WECHAT_IDX_WRITE_CHAR]  = {ATT_DECL_CHARACTERISTIC, PERM(RD, ENABLE), 0, 0},
//  Characteristic Value
    [WECHAT_IDX_WRITE_VAL]  = {ATT_WECHAT_WRITE_CHAR, PERM(WRITE_REQ, ENABLE), PERM(RI, ENABLE), WECAHT_WRITE_DATA_LEN * sizeof(uint8_t)},
// Wechat read Char Declaration
    [WECHAT_IDX_READ_CHAR]  = {ATT_DECL_CHARACTERISTIC, PERM(RD, ENABLE), 0, 0},
// Characteristic Value
    [WECHAT_IDX_READ_VAL] = {ATT_WECHAT_READ_CHAR, PERM(RD, ENABLE), PERM(RI, ENABLE), WECAHT_READ_DATA_LEN * sizeof(uint8_t)},

// Wechat Indicate Char Declaration
    [WECHAT_IDX_INDICATE_CHAR] = {ATT_DECL_CHARACTERISTIC,   PERM(RD, ENABLE), 0, 0},
// Wechat Indicate Characteristic Value
    [WECHAT_IDX_INDICATE_VAL] = {ATT_WECHAT_INDICATE_CHAR,   PERM(IND, ENABLE), PERM(RI, ENABLE), WECAHT_INDI_DATA_LEN * sizeof(uint8_t)},

//Wechat Indicate Characteristic - Client Characteristic Configuration Descriptor
    [WECHAT_IDX_INDICATE_CFG] = {ATT_DESC_CLIENT_CHAR_CFG,  PERM(RD, ENABLE) | PERM(WRITE_REQ, ENABLE), 0, 0},

};



static uint8_t wechat_init (struct prf_task_env *env, uint16_t *start_hdl, uint16_t app_task, uint8_t sec_lvl,  struct wechat_db_cfg *params)
{
    uint16_t shdl;
    struct wechat_env_tag *wechat_env = NULL;
    // Status
    uint8_t status = GAP_ERR_NO_ERROR;

    //-------------------- allocate memory required for the profile  ---------------------
    wechat_env = (struct wechat_env_tag * ) ke_malloc(sizeof(struct wechat_env_tag), KE_MEM_ATT_DB);
    memset(wechat_env, 0, sizeof(struct wechat_env_tag));

    // Service content flag
    uint8_t cfg_flag = WECHAT_CFG_FLAG_MANDATORY_MASK;

    // Save database configuration
    wechat_env->features |= (params->features) ;

    // Check if notifications are supported
    if (params->features == WECHAT_INDICATE_SUP)
    {
        cfg_flag |= WECHAT_CFG_FLAG_NTF_SUP_MASK;
    }

    shdl = *start_hdl;

    //------------------ create the attribute database for the profile -------------------
    status = attm_svc_create_db(&(shdl), ATT_WECHAT_SERVICE, (uint8_t *)&cfg_flag,
                                WECHAT_IDX_NB, NULL, env->task, &wechat_att_db[0],
                                (sec_lvl & (PERM_MASK_SVC_DIS | PERM_MASK_SVC_AUTH | PERM_MASK_SVC_EKS)));

    //Set optional permissions
    if (status == GAP_ERR_NO_ERROR)
    {
        //Set optional permissions
        if (params->features == WECHAT_INDICATE_SUP)
        {
            // Battery Level characteristic value permissions
            //uint16_t perm = PERM(RD, ENABLE) | PERM(IND, ENABLE);
            uint16_t perm = PERM(IND, ENABLE);

            attm_att_set_permission(shdl + WECHAT_IDX_INDICATE_VAL, perm, 0);
        }
    }

    //-------------------- Update profile task information  ---------------------
    if (status == ATT_ERR_NO_ERROR)
    {
        // allocate BASS required environment variable
        env->env = (prf_env_t *) wechat_env;
        *start_hdl = shdl;
        wechat_env->start_hdl = *start_hdl;
        wechat_env->prf_env.app_task = app_task
                                       | (PERM_GET(sec_lvl, SVC_MI) ? PERM(PRF_MI, ENABLE) : PERM(PRF_MI, DISABLE));
        wechat_env->prf_env.prf_task = env->task | PERM(PRF_MI, DISABLE);

        // initialize environment variable
        env->id                     = TASK_ID_WECHAT;
        env->desc.idx_max           = WECAHT_IDX_MAX;
        env->desc.state             = wechat_env->state;
        env->desc.default_handler   = &wechat_default_handler;

        // service is ready, go into an Idle state
        ke_state_set(env->task, WECHAT_IDLE);
    }
    else if (wechat_env != NULL)
    {
        ke_free(wechat_env);
    }

    return (status);
}


static void wechat_destroy(struct prf_task_env *env)
{
    struct wechat_env_tag *wechat_env = (struct wechat_env_tag *) env->env;

    // clear on-going operation
    if (wechat_env->operation != NULL)
    {
        ke_free(wechat_env->operation);
    }

    // free profile environment variables
    env->env = NULL;
    ke_free(wechat_env);
}

static void wechat_create(struct prf_task_env *env, uint8_t conidx)
{
    struct wechat_env_tag *wechat_env = (struct wechat_env_tag *) env->env;
    ASSERT_ERR(conidx < BLE_CONNECTION_MAX);

    // force notification config to zero when peer device is connected
    wechat_env->indi_cfg[conidx] = 0;
}




static void wechat_cleanup(struct prf_task_env *env, uint8_t conidx, uint8_t reason)
{
    struct wechat_env_tag *wechat_env = (struct wechat_env_tag *) env->env;

    ASSERT_ERR(conidx < BLE_CONNECTION_MAX);
    // force notification config to zero when peer device is disconnected
    wechat_env->indi_cfg[conidx] = 0;
}


void wechat_indicate_lvl(struct wechat_env_tag *wechat_env, struct wechat_ind_upd_req const *param)
{
    // Allocate the GATT notification message
    struct gattc_send_evt_cmd *send_lvl = KE_MSG_ALLOC_DYN(GATTC_SEND_EVT_CMD,
                                          KE_BUILD_ID(TASK_GATTC, 0), prf_src_task_get(&(wechat_env->prf_env), 0),
                                          gattc_send_evt_cmd, sizeof(uint8_t) * WECAHT_INDI_DATA_LEN);

    // Fill in the parameter structure
    send_lvl->operation = GATTC_INDICATE;
    send_lvl->handle = wechat_get_att_handle(WECHAT_IDX_INDICATE_VAL);
    // pack measured value in database
    send_lvl->length = param->length;
    memcpy(&send_lvl->value[0], &param->indicate_buf[0], param->length);
    // send notification to peer device
    ke_msg_send(send_lvl);
}



/// BASS Task interface required by profile manager
const struct prf_task_cbs wechat_itf =
{
    (prf_init_fnct) wechat_init,
    wechat_destroy,
    wechat_create,
    wechat_cleanup,
};


const struct prf_task_cbs *wechat_prf_itf_get(void)
{
    return &wechat_itf;
}


uint16_t wechat_get_att_handle( uint8_t att_idx)
{

    struct wechat_env_tag *wechat_env = PRF_ENV_GET(WECHAT, wechat);
    uint16_t handle = ATT_INVALID_HDL;

    handle = wechat_env->start_hdl;

    // increment index according to expected index
    if (att_idx < WECHAT_IDX_INDICATE_CFG)
    {
        handle += att_idx;
    }
    // FFF1 notification
    else if ((att_idx == WECHAT_IDX_INDICATE_CFG) && (((wechat_env->features ) & 0x01) == WECHAT_INDICATE_SUP))
    {
        handle += WECHAT_IDX_INDICATE_CFG;

    }
    else
    {
        handle = ATT_INVALID_HDL;
    }

    return handle;
}

uint8_t wechat_get_att_idx(uint16_t handle, uint8_t *att_idx)
{


    struct wechat_env_tag *wechat_env = PRF_ENV_GET(WECHAT, wechat);
    uint16_t hdl_cursor = wechat_env->start_hdl;
    uint8_t status = PRF_APP_ERROR;

    // Browse list of services
    // handle must be greater than current index
    // check if it's a mandatory index
    if (handle <= (hdl_cursor + WECHAT_IDX_INDICATE_VAL))
    {
        *att_idx = handle - hdl_cursor;
        status = GAP_ERR_NO_ERROR;

    }

    hdl_cursor += WECHAT_IDX_INDICATE_VAL;

    // check if it's a notify index
    if (((wechat_env->features ) & 0x01) == WECHAT_INDICATE_SUP)
    {
        hdl_cursor++;
        if (handle == hdl_cursor)
        {
            *att_idx = WECHAT_IDX_INDICATE_CFG;
            status = GAP_ERR_NO_ERROR;
        }
    }

    hdl_cursor++;

    return (status);
}


#endif // (BLE_fff0_SERVER)




