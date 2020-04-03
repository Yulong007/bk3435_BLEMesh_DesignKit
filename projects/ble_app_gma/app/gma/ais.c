/**
 ****************************************************************************************
 *
 * @file fff0s.c
 *
 * @brief fff0 Server Implementation.
 *
 * Copyright (C) beken 2009-2015
 *
 *
 ****************************************************************************************
 */
#include "rwip_config.h"

#if (BLE_GMA_SERVER)
#include "attm.h"
#include "ais.h"
#include "ais_task.h"
#include "prf_utils.h"
#include "prf.h"
#include "ke_mem.h"
#include "uart.h"
#include "mesh_log.h"
// GMA Database Description - Used to add attributes into the database
const struct attm_desc gmas_att_db[GMAS_IDX_NB] =
{
    // GMA Service
    [GMAS_IDX_SVC]        =   {ATT_DECL_PRIMARY_SERVICE, PERM(RD, ENABLE), 0, 0},
    
    // GMA_READ Characteristic Declaration
	[GMAS_IDX_READ_CHAR]  =   {ATT_DECL_CHARACTERISTIC, PERM(RD, ENABLE), 0, 0},
    // GMA_READ Characteristic Value
    [GMAS_IDX_READ_VAL]   =   {ATT_USER_SERVER_CHAR_READ, PERM(RD, ENABLE), PERM(RI, ENABLE), GMA_READ_DATA_LEN*sizeof(uint8_t)},

    // GMA_WRITE Characteristic Declaration
	[GMAS_IDX_WRITE_CHAR]  =   {ATT_DECL_CHARACTERISTIC, PERM(RD, ENABLE), 0, 0},
    // GMA_WRITE Characteristic Value
    [GMAS_IDX_WRITE_VAL]   =   {ATT_USER_SERVER_CHAR_WRITE, PERM(WRITE_REQ, ENABLE), PERM(RI, ENABLE), GMA_WRITE_DATA_LEN*sizeof(uint8_t)},

    // GMA_INDICATE Characteristic Declaration
	[GMAS_IDX_INDICATE_CHAR]  =   {ATT_DECL_CHARACTERISTIC, PERM(RD, ENABLE), 0, 0},
    // GMA_INDICATE Characteristic Value
    [GMAS_IDX_INDICATE_VAL]   =   {ATT_USER_SERVER_CHAR_INDICATE, PERM(IND, ENABLE) , PERM(RI, ENABLE), GMA_INDICATE_DATA_LEN*sizeof(uint8_t)},
    // GMA_INDICATE Characteristic Configuration
	[GMAS_IDX_INDICATE_CFG]   =   {ATT_DESC_CLIENT_CHAR_CFG, PERM(RD, ENABLE)|PERM(WRITE_REQ, ENABLE), 0, 0},

    // GMA_WRITEWITHNORESP Characteristic Declaration
	[GMAS_IDX_WRITEWITHNORESP_CHAR]  =   {ATT_DECL_CHARACTERISTIC, PERM(RD, ENABLE), 0, 0},
    // GMA_WRITEWITHNORESP Characteristic Value
    [GMAS_IDX_WRITEWITHNORESP_VAL]   =   {ATT_USER_SERVER_CHAR_WRITEWITHNORESP, PERM(WRITE_COMMAND, ENABLE), PERM(RI, ENABLE), GMA_WRITEWITHNORESP_DATA_LEN*sizeof(uint8_t)},

    // GMA_NOTIFY Characteristic Declaration
	[GMAS_IDX_NOTIFY_CHAR]  =   {ATT_DECL_CHARACTERISTIC, PERM(RD, ENABLE), 0, 0},
    // GMA_NOTIFY Characteristic Value
    [GMAS_IDX_NOTIFY_VAL]   =   {ATT_USER_SERVER_CHAR_NOTIFY, PERM(NTF, ENABLE), PERM(RI, ENABLE), GMA_NOTIFY_DATA_LEN*sizeof(uint8_t)},
    // GMA_NOTIFY Characteristic Configuration
	[GMAS_IDX_NOTIFY_CFG]   =   {ATT_DESC_CLIENT_CHAR_CFG, PERM(RD, ENABLE)|PERM(WRITE_REQ, ENABLE), 0, 0},
};

static uint8_t gmas_init (struct prf_task_env* env, uint16_t* start_hdl, uint16_t app_task, uint8_t sec_lvl, struct gmas_db_cfg* params)
{
    uint16_t shdl;
    struct gmas_env_tag* gmas_env = NULL;
    uint8_t status = GAP_ERR_NO_ERROR;
    // Service content flag
    uint16_t cfg_flag = 0xFFFF;

    //-------------------- allocate memory required for the profile  ---------------------
    gmas_env = (struct gmas_env_tag*) ke_malloc(sizeof(struct gmas_env_tag), KE_MEM_ATT_DB);
    memset(gmas_env, 0 , sizeof(struct gmas_env_tag));

    shdl = *start_hdl;

    //Create FFF0 in the DB
    //------------------ create the attribute database for the profile -------------------
    status = attm_svc_create_db(&(shdl), ATT_USER_SERVER_GMA, (uint8_t *)&cfg_flag,
            GMAS_IDX_NB, NULL, env->task, &gmas_att_db[0],
            (sec_lvl & (PERM_MASK_SVC_DIS | PERM_MASK_SVC_AUTH | PERM_MASK_SVC_EKS)));
				
    //-------------------- Update profile task information  ---------------------
    if (status == ATT_ERR_NO_ERROR)
    {
        // allocate BASS required environment variable
        env->env = (prf_env_t*) gmas_env;
        *start_hdl = shdl;
        gmas_env->start_hdl = *start_hdl;
        gmas_env->prf_env.app_task = app_task
                | (PERM_GET(sec_lvl, SVC_MI) ? PERM(PRF_MI, ENABLE) : PERM(PRF_MI, DISABLE));
        gmas_env->prf_env.prf_task = env->task | PERM(PRF_MI, DISABLE);

        // initialize environment variable
        env->id                     = TASK_ID_GMAS;
        env->desc.idx_max           = GMAS_IDX_MAX;
        env->desc.state             = gmas_env->state;
        env->desc.default_handler   = &gmas_default_handler;

        // service is ready, go into an Idle state
        ke_state_set(env->task, GMAS_IDLE);
    }
    else if(gmas_env != NULL)
    {
        ke_free(gmas_env);
    }
     
    return (status);
}

static void gmas_destroy(struct prf_task_env* env)
{
    struct gmas_env_tag* gmas_env = (struct gmas_env_tag*) env->env;

    // clear on-going operation
    if(gmas_env->operation != NULL)
    {
        ke_free(gmas_env->operation);
    }

    // free profile environment variables
    env->env = NULL;
    ke_free(gmas_env);
}

static void gmas_create(struct prf_task_env* env, uint8_t conidx)
{
    struct gmas_env_tag* gmas_env = (struct gmas_env_tag*) env->env;
    ASSERT_ERR(conidx < BLE_CONNECTION_MAX);

    // force notification config to zero when peer device is connected
    gmas_env->tx_ntf_cfg[conidx] = 0;
}

static void gmas_cleanup(struct prf_task_env* env, uint8_t conidx, uint8_t reason)
{
    struct gmas_env_tag* gmas_env = (struct gmas_env_tag*) env->env;

    ASSERT_ERR(conidx < BLE_CONNECTION_MAX);
    // force notification config to zero when peer device is disconnected
    gmas_env->tx_ntf_cfg[conidx] = 0;
}

void gmas_data_send(struct gmas_env_tag* gmas_env, struct gma_tx_ind const *param)
{
    // Allocate the GATT notification message
    struct gattc_send_evt_cmd *gmas_req = KE_MSG_ALLOC_DYN(GATTC_SEND_EVT_CMD,
                                          KE_BUILD_ID(TASK_GATTC, gmas_env->conidx), prf_src_task_get(&(gmas_env->prf_env), 0),
                                          gattc_send_evt_cmd, sizeof(uint8_t)*(param->length));

    // Fill in the parameter structure
    if(param->operation == GATTC_NOTIFY)
    {
        gmas_req->operation = GATTC_NOTIFY;
        gmas_req->handle = gmas_get_att_handle(GMAS_IDX_NOTIFY_VAL);
    }
    else
    {
        gmas_req->operation = GATTC_INDICATE;
        gmas_req->handle = gmas_get_att_handle(GMAS_IDX_INDICATE_VAL);
    }
    gmas_req->length = param->length;
	memcpy(&gmas_req->value[0], &param->data[0],param->length);
    // send notification to peer device
    ke_msg_send(gmas_req);
}

/// gma Task interface required by profile manager
const struct prf_task_cbs gmas_itf =
{
    (prf_init_fnct) gmas_init,
    gmas_destroy,
    gmas_create,
    gmas_cleanup,
};

const struct prf_task_cbs* gmas_prf_itf_get(void)
{
   return &gmas_itf;
}

uint16_t gmas_get_att_handle( uint8_t att_idx)
{
		
    struct gmas_env_tag* gmas_env = PRF_ENV_GET(GMAS, gmas);
    uint16_t handle = ATT_INVALID_HDL;
   
    handle = gmas_env->start_hdl;

    // increment index according to expected index
    if(att_idx < GMAS_IDX_NB)
    {
        handle += att_idx;
    }	      
    else
    {
        handle = ATT_INVALID_HDL;
    }
    
    return handle;
}

uint8_t gmas_get_att_idx(uint16_t handle, uint8_t *att_idx)
{
    struct gmas_env_tag* gmas_env = PRF_ENV_GET(GMAS, gmas);
    uint8_t status = PRF_APP_ERROR;

    // Browse list of services
    // handle must be greater than current index 
    // check if it's a mandatory index
    if(handle <= (gmas_env->start_hdl + GMAS_IDX_NOTIFY_CFG))
    {
        *att_idx = handle - gmas_env->start_hdl;
        status = GAP_ERR_NO_ERROR;
    }
    
    return (status);
}
#endif // (BLE_gma_SERVER)
