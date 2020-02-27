/**
 ****************************************************************************************
 *
 * @file app_braces.c
 *
 * @brief Bracelet Application Module entry point
 *
 * @auth  gang.cheng
 *
 * @date  2016.09.06
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
#include "mesh_log.h"

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include <string.h>
#include "app_oads.h"                // Bracese Application Module Definitions
#include "app.h"                     // Application Definitions
#include "app_task.h"                // application task definitions
#include "oads.h"
#include "oads_task.h"               // health thermometer functions
#include "co_bt.h"
#include "prf_types.h"               // Profile common types definition
#include "arch.h"                    // Platform Definitions
#include "prf.h"
#include "lld_evt.h"
#include "uart.h"

/*
 * LOCATION FUN DEFINES
 ****************************************************************************************
 */

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

/// braces Application Module Environment Structure
struct app_oads_env_tag app_oads_env;

/*
 * GLOBAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */

void app_oads_init(void)
{
    // Reset the environment
    memset(&app_oads_env, 0, sizeof(struct app_oads_env_tag));
}

void app_oad_add_oads(void)
{

    MESH_APP_PRINT_INFO("app_oad_add_oads\n");
    struct oads_db_cfg *db_cfg;

    struct gapm_profile_task_add_cmd *req = KE_MSG_ALLOC_DYN(GAPM_PROFILE_TASK_ADD_CMD,
                                            TASK_GAPM, TASK_APP,
                                            gapm_profile_task_add_cmd, sizeof(struct oads_db_cfg));
    // Fill message
    req->operation = GAPM_PROFILE_TASK_ADD;
    req->sec_lvl = 0;//PERM(SVC_AUTH, ENABLE);
    req->prf_task_id = TASK_ID_OADS;
    req->app_task = TASK_APP;
    req->start_hdl = 0; //req->start_hdl = 0; dynamically allocated


    // Set parameters
    db_cfg = (struct oads_db_cfg* ) req->param;

    // Sending of notifications is supported
    db_cfg->features = OADS_NTF_SUP;
    MESH_APP_PRINT_INFO("app_oad_add_oads d = %x,s = %x\n", TASK_GAPM, TASK_APP);
    // Send the message
    ke_msg_send(req);
}

void app_oads_enable_prf(uint8_t conidx)
{
    app_oads_env.conidx = conidx;

    // Allocate the message
    struct oads_enable_req * req = KE_MSG_ALLOC(OADS_ENABLE_REQ,
                                   prf_get_task_from_id(TASK_ID_OADS),
                                   TASK_APP,
                                   oads_enable_req);

    // Fill in the parameter structure
    req->conidx             = conidx;

    // NTF initial status - Disabled
    req->ffc1_ntf_cfg           = PRF_CLI_STOP_NTFIND;
    req->ffc2_ntf_cfg           = PRF_CLI_STOP_NTFIND;


    // Send the message
    ke_msg_send(req);
}

static int oads_enable_rsp_handler(ke_msg_id_t const msgid,
                                   struct oads_enable_rsp const *param,
                                   ke_task_id_t const dest_id,
                                   ke_task_id_t const src_id)
{
    return (KE_MSG_CONSUMED);
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
static int app_oads_msg_dflt_handler(ke_msg_id_t const msgid,
                                     void const *param,
                                     ke_task_id_t const dest_id,
                                     ke_task_id_t const src_id)
{
    // Drop the message
    MESH_APP_PRINT_INFO("%s\n", __func__);
    MESH_APP_PRINT_INFO("msgid = 0x%04x,destid = 0x%x,srcid = 0x%x\n", msgid, dest_id, src_id);
    return (KE_MSG_CONSUMED);
}

static int app_ffc1_writer_req_handler(ke_msg_id_t const msgid,
                                       struct oads_ffc1_writer_ind *param,
                                       ke_task_id_t const dest_id,
                                       ke_task_id_t const src_id)
{
    // Drop the message

    MESH_APP_PRINT_INFO("%s\n", __func__);

    oadImgIdentifyWrite(0x0, param->length, param->data );

    return (KE_MSG_CONSUMED);
}

static int app_ffc2_writer_req_handler(ke_msg_id_t const msgid,
                                       struct oads_ffc2_writer_ind *param,
                                       ke_task_id_t const dest_id,
                                       ke_task_id_t const src_id)
{
    // Drop the message

    MESH_APP_PRINT_INFO("%s\r\n", __func__);

    return (KE_MSG_CONSUMED);
}


void app_ffc1_ntf_req(void)
{
    // Drop the message

    MESH_APP_PRINT_INFO("%s\r\n", __func__);

    struct oads_ffc1_upd_req * req = KE_MSG_ALLOC(OADS_FFC1_UPD_REQ,
                                     prf_get_task_from_id(TASK_ID_OADS),
                                     TASK_APP,
                                     oads_ffc1_upd_req);
    // Fill in the parameter structure

    req->length = 20;
    req->data[0]   = 100;

    // Send the message
    ke_msg_send(req);

}

void app_ffc2_ntf_req(void)
{
    // Drop the message

    MESH_APP_PRINT_INFO("%s\r\n", __func__);


    struct oads_ffc2_upd_req * req = KE_MSG_ALLOC(OADS_FFC2_UPD_REQ,
                                     prf_get_task_from_id(TASK_ID_OADS),
                                     TASK_APP,
                                     oads_ffc2_upd_req);

    // Fill in the parameter structure

    req->length = 20;
    req->data[0]   = 50;

    // Send the message
    ke_msg_send(req);

}

static int oads_ffc1_upd_rsp_handler(ke_msg_id_t const msgid,
                                     struct oads_ffc1_upd_rsp *param,
                                     ke_task_id_t const dest_id,
                                     ke_task_id_t const src_id)
{

    MESH_APP_PRINT_INFO("%s\n", __func__);
    if (param->status == GAP_ERR_NO_ERROR)
    {

    }

    return (KE_MSG_CONSUMED);
}

static int oads_ffc2_upd_rsp_handler(ke_msg_id_t const msgid,
                                     struct oads_ffc2_upd_rsp *param,
                                     ke_task_id_t const dest_id,
                                     ke_task_id_t const src_id)
{

    MESH_APP_PRINT_INFO("%s\n", __func__);
    if (param->status == GAP_ERR_NO_ERROR)
    {

    }

    return (KE_MSG_CONSUMED);
}

/*
 * LOCAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

/// Default State handlers definition
const struct ke_msg_handler app_oads_msg_handler_list[] =
{
    // Note: first message is latest message checked by kernel so default is put on top.
    {KE_MSG_DEFAULT_HANDLER,          (ke_msg_func_t)app_oads_msg_dflt_handler},

    {OADS_ENABLE_RSP,                   (ke_msg_func_t)oads_enable_rsp_handler},

    {OADS_FFC1_WRITER_REQ_IND,      (ke_msg_func_t)app_ffc1_writer_req_handler},

    {OADS_FFC1_UPD_RSP,                         (ke_msg_func_t)oads_ffc1_upd_rsp_handler},

    {OADS_FFC2_WRITER_REQ_IND,      (ke_msg_func_t)app_ffc2_writer_req_handler},

    {OADS_FFC2_UPD_RSP,                         (ke_msg_func_t)oads_ffc2_upd_rsp_handler},
};

const struct ke_state_handler app_oads_table_handler =
{&app_oads_msg_handler_list[0], (sizeof(app_oads_msg_handler_list)/sizeof(struct ke_msg_handler))};

