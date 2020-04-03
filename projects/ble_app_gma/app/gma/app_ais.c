/**
 ****************************************************************************************
 *
 * @file app_fff0.c
 *
 * @brief fff0 Application Module entry point
 *
 * @auth  gang.cheng
 *
 * @date  2016.05.31
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

#include <string.h>
#include "app_ais.h"              // Battery Application Module Definitions
#include "app.h"                    // Application Definitions
#include "app_task.h"             // application task definitions
#include "ais_task.h"           // health thermometer functions
#include "co_bt.h"
#include "prf_types.h"             // Profile common types definition
#include "arch.h"                    // Platform Definitions
#include "prf.h"
#include "ais.h"
#include "ke_timer.h"
#include "uart.h"
#include "gma.h"

void app_gma_add_gmas(void)
{
	struct gapm_profile_task_add_cmd *req = KE_MSG_ALLOC_DYN(GAPM_PROFILE_TASK_ADD_CMD,
                	                                         TASK_GAPM, TASK_APP,
                	                                         gapm_profile_task_add_cmd, 
                	                                         sizeof(struct gmas_db_cfg));
	// Fill message
	req->operation = GAPM_PROFILE_TASK_ADD;
	req->sec_lvl =   0;
	req->prf_task_id = TASK_ID_GMAS;
	req->app_task = TASK_APP;
	req->start_hdl = 0; //req->start_hdl = 0; dynamically allocated

	// Send the message
	ke_msg_send(req);
}

static int app_gma_msg_dflt_handler(ke_msg_id_t const msgid,
                                     void const *param,
                                     ke_task_id_t const dest_id,
                                     ke_task_id_t const src_id)
{
	// Drop the message
	return (KE_MSG_CONSUMED);
}

static int app_gma_data_receive_handler(ke_msg_id_t const msgid,
                                          struct gma_rx_ind *param,
                                          ke_task_id_t const dest_id,
                                          ke_task_id_t const src_id)
{    
    gma_recv_decode(&param->data[0], param->length);

	return (KE_MSG_CONSUMED);
}

void app_gma_data_send(uint8_t* buf, uint8_t len, uint8_t operation)
{
	struct gma_tx_ind *req = KE_MSG_ALLOC(GMA_ID_TX,
	                                      prf_get_task_from_id(TASK_ID_GMAS),
	                                      TASK_APP,
	                                      gma_tx_ind);
	// Fill in the parameter structure
	req->length = len;
    req->operation = operation;
	memcpy(req->data, buf, len);

	// Send the message
	ke_msg_send(req);
}

/// Default State handlers definition
const struct ke_msg_handler app_gmas_msg_handler_list[] =
{
	// Note: first message is latest message checked by kernel so default is put on top.
	{KE_MSG_DEFAULT_HANDLER,     (ke_msg_func_t)app_gma_msg_dflt_handler},
	{GMA_ID_RX,                  (ke_msg_func_t)app_gma_data_receive_handler},
};

const struct ke_state_handler app_gmas_table_handler =
                              {&app_gmas_msg_handler_list[0], (sizeof(app_gmas_msg_handler_list)/sizeof(struct ke_msg_handler))};
