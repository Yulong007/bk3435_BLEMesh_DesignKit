/**
 ****************************************************************************************
 *
 * @file app_mm_msg.h
 *
 * @brief mesh Application Module entry point
 *
 * @auth  gang.cheng
 *
 * @date  2019.03.31
 *
 * Copyright (C) Beken 2009-2020
 *
 *
 ****************************************************************************************
 */

#ifndef APP_MM_MSG_H_
#define APP_MM_MSG_H_
/**
 ****************************************************************************************
 * @addtogroup APP
 * @ingroup BEKEN
 *
 * @brief OADS Application Module entry point
 *
 * @{
 ****************************************************************************************
 */
/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"     // SW configuration


#include <stdint.h>          // Standard Integer Definition
#include "ke_task.h"         // Kernel Task Definition
#include "m_tb_state.h"
#include "mesh_tb_timer.h"
#include "nvds.h"
#include "app.h"
#include "pwm.h"




int app_models_msg_pro_handler(ke_msg_id_t const msgid,
                               void const *param,
                               ke_task_id_t const dest_id,
                               ke_task_id_t const src_id);

void app_mesh_add_models_server(void);

#endif // APP_MM_MSG_H_

