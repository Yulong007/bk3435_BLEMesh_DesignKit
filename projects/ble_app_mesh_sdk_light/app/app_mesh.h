/**
 ****************************************************************************************
 *
 * @file app_mesh.h
 *
 * @brief mesh Application Module entry point
 *
 * @auth  gang.cheng
 *
 * @date  2018.07.9
 *
 * Copyright (C) Beken 2009-2016
 *
 *
 ****************************************************************************************
 */

#ifndef APP_MESH_H_
#define APP_MESH_H_
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

/*
 *  DEFINITION
 ****************************************************************************************
 */

#define ALI_MESH 1

#define TEST_MESH_OTA  0

#define MESH_UNPROV_ADV_TIME  (10 * 60 * 1000)
/*
 * STRUCTURES DEFINITION
 ****************************************************************************************
 */

/// mesh Application Module Environment Structure
typedef struct app_mesh_env_tag
{
    /// Connection handle
    uint8_t conidx;

    mesh_tb_timer_t               timer_upd;
} app_mesh_env_tag_t;
/*
 * GLOBAL VARIABLES DECLARATIONS
 ****************************************************************************************
 */

/// mesh Application environment
extern struct app_mesh_env_tag app_mesh_env;

/// Table of message handlers
extern const struct ke_state_handler app_mesh_table_handler;
/*
 * FUNCTIONS DECLARATION
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 *
 * braces Application Functions
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Initialize braces Application Module
 ****************************************************************************************
 */
void app_mesh_init(void);
/**
 ****************************************************************************************
 * @brief Add a mesh  instance in the DB
 ****************************************************************************************
 */
void app_mesh_add_mesh(void);
/**
 ****************************************************************************************
 * @brief Enable the oad Service
 ****************************************************************************************
 */


void app_check_power_reset_state(void);


void app_unprov_adv_timeout_set(uint32_t timer);
/**
 ****************************************************************************************
 * @brief Send a step_info
 ****************************************************************************************
 */

/*********************************************************************
* LOCAL FUNCTIONS
*/

#endif // APP_MESH_H_

