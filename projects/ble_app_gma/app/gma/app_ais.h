/**
 ****************************************************************************************
 *
 * @file app_fff0.c
 *
 * @brief findt Application Module entry point
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
#ifndef APP_AIS_H_
#define APP_AIS_H_
/**
 ****************************************************************************************
 * @addtogroup APP
 * @ingroup RICOW
 *
 * @brief fff0 Application Module entry point
 *
 * @{
 ****************************************************************************************
 */
#include "rwip_config.h"     // SW configuration
#include <stdint.h>          // Standard Integer Definition
#include "ke_task.h"         // Kernel Task Definition

/// Table of message handlers
extern const struct ke_state_handler app_gmas_table_handler;

void app_gma_add_gmas(void);

void app_gma_enable_prf(uint8_t conidx);

void app_gma_data_send(uint8_t* buf, uint8_t len, uint8_t operation);

#endif // APP_BATT_H_
