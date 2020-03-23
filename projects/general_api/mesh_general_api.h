/**
 ****************************************************************************************
 *
 * @file mesh_general_api.h
 *
 * @brief mesh Application general api Module entry point
 *
 * @auth  gang.cheng
 *
 * @date  2019.11.28
 *
 * Copyright (C) Beken 2009-2020
 *
 *
 ****************************************************************************************
 */

#ifndef MESH_GENERAL_API_H_
#define MESH_GENERAL_API_H_
/**
 ****************************************************************************************
 * @addtogroup APP
 * @ingroup BEKEN
 *
 * @brief Mesh Application Module entry point
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
#include "Mesh_api.h"


#define SYS_RESET_BY_POWER_ON    0
#define SYS_RESET_BY_WDT         1

#define UART_CMD_PROV_EN         0

#define LED_SHOW_P1_ENABLE     1

#define POWER_3P5_ENABLE         0  

typedef uint8_t sys_reset_src_t;
 
void app_mesh_enable(void);
void app_mesh_disable(void);
void app_store_mesh_info(void);


uint8_t app_relay_user_adv(uint16_t interval, uint8_t nb_tx, uint8_t data_len, const uint8_t *data);
sys_reset_src_t sys_check_reset_src(void);



#if (UART_CMD_PROV_EN)
void app_test_add_key(void);
#endif /* UART_CMD_PROV_EN */

#endif // MESH_GENERAL_API_H_




