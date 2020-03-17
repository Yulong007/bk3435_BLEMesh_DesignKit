/**
 ****************************************************************************************
 *
 * @file fff0s_task.h
 *
 * @brief Header file - Battery Service Server Role Task.
 *
 * Copyright (C) RivieraWaves 2009-2015
 *
 *
 ****************************************************************************************
 */
#ifndef _AIS_TASK_H_
#define _AIS_TASK_H_
#include "rwprf_config.h"
#if (BLE_GMA_SERVER)
#include <stdint.h>
#include "rwip_task.h" // Task definitions

///Maximum number of FFF0 Server task instances
#define  GMAS_IDX_MAX                 0x01

#define  GMA_READ_DATA_LEN            50
#define  GMA_WRITE_DATA_LEN           50
#define  GMA_INDICATE_DATA_LEN        50
#define  GMA_WRITEWITHNORESP_DATA_LEN 200
#define  GMA_NOTIFY_DATA_LEN          50

/// Possible states of the FFF0S task
enum gmas_state
{
    /// Idle state
    GMAS_IDLE,
    /// busy state
    GMAS_BUSY,
    /// Number of defined states.
    GMAS_STATE_MAX
};

/// Messages for FFF0 Server
enum gmas_msg_id
{
    /// Start the FFF0 Server - at connection used to restore bond data
	GMAS_CREATE_DB_REQ   = TASK_FIRST_MSG(TASK_ID_GMAS),
	
    GMA_ID_RX,
    
    GMA_ID_TX,
};

/// Features Flag Masks
enum gmas_features
{
    /// FFF1 Level Characteristic doesn't support notifications
    GMA_NTF_NOT_SUP,
    /// FFF1 Level Characteristic support notifications
    GMA_NTF_SUP,

    GMA_IND_SUP,
};

/// Parameters for the database creation
struct gmas_db_cfg
{
    /// Number of FFF0 to add
    uint8_t gma_nb;
    /// Features of each FFF0 instance
    uint8_t features;
};

/// Parameters of the @ref FFF0S_FFF2_WRITER_REQ_IND message
struct gma_rx_ind
{
    uint8_t data[GMA_WRITEWITHNORESP_DATA_LEN];
	
	uint8_t length;
};

/// Parameters of the @ref FFF0S_FFF2_WRITER_REQ_IND message
struct gma_tx_ind
{
    uint8_t data[GMA_NOTIFY_DATA_LEN];
	
	uint8_t length;
    /// Request type (notification / indication)
    uint8_t operation; 
};

extern const struct ke_state_handler gmas_default_handler;
#endif // BLE_FFF0_SERVER
#endif /* _AIS_TASK_H_ */
