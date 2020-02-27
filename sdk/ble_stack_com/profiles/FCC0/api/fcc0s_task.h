/**
 ****************************************************************************************
 *
 * @file ffC0s_task.h
 *
 * @brief Header file - Battery Service Server Role Task.
 *
 * Copyright (C) RivieraWaves 2009-2015
 *
 *
 ****************************************************************************************
 */


#ifndef _FCC0S_TASK_H_
#define _FCC0S_TASK_H_


#include "rwprf_config.h"
#if (BLE_FCC0_SERVER)
#include <stdint.h>
#include "rwip_task.h" // Task definitions
/*
 * DEFINES
 ****************************************************************************************
 */

///Maximum number of FFF0 Server task instances
#define FCC0S_IDX_MAX     0x01
///Maximal number of FFF0 that can be added in the DB

#define  FCC0_FCC1_DATA_LEN  128
#define  FCC0_FCC2_DATA_LEN  128
/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

/// Possible states of the FFF0S task
enum fcc0s_state
{
    /// Idle state
    FCC0S_IDLE,
    /// busy state
    FCC0S_BUSY,
    /// Number of defined states.
    FCC0S_STATE_MAX
};

/// Messages for FFF0 Server
enum fcc0s_msg_id
{
    /// Start the FFF0 Server - at connection used to restore bond data
    FCC0S_CREATE_DB_REQ   = TASK_FIRST_MSG(TASK_ID_FCC0S),

    /// FFF1 Level Value Update Request
    FCC0S_FCC1_LEVEL_UPD_REQ,
    /// Inform APP if FFF1 Level value has been notified or not
    FCC0S_FCC1_LEVEL_UPD_RSP,
    /// Inform APP that FFF1 Level Notification Configuration has been changed - use to update bond data
    FCC0S_FCC1_LEVEL_NTF_CFG_IND,

    FCC0S_FCC2_WRITER_REQ_IND,

    FCC0S_FCC1_LEVEL_PERIOD_NTF


};

/// Features Flag Masks
enum fcc0s_features
{
    /// FFF1 Level Characteristic doesn't support notifications
    FCC0_FCC1_LVL_NTF_NOT_SUP,
    /// FFF1 Level Characteristic support notifications
    FCC0_FCC1_LVL_NTF_SUP,
};
/*
 * APIs Structures
 ****************************************************************************************
 */

/// Parameters for the database creation
struct fcc0s_db_cfg
{
    /// Number of FFF0 to add
    uint8_t fcc0_nb;
    /// Features of each FFF0 instance
    uint8_t features;
};

/// Parameters of the @ref FFF0S_ENABLE_REQ message
struct fcc0s_enable_req
{
    /// connection index
    uint8_t  conidx;
    /// Notification Configuration
    uint8_t  ntf_cfg;
    /// Old FFF1 Level used to decide if notification should be triggered
    uint8_t  old_fcc1_lvl;
};


///Parameters of the @ref FFF0S_BATT_LEVEL_UPD_REQ message
struct fcc0s_fcc1_level_upd_req
{
    /// BAS instance
    uint8_t conidx;

    uint8_t length;
    /// fff1 Level
    uint8_t fcc1_level[FCC0_FCC1_DATA_LEN];
};

///Parameters of the @ref FFF0S_FFF1_LEVEL_UPD_RSP message
struct fcc0s_fcc1_level_upd_rsp
{
    ///status
    uint8_t status;
};

///Parameters of the @ref BASS_BATT_LEVEL_NTF_CFG_IND message
struct fcc0s_fcc1_level_ntf_cfg_ind
{
    /// connection index
    uint8_t  conidx;
    ///Notification Configuration
    uint8_t  ntf_cfg;
};


/// Parameters of the @ref FFF0S_FFF2_WRITER_REQ_IND message
struct fcc0s_fcc2_writer_ind
{
    /// Alert level
    uint8_t fcc2_value[FCC0_FCC2_DATA_LEN];

    uint8_t length;
    /// Connection index
    uint8_t conidx;
};


/*
 * TASK DESCRIPTOR DECLARATIONS
 ****************************************************************************************
 */

extern const struct ke_state_handler fcc0s_default_handler;
#endif // BLE_FFF0_SERVER


#endif /* _FFF0S_TASK_H_ */

