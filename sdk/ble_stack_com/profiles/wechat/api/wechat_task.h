/**
 ****************************************************************************************
 *
 * @file wechat_task.h
 *
 * @brief Header file - Battery Service Server Role Task.
 *
 * Copyright (C) RivieraWaves 2009-2015
 *
 *
 ****************************************************************************************
 */


#ifndef _WECHAT_TASK_H_
#define _WECHAT_TASK_H_


#include "rwprf_config.h"
#if (BLE_WECHAT_SERVER)
#include <stdint.h>
#include "rwip_task.h" // Task definitions
/*
 * DEFINES
 ****************************************************************************************
 */

///Maximum number of WECHAT Server task instances
#define   WECAHT_IDX_MAX       0x01
///Maximal number of WECHAT that can be added in the DB

#define  WECAHT_WRITE_DATA_LEN  20
#define  WECAHT_READ_DATA_LEN   20
#define  WECAHT_INDI_DATA_LEN   20
/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

/// Possible states of the WECHAT task
enum wechat_state
{
    /// Idle state
    WECHAT_IDLE,
    /// busy state
    WECHAT_BUSY,
    /// Number of defined states.
    WECHAT_STATE_MAX
};

/// Messages for WECHAT Server
enum wechat_msg_id
{
    /// Start the WECHAT Server - at connection used to restore bond data
    WECHAT_CREATE_DB_REQ   = TASK_FIRST_MSG(TASK_ID_WECHAT),

    /// WECHAT Level Value Update Request
    WECHAT_INDICATE_UPD_REQ,
    /// Inform APP if WECHAT Level value has been notified or not
    WECHAT_INDICATE_UPD_RSP,
    /// Inform APP that WECHAT Level Notification Configuration has been changed - use to update bond data
    WECHAT_INDICATE_IND_CFG_IND,

    WECHAT_WRITER_REQ_IND,

    WECHAT_GATTC_CMP_EVT,
};

/// Features Flag Masks
enum wechat_features
{
    /// FFF1 Level Characteristic doesn't support notifications
    WECHAT_INDICATE_NOT_SUP,
    /// FFF1 Level Characteristic support notifications
    WECHAT_INDICATE_SUP,
};
/*
 * APIs Structures
 ****************************************************************************************
 */

/// Parameters for the database creation
struct wechat_db_cfg
{
    /// Number of WECHAT to add
    uint8_t wechat_nb;
    /// Features of each WECHAT instance
    uint8_t features;
};

/// Parameters of the @ref FFF0S_ENABLE_REQ message
struct wechat_enable_req
{
    /// connection index
    uint8_t  conidx;
    /// Notification Configuration
    uint8_t  inti_cfg;
};


/// Parameters of the @ref FFF0S_ENABLE_RSP message
struct wechat_enable_rsp
{
    /// connection index
    uint8_t conidx;
    ///status
    uint8_t status;
};

///Parameters of the @ref FFF0S_BATT_LEVEL_UPD_REQ message
struct wechat_ind_upd_req
{
    /// BAS instance
    uint8_t conidx;

    uint8_t length;
    /// fff1 Level
    uint8_t indicate_buf[WECAHT_INDI_DATA_LEN];
};

///Parameters of the @ref FFF0S_FFF1_LEVEL_UPD_RSP message
struct wechat_ind_upd_rsp
{
    ///status
    uint8_t status;
};

///Parameters of the @ref BASS_BATT_LEVEL_NTF_CFG_IND message
struct wechat_indi_cfg_ind
{
    /// connection index
    uint8_t  conidx;
    ///Notification Configuration
    uint8_t  indi_cfg;
};


/// Parameters of the @ref FFF0S_FFF2_WRITER_REQ_IND message
struct wechat_write_ind
{
    /// Alert level
    uint8_t write_buf[WECAHT_WRITE_DATA_LEN];

    uint8_t length;
    /// Connection index
    uint8_t conidx;
};


/*
 * TASK DESCRIPTOR DECLARATIONS
 ****************************************************************************************
 */

extern const struct ke_state_handler wechat_default_handler;
#endif // BLE_WECHAT_SERVER


#endif /* _FFF0S_TASK_H_ */

