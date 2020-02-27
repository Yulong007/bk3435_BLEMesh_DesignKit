/**
 ****************************************************************************************
 *
 * @file wechat.h
 *
 * @brief Header file - WCT Service Server Role
 *
 * Copyright (C) beken 2009-2015
 *
 *
 ****************************************************************************************
 */
#ifndef _WECHAT_H_
#define _WECHAT_H_

/**
 ****************************************************************************************
 * @addtogroup  FFF0 'Profile' Server
 * @ingroup FFF0
 * @brief FFF0 'Profile' Server
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"
#include "rwprf_config.h"

#if (BLE_WECHAT_SERVER)

#include "wechat_task.h"
#include "atts.h"
#include "prf_types.h"
#include "prf.h"

/*
 * DEFINES
 ****************************************************************************************
 */

#define WECHAT_CFG_FLAG_MANDATORY_MASK        (0xFF)
#define WECHAT_CFG_FLAG_NTF_SUP_MASK          (0x08)



enum
{
    ATT_WECHAT_SERVICE                      = ATT_UUID_16(0xFEE7),
    ATT_WECHAT_WRITE_CHAR                   = ATT_UUID_16(0xFEC7),
    ATT_WECHAT_INDICATE_CHAR                = ATT_UUID_16(0xFEC8),
    ATT_WECHAT_READ_CHAR                    = ATT_UUID_16(0xFEC9),
};

/// wechat Service Attributes Indexes
enum
{
    WECHAT_IDX_SVC,
    WECHAT_IDX_WRITE_CHAR,
    WECHAT_IDX_WRITE_VAL,
    WECHAT_IDX_READ_CHAR,
    WECHAT_IDX_READ_VAL,
    WECHAT_IDX_INDICATE_CHAR,
    WECHAT_IDX_INDICATE_VAL,
    WECHAT_IDX_INDICATE_CFG,
    WECHAT_IDX_NB,
};

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */


/// wechat 'Profile' Server environment variable
struct wechat_env_tag
{
    /// profile environment
    prf_env_t prf_env;

    /// On-going operation
    struct ke_msg *operation;
    /// WCTS Services Start Handle
    uint16_t start_hdl;
    //indication data len
    uint16_t indi_len;
    /// Level of the write
    uint8_t write_buffer[WECAHT_WRITE_DATA_LEN];
    //Level of the indicate
    uint8_t indi_buffer[WECAHT_INDI_DATA_LEN];
    //Level of the read
    uint8_t read_buffer[WECAHT_READ_DATA_LEN];
    /// BASS task state
    ke_state_t state[WECAHT_IDX_MAX];
    /// Notification configuration of peer devices.
    uint8_t indi_cfg[BLE_CONNECTION_MAX];
    /// Database features
    uint8_t features;

};



/**
 ****************************************************************************************
 * @brief Retrieve wechat service profile interface
 *
 * @return wechat service profile interface
 ****************************************************************************************
 */
const struct prf_task_cbs *wechat_prf_itf_get(void);

uint16_t wechat_get_att_handle(uint8_t att_idx);

uint8_t  wechat_get_att_idx(uint16_t handle, uint8_t *att_idx);

void wechat_indicate_lvl(struct wechat_env_tag *wechat_env, struct wechat_ind_upd_req const *param);

#endif /* #if (BLE_WECHAT_SERVER) */



#endif /*  _WECHAT_H_ */








































