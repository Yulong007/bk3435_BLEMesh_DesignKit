/**
 ****************************************************************************************
 *
 * @file fff0s.h
 *
 * @brief Header file - FFF0 Service Server Role
 *
 * Copyright (C) beken 2009-2015
 *
 *
 ****************************************************************************************
 */
#ifndef _FCC0S_H_
#define _FCC0S_H_

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

#if (BLE_FCC0_SERVER)

#include "fcc0s_task.h"
#include "atts.h"
#include "prf_types.h"
#include "prf.h"

/*
 * DEFINES
 ****************************************************************************************
 */

#define FCC0S_CFG_FLAG_MANDATORY_MASK       (0x3F)
#define FCC0_CFG_FLAG_NTF_SUP_MASK          (0x08)
#define FCC0_CFG_FLAG_MTP_FFF1_MASK         (0x40)

#define FCC1_LVL_MAX                        (100)

#define FCC1_FLAG_NTF_CFG_BIT             (0x02)



enum
{
    ATT_USER_SERVER_FCC0                        = ATT_UUID_16(0xFCC0),

    ATT_USER_SERVER_CHAR_FCC1                   = ATT_UUID_16(0xFCC1),

    ATT_USER_SERVER_CHAR_FCC2                   = ATT_UUID_16(0xFCC2),

};

/// Battery Service Attributes Indexes
enum
{
    FCC0S_IDX_SVC,

    FCC0S_IDX_FCC2_LVL_CHAR,
    FCC0S_IDX_FCC2_LVL_VAL,

    FCC0S_IDX_FCC1_LVL_CHAR,
    FCC0S_IDX_FCC1_LVL_VAL,
    FCC0S_IDX_FCC1_LVL_NTF_CFG,
    FCC0S_IDX_FCC1_LVL_USER_DESC,

    FCC0S_IDX_NB,
};

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */


/// FFF0 'Profile' Server environment variable
struct fcc0s_env_tag
{
    /// profile environment
    prf_env_t prf_env;

    /// On-going operation
    struct ke_msg *operation;
    /// FFF0 Services Start Handle
    uint16_t start_hdl;
    /// Level of the FFF1
    uint8_t fcc1_lvl[FCC0_FCC1_DATA_LEN];

    uint8_t fcc2_value[FCC0_FCC2_DATA_LEN];
    /// BASS task state
    ke_state_t state[FCC0S_IDX_MAX];
    /// Notification configuration of peer devices.
    uint8_t ntf_cfg[BLE_CONNECTION_MAX];
    /// Database features
    uint8_t features;

};



/**
 ****************************************************************************************
 * @brief Retrieve fff0 service profile interface
 *
 * @return fff0 service profile interface
 ****************************************************************************************
 */
const struct prf_task_cbs *fcc0s_prf_itf_get(void);

uint16_t fcc0s_get_att_handle(uint8_t att_idx);

uint8_t  fcc0s_get_att_idx(uint16_t handle, uint8_t *att_idx);

void fcc0s_notify_fcc1_lvl(struct fcc0s_env_tag *fcc0s_env, struct fcc0s_fcc1_level_upd_req const *param);

#endif /* #if (BLE_FFF0_SERVER) */



#endif /*  _FFF0_H_ */



