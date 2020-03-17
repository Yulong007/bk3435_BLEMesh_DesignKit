/**
 ****************************************************************************************
 *
 * @file ais.h
 *
 * @brief Header file - GMA Service Server Role
 *
 * Copyright (C) beken 2009-2015
 *
 *
 ****************************************************************************************
 */
#ifndef _AIS_H_
#define _AIS_H_

#include "rwip_config.h"
#include "rwprf_config.h"

#if (BLE_GMA_SERVER)
#include "ais_task.h"
#include "atts.h"
#include "prf_types.h"
#include "prf.h"

#define GMA_SERVICE_UUID                    0xFEB3
#define GMA_READ_UUID                       0xFED4
#define GMA_WRITE_UUID                      0xFED5
#define GMA_INDICATE_UUID                   0xFED6
#define GMA_WRITEWITHNORESP_UUID            0xFED7
#define GMA_NOTIFY_UUID                     0xFED8

enum
{		
    ATT_USER_SERVER_GMA 				   = ATT_UUID_16(GMA_SERVICE_UUID),

    ATT_USER_SERVER_CHAR_READ              = ATT_UUID_16(GMA_READ_UUID),

    ATT_USER_SERVER_CHAR_WRITE			   = ATT_UUID_16(GMA_WRITE_UUID),
    
    ATT_USER_SERVER_CHAR_INDICATE          = ATT_UUID_16(GMA_INDICATE_UUID),
    
    ATT_USER_SERVER_CHAR_WRITEWITHNORESP   = ATT_UUID_16(GMA_WRITEWITHNORESP_UUID),
    
    ATT_USER_SERVER_CHAR_NOTIFY            = ATT_UUID_16(GMA_NOTIFY_UUID),
};

/// Alibaba GMA Service Attributes Indexes
enum
{
	GMAS_IDX_SVC,

	GMAS_IDX_READ_CHAR,
	GMAS_IDX_READ_VAL,
	
    GMAS_IDX_WRITE_CHAR,
    GMAS_IDX_WRITE_VAL,

	GMAS_IDX_INDICATE_CHAR,
	GMAS_IDX_INDICATE_VAL,
	GMAS_IDX_INDICATE_CFG,
	
    GMAS_IDX_WRITEWITHNORESP_CHAR,
    GMAS_IDX_WRITEWITHNORESP_VAL,
    
    GMAS_IDX_NOTIFY_CHAR,
    GMAS_IDX_NOTIFY_VAL,
    GMAS_IDX_NOTIFY_CFG,

	GMAS_IDX_NB,
};

/// gma 'Profile' Server environment variable
struct gmas_env_tag
{
    /// profile environment
    prf_env_t prf_env;
    /// On-going operation
    struct ke_msg * operation;
    /// FFF0 Services Start Handle
    uint16_t start_hdl;
    /// gma task state
    ke_state_t state[GMAS_IDX_MAX];
	/// Notification configuration of peer devices.
	uint16_t tx_ntf_cfg[BLE_CONNECTION_MAX];
    /// connection index
    uint8_t conidx;
};

const struct prf_task_cbs* gmas_prf_itf_get(void);

uint16_t gmas_get_att_handle(uint8_t att_idx);

uint8_t gmas_get_att_idx(uint16_t handle, uint8_t *att_idx);

void gmas_data_send(struct gmas_env_tag* gmas_env, struct gma_tx_ind const *param);

#endif /* #if (BLE_GMA_SERVER) */
#endif /*  _AIS_H_ */
