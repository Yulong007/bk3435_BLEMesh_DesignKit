/**
 ****************************************************************************************
 *
 * @file mesh_param_int.c
 *
 * @brief Mesh Stack param Interface
 *
 * Copyright (C) Beken 2017-2019
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "mesh_param_int.h"
#include "m_bcn.h"
#include "m_config.h"
#include <string.h>
#include "m_bcn.h"
#include "m_config.h"


mesh_stack_param_int_t m_stack_param;

void mesh_stack_param_init(void)
{
    /* Note: When initializing structure parameter values, we should used the Macro instead of the constant.
      *  That's beacause the meaning of the constant is not clear and it is hard to know where to config the value.
      */
    memset(&m_stack_param, 0, sizeof(m_stack_param));
    m_stack_param.m_bcn_default_unprov_bcn_intv_ms = M_BCN_DEFAULT_UNPROV_BCN_INTV_MS;

    m_stack_param.m_ttl_default = M_TTL_DEFAULT;

    m_stack_param.m_adv_nb_tx = M_ADV_NB_TX;
    m_stack_param.m_adv_nb_net_tx = M_ADV_NB_TX;
    m_stack_param.m_adv_interval = M_ADV_INTERVAL;
    m_stack_param.m_adv_net_interval = M_ADV_NET_INTERVAL;
    m_stack_param.m_adv_con_interval = M_ADV_CON_INTERVAL;

    m_stack_param.m_adv_scan_interval = M_ADV_SCAN_INTERVAL;

    m_stack_param.m_proxy_con_adv_update_dur = M_PROXY_CON_ADV_UPDATE_DUR;
}

void mesh_stack_param_unprov_bcn_intv_ms_set(uint32_t ms)
{
    m_stack_param.m_bcn_default_unprov_bcn_intv_ms = ms;
}

