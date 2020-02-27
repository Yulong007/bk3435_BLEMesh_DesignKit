/**
 ****************************************************************************************
 *
 * @file mm_api.c
 *
 * @brief Mesh Model Application Programming Interface
 *
 * Copyright (C) RivieraWaves 2017-2019
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup MM_API
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "mm_inc.h"             // Mesh Model Includes
#include "mm_dbg.h"             // Mesh Model Debug Module Definitions
#include "mm_route.h"           // Mesh Model Routing Manager Definitions
#include "mm_tb.h"              // Mesh Model Tool Boxes Definitions
#include "mm_api.h"
#include "uart.h"
/*
 * LOCAL VARIABLES
 ****************************************************************************************
 */

/// Callback used to discuss with a native application
const mm_api_cb_t *p_mm_api_cb;

/*
 * EXTERNAL FUNCTIONS DECLARATION
 ****************************************************************************************
 */

#if (BLE_MESH_MSG_API)
extern uint16_t mm_api_msg_init(bool reset, void *p_env, const mm_cfg_t *p_cfg);
extern uint16_t mm_api_msg_get_env_size(const mm_cfg_t *p_cfg);
extern uint8_t mm_api_msg_handler(uint16_t msg_id, uint16_t src_id, const void *p_param);
#endif //(BLE_MESH_MSG_API)

/*
 * COMMON API FUNCTIONS
 ****************************************************************************************
 */

uint16_t mm_api_set(const mm_api_cb_t *p_cb_api)
{
    uint16_t status = MESH_ERR_INVALID_PARAM;

    // Sanity check to ensure that all callback function are configured
    if (p_cb_api
            && p_cb_api->cb_register_ind
#if (BLE_MESH_MDL_SERVER)
            && p_cb_api->cb_srv_state_upd_ind
            && p_cb_api->cb_srv_array_state_upd_ind
            && p_cb_api->cb_srv_state_req_ind
#if (BLE_MESH_MDL_GENS)
            && p_cb_api->cb_srv_locg_upd_ind
            && p_cb_api->cb_srv_locl_upd_ind
            && p_cb_api->cb_srv_prop_get_req_ind
            && p_cb_api->cb_srv_prop_set_req_ind
#endif //(BLE_MESH_MDL_GENS)
#endif //(BLE_MESH_MDL_SERVER)
#if (BLE_MESH_MDL_CLIENT)
            && p_cb_api->cb_cli_state_ind
#if (BLE_MESH_MDL_GENC)
            && p_cb_api->cb_cli_bat_ind
            && p_cb_api->cb_cli_locg_ind
            && p_cb_api->cb_cli_locl_ind
            && p_cb_api->cb_cli_prop_ind
            && p_cb_api->cb_cli_prop_list_ind
#endif //(BLE_MESH_MDL_GENC)
#endif //(BLE_MESH_MDL_CLIENT)
       )
    {
        p_mm_api_cb = p_cb_api;
        status = MESH_ERR_NO_ERROR;
    }
    else
    {
        ASSERT_ERR(0);
    }

    return (status);
}

#if (BLE_MESH_MDL_SERVER)
uint16_t mm_api_register_server(uint8_t elmt_idx, uint8_t mdl_cfg_idx, uint8_t info)
{
    // Status
    uint16_t status;
    // Check provided model configuration index
    do
    {
#if (BLE_MESH_MDL_GENS)
        if (mdl_cfg_idx <= MM_CFG_IDX_GENS_MAX)
        {
            status = mm_gens_register(elmt_idx, mdl_cfg_idx, info);
            break;
        }
#endif //(BLE_MESH_MDL_GENS)

#if (BLE_MESH_MDL_LIGHTS)
        if ((mdl_cfg_idx >= MM_CFG_IDX_LIGHTS_MIN)
                && (mdl_cfg_idx <= MM_CFG_IDX_LIGHTS_MAX))
        {
            status = mm_lights_register(elmt_idx, mdl_cfg_idx, info);
            break;
        }
#endif //(BLE_MESH_MDL_LIGHTS)

        status = MESH_ERR_MDL_INVALID_CFG;
    }
    while (0);

    return (status);
}

#if (BLE_MESH_MDL_GENS)
uint16_t mm_api_register_server_prop(uint8_t elmt_idx, uint8_t req_queue_len,
                                     uint8_t nb_prop_user, uint8_t nb_prop_admin,
                                     uint8_t nb_prop_manuf, uint8_t nb_prop_cli,
                                     const mm_prop_t *p_props)
{
#if  (BLE_MESH_MDL_GENS_PROP)
    return (mm_gens_prop_register(elmt_idx, req_queue_len, nb_prop_user, nb_prop_admin,
                                  nb_prop_manuf, nb_prop_cli, p_props));
#else
    return MESH_ERR_NO_ERROR;
#endif

}
#endif //(BLE_MESH_MDL_GENS)

uint16_t mm_api_srv_set(m_lid_t mdl_lid, uint16_t state_id, uint32_t state)
{
    // Status
    uint16_t status;
    MESH_APP_PRINT_INFO("mm_api_srv_set\r\n");
    do
    {
        // Get model environment
        mm_tb_state_mdl_env_t *p_mdl_env = mm_tb_state_get_env(mdl_lid);
        // List of server role specific callback functions
        mm_srv_cb_t *p_cb_srv;

        if (!p_mdl_env)
        {
            status = MESH_ERR_MDL_INVALID_MDL_LID;
            break;
        }

        // Check that model is well a server model
        if (GETF(p_mdl_env->info, MM_TB_STATE_MDL_INFO_ROLE) != MM_TB_STATE_ROLE_SRV)
        {
            status = MESH_ERR_MDL_INVALID_ROLE;
            break;
        }

        // Get list of server role specific callback functions
        p_cb_srv = p_mdl_env->cb.u.p_cb_srv;

        if (!p_cb_srv || !p_cb_srv->cb_set)
        {
            status = MESH_ERR_MDL_COMMAND_NOT_AVAILABLE;
            break;
        }

        // Call the get callback
        status = p_cb_srv->cb_set(p_mdl_env, state_id, state);
    }
    while (0);

    return (status);
}

#if (BLE_MESH_MDL_GENS)

#if (BLE_MESH_MDL_GENS_BAT)
void mm_api_srv_bat_cfm(uint16_t status, uint8_t elmt_idx, uint8_t bat_lvl, uint32_t time_charge,
                        uint32_t time_discharge, uint8_t flags)
{
    return (mm_gens_cfm_bat(status, elmt_idx, bat_lvl, time_charge, time_discharge, flags));
}
#endif //BLE_MESH_MDL_GENS_BAT

#if (BLE_MESH_MDL_GENS_LOC)
void mm_api_srv_locg_cfm(uint16_t status, uint8_t elmt_idx, int32_t latitude, int32_t longitude,
                         int16_t altitude)
{
    return (mm_gens_cfm_locg(status, elmt_idx, latitude, longitude, altitude));
}

void mm_api_srv_locl_cfm(uint16_t status, uint8_t elmt_idx, int16_t north, int16_t east,
                         int16_t altitude, uint8_t floor, uint16_t uncertainty)
{
    return (mm_gens_cfm_locl(status, elmt_idx, north, east, altitude, floor, uncertainty));
}
#endif //BLE_MESH_MDL_GENS_LOC

#if (BLE_MESH_MDL_GENS_PROP)
void mm_api_srv_prop_cfm(uint16_t status, uint8_t elmt_idx, uint8_t prop_type, uint16_t prop_id,
                         uint16_t length, const uint8_t *p_val)
{
    return (mm_gens_prop_cfm(status, elmt_idx, prop_type, prop_id, length, p_val));
}
#endif //(BLE_MESH_MDL_GENS_PROP)

#endif //(BLE_MESH_MDL_GENS)

void mm_api_grp_add_local(uint8_t elmt_idx, uint32_t mdl_id, uint8_t grp_lid)
{
    // Look for indicated model
    m_lid_t mdl_lid = mm_tb_state_get_lid(elmt_idx, mdl_id);

    if (mdl_lid != MESH_INVALID_LID)
    {
        switch (mdl_id)
        {
#if (BLE_MESH_MDL_GENS)
            case (MM_ID_GENS_OO):
            case (MM_ID_GENS_LVL):

            {
                mm_gens_add_to_grp(elmt_idx, mdl_id, grp_lid);
            } break;
#endif //(BLE_MESH_MDL_GENS)  
            default:
            {
            } break;
        }
    }
}
#endif //(BLE_MESH_MDL_SERVER)

#if (BLE_MESH_MDL_CLIENT)
uint16_t mm_api_register_client(uint16_t cmdl_idx)
{
    // Status
    uint16_t status;
    MESH_APP_PRINT_INFO("%s, cmdl_idx = %d\r\n", __func__, cmdl_idx);
    // Check client model index
    do
    {
#if (BLE_MESH_MDL_GENC)
        if (cmdl_idx <= MM_CMDL_IDX_GENC_MAX)
        {
            status = mm_genc_register(cmdl_idx);
            break;
        }
#endif //(BLE_MESH_MDL_GENC)

#if (BLE_MESH_MDL_LIGHTC)
        if ((cmdl_idx >= MM_CMDL_IDX_LIGHTC_MIN)
                && (cmdl_idx <= MM_CMDL_IDX_LIGHTC_MAX))
        {
            status = mm_lightc_register(cmdl_idx);
            break;
        }
#endif //(BLE_MESH_MDL_LIGHTC)

        status = MESH_ERR_MDL_INVALID_CFG;
    }
    while (0);

    return (status);
}

uint16_t mm_api_cli_get(m_lid_t mdl_lid, uint16_t dst, uint16_t get_info)
{
    // Status
    uint16_t status;

    do
    {
        // Get model environment
        mm_tb_state_mdl_env_t *p_mdl_env = mm_tb_state_get_env(mdl_lid);
        // List of client role specific callback functions
        mm_cli_cb_t *p_cb_cli;

        if (!p_mdl_env)
        {
            status = MESH_ERR_MDL_INVALID_MDL_LID;
            break;
        }

        // Check that model is well a client model
        if (GETF(p_mdl_env->info, MM_TB_STATE_MDL_INFO_ROLE) != MM_TB_STATE_ROLE_CLI)
        {
            status = MESH_ERR_MDL_INVALID_ROLE;
            break;
        }

        // Get list of client role specific callback functions
        p_cb_cli = p_mdl_env->cb.u.p_cb_cli;

        if (!p_cb_cli || !p_cb_cli->cb_get)
        {
            status = MESH_ERR_MDL_COMMAND_NOT_AVAILABLE;
            break;
        }

        // Call the get callback
        status = p_cb_cli->cb_get(p_mdl_env, dst, get_info);
    }
    while (0);

    return (status);
}

uint16_t mm_api_cli_set(m_lid_t mdl_lid, uint16_t dst, uint32_t state_1, uint32_t state_2,
                        uint16_t set_info)
{
    // Status
    uint16_t status;

    do
    {
        // Get model environment
        mm_tb_state_mdl_env_t *p_mdl_env = mm_tb_state_get_env(mdl_lid);
        // List of client role specific callback functions
        mm_cli_cb_t *p_cb_cli;

        if (!p_mdl_env)
        {
            status = MESH_ERR_MDL_INVALID_MDL_LID;
            break;
        }

        // Check that model is well a client model
        if (GETF(p_mdl_env->info, MM_TB_STATE_MDL_INFO_ROLE) != MM_TB_STATE_ROLE_CLI)
        {
            status = MESH_ERR_MDL_INVALID_ROLE;
            break;
        }

        // Get list of client role specific callback functions
        p_cb_cli = p_mdl_env->cb.u.p_cb_cli;

        if (!p_cb_cli || !p_cb_cli->cb_set)
        {
            status = MESH_ERR_MDL_COMMAND_NOT_AVAILABLE;
            break;
        }

        // Call the set callback
        status = p_cb_cli->cb_set(p_mdl_env, dst, state_1, state_2, set_info);
    }
    while (0);

    return (status);
}

#if (BLE_MESH_MDL_GENC)
uint16_t mm_api_cli_set_locg(m_lid_t mdl_lid, uint16_t dst, uint8_t set_info,
                             int32_t latitude, int32_t longitude, int16_t altitude)
{
    // Status
    uint16_t status;
    // Get model environment
    mm_tb_state_mdl_env_t *p_mdl_env = mm_tb_state_get_env(mdl_lid);

    if (p_mdl_env)
    {
        if (p_mdl_env->mdl_id == MM_ID_GENC_LOC)
        {
            status = mm_genc_loc_set_global(p_mdl_env, dst, set_info, latitude, longitude,
                                            altitude);
        }
        else
        {
            status = MESH_ERR_COMMAND_DISALLOWED;
        }
    }
    else
    {
        status = MESH_ERR_INVALID_PARAM;
    }

    return (status);
}

uint16_t mm_api_cli_set_locl(m_lid_t mdl_lid, uint16_t dst, uint8_t set_info,
                             int16_t north, int16_t east, int16_t altitude, uint8_t floor,
                             uint16_t uncertainty)
{
    // Status
    uint16_t status;
    // Get model environment
    mm_tb_state_mdl_env_t *p_mdl_env = mm_tb_state_get_env(mdl_lid);

    if (p_mdl_env)
    {
        if (p_mdl_env->mdl_id == MM_ID_GENC_LOC)
        {
            status = mm_genc_loc_set_local(p_mdl_env, dst, set_info, north, east, altitude,
                                           floor, uncertainty);
        }
        else
        {
            status = MESH_ERR_COMMAND_DISALLOWED;
        }
    }
    else
    {
        status = MESH_ERR_INVALID_PARAM;
    }

    return (status);
}

uint16_t mm_api_cli_get_prop(m_lid_t mdl_lid, uint16_t dst, uint8_t get_type, uint16_t prop_id)
{
    // Status
    uint16_t status;
    // Get model environment
    mm_tb_state_mdl_env_t *p_mdl_env = mm_tb_state_get_env(mdl_lid);

    if (p_mdl_env)
    {
        if (p_mdl_env->mdl_id == MM_ID_GENC_PROP)
        {
            status = mm_genc_prop_get(p_mdl_env, dst, get_type, prop_id);
        }
        else
        {
            status = MESH_ERR_COMMAND_DISALLOWED;
        }
    }
    else
    {
        status = MESH_ERR_INVALID_PARAM;
    }

    return (status);
}

uint16_t mm_api_cli_set_prop(m_lid_t mdl_lid, uint16_t dst, uint8_t set_info, uint16_t prop_id,
                             uint8_t user_access, uint16_t length, const uint8_t *p_val)
{
    // Status
    uint16_t status;
    // Get model environment
    mm_tb_state_mdl_env_t *p_mdl_env = mm_tb_state_get_env(mdl_lid);

    if (p_mdl_env)
    {
        if (p_mdl_env->mdl_id == MM_ID_GENC_PROP)
        {
            status = mm_genc_prop_set(p_mdl_env, dst, set_info, prop_id, user_access, length, p_val);
        }
        else
        {
            status = MESH_ERR_COMMAND_DISALLOWED;
        }
    }
    else
    {
        status = MESH_ERR_INVALID_PARAM;
    }

    return (status);
}
#endif //(BLE_MESH_MDL_GENC)

uint16_t mm_api_cli_transition(m_lid_t mdl_lid, uint16_t dst, uint32_t state_1, uint32_t state_2,
                               uint32_t trans_time_ms, uint16_t delay_ms, uint16_t trans_info)
{
    // Status
    uint16_t status;
    MESH_APP_PRINT_INFO("%s\r\n", __func__);
    do
    {
        // Get model environment
        mm_tb_state_mdl_env_t *p_mdl_env = mm_tb_state_get_env(mdl_lid);
        // List of client role specific callback functions
        mm_cli_cb_t *p_cb_cli;

        if (!p_mdl_env)
        {
            MESH_APP_PRINT_INFO("%s, MESH_ERR_MDL_INVALID_MDL_LID, mdl_lid = 0x%x\r\n", __func__, mdl_lid);
            status = MESH_ERR_MDL_INVALID_MDL_LID;
            break;
        }

        // Check that model is well a client model
        if (GETF(p_mdl_env->info, MM_TB_STATE_MDL_INFO_ROLE) != MM_TB_STATE_ROLE_CLI)
        {
            MESH_APP_PRINT_INFO("%s, MESH_ERR_MDL_INVALID_ROLE\r\n", __func__);
            status = MESH_ERR_MDL_INVALID_ROLE;
            break;
        }

        // Get list of client role specific callback functions
        p_cb_cli = p_mdl_env->cb.u.p_cb_cli;

        if (!p_cb_cli || !p_cb_cli->cb_transition)
        {
            MESH_APP_PRINT_INFO("%s, MESH_ERR_MDL_COMMAND_NOT_AVAILABLE\r\n", __func__);
            status = MESH_ERR_MDL_COMMAND_NOT_AVAILABLE;
            break;
        }

        // Call the set callback
        status = p_cb_cli->cb_transition(p_mdl_env, dst, state_1, state_2, trans_time_ms,
                                         delay_ms, trans_info);
    }
    while (0);

    return (status);
}
#endif //(BLE_MESH_MDL_CLIENT)

uint16_t mm_api_enable(void)
{
    return (mm_tb_state_enable());
}

/*
 * GLOBAL FUNCTIONS
 ****************************************************************************************
 */

uint16_t mm_api_init(bool reset, void *p_env, const mm_cfg_t *p_cfg)
{
    // Stack initialization
    uint8_t *p_env_cursor = (uint8_t *)p_env;

    p_env_cursor += CO_ALIGN4_HI(mm_tb_init(reset, (void *)p_env_cursor, p_cfg));
    p_env_cursor += CO_ALIGN4_HI(mm_route_init(reset, (void *)p_env_cursor, p_cfg));

#if (BLE_MESH_DBG)
    p_env_cursor += CO_ALIGN4_HI(mm_dbg_init(reset, (void *)p_env_cursor, p_cfg));
#endif //(BLE_MESH_DBG)

#if (BLE_MESH_MDL_GENS)
    p_env_cursor += CO_ALIGN4_HI(mm_gens_init(reset, (void *)p_env_cursor, p_cfg));
#endif //(BLE_MESH_MDL_GENS)
#if (BLE_MESH_MDL_GENC)
    p_env_cursor += CO_ALIGN4_HI(mm_genc_init(reset, (void *)p_env_cursor, p_cfg));
#endif //(BLE_MESH_MDL_GENC)
#if (BLE_MESH_MDL_LIGHTS)
    p_env_cursor += CO_ALIGN4_HI(mm_lights_init(reset, (void*)p_env_cursor, p_cfg));
#endif //(BLE_MESH_MDL_LIGHTS)
#if (BLE_MESH_MDL_LIGHTC)
    p_env_cursor += CO_ALIGN4_HI(mm_lightc_init(reset, (void*)p_env_cursor, p_cfg));
#endif //(BLE_MESH_MDL_LIGHTC)
#if (BLE_MESH_MDL_SENSS)
    p_env_cursor += CO_ALIGN4_HI(mm_senss_init(reset, (void *)p_env_cursor, p_cfg));
#endif //(BLE_MESH_MDL_SENSS)
#if (BLE_MESH_MDL_SENSC)
    p_env_cursor += CO_ALIGN4_HI(mm_sensc_init(reset, (void *)p_env_cursor, p_cfg));
#endif //(BLE_MESH_MDL_SENSC)
#if (BLE_MESH_MDL_TSCNS)
    p_env_cursor += CO_ALIGN4_HI(mm_tscns_init(reset, (void *)p_env_cursor, p_cfg));
#endif //(BLE_MESH_MDL_TSCNS)
#if (BLE_MESH_MDL_TSCNC)
    p_env_cursor += CO_ALIGN4_HI(mm_tscnc_init(reset, (void *)p_env_cursor, p_cfg));
#endif //(BLE_MESH_MDL_TSCNC)

#if (BLE_MESH_MSG_API)
    p_env_cursor += CO_ALIGN4_HI(mm_api_msg_init(reset, (void *)p_env_cursor, p_cfg));
#endif //(BLE_MESH_MSG_API)

    // Return size of the environment used
    return (((uint32_t) p_env_cursor) - ((uint32_t) p_env));
}

uint16_t mm_api_get_env_size(const mm_cfg_t *p_cfg)
{
    uint16_t total_env_size = 0;

    total_env_size += CO_ALIGN4_HI(mm_tb_get_env_size(p_cfg));
    total_env_size += CO_ALIGN4_HI(mm_route_get_env_size(p_cfg));

#if (BLE_MESH_DBG)
    total_env_size += CO_ALIGN4_HI(mm_dbg_get_env_size(p_cfg));
#endif //(BLE_MESH_DBG)

    // Retrieve size of environment variable required for following mesh layers
#if (BLE_MESH_MDL_GENS)
    total_env_size += CO_ALIGN4_HI(mm_gens_get_env_size(p_cfg));
#endif //(BLE_MESH_MDL_GENS)
#if (BLE_MESH_MDL_GENC)
    total_env_size += CO_ALIGN4_HI(mm_genc_get_env_size(p_cfg));
#endif //(BLE_MESH_MDL_GENC)
#if (BLE_MESH_MDL_LIGHTS)
    total_env_size += CO_ALIGN4_HI(mm_lights_get_env_size(p_cfg));
#endif //(BLE_MESH_MDL_LIGHTS)
#if (BLE_MESH_MDL_LIGHTC)
    total_env_size += CO_ALIGN4_HI(mm_lightc_get_env_size(p_cfg));
#endif //(BLE_MESH_MDL_LIGHTC)
#if (BLE_MESH_MDL_SENSS)
    total_env_size += CO_ALIGN4_HI(mm_senss_get_env_size(p_cfg));
#endif //(BLE_MESH_MDL_SENSS)
#if (BLE_MESH_MDL_SENSC)
    total_env_size += CO_ALIGN4_HI(mm_sensc_get_env_size(p_cfg));
#endif //(BLE_MESH_MDL_SENSC)
#if (BLE_MESH_MDL_TSCNS)
    total_env_size += CO_ALIGN4_HI(mm_tscns_get_env_size(p_cfg));
#endif //(BLE_MESH_MDL_TSCNS)
#if (BLE_MESH_MDL_TSCNC)
    total_env_size += CO_ALIGN4_HI(mm_tscnc_get_env_size(p_cfg));
#endif //(BLE_MESH_MDL_TSCNC)

#if (BLE_MESH_MSG_API)
    total_env_size += CO_ALIGN4_HI(mm_api_msg_get_env_size(p_cfg));
#endif //(BLE_MESH_MSG_API)

    // Return size of environment
    return (total_env_size);
}

uint8_t mm_api_handler(uint16_t msg_id, uint16_t src_id, const void *p_param)
{
    uint8_t msg_status = MAL_MSG_FREE;

    // Mesh Model API
    if (msg_id < MESH_MDL_DBG_FIRST)
    {
#if (BLE_MESH_MSG_API)
        msg_status = mm_api_msg_handler(msg_id, src_id, p_param);
#else //(BLE_MESH_MSG_API)
        ASSERT_WARN(0, msg_id, src_id);
#endif //(BLE_MESH_MSG_API)
    }
    // Mesh Model Debug API
    else
    {
#if (BLE_MESH_DBG)
        msg_status = mm_dbg_handler(msg_id, src_id, p_param);
#else // !(BLE_MESH_DBG)
        ASSERT_WARN(0, msg_id, src_id);
#endif // (BLE_MESH_DBG)
    }

    return (msg_status);
}

/*
 * INTERNAL FUNCTIONS
 ****************************************************************************************
 */

void mm_api_send_register_ind(uint32_t model_id, uint8_t elmt_idx, m_lid_t mdl_lid)
{
    // Call the callback function
    p_mm_api_cb->cb_register_ind(model_id, elmt_idx, mdl_lid);
}

#if (BLE_MESH_MDL_SERVER)
void mm_api_send_srv_state_upd_ind(uint16_t state_id, uint8_t elmt_idx, uint32_t state,
                                   uint32_t trans_time_ms)
{
    // Call the callback function
    p_mm_api_cb->cb_srv_state_upd_ind(state_id, elmt_idx, state, trans_time_ms);
}

void mm_api_send_srv_array_state_upd_ind(uint16_t state_id, uint8_t elmt_idx, uint8_t len, uint8_t * state,
        uint32_t trans_time_ms)
{
    // Call the callback function
    p_mm_api_cb->cb_srv_array_state_upd_ind(state_id, elmt_idx, len, state, trans_time_ms);
}

void mm_api_send_srv_state_req_ind(uint32_t req_ind_code, uint8_t elmt_idx)
{
    // Call the callback function
    p_mm_api_cb->cb_srv_state_req_ind(req_ind_code, elmt_idx);
}

#if (BLE_MESH_MDL_GENS)
void mm_api_send_srv_locg_upd_ind(uint8_t elmt_idx, int32_t latitude, int32_t longitude,
                                  int16_t altitude)
{
    // Call the callback function
    p_mm_api_cb->cb_srv_locg_upd_ind(elmt_idx, latitude, longitude, altitude);
}

void mm_api_send_srv_locl_upd_ind(uint8_t elmt_idx, int16_t north, int16_t east, int16_t altitude,
                                  uint8_t floor, uint16_t uncertainty)
{
    // Call the callback function
    p_mm_api_cb->cb_srv_locl_upd_ind(elmt_idx, north, east, altitude, floor, uncertainty);
}

void mm_api_send_srv_prop_get(uint8_t elmt_idx, uint8_t prop_type, uint16_t prop_id)
{
    // Call the callback function
    p_mm_api_cb->cb_srv_prop_get_req_ind(elmt_idx, prop_type, prop_id);
}

void mm_api_send_srv_prop_set(uint8_t elmt_idx, uint8_t prop_type, uint16_t prop_id, bool ack,
                              uint16_t length, uint8_t *p_val)
{
    // Call the callback function
    p_mm_api_cb->cb_srv_prop_set_req_ind(elmt_idx, prop_type, prop_id, ack, length, p_val);
}
#endif //(BLE_MESH_MDL_GENS)

#endif //(BLE_MESH_MDL_SERVER)

#if (BLE_MESH_MDL_CLIENT)
void mm_api_send_cli_state_ind(uint16_t src, uint16_t state_id, uint32_t state_1,
                               uint32_t state_2, uint32_t rem_time_ms)
{
    // Call the callback function
    p_mm_api_cb->cb_cli_state_ind(src, state_id, state_1, state_2, rem_time_ms);
}

void mm_api_send_cli_bat_ind(uint16_t src, uint8_t bat_lvl, uint32_t time_discharge,
                             uint32_t time_charge, uint8_t flags)
{
    // Call the callback function
    p_mm_api_cb->cb_cli_bat_ind(src, bat_lvl, time_discharge, time_charge, flags);
}

void mm_api_send_cli_locg_ind(uint16_t src, int32_t latitude, int32_t longitude, int16_t altitude)
{
    // Call the callback function
    p_mm_api_cb->cb_cli_locg_ind(src, latitude, longitude, altitude);
}

void mm_api_send_cli_locl_ind(uint16_t src, int16_t north, int16_t east, int16_t altitude,
                              uint8_t floor, uint16_t uncertainty)
{
    // Call the callback function
    p_mm_api_cb->cb_cli_locl_ind(src, north, east, altitude, floor, uncertainty);
}

void mm_api_send_cli_prop_ind(uint16_t src, uint16_t prop_id, uint8_t user_access, uint16_t length,
                              uint8_t *p_val)
{
    // Call the callback function
    p_mm_api_cb->cb_cli_prop_ind(src, prop_id, user_access, length, p_val);
}

void mm_api_send_cli_prop_list_ind(uint16_t src, uint8_t prop_type, uint16_t nb_prop,
                                   uint16_t *p_prop_ids)
{
    // Call the callback function
    p_mm_api_cb->cb_cli_prop_list_ind(src, prop_type, nb_prop, p_prop_ids);
}
#endif //(BLE_MESH_MDL_CLIENT)

/// @} end of group
