/**
 ****************************************************************************************
 * @file mm_gens_loc.c
 *
 * @brief Mesh Model Generic Location Server Module
 *
 * Copyright (C) RivieraWaves 2017-2019
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup MM_GENS_LOC
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "mm_gens_int.h"      // Mesh Model Generic Server Module Internal Definitions

#if (BLE_MESH_MDL_GENS_LOC)
/*
 * ENUMERATIONS
 ****************************************************************************************
 */

/// Information bit field
/// 8               4              3               2           1            0
/// +---------------+--------------+---------------+-----------+------------+
/// |       RFU     | Wait App Loc | Wait App Glob | Publi Loc | Publi Glob |
/// +---------------+--------------+---------------+-----------+------------+
enum mm_gens_loc_info
{
    /// Publication for Generic Location state (global part) to be sent after
    /// confirmation received from application
    MM_GENS_LOC_INFO_PUBLI_GLOB_POS = 0,
    MM_GENS_LOC_INFO_PUBLI_GLOB_BIT = CO_BIT(MM_GENS_LOC_INFO_PUBLI_GLOB_POS),

    /// Publication for Generic Location state (local part) to be sent after
    /// confirmation received from application
    MM_GENS_LOC_INFO_PUBLI_LOC_POS = 1,
    MM_GENS_LOC_INFO_PUBLI_LOC_BIT = CO_BIT(MM_GENS_LOC_INFO_PUBLI_LOC_POS),

    /// Wait for confirmation from application for Generic Location Global Get message
    MM_GENS_LOC_INFO_WAIT_APP_GLOB_POS = 2,
    MM_GENS_LOC_INFO_WAIT_APP_GLOB_BIT = CO_BIT(MM_GENS_LOC_INFO_WAIT_APP_GLOB_POS),

    /// Wait for confirmation from application for Generic Location Local Get message
    MM_GENS_LOC_INFO_WAIT_APP_LOC_POS = 3,
    MM_GENS_LOC_INFO_WAIT_APP_LOC_BIT = CO_BIT(MM_GENS_LOC_INFO_WAIT_APP_LOC_POS),
};

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

/// Structure for Generic Location Server model environment
typedef struct mm_gens_loc_env
{
    /// Basic model environment - Must be first element in the structure - DO NOT MOVE
    mm_tb_state_mdl_env_t env;
    /// Timer for sending of publications
    mesh_tb_timer_t tmr_publi;
    /// Publication period
    uint32_t publi_period_ms;

    /// List of prepared Generic Location Global Status messages
    co_list_t list_status_glob;
    /// List of prepared Generic Location Local Status messages
    co_list_t list_status_loc;

    /// Information (@see enum mm_gens_loc_info)
    uint8_t info;
} mm_gens_loc_env_t;

/// Structure for Generic Location Setup Server model environment
typedef struct mm_gens_locs_env
{
    /// Basic model environment - Must be first element in the structure - DO NOT MOVE
    mm_tb_state_mdl_env_t env;
    /// Pointer to environment of associated Generic Location Server model
    mm_gens_loc_env_t *p_env_loc;
} mm_gens_locs_env_t;

/*
 * LOCAL FUNCTIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief TODO [LT]
 ****************************************************************************************
 */
__STATIC mesh_tb_buf_t *mm_gens_loc_find_buf(co_list_t *p_list, uint16_t src, bool extract)
{
    // Get first buffer
    mesh_tb_buf_t *p_buf = (mesh_tb_buf_t *)co_list_pick(p_list);

    while (p_buf)
    {
        // Read environment
        mm_route_buf_env_t *p_env = (mm_route_buf_env_t *)&p_buf->env[0];

        if (p_env->u_addr.src == src)
        {
            // Extract the buffer if required
            if (extract)
            {
                CO_LIST_POP_ELT(*p_list, p_buf);
            }

            break;
        }

        // Get next buffer
        p_buf = (mesh_tb_buf_t *)p_buf->hdr.next;
    }

    return (p_buf);
}

/**
 ****************************************************************************************
 * @brief Prepare and send a Generic Location Global Status message
 *
 * @param[in] p_env_loc          Pointer to Generic Location Server model environment
 ****************************************************************************************
 */
__STATIC void mm_gens_loc_send_status_global(mm_gens_loc_env_t *p_env_loc, mesh_tb_buf_t *p_buf,
        mm_route_buf_env_t *p_route_env,
        int32_t latitude, int32_t longitude, int16_t altitude)
{
    // Status
    uint8_t status;

    if (p_buf)
    {
        status = MESH_ERR_NO_ERROR;
    }
    else
    {
        status = mm_route_buf_alloc(&p_buf, MM_GEN_LOC_STATUS_GLOB_LEN);
    }

    if (status == MESH_ERR_NO_ERROR)
    {
        // Get pointer to data
        uint8_t *p_data = MESH_TB_BUF_DATA(p_buf);
        // Get pointer to environment
        mm_route_buf_env_t *p_env = (mm_route_buf_env_t *)&p_buf->env[0];

        if (p_route_env)
        {
            memcpy(p_env, p_route_env, sizeof(mm_route_buf_env_t));
        }

        p_env->opcode = MM_MSG_GEN_LOCG_STATUS;
        p_env->mdl_lid = p_env_loc->env.mdl_lid;
        SETB(p_env->info, MM_ROUTE_BUF_INFO_PUBLISH, (p_buf == NULL) && (p_route_env == NULL));
        SETB(p_env->info, MM_ROUTE_BUF_INFO_RX, 0);

        // Fill the message
        co_write32p(p_data + MM_GEN_LOC_STATUS_GLOB_LAT_POS, latitude);
        co_write32p(p_data + MM_GEN_LOC_STATUS_GLOB_LONG_POS, longitude);
        co_write16p(p_data + MM_GEN_LOC_STATUS_GLOB_ALT_POS, altitude);

        // Send the message
        mm_route_send(p_buf);
    }
    else
    {
        MESH_MODEL_PRINT_WARN("%s, buffer alloc fail.\n", __func__);
    }
}

/**
 ****************************************************************************************
 * @brief Prepare and send a Generic Location Local Status message
 *
 * @param[in] p_env_loc          Pointer to Generic Location Server model environment
 ****************************************************************************************
 */
__STATIC void mm_gens_loc_send_status_local(mm_gens_loc_env_t *p_env_loc, mesh_tb_buf_t *p_buf,
        mm_route_buf_env_t *p_route_env,
        int16_t north, int16_t east, int16_t altitude, uint8_t floor,
        uint16_t uncertainty)
{
    // Status
    uint8_t status;

    if (p_buf)
    {
        status = MESH_ERR_NO_ERROR;
    }
    else
    {
        status = mm_route_buf_alloc(&p_buf, MM_GEN_LOC_STATUS_LOC_LEN);
    }

    if (status == MESH_ERR_NO_ERROR)
    {
        // Get pointer to data
        uint8_t *p_data = MESH_TB_BUF_DATA(p_buf);
        // Get pointer to environment
        mm_route_buf_env_t *p_env = (mm_route_buf_env_t *)&p_buf->env[0];

        if (p_route_env)
        {
            memcpy(p_env, p_route_env, sizeof(mm_route_buf_env_t));
        }

        p_env->opcode = MM_MSG_GEN_LOCL_STATUS;
        p_env->mdl_lid = p_env_loc->env.mdl_lid;
        SETB(p_env->info, MM_ROUTE_BUF_INFO_PUBLISH, (p_buf == NULL) && (p_route_env == NULL));
        SETB(p_env->info, MM_ROUTE_BUF_INFO_RX, 0);

        // Fill the message
        co_write16p(p_data + MM_GEN_LOC_STATUS_LOC_NORTH_POS, north);
        co_write16p(p_data + MM_GEN_LOC_STATUS_LOC_EAST_POS, east);
        co_write16p(p_data + MM_GEN_LOC_STATUS_LOC_ALT_POS, altitude);
        *(p_data + MM_GEN_LOC_STATUS_LOC_FLOOR_POS) = floor;
        co_write16p(p_data + MM_GEN_LOC_STATUS_LOC_UNCERT_POS, uncertainty);

        // Send the message
        mm_route_send(p_buf);
    }
    else
    {
        MESH_MODEL_PRINT_WARN("%s, Buffer alloc fail.\n", __func__);
    }
}

/**
 ****************************************************************************************
 * @brief Inform application that current Generic Location State is required (global part)
 *
 * @param[in] p_env_loc          Pointer to Generic Location Server model environment
 ****************************************************************************************
 */
__STATIC void mm_gens_loc_send_global_req_ind(mm_gens_loc_env_t *p_env_loc)
{
    // Check that a confirmation is not already expected from application
    if (!GETB(p_env_loc->info, MM_GENS_LOC_INFO_WAIT_APP_GLOB))
    {
        SETB(p_env_loc->info, MM_GENS_LOC_INFO_WAIT_APP_GLOB, 1);
        mm_api_send_srv_state_req_ind(MM_API_SRV_LOCG_REQ_IND, p_env_loc->env.elmt_idx);
    }
}

/**
 ****************************************************************************************
 * @brief Inform application that current Generic Location State is required (local part)
 *
 * @param[in] p_env_loc          Pointer to Generic Location Server model environment
 ****************************************************************************************
 */
__STATIC void mm_gens_loc_send_local_req_ind(mm_gens_loc_env_t *p_env_loc)
{
    // Check that a confirmation is not already expected from application
    if (!GETB(p_env_loc->info, MM_GENS_LOC_INFO_WAIT_APP_LOC))
    {
        SETB(p_env_loc->info, MM_GENS_LOC_INFO_WAIT_APP_LOC, 1);
        mm_api_send_srv_state_req_ind(MM_API_SRV_LOCL_REQ_IND, p_env_loc->env.elmt_idx);
    }
}

/**
 ****************************************************************************************
 * @brief Handler for Generic Location Global Set/Set Unacknowledged message
 *
 * @param[in] p_env_loc     Pointer to Generic Location Server model environment
 * @param[in] p_buf         Pointer to buffer containing the received message
 * @param[in] opcode        Received operation code
 ****************************************************************************************
 */
__STATIC void mm_gens_loc_handler_set_global(mm_gens_loc_env_t *p_env_loc, mesh_tb_buf_t *p_buf,
        mm_route_buf_env_t *p_route_env)
{
    // Get pointer to data
    uint8_t *p_data = MESH_TB_BUF_DATA(p_buf);
    // Global Latitude
    int32_t latitude = (int32_t)co_read32p(p_data + MM_GEN_LOC_SET_GLOB_LAT_POS);
    // Global Longitude
    int32_t longitude = (int32_t)co_read32p(p_data + MM_GEN_LOC_SET_GLOB_LONG_POS);
    // Global Altitude
    int16_t altitude = (int16_t)co_read16p(p_data + MM_GEN_LOC_SET_GLOB_ALT_POS);

    // Inform application about new state value
    mm_api_send_srv_locg_upd_ind(p_env_loc->env.elmt_idx, latitude, longitude, altitude);

    if (p_route_env->opcode == MM_MSG_GEN_LOCG_SET)
    {
        // Send a Generic Location Global Status message
        mm_gens_loc_send_status_global(p_env_loc, NULL, p_route_env, latitude, longitude, altitude);
    }
}

/**
 ****************************************************************************************
 * @brief Handler for Generic Location Local Set/Set Unacknowledged message
 *
 * @param[in] p_env_loc     Pointer to Generic Location Server model environment
 * @param[in] p_buf         Pointer to buffer containing the received message
 * @param[in] opcode        Received operation code
 ****************************************************************************************
 */
__STATIC void mm_gens_loc_handler_set_local(mm_gens_loc_env_t *p_env_loc, mesh_tb_buf_t *p_buf,
        mm_route_buf_env_t *p_route_env)
{
    // Get pointer to data
    uint8_t *p_data = MESH_TB_BUF_DATA(p_buf);
    // Local North
    int16_t north = (int16_t)co_read16p(p_data + MM_GEN_LOC_SET_GLOB_LAT_POS);
    // Local East
    int16_t east = (int16_t)co_read16p(p_data + MM_GEN_LOC_SET_GLOB_LONG_POS);
    // Global Altitude
    int16_t altitude = (int16_t)co_read16p(p_data + MM_GEN_LOC_SET_GLOB_ALT_POS);
    // Floor
    uint8_t floor = *(p_data + MM_GEN_LOC_SET_GLOB_LONG_POS);
    // Uncertainty
    uint16_t uncertainty = co_read16p(p_data + MM_GEN_LOC_SET_GLOB_ALT_POS);

    // Inform application about new state value
    mm_api_send_srv_locl_upd_ind(p_env_loc->env.elmt_idx, north, east, altitude,
                                 floor, uncertainty);

    if (p_route_env->opcode == MM_MSG_GEN_LOCL_SET)
    {
        // Send a Generic Location Local Status message
        mm_gens_loc_send_status_local(p_env_loc, NULL, p_route_env, north, east, altitude,
                                      floor, uncertainty);
    }
}

/**
 ****************************************************************************************
 * @brief Prepare a buffer for transmission of Generic Location Global Status or Generic
 * Location Local Status message after reception of a get message
 *
 * @param[in] p_env_loc          Pointer to Generic Location Server model environment
 * @param[in] p_route_env        Information about received Generic Location Global/Local Get message
 * @param[in] global             True if Generic Location Global Get message has been received, else False
 ****************************************************************************************
 */
__STATIC void mm_gens_loc_prepare_status(mm_gens_loc_env_t *p_env_loc,
        mm_route_buf_env_t *p_route_env, bool global)
{
    // Pointer to the list of received get messages to parse
    co_list_t *p_list = (global) ? &p_env_loc->list_status_glob : &p_env_loc->list_status_loc;

    // Check that a Generic Location Global/Local Get message from the same source address has not already
    // been received
    if (!mm_gens_loc_find_buf(p_list, p_route_env->u_addr.src, false))
    {
        // Buffer that will contain the Generic Location Global/Local Status message
        mesh_tb_buf_t *p_buf_status;
        // Allocate a new buffer
        uint16_t status = mm_route_buf_alloc(&p_buf_status,
                                             (global) ? MM_GEN_LOC_STATUS_GLOB_LEN
                                             : MM_GEN_LOC_STATUS_LOC_LEN);

        if (status == MESH_ERR_NO_ERROR)
        {
            // Copy the received environment
            memcpy(&p_buf_status->env[0], p_route_env, sizeof(mm_route_buf_env_t));

            // Insert the buffer in the list of received get messages
            co_list_push_back(p_list, &p_buf_status->hdr);

            // Retrieve the Generic Location state from the application
            if (global)
            {
                mm_gens_loc_send_global_req_ind(p_env_loc);
            }
            else
            {
                mm_gens_loc_send_local_req_ind(p_env_loc);
            }
        }
        else
        {
            MESH_MODEL_PRINT_WARN("%s, Buffer alloc fail.\n", __func__);
        }
    }
}

/*
 * INTERNAL CALLBACK FUNCTIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Callback function called when timer monitoring publication duration for
 * Generic Location Server model expires
 *
 * @param[in] p_env     Pointer to model environment for Generic Location Server model
 ****************************************************************************************
 */
__STATIC void mm_gens_loc_cb_tmr_publi(void *p_env)
{
    // Get allocated environment
    mm_gens_loc_env_t *p_env_loc = (mm_gens_loc_env_t *)p_env;

    if (p_env_loc->publi_period_ms)
    {
        // Keep in mind that a publication must be sent
        SETB(p_env_loc->info, MM_GENS_LOC_INFO_PUBLI_LOC, 1);
        SETB(p_env_loc->info, MM_GENS_LOC_INFO_PUBLI_GLOB, 1);

        // Retrieve current Generic Location state (Global part) from the application
        mm_gens_loc_send_global_req_ind(p_env_loc);
        // Retrieve current Generic Location state (Location part) from the application
        mm_gens_loc_send_local_req_ind(p_env_loc);

        // Restart the timer
        mesh_tb_timer_set(&p_env_loc->tmr_publi, p_env_loc->publi_period_ms);
    }
}

/**
 ****************************************************************************************
 * @brief Inform Generic Location Server model about a received message
 *
 * @param[in] p_env         Pointer to the environment allocated for the Generic Location
 * Server model
 * @param[in] p_buf         Pointer to the buffer containing the received message
 * @param[in] p_route_env   Pointer to structure containing reception parameters provided
 * by the Mesh Profile block
 ****************************************************************************************
 */
__STATIC void mm_gens_loc_cb_rx(mm_tb_state_mdl_env_t *p_env, mesh_tb_buf_t *p_buf, mm_route_buf_env_t *p_route_env)
{
    // Environment for Generic Location Server model
    mm_gens_loc_env_t *p_env_loc;

    if ((p_route_env->opcode == MM_MSG_GEN_LOCG_GET)
            || (p_route_env->opcode == MM_MSG_GEN_LOCL_GET))
    {
        p_env_loc = (mm_gens_loc_env_t *)p_env;

        // Prepare environment for transition of status message
        mm_gens_loc_prepare_status(p_env_loc, p_route_env,
                                   (p_route_env->opcode == MM_MSG_GEN_LOCG_GET));
    }
    else
    {
        // Environment for Generic Location Setup Server model
        mm_gens_locs_env_t *p_env_locs = (mm_gens_locs_env_t *)p_env;

        p_env_loc = p_env_locs->p_env_loc;

        if ((p_route_env->opcode == MM_MSG_GEN_LOCG_SET)
                || (p_route_env->opcode == MM_MSG_GEN_LOCG_SET_UNACK))
        {
            // Handle the received message
            mm_gens_loc_handler_set_global(p_env_loc, p_buf, p_route_env);
        }
        else
        {
            // Handle the receive message
            mm_gens_loc_handler_set_local(p_env_loc, p_buf, p_route_env);
        }
    }
}

/**
 ****************************************************************************************
 * @brief Check if a given opcode is handled by the Generic Location Server model
 *
 * @param[in] p_env         Pointer to the environment allocated for the Generic Location
 * Server model
 * @param[in] opcode        Opcode to check
 *
 * @return An error status (@see enum mesh_err)
 ****************************************************************************************
 */
__STATIC uint16_t mm_gens_loc_cb_opcode_check(mm_tb_state_mdl_env_t *p_env, uint32_t opcode)
{
    uint16_t status = MESH_ERR_MDL_INVALID_OPCODE;

    if (p_env->mdl_id == MM_ID_GENS_LOC)
    {
        if ((opcode == MM_MSG_GEN_LOCG_GET)
                || (opcode == MM_MSG_GEN_LOCL_GET))
        {
            status = MESH_ERR_NO_ERROR;
        }
    }
    else if (p_env->mdl_id == MM_ID_GENS_LOCS)
    {
        if ((opcode == MM_MSG_GEN_LOCG_SET)
                || (opcode == MM_MSG_GEN_LOCG_SET_UNACK)
                || (opcode == MM_MSG_GEN_LOCL_SET)
                || (opcode == MM_MSG_GEN_LOCL_SET_UNACK))
        {
            status = MESH_ERR_NO_ERROR;
        }
    }

    return (status);
}

/**
 ****************************************************************************************
 * @brief Inform Generic Location Server model about received publication parameters
 *
 * @param[in] p_env         Pointer the the environment allocated for the Generic Location
 * Server model
 * @param[in] addr          Publication address
 * @param[in] period_ms     Publication period in milliseconds
 ****************************************************************************************
 */
__STATIC void mm_gens_loc_cb_publish_param(mm_tb_state_mdl_env_t *p_env, uint16_t addr, uint32_t period_ms)
{
    mm_gens_loc_env_t *p_env_loc = (mm_gens_loc_env_t *)p_env;

    // Inform the state manager about received publication parameters
    mm_tb_state_publish_param_ind((mm_tb_state_mdl_publi_env_t *)p_env, addr, period_ms);

    if (p_env_loc->publi_period_ms == 0)
    {
        SETB(p_env_loc->info, MM_GENS_LOC_INFO_PUBLI_LOC, 0);
        SETB(p_env_loc->info, MM_GENS_LOC_INFO_PUBLI_GLOB, 0);
    }
}

/*
 * GLOBAL FUNCTIONS
 ****************************************************************************************
 */

uint16_t mm_gens_loc_register(uint8_t elmt_idx)
{
    uint16_t status;

    do
    {
        // Allocated model local index
        m_lid_t mdl_lid;
        // Pointer to allocated environment for Generic Location Server model
        mm_gens_loc_env_t *p_env_loc;

        // Register Generic Location Server model
        status = m_api_register_model(MM_ID_GENS_LOC, elmt_idx, M_MDL_CONFIG_PUBLI_AUTH_BIT,
                                      &mm_route_cb, &mdl_lid);

        if (status != MESH_ERR_NO_ERROR)
        {
            MESH_MODEL_PRINT_WARN("%s, egister Generic Location Server model fail, status = 0x%x\n",
                                  __func__, status);
            break;
        }

        // Inform the Model State Manager about registered model
        status = mm_tb_state_register(elmt_idx, MM_ID_GENS_LOC, mdl_lid,
                                      MM_TB_STATE_ROLE_SRV, sizeof(mm_gens_loc_env_t));

        if (status != MESH_ERR_NO_ERROR)
        {
            MESH_MODEL_PRINT_WARN("%s, register Generic Location Server Model State fail, status = 0x%x\n",
                                  __func__, status);
            break;
        }

        // Get allocated environment
        p_env_loc = (mm_gens_loc_env_t *)mm_tb_state_get_env(mdl_lid);

        // Set internal callback functions
        p_env_loc->tmr_publi.cb = mm_gens_loc_cb_tmr_publi;
        p_env_loc->tmr_publi.p_env = (void *)p_env_loc;

        // Set internal callback functions
        p_env_loc->env.cb.cb_rx = mm_gens_loc_cb_rx;
        p_env_loc->env.cb.cb_opcode_check = mm_gens_loc_cb_opcode_check;
        p_env_loc->env.cb.cb_publish_param = mm_gens_loc_cb_publish_param;

        // Inform application about registered model
        mm_api_send_register_ind(MM_ID_GENS_LOC, elmt_idx, mdl_lid);

        // Register Generic Location Setup Server model
        status = m_api_register_model(MM_ID_GENS_LOCS, elmt_idx, 0, &mm_route_cb, &mdl_lid);

        if (status != MESH_ERR_NO_ERROR)
        {
            break;
        }

        // Inform the Model State Manager about registered model
        status = mm_tb_state_register(elmt_idx, MM_ID_GENS_LOCS, mdl_lid,
                                      MM_TB_STATE_ROLE_SRV, sizeof(mm_gens_locs_env_t));

        if (status == MESH_ERR_NO_ERROR)
        {
            // Get allocated environment
            mm_gens_locs_env_t *p_env_locs = (mm_gens_locs_env_t *)mm_tb_state_get_env(mdl_lid);

            // Set internal callback functions
            p_env_locs->env.cb.cb_rx = mm_gens_loc_cb_rx;
            p_env_locs->env.cb.cb_opcode_check = mm_gens_loc_cb_opcode_check;

            // Link environment
            p_env_locs->p_env_loc = p_env_loc;

            // Inform application about registered model
            mm_api_send_register_ind(MM_ID_GENS_LOCS, elmt_idx, mdl_lid);
        }
        else
        {
            MESH_MODEL_PRINT_WARN("%s, register Generic Location Server Model State fail, status = 0x%x\n",
                                  __func__, status);
        }
    }
    while (0);

    return (status);
}

void mm_gens_loc_global_cfm(mm_tb_state_mdl_env_t *p_env, uint16_t status, int32_t latitude,
                            int32_t longitude, int16_t altitude)
{
    mm_gens_loc_env_t *p_env_loc = (mm_gens_loc_env_t *)p_env;

    // Check if confirmation from application was expected
    if (GETB(p_env_loc->info, MM_GENS_LOC_INFO_WAIT_APP_GLOB))
    {
        SETB(p_env_loc->info, MM_GENS_LOC_INFO_WAIT_APP_GLOB, 0);

        if (status != MESH_ERR_NO_ERROR)
        {
            latitude = MM_LOC_GLOBAL_LAT_NOT_CONFIG;
            longitude = MM_LOC_GLOBAL_LONG_NOT_CONFIG;
            altitude = MM_LOC_GLOBAL_ALT_NOT_CONFIG;
        }

        // Send responses for received get requests
        while (!co_list_is_empty(&p_env_loc->list_status_glob))
        {
            mesh_tb_buf_t *p_buf = (mesh_tb_buf_t *)co_list_pop_front(&p_env_loc->list_status_glob);

            mm_gens_loc_send_status_global(p_env_loc, p_buf, NULL,
                                           latitude, longitude, altitude);
        }

        // Send a publication if needed
        if (GETB(p_env_loc->info, MM_GENS_LOC_INFO_PUBLI_GLOB))
        {
            SETB(p_env_loc->info, MM_GENS_LOC_INFO_PUBLI_GLOB, 0);

            mm_gens_loc_send_status_global(p_env_loc, NULL, NULL,
                                           latitude, longitude, altitude);
        }
    }
}

void mm_gens_loc_local_cfm(mm_tb_state_mdl_env_t *p_env, uint16_t status, int16_t north,
                           int16_t east, int16_t altitude, uint8_t floor, uint16_t uncertainty)
{
    mm_gens_loc_env_t *p_env_loc = (mm_gens_loc_env_t *)p_env;

    // Check if confirmation from application was expected
    if (GETB(p_env_loc->info, MM_GENS_LOC_INFO_WAIT_APP_LOC))
    {
        SETB(p_env_loc->info, MM_GENS_LOC_INFO_WAIT_APP_LOC, 0);

        if (status != MESH_ERR_NO_ERROR)
        {
            north = MM_LOC_LOCAL_NORTH_NOT_CONFIG;
            east = MM_LOC_LOCAL_EAST_NOT_CONFIG;
            altitude = MM_LOC_LOCAL_ALT_NOT_CONFIG;
            floor = MM_LOC_FLOOR_NOT_CONFIG;
            uncertainty = 0;
        }

        // Send responses for received get requests
        while (!co_list_is_empty(&p_env_loc->list_status_loc))
        {
            mesh_tb_buf_t *p_buf = (mesh_tb_buf_t *)co_list_pop_front(&p_env_loc->list_status_loc);

            mm_gens_loc_send_status_local(p_env_loc, p_buf, NULL,
                                          north, east, altitude, floor, uncertainty);
        }

        // Send a publication if needed
        if (GETB(p_env_loc->info, MM_GENS_LOC_INFO_PUBLI_LOC))
        {
            SETB(p_env_loc->info, MM_GENS_LOC_INFO_PUBLI_LOC, 0);

            mm_gens_loc_send_status_local(p_env_loc, NULL, NULL,
                                          north, east, altitude, floor, uncertainty);
        }
    }
}
#endif //(BLE_MESH_MDL_GENS_LOC)
/// @} end of group
