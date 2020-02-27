/**
 ****************************************************************************************
 *
 * @file mm_api_msg.c
 *
 * @brief Mesh Model Message Application Programming Interface
 *
 * Copyright (C) RivieraWaves 2017-2019
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup MM_API_MSG
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "mm_inc.h"          // Mesh Model Includes

#if (BLE_MESH_MSG_API)
#include "mesh_api_msg.h"
#include "mal.h"
#include "uart.h"
/*
 * LOCAL FUNCTIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Send a basic command complete message to the application
 *
 * @param[in] cmd_code      Command Code
 * @param[in] src_id        Source identifier of task that request command execution
 * @param[in] status        Status error code of the command execution (@see enum hl_err)
 ****************************************************************************************
 */
__STATIC void mm_api_msg_send_basic_cmp_evt(uint16_t cmd_code, uint16_t src_id, uint16_t status)
{
    // Allocate the command complete event message
    mm_api_cmp_evt_t *p_cmp_evt = MAL_MSG_ALLOC(MESH_MDL_API_CMP_EVT, mm_api_cmp_evt_t);

    // Fill the content
    p_cmp_evt->cmd_code = cmd_code;
    p_cmp_evt->status = status;

    // Send the message
    mal_msg_send(src_id, p_cmp_evt);
}

/*
 * CALLBACKS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Callback function called by Mesh Model block upon registration of a model if
 * API MSG is used.
 * Allocate and fill a MESH_MDL_API_IND message with a MM_API_REGISTER_IND indication
 * code.
 *
 * @param[in] model_id      Model identifier
 * @param[in] elmt_idx      Element Index
 * @param[in] mdl_lid       Model local index
 ****************************************************************************************
 */
__STATIC void mm_api_msg_cb_register_ind(uint32_t model_id, uint8_t elmt_idx, m_lid_t mdl_lid)
{
    // Allocate the indication message
    mm_api_register_ind_t *p_ind = MAL_MSG_ALLOC(MESH_MDL_API_IND, mm_api_register_ind_t);

    // Fill the content
    p_ind->ind_code = MM_API_REGISTER_IND;
    p_ind->model_id = model_id;
    p_ind->elmt_idx = elmt_idx;
    p_ind->mdl_lid = mdl_lid;

    // Send the message
    mal_msg_send(mal_app_id_get(), p_ind);
}

#if (BLE_MESH_MDL_SERVER)
/**
 ****************************************************************************************
 * @brief Callback function called by Mesh Model block to inform the application about update
 * of a local state
 *
 * @param[in] state_id          State identifier
 * @param[in] elmt_idx          Element Index
 * @param[in] state             New state or targeted state value depending on the provided
 * transition time
 * @param[in] trans_time_ms     Transition time in milliseconds
 ****************************************************************************************
 */
__STATIC void mm_api_msg_cb_srv_state_upd_ind(uint16_t state_id, uint8_t elmt_idx, uint32_t state,
        uint32_t trans_time_ms)
{

    // Allocate the indication message
    mm_api_srv_state_upd_ind_t *p_ind = MAL_MSG_ALLOC(MESH_MDL_API_IND, mm_api_srv_state_upd_ind_t);

    // Fill the content
    p_ind->ind_code = MM_API_SRV_STATE_UPD_IND;
    p_ind->elmt_idx = elmt_idx;
    p_ind->state = state;
    p_ind->state_id = state_id;
    p_ind->trans_time_ms = trans_time_ms;

    // Send the message
    mal_msg_send(mal_app_id_get(), p_ind);
}

__STATIC void mm_api_msg_cb_srv_array_state_upd_ind(uint16_t state_id, uint8_t elmt_idx, uint8_t len, uint8_t* state,
        uint32_t trans_time_ms)
{

    // Allocate the indication message
    mm_api_srv_array_state_upd_ind_t *p_ind = MAL_MSG_ALLOC_DYN(MESH_MDL_API_IND, mm_api_srv_array_state_upd_ind_t, len);

    // Fill the content
    p_ind->ind_code = MM_API_SRV_ARRAY_STATE_UPD_IND;
    p_ind->elmt_idx = elmt_idx;
    p_ind->len = len;
    memcpy(p_ind->val, state, len);
    p_ind->state_id = state_id;
    p_ind->trans_time_ms = trans_time_ms;

    // Send the message
    mal_msg_send(mal_app_id_get(), p_ind);
}



/**
 ****************************************************************************************
 * @brief Callback function called by Mesh Model block when a state value not stored locally
 * must be retrieved from upper application.
 * Allocate and fill a MESH_MDL_API_REQ_IND message with a provided req_ind_code request
 * indication code.
 *
 * @param[in] req_ind_code      Request indication code
 * @param[in] elmt_idx          Element Index
 ****************************************************************************************
 */
__STATIC void mm_api_msg_cb_srv_state_req_ind(uint32_t req_ind_code, uint8_t elmt_idx)
{
    // Allocate the indication message
    mm_api_srv_state_req_ind_t *p_req_ind = MAL_MSG_ALLOC(MESH_MDL_API_REQ_IND, mm_api_srv_state_req_ind_t);

    // Fill the content
    p_req_ind->req_ind_code = req_ind_code;
    p_req_ind->elmt_idx = elmt_idx;

    // Send the message
    mal_msg_send(mal_app_id_get(), p_req_ind);
}

#if (BLE_MESH_MDL_GENS)
/**
 ****************************************************************************************
 * @brief Callback function called by Mesh Model block when global part of Generic Location
 * state value is set.
 * Allocate and fill a MESH_MDL_API_IND message with a MM_API_CLI_LOCG_IND indication
 * code.
 *
 * @param[in] elmt_idx      Element Index
 * @param[in] latitude      Global latitude
 * @param[in] longitude     Global longitude
 * @param[in] altitude      Global altitude
 ****************************************************************************************
 */
__STATIC void mm_api_msg_cb_srv_locg_upd_ind(uint8_t elmt_idx, int32_t latitude, int32_t longitude,
        int16_t altitude)
{
    // Allocate the indication message
    mm_api_srv_locg_upd_ind_t *p_ind = MAL_MSG_ALLOC(MESH_MDL_API_IND, mm_api_srv_locg_upd_ind_t);

    // Fill the content
    p_ind->ind_code = MM_API_CLI_LOCG_IND;
    p_ind->latitude = latitude;
    p_ind->longitude = longitude;
    p_ind->altitude = altitude;
    p_ind->elmt_idx = elmt_idx;

    // Send the message
    mal_msg_send(mal_app_id_get(), p_ind);
}

/**
 ****************************************************************************************
 * @brief Callback function called by Mesh Model block when local part of Generic Location
 * state value is set.
 * Allocate and fill a MESH_MDL_API_IND message with a MM_API_CLI_LOCL_IND indication
 * code.
 *
 * @param[in] elmt_idx          Element Index
 * @param[in] north             Local North
 * @param[in] east              Local East
 * @param[in] altitude          Local Altitude
 * @param[in] floor             Floor Number
 * @param[in] uncertainty       Uncertainty
 ****************************************************************************************
 */
__STATIC void mm_api_msg_cb_srv_locl_upd_ind(uint8_t elmt_idx, int16_t north, int16_t east,
        int16_t altitude, uint8_t floor, uint16_t uncertainty)
{
    // Allocate the indication message
    mm_api_srv_locl_upd_ind_t *p_ind = MAL_MSG_ALLOC(MESH_MDL_API_IND, mm_api_srv_locl_upd_ind_t);

    // Fill the content
    p_ind->ind_code = MM_API_CLI_LOCL_IND;
    p_ind->north = north;
    p_ind->east = east;
    p_ind->altitude = altitude;
    p_ind->floor = floor;
    p_ind->uncertainty = uncertainty;
    p_ind->elmt_idx = elmt_idx;

    // Send the message
    mal_msg_send(mal_app_id_get(), p_ind);
}

/**
 ****************************************************************************************
 * @brief TODO [LT]
 ****************************************************************************************
 */
__STATIC void mm_api_msg_cb_srv_prop_get_req_ind(uint8_t elmt_idx, uint8_t prop_type, uint16_t prop_id)
{
    // Allocate the indication message
    mm_api_srv_prop_get_req_ind_t *p_req_ind
        = MAL_MSG_ALLOC(MESH_MDL_API_REQ_IND, mm_api_srv_prop_get_req_ind_t);

    // Fill the content
    p_req_ind->req_ind_code = MM_API_SRV_PROP_GET_REQ_IND;
    p_req_ind->elmt_idx = elmt_idx;
    p_req_ind->prop_type = prop_type;
    p_req_ind->prop_id = prop_id;

    // Send the message
    mal_msg_send(mal_app_id_get(), p_req_ind);
}

/**
 ****************************************************************************************
 * @brief TODO [LT]
 ****************************************************************************************
 */
__STATIC void mm_api_msg_cb_srv_prop_set_req_ind(uint8_t elmt_idx, uint8_t prop_type, uint16_t prop_id,
        bool ack, uint16_t length, uint8_t *p_val)
{
    // Allocate the indication message
    mm_api_srv_prop_set_req_ind_t *p_req_ind
        = MAL_MSG_ALLOC_DYN(MESH_MDL_API_REQ_IND, mm_api_srv_prop_set_req_ind_t, length);

    // Fill the content
    p_req_ind->req_ind_code = MM_API_SRV_PROP_SET_REQ_IND;
    p_req_ind->elmt_idx = elmt_idx;
    p_req_ind->prop_type = prop_type;
    p_req_ind->prop_id = prop_id;
    p_req_ind->ack = ack;
    p_req_ind->length = length;
    memcpy(&p_req_ind->val[0], p_val, length);

    // Send the message
    mal_msg_send(mal_app_id_get(), p_req_ind);
}
#endif //(BLE_MESH_MDL_GENS)
#endif //(BLE_MESH_MDL_SERVER)

#if (BLE_MESH_MDL_CLIENT)
/**
 ****************************************************************************************
 * @brief Callback function called by Mesh Model block when a state value has been received
 * Allocate and fill a MESH_MDL_API_IND message with a MM_API_CLI_STATE_IND indication
 * code.
 *
 * @param[in] src               Address of node's element that has reported one of its state value
 * @param[in] state_id          State Identifier
 * @param[in] state_1           State value 1
 * @param[in] state_2           State value 2
 * @param[in] rem_time_ms       Remaining time before end of transition in milliseconds
 ****************************************************************************************
 */
__STATIC void mm_api_msg_cb_cli_state_ind(uint16_t src, uint16_t state_id, uint32_t state_1,
        uint32_t state_2, uint32_t rem_time_ms)
{
    // Allocate the indication message
    mm_api_cli_state_ind_t *p_ind = MAL_MSG_ALLOC(MESH_MDL_API_IND, mm_api_cli_state_ind_t);

    // Fill the content
    p_ind->ind_code = MM_API_CLI_STATE_IND;
    p_ind->state_1 = state_1;
    p_ind->state_2 = state_2;
    p_ind->rem_time_ms = rem_time_ms;
    p_ind->src = src;
    p_ind->state_id = state_id;

    // Send the message
    mal_msg_send(mal_app_id_get(), p_ind);
}

#if (BLE_MESH_MDL_GENC)
/**
 ****************************************************************************************
 * @brief Callback function called by Mesh Model block upon reception of a Generic Battery
 * Status message
 * Allocate and fill a MESH_MDL_API_IND message with a MM_API_CLI_BAT_IND indication
 * code.
 *
 * @param[in] src               Address of node's element that has reported its Generic Battery state
 * @param[in] bat_lvl           Battery Level
 * @param[in] time_discharge    Time to discharge in minutes
 * @param[in] time_charge       Time to charge in minutes
 * @param[in] flags             Flags
 ****************************************************************************************
 */
__STATIC void mm_api_msg_cb_cli_bat_ind(uint16_t src, uint8_t bat_lvl, uint32_t time_discharge,
                                        uint32_t time_charge, uint8_t flags)
{
    // Allocate the indication message
    mm_api_cli_battery_ind_t *p_ind = MAL_MSG_ALLOC(MESH_MDL_API_IND, mm_api_cli_battery_ind_t);

    // Fill the content
    p_ind->ind_code = MM_API_CLI_BAT_IND;
    p_ind->bat_lvl = bat_lvl;
    p_ind->time_discharge = time_discharge;
    p_ind->time_charge = time_charge;
    p_ind->flags = flags;
    p_ind->src = src;

    // Send the message
    mal_msg_send(mal_app_id_get(), p_ind);
}

/**
 ****************************************************************************************
 * @brief Callback function called by Mesh Model block upon reception of a Generic Location
 * Global Status message
 * Allocate and fill a MESH_MDL_API_IND message with a MM_API_CLI_LOCG_IND indication
 * code.
 *
 * @param[in] src               Address of node's element that has reported its Generic Location state
 * @param[in] latitude          Global latitude
 * @param[in] longitude         Global longitude
 * @param[in] altitude          Global altitude
 ****************************************************************************************
 */
__STATIC void mm_api_msg_cb_cli_locg_ind(uint16_t src, int32_t latitude, int32_t longitude,
        int16_t altitude)
{
    // Allocate the indication message
    mm_api_cli_locg_ind_t *p_ind = MAL_MSG_ALLOC(MESH_MDL_API_IND, mm_api_cli_locg_ind_t);

    // Fill the content
    p_ind->ind_code = MM_API_CLI_LOCG_IND;
    p_ind->latitude = latitude;
    p_ind->longitude = longitude;
    p_ind->altitude = altitude;
    p_ind->src = src;

    // Send the message
    mal_msg_send(mal_app_id_get(), p_ind);
}

/**
 ****************************************************************************************
 * @brief Callback function called by Mesh Model block upon reception of a Generic Location
 * Local Status message
 * Allocate and fill a MESH_MDL_API_IND message with a MM_API_CLI_LOCL_IND indication
 * code.
 *
 * @param[in] src               Address of node's element that has reported its Generic Location state
 * @param[in] north             Local North
 * @param[in] east              Local East
 * @param[in] altitude          Local Altitude
 * @param[in] floor             Floor Number
 * @param[in] uncertainty       Uncertainty
 ****************************************************************************************
 */
__STATIC void mm_api_msg_cb_cli_locl_ind(uint16_t src, int16_t north, int16_t east, int16_t altitude,
        uint8_t floor, uint16_t uncertainty)
{
    // Allocate the indication message
    mm_api_cli_locl_ind_t *p_ind = MAL_MSG_ALLOC(MESH_MDL_API_IND, mm_api_cli_locl_ind_t);

    // Fill the content
    p_ind->ind_code = MM_API_CLI_LOCL_IND;
    p_ind->north = north;
    p_ind->east = east;
    p_ind->altitude = altitude;
    p_ind->floor = floor;
    p_ind->uncertainty = uncertainty;
    p_ind->src = src;

    // Send the message
    mal_msg_send(mal_app_id_get(), p_ind);
}

/**
 ****************************************************************************************
 * @brief Callback function called by Mesh Model block upon reception of a Property value
 * Allocate and fill a MESH_MDL_API_IND message with a MM_API_CLI_PROP_IND indication
 * code.
 *
 * @param[in] src               Address of node's element that has reported its list of properties
 * @param[in] prop_id           Property ID
 * @param[in] user_access       User access
 * @param[in] length            Property value length
 * @param[in] p_val             Pointer to Property value
 ****************************************************************************************
 */
__STATIC void mm_api_msg_cb_cli_prop_ind(uint16_t src, uint16_t prop_id, uint8_t user_access,
        uint16_t length, uint8_t *p_val)
{
    // Allocate the indication message
    mm_api_cli_prop_ind_t *p_ind = MAL_MSG_ALLOC_DYN(MESH_MDL_API_IND, mm_api_cli_prop_ind_t, length);

    // Fill the content
    p_ind->ind_code = MM_API_CLI_PROP_IND;
    p_ind->src = src;
    p_ind->prop_id = prop_id;
    p_ind->user_access = user_access;
    p_ind->length = length;
    memcpy(&p_ind->val[0], p_val, length);

    // Send the message
    mal_msg_send(mal_app_id_get(), p_ind);
}

/**
 ****************************************************************************************
 * @brief Callback function called by Mesh Model block upon reception of a list of Properties
 * (User, Manufacturer, Admin or Client)
 * Allocate and fill a MESH_MDL_API_IND message with a MM_API_CLI_PROP_LIST_IND indication
 * code.
 *
 * @param[in] src               Address of node's element that has reported its list of properties
 * @param[in] prop_type         Property Type
 * @param[in] nb_prop           Number of Properties
 * @param[in] p_prop_ids        Pointer to list of supported Property IDs
 ****************************************************************************************
 */
__STATIC void mm_api_msg_cb_cli_prop_list_ind(uint16_t src, uint8_t prop_type, uint16_t nb_prop,
        uint16_t *p_prop_ids)
{
    // Allocate the indication message
    mm_api_cli_prop_list_ind_t *p_ind = MAL_MSG_ALLOC_DYN(MESH_MDL_API_IND, mm_api_cli_prop_list_ind_t,
                                        nb_prop * sizeof(uint16_t));

    // Fill the content
    p_ind->ind_code = MM_API_CLI_PROP_LIST_IND;
    p_ind->src = src;
    p_ind->prop_type = prop_type;
    p_ind->nb_prop = nb_prop;
    memcpy(&p_ind->prop_ids[0], p_prop_ids, nb_prop * sizeof(uint16_t));

    // Send the message
    mal_msg_send(mal_app_id_get(), p_ind);
}
#endif //(BLE_MESH_MDL_GENC)
#endif //(BLE_MESH_MDL_CLIENT)

/*
 * GLOBAL VARIABLES
 ****************************************************************************************
 */

/// Mesh Model Callback Structure
__STATIC const mm_api_cb_t mm_api_msg_cb =
{
    .cb_register_ind = mm_api_msg_cb_register_ind,
#if (BLE_MESH_MDL_SERVER)
    .cb_srv_state_upd_ind = mm_api_msg_cb_srv_state_upd_ind,
    .cb_srv_array_state_upd_ind = mm_api_msg_cb_srv_array_state_upd_ind,
    .cb_srv_state_req_ind = mm_api_msg_cb_srv_state_req_ind,
#if (BLE_MESH_MDL_GENS)
    .cb_srv_locg_upd_ind = mm_api_msg_cb_srv_locg_upd_ind,
    .cb_srv_locl_upd_ind = mm_api_msg_cb_srv_locl_upd_ind,
    .cb_srv_prop_get_req_ind = mm_api_msg_cb_srv_prop_get_req_ind,
    .cb_srv_prop_set_req_ind = mm_api_msg_cb_srv_prop_set_req_ind,
#endif //(BLE_MESH_MDL_GENS)
#endif //(BLE_MESH_MDL_SERVER)
#if (BLE_MESH_MDL_CLIENT)
    .cb_cli_state_ind = mm_api_msg_cb_cli_state_ind,
#if (BLE_MESH_MDL_GENC)
    .cb_cli_bat_ind = mm_api_msg_cb_cli_bat_ind,
    .cb_cli_locg_ind = mm_api_msg_cb_cli_locg_ind,
    .cb_cli_locl_ind = mm_api_msg_cb_cli_locl_ind,
    .cb_cli_prop_ind = mm_api_msg_cb_cli_prop_ind,
    .cb_cli_prop_list_ind = mm_api_msg_cb_cli_prop_list_ind
#endif //(BLE_MESH_MDL_GENC)
#endif //(BLE_MESH_MDL_CLIENT)
};

/*
 * MESSAGE HANDLERS
 ****************************************************************************************
 */

#if (BLE_MESH_MDL_SERVER)
/**
 ****************************************************************************************
 * @brief Handler for MM_API_REGISTER_SERVER command
 *
 * @param[in] src_id    Identifier of the task that has sent the command
 * @param[in] p_cmd     Pointer to command parameters
 ****************************************************************************************
 */
__STATIC void mm_api_msg_register_server_cmd_handler(uint16_t src_id,
        const mm_api_register_server_cmd_t *p_cmd)
{
    // Forward the command
    uint16_t status = mm_api_register_server(p_cmd->elmt_idx, p_cmd->mdl_cfg_idx, p_cmd->info);

    // Send the command complete event
    mm_api_msg_send_basic_cmp_evt(MM_API_REGISTER_SERVER, src_id, status);
}

#if (BLE_MESH_MDL_GENS)
/**
 ****************************************************************************************
 * @brief Handler for MM_API_REGISTER_SERVER_PROP command
 *
 * @param[in] src_id    Identifier of the task that has sent the command
 * @param[in] p_cmd     Pointer to command parameters
 ****************************************************************************************
 */
__STATIC void mm_api_msg_register_server_prop_cmd_handler(uint16_t src_id,
        const mm_api_register_server_prop_cmd_t *p_cmd)
{
    // Forward the command
    uint16_t status = mm_api_register_server_prop(p_cmd->elmt_idx, p_cmd->req_queue_len, p_cmd->nb_prop_user,
                      p_cmd->nb_prop_admin, p_cmd->nb_prop_manuf,
                      p_cmd->nb_prop_cli, &p_cmd->prop_info[0]);

    // Send the command complete event
    mm_api_msg_send_basic_cmp_evt(MM_API_REGISTER_SERVER_PROP, src_id, status);
}
#endif //(BLE_MESH_MDL_GENS)

/**
 ****************************************************************************************
 * @brief General handler for MM_API_SRV_SET command
 *
 * @param[in] src_id    Identifier of the task that has sent the command
 * @param[in] p_cmd     Pointer to command parameters
 ****************************************************************************************
 */
__STATIC void mm_api_msg_srv_set_cmd_handler(uint16_t src_id, const mm_api_srv_set_cmd_t *p_cmd)
{
    // Forward the command
    uint16_t status = mm_api_srv_set(p_cmd->mdl_lid, p_cmd->state_id, p_cmd->state);

    // Send the command complete event
    mm_api_msg_send_basic_cmp_evt(MM_API_SRV_SET, src_id, status);
}

#if (BLE_MESH_MDL_GENS)
/**
 ****************************************************************************************
 * @brief Handler for confirmation of MM_API_SRV_BAT_REQ_IND request indication
 *
 * @param[in] p_cfm     Pointer to confirmation parameters
 ****************************************************************************************
 */

__STATIC void mm_api_msg_srv_bat_cfm_handler(const mm_api_srv_bat_cfm_t *p_cfm)
{
#if (BLE_MESH_MDL_GENS_BAT)
    // Forward the command
    mm_api_srv_bat_cfm(p_cfm->status, p_cfm->elmt_idx, p_cfm->bat_lvl, p_cfm->time_charge,
                       p_cfm->time_discharge, p_cfm->flags);
#endif //
}

/**
 ****************************************************************************************
 * @brief Handler for confirmation of MM_API_SRV_LOCG_REQ_IND request indication
 *
 * @param[in] p_cfm     Pointer to confirmation parameters
 ****************************************************************************************
 */

__STATIC void mm_api_msg_srv_locg_cfm_handler(const mm_api_srv_locg_cfm_t *p_cfm)
{
#if (BLE_MESH_MDL_GENS_LOC)
    // Forward the command
    mm_api_srv_locg_cfm(p_cfm->status, p_cfm->elmt_idx, p_cfm->latitude, p_cfm->longitude,
                        p_cfm->altitude);
#endif
}

/**
 ****************************************************************************************
 * @brief Handler for confirmation of MM_API_SRV_LOCL_REQ_IND request indication
 *
 * @param[in] p_cfm     Pointer to confirmation parameters
 ****************************************************************************************
 */
__STATIC void mm_api_msg_srv_locl_cfm_handler(const mm_api_srv_locl_cfm_t *p_cfm)
{
#if (BLE_MESH_MDL_GENS_LOC)
    // Forward the command
    mm_api_srv_locl_cfm(p_cfm->status, p_cfm->elmt_idx, p_cfm->north, p_cfm->east,
                        p_cfm->altitude, p_cfm->floor, p_cfm->uncertainty);
#endif
}

/**
 ****************************************************************************************
 * @brief Handler for confirmation of MM_API_SRV_PROP_GET_REQ_IND or
 * MM_API_SRV_PROP_SET_REQ_IND request indication
 *
 * @param[in] p_cfm     Pointer to confirmation parameters
 ****************************************************************************************
 */

__STATIC void mm_api_msg_srv_prop_cfm_handler(const mm_api_srv_prop_cfm_t *p_cfm)
{
#if (BLE_MESH_MDL_GENS_PROP)
    // Forward the command
    mm_api_srv_prop_cfm(p_cfm->status, p_cfm->elmt_idx, p_cfm->prop_type, p_cfm->prop_id,
                        p_cfm->length, &p_cfm->val[0]);
#endif
}
#endif //(BLE_MESH_MDL_GENS))
#endif //(BLE_MESH_MDL_SERVER)

#if (BLE_MESH_MDL_CLIENT)
/**
 ****************************************************************************************
 * @brief General handler for MM_API_REGISTER_CLIENT command
 *
 * @param[in] src_id    Identifier of the task that has sent the command
 * @param[in] p_cmd     Pointer to command parameters
 ****************************************************************************************
 */
__STATIC void mm_api_msg_register_client_cmd_handler(uint16_t src_id,
        const mm_api_register_client_cmd_t *p_cmd)
{
    // Forward the command
    uint16_t status = mm_api_register_client(p_cmd->cmdl_idx);

    // Send the command complete event
    mm_api_msg_send_basic_cmp_evt(MM_API_REGISTER_CLIENT, src_id, status);
}

/**
 ****************************************************************************************
 * @brief General handler for MM_API_CLI_GET command
 *
 * @param[in] src_id    Identifier of the task that has sent the command
 * @param[in] p_cmd     Pointer to command parameters
 ****************************************************************************************
 */
__STATIC void mm_api_msg_cli_get_cmd_handler(uint16_t src_id, const mm_api_cli_get_cmd_t *p_cmd)
{
    // Forward the command
    uint16_t status = mm_api_cli_get(p_cmd->mdl_lid, p_cmd->dst, p_cmd->get_info);

    // Send the command complete event
    mm_api_msg_send_basic_cmp_evt(MM_API_CLI_GET, src_id, status);
}

/**
 ****************************************************************************************
 * @brief General handler for MM_API_CLI_SET command
 *
 * @param[in] src_id    Identifier of the task that has sent the command
 * @param[in] p_cmd     Pointer to command parameters
 ****************************************************************************************
 */
__STATIC void mm_api_msg_cli_set_cmd_handler(uint16_t src_id, const mm_api_cli_set_cmd_t *p_cmd)
{
    // Forward the command
    uint16_t status = mm_api_cli_set(p_cmd->mdl_lid, p_cmd->dst, p_cmd->state_1,
                                     p_cmd->state_2, p_cmd->set_info);

    // Send the command complete event
    mm_api_msg_send_basic_cmp_evt(MM_API_CLI_SET, src_id, status);
}

/**
 ****************************************************************************************
 * @brief General handler for MM_API_CLI_SET_LOCG command
 *
 * @param[in] src_id    Identifier of the task that has sent the command
 * @param[in] p_cmd     Pointer to command parameters
 ****************************************************************************************
 */
__STATIC void mm_api_msg_cli_set_locg_cmd_handler(uint16_t src_id, const mm_api_cli_set_locg_cmd_t *p_cmd)
{
    // Forward the command
    uint16_t status = mm_api_cli_set_locg(p_cmd->mdl_lid, p_cmd->dst, p_cmd->set_info, p_cmd->latitude,
                                          p_cmd->longitude, p_cmd->altitude);

    // Send the command complete event
    mm_api_msg_send_basic_cmp_evt(MM_API_CLI_SET_LOCG, src_id, status);
}

/**
 ****************************************************************************************
 * @brief General handler for MM_API_CLI_SET_LOCL command
 *
 * @param[in] src_id    Identifier of the task that has sent the command
 * @param[in] p_cmd     Pointer to command parameters
 ****************************************************************************************
 */
__STATIC void mm_api_msg_cli_set_locl_cmd_handler(uint16_t src_id, const mm_api_cli_set_locl_cmd_t *p_cmd)
{
    // Forward the command
    uint16_t status = mm_api_cli_set_locl(p_cmd->mdl_lid, p_cmd->dst, p_cmd->set_info, p_cmd->north,
                                          p_cmd->east, p_cmd->altitude, p_cmd->floor, p_cmd->uncertainty);

    // Send the command complete event
    mm_api_msg_send_basic_cmp_evt(MM_API_CLI_SET_LOCL, src_id, status);
}

/**
 ****************************************************************************************
 * @brief General handler for MM_API_CLI_GET_PROP command
 *
 * @param[in] src_id    Identifier of the task that has sent the command
 * @param[in] p_cmd     Pointer to command parameters
 ****************************************************************************************
 */
__STATIC void mm_api_msg_cli_get_prop_cmd_handler(uint16_t src_id, const mm_api_cli_get_prop_cmd_t *p_cmd)
{
    // Forward the command
    uint16_t status = mm_api_cli_get_prop(p_cmd->mdl_lid, p_cmd->dst, p_cmd->get_type, p_cmd->prop_id);

    // Send the command complete event
    mm_api_msg_send_basic_cmp_evt(MM_API_CLI_GET_PROP, src_id, status);
}

/**
 ****************************************************************************************
 * @brief General handler for MM_API_CLI_SET_PROP command
 *
 * @param[in] src_id    Identifier of the task that has sent the command
 * @param[in] p_cmd     Pointer to command parameters
 ****************************************************************************************
 */
__STATIC void mm_api_msg_cli_set_prop_cmd_handler(uint16_t src_id, const mm_api_cli_set_prop_cmd_t *p_cmd)
{
    // Forward the command
    uint16_t status = mm_api_cli_set_prop(p_cmd->mdl_lid, p_cmd->dst, p_cmd->set_info,
                                          p_cmd->prop_id, p_cmd->user_access, p_cmd->length,
                                          &p_cmd->val[0]);

    // Send the command complete event
    mm_api_msg_send_basic_cmp_evt(MM_API_CLI_SET_PROP, src_id, status);
}

/**
 ****************************************************************************************
 * @brief General handler for MM_API_CLI_TRANSITION command
 *
 * @param[in] src_id    Identifier of the task that has sent the command
 * @param[in] p_cmd     Pointer to command parameters
 ****************************************************************************************
 */
__STATIC void mm_api_msg_cli_transition_cmd_handler(uint16_t src_id, const mm_api_cli_transition_cmd_t *p_cmd)
{
    // Forward the command
    uint16_t status = mm_api_cli_transition(p_cmd->mdl_lid, p_cmd->dst, p_cmd->state_1, p_cmd->state_2,
                                            p_cmd->trans_time_ms, p_cmd->delay_ms, p_cmd->trans_info);

    // Send the command complete event
    mm_api_msg_send_basic_cmp_evt(MM_API_CLI_TRANSITION, src_id, status);
}
#endif //(BLE_MESH_MDL_CLIENT)

/**
 ****************************************************************************************
 * @brief General handler for MESH_MDL_API_CMD message
 *
 * @param[in] src_id    Identifier of the task that issue message
 * @param[in] p_cmd     Pointer to command parameters
 *
 * @return Status of message after execution of the handler (@see enum mal_msg_status)
 ****************************************************************************************
 */
__STATIC uint8_t mm_api_msg_handler_cmd(uint16_t src_id, const mm_api_cmd_t *p_cmd)
{
    uint8_t msg_status = MAL_MSG_FREE;


    // Forward the message to the appropriate model
    switch (p_cmd->cmd_code)
    {
#if (BLE_MESH_MDL_SERVER)
        case (MM_API_REGISTER_SERVER):
        {
            mm_api_msg_register_server_cmd_handler(src_id, (const mm_api_register_server_cmd_t *)p_cmd);
        } break;

#if (BLE_MESH_MDL_GENS)
        case (MM_API_REGISTER_SERVER_PROP):
        {
            mm_api_msg_register_server_prop_cmd_handler(src_id,
                    (const mm_api_register_server_prop_cmd_t *)p_cmd);
        } break;
#endif //(BLE_MESH_MDL_GENS)
#endif //(BLE_MESH_MDL_SERVER)

#if (BLE_MESH_MDL_CLIENT)
        case (MM_API_REGISTER_CLIENT):
        {
            mm_api_msg_register_client_cmd_handler(src_id, (const mm_api_register_client_cmd_t *)p_cmd);
        } break;

        case (MM_API_CLI_GET):
        {
            mm_api_msg_cli_get_cmd_handler(src_id, (const mm_api_cli_get_cmd_t *)p_cmd);
        } break;

        case (MM_API_CLI_SET):
        {
            mm_api_msg_cli_set_cmd_handler(src_id, (const mm_api_cli_set_cmd_t *)p_cmd);
        } break;

        case (MM_API_CLI_SET_LOCG):
        {
            mm_api_msg_cli_set_locg_cmd_handler(src_id, (const mm_api_cli_set_locg_cmd_t *)p_cmd);
        } break;

        case (MM_API_CLI_SET_LOCL):
        {
            mm_api_msg_cli_set_locl_cmd_handler(src_id, (const mm_api_cli_set_locl_cmd_t *)p_cmd);
        } break;

        case (MM_API_CLI_GET_PROP):
        {
            mm_api_msg_cli_get_prop_cmd_handler(src_id, (const mm_api_cli_get_prop_cmd_t *)p_cmd);
        } break;

        case (MM_API_CLI_SET_PROP):
        {
            mm_api_msg_cli_set_prop_cmd_handler(src_id, (const mm_api_cli_set_prop_cmd_t *)p_cmd);
        } break;

        case (MM_API_CLI_TRANSITION):
        {
            mm_api_msg_cli_transition_cmd_handler(src_id, (const mm_api_cli_transition_cmd_t *)p_cmd);
        } break;
#endif //(BLE_MESH_MDL_CLIENT)

#if (BLE_MESH_MDL_SERVER)
        case (MM_API_SRV_SET):
        {
            mm_api_msg_srv_set_cmd_handler(src_id, (const mm_api_srv_set_cmd_t *)p_cmd);
        } break;
#endif //(BLE_MESH_MDL_SERVER)

        default:
        {
            MESH_APP_PRINT_INFO("%s,default,cmd_code:%x\r\n", __func__, p_cmd->cmd_code);
        } break;
    }

    return (msg_status);
}

/**
 ****************************************************************************************
 * @brief General handler for MESH_MDL_API_CFM message
 *
 * @param[in] p_cfm    Pointer to confirmation parameters
 *
 * @return Status of message after execution of the handler (@see enum mal_msg_status)
 ****************************************************************************************
 */
__STATIC uint8_t mm_api_msg_handler_cfm(const mm_api_cfm_t *p_cfm)
{
    uint8_t msg_status = MAL_MSG_FREE;

    // Forward the message to the appropriate model
    switch (p_cfm->req_ind_code)
    {
#if (BLE_MESH_MDL_GENS)
        case (MM_API_SRV_BAT_REQ_IND):
        {
            mm_api_msg_srv_bat_cfm_handler((const mm_api_srv_bat_cfm_t *)p_cfm);
        } break;

        case (MM_API_SRV_LOCG_REQ_IND):
        {
            mm_api_msg_srv_locg_cfm_handler((const mm_api_srv_locg_cfm_t *)p_cfm);
        } break;

        case (MM_API_SRV_LOCL_REQ_IND):
        {
            mm_api_msg_srv_locl_cfm_handler((const mm_api_srv_locl_cfm_t *)p_cfm);
        } break;

        case (MM_API_SRV_PROP_GET_REQ_IND):
        case (MM_API_SRV_PROP_SET_REQ_IND):
        {
            mm_api_msg_srv_prop_cfm_handler((const mm_api_srv_prop_cfm_t *)p_cfm);
        } break;
#endif //(BLE_MESH_MDL_GENS)

        default:
        {
        } break;
    }

    return (msg_status);
}

/*
 * GLOBAL FUNCTIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Initialization of Mesh Model Message Application Programming Interface
 *
 * @param[in] reset     True means SW reset, False means task initialization
 * @param[in] p_env     Pointer to the environment structure
 * @param[in] p_cfg     Pointer to Mesh Model Configuration Structure provided by the Application
 *
 * @return Size of environment required for this module
 ****************************************************************************************
 */
uint16_t mm_api_msg_init(bool reset, void *p_env, const mm_cfg_t *p_cfg)
{
    // Set callback functions for asynchronous events
    mm_api_set(&mm_api_msg_cb);

    // No environment
    return (0);
}

/**
 ****************************************************************************************
 * @brief Return memory size needed for environment allocation of Mesh Model Message Application
 * Program Interface
 *
 * @param[in] p_cfg     Pointer to Mesh Model Configuration Structure provided by the Application
 *
 * @return Size of environment required for this module
 ****************************************************************************************
 */
uint16_t mm_api_msg_get_env_size(const mm_cfg_t *p_cfg)
{
    // No environment
    return (0);
}

/**
 ****************************************************************************************
 * @brief Handler for Mesh Model API messages
 *
 * @param[in] msg_id    Identifier of the message received
 * @param[in] src_id    Identifier of the task that issue message
 * @param[in] p_param   Message parameters
 *
 * @return Status of message after execution of the handler (@see enum mal_msg_status)
 ****************************************************************************************
 */
uint8_t mm_api_msg_handler(uint16_t msg_id, uint16_t src_id, const void *p_param)
{
    uint8_t msg_status = MAL_MSG_FREE;


    switch (msg_id)
    {
        // Mesh Model Command
        case (MESH_MDL_API_CMD):
        {
            msg_status = mm_api_msg_handler_cmd(src_id, (const mm_api_cmd_t *)p_param);
        } break;

        // Mesh Model Confirm
        case (MESH_MDL_API_CFM):
        {
            msg_status = mm_api_msg_handler_cfm((const mm_api_cfm_t *)p_param);
        } break;

        // Unknown message
        default:
        {
            //ASSERT_WARN(0, msg_id, src_id);
        } break;
    }

    return (msg_status);
}

#endif //(BLE_MESH_MSG_API)
/// @} MM_API_MSG
