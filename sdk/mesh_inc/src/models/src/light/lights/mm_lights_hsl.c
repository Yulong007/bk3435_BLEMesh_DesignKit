/**
 ****************************************************************************************
 * @file mm_lights_hsl.c
 *
 * @brief Mesh Model Light HSL Server Module
 *
 * Copyright (C) RivieraWaves 2017-2019
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup MM_LIGHTS_HSL
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "mm_lights_int.h"      // Mesh Model Lighting Module Internal Definitions

#if (BLE_MESH_MDL_LIGHTS_HSL)
/*
 * DEFINES
 ****************************************************************************************
 */

/// Validity of information provided to the Replay Manager
#define MM_LIGHTS_HSL_REPLAY_MS               (6000)

/*
 * ENUMERATIONS
 ****************************************************************************************
 */

/// Bit field content for status_info value present in environment for Light HSL Server model
/// 7              4                3                2               1       0
/// +--------------+----------------+----------------+---------------+-------+
/// |      RFU     | Wait Sat Trans | Wait Hue Trans | Wait LN Trans | Relay |
/// +--------------+----------------+----------------+---------------+-------+
enum mm_lights_hsl_sinfo_bf
{
    /// Relaying of sent status message is authorized
    MM_LIGHTS_HSL_SINFO_RELAY_POS = 0,
    MM_LIGHTS_HSL_SINFO_RELAY_BIT = CO_BIT(MM_LIGHTS_HSL_SINFO_RELAY_POS),

    /// Wait for start of transition for Light HSL Lightness state
    MM_LIGHTS_HSL_SINFO_WAIT_LN_POS = 1,
    MM_LIGHTS_HSL_SINFO_WAIT_LN_BIT = CO_BIT(MM_LIGHTS_HSL_SINFO_WAIT_LN_POS),

    /// Wait for start of transition for Light HSL Hue state
    MM_LIGHTS_HSL_SINFO_WAIT_HUE_POS = 2,
    MM_LIGHTS_HSL_SINFO_WAIT_HUE_BIT = CO_BIT(MM_LIGHTS_HSL_SINFO_WAIT_HUE_POS),

    /// Wait for start of transition for Light HSL Saturation state
    MM_LIGHTS_HSL_SINFO_WAIT_SAT_POS = 3,
    MM_LIGHTS_HSL_SINFO_WAIT_SAT_BIT = CO_BIT(MM_LIGHTS_HSL_SINFO_WAIT_SAT_POS),

    /// Wait for transition start (either Light HSL Lightness or Light HSL Hue or Light HSL
    /// Saturation)
    MM_LIGHTS_HSL_SINFO_WAIT_LSB = 1,
    MM_LIGHTS_HSL_SINFO_WAIT_MASK = 0x0E,
};

/// Bit field content for status_info value present in environment for Light HSL Hue Server
/// and Light HSL Saturation Server models
/// 7              2        1       0
/// +--------------+--------+-------+
/// |      RFU     | Direct | Relay |
/// +--------------+--------+-------+
enum mm_lights_hsl_sinfo_hs_bf
{
    /// Relaying of sent status message is authorized
    MM_LIGHTS_HSL_SINFO_HS_RELAY_POS = 0,
    MM_LIGHTS_HSL_SINFO_HS_RELAY_BIT = CO_BIT(MM_LIGHTS_HSL_SINFO_HS_RELAY_POS),

    /// Set message directly received on the model
    MM_LIGHTS_HSL_SINFO_HS_DIRECT_POS = 1,
    MM_LIGHTS_HSL_SINFO_HS_DIRECT_BIT = CO_BIT(MM_LIGHTS_HSL_SINFO_HS_DIRECT_POS),
};

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

typedef struct mm_lights_hsl_env mm_lights_hsl_env_t;
typedef struct mm_lights_hsls_env mm_lights_hsls_env_t;
typedef struct mm_lights_hslh_env mm_lights_hslh_env_t;
typedef struct mm_lights_hslsat_env mm_lights_hslsat_env_t;

/// Structure for Light HSL Server model environment
struct mm_lights_hsl_env
{
    /// Basic model environment - Must be first element in the structure - DO NOT MOVE
    mm_tb_state_mdl_env_t env;
    /// Timer for sending of publications
    mesh_tb_timer_t tmr_publi;
    /// Publication period in milliseconds
    uint32_t publi_period_ms;

    /// Environment for replay protection mechanism
    mm_tb_replay_mdl_env_t replay_env;

    /// Pointer to environment of associated Light HSL Hue model
    mm_lights_hslh_env_t *p_env_hslh;
    /// Pointer to environment of associated Light HSL Saturation model
    mm_lights_hslsat_env_t *p_env_hslsat;

    /// Delta value in case of move transition for Light HSL Lightness state value
    int16_t move_delta;
    /// Light HSL Lightness state value
    uint16_t ln;
    /// Target Light HSL Lightness state value
    uint16_t ln_tgt;

    /// Source address of set message that has triggered last or current transition
    uint16_t status_dst_addr;
    /// Application key local index to be used for transmission of Light HSL Status
    /// or Light HSL Linear Status message
    m_lid_t status_app_key_lid;
    /// Status information bitfield (@see enum mm_lights_hsl_sinfo_bf)
    uint8_t status_info;
};

/// Structure for Light HSL Setup Server model environment
struct mm_lights_hsls_env
{
    /// Basic model environment - Must be first element in the structure - DO NOT MOVE
    mm_tb_state_mdl_env_t env;

    /// Pointer to environment of associated Light HSL Server model
    mm_lights_hsl_env_t *p_env_hsl;
};

/// Structure for Light HSL Hue Server model environment
struct mm_lights_hslh_env
{
    /// Basic model environment - Must be first element in the structure - DO NOT MOVE
    mm_tb_state_mdl_env_t env;
    /// Timer for sending of publications
    mesh_tb_timer_t tmr_publi;
    /// Publication period in milliseconds
    uint32_t publi_period_ms;

    /// Pointer to environment of associated Light HSL model
    mm_lights_hsl_env_t *p_env_hsl;

    /// Delta value in case of move transition for Light HSL Hue state value
    int16_t move_delta;
    /// Light HSL Hue state value
    uint16_t hue;
    /// Target Light HSL Hue state value
    uint16_t hue_tgt;
    /// Light HSL Hue Min state value
    uint16_t hue_min;
    /// Light HSL Hue Max state value
    uint16_t hue_max;
    /// Light HSL Hue Default state value
    uint16_t hue_dflt;

    /// Source address of set message that has triggered last or current transition
    uint16_t status_dst_addr;
    /// Application key local index to be used for transmission of Light HSL Status
    /// or Light HSL Linear Status message
    m_lid_t status_app_key_lid;
    /// Status information bitfield (@see enum mm_lights_hsl_sinfo_hs_bf)
    uint8_t status_info;
};

/// Structure for Light HSL Saturation Server model environment
struct mm_lights_hslsat_env
{
    /// Basic model environment - Must be first element in the structure - DO NOT MOVE
    mm_tb_state_mdl_env_t env;
    /// Timer for sending of publications
    mesh_tb_timer_t tmr_publi;
    /// Publication period in milliseconds
    uint32_t publi_period_ms;

    /// Pointer to environment of associated Light HSL model
    mm_lights_hsl_env_t *p_env_hsl;

    /// Delta value in case of move transition for Light HSL Hue state value
    int16_t move_delta;
    /// Light HSL Saturation state value
    uint16_t sat;
    /// Target Light HSL Saturation state value
    uint16_t sat_tgt;
    /// Light HSL Saturation Min state value
    uint16_t sat_min;
    /// Light HSL Saturation Max state value
    uint16_t sat_max;
    /// Light HSL Saturation Default state value
    uint16_t sat_dflt;

    /// Source address of set message that has triggered last or current transition
    uint16_t status_dst_addr;
    /// Application key local index to be used for transmission of Light HSL Status
    /// or Light HSL Linear Status message
    m_lid_t status_app_key_lid;
    /// Status information bitfield (@see enum mm_lights_hsl_sinfo_hs_bf)
    uint8_t status_info;
};

/*
 * LOCAL FUNCTIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Prepare and send a Light HSL Status or Light HSL Target Status message.
 * Note that both messages have the same content
 *
 * @param[in] p_env_hsl         Pointer to Light HSL Server model environment
 * @param[in] p_route_env       Pointer to structure containing reception information provided
 * by Mesh Profile for received request message
 * @param[in] publish           Indicate if message is sent as a publication (true) or as
 * a response to a received request (false)
 * @param[in] target            True if Light HSL Target Status message, else false
 ****************************************************************************************
 */
__STATIC void mm_lights_hsl_send_status(mm_lights_hsl_env_t *p_env_hsl,
                                        mm_route_buf_env_t *p_route_env, bool publish, bool target)
{


    // Get environment for Light Lightness HSL Hue Server model
    mm_lights_hslh_env_t *p_env_hslh = p_env_hsl->p_env_hslh;
    // Get environment for Light Lightness HSL Saturation Server model
    mm_lights_hslsat_env_t *p_env_hslsat = p_env_hsl->p_env_hslsat;
    // Pointer to the buffer that will contain the message
    mesh_tb_buf_t *p_buf_status;
    // Remaining time
    uint8_t ln_rem_time, hue_rem_time, sat_rem_time;
    // Transition type
    uint8_t ln_trans_type, hue_trans_type, sat_trans_type;
    // Data length
    uint8_t data_length;
    uint16_t status;
    UNUSED(status);
    MESH_MODEL_PRINT_DEBUG("%s publish:%d,target:%d\r\n", __func__, publish, target);
    // Check if a transition has been started and deduce data length
    mm_tb_bind_get_trans_info(p_env_hsl->env.grp_lid, &ln_trans_type, &ln_rem_time);
    mm_tb_bind_get_trans_info(p_env_hslh->env.grp_lid, &hue_trans_type, &hue_rem_time);
    mm_tb_bind_get_trans_info(p_env_hslsat->env.grp_lid, &sat_trans_type, &sat_rem_time);

    // Deduce deduce data length
    data_length = ((ln_trans_type != MM_TRANS_TYPE_NONE)
                   || (hue_trans_type != MM_TRANS_TYPE_NONE)
                   || (sat_trans_type != MM_TRANS_TYPE_NONE))
                  ? MM_LIGHT_HSL_STATUS_LEN : MM_LIGHT_HSL_STATUS_MIN_LEN;

    if ((status = mm_route_buf_alloc(&p_buf_status, data_length)) == MESH_ERR_NO_ERROR)
    {
        // Get pointer to data
        uint8_t *p_data = MESH_TB_BUF_DATA(p_buf_status);
        // Get pointer to environment
        mm_route_buf_env_t *p_buf_env = (mm_route_buf_env_t *)&p_buf_status->env;
        // State value
        uint16_t state;
        // Remaining time
        uint8_t rem_time = 0;

        // Prepare environment
        if (p_route_env)
        {
            memcpy(p_buf_env, p_route_env, sizeof(mm_route_buf_env_t));
        }
        else if (!publish)
        {
            p_buf_env->app_key_lid = p_env_hsl->status_app_key_lid;
            p_buf_env->u_addr.dst = p_env_hsl->status_dst_addr;
            SETB(p_buf_env->info, MM_ROUTE_BUF_INFO_RELAY,
                 GETB(p_env_hsl->status_info, MM_LIGHTS_HSL_SINFO_RELAY));
        }

        SETB(p_buf_env->info, MM_ROUTE_BUF_INFO_RX, 0);
        SETB(p_buf_env->info, MM_ROUTE_BUF_INFO_PUBLISH, 0);
        p_buf_env->mdl_lid = p_env_hsl->env.mdl_lid;
        p_buf_env->opcode = (target) ? MM_MSG_LIGHT_HSL_TGT_STATUS
                            : MM_MSG_LIGHT_HSL_STATUS;

        // Fill the message
        if (target && (ln_trans_type != MM_TRANS_TYPE_NONE))
        {
            if (ln_trans_type == MM_TRANS_TYPE_MOVE)
            {
                state = (p_env_hsl->move_delta > 0)
                        ? mm_lights_ln_get(p_env_hsl->env.elmt_idx, MM_STATE_LIGHT_LN_RANGE_MAX)
                        : mm_lights_ln_get(p_env_hsl->env.elmt_idx, MM_STATE_LIGHT_LN_RANGE_MIN);
                rem_time = MM_TRANS_TIME_UNKNOWN;
            }
            else
            {
                state = p_env_hsl->ln_tgt;
            }
        }
        else
        {
            state = p_env_hsl->ln;
        }


        co_write16p(p_data + MM_LIGHT_HSL_STATUS_LIGHTNESS_POS, state);

        if (target && (hue_trans_type != MM_TRANS_TYPE_NONE))
        {
            if (hue_trans_type == MM_TRANS_TYPE_MOVE)
            {
                state = (p_env_hslh->move_delta > 0) ? p_env_hslh->hue_max : p_env_hslh->hue_min;
                rem_time = MM_TRANS_TIME_UNKNOWN;
            }
            else
            {
                state = p_env_hslh->hue_tgt;
            }
        }
        else
        {
            state = p_env_hslh->hue;
        }
        co_write16p(p_data + MM_LIGHT_HSL_STATUS_HUE_POS, state);

        if (target && (sat_trans_type != MM_TRANS_TYPE_NONE))
        {
            if (sat_trans_type == MM_TRANS_TYPE_MOVE)
            {
                state = (p_env_hslsat->move_delta > 0) ? p_env_hslsat->sat_max : p_env_hslsat->sat_min;
                rem_time = MM_TRANS_TIME_UNKNOWN;
            }
            else
            {
                state = p_env_hslsat->sat_tgt;
            }
        }
        else
        {
            state = p_env_hslsat->sat;
        }

        MESH_MODEL_PRINT_DEBUG("send_status p_env_hsl->ln :%d,p_env_hslh->hue:%d,p_env_hslsat->sat:%d\r\n", p_env_hsl->ln, p_env_hslh->hue, p_env_hslsat->sat);
        co_write16p(p_data + MM_LIGHT_HSL_STATUS_SAT_POS, state);

        if (data_length == MM_LIGHT_HSL_STATUS_LEN)
        {
            if (rem_time != MM_TRANS_TIME_UNKNOWN)
            {
                rem_time = co_max(co_max(ln_rem_time, hue_rem_time), sat_rem_time);
            }
            *(p_data + MM_LIGHT_HSL_STATUS_REM_TIME_POS) = rem_time;
        }

        // Send the message
        mm_route_send(p_buf_status);
    }
    else
    {
        MESH_MODEL_PRINT_ERR("%s  mm_route_buf_alloc error!!, status = 0x%x\r\n", __func__, status);
    }

}

/**
 ****************************************************************************************
 * @brief Prepare and send a Light HSL Hue Status message
 *
 * @param[in] p_env_hslsat       Pointer to Light HSL Hue Server model environment
 * @param[in] p_route_env        Pointer to structure containing reception information provided
 * by Mesh Profile for received request message
 * @param[in] publish            Indicate if message is sent as a publication (true) or as
 * a response to a received request (false)
 ****************************************************************************************
 */
__STATIC void mm_lights_hsl_send_status_hue(mm_lights_hslh_env_t *p_env_hslh,
        mm_route_buf_env_t *p_route_env, bool publish)
{
    // Pointer to the buffer that will contain the message
    mesh_tb_buf_t *p_buf_status;
    // Remaining time
    uint8_t rem_time;
    // Transition type
    uint8_t trans_type;
    // Data length
    uint8_t data_length;

    // Check if a transition has been started
    mm_tb_bind_get_trans_info(p_env_hslh->env.grp_lid, &trans_type, &rem_time);

    // Deduce deduce data length
    data_length = (trans_type != MM_TRANS_TYPE_NONE)
                  ? MM_LIGHT_HSL_HUE_STATUS_LEN : MM_LIGHT_HSL_HUE_STATUS_MIN_LEN;

    if (mm_route_buf_alloc(&p_buf_status, data_length) == MESH_ERR_NO_ERROR)
    {
        // Get pointer to data
        uint8_t *p_data = MESH_TB_BUF_DATA(p_buf_status);
        // Get pointer to environment
        mm_route_buf_env_t *p_buf_env = (mm_route_buf_env_t *)&p_buf_status->env;

        // Prepare environment
        if (p_route_env)
        {
            memcpy(p_buf_env, p_route_env, sizeof(mm_route_buf_env_t));
        }
        else if (!publish)
        {
            p_buf_env->app_key_lid = p_env_hslh->status_app_key_lid;
            p_buf_env->u_addr.dst = p_env_hslh->status_dst_addr;
            SETB(p_buf_env->info, MM_ROUTE_BUF_INFO_RELAY,
                 GETB(p_env_hslh->status_info, MM_LIGHTS_HSL_SINFO_RELAY));
        }

        SETB(p_buf_env->info, MM_ROUTE_BUF_INFO_RX, 0);
        SETB(p_buf_env->info, MM_ROUTE_BUF_INFO_PUBLISH, 0);
        p_buf_env->mdl_lid = p_env_hslh->env.mdl_lid;
        p_buf_env->opcode = MM_MSG_LIGHT_HSL_HUE_STATUS;

        // Fill the message
        co_write16p(p_data + MM_LIGHT_HSL_HUE_STATUS_HUE_POS, p_env_hslh->hue);

        if (data_length == MM_LIGHT_HSL_HUE_STATUS_LEN)
        {
            // Sent Target Light HSL Hue state values
            uint16_t hue_tgt;

            if (trans_type == MM_TRANS_TYPE_MOVE)
            {
                hue_tgt = (p_env_hslh->move_delta > 0) ? p_env_hslh->hue_max
                          : p_env_hslh->hue_min;
                rem_time = MM_TRANS_TIME_UNKNOWN;
            }
            else
            {
                hue_tgt = p_env_hslh->hue_tgt;
            }

            co_write16p(p_data + MM_LIGHT_HSL_HUE_STATUS_TGT_HUE_POS, hue_tgt);
            *(p_data + MM_LIGHT_HSL_HUE_STATUS_REM_TIME_POS) = rem_time;
        }

        // Send the message
        mm_route_send(p_buf_status);
    }
    else
    {
        MESH_MODEL_PRINT_WARN("%s, buffer alloc fail. \n", __func__);
    }
}

/**
 ****************************************************************************************
 * @brief Prepare and send a Light HSL Saturation Status message
 *
 * @param[in] p_env_hslsat       Pointer to Light HSL Saturation Server model environment
 * @param[in] p_route_env        Pointer to structure containing reception information provided
 * by Mesh Profile for received request message
 * @param[in] publish            Indicate if message is sent as a publication (true) or as
 * a response to a received request (false)
 ****************************************************************************************
 */
__STATIC void mm_lights_hsl_send_status_sat(mm_lights_hslsat_env_t *p_env_hslsat,
        mm_route_buf_env_t *p_route_env, bool publish)
{
    // Pointer to the buffer that will contain the message
    mesh_tb_buf_t *p_buf_status;
    // Remaining time
    uint8_t rem_time;
    // Transition type
    uint8_t trans_type;
    // Data length
    uint8_t data_length;

    // Check if a transition has been started
    mm_tb_bind_get_trans_info(p_env_hslsat->env.grp_lid, &trans_type, &rem_time);

    // Deduce deduce data length
    data_length = (trans_type != MM_TRANS_TYPE_NONE)
                  ? MM_LIGHT_HSL_SAT_STATUS_LEN : MM_LIGHT_HSL_SAT_STATUS_MIN_LEN;

    if (mm_route_buf_alloc(&p_buf_status, data_length) == MESH_ERR_NO_ERROR)
    {
        // Get pointer to data
        uint8_t *p_data = MESH_TB_BUF_DATA(p_buf_status);
        // Get pointer to environment
        mm_route_buf_env_t *p_buf_env = (mm_route_buf_env_t *)&p_buf_status->env;

        // Prepare environment
        if (p_route_env)
        {
            memcpy(p_buf_env, p_route_env, sizeof(mm_route_buf_env_t));
        }
        else if (!publish)
        {
            p_buf_env->app_key_lid = p_env_hslsat->status_app_key_lid;
            p_buf_env->u_addr.dst = p_env_hslsat->status_dst_addr;
            SETB(p_buf_env->info, MM_ROUTE_BUF_INFO_RELAY,
                 GETB(p_env_hslsat->status_info, MM_LIGHTS_HSL_SINFO_RELAY));
        }

        SETB(p_buf_env->info, MM_ROUTE_BUF_INFO_RX, 0);
        SETB(p_buf_env->info, MM_ROUTE_BUF_INFO_PUBLISH, 0);
        p_buf_env->mdl_lid = p_env_hslsat->env.mdl_lid;
        p_buf_env->opcode = MM_MSG_LIGHT_HSL_SAT_STATUS;

        // Fill the message
        co_write16p(p_data + MM_LIGHT_HSL_SAT_STATUS_SAT_POS, p_env_hslsat->sat);

        if (data_length == MM_LIGHT_HSL_SAT_STATUS_LEN)
        {
            // Sent Target Light HSL Saturation state values
            uint16_t sat_tgt;

            if (trans_type == MM_TRANS_TYPE_MOVE)
            {
                sat_tgt = (p_env_hslsat->move_delta > 0) ? p_env_hslsat->sat_max
                          : p_env_hslsat->sat_min;
                rem_time = MM_TRANS_TIME_UNKNOWN;
            }
            else
            {
                sat_tgt = p_env_hslsat->sat_tgt;
            }

            co_write16p(p_data + MM_LIGHT_HSL_SAT_STATUS_TGT_SAT_POS, sat_tgt);
            *(p_data + MM_LIGHT_HSL_SAT_STATUS_REM_TIME_POS) = rem_time;
        }

        // Send the message
        mm_route_send(p_buf_status);
    }
    else
    {
        MESH_MODEL_PRINT_WARN("%s, buffer alloc fail. \n", __func__);
    }
}

/**
 ****************************************************************************************
 * @brief Prepare and send a Light HSL Default Status message
 *
 * @param[in] p_env_hsl          Pointer to Light HSL Server model environment
 * @param[in] p_route_env       Pointer to structure containing reception information provided
 * by Mesh Profile for received request message
 ****************************************************************************************
 */
__STATIC void mm_lights_hsl_send_status_dflt(mm_lights_hsl_env_t *p_env_hsl,
        mm_route_buf_env_t *p_route_env)
{
    // Pointer to the buffer that will contain the message
    mesh_tb_buf_t *p_buf_status;
    // Allocate a new buffer for the publication
    uint8_t status = mm_route_buf_alloc_status(&p_buf_status,
                     MM_LIGHT_HSL_DFLT_STATUS_LEN, p_route_env);

    if (status == MESH_ERR_NO_ERROR)
    {
        // Get environment for Light Lightness HSL Hue Server model
        mm_lights_hslh_env_t *p_env_hslh = p_env_hsl->p_env_hslh;
        // Get environment for Light Lightness HSL Saturation Server model
        mm_lights_hslsat_env_t *p_env_hslsat = p_env_hsl->p_env_hslsat;
        // Get pointer to data
        uint8_t *p_data = MESH_TB_BUF_DATA(p_buf_status);
        // Get pointer to environment
        mm_route_buf_env_t *p_buf_env = (mm_route_buf_env_t *)&p_buf_status->env;

        // Prepare environment
        p_buf_env->mdl_lid = p_env_hsl->env.mdl_lid;
        p_buf_env->opcode = MM_MSG_LIGHT_HSL_DFLT_STATUS;

        // Fill the message
        co_write16p(p_data + MM_LIGHT_HSL_DFLT_STATUS_LIGHTNESS_POS,
                    mm_lights_ln_get(p_env_hsl->env.elmt_idx, MM_STATE_LIGHT_LN_DFLT));
        co_write16p(p_data + MM_LIGHT_HSL_DFLT_STATUS_HUE_POS, p_env_hslh->hue_dflt);
        co_write16p(p_data + MM_LIGHT_HSL_DFLT_STATUS_SAT_POS, p_env_hslsat->sat_dflt);

        // Send the message
        mm_route_send(p_buf_status);
    }
    else
    {
        MESH_MODEL_PRINT_WARN("%s, buffer alloc fail. \n", __func__);
    }
}

/**
 ****************************************************************************************
 * @brief Prepare and send a Light HSL Range Status message
 *
 * @param[in] p_env_hsl         Pointer to Light HSL Server model environment
 * @param[in] p_route_env       Pointer to structure containing reception information provided
 * by Mesh Profile for received request message
 * @param[in] status            Status sent in the Light HSL Status message
 * (@see enum mm_status)
 ****************************************************************************************
 */
__STATIC void mm_lights_hsl_send_status_range(mm_lights_hsl_env_t *p_env_hsl,
        mm_route_buf_env_t *p_route_env,
        uint8_t status)
{
    // Pointer to the buffer that will contain the message
    mesh_tb_buf_t *p_buf_status;
    // Allocate a new buffer for the publication
    uint16_t alloc_status = mm_route_buf_alloc_status(&p_buf_status,
                            MM_LIGHT_HSL_RANGE_STATUS_LEN, p_route_env);

    if (alloc_status == MESH_ERR_NO_ERROR)
    {
        // Get environment for Light Lightness HSL Hue Server model
        mm_lights_hslh_env_t *p_env_hslh = p_env_hsl->p_env_hslh;
        // Get environment for Light Lightness HSL Saturation Server model
        mm_lights_hslsat_env_t *p_env_hslsat = p_env_hsl->p_env_hslsat;
        // Get pointer to data
        uint8_t *p_data = MESH_TB_BUF_DATA(p_buf_status);
        // Get pointer to environment
        mm_route_buf_env_t *p_buf_env = (mm_route_buf_env_t *)&p_buf_status->env;

        // Prepare environment
        p_buf_env->mdl_lid = p_env_hsl->env.mdl_lid;
        p_buf_env->opcode = MM_MSG_LIGHT_HSL_RANGE_STATUS;

        // Fill the message
        *(p_data + MM_LIGHT_HSL_RANGE_STATUS_CODE_POS) = status;
        co_write16p(p_data + MM_LIGHT_HSL_RANGE_STATUS_HUE_MIN_POS, p_env_hslh->hue_min);
        co_write16p(p_data + MM_LIGHT_HSL_RANGE_STATUS_HUE_MAX_POS, p_env_hslh->hue_max);
        co_write16p(p_data + MM_LIGHT_HSL_RANGE_STATUS_SAT_MIN_POS, p_env_hslsat->sat_min);
        co_write16p(p_data + MM_LIGHT_HSL_RANGE_STATUS_SAT_MAX_POS, p_env_hslsat->sat_max);

        // Send the message
        mm_route_send(p_buf_status);
    }
    else
    {
        MESH_MODEL_PRINT_WARN("%s, buffer alloc fail. \n", __func__);
    }
}

/**
 ****************************************************************************************
 * @brief Publish Light HSL state value if sending of publications is enabled
 *
 * @param[in] p_env_hsl         Pointer to Light HSL Server model environment
 * @param[in] target            Publish current state or target state
 ****************************************************************************************
 */
__STATIC void mm_lights_hsl_publish(mm_lights_hsl_env_t *p_env_hsl, bool target)
{
    // Check if sending of publication is enabled
    if (GETB(p_env_hsl->env.info, MM_TB_STATE_MDL_INFO_PUBLI))
    {
        mm_lights_hsl_send_status(p_env_hsl, NULL, true, target);
    }
}

/**
 ****************************************************************************************
 * @brief Publish Light HSL Hue or Light HSL Saturation state value if sending of publications
 * is enabled
 *
 * @param[in] p_env         Pointer to Light HSL Hue/Saturation Server model environment
 ****************************************************************************************
 */
__STATIC void mm_lights_hsl_publish_hue_sat(mm_tb_state_mdl_env_t *p_env)
{
    // Check if sending of publication is enabled
    if (GETB(p_env->info, MM_TB_STATE_MDL_INFO_PUBLI))
    {
        if (p_env->mdl_id == MM_ID_LIGHTS_HSLH)
        {
            mm_lights_hsl_send_status_hue((mm_lights_hslh_env_t *)p_env, NULL, true);
        }
        else     // (p_env->mdl_id == MM_ID_LIGHTS_HSLSAT)
        {
            mm_lights_hsl_send_status_sat((mm_lights_hslsat_env_t *)p_env, NULL, true);
        }
    }
}

/**
 ****************************************************************************************
 * @brief Check if a Light HSL Status message must be sent and send it if it is the case
 *
 * @param[in] p_env_hsl        Pointer to Light HSL Server model environment
 ****************************************************************************************
 */
__STATIC void mm_lights_hsl_check_status_rsp(mm_lights_hsl_env_t *p_env_hsl)
{
    if ((p_env_hsl->status_dst_addr != MESH_UNASSIGNED_ADDR)
            && !GETF(p_env_hsl->status_info, MM_LIGHTS_HSL_SINFO_WAIT))
    {
        // Send a response to the node that has required the transition
        mm_lights_hsl_send_status(p_env_hsl, NULL, false, false);

        p_env_hsl->status_dst_addr = MESH_UNASSIGNED_ADDR;
    }
}

/**
 ****************************************************************************************
 * @brief Check if a Light HSL Hue Status message or a Light HSL Saturation Status message
 * must be sent and send it if it is the case
 *
 * @param[in] p_env        Pointer to Light HSL Hue/Saturation Server model environment
 ****************************************************************************************
 */
__STATIC void mm_lights_hsl_check_status_rsp_hue_sat(mm_tb_state_mdl_env_t *p_env)
{
    if (p_env->mdl_id == MM_ID_LIGHTS_HSLH)
    {
        // Get environment allocated for Light HSL Hue Server model
        mm_lights_hslh_env_t *p_env_hslh = (mm_lights_hslh_env_t *)p_env;

        if (GETB(p_env_hslh->status_info, MM_LIGHTS_HSL_SINFO_HS_DIRECT))
        {
            if (p_env_hslh->status_dst_addr != MESH_UNASSIGNED_ADDR)
            {
                // Send a response to the node that has required the transition
                mm_lights_hsl_send_status_hue(p_env_hslh, NULL, false);

                SETB(p_env_hslh->status_info, MM_LIGHTS_HSL_SINFO_HS_DIRECT, 0);
                p_env_hslh->status_dst_addr = MESH_UNASSIGNED_ADDR;
            }
        }
        else
        {
            mm_lights_hsl_check_status_rsp(p_env_hslh->p_env_hsl);
        }
    }
    else
    {
        // Get environment allocated for Light HSL Saturation Server model
        mm_lights_hslsat_env_t *p_env_hslsat = (mm_lights_hslsat_env_t *)p_env;

        if (GETB(p_env_hslsat->status_info, MM_LIGHTS_HSL_SINFO_HS_DIRECT))
        {
            if (p_env_hslsat->status_dst_addr != MESH_UNASSIGNED_ADDR)
            {
                // Send a response to the node that has required the transition
                mm_lights_hsl_send_status_sat(p_env_hslsat, NULL, false);

                SETB(p_env_hslsat->status_info, MM_LIGHTS_HSL_SINFO_HS_DIRECT, 0);
                p_env_hslsat->status_dst_addr = MESH_UNASSIGNED_ADDR;
            }
        }
        else
        {
            mm_lights_hsl_check_status_rsp(p_env_hslsat->p_env_hsl);
        }
    }
}

/*
 * MESSAGE HANDLERS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Handler for Light HSL Set/Set Unacknowledged message
 *
 * @param[in] p_env         Pointer to environment of model for which message has been received
 * @param[in] p_buf         Pointer to buffer containing the received message
 * @param[in] p_route_env   Pointer to routing information for the received buffer
 ****************************************************************************************
 */
__STATIC void mm_lights_hsl_handler_set(mm_lights_hsl_env_t *p_env_hsl, mesh_tb_buf_t *p_buf,
                                        mm_route_buf_env_t *p_route_env)
{
    do
    {
        // Get environment for Light Lightness HSL Hue Server model
        mm_lights_hslh_env_t *p_env_hslh = p_env_hsl->p_env_hslh;
        // Get environment for Light Lightness HSL Saturation Server model
        mm_lights_hslsat_env_t *p_env_hslsat = p_env_hsl->p_env_hslsat;
        // Check if a status message must be sent
        bool send_status = (p_route_env->opcode == MM_MSG_LIGHT_HSL_SET);
        // Get pointer to data
        uint8_t *p_data = MESH_TB_BUF_DATA(p_buf);
        // TID value
        uint8_t tid = *(p_data + MM_LIGHT_CTL_SET_TID_POS);
        // Light HSL Lightness, Light HSL Hue and Light HSL Saturation value
        uint16_t ln, hue, sat;
        // Transition time
        uint8_t trans_time;
        // Delay
        uint8_t delay;

        MESH_MODEL_PRINT_DEBUG("%s\r\n", __func__);

        // Check if request can be processed
        if ((p_env_hsl->status_dst_addr != MESH_UNASSIGNED_ADDR)
                || (p_env_hslh->status_dst_addr != MESH_UNASSIGNED_ADDR)
                || (p_env_hslsat->status_dst_addr != MESH_UNASSIGNED_ADDR)
                || (mm_tb_replay_is_retx(&p_env_hsl->replay_env, p_route_env->u_addr.src, tid)))
        {
            // Send a Light HSL Status message
            if (send_status)
            {
                MESH_MODEL_PRINT_DEBUG("hsl_send_status 1\r\n");
                mm_lights_hsl_send_status(p_env_hsl, p_route_env, false, false);
            }
            break;
        }

        // Extract and check optional parameters if present
        if (p_buf->data_len == MM_LIGHT_HSL_SET_LEN)
        {
            trans_time = (uint16_t)(*(p_data + MM_LIGHT_HSL_SET_TRANS_TIME_POS));

            // Check received value
            if (GETF(trans_time, MM_TRANS_TIME_STEP_NB) > MM_TRANS_TIME_NB_STEPS_MAX)
            {
                // Drop the message
                break;
            }

            delay = *(p_data + MM_LIGHT_HSL_SET_DELAY_POS);
        }
        else
        {
            trans_time = MM_TRANS_TIME_UNKNOWN;
            delay = 0;
        }

        ln = co_read16p(p_data + MM_LIGHT_HSL_SET_LIGHTNESS_POS);
        hue = co_read16p(p_data + MM_LIGHT_HSL_SET_HUE_POS);
        sat = co_read16p(p_data + MM_LIGHT_HSL_SET_SAT_POS);



        // Ensure that Light HSL Hue state value is between Light HSL Hue Range Min and Max values
        if (hue > p_env_hslh->hue_max)
        {
            hue = p_env_hslh->hue_max;
        }
        else if (hue < p_env_hslh->hue_min)
        {
            hue = p_env_hslh->hue_min;
        }

        // Ensure that Light HSL Saturation state value is between Light HSL Saturation Range Min and
        // Max values
        if (sat > p_env_hslsat->sat_max)
        {
            sat = p_env_hslsat->sat_max;
        }
        else if (sat < p_env_hslsat->sat_min)
        {
            sat = p_env_hslsat->sat_min;
        }


        MESH_MODEL_PRINT_DEBUG("ln :%d,p_env_hsl->ln:%d\r\n", ln, p_env_hsl->ln);
        MESH_MODEL_PRINT_DEBUG("hue :%d,p_env_hslh->hue:%d\r\n", hue, p_env_hslh->hue);
        MESH_MODEL_PRINT_DEBUG("sat :%d,p_env_hslsat->sat:%d\r\n", sat, p_env_hslsat->sat);

        // Check if at least one of the states is modified
        if ((ln == p_env_hsl->ln)
                && (hue == p_env_hslh->hue)
                && (sat == p_env_hslsat->sat))
        {
            // Send a Light HSL Status message
            if (send_status)
            {
                mm_lights_hsl_send_status(p_env_hsl, p_route_env, false, false);
            }
            break;
        };

        if (send_status)
        {
            // Keep information for transmission of status
            p_env_hsl->status_dst_addr = p_route_env->u_addr.src;
            p_env_hsl->status_app_key_lid = p_route_env->app_key_lid;
            SETB(p_env_hsl->status_info, MM_LIGHTS_HSL_SINFO_RELAY,
                 GETB(p_route_env->info, MM_ROUTE_BUF_INFO_RELAY));
#if 0
            SETB(p_env_hsl->status_info, MM_LIGHTS_HSL_SINFO_WAIT_LN, (ln != p_env_hsl->ln));
            SETB(p_env_hsl->status_info, MM_LIGHTS_HSL_SINFO_WAIT_HUE, (hue != p_env_hslh->hue));
            SETB(p_env_hsl->status_info, MM_LIGHTS_HSL_SINFO_WAIT_SAT, (sat != p_env_hslsat->sat));
#endif
            SETB(p_env_hsl->status_info, MM_LIGHTS_HSL_SINFO_WAIT_LN, 1);
            SETB(p_env_hsl->status_info, MM_LIGHTS_HSL_SINFO_WAIT_HUE, 1);
            SETB(p_env_hsl->status_info, MM_LIGHTS_HSL_SINFO_WAIT_SAT, 1);
        }


        // if (hue != p_env_hslh->hue)
        {
            // Update target state
            p_env_hslh->hue_tgt = hue;

            // Inform the Binding Manager about new transition
            mm_tb_bind_trans_new(p_env_hslh->env.grp_lid, MM_TRANS_TYPE_CLASSIC, trans_time, delay);
        }

        //  if (sat != p_env_hslsat->sat)
        {
            // Update target state
            p_env_hslsat->sat_tgt = sat;

            // Inform the Binding Manager about new transition
            mm_tb_bind_trans_new(p_env_hslsat->env.grp_lid, MM_TRANS_TYPE_CLASSIC, trans_time, delay);
        }

        // if (ln != p_env_hsl->ln)
        {
            // Update target state
            p_env_hsl->ln_tgt = ln;

            // Inform the Binding Manager about new transition
            mm_tb_bind_trans_new(p_env_hsl->env.grp_lid, MM_TRANS_TYPE_CLASSIC, trans_time, delay);
        }



    }
    while (0);
}

/**
 ****************************************************************************************
 * @brief Handler for Light HSL Hue Set/Set Unacknowledged message
 *
 * @param[in] p_env         Pointer to environment of model for which message has been received
 * @param[in] p_buf         Pointer to buffer containing the received message
 * @param[in] p_route_env   Pointer to routing information for the received buffer
 ****************************************************************************************
 */
__STATIC void mm_lights_hsl_handler_set_hue(mm_lights_hslh_env_t *p_env_hslh, mesh_tb_buf_t *p_buf,
        mm_route_buf_env_t *p_route_env)
{
    do
    {
        // Get pointer to environment for Light HSL Server model
        mm_lights_hsl_env_t *p_env_hsl = p_env_hslh->p_env_hsl;
        // Check if a status message must be sent
        bool send_status = (p_route_env->opcode == MM_MSG_LIGHT_HSL_HUE_SET);
        // Get pointer to data
        uint8_t *p_data = MESH_TB_BUF_DATA(p_buf);
        // TID value
        uint8_t tid = *(p_data + MM_LIGHT_HSL_HUE_SET_TID_POS);
        // Received Light CTL Hue state values
        uint16_t hue;
        // Transition time
        uint8_t trans_time;
        // Delay
        uint8_t delay = 0;

        // Check if request can be processed
        if ((p_env_hsl->status_dst_addr != MESH_UNASSIGNED_ADDR)
                || (mm_tb_replay_is_retx(&p_env_hsl->replay_env, p_route_env->u_addr.src, tid)))
        {
            // Send a Light HSL Hue Status message
            if (send_status)
            {
                mm_lights_hsl_send_status_hue(p_env_hslh, p_route_env, false);
            }
            break;
        }

        // Extract and check optional parameters if present
        if (p_buf->data_len == MM_MSG_LIGHT_HSL_HUE_SET)
        {
            trans_time = (uint16_t)(*(p_data + MM_LIGHT_HSL_HUE_SET_TRANS_TIME_POS));

            // Check received value
            if (GETF(trans_time, MM_TRANS_TIME_STEP_NB) > MM_TRANS_TIME_NB_STEPS_MAX)
            {
                // Drop the message
                break;
            }

            delay = *(p_data + MM_LIGHT_HSL_HUE_SET_DELAY_POS);
        }
        else
        {
            delay = 0;
            trans_time = MM_TRANS_TIME_UNKNOWN;
        }


        hue = co_read16p(p_data + MM_LIGHT_HSL_HUE_SET_HUE_POS);

        // Ensure that Light HSL Saturation state value is between Light HSL Saturation Range
        // Min and Max values
        if (hue > p_env_hslh->hue_max)
        {
            hue = p_env_hslh->hue_max;
        }
        else if (hue < p_env_hslh->hue_min)
        {
            hue = p_env_hslh->hue_min;
        }

        // Check if state is modified
        if (hue == p_env_hslh->hue)
        {
            // Send a Light HSL Hue Status message
            if (send_status)
            {
                mm_lights_hsl_send_status_hue(p_env_hslh, p_route_env, false);
            }
            break;
        };

        if (send_status)
        {
            // Keep information for transmission of status
            p_env_hslh->status_dst_addr = p_route_env->u_addr.src;
            p_env_hslh->status_app_key_lid = p_route_env->app_key_lid;
            SETB(p_env_hslh->status_info, MM_LIGHTS_HSL_SINFO_HS_RELAY,
                 GETB(p_route_env->info, MM_ROUTE_BUF_INFO_RELAY));
            SETB(p_env_hslh->status_info, MM_LIGHTS_HSL_SINFO_HS_DIRECT, 1);
        }

        // Update target state
        p_env_hslh->hue_tgt = hue;

        // Inform the Binding Manager about new transition
        mm_tb_bind_trans_new(p_env_hslh->env.grp_lid, MM_TRANS_TYPE_CLASSIC, trans_time, delay);
    }
    while (0);
}

/**
 ****************************************************************************************
 * @brief Handler for Light HSL Saturation Set/Set Unacknowledged message
 *
 * @param[in] p_env         Pointer to environment of model for which message has been received
 * @param[in] p_buf         Pointer to buffer containing the received message
 * @param[in] p_route_env   Pointer to routing information for the received buffer
 ****************************************************************************************
 */
__STATIC void mm_lights_hsl_handler_set_sat(mm_lights_hslsat_env_t *p_env_hslsat, mesh_tb_buf_t *p_buf,
        mm_route_buf_env_t *p_route_env)
{
    do
    {
        // Get pointer to environment for Light HSL Server model
        mm_lights_hsl_env_t *p_env_hsl = p_env_hslsat->p_env_hsl;
        // Check if a status message must be sent
        bool send_status = (p_route_env->opcode == MM_MSG_LIGHT_HSL_SAT_SET);
        // Get pointer to data
        uint8_t *p_data = MESH_TB_BUF_DATA(p_buf);
        // TID value
        uint8_t tid = *(p_data + MM_LIGHT_HSL_SAT_SET_TID_POS);;
        // Received Light CTL Saturation state values
        uint16_t sat;
        // Transition time
        uint8_t trans_time;
        // Delay
        uint8_t delay;

        // Check if request can be processed
        if ((p_env_hsl->status_dst_addr != MESH_UNASSIGNED_ADDR)
                || (mm_tb_replay_is_retx(&p_env_hsl->replay_env, p_route_env->u_addr.src, tid)))
        {
            // Send a Light HSL Saturation Status message
            if (send_status)
            {
                mm_lights_hsl_send_status_sat(p_env_hslsat, p_route_env, false);
            }
            break;
        }

        // Extract and check optional parameters if present
        if (p_buf->data_len == MM_MSG_LIGHT_HSL_SAT_SET)
        {
            trans_time = (uint16_t)(*(p_data + MM_LIGHT_HSL_SAT_SET_TRANS_TIME_POS));

            // Check received value
            if (GETF(trans_time, MM_TRANS_TIME_STEP_NB) > MM_TRANS_TIME_NB_STEPS_MAX)
            {
                // Drop the message
                break;
            }

            delay = *(p_data + MM_LIGHT_HSL_SAT_SET_DELAY_POS);
        }
        else
        {
            delay = 0;
            trans_time = MM_TRANS_TIME_UNKNOWN;
        }

        sat = co_read16p(p_data + MM_LIGHT_HSL_SAT_SET_SAT_POS);

        // Ensure that Light HSL Saturation state value is between Light HSL Saturation Range
        // Min and Max values
        if (sat > p_env_hslsat->sat_max)
        {
            sat = p_env_hslsat->sat_max;
        }
        else if (sat < p_env_hslsat->sat_min)
        {
            sat = p_env_hslsat->sat_min;
        }

        // Check if state is modified
        if (sat == p_env_hslsat->sat)
        {
            // Send a Light HSL Saturation Status message
            if (send_status)
            {
                mm_lights_hsl_send_status_sat(p_env_hslsat, p_route_env, false);
            }
            break;
        };

        if (send_status)
        {
            // Keep information for transmission of status
            p_env_hslsat->status_dst_addr = p_route_env->u_addr.src;
            p_env_hslsat->status_app_key_lid = p_route_env->app_key_lid;
            SETB(p_env_hslsat->status_info, MM_LIGHTS_HSL_SINFO_HS_RELAY,
                 GETB(p_route_env->info, MM_ROUTE_BUF_INFO_RELAY));
            SETB(p_env_hslsat->status_info, MM_LIGHTS_HSL_SINFO_HS_DIRECT, 1);
        }

        // Update target state
        p_env_hslsat->sat_tgt = sat;

        // Inform the Binding Manager about new transition
        mm_tb_bind_trans_new(p_env_hslsat->env.grp_lid, MM_TRANS_TYPE_CLASSIC, trans_time, delay);
    }
    while (0);
}

/**
 ****************************************************************************************
 * @brief Handler for Light HSL Default Set/Set Unacknowledged message
 *
 * @param[in] p_env         Pointer to environment of model for which message has been received
 * @param[in] p_buf         Pointer to buffer containing the received message
 * @param[in] p_route_env   Pointer to routing information for the received buffer
 ****************************************************************************************
 */
__STATIC void mm_lights_hsl_handler_set_dflt(mm_lights_hsl_env_t *p_env_hsl,
        mesh_tb_buf_t *p_buf, mm_route_buf_env_t *p_route_env)
{
    // Get pointer to data
    uint8_t *p_data = MESH_TB_BUF_DATA(p_buf);
    // Get environment for Light Lightness HSL Hue Server model
    mm_lights_hslh_env_t *p_env_hslh = p_env_hsl->p_env_hslh;
    // Get environment for Light Lightness HSL Saturation Server model
    mm_lights_hslsat_env_t *p_env_hslsat = p_env_hsl->p_env_hslsat;
    // Extract Light HSL Lightness Default state value
    uint16_t ln_dflt = co_read16p(p_data + MM_LIGHT_HSL_DFLT_SET_LIGHTNESS_POS);
    // Extract Light HSL Hue Default state value
    uint16_t hue_dflt = co_read16p(p_data + MM_LIGHT_HSL_DFLT_SET_HUE_POS);
    // Extract Light HSL Saturation Default state value
    uint16_t sat_dflt = co_read16p(p_data + MM_LIGHT_HSL_DFLT_SET_SAT_POS);

    // Inform Light Lightness Server model about received Light Lightness Default state value
    mm_lights_ln_set_dflt(p_env_hsl->env.elmt_idx, ln_dflt);

    if (hue_dflt != p_env_hslh->hue_dflt)
    {
        // Keep received value
        p_env_hslh->hue_dflt = hue_dflt;

        // Inform application about received value
        mm_api_send_srv_state_upd_ind(MM_STATE_LIGHT_HSL_DFLT_HUE,
                                      p_env_hslh->env.elmt_idx, hue_dflt, 0);
    }

    if (sat_dflt != p_env_hslsat->sat_dflt)
    {
        // Keep received value
        p_env_hslsat->sat_dflt = sat_dflt;

        // Inform application about received value
        mm_api_send_srv_state_upd_ind(MM_STATE_LIGHT_HSL_DFLT_SAT,
                                      p_env_hslsat->env.elmt_idx, sat_dflt, 0);
    }

    // If needed, send a Light HSL Default Status message to the requester
    if (p_route_env->opcode == MM_MSG_LIGHT_HSL_DFLT_SET)
    {
        mm_lights_hsl_send_status_dflt(p_env_hsl, p_route_env);
    }
}

/**
 ****************************************************************************************
 * @brief Handler for Light HSL Range Set/Set Unacknowledged message
 *
 * @param[in] p_env         Pointer to environment of model for which message has been received
 * @param[in] p_buf         Pointer to buffer containing the received message
 * @param[in] p_route_env   Pointer to routing information for the received buffer
 ****************************************************************************************
 */
__STATIC void mm_lights_hsl_handler_set_range(mm_lights_hsl_env_t *p_env_hsl,
        mesh_tb_buf_t *p_buf, mm_route_buf_env_t *p_route_env)
{
    do
    {
        // Get pointer to data
        uint8_t *p_data = MESH_TB_BUF_DATA(p_buf);
        // Get environment for Light Lightness HSL Hue Server model
        mm_lights_hslh_env_t *p_env_hslh = p_env_hsl->p_env_hslh;
        // Get environment for Light Lightness HSL Saturation Server model
        mm_lights_hslsat_env_t *p_env_hslsat = p_env_hsl->p_env_hslsat;
        // Extract Light HSL Hue Min state value
        uint16_t hue_min = co_read16p(p_data + MM_LIGHT_HSL_RANGE_SET_HUE_MIN_POS);
        // Extract Light HSL Hue Max state value
        uint16_t hue_max = co_read16p(p_data + MM_LIGHT_HSL_RANGE_SET_HUE_MAX_POS);
        // Extract Light HSL Saturation Min state value
        uint16_t sat_min = co_read16p(p_data + MM_LIGHT_HSL_RANGE_SET_SAT_MIN_POS);
        // Extract Light HSL Saturation Max state value
        uint16_t sat_max = co_read16p(p_data + MM_LIGHT_HSL_RANGE_SET_SAT_MAX_POS);

        // Check provided values
        if ((hue_min > hue_max)
                || (sat_min > sat_max))
        {
            // Drop the message
            break;
        }

        if ((p_env_hslh->hue_min != hue_min)
                || (p_env_hslh->hue_max != hue_max))
        {
            p_env_hslh->hue_min = hue_min;
            p_env_hslh->hue_max = hue_max;

            // Inform application about received value
            mm_api_send_srv_state_upd_ind(MM_STATE_LIGHT_HSL_RANGE_HUE, p_env_hslh->env.elmt_idx,
                                          (uint32_t)hue_min | ((uint32_t)hue_max << 16), 0);
        }

        if ((p_env_hslsat->sat_min != sat_min)
                || (p_env_hslsat->sat_max != sat_max))
        {
            p_env_hslsat->sat_min = sat_min;
            p_env_hslsat->sat_max = sat_max;

            // Inform application about received value
            mm_api_send_srv_state_upd_ind(MM_STATE_LIGHT_HSL_RANGE_SAT, p_env_hslsat->env.elmt_idx,
                                          (uint32_t)sat_min | ((uint32_t)sat_max << 16), 0);
        }

        // If needed, send a Light HSL Range Status message to the requester
        if (p_route_env->opcode == MM_MSG_LIGHT_HSL_RANGE_SET)
        {
            mm_lights_hsl_send_status_range(p_env_hsl, p_route_env, MM_STATUS_SUCCESS);
        }
    }
    while (0);
}

/*
 * INTERNAL CALLBACK FUNCTIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Callback function called when timer monitoring publication duration for
 * Light HSL Server model or for Light HSL Hue Server model or for Light HSL Saturation
 * Server model expires
 *
 * @param[in] p_env     Pointer to model environment for Light HSL Server or Light HSL
 * Hue Server or Light HSL Saturation Server model
 ****************************************************************************************
 */
__STATIC void mm_lights_hsl_cb_tmr_publi(void *p_env)
{
    // Get allocated environment
    mm_tb_state_mdl_publi_env_t *p_env_publi = (mm_tb_state_mdl_publi_env_t *)p_env;

    if (p_env_publi->publi_period_ms)
    {
        if (p_env_publi->env.mdl_id == MM_ID_LIGHTS_HSL)
        {
            // Publish a Light HSL Status message
            mm_lights_hsl_publish((mm_lights_hsl_env_t *)p_env, false);
        }
        else
        {
            // Publish a Light HSL Hue Status message
            mm_lights_hsl_publish_hue_sat(&p_env_publi->env);
        }

        // Restart the timer
        mesh_tb_timer_set(&p_env_publi->tmr_publi, p_env_publi->publi_period_ms);
    }
}

/**
 ****************************************************************************************
 * @brief Inform Light HSL Server model about a received message
 *
 * @param[in] p_env         Pointer to the environment allocated for the Light HSL
 * Server model
 * @param[in] p_buf         Pointer to the buffer containing the received message
 * @param[in] p_route_env   Pointer to structure containing reception parameters provided
 * by the Mesh Profile block
 ****************************************************************************************
 */
__STATIC void mm_lights_hsl_cb_rx(mm_tb_state_mdl_env_t *p_env, mesh_tb_buf_t *p_buf,
                                  mm_route_buf_env_t *p_route_env)
{
    do
    {
        mm_lights_hsl_env_t *p_env_hsl;

        {
            MESH_MODEL_PRINT_DEBUG("%s opcode :0x%x\r\n", __func__, p_route_env->opcode);
            uint8_t *p_data = MESH_TB_BUF_DATA(p_buf);
            MESH_MODEL_PRINT_DEBUG("###############################\r\n");
            MESH_MODEL_PRINT_DEBUG("data: %s \n", mesh_buffer_to_hex(p_data, p_buf->data_len));
            MESH_MODEL_PRINT_DEBUG("###############################\r\n");
        }

        if (p_env->mdl_id == MM_ID_LIGHTS_HSL)
        {
            // Get Environment for Light HSL Server model
            p_env_hsl = (mm_lights_hsl_env_t *)p_env;

            switch (p_route_env->opcode)
            {
                case (MM_MSG_LIGHT_HSL_GET):
                case (MM_MSG_LIGHT_HSL_TGT_GET):
                {
                    // Send a Light HSL Status message
                    mm_lights_hsl_send_status(p_env_hsl, p_route_env, false,
                                              (p_route_env->opcode == MM_MSG_LIGHT_HSL_TGT_GET));
                } break;

                case (MM_MSG_LIGHT_HSL_SET):
                case (MM_MSG_LIGHT_HSL_SET_UNACK):
                {
                    // Handle the message
                    mm_lights_hsl_handler_set(p_env_hsl, p_buf, p_route_env);
                } break;

                case (MM_MSG_LIGHT_HSL_DFLT_GET):
                {
                    // Send a Light HSL Default Status message
                    mm_lights_hsl_send_status_dflt(p_env_hsl, p_route_env);
                } break;

                case (MM_MSG_LIGHT_HSL_RANGE_GET):
                {
                    // Send a Light HSL Range Status message
                    mm_lights_hsl_send_status_range(p_env_hsl, p_route_env, MM_STATUS_SUCCESS);
                } break;

                default:
                {
                } break;
            }
        }
        else if (p_env->mdl_id == MM_ID_LIGHTS_HSLS)
        {
            // Get Environment for Light HSL Setup Server model
            mm_lights_hsls_env_t *p_env_hsls = (mm_lights_hsls_env_t *)p_env;

            // Get environment for Light HSL Server model
            p_env_hsl = p_env_hsls->p_env_hsl;

            switch (p_route_env->opcode)
            {
                case (MM_MSG_LIGHT_HSL_DFLT_SET):
                case (MM_MSG_LIGHT_HSL_DFLT_SET_UNACK):
                {
                    // Handle the message
                    mm_lights_hsl_handler_set_dflt(p_env_hsl, p_buf, p_route_env);
                } break;

                case (MM_MSG_LIGHT_HSL_RANGE_SET):
                case (MM_MSG_LIGHT_HSL_RANGE_SET_UNACK):
                {
                    // Handle the message
                    mm_lights_hsl_handler_set_range(p_env_hsl, p_buf, p_route_env);
                } break;

                default:
                {
                } break;
            }
        }
        else if (p_env->mdl_id == MM_ID_LIGHTS_HSLH)
        {
            // Get environment for Light HSL Hue model
            mm_lights_hslh_env_t *p_env_hslh = (mm_lights_hslh_env_t *)p_env;

            switch (p_route_env->opcode)
            {
                case (MM_MSG_LIGHT_HSL_HUE_GET):
                {
                    // Send a Light HSL Hue Status message
                    mm_lights_hsl_send_status_hue(p_env_hslh, p_route_env, false);
                } break;

                case (MM_MSG_LIGHT_HSL_HUE_SET):
                case (MM_MSG_LIGHT_HSL_HUE_SET_UNACK):
                {
                    // Handle the message
                    mm_lights_hsl_handler_set_hue(p_env_hslh, p_buf, p_route_env);
                } break;

                default:
                {
                } break;
            }
        }
        else     // (p_env->mdl_id == MM_ID_LIGHTS_HSLSAT)
        {
            // Get environment for Light HSL Saturation model
            mm_lights_hslsat_env_t *p_env_hslsat = (mm_lights_hslsat_env_t *)p_env;

            switch (p_route_env->opcode)
            {
                case (MM_MSG_LIGHT_HSL_SAT_GET):
                {
                    // Send a Light HSL Saturation Status message
                    mm_lights_hsl_send_status_sat(p_env_hslsat, p_route_env, false);
                } break;

                case (MM_MSG_LIGHT_HSL_SAT_SET):
                case (MM_MSG_LIGHT_HSL_SAT_SET_UNACK):
                {
                    // Handle the message
                    mm_lights_hsl_handler_set_sat(p_env_hslsat, p_buf, p_route_env);
                } break;

                default:
                {
                } break;
            }
        }
    }
    while (0);
}

/**
 ****************************************************************************************
 * @brief Check if a given opcode is handled by the Light HSL Server model
 *
 * @param[in] p_env         Pointer to the environment allocated for the Light HSL
 * Server model
 * @param[in] opcode        Opcode to check
 *
 * @return An error status (@see enum mesh_err)
 ****************************************************************************************
 */
__STATIC uint16_t mm_lights_hsl_cb_opcode_check(mm_tb_state_mdl_env_t *p_env, uint32_t opcode)
{
    uint16_t status = MESH_ERR_MDL_INVALID_OPCODE;

    if (p_env->mdl_id == MM_ID_LIGHTS_HSL)
    {
        if ((opcode == MM_MSG_LIGHT_HSL_GET)
                || (opcode == MM_MSG_LIGHT_HSL_SET)
                || (opcode == MM_MSG_LIGHT_HSL_SET_UNACK)
                || (opcode == MM_MSG_LIGHT_HSL_TGT_GET)
                || (opcode == MM_MSG_LIGHT_HSL_RANGE_GET)
                || (opcode == MM_MSG_LIGHT_HSL_DFLT_GET))
        {
            status = MESH_ERR_NO_ERROR;
        }
    }
    else if (p_env->mdl_id == MM_ID_LIGHTS_HSLS)
    {
        if ((opcode == MM_MSG_LIGHT_HSL_DFLT_SET)
                || (opcode == MM_MSG_LIGHT_HSL_DFLT_SET_UNACK)
                || (opcode == MM_MSG_LIGHT_HSL_RANGE_SET)
                || (opcode == MM_MSG_LIGHT_HSL_RANGE_SET_UNACK))
        {
            status = MESH_ERR_NO_ERROR;
        }
    }
    else if (p_env->mdl_id == MM_ID_LIGHTS_HSLH)
    {
        if ((opcode == MM_MSG_LIGHT_HSL_HUE_GET)
                || (opcode == MM_MSG_LIGHT_HSL_HUE_SET)
                || (opcode == MM_MSG_LIGHT_HSL_HUE_SET_UNACK))
        {
            status = MESH_ERR_NO_ERROR;
        }
    }
    else if (p_env->mdl_id == MM_ID_LIGHTS_HSLSAT)
    {
        if ((opcode == MM_MSG_LIGHT_HSL_SAT_GET)
                || (opcode == MM_MSG_LIGHT_HSL_SAT_SET)
                || (opcode == MM_MSG_LIGHT_HSL_SAT_SET_UNACK))
        {
            status = MESH_ERR_NO_ERROR;
        }
    }

    if (status != MESH_ERR_NO_ERROR)
    {
        MESH_MODEL_PRINT_ERR("%s, Invalid opcode (0x%x)\n", __func__, opcode);
    }

    return (status);
}

/**
 ****************************************************************************************
 * @brief Inform Light HSL Server model about received publication parameters
 *
 * @param[in] p_env         Pointer the the environment allocated for the Light CT:
 * Server model
 * @param[in] addr          Publication address
 * @param[in] period_ms     Publication period in milliseconds
 ****************************************************************************************
 */
__STATIC void mm_lights_hsl_cb_publish_param(mm_tb_state_mdl_env_t *p_env, uint16_t addr,
        uint32_t period_ms)
{
    // Inform the state manager about received publication parameters
    mm_tb_state_publish_param_ind((mm_tb_state_mdl_publi_env_t *)p_env, addr, period_ms);
}

/**
 ****************************************************************************************
 * @brief Callback function called upon reception of application request to set a Light
 * HSL state value
 *
 * @param[in] p_env         Pointer the the environment allocated for model
 * @param[in] state_id      State identifier
 * @param[in] state         State value
 *
 * @return An error status (@see enum mesh_err)
 ****************************************************************************************
 */
__STATIC uint16_t mm_lights_hsl_cb_set(mm_tb_state_mdl_env_t *p_env, uint16_t state_id,
                                       uint32_t state)
{
    // Returned status
    uint16_t status = MESH_ERR_NO_ERROR;

    if (state_id == MM_STATE_LIGHT_HSL_LN)
    {
        // Get environment for the Light HSL Server model
        mm_lights_hsl_env_t *p_env_hsl = (mm_lights_hsl_env_t *)p_env;

        // Keep the provided state value
        p_env_hsl->ln = state;

        // Set the Light Lightness state value
        mm_tb_bind_set_state(p_env_hsl->env.grp_lid, MM_STATE_TYPE_CURRENT,
                             MM_ID_LIGHTS_LN, state);
    }
    else
    {
        MESH_MODEL_PRINT_WARN("%s, Invalid parameter. \n", __func__);
        status = MESH_ERR_INVALID_PARAM;
    }

    return (status);
}

/**
 ****************************************************************************************
 * @brief Callback function called upon reception of application request to set a Light
 * HSL Hue state value
 *
 * @param[in] p_env         Pointer the the environment allocated for model
 * @param[in] state_id      State identifier
 * @param[in] state         State value
 *
 * @return An error status (@see enum mesh_err)
 ****************************************************************************************
 */
__STATIC uint16_t mm_lights_hsl_cb_set_hue(mm_tb_state_mdl_env_t *p_env, uint16_t state_id,
        uint32_t state)
{
    // Returned status
    uint16_t status = MESH_ERR_NO_ERROR;
    // Get environment for the Light HSL Hue Server model
    mm_lights_hslh_env_t *p_env_hslh = (mm_lights_hslh_env_t *)p_env;

    switch (state_id)
    {
        case (MM_STATE_LIGHT_HSL_HUE):
        {
            // Keep the provided state value
            p_env_hslh->hue = state;

            // Set the Generic Level state value
            mm_tb_bind_set_state(p_env_hslh->env.grp_lid, MM_STATE_TYPE_CURRENT,
                                 MM_ID_GENS_LVL, state - 32768);
        } break;

        case (MM_STATE_LIGHT_HSL_DFLT_HUE):
        {
            // Keep the provided state value
            p_env_hslh->hue_dflt = state;
        } break;

        case (MM_STATE_LIGHT_HSL_RANGE_HUE):
        {
            // Keep the provided state value
            p_env_hslh->hue_min = state;
            p_env_hslh->hue_max = (state >> 16);
        } break;

        default:
        {
            MESH_MODEL_PRINT_WARN("%s, Invalid parameter. \n", __func__);
            status = MESH_ERR_INVALID_PARAM;
        } break;
    }

    return (status);
}

/**
 ****************************************************************************************
 * @brief Callback function called upon reception of application request to set a Light
 * HSL Saturation state value
 *
 * @param[in] p_env         Pointer the the environment allocated for model
 * @param[in] state_id      State identifier
 * @param[in] state         State value
 *
 * @return An error status (@see enum mesh_err)
 ****************************************************************************************
 */
__STATIC uint16_t mm_lights_hsl_cb_set_sat(mm_tb_state_mdl_env_t *p_env, uint16_t state_id, uint32_t state)
{
    // Returned status
    uint16_t status = MESH_ERR_NO_ERROR;
    // Get environment for the Light HSL Saturation Server model
    mm_lights_hslsat_env_t *p_env_hslsat = (mm_lights_hslsat_env_t *)p_env;

    switch (state_id)
    {
        case (MM_STATE_LIGHT_HSL_SAT):
        {
            // Keep the provided state value
            p_env_hslsat->sat = state;

            // Set the Generic Level state value
            mm_tb_bind_set_state(p_env_hslsat->env.grp_lid, MM_STATE_TYPE_CURRENT,
                                 MM_ID_GENS_LVL, state - 32768);
        } break;

        case (MM_STATE_LIGHT_HSL_DFLT_SAT):
        {
            // Keep the provided state value
            p_env_hslsat->sat_dflt = state;
        } break;

        case (MM_STATE_LIGHT_HSL_RANGE_SAT):
        {
            // Keep the provided state value
            p_env_hslsat->sat_min = state;
            p_env_hslsat->sat_max = (state >> 16);
        } break;

        default:
        {
            MESH_MODEL_PRINT_WARN("%s, Invalid parameter. \n", __func__);
            status = MESH_ERR_INVALID_PARAM;
        } break;
    }

    return (status);
}

/*
 * REGISTRATION FUNCTIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Register Light HSL Server model
 *
 * @param[in] elmt_idx      Index of element on which the model must be registered
 * @param[out] p_mdl_lid    Address at which allocated model local index must be returned
 *
 * @return An handling status (@see enum mesh_err)
 ****************************************************************************************
 */
__STATIC uint16_t mm_lights_hsl_register_hsl(uint8_t elmt_idx, m_lid_t *p_mdl_lid)
{
    // Register Light HSL Server model
    uint16_t status = m_api_register_model(MM_ID_LIGHTS_HSL, elmt_idx, M_MDL_CONFIG_PUBLI_AUTH_BIT,
                                           &mm_route_cb, p_mdl_lid);

    if (status == MESH_ERR_NO_ERROR)
    {
        // Inform the Model State Manager about registered model
        status = mm_tb_state_register(elmt_idx, MM_ID_LIGHTS_HSL, *p_mdl_lid,
                                      MM_TB_STATE_ROLE_SRV | MM_TB_STATE_CFG_CB_BIT,
                                      sizeof(mm_lights_hsl_env_t));

        if (status == MESH_ERR_NO_ERROR)
        {
            // Get allocated environment
            mm_lights_hsl_env_t *p_env_hsl = (mm_lights_hsl_env_t *)mm_tb_state_get_env(*p_mdl_lid);
            // Pointer to server-specific callback functions
            mm_srv_cb_t *p_cb_srv = p_env_hsl->env.cb.u.p_cb_srv;

            // Prepare environment for Replay Manager
            p_env_hsl->replay_env.delay_ms = MM_LIGHTS_HSL_REPLAY_MS;

            p_env_hsl->ln = 0x8000;//sean add for init

            // Prepare timer for publications
            p_env_hsl->tmr_publi.cb = mm_lights_hsl_cb_tmr_publi;
            p_env_hsl->tmr_publi.p_env = (void *)p_env_hsl;

            // Set internal callback functions
            p_env_hsl->env.cb.cb_rx = mm_lights_hsl_cb_rx;
            p_env_hsl->env.cb.cb_opcode_check = mm_lights_hsl_cb_opcode_check;
            p_env_hsl->env.cb.cb_publish_param = mm_lights_hsl_cb_publish_param;
            p_cb_srv->cb_set = mm_lights_hsl_cb_set;

            // Inform application about registered model
            mm_api_send_register_ind(MM_ID_LIGHTS_HSL, elmt_idx, *p_mdl_lid);
        }
    }

    return (status);
}

/**
 ****************************************************************************************
 * @brief Register Light HSL Setup Server model
 *
 * @param[in] elmt_idx      Index of element on which the model must be registered
 * @param[out] p_mdl_lid    Address at which allocated model local index must be returned
 *
 * @return An handling status (@see enum mesh_err)
 ****************************************************************************************
 */
__STATIC uint16_t mm_lights_hsl_register_hsls(uint8_t elmt_idx, m_lid_t *p_mdl_lid)
{
    // Register Light HSL Setup Server model
    uint16_t status = m_api_register_model(MM_ID_LIGHTS_HSLS, elmt_idx, M_MDL_CONFIG_PUBLI_AUTH_BIT,
                                           &mm_route_cb, p_mdl_lid);

    if (status == MESH_ERR_NO_ERROR)
    {
        // Inform the Model State Manager about registered model
        status = mm_tb_state_register(elmt_idx, MM_ID_LIGHTS_HSLS, *p_mdl_lid,
                                      MM_TB_STATE_ROLE_SRV, sizeof(mm_lights_hsls_env_t));

        if (status == MESH_ERR_NO_ERROR)
        {
            // Get allocated environment
            mm_lights_hsls_env_t *p_env_hsls
                = (mm_lights_hsls_env_t *)mm_tb_state_get_env(*p_mdl_lid);

            // Set internal callback functions
            p_env_hsls->env.cb.cb_rx = mm_lights_hsl_cb_rx;
            p_env_hsls->env.cb.cb_opcode_check = mm_lights_hsl_cb_opcode_check;

            // Inform application about registered model
            mm_api_send_register_ind(MM_ID_LIGHTS_HSLS, elmt_idx, *p_mdl_lid);
        }
        else
        {
            MESH_MODEL_PRINT_WARN("%s, Model State register fail. \n", __func__);
        }
    }
    else
    {
        MESH_MODEL_PRINT_WARN("%s, Model register fail. \n", __func__);
    }

    return (status);
}

/**
 ****************************************************************************************
 * @brief Register Light HSL Hue Server model
 *
 * @param[in] elmt_idx      Index of element on which the model must be registered
 * @param[out] p_mdl_lid    Address at which allocated model local index must be returned
 *
 * @return An handling status (@see enum mesh_err)
 ****************************************************************************************
 */
__STATIC uint16_t mm_lights_hsl_register_hslh(uint8_t elmt_idx, m_lid_t *p_mdl_lid)
{
    // Register Light HSL Hue Server model
    uint16_t status = m_api_register_model(MM_ID_LIGHTS_HSLH, elmt_idx, M_MDL_CONFIG_PUBLI_AUTH_BIT,
                                           &mm_route_cb, p_mdl_lid);

    if (status == MESH_ERR_NO_ERROR)
    {
        // Inform the Model State Manager about registered model
        status = mm_tb_state_register(elmt_idx, MM_ID_LIGHTS_HSLH, *p_mdl_lid,
                                      MM_TB_STATE_ROLE_SRV | MM_TB_STATE_CFG_CB_BIT,
                                      sizeof(mm_lights_hslh_env_t));

        if (status == MESH_ERR_NO_ERROR)
        {
            // Get allocated environment
            mm_lights_hslh_env_t *p_env_hslh = (mm_lights_hslh_env_t *)mm_tb_state_get_env(*p_mdl_lid);
            // Pointer to server-specific callback functions
            mm_srv_cb_t *p_cb_srv = p_env_hslh->env.cb.u.p_cb_srv;

            // Prepare timer for publications
            p_env_hslh->tmr_publi.cb = mm_lights_hsl_cb_tmr_publi;
            p_env_hslh->tmr_publi.p_env = (void *)p_env_hslh;

            // Set internal callback functions
            p_env_hslh->env.cb.cb_rx = mm_lights_hsl_cb_rx;
            p_env_hslh->env.cb.cb_opcode_check = mm_lights_hsl_cb_opcode_check;
            p_env_hslh->env.cb.cb_publish_param = mm_lights_hsl_cb_publish_param;
            p_cb_srv->cb_set = mm_lights_hsl_cb_set_hue;

            // Set range value
            p_env_hslh->hue_max = 0xFFFF;

            // Inform application about registered model
            mm_api_send_register_ind(MM_ID_LIGHTS_HSLH, elmt_idx, *p_mdl_lid);
        }
        else
        {
            MESH_MODEL_PRINT_WARN("%s, Model State register fail. \n", __func__);
        }
    }
    else
    {
        MESH_MODEL_PRINT_WARN("%s, Model register fail. \n", __func__);
    }
    return (status);
}

/**
 ****************************************************************************************
 * @brief Register Light HSL Saturation Server model
 *
 * @param[in] elmt_idx      Index of element on which the model must be registered
 * @param[out] p_mdl_lid    Address at which allocated model local index must be returned
 *
 * @return An handling status (@see enum mesh_err)
 ****************************************************************************************
 */
__STATIC uint16_t mm_lights_hsl_register_hslsat(uint8_t elmt_idx, m_lid_t *p_mdl_lid)
{
    // Register Light HSL Server model
    uint16_t status = m_api_register_model(MM_ID_LIGHTS_HSLSAT, elmt_idx, M_MDL_CONFIG_PUBLI_AUTH_BIT,
                                           &mm_route_cb, p_mdl_lid);

    if (status == MESH_ERR_NO_ERROR)
    {
        // Inform the Model State Manager about registered model
        status = mm_tb_state_register(elmt_idx, MM_ID_LIGHTS_HSLSAT, *p_mdl_lid,
                                      MM_TB_STATE_ROLE_SRV | MM_TB_STATE_CFG_CB_BIT,
                                      sizeof(mm_lights_hslsat_env_t));

        if (status == MESH_ERR_NO_ERROR)
        {
            // Get allocated environment
            mm_lights_hslsat_env_t *p_env_hslsat
                = (mm_lights_hslsat_env_t *)mm_tb_state_get_env(*p_mdl_lid);
            // Pointer to server-specific callback functions
            mm_srv_cb_t *p_cb_srv = p_env_hslsat->env.cb.u.p_cb_srv;

            // Prepare timer for publications
            p_env_hslsat->tmr_publi.cb = mm_lights_hsl_cb_tmr_publi;
            p_env_hslsat->tmr_publi.p_env = (void *)p_env_hslsat;

            // Set internal callback functions
            p_env_hslsat->env.cb.cb_rx = mm_lights_hsl_cb_rx;
            p_env_hslsat->env.cb.cb_opcode_check = mm_lights_hsl_cb_opcode_check;
            p_env_hslsat->env.cb.cb_publish_param = mm_lights_hsl_cb_publish_param;
            p_cb_srv->cb_set = mm_lights_hsl_cb_set_sat;

            // Set range value
            p_env_hslsat->sat_max = 0xFFFF;

            // Inform application about registered model
            mm_api_send_register_ind(MM_ID_LIGHTS_HSLSAT, elmt_idx, *p_mdl_lid);
        }
        else
        {
            MESH_MODEL_PRINT_WARN("%s, Model State register fail. \n", __func__);
        }
    }
    else
    {
        MESH_MODEL_PRINT_WARN("%s, Model register fail. \n", __func__);
    }

    return (status);
}

/**
 ****************************************************************************************
 * @brief Bind the Light HSL Server, the Light HSL Setup Server, the Light HSL Hue Server
 * and the Light HSL Saturation Server models together
 *
 * @param[in] mdl_lid           Model local index for the Light HSL Server model
 * @param[in] mdl_lid_setup     Model local index for the Light HSL Setup Server model
 * @param[in] mdl_lid_hue       Model local index for the Light HSL Hue Server model
 * @param[in] mdl_lid_sat       Model local index for the Light HSL Saturation Server model
 ****************************************************************************************
 */
void mm_lights_hsl_bind(m_lid_t mdl_lid, m_lid_t mdl_lid_setup, m_lid_t mdl_lid_hue,
                        m_lid_t mdl_lid_sat)
{
    // Pointer to environment for the Light HSL Server model
    mm_lights_hsl_env_t *p_env_hsl = (mm_lights_hsl_env_t *)mm_tb_state_get_env(mdl_lid);
    // Pointer to environment for the Light HSL Setup Server model
    mm_lights_hsls_env_t *p_env_hsls = (mm_lights_hsls_env_t *)mm_tb_state_get_env(mdl_lid_setup);
    // Pointer to environment for the Light HSL Hue Server model
    mm_lights_hslh_env_t *p_env_hslh = (mm_lights_hslh_env_t *)mm_tb_state_get_env(mdl_lid_hue);
    // Pointer to environment for the Light HSL Saturation Server model
    mm_lights_hslsat_env_t *p_env_hslsat = (mm_lights_hslsat_env_t *)mm_tb_state_get_env(mdl_lid_sat);

    // Bind environment together
    p_env_hsl->p_env_hslh = p_env_hslh;
    p_env_hsl->p_env_hslsat = p_env_hslsat;
    p_env_hsls->p_env_hsl = p_env_hsl;
    p_env_hslh->p_env_hsl = p_env_hsl;
    p_env_hslsat->p_env_hsl = p_env_hsl;
}

/*
 * GLOBAL FUNCTIONS
 ****************************************************************************************
 */

uint16_t mm_lights_hsl_register(uint8_t elmt_idx, m_lid_t *p_mdl_lid, m_lid_t *p_mdl_lid_hue,
                                m_lid_t *p_mdl_lid_sat)
{
    uint16_t status;

    do
    {
        // Model local index for Light HSL Setup Server model
        m_lid_t mdl_lid_setup;

        // Register Light HSL Server model
        status = mm_lights_hsl_register_hsl(elmt_idx, p_mdl_lid);

        if (status != MESH_ERR_NO_ERROR)
        {
            break;
        }

        // Register Light HSL Setup Server model
        status = mm_lights_hsl_register_hsls(elmt_idx, &mdl_lid_setup);

        if (status != MESH_ERR_NO_ERROR)
        {
            break;
        }

        // Register Light HSL Hue Server model
        status = mm_lights_hsl_register_hslh(elmt_idx + 1, p_mdl_lid_hue);

        if (status != MESH_ERR_NO_ERROR)
        {
            break;
        }

        // Register Light HSL Saturation Server model
        status = mm_lights_hsl_register_hslsat(elmt_idx + 2, p_mdl_lid_sat);

        if (status == MESH_ERR_NO_ERROR)
        {
            // Bound models together
            mm_lights_hsl_bind(*p_mdl_lid, mdl_lid_setup, *p_mdl_lid_hue, *p_mdl_lid_sat);
        }
    }
    while (0);

    return (status);
}

/*
 * CALLBACK FUNCTIONS FOR BINDING MANAGER
 ****************************************************************************************
 */
void mm_lights_hsl_cb_grp_event(m_lid_t mdl_lid, uint8_t event, uint8_t info)
{
    // Get environment for Light HSL Server model
    mm_lights_hsl_env_t *p_env_hsl = (mm_lights_hsl_env_t *)mm_tb_state_get_env(mdl_lid);

    MESH_MODEL_PRINT_DEBUG("%s mdl_lid:%d,event:%d\r\n", __func__, mdl_lid, event);
    switch (event)
    {
        case (MESH_MDL_GRP_EVENT_TRANS_DELAY_EXPIRED):
        {
            // Set the targeted Light Lightness state value
            mm_tb_bind_set_state(p_env_hsl->env.grp_lid, MM_STATE_TYPE_TARGET, MM_ID_LIGHTS_LN,
                                 p_env_hsl->ln_tgt);

            // Start the transition
            mm_tb_bind_trans_start(p_env_hsl->env.grp_lid);

        } break;

        case (MESH_MDL_GRP_EVENT_TRANS_IMMEDIATE):
        {
            p_env_hsl->ln = p_env_hsl->ln_tgt;
        } // no break;

        case (MESH_MDL_GRP_EVENT_TRANS_STARTED):
        {
            uint8_t trans_time = info;

            p_env_hsl->ln = p_env_hsl->ln_tgt;//sean add

            // Transition has been started
            SETB(p_env_hsl->status_info, MM_LIGHTS_HSL_SINFO_WAIT_LN, 0);

            // Inform application about state update
            mm_api_send_srv_state_upd_ind(MM_STATE_LIGHT_HSL_LN, p_env_hsl->env.elmt_idx,
                                          p_env_hsl->ln_tgt,
                                          (trans_time) ? mm_tb_get_trans_time_ms(trans_time) : 0);

            // Check if a status message must be sent
            mm_lights_hsl_check_status_rsp(p_env_hsl);

            // Send a publication
            mm_lights_hsl_publish(p_env_hsl, (event == MESH_MDL_GRP_EVENT_TRANS_STARTED));
        } break;

        case (MESH_MDL_GRP_EVENT_TRANS_END):
        {
            p_env_hsl->ln = p_env_hsl->ln_tgt;


            // Inform application about state update
            mm_api_send_srv_state_upd_ind(MM_STATE_LIGHT_HSL_LN,
                                          p_env_hsl->env.elmt_idx, p_env_hsl->ln, 0);

            // Check if a status message must be sent
            mm_lights_hsl_check_status_rsp(p_env_hsl);

            // Send a publication
            mm_lights_hsl_publish(p_env_hsl, false);

        } break;

        case (MESH_MDL_GRP_EVENT_GROUP_FULL):
        {
            // Set the current Light Lightness state value
            mm_tb_bind_set_state(p_env_hsl->env.grp_lid, MM_STATE_TYPE_CURRENT, MM_ID_LIGHTS_LN,
                                 p_env_hsl->ln);
        } break;

        //case (MESH_MDL_GRP_EVENT_TRANS_ABORTED):
        default:
        {
        } break;
    }
}

void mm_lights_hsl_cb_grp_event_hue_sat(m_lid_t mdl_lid, uint8_t event, uint8_t info)
{
    // Pointer to environment for the main model
    mm_tb_state_mdl_env_t *p_env = (mm_tb_state_mdl_env_t *)mm_tb_state_get_env(mdl_lid);
    // Pointer to the environment allocated for the Light HSL Server model
    mm_lights_hsl_env_t *p_env_hsl;
    // Pointer to the needed value in the main model environment
    uint16_t *p_tgt_state, *p_state;
    // State identifier
    uint32_t state_id;

    MESH_MODEL_PRINT_DEBUG("%s mdl_lid:%d,event:%d\r\n", __func__, mdl_lid, event);
    if (p_env->mdl_id == MM_ID_LIGHTS_HSLH)
    {
        // Pointer to environment for the Light HSL Hue Server model
        mm_lights_hslh_env_t *p_env_hslh = (mm_lights_hslh_env_t *)p_env;

        p_tgt_state = &p_env_hslh->hue_tgt;
        p_state = &p_env_hslh->hue;
        p_env_hsl = p_env_hslh->p_env_hsl;
        state_id = MM_STATE_LIGHT_HSL_HUE;
    }
    else     // (p_env->mdl_id == MM_ID_LIGHTS_HSLSAT)
    {
        // Pointer to environment for the Light HSL Saturation Server model
        mm_lights_hslsat_env_t *p_env_hslsat = (mm_lights_hslsat_env_t *)p_env;

        p_tgt_state = &p_env_hslsat->sat_tgt;
        p_state = &p_env_hslsat->sat;
        p_env_hsl = p_env_hslsat->p_env_hsl;
        state_id = MM_STATE_LIGHT_HSL_SAT;
    }

    switch (event)
    {
        case (MESH_MDL_GRP_EVENT_TRANS_DELAY_EXPIRED):
        {
            // Generic Level = Light HSL Hue/Saturation - 32768
            int16_t lvl = ((int32_t) * p_tgt_state) - 32768;

            // Set the targeted Generic Level state value
            mm_tb_bind_set_state(p_env->grp_lid, MM_STATE_TYPE_TARGET, MM_ID_GENS_LVL, lvl);

            // Start the transition
            mm_tb_bind_trans_start(p_env->grp_lid);
        } break;

        case (MESH_MDL_GRP_EVENT_TRANS_IMMEDIATE):
        {
            *p_state = *p_tgt_state;
        } // no break;

        case (MESH_MDL_GRP_EVENT_TRANS_STARTED):
        {
            uint8_t trans_time = info;
            *p_state = *p_tgt_state;//sean add
            // Transition has been started
            if (p_env->mdl_id == MM_ID_LIGHTS_HSLH)
            {
                SETB(p_env_hsl->status_info, MM_LIGHTS_HSL_SINFO_WAIT_HUE, 0);
            }
            else
            {
                SETB(p_env_hsl->status_info, MM_LIGHTS_HSL_SINFO_WAIT_SAT, 0);
            }

            // Inform application about state update
            mm_api_send_srv_state_upd_ind(state_id, p_env->elmt_idx, *p_tgt_state,
                                          (trans_time) ? mm_tb_get_trans_time_ms(trans_time) : 0);

            // Check if a status message must be sent
            mm_lights_hsl_check_status_rsp_hue_sat(p_env);


            // Send a publication
            mm_lights_hsl_publish_hue_sat(p_env);
        } break;

        case (MESH_MDL_GRP_EVENT_TRANS_END):
        {
            *p_state = *p_tgt_state;

            // Inform application about state update
            mm_api_send_srv_state_upd_ind(state_id, p_env->elmt_idx, *p_state, 0);

            // Check if a status message must be sent
            mm_lights_hsl_check_status_rsp_hue_sat(p_env);

            // Send a publication
            mm_lights_hsl_publish_hue_sat(p_env);

        } break;

        case (MESH_MDL_GRP_EVENT_GROUP_FULL):
        {
            // Generic Level = Light HSL Hue/Saturation - 32768
            int16_t lvl = ((int32_t) * p_state) - 32768;

            // Set the current Generic Level state value
            mm_tb_bind_set_state(p_env->grp_lid, MM_STATE_TYPE_CURRENT, MM_ID_GENS_LVL, lvl);
        } break;

        //case (MESH_MDL_GRP_EVENT_TRANS_ABORTED):
        default:
        {
        } break;
    }
}

void mm_lights_hsl_cb_trans_req(m_lid_t main_mdl_lid, uint32_t req_model_id, uint8_t trans_type,
                                uint32_t state_delta)
{
    // Get environment for Light HSL Server model
    mm_lights_hsl_env_t *p_env_hsl = (mm_lights_hsl_env_t *)mm_tb_state_get_env(main_mdl_lid);
    // Targeted Light HSL Lightness state value
    uint16_t ln_tgt;

    MESH_MODEL_PRINT_DEBUG("%s main_mdl_lid:%d,req_model_id:0x%x\r\n", __func__, main_mdl_lid, req_model_id);
    if (req_model_id == MM_ID_LIGHTS_LN)
    {
        // Light HSL Lightness = Light Lightness Actual
        ln_tgt = (uint16_t)state_delta;
    }
    else
    {
        if (req_model_id == MM_ID_GENS_OO)
        {
            // Requested Generic OnOff state value
            uint8_t onoff = (uint8_t)state_delta;

            // Deduce Target Light HSL Lightness state value
            ln_tgt = (onoff == 0) ? 0 : mm_lights_ln_get(p_env_hsl->env.elmt_idx,
                     MM_STATE_LIGHT_LN_LAST/*MM_STATE_LIGHT_LN_DFLT*/);
        }
        else     // (req_model_id == MM_ID_GENS_LVL)
        {
            if (trans_type == MM_TRANS_TYPE_CLASSIC)
            {
                // Requested Generic Level state value
                int16_t level = (int16_t)state_delta;

                // Light HSL Lightness = Generic Level + 32768
                ln_tgt = 32768 + level;
            }
            else     // ((trans_type == MM_TRANS_TYPE_DELTA) || trans_type == MM_TRANS_TYPE_MOVE))
            {
                // Delta value
                int32_t delta;

                if (trans_type == MM_TRANS_TYPE_MOVE)
                {
                    delta = (int16_t)state_delta;

                    // Keep the provided delta value
                    p_env_hsl->move_delta = (int16_t)state_delta;
                }
                else
                {
                    delta = (int32_t)state_delta;
                }

                // Add the Light HSL Lightness state value to the received delta value
                delta += p_env_hsl->ln;

                // The Light HSL Actual state value cannot wrap
                if (delta < 0)
                {
                    ln_tgt = 0;
                }
                else
                {
                    ln_tgt = (delta > 0xFFFF) ? 0xFFFF : (uint16_t)delta;
                }
            }
        }

        if (ln_tgt != 0)
        {
            // Get range value
            uint16_t ln_min = mm_lights_ln_get(p_env_hsl->env.elmt_idx, MM_STATE_LIGHT_LN_RANGE_MIN);
            uint16_t ln_max = mm_lights_ln_get(p_env_hsl->env.elmt_idx, MM_STATE_LIGHT_LN_RANGE_MAX);

            // Ensure that Light HSL Lightness state value is between Light HSL Lightness Range
            // Min and Max values
            if (ln_tgt > ln_max)
            {
                ln_tgt = ln_max;
            }
            else if (ln_tgt < ln_min)
            {
                ln_tgt = ln_min;
            }
        }
    }

    MESH_MODEL_PRINT_DEBUG("ln_tgt:%d,p_env_hsl->ln:%d\r\n", ln_tgt, p_env_hsl->ln);
    // Check if Light HSL Actual state value is modified
    if (ln_tgt != p_env_hsl->ln)
    {
        p_env_hsl->ln_tgt = ln_tgt;

        // Start a new transition
        mm_tb_bind_trans_new(p_env_hsl->env.grp_lid, trans_type, 0, 0);
    }
    else
    {
        // Reject the transition
        mm_tb_bind_trans_reject(p_env_hsl->env.grp_lid);
    }
}

void mm_lights_hsl_cb_trans_req_hue_sat(m_lid_t main_mdl_lid, uint32_t req_model_id, uint8_t trans_type,
                                        uint32_t state_delta)
{
    // Pointer to environment for the main model
    mm_tb_state_mdl_env_t *p_env = (mm_tb_state_mdl_env_t *)mm_tb_state_get_env(main_mdl_lid);
    // Pointer to the needed value in the main model environment
    uint16_t *p_range_min, *p_range_max, *p_tgt_state, *p_state;
    int16_t *p_move_delta;
    // Target Light HSL Hue or Light HSL Saturation state value
    uint16_t tgt_state;

    MESH_MODEL_PRINT_DEBUG("%s main_mdl_lid:%d,req_model_id:0x%x\r\n", __func__, main_mdl_lid, req_model_id);
    if (p_env->mdl_id == MM_ID_LIGHTS_HSLH)
    {
        // Pointer to environment for the Light HSL Hue Server model
        mm_lights_hslh_env_t *p_env_hslh = (mm_lights_hslh_env_t *)p_env;

        p_range_min = &p_env_hslh->hue_min;
        p_range_max = &p_env_hslh->hue_max;
        p_tgt_state = &p_env_hslh->hue_tgt;
        p_state = &p_env_hslh->hue;
        p_move_delta = &p_env_hslh->move_delta;
    }
    else     // (p_env->mdl_id == MM_ID_LIGHTS_HSLSAT)
    {
        // Pointer to environment for the Light HSL Saturation Server model
        mm_lights_hslsat_env_t *p_env_hslsat = (mm_lights_hslsat_env_t *)p_env;

        p_range_min = &p_env_hslsat->sat_min;
        p_range_max = &p_env_hslsat->sat_max;
        p_tgt_state = &p_env_hslsat->sat_tgt;
        p_state = &p_env_hslsat->sat;
        p_move_delta = &p_env_hslsat->move_delta;
    }

    if (trans_type == MM_TRANS_TYPE_CLASSIC)
    {
        // Light HSL Hue/Saturation = Generic Level + 32768
        tgt_state = 32768 + (int16_t)state_delta;
    }
    else     // ((trans_type == MM_TRANS_TYPE_DELTA) || trans_type == MM_TRANS_TYPE_MOVE))
    {
        // Delta value
        int32_t delta;

        if (trans_type == MM_TRANS_TYPE_MOVE)
        {
            delta = (int16_t)state_delta;

            // Keep the provided delta value
            *p_move_delta = (int16_t)state_delta;
        }
        else
        {
            delta = (int32_t)state_delta;
        }

        // Add the current state value to the received delta value
        delta += *p_state;;

        // The state value cannot wrap
        if (delta < 0)
        {
            tgt_state = 0;
        }
        else
        {
            tgt_state = (delta > 0xFFFF) ? 0xFFFF : (uint16_t)delta;
        }
    }

    // Check that new targeted Light HSL Hue/Saturation state value is well within the set Light HSL
    // Hue/Saturation state min and max values
    if (tgt_state > *p_range_max)
    {
        tgt_state = *p_range_max;
    }
    else if (tgt_state < *p_range_min)
    {
        tgt_state = *p_range_min;
    }

    // Check if state value is modified
    if (tgt_state != *p_tgt_state)
    {
        *p_tgt_state = tgt_state;

        // Start a new transition
        mm_tb_bind_trans_new(p_env->grp_lid, trans_type, 0, 0);
    }
    else
    {
        // Reject the transition
        mm_tb_bind_trans_reject(p_env->grp_lid);
    }
}
#endif //(BLE_MESH_MDL_LIGHTS_HSL)
/// @} end of group
