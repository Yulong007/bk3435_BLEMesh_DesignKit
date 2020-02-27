/**
 ****************************************************************************************
 * @file mm_lights_ctl.c
 *
 * @brief Mesh Model Light CTL Server Module
 *
 * Copyright (C) RivieraWaves 2017-2019
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup MM_LIGHTS_CTL
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "mm_lights_int.h"      // Mesh Model Lighting Module Internal Definitions
#include "uart.h"

#if (BLE_MESH_MDL_LIGHTS_CTL)
/*
 * DEFINES
 ****************************************************************************************
 */

/// Validity of information provided to the Replay Manager
#define MM_LIGHTS_CTL_REPLAY_MS               (6000)
/// Minimum value for Light CTL Temperature state value
#define MM_LIGHTS_CTL_TEMP_MIN                (0x0320)
/// Maximum value for Light CTL Temperature state value
#define MM_LIGHTS_CTL_TEMP_MAX                (0x4E20)

/*
 * ENUMERATIONS
 ****************************************************************************************
 */

/// Bit field content for status_info value present in environment for Light CTL Server model
/// 7              4             3                 2               1       0
/// +--------------+-------------+-----------------+---------------+-------+
/// |      RFU     | Temp Status | Wait Temp Trans | Wait LN Trans | Relay |
/// +--------------+-------------+-----------------+---------------+-------+
enum mm_lights_ctl_sinfo_bf
{
    /// Relaying of sent status message is authorized
    MM_LIGHTS_CTL_SINFO_RELAY_POS = 0,
    MM_LIGHTS_CTL_SINFO_RELAY_BIT = CO_BIT(MM_LIGHTS_CTL_SINFO_RELAY_POS),

    /// Wait for start of transition for Light CTL Lightness state
    MM_LIGHTS_CTL_SINFO_WAIT_LN_POS = 1,
    MM_LIGHTS_CTL_SINFO_WAIT_LN_BIT = CO_BIT(MM_LIGHTS_CTL_SINFO_WAIT_LN_POS),

    /// Wait for start of transition for Light CTL Temperature state
    MM_LIGHTS_CTL_SINFO_WAIT_TEMP_POS = 2,
    MM_LIGHTS_CTL_SINFO_WAIT_TEMP_BIT = CO_BIT(MM_LIGHTS_CTL_SINFO_WAIT_TEMP_POS),

    /// Wait for transition start (either Light CTL Lightness or Light CTL Temperature or both)
    MM_LIGHTS_CTL_SINFO_WAIT_LSB = 1,
    MM_LIGHTS_CTL_SINFO_WAIT_MASK = 0x06,

    /// Indicate if Light CTL Status (0) or Light CTL Temperature Status message must be sent
    MM_LIGHTS_CTL_SINFO_TEMP_STATUS_POS = 3,
    MM_LIGHTS_CTL_SINFO_TEMP_STATUS_BIT = CO_BIT(MM_LIGHTS_CTL_SINFO_TEMP_STATUS_POS),
};

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

typedef struct mm_lights_ctl_env mm_lights_ctl_env_t;
typedef struct mm_lights_ctls_env mm_lights_ctls_env_t;
typedef struct mm_lights_ctlt_env mm_lights_ctlt_env_t;

/// Structure for Light CTL Server model environment
struct mm_lights_ctl_env
{
    /// Basic model environment - Must be first element in the structure - DO NOT MOVE
    mm_tb_state_mdl_env_t env;
    /// Timer for sending of publications
    mesh_tb_timer_t tmr_publi;
    /// Publication period in milliseconds
    uint32_t publi_period_ms;

    /// Environment for replay protection mechanism
    mm_tb_replay_mdl_env_t replay_env;

    /// Pointer to environment of associated Light CTL Temperature model
    mm_lights_ctlt_env_t *p_env_ctlt;

    /// Delta value in case of move transition for Light CTL Lightness state value
    int16_t move_delta;
    /// Light CTL Lightness state value
    uint16_t ln;
    /// Target Light CTL Lightness state value
    uint16_t ln_tgt;
    /// Light CTL Delta UV state value
    int16_t delta_uv;
    /// Target Light CTL Delta UV state value
    int16_t delta_uv_tgt;
    /// Light CTL Delta Default state value
    int16_t delta_uv_dflt;

    /// Source address of set message that has triggered last or current transition
    uint16_t status_dst_addr;
    /// Application key local index to be used for transmission of Light CTL Status
    /// or Light CTL Linear Status message
    m_lid_t status_app_key_lid;
    /// Status information bitfield (@see enum mm_lights_ctl_sinfo_bf)
    uint8_t status_info;
};

/// Structure for Light CTL Setup Server model environment
struct mm_lights_ctls_env
{
    /// Basic model environment - Must be first element in the structure - DO NOT MOVE
    mm_tb_state_mdl_env_t env;

    /// Pointer to environment of associated Light CTL Server model
    mm_lights_ctl_env_t *p_env_ctl;
};

/// Structure for Light CTL Temperature Server model environment
struct mm_lights_ctlt_env
{
    /// Basic model environment - Must be first element in the structure - DO NOT MOVE
    mm_tb_state_mdl_env_t env;
    /// Timer for sending of publications
    mesh_tb_timer_t tmr_publi;
    /// Publication period in milliseconds
    uint32_t publi_period_ms;

    /// Pointer to environment of associated Light CTL Temperature model
    mm_lights_ctl_env_t *p_env_ctl;

    /// Delta value in case of move transition for Light CTL Temperature state value
    int16_t move_delta;
    /// Light CTL Temperature
    uint16_t temp;
    /// Target Light CTL Temperature
    uint16_t temp_tgt;
    /// Light CTL Temperature Default
    uint16_t temp_dflt;
    /// Light CTL Temperature Range Min state value
    uint16_t temp_min;
    /// Light CTL Temperature Range Max state value
    uint16_t temp_max;
};

/*
 * LOCAL FUNCTIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Prepare and send a Light CTL Status message.
 *
 * @param[in] p_env_ctl         Pointer to Light CTL Server model environment
 * @param[in] p_route_env       Pointer to structure containing reception information provided
 * by Mesh Profile for received request message
 * @param[in] publish           Indicate if message is sent as a publication (true) or as
 * a response to a received request (false)
 ****************************************************************************************
 */
__STATIC void mm_lights_ctl_send_status(mm_lights_ctl_env_t *p_env_ctl,
                                        mm_route_buf_env_t *p_route_env, bool publish)
{
    // Get environment for Light CTL Temperature Server model
    mm_lights_ctlt_env_t *p_env_ctlt = p_env_ctl->p_env_ctlt;
    // Pointer to the buffer that will contain the message
    mesh_tb_buf_t *p_buf_status;
    // Remaining time for Light CTL Lightness state and for Light CTL Temperature
    // state transition
    uint8_t ln_rem_time, temp_rem_time;
    // Transition type for Light CTL Lightness and for Light CTL Temperature states
    uint8_t ln_trans_type, temp_trans_type;
    // Data length
    uint8_t data_length;
    uint16_t status;
    UNUSED(status);
    // Check if a transition has been started
    mm_tb_bind_get_trans_info(p_env_ctl->env.grp_lid, &ln_trans_type, &ln_rem_time);
    mm_tb_bind_get_trans_info(p_env_ctlt->env.grp_lid, &temp_trans_type, &temp_rem_time);

    // Deduce deduce data length
    data_length = ((ln_trans_type != MM_TRANS_TYPE_NONE) || (temp_trans_type != MM_TRANS_TYPE_NONE))
                  ? MM_LIGHT_CTL_STATUS_LEN : MM_LIGHT_CTL_STATUS_MIN_LEN;

    if ((status = mm_route_buf_alloc(&p_buf_status, data_length)) == MESH_ERR_NO_ERROR)
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
            p_buf_env->app_key_lid = p_env_ctl->status_app_key_lid;
            p_buf_env->u_addr.dst = p_env_ctl->status_dst_addr;
            SETB(p_buf_env->info, MM_ROUTE_BUF_INFO_RELAY,
                 GETB(p_env_ctl->status_info, MM_LIGHTS_CTL_SINFO_RELAY));
        }

        SETB(p_buf_env->info, MM_ROUTE_BUF_INFO_RX, 0);
        SETB(p_buf_env->info, MM_ROUTE_BUF_INFO_PUBLISH, 0);
        p_buf_env->mdl_lid = p_env_ctl->env.mdl_lid;
        p_buf_env->opcode = MM_MSG_LIGHT_CTL_STATUS;

        // Fill the message
        co_write16p(p_data + MM_LIGHT_CTL_STATUS_LIGHTNESS_POS, p_env_ctl->ln);
        co_write16p(p_data + MM_LIGHT_CTL_STATUS_TEMP_POS, p_env_ctlt->temp);

        if (data_length == MM_LIGHT_CTL_STATUS_LEN)
        {
            // Sent Target Light CTL Temperature and Light CTL Lightness state values
            uint16_t temp_tgt, ln_tgt;
            // Sent Remaining Time value
            uint8_t rem_time = 0;

            if (ln_trans_type != MM_TRANS_TYPE_NONE)
            {
                if (ln_trans_type == MM_TRANS_TYPE_MOVE)
                {
                    // Light Lightness Range
                    uint16_t ln_min = mm_lights_ln_get(p_env_ctl->env.elmt_idx,
                                                       MM_STATE_LIGHT_LN_RANGE_MIN);
                    uint16_t ln_max = mm_lights_ln_get(p_env_ctl->env.elmt_idx,
                                                       MM_STATE_LIGHT_LN_RANGE_MAX);

                    ln_tgt = (p_env_ctl->move_delta > 0) ? ln_max : ln_min;
                    rem_time = MM_TRANS_TIME_UNKNOWN;
                }
                else
                {
                    ln_tgt = p_env_ctl->ln_tgt;
                    rem_time = ln_rem_time;
                }
            }
            else
            {
                ln_tgt = p_env_ctl->ln;
            }

            if (temp_trans_type != MM_TRANS_TYPE_NONE)
            {
                if (temp_trans_type == MM_TRANS_TYPE_MOVE)
                {
                    temp_tgt = (p_env_ctlt->move_delta > 0) ? p_env_ctlt->temp_max
                               : p_env_ctlt->temp_min;
                    rem_time = MM_TRANS_TIME_UNKNOWN;
                }
                else
                {
                    temp_tgt = p_env_ctlt->temp_tgt;

                    if (rem_time != MM_TRANS_TIME_UNKNOWN)
                    {
                        if (temp_rem_time > ln_rem_time)
                        {
                            rem_time = temp_rem_time;
                        }
                    }
                }
            }
            else
            {
                temp_tgt = p_env_ctlt->temp;
            }

            co_write16p(p_data + MM_LIGHT_CTL_STATUS_TGT_LIGHTNESS_POS, ln_tgt);
            co_write16p(p_data + MM_LIGHT_CTL_STATUS_TGT_TEMP_POS, temp_tgt);
            *(p_data + MM_LIGHT_CTL_STATUS_REM_TIME_POS) = rem_time;
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
 * @brief Prepare and send a Light CTL Temperature Status message
 *
 * @param[in] p_env_ctlt         Pointer to Light CTL Temperature Server model environment
 * @param[in] p_route_env        Pointer to structure containing reception information provided
 * by Mesh Profile for received request message
 * @param[in] publish            Indicate if message is sent as a publication (true) or as
 * a response to a received request (false)
 ****************************************************************************************
 */
__STATIC void mm_lights_ctl_send_status_temp(mm_lights_ctlt_env_t *p_env_ctlt,
        mm_route_buf_env_t *p_route_env, bool publish)
{
    // Get environment for Light CTL Server model
    mm_lights_ctl_env_t *p_env_ctl = p_env_ctlt->p_env_ctl;
    // Pointer to the buffer that will contain the message
    mesh_tb_buf_t *p_buf_status;
    // Remaining time for Light CTL Lightness state and for Light CTL Temperature
    // state transition
    uint8_t ln_rem_time, temp_rem_time;;
    // Transition type for Light CTL Lightness and for Light CTL Temperature states
    uint8_t ln_trans_type, temp_trans_type;
    // Data length
    uint8_t data_length;

    // Check if a transition has been started
    mm_tb_bind_get_trans_info(p_env_ctl->env.grp_lid, &ln_trans_type, &ln_rem_time);
    mm_tb_bind_get_trans_info(p_env_ctlt->env.grp_lid, &temp_trans_type, &temp_rem_time);

    // Deduce deduce data length
    data_length = ((ln_trans_type != MM_TRANS_TYPE_NONE) || (temp_trans_type != MM_TRANS_TYPE_NONE))
                  ? MM_LIGHT_CTL_TEMP_STATUS_LEN : MM_LIGHT_CTL_TEMP_STATUS_MIN_LEN;

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
            p_buf_env->app_key_lid = p_env_ctl->status_app_key_lid;
            p_buf_env->u_addr.dst = p_env_ctl->status_dst_addr;
            SETB(p_buf_env->info, MM_ROUTE_BUF_INFO_RELAY,
                 GETB(p_env_ctl->status_info, MM_LIGHTS_CTL_SINFO_RELAY));
        }

        SETB(p_buf_env->info, MM_ROUTE_BUF_INFO_RX, 0);
        SETB(p_buf_env->info, MM_ROUTE_BUF_INFO_PUBLISH, 0);
        p_buf_env->mdl_lid = p_env_ctlt->env.mdl_lid;
        p_buf_env->opcode = MM_MSG_LIGHT_CTL_TEMP_STATUS;

        // Fill the message
        co_write16p(p_data + MM_LIGHT_CTL_TEMP_STATUS_TEMP_POS, p_env_ctlt->temp);
        co_write16p(p_data + MM_LIGHT_CTL_TEMP_STATUS_DELTA_UV_POS, p_env_ctl->delta_uv);

        if (data_length == MM_LIGHT_CTL_TEMP_STATUS_LEN)
        {
            // Sent Target Light CTL Temperature and Light CTL Delta UV state values
            uint16_t temp_tgt, delta_uv_tgt;
            // Sent Remaining Time value
            uint8_t rem_time = 0;

            if (ln_trans_type != MM_TRANS_TYPE_NONE)
            {
                delta_uv_tgt = p_env_ctl->delta_uv_tgt;
                rem_time = ln_rem_time;
            }
            else
            {
                delta_uv_tgt = p_env_ctl->delta_uv;
            }

            if (temp_trans_type != MM_TRANS_TYPE_NONE)
            {
                if (temp_trans_type == MM_TRANS_TYPE_MOVE)
                {
                    temp_tgt = (p_env_ctlt->move_delta > 0) ? p_env_ctlt->temp_max
                               : p_env_ctlt->temp_min;
                    rem_time = MM_TRANS_TIME_UNKNOWN;
                }
                else
                {
                    temp_tgt = p_env_ctlt->temp_tgt;

                    if (temp_rem_time > ln_rem_time)
                    {
                        rem_time = temp_rem_time;
                    }
                }
            }
            else
            {
                temp_tgt = p_env_ctlt->temp;
            }

            co_write16p(p_data + MM_LIGHT_CTL_TEMP_STATUS_TGT_TEMP_POS, temp_tgt);
            co_write16p(p_data + MM_LIGHT_CTL_TEMP_STATUS_TGT_DELTA_UV_POS, delta_uv_tgt);
            *(p_data + MM_LIGHT_CTL_TEMP_STATUS_REM_TIME_POS) = rem_time;
        }
        else
        {
            MESH_MODEL_PRINT_WARN("%s, Invalid data length (%d). \n", __func__, data_length);
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
 * @brief Prepare and send a Light CTL Default Status message
 *
 * @param[in] p_env_ctl          Pointer to Light CTL Server model environment
 * @param[in] p_route_env       Pointer to structure containing reception information provided
 * by Mesh Profile for received request message
 ****************************************************************************************
 */
__STATIC void mm_lights_ctl_send_status_dflt(mm_lights_ctl_env_t *p_env_ctl,
        mm_route_buf_env_t *p_route_env)
{
    // Pointer to the buffer that will contain the message
    mesh_tb_buf_t *p_buf_status;
    // Allocate a new buffer for the publication
    uint8_t status = mm_route_buf_alloc_status(&p_buf_status,
                     MM_LIGHT_CTL_DFLT_STATUS_LEN, p_route_env);

    if (status == MESH_ERR_NO_ERROR)
    {
        // Get pointer to environment for Light Lightness CTL Temperature Server model
        mm_lights_ctlt_env_t *p_env_ctlt = p_env_ctl->p_env_ctlt;
        // Get pointer to data
        uint8_t *p_data = MESH_TB_BUF_DATA(p_buf_status);
        // Get pointer to environment
        mm_route_buf_env_t *p_buf_env = (mm_route_buf_env_t *)&p_buf_status->env;

        // Prepare environment
        p_buf_env->mdl_lid = p_env_ctl->env.mdl_lid;
        p_buf_env->opcode = MM_MSG_LIGHT_CTL_DFLT_STATUS;

        // Fill the message
        co_write16p(p_data + MM_LIGHT_CTL_DFLT_STATUS_LIGHTNESS_POS,
                    mm_lights_ln_get(p_env_ctl->env.elmt_idx, MM_STATE_LIGHT_LN_DFLT));
        co_write16p(p_data + MM_LIGHT_CTL_DFLT_STATUS_TEMP_POS, p_env_ctlt->temp_dflt);
        co_write16p(p_data + MM_LIGHT_CTL_DFLT_STATUS_DELTA_UV_POS, p_env_ctl->delta_uv_dflt);

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
 * @brief Prepare and send a Light CTL Temperature Range Status message
 *
 * @param[in] p_env_ctlt        Pointer to Light CTL Temperature Server model environment
 * @param[in] p_route_env       Pointer to structure containing reception information provided
 * by Mesh Profile for received request message
 * @param[in] status            Status sent in the Light CTL Temperature Status message
 * (@see enum mm_status)
 ****************************************************************************************
 */
__STATIC void mm_lights_ctl_send_status_temp_range(mm_lights_ctl_env_t *p_env_ctl,
        mm_route_buf_env_t *p_route_env,
        uint8_t status)
{
    // Pointer to the buffer that will contain the message
    mesh_tb_buf_t *p_buf_status;
    // Allocate a new buffer for the publication
    uint16_t alloc_status = mm_route_buf_alloc_status(&p_buf_status,
                            MM_LIGHT_CTL_TEMP_RANGE_STATUS_LEN, p_route_env);

    if (alloc_status == MESH_ERR_NO_ERROR)
    {
        // Pointer to environment for Light CTL Temperature Server model
        mm_lights_ctlt_env_t *p_env_ctlt = p_env_ctl->p_env_ctlt;
        // Get pointer to data
        uint8_t *p_data = MESH_TB_BUF_DATA(p_buf_status);
        // Get pointer to environment
        mm_route_buf_env_t *p_buf_env = (mm_route_buf_env_t *)&p_buf_status->env;

        // Prepare environment
        p_buf_env->mdl_lid = p_env_ctl->env.mdl_lid;
        p_buf_env->opcode = MM_MSG_LIGHT_CTL_TEMP_RANGE_STATUS;

        // Fill the message
        *(p_data + MM_LIGHT_CTL_TEMP_RANGE_STATUS_CODE_POS) = status;
        co_write16p(p_data + MM_LIGHT_CTL_TEMP_RANGE_STATUS_MIN_POS, p_env_ctlt->temp_min);
        co_write16p(p_data + MM_LIGHT_CTL_TEMP_RANGE_STATUS_MAX_POS, p_env_ctlt->temp_max);

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
 * @brief Publish Light CTL state value if sending of publications is enabled
 *
 * @param[in] p_env_ctl         Pointer to Light CTL Server model environment
 ****************************************************************************************
 */
__STATIC void mm_lights_ctl_publish(mm_lights_ctl_env_t *p_env_ctl)
{
    // Check if sending of publication is enabled
    if (GETB(p_env_ctl->env.info, MM_TB_STATE_MDL_INFO_PUBLI))
    {
        mm_lights_ctl_send_status(p_env_ctl, NULL, true);
    }
}

/**
 ****************************************************************************************
 * @brief Publish Light CTL Temperature state value if sending of publications is enabled
 *
 * @param[in] p_env_ctlt         Pointer to Light CTL Temperature Server model environment
 ****************************************************************************************
 */
__STATIC void mm_lights_ctl_publish_temp(mm_lights_ctlt_env_t *p_env_ctlt)
{
    // Check if sending of publication is enabled
    if (GETB(p_env_ctlt->env.info, MM_TB_STATE_MDL_INFO_PUBLI))
    {
        mm_lights_ctl_send_status_temp(p_env_ctlt, NULL, true);
    }
}

/**
 ****************************************************************************************
 * @brief Check if a Light CTL Status or a Light CTL Temperature Status message must be sent
 * and send it if it is the case
 *
 * @param[in] p_env_ctl        Pointer to Light CTL Server model environment
 ****************************************************************************************
 */
__STATIC void mm_lights_ctl_check_status_rsp(mm_lights_ctl_env_t *p_env_ctl)
{
    if ((p_env_ctl->status_dst_addr != MESH_UNASSIGNED_ADDR)
            && !GETF(p_env_ctl->status_info, MM_LIGHTS_CTL_SINFO_WAIT))
    {
        // Send a response to the node that has required the transition
        if (!GETB(p_env_ctl->status_info, MM_LIGHTS_CTL_SINFO_TEMP_STATUS))
        {
            mm_lights_ctl_send_status(p_env_ctl, NULL, false);
        }
        else
        {
            mm_lights_ctl_send_status_temp(p_env_ctl->p_env_ctlt, NULL, false);
        }

        p_env_ctl->status_dst_addr = MESH_UNASSIGNED_ADDR;
    }
}

/**
 ****************************************************************************************
 * @brief Convert a Generic Level value into a Light CTL Temperature value
 *
 * @param[in] lvl           Generic level value
 * @param[in] temp_min      Light CTL Temperature Min value
 * @param[in] temp_max      Light CTL Temperature Max value
 *
 * @return Light CTL Temperature value
 ****************************************************************************************
 */
__STATIC uint16_t mm_lights_ctl_lvl_to_temp(int16_t lvl, uint16_t temp_min, uint16_t temp_max)
{
    // Temperature value
    uint32_t temp = (int32_t)lvl + 32768;

    temp *= (temp_max - temp_min);
    temp /= 65535;
    temp += temp_min;

    return ((uint16_t)temp);
}

/**
 ****************************************************************************************
 * @brief Convert a Light CTL Temperature value into a Generic Level value
 *
 * @param[in] temp          Light CTL Temperature value
 * @param[in] temp_min      Light CTL Temperature Min value
 * @param[in] temp_max      Light CTL Temperature Max value
 *
 * @return Generic Level value
 ****************************************************************************************
 */
__STATIC int16_t mm_lights_ctl_temp_to_lvl(uint16_t temp, uint16_t temp_min, uint16_t temp_max)
{
    // Level value
    int32_t lvl = ((int32_t)temp - temp_min);

    lvl *= 65535;
    lvl /= (temp_max - temp_min);
    lvl -= 32768;

    return ((int16_t)lvl);
}

/**
 ****************************************************************************************
 * @brief Convert a Light CTL Temperature value into a Generic Level value
 *
 * @param[in] temp          Light CTL Temperature value
 * @param[in] temp_min      Light CTL Temperature Min value
 * @param[in] temp_max      Light CTL Temperature Max value
 *
 * @return Generic Level value
 ****************************************************************************************
 */
__STATIC uint16_t mm_lights_ctl_temp_add_delta(uint16_t temp, int32_t delta,
        uint16_t temp_min, uint16_t temp_max)
{
    delta *= (temp_max - temp_min);
    delta /= 65535;
    delta += temp;

    return ((uint16_t)delta);
}

/*
 * MESSAGE HANDLERS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Handler for Light CTL Set/Set Unacknowledged message
 *
 * @param[in] p_env         Pointer to environment of model for which message has been received
 * @param[in] p_buf         Pointer to buffer containing the received message
 * @param[in] p_route_env   Pointer to routing information for the received buffer
 ****************************************************************************************
 */
__STATIC void mm_lights_ctl_handler_set(mm_lights_ctl_env_t *p_env_ctl, mesh_tb_buf_t *p_buf,
                                        mm_route_buf_env_t *p_route_env)
{
    do
    {
        // Get environment for Light CTL Temperature Server model
        mm_lights_ctlt_env_t *p_env_ctlt = p_env_ctl->p_env_ctlt;
        // Check if a status message must be sent
        bool send_status = (p_route_env->opcode == MM_MSG_LIGHT_CTL_SET);
        // Get pointer to data
        uint8_t *p_data = MESH_TB_BUF_DATA(p_buf);
        // TID value
        uint8_t tid = *(p_data + MM_LIGHT_CTL_SET_TID_POS);
        // Light CTL Lightness, Light CTL Temperature and Light CTL Delta UV value
        uint16_t ln, temp;
        int16_t delta_uv;
        // Transition time
        uint8_t trans_time;
        // Delay
        uint8_t delay;

        MESH_MODEL_PRINT_DEBUG("%s opcode :0x%x\r\n", __func__, p_route_env->opcode);



        // Check if request can be processed
        if ((p_env_ctl->status_dst_addr != MESH_UNASSIGNED_ADDR)
                || (mm_tb_replay_is_retx(&p_env_ctl->replay_env, p_route_env->u_addr.src, tid)))
        {
            // Send a Light CTL Status message
            if (send_status)
            {
                MESH_MODEL_PRINT_DEBUG("ctl_send_status 0\r\n");
                mm_lights_ctl_send_status(p_env_ctl, p_route_env, false);
            }
            break;
        }

        // Extract and check optional parameters if present
        if (p_buf->data_len == MM_LIGHT_CTL_SET_LEN)
        {
            trans_time = (uint16_t)(*(p_data + MM_LIGHT_CTL_SET_TRANS_TIME_POS));

            // Check received value
            if (GETF(trans_time, MM_TRANS_TIME_STEP_NB) > MM_TRANS_TIME_NB_STEPS_MAX)
            {
                // Drop the message
                break;
            }

            delay = *(p_data + MM_LIGHT_CTL_SET_DELAY_POS);
        }
        else
        {
            trans_time = MM_TRANS_TIME_UNKNOWN;
            delay = 0;
        }

        ln = co_read16p(p_data + MM_LIGHT_CTL_SET_LIGHTNESS_POS);
        temp = co_read16p(p_data + MM_LIGHT_CTL_SET_TEMP_POS);
        delta_uv = co_read16p(p_data + MM_LIGHT_CTL_SET_DELTA_UV_POS);

        MESH_MODEL_PRINT_DEBUG("ln :%d,p_ln:%d\r\n", ln, p_env_ctl->ln);
        MESH_MODEL_PRINT_DEBUG("temp :%d,p_temp:%d\r\n", temp, p_env_ctl->ln);
        MESH_MODEL_PRINT_DEBUG("delta_uv :%d,p_delta_uv:%d\r\n", delta_uv, p_env_ctl->ln);

        // Ensure that Light CTL Temperature state value is between Light CTL Temperature
        // Range Min and Max values
        if (temp > p_env_ctlt->temp_max)
        {
            temp = p_env_ctlt->temp_max;
        }
        else if (temp < p_env_ctlt->temp_min)
        {
            temp = p_env_ctlt->temp_min;
        }

        // Check if at least one of the states is modified
        if ((ln == p_env_ctl->ln)
                && (temp == p_env_ctlt->temp)
                && (delta_uv == p_env_ctl->delta_uv))
        {
            // Send a Light CTL Status message
            if (send_status)
            {
                mm_lights_ctl_send_status(p_env_ctl, p_route_env, false);
            }
            break;
        };

        if (send_status)
        {
            // Keep information for transmission of status
            p_env_ctl->status_dst_addr = p_route_env->u_addr.src;
            p_env_ctl->status_app_key_lid = p_route_env->app_key_lid;
            SETB(p_env_ctl->status_info, MM_LIGHTS_CTL_SINFO_TEMP_STATUS, 0);
            SETB(p_env_ctl->status_info, MM_LIGHTS_CTL_SINFO_RELAY,
                 GETB(p_route_env->info, MM_ROUTE_BUF_INFO_RELAY));
#if 0
            SETB(p_env_ctl->status_info, MM_LIGHTS_CTL_SINFO_WAIT_LN,
                 (ln != p_env_ctl->ln) || (delta_uv != p_env_ctl->delta_uv));
            SETB(p_env_ctl->status_info, MM_LIGHTS_CTL_SINFO_WAIT_TEMP, (temp != p_env_ctlt->temp));
#endif
            SETB(p_env_ctl->status_info, MM_LIGHTS_CTL_SINFO_WAIT_LN, 1);
            SETB(p_env_ctl->status_info, MM_LIGHTS_CTL_SINFO_WAIT_TEMP, 1);
        }

        // if (temp != p_env_ctlt->temp)
        {
            // Update target state
            p_env_ctlt->temp_tgt = temp;

            // Inform the Binding Manager about new transition
            mm_tb_bind_trans_new(p_env_ctlt->env.grp_lid, MM_TRANS_TYPE_CLASSIC, trans_time, delay);
        }

        //   if ((ln != p_env_ctl->ln)
//                || (delta_uv != p_env_ctl->delta_uv))
        {
            // Update target state
            p_env_ctl->ln_tgt = ln;
            p_env_ctl->delta_uv_tgt = delta_uv;

            // Inform the Binding Manager about new transition
            mm_tb_bind_trans_new(p_env_ctl->env.grp_lid, MM_TRANS_TYPE_CLASSIC, trans_time, delay);
        }


    }
    while (0);
}

/**
 ****************************************************************************************
 * @brief Handler for Light CTL Temperature Set/Set Unacknowledged message
 *
 * @param[in] p_env         Pointer to environment of model for which message has been received
 * @param[in] p_buf         Pointer to buffer containing the received message
 * @param[in] p_route_env   Pointer to routing information for the received buffer
 ****************************************************************************************
 */
__STATIC void mm_lights_ctl_handler_set_temp(mm_lights_ctlt_env_t *p_env_ctlt, mesh_tb_buf_t *p_buf,
        mm_route_buf_env_t *p_route_env)
{
    do
    {
        // Get pointer to environment for Light Lightness CTL Server model
        mm_lights_ctl_env_t *p_env_ctl = p_env_ctlt->p_env_ctl;
        // Check if a status message must be sent
        bool send_status = (p_route_env->opcode == MM_MSG_LIGHT_CTL_TEMP_SET);
        // Get pointer to data
        uint8_t *p_data = MESH_TB_BUF_DATA(p_buf);
        // Received ight CTL Temperature and Light CTL Delta UV state values
        uint16_t temp;
        int16_t delta_uv;
        // Transition time
        uint8_t trans_time = MM_TRANS_TIME_UNKNOWN;
        // Delay
        uint8_t delay = 0;
        // TID value
        uint8_t tid = *(p_data + MM_LIGHT_CTL_TEMP_SET_TID_POS);;

        // Check if request can be processed
        if ((p_env_ctl->status_dst_addr != MESH_UNASSIGNED_ADDR)
                || (mm_tb_replay_is_retx(&p_env_ctl->replay_env, p_route_env->u_addr.src, tid)))
        {
            // Send a Light CTL Temperature Status message
            if (send_status)
            {
                mm_lights_ctl_send_status_temp(p_env_ctlt, p_route_env, false);
            }
            break;
        }

        // Extract and check optional parameters if present
        if (p_buf->data_len == MM_LIGHT_CTL_TEMP_SET_LEN)
        {
            trans_time = (uint16_t)(*(p_data + MM_LIGHT_CTL_TEMP_SET_TRANS_TIME_POS));

            // Check received value
            if (GETF(trans_time, MM_TRANS_TIME_STEP_NB) > MM_TRANS_TIME_NB_STEPS_MAX)
            {
                // Drop the message
                break;
            }

            delay = *(p_data + MM_LIGHT_CTL_TEMP_SET_DELAY_POS);
        }

        temp = co_read16p(p_data + MM_LIGHT_CTL_TEMP_SET_TEMP_POS);
        delta_uv = co_read16p(p_data + MM_LIGHT_CTL_TEMP_SET_DELTA_UV_POS);

        // Ensure that Light CTL Temperature state value is between Light CTL Temperature Range
        // Min and Max values
        if (temp > p_env_ctlt->temp_max)
        {
            temp = p_env_ctlt->temp_max;
        }
        else if (temp < p_env_ctlt->temp_min)
        {
            temp = p_env_ctlt->temp_min;
        }

        // Check if at least one of the states is modified
        if ((temp == p_env_ctlt->temp)
                && (delta_uv == p_env_ctl->delta_uv))
        {
            // Send a Light CTL Temperature Status message
            if (send_status)
            {
                mm_lights_ctl_send_status_temp(p_env_ctlt, p_route_env, false);
            }
            break;
        };

        if (send_status)
        {
            // Keep information for transmission of status
            p_env_ctl->status_dst_addr = p_route_env->u_addr.src;
            p_env_ctl->status_app_key_lid = p_route_env->app_key_lid;
            SETB(p_env_ctl->status_info, MM_LIGHTS_CTL_SINFO_TEMP_STATUS, 1);
            SETB(p_env_ctl->status_info, MM_LIGHTS_CTL_SINFO_RELAY,
                 GETB(p_route_env->info, MM_ROUTE_BUF_INFO_RELAY));
            SETB(p_env_ctl->status_info, MM_LIGHTS_CTL_SINFO_WAIT_LN,
                 (delta_uv != p_env_ctl->delta_uv));
            SETB(p_env_ctl->status_info, MM_LIGHTS_CTL_SINFO_WAIT_TEMP, (temp != p_env_ctlt->temp));
        }

        // Check if Light CTL Temperature value is modified
        if (temp != p_env_ctlt->temp)
        {
            // Update target state
            p_env_ctlt->temp_tgt = temp;

            // Inform the Binding Manager about new transition
            mm_tb_bind_trans_new(p_env_ctlt->env.grp_lid, MM_TRANS_TYPE_CLASSIC, trans_time, delay);
        }

        // Check if Light CTL Delta UV value is modified
        if (delta_uv != p_env_ctl->delta_uv)
        {
            // Update target state
            p_env_ctl->delta_uv_tgt = delta_uv;
            p_env_ctl->ln_tgt = p_env_ctl->ln;

            // Inform the Binding Manager about new transition
            mm_tb_bind_trans_new(p_env_ctl->env.grp_lid, MM_TRANS_TYPE_CLASSIC, trans_time, delay);
        }
    }
    while (0);
}

/**
 ****************************************************************************************
 * @brief Handler for Light CTL Default Set/Set Unacknowledged message
 *
 * @param[in] p_env         Pointer to environment of model for which message has been received
 * @param[in] p_buf         Pointer to buffer containing the received message
 * @param[in] p_route_env   Pointer to routing information for the received buffer
 ****************************************************************************************
 */
__STATIC void mm_lights_ctl_handler_set_dflt(mm_lights_ctl_env_t *p_env_ctl,
        mesh_tb_buf_t *p_buf, mm_route_buf_env_t *p_route_env)
{
    // Get pointer to data
    uint8_t *p_data = MESH_TB_BUF_DATA(p_buf);
    // Get environment for Light Lightness CTL Temperature Server model
    mm_lights_ctlt_env_t *p_env_ctlt = p_env_ctl->p_env_ctlt;
    // Extract received state values
    uint16_t ln_dflt = co_read16p(p_data + MM_LIGHT_CTL_DFLT_SET_LIGHTNESS_POS);
    uint16_t temp_dflt = co_read16p(p_data + MM_LIGHT_CTL_DFLT_SET_TEMP_POS);
    uint16_t delta_uv_dflt = co_read16p(p_data + MM_LIGHT_CTL_DFLT_SET_DELTA_UV_POS);

    // Inform Light Lightness Server model about received Light Lightness Default state value
    mm_lights_ln_set_dflt(p_env_ctl->env.elmt_idx, ln_dflt);

    if (temp_dflt != p_env_ctlt->temp_dflt)
    {
        // Keep received value
        p_env_ctlt->temp_dflt = temp_dflt;

        // Inform application about received value
        mm_api_send_srv_state_upd_ind(MM_STATE_LIGHT_CTL_TEMP_DFLT,
                                      p_env_ctlt->env.elmt_idx, temp_dflt, 0);
    }

    if (delta_uv_dflt != p_env_ctl->delta_uv_dflt)
    {
        // Keep received value
        p_env_ctl->delta_uv_dflt = delta_uv_dflt;

        // Inform application about received value
        mm_api_send_srv_state_upd_ind(MM_STATE_LIGHT_CTL_DELTA_UV_DFLT,
                                      p_env_ctl->env.elmt_idx, delta_uv_dflt, 0);
    }

    // If needed, send a Light CTL Default Status message to the requester
    if (p_route_env->opcode == MM_MSG_LIGHT_CTL_DFLT_SET)
    {
        mm_lights_ctl_send_status_dflt(p_env_ctl, p_route_env);
    }
}

/**
 ****************************************************************************************
 * @brief Handler for Light CTL Temperature Range Set/Set Unacknowledged message
 *
 * @param[in] p_env         Pointer to environment of model for which message has been received
 * @param[in] p_buf         Pointer to buffer containing the received message
 * @param[in] p_route_env   Pointer to routing information for the received buffer
 ****************************************************************************************
 */
__STATIC void mm_lights_ctl_handler_set_temp_range(mm_lights_ctl_env_t *p_env_ctl,
        mesh_tb_buf_t *p_buf,
        mm_route_buf_env_t *p_route_env)
{
    do
    {
        // Get pointer to data
        uint8_t *p_data = MESH_TB_BUF_DATA(p_buf);
        // Get environment for Light Lightness CTL Temperature Server model
        mm_lights_ctlt_env_t *p_env_ctlt = p_env_ctl->p_env_ctlt;
        // Status
        uint8_t status = MM_STATUS_SUCCESS;
        // Extract Light CTL Temperature Range state value
        uint16_t temp_min = co_read16p(p_data + MM_LIGHT_CTL_TEMP_RANGE_SET_MIN_POS);
        // Extract Light CTL Range state value
        uint16_t temp_max = co_read16p(p_data + MM_LIGHT_CTL_TEMP_RANGE_SET_MAX_POS);

        // Check provided values
        if (temp_min < MM_LIGHTS_CTL_TEMP_MIN)
        {
            status = MM_STATUS_CANNOT_SET_RANGE_MIN;
        }
        else if (temp_max > MM_LIGHTS_CTL_TEMP_MAX)
        {
            status = MM_STATUS_CANNOT_SET_RANGE_MAX;
        }
        else if (temp_min > temp_max)
        {
            // Drop the message
            break;
        }

        if ((status == MM_STATUS_SUCCESS)
                && ((p_env_ctlt->temp_min != temp_min)
                    || (p_env_ctlt->temp_max != temp_max)))
        {
            p_env_ctlt->temp_min = temp_min;
            p_env_ctlt->temp_max = temp_max;

            // Inform application about received value
            mm_api_send_srv_state_upd_ind(MM_STATE_LIGHT_CTL_TEMP_RANGE, p_env_ctlt->env.elmt_idx,
                                          (uint32_t)temp_min | ((uint32_t)temp_max << 16), 0);
        }

        // If needed, send a Light CTL Temperature Range Status message to the requester
        if (p_route_env->opcode == MM_MSG_LIGHT_CTL_TEMP_RANGE_SET)
        {
            mm_lights_ctl_send_status_temp_range(p_env_ctl, p_route_env, status);
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
 * Light CTL Server model or for Light CTL Temperature expires
 *
 * @param[in] p_env     Pointer to model environment for Light CTL Server or Light CTL
 * Temperature model
 ****************************************************************************************
 */
__STATIC void mm_lights_ctl_cb_tmr_publi(void *p_env)
{
    // Get allocated environment
    mm_tb_state_mdl_publi_env_t *p_env_publi = (mm_tb_state_mdl_publi_env_t *)p_env;

    if (p_env_publi->publi_period_ms)
    {
        if (p_env_publi->env.mdl_id == MM_ID_LIGHTS_CTL)
        {
            // Publish a Light CTL Status message
            mm_lights_ctl_publish((mm_lights_ctl_env_t *)p_env);
        }
        else     // (p_env_publi->env.mdl_id == MM_ID_LIGHTS_CTLT)
        {
            // Publish a Light CTL Temperature Status message
            mm_lights_ctl_publish_temp((mm_lights_ctlt_env_t *)p_env);
        }

        // Restart the timer
        mesh_tb_timer_set(&p_env_publi->tmr_publi, p_env_publi->publi_period_ms);
    }
}

/**
 ****************************************************************************************
 * @brief Inform Light CTL Server model about a received message
 *
 * @param[in] p_env         Pointer to the environment allocated for the Light CTL
 * Server model
 * @param[in] p_buf         Pointer to the buffer containing the received message
 * @param[in] p_route_env   Pointer to structure containing reception parameters provided
 * by the Mesh Profile block
 ****************************************************************************************
 */
__STATIC void mm_lights_ctl_cb_rx(mm_tb_state_mdl_env_t *p_env, mesh_tb_buf_t *p_buf,
                                  mm_route_buf_env_t *p_route_env)
{
    do
    {
        // Environment for Light CTL Server model
        mm_lights_ctl_env_t *p_env_ctl;
        {
            MESH_MODEL_PRINT_DEBUG("%s opcode :0x%x\r\n", __func__, p_route_env->opcode);
            uint8_t *p_data = MESH_TB_BUF_DATA(p_buf);
            MESH_MODEL_PRINT_DEBUG("###############################\r\n");
            MESH_MODEL_PRINT_DEBUG("data: %s \n", mesh_buffer_to_hex(p_data, p_buf->data_len));
            MESH_MODEL_PRINT_DEBUG("###############################\r\n");
        }
        if (p_env->mdl_id == MM_ID_LIGHTS_CTL)
        {
            p_env_ctl = (mm_lights_ctl_env_t *)p_env;

            switch (p_route_env->opcode)
            {
                case (MM_MSG_LIGHT_CTL_GET):
                {
                    // Send a Light CTL Status message
                    mm_lights_ctl_send_status(p_env_ctl, p_route_env, false);
                } break;

                case (MM_MSG_LIGHT_CTL_SET):
                case (MM_MSG_LIGHT_CTL_SET_UNACK):
                {
                    // Handle the message
                    mm_lights_ctl_handler_set(p_env_ctl, p_buf, p_route_env);
                } break;

                case (MM_MSG_LIGHT_CTL_TEMP_RANGE_GET):
                {
                    // Send a Light CTL Temperature Range Status message
                    mm_lights_ctl_send_status_temp_range(p_env_ctl, p_route_env, MM_STATUS_SUCCESS);
                } break;

                case (MM_MSG_LIGHT_CTL_DFLT_GET):
                {
                    // Send a Light CTL Default Status message
                    mm_lights_ctl_send_status_dflt(p_env_ctl, p_route_env);
                } break;

                default:
                {
                } break;
            }
        }
        else if (p_env->mdl_id == MM_ID_LIGHTS_CTLS)
        {
            // Environment for Light CTL Setup Server model
            mm_lights_ctls_env_t *p_env_ctls = (mm_lights_ctls_env_t *)p_env;

            p_env_ctl = p_env_ctls->p_env_ctl;

            switch (p_route_env->opcode)
            {
                case (MM_MSG_LIGHT_CTL_DFLT_SET):
                case (MM_MSG_LIGHT_CTL_DFLT_SET_UNACK):
                {
                    // Handle the message
                    mm_lights_ctl_handler_set_dflt(p_env_ctl, p_buf, p_route_env);
                } break;

                case (MM_MSG_LIGHT_CTL_TEMP_RANGE_SET):
                case (MM_MSG_LIGHT_CTL_TEMP_RANGE_SET_UNACK):
                {
                    // Handle the message
                    mm_lights_ctl_handler_set_temp_range(p_env_ctl, p_buf, p_route_env);
                } break;

                default:
                {
                } break;
            }
        }
        else     // (p_env->mdl_id == MM_ID_LIGHTS_CTLT)
        {
            // Environment for Light CTL Temperature model
            mm_lights_ctlt_env_t *p_env_ctlt = (mm_lights_ctlt_env_t *)p_env;

            switch (p_route_env->opcode)
            {
                case (MM_MSG_LIGHT_CTL_TEMP_GET):
                {
                    // Send a Light CTL Temperature Status message
                    mm_lights_ctl_send_status_temp(p_env_ctlt, p_route_env, false);
                } break;

                case (MM_MSG_LIGHT_CTL_TEMP_SET):
                case (MM_MSG_LIGHT_CTL_TEMP_SET_UNACK):
                {
                    // Handle the message
                    mm_lights_ctl_handler_set_temp(p_env_ctlt, p_buf, p_route_env);
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
 * @brief Check if a given opcode is handled by the Light CTL Server model
 *
 * @param[in] p_env         Pointer to the environment allocated for the Light CTL
 * Server model
 * @param[in] opcode        Opcode to check
 *
 * @return An error status (@see enum mesh_err)
 ****************************************************************************************
 */
__STATIC uint16_t mm_lights_ctl_cb_opcode_check(mm_tb_state_mdl_env_t *p_env, uint32_t opcode)
{
    uint16_t status = MESH_ERR_MDL_INVALID_OPCODE;

    if (p_env->mdl_id == MM_ID_LIGHTS_CTL)
    {
        if ((opcode == MM_MSG_LIGHT_CTL_GET)
                || (opcode == MM_MSG_LIGHT_CTL_SET)
                || (opcode == MM_MSG_LIGHT_CTL_SET_UNACK)
                || (opcode == MM_MSG_LIGHT_CTL_TEMP_RANGE_GET)
                || (opcode == MM_MSG_LIGHT_CTL_DFLT_GET))
        {
            status = MESH_ERR_NO_ERROR;
        }
    }
    else if (p_env->mdl_id == MM_ID_LIGHTS_CTLS)
    {
        if ((opcode == MM_MSG_LIGHT_CTL_DFLT_SET)
                || (opcode == MM_MSG_LIGHT_CTL_DFLT_SET_UNACK)
                || (opcode == MM_MSG_LIGHT_CTL_TEMP_RANGE_SET)
                || (opcode == MM_MSG_LIGHT_CTL_TEMP_RANGE_SET_UNACK))
        {
            status = MESH_ERR_NO_ERROR;
        }
    }
    else if (p_env->mdl_id == MM_ID_LIGHTS_CTLT)
    {
        if ((opcode == MM_MSG_LIGHT_CTL_TEMP_GET)
                || (opcode == MM_MSG_LIGHT_CTL_TEMP_SET)
                || (opcode == MM_MSG_LIGHT_CTL_TEMP_SET_UNACK))
        {
            status = MESH_ERR_NO_ERROR;
        }
    }

    if (status != MESH_ERR_NO_ERROR)
    {
        MESH_MODEL_PRINT_INFO("%s, Invalid opcode (0x%x).\n", __func__, opcode);
    }

    return (status);
}

/**
 ****************************************************************************************
 * @brief Inform Light CTL Server model about received publication parameters
 *
 * @param[in] p_env         Pointer the the environment allocated for the Light CT:
 * Server model
 * @param[in] addr          Publication address
 * @param[in] period_ms     Publication period in milliseconds
 ****************************************************************************************
 */
__STATIC void mm_lights_ctl_cb_publish_param(mm_tb_state_mdl_env_t *p_env, uint16_t addr,
        uint32_t period_ms)
{
    // Inform the state manager about received publication parameters
    mm_tb_state_publish_param_ind((mm_tb_state_mdl_publi_env_t *)p_env, addr, period_ms);
}

/**
 ****************************************************************************************
 * @brief
 *
 * @param[in] p_env         Pointer the the environment allocated for the Generic OnOff
 * Server model
 * @param[in] state_id      State identifier
 * @param[in] state         Light CTL Actual or Light CTL Default or Light
 * Lightness Range state value
 *
 * @return An error status (@see enum mesh_err)
 ****************************************************************************************
 */
__STATIC uint16_t mm_lights_ctl_cb_set(mm_tb_state_mdl_env_t *p_env, uint16_t state_id, uint32_t state)
{
    // Returned status
    uint16_t status = MESH_ERR_NO_ERROR;
    // Get environment for the Light CTL Server model
    mm_lights_ctl_env_t *p_env_ctl = (mm_lights_ctl_env_t *)p_env;
    // Get environment for the Light CTL Temperature Server model
    mm_lights_ctlt_env_t *p_env_ctlt = p_env_ctl->p_env_ctlt;

    switch (state_id)
    {
        case (MM_STATE_LIGHT_CTL_LN):
        {
            // Keep the provided state value
            p_env_ctl->ln = state;

            // Set the Light Lightness state value
            mm_tb_bind_set_state(p_env_ctl->env.grp_lid, MM_STATE_TYPE_CURRENT,
                                 MM_ID_LIGHTS_LN, state);
        } break;

        case (MM_STATE_LIGHT_CTL_DELTA_UV):
        {
            // Keep the provided state value
            p_env_ctl->delta_uv = state;
        } break;

        case (MM_STATE_LIGHT_CTL_DELTA_UV_DFLT):
        {
            // Keep the provided state value
            p_env_ctl->delta_uv_dflt = state;
        } break;

        case (MM_STATE_LIGHT_CTL_TEMP):
        {
            // Keep the provided state value
            p_env_ctlt->temp = state;
        } break;

        case (MM_STATE_LIGHT_CTL_TEMP_DFLT):
        {
            // Keep the provided state value
            p_env_ctlt->temp_dflt = state;
        } break;

        case (MM_STATE_LIGHT_CTL_TEMP_RANGE):
        {
            // Keep the provided state value
            p_env_ctlt->temp_min = state;
            p_env_ctlt->temp_max = state;
        } break;

        default:
        {
            status = MESH_ERR_INVALID_PARAM;
        } break;
    }

    return (status);
}

/*
 * GLOBAL FUNCTIONS
 ****************************************************************************************
 */

uint16_t mm_lights_ctl_register(uint8_t elmt_idx, m_lid_t *p_mdl_lid, m_lid_t *p_mdl_lid_temp)
{
    uint16_t status;

    do
    {
        // Pointer to allocated environment for Light CTL Server model
        mm_lights_ctl_env_t *p_env_ctl;
        // Pointer to allocated environment for Light CTL Temperature Server model
        mm_lights_ctlt_env_t *p_env_ctlt;
        // Model local index
        m_lid_t mdl_lid;
        // Pointer to server-specific callback functions
        mm_srv_cb_t *p_cb_srv;

        // Register Light CTL Server model
        status = m_api_register_model(MM_ID_LIGHTS_CTL, elmt_idx, M_MDL_CONFIG_PUBLI_AUTH_BIT,
                                      &mm_route_cb, p_mdl_lid);

        if (status != MESH_ERR_NO_ERROR)
        {
            break;
        }

        // Inform the Model State Manager about registered model
        status = mm_tb_state_register(elmt_idx, MM_ID_LIGHTS_CTL, *p_mdl_lid,
                                      MM_TB_STATE_ROLE_SRV | MM_TB_STATE_CFG_CB_BIT,
                                      sizeof(mm_lights_ctl_env_t));

        if (status != MESH_ERR_NO_ERROR)
        {
            break;
        }

        // Get allocated environment
        p_env_ctl = (mm_lights_ctl_env_t *)mm_tb_state_get_env(*p_mdl_lid);

        // Get server-specific callback functions
        p_cb_srv = p_env_ctl->env.cb.u.p_cb_srv;

        // Prepare environment for Replay Manager
        p_env_ctl->replay_env.delay_ms = MM_LIGHTS_CTL_REPLAY_MS;

        // Prepare timer for publications
        p_env_ctl->tmr_publi.cb = mm_lights_ctl_cb_tmr_publi;
        p_env_ctl->tmr_publi.p_env = (void *)p_env_ctl;

        // Set internal callback functions
        p_env_ctl->env.cb.cb_rx = mm_lights_ctl_cb_rx;
        p_env_ctl->env.cb.cb_opcode_check = mm_lights_ctl_cb_opcode_check;
        p_env_ctl->env.cb.cb_publish_param = mm_lights_ctl_cb_publish_param;
        p_cb_srv->cb_set = mm_lights_ctl_cb_set;

        // Inform application about registered model
        mm_api_send_register_ind(MM_ID_LIGHTS_CTL, elmt_idx, *p_mdl_lid);

        // Register Light CTL Setup Server model
        status = m_api_register_model(MM_ID_LIGHTS_CTLS, elmt_idx, 0, &mm_route_cb, &mdl_lid);

        if (status != MESH_ERR_NO_ERROR)
        {
            break;
        }

        // Inform the Model State Manager about registered model
        status = mm_tb_state_register(elmt_idx, MM_ID_LIGHTS_CTLS, mdl_lid,
                                      MM_TB_STATE_ROLE_SRV, sizeof(mm_lights_ctls_env_t));

        if (status == MESH_ERR_NO_ERROR)
        {
            // Get allocated environment
            mm_lights_ctls_env_t *p_env_ctls = (mm_lights_ctls_env_t *)mm_tb_state_get_env(mdl_lid);

            // Set internal callback functions
            p_env_ctls->env.cb.cb_rx = mm_lights_ctl_cb_rx;
            p_env_ctls->env.cb.cb_opcode_check = mm_lights_ctl_cb_opcode_check;

            // Link environment
            p_env_ctls->p_env_ctl = p_env_ctl;

            // Inform application about registered model
            mm_api_send_register_ind(MM_ID_LIGHTS_CTLS, elmt_idx, mdl_lid);
        }

        // Register Light CTL Temperature Server model
        status = m_api_register_model(MM_ID_LIGHTS_CTLT, elmt_idx + 1, M_MDL_CONFIG_PUBLI_AUTH_BIT,
                                      &mm_route_cb, p_mdl_lid_temp);

        if (status != MESH_ERR_NO_ERROR)
        {
            break;
        }

        // Inform the Model State Manager about registered model
        status = mm_tb_state_register(elmt_idx + 1, MM_ID_LIGHTS_CTLT, *p_mdl_lid_temp,
                                      MM_TB_STATE_ROLE_SRV | MM_TB_STATE_CFG_CB_BIT,
                                      sizeof(mm_lights_ctlt_env_t));

        if (status != MESH_ERR_NO_ERROR)
        {
            break;
        }

        // Get allocated environment
        p_env_ctlt = (mm_lights_ctlt_env_t *)mm_tb_state_get_env(*p_mdl_lid_temp);

        // Get server-specific callback functions
        p_cb_srv = p_env_ctlt->env.cb.u.p_cb_srv;

        // Set initial range
        p_env_ctlt->temp_min = MM_LIGHTS_CTL_TEMP_MIN;
        p_env_ctlt->temp_max = MM_LIGHTS_CTL_TEMP_MAX;

        // And initial value
        p_env_ctlt->temp = MM_LIGHTS_CTL_TEMP_MIN;

        // Prepare timer for publications
        p_env_ctlt->tmr_publi.cb = mm_lights_ctl_cb_tmr_publi;
        p_env_ctlt->tmr_publi.p_env = (void *)p_env_ctlt;

        // Set internal callback functions
        p_env_ctlt->env.cb.cb_rx = mm_lights_ctl_cb_rx;
        p_env_ctlt->env.cb.cb_opcode_check = mm_lights_ctl_cb_opcode_check;
        p_env_ctlt->env.cb.cb_publish_param = mm_lights_ctl_cb_publish_param;
        p_cb_srv->cb_set = mm_lights_ctl_cb_set;

        // Inform application about registered model
        mm_api_send_register_ind(MM_ID_LIGHTS_CTLT, elmt_idx + 1, *p_mdl_lid_temp);

        // Bound models together
        p_env_ctl->p_env_ctlt = p_env_ctlt;
        p_env_ctlt->p_env_ctl = p_env_ctl;
    }
    while (0);

    return (status);
}

/*
 * CALLBACK FUNCTIONS FOR BINDING MANAGER
 ****************************************************************************************
 */

void mm_lights_ctl_cb_grp_event(m_lid_t mdl_lid, uint8_t event, uint8_t info)
{
    // Get environment for Light CTL Server model
    mm_lights_ctl_env_t *p_env_ctl = (mm_lights_ctl_env_t *)mm_tb_state_get_env(mdl_lid);
    // Indicate if Light CTL Lightness and Light CTL Delta UV values have been modified
    bool ln_upd = 1;//(p_env_ctl->ln != p_env_ctl->ln_tgt);
    bool delta_uv_upd = (p_env_ctl->delta_uv != p_env_ctl->delta_uv_tgt);
    MESH_MODEL_PRINT_DEBUG("%s mdl_lid:%d,event:%d\r\n", __func__, mdl_lid, event);
    switch (event)
    {
        case (MESH_MDL_GRP_EVENT_TRANS_DELAY_EXPIRED):
        {
            if (p_env_ctl->ln != p_env_ctl->ln_tgt)
            {
                // Set the targeted Light Lightness state value
                mm_tb_bind_set_state(p_env_ctl->env.grp_lid, MM_STATE_TYPE_TARGET, MM_ID_LIGHTS_LN,
                                     p_env_ctl->ln);
            }



            // Start the transition
            mm_tb_bind_trans_start(p_env_ctl->env.grp_lid);
        } break;

        case (MESH_MDL_GRP_EVENT_TRANS_IMMEDIATE):
        {
            p_env_ctl->ln = p_env_ctl->ln_tgt;
            p_env_ctl->delta_uv = p_env_ctl->delta_uv_tgt;
        } // no break;

        case (MESH_MDL_GRP_EVENT_TRANS_STARTED):
        {
            uint8_t trans_time = info;

            // Transition has been started
            SETB(p_env_ctl->status_info, MM_LIGHTS_CTL_SINFO_WAIT_LN, 0);

            // Inform application about state update
            if (ln_upd)
            {
                mm_api_send_srv_state_upd_ind(MM_STATE_LIGHT_CTL_LN, p_env_ctl->env.elmt_idx,
                                              p_env_ctl->ln_tgt,
                                              (trans_time) ? mm_tb_get_trans_time_ms(trans_time) : 0);
            }
            if (delta_uv_upd)
            {
                mm_api_send_srv_state_upd_ind(MM_STATE_LIGHT_CTL_DELTA_UV, p_env_ctl->env.elmt_idx,
                                              p_env_ctl->delta_uv_tgt,
                                              (trans_time) ? mm_tb_get_trans_time_ms(trans_time) : 0);
            }

            // Check if a status message must be sent
            mm_lights_ctl_check_status_rsp(p_env_ctl);

            // Send a publication
            mm_lights_ctl_publish(p_env_ctl);
        } break;

        case (MESH_MDL_GRP_EVENT_TRANS_END):
        {
            p_env_ctl->ln = p_env_ctl->ln_tgt;
            p_env_ctl->delta_uv = p_env_ctl->delta_uv_tgt;

            // Inform application about state update
            if (ln_upd)
            {
                mm_api_send_srv_state_upd_ind(MM_STATE_LIGHT_CTL_LN,
                                              p_env_ctl->env.elmt_idx, p_env_ctl->ln, 0);
            }
            if (delta_uv_upd)
            {
                mm_api_send_srv_state_upd_ind(MM_STATE_LIGHT_CTL_DELTA_UV,
                                              p_env_ctl->env.elmt_idx, p_env_ctl->delta_uv, 0);
            }

            // Check if a status message must be sent
            mm_lights_ctl_check_status_rsp(p_env_ctl);

            // Send a publication
            mm_lights_ctl_publish(p_env_ctl);
        } break;

        case (MESH_MDL_GRP_EVENT_GROUP_FULL):
        {
            // Set the targeted Light Lightness state value
            mm_tb_bind_set_state(p_env_ctl->env.grp_lid, MM_STATE_TYPE_CURRENT, MM_ID_LIGHTS_LN,
                                 p_env_ctl->ln);
        } break;

        //case (MESH_MDL_GRP_EVENT_TRANS_ABORTED):
        default:
        {
        } break;
    }
}

void mm_lights_ctl_cb_grp_event_temp(m_lid_t mdl_lid, uint8_t event, uint8_t info)
{
    // Get environment for Light CTL Temperature Server model
    mm_lights_ctlt_env_t *p_env_ctlt = (mm_lights_ctlt_env_t *)mm_tb_state_get_env(mdl_lid);
    // Get environment for Light CTL Server model
    mm_lights_ctl_env_t *p_env_ctl = p_env_ctlt->p_env_ctl;

    MESH_MODEL_PRINT_DEBUG("%s mdl_lid:%d,event:%d\r\n", __func__, mdl_lid, event);
    switch (event)
    {
        case (MESH_MDL_GRP_EVENT_TRANS_DELAY_EXPIRED):
        {
            // Generic Level = (Light CTL Temperature - T _MIN) * 65535 / (T_MAX - T_MIN) - 32768
            int32_t lvl32;
            int16_t lvl16;

            lvl32 = ((p_env_ctlt->temp_tgt - p_env_ctlt->temp_min) * 65535);
            lvl32 /= (p_env_ctlt->temp_max - p_env_ctlt->temp_min);
            lvl32 -= 32768;
            lvl16 = (int16_t)lvl32;

            // Set the targeted Generic Level state value
            mm_tb_bind_set_state(p_env_ctlt->env.grp_lid, MM_STATE_TYPE_TARGET, MM_ID_GENS_LVL, lvl16);

            // Start the transition
            mm_tb_bind_trans_start(p_env_ctlt->env.grp_lid);
        } break;

        case (MESH_MDL_GRP_EVENT_TRANS_IMMEDIATE):
        {
            p_env_ctlt->temp = p_env_ctlt->temp_tgt;
        } // no break;

        case (MESH_MDL_GRP_EVENT_TRANS_STARTED):
        {
            uint8_t trans_time = info;

            // Mark transition as finished
            SETB(p_env_ctl->status_info, MM_LIGHTS_CTL_SINFO_WAIT_TEMP, 0);

            // Inform application about state update
            mm_api_send_srv_state_upd_ind(MM_STATE_LIGHT_CTL_TEMP, p_env_ctlt->env.elmt_idx,
                                          p_env_ctlt->temp_tgt,
                                          (trans_time) ? mm_tb_get_trans_time_ms(trans_time) : 0);

            // Check if a status message must be sent
            mm_lights_ctl_check_status_rsp(p_env_ctl);

            // Send a publication
            mm_lights_ctl_publish_temp(p_env_ctlt);
        } break;

        case (MESH_MDL_GRP_EVENT_TRANS_END):
        {
            p_env_ctlt->temp = p_env_ctlt->temp_tgt;

            // Mark transition as finished
            SETB(p_env_ctl->status_info, MM_LIGHTS_CTL_SINFO_WAIT_TEMP, 0);

            // Inform application about state update
            mm_api_send_srv_state_upd_ind(MM_STATE_LIGHT_CTL_TEMP, p_env_ctlt->env.elmt_idx,
                                          p_env_ctlt->temp, 0);

            // Check if a status message must be sent
            mm_lights_ctl_check_status_rsp(p_env_ctl);

            // Send a publication
            mm_lights_ctl_publish_temp(p_env_ctlt);
        } break;

        case (MESH_MDL_GRP_EVENT_GROUP_FULL):
        {
            // Deduce Generic Level state value from Light CTL Temperature state value
            int16_t lvl = mm_lights_ctl_temp_to_lvl(p_env_ctlt->temp, p_env_ctlt->temp_min,
                                                    p_env_ctlt->temp_max);

            // Set the current Generic Level state value
            mm_tb_bind_set_state(p_env_ctlt->env.grp_lid, MM_STATE_TYPE_CURRENT, MM_ID_GENS_LVL, lvl);
        };

        //case (MESH_MDL_GRP_EVENT_TRANS_ABORTED):
        default:
        {
        } break;
    }
}

void mm_lights_ctl_cb_trans_req(m_lid_t main_mdl_lid, uint32_t req_model_id, uint8_t trans_type,
                                uint32_t state_delta)
{
    // Get environment for Light CTL Server model
    mm_lights_ctl_env_t *p_env_ctl = (mm_lights_ctl_env_t *)mm_tb_state_get_env(main_mdl_lid);
    // Targeted Light CTL Actual state value
    uint16_t ln_tgt;

    MESH_MODEL_PRINT_DEBUG("%s main_mdl_lid:%d,req_model_id:0x%x\r\n", __func__, main_mdl_lid, req_model_id);
    if (req_model_id == MM_ID_LIGHTS_LN)
    {
        ln_tgt = (uint16_t)state_delta;
    }
    else
    {
        if (req_model_id == MM_ID_GENS_OO)
        {
            // Requested Generic OnOff state value
            uint8_t onoff = (uint8_t)state_delta;

            if (onoff == 0)
            {
                ln_tgt = 0;
            }
            else
            {
                ln_tgt = 0;
            }
        }
        else     // (req_model_id == MM_ID_GENS_LVL)
        {
            if (trans_type == MM_TRANS_TYPE_CLASSIC)
            {
                // Requested Generic Level state value
                int16_t level = (int16_t)state_delta;

                // Light CTL Actual = Generic Level + 32768
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
                    p_env_ctl->move_delta = (int16_t)state_delta;
                }
                else
                {
                    delta = (int32_t)state_delta;
                }

                // Add the Light CTL Actual state value to the received delta value
                delta += p_env_ctl->ln;

                // The Light CTL Actual state value cannot wrap
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
            uint16_t ln_min = mm_lights_ln_get(p_env_ctl->env.elmt_idx, MM_STATE_LIGHT_LN_RANGE_MIN);
            uint16_t ln_max = mm_lights_ln_get(p_env_ctl->env.elmt_idx, MM_STATE_LIGHT_LN_RANGE_MAX);

            // Ensure that Light CTL Lightness state value is between Light CTL Lightness Range
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


    MESH_MODEL_PRINT_DEBUG("ln_tgt:%d,p_env_ctl->ln:%d\r\n", ln_tgt, p_env_ctl->ln);

    // Check if Light CTL Actual state value is modified
    if (ln_tgt != p_env_ctl->ln)
    {
        p_env_ctl->ln_tgt = ln_tgt;

        // Start a new transition
        mm_tb_bind_trans_new(p_env_ctl->env.grp_lid, trans_type, 0, 0);
    }
    else
    {
        // Reject the transition
        mm_tb_bind_trans_reject(p_env_ctl->env.grp_lid);
    }
}

void mm_lights_ctl_cb_trans_req_temp(m_lid_t main_mdl_lid, uint32_t req_model_id, uint8_t trans_type,
                                     uint32_t state_delta)
{
    // Get environment for Light CTL Temperature Server model
    mm_lights_ctlt_env_t *p_env_ctlt = (mm_lights_ctlt_env_t *)mm_tb_state_get_env(main_mdl_lid);
    // Targeted Light CTL Temperature state value
    uint16_t temp_tgt;

    MESH_MODEL_PRINT_DEBUG("%s main_mdl_lid:%d,req_model_id:0x%x\r\n", __func__, main_mdl_lid, req_model_id);

    if (trans_type == MM_TRANS_TYPE_CLASSIC)
    {
        // Requested Generic Level state value
        int16_t lvl = (int16_t)state_delta;

        temp_tgt = mm_lights_ctl_lvl_to_temp(lvl, p_env_ctlt->temp_min, p_env_ctlt->temp_max);
    }
    else     // ((trans_type == MM_TRANS_TYPE_DELTA) || trans_type == MM_TRANS_TYPE_MOVE))
    {
        // Delta value
        int32_t delta;

        if (trans_type == MM_TRANS_TYPE_MOVE)
        {
            delta = (int16_t)state_delta;

            // Keep the provided delta value
            p_env_ctlt->move_delta = delta;
        }
        else
        {
            delta = (int32_t)state_delta;
        }

        temp_tgt = mm_lights_ctl_temp_add_delta(p_env_ctlt->temp, delta,
                                                p_env_ctlt->temp_min, p_env_ctlt->temp_max);
    }

    // Ensure that Light CTL Temperature state value is between Light CTL Temperature Range
    // Min and Max values
    if (temp_tgt > p_env_ctlt->temp_max)
    {
        temp_tgt = p_env_ctlt->temp_max;
    }
    else if (temp_tgt < p_env_ctlt->temp_min)
    {
        temp_tgt = p_env_ctlt->temp_min;
    }

    MESH_MODEL_PRINT_DEBUG("temp_tgt:%d,p_env_ctlt->temp:%d \r\n", temp_tgt, p_env_ctlt->temp);
    // Check if Light CTL Temperature state value is modified
    if (temp_tgt != p_env_ctlt->temp)
    {
        p_env_ctlt->temp_tgt = temp_tgt;

        // Start a new transition
        mm_tb_bind_trans_new(p_env_ctlt->env.grp_lid, trans_type, 0, 0);
    }
    else
    {
        // Reject the transition
        mm_tb_bind_trans_reject(p_env_ctlt->env.grp_lid);
    }
}
#endif //(BLE_MESH_MDL_LIGHTS_CTL)
/// @} end of group
