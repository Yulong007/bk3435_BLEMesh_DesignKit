/**
 ****************************************************************************************
 * @file mm_lights.c
 *
 * @brief Mesh Model Lighting Server Module
 *
 * Copyright (C) RivieraWaves 2017-2019
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup MM_LIGHTS
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "mm_lights_int.h"      // Mesh Model Lighting Server Module Internal Definitions


#if (BLE_MESH_MDL_LIGHTS)
/*
 * LOCAL FUNCTIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Register Light Lightness Server and Light Lightness Setup Server models
 * needed for management of Light Lightness state. Also register the Generic Power
 * OnOff Server and the Generic Level Server models and the Generic OnOff Server model.
 *
 * @param[in] elmt_idx      Index of element on which required models must be registered
 * @param[in] main          True if Generic Power Level model is a main model, else false
 *
 * @return An error status
 ****************************************************************************************
 */
__STATIC uint16_t mm_lights_register_ln(uint8_t elmt_idx, m_lid_t *p_ln_mdl_lid, bool main)
{
    // Status
    uint16_t status = MESH_ERR_MDL_ALREADY_REGISTERED;

    if ((mm_tb_state_get_lid(elmt_idx, MM_ID_LIGHTS_LN) == MESH_INVALID_LID)
            && (mm_tb_state_get_lid(elmt_idx, MM_ID_LIGHTS_LNS) == MESH_INVALID_LID)
            && (mm_tb_state_get_lid(elmt_idx, MM_ID_GENS_LVL) == MESH_INVALID_LID)
            && (mm_tb_state_get_lid(elmt_idx, MM_ID_GENS_OO) == MESH_INVALID_LID)
            && (mm_tb_state_get_lid(elmt_idx, MM_ID_GENS_POO) == MESH_INVALID_LID))
    {
        do
        {
            // Register the Generic OnOff Server, the Generic Power OnOff Server and the Generic Power
            // OnOff Setup Server models
            status = mm_api_register_server(elmt_idx, MM_CFG_IDX_GENS_POWER_ONOFF, 0);

            if (status != MESH_ERR_NO_ERROR)
            {
                MESH_MODEL_PRINT_WARN("%s, Register the Generic OnOff Server fail.\n", __func__);
                break;
            }

            // Register the Generic Level Server model
            status = mm_api_register_server(elmt_idx, MM_CFG_IDX_GENS_LEVEL, 0);

            if (status != MESH_ERR_NO_ERROR)
            {
                MESH_MODEL_PRINT_WARN("%s, Register the Generic Level Server model fail.\n", __func__);
                break;
            }

            // Register the Light Lightness Server and the Light Lightness Setup Server
            // models
            status = mm_lights_ln_register(elmt_idx, p_ln_mdl_lid);

            if (status != MESH_ERR_NO_ERROR)
            {
                MESH_MODEL_PRINT_WARN("%s, Register the Light Lightness Server and the Light Lightness Setup Server fail.\n", __func__);
            }
            if (main &&
                    (status == MESH_ERR_NO_ERROR))
            {
                // Group local index
                m_lid_t grp_lid;

                // Create group and set Light Lightness Server model as main model
                mm_tb_bind_add_group(2, elmt_idx, &grp_lid, *p_ln_mdl_lid,
                                     mm_lights_ln_cb_grp_event, mm_lights_ln_cb_trans_req);


                // Add Generic Level Server model to the group
                mm_api_grp_add_local(elmt_idx, MM_ID_GENS_LVL, grp_lid);
                // Add Generic OnOff Server model to the group
                mm_api_grp_add_local(elmt_idx, MM_ID_GENS_OO, grp_lid);
            }
        }
        while (0);
    }

    return (status);
}

/**
 ****************************************************************************************
 * @brief Register Light CTL Server, Light CTL Setup Server, Light Lightness Temperature
 * Server models and associated models
 *
 * @param[in] elmt_idx      Index of element on which required models must be registered
 * @param[in] main          True if Generic Power Level model is a main model, else false
 *
 * @return An error status
 ****************************************************************************************
 */
__STATIC uint16_t mm_lights_register_ctl(uint8_t elmt_idx, bool main)
{
    // Status
    uint16_t status = MESH_ERR_MDL_ALREADY_REGISTERED;

    if ((mm_tb_state_get_lid(elmt_idx, MM_ID_LIGHTS_CTL) == MESH_INVALID_LID)
            && (mm_tb_state_get_lid(elmt_idx, MM_ID_LIGHTS_CTLS) == MESH_INVALID_LID)
            && (mm_tb_state_get_lid(elmt_idx + 1, MM_ID_LIGHTS_CTLT) == MESH_INVALID_LID))
    {
        do
        {
            // Model local index for Light CTL Server model and for Light CTL Tempereture Server model
            // and for Light Lightness Server model
            m_lid_t ln_mdl_lid, ctl_mdl_lid, ctlt_mdl_lid;

            // Register Light Lightness Server model and associated models
            status = mm_lights_register_ln(elmt_idx, &ln_mdl_lid, false);

            if (status != MESH_ERR_NO_ERROR)
            {
                MESH_MODEL_PRINT_WARN("%s, Register Light Lightness Server model and associated models fail.\n", __func__);
                break;
            }

            // Register Generic Level Server model
            status = mm_api_register_server(elmt_idx + 1, MM_CFG_IDX_GENS_LEVEL, 0);

            if (status != MESH_ERR_NO_ERROR)
            {
                MESH_MODEL_PRINT_WARN("%s, Register Generic Level Server model fail.\n", __func__);
                break;
            }

            // Register the Light CTL Server model and associated models
            status = mm_lights_ctl_register(elmt_idx, &ctl_mdl_lid, &ctlt_mdl_lid);

            if (status != MESH_ERR_NO_ERROR)
            {
                MESH_MODEL_PRINT_WARN("%s, Register the Light CTL Server model and associated models fail.\n", __func__);
            }
            if (main && (status == MESH_ERR_NO_ERROR))
            {
                // Group local index
                m_lid_t grp_lid;

                // Create group and set Light CTL Server model as main model
                mm_tb_bind_add_group(3, elmt_idx, &grp_lid, ctl_mdl_lid,
                                     mm_lights_ctl_cb_grp_event, mm_lights_ctl_cb_trans_req);

                // Add Generic Level Server model to the group
                mm_api_grp_add_local(elmt_idx, MM_ID_GENS_LVL, grp_lid);
                // Add Generic OnOff Server model to the group
                mm_api_grp_add_local(elmt_idx, MM_ID_GENS_OO, grp_lid);
                // Add Light Lightness Server model to the group
                mm_tb_bind_group_add_mdl(grp_lid, ln_mdl_lid, MM_ID_LIGHTS_LN,
                                         mm_lights_ln_cb_grp_event, mm_lights_ln_cb_set_state);

                // Create group and set Light CTL Temperature Server model as main model
                mm_tb_bind_add_group(1, elmt_idx + 1, &grp_lid, ctlt_mdl_lid,
                                     mm_lights_ctl_cb_grp_event_temp, mm_lights_ctl_cb_trans_req_temp);

                // Add Generic Level Server model to the group
                mm_api_grp_add_local(elmt_idx + 1, MM_ID_GENS_LVL, grp_lid);
            }
        }
        while (0);
    }

    return (status);
}

/**
 ****************************************************************************************
 * @brief Register Light HSL Server, Light HSL Setup Server, Light Lightness Hue Server,
 * Light Lightness Saturation Server models and associated models
 *
 * @param[in] elmt_idx      Index of element on which required models must be registered
 * @param[in] main          True if Generic Power Level model is a main model, else false
 *
 * @return An error status
 ****************************************************************************************
 */
__STATIC uint16_t mm_lights_register_hsl(uint8_t elmt_idx, bool main)
{
    // Status
    uint16_t status = MESH_ERR_MDL_ALREADY_REGISTERED;


    if ((mm_tb_state_get_lid(elmt_idx, MM_ID_LIGHTS_HSL) == MESH_INVALID_LID)
            && (mm_tb_state_get_lid(elmt_idx, MM_ID_LIGHTS_HSLS) == MESH_INVALID_LID)
            && (mm_tb_state_get_lid(elmt_idx + 1, MM_ID_LIGHTS_HSLH) == MESH_INVALID_LID)
            && (mm_tb_state_get_lid(elmt_idx + 2, MM_ID_LIGHTS_HSLSAT) == MESH_INVALID_LID))
    {
        do
        {
            // Model local index for Light CTL Server model, for Light CTL Hue Server model
            // for Light CTL Saturation Server model and for Light Lightness Server model
            m_lid_t ln_mdl_lid, hsl_mdl_lid, hslh_mdl_lid, hslsat_mdl_lid;


            // Register Light Lightness Server model and associated models
            status = mm_lights_register_ln(elmt_idx, &ln_mdl_lid, false);


            if (status != MESH_ERR_NO_ERROR)
            {
                MESH_MODEL_PRINT_WARN("%s, Register Light Lightness Server model and associated models fail.\n", __func__);
                break;
            }

            // Register Generic Level Server model
            status = mm_api_register_server(elmt_idx + 1, MM_CFG_IDX_GENS_LEVEL, 0);

            if (status != MESH_ERR_NO_ERROR)
            {
                MESH_MODEL_PRINT_WARN("%s, Register Generic Level Server model fail.\n", __func__);
                break;
            }

            // Register Generic Level Server model
            status = mm_api_register_server(elmt_idx + 2, MM_CFG_IDX_GENS_LEVEL, 0);

            if (status != MESH_ERR_NO_ERROR)
            {
                MESH_MODEL_PRINT_WARN("%s, Register Generic Level Server model fail.\n", __func__);
                break;
            }

            // Register the Light HSL Server model and associated models
            status = mm_lights_hsl_register(elmt_idx, &hsl_mdl_lid, &hslh_mdl_lid, &hslsat_mdl_lid);

            //  main = 0;
            if (main && (status == MESH_ERR_NO_ERROR))
            {
                // Group local index
                m_lid_t grp_lid;

                // Create group and set Light HSL Server model as main model
                mm_tb_bind_add_group(3, elmt_idx, &grp_lid, hsl_mdl_lid,
                                     mm_lights_hsl_cb_grp_event, mm_lights_hsl_cb_trans_req);

                // Add Generic Level Server model to the group
                mm_api_grp_add_local(elmt_idx, MM_ID_GENS_LVL, grp_lid);
                // Add Generic OnOff Server model to the group
                mm_api_grp_add_local(elmt_idx, MM_ID_GENS_OO, grp_lid);
                // Add Light Lightness Server model to the group
                mm_tb_bind_group_add_mdl(grp_lid, ln_mdl_lid, MM_ID_LIGHTS_LN,
                                         mm_lights_ln_cb_grp_event, mm_lights_ln_cb_set_state);

                // Create group and set Light HSL Hue Server model as main model
                mm_tb_bind_add_group(1, elmt_idx + 1, &grp_lid, hslh_mdl_lid,
                                     mm_lights_hsl_cb_grp_event_hue_sat,
                                     mm_lights_hsl_cb_trans_req_hue_sat);

                // Add Generic Level Server model to the group
                mm_api_grp_add_local(elmt_idx + 1, MM_ID_GENS_LVL, grp_lid);

                // Create group and set Light HSL Saturation Server model as main model
                mm_tb_bind_add_group(1, elmt_idx + 2, &grp_lid, hslsat_mdl_lid,
                                     mm_lights_hsl_cb_grp_event_hue_sat,
                                     mm_lights_hsl_cb_trans_req_hue_sat);

                // Add Generic Level Server model to the group
                mm_api_grp_add_local(elmt_idx + 2, MM_ID_GENS_LVL, grp_lid);
            }
        }
        while (0);
    }
    return (status);
}

/**
 ****************************************************************************************
 * @brief Register Light HSL Server, Light HSL Setup Server, Light Lightness Hue Server,
 * Light Lightness Saturation Server models and associated models
 *
 * @param[in] elmt_idx      Index of element on which required models must be registered
 * @param[in] main          True if Generic Power Level model is a main model, else false
 *
 * @return An error status
 ****************************************************************************************
 */
__STATIC uint16_t mm_lights_register_ctl_hsl(uint8_t elmt_idx, bool main)
{
    // Status
    uint16_t status = MESH_ERR_MDL_ALREADY_REGISTERED;


    if ((mm_tb_state_get_lid(elmt_idx, MM_ID_LIGHTS_HSL) == MESH_INVALID_LID)
            && (mm_tb_state_get_lid(elmt_idx, MM_ID_LIGHTS_HSLS) == MESH_INVALID_LID)
            && (mm_tb_state_get_lid(elmt_idx + 1, MM_ID_LIGHTS_HSLH) == MESH_INVALID_LID)
            && (mm_tb_state_get_lid(elmt_idx + 2, MM_ID_LIGHTS_HSLSAT) == MESH_INVALID_LID))
    {
        do
        {
            // Model local index for Light HSL Server model, for Light HSL Hue Server model
            // for Light HSL Saturation Server model and for Light Lightness Server model
            m_lid_t ln_mdl_lid, hsl_mdl_lid, hslh_mdl_lid, hslsat_mdl_lid;

            // Model local index for Light CTL Server model, for Light CTL temp Server model
            m_lid_t ctl_mdl_lid, ctlt_mdl_lid;
            // Register Light Lightness Server model and associated models

            status = mm_lights_register_ln(elmt_idx, &ln_mdl_lid, true);

            if (status != MESH_ERR_NO_ERROR)
            {
                MESH_MODEL_PRINT_WARN("%s, Register Light Lightness Server model and associated models fail.\n", __func__);
                break;
            }

            // Register Light Lightness Server model and associated models
            status = mm_lights_ctl_register(elmt_idx, &ctl_mdl_lid, &ctlt_mdl_lid);

            if (status != MESH_ERR_NO_ERROR)
            {
                MESH_MODEL_PRINT_WARN("%s, Register Light Lightness Server model and associated models fail.\n", __func__);
                break;
            }

            if (main && (status == MESH_ERR_NO_ERROR))
            {
                // Group local index
                m_lid_t grp_lid;

                // Create group and set Light CTL Server model as main model
                mm_tb_bind_add_group(3, elmt_idx, &grp_lid, ctl_mdl_lid,
                                     mm_lights_ctl_cb_grp_event, mm_lights_ctl_cb_trans_req);

                // Add Generic Level Server model to the group
                mm_api_grp_add_local(elmt_idx, MM_ID_GENS_LVL, grp_lid);
                // Add Generic OnOff Server model to the group
                mm_api_grp_add_local(elmt_idx, MM_ID_GENS_OO, grp_lid);


                // Create group and set Light CTL Temperature Server model as main model
                mm_tb_bind_add_group(1, elmt_idx + 1, &grp_lid, ctlt_mdl_lid,
                                     mm_lights_ctl_cb_grp_event_temp, mm_lights_ctl_cb_trans_req_temp);

                // Add Generic Level Server model to the group
                mm_api_grp_add_local(elmt_idx + 1, MM_ID_GENS_LVL, grp_lid);

            }


            // Register Generic Level Server model
            status = mm_api_register_server(elmt_idx + 1, MM_CFG_IDX_GENS_LEVEL, 0);

            if (status != MESH_ERR_NO_ERROR)
            {
                MESH_MODEL_PRINT_WARN("%s, Register Generic Level Server model fail.\n", __func__);
                break;
            }

            // Register Generic Level Server model
            status = mm_api_register_server(elmt_idx + 2, MM_CFG_IDX_GENS_LEVEL, 0);

            if (status != MESH_ERR_NO_ERROR)
            {
                MESH_MODEL_PRINT_WARN("%s, Register Generic Level Server model fail.\n", __func__);
                break;
            }

            // Register the Light HSL Server model and associated models
            status = mm_lights_hsl_register(elmt_idx, &hsl_mdl_lid, &hslh_mdl_lid, &hslsat_mdl_lid);

            //  main = 0;
            if (main && (status == MESH_ERR_NO_ERROR))
            {
                // Group local index
                m_lid_t grp_lid;

                // Create group and set Light HSL Server model as main model
                mm_tb_bind_add_group(2, elmt_idx, &grp_lid, hsl_mdl_lid,
                                     mm_lights_hsl_cb_grp_event, mm_lights_hsl_cb_trans_req);

                // Add Generic Level Server model to the group
                mm_api_grp_add_local(elmt_idx, MM_ID_GENS_LVL, grp_lid);
                // Add Generic OnOff Server model to the group
                mm_api_grp_add_local(elmt_idx, MM_ID_GENS_OO, grp_lid);
                // Add Light Lightness Server model to the group
                //    mm_tb_bind_group_add_mdl(grp_lid, ln_mdl_lid, MM_ID_LIGHTS_LN,
                //                  mm_lights_ln_cb_grp_event, mm_lights_ln_cb_set_state);

                // Create group and set Light HSL Hue Server model as main model
                mm_tb_bind_add_group(1, elmt_idx + 1, &grp_lid, hslh_mdl_lid,
                                     mm_lights_hsl_cb_grp_event_hue_sat,
                                     mm_lights_hsl_cb_trans_req_hue_sat);

                // Add Generic Level Server model to the group
                mm_api_grp_add_local(elmt_idx + 1, MM_ID_GENS_LVL, grp_lid);

                // Create group and set Light HSL Saturation Server model as main model
                mm_tb_bind_add_group(1, elmt_idx + 2, &grp_lid, hslsat_mdl_lid,
                                     mm_lights_hsl_cb_grp_event_hue_sat,
                                     mm_lights_hsl_cb_trans_req_hue_sat);

                // Add Generic Level Server model to the group
                mm_api_grp_add_local(elmt_idx + 2, MM_ID_GENS_LVL, grp_lid);
            }
        }
        while (0);
    }
    return (status);
}

/*
 * FUNCTIONS
 ****************************************************************************************
 */

uint16_t mm_lights_init(bool reset, void *p_env, const mm_cfg_t *p_cfg)
{
    // Return size of environment
    return (0);
}

uint16_t mm_lights_get_env_size(const mm_cfg_t *p_cfg)
{
    uint16_t total_env_size = 0;

    // Return size of environment
    return (total_env_size);
}

uint16_t mm_lights_register(uint8_t elmt_idx, uint8_t mdl_cfg_idx, bool main)
{
    // Status
    uint16_t status = MESH_ERR_NO_ERROR;

    switch (mdl_cfg_idx)
    {
        case (MM_CFG_IDX_LIGHTS_LN):
        {
            // Model local index for register Light Lightness Server model
            m_lid_t ln_mdl_lid;

            // Add models needed for use of Light Lightness Server model
            status = mm_lights_register_ln(elmt_idx, &ln_mdl_lid, main);
        } break;

        case (MM_CFG_IDX_LIGHTS_CTL):
        {
            // Add models needed for use of Light CTL Server model
            status = mm_lights_register_ctl(elmt_idx, main);
        } break;

        case (MM_CFG_IDX_LIGHTS_HSL):
        {
            // Add models needed for use of Light HSL Server model
            status = mm_lights_register_hsl(elmt_idx, main);
        } break;

        case (MM_CFG_IDX_LIGHTS_CTL_HSL):
        {
            // Add models needed for use of Light HSL Server model
            status = mm_lights_register_ctl_hsl(elmt_idx, main);
        } break;

        default:
        {
            // Configuration is unknown
            ASSERT_INFO(0, mdl_cfg_idx, 0);
            status = MESH_ERR_COMMAND_DISALLOWED;
        }
    }

    return (status);
}

uint32_t mm_lights_isqrt(uint32_t n)
{
    uint64_t isqrt;
    uint64_t min = 0;
    uint64_t max = ((uint64_t)1) << 32;

    while (1)
    {
        uint64_t sq;

        if (max <= (1 + min))
        {
            isqrt = min;
            break;
        }

        isqrt = min + ((max - min) >> 1);
        sq = isqrt * isqrt;

        if (sq == n)
        {
            break;
        }

        if (sq > n)
        {
            max = isqrt;
        }
        else
        {
            min = isqrt;
        }
    }

    return (isqrt);
}
#endif //(BLE_MESH_MDL_LIGHTS)
/// @} end of group
