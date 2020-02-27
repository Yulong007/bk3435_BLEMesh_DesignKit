/**
 ****************************************************************************************
 * @file mm_lightc.c
 *
 * @brief Mesh Model Lighting Client Module
 *
 * Copyright (C) RivieraWaves 2017-2019
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup MM_LIGHTC
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "mm_lightc_int.h"      // Mesh Model Lighting Client Module Internal Definitions


#if (BLE_MESH_MDL_LIGHTC)
/*
 * FUNCTIONS
 ****************************************************************************************
 */

uint16_t mm_lightc_init(bool reset, void *p_env, const mm_cfg_t *p_cfg)
{
    // Return size of environment
    return (0);
}

uint16_t mm_lightc_get_env_size(const mm_cfg_t *p_cfg)
{
    // Return size of environment
    return (0);
}

uint16_t mm_lightc_register(uint8_t cmdl_idx)
{
    // Status
    uint16_t status = MESH_ERR_NO_ERROR;

    switch (cmdl_idx)
    {
        case (MM_CMDL_IDX_LIGHTC_LN):
        {
            // Add Light Lightness Client model if not already present
            if (mm_tb_state_get_lid(0, MM_ID_LIGHTC_LN) == MESH_INVALID_LID)
            {
                // Register the Light Lightness Client model
                status = mm_lightc_ln_register();
            }
        } break;

        case (MM_CMDL_IDX_LIGHTC_CTL):
        {
            // Add Light CTL Client model if not already present
            if (mm_tb_state_get_lid(0, MM_ID_LIGHTC_CTL) == MESH_INVALID_LID)
            {
                // Register the Light CTL Client model
                status = mm_lightc_ctl_register();
            }
        } break;

        case (MM_CMDL_IDX_LIGHTC_HSL):
        {
            // Add Light HSL Client model if not already present
            if (mm_tb_state_get_lid(0, MM_ID_LIGHTC_HSL) == MESH_INVALID_LID)
            {
                // Register the Light HSL Client model
                status = mm_lightc_hsl_register();
            }
        } break;

        case (MM_CMDL_IDX_LIGHTC_XYL):
        {
            // Add Light xyL Client model if not already present
            if (mm_tb_state_get_lid(0, MM_ID_LIGHTC_XYL) == MESH_INVALID_LID)
            {
                // Register the Light xyL Client model
                status = mm_lightc_xyl_register();
            }
        } break;

        default:
        {
            MESH_MODEL_PRINT_WARN("%s, Invalid parameters.\n", __func__);
            // Model is unknown
            status = MESH_ERR_INVALID_PARAM;
        }
    }

    return (status);
}
#endif //(BLE_MESH_MDL_LIGHTC)
/// @} end of group
