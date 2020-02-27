/**
 ****************************************************************************************
 *
 * @file mm_genc.c
 *
 * @brief Mesh Model Generic Client Module
 *
 * Copyright (C) RivieraWaves 2017-2019
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup MM_GENC
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "mm_genc_int.h"      // Mesh Model Generic Client Module Internal Definitions

#if (BLE_MESH_MDL_GENC)
/*
 * FUNCTIONS
 ****************************************************************************************
 */

uint16_t mm_genc_init(bool reset, void *p_env, const mm_cfg_t *p_cfg)
{
    // Return size of environment
    return (0);
}

uint16_t mm_genc_get_env_size(const mm_cfg_t *p_cfg)
{
    // Return size of environment
    return (0);
}

uint16_t mm_genc_register(uint8_t cmdl_idx)
{
    // Status
    uint16_t status = MESH_ERR_NO_ERROR;

    switch (cmdl_idx)
    {
        case (MM_CMDL_IDX_GENC_ONOFF):
        {
            // Add Generic OnOff Client model if not already present
            if (mm_tb_state_get_lid(0, MM_ID_GENC_OO) == MESH_INVALID_LID)
            {
                // Register the Generic OnOff Client model
                status = mm_genc_oo_register();
            }
        } break;

        case (MM_CMDL_IDX_GENC_LEVEL):
        {
            // Add Generic Level Client model if not already present
            if (mm_tb_state_get_lid(0, MM_ID_GENC_LVL) == MESH_INVALID_LID)
            {
                // Register the Generic Level Client model
                status = mm_genc_lvl_register();
            }
        } break;

        case (MM_CMDL_IDX_GENC_DFT_TRANS_TIME):
        {
            // Add Generic Default Transition Time Client model if not already present
            if (mm_tb_state_get_lid(0, MM_ID_GENC_DTT) == MESH_INVALID_LID)
            {
                // Register the Generic Default Transition Time Client model
                status = mm_genc_dtt_register();
            }
        } break;

        case (MM_CMDL_IDX_GENC_POWER_ONOFF):
        {
            // Add Generic Power OnOff Client model if not already present
            if (mm_tb_state_get_lid(0, MM_ID_GENC_POO) == MESH_INVALID_LID)
            {
                // Register the Generic Power OnOff Client model
                status = mm_genc_poo_register();
            }
        } break;

        case (MM_CMDL_IDX_GENC_POWER_LEVEL):
        {
            // Add Generic Power Level Client model if not already present
            if (mm_tb_state_get_lid(0, MM_ID_GENC_PLVL) == MESH_INVALID_LID)
            {
                // Register the Generic Power Level Client model
                status = mm_genc_plvl_register();
            }
        } break;

        case (MM_CMDL_IDX_GENC_BATTERY):
        {
            // Add Generic Battery Client model if not already present
            if (mm_tb_state_get_lid(0, MM_ID_GENC_BAT) == MESH_INVALID_LID)
            {
                // Register the Generic Battery Client model
                status = mm_genc_bat_register();
            }
        } break;

        case (MM_CMDL_IDX_GENC_LOCATION):
        {
            // Add Generic Location Client model if not already present
            if (mm_tb_state_get_lid(0, MM_ID_GENC_LOC) == MESH_INVALID_LID)
            {
                // Register the Generic Location Client model
                status = mm_genc_loc_register();
            }
        } break;

        case (MM_CMDL_IDX_GENC_PROPERTY):
        {
            // Add Generic Property Client model if not already present
            if (mm_tb_state_get_lid(0, MM_ID_GENC_PROP) == MESH_INVALID_LID)
            {
                // Register the Generic Property Client model
                status = mm_genc_prop_register();
            }
        } break;

        default:
        {
            // Model is unknown
            status = MESH_ERR_INVALID_PARAM;
        }
    }

    return (status);
}
#endif //(BLE_MESH_MDL_GENC)
/// @} end of group
