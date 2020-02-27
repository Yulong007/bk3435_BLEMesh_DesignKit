/**
 ****************************************************************************************
 * @file mm_gens.c
 *
 * @brief Mesh Model Generic Server Module
 *
 * Copyright (C) RivieraWaves 2017-2019
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup MM_GENS
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "mm_gens_int.h"      // Mesh Model Generic Server Module Internal Definitions

#if (BLE_MESH_MDL_GENS)
/*
 * LOCAL FUNCTIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Register Generic OnOff Server model needed for management of Generic OnOff state.
 *
 * @param[in] elmt_idx      Index of element on which model must be registered
 * @param[in] main          True if model is a main model, else false
 *
 * @return An error status
 ****************************************************************************************
 */
__STATIC uint16_t mm_gens_register_oo(uint8_t elmt_idx, bool main)
{
    // Status
    uint16_t status = MESH_ERR_MDL_ALREADY_REGISTERED;

    if (mm_tb_state_get_lid(elmt_idx, MM_ID_GENS_OO) == MESH_INVALID_LID)
    {
        // Model local index
        m_lid_t mdl_lid;

        // Register the Generic OnOff Server model
        status = mm_gens_oo_register(elmt_idx, &mdl_lid);
        if (status != MESH_ERR_NO_ERROR)
        {
            MESH_MODEL_PRINT_WARN("%s, Register the Generic OnOff Server model fail, status = 0x%x\n", __func__, status);
        }

        if (main && (status == MESH_ERR_NO_ERROR))
        {
            // Group local index
            m_lid_t grp_lid;

            // Create group
            mm_tb_bind_add_group(0, elmt_idx, &grp_lid, mdl_lid,
                                 mm_gens_oo_cb_grp_event, NULL);
        }
    }

    return (status);
}

/**
 ****************************************************************************************
 * @brief Register Generic Power OnOff Server and Generic Power OnOff Setup Server models
 * needed for management of Generic OnPowerUp state. Also register the Generic OnOff models
 * which is extended by the Generic Power OnOff Server model
 *
 * @param[in] elmt_idx      Index of element on which models must be registered
 * @param[in] main          True if Generic OnOff model is a main model, else false
 *
 * @return An error status
 ****************************************************************************************
 */
#if (BLE_MESH_MDL_GENS_POO)
__STATIC uint16_t mm_gens_register_poo(uint8_t elmt_idx, bool main)
{
    // Status
    uint16_t status;

    do
    {
        // Add Generic OnOff Server model if not already present
        status = mm_gens_register_oo(elmt_idx, main);

        if (status != MESH_ERR_NO_ERROR)
        {
            MESH_MODEL_PRINT_WARN("%s, Register the Generic OnOff Server model fail, status = 0x%x\n", __func__, status);
            break;
        }

        // Add Generic Power OnOff Server and Generic Power OnOff Setup Server models if not
        // already present
        if ((mm_tb_state_get_lid(elmt_idx, MM_ID_GENS_POO) == MESH_INVALID_LID)
                && (mm_tb_state_get_lid(elmt_idx, MM_ID_GENS_POOS) == MESH_INVALID_LID))
        {
            status = mm_gens_poo_register(elmt_idx);
            if (status != MESH_ERR_NO_ERROR)
            {
                MESH_MODEL_PRINT_WARN("%s, Register the Generic OnOff Server model fail, status = 0x%x\n", __func__, status);
            }
        }
    }
    while (0);

    return (status);
}
#endif

/**
 ****************************************************************************************
 * @brief Register Generic Power Level Server and Generic Power Level Setup Server models
 * needed for management of Generic Power Level state. Also register the Generic Power
 * OnOff Server and the Generic Level Server models extended by the Generic Power Level
 * Server model, and the Generic OnOff Server model extended by the Generic Power OnOff
 * Server model.
 *
 * @param[in] elmt_idx      Index of element on which required models must be registered
 * @param[in] main          True if Generic Power Level model is a main model, else false
 *
 * @return An error status
 ****************************************************************************************
 */
#if (BLE_MESH_MDL_GENS_PLVL)
__STATIC uint16_t mm_gens_register_plvl(uint8_t elmt_idx, bool main)
{
    // Status
    uint16_t status = MESH_ERR_MDL_ALREADY_REGISTERED;

    if ((mm_tb_state_get_lid(elmt_idx, MM_ID_GENS_PLVL) == MESH_INVALID_LID)
            && (mm_tb_state_get_lid(elmt_idx, MM_ID_GENS_PLVLS) == MESH_INVALID_LID)
            && (mm_tb_state_get_lid(elmt_idx, MM_ID_GENS_LVL) == MESH_INVALID_LID))
    {
        do
        {
            // Model local index Generic Level Server and Generic Power Level Server models
            m_lid_t lvl_mdl_lid, plvl_mdl_lid;

            // Register the Generic OnOff Server, the Generic Power OnOff Server and the Generic Power
            // OnOff Setup Server models
            status = mm_gens_register_poo(elmt_idx, false);

            if (status != MESH_ERR_NO_ERROR)
            {
                MESH_MODEL_PRINT_WARN("%s, Register the Generic OnOff Server model fail, status = 0x%x\n", __func__, status);
                break;
            }

            // Register the Generic Level Server model
            status = mm_gens_lvl_register(elmt_idx, &lvl_mdl_lid);

            if (status != MESH_ERR_NO_ERROR)
            {
                MESH_MODEL_PRINT_WARN("%s, Register the Generic Level Server model fail, status = 0x%x\n", __func__, status);
                break;
            }

            // Register the Generic Power Level Server and the Generic Power Level Setup Server
            // models
            status = mm_gens_plvl_register(elmt_idx, &plvl_mdl_lid);

            if (status == MESH_ERR_NO_ERROR)
            {
                if (main)
                {
                    // Retrieve model local index allocated for Generic OnOff Server model
                    m_lid_t oo_mdl_lid = mm_tb_state_get_lid(elmt_idx, MM_ID_GENS_OO);
                    // Group local index
                    m_lid_t grp_lid;

                    // Create group and set Generic Power Level Server model as main model
                    mm_tb_bind_add_group(2, elmt_idx, &grp_lid, plvl_mdl_lid,
                                         mm_gens_plvl_cb_grp_event, mm_gens_plvl_cb_trans_req);

                    // Add Generic Level Server model to the group
                    mm_tb_bind_group_add_mdl(grp_lid, lvl_mdl_lid, MM_ID_GENS_LVL,
                                             mm_gens_lvl_cb_grp_event,
                                             mm_gens_lvl_cb_set_state);

                    // Add Generic OnOff Server model to the group
                    mm_tb_bind_group_add_mdl(grp_lid, oo_mdl_lid, MM_ID_GENS_OO,
                                             mm_gens_oo_cb_grp_event,
                                             mm_gens_oo_cb_set_state);
                }
            }
            else
            {
                MESH_MODEL_PRINT_WARN("%s, Register the Generic Power Level Server and the Generic Power Level fail, status = 0x%x\n", __func__, status);
            }
        }
        while (0);
    }

    return (status);
}
#endif
/*
 * GLOBAL FUNCTIONS
 ****************************************************************************************
 */

uint16_t mm_gens_init(bool reset, void *p_env, const mm_cfg_t *p_cfg)
{
    // Return size of environment
    return (0);
}

uint16_t mm_gens_get_env_size(const mm_cfg_t *p_cfg)
{
    // Return size of environment
    return (0);
}

uint16_t mm_gens_register(uint8_t elmt_idx, uint8_t mdl_cfg_idx, bool main)
{
    // Status
    uint16_t status = MESH_ERR_NO_ERROR;

    switch (mdl_cfg_idx)
    {
        case (MM_CFG_IDX_GENS_ONOFF):
        {
            // Add Generic OnOff Server model if not already present
            status = mm_gens_register_oo(elmt_idx, main);
        } break;
#if (BLE_MESH_MDL_GENS_LVL)
        case (MM_CFG_IDX_GENS_LEVEL):
        {
            // Add Generic Level Server model if not already present
            if (mm_tb_state_get_lid(elmt_idx, MM_ID_GENS_LVL) == MESH_INVALID_LID)
            {
                // Model local index
                m_lid_t mdl_lid;

                status = mm_gens_lvl_register(elmt_idx, &mdl_lid);

                if ((status == MESH_ERR_NO_ERROR) && main)
                {
                    // Group local index
                    m_lid_t grp_lid;

                    // Create group
                    mm_tb_bind_add_group(0, elmt_idx, &grp_lid, mdl_lid,
                                         mm_gens_lvl_cb_grp_event, NULL);
                }
            }
        } break;
#endif

#if (BLE_MESH_MDL_GENS_DTT)
        case (MM_CFG_IDX_GENS_DFT_TRANS_TIME):
        {
            // Add Generic Default Transition Time Server model if not already present
            if (mm_tb_state_get_lid(elmt_idx, MM_ID_GENS_DTT) == MESH_INVALID_LID)
            {
                status = mm_gens_dtt_register(elmt_idx);
            }
        } break;
#endif

#if (BLE_MESH_MDL_GENS_POO)
        case (MM_CFG_IDX_GENS_POWER_ONOFF):
        {
            // Add models needed for use of Generic Power OnOff Server model
            status = mm_gens_register_poo(elmt_idx, main);
        } break;
#endif

#if (BLE_MESH_MDL_GENS_PLVL)
        case (MM_CFG_IDX_GENS_POWER_LEVEL):
        {
            // Add models needed for use of Generic Power Level Server model
            status = mm_gens_register_plvl(elmt_idx, main);
        } break;
#endif

#if (BLE_MESH_MDL_GENS_BAT)
        case (MM_CFG_IDX_GENS_BATTERY):
        {
            // Add Generic Battery Server model if not already present
            if (mm_tb_state_get_lid(elmt_idx, MM_ID_GENS_BAT) == MESH_INVALID_LID)
            {
                status = mm_gens_bat_register(elmt_idx);
            }
        } break;
#endif

#if (BLE_MESH_MDL_GENS_LOC)
        case (MM_CFG_IDX_GENS_LOCATION):
        {
            // Add Generic Location Server and Generic Location Setup Server models
            // if not already present
            if ((mm_tb_state_get_lid(elmt_idx, MM_ID_GENS_LOC) == MESH_INVALID_LID)
                    && (mm_tb_state_get_lid(elmt_idx, MM_ID_GENS_LOCS) == MESH_INVALID_LID))
            {
                status = mm_gens_loc_register(elmt_idx);
            }
        } break;
#endif
        default:
        {
            // Configuration is unknown
            ASSERT_INFO(0, mdl_cfg_idx, 0);
            status = MESH_ERR_COMMAND_DISALLOWED;

        }
    }
    return (status);
}

void mm_gens_add_to_grp(uint8_t elmt_idx, uint32_t model_id, uint8_t grp_lid)
{
    // Retrieve model local index
    m_lid_t mdl_lid = mm_tb_state_get_lid(elmt_idx, model_id);

    if (mdl_lid != MESH_INVALID_LID)
    {
        mm_tb_bind_grp_event_cb cb_grp_event = NULL;
        mm_tb_bind_set_state_cb cb_set_state = NULL;

        switch (model_id)
        {
            case (MM_ID_GENS_OO):
            {
                cb_grp_event = mm_gens_oo_cb_grp_event;
                cb_set_state = mm_gens_oo_cb_set_state;
            } break;
#if (BLE_MESH_MDL_GENS_LVL)
            case (MM_ID_GENS_LVL):
            {
                cb_grp_event = mm_gens_lvl_cb_grp_event;
                cb_set_state = mm_gens_lvl_cb_set_state;
            } break;
#endif
            default:
            {
            } break;
        }

        mm_tb_bind_group_add_mdl(grp_lid, mdl_lid, model_id, cb_grp_event, cb_set_state);
    }
}
#if (BLE_MESH_MDL_GENS_BAT)
void mm_gens_cfm_bat(uint16_t status, uint8_t elmt_idx, uint8_t bat_lvl, uint32_t time_charge,
                     uint32_t time_discharge, uint8_t flags)
{
    // Look for Generic Battery Server model in the list of models
    m_lid_t mdl_lid = mm_tb_state_get_lid(elmt_idx, MM_ID_GENS_BAT);

    if (mdl_lid != MESH_INVALID_LID)
    {
        // Get environment
        mm_tb_state_mdl_env_t *p_mdl_env = mm_tb_state_get_env(mdl_lid);

        // Inform the model about reception of the confirmation
        mm_gens_bat_cfm(p_mdl_env, status, bat_lvl, time_discharge, time_charge, flags);
    }
}
#endif //(BLE_MESH_MDL_GENS_POO)

#if (BLE_MESH_MDL_GENS_LOC)
void mm_gens_cfm_locg(uint16_t status, uint8_t elmt_idx, int32_t latitude, int32_t longitude,
                      int16_t altitude)
{
    // Look for Generic Location Server model in the list of models
    m_lid_t mdl_lid = mm_tb_state_get_lid(elmt_idx, MM_ID_GENS_LOC);

    if (mdl_lid != MESH_INVALID_LID)
    {
        // Get environment
        mm_tb_state_mdl_env_t *p_mdl_env = mm_tb_state_get_env(mdl_lid);

        // Inform the model about reception of the confirmation
        mm_gens_loc_global_cfm(p_mdl_env, status, latitude, longitude, altitude);
    }
}

void mm_gens_cfm_locl(uint16_t status, uint8_t elmt_idx, int16_t north, int16_t east,
                      int16_t altitude, uint8_t floor, uint16_t uncertainty)
{
    // Look for Generic Location Server model in the list of models
    m_lid_t mdl_lid = mm_tb_state_get_lid(elmt_idx, MM_ID_GENS_LOC);

    if (mdl_lid != MESH_INVALID_LID)
    {
        // Get environment
        mm_tb_state_mdl_env_t *p_mdl_env = mm_tb_state_get_env(mdl_lid);

        // Inform the model about reception of the confirmation
        mm_gens_loc_local_cfm(p_mdl_env, status, north, east, altitude, floor, uncertainty);
    }
}
#endif //(BLE_MESH_MDL_GENS_POO)

#endif //(BLE_MESH_MDL_GENS) //(BLE_MESH_MDL_GENS)
/// @} end of group
