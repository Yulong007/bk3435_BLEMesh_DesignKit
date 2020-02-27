/**
 ****************************************************************************************
 *
 * @file app.c
 *
 * @brief Application entry point
 *
 * Copyright (C) RivieraWaves 2009-2015
 *
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup APP
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"             // SW configuration

#if (BLE_APP_PRESENT)
#include <string.h>
//#include "rwapp_config.h"
#include "app_task.h"                // Application task Definition
#include "app.h"                     // Application Definition
#include "gap.h"                     // GAP Definition
#include "gapm_task.h"               // GAP Manager Task API
#include "gapc_task.h"               // GAP Controller Task API

#include "co_bt.h"                   // Common BT Definition
#include "co_math.h"                 // Common Maths Definition
#include "ke_timer.h"
#include "app_fff0.h"                 // Application security Definition
#include "app_dis.h"                 // Device Information Service Application Definitions
#include "app_batt.h"                // Battery Application Definitions
#include "app_oads.h"                 // Application oads Definition
#include "app_mesh.h"

#include "nvds.h"                    // NVDS Definitions
#include "rf.h"
#include "uart.h"
#include "adc.h"
#include "gpio.h"
#include "wdt.h"
#include "mesh_log.h"

/*
 * DEFINES
 ****************************************************************************************
 */
#define APP_DEVICE_NAME_LENGTH_MAX      (18)


/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

typedef void (*appm_add_svc_func_t)(void);

/*
 * ENUMERATIONS
 ****************************************************************************************
 */

/// List of service to add in the database
enum appm_svc_list
{
    APPM_SVC_FFF0,
    APPM_SVC_DIS,
    APPM_SVC_BATT,
    APPM_SVC_OADS,
    APPM_SVC_MESH,
    APPM_SVC_LIST_STOP,
};

/*
 * LOCAL VARIABLES DEFINITIONS
 ****************************************************************************************
 */

/// Application Task Descriptor
static const struct ke_task_desc TASK_DESC_APP = {NULL, &appm_default_handler,
           appm_state, APPM_STATE_MAX, APP_IDX_MAX
};

/// List of functions used to create the database
static const appm_add_svc_func_t appm_add_svc_func_list[APPM_SVC_LIST_STOP] =
{
    (appm_add_svc_func_t)app_fff0_add_fff0s,
    (appm_add_svc_func_t)app_dis_add_dis,
    (appm_add_svc_func_t)app_batt_add_bas,
    (appm_add_svc_func_t)app_oad_add_oads,
    (appm_add_svc_func_t)app_mesh_add_mesh,
};

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

/// Application Environment Structure
struct app_env_tag app_env;

/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */

void appm_init()
{
    uint8_t key_len = KEY_LEN;

    // Reset the application manager environment
    memset(&app_env, 0, sizeof(app_env));

    // Create APP task
    ke_task_create(TASK_APP, &TASK_DESC_APP);

    // Initialize Task state
    ke_state_set(TASK_APP, APPM_INIT);

    if (nvds_get(NVDS_TAG_LOC_IRK, &key_len, app_env.loc_irk) != NVDS_OK)
    {
        uint8_t counter;

        // generate a new IRK
        for (counter = 0; counter < KEY_LEN; counter++)
        {
            app_env.loc_irk[counter]    = (uint8_t)co_rand_word();
        }

        // Store the generated value in NVDS
        if (nvds_put(NVDS_TAG_LOC_IRK, KEY_LEN, (uint8_t *)&app_env.loc_irk) != NVDS_OK)
        {
            ASSERT_INFO(0, NVDS_TAG_LOC_IRK, 0);
        }
    }

    /*------------------------------------------------------
     * INITIALIZE ALL MODULES
     *------------------------------------------------------*/

    // Device Information Module
    app_dis_init();

    // Battery Module
    app_batt_init();

    app_oads_init();

    app_mesh_init();
}

bool appm_add_svc(void)
{
    // Indicate if more services need to be added in the database
    bool more_svc = false;

    // Check if another should be added in the database
    if (app_env.next_svc != APPM_SVC_LIST_STOP)
    {
        ASSERT_INFO(appm_add_svc_func_list[app_env.next_svc] != NULL, app_env.next_svc, 1);

        // Call the function used to add the required service
        appm_add_svc_func_list[app_env.next_svc]();

        // Select following service to add
        app_env.next_svc++;
        more_svc = true;
    }

    return more_svc;
}



void appm_update_param(struct gapc_conn_param *conn_param)
{
    // Prepare the GAPC_PARAM_UPDATE_CMD message
    struct gapc_param_update_cmd *cmd = KE_MSG_ALLOC(GAPC_PARAM_UPDATE_CMD,
                                        KE_BUILD_ID(TASK_GAPC, app_env.conidx), TASK_APP,
                                        gapc_param_update_cmd);

    cmd->operation  = GAPC_UPDATE_PARAMS;
    cmd->intv_min   = conn_param->intv_min;
    cmd->intv_max   = conn_param->intv_max;
    cmd->latency    = conn_param->latency;
    cmd->time_out   = conn_param->time_out;

    // not used by a slave device
    cmd->ce_len_min = 0xFFFF;
    cmd->ce_len_max = 0xFFFF;

    MESH_APP_PRINT_INFO("intv_min = %d,intv_max = %d,latency = %d,time_out = %d\r\n",
                        cmd->intv_min, cmd->intv_max, cmd->latency, cmd->time_out);

    // Send the message
    ke_msg_send(cmd);
}


uint8_t appm_set_dev_name(char* name, uint8_t len)
{
    app_env.dev_name_len = len;
    // copy name to provided pointer
    memcpy(app_env.dev_name, name, len);
    // return name length
    MESH_APP_PRINT_INFO("func : %s,%s\r\n", __func__, app_env.dev_name);
    return app_env.dev_name_len;
}


uint8_t appm_get_dev_name(uint8_t* name)
{
    // copy name to provided pointer
    memcpy(name, app_env.dev_name, app_env.dev_name_len);
    // return name length
    return app_env.dev_name_len;
}

void hci_ahi_event_send( uint8_t(*callback) (void*, uint8_t), void* dummy)
{
    struct hci_ahi_msg_pro_t *cmd = KE_MSG_ALLOC(HCI_AHI_MSG_PRO_CMD,
                                    TASK_APP, TASK_APP,
                                    hci_ahi_msg_pro_t);

    cmd->callback = callback;
    cmd->dummy = dummy;
    // Send the message
    ke_msg_send(cmd);


}


#endif //(BLE_APP_PRESENT)

/// @} APP


