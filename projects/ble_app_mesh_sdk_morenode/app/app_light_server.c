#include "m_api.h"
#include "mesh_api_msg.h"
#include "mal.h"
#include "m_fnd_int.h"

#include "mm_gens_int.h"
#include "mm_lights_int.h"

#include "app_light_server.h"
#include "app.h"
#include "mm_vendors_morenode.h"
#include "m_fnd_Fw_Update.h"
#include "uart.h"
#include "led.h"
#include "pwm.h"
#include "flash.h"
#include "nvds.h"
#include "mm_vendors_TRSPX.h"

/**
 * @brief initialize lightness light
 */

/** 0:CWRGB, 2:RGB */
#define GENERIC_ON_OFF_OFFSET       0

uint8_t led_status = 0;
uint32_t Temperature_percent;
uint32_t lightness_value = PWM_16KHZ/2;

/***************factory restore*******************/
extern uint16_t quick_onoff_count;
mesh_tb_timer_t quick_onoff_timer;

mesh_tb_timer_t light_state_store_timer;

mesh_tb_timer_t light_prov_timer;
mesh_tb_timer_t light_unbind_timer;
mesh_tb_timer_t light_prov_timeout_timer;


/****************lighting states*****************/
//these values will be moved to each model's database later
//generic onoff states



//light lightness states


//CTL model states
extern uint16_t ctl_lightness;
extern uint16_t ctl_temperature;
extern uint16_t ctl_delta_uv;
//HSL model states
extern uint16_t hsl_lightness;
uint16_t light_hsl_16[3];

void light_state_store_timer_cb(void *timer)
{
    MESH_APP_PRINT_INFO("light_state_store\r\n"); //APP_DBG_PRINTF


    if (light_state_get() != 0)
    {
        light_state_nv_store(FLASH_LIGHT_PARAM_TYPE_LIGHT_STATE);

    }
}

void light_power_on_timeout_cb(void *timer)
{
    if (quick_onoff_count != 0)
    {
        quick_onoff_count = 0;
        light_state_nv_store(FLASH_LIGHT_PARAM_TYPE_POWER_ON_COUNT);
    }
    MESH_APP_PRINT_INFO("light_power_on_timeout_cb:power_on_count = %d\n", quick_onoff_count);
}

void light_prov_success_flash_timer_cb(void *timer)
{
    static uint8_t prov_flash_count = 6;
    uint16_t lightnessRec;

    MESH_APP_PRINT_INFO("light_prov_success_flash_timer_cb:prov_flash_count = %d\n", prov_flash_count);


    if (prov_flash_count%2 == 0)
    {
        //  3 or 1;

        light_lightness_set(1);

    }
    else
    {
        light_lightness_set(0x8000);

    }

    if (prov_flash_count > 1)
    {
        light_prov_timer.cb = light_prov_success_flash_timer_cb;
        light_prov_timer.period = LIGHT_PROV_FLASH_INTERVAL;
        mesh_tb_timer_set(&light_prov_timer, light_prov_timer.period);
        prov_flash_count--;

    }
    else
    {
        prov_flash_count = 6;
        LED_default_ctrl_param_set(DEFAULT_PWM_MOVE_STEP, MIN_MOVE_STEP_NUM);
    }

}

void light_prov_fail_flash_timer_cb(void *timer)
{
    static uint8_t prov_flash_count = 6;
    uint16_t lightnessRec;

    MESH_APP_PRINT_INFO("light_prov_fail_flash_timer_cb:prov_flash_count = %d\n", prov_flash_count);


    if (prov_flash_count % 2 == 0)
    {
        //  3 or 1;
        //  lightnessRec = light_lightness_bak;
        light_lightness_set(0x8000);
        // light_lightness_bak = lightnessRec;
    }
    else
    {
        light_lightness_set(0);
    }

    if (prov_flash_count > 1)
    {
        light_prov_timer.cb = light_prov_fail_flash_timer_cb;
        light_prov_timer.period = LIGHT_PROV_FLASH_INTERVAL;
        mesh_tb_timer_set(&light_prov_timer, light_prov_timer.period);
        prov_flash_count--;

    }
    else
    {
        prov_flash_count = 6;

        LED_default_ctrl_param_set(DEFAULT_PWM_MOVE_STEP, MIN_MOVE_STEP_NUM);
    }

}

#define LIGHT_PROV_TIMEOUT  40000
void light_prov_start(void)
{
    MESH_APP_PRINT_INFO("light_prov_start\n");
    light_prov_timeout_timer.cb = light_prov_fail_flash_timer_cb;
    light_prov_timeout_timer.period = LIGHT_PROV_TIMEOUT;
    mesh_tb_timer_set(&light_prov_timeout_timer, light_prov_timeout_timer.period);
}

void light_prov_end(void)
{
    MESH_APP_PRINT_INFO("light_prov_end\n");
    light_prov_timeout_timer.cb = NULL;
    light_prov_timeout_timer.period = LIGHT_PROV_TIMEOUT;
    mesh_tb_timer_clear(&light_prov_timeout_timer);
}

void light_prov_complete(void)
{
    MESH_APP_PRINT_INFO("prov_complete\n");
    light_prov_end();
    LED_default_ctrl_param_set(262, 200);
    light_prov_success_flash_timer_cb(0); //can be called directly
}

void light_prov_fail(void)
{
    MESH_APP_PRINT_INFO("light_prov_fail\n");
    LED_default_ctrl_param_set(262, 200);
    light_prov_fail_flash_timer_cb(0); //can be called directly
}

void light_unBind_flash_timer_cb(void *timer)
{
    static uint8_t unbind_flash_count = 10;
    uint16_t lightnessRec;

    MESH_APP_PRINT_INFO("light_unBind_flash_timer_cb:prov_flash_count = %d\n", unbind_flash_count);

    if (unbind_flash_count % 2 == 0)
    {
        //  3 or 1;
        light_lightness_set(0);
    }
    else
    {
        light_lightness_set(0x8000);
    }

    if (unbind_flash_count > 1)
    {
        light_unbind_timer.cb = light_unBind_flash_timer_cb;
        light_unbind_timer.period = LIGHT_PROV_FLASH_INTERVAL;
        mesh_tb_timer_set(&light_unbind_timer, light_unbind_timer.period);
        unbind_flash_count--;
    }
    else
    {
        unbind_flash_count = 10;
        LED_default_ctrl_param_set(DEFAULT_PWM_MOVE_STEP, MIN_MOVE_STEP_NUM);
    }

}

void light_unBind_complete(void)
{
    MESH_APP_PRINT_INFO("light_unBind_complete\n");

    nvds_deinit();
    // Initialize NVDS module
    struct nvds_env_tag env;
    env.flash_read = &flash_read;
    env.flash_write = &flash_write;
    env.flash_erase = &flash_erase;
    nvds_init(env);

    light_mode_set(LIGHT_MODE_CTL);

    light_lightness_set(0x8000);
    LED_default_ctrl_param_set(262, 200);
    light_unBind_flash_timer_cb(0); //can be called directly
}

void light_quick_onoff_timer(void)
{
    static uint8_t flag = 0; //TO DO: need to delete it?
    if (flag == 0)
    {
        MESH_APP_PRINT_INFO("light_quick_onoff_timer:power_on_count = %d\n", quick_onoff_count);
        quick_onoff_timer.cb = light_power_on_timeout_cb;
        quick_onoff_timer.period = LIGHT_POWER_ON_TIME;
        mesh_tb_timer_set(&quick_onoff_timer, quick_onoff_timer.period);
        flag = 1;
    }
}

void light_app_init(void)
{
    uint32_t ret;

    MESH_APP_PRINT_INFO("light_app_init quick =%d\r\n", quick_onoff_count);
#if 0
    ret = light_app_nv_restore(FLASH_LIGHT_PARAM_TYPE_POWER_ON_COUNT);

    if (ret == 0)
    {
        quick_onoff_count++;
        if (quick_onoff_count < 6)
        {
            MESH_APP_PRINT_INFO("quick_onoff_count0 %d\n", quick_onoff_count);
            light_quick_onoff_timer();
            light_state_nv_store(FLASH_LIGHT_PARAM_TYPE_POWER_ON_COUNT);
        }
        else //quick_onoff_count>=6, do factory reset
        {
            MESH_APP_PRINT_INFO("quick_onoff_count1 %d,do factory reset\n", quick_onoff_count);

            light_unBind_complete();
            light_quick_onoff_timer();
            light_state_nv_store(FLASH_LIGHT_PARAM_TYPE_POWER_ON_COUNT);
            return;
        }

    }
    else
    {
        MESH_APP_PRINT_INFO("quick_onoff_count2 %d\n", quick_onoff_count);
        quick_onoff_count = 1;
        light_state_nv_store(FLASH_LIGHT_PARAM_TYPE_POWER_ON_COUNT);
        light_quick_onoff_timer();
    }

#endif
    ret = light_app_nv_restore(FLASH_LIGHT_PARAM_TYPE_LIGHT_STATE);

    if (ret!=0)
    {
        MESH_APP_PRINT_DEBUG("light state nv read failed\n");
        light_factory_reset();
        MESH_APP_PRINT_DEBUG("light state nv read failed\n");
    }
    else
    {
        MESH_APP_PRINT_INFO("flash read succ\n");
        light_state_recover();
        MESH_APP_PRINT_INFO("flash opt end\n");
    }
    #if 0	///2020 0116
    light_state_store_timer.cb = light_state_store_timer_cb;
    light_state_store_timer.period = 10000;
    mesh_tb_timer_set(&light_state_store_timer, light_state_store_timer.period);
    #endif
}

m_lid_t g_vdr_lid, g_ln_mdl_lid, g_ctl_ln_mdl_lid;
m_lid_t g_hsl_ln_mdl_lid, g_ctl_mdl_lid;
m_lid_t g_ctlt_mdl_lid, g_oo_mdl_lid;
m_lid_t g_hsl_mdl_lid, g_hslh_mdl_lid, g_hslsat_mdl_lid;
m_lid_t g_vdr_trspx_lid;

void app_lights_models_init(uint8_t elmt_idx)
{
    uint16_t status = MESH_ERR_MDL_ALREADY_REGISTERED;

    if ((mm_tb_state_get_lid(elmt_idx, MM_ID_LIGHTS_HSL) == MESH_INVALID_LID)
            && (mm_tb_state_get_lid(elmt_idx, MM_ID_GENS_OO) == MESH_INVALID_LID)
            && (mm_tb_state_get_lid(elmt_idx, MM_ID_LIGHTS_LN) == MESH_INVALID_LID)
            && (mm_tb_state_get_lid(elmt_idx + 1, MM_ID_LIGHTS_LN) == MESH_INVALID_LID)
            && (mm_tb_state_get_lid(elmt_idx + 2, MM_ID_LIGHTS_LN) == MESH_INVALID_LID)
            && (mm_tb_state_get_lid(elmt_idx, MM_ID_LIGHTS_CTL) == MESH_INVALID_LID)
            && (mm_tb_state_get_lid(elmt_idx, MM_ID_VENDORS) == MESH_INVALID_LID)
			&& (mm_tb_state_get_lid(elmt_idx, MM_ID_VENDORS_TRSPX) == MESH_INVALID_LID))

	
    {
        do
        {
           status = mm_vendors_register(elmt_idx, &g_vdr_lid);
 
	   MESH_APP_PRINT_INFO("vendors register status = 0x%x\n",status);

		  status = mm_vendors_trspx_register(elmt_idx, &g_vdr_trspx_lid);

            // Register Light Lightness Server model and associated models
            status = mm_gens_oo_register(elmt_idx, &g_oo_mdl_lid);
            if (status != MESH_ERR_NO_ERROR)
            {
                break;
            }
            // Register Light Lightness Server model and associated models
            status = mm_lights_ln_register(elmt_idx, &g_ln_mdl_lid);
            if (status != MESH_ERR_NO_ERROR)
            {
                break;
            }
            status = mm_lights_ln_register(elmt_idx + 1, &g_ctl_ln_mdl_lid);
            if (status != MESH_ERR_NO_ERROR)
            {
                break;
            }

            status = mm_lights_ctl_register(elmt_idx, &g_ctl_mdl_lid, &g_ctlt_mdl_lid);
            if (status != MESH_ERR_NO_ERROR)
            {
                break;
            }

            if (status == MESH_ERR_NO_ERROR)
            {

                // Group local index
                m_lid_t grp_lid;

                // Create group and set Light Lightness Server model as main model
                mm_tb_bind_add_group(1, elmt_idx, &grp_lid, g_ln_mdl_lid,
                                     mm_lights_ln_cb_grp_event, mm_lights_ln_cb_trans_req);


                // Add Generic OnOff Server model to the group
                mm_api_grp_add_local(elmt_idx, MM_ID_GENS_OO, grp_lid);

            }

            if (status == MESH_ERR_NO_ERROR)
            {
                // Group local index
                m_lid_t grp_lid;

                // Create group and set Light CTL Server model as main model
                mm_tb_bind_add_group(1, elmt_idx, &grp_lid, g_ctl_mdl_lid,
                                     mm_lights_ctl_cb_grp_event, mm_lights_ctl_cb_trans_req);


                // Add Light Lightness Server model to the group
                mm_tb_bind_group_add_mdl(grp_lid, g_ctl_ln_mdl_lid, MM_ID_LIGHTS_LN,
                                         mm_lights_ln_cb_grp_event, mm_lights_ln_cb_set_state);


                // Create group and set Light CTL Temperature Server model as main model
                mm_tb_bind_add_group(0, elmt_idx + 1, &grp_lid, g_ctlt_mdl_lid,
                                     mm_lights_ctl_cb_grp_event_temp, mm_lights_ctl_cb_trans_req_temp);
            }

        }
        while (0);
    }
}



