#include "m_api.h"
#include "mesh_api_msg.h"
#include "mal.h"
#include "m_fnd_int.h"
#include "m_fnd_Scenes.h"

#include "mm_gens_int.h"
#include "mm_lights_int.h"

#include "app_light_ali_server.h"
#include "app.h"
#include "mm_vendors.h"
#include "m_fnd_BLOB_Transfer.h"
#include "m_fnd_Fw_Update.h"
#include "uart.h"
#include "led.h"
#include "pwm.h"
#include "flash.h"
#include "nvds.h"

/**
 * @brief initialize alibaba lightness light
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
 
typedef struct
{
    mesh_tb_timer_t report_timer;
    uint16_t attr;
    uint8_t len;
    uint8_t status;
    uint8_t retry_times;
} ali_light_report_t;

ali_light_report_t  ali_light_report;

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


static uint16_t scene_num[5];
static uint16_t cur_scene_num;

int32_t light_scene_server_data(const m_fnd_model_env_p pmodel_info, uint32_t type,
                                void *pargs)
{
    MESH_APP_PRINT_INFO("%s type = 0x%x\r\n", __func__, type);
    switch (type)
    {
        case M_FND_SCENE_SERVER_GET_PRESENT:
        {
            m_fnd_scene_server_get_present_t *pdata = pargs;
            pdata->scene_number = cur_scene_num;
        } break;

        case M_FND_SCENE_SERVER_RECALL:
        {
            m_fnd_scene_server_set_t *pdata = pargs;
            if (0 == pdata->remaining_time.num_steps)
            {
                cur_scene_num = pdata->scene_number;
            }
        } break;

        case M_FND_SCENE_SERVER_STORE:
        {
            m_fnd_scenes_store_t *pdata = pargs;

            cur_scene_num = pdata->scene_number;
            {
                for (int i =0; i < M_FND_SCENES_STORE_MAX; i++)
                {
                    if ((scene_num[i] == 0 ) || (scene_num[i] == pdata->scene_number))
                    {
                        scene_num[i]    = pdata->scene_number;
                        break;
                    }
                }
            }

        } break;

        case M_FND_SCENE_SERVER_DELETE:
        {
            MESH_APP_PRINT_INFO("M_FND_SCENE_SERVER_DELETE\r\n");
            m_fnd_scenes_delete_t *pdata = pargs;
            for (int i =0; i < M_FND_SCENES_STORE_MAX; i++)
            {
                if (scene_num[i] == pdata->scene_number)
                {
                    scene_num[i] = 0;
                    break;
                }
            }
        } break;

        case M_FND_SCENE_SERVER_GET_REGISTER:
        {
            MESH_APP_PRINT_INFO("M_FND_SCENE_SERVER_GET_REGISTER\r\n");
            m_fnd_scene_server_get_register_t *pdata = pargs;
            uint8_t j = 0;
            for (int i =0; i < M_FND_SCENES_STORE_MAX; i++)
            {
                if (scene_num[i] != 0)
                {
                    pdata->scenes[j++] =  scene_num[i];
                }
            }
        } break;
        default:break;
    }

    return 0;
}

void light_state_store_timer_cb(void *timer)
{
    MESH_APP_PRINT_INFO("light_state_store\r\n");

    if (light_state_get() != 0)
    {
        light_state_nv_store(FLASH_LIGHT_PARAM_TYPE_LIGHT_STATE);

    }

    mesh_tb_timer_set(&light_state_store_timer, light_state_store_timer.period);
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
        light_lightness_set(0);

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

    if (prov_flash_count%2 == 0)
    {
        light_lightness_set(0x8000);
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

void light_status_report_timer_cb(void *timer)
{
    MESH_APP_PRINT_INFO("%s\n", __func__);

    if (mm_vendors_cfm_recved())
    {
        mm_vendors_cfm_clear();
        ali_light_report.report_timer.cb = NULL;
        ali_light_report.report_timer.period = MM_LIGHT_REPORT_MS;
        ali_light_report.retry_times = 0;
        mesh_tb_timer_clear(&ali_light_report.report_timer);
        MESH_APP_PRINT_INFO("receive cfm message\n");
    }
    else
    {
        if (ali_light_report.retry_times <= MM_REPORT_MAX_TIMES)
        {
            ali_light_report.retry_times++;
            mm_vendor_attr_indication(ali_light_report.attr, ali_light_report.len, &ali_light_report.status);
            //reset timer
            mesh_tb_timer_set(&ali_light_report.report_timer, ali_light_report.report_timer.period);
        }
        else
        {
            ali_light_report.retry_times = 0;
            mesh_tb_timer_clear(&ali_light_report.report_timer);
        }
    }
}

void ali_light_status_report(uint16_t attr, uint8_t len, uint8_t status)
{
    mm_vendors_cfm_clear();
    mm_vendor_attr_indication(attr, len, &status);
    ali_light_report.attr = attr;
    ali_light_report.len = len;
    ali_light_report.status = status;
    ali_light_report.report_timer.cb = light_status_report_timer_cb;
    ali_light_report.report_timer.period = MM_LIGHT_REPORT_MS;
    ali_light_report.retry_times++;
    mesh_tb_timer_set(&ali_light_report.report_timer, ali_light_report.report_timer.period);
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

void light_ali_quick_onoff_timer(void)
{
    static uint8_t flag = 0; //TO DO: need to delete it?
    if (flag == 0)
    {
        MESH_APP_PRINT_INFO("light_ali_quick_onoff_timer:power_on_count = %d\n", quick_onoff_count);
        quick_onoff_timer.cb = light_power_on_timeout_cb;
        quick_onoff_timer.period = LIGHT_POWER_ON_TIME;
        mesh_tb_timer_set(&quick_onoff_timer, quick_onoff_timer.period);
        flag = 1;
    }
}

void light_ali_app_init(void)
{
    uint32_t ret;

    MESH_APP_PRINT_INFO("light_ali_app_init quick =%d\n", quick_onoff_count);
    ret = light_app_nv_restore(FLASH_LIGHT_PARAM_TYPE_POWER_ON_COUNT);

    if (ret == 0)
    {
        quick_onoff_count++;
        if (quick_onoff_count < 6)
        {
            MESH_APP_PRINT_INFO("quick_onoff_count0 %d\n", quick_onoff_count);
            light_ali_quick_onoff_timer();
            light_state_nv_store(FLASH_LIGHT_PARAM_TYPE_POWER_ON_COUNT);
        }
        else //quick_onoff_count>=6, do factory reset
        {
            MESH_APP_PRINT_INFO("quick_onoff_count1 %d,do factory reset\n", quick_onoff_count);

            light_unBind_complete();
            light_ali_quick_onoff_timer();
            //   light_ali_factory_reset();
            light_state_nv_store(FLASH_LIGHT_PARAM_TYPE_POWER_ON_COUNT);
            return;
        }
    }
    else
    {
        MESH_APP_PRINT_INFO("quick_onoff_count2 %d\n", quick_onoff_count);
        quick_onoff_count = 1;
        light_state_nv_store(FLASH_LIGHT_PARAM_TYPE_POWER_ON_COUNT);
        light_ali_quick_onoff_timer();
    }

    ret = light_app_nv_restore(FLASH_LIGHT_PARAM_TYPE_LIGHT_STATE);

    if (ret!=0)
    {
        MESH_APP_PRINT_INFO("light state nv read failed\n");
        light_factory_reset();
        MESH_APP_PRINT_INFO("light state nv read failed\n");
    }
    else
    {
        MESH_APP_PRINT_INFO("flash read succ\n");
        light_state_recover();
        MESH_APP_PRINT_INFO("flash opt end\n");
    }
    light_state_store_timer.cb = light_state_store_timer_cb;
    light_state_store_timer.period = 10000;
    mesh_tb_timer_set(&light_state_store_timer, light_state_store_timer.period);
}


m_lid_t g_vdr_lid, g_ln_mdl_lid, g_ctl_ln_mdl_lid;
m_lid_t g_hsl_ln_mdl_lid, g_ctl_mdl_lid;
m_lid_t g_ctlt_mdl_lid, g_oo_mdl_lid;
m_lid_t g_hsl_mdl_lid, g_hslh_mdl_lid, g_hslsat_mdl_lid;

void app_ai_lights_models_init(uint8_t elmt_idx)
{
    uint16_t status = MESH_ERR_MDL_ALREADY_REGISTERED;

    MESH_APP_PRINT_DEBUG("%s, hsl 0x%x, oo 0x%x, ln0 0x%x, ln1 0x%x, ln2 0x%x, ctl 0x%x, vendors 0x%x\n", 
                         __func__, mm_tb_state_get_lid(elmt_idx, MM_ID_LIGHTS_HSL),
                         mm_tb_state_get_lid(elmt_idx, MM_ID_GENS_OO),
                         mm_tb_state_get_lid(elmt_idx, MM_ID_LIGHTS_LN),
                         mm_tb_state_get_lid(elmt_idx + 1, MM_ID_LIGHTS_LN),
                         mm_tb_state_get_lid(elmt_idx + 2, MM_ID_LIGHTS_LN),
                         mm_tb_state_get_lid(elmt_idx, MM_ID_LIGHTS_CTL),
                         mm_tb_state_get_lid(elmt_idx, MM_ID_VENDORS));
    if ((mm_tb_state_get_lid(elmt_idx, MM_ID_LIGHTS_HSL) == MESH_INVALID_LID)
            && (mm_tb_state_get_lid(elmt_idx, MM_ID_GENS_OO) == MESH_INVALID_LID)
            && (mm_tb_state_get_lid(elmt_idx, MM_ID_LIGHTS_LN) == MESH_INVALID_LID)
            && (mm_tb_state_get_lid(elmt_idx + 1, MM_ID_LIGHTS_LN) == MESH_INVALID_LID)
            && (mm_tb_state_get_lid(elmt_idx + 2, MM_ID_LIGHTS_LN) == MESH_INVALID_LID)
            && (mm_tb_state_get_lid(elmt_idx, MM_ID_LIGHTS_CTL) == MESH_INVALID_LID)
            && (mm_tb_state_get_lid(elmt_idx, MM_ID_VENDORS) == MESH_INVALID_LID))
    {
        do
        {
            mm_vendors_register(elmt_idx, &g_vdr_lid);
            m_fnd_scenes_init(light_scene_server_data);
            m_fnd_blob_init(elmt_idx);
            m_fnd_fw_update_init(elmt_idx);

            // Register Light Lightness Server model and associated models
            status = mm_gens_oo_register(elmt_idx, &g_oo_mdl_lid);
            if (status != MESH_ERR_NO_ERROR)
            {
                break;
            }

            MESH_APP_PRINT_DEBUG("%s, g_oo_mdl_lid = %d, model_id 0x%x\n", __func__, g_oo_mdl_lid, m_tb_mio_get_model_id(g_oo_mdl_lid));
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



