/**
 ****************************************************************************************
 * @file m_fnd_generic_transition_time.c
 *
 * @brief Mesh generic_transition Server Model
 *
 * Copyright (C) Beken 2018-2019
 *
 ****************************************************************************************
 */
/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "m_fnd_int.h"      // Internal defines
#include <string.h>         // For use of memcpy function
#include "co_math.h"        // For use of CO_BIT macros
#include "m_lay.h"
#include "mesh_tb_timer.h"
#include "m_tb_mio.h"
#include "m_fnd_generic_transition_time.h"

#include "uart.h"

typedef struct _trans_list
{
    struct _trans_list *prev;
    struct _trans_list *next;
} trans_list_t, *trans_list_p;


typedef struct _trans_time_remain
{
    mm_tb_state_mdl_env_t * pmodel_info;
    uint32_t trans_type;
    generic_transition_time_t total_time;
    int8_t remain_num_steps;
    int16_t remain_step_ticks;
    generic_transition_step_change_cb step_change;
    trans_list_t node;
} trans_time_remain_t, *trans_time_remain_p;

static trans_list_t trans_list_head;


__STATIC mesh_tb_timer_t  m_fnd_mesh_tick_time;


const int16_t tick_map[] = {1, 10, 100, 6000};

static bool trans_time_is_empty(void)
{
    return (trans_list_head.next == &trans_list_head);
}

static bool trans_time_insert(mm_tb_state_mdl_env_t * pmodel_info,
                              uint32_t trans_type,
                              generic_transition_time_t trans_time,
                              generic_transition_step_change_cb step_change)
{
    trans_list_p pcur = trans_list_head.next;
    trans_time_remain_p ptime = NULL;
    for (; pcur != &trans_list_head; pcur = pcur->next)
    {
        ptime = CONTAINER_OF(pcur, trans_time_remain_t, node);
        if ((ptime->pmodel_info == pmodel_info) &&
                (ptime->trans_type == trans_type))
        {
            MESH_MODEL_PRINT_INFO("stop trans_time_insert  break\r\n");
            break;
        }
    }

    if (pcur != &trans_list_head)
    {
        /* reset transition time */
        ptime->total_time = trans_time;
        ptime->remain_num_steps = trans_time.num_steps;
        ptime->remain_step_ticks = tick_map[trans_time.step_resolution];
        MESH_MODEL_PRINT_INFO("reset transition time\r\n");
        return 1;
    }

    trans_time_remain_p premain_time = mal_malloc(sizeof(trans_time_remain_t));
    if (NULL == premain_time)
    {
        MESH_MODEL_PRINT_INFO("trans_time_insert: allocate transition time memory failed");
        return 0;
    }
    premain_time->pmodel_info = pmodel_info;
    premain_time->trans_type = trans_type;
    premain_time->total_time = trans_time;
    premain_time->remain_num_steps = trans_time.num_steps;
    premain_time->remain_step_ticks = tick_map[trans_time.step_resolution];
    premain_time->step_change = step_change;
    premain_time->node.next = NULL;
    premain_time->node.prev = NULL;
    /* sort insert */
    pcur = trans_list_head.next;
    for (; pcur != &trans_list_head; pcur = pcur->next)
    {
        trans_time_remain_p ptime = CONTAINER_OF(pcur, trans_time_remain_t, node);
        int32_t cur_time_tick = ptime->remain_num_steps * tick_map[ptime->total_time.step_resolution];
        int32_t insert_time_tick = premain_time->remain_num_steps *
                                   tick_map[ptime->total_time.step_resolution];
        if (cur_time_tick > insert_time_tick)
        {
            break;
        }
    }

    pcur->prev->next = &premain_time->node;
    premain_time->node.prev = pcur->prev;
    pcur->prev = &premain_time->node;
    premain_time->node.next = pcur;

    return 1;
}


static bool trans_time_remove_head(void)
{
    trans_list_p pfirst = trans_list_head.next;
    if (pfirst == &trans_list_head)
    {
        return 0;
    }

    trans_time_remain_p premain_time = CONTAINER_OF(pfirst, trans_time_remain_t, node);
    if (premain_time->remain_num_steps <= 0)
    {
        /* remove first */
        pfirst->prev->next = pfirst->next;
        pfirst->next->prev = pfirst->prev;
        mal_free(premain_time);
        return 1;
    }

    return 0;
}


uint8_t tick_timer_is_running = 0;

static void trans_time_timeout_handle(void *p_env)
{
    trans_list_p pcur = trans_list_head.next;
    trans_time_remain_p ptime = NULL;
    for (; pcur != &trans_list_head; pcur = pcur->next)
    {
        ptime = CONTAINER_OF(pcur, trans_time_remain_t, node);
        ptime->remain_step_ticks --;
        if (ptime->remain_step_ticks < 0)
        {
            if (ptime->remain_num_steps > 0)
            {
                ptime->remain_num_steps --;
            }
            /* reload step tick */
            ptime->remain_step_ticks = tick_map[ptime->total_time.step_resolution];
            if (NULL != ptime->step_change)
            {
                /* notify remaining step change */
                generic_transition_time_t remaining_time = {ptime->remain_num_steps, ptime->total_time.step_resolution};
                ptime->step_change(ptime->pmodel_info, ptime->trans_type, ptime->total_time, remaining_time);
            }
        }
    }

    /* remove expired transition */
    while (trans_time_remove_head());


    /* check empty */
    if (trans_time_is_empty())
    {
        MESH_MODEL_PRINT_INFO("stop transition timer");
        mesh_tb_timer_clear(&m_fnd_mesh_tick_time);
        tick_timer_is_running = 0;
    }
    else
    {
        mesh_tb_timer_set(&m_fnd_mesh_tick_time, 100);
        tick_timer_is_running = 1;
    }
}



bool generic_transition_timer_start( mm_tb_state_mdl_env_t * pmodel,
                                     uint32_t trans_type,
                                     generic_transition_time_t trans_time,
                                     generic_transition_step_change_cb step_change)
{

    MESH_MODEL_PRINT_INFO("generic_transition_timer_start\r\n");
    bool ret = 0;
    if (!tick_timer_is_running)//(mesh_tick_timer_is_running())
    {
        if (trans_time_insert(pmodel, trans_type, trans_time, step_change))
        {
            mesh_tb_timer_set(&m_fnd_mesh_tick_time, 2);
            tick_timer_is_running = 1;
            ret = 1;
        }
    }
    else
    {
        ret = trans_time_insert(pmodel, trans_type, trans_time, step_change);
    }

    return ret;
}


void generic_transition_timer_stop(mm_tb_state_mdl_env_t* pmodel_info,
                                   uint32_t trans_type)
{
    /* remove specified model timer */
    MESH_MODEL_PRINT_INFO("generic_transition_timer_stop\r\n");
    trans_list_p pcur = trans_list_head.next;
    for (; pcur != &trans_list_head; pcur = pcur->next)
    {
        trans_time_remain_p ptime = CONTAINER_OF(pcur, trans_time_remain_t, node);
        if ((ptime->pmodel_info == pmodel_info) &&
                (ptime->trans_type == trans_type))
        {
            MESH_MODEL_PRINT_INFO("remove timer\r\n");
            ptime->remain_num_steps = 0;
            break;
        }
    }

}


generic_transition_time_t generic_transition_time_get(mm_tb_state_mdl_env_t * pmodel_info,
        uint32_t trans_type)
{
    trans_list_p pcur = trans_list_head.next;
    trans_time_remain_p ptime = NULL;
    for (; pcur != &trans_list_head; pcur = pcur->next)
    {
        ptime = CONTAINER_OF(pcur, trans_time_remain_t, node);
        if ((ptime->pmodel_info == pmodel_info) &&
                (ptime->trans_type == trans_type))
        {
            break;
        }
    }

    generic_transition_time_t remaining_time = {0, 0};
    if (pcur != &trans_list_head)
    {
        remaining_time.num_steps = ptime->remain_num_steps;
        remaining_time.step_resolution = ptime->total_time.step_resolution;
    }

    return remaining_time;
}

bool generic_transition_time_init(void)
{

    if ((NULL == trans_list_head.prev) &&
            (NULL == trans_list_head.next))
    {
        trans_list_head.prev = &trans_list_head;
        trans_list_head.next = &trans_list_head;
    }

    m_fnd_mesh_tick_time.cb = trans_time_timeout_handle;
    m_fnd_mesh_tick_time.period = 100;

    return 1;
}


