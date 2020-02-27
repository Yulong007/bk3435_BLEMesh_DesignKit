

/**
 ****************************************************************************************
 * @file m_fnd_Scenes.c
 *
 * @brief Mesh Scene Server Model
 *
 * Copyright (C) Beken 2018-2019
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup M_FND_SCENES
 * @{
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
#include "m_fnd_Scenes.h"
#include "m_fnd_generic_transition_time.h"
/*
 * DEFINES
 ****************************************************************************************
 */
/// Scene Server SIG Model ID
#define M_FND_SCENES_MODEL_ID                        (0x1203)



/*
 * ENUMERATIONS
 ****************************************************************************************
 */



/*
 * STRUCTURES
 ****************************************************************************************
 */
typedef struct
{
    uint8_t tid;
    uint16_t target_scenes;

} m_fnd_scenes_info_t, *m_fnd_scenes_info_p;

/*
 * PRIVATE VARIABLES
 ****************************************************************************************
 */


/// Foundation model layer environment

__STATIC m_fnd_model_env_t *p_m_fnd_env;



/*
 * STATIC FUNCTION DEFINITIONS
 ****************************************************************************************
 */

__STATIC void m_fnd_scenes_process_next(void);


/*
 * STATIC FUNCTIONS
 ****************************************************************************************
 */


/**
 ****************************************************************************************
 * @brief Allocate a buffer.
 *
 * @param[out] pp_buf
 * @param[in]  data_len
 *
 * @return M_ERR_NO_ERR if buffer has been successfully allocated, else another error code.
 ****************************************************************************************
 */
__STATIC uint16_t m_fnd_scenes_buf_alloc(mesh_tb_buf_t **pp_buf, uint16_t data_len)
{
    // Try to allocate a buffer and return the status
    return (mesh_tb_buf_alloc(pp_buf, M_FND_BUF_HEAD_LEN, data_len, 0));
}


/**
 ****************************************************************************************
 * @brief Push a buffer for transmission (response).
 *
 * @param[in] p_buf     Pointer to the buffer containing the message to send.
 * @param[in] opcode    Operation code
 ****************************************************************************************
 */
__STATIC void m_fnd_scenes_send(mesh_tb_buf_t *p_buf, uint16_t opcode)
{
    // Retrieve buffer containing the message for which a response is sent
    mesh_tb_buf_t *p_buf_req = (mesh_tb_buf_t *)co_list_pick(&p_m_fnd_env->process_queue);
    // Get buffer environment
    m_lay_buf_env_t *p_env = (m_lay_buf_env_t *)&p_buf_req->env[0];
    // Use request source address as destination address
    uint16_t dst = p_env->src;

    MESH_MODEL_PRINT_INFO("%s\r\n", __func__);
    // Send the provided message
    m_api_model_rsp_send(p_m_fnd_env->model_lid, (uint32_t)opcode, 0,
                         p_buf, p_env->app_lid, dst, false, false);
}


/**
 ****************************************************************************************
 * @brief Push a buffer for transmission (publication).
 *
 * @param[in] p_buf     Pointer to the buffer containing the message to send.
 * @param[in] opcode    Operation code
 ****************************************************************************************
 */
void m_fnd_scenes_publish(uint8_t stat, uint16_t scene)
{

    MESH_MODEL_PRINT_INFO("[func]%s\r\n", __func__);
    mesh_tb_buf_t *p_buf;
    uint16_t status;
    if (m_fnd_scenes_buf_alloc(&p_buf, 3) == MESH_ERR_NO_ERROR)
    {
        // Get pointer to data
        uint8_t *p_data = MESH_TB_BUF_DATA(p_buf);
        p_data[0] = stat;
        memcpy(&p_data[1], (uint8_t *)&scene, 2);
        MESH_MODEL_PRINT_INFO("buf[0] = 0x%x,buf[1] = 0x%x,buf[2] = 0x%x\r\n", p_data[0], p_data[1], p_data[2]);
        // Send the provided message
        status = m_api_model_publish(p_m_fnd_env->model_lid, M_FND_SCENES_OPCODE_STATUS, 0, p_buf, false);
        if (status != MESH_ERR_NO_ERROR)
        {
            m_api_buf_release(p_buf);
        }
    }
    else
    {
        MESH_MODEL_PRINT_INFO("m_fnd_scenes_publish m_fnd_genonoffs_buf_alloc fail\r\n");
    }

}


static uint16_t get_present_scene(const m_fnd_model_env_p pmodel_info)
{
    MESH_MODEL_PRINT_INFO("[func]%s\r\n", __func__);
    m_fnd_scenes_get_t get_data = {0};

    if (NULL != pmodel_info->model_data_cb)
    {
        pmodel_info->model_data_cb(pmodel_info, M_FND_SCENE_SERVER_GET_PRESENT, &get_data);
    }
    return get_data.scene_number;
}



static void m_fnd_scene_trans_step_change(const m_fnd_model_env_p pmodel_info, uint32_t type,
        generic_transition_time_t total_time,
        generic_transition_time_t remaining_time)
{
    MESH_MODEL_PRINT_INFO("[func]%s\r\n", __func__);
    m_fnd_scene_server_set_t set_data;
    m_fnd_scenes_info_t *ptarget = pmodel_info->pargs;
    set_data.scene_number = ptarget->target_scenes;
    set_data.total_time = total_time;
    set_data.remaining_time = remaining_time;
    if (NULL != pmodel_info->model_data_cb)
    {
        pmodel_info->model_data_cb(pmodel_info, M_FND_SCENE_SERVER_RECALL, &set_data);
    }
    if (0 == remaining_time.num_steps)
    {
        uint16_t present_scene = get_present_scene(pmodel_info);
        m_fnd_scenes_publish(0, present_scene);
    }
}




/**
 ****************************************************************************************
 * @brief Prepare and send a scenes Model Status message.
 *
 * @param[in] status        Handling status for the command that triggered sending of this message.
 * @param[in] p_msg         Pointer to the genonoffs_onoff_status message.
 * @param[in] vendor        True if model identifier is a vendor model identifier.
 ****************************************************************************************
 */

__STATIC void m_fnd_scenes_send_model_status(m_fnd_scenes_status_t status, uint8_t optional)
{
    MESH_MODEL_PRINT_INFO("%s\r\n", __func__);
    // Pointer to the buffer that will contain the message
    mesh_tb_buf_t *p_buf_status;

    uint16_t data_length;
    // Data length
    if (optional)
    {
        data_length = sizeof(m_fnd_scenes_status_t);
    }
    else
    {
        data_length = offsetof(m_fnd_scenes_status_t, target_scene); // 1;
    }

    MESH_MODEL_PRINT_INFO("data_length = 0x%x\r\n", data_length);
    m_fnd_scenes_status_t *p_status = &status;

    if (m_fnd_scenes_buf_alloc(&p_buf_status, data_length) == MESH_ERR_NO_ERROR)
    {
        // Get pointer to data
        uint8_t *p_data = MESH_TB_BUF_DATA(p_buf_status);

        memcpy(p_data, (uint8_t *)&status, data_length);

        MESH_MODEL_PRINT_INFO("current_scene = 0x%x,target_scene = 0x%x\r\n", p_status->current_scene, p_status->target_scene);

        // Send the message
        m_fnd_scenes_send(p_buf_status, M_FND_SCENES_OPCODE_STATUS); //
    }
}

/**
 ****************************************************************************************
 * @brief Prepare and send a scenes Model register Status message.
 *
 * @param[in] status        Handling status for the command that triggered sending of this message.
 * @param[in] p_msg         Pointer to the genonoffs_onoff_status message.
 * @param[in] vendor        True if model identifier is a vendor model identifier.
 ****************************************************************************************
 */

__STATIC void m_fnd_scenes_send_model_register_status(m_fnd_scenes_register_status_t status, uint8_t scene_cnt)
{
    MESH_MODEL_PRINT_INFO("%s\r\n", __func__);
    // Pointer to the buffer that will contain the message
    mesh_tb_buf_t *p_buf_status;
    // Data length
    uint16_t data_length = M_FND_SCENES_MODEL_REGISTER_STATUS_MIN_LEN;

    data_length += (scene_cnt * 2);
    MESH_MODEL_PRINT_INFO("data_length = %d\r\n", data_length);
    m_fnd_scenes_register_status_t *p_status = &status;

    if (m_fnd_scenes_buf_alloc(&p_buf_status, data_length) == MESH_ERR_NO_ERROR)
    {
        // Get pointer to data
        uint8_t *p_data = MESH_TB_BUF_DATA(p_buf_status);
        // Write status
        memcpy(p_data, (uint8_t *)&status, sizeof(m_fnd_scenes_register_status_t));

        MESH_MODEL_PRINT_INFO("status_code = 0x%x,current_scene = 0x%x\r\n", p_status->status_code, p_status->current_scene);
        for (int i = 0; i < scene_cnt; i++)
        {
            MESH_MODEL_PRINT_INFO("scene[%d] = 0x%x\r\n", i, p_status->scenes[i]);
        }
        // Send the message
        m_fnd_scenes_send(p_buf_status, M_FND_SCENES_2B_OPCODE(M_FND_SCENES_OPCODE_REGISTER_STATUS));
    }

}



/**
 ****************************************************************************************
 * @brief Handle Scene Get ,recall or recall unack message
 * Unacknowledged message received from provisioner.
 *
 * @param[in] p_buf     Pointer to the buffer containing the message.
 * @param[in] opcode    Operation code
 ****************************************************************************************
 */


__STATIC void m_fnd_scenes_handler_recall(mesh_tb_buf_t *p_buf, uint8_t opcode)
{
    MESH_MODEL_PRINT_INFO("%s,opcode = %x\r\n", __func__, opcode);
    switch (opcode)
    {
        case M_FND_SCENES_OPCODE_GET:
        {

            m_fnd_scenes_status_t status;

            m_fnd_scenes_info_p pscenes_info = p_m_fnd_env->pargs;
            m_fnd_scenes_get_t get_data = {0};
            if (NULL != p_m_fnd_env->model_data_cb)
            {
                p_m_fnd_env->model_data_cb(p_m_fnd_env, M_FND_SCENE_SERVER_GET_PRESENT, &get_data);
            }
            status.status_code = 0;
            status.current_scene = get_data.scene_number;
            status.target_scene = pscenes_info->target_scenes;
            generic_transition_time_t remaining_time = {0};//generic_transition_time_get(p_m_fnd_env,GENERIC_TRANSITION_TYPE_SCENE);

            status.rema_time = remaining_time;
            MESH_MODEL_PRINT_INFO("current_scene = 0x%x,target_scene = 0x%x\r\n", status.current_scene, status.target_scene);

            m_fnd_scenes_send_model_status(status, (M_FND_GENERIC_TRANSITION_NUM_STEPS_IMMEDIATE == remaining_time.num_steps) ? 0 : 1);

        } break;
        case M_FND_SCENES_OPCODE_RECALL:
        case M_FND_SCENES_OPCODE_RECALL_UNACK:
        {
            uint16_t scene_number_before_recall = 0;
            uint16_t scene_number_after_recall = 0;

            m_fnd_scenes_recall_t  *pmsg = (m_fnd_scenes_recall_t *) MESH_TB_BUF_DATA(p_buf);
            generic_transition_time_t trans_time = {M_FND_GENERIC_TRANSITION_NUM_STEPS_IMMEDIATE};
            m_fnd_scenes_info_p pscenes_info = p_m_fnd_env->pargs;
            MESH_MODEL_PRINT_INFO("M_FND_SCENES_OPCODE_RECALL/UNACK\r\n");
            if (p_buf->data_len == sizeof(m_fnd_scenes_recall_t))
            {
                trans_time = pmsg->trans_time;
                MESH_MODEL_PRINT_INFO("-------------full------------------");
                MESH_MODEL_PRINT_INFO("Scene_Number 	 = 0x%02x,", pmsg->scene_number);
                MESH_MODEL_PRINT_INFO("TID	= 0x%02x,", pmsg->tid);
                MESH_MODEL_PRINT_INFO("Transition_Time = 0x%02x,", pmsg->trans_time.num_steps);
                MESH_MODEL_PRINT_INFO("Delay 	 = 0x%02x\r\n", pmsg->delay);
            }
            else if (p_buf->data_len == offsetof(m_fnd_scenes_recall_t, trans_time))
            {
                m_fnd_scene_server_get_default_transition_time_t get_time = {0};
                if (NULL != p_m_fnd_env->model_data_cb)
                {
                    p_m_fnd_env->model_data_cb(p_m_fnd_env, M_FND_SCENE_SERVER_GET_DEFAULT_TRANSITION_TIME, &get_time);
                }
                trans_time = get_time.trans_time;
                MESH_MODEL_PRINT_INFO("-------------part------------------");
                MESH_MODEL_PRINT_INFO("Scene_Number 	 = 0x%02x,", pmsg->scene_number);
                MESH_MODEL_PRINT_INFO("TID	= 0x%02x\r\n", pmsg->tid);
                MESH_MODEL_PRINT_INFO("Transition_Time = 0x%02x,", pmsg->trans_time.num_steps);

            }
            if (M_IS_SCENES_VALID(pmsg->scene_number))
            {
                pscenes_info->target_scenes = pmsg->scene_number;
                pscenes_info->tid = pmsg->tid;

                /* get scene before set */
                scene_number_before_recall = get_present_scene(p_m_fnd_env);

                m_fnd_scene_server_set_t trans_set_data;
                trans_set_data.scene_number = pmsg->scene_number;
                trans_set_data.total_time = trans_time;
                trans_set_data.remaining_time = trans_time;

                if (NULL != p_m_fnd_env->model_data_cb)
                {
                    p_m_fnd_env->model_data_cb(p_m_fnd_env, M_FND_SCENE_SERVER_RECALL, &trans_set_data);
                }

                if (M_FND_GENERIC_TRANSITION_NUM_STEPS_IMMEDIATE != trans_time.num_steps)
                {
                    // generic_transition_timer_start(p_m_fnd_env, GENERIC_TRANSITION_TYPE_SCENE, trans_time, m_fnd_scene_trans_step_change);
                }


                scene_number_after_recall = get_present_scene(p_m_fnd_env);

                MESH_MODEL_PRINT_INFO("scene_number_before_recall = 0x%x,scene_number_after_recall = 0x%x\r\n", scene_number_before_recall, scene_number_after_recall);

                if (scene_number_before_recall != scene_number_after_recall)
                {
                    m_fnd_scenes_publish(0, scene_number_after_recall);
                }
                if (opcode == M_FND_SCENES_OPCODE_RECALL)
                {
                    m_fnd_scenes_status_t status;

                    status.status_code = 0;
                    status.current_scene = scene_number_after_recall;
                    status.target_scene = pscenes_info->target_scenes;
                    generic_transition_time_t remaining_time = {M_FND_GENERIC_TRANSITION_NUM_STEPS_IMMEDIATE};//generic_transition_time_get(p_m_fnd_env,GENERIC_TRANSITION_TYPE_SCENE);
                    status.rema_time = remaining_time;
                    MESH_MODEL_PRINT_INFO("current_scene = 0x%x,target_scene = 0x%x\r\n", status.current_scene, status.target_scene);

                    m_fnd_scenes_send_model_status(status, (M_FND_GENERIC_TRANSITION_NUM_STEPS_IMMEDIATE == trans_time.num_steps) ? 0 : 1);
                }
            }
        } break;

    }

    // Process next received message
    m_fnd_scenes_process_next();

}

/**
 ****************************************************************************************
 * @brief Handle Scene Register Get message
 * Unacknowledged message received from provisioner.
 *
 * @param[in] p_buf     Pointer to the buffer containing the message.
 * @param[in] opcode    Operation code
 ****************************************************************************************
 */


__STATIC void m_fnd_scenes_handler_register(mesh_tb_buf_t *p_buf, uint8_t opcode)
{
    MESH_MODEL_PRINT_INFO("%s,opcode = %x\r\n", __func__, opcode);


    if (opcode == M_FND_SCENES_OPCODE_REGISTER_GET)
    {
        m_fnd_scenes_register_status_t status;

        //m_fnd_scenes_info_p pscenes_info = p_m_fnd_env->pargs;
        m_fnd_scene_server_get_register_t get_data = {0};
        if (NULL != p_m_fnd_env->model_data_cb)
        {
            p_m_fnd_env->model_data_cb(p_m_fnd_env, M_FND_SCENE_SERVER_GET_REGISTER, &get_data);
        }
        status.status_code = 0;
        status.current_scene = get_present_scene(p_m_fnd_env);
        uint8_t j = 0;
        for (int i = 0 ; i < M_FND_SCENES_STORE_MAX; i ++)
        {
            if (get_data.scenes[i])
            {
                status.scenes[j++] = get_data.scenes[i];
            }
        }

        MESH_MODEL_PRINT_INFO("status_code = 0x%x,current_scene = 0x%x\r\n", status.status_code, status.current_scene);

        m_fnd_scenes_send_model_register_status(status, j);

    }
    // Process next received message
    m_fnd_scenes_process_next();

}


/**
 ****************************************************************************************
 * @brief Call the appropriate handler for a given buffer.
 *
 * @param[in] p_buf     Buffer to handle.
 ****************************************************************************************
 */
__STATIC void m_fnd_scenes_process(m_api_buf_t *p_buf)
{

    MESH_MODEL_PRINT_INFO("%s\r\n", __func__);
    // Get buffer environment
    m_lay_buf_env_t *p_env = (m_lay_buf_env_t *)(&((mesh_tb_buf_t *)p_buf)->env[0]);

    // Check that opcode is a 2-byte opcode
    if (MESH_IS_2_OCT_OPCODE(p_env->u5.opcode))
    {
        // Get only second byte of the opcode
        uint8_t opcode_2b = (uint8_t)(p_env->u5.opcode >> 8);

        switch (opcode_2b)
        {
            case (M_FND_SCENES_OPCODE_GET)          :
            case (M_FND_SCENES_OPCODE_RECALL)       :
            case (M_FND_SCENES_OPCODE_RECALL_UNACK) : m_fnd_scenes_handler_recall(p_buf, opcode_2b);  break;
            case (M_FND_SCENES_OPCODE_REGISTER_GET) : m_fnd_scenes_handler_register(p_buf, opcode_2b);  break;
            default                                 : m_fnd_scenes_process_next();                       break;
        }
    }
}




/**
 ****************************************************************************************
 * @brief Pop first buffer from the processing queue, release it and start processing of
 * next one in the processing queue.
 ****************************************************************************************
 */
__STATIC void m_fnd_scenes_process_next(void)
{

    MESH_MODEL_PRINT_INFO("%s\r\n", __func__);
    // Remove first buffer in the process queue
    mesh_tb_buf_t *p_buf = (mesh_tb_buf_t *)co_list_pop_front(&p_m_fnd_env->process_queue);

    // Release the buffer
    mesh_tb_buf_release(p_buf);

    // Process next buffer
    p_buf = (mesh_tb_buf_t *)co_list_pick(&p_m_fnd_env->process_queue);

    if (p_buf != NULL)
    {
        m_fnd_scenes_process(p_buf);
    }
}

/**
 ****************************************************************************************
 * @brief Callback function called upon reception of a message.
 *
 * @param[in] model_lid       Model local index.
 * @param[in] opcode          Operation code.
 * @param[in] p_api_buf       Pointer to the buffer containing the message PDU.
 * @param[in] app_key_lid     Application key local index (Required for a response).
 * @param[in] src             Source address of the message (Required for a response).
 * @param[in] rssi            Measured RSSI level for the received PDU.
 * @param[in] not_relayed     True if message have been received by an immediate peer;
 *                            False, it can have been relayed.
 ****************************************************************************************
 */
__STATIC void m_fnd_scenes_cb_rx(m_lid_t model_lid, uint32_t opcode, m_api_buf_t *p_api_buf, m_lid_t app_key_lid,
                                 uint16_t src, int8_t rssi, bool not_relayed)
{

    MESH_MODEL_PRINT_INFO("%s\r\n", __func__);
    // Data length
    uint16_t data_len = ((mesh_tb_buf_t *)p_api_buf)->data_len;
    // Allocate a buffer
    mesh_tb_buf_t *p_buf;

    mesh_tb_buf_alloc(&p_buf, 0, data_len, 0);

    // Copy content of the received buffer in the allocated one
    mesh_tb_buf_copy((mesh_tb_buf_t *)p_api_buf, p_buf, data_len, true);

    // Insert the buffer in the process queue
    co_list_push_back(&p_m_fnd_env->process_queue, &p_buf->hdr);

    // Check if buffer can be processed now or if another buffer is currently processed
    if ((mesh_tb_buf_t *)co_list_pick(&p_m_fnd_env->process_queue) == p_buf)
    {
        m_fnd_scenes_process(p_buf);
    }
}
/**
 ****************************************************************************************
 * @brief Callback function called upon reception of a message in order to know if the model
 * supports the received operation code.
 *
 * @param[in] model_lid     Model local index
 * @param[in] opcode        Operation code to check
 ****************************************************************************************
 */
__STATIC void m_fnd_scenes_cb_opcode_check(m_lid_t model_lid, uint32_t opcode)
{

    // Status
    uint16_t status = MESH_ERR_NOT_SUPPORTED;

    // Check that opcode is a 2-byte opcode
    if (MESH_IS_2_OCT_OPCODE(opcode) && M_IS_SCENES_OPCODE(opcode))
    {
        // Get only second byte of the opcode
        uint8_t opcode_2b = (uint8_t)(opcode >> 8);

        switch (opcode_2b)
        {
            case M_FND_SCENES_OPCODE_GET:
            case M_FND_SCENES_OPCODE_RECALL:
            case M_FND_SCENES_OPCODE_RECALL_UNACK:
            case M_FND_SCENES_OPCODE_STATUS:
            case M_FND_SCENES_OPCODE_REGISTER_GET:
            case M_FND_SCENES_OPCODE_REGISTER_STATUS:
            {
                status = MESH_ERR_NO_ERROR;
            } break;

            default:break;
        }

    }
    MESH_MODEL_PRINT_INFO("%s,status = 0x%x,opcode = 0x%x\r\n", __func__, status, opcode);

    // Indicate if provided operation code is supported or not
    m_api_model_opcode_status(model_lid, opcode, status);
}


/**
 ****************************************************************************************
 * @brief Callback function called once buffer pushed by a model has been transmitted or if
 * an issue has been raised during processing.
 *
 * @param[in] model_lid     Model local index.
 * @param[in] tx_hdl        Handle value configured by model when message has been pushed for transmission.
 * @param[in] p_api_buf     Pointer to the buffer containing the transmitted PDU.
 * @param[in] status        Transmission status.
 ****************************************************************************************
 */
__STATIC void m_fnd_scenes_cb_sent(m_lid_t model_lid, uint8_t tx_hdl, m_api_buf_t *p_api_buf, uint16_t status)
{
    MESH_MODEL_PRINT_INFO("%s\r\n", __func__);

    // Release the buffer
    mesh_tb_buf_release((mesh_tb_buf_t *)p_api_buf);
}



/**
 ****************************************************************************************
 * @brief Callback function called upon reception of new publication parameters in order to inform
 * the model about a new publish period.
 *
 * @param[in] model_lid     Model local index
 * @param[in] period_ms     Publish period in milliseconds
 ****************************************************************************************
 */
__STATIC void m_fnd_scenes_cb_publish_period(m_lid_t model_lid, uint16_t addr, uint32_t period_ms)
{
    MESH_MODEL_PRINT_INFO("%s\r\n", __func__);
    // Inform the application about the new publication period
    // m_api_send_fault_period_ind(period_ms, period_ms >> p_m_fnd_hlths_env->fast_period_divisor);
}


/*
 * CALLBACK FUNCTIONS
 ****************************************************************************************
 */

/// Health Client Model callback functions
const m_api_model_cb_t m_fnd_scenes_cb =
{
    .cb_rx             = m_fnd_scenes_cb_rx,
    .cb_sent           = m_fnd_scenes_cb_sent,
    .cb_opcode_check   = m_fnd_scenes_cb_opcode_check,
    .cb_publish_param = m_fnd_scenes_cb_publish_period,
};



/*
 * GLOBAL FUNCTIONS
 ****************************************************************************************
 */

uint16_t m_fnd_scenes_init(m_fnd_data_cb_p cb)
{
    p_m_fnd_env = mal_malloc(sizeof(m_fnd_model_env_t));

    p_m_fnd_env->model_id = M_FND_SCENES_MODEL_ID;

    p_m_fnd_env->pargs = mal_malloc(sizeof(m_fnd_scenes_info_t));

    MESH_MODEL_PRINT_INFO("size m_fnd_scenes_info_t = %d\r\n", sizeof(m_fnd_scenes_info_t));
    if (NULL == p_m_fnd_env->pargs)
    {
        MESH_MODEL_PRINT_INFO("m_fnd_scenes_init: fail to allocate memory for the new model extension data!\r\n");
        return 0;
    }
    memset(p_m_fnd_env->pargs, 0, sizeof(m_fnd_scenes_info_t));
    p_m_fnd_env->model_data_cb = cb;

    MESH_MODEL_PRINT_INFO("p_m_fnd_env->model_data_cb addr = 0x%x\r\n", p_m_fnd_env->model_data_cb);
    if (NULL == p_m_fnd_env->model_data_cb)
    {
        MESH_MODEL_PRINT_INFO("m_fnd_scenes_init: missing model data process callback!");
        return 0;
    }

    // Register the model
    m_api_register_model(p_m_fnd_env->model_id, 0, false, &m_fnd_scenes_cb,
                         &p_m_fnd_env->model_lid);

    p_m_fnd_env->djob.cb = m_fnd_scenes_process;

    // Return environment size
    return (sizeof(m_fnd_model_env_t));
}


uint16_t m_fnd_scenes_get_env_size(const m_cfg_t* p_cfg)
{
    // Return environment size
    return (sizeof(m_fnd_model_env_t));
}




/// @} end of group
