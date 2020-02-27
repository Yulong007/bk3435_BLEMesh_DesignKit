/**
 ****************************************************************************************
 *
 * @file ahi_task.c
 *
 * @brief This file contains definitions related to the Application Host Interface
 *
 * Copyright (C) RivieraWaves 2009-2015
 *
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup AHI
 * @{
 ****************************************************************************************
 */


/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"     // SW configuration

#if (AHI_TL_SUPPORT)


#include "ahi.h"
#include "ahi_task.h"
#include "ke_msg.h"          // kernel message defines
#include "gapm.h"
#include "uart.h"
/*
 * DEFINES
 ****************************************************************************************
 */



/*
 * MESSAGE HANDLERS
 ****************************************************************************************
 */

static void ahi_ke_msg_tx_done(uint8_t *tx_data)
{
    // retrieve message pointer
    struct ke_msg *msg = (struct ke_msg *) (tx_data - sizeof(struct co_list_hdr));

    // free it.
    ke_msg_free(msg);
}


/**
 ****************************************************************************************
 * @brief Function called to send a message through UART.
 *
 * @param[in]  msgid   U16 message id from ke_msg.
 * @param[in] *param   Pointer to parameters of the message in ke_msg.
 * @param[in]  dest_id Destination task id.
 * @param[in]  src_id  Source task ID.
 *
 * @return             Kernel message state, must be KE_MSG_NO_FREE.
 *****************************************************************************************
 */
static int ahi_msg_send_handler (ke_msg_id_t const msgid,
                                 void *param,
                                 ke_task_id_t const dest_id,
                                 ke_task_id_t const src_id)
{
    //extract the ke_msg pointer from the param passed and push it in AHI queue
    struct ke_msg *msg = ke_param2msg(param);

    UART_PRINTF("ahi_msg_send_handler\r\n");


    // Update source and dest task number with task identifiers
    msg->src_id  = gapm_get_id_from_task(msg->src_id);
    msg->dest_id = gapm_get_id_from_task(msg->dest_id);

    UART_PRINTF("src_id = 0x%x,dest_id = 0x%x\r\n", msg->src_id, msg->dest_id);
    UART_PRINTF("send data = ");
    uint8_t *p_data = (uint8_t *) & (msg->id);
    for (int i  = 0; i < msg->param_len + AHI_KE_MSG_HDR_LEN; i ++)
    {
        UART_PRINTF("%x ", *p_data++);
    }
    UART_PRINTF("\r\n");

    // request to send the message.
    ahi_send_msg(AHI_KE_MSG_TYPE, msg->param_len + AHI_KE_MSG_HDR_LEN, (uint8_t *) & (msg->id),  &ahi_ke_msg_tx_done);

    //return NO_FREE always since ahi_eif_write handles the freeing
    return KE_MSG_NO_FREE;
}

/*
 * TASK DESCRIPTOR DEFINITIONS
 ****************************************************************************************
 */

/// Specifies the message handlers that are common to all states.
const struct ke_msg_handler ahi_default_state[] =
{

    /** Default handler for AHI TX message, this entry has to be put first as table is
        parsed from end to start by Kernel */
    {KE_MSG_DEFAULT_HANDLER,  (ke_msg_func_t)ahi_msg_send_handler},
};

/// Specifies the message handlers that are common to all states.
const struct ke_state_handler ahi_default_handler = KE_STATE_HANDLER(ahi_default_state);

/// Defines the placeholder for the states of all the task instances.
ke_state_t ahi_state[AHI_IDX_MAX];


#endif //AHI_TL_SUPPORT

/// @} AHI
