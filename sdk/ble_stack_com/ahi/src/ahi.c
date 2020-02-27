/**
 ****************************************************************************************
 *
 * @file ahi.c
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
#include "ke_event.h"
#include "ke_task.h"
#include "ke_mem.h"
#include "co_bt.h"           // BT standard definitions
#include "co_list.h"
#include "h4tl.h"            // H4 Transport Layer

#if defined(CFG_AUDIO_AOAHI)
#include "aoahi.h"
#endif // defined(CFG_AUDIO_AOAHI)

/*
 * DEFINES
 ****************************************************************************************
 */

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */
///Structure for application system rx interface packet
struct ahi_rx_data
{
    ///Message id
    ke_msg_id_t  id;
    ///Destination task identifier for KE
    ke_task_id_t dest_id;
    ///Source task identifier for KE
    ke_task_id_t src_id;
    ///Message parameter length
    uint16_t     param_len;

    uint8_t data[__ARRAY_EMPTY];
};

/// Data structure that describe packet to send
struct ahi_tx_data
{
    /// list element
    struct co_list_hdr hdr;
    /// Message data pointer to send
    uint8_t           *data;
    /// TX callback to be call when TX is over
    void               (*tx_callback)(uint8_t *);
    /// Data length to send.
    uint16_t           len;
    /// Message type
    uint8_t            msg_type;
};


///AHI Environment context structure
struct ahi_env_tag
{
    /// list of TX buffer in pending queue
    struct co_list      tx_queue;

    /// message wich is currently TX.
    struct ahi_tx_data *tx_ptr;
};


/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

/// AHI environment context
struct ahi_env_tag ahi_env;

/// AHI task descriptor
static const struct ke_task_desc TASK_DESC_AHI = {NULL, &ahi_default_handler, ahi_state, AHI_STATE_MAX, AHI_IDX_MAX};

/*
 * LOCAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Actions after external interface TX.
 *
 * Analyzes the status value and sets the AHI environment state to TX_DONE/ERR
 * accordingly. This allows the higher function calling write to have feedback
 * and decide the following action (repeat/abort tx in case of error, continue otherwise).
 *****************************************************************************************
 */
static void ahi_tx_done(void)
{
    // Defer the freeing of resources to ensure that it is done in background
    ke_event_set(KE_EVENT_AHI_TX_DONE);
}

/**
 ****************************************************************************************
 * @brief Send Messages over H4TL
 *
 * @param[in] tx_data contains information of buffer to send
 ****************************************************************************************
 */
static void ahi_h4tl_send(struct ahi_tx_data *tx_data)
{
    ASSERT_ERR(tx_data != NULL);
    ahi_env.tx_ptr = tx_data;
    // Set AHI task busy
    ke_state_set(TASK_AHI, AHI_TX_ONGOING);

    // Send data over the AHI EIF Transport
    h4tl_write(tx_data->msg_type, tx_data->data, tx_data->len, &ahi_tx_done);
}

/**
 ****************************************************************************************
 * @brief Function called after sending message through external interface, to free ke_msg space and
 * push the next message for transmission if any.
 *
 * The message is popped from the tx queue kept in ahi_env and freed using ke_msg_free.
 *****************************************************************************************
 */
static void ahi_tx_done_evt_handler(void)
{
    struct ahi_tx_data *tx_data = ahi_env.tx_ptr;
    // Clear the event
    ke_event_clear(KE_EVENT_AHI_TX_DONE);

    // try to call the tx end callback
    if (tx_data->tx_callback != NULL)
    {
        // message free is managed by another handler
        ahi_env.tx_ptr->tx_callback(tx_data->data);
    }

    // Free the temporary tx pointer
    ke_free(ahi_env.tx_ptr);

    // check if there is something in TX queue
    if (! co_list_is_empty(&ahi_env.tx_queue))
    {
        //extract the ke_msg pointer from the top of the queue
        struct ahi_tx_data *tx_data = (struct ahi_tx_data *) co_list_pop_front(&ahi_env.tx_queue);

        // send the message on top of queue using H4TL
        ahi_h4tl_send(tx_data);
    }
    else
    {
        // Set AHI task to TX IDLE state
        ke_state_set(TASK_AHI, AHI_TX_IDLE);
    }
}


/*
 * EXPORTED FUNCTION DEFINITIONS
 ****************************************************************************************
 */

void ahi_init(void)
{
    // Create AHI Task
    ke_task_create(TASK_AHI, &TASK_DESC_AHI);

    // Register AHI TX DONE kernel event
    ke_event_callback_set(KE_EVENT_AHI_TX_DONE, &ahi_tx_done_evt_handler);

    // Initialize AHI task to idle state
    ke_state_set(TASK_AHI, AHI_TX_IDLE);

    // Initialize TX queue
    co_list_init(&ahi_env.tx_queue);
}


void ahi_send_msg(uint8_t msg_type, uint16_t len, uint8_t *data, void (*tx_callback)(uint8_t *))
{
    switch (msg_type)
    {
#if defined(AOAHI_TL_SUPPORT)
        case AOAHI_AUDIO_MSG_TYPE:
#endif // defined(AOAHI_TL_SUPPORT)
        case AHI_KE_MSG_TYPE:
        {
            struct ahi_tx_data *tx_data = (struct ahi_tx_data *) ke_malloc(sizeof(struct ahi_tx_data), KE_MEM_KE_MSG);
            tx_data->len         = len;
            tx_data->data        = data;
            tx_data->msg_type    = msg_type;
            tx_data->tx_callback = tx_callback;

            // Check if there is no transmission ongoing
            if (ke_state_get(TASK_AHI) == AHI_TX_IDLE)
            {
                // request to send the message to H4 TL.
                ahi_h4tl_send(tx_data);
            }
            else
            {
                // put message at end of queue
                co_list_push_back(&ahi_env.tx_queue, &(tx_data->hdr));
            }
        } break;
        // ensure that only supported messages are sent over AHI interface
        default:
        {
            ASSERT_INFO(0, msg_type, len);
        } break;
    }

}

#endif //AHI_TL_SUPPORT

/// @} AHI
