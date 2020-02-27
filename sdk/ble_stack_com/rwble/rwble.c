/**
 ****************************************************************************************
 *
 * @file rwble.c
 *
 * @brief Entry points the BLE software
 *
 * Copyright (C) RivieraWaves 2009-2015
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup ROOT
 * @{
 ****************************************************************************************
 */


/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include <string.h>
#include "co_version.h"
#include "rwble.h"
#include "icu.h"
#if HCI_PRESENT
#include "hci.h"
#endif //HCI_PRESENT
#include "ke_event.h"
#include "ke_timer.h"
#include "em_buf.h"
#include "ea.h"
#include "lld.h"
#include "llc.h"
#include "llm.h"
#include "dbg.h"
#include "ea.h"
#include "reg_blecore.h"
#include "user_config.h"
#include "wdt.h"
#include "lld_adv_test.h"
#include "nvds.h"         // NVDS definitions
#if ((BLE_CENTRAL || BLE_PERIPHERAL) && (BLE_AUDIO))
#include "audio.h"
#if defined (PLF_LE_AUDIO_PATH)
#include "leaudio.h"
#endif
#endif /*((BLE_CENTRAL || BLE_PERIPHERAL) && (BLE_AUDIO))*/



/*
 * LOCAL FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Initializes the diagnostic port.
 ****************************************************************************************
 */
static void rwble_diagport_init(void)
{

}


/*
 * EXPORTED FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */

void rwble_init(void)
{
    // Initialize buffers
    em_buf_init();
    // Initialize the Link Layer Driver
    lld_init(false);

    lld_adv_test_init(false);
#if (BLE_CENTRAL || BLE_PERIPHERAL)
    // Initialize the Link Layer Controller
    llc_init();
#endif /* #if (BLE_CENTRAL || BLE_PERIPHERAL) */
    // Initialize the Link Layer Manager
    llm_init(false);
    // Initialize diagport - for test and debug only
    rwble_diagport_init();

#if BLE_HOST_PRESENT
    // Signal the completion of the boot process to the host layers
    llm_ble_ready();
#endif //BLE_HOST_PRESENT

}

void rwble_reset(void)
{
    uint32_t seed;

    // Disable interrupts before reset procedure is completed
    GLOBAL_INT_DISABLE();
    // Sample the base time count
    ble_samp_setf(1);
    while (ble_samp_getf());
    seed = ble_basetimecnt_get();
    seed += ble_finetimecnt_get();
    seed += ble_bdaddrl_get();
    // Init the random seed
    co_random_init(seed);

    // Reset the BLE core
    lld_core_reset();

    // init the link layer driver
    lld_init(true);
    lld_adv_test_init(true);
#if (BLE_CENTRAL || BLE_PERIPHERAL)
    // Reset the link layer controller
    llc_reset();
#endif /* #if (BLE_CENTRAL || BLE_PERIPHERAL) */

    // init the link layer manager
    llm_init(true);

    // Initialize Descriptors and buffers
    em_buf_init();

#if ((BLE_CENTRAL || BLE_PERIPHERAL) && (BLE_AUDIO))
    audio_init(true);
#endif /* #if ((BLE_CENTRAL || BLE_PERIPHERAL) && (BLE_AUDIO)) */

    // Restore interrupts after reset procedure is completed
    GLOBAL_INT_RESTORE();
}


bool rwble_sleep_check(void)
{
    return (co_list_is_empty(&lld_evt_env.elt_prog));
}


bool rwble_activity_ongoing_check(void)
{
    bool status = true;
    if (co_list_is_empty(&lld_evt_env.elt_prog) && co_list_is_empty(&lld_evt_env.elt_wait)  && co_list_is_empty(&lld_evt_env.elt_deferred))
    {
        status = false;
    }
    return (status);
}


void rwble_version(uint8_t *fw_version, uint8_t *hw_version)
{
    // FW version
    *(fw_version + 3) = RWBT_SW_VERSION_MAJOR;
    *(fw_version + 2) = RWBT_SW_VERSION_MINOR;
    *(fw_version + 1) = RWBT_SW_VERSION_BUILD;
    *(fw_version)   = RWBT_SW_VERSION_SUB_BUILD;

    // HW version
    *(hw_version + 3) = ble_typ_getf();
    *(hw_version + 2) = ble_rel_getf();
    *(hw_version + 1) = ble_upg_getf();
    *(hw_version)   = ble_build_getf();
}

__BLEIRQ void rwble_isr(void)
{
    // Loop until no more interrupts have to be handled
    while (1)
    {
        // Check BLE interrupt status and call the appropriate handlers
        uint32_t irq_stat = ble_intstat_get();

        if (irq_stat == 0)
        {
            break;
        }
#if SYSTEM_SLEEP
#if !BT_DUAL_MODE
        // End of sleep interrupt
        if (irq_stat & BLE_SLPINTSTAT_BIT)
        {
            DBG_SWDIAG(BLE_ISR, SLPINT, 1);

            // Clear the interrupt
            ble_intack_clear(BLE_SLPINTACK_BIT);
            // Handle wake-up
            rwip_wakeup();

            DBG_SWDIAG(BLE_ISR, SLPINT, 0);
        }

        // Slot interrupt
        if (irq_stat & BLE_CSCNTINTSTAT_BIT)
        {
            DBG_SWDIAG(BLE_ISR, CSCNTINT, 1);

            // Clear the interrupt
            ble_intack_clear(BLE_CSCNTINTACK_BIT);

            // Handle end of wake-up
            rwip_wakeup_end();

            // Try to schedule immediately
            ea_finetimer_isr();

            DBG_SWDIAG(BLE_ISR, CSCNTINT, 0);
        }
#endif //!BT_DUAL_MODE
#endif //SYSTEM_SLEEP

        // Event target interrupt
        if (irq_stat & BLE_FINETGTIMINTSTAT_BIT)
        {
            DBG_SWDIAG(BLE_ISR, FINETGTIMINT, 1);

            // Clear the interrupt
            ble_intack_clear(BLE_FINETGTIMINTACK_BIT);

            ea_finetimer_isr();

            DBG_SWDIAG(BLE_ISR, FINETGTIMINT, 0);
        }
#if ((BLE_CENTRAL || BLE_PERIPHERAL) && (BLE_AUDIO))
        // Audio channel 0 interrupt
        if (irq_stat & BLE_AUDIOINT0STAT_BIT)
        {
            DBG_SWDIAG(BLE_ISR2, AUDIO0INT, 1);
            ble_intack_clear(BLE_AUDIOINT0STAT_BIT);
            audio_evt_processed(AUDIO_CHANNEL_0);
            DBG_SWDIAG(BLE_ISR2, AUDIO0INT, 0);
        }
        // Audio channel 1 interrupt
        if (irq_stat & BLE_AUDIOINT1STAT_BIT)
        {
            DBG_SWDIAG(BLE_ISR2, AUDIO1INT, 1);
            ble_intack_clear(BLE_AUDIOINT1STAT_BIT);
            audio_evt_processed(AUDIO_CHANNEL_1);
            DBG_SWDIAG(BLE_ISR2, AUDIO1INT, 0);
        }
        // Audio channel 2 interrupt
        if (irq_stat & BLE_AUDIOINT2STAT_BIT)
        {
            DBG_SWDIAG(BLE_ISR2, AUDIO2INT, 1);
            ble_intack_clear(BLE_AUDIOINT2STAT_BIT);
            audio_evt_processed(AUDIO_CHANNEL_2);
            DBG_SWDIAG(BLE_ISR2, AUDIO2INT, 0);
        }
#endif
        // End of event interrupt
        if (irq_stat & BLE_EVENTINTSTAT_BIT)
        {
            DBG_SWDIAG(BLE_ISR, EVENTINT, 1);

            // Clear the interrupt
            ble_intack_clear(BLE_EVENTINTSTAT_BIT);

            if (irq_stat & BLE_RXINTSTAT_BIT)
            {
                ble_intack_clear(BLE_RXINTSTAT_BIT);
                irq_stat &= ~BLE_RXINTSTAT_BIT;
            }
            lld_evt_end_isr(false);

            wdt_feed(0x1fff);

            DBG_SWDIAG(BLE_ISR, EVENTINT, 0);
        }

        // AFPM interrupt
        if (irq_stat & BLE_EVENTAPFAINTSTAT_BIT)
        {
            DBG_SWDIAG(BLE_ISR2, EVENTAPFMINT, 1);
            // Clear the interrupt
            ble_intack_clear(BLE_EVENTAPFAINTSTAT_BIT);
            if (irq_stat & BLE_RXINTSTAT_BIT)
            {
                ble_intack_clear(BLE_RXINTSTAT_BIT);
                irq_stat &= ~BLE_RXINTSTAT_BIT;
            }

            lld_evt_end_isr(true);

            DBG_SWDIAG(BLE_ISR2, EVENTAPFMINT, 0);
        }

        // Rx interrupt
        if (irq_stat & BLE_RXINTSTAT_BIT)
        {
            DBG_SWDIAG(BLE_ISR, RXINT, 1);
            ble_intack_clear(BLE_RXINTSTAT_BIT);
            lld_evt_rx_isr();
            DBG_SWDIAG(BLE_ISR, RXINT, 0);
        }

        // SW interrupt
        if (irq_stat & BLE_SWINTSTAT_BIT)
        {
            DBG_SWDIAG(BLE_ISR2, SWINT, 1);
            // Clear the interrupt
            ble_intack_clear(BLE_SWINTSTAT_BIT);

            ea_sw_isr();
            DBG_SWDIAG(BLE_ISR2, SWINT, 0);
        }

#if (!BT_DUAL_MODE)
        // General purpose timer interrupt
        if (irq_stat & BLE_GROSSTGTIMINTSTAT_BIT)
        {
            DBG_SWDIAG(BLE_ISR, GROSSTGTIMINT, 1);

            // Clear the interrupt
            ble_intack_clear(BLE_GROSSTGTIMINTACK_BIT);

            lld_evt_timer_isr();

            DBG_SWDIAG(BLE_ISR, GROSSTGTIMINT, 0);
        }
#endif //(!BT_DUAL_MODE)

        // End of encryption interrupt
        if (irq_stat & BLE_CRYPTINTSTAT_BIT)
        {
            DBG_SWDIAG(BLE_ISR, CRYPTINT, 1);

            ble_intack_clear(BLE_CRYPTINTSTAT_BIT);

            lld_crypt_isr();

            DBG_SWDIAG(BLE_ISR, CRYPTINT, 0);
        }

        // Error interrupt
        if (irq_stat & BLE_ERRORINTSTAT_BIT)
        {
            DBG_SWDIAG(BLE_ISR, ERRORINT, 1);

            ASSERT_INFO(0, ble_errortypestat_get(), 0);

            // Clear the interrupt
            ble_intack_clear(BLE_ERRORINTSTAT_BIT);
            DBG_SWDIAG(BLE_ISR, ERRORINT, 0);
        }
    }
}

/// @} RWBLE
