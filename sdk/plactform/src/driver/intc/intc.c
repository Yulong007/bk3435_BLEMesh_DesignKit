/**
 ****************************************************************************************
 *
 * @file intc.c
 *
 * @brief Definition of the Interrupt Controller (INTCTRL) API.
 *
 * Copyright (C) RivieraWaves 2009-2015
 *
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup INTC
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "compiler.h"        // for inline functions
#include "arch.h"            // arch definition
#include "co_math.h"         // math library definition
#include "rwip_config.h"     // stack configuration
#include "user_config.h"
#include "rwip.h"
#if defined(CFG_BT)
#include "rwbt.h"            // rwbt core
#endif //CFG_BT
#if defined(CFG_BLE)
#include "rwble.h"           // rwble core
#endif //CFG_BLE

#include "intc.h"            // interrupt controller
#include "reg_intc.h"        // intc registers
#include "icu.h"
#if (UART_DRIVER)            // uart definitions
#if (BLE_RF_TEST)
#include "uart_rf.h"
#else
#include "uart.h"
#endif
#endif
#if (GPIO_DRIVER)
#include "gpio.h"
#endif
#if (AUDIO_DRIVER)
#include "audio.h"
#endif
#if (PWM_DRIVER)
#include "pwm.h"
#endif
#if (ADC_DRIVER)
#include "adc.h"
#endif

#include "led.h"

/*
 * DEFINES
 ****************************************************************************************
 */

#define RWBT_INT      CO_BIT(INTC_BT)
#define PCM_INT       CO_BIT(INTC_PCM)
#define TIMER_INT     CO_BIT(INTC_TIMER1)
#define UART_INT      CO_BIT(INTC_UART)
#define RWBLE_INT     CO_BIT(INTC_BLE)
#define DMA_INT       CO_BIT(INTC_DMA)

// enable the supported interrupts
#define PLF_INT     (UART_INT | DMA_INT)
#if defined(CFG_BT)
#define BT_INT      (RWBT_INT)
#else
#define BT_INT       0
#endif // #if defined(CFG_BT)
#if defined(CFG_BLE)
#define BLE_INT     (RWBLE_INT)
#else
#define BLE_INT      0
#endif // #if defined(CFG_BLE)



/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */
/// locally define this type to be able to qualify the array.
typedef void (*void_fn)(void);

extern uint8_t system_mode;


void intc_spurious(void)
{
    // force error
    ASSERT_ERR(0);
}

void intc_init(void)
{
    // Clear all interrupts
    intc_enable_clear(INT_IRQ_BIT | FIQ_IRQ_BIT);

    // enable the supported interrupts
    intc_enable_set(INT_IRQ_BIT | FIQ_IRQ_BIT);

    intc_module_enable_set(INT_BLE_bit | INT_UART_bit);
}

void intc_stat_clear(void)
{
    // Clear all interrupts
    intc_status_clear(0xFFFF);

}

void IRQ_Exception(void)
{
    uint32_t IntStat;
    uint32_t irq_status;

    IntStat = intc_status_get();

#if (SYSTEM_SLEEP)
    if (system_mode == RW_NO_MODE)
        cpu_wakeup();
#endif
#if (UART_DRIVER)
    // call the function handler
    if (IntStat & INT_STATUS_UART_bit)
    {
        irq_status |= INT_STATUS_UART_bit;
        uart_isr();
    }
#endif
#if (AUDIO_DRIVER)
    if (IntStat & INT_STATUS_SDM_bit)
    {
        irq_status |= INT_STATUS_SDM_bit;
        audio_isr();
    }
#endif
#if (GPIO_DRIVER)
    if (IntStat & INT_STATUS_GPIO_bit)
    {
        irq_status |= INT_STATUS_GPIO_bit;
        gpio_isr();
    }
#endif
#if (PWM_DRIVER)
    if (IntStat & INT_STATUS_PWM1_bit)
    {
        irq_status |= INT_STATUS_PWM1_bit;
        pwm_isr();
    }
    if (IntStat & INT_STATUS_PWM2_bit)
    {
        irq_status |= INT_STATUS_PWM2_bit;
        pwm_isr();
    }
    if (IntStat & INT_STATUS_PWM3_bit)
    {
        irq_status |= INT_STATUS_PWM3_bit;
        pwm_isr();
    }
    if (IntStat & INT_STATUS_PWM4_bit)
    {
        irq_status |= INT_STATUS_PWM4_bit;
        pwm_isr();
    }

    if (IntStat & INT_STATUS_PWM5_bit)
    {
        irq_status |= INT_STATUS_PWM5_bit;

        pwm6_irq_done();
    }
#endif
#if (ADC_DRIVER)
    if (IntStat & INT_STATUS_ADC_bit)
    {
        irq_status |= INT_STATUS_ADC_bit;
        adc_isr();
    }
#endif
    intc_status_clear(irq_status);

}

void FIQ_Exception(void)
{
    uint32_t IntStat;
    uint32_t fiq_status;
    IntStat = intc_status_get();

#if (SYSTEM_SLEEP)
    if (system_mode == RW_NO_MODE)
        cpu_wakeup();
#endif

    // call the function handler
    if (IntStat & INT_STATUS_BLE_bit)
    {
        fiq_status |= INT_STATUS_BLE_bit;
        rwble_isr();
    }
    if (IntStat & INT_STATUS_LBD_bit)
    {
        fiq_status |= INT_STATUS_LBD_bit;
    }
#if (GPIO_DRIVER)
    if (IntStat & INT_STATUS_GPIO_bit)
    {
        fiq_status |= INT_STATUS_GPIO_bit;
        gpio_isr();
    }
#endif
#if (PWM_DRIVER)
    if (IntStat & INT_STATUS_PWM0_bit)
    {
        fiq_status |= INT_STATUS_PWM0_bit;
        pwm_isr();
    }
#endif

    intc_status_clear(IntStat);
}


#pragma ARM//不要修改以下两个文件
__IRQ FAST_IRQ_ENTRY void SYSirq_IRQ_Handler(void)
{
    IRQ_Exception();
}

__FIQ FAST_FIQ_ENTRY void SYSirq_FIQ_Handler(void)
{
    FIQ_Exception();
}


/// @} INTC
