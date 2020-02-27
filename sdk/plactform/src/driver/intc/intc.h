/**
 ****************************************************************************************
 *
 * @file intc.h
 *
 * @brief Declaration of the Interrupt Controller API.
 *
 * Copyright (C) RivieraWaves 2009-2015
 *
 *
 ****************************************************************************************
 */

#ifndef _INTC_H_
#define _INTC_H_

/**
 ****************************************************************************************
 * @addtogroup INTC INTC
 * @ingroup DRIVERS
 *
 * @brief Declaration of the Interrupt Controller API.
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "compiler.h"

/*
 * DEFINES
 ****************************************************************************************
 */

/*****************************************************/
//              don't change this !!!!!!!
#define ISR_MAX_NUM  2
#define ISR_FIQ_ISR_IDX  0
#define ISR_IRQ_ISR_IDX  1

#define ISR_MAX_NUM_EVENT_USERS  19
#define PWM0_FIQ_ISR_IDX                                0
#define PWM1_IRQ_ISR_IDX                                1
#define PWM2_IRQ_ISR_IDX                                2
#define PWM3_IRQ_ISR_IDX                                3
#define PWM4_IRQ_ISR_IDX                                4

#define UART_IRQ_ISR_IDX                                5
#define SPI_IRQ_ISR_IDX                                 6
#define I2C_IRQ_ISR_IDX                                 7
#define ADC_IRQ_ISR_IDX                                 8
#define GPIO_IRQ_ISR_IDX                                9
#define RES_ISR_IDX0                                    10

#define USB_IRQ_ISR_IDX                                 11
#define RTC_IRQ_ISR_IDX                                 12
#define PWM5_IRQ_ISR_IDX                                13
#define PWM_3DSIRQ_ISR_IDX                              14
#define BLE_FIQ_ISR_IDX                                 15
#define UART2_IRQ_ISR_IDX                               16
#define SDM_IRQ_ISR_IDX                                 17
#define I2S_IRQ_ISR_IDX                                 18


/***************************************************/

/** @name Mapping of the peripherals interrupts in the interrupt controller.
 * @{
 */

//FIQ
#define INT_PWM0_bit     (0x01<<0)
#define INT_LBD_bit      (0x01<<10)
#define INT_BLE_bit      (0x01<<15)
//IRQ
#define INT_RTC_bit      (0x01<<12)
#define INT_GPIO_bit     (0x01<< 9)
#define INT_ADC_bit      (0x01<< 8)
#define INT_I2C_bit      (0x01<< 7)
#define INT_SPI_bit      (0x01<< 6)
#define INT_UART_bit     (0x01<< 5)
#define INT_PWM4_bit     (0x01<< 4)
#define INT_PWM3_bit     (0x01<< 3)
#define INT_PWM2_bit     (0x01<< 2)
#define INT_PWM1_bit     (0x01<< 1)

//FIQ
#define INT_STATUS_PWM0_bit     (0x01<<0)
#define INT_STATUS_LBD_bit      (0x01<<10)
#define INT_STATUS_BLE_bit      (0x01<<15)
//IRQ
#define INT_STATUS_SDM_bit      (0x01<<17)
#define INT_STATUS_RTC_bit      (0x01<<12)
#define INT_STATUS_GPIO_bit     (0x01<< 9)
#define INT_STATUS_ADC_bit      (0x01<< 8)
#define INT_STATUS_I2C_bit      (0x01<< 7)
#define INT_STATUS_SPI_bit      (0x01<< 6)
#define INT_STATUS_UART_bit     (0x01<< 5)
#define INT_STATUS_PWM4_bit     (0x01<< 4)
#define INT_STATUS_PWM3_bit     (0x01<< 3)
#define INT_STATUS_PWM2_bit     (0x01<< 2)
#define INT_STATUS_PWM1_bit     (0x01<< 1)

#define INT_IRQ_BIT             (0x01<<0)
#define FIQ_IRQ_BIT             (0x01<<1)

/// @} INTC mapping


#define FAST_IRQ_ENTRY  __attribute__((section("sys_irq_entry")))
#define FAST_FIQ_ENTRY  __attribute__((section("sys_fiq_entry")))


/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */
/**
 ****************************************************************************************
 * @brief Initialize and configure the reference interrupt controller.
 *
 * This function configures the interrupt controller according to the system needs.
 *
 ****************************************************************************************
 */
void intc_init(void);

/**
 ****************************************************************************************
 * @brief Clear status of interrupts.
 *
 * This function clear interrupt status.
 *
 ****************************************************************************************
 */
void intc_stat_clear(void);

/**
 ****************************************************************************************
 * @brief IRQ Handler.
 *
 ****************************************************************************************
 */
__IRQ void intc_irq(void);

/**
 ****************************************************************************************
 * @brief FIQ Handler.
 *
 ****************************************************************************************
 */
__FIQ void intc_fiq(void);

/// @} INTC

#endif // _INTC_H_
