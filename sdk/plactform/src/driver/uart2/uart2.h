/**
 ****************************************************************************************
 *
 * @file uart.h
 *
 * @brief UART Driver for HCI over UART operation.
 *
 * Copyright (C) Beken 2009-2015
 *
 *
 ****************************************************************************************
 */

#ifndef _UART2_H_
#define _UART2_H_

/**
 ****************************************************************************************
 * @defgroup UART UART
 * @ingroup DRIVERS
 * @brief UART driver
 *
 * @{
 *
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include <stdbool.h>          // standard boolean definitions
#include <stdint.h>           // standard integer functions
#include "rwip_config.h"
#include "user_config.h"


#if (UART_PRINTF_EN && UART_DRIVER)

#if  !BLE_TESTER
#define UART2_PRINTF	uart2_printf //uart_printf
#else
#define UART2_PRINTF uart2_printf //uart_printf
#endif //!BLE_TESTER

#else
#define UART2_PRINTF uart2_printf_null
#endif // #if UART_PRINTF_EN


/*
 * ENUMERATION DEFINITIONS
 *****************************************************************************************
 */

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Initializes the UART to default values.
 *****************************************************************************************
 */
void uart2_init(uint32_t baudrate);

void dbg2_initial(void);

void uart2_clear_rxfifo(void);

uint8_t Uart2_Read_Byte(void);
uint8_t Read2_Uart_Buf(void);

int dbg2_putchar(char * st);
int uart2_putchar(char * st);
int uart2_printf(const char *fmt,...);
int uart2_printf_null(const char *fmt,...);
int dbg2_printf(const char *fmt,...);
void uart2_print_int(unsigned int num);
uint8_t check_uart2_stop(void);

void cpu2_delay( volatile unsigned int times);


/****** REG  ****************/
void uart2_send(unsigned char *buff, int len);
void TRAhcit_UART2_Rx(void);

#define UART2_RX_FIFO_MAX_COUNT  64

#define UART2_TX_FIFO_MAX_COUNT  64

extern unsigned char uart2_rx_buf[UART2_RX_FIFO_MAX_COUNT];
extern unsigned char uart2_tx_buf[UART2_TX_FIFO_MAX_COUNT];
extern volatile bool uart2_rx_done ;
extern volatile unsigned long uart2_rx_index ;
/****** REG  ****************/



#ifndef CFG_ROM
/**
 ****************************************************************************************
 * @brief Enable UART flow.
 *****************************************************************************************
 */
void uart2_flow_on(void);

/**
 ****************************************************************************************
 * @brief Disable UART flow.
 *****************************************************************************************
 */
bool uart2_flow_off(void);
#endif //CFG_ROM

/**
 ****************************************************************************************
 * @brief Finish current UART transfers
 *****************************************************************************************
 */
void uart2_finish_transfers(void);

/**
 ****************************************************************************************
 * @brief Starts a data reception.
 *
 * @param[out] bufptr   Pointer to the RX buffer
 * @param[in]  size     Size of the expected reception
 * @param[in]  callback Pointer to the function called back when transfer finished
 * @param[in]  dummy    Dummy data pointer returned to callback when reception is finished
 *****************************************************************************************
 */
void uart2_read(uint8_t *bufptr, uint32_t size, void (*callback) (void*, uint8_t), void* dummy);

/**
 ****************************************************************************************
 * @brief Starts a data transmission.
 *
 * @param[in] bufptr   Pointer to the TX buffer
 * @param[in] size     Size of the transmission
 * @param[in] callback Pointer to the function called back when transfer finished
 * @param[in] dummy    Dummy data pointer returned to callback when transmission is finished
 *****************************************************************************************
 */
void uart2_write(uint8_t *bufptr, uint32_t size, void (*callback) (void*, uint8_t), void* dummy);

#if defined(CFG_ROM)
/**
 ****************************************************************************************
 * @brief Poll UART on reception and transmission.
 *
 * This function is used to poll UART for reception and transmission.
 * It is used when IRQ are not used to detect incoming bytes.
 *****************************************************************************************
 */
void uart2_poll(void);
#endif //CFG_ROM

/**
 ****************************************************************************************
 * @brief Serves the data transfer interrupt requests.
 *
 * It clears the requests and executes the appropriate callback function.
 *****************************************************************************************
 */
void uart2_isr(void);

typedef void (*UART_RX_CALLBACK_T)(uint8_t *buf, uint8_t len);

void uart2_cb_register(UART_RX_CALLBACK_T cb);

void uart2_cb_clear(void);


/// @} UART
#endif /* _UART_H_ */

