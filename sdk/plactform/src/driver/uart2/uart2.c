/**
 ****************************************************************************************
 *
 * @file uart.c
 *
 * @brief UART driver
 *
 * Copyright (C) RivieraWaves 2009-2015
 *
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup UART
 * @{
 ****************************************************************************************
 */
/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include <stddef.h>     // standard definition
#include "timer.h"      // timer definition
#include "uart2.h"       // uart definition
#include "BK3435_reg.h"
#include "reg_uart2.h"   // uart register
#include "rwip.h"       // SW interface
#include "h4tl.h"
#include "nvds.h"       // NVDS

#include "dbg.h"

/*
 * DEFINES
 *****************************************************************************************
 */

/// Max baudrate supported by this UART (in bps)
#define UART2_BAUD_MAX      		  3500000
/// Min baudrate supported by this UART (in bps)
#define UART2_BAUD_MIN      		  9600

/// Duration of 1 byte transfer over UART (10 bits) in us (for 921600 default baudrate)
#define UART2_CHAR_DURATION        11

/*
 * ENUMERATION DEFINITIONS
 *****************************************************************************************
 */



/*
* common.c
*
*  Created on: 2013-5-7
*  Author: pujie
*/

#include  <stdarg.h>
#include  <stdio.h>
#include "BK3435_reg.h"
#include "ea.h"


#define Uart2_Write_Byte(v)               (REG_APB3_UART2_PORT=v)

#define UART2_TX_FIFO_COUNT               (REG_APB3_UART2_FIFO_STAT&0xff)
#define UART2_RX_FIFO_COUNT               ((REG_APB3_UART2_FIFO_STAT>>8)&0xff)
#define UART2_TX_FIFO_FULL                (REG_APB3_UART2_FIFO_STAT&0x00010000)
#define UART2_TX_FIFO_EMPTY               (REG_APB3_UART2_FIFO_STAT&0x00020000)
#define UART2_RX_FIFO_FULL                (REG_APB3_UART2_FIFO_STAT&0x00040000)
#define UART2_RX_FIFO_EMPTY               (REG_APB3_UART2_FIFO_STAT&0x00080000)
#define UART2_TX_WRITE_READY              (REG_APB3_UART2_FIFO_STAT&0x00100000)
#define UART2_RX_READ_READY               (REG_APB3_UART2_FIFO_STAT&0x00200000)
#define bit_UART2_TXFIFO_NEED_WRITE        0x01
#define bit_UART2_RXFIFO_NEED_READ         0x02
#define bit_UART2_RXFIFO_OVER_FLOW         0x04
#define bit_UART2_RX_PARITY_ERROR          0x08
#define bit_UART2_RX_STOP_ERROR            0x10
#define bit_UART2_TX_PACKET_END            0x20
#define bit_UART2_RX_PACKET_END            0x40
#define bit_UART2_RXD_WAKEUP_DETECT        0x80


#define uart2_tx_en    0x1      // 0: Disable TX, 1: Enable TX
#define uart2_rx_en    0x1      // 0: Disable RX, 1: Enable RX
#define uart2_irda_mode     0x0      // 0: UART  MODE, 1: IRDA MODE
#define uart2_data_len      0x3      // 0: 5 bits, 1: 6 bits, 2: 7 bits, 3: 8 bits
#define uart2_parity_en     0x0      // 0: NO Parity, 1: Enable Parity
#define uart2_parity_mode   0x1      // 0: Odd Check, 1: Even Check
#define uart2_stop_bits     0x0      // 0: 1 stop-bit, 1: 2 stop-bit
#define uart2_clks     16000000 // UART's Main-Freq, 26M
#define uart2_baud_rate     115200   // UART's Baud-Rate,  1M


unsigned char uart2_rx_buf[UART2_RX_FIFO_MAX_COUNT];
unsigned char uart2_tx_buf[UART2_TX_FIFO_MAX_COUNT];
volatile bool uart2_rx_done = 0;
volatile unsigned long uart2_rx_index = 0;
uint8_t uart2_cur_read_buf_idx = 0;

extern uint8_t system_mode;
#define UART2_READ_BYTE()                 (REG_APB3_UART2_PORT&0xff)


///UART Character format
enum UART2_CHARFORMAT
{
    UART2_CHARFORMAT_8 = 0,
    UART2_CHARFORMAT_7 = 1
};

///UART Stop bit
enum UART2_STOPBITS
{
    UART2_STOPBITS_1 = 0,
    UART2_STOPBITS_2 = 1  /* Note: The number of stop bits is 1.5 if a character format
                            with 5 bit is chosen*/
};

///UART Parity enable
enum UART2_PARITY
{
    UART2_PARITY_DISABLED = 0,
    UART2_PARITY_ENABLED  = 1
};

///UART Parity type
enum UART2_PARITYBIT
{
    UART2_PARITYBIT_EVEN  = 0,
    UART2_PARITYBIT_ODD   = 1,
    UART2_PARITYBIT_SPACE = 2, // The parity bit is always 0.
    UART2_PARITYBIT_MARK  = 3  // The parity bit is always 1.
};

///UART HW flow control
enum UART2_HW_FLOW_CNTL
{
    UART2_HW_FLOW_CNTL_DISABLED = 0,
    UART2_HW_FLOW_CNTL_ENABLED = 1
};

///UART Input clock select
enum UART2_INPUT_CLK_SEL
{
    UART2_INPUT_CLK_SEL_0 = 0,
    UART2_INPUT_CLK_SEL_1 = 1,
    UART2_INPUT_CLK_SEL_2 = 2,
    UART2_INPUT_CLK_SEL_3 = 3
};

///UART Interrupt enable/disable
enum UART2_INT
{
    UART2_INT_DISABLE = 0,
    UART2_INT_ENABLE = 1
};

///UART Error detection
enum UART2_ERROR_DETECT
{
    UART2_ERROR_DETECT_DISABLED = 0,
    UART2_ERROR_DETECT_ENABLED  = 1
};

/*
 * STRUCT DEFINITIONS
 *****************************************************************************************
 */
/* TX and RX channel class holding data used for asynchronous read and write data
 * transactions
 */
/// UART TX RX Channel
struct uart2_txrxchannel
{
	/// call back function pointer
	void (*callback) (void*, uint8_t);
	/// Dummy data pointer returned to callback when operation is over.
	void* dummy;
};

/// UART environment structure
struct uart2_env_tag
{
	/// tx channel
	struct uart2_txrxchannel tx;
	/// rx channel
	struct uart2_txrxchannel rx;
	/// error detect
	uint8_t errordetect;
	/// external wakeup
	bool ext_wakeup;
};

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

/// uart environment structure
struct uart2_env_tag uart2_env;
char uart2_buff[64];



static UART_RX_CALLBACK_T usrt2_rx_cb = NULL;

/*
 * LOCAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */


/*
 * EXPORTED FUNCTION DEFINITIONS
 ****************************************************************************************
 */
uint8_t Read_Uart2_Buf(void)
{
	return uart2_rx_buf[uart2_cur_read_buf_idx++];
}

uint8_t Uart2_Read_Byte(void)
{
	return (REG_APB3_UART2_PORT&0xff);
}


int uart2_putchar(char * st)
{
	uint8_t num = 0;
	while (*st)
	{
		if(UART2_TX_WRITE_READY)
		{
			REG_APB3_UART2_PORT = *st;
			st++;
			num++;
		}
	}
	return num;
}

int uart2_printf(const char *fmt,...)
{
#if (UART_PRINTF_EN && UART_DRIVER)
	int n;

	va_list ap;
	va_start(ap, fmt);
	n=vsprintf(uart2_buff, fmt, ap);
	va_end(ap);
	uart2_putchar(uart2_buff);
	if(n > sizeof(uart2_buff))
	{
		uart2_putchar("buff full \r\n");
	}

	return n;
#else
    return 0;
#endif
}

int uart2_printf_null(const char *fmt,...)
{
	return 0;
}


char *uart2_hex2Str( uint8_t data)
{

	char hex[] = "0123456789ABCDEF";
	static char str[3];
	char *pStr = str;

	*pStr++ = hex[data >> 4];
	*pStr++ = hex[data & 0x0F];
	*pStr = 0;

	return str;
}




void cpu2_delay( volatile unsigned int times)
{
	while(times--)
	{
		for(uint32_t i = 0; i < 1000; i++)
			;
	}
}


void uart2_init(uint32_t baudrate)
{
	unsigned int baud_divisor ;
	REG_AHB0_ICU_UARTCLKCON   &= ~(0x1 << 0) ;  // Enable Uart's Clocks
	switch(baudrate)
	{
	    case 9600://16M/9600
            baud_divisor = 1666;
            break;
        case 19200://16M/19200
            baud_divisor = 833;
            break;
        case 38400://16M/38400
            baud_divisor = 416;
            break;
        case 57600://16M/57600
            baud_divisor = 277;//
            break;
        case 115200:
            baud_divisor = 0x89;//137
            break;
        case 256000:
            baud_divisor = 62;//137
            break;
        case 512000:
            baud_divisor = 31;//137
            break;
        case 750000:
            baud_divisor = 21;
            break;
        case 921600:
            baud_divisor = 17;
            break;

		case 1500000:
			baud_divisor = 10;
			break;
		case 2000000:
			baud_divisor = 7;
			break;
		
	default:
		baud_divisor = 0x89;
		break;
	}
	REG_APB3_UART2_CFG  = (baud_divisor<<8) +
	                     (uart2_stop_bits   <<7) +
	                     (uart2_data_len    <<3) +
	                     (uart2_irda_mode   <<2) +
	                     (uart2_rx_en  <<1) +
	                     uart2_tx_en ;

	REG_APB3_UART2_FIFO_CFG = (1<<BIT2_TX_FIFO_THRESHOLD)|(16<<BIT2_RX_FIFO_THRESHOLD)|(0x2 << 	BIT2_STOP_DETECT_TIME);//(0x3 << BIT_STOP_DETECT_TIME);
	REG_APB3_UART2_INT_ENABLE = ((0x01 << 1) | (0x01 << 6) | (0x01 << 7));
	REG_APB3_UART2_FLOW_CFG  = 0x00000000 ;  // No Flow Control
	REG_APB3_UART2_WAKE_CFG  = ((0x01 << 0 )| (0x01 << 20) |  (0x01 << 21)| (0x01 << 22));  // No Wake Control

	REG_APB5_GPIOB_CFG  &= ~((0xc0<<BIT_GPIO_PULL_UP)  + (0xc0<<BIT_GPIO_PERI_EN));
	REG_APB5_GPIOB_CFG  |= ((0xc0<<BIT_GPIO_PULL_UP));
	REG_APB5_GPIOB_CFG  |=   (0xc0<<BIT_GPIO_OUT_EN_N);

	REG_APB5_GPIOB_DATA &= ~ (0xc0<<BIT_GPIO_INPUT_EN);


	uart2_env.rx.callback = NULL;
	uart2_env.rx.dummy    = NULL;

	REG_AHB0_ICU_INT_ENABLE |= (0x01 << 16);

}



void uart2_flow_on(void)
{
	// Configure modem (HW flow control enable)
	// uart_flow_en_setf(0);
}

void uart2_clear_rxfifo(void)
{

	while(uart2_rx_fifo_rd_ready_getf())
	{
		Uart2_Read_Byte();
	}
	memset(uart2_rx_buf,0,UART2_RX_FIFO_MAX_COUNT);

}
bool uart2_flow_off(void)
{

	return true;

}

void uart2_finish_transfers(void)
{
	uart2_flow_en_setf(1);

	// Wait TX FIFO empty
	while(!uart2_tx_fifo_empty_getf());
}


void uart2_read(uint8_t *bufptr, uint32_t size, void (*callback) (void*, uint8_t), void* dummy)
{
	// Sanity check
	ASSERT_ERR(bufptr != NULL);
	ASSERT_ERR(size != 0);
	ASSERT_ERR(callback != NULL);
	uart2_env.rx.callback = callback;

	uart2_env.rx.dummy    = dummy;

}



void uart2_write(uint8_t *bufptr, uint32_t size, void (*callback) (void*, uint8_t), void* dummy)
{
	// Sanity check
	ASSERT_ERR(bufptr != NULL);
	ASSERT_ERR(size != 0);
	ASSERT_ERR(callback != NULL);

	uint8_t len;
	len = size;

	uart2_env.tx.callback = callback;
	uart2_env.tx.dummy    = dummy;

	//Delay_ms(100);
	while(len--)
	{
		//cpu_delay(10);
		if(UART2_TX_WRITE_READY)
		{
			REG_APB3_UART2_PORT = *bufptr;
			bufptr++;
		}
	}

	if(callback != NULL)
	{
		// Clear callback pointer
		uart2_env.tx.callback = NULL;
		uart2_env.tx.dummy    = NULL;
		// Call handler
		callback(dummy, RWIP_EIF_STATUS_OK);
	}
}

static void uart2_send_byte(unsigned char data)
{
	while (!uart2_tx_fifo_empty_getf());

	REG_APB3_UART2_PORT = data ;
}

void uart2_send(unsigned char *buff, int len)
{
	while (len--)
		uart2_send_byte(*buff++);
}

void uart2_cb_register(UART_RX_CALLBACK_T cb)
{
	if(cb)
	{
		usrt2_rx_cb = cb;
	}
}

void uart2_cb_clear(void)
{
	if(usrt2_rx_cb)
	{
		usrt2_rx_cb = NULL;
	}
}

volatile uint8_t uart2_dut_reg_flag = 0;
extern void pn9_test_process(uint8_t* buffer, uint8_t len);

enum
{
    NORMAL_MODE = 0,
    DUT_MODE = 1,
    REG_MODE = 2,
};

void uart2_isr(void)
{
	uint32_t IntStat;

	IntStat = uart2_isr_stat_get();
	if(uart2_rx_fifo_need_rd_isr_getf() || uart2_rx_end_isr_getf()|| uart2_rxd_wakeup_isr_getf())
	{
		while((REG_APB3_UART2_FIFO_STAT & (0x01 << 21)))
		{
			uart2_rx_buf[uart2_rx_index++] = UART2_READ_BYTE();
			if( UART2_RX_FIFO_MAX_COUNT == uart2_rx_index )
			{
				uart2_rx_index = 0;
			}
		}
	}

	uart2_rx_done = 1;
	if(usrt2_rx_cb)
	{
		(*usrt2_rx_cb)(uart2_rx_buf, uart2_rx_index);
	}
	uart2_rx_index = 0;

	uart2_isr_stat_set(IntStat);
}

uint8_t check_uart2_stop(void)
{
	return uart2_tx_fifo_empty_getf();
}

/// @} UART2

