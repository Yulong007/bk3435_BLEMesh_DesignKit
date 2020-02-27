/**
 ****************************************************************************************
 *
 * @file arch_main.c
 *
 * @brief Main loop of the application.
 *
 * Copyright (C) RivieraWaves 2009-2015
 *
 *
 ******** ********************************************************************************
 */


/*



           |------------------|
           |   BACK_UP(176KB) |
           |                                    |
0x00054000  |------------------| F_ADDR = 0x00054000
           |                                    |
           |   USER_CONF(4KB) |
0x00053000  |------------------| F_ADDR = 0x00053000
           |   NVDS(4KB)      |
           |                                    |
0x00052000  |------------------| F_ADDR = 0x00052000
           |                                    |
           |   APP(176KB)     |
           |                                    |
0x00026000  |------------------| C_ADDR = 0x23E00,F_ADDR = 0x261E0 (APP)(0x261D0 16byte stack image header)
           |                                    |
           |  STACK(140KB)    |
           |                                    |
0x00003000  |------------------| C_ADDR = 0x2F00,F_ADDR = 0x31F0 (STACK)(0x31e0 16byte stack image header)
           |                                    |
           |    BIM(12KB)     |
           |                                    |
0x00000000  |------------------|

*/


/*
 * INCLUDES
 ****************************************************************************************
 */

#include "rwip_config.h" // RW SW configuration

#include "arch.h"      // architectural platform definitions
#include <stdlib.h>    // standard lib functions
#include <string.h>    // standard lib functions
#include <stdio.h>    // standard lib functions
#include <stddef.h>    // standard definitions
#include <stdint.h>    // standard integer definition
#include <stdbool.h>   // boolean definition
#include "boot.h"      // boot definition
#include "rwip.h"      // RW SW initialization
#include "syscntl.h"   // System control initialization
#include "emi.h"       // EMI initialization
#include "intc.h"      // Interrupt initialization
#include "timer.h"     // TIMER initialization
#include "icu.h"
#include "flash.h"
#include "uart.h"       // UART initialization
#include "flash.h"     // Flash initialization
//#include "led.h"       // Led initialization
#if (BLE_EMB_PRESENT || BT_EMB_PRESENT)
#include "rf.h"        // RF initialization
#endif // BLE_EMB_PRESENT || BT_EMB_PRESENT

#if (BLE_APP_PRESENT)
#include "app.h"       // application functions
#endif // BLE_APP_PRESENT

#include "nvds.h"         // NVDS definitions

#include "reg_assert_mgr.h"
#include "BK3435_reg.h"
#include "RomCallFlash.h"
#include "gpio.h"
#include "pwm.h"
#include "audio.h"
#include "app_task.h"
#include "ir.h"
#include "oads.h"
#include "wdt.h"
#include "user_config.h"
#include "app_mesh.h"
#include "led.h"
#include "app_light_server.h"
#include "lld_adv_test.h"


/**
 ****************************************************************************************
 * @addtogroup DRIVERS
 * @{
 *
 *
 * ****************************************************************************************
 */




/*
 * STRUCTURE DEFINITIONS
 ****************************************************************************************
 */


/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

// Creation of uart external interface api
struct rwip_eif_api uart_api;
extern uint8_t system_mode;

/*
 * LOCAL FUNCTION DECLARATIONS
 ****************************************************************************************
 */
extern void Delay_ms(int num) ;

static void Stack_Integrity_Check(void);
extern void code_sanity_check(void);
extern void rw_fcc_enter(void);
extern void rw_pn9_enter(void);

#if (UART_DRIVER)
void uart_rx_handler(uint8_t *buf, uint8_t len);
#endif

#if ((UART_PRINTF_EN) &&(UART_DRIVER))
void assert_err(const char *condition, const char * file, int line)
{
    //while(1)
    MESH_APP_PRINT_INFO("%s,condition %s,file %s,line = %d\r\n", __func__, condition, file, line);
// Delay_ms(5000);
}

void assert_param(int param0, int param1, const char * file, int line)
{
//  Delay_ms(5000);
    MESH_APP_PRINT_INFO("%s,param0 = 0x%x,param1 = 0x%x,file = %s,line = %d\r\n", __func__, param0, param1, file, line);

}

void assert_warn(int param0, int param1, const char * file, int line)
{
//   Delay_ms(5000);
    MESH_APP_PRINT_INFO("%s,param0 = 0x%x,param1 = 0x%x,file = %s,line = %d\r\n", __func__, param0, param1, file, line);

}

void dump_data(uint8_t* data, uint16_t length)
{
    //Delay_ms(5000);
    MESH_APP_PRINT_INFO("%s,data = 0x%x,length = 0x%x,file = %s,line = %d\r\n", __func__, data, length);

}
#else
void assert_err(const char *condition, const char * file, int line)
{

}

void assert_param(int param0, int param1, const char * file, int line)
{

}

void assert_warn(int param0, int param1, const char * file, int line)
{

}

void dump_data(uint8_t* data, uint16_t length)
{

}
#endif //UART_PRINTF_EN

#if 1
/*
 * LOCAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */

//用来加密的原始数据，需要在烧录代码的时候提供
//0102210630355cff0078b69d5538dd22
uint8_t encrypt_key_array[16] =
{
    0x01, 0x02, 0x21, 0x06,
    0x30, 0x35, 0x5c, 0xff,
    0x00, 0x78, 0xb6, 0x9d,
    0x55, 0x38, 0xdd, 0x22
};
#endif

/*
 * EXPORTED FUNCTION DEFINITIONS
 ****************************************************************************************
 */


void platform_reset(uint32_t error)
{
    //void (*pReset)(void);

    MESH_APP_PRINT_INFO("error = %x\r\n", error);

    // Disable interrupts
    GLOBAL_INT_STOP();

#if UART_PRINTF_EN
    // Wait UART transfer finished
    uart_finish_transfers();
#endif //UART_PRINTF_EN


    if (error == RESET_AND_LOAD_FW || error == RESET_TO_ROM)
    {
        // Not yet supported
    }
    else
    {
        //Restart FW
        //pReset = (void * )(0x0);
        //pReset();
        wdt_enable(10);
        while (1);
    }
}

/************************************************************************************************/
/************************************************************************************************/

uint32_t loop = 0;
uint32_t loop1 = 0;

void sys_mode_init(void)
{
    system_mode = RW_NO_MODE;
}


void rw_dut_enter(void)
{
    /*
     ***************************************************************************
     * Main loop
     ***************************************************************************
     */
    while (1)
    {
        // schedule all pending events
        rwip_schedule();
    }
}

void rw_app_enter(void)
{

#if SYSTEM_SLEEP
    uint8_t sleep_type = 0;
#endif

    /*
    ***************************************************************************
    * Main loop
    ***************************************************************************
    */
    while (1)
    {
        //schedule all pending events
        rwip_schedule();

        // Checks for sleep have to be done with interrupt disabled
        GLOBAL_INT_DISABLE();

        oad_updating_user_section_pro();

#if 0//SYSTEM_SLEEP 
        // Check if the processor clock can be gated
        sleep_type = rwip_sleep();
        if ((sleep_type & RW_MCU_DEEP_SLEEP) == RW_MCU_DEEP_SLEEP)
        {
            // 1:idel  0:reduce voltage
            if (icu_get_sleep_mode())
            {
                cpu_idle_sleep();
            }
            else
            {
                cpu_reduce_voltage_sleep();
            }
        }
        else if ((sleep_type & RW_MCU_IDLE_SLEEP) == RW_MCU_IDLE_SLEEP)
        {
            cpu_idle_sleep();
        }
#endif
        Stack_Integrity_Check();
        GLOBAL_INT_RESTORE();
        loop++;
        if (loop % 850000 == 0)
        {
            MESH_APP_PRINT_INFO("rwip_schedule %d\n", loop1++);
        }
    }
}

/************************************************************************************************/
/************************************************************************************************/

void bdaddr_env_init(void)
{
    struct bd_addr co_bdaddr;
    flash_read_data(&co_bdaddr.addr[0], 0x400e3, 6);
    if (co_bdaddr.addr[0]!=0xff ||co_bdaddr.addr[1]!=0xff||
            co_bdaddr.addr[2]!=0xff||co_bdaddr.addr[3]!=0xff||
            co_bdaddr.addr[4]!=0xff||co_bdaddr.addr[5]!=0xff )
    {
        memcpy(&co_default_bdaddr, &co_bdaddr, 6);
    }
#if MAC78da07bcd71b
    co_default_bdaddr.addr[0] = 0x78;//78da07bcd71b;
    co_default_bdaddr.addr[1] = 0xda;
    co_default_bdaddr.addr[2] = 0x07;
    co_default_bdaddr.addr[3] = 0xbc;
    co_default_bdaddr.addr[4] = 0xd7;
    co_default_bdaddr.addr[5] = 0x1b;
#elif MAC78da07bcd71c
    co_default_bdaddr.addr[0] = 0x78;//78da07bcd71b;
    co_default_bdaddr.addr[1] = 0xda;
    co_default_bdaddr.addr[2] = 0x07;
    co_default_bdaddr.addr[3] = 0xbc;
    co_default_bdaddr.addr[4] = 0xd7;
    co_default_bdaddr.addr[5] = 0x1c;
#elif   MAC78da07bcd71d
    co_default_bdaddr.addr[0] = 0x78;//78da07bcd71b;
    co_default_bdaddr.addr[1] = 0xda;
    co_default_bdaddr.addr[2] = 0x07;
    co_default_bdaddr.addr[3] = 0xbc;
    co_default_bdaddr.addr[4] = 0xd7;
    co_default_bdaddr.addr[5] = 0x1d;
#endif

    MESH_APP_PRINT_INFO("co_default_bdaddr.addr: ");
    for (int i =0 ; i < 6; i++)
    {
        MESH_APP_PRINT_INFO("%02x ", co_default_bdaddr.addr[i]);
    } MESH_APP_PRINT_INFO("\r\n");
}


/**
 *******************************************************************************
 * @brief RW main function.
 *
 * This function is called right after the booting process has completed.
 *
 * @return status   exit status
 *******************************************************************************
 */

extern struct rom_env_tag rom_env;

void rwip_eif_api_init(void);

int main(void)
{
    /*
     ***************************************************************************
     * Platform initialization
     ***************************************************************************
     */

    gpio_init();
    // Initialize random process
    srand(1);

    sys_mode_init();

    //get System sleep flag
    system_sleep_init();

    // Initialize the exchange memory interface
    emi_init();

    // Initialize timer module
    timer_init();

    rwip_eif_api_init();

    // Initialize the Interrupt Controller
    intc_init();
    // Initialize UART component
#if (UART_DRIVER)
    uart_init(115200);
    uart_cb_register(uart_rx_handler);
#endif
    MESH_APP_PRINT_INFO("--------------------2019.5.17 ------------------------------------------\r\n");
    MESH_APP_PRINT_INFO("|                                                                        |\r\n");
    MESH_APP_PRINT_INFO("----------------compiler TIME: %s -----------------\r\n", __TIME__);
    MESH_APP_PRINT_INFO("|                                                                        |\r\n");
    MESH_APP_PRINT_INFO("-------------------------------------------------------------------------\r\n");
    flash_advance_init();
    bdaddr_env_init();

#if  0//bk encrypt interface test
    code_sanity_check();
#endif

    // Initialize NVDS module
    struct nvds_env_tag env;
    env.flash_read = &flash_read;
    env.flash_write = &flash_write;
    env.flash_erase = &flash_erase;
    nvds_init(env);

    icu_init();

    flash_init();

    rom_env_init(&rom_env);
    MESH_APP_PRINT_INFO("rom_env_init\n");
    /*
    ***************************************************************************
    * RW SW stack initialization
    ***************************************************************************
    */
    // Initialize RW SW stack
    rwip_init(0);
    MESH_APP_PRINT_INFO("rwip_init\n");
    if (system_mode == RW_DUT_MODE)
    {
        switch_clk(MCU_CLK_16M);
    }
    else
    {
        switch_clk(MCU_CLK_64M);
    }


    light_status_init();
    MESH_APP_PRINT_INFO("light_status_init\n");
    REG_AHB0_ICU_INT_ENABLE |= (0x01 << 15); //BLE INT
    REG_AHB0_ICU_IRQ_ENABLE = 0x03;

    // finally start interrupt handling
    GLOBAL_INT_START();


    /*
     ***************************************************************************
     * Choose whitch enter to use
     ***************************************************************************
     */
    if (system_mode == RW_DUT_MODE)
    {
        MESH_APP_PRINT_INFO("DUT Mode Start\n");
        rw_dut_enter();
    }
    else if (system_mode == RW_FCC_MODE)
    {
        MESH_APP_PRINT_INFO("FCC Mode Start\n");
        //rw_fcc_enter();
    }
    else if (system_mode == RW_PN9_MODE)
    {
        MESH_APP_PRINT_INFO("PN9 Mode Start\n");
        rw_pn9_enter();
    }
    else //normal mode
    {
        MESH_APP_PRINT_INFO("NORMAL Mode Start\n");
        rw_app_enter();
    }


    while (1)
    {
        ;
    }

}


#if (UART_DRIVER)
//USE TEST
static void uart_rx_handler(uint8_t *buf, uint8_t len)
{
    if (buf[0] == 0x01)
    {
        MESH_APP_PRINT_INFO("report on\n");
    }
    else if (buf[0] == 0x02)
    {
        MESH_APP_PRINT_INFO("report off\n");
    }
}
#endif

void rwip_eif_api_init(void)
{
    uart_api.read = &uart_read;
    uart_api.write = &uart_write;
    uart_api.flow_on = &uart_flow_on;
    uart_api.flow_off = &uart_flow_off;
}

const struct rwip_eif_api* rwip_eif_get(uint8_t type)
{
    const struct rwip_eif_api* ret = NULL;
    switch (type)
    {
        case RWIP_EIF_AHI:
        {
            ret = &uart_api;
        }
        break;
#if (BLE_EMB_PRESENT) || (BT_EMB_PRESENT)
        case RWIP_EIF_HCIC:
        {
            ret = &uart_api;
        }
        break;
#elif !(BLE_EMB_PRESENT) || !(BT_EMB_PRESENT)
        case RWIP_EIF_HCIH:
        {
            ret = &uart_api;
        }
        break;
#endif
        default:
        {
            ASSERT_INFO(0, type, 0);
        }
        break;
    }
    return ret;
}

static void Stack_Integrity_Check(void)
{
    if ((REG_PL_RD(STACK_BASE_UNUSED)!= BOOT_PATTERN_UNUSED))
    {
        while (1)
        {
            uart_putchar("Stack_Integrity_Check STACK_BASE_UNUSED fail!\r\n");
        }
    }

    if ((REG_PL_RD(STACK_BASE_SVC)!= BOOT_PATTERN_SVC))
    {
        while (1)
        {
            uart_putchar("Stack_Integrity_Check STACK_BASE_SVC fail!\r\n");
        }
    }

    if ((REG_PL_RD(STACK_BASE_FIQ)!= BOOT_PATTERN_FIQ))
    {
        while (1)
        {
            uart_putchar("Stack_Integrity_Check STACK_BASE_FIQ fail!\r\n");
        }
    }

    if ((REG_PL_RD(STACK_BASE_IRQ)!= BOOT_PATTERN_IRQ))
    {
        while (1)
        {
            uart_putchar("Stack_Integrity_Check STACK_BASE_IRQ fail!\r\n");
        }
    }

}


void rom_env_init(struct rom_env_tag *api)
{
    memset(&rom_env, 0, sizeof(struct rom_env_tag));
    rom_env.prf_get_id_from_task = prf_get_id_from_task;
    rom_env.prf_get_task_from_id = prf_get_task_from_id;
    rom_env.prf_init = prf_init;
    rom_env.prf_create = prf_create;
    rom_env.prf_cleanup = prf_cleanup;
    rom_env.prf_add_profile = prf_add_profile;
    rom_env.rwble_hl_reset = rwble_hl_reset;
    rom_env.rwip_reset = rwip_reset;

    rom_env.lld_adv_test_rx_isr_cb = lld_adv_test_rx_isr_cb;
    rom_env.lld_adv_test_end_cb = lld_adv_test_end_cb;

#if SYSTEM_SLEEP
    rom_env.rwip_prevent_sleep_set = rwip_prevent_sleep_set;
    rom_env.rwip_prevent_sleep_clear = rwip_prevent_sleep_clear;
    rom_env.rwip_sleep_lpcycles_2_us = rwip_sleep_lpcycles_2_us;
    rom_env.rwip_us_2_lpcycles = rwip_us_2_lpcycles;
    rom_env.rwip_wakeup_delay_set = rwip_wakeup_delay_set;
#endif
    rom_env.platform_reset = platform_reset;
    rom_env.assert_err = assert_err;
    rom_env.assert_param = assert_param;
    //rom_env.Read_Uart_Buf = read_uart_ringbuf_data;
    rom_env.uart_clear_rxfifo = uart_clear_rxfifo;
    rom_env.Read_Uart_Buf = Read_Uart_Buf;
}

/// @} DRIVERS
