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
#include "uart2.h"       

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
#include "mm_defines.h"
#include "mesh_general_api.h"
#include "mm_vendors_morenode.h"
#include "m_lay_int.h"

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
        if (loop % (85000 * 10) == 0)    ///*10为10s
        {
            //MESH_APP_PRINT_INFO("rwip_schedule %d\n", loop1++);
            MESH_APP_PRINT_INFO("L%d\n", loop1++);
        }
    }
}

#if 1
#define FLASH_ALI_DATA_ADDRESS    0x7E000

#define FLASH_ALI_MAC_ADDR_OFFSET 0
#define FLASH_ALI_PRODUCT_ID_ADDR_OFFSET 6
#define FLASH_ALI_SECRET_ADDR_OFFSET 10


#define FLASH_ALI_MAC_ADDR_LEN 6
#define FLASH_ALI_PRODUCT_ID_LEN 4
#define FLASH_ALI_SECRET_LEN 16

bool user_data_read_ali_mac(uint8_t *addr, uint8_t mode)
{

    uint8_t l_addr[FLASH_ALI_MAC_ADDR_LEN];
    flash_read_data(l_addr, FLASH_ALI_DATA_ADDRESS + FLASH_ALI_MAC_ADDR_OFFSET, FLASH_ALI_MAC_ADDR_LEN);

    if (mode)
    {
        memcpy(addr, l_addr, FLASH_ALI_MAC_ADDR_LEN);

    }
    else
    {
        for (int i = 0; i < FLASH_ALI_MAC_ADDR_LEN; i++)
        {
            addr[i] = l_addr[FLASH_ALI_MAC_ADDR_LEN - 1 - i];
        }
    }
    MESH_APP_PRINT_INFO("%s, addr = %s\n", __func__, mesh_buffer_to_hex(addr, FLASH_ALI_MAC_ADDR_LEN));
    return true;
}

bool user_data_contains_ali_data(void)
{
    uint8_t l_addr[FLASH_ALI_MAC_ADDR_LEN];
    flash_read_data(l_addr, FLASH_ALI_DATA_ADDRESS + FLASH_ALI_MAC_ADDR_OFFSET, FLASH_ALI_MAC_ADDR_LEN);

    MESH_APP_PRINT_INFO("l_addr %02x %02x %02x %02x %02x %02x\r\n", l_addr[0], l_addr[1], l_addr[2], l_addr[3], l_addr[4], l_addr[5]);
    if (l_addr[0]!=0xff ||l_addr[1]!=0xff||
            l_addr[2]!=0xff||l_addr[3]!=0xff||
            l_addr[4]!=0xff||l_addr[5]!=0xff )
    {
        return true;
    }
    else
    {
        return false;
    }
}
#endif


/************************************************************************************************/
/************************************************************************************************/

void bdaddr_env_init(void)
{
    struct bd_addr co_bdaddr;
   #if 0
    flash_read_data(&co_bdaddr.addr[0], 0x400e3, 6);
    if (co_bdaddr.addr[0]!=0xff ||co_bdaddr.addr[1]!=0xff||
            co_bdaddr.addr[2]!=0xff||co_bdaddr.addr[3]!=0xff||
            co_bdaddr.addr[4]!=0xff||co_bdaddr.addr[5]!=0xff )
    {
        memcpy(&co_default_bdaddr, &co_bdaddr, 6);
    }
    #endif
	
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

   #if 1 ///add 191230
     uint8_t ali_mac_addr[6];
    if (user_data_contains_ali_data())
    {
        if (user_data_read_ali_mac(ali_mac_addr, 0))
        {
            memcpy(&(co_default_bdaddr.addr[0]), ali_mac_addr, 6);
        }
    }
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

uint8_t g_net_tx_max   = M_ADV_NET_NB_TX_MAX;   ///add 2020 0116

uint8_t nvds_ctlset_value = 0;
uint8_t nvds_ctlset_value_len = 1;
	
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
    //uart_init(115200);
    uart_init(921600);
    uart_cb_register(uart_rx_handler);
#endif


#ifdef UART_2_TEST
    uart2_init(9600);
	uart_stack_register(uart2_printf);

     uart2_putchar("UART2 TEST-----------\r\n");

    UART2_PRINTF("start uart2---------------------\r\n");

#endif
		MESH_APP_PRINT_INFO("------------compiler DATE:%s---------------------------------------\r\n",__DATE__);
    MESH_APP_PRINT_INFO("|                                                                        |\r\n");
    MESH_APP_PRINT_INFO("----------------compiler TIME: %s -----------------\r\n", __TIME__);
    MESH_APP_PRINT_INFO("|                                                                        |\r\n");
    MESH_APP_PRINT_INFO("-------------------------------------------------------------------------\r\n");

    MESH_APP_PRINT_INFO("MESH param g_net_tx_max = %d\r\n",g_net_tx_max);

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

    #if 0
    if(!nvds_get(NVDS_TAG_MESH_CTLSET_VALUE,&nvds_ctlset_value_len,&nvds_ctlset_value))  ///add 0305
    {
    	MESH_APP_PRINT_INFO("****get nvds CTLSET value = 0x%x, = %d,*10 = %d\n",nvds_ctlset_value,nvds_ctlset_value,nvds_ctlset_value*10);
    }
    else
    {
    	MESH_APP_PRINT_INFO("****get nvds CTLSET value Error\n");
    }
    #endif
	
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
 		#if(POWER_3P5_ENABLE)
	 	bk3435_set_power(0x0f);
		//bk3435_singleWaveCfg(06,0x0f);  ///delete powerlevel func
		#endif
        	rw_app_enter();
    }


    while (1)
    {
        ;
    }

}


#if (UART_DRIVER)
//USE TEST
#define M_LAY_PROXY_CTL                      (2)
#define SEND_MSG 	MESH_APP_PRINT_INFO

//extern void Uart_api_on_off_test(uint8_t opcode,uint8_t parameter,uint16_t dst);

extern uint16_t user_models_publish_set(uint16_t app_keyid,m_lid_t m_lid, uint16_t addr,uint8_t period);

extern void mm_vendor_attr_indication(uint16_t attr_type,uint8_t len,uint8_t *value,uint16_t dst);

extern uint16_t m_tb_mio_get_publi_param(m_lid_t model_lid, uint16_t *p_addr, uint8_t *p_va_lid,                                  
									    uint8_t *p_app_key_lid, uint8_t *p_ttl, uint8_t *p_period,                                 
									    uint8_t *p_retx_params, uint8_t *p_friend_cred);

extern void app_onegroup_get_timer_set(uint32_t timer);

extern uint16_t onegroup_data[ONEGROUP_NODE_MAX][2];

extern uint16_t onegroup_node_calc;


uint8_t p_va_lid_test;                                
uint8_t p_app_key_lid_test; 
uint8_t p_ttl_test;
uint8_t p_period_test;                            
uint8_t p_retx_params_test;
uint8_t p_friend_cred_test;

uint16_t on_off_times_limit = 0;
uint8_t sys_tick_start = 0;//for app_gen_on_off_period_timer_handler
uint16_t max_times;//max test times,UART_START_GEN_ON_OFF_TEST_START initialize
uint16_t tim_reload = 0;//for unlimit gen_on_off test
uint32_t app_tmr_tick = 0;//each time call app_gen_on_off_period to increase app_tmr_tick
uint32_t _100x_app_tmr_tick = 0;
uint32_t lst_tst_tmr_tick = 0;

//static uint8_t gen_on_off_parameter = 0;

uint16_t uaddr = 0;
uint16_t gdst = 0;//0xC000;
uint16_t sub_dst = 0;
uint8_t ctlparam = 0;
uint16_t g_app_key_id = 0;


uint16_t send_nb = 100;  
uint8_t send_period = 5; ///500ms

uint16_t nodedst = 0;//0xC000;

uint16_t  othercmd = 0;

//static uint8_t  on_off_tst = 0;
static uint8_t  skip_tst = 0;
//static uint8_t  lpn_proxy_limit;

//chengyi add trspx test
uint8_t  trspx_tst = 0;
uint16_t trspx_times_limit = 0;
uint16_t trspx_max_time = 0;
/*KE_HEAP_EXTEND*/  uint8_t loop_index;
/*KE_HEAP_EXTEND*/  uint16_t prov_tradition_ble_interval;
/*KE_HEAP_EXTEND*/  uint8_t is_tradition_ble;

uint8_t vendor_data[5] ={0x00,0x82,0x83,0x84,0x85};
static void uart_rx_handler(uint8_t *buf, uint8_t len)
{
    #if 0 
    if (buf[0] == 0x01)
    {
        MESH_APP_PRINT_INFO("report on\n");
    }
    else if (buf[0] == 0x02)
    {
        MESH_APP_PRINT_INFO("report off\n");
    }
    #else
    #if 0
    uint8_t states = 0;
    #endif
    uint8_t i;
    void display_small_block_manager(void);
    #if 0
    static uint8_t wait_cnt = 0; 
    #endif
    uint8_t buffer[17];
    uint8_t upload_tmp_buf[10];
    UNUSED(upload_tmp_buf);
    uint16_t dst;
	uint8_t net_tx_count = buffer[6];
	
	uint8_t low_tx_count = 6;
	uint8_t ctlparam_len = 1;

	uint8_t model_lid = 0;
	
    //CHARLES_PRINTF("hci_ahi_msg_process_handler\r\n");	
    //read_uart_ringbuf_data(buffer,5);  ///close 191227

    for(i=0;i<17;i++)
    {
    	buffer[i] = buf[i];  ///add 191227
    }

    m_tb_state_set_relay_state(0,0);
    switch(buffer[0])//op code
    {
        case UART_GEN_ON_OFF_SINGLE://ex 0x01 c0 00 00 00
                dst = buffer[1];
                dst <<= 8;
                dst |= buffer[2];
                if(buffer[3] == OFF)
                {
//                    gen_on_off_parameter = OFF;
//                    Uart_api_on_off(OFF,dst);
                }
                else
                {
//                    gen_on_off_parameter = ON;
//                    Uart_api_on_off(ON,dst);
                }
        break;
		
        case UART_GEN_ON_OFF_CYCLE ://ex 0x02 c0 00 00 01 05

            MESH_APP_PRINT_INFO("UART_GEN_ON_OFF_CYCLE\n");
            gdst = buffer[1];
            gdst <<= 8;
            gdst |= buffer[2];

		send_nb = buffer[3];
		send_nb <<= 8;
		send_nb |= buffer[4];

		send_period = buffer[5];

		#if 0
		net_tx_count = buffer[6];

		if (net_tx_count)
		{
		    m_tb_state_set_net_tx_count(net_tx_count);
		}
		#endif
	        MESH_APP_PRINT_DEBUG("buffer[6] = 0x%x\n", buffer[6]);

	        MESH_APP_PRINT_INFO("UART_GEN_ON_OFF_CYCLE, 0x%x, 0x%x, 0x%x\n",gdst, send_nb, send_period);
		
	        user_models_publish_set(g_app_key_id,g_oo_mdl_lid, gdst, send_period);  ///add 191227 //publish set ///c000 -> gdst

            break;

	case UART_START_GEN_ON_OFF_TEST_END : //ex 0x05 C0 00 00 00 
	
	nodedst = buffer[1];
	nodedst <<= 8;
	nodedst |= buffer[2];

	MESH_APP_PRINT_INFO("UART_START_GEN_ON_OFF_TEST_END\n");

	mm_vendor_attr_indication(0x88,5,vendor_data,nodedst);
	break;

	case UART_VENDOR_TEST_START://ex 0x18 ///2020 0108
        {
	     #if 0
            for(i=0; i<10; i = i + 2)
            {
                upload_tmp_buf[i] = (uint8_t)(1000 & 0xff00) >> 8;
                upload_tmp_buf[i+1] = (uint8_t)(1000 & 0x00ff);
            }

            upload_tmp_buf[2] = buffer[1];
            upload_tmp_buf[3] = buffer[2];
	     #endif
	     MESH_APP_PRINT_INFO("UART_VENDOR_TEST_START\n");
		gdst = buffer[1];            

		gdst <<= 8;
		gdst |= buffer[2];

		send_nb = buffer[3];
		send_nb <<= 8;
		send_nb |= buffer[4];

		send_period = buffer[5];

		othercmd = buffer[6];
		othercmd <<= 8;
		othercmd |= buffer[7];

		net_tx_count = buffer[8];

		low_tx_count = buffer[9];
		
        if (net_tx_count)
        {
            m_tb_state_set_net_tx_count(net_tx_count);
        }

	if(low_tx_count)  ///add 0306
	{
		m_lay_ltrans_set_nb_set_retrans(low_tx_count);
	}

	MESH_APP_PRINT_INFO("****net_tx_count = %d,low_tx_count = %d\n",m_tb_state_get_net_tx_count(),m_lay_ltrans_get_nb_retrans());

        uint8_t   nb_tx;
        uint16_t  intv_slots;
		m_tb_state_get_net_tx_params(&nb_tx, &intv_slots);
		m_tb_store_config(0);
		MESH_APP_PRINT_INFO("UART_VENDOR_TEST_START 0x%x, 0x%x ,0x%x\n",gdst,send_nb,send_period);
		MESH_APP_PRINT_DEBUG("++++++++++ trans_count %d, nb_tx %d, intv_slots %d++++++++++++++++++++++++\n", 
		                   net_tx_count, nb_tx, intv_slots);
		user_models_publish_set(g_app_key_id,g_vdr_lid,gdst, send_period);
														///100     ->	0x64
														///1000   ->	0x3e8
														///10000 ->   0x2710
       }
        break;

	case UART_VENDOR_MOREGROUP_SEND: 	///0x28 
	send_nb = buffer[3];
	send_nb <<= 8;
	send_nb |= buffer[4];

	send_period = buffer[5];

	net_tx_count = buffer[6];  
	
	low_tx_count = buffer[7]; ///add 0304

	///add 0310
	// if (net_tx_count)
        {
            m_tb_state_set_net_tx_count(0x03);
        }

	//if(low_tx_count)  ///add 0306
	{
		m_lay_ltrans_set_nb_set_retrans(0x01);
	}
	
	m_tb_mio_get_publi_param(g_vdr_lid,&gdst,
								&p_va_lid_test,                             
								&p_app_key_lid_test,
								&p_ttl_test,
								&p_period_test,                            
								&p_retx_params_test,
								&p_friend_cred_test);
	
	MESH_APP_PRINT_INFO("UART_VENDOR_MOREGROUP_SEND GROUP ADDR = 0x%x\n", gdst);
	if(gdst >= 0xc000)
	{
		//user_models_publish_set(g_vdr_lid, gdst,send_period); ///mask 2020 0111
		//user_models_publish_set_vendor(g_oo_mdl_lid,0xc000);
	}
	
	#if 0 
    if (net_tx_count)
    {
        m_tb_state_set_net_tx_count(net_tx_count);
    }
	#endif
	//m_tb_store_config(0);
	vendor_data[0] = buffer[3];
	vendor_data[1] = buffer[4];
	vendor_data[2] = buffer[5];
	vendor_data[3] = net_tx_count; ///add 0216
	vendor_data[4] = low_tx_count; ///add 0304
	
	MESH_APP_PRINT_INFO("%s, vendor_data[4] = %d, low_tx_count = %d \n", __func__, vendor_data[4], low_tx_count);
	mm_vendor_attr_indication_publish(VENDOR_MOREGROUP_SEND_ATTRTYPE, 5, vendor_data, gdst);  ///add 2020 0118
	break;

	case UART_VENDOR_ONEGROUP_GET:	///0x29
	
	m_tb_store_config(0);

	gdst = buffer[1];            

	gdst <<= 8;
	gdst |= buffer[2];

	send_nb = buffer[3];
	send_nb <<= 8;
	send_nb |= buffer[4];
	
	MESH_APP_PRINT_INFO("UART_VENDOR_ONEGROUP_GET GROUP ADDR = 0x%x\n", gdst);
  	#if 1
	if(gdst >= 0xc000)
	{
		//mm_vendor_attr_indication_publish(VENDOR_ONEGROUP_GET_ATTRTYPE,5,gdst); ///for debug test
		app_onegroup_get_timer_set(500);  ///add 2020 0114 //loop publish
	}
	#endif
	break;

	case UART_VENDOR_ONEGROUP_GET_STOP:	///0x2a
	MESH_APP_PRINT_INFO("UART_VENDOR_ONEGROUP_GET_STOP\n");
	#if 1
	for(uint8_t i = 0; i< ONEGROUP_NODE_MAX; i++)
	{
		MESH_APP_PRINT_INFO("u_addr = 0x%x, rx num = 0x%x\n", onegroup_data[i][0], onegroup_data[i][1]);
	}
	for(uint8_t j = 0; j< ONEGROUP_NODE_MAX; j++)  ///clear array value
	{
		onegroup_data[j][0] =  0;
		onegroup_data[j][1] =  0;
	}
	onegroup_node_calc = 0;
	app_onegroup_get_timer_set(0);
	#endif
	break;

	//add 2020 0119
	case UART_ONOFF_MOREGROUP_SEND:
    send_nb = buffer[3];
	send_nb <<= 8;
	send_nb |= buffer[4];

	send_period = buffer[5];
	m_tb_mio_get_publi_param(g_oo_mdl_lid,&gdst,
								&p_va_lid_test,                             
								&p_app_key_lid_test,
								&p_ttl_test,
								&p_period_test,                            
								&p_retx_params_test,
								&p_friend_cred_test);

    net_tx_count = buffer[6];
	#if 0 
    if (net_tx_count)
    {
        m_tb_state_set_net_tx_count(net_tx_count);
    }
	#endif
	MESH_APP_PRINT_INFO("UART_ONOFF_MOREGROUP_SEND GROUP ADDR = 0x%x\n", gdst);
	if(gdst >= 0xc000)
	{
		//user_models_publish_set(g_oo_mdl_lid, gdst,send_period); ///mask 2020 0111
		//user_models_publish_set_vendor(g_oo_mdl_lid,0xc000);
	}
	
	//m_tb_store_config(0);
	vendor_data[0] = buffer[3];
	vendor_data[1] = buffer[4];
	vendor_data[2] = buffer[5];
	vendor_data[3] = net_tx_count;
	mm_vendor_attr_indication_publish(ONOFF_MOREGROUP_SEND_ATTRTYPE,5,vendor_data,gdst);  ///add 2020 0118	
		
	break;

	case UART_ONOFF_ONEGROUP_GET:
	m_tb_store_config(0);

	gdst = buffer[1];            

	gdst <<= 8;
	gdst |= buffer[2];

	send_nb = buffer[3];
	send_nb <<= 8;
	send_nb |= buffer[4];
	
	MESH_APP_PRINT_INFO("UART_ONOFF_ONEGROUP_GET GROUP ADDR = 0x%x\n", gdst);
	if(gdst >= 0xc000)
	{
		//mm_vendor_attr_indication_publish(ONOFF_ONEGROUP_GET_ATTRTYPE,5,gdst); ///for debug test
		app_ONOFF_onegroup_get_timer_set(500);  ///add 2020 0216 //loop publish
	}
	break;

	case UART_ONOFF_ONEGROUP_GET_STOP:
	MESH_APP_PRINT_INFO("UART_ONOFF_ONEGROUP_GET_STOP\n");
	#if 1
	for(uint8_t i = 0; i< ONEGROUP_NODE_MAX;i++)
	{
		MESH_APP_PRINT_INFO("u_addr = 0x%x,   rx num = 0x%x\n",onegroup_data[i][0],onegroup_data[i][1]);
	}
	app_ONOFF_onegroup_get_timer_set(0);  ///after get change,open this sentence
	#endif

	break;

	case UART_VENDOR_TEST_GET_COUNT://ex 0x19 00 0N///2020 0108
            MESH_APP_PRINT_INFO("UART_VENDOR_TEST_GET_COUNT\n");
	     #if 0
            for(i=0; i<10; i++)
            {
                upload_tmp_buf[i] = 0;
            }
	     #endif
		 
            dst = buffer[1];
            dst <<= 8;
            dst |= buffer[2];
	     mm_vendor_attr_indication(VENDOR_ONENODE_GET_ATTRTYPE,5,vendor_data,dst);
            break;

	case UART_ONOFF_SEND_STOP:	///0x26,onoff send STOP
	       MESH_APP_PRINT_INFO("UART_ONOFF_SEND_STOP\n");
		   m_tb_mio_get_publi_param(g_vdr_lid,&gdst,     
								&p_va_lid_test,                             
								&p_app_key_lid_test,
								&p_ttl_test,
								&p_period_test,                            
								&p_retx_params_test,
								&p_friend_cred_test);
		MESH_APP_PRINT_DEBUG("publish_clear gdst = 0x%x\n",gdst);
	 	user_models_publish_clear(g_app_key_id,g_oo_mdl_lid, gdst);  
	break;

	case UART_VENDOR_SEND_STOP:	///0X27,vendor send STOP
		MESH_APP_PRINT_INFO("UART_VENDOR_SEND_STOP\n");
		m_tb_mio_get_publi_param(g_vdr_lid,&gdst,     
								&p_va_lid_test,                             
								&p_app_key_lid_test,
								&p_ttl_test,
								&p_period_test,                            
								&p_retx_params_test,
								&p_friend_cred_test);
		MESH_APP_PRINT_DEBUG("publish_clear gdst = 0x%x\n",gdst);
		user_models_publish_clear(g_app_key_id,g_vdr_lid, gdst);  
	break;
				
        case UART_STOP_GEN_ON_OFF_CYCLE ://ex 0x03
                MESH_APP_PRINT_INFO("UART_STOP_GEN_ON_OFF_CYCLE\n");
				m_tb_mio_get_publi_param(g_vdr_lid,&gdst,         
								&p_va_lid_test,                              
								&p_app_key_lid_test,
								&p_ttl_test,
								&p_period_test,                            
								&p_retx_params_test,
								&p_friend_cred_test);
		 MESH_APP_PRINT_DEBUG("publish_clear gdst = 0x%x\n",gdst);
        	 user_models_publish_clear(g_app_key_id,g_oo_mdl_lid, gdst);  

        break;
				
        case UART_START_GEN_ON_OFF_TEST_START ://ex 0x04 C0 00 00 32  //50
                                               //ex 04 C0 00 01 F4  //500
                                               //ex 04 C0 00 03 E8  //1000
        {
            uint16_t pack_num;
            //SEND_MSG("UART_START_GEN_ON_OFF_TEST_SET\r\n" );
            dst = buffer[1];
            dst <<= 8;
            dst |= buffer[2];
            //Uart_api_on_off_test(M_FND_GENONOFFS_OPCODE_ONOFF_TEST_START,1,dst);

	    
           // Uart_api_on_off_test(MM_MSG_GEN_OO_TEST_START,1,dst);
			
            pack_num = buffer[3];
            pack_num <<= 8;
            pack_num |= buffer[4];
            max_times = on_off_times_limit = pack_num;
        }
        break;
	 case UART_DUMP_MEM ://ex 0x06
            //display_small_block_manager();
        break;
			
        #if(MEM_REQ_RSP)
        case UART_GET_NODE_MEM ://ex 07 C0 00 00 00
        {
            //CHARLES_PRINTF("case UART_GET_NODE_MEM\r\n" );
            //calc_tim(START);
            dst = buffer[1];
            dst <<= 8;
            dst |= buffer[2];
//            Uart_api_on_off_test(M_FND_GENONOFFS_OPCODE_MEM_REQ,1,dst);
        }
        break;
        #endif
		
        case UART_TMR_ON : //ex 0x08 00 00 00 00
        {
            //SEND_MSG("case UART_TMR_ON\r\n" );
            app_tmr_tick = 0;
            sys_tick_start = 1;
            //reset_sys_tick_timer(1);
            wdt_enable(0xFFFF);
            //ke_timer_set(APP_GEN_ON_OFF_PERIOD_TIMER,TASK_APP,20);//fix 20 * 10 ms = 200ms /tick
            
        }
        break;
		
        case UART_TMR_OFF : //ex 0x09 00 00 00 00
        {
            //CHARLES_PRINTF("case UART_TMR_OFF\r\n" );
            sys_tick_start = 0;
            wdt_disable();
            //ke_timer_clear(APP_SYS_TICK_TIMER,TASK_APP);  ///APP_SYS_TICK_TIMER ??? ///191227
        }
        break;
		
        case UART_SEND_NOTIFY : //ex 0x0A for FF81
        {
            
            
            for(i=0;i<10;i++)
            {
//                upload_tmp_buf[i] = i;
            }
            #if defined(CFG_PRF_FFF0)
//            app_fff1_send_lvl(upload_tmp_buf, 10);//send 0xFF91 MESH GATT 透传上行特性
            #endif
            #if defined(CFG_PRF_FF80)
            app_ff81_send_lvl(upload_tmp_buf, 10);//send 0xFF81 MESH GATT 控制上行特性
            #endif
        }
        break;
        //chengyi add for midea
        case UART_GEN_ON_OFF_SET://ex 0x0B
            dst = buffer[1];
            dst <<= 8;
            dst |= buffer[2];
			
            //uart_genonoff_client_test(0x02, 0x00, dst);
            
            break;
        case UART_GEN_ON_OFF_SET_NO_ACK://ex 0x0C
            dst = buffer[1];
            dst <<= 8;
            dst |= buffer[2];
			
            //uart_genonoff_client_test(0x03, 0x01, dst);
			
            break;
        case UART_GEN_ON_OFF_GET://ex 0x0D
            dst = buffer[1];
            dst <<= 8;
            dst |= buffer[2];
			
            //uart_genonoff_client_test(0x01, 0x00, dst);
			
            break;
        case UART_GEN_LEVEL_SET://ex 0x0E
            #if (MODEL_GEN_LEVEL_SUPPORT)
            dst = buffer[1];
            dst <<= 8;
            dst |= buffer[2];
            //uart_gen_level_client_test(0x06,0x11,dst); 
            #else
            SEND_MSG("Not SUPPORT UART_GEN_LEVEL_SET CMD\r\n");
            #endif
        break;

        case UART_GEN_LEVEL_SET_NO_ACK://ex 0x0F
            #if (MODEL_GEN_LEVEL_SUPPORT)
            dst = buffer[1];
            dst <<= 8;
            dst |= buffer[2];
            //uart_gen_level_client_test(0x07,0x12,dst); 
            #else
            //SEND_MSG("Not SUPPORT UART_GEN_LEVEL_SET_NO_ACK CMD\r\n");
            #endif
        break;
        case UART_GEN_LEVEL_GET://ex 0x10
            #if (MODEL_GEN_LEVEL_SUPPORT)
            dst = buffer[1];
            dst <<= 8;
            dst |= buffer[2];
            //uart_gen_level_client_test(0x05,0x11,dst); 
            #else
            //SEND_MSG("Not SUPPORT UART_GEN_LEVEL_GET CMD\r\n");
            #endif
        break;
        case UART_VENDOR_COMMAND_SET ://ex 0x11 
            dst = buffer[1];
            dst <<= 8;
            dst |= buffer[2];
            //uart_vendor_client_test(0xD0,0x01,dst);
        break;
        case UART_VENDOR_COMMAND_SET_NO_ACK://ex 0x12
            dst = buffer[1];
            dst <<= 8;
            dst |= buffer[2];
            //uart_vendor_client_test(0xD1,0x01,dst);
        break;
        case UART_VENDOR_COMMAND_GET ://ex 0x13
            dst = buffer[1];
            dst <<= 8;
            dst |= buffer[2];
            //uart_vendor_client_test(0xD2,0x00,dst);
        break;
        case UART_VENDOR_COMMAND_NOTIFY ://ex 0x14
            dst = buffer[1];
            dst <<= 8;
            dst |= buffer[2];
           // uart_vendor_test(0xE1, dst);        
        break;
        case UART_VENDOR_COMMAND_MESSAGE_PASS ://ex 0x15
            for(i=0; i<10; i++)
            {
//                upload_tmp_buf[i] = i;
            }
            
            loop_index = (loop_index % 255) + 1;
            
//            upload_tmp_buf[9] = loop_index;
            
            dst = buffer[1];
            dst <<= 8;
            dst |= buffer[2];            
            //uart_vendor_trspx_test(0xC0,upload_tmp_buf,dst);
        break;
        case UART_VENDOR_TEST_CLEAR://ex 0x16
            for(i=0; i<10; i++)
            {
                upload_tmp_buf[i] = i;
            }
            
            dst = buffer[1];
            dst <<= 8;
            dst |= buffer[2];
            //uart_vendor_trspx_test(0xC1,upload_tmp_buf,dst);
        break;
        case UART_VENDOR_TEST_SET://ex 0x17 00 01 03 e8
        {
            uint16_t pack_num;
            m_tb_state_set_relay_state(0,0);
            for(i=0; i<10; i++)
            {
                upload_tmp_buf[i] = i;
            }
            upload_tmp_buf[0] = buffer[1];
            upload_tmp_buf[1] = buffer[2];
            upload_tmp_buf[2] = buffer[3];
            upload_tmp_buf[3] = buffer[4];
            pack_num = buffer[3];
            pack_num <<= 8;
            pack_num |= buffer[4];
            dst = buffer[1];
            dst <<= 8;
            dst |= buffer[2];
            SEND_MSG("UART_VENDOR_TEST_SET,dst(X'%x)(D'%d)\r\n",dst,dst);
            trspx_times_limit = trspx_max_time = pack_num;
            //uart_vendor_trspx_test(0xC2,upload_tmp_buf, dst);
        }
        break;
		
        case UART_INTER_LOW_POWER_MODE://ex 0x1a
           // uart_test_lpn_function(0x00000000, 0x00000000, 0x00, 0x00, 0x00, 0x00, false);
            break;
        case UART_LOW_POWER_MODE_EXIT://ex 0x1b
           // app_task_clean_lpn_set_timer();
           // uart_test_lpn_function_exit();
           // SEND_MSG("proxy(2)\r\n");
            m_api_proxy_ctrl(M_LAY_PROXY_CTL);//uart exit lpn mode 
            //m_bearer_adv_update(M_BEARER_ACT_RESUME);  ///M_BEARER_ACT_RESUME ???
            break;
        case UART_BEARER_CTRL_SCAN://ex 0x1c
            //CHARLES_PRINTF("case UART_BEARER_CTRL_SCAN:(%x)\r\n",buffer[1]);
            if(buffer[1] == 0)
            {
                //m_bearer_adv_update(M_BEARER_ACT_PAUSE);
            }
            else 
            if(buffer[1] == 1)
            {
               
                //m_bearer_adv_update(M_BEARER_ACT_RESUME);
            }
           
        break;
        case UART_CLR_LPN_SND_STR_CNT ://ex 0x1d
        {
            // void clear_friend_to_lpn_snd_store_cnt(void);
            // clear_friend_to_lpn_snd_store_cnt();
        }
        break;
        case UART_SET_RELAY://ex 0x1e
        {
            if(buffer[1] == 0)
            {
                m_tb_state_set_relay_state(0,0);
            }
            else 
            if(buffer[1] == 1)
            {
                m_tb_state_set_relay_state(1,0);
            }
        }
        break;
        case UART_STOP_ADV://ex 0x1f
//            m_prov_bearer_gatt_stop();
        break;
        case UART_START_ADV://ex 0x20 00 01 07 d0
        {

     #if(CLIENT_WHO == CLIENT_MEDIA_SUPPORT)             
            uint16_t interval = 0x00;
            
         
            is_tradition_ble = buffer[2];          
            interval = buffer[3];
            interval <<= 8;
            interval |= buffer[4];
          
            if(is_tradition_ble)
            {
                prov_tradition_ble_interval = interval;
                //set_traditioin_parameter(true, interval);
                
                //app_tradition_ble_timer_set();
            }
            else
            {
                prov_tradition_ble_interval = 0x00;
                //set_traditioin_parameter(false, 0x00);
                //app_tradition_ble_timer_remove();
                //SEND_MSG("m_tb_state_get_prov_state X'%x\r\n", m_tb_state_get_prov_state());
                if (m_tb_state_get_prov_state() == M_TB_STATE_PROV_STATE_BEING_PROV)
                {
//                    m_prov_bearer_gatt_start();
                }
            }
       #endif
	   
       #if(CLIENT_WHO == CLIENT_BEKEN_STANDARD)
       //SEND_MSG("not support UART_START_ADV\r\n");       
       #endif
	   
        }    
        break;


        case UART_PROXY_CTRL://ex 0x21
            if(buffer[1] == 0)
            {
                SEND_MSG("proxy(0)\r\n");
                m_api_proxy_ctrl(0);//uart proxy ctrl
//                m_prov_bearer_gatt_stop();
            }
            else 
            if(buffer[1] == 1)
            {
                SEND_MSG("proxy(2)\r\n");
                m_api_proxy_ctrl(M_LAY_PROXY_CTL);//uart proxy ctrl
                if(buffer[2] != 0)
                {
//                    lpn_proxy_limit = 10;//10 sec
                }
            }
            
        break;
		
        case UART_CONN_UPDATE://ex 0x22
//             ke_timer_set(APP_PARAM_UPDATE_REQ_IND,TASK_APP,100);//10 * 10ms = 100ms 
        break;

        case UART_DISABLE_UART_INT://ex 0x23
            REG_AHB0_ICU_INT_ENABLE &= (~(0x01 << 5));
        break;
		
        case UART_TEST_UNLIMIT_SET ://ex 0x24
             SEND_MSG("Gen_on_off_test_unlimit\r\n");
              tim_reload = 1;
        break;
		
        case UART_HW_SCAN_ABORT ://ex 0x25
        {
		//void remove_active_tst_type(void);
		void Delay_ms(int num);
		//ble_rwblecntl_set(ble_rwblecntl_get() | BLE_SCAN_ABORT_BIT); ///BLE_SCAN_ABORT_BIT ？？？
		Delay_ms(3);
		// Initialize buffers
		// SEND_MSG("em_buf_init\r\n");         
		//em_buf_init();
		//            on_off_tst = 0;
		// remove_active_tst_type();
            
        }
        break;
		
#if UART_CMD_PROV_EN
	case UART_SET_UADDR: 		///add 0310
	uaddr = buffer[1];            
	uaddr <<= 8;
	uaddr |= buffer[2];
		
	MESH_APP_PRINT_INFO("UART_SET_UADDR uaddr = 0x%x\n",uaddr);	
	if(uaddr > 0)
	{
		m_tb_mio_set_prim_addr(uaddr);   
	} 
	m_tb_store_config(3);
	break;

	case UART_SET_ONOFF_PARAM:
	case UART_SET_VENDOR_PARAM: ///add 0310
	//opcode(1byte) + uaddr(2byte) + gdst(2byte) + sub_dst(2byte) + ctlparam(1byte)
	if(buffer[0] == UART_SET_VENDOR_PARAM)
	{
		model_lid = g_vdr_lid;
	}
	else 
	{
		model_lid = g_oo_mdl_lid;
	}
	
	       uaddr = buffer[1];            
		uaddr <<= 8;
		uaddr |= buffer[2];

		gdst = buffer[3];
		gdst <<= 8;
		gdst |= buffer[4];

		sub_dst = buffer[5];
		sub_dst <<= 8;
		sub_dst |= buffer[6];
		
		ctlparam = buffer[7];
		
      	MESH_APP_PRINT_INFO("UART_SET_MODEL_PARAM uadd = 0x%x,Paddr = 0x%x,Saddr = 0x%x,Ctl Param = 0x%x\n",uaddr,gdst,sub_dst,ctlparam);  
        MESH_APP_PRINT_INFO("buffer: %s\n", mesh_buffer_to_hex(buffer, sizeof(buffer)));
        for (int i = 0; i < sizeof(buffer); i++)
        {
            MESH_APP_PRINT_INFO("buffer[%d] = %x\n", i, buffer[i]);
        }
	send_nb = 0;
	
	if(uaddr > 0)
	{
		m_tb_mio_set_prim_addr(uaddr);   
	}

	if(gdst >= 0xc000)
	{
		user_models_publish_set(g_app_key_id,model_lid, gdst,05);    ///0313
	}

	if(sub_dst >= 0xc000)
	{
       		user_models_subs_group_addr(model_lid, sub_dst);  
	}

	if(ctlparam > 0)
	{
		if(!nvds_put(NVDS_TAG_MESH_CTLSET_VALUE,1,&ctlparam)) ///add 0305
		{
			MESH_APP_PRINT_INFO("nvds store CTLSET_VALUE OK!");
		}
		else
		{
			MESH_APP_PRINT_INFO("nvds store CTLSET_VALUE Error!");
		}
	}
	m_tb_store_config(3);
	break;


		

	case UART_SET_CTL_PARAM:
		
	ctlparam = buffer[1];	
	MESH_APP_PRINT_INFO("UART_SET_CTL_PARAM PARAM = 0x%x\n",ctlparam);
	
	if(ctlparam > 0)
	{
		if(!nvds_put(NVDS_TAG_MESH_CTLSET_VALUE,1,&ctlparam)) ///add 0305
		{
			MESH_APP_PRINT_INFO("nvds store CTLSET_VALUE OK!");
		}
		else
		{
			MESH_APP_PRINT_INFO("nvds store CTLSET_VALUE Error!");
		}
	}
	m_tb_store_config(3);
	break;
#endif

        //end
        default:
        break;
    }
	
       memset(buffer,0,10);
	#endif ///add 191227
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
    if ((REG_PL_RD(STACK_BASE_UNUSED) != BOOT_PATTERN_UNUSED))
    {
        while (1)
        {
            uart_putchar("Stack_Integrity_Check STACK_BASE_UNUSED fail!\r\n");
        }
    }

    if ((REG_PL_RD(STACK_BASE_SVC) != BOOT_PATTERN_SVC))
    {
        while (1)
        {
            uart_putchar("Stack_Integrity_Check STACK_BASE_SVC fail!\r\n");
        }
    }

    if ((REG_PL_RD(STACK_BASE_FIQ) != BOOT_PATTERN_FIQ))
    {
        while (1)
        {
            uart_putchar("Stack_Integrity_Check STACK_BASE_FIQ fail!\r\n");
        }
    }

    if ((REG_PL_RD(STACK_BASE_IRQ) != BOOT_PATTERN_IRQ))
    {
        while (1)
        {
            uart_putchar("Stack_Integrity_Check STACK_BASE_IRQ fail!\r\n");
        }
    }

}

void stack_gpio_triger(uint8_t gpio_num)
{
    gpio_triger(gpio_num);
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
   // rom_env.gpio_triger = stack_gpio_triger;
}      
         
/// @} DRIVERS
