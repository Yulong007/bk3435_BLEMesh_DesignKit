/**
 *******************************************************************************
 *
 * @file user_config.h
 *
 * @brief Application configuration definition
 *
 * Copyright (C) RivieraWaves 2009-2016
 *
 *******************************************************************************
 */

#ifndef _USER_CONFIG_H_
#define _USER_CONFIG_H_


/*******************************************************************************
 *#############################################################################*
 *                              MESH MACRO CTRL                         *
 *#############################################################################*
 *******************************************************************************/

// Open the mesh log
#define CONFIG_MESH_LOG_OUTPUT

#define CFG_BLE_MESH_MSG_API

/* For the Ali Mesh, don't need to used the gatt proxy and gatt provision */
#define CFG_BLE_MESH_GATT_PROXY
#define CFG_BLE_MESH_GATT_PROV
#define CFG_BLE_MESH_RELAY
#undef CFG_BLE_MESH_FRIEND
#undef CFG_BLE_MESH_LPN
#define CFG_BLE_MESH_STORAGE_NVDS

#define CFG_BLE_MESH_MDL_SERVER


#define CFG_BLE_MESH_MDL_GENS
#define CFG_BLE_MESH_MDL_GENS_OO
#undef CFG_BLE_MESH_MDL_GENS_DTT
#undef CFG_BLE_MESH_MDL_GENS_POO
#undef CFG_BLE_MESH_MDL_GENS_PLVL
#undef CFG_BLE_MESH_MDL_GENS_LVL
#undef CFG_BLE_MESH_MDL_GENS_LOC
#undef CFG_BLE_MESH_MDL_GENS_PROP

#define CFG_BLE_MESH_MDL_LIGHTS
#define CFG_BLE_MESH_MDL_LIGNTS_LN
#define CFG_BLE_MESH_MDL_LIGNTS_CTL
#define CFG_BLE_MESH_MDL_LIGNTS_HSL
 
#undef CFG_BLE_MESH_MDL_CLIENT
#undef CFG_BLE_MESH_MDL_GENC
#undef CFG_BLE_MESH_MDL_LIGHTC
 
#undef CFG_MESH_MEM_TB_BUF_DBG
 

/******************************************************************************
 *############################################################################*
 *                             SYSTEM MACRO CTRL                              *
 *############################################################################*
 *****************************************************************************/

//如果需要使用GPIO进行调试，需要打开这个宏
#define GPIO_DBG_MSG                    0
//UART使能控制宏
#define UART_PRINTF_EN                  1
//蓝牙硬件调试控制
#define DEBUG_HW                        1





/*******************************************************************************
 *#############################################################################*
 *                              APPLICATION MACRO CTRL                         *
 *#############################################################################*
 *******************************************************************************/

//连接参数更新控制
#define UPDATE_CONNENCT_PARAM           1

//最小连接间隔
#define BLE_UAPDATA_MIN_INTVALUE        20
//最大连接间隔
#define BLE_UAPDATA_MAX_INTVALUE        40
//连接Latency
#define BLE_UAPDATA_LATENCY             0
//连接超时
#define BLE_UAPDATA_TIMEOUT             600


//设备名称
#define APP_DFLT_DEVICE_NAME           ("BK3435-GATT01")


//广播包UUID配置
#define APP_FFF0_ADV_DATA_UUID        "\x03\x03\xF0\xFF"
#define APP_FFF0_ADV_DATA_UUID_LEN    (4)

//扫描响应包数据
#define APP_SCNRSP_DATA        "\x0e\x08\x42\x4B\x33\x34\x33\x35\x2D\x47\x41\x54\x54\x30\x31" //BK3435-GATT"
#define APP_SCNRSP_DATA_LEN     (15)


//广播参数配置
/// Advertising channel map - 37, 38, 39
#define APP_ADV_CHMAP           (0x07)
/// Advertising minimum interval - 100ms (160*0.625ms)
#define APP_ADV_INT_MIN         (80)
/// Advertising maximum interval - 100ms (160*0.625ms)
#define APP_ADV_INT_MAX         (80)
/// Fast advertising interval
#define APP_ADV_FAST_INT        (32)


#define UNPROV_CONN_EN           (1)
#define PROXY_KEEP_CONN_EN       (1)
#define UNPROV_TIMEOUT_ADV       (1)
#define POWER_ONOFF_ONLY         (1)

enum uart_command
{
  UART_GEN_ON_OFF_SINGLE = 1,  
  UART_GEN_ON_OFF_CYCLE,///0x02
  UART_STOP_GEN_ON_OFF_CYCLE,///0x03
  UART_START_GEN_ON_OFF_TEST_START,///0x04
  UART_START_GEN_ON_OFF_TEST_END,///0x05
  UART_DUMP_MEM,
  UART_GET_NODE_MEM,
  UART_TMR_ON,
  UART_TMR_OFF,
  UART_SEND_NOTIFY, ///0a
  UART_GEN_ON_OFF_SET,
  UART_GEN_ON_OFF_SET_NO_ACK,
  UART_GEN_ON_OFF_GET,
  UART_GEN_LEVEL_SET,
  UART_GEN_LEVEL_SET_NO_ACK, ///0f
  UART_GEN_LEVEL_GET,
  UART_VENDOR_COMMAND_SET, ///
  UART_VENDOR_COMMAND_SET_NO_ACK,
  UART_VENDOR_COMMAND_GET,
  UART_VENDOR_COMMAND_NOTIFY,
  UART_VENDOR_COMMAND_MESSAGE_PASS,
  UART_VENDOR_TEST_CLEAR, ///16
  UART_VENDOR_TEST_SET,///17
  
  UART_VENDOR_TEST_START,///18   ///0210 11:22
  
  UART_VENDOR_TEST_GET_COUNT,///19
  UART_INTER_LOW_POWER_MODE,////1a
  UART_LOW_POWER_MODE_EXIT,
  UART_BEARER_CTRL_SCAN,///1c
  UART_CLR_LPN_SND_STR_CNT,	///1d	//clear_friend_to_lpn_snd_store_cnt
  UART_SET_RELAY,///1e
  UART_STOP_ADV,///1f
  UART_START_ADV,///20
  UART_PROXY_CTRL,///21
  UART_CONN_UPDATE,///0x22
  UART_DISABLE_UART_INT,		///0x23
  UART_TEST_UNLIMIT_SET,		///0x24
  UART_HW_SCAN_ABORT,			///0x25
  UART_ONOFF_SEND_STOP,   		///0X26
  UART_VENDOR_SEND_STOP, 		///0X27
  
  UART_VENDOR_MOREGROUP_SEND, ///0X28
  UART_VENDOR_ONEGROUP_GET,       ///0X29
  UART_VENDOR_ONEGROUP_GET_STOP, ///0X2A

  UART_ONOFF_MOREGROUP_SEND, ///0X2B
  UART_ONOFF_ONEGROUP_GET,       ///0X2C
  UART_ONOFF_ONEGROUP_GET_STOP, ///0X2D

  UART_VENDOR_RSP_SEND, ///0X2E

  UART_SET_APPKEY = 0x40,		///0X40
  UART_SET_DEVKEY,				///0X41
  UART_SET_NETKEY,				///0X42
  UART_SET_UADDR,			       ///0X43
  UART_SET_MODEL_APPKEY,		///0X44
  UART_SET_VENDOR_PARAM,	       ///0X45
  UART_SET_ONOFF_PARAM,	       ///0X46
  UART_SET_CTL_PARAM 			///0X47
};

enum UART_GEN_ON_OFF_parm
{
    OFF,
    ON,
};


/*******************************************************************************
 *#############################################################################*
 *                              DRIVER MACRO CTRL                              *
 *#############################################################################*
 ******************************************************************************/

//DRIVER CONFIG
#define UART_DRIVER                     1
#define GPIO_DRIVER                     1
#define AUDIO_DRIVER                    0
#define RTC_DRIVER                      0
#define ADC_DRIVER                      0
#define I2C_DRIVER                      0
#define PWM_DRIVER                      1


#define MAC78da07bcd71b 1 //yushi   
#define MAC78da07bcd71c 0 //mengkou  
#define MAC78da07bcd71d 0 //chufang  


#define GEN_ONOFF_TX_MAX  		100    

#define ONEGROUP_NODE_MAX 		10

//#define	M_ADV_NET_NB_TX_MAX  		3

//#define UART_CMD_PROV_EN         0


#endif /* _USER_CONFIG_H_ */
