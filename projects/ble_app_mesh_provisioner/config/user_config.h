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
// #define CFG_BLE_MESH_DBG
#define CFG_BLE_MESH_ROLE_PROVISIONER
#define CFG_BLE_MESH_MSG_API

/* For the Ali Mesh, don't need to used the gatt proxy and gatt provision */
#define CFG_BLE_MESH_GATT_PROXY
#define CFG_BLE_MESH_GATT_PROV
#define CFG_BLE_MESH_RELAY
//#define CFG_BLE_MESH_FRIEND
//#define CFG_BLE_MESH_LPN
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

#define CFG_BLE_MESH_MDL_GENC_OO
#define CFG_BLE_MESH_MDL_CLIENT
#define CFG_BLE_MESH_MDL_GENC
#define CFG_BLE_MESH_MDL_LIGHTC
#define CFG_BLE_MESH_MDL_GENC_LOC
#define CFG_BLE_MESH_MDL_LIGNTC_CTL
#define CFG_BLE_MESH_MDL_GENC_PROP
#define CFG_BLE_MESH_MDL_LIGNTC_HSL
#define CFG_BLE_MESH_MDL_LIGNTC_LN
#define CFG_BLE_MESH_MDL_LIGNTC_XYL
#define CFG_BLE_MESH_MDL_GENC_BAT
#define CFG_BLE_MESH_MDL_GENC_DTT
#define CFG_BLE_MESH_MDL_GENC_LVL
#define CFG_BLE_MESH_MDL_GENC_PLVL
#define CFG_BLE_MESH_MDL_GENC_POO

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
#define APP_SCNRSP_DATA        "\x0c\x08\x42\x4B\x33\x34\x33\x35\x2D\x47\x41\x54\x54\x30\x31" //BK3435-GATT"
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
#define POWER_ONOFF_ONLY         (0)


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


#define MAC78da07bcd71b 1
#define MAC78da07bcd71c 0
#define MAC78da07bcd71d 0
#define MAC38d2ca08ca20 0
#define MACf8a7632a93a3_FSD 0
#define MACf8a76324a49f 0

#endif /* _USER_CONFIG_H_ */
