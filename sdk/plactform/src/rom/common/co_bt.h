/**
 ****************************************************************************************
 *
 * @file co_bt.h
 *
 * @brief This file contains the common Bluetooth defines, enumerations and structures
 *        definitions for use by all modules in BLE stack.
 *
 * Copyright (C) RivieraWaves 2009-2015
 *
 *
 ****************************************************************************************
 */

#ifndef CO_BT_H_
#define CO_BT_H_

/**
 ****************************************************************************************
 * @addtogroup CO_BT Common Bluetooth defines
 * @ingroup COMMON
 *
 * @brief Common Bluetooth definitions and structures.
 *
 * @{
 ****************************************************************************************
 */


/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include <stdint.h>       //  standard integer definitions


/*
 * DEFINES
 ****************************************************************************************
 */

///Length of fields in Bluetooth messages, in number of bytes
#define EVT_MASK_LEN        0x08
#define BD_ADDR_LEN         0x06
#define ACCESS_ADDR_LEN     0x04
#define LE_PASSKEY_LEN      0x04
#define BD_NAME_SIZE        0xF8
#define ADV_DATA_LEN        0x1F
#define DATA_LEN            0x1B
#define SCAN_RSP_DATA_LEN   0x1F
#define CHNL_MAP_LEN        0x05
#define KEY_LEN             0x10
#define CFM_LEN             0x10
#define ENC_DATA_LEN        0x10
#define RAND_VAL_LEN        0x10
#define RAND_NB_LEN         0x08
#define LE_FEATS_LEN        0x08
#define SUPP_CMDS_LEN       0x40
#define LMP_FEATS_LEN       0x08
#define LE_STATES_LEN       0x08
#define WHITE_LIST_LEN      0x0A
#define LE_FREQ_LEN         0x28
#define LE_FREQ_LEN         0x28
#define CRC_INIT_LEN        0x03
#define SESS_KEY_DIV_LEN    0x08
#define INIT_VECT_LEN       0x04
#define MIC_LEN             0x04

/// Format of the Advertising packets
#define ADV_ADDR_OFFSET     0
#define ADV_ADDR_LEN        BD_ADDR_LEN
#define ADV_DATA_OFFSET    (ADV_ADDR_OFFSET + ADV_ADDR_LEN)


/// BLE supported features
#define BLE_ENC_FEATURE     1

/// BLE supported states
//byte 0
#define BLE_NON_CON_ADV_STATE               0x01
#define BLE_DISC_ADV_STATE                  0x02
#define BLE_CON_ADV_STATE                   0x04
#define BLE_DIRECT_ADV_STATE                0x08
#define BLE_PASS_SCAN_STATE                 0x10
#define BLE_ACTIV_SCAN_STATE                0x20
#define BLE_INIT_MASTER_STATE               0x40
#define BLE_CON_SLAVE_STATE                 0x80

//byte 1
#define BLE_NON_CON_ADV_PASS_SCAN_STATE     0x01
#define BLE_DISC_ADV_PASS_SCAN_STATE        0x02
#define BLE_CON_ADV_PASS_SCAN_STATE         0x04
#define BLE_DIRECT_ADV_PASS_SCAN_STATE      0x08
#define BLE_NON_CON_ADV_ACTIV_SCAN_STATE    0x10
#define BLE_DISC_ADV_ACTIV_SCAN_STATE       0x20
#define BLE_CON_ADV_ACTIV_SCAN_STATE        0x40
#define BLE_DIRECT_ADV_ACTIV_SCAN_STATE     0x80

//byte 2
#define BLE_NON_CON_ADV_INIT_STATE          0x01
#define BLE_DISC_ADV_INIT_STATE             0x02
#define BLE_NON_CON_ADV_MASTER_STATE        0x04
#define BLE_DISC_ADV_MASTER_STATE           0x08
#define BLE_NON_CON_ADV_SLAVE_STATE         0x10
#define BLE_DISC_ADV_SLAVE_STATE            0x20
#define BLE_PASS_SCAN_INIT_STATE            0x40
#define BLE_ACTIV_SCAN_INIT_STATE           0x80

//byte 3
#define BLE_PASS_SCAN_MASTER_STATE          0x01
#define BLE_ACTIV_SCAN_MASTER_STATE         0x02
#define BLE_PASS_SCAN_SLAVE_STATE           0x04
#define BLE_ACTIV_SCAN_SLAVE_STATE          0x08
#define BLE_INIT_MASTER_MASTER_STATE        0x10

/// BLE supported commands
//byte0
#define BLE_DISC_CMD                0x20
//byte2
#define BLE_RD_REM_VERS_CMD         0x80
//byte5
#define BLE_SET_EVT_MSK_CMD         0x40
#define BLE_RESET_CMD               0x80
//byte10
#define BLE_HL_BUF_SIZE_CMD         0x40
#define BLE_RD_TX_PWR_CMD           0x04

#define BLE_HL_NB_CMP_PKT_CMD       0x80
//byte14
#define BLE_RD_LOC_VERS_CMD         0x08
#define BLE_RD_LOC_SUP_FEAT_CMD     0x20
#define BLE_RD_BUF_SIZE_CMD         0x80
//byte15
#define BLE_RD_BD_ADDR_CMD          0x02
#define BLE_RD_RSSI_CMD             0x20
//byte22
#define BLE_SET_EVT_MSK_PG2_CMD     0x04
//byte25
#define BLE_LE_SET_EVT_MSK_CMD      0x01
#define BLE_LE_RD_BUF_SIZE_CMD      0x02
#define BLE_LE_RD_LOC_SUP_FEAT_CMD  0x04
#define BLE_LE_SET_RAND_ADDR_CMD    0x10
#define BLE_LE_SET_ADV_PARAM_CM     0x20
#define BLE_LE_RD_ADV_TX_PWR_CMD    0x40
#define BLE_LE_SET_ADV_DATA_CMD     0x80
//byte26
#define BLE_LE_SET_SC_RSP_DATA_CMD  0x01
#define BLE_LE_SET_ADV_EN_CMD       0x02
#define BLE_LE_SET_SC_PARAM_CMD     0x04
#define BLE_LE_SET_SC_EN_CMD        0x08
#define BLE_LE_CREAT_CNX_CMD        0x10
#define BLE_LE_CREAT_CNX_CNL_CMD    0x20
#define BLE_LE_RD_WL_SIZE_CMD       0x40
#define BLE_LE_CLEAR_WL_CMD         0x80
//byte27
#define BLE_LE_ADD_DEV_WL_CMD       0x01
#define BLE_LE_REM_DEV_WL_CMD       0x02
#define BLE_LE_CNX_UPDATE_CMD       0x04
#define BLE_LE_SET_HL_CH_CLASS_CMD  0x08
#define BLE_LE_RD_CH_MAP_CMD        0x10
#define BLE_LE_RD_REM_USED_FEAT_CMD 0x20
#define BLE_LE_ENCRYPT_CMD          0x40
#define BLE_LE_RAND_CMD             0x80
//byte28
#define BLE_LE_START_ENC_CMD        0x01
#define BLE_LE_LTK_REQ_RPLY_CMD     0x02
#define BLE_LE_LTK_REQ_NEG_RPLY_CMD 0x04
#define BLE_LE_RD_SUPP_STATES_CMD   0x08
#define BLE_LE_RX_TEST_CMD          0x10
#define BLE_LE_TX_TEST_CMD          0x20
#define BLE_LE_STOP_TEST_CMD        0x40


///BT supported features
#define BT_FEATURES_BYTE0  0
#define BT_FEATURES_BYTE1  0
#define BT_FEATURES_BYTE2  0
#define BT_FEATURES_BYTE3  0
#define BT_FEATURES_BYTE4  0
#define BT_FEATURES_BYTE5  0
#define BT_FEATURES_BYTE6  0
#define BT_FEATURES_BYTE7  0


/*
 * ENUMERATIONS
 ****************************************************************************************
 */

///Transmit Power level types
enum
{
    ///Current Power Level
    TX_PW_LVL_CURRENT             = 0x00,
    ///Maximum power level
    TX_PW_LVL_MAX,
    ///Enumeration end value for power level type value check
    TX_PW_LVL_END,
};

///Controller to Host flow control
enum
{
    /// C-> H flow control off
    FLOW_CTRL_OFF                 = 0x00,
    ///C->H ACL flow control on only
    FLOW_CTRL_ON_ACL_OFF_SYNC,
    ///C->H Sync flow control on only
    FLOW_CTRL_OFF_ACL_ON_SYNC,
    ///C->H ACL and Sync  flow control on
    FLOW_CTRL_ON_ACL_ON_SYNC,
    ///Enumeration end value for flow control value check
    FLOW_CTRL_END
};

///LE Supported Host enable
enum
{
    ///Disable LE supported Host
    LE_SUPP_HOST_DIS              = 0x00,
    ///Enable LE Supported Host
    LE_SUPP_HOST_EN,
    ///Enumeration end value for LE supported Host enable check
    LE_SUPP_HOST_END
};

///Simultaneous LE Host enable
enum
{
    ///Disable LE simultaneous Host disable
    SIMULT_LE_HOST_DIS            = 0x00,
    ///Enable LE simultaneous Host disable
    SIMULT_LE_HOST_EN,
    ///Enumeration end value for LE simultaneous Host enable check
    SIMULT_LE_HOST_END
};

///Advertising HCI Type
enum
{
    ///Connectable Undirected advertising
    ADV_CONN_UNDIR                = 0x00,
    ///Connectable directed advertising
    ADV_CONN_DIR,
    ///Discoverable undirected advertising
    ADV_DISC_UNDIR,
    ///Non-connectable undirected advertising
    ADV_NONCONN_UNDIR,
    ///Enumeration end value for advertising type value check
    ADV_END
};

///Scanning HCI Type
enum
{
    ///Scan request
    SCAN_REQ,
    ///Scan response
    SCAN_RSP,
    ///Enumeration end value for scanning type value check
    SCAN_LEN
};

///BD address type
enum
{
    ///Public BD address
    ADDR_PUBLIC                   = 0x00,
    ///Random BD Address
    ADDR_RAND,
    ///Enumeration end value for BD address type value check
    ADDR_END
};

///Advertising channels enables
enum
{
    ///Byte value for advertising channel map for channel 37 enable
    ADV_CHNL_37_EN                = 0x01,
    ///Byte value for advertising channel map for channel 38 enable
    ADV_CHNL_38_EN,
    ///Byte value for advertising channel map for channel 39 enable
    ADV_CHNL_39_EN                = 0x04,
    ///Byte value for advertising channel map for channel 37, 38 and 39 enable
    ADV_ALL_CHNLS_EN              = 0x07,
    ///Enumeration end value for advertising channels enable value check
    ADV_CHNL_END
};

///Advertising filter policy
enum
{
    ///Allow both scan and connection requests from anyone
    ADV_ALLOW_SCAN_ANY_CON_ANY    = 0x00,
    ///Allow both scan req from White List devices only and connection req from anyone
    ADV_ALLOW_SCAN_WLST_CON_ANY,
    ///Allow both scan req from anyone and connection req from White List devices only
    ADV_ALLOW_SCAN_ANY_CON_WLST,
    ///Allow scan and connection requests from White List devices only
    ADV_ALLOW_SCAN_WLST_CON_WLST,
    ///Enumeration end value for advertising filter policy value check
    ADV_ALLOW_SCAN_END
};

///Advertising enables
enum
{
    ///Disable advertising
    ADV_DIS                       = 0x00,
    ///Enable advertising
    ADV_EN,
    ///Enumeration end value for advertising enable value check
    ADV_EN_END
};

///LE Scan type
enum
{
    ///Passive scan
    SCAN_PASSIVE                  = 0x00,
    ///Active scan
    SCAN_ACTIVE,
    ///Enumeration end value for scan type value check
    SCAN_END
};

///Scan filter policy
enum
{
    ///Allow advertising packets from anyone
    SCAN_ALLOW_ADV_ALL            = 0x00,
    ///Allow advertising packets from White List devices only
    SCAN_ALLOW_ADV_WLST,
    ///Enumeration end value for scan filter policy value check
    SCAN_ALLOW_ADV_END
};

///Le Scan enables
enum
{
    ///Disable scan
    SCAN_DIS                      = 0x00,
    ///Enable scan
    SCAN_EN,
    ///Enumeration end value for scan enable value check
    SCAN_EN_END
};

///Filter duplicates
enum
{
    ///Disable filtering of duplicate packets
    SCAN_FILT_DUPLIC_DIS          = 0x00,
    ///Enable filtering of duplicate packets
    SCAN_FILT_DUPLIC_EN,
    ///Enumeration end value for scan duplicate filtering value check
    SCAN_FILT_DUPLIC_END
};

///Initiator Filter policy
enum
{
    ///Initiator will ignore White List
    INIT_FILT_IGNORE_WLST         = 0x00,
    ///Initiator will use White List
    INIT_FILT_USE_WLST,
    ///Enumeration end value for initiator filter policy value check
    INIT_FILT_END
};

///Transmitter test Packet Payload Type
enum
{
    ///Pseudo-random 9 TX test payload type
    PAYL_PSEUDO_RAND_9            = 0x00,
    ///11110000 TX test payload type
    PAYL_11110000,
    ///10101010 TX test payload type
    PAYL_10101010,
    ///Pseudo-random 15 TX test payload type
    PAYL_PSEUDO_RAND_15,
    ///All 1s TX test payload type
    PAYL_ALL_1,
    ///All 0s TX test payload type
    PAYL_ALL_0,
    ///00001111 TX test payload type
    PAYL_00001111,
    ///01010101 TX test payload type
    PAYL_01010101,
    ///Enumeration end value for TX test payload type value check
    PAYL_END
};

/// Constant defining the role
enum
{
    ///Master role
    ROLE_MASTER,
    ///Slave role
    ROLE_SLAVE,
    ///Enumeration end value for role value check
    ROLE_END
};

/// Constant clock accuracy
enum
{
    ///Clock accuracy at 500PPM
    SCA_500PPM,
    ///Clock accuracy at 250PPM
    SCA_250PPM,
    ///Clock accuracy at 150PPM
    SCA_150PPM,
    ///Clock accuracy at 100PPM
    SCA_100PPM,
    ///Clock accuracy at 75PPM
    SCA_75PPM,
    ///Clock accuracy at 50PPM
    SCA_50PPM,
    ///Clock accuracy at 30PPM
    SCA_30PPM,
    ///Clock accuracy at 20PPM
    SCA_20PPM
};

/// Controller to host flow control
enum
{
    ///Flow control OFF
    FC_OFF,
    ///ACL data flow control ON, Sync data OFF
    FC_DATA_ON_SYNC_OFF,
    ///ACL data flow control OFF, Sync data ON
    FC_DATA_OFF_SYNC_ON,
    /// ACL and Sync data flow control ON
    FC_ON
};

/// Control packet opcode
enum
{
    /// Connection update request
    LL_CONNECTION_UPDATE_REQ,
    /// Channep map request
    LL_CHANNEL_MAP_REQ,
    /// Termination indication
    LL_TERMINATE_IND,
    /// Encryption request
    LL_ENC_REQ,
    /// Encryption response
    LL_ENC_RSP,
    /// Start encryption request
    LL_START_ENC_REQ,
    /// Start encryption response
    LL_START_ENC_RSP,
    /// Unknown response
    LL_UNKNOWN_RSP,
    /// Feature request
    LL_FEATURE_REQ,
    /// Feature response
    LL_FEATURE_RSP,
    /// Pause encryption request
    LL_PAUSE_ENC_REQ,
    /// Pause encryption response
    LL_PAUSE_ENC_RSP,
    /// Version indication
    LL_VERSION_IND,
    /// Reject indication
    LL_REJECT_IND,
    /// Idle
    LL_IDLE,
    /// Opcode length
    LL_OPCODE_LEN

};

/// LLID packet
enum
{
    /// Reserver for future use
    LLID_RFU,
    /// Continue
    LLID_CONTINUE,
    /// Start
    LLID_START,
    /// Control
    LLID_CNTL,
    /// End
    LLID_END,
};


/*
 * STRUCTURE DEFINITONS
 ****************************************************************************************
 */

///BD name structure
struct bd_name
{
    ///length for name
    uint8_t  namelen;
    ///array of bytes for name
    uint8_t  name[BD_NAME_SIZE];
};


///Event mask structure
struct evt_mask
{
    ///8-byte array for mask value
    uint8_t    mask[EVT_MASK_LEN];
};

///Host number of completed packets structure, for 1 connection handle
struct host_cmpl_pkts
{
    ///Connection handle
    uint16_t  con_hdl;
    ///Number of completed packets
    uint16_t  nb_cmpl_pkts;
};

///BD Address structure
struct bd_addr
{
    ///6-byte array address value
    uint8_t  addr[BD_ADDR_LEN];
};

///Access address structure
struct access_addr
{
    ///4-byte array access address
    uint8_t  addr[ACCESS_ADDR_LEN];
};

///Advertising data structure
struct adv_data
{
    ///Maximum length data bytes array
    uint8_t        data[ADV_DATA_LEN];
};

///Scan response data structure
struct scan_rsp_data
{
    ///Maximum length data bytes array
    uint8_t        data[SCAN_RSP_DATA_LEN];
};

///Channel map structure
struct chnl_map
{
    ///5-byte channel map array
    uint8_t map[CHNL_MAP_LEN];
};

///Long Term Key structure
struct ltk
{
    ///16-byte array for LTK value
    uint8_t        ltk[KEY_LEN];
};

///Random number structure
struct rand_nb
{
    ///8-byte array for random number
    uint8_t  nb[RAND_NB_LEN];
};

///Controller number of completed packets structure
struct nb_cmpl_pk
{
    ///Connection handle
    uint16_t       con_hdl;
    ///Controller number of data packets that have been completed since last time
    uint16_t       nb_hc_cmpl_pkts;
};

///Advertising report structure
struct adv_report
{
    ///event type
    uint8_t        evt_type;
    ///Advertising address type: public/random
    uint8_t        adv_addr_type;
    ///Advertising address value
    struct bd_addr adv_addr;
    ///Data length in advertising packet
    uint8_t        data_len;
    ///Data of advertising packet
    uint8_t        data[ADV_DATA_LEN];
    ///RSSI value for advertising packet
    uint8_t        rssi;

};

///Supported LE Features  structure
struct le_features
{
    ///8-byte arra for LE features
    uint8_t feats[LE_FEATS_LEN];
};

///Supported commands structure
struct supp_cmds
{
    ///64-byte array for supported commands
    uint8_t cmds[SUPP_CMDS_LEN];
};

///Supported LMP features structure
struct lmp_features
{
    ///8-byte array for LMp features
    uint8_t feats[LMP_FEATS_LEN];
};

///Supported LE states structure
struct le_states
{
    ///8-byte array for LE states
    uint8_t  supp_states[LE_STATES_LEN];
};

///White List element structure
struct white_list
{
    ///BD address of device entry
    struct bd_addr wl_bdaddr;
    ///BD address type of device entry
    uint8_t wl_bdaddr_type;
};

///CRC init structure
struct crc_init
{
    ///3-byte array CRC initial value
    uint8_t crc[CRC_INIT_LEN];
};

///Session key diversifier structure
struct sess_k_div
{
    ///8-byte array for diversifier value
    uint8_t skdiv[SESS_KEY_DIV_LEN];
};

///Session key structure
struct sess_k
{
    ///16-byte array for session key.
    uint8_t sk[2 * SESS_KEY_DIV_LEN];
};

///Initiator vector
struct init_vect
{
    ///4-byte array for vector
    uint8_t iv[INIT_VECT_LEN];
};

/// @} CO_BT
#endif // CO_BT_H_
