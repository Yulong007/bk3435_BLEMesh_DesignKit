#include "gma_include.h"
#include "gma.h"
#include "sha256.h"
#include "aes.h"
#include "app.h"
#include "app_task.h"
#include "gattc_task.h"
#include "app_ais.h"
#include "flash.h"
#include "ke_timer.h"
#include "co_bt.h"

#include "mesh_log.h"

#include "ali_config.h"
#include "app.h"
#include "mesh_general_api.h"
#include "m_prov_int.h"

#if GMA_SUPPORT

#define		Flag_InGmaState				(0x01<<0)
#define		Flag_Flash_Start			(0x01<<2)
#define		Flag_unBind_Restart			(0x01<<3)
#define		GmaOta_FlashTimes		    (2)
#define STREAM_TO_UINT16(u16, p) {u16 = ((UINT16)(*(p)) + (((UINT16)(*((p) + 1))) << 8)); (p) += 2;}
img_hdr_t img_hdr_new;

#if GMA_AES
AES_CTX ali_aes_ctx;
uint8_t iv_temp[16] = "123aqwed#*$!(4ju";
#endif

gma_para_proc_s gma_para_proc;
uint8_t inGmaOtaTime;

gma_callback_t gma_discon_cb = NULL;
uint8_t flag_global, flash_times_cnt, flag_gma;

#if GMM_MAC2
//7780,2e23120486709b6a2eafa41d87fe0bcc,f8a76351d5d8
ali_para_s ali_para = { 
	{0x32,0x1b,0x28,0x63,0xa7,0xf8},
	6541,
	{0xa1,0x8a,0x9f,0xd8,0x46,0x3f,0xbd,0x16,0x07,0x03,0x88,0xef,0x61,0xc4,0x8d,0x1c},
};

uint8_t adv_gma_data[25] = {
    /* Flags Field */ 0x02,0x01,0x06,                 /*GAP_LE_BR_EDR_NOT_SUPPORTED*/
    /* Service UUID */ 0x03,0x02,0xB3,0xFE,           //GMA service UUID: 0xFEB3
    /* Manufacturer data */ 0x11,0xFF,0xA8,0x01,0xB5,0x09,  0x8d,0x19,0x00,0x00,  0x32,0x1b,0x28,0x63,0xa7,0xf8, 0x12,0x00  //CID:0x01A8, VID:0xB5, FMSK:0x09, PID:2588, MAC:0xf8a76369da06, EXT:0x0012
};
#elif ZND_MAC_720F
//7149,fdda9fa7c036dfbd123143f903ef2487,f8a76344720f
//1bed
ali_para_s ali_para = { 
	{0x0f,0x72,0x44,0x63,0xa7,0xf8},
	7149,
	{0xfd,0xda,0x9f,0xa7,0xc0,0x36,0xdf,0xbd,0x12,0x31,0x43,0xf9,0x03,0xef,0x24,0x87},
};

uint8_t adv_gma_data[25] = {
    /* Flags Field */ 0x02,0x01,0x06,                 /*GAP_LE_BR_EDR_NOT_SUPPORTED*/
    /* Service UUID */ 0x03,0x02,0xB3,0xFE,           //GMA service UUID: 0xFEB3
    /* Manufacturer data */ 0x11,0xFF,0xA8,0x01,0xB5,0x09,  0xed,0x1b,0x00,0x00,  0x0f,0x72,0x44,0x63,0xa7,0xf8, 0x12,0x00  //CID:0x01A8, VID:0xB5, FMSK:0x09, PID:2588, MAC:0xf8a76369da06, EXT:0x0012
};

#elif Belon_MAC_7149_720F
//7149,f8a76344720f,fdda9fa7c036dfbd123143f903ef2487
//00001bed,f8a76344720f,fdda9fa7c036dfbd123143f903ef2487
ali_para_s ali_para = { 
	{0x0f,0x72,0x44,0x63,0xa7,0xf8},
	7149,
	{0xfd,0xda,0x9f,0xa7,0xc0,0x36,0xdf,0xbd,0x12,0x31,0x43,0xf9,0x03,0xef,0x24,0x87},	//Device Secret
};

uint8_t adv_gma_data[25] = {
    /* Flags Field */ 0x02,0x01,0x06,                 /*GAP_LE_BR_EDR_NOT_SUPPORTED*/
    /* Service UUID */ 0x03,0x02,0xB3,0xFE,           //GMA service UUID: 0xFEB3
    /* Manufacturer data */ 0x11,0xFF,0xA8,0x01,0xB5,0x09,  0xed,0x1b,  0x00,0x00, 0x0f,0x72,0x44,0x63,0xa7,0xf8,   0x12,0x00  //CID:0x01A8, VID:0xB5, FMSK:0x09, PID:2588, MAC:0xf8a76369da06, EXT:0x0012
};

#elif GMM_MAC_D5D8
//7780,f8a76351d5d8,2e23120486709b6a2eafa41d87fe0bcc
//000016e4,f8a76351d5d8,2e23120486709b6a2eafa41d87fe0bcc
ali_para_s ali_para = { 
	{0xd8,0xd5,0x51,0x63,0xa7,0xf8},
	7780,
	{0x2e,0x23,0x12,0x04,0x86,0x70,0x9b,0x6a,0x2e,0xaf,0xa4,0x1d,0x87,0xfe,0x0b,0xcc},	//Device Secret
};

uint8_t adv_gma_data[25] = {
    /* Flags Field */ 0x02,0x01,0x06,                 /*GAP_LE_BR_EDR_NOT_SUPPORTED*/
    /* Service UUID */ 0x03,0x02,0xB3,0xFE,           //GMA service UUID: 0xFEB3
    /* Manufacturer data */ 0x11,0xFF,0xA8,0x01,0xB5,0x09,  0x64,0x1E,  0x00,0x00, 0xd8,0xd5,0x51,0x63,0xa7,0xf8,   0x12,0x00  //CID:0x01A8, VID:0xB5, FMSK:0x09, PID:2588, MAC:0xf8a76369da06, EXT:0x0012
};

#elif	QCY_MAC_21D0

//product info: 00002881,f8a7637d21d0,9583d4ff1138f61af441414aa333c717
//SHA256(00002881,f8a7637d21d0,9583d4ff1138f61af441414aa333c717)
//=67d3417b6c35e3b9 41f82bbab1e5f07d  cfb31e98fa66b26c89f19870401f0672 (取前16个字节)
ali_para_s ali_para = {
	{0xd0,0x21,0x7d,0x63,0xa7,0xf8},
	10369,
	{0x95,0x83,0xd4,0xff,0x11,0x38,0xf6,0x1a,0xf4,0x41,0x41,0x4a,0xa3,0x33,0xc7,0x17},	//Device Secret
};

uint8_t adv_gma_data[25] = {
    /* Flags Field */ 0x02,0x01,0x06,                 /*GAP_LE_BR_EDR_NOT_SUPPORTED*/
    /* Service UUID */ 0x03,0x02,0xB3,0xFE,           //GMA service UUID: 0xFEB3
    /* Manufacturer data */ 0x11,0xFF,0xA8,0x01,0xB5,0x09,  0x81,0x28,  0x00,0x00, 0xd0,0x21,0x7d,0x63,0xa7,0xf8,   0x12,0x00  //CID:0x01A8, VID:0xB5, FMSK:0x09, PID:2588, MAC:0xf8a76369da06, EXT:0x0012
};

#elif GMM_MAC3

ali_para_s ali_para = { 
	{0xbf,0xb4,0x7d,0x63,0xa7,0xf8},
	7148,
	{0xf1,0x30,0x99,0x49,0xa4,0x3b,0xcc,0x0e,0xc6,0x8e,0xda,0x53,0x16,0xe4,0x6b,0x32},
};

uint8_t adv_gma_data[25] = {
    /* Flags Field */ 0x02,0x01,0x06,                 /*GAP_LE_BR_EDR_NOT_SUPPORTED*/
    /* Service UUID */ 0x03,0x02,0xB3,0xFE,           //GMA service UUID: 0xFEB3
    /* Manufacturer data */ 0x11,0xFF,0xA8,0x01,0xB5,0x09,  0xec,0x1b,0x00,0x00,  0xbf,0xb4,0x7d,0x63,0xa7,0xf8, 0x12,0x00  //CID:0x01A8, VID:0xB5, FMSK:0x09, PID:2588, MAC:0xf8a76369da06, EXT:0x0012
};


#elif BeiCi_Pen1
ali_para_s ali_para = { 
	{0x06,0xda,0x69,0x63,0xa7,0xf8},
	9608,
	//{0xa7,0x17,0xdc,0x78,0xf2,0x08,0x61,0xa3,0xad,0x79,0x36,0x0d,0xb2,0x6f,0x12,0xae},
	{0xa7,0x17,0xdc,0x78,0xf2,0x08,0x61,0xa3,0xad,0x79,0x36,0x0d,0xb2,0x6f,0x12,0xaa},
};

uint8_t adv_gma_data[25] = {
    /* Flags Field */ 0x02,0x01,0x06,                 /*GAP_LE_BR_EDR_NOT_SUPPORTED*/
    /* Service UUID */ 0x03,0x02,0xB3,0xFE,           //GMA service UUID: 0xFEB3
    /* Manufacturer data */ 0x11,0xFF,0xA8,0x01,0xB5,0x09,0x88,0x25,0x00,0x00,0x06,0xda,0x69,0x63,0xa7,0xf8,0x12,0x00  //CID:0x01A8, VID:0xB5, FMSK:0x09, PID:2588, MAC:0xf8a76369da06, EXT:0x0012
};
#else
ali_para_s ali_para = {
    {0xb6,0x08,0x00,0xe6,0x2f,0xd8},
    1131,
    {0x1d,0xe0,0xc8,0xd7,0xba,0x37,0x3d,0xd6,0x94,0xfb,0x68,0xf1,0x94,0x0f,0x26,0x86},
};

uint8_t adv_gma_data[23] = {
    /* Flags Field */ 0x02,0x01,0x06 /*GAP_LE_BR_EDR_NOT_SUPPORTED*/,
    /* Service UUID */ 0x03,0x02,0xB3,0xFE,           //GMA service UUID: 0xFEB3
    /* Manufacturer data */ 0x0F,0xFF,0xA8,0x01,0xA5,0x0D,0x6B,0x04,0x00,0x00,0xb6,0x08,0x00,0xe6,0x2f,0xd8,  //CID:0x01A8, VID:0xA5, FMSK:0x0D, PID:1080, MAC:0xd82fe6000833
};
#endif

/********************************GMA OTA BEGIN*******************************/
#if GMA_OTA
// GMA ota header length
#define GMA_OTA_HEADER_LEN                  32

// GMA ota result
#define GMA_OTA_FAIL                        0x00
#define GMA_OTA_SUCCESS                     0x01

typedef struct
{
    uint8_t start_flag;
    uint8_t bin_flag;
    uint8_t wdt_reset_flag;
    uint16_t frame_len;
    uint8_t frame[512]; 
    uint32_t data_len;
    uint32_t data_addr;
    uint32_t bin_len;    
    uint16_t bin_crc;
}__attribute__((packed))xm_ota_data_s;

static xm_ota_data_s gma_ota_data;
uint8_t gma_ota_is_ongoing(void);
static void gma_register_disconnect_cb(gma_callback_t cb);
static void gma_ble_disconnect_cb(void);
static void CloseMeshAdv_Proc(void);
static void CloseGmaOtaAdv_OpenMeshAdv(void);
static void CloseGmaOtaAdv_ClearFlag(void);

#endif
/**********************************GMA OTA END*********************************/
#if GMA_AES
void aes_iv_set(uint8_t *iv_buf)
{
    if(iv_buf == NULL) return;
    memcpy(ali_aes_ctx.iv,iv_buf,16);
}

void ali_digest_cal(uint8_t *digest, uint8_t *random, ali_para_s t_ali_para)
{
    char spc[80];
    char s_addr[13];
    char s_key[33];
    char s_random[17];
    int i = 0;
    SHA256_CTX sha256_context;

    memset(spc, 0, sizeof(spc));
    
    /*memset(s_random, 0, sizeof(s_random));
    for(i = 0; i < 16; i++)
    {
        sprintf(s_random, "%s%02x", s_random,random[i]);
    }*/
    
    memset(s_random, 0, sizeof(s_random));
    memcpy(s_random, random, 16);
    
    memset(s_addr, 0, sizeof(s_addr));
    for(i = 0; i < 6; i++)
    {
        sprintf(s_addr, "%s%02x", s_addr, t_ali_para.mac[5-i]);
    }

    memset(s_key, 0, sizeof(s_key));
    for(i = 0; i < 16; i++)
    {
        sprintf(s_key, "%s%02x", s_key, t_ali_para.secret[i]);
    }

    sprintf(spc, "%s,%08x,%s,%s",s_random, t_ali_para.pid , s_addr, s_key);

    ali_sha256_init(&sha256_context);
    ali_sha256_update(&sha256_context, (unsigned char*)spc, strlen(spc));
    ali_sha256_final(&sha256_context, digest);
    
    GMA_DEBUG(">>>><<<<<key: [%s] %ld\r\n", spc, strlen(spc));
    GMA_DEBUG(">>>><<<<<hash: ");
    for(i = 0; i < 16; i++)
        GMA_DEBUG("%02X ", (uint8_t)digest[i]);
    GMA_DEBUG("\r\n");
}

void ali_digest_AuthData(uint8_t *digest,ali_para_s t_ali_para)
{
    char spc[50];
    char s_addr[13];
    char s_key[33];
    int i = 0;
    SHA256_CTX sha256_context;

    memset(spc, 0, sizeof(spc));
	
    memset(s_addr, 0, sizeof(s_addr));
    for(i = 0; i < 6; i++)
    {
        sprintf(s_addr, "%s%02x", s_addr, t_ali_para.mac[5-i]);
    }

    memset(s_key, 0, sizeof(s_key));
    for(i = 0; i < 16; i++)
    {
        sprintf(s_key, "%s%02x", s_key, t_ali_para.secret[i]);
    }

    sprintf(spc, "%08x,%s,%s", t_ali_para.pid , s_addr, s_key);

	GMA_DEBUG(" --+--data=");

	GMA_DEBUG("%s",spc);
	
	GMA_DEBUG("\n");
    ali_sha256_init(&sha256_context);
    ali_sha256_update(&sha256_context, (unsigned char*)spc, strlen(spc));
    ali_sha256_final(&sha256_context, digest);
    
    GMA_DEBUG(">>>><<<<<authdata: ");
    for(i = 0; i < 16; i++)
        GMA_DEBUG("%02X ", (uint8_t)digest[i]);
    GMA_DEBUG("\r\n");

	
}


#endif

void gma_init(void)
{
    memset((void*)&gma_para_proc, 0, sizeof(gma_para_proc_s));
    memcpy(gma_para_proc.ble_mac, ali_para.mac , 6);
    memcpy(gma_para_proc.dev_random, "1234567890123456", 16);
    gma_para_proc.init_flag = 1;
    
    memcpy(&adv_gma_data[13], (uint8_t *)&ali_para.pid, 4);
    memcpy(&adv_gma_data[17], ali_para.mac, 6);
    
    extern struct bd_addr co_default_bdaddr;
    memcpy(co_default_bdaddr.addr, ali_para.mac, 6);

	flash_read(FLASH_MAIN_BASE_ADDR, SEC_IMAGE_OAD_HEADER_APP_FADDR, sizeof(img_hdr_t), (uint8_t *)&img_hdr_new, NULL);

	gma_register_disconnect_cb(gma_ble_disconnect_cb);
}


void gma_AliInit(void)
{
	uint8_t dev_mac[6];
	uint32_t	dev_pid;
	uint8_t dev_deviceKey[16];
	uint8_t auth_data[32];
	uint8_t i;
    memset((void*)&gma_para_proc, 0, sizeof(gma_para_proc_s));

	
	if(user_data_contains_ali_data())
	{
		user_data_read_ali_mac(ali_para.mac,1);
		ali_para.pid = user_data_read_ali_product_id();
		user_data_read_ali_secret_froward_key(ali_para.secret);
	}

	GMA_DEBUG("\n**--**mac=");
	for(i=0;i<6;i++)
	{
		GMA_DEBUG("%2x",ali_para.mac[i]);
	}

	GMA_DEBUG("\n");

	GMA_DEBUG("\n**--**pid=%d\n",ali_para.pid);

	GMA_DEBUG("--*--secret key=");
	for(i=0;i<16;i++)
	{
		GMA_DEBUG("%2x",ali_para.secret[i]);
	}
	GMA_DEBUG("\n");




    memcpy(gma_para_proc.ble_mac, ali_para.mac , 6);
    memcpy(gma_para_proc.dev_random, "1234567890123456", 16);
	
    gma_para_proc.init_flag = 1;
    
    memcpy(&adv_gma_data[13], (uint8_t *)&ali_para.pid, 4);
    memcpy(&adv_gma_data[17], ali_para.mac, 6);
    
    extern struct bd_addr co_default_bdaddr;
    memcpy(co_default_bdaddr.addr, ali_para.mac, 6);

	gma_register_disconnect_cb(gma_ble_disconnect_cb);
	flash_read(FLASH_MAIN_BASE_ADDR, SEC_IMAGE_OAD_HEADER_APP_FADDR, sizeof(img_hdr_t), (uint8_t *)&img_hdr_new, NULL);
}


#define		AuthData_Len	16
void User_GetAuthData(uint8_t* authdata){
	uint8_t auth_data_temp[32];
	uint8_t i;
	ali_digest_AuthData(auth_data_temp,ali_para);
	for(i=0;i<AuthData_Len;i++){
		authdata[i] = auth_data_temp[AuthData_Len - 1-i];
	}
}

void gma_flag_clear(void)
{
    //if(!LEconnection_LE_Connected())
    {
        gma_para_proc.init_flag = 1;
    }
}

void gma_data_send(uint8_t *buf, uint16_t len)
{
    if(!gma_para_proc.speech_flag && !gma_ota_is_ongoing())
    {
    	uint8_t i;
        GMA_DEBUG("gma send %d: ", len);
        for(i = 0; i < len; i++)
            GMA_DEBUG("%02x ", buf[i]);
        GMA_DEBUG("\r\n\r\n");
    }

    if(appm_ble_is_connected())
    {
        switch(buf[1])    //cmd
        {
            // GMA general command type
            case CMD_DEV_REPORT:
            case CMD_APP_REQ:
            case CMD_APP_RSP:
            case CMD_DEV_REQ:
            case CMD_DEV_RSP:
            case CMD_APP_CMD:
            case CMD_APP_MANU_REQ:
            case CMD_APP_MANU_RSP:
            case CMD_DEV_ERROR:
                
            // GMA authentication command type
            case CMD_AUTH_START_REQ:
            case CMD_AUTH_START_RSP:
            case CMD_AUTH_RESULT_REQ:
            case CMD_AUTH_RESULT_RSP:
            case CMD_AUTH_CONFIRM_NET_REQ:
            case CMD_AUTH_CONFIRM_NET_RSP:

            // GMA voice command type
            case CMD_DEV_INFO_REQ:
            case CMD_DEV_INFO_RSP:
            case CMD_ACTIVE_REQ:
            case CMD_ACTIVE_RSP:
            case CMD_STATUS_SET_REQ:
            case CMD_STATUS_SET_RSP:
            case CMD_A2DP_SET_REQ:
            case CMD_A2DP_SET_RSP:
            case CMD_HFP_SET_REQ:
            case CMD_HFP_SET_RSP:
            case CMD_MIC_READY_REQ:
            case CMD_MIC_READY_RSP:
            case CMD_DEV_PARAM_REQ:
            case CMD_DEV_PARAM_RSP:
                app_gma_data_send(buf, len, GATTC_INDICATE);
                break;

            // GMA ota command type
            case CMD_OTA_FW_VER_REQ:
            case CMD_OTA_FW_VER_RSP:
            case CMD_OTA_START_REQ:
            case CMD_OTA_START_RSP:
            case CMD_OTA_FRAME_LEN_RPT:
            case CMD_OTA_END_CHECK_REQ:
            case CMD_OTA_END_CHECK_RSP:
            case CMD_OTA_PDU:
            
            // GMA voice command type
            case CMD_MIC_VOICE_SEND:
                app_gma_data_send(buf, len, GATTC_NOTIFY);
                break;
                
            default:
                GMA_PRINTF("gma_data_send CMD error!!!");
                break;
        }
    }
    else
    {
        GMA_PRINTF("gma_data_send error!!!");
    }
}

void gma_send_dev_report(uint8_t msg_id, uint8_t *data, uint8_t len)
{
    gma_frame_head_s reply_rsp_head;
    uint8_t send_buf[50 + GMA_PAYLOAD_HEAD];
    int en_len;
    uint8_t i = 0;

    reply_rsp_head.msg_id = msg_id;
    reply_rsp_head.save_flag = gma_para_proc.auth_flag;
    reply_rsp_head.version = 0;
    reply_rsp_head.cmd = CMD_DEV_REPORT;
    reply_rsp_head.fn = 0;
    reply_rsp_head.total_fn = 0;

    GMA_DEBUG("gma_send_dev_report %d: ", len);
    for(i = 0; i < len; i++)
        GMA_DEBUG("%02x ", data[i]);
    GMA_DEBUG("\r\n");

    if(gma_para_proc.auth_flag)
    {
#if GMA_AES
        aes_iv_set(iv_temp);
        aes_cbc_encrypt_pkcs7(&ali_aes_ctx, gma_para_proc.ble_key, data, len, &send_buf[GMA_PAYLOAD_HEAD], &en_len);
#endif
    }
    else
    {
        memcpy(&send_buf[GMA_PAYLOAD_HEAD], data, len);
        en_len = len;
    }

    reply_rsp_head.f_len = en_len;
    memcpy(send_buf, (uint8_t*)&reply_rsp_head, sizeof(gma_frame_head_s));

    gma_data_send(send_buf, en_len + GMA_PAYLOAD_HEAD);
    return;
}

void gma_reply_app_req(uint8_t msg_id, uint8_t *data, uint8_t len)
{
    gma_frame_head_s reply_rsp_head;
    uint8_t send_buf[50 + GMA_PAYLOAD_HEAD];
    int en_len = 0;
    uint8_t i = 0;

    reply_rsp_head.msg_id = msg_id;
    reply_rsp_head.save_flag = gma_para_proc.auth_flag;
    reply_rsp_head.version = 0;
    reply_rsp_head.cmd = CMD_APP_RSP;
    reply_rsp_head.fn = 0;
    reply_rsp_head.total_fn = 0;

    GMA_DEBUG("gma_reply_app_req %d: ", len);
    for(i = 0; i < len; i++)
        GMA_DEBUG("%02x ", data[i]);
    GMA_DEBUG("\r\n");

    if(gma_para_proc.auth_flag)
    {
#if GMA_AES
        aes_iv_set(iv_temp);
        aes_cbc_encrypt_pkcs7(&ali_aes_ctx, gma_para_proc.ble_key, data, len, &send_buf[GMA_PAYLOAD_HEAD], &en_len);
#endif
    }
    else
    {
        memcpy(&send_buf[GMA_PAYLOAD_HEAD], data, len);
        en_len = len;
    }

    reply_rsp_head.f_len = en_len;
    memcpy(send_buf, (uint8_t*)&reply_rsp_head, sizeof(gma_frame_head_s));

    gma_data_send(send_buf, en_len + GMA_PAYLOAD_HEAD);
    return;
}

uint8_t gma_recv_app_req(gma_recv_data_s *gma_recv_data)
{
    uint8_t de_out[50], i;
    int de_len;

    if(gma_recv_data->gma_frame_head.save_flag)
    {
        aes_iv_set(iv_temp);
        aes_cbc_decrypt_pkcs7(&ali_aes_ctx, gma_para_proc.ble_key, gma_recv_data->data, gma_recv_data->gma_frame_head.f_len, de_out, &de_len);
    }
    else
    {
        memcpy(de_out, gma_recv_data->data, gma_recv_data->gma_frame_head.f_len);
        de_len = gma_recv_data->gma_frame_head.f_len;
    }
 
    GMA_PRINTF("gma_recv_app_req %d: ", de_len);
    for(i = 0; i < de_len; i++)
        GMA_PRINTF("%02x ", de_out[i]);
    GMA_PRINTF("\r\n");
    
    gma_reply_app_req(gma_recv_data->gma_frame_head.msg_id, NULL, 0);
    return 0;
}

void gma_send_dev_req(uint8_t msg_id, uint8_t *data, uint8_t len)
{
    gma_frame_head_s reply_rsp_head;
    uint8_t send_buf[50 + GMA_PAYLOAD_HEAD];
    int en_len;
    uint8_t i = 0;

    reply_rsp_head.msg_id = msg_id;
    reply_rsp_head.save_flag = gma_para_proc.auth_flag;
    reply_rsp_head.version = 0;
    reply_rsp_head.cmd = CMD_DEV_REQ;
    reply_rsp_head.fn = 0;
    reply_rsp_head.total_fn = 0;

    GMA_DEBUG("gma_send_dev_req %d: ", len);
    for(i = 0; i < len; i++)
        GMA_DEBUG("%02x ", data[i]);
    GMA_DEBUG("\r\n");

    if(gma_para_proc.auth_flag)
    {
#if GMA_AES
        aes_iv_set(iv_temp);
        aes_cbc_encrypt_pkcs7(&ali_aes_ctx, gma_para_proc.ble_key, data, len, &send_buf[GMA_PAYLOAD_HEAD], &en_len);
#endif
    }
    else
    {
        memcpy(&send_buf[GMA_PAYLOAD_HEAD], data, len);
        en_len = len;
    }

    reply_rsp_head.f_len = en_len;
    memcpy(send_buf, (uint8_t*)&reply_rsp_head, sizeof(gma_frame_head_s));

    gma_data_send(send_buf, en_len + GMA_PAYLOAD_HEAD);
    return;
}

uint8_t gma_recv_dev_rsp(gma_recv_data_s *gma_recv_data)
{
    uint8_t de_out[50], i;
    int de_len;

    if(gma_recv_data->gma_frame_head.save_flag)
    {
        aes_iv_set(iv_temp);
        aes_cbc_decrypt_pkcs7(&ali_aes_ctx, gma_para_proc.ble_key, gma_recv_data->data, gma_recv_data->gma_frame_head.f_len, de_out, &de_len);
    }
    else
    {
        memcpy(de_out, gma_recv_data->data, gma_recv_data->gma_frame_head.f_len);
        de_len = gma_recv_data->gma_frame_head.f_len;
    }
 
    GMA_PRINTF("gma_recv_dev_rsp %d: ", de_len);
    for(i = 0; i < de_len; i++)
        GMA_PRINTF("%02x ", de_out[i]);
    GMA_PRINTF("\r\n");
    
    return 0;
}

uint8_t gma_recv_app_cmd(gma_recv_data_s *gma_recv_data)
{
    uint8_t de_out[50], i;
    int de_len;

    if(gma_recv_data->gma_frame_head.save_flag)
    {
        aes_iv_set(iv_temp);
        aes_cbc_decrypt_pkcs7(&ali_aes_ctx, gma_para_proc.ble_key, gma_recv_data->data, gma_recv_data->gma_frame_head.f_len, de_out, &de_len);
    }
    else
    {
        memcpy(de_out, gma_recv_data->data, gma_recv_data->gma_frame_head.f_len);
        de_len = gma_recv_data->gma_frame_head.f_len;
    }
 
    GMA_PRINTF("gma_recv_app_cmd %d: ", de_len);
    for(i = 0; i < de_len; i++)
        GMA_PRINTF("%02x ", de_out[i]);
    GMA_PRINTF("\r\n");
    
    return 0;
}

void gma_reply_app_manu_req(uint8_t msg_id, uint8_t *data, uint8_t len)
{
    gma_frame_head_s reply_rsp_head;
    uint8_t send_buf[50 + GMA_PAYLOAD_HEAD];
    int en_len;
    uint8_t i = 0;

    reply_rsp_head.msg_id = msg_id;
    reply_rsp_head.save_flag = gma_para_proc.auth_flag;
    reply_rsp_head.version = 0;
    reply_rsp_head.cmd = CMD_APP_MANU_RSP;
    reply_rsp_head.fn = 0;
    reply_rsp_head.total_fn = 0;

    GMA_DEBUG("gma_reply_app_manu_req %d: ", len);
    for(i = 0; i < len; i++)
        GMA_DEBUG("%02x ", data[i]);
    GMA_DEBUG("\r\n");

    if(gma_para_proc.auth_flag)
    {
#if GMA_AES
        aes_iv_set(iv_temp);
        aes_cbc_encrypt_pkcs7(&ali_aes_ctx, gma_para_proc.ble_key, data, len, &send_buf[GMA_PAYLOAD_HEAD], &en_len);
#endif
    }
    else
    {
        memcpy(&send_buf[GMA_PAYLOAD_HEAD], data, len);
        en_len = len;
    }

    reply_rsp_head.f_len = en_len;
    memcpy(send_buf, (uint8_t*)&reply_rsp_head, sizeof(gma_frame_head_s));

    gma_data_send(send_buf, en_len + GMA_PAYLOAD_HEAD);
    return;
}

uint8_t gma_recv_app_manu_req(gma_recv_data_s *gma_recv_data)
{
    uint8_t de_out[50], i;
    int de_len;

    if(gma_recv_data->gma_frame_head.save_flag)
    {
        aes_iv_set(iv_temp);
        aes_cbc_decrypt_pkcs7(&ali_aes_ctx, gma_para_proc.ble_key, gma_recv_data->data, gma_recv_data->gma_frame_head.f_len, de_out, &de_len);
    }
    else
    {
        memcpy(de_out, gma_recv_data->data, gma_recv_data->gma_frame_head.f_len);
        de_len = gma_recv_data->gma_frame_head.f_len;
    }
 
    GMA_PRINTF("gma_recv_app_manu_req %d: ", de_len);
    for(i = 0; i < de_len; i++)
        GMA_PRINTF("%02x ", de_out[i]);
    GMA_PRINTF("\r\n");
    
    gma_reply_app_manu_req(gma_recv_data->gma_frame_head.msg_id, NULL, 0);
    return 0;
}

void gma_send_dev_error(uint8_t msg_id, uint8_t *data, uint8_t len)
{
    gma_frame_head_s reply_rsp_head;
    uint8_t send_buf[50 + GMA_PAYLOAD_HEAD];
    int en_len;
    uint8_t i = 0;

    reply_rsp_head.msg_id = msg_id;
    reply_rsp_head.save_flag = gma_para_proc.auth_flag;
    reply_rsp_head.version = 0;
    reply_rsp_head.cmd = CMD_DEV_ERROR;
    reply_rsp_head.fn = 0;
    reply_rsp_head.total_fn = 0;

    GMA_DEBUG("gma_send_dev_error %d: ", len);
    for(i = 0; i < len; i++)
        GMA_DEBUG("%02x ", data[i]);
    GMA_DEBUG("\r\n");

    if(gma_para_proc.auth_flag)
    {
#if GMA_AES
        aes_iv_set(iv_temp);
        aes_cbc_encrypt_pkcs7(&ali_aes_ctx, gma_para_proc.ble_key, data, len, &send_buf[GMA_PAYLOAD_HEAD], &en_len);
#endif
    }
    else
    {
        memcpy(&send_buf[GMA_PAYLOAD_HEAD], data, len);
        en_len = len;
    }

    reply_rsp_head.f_len = en_len;
    memcpy(send_buf, (uint8_t*)&reply_rsp_head, sizeof(gma_frame_head_s));

    gma_data_send(send_buf, en_len + GMA_PAYLOAD_HEAD);
    return;
}

#if GMA_AES
void gma_reply_auth_start(uint8_t msg_id, uint8_t *payload, uint16_t len)
{
    gma_frame_head_s reply_rsp_head;
    uint8_t send_buf[32 + GMA_PAYLOAD_HEAD] = {0};
    uint8_t i;
    
    GMA_DEBUG("reply_auth_start %d: ", len);
    for(i = 0; i < len; i++)
        GMA_DEBUG("%02x ", payload[i]);
    GMA_DEBUG("\r\n");

    reply_rsp_head.msg_id = msg_id;
    reply_rsp_head.save_flag = 0;
    reply_rsp_head.version = 0;
    reply_rsp_head.cmd = CMD_AUTH_START_RSP;
    reply_rsp_head.fn = 0;
    reply_rsp_head.total_fn = 0;
    reply_rsp_head.f_len = len;
    
    memcpy(send_buf, (uint8_t*)(&reply_rsp_head), sizeof(gma_frame_head_s));
    memcpy(&send_buf[GMA_PAYLOAD_HEAD], payload, len);
    gma_data_send(send_buf, len + GMA_PAYLOAD_HEAD);
    return;
}

uint8_t gma_recv_auth_start(gma_recv_data_s *gma_recv_data)
{
    uint8_t digest_t[32];
    uint8_t en_output[32], i;
    int en_len;

    GMA_PRINTF("auth_start %d: ", gma_recv_data->gma_frame_head.f_len);
    for(i = 0; i < gma_recv_data->gma_frame_head.f_len; i++)
        GMA_PRINTF("%02x ", gma_recv_data->data[i]);
    GMA_PRINTF("\r\n");

    memcpy(gma_para_proc.remote_random, gma_recv_data->data, gma_recv_data->gma_frame_head.f_len);
    ali_digest_cal(digest_t, gma_para_proc.remote_random, ali_para);
    memcpy(gma_para_proc.ble_key, digest_t, 16); 
     
    aes_iv_set(iv_temp);
    aes_cbc_encrypt_pkcs7(&ali_aes_ctx, gma_para_proc.ble_key, gma_para_proc.remote_random, 16, en_output, &en_len);

    en_len = 16;
    gma_reply_auth_start(gma_recv_data->gma_frame_head.msg_id, en_output, en_len);
    return 0;
}

void gma_reply_auth_result(uint8_t msg_id,uint8_t data)
{
    gma_frame_head_s reply_rsp_head;
    uint8_t send_buf[1 + GMA_PAYLOAD_HEAD] = {0};

    reply_rsp_head.msg_id = msg_id;
    reply_rsp_head.save_flag = 0;
    reply_rsp_head.version = 0;
    reply_rsp_head.cmd = CMD_AUTH_RESULT_RSP;
    reply_rsp_head.fn = 0;
    reply_rsp_head.total_fn = 0;
    reply_rsp_head.f_len = 1;
    
    memcpy(send_buf, (uint8_t*)(&reply_rsp_head), sizeof(gma_frame_head_s));
    send_buf[GMA_PAYLOAD_HEAD] = data;
    gma_data_send(send_buf, 1 + GMA_PAYLOAD_HEAD);
    return;
}

uint8_t gma_recv_auth_result(gma_recv_data_s *gma_recv_data)
{
    if(SUCCESS == gma_recv_data->data[0])
    {
        gma_para_proc.auth_flag = 1;
        GMA_PRINTF("auth result success\r\n");
    }
    else if(FAIL == gma_recv_data->data[0])
    {
        GMA_PRINTF("auth result fail\r\n");
    }
    else
    {
        GMA_PRINTF("auth result unidentify\r\n");
        return 1;
    }
    gma_reply_auth_result(gma_recv_data->gma_frame_head.msg_id, gma_recv_data->data[0]);
    return 0;
}
#endif

void gma_reply_auth_confirm_net(uint8_t msg_id, uint8_t *payload, uint16_t len)
{
    gma_frame_head_s reply_rsp_head;
    uint8_t send_buf[16 + GMA_PAYLOAD_HEAD] = {0};
    int en_len = 0;

    reply_rsp_head.msg_id = msg_id;
    reply_rsp_head.save_flag = gma_para_proc.auth_flag;
    reply_rsp_head.version = 0;
    reply_rsp_head.cmd = CMD_AUTH_CONFIRM_NET_RSP;
    reply_rsp_head.fn = 0;
    reply_rsp_head.total_fn = 0;

    if(gma_para_proc.auth_flag)
    {
#if GMA_AES
        aes_iv_set(iv_temp);
        aes_cbc_encrypt_pkcs7(&ali_aes_ctx, gma_para_proc.ble_key, payload, len, &send_buf[GMA_PAYLOAD_HEAD], &en_len);
#endif
    }
    else
    {
        memcpy(&send_buf[GMA_PAYLOAD_HEAD], payload, len);
        en_len = len;
    }
    
    reply_rsp_head.f_len = en_len;
    memcpy(send_buf, (uint8_t*)(&reply_rsp_head), sizeof(gma_frame_head_s));
    
    gma_data_send(send_buf, en_len + GMA_PAYLOAD_HEAD);
    return;
}

uint8_t gma_recv_auth_confirm_net(gma_recv_data_s *gma_recv_data)
{
    uint8_t de_out[1], i;
    int de_len;
    
    if(gma_recv_data->gma_frame_head.save_flag)
    {
#if GMA_AES
        aes_iv_set(iv_temp);
        aes_cbc_decrypt_pkcs7(&ali_aes_ctx, gma_para_proc.ble_key, gma_recv_data->data, gma_recv_data->gma_frame_head.f_len, de_out, &de_len);
#endif
    }
    else
    {
        memcpy(de_out, gma_recv_data->data, gma_recv_data->gma_frame_head.f_len);
        de_len = gma_recv_data->gma_frame_head.f_len;
    }
    
    GMA_DEBUG("gma_recv_auth_confirm_net %d: ", de_len);
    for(i = 0; i < de_len; i++)
        GMA_DEBUG("%02x ", de_out[i]);
    GMA_DEBUG("\r\n");

    if(BOND == de_out[0])
    {
        gma_para_proc.auth_flag = 1;
        GMA_PRINTF("auth confirm network success\r\n");
    }
    else if(DEBOND == de_out[0])
    {
        GMA_PRINTF("auth confirm network fail\r\n");
    }
    else
    {
        GMA_PRINTF("auth confirm network unidentify\r\n");
        return 1;
    }
    gma_reply_auth_confirm_net(gma_recv_data->gma_frame_head.msg_id, de_out, sizeof(de_out));
    return 0;
}

void gma_reply_dev_info(uint8_t msg_id, uint8_t *data, uint8_t len)
{
    gma_frame_head_s reply_rsp_head;
    gma_dev_info_s dev_info;
    uint8_t send_buf[16 + GMA_PAYLOAD_HEAD] = {0};
    int en_len = 0;
    uint8_t i;

    reply_rsp_head.msg_id = msg_id;
    reply_rsp_head.save_flag = gma_para_proc.auth_flag;
    reply_rsp_head.version = 0;
    reply_rsp_head.cmd = CMD_DEV_INFO_RSP;
    reply_rsp_head.fn = 0;
    reply_rsp_head.total_fn = 0;  

    dev_info.ability = 0x50;
    dev_info.audio_coding = 0x02;      //0x00:PCM, 0x01:ADPCM, 0x02:OPUS
    dev_info.gma_ver = 0x0001;
    memcpy(dev_info.classic_mac, gma_para_proc.ble_mac, 6);

    GMA_DEBUG("reply_dev_info %d: ", sizeof(gma_dev_info_s));
    for(i = 0; i < sizeof(gma_dev_info_s); i++)
        GMA_DEBUG("%02x ", ((uint8_t*)&dev_info)[i]);
    GMA_DEBUG("\r\n");

    if(gma_para_proc.auth_flag)
    {
#if GMA_AES
        aes_iv_set(iv_temp);
        aes_cbc_encrypt_pkcs7(&ali_aes_ctx, gma_para_proc.ble_key, (uint8_t*)&dev_info, sizeof(gma_dev_info_s), &send_buf[GMA_PAYLOAD_HEAD], &en_len);
#endif
    }
    else
    {
        memcpy(&send_buf[GMA_PAYLOAD_HEAD], (uint8_t*)&dev_info, sizeof(gma_dev_info_s));
        en_len = sizeof(gma_dev_info_s);
    }
    
    reply_rsp_head.f_len = en_len; 
    memcpy(send_buf, (uint8_t *)&reply_rsp_head, sizeof(gma_frame_head_s));

    gma_data_send(send_buf, en_len + GMA_PAYLOAD_HEAD);
    return;
}

uint8_t gma_recv_dev_info(gma_recv_data_s *gma_recv_data)
{
    gma_mobile_info_s *mobile_info;
    uint8_t de_out[3], i;
    int de_len;
    
    if(gma_recv_data->gma_frame_head.save_flag)
    {
#if GMA_AES
        aes_iv_set(iv_temp);
        aes_cbc_decrypt_pkcs7(&ali_aes_ctx, gma_para_proc.ble_key, gma_recv_data->data, gma_recv_data->gma_frame_head.f_len, de_out, &de_len);
#endif
    }
    else
    {
        memcpy(de_out, gma_recv_data->data, gma_recv_data->gma_frame_head.f_len);
        de_len = gma_recv_data->gma_frame_head.f_len;
    }
 
    mobile_info = (gma_mobile_info_s *)de_out;
    
    GMA_PRINTF("dev_info %d: ", de_len);
    for(i = 0; i < de_len; i++)
        GMA_PRINTF("%02x ", de_out[i]);
    GMA_PRINTF("\r\n");
    if(0x01 == mobile_info->mobile_type)
        GMA_PRINTF("android\r\n");
    else
        GMA_PRINTF("ios\r\n");

    gma_reply_dev_info(gma_recv_data->gma_frame_head.msg_id, NULL, 0);
    return 0;
}

#if GMA_AES
void gma_reply_active(uint8_t msg_id)
{
    gma_frame_head_s reply_rsp_head;
    gma_active_s active_value;
    uint8_t send_buf[48 + GMA_PAYLOAD_HEAD];
    uint8_t sha256_result[32];
    int en_len;
   
    reply_rsp_head.msg_id = msg_id;
    reply_rsp_head.save_flag = gma_para_proc.auth_flag;
    reply_rsp_head.version = 0;
    reply_rsp_head.cmd = CMD_ACTIVE_RSP;
    reply_rsp_head.fn = 0;
    reply_rsp_head.total_fn = 0;

    ali_digest_cal(sha256_result, gma_para_proc.dev_random, ali_para);
    memcpy(active_value.random, gma_para_proc.dev_random, 16);
    memcpy(active_value.digest, sha256_result, 16);
    
    if(gma_para_proc.auth_flag)
    {
        aes_iv_set(iv_temp);
        aes_cbc_encrypt_pkcs7(&ali_aes_ctx,gma_para_proc.ble_key, (uint8_t*)&active_value, sizeof(gma_active_s), &send_buf[GMA_PAYLOAD_HEAD], &en_len);
    }
    else
    {
        memcpy(&send_buf[GMA_PAYLOAD_HEAD], (uint8_t*)&active_value, sizeof(gma_active_s));
        en_len = sizeof(gma_active_s);
    }

    reply_rsp_head.f_len = en_len;
    memcpy(send_buf, (uint8_t*)&reply_rsp_head, sizeof(gma_frame_head_s));
    
    gma_data_send(send_buf, en_len + GMA_PAYLOAD_HEAD);
    return;
}

uint8_t gma_recv_active(gma_recv_data_s *gma_recv_data)
{
    GMA_PRINTF("recv_active\r\n");
    gma_reply_active(gma_recv_data->gma_frame_head.msg_id);
    return 0;
}
#endif

void gma_reply_status_set(uint8_t msg_id, gma_tlv_s para_tlv)
{
    gma_frame_head_s reply_rsp_head;
    uint8_t send_buf[16 + GMA_PAYLOAD_HEAD];
    int en_len;

    reply_rsp_head.msg_id = msg_id;
    reply_rsp_head.save_flag = gma_para_proc.auth_flag;
    reply_rsp_head.version = 0;
    reply_rsp_head.cmd = CMD_STATUS_SET_RSP;
    reply_rsp_head.fn = 0;
    reply_rsp_head.total_fn = 0;

    if(gma_para_proc.auth_flag)
    {
#if GMA_AES
        aes_iv_set(iv_temp);
        aes_cbc_encrypt_pkcs7(&ali_aes_ctx, gma_para_proc.ble_key, (uint8_t*)(&para_tlv), sizeof(gma_tlv_s), &send_buf[GMA_PAYLOAD_HEAD], &en_len);
#endif
    }
    else
    {
        memcpy(&send_buf[GMA_PAYLOAD_HEAD], (uint8_t*)(&para_tlv), sizeof(gma_tlv_s));
        en_len = sizeof(gma_tlv_s);
    }
    
    reply_rsp_head.f_len = en_len;
    memcpy(send_buf, (uint8_t*)&reply_rsp_head, sizeof(gma_frame_head_s));

    gma_data_send(send_buf, reply_rsp_head.f_len + GMA_PAYLOAD_HEAD);
    return;
}

uint8_t gma_recv_status_set(gma_recv_data_s *gma_recv_data)
{
    uint8_t de_out[3], i;
    int de_len;
    uint8_t result_value;
    gma_tlv_s gma_tlv;
      
    if(gma_recv_data->gma_frame_head.save_flag)
    {
#if GMA_AES
        aes_iv_set(iv_temp);
        aes_cbc_decrypt_pkcs7(&ali_aes_ctx,gma_para_proc.ble_key, gma_recv_data->data, gma_recv_data->gma_frame_head.f_len, de_out, &de_len);
#endif
    }
    else
    {
        memcpy(de_out, gma_recv_data->data, gma_recv_data->gma_frame_head.f_len);
        de_len = gma_recv_data->gma_frame_head.f_len;
    }

    GMA_PRINTF("status set %d: ", de_len);
    for(i = 0; i < de_len; i++)
        GMA_PRINTF("%02x ", de_out[i]);
    GMA_PRINTF("\r\n");
    
    switch(de_out[0])
    {
        case CONNCT_READY:
            result_value = 0;
            gma_tlv.type = CONNCT_READY;
            gma_tlv.value = result_value;
            GMA_PRINTF("CONNCT_READY\r\n");  
            break;

        case MIC_START:
            GMA_PRINTF("MIC_START\r\n");  
            result_value = 1;
            gma_tlv.type = MIC_START;
            gma_tlv.value = result_value;
            
            gma_para_proc.speech_flag = 1;
            //aud_adc_initial(16000, 1, 16);
            //aud_mic_open(1);
    	    //opus_encode_init();
            break;

        case MIC_STOP:
            GMA_PRINTF("MIC_STOP\r\n");  
            result_value = 1;
            gma_tlv.type = MIC_STOP;
            gma_tlv.value = result_value;
            
            gma_para_proc.speech_flag = 0;
            //aud_mic_open(0);
            //opus_encode_destroy();
            //if(bt_flag1_is_set(APP_FLAG_MUSIC_PLAY|APP_FLAG_WAVE_PLAYING))
            //    app_audio_restore();
            break;

        case PLAY_TTS:
            GMA_PRINTF("PLAY_TTS\r\n"); 
            result_value = 1;
            gma_tlv.type = PLAY_TTS;
            gma_tlv.value = result_value;
            break;

        case STOP_TTS:
            GMA_PRINTF("STOP_TTS\r\n");  
            result_value = 1;
            gma_tlv.type = STOP_TTS;
            gma_tlv.value = result_value;
            break;
        
        case RECORD_START:
            GMA_PRINTF("RECORD_START\r\n");  
            break;
        
        case RECORD_STOP:
            GMA_PRINTF("RECORD_STOP\r\n");  
            break;

        default:
            break;
    }
    gma_tlv.len = 1;
    gma_reply_status_set(gma_recv_data->gma_frame_head.msg_id, gma_tlv);
    return 0;
}

void gma_reply_dev_param(uint8_t msg_id, uint8_t *data, uint8_t len)
{
    gma_frame_head_s reply_rsp_head;
    uint8_t send_buf[16 + GMA_PAYLOAD_HEAD];
    int en_len;
    uint8_t i = 0;
    
    reply_rsp_head.msg_id = msg_id;
    reply_rsp_head.save_flag = gma_para_proc.auth_flag;
    reply_rsp_head.version = 0;
    reply_rsp_head.cmd = CMD_DEV_PARAM_RSP;
    reply_rsp_head.fn = 0;
    reply_rsp_head.total_fn = 0;
    
    GMA_DEBUG("reply_dev_param %d: ", len);
    for(i = 0; i < len; i++)
        GMA_DEBUG("%02x ", data[i]);
    GMA_DEBUG("\r\n");

    if(gma_para_proc.auth_flag)
    {
#if GMA_AES
        aes_iv_set(iv_temp);
        aes_cbc_encrypt_pkcs7(&ali_aes_ctx, gma_para_proc.ble_key, data, len, &send_buf[GMA_PAYLOAD_HEAD], &en_len);
#endif
    }
    else
    {
        memcpy(&send_buf[GMA_PAYLOAD_HEAD], data, len);
        en_len = len;
    }
    
    reply_rsp_head.f_len = en_len;
    memcpy(send_buf, (uint8_t*)&reply_rsp_head, sizeof(gma_frame_head_s));

    gma_data_send(send_buf, en_len + GMA_PAYLOAD_HEAD);
    return;
}

void gma_recv_dev_param(gma_recv_data_s *gma_recv_data)
{
    uint8_t de_out[30], i;
    int de_len;
    uint8_t data[10];
        
    if(gma_recv_data->gma_frame_head.save_flag)
    {
#if GMA_AES
        aes_iv_set(iv_temp);
        aes_cbc_decrypt_pkcs7(&ali_aes_ctx,gma_para_proc.ble_key, gma_recv_data->data, gma_recv_data->gma_frame_head.f_len, de_out, &de_len);
#endif
    }
    else
    {
        memcpy(de_out, gma_recv_data->data, gma_recv_data->gma_frame_head.f_len);
        de_len = gma_recv_data->gma_frame_head.f_len;
    }
    
    GMA_PRINTF("recv_dev_param %d: ",de_len);
    for(i = 0; i < de_len; i++)
        GMA_PRINTF("%02x ", de_out[i]);
    GMA_PRINTF("\r\n");
    
    switch(de_out[0])
    {
        case BATTARY_LEVEL:
            GMA_PRINTF("BATTARY_LEVEL\r\n"); 
            data[0] = BATTARY_LEVEL;
            data[1] = 0x02;      //length
            data[2] = 0x64;      //battatry level: 100%
            data[3] = 0x00;      //battatry status: good
            break;

        case BATTARY_STATUS:
            GMA_PRINTF("BATTARY_STATUS\r\n");  
            data[0] = BATTARY_STATUS;
            data[1] = 0x01;      //length
            data[2] = 0x01;      //battatry on using
            break;

        case SET_FM_FREQUENCY:
            GMA_PRINTF("SET_FM_FREQUENCY\r\n");  
            data[0] = SET_FM_FREQUENCY;
            data[1] = 0x01;      //length
            data[2] = 0x00;      //success
            break;

        case GET_FM_FREQUENCY:
            GMA_PRINTF("GET_FM_FREQUENCY\r\n"); 
            data[0] = GET_FM_FREQUENCY;
            data[1] = 0x01;      //length
            data[2] = 0x99;      //FM frequency
            break;
            
        case FIRMWARE_VERSION:
            GMA_PRINTF("FIRMWARE_VERSION\r\n"); 
            data[0] = FIRMWARE_VERSION;
            data[1] = 0x04;      //length
            data[2] = 0x00;      //firmware version
            data[3] = 0x01;      
            data[4] = 0x00;      
            data[5] = 0x04;      
            break;
                
        case DEV_NAME:
            GMA_PRINTF("DEV_NAME\r\n"); 
            data[0] = DEV_NAME;
            data[1] = 0x01;      //length
            data[2] = 0x01;      //name
            break;
            
        case MIC_STATUS:
            GMA_PRINTF("MIC_STATUS\r\n"); 
            data[0] = MIC_STATUS;
            data[1] = 0x01;      //length
            data[2] = 0x00;      //mic unusing
            break;

        default:
            GMA_PRINTF("Dev_param type not support!!!\r\n");
            break;
    }
    gma_reply_dev_param(gma_recv_data->gma_frame_head.msg_id, data, data[1] + 2);
}

#if GMA_OTA
uint16_t genCrc16CCITT(uint16_t crc, uint8_t* data, uint16_t len) 
{        
    uint16_t i;
    for (i = 0; i < len; i++) 
    {        
         crc = ((crc >> 8) | (crc << 8)) & 0xffff;   
         crc ^= (data[i] & 0xff);// byte to int, trunc sign    
         crc ^= ((crc & 0xff) >> 4);      
         crc ^= (crc << 12) & 0xffff;   
         crc ^= ((crc & 0xFF) << 5) & 0xffff;   
    }       
    return (crc&0xffff);
}

uint16_t calccrc1 = 0xFFFF;
uint8_t gma_ota_calc_crc(void)
{
	uint32_t i;
	uint8_t data[256];
	uint8_t data1[5];
	uint32_t read_addr;
	uint16_t calccrc = 0xFFFF;
    uint16_t savecrc = 0;
    uint32_t data_len = 0;
    uint16_t len = 0;
	img_hdr_t ImgHdr;

    read_addr = SEC_IMAGE_BACKUP_OAD_HEADER_FADDR;
    data_len = gma_ota_data.bin_len;
    savecrc = gma_ota_data.bin_crc;

	for (i = 0; i < data_len; )
    {
        if((i + sizeof(data)) <= data_len)
        {
            len = sizeof(data);
        }
        else
        {
            len = data_len - i;
        }
            
        flash_read(FLASH_MAIN_BASE_ADDR, read_addr, len, data,NULL);
        calccrc = genCrc16CCITT(calccrc, data, len);
        read_addr += len;
        i += len;
    }
	calccrc = calccrc1;
	GMA_PRINTF("CRC : calc_crc = %x, save_crc = %x \r\n", calccrc, savecrc);
    if(calccrc == savecrc)
    {
        img_hdr_new.crc = calccrc;
        img_hdr_new.crc_status = CRC_CHECK_OK;

		MESH_APP_PRINT_INFO("--crc check ok---before write rom_ver=0x%x,ver=%x\n",img_hdr_new.rom_ver,img_hdr_new.ver);
		
        //flash_write(0, SEC_IMAGE_BACKUP_OAD_HEADER_FADDR, sizeof(img_hdr_t), (uint8_t *)&img_hdr_new, NULL);

		flash_read(FLASH_MAIN_BASE_ADDR, SEC_IMAGE_BACKUP_OAD_HEADER_FADDR, sizeof(img_hdr_t), (uint8_t *)&ImgHdr,NULL);
		data1[0]= 0;
		data1[1] = (ImgHdr.ver & 0xff);			  //firmware version
		data1[2] = ((ImgHdr.ver >> 8) & 0xff);	  //firmware version
		data1[3] = (ImgHdr.rom_ver & 0xff);		  //firmware version
		data1[4] = ((ImgHdr.rom_ver >> 8) & 0xff); //firmware version
		// Copy the version info to the new version structuer.
		
		//memcpy(&img_hdr_new, &ImgHdr, sizeof(img_hdr_t));
		GMA_DEBUG("read_ota_fw_ver %d: ", sizeof(data1));
		for(i = 0; i < sizeof(data1); i++)
			GMA_DEBUG("%02x ", data1[i]);
		GMA_DEBUG("\r\n");
		
		MESH_APP_PRINT_INFO("ImgHdr.rom_ver = %x \r\n", ImgHdr.rom_ver);
		MESH_APP_PRINT_INFO("ImgHdr.uid = %x \r\n", ImgHdr.uid);
		MESH_APP_PRINT_INFO("ImgHdr.ver = %x \r\n", ImgHdr.ver);

	    return 1;
	}
    else
        return 0;
}

uint8_t gma_ota_is_ongoing(void)
{
    return gma_ota_data.start_flag;
}

void gma_ota_clear_ongoingFlag(void)
{
	gma_ota_data.start_flag = 0;
}

void gma_ota_wdt_reset_flag_set(uint8_t value)
{
    gma_ota_data.wdt_reset_flag = value;
}

void gma_ota_write_flash(void) 
{
    if(gma_ota_is_ongoing() && (gma_ota_data.frame_len))
    {
    	//GMA_PRINTF("frame_len=%d",gma_ota_data.frame_len);
        calccrc1 = genCrc16CCITT(calccrc1, gma_ota_data.frame, gma_ota_data.frame_len);
    	flash_write(FLASH_MAIN_BASE_ADDR, gma_ota_data.data_addr, gma_ota_data.frame_len,gma_ota_data.frame,NULL);
    	gma_ota_data.data_addr += gma_ota_data.frame_len;
    	gma_ota_data.frame_len = 0;
	}
}

void gma_ota_save_data(uint8_t *buf, uint32_t len) 
{
    memcpy(gma_ota_data.frame , buf, len);
    gma_ota_data.frame_len = len;      
}

uint8_t gma_ota_end_result(void) 
{
    uint8_t result = GMA_OTA_FAIL;
	  GMA_PRINTF("OTA_END:\r\n");
	
	  CloseGmaOtaAdv_ClearFlag();
	
    if(gma_ota_calc_crc())
    {
        result = GMA_OTA_SUCCESS;
        GMA_PRINTF("OTA_SUCCESS!!!\r\n");
		wdt_reset(0xFFF);
        //wdt_enable(100);
        //while (1);
    }
    else
    {
        result = GMA_OTA_FAIL;
        GMA_PRINTF("OTA_FAIL!!!\r\n");
#if GMA_SUPPORT		//clean backup flash.
		gma_callback_t cb;
		if ((cb = gma_get_disconn_handler()) != NULL)
		{
			cb();
		}
#endif
    }

    return result;
}

void gma_reply_ota_fw_ver(uint8_t msg_id)
{
    gma_frame_head_s reply_rsp_head;
    uint8_t send_buf[16 + GMA_PAYLOAD_HEAD];
    uint8_t data[5];
    int en_len;
    uint8_t i = 0;
    img_hdr_t ImgHdr;
    
    reply_rsp_head.msg_id = msg_id;
    reply_rsp_head.save_flag = gma_para_proc.auth_flag;
    reply_rsp_head.version = 0;
    reply_rsp_head.cmd = CMD_OTA_FW_VER_RSP;
    reply_rsp_head.fn = 0;
    reply_rsp_head.total_fn = 0;
    data[0] = 0x00;                           // firmware type
    ImgHdr.ver = 0;
    ImgHdr.rom_ver = 0;
    flash_read(FLASH_MAIN_BASE_ADDR, SEC_IMAGE_OAD_HEADER_APP_FADDR, sizeof(img_hdr_t), (uint8_t *)&ImgHdr,NULL);
    data[4] = (ImgHdr.ver & 0xff);            //firmware version
    data[3] = ((ImgHdr.ver >> 8) & 0xff);     //firmware version
    data[2] = (ImgHdr.rom_ver & 0xff);        //firmware version
    data[1] = ((ImgHdr.rom_ver >> 8) & 0xff); //firmware version
    // Copy the version info to the new version structuer.
    
    //memcpy(&img_hdr_new, &ImgHdr, sizeof(img_hdr_t));
    GMA_DEBUG("reply_ota_fw_ver %d: ", sizeof(data));
    for(i = 0; i < sizeof(data); i++)
        GMA_DEBUG("%02x ", data[i]);
    GMA_DEBUG("\r\n");

    MESH_APP_PRINT_INFO("ImgHdr.rom_ver = %x \r\n", ImgHdr.rom_ver);
    MESH_APP_PRINT_INFO("ImgHdr.uid = %x \r\n", ImgHdr.uid);
    MESH_APP_PRINT_INFO("ImgHdr.ver = %x \r\n", ImgHdr.ver);
    if(gma_para_proc.auth_flag)
    {
#if GMA_AES
        aes_iv_set(iv_temp);
        aes_cbc_encrypt_pkcs7(&ali_aes_ctx, gma_para_proc.ble_key, data, sizeof(data), &send_buf[GMA_PAYLOAD_HEAD], &en_len);
#endif
    }
    else
    {
        memcpy(&send_buf[GMA_PAYLOAD_HEAD], data, sizeof(data));
        en_len = sizeof(data);
    }
    
    reply_rsp_head.f_len = en_len;
    memcpy(send_buf, (uint8_t*)&reply_rsp_head, sizeof(gma_frame_head_s));

    gma_data_send(send_buf, en_len + GMA_PAYLOAD_HEAD);
    return;

}

uint8_t gma_recv_ota_fw_ver(gma_recv_data_s *gma_recv_data)
{
    uint8_t de_out[50], i;
    int de_len;
    
    if(gma_recv_data->gma_frame_head.save_flag)
    {
#if GMA_AES
        aes_iv_set(iv_temp);
        aes_cbc_decrypt_pkcs7(&ali_aes_ctx, gma_para_proc.ble_key, gma_recv_data->data, gma_recv_data->gma_frame_head.f_len, de_out, &de_len);
#endif
    }
    else
    {
        memcpy(de_out, gma_recv_data->data, gma_recv_data->gma_frame_head.f_len);
        de_len = gma_recv_data->gma_frame_head.f_len;
    }
 
    GMA_PRINTF("[GMA]:recv_ota_fw_ver %d: ", de_len);
    for(i = 0; i < de_len; i++)
        GMA_PRINTF("%02x ", de_out[i]);
    GMA_PRINTF("\r\n");

    gma_reply_ota_fw_ver(gma_recv_data->gma_frame_head.msg_id);
    return 0;
}

void gma_reply_ota_start(uint8_t msg_id)
{
    gma_frame_head_s reply_rsp_head;
    uint8_t send_buf[16 + GMA_PAYLOAD_HEAD];
    uint8_t data[6];
    int en_len;
    uint8_t i = 0;
    
    reply_rsp_head.msg_id = msg_id;
    reply_rsp_head.save_flag = gma_para_proc.auth_flag;
    reply_rsp_head.version = 0;
    reply_rsp_head.cmd = CMD_OTA_START_RSP;
    reply_rsp_head.fn = 0;
    reply_rsp_head.total_fn = 0;

    data[0] = 0x01;               // ota_start permission
    data[1] = 0x00;               // last receive data addr
    data[2] = 0x00;
    data[3] = 0x00;
    data[4] = 0x00;
    data[5] = 0x0F;               // total frame

    GMA_DEBUG("reply_ota_start %d: ", sizeof(data));
    for(i = 0; i < sizeof(data); i++)
        GMA_DEBUG("%02x ", data[i]);
    GMA_DEBUG("\r\n");

    if(gma_para_proc.auth_flag)
    {
        aes_iv_set(iv_temp);
        aes_cbc_encrypt_pkcs7(&ali_aes_ctx, gma_para_proc.ble_key, data, sizeof(data), &send_buf[GMA_PAYLOAD_HEAD], &en_len);
    }
    else
    {
        memcpy(&send_buf[GMA_PAYLOAD_HEAD], data, sizeof(data));
        en_len = sizeof(data);
    }
    
    reply_rsp_head.f_len = en_len;
    memcpy(send_buf, (uint8_t*)&reply_rsp_head, sizeof(gma_frame_head_s));

    gma_data_send(send_buf, en_len + GMA_PAYLOAD_HEAD);
    return;
}

uint8_t gma_recv_ota_start(gma_recv_data_s *gma_recv_data)
{
    uint8_t de_out[12], i;
    int de_len;
    uint8_t ota_flag;
    uint8_t fw_type;
    uint32_t ver_new;
    uint32_t ver_old;
    img_hdr_t ImgHdr;

    if(gma_recv_data->gma_frame_head.save_flag)
    {
        aes_iv_set(iv_temp);
        aes_cbc_decrypt_pkcs7(&ali_aes_ctx, gma_para_proc.ble_key, gma_recv_data->data, gma_recv_data->gma_frame_head.f_len, de_out, &de_len);
    }
    else
    {
        memcpy(de_out, gma_recv_data->data, gma_recv_data->gma_frame_head.f_len);
        de_len = gma_recv_data->gma_frame_head.f_len;
    }
 
    GMA_PRINTF("recv_ota_start %d: ", de_len);
    for(i = 0; i < de_len; i++)
        GMA_PRINTF("%02x ", de_out[i]);
    GMA_PRINTF("\r\n");

    memset((uint8_t*)&gma_ota_data, 0, sizeof(xm_ota_data_s));
    
	flash_read(FLASH_MAIN_BASE_ADDR, SEC_IMAGE_OAD_HEADER_APP_FADDR, sizeof(img_hdr_t), (uint8_t *)&ImgHdr,NULL);
    ver_old = ImgHdr.ver = 1;
    memcpy((uint8_t *)&fw_type, &de_out[0], 1);
    memcpy((uint8_t *)&ver_new, &de_out[1], 4);
    memcpy((uint8_t *)&gma_ota_data.bin_len, &de_out[5], 4);
    memcpy((uint8_t *)&gma_ota_data.bin_crc, &de_out[9], 2);
    memcpy((uint8_t *)&ota_flag, &de_out[11], 1);
    uint8_t *stream = &de_out[1];
    img_hdr_new.len = gma_ota_data.bin_len;
    img_hdr_new.crc = gma_ota_data.bin_crc;
    STREAM_TO_UINT16(img_hdr_new.rom_ver, stream);
    STREAM_TO_UINT16(img_hdr_new.ver, stream);
    GMA_PRINTF("fw_type:%x, ver_new:%x, ver_old:%x, rom_ver=0x%x,ver=0x%x,total_len:%x, total_crc:%x, ota_flag:%x\r\n",fw_type,ver_new,ver_old,img_hdr_new.rom_ver,img_hdr_new.ver,gma_ota_data.bin_len,gma_ota_data.bin_crc,ota_flag);

	
    if((ver_new != ver_old) && (!ota_flag))
    {
        gma_ota_data.start_flag = 1;
		
        CloseMeshAdv_Proc();
		
        gma_reply_ota_start(gma_recv_data->gma_frame_head.msg_id);
    }
    return 0;
}

void gma_reply_ota_end_check(uint8_t msg_id)
{
    gma_frame_head_s reply_rsp_head;
    uint8_t send_buf[16 + GMA_PAYLOAD_HEAD];
    uint8_t data[1];
    int en_len;
    uint8_t i = 0;
    
    reply_rsp_head.msg_id = msg_id;
    reply_rsp_head.save_flag = gma_para_proc.auth_flag;
    reply_rsp_head.version = 0;
    reply_rsp_head.cmd = CMD_OTA_END_CHECK_RSP;
    reply_rsp_head.fn = 0;
    reply_rsp_head.total_fn = 0;
    
    while(gma_ota_data.frame_len)
    {
    	GMA_PRINTF("-*-in gma_reply");
        gma_ota_write_flash();         //ensure last data write to flash
    }

    data[0] = gma_ota_end_result();    //ota result: 0:fail, 1:success
    
    memset((uint8_t*)&gma_ota_data, 0, sizeof(xm_ota_data_s));
    
    if(data[0] == GMA_OTA_SUCCESS)
    {
        ke_timer_set(APP_PERIOD_TIMER, TASK_APP, 100);  // timeout 1 second to reboot devices
    }

    GMA_DEBUG("reply_ota_end_check %d: ", sizeof(data));
    for(i = 0; i < sizeof(data); i++)
        GMA_DEBUG("%02x ", data[i]);
    GMA_DEBUG("\r\n");

    if(gma_para_proc.auth_flag)
    {
        aes_iv_set(iv_temp);
        aes_cbc_encrypt_pkcs7(&ali_aes_ctx, gma_para_proc.ble_key, data, sizeof(data), &send_buf[GMA_PAYLOAD_HEAD], &en_len);
    }
    else
    {
        memcpy(&send_buf[GMA_PAYLOAD_HEAD], data, sizeof(data));
        en_len = sizeof(data);
    }

    reply_rsp_head.f_len = en_len;
    memcpy(send_buf, (uint8_t*)&reply_rsp_head, sizeof(gma_frame_head_s));

    gma_data_send(send_buf, en_len + GMA_PAYLOAD_HEAD);
    return;
}

uint8_t gma_recv_ota_end_check(gma_recv_data_s *gma_recv_data)
{
    uint8_t de_out[1], i;
    int de_len;

    if(gma_recv_data->gma_frame_head.save_flag)
    {
        aes_iv_set(iv_temp);
        aes_cbc_decrypt_pkcs7(&ali_aes_ctx, gma_para_proc.ble_key, gma_recv_data->data, gma_recv_data->gma_frame_head.f_len, de_out, &de_len);
    }
    else
    {
        memcpy(de_out, gma_recv_data->data, gma_recv_data->gma_frame_head.f_len);
        de_len = gma_recv_data->gma_frame_head.f_len;
    }

    GMA_PRINTF("recv_ota_end_check %d: ", de_len);
    for(i = 0; i < de_len; i++)
        GMA_PRINTF("%02x ", de_out[i]);
    GMA_PRINTF("\r\n");
    
    gma_reply_ota_end_check(gma_recv_data->gma_frame_head.msg_id);
    return 0;
}

void gma_reply_ota_pdu(uint8_t msg_id, uint8_t *data, uint8_t len)
{
    gma_frame_head_s reply_rsp_head;
    uint8_t send_buf[5 + GMA_PAYLOAD_HEAD];

    reply_rsp_head.msg_id = msg_id;
    reply_rsp_head.save_flag = 0;
    reply_rsp_head.version = 0;
    reply_rsp_head.cmd = CMD_OTA_FRAME_LEN_RPT;
    reply_rsp_head.fn = 0;
    reply_rsp_head.total_fn = 0;
    reply_rsp_head.f_len = len;

    memcpy(send_buf, (uint8_t*)&reply_rsp_head, sizeof(gma_frame_head_s));
    memcpy(&send_buf[GMA_PAYLOAD_HEAD], data, len);

    gma_data_send(send_buf, sizeof(send_buf));
    return;
}

uint8_t gma_recv_ota_pdu(gma_recv_data_s *gma_recv_data)
{
    uint8_t data[5];

    if(!gma_ota_is_ongoing())
        return 0;

    if(gma_ota_data.data_len == 0)
    {
        gma_ota_data.data_addr = SEC_IMAGE_BACKUP_OAD_HEADER_FADDR;
    }

    gma_ota_data.data_len += gma_recv_data->gma_frame_head.f_len;
    gma_ota_save_data(gma_recv_data->data, gma_recv_data->gma_frame_head.f_len);
    gma_ota_write_flash();
    if(gma_recv_data->gma_frame_head.fn == gma_recv_data->gma_frame_head.total_fn)
    {
        data[0] = (uint8_t)gma_recv_data->gma_frame_head.fn | (uint8_t)(gma_recv_data->gma_frame_head.total_fn << 4);
        data[1] = (uint8_t)((gma_ota_data.data_len >> 0)  & 0xFF);   //last data length
        data[2] = (uint8_t)((gma_ota_data.data_len >> 8)  & 0xFF);   //last data length
        data[3] = (uint8_t)((gma_ota_data.data_len >> 16) & 0xFF);   //last data length
        data[4] = (uint8_t)((gma_ota_data.data_len >> 24) & 0xFF);   //last data length

        gma_reply_ota_pdu(gma_recv_data->gma_frame_head.msg_id, data, sizeof(data));
    }
    return 0;
}
#endif

void gma_recv_proc(uint8_t *buf, uint16_t len)
{
    gma_recv_data_s* gma_recv_data = (gma_recv_data_s*)buf;
    uint8_t i = 0;

    if(!gma_ota_data.start_flag)
    {
        GMA_DEBUG("\r\ngma_recv: ");
        for(i = 0; i < len; i++)
            GMA_DEBUG("%x ", (unsigned char)buf[i]);
        GMA_DEBUG("\r\n");
    }

    if(gma_para_proc.init_flag)
    {
        gma_para_proc.init_flag = 0;
        gma_para_proc.next_msgid = gma_recv_data->gma_frame_head.msg_id + 1;
    }
    else if((gma_para_proc.next_msgid++) != gma_recv_data->gma_frame_head.msg_id)
    {
        gma_para_proc.next_msgid = gma_recv_data->gma_frame_head.msg_id + 1;
        GMA_PRINTF("msg id error!!!\r\n");
        return;
    }

    switch(gma_recv_data->gma_frame_head.cmd)
    {
        case CMD_APP_REQ:
            gma_recv_app_req(gma_recv_data);
            break;
        case CMD_DEV_RSP:
            gma_recv_dev_rsp(gma_recv_data);
            break;
        case CMD_APP_CMD:
            gma_recv_app_cmd(gma_recv_data);
            break;
        case CMD_APP_MANU_REQ:
            gma_recv_app_manu_req(gma_recv_data);
            break;
#if GMA_AES
        case CMD_AUTH_START_REQ:
            gma_recv_auth_start(gma_recv_data);
            break;
            
        case CMD_AUTH_RESULT_REQ:
            gma_recv_auth_result(gma_recv_data);
            break;
#endif   
        case CMD_AUTH_CONFIRM_NET_REQ:
            gma_recv_auth_confirm_net(gma_recv_data);
            break;
#if GMA_OTA       
        case CMD_OTA_FW_VER_REQ:
            gma_recv_ota_fw_ver(gma_recv_data);
            break;
            
        case CMD_OTA_START_REQ:
            gma_recv_ota_start(gma_recv_data);
            break; 
            
        case CMD_OTA_END_CHECK_REQ:
            gma_recv_ota_end_check(gma_recv_data);
            break;   
            
        case CMD_OTA_PDU:
            gma_recv_ota_pdu(gma_recv_data);
            break;  
#endif      
        case CMD_DEV_INFO_REQ:
            gma_recv_dev_info(gma_recv_data);
            break;   
#if GMA_AES        
        case CMD_ACTIVE_REQ:
            gma_recv_active(gma_recv_data);
            break;
#endif            
        case CMD_STATUS_SET_REQ:
            gma_recv_status_set(gma_recv_data);
            break;
            
        case CMD_DEV_PARAM_REQ:
            gma_recv_dev_param(gma_recv_data);
            break;

        default:
            GMA_PRINTF("GMA cmd not support, cmd:%x!!!\r\n", gma_recv_data->gma_frame_head.cmd);
            break;
    }
}

uint8_t gma_raw_data[200] = {0};
uint16_t gma_raw_data_len = 0;
void gma_recv_decode(uint8_t *buf, uint16_t len)
{
    uint16_t frame_len = 0;
    
    if((len + gma_raw_data_len) <= sizeof(gma_raw_data))
    {
        memcpy(gma_raw_data + gma_raw_data_len, buf, len);
        gma_raw_data_len += len;
    }
    else
    {
        GMA_PRINTF("gma_raw_data_overflow:%d,%d!\r\n", gma_raw_data_len, len);
    }
    
    frame_len = gma_raw_data[3] + GMA_PAYLOAD_HEAD;
    
    while((gma_raw_data_len >= GMA_PAYLOAD_HEAD) && (frame_len <= gma_raw_data_len))
    {
        gma_recv_proc(gma_raw_data, frame_len);
        gma_raw_data_len -= frame_len;
        if(gma_raw_data_len)
        {
            memcpy(gma_raw_data, gma_raw_data + frame_len, gma_raw_data_len);
        }
        frame_len = gma_raw_data[3] + GMA_PAYLOAD_HEAD;
    }
}

void Start_GmaOTA_Adv(void){
	extern void m_prov_bearer_gatt_custom_adv_set(uint8_t *adv_data, uint8_t len);
	extern void m_prov_bearer_gatt_start(void);
	m_prov_bearer_gatt_custom_adv_set(adv_gma_data, sizeof(adv_gma_data)/sizeof(adv_gma_data[0]));
	m_prov_bearer_gatt_start();
}

void Stop_GmaOTA_Adv(void){
	extern void m_prov_bearer_gatt_stop(void);
	m_prov_bearer_gatt_stop();
}


static void gma_ble_disconnect_cb(void)
{
	if(gma_ota_is_ongoing()){
		GLOBAL_INT_STOP();
		oads_erase_backup_sec();
		GLOBAL_INT_START();
		CloseGmaOtaAdv_ClearFlag();
	}
}


static void gma_register_disconnect_cb(gma_callback_t cb)
{
    gma_discon_cb = cb;
}

gma_callback_t gma_get_disconn_handler(void)
{
	GMA_PRINTF("in callback\n");

	
    return gma_discon_cb;
}

static void CloseMeshAdv_Proc(void)
{
	app_mesh_disable();
	MESH_APP_PRINT_INFO("--1app mesh disable");
	m_lay_proxy_bearer_stop();
	MESH_APP_PRINT_INFO("--disable mesh adv\n");
}

void CloseMeshAdv_OpenGmaOtaAdv(void)
{
	flag_global |= Flag_Flash_Start;
	flash_times_cnt = GmaOta_FlashTimes;
	app_mesh_disable();
	m_lay_proxy_bearer_stop();
	
	Start_GmaOTA_Adv();
	MESH_APP_PRINT_INFO("--disable mesh adv,open gmaOta adv\n");
}

static void CloseGmaOtaAdv_OpenMeshAdv(void)
{
	void m_prov_bearer_gatt_start(void);
	MESH_APP_PRINT_INFO("-**-disable gmaOta adv,open mesh adv\n");
	app_mesh_enable();
	m_prov_bearer_gatt_start();
	Stop_GmaOTA_Adv();
}

static void CloseGmaOtaAdv_ClearFlag(void)
{
	gma_ota_clear_ongoingFlag();
	flag_gma &= (~Flag_InGmaState);
	CloseGmaOtaAdv_OpenMeshAdv();
}


#endif
