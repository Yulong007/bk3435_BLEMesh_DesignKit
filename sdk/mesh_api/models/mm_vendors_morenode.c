/**
 ****************************************************************************************
 * @file mm_vendors.c
 *
 * @brief Mesh Model vendor Server Module
 *
 * Copyright (C) Beken 2019-2030
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup MM_VENDORS
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "mm_vendors_morenode.h"
#include "co_utils.h"
#include "m_tb_key.h"
#include "mesh_log.h"
#include "app_mesh.h"
#include "m_config.h"
#include "m_tb_store.h"
#include "m_tb_mio.h"
#include "gpio.h"
#include "m_lay_int.h"
#include "lld_adv_test.h"
#include "mesh_general_api.h"

/*
 * GLOBAL FUNCTIONS
 ****************************************************************************************
 */
#define MM_VENDOR_3B_OPCODE(opcode)                                                             \
    ((uint32_t)(opcode))



#define MM_IS_VENDOR_OPCODE(opcode)       ((((opcode) & 0xFFFF00) >> 8) == 0x01A8)
static bool is_cfm_msg_recved  = false;

extern m_lid_t g_vdr_lid,g_oo_mdl_lid;
extern uint16_t othercmd;

static m_lid_t vdr_lid;
uint16_t genoff_rx_calc = 0;
uint16_t vendor_20bytes_tx_calc = 0;
uint8_t vendor_rspdata[20];  
uint8_t * get_attr_info(mm_vendors_env_t *p_env_vdr, uint16_t attr_type, uint8_t *len, uint8_t opcode)
{
    MESH_APP_PRINT_DEBUG("__^^__%s,attr_type = 0x%x,opcode =0x%x\n",__func__,attr_type,opcode);
    for (uint32_t i = 0; i< MM_VENDORS_ATTR_MAX_NUM; i++)
    {
        if ((p_env_vdr->attr[i].attr_type == attr_type)&&(p_env_vdr->attr[i].attr_type != 0))
        {
        	MESH_APP_PRINT_DEBUG("__^^__%s 2\n",__func__);
        	if((attr_type == 0x88) || (attr_type == ONOFF_ONEGROUP_GET_ATTRTYPE))  ///191230 ///20 0122
        	{
                p_env_vdr->attr[i].attr_param[0] = genoff_rx_calc;
                p_env_vdr->attr[i].attr_param[1] = genoff_rx_calc >> 8;
        	}

		if((attr_type == 0x8a) || (attr_type == VENDOR_ONEGROUP_GET_ATTRTYPE)) ///191230
        {
            p_env_vdr->attr[i].attr_param[0] = vendor_20bytes_tx_calc;
            p_env_vdr->attr[i].attr_param[1] = vendor_20bytes_tx_calc >> 8;
        }

		//if((attr_type == VENDOR_ONENODE_SEND_ATTRTYPE) && (opcode == 1)) ///add 20 0204
		if((attr_type == VENDOR_SNED_RSP_ATTRTYPE) && (opcode == 1))	///VENDOR_SNED_RSP_ATTRTYPE
		{
			for(uint8_t j = 0; j < 18; j++)
			{
				vendor_rspdata[j+1] = 0x20+j+1;
			}
			vendor_rspdata[0]++;  
			MESH_APP_PRINT_DEBUG("vendor rspdata[0] = 0x%x\n",vendor_rspdata[0]); ///add 0206
			memcpy(&p_env_vdr->attr[i].attr_param[0],vendor_rspdata,2);///2 -> 14
		}
		
		#if 0
		if(attr_type == 0x8b)  ///200110
		{
			 m_tb_mio_get_publi_param(g_vdr_lid,gdst_temp,NULL,
							  NULL,NULL,NULL,
							  NULL,NULL);
			MESH_APP_PRINT_INFO("^^__attr type 8b UART_VENDOR_MOREGROUP_SEND GROUP ID = 0x%x\n",gdst_temp);
			if(gdst_temp >= 0xc000)
			{
				//user_models_publish_set(g_oo_mdl_lid, gdst,send_period);
				 user_models_publish_set_vendor(g_vdr_lid,gdst_temp);
			}
		}
		#endif

            *len = p_env_vdr->attr[i].attr_len;
            return p_env_vdr->attr[i].attr_param;
        }
    }
    *len = 0;
    return NULL;

}


uint8_t get_vendor_tid(mm_vendors_env_t *p_env_vdr)
{
    return p_env_vdr->tid++;
}


uint8_t find_index_by_type(mm_vendors_env_t *p_env_vdr)
{
    int i = 0;
    for (; i < MM_VENDORS_ATTR_MAX_NUM; i ++)
    {
        if (p_env_vdr->cur_attr_type == p_env_vdr->attr[i].attr_type)
        {
            return i;
        }
    }
    return i;
}

/**
 ****************************************************************************************
 * @brief Prepare and send a Light Lightness Status or a Light Lightness Linear Status message.
 * Note that both messages have the same content.
 *
 * @param[in] p_env_ln          Pointer to Light Lightness Server model environment
 * @param[in] p_route_env       Pointer to structure containing reception information provided
 * by Mesh Profile for received request message
 * @param[in] publish           Indicate if message is sent as a publication (true) or as
 * a response to a received request (false)
 * @param[in] linear            True if Light Lightness Status must be sent, else False
 ****************************************************************************************
 */
__STATIC void mm_vendors_send_status(mm_vendors_env_t *p_env_vdr,
                                     mm_route_buf_env_t *p_route_env, uint16_t publish, uint16_t attr_type)
{
    MESH_MODEL_PRINT_DEBUG("%s\r\n", __func__);

    // Pointer to the buffer that will contain the message
    mesh_tb_buf_t *p_buf_status;
    // Remaining time
    uint8_t rem_time;
    // Transition type
    uint8_t trans_type;
    // Data length
    uint8_t data_length;

    // Check if a transition has been started
    mm_tb_bind_get_trans_info(p_env_vdr->env.grp_lid, &trans_type, &rem_time);

    // Deduce deduce data length
    uint8_t *attr_param = NULL;
    attr_param = get_attr_info(p_env_vdr,attr_type,&data_length,1);

    MESH_MODEL_PRINT_DEBUG("attr_param :0x%x,attr_type:0x%x,data_length:%d\r\n",attr_param,attr_type,data_length);
    if (attr_param)
    {
        if (mm_route_buf_alloc(&p_buf_status, data_length + 3) == MESH_ERR_NO_ERROR)
        {
            // Get pointer to data
            uint8_t *p_data = MESH_TB_BUF_DATA(p_buf_status);
            // Get pointer to environment
            mm_route_buf_env_t *p_buf_env = (mm_route_buf_env_t *)&p_buf_status->env;
            // Prepare environment
            if (p_route_env)
            {
                memcpy(p_buf_env, p_route_env, sizeof(mm_route_buf_env_t));
            }
            else if (!publish)
            {
                p_buf_env->app_key_lid = p_env_vdr->status_app_key_lid;
                p_buf_env->u_addr.dst = p_env_vdr->status_dst_addr;
                SETB(p_buf_env->info, MM_ROUTE_BUF_INFO_RELAY, p_env_vdr->status_relay);
            }
            SETB(p_buf_env->info, MM_ROUTE_BUF_INFO_RX, 0);
            SETB(p_buf_env->info, MM_ROUTE_BUF_INFO_PUBLISH, publish);
            p_buf_env->mdl_lid = p_env_vdr->env.mdl_lid;
            p_buf_env->opcode = MM_VENDOR_3B_OPCODE(MM_MSG_VENDOR_ATTR_ATTR_STATUS);
            // Fill the message
            *p_data = get_vendor_tid(p_env_vdr);
            co_write16p(p_data + MM_VENDORS_STATUS_ATTR_TYPE_POS, attr_type);
            memcpy(p_data + MM_VENDORS_STATUS_ATTR_PARAM_POS,attr_param,data_length);
            // Send the message
            mm_route_send(p_buf_status);

            MESH_MODEL_PRINT_DEBUG("mm_route_send satus\r\n");
        }
    }
}

uint8_t vendor_senddata[22];
__STATIC void mm_vendors_send_data(mm_vendors_env_t *p_env_vdr,
                                     mm_route_buf_env_t *p_route_env, uint16_t publish, uint16_t attr_type)
{
    MESH_MODEL_PRINT_DEBUG("%s\r\n", __func__);

    // Pointer to the buffer that will contain the message
    mesh_tb_buf_t *p_buf_status;
    // Remaining time
    uint8_t rem_time;
    // Transition type
    uint8_t trans_type;
    // Data length
    uint8_t data_length;
	
    for(uint8_t i = 0; i < 19; i++)
    {
        vendor_senddata[i+1] = i+1;
    }
    vendor_senddata[0]++;

    #if 0
    if(othercmd == VENDOR_RSP_ENABLE_CMD)
    {
    	vendor_senddata[1] = VENDOR_RSP_ENABLE_CMD;
    }
    #endif

    MESH_MODEL_PRINT_DEBUG("vendor_senddata[0] = %d\n",vendor_senddata[0]);
 
    // Check if a transition has been started
    mm_tb_bind_get_trans_info(p_env_vdr->env.grp_lid, &trans_type, &rem_time);

    // Deduce deduce data length
    uint8_t *attr_param = NULL;
    attr_param = get_attr_info(p_env_vdr,attr_type,&data_length,0);

    MESH_MODEL_PRINT_DEBUG("attr_param :0x%x,attr_type:0x%x,data_length:%d\r\n",attr_param,attr_type,data_length);
    if (attr_param)
    {
        if (mm_route_buf_alloc(&p_buf_status, data_length + 3) == MESH_ERR_NO_ERROR)
        {
            // Get pointer to data
            uint8_t *p_data = MESH_TB_BUF_DATA(p_buf_status);
            // Get pointer to environment
            mm_route_buf_env_t *p_buf_env = (mm_route_buf_env_t *)&p_buf_status->env;
            // Prepare environment
            if (p_route_env)
            {
                memcpy(p_buf_env, p_route_env, sizeof(mm_route_buf_env_t));
            }
            else if (!publish)
            {
                p_buf_env->app_key_lid = p_env_vdr->status_app_key_lid;
                p_buf_env->u_addr.dst = p_env_vdr->status_dst_addr;
                SETB(p_buf_env->info, MM_ROUTE_BUF_INFO_RELAY, p_env_vdr->status_relay);
            }
            SETB(p_buf_env->info, MM_ROUTE_BUF_INFO_RX, 0);
            SETB(p_buf_env->info, MM_ROUTE_BUF_INFO_PUBLISH, publish);
            p_buf_env->mdl_lid = p_env_vdr->env.mdl_lid;
            //p_buf_env->opcode = MM_VENDOR_3B_OPCODE(MM_MSG_VENDOR_ATTR_ATTR_STATUS);
            p_buf_env->opcode = MM_VENDOR_3B_OPCODE(MM_MSG_VENDOR_ATTR_ATTR_SET);  ///change 2020 0108
			
            // Fill the message
            *p_data = get_vendor_tid(p_env_vdr);
	     #if 0
            co_write16p(p_data + MM_VENDORS_STATUS_ATTR_TYPE_POS, attr_type);
            memcpy(p_data + MM_VENDORS_STATUS_ATTR_PARAM_POS,attr_param,data_length);
	     #else
	     //co_write16p(p_data + MM_VENDORS_STATUS_ATTR_TYPE_POS, 0x123);   ///change 2020 0108 //0212 VENDOR_ONENODE_SEND_ATTRTYPE
	     if(othercmd == VENDOR_RSP_ENABLE_CMD)
	     {
	     		//MESH_APP_PRINT_INFO("__^^__memcpy 14\n");
			co_write16p(p_data + MM_VENDORS_STATUS_ATTR_TYPE_POS, VENDOR_SNED_RSP_ATTRTYPE);   ///change 2020 0108 //0212 VENDOR_ONENODE_SEND_ATTRTYPE
            		memcpy(p_data + MM_VENDORS_STATUS_ATTR_PARAM_POS,vendor_senddata,VENDOR_SEND_DATA_LENGTH); ///2->14 ///for two package
	     }
	     else
	     {
			//MESH_APP_PRINT_INFO("__^^__memcpy 14\n");
			co_write16p(p_data + MM_VENDORS_STATUS_ATTR_TYPE_POS, VENDOR_ONENODE_SEND_ATTRTYPE);   ///change 2020 0108 //0212 VENDOR_ONENODE_SEND_ATTRTYPE
			memcpy(p_data + MM_VENDORS_STATUS_ATTR_PARAM_POS, vendor_senddata, VENDOR_SEND_DATA_LENGTH); ///2->14 ///for two package
	     }
	     #endif
            // Send the message
            mm_route_send(p_buf_status);
            MESH_MODEL_PRINT_DEBUG("mm_route_send satus\r\n");
        }
    }
}


/**
 ****************************************************************************************
 * @brief Publish vendors state value if sending of publications is enabled
 *
 * @param[in] p_env_oo          Pointer to vendors Server model environment
 ****************************************************************************************
 */
__STATIC void mm_vendors_publish(mm_vendors_env_t *p_env_vdr)
{
    MESH_MODEL_PRINT_DEBUG("mm_vendors_publish ^^00\r\n");
    // Check if sending of publication is enabled
    if (GETB(p_env_vdr->env.info, MM_TB_STATE_MDL_INFO_PUBLI))
    {
        //mm_vendors_send_status(p_env_vdr, NULL, true, p_env_vdr->cur_attr_type);
    }
}

__STATIC void mm_vendors_publish_data(mm_vendors_env_t *p_env_vdr)
{
    MESH_MODEL_PRINT_DEBUG("mm_vendors_publish ^^01\r\n");
    // Check if sending of publication is enabled
    if (GETB(p_env_vdr->env.info, MM_TB_STATE_MDL_INFO_PUBLI))
    {
        //mm_vendors_send_status(p_env_vdr, NULL, true, p_env_vdr->cur_attr_type);
        mm_vendors_send_data(p_env_vdr, NULL, true, p_env_vdr->cur_attr_type);
    }
}

/*
 * MESSAGE HANDLERS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Check if a Generic OnOff Status message must be sent and send it if it
 * is the case
 *
 * @param[in] p_env_oo        Pointer to Generic OnOff Server model environment
 ****************************************************************************************
 */
__STATIC void mm_vendors_check_status_rsp(mm_vendors_env_t *p_env_vdr)
{
    if (p_env_vdr->status_dst_addr)
    {
    	    MESH_MODEL_PRINT_DEBUG("%s\r\n ^^02",__func__);

        // Send a response to the node that has required the transition
        mm_vendors_send_status(p_env_vdr, NULL, false,p_env_vdr->cur_attr_type); ///open 2020 0121

        p_env_vdr->status_dst_addr = MESH_UNASSIGNED_ADDR;
    }
}

/**
 ****************************************************************************************
 * @brief Inform Generic OnOff Server model about received publication parameters
 *
 * @param[in] p_env         Pointer the the environment allocated for the Generic OnOff
 * Server model
 * @param[in] addr          Publication address
 * @param[in] period_ms     Publication period in milliseconds
 ****************************************************************************************
 */
__STATIC void mm_vendors_cb_publish_param(mm_tb_state_mdl_env_t *p_env, uint16_t addr,
        uint32_t period_ms)
{
    // Inform the state manager about received publication parameters
    MESH_MODEL_PRINT_DEBUG("mm_vendors_cb_publish_param addr:%x,period_ms:%d\r\n",addr, period_ms);
    mm_tb_state_publish_param_ind((mm_tb_state_mdl_publi_env_t *)p_env, addr, period_ms); ///add 2020 0108
}
/**
 ****************************************************************************************
 * @brief Handler for vENDORS Set/Set Unacknowledged
 * Set/Set Unacknowledged message.
 * Note that all messages have the same content.
 *
 * @param[in] p_env         Pointer to environment of model for which message has been received
 * @param[in] p_buf         Pointer to buffer containing the received message
 * @param[in] p_route_env   Pointer to routing information for the received buffer
 ****************************************************************************************
 */
extern uint8_t p_va_lid_test;                                
extern uint8_t p_app_key_lid_test; 
extern uint8_t p_ttl_test;
extern uint8_t p_period_test;                            
extern uint8_t p_retx_params_test;
extern uint8_t p_friend_cred_test;
extern uint16_t send_nb;  
extern uint16_t send_period;
extern uint16_t nodedst;

extern uint8_t nvds_ctlset_value;
extern uint8_t nvds_ctlset_value_len;

extern uint16_t g_app_key_id;


void app_onegroup_get_timer_set(uint32_t timer);

uint8_t vendor_led_flag = 0;

uint16_t onegroup_data[ONEGROUP_NODE_MAX][2];
uint16_t onegroup_node_calc = 0;

uint16_t vendor_rsp_calc = 0;

mesh_tb_timer_t vendor_node_send_delay_timer;
mesh_tb_timer_t onoff_node_send_delay_timer;
mesh_tb_timer_t vendor_rsp_send_delay_timer;


uint16_t gdst_temp;

#include "lld_evt.h"
#include "m_config.h"
#include "m_tb_store.h"

uint16_t user_models_publish_set(uint16_t app_keyid,m_lid_t m_lid, uint16_t addr,uint8_t period);

__STATIC void vendor_node_send_delay_cb(void *p_timer)
{
    MESH_APP_PRINT_DEBUG("++++ %s +++\n", __func__);
    user_models_publish_set(g_app_key_id,g_vdr_lid,gdst_temp,send_period);
}

__STATIC void onoff_node_send_delay_cb(void *p_timer)
{
    MESH_APP_PRINT_DEBUG("++++ %s +++\n", __func__);
    user_models_publish_set(g_app_key_id,g_oo_mdl_lid,gdst_temp,send_period);
    //user_models_publish_set(g_oo_mdl_lid,gdst_temp,send_period); ///add 2020 0111
}

mm_vendors_env_t *p_env_vdr_vendor_rsp;
mm_route_buf_env_t *p_route_env_vendor_rsp;
uint16_t gdst_vendor_rsp = 0;
uint16_t node_net_addr = 0;
extern uint8_t vendor_data[5];
__STATIC void vendor_rsp_send_delay_cb(void *p_timer)   ///add 2020 0204 17:28
{ 
    MESH_APP_PRINT_DEBUG("++++ %s +++\n", __func__);
    #if 0
    mm_vendors_send_status(p_env_vdr_vendor_rsp, p_route_env_vendor_rsp, false,VENDOR_SNED_RSP_ATTRTYPE);
    #else
   if(gdst_vendor_rsp < 0xc000)
   {
    	m_tb_mio_get_publi_param(g_vdr_lid,&gdst_vendor_rsp,
								&p_va_lid_test,              
								&p_app_key_lid_test,
								&p_ttl_test,
								&p_period_test,                            
								&p_retx_params_test,
								&p_friend_cred_test);
	m_tb_mio_get_element_addr(g_vdr_lid,&node_net_addr);
	MESH_APP_PRINT_DEBUG("node_net_addr = 0x%x\n",node_net_addr);
   }
   
    if(gdst_vendor_rsp  >= 0xc000)
    {
    		vendor_rspdata[0]++;
		vendor_rspdata[1] = node_net_addr;
		vendor_rspdata[2] = node_net_addr >> 8;
    		mm_vendor_attr_indication_publish(VENDOR_SNED_RSP_ACK_ATTRTYPE,VENDOR_SEND_DATA_LENGTH,vendor_rspdata,gdst_vendor_rsp);  ///add 2020 0118 ///14 -> 2	 
    }
    #endif
    mal_free(p_env_vdr_vendor_rsp);
    mal_free(p_route_env_vendor_rsp);
    p_env_vdr_vendor_rsp = NULL;
    p_route_env_vendor_rsp = NULL;
}

extern void Delay_ms(int num) ;
__STATIC void mm_vendors_handler_set(mm_vendors_env_t *p_env_vdr, mesh_tb_buf_t *p_buf,
                                     mm_route_buf_env_t *p_route_env)
{
    uint16_t vendor_data_total_value = 0;
    uint8_t txnode_net_tx_count = 0;  //add 0216
    uint8_t txnode_low_tx_count = 0;     ///add 0304
	
    //uint16_t gdst_temp;
    do
    {
        // Get pointer to data
        uint8_t *p_data = MESH_TB_BUF_DATA(p_buf);
        uint8_t data_len;
        uint8_t trans_time = 0;
        uint8_t delay = 0;

        // Extract TID value
        uint8_t tid = *(p_data + MM_VENDORS_STATUS_TID_POS);

        uint16_t attr_type =  co_read16(p_data + MM_VENDORS_STATUS_ATTR_TYPE_POS);
        MESH_MODEL_PRINT_DEBUG("vendor model,attr_type: 0x%x\r\n", attr_type);
        uint8_t * attr_param = get_attr_info(p_env_vdr,attr_type,&data_len,0);

        MESH_MODEL_PRINT_DEBUG("vendor model,p_buf->data_len: %d\r\n", p_buf->data_len);
        MESH_MODEL_PRINT_DEBUG("vendor model,p_buf->tail_len: %d\r\n", p_buf->tail_len);
        MESH_MODEL_PRINT_DEBUG("vendor model,p_buf->head_len: %d\r\n", p_buf->head_len);
        MESH_MODEL_PRINT_DEBUG("p_buf DATA:");

	//if(attr_type == 0x8b)  ///200110
	if(attr_type == VENDOR_MOREGROUP_SEND_ATTRTYPE)  ///200110
	{
		m_tb_mio_get_publi_param(g_vdr_lid,&gdst_temp,
								&p_va_lid_test,                             
								&p_app_key_lid_test,
								&p_ttl_test,
								&p_period_test,                            
								&p_retx_params_test,
								&p_friend_cred_test);
		send_nb = p_data[3];
		send_nb <<= 8;
		send_nb |= p_data[4];
		send_period	=	p_data[5];
		
		#if 0
		m_tb_state_set_relay_state(M_CONF_RELAY_STATE_DISABLED, M_CONF_RELAY_STATE_DISABLED);
		#endif
		txnode_net_tx_count = p_data[6];  ///add 0216
		txnode_low_tx_count = p_data[7];     ///add 0304
		if (txnode_net_tx_count)
    		{
         		m_tb_state_set_net_tx_count(txnode_net_tx_count);
    		}
		
		if (txnode_low_tx_count)
		{
		    m_lay_ltrans_set_nb_set_retrans(txnode_low_tx_count);
		}
		
		MESH_APP_PRINT_INFO(" txnode_net_tx_count = %d\n",txnode_net_tx_count);
		MESH_APP_PRINT_INFO(" txnode_low_tx_count = %d\n",txnode_low_tx_count);
		MESH_APP_PRINT_INFO("nb_retrans %d\n", m_lay_ltrans_get_nb_retrans());
		MESH_APP_PRINT_DEBUG("^^__attr type 8b UART_VENDOR_MOREGROUP_SEND GROUP ID = 0x%x,send_nb = %d,send_period = %d\n",gdst_temp,send_nb,send_period);
		
		  if(gdst_temp >= 0xc000)
		  {
			 //m_tb_store_config(0);
			 //uint32_t time_slot;
			// uint32_t time_us;
			 //lld_evt_time_get_us(&time_slot, &time_us);
			 //uint32_t delay = (rand() + time_slot + time_us) % 470 + 10;

			#if 0
			if(!nvds_get(NVDS_TAG_MESH_CTLSET_VALUE,&nvds_ctlset_value_len,&nvds_ctlset_value))  ///add 0305
			{
				MESH_APP_PRINT_INFO("****get nvds CTLSET value = %d\n",nvds_ctlset_value);
			}
			else
			{
				MESH_APP_PRINT_INFO("****get nvds CTLSET value Error\n");
			}
			#endif
			 
			 vendor_node_send_delay_timer.cb = vendor_node_send_delay_cb;
			 vendor_node_send_delay_timer.time_ms = nvds_ctlset_value*10;
			 MESH_APP_PRINT_INFO("%s, delay = %d\n", __func__, nvds_ctlset_value*10);
			 mesh_tb_timer_set(&vendor_node_send_delay_timer, nvds_ctlset_value*10);
			 //user_models_publish_set(g_vdr_lid,gdst_temp,send_period); ///add 2020 0111
      		  }
	}

	if(attr_type == ONOFF_MOREGROUP_SEND_ATTRTYPE)  ///200110
	{
		m_tb_mio_get_publi_param(g_oo_mdl_lid,&gdst_temp,
								&p_va_lid_test,                             
								&p_app_key_lid_test,
								&p_ttl_test,
								&p_period_test,                            
								&p_retx_params_test,
								&p_friend_cred_test);
		send_nb = p_data[3];
		send_nb <<= 8;
		send_nb |= p_data[4];
		
		send_period	= p_data[5];
		
		#if 0
		m_tb_state_set_relay_state(M_CONF_RELAY_STATE_DISABLED, M_CONF_RELAY_STATE_DISABLED);
		#endif
		txnode_net_tx_count = p_data[6];      ///add 0216
		txnode_low_tx_count = p_data[7];     ///add 0304
		if (txnode_net_tx_count)
		{
			m_tb_state_set_net_tx_count(txnode_net_tx_count);
		}
		
    		if (txnode_low_tx_count)
    		{
            		m_lay_ltrans_set_nb_set_retrans(txnode_low_tx_count);
    		}
			
		MESH_APP_PRINT_INFO(" txnode_net_tx_count = %d\n",txnode_net_tx_count);
		MESH_APP_PRINT_INFO(" txnode_net_tx_count = %d\n",txnode_net_tx_count);
		MESH_APP_PRINT_INFO("nb_retrans %d\n", m_lay_ltrans_get_nb_retrans);
		MESH_APP_PRINT_DEBUG("^^__attr type 8D UART_ONOFF_MOREGROUP_SEND GROUP ID = 0x%x,send_nb = %d,send_period = %d\n",gdst_temp,send_nb,send_period);
		
		if(gdst_temp >= 0xc000)
		{
			//m_tb_store_config(0);
			//uint32_t time_slot;
			//uint32_t time_us;
			//lld_evt_time_get_us(&time_slot, &time_us);
			//uint32_t delay = (rand() + time_slot + time_us) % 470 + 10;

			#if 0
			if(!nvds_get(NVDS_TAG_MESH_CTLSET_VALUE,&nvds_ctlset_value_len,&nvds_ctlset_value))  ///add 0305
			{
				MESH_APP_PRINT_INFO("****get nvds CTLSET value = %d\n",nvds_ctlset_value);
			}
			else
			{
				MESH_APP_PRINT_INFO("****get nvds CTLSET value Error\n");
			}
			#endif

			onoff_node_send_delay_timer.cb = onoff_node_send_delay_cb;
			onoff_node_send_delay_timer.time_ms = nvds_ctlset_value*10;
			MESH_APP_PRINT_INFO("%s, delay = %d\n", __func__, nvds_ctlset_value*10);
			mesh_tb_timer_set(&onoff_node_send_delay_timer, nvds_ctlset_value*10);
			//user_models_publish_set(g_oo_mdl_lid,gdst_temp,send_period); ///add 2020 0111
		}
	}

	MESH_APP_PRINT_DEBUG("==%d\n",p_buf->data_len);
	#if 0
	for (int i=0; i<p_buf->data_len; i++)
	{
			MESH_APP_PRINT_INFO("0x%x ", p_data[i]); ///2020 0203 08:26
	}
	MESH_APP_PRINT_INFO("p_route_env->opcode = 0x%x,attr_type=0x%x\r\n",p_route_env->opcode,attr_type);
	#endif
		
	if(((attr_type == VENDOR_ONENODE_SEND_ATTRTYPE) || (attr_type == VENDOR_SNED_RSP_ATTRTYPE)) && (p_route_env->opcode == MM_MSG_VENDOR_ATTR_ATTR_SET)) ///modify 2020 0204
	{
		//gpio_set(0x31, vendor_20bytes_tx_calc % 2);
		m_tb_state_set_relay_state(M_CONF_RELAY_STATE_DISABLED, M_CONF_RELAY_STATE_DISABLED); ///add 0216
		#if(LED_SHOW_P1_ENABLE)
		gpio_set(0x10, 1);
		gpio_set(0x11, p_data[3] % 2);
		gpio_set(0x12, 1);
		#else
		gpio_set(0x31, p_data[3] % 2);
		#endif
		vendor_20bytes_tx_calc++;
		//m_tb_store_config(0);
		//MESH_APP_PRINT_INFO("vendor_rx_calc = %d\n",vendor_20bytes_tx_calc);
		//MESH_APP_PRINT_INFO("source addr = 0x%x,vendor_rx_calc = %d\n",p_route_env->u_addr.src,vendor_20bytes_tx_calc); //change 0216
		MESH_APP_PRINT_INFO("R%d\n",vendor_20bytes_tx_calc);

		#if 0
		Delay_ms(1);
		MESH_APP_PRINT_DEBUG("__^^__ vendor rsp send 2!\n");
		mm_vendors_send_status(p_env_vdr, p_route_env, false,VENDOR_ONENODE_SEND_ATTRTYPE);  ///2020 0204 17:27
		#endif
	}

	  if((p_route_env->opcode == MM_MSG_VENDOR_ATTR_ATTR_SET) && (attr_type == VENDOR_SNED_RSP_ACK_ATTRTYPE))
	  {
	  	vendor_rsp_calc ++;
	  	//MESH_APP_PRINT_INFO("vendor rsp source addr = 0x%x,vendor_rsp_calc = %d!\n",p_route_env->u_addr.src,vendor_rsp_calc); ///add 2020 0204
	  	MESH_APP_PRINT_INFO("Rsp%d!\n",vendor_rsp_calc);
	  }

        // Check if received message is a retransmitted one, if state is modified and if
        // a new transition can be started now
        ///for test ///0219
        MESH_APP_PRINT_DEBUG("&&&&p_env_vdr->status_dst_addr = 0X%x\n", p_env_vdr->status_dst_addr);
	 MESH_APP_PRINT_DEBUG("&&&&mm_tb_replay_is_retx bool = 0X%x\n", mm_tb_replay_is_retx(&p_env_vdr->replay_env, p_route_env->u_addr.src, tid));	
	 MESH_APP_PRINT_DEBUG("&&& attr_param bool = 0x%x\n",attr_param);
	 
        if (1
		//(p_env_vdr->status_dst_addr != MESH_UNASSIGNED_ADDR)
                //|| mm_tb_replay_is_retx(&p_env_vdr->replay_env, p_route_env->u_addr.src, tid)
                /*|| (!memcmp(attr_param, p_data + MM_VENDORS_STATUS_ATTR_PARAM_POS,data_len))*/
                //|| (attr_param == NULL)
                )
        {
            MESH_APP_PRINT_DEBUG("__^^__ vendor rsp send 1!\n");
            if (p_route_env->opcode == MM_MSG_VENDOR_ATTR_ATTR_SET)
            {
                // Send Generic OnOff status message
            if(
			(attr_type == ONOFF_ONENODE_GET_ATTRTYPE) 
			||(attr_type == ONOFF_ONEGROUP_GET_ATTRTYPE) ///add 0220
			||(attr_type == VENDOR_ONENODE_GET_ATTRTYPE)||(attr_type == VENDOR_ONEGROUP_GET_ATTRTYPE)
		    )  ///add 2020 0110
                {
                	mm_vendors_send_status(p_env_vdr, p_route_env, false,attr_type);
                }
				
		  if(
		  	(attr_type == VENDOR_SNED_RSP_ATTRTYPE)  ///add 2020 0204
		  		//&&(p_data[4] == VENDOR_RSP_ENABLE_CMD)	///add 2020 0212
            		)
		  
		  {
		  	#if 1
			p_env_vdr_vendor_rsp = mal_malloc(sizeof(mm_vendors_env_t));
			p_route_env_vendor_rsp = mal_malloc(sizeof(mm_route_buf_env_t));
			if (p_env_vdr_vendor_rsp && p_route_env_vendor_rsp)
			{
                		memcpy(p_env_vdr_vendor_rsp, p_env_vdr, sizeof(mm_vendors_env_t));
                		memcpy(p_route_env_vendor_rsp, p_route_env, sizeof(mm_route_buf_env_t));
			}
			//uint32_t time_slot;
			//uint32_t time_us;
			//lld_evt_time_get_us(&time_slot, &time_us);
			//uint32_t delay = (rand() + time_slot + time_us) % 470 + 10;
			if(m_tb_state_get_net_tx_count() != 1)
			{
				m_tb_state_set_net_tx_count(1);
			}
			
                      if(m_lay_ltrans_get_nb_retrans() != 3)
                      {
                      	m_lay_ltrans_set_nb_set_retrans(3);
                      }

			#if 0
			if(!nvds_get(NVDS_TAG_MESH_CTLSET_VALUE,&nvds_ctlset_value_len,&nvds_ctlset_value))  ///add 0305
			{
				MESH_APP_PRINT_INFO("****get nvds CTLSET value = %d\n",nvds_ctlset_value);
			}
			else
			{
				MESH_APP_PRINT_INFO("****get nvds CTLSET value Error\n");
			}
			#endif
			
			vendor_rsp_send_delay_timer.cb = vendor_rsp_send_delay_cb;
			vendor_rsp_send_delay_timer.time_ms = nvds_ctlset_value*10;
			MESH_APP_PRINT_INFO("=%d\n",nvds_ctlset_value*10);
			mesh_tb_timer_set(&vendor_rsp_send_delay_timer, nvds_ctlset_value*10);
			
			#else
			Delay_ms(1);
			MESH_APP_PRINT_DEBUG("__^^__ vendor rsp send 2!\n");
		  	mm_vendors_send_status(p_env_vdr, p_route_env, false,VENDOR_ONENODE_SEND_ATTRTYPE);  ///2020 0204 17:27
			#endif
		  }
		  
                MESH_MODEL_PRINT_DEBUG("%s  ^^03\r\n",__func__);
            }
            //break; ///0219 10:32
        }

	  if( (p_route_env->opcode == MM_MSG_VENDOR_ATTR_ATTR_STATUS) && ((attr_type == 0x8a) ||(attr_type ==VENDOR_ONEGROUP_GET_ATTRTYPE)))
	  {
	  	       vendor_data_total_value = p_data[3] + (p_data[4] << 8);
			MESH_APP_PRINT_DEBUG("source addr = 0x%x,Rx Vendor data NB = %d\n",p_route_env->u_addr.src,vendor_data_total_value);		///change 20200106
			
		  	//onegroup node_calc
		  	for(uint8_t i = 0; i< send_nb; i++)
		  	{
		  		if(onegroup_data[i][0] == p_route_env->u_addr.src)
		  		{
		  			MESH_APP_PRINT_DEBUG("__^^__%s return\n",__func__);
		  			return;
		  		}
		  	}
			
			onegroup_data[onegroup_node_calc][0] = p_route_env->u_addr.src;
			onegroup_data[onegroup_node_calc][1] =  vendor_data_total_value;
			onegroup_node_calc++;
			if(onegroup_node_calc == send_nb)
			{
				for(uint8_t j = 0; j < send_nb;j++)
				{
					MESH_APP_PRINT_INFO("u_addr = 0x%x,   rx num = %d\n",onegroup_data[j][0],onegroup_data[j][1]);
 				}
				
				for(uint8_t k = 0; k < ONEGROUP_NODE_MAX; k++)  ///clear array value
				{
					onegroup_data[k][0] =  0;
					onegroup_data[k][1] =  0;
				}
				onegroup_node_calc = 0;
				MESH_APP_PRINT_DEBUG("__^^__%s onegroup get stop\n",__func__);

				app_onegroup_get_timer_set(0);
			}
	  }

	   	//if((p_env_vdr->attr[index].attr_len == 2)&&(p_env_vdr->attr[index].attr_type == 0x88))
	   	if( (p_route_env->opcode == MM_MSG_VENDOR_ATTR_ATTR_STATUS) && ((attr_type == ONOFF_ONENODE_GET_ATTRTYPE)||(attr_type ==ONOFF_ONEGROUP_GET_ATTRTYPE)))
		  {
		  	uint16_t gen_onoff_rx_calc=0;
		  	gen_onoff_rx_calc = p_data[3] + (p_data[4] << 8);
		  	MESH_APP_PRINT_INFO("NodeADDR: 0x%x,	Rx Nb.: %d\n",p_route_env->u_addr.src,gen_onoff_rx_calc);		///change 2020 0117
		  }

		  #if 0
		   if((p_env_vdr->attr[index].attr_len == 2)&&(p_env_vdr->attr[index].attr_type == 0x8a))
		  {
		  	vendor_data_total_value = p_env_vdr->attr[index].attr_param[0] + (p_env_vdr->attr[index].attr_param[1] << 8);
		  	MESH_APP_PRINT_INFO("____^^^^^^^^^____Rx Vendor data NB = %d\n",vendor_data_total_value);		///change 20200106
		  }
		  #endif

	  

        // Keep information for transmission of status if needed
        if (p_route_env->opcode == MM_MSG_VENDOR_ATTR_ATTR_SET)
        {
            p_env_vdr->status_dst_addr = p_route_env->u_addr.src;
            p_env_vdr->status_app_key_lid = p_route_env->app_key_lid;
            p_env_vdr->status_relay = GETB(p_route_env->info, MM_ROUTE_BUF_INFO_RELAY);
        }

        p_env_vdr->cur_attr_type = attr_type;

        if (GETB(p_env_vdr->env.info, MM_TB_STATE_MDL_INFO_MAIN))
        {
		// Update state
		uint8_t index = find_index_by_type(p_env_vdr);
		if (index != MM_VENDORS_ATTR_MAX_NUM)
		{
		    memcpy(p_env_vdr->attr[index].tgt_attr_param, p_data + MM_VENDORS_STATUS_ATTR_PARAM_POS,data_len);
        }
		// Inform the Binding Manager about new transition
		//mm_tb_bind_trans_new(p_env_vdr->env.grp_lid, MM_TRANS_TYPE_CLASSIC,
		//                    trans_time, delay);
		MESH_MODEL_PRINT_DEBUG("vendor model, mm_tb_bind_trans_new\r\n");
        }
        else
        {
		MESH_MODEL_PRINT_DEBUG("vendor model, mm_tb_bind_trans_array_req\r\n");
		// Inform the Binding Manager
		//mm_tb_bind_trans_array_req(p_env_vdr->env.grp_lid, p_env_vdr->env.mdl_lid,
		//MM_TRANS_TYPE_CLASSIC, p_data + MM_VENDORS_STATUS_ATTR_PARAM_POS, trans_time, delay);
        }
    }
    while (0);
}


__STATIC void mm_vendors_handler_status(mm_vendors_env_t *p_env_vdr, mesh_tb_buf_t *p_buf,
                                     mm_route_buf_env_t *p_route_env)
{
	
}


/**
 ****************************************************************************************
 * @brief Callback function called when timer monitoring publication duration for
 * vendor  Server model expires
 *
 * @param[in] p_env     Pointer to model environment for vendor Server model
 ****************************************************************************************
 */

uint16_t vendor_20bytes_tx_nb = 0;
extern m_lid_t g_vdr_lid;
extern uint16_t gdst;
extern uint16_t user_models_publish_clear(uint16_t app_keyid,m_lid_t m_lid, uint16_t addr);
extern uint16_t send_nb;
__STATIC void mm_vendor_cb_tmr_publi(void *p_env)
{
    // Get allocated environment
    mm_vendors_env_t *p_env_vdr = (mm_vendors_env_t *)p_env;
    MESH_MODEL_PRINT_DEBUG("mm_vendors_publish,publi_period_ms:%d\r\n",p_env_vdr->publi_period_ms);
    if (p_env_vdr->publi_period_ms)
    {
        // Publish a Generic OnOff Status message
        //mm_vendors_publish(p_env_vdr);
        if(othercmd == VENDOR_RSP_ENABLE_CMD)
        {
       		p_env_vdr->cur_attr_type = VENDOR_SNED_RSP_ATTRTYPE;
        }
	else
	{
		p_env_vdr->cur_attr_type = VENDOR_ONENODE_SEND_ATTRTYPE;
	}
	
	mm_vendors_publish_data(p_env_vdr);

	#if 0
        // Restart the timer
        mesh_tb_timer_set(&p_env_vdr->tmr_publi, p_env_vdr->publi_period_ms);
	#else
	if(vendor_20bytes_tx_nb <  (send_nb-1) ||(send_nb == 0xffff))
        {
        	mesh_tb_timer_set(&p_env_vdr->tmr_publi, p_env_vdr->publi_period_ms);
		vendor_20bytes_tx_nb++;
		#if(LED_SHOW_P1_ENABLE)
		gpio_set(0x10, vendor_20bytes_tx_nb % 2);
		gpio_set(0x11, 1); 
		gpio_set(0x12, 1);
		#else
		gpio_set(0x32, vendor_20bytes_tx_nb % 2); ///add 200110
		#endif
		//MESH_TB_PRINT_INFO("send NB = %d!",vendor_20bytes_tx_nb);
		MESH_APP_PRINT_INFO("T%d!",vendor_20bytes_tx_nb);
    }
	else
	{
		m_tb_store_config(3);  ///add 0303
		m_tb_mio_get_publi_param(g_vdr_lid,&gdst,     
								&p_va_lid_test,                             
								&p_app_key_lid_test,
								&p_ttl_test,
								&p_period_test,                            
								&p_retx_params_test,
								&p_friend_cred_test);
		
		MESH_APP_PRINT_DEBUG("publish_clear gdst = 0x%x\n", gdst);
		user_models_publish_clear(g_app_key_id,g_vdr_lid, gdst);  ///add 191227 //publish set ///c000 -> gdst
		vendor_20bytes_tx_nb++;
		//MESH_APP_PRINT_INFO("send END!send NB = %d\n", vendor_20bytes_tx_nb);
		MESH_APP_PRINT_INFO("ET%d\n", vendor_20bytes_tx_nb);
		vendor_20bytes_tx_nb = 0;
		othercmd = 0;
	}
	#endif
    }
}

///for rx MM_MSG_VENDOR_ATTR_CONTROL_SET done
__STATIC void mm_vendors_handler_control_set(mm_vendors_env_t *p_env_vdr, mesh_tb_buf_t *p_buf,
                                     mm_route_buf_env_t *p_route_env)
{
	uint8_t *p_data = MESH_TB_BUF_DATA(p_buf);
	
	for (int i=0; i<p_buf->data_len; i++)
	{
			MESH_APP_PRINT_INFO("ctrl param = 0x%x, =%d\n", p_data[i],p_data[i]); ///2020 0203 08:26
	}
	//MESH_APP_PRINT_INFO("p_route_env->opcode = 0x%x\r\n",p_route_env->opcode);
	
	//if data_len = 1 ,store into nvds ///0305
	if(p_buf->data_len == 1)
	{
		#if 0
		ret = nvds_put(NVDS_TAG_POWER_RESET_CNT, len, (uint8_t*)&quick_onoff_count);
		#else
		if(!nvds_put(NVDS_TAG_MESH_CTLSET_VALUE,p_buf->data_len,p_data)) ///add 0305
		{
		    MESH_APP_PRINT_INFO("nvds store CTLSET_VALUE OK!");
		}
		else
		{
			MESH_APP_PRINT_INFO("nvds store CTLSET_VALUE Error!");
		}
		#endif
		nvds_ctlset_value = *p_data;
	}
}


/**
 ****************************************************************************************
 * @brief Inform Vendor  Server model about a received message
 *
 * @param[in] p_env         Pointer to the environment allocated for the Generic OnOff
 * Server model
 * @param[in] p_buf         Pointer to the buffer containing the received message
 * @param[in] p_route_env   Pointer to structure containing reception parameters provided
 * by the Mesh Profile block
 ****************************************************************************************
 */
__STATIC void mm_vendors_cb_rx(mm_tb_state_mdl_env_t *p_env, mesh_tb_buf_t *p_buf,
                               mm_route_buf_env_t *p_route_env)
{
    // Get environment for Generic OnOff Server model
    mm_vendors_env_t *p_env_vdr = (mm_vendors_env_t *)p_env;

    // Get only second byte of the opcode
    uint32_t opcode_3b = p_route_env->opcode;//(uint8_t)(p_route_env->opcode >> 16);

    MESH_MODEL_PRINT_DEBUG("mm_vendors_cb_rx, opcode = 0x%x\r\n", opcode_3b);
    switch (opcode_3b)
    {
        case MM_MSG_VENDOR_ATTR_GET:
        {
            uint8_t *p_data = MESH_TB_BUF_DATA(p_buf);
            uint16_t attr_type =  co_read16(p_data + MM_VENDORS_STATUS_ATTR_TYPE_POS);
            p_env_vdr->cur_attr_type = attr_type;
            MESH_MODEL_PRINT_DEBUG("________vendor model,attr_type: 0x%x\r\n", attr_type);
            mm_vendors_send_status(p_env_vdr, p_route_env, false,attr_type);
        } break;

	    case MM_MSG_VENDOR_ATTR_ATTR_STATUS:
        case MM_MSG_VENDOR_ATTR_ATTR_SET:
        case MM_MSG_VENDOR_ATTR_ATTR_SET_UNACK:
        {
            mm_vendors_handler_set(p_env_vdr,p_buf,p_route_env);
        } break;

        case MM_MSG_VENDOR_ATTR_CONFIRMATION:
        {
            MESH_MODEL_PRINT_DEBUG("RECV vendors cfm msg\r\n");
            uint8_t *p_data = MESH_TB_BUF_DATA(p_buf);
            uint16_t attr_type =  co_read16(p_data + MM_VENDORS_STATUS_ATTR_TYPE_POS);
            p_env_vdr->cur_attr_type = attr_type;
            is_cfm_msg_recved = true;
            MESH_MODEL_PRINT_DEBUG("vendor model,attr_type: 0x%x\r\n", attr_type);
        } break;

        case MM_MSG_VENDOR_ATTR_TRANSPARENT:
        {

        } break;
		
	case MM_MSG_VENDOR_ATTR_CONTROL_SET:
	{
		    MESH_APP_PRINT_INFO("MM_MSG_VENDOR_ATTR_CONTROL_SET\r\n");
		    mm_vendors_handler_control_set(p_env_vdr,p_buf,p_route_env);
	}break;

        default:break;
    }

}


void mm_vendors_cfm_clear(void)
{
    is_cfm_msg_recved = false;
}

bool mm_vendors_cfm_recved(void)
{
    return is_cfm_msg_recved;
}

/**
 ****************************************************************************************
 * @brief Check if a given opcode is handled by the Vendor Server model
 *
 * @param[in] p_env         Pointer to the environment allocated for the Vendor
 * Server model
 * @param[in] opcode        Opcode to check
 *
 * @return An error status (@see enum mesh_err)
 ****************************************************************************************
 */
__STATIC uint16_t mm_vendors_cb_opcode_check(mm_tb_state_mdl_env_t *p_env, uint32_t opcode)
{
    uint16_t status = MESH_ERR_MDL_INVALID_OPCODE;

    MESH_MODEL_PRINT_DEBUG("mm_vendors_cb_opcode_check, opcode = 0x%x, p_env->mdl_id = 0x%x\r\n", opcode, p_env->mdl_id);
    if (p_env->mdl_id == MM_ID_VENDORS)
    {
        if ((MM_IS_VENDOR_OPCODE(opcode)) || (opcode == MM_MSG_VENDOR_ATTR_CONTROL_SET))
        {
            // Get only second byte of the opcode
            uint32_t opcode_3b = opcode; //(uint8_t)(opcode >> 16);
            if ((opcode_3b == MM_MSG_VENDOR_ATTR_GET)
                    || (opcode_3b == MM_MSG_VENDOR_ATTR_ATTR_SET)
                    || (opcode_3b == MM_MSG_VENDOR_ATTR_ATTR_SET_UNACK)
                    || (opcode_3b == MM_MSG_VENDOR_ATTR_CONFIRMATION)
                    || (opcode_3b == MM_MSG_VENDOR_ATTR_ATTR_STATUS)
                    || (opcode_3b == MM_MSG_VENDOR_ATTR_TRANSPARENT)//)
                    || (opcode_3b == MM_MSG_VENDOR_ATTR_CONTROL_SET))
            {
                status = MESH_ERR_NO_ERROR;
            }
        }

    }

    if (status != MESH_ERR_NO_ERROR)
    {
        MESH_MODEL_PRINT_INFO("%s, Invalid opcode (0x%x).\n", __func__, opcode);
    }

    return (status);
}

/*
 * CALLBACK FUNCTIONS FOR BINDING MANAGER
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Callback function called by the Binding Manager when group event occurs
 *
 * @param[in] mdl_lid       Model local index
 * @param[in] event         Event
 * @param[in] info          Information
 ****************************************************************************************
 */
void mm_vendors_cb_grp_event(m_lid_t mdl_lid, uint8_t event, uint8_t info)
{
    uint16_t gen_onoff_rx_calc;
    uint16_t vendor_data_total_value = 0;
    // Get environment for Vendor Server model
    mm_vendors_env_t *p_env_vdr = (mm_vendors_env_t *)mm_tb_state_get_env(mdl_lid);

    MESH_MODEL_PRINT_DEBUG("mm_vendors_cb_grp_event, event: %d \r\n", event);
    switch (event)
    {
        case (MESH_MDL_GRP_EVENT_TRANS_REJECTED):
        {
            // Send a response to the node that has required the transition
            MESH_MODEL_PRINT_DEBUG("__^^__MESH_MDL_GRP_EVENT_TRANS_REJECTED \r\n");
            mm_vendors_check_status_rsp(p_env_vdr);
        } break;

        case (MESH_MDL_GRP_EVENT_TRANS_DELAY_EXPIRED):
        {
            // Start the transition
            mm_tb_bind_trans_start(p_env_vdr->env.grp_lid);
        } break;

        case (MESH_MDL_GRP_EVENT_TRANS_IMMEDIATE):
        {
            uint8_t index = 0;
            index = find_index_by_type(p_env_vdr);
            if (MM_VENDORS_ATTR_MAX_NUM  != index)
            {
                memcpy(p_env_vdr->attr[index].attr_param,p_env_vdr->attr[index].tgt_attr_param,p_env_vdr->attr[index].attr_len);

		 MESH_MODEL_PRINT_DEBUG("attr[index].attr_type = 0x%x\n",p_env_vdr->attr[index].attr_type);

		 #if 0
                MESH_MODEL_PRINT_DEBUG("mm_vendors_cb_grp_event, pDATA:");
				
                for (int i=0; i<p_env_vdr->attr[index].attr_len; i++)
                {
                    MESH_MODEL_PRINT_DEBUG("0x%x ", p_env_vdr->attr[index].attr_param[i]);
                }
				
		 MESH_MODEL_PRINT_DEBUG("\r\n");
		 #endif

		 if((p_env_vdr->attr[index].attr_len == 14) && (p_env_vdr->attr[index].attr_type == 0x89))
		 {
		 	MESH_MODEL_PRINT_DEBUG("rx vendor 20btyes++ %d\n",vendor_20bytes_tx_calc);
		 	vendor_20bytes_tx_calc++;

		 }

		  if((p_env_vdr->attr[index].attr_len == 2)&&(p_env_vdr->attr[index].attr_type == 0x88))
		  {
		  	gen_onoff_rx_calc = p_env_vdr->attr[index].attr_param[0] + (p_env_vdr->attr[index].attr_param[1] << 8);
		  	MESH_APP_PRINT_INFO("NodeADDR: 0x%x,	Rx Nb.: %d\n",nodedst,gen_onoff_rx_calc);		///change 2020 0117
		  }

		   if((p_env_vdr->attr[index].attr_len == 2)&&(p_env_vdr->attr[index].attr_type == 0x8a))
		  {
		  	vendor_data_total_value = p_env_vdr->attr[index].attr_param[0] + (p_env_vdr->attr[index].attr_param[1] << 8);
		  	MESH_APP_PRINT_INFO("____^^^^^^^^^____Rx Vendor data NB = %d\n",vendor_data_total_value);		///change 20200106
		  }
		  
            }
        } // no break;

        case (MESH_MDL_GRP_EVENT_TRANS_STARTED):
        {
            // Inform application about the update
            if (GETB(p_env_vdr->env.info, MM_TB_STATE_MDL_INFO_MAIN))
            {
                uint8_t trans_time = info;

                uint8_t index = 0;
                index = find_index_by_type(p_env_vdr);

                if (p_env_vdr->cur_attr_type == LIGHT_COLOR)
                {
                    // Inform application about state update
                    mm_api_send_srv_array_state_upd_ind(MM_STATE_VENDORS_HSL, p_env_vdr->env.elmt_idx,
                                                        p_env_vdr->attr[index].attr_len,p_env_vdr->attr[index].tgt_attr_param,
                                                        (trans_time) ? mm_tb_get_trans_time_ms(trans_time) : 0);
                }
            }

            // Send a response to the node that has required the transition
            mm_vendors_check_status_rsp(p_env_vdr);

            // Send a publication
            mm_vendors_publish(p_env_vdr);

        } break;

        case (MESH_MDL_GRP_EVENT_TRANS_END):
        {
            // New state is the target state
            uint8_t index = 0;
            index = find_index_by_type(p_env_vdr);
            if (MM_VENDORS_ATTR_MAX_NUM  != index)
            {
                memcpy(p_env_vdr->attr[index].attr_param,p_env_vdr->attr[index].tgt_attr_param,p_env_vdr->attr[index].attr_len);
            }

            // Inform application about the update
            if (GETB(p_env_vdr->env.info, MM_TB_STATE_MDL_INFO_MAIN))
            {
                uint8_t index = 0;
                index = find_index_by_type(p_env_vdr);

                if (p_env_vdr->cur_attr_type == LIGHT_COLOR)
                {
                    // Inform application about state update
                    mm_api_send_srv_array_state_upd_ind(MM_STATE_VENDORS_HSL, p_env_vdr->env.elmt_idx,
                                                        p_env_vdr->attr[index].attr_len,p_env_vdr->attr[index].tgt_attr_param, 0);
                }
            }
            // Send a publication
            mm_vendors_publish(p_env_vdr);
        } break;

        // case (MESH_MDL_GRP_EVENT_TRANS_ABORTED):
        // case (MESH_MDL_GRP_EVENT_GROUP_FULL):
        default:
        {
        } break;
    }
}



/**
 ****************************************************************************************
 * @brief Set vendors state value
 *
 * @param[in] p_env         Pointer the the environment allocated for the Generic OnOff
 * Server model
 * @param[in] onoff         Generic OnOff state value
 *
 * @return An error status (@see enum mesh_err)
 ****************************************************************************************
 */
__STATIC uint16_t mm_vendors_cb_set(mm_tb_state_mdl_env_t *p_env, uint16_t state_id, uint32_t state)
{
    // Returned status
    uint16_t status = MESH_ERR_NO_ERROR;

    MESH_APP_PRINT_INFO("mm_vendors_cb_set\r\n");

    return (status);
}


uint16_t mm_vendors_register(uint8_t elmt_idx, m_lid_t *p_mdl_lid)
{
    // Register the model
    uint16_t status = m_api_register_model(MM_ID_VENDORS, elmt_idx, M_MDL_CONFIG_PUBLI_AUTH_BIT,
                                           &mm_route_cb, p_mdl_lid);

    do
    {
        // Check if model has been properly registered
        if (status != MESH_ERR_NO_ERROR)
        {
            break;
        }

        // Inform the Model State Manager about registered model
        status = mm_tb_state_register(elmt_idx, MM_ID_VENDORS, *p_mdl_lid,
                                      MM_TB_STATE_ROLE_SRV | MM_TB_STATE_CFG_CB_BIT,
                                      sizeof(mm_vendors_env_t));

        if (status == MESH_ERR_NO_ERROR)
        {
  	        MESH_APP_PRINT_INFO("vendors register test1%x\n",status);

            // Get allocated environment
            mm_vendors_env_t *p_env_vdr = (mm_vendors_env_t *)mm_tb_state_get_env(*p_mdl_lid);
            // Get server-specific callback functions
            mm_srv_cb_t *p_cb_srv = p_env_vdr->env.cb.u.p_cb_srv;

            // Prepare timer for publication
            p_env_vdr->tmr_publi.cb = mm_vendor_cb_tmr_publi;
            p_env_vdr->tmr_publi.p_env = (void *)p_env_vdr;

            vdr_lid = *p_mdl_lid;
            p_env_vdr->tid = 0;

            // Prepare environment for Replay Manager
            p_env_vdr->replay_env.delay_ms = MM_VENDORS_REPLAY_MS;

            // Set internal callback functions
            p_env_vdr->env.cb.cb_rx = mm_vendors_cb_rx;
            p_env_vdr->env.cb.cb_opcode_check = mm_vendors_cb_opcode_check;
            p_env_vdr->env.cb.cb_publish_param = mm_vendors_cb_publish_param;
            p_cb_srv->cb_set = mm_vendors_cb_set;


            //light switch status type
            p_env_vdr->attr[0].attr_type = 0x0100;
            p_env_vdr->attr[0].attr_len = 0x01;
			
            //light color status type
            p_env_vdr->attr[1].attr_type = 0x0123;
            p_env_vdr->attr[1].attr_len = 0x06;

	        p_env_vdr->attr[2].attr_type = 0x88;  	///for onoff get
            p_env_vdr->attr[2].attr_len = 0x02;   	///add 191230

	        p_env_vdr->attr[3].attr_type = VENDOR_ONENODE_SEND_ATTRTYPE; 	///for vendor 20bytes send
	        //p_env_vdr->attr[3].attr_len = 12;   		///14->2 add 2020 0108
		    p_env_vdr->attr[3].attr_len = 12;   

	        p_env_vdr->attr[4].attr_type = VENDOR_ONENODE_GET_ATTRTYPE;	///for vendor 20byte get
            p_env_vdr->attr[4].attr_len = 0x02;   	///add 2020 0109

	        p_env_vdr->attr[5].attr_type = VENDOR_MOREGROUP_SEND_ATTRTYPE;		//0x8b///for vendor moregroup send
	        p_env_vdr->attr[5].attr_len = 0x02;	     ///add 2020 0110

	        p_env_vdr->attr[6].attr_type = VENDOR_ONEGROUP_GET_ATTRTYPE;	///for vendor onegroup get
	        p_env_vdr->attr[6].attr_len = 0x02;		///add 2020 0113

	        p_env_vdr->attr[7].attr_type = ONOFF_MOREGROUP_SEND_ATTRTYPE;		//0x8b///for vendor moregroup send
	        p_env_vdr->attr[7].attr_len = 0x02;	     ///add 2020 0110

	        p_env_vdr->attr[8].attr_type = ONOFF_ONEGROUP_GET_ATTRTYPE;	///for vendor onegroup get
	        p_env_vdr->attr[8].attr_len = 0x02;		///add 2020 0113

	        p_env_vdr->attr[9].attr_type = VENDOR_SNED_RSP_ATTRTYPE;	
	        //p_env_vdr->attr[9].attr_len = 12;
	        p_env_vdr->attr[9].attr_len = 12;

	        p_env_vdr->attr[10].attr_type = VENDOR_SNED_RSP_ACK_ATTRTYPE;	
	        //p_env_vdr->attr[10].attr_len = 12;
		    p_env_vdr->attr[10].attr_len = 12;	

            p_env_vdr->cur_attr_type = VENDOR_ONENODE_SEND_ATTRTYPE; ///??? 2020 0122
            //p_env_vdr->cur_attr_type = 0X89;
            

            // Group local index
            m_lid_t grp_lid;

            // Create group
            #if 0  ///200121
            mm_tb_bind_add_group(0, elmt_idx, &grp_lid, *p_mdl_lid,
                                 mm_vendors_cb_grp_event, NULL);
	     #endif

            // Inform application about registered model
            mm_api_send_register_ind(MM_ID_VENDORS, elmt_idx, *p_mdl_lid);
        }
    }
    while (0);

    return (status);
}



void mm_vendors_cb_set_array_state(m_lid_t mdl_lid, uint8_t type, uint8_t* buf)
{
    // Get environment for Vendor Server model
    mm_vendors_env_t *p_env_vdr = (mm_vendors_env_t *)mm_tb_state_get_env(mdl_lid);

    // Extract TID value

    uint8_t data_len;
    uint8_t tid = *(buf + MM_VENDORS_STATUS_TID_POS);

    uint16_t attr_type =  co_read16(buf + MM_VENDORS_STATUS_ATTR_TYPE_POS);
    uint8_t * attr_param = get_attr_info(p_env_vdr,attr_type,&data_len,0);
    uint8_t index = find_index_by_type(p_env_vdr);

    if (attr_param)
    {
        if (type == MM_STATE_TYPE_CURRENT)
        {
            memcpy(attr_param,(buf + MM_VENDORS_STATUS_ATTR_PARAM_POS),data_len);
        }
        else
        {
            if (index != MM_VENDORS_ATTR_MAX_NUM)
            {
                memcpy(p_env_vdr->attr[index].tgt_attr_param, (buf + MM_VENDORS_STATUS_ATTR_PARAM_POS),data_len);
            }
        }
    }

}

void mm_vendor_attr_indication(uint16_t attr_type,uint8_t len,uint8_t *value,uint16_t dst)
{

    // Pointer to the buffer that will contain the message
    mesh_tb_buf_t *p_buf_status;
    if (mm_route_buf_alloc(&p_buf_status, len + 3) == MESH_ERR_NO_ERROR)
    {

        // Get allocated environment
        mm_vendors_env_t *p_env_vdr = (mm_vendors_env_t *)mm_tb_state_get_env(vdr_lid);

        MESH_APP_PRINT_DEBUG("indication mdl_id = 0x%x\r\n", p_env_vdr->env.mdl_id);
        // Get pointer to data
        uint8_t *p_data = MESH_TB_BUF_DATA(p_buf_status);
        // Get pointer to environment
        mm_route_buf_env_t *p_buf_env = (mm_route_buf_env_t *)&p_buf_status->env;

        // Prepare environment
        m_tb_key_app_find(0, &(p_buf_env->app_key_lid));
        SETB(p_buf_env->info, MM_ROUTE_BUF_INFO_RELAY, p_env_vdr->status_relay);


        SETB(p_buf_env->info, MM_ROUTE_BUF_INFO_RX, 0);
       // SETB(p_buf_env->info, MM_ROUTE_BUF_INFO_PUBLISH, 1);
       SETB(p_buf_env->info, MM_ROUTE_BUF_INFO_PUBLISH, 0);
	   
        p_buf_env->mdl_lid = p_env_vdr->env.mdl_lid;
        //p_buf_env->opcode = MM_VENDOR_3B_OPCODE(MM_MSG_VENDOR_ATTR_INDICATION);
        p_buf_env->opcode = MM_VENDOR_3B_OPCODE(MM_MSG_VENDOR_ATTR_ATTR_SET);  ///191230

	 #if 1
	 p_buf_env->u_addr.dst = dst;
	 #endif
	 
        p_env_vdr->cur_attr_type = attr_type;

        MESH_APP_PRINT_DEBUG("indication mdl_lid = %x,p_buf_env->opcode = 0x%x\r\n",p_env_vdr->env.mdl_lid,p_buf_env->opcode);
        // Fill the message
        *p_data = get_vendor_tid(p_env_vdr);
        co_write16p(p_data + MM_VENDORS_STATUS_ATTR_TYPE_POS, attr_type);
        memcpy(p_data + MM_VENDORS_STATUS_ATTR_PARAM_POS, value, len);

        mm_vendors_cb_set_array_state(p_env_vdr->env.mdl_lid, MM_STATE_TYPE_CURRENT, p_data);
        // Send the message
        mm_route_send(p_buf_status);

    }

}

void mm_vendor_attr_indication_publish(uint16_t attr_type,uint8_t len,uint8_t *value,uint16_t dst)
{

    // Pointer to the buffer that will contain the message
    mesh_tb_buf_t *p_buf_status;
    if (mm_route_buf_alloc(&p_buf_status, len + 3) == MESH_ERR_NO_ERROR)
    {

        // Get allocated environment
        mm_vendors_env_t *p_env_vdr = (mm_vendors_env_t *)mm_tb_state_get_env(vdr_lid);

        MESH_APP_PRINT_DEBUG("indication mdl_id = 0x%x\r\n", p_env_vdr->env.mdl_id);
        // Get pointer to data
        uint8_t *p_data = MESH_TB_BUF_DATA(p_buf_status);
        // Get pointer to environment
        mm_route_buf_env_t *p_buf_env = (mm_route_buf_env_t *)&p_buf_status->env;

        // Prepare environment
        m_tb_key_app_find(0, &(p_buf_env->app_key_lid));
        SETB(p_buf_env->info, MM_ROUTE_BUF_INFO_RELAY, p_env_vdr->status_relay);


        SETB(p_buf_env->info, MM_ROUTE_BUF_INFO_RX, 0);
        SETB(p_buf_env->info, MM_ROUTE_BUF_INFO_PUBLISH, 1);
       //SETB(p_buf_env->info, MM_ROUTE_BUF_INFO_PUBLISH, 0);
	   
        p_buf_env->mdl_lid = p_env_vdr->env.mdl_lid;
        //p_buf_env->opcode = MM_VENDOR_3B_OPCODE(MM_MSG_VENDOR_ATTR_INDICATION);
        p_buf_env->opcode = MM_VENDOR_3B_OPCODE(MM_MSG_VENDOR_ATTR_ATTR_SET);  ///191230

	 #if 1
	 p_buf_env->u_addr.dst = dst;
	 #endif
	 
        p_env_vdr->cur_attr_type = attr_type;

        MESH_APP_PRINT_DEBUG("indication mdl_lid = %x,p_buf_env->opcode = 0x%x\r\n",p_env_vdr->env.mdl_lid,p_buf_env->opcode);
        // Fill the message
        *p_data = get_vendor_tid(p_env_vdr);
        co_write16p(p_data + MM_VENDORS_STATUS_ATTR_TYPE_POS, attr_type);
        memcpy(p_data + MM_VENDORS_STATUS_ATTR_PARAM_POS, value, len);

        mm_vendors_cb_set_array_state(p_env_vdr->env.mdl_lid, MM_STATE_TYPE_CURRENT, p_data);
        // Send the message
        mm_route_send(p_buf_status);

    }

}

uint16_t mm_vendorc_transition(m_lid_t mdl_lid, m_lid_t app_key_lid, 
                                           uint16_t dst, 
                                           uint32_t opcode, uint8_t *data, uint16_t length)
{
    if (!data || !length)
    {
        MESH_MODEL_PRINT_WARN("%s, Invalid parameters, data is NULL.\n", __func__);
        return MESH_ERR_INVALID_PARAM;
    }
    
    // Pointer to the buffer that will contain the message
    mesh_tb_buf_t *p_buf_set;
    // Allocate a new buffer
    uint8_t status = mm_route_buf_alloc(&p_buf_set, length);

    if (status == MESH_ERR_NO_ERROR)
    {
        // Get pointer to data
        uint8_t *p_data = MESH_TB_BUF_DATA(p_buf_set);
        // Get pointer to environment
        mm_route_buf_env_t *p_buf_env = (mm_route_buf_env_t *)&p_buf_set->env;

        // Prepare environment
        p_buf_env->app_key_lid = app_key_lid; // TODO [LT]
        p_buf_env->u_addr.dst = dst;
        p_buf_env->info = 0;
        p_buf_env->mdl_lid = mdl_lid;
        p_buf_env->opcode = opcode;

        memcpy(p_data, data, length);

        // Send the message
        mm_route_send(p_buf_set);
    }
    else
    {
        MESH_MODEL_PRINT_WARN("%s, Alloc buffer fail.\n", __func__);
    }

    return (status);
}

