
#ifndef __ATTM_UTIL128_H__
#define __ATTM_UTIL128_H__

#include <stdbool.h>          // standard boolean definitions
#include <stdint.h>           // standard integer functions

#include "rwip_config.h"
#include "rwprf_config.h"
#include "gattm.h"
#include "attm_db.h"




void attm_serv_convert_to128(uint8_t *uuid128, const uint8_t *uuid, uint8_t uuid_len);


void attm_char_convert_to128(uint8_t *uuid128, const uint8_t *uuid, uint8_t uuid_len);


uint8_t attm_util_svc_create_db128(uint16_t *shdl, uint16_t uuid, uint8_t *cfg_flag, uint8_t max_nb_att, uint8_t *att_tbl, ke_task_id_t const dest_id, const struct attm_desc *att_db, uint8_t svc_perm);



#endif
