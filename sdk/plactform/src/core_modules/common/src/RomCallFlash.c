#include "RomCallFlash.h"
#include "co_bt.h"
#include "co_utils.h"
#include "rwip.h"


struct rwip_rf_api rwip_rf;
struct rom_env_tag rom_env;

/// Default BD address
struct bd_addr co_default_bdaddr = {0x88, 0x77, 0x22, 0x24, 0x34, 0xbb};




