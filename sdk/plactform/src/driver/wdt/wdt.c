#include <stdint.h>        // standard integer definition
#include <string.h>        // string manipulation
#include <stddef.h>        // standard definition
#include "BK3435_reg.h"
#include "wdt.h"

uint8_t wdt_disable_flag = 0;
static uint8_t wdt_enable_status = 0;

void wdt_disable(void)
{
    REG_AHB0_ICU_WDTCLKCON = 0x1 ; // Step1. WDT clock disable
    REG_APB0_WDT_CFG = 0x005A0000;// Step3. Feed dog. Write WDT key: 0x5A firstly and 0xA5 secondly.
    REG_APB0_WDT_CFG = 0x00A50000;
    wdt_enable_status = 0;
}

//每个单位250us，最大0xffff，约16s
void wdt_enable(uint16_t wdt_cnt)
{
    REG_AHB0_ICU_WDTCLKCON = 0x0 ; // Step1. WDT clock enable,
    REG_APB0_WDT_CFG  = wdt_cnt;   // Step2. Set WDT period=0xFF
    // Do two things together: 1. Set WDT period. 2. Write WDT key to feed dog.
    // Write WDT key: 0x5A firstly and 0xA5 secondly.
    REG_APB0_WDT_CFG = (0x5A << 16) + wdt_cnt;
    REG_APB0_WDT_CFG = (0xA5 << 16) + wdt_cnt;
    wdt_disable_flag = 0;
    wdt_enable_status = 1;
}



//每个单位250us，最大0xffff，约16s
void wdt_reset(uint16_t wdt_cnt)
{
    REG_AHB0_ICU_WDTCLKCON = 0x0 ; // Step1. WDT clock enable,
    REG_APB0_WDT_CFG  = wdt_cnt;   // Step2. Set WDT period=0xFF
    // Do two things together: 1. Set WDT period. 2. Write WDT key to feed dog.
    // Write WDT key: 0x5A firstly and 0xA5 secondly.
    REG_APB0_WDT_CFG = (0x5A << 16) + wdt_cnt;
    REG_APB0_WDT_CFG = (0xA5 << 16) + wdt_cnt;
    wdt_disable_flag = 0;
    wdt_enable_status = 0;
}


void wdt_feed(uint16_t wdt_cnt)
{
    if (wdt_enable_status == 1)
    {
        //  REG_AHB0_ICU_WDTCLKCON = 0x0 ; // Step1. WDT clock enable,
        // REG_APB0_WDT_CFG  = wdt_cnt;   // Step2. Set WDT period=0xFF
        // Write WDT key: 0x5A firstly and 0xA5 secondly.
        REG_APB0_WDT_CFG = ((WDKEY_ENABLE1 << WDT_CONFIG_WDKEY_POSI)
                            | (wdt_cnt     << WDT_CONFIG_PERIOD_POSI));
        REG_APB0_WDT_CFG = ((WDKEY_ENABLE2 << WDT_CONFIG_WDKEY_POSI)
                            | (wdt_cnt     << WDT_CONFIG_PERIOD_POSI));
    }
}


