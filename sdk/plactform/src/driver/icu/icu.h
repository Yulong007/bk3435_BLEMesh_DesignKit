#ifndef _ICU_H_
#define _ICU_H_

#include "stdint.h"


#define MCU_CLK_16M   1
#define MCU_CLK_64M   2
#define MCU_CLK_32M   3
#define MCU_CLK_48M   4

#define REDUCE_VOL_SLEEP        1

#define MCU_DEFAULT_CLK     MCU_CLK_48M //MCU_CLK_64M

void system_sleep_init(void);



void icu_init(void);

uint8_t get_sleep_mode(void);

void cpu_reduce_voltage_sleep(void);

void cpu_idle_sleep(void);

uint8_t icu_get_sleep_mode(void);

void icu_set_sleep_mode(uint8_t v);

void cpu_wakeup(void);

void mcu_clk_config(void);

void udi_wdt_enable(uint16_t wdt_cnt);

void switch_clk(uint8_t clk);

void bk3435_singleWaveCfg(uint8_t freq, uint8_t power_level);

void bk3435_set_power(uint8_t power_level);

#endif

