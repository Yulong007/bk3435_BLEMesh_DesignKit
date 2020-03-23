/**
****************************************************************************************
*
* @file gpio.c
*
* @brief icu initialization and specific functions
*
* Copyright (C) Beken 2009-2016
*
* $Rev: $
*
****************************************************************************************
*/

#include <stddef.h>     // standard definition
#include "BK3435_reg.h"
#include "gpio.h"
#include "timer.h"      // timer definition

#include "rwip.h"       // SW interface
#include "h4tl.h"
#include "nvds.h"       // NVDS

#include "dbg.h"
#include "icu.h"
#include "rf.h"
#include "app.h"
#include "app_task.h"
#include "uart.h"
#include "user_config.h"


volatile unsigned long *GPIO_CFG[] =
{
    &REG_APB5_GPIOA_CFG,
    &REG_APB5_GPIOB_CFG,
    &REG_APB5_GPIOC_CFG,
    &REG_APB5_GPIOD_CFG,
    &REG_APB5_GPIOE_CFG
};


volatile unsigned long *GPIO_DATA[] =
{
    &REG_APB5_GPIOA_DATA,
    &REG_APB5_GPIOB_DATA,
    &REG_APB5_GPIOC_DATA,
    &REG_APB5_GPIOD_DATA,
    &REG_APB5_GPIOE_DATA
};

unsigned int GPIO_CFG_BACKUP[5];
unsigned int GPIO_DATA_BACKUP[5];


static GPIO_INT_CALLBACK_T gpio_int_cb = NULL;

void gpio_config(uint8_t gpio, Dir_Type dir, Pull_Type pull)
{
    uint32_t  temp_cfg, temp_data;
    uint8_t port = (gpio & 0xf0) >> 4;
    uint8_t  pin = gpio & 0xf;
    temp_cfg = *(GPIO_CFG[port]);
    temp_data = *(GPIO_DATA[port]);

    if (dir == OUTPUT)
    {
        temp_cfg |= (1 << pin);
        temp_cfg &= ~(1 << (pin + 8));

        temp_data &= ~(1 << pin << 16);
    }
    else if (dir == INPUT)
    {
        temp_cfg |= 1 << pin;
        temp_cfg |= (1 << (pin + 8));
        temp_data |= (1 << pin) << 16;
    }
    else if (dir == HIRESI)
    {
        temp_cfg |= 1 << pin;
        temp_cfg |= (1 << (pin + 8));
        temp_data  &= ~(1 << pin << 16);
    }
    else
    {
        temp_cfg &= ~(1 << pin);
        temp_cfg |= (1 << (pin + 8));
        temp_data  |= (1 << pin << 16);
    }

    switch (pull)
    {
        case PULL_HIGH:
            temp_cfg |= (1 << pin << 16);
            temp_cfg &= ~(1 << pin << 24);
            break;
        case PULL_LOW:
            temp_cfg &= ~(1 << pin << 16);
            temp_cfg |= (1 << pin << 24);
            break;
        case PULL_NONE:
            temp_cfg &= ~(1 << pin << 16);
            temp_cfg &= ~(1 << pin << 24);
            break;
    }
    *(GPIO_CFG[port]) = temp_cfg;
    *(GPIO_DATA[port]) = temp_data;
}

uint8_t gpio_get_input(uint8_t gpio)
{
    uint32_t temp;
    uint8_t port = (gpio & 0xf0) >> 4;
    uint8_t  pin = gpio & 0xf;

    temp = *(GPIO_DATA[port]);
    temp = ((temp >> 0x08) & 0xff) >> pin;

    return temp & 0x01;
}

void gpio_set(uint8_t gpio, uint8_t val)
{
    uint32_t temp;
    uint8_t port = (gpio & 0xf0) >> 4;
    uint8_t  pin = gpio & 0xf;

    temp = *(GPIO_DATA[port]);
    if (val)
    {
        temp |= 1 << pin;
    }
    else
    {
        temp &= ~(1 << pin);
    }

    *(GPIO_DATA[port]) = temp;
}

#if GPIO_DBG_MSG
void gpio_debug_msg_init()
{

    *(GPIO_CFG[0]) = 0X0000ff;
    *(GPIO_DATA[0]) = 0;
    REG_AHB0_ICU_DIGITAL_PWD |= (0X01 << 4);
}
#endif

void gpio_init(void)
{
#if DEBUG_HW
    *(GPIO_CFG[2]) = 0XFF0000;
    *(GPIO_DATA[2]) = 0;
    REG_AHB0_ICU_DIGITAL_PWD |= (0X01 << 4);
#endif

#if 0// GPIO_DBG_MSG
    gpio_debug_msg_init();
    gpio_config(0x34, OUTPUT, PULL_NONE);
    gpio_set(0x34, 0);
#endif

    //gpio_config(0x10, OUTPUT, PULL_LOW);
    //gpio_set(0x10, 0);
    gpio_config(0x16, OUTPUT, PULL_NONE);
    gpio_config(0x17, OUTPUT, PULL_NONE);
    gpio_config(0x30, OUTPUT, PULL_NONE);
    gpio_config(0x31, OUTPUT, PULL_NONE);
    gpio_config(0x32, OUTPUT, PULL_NONE);
    gpio_config(0x33, OUTPUT, PULL_NONE);
    gpio_config(0x34, OUTPUT, PULL_NONE);
    gpio_config(0x04, OUTPUT, PULL_NONE);
    gpio_config(0x15, OUTPUT, PULL_NONE);
    gpio_config(0x16, OUTPUT, PULL_NONE);
    gpio_config(0x03, OUTPUT, PULL_NONE);
    gpio_config(0x07, OUTPUT, PULL_NONE);
    gpio_config(0x11, OUTPUT, PULL_NONE);
    gpio_config(0x12, OUTPUT, PULL_NONE);
    gpio_config(0x06, OUTPUT, PULL_NONE);
    gpio_config(0x02, OUTPUT, PULL_NONE);
#if (POWER_ONOFF_ONLY)
    gpio_config(0x10, OUTPUT, PULL_NONE);
#endif
    gpio_set(0x30, 0);
    gpio_set(0x31, 0);
    gpio_set(0x32, 0);
    gpio_set(0x34, 0);
    gpio_set(0x04, 0);
    gpio_set(0x05, 0);
    gpio_set(0x15, 0);
    gpio_set(0x16, 0);
    gpio_set(0x03, 0);
    gpio_set(0x07, 0);
    gpio_set(0x11, 0);
	gpio_set(0x12, 0);
    gpio_set(0x06, 0);
    gpio_set(0x02, 0);
#if (POWER_ONOFF_ONLY)
    gpio_set(0x10, 1);  ///1-> 0 ???
#endif
}


void gpio_triger(uint8_t gpio)
{
    gpio_set(gpio, 1);
    gpio_set(gpio, 0);
}


void gpio_cb_register(GPIO_INT_CALLBACK_T cb)
{
    if (cb)
    {
        gpio_int_cb = cb;
    }
}

void gpio_isr(void)
{
    REG_APB5_GPIO_WUATOD_ENABLE = 0x00000000;
    REG_APB5_GPIO_WUATOD_STAT = 0xffffffff;
    REG_AHB0_ICU_INT_ENABLE &= (~(0x01 << 9));
    //gpio_triger(0x14);

    //triger int callback
    if (gpio_int_cb)
    {
        (*gpio_int_cb)();
    }
}

void gpio_test_init(void)
{
    gpio_config(GPIOC_0, FLOAT, PULL_HIGH);
    gpio_config(GPIOC_0, FLOAT, PULL_HIGH);
    gpio_config(GPIOC_0, FLOAT, PULL_HIGH);
    gpio_config(GPIOC_0, FLOAT, PULL_HIGH);
    gpio_config(GPIOC_0, FLOAT, PULL_HIGH);
}

void gpio_sleep(void)
{
    GPIO_CFG_BACKUP[0] = REG_APB5_GPIOA_CFG;
    GPIO_CFG_BACKUP[1] = REG_APB5_GPIOB_CFG;
    GPIO_CFG_BACKUP[2] = REG_APB5_GPIOC_CFG;
    GPIO_CFG_BACKUP[3] = REG_APB5_GPIOD_CFG;
    GPIO_CFG_BACKUP[4] = REG_APB5_GPIOE_CFG;

    GPIO_DATA_BACKUP[0] = REG_APB5_GPIOA_DATA;
    GPIO_DATA_BACKUP[1] = REG_APB5_GPIOB_DATA;
    GPIO_DATA_BACKUP[2] = REG_APB5_GPIOC_DATA;
    GPIO_DATA_BACKUP[3] = REG_APB5_GPIOD_DATA;
    GPIO_DATA_BACKUP[4] = REG_APB5_GPIOE_DATA;


    REG_APB5_GPIOA_CFG = 0x0000ffff;
    REG_APB5_GPIOA_DATA = 0x0000ffff;

    REG_APB5_GPIOB_CFG = 0x0000ffff;
    REG_APB5_GPIOB_DATA = 0x0000ffff;

    REG_APB5_GPIOC_CFG = 0x0000ffff;
    REG_APB5_GPIOC_DATA = 0x0000ffff;

    REG_APB5_GPIOD_CFG = 0x0000ffff;
    REG_APB5_GPIOD_DATA = 0x0000ffff;

    REG_APB5_GPIOE_CFG = 0x0000ffff;
    REG_APB5_GPIOE_DATA = 0x0000ffff;
}

void gpio_wakeup(void)
{
    REG_APB5_GPIOA_CFG = GPIO_CFG_BACKUP[0];
    REG_APB5_GPIOB_CFG = GPIO_CFG_BACKUP[1];
    REG_APB5_GPIOC_CFG = GPIO_CFG_BACKUP[2];
    REG_APB5_GPIOD_CFG = GPIO_CFG_BACKUP[3];
    REG_APB5_GPIOE_CFG = GPIO_CFG_BACKUP[4];

    REG_APB5_GPIOA_DATA = GPIO_DATA_BACKUP[0];
    REG_APB5_GPIOB_DATA = GPIO_DATA_BACKUP[1];
    REG_APB5_GPIOC_DATA = GPIO_DATA_BACKUP[2];
    REG_APB5_GPIOD_DATA = GPIO_DATA_BACKUP[3];
    REG_APB5_GPIOE_DATA = GPIO_DATA_BACKUP[4];
}




void DEBUG_MSG(uint8_t x)
{
#if GPIO_DBG_MSG
    REG_APB5_GPIOA_DATA = x & 0xff;
    gpio_triger(GPIOD_4);
#endif
}


