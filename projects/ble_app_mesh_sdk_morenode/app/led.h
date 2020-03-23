/**
 ****************************************************************************************
 *
 * @file led.h
 *
 * @brief led Driver for led operation.
 *
 * Copyright (C) Beken 2009-2015
 *
 *
 ****************************************************************************************
 */

#ifndef _LED_H_
#define _LED_H_
#include <stddef.h>    // standard definitions
#include <stdint.h>    // standard integer definition
#include <stdbool.h>
#include "pwm.h"

enum
{

    COLOR_WARM_WHITE,
    COLOR_PURE_WHITE,
    COLOR_BLUE,
    COLOR_RED,
    COLOR_GREEN,
    COLOR_NB,
};


#define LED_PWM_END_VALUE 65500
#define LED_PWM_STEP_VALUE (655 * 4)

extern struct led_env_tag led_env;

void led_init(void);
void led_deinit(void);

void hsl_2_rgb(uint16_t rgb[3], uint16_t hsl[3]);

void pwm6_irq_done(void);


//PWM时钟选择为16M时，周期(endvalue)计算方法为，
//endvalue = 16MHZ /目标值，
//例如:设置16KHZ的周期，带入公式如下，
//endvalue = 16MHZ /16KHZ = 1000
//#define   PWM_16KHZ           1000
#define     PWM_16KHZ           PWM_Frequency
#define     PWM_Frequency       PWM_1KHZ

#define     PWM_1KHZ            16000

#define     PWM_250HZ           64000

#define     PWM_DUTY_INIT       0

#define     PWM_DUTY_STEP   10 // 10(200MS)  ->  2(1S)  

//Temperature MAX
//#define TEMPERATURE_MAX   (0X4E20-0X320)

//CALC:16MHZ /500HZ(2MS) =
#define  TIMER_INTERVAL_VALUE   32000

#define TEMPERATURE_MAX 6500
#define TEMPERATURE_MIN 2700
#define TEMPERATURE_WIDTH (TEMPERATURE_MAX - TEMPERATURE_MIN)



//#define DEFAULT_MOVE_STEP_NUM  500  // default:65535 to 0 uses 1000ms, timer interval 2ms, step = 1000/ 2 =500
#define DEFAULT_PWM_MOVE_STEP 131 // 65535/DEFAULT_MOVE_STEP_NUM

#define MIN_MOVE_STEP_NUM 300 //default min timer is 2ms*300=600ms

extern PWM_DRV_DESC pwm1;   //WARM -> P10
extern PWM_DRV_DESC pwm2;   //PURE  -> P11
extern PWM_DRV_DESC pwm3;   //B       ->P12
extern PWM_DRV_DESC pwm4;   //R      ->P13
extern PWM_DRV_DESC pwm5;   //G    -> P14
extern PWM_DRV_DESC pwm6;   //for 2ms timer



#define LIGHT_POWER_ON_COUNT                            5 //!< close the light LIGHT_POWER_ON_COUNT times to reset
#define LIGHT_POWER_ON_TIME                             20000 //!< millisecond
#define LIGHT_PROV_FLASH_INTERVAL                   500 //!< millisecond


#define LIGHT_GRA_TIME                                  200//!< millisecond
#define LED_PWM_FREQ        16000 //!< Hz
#define LED_PWM_COUNT       16000 // (40000000/LED_PWM_FREQ)

/* Defines ------------------------------------------------------------------*/

typedef struct
{
    uint32_t period;
    uint8_t used;
    uint8_t padding[3];
} flash_light_upload_period_t;

typedef struct
{
    uint8_t count;
    uint8_t used;
    uint8_t padding[2];
} flash_light_power_on_count_t;


//    LED_W= 0,
//    LED_C,
typedef enum
{
    LED_C= 0,
    LED_W,
    LED_R,
    LED_G,
    LED_B,
    LED_NUM
} light_channel_t;


enum light_gra_dir
{
    ADD_DIR = 1,
    REDU_DIR
} ;

enum light_status
{
    LIGHT_FREE = 0,
    LIGHT_START,
    LIGHT_BUSY,

} ;

enum
{
    GEN_ONOFF_OFF = 0,
    GEN_ONOFF_ON,
};

enum
{
    LIGHT_STS_NO_CHANGE = 0,
    LIGHT_STS_CHANGED,
};


enum
{
    LIGHT_MODE_CTL = 0,
    LIGHT_MODE_RGB,
};

typedef struct
{
    uint8_t r;
    uint8_t g;
    uint8_t b;
} rgb_value_t;

typedef struct
{
    uint32_t col_tmp;//2000-7000
    uint32_t lv;//0-100
    uint16_t cl;//0-65535
    uint16_t wm;//0-65535

} cw_value_t;

typedef struct
{
    double hue;
    double sat;
    double lig;
} hsl_value_t;

typedef struct
{
    uint16_t hue;
    uint8_t sat;
    uint8_t val;
} hsv_value_t;
typedef struct
{
    uint8_t main_switch;
    rgb_value_t rgb;
    uint16_t bri;//0-65535
    uint8_t work_mode;
    cw_value_t cw;
    uint8_t night_switch;
    hsl_value_t hsl;
    hsv_value_t hsv;
    uint8_t chan_mode;//0 rgb 1 cold warm
} light_property_t;

typedef struct
{


    //light lightness states
    uint16_t light_lightness;
    //  uint16_t light_lightness_bak;


    //CTL model states
    //uint16_t ctl_lightness;
    uint16_t ctl_temperature;
    uint16_t ctl_delta_uv;

    //HSL model states
    uint16_t hsl_hue;
    uint16_t hsl_saturation;
    uint16_t hsl_lightness;

    //light mode value
    uint8_t light_mode;

    uint16_t write_mark;
} flash_light_state_t;


typedef enum
{
    FLASH_LIGHT_PARAM_TYPE_LIGHT_STATE,
    FLASH_LIGHT_PARAM_TYPE_UPLOAD_PERIOD,
    FLASH_LIGHT_PARAM_TYPE_POWER_ON_COUNT
} flash_light_param_type_t;

extern uint16_t light_cwrgb[5];
extern uint16_t light_cwrgb_last[5];
extern uint32_t light_hsl[3];
extern uint16_t light_hsl_16[3];

extern uint16_t light_ctl[3];
extern uint16_t light_temperature;
extern uint16_t light_delta_uv;

extern light_property_t now_light_prop;
extern light_property_t gra_light_prop;

void light_mode_set(uint8_t mode);


void light_state_set(uint8_t state); // on/off{

uint8_t light_mode_get(void); // on/off{

uint8_t light_state_get(void); // on/off



void light_set_cwrgb(uint16_t cwrgb[5]);


void light_lightness_set(uint16_t lightness);

void light_ctl_hsl_lightness_set(uint16_t lightness);

void light_set_hsl(uint16_t ln_lightness, uint16_t hue, uint16_t saturation, uint16_t lightness);
void light_set_hsl_hue(uint16_t hue);
void light_set_hsl_saturation(uint16_t saturation);

void light_set_ctl(uint16_t lightness, uint16_t temperature, int16_t delta_uv);
void light_set_ctl_temp(uint16_t temperature, int16_t delta_uv);

void light_linear_32_to_16(uint32_t hsl32[3], uint16_t hsl16[3]);


void light_status_init(void);
void lig_gar_init(void);

void light_lighten(light_channel_t channel, uint16_t state);
uint8_t light_onoff(uint8_t onoff);

void light_prov_complete(void);
void light_prov_fail(void);

void LED_default_ctrl_param_set(uint32_t pwm_move_step_param, uint32_t min_move_step_num_param);

uint32_t light_state_nv_store(flash_light_param_type_t type);
uint32_t light_app_nv_restore(flash_light_param_type_t type);
void light_factory_reset(void);

void light_state_recover(void);

#endif //



