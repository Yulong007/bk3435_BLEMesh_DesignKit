
/**
 ****************************************************************************************
 *
 * @file led.h
 *
 * @brief led Driver for pwm operation.
 *
 * Copyright (C) Beken 2009-2015
 *
 *
 ****************************************************************************************
 */

#include "led.h"
#include "pwm.h"
#include <stddef.h>    // standard definitions
#include <stdint.h>    // standard integer definition
#include "mesh_tb_timer.h"
#include <math.h>
#include "gpio.h"
#include "rwip.h"
#include "mesh_log.h"

/**
 ****************************************************************************************
 *
 * @file led.h
 *
 * @brief led Driver for pwm operation.
 *
 * Copyright (C) Beken 2009-2015
 *
 *
 ****************************************************************************************
 */


#include <stddef.h>    // standard definitions
#include <stdint.h>    // standard integer definition
#include "m_tb_state.h"

#include "led.h"
#include "pwm.h"
#include "nvds.h"
#include "app.h"
#include "uart.h"


PWM_DRV_DESC pwm1;  //WARM -> P10
PWM_DRV_DESC pwm2;  //PURE  -> P11
PWM_DRV_DESC pwm3;  //B       ->P12
PWM_DRV_DESC pwm4;  //R      ->P13
PWM_DRV_DESC pwm5;  //G    -> P14
PWM_DRV_DESC pwm6;  //for 2ms timer

uint8_t light_mode = 0;
uint8_t light_state = 0;
uint16_t light_lightness;
uint16_t light_lightness_bak;


//CTL model states
uint16_t ctl_lightness;
uint16_t ctl_temperature = 800;;
uint16_t ctl_delta_uv = 0;

//HSL model states
uint16_t hsl_lightness;
uint16_t hsl_hue;
uint16_t hsl_saturation;

/* light state */
uint16_t light_cwrgb[5] = {0, 0xffff, 0, 0, 0}; //!< c,w,r,g,b // save to flash
//uint16_t light_cwrgb_last[5] = {0,0xffff,0,0,0}; //!< used by hsl models
//uint32_t light_hsl[3]; //!< in case of hue/saturation info loss, todo: save to flash?
//uint16_t light_temperature;
//uint16_t light_delta_uv;
//uint16_t light_hsl_16[3]; //
//uint16_t light_ctl[3]; //!< temperature, delta_uv, lightness
//uint32_t light_upload_period = 15; //!< unit: second

static uint16_t pwm_duty_scale[5] = {0, 0, 0, 0, 0};

uint8_t gen_onoff ;

/* light factory restore */
uint8_t power_on_count;
mesh_tb_timer_t power_on_timer;
mesh_tb_timer_t light_gra_timer;
//light_property_t now_light_prop;
//light_property_t gra_light_prop;
uint8_t light_status = 1;
uint8_t led_timer_status = LIGHT_FREE;

static uint8_t prov_comp_light_cnt = 0;
static uint8_t uprov_light_cnt = 0;



#define pwm_duty_scale_1   120//546
#define half_pwm_duty_scale_1  273

#define pwm_duty_scale_2   80

#define  pwm_low_line_250   250

#define  pwm_low_line       250

void light_periodic_ctrl(void);


void pwm6_irq_done(void)
{
    pwm_isr();
    light_periodic_ctrl();
}

void light_cwrgb_init(void)
{
    rwip_prevent_sleep_set(BK_DRIVER_TIMER_ACTIVE);
#if (POWER_ONOFF_ONLY)
    return; //xuhai
#endif
    //pwm_disable(0);
#if 0
    pwm1.channel = 0;       //channel1(pwm2)
    pwm1.mode = (1 << 0)    //channel1(pwm2) enbale;default pwm mode
                | (1 << 4);    // select 16MHz
    pwm1.pre_divid = 1;
    pwm1.end_value = PWM_16KHZ;
    pwm1.duty_cycle = PWM_DUTY_INIT ;//PWM_DUTY_INIT;
    pwm1.duty_step = PWM_DUTY_STEP;
    pwm_init(&pwm1);


    pwm2.channel = 1;       //channel1(pwm2)
    pwm2.mode = (1 << 0)    //channel1(pwm2) enbale;default pwm mode
                | (1 << 4);  // select 16MHz
    pwm2.pre_divid = 1;
    pwm2.end_value = PWM_16KHZ;
    pwm2.duty_cycle = PWM_DUTY_INIT;//PWM_DUTY_INIT;
    pwm2.duty_step = PWM_DUTY_STEP;
    pwm_init(&pwm2);


    pwm3.channel = 2;
    pwm3.mode = (1 << 0)
                | (1 << 4);
    pwm3.pre_divid = 1;
    pwm3.end_value = PWM_16KHZ;
    pwm3.duty_cycle = PWM_DUTY_INIT;
    pwm3.duty_step = PWM_DUTY_STEP;
    pwm_init(&pwm3);

    pwm4.channel = 3;
    pwm4.mode = (1 << 0)
                | (1 << 4);
    pwm4.pre_divid = 1;
    pwm4.end_value = PWM_16KHZ ;
    pwm4.duty_cycle = PWM_DUTY_INIT;//PWM_DUTY_INIT;
    pwm4.duty_step = PWM_DUTY_STEP;
    pwm_init(&pwm4);

    pwm5.channel = 4;
    pwm5.mode = (1 << 0)
                //|(1<<1)       //pwm/timer int enable
                | (1 << 4);
    pwm5.pre_divid = 1;
    pwm5.end_value = PWM_16KHZ;
    pwm5.duty_cycle = PWM_DUTY_INIT;//PWM_DUTY_INIT;
    pwm5.duty_step = PWM_DUTY_STEP;
    pwm_init(&pwm5);

#endif

#if 0
    pwm6.channel = 5;           //channel5(PWM6)
    pwm6.mode = (1 << 0)            //pwm/timer enable
                |(1<<1)       //pwm/timer int enable
                |(0x01 << 2)    //select timer mode
                | (1 << 4);     // select 16MHz
    //  REG_AHB0_ICU_INT_ENABLE |= (0x01 << pwm6.channel ); //enable pwmX int //181109
    //   pwm6.p_Int_Handler = pwm6_irq_done;
    pwm6.pre_divid = 1;
    pwm6.end_value = TIMER_INTERVAL_VALUE;//65535; 2ms
    //  pwm6.duty_cycle = 0x4000;
    pwm_init(&pwm6);
#endif

}


void led_deinit(void)
{
#if 1
    pwm_disable(0);
    pwm_disable(1);
    pwm_disable(2);
    pwm_disable(3);
    pwm_disable(4);
    pwm_disable(5);
#endif
    rwip_prevent_sleep_clear(BK_DRIVER_TIMER_ACTIVE);
}






static const uint32_t temperature_map[] =
{
    0xff2000, 0xff2700, 0xff3300, 0xff4500, 0xff5200, 0xff5d00, 0xff6600, 0xff6f00,
    0xff7600, 0xff7c00, 0xff8200, 0xff8700, 0xff8d0b, 0xff921d, 0xff9829, 0xff9d33,
    0xffa23c, 0xffa645, 0xffaa4d, 0xffae54, 0xffb25b, 0xffb662, 0xffb969, 0xffbd6f,
    0xffc076, 0xffc37c, 0xffc682, 0xffc987, 0xffcb8d, 0xffce92, 0xffd097, 0xffd39c,
    0xffd5a1, 0xffd7a6, 0xffd9ab, 0xffdbaf, 0xffddb4, 0xffdfb8, 0xffe1bc, 0xffe2c0,
    0xffe4c4, 0xffe5c8, 0xffe7cc, 0xffe8d0, 0xffead3, 0xffebd7, 0xffedda, 0xffeede,
    0xffefe1, 0xfff0e4, 0xfff1e7, 0xfff3ea, 0xfff4ed, 0xfff5f0, 0xfff6f3, 0xfff7f5,
    0xfff8f8, 0xfff9fb, 0xfff9fd, 0xfefaff, 0xfcf8ff, 0xfaf7ff, 0xf7f5ff, 0xf5f4ff,
    0xf3f3ff, 0xf1f1ff, 0xeff0ff, 0xeeefff, 0xeceeff, 0xeaedff, 0xe9ecff, 0xe7eaff,
    0xe5e9ff, 0xe4e9ff, 0xe3e8ff, 0xe1e7ff, 0xe0e6ff, 0xdfe5ff, 0xdde4ff, 0xdce3ff,
    0xdbe2ff, 0xdae2ff, 0xd9e1ff, 0xd8e0ff, 0xd7dfff, 0xd6dfff, 0xd5deff, 0xd4ddff,
    0xd3ddff, 0xd2dcff, 0xd1dcff, 0xd0dbff, 0xcfdaff, 0xcfdaff, 0xced9ff, 0xcdd9ff,
    0xccd8ff, 0xccd8ff, 0xcbd7ff, 0xcad7ff, 0xcad6ff, 0xc9d6ff, 0xc8d5ff, 0xc8d5ff,
    0xc7d4ff, 0xc6d4ff, 0xc6d4ff, 0xc5d3ff, 0xc5d3ff, 0xc4d2ff, 0xc4d2ff, 0xc3d2ff,
    0xc3d1ff, 0xc2d1ff, 0xc2d0ff, 0xc1d0ff, 0xc1d0ff, 0xc0cfff, 0xc0cfff, 0xbfcfff,
    0xbfceff, 0xbeceff, 0xbeceff, 0xbeceff, 0xbdcdff, 0xbdcdff, 0xbccdff, 0xbcccff,
    0xbcccff, 0xbbccff, 0xbbccff, 0xbbcbff, 0xbacbff, 0xbacbff, 0xbacbff, 0xb9caff,
    0xb9caff, 0xb9caff, 0xb8caff, 0xb8c9ff, 0xb8c9ff, 0xb8c9ff, 0xb7c9ff, 0xb7c9ff,
    0xb7c8ff, 0xb6c8ff, 0xb6c8ff, 0xb6c8ff, 0xb6c8ff, 0xb5c7ff, 0xb5c7ff, 0xb5c7ff,
    0xb5c7ff, 0xb4c7ff, 0xb4c6ff, 0xb4c6ff, 0xb4c6ff, 0xb3c6ff, 0xb3c6ff, 0xb3c6ff,
    0xb3c5ff, 0xb3c5ff, 0xb2c5ff, 0xb2c5ff, 0xb2c5ff, 0xb2c5ff, 0xb2c4ff, 0xb1c4ff,
    0xb1c4ff, 0xb1c4ff, 0xb1c4ff, 0xb1c4ff, 0xb0c4ff, 0xb0c3ff, 0xb0c3ff, 0xb0c3ff,
    0xb0c3ff, 0xb0c3ff, 0xafc3ff, 0xafc3ff, 0xafc2ff, 0xafc2ff, 0xafc2ff, 0xafc2ff,
    0xaec2ff, 0xaec2ff, 0xaec2ff, 0xaec2ff, 0xaec2ff, 0xaec1ff, 0xaec1ff, 0xadc1ff,
    0xadc1ff
};

float hue_2_rgb(float v1, float v2, float h)
{
    if (h < 0)
    {
        h += 1;
    }
    else if (h > 1)
    {
        h -= 1;
    }
    if (6 * h < 1)
    {
        return v1 + (v2 - v1) * 6 * h;
    }
    else if (2 * h < 1)
    {
        return v2;
    }
    else if (3 * h < 2)
    {
        return v1 + (v2 - v1) * (4 - 6 * h);
    }
    else
    {
        return v1;
    }
}

/**
 * Converts an HSL color value to RGB. Conversion formula
 * adapted from http://en.wikipedia.org/wiki/HSL_color_space.
 * Assumes h, s, and l are contained in the set [0, 1] and
 * returns r, g, and b in the set [0, 255].
 *
 * @param   Number  h       The hue
 * @param   Number  s       The saturation
 * @param   Number  l       The lightness
 * @return  Array           The RGB representation
 */

void hsl_2_rgb(uint16_t rgb[3], uint16_t hsl[3])
{
    if (hsl[1] == 0)
    {
        rgb[0] = rgb[1] = rgb[2] = hsl[2];
    }
    else
    {
        float h, s, l, v1, v2;
        h = (float)hsl[0] / 65535.0;
        s = (float)hsl[1] / 65535.0;
        l = (float)hsl[2] / 65535.0;
        if (l < 0.5f)
        {
            v2 = l * (1.0 + s);
        }
        else
        {
            v2 = (l + s) - (s * l);
        }
        v1 = 2.0 * l - v2;
        rgb[0] = ceil(65535 * hue_2_rgb(v1, v2, h + 1.0 / 3));
        rgb[1] = ceil(65535 * hue_2_rgb(v1, v2, h));
        rgb[2] = ceil(65535 * hue_2_rgb(v1, v2, h - 1.0 / 3));
    }
}


#ifndef MAX
#define MAX(a,b) ((a) > (b) ? (a) : (b))
#define MIN(a,b) ((a) < (b) ? (a) : (b))
#endif

/* will lost hue/saturatin info when lightness is highest */
void rgb_2_hsl(uint16_t hsl[3], uint16_t rgb[3])
{
    uint16_t max, min;
    max = MAX(rgb[0], rgb[1]);
    max = MAX(max, rgb[2]);
    min = MIN(rgb[0], rgb[1]);
    min = MIN(min, rgb[2]);
    if (max == min)
    {
        hsl[0] = 0;
    }
    else if (max == rgb[0]) //!< red: 0 degree
    {
        if (rgb[1] >= rgb[2])
        {
            hsl[0] = (65536 / 6) * (rgb[1] - rgb[2]) / (max - min);
        }
        else
        {
            uint32_t tmp = 65536 - (65536 / 6) * (rgb[2] - rgb[1]) / (max - min);
            if (tmp == 65536)
            {
                hsl[0] = 0;
            }
            else
            {
                hsl[0] = tmp;
            }
        }
    }
    else if (max == rgb[1]) //!< green: 120 degree
    {
        hsl[0] = 65536 / 3 + (65536 / 6) * (rgb[2] - rgb[0]) / (max - min);
    }
    else //!< blue: 240 degree
    {
        hsl[0] = 65536 * 2 / 3 + (65536 / 6) * (rgb[0] - rgb[1]) / (max - min);
    }
    hsl[2] = (max + min) >> 1;
    if (hsl[2] == 0 || max == min)
    {
        hsl[1] = 0;
    }
    else if (hsl[2] <= 32767)
    {
        hsl[1] = ceil(65535 * (double)(max - min) / (max + min));
    }
    else
    {
        hsl[1] = ceil(65535 * (double)(max - min) / (2 * 65535 - max - min));
    }
}

static uint8_t color_16to8(uint16_t color)
{
    return color / 65535.0 * 255;
}


/**
 * @note do not support delta uv right now
 */
void rgb_2_ctl(uint16_t ctl[3], uint16_t cwrgb[5])
{
    ctl[1] = 0;
    ctl[2] = cwrgb[0];

    uint32_t min_delta = 0xffffffff;
    uint32_t delta = 0;
    uint16_t min_index = 0;
    uint32_t rgb = color_16to8(cwrgb[2]);
    rgb <<= 8;
    rgb |= color_16to8(cwrgb[3]);
    rgb <<= 8;
    rgb = color_16to8(cwrgb[4]);
    rgb <<= 8;
    for (uint16_t i = 0; i < sizeof(temperature_map) / sizeof(temperature_map[0]); ++i)
    {
        delta = (temperature_map[i] > rgb) ? (temperature_map[i] - rgb) : (rgb - temperature_map[i]);
        if (delta < min_delta)
        {
            min_delta = delta;
            min_index = i;
        }
    }
    ctl[0] = 800 + min_index * 100;
}

uint32_t temperature_to_rgb(uint16_t temperature, int16_t delta_uv)
{
    UNUSED(delta_uv);
    uint16_t index = round((temperature - 800) / 100.0);
    return temperature_map[index];
}



static  uint16_t color_8to16(uint8_t color)
{
    return color / 255.0 * 65535;
}
void light_set_cwrgb(uint16_t cwrgb[5])
{
    bool save_flash = false;
    uint8_t channel;


    MESH_APP_PRINT_INFO("********************************-light_set_cwrgb  STS = %d\r\n", led_timer_status);
    MESH_APP_PRINT_INFO("light_state :%s\r\n", light_state_get() == 1? "light_on" : "light_off");
    MESH_APP_PRINT_INFO("light_mode :%s\r\n", light_mode_get() == 1? "LIGHT_MODE_RGB" : "LIGHT_MODE_CTL");
    MESH_APP_PRINT_INFO("light_cwrgb[0] = %x\r\n", cwrgb[0]);
    MESH_APP_PRINT_INFO("light_cwrgb[1] = %x\r\n", cwrgb[1]);
    MESH_APP_PRINT_INFO("light_cwrgb[2] = %d\r\n", cwrgb[2]);
    MESH_APP_PRINT_INFO("light_cwrgb[3] = %d\r\n", cwrgb[3]);
    MESH_APP_PRINT_INFO("light_cwrgb[4] = %d\r\n", cwrgb[4]);

    MESH_APP_PRINT_INFO("*************************************************************\r\n");

    if (led_timer_status == LIGHT_FREE)
    {
        for (channel = 0; channel < 5; channel++)
        {
            light_cwrgb[channel] = cwrgb[channel];
        }
//        MESH_APP_PRINT_INFO("light_cwrgb[0] = %x\r\n",light_cwrgb[0]);
//        MESH_APP_PRINT_INFO("light_cwrgb[1] = %x\r\n",light_cwrgb[1]);
//        MESH_APP_PRINT_INFO("light_cwrgb[2] = %d,light_cwrgb[3] = %d,light_cwrgb[4] = %d\r\n",light_cwrgb[2],light_cwrgb[3],light_cwrgb[4]);
        led_timer_status = LIGHT_START;

    }
    else
    {
        MESH_APP_PRINT_INFO("light_set_cwrgb BUSY %d\n", led_timer_status);
    }
}


void light_set_hsl_hue(uint16_t hue)
{
    MESH_APP_PRINT_INFO("light_set_hsl hue = %d\n", hue);
    hsl_hue = hue;

}

void light_set_hsl_saturation(uint16_t saturation)
{
    MESH_APP_PRINT_INFO("light_set_hsl saturation = %d\n", saturation);
    hsl_saturation = saturation;

}

void light_set_hsl_lightness(uint16_t lightness)
{
    MESH_APP_PRINT_INFO("light_set_hsl ln_lightness = %d\n", lightness);
    hsl_lightness = lightness;
}


void light_set_hsl(uint16_t ln_lightness, uint16_t hue, uint16_t saturation, uint16_t lightness)
{
    MESH_APP_PRINT_INFO("light_set_hsl ln_lightness = %d\n", ln_lightness);
    MESH_APP_PRINT_INFO("hue = %d\n", hue);
    MESH_APP_PRINT_INFO("saturation = %d\n", saturation);
    MESH_APP_PRINT_INFO("lightness = 0x%x\n", lightness);

    uint16_t cwrgb[5];
    uint16_t hsl[3];
    uint16_t rgb[3];

    hsl[0] = hue;
    hsl[1] = saturation;
    hsl[2] = lightness;

    hsl_2_rgb(rgb, hsl);
    MESH_APP_PRINT_INFO("r = %d,g = %d,b = %d\n", rgb[0], rgb[1], rgb[2]);

    cwrgb[0] = 0;
    cwrgb[1] = 0;
    cwrgb[2] = rgb[0] * ln_lightness / 65535;
    cwrgb[3] = rgb[1] * ln_lightness / 65535;
    cwrgb[4] = rgb[2] * ln_lightness / 65535;
    light_set_cwrgb(cwrgb);
}



void light_set_ctl_temp(uint16_t temperature, int16_t delta_uv)
{
    uint16_t cwrgb[5];
    MESH_APP_PRINT_INFO("light_set_ctl_temp temperature =0x%x,delta_uv = 0x%x\n", temperature, delta_uv);

    ctl_temperature = temperature;
    ctl_delta_uv = delta_uv;
}

void light_set_ctl(uint16_t lightness, uint16_t temperature, int16_t delta_uv)
{
    uint16_t cwrgb[5];
    MESH_APP_PRINT_INFO("light_set_ctl ln_lightness = %d\n", lightness);
    MESH_APP_PRINT_INFO("temperature = %d\n", temperature);
    MESH_APP_PRINT_INFO("delta_uv = %d\n", delta_uv);

    uint32_t rgb = temperature_to_rgb(temperature, delta_uv);
    cwrgb[0] = 0;
    cwrgb[1] = 0;
    cwrgb[2] = color_8to16(rgb >> 16) * lightness / 65535;
    cwrgb[3] = color_8to16(rgb >> 8) * lightness / 65535;
    cwrgb[4] = color_8to16(rgb) * lightness / 65535;
    light_set_cwrgb(cwrgb);
}



void light_mode_set(uint8_t mode)// CTL / RGB
{
    if (mode > 1)light_mode = 0;
    light_mode = mode;
}

void light_state_set(uint8_t state) // on/off
{
    light_state = state;
}

uint8_t light_mode_get(void) // CTL / RGB
{
    return light_mode;
}

uint8_t light_state_get(void) // on/off
{
    return light_state;
}


void light_lightness_set(uint16_t lightness)
{
    uint8_t sta;
    MESH_APP_PRINT_INFO("light_lightness_set\r\n");
    if (lightness != 0)
    {

        if (light_state_get() == 1) // light is on->on
        {
            MESH_APP_PRINT_INFO("light is on->on\r\n");
            light_state_set(1);
            if (light_mode_get() == LIGHT_MODE_RGB)
            {
                light_lightness = lightness;
                light_set_hsl(light_lightness, hsl_hue, hsl_saturation, hsl_lightness);
            }
            else if (light_mode_get() == LIGHT_MODE_CTL)
            {
                //  ctl_lightness = lightness;
                light_lightness = lightness;
                light_set_ctl(light_lightness, ctl_temperature, ctl_delta_uv);
            }
#if (POWER_ONOFF_ONLY)
            gpio_set(0x10, 1);
#endif

        }
        else  // light is off->on
        {
            MESH_APP_PRINT_INFO("light is off->on\r\n");
            light_lightness = lightness;
            light_state_set(1);
            if (light_mode_get() == LIGHT_MODE_RGB)
            {
                light_set_hsl(light_lightness, hsl_hue, hsl_saturation, hsl_lightness);
            }
            else if (light_mode_get() == LIGHT_MODE_CTL)
            {
                ctl_lightness = lightness;
                light_set_ctl(light_lightness, ctl_temperature, ctl_delta_uv);
            }
#if (POWER_ONOFF_ONLY)
            gpio_set(0x10, 1);
#endif
        }


    }
    else
    {
        MESH_APP_PRINT_INFO("light is on->off\r\n");
        light_lightness = lightness;
        light_state_set(0);
        if (light_mode == LIGHT_MODE_RGB)
        {
            light_set_hsl(light_lightness, hsl_hue, hsl_saturation, hsl_lightness);
        }
        else if (light_mode == LIGHT_MODE_CTL)
        {
            //  ctl_lightness = lightness;
            light_lightness = lightness;
            light_set_ctl(light_lightness, ctl_temperature, ctl_delta_uv);
        }
#if (POWER_ONOFF_ONLY)
        gpio_set(0x10, 0);
#endif
    }

}


void light_ctl_hsl_lightness_set(uint16_t lightness)
{
    MESH_APP_PRINT_INFO("light_hsl_lightness_set\r\n");
    if (lightness != 0)
    {

        if (light_state_get() == 1) // light is on->on
        {
            MESH_APP_PRINT_INFO("light is on->on\r\n");
            light_state_set(1);
            if (light_mode_get() == LIGHT_MODE_RGB)
            {
                hsl_lightness = lightness;
                light_set_hsl(light_lightness, hsl_hue, hsl_saturation, hsl_lightness);
            }
            else if (light_mode_get() == LIGHT_MODE_CTL)
            {
                ctl_lightness = lightness;
                //   light_lightness = lightness;
                light_set_ctl(light_lightness, ctl_temperature, ctl_delta_uv);
            }
        }
        else  // light is off->on
        {
            MESH_APP_PRINT_INFO("light is off->on\r\n");
            // light_lightness = lightness;
            light_state_set(1);
            if (light_mode_get() == LIGHT_MODE_RGB)
            {
                light_set_hsl(light_lightness, hsl_hue, hsl_saturation, hsl_lightness);
            }
            else if (light_mode_get() == LIGHT_MODE_CTL)
            {
                ctl_lightness = lightness;
                light_set_ctl(light_lightness, ctl_temperature, ctl_delta_uv);
            }
        }
    }
    else
    {
        MESH_APP_PRINT_INFO("light is on->off\r\n");
        //  light_lightness = lightness;
        light_state_set(0);
        if (light_mode == LIGHT_MODE_RGB)
        {
            light_set_hsl(light_lightness, hsl_hue, hsl_saturation, hsl_lightness);
        }
        else if (light_mode == LIGHT_MODE_CTL)
        {
            ctl_lightness = lightness;
            //   light_lightness = lightness;
            light_set_ctl(light_lightness, ctl_temperature, ctl_delta_uv);
        }

    }

}



void light_lighten(light_channel_t channel, uint16_t state)
{
    if (channel > LED_B)
    {
        return;
    }
    uint32_t high_count;
    if (state == 0xffff)
    {
        high_count = LED_PWM_COUNT;
    }
    else
    {
        high_count = (LED_PWM_COUNT / 65535.0) * state;
    }

    switch (channel)
    {
        case LED_C:
        {
            pwm2.duty_cycle = high_count;//PWM_DUTY_INIT;
            pwm_duty_cycle(&pwm2);

        } break;
        case LED_W:
        {
            pwm1.duty_cycle = high_count;//PWM_DUTY_INIT;
            pwm_duty_cycle(&pwm1);

        } break;
        case LED_R:
        {
            pwm4.duty_cycle = high_count;//PWM_DUTY_INIT;
            pwm_duty_cycle(&pwm4);

        } break;
        case LED_G:
        {
            pwm5.duty_cycle = high_count;//PWM_DUTY_INIT;
            pwm_duty_cycle(&pwm5);
        } break;
        case LED_B:
        {
            pwm3.duty_cycle = high_count;//PWM_DUTY_INIT;
            pwm_duty_cycle(&pwm3);
        } break;

        default:
            break;
    }
}

uint32_t pwm_move_step;
uint32_t min_move_step_num;
//uint32_t led_trans_time = 1000; //1000ms  MAY BE VARIABLE IN THE FUTURE

void LED_default_ctrl_param_set(uint32_t pwm_move_step_param, uint32_t min_move_step_num_param)
{
    pwm_move_step = pwm_move_step_param;
    min_move_step_num = min_move_step_num_param;
}

void light_periodic_ctrl(void)
{
    static uint16_t LED_pwm_current[5] = {0, 0, 0, 0, 0};
    static uint16_t LED_pwm_move_step[5] = {0, 0, 0, 0, 0};
    //  uint8_t ledId;

    light_channel_t ledId;

    switch (led_timer_status)
    {
        case LIGHT_FREE:
        {
            return;
        }
        case LIGHT_START: //calc move step
        {
            uint32_t stepNum;

            for (ledId = LED_C; ledId<LED_NUM; ledId++)
            {
                if (LED_pwm_current[ledId] == light_cwrgb[ledId])
                {
                    LED_pwm_move_step[ledId] = 0;
                    continue;
                }
                else if (LED_pwm_current[ledId] > light_cwrgb[ledId])
                {
                    stepNum = (LED_pwm_current[ledId] - light_cwrgb[ledId])/pwm_move_step;
                    if (stepNum >= min_move_step_num)
                    {
                        LED_pwm_move_step[ledId] = pwm_move_step;
                    }
                    else
                    {
                        LED_pwm_move_step[ledId] = (LED_pwm_current[ledId] - light_cwrgb[ledId])/min_move_step_num;
                    }
                }
                else
                {
                    stepNum = (light_cwrgb[ledId] -LED_pwm_current[ledId])/pwm_move_step;
                    //MESH_APP_PRINT_INFO("num[%d]=%d,li=%d,ocu=%d\r\n",ledId,stepNum,light_cwrgb[ledId],LED_pwm_current[ledId]);
                    if (stepNum >= min_move_step_num)
                    {
                        LED_pwm_move_step[ledId] = pwm_move_step;
                    }
                    else
                    {
                        LED_pwm_move_step[ledId] = (light_cwrgb[ledId] - LED_pwm_current[ledId])/min_move_step_num;
                    }
                }
                //MESH_APP_PRINT_INFO("step[%d] = %d\r\n",ledId,LED_pwm_move_step[ledId]);
            }
            led_timer_status = LIGHT_BUSY;
        }
        case LIGHT_BUSY:  //change LED
        {
            static uint32_t testCount = 0;
            uint8_t equalCount = 0;
            testCount++;
            //////MESH_APP_PRINT_INFO("tc=%d\r\n",testCount);
            for (ledId = LED_C; ledId<LED_NUM; ledId++)
            {
                if (LED_pwm_current[ledId] == light_cwrgb[ledId])
                {
                    equalCount++;
                    continue;
                }
                else if (LED_pwm_current[ledId] > light_cwrgb[ledId])
                {
                    if (((LED_pwm_current[ledId] - light_cwrgb[ledId]) < LED_pwm_move_step[ledId]) ||
                            (LED_pwm_move_step[ledId] == 0))
                    {
                        LED_pwm_current[ledId] = light_cwrgb[ledId];
                    }
                    else
                    {
                        LED_pwm_current[ledId] -= LED_pwm_move_step[ledId];
                    }
                    light_lighten(ledId, LED_pwm_current[ledId]);
                }
                else
                {
                    if (((light_cwrgb[ledId] - LED_pwm_current[ledId]) < LED_pwm_move_step[ledId]) ||
                            (LED_pwm_move_step[ledId] == 0))
                    {
                        LED_pwm_current[ledId] = light_cwrgb[ledId];
                    }
                    else
                    {
                        LED_pwm_current[ledId] += LED_pwm_move_step[ledId];
                    }
                    light_lighten(ledId, LED_pwm_current[ledId]);
                }
                //////MESH_APP_PRINT_INFO("cur[%d]=%x,tgt=%x\r\n",ledId,LED_pwm_current[ledId],light_cwrgb[ledId]);
            }

            if (equalCount == 5)
            {
                led_timer_status = LIGHT_FREE;
                //MESH_APP_PRINT_INFO("fin=%d,tc=%d\r\n",equalCount,testCount);
            }
            break;
        }


        default:
            MESH_APP_PRINT_INFO("**************light status error***********%d*\r\n", led_timer_status);
            break;
    }
}

void light_status_init(void)
{
    light_cwrgb_init();
    LED_default_ctrl_param_set(DEFAULT_PWM_MOVE_STEP, MIN_MOVE_STEP_NUM);
}



void light_factory_reset(void)
{
    light_mode_set(LIGHT_MODE_CTL);
    light_lightness_set(0x8000);

}
void light_state_recover(void)
{
    light_state_set(0);
    light_lightness_set(light_lightness);

}


uint16_t quick_onoff_count = 0;
uint32_t light_app_nv_restore(flash_light_param_type_t type)
{
    uint32_t ret;
    nvds_tag_len_t len;
#if 1
    MESH_APP_PRINT_INFO("light_app_nv_restore \r\n");
    switch (type)
    {
        case FLASH_LIGHT_PARAM_TYPE_POWER_ON_COUNT:
        {
            uint32_t flash_light_power_on_count;

            len = sizeof(flash_light_power_on_count);
            ret = nvds_get(NVDS_TAG_POWER_RESET_CNT, &len, (uint8_t*)&quick_onoff_count);
        }
        break;
        case FLASH_LIGHT_PARAM_TYPE_LIGHT_STATE:
        {
            flash_light_state_t flash_light_state;

            len = sizeof(flash_light_state);
            ret = nvds_get(NVDS_TAG_LIGHT_STATE, &len, (uint8_t*)&flash_light_state);

            if (ret == 0)
            {

                light_lightness = flash_light_state.light_lightness ;
                ctl_temperature = flash_light_state.ctl_temperature ;
                ctl_delta_uv = flash_light_state.ctl_delta_uv ;

                hsl_hue = flash_light_state.hsl_hue ;
                hsl_saturation = flash_light_state.hsl_saturation ;
                hsl_lightness = flash_light_state.hsl_lightness ;

                light_mode_set(flash_light_state.light_mode);

                MESH_APP_PRINT_INFO("--1--light_lightness= %d,mode = %d\r\n", light_lightness, flash_light_state.light_mode);

            }
            else
            {

                light_lightness = 0x8000;
                ctl_temperature = 800;
                ctl_delta_uv = 0;

                hsl_hue = 0;
                hsl_saturation = 0;
                hsl_lightness = 0;

                light_mode_set(0);

                MESH_APP_PRINT_INFO("--2--light_lightness= %d,mode = %d\r\n", light_lightness, light_mode_get());

            }
        }
        break;
        default:
            ret = 1;
            break;
    }
#endif
    return ret;
}

uint32_t light_state_nv_store(flash_light_param_type_t type)
{
#if 1
    uint32_t ret;
    nvds_tag_len_t len;

    switch (type)
    {
        case FLASH_LIGHT_PARAM_TYPE_POWER_ON_COUNT:
        {
            len = sizeof(quick_onoff_count);
            ret = nvds_put(NVDS_TAG_POWER_RESET_CNT, len, (uint8_t*)&quick_onoff_count);
            MESH_APP_PRINT_INFO("---3-quick_onoff_count= %d,ret:%d\r\n", quick_onoff_count, ret);
        }
        break;


        case FLASH_LIGHT_PARAM_TYPE_LIGHT_STATE:
        {
            flash_light_state_t flash_light_state;
            len = sizeof(flash_light_state_t);

            flash_light_state.light_lightness = light_lightness;
            flash_light_state.ctl_temperature = ctl_temperature;
            flash_light_state.ctl_delta_uv =ctl_delta_uv;

            flash_light_state.hsl_hue = hsl_hue;
            flash_light_state.hsl_saturation = hsl_saturation;
            flash_light_state.hsl_lightness = hsl_lightness;

            flash_light_state.light_mode = light_mode_get();


            ret = nvds_put(NVDS_TAG_LIGHT_STATE, len, (uint8_t*)&flash_light_state);

            MESH_APP_PRINT_INFO("---3-light_lightness= %d,light_mode= %d,ret:%d\r\n", light_lightness, light_mode_get(), ret);
        }
        break;

        default:
            ret = 1;
            break;
    }

    if (ret != 0)
    {
        MESH_APP_PRINT_INFO("light_flash_store: failed, ret = %d\n", ret);
    }
#endif

    return ret;
}



