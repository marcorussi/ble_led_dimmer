/*
 * The MIT License (MIT)
 *
 * Copyright (c) [2015] [Marco Russi]
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
*/


/*
	TODO: improve fade mechanism
*/


/* ---------------- Inclusions --------------------- */
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stddef.h>
#include "nordic_common.h"
#include "nrf.h"
#include "nrf51_bitfields.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "softdevice_handler.h"
#include "app_timer.h"
#include "app_util_platform.h"
#include "app_pwm.h"

#include "config.h"
#include "memory.h"
#include "dimmer_service.h"
#include "led_strip.h"




/* ------------- Local defines --------------- */

/* Memory position: fade percentage field */
#define FADE_MEM_POSITION				(BLE_DIMMER_CONFIG_CHAR_POS + 0)	/* First location in memory data structure */

/* PWM fade period in ms */
#define PWM_FADE_PERIOD_MS				100    

/* Maximum PWM DC value */
#define PWM_DC_MAX_VALUE                100        

/* Minimum PWM DC value */                                 
#define PWM_DC_MIN_VALUE                0                                                                 		

/* PWM channel for RED */   
#define PWM_CH_R          				5

/* PWM channel for GREEN */   
#define PWM_CH_G          				6

/* PWM channel for BLUE */   
#define PWM_CH_B          				8

/* PWM channel for BLUE */   
#define PWM_CH_W         				10




/* ---------------- Local macros --------------------- */   

/* Create the instance "PWM1" using TIMER2. */
APP_PWM_INSTANCE(PWM1,1);  

/* Create the instance "PWM2" using TIMER3. */                 
APP_PWM_INSTANCE(PWM2,2);  




/* ---------------- Local variables --------------------- */   

/* A flag indicating PWM1 status. */
static volatile bool pwm1_ready_flag = false;

/* A flag indicating PWM2 status. */		
static volatile bool pwm2_ready_flag = false;	

/* Variables to store target PWM values */
static uint8_t red_pwm_target = 0;
static uint8_t green_pwm_target = 0;
static uint8_t blue_pwm_target = 0;
static uint8_t white_pwm_target = 0;

/* Variables to store current PWM values */
static uint8_t red_pwm_value = 0;
static uint8_t green_pwm_value = 0;
static uint8_t blue_pwm_value = 0;
static uint8_t white_pwm_value = 0;
static uint8_t fade_count = 0;
static uint8_t fade_percent_value;

/* Variables to store step PWM values */
static int8_t red_pwm_step = 0;
static int8_t green_pwm_step = 0;
static int8_t blue_pwm_step = 0;
static int8_t white_pwm_step = 0;




/* ------------- Local functions prototypes --------------- */

static void pwm_ready_callback(uint32_t);




/* ------------- Exported functions implementations --------------- */

/* Function to init LED light module */
void led_light_init(void)
{
	ret_code_t err_code;

	/* 2-channel PWM1, 200Hz, output on DK LED pins. */
    app_pwm_config_t pwm1_cfg = APP_PWM_DEFAULT_CONFIG_2CH(5000L, PWM_CH_R, PWM_CH_G);
    /* 2-channel PWM2, 200Hz, output on DK LED pins. */
    app_pwm_config_t pwm2_cfg = APP_PWM_DEFAULT_CONFIG_2CH(5000L, PWM_CH_B, PWM_CH_W);
    
    /* Set PWM R channel polarity */
    pwm1_cfg.pin_polarity[0] = APP_PWM_POLARITY_ACTIVE_HIGH;
    /* Set PWM G channel polarity */
    pwm1_cfg.pin_polarity[1] = APP_PWM_POLARITY_ACTIVE_HIGH;
    /* Set PWM B channel polarity */
    pwm2_cfg.pin_polarity[0] = APP_PWM_POLARITY_ACTIVE_HIGH;
	/* Set PWM W channel polarity */
    pwm2_cfg.pin_polarity[1] = APP_PWM_POLARITY_ACTIVE_HIGH;
    
    /* Initialize PWM1 */
    err_code = app_pwm_init(&PWM1, &pwm1_cfg, pwm_ready_callback);
    APP_ERROR_CHECK(err_code);
    /* Initialize PWM2 */
    err_code = app_pwm_init(&PWM2, &pwm2_cfg, pwm_ready_callback);
    APP_ERROR_CHECK(err_code);

    /* Enable PWM1 and PWM2 */
    app_pwm_enable(&PWM1);
    app_pwm_enable(&PWM2);

	/* ready to do first PWM1/2 update */
    pwm1_ready_flag = true;
    pwm2_ready_flag = true;

	/* ATTENTION: fade value is read from memory once during init */
	/* get fade value at init only */
	fade_percent_value = char_values[FADE_MEM_POSITION];
}


/* Function to turn OFF all LED channels */
void led_turn_off(void)
{
	while(false == pwm1_ready_flag);
	while(app_pwm_channel_duty_set(&PWM1, 0, 0) == NRF_ERROR_BUSY);

	while(false == pwm1_ready_flag);
	while(app_pwm_channel_duty_set(&PWM1, 1, 0) == NRF_ERROR_BUSY);

	while(false == pwm2_ready_flag);
	while(app_pwm_channel_duty_set(&PWM2, 0, 0) == NRF_ERROR_BUSY);

	while(false == pwm2_ready_flag);
	while(app_pwm_channel_duty_set(&PWM2, 1, 0) == NRF_ERROR_BUSY);
}


/* Function to update LED ligth */
void led_update_light(	uint8_t red_value, 
					  	uint8_t green_value, 
						uint8_t blue_value, 
						uint8_t white_value)
{
	/* check values */
	if((red_value <= 100)
	&& (green_value <= 100)
	&& (blue_value <= 100)
	&& (white_value <= 100))
	{
		red_pwm_target = red_value;
		green_pwm_target = green_value;
		blue_pwm_target = blue_value;
		white_pwm_target = white_value;

		/* calculate PWM steps for each channel */
		red_pwm_step   = (int8_t)(((int16_t)(red_pwm_target   - red_pwm_value)   * fade_percent_value) / 100);
		green_pwm_step = (int8_t)(((int16_t)(green_pwm_target - green_pwm_value) * fade_percent_value) / 100);
		blue_pwm_step  = (int8_t)(((int16_t)(blue_pwm_target  - blue_pwm_value)  * fade_percent_value) / 100);
		white_pwm_step = (int8_t)(((int16_t)(white_pwm_target - white_pwm_value) * fade_percent_value) / 100);

		/* re-calculate update counts for fade */
		fade_count = (uint8_t)(100 / fade_percent_value);
	}
	else
	{
		/* do nothing */
	}
}


/* Function to manage light periodically */
void led_manage_light(void)
{
	/* update PWM until fade count expires. Last run is set directly to target PWM value.
	   Indeed the target value can not be a multiple of calculated step. */
	while(fade_count > 0)
	{
		/* increment R channel PWM */
		if(fade_count == 1)
		{
			red_pwm_value = red_pwm_target;
		}
		else
		{
			red_pwm_value = (uint8_t)(red_pwm_value + red_pwm_step);
		}
		while(false == pwm1_ready_flag);
		while(app_pwm_channel_duty_set(&PWM1, 0, red_pwm_value) == NRF_ERROR_BUSY);

		/* increment G channel PWM */
		if(fade_count == 1)
		{
			green_pwm_value = green_pwm_target;
		}
		else
		{
			green_pwm_value = (uint8_t)(green_pwm_value + green_pwm_step);
		}
		while(false == pwm1_ready_flag);
		while(app_pwm_channel_duty_set(&PWM1, 1, green_pwm_value) == NRF_ERROR_BUSY);

		/* increment B channel PWM */
		if(fade_count == 1)
		{
			blue_pwm_value = blue_pwm_target;
		}
		else
		{
			blue_pwm_value = (uint8_t)(blue_pwm_value + blue_pwm_step);
		}
		while(false == pwm2_ready_flag);
		while(app_pwm_channel_duty_set(&PWM2, 0, blue_pwm_value) == NRF_ERROR_BUSY);

		/* increment W channel PWM */
		if(fade_count == 1)
		{
			white_pwm_value = white_pwm_target;
		}
		else
		{
			white_pwm_value = (uint8_t)(white_pwm_value + white_pwm_step);
		}
		while(false == pwm2_ready_flag);
		while(app_pwm_channel_duty_set(&PWM2, 1, white_pwm_value) == NRF_ERROR_BUSY);

		/* wait for next fade increment */
		nrf_delay_ms(PWM_FADE_PERIOD_MS);
		fade_count--;
	}
}




/* ------------- Local functions implementation --------------- */

/* PWM ready callback function */
static void pwm_ready_callback(uint32_t pwm_id)
{
    /* set related PWM ready flag */
    if(pwm_id == 0)
    {
        pwm1_ready_flag = true;
    }
    else if(pwm_id == 1)
    {
		pwm2_ready_flag = true;
    }
    else
    {
		/* invalid PWM index */
    }
}




/* End of file */



