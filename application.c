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
	ATTENTION:
	LIGHT characteristic values are stored and have default values but are not used.
   	They are only read upon reception of new values. Then they are stored again just for logging. 
*/




/* ------------- Inclusions --------------- */

#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include "nordic_common.h"
#include "nrf.h"
#include "nrf51_bitfields.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "softdevice_handler.h"
#include "app_timer.h"

#include "ble_manager.h"
#include "dimmer_service.h"
#include "led_strip.h"
#include "memory.h"

#include "application.h"




/* ---------------- Local defines --------------------- */

// TODO: consider to unify timers management between all modules
/* Value of the RTC1 PRESCALER register. */
#define APP_TIMER_PRESCALER          			0
#define APP_TIMER_OP_QUEUE_SIZE         		4   
/* APP timer tick period in ms */  
#define APP_TIMER_TICK_PERIOD_MS				10000
/* get APP timer tick count */
#define APP_TIMER_TICK_COUNT					APP_TIMER_TICKS(APP_TIMER_TICK_PERIOD_MS, APP_TIMER_PRESCALER) 


/* Default RGB channels and fade values */
#define DEF_RED_PWM_PERCENT						100		/* 100% */
#define DEF_GREEN_PWM_PERCENT					100		/* 100% */
#define DEF_BLUE_PWM_PERCENT					100		/* 100% */
#define DEF_WHITE_PWM_PERCENT					100		/* 100% */
#define DEF_FADE_PWM_PERCENT					50		/* 50% */




/* -------------- Local macros ---------------- */

/* Define timer for idle timeout update */
APP_TIMER_DEF(tick_timer);    




/* -------------- Local variables ---------------- */

/* Default characteristic values */
const uint8_t default_values[BLE_DIMMER_SERVICE_CHARS_LENGTH] = 
{
	DEF_RED_PWM_PERCENT,		/* Light - R */						
	DEF_GREEN_PWM_PERCENT,		/* Light - G */
	DEF_BLUE_PWM_PERCENT,		/* Light - B */
	DEF_WHITE_PWM_PERCENT,		/* Light - W */
	DEF_FADE_PWM_PERCENT,		/* Light - Fade */
	20,		/* Preset 1 - R */						
	0,		/* Preset 1 - G */
	0,		/* Preset 1 - B */
	0,		/* Preset 1 - W */
	100,	/* Preset 1 - Fade */
	0,		/* Preset 2 - R */						
	60,		/* Preset 2 - G */
	0,		/* Preset 2 - B */
	0,		/* Preset 2 - W */
	30,		/* Preset 2 - Fade */
	0,		/* Preset 3 - R */						
	0,		/* Preset 3 - G */
	95,		/* Preset 3 - B */
	0,		/* Preset 3 - W */
	10,		/* Preset 3 - Fade */
	50,		/* Preset 4 - R */						
	50,		/* Preset 4 - G */
	0,		/* Preset 4 - B */
	0,		/* Preset 4 - W */
	30,		/* Preset 4 - Fade */
	0,		/* Preset 5 - R */						
	0,		/* Preset 5 - G */
	0,		/* Preset 5 - B */
	100,	/* Preset 5 - W */
	90,		/* Preset 5 - Fade */
	0,		/* Preset 6 - R */						
	0,		/* Preset 6 - G */
	0,		/* Preset 6 - B */
	0,		/* Preset 6 - W */
	70,		/* Preset 6 - Fade */
};




/* ---------------- Local functions prototype --------------------- */   

static void tick_timer_function(void * p_context);          




/* ---------------- Local functions --------------------- */   

/* Tick timer timeout handler function */
static void tick_timer_function(void * p_context)
{
	UNUSED_PARAMETER(p_context);

	/* stop advertising */
	ble_man_adv_stop();

	/* start scanning */
	ble_man_scan_start();
}




/* ---------------- Exported functions --------------------- */   

/* callback on connection event */
void application_on_conn( void )
{
	uint32_t err_code;

	/* stop tick timer */
	err_code = app_timer_stop(tick_timer);
	APP_ERROR_CHECK(err_code);
}


/* callback on disconnection event */
void application_on_disconn( void )
{
	uint32_t err_code;

	/* start avertising */
	ble_man_adv_start();

	/* start tick timer */
	err_code = app_timer_start(tick_timer, APP_TIMER_TICK_COUNT, NULL);
	APP_ERROR_CHECK(err_code);
}


/* init application */
void application_init( void )
{
	uint32_t err_code;

	/* init NUS peripheral connection */
	ble_man_init();

	/* if persistent memory is initialised successfully */
	if(true == memory_init(default_values))
	{
		/* wait for completion */
		while(false != memory_is_busy());
	}
	else
	{
		/* very bad, use default setting as recovery */
	}

	/* init LED module */
	led_light_init();

	/* start avertising */
	ble_man_adv_start();

	/* init tick timer (for managing timeouts) */
	err_code = app_timer_create(&tick_timer, APP_TIMER_MODE_SINGLE_SHOT, &tick_timer_function);
	APP_ERROR_CHECK(err_code);

	/* start tick timer */
	err_code = app_timer_start(tick_timer, APP_TIMER_TICK_COUNT, NULL);
	APP_ERROR_CHECK(err_code);
}


/* main application loop */
void application_run( void )
{
	/* manage light */
	led_manage_light();
}




/* End of file */






