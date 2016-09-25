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
#include "bootloader.h"

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

/* Password for starting DFU Upgrade on char write */
#define DFU_UPGRADE_CHAR_PASSWORD				0xA9

/* Default fade percentage value */
#define DEF_FADE_PWM_PERCENT					50		/* 50% */




/* -------------- Local macros ---------------- */

/* Macro to set spacial value on GPREGRET register to start bootloader after reset */
#define SET_REG_VALUE_TO_START_BOOTLOADER()  		(NRF_POWER->GPREGRET = BOOTLOADER_DFU_START)

/* Define timer for idle timeout update */
APP_TIMER_DEF(tick_timer);    




/* -------------- Local variables ---------------- */

/* Default characteristic values */
const uint8_t default_values[BLE_DIMMER_SERVICE_CHARS_LENGTH] = 
{
	DEF_FADE_PWM_PERCENT,		/* Light - Fade */
	0xFF,
	0xFF,
	0xFF
};




/* ---------------- Local functions prototype --------------------- */   

static void tick_timer_function(void * p_context);          




/* ---------------- Local functions --------------------- */   

/* ATTENTION: consider to start scanning from adv timeout instead of using this timer */
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

/* callback on SPECIAL_OP characteristic write */
void app_on_special_op( uint8_t special_op_byte )
{
	/* if received data is the password for DFU Upgrade */
	if(special_op_byte == DFU_UPGRADE_CHAR_PASSWORD)
	{
		/* set special register value to start bootloader */
		SET_REG_VALUE_TO_START_BOOTLOADER();

		/* perform a system reset */
		NVIC_SystemReset();
	}
	else
	{
		/* do nothing */
	}
}


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

	/* init peripheral connection */
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






