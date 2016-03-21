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

#include "ble_periph.h"
#include "ble_scanner.h"
#include "dimmer_service.h"
#include "led_strip.h"
#include "memory.h"




/* ---------------- Local defines --------------------- */

// TODO: consider to unify timers management between all modules
/* Value of the RTC1 PRESCALER register. */
#define APP_TIMER_PRESCALER          	0
#define APP_TIMER_OP_QUEUE_SIZE         4   
/* APP timer tick period in ms */  
#define APP_TIMER_TICK_PERIOD_MS		4000
/* get APP timer tick count */
#define APP_TIMER_TICK_COUNT			APP_TIMER_TICKS(APP_TIMER_TICK_PERIOD_MS, APP_TIMER_PRESCALER) 


/* Value used as error code on stack dump, can be used to identify stack location on stack unwind. */                                       
#define DEAD_BEEF                       0xDEADBEEF 


/* Default RGB channels and fade values */
#define DEF_RED_PWM_PERCENT						0x32
#define DEF_GREEN_PWM_PERCENT					0x32
#define DEF_BLUE_PWM_PERCENT					0x32
#define DEF_WHITE_PWM_PERCENT					0x32
#define DEF_FADE_PWM_PERCENT					0x32




/* -------------- Local macros ---------------- */

/* Define timer for idle timeout update */
APP_TIMER_DEF(tick_timer);    


/* Default characteristic values */
const uint8_t default_values[BLE_DIMMER_SERVICE_CHARS_LENGTH] = 
{
	DEF_RED_PWM_PERCENT,		/* Light - R */						
	DEF_GREEN_PWM_PERCENT,		/* Light - G */
	DEF_BLUE_PWM_PERCENT,		/* Light - B */
	DEF_WHITE_PWM_PERCENT,		/* Light - W */
	DEF_FADE_PWM_PERCENT,		/* Light - Fade */
	30,		/* Preset 1 - R */						
	30,		/* Preset 1 - G */
	30,		/* Preset 1 - B */
	30,		/* Preset 1 - W */
	100,		/* Preset 1 - Fade */
	95,		/* Preset 2 - R */						
	95,		/* Preset 2 - G */
	95,		/* Preset 2 - B */
	95,		/* Preset 2 - W */
	20,		/* Preset 2 - Fade */
	DEF_RED_PWM_PERCENT,		/* Preset 3 - R */						
	DEF_GREEN_PWM_PERCENT,		/* Preset 3 - G */
	DEF_BLUE_PWM_PERCENT,		/* Preset 3 - B */
	DEF_WHITE_PWM_PERCENT,		/* Preset 3 - W */
	DEF_FADE_PWM_PERCENT,		/* Preset 3 - Fade */
	DEF_RED_PWM_PERCENT,		/* Preset 4 - R */						
	DEF_GREEN_PWM_PERCENT,		/* Preset 4 - G */
	DEF_BLUE_PWM_PERCENT,		/* Preset 4 - B */
	DEF_WHITE_PWM_PERCENT,		/* Preset 4 - W */
	DEF_FADE_PWM_PERCENT,		/* Preset 4 - Fade */
	DEF_RED_PWM_PERCENT,		/* Preset 5 - R */						
	DEF_GREEN_PWM_PERCENT,		/* Preset 5 - G */
	DEF_BLUE_PWM_PERCENT,		/* Preset 5 - B */
	DEF_WHITE_PWM_PERCENT,		/* Preset 5 - W */
	DEF_FADE_PWM_PERCENT,		/* Preset 5 - Fade */
	DEF_RED_PWM_PERCENT,		/* Preset 6 - R */						
	DEF_GREEN_PWM_PERCENT,		/* Preset 6 - G */
	DEF_BLUE_PWM_PERCENT,		/* Preset 6 - B */
	DEF_WHITE_PWM_PERCENT,		/* Preset 6 - W */
	DEF_FADE_PWM_PERCENT,		/* Preset 6 - Fade */
};




/* ---------------- Local functions prototype --------------------- */   

static void tick_timer_function(void * p_context);                             




/* ---------------- Local functions implementation --------------------- */   

/* Function for assert macro callback.
   This function will be called in case of an assert in the SoftDevice.
   This handler is an example only and does not fit a final product. You need to analyse 
   how your product is supposed to react in case of Assert.
   On assert from the SoftDevice, the system can only recover on reset */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}


/* Function for placing the application in low power state while waiting for events */
static void power_manage(void)
{
    uint32_t err_code = sd_app_evt_wait();
    APP_ERROR_CHECK(err_code);
}

uint8_t data_byte = 1;
/* Tick timer timeout handler function */
static void tick_timer_function(void * p_context)
{
	UNUSED_PARAMETER(p_context);

	//TODO
	//led_turn_off();

	if(data_byte == 1)
	{
		data_byte = 2;
	}
	else
	{
		data_byte = 1;
	}

	led_update_light(data_byte);
}


/* Application main function */
int main(void)
{
	//uint32_t err_code;

    /* Initialize timers */
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, false);

/*
	nrf_gpio_pin_dir_set(23, NRF_GPIO_PIN_DIR_OUTPUT);
	nrf_gpio_pin_dir_set(24, NRF_GPIO_PIN_DIR_OUTPUT);
	nrf_gpio_pin_write(23, 1);
	nrf_gpio_pin_write(24, 1);
*/
	/* init NUS peripheral connection */
	//ble_periph_init();

	/* init connection */
	conn_init();

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
	//ble_periph_adv_start();

	/* start scan */
	conn_start_scan();

	/* init tick timer (for managing timeouts) */
	//err_code = app_timer_create(&tick_timer, APP_TIMER_MODE_REPEATED, &tick_timer_function);
	//APP_ERROR_CHECK(err_code);

	/* start tick timer */
	//err_code = app_timer_start(tick_timer, APP_TIMER_TICK_COUNT, NULL);
	//APP_ERROR_CHECK(err_code);

    while(true)
    {
		led_manage_light();

		/* manage power */
		//power_manage();
    }
}




/* End of file */

