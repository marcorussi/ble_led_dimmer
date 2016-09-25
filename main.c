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
#include "nrf_gpio.h"
#include "nrf51_bitfields.h"
#include "nrf_delay.h"
#include "softdevice_handler.h"
#include "app_timer.h"

#include "config.h"
#include "application.h"




/* ---------------- Local defines --------------------- */

// TODO: consider to unify timers management between all modules
/* Value of the RTC1 PRESCALER register. */
#define APP_TIMER_PRESCALER          	0
#define APP_TIMER_OP_QUEUE_SIZE         4  


/* Value used as error code on stack dump, can be used to identify stack location on stack unwind. */                                       
#define DEAD_BEEF                       0xDEADBEEF 




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

#if 1
/* Function for placing the application in low power state while waiting for events */
static void power_manage(void)
{
    uint32_t err_code = sd_app_evt_wait();
    APP_ERROR_CHECK(err_code);
}
#endif

/* Application main function */
int main(void)
{
    /* Initialize timers */
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, false);
#ifdef LED_DEBUG
	/* prototype test pin */
 	nrf_gpio_pin_dir_set(24, NRF_GPIO_PIN_DIR_OUTPUT );
	nrf_gpio_pin_write(24, 1);
#endif
	application_init();

    while(true)
    {
		application_run();

		/* manage power */
		power_manage();
    }
}




/* End of file */

