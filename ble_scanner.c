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


/* ---------- Inclusions ---------- */

/* Compiler libraries */
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
/* Nordic common library */
#include "nordic_common.h"

/* softdevice handler */
#include "softdevice_handler.h"

/* nrf drivers */
#include "nrf.h"
#include "nrf_gpio.h"

/* BLE components */
#include "ble.h"
#include "ble_gap.h"
#include "ble_hci.h"
#include "ble_advdata.h"

/* APP components */
#include "app_error.h"
#include "app_uart.h"
#include "app_timer.h"

/* Other components */
#include "dimmer_service.h"
#include "led_strip.h"

/* header file */
#include "ble_scanner.h"




/* ---------- Local definitions ---------- */

/* Adv fixed fields values */
#define ADV_FLAGS_TYPE					BLE_GAP_AD_TYPE_FLAGS
#define BR_EDR_NOT_SUPPORTED			BLE_GAP_ADV_FLAG_BR_EDR_NOT_SUPPORTED
#define MANUF_DATA_TYPE					BLE_GAP_AD_TYPE_MANUFACTURER_SPECIFIC_DATA
#define MANUFACTURER_ID					0x0059
#define MANUF_DATA_LENGTH				19
#define MANUF_PRODUCT_ID				0x0202	
#define MANUF_SERVICE_ID				0x0110	
                                                                
/* Scanning parameters */    
/* Determines scan interval in units of 0.625 millisecond */                                                        
#define SCAN_INTERVAL           		0x0640//0x00A0    
 /* Determines scan window in units of 0.625 millisecond */                      
#define SCAN_WINDOW             		0x01e0//0x0050     
/* If 1, performe active scanning (scan requests) */                    
#define SCAN_ACTIVE             		1 
/* If 1, ignore unknown devices (non whitelisted) */                              
#define SCAN_SELECTIVE          		0 
/* Scan timeout. 0 means Disabled */                              
#define SCAN_TIMEOUT            		0x0000                          




/* ---------- Local typedefs ---------- */

/* Adv packet format to scan */
typedef enum
{
	FIRST_LENGTH_POS,					/* first length */
	ADV_TYPE_FLAGS_POS,					/* adv flags type */
	BR_EDR_NOT_SUPPORTED_POS,			/* BR/EDR not supported */
	SECOND_LENGTH_POS,					/* second length */
	MANUF_DATA_TYPE_POS,				/* manufacturer data type */
	MANUF_ID_BYTE_0_POS,				/* manufacturer ID lower byte */
	MANUF_ID_BYTE_1_POS,				/* manufacturer ID higher byte */
	MANUF_DATA_LENGTH_POS,				/* data length */
	PRODUCT_ID_BYTE_0_POS,				/* product type ID lower byte */
	PRODUCT_ID_BYTE_1_POS,				/* product type ID higher byte */
	SERVICE_UUID_BYTE_0_POS,			/* service ID lower byte */
	SERVICE_UUID_BYTE_1_POS,			/* service ID higher byte */
	DEVICE_ID_BYTE_0_POS,				/* device ID byte 0 */
	DEVICE_ID_BYTE_1_POS,				/* device ID byte 1 */
	DEVICE_ID_BYTE_2_POS,				/* device ID byte 2 */
	DEVICE_ID_BYTE_3_POS,				/* device ID byte 3 */
	DATA_BYTE_0_POS,					/* data byte 0 */
	DATA_BYTE_1_POS,					/* data byte 1 */
	DATA_BYTE_2_POS,					/* data byte 2 */
	DATA_BYTE_3_POS,					/* data byte 3 */
	DATA_BYTE_4_POS,					/* data byte 4 */
	DATA_BYTE_5_POS,					/* data byte 5 */
	DATA_BYTE_6_POS,					/* data byte 6 */
	DATA_BYTE_7_POS,					/* data byte 7 */
	RESERVED_0_POS,						/* reserved 0 */
	RESERVED_1_POS,						/* reserved 1 */
	CALIB_RSSI_POS,						/* calibrated RSSI */
	ADV_PACKET_LENGTH					/* Adv packet length. This is not included. It is for fw purpose only */
} adv_packet_form_e;




/* ---------- Local const variables ---------- */

/* Preamble of the Adv packet. This string represent a fixed part of the adv packet */
static const uint8_t preamble_adv[DEVICE_ID_BYTE_0_POS] = 
{
	0x02,								/* first length */
	ADV_FLAGS_TYPE,						/* adv flags type */
	BR_EDR_NOT_SUPPORTED,				/* BR/EDR not supported */
	(uint8_t)(MANUF_DATA_LENGTH + 4),	/* second length */
	MANUF_DATA_TYPE,					/* manufacturer data type */
	(uint8_t)MANUFACTURER_ID,			/* manufacturer ID lower byte */
	(uint8_t)(MANUFACTURER_ID >> 8),	/* manufacturer ID higher byte */
	MANUF_DATA_LENGTH,					/* manufacturer specific data length */
	(uint8_t)MANUF_PRODUCT_ID,			/* product type ID lower byte */
	(uint8_t)(MANUF_PRODUCT_ID >> 8),	/* product type ID higher byte */
	(uint8_t)MANUF_SERVICE_ID,			/* service UUID lower byte */
	(uint8_t)(MANUF_SERVICE_ID >> 8)	/* service UUID higher byte */
};


/* Parameters used when scanning. */
static const ble_gap_scan_params_t m_scan_params = 
{
	.active      = SCAN_ACTIVE,
	.selective   = SCAN_SELECTIVE,
	.p_whitelist = NULL,
	.interval    = SCAN_INTERVAL,
	.window      = SCAN_WINDOW,
	.timeout     = SCAN_TIMEOUT
};




/* ---------- Local variables ---------- */

/* Store the last received data from adv packet */
static uint8_t last_data_byte = 0xFF;




/* ---------- Local functions prototypes ---------- */

static void get_advertising_fields(uint8_t *, uint8_t);
static void on_ble_evt(ble_evt_t *);
static void ble_stack_init(void);




/* ------------ Exported functions implementation --------------- */

/* Function to init the connection manager */
void conn_init(void)
{
	/* init BLE stack */
	ble_stack_init();
}


/* Function to start scanning devices */
void conn_start_scan(void)
{
	uint32_t err_code;

    err_code = sd_ble_gap_scan_start(&m_scan_params);
    APP_ERROR_CHECK(err_code);
}


/* Function to start scanning devices */
void conn_stop_scan(void)
{
	/* stop scanning */
	sd_ble_gap_scan_stop();
}




/* ---------- Local functions implementation ---------- */


/* Function to get advertising fields */
static void get_advertising_fields(uint8_t *p_data, uint8_t data_length)
{
	/* consider only packets with a specific expected length */
	if(data_length == ADV_PACKET_LENGTH)
	{
		/* if adv preamble is as expected */
		if(0 == memcmp(p_data, &preamble_adv, DEVICE_ID_BYTE_0_POS))
		{
			/* preable is valid. Device found */
			/* ATTENTION: device ID is not considered at the moment */
			/* get interesting data */
			uint8_t data_byte = p_data[DATA_BYTE_0_POS];
			/* ATTENTION: everything else is not considered at the moment */
	
			/* if data byte is different than last one and within the vald range */
			if((data_byte <= BLE_PRESET_NUM_OF_VALUES)
			&& (data_byte != last_data_byte))
			{
				/* store last data byte */
				last_data_byte = data_byte;

				/* update light according to required preset index */
				led_update_light(last_data_byte);
			}
			else
			{
				/* invalid preset index: do nothing */
			}			
		}
		else
		{
			/* wrong preamble. discard it */
		}
	}
	else
	{
		/* discard it */
	}
}


/* Function for handling the Application's BLE Stack events.
   Parameters: p_ble_evt   Bluetooth stack event.
*/
static void on_ble_evt(ble_evt_t * p_ble_evt)
{
    const ble_gap_evt_t * p_gap_evt = &p_ble_evt->evt.gap_evt;	
    
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_ADV_REPORT:
        {
            const ble_gap_evt_adv_report_t *p_adv_report = &p_gap_evt->params.adv_report;
			
			//TODO: consider to check a specific addess (p_adv_report->peer_addr)
			
			/* if advertising packet and no scan response */
			if(p_adv_report->scan_rsp == 0)
 			{
				/* get advertising fields */
				get_advertising_fields((uint8_t *)p_adv_report->data, (uint8_t)p_adv_report->dlen);
			}
			else
			{
				/* do not consider scan responses: do nothing */
			}
            break;
        }
        case BLE_GAP_EVT_CONNECTED:
		{
			/* it should not pass here */
            break;
		}
		case BLE_GAP_EVT_DISCONNECTED:
		{
			/* it should not pass here */
            break;
    	}
        case BLE_GAP_EVT_TIMEOUT:
		{
            if (p_gap_evt->params.timeout.src == BLE_GAP_TIMEOUT_SRC_SCAN)
            {
                /* scan timed out. it should not pass here since timeout is disabled */
            }
            else if (p_gap_evt->params.timeout.src == BLE_GAP_TIMEOUT_SRC_CONN)
            {
                /* connection request timed out: it should not pass here */
            }
			else
			{
				/* do nothing */
			}
            break;
        }
        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
		{
            /* it should not pass here */
            break;
    	}
        case BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST:
		{
            /* it should not pass here */
            break;
		}
        default:
            break;
    }
}


/* Function for initializing the BLE stack.
   Initializes the SoftDevice and the BLE event interrupt.
*/
static void ble_stack_init(void)
{
    uint32_t err_code;

    /* Initialize the SoftDevice handler module. */
    SOFTDEVICE_HANDLER_INIT(NRF_CLOCK_LFCLKSRC_RC_250_PPM_TEMP_4000MS_CALIBRATION, NULL);

    /* Enable BLE stack. */
    ble_enable_params_t ble_enable_params;
    memset(&ble_enable_params, 0, sizeof(ble_enable_params));

    ble_enable_params.gatts_enable_params.attr_tab_size   = BLE_GATTS_ATTR_TAB_SIZE_DEFAULT;
	ble_enable_params.gatts_enable_params.service_changed = false;

	/* enable BLE */
    err_code = sd_ble_enable(&ble_enable_params);
    APP_ERROR_CHECK(err_code);

	/* Register with the SoftDevice handler module for BLE events in central role */
	err_code = softdevice_ble_evt_handler_set(on_ble_evt);
	APP_ERROR_CHECK(err_code);
}




/* End of file */


