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


/* ------------- Inclusions --------------- */

#include "ble.h"
#include "ble_srv_common.h"
#include <stdint.h>
#include <stdbool.h>




/* ------------- Exported definitions --------------- */

/* The UUID of the Nordic UART Service. */
#define BLE_UUID_DIMMER_SERVICE 					0x0001		

/* UUID type for the DIMMER Service (vendor specific) */ 
#define DIMMER_SERVICE_UUID_TYPE					BLE_UUID_TYPE_VENDOR_BEGIN  
  
/* CONFIG Characteristic value length in bytes */
#define BLE_DIMMER_CONFIG_CHAR_LENGTH				8

/* Length of SPECIAL_OP characteristic in bytes */
#define BLE_DIMMER_SPECIAL_OP_CHAR_LENGTH			1	

/* Total characteristics length in bytes */
#define BLE_DIMMER_SERVICE_CHARS_LENGTH 			(BLE_DIMMER_CONFIG_CHAR_LENGTH + BLE_DIMMER_SPECIAL_OP_CHAR_LENGTH)

/* CONFIG Characteristic value position in bytes */
#define BLE_DIMMER_CONFIG_CHAR_POS					0

/* SPECIAL_OP Characteristic value position in bytes */
#define BLE_DIMMER_SPECIAL_OP_CHAR_POS				(BLE_DIMMER_CONFIG_CHAR_POS + BLE_DIMMER_CONFIG_CHAR_LENGTH)




/* ------------- Exported structures --------------- */

/* Forward declaration of the ble_dimmer_st type */
typedef struct ble_dimmer_s ble_dimmer_st;

/* DIMMER Service event handler type */
typedef void (*ble_dimmer_data_handler_st) (ble_dimmer_st * p_dimmer);

/* DIMMER Service initialization structure.
   This structure contains the initialization information for the service. The application
   must fill this structure and pass it to the service using the ble_dimmer_init() function */
typedef struct
{
    ble_dimmer_data_handler_st data_handler;	/* Event handler to be called for handling received data. */
} ble_dimmer_init_st;


/* DIMMER Service structure.
   This structure contains status information related to the service */
struct ble_dimmer_s
{
    uint8_t                  	uuid_type;          	/* UUID type for DIMMER Service Base UUID. */
    uint16_t                 	service_handle;			/* Handle of DIMMER Service. */
	ble_gatts_char_handles_t	cfg_charac_handles;		/* Handle for storing CONFIG values */
	ble_gatts_char_handles_t	special_op_charac_handles;		/* Handle for rebooting into DFU Upgrade */
    uint16_t                 	conn_handle;			/* Handle of the current connection. BLE_CONN_HANDLE_INVALID if not in a connection. */
    ble_dimmer_data_handler_st	data_handler;			/* Event handler to be called for handling received data. */
};




/* ------------- Exported variables --------------- */

/* Store characteristic values */
extern uint8_t char_values[BLE_DIMMER_SERVICE_CHARS_LENGTH];




/* ------------- Exported functions prototypes --------------- */

/* Function for initializing the DIMMER Service */
extern uint32_t ble_dimmer_init(ble_dimmer_st *, const ble_dimmer_init_st *);


/* Function for handling the DIMMER Service's BLE events.
 * The DIMMER Service expects the application to call this function each time an
 * event is received from the SoftDevice. This function processes the event if it
 * is relevant and calls the DIMMER Service event handler of the
 * application if necessary */
extern void ble_dimmer_on_ble_evt(ble_dimmer_st *, ble_evt_t *);




/* End of file */


