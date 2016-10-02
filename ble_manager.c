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


/* ----------------------- Inclusions ------------------------ */

#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "nrf_drv_timer.h"
#include "app_error.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_dis.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_conn_params.h"
#include "softdevice_handler.h"
#include "app_timer.h"
#include "pstorage.h"
#include "app_trace.h"
#include "app_util_platform.h"
#include "dfu_init.h"

#include "config.h"
#include "ble_manager.h"
#include "dimmer_service.h"
#include "led_strip.h"
#include "application.h"




/* ----------------------- Local defines ------------------------ */ 

/* Low frequency clock source to be used by the SoftDevice */
#define NRF_CLOCK_LFCLKSRC      			{.source        = NRF_CLOCK_LF_SRC_RC,				\
                                 		 	 .rc_ctiv       = 15,                              	\
                                 		 	 .rc_temp_ctiv  = 15,								\
                                 		 	 .xtal_accuracy = 0}

/* Number of central links used by the application. When changing this number remember to adjust the RAM settings */
#define CENTRAL_LINK_COUNT              	1   

/* Number of peripheral links used by the application. When changing this number remember to adjust the RAM settings */
#define PERIPHERAL_LINK_COUNT            	1                                          

/* The advertising interval (in units of 0.625 ms. This value corresponds to 25 ms) */
#define APP_ADV_INTERVAL                 	100 

/* The advertising timeout (in units of seconds). */
#define APP_ADV_TIMEOUT_IN_SECONDS      	ADV_TIMEOUT_TO_START_SCAN_S                                        

//TODO: consider to unify these two defines
/* Value of the RTC1 PRESCALER register */
#define APP_TIMER_PRESCALER              	0                                                                                                               

/* Minimum acceptable connection interval (0.1 seconds) */
#define MIN_CONN_INTERVAL                	MSEC_TO_UNITS(100, UNIT_1_25_MS)           

/* Maximum acceptable connection interval (0.2 second) */
#define MAX_CONN_INTERVAL                	MSEC_TO_UNITS(200, UNIT_1_25_MS)           

/* Slave latency */
#define SLAVE_LATENCY                    	0                                          

/* Connection supervisory timeout (4 seconds) */
#define CONN_SUP_TIMEOUT                 	MSEC_TO_UNITS(4000, UNIT_10_MS)            

/* Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds) */
#define FIRST_CONN_PARAMS_UPDATE_DELAY   	APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER) 

/* Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds) */
#define NEXT_CONN_PARAMS_UPDATE_DELAY    	APP_TIMER_TICKS(30000, APP_TIMER_PRESCALER)

/* Number of attempts before giving up the connection parameter negotiation */
#define MAX_CONN_PARAMS_UPDATE_COUNT     	3                                                                                                       

/* TX Power Level value. This will be set both in the TX Power service, in the advertising data, and also used to set the radio transmit power */
#define TX_POWER_LEVEL                     	TX_POWER_MEASURED_RSSI                                                                    

/* Adv fixed fields values to scan and match */
#define ADV_FLAGS_TYPE						BLE_GAP_AD_TYPE_FLAGS
#define BR_EDR_NOT_SUPPORTED				BLE_GAP_ADV_FLAG_BR_EDR_NOT_SUPPORTED
#define MANUF_DATA_TYPE						BLE_GAP_AD_TYPE_MANUFACTURER_SPECIFIC_DATA
#define MANUFACTURER_ID						TEMP_COMPANY_ID
#define MANUF_DATA_LENGTH					11
#define MANUF_SERVICE_ID					0x0110	

/* Scanning parameters */    
/* TODO: consider to move the first 2 following defines to config.h */
/* Determines scan interval in units of 0.625 millisecond */                                                        
#define SCAN_INTERVAL           			1600	/* 1000 ms */
 /* Determines scan window in units of 0.625 millisecond */                      
#define SCAN_WINDOW             			800		/* 500 ms */
/* If 1, performe active scanning (scan requests) */                    
#define SCAN_ACTIVE             			1 
/* If 1, ignore unknown devices (non whitelisted) */                              
#define SCAN_SELECTIVE          			0 
/* Scan timeout. 0 means Disabled */                              
#define SCAN_TIMEOUT            			0x0000     




/* ----------------------- Local typedefs ---------------------- */

/* Device serial number integer type */
typedef uint32_t serial_num_int;

/* Device serial number string type */
typedef char serial_num_string[12];

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
	SERVICE_ID_BYTE_0_POS,				/* service ID lower byte */
	SERVICE_ID_BYTE_1_POS,				/* service ID higher byte */
	DATA_BYTE_0_POS,					/* data byte 0 */
	DATA_BYTE_1_POS,					/* data byte 1 */
	DATA_BYTE_2_POS,					/* data byte 2 */
	DATA_BYTE_3_POS,					/* data byte 3 */
	DATA_BYTE_4_POS,					/* data byte 4 */
	DATA_BYTE_5_POS,					/* data byte 5 */
	DATA_BYTE_6_POS,					/* data byte 6 */
	DATA_BYTE_7_POS,					/* data byte 7 */
	CALIB_RSSI_POS,						/* calibrated RSSI */
	ADV_DATA_PACKET_LENGTH				/* Adv packet length. This is not included. It is for fw purpose only */
} adv_data_packet_e;




/* ----------------------- Local variables ---------------------- */

/* Store the last received data flag from adv packet */
static uint8_t last_data_flag = 0xFF;

/* Structure to store advertising parameters */
static ble_gap_adv_params_t adv_params;

/* Structure to identify the DIMMER Service */
static ble_dimmer_st m_dimmer;                                                                             

/* Handle of the current connection. */
static uint16_t m_conn_handle = BLE_CONN_HANDLE_INVALID;   

/* BLE UUID fields */
static ble_uuid_t adv_uuids[] =
{
    {BLE_UUID_DIMMER_SERVICE, DIMMER_SERVICE_UUID_TYPE},
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

/* Preamble of the Adv packet. This string represent a fixed part of the adv packet */
static const uint8_t preamble_adv[DATA_BYTE_0_POS] = 
{
	0x02,								/* first length */
	ADV_FLAGS_TYPE,						/* adv flags type */
	BR_EDR_NOT_SUPPORTED,				/* BR/EDR not supported */
	(uint8_t)(MANUF_DATA_LENGTH + 4),	/* second length */
	MANUF_DATA_TYPE,					/* manufacturer data type */
	(uint8_t)MANUFACTURER_ID,			/* manufacturer ID lower byte */
	(uint8_t)(MANUFACTURER_ID >> 8),	/* manufacturer ID higher byte */
	MANUF_DATA_LENGTH,					/* manufacturer specific data length */
	(uint8_t)MANUF_SERVICE_ID,			/* service ID lower byte */
	(uint8_t)(MANUF_SERVICE_ID >> 8)	/* service ID higher byte */
};




/* ---------------------- Local macros ----------------------- */

/* Define a pointer type to the device serial number stored in the UICR */
#define UICR_DEVICE_SERIAL_NUM			(*((serial_num_int *)(NRF_UICR_BASE + UICR_CUSTOMER_RESERVED_OFFSET)))




/* ---------------------- Local functions prototypes ----------------------- */

static void dimmer_data_handler(ble_dimmer_st *);
static void gap_params_init(void);
static void services_init(void);
static void on_conn_params_evt(ble_conn_params_evt_t *);
static void conn_params_error_handler(uint32_t);
static void conn_params_init(void);
static void get_advertising_fields(uint8_t *, uint8_t);
static void on_ble_evt(ble_evt_t *);
static void ble_evt_dispatch(ble_evt_t *);
static void ble_stack_init(void);
static void ble_periph_adv_set_data(void);




/* ---------------------- Local functions ----------------------- */

/* dimmer data handler function */
static void dimmer_data_handler(ble_dimmer_st * p_dimmer)
{
	UNUSED_PARAMETER(p_dimmer);

	/* do nothing */
}


/* Function for the GAP initialization.
   This function sets up all the necessary GAP (Generic Access Profile) parameters of the
   device including the device name, appearance, and the preferred connection parameters. */
static void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

	/* set device name in GAP layer */
    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    /* TODO: consider to change the appearance */
    err_code = sd_ble_gap_appearance_set(BLE_APPEARANCE_GENERIC_REMOTE_CONTROL);
    APP_ERROR_CHECK(err_code); 

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}


/* Function for initializing services that will be used by the application */
static void services_init(void)
{
	uint32_t err_code;
	ble_dis_init_t dis_init_obj;
	ble_dimmer_init_st dimmer_init;

	/* init Device Information Service */
    memset(&dis_init_obj, 0, sizeof(dis_init_obj));
	/* set device information fields */
	serial_num_string serial_num_ascii;
	ble_srv_ascii_to_utf8(&dis_init_obj.hw_rev_str, HW_REVISION);
	ble_srv_ascii_to_utf8(&dis_init_obj.fw_rev_str, FW_REVISION);
    ble_srv_ascii_to_utf8(&dis_init_obj.manufact_name_str, MANUFACTURER_NAME);
	/* set serial number from the UICR */
	sprintf(serial_num_ascii, "%d", (unsigned int)UICR_DEVICE_SERIAL_NUM);
	ble_srv_ascii_to_utf8(&dis_init_obj.serial_num_str, serial_num_ascii);
/*
	ble_srv_utf8_str_t 	manufact_name_str
	ble_srv_utf8_str_t 	model_num_str
	ble_srv_utf8_str_t 	serial_num_str
	ble_srv_utf8_str_t 	hw_rev_str
	ble_srv_utf8_str_t 	fw_rev_str
	ble_srv_utf8_str_t 	sw_rev_str
	ble_dis_sys_id_t * 	p_sys_id
	ble_dis_reg_cert_data_list_t * 	p_reg_cert_data_list
	ble_dis_pnp_id_t * 	p_pnp_id
	ble_srv_security_mode_t 	dis_attr_md
*/
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&dis_init_obj.dis_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&dis_init_obj.dis_attr_md.write_perm);
	/* init Device Info service */
    err_code = ble_dis_init(&dis_init_obj);
    APP_ERROR_CHECK(err_code);

    /* init DIMMER service structure */
    memset(&dimmer_init, 0, sizeof(dimmer_init));
	/* set DIMMER data handler */
    dimmer_init.data_handler = dimmer_data_handler;
    /* init DIMMER service */
    err_code = ble_dimmer_init(&m_dimmer, &dimmer_init);
    APP_ERROR_CHECK(err_code);
}


/* Function for handling the Connection Parameters Module.
   This function will be called for all events in the Connection Parameters Module which
   are passed to the application.
   All this function does is to disconnect. This could have been done by simply
   setting the disconnect_on_fail config parameter, but instead we use the event
   handler mechanism to demonstrate its use.
   Parameters:
   - p_evt: Event received from the Connection Parameters Module. */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    uint32_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}


/* Function for handling a Connection Parameters error.
   Parameters:
   - nrf_error: Error code containing information about what went wrong. */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/* Function for initializing the Connection Parameters module. */
static void conn_params_init(void)
{
    uint32_t err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}


/* Function to get advertising fields */
static void get_advertising_fields(uint8_t *p_data, uint8_t data_length)
{
	/* consider only packets with a specific expected length */
	if(data_length == ADV_DATA_PACKET_LENGTH)
	{
		/* if adv preamble is as expected */
		if(0 == memcmp(p_data, &preamble_adv, DATA_BYTE_0_POS))
		{
			/* preamble is valid. Device found */
			/* get interesting data */
			uint8_t data_flag = p_data[DATA_BYTE_2_POS];
			/* ATTENTION: everything else is not considered at the moment */
	
			/* if data flag is different than last one */
			if(data_flag != last_data_flag)
			{
				/* store last data byte */
				last_data_flag = data_flag;

				/* update light according to required preset index */
				led_update_light(	p_data[DATA_BYTE_3_POS],
									p_data[DATA_BYTE_4_POS],
									p_data[DATA_BYTE_5_POS],
									p_data[DATA_BYTE_6_POS]);
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
   Parameters:
   - p_ble_evt: Bluetooth stack event. */
static void on_ble_evt(ble_evt_t * p_ble_evt)
{
	uint32_t err_code;
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
		case BLE_GAP_EVT_TIMEOUT:
		{
            if (p_gap_evt->params.timeout.src == BLE_GAP_TIMEOUT_SRC_ADVERTISING)
            {
				/* inform application */
				app_on_adv_timeout();
			}
			else if (p_gap_evt->params.timeout.src == BLE_GAP_TIMEOUT_SRC_SCAN)
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
		}
        case BLE_GAP_EVT_CONNECTED:
		{
			/* store connection handle */
            m_conn_handle = p_gap_evt->conn_handle;

			/* application callback */
			application_on_conn();
            break;
		}
        case BLE_GAP_EVT_DISCONNECTED:
		{
			/* reset connection handle */
            m_conn_handle = BLE_CONN_HANDLE_INVALID;

			/* application callback */
			application_on_disconn();
            break;
		}
		case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
		{
            /* Pairing not supported */
            err_code = sd_ble_gap_sec_params_reply(m_conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
            APP_ERROR_CHECK(err_code);
            break;
		}
        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
		{
            /* No system attributes have been stored */
            err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
            APP_ERROR_CHECK(err_code);
            break;
		}
		case BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST:
		{
            /* it should not pass here */
            break;
		}
		case BLE_GATTC_EVT_TIMEOUT:	/* this case should not be managed */
        case BLE_GATTS_EVT_TIMEOUT:
		{
            /* Disconnect on GATT Server and Client timeout events. */
            err_code = sd_ble_gap_disconnect(m_conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;
		}
        default:
		{
            /* No implementation needed. */
            break;
		}
    }
}


/* Function for dispatching a BLE stack event to all modules with a BLE stack event handler.
   This function is called from the BLE Stack event interrupt handler after a BLE stack
   event has been received.
   Parameter:
   - p_ble_evt:  Bluetooth stack event. */
static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
    ble_conn_params_on_ble_evt(p_ble_evt);
	ble_dimmer_on_ble_evt(&m_dimmer, p_ble_evt);  
	on_ble_evt(p_ble_evt);
}


/* Function for initializing the BLE stack.
   Initializes the SoftDevice and the BLE event interrupt. */
static void ble_stack_init(void)
{
    uint32_t err_code;
    
    nrf_clock_lf_cfg_t clock_lf_cfg = NRF_CLOCK_LFCLKSRC;
    
    /* Initialize the SoftDevice handler module. */
    SOFTDEVICE_HANDLER_INIT(&clock_lf_cfg, NULL);

    ble_enable_params_t ble_enable_params;
    err_code = softdevice_enable_get_default_config(CENTRAL_LINK_COUNT,
                                                    PERIPHERAL_LINK_COUNT,
                                                    &ble_enable_params);
    APP_ERROR_CHECK(err_code);
    
    /* Check the ram settings against the used number of links */
    CHECK_RAM_START_ADDR(CENTRAL_LINK_COUNT,PERIPHERAL_LINK_COUNT);
    
    /* Enable BLE stack. */
    err_code = softdevice_enable(&ble_enable_params);
    APP_ERROR_CHECK(err_code);

    /* Register with the SoftDevice handler module for BLE events. */
    err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
    APP_ERROR_CHECK(err_code);
}


/* Function to set advertisement data */
static void ble_periph_adv_set_data(void)
{
	uint32_t      				err_code;
    int8_t        				tx_power_level = TX_POWER_LEVEL;
	static ble_advdata_t 		ble_adv_data;
	static ble_advdata_t 		ble_scan_resp;

    /* clear and set advertising data */
    memset(&ble_adv_data, 0, sizeof(ble_adv_data));
	/* set name type, appearance, flags and TX power */
    ble_adv_data.name_type               = BLE_ADVDATA_FULL_NAME;
    ble_adv_data.include_appearance      = true;
    ble_adv_data.flags                   = BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE;
    ble_adv_data.p_tx_power_level        = &tx_power_level;
	/* NO manufacturer specific data structure */
	ble_adv_data.p_manuf_specific_data = NULL;

	/* clear and set scan response data */
	memset(&ble_scan_resp, 0, sizeof(ble_scan_resp));
	/* set UUIDs fields */
    ble_scan_resp.uuids_complete.uuid_cnt = sizeof(adv_uuids) / sizeof(adv_uuids[0]);
    ble_scan_resp.uuids_complete.p_uuids  = adv_uuids;

	/* set advertising data. Maximum available size is BLE_GAP_ADV_MAX_SIZE */
	err_code = ble_advdata_set(&ble_adv_data, &ble_scan_resp);
	APP_ERROR_CHECK(err_code);

	/* Initialize advertising parameters with defaults values */
    memset(&adv_params, 0, sizeof(adv_params));
    
	/* indirect advertising */
    adv_params.type = BLE_GAP_ADV_TYPE_ADV_IND;
	/* No peer address */
    adv_params.p_peer_addr = NULL;
	/* Allow scan requests and connect requests from any device */
    adv_params.fp = BLE_GAP_ADV_FP_ANY;
	/* No whitelist */
    adv_params.p_whitelist = NULL;
	/* Set advertising interval */
	adv_params.interval = APP_ADV_INTERVAL;
	/* set advertising timeout */
    adv_params.timeout = APP_ADV_TIMEOUT_IN_SECONDS;

	/* advertising is started in a separated function */
}




/* ------------------ Exported functions -------------------- */

/* Function for BLE services init and start advertising */
void ble_man_init(void)
{
	/* init stack */
    ble_stack_init();
	/* init gap params */
    gap_params_init();
	/* init services */
    services_init();
	/* init connection params */
    conn_params_init();
}


/* Function to start scanning devices */
void ble_man_scan_start(void)
{
	uint32_t err_code;

	/* start scanning */
    err_code = sd_ble_gap_scan_start(&m_scan_params);
    APP_ERROR_CHECK(err_code);
}


/* Function to start scanning devices */
void ble_man_scan_stop(void)
{
	/* stop scanning */
	sd_ble_gap_scan_stop();
}


/* Function to start advertising */
void ble_man_adv_start(void)
{
	uint32_t err_code;

	/* set advertising data */
	ble_periph_adv_set_data();
	
	/* start advertising */
	err_code = sd_ble_gap_adv_start(&adv_params);
    APP_ERROR_CHECK(err_code);
}


/* Function to stop advertising */
void ble_man_adv_stop(void)
{
	uint32_t err_code;

	/* stop advertising */
	err_code = sd_ble_gap_adv_stop();
	APP_ERROR_CHECK(err_code);
}


/*
	uint32_t err_code;

	err_code = sd_ble_gap_tx_power_set(TX_POWER_LEVEL);
	APP_ERROR_CHECK(err_code);
*/




/* End of file */


