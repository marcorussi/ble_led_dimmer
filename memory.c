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

#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "nrf_gpio.h"
#include "softdevice_handler.h"
#include "pstorage.h"

#include "config.h"
#include "memory.h"
#include "dimmer_service.h"




/* ------------- Local typedefs --------------- */

/* Current state enum */
typedef enum
{
	ERROR_STATE,
	IDLE_STATE,
	LOAD_SIGNATURE,
	LOAD_DATA,
	STORE_DATA,
	UPDATE_DATA,
	RESTORE_DEFAULT
} curr_state_e;




/* ------------- Local defines --------------- */

/* Validity signature for default value */
#define MEM_DEFAULT_SIGNATURE						0x22224499

/* Signature length in bytes */
#define MEM_SIGNATURE_LENGTH_BYTES					4

/* Memory blocks size in bytes. Muat be aligned to 4. 
TODO: implement auto-alignment and check if greater than or equal to BLE_DIMMER_SERVICE_CHARS_LENGTH + MEM_SIGNATURE_LENGTH_BYTES */
#define MEM_BLOCK_SIZE_BYTES						40

/* Memory validity signature position */
#define MEM_SIGNATURE_FIELD_BYTE_POS				(MEM_BLOCK_SIZE_BYTES - MEM_SIGNATURE_LENGTH_BYTES)

/* Number of blocks */
#define NUM_OF_MEM_BLOCKS							1




/* ------------- Local variables --------------- */

/* Current memory state */
static uint8_t curr_state = IDLE_STATE;

/* Signature for validity check */
static uint32_t ps_signature;

/* Persistent storage base handle */
static pstorage_handle_t base_handle;

/* Persistent storage block handle */
static pstorage_handle_t block_handle;

/* Pointer to temporary block data */
static uint8_t temp_data[MEM_BLOCK_SIZE_BYTES];

/* Pointer to default data values */
static const uint8_t *p_def_values;

/* Signature field value */
static const uint32_t signature_field = MEM_DEFAULT_SIGNATURE;




/* ------------- Local functions prototypes --------------- */

static void sys_evt_dispatch(uint32_t);
static void ps_cb_handler(pstorage_handle_t *, uint8_t, uint32_t, uint8_t *, uint32_t);




/* ------------- Exported functions --------------- */

/* Function to get memory status */
bool memory_is_busy(void)
{
	bool mem_is_busy = false;
		
	if(curr_state != IDLE_STATE)
	{
		mem_is_busy = true;
	}

	return mem_is_busy;
}


/* Function to update a memory field */
bool memory_update_field(uint8_t mem_position, uint8_t *p_data, uint8_t length)
{
	uint32_t retval;
	bool ps_success = true;

	for(uint8_t i=0; i<length; i++)
	{
		temp_data[i+mem_position] = p_data[i];
	}

	/* go to UPDATE_DATA */
	curr_state = UPDATE_DATA;
	/* update a field */
	retval = pstorage_update(&base_handle, temp_data, MEM_BLOCK_SIZE_BYTES, 0);
	if (retval != NRF_SUCCESS)
	{
		/* failed to update data: persistent storage failure */
		ps_success = false;
	}

	return ps_success;
}


/* Function to init persistent memory */
bool memory_init(const uint8_t *p_def_val)
{
	uint32_t err_code;
	uint32_t retval;
	bool ps_success;
	pstorage_module_param_t param;

	ps_success = true;

	err_code = softdevice_sys_evt_handler_set(sys_evt_dispatch);
    APP_ERROR_CHECK(err_code);
	
	/* init persistent storage */
	retval = pstorage_init();

	/* if module initialization successful */
	if(retval == NRF_SUCCESS)
	{
		/* init parameters */
		param.block_size  = MEM_BLOCK_SIZE_BYTES;
		param.block_count = NUM_OF_MEM_BLOCKS;
		param.cb          = ps_cb_handler;

		/* register persistent storage */
		retval = pstorage_register(&param, &base_handle);

		/* if registration successful */
		if (retval == NRF_SUCCESS)
		{
			/* request to get identifier for first block */
			retval = pstorage_block_identifier_get(&base_handle, 0, &block_handle);
			/* get Block Identifier successful */
			if (retval == NRF_SUCCESS)
			{
				/* copy pointer to default values */
				p_def_values = p_def_val;

				/* operation success: wait for signature value */
				curr_state = LOAD_SIGNATURE;
				/* load signature field */
				retval = pstorage_load((uint8_t *)&ps_signature, &block_handle, MEM_SIGNATURE_LENGTH_BYTES, MEM_SIGNATURE_FIELD_BYTE_POS);
				if (retval != NRF_SUCCESS)
				{
					/* failed to load the signature: persistent storage failure */
					ps_success = false;
				}
			}
			else
			{
				/* failed to get block id: persistent storage failure */
				ps_success = false;
			}
		}
		else
		{
			/* failed to register: persistent storage failure */
			ps_success = false;
		}
	}
	else
	{
		/* initialization failed: persistent storage failure */
		ps_success = false;
	}

	return ps_success;
}




/* ------------- Local functions --------------- */

/* Function for dispatching a system event to interested modules */
static void sys_evt_dispatch(uint32_t sys_evt)
{
    pstorage_sys_event_handler(sys_evt);
}


/* PS event notification handler */
static void ps_cb_handler(pstorage_handle_t *handle, uint8_t op_code, uint32_t result, uint8_t *p_data, uint32_t data_len)
{
	uint32_t retval;

	/* manage received operation code */
	switch(op_code)
	{
		case PSTORAGE_STORE_OP_CODE:
		{
			if (result == NRF_SUCCESS)
			{
				/* if store data state */
				if(curr_state == STORE_DATA)
				{
					/* store success: go to IDLE */
					curr_state = IDLE_STATE; 
				}
				else
				{
					/* do nothing */
				}
			}
			else
			{
				curr_state = ERROR_STATE;
			}
			break;
		}
		case PSTORAGE_LOAD_OP_CODE:
		{
			if (result == NRF_SUCCESS)
			{
				/* if load signature state */
				if(curr_state == LOAD_SIGNATURE)
				{
					/* if data are valid */
					if(ps_signature == MEM_DEFAULT_SIGNATURE)
					{
						/* operation success: wait for data */
						curr_state = LOAD_DATA;
						/* use stored data: load entire block with signature field */
						retval = pstorage_load(temp_data, &block_handle, MEM_BLOCK_SIZE_BYTES, 0);
						if (retval != NRF_SUCCESS)
						{
							/* failed to load the signature: persistent storage failure */
							curr_state = ERROR_STATE;
						}
					}
					else
					{
						/* copy default data to values */
						memcpy((void *)char_values, (const void *)p_def_values, BLE_DIMMER_SERVICE_CHARS_LENGTH);

						/* go to RESTORE DEFAULT state */
						curr_state = RESTORE_DEFAULT;
						/* invalid data: use the default values. Clear the entire block first */
						retval = pstorage_clear(&base_handle, MEM_BLOCK_SIZE_BYTES);	
						if (retval != NRF_SUCCESS)
						{
							/* failed to clear memory: persistent storage failure */
							curr_state = ERROR_STATE;
						}
					}
				}
				/* if load data state */
				else if(curr_state == LOAD_DATA)
				{
					/* copy loaded data to values */
					memcpy((void *)char_values, (const void *)temp_data, BLE_DIMMER_SERVICE_CHARS_LENGTH);
	
					/* data loaded successfully */
					/* go to IDLE state */
					curr_state = IDLE_STATE;
				}
				else
				{
					/* do nothing */
				}
			}
			else
			{
				curr_state = ERROR_STATE;
			}
			break;
		}
		case PSTORAGE_UPDATE_OP_CODE:
		{
			if (result == NRF_SUCCESS)
			{
				/* if update data state */
				if(curr_state == UPDATE_DATA)
				{
					/* store success: go to IDLE */
					curr_state = IDLE_STATE; 
				}
				else
				{
					/* do nothing */
				}
			}
			else
			{
				curr_state = ERROR_STATE;
			}
			break;
		}
		case PSTORAGE_CLEAR_OP_CODE:
		{
			if (result == NRF_SUCCESS)
			{
				/* if restore default state */
				if(curr_state == RESTORE_DEFAULT)
				{
					/* prepare data to be stored */
					memcpy((void *)temp_data, (const void *)p_def_values, BLE_DIMMER_SERVICE_CHARS_LENGTH); 
					memcpy((void *)&temp_data[MEM_SIGNATURE_FIELD_BYTE_POS], (const void *)&signature_field, MEM_SIGNATURE_LENGTH_BYTES); 

					/* go to STORE_DATA state */
					curr_state = STORE_DATA; 
					/* store default data with signature */
					retval = pstorage_store(&block_handle, temp_data, MEM_BLOCK_SIZE_BYTES, 0);
					if (retval != NRF_SUCCESS)
					{
						/* failed to store data: persistent storage failure */
						curr_state = ERROR_STATE;
					}
				}
				else
				{
					/* do nothing */
				}
			}
			else
			{
				curr_state = ERROR_STATE;
			}
			break;
		}
		default:
		{
			break;
		}
	}
}




/* End of file */





