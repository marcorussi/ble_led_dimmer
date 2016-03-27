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

#include <stdint.h>
#include <stdbool.h>




/* ------------- Exported defines --------------- */

/* Position in words of the CMD field */
#define MEM_CMD_FIELD_WORD_POS						0

/* Position in words of the POWER S field */
#define MEM_POWER_S_FIELD_WORD_POS					1

/* Position in words of the timers fields */
#define MEM_TIMERS_FIELDS_WORD_POS					2




/* ------------- Exported functions --------------- */

extern bool memory_is_busy		(void);
extern bool memory_update_field	(uint8_t, uint8_t *, uint8_t);
extern bool memory_init			(const uint8_t *);




/* End of file */




