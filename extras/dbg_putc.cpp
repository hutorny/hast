/* dbg_putc.cpp - a C wrapper for HAST Transmitter
 * 
 * HAST - Software Defined High-speed Asynchronous Serial Transmitter
 * 
 * Copyright (C) 2018 Eugene Hutorny <eugene@hutorny.in.ua>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * IN THE SOFTWARE.
 *
 * https://opensource.org/licenses/MIT
 */

#include <avr/io.h>
#include "hast.hpp"

extern "C"
void dbg_putc(char c) {
	hast::avr::transmitter<
		8000000,					/* 8 MHz					*/
		hast::_921600,				/* 921600 baud				*/
		ADDR_OF(PORTB), PORTB3, 0,	/* pin PORTB3				*/
		hast::stopbits::none		/* no delay after last bit:
			ret load call, start takes more that 7 cycles,
			needed for the stop bit								*/ 
	>::send(c);
}