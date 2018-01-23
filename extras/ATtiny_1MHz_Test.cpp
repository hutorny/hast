/* ATtiny_1MHz_Test.cpp - a test for HAST Transmitter
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

using trans = hast::avr::transmitter<
	1000000,					/* 1 MHz								*/
	hast::_230400,				/* 230400 baud, maximal for 1 MHz		*/
	ADDR_OF(PORTB),				/* port									*/
	PORTB3,						/* pin									*/
	0,							/* space (zero) level					*/
	hast::stopbits::one>;

inline void test() {
	for(uint8_t i = 0; i < 80; ++i) {
		trans::send(0xFF);
	}
	for(uint8_t i = 0; i < 80; ++i) {
		trans::send(0x00);
	}
	for(uint8_t i = 0; i < 80; ++i) {
		trans::send(0xF0);
	}
	for(uint8_t i = 0; i < 80; ++i) {
		trans::send(0x0F);
	}
	for(uint8_t i = 0; i < 80; ++i) {
		trans::send(0xCC);
	}
	for(uint8_t i = 0; i < 80; ++i) {
		trans::send(0x33);
	}
	for(uint8_t i = 0; i < 80; ++i) {
		trans::send(0xAA);
	}
	for(uint8_t i = 0; i < 80; ++i) {
		trans::send(0x55);
	}
}

/** ROM usage on ATtiny45: 242 bytes							*/
int main(void) {
	/* RC oscillator may need fine-tuning if operation conditions 
	  (Vcc, temperature) differ from calibration conditions 
	  Refer to the datasheet for details						*/
	/OSCCAL -= 5; 	
	trans::init();
    while(true) test();		
}

