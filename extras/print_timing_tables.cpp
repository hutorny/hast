
/* print_timing_tables.cpp - time table dumper for HAST Transmitter
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

#include <cstdio>
#define HAST_DEBUG
#include "hast.hpp"
using namespace std;

static constexpr hast::clock_t clock = 8000000;

using test9k = hast::transmitter<clock,hast::_9600,
		hast::avr::driver<34,33,32,0>, hast::stopbits::two>;


using test19k = hast::transmitter<clock,hast::_19200,
		hast::avr::driver<34,33,32,0>, hast::stopbits::two>;

using test38k = hast::transmitter<clock,hast::_38400,
		hast::avr::driver<34,33,32,0>, hast::stopbits::two>;

using test56k = hast::transmitter<clock,hast::_57600,
		hast::avr::driver<34,33,32,0>, hast::stopbits::two>;

using test115k = hast::transmitter<clock,hast::_115200,
		hast::avr::driver<34,33,32,0>, hast::stopbits::two>;

using test230k = hast::transmitter<clock,hast::_230400,
		hast::avr::driver<34,33,32,0>>;

using test460k = hast::transmitter<clock,hast::_460800,
		hast::avr::driver<34,33,32,0>>;

using test920k = hast::transmitter<clock,hast::_921600,
		hast::avr::driver<34,33,32,0>>;

namespace hast {
	template<clock_t clock, clock_t baudrate, typename driver, stopbits stopbit>
	template<uint8_t bit>
	void transmitter<clock, baudrate, driver, stopbit>::t<bit>::debug() {
		printf(":%d :%8lld :%8lld :%6u :%6u : %4d :%5lld :%5.1f%% :\n",
			bit, mstart, mfinish, rstart, rfinish, wait, error,
			(100.0 * error) / milicycles_per_bit
		);
	}
}

template<template<uint8_t> class T, uint8_t ... L>
struct iterate;

template<template<uint8_t> class T>
struct iterate<T> {
	static void debug() {}
};


template<template<uint8_t> class T, uint8_t V, uint8_t ... L>
struct iterate<T, V, L...> {
	static void debug() {
		T<V>::debug();
		iterate<T, L...>::debug();
	}
};

void print_section(hast::clock_t baud, hast::clock_t clock) {
	printf(":--:---------:---------:-------:-------:------:------:-------:\n");
	printf(": Baudrate %6lld, clock %8lld Hz                         :\n",
			baud, clock);
	printf(":--:---------:---------:-------:-------:------:------:-------:\n");
}


int main() {
	printf(":# : mstart  : mfinish : rstart:rfinish: wait : error:  err%% :\n");
	print_section(115200, clock);
	iterate<test115k::t,0,1,2,3,4,5,6,7,8,9>::debug();
	print_section(230400, clock);
	iterate<test230k::t,0,1,2,3,4,5,6,7,8,9>::debug();
	print_section(460800, clock);
	iterate<test460k::t,0,1,2,3,4,5,6,7,8,9>::debug();
	print_section(921600, clock);
	iterate<test920k::t,0,1,2,3,4,5,6,7,8,9>::debug();
	return 0;
}
