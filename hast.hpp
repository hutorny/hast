/* HAST - Software Defined High-speed Asynchronous Serial Transmitter
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

#pragma once
#include <stdint.h> //TODO change to cstdint when it becomes available
#ifdef __AVR__
#	include <avr/io.h>
#endif

#if __cplusplus <  201103L 
#	error "HAST requires --std=c++11 (or higher)"
#	include <finalstop>
#endif

/* Suppress assert for debugging purposes									*/
#ifdef HAST_DEBUG
#	define static_warn(a,b) static_assert(true,b)
#else
#	define static_warn(a,b) static_assert(a,b)
#endif

/*  Video instructions are available at 
    https://www.youtube.com/watch?v=W3q8Od5qJio								*/

namespace hast {
	using time_t		= int64_t;	/** time in ns							*/
	using clock_t		= uint64_t;	/** frequency in Hz						*/
	using cycle_t		= int32_t;	/** count of instruction cycles			*/
	using milicycle_t	= int64_t;	/** cycle_t * 1000						*/
	
	static constexpr time_t nano = 1000000000LL;	/* nano factor			*/
	static constexpr milicycle_t mili = 1000LL;		/* mili factor			*/
	static constexpr uint8_t uart_space_level = 0;	/* default SPACE level	*/


	/** standard baud rates													*/
	enum baudrate : clock_t { 
		/* not made enum class to allow arbitrary baudrates					*/
		_9600   =   9600ULL,
		_19200  =  19200ULL,
		_38400  =  38400ULL,
		_57600  =  57600ULL,
		_115200 = 115200ULL,
		_230400 = 230400ULL,
		_460800 = 460800ULL,
		_921600 = 921600ULL
		
	};
	
	/** Stop bits values. 
	 * Stop-bits are elapsed after actuating the last bit to ensure proper 
	 * framing. An application  may request shorter stop-bits, provided 
	 * it will not start next byte too early								*/  
	enum class stopbits : uint8_t {
		none, 		/* app may use none/half if interval between chars		*/
		half, 		/* is longer that duration of a bit/half bit 			*/
		one,		/** on bit												*/
		one_half, 	/** one and a half bits									*/
		two 		/** two	bits											*/
	};

	/* local constexpr abs													*/
	inline constexpr time_t abs(time_t v) { return v < 0 ? -v : v; }

	/* template recursion with a brake										*/
	template<uint8_t i, template<uint8_t> class A, class B>
	struct prev : A<i-1> {};

	template<template<uint8_t> class A, class B>
	struct prev<0,A,B> : B {};
	
	
	/** transmitter template
	  * params
	  *   clock		- system cloc, Hz
	  *   baudrate	- UART baudrate, baud
	  *   driver	- class, implementing hardware specific line control
	  *				  driver also refers to underlaying MCU for delay 
	  *				  implementation
	  *   stopbit	- stop bit duration
	  *																		*/
	template<clock_t clock, clock_t baudrate, typename driver,
		stopbits stopbit = stopbits::one>
	struct transmitter {
		using mcu = typename driver::mcu;
		static constexpr auto cpi = mcu::cpi;
		template<cycle_t period_ic>
		struct delay : mcu::template delay<period_ic> {};
		
		/** send one byte of data											*/		  
		__attribute__((optimize("-Os")))
		static volatile void send(uint8_t data) {
			driver::start(data);
			delay<t<0>::wait>::cycles();
			driver::template send<0>(data);
			delay<t<1>::wait>::cycles();
			driver::template send<1>(data);
			delay<t<2>::wait>::cycles();
			driver::template send<2>(data);
			delay<t<3>::wait>::cycles();
			driver::template send<3>(data);
			delay<t<4>::wait>::cycles();
			driver::template send<4>(data);
			delay<t<5>::wait>::cycles();
			driver::template send<5>(data);
			delay<t<6>::wait>::cycles();
			driver::template send<6>(data);
			delay<t<7>::wait>::cycles();
			driver::template send<7>(data);
			delay<t<8>::wait>::cycles();
			driver::stop(data);
			delay<t<9>::wait>::cycles();
		}
		
		/** initialize the line (call the driver)							*/
		static inline void init() {
			driver::init();
		}

		static constexpr milicycle_t milicycles_per_bit =
				(mili * clock / cpi) / baudrate;
		static_warn(milicycles_per_bit >= mili * driver::send_cycles,
				"Baudrate is too high for given clock and driver");
		static_warn(milicycles_per_bit <= mili * delay<milicycles_per_bit/mili>::max,
				"Baudrate is too low for given clock and driver");

		static constexpr cycle_t timing(uint8_t bit, cycle_t rstart, time_t mfinish) {
			return (mfinish - rstart * mili) / mili;
		}

		static constexpr cycle_t best(time_t goal, cycle_t rstart, cycle_t a, cycle_t b) {
			return abs(goal - rstart*mili - a*mili) < abs(goal - rstart*mili - b*mili) ? a : b;
		}
			
		static constexpr cycle_t bitlen(uint8_t bit, cycle_t rstart, time_t mfinish) {
			return best(mfinish, rstart, timing(bit,rstart, mfinish), timing(bit,rstart, mfinish) + 1);
		}
			
		/** recursion terminator 											*/
		struct t0 {
			static constexpr cycle_t rfinish = 0;
		};

		/** waveform time-table 											*/
		template<uint8_t bit>
		struct t { /* bit 0 is the start bit 								*/
			/** cycles to actuate current bit */
			static constexpr cycle_t cycles = 
				(bit == 0) ? driver::start_cycles + driver::send_cycles:
				(bit == 9 ? driver::stop_cycles : driver::send_cycles);
			/** ideal bit start time in mili cycles */
			static constexpr milicycle_t mstart  = milicycles_per_bit * bit;
			/** ideal bit finish time in mili cycles */
			static constexpr milicycle_t mfinish = bit == 9
					? (milicycles_per_bit*bit +
					  (milicycles_per_bit*static_cast<milicycle_t>(stopbit))/2)
					: milicycles_per_bit * (bit+1);

			/** actual bit start time in cycles - where the previous finishes */
			static constexpr cycle_t rstart  = prev<bit, t, t0>::rfinish;
			/** computed bit length in cycles */
			static constexpr cycle_t length = bitlen(bit, rstart, mfinish);
			/** bit padding period in cycles */
			static constexpr cycle_t wait = (length < cycles) ? 0 : (length - cycles);
			/** actual finish time */
			static constexpr cycle_t rfinish = rstart + wait + cycles;
			/** absolute error in milicycles 								*/
			static constexpr milicycle_t error = abs(mfinish - mili * rfinish);
			static_warn(bit == 9 || error <= (milicycles_per_bit / 5),
					"Bit error exceeds 20%, use different (lower) baudrate");
			static_warn(bit == 9 || wait <= driver::mcu::template delay<0>::max,
				"Bit error exceeds 20%, use different (lower) baudrate");

			static void debug(); /* implement this method as needed
													for time-table debugging */
		};
	};
	
	/************************************************************************/
	/** AVR-specific concerns												*/
	struct avr {
		static constexpr cycle_t cpi = 1; /* clock per instruction			*/
		using sfr = volatile uint8_t;
		/** delay loops with 3 (short) and 6 (long) ic per iteration 		*/
		template<cycle_t period_ic, bool longloop>
		struct loop;

		/** 
		 * Register usage:
		 * r18 - loop count and work register
		 * r19 - keeps __SREG__ 
		 * r20 - keeps port state
		 * r21 - prepared data (data &= (data<<1))
		 * r22 - toggle mask (1<<bit)
		 */

		/** delay injector 													*/
		template<cycle_t period_ic>	
		struct delay {
			static constexpr cycle_t max = 255*6 + 2;
			__attribute__((always_inline)) 
			static inline volatile void cycles() {
				loop<period_ic, longloop>::cycles();
			}
			static_warn(period_ic <= max, "Delay too long");
		private:
			static constexpr bool longloop = period_ic > (255*3);
		};

		/** PORT driver - uses out to drive the pin							*/
		template<uint8_t port, uint8_t ddr, uint8_t pin, uint8_t space = uart_space_level>
		struct driver {
			/* NOTE: driver disables interrupts on start and enables on stop
					 driver uses the following sequence to send a bit:
						sbrc (data), (bit)
						eor  r20, (1<<bit)
						out  (port), r20									*/
			using mcu = avr;  /* hast::trasmitter uses mcu::delay			*/
			static constexpr cycle_t send_cycles  = 3; /* sbrc, eor, out	*/
			static constexpr cycle_t start_cycles = 0; /* nothing after out */
			static constexpr cycle_t stop_cycles  = 2; /* ori, out			*/

			/** initialize port for output									*/
			__attribute__((always_inline)) 
			static inline void init() {
				if( space == 0 )
					asm volatile ("sbi %0,%2\n sbi %1, %2" 
								 :: "I" (port), "I" (ddr), "I" (pin): "memory");
				else
					asm volatile ("cbi %0,%2\n sbi %1, %2" 
								 :: "I" (port), "I" (ddr), "I" (pin): "memory");
			}

			/** prepare data, disable interrupts, send start bit 			*/
			__attribute__((always_inline)) 
			static inline void start(uint8_t data) {
				asm volatile ("mov r18, %0\n mov r21, r18" 
							 :"=r"(data) :: "cc", "r18", "r21", "memory");
				asm volatile ("lsl r18\n eor r21, r18\n ldi r22,%0" 
							 ::"M"(1<<pin): "cc", "r18", "r21", "r22", "memory");
				if( space == 0 )
					asm volatile ("in r20, %0\n andi r20,~%1\n in r19, __SREG__\n cli\n out %0, r20"
								 ::"I" (port), "M"(1<<pin) : "cc", "r20", "r19", "memory");
				else
					asm volatile ("in r20, %0\n ori  r20,%1\n in r19, __SREG__\n cli\n out %0, r20"
								 ::"I" (port), "M"(1<<pin) : "cc", "r20", "r19", "memory");
			}
			/** send a bit 															*/
			template<uint8_t bit>
			__attribute__((always_inline))
			static inline void send(uint8_t data) {
				asm volatile ("sbrc r21, %0\n eor r20, r22\n out %1, r20" 
							 :: "I" (bit), "I" (port): "r21", "r22", "memory");
			}
			/** send stop bit, enable interrupts				 					*/
			__attribute__((always_inline))
			static inline volatile void stop(uint8_t data) {
				if( space == 0 )
					asm volatile ("ori r20,%1\n out %0, r20\n out __SREG__, r19"
								 ::"I" (port), "M"(1<<pin):"cc", "r20", "r19", "memory");
				else
					asm volatile ("andi r20,~%1\n out %0, r20\n out __SREG__, r19"
								 ::"I" (port), "M"(1<<pin):"cc", "r20", "r19", "memory");
			}
		private:
			static_assert(space==0 || space==1, "Invalid space value. Valid values are: 0, 1");
		};
		
		/** shortcut for hast::transmitter with avr::driver							*/
		template<clock_t clock, clock_t baudrate, uint8_t port, uint8_t pin, 
				 uint8_t space = uart_space_level, stopbits sb=stopbits::one>
		struct transmitter;
	};

	/* Implementation of delay injectors
	   credits to http://www.bretmulvey.com/avrdelay.html							*/
	template<> inline volatile void avr::delay< 0>::cycles() {}
	template<> inline volatile void avr::delay< 1>::cycles() { asm volatile("nop"); }
	template<> inline volatile void avr::delay< 2>::cycles() { asm volatile("rjmp ."); }
	template<> inline volatile void avr::delay< 3>::cycles() { asm volatile("lpm"); }
	template<> inline volatile void avr::delay< 4>::cycles() { delay<2>::cycles(); delay<2>::cycles(); }
	template<> inline volatile void avr::delay< 5>::cycles() { delay<3>::cycles(); delay<2>::cycles(); }
	template<> inline volatile void avr::delay< 6>::cycles() { delay<3>::cycles(); delay<3>::cycles(); }
	template<> inline volatile void avr::delay< 7>::cycles() { delay<6>::cycles(); delay<1>::cycles(); }
	template<> inline volatile void avr::delay< 8>::cycles() { delay<6>::cycles(); delay<2>::cycles(); }
	template<> inline volatile void avr::delay< 9>::cycles() { delay<6>::cycles(); delay<3>::cycles(); }
	template<> inline volatile void avr::delay<10>::cycles() { delay<9>::cycles(); delay<1>::cycles(); }
	template<> inline volatile void avr::delay<11>::cycles() { delay<9>::cycles(); delay<2>::cycles(); }
		
	/** Short loop delay (period/3 cycles)											*/
	template<cycle_t period_ic>
	struct avr::loop<period_ic, false> {
		static constexpr cycle_t count = period_ic/3;
		static_warn(count <= 255, "Loop is too long");
		__attribute__((always_inline)) 
		static inline volatile void cycles() {
			asm volatile ("ldi  r18, %0\n dec  r18\n brne .-4"
						::"M"(count):"cc","r18");
			delay<period_ic % 3>::cycles();			
		}
	};
	
	/** Long loop delay (period/6 cycles)											*/
	template<cycle_t period_ic>
	struct avr::loop<period_ic, true> {
		static constexpr cycle_t count = period_ic/6;
		static_warn(count <= 255, "Loop is too long");
		__attribute__((always_inline)) 
		static inline volatile void cycles() {
			asm volatile ("ldi  r18, %0\n lpm; \n dec  r18\n brne .-5"
						::"M" (count):"cc","r18");
			delay<period_ic % 6>::cycles();			
		}
	};

/* Use of PORTx is not allowed in templates, ADDR_OF provides a workaround		*/
#ifdef _SFR_IO8
#define ADDR_OF(R) ((&R)-&_SFR_IO8(0))

#if defined(PORTA0) || defined(PORTA1) || defined(PORTA2) || defined(PORTA3) || \
	defined(PORTA4) || defined(PORTA5) || defined(PORTA6) || defined(PORTA7)
	template<clock_t clock, clock_t baudrate, uint8_t pin, uint8_t space, stopbits sb>
	struct avr::transmitter<clock, baudrate, ADDR_OF(PORTA), pin, space, sb> :
		hast::transmitter<clock, baudrate, driver<ADDR_OF(PORTA), 
		ADDR_OF(DDRA), pin, space>, sb> {};
#endif

#if defined(PORTB0) || defined(PORTB1) || defined(PORTB2) || defined(PORTB3) || \
	defined(PORTB4) || defined(PORTB5) || defined(PORTB6) || defined(PORTB7) 
	template<clock_t clock, clock_t baudrate, uint8_t pin, uint8_t space, stopbits sb>
	struct avr::transmitter<clock, baudrate, ADDR_OF(PORTB), pin, space, sb> :
		hast::transmitter<clock, baudrate, driver<ADDR_OF(PORTB), 
		ADDR_OF(DDRB), pin, space>, sb> {};
#endif

#if defined(PORTC0) || defined(PORTC1) || defined(PORTC2) || defined(PORTC3) || \
	defined(PORTC4) || defined(PORTC5) || defined(PORTC6) || defined(PORTC7)
	template<clock_t clock, clock_t baudrate, uint8_t pin, uint8_t space, stopbits sb>
	struct avr::transmitter<clock, baudrate, ADDR_OF(PORTC), pin, space, sb> :
		hast::transmitter<clock, baudrate, driver<ADDR_OF(PORTC), 
		ADDR_OF(DDRC), pin, space>, sb> {};
#endif

#if defined(PORTD0) || defined(PORTD1) || defined(PORTD2) || defined(PORTD3) || \
	defined(PORTD4) || defined(PORTD5) || defined(PORTD6) || defined(PORTD7)
	template<clock_t clock, clock_t baudrate, uint8_t pin, uint8_t space, stopbits sb>
	struct avr::transmitter<clock, baudrate, ADDR_OF(PORTD), pin, space, sb> :
		hast::transmitter<clock, baudrate, driver<ADDR_OF(PORTD), 
		ADDR_OF(DDRD), pin, space>, sb> {};
#endif
#endif
	
}
