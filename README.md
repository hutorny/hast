# HAST
Software Defined High-speed Asynchronous Serial Transmitter

`HAST` is a C++11 template that generates a transmitter routine for a software `UART`.
It deploys unrolled loop with variable padding technique for generating precise timing between bits, and a short sequence (3 cycles) for driving the output. Such approach allows baud rates up to `CLOCK/3`.


## Usage
HAST is a header-only library. Include `hast.hpp` and instantiate `hast::avr::trasmitter` as the following

```
#include "hast.hpp"

void hast_putc(char c) {
	hast::avr::transmitter<
		1000000,							/* 1 MHz			*/
		hast::_115200,					/* 115200 baud	*/
		ADDR_OF(PORTB), PORTB3, 0,	/* pin PORTB3	*/
		hast::stopbits::one			/* one stop bit	*/ 
	>::send(c);
}
```

## Notes

### Clock error

Built-in RC-oscillator is calibrated for certain conditions. If operating conditions differ, RC-oscillator may deviate from the expected frequency. This deviation may have negative impact on HAST operations. To improve results, you may need fine tune the RC-oscillator.

### Timing table
Baudrate 921600, clock  8 MHz

|# | start<br>μs | finish<br>μs | start<br>cycles| finish<br>cycles | wait<br>cycles | error<br>μs|  err % |
|--|--------:|--------:|------:|------:|-----:|-----:|------:|
|0 |       0 |    8680 |     0 |     9 |    6 |  320 |  3.7% |
|1 |    8680 |   17360 |     9 |    17 |    5 |  360 |  4.1% |
|2 |   17360 |   26040 |    17 |    26 |    6 |   40 |  0.5% |
|3 |   26040 |   34720 |    26 |    35 |    6 |  280 |  3.2% |
|4 |   34720 |   43400 |    35 |    43 |    5 |  400 |  4.6% |
|5 |   43400 |   52080 |    43 |    52 |    6 |   80 |  0.9% |
|6 |   52080 |   60760 |    52 |    61 |    6 |  240 |  2.8% |
|7 |   60760 |   69440 |    61 |    69 |    5 |  440 |  5.1% |
|8 |   69440 |   78120 |    69 |    78 |    6 |  120 |  1.4% |
|9 |   78120 |   86800 |    78 |    87 |    7 |  200 |  2.3% |
