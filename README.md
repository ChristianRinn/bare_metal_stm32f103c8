# Minimal gcc makefile project for STM32F103C8

This contains the absolute bare minimum needed
to compile a blinky example for the STM32F103C8
on the command line with gcc and make and then
flash it to the demo board with OpenOCD. It is
using the original headers provided by STM and
ARM but only those for the core and peripheral
registers, none of the HAL or any other optional
libs.

The startup code has been implemented from scratch 
and without the Atollic copyright, the startup code 
and the SystemInit have been implemented in plain C 
and are in the file gcc_startup_system.c.


# Usage

## Prerequisites

* install Linux on your PC
* get a "blue pill" board and an ST-Link V2/V2.1 (Discovery or Nucleo boards have one of these)
* install the official arm-none-eabi-gcc toolchain from Launchpad
* install OpenOCD

## Build and run

* clone this repository
* connect the ST-Link to your "blue pill" and to your PC
* `$ make install` or `$ make OOCD_IFACE=stlink-v2-1 install`
* watch the red LED blink while studying the reference manual
* by default example01 in the src directory is built, run `make EXAMPLE=example02` to compile the second one, etc.

happy hacking :-)

