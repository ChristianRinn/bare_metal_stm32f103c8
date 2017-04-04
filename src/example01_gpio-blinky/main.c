#include <stm32f1xx.h>


/*
 * The red LED is connected to port PC13,
 * -> see schematic or pinout of "blue pill" board
 */


/**
 * Quick 'n' dirty delay
 *
 * @param time the larger it is the longer it will block
 */
static void delay(unsigned time) {
    for (unsigned i=0; i<time; i++)
        for (volatile unsigned j=0; j<2000; j++);
}


/**
 * Hello world blinky program with busy wait
 *
 * @return never
 */
int main(void) {

    /*
     * Turn on the GPIOC unit,
     * -> see section 7.3.7 in the manual
     */
    RCC->APB2ENR  |= RCC_APB2ENR_IOPCEN;


    /*
     * Clear configuration and mode bits of PC_13
     * configuration bits 0b00 -> GPIO push-pull
     * Set LED-Pin mode bits to 0b01 -> output mode max. speed 2 MHz
     * The configuration register is split into two registers for the STM32F1.
     * Configuration register low (CRL) is for pin 0 to 7 and the
     * "Configuration Register High" (CRH) is for pin 8 to 15. 
     * 
     * -> see section 9.2.2 in the manual
     */
    GPIOC->CRH = (GPIOC->CRH & (~(GPIO_CRH_CNF13 | GPIO_CRH_MODE13))) | (1 << GPIO_CRH_MODE13_Pos);


    while(1) {

        /*
         * LED on
         * On the blue pill board this actually means to pull the pin down.
         * Pin is set low or high through the BSRR (Bit Set Reset Register) 
         * -> see section 9.2.5 in the manual
         * You can use the Output Data Register (ODR) as well for setting
         * output pins high or low. Using the BSRR provides atomic write
         * capabilities so we do not have to read the content first if we only
         * want to set (turn high) or reset (turn low) a certain pin.
         */
        GPIOC->BSRR = (GPIO_BSRR_BR0 << 13);

        delay(2000);

        /*
         *  LED off
         */
        GPIOC->BSRR = (GPIO_BSRR_BS0 << 13);

        delay(1000);
    }
}
