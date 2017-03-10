#include <stm32f1xx.h>


/*
 * The red LED is connected to port PC13,
 * -> see schematic or pinout of "blue pill" board
 */
#define LED_GPIO        GPIOC
#define LED_PIN         13


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
 * Hello world blinky program
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
     * Clear configuration and mode bits of LED-Pin
     * configuration bits 0b00 -> GPIO push-pull
     * Set LED-Pin mode bits to 0b01 -> output mode max. speed 2 MHz
     * 
     * -> see section 9.2.2 in the manual
     */
    LED_GPIO->CRH = (LED_GPIO->CRH & (~(GPIO_CRH_CNF13 | GPIO_CRH_MODE13))) | (1 << GPIO_CRH_MODE13_Pos);


    while(1) {

        /*
         * LED on
         * On the blue pill board this actually means to pull the pin down.
         * Pin is set low through the BSRR 
         * -> see section 9.2.5 in the manual
         */
        LED_GPIO->BSRR = (GPIO_BSRR_BR0 << LED_PIN);

        delay(2000);

        /*
         *  LED off
         */
        LED_GPIO->BSRR = (GPIO_BSRR_BS0 << LED_PIN);

        delay(1000);
    }
}