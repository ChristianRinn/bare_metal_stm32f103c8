#include <stm32f1xx.h>


/*
 * The LED pin (PC13) of the "blue pill" boards has no timer functionality attached to it.
 * For this example we will pick PA6 and PA7 as CH1 and CH2 of timer 3.
 * The pin defines below are actually not used in the gpio_init() function.
 * Adjust them if you want to but be aware that not every pin can be attached to every
 * channel of every timer. You will find a table with possible configurations in the datasheet.
 */

#define PWM_GPIO        GPIOA
#define PWM_PIN1        6
#define PWM_PIN2        7

void gpio_init(void) {

    /*
     * Turn on the GPIOA unit,
     * -> see section 7.3.7 in the manual
     */
    RCC->APB2ENR  |= RCC_APB2ENR_IOPAEN;


    /*
     * Clear configuration and mode bits of Timer-Pin
     * Set timer pin mode bits to 0b01 -> output mode max. speed 2 MHz
     * Set configuration bits to 0b10 -> Alternate function push-pull
     * 
     * -> see section 9.2.2 in the manual
     */
    PWM_GPIO->CRL = ((PWM_GPIO->CRL & (~(GPIO_CRL_CNF6 | GPIO_CRL_MODE6))) 
                    | GPIO_CRL_MODE6_0
                    | GPIO_CRL_CNF6_1);

    /*
     * Repeat to configure PA7. Could be done in one step.
     */
    PWM_GPIO->CRL = ((PWM_GPIO->CRL & (~(GPIO_CRL_CNF7 | GPIO_CRL_MODE7))) 
                    | GPIO_CRL_MODE7_0
                    | GPIO_CRL_CNF7_1);
}



void timer_init(void) {

    /*
     * In this example we have to use at least a "general purpose timer" because
     * a "basic timer" has no capture/compare capabilities. 
     */

    /* enable the clock to timer 3 in the APB1 enable register */
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
    RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;

    /*
     * Timer 3 runs from the APB1 clock which is SCLK/2, i.e. 36 MHz.
     * So if we divide the APB1 clock by 3600 our timer runs at a tick rate
     * of 10 kHz. So we write 3599 in the prescaler register of the timer.
     * Why 3599 and not 3600? Because a value of 0 in the prescaler does 
     * (fortunately) not divide by zero but by one.
     */ 
    TIM3->PSC = 36000 -1; // tick_rate = 36.000.000 Hz / (PSC+1) = 1000 Hz

    /* set the value to count up to / to count down from */
    TIM3->ARR = 2000 -1; 


    /* 
     * "General purpose" timers can not only count up but also down. 
     * The default is upcounting mode but to show how to enable it
     * let's set it to downcounting mode. If you look at periodic signals,
     * the difference between mode 1 and mode 2 is only a phase shift.
     * 
     * -> see section 15.4.1 in the manual
     */
    TIM3->CR1 |= TIM_CR1_DIR;


    /*
     * By setting the OC1M[2:0] bits in the capture/compare mode register to 0b110 we 
     * activate PWM mode 1. In downcounting mode it means that the output is low if the
     * counter value is higher than the value in the capture/compare register and high
     * otherwise. PWM mode 2 would be exactly the opposite.
     * 
     * -> see section 15.4.7 in the manual
     */
    TIM3->CCMR1 |= (TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1 );


    /* set the capture compare register to 75% duty cycle */
    TIM3->CCR1 = 1500 -1;

    /* 
     * The resulting square wave should look like this
     * 
     *    ______    ______    ______ 
     * __|      |__|      |__|      
     * 
     * 
     * -> see section 15.4.9 in the manual
     */
    TIM3->CCER |= TIM_CCER_CC1E;


    /*
     * Now we could actually enable the counter and everything would be hunky-dory.
     * However let's configure one more capture/compare channel and make use of the
     * polarity inversion.
     */

    /* set "pwm mode 1" for channel 2 as well */
    TIM3->CCMR1 |= (TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1);

    /* set CCR2 to 1340 which would be 67% duty cycle, but... */
    TIM3->CCR2 = 1340 - 1;

    /* ... we set the output polarity opposite, resulting in 33% duty cycle */
    TIM3->CCER |= TIM_CCER_CC2P;

    /* enable CC channel 2 */
    TIM3->CCER |= TIM_CCER_CC2E;

    /*
     * So without switching the polarity of our second channel, it would look more
     * or less like the one we have seen before:
     *      _____     _____     _____ 
     *  ___|     |___|     |___|     |
     * 
     * but since we inverted it, it now looks like this:
     * 
     *  ____      ____      ____ 
     * |   |_____|   |_____|   |_____
     *
     * so we have
     * 
     *          ______    ______    ______ 
     *  CC1: __|      |__|      |__|      |
     *       
     *        ____     ____      ____ 
     *  CC2: |   |____|   |_____|   |_____|
     *
     */

    /* 
     * Enable the counter by setting the counter enable bit in the control register
     * of timer 3.
     */
    TIM3->CR1 |= TIM_CR1_CEN;

    /* 
     * make timer stop when core is halted while debugging 
     * -> see section 31.16.3 in the manual
     */
    DBGMCU->CR |= DBGMCU_CR_DBG_TIM3_STOP;

}


/**
 * Hello world blinky program with timer hardware pwm
 *
 */
int main(void) {

    gpio_init();

    timer_init();

    while(1);

}