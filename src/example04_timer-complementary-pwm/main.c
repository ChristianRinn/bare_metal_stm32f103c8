#include <stm32f1xx.h>


/* 
 * This is an example of complementary PWM output with an "advanced timer". 
 * We will use the channel 1 of timer 1 and the pins PA8 for CH1 and PB13 for CH1N.
 */

void gpio_init(void) {

    /*
     * Turn on the GPIOA and GPIOB unit,
     * -> see section 7.3.7 in the manual
     */
    RCC->APB2ENR  |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN ;


    /*
     * Clear configuration and mode bits of PA8 and PB13
     * Set timer pin mode bits to 0b01 -> output mode max. speed 2 MHz
     * Set configuration bits to 0b10 -> Alternate function push-pull
     * 
     * -> see section 9.2.2 in the manual
     */
    GPIOA->CRH = ((GPIOA->CRH & (~(GPIO_CRH_CNF8 | GPIO_CRH_MODE8))) 
                    | GPIO_CRH_MODE8_0
                    | GPIO_CRH_CNF8_1);

    /*
     * Repeat procedure to configure PB13. Could be done in one step.
     */
    GPIOB->CRH = ((GPIOB->CRH & (~(GPIO_CRH_CNF13 | GPIO_CRH_MODE13))) 
                    | GPIO_CRH_MODE13_0
                    | GPIO_CRH_CNF13_1);
}



void timer_init(void) {

    /*
     * In this example we have to use at least a "general purpose timer" because
     * a "basic timer" has no capture/compare capabilities. 
     */

    /* enable the clock to timer 1 in the APB2 enable register */
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
    RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;

    /*
     * Timer 1 runs from the APB2 clock which is SCLK, i.e. 72 MHz.
     * So we have to divide the APB2 clock by 7200 to get a tick rate
     * of 10 kHz. 
     */ 
    TIM1->PSC = 7200 -1; // timer tick rate = 72.000.000 Hz / (PSC+1) = 10000 Hz

    /* 
     * Set the value to count up to / to count down from.
     * This sets the PWM period to 0.2 seconds (= 2000/10000Hz)
     */
    TIM1->ARR = 2000 -1; 



    /*
     * Set PWM mode 1
     * 
     * -> see section 14.4.7 in the manual
     */
    TIM1->CCMR1 |= (TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1 );


    /* set the capture compare register to 75% duty cycle for channel 1*/
    TIM1->CCR1 = 1500 -1;

    /* 
     * Enable TIM1 CH1 and TIM1 CH1N
     * 
     * -> see section 14.4.9 in the manual
     */
    TIM1->CCER |= TIM_CCER_CC1E | TIM_CCER_CC1NE;

    /*
     * Set "main output enable" bit in the "break and dead-time" register of timer 1
     * Otherwise you will not see anything at the output.
     */
    TIM1->BDTR |= TIM_BDTR_MOE;

    /* 
     * Enable the counter by setting the counter enable bit in the control register
     * of timer 2.
     */
    TIM1->CR1 |= TIM_CR1_CEN;

    /* 
     * make timer stop when core is halted while debugging 
     * -> see section 31.16.3 in the manual
     */
    DBGMCU->CR |= DBGMCU_CR_DBG_TIM1_STOP;

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