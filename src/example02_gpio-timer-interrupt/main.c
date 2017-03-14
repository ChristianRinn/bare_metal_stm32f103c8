#include <stm32f1xx.h>


/*
 * The red LED is connected to port PC13,
 * -> see schematic or pinout of "blue pill" board
 */
#define LED_GPIO        GPIOC
#define LED_PIN         13

void gpio_init(void) {

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
}

void gpio_toggle(void) {

    /*
     * Toggle the LED by xor'ing the equivalent bit in the output data register of
     * the GPIO port. 
     * 
     * -> see section 9.2.4 in the manual
     */

    LED_GPIO->ODR ^= (1 << LED_PIN);
}


void timer_init(void) {

    /*
     * Let's take a "general-purpose timer" for the example since they are the most basic timer
     * we have with the STM32F103C8. We'll take timer 2.
     */

    /* enable the clock to timer 2 in the APB1 enable register */
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

    /*
     * Timer 2 runs from the APB1 clock which is SCLK/2, i.e. 36 MHz.
     * So if we divide the APB1 clock by 3600 our timer runs at a tick rate
     * of 10 kHz. So we write 3599 in the prescaler register of the timer.
     * Why 3599 and not 3600? Because a value of 0 in the prescaler does 
     * (fortunately) not divide by zero but by one.
     */ 
    TIM2->PSC = 3600 -1; // clock = 36.000.000 Hz -> CK_PSK = 10.000 Hz

    /* count to 5000 */
    TIM2->ARR = 5000 -1; // 10.000 Hz / 5000 = 2 Hz

    /* 
     * Enable the counter by setting the counter enable bit in the control register
     * of timer 2.
     */
    TIM2->CR1 |= TIM_CR1_CEN;

    /* enable interrupt for timer 6 */
    NVIC_EnableIRQ(TIM2_IRQn);

    /* enable update interrupt */
    TIM2->DIER |= TIM_DIER_UIE; 

    /* 
     * make timer stop when core is halted while debugging 
     * -> see section 31.16.3 in the manual
     */
    DBGMCU->CR |= DBGMCU_CR_DBG_TIM2_STOP;

}

void TIM2_IRQHandler(void) {

    /* clear interrupt flag or the timer will keep firing interrupts forever */
    TIM2->SR &= ~(TIM_SR_UIF);

    /* do something in the interrupt handler */
    gpio_toggle();
}

/**
 * Hello world blinky program with timer interrupt
 *
 */
int main(void) {

    gpio_init();

    timer_init();

    while(1);

}