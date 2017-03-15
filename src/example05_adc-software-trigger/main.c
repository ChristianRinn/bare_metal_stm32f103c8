#include <stm32f1xx.h>


/* 
 * In this example we will make use of the ADC. The principle is the one of a simple servo tester,
 * i.e. we measure the voltage at a potentiometer and map that to a pulse between 1ms and 2ms, while
 * the period of the pwm is 20ms (50Hz). In this particular example PA3 is used as the ADC input pin
 * and PA1 is used as the pwm output (TIM2CH2).
 * 
 * For an overview which pins can be used as ADC input pins, please have a look at the datasheet. 
 * If you plan to use a trigger of a certain timer capture/compare channel, have a look at
 * section 11.7 ("Conversion on external trigger") in the manual.
 * 
 * In this example we will trigger from software though and thus any ADC input pin can be used.
 * 
 */

void gpio_init(void) {

    /*
     * Turn on the GPIOA,
     * -> see section 7.3.7 in the manual
     */
    RCC->APB2ENR  |= RCC_APB2ENR_IOPAEN;


    /*
     * Clear configuration and mode bits of PA3 (ADC) and PA1 (PWM)
     * 
     * Set adc pin mode bits to 0b00 -> analog mode
     * Set adc pin configuration bits to 0b00 -> input
     * 
     * Set timer pin mode bits to 0b01 -> output mode max. speed 2 MHz
     * Set timer pin configuration bits to 0b10 -> Alternate function push-pull
     * 
     * -> see section 9.2.2 in the manual
     */

    /* Just clear the bits for PA3 (ADC) */
    GPIOA->CRL = (GPIOA->CRL & (~(GPIO_CRL_CNF3 | GPIO_CRL_MODE3)));

    /* Set appropriate bits for PA1 (PWM) */
    GPIOA->CRL = ((GPIOA->CRL & (~(GPIO_CRL_CNF1 | GPIO_CRL_MODE1))) 
                    | GPIO_CRL_MODE1_0
                    | GPIO_CRL_CNF1_1);
}

void adc_init(void) {
    
    /* set ADC clock divider (72MHz / 6 = 12MHz) */
    RCC->CFGR |= RCC_CFGR_ADCPRE_DIV6;

    /* start the ADC clock */
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

    /*
     * Set sample time of ADC channel 3 to 239.5 cycles, resulting in a total conversion 
     * time of 252 cycles / 12 MHz = 21 µs. This is optional and could be left in the reset
     * state (1.5 cycles sample time -> 14 cycles total conversion time) as well.
     * 
     * -> see section 11.6 and 11.12.5 in the manual
     */
    ADC1->SMPR2 |= ADC_SMPR2_SMP3;

    /*
     * We convert with a "regular sequence" which is triggered by software. Though we have to
     * set the channel we want to convert in the "regular sequence register" by writing the 
     * channel number into the SQx[4:0] bit field. Since we only want to sample one channel 
     * we can leave the number of conversions of the "regular sequence" to the reset state
     * which is one conversion (L[3:0] in ADC_SQR1).
     * 
     * -> see section 11.12.9 and 11.12.11 in the manual
     */
    ADC1->SQR3 |= 3U;

    /*  
     * Turn on the ADC by setting the ADON flag in the CR2 register of ADC1. 
     * -> see section 11.3.1 in the manual
     * 
     * You can start a single conversion by setting this bit again, from any function
     * or interrupt handler, which we will do in this example (for the sake of simplicity).
     */
    ADC1->CR2 |= ADC_CR2_ADON;

    /*
     * In order to be able to trigger from software, we need to set the EXTSEL[2:0] bits to 0b111 (SWSTART).
     * 
     * -> see section 11.7 and 11.12.3 in the manual
     */
    ADC1->CR2 |= ADC_CR2_EXTSEL;

    /*
     * Wait some time for the ADC to get ready. The manual says at least two ADC clock cycles,
     * i.e. 12 system clock cycles. Let's give it 42 ;)
     */
    volatile int i;
    for (i=0; i<42; i++) {
        asm("nop");
    }

    /*
     * Calibrate the ADC
     * -> see section 11.4 in the manual
     */
    ADC1->CR2 |= ADC_CR2_CAL;

    /* wait for calibration to complete */
    while(ADC1->CR2 & ADC_CR2_CAL);

    /* actually start the first conversion */
    ADC1->CR2 |= ADC_CR2_ADON;

}


/* 
 * As stated in section 7.2 (Clocks) in the manual,
 * "the ADCs are clocked by the clock of the High Speed domain (APB2) divided by 2, 4, 6 or 8."
 * In the ADC introduction however it says, that the ADC clock must not exceed 14MHz,
 * so if we run with a system clock of 72MHz and APB2 is 72MHz as well, we have to use at least
 * a prescaler of 6, resulting in an ADC clock of 12MHz.
 */

void timer_init(void) {

    /*
     * In this example we have to use at least a "general purpose timer" because
     * a "basic timer" has no capture/compare capabilities. 
     */

    /* enable the clock to timer 2 in the APB2 enable register */
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;

    /*
     * Timer 2 runs from the APB1 clock multiplied by 2 which is SCLK, i.e. 72 MHz.
     * So we have to divide the APB2 clock by 72 to get a tick rate
     * of 1 MHz. 
     */ 
    TIM2->PSC = 72 -1; // timer tick rate = 72.000.000 Hz / (PSC+1) = 1 MHz

    /* 
     * Set the value to count up to / to count down from.
     * This sets the PWM period to 20 ms (= 2000/1000000Hz)
     */
    TIM2->ARR = 20000 -1; 



    /*
     * Set PWM mode 1
     * 
     * -> see section 14.4.7 in the manual
     */
    TIM2->CCMR1 |= (TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1 );


    /* set the initial capture compare register to 1500 µs */
    TIM2->CCR2 = 1500 -1;

    /* 
     * Enable TIM2 CH2
     * 
     * -> see section 15.4.9 in the manual
     */
    TIM2->CCER |= TIM_CCER_CC2E;

    /*
     * Set the "capture/compare interrupt enable" flag in the "DMA and interrupt enable" register.
     * This is done in order to attach an interrupt to a compare event, in which we will trigger the ADC.
     * Obviously this is not very efficient but it's simple.
     */
    TIM2->DIER |= TIM_DIER_CC2IE;

    /* enable interrupt for timer 2 */
    NVIC_EnableIRQ(TIM2_IRQn);

    /* 
     * Enable the counter by setting the counter enable bit in the control register
     * of timer 2.
     */
    TIM2->CR1 |= TIM_CR1_CEN;



    /* 
     * make timer stop when core is halted while debugging 
     * -> see section 31.16.3 in the manual
     */
    DBGMCU->CR |= DBGMCU_CR_DBG_TIM2_STOP;

}

void adc_to_pwm(void) {
    /*
     * Read the ADC sample
     */
    uint16_t adc_val = ADC1->DR & 0xFFFF;

    /*
     * We have a 12 bit ADC, so the maximum value is 4095. Now we want to map this
     * range from 0 to 4095 to a range ~1000 to ~2000. To do so we divide the ADC sample
     * by four and add 1024 so the resulting pulse width is between 1024 and 2048.
     */
    uint16_t pulsewidth = (adc_val >> 2) + 1024;

    /* write the pulsewidth to the CCR of TIM2 */
    TIM2->CCR2 = pulsewidth;

    /* start next ADC conversion */
    ADC1->CR2 |= ADC_CR2_ADON;
}


void TIM2_IRQHandler(void) {

    /* clear interrupt flag or the timer will keep firing interrupts forever */
    TIM2->SR &= ~(TIM_SR_CC2IF);

    /*
     * Update CCR2 of TIM2 from ADC value 
     * This function could be called from anywhere in your program, not just from this interrupt handler.
     */
    adc_to_pwm();
}


/**
 * Simple servo tester, connect potentiometer to PA0 and servo/logic analyzer/oscilloscope to PA1.
 */
int main(void) {

    gpio_init();

    adc_init();

    timer_init();

    while(1);
}

