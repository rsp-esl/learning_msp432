//////////////////////////////////////////////////////////////////////////////
#include "msp.h"

#define LED_BIT     (1 << 0)  // Port P1 Bit 0
#define LOAD_VALUE  (SystemCoreClock/2) // assume that the core clock freq. is 3MHz

void init_led1() {
    // set up bit 0 of P1 as output
    P1->DIR |= LED_BIT;

    // intialize bit 0 of P1 to 0
    P1->OUT &= ~LED_BIT;
}

void init_systick() {
    SysTick->LOAD = LOAD_VALUE-1;   // set the reload value
    SysTick->CTRL = 0b111;          //    ENABLE bit=1: enable SysTick interrupt
                                    //   TICKINT bit=1: enable interrupt
                                    // CLKSOURCE bit=1: use the core clock 

    NVIC_SetPriority( SysTick_IRQn, 2 ); // set priority to 2 for SysTick 
}

void main(void) {
    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD; // stop watchdog timer

    init_led1();       // initialize the LED1 output pin
    init_systick();    // initialize the SysTick
    __enable_irq();    // enable global interrupt 
    
    while (1) {
       // empty
    }
}

void SysTick_Handler(void) {
    P1->OUT ^= BIT0;    // toggle the LED1 
}
//////////////////////////////////////////////////////////////////////////////
