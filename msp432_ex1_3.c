//////////////////////////////////////////////////////////////////////////////
// LED blink using SysTick timer

#include "msp.h"

#define LED_BIT                (1 << 0)  // Port P1 Bit 0
#define SYSTICK_COUNTFLAG_BIT  (1<<16)
#define INTERVAL_MSEC          (500)

volatile uint32_t msec_ticks = 0;

void init_led1() {
    // set up bit 0 of P1 as output
    P1->DIR |= LED_BIT;

    // intialize bit 0 of P1 to 0
    P1->OUT &= ~LED_BIT;
}

void init_systick() {
    SysTick->LOAD = 3000-1;         // set the reload value (for 3MHz core clock)
    SysTick->CTRL = 0b111;          //    ENABLE bit=1: enable SysTick interrupt
                                    //   TICKINT bit=1: enable interrupt
                                    // CLKSOURCE bit=1: use the core clock 

    NVIC_SetPriority( SysTick_IRQn, 2 ); // set priority to 2 for SysTick 
}

void main(void) {
    uint32_t ts;
    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD; // stop watchdog timer

    init_led1();       // initialize the LED1 output pin
    init_systick();    // initialize the SysTick
    __enable_irq();    // enable global interrupt 
    
    ts = msec_ticks;
    while (1) {
       if ( msec_ticks - ts >= INTERVAL_MSEC ) {
          ts += INTERVAL_MSEC;
          P1->OUT ^= LED_BIT;          
       }
    }
}

void SysTick_Handler(void) {
    if ( SysTick->CTRL & SYSTICK_COUNTFLAG_BIT ) 
       msec_ticks++;
}
//////////////////////////////////////////////////////////////////////////////
