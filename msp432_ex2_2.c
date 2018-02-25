///////////////////////////////////////////////////////////////////////////////
#include "msp.h"

/* Port1 ISR (Interrupt Service Routine) */
void PORT1_IRQHandler(void) {
    // Toggling the output on the LED
    if (P1->IFG & BIT1) {
       P1->OUT ^= BIT0;     // toggle the LED output
    }
    __delay_cycles(10000);  // delay for switch debounce
    P1->IFG &= ~BIT1;       // clear interrupt flag on P1.1
}

void main(void) {

    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD; // stop watchdog timer

    P1->DIR |= BIT0;    // configure P1.0 (LED1) as digital output
    P1->OUT &= ~BIT0;   // clear bit 0 on P1OUT (turn off LED1)

    P1->DIR &= ~BIT1;   // configure P1.1 (S1 switch) as digital input
    P1->REN |= BIT1;    // enable internal resistor P1.1
    P1->OUT |= BIT1;    // write 1 to P1.1 to enable pull-up resistor

    P1->IES |= BIT1;    // detect the falling edge on P1.1
    P1->IE  |= BIT1;    // enable external interrupt on P1.1
    P1->IFG &= ~BIT1;   // clear flag to ensure no pending interrupts on P1.1

    // enable Port 1 interrupt 
#if 0 
    NVIC->ISER[1] = 1 << ((PORT1_IRQn) & 0b11111);  // register access
#else
    NVIC_EnableIRQ( PORT1_IRQn );                   // call a function
#endif

    __enable_interrupt();           // enable the global interrupt

    while (1) {
        // empty
    }
}
///////////////////////////////////////////////////////////////////////////////
