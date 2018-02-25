///////////////////////////////////////////////////////////////////////////////
#include "msp.h"

volatile uint32_t update = 0;
volatile uint32_t rotation_dir = 1, running = 0;  

/* Port1 ISR */
void PORT1_IRQHandler(void) {
    if (P1->IFG & BIT1) {        // P1.1 interrupt flag detected
       rotation_dir ^= 1;        // toggle rotation direction mode
       P1->IE  &= ~BIT1;         // disable interrupt on P1.1
       update = 1;               // set update flag
    }
    if (P1->IFG & BIT4) {        // P1.4 interrupt flag detected
       running ^= 1;             // toggle the running/pause mode  
       P1->IE  &= ~BIT4;         // disable interrupt on P1.4
       update = 1;               // set update flag
    }
    P1->IFG &= ~(BIT4 | BIT1);   // clear interrupt flag on P1.1 & P1.4
}

void main(void) {
    uint32_t value = 0b001;
    uint32_t cnt = 0;

    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD; // stop watchdog timer

    P2->SEL1 &= ~(BIT2 | BIT1 | BIT0);  // configure P2.2-P2.0 as GPIO 
    P2->SEL0 &= ~(BIT2 | BIT1 | BIT0);
    P2->DIR |= (BIT2| BIT1 | BIT0);     // configure P2.0, P2.1, P2.2 as output

    P1->SEL1 &= ~(BIT4 | BIT1);    // configure P1.1 and P1.4 as GPIO 
    P1->SEL0 &= ~(BIT4 | BIT1); 
    P1->DIR &= ~(BIT4 | BIT1);     // configure P1.1 and P1.4 (S1 & S2 switch) as input
    P1->REN |= (BIT4 | BIT1);      // enable internal resistor on P1.1 and P1.4
    P1->OUT |= (BIT4 | BIT1);      // enable pull-up resistor on P1.1 and P1.4

    P1->IES |= (BIT4 | BIT1);      // detect the falling edge on P1.1 and P1.4
    P1->IE  |= (BIT4 | BIT1);      // enable external interrupt on P1.1 and P1.4
    P1->IFG &= ~(BIT4 | BIT1);     // clear flag to ensure no pending interrupts 

    NVIC_SetPriority( PORT1_IRQn, 3) ;  // set priority to 3 in NVIC
    NVIC_EnableIRQ( PORT1_IRQn );       // enable Port 1 interrupt 
    __enable_interrupt();               // enable the global interrupt

    while (1) {
        if ( update ) {
           update = 0;              // clear update flag
           P1->IE |= (BIT4 | BIT1); // re-enable interrrupt on P1.1 and P1.4
        }
        if ( running && cnt == 0 ) {
           if ( rotation_dir ) { // R -> G -> B -> ...
              value = ((value << 1) | (value >> 2)) & 0b111; // rotate shift left
           } else { // R -> B -> G -> ...
              value = ((value << 2) | (value >> 1)) & 0b111; // rotate shift right
           }
           P2->OUT &= ~0b111;
           P2->OUT |= value; // update RGB LED output bits
        } 
        __delay_cycles( 100000 );
        cnt = (cnt+1) % 20;
    }
}
///////////////////////////////////////////////////////////////////////////////

