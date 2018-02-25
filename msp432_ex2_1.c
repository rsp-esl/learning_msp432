///////////////////////////////////////////////////////////////////////////
#include "msp.h"

#define CNT_MAX  (100)

void main(void) {
    uint32_t cnt = 0;   // count variable

    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD; // stop watchdog timer

    P1->DIR |= BIT0;    // configure P1.0 (LED1) as digital output
    P1->OUT &= ~BIT0;   // clear bit 0 on P1OUT

    P1->DIR &= ~BIT1;   // configure P1.1 (Push Switch S1) as digital input
    P1->REN |= BIT1;    // enable internal resistor P1.1
    P1->OUT |= BIT1;    // write 1 to P1.1 to enable pull-up resistor

    while (1) {
        while ( !(P1->IN & BIT1) ) { // poling check: if button is pressed
            if ( cnt < CNT_MAX ) {
                cnt++;
            }
            __delay_cycles(1000); 
        }
        if ( cnt >= CNT_MAX ) {
           P1->OUT ^= BIT0;  // toggle the LED1 output
        }
        cnt = 0;   // reset the count variable
    }
}

///////////////////////////////////////////////////////////////////////////
