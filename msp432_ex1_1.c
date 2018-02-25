//////////////////////////////////////////////////////////////////////////////
#include "msp.h"

#define LED_BIT       (1 << 0)   // Port P1 Bit 0
#define DELAY_CYCLES  (3000000/2) // assume that the system clock freq. is 3MHz

void main(void) {

    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD; // stop watchdog timer

    // set up bit 0 of P1 as output
    P1DIR |= LED_BIT;
    
    // intialize bit 0 of P1 to 0
    P1OUT &= ~LED_BIT;

    while (1) {
       P1OUT ^= LED_BIT;                 // toggle the LED output
       __delay_cycles( DELAY_CYCLES );   // delay approx. 0.5 sec (@3MHz clock)
    }
}
//////////////////////////////////////////////////////////////////////////////
