///////////////////////////////////////////////////////////////////////////////
#include "msp.h"

void main(void) {
    uint32_t value = 0b001;

    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD; // stop watchdog timer

    P2DIR |= 0b111;      // configure the P2.0, P2.1, P2.2 pins as output
    P2OUT &= ~(0b111);   // turn off the red, green, and blue LED

    while (1) {
       P2OUT = value;    // update the P2 output according to the current value.
       _delay_cycles( 500000 );
       value = ((value << 1) | (value >> 2)) & 0b111; // rotate shift left
    }
}
///////////////////////////////////////////////////////////////////////////////
