#include "msp.h"
#include <string.h>
#include <stdlib.h>

#define LOAD_VALUE   (SystemCoreClock/1000 - 1)

#define BAUDRATE     (115200)
#define BRW_VALUE    (SystemCoreClock/BAUDRATE)

void init_uart(void) {
    P1->SEL0 |=  0x0C;             // config P1.3 and P1.2 for EUSCI_A0 (UART)
    P1->SEL1 &= ~0x0C;
    EUSCI_A0->CTLW0 |= 1;          // disable EUSCI_A0
    EUSCI_A0->MCTLW = 0;           // disable oversampling 
    EUSCI_A0->CTLW0 = 0x0081;      // 1 stop bit, no parity, SMCLK, 8-bit data 
    EUSCI_A0->BRW = BRW_VALUE;     // set baudrate
    EUSCI_A0->CTLW0 &= ~1;         // enable EUSCI_A0
}

void send_str( const char *str ) {
    int i, len = strlen(str);
    for ( i=0; i < len; i++ ) {
        while( !(EUSCI_A0->IFG & UCTXIFG) ) ; // wait until TX buffer is ready
        EUSCI_A0->TXBUF = str[i];
    }
}

void init_timer32() {
    TIMER32_1->CONTROL &= ~(1 << 7);     // disable Timer32
    TIMER32_1->CONTROL = (1 << 6)        // 0=free-running mode, 1=periodic mode
                       | (1 << 5)        // 1=enable interrupt for the timer
                       | (0 << 2)        // prescale: 0=/1, 1=/16, 2=/256
                       | (1 << 1)        // 0=16-bit, 1=32-bit mode
                       | (0 << 0);       // 0=wrapping mode, 1=one-shot mode

    TIMER32_1->LOAD = LOAD_VALUE;        // set the reload value for Timer32 
    TIMER32_1->CONTROL |= (1  << 7);     // enable Timer32

    // call the CMSIS core functions to set priority and enable interrupt
    NVIC_SetPriority( T32_INT1_IRQn, 3 );   // set priority level
    NVIC_EnableIRQ( T32_INT1_IRQn );        // enable interrupt in NVIC 
}

volatile uint32_t ticks_1msec = 0;

void T32_INT1_IRQHandler(void) {
    TIMER32_1->INTCLR = 0;                // clear raw interrupt flag 
    ticks_1msec++;
}

inline uint32_t millis( void ) {
    return ticks_1msec;
}

void main(void) {
    uint32_t count = 0;
    uint32_t ts;
    char buf[32];

    init_timer32();   // initialize the timer (TIMER32_1)
    init_uart();      // initalize the UART (EUSCI_A0)
    
    send_str( "MSP432P401R LaunchPad Demo..\r\n");
    ts = millis();
    while (1) {
        if ( millis() - ts >= 1000 ) { // send a message every 1 sec
            ts += 1000;
            sprintf( buf, "Count: %u\r\n", count++ );
            send_str( buf );
        }
    }
}
