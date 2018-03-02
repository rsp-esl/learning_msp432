// Soft-PWM: generate a 500Hz PWM waveform
//  The duty cycle can be incremented or decremented in 10% step 
//  when pressing the push button S1 and S2 respectively.

#include "msp.h"

#define LED_BIT      (1 << 6)
#define BTN1_BIT     (1 << 1)
#define BTN2_BIT     (1 << 4)

#define LOAD_VALUE (SystemCoreClock/50000 - 1)

volatile int ticks = 0;
volatile int duty_cycle = 0;

void init_led() {
    P1->DIR |= LED_BIT;     // configure P1.0 as output
}

void init_switch(void) {
    P1->DIR &= ~BTN1_BIT;    // configure P1.1 (S1 switch) as digital input
    P1->REN |=  BTN1_BIT;    // enable internal resistor P1.1
    P1->OUT |=  BTN1_BIT;    // write 1 to P1.1 to enable pull-up resistor

    P1->IES |=  BTN1_BIT;    // detect the falling edge on P1.1
    P1->IE  |=  BTN1_BIT;    // enable external interrupt on P1.1
    P1->IFG &= ~BTN1_BIT;    // clear flag to ensure no pending interrupts on P1.1

    P1->DIR &= ~BTN2_BIT;    // configure P1.4 (S2 switch) as digital input
    P1->REN |=  BTN2_BIT;    // enable internal resistor P1.4
    P1->OUT |=  BTN2_BIT;    // write 1 to P1.4 to enable pull-up resistor

    P1->IES |=  BTN2_BIT;    // detect the falling edge on P1.4
    P1->IE  |=  BTN2_BIT;    // enable external interrupt on P1.4
    P1->IFG &= ~BTN2_BIT;    // clear flag to ensure no pending interrupts on P1.4

    // call the CMSIS core functions to set priority and enable interrupt
    NVIC_EnableIRQ( PORT1_IRQn );      // enable Port 1 interrupt 
    NVIC_SetPriority( PORT1_IRQn, 4 ); // set priority to 4 for Port 1 interrupt
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

void T32_INT1_IRQHandler(void) {
    TIMER32_1->INTCLR = 0;                // clear raw interrupt flag 
    ticks = (ticks + 1) % 100;
    if ( duty_cycle == ticks ) {
       P1->OUT &= ~LED_BIT; 
    } 
    else if ( ticks == 0 ) {
       P1->OUT |= LED_BIT; 
    }
}

void PORT1_IRQHandler(void) {
    if ( P1->IFG & BTN1_BIT ) {    // check the interrupt flag for Port 1, Bit 1
       if ( duty_cycle >= 100 ) {
          duty_cycle = 0;
       } else {
          duty_cycle = duty_cycle + 10;
       }
       P1->IFG &= ~BTN1_BIT;          // clear interrupt flag on P1.1
    }
    if ( P1->IFG & BTN2_BIT ) {    // check the interrupt flag for Port 1, Bit 4
       if ( duty_cycle == 0 ) {
          duty_cycle = 100;
       } else {
          duty_cycle = duty_cycle - 10;
       }
       P1->IFG &= ~BTN2_BIT;          // clear interrupt flag on P1.4
    }
}

void main() {
    init_led();           // initialize the P1.0 pin for RED LED
    init_switch();        // initialize the P1.1 pin for input switch (push button)
    init_timer32();       // initialize the Timer32 unit
    __enable_irq();       // enable global interrupt

    while (1) {
        // empty
    }
}
