//////////////////////////////////////////////////////////////////////////////
// Soft PWM technique

#include "msp.h"

#define LED_BIT     (1 << 0)  // Port P1 Bit 0
#define SW1_BIT     (1 << 1)  // Port P1 Bit 1

volatile uint32_t ticks = 0;
volatile uint32_t duty_cycles = 0;

void init_sw1() {
    P1->DIR &= ~SW1_BIT;   // configure P1.1 (Push Switch S1) as digital input
    P1->REN |= SW1_BIT;    // enable internal resistor P1.1
    P1->OUT |= SW1_BIT;    // write 1 to P1.1 to enable pull-up resistor

    P1->IES |= SW1_BIT;    // detect the falling edge on P1.1
    P1->IE  |= SW1_BIT;    // enable external interrupt on P1.1
    P1->IFG &= ~SW1_BIT;   // clear flag to ensure no pending interrupts on P1.1

    // enable Port 1 interrupt 
    NVIC_EnableIRQ( PORT1_IRQn ); 
}

void init_led1() {
    // set up bit 0 of P1 as output
    P1->DIR |= LED_BIT;

    // intialize bit 0 of P1 to 0
    P1->OUT &= ~LED_BIT;
}

void init_systick() {
    SysTick->LOAD = 150-1;   // set the reload value (assume 3MHz core clock)
    SysTick->CTRL = 0b111;   //    ENABLE bit=1: enable SysTick interrupt
                             //   TICKINT bit=1: enable interrupt
                             // CLKSOURCE bit=1: use the core clock 

    NVIC_SetPriority( SysTick_IRQn, 2 ); // set priority to 2 for SysTick 
}

void main(void) {
    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD; // stop watchdog timer

    init_led1();       // initialize the LED1 output pin
    init_sw1();        // initialize the S1 switch pin
    init_systick();    // initialize the SysTick
    __enable_irq();    // enable global interrupt 

    while (1) {
       if ( (ticks%100) < duty_cycles ) {
           P1->OUT |= LED_BIT;
       }
       else {
           P1->OUT &= ~LED_BIT;
       }
    }
}

void PORT1_IRQHandler(void) {
    if ( P1->IFG & SW1_BIT ) { // check the interrupt on P1.1
       duty_cycles = (duty_cycles+10) % 100;
    }
    P1->IFG &= ~SW1_BIT;       // clear interrupt flag on P1.1
}

void SysTick_Handler(void) {
    ticks++;
}
//////////////////////////////////////////////////////////////////////////////
