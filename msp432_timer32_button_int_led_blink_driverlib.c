////////////////////////////////////////////////////////////////////////////////
// Date: 2018-04-02
// Author: RSP (IoT Engineering Education @ KMUTNB)
////////////////////////////////////////////////////////////////////////////////

#include <ti/devices/msp432p4xx/driverlib/driverlib.h>

#include <stdint.h>
#include <stdbool.h>

#define PERIOD_CYCLES ( (SystemCoreClock/16)/2 - 1)

volatile bool toggle_enabled = true;
volatile bool tick = false;

void PORT1_IRQHandler(void) {
    uint32_t status;

    status = MAP_GPIO_getEnabledInterruptStatus( GPIO_PORT_P1 );
    MAP_GPIO_clearInterruptFlag( GPIO_PORT_P1, status );

    if ( status & GPIO_PIN1 ) {
        toggle_enabled = !toggle_enabled;
    }
}

void GPIO_init() {
    // LED output pin (P1.0)
    MAP_GPIO_setAsOutputPin( GPIO_PORT_P1, GPIO_PIN0 );
    MAP_GPIO_setOutputLowOnPin( GPIO_PORT_P1, GPIO_PIN0 );

    // Push button pin (P1.1)
    MAP_GPIO_setAsInputPinWithPullUpResistor( GPIO_PORT_P1, GPIO_PIN1 );
    MAP_Timer32_registerInterrupt ( GPIO_PORT_P1, PORT1_IRQHandler ); // register ISR
    MAP_GPIO_clearInterruptFlag( GPIO_PORT_P1, GPIO_PIN1 ); // clear Por1 interrupt flag
    MAP_GPIO_enableInterrupt( GPIO_PORT_P1, GPIO_PIN1 );    // enable interrupt on P1.1 pin
    MAP_Interrupt_enableInterrupt( INT_PORT1 );             // enable Port1 interrupt in NVIC
}

void T32_INT1_IRQHandler( void ){
    MAP_Timer32_clearInterruptFlag( TIMER32_0_BASE );      // acknowledge the interrupt
    MAP_Timer32_setCount( TIMER32_0_BASE, PERIOD_CYCLES ); // reload the count period
    tick = true;
}

void Timer32_init() {
    // Assume 48MHz System Clock Frequency

    MAP_Timer32_initModule(
       TIMER32_0_BASE,          // use the Timer32 module 0
       TIMER32_PRESCALER_16,    // set the prescaler (1,16,256): 16
       TIMER32_32BIT,           // use 16-bit or 32-bit mode: 32-bit
       TIMER32_FREE_RUN_MODE    // use free-running or periodic mode: free-running
    );
    MAP_Timer32_setCount ( TIMER32_0_BASE, PERIOD_CYCLES );
    MAP_Interrupt_enableInterrupt( INT_T32_INT1 ); // enable TM32_INT1 (Timer32 module 0) in NVIC
    MAP_Timer32_enableInterrupt( TIMER32_0_BASE ); // enable interrupt for Timer32 module 0
    MAP_Timer32_registerInterrupt ( TIMER32_0_INTERRUPT, T32_INT1_IRQHandler); // register ISR
    MAP_Timer32_startTimer( TIMER32_0_BASE, true );
}

int main(void) {
    MAP_WDT_A_holdTimer();        // stop WDT (watchdog timer)
    GPIO_init();                  // initialize the GPIO (LED pin)
    Timer32_init();               // initialize Timer32 module 0
    MAP_Interrupt_enableMaster(); // enable the global interrupt

    while(1)  {
      if ( tick ) {
          tick = false;
          if ( toggle_enabled ) { // toggle the LED if LED toggle is enabled
              MAP_GPIO_toggleOutputOnPin( GPIO_PORT_P1, GPIO_PIN0 );
          } else { // turn off the LED if LED toggle is disabled
              MAP_GPIO_setOutputLowOnPin( GPIO_PORT_P1, GPIO_PIN0 );
          }
      }
    }
}
////////////////////////////////////////////////////////////////////////////////
