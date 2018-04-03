////////////////////////////////////////////////////////////////////////////////
// Date: 2018-04-02
// Author: RSP (IoT Engineering Education @ KMUTNB)
// MCU Board: MSP432P401R LaunchPad
////////////////////////////////////////////////////////////////////////////////

#include <ti/devices/msp432p4xx/driverlib/driverlib.h>

#include <stdint.h>
#include <stdbool.h>

void GPIO_init() {
    // LED output pin (P1.0)
    MAP_GPIO_setAsOutputPin( GPIO_PORT_P1, GPIO_PIN0 );
    MAP_GPIO_setOutputLowOnPin( GPIO_PORT_P1, GPIO_PIN0 );
}

void Timer32_init() {
    // Assume 12MHz System Clock Frequency
    MAP_Timer32_initModule(
       TIMER32_0_BASE,          // use the Timer32 module 0
       TIMER32_PRESCALER_256,   // set the prescaler (1,16,256): 16
       TIMER32_16BIT,           // use 16-bit or 32-bit mode: 16-bit
       TIMER32_FREE_RUN_MODE    // use free-running or periodic mode: free-running
    );
    MAP_Timer32_startTimer( TIMER32_0_BASE, false ); // continuous mode
}

int main(void) {
    uint32_t ts, new_ts;

    MAP_WDT_A_holdTimer();        // stop WDT (watchdog timer)
    GPIO_init();                  // initialize the GPIO (LED pin)
    Timer32_init();               // initialize Timer32 module 0

    ts = MAP_Timer32_getValue(TIMER32_0_BASE);
    while(1)  {
        new_ts = MAP_Timer32_getValue(TIMER32_0_BASE);
        if ( ((uint16_t)(ts - new_ts)) >= 23437 ) {
            ts = new_ts;
            MAP_GPIO_toggleOutputOnPin( GPIO_PORT_P1, GPIO_PIN0 ); // toggle the LED
        }
    }
}
////////////////////////////////////////////////////////////////////////////////
