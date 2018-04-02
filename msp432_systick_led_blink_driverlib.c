////////////////////////////////////////////////////////////////////////////////
// Date: 2018-04-02
// Author: RSP (IoT Engineering Education @ KMUTNB)
// MCU Board: MSP432P401R SimpleLink LaunchPad
////////////////////////////////////////////////////////////////////////////////

#include <ti/devices/msp432p4xx/driverlib/driverlib.h>

#include <stdint.h>
#include <stdbool.h>

volatile uint32_t ticks = 0;

#define COUNT_PERIOD  (SystemCoreClock/1000)

void GPIO_init() {
   MAP_GPIO_setAsOutputPin( GPIO_PORT_P1, GPIO_PIN0 );
   MAP_GPIO_setOutputLowOnPin( GPIO_PORT_P1, GPIO_PIN0 );
}

void SysTick_init() {
    // Assume 12 MHz System Core Clock
    MAP_SysTick_enableModule();
    MAP_SysTick_setPeriod( COUNT_PERIOD - 1 ); // set 24-bit value (max. value = 2^24-1)
    MAP_SysTick_enableInterrupt();
}

int main(void) {
    uint32_t ts;

    MAP_WDT_A_holdTimer();            // disable WDT (watchdog timer)
    MAP_Interrupt_disableMaster();    // disable the global interrupt
    GPIO_init();
    SysTick_init();
    MAP_Interrupt_enableMaster();     // enable the global interrupt

    ts = ticks;
    while (1) {
        if ( ticks - ts >= 500 ) {
            ts += 500;
            MAP_GPIO_toggleOutputOnPin( GPIO_PORT_P1, GPIO_PIN0 );
        }
    }
}

void SysTick_Handler(void) {
    ticks++;
}

