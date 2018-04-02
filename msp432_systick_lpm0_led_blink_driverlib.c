////////////////////////////////////////////////////////////////////////////////
// Date: 2018-04-02
// Author: RSP (IoT Engineering Education @ KMUTNB)
// MCU Board: MSP432P401R SimpleLink LaunchPad
////////////////////////////////////////////////////////////////////////////////

#include <ti/devices/msp432p4xx/driverlib/driverlib.h>

#include <stdint.h>
#include <stdbool.h>

void GPIO_init() {
   MAP_GPIO_setAsOutputPin( GPIO_PORT_P1, GPIO_PIN0 );
   MAP_GPIO_setOutputLowOnPin( GPIO_PORT_P1, GPIO_PIN0 );

   MAP_GPIO_setAsOutputPin( GPIO_PORT_P2, GPIO_PIN0 );
   MAP_GPIO_setOutputLowOnPin( GPIO_PORT_P2, GPIO_PIN0 );
}

void SysTick_init() {
    // Assume 12 MHz System Core Clock
    MAP_SysTick_enableModule();
    MAP_SysTick_setPeriod( SystemCoreClock/2-1 ); // set 24-bit value (max. value = 2^24-1)
    MAP_Interrupt_enableSleepOnIsrExit();
    MAP_SysTick_enableInterrupt();
}

int main(void) {
    MAP_WDT_A_holdTimer();            // disable WDT (watchdog timer)
    MAP_Interrupt_disableMaster();    // disable the global interrupt
    GPIO_init();
    SysTick_init();
    MAP_Interrupt_enableMaster();     // enable the global interrupt

    //MAP_PCM_gotoLPM0();               // enter low-power mode (LPM0)

    while (1) {
          MAP_GPIO_toggleOutputOnPin( GPIO_PORT_P2, GPIO_PIN0 );
    }
}

void SysTick_Handler(void) {
    MAP_GPIO_toggleOutputOnPin( GPIO_PORT_P1, GPIO_PIN0 );
}
////////////////////////////////////////////////////////////////////////////////
