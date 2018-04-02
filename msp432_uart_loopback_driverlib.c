////////////////////////////////////////////////////////////////////////////////
// Date: 2018-04-02
// Author: RSP (IoT Engineering Education @ KMUTNB)
// MCU Board: MSP432P401R SimpleLink LaunchPad
////////////////////////////////////////////////////////////////////////////////

#include <ti/devices/msp432p4xx/driverlib/driverlib.h>

#include <stdint.h>
#include <stdbool.h>

#define EUSCI_A_MODULE  EUSCI_A0_BASE

// MSP430/432 USCI/EUSCI UART Baud Rate Calculation
// http://software-dl.ti.com/msp430/msp430_public_sw/mcu/msp430/MSP430BaudRateConverter/index.html

// Assume SystemCoreClock = 48MHz, Baudrate = 115200
eUSCI_UART_Config uartConfig = {
        EUSCI_A_UART_CLOCKSOURCE_SMCLK,          // SMCLK Clock Source
        26,                                      // BRDIV
        0,                                       // UCxBRF (First Mod Register)
        111,                                     // UCxBRS (Second Mod Register)
        EUSCI_A_UART_NO_PARITY,                  // No Parity
        EUSCI_A_UART_LSB_FIRST,                  // MSB First
        EUSCI_A_UART_ONE_STOP_BIT,               // One stop bit
        EUSCI_A_UART_MODE,                       // UART mode
        EUSCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION  // Oversampling
};


void UART_init() {
    MAP_UART_initModule( EUSCI_A_MODULE, &uartConfig );
    MAP_UART_enableModule( EUSCI_A_MODULE );
}
void GPIO_init() {
    // Selecting P1.2 and P1.3 in UART mode
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin( GPIO_PORT_P1,
             GPIO_PIN2 | GPIO_PIN3, GPIO_PRIMARY_MODULE_FUNCTION );
    // P1.1 (LED pin) as output
    MAP_GPIO_setAsOutputPin( GPIO_PORT_P1, GPIO_PIN0 );
    MAP_GPIO_setOutputLowOnPin( GPIO_PORT_P1, GPIO_PIN0 );
}

void UART_sendString (const char *str ) {
    const char *p = &str[0];
    while ( *p != '\0' ) {
       MAP_UART_transmitData( EUSCI_A_MODULE, *p++ );
    }
}

int main(void) {
    uint8_t mask, flags, data;

    MAP_WDT_A_holdTimer();          // disable WDT (watchdog timer)
    GPIO_init();                    // initialize GPIO
    UART_init();                    // initialize EUSCI_A

    mask = EUSCI_A_UART_FRAMING_ERROR | EUSCI_A_UART_OVERRUN_ERROR |
           EUSCI_A_UART_PARITY_ERROR | EUSCI_A_UART_BREAK_DETECT |
           EUSCI_A_UART_RECEIVE_ERROR;

    UART_sendString( "MSP432P401 LaunchPad...\r\n" );

    while(1)  {
        data = MAP_UART_receiveData( EUSCI_A_MODULE );  // (blocking) wait for incoming byte
        flags = MAP_UART_queryStatusFlags( EUSCI_A0_BASE, mask );
        if (!flags) { // no errors
            MAP_UART_transmitData( EUSCI_A_MODULE, data ); // send the data bytes back (loopback)
        } else {
            UART_sendString( "UART Receive Error!\r\n" );
        }
        MAP_GPIO_toggleOutputOnPin( GPIO_PORT_P1, GPIO_PIN0 );  // toggle the LED
    }
}
////////////////////////////////////////////////////////////////////////////////
