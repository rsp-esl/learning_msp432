////////////////////////////////////////////////////////////////////////////////
// Date: 2018-04-02
// Author: RSP (IoT Engineering Education @ KMUTNB)
// MCU Board: MSP432P401R SimpleLink LaunchPad
////////////////////////////////////////////////////////////////////////////////

#include <ti/devices/msp432p4xx/driverlib/driverlib.h>

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#define EUSCI_A_MODULE  EUSCI_A0_BASE            // use eUSCI_A0 for UART

// Please see: MSP430/432 USCI/EUSCI UART Baud Rate Calculation
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

void UART_init() { // configure eUSCI_A0
    MAP_UART_initModule( EUSCI_A_MODULE, &uartConfig ); 
    MAP_UART_enableModule( EUSCI_A_MODULE );

    // Configure the P1.2/RxD and P1.3/TxD pins for UART operation
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin( GPIO_PORT_P1,
             GPIO_PIN2, GPIO_PRIMARY_MODULE_FUNCTION );
    MAP_GPIO_setAsPeripheralModuleFunctionOutputPin( GPIO_PORT_P1,
             GPIO_PIN3, GPIO_PRIMARY_MODULE_FUNCTION );
}

void GPIO_init() { // configure P1.0 (LED pin) as output
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
    MAP_WDT_A_holdTimer();          // disable WDT (watchdog timer)
    GPIO_init();                    // initialize GPIO for LED
    UART_init();                    // initialize EUSCI_A0

    UART_sendString( "\r\nMSP432P401 LaunchPad: UART testing...\r\n" );

    while(1)  { // polling loop
        char ch = MAP_UART_receiveData( EUSCI_A_MODULE );       // wait for the next incoming data byte
        MAP_UART_transmitData( EUSCI_A_MODULE, ch );            // send the data byte back
        MAP_GPIO_toggleOutputOnPin( GPIO_PORT_P1, GPIO_PIN0 );  // toggle the LED
    }
}
////////////////////////////////////////////////////////////////////////////////
