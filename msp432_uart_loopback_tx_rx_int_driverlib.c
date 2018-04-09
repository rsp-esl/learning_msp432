////////////////////////////////////////////////////////////////////////////////
// Date: 2018-04-02
// Author: RSP (IoT Engineering Education @ KMUTNB)
// MCU Board: MSP432P401R SimpleLink LaunchPad
////////////////////////////////////////////////////////////////////////////////

#include <ti/devices/msp432p4xx/driverlib/driverlib.h>

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#define EUSCI_A_MODULE    EUSCI_A0_BASE   // use the eUSCI_A0 for UART

#define BUF_SIZE          (64)

volatile bool line_complete = false;
volatile uint8_t buf_index = 0;
volatile char buf[ BUF_SIZE+1 ];
volatile bool tx_busy = false;
volatile const char *tx_str_ptr = NULL;

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

void UART_init() {
    MAP_UART_initModule( EUSCI_A_MODULE, &uartConfig );
    MAP_UART_enableModule( EUSCI_A_MODULE );

    // enable RX interrupt on EUSCI_A module
    MAP_UART_enableInterrupt( EUSCI_A_MODULE, EUSCI_A_UART_RECEIVE_INTERRUPT );

    // enable EUSCI_A interrupt
    MAP_Interrupt_enableInterrupt( INT_EUSCIA0 );

    // Selecting P1.2/RxD and P1.3/TxD in UART mode
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin( GPIO_PORT_P1,
             GPIO_PIN2 | GPIO_PIN3, GPIO_PRIMARY_MODULE_FUNCTION );
}

void GPIO_init() {
    // Configure P1.0 (LED pin) as output
    MAP_GPIO_setAsOutputPin( GPIO_PORT_P1, GPIO_PIN0 );
    MAP_GPIO_setOutputLowOnPin( GPIO_PORT_P1, GPIO_PIN0 );
}

void UART_sendString( const char *str ) {
    if ( str == NULL || *str == '\0' ) return ;
    while (tx_busy) ;
    tx_busy = true;
    tx_str_ptr = &str[0];
    // clear TX interrupt flag
    MAP_UART_clearInterruptFlag( EUSCI_A_MODULE, EUSCI_A_UART_TRANSMIT_INTERRUPT_FLAG );
    // enable TX interrupt on EUSCI_A module
    MAP_UART_enableInterrupt( EUSCI_A_MODULE, EUSCI_A_UART_TRANSMIT_INTERRUPT );
    // send the first data byte
    MAP_UART_transmitData( EUSCI_A_MODULE, *tx_str_ptr++ );
    MAP_PCM_gotoLPM0(); // enter sleep mode LPM 0
}

void EUSCIA0_IRQHandler(void) {
    uint32_t status = MAP_UART_getEnabledInterruptStatus( EUSCI_A_MODULE );

    if ( status & EUSCI_A_UART_TRANSMIT_INTERRUPT_FLAG ) { // TX Interrupt
        MAP_UART_clearInterruptFlag( EUSCI_A_MODULE, EUSCI_A_UART_TRANSMIT_INTERRUPT_FLAG );
        if ( *tx_str_ptr != '\0' ) {
           MAP_UART_transmitData( EUSCI_A_MODULE, *tx_str_ptr++ );
        }
        else {
           // disable TX interrupt on EUSCI_A module
           MAP_UART_disableInterrupt( EUSCI_A_MODULE, EUSCI_A_UART_TRANSMIT_INTERRUPT );
           tx_str_ptr = NULL;
           tx_busy = false;
        }
    }

    if ( status & EUSCI_A_UART_RECEIVE_INTERRUPT_FLAG ) { // RX Interrupt
        uint8_t data, mask, flags;
        mask = EUSCI_A_UART_FRAMING_ERROR | EUSCI_A_UART_OVERRUN_ERROR |
               EUSCI_A_UART_PARITY_ERROR | EUSCI_A_UART_BREAK_DETECT |
               EUSCI_A_UART_RECEIVE_ERROR;
        flags = MAP_UART_queryStatusFlags( EUSCI_A_MODULE, mask );
        data  = MAP_UART_receiveData( EUSCI_A_MODULE );
        if ( !flags && !line_complete ) {
            if ( buf_index < (BUF_SIZE-1) ) {
               buf[ buf_index++] = data;
               if ( data == '\r' ) { // line complete
                  buf[ buf_index++ ] = '\n';
                  buf[ buf_index++ ] = '\0';
                  line_complete = true;
                  buf_index = 0;
                  MAP_Interrupt_disableSleepOnIsrExit();
               }
            }
        }
    }
    MAP_GPIO_toggleOutputOnPin( GPIO_PORT_P1, GPIO_PIN0 );
}

int main(void) {
    char str[ BUF_SIZE+1 ];

    MAP_WDT_A_holdTimer();          // disable WDT (watchdog timer)
    MAP_Interrupt_disableMaster();  // disable global interrupt
    GPIO_init();                    // initialize GPIO
    UART_init();                    // initialize EUSCI_A
    MAP_Interrupt_enableMaster();   // enable global interrupt

    UART_sendString( "\r\n\r\n\r\n" );
    UART_sendString( "TI MSP432P401 LaunchPad: " );
    UART_sendString( "UART testing...\r\n" );
    while (tx_busy) ;

    MAP_Interrupt_enableSleepOnIsrExit();
    MAP_PCM_gotoLPM0(); // enter sleep mode LPM 0

    while(1)  {
        if ( line_complete ) {
           MAP_Interrupt_disableSleepOnIsrExit();
           strncpy( str, (const char *)buf, BUF_SIZE-1 );
           UART_sendString( "Received: " );
           UART_sendString( str );
           line_complete = false;
           buf[0] = '\0';
           while (tx_busy) ;
           MAP_Interrupt_enableSleepOnIsrExit();
        }
        MAP_PCM_gotoLPM0(); // enter sleep mode LPM 0
    }
}
////////////////////////////////////////////////////////////////////////////////
