////////////////////////////////////////////////////////////////////////////////
// Date: 2018-04-07
// Author: RSP (IoT Engineering Education @ KMUTNB)
// MCU Board: MSP432P401R LaunchPad
////////////////////////////////////////////////////////////////////////////////

#include <ti/devices/msp432p4xx/driverlib/driverlib.h>

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>

#define EUSCI_I2C   EUSCI_B1_BASE    // use the eUSCI_B1 for I2C
#define SCL_PIN     GPIO_PIN5        // P6.5 / SCL
#define SDA_PIN     GPIO_PIN4        // P6.4 / SDA

#define I2C_SLAVE_ADDR   (0x38)      // PCF8574A address (default)

void LED_init( ) {
    // configure LED (P1.0) as output
    MAP_GPIO_setAsOutputPin( GPIO_PORT_P1, GPIO_PIN0 );
    MAP_GPIO_setOutputLowOnPin( GPIO_PORT_P1, GPIO_PIN0 );
}

void I2C1_init() { // use EUSCI_B1

    eUSCI_I2C_MasterConfig i2cConfig = {
       EUSCI_B_I2C_CLOCKSOURCE_SMCLK,     // use SMCLK Clock Source
       SystemCoreClock,                   // specify SMCLK clock frequency
       EUSCI_B_I2C_SET_DATA_RATE_100KBPS, // set the I2C speed
       0,                                 // No byte counter threshold
       EUSCI_B_I2C_NO_AUTO_STOP           // No Autostop
    };

    // P6.5 (SCL), P6.4 (SDA) as I2C pins
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(
            GPIO_PORT_P6, SCL_PIN | SDA_PIN, GPIO_PRIMARY_MODULE_FUNCTION );

    MAP_I2C_disableModule( EUSCI_I2C );
    MAP_I2C_initMaster( EUSCI_I2C, &i2cConfig );   // configure eUSCI_B1 as I2C master
    MAP_I2C_enableModule( EUSCI_I2C );             // enable eUSCI_B1
    MAP_Interrupt_disableInterrupt( INT_EUSCIB1 ); // disable eUSCI_B1 interrupt
}

int main(void) {
    uint8_t wdata = 0xFE;
    uint8_t rdata;

    MAP_WDT_A_holdTimer();  // disable WDT
    LED_init();             // initialize GPIO (LED pin)
    I2C1_init();            // initialize eUSCI_B1 for I2C

    MAP_I2C_setSlaveAddress( EUSCI_I2C, I2C_SLAVE_ADDR ); // set I2C slave address for PCF8574A

    while(1)  {

#if 0
        // write a single data byte from PCF8574A
        MAP_I2C_setMode( EUSCI_I2C, EUSCI_B_I2C_TRANSMIT_MODE );
        // send START condition, transmit a single data byte and send STOP condition
        MAP_I2C_masterSendSingleByte( EUSCI_I2C, wdata );
        while ( MAP_I2C_masterIsStopSent(EUSCI_I2C) == EUSCI_B_I2C_SENDING_STOP ) ;

        __delay_cycles(4800);

        // read a single data byte from the PCF8574A
        MAP_I2C_setMode( EUSCI_I2C, EUSCI_B_I2C_RECEIVE_MODE );
        rdata = MAP_I2C_masterReceiveSingleByte( EUSCI_I2C );
        while ( MAP_I2C_masterIsStopSent(EUSCI_I2C) == EUSCI_B_I2C_SENDING_STOP ) ;

#else
        // write a single data byte from PCF8574A
        MAP_I2C_setMode( EUSCI_I2C, EUSCI_B_I2C_TRANSMIT_MODE );
        // send START condition and transmit only a single data byte
        MAP_I2C_masterSendMultiByteStart( EUSCI_I2C, wdata );
        // wait for Tx interrupt flag, then send STOP condition
        MAP_I2C_masterSendMultiByteStop( EUSCI_I2C );
        while ( MAP_I2C_masterIsStopSent(EUSCI_I2C) == EUSCI_B_I2C_SENDING_STOP ) ;

        __delay_cycles(4800);

        // read a single data byte from the PCF8574A
        MAP_I2C_setMode( EUSCI_I2C, EUSCI_B_I2C_RECEIVE_MODE );
        // send START condition
        MAP_I2C_masterReceiveStart( EUSCI_I2C );
        // send STOP condition before read the data byte (only a single byte)
        MAP_I2C_masterReceiveMultiByteStop( EUSCI_B1_BASE );
        rdata = MAP_I2C_masterReceiveMultiByteFinish( EUSCI_I2C );
        while ( MAP_I2C_masterIsStopSent(EUSCI_I2C) == EUSCI_B_I2C_SENDING_STOP ) ;
#endif

        wdata ^= 0x03; // toggle two LEDs by inverting the two lowest bits of wdata

        // check whether the button is pressed (that is, the 7-bit of rdata is 0)
        if ( (rdata & 0x80) == 0x00 ) { // The PB is pressed.
            MAP_GPIO_setOutputLowOnPin( GPIO_PORT_P1, GPIO_PIN0 );  // turn off LED
        } else {
            MAP_GPIO_toggleOutputOnPin( GPIO_PORT_P1, GPIO_PIN0 );  // toggle LED
        }

        __delay_cycles(4000000);
    }
}
////////////////////////////////////////////////////////////////////////////////
