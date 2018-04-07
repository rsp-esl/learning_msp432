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

#define SCL_PIN     GPIO_PIN5  // P6.5
#define SDA_PIN     GPIO_PIN4  // P6.4

#define I2C_SLAVE_ADDR   (0x38)  // PCF8574A address (default)

void LED_init( ) {
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

    MAP_I2C_initMaster( EUSCI_B1_BASE, &i2cConfig );
    MAP_I2C_enableModule( EUSCI_B1_BASE );
}

int main(void) {
    uint8_t wdata = 0xFE;
    uint8_t rdata;

    MAP_WDT_A_holdTimer();
    LED_init();
    I2C1_init();

    while(1)  {
        while ( MAP_I2C_isBusBusy(EUSCI_B1_BASE) );

        // write a data byte (wdata) to the PCF8574A via I2C bus
        MAP_I2C_setMode( EUSCI_B1_BASE, EUSCI_B_I2C_TRANSMIT_MODE );
        MAP_I2C_setSlaveAddress( EUSCI_B1_BASE, I2C_SLAVE_ADDR );
#if 0
        // send START, transmit only a single data byte and send STOP
        MAP_I2C_masterSendSingleByte( EUSCI_B1_BASE, wdata );
#else
        MAP_I2C_masterSendMultiByteStart( EUSCI_B1_BASE, wdata );
        MAP_I2C_masterSendMultiByteStop( EUSCI_B1_BASE );
#endif
        while ( !MAP_I2C_masterIsStopSent( EUSCI_B1_BASE) );

        wdata ^= 0x03; // toggle two LEDs by inverting the two lowest bits of wdata

        __delay_cycles(100000);

        // read a data byte from the PCF8574A via I2C bus
        MAP_I2C_setSlaveAddress( EUSCI_B1_BASE, I2C_SLAVE_ADDR );
        MAP_I2C_setMode( EUSCI_B1_BASE, EUSCI_B_I2C_RECEIVE_MODE );
        MAP_I2C_masterReceiveStart( EUSCI_B1_BASE );
#if 0
        rdata = MAP_I2C_masterReceiveSingleByte( EUSCI_B1_BASE );
#else
        rdata = MAP_I2C_masterReceiveMultiByteNext( EUSCI_B1_BASE );
#endif
        MAP_I2C_masterReceiveMultiByteStop( EUSCI_B1_BASE );

        // check whether the button is pressed (that is, the 7-bit of rdata is 0)
        if ( (rdata & 0x80) == 0x00 ) { // button is pressed
            MAP_GPIO_setOutputLowOnPin( GPIO_PORT_P1, GPIO_PIN0 );  // turn off LED
        } else {
            MAP_GPIO_toggleOutputOnPin( GPIO_PORT_P1, GPIO_PIN0 );  // toggle LED
        }

        __delay_cycles(4000000);
    }
}

////////////////////////////////////////////////////////////////////////////////
