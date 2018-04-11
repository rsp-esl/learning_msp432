////////////////////////////////////////////////////////////////////////////////
// Date: 2018-04-11
// Author: RSP (IoT Engineering Education @ KMUTNB)
// MCU Board: MSP432P401R LaunchPad
////////////////////////////////////////////////////////////////////////////////

#include <ti/devices/msp432p4xx/driverlib/driverlib.h>

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>

#define EUSCI_I2C   EUSCI_B1_BASE      // use the eUSCI_B1 for I2C
#define SCL_PIN     GPIO_PIN5          // P6.5 / SCL
#define SDA_PIN     GPIO_PIN4          // P6.4 / SDA

#define SI7021_MEASTEMP_NOHOLD_CMD     (0xF3)
#define SI7021_MEASRH_NOHOLD_CMD       (0xF5)
#define SI7021_RESET_CMD               (0xFE)
#define SI7021_READ_USER_REG           (0xE7)

#define SI7021_I2C_ADDR                (0x40)
#define I2C_ADDR                        SI7021_I2C_ADDR

char sbuf[64];                         // used for sprintf()

///////////////////////////////////////////////////////////////////////////////
// Assume SystemCoreClock = 48MHz, Baudrate = 115200
const eUSCI_UART_Config uartConfig = {
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
    MAP_UART_initModule( EUSCI_A0_BASE, &uartConfig );
    MAP_UART_enableModule( EUSCI_A0_BASE );
    // Selecting P1.2 and P1.3 in UART mode
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin( GPIO_PORT_P1,
             GPIO_PIN2 | GPIO_PIN3, GPIO_PRIMARY_MODULE_FUNCTION );
}

void UART_sendString( const char *str ) {
    const char *p = &str[0];
    while ( *p != '\0' ) {
       MAP_UART_transmitData( EUSCI_A0_BASE, *p++ );
    }
}

///////////////////////////////////////////////////////////////////////////////

void LED_init( ) { // configure P1.0 (LED) as output
    MAP_GPIO_setAsOutputPin( GPIO_PORT_P1, GPIO_PIN0 );
    MAP_GPIO_setOutputLowOnPin( GPIO_PORT_P1, GPIO_PIN0 );
}

///////////////////////////////////////////////////////////////////////////////

#define TIMEOUT  48000

void I2C1_init() { // use EUSCI_B1

    eUSCI_I2C_MasterConfig i2cConfig = {
       EUSCI_B_I2C_CLOCKSOURCE_SMCLK,     // use SMCLK Clock Source
       SystemCoreClock,                   // specify SMCLK clock frequency
       EUSCI_B_I2C_SET_DATA_RATE_400KBPS, // set the I2C speed
       0,                                 // No byte counter threshold
       EUSCI_B_I2C_NO_AUTO_STOP           // No Autostop
    };

    // P6.5 (SCL), P6.4 (SDA) as I2C pins
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(
            GPIO_PORT_P6, SCL_PIN | SDA_PIN, GPIO_PRIMARY_MODULE_FUNCTION );

    MAP_I2C_disableModule( EUSCI_I2C );
    MAP_Interrupt_disableInterrupt( INT_EUSCIB1 ); // disable eUSCI_B1 interrupt
    MAP_I2C_initMaster( EUSCI_I2C, &i2cConfig );
    MAP_I2C_enableModule( EUSCI_I2C );

    MAP_I2C_setSlaveAddress( EUSCI_B1_BASE, I2C_ADDR ); // set slave address
}

int I2C1_write( const uint8_t *data, uint32_t len ) {
    int i, status=0;

    //MAP_I2C_setMode( EUSCI_B1_BASE, EUSCI_B_I2C_TRANSMIT_MODE );
    if ( len==1 ) {
        if ( !MAP_I2C_masterSendSingleByteWithTimeout( EUSCI_I2C, data[0], TIMEOUT ) ) {
            status = -1;
            return status;
        }
        return status;
    }

    for ( i=0; i < len; i++ ) {
        if ( i==0 ) { // send START condition and transmit the first byte
            if ( !MAP_I2C_masterSendMultiByteStartWithTimeout( EUSCI_I2C, data[i], TIMEOUT ) ) {
                status = -1;
                break;
            }
        }
        else if (i==(len-1)) { // send the last byte and send STOP condition
            if ( !MAP_I2C_masterSendMultiByteFinishWithTimeout( EUSCI_I2C, data[i], TIMEOUT ) ) {
                status = -3;
                break;
            }
        }
        else {
            if ( !MAP_I2C_masterSendMultiByteNextWithTimeout( EUSCI_I2C, data[i], TIMEOUT ) ) {
                status = -2;
                break;
            }
        }
    }
    return status;
}

int I2C1_read( uint8_t *data, uint32_t len ) {
    int i;
    uint32_t cnt;

    //MAP_I2C_setMode( EUSCI_I2C, EUSCI_B_I2C_RECEIVE_MODE );
    MAP_I2C_masterReceiveStart( EUSCI_I2C ); // send START condition
    for ( i=0; i < len; i++ ) {
        if ( i==(len-1)) { // send STOP condition before reading the last byte
            MAP_I2C_masterReceiveMultiByteStop( EUSCI_I2C );
        }
        // poll for I2C RX interrupt flag
        cnt = TIMEOUT;
        while ( !(EUSCI_B_CMSIS(EUSCI_I2C)->IFG & EUSCI_B_IFG_RXIFG) ) {
            cnt--;
            if ( cnt == 0 ) {
                MAP_I2C_masterReceiveMultiByteStop( EUSCI_I2C );
                return -1;
            }
        }
        data[i] = MAP_I2C_masterReceiveMultiByteNext( EUSCI_I2C );
    }
    while ( MAP_I2C_masterIsStopSent(EUSCI_I2C) == EUSCI_B_I2C_SENDING_STOP ) ;
    return 0;
}

///////////////////////////////////////////////////////////////////////////////

void SI7021_reset() { // soft reset
    uint8_t cmd, data = 0x00;
    int status;

    while ( MAP_I2C_isBusBusy( EUSCI_I2C ) == EUSCI_B_I2C_BUS_BUSY );

    cmd = SI7021_RESET_CMD;
    status = I2C1_write( &cmd, 1 );
    if ( status != 0 ) {
       sprintf( sbuf, "SI7021_reset(): i2c write timeout (%d)\r\n", status );
       UART_sendString( sbuf );
       return;
    }
    __delay_cycles( 1200000 );

    cmd = SI7021_READ_USER_REG;
    status = I2C1_write( &cmd, 1 );
    if (status != 0 ) {
       sprintf( sbuf, "SI7021_reset(): i2c write timeout (%d)\r\n", status );
       UART_sendString( sbuf );
       return;
    }

    __delay_cycles( 4800 );

    status = I2C1_read( &data, 1 ); // dummy read user register
    if ( status != 0 ) {
       sprintf( sbuf, "SI7021_reset(): i2c read timeout (%d)\r\n", status );
       UART_sendString( sbuf );
       return;
    }
}

uint8_t SI7021_CRC8( const uint8_t *data, uint8_t len ) {
    uint8_t crc = 0x00;
    int i, j;
    for ( i=0; i < len; ++i ) {
        crc ^= data[i];
        for ( j=8; j > 0; --j) {
           if ( crc & 0x80 ) {
               crc = (crc << 1) ^ 0x131;
           } else {
               crc = (crc << 1);
           }
       }
    }
    return crc;
}

uint16_t SI7021_readRaw( uint8_t cmd ) {
    static uint8_t buf[3];
    uint8_t crc;
    uint16_t value;
    int status;

    while ( MAP_I2C_isBusBusy( EUSCI_I2C ) == EUSCI_B_I2C_BUS_BUSY );

    status = I2C1_write( &cmd, 1 );
    if ( status ) {
        sprintf( sbuf, "SI7021_readRaw(): i2c write timeout (%d)\r\n", status );
        UART_sendString( sbuf );
        return 0;
    }

    __delay_cycles( 2400000 ); // wait at least 25 msec

    memset( buf, 0x00, 3 );
    status = I2C1_read( buf, 3 ); // read 3 bytes from the I2C device
    if ( status ) {
        sprintf( sbuf, "SI7021_readRaw(): i2c read timeout (%d)\r\n", status );
        UART_sendString( sbuf );
        return 0;
    }

    crc = SI7021_CRC8( buf, 2 );
    if ( crc != buf[2] ) { // CRC error
       UART_sendString( "CRC8 failed\r\n" );
       sprintf( sbuf, "  %02X != %02X\r\n", crc, buf[2] );
       UART_sendString( sbuf );
       return 0;
    }
    value = buf[0];
    value = (value << 8) | buf[1];
    return value;
}

void SI7021_readTempRH( ) {
    uint16_t value;
    float temperature;
    float humidity;

    value = SI7021_readRaw( SI7021_MEASTEMP_NOHOLD_CMD );
    if ( value != 0 ) {
       temperature = value;
       temperature *= 175.72f;
       temperature /= 65536;
       temperature -= 46.85f;
       sprintf( sbuf, "Temperature: %.1f deg.C, ", temperature );
       UART_sendString( sbuf );
    }

    value = SI7021_readRaw( SI7021_MEASRH_NOHOLD_CMD );
    if ( value != 0 ) {
       humidity  = value;
       humidity *= 125;
       humidity /= 65536;
       humidity -= 6;
       sprintf( sbuf, "Humidity: %.1f %%RH\r\n", humidity );
       UART_sendString( sbuf );
    }
}

///////////////////////////////////////////////////////////////////////////////

int main(void) {
    MAP_WDT_A_holdTimer();
    LED_init();
    UART_init();
    I2C1_init();

    UART_sendString( "\r\n\r\n" );
    UART_sendString( "MSP432 LaunchPad: Si7021 Sensor Reading....\r\n" );

    SI7021_reset();

    while(1)  {
        SI7021_readTempRH();
        MAP_GPIO_toggleOutputOnPin( GPIO_PORT_P1, GPIO_PIN0 );  // toggle the LED
        __delay_cycles( 24000000 );
    }
}
///////////////////////////////////////////////////////////////////////////////
