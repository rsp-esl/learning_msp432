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

#define SHT3x_I2C_ADDR                 (0x44) // SHT31-D
#define I2C_ADDR                        SHT3x_I2C_ADDR

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

bool SHT3x_CRC8( uint8_t *buf ) {
    uint8_t i, _crc = 0xFF;

    _crc ^= buf[0];
    for ( i=0; i < 8; i++) {
       _crc = _crc & 0x80 ? (_crc << 1) ^ 0x31 : _crc << 1;
    }
    _crc ^= buf[1];
    for ( i=0; i < 8; i++) {
       _crc = _crc & 0x80 ? (_crc << 1) ^ 0x31 : _crc << 1;
    }
    return (_crc == buf[2]) ? true : false;
}

int SHT3x_sendCmd( uint16_t cmd ) {
    static uint8_t buf[2];
    int status;

    buf[0] = (cmd >> 8);
    buf[1] = (cmd & 0xff);
    status = I2C1_write( buf, 2 );
    if ( status != 0 ) {
        return status;
    }
    return 0;
}

int SHT3x_reset( ) {
    return SHT3x_sendCmd( 0x30A2 );
}

int SHT3x_startMeasurement( ) {
    return SHT3x_sendCmd( 0x2400 ); // high resolution measurement
}

bool SHT3x_readMeasurement( float *temperature, float *humidity ) {
    static uint8_t data[6];
    int status;

    status = I2C1_read( data, 6 );
    if ( status != 0 ) {
        return false; // error (timeout)
    }

    if ( SHT3x_CRC8(&data[0]) && SHT3x_CRC8(&data[3]) ) {
        uint16_t value;
        value  = (data[0] << 8) + data[1];
        *temperature  = ((value * 175.0) / 65535) - 45; // temperature
        value  = (data[3] << 8) + data[4];
        *humidity = ((value * 100.0) / 65535); // relative humidity
        return true; // ok
    }
    return false; // error (crc8 error)
}

///////////////////////////////////////////////////////////////////////////////

int main(void) {
    float temperature, humidity;
    MAP_WDT_A_holdTimer();
    LED_init();
    UART_init();
    I2C1_init();

    UART_sendString( "\r\n\r\n" );
    UART_sendString( "MSP432 LaunchPad: SHT3x Sensor Reading....\r\n" );

    SHT3x_reset();
    __delay_cycles( 2400000 );

    while(1)  {
        SHT3x_startMeasurement();
        __delay_cycles( 24000000 ); // delay for 500 msec
        if ( SHT3x_readMeasurement( &temperature, &humidity ) ) {
           sprintf( sbuf, "Temperature: %.1f deg.C, ", temperature );
           UART_sendString( sbuf );
           sprintf( sbuf, "Humidity: %.1f %%RH\r\n", humidity );
           UART_sendString( sbuf );
        } else {
            UART_sendString( "SHT3x reading error\r\n" );
            MAP_I2C_disableModule( EUSCI_I2C );
            __delay_cycles( 48000 );
            MAP_I2C_enableModule( EUSCI_I2C );
            __delay_cycles( 48000 );
        }
        MAP_GPIO_toggleOutputOnPin( GPIO_PORT_P1, GPIO_PIN0 );  // toggle the LED
    }
}

///////////////////////////////////////////////////////////////////////////////
