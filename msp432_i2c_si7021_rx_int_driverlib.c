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

#define SI7021_I2C_ADDR                (0x40)
#define I2C_ADDR                        SI7021_I2C_ADDR

#define SI7021_MEASTEMP_NOHOLD_CMD     (0xF3)
#define SI7021_MEASRH_NOHOLD_CMD       (0xF5)
#define SI7021_RESET_CMD               (0xFE)
#define SI7021_READ_USER_REG           (0xE7)

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

    MAP_I2C_disableModule( EUSCI_I2C );   // disable the uSCI
    MAP_Interrupt_enableInterrupt( INT_EUSCIB1 ); // enable eUSCI_B1 interrupt
    MAP_I2C_initMaster( EUSCI_I2C, &i2cConfig );
    MAP_I2C_enableModule( EUSCI_I2C );    // enable the eUSCI

    MAP_I2C_setSlaveAddress( EUSCI_I2C, I2C_ADDR ); // set the slave address
}

volatile uint32_t rx_len, rx_index;
volatile uint8_t rx_data[4];
volatile bool rx_done;

#define TIMEOUT 48000

void EUSCIB1_IRQHandler(void) {
    uint_fast16_t status;
    status = MAP_I2C_getEnabledInterruptStatus( EUSCI_I2C );
    MAP_I2C_clearInterruptFlag( EUSCI_I2C, status );

    if ( status & EUSCI_B_I2C_RECEIVE_INTERRUPT0 ) {
        if ( rx_index == (rx_len-2) ) {
            MAP_I2C_disableInterrupt( EUSCI_I2C, EUSCI_B_I2C_RECEIVE_INTERRUPT0 );
            MAP_I2C_enableInterrupt( EUSCI_I2C, EUSCI_B_I2C_STOP_INTERRUPT );
            MAP_I2C_masterReceiveMultiByteStop( EUSCI_I2C );
        }
        rx_data[ rx_index++ ] = MAP_I2C_masterReceiveMultiByteNext( EUSCI_I2C );
    }

    if ( status & EUSCI_B_I2C_STOP_INTERRUPT ) {
        MAP_I2C_disableInterrupt( EUSCI_I2C, EUSCI_B_I2C_STOP_INTERRUPT );
        rx_data[ rx_index++ ] = MAP_I2C_masterReceiveMultiByteFinish( EUSCI_I2C );
        rx_done = true;
    }
}

int I2C_readRaw( uint8_t cmd, uint8_t *data, uint32_t len ) {
    uint32_t cnt;

    //MAP_I2C_setMode( EUSCI_I2C, EUSCI_B_I2C_TRANSMIT_MODE );
    if ( !MAP_I2C_masterSendMultiByteStartWithTimeout( EUSCI_I2C, cmd, TIMEOUT ) ) {
        return -1;
    }
    if ( !MAP_I2C_masterSendMultiByteStopWithTimeout( EUSCI_I2C, TIMEOUT ) ) {
        return -2;
    }

    __delay_cycles( 1200000 );

    rx_done = false;
    rx_index = 0;
    rx_len = 3;
    memset( data, 0x00, 3 );

    //MAP_I2C_setMode( EUSCI_I2C, EUSCI_B_I2C_RECEIVE_MODE );
    MAP_I2C_masterReceiveStart( EUSCI_I2C );
    MAP_I2C_enableInterrupt( EUSCI_I2C, EUSCI_B_I2C_RECEIVE_INTERRUPT0 );

    cnt = TIMEOUT;
    while ( !rx_done && cnt > 0 ) {
        cnt--;
    }
    memcpy( data, (void *)rx_data, 3 );

    if ( cnt == 0 ) {
        return -3;
    }
    return 0;
}

///////////////////////////////////////////////////////////////////////////////

int SI7021_readReg( uint8_t cmd, uint8_t *data ) {
    *data = 0x00;

    //MAP_I2C_setMode( EUSCI_I2C, EUSCI_B_I2C_TRANSMIT_MODE );
    if ( !MAP_I2C_masterSendSingleByteWithTimeout( EUSCI_I2C, cmd, TIMEOUT ) ) {
        return -1;
    }

    __delay_cycles( 4800 );

    //MAP_I2C_setMode( EUSCI_I2C, EUSCI_B_I2C_RECEIVE_MODE );
    MAP_I2C_masterReceiveStart( EUSCI_I2C );
    if ( !MAP_I2C_masterReceiveMultiByteFinishWithTimeout( EUSCI_I2C, data, TIMEOUT ) ) {
        return -2;
    }
    return 0;
}

int SI7021_reset() { // soft reset
    uint8_t cmd, data = 0x00;

    while ( MAP_I2C_isBusBusy( EUSCI_I2C) );

    //MAP_I2C_setMode( EUSCI_I2C, EUSCI_B_I2C_TRANSMIT_MODE );
    cmd = SI7021_RESET_CMD;
    if ( !MAP_I2C_masterSendSingleByteWithTimeout( EUSCI_I2C, cmd, TIMEOUT ) ) {
        return -1;
    }

    __delay_cycles( 1200000 );

    cmd = SI7021_READ_USER_REG;
    return ( SI7021_readReg( cmd, &data ) );  // read dummy user reg
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

    I2C_readRaw( cmd, buf, 3 );

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
    static uint32_t cnt = 1;
    uint16_t value;
    float temperature, humidity;

    sprintf( sbuf, "%d) ", cnt++ );
    UART_sendString( sbuf );

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
    int ret_val;

    MAP_WDT_A_holdTimer();
    MAP_Interrupt_disableMaster();  // disable global interrupt
    LED_init();
    UART_init();
    I2C1_init();
    MAP_Interrupt_enableMaster();  // enable global interrupt

    UART_sendString( "\r\n\r\n" );
    UART_sendString( "MSP432 LaunchPad: Si7021 Sensor Reading....\r\n" );

    if ( (ret_val = SI7021_reset()) != 0 ) {
        sprintf( sbuf, "I2C reset error (%d)\r\n", ret_val );
        UART_sendString( sbuf );
    }

    while(1)  {
        SI7021_readTempRH();
        MAP_GPIO_toggleOutputOnPin( GPIO_PORT_P1, GPIO_PIN0 );  // toggle the LED
        __delay_cycles( 24000000 ); // delay for 0.5 seconds @48MHz clock speed
    }
}
///////////////////////////////////////////////////////////////////////////////
