//////////////////////////////////////////////////////////////////////////////
#include "msp.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#define I2C_SLAVE_ADDR             (0x23) 

#define BH1750_RESET               (0x07)
#define BH1750_CONT_HIGH_RES_MODE  (0x10)

#define I2C_BRW_VALUE    (SystemCoreClock/100000)
#define TIMEOUT_CNT      (10000UL)

void delay_msec(uint32_t); 

char sbuf[64];  // used for sprintf()

// This example uses one of the Enhanced Universal Serial Communication Interface B
// (eUSCI_B) units which support two serial communication modes: I2C mode and SPI mode.
// The MSP432P401R has four eUSCI_Bx units.

void init_I2C1(void) {
    // first, disable eUSCI_B1  (SWRST=1)
    EUSCI_B1->CTLW0 |= EUSCI_B_CTLW0_SWRST;

    // UCA10 = 0     : own address is 7-bit
    // UCSLA10 = 0   : slave address is 7-bit
    // UCMM = 0      : single-master (not multi-master)
    // UCMODEx = 11  : I2C mode
    // UCSYNC = 1    : synchronous 
    // UCMST = 1     : master mode
    // UCA10 = 0     : 7-bit addressing is selected
    // CSSELx = 11   : SMCLK

    EUSCI_B1->CTLW0 |= EUSCI_B_CTLW0_SWRST
                    | EUSCI_B_CTLW0_MODE_3 
                    | EUSCI_B_CTLW0_MST 
                    | EUSCI_B_CTLW0_SYNC 
                    | EUSCI_B_CTLW0_SSEL__SMCLK;

    EUSCI_B1->BRW = I2C_BRW_VALUE;  // set clock prescaler 3MHz / 30 = 100kHz

    P6->SEL0 |=  0x30;              // configure P6.5 (SCL), P6.4 (SDA) as I2C pins
    P6->SEL1 &= ~0x30;

    // finally, enable UCB1 after configuration (SWRST=0)
    EUSCI_B1->CTLW0 &= ~EUSCI_B_CTLW0_SWRST; 
}

int I2C1_write( uint8_t addr, const uint8_t *data, uint32_t len ) {
    uint32_t i, cnt;

    cnt = 0;
    while ( EUSCI_B1->STATW & EUSCI_B_STATW_BBUSY ) { // wait if I2C bus busy
        cnt++;
        if (cnt > TIMEOUT_CNT) { // timeout 
            return -1; 
        }
    }    
    EUSCI_B1->I2CSA = addr;                 // set the slave address 
    EUSCI_B1->CTLW0 |= EUSCI_B_CTLW0_TR;    // I2C TX mode (UCTR=1)
    EUSCI_B1->CTLW0 |= EUSCI_B_CTLW0_TXSTT; // send START + address (UCTXSTT=1)

    // wait until the UCTXSTT flag is clear
    cnt = 0;
    while ( (EUSCI_B1->CTLW0 & EUSCI_B_CTLW0_TXSTT) ) {
        cnt++;
        if (cnt > TIMEOUT_CNT) { // timeout 
            return -2; 
        }
    }

    for ( i=0; i < len; i++ ) {
        EUSCI_B1->TXBUF = data[i];      // write data byte to the TX buffer
        // wait until the UCTXIFG0 flag set when UCBxTXBUF is empty.
        cnt = 0;
        while( !(EUSCI_B1->IFG & EUSCI_B_IFG_TXIFG0) ) {
           cnt++;
           if (cnt > TIMEOUT_CNT) { // timeout 
              return -3; 
           }
        } 
        if ( i == (len-1) ) { // is this the last byte
           EUSCI_B1->CTLW0 |= EUSCI_B_CTLW0_TXSTP;  // send STOP
        }
    }

    // wait until STOP is sent (the UCTXSTP flag is clear) 
    cnt = 0;
    while ( (EUSCI_B1->CTLW0 & EUSCI_B_CTLW0_TXSTP) ) { 
        cnt++;
        if (cnt > TIMEOUT_CNT) { // timeout 
           return -4;
        }
    }
    return 0;                           // no error 
}

int I2C1_read( uint8_t addr, uint8_t *data, uint32_t len ) {
    uint32_t i, cnt;

    cnt = 0;
    while ( EUSCI_B1->STATW & EUSCI_B_STATW_BBUSY ) { // wait if I2C bus busy
        cnt++;
        if (cnt > TIMEOUT_CNT) { // timeout 
            return -1; 
        }
    }

    EUSCI_B1->I2CSA = addr;                 // set the slave address 
    EUSCI_B1->CTLW0 &= ~EUSCI_B_CTLW0_TR;   // Rx mode (UCTR=0)
    EUSCI_B1->CTLW0 |= EUSCI_B_CTLW0_TXSTT; // send START + address (UCTXSTT=1)

    // wait until the UCTXSTT flag is clear
    cnt = 0;
    while ( (EUSCI_B1->CTLW0 & EUSCI_B_CTLW0_TXSTT) ) {    
        cnt++;
        if (cnt > TIMEOUT_CNT) { // timeout 
            return -2; 
        }
    }

    for ( i=0; i < len; i++ ) {
        // wait until the RXBUF has received a complete data byte
        cnt = 0;
        while ( !(EUSCI_B1->IFG & EUSCI_B_IFG_RXIFG0) ) {  
            cnt++;
            if (cnt > TIMEOUT_CNT) { // timeout 
               return -3; 
            }
            if ( EUSCI_B1->IFG & (EUSCI_B_IFG_NACKIFG | EUSCI_B_IFG_ALIFG) ) {
               return -5;   
            }
        }
        data[i] = EUSCI_B1->RXBUF;  // save the data byte to the buffer
        if ( i == (len-1) ) { // is this the last byte ?
            EUSCI_B1->CTLW0 |= EUSCI_B_CTLW0_TXSTP;  // send STOP 
        }
    }

    // wait until STOP is sent (the UCTXSTP flag is clear) 
    cnt = 0;
    while ( (EUSCI_B1->CTLW0 & EUSCI_B_CTLW0_TXSTP) ) { 
       cnt++;
       if (cnt > TIMEOUT_CNT) { // timeout 
         return -4;
       }
    }
    return 0;    // no error 
}

//-----------------------------------------------------------------------------
volatile uint32_t ticks = 0;

void delay_msec( uint32_t dly ) {
    uint32_t ts = ticks;
    while (1) { 
        if (ticks - ts >= dly)
          break;
    }
}
void SysTick_Handler(void) {
  if ( SysTick->CTRL & (1 << 16) ) {// check and clear the counting flag
     ticks++;
  }
}

void init_systick() {
  SysTick->LOAD = SystemCoreClock/1000-1;  // set the reload value (=> 1msec tick)
  SysTick->CTRL = 0b111;          //    ENABLE bit=1: enable SysTick interrupt
                                  //   TICKINT bit=1: enable interrupt
                                  // CLKSOURCE bit=1: use the core clock 
  // call the CMSIS core function to set priority for SysTick interrupt
  NVIC_SetPriority( SysTick_IRQn, 2 ); // set priority to 2 for SysTick 
}

//-----------------------------------------------------------------------------
#define BAUDRATE        (115200)
#define UART_BRW_VALUE  (SystemCoreClock/BAUDRATE)

void init_UART0(void) {
    P1->SEL0 |=  0x0C;               // config P1.3 and P1.2 for EUSCI_A0 (UART)
    P1->SEL1 &= ~0x0C;
    EUSCI_A0->CTLW0 |= 1;            // put the EUSCI_A0 to reset mode
    EUSCI_A0->MCTLW = 0;             // disable oversampling 
    EUSCI_A0->CTLW0 = 0x0081;        // 1 stop bit, no parity, SMCLK, 8-bit data 
    EUSCI_A0->BRW = UART_BRW_VALUE;  // set baudrate
    EUSCI_A0->CTLW0 &= ~1;           // enable EUSCI_A0 (no Tx/Rx interrupt)
}

void send_str( const char *str ) {
    int i, len = strlen(str);
    for ( i=0; i < len; i++ ) {
        while( !(EUSCI_A0->IFG & UCTXIFG) ) ; // wait until TX buffer is ready
        EUSCI_A0->TXBUF = str[i];
    }
}
//-----------------------------------------------------------------------------

void init_BH1750() {
    int error = 0;
    uint8_t cmd, mode;

    cmd   = BH1750_RESET;
    error |= I2C1_write( I2C_SLAVE_ADDR, &cmd, 1 );
    delay_msec(100);

    mode  = BH1750_CONT_HIGH_RES_MODE;
    error |= I2C1_write( I2C_SLAVE_ADDR, &mode, 1 );
    delay_msec(200);

    if (!error) {
        send_str( "BH1750 setup OK...\r\n" );
    } else {
        sprintf( sbuf, "BH1750 setup FAILED!!! (%d)\r\n", error );
        send_str( sbuf );
    }
}

void read_BH1750() {
    int error;
    uint32_t level, lux;
    uint8_t data[2];

    memset( data, 0x00, 2 );
    error = I2C1_read( I2C_SLAVE_ADDR, data, 2 );
    if (error) {
       sprintf( sbuf, "BH1750 Reading: FAILED (%d)\r\n", error );
       send_str( sbuf );
       init_I2C1();
    } 
    else {
       level = data[0];
       level = (level << 8) | data[1];
       lux = (10*level)/12;
       sprintf( sbuf, "%d lux\r\n", lux );
       send_str( sbuf ); 
    }
}

void main(void) {
    uint32_t ts;
    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD; // stop watchdog timer

    init_systick();        // initialize the SysTick
    __enable_interrupt();  // enable the global interrupt

    init_UART0();          // initialize UART0 (eUSCI_A0)
    init_I2C1();           // initialize I2C1 (eUSCI_B1)
    init_BH1750();         // initialize the BH1750 module

    send_str( "MSP432 I2C BH1750 demo...\r\n" );
    ts = ticks;
    while (1) {
        if ( ticks - ts >= 500 ) {
          ts += 500;
          read_BH1750();
        }
    }
}
//////////////////////////////////////////////////////////////////////////////
