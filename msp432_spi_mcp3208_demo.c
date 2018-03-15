
#include <msp.h>
#include <cstdio>
#include <stdlib.h>
#include <string.h>

//-----------------------------------------------------------------------------
#define BAUDRATE        (115200)
#define UART_BRW_VALUE  (SystemCoreClock/BAUDRATE)

char sbuf[32];

void UART0_init(void) {
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

#define BRW_VALUE  (SystemCoreClock/1000000)
#define SS_BIT     (1 << 6)  

void SPI_init(void) {
    EUSCI_B0->CTLW0 = EUSCI_B_CTLW0_SWRST;           // disable eUSCI_B0 before config
    EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_SSEL__SMCLK     // 0x0080 (clock source: SMCLK)
                    | EUSCI_B_CTLW0_SYNC             // 0x0100 (Synchronous mode)
                    | EUSCI_B_CTLW0_MST              // 0x0800 (SPI master)
                    | EUSCI_B_CTLW0_MODE_0           // 0x0000 (3-wire SPI)
                    | EUSCI_B_CTLW0_MSB              // 0x2000 (MSB first)
              //    | EUSCI_B_CTLW0_CKPL             // 0x4000 (set Clock Polarity High)
                    | EUSCI_B_CTLW0_CKPH             // 0x8000 (set Clock Phase High)
                    | EUSCI_B_CTLW0_SWRST;
   
    EUSCI_B0->BRW = BRW_VALUE;                       // SPI clock speed: 1MHz 
    EUSCI_B0->CTLW0 &= ~EUSCI_B_CTLW0_SWRST;         // enable eUSCI_B0 after config 

    // P1.5, P1.6, P1.7 for UCB0CLK (SCK), UCB0SIMO (MOSI), UCB0SIMI (MISO) respectively
    P1->SEL0 |=  (BIT7 | BIT6 | BIT5);
    P1->SEL1 &= ~(BIT7 | BIT6 | BIT5);
    
    P4->DIR  |= SS_BIT;   // P4.6 as output
    P4->OUT  |= SS_BIT;   // P4.6 high
}

uint8_t SPI_transfer( uint8_t data ) {
    while (!(EUSCI_B0->IFG & EUSCI_B_IFG_TXIFG)) ;   // wait for transmit buffer empty
    EUSCI_B0->TXBUF = data;          // write data byte to TX buffer
    while (!(EUSCI_B0->IFG & EUSCI_B_IFG_RXIFG)) ;    // wait for receiver buffer  
    return EUSCI_B0->RXBUF;
} 

uint16_t readADC( uint8_t chan /* 0 or 1 */ ) {
   uint16_t value = 0;
   uint8_t wdata;

   // Singled-ended, channel=chan, MSB first
   wdata = (1 << 7) | ((chan & 1) << 6) | (1 << 5); 

   P4->OUT &= ~SS_BIT;
   SPI_transfer( 0x01 );                        // send the 1st byte   (start bit)
   value = SPI_transfer( wdata );               // send the 2nd byte (config bits)
   value = (value << 8) | SPI_transfer( 0x00 ); // send the 3rd byte
   P4->OUT |= SS_BIT;
   return (value & 0x0fff); 
}

void main(void) {
   uint8_t  i;
   uint16_t values[2];

   WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD; // stop watchdog timer

   SPI_init();                       // initialize the SPI (EUSCI_B0)
   UART0_init();

   send_str( "MSP432 SPI MCP3202 ADC demo...\r\n" );
   while (1) {
       for ( i=0; i < 2; i++ ) {
          values[i] = readADC( i );
       }
      sprintf( sbuf, "CH0: %04d, CH1: %04d\r\n", values[0], values[1] );
      send_str( sbuf );
      __delay_cycles( 20000000UL );
   }
}

