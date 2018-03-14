#include <msp.h>

#define STCP       (1<<4)
#define BRW_VALUE  (SystemCoreClock/1000000)

void SPI_init(void) {
    EUSCI_B0->CTLW0 = EUSCI_B_CTLW0_SWRST;       // disable UCB0 before config
    
    EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_SSEL__SMCLK     // 0x0080
                    | EUSCI_B_CTLW0_SYNC             // 0x0100
                    | EUSCI_B_CTLW0_MST              // 0x0800
             //     | EUSCI_B_CTLW0_MODE_1           // 0x0200 4-pin master mode, UCxSTE is a digital output for SEL    
                    | EUSCI_B_CTLW0_MSB              // 0x2000
             //     | EUSCI_B_CTLW0_CKPL             // 0x4000
                    | EUSCI_B_CTLW0_CKPH             // 0x8000
                    | EUSCI_B_CTLW0_SWRST;
   
    EUSCI_B0->BRW = BRW_VALUE;                      // SPI clock speed: 1MHz 
    EUSCI_B0->CTLW0 &= ~EUSCI_B_CTLW0_SWRST;        // enable UCB0 after config 

    P1->SEL0 |=  0x60;              // P1.5, P1.6 for UCB0CLK, UCB0SIMO respectively
    P1->SEL1 &= ~0x60;

    P6->DIR |=  STCP;               // STCP pin output (P6.4) 
    P6->OUT &= ~STCP;               // STCP LOW
}

void SPI_write( uint8_t data ) {
    while (!(EUSCI_B0->IFG & 2)) ;   // wait for transmit buffer empty
    EUSCI_B0->TXBUF = data;          // write data byte to TX buffer
    while (EUSCI_B0->STATW & 1) ;    // wait for transmit done 
    P6->OUT |= STCP;                 // STCP HIGH
    P6->OUT &= ~STCP;                // STCP LOW
} 

void main(void) {
   uint8_t data = 0x01;
  
   WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD; // stop watchdog timer

   SPI_init();

   while (1) {
      SPI_write( data );
      data ^= 0x81;
      __delay_cycles( 10000000UL );
   }
}

