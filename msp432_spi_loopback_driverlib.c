////////////////////////////////////////////////////////////////////////////////
// Date: 2018-04-05
// Author: RSP (IoT Engineering Education @ KMUTNB)
// MCU Board: MSP432P401R LaunchPad
////////////////////////////////////////////////////////////////////////////////


#include <ti/devices/msp432p4xx/driverlib/driverlib.h>

#include <stdint.h>
#include <stdbool.h>

#define SDO_PIN     GPIO_PIN6
#define SDI_PIN     GPIO_PIN7
#define SCLK_PIN    GPIO_PIN5

#define CS_PIN      GPIO_PIN6

void SPI_init() {

    MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(
       GPIO_PORT_P1, SDO_PIN | SDI_PIN | SCLK_PIN,
       GPIO_PRIMARY_MODULE_FUNCTION );

    MAP_GPIO_setAsOutputPin( GPIO_PORT_P4, CS_PIN );   // use P4.6 for chip select
    MAP_GPIO_setOutputHighOnPin( GPIO_PORT_P4, CS_PIN );

    /*
     SPI_POL0_PHA0
        EUSCI_B_SPI_CLOCKPOLARITY_INACTIVITY_LOW |
        EUSCI_B_SPI_PHASE_DATA_CAPTURED_ONFIRST_CHANGED_ON_NEXT,

     SPI_POL0_PHA1
        EUSCI_B_SPI_CLOCKPOLARITY_INACTIVITY_LOW |
        EUSCI_B_SPI_PHASE_DATA_CHANGED_ONFIRST_CAPTURED_ON_NEXT,

      SPI_POL1_PHA0
        EUSCI_B_SPI_CLOCKPOLARITY_INACTIVITY_HIGH |
        EUSCI_B_SPI_PHASE_DATA_CAPTURED_ONFIRST_CHANGED_ON_NEXT,

      SPI_POL1_PHA1
        EUSCI_B_SPI_CLOCKPOLARITY_INACTIVITY_HIGH |
        EUSCI_B_SPI_PHASE_DATA_CHANGED_ONFIRST_CAPTURED_ON_NEXT
   */

    /*
           EUSCI_B_SPI_MSB_FIRST
           EUSCI_B_SPI_LSB_FIRST

           EUSCI_B_SPI_PHASE_DATA_CHANGED_ONFIRST_CAPTURED_ON_NEXT
           EUSCI_B_SPI_PHASE_DATA_CAPTURED_ONFIRST_CHANGED_ON_NEXT

           EUSCI_B_SPI_CLOCKPOLARITY_INACTIVITY_LOW
           EUSCI_B_SPI_CLOCKPOLARITY_INACTIVITY_HIGH
    */

    /* SPI Master Configuration Parameter */
    eUSCI_SPI_MasterConfig spiMasterConfig = {
           EUSCI_B_SPI_CLOCKSOURCE_SMCLK,             // SMCLK Clock Source
           SystemCoreClock,                           // SMCLK frequency
           500000,                                    // SPICLK = 500khz
           EUSCI_B_SPI_MSB_FIRST,                     // MSB First
           EUSCI_B_SPI_PHASE_DATA_CAPTURED_ONFIRST_CHANGED_ON_NEXT, // Phase
           EUSCI_B_SPI_CLOCKPOLARITY_INACTIVITY_HIGH,                // Polarity
           EUSCI_B_SPI_3PIN                                         // 3-Wire SPI Mode
    };

    MAP_SPI_initMaster( EUSCI_B0_BASE, &spiMasterConfig );
    MAP_SPI_enableModule( EUSCI_B0_BASE );

    MAP_Interrupt_disableInterrupt( INT_EUSCIB0 );
    MAP_SPI_clearInterruptFlag( EUSCI_B0_BASE, EUSCI_B_SPI_RECEIVE_INTERRUPT );
  // MAP_Interrupt_enableInterrupt( INT_EUSCIB0 );

  // uint8_t status = MAP_SPI_getEnabledInterruptStatus(EUSCI_B0_BASE);
  // MAP_SPI_clearInterruptFlag(EUSCI_B0_BASE, status);

}

int main(void) {
    uint32_t i;
    uint8_t wdata = 0x00;
    uint8_t rdata;

    MAP_WDT_A_holdTimer();
    SPI_init();

    while(1)  {
        MAP_GPIO_setOutputLowOnPin( GPIO_PORT_P4, CS_PIN );

        // Polling to see if the TX buffer is ready
#if 0
        while (!(EUSCI_B0->IFG & EUSCI_B_IFG_TXIFG)) ;   // wait for transmit buffer empty
#else
        while ( !(MAP_SPI_getInterruptStatus( EUSCI_B0_BASE, EUSCI_B_SPI_TRANSMIT_INTERRUPT)) );
#endif

        MAP_SPI_transmitData( EUSCI_B0_BASE, wdata );
        while ( MAP_SPI_isBusy( EUSCI_B0_BASE ) ) ;

        // Polling to see if the RX buffer is ready
#if 0
        while (!(EUSCI_B0->IFG & EUSCI_B_IFG_RXIFG)) ;    // wait for receiver buffer
#else
        while ( !(MAP_SPI_getInterruptStatus( EUSCI_B0_BASE, EUSCI_B_SPI_RECEIVE_INTERRUPT)) );
#endif
        rdata = MAP_SPI_receiveData(EUSCI_B0_BASE);

        MAP_GPIO_setOutputHighOnPin( GPIO_PORT_P4, CS_PIN );

        wdata++;
        for (i=0; i < 100000; i++) { __asm("nop"); }
    }
}

////////////////////////////////////////////////////////////////////////////////
