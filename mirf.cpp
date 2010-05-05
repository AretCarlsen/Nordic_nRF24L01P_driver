/*
    Copyright (c) 2007 Stefan Engelke <mbox@stefanengelke.de>

    Permission is hereby granted, free of charge, to any person 
    obtaining a copy of this software and associated documentation 
    files (the "Software"), to deal in the Software without 
    restriction, including without limitation the rights to use, copy, 
    modify, merge, publish, distribute, sublicense, and/or sell copies 
    of the Software, and to permit persons to whom the Software is 
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be 
    included in all copies or substantial portions of the Software.

    (The above Blowhard License requires itself to not be removed.
     Though this significantly lowers the benefit/cost ratio, the library
     is still worth using.)

    No warranty is provided, express or implied.

    $Id$
*/

#include "mirf.hpp"
#include "nRF24L01.hpp"
#include "nRF-spi.hpp"
#include <avr/io.h>
#include <avr/interrupt.h>

// Defines for setting the MiRF registers for transmitting or receiving mode
#define TX_POWERUP mirf_config_register(CONFIG, mirf_CONFIG | ( (1<<PWR_UP) | (0<<PRIM_RX) ) )
#define RX_POWERUP mirf_config_register(CONFIG, mirf_CONFIG | ( (1<<PWR_UP) | (1<<PRIM_RX) ) )


// Flag which denotes transmitting mode
volatile uint8_t PTX;

void mirf_init(){ 
// Initializes pins and interrupt to communicate with the MiRF module
// Should be called in the early initializing phase at startup.
    // Define CSN and CE as Output and set them to default
    sbi(DDRB, CSN);
    sbi(DDRB, CE); //    DDRB |= ((1<<CSN)|(1<<CE));
    mirf_CE_lo();
    mirf_CSN_hi();

// Mod: Removed Mega8 v 168 check (ANC)
    // Initialize external interrupt on port PD6 (PCINT22)
    cbi(DDRD, INT_PIN); //    DDRB &= ~(1<<PD6);
    PCMSK2 = (1<<INT_FLAG);
    PCICR  = (1<<PCIE2);

    // Initialize spi module
    spi_init();
}

void mirf_config(){
// Sets the important registers in the MiRF module and powers the module
// in receiving mode
    // Set RF channel
    mirf_config_register(RF_CH,mirf_CH);

    // Set length of incoming payload 
    mirf_config_register(RX_PW_P0, mirf_PAYLOAD);

    // Start receiver 
    PTX = 0;        // Start in receiving mode
    RX_POWERUP;     // Power up in receiving mode
    mirf_CE_hi();     // Listening for pakets
}

void mirf_set_RADDR(uint8_t * adr){ 
// Sets the receiving address
    mirf_CE_lo();
    mirf_write_register(RX_ADDR_P0,adr,5);
    mirf_CE_hi();
}

void mirf_set_TADDR(uint8_t * adr){
// Sets the transmitting address
    mirf_write_register(TX_ADDR, adr,5);
}

// Mod: SIGNAL to ISR (ANC); SIG_PIN_CHANGE2 to PCINT2_vect
ISR(PCINT2_vect){
// Interrupt handler 
    uint8_t status;   
    // If still in transmitting mode then finish transmission
    if (PTX) {
        // Read MiRF status 
        mirf_CSN_lo();                                // Pull down chip select
        status = spi_fast_shift(NOP);               // Read status register
        mirf_CSN_hi();                                // Pull up chip select

        mirf_CE_lo();                             // Deactivate transreceiver
        RX_POWERUP;                             // Power up in receiving mode
        mirf_CE_hi();                             // Listening for pakets
        PTX = 0;                                // Set to receiving mode

        // Reset status register for further interaction
        mirf_config_register(STATUS,(1<<TX_DS)|(1<<MAX_RT)); // Reset status register
    }
}

extern uint8_t mirf_data_ready(){ 
// Checks if data is available for reading
    if (PTX) return 0;
    uint8_t status;
    // Read MiRF status 
    mirf_CSN_lo();                                // Pull down chip select
    status = spi_fast_shift(NOP);               // Read status register
    mirf_CSN_hi();                                // Pull up chip select
    return status & (1<<RX_DR);
}

extern void mirf_get_data(uint8_t * data){
// Reads mirf_PAYLOAD bytes into data array
    mirf_CSN_lo();                               // Pull down chip select
    spi_fast_shift( R_RX_PAYLOAD );            // Send cmd to read rx payload
    spi_transfer_sync(data,data,mirf_PAYLOAD); // Read payload
    mirf_CSN_hi();                               // Pull up chip select
    mirf_config_register(STATUS,(1<<RX_DR));   // Reset status register
}

void mirf_config_register(uint8_t reg, uint8_t value){
// Clocks only one byte into the given MiRF register
    mirf_CSN_lo();
    spi_fast_shift(W_REGISTER | (REGISTER_MASK & reg));
    spi_fast_shift(value);
    mirf_CSN_hi();
}

void mirf_read_register(uint8_t reg, uint8_t * value, uint8_t len){
// Reads an array of bytes from the given start position in the MiRF registers.
    mirf_CSN_lo();
    spi_fast_shift(R_REGISTER | (REGISTER_MASK & reg));
    spi_transfer_sync(value,value,len);
    mirf_CSN_hi();
}

void mirf_write_register(uint8_t reg, uint8_t * value, uint8_t len){
// Writes an array of bytes into inte the MiRF registers.
    mirf_CSN_lo();
    spi_fast_shift(W_REGISTER | (REGISTER_MASK & reg));
    spi_transmit_sync(value,len);
    mirf_CSN_hi();
}

void mirf_send(uint8_t * value, uint8_t len){
// Sends a data package to the default address. Be sure to send the correct
// amount of bytes as configured as payload on the receiver.
    while (PTX) {}                  // Wait until last packet is send

    mirf_CE_lo();

    PTX = 1;                        // Set to transmitter mode
    TX_POWERUP;                     // Power up
    
    mirf_CSN_lo();                    // Pull down chip select
    spi_fast_shift( FLUSH_TX );     // Write cmd to flush tx fifo
    mirf_CSN_hi();                    // Pull up chip select
    
    mirf_CSN_lo();                    // Pull down chip select
    spi_fast_shift( W_TX_PAYLOAD ); // Write cmd to write payload
    spi_transmit_sync(value,len);   // Write payload
    mirf_CSN_hi();                    // Pull up chip select
    
    mirf_CE_hi();                     // Start transmission
}

