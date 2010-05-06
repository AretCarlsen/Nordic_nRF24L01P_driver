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

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, 
    EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF 
    MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND 
    NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT 
    HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, 
    WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, 
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER 
    DEALINGS IN THE SOFTWARE.

    $Id$
*/

#pragma once

#include "../globals.hpp"

#include "../../AVRPin/AVRPin.hpp"

// Maximum payload size
#define MAX_PAYLOAD  32
// Total channel count
#define CHANNEL_COUNT 128
// Maximum channel
#define MAX_CHANNEL   (CHANNEL_COUNT-1)

/*
// Example usage:
    Nordic_nRF24L01P mirf(AVROutputPin(DDRB, PB0), AVROutputPin(DDRB, PB1));
*/

// Poll-based. (No interrupts.)
class Nordic_nRF24L01P : public ChannelBus {
  AVRPin CSN_pin;
  AVRPin CE_pin;

// Static definitions
  #include "Nordic_nRF24L01P_definitions.hpp"

public:
  Nordic_nRF24L01P(const AVROutputPin &new_CSN_pin, const AVROutputPin new_CE_pin)
  : CSN_pin(new_CSN_pin), CE_pin(new_CE_pin)
  {
// Initial pin states
    CE_pin.set_low();
    CSN_pin.set_high();

    enable_features(true);
  }

// Strobe the CE pin for the minimum required 10us, e.g. to initiate a packet transmission.
  inline void strobe_CE(){
  // Nothing to do if CE is already high.
    if(CE_pin.get_state()) return;

    CE_pin.set_high();
    // Must be high for at least 10us.
    _delay_us(10);
    CE_pin.set_low();
  }

/* REGISTER CONTROL */

  // Get status.
  inline void read_status(){
  // Status is returned during opcode transmission.
    CSN_pin.set_low();
  // Send an idle opcode.
    uint8_t status = spi_transceive(Opcode_NOP);
    CSN_pin.set_high();
    return status;
  }
  // Write a single register byte.
  inline void write_register(const Register_t reg, const uint8_t value){
    CSN_pin.set_low();
    spi_transceive(Opcode_WRITE_REGISTER | (REGISTER_MASK & reg));
    spi_transceive(value);
    CSN_pin.set_high();
  }
  // Read an array of register bytes.
  inline void write_registers(const Register_t reg, uint8_t *buf, uint8_t len){
    CSN_pin.set_low();
    spi_transceive(Opcode_WRITE_REGISTER | (REGISTER_MASK & reg));
    spi_transmit(buf, len);
    CSN_pin.set_high();
  }
  // OR a single register byte with a value.
  inline void write_register_OR(const Register_t reg, const uint8_t value){
    write_register(reg, read_register(reg) | value);
  }
  // AND a single register byte with a value.
  inline void write_register_AND(const Register_t reg, const uint8_t value){
    write_register(reg, read_register(reg) & value);
  }
  // AND or OR a single register byte with a value, where the choice is based on a bool.
  // (This is intended to set or clear certain bits within a register.)
  inline void write_register_bits(const Register_t reg, const uint8_t mask, bool value){
    if(new_value) write_register_OR(reg, value);
    else write_register_AND(reg, ~value);
  }

  // Read a single register byte.
  inline void read_register(const Register_t reg) const{
    CSN_pin.set_low();
    spi_transceive(Opcode_READ_REGISTER | (REGISTER_MASK & reg));
    spi_transceive(value);
    CSN_pin.set_high();
  }
  // Read an array of register bytes.
  inline void read_registers(const Register_t reg, uint8_t* buf, uint8_t len) const{
    CSN_pin.set_low();
    spi_transceive(Opcode_READ_REGISTER | (REGISTER_MASK & reg));
    spi_receive(buf, len);
    CSN_pin.set_high();
  }

  // Enable all features
  inline void enable_features(bool new_value){
    // Enable dynamic payloads, ack payloads, and per-packet ack disabling
    write_register_bits(Register_FEATURE, Bit_EN_DPL | Bit_EN_ACK_PAY | Bit_EN_DYN_ACK, new_value);
  }
  // True = mode TX, false = mode RX.
  inline void set_mode_TX(bool new_value){
    if(new_value) write_register_AND(Register_CONFIG, ~Bit_PRIM_RX);
    else write_register_OR(Register_CONFIG, Bit_PRIM_RX);
  }
  // True = powered up, False = powered down.
  inline void set_power_state(bool new_value){
    write_register_bits(Register_CONFIG, Bit_PWR_UP, new_value);
  }
  // True = chip enabled (CE high); false = chip disabled (CE low).
  inline void set_chip_enabled(bool new_value){
    CE_pin.set_value(new_value);
  }
  // Enable/disable interrupts. (True = enabled, false = masked.)
  inline void set_interrupt_RXDR(bool new_value){
    // Inverted, because bit actually indicates masked interrupt.
    write_register_bits(Register_CONFIG, Bit_MASK_RX_DR, ~new_value);
  }
  inline void set_interrupt_TXDS(bool new_value){
    write_register_bits(Register_CONFIG, Bit_MASK_TX_DS, ~new_value);
  }
  inline void set_interrupt_MAXRT(bool new_value){
    write_register_bits(Register_CONFIG, Bit_MASK_MAX_RT, ~new_value);
  }
  // Set the number of transmitted CRC bytes.
  inline void set_CRC_bytes(uint8_t new_value){
    // EN_CRC=CRC0=0
    if(new_value == 0) write_register_bits(Register_CONFIG, Bit_EN_CRC | Bit_CRC0, false);
    // EN_CRC=1, CRC0=0
    else if(new_value == 1) write_register(Register_CONFIG, (read_register(Register_CONFIG) & ~Bit_EN_CRC0) | Bit_EN_CRC);
    // EN_CRC=CRC0=1
    else if(new_value == 2) write_register_bits(Register_CONFIG, Bit_EN_CRC | Bit_CRC0, true);
    // Ignore values over 2
  }

/* FIFO CONTROL */
  // In TX mode, add a packet to the outgoing queue. The argument determines whether an ACK is requested.
  // (This opcode is presumably ignored in RX mode.)
  inline void queue_packet(bool ack_requested = true, uint8_t buf, uint8_t buf_len){
    CSN_pin.set_low();
    if(ack_requested) spi_transceive(Opcode_W_TX_PAYLOAD);
    else W_TX_PAYLOAD
    spi_transceive(buf, len); 
    CSN_pin.set_high();
  }
  // Transmit a single packet. Does nothing if transmission is already in progress, in RX mode, or if
  // MIRF is in a constantly transmitting state.
  inline void transmit_packet(){
    strobe_CE();
  }

  inline void receive_packet(uint8_t buf, uint8_t buf_len){
    
  }

  // Check whether the transmission packet FIFO is full.
  inline bool transmit_buffer_full(){
    return (read_status() & Bit_TX_FULL):
  }
  // Check whether the transmission packet FIFO is empty.
  inline bool transmit_buffer_empty(){
    return (read_register(Register_FIFO_STATUS) & Bit_FIFO_TX_FULL);
  }
  // Flush the transmission FIFO.
  inline void transmit_buffer_flush(){
    CSN_pin.set_low();
    spi_transceive(Opcode_FLUSH_TX);
    CSN_pin.set_high();
  }
  // Check whether the reception packet FIFO is full.
  inline bool receive_buffer_full(){
    return (read_register(Register_FIFO_STATUS) & Bit_RX_FULL);
  }
  // Check whether the reception packet FIFO is empty.
  inline bool receive_buffer_empty(){
    return (read_register(Register_FIFO_STATUS) & Bit_RX_EMPTY);
  }
  // Flush the reception FIFO.
  inline void receive_buffer_flush(){
    CSN_pin.set_low();
    spi_transceive(Opcode_FLUSH_RX);
    CSN_pin.set_high();
  }


  void queue_packet(bool no_ack = false){
    
  }
  void set_channel(Channel_t new_channel){
    
  }
};

