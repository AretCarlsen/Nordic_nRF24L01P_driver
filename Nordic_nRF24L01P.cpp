// Copyright (C) 2010, Aret N Carlsen (aretcarlsen@autonomoustools.com).
// Nordic nRF24L01P comprehensive driver (C++).
// Licensed under GPLv3 and later versions. See license.txt or <http://www.gnu.org/licenses/>.

#include "Nordic_nRF24L01P.hpp"
#include "Nordic_nRF24L01P.inline.hpp"

// Initialize MIRF. Power up, if desired.
template <typename CSN_pin_t, typename CE_pin_t, typename SPI_bus_t> void Nordic_nRF::nRF24L01P<CSN_pin_t, CE_pin_t, SPI_bus_t>::init(bool power_up){
  DEBUGprint_NRF("nRF: init;");

  // Initial pin states.
  CE_pin.set_output_low();
  CSN_pin.set_output_high();

  // Power up, if desired.
  if(power_up) set_power_state(true);

  // Enable features (Auto Ack, Dynamic Payload sizes, etc).
  set_features_enabled(true);
  // Set an initial ARD.
  set_auto_retransmit_delay(ARD_Default);
}

// Write an address to a register.
template <typename CSN_pin_t, typename CE_pin_t, typename SPI_bus_t> void Nordic_nRF::nRF24L01P<CSN_pin_t, CE_pin_t, SPI_bus_t>::write_address(const uint8_t reg, const uint8_t* new_address, uint8_t len){
  // Sanity check
  if(reg > Register_Max) return;
  // Trim size to maximum address size.
  if(len > AddressSize_Max) len = AddressSize_Max; // new_address += AddressSize_Max - len;

  // Begin command.
  CSN_pin.set_output_low();
  // Opcode: write register.
  SPI_bus.transmit(Opcode_WRITE_REGISTER | reg);

  uint8_t i = 0;
  // Write the address data LSBs.
  for(; i < len; i++){
    SPI_bus.transmit(*new_address);
    new_address++;
  }
  // Write the zero'd MSBs, if any.
  for(; i < AddressSize_Max; i++)
    SPI_bus.transmit(0);

  // End command.
  CSN_pin.set_output_high();
}

// Set the transmit address.
template <typename CSN_pin_t, typename CE_pin_t, typename SPI_bus_t> void Nordic_nRF::nRF24L01P<CSN_pin_t, CE_pin_t, SPI_bus_t>::set_TX_address(uint8_t* new_address, const uint8_t len, const bool set_matching_ACK){
  // Write the address. Address length sanity check is performed within write_address().
  write_address(Register_TX_ADDR, new_address, len);

  // Set ACK receive pipe to matching address, if desired.
  if(set_matching_ACK){
    // Set address.
    set_RX_address(new_address, len, ACK_Rx_Pipe);
    // Enable pipe.
    set_RX_pipe_enabled(true, ACK_Rx_Pipe);
  }
}

// Set the receive address for a specific pipe.
template <typename CSN_pin_t, typename CE_pin_t, typename SPI_bus_t> void Nordic_nRF::nRF24L01P<CSN_pin_t, CE_pin_t, SPI_bus_t>::set_RX_address(const uint8_t* new_address, const uint8_t len, bool enable_pipe, uint8_t pipe){
  // Sanity check. Address length sanity check is performed within write_address().
  if(pipe > Pipe_Max) return;
  // Write the address registers. Pipes >= 2 only contain one byte.
  if(pipe >= 2) write_register(Register_RX_ADDR_P0 + pipe, *new_address);
  else write_address(Register_RX_ADDR_P0 + pipe, new_address, len);
  // Enable pipe, if desired.
  if(enable_pipe) set_RX_pipe_enabled(true, pipe);
}

// Queue a packet payload for transmission.
template <typename CSN_pin_t, typename CE_pin_t, typename SPI_bus_t> void Nordic_nRF::nRF24L01P<CSN_pin_t, CE_pin_t, SPI_bus_t>::queue_packet(const uint8_t opcode, const uint8_t* buf, uint8_t buf_len){
  // Begin command.
  CSN_pin.set_output_low();
  // Send opcode.
  SPI_bus.transmit(opcode);
  // Send packet payload.
  SPI_bus.transmit(buf, buf_len);
  // End command.
  CSN_pin.set_output_high();
}

// Receive a packet. Returns packet payload size.
template <typename CSN_pin_t, typename CE_pin_t, typename SPI_bus_t> uint8_t Nordic_nRF::nRF24L01P<CSN_pin_t, CE_pin_t, SPI_bus_t>::receive_packet(uint8_t *buf, uint8_t buf_len){
  // Read the received packet size.
  uint8_t packet_size = read_received_packet_size();
  // Stop if packet is empty.
  if(packet_size == 0) return 0;
  // Invalid packet?  (See datasheet.)
  if(packet_size > PacketSize_Max){
    // This may interrupt ACKs...
    flush_receive_buffer();
    return 0;
  }

  // Set receive size.
  if(buf_len > packet_size) buf_len = packet_size;

  // Receive packet.
  // Begin command.
  CSN_pin.set_output_low();

  // Opcode: Receive RX payload.
  SPI_bus.transmit(Opcode_R_RX_PAYLOAD);
  // Receive the payload response bytes.
  SPI_bus.receive(buf, buf_len);
  // Trim packet size to buffer size.
    // It is not clear whether the *entire* packet needs to be read from the nRF to remove it from the RX FIFO.
    // Until sure (by testing?), read all bytes just in case.
  for(; packet_size > buf_len; packet_size--) SPI_bus.transmit(0);

  // End command.
  CSN_pin.set_output_high();

  // Return the packet size.
  return packet_size;
}

// Receive a packet, including pipe index.
template <typename CSN_pin_t, typename CE_pin_t, typename SPI_bus_t> bool Nordic_nRF::nRF24L01P<CSN_pin_t, CE_pin_t, SPI_bus_t>::receive_packet(ReceivedPacket &packet){
  // Save the received packet pipe.
  packet.rx_pipe = read_received_packet_pipe();
  // Sanity check (e.g. no packet pending).
  if(packet.rx_pipe > Pipe_Max){
    packet.size = 0;
    return false;
  }

  packet.size = receive_packet(packet.buf, packet.buf_len);
  // Sanity check.
  if(packet.size == 0) return false;

  return true;
}

