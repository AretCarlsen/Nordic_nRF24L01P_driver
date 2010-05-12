// Copyright (C) 2010, Aret N Carlsen (aretcarlsen@autonomoustools.com).
// Nordic nRF24L01P comprehensive driver (C++).
// Licensed under GPLv3 and later versions. See license.txt or <http://www.gnu.org/licenses/>.

#include "../globals.hpp"
#include "Nordic_nRF24L01P.hpp"

template <typename CSN_pin_t, typename CE_pin_t> void Nordic_nRF24L01P<CSN_pin_t, CE_pin_t>::init(bool power_up){
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

template <typename CSN_pin_t, typename CE_pin_t> void Nordic_nRF24L01P<CSN_pin_t, CE_pin_t>::write_address(const uint8_t reg, const uint8_t* new_address, uint8_t len){
  // Trim size to maximum address size.
  if(len > AddressSize_Max) len = AddressSize_Max; // new_address += AddressSize_Max - len;

  CSN_pin.set_output_low();

  // Opcode
  SPI::transceive(Opcode_WRITE_REGISTER | (Opcode_REGISTER_MASK & reg));
  uint8_t i = 0;
  // Write the actual data
  for(; i < len; i++){
    SPI::transceive(*new_address);
    new_address++;
  }
  // Write the zero'd MSBs, if any
  for(; i < AddressSize_Max; i++)
    SPI::transceive(0);

  CSN_pin.set_output_high();
}

template <typename CSN_pin_t, typename CE_pin_t> void Nordic_nRF24L01P<CSN_pin_t, CE_pin_t>::set_TX_address(uint8_t* new_address, const uint8_t len, const bool set_matching_ACK){
  write_address(Register_TX_ADDR, new_address, len);

  // Set ACK receive pipe to matching address, if desired.
  if(set_matching_ACK){
    set_RX_address(new_address, len, ACK_Rx_Pipe);
    set_RX_pipe_enabled(true, ACK_Rx_Pipe);
  }
}

template <typename CSN_pin_t, typename CE_pin_t> void Nordic_nRF24L01P<CSN_pin_t, CE_pin_t>::set_RX_address(const uint8_t* new_address, const uint8_t len, bool enable_pipe, uint8_t pipe){
  // Sanity checks
  if(len > AddressSize_Max || pipe > Pipe_Max) return;
  // Pipes >= 2 only contain one byte.
  if(pipe >= 2) write_register(Register_RX_ADDR_P0 + pipe, *new_address);
  write_address(Register_RX_ADDR_P0 + pipe, new_address, len);
  // Enable pipe, if desired.
  if(enable_pipe) set_RX_pipe_enabled(true, pipe);
}

template <typename CSN_pin_t, typename CE_pin_t> void Nordic_nRF24L01P<CSN_pin_t, CE_pin_t>::queue_packet(const uint8_t opcode, const uint8_t* buf, uint8_t buf_len){
  CSN_pin.set_output_low();
  SPI::transceive(opcode);
  // Send packet payload.
  SPI::transmit(buf, buf_len);
  CSN_pin.set_output_high();
}

// Receive a packet. Returns packet payload size.
template <typename CSN_pin_t, typename CE_pin_t> uint8_t Nordic_nRF24L01P<CSN_pin_t, CE_pin_t>::receive_packet(uint8_t *buf, uint8_t buf_len){
  uint8_t packet_size = read_received_packet_size();
  if(packet_size == 0) return 0;
  // Invalid packet?  (See datasheet.)
  if(packet_size > PacketSize_Max){
    // This may interrupt ACKs...
    flush_receive_buffer();
    return 0;
  }

  // Trim packet size to buffer size.
  if(packet_size > buf_len) packet_size = buf_len;

  // Receive packet.
  CSN_pin.set_output_low();
  SPI::transceive(Opcode_R_RX_PAYLOAD);
  SPI::receive(buf, buf_len);
  CSN_pin.set_output_high();

  return packet_size;
}

template <typename CSN_pin_t, typename CE_pin_t> bool Nordic_nRF24L01P<CSN_pin_t, CE_pin_t>::receive_packet(ReceivedPacket &packet){
  // Save the received packet pipe.
  packet.rx_pipe = read_received_packet_pipe();
  // Sanity check (e.g. no packet pending).
  if(packet.rx_pipe > Pipe_Max) return false;

  packet.size = receive_packet(packet.buf, packet.buf_len);
  // Sanity check.
  if(packet.size == 0) return false;

  return true;
}

