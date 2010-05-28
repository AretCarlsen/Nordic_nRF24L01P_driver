// Copyright (C) 2010, Aret N Carlsen (aretcarlsen@autonomoustools.com).
// Nordic Nordic_nRF::nRF24L01P comprehensive driver (C++).
// Licensed under GPLv3 and later versions. See license.txt or <http://www.gnu.org/licenses/>.

#pragma once

#include <util/delay.h>

/* COMMUNICATION (pin/register) */

// Toggle the CE pin state for the minimum period required for packet transmission/retransmission etc.
template <typename CSN_pin_t, typename CE_pin_t, typename SPI_bus_t> void Nordic_nRF::nRF24L01P<CSN_pin_t, CE_pin_t, SPI_bus_t>::strobe_CE(){
  CE_pin.toggle_output();
  // Must be shifted for at least 10us. (At least, it must be for a high pulse. The retransmit pulse minimum length is not specified)
  _delay_us(10);
  CE_pin.toggle_output();
}

// Read the status register. This register can be accessed more quickly than other registers,
// as it is returned as the opcode is transmitted. (The NOP opcode is used for this purpose.)
template <typename CSN_pin_t, typename CE_pin_t, typename SPI_bus_t> uint8_t Nordic_nRF::nRF24L01P<CSN_pin_t, CE_pin_t, SPI_bus_t>::read_status(){
  // Begin command.
  CSN_pin.set_output_low();
  // Send an idle opcode. Status is returned during opcode transmission.
  uint8_t status = SPI_bus.transceive(Opcode_NOP);
  // End command.
  CSN_pin.set_output_high();
  // Return the opcode.
  DEBUGprint_NRF("Rdreg st val X%x;", status);
  return status;
}
// Get the size of a register (in bytes).
template <typename CSN_pin_t, typename CE_pin_t, typename SPI_bus_t> uint8_t Nordic_nRF::nRF24L01P<CSN_pin_t, CE_pin_t, SPI_bus_t>::register_size(const Register_t reg) const{
  // Most registers are one byte.
  uint8_t regLen = 1;
  // RX0, RX1, and TX address registers are 5 bytes.
  if(reg == 0x0A || reg == 0x0B || reg == 0x10) regLen = 5;
  return regLen;
}
// Write a register.
template <typename CSN_pin_t, typename CE_pin_t, typename SPI_bus_t> void Nordic_nRF::nRF24L01P<CSN_pin_t, CE_pin_t, SPI_bus_t>::write_register(const Register_t reg, const uint8_t value){
  // Sanity check
  if(reg > Register_Max) return;

  // Begin command.
  CSN_pin.set_output_low();
  // Opcode: write register. Argument: register index.
  SPI_bus.transceive(Opcode_WRITE_REGISTER | reg);
  // Argument: new register value.
  SPI_bus.transceive(value);
  // End command.
  CSN_pin.set_output_high();
}
// Write a series of registers.
template <typename CSN_pin_t, typename CE_pin_t, typename SPI_bus_t> void Nordic_nRF::nRF24L01P<CSN_pin_t, CE_pin_t, SPI_bus_t>::write_registers(const Register_t reg, uint8_t *buf, uint8_t len){
  // Sanity check
  if(reg > Register_Max) return;

  // Begin command.
  CSN_pin.set_output_low();
  // Opcode: write register. Argument: register index.
  SPI_bus.transceive(Opcode_WRITE_REGISTER | reg);
  // Arguments: new register values.
  SPI_bus.transmit(buf, len);
  // End command.
  CSN_pin.set_output_high();
}
// Set or clear multiple register bits. The bits in 'mask' are set or cleared on the basis of 'value'.
template <typename CSN_pin_t, typename CE_pin_t, typename SPI_bus_t> void Nordic_nRF::nRF24L01P<CSN_pin_t, CE_pin_t, SPI_bus_t>::write_register_bits(const Register_t reg, const uint8_t mask, bool value){
  // Read the current register value.
  uint8_t reg_value = read_register(reg);
  // Set or clear the bits in 'mask'.
  if(value) reg_value |= mask;
  else reg_value &= ~mask;
  // Write the new register value.
  write_register_OR(reg, reg_value);
}
// Read a register.
template <typename CSN_pin_t, typename CE_pin_t, typename SPI_bus_t> uint8_t Nordic_nRF::nRF24L01P<CSN_pin_t, CE_pin_t, SPI_bus_t>::read_register(const Register_t reg) const{
  // Sanity check
  if(reg > Register_Max) return 0;

  // Begin command
  CSN_pin.set_output_low();
  // Opcode: read register. Argument: register index.
  SPI_bus.transceive(Opcode_READ_REGISTER | reg);
  // Receive the register value.
  uint8_t value = SPI_bus.receive();
  // End command
  CSN_pin.set_output_high();
  DEBUGprint_NRF("Rdreg %d val X%x;", reg, value);
  return value;
}
// Read a series of registers.
template <typename CSN_pin_t, typename CE_pin_t, typename SPI_bus_t> void Nordic_nRF::nRF24L01P<CSN_pin_t, CE_pin_t, SPI_bus_t>::read_registers(const Register_t reg, uint8_t* buf, uint8_t len) const{
  // Sanity check
  if(reg > Register_Max) return;

  // Begin command.
  CSN_pin.set_output_low();
  // Opcode: read register. Argument: register index.
  SPI_bus.transceive(Opcode_READ_REGISTER | reg);
  // Receive the register values.
  SPI_bus.receive(buf, len);
  // End command.
  CSN_pin.set_output_high();
}
// Buffer must be of size DumpSize (36B).
template <typename CSN_pin_t, typename CE_pin_t, typename SPI_bus_t> void Nordic_nRF::nRF24L01P<CSN_pin_t, CE_pin_t, SPI_bus_t>::dump_registers(uint8_t* buf) const{
  for(uint8_t reg = 0; reg <= Register_Max; reg++){
  // Registers 0x18 through 0x1B cannot be accessed with R_REGISTER.
    if(reg == 0x18) reg = 0x1C;

    uint8_t regLen = register_size(reg);
/*
    if(regLen == 1){
      *buf = read_register(reg);
      buf++;
    }else{
*/
    read_registers(reg, buf, regLen);
    buf += regLen;
  }
}

/* CONFIGURATION CONTROL */

// Set the current power state (powered up or powered down).
template <typename CSN_pin_t, typename CE_pin_t, typename SPI_bus_t> void Nordic_nRF::nRF24L01P<CSN_pin_t, CE_pin_t, SPI_bus_t>::set_power_state(bool new_value){
  // Set or clear the PWR_UP bit in the CONFIG register.
  write_register_bits(Register_CONFIG, _BV(Bit_PWR_UP), new_value);
  // If powering up, delay for the proper number of milliseconds to allow the transmitter clock to settle.
  if((PowerUp_Delay > 0) && new_value) _delay_ms(PowerUp_Delay);
}
// Enable/disable dynamic payloads, ack payloads, and per-packet ack disabling.
template <typename CSN_pin_t, typename CE_pin_t, typename SPI_bus_t> void Nordic_nRF::nRF24L01P<CSN_pin_t, CE_pin_t, SPI_bus_t>::set_features_enabled(bool new_value){
  // Set/clear the bits in the FEATURE register.
  write_register_bits(Register_FEATURE, _BV(Bit_EN_DPL) | _BV(Bit_EN_ACK_PAY) | _BV(Bit_EN_DYN_ACK), new_value);
  // Enable/disable dynamic payloads in all pipes.
  set_all_dynamic_payloads_allowed(new_value);
}

/* PACKET CONTROL */

// Set the size of the per-packet CRC.
template <typename CSN_pin_t, typename CE_pin_t, typename SPI_bus_t> void Nordic_nRF::nRF24L01P<CSN_pin_t, CE_pin_t, SPI_bus_t>::set_CRC_size(uint8_t new_value){
  // Clamp to CRC size limits.
  if(new_value > CRC_Max) new_value = CRC_Max;

  // Read the CONFIG register, clearing the CRC flags.
  uint8_t config = read_register(Register_CONFIG) & ~(_BV(Bit_EN_CRC) | _BV(Bit_CRCO));
  // For CRC size 0, no change
  // For CRC size 1, EN_CRC=1, CRCO=0
  if(new_value == 1) config |= _BV(Bit_EN_CRC);
  // For CRC size 2, EN_CRC=CRCO=1
  else if(new_value == 2) config |= _BV(Bit_EN_CRC) | _BV(Bit_CRCO);

  // Write the CONFIG register.
  write_register(Register_CONFIG, config);
}
// Set the static receive packet size. Only relevant if dynamic payloads are not enabled.
template <typename CSN_pin_t, typename CE_pin_t, typename SPI_bus_t> void Nordic_nRF::nRF24L01P<CSN_pin_t, CE_pin_t, SPI_bus_t>::set_receive_packet_size(const Pipe_t pipe, const PacketSize_t new_packetSize){
// Sanity checks
  if((pipe > Pipe_Max) || (new_packetSize > PacketSize_Max)) return;

// Set the new size.
  write_register(Register_RX_PW_P0 + pipe, new_packetSize);
}
// Activate dynamic payload size for a pipe.
template <typename CSN_pin_t, typename CE_pin_t, typename SPI_bus_t> void Nordic_nRF::nRF24L01P<CSN_pin_t, CE_pin_t, SPI_bus_t>::set_dynamic_payload_allowed(const Pipe_t pipe, bool new_value){
  // Sanity check
  if(pipe > Pipe_Max) return;

  // Take advantage of the nicely lined-up bits in the DYNPD register.
  write_register_bits(Register_DYNPD, _BV(Bit_DPL_P0) << pipe, new_value);
}
// Enable/disable dynamic payloads for all pipes.
template <typename CSN_pin_t, typename CE_pin_t, typename SPI_bus_t> void Nordic_nRF::nRF24L01P<CSN_pin_t, CE_pin_t, SPI_bus_t>::set_all_dynamic_payloads_allowed(bool new_value){
  // Set/clear all bits at once.
  write_register_bits(Register_DYNPD, _BV(Bit_DPL_P5) | _BV(Bit_DPL_P4) | _BV(Bit_DPL_P3) | _BV(Bit_DPL_P2) | _BV(Bit_DPL_P1) | _BV(Bit_DPL_P0), new_value);
}
// Enable/disable auto acknowledge for a specific pipe.
template <typename CSN_pin_t, typename CE_pin_t, typename SPI_bus_t> void Nordic_nRF::nRF24L01P<CSN_pin_t, CE_pin_t, SPI_bus_t>::set_auto_acknowledge(const Pipe_t pipe, const bool new_value){
  // Sanity check
  if(pipe > Pipe_Max) return;

  // Take advantage of the nicely lined-up bits in the EN_AA register.
  write_register_bits(Register_EN_AA, _BV(Bit_ENAA_P0) << pipe, new_value);
}
// Set the auto retransmit delay. Argument is in 250us units.
template <typename CSN_pin_t, typename CE_pin_t, typename SPI_bus_t> void Nordic_nRF::nRF24L01P<CSN_pin_t, CE_pin_t, SPI_bus_t>::set_auto_retransmit_delay(ARD_t new_delay){
// Clamp to maximum delay.
  if(new_delay > AutoRetransmitDelay_Max) new_delay = AutoRetransmitDelay_Max;
// Write the SETUP_RETR register with the new delay. Mask the current value to keep other bit values as they are.
  write_register(Register_SETUP_RETR, (read_register(Register_SETUP_RETR) & ~BitMask_ARD) | (new_delay << Bit_ARD));
}
// Set the auto retransmit count limit.
template <typename CSN_pin_t, typename CE_pin_t, typename SPI_bus_t> void Nordic_nRF::nRF24L01P<CSN_pin_t, CE_pin_t, SPI_bus_t>::set_auto_retransmit_count_limit(uint8_t new_limit){
  // Clamp to maximum retransmit count.
  if(new_limit > AutoRetransmitCount_Max) new_limit = AutoRetransmitCount_Max;
// Write the SETUP_RETR register with the new count. Mask the current value to keep other bit values as they are.
  write_register(Register_SETUP_RETR, (read_register(Register_SETUP_RETR) & ~BitMask_ARC) | (new_limit << Bit_ARC));
}

/* RF CONTROL */

// Set the current RF channel.
template <typename CSN_pin_t, typename CE_pin_t, typename SPI_bus_t> void Nordic_nRF::nRF24L01P<CSN_pin_t, CE_pin_t, SPI_bus_t>::set_channel(Channel_t new_channel, const bool clamp_2MHz){
  // Clamp to max channel.
  if(new_channel > Channel_Max) new_channel = Channel_Max;
  // Clamp to 2MHz channel width, if requested. Assumes that MIN and MAX channels are even.
  else if(clamp_2MHz) new_channel -= new_channel % 2;
  // Write the RF_CH register.
  write_register(Register_RF_CH, new_channel);
}
// Set the current RF frequency. (Like set_channel(), but with MHz units.)
template <typename CSN_pin_t, typename CE_pin_t, typename SPI_bus_t> void Nordic_nRF::nRF24L01P<CSN_pin_t, CE_pin_t, SPI_bus_t>::set_frequency(Frequency_t new_frequency){
  // Clamp to frequency limits.
  if(new_frequency < Frequency_Min) new_frequency = Frequency_Min;
  else if(new_frequency > Frequency_Max) new_frequency = Frequency_Max;
  // Channel is MHz above Frequency_Min (2400MHz). For instance, 2405MHz is channel 5.
  set_channel(new_frequency - Frequency_Min);
}
// Set the current gain in dBm. Note that only certain quantized values can actually be set; this method rounds _down_.
template <typename CSN_pin_t, typename CE_pin_t, typename SPI_bus_t> void Nordic_nRF::nRF24L01P<CSN_pin_t, CE_pin_t, SPI_bus_t>::set_gain(Gain_t new_gain){
  // Clamp to limits.
  if(new_gain < Gain_Min) new_gain = Gain_Min;
  else if(new_gain > Gain_Max) new_gain = Gain_Max;

  // Translate into nonnegative integer space and scale.
  write_register(Register_RF_SETUP, ((new_gain - Gain_Min) / 6) << Bit_RF_PWR);
}
// Set the current data rate to an enumerated value.
template <typename CSN_pin_t, typename CE_pin_t, typename SPI_bus_t> void Nordic_nRF::nRF24L01P<CSN_pin_t, CE_pin_t, SPI_bus_t>::set_data_rate(const Rate_t new_rate){
  // Read the register, clearing both bits.
  uint8_t rf_setup = read_register(Register_RF_SETUP) & ~(_BV(Bit_RF_DR_HIGH) | _BV(Bit_RF_DR_LOW));
  // For rate 250K, set DR_LOW.
  if(new_rate == Rate_250K) rf_setup |= _BV(Bit_RF_DR_LOW);
  // For rate 2M, set DR_HIGH.
  else if(new_rate == Rate_2M) rf_setup |= _BV(Bit_RF_DR_HIGH);
  // For rate 1M, leave both bits cleared.

  // Write out RF_SETUP register.
  write_register(Register_RF_SETUP, rf_setup);
}

/* ADDRESS CONTROL */

// Set the address size (in bytes).
template <typename CSN_pin_t, typename CE_pin_t, typename SPI_bus_t> void Nordic_nRF::nRF24L01P<CSN_pin_t, CE_pin_t, SPI_bus_t>::set_address_size(uint8_t new_size){
  // Clamp to limits
  if(new_size < AddressSize_Min) new_size = AddressSize_Min;
  else if(new_size > AddressSize_Max) new_size = AddressSize_Max;
  // Register contains address size, offset by 1.
  write_register(Register_SETUP_AW, new_size + 1);
}
// Set the transmit address from a 32-bit uint.
template <typename CSN_pin_t, typename CE_pin_t, typename SPI_bus_t> void Nordic_nRF::nRF24L01P<CSN_pin_t, CE_pin_t, SPI_bus_t>::set_TX_address(const uint32_t &new_address, const bool set_matching_ACK){
  // Presumes little-endian integer storage under the compilation architecture.
  set_TX_address((uint8_t*) &new_address, sizeof(uint32_t), set_matching_ACK);
}
// Set the receive address for a specific pipe.
template <typename CSN_pin_t, typename CE_pin_t, typename SPI_bus_t> void Nordic_nRF::nRF24L01P<CSN_pin_t, CE_pin_t, SPI_bus_t>::set_RX_address(const uint32_t &new_address, const bool enable_pipe, const uint8_t pipe){
  // Presumes little-endian integer storage.
  set_RX_address((uint8_t*) &new_address, sizeof(uint32_t), enable_pipe, pipe);
}
// Enable a specific RX pipe.
template <typename CSN_pin_t, typename CE_pin_t, typename SPI_bus_t> void Nordic_nRF::nRF24L01P<CSN_pin_t, CE_pin_t, SPI_bus_t>::set_RX_pipe_enabled(const bool new_value, const uint8_t pipe){
  // Sanity check
  if(pipe > Pipe_Max) return;
  // Take advantage of pipe bits lined up nicely in EN_RXADDR.
  write_register_bits(Register_EN_RXADDR, Bit_ERX_P0 << pipe, new_value);
}

/* FIFO CONTROL */

// Queue up a TX packet. Request ack, if desired.
template <typename CSN_pin_t, typename CE_pin_t, typename SPI_bus_t> void Nordic_nRF::nRF24L01P<CSN_pin_t, CE_pin_t, SPI_bus_t>::queue_TX_packet(const uint8_t* buf, uint8_t buf_len, const bool ack_requested){
  // Clamp packet size to upper limit.
  if(buf_len > PacketSize_Max) buf_len = PacketSize_Max;

  // Queue up packet. Opcode depends on ACK preference.
  queue_packet(ack_requested? Opcode_W_TX_PAYLOAD : Opcode_W_TX_PAYLOAD_NOACK, buf, buf_len);
}
// Queue up an RX packet in a specific pipe.
template <typename CSN_pin_t, typename CE_pin_t, typename SPI_bus_t> void Nordic_nRF::nRF24L01P<CSN_pin_t, CE_pin_t, SPI_bus_t>::queue_RX_packet(const uint8_t* buf, uint8_t buf_len, const Pipe_t rx_pipe){
  // Reject if RX pipe out of bounds.
  if(rx_pipe > Pipe_Max) return;
  // Clamp packet size to limit.
  if(buf_len > PacketSize_Max) buf_len = PacketSize_Max;

  // Queue up the RX packet (ACK payload). Opcode: ack payload; argument: pipe.
  queue_packet(Opcode_W_ACK_PAYLOAD | rx_pipe, buf, buf_len);
}
// Retransmit a packet.
template <typename CSN_pin_t, typename CE_pin_t, typename SPI_bus_t> void Nordic_nRF::nRF24L01P<CSN_pin_t, CE_pin_t, SPI_bus_t>::transmit_reused(){
  // Begin command.
  CSN_pin.set_output_low();
  // Opcode: Reuse TX payload.
  SPI_bus.transmit(Opcode_REUSE_TX_PL);
  // End command.
  CSN_pin.set_output_high();
}
// Read the size of the packet in the front of the RX FIFO queue.
template <typename CSN_pin_t, typename CE_pin_t, typename SPI_bus_t> uint8_t Nordic_nRF::nRF24L01P<CSN_pin_t, CE_pin_t, SPI_bus_t>::read_received_packet_size(){
  // Begin command.
  CSN_pin.set_output_low();
  // Opcode: Read received-payload width.
  SPI_bus.transmit(Opcode_R_RX_PL_WID);
  // Receive the width response.
  uint8_t size = SPI_bus.receive();
  // End command.
  CSN_pin.set_output_high();
  // Return payload size.
  return size;
}
template <typename CSN_pin_t, typename CE_pin_t, typename SPI_bus_t> void Nordic_nRF::nRF24L01P<CSN_pin_t, CE_pin_t, SPI_bus_t>::flush_transmit_buffer(){
  // Begin command.
  CSN_pin.set_output_low();
  // Opcode: Flush transmit FIFO.
  SPI_bus.transmit(Opcode_FLUSH_TX);
  // End command.
  CSN_pin.set_output_high();
}
template <typename CSN_pin_t, typename CE_pin_t, typename SPI_bus_t> void Nordic_nRF::nRF24L01P<CSN_pin_t, CE_pin_t, SPI_bus_t>::flush_receive_buffer(){
  // Begin command.
  CSN_pin.set_output_low();
  // Opcode: Flush receive FIFO.
  SPI_bus.transmit(Opcode_FLUSH_RX);
  // End command.
  CSN_pin.set_output_high();
}

