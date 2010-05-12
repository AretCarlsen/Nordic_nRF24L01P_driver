#include <util/delay.h>

/* COMMUNICATION (pin/register) */


template <typename CSN_pin_t, typename CE_pin_t> void Nordic_nRF24L01P<CSN_pin_t, CE_pin_t>::strobe_CE(){
  CE_pin.toggle_output();
  // Must be shifted for at least 10us. (At least, it must be for a high pulse.)
  _delay_us(10);
  CE_pin.toggle_output();
}

template <typename CSN_pin_t, typename CE_pin_t> uint8_t Nordic_nRF24L01P<CSN_pin_t, CE_pin_t>::read_status(){
  // Status is returned during opcode transmission.
  CSN_pin.set_output_low();
  // Send an idle opcode.
  uint8_t status = SPI::transceive(Opcode_NOP);
  CSN_pin.set_output_high();
  return status;
}
template <typename CSN_pin_t, typename CE_pin_t> void Nordic_nRF24L01P<CSN_pin_t, CE_pin_t>::write_register(const Register_t reg, const uint8_t value){
  CSN_pin.set_output_low();
  SPI::transceive(Opcode_WRITE_REGISTER | (Opcode_REGISTER_MASK & reg));
  SPI::transceive(value);
  CSN_pin.set_output_high();
}
template <typename CSN_pin_t, typename CE_pin_t> void Nordic_nRF24L01P<CSN_pin_t, CE_pin_t>::write_registers(const Register_t reg, uint8_t *buf, uint8_t len){
  CSN_pin.set_output_low();
  SPI::transceive(Opcode_WRITE_REGISTER | (Opcode_REGISTER_MASK & reg));
  SPI::transmit(buf, len);
  CSN_pin.set_output_high();
}
template <typename CSN_pin_t, typename CE_pin_t> void Nordic_nRF24L01P<CSN_pin_t, CE_pin_t>::write_register_bits(const Register_t reg, const uint8_t mask, bool value){
  uint8_t reg_value = read_register(reg);
  if(value) reg_value |= value;
  else reg_value &= value;
  write_register_OR(reg, reg_value);
}
template <typename CSN_pin_t, typename CE_pin_t> uint8_t Nordic_nRF24L01P<CSN_pin_t, CE_pin_t>::read_register(const Register_t reg) const{
  CSN_pin.set_output_low();
  SPI::transceive(Opcode_READ_REGISTER | (Opcode_REGISTER_MASK & reg));
  // Dummy byte
  uint8_t value = SPI::transceive(0);
  CSN_pin.set_output_high();
  return value;
}
template <typename CSN_pin_t, typename CE_pin_t> void Nordic_nRF24L01P<CSN_pin_t, CE_pin_t>::read_registers(const Register_t reg, uint8_t* buf, uint8_t len) const{
  CSN_pin.set_output_low();
  SPI::transceive(Opcode_READ_REGISTER | (Opcode_REGISTER_MASK & reg));
  SPI::receive(buf, len);
  CSN_pin.set_output_high();
}

/* CONFIGURATION CONTROL */

template <typename CSN_pin_t, typename CE_pin_t> void Nordic_nRF24L01P<CSN_pin_t, CE_pin_t>::set_power_state(bool new_value){
  write_register_bits(Register_CONFIG, _BV(Bit_PWR_UP), new_value);
  if(PowerUp_Delay > 0) _delay_ms(PowerUp_Delay);
}

/* PACKET CONTROL */

template <typename CSN_pin_t, typename CE_pin_t> void Nordic_nRF24L01P<CSN_pin_t, CE_pin_t>::set_CRC_size(uint8_t new_value){
  // Clamp to limits
  if(new_value > CRC_Max) new_value = CRC_Max;

  // Read config, clearing relevant flags.
  uint8_t config = read_register(Register_CONFIG) & ~(_BV(Bit_EN_CRC) | _BV(Bit_CRCO));
  // For CRC size 0, no change
  // For CRC size 1, EN_CRC=1, CRCO=0
  if(new_value == 1) config |= _BV(Bit_EN_CRC);
  // For CRC size 2, EN_CRC=CRCO=1
  else if(new_value == 2) config |= _BV(Bit_EN_CRC) | _BV(Bit_CRCO);

  write_register(Register_CONFIG, config);
}
template <typename CSN_pin_t, typename CE_pin_t> void Nordic_nRF24L01P<CSN_pin_t, CE_pin_t>::set_receive_packet_size(const Pipe_t pipe, const PacketSize_t new_packetSize){
// Sanity checks
  if(pipe > Pipe_Max || new_packetSize > PacketSize_Max) return;

  write_register(Register_RX_PW_P0 + pipe, new_packetSize);
}
template <typename CSN_pin_t, typename CE_pin_t> void Nordic_nRF24L01P<CSN_pin_t, CE_pin_t>::set_all_dynamic_payloads_allowed(bool new_value){
  write_register_bits(Register_DYNPD, _BV(Bit_DPL_P5) | _BV(Bit_DPL_P4) | _BV(Bit_DPL_P3) | _BV(Bit_DPL_P2) | _BV(Bit_DPL_P1) | _BV(Bit_DPL_P0), new_value);
}
template <typename CSN_pin_t, typename CE_pin_t> void Nordic_nRF24L01P<CSN_pin_t, CE_pin_t>::set_auto_acknowledge(const Pipe_t pipe, const bool new_value){
  // Sanity check
  if(pipe > Pipe_Max) return;
  // Could use a switch statement to pick up the individual definitions.
  // However, they are all lined up in the register anyway, and the sanity check already happened.
  write_register_bits(Register_EN_AA, _BV(Bit_ENAA_P0) << pipe, new_value);
}
template <typename CSN_pin_t, typename CE_pin_t> void Nordic_nRF24L01P<CSN_pin_t, CE_pin_t>::set_auto_retransmit_delay(ARD_t new_delay){
  if(new_delay > AutoRetransmitDelay_Max) new_delay = AutoRetransmitDelay_Max;
  write_register(Register_SETUP_RETR, (read_register(Register_SETUP_RETR) & ~BitMask_ARD) | (new_delay << Bit_ARD));
}
template <typename CSN_pin_t, typename CE_pin_t> void Nordic_nRF24L01P<CSN_pin_t, CE_pin_t>::set_auto_retransmit_count_limit(uint8_t new_limit){
  // Clamp to limits
  if(new_limit > AutoRetransmitCount_Max) new_limit = AutoRetransmitCount_Max;
  write_register(Register_SETUP_RETR, (read_register(Register_SETUP_RETR) & ~BitMask_ARC) | (new_limit << Bit_ARC));
}

/* RF CONTROL */

template <typename CSN_pin_t, typename CE_pin_t> void Nordic_nRF24L01P<CSN_pin_t, CE_pin_t>::set_channel(Channel_t new_channel, const bool clamp_2MHz){
  // Clamp to max channel.
  if(new_channel > Channel_Max) new_channel = Channel_Max;
  // Clamp to 2MHz channel width. Assumes that MIN and MAX channels are even.
  else if(clamp_2MHz) new_channel -= new_channel % 2;
  write_register(Register_RF_CH, new_channel);
}
template <typename CSN_pin_t, typename CE_pin_t> void Nordic_nRF24L01P<CSN_pin_t, CE_pin_t>::set_frequency(Frequency_t new_frequency){
  // Clamp to limits.
  if(new_frequency < Frequency_Min) new_frequency = Frequency_Min;
  else if(new_frequency > Frequency_Max) new_frequency = Frequency_Max;
  // Channel is MHz above Frequency_Min (2400MHz). For instance, 2405MHz is channel 5.
  set_channel(new_frequency - Frequency_Min);
}
template <typename CSN_pin_t, typename CE_pin_t> void Nordic_nRF24L01P<CSN_pin_t, CE_pin_t>::set_gain(Gain_t new_gain){
  // Clamp to limits.
  if(new_gain < Gain_Min) new_gain = Gain_Min;
  else if(new_gain > Gain_Max) new_gain = Gain_Max;

  // Translate into nonnegative integer space and scale.
  write_register(Register_RF_SETUP, ((new_gain - Gain_Min) / 6) << Bit_RF_PWR);
}
template <typename CSN_pin_t, typename CE_pin_t> void Nordic_nRF24L01P<CSN_pin_t, CE_pin_t>::set_data_rate(const Rate_t new_rate){
  // Clear both bits.
  uint8_t rf_setup = read_register(Register_RF_SETUP) & ~(_BV(Bit_RF_DR_HIGH) | _BV(Bit_RF_DR_LOW));
  // For rate 250K, set DR_LOW.
  if(new_rate == Rate_250K) rf_setup |= _BV(Bit_RF_DR_LOW);
  // For rate 2M, set DR_HIGH.
  else if(new_rate == Rate_2M) rf_setup |= _BV(Bit_RF_DR_HIGH);
  // For rate 1M, leave both bits cleared.

  // Write out config.
  write_register(Register_RF_SETUP, rf_setup);
}

/* ADDRESS CONTROL */

template <typename CSN_pin_t, typename CE_pin_t> void Nordic_nRF24L01P<CSN_pin_t, CE_pin_t>::set_address_size(uint8_t new_size){
  // Clamp to limits
  if(new_size < AddressSize_Min) new_size = AddressSize_Min;
  else if(new_size > AddressSize_Max) new_size = AddressSize_Max;
  // Register contains address size, offset by 1.
  write_register(Register_SETUP_AW, new_size + 1);
}
template <typename CSN_pin_t, typename CE_pin_t> void Nordic_nRF24L01P<CSN_pin_t, CE_pin_t>::set_TX_address(const uint32_t &new_address, const bool set_matching_ACK){
  // Presumes little-endian integer storage.
  set_TX_address((uint8_t*) &new_address, sizeof(uint32_t), set_matching_ACK);
}
template <typename CSN_pin_t, typename CE_pin_t> void Nordic_nRF24L01P<CSN_pin_t, CE_pin_t>::set_RX_address(const uint32_t &new_address, const bool enable_pipe, const uint8_t pipe){
  // Presumes little-endian integer storage.
  set_RX_address((uint8_t*) &new_address, sizeof(uint32_t), enable_pipe, pipe);
}
template <typename CSN_pin_t, typename CE_pin_t> void Nordic_nRF24L01P<CSN_pin_t, CE_pin_t>::set_RX_pipe_enabled(const bool new_value, const uint8_t pipe){
  // Sanity check
  if(pipe > Pipe_Max) return;
  // Could use a switch statement to pick up the individual definitions.
  // However, they are all lined up in the register anyway, and the sanity check already happened.
  write_register_bits(Register_EN_RXADDR, Bit_ERX_P0 << pipe, new_value);
}

/* FIFO CONTROL */

template <typename CSN_pin_t, typename CE_pin_t> void Nordic_nRF24L01P<CSN_pin_t, CE_pin_t>::queue_TX_packet(const uint8_t* buf, uint8_t buf_len, const bool ack_requested){
  // Clamp packet size to limit.
  if(buf_len > PacketSize_Max) buf_len = PacketSize_Max;

  // Opcode depends on ACK preference.
  queue_packet(ack_requested? Opcode_W_TX_PAYLOAD : Opcode_W_TX_PAYLOAD_NOACK, buf, buf_len);
}
template <typename CSN_pin_t, typename CE_pin_t> void Nordic_nRF24L01P<CSN_pin_t, CE_pin_t>::queue_RX_packet(const uint8_t* buf, uint8_t buf_len, const Pipe_t rx_pipe){
  // Reject if RX pipe out of bounds.
  if(rx_pipe > Pipe_Max) return;
  // Clamp packet size to limit.
  if(buf_len > PacketSize_Max) buf_len = PacketSize_Max;

  queue_packet(Opcode_W_ACK_PAYLOAD | rx_pipe, buf, buf_len);
}
template <typename CSN_pin_t, typename CE_pin_t> void Nordic_nRF24L01P<CSN_pin_t, CE_pin_t>::transmit_reused(){
  CSN_pin.set_output_low();
  SPI::transceive(Opcode_REUSE_TX_PL);
  CSN_pin.set_output_high();
}
template <typename CSN_pin_t, typename CE_pin_t> uint8_t Nordic_nRF24L01P<CSN_pin_t, CE_pin_t>::read_received_packet_size(){
  CSN_pin.set_output_low();
  uint8_t size = SPI::transceive(Opcode_R_RX_PL_WID);
  CSN_pin.set_output_high();
  return size;
}
template <typename CSN_pin_t, typename CE_pin_t> void Nordic_nRF24L01P<CSN_pin_t, CE_pin_t>::flush_transmit_buffer(){
  CSN_pin.set_output_low();
  SPI::transceive(Opcode_FLUSH_TX);
  CSN_pin.set_output_high();
}
template <typename CSN_pin_t, typename CE_pin_t> void Nordic_nRF24L01P<CSN_pin_t, CE_pin_t>::flush_receive_buffer(){
  CSN_pin.set_output_low();
  SPI::transceive(Opcode_FLUSH_RX);
  CSN_pin.set_output_high();
}

