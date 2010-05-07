// Copyright (C) 2010, Aret N Carlsen (aretcarlsen@autonomoustools.com).
// Nordic nRF24L01P comprehensive driver (C++).
// Licensed under GPLv3 and later versions. See license.txt or <http://www.gnu.org/licenses/>.

#pragma once

#include "../globals.hpp"

#include "../../common/SPI/SPI.hpp"
#include "../../common/AVRPin/AVRPin.hpp"

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
class Nordic_nRF24L01P {
  AVROutputPin &CSN_pin;
  AVROutputPin &CE_pin;

public:

  Nordic_nRF24L01P(AVROutputPin &new_CSN_pin, AVROutputPin &new_CE_pin)
  : CSN_pin(new_CSN_pin), CE_pin(new_CE_pin)
  { }

// Initialize
  void init(bool power_up = true){
// Initial pin states
    CE_pin.set_low();
    CSN_pin.set_high();

    if(power_up){
      set_power_state(true);
      _delay_ms(3);
    }

    set_features_enabled(true);
  // Set an initial ARD of 500us.
    set_auto_retransmit_delay(2);
  }

// Register
  typedef uint8_t Register_t;
  typedef uint8_t Bit_t;
  typedef uint8_t BitMask_t;
// SPI opcode
  typedef uint8_t Opcode_t;
// RF channel
  typedef uint8_t Channel_t;
// RF frequency, MHz
  typedef uint16_t Frequency_t;
// RF gain, dBm
  typedef int8_t Gain_t;
// Data rate
  typedef enum { Rate_250K, Rate_1M, Rate_2M } Rate_t;
// Modes
  typedef enum { Mode_RX, Mode_TX } Mode_t;
// Auto retransmit delay
  typedef uint8_t ARD_t;
// Interrupts
  typedef enum { Interrupt_RX_DR = 0, Interrupt_TX_DS = 1, Interrupt_MAX_RT = 2 } Interrupt_t;
// Packet size
  typedef uint8_t PacketSize_t;
// Pipe index
  typedef uint8_t Pipe_t;

// Constant definitions
  #include "Nordic_nRF24L01P_definitions.hpp"


/* PIN CONTROL */

// Strobe the CE pin, e.g. to initiate a packet transmission or retransmission.
  inline void strobe_CE(){
    CE_pin.toggle();
    // Must be shifted for at least 10us. (At least, it must be for a high pulse.)
    _delay_us(10);
    CE_pin.toggle();
  }

/* REGISTER CONTROL */

  // Get status.
  inline uint8_t read_status(){
  // Status is returned during opcode transmission.
    CSN_pin.set_low();
  // Send an idle opcode.
    uint8_t status = SPI::transceive(Opcode_NOP);
    CSN_pin.set_high();
    return status;
  }
  // Write a single register byte.
  inline void write_register(const Register_t reg, const uint8_t value){
    CSN_pin.set_low();
    SPI::transceive(Opcode_WRITE_REGISTER | (Opcode_REGISTER_MASK & reg));
    SPI::transceive(value);
    CSN_pin.set_high();
  }
  // Read an array of register bytes.
  inline void write_registers(const Register_t reg, uint8_t *buf, uint8_t len){
    CSN_pin.set_low();
    SPI::transceive(Opcode_WRITE_REGISTER | (Opcode_REGISTER_MASK & reg));
    SPI::transmit(buf, len);
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
    if(value) write_register_OR(reg, value);
    else write_register_AND(reg, ~value);
  }

  // Read a single register byte.
  inline uint8_t read_register(const Register_t reg) const{
    CSN_pin.set_low();
    SPI::transceive(Opcode_READ_REGISTER | (Opcode_REGISTER_MASK & reg));
    // Dummy byte
    uint8_t value = SPI::transceive(0);
    CSN_pin.set_high();
    return value;
  }
  // Read an array of register bytes.
  inline void read_registers(const Register_t reg, uint8_t* buf, uint8_t len) const{
    CSN_pin.set_low();
    SPI::transceive(Opcode_READ_REGISTER | (Opcode_REGISTER_MASK & reg));
    SPI::receive(buf, len);
    CSN_pin.set_high();
  }

  // Enable all features
  inline void set_features_enabled(bool new_value){
    // Enable dynamic payloads, ack payloads, and per-packet ack disabling
    write_register_bits(Register_FEATURE, _BV(Bit_EN_DPL) | _BV(Bit_EN_ACK_PAY) | _BV(Bit_EN_DYN_ACK), new_value);
  }
  // True = mode TX, false = mode RX.
  inline void set_mode(Mode_t new_value){
    write_register_bits(Register_CONFIG, _BV(Bit_PRIM_RX), new_value == Mode_RX);
  }
  // True = powered up, False = powered down.
  inline void set_power_state(bool new_value){
    write_register_bits(Register_CONFIG, _BV(Bit_PWR_UP), new_value);
  }
  // True = chip enabled (CE high); false = chip disabled (CE low).
  inline void set_chip_enabled(bool new_value){
    CE_pin.set_value(new_value);
  }
  // Enable/disable which interrupts show up on the IRQ pin. (True = enabled, false = masked.)
  inline void set_interrupt_pin(Interrupt_t interrupt, bool new_value){
    // Inverted, because bit actually indicates masked interrupt.
    write_register_bits(Register_CONFIG, _BV(Bit_MAX_RT) << interrupt, ~new_value);
  }
  // Read an interrupt.
  inline bool read_interrupt(Interrupt_t interrupt){
    return read_status() & (_BV(Bit_MAX_RT) << interrupt);
  }
  // Clear an interrupt
  inline void clear_interrupt(Interrupt_t interrupt){
    // Interrupts are cleared by writing a 1 to their flags the STATUS register.
    write_register_OR(Register_STATUS, _BV(Bit_MAX_RT) << interrupt); 
  }
  // Clear all three interrupts.
  inline void clear_all_interrupts(){
    write_register_bits(Register_STATUS, _BV(Bit_MASK_RX_DR) | _BV(Bit_MASK_TX_DS) | _BV(Bit_MASK_MAX_RT), true);
  }
  // Check interrupts
  inline bool check_packet_transmitted(){
    return read_interrupt(Interrupt_TX_DS);
  }
  inline void clear_packet_transmitted(){
    clear_interrupt(Interrupt_TX_DS);
  }
  inline bool check_maximum_retries_reached(){
    return read_interrupt(Interrupt_MAX_RT);
  }
  inline void clear_maximum_retries_reached(){
    clear_interrupt(Interrupt_MAX_RT);
  }
  inline bool check_packet_received(){
    return read_interrupt(Interrupt_RX_DR);
  }
  inline void clear_packet_received(){
    clear_interrupt(Interrupt_RX_DR);
  }

/* PACKET CONTROL */
  // Set the number of transmitted CRC bytes.
  // Note that a value of 0 will be clamped to 1 if AutoAcknowledge is enabled.
  inline void set_CRC_size(uint8_t new_value){
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
  // Configure static payload size for a pipe.
  inline void set_receive_packet_size(const Pipe_t pipe, const PacketSize_t new_packetSize){
  // Sanity checks
    if(pipe > Pipe_Max || new_packetSize > PacketSize_Max) return;

    write_register(Register_RX_PW_P0 + pipe, new_packetSize);
  }
  // Activate dynamic payload size for a pipe.
  inline void set_dynamic_payload_allowed(const Pipe_t pipe, bool new_value){
    write_register_bits(Register_DYNPD, _BV(Bit_DPL_P5), new_value);
  }
  // Activate all dynamic payloads
  inline void set_all_dynamic_payloads_allowed(bool new_value){
    write_register_bits(Register_DYNPD, _BV(Bit_DPL_P5) | _BV(Bit_DPL_P4) | _BV(Bit_DPL_P3) | _BV(Bit_DPL_P2) | _BV(Bit_DPL_P1) | _BV(Bit_DPL_P0), new_value);
  }
  // Enable/disable auto ACK for a pipe.
    // Note that auto ACK is incompatible with the nRF24L01 (without the +).
  inline void set_auto_acknowledge(const Pipe_t pipe, const bool new_value){
  // Sanity check
    if(pipe > Pipe_Max) return;
  // Could use a switch statement to pick up the individual definitions.
  // However, they are all lined up in the register anyway, and the sanity check already happened.
    write_register_bits(Register_EN_AA, _BV(Bit_ENAA_P0) << pipe, new_value);
  }
  // Set the auto-retransmit delay (ARD), in 250us units. This should be at least 500us in many situations.
  inline void set_auto_retransmit_delay(ARD_t new_delay){
    if(new_delay > AutoRetransmitDelay_Max) new_delay = AutoRetransmitDelay_Max;
    write_register(Register_SETUP_RETR, (read_register(Register_SETUP_RETR) & ~BitMask_ARD) | (new_delay << Bit_ARD));
  }
  // Set the maximum auto-retransmit count. 0 => disabled.
  inline void set_auto_retransmit_count_limit(uint8_t new_limit){
    // Clamp to limits
    if(new_limit > AutoRetransmitCount_Max) new_limit = AutoRetransmitCount_Max;
    write_register(Register_SETUP_RETR, (read_register(Register_SETUP_RETR) & ~BitMask_ARC) | (new_limit << Bit_ARC));
  }
  // Persists until RF channel is changed.
  inline uint8_t read_lost_packet_count(){
     return (read_register(Register_OBSERVE_TX) & BitMask_PLOS_CNT) >> Bit_PLOS_CNT;
  }
  // Resets on every new packet transmission.
  inline uint8_t read_retransmission_count(){
     return (read_register(Register_OBSERVE_TX) & BitMask_ARC_CNT) >> Bit_ARC_CNT;
  }

/* RF CONTROL */
  // Set the RF send/receive channel.
  inline void set_channel(Channel_t new_channel, const bool clamp_2MHz = false){
    // Clamp to max channel.
    if(new_channel > Channel_Max) new_channel = Channel_Max;
    // Clamp to 2MHz channel width. Assumes that MIN and MAX channels are even.
    else if(clamp_2MHz) new_channel -= new_channel % 2;
    write_register(Register_RF_CH, new_channel);
  }
  // Set the RF send/receive frequency, in MHz.
  inline void set_frequency(Frequency_t new_frequency){
    // Clamp to limits.
    if(new_frequency < Frequency_Min) new_frequency = Frequency_Min;
    else if(new_frequency > Frequency_Max) new_frequency = Frequency_Max;
  // Channel is MHz above Frequency_Min (2400MHz). For instance, 2405MHz is channel 5.
    set_channel(new_frequency - Frequency_Min);
  }
  // Set the RF gain. Provided value is rounded down to next lowest allowed value.
  inline void set_gain(Gain_t new_gain){
  // Clamp to limits.
    if(new_gain < Gain_Min) new_gain = Gain_Min;
    else if(new_gain > Gain_Max) new_gain = Gain_Max;

  // Translate into nonnegative integer space and scale.
    write_register(Register_RF_SETUP, ((new_gain - Gain_Min) / 6) << Bit_RF_PWR);
  }
  // Set the data rate to one of the allowed values.
  inline void set_data_rate(Rate_t new_rate){
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

  // Received Power Detector (RPD)
  bool read_received_power_detector(){
    return read_register(Register_RPD);
  }

/* ADDRESS CONTROL */
  inline void set_address_size(uint8_t new_size){
  // Clamp to limits
    if(new_size < AddressSize_Min) new_size = AddressSize_Min;
    else if(new_size > AddressSize_Max) new_size = AddressSize_Max;
  // Register contains address size, offset by 1.
    write_register(Register_SETUP_AW, new_size + 1);
  }
  inline void write_address(const uint8_t reg, uint8_t* new_address, uint8_t len){
  // Trim size to maximum address size.
    if(len > AddressSize_Max) len = AddressSize_Max; // new_address += AddressSize_Max - len;

    CSN_pin.set_low();

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

    CSN_pin.set_high();
  }
  // Set the transmission address from an array of bytes.
  // The second argument determines whether RX pipe 0 is set to a matching address s.t. ACKs can be received.
  inline void set_TX_address(uint8_t* new_address, const uint8_t len, const bool set_matching_ACK = true){
    write_address(Register_TX_ADDR, new_address, len);

  // Set ACK receive pipe to matching address, if desired.
    if(set_matching_ACK){
      set_RX_address(ACK_Rx_Pipe, new_address, len);
      set_RX_pipe_enabled(ACK_Rx_Pipe, true);
    }
  }
  // Set transmission address from a 32-bit unsigned int, rather than an array.
  inline void set_TX_address(const uint32_t &new_address, bool set_matching_ACK = true){
  // Presumes little-endian integer storage.
    set_TX_address((uint8_t*) &new_address, sizeof(uint32_t), set_matching_ACK);
  }
  // Set the receive address for a pipe from an array of bytes.
  inline void set_RX_address(uint8_t pipe, uint8_t* const new_address, const uint8_t len){
  // Sanity checks
    if(len > AddressSize_Max || pipe > Pipe_Max) return;
  // Pipes >= 2 only contain one byte.
    if(pipe >= 2) write_register(Register_RX_ADDR_P0 + pipe, *new_address);
    write_address(Register_RX_ADDR_P0 + pipe, new_address, len); 
  }
  // Set receive address from a 32-bit unsigned int, rather than an array.
  inline void set_RX_address(uint8_t pipe, const uint32_t &new_address){
  // Presumes little-endian integer storage.
    set_RX_address(pipe, (uint8_t*) &new_address, sizeof(uint32_t));
  }
  inline void set_RX_pipe_enabled(uint8_t pipe, bool new_value){
  // Sanity check
    if(pipe > Pipe_Max) return;
  // Could use a switch statement to pick up the individual definitions.
  // However, they are all lined up in the register anyway, and the sanity check already happened.
    write_register_bits(Register_EN_RXADDR, Bit_ERX_P0 << pipe, new_value);
  }

/* FIFO CONTROL */
  inline void queue_packet(const uint8_t opcode, const uint8_t *buf, uint8_t buf_len){
    CSN_pin.set_low();
    SPI::transceive(opcode);
  // Send packet payload.
    SPI::transmit(buf, buf_len);
    CSN_pin.set_high();
  }
  // In TX mode, add a packet to the outgoing queue. The argument determines whether an ACK is requested.
  // In RX mode, add a packet to the ACK-payload outgoing queue. (Ack_requested argument is ignored.)
  inline void queue_TX_packet(const uint8_t* buf, uint8_t buf_len, bool ack_requested = true){
    if(buf_len > PacketSize_Max) buf_len = PacketSize_Max;

  // Opcode depends on ACK preference.
    queue_packet(ack_requested? Opcode_W_TX_PAYLOAD : Opcode_W_TX_PAYLOAD_NOACK, buf, buf_len);
  }
  // Queue an outgoing RX packet (ACK payload), associated with a specific address/pipe.
  inline void queue_RX_packet(const uint8_t* buf, uint8_t buf_len, const Pipe_t rx_pipe){
    // Reject if RX pipe out of bounds
    if(rx_pipe > Pipe_Max) return;
    if(buf_len > PacketSize_Max) buf_len = PacketSize_Max;

    queue_packet(Opcode_W_ACK_PAYLOAD | rx_pipe, buf, buf_len);
  }
  // Transmit a single packet. Does nothing if transmission is already in progress, in RX mode, or if
  // MIRF is in a constantly transmitting state.
  inline void transmit_packet(){
    strobe_CE();
  }
  // Reuse the last transmitted payload.
  // Reuse continues until queue_TX_packet() or flush_transmit_buffer() is called.
  inline void transmit_reused(){
    CSN_pin.set_low();
    SPI::transceive(Opcode_REUSE_TX_PL);
    CSN_pin.set_high();
  }

  // Get the size of the packet in the front of the RX FIFO.
  // Returns 0 if no packet?
  inline uint8_t read_received_packet_size(){
    CSN_pin.set_low();
    uint8_t size = SPI::transceive(Opcode_R_RX_PL_WID);
    CSN_pin.set_high(); 
    return size; 
  }
  // Receive a packet. Returns the packet size (or 0 if none received).
  // Will read buf_len or packet_size bytes, whichever is shorter.
  inline uint8_t receive_packet(uint8_t &rx_pipe, uint8_t *buf, uint8_t buf_len = 32){
    uint8_t packet_size = read_received_packet_size();
  // No packet?
    if(packet_size == 0) return 0;
  // Invalid packet?  (See datasheet.)
    if(packet_size > 32){
      // This may interrupt ACKs...
      flush_receive_buffer();
      return 0;
    }

  // Save the received packet pipe.
    rx_pipe = read_received_packet_pipe();

  // Trim packet size to buffer size.
    if(packet_size > buf_len) packet_size = buf_len;
  // Receive packet.
    CSN_pin.set_low();
    SPI::transceive(Opcode_R_RX_PAYLOAD); 
    SPI::receive(buf, buf_len); 
    CSN_pin.set_high();

    return packet_size;
  }
  // rx_pipe default parameter.
  inline uint8_t receive_packet(uint8_t *buf, uint8_t buf_len = 32){
    uint8_t rx_pipe;
    return receive_packet(rx_pipe, buf, buf_len);
  }

  // Check whether the transmission packet FIFO is full.
  inline bool transmit_buffer_full(){
    return (read_status() & _BV(Bit_STATUS_TX_FULL));
  }
  // Check whether the transmission packet FIFO is empty.
  inline bool transmit_buffer_empty(){
    return (read_register(Register_FIFO_STATUS) & _BV(Bit_FIFO_TX_FULL));
  }
  // Flush the transmission FIFO.
  inline void flush_transmit_buffer(){
    CSN_pin.set_low();
    SPI::transceive(Opcode_FLUSH_TX);
    CSN_pin.set_high();
  }

  // Check whether the reception packet FIFO is full.
  inline bool receive_buffer_full(){
    return (read_register(Register_FIFO_STATUS) & _BV(Bit_RX_FULL));
  }
  // Check which RX pipe a received packet (at the front of the FIFO) is
  // associated with. Returns a nonsensical value (7) if no packet
  // is pending.
  inline uint8_t read_received_packet_pipe(){
    return (read_status() & BitMask_RX_P_NO) >> Bit_RX_P_NO;
  }
  // Check whether the reception packet FIFO is empty.
  inline bool receive_buffer_empty(){
    return ((read_status() & BitMask_RX_P_NO) == BitMask_RX_P_NO_Empty);   // read_register(Register_FIFO_STATUS) & _BV(Bit_RX_EMPTY)
  }
  // Flush the reception FIFO. Note that this may corrupt any ACKs in progress.
  inline void flush_receive_buffer(){
    CSN_pin.set_low();
    SPI::transceive(Opcode_FLUSH_RX);
    CSN_pin.set_high();
  }
};

