// Copyright (C) 2010, Aret N Carlsen (aretcarlsen@autonomoustools.com).
// Nordic nRF24L01P comprehensive driver (C++).
// Licensed under GPLv3 and later versions. See license.txt or <http://www.gnu.org/licenses/>.

#pragma once

#include "../globals.hpp"

#include "../SPI/SPI.hpp"

//template <typename CSN_pin_t, typename CE_pin_t>
template <typename CSN_pin_t, typename CE_pin_t>
class Nordic_nRF24L01P {
private:
  // Driver is poll-based. (No interrupts.) Therefore, only CSN and CE pins are required.
  CSN_pin_t CSN_pin;
  CE_pin_t CE_pin;

public:

  Nordic_nRF24L01P(CSN_pin_t &new_CSN_pin, CE_pin_t &new_CE_pin)
  : CSN_pin(new_CSN_pin), CE_pin(new_CE_pin)
  { }
  Nordic_nRF24L01P()
  { }

  // Register/Bit
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

  // Received packet container.
  struct ReceivedPacket{
    uint8_t rx_pipe;
    uint8_t size;
    uint8_t *buf;
    uint8_t buf_len;
  };

  // Constant definitions.
  #include "Nordic_nRF24L01P_definitions.hpp"

  // Default RX pipe. Not 0, to avoid Ack conflicts.
  static const Pipe_t Pipe_RX_Default = 1;
  // Default auto-retransmit delay, in 250us increments.
  static const ARD_t ARD_Default = 2;
  // Power-up delay, in ms.
  static const uint8_t PowerUp_Delay = 3;

  // Initialize.
  void init(bool power_up = true);

/* COMMUNICATION (pin/register) */
  // Strobe the CE pin, e.g. to initiate a packet transmission or retransmission.
  inline void strobe_CE();

  // Get status.
  inline uint8_t read_status();
  // Write a single register byte.
  inline void write_register(const Register_t reg, const uint8_t value);

  // Read an array of register bytes.
  inline void write_registers(const Register_t reg, uint8_t *buf, uint8_t len);
  // OR a single register byte with a value.
  inline void write_register_OR(const Register_t reg, const uint8_t value){ write_register(reg, read_register(reg) | value); }
  // AND a single register byte with a value.
  inline void write_register_AND(const Register_t reg, const uint8_t value){ write_register(reg, read_register(reg) & value); }
  // AND or OR a single register byte with a value, where the choice is based on a bool.
  // (This is intended to set or clear certain bits within a register.)
  inline void write_register_bits(const Register_t reg, const uint8_t mask, bool value);

  // Read a single register byte.
  inline uint8_t read_register(const Register_t reg) const;
  // Read an array of register bytes.
  inline void read_registers(const Register_t reg, uint8_t* buf, uint8_t len) const;

/* CONFIGURATION CONTROL */
  // Enable all features
    // Enable dynamic payloads, ack payloads, and per-packet ack disabling
  inline void set_features_enabled(bool new_value){ write_register_bits(Register_FEATURE, _BV(Bit_EN_DPL) | _BV(Bit_EN_ACK_PAY) | _BV(Bit_EN_DYN_ACK), new_value); }
  // True = mode TX, false = mode RX.
  inline void set_mode(Mode_t new_value){ write_register_bits(Register_CONFIG, _BV(Bit_PRIM_RX), new_value == Mode_RX); }
  // True = powered up, False = powered down.
  inline void set_power_state(bool new_value);
  // True = chip enabled (CE high); false = chip disabled (CE low).
  inline void set_chip_enabled(bool new_value){ if(new_value) CE_pin.set_output_high(); else CE_pin.set_output_low(); }
  // Enable/disable which interrupts show up on the IRQ pin. (True = enabled, false = masked.)
    // Inverted, because bit actually indicates masked interrupt.
  inline void set_interrupt_pin(Interrupt_t interrupt, bool new_value){ write_register_bits(Register_CONFIG, _BV(Bit_MAX_RT) << interrupt, ~new_value); }
  // Read an interrupt.
  inline bool read_interrupt(Interrupt_t interrupt){ return read_status() & (_BV(Bit_MAX_RT) << interrupt); }
  // Clear an interrupt
    // Interrupts are cleared by writing a 1 to their flags the STATUS register.
  inline void clear_interrupt(Interrupt_t interrupt){ write_register_OR(Register_STATUS, _BV(Bit_MAX_RT) << interrupt); }
  // Clear all three interrupts.
  inline void clear_all_interrupts(){ write_register_bits(Register_STATUS, _BV(Bit_MASK_RX_DR) | _BV(Bit_MASK_TX_DS) | _BV(Bit_MASK_MAX_RT), true); }
  // Check interrupts
  inline bool check_packet_transmitted(){ return read_interrupt(Interrupt_TX_DS); }
  inline void clear_packet_transmitted(){ clear_interrupt(Interrupt_TX_DS); }
  inline bool check_maximum_retries_reached(){ return read_interrupt(Interrupt_MAX_RT); }
  inline void clear_maximum_retries_reached(){ clear_interrupt(Interrupt_MAX_RT); }
  inline bool check_packet_received(){ return read_interrupt(Interrupt_RX_DR); }
  inline void clear_packet_received(){ clear_interrupt(Interrupt_RX_DR); }

/* PACKET CONTROL */
  // Set the number of transmitted CRC bytes.
  // Note that a value of 0 will be clamped to 1 if AutoAcknowledge is enabled.
  inline void set_CRC_size(uint8_t new_value);
  // Configure static payload size for a pipe.
  inline void set_receive_packet_size(const Pipe_t pipe, const PacketSize_t new_packetSize);
  // Activate dynamic payload size for a pipe.
  inline void set_dynamic_payload_allowed(const Pipe_t pipe, bool new_value){ write_register_bits(Register_DYNPD, _BV(Bit_DPL_P5), new_value); }
  // Activate all dynamic payloads
  inline void set_all_dynamic_payloads_allowed(bool new_value);
  // Enable/disable auto ACK for a pipe.
    // Note that auto ACK is incompatible with the nRF24L01 (without the +).
  inline void set_auto_acknowledge(const Pipe_t pipe, const bool new_value);
  // Set the auto-retransmit delay (ARD), in 250us units. This should be at least 500us in many situations.
  inline void set_auto_retransmit_delay(ARD_t new_delay);
  // Set the maximum auto-retransmit count. 0 => disabled.
  inline void set_auto_retransmit_count_limit(uint8_t new_limit);
  // Persists until RF channel is changed.
  inline uint8_t read_lost_packet_count(){ return (read_register(Register_OBSERVE_TX) & BitMask_PLOS_CNT) >> Bit_PLOS_CNT; }
  // Resets on every new packet transmission.
  inline uint8_t read_retransmission_count(){ return (read_register(Register_OBSERVE_TX) & BitMask_ARC_CNT) >> Bit_ARC_CNT; }

/* RF CONTROL */
  // Set the RF send/receive channel.
  inline void set_channel(Channel_t new_channel, const bool clamp_2MHz = false);
  // Set the RF send/receive frequency, in MHz.
  inline void set_frequency(Frequency_t new_frequency);
  // Set the RF gain. Provided value is rounded down to next lowest allowed value.
  inline void set_gain(Gain_t new_gain);
   // Set the data rate to one of the allowed values.
  inline void set_data_rate(Rate_t new_rate);

  // Received Power Detector (RPD)
  bool read_received_power_detector(){ return read_register(Register_RPD); }

/* ADDRESS CONTROL */
  inline void set_address_size(uint8_t new_size);
  void write_address(const uint8_t reg, const uint8_t* new_address, uint8_t len);
  // Set the transmission address from an array of bytes.
  // The second argument determines whether RX pipe 0 is set to a matching address s.t. ACKs can be received.
  void set_TX_address(uint8_t* new_address, const uint8_t len, const bool set_matching_ACK = true);
  // Set transmission address from a 32-bit unsigned int, rather than an array.
  inline void set_TX_address(uint32_t const &new_address, bool set_matching_ACK = true);
  // Set the receive address for a pipe from an array of bytes.
  void set_RX_address(const uint8_t* new_address, const uint8_t len, const bool enable_pipe = true, uint8_t pipe = Pipe_RX_Default);
  // Set receive address from a 32-bit unsigned int, rather than an array.
  inline void set_RX_address(uint32_t const &new_address, const bool enable_pipe = true, const uint8_t pipe = Pipe_RX_Default);
  inline void set_RX_pipe_enabled(const bool new_value, const uint8_t pipe = Pipe_RX_Default);

/* FIFO CONTROL */
  void queue_packet(const uint8_t opcode, const uint8_t* buf, uint8_t buf_len);
  // In TX mode, add a packet to the outgoing queue. The argument determines whether an ACK is requested.
  // In RX mode, add a packet to the ACK-payload outgoing queue. (Ack_requested argument is ignored.)
  inline void queue_TX_packet(const uint8_t* buf, uint8_t buf_len, const bool ack_requested = true);
  // Queue an outgoing RX packet (ACK payload), associated with a specific address/pipe.
  inline void queue_RX_packet(const uint8_t* buf, uint8_t buf_len, const Pipe_t rx_pipe = Pipe_RX_Default);
  // Transmit a single packet. Does nothing if transmission is already in progress, in RX mode, or if
  // MIRF is in a constantly transmitting state.
  inline void transmit_packet(){ strobe_CE(); }
  // Reuse the last transmitted payload.
  // Reuse continues until queue_TX_packet() or flush_transmit_buffer() is called.
  inline void transmit_reused();

  // Get the size of the packet in the front of the RX FIFO.
  // Returns 0 if no packet?
  inline uint8_t read_received_packet_size();
  // Receive a packet. Returns packet payload size.
  uint8_t receive_packet(uint8_t *buf, uint8_t buf_len = PacketSize_Max);
  // Receive a packet. Returns the packet size (or 0 if none received).
  // Will read buf_len or packet_size bytes, whichever is shorter.
  bool receive_packet(ReceivedPacket &packet);

  // Check whether the transmission packet FIFO is full.
  inline bool transmit_buffer_full(){ return (read_status() & _BV(Bit_STATUS_TX_FULL)); }
  // Check whether the transmission packet FIFO is empty.
  inline bool transmit_buffer_empty(){ return (read_register(Register_FIFO_STATUS) & _BV(Bit_FIFO_TX_FULL)); }
  // Flush the transmission FIFO.
  inline void flush_transmit_buffer();

  // Check whether the reception packet FIFO is full.
  inline bool receive_buffer_full(){ return (read_register(Register_FIFO_STATUS) & _BV(Bit_RX_FULL)); }
  // Check which RX pipe a received packet (at the front of the FIFO) is
  // associated with. Returns a nonsensical value (7) if no packet
  // is pending.
  inline uint8_t read_received_packet_pipe(){ return (read_status() & BitMask_RX_P_NO) >> Bit_RX_P_NO; }
  // Check whether the reception packet FIFO is empty.
  inline bool receive_buffer_empty(){ return ((read_status() & BitMask_RX_P_NO) == BitMask_RX_P_NO_Empty); }
  // Flush the reception FIFO. Note that this may corrupt any ACKs in progress.
  inline void flush_receive_buffer();
};

#include "Nordic_nRF24L01P.inline.hpp"

