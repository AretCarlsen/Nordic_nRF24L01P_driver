// Copyright (C) 2010, Aret N Carlsen (aretcarlsen@autonomoustools.com).
// Nordic nRF24L01P comprehensive driver (C++).
// Licensed under GPLv3 and later versions. See license.txt or <http://www.gnu.org/licenses/>.

// AVR usage example for two devices, one master and one slave.
// Example compilation:
//   avr-g++ -g -Wall -Wundef -Werror -std=c++0x -Os -fno-implicit-templates -ffreestanding -mmcu=atmega328p -DF_CPU=20000000L  -I ~/AT -I . -o USAGE.elf USAGE.cpp
// where ~/AT contains the subdirectories AT_common, AVR_Objects, and Nordic_nRF24L01P_driver

// Comment out this line on the slave device. 
#define MODE_MASTER


// Common definitions, such as uint8_t, from the AT_common code repository.
// You could directly include the relevant <avr/...> libraries instead.
#include <ATcommon/arch/avr/avr.hpp>
// AVR Pin and SPI objects
#include <AVR_Objects/io.hpp>
// nRF24L01P driver
#include <Nordic_nRF24L01P_driver/Nordic_nRF24L01P.hpp>

// Setup nRF pins.
typedef OutPin_B_0 CSN_pin_t;
typedef OutPin_B_1 CE_pin_t;
typedef SPI SPI_bus_t;
typedef Nordic_nRF24L01P::Nordic_nRF24L01P<CSN_pin_t, CE_pin_t, SPI_bus_t> MIRF_t;
// Instantiate template
template class Nordic_nRF24L01P::Nordic_nRF24L01P<CSN_pin_t, CE_pin_t, SPI_bus_t>;

#define RF_CHAN 5
#define SLAVE_ADDRESS_LENGTH 5
static const uint8_t SLAVE_ADDRESS[SLAVE_ADDRESS_LENGTH] = { 0xB3, 0xB4, 0xB5, 0xB6, 0x05 };

void main(){
  uint8_t packet[32];
  uint8_t packet_size;

  // The template arguments are the CSN pin, the CE pin, and the SPI bus.
  MIRF_t mirf;
  // All features are enabled by default, including dynamic payload sizes on all pipes.
  mirf.init();
  // Set RF frequency.
  mirf.set_channel(RF_CHAN);


#ifdef MODE_MASTER
// Example usage, master.
  // Master is in TX mode.
  mirf.set_mode(Nordic_nRF24L01P::Mode_TX);
  // Set TX address to slave. RX pipe 0 will be set to the same address (to receive the auto ACK).
  mirf.set_TX_address(SLAVE_ADDRESS, SLAVE_ADDRESS_LENGTH);
  // Immediately transmit any queued packet.
  mirf.set_chip_enabled(true);
  while(true){
    const uint8_t msg[15] = "Hello (master)";
    // Transmit a packet, enabling auto-ack.
    mirf.queue_TX_packet(msg, 15);
    // Wait for the packet to finish transmitting.
    while(! mirf.check_packet_transmitted());
    // Clear the transmit interrupt.
    mirf.clear_packet_transmitted();
    // Check for an ACK payload
    if(mirf.check_packet_received()){
      // Clear the receive interrupt.
      mirf.clear_packet_received();
      // Retrieve the incoming packet payload.
      packet_size = mirf.receive_packet(packet);
      // 'packet' now contains the ACK payload of size 'packet_size'.
      // Do whatever you want with the contents here, e.g. print out to a UART.
    }
  }

#else
// Example usage, slave.
  // Slave is in RX mode.
  mirf.set_mode(Nordic_nRF24L01P::Mode_RX);
  // Set the receive address (on pipe 0, and enabled the pipe).
  mirf.set_RX_address(SLAVE_ADDRESS, SLAVE_ADDRESS_LENGTH);
  // Enable auto-ack on pipe 0.
  mirf.set_auto_acknowledge(0, true);
  // Begin listening for packets.
  mirf.set_chip_enabled(true);
  while(true){
    const uint8_t msg[14] = "Hello (slave)";
    // Queue an ACK payload.
    mirf.queue_RX_packet(msg, 14);
    // Wait for an incoming packet. (The ACK payload will be transmitted automatically.)
    while(! mirf.check_packet_received());
    // Clear the receive interrupt.
    mirf.clear_packet_received();
    // Retrieve the incoming packet payload.
    packet_size = mirf.receive_packet(packet);
    // 'packet' now contains the ACK payload of size 'packet_size'.
    // Do whatever you want with the contents here, e.g. print out to a UART.
  }
#endif
}

