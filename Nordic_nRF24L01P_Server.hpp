
#include <Upacket/Servers/SimpleServer.hpp>

#ifndef DEBUGprint
#define DEBUGprint(...)
#endif

namespace Nordic_nRF {

template <typename MIRF_t>
class nRF24L01P_Server : public SimpleServer, public Process {
  MIRF_t *mirf;

//  static const uint8_t InitialPacketCapacity = (2 + sizeof(Value_t)+1 + 5);
//  static const uint8_t PacketCapacityIncrement = 10;

public:
  nRF24L01P_Server(MIRF_t *new_mirf, MemoryPool *new_memoryPool, MAP::MAPPacketSink *new_outputPacketSink)
  : SimpleServer(new_memoryPool, new_outputPacketSink), mirf(new_mirf)
  { }

  Status::Status_t process();

private:
  bool replyReceivedPacket(Nordic_nRF::ReceivedPacket &rPacket);
};

template <typename MIRF_t>
bool nRF24L01P_Server<MIRF_t>::replyReceivedPacket(ReceivedPacket &rPacket){
  // Attempt to prepare reply packet (if memory available etc).
  MAP::MAPPacket *replyPacket;

  // Sink string length in C78. Educated guess as to C78 size.
  if(! SimpleServer::prepareReply(&replyPacket, offsetPacket.packet, 1 + rPacket.buf_len)) return false;
  if(! replyPacket->sinkC78(rPacket.buf_len)){ MAP::dereferencePacket(replyPacket); return false; }

  // Sink string itself
  uint8_t *buf = rPacket.buf;
  for(uint8_t buf_len = rPacket.buf_len; buf_len > 0; buf_len--){
    if(! replyPacket->sinkData(*buf)){ MAP::dereferencePacket(replyPacket); return false; }
    buf++;
  }

  // Reference and transmit packet.
  return sendPacket(replyPacket);
}

/*
namespace outside{
inline bool replyBoolean(bool val){
  DEBUGprint_NRF("nB:");
  if(val){
    DEBUGprint_NRF("1");
  }else{
    DEBUGprint_NRF("0");
  }
  return true;
}
}
*/

template <typename MIRF_t>
Status::Status_t nRF24L01P_Server<MIRF_t>::process(){
  // Anything to process?
  if(offsetPacket.packet == NULL) return Status::Status__Good;

  // Make sure data present.
  MAP::Data_t *packetData = offsetPacket.packet->get_data(offsetPacket.headerOffset);
  if(packetData == NULL) return finishedWithPacket();

#define OPCODE(code) } break; case code: {
#define GET_RAW_BOOL(raw_value) bool raw_value; if(! offsetPacket.packet->sourceBool(raw_value, packetData)) return finishedWithPacket(); packetData++
#define GET_RAW_UINT(raw_value) uint32_t raw_value; if(! offsetPacket.packet->sourceC78(raw_value, packetData)) return finishedWithPacket(); packetData++
#define GET_RAW_C78String(strBuf, read_len, max_size) uint8_t strBuf[max_size]; MAP::MAPPacket::Capacity_t read_len; if(! offsetPacket.packet->sourceC78String(strBuf, read_len, max_size, packetData)) return finishedWithPacket(); packetData++
  uint8_t *packetBack = offsetPacket.packet->back();
  uint8_t opcode = *packetData;
  packetData++;
  DEBUGprint_NRF("n-op2:%d;", opcode);
  switch(opcode){
  /* CONFIGURATION CONTROL */
    case 0: { // set_mode(enum)
      GET_RAW_BOOL(raw_value);
      DEBUGprint("n-sM:%d;", raw_value? Nordic_nRF::Mode_TX : Nordic_nRF::Mode_RX);
      mirf->set_mode(raw_value? Nordic_nRF::Mode_TX : Nordic_nRF::Mode_RX);
    OPCODE(1) // set_power_state(bool)
      if(packetData >= packetBack) break;
      mirf->set_power_state(*packetData != 0);
    OPCODE(2) // set_chip_enabled(bool)
      if(packetData >= packetBack) break;
      mirf->set_chip_enabled(*packetData != 0);
    OPCODE(3) // clear_all_interrupts()
      mirf->clear_all_interrupts();
    OPCODE(4) // bool check_packet_transmitted()
      replyBoolean(mirf->check_packet_transmitted());
    OPCODE(5) // clear_packet_transmitted()
      mirf->clear_packet_transmitted();
    OPCODE(6) // bool check_maximum_retries_reached()
      replyBoolean(mirf->check_maximum_retries_reached());
    OPCODE(7) // clear_maximum_retries_reached()
      mirf->clear_maximum_retries_reached();
    OPCODE(8) // bool check_packet_received()
      replyBoolean(mirf->check_packet_received());
    OPCODE(9) // clear_packet_received()
      mirf->clear_packet_received();

  /* PACKET CONTROL */
    OPCODE(10) // set_CRC_size(uint)
      GET_RAW_UINT(raw_value);
      mirf->set_CRC_size(raw_value);
    OPCODE(11) // set_auto_acknowledge(uint, bool)
      uint32_t raw_pipe;  bool raw_value;
      // Retrieve pipe index and new setting.
      if(! offsetPacket.packet->sourceC78(raw_pipe, packetData)) break;
      packetData++;
      if(! offsetPacket.packet->sourceBool(raw_value, packetData)) break;
      mirf->set_auto_acknowledge(raw_pipe, raw_value);
    OPCODE(12) // set_auto_retransmit_delay(uint);  <-- 250us units
      GET_RAW_UINT(raw_value);
      mirf->set_auto_retransmit_delay(raw_value * 250);
    OPCODE(13) // set_auto_retransmit_count_limit(uint8_t new_limit)
      GET_RAW_UINT(raw_value);
      mirf->set_auto_retransmit_count_limit(raw_value);
    OPCODE(14) // uint8_t read_lost_packet_count()
      replyC78(mirf->read_lost_packet_count());
    OPCODE(15) // uint8_t read_retransmission_count()
      replyC78(mirf->read_retransmission_count());

  /* RF CONTROL */
    OPCODE(16) // set_channel(Channel_t new_channel, const bool clamp_2MHz = false)
      uint32_t raw_chan;  bool raw_value;
      if(! offsetPacket.packet->sourceC78(raw_chan, packetData)) break;
      packetData++;
      if(! offsetPacket.packet->sourceBool(raw_value, packetData)) break;
      mirf->set_channel(raw_chan, raw_value);
    OPCODE(17) // set_frequency(Frequency_t new_frequency)   <-- MHz
      GET_RAW_UINT(raw_value);
      mirf->set_frequency(raw_value);
    OPCODE(18) // set_gain(Gain_t new_gain)  <-- dBm (negative)
      GET_RAW_UINT(raw_value);
      mirf->set_gain(raw_value);
    OPCODE(19) // set_data_rate(Rate_t new_rate)  <-- enum
      GET_RAW_UINT(raw_value);
      Nordic_nRF::Rate_t new_rate;
      if(raw_value == 250) new_rate = Nordic_nRF::Rate_250K;
      else if(raw_value == 1000) new_rate = Nordic_nRF::Rate_1M;
      else if(raw_value == 2000) new_rate = Nordic_nRF::Rate_2M;
      else break;
      mirf->set_data_rate(new_rate);
    OPCODE(20) // bool read_received_power_detector()
      replyBoolean(mirf->read_received_power_detector());

 /* ADDRESS CONTROL */
    OPCODE(21) // set_address_size(uint8_t)
      GET_RAW_UINT(raw_value);
      mirf->set_address_size(raw_value);
    OPCODE(22) // set_TX_address(uint8_t* new_address, const uint8_t len, const bool set_matching_ACK = true)
      GET_RAW_C78String(addressBuf, len, MIRF_t::AddressSize_Max);
      GET_RAW_BOOL(setMatchingAck);
      mirf->set_TX_address(addressBuf, len, setMatchingAck);
    OPCODE(23) // set_TX_address(uint32_t const &new_address, bool set_matching_ACK = true)
      GET_RAW_UINT(newAddress);
      // If bool is not present, will be left at true (default).
      bool setMatchingAck = true; offsetPacket.packet->sourceBool(setMatchingAck, packetData);
      mirf->set_TX_address(newAddress, setMatchingAck);
    OPCODE(24) // set_RX_address(const uint8_t* new_address, const uint8_t len, const bool enable_pipe = true, uint8_t pipe = Pipe_RX_Default)
      MAP::MAPPacket::Capacity_t len; uint8_t addressBuf[MIRF_t::AddressSize_Max];
      // Length will be stored in len, up to Max
      if(! offsetPacket.packet->sourceC78String(addressBuf, len, MIRF_t::AddressSize_Max, packetData)) break;
      bool enablePipe = true; uint32_t pipe = MIRF_t::Pipe_RX_Default;
      if(offsetPacket.packet->sourceBool(enablePipe, packetData)){
        packetData++;
        offsetPacket.packet->sourceC78(pipe, packetData);
      }
      mirf->set_RX_address(addressBuf, len, enablePipe, pipe);
    OPCODE(25) // set_RX_address(uint32_t const &new_address, const bool enable_pipe = true, const uint8_t pipe = Pipe_RX_Default)
      GET_RAW_UINT(newAddress);
      // If bool is not present, will be left at true (default).
      bool enablePipe = true; uint32_t pipe = MIRF_t::Pipe_RX_Default;
      if(offsetPacket.packet->sourceBool(enablePipe, packetData)){
        packetData++;
        offsetPacket.packet->sourceC78(pipe, packetData);
      }
      mirf->set_RX_address(newAddress, enablePipe, pipe);
    OPCODE(26) // set_RX_pipe_enabled(const bool new_value, const uint8_t pipe = Pipe_RX_Default)
      GET_RAW_BOOL(new_value);
      uint32_t pipe = MIRF_t::Pipe_RX_Default;  offsetPacket.packet->sourceC78(pipe, packetData);
      mirf->set_RX_pipe_enabled(new_value, pipe);

  /* FIFO CONTROL */
    OPCODE(27) // queue_TX_packet(const uint8_t* buf, uint8_t buf_len, const bool ack_requested = true)
      GET_RAW_C78String(packetBuf, len, MIRF_t::PacketSize_Max);
      bool ackRequested = true;
#ifndef DEBUGprint_NRF
      bool ret = 
#endif
      offsetPacket.packet->sourceBool(ackRequested, packetData);
      DEBUGprint_NRF("nQ:r%d,b%d;", ret? 1:0, ackRequested? 1:0);
      mirf->queue_TX_packet(packetBuf, len, ackRequested);
    OPCODE(28) // queue_RX_packet(const uint8_t* buf, uint8_t buf_len, const Pipe_t rx_pipe = Pipe_RX_Default)
      GET_RAW_C78String(packetBuf, len, MIRF_t::PacketSize_Max);
      uint32_t rx_pipe = 1; offsetPacket.packet->sourceC78(rx_pipe, packetData);
      mirf->queue_TX_packet(packetBuf, len, rx_pipe != 0);
    OPCODE(29) // transmit_packet()
      mirf->transmit_packet();
    OPCODE(30) // transmit_reused()
      mirf->transmit_reused();
    OPCODE(31) // uint8_t receive_packet(uint8_t *buf, uint8_t buf_len = PacketSize_Max)
      // Retrieve received packet.
      uint8_t packetBuf[MIRF_t::PacketSize_Max];
      uint8_t packetSize = mirf->receive_packet(packetBuf, MIRF_t::PacketSize_Max);
      // Reply with packet contents.
      replyC78String(packetBuf, packetSize);
    OPCODE(32) // receive_packet(ReceivedPacket &packet)   <-- includes pipe
      ReceivedPacket rPacket;
      mirf->receive_packet(rPacket);
      replyReceivedPacket(rPacket);
    OPCODE(33) // bool transmit_buffer_full()
      replyBoolean(mirf->transmit_buffer_full());
    OPCODE(34) // bool transmit_buffer_empty()
      replyBoolean(mirf->transmit_buffer_empty());
    OPCODE(35) // flush_transmit_buffer()
      mirf->flush_transmit_buffer();
    OPCODE(36) // bool receive_buffer_full()
      replyBoolean(mirf->receive_buffer_full());
    OPCODE(37) // bool receive_buffer_empty()
      replyBoolean(mirf->receive_buffer_empty());
    OPCODE(38) // uint8_t read_received_packet_pipe()
      replyC78(mirf->read_received_packet_pipe());
    OPCODE(39) // flush_receive_buffer()
      replyBoolean(mirf->receive_buffer_empty());

    OPCODE(40) // uint8_t read_status()
      replyC78(mirf->read_status());
    OPCODE(41) // uint8_t read_register(uint8_t reg)
      GET_RAW_UINT(reg);
      replyC78(mirf->read_register(reg));
    OPCODE(42) // dump_registers(uint8_t *buf)
      uint8_t buf[MIRF_t::DumpSize];
      mirf->dump_registers(buf);
      replyC78String(buf, MIRF_t::DumpSize);
    } break;
  }

  finishedWithPacket();
  return Status::Status__Good;
}

// End namespace: Nordic_nRF
}

