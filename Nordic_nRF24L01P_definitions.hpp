// Copyright (C) 2010, Aret N Carlsen (aretcarlsen@autonomoustools.com).
// Nordic nRF24L01P comprehensive driver (C++).
// Licensed under GPLv3 and later versions. See license.txt or <http://www.gnu.org/licenses/>.


//namespace Nordic_nRF24L01P {

  static const uint8_t AddressSize_Min = 3;
  static const uint8_t AddressSize_Max = 5;

  static const uint8_t ACK_Rx_Pipe = 0;
  static const uint8_t Pipe_Max = 5;

  static const PacketSize_t PacketSize_Max = 32;

  static const uint8_t CRC_Max = 2;

  // Auto retransmit delay. 250us units.
  static const ARD_t AutoRetransmitDelay_Max = 15;
  // Auto retransmit count. 0 is effectively disabled.
  static const ARD_t AutoRetransmitCount_Max = 15;

  // Radio channels
  static const Channel_t Channel_Max = 127;
  // Radio frequency, MHz
  static const Frequency_t Frequency_Min = 2400;
  static const Frequency_t Frequency_Max = 2527;  // ?

  // Radio gain, dBm
  static const Gain_t Gain_Min = -18;
  static const Gain_t Gain_Max = 0;

  // Register dump size (total of readable registers)
  // 0x17 + 3 (for 0x00, 0x1C, 0x1D) + 3*4 (for the three 5-byte address registers)
  static const uint8_t DumpSize = 38;

// Much underlying data from Stefan Engelke <mbox@stefanengelke.de>, 2007.
// Registers
  static const Register_t Register_Max         = 0x1D;

  static const Register_t Register_CONFIG      = 0x00;
  static const Register_t Register_EN_AA       = 0x01;
  static const Register_t Register_EN_RXADDR   = 0x02;
  static const Register_t Register_SETUP_AW    = 0x03;
  static const Register_t Register_SETUP_RETR  = 0x04;
  static const Register_t Register_RF_CH       = 0x05;
  static const Register_t Register_RF_SETUP    = 0x06;
  static const Register_t Register_STATUS      = 0x07;
  static const Register_t Register_OBSERVE_TX  = 0x08;
  static const Register_t Register_RPD         = 0x09;
  static const Register_t Register_RX_ADDR_P0  = 0x0A;
  static const Register_t Register_RX_ADDR_P1  = 0x0B;
  static const Register_t Register_RX_ADDR_P2  = 0x0C;
  static const Register_t Register_RX_ADDR_P3  = 0x0D;
  static const Register_t Register_RX_ADDR_P4  = 0x0E;
  static const Register_t Register_RX_ADDR_P5  = 0x0F;
  static const Register_t Register_TX_ADDR     = 0x10;
  static const Register_t Register_RX_PW_P0    = 0x11;
  static const Register_t Register_RX_PW_P1    = 0x12;
  static const Register_t Register_RX_PW_P2    = 0x13;
  static const Register_t Register_RX_PW_P3    = 0x14;
  static const Register_t Register_RX_PW_P4    = 0x15;
  static const Register_t Register_RX_PW_P5    = 0x16;
  static const Register_t Register_FIFO_STATUS = 0x17;
  static const Register_t Register_DYNPD       = 0x1C;
  static const Register_t Register_FEATURE     = 0x1D;

// Bitflags
  static const Bit_t Bit_MASK_RX_DR  = 6;
  static const Bit_t Bit_MASK_TX_DS  = 5;
  static const Bit_t Bit_MASK_MAX_RT = 4;
  static const Bit_t Bit_EN_CRC      = 3;
  static const Bit_t Bit_CRCO        = 2;
  static const Bit_t Bit_PWR_UP      = 1;
  static const Bit_t Bit_PRIM_RX     = 0;
  static const Bit_t Bit_ENAA_P5     = 5;
  static const Bit_t Bit_ENAA_P4     = 4;
  static const Bit_t Bit_ENAA_P3     = 3;
  static const Bit_t Bit_ENAA_P2     = 2;
  static const Bit_t Bit_ENAA_P1     = 1;
  static const Bit_t Bit_ENAA_P0     = 0;
  static const Bit_t Bit_ERX_P5      = 5;
  static const Bit_t Bit_ERX_P4      = 4;
  static const Bit_t Bit_ERX_P3      = 3;
  static const Bit_t Bit_ERX_P2      = 2;
  static const Bit_t Bit_ERX_P1      = 1;
  static const Bit_t Bit_ERX_P0      = 0;
  static const Bit_t Bit_AW          = 0;
  static const Bit_t Bit_ARD         = 4;
  static const Bit_t Bit_ARC         = 0;
  static const Bit_t Bit_PLL_LOCK    = 4;
  static const Bit_t Bit_RF_DR_HIGH  = 3;
  static const Bit_t Bit_RF_DR_LOW   = 5;
  static const Bit_t Bit_RF_PWR      = 1;
  static const Bit_t Bit_LNA_HCURR   = 0;
  static const Bit_t Bit_RX_DR       = 6;
  static const Bit_t Bit_TX_DS       = 5;
  static const Bit_t Bit_MAX_RT      = 4;
  static const Bit_t Bit_RX_P_NO     = 1;
  // The TX_FULL flag is in different positions in the STATUS and FIFO_STATUS registers.
  static const Bit_t Bit_STATUS_TX_FULL = 0;
  static const Bit_t Bit_FIFO_TX_FULL = 5;
  static const Bit_t Bit_PLOS_CNT    = 4;
  static const Bit_t Bit_ARC_CNT     = 0;
  static const Bit_t Bit_RPD         = 0;
  static const Bit_t Bit_TX_REUSE    = 6;
  static const Bit_t Bit_TX_EMPTY    = 4;
  static const Bit_t Bit_RX_FULL     = 1;
  static const Bit_t Bit_RX_EMPTY    = 0;
  static const Bit_t Bit_DPL_P5      = 5;
  static const Bit_t Bit_DPL_P4      = 4;
  static const Bit_t Bit_DPL_P3      = 3;
  static const Bit_t Bit_DPL_P2      = 2;
  static const Bit_t Bit_DPL_P1      = 1;
  static const Bit_t Bit_DPL_P0      = 0;
  static const Bit_t Bit_EN_DPL      = 2;
  static const Bit_t Bit_EN_ACK_PAY  = 1;
  static const Bit_t Bit_EN_DYN_ACK  = 0;
  

  static const BitMask_t BitMask_RX_P_NO       = 0b1110;
  static const BitMask_t BitMask_RX_P_NO_Empty = 0b1110;
  static const BitMask_t BitMask_ARD           = 0xF0;
  static const BitMask_t BitMask_ARC           = 0x0F;
  static const BitMask_t BitMask_PLOS_CNT      = 0xF0;
  static const BitMask_t BitMask_ARC_CNT       = 0x0F;

// Opcodes
  static const Opcode_t Opcode_READ_REGISTER  = 0x00;
  static const Opcode_t Opcode_WRITE_REGISTER = 0x20;
  static const Opcode_t Opcode_REGISTER_MASK  = 0x1F;
  static const Opcode_t Opcode_R_RX_PAYLOAD   = 0x61;
  static const Opcode_t Opcode_W_TX_PAYLOAD   = 0xA0;
  static const Opcode_t Opcode_W_TX_PAYLOAD_NOACK = 0xB0;
  static const Opcode_t Opcode_W_ACK_PAYLOAD  = 0xA8;
  static const Opcode_t Opcode_FLUSH_TX       = 0xE1;
  static const Opcode_t Opcode_FLUSH_RX       = 0xE2;
  static const Opcode_t Opcode_REUSE_TX_PL    = 0xE3;
  static const Opcode_t Opcode_R_RX_PL_WID    = 0x60;
  static const Opcode_t Opcode_NOP            = 0xFF;
//}

