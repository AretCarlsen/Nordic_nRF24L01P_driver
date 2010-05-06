
// Underlying data from Stefan Engelke <mbox@stefanengelke.de>, 2007
//namespace Nordic_nRF24L01P {
// Registers
  typedef uint8_t Register_t;
  static const Register_t Register_CONFIG      = 0x00;
  static const Register_t Register_EN_AA       = 0x01;
  static const Register_t Register_EN_RXADDR   = 0x02;
  static const Register_t Register_SETUP_AW    = 0x03;
  static const Register_t Register_SETUP_RETR  = 0x04;
  static const Register_t Register_RF_CH       = 0x05;
  static const Register_t Register_RF_SETUP    = 0x06;
  static const Register_t Register_STATUS      = 0x07;
  static const Register_t Register_OBSERVE_TX  = 0x08;
  static const Register_t Register_CD          = 0x09;
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

// Bitflags
  typedef uint8_t BitPos_t;
  static const BitPos_t BitPos_MASK_RX_DR  = 6;
  static const BitPos_t BitPos_MASK_TX_DS  = 5;
  static const BitPos_t BitPos_MASK_MAX_RT = 4;
  static const BitPos_t BitPos_EN_CRC      = 3;
  static const BitPos_t BitPos_CRCO        = 2;
  static const BitPos_t BitPos_PWR_UP      = 1;
  static const BitPos_t BitPos_PRIM_RX     = 0;
  static const BitPos_t BitPos_ENAA_P5     = 5;
  static const BitPos_t BitPos_ENAA_P4     = 4;
  static const BitPos_t BitPos_ENAA_P3     = 3;
  static const BitPos_t BitPos_ENAA_P2     = 2;
  static const BitPos_t BitPos_ENAA_P1     = 1;
  static const BitPos_t BitPos_ENAA_P0     = 0;
  static const BitPos_t BitPos_ERX_P5      = 5;
  static const BitPos_t BitPos_ERX_P4      = 4;
  static const BitPos_t BitPos_ERX_P3      = 3;
  static const BitPos_t BitPos_ERX_P2      = 2;
  static const BitPos_t BitPos_ERX_P1      = 1;
  static const BitPos_t BitPos_ERX_P0      = 0;
  static const BitPos_t BitPos_AW          = 0;
  static const BitPos_t BitPos_ARD         = 4;
  static const BitPos_t BitPos_ARC         = 0;
  static const BitPos_t BitPos_PLL_LOCK    = 4;
  static const BitPos_t BitPos_RF_DR       = 3;
  static const BitPos_t BitPos_RF_PWR      = 1;
  static const BitPos_t BitPos_LNA_HCURR   = 0;
  static const BitPos_t BitPos_RX_DR       = 6;
  static const BitPos_t BitPos_TX_DS       = 5;
  static const BitPos_t BitPos_MAX_RT      = 4;
  static const BitPos_t BitPos_RX_P_NO     = 1;
  // The TX_FULL flag is in different positions in the STATUS and FIFO_STATUS registers.
  static const BitPos_t BitPos_STATUS_TX_FULL = 0;
  static const BitPos_t BitPos_FIFO_TX_FULL = 5;
  static const BitPos_t BitPos_PLOS_CNT    = 4;
  static const BitPos_t BitPos_ARC_CNT     = 0;
  static const BitPos_t BitPos_TX_REUSE    = 6;
  static const BitPos_t BitPos_TX_EMPTY    = 4;
  static const BitPos_t BitPos_RX_FULL     = 1;
  static const BitPos_t BitPos_RX_EMPTY    = 0;

  typedef uint8_t Bit_t;
  static const Bit_t Bit_MASK_RX_DR  = _BV(BitPos_MASK_RX_DR);
  static const Bit_t Bit_MASK_TX_DS  = _BV(BitPos_MASK_TX_DS);
  static const Bit_t Bit_MASK_MAX_RT = _BV(BitPos_MASK_MAX_RT);
  static const Bit_t Bit_EN_CRC      = _BV(BitPos_EN_CRC);
  static const Bit_t Bit_CRCO        = _BV(BitPos_CRCO);
  static const Bit_t Bit_PWR_UP      = _BV(BitPos_PWR_UP);
  static const Bit_t Bit_PRIM_RX     = _BV(BitPos_PRIM_RX);
  static const Bit_t Bit_ENAA_P5     = _BV(BitPos_ENAA_P5);
  static const Bit_t Bit_ENAA_P4     = _BV(BitPos_ENAA_P4);
  static const Bit_t Bit_ENAA_P3     = _BV(BitPos_ENAA_P3);
  static const Bit_t Bit_ENAA_P2     = _BV(BitPos_ENAA_P2);
  static const Bit_t Bit_ENAA_P1     = _BV(BitPos_ENAA_P1);
  static const Bit_t Bit_ENAA_P0     = _BV(BitPos_ENAA_P0);
  static const Bit_t Bit_ERX_P5      = _BV(BitPos_ERX_P5);
  static const Bit_t Bit_ERX_P4      = _BV(BitPos_ERX_P4);
  static const Bit_t Bit_ERX_P3      = _BV(BitPos_ERX_P3);
  static const Bit_t Bit_ERX_P2      = _BV(BitPos_ERX_P2);
  static const Bit_t Bit_ERX_P1      = _BV(BitPos_ERX_P1);
  static const Bit_t Bit_ERX_P0      = _BV(BitPos_ERX_P0);
  static const Bit_t Bit_AW          = _BV(BitPos_AW);
  static const Bit_t Bit_ARD         = _BV(BitPos_ARD);
  static const Bit_t Bit_ARC         = _BV(BitPos_ARC);
  static const Bit_t Bit_PLL_LOCK    = _BV(BitPos_PLL_LOCK);
  static const Bit_t Bit_RF_DR       = _BV(BitPos_RF_DR);
  static const Bit_t Bit_RF_PWR      = _BV(BitPos_RF_PWR);
  static const Bit_t Bit_LNA_HCURR   = _BV(BitPos_LNA_HCURR);
  static const Bit_t Bit_RX_DR       = _BV(BitPos_RX_DR);
  static const Bit_t Bit_TX_DS       = _BV(BitPos_TX_DS);
  static const Bit_t Bit_MAX_RT      = _BV(BitPos_MAX_RT);
  static const Bit_t Bit_RX_P_NO     = _BV(BitPos_RX_P_NO);
  // The TX_FULL flag is in different positions in the STATUS and FIFO_STATUS registers.
  static const Bit_t Bit_STATUS_TX_FULL = _BV(BitPos_STATUS_TX_FULL);
  static const Bit_t Bit_FIFO_TX_FULL = _BV(BitPos_FIFO_TX_FULL);
  static const Bit_t Bit_PLOS_CNT    = _BV(BitPos_PLOS_CNT);
  static const Bit_t Bit_ARC_CNT     = _BV(BitPos_ARC_CNT);
  static const Bit_t Bit_TX_REUSE    = _BV(BitPos_TX_REUSE);
  static const Bit_t Bit_TX_EMPTY    = _BV(BitPos_TX_EMPTY);
  static const Bit_t Bit_RX_FULL     = _BV(BitPos_RX_FULL);
  static const Bit_t Bit_RX_EMPTY    = _BV(BitPos_RX_EMPTY);


// Opcodes
  typedef uint8_t Opcode_t;
  static const Opcode_t Opcode_READ_REGISTER    = 0x00;
  static const Opcode_t Opcode_WRITE_REGISTER    = 0x20;
  static const Opcode_t Opcode_REGISTER_MASK = 0x1F;
  static const Opcode_t Opcode_R_RX_PAYLOAD  = 0x61;
  static const Opcode_t Opcode_W_TX_PAYLOAD  = 0xA0;
  static const Opcode_t Opcode_FLUSH_TX      = 0xE1;
  static const Opcode_t Opcode_FLUSH_RX      = 0xE2;
  static const Opcode_t Opcode_REUSE_TX_PL   = 0xE3;
  static const Opcode_t Opcode_NOP           = 0xFF;
//}

