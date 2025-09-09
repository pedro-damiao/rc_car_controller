#ifndef __MW_TRANSCEIVER_RF_HPP
#define __MW_TRANSCEIVER_RF_HPP

#include "hw_spi.hpp"
#include "hw_gpio.hpp"
#include <cstdint>

#define NRF24L01P_PAYLOAD_LENGTH          8     // 1 - 32bytes
#define DUMMY                       0x00

/* nRF24L01+ typedefs */
typedef uint8_t count;
typedef uint8_t widths;
typedef uint8_t length;
typedef uint16_t delay;
typedef uint16_t channel;

// === Payload ===
inline constexpr uint8_t PAYLOAD_LENGTH = 32;

// === Commands ===
inline constexpr uint8_t CMD_R_REGISTER          = 0x00;
inline constexpr uint8_t CMD_W_REGISTER          = 0x20;
inline constexpr uint8_t CMD_R_RX_PAYLOAD        = 0x61;
inline constexpr uint8_t CMD_W_TX_PAYLOAD        = 0xA0;
inline constexpr uint8_t CMD_FLUSH_TX            = 0xE1;
inline constexpr uint8_t CMD_FLUSH_RX            = 0xE2;
inline constexpr uint8_t CMD_REUSE_TX_PL         = 0xE3;
inline constexpr uint8_t CMD_R_RX_PL_WID         = 0x60;
inline constexpr uint8_t CMD_W_ACK_PAYLOAD       = 0xA8;
inline constexpr uint8_t CMD_W_TX_PAYLOAD_NOACK  = 0xB0;
inline constexpr uint8_t CMD_NOP                 = 0xFF;

// === Register Addresses ===
inline constexpr uint8_t REG_CONFIG        = 0x00;
inline constexpr uint8_t REG_EN_AA         = 0x01;
inline constexpr uint8_t REG_EN_RXADDR     = 0x02;
inline constexpr uint8_t REG_SETUP_AW      = 0x03;
inline constexpr uint8_t REG_SETUP_RETR    = 0x04;
inline constexpr uint8_t REG_RF_CH         = 0x05;
inline constexpr uint8_t REG_RF_SETUP      = 0x06;
inline constexpr uint8_t REG_STATUS        = 0x07;
inline constexpr uint8_t REG_OBSERVE_TX    = 0x08;
inline constexpr uint8_t REG_RPD           = 0x09;
inline constexpr uint8_t REG_RX_ADDR_P0    = 0x0A;
inline constexpr uint8_t REG_RX_ADDR_P1    = 0x0B;
inline constexpr uint8_t REG_RX_ADDR_P2    = 0x0C;
inline constexpr uint8_t REG_RX_ADDR_P3    = 0x0D;
inline constexpr uint8_t REG_RX_ADDR_P4    = 0x0E;
inline constexpr uint8_t REG_RX_ADDR_P5    = 0x0F;
inline constexpr uint8_t REG_TX_ADDR       = 0x10;
inline constexpr uint8_t REG_RX_PW_P0      = 0x11;
inline constexpr uint8_t REG_RX_PW_P1      = 0x12;
inline constexpr uint8_t REG_RX_PW_P2      = 0x13;
inline constexpr uint8_t REG_RX_PW_P3      = 0x14;
inline constexpr uint8_t REG_RX_PW_P4      = 0x15;
inline constexpr uint8_t REG_RX_PW_P5      = 0x16;
inline constexpr uint8_t REG_FIFO_STATUS   = 0x17;
inline constexpr uint8_t REG_DYNPD         = 0x1C;
inline constexpr uint8_t REG_FEATURE       = 0x1D;

// === Enums ===
enum class air_data_rate : uint8_t {
    _250kbps = 2,
    _1Mbps   = 0,
    _2Mbps   = 1
};

enum class output_power : uint8_t {
    _18dBm = 0,
    _12dBm = 1,
    _6dBm  = 2,
    _0dBm  = 3
};

// === Transceiver Class ===
class TransceiverRF {
public:
    TransceiverRF(Spi& spi, Gpio& enable);
    void tx_irq();
    void rx_irq(uint8_t* rx_payload);  
    uint8_t read_register(uint8_t reg);

    enum class Mode {SB, RX, TX };
    Mode get_mode() const { return m_mode; }

private:
    Spi& m_spi;
    Gpio& m_enable;
    Mode m_mode = Mode::SB;
    
    void init(channel MHz, air_data_rate bps);

    void tx_transmit(uint8_t* tx_payload);

    void reset();

    void prx_mode();
    void ptx_mode();

    void power_up();
    void power_down();

    uint8_t get_status();
    uint8_t get_fifo_status();

    void rx_set_payload_widths(widths bytes);

    uint8_t read_rx_fifo(uint8_t* rx_payload);
    uint8_t write_tx_fifo(uint8_t* tx_payload);

    void flush_rx_fifo();
    void flush_tx_fifo();

    void clear_rx_dr();
    void clear_tx_ds();
    void clear_max_rt();

    void set_rf_channel(channel MHz);
    void set_rf_tx_output_power(output_power dBm);
    void set_rf_air_data_rate(air_data_rate bps);

    void set_crc_length(length bytes);
    void set_address_widths(widths bytes);
    void auto_retransmit_count(count cnt);
    void auto_retransmit_delay(delay us);

    uint8_t write_register(uint8_t reg, uint8_t value);
};

#endif /* __MW_TRANSCEIVER_RF_HPP */