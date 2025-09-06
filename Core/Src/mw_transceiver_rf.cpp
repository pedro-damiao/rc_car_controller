#include "mw_transceiver_rf.hpp"

TransceiverRF::TransceiverRF(Spi& spi, Gpio& enable)
    : m_spi(spi), m_enable(enable) {
	m_spi.enable();
    tx_init(2500, air_data_rate::_1Mbps);
}

// Reads a register from the via SPI.
uint8_t TransceiverRF::read_register(uint8_t reg)
{
    uint8_t command = CMD_R_REGISTER | reg;
    uint8_t status;
    uint8_t read_val;

    m_spi.select();
    m_spi.writeRead(command, &status, 1);
    m_spi.deselect();

    LL_mDelay(1);

    m_spi.select();
    m_spi.writeRead(DUMMY, &read_val, 1);
    m_spi.deselect();

    return read_val;
}

// Writes a value to a register via SPI.
uint8_t TransceiverRF::write_register(uint8_t reg, uint8_t value)
{
    uint8_t command = CMD_W_REGISTER | reg;
    uint8_t status;
    uint8_t write_val = value;
    m_spi.select();
    m_spi.writeRead(command, &status, 1);
    m_spi.deselect();

    LL_mDelay(1);

    m_spi.select();
    m_spi.write(&write_val, 1);
    m_spi.deselect();
    
    return write_val;
}

// Flushes the RX FIFO buffer
void TransceiverRF::flush_rx_fifo()
{
    uint8_t command = CMD_FLUSH_RX;
    uint8_t status;

    m_spi.select();
    m_spi.writeRead(command, &status, 1);
    m_spi.deselect();
}

// Flushes the TX FIFO buffer
void TransceiverRF::flush_tx_fifo()
{
    uint8_t command = CMD_FLUSH_TX;
    uint8_t status;

    m_spi.select();
    m_spi.writeRead(command, &status, 1);
    m_spi.deselect();
}

// Powers up by setting the PWR_UP bit in CONFIG
void TransceiverRF::power_up()
{
    uint8_t new_config = read_register(REG_CONFIG);
    new_config |= 1 << 1;

    write_register(REG_CONFIG, new_config);
}

// Resets to default configuration and clears FIFOs
void TransceiverRF::reset()
{
    // Reset pins
    m_spi.deselect();
    m_enable.reset();

    // Reset registers
    write_register(REG_CONFIG, 0x08);
    write_register(REG_EN_AA, 0x3F);
    write_register(REG_EN_RXADDR, 0x03);
    write_register(REG_SETUP_AW, 0x03);
    write_register(REG_SETUP_RETR, 0x03);
    write_register(REG_RF_CH, 0x02);
    write_register(REG_RF_SETUP, 0x07);
    write_register(REG_STATUS, 0x7E);
    write_register(REG_RX_PW_P0, 0x00);
    write_register(REG_RX_PW_P0, 0x00);
    write_register(REG_RX_PW_P1, 0x00);
    write_register(REG_RX_PW_P2, 0x00);
    write_register(REG_RX_PW_P3, 0x00);
    write_register(REG_RX_PW_P4, 0x00);
    write_register(REG_RX_PW_P5, 0x00);
    write_register(REG_FIFO_STATUS, 0x11);
    write_register(REG_DYNPD, 0x00);
    write_register(REG_FEATURE, 0x00);

    // Reset FIFO
    flush_rx_fifo();
    flush_tx_fifo();
}

// Sets the static payload width for RX pipe 0
void TransceiverRF::rx_set_payload_widths(widths bytes)
{
    write_register(REG_RX_PW_P0, bytes);
}

// Sets the RF channel (frequency) for the transceiver
void TransceiverRF::set_rf_channel(channel MHz)
{
	uint16_t new_rf_ch = MHz - 2400;
    write_register(REG_RF_CH, new_rf_ch);
}

// Sets the air data rate (bitrate) for the transceiver
void TransceiverRF::set_rf_air_data_rate(air_data_rate bps)
{
    // Set value to 0
    uint8_t new_rf_setup = read_register(REG_RF_SETUP) & 0xD7;
    
    switch(bps)
    {
        case air_data_rate::_1Mbps: 
            break;
        case air_data_rate::_2Mbps: 
            new_rf_setup |= 1 << 3;
            break;
        case air_data_rate::_250kbps:
            new_rf_setup |= 1 << 5;
            break;
    }
    write_register(REG_RF_SETUP, new_rf_setup);
}

// Sets the RF output power
void TransceiverRF::set_rf_tx_output_power(output_power dBm)
{
    uint8_t new_rf_setup = read_register(REG_RF_SETUP) & 0xF9;
    new_rf_setup |= (static_cast<uint8_t>(dBm) << 1);

    write_register(REG_RF_SETUP, new_rf_setup);
}

// Sets the CRC length (1 or 2 bytes)
void TransceiverRF::set_crc_length(length bytes)
{
    uint8_t new_config = read_register(REG_CONFIG);
    
    switch(bytes)
    {
        // CRCO bit in CONFIG resiger set 0
        case 1:
            new_config &= 0xFB;
            break;
        // CRCO bit in CONFIG resiger set 1
        case 2:
            new_config |= 1 << 2;
            break;
    }

    write_register(REG_CONFIG, new_config);
}

// Sets the address width for the transceiver
void TransceiverRF::set_address_widths(widths bytes)
{
    write_register(REG_SETUP_AW, bytes - 2);
}

// Sets the auto retransmit count (number of retries)
void TransceiverRF::auto_retransmit_count(count cnt)
{
    uint8_t new_setup_retr = read_register(REG_SETUP_RETR);
    
    // Reset ARC register 0
    new_setup_retr |= 0xF0;
    new_setup_retr |= cnt;
    write_register(REG_SETUP_RETR, new_setup_retr);
}

// Sets the auto retransmit delay (in microseconds)
void TransceiverRF::auto_retransmit_delay(delay us)
{
    uint8_t new_setup_retr = read_register(REG_SETUP_RETR);

    // Reset ARD register 0
    new_setup_retr |= 0x0F;
    new_setup_retr |= ((us / 250) - 1) << 4;
    write_register(REG_SETUP_RETR, new_setup_retr);
}

// Sets the device to PRX (primary RX) mode
void TransceiverRF::prx_mode()
{
    uint8_t new_config = read_register(REG_CONFIG);
    new_config |= 1 << 0;

    write_register(REG_CONFIG, new_config);
}

// Initializes the transceiver in RX mode with given channel and data rate
void TransceiverRF::rx_init(channel MHz, air_data_rate bps)
{
    reset();

    prx_mode();
    power_up();

    rx_set_payload_widths(PAYLOAD_LENGTH);

    set_rf_channel(MHz);
    set_rf_air_data_rate(bps);
    set_rf_tx_output_power(output_power::_0dBm);

    set_crc_length(1);
    set_address_widths(5);

    auto_retransmit_count(3);
    auto_retransmit_delay(250);
    
    m_enable.set();
}

// Clears the RX_DR (data ready) interrupt flag
void TransceiverRF::clear_rx_dr()
{
    uint8_t new_status = get_status();
    new_status |= 0x40;

    write_register(REG_STATUS, new_status);
}

// Reads a payload from the RX FIFO and clears the RX_DR flag
void TransceiverRF::rx_receive(uint8_t* rx_payload)
{
    read_rx_fifo(rx_payload);
    clear_rx_dr();

    // LED TOGGLE
}

// Sets the device to PTX (primary TX) mode
void TransceiverRF::ptx_mode()
{
    uint8_t new_config = read_register(REG_CONFIG);
    new_config &= 0xFE;

    write_register(REG_CONFIG, new_config);
}

// Initializes the transceiver in TX mode with given channel and data rate
void TransceiverRF::tx_init(channel MHz, air_data_rate bps)
{
    reset();

    ptx_mode();
    power_up();

    set_rf_channel(MHz);
    set_rf_air_data_rate(bps);
    set_rf_tx_output_power(output_power::_0dBm);

    set_crc_length(1);
    set_address_widths(5);

    auto_retransmit_count(3);
    auto_retransmit_delay(250);

    m_enable.set();
}

// Writes a payload to the TX FIFO and returns the status
uint8_t TransceiverRF::write_tx_fifo(uint8_t* tx_payload)
{
    uint8_t command = CMD_W_TX_PAYLOAD;
    uint8_t status;

    m_spi.select();
    m_spi.writeRead(command, &status, 1);
    m_spi.write(tx_payload, PAYLOAD_LENGTH);
    m_spi.deselect();

    return status;
}

// Transmits a payload by writing it to the TX FIFO
void TransceiverRF::tx_transmit(uint8_t* tx_payload)
{
    write_tx_fifo(tx_payload);
}

// Reads the STATUS register (returns the current status flags)
uint8_t TransceiverRF::get_status()
{
    uint8_t command = CMD_NOP;
    uint8_t status;

    m_spi.select();
    m_spi.writeRead(command, &status, 1);
    m_spi.deselect();

    return status;
}

// Clears the MAX_RT (maximum retransmit) interrupt flag
void TransceiverRF::clear_max_rt()
{
    uint8_t new_status = get_status();
    new_status |= 0x10;

    write_register(REG_STATUS, new_status); 
}

// Handles TX IRQ: checks if TX_DS or MAX_RT occurred and clears the appropriate flag
void TransceiverRF::tx_irq()
{
    uint8_t tx_ds = get_status();
    tx_ds &= 0x20;

    if(tx_ds)
    {   
        // TX_DS
        // LED TOGGLE
        clear_tx_ds();
    }

    else
    {
        // MAX_RT
        // LED SET
        clear_max_rt();
    }
}

// Reads a payload from the RX FIFO and returns the status
uint8_t TransceiverRF::read_rx_fifo(uint8_t* rx_payload)
{
    uint8_t command = CMD_R_RX_PAYLOAD;
    uint8_t status;

    m_spi.select();
    m_spi.writeRead(command, &status, 1);
    m_spi.writeRead(DUMMY, rx_payload, PAYLOAD_LENGTH);
    m_spi.deselect();

    return status;
}

// Reads the FIFO_STATUS register
uint8_t TransceiverRF::get_fifo_status()
{
    return read_register(REG_FIFO_STATUS);
}

// Clears the TX_DS (data sent) interrupt flag
void TransceiverRF::clear_tx_ds()
{
    uint8_t new_status = get_status();
    new_status |= 0x20;

    write_register(REG_STATUS, new_status);     
}

// Powers down the nRF24L01+ by clearing the PWR_UP bit in CONFIG
void TransceiverRF::power_down()
{
    uint8_t new_config = read_register(REG_CONFIG);
    new_config &= 0xFD;

    write_register(REG_CONFIG, new_config);
}