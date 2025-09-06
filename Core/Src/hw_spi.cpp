#include "hw_spi.hpp"

Spi::Spi(SPI_TypeDef* spi, Gpio csPin)
    : m_spi(spi), m_csPin(csPin) {}

void Spi::deselect(){
    m_csPin.set();
    LL_mDelay(1);
}

void Spi::select(){
    m_csPin.reset();
    LL_mDelay(1);
}

void Spi::write(uint8_t *buffer, uint8_t length)
{
    for(uint8_t i=0; i < length; i++)
    { 
        TransmitAndReceive(buffer[i]);
    }
}

// TODO:  ?? data for write should be an array and add timer.
void Spi::writeRead(uint8_t data, uint8_t *buffer, uint8_t length)
{
    // read multiple bytes
    for(uint8_t i=0; i < length; i++)
    {
        buffer[i] = TransmitAndReceive(data);
    }
}

uint8_t Spi::TransmitAndReceive(uint8_t data) {
    // transmit
    LL_SPI_TransmitData8(m_spi, data);
    while(!LL_SPI_IsActiveFlag_TXE(m_spi));

    // receive
    while(!LL_SPI_IsActiveFlag_RXNE(m_spi));
    return LL_SPI_ReceiveData8(m_spi);
}

void Spi::enable() {
    LL_SPI_Enable(m_spi);
    while(!LL_SPI_IsEnabled(m_spi));
}