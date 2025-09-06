#ifndef __HW_SPI_HPP
#define __HW_SPI_HPP

#include "stm32f4xx_ll_spi.h"
#include "stm32f4xx_ll_gpio.h"
#include "stm32f4xx_ll_utils.h"
#include "hw_gpio.hpp"

class Spi {
public:
    Spi(SPI_TypeDef* spi, Gpio csPin);

    void write(uint8_t *buffer, uint8_t length);
    void writeRead(uint8_t data, uint8_t *buffer, uint8_t length);
    void select();
    void deselect();
    void enable();

private:
    uint8_t TransmitAndReceive(uint8_t data);

    SPI_TypeDef* m_spi;
    Gpio m_csPin;
};

#endif /* __HW_SPI_HPP */
