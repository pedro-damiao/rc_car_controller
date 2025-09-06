#ifndef __HW_GPIO_HPP
#define __HW_GPIO_HPP

#include "stm32f4xx_ll_gpio.h"

class Gpio {
public:
    Gpio(GPIO_TypeDef* port, uint16_t pin);

    void set();
    void reset();
    void toggle();
    uint32_t read();

private:
    GPIO_TypeDef* m_port;
    uint16_t m_pin;
};

#endif /* _HW_GPIO_HPP */
