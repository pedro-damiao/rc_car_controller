#include "hw_gpio.hpp"

Gpio::Gpio(GPIO_TypeDef* port, uint16_t pin)
    : m_port(port), m_pin(pin) {}

void Gpio::set() {
    LL_GPIO_SetOutputPin(m_port, m_pin);
}

void Gpio::reset() {
    LL_GPIO_ResetOutputPin(m_port, m_pin);
}

void Gpio::toggle() {
    LL_GPIO_TogglePin(m_port, m_pin);
}

uint32_t Gpio::read() {
    return (LL_GPIO_ReadOutputPort(m_port) & m_pin);
}
