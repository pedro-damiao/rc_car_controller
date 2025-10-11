#include "mw_joystick.hpp"

Mw_Joystick::Mw_Joystick(HW_ADC& adc)
    : m_adc(adc) {}

void Mw_Joystick::update() {
    m_adc.readValues();
    m_x = m_adc.getValue(m_xChannel);
    m_y = m_adc.getValue(m_yChannel);
}

uint16_t Mw_Joystick::getX() const {
    return m_x;
}

uint16_t Mw_Joystick::getY() const {
    return m_y;
}