#include "mw_joystick.hpp"

MW_Joystick::MW_Joystick(HW_ADC& adc, uint8_t xChannel, uint8_t yChannel)
    : m_adc(adc), m_xChannel(xChannel), m_yChannel(yChannel) {}

void MW_Joystick::update() {
    m_adc.readValues();
    m_x = m_adc.getValue(m_xChannel);
    m_y = m_adc.getValue(m_yChannel);
}

uint16_t MW_Joystick::getX() const {
    return m_x;
}

uint16_t MW_Joystick::getY() const {
    return m_y;
}