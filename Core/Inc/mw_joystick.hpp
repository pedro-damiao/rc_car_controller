#ifndef __MW_JOYSTICK_HPP
#define __MW_JOYSTICK_HPP

#include "hw_adc.hpp"

class MW_Joystick {
public:
    MW_Joystick(HW_ADC& adc, uint8_t xChannel, uint8_t yChannel);

    void update();
    uint16_t getX() const;
    uint16_t getY() const;

private:
    HW_ADC& m_adc;
    uint8_t m_xChannel;
    uint8_t m_yChannel;
    uint16_t m_x = 0;
    uint16_t m_y = 0;
};

#endif // __MW_JOYSTICK_HPP