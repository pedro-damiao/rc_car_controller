#ifndef __HW_ADC_HPP
#define __HW_ADC_HPP

#include "stm32f4xx_ll_adc.h"
#include <cstdint>
#include <cstdlib>

class HW_ADC {
public:
    enum class Mode {
        Single,
        Scan
    };

    HW_ADC(ADC_TypeDef* adcInstance, Mode mode, uint8_t numChannels);
    ~HW_ADC();

    void init();
    void readValues();
    uint16_t getValue(uint8_t channelIdx) const; 

private:
    ADC_TypeDef* m_adc;
    Mode m_mode;
    uint8_t m_numChannels;
    uint16_t* m_channels = nullptr;
};

#endif /* __HW_ADC_HPP */