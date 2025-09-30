#include "hw_adc.hpp"

HW_ADC::HW_ADC(ADC_TypeDef* adcInstance, Mode mode, uint8_t numChannels){
    // Constructor implementation
    m_adc = adcInstance;
    m_mode = mode;
    m_numChannels = numChannels;
}

HW_ADC::~HW_ADC() {
    if (m_channels) {
        free(m_channels);
        m_channels = nullptr;
    }
}

void HW_ADC::init() {
    if (m_channels) {
        free(m_channels);
    }
    m_channels = static_cast<uint16_t*>(malloc(sizeof(uint16_t) * m_numChannels));
    LL_ADC_Enable(m_adc);
}

void HW_ADC::readValues() {
    if (m_mode != Mode::Scan || m_channels == nullptr)
        return;

    LL_ADC_REG_StartConversionSWStart(m_adc);

    for (uint8_t i = 0; i < m_numChannels; ++i) {
        while (!LL_ADC_IsActiveFlag_EOCS(m_adc));
        m_channels[i] = LL_ADC_REG_ReadConversionData12(m_adc);
        LL_ADC_ClearFlag_EOCS(m_adc);
    }
}

uint16_t HW_ADC::getValue(uint8_t channelIdx) const {
    if (channelIdx >= m_numChannels || m_channels == nullptr)
        return 0;
    return m_channels[channelIdx];
}
