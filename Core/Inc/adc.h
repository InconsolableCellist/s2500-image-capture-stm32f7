#ifndef __ADC_H
#define __ADC_H

#include <stm32f7xx_hal_adc.h>
                                                    // fastest, most error, least bits of resolution
#define ADC_CUSTOM_SPEED_RAPID                 0U
#define ADC_CUSTOM_SPEED_HALF                  1U
#define ADC_CUSTOM_SPEED_HALF_SLOWER           2U
#define ADC_CUSTOM_SPEED_THREEQUARTERS         3U
#define ADC_CUSTOM_SPEED_THREEQUARTERS_SLOWER  4U
#define ADC_CUSTOM_SPEED_PHOTO                 5U
#define ADC_CUSTOM_SPEED_MAX_VALUE             5U
                                                    // slowest, least error, most bits of resolution

void ADC_SwitchSamplingMode(ADC_HandleTypeDef* hadc, uint8_t adc_custom_speed);

#endif
