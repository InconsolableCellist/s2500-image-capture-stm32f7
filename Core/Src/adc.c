#include <main.h>
#include <adc.h>

static void modeRapid(ADC_HandleTypeDef* hadc);
static void modeHalf(ADC_HandleTypeDef* hadc);
static void modeHalfSlower(ADC_HandleTypeDef* hadc);
static void modeThreeQuarters(ADC_HandleTypeDef* hadc);
static void modeThreeQuartersSlower(ADC_HandleTypeDef* hadc);
static void modePhoto(ADC_HandleTypeDef* hadc);
static void modePhotoSlowest(ADC_HandleTypeDef* hadc);
static void switchMode(ADC_HandleTypeDef* hadc, ADC_ChannelConfTypeDef* sConfig);

//  6 bits   9 cycles
//  8 bites 11 cycles
// 10 bits  13 cycles
// 12 bits  15 cycles

// 3, 15, 28, 56, 84, 112, 144, 480

void ADC_SwitchSamplingMode(ADC_HandleTypeDef* hadc, uint8_t adc_custom_speed) {
    switch (adc_custom_speed) {
        case ADC_CUSTOM_SPEED_RAPID:
            modeRapid(hadc); break;
        case ADC_CUSTOM_SPEED_HALF:
            modeHalf(hadc); break;
        case ADC_CUSTOM_SPEED_HALF_SLOWER:
            modeHalfSlower(hadc); break;
        case ADC_CUSTOM_SPEED_THREEQUARTERS:
            modeThreeQuarters(hadc); break;
        case ADC_CUSTOM_SPEED_THREEQUARTERS_SLOWER:
            modeThreeQuartersSlower(hadc); break;
        case ADC_CUSTOM_SPEED_PHOTO:
            modePhoto(hadc); break;
        case ADC_CUSTOM_SPEED_PHOTO_SLOWEST:
            modePhotoSlowest(hadc); break;
        default:
            break;
    }
}

// 135 horizontal pixels
static void modeRapid(ADC_HandleTypeDef* hadc) {
    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
    hadc->Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
    hadc->Init.Resolution = ADC_RESOLUTION_6B;
    switchMode(hadc, &sConfig);
}

// 1,308 horizontal pixels
static void modeHalf(ADC_HandleTypeDef* hadc) {
    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES;
    hadc->Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
    hadc->Init.Resolution = ADC_RESOLUTION_12B;
    switchMode(hadc, &sConfig);
}

static void modeHalfSlower(ADC_HandleTypeDef* hadc) {
    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
    hadc->Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
    hadc->Init.Resolution = ADC_RESOLUTION_12B;
    switchMode(hadc, &sConfig);
}

// 1,994 horizontal pixels
static void modeThreeQuarters(ADC_HandleTypeDef* hadc) {
    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
    hadc->Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
    hadc->Init.Resolution = ADC_RESOLUTION_12B;
    switchMode(hadc, &sConfig);
}

static void modeThreeQuartersSlower(ADC_HandleTypeDef* hadc) {
    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
    hadc->Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV8;
    hadc->Init.Resolution = ADC_RESOLUTION_12B;
    switchMode(hadc, &sConfig);
}

static void modePhoto(ADC_HandleTypeDef* hadc) {
    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.SamplingTime = ADC_SAMPLETIME_144CYCLES;
    hadc->Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV8;
    hadc->Init.Resolution = ADC_RESOLUTION_12B;
    switchMode(hadc, &sConfig);
}

static void modePhotoSlowest(ADC_HandleTypeDef* hadc) {
    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
    hadc->Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV8;
    hadc->Init.Resolution = ADC_RESOLUTION_12B;
    switchMode(hadc, &sConfig);
}

static void switchMode(ADC_HandleTypeDef* hadc, ADC_ChannelConfTypeDef* sConfig) {
    hadc->Instance = ADC1;
    hadc->Init.ScanConvMode = ADC_SCAN_DISABLE;
    hadc->Init.ContinuousConvMode = ENABLE;
    hadc->Init.DiscontinuousConvMode = DISABLE;
    hadc->Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc->Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc->Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc->Init.NbrOfConversion = 1;
    hadc->Init.DMAContinuousRequests = DISABLE;
    hadc->Init.EOCSelection = ADC_EOC_SEQ_CONV;
    if (HAL_ADC_Init(hadc) != HAL_OK) {
        DEBUG_HIGH DEBUG_LOW
        DEBUG_HIGH DEBUG_LOW
        DEBUG_HIGH DEBUG_LOW
    }

    sConfig->Channel = ADC_CHANNEL_6;
    sConfig->Rank = ADC_REGULAR_RANK_1;
    if (HAL_ADC_ConfigChannel(hadc, sConfig) != HAL_OK) {
        DEBUG_HIGH DEBUG_LOW
        DEBUG_HIGH DEBUG_LOW
        DEBUG_HIGH DEBUG_LOW
        DEBUG_HIGH DEBUG_LOW
    }
}

