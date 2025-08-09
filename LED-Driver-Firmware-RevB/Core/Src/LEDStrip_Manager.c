#include "LEDStrip_Manager.h"

typedef struct {
    SK6812_HandleTypeDef strip;
    SK6812_DATA_RGB led_data[SK6812_NUM_LEDS];
    uint8_t dma_buf[SK6812_DMA_BUF_LEN(SK6812_NUM_LEDS)];
} LEDStrip;

static LEDStrip strips[LEDSTRIP_COUNT];

HAL_StatusTypeDef LEDStrip_Manager_Init() {

    SK6812_Init(&strips[LEDSTRIP_1].strip, &htim3, TIM_CHANNEL_4, strips[LEDSTRIP_1].dma_buf, SK6812_DMA_BUF_LEN(SK6812_NUM_LEDS), strips[LEDSTRIP_1].led_data, SK6812_NUM_LEDS);
    SK6812_Init(&strips[LEDSTRIP_2].strip, &htim3, TIM_CHANNEL_3, strips[LEDSTRIP_2].dma_buf, SK6812_DMA_BUF_LEN(SK6812_NUM_LEDS), strips[LEDSTRIP_2].led_data, SK6812_NUM_LEDS);
    SK6812_Init(&strips[LEDSTRIP_3].strip, &htim8, TIM_CHANNEL_1, strips[LEDSTRIP_3].dma_buf, SK6812_DMA_BUF_LEN(SK6812_NUM_LEDS), strips[LEDSTRIP_3].led_data, SK6812_NUM_LEDS);
    SK6812_Init(&strips[LEDSTRIP_4].strip, &htim3, TIM_CHANNEL_1, strips[LEDSTRIP_4].dma_buf, SK6812_DMA_BUF_LEN(SK6812_NUM_LEDS), strips[LEDSTRIP_4].led_data, SK6812_NUM_LEDS);
    SK6812_Init(&strips[LEDSTRIP_5].strip, &htim5, TIM_CHANNEL_4, strips[LEDSTRIP_5].dma_buf, SK6812_DMA_BUF_LEN(SK6812_NUM_LEDS), strips[LEDSTRIP_5].led_data, SK6812_NUM_LEDS);
    SK6812_Init(&strips[LEDSTRIP_6].strip, &htim5, TIM_CHANNEL_3, strips[LEDSTRIP_6].dma_buf, SK6812_DMA_BUF_LEN(SK6812_NUM_LEDS), strips[LEDSTRIP_6].led_data, SK6812_NUM_LEDS);
    SK6812_Init(&strips[LEDSTRIP_7].strip, &htim5, TIM_CHANNEL_2, strips[LEDSTRIP_7].dma_buf, SK6812_DMA_BUF_LEN(SK6812_NUM_LEDS), strips[LEDSTRIP_7].led_data, SK6812_NUM_LEDS);
    SK6812_Init(&strips[LEDSTRIP_8].strip, &htim5, TIM_CHANNEL_1, strips[LEDSTRIP_8].dma_buf, SK6812_DMA_BUF_LEN(SK6812_NUM_LEDS), strips[LEDSTRIP_8].led_data, SK6812_NUM_LEDS);

    return HAL_OK;

}

SK6812_HandleTypeDef* LEDStrip_Manager_Get_Strip_Handle(LEDStrip_id id) {

    return &strips[id].strip;

}

