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

/**
 * @brief HAL Timer PWM Pulse Finished Callback
 * @param htim Timer handle
 *
 * This callback is invoked by the HAL when a timer PWM DMA transfer completes.
 * It routes the completion event to the appropriate LED strip driver based on
 * which timer and channel triggered the callback.
 *
 * Timer/Channel to Strip Mapping:
 * - TIM3 CH1 → LEDSTRIP_4
 * - TIM3 CH3 → LEDSTRIP_2
 * - TIM3 CH4 → LEDSTRIP_1
 * - TIM5 CH1 → LEDSTRIP_8
 * - TIM5 CH2 → LEDSTRIP_7
 * - TIM5 CH3 → LEDSTRIP_6
 * - TIM5 CH4 → LEDSTRIP_5
 * - TIM8 CH1 → LEDSTRIP_3
 */
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim) {

	if (htim == &htim3) {
		switch (htim->Channel) {
			case HAL_TIM_ACTIVE_CHANNEL_1:
				SK6812_DMACompleteCallback(LEDStrip_Manager_Get_Strip_Handle(LEDSTRIP_4));
				break;
			case HAL_TIM_ACTIVE_CHANNEL_3:
				SK6812_DMACompleteCallback(LEDStrip_Manager_Get_Strip_Handle(LEDSTRIP_2));
				break;
			case HAL_TIM_ACTIVE_CHANNEL_4:
				SK6812_DMACompleteCallback(LEDStrip_Manager_Get_Strip_Handle(LEDSTRIP_1));
				break;
			default:
				break;
		}
	}
	else if (htim == &htim5) {
		switch (htim->Channel) {
			case HAL_TIM_ACTIVE_CHANNEL_1:
				SK6812_DMACompleteCallback(LEDStrip_Manager_Get_Strip_Handle(LEDSTRIP_8));
				break;
			case HAL_TIM_ACTIVE_CHANNEL_2:
				SK6812_DMACompleteCallback(LEDStrip_Manager_Get_Strip_Handle(LEDSTRIP_7));
				break;
			case HAL_TIM_ACTIVE_CHANNEL_3:
				SK6812_DMACompleteCallback(LEDStrip_Manager_Get_Strip_Handle(LEDSTRIP_6));
				break;
			case HAL_TIM_ACTIVE_CHANNEL_4:
				SK6812_DMACompleteCallback(LEDStrip_Manager_Get_Strip_Handle(LEDSTRIP_5));
				break;
			default:
				break;
		}
	}
	else if (htim == &htim8) {
		switch (htim->Channel) {
			case HAL_TIM_ACTIVE_CHANNEL_1:
				SK6812_DMACompleteCallback(LEDStrip_Manager_Get_Strip_Handle(LEDSTRIP_3));
				break;
			default:
				break;
		}
	}

}
