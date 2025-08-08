/*
 * SK6812-LEDStrip.h
 *
 *  Created on: Apr 29, 2025
 *      Author: nick
 */

#include "stm32f1xx_hal.h"

#ifndef INC_SK6812_LEDSTRIP_H_
#define INC_SK6812_LEDSTRIP_H_

#define SK6812_NUM_LEDS 	120
#define SK6812_RST_PERIODS 	64 // 80us/(1.25us per period)
#define SK6812_BITS_PER_LED 24
#define SK6812_DMA_BUF_LEN(num_leds)  (((num_leds) * SK6812_BITS_PER_LED) + SK6812_RST_PERIODS)

typedef union {

	struct { // Bit order: G7-G0,R7-R0,B7-B0
		uint8_t green;
		uint8_t red;
		uint8_t blue;
	} colour;

	uint32_t data;

} SK6812_DATA_RGB;

typedef struct {
	TIM_HandleTypeDef* timer;
	uint32_t channel;
	uint8_t* dma_buf;
	uint16_t dma_buf_len;
	SK6812_DATA_RGB* led_data;
	uint16_t num_leds;
	volatile uint8_t dma_done_flag;
} SK6812_HandleTypeDef;

HAL_StatusTypeDef SK6812_Init(SK6812_HandleTypeDef* stripHandle, TIM_HandleTypeDef* timer, uint32_t channel, uint8_t* dma_buf, uint16_t dma_buf_len, SK6812_DATA_RGB* led_data, uint16_t num_leds);
void SK6812_SetColour(SK6812_HandleTypeDef* stripHandle, uint16_t index, uint8_t red, uint8_t green, uint8_t blue);
HAL_StatusTypeDef SK6812_Update(SK6812_HandleTypeDef* stripHandle);
void SK6812_DMACompleteCallback(SK6812_HandleTypeDef* stripHandle);

#endif /* INC_SK6812_LEDSTRIP_H_ */
