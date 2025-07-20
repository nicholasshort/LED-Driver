/*
 * SK6812-LEDStrip.h
 *
 *  Created on: Apr 29, 2025
 *      Author: nick
 */

#include "stm32f1xx_hal.h"

#ifndef INC_SK6812_LEDSTRIP_H_
#define INC_SK6812_LEDSTRIP_H_

#define SK6812_TIM 			htim3
#define SK6812_TIM_CHANNEL	TIM_CHANNEL_3

#define SK6812_NUM_LEDS 	120

// Sending bits at 800kHz, duty cycle measured from 0-90 (0%-100%)
#define SK6812_T0H_VAL 		22 // 0.3us
#define SK6812_T1H_VAL 		43 // 0.6us

#define SK6812_RST_PERIODS 	64 // 80us/(1.25us per period)

#define SK6812_BITS_PER_LED 24

#define SK6812_DMA_BUF_LEN 	((SK6812_NUM_LEDS * SK6812_BITS_PER_LED) + SK6812_RST_PERIODS)


typedef union {

	struct { // Bit order: G7-G0,R7-R0,B7-B0
		uint8_t green;
		uint8_t red;
		uint8_t blue;
	} colour;

	uint32_t data;

} SK6812_DATA_RGB;

extern 			SK6812_DATA_RGB		SK6812_LEDSTRIP_DATA[SK6812_NUM_LEDS];
extern 			uint8_t				SK6812_DMA_BUF[SK6812_DMA_BUF_LEN];
extern volatile uint8_t				SK6812_DMA_COMPLETE_FLAG;
extern 			TIM_HandleTypeDef   SK6812_TIM;



HAL_StatusTypeDef	SK6812_Init();
void				SK6812_SetColour(uint8_t index, uint8_t red, uint8_t green, uint8_t blue);
HAL_StatusTypeDef	SK6812_Update();
void				SK6812_Callback();

#endif /* INC_SK6812_LEDSTRIP_H_ */
