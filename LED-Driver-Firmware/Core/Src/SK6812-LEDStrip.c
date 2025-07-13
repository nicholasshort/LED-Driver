/*
 * SK6812-LEDStrip.c
 *
 *  Created on: Apr 29, 2025
 *      Author: nick
 */

#include "SK6812-LEDStrip.h"

SK6812_DATA_RGB SK6812_LEDSTRIP_DATA[SK6812_NUM_LEDS];

uint8_t	SK6812_DMA_BUF[SK6812_DMA_BUF_LEN];

volatile uint8_t SK6812_DMA_COMPLETE_FLAG;


HAL_StatusTypeDef SK6812_Init() {

	HAL_StatusTypeDef ret = HAL_TIM_PWM_Init(&SK6812_TIM);

	for (uint16_t i = 0; i < SK6812_DMA_BUF_LEN; i++) {

		SK6812_DMA_BUF[i] = 0;

	}

	SK6812_DMA_COMPLETE_FLAG = 1;

	return ret;

}

void SK6812_SetColour(uint8_t index, uint8_t red, uint8_t green, uint8_t blue) {

	SK6812_LEDSTRIP_DATA[index].colour.red = red;
	SK6812_LEDSTRIP_DATA[index].colour.green = green;
	SK6812_LEDSTRIP_DATA[index].colour.blue = blue;

}

HAL_StatusTypeDef SK6812_Update() {

	if (!SK6812_DMA_COMPLETE_FLAG) {

		return HAL_BUSY;

	}

	uint16_t bufIndex = 0;

	for (uint8_t ledIndex = 0; ledIndex < SK6812_NUM_LEDS; ledIndex++) {

		uint8_t transmitBitIndex = 0;

		for (uint8_t bitIndex = 0; bitIndex < SK6812_BITS_PER_LED; bitIndex++) {

			transmitBitIndex = (7 - (bitIndex % 8)) + ((bitIndex / 8) * 8);

			if ((SK6812_LEDSTRIP_DATA[ledIndex].data >> transmitBitIndex) & 0x01) {

				SK6812_DMA_BUF[bufIndex] = SK6812_T1H_VAL;

			} else {

				SK6812_DMA_BUF[bufIndex] = SK6812_T0H_VAL;

			}

			bufIndex++;

		}

	}

	// 80us reset period from Init function's 0s

	HAL_StatusTypeDef ret = HAL_TIM_PWM_Start_DMA(&SK6812_TIM, SK6812_TIM_CHANNEL, (uint32_t*)SK6812_DMA_BUF, SK6812_DMA_BUF_LEN);

	if (ret == HAL_OK) {

		SK6812_DMA_COMPLETE_FLAG = 0;

	}

	return ret;

}

void SK6812_Callback() {

	HAL_TIM_PWM_Stop_DMA(&SK6812_TIM, SK6812_TIM_CHANNEL);

	SK6812_DMA_COMPLETE_FLAG = 1;

}
