/*
 * SK6812-LEDStrip.c
 *
 *  Created on: Apr 29, 2025
 *      Author: nick
 */

 #include "SK6812-LEDStrip.h"
 #include <string.h>
 
 // Sending bits at 800kHz, duty cycle measured from 0-90 (0%-100%)
 #define SK6812_T0H_VAL 		22 // 0.3us
 #define SK6812_T1H_VAL 		43 // 0.6us
 
 HAL_StatusTypeDef SK6812_Init(
         SK6812_HandleTypeDef* stripHandle,
         TIM_HandleTypeDef* timer,
         uint32_t channel,
         uint8_t* dma_buf,
         uint16_t dma_buf_len,
         SK6812_DATA_RGB* led_data,
         uint16_t num_leds
 ) {
 
     stripHandle->timer        = timer;
     stripHandle->channel      = channel;
     stripHandle->dma_buf      = dma_buf;
     stripHandle->dma_buf_len  = dma_buf_len;
     stripHandle->led_data     = led_data;
     stripHandle->num_leds     = num_leds;
     stripHandle->dma_done_flag     = 1;
 
     memset(stripHandle->dma_buf, 0, stripHandle->dma_buf_len);
 
     return HAL_TIM_PWM_Init(timer);
 
 }
 
 
 void SK6812_SetColour(SK6812_HandleTypeDef* stripHandle, uint16_t index, uint8_t red, uint8_t green, uint8_t blue) {
 
     if (index >= stripHandle->num_leds) return;
     stripHandle->led_data[index].colour.red = red;
     stripHandle->led_data[index].colour.green = green;
     stripHandle->led_data[index].colour.blue = blue;
 
 }
 
 
 HAL_StatusTypeDef SK6812_Update(SK6812_HandleTypeDef* stripHandle) {
 
     if (!stripHandle->dma_done_flag) return HAL_BUSY;
 
     uint16_t buf_idx = 0;
     for (uint16_t led = 0; led < stripHandle->num_leds; led++) {
         for (uint8_t bit = 0; bit < SK6812_BITS_PER_LED; bit++) {
             uint8_t bit_pos = (7 - (bit % 8)) + ((bit / 8) * 8);
             uint8_t val = ((stripHandle->led_data[led].data >> bit_pos) & 0x01) ? SK6812_T1H_VAL : SK6812_T0H_VAL;
             stripHandle->dma_buf[buf_idx++] = val;
         }
     }
 
     if (HAL_TIM_PWM_Start_DMA(stripHandle->timer, stripHandle->channel, (uint32_t*)stripHandle->dma_buf, stripHandle->dma_buf_len) == HAL_OK) {
         stripHandle->dma_done_flag = 0;
         return HAL_OK;
     }
     return HAL_ERROR;
 
 }
 
 void SK6812_DMACompleteCallback(SK6812_HandleTypeDef* stripHandle) {
 
     HAL_TIM_PWM_Stop_DMA(stripHandle->timer, stripHandle->channel);
     stripHandle->dma_done_flag = 1;
 
 }