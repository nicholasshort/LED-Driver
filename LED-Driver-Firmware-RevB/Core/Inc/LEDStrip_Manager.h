#ifndef LEDSTRIP_MANAGER_H
#define LEDSTRIP_MANAGER_H

#include "SK6812-LEDStrip.h"

typedef enum {
    LEDSTRIP_1,
    LEDSTRIP_2,
    LEDSTRIP_3,
    LEDSTRIP_4,
    LEDSTRIP_5,
    LEDSTRIP_6,
    LEDSTRIP_7,
    LEDSTRIP_8,
    LEDSTRIP_COUNT
} LEDStrip_id;

HAL_StatusTypeDef LEDStrip_Manager_Init();
SK6812_HandleTypeDef* LEDStrip_Manager_Get_Strip_Handle(LEDStrip_id id);

extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim8;
extern DMA_HandleTypeDef hdma_tim3_ch1_trig;
extern DMA_HandleTypeDef hdma_tim3_ch3;
extern DMA_HandleTypeDef hdma_tim3_ch4_up;
extern DMA_HandleTypeDef hdma_tim5_ch1;
extern DMA_HandleTypeDef hdma_tim5_ch2;
extern DMA_HandleTypeDef hdma_tim5_ch3_up;
extern DMA_HandleTypeDef hdma_tim5_ch4_trig;
extern DMA_HandleTypeDef hdma_tim8_ch1;

#endif // LEDSTRIP_MANAGER_H