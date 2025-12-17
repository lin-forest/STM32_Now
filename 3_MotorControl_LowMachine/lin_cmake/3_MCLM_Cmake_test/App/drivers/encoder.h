#ifndef __ENCODER_H
#define __ENCODER_H

#include "main.h"

void Encoder_Init(TIM_HandleTypeDef *htim);
int16_t Encoder_GetCount(TIM_HandleTypeDef *htim);

#endif /* __ENCODER_H */
