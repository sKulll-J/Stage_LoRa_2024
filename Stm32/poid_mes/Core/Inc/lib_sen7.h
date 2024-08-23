/*
 * lib_sen7.h
 *
 *  Created on: Jul 17, 2024
 *      Author: G45
 */

#ifndef INC_LIB_SEN7_H_
#define INC_LIB_SEN7_H_

#include <stdlib.h>
#include <stdint.h>
#include "stm32l4xx_hal.h"

#define SEN7_GAIN_128 1
#define SEN7_GAIN_32 2
#define SEN7_GAIN_64 3

typedef struct
{
	TIM_HandleTypeDef* tim;
	GPIO_TypeDef* SCK_GPIO_Port;
	uint16_t SCK_GPIO_Pin;
	GPIO_TypeDef* DT_GPIO_Port;
	uint16_t DT_GPIO_Pin;

	int gain;

	uint32_t zero;
	float coef;
}senDatas;

void SEN7_Init(TIM_HandleTypeDef* tim,GPIO_TypeDef* SCK_GPIO_Port,uint16_t SCK_GPIO_Pin,GPIO_TypeDef* DT_GPIO_Port,uint16_t DT_GPIO_Pin, int _gain);
float SEN7_GetWeight();

uint32_t SEN7_Tare();
float SEN7_Calibrate(int16_t poid);


uint32_t SEN7_GetRaw();

#endif /* INC_LIB_SEN7_H_ */
