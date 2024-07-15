/*
 * lib_dht22.h
 *
 *  Created on: Jul 11, 2024
 *      Author: erwan
 */

#ifndef INC_LIB_DHT22_H_
#define INC_LIB_DHT22_H_

#include <stdint.h>
#include "stm32l4xx_hal.h"

typedef struct{
	float humi;
	float temp;
	TIM_HandleTypeDef* tim;
	GPIO_TypeDef* GPIO_Port;
	uint16_t GPIO_Pin;

}data;

void DHT_Init(GPIO_TypeDef* _GPIO_Port, uint16_t _GPIO_Pin, TIM_HandleTypeDef* _tim);
void delay_us (uint16_t us);
uint8_t verifcation_rep(void);
uint32_t DHT_Read (void);
uint8_t bitPosToVal(uint16_t mot);
void DHT_GetValue();
float DHT_GetHumi();
float DHT_GetTemp();

data DHT_GetDt();

#endif /* INC_LIB_DHT22_H_ */
