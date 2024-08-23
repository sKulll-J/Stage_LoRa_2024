/*
 * lib_sen7.c
 *
 *  Created on: Jul 17, 2024
 *      Author: G45
 */
#include "lib_sen7.h"


senDatas senDt;


void SEN7_Init(TIM_HandleTypeDef* tim,GPIO_TypeDef* SCK_GPIO_Port,uint16_t SCK_GPIO_Pin,GPIO_TypeDef* DT_GPIO_Port,uint16_t DT_GPIO_Pin, int _gain)
{
	senDt.tim= tim;
	senDt.SCK_GPIO_Port = SCK_GPIO_Port;
	senDt.SCK_GPIO_Pin = SCK_GPIO_Pin;
	senDt.DT_GPIO_Port =DT_GPIO_Port;
	senDt.DT_GPIO_Pin=DT_GPIO_Pin;
	senDt.gain = _gain;

	HAL_Delay(100);
	SEN7_GetRaw();
}


void delay_us (uint16_t us)
{

	__HAL_TIM_SET_COUNTER(senDt.tim,0);  // set the counter value a 0
	while (__HAL_TIM_GET_COUNTER(senDt.tim) < us);  // wait for the counter to reach the us input in the parameter
}

uint32_t SEN7_GetRaw()
{
	uint32_t mot=0;

	int bit_s = 24;
	int gain = 1; //1:128 | 2:32 | 3:64

	for(int i=0;i<bit_s; i++)
	{
		HAL_GPIO_WritePin(senDt.SCK_GPIO_Port, senDt.SCK_GPIO_Pin, GPIO_PIN_SET);
		delay_us (1);
		mot|=(HAL_GPIO_ReadPin(senDt.DT_GPIO_Port, senDt.DT_GPIO_Pin)==GPIO_PIN_SET)<<((bit_s-1)-i);

		HAL_GPIO_WritePin(senDt.SCK_GPIO_Port, senDt.SCK_GPIO_Pin, GPIO_PIN_RESET);
		delay_us (1);
	}
	//envoie 1,2 ou 3 bit de plus pour spécifier le gain
	for(int i=0;i<senDt.gain; i++)
	{
		HAL_GPIO_WritePin(senDt.SCK_GPIO_Port, senDt.SCK_GPIO_Pin, GPIO_PIN_SET);
		delay_us (1);

		HAL_GPIO_WritePin(senDt.SCK_GPIO_Port, senDt.SCK_GPIO_Pin, GPIO_PIN_RESET);
		delay_us (1);
	}

	return mot;
}


uint32_t SEN7_Tare()//definie le poid actuelle comme le zero
{
	senDt.zero = SEN7_GetRaw();
	return senDt.zero;
}

float SEN7_Calibrate(int16_t poid)
{
	senDt.coef=((float)poid)/(SEN7_GetRaw()-senDt.zero);
	return senDt.coef;
}

float SEN7_GetWeight()
{
	float value = ((int)((long)(SEN7_GetRaw()-senDt.zero)*senDt.coef/10))/100.0;
	return value;
}
