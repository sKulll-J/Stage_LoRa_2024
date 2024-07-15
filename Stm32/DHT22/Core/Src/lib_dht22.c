/*
 * lib_dht22.c
 *
 *  Created on: Jul 11, 2024
 *      Author: erwan
 */
#include "lib_dht22.h"
#include <stdint.h>



data myDt;

//initialise le dht22
void DHT_Init(GPIO_TypeDef* _GPIO_Port, uint16_t _GPIO_Pin, TIM_HandleTypeDef* _tim)
{
	myDt.GPIO_Port = _GPIO_Port;
	myDt.GPIO_Pin = _GPIO_Pin;
	myDt.tim = _tim;
	  HAL_TIM_Base_Start(myDt.tim);
}

void delay_us (uint16_t us)
{

	__HAL_TIM_SET_COUNTER(myDt.tim,0);  // set the counter value a 0
	while (__HAL_TIM_GET_COUNTER(myDt.tim) < us);  // wait for the counter to reach the us input in the parameter
}

uint8_t bitPosToVal(uint16_t mot) //recupere la position du bit a 1 dans un registre
{
	for(uint16_t i=0; i<16;i++)
	{
		if((mot>>i)==1)
			return i;
	}
	return 0;
}


uint8_t verifcation_rep(void)
{
	uint8_t reponse = 0;
	HAL_Delay(1);
	//initialisation du capteur
	myDt.GPIO_Port->MODER |= (0x1UL<<(2*bitPosToVal(myDt.GPIO_Pin)));

	HAL_GPIO_WritePin(myDt.GPIO_Port, myDt.GPIO_Pin, 0);
	HAL_Delay(1);
	myDt.GPIO_Port->MODER &= ~(0x3UL<<(2*bitPosToVal(myDt.GPIO_Pin)));
//	myDt.GPIO_Port->MODER &= ~(GPIO_MODER_MODER0);
	//récupération de la présence

	delay_us(40);                                     //attente de la fin du pin 0
	if (!(HAL_GPIO_ReadPin (myDt.GPIO_Port, myDt.GPIO_Pin)))	//si la ligne est a 0
		{
			delay_us (80);								//attenjte de la montée a 1
			if ((HAL_GPIO_ReadPin (myDt.GPIO_Port, myDt.GPIO_Pin)))
				reponse = 1;	//si retour à 1 réponse à 1
			else reponse = -1;									//sinon rép -1
		}
	while ((HAL_GPIO_ReadPin (myDt.GPIO_Port, myDt.GPIO_Pin)));   // attente de retour de la ligne au niveau initial
	return reponse;
}

uint32_t DHT_Read (void)
{
	uint32_t i,j;

	for (j=0;j<32;j++)
	{
		while (!(HAL_GPIO_ReadPin (myDt.GPIO_Port, myDt.GPIO_Pin)));   // On attends que le pin soit a l'état haut
		delay_us (40);   									// attente de 40 us
		if (!(HAL_GPIO_ReadPin (myDt.GPIO_Port, myDt.GPIO_Pin)))   //si le pin est à 0
		{
			i&= ~(1<<(31-j));   						// on écrit 0
		}
		else i|= (1<<(31-j));  						// si le pin est à 1, on ecrit 1
		while ((HAL_GPIO_ReadPin (myDt.GPIO_Port, myDt.GPIO_Pin)));  // on attends le retour à l'état initial
	}
	return i;
}

void DHT_GetValue()
{
	uint32_t val;
	if(verifcation_rep()==1)
		val = DHT_Read();
	myDt.humi = (val>>16)/10;
	myDt.temp= (val&0xFFFF)/10;
}

float DHT_GetHumi()
{
	return myDt.humi;
}

float DHT_GetTemp()
{
	return myDt.temp;
}

data DHT_GetDt()
{
	return myDt;
}
