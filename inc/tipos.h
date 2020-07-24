/*
 * tipos.h
 *
 *  Created on: 1 nov. 2019
 *      Author: Buck
 */

#ifndef RTOS_1_10MA_CLASE_2_1_3_MULTI_INC_TIPOS_H_
#define RTOS_1_10MA_CLASE_2_1_3_MULTI_INC_TIPOS_H_

#include "../../../RTOS1/Ej_3_1_multi/inc/FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "sapi.h"
#include "semphr.h"

#define CANT	2


typedef enum
{
    STATE_BUTTON_UP,
    STATE_BUTTON_DOWN,
    STATE_BUTTON_FALLING,
    STATE_BUTTON_RISING
} fsmButtonState_t;

typedef struct
{
	gpioMap_t tecla;			//config
	gpioMap_t led;				//config

	TickType_t tiempo_medido;	//variables
	fsmButtonState_t fsmButtonState;//variables
	TickType_t tiempo_down;		//variables
	TickType_t tiempo_up;		//variables

	uint8_t contFalling  ;		//variables
	uint8_t contRising  ;		//variables

	QueueHandle_t queue_tec_pulsada;
} tLedTecla;


#endif /* RTOS_1_10MA_CLASE_2_1_3_MULTI_INC_TIPOS_H_ */
