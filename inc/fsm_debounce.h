/*
 * fsm_debounce.h
 *
 *  Created on: Jun 13, 2020
 *      Author: luchao90
 */

#ifndef PROGRAMS_RTOS_1_EJ_1_3_INC_FSM_DEBOUNCE_H_
#define PROGRAMS_RTOS_1_EJ_1_3_INC_FSM_DEBOUNCE_H_

#include "../../../RTOS1/Ej_1_3/inc/FreeRTOSConfig.h"
#include "sapi.h"
#include "FreeRTOS.h"
#include "task.h"

typedef enum
{
    STATE_BUTTON_UP,
    STATE_BUTTON_DOWN,
    STATE_BUTTON_FALLING,
    STATE_BUTTON_RISING
} fsmButtonState_t;

void fsmButtonError( void );
void fsmButtonInit( void );
void fsmButtonUpdate( gpioMap_t tecla );
void buttonPressed( void );
void buttonReleased( void );

#endif /* PROGRAMS_RTOS_1_EJ_1_3_INC_FSM_DEBOUNCE_H_ */
