/*=============================================================================
 * Copyright (c) 2020, Martin N. Menendez <menendezmartin81@gmail.com>
 * All rights reserved.
 * License: Free
 * Date: 2020/09/03
 * Version: v1.1
 *===========================================================================*/
#ifndef _AUXS_H_
#define _AUXS_H_

/*==================[inclusiones]============================================*/
#include "FreeRTOS.h"
#include "task.h"
#include "sapi.h"
#include "semphr.h"
#include "tasks_force.h"

#include <string.h>
#include "FreeRTOSConfig.h"

/*==================[definiciones y macros]==================================*/
#define LED_ERROR 		LEDR
#define MAX_SEM 3

#define PRIORITY 1
#define SIZE     2
#define STR_AUX  20
#define SIZE_UINT sizeof(unsigned int)

#define MSG_ERROR_TASK   "Error al crear las tareas. \r\n"
#define MSG_ERROR_SEM	 "Error al crear los semaforos.\r\n"
#define MSG_ERROR_QUEUE	 "Error al crear las colas.\r\n"
/*==================[prototipos]=========================*/
//void tecla_led_init(void);
//void tareas_crear(TaskFunction_t tarea,const char * const nombre);
void create_task(TaskFunction_t tarea,const char * const nombre,uint8_t stack,void * const parametros,uint8_t prioridad,TaskHandle_t * const puntero);
void cola_crear(QueueHandle_t nombre,UBaseType_t longitud,UBaseType_t escala);
void sem_crear(SemaphoreHandle_t nombre);

#endif /* _AUXS_H_ */
