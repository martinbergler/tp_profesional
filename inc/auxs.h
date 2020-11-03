/*===========================================================================*/
#ifndef _AUXS_H_
#define _AUXS_H_

/*==================[inclusiones]============================================*/
#include "FreeRTOS.h"
#include "task.h"
#include "sapi.h"
#include "semphr.h"
#include "queue.h"
#include <string.h>
#include "FreeRTOSConfig.h"

/*==================[definiciones y macros]==================================*/
#define LED_ERROR 		LEDR
#define MAX_SEM 3

#define PRIORITY 	1
#define SIZE     	configMINIMAL_STACK_SIZE*2
#define STR_AUX  	20
#define SIZE_UINT 	sizeof(unsigned int)

#define MSG_ERROR_TASK   "Error al crear las tareas. \r\n"
#define MSG_ERROR_SEM	 "Error al crear los semaforos.\r\n"
#define MSG_ERROR_QUEUE	 "Error al crear las colas.\r\n"
/*==================[prototipos]=========================*/
void tarea_crear(TaskFunction_t tarea,const char * const nombre,uint8_t stack,void * const parametros,uint8_t prioridad,TaskHandle_t * const puntero);
void cola_crear(QueueHandle_t nombre,UBaseType_t longitud,UBaseType_t escala);
void sem_crear(SemaphoreHandle_t nombre);

#endif /* _AUXS_H_ */
