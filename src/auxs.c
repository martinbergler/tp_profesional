/*=============================================================================
 * Copyright (c) 2020, Martin N. Menendez <menendezmartin81@gmail.com>
 * All rights reserved.
 * License: Free
 * Date: 2020/09/03
 * Version: v1.1
 *===========================================================================*/

/*==================[inclusiones]============================================*/
#include "auxs.h"
/*==================[definiciones y macros]==================================*/

/*==================[definiciones de datos internos]=========================*/

/*==================[definiciones de datos externos]=========================*/
gpioMap_t leds[]   = {LEDB,LED1,LED2,LED3};

/*==================[declaraciones de funciones internas]====================*/

/*==================[declaraciones de funciones externas]====================*/

/*==================[tareas]====================*/

// Funcion que inicializa la estructura principal


// Funcion que crea y valida las tareas de FreeRTOS
void tarea_crear(TaskFunction_t tarea,const char * const nombre,uint8_t stack,void * const parametros,uint8_t prioridad,TaskHandle_t * const puntero)
{
	// Crear tarea en freeRTOS
	BaseType_t res = xTaskCreate(tarea,nombre,configMINIMAL_STACK_SIZE*stack,parametros,tskIDLE_PRIORITY+prioridad,puntero);                         		// Puntero a la tarea creada en el sistema

	// Gestion de errores
	if(res == pdFAIL)
	{
		gpioWrite( LED_ERROR , ON );
		printf( MSG_ERROR_TASK );
		while(TRUE);						// VER ESTE LINK: https://pbs.twimg.com/media/BafQje7CcAAN5en.jpg
	}
}

// Funcion que crea y valida las colas de FreeRTOS
void cola_crear(QueueHandle_t nombre, UBaseType_t longitud, UBaseType_t escala){
	nombre = xQueueCreate(longitud, escala);
	if (nombre == NULL){
		gpioWrite( LED_ERROR , ON );
		printf( MSG_ERROR_QUEUE);
		while(TRUE);
	}
}

void sem_crear(SemaphoreHandle_t nombre){
	nombre =  xSemaphoreCreateBinary();
	if (nombre == NULL){
		gpioWrite( LED_ERROR , ON );
		printf( MSG_ERROR_SEM);
		while(TRUE);
	}
}
