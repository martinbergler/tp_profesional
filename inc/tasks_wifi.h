/*===========================================================================*/
#ifndef _TASKS_WIFI_H_
#define _TASKS_WIFI_H_

/*==================[inclusiones]============================================*/

// Includes de FreeRTOS
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "sapi.h"
#include "FreeRTOSConfig.h"
#include "auxs.h"
#include "tasks_force.h"

/*==================[definiciones y macros]==================================*/

/*==================[definiciones de datos internos]=========================*/

/*==================[definiciones de datos externos]=========================*/

/*==================[declaraciones de funciones internas]====================*/

/*==================[declaraciones de funciones externas]====================*/

// Prototipo de funcion de la tarea
void task_Rx_WIFI( void* taskParmPtr );
void task_Tx_WIFI( void* taskParmPtr );


#endif /* _TASKS_WIFI_H_ */
