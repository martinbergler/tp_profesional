/*===========================================================================*/
#ifndef _TASKS_WIFI_H_
#define _TASKS_WIFI_H_

/*==================[inclusiones]============================================*/
// Includes de FreeRTOS
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"
#include "sapi.h"
//#include "auxs.h"
#include "tasks_force.h"

#include "FreeRTOSConfig.h"
/*==================[definiciones y macros]==================================*/

/*==================[definiciones de datos internos]=========================*/

/*==================[definiciones de datos externos]=========================*/

/*==================[declaraciones de funciones internas]====================*/

/*==================[declaraciones de funciones externas]====================*/
TickType_t get_diff();
void clear_diff();

// Prototipo de funcion de la tarea
void tarea_Rx_WIFI( void* taskParmPtr );
void tarea_Tx_WIFI( void* taskParmPtr );
/*void tarea_medir( void* taskParmPtr );
void tarea_esperar( void* taskParmPtr );
void tarea_promediar( void* taskParmPtr );
void tarea_tarar( void* taskParmPtr );

// Handles de las tareas
TaskHandle_t TaskHandle_esperar;
TaskHandle_t TaskHandle_medir;*/

#endif /* _TASKS_WIFI_H_ */
