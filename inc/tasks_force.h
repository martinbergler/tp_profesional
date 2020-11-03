/*===========================================================================*/
#ifndef _TASKS_FORCE_H_
#define _TASKS_FORCE_H_

/*==================[inclusiones]============================================*/
// Includes de FreeRTOS
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"
#include "sapi.h"
#include "auxs.h"

#include "FreeRTOSConfig.h"
/*==================[definiciones y macros]==================================*/
#define DataPin 		GPIO8
#define ClockPin 		GPIO7
#define SCALE			17110

#define GAIN_128		24
#define GAIN_64			25
#define GAIN_32			26
#define XOR_VALUE 		0x800000
#define CANT_MEDICIONES	15

#define TASK_RATE_1000_MS	1000
#define TASK_RATE_500_MS	500
#define TASK_RATE_50_MS		50

#define TASK_RATE_1000 pdMS_TO_TICKS(TASK_RATE_1000_MS)
#define TASK_RATE_500 pdMS_TO_TICKS(TASK_RATE_500_MS)
#define TASK_RATE_50 pdMS_TO_TICKS(TASK_RATE_50_MS)

/*==================[definiciones de datos internos]=========================*/

/*==================[definiciones de datos externos]=========================*/
DEBUG_PRINT_ENABLE;

//Handle de la cola
QueueHandle_t cola_datos_calculados;
QueueHandle_t cola_tarar;
QueueHandle_t cola_fuerza;
SemaphoreHandle_t sem_medir_fuerza;

/*==================[declaraciones de funciones internas]====================*/

/*==================[declaraciones de funciones externas]====================*/
TickType_t get_diff();
void clear_diff();

// Prototipo de funcion de la tarea
void tarea_medir( void* taskParmPtr );
void tarea_esperar( void* taskParmPtr );
void tarea_promediar( void* taskParmPtr );
void tarea_tarar( void* taskParmPtr );

// Handles de las tareas
TaskHandle_t TaskHandle_esperar;
TaskHandle_t TaskHandle_medir;

#endif /* _TASKS_FORCE_H_ */
