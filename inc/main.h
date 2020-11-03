/*===========================================================================*/
#ifndef _MAIN_H_
#define _MAIN_H_

/*==================[inclusiones]============================================*/

// Includes de FreeRTOS
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "sapi.h"
#include "FreeRTOSConfig.h"

#include "tasks_wifi.h"
#include "tasks_force.h"
#include "auxs.h"
/*==================[definiciones de datos externos]=========================*/
DEBUG_PRINT_ENABLE;

/*==================[definiciones y macros]==================================*/
#define BAUD_RATE 115200
#define UART UART_USB

#endif /* _MAIN_H_ */
