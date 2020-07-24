/* Copyright 2017-2018, Eric Pernia
 * All rights reserved.
 *
 * This file is part of sAPI Library.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*==================[inlcusiones]============================================*/

// Includes de FreeRTOS
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "tipos.h"

// sAPI header
#include "sapi.h"

#include "FreeRTOSConfig.h"

/*==================[definiciones y macros]==================================*/

/*==================[definiciones de datos internos]=========================*/

unsigned int weight;
/*==================[definiciones de datos externos]=========================*/


DEBUG_PRINT_ENABLE;

QueueHandle_t queue_calc_weight;

/*==================[declaraciones de funciones internas]====================*/

/*==================[declaraciones de funciones externas]====================*/
TickType_t get_diff();
void clear_diff();

// Prototipo de funcion de la tarea
void tarea_weight( void* taskParmPtr );
void tarea_Rx_WIFI( void* taskParmPtr );
void tarea_Tx_WIFI( void* taskParmPtr );


/*==================[funcion principal]======================================*/

// FUNCION PRINCIPAL, PUNTO DE ENTRADA AL PROGRAMA LUEGO DE ENCENDIDO O RESET.
int main( void )
{
    // ---------- CONFIGURACIONES ------------------------------
    // Inicializar y configurar la plataforma
    boardConfig();

    // UART for debug messages
    debugPrintConfigUart( UART_USB, 115200 );
    debugPrintlnString( "TP Profesional." );

    // Led para dar señal de vida
    gpioWrite( LED3, ON );

    queue_calc_weight = xQueueCreate(1,sizeof(unsigned int));

    weight = 0;

    BaseType_t res =
    xTaskCreate(
    	tarea_Rx_WIFI,                     // Funcion de la tarea a ejecutar
        ( const char * )"tarea_Rx",   // Nombre de la tarea como String amigable para el usuario
        configMINIMAL_STACK_SIZE*2, 	// Cantidad de stack de la tarea
        0,                          	// Parametros de tarea
        tskIDLE_PRIORITY+1,         	// Prioridad de la tarea
        0                           	// Puntero a la tarea creada en el sistema
    );

    if(res == pdFAIL)
    {
    	//error
    }

    res =
    xTaskCreate(
    	tarea_Tx_WIFI,                     // Funcion de la tarea a ejecutar
        ( const char * )"tarea_Tx",   // Nombre de la tarea como String amigable para el usuario
        configMINIMAL_STACK_SIZE*2, 	// Cantidad de stack de la tarea
        0,                          	// Parametros de tarea
        tskIDLE_PRIORITY+1,         	// Prioridad de la tarea
        0                           	// Puntero a la tarea creada en el sistema
    );

    if(res == pdFAIL)
    {
    	//error
    }

    	// Iniciar scheduler
    vTaskStartScheduler();

    // ---------- REPETIR POR SIEMPRE --------------------------
    while( TRUE )
    {
        // Si cae en este while 1 significa que no pudo iniciar el scheduler
    }

    // NO DEBE LLEGAR NUNCA AQUI, debido a que a este programa se ejecuta
    // directamenteno sobre un microcontroladore y no es llamado por ningun
    // Sistema Operativo, como en el caso de un programa para PC.
    return 0;
}

/*==================[definiciones de funciones internas]=====================*/

/*==================[definiciones de funciones externas]=====================*/

void tarea_Rx_WIFI( void* taskParmPtr )
{
	fsmButtonInit();

	while( 1 )
	{
		fsmButtonUpdate( TEC1 );
	 	vTaskDelay( 1 / portTICK_RATE_MS );
	}
}

// Implementacion de funcion de la tarea
void tarea_Tx_WIFI( void* taskParmPtr )
{
    while( 1 )
    {
    	if(xQueueReceive(queue_calc_weight , &weight,  portMAX_DELAY)){			// Esperamos tecla

			gpioWrite( LED1, ON );
			vTaskDelay(40 / portTICK_RATE_MS);
			gpioWrite( LED1, OFF );

			debugPrintString( "El peso es: " );
			debugPrintlnUInt(weight);
    	}

    }
}

// Implementacion de funcion de la tarea
void tarea_weight( void* taskParmPtr )
{
    // ---------- CONFIGURACIONES ------------------------------

    // ---------- REPETIR POR SIEMPRE --------------------------

	TickType_t dif = *( (TickType_t*)  taskParmPtr );

	gpioWrite( LEDB , 1 );
	vTaskDelay( dif );
	gpioWrite( LEDB , 0 );

	weight = 53;	//Peso fijo
	xQueueSend(queue_calc_weight , &weight,  portMAX_DELAY);

	vTaskDelete(NULL);
}


/*==================[fin del archivo]========================================*/
