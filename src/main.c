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
#include "sapi.h"
#include "semphr.h"
#include "fsm_debounce.h"

/*==================[definiciones y macros]==================================*/

#define DEBOUNCE_TICK_TIME 40 / portTICK_RATE_MS


/*==================[definiciones de datos internos]=========================*/
int aux = 0;
unsigned int peso = 0;

/*==================[definiciones de datos externos]=========================*/
DEBUG_PRINT_ENABLE;
// Crear cola
QueueHandle_t queue_tec_pulsada;

/*==================[declaraciones de funciones internas]====================*/

/*==================[declaraciones de funciones externas]====================*/

// Prototipo de funcion de la tarea
void tarea_enviar_wifi( void* taskParmPtr );
void tarea_recibir_wifi( void* taskParmPtr );
void tarea_peso( void* taskParmPtr );

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
    gpioWrite( LED3 , ON );

    queue_tec_pulsada = xQueueCreate(1,sizeof(unsigned int));

    // Crear tarea en freeRTOS

    BaseType_t res =
    xTaskCreate(
    		tarea_recibir_wifi,                     	// Funcion de la tarea a ejecutar
    		( const char * )"tarea_recibir_wifi",   // Nombre de la tarea como String amigable para el usuario
    		configMINIMAL_STACK_SIZE*2, 	// Cantidad de stack de la tarea
    		0,        	// Parametros de tarea
    		tskIDLE_PRIORITY+1,         	// Prioridad de la tarea
    		0                           	// Puntero a la tarea creada en el sistema
    	);

    if(res == pdFAIL)
        {
        	//error
        }

    res =
    xTaskCreate(
        tarea_enviar_wifi,                     // Funcion de la tarea a ejecutar
        ( const char * )"tarea_enviar_wifi",   // Nombre de la tarea como String amigable para el usuario
        configMINIMAL_STACK_SIZE*2, // Cantidad de stack de la tarea
        0,                          // Parametros de tarea
        tskIDLE_PRIORITY+1,         // Prioridad de la tarea
        0                           // Puntero a la tarea creada en el sistema
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



// Tarea que recibe un caracter de wifi (aca lo hacemos con presionar una vez la tecla 1)
void tarea_recibir_wifi( void* taskParmPtr )
{
	fsmButtonInit();
	peso = 0;
	//debugPrintlnString( "Tarea Recibir WIFI" );

	while( 1 )
	{
		fsmButtonUpdate( TEC1 );
		vTaskDelay( 1 / portTICK_RATE_MS );
	}
}

//Tarea que pesa al usuario
void tarea_peso( void* taskParmPtr )
{
	//debugPrintlnString( "Tarea peso" );
	peso = 100;	//Peso del usuario
	xQueueSend(queue_tec_pulsada , &peso,  portMAX_DELAY  );	//Se manda el peso por una cola
	vTaskDelete(NULL);		//Se elimina la tarea luego de que termina de enviar la cola
}

// Tarea que envia el valor por wifi (aca se envia el valor de la cola por puerto serie y se prende un led)
void tarea_enviar_wifi( void* taskParmPtr )
{
	// ---------- CONFIGURACIONES ------------------------------
		TickType_t xPeriodicity =  1000 / portTICK_RATE_MS;		// Tarea periodica cada 1000 ms
		TickType_t xLastWakeTime = xTaskGetTickCount();
	    // ---------- REPETIR POR SIEMPRE --------------------------
		//debugPrintlnString( "Tarea Enviar WIFI" );
    while( TRUE )
    {
    	unsigned int peso2;
		//Se recibe el valor de la cola y se lo imprime por puerto serie
		xQueueReceive(queue_tec_pulsada , &peso2,  portMAX_DELAY );			// Esperamos tecla
		debugPrintlnString( "El peso es: ");
		debugPrintUInt(peso2);
		debugPrintEnter();

		gpioWrite( LED1 , 1 );
		vTaskDelay( 40 / portTICK_RATE_MS );
		gpioWrite( LED1 , 0 );

		vTaskDelayUntil( &xLastWakeTime , xPeriodicity );

    }
}

/*==================[fin del archivo]========================================*/
