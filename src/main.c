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

/*==================[definiciones y macros]==================================*/

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

    xTaskCreate(
    		tarea_recibir_wifi,                     	// Funcion de la tarea a ejecutar
    		( const char * )"tarea_recibir_wifi",   // Nombre de la tarea como String amigable para el usuario
    		configMINIMAL_STACK_SIZE*2, 	// Cantidad de stack de la tarea
    		0,        	// Parametros de tarea
    		tskIDLE_PRIORITY+1,         	// Prioridad de la tarea
    		0                           	// Puntero a la tarea creada en el sistema
    	);

    xTaskCreate(
        tarea_enviar_wifi,                     // Funcion de la tarea a ejecutar
        ( const char * )"tarea_enviar_wifi",   // Nombre de la tarea como String amigable para el usuario
        configMINIMAL_STACK_SIZE*2, // Cantidad de stack de la tarea
        0,                          // Parametros de tarea
        tskIDLE_PRIORITY+1,         // Prioridad de la tarea
        0                           // Puntero a la tarea creada en el sistema
    );


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


// Tarea que recibe un caracter de wifi (aca lo hacemos con presionar una vez la tecla 1)
void tarea_recibir_wifi( void* taskParmPtr )
{
	peso = 0;

	while( 1 )
	{
		if( !gpioRead( TEC1 ) )
		{
			if (aux == 0){	//aux se usa para saber si se quiere pesar a la persona o se quiere medir el salto
							//(la parte del salto todavia no esta implementada)
				// Crea la tarea que pesa al usuario
			    xTaskCreate(
			    	tarea_peso,                     // Funcion de la tarea a ejecutar
			        ( const char * )"tarea_peso",  	// Nombre de la tarea como String amigable para el usuario
			        configMINIMAL_STACK_SIZE*2, 	 // Cantidad de stack de la tarea
					0,            					// Parametros de tarea
			        tskIDLE_PRIORITY+2,         	// Prioridad de la tarea
			        0                           	// Puntero a la tarea creada en el sistema
			    );
			}
		}
	}
}

//Tarea que pesa al usuario
void tarea_peso( void* taskParmPtr )
{
	peso = rand();	//Peso del usuario
	aux++;	//Se indica que ya se peso y que lo siguiente va a ser medir el salto, cuando lo indique el wifi
	xQueueSend(queue_tec_pulsada , &peso,  portMAX_DELAY  );	//Se manda el peso por una cola
	vTaskDelete(NULL);		//Se elimina la tarea luego de que termina de enviar la cola
}

// Tarea que envia el valor por wifi (aca se envia el valor de la cola por puerto serie y se prende un led)
void tarea_enviar_wifi( void* taskParmPtr )
{
    while( TRUE )
    {
    	//Se recibe el valor de la cola y se lo imprime por puerto serie
    	xQueueReceive(queue_tec_pulsada , &peso,  portMAX_DELAY );			// Esperamos tecla
    	debugPrintlnString( "El peso es: ");
    	debugPrintUInt(peso);
    	debugPrintEnter();

    	//Se prende un led
    	gpioWrite(LED1, 1 );
    	vTaskDelay(500);
    	gpioWrite(LED1, 0 );

    	//Se resetea el valor de aux porque por ahora solo se mide el peso
    	aux = 0;

    }
}

/*==================[fin del archivo]========================================*/
