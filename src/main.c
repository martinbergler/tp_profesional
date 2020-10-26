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

#include "sapi.h"

#include "FreeRTOSConfig.h"
/*==================[definiciones y macros]==================================*/

#define DataPin GPIO8
#define ClockPin GPIO7
#define SCALE		17110

/*==================[definiciones de datos internos]=========================*/

int cant_promediada = 15;
unsigned long Count = 0;
volatile unsigned long OFFSET = 0;
bool tarado = false;

/*==================[definiciones de datos externos]=========================*/
DEBUG_PRINT_ENABLE;

//Handle de la cola
QueueHandle_t cola_datos_calculados;
QueueHandle_t cola_tarar;
QueueHandle_t cola_fuerza;
SemaphoreHandle_t sem_medir_fuerza;
//SemaphoreHandle_t sem_promediar_medicion;

/*==================[declaraciones de funciones internas]====================*/

/*==================[declaraciones de funciones externas]====================*/
TickType_t get_diff();
void clear_diff();

// Prototipo de funcion de la tarea
void tarea_Rx_WIFI( void* taskParmPtr );
void tarea_Tx_WIFI( void* taskParmPtr );
void tarea_medir( void* taskParmPtr );
void tarea_esperar( void* taskParmPtr );
void tarea_promediar( void* taskParmPtr );
void tarea_tarar( void* taskParmPtr );

// Handles de las tareas
TaskHandle_t TaskHandle_esperar;
TaskHandle_t TaskHandle_medir;

/*==================[funcion principal]======================================*/

// FUNCION PRINCIPAL, PUNTO DE ENTRADA AL PROGRAMA LUEGO DE ENCENDIDO O RESET.
int main( void )
{
    // ---------- CONFIGURACIONES ------------------------------
    // Inicializar y configurar la plataforma
    boardConfig();
    HX711Config();

    // FALTA VER EL TEMA DE LA TARA AL PRINCIPIO

    // UART for debug messages
    debugPrintConfigUart( UART_USB, 115200 );
    debugPrintlnString( "TP Profesional." );

    // Led para dar señal de vida
    gpioWrite( LED3, ON );

    // Creación de la cola
    cola_datos_calculados = xQueueCreate(cant_promediada,sizeof(unsigned int));
    cola_tarar = xQueueCreate(1,sizeof(unsigned int));
    cola_fuerza = xQueueCreate(1,sizeof(unsigned int));

    // Creación del semáforo
    sem_medir_fuerza = xSemaphoreCreateBinary();
    //sem_promediar_medicion = xSemaphoreCreateCounting( cant_promediada , 0 );


    // Creación de las tareas

    BaseType_t res =
	xTaskCreate(
		tarea_tarar,                     // Funcion de la tarea a ejecutar
		( const char * )"tarea_tarar",   // Nombre de la tarea como String amigable para el usuario
		configMINIMAL_STACK_SIZE*2, 	// Cantidad de stack de la tarea
		0,                          	// Parametros de tarea
		tskIDLE_PRIORITY+2,         	// Prioridad de la tarea
		0                           	// Puntero a la tarea creada en el sistema
	);

	if(res == pdFAIL)
	{
		//error
	}

	res =
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
	unsigned long fu = 0;
	double p = 0;
	char str_aux[50] = {};
    // ---------- CONFIGURACIONES ------------------------------
	TickType_t xPeriodicity =  1000 / portTICK_RATE_MS;		// Tarea periodica cada 1000 ms
	TickType_t xLastWakeTime = xTaskGetTickCount();
	// ---------- REPETIR POR SIEMPRE --------------------------
    while( 1 )
    {
    	if(xQueueReceive(cola_fuerza , &fu,  portMAX_DELAY)){			// Esperamos tecla

			gpioWrite( LED1, ON );
			vTaskDelay(40 / portTICK_RATE_MS);
			gpioWrite( LED1, OFF );

			p = (fu - OFFSET) / SCALE;

			if (p < 0){
				debugPrintString("Negativo corregido");
				p = 0;
			}

			debugPrintlnUInt(OFFSET);
			debugPrintString( "Fuerza: " );
			debugPrintlnUInt(fu);
			sprintf(str_aux, "%d",p);
			debugPrintString( "Peso: " );
			debugPrintString( str_aux );
			debugPrintEnter();
			debugPrintlnInt(p);

			// Delay periódico
			vTaskDelayUntil( &xLastWakeTime , xPeriodicity );
    	}

    }
}

// Tarea que mide la fuerza
void tarea_medir( void* taskParmPtr )
{
	// ---------- CONFIGURACIONES ------------------------------
		TickType_t xPeriodicity =  500 / portTICK_RATE_MS;		// Tarea periodica cada 1000 ms
		TickType_t xLastWakeTime = xTaskGetTickCount();
		TickType_t dif = *( (TickType_t*)  taskParmPtr );

		// Inicialización de parámetros
		unsigned char i;

		//gpioWrite(DataPin,1);
		gpioWrite(ClockPin,0);

		// Crear tarea de espera
			BaseType_t res =
			xTaskCreate(
				tarea_esperar,                     	// Funcion de la tarea a ejecutar
				( const char * )"tarea_esperar",   	// Nombre de la tarea como String amigable para el usuario
				configMINIMAL_STACK_SIZE*2, 	// Cantidad de stack de la tarea
				0,                	// Parametros de tarea
				tskIDLE_PRIORITY+2,         	// Prioridad de la tarea
				&TaskHandle_esperar                          	// Puntero a la tarea creada en el sistema
			);

			if(res == pdFAIL)
			{
				//error
			}

			// Crear tarea de espera
			res =
			xTaskCreate(
				tarea_promediar,                     	// Funcion de la tarea a ejecutar
				( const char * )"tarea_promediar",   	// Nombre de la tarea como String amigable para el usuario
				configMINIMAL_STACK_SIZE*2, 	// Cantidad de stack de la tarea
				0,                	// Parametros de tarea
				tskIDLE_PRIORITY+2,         	// Prioridad de la tarea
				0                          	// Puntero a la tarea creada en el sistema
			);

			if(res == pdFAIL)
			{
				//error
			}

	    // ---------- REPETIR POR SIEMPRE --------------------------
		while ( 1 ){

			// Esperar a que el módulo HX711 esté listo
			if (xSemaphoreTake( sem_medir_fuerza  ,  portMAX_DELAY )){

				gpioWrite( LEDG , 1 );
				vTaskDelay( dif );
				gpioWrite( LEDG , 0 );

				Count = 0;

				// Medir 24 pulsos
				for (i=0;i<24;i++)
					{
						gpioWrite(ClockPin,1);
						Count=Count<<1;
						gpioWrite(ClockPin,0);
						if(gpioRead(DataPin)){

							Count++;
						}
					}
				// Hacer medición final
				gpioWrite(ClockPin,1);
				Count=Count^0x800000;
				gpioWrite(ClockPin,0);



			xQueueSend(cola_datos_calculados , &Count,  portMAX_DELAY);

			}

			// Delay periódico
			vTaskDelayUntil( &xLastWakeTime , xPeriodicity );
		}

}

// Tarea que espera a que el HX711 este listo para medir
void tarea_esperar( void* taskParmPtr )
{
    // ---------- CONFIGURACIONES ------------------------------
	TickType_t xPeriodicity =  50 / portTICK_RATE_MS;		// Tarea periodica cada 1000 ms
	TickType_t xLastWakeTime = xTaskGetTickCount();

    // ---------- REPETIR POR SIEMPRE --------------------------
	while ( 1 ){

			gpioWrite( LEDR , 1 );
			vTaskDelay( 40 / portTICK_RATE_MS );
			gpioWrite( LEDR , 0 );

			if( !gpioRead(DataPin) ){
				// Indicar que el módulo ya está listo para empezar a medir
				xSemaphoreGive( sem_medir_fuerza );
				vTaskDelete(NULL);
			}

			// Delay periódico
			vTaskDelayUntil( &xLastWakeTime , xPeriodicity );

		}

}

// Tarea que promedia los valores medidos
void tarea_promediar( void* taskParmPtr )
{
    // ---------- CONFIGURACIONES ------------------------------
	TickType_t xPeriodicity =  500 / portTICK_RATE_MS;		// Tarea periodica cada 1000 ms
	TickType_t xLastWakeTime = xTaskGetTickCount();

	unsigned long f;
	unsigned long sum = 0;
	int contador = 0;
	unsigned long prom;
    // ---------- REPETIR POR SIEMPRE --------------------------
	while ( 1 ){

			gpioWrite( LEDB , 1 );
			vTaskDelay( 40 / portTICK_RATE_MS );
			gpioWrite( LEDB , 0 );

			if(xQueueReceive(cola_datos_calculados , &f,  portMAX_DELAY)){			// Esperamos tecla
				//debugPrintString( "Espacio en cola: " );
				//debugPrintlnUInt(uxQueueSpacesAvailable(cola_datos_calculados));
				if (contador >= cant_promediada){
					prom = sum / cant_promediada;
					if (!tarado){
						xQueueSend(cola_tarar , &prom,  portMAX_DELAY);
						vTaskDelete(TaskHandle_medir);
						vTaskDelete(NULL);
					}
					else{
						xQueueSend(cola_fuerza , &prom,  portMAX_DELAY);
						vTaskDelete(TaskHandle_medir);
						vTaskDelete(NULL);
					}
				}
				else{
					sum += f;
					contador++;
					xSemaphoreGive( sem_medir_fuerza );
				}
			}

			// Delay periódico
			vTaskDelayUntil( &xLastWakeTime , xPeriodicity );

		}

}

// Tarea que setea el OFFSET
void tarea_tarar( void* taskParmPtr )
{
    // ---------- CONFIGURACIONES ------------------------------
	TickType_t xPeriodicity =  500 / portTICK_RATE_MS;		// Tarea periodica cada 1000 ms
	TickType_t xLastWakeTime = xTaskGetTickCount();

	TickType_t tiempo_diff = 40/ portTICK_RATE_MS;
	// Crear tarea en freeRTOS
		BaseType_t res =
		xTaskCreate(
			tarea_medir,                     	// Funcion de la tarea a ejecutar
			( const char * )"tarea_medir",   	// Nombre de la tarea como String amigable para el usuario
			configMINIMAL_STACK_SIZE*2, 	// Cantidad de stack de la tarea
			&tiempo_diff,                	// Parametros de tarea
			tskIDLE_PRIORITY+2,         	// Prioridad de la tarea
			&TaskHandle_medir                          	// Puntero a la tarea creada en el sistema
		);

		if(res == pdFAIL)
		{
			//error
		}
	unsigned long offset;
    // ---------- REPETIR POR SIEMPRE --------------------------
	while ( 1 ){

			gpioWrite( LED2 , 1 );
			vTaskDelay( 40 / portTICK_RATE_MS );
			gpioWrite( LED2 , 0 );



			if(xQueueReceive(cola_tarar , &offset,  portMAX_DELAY)){			// Esperamos tecla
				OFFSET = offset;
				debugPrintString( "Offset: " );
				debugPrintlnUInt(offset);
				tarado = true;
				gpioWrite( LED2, ON );
				vTaskDelete(NULL);
			}
			else{
				debugPrintString( "Offset no paso" );
			}

			// Delay periódico
			vTaskDelayUntil( &xLastWakeTime , xPeriodicity );

		}

}

/*==================[fin del archivo]========================================*/
