#include "tasks_force.h"

DEBUG_PRINT_ENABLE;

//Handle de la cola
QueueHandle_t cola_datos_calculados;
QueueHandle_t cola_tarar;
QueueHandle_t cola_fuerza;
SemaphoreHandle_t sem_medir_fuerza;

volatile unsigned long OFFSET = 0;
bool tarado = false;

// Handles de las tareas
TaskHandle_t TaskHandle_esperar;
TaskHandle_t TaskHandle_medir;

/*==================[definiciones de funciones internas]=====================*/
// Tarea que mide la fuerza
void tarea_medir( void* taskParmPtr )
{
	// ---------- CONFIGURACIONES ------------------------------
		TickType_t xPeriodicity =  TASK_RATE_500;		// Tarea periodica cada 500 ms
		TickType_t xLastWakeTime = xTaskGetTickCount();
		TickType_t dif = *( (TickType_t*)  taskParmPtr );

		// Inicialización de parámetros
		unsigned char i;
		unsigned long Count = 0;

		//gpioWrite(DataPin,1);
		gpioWrite(ClockPin , OFF);

		tarea_crear(tarea_esperar,"tarea_esperar",SIZE,0,PRIORITY+1,&TaskHandle_esperar);
		tarea_crear(tarea_promediar,"tarea_promediar",SIZE,0,PRIORITY+1,NULL);

/*
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
				gpioWrite( LED_ERROR , ON );
				printf( MSG_ERROR_TASK );
				while(TRUE);
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
				gpioWrite( LED_ERROR , ON );
				printf( MSG_ERROR_TASK );
				while(TRUE);
			}*/

	    // ---------- REPETIR POR SIEMPRE --------------------------
		while ( TRUE ){

			// Esperar a que el módulo HX711 esté listo
			if (xSemaphoreTake( sem_medir_fuerza  ,  portMAX_DELAY )){

				gpioWrite( LEDG , ON );
				vTaskDelay( dif );
				gpioWrite( LEDG , OFF );

				Count = 0;

				// Medir 24 pulsos
				for (i = 0;i < GAIN_128;i++)
					{
						gpioWrite(ClockPin , ON);
						Count=Count<<1;
						gpioWrite(ClockPin , OFF);
						if(gpioRead(DataPin)){

							Count++;
						}
					}
				// Hacer medición final
				gpioWrite(ClockPin , ON);
				Count=Count^XOR_VALUE;
				gpioWrite(ClockPin , OFF);

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
	TickType_t xPeriodicity =  TASK_RATE_50;		// Tarea periodica cada 50 ms
	TickType_t xLastWakeTime = xTaskGetTickCount();

    // ---------- REPETIR POR SIEMPRE --------------------------
	while ( TRUE ){

			gpioWrite( LEDR , ON );
			vTaskDelay( 40 / portTICK_RATE_MS );
			gpioWrite( LEDR , OFF );

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
	TickType_t xPeriodicity =  TASK_RATE_500;		// Tarea periodica cada 500 ms
	TickType_t xLastWakeTime = xTaskGetTickCount();

	unsigned long f;
	unsigned long sum = 0;
	int contador = 0;
	unsigned long prom;
    // ---------- REPETIR POR SIEMPRE --------------------------
	while ( TRUE ){

			gpioWrite( LEDB , ON );
			vTaskDelay( 40 / portTICK_RATE_MS );
			gpioWrite( LEDB , OFF );

			if(xQueueReceive(cola_datos_calculados , &f,  portMAX_DELAY)){			// Esperamos tecla
				//debugPrintString( "Espacio en cola: " );
				//debugPrintlnUInt(uxQueueSpacesAvailable(cola_datos_calculados));
				if (contador >= CANT_MEDICIONES){
					prom = sum / CANT_MEDICIONES;
					if (!tarado){
						debugPrintlnUInt(prom);
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
	TickType_t xPeriodicity =  TASK_RATE_500;		// Tarea periodica cada 500 ms
	TickType_t xLastWakeTime = xTaskGetTickCount();

	TickType_t tiempo_diff = 40 / portTICK_RATE_MS;

	tarea_crear(tarea_medir,"tarea_medir",SIZE,&tiempo_diff,PRIORITY+1,&TaskHandle_medir);
/*
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
			gpioWrite( LED_ERROR , ON );
			printf( MSG_ERROR_TASK );
			while(TRUE);
		}*/

	unsigned long offset;
	char str_aux[50] = {};
    // ---------- REPETIR POR SIEMPRE --------------------------
	while ( TRUE ){

			gpioWrite( LED2 , ON );
			vTaskDelay( 40 / portTICK_RATE_MS );
			gpioWrite( LED2 , OFF );

			if(xQueueReceive(cola_tarar , &offset,  portMAX_DELAY)){			// Esperamos tecla
				OFFSET = offset;
				sprintf(str_aux, "Offset: %lu \r\n", offset);
				uartWriteString(UART,str_aux);
				//debugPrintString( "Offset: " );
				//debugPrintlnUInt(offset);
				tarado = true;
				gpioWrite( LED2, ON );
				vTaskDelay( 40 / portTICK_RATE_MS );
				gpioWrite( LED2 , OFF );
				vTaskDelete(NULL);
			}
			else{
				//debugPrintString( "Offset no paso" );
				uartWriteString(UART,"Offset no pasó \r\n");
			}

			// Delay periódico
			vTaskDelayUntil( &xLastWakeTime , xPeriodicity );

		}

}

/*==================[fin del archivo]========================================*/
