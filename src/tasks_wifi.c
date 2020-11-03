#include "tasks_wifi.h"

/*==================[definiciones de datos externos]=========================*/
DEBUG_PRINT_ENABLE;

//Handle de las colas y semaforos
extern QueueHandle_t cola_fuerza;
extern volatile unsigned long OFFSET;
/*==================[definiciones de funciones externas]=====================*/

void tarea_Rx_WIFI( void* taskParmPtr )
{
	fsmButtonInit();

	while( TRUE )
	{
		fsmButtonUpdate( TEC1 );
	 	vTaskDelay( 1 / portTICK_RATE_MS );
	}
}

// Implementacion de funcion de la tarea
void tarea_Tx_WIFI( void* taskParmPtr )
{
	unsigned long fu = 0;
	float p = 0;
	char str_aux[50] = {};
    // ---------- CONFIGURACIONES ------------------------------
	TickType_t xPeriodicity =  TASK_RATE_1000;		// Tarea periodica cada 1000 ms
	TickType_t xLastWakeTime = xTaskGetTickCount();
	// ---------- REPETIR POR SIEMPRE --------------------------
    while( 1 )
    {
    	if(xQueueReceive(cola_fuerza , &fu,  portMAX_DELAY)){			// Esperamos tecla

			gpioWrite( LED1, ON );
			vTaskDelay(40 / portTICK_RATE_MS);
			gpioWrite( LED1, OFF );

			p = (fu - OFFSET) / SCALE;
			//chequear aca que se guardo en p (BREAKPOINT)
			if (p < 0){
				debugPrintString("Negativo corregido");
				p = 0;
			}

			debugPrintlnUInt(OFFSET);
			debugPrintString( "Fuerza: " );
			debugPrintlnUInt(fu);
			sprintf(str_aux, "%2.2f",p);	//PROBAR
			debugPrintString( "Peso: " );
			debugPrintString( str_aux );
			debugPrintEnter();
			debugPrintlnInt(p); //251020

			// Delay periódico
			vTaskDelayUntil( &xLastWakeTime , xPeriodicity );
    	}

    }
}
/*
// Tarea que mide la fuerza
void tarea_medir( void* taskParmPtr )
{
	// ---------- CONFIGURACIONES ------------------------------
		TickType_t xPeriodicity =  TASK_RATE_500;		// Tarea periodica cada 500 ms
		TickType_t xLastWakeTime = xTaskGetTickCount();
		TickType_t dif = *( (TickType_t*)  taskParmPtr );

		// Inicialización de parámetros
		unsigned char i;

		//gpioWrite(DataPin,1);
		gpioWrite(ClockPin , OFF);

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

			if(res == pdFAIL){
				gpioWrite( LED2 , ON);
				debugPrintlnString( "Error: creación tarea_esperar" );
				while(TRUE){
				}
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

			if(res == pdFAIL){
				gpioWrite( LED2 , ON);
				debugPrintlnString( "Error: creación tarea_promediar" );
				while(TRUE){
				}
			}

	    // ---------- REPETIR POR SIEMPRE --------------------------
		while ( 1 ){

			// Esperar a que el módulo HX711 esté listo
			if (xSemaphoreTake( sem_medir_fuerza  ,  portMAX_DELAY )){

				gpioWrite( LEDG , ON ); //PROBAR ON/OFF
				vTaskDelay( dif );
				gpioWrite( LEDG , OFF );

				Count = 0;

				// Medir 24 pulsos (NO HARDCODEAR - gainX3)
				for (i = 0 ; i < GAIN_128; i++)
					{
						gpioWrite(ClockPin , ON); //SET_ON=1 SET_OFF =0
						Count=Count<<1;
						gpioWrite(ClockPin , OFF);
						if(gpioRead(DataPin)){
							Count++;
						}
					}
				// Hacer medición final
				gpioWrite(ClockPin , ON);
				Count=Count^XOR_VALUE;	//NO HARDCODEAR
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
	while ( 1 ){

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
	//CANT PROMEDIADA ETIQUETA/MACRO
	unsigned long f;
	unsigned long sum = 0;
	int contador = 0;
	unsigned long prom;
    // ---------- REPETIR POR SIEMPRE --------------------------
	while ( 1 ){

			gpioWrite( LEDB , ON );
			vTaskDelay( 40 / portTICK_RATE_MS );
			gpioWrite( LEDB , OFF );

			if(xQueueReceive(cola_datos_calculados , &f,  portMAX_DELAY)){			// Esperamos tecla
				//debugPrintString( "Espacio en cola: " );
				//debugPrintlnUInt(uxQueueSpacesAvailable(cola_datos_calculados));
				if (contador >= CANT_MEDICIONES){
					prom = sum / CANT_MEDICIONES;
					if (!tarado){
						xQueueSend(cola_tarar , &prom,  portMAX_DELAY);
						vTaskDelete(TaskHandle_medir);	//AFUERA
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
	TickType_t xPeriodicity =  TASK_RATE_500;		// Tarea periodica cada 1000 ms
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

		if(res == pdFAIL){
			gpioWrite( LED2 , ON);
			debugPrintlnString( "Error: creación tarea_medir" );
			while(TRUE){
			}
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
