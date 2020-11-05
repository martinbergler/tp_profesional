#include "tasks_force.h"

DEBUG_PRINT_ENABLE;

//Handle de la cola
QueueHandle_t queue_measurement;
QueueHandle_t queue_tare;
QueueHandle_t queue_force;
SemaphoreHandle_t sem_measurement;
//SemaphoreHandle_t sem_wait;

/*==================[definiciones de datos internos]=========================*/

volatile unsigned long OFFSET = 0;
bool tarado = false;
int new_measurement = 0;
/*
typedef enum
{
    STAGE_TARE,
    STAGE_WEIGHT,
    STAGE_FORCE,
    STAGE_PRESSURE
} platformStage_t;

struct platformState{
	platformStage_t pStage;	//etapa de medicion (tara:0, peso:1, fuerza:2, presion:3)
	unsigned long offset;	//valor de tara
	int measurementsNumber;	//cantidad de mediciones realizadas en cada instante
	bool UARTSent;	//indica si se envio por UART o no lo que se deseaba
	unsigned long force;
	TickType_t timePassed;
} platformState_t;

struct platformMeasurements{
	float weight;
	float takeoffSpeed;
	float jumpTime;
	float jumpHeight;
	float jumpPower;
	float rightPressure[24][14]; //memset(arr, 0, sizeof arr);
	float leftPressure[24][14]; //memset(arr, 0, sizeof arr);
} platformMeasurements_t;*/

// Handles de las tareas
TaskHandle_t TaskHandle_wait;
TaskHandle_t TaskHandle_measurement;
TaskHandle_t TaskHandle_average;

/*==================[definiciones de funciones internas]=====================*/
// Tarea que mide la fuerza
void task_measurement( void* taskParmPtr )
{
	// ---------- CONFIGURACIONES ------------------------------
		TickType_t xPeriodicity =  TASK_RATE_500;		// Tarea periodica cada 500 ms
		TickType_t xLastWakeTime = xTaskGetTickCount();
		TickType_t dif = *( (TickType_t*)  taskParmPtr );

		// Inicialización de parámetros
		unsigned char i;
		unsigned long Count = 0;
		char str_aux[50] = {};

		//gpioWrite(DataPin,1);
		gpioWrite(ClockPin , OFF);

		//create_task(task_wait,"task_wait",SIZE,0,PRIORITY+1,&TaskHandle_wait);
		//create_task(task_average,"task_average",SIZE,0,PRIORITY+1,NULL);

	    // ---------- REPETIR POR SIEMPRE --------------------------
		while ( TRUE ){
			//sprintf(str_aux, "NEW_MEAS: %d \r\n", new_measurement);
			//uartWriteString(UART,str_aux);
			if (new_measurement == 0){
				vTaskResume(TaskHandle_wait);
				vTaskResume(TaskHandle_average);
				new_measurement = 1;
			}
			//Resumo las tareas de espera y promedio de fuerza


			// Esperar a que el módulo HX711 esté listo
			if (xSemaphoreTake( sem_measurement  ,  portMAX_DELAY )){

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

			xQueueSend(queue_measurement , &Count,  portMAX_DELAY);

			}

			// Delay periódico
			vTaskDelayUntil( &xLastWakeTime , xPeriodicity );
		}

}

// Tarea que espera a que el HX711 este listo para medir
void task_wait( void* taskParmPtr )
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
			xSemaphoreGive( sem_measurement );
			//vTaskDelete(NULL);
			vTaskSuspend(NULL);
		}

		// Delay periódico
		vTaskDelayUntil( &xLastWakeTime , xPeriodicity );

	}

}

// Tarea que promedia los valores medidos
void task_average( void* taskParmPtr )
{
    // ---------- CONFIGURACIONES ------------------------------
	TickType_t xPeriodicity =  TASK_RATE_500;		// Tarea periodica cada 500 ms
	TickType_t xLastWakeTime = xTaskGetTickCount();

	unsigned long f;
	unsigned long sum = 0;
	int contador = 0;
	unsigned long prom;
	char str_aux[50] = {};
    // ---------- REPETIR POR SIEMPRE --------------------------
	while ( TRUE ){

			gpioWrite( LEDB , ON );
			vTaskDelay( 40 / portTICK_RATE_MS );
			gpioWrite( LEDB , OFF );

			if(xQueueReceive(queue_measurement , &f,  portMAX_DELAY)){
				if (contador == CANT_MEDICIONES){
					prom = sum / CANT_MEDICIONES;
					contador = 0;
					sum = 0;
					if (!tarado){
						xQueueSend(queue_tare , &prom,  portMAX_DELAY);
					}
					else{
						xQueueSend(queue_force , &prom,  portMAX_DELAY);
					}
					//vTaskDelete(TaskHandle_measurement);
					//vTaskDelete(NULL);
					vTaskSuspend(TaskHandle_measurement);
					vTaskSuspend(NULL);
				}
				else{
					//sprintf(str_aux, "SUM: %d, CONT: %d \r\n", sum, contador);
					//uartWriteString(UART,str_aux);
					sum += f;
					contador++;
					xSemaphoreGive( sem_measurement );
				}
			}

			// Delay periódico
			vTaskDelayUntil( &xLastWakeTime , xPeriodicity );

		}

}

// Tarea que setea el OFFSET
void task_tare( void* taskParmPtr )
{
    // ---------- CONFIGURACIONES ------------------------------
	TickType_t xPeriodicity =  TASK_RATE_500;		// Tarea periodica cada 500 ms
	TickType_t xLastWakeTime = xTaskGetTickCount();

	TickType_t tiempo_diff = 40 / portTICK_RATE_MS;

	//Creo las tareas de fuerza
	create_task(task_measurement,"task_measurement",SIZE,&tiempo_diff,PRIORITY+1,&TaskHandle_measurement);
	create_task(task_wait,"task_wait",SIZE,0,PRIORITY+1,&TaskHandle_wait);
	create_task(task_average,"task_average",SIZE,0,PRIORITY+1,&TaskHandle_average);

	//Suspendo las tareas que no quiero que funcionen primero
	vTaskSuspend(TaskHandle_wait);
	vTaskSuspend(TaskHandle_average);

	unsigned long offset;
	char str_aux[50] = {};
    // ---------- REPETIR POR SIEMPRE --------------------------
	while ( TRUE ){

			gpioWrite( LED2 , ON );
			vTaskDelay( 40 / portTICK_RATE_MS );
			gpioWrite( LED2 , OFF );

			if(xQueueReceive(queue_tare , &offset,  portMAX_DELAY)){			// Esperamos tecla
				OFFSET = offset;
				sprintf(str_aux, "Offset: %lu \r\n", offset);
				uartWriteString(UART,str_aux);
				tarado = true;
				gpioWrite( LED2, ON );
				vTaskDelay( 40 / portTICK_RATE_MS );
				gpioWrite( LED2 , OFF );
				vTaskDelete(NULL);
			}

			// Delay periódico
			vTaskDelayUntil( &xLastWakeTime , xPeriodicity );

		}

}

/*==================[fin del archivo]========================================*/
