/* Copyright 2016, Eric Pernia.
 * All rights reserved.
 *
 * This file is part sAPI library for microcontrollers.
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
 *
 */

/*
 * Date: 2016-04-26
 */



/*==================[inclusions]=============================================*/

#include "main.h"
#include "sapi.h"       // <= sAPI header
#include <stdio.h>		//header file standard input / output functions
#include <string.h>   	// <= Biblioteca de manejo de Strings


DEBUG_PRINT_ENABLE;


/*==================[macros and definitions]=================================*/

// Macros de las UART de bluetooth y WiFi
#define UART_BLUETOOTH 	UART_232
#define UART_WIFI  		UART_GPIO

// Se definen las funciones para setear como salida, entrada, leer, poner en cero
// y poner en uno el pin correspondiente
#define owOUT(port,pin)		Chip_GPIO_SetPinDIROutput(LPC_GPIO_PORT, port, pin)
#define owIN(port,pin)		Chip_GPIO_SetPinDIRInput(LPC_GPIO_PORT, port, pin)
#define owREAD(port,pin)	Chip_GPIO_GetPinState(LPC_GPIO_PORT, port, pin)
#define owLOW(port,pin)		Chip_GPIO_SetPinOutLow(LPC_GPIO_PORT, port, pin)
#define owHIGH(port,pin)	Chip_GPIO_SetPinOutHigh(LPC_GPIO_PORT, port, pin)

// Se setean los pines para el módulo HX711
// GPIO8 para el pin DT:
#define dataport	2
#define datapin		8
// GPIO7 para SCK:
#define clockport	3
#define clockpin	7

// Factor de calibración del módulo HX711
#define SCALE		18400

/*==================[internal data declaration]==============================*/

// Offset de la balanza, que se utilizará para tararla
volatile unsigned long OFFSET = 0;

// Se define la frecuencia del Clock
static uint32_t ClockSpeed = EDU_CIAA_NXP_CLOCK_SPEED;

/*==================[internal functions declaration]=========================*/

/*==================[internal data definition]===============================*/

/*==================[external data definition]===============================*/

/*==================[internal functions definition]==========================*/

/*==================[external functions definition]==========================*/

/*-------------------------------HX711 Pesaje--------------------------------*/


// Se inicializa la conexión con el módulo HX711
void owInit(void)
{
	// Se setea el pin GPIO8 para DT como entrada
    Chip_SCU_PinMux(dataport,datapin,SCU_MODE_INACT  | SCU_MODE_ZIF_DIS, SCU_MODE_FUNC0 );
    owIN(dataport,datapin);

    // Se setea el pin GPIO7 para SCK como salida
    Chip_SCU_PinMux(clockport,clockpin,SCU_MODE_INACT  | SCU_MODE_ZIF_DIS, SCU_MODE_FUNC0 );
    owOUT(clockport,clockpin);
}

// Espera a que el módulo HX711 esté listo.
void wait_ready(void) {
	// Esta es una implementación bloqueante y detendrá el proceso hasta que una celda
	// de carga esté conectada.
	// Cuando DT = 0, el módulo HX711 está listo para enviar información
	while (owREAD(dataport,datapin)) {
	}
}


// Lee la información del módulo HX711
unsigned long read_count(void)
{
	unsigned long Count;
	unsigned char i;

	// DT = 1, SCK = 0
	owHIGH(dataport,datapin);
	owLOW(clockport,clockpin);
	Count=0;

	// Se espera a que el módulo esté listo para enviar información
	wait_ready();

	// Se envian 24 pulsos al clock SCK y se va leyendo el pin DT
	for (i=0;i<24;i++)
	{
		owHIGH(clockport,clockpin);
		Count=Count<<1;
		owLOW(clockport,clockpin);
		if(owREAD(dataport,datapin)){
			Count++;
		}
	}

	// El pulso 25 define que la ganancia sea 128 e implica el final de la medición
	owHIGH(clockport,clockpin);
	Count=Count^0x800000;
	owLOW(clockport,clockpin);

	// Se devuelve el valor de la medición
	return(Count);
}

// Se hace un promedio para que el valor sea el más exacto posible
unsigned long read_average(int times) {
	unsigned long sum = 0;
	for (int i = 0; i < times; i++) {
		sum += read_count();
	}
	return sum/times;
}

// Se toma el valor promediado y se le resta el OFFSET
unsigned long get_value(int times) {
	return read_average(times) - OFFSET;

}

// Una vez restado el OFFSET, se divide por el valor de calibración
unsigned long get_units(int times) {
	unsigned long scale = get_value(times) / SCALE;

	// La lectura a veces da un valor muy grande cuando se está en cero, así que
	// se setea un valor máximo y si da mayor a él, se supone que se midió cero
	if (scale >=300){
		scale = 0;
	}
	return scale;
}

// Se tara la balanza y se define el valor de offset
void tare(int times) {
	double sum = read_average(times);
	set_offset(sum);
}

// Se guarda el valor de offset en la variable OFFSET
void set_offset(double offset) {
	OFFSET = offset;
}

/*-------------------------------------------------------------*/


// FUNCIÓN PRINCIPAL
int main(void){

	// Inicializaciones
	boardConfig();
	gpioConfig(GPIO7,GPIO_INPUT);
	gpioConfig(GPIO8,GPIO_OUTPUT);
	SystemCoreClockUpdate();
	cyclesCounterConfig(ClockSpeed);
	owInit();
	tare(10);

	/*debugPrintConfigUart(UART_USB, 115200 );
	debugPrintString( "DEBUG c/sAPI\r\n" );*/

	// Inicializar UART_232 para conectar al módulo bluetooth y UART_GPIO para el módulo WIFI
	uartConfig( UART_BLUETOOTH, 9600 );
	uartConfig( UART_WIFI, 115200 );

	// Inicialización de variables
	int state_flag = 0;			// Variable de switch
	bool_t saltando = false;	// Indica si la persona está saltando o no

	// Valores que se van a medir/calcular y mostrar por bluetooth y WiFi
	unsigned long peso = 0;
	float tiempo = 0;
	float altura = 0;
	float velocidad = 0;
	float potencia = 0;

	// Variables del bluetooth
	char bt_data = 0;
	bool_t bt_recibido = false;
	char bt_send[50] = {};

	// Variables del WiFi
	char wifi_send[100] = {};

		// Variables auxiliares
	float peso_aux = 0;			// Se compara con el peso de la persona
	uint32_t Counter = 0;		// Se guardan la cantidad de ciclos pasados
	bool_t peso_state = true;	// Se coloca en false cuando ya se pesó a la persona
	float g = 9.81;				// Gravedad en m/s
	float tiempo_s = 0;			// Tiempo en segundos


	// Loop
	while(1){
		saltando = false;
		SystemCoreClockUpdate();

		switch(state_flag){
		// Se recibe un byte de bluetooth para saber cual es el siguiente paso
		case 0:
			delay(1000);
			bt_data = 0;
			bt_recibido = false;
			while(!bt_recibido){
				bt_recibido = uartReadByte(UART_BLUETOOTH, &bt_data);
			}
			// Si se recibió un 1 y todavía no se pesó a la persona, hay que pesarla
			if (bt_data == 1 && peso_state == true){
				state_flag = 1;
			}
			// Si se recibió un 2, hay que medir el salto
			else if (bt_data == 2){
				state_flag = 2;
				}
			break;

		// Se pesa al usuario
		case 1:
			// Se pesa 4 veces inicialmente para que la medición sea lo más correcta posible
			/*for (int i = 0; i < 4; i++){
				get_units(5);
			}
			// Se pesa a la persona y se guarda el valor en la variable peso
			delay(1000);
			peso = get_units(10);
			delay(1000);*/
			peso = 67;
			sprintf(bt_send, "OK|%u|",peso);
			while (bt_recibido = uartReadByte(UART_BLUETOOTH, &bt_data) && bt_data == 1){
				uartWriteString(UART_BLUETOOTH, bt_send);
				delay(100);
			}
			// Se imprime la variable peso en la aplicación, a través del bluetooth
			debugPrintString(bt_send);
			// Se vuelve a esperar recibir un byte de bluetooth
			state_flag = 0;
			peso_state = false;
			break;


		// Medición del salto y los parámetros a partir de él
		case 2:
			delay(500);
			peso_aux = peso;
			tiempo = 0;

			/*while(saltando == false){
				// Se corrobora que el peso medido sea menor o igual al peso del usuario menos cinco
				if (peso_aux >= (peso-5)){
					peso_aux = get_units(1);
				}
				else{
					// Cuando comienza a saltar, se resetea el contador de ciclos
					saltando = true;
					cyclesCounterReset();
					// Se mide el peso hasta que este sea igual o mayor al peso menos 5 (es decir, ya cayó)
					while(peso_aux <= (peso-5)){
						peso_aux = get_units(1);
					}
					// Se cuentan los ciclos transcurridos en el salto
					Counter = cyclesCounterRead();
				}
			}
			// Se pasan los ciclos contados a ms y se calculan el tiempo, la altura, la velocidad y la potencia
			tiempo=cyclesCounterToMs(Counter);*/
			tiempo = 621.32;
			tiempo_s = tiempo/1000;
			altura = ((g * tiempo_s * tiempo_s)/8)*100;
			velocidad = (g * tiempo_s) / 2;
			potencia = peso * velocidad;

			// Se envian los parámetros calculados a bluetooth
			sprintf(bt_send, "OK2|%2.2f|%2.2f|%2.2f|%2.2f|",tiempo,altura,velocidad,potencia);
			while (bt_recibido = uartReadByte(UART_BLUETOOTH, &bt_data) && bt_data == 2){
				uartWriteString(UART_BLUETOOTH, bt_send);
				delay(100);
			}
			debugPrintString(bt_send);
			state_flag = 3;
			break;
		case 3:
			// Se envían los parámetros calculados a WiFi
			sprintf(wifi_send, "{\"val\":\"%2.2f|%2.2f|%2.2f|%2.2f|%2.2f\"}\n",peso,tiempo,altura,velocidad,potencia);
			uartWriteString(UART_WIFI, wifi_send);
			state_flag = 0;
			peso_state = true;
			break;
		default:
			break;
		}
	}

}

/* ******************************************************************************************************************************** */
