/* Copyright 2018, Eric Pernia.
 * All rights reserved.
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

#include "sapi.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

typedef enum
{
    STATE_BUTTON_UP,
    STATE_BUTTON_DOWN,
    STATE_BUTTON_FALLING,
    STATE_BUTTON_RISING
} fsmButtonState_t;

void fsmButtonError( void );
void fsmButtonInit( void );
void fsmButtonUpdate( gpioMap_t tecla );
void buttonPressed( void );
void buttonReleased( void );

fsmButtonState_t fsmButtonState;

TickType_t tiempo_down;
TickType_t tiempo_up;
TickType_t tiempo_diff;

/* prototipo de la tarea led   */
void tarea_medir( void* taskParmPtr );

// Handles de las tareas
TaskHandle_t TaskHandle_medir;
//TaskHandle_t TaskHandle_medicion;

TickType_t get_diff()
{
	return tiempo_diff;
}

void clear_diff()
{
	tiempo_diff = 0;
}


/* accion de el evento de tecla pulsada */
void buttonPressed( void )
{
	tiempo_down = xTaskGetTickCount();
}

/* accion de el evento de tecla liberada */
void buttonReleased( void )
{
	tiempo_up = xTaskGetTickCount();
	tiempo_diff = tiempo_up - tiempo_down;

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

}

void fsmButtonError( void )
{
    fsmButtonState = BUTTON_UP;
}

void fsmButtonInit( void )
{
    fsmButtonState = BUTTON_UP;  // Set initial state
}

#define DEBOUNCE_TIME 40

// FSM Update Sate Function
void fsmButtonUpdate( gpioMap_t tecla)
{
   // static bool_t flagFalling = FALSE;
    static bool_t flagRising = FALSE;

    static uint8_t contFalling = 0;
    static uint8_t contRising = 0;

    switch( fsmButtonState )
    {
        case STATE_BUTTON_UP:
            /* CHECK TRANSITION CONDITIONS */
            if( !gpioRead( tecla ) )
            {
                fsmButtonState = STATE_BUTTON_FALLING;
            }
            break;

        case STATE_BUTTON_FALLING:
            /* ENTRY */

            /* CHECK TRANSITION CONDITIONS */
            if( contFalling >= DEBOUNCE_TIME )
            {
                if( !gpioRead( tecla ) )
                {
                    fsmButtonState = STATE_BUTTON_DOWN;

                    /* ACCION DEL EVENTO !*/
                    buttonPressed();
                }
                else
                {
                    fsmButtonState = STATE_BUTTON_UP;
                }

                contFalling = 0;
            }

            contFalling++;

            /* LEAVE */
            break;

        case STATE_BUTTON_DOWN:
			/* CHECK TRANSITION CONDITIONS */
			if( gpioRead( tecla ) )
			{
				fsmButtonState = STATE_BUTTON_RISING;
			}
			break;

        case STATE_BUTTON_RISING:
            /* ENTRY */

            /* CHECK TRANSITION CONDITIONS */

            if( contRising >= DEBOUNCE_TIME )
            {
                if( gpioRead( tecla ) )
                {
                    fsmButtonState = STATE_BUTTON_UP;

                    /* ACCION DEL EVENTO ! */
                    buttonReleased();
                }
                else
                {
                    fsmButtonState = STATE_BUTTON_DOWN;
                }
                contRising = 0;
            }
            contRising++;

            /* LEAVE */
            break;

        default:
            fsmButtonError();
            break;
    }
}
