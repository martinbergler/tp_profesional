/*==================[inclusions]=============================================*/
#include "FreeRTOS.h"
#include "task.h"
#include "sapi.h"
#include "FreeRTOSConfig.h"

/*==================[macros and definitions]=================================*/

#define DataPin GPIO8
#define ClockPin GPIO7
#define SCALE		17000
#define GainIterationNumber 2

/*==================[internal data declaration]==============================*/

volatile unsigned long OFFSSET = 0;
int Times = 5;

/*==================[internal functions declaration]=========================*/

void HX711Config(void);
void HX711Tare(int times);
void HX711WaitReady(void);
unsigned long HX711ReadValue(void);
unsigned long HX711GetAverage(int times);
double HX711GetForce(int times);

/*==================[internal data definition]===============================*/

/*==================[external data definition]===============================*/

/*==================[internal functions definition]==========================*/

/*==================[external functions definition]==========================*/

/* ******************************************************************************************************************************** */


void HX711Config(void)
{
	// Se setea el Clock en GPIO7 y el Data en GPIO8
	gpioConfig(ClockPin,GPIO_OUTPUT);
	gpioConfig(DataPin,GPIO_INPUT);
	//gpioWrite(DataPin,1);
	//HX711Tare(Times);

}

void HX711Tare(int times) {
	double sum = HX711GetAverage(times);
	OFFSSET = sum;
}

void HX711WaitReady(void) {
	// Wait for the chip to become ready.
	// This is a blocking implementation and will
	// halt the sketch until a load cell is connected.
	while (gpioRead(DataPin)) {
		vTaskDelay( 1 / portTICK_RATE_MS );
	}
}



unsigned long HX711ReadValue(void)
{
	unsigned long Count;
	unsigned char i;

	gpioWrite(DataPin,1);
	gpioWrite(ClockPin,0);
	Count=0;

	HX711WaitReady();

	for (i=0;i<GainIterationNumber;i++)
	{
		gpioWrite(ClockPin,1);
		Count=Count<<1;
		gpioWrite(ClockPin,0);
		if(gpioRead(DataPin)){
			Count++;
		}
	}
	gpioWrite(ClockPin,1);
	Count=Count^0x800000;
	gpioWrite(ClockPin,0);
	return Count;
}


unsigned long HX711GetAverage(int times) {
	unsigned long sum = 0;
	for (int i = 0; i < times; i++) {
		sum += HX711ReadValue();
	}
	return sum / times;
}

double HX711GetForce(int times) {

	double aux = HX711GetAverage(times) - OFFSSET;
	return aux / SCALE;
}
