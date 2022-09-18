/*
 * 01_led_toggle.c
 *
 *
 *
 */

#include "stm32f4xx.h"
#include <stdio.h>

void delay(void)
{
	for(uint32_t i = 0; i < 500000; i++);		//killing some time
}

int main(void)
{
	GPIO_Handle_t GpioLed;	//creating a new variable for the structure to access and give definitions

	memset((void *)&GpioLed, 0, (uint16_t) sizeof(GpioLed));

	//ctrl+space to get recommendations of definitions and define them all
	GpioLed.pGPIOx = GPIOC;										/* LD4 green LED at PC13 */
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_13;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OTYPE_PP;		// using push-pull
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PUPD_NO;	//no pupd needed for push-pull

	GPIO_PeriClockControl(GPIOC, ENABLE);	//enable peripheral clock
	GPIO_Init(&GpioLed);					//call API and send address to run them

	GPIO_WriteToOutputPin(GPIOC, GPIO_PIN_13, ENABLE);

	while(1)
	{
		GPIO_ToggleOutputPin(GPIOC, GPIO_PIN_13);
		delay();

	}
	return 0;
}
