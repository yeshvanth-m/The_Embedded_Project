/*
 * 02_button_interrupt.c
 *
 *  Created on: Sep 18, 2022
 *      Author: yeshv
 */

#include "stm32f4xx.h"
#include "string.h"

void delay(void)
{
	for(uint32_t i = 0; i < 500000; i++);		//killing some time
}

int main(void)
{
	GPIO_Handle_t GpioLed, GpioButton;	//creating a new variable for the structure to access and give definitions

	memset(&GpioLed,0,sizeof(GpioLed));
	memset(&GpioButton,0,sizeof(GpioButton));

	/* Initialize GPIO LED in PC13 */
	//ctrl+space to get recommendations of definitions and define them all
	GpioLed.pGPIOx = GPIOC;										/* LD4 green LED at PC13 */
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_13;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OTYPE_PP;		// using push-pull
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PUPD_NO;	//no pupd needed for push-pull

	GPIO_PeriClockControl(GPIOC, ENABLE);	//enable peripheral clock
	GPIO_Init(&GpioLed);					//call API and send address to run them

	/* Clear the LED GPIO bit */
	GPIO_WriteToOutputPin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);

	/* Initialize GPIO Button in PA0 */
	//ctrl+space to get recommendations of definitions and define them all
	GpioButton.pGPIOx = GPIOA;										/* LD4 green LED at PC13 */
	GpioButton.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_0;
	GpioButton.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_RFT;
	GpioButton.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
	GpioButton.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PUPD_PU;	//Pull-up for button

	GPIO_PeriClockControl(GPIOA, ENABLE);	//enable peripheral clock
	GPIO_Init(&GpioButton);					//call API and send address to run them

	GPIO_IRQPriorityConfig(IRQ_NO_EXTI0, NVIC_IRQ_PRI_15);
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI0, ENABLE);

	while(1);

}

void EXTI0_IRQHandler(void)
{
	delay();
	GPIO_IRQHandling(GPIO_PIN_0);
	GPIO_ToggleOutputPin(GPIOC, GPIO_PIN_13);
}
