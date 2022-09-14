/*
 * stm32f4xx_gpio_driver.h
 *
 *
 *
 */

#ifndef INC_STM32F4XX_GPIO_DRIVER_H_
#define INC_STM32F4XX_GPIO_DRIVER_H_
#include "stm32f4xx.h"

/* configuration structure for GPIO */
typedef struct
{									//pin number is 15 at max and so uint8_t i.e., 1 byte is suffice
	uint8_t GPIO_PinNumber;			/*< possible values from @GPIO_pin_numbers >*/
	uint8_t	GPIO_PinMode;			/*< possible values from @GPIO_pin_modes >*/
	uint8_t GPIO_PinSpeed;			/*< possible values from @GPIO_pin_speeds >*/
	uint8_t GPIO_PinPuPdControl;	/*< possible values from @GPIO_pin_pupds >*/
	uint8_t GPIO_PinOPType;			/*< possible values from @GPIO_pin_types >*/
	uint8_t GPIO_PinAltFunMode;
}GPIO_PinConfig_t;

/* handle structure for GPIO pin */
typedef struct
{
	//pointer to hold the base address of GPIO peripheral
	GPIO_RegDef_t *pGPIOx;	//holds base address of the GPIO to which the pin belongs
	GPIO_PinConfig_t GPIO_PinConfig;	//holds GPIO pin configuration settings

}GPIO_Handle_t;

/*
 * Expanding GPIO_PinConfig_t configuration structure
 * Macros for GPIO_Init(GPIO_Handle_t *pGPIOHandle) in gpio_driver.c file
 * Defining all the individual registers and their definitions for GPIOx
 */
#define GPIO_PIN_0			0
#define GPIO_PIN_1			1
#define GPIO_PIN_2			2
#define GPIO_PIN_3			3
#define GPIO_PIN_4			4
#define GPIO_PIN_5			5
#define GPIO_PIN_6			6
#define GPIO_PIN_7			7
#define GPIO_PIN_8			8
#define GPIO_PIN_9			9
#define GPIO_PIN_10			10
#define GPIO_PIN_11			11
#define GPIO_PIN_12			12
#define GPIO_PIN_13			13
#define GPIO_PIN_14			14
#define GPIO_PIN_15			15

/*
 * GPIO_pin_modes (GPIOx_MODER)
 * |---------------------------|
 * | 00 | 01  |	 10	  |  11	   |
 * | --------------------------|
 * | IN | OUT | ALTFN | ANALOG |
 * |---------------------------|
 */
#define GPIO_MODE_IN		0	//these mode values predefined in user manual under MODER
#define GPIO_MODE_OUT		1
#define GPIO_MODE_ALTFN		2
#define GPIO_MODE_ANALOG	3
//Interrupt modes(requires additional blocks than GPIOx)
#define GPIO_MODE_IT_FT		4	//falling edge trigger interrupt
#define GPIO_MODE_IT_RT		5	//rising edge trigger interrupt
#define GPIO_MODE_IT_RFT	6

/* GPIO_pin_output_types (GPIOx_OTYPER) */
#define GPIO_OTYPE_PP		0	//open drain
#define GPIO_OTYPE_OD		1	//push-pull

/* GPIO_pin_output_speeds (GPIOx_OSPEEDR) */
#define GPIO_SPEED_LOW		0
#define GPIO_SPEED_MID		1
#define GPIO_SPEED_HIGH		2

/* GPIO_pin_pupds (GPIOx_PUPDR) */
#define GPIO_PUPD_NO		0	//no pull-up, pull-down
#define GPIO_PUPD_PU		1	//pull-up
#define GPIO_PUPD_PD		2	//pull-down

/*
 *  APIs supported by this driver
 */
//peripheral clock setup
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);

//initialization and de-initialization
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

//data read and write
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);	//it has to return value
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);	//nothing to return i.e., void
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

//interrupt configuration and handling
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNumber);

#endif /* INC_STM32F303XX_GPIO_DRIVER_H_ */
