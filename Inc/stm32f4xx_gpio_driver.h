/*
 * stm32f4xx_gpio_driver.h
 *
 */

#ifndef INC_STM32F4XX_GPIO_DRIVER_H_
#define INC_STM32F4XX_GPIO_DRIVER_H_
#include "stm32f4xx.h"

/* configuration structure for GPIO */
typedef struct
{									/* Pin number is 15 at max and so uint8_t i.e., 1 byte is suffice */
	uint8_t GPIO_PinNumber;			/*< possible values from @GPIO_PIN_NO_numbers >*/
	uint8_t	GPIO_PinMode;			/*< possible values from @GPIO_PIN_NO_modes >*/
	uint8_t GPIO_PinSpeed;			/*< possible values from @GPIO_PIN_NO_speeds >*/
	uint8_t GPIO_PinPuPdControl;	/*< possible values from @GPIO_PIN_NO_pupds >*/
	uint8_t GPIO_PinOPType;			/*< possible values from @GPIO_PIN_NO_types >*/
	uint8_t GPIO_PinAltFunMode;
} GPIO_PinConfig_t;

/* Handle structure for GPIO pin */
typedef struct
{
	/* Pointer to hold the base address of GPIO peripheral */
	GPIO_RegDef_t *pGPIOx;	/* Holds base address of the GPIO to which the pin belongs */
	GPIO_PinConfig_t GPIO_PinConfig;	/* Holds GPIO pin configuration settings */
} GPIO_Handle_t;

/*
 * Expanding GPIO_PinConfig_t configuration structure
 * Macros for GPIO_Init(GPIO_Handle_t *pGPIOHandle) in gpio_driver.c file
 * Defining all the individual registers and their definitions for GPIOx
 */
#define GPIO_PIN_NO_0		0
#define GPIO_PIN_NO_1		1
#define GPIO_PIN_NO_2		2
#define GPIO_PIN_NO_3		3
#define GPIO_PIN_NO_4		4
#define GPIO_PIN_NO_5		5
#define GPIO_PIN_NO_6		6
#define GPIO_PIN_NO_7		7
#define GPIO_PIN_NO_8		8
#define GPIO_PIN_NO_9		9
#define GPIO_PIN_NO_10		10
#define GPIO_PIN_NO_11		11
#define GPIO_PIN_NO_12		12
#define GPIO_PIN_NO_13		13
#define GPIO_PIN_NO_14		14
#define GPIO_PIN_NO_15		15

/*
 * GPIO_PIN_NO_modes (GPIOx_MODER)
 * |---------------------------|
 * | 00 | 01  |	 10	  |  11	   |
 * | --------------------------|
 * | IN | OUT | ALTFN | ANALOG |
 * |---------------------------|
 */
#define GPIO_MODE_IN		0	/* These mode values predefined in user manual under MODER */
#define GPIO_MODE_OUT		1
#define GPIO_MODE_ALTFN		2
#define GPIO_MODE_ANALOG	3
/* Interrupt modes(requires additional blocks than GPIOx) */
#define GPIO_MODE_IT_FT		4	/* falling edge trigger interrupt */
#define GPIO_MODE_IT_RT		5	/* rising edge trigger interrupt */
#define GPIO_MODE_IT_RFT	6

/* GPIO_PIN_NO_output_types (GPIOx_OTYPER) */
#define GPIO_OP_TYPE_PP		0	/* open drain */
#define GPIO_OP_TYPE_OD		1	/* push-pull */

/* GPIO_PIN_NO_output_speeds (GPIOx_OSPEEDR) */
#define GPIO_SPEED_SLOW		0
#define GPIO_SPEED_MID		1
#define GPIO_SPEED_FAST		2

/* GPIO_PIN_pupds (GPIOx_PUPDR) */
#define GPIO_NO_PUPD		0	/* no pull-up, pull-down */
#define GPIO_PUPD_PU		1	/* pull-up */
#define GPIO_PUPD_PD		2	/* pull-down */

/*
 *  APIs supported by this driver
 */
/* Peripheral Clock Setup */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);

/* Initialization and de-initialization */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

/* Data read and write */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);	/* It has to return value */
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);	/* Nothing to return i.e., void */
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

/* Interrupt configuration and handling */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNumber);

#endif /* INC_STM32F303XX_GPIO_DRIVER_H_ */
