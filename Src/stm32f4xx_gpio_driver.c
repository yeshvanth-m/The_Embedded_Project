/*
 * stm32f4xx_gpio_driver.c
 *
 *
 *
 */

#include "stm32f4xx_gpio_driver.h"

/*********************************************************
 * @fn			- GPIO_PeriClockControl
 * @brief		- This function enables or disables peripheral clock for the given GPIO port
 *
 * @param[in]	- base address of the GPIO peripheral
 * @param[in]	- ENABLE/DISABLE macro
 *
 * @return		- none, its a void
 *
 * @notes		- none
 *
 */
//peripheral clock setup
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_EN();
		}else if(pGPIOx == GPIOB)
		{
			GPIOB_PCLK_EN();
		}else if(pGPIOx == GPIOC)
		{
			GPIOC_PCLK_EN();
		}else if(pGPIOx == GPIOD)
		{
			GPIOD_PCLK_EN();
		}else if(pGPIOx == GPIOE)
		{
			GPIOE_PCLK_EN();
		}else if(pGPIOx == GPIOF)
		{
			GPIOF_PCLK_EN();
		}else if(pGPIOx == GPIOG)
		{
			GPIOG_PCLK_EN();
		}else if(pGPIOx == GPIOH)
		{
			GPIOH_PCLK_EN();
		}
	}
	else
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_DI();
		}else if(pGPIOx == GPIOB)
		{
			GPIOB_PCLK_DI();
		}else if(pGPIOx == GPIOC)
		{
			GPIOC_PCLK_DI();
		}else if(pGPIOx == GPIOD)
		{
			GPIOD_PCLK_DI();
		}else if(pGPIOx == GPIOE)
		{
			GPIOE_PCLK_DI();
		}else if(pGPIOx == GPIOF)
		{
			GPIOF_PCLK_DI();
		}else if(pGPIOx == GPIOG)
		{
			GPIOG_PCLK_DI();
		}else if(pGPIOx == GPIOH)
		{
			GPIOH_PCLK_DI();
		}
	}
}

/*********************************************************
 * @fn			- GPIO_Init
 * @brief		- This function configures the features of GPIO pin
 *
 * @param[in]	-base address of the GPIO peripheral
 * @param[in]	-
 *
 * @return		- none, its a void
 *
 * @notes		- none
 *
 */
//initialization of GPIOs and functionalities
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	uint32_t temp = 0;	//temp register

	//enable peripheral clock before using this peripheral
	GPIO_PeriClockControl(pGPIOHandle ->pGPIOx, ENABLE);

	//1)configure pin mode
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		//multiplied by 2 for appropriate left shift
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber ) );
		pGPIOHandle->pGPIOx->MODER &= ~( 0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); //clearing
		pGPIOHandle->pGPIOx->MODER |= temp; //setting
	}else
	{
		/* interrupt mode */
		if(pGPIOHandle ->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT)
		{
			//1) configure  the FTSR
			EXTI->FTSR1 |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			//clear the corresponding RTSR bit
			EXTI->RTSR1 &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		}else if(pGPIOHandle ->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT)
		{
			//2) configure the RTSR
			EXTI->RTSR1 |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			//clear the corresponding RTSR bit
			EXTI->FTSR1 &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		}else if(pGPIOHandle ->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT)
		{
			//1) configure the FTSR
			EXTI->FTSR1 |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			//2) configure the RTSR
			EXTI->RTSR1 |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}

		//2) configure the GPIO port selection in SYSCFG_EXTICR
		uint8_t temp1 = pGPIOHandle ->GPIO_PinConfig.GPIO_PinNumber / 4;
		uint8_t temp2 = pGPIOHandle ->GPIO_PinConfig.GPIO_PinNumber % 4;
		uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle ->pGPIOx);

		SYSCFG_PCLK_EN();	//enable clock to activate it
		SYSCFG ->EXTICR[temp1] = portcode << (temp2 * 4);

		//3) enable the EXTI interrupt delivery using IMR
		EXTI->IMR1 |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	}

	//2)configure pin speed
	temp = (pGPIOHandle -> GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR &= ~( 0x3 << ( 2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); //clearing
	pGPIOHandle ->pGPIOx ->OSPEEDR |= temp;

	//3)configure pupd settings
	temp = (pGPIOHandle -> GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR &= ~( 0x3 << ( 2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); //clearing
	pGPIOHandle ->pGPIOx ->PUPDR |= temp;

	//4)configure optype (only 1 bit, no need to x2)
	temp = (pGPIOHandle -> GPIO_PinConfig.GPIO_PinOPType << pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER &= ~( 0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //clearing
	pGPIOHandle ->pGPIOx ->OTYPER |= temp;

	//5)configure alt  functionality
	if(pGPIOHandle -> GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
	{
		uint32_t temp1, temp2;
		temp1 = pGPIOHandle ->GPIO_PinConfig.GPIO_PinNumber / 8; //divide by 8 to check if its AFR[0] or AFR[1]
		temp2 = pGPIOHandle ->GPIO_PinConfig.GPIO_PinNumber % 8; //modulus by 8
		pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << ( 4 * temp2 ) ); //clearing
		pGPIOHandle ->pGPIOx ->AFR[temp1] |= (pGPIOHandle ->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * temp2));
	}
}

/*********************************************************
 * @fn			- GPIO_DeInit
 * @brief		- This function resets the features of GPIO pin configuring AHBRSTR register
 *
 * @param[in]	- base address of the GPIO peripheral
 * @param[in]	-
 *
 * @return		- none, its a void
 *
 * @notes		- none
 *
 */
//de-initialization of GPIOs
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
	if(pGPIOx == GPIOA)
	{
		GPIOA_REG_RESET();
	}else if(pGPIOx == GPIOB)
	{
		GPIOB_REG_RESET();
	}else if(pGPIOx == GPIOC)
	{
		GPIOC_REG_RESET();
	}else if(pGPIOx == GPIOD)
	{
		GPIOD_REG_RESET();
	}else if(pGPIOx == GPIOE)
	{
		GPIOE_REG_RESET();
	}else if(pGPIOx == GPIOF)
	{
		GPIOF_REG_RESET();
	}else if(pGPIOx == GPIOG)
	{
		GPIOG_REG_RESET();
	}else if(pGPIOx == GPIOH)
	{
		GPIOH_REG_RESET();
	}
}

/*********************************************************
 * @fn			- GPIO_ReadFromInputPin
 * @brief		- This function reads the input data
 *
 * @param[in]	- base address of the GPIO peripheral
 * @param[in]	- reading pin number
 *
 * @return		- reading value
 *
 * @notes		- Input data register is read on the defined pin number and is moved to leftmost place in register(IDR0)
 *
 */
//data read pin
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)	//it has to return value
{
	uint8_t	value;
	value = (uint8_t)((pGPIOx -> IDR >> PinNumber) & 0x00000001); //left shifting the reading bit to leftmost register and masking to read it
	return value;
}

/*********************************************************
 * @fn			- GPIO_ReadFromInputPort
 * @brief		- This function reads the data at port
 *
 * @param[in]	- base address of the GPIO peripheral
 * @param[in]	-
 *
 * @return		- read value at input data register(IDR)
 *
 * @notes		- each port is of 16 bit wide and so uint16_t is to be used
 *
 */
//data read port
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t	value;
	value = (uint16_t)pGPIOx->IDR;
	return value;
}

/*********************************************************
 * @fn			- GPIO_WriteToOutputPin
 * @brief		- This function writes the data at pin
 *
 * @param[in]	- base address of the GPIO peripheral
 * @param[in]	- pin number
 * @param[in]	- writing value
 *
 * @return		- void
 *
 * @notes		- write value to output data register(ODR)
 *
 */
//data write to output pin
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)	//nothing to return i.e., void
{
	if(Value == GPIO_PIN_SET)
	{
		//write 1 in the bit field corresponding to pin  the number
		pGPIOx->ODR |= (1 << PinNumber);
	}else
	{
		//write 0
		pGPIOx->ODR &= ~(1 << PinNumber);
	}
}

/*********************************************************
 * @fn			- GPIO_WriteToOutputPort
 * @brief		- This function writes the data at output port
 *
 * @param[in]	- base address of the GPIO peripheral
 * @param[in]	- writing value
 *
 * @return		- void
 *
 * @notes		- write value directly to the output data register
 *
 */
//data write to output port
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value)
{
	pGPIOx ->ODR = Value;
}

/*********************************************************
 * @fn			- GPIO_ToggleOutputPin
 * @brief		- This function toggles defined output pin
 *
 * @param[in]	- base address of the GPIO peripheral
 * @param[in]	- toggling pin number
 *
 * @return		- void
 *
 * @notes		- Toggling with XOR operator.
 * 				  Toggling means if bit is 1, make it 0 and if 0, make it 1
 *
 */
//toggle output pin
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx ->ODR ^= (1 << PinNumber);
}

/*********************************************************
 * @fn			- GPIO_IRQInterruptConfig
 * @brief		- Interrupt configuration and handling
 *
 * @param[in]	-
 * @param[in]	-
 *
 * @return		- void
 *
 * @notes		- referring to the ARM Cortex M4 reference manual 'NVIC Register Summary'
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(IRQNumber <= 31)
		{
			//program ISER0
			*NVIC_ISER0 |= (1 << IRQNumber);

		}else if(IRQNumber > 31 && IRQNumber < 64)
		{
			//program ISER1
			*NVIC_ISER1 |= (1 << IRQNumber % 32);	//moving to the next register ISER1(32 bit register)

		}else if(IRQNumber >= 64 && IRQNumber < 96)
		{
			//program ISER2
			*NVIC_ISER2 |= (1 << IRQNumber % 64);	////moving to the next register ISER2
		}
	}else
	{
		if(IRQNumber <= 31)
		{
			//program ICER0
			*NVIC_ICER0 |= (1 << IRQNumber);

		}else if(IRQNumber > 31 && IRQNumber < 64)
		{
			//program ICER1
			*NVIC_ICER1 |= (1 << IRQNumber % 32);

		}else if(IRQNumber >= 64 && IRQNumber < 96)
		{
			//program ICER2
			*NVIC_ICER2 |= (1 << IRQNumber % 64);
		}
	}
}

/*********************************************************
 * @fn			- GPIO_IRQPriorityConfig
 * @brief		- Interrupt configuration
 *
 * @param[in]	- IRQNumber
 * @param[in]	-
 *
 * @return		- void
 *
 * @notes		-
 */
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	//1) find out the IPRx
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;
	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);

	//IPRx multiplied by 4 because 4 bits out of 8 on right side should not be modified(only if uint8_t IRQPriority or define uint32_t IRQPriority)
	*(NVIC_PR_BASE_ADDR + iprx) |= (IRQPriority << shift_amount);
}

/*********************************************************
 * @fn			- GPIO_IRQHandling
 * @brief		- Interrupt handling
 *
 * @param[in]	-
 * @param[in]	-
 *
 * @return		- void
 *
 * @notes		-
 */
void GPIO_IRQHandling(uint8_t PinNumber)
{
 //clear the EXTIPR register
	if(EXTI ->PR1 |= (1 << PinNumber))
	{
		//clear
		EXTI ->PR1 &= ~(1 << PinNumber);
	}
}
