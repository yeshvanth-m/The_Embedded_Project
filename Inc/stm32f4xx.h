/*
 * @File stm32f4xx.h
 *
 * @Description
 *  MCU Specific Header File
 *  Contains MACROS defining Base Address of Peripherals,
 *  Constant values used across the drivers and application,
 *  and Register Definition Structures
 *
 */

#ifndef INC_STM32F4XX_H_
#define INC_STM32F4XX_H_

#include <stdint.h>		/* standard header file for to define integers (uint32_t etc,..) */
#define __vo volatile	/* MACRO for volatile keyword */

/************ Processor specific details *****************/
/*
 * ARM Cortex M4 processor NVIC ISERx register addresses
 */
#define NVIC_ISER0				((__vo uint32_t*)0xE000E100)
#define NVIC_ISER1				((__vo uint32_t*)0xE000E104)
#define NVIC_ISER2				((__vo uint32_t*)0xE000E108)
#define NVIC_ISER3				((__vo uint32_t*)0xE000E10C)

/*
 * ARM Cortex M4 processor NVIC ICERx register addresses
 */
#define NVIC_ICER0				((__vo uint32_t*)0xE000E180)
#define NVIC_ICER1				((__vo uint32_t*)0xE000E184)
#define NVIC_ICER2				((__vo uint32_t*)0xE000E188)
#define NVIC_ICER3				((__vo uint32_t*)0xE000E18C)

/*
 * ARM Cortex M4 processor priority register address calculation
 */
#define NVIC_PR_BASE_ADDR 		((__vo uint32_t*)0xE000E400)

/*
 * ARM Cortex Mx processor number of priority bits implemented in priority register
 */
#define NO_PR_BITS_IMPLEMENTED 	4	/* Interrupt priority bits fixed for all Cortex M4 */


/* MCU Specific addresses - these are specific to STM32F4xx Microcontrollers */
/* Base addresses of Flash and SRAM */
#define FLASH_BASEADDR			0x08000000UL	/* Flash memory base address, extends till  0x0807FFFF */
#define SRAM_BASEADDR			0x20000000UL	/* SRAM base address, 64KB size */
#define ROM_BASEADDR			0x1FFF0000UL    /* 0x1FFFD800UL	system memory base address, 8K size */

/* APBx and AHBx bus domain peripheral base addresses */
#define PERIPH_BASEADDR			0x40000000UL	/* All the external peripheral starts after this address */
#define APB1PERIPH_BASEADDR		0x40000000UL
#define APB2PERIPH_BASEADDR		0x40010000UL

#define AHB1PERIPH_BASEADDR		0x40020000UL
#define AHB2PERIPH_BASEADDR		0x50000000UL
#define AHB3PERIPH_BASEADDR		0xA0000000UL

/* Base addresses of peripherals on AHB1 */
#define GPIOA_BASEADDR			(AHB1PERIPH_BASEADDR + 0x0000)	/* GPIOA starts at base address of AHB2 */
#define GPIOB_BASEADDR			(AHB1PERIPH_BASEADDR + 0x0400)
#define GPIOC_BASEADDR			(AHB1PERIPH_BASEADDR + 0x0800)
#define GPIOD_BASEADDR			(AHB1PERIPH_BASEADDR + 0x0C00)
#define GPIOE_BASEADDR			(AHB1PERIPH_BASEADDR + 0x1000)
#define GPIOH_BASEADDR			(AHB1PERIPH_BASEADDR + 0x1C00)
#define CRC_BASE_ADDR			(AHB1PERIPH_BASEADDR + 0x3000)
#define RCC_BASEADDR			(AHB1PERIPH_BASEADDR + 0x3800)	/* RCC starts at 0x3800 offset from AHB1 base address */
#define FIR_BASEADDR			(AHB1PERIPH_BASEADDR + 0x3C00)  /* Flash Interface register */
#define DMA1_BASEADDR          	(AHB1PERIPH_BASEADDR + 0x6000)
#define DMA2_BASEADDR          	(AHB1PERIPH_BASEADDR + 0x6400)

/* Base addresses of peripherals on APB1 */
#define TIM2_BASEADDR 			(APB1PERIPH_BASEADDR + 0x0000)
#define TIM3_BASEADDR 			(APB1PERIPH_BASEADDR + 0x0400)
#define TIM4_BASEADDR 			(APB1PERIPH_BASEADDR + 0x0800)
#define TIM5_BASEADDR 			(APB1PERIPH_BASEADDR + 0x0C00)
#define RTC_BASEADDR			(APB1PERIPH_BASEADDR + 0x2800)
#define SPI2_BASEADDR			(APB1PERIPH_BASEADDR + 0x3800)
#define SPI3_BASEADDR			(APB1PERIPH_BASEADDR + 0x3C00)
#define USART2_BASEADDR			(APB1PERIPH_BASEADDR + 0x4400)
#define USART3_BASEADDR			(APB1PERIPH_BASEADDR + 0x4800)
#define UART4_BASEADDR			(APB1PERIPH_BASEADDR + 0x4C00)
#define UART5_BASEADDR			(APB1PERIPH_BASEADDR + 0x5000)
#define I2C1_BASEADDR			(APB1PERIPH_BASEADDR + 0x5400)
#define I2C2_BASEADDR			(APB1PERIPH_BASEADDR + 0x5800)
#define I2C3_BASEADDR			(APB1PERIPH_BASEADDR + 0x5C00)
#define PWR_BASEADDR			(APB1PERIPH_BASEADDR + 0x7000)

/* Base addresses of peripherals on APB2 */
#define TIM1_BASEADDR 			(APB2PERIPH_BASEADDR + 0x0000)
#define USART1_BASEADDR 		(APB2PERIPH_BASEADDR + 0x1000)
#define USART6_BASEADDR 		(APB2PERIPH_BASEADDR + 0x1400)
#define ADC1_BASEADDR	 		(APB2PERIPH_BASEADDR + 0x2000)
#define SDIO_BASEADDR	 		(APB2PERIPH_BASEADDR + 0x2C00)
#define SPI1_BASEADDR			(APB2PERIPH_BASEADDR + 0x3000)
#define SPI4_BASEADDR			(APB2PERIPH_BASEADDR + 0x3400)
#define SYSCFG_BASEADDR			(APB2PERIPH_BASEADDR + 0x3800)
#define EXTI_BASEADDR			(APB2PERIPH_BASEADDR + 0x3C00)
#define TIM9_BASEADDR			(APB2PERIPH_BASEADDR + 0x4000)
#define TIM10_BASEADDR			(APB2PERIPH_BASEADDR + 0x4400)
#define TIM11_BASE_ADDR 		(APB2PERIPH_BASEADDR + 0x4800)


/******** Peripheral Register Definition Structures ********/

/* This includes each register name and address for peripherals defined above in each buses
 * Written referring to the device Reference Manual register map
 * The offset of 0x04 is auto taken from the starting address as 0x00 inside structure definitions
 * Make sure that each register names are in consecutive order of offset
 */

typedef struct
{
	__vo uint32_t MODER;		/* address offset 0x00 */
	__vo uint32_t OTYPER;		/* address offset 0x04 */
	__vo uint32_t OSPEEDR;		/* address offset 0x08 */
	__vo uint32_t PUPDR;		/* address offset 0x0C */
	__vo uint32_t IDR;			/* address offset 0x10 */
	__vo uint32_t ODR;			/* address offset 0x14 */
	__vo uint32_t BSRR;			/* address offset 0x18 */
	__vo uint32_t LCKR;			/* address offset 0x1C */
	__vo uint32_t AFR[2];		/* AFR[0] is AFRH register and AFR[1] is AFRL register */
} GPIO_RegDef_t;	/* _t is the type definition */

typedef struct
{
	__vo uint32_t CR;		  /* address offset 0x00 */
	__vo uint32_t PLLCFGR;    /* address offset 0x04 */
	__vo uint32_t CFGR;		  /* address offset 0x08 */
	__vo uint32_t CIR;
	__vo uint32_t AHB1RSTR;
	__vo uint32_t AHB2RSTR;
	__vo uint32_t RESERVED1;
	__vo uint32_t RESERVED2;
	__vo uint32_t APB1RSTR;
	__vo uint32_t APB2RSTR;
	__vo uint32_t RESERVED3;
	__vo uint32_t RESERVED4;
	__vo uint32_t AHB1ENR;
	__vo uint32_t AHB2ENR;
	__vo uint32_t RESERVED5;
	__vo uint32_t RESERVED6;
	__vo uint32_t APB1ENR;
	__vo uint32_t APB2ENR;
	__vo uint32_t RESERVED7;
	__vo uint32_t RESERVED8;
	__vo uint32_t AHB1LPENR;
	__vo uint32_t AHB2LPENR;
	__vo uint32_t RESERVED10;
	__vo uint32_t RESERVED11;
	__vo uint32_t APB1LPENR;
	__vo uint32_t APB2LPENR;
	__vo uint32_t RESERVED12;
	__vo uint32_t RESERVED13;
	__vo uint32_t BDCR;
	__vo uint32_t CSR;
	__vo uint32_t RESERVED14;
	__vo uint32_t RESERVED15;
	__vo uint32_t SSCGR;
	__vo uint32_t PLLI2SCFGR;
	__vo uint32_t RESERVED16;
	__vo uint32_t DCKCFGR;
} RCC_RegDef_t;

/* peripheral definition structure for EXTI */
typedef struct
{
	__vo uint32_t IMR;		//address offset 0x00
	__vo uint32_t EMR;		//address offset 0x04
	__vo uint32_t RTSR;
	__vo uint32_t FTSR;
	__vo uint32_t SWIER;
	__vo uint32_t PR;
} EXTI_RegDef_t;

/* peripheral definition structure for SYSCFG */
typedef struct
{
	__vo uint32_t MEMRMP;		//address offset 0x00
	__vo uint32_t PMC;			//address offset 0x04
	__vo uint32_t EXTICR[4];	//address offset 0x08 to 0x14
	__vo uint32_t CMPCR;		//address offset 0x20
} SYSCFG_RegDef_t;

/* peripheral definition structure for SPI */
typedef struct
{
	__vo uint32_t CR1;
	__vo uint32_t CR2;
	__vo uint32_t SR;
	__vo uint32_t DR;
	__vo uint32_t CRCPR;
	__vo uint32_t RXCRCR;
	__vo uint32_t TXCRCR;
	__vo uint32_t I2SCFGR;
	__vo uint32_t I2SPR;
} SPI_RegDef_t;

/*
 * peripheral definitions (peripheral base address typecasted to xxx_RegDef_t)
 * GPIO_RegDef_t *pGPIOA = (GPIO_RegDef_t*) GPIOA_BASEADDR
 * Instead of GPIO_RegDef_t *pGPIOA, macros are defined as GPIOx.
 */
#define GPIOA			((GPIO_RegDef_t*) GPIOA_BASEADDR)
#define GPIOB			((GPIO_RegDef_t*) GPIOB_BASEADDR)
#define GPIOC			((GPIO_RegDef_t*) GPIOC_BASEADDR)
#define GPIOD			((GPIO_RegDef_t*) GPIOD_BASEADDR)
#define GPIOE			((GPIO_RegDef_t*) GPIOE_BASEADDR)
#define GPIOH			((GPIO_RegDef_t*) GPIOH_BASEADDR)

#define RCC 			((RCC_RegDef_t*) RCC_BASEADDR)
#define EXTI			((EXTI_RegDef_t*) EXTI_BASEADDR)
#define SYSCFG			((SYSCFG_RegDef_t*) SYSCFG_BASEADDR)

#define SPI1			((SPI_RegDef_t*) SPI1_BASEADDR)
#define SPI2			((SPI_RegDef_t*) SPI2_BASEADDR)
#define SPI3			((SPI_RegDef_t*) SPI3_BASEADDR)
#define SPI4			((SPI_RegDef_t*) SPI4_BASEADDR)

/******** Peripheral clock enable referring to RCC_AHBENR/RCC_APBxENR register map ********/
/*
 * Macros for the GPIOx peripheral clock enable
 * GPIOx_PCLK_EN() will be higher level macro to enable peripheral clock as API
 * Example: dereferencing RCC as RCC->AHBENR = 25; (compiler will take it as *(RCC_BASEADDR + 0x14) = 25;
 */
#define GPIOA_PCLK_EN()	(RCC->AHB1ENR |= (1 << 0))	/* setting 0th bit of AHBENR register for GPIOA */
#define GPIOB_PCLK_EN()	(RCC->AHB1ENR |= (1 << 1))	/* setting 1st bit of AHBENR register for GPIOB */
#define GPIOC_PCLK_EN()	(RCC->AHB1ENR |= (1 << 2))
#define GPIOD_PCLK_EN()	(RCC->AHB1ENR |= (1 << 3))
#define GPIOE_PCLK_EN()	(RCC->AHB1ENR |= (1 << 4))
#define GPIOH_PCLK_EN()	(RCC->AHB1ENR |= (1 << 7))

/*
 * Macros for I2Cx peripheral clock enable
 */
#define I2C1_PCLK_EN()	(RCC->APB1ENR |= (1 << 21))
#define I2C2_PCLK_EN()	(RCC->APB1ENR |= (1 << 22))
#define I2C3_PCLK_EN()	(RCC->APB1ENR |= (1 << 23))

/*
 * Macros for SPIx peripheral clock enable
 */
#define SPI1_PCLK_EN()	(RCC->APB2ENR |= (1 << 12))
#define SPI2_PCLK_EN()	(RCC->APB1ENR |= (1 << 14))
#define SPI3_PCLK_EN()	(RCC->APB1ENR |= (1 << 15))
#define SPI4_PCLK_EN()	(RCC->APB2ENR |= (1 << 13))

/*
 * Macros for USARTx/UARTx peripheral clock enable
 */
#define USART1_PCLK_EN()	(RCC->APB2ENR |= (1 << 4))
#define USART2_PCLK_EN()	(RCC->APB1ENR |= (1 << 17))
#define USART6_PCLK_EN()	(RCC->APB2ENR |= (1 << 5))

/*
 * Macros for SYSCFG peripheral clock enable
 */
#define SYSCFG_PCLK_EN()	(RCC->APB2ENR |= (1 << 14))


/******** Peripheral clock disable referring to RCC_AHBENR/RCC_APBxENR register map ********/

/*
 * Macros for GPIOx peripheral clock disable
 */
#define GPIOA_PCLK_DI()	(RCC->AHB1ENR &= ~(1 << 0))	/* clearing 0th bit of AHBENR register for GPIOA */
#define GPIOB_PCLK_DI()	(RCC->AHB1ENR &= ~(1 << 1))	/* clearing 1st bit of AHBENR register for GPIOB */
#define GPIOC_PCLK_DI()	(RCC->AHB1ENR &= ~(1 << 2))
#define GPIOD_PCLK_DI()	(RCC->AHB1ENR &= ~(1 << 3))
#define GPIOE_PCLK_DI()	(RCC->AHB1ENR &= ~(1 << 4))
#define GPIOH_PCLK_DI()	(RCC->AHB1ENR &= ~(1 << 7))

/*
 * Macros for I2Cx peripheral clock disable
 */
#define I2C1_PCLK_DI()	(RCC->APB1ENR &= ~(1 << 21))
#define I2C2_PCLK_DI()	(RCC->APB1ENR &= ~(1 << 22))
#define I2C3_PCLK_DI()	(RCC->APB1ENR &= ~(1 << 23))

/*
 * Macros for SPIx peripheral clock disable
 */
#define SPI1_PCLK_DI()	(RCC->APB2ENR &= ~(1 << 12))
#define SPI2_PCLK_DI()	(RCC->APB1ENR &= ~(1 << 14))
#define SPI3_PCLK_DI()	(RCC->APB1ENR &= ~(1 << 15))
#define SPI4_PCLK_DI()	(RCC->APB2ENR &= ~(1 << 13))

/*
 * Macros for USARTx/UARTx peripheral clock disable
 */
#define USART1_PCLK_DI()	(RCC->APB2ENR &= ~(1 << 4))
#define USART2_PCLK_DI()	(RCC->APB1ENR &= ~(1 << 17))
#define USART6_PCLK_DI()	(RCC->APB2ENR &= ~(1 << 5))

/*
 * Macros for SYSCFG peripheral clock disable
 */
#define SYSCFG_PCLK_DI()	(RCC->APB2ENR &= ~(1 << 14))

/*
 * Macros to reset GPIO peripherals using AHB bus reset register (AHBRSTR)
 * The bit has to be set to perform reset operation and then we should release the bit(make it 1 and then 0)
 * Using do-while loop to execute multiple instruction in single macro
 */
#define GPIOA_REG_RESET()	do{ (RCC ->AHB1RSTR |= (1 << 0)); (RCC -> AHB1RSTR &= ~(1 << 0)); } while(0)
#define GPIOB_REG_RESET()	do{ (RCC ->AHB1RSTR |= (1 << 1)); (RCC -> AHB1RSTR &= ~(1 << 1)); } while(0)
#define GPIOC_REG_RESET()	do{ (RCC ->AHB1RSTR |= (1 << 2)); (RCC -> AHB1RSTR &= ~(1 << 2)); } while(0)
#define GPIOD_REG_RESET()	do{ (RCC ->AHB1RSTR |= (1 << 3)); (RCC -> AHB1RSTR &= ~(1 << 3)); } while(0)
#define GPIOE_REG_RESET()	do{ (RCC ->AHB1RSTR |= (1 << 4)); (RCC -> AHB1RSTR &= ~(1 << 4)); } while(0)
#define GPIOH_REG_RESET()	do{ (RCC ->AHB1RSTR |= (1 << 7)); (RCC -> AHB1RSTR &= ~(1 << 7)); } while(0)

/* Ternary conditional operator function */
#define GPIO_BASEADDR_TO_CODE(x)   ((x == GPIOA)?0:\
									(x == GPIOB)?1:\
									(x == GPIOC)?2:\
									(x == GPIOD)?3:\
									(x == GPIOE)?4:\
									(x == GPIOH)?7:0);

/*
 * Interrupt request position numbers for STM32F401xx
 * Taken from Vector Table in reference manual
 */
#define IRQ_NO_EXTI0				6
#define IRQ_NO_EXTI1				7
#define IRQ_NO_EXTI2				8
#define IRQ_NO_EXTI3				9
#define IRQ_NO_EXTI4				10
#define IRQ_NO_EXTI9_5				23
#define	IRQ_NO_EXTI15_10			40

/*
 * Interrupt priority for STM32F303xx
 */
#define NVIC_IRQ_PRI_0		0
#define NVIC_IRQ_PRI_1		1
#define NVIC_IRQ_PRI_2		2
#define NVIC_IRQ_PRI_3		3
#define NVIC_IRQ_PRI_4		4
#define NVIC_IRQ_PRI_5		5
#define NVIC_IRQ_PRI_6		6
#define NVIC_IRQ_PRI_7		7
#define NVIC_IRQ_PRI_8		8
#define NVIC_IRQ_PRI_9		9
#define NVIC_IRQ_PRI_10		10
#define NVIC_IRQ_PRI_11		11
#define NVIC_IRQ_PRI_12		12
#define NVIC_IRQ_PRI_13		13
#define NVIC_IRQ_PRI_14		14
#define NVIC_IRQ_PRI_15		15


/*
 * Generic macros
 */
#define SET					1
#define RESET				0
#define ENABLE				SET
#define DISABLE 			RESET
#define GPIO_PIN_SET		SET
#define GPIO_PIN_RESET		RESET
#define FLAG_RESET			RESET
#define FLAG_SET			SET

/* Including GPIO driver header to have dependency with stm32f4xx.h and pull both to main.c in one header definition. */
#include "stm32f4xx_gpio_driver.h"
#include "stm32f4xx_spi_driver.h"

#endif /* INC_STM32F4XX_H_ */
