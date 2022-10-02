/*
 * stm32f4xx_spi_driver.c
 *
 */

#include "stm32f4xx_spi_driver.h"

/*********************************************************************
 * @fn      		  - SPI_PeriClockControl
 *
 * @brief             - SPI Peripheral Clock Control
 *
 * @param[in]         - Base address of SPI peripheral
 * @param[in]         - Flag to indicate enable or disable
 *
 * @return            -	None
 *
 * @Note              - Enable or Disable clock for the SPI peripheral
 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{

	if(EnorDi == ENABLE)
	{
		if(pSPIx == SPI1)
		{
			SPI1_PCLK_EN();
		}
		else if (pSPIx == SPI2)
		{
			SPI2_PCLK_EN();
		}
		else if (pSPIx == SPI3)
		{
			SPI3_PCLK_EN();
		}
		else if (pSPIx == SPI4)
		{
			SPI4_PCLK_EN();
		}
	}
	else
	{
		if(pSPIx == SPI1)
		{
			SPI1_PCLK_DI();
		}
		else if (pSPIx == SPI2)
		{
			SPI2_PCLK_DI();
		}
		else if (pSPIx == SPI3)
		{
			SPI3_PCLK_DI();
		}
		else if (pSPIx == SPI4)
		{
			SPI4_PCLK_DI();
		}
	}
}


/*********************************************************************
 * @fn      		  - SPI_Init
 *
 * @brief             - SPI Init function
 *
 * @param[in]         - Pointer to SPI handle structure
 *
 * @return            - None
 *
 * @Note              - Initialize SPI peripheral
 */
void SPI_Init(SPI_Handle_t *pSPIHandle)
{

	/* Peripheral clock enable */
	SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);

	/* First configure the SPI_CR1 register */
	uint32_t tempreg = 0;

	/* 1. configure the device mode */
	tempreg |= pSPIHandle->SPI_Config.SPI_DeviceMode << SPI_CR1_MSTR;

	/* 2. Configure the bus config */
	if(pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_FD)
	{
		/* Bi-Di mode should be cleared since full-duplex uses 2 lines in unidirectional */
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);
	}
	else if (pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_HD)
	{
		/* Bi-Di mode should be set as half duplex uses 1 line in bi-directinal */
		tempreg |= (1 << SPI_CR1_BIDIMODE);
	}
	else if (pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY)
	{
		/* Bi-Di mode should be cleared as simplex uses 1 line in uni-directional */
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);
		/* Rx ONLY bit must be set since its in simplex Rx only config */
		tempreg |= (1 << SPI_CR1_RXONLY);
	}

	/* 3. Configure the SPI serial clock speed (baud rate) */
	tempreg |= pSPIHandle->SPI_Config.SPI_SclkSpeed << SPI_CR1_BR;

	/* 4. Configure the DFF */
	tempreg |= pSPIHandle->SPI_Config.SPI_DFF << SPI_CR1_DFF;

	/* 5. Configure the CPOL */
	tempreg |= pSPIHandle->SPI_Config.SPI_CPOL << SPI_CR1_CPOL;

	/* 6. Configure the CPHA */
	tempreg |= pSPIHandle->SPI_Config.SPI_CPHA << SPI_CR1_CPHA;

	/* 6. Configure the SSM */
	tempreg |= pSPIHandle->SPI_Config.SPI_SSM << SPI_CR1_SSM;

	pSPIHandle->pSPIx->CR1 = tempreg;

	if(pSPIHandle->SPI_Config.SPI_SSM == 0)
	{
		pSPIHandle->pSPIx->CR2 = 1 << SPI_CR2_SSOE;
	}

}


/*********************************************************************
 * @fn      		  - SPI_DeInit
 *
 * @brief             - De-initialize the SPI peripheral
 *
 * @param[in]         -	SPI Base Address
 *
 * @return            -	None
 *
 * @Note              -	Disable the clock to SPI peripheral
 */
void SPI_DeInit(SPI_RegDef_t *pSPIx)
{
	/* Peripheral clock disable */
	SPI_PeriClockControl(pSPIx, DISABLE);
}


/*********************************************************************
 * @fn      		  - SPI_GetFlagStatus
 *
 * @brief             - Get Flag Status of the SPI peripheral
 *
 * @param[in]         -	SPI Base Address
 * @param[in]         -	FlagName (The flag for which status is requried)
 *
 * @return            -	None
 *
 * @Note              -	Get status of flag
 */
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx , uint32_t FlagName)
{
	uint8_t status;
	if(pSPIx->SR & FlagName)
	{
		status = FLAG_SET;
	}
	else
	{
		status = FLAG_RESET;
	}
	return status;
}


/*********************************************************************
 * @fn      		  - SPI_SendData
 *
 * @brief             -	Send Data over the SPI lines
 *
 * @param[in]         -	Pointer to the SPI base address
 * @param[in]         -	Pointer to SPI Tx Buffer
 * @param[in]         -	Length of the buffer
 *
 * @return            - None
 *
 * @Note              - This is blocking call
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len)
{
	while(Len > 0)
	{
		/* 1. Wait until TXE (Tx Buffer Empty) flag bit is set */
		while(SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET);

		/* 2. Check the DFF bit in CR1 */
		if((pSPIx->CR1 & (1 << SPI_CR1_DFF)))
		{
			/* If DFF is 16 bit then dereference 16 bits from buffer */
			/* 1. load the data in to the DR */
			pSPIx->DR = *((uint16_t*)pTxBuffer);
			Len -= 2; /* Decrement length by 2 as we sent 2 bytes from the buffer */
			(uint16_t*)pTxBuffer++; /* Move through the buffer (2 bytes done) */
		}
		else
		{
			/* 8 bit DFF */
			pSPIx->DR = *pTxBuffer;
			Len--; /* Decrement length by 1 as we sent 1 byte from the buffer */
			pTxBuffer++; /* Move through the buffer (1 byte done) */
		}
	}
}


/*********************************************************************
 * @fn      		  - SPI_ReceiveData
 *
 * @brief             - Receive Data over the SPI lines
 *
 * @param[in]         -	Pointer to the SPI base address
 * @param[in]         - Pointer to the buffer holding the data to be transmitted
 * @param[in]         - Length of the data to be transmitted over the buffer
 *
 * @return            -	none
 *
 * @Note              - This is also a blocking call
 */
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len)
{
	while(Len > 0)
	{
		/* 1. Wait until RXNE is set */
		while(SPI_GetFlagStatus(pSPIx, SPI_RXNE_FLAG) == (uint8_t)FLAG_RESET);

		/* 2. Check the DFF bit in CR1 */
		if((pSPIx->CR1 & (1 << SPI_CR1_DFF)))
		{
			/* 16 bit DFF */
			/* 1. load the data from DR to Rx buffer address */
			*((uint16_t*)pRxBuffer) = pSPIx->DR ;
			Len -= 2;
			(uint16_t*)pRxBuffer++;
		}
		else
		{
			/* 8 bit DFF */
			*(pRxBuffer) = pSPIx->DR ;
			Len--;
			pRxBuffer++;
		}
	}
}


/*********************************************************************
 * @fn      		  - SPI_PeripheralControl
 *
 * @brief             - Enable or Disable SPI peripheral
 *
 * @param[in]         - Base address of SPI Peripheral
 * @param[in]         - Enable or Disable
 *
 * @return            - None
 *
 * @Note              - Set or Clear SPI enable bit in SPI_CR1
 */
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pSPIx->CR1 |= (1 << SPI_CR1_SPE);
	}
	else
	{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
	}
}


/*********************************************************************
 * @fn      		  - SPI_SSIConfig
 *
 * @brief             - Configure internal slave select
 *
 * @param[in]         - Pointer to base address of SPI peripheral
 * @param[in]         -	Enable or Disable
 *
 * @return            - None
 *
 * @Note              - Set or Clear SSI pin in SPI_CR1
 */
void  SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pSPIx->CR1 |=  (1 << SPI_CR1_SSI);
	}
	else
	{
		pSPIx->CR1 &=  ~(1 << SPI_CR1_SSI);
	}
}

