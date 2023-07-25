/*
 * stm32f407xx_spi_driver.c
 *
 *  Created on: Jul 7, 2023
 *      Author: mucahit
 */

#include "stm32f407xx_spi_driver.h"

static void spi_txe_interrupt_handle(SPI_Handle_t *pHandle);
static void spi_rxne_interrupt_handle(SPI_Handle_t *pHandle);
static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pHandle);
//here we used static to say that these functions are private we do not need user application to have access

//some helper function implementation

static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle) {

	//2.check DFF bit in CR1
	if (pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF)) {

		//16 bit DFF
		// load data to DR
		pSPIHandle->pSPIx->DR = *((uint16_t*) pSPIHandle->pTxBuffer); // here we load the data to DR register and we convert
		//tx buffer to 16 because the DFF is 1
		pSPIHandle->TxLen--;
		pSPIHandle->TxLen--;
		// here we decrease the data length twice because it is 16 bit
		(uint16_t*) pSPIHandle->pTxBuffer++; // here we increment the buffer that mean data was sent

	} else {

		//8 bit DFF
		// load data to DR
		pSPIHandle->pSPIx->DR = *pSPIHandle->pTxBuffer; // here we load the data to DR register
		pSPIHandle->TxLen--;
		pSPIHandle->pTxBuffer++;

	}

	if (!pSPIHandle->TxLen) {

		//if Txlen is zero then close the spi communication

		// this prevent the interrupts from setting up of TXE flag
		SPI_CloseTransmisson(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_TX_CMPLT);

	}

}

static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle) {

	if (pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF)) {

		//16 bit DFF
		// read data from DR to Rx buffer
		*((uint16_t*) pSPIHandle->pRxBuffer) = pSPIHandle->pSPIx->DR;
		pSPIHandle->RxLen--;
		pSPIHandle->RxLen--;
		// here we decrease the data length twice because it is 16 bit
		(uint16_t*) pSPIHandle->pRxBuffer++; // here we increment the buffer that mean data was sent

	} else {

		//8 bit DFF
		// read data from DR
		*pSPIHandle->pRxBuffer = pSPIHandle->pSPIx->DR; // here we read the data from Rx buffer
		pSPIHandle->RxLen--;
		pSPIHandle->pRxBuffer++;
	}

	if (!pSPIHandle->RxLen) {

		//if Rxlen is zero then close the spi communication
		// reception is complete
		// this prevent the interrupts from setting up of RXE flag
		SPI_CloseReception(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_RX_CMPLT);

	}

}

static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle) {

	uint8_t temp;
	//1.clear over run flag by reading data from DR register
	if (pSPIHandle->TxState != SPI_BUSY_IN_TX) {

		temp = pSPIHandle->pSPIx->DR;
		temp = pSPIHandle->pSPIx->SR;
	}

	//2.inform the application
	SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_OVR_ERR);

}

void SPI_CloseTransmisson(SPI_Handle_t *pSPIHandle) {

	pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_TXEIE);

	//reset txbuffer
	pSPIHandle->pTxBuffer = NULL;
	pSPIHandle->TxLen = 0;
	pSPIHandle->TxState = SPI_READY;

}

void SPI_CloseReception(SPI_Handle_t *pSPIHandle) {

	pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_RXNEIE);

	//reset Rxbuffer
	pSPIHandle->pRxBuffer = NULL;
	pSPIHandle->RxLen = 0;
	pSPIHandle->RxState = SPI_READY;

}

void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx) {

	uint8_t temp;
	temp = pSPIx->DR;
	temp = pSPIx->SR;
	(void) temp; //to avoid the warning that the variable not used

}

uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName) {

	if (pSPIx->SR & FlagName) { // if this statement is true that mean the flag is set

		return FLAG_SET;
	}

	return FLAG_RESET;
}

//peripheral clock setup
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi) {

	if (EnorDi == ENABLE) {

		if (pSPIx == SPI1) {
			SPI1_PCLK_EN();
		} else if (pSPIx == SPI2) {
			SPI2_PCLK_EN();
		} else if (pSPIx == SPI3) {
			SPI3_PCLK_EN();
		} else if (pSPIx == SPI4) {
			SPI4_PCLK_EN();
		}
	} else {

		if (pSPIx == SPI1) {
			SPI1_PCLK_DI();
		} else if (pSPIx == SPI2) {
			SPI2_PCLK_DI();
		} else if (pSPIx == SPI3) {
			SPI3_PCLK_DI();
		} else if (pSPIx == SPI4) {
			SPI4_PCLK_DI();
		}
	}
}

//init and de-init
void SPI_Init(SPI_Handle_t *pSPIHandle) {

	//peripheral clock enable
	SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);

	//first lets configure the SPI_CR1 register
	uint32_t tempreg = 0;

	//1.configure the device mode
	tempreg |= pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR;

	//2.configure the bus config
	if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_FULL_DUPLEX) {

		//BIDIMODE should be cleared
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);

	} else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_HALF_DUPLEX) {

		//BIDIMODE should be set
		tempreg |= 1 << SPI_CR1_BIDIMODE;

	} else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_SIMPLEX_RXONLY) {

		//BIDIMODE should be cleared
		//RXONLY bit should be set
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);
		tempreg |= 1 << SPI_CR1_RXONLY;

	}

	//3.configure the SCLK speed
	tempreg |= pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR;

	//4.configure DFF
	tempreg |= pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF;

	//5.configure CPOL
	tempreg |= pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL;

	//6.configure CPOL
	tempreg |= pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA;

	//7.configure  SSM
	tempreg |= pSPIHandle->SPIConfig.SPI_SSM << SPI_CR1_SSM;

	// now we set the value of tempreg to the CR1 register
	pSPIHandle->pSPIx->CR1 = tempreg;
}

void SPI_DeInit(SPI_RegDef_t *pSPIx) {

	if (pSPIx == SPI1) {
		SPI1_REG_RESET();
	} else if (pSPIx == SPI2) {
		SPI2_REG_RESET();
	} else if (pSPIx == SPI3) {
		SPI3_REG_RESET();
	} else if (pSPIx == SPI4) {
		SPI4_REG_RESET();
	}
}

// Data send and receive
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len) {

	while (Len > 0) {

		//1.wait until  TXE is set which mean Tx buffer is empty
		while (SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET)
			;

		//2.check DFF bit in CR1
		if (pSPIx->CR1 & (1 << SPI_CR1_DFF)) {

			//16 bit DFF
			// load data to DR
			pSPIx->DR = *((uint16_t*) pTxBuffer); // here we load the data to DR register and we convert
			//tx buffer to 16 because the DFF is 1
			Len--;
			Len--;
			// here we decrease the data length twice because it is 16 bit
			(uint16_t*) pTxBuffer++; // here we increment the buffer that mean data was sent

		} else {

			//8 bit DFF
			// load data to DR
			pSPIx->DR = *pTxBuffer; // here we load the data to DR register
			Len--;
			pTxBuffer++;

		}

	}

}

void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len) {

	while (Len > 0) {

		//1.wait until  RXE is set which mean Rx buffer is non empty
		while (SPI_GetFlagStatus(pSPIx, SPI_RXE_FLAG) == FLAG_RESET)
			;

		//2.check DFF bit in CR1
		if (pSPIx->CR1 & (1 << SPI_CR1_DFF)) {

			//16 bit DFF
			// read data from DR to Rx buffer
			*((uint16_t*) pRxBuffer) = pSPIx->DR;
			Len--;
			Len--;
			// here we decrease the data length twice because it is 16 bit
			(uint16_t*) pRxBuffer++; // here we increment the buffer that mean data was sent

		} else {

			//8 bit DFF
			// read data from DR
			*pRxBuffer = pSPIx->DR; // here we read the data from Rx buffer
			Len--;
			pRxBuffer++;

		}

	}

}

uint8_t SPI_SendDataIT(SPI_Handle_t *pHandle, uint8_t *pTxBuffer, uint32_t Len) {

	uint8_t state = pHandle->TxState;

	if (state != SPI_BUSY_IN_TX) { //if the spi is busy we can not modify anything

		//1.save the tx buffer address and len information in some global variable
		pHandle->pTxBuffer = pTxBuffer;
		pHandle->TxLen = Len;

		//2. mark the SPI state as busy in transmission so that no other code can take
		//over same SPI peripheral until transmission is over
		pHandle->TxState = SPI_BUSY_IN_TX;

		//3. enable that TXEIE control bit to get interrupt whenever TXE flag is set in SR
		pHandle->pSPIx->CR2 |= (1 << SPI_CR2_TXEIE);

		//4. data transmission will be handled in the ISR code (will implement later)
	}

	return state;
}

uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pHandle, uint8_t *pRxBuffer,
		uint32_t Len) {

	uint8_t state = pHandle->RxState;

	if (state != SPI_BUSY_IN_RX) { //if the spi is busy we can not modify anything

		//1.save the Rx buffer address and len information in some global variable
		pHandle->pRxBuffer = pRxBuffer;
		pHandle->RxLen = Len;

		//2. mark the SPI state as busy in receiving so that no other code can take
		//over same SPI peripheral until receiving is over
		pHandle->RxState = SPI_BUSY_IN_RX;

		//3. enable that RXNEIE control bit to get interrupt whenever RXE flag is set in SR
		pHandle->pSPIx->CR2 |= (1 << SPI_CR2_RXNEIE);

		//4. data transmission will be handled the ISR code (will implement later)
	}

	return state;

}

void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi) {

	if (EnorDi == ENABLE) {

		pSPIx->CR1 |= (1 << SPI_CR1_SPE); // here we enabling the SPI peripheral, it will not work without enabling this bit

	} else {

		pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);

	}
}

void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi) {

	if (EnorDi == ENABLE) {

		pSPIx->CR1 |= (1 << SPI_CR1_SSI);

	} else {

		pSPIx->CR1 &= ~(1 << SPI_CR1_SSI);

	}
}

// IRQ configuration and ISR handling

void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi) {

	if (EnorDi == ENABLE) {
		if (IRQNumber <= 31) {
			//program ISER0 register
			*NVIC_ISER0 |= (1 << IRQNumber);

		} else if (IRQNumber > 31 && IRQNumber < 64) {
			//program ISER1 register
			*NVIC_ISER1 |= (1 << IRQNumber % 32);

		} else if (IRQNumber >= 64) {
			//program ISER1 register
			*NVIC_ISER1 |= (1 << IRQNumber % 64);

		}

	} else {
		if (IRQNumber <= 31) {
			//program ICER0 register
			*NVIC_ICER0 |= (1 << IRQNumber);

		} else if (IRQNumber > 31 && IRQNumber < 64) {
			//program ICER1 register
			*NVIC_ICER1 |= (1 << IRQNumber % 32);

		} else if (IRQNumber >= 64) {
			//program ICER1 register
			*NVIC_ICER1 |= (1 << IRQNumber % 64);

		}
	}
}

void SPI_IRQPriorityConfig(uint8_t IRQNumber  , uint32_t IRQPriority){

	uint8_t iprx = IRQNumber / 4; // we divide by 4 because each registers take 4 IRQ Num
	uint8_t iprx_section = IRQNumber % 4;
	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLIMENTED); // here multiply by 8 because each IRQ num has 8 bit
	/*(8 - NO_PR_BITS_IMPLIMENTED) this is because in the 8 bits the implementation
	 just be in the higher 4 bits  for example 0x00001000 this value will be read 0000 so
	 we do shifting by 4 and the value be 1000  */
	*(NVIC_PR_BASE_ADDR + iprx) |= (IRQPriority << shift_amount);

}

//there are 6 possible interrupt  we handled just 3 they may happening
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle) {

	uint8_t temp1, temp2;
	//checking for TXE
	temp1 = pSPIHandle->pSPIx->SR & (1 << SPI_SR_TXE); // here if TXE flag is set then the temp1 will be 1
	temp2 = pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_TXEIE);

	if (temp1 && temp2) { // if the condition is true then it will handle txe

		//handle TXE
		spi_txe_interrupt_handle(pSPIHandle);
	}

	//checking for RXNE
	temp1 = pSPIHandle->pSPIx->SR & (1 << SPI_SR_RXNE); // here if RXNE flag is set then the temp1 will be 1
	temp2 = pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_TXEIE);

	if (temp1 && temp2) { // if the condition is true then it will handle rxne

		//handle RXNE
		spi_rxne_interrupt_handle(pSPIHandle);
	}

	//checking for OVR
	temp1 = pSPIHandle->pSPIx->SR & (1 << SPI_SR_OVR); // here if ovr flag is set then the temp1 will be 1
	temp2 = pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_ERRIE);

	if (temp1 && temp2) { // if the condition is true then it will handle rxne

		//handle OVR
		spi_ovr_err_interrupt_handle(pSPIHandle);
	}

}

__weak void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle,
		uint8_t AppEv) {

	// this is a weak implementation the user application may override this function
}
