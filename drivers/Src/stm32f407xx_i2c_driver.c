/*
 * stm32f407xx_i2c_driver.c
 *
 *  Created on: Jul 13, 2023
 *      Author: mucahit
 */

#include "stm32f407xx_i2c_driver.h"
#include "stm32f407xx_rcc_driver.h"



static void I2C_MasterHandleTXEInterrupt(I2C_Handle_t *pI2CHandle);
static void I2C_MasterHandleRXNEInterrupt(I2C_Handle_t *pI2CHandle);
static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle);

void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle) {

	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITBUFEN);
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITEVTEN);

	//reset Rxbuffer
	pI2CHandle->pRxBuffer = NULL;
	pI2CHandle->RxLen = 0;
	pI2CHandle->RxSize = 0;
	pI2CHandle->TxRxState = I2C_READY;
	if (pI2CHandle->I2C_Config.I2C_ACKControl == I2C_ACK_ENABLE) {
		pI2CHandle->pI2Cx->CR1 |= (1 << I2C_CR1_ACK);
	}
}

void I2C_CloseSendeData(I2C_Handle_t *pI2CHandle) {

	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITBUFEN);
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITEVTEN);

	//reset Txbuffer
	pI2CHandle->pTxBuffer = NULL;
	pI2CHandle->TxLen = 0;
	pI2CHandle->TxRxState = I2C_READY;

}

static void I2C_MasterHandleTXEInterrupt(I2C_Handle_t *pI2CHandle) {

	if (pI2CHandle->TxLen > 0) {

		pI2CHandle->pI2Cx->DR = *(pI2CHandle->pTxBuffer);
		pI2CHandle->TxLen--;
		pI2CHandle->pTxBuffer++;

	}
}

static void I2C_MasterHandleRXNEInterrupt(I2C_Handle_t *pI2CHandle) {

	if (pI2CHandle->RxSize == 1) {

		*pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->DR;
		pI2CHandle->RxLen--;

	}
	if (pI2CHandle->RxSize > 1) {

		if (pI2CHandle->RxLen == 2) {
			//disable ack
			pI2CHandle->pI2Cx->CR1 &= ~(1 << I2C_CR1_ACK);
		}

		*pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->DR;
		pI2CHandle->pRxBuffer++;
		pI2CHandle->RxLen--;

	}
	if (pI2CHandle->RxLen == 0) {

		//close data reception and notify the application
		//1.generate stop condition
		pI2CHandle->pI2Cx->CR1 |= (1 << I2C_CR1_STOP);

		//2.close i2c reception
		I2C_CloseReceiveData(pI2CHandle);

		//3.notify the application
		I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_RX_CMPLT);

	}

}

static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle) {
	//check for device mode
	if (pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL)) {	// device in master mode

		if (pI2CHandle->TxRxState == I2C_BUSY_IN_RX) {

			if (pI2CHandle->RxSize == 1) {
				//disable ACK
				pI2CHandle->pI2Cx->CR1 &= ~(1 << I2C_CR1_ACK);

				//clearing the ADDR flag by read SR1 , SR2
				uint32_t dummyRead = pI2CHandle->pI2Cx->SR1;
				dummyRead = pI2CHandle->pI2Cx->SR1;
				(void) dummyRead;

			}
		} else { // here if it is busy in tx we just clear the flag

			//clearing the ADDR flag by read SR1 , SR2
			uint32_t dummyRead = pI2CHandle->pI2Cx->SR1;
			dummyRead = pI2CHandle->pI2Cx->SR1;
			(void) dummyRead;

		}
	} else { // device in slave mode

		//clearing the ADDR flag by read SR1 , SR2
		uint32_t dummyRead = pI2CHandle->pI2Cx->SR1;
		dummyRead = pI2CHandle->pI2Cx->SR1;
		(void) dummyRead;

	}
}


void I2C_SlaveEnableDisableCallBackEvent(I2C_RegDef_t *pI2Cx , uint8_t EnorDi){

	if(EnorDi == ENABLE){

		pI2Cx->CR2 |= (1 << I2C_CR2_ITEVTEN);
	    pI2Cx->CR2 |= (1 << I2C_CR2_ITBUFEN);
	    pI2Cx->CR2 |= (1 << I2C_CR2_ITERREN);

	}else{

		pI2Cx->CR2 &= ~(1 << I2C_CR2_ITEVTEN);
	    pI2Cx->CR2 &= ~(1 << I2C_CR2_ITBUFEN);
	    pI2Cx->CR2 &= ~(1 << I2C_CR2_ITERREN);

	}
}




//peripheral clock setup
void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi) {

	if (EnorDi == ENABLE) {

		if (pI2Cx == I2C1) {
			I2C1_PCLK_EN();
		} else if (pI2Cx == I2C2) {
			I2C2_PCLK_EN();
		} else if (pI2Cx == I2C3) {
			I2C3_PCLK_EN();
		}
	} else {

		if (pI2Cx == I2C1) {
			I2C1_PCLK_DI();
		} else if (pI2Cx == I2C2) {
			I2C2_PCLK_DI();
		} else if (pI2Cx == I2C3) {
			I2C3_PCLK_DI();
		}
	}

}

uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName) {

	if (pI2Cx->SR1 & FlagName) { // if this statement is true that mean the flag is set

		return FLAG_SET;
	}

	return FLAG_RESET;
}

void I2C_Init(I2C_Handle_t *pI2CHandle) {

	uint32_t tempreg = 0;

	//enable the clock for I2Cx peripheral
	I2C_PeriClockControl(pI2CHandle->pI2Cx, ENABLE);

	// ACK control bit
	tempreg |= pI2CHandle->I2C_Config.I2C_ACKControl << 10;
	pI2CHandle->pI2Cx->CR1 = tempreg;

	// configure FREQ fields in CR2
	tempreg = 0;
	tempreg = RCC_GetPCLK1Value() / 1000000U; //here we need just the number the clock is 16MHz we divided by 1*10^6 so the value is 16
	pI2CHandle->pI2Cx->CR2 = (tempreg & 0x3F); //here we musk other bits value 0x3F mean 111111 in binary

	//program the device own address
	tempreg = 0;
	tempreg |= (1 << 14);
	tempreg |= (pI2CHandle->I2C_Config.I2C_DeviceAddress << 1);
	pI2CHandle->pI2Cx->OAR1 = tempreg;

	//CCR calculation
	uint16_t ccr_value;
	tempreg = 0;
	if (pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SLC_SPEED_SM) {

		ccr_value = RCC_GetPCLK1Value()
						/ (2 * pI2CHandle->I2C_Config.I2C_SCLSpeed);
		//here we calculated the ccr value in SM the formula we can see in reference manual
		tempreg |= (ccr_value & 0xFFF); // here we interest in 12 bit so we masked them by 0xFFF which is 12 ones

	} else { // here we are in FM

		tempreg |= (1 << 15); // here we select FM
		tempreg |= (pI2CHandle->I2C_Config.I2C_FMDutyCycle << 14);// here to select duty cycle
		if (pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_2) {

			ccr_value = RCC_GetPCLK1Value()
							/ (3 * pI2CHandle->I2C_Config.I2C_SCLSpeed);

		} else { // 16 / 9 duty cycle

			ccr_value = RCC_GetPCLK1Value()
							/ (25 * pI2CHandle->I2C_Config.I2C_SCLSpeed);
		}
		tempreg |= (ccr_value & 0xFFF);

	}
	pI2CHandle->pI2Cx->CCR = tempreg;

	// TRISE configuration
	// the formula  [Fpclk * Trise(max)] + 1
	if (pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SLC_SPEED_SM) {

		// Trise in SM is 1000ns
		tempreg = ((RCC_GetPCLK1Value() / 1000000U)) + 1;

	} else { // here we are in FM

		// Trise in FM is 300ns
		tempreg = ((RCC_GetPCLK1Value() * 300 / 1000000000U)) + 1;
	}
	pI2CHandle->pI2Cx->TRISE = (tempreg & 0x3F); // we interest in first 6 bit so we masked the other by 3F

}

void I2C_DeInit(I2C_RegDef_t *pI2Cx){

	if (pI2Cx == I2C1) {
		I2C1_REG_RESET();
	} else if (pI2Cx == I2C2) {
		I2C2_REG_RESET();
	} else if (pI2Cx == I2C3) {
		I2C3_REG_RESET();
	}
}


void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi) {

	if (EnorDi == ENABLE) {

		pI2Cx->CR1 |= (1 << I2C_CR1_PE); // here we enabling the I2C peripheral, it will not work without enabling this bit

	} else {

		pI2Cx->CR1 &= ~(1 << I2C_CR1_PE);

	}
}

// sending and receiving data

void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxbuffer,uint32_t Len, uint8_t slaveAdrress, uint8_t sr) {

	// 1. generating START condition
	pI2CHandle->pI2Cx->CR1 |= (1 << I2C_CR1_START);


	// 2. confirm that start generation is completed by checking SB flag in the SR1
	while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB));  // this while holds when its value 1

	// 3. send the address of slave with r/nw bit set to w(0) total bits 8
	slaveAdrress = slaveAdrress << 1; // 7 bit slave address is moved by one bit for r/w
	slaveAdrress &= ~(1); // here we set the lsb to 0 to write the data
	pI2CHandle->pI2Cx->DR = slaveAdrress; // copy salve address + r/w bit to data register to transmit

	// 4. confirm that address phase is completed by checking the ADDR flag is the SR1
	while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR));

	//5. clearing ADDR flag
	// we should clear the flag after setting him so the ADDR flag clear by software reading SR1 register followed reading SR2
	I2C_ClearADDRFlag(pI2CHandle);

	// 6. send data until len becomes 0
	while (Len > 0) {

		while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE)); //wait until txe flag set to 1 that mean data register is empty

		pI2CHandle->pI2Cx->DR = *pTxbuffer;
		pTxbuffer++;
		Len--;
	}

	// 7. when len becomes 0 wait for TXE=1 and BTF=1 before generating stop condition
	//NOTE when TXE=1 , BTF=1  mean the both SR and DR registers are empty and next transmission should began
	//when BTE=1 the SCL will be stretched (pulled to low)

	while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE)); //wait until TXE flag set to 1
	while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_BTF)); //wait until BTF flag set to 1

	// 8. generating STOP condition
	if (sr == I2C_DISABLE_REAPETEDS) {
		pI2CHandle->pI2Cx->CR1 |= (1 << I2C_CR1_STOP);
	}
}

void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxbuffer,
		uint32_t Len, uint8_t slaveAdrress, uint8_t sr) {

	// 1. generating START condition
	pI2CHandle->pI2Cx->CR1 |= (1 << I2C_CR1_START);

	// 2. confirm that start generation is completed by checking SB flag in the SR1
	while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB));  // this while holds when its value 1

	// 3. send the address of slave with r/nw bit set to w(0) total bits 8
	slaveAdrress = slaveAdrress << 1; // 7 bit slave address is moved by one bit for r/w
	slaveAdrress |= 1; // here we set the lsb to 1 to read the data
	pI2CHandle->pI2Cx->DR = slaveAdrress; // copy salve address + r/w bit to data register to transmit

	// 4. confirm that address phase is completed by checking the ADDR flag is the SR1
	while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR));

	if (Len == 1) {

		// disable acking
		pI2CHandle->pI2Cx->CR1 &= ~(1 << I2C_CR1_ACK);

		// clearing ADDR flag
		// we should clear the flag after setting him so the ADDR flag clear by software reading SR1 register followed reading SR2
		I2C_ClearADDRFlag(pI2CHandle);

		// wait until RXNE becomes 1
		while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE));

		// generate STOP condition
		if (sr == I2C_DISABLE_REAPETEDS) {
			pI2CHandle->pI2Cx->CR1 |= (1 << I2C_CR1_STOP);
		}

		// read the data from the buffer
		*pRxbuffer = pI2CHandle->pI2Cx->DR;

	}

	if (Len > 1) {

		// clearing ADDR flag
		// we should clear the flag after setting him so the ADDR flag clear by software reading SR1 register followed reading SR2
		I2C_ClearADDRFlag(pI2CHandle);

		for (uint8_t i = Len; i > 0; i--) {

			// wait until RXNE becomes 1
			while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE));

			if (i == 2) { // if last 2 byte is remaining

				// disable acking
				pI2CHandle->pI2Cx->CR1 &= ~(1 << I2C_CR1_ACK);

				// generate STOP condition
				if (sr == I2C_DISABLE_REAPETEDS) {
					pI2CHandle->pI2Cx->CR1 |= (1 << I2C_CR1_STOP);
				}

			}

			// read the data from the buffer
			*pRxbuffer = pI2CHandle->pI2Cx->DR;
			pRxbuffer++;
		}

	}

	//re-enable Acking
	if (pI2CHandle->I2C_Config.I2C_ACKControl == I2C_ACK_ENABLE) {
		pI2CHandle->pI2Cx->CR1 |= (1 << I2C_CR1_ACK);
	}

}

uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pTxbuffer,
		uint32_t Len, uint8_t slaveAdrress, uint8_t sr) {

	uint8_t busystate = pI2CHandle->TxRxState;

	if ((busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX)) {
		pI2CHandle->pTxBuffer = pTxbuffer;
		pI2CHandle->TxLen = Len;
		pI2CHandle->TxRxState = I2C_BUSY_IN_TX;
		pI2CHandle->SlaveAddr = slaveAdrress;
		pI2CHandle->Sr = sr;

		//Implement code to Generate START Condition
		pI2CHandle->pI2Cx->CR1 |= (1 << I2C_CR1_START);

		//Implement the code to enable ITBUFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITBUFEN);

		//Implement the code to enable ITEVFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITEVTEN);

		//Implement the code to enable ITERREN Control Bit
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITERREN);

	}

	return busystate;

}

uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pRxbuffer,
		uint32_t Len, uint8_t slaveAdrress, uint8_t sr) {

	uint8_t busystate = pI2CHandle->TxRxState;

	if ((busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX)) {
		pI2CHandle->pRxBuffer = pRxbuffer;
		pI2CHandle->RxLen = Len;
		pI2CHandle->TxRxState = I2C_BUSY_IN_RX;
		pI2CHandle->RxSize = Len; //Rxsize is used in the ISR code to manage the data reception
		pI2CHandle->SlaveAddr = slaveAdrress;
		pI2CHandle->Sr = sr;

		//Implement code to Generate START Condition
		pI2CHandle->pI2Cx->CR1 |= (1 << I2C_CR1_START);

		//Implement the code to enable ITBUFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITBUFEN);

		//Implement the code to enable ITEVFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITEVTEN);

		//Implement the code to enable ITERREN Control Bit
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITERREN);

	}

	return busystate;
}

// IRQ configuration and ISR handling






void I2C_SlaveSendData(I2C_RegDef_t *pI2Cx , uint8_t data){

	pI2Cx->DR = data;
}

uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *pI2Cx){

	return (uint8_t)pI2Cx->DR;
}





void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi) {

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

void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority) {

	uint8_t iprx = IRQNumber / 4; // we divide by 4 because each registers take 4 IRQ Num
	uint8_t iprx_section = IRQNumber % 4;
	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLIMENTED); // here multiply by 8 because each IRQ num has 8 bit
	/*(8 - NO_PR_BITS_IMPLIMENTED) this is because in the 8 bits the implementation
	 just be in the higher 4 bits  for example 0x00001000 this value will be read 0000 so
	 we do shifting by 4 and the value be 1000  */
	*(NVIC_PR_BASE_ADDR + iprx) |= (IRQPriority << shift_amount);


}



void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle) {

	//Interrupt handling for both master and slave mode of a device

	uint8_t temp1, temp2, temp3;
	temp1 = pI2CHandle->pI2Cx->CR2 & (1 << I2C_CR2_ITEVTEN);
	temp2 = pI2CHandle->pI2Cx->CR2 & (1 << I2C_CR2_ITBUFEN);

	//1. Handle For interrupt generated by SB event
	//	Note : SB flag is only applicable in Master mode
	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_SB);

	if (temp1 && temp3) {

		// SB flag is set
		if (pI2CHandle->TxRxState == I2C_BUSY_IN_TX) {

			pI2CHandle->SlaveAddr = pI2CHandle->SlaveAddr << 1; // 7 bit slave address is moved by one bit for r/w
			pI2CHandle->SlaveAddr &= ~(0); // here we set the lsb to 0 to write the data
			pI2CHandle->pI2Cx->DR = pI2CHandle->SlaveAddr; // copy salve address + r/w bit to data register to transmit

		} else if (pI2CHandle->TxRxState == I2C_BUSY_IN_RX) {

			pI2CHandle->SlaveAddr = pI2CHandle->SlaveAddr << 1; // 7 bit slave address is moved by one bit for r/w
			pI2CHandle->SlaveAddr |= 0; // here we set the lsb to 1 to read the data
			pI2CHandle->pI2Cx->DR = pI2CHandle->SlaveAddr; // copy salve address + r/w bit to data register to transmit
		}
	}

	//2. Handle For interrupt generated by ADDR event
	//Note : When master mode  : Address is sent
	//		 When Slave mode   : Address matched with own address
	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_ADDR);

	if (temp1 && temp3) {

		// ADDR flag is set
		// clearing ADDR flag
		// we should clear the flag after setting him so the ADDR flag clear by software reading SR1 register followed reading SR2
		I2C_ClearADDRFlag(pI2CHandle);

	}

	//3. Handle For interrupt generated by BTF(Byte Transfer Finished) event
	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_BTF);

	if (temp1 && temp3) {

		// BTF flag is set
		if (pI2CHandle->TxRxState == I2C_BUSY_IN_TX) {

			//make sure TXE is also set
			if (pI2CHandle->pI2Cx->SR1 == I2C_SR1_TxE) {

				if (pI2CHandle->TxLen == 0) {

					// BTF = 1 and TXE = 1

					//1. generate stop condition
					if (pI2CHandle->Sr == I2C_DISABLE_REAPETEDS) {
						pI2CHandle->pI2Cx->CR1 |= (1 << I2C_CR1_STOP);
					}

					//2. reset all the member elements of the handle structure
					I2C_CloseSendData(pI2CHandle);

					//3. notify the application about transmission complete
					I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_TX_CMPLT);
				}
			}

		} else if (pI2CHandle->TxRxState == I2C_BUSY_IN_RX) {

			; //nothing to do
		}
	}

	//4. Handle For interrupt generated by STOPF event
	// Note : Stop detection flag is applicable only slave mode . For master this flag will never be set

	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_STOPF);

	if (temp1 && temp3) {

		// STOPF flag is set
		//Cleared by software reading the SR1 register followed by a write in the CR1 register,
		// above we read the SR1
		//now we will writhe some data to CR1
		pI2CHandle->pI2Cx->CR1 |= 0x0000; // we writhe some data and this data will not effect to the values of CR1

		// notify the application the STOP condition is detected
		I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_STOP);
	}

	//5. Handle For interrupt generated by TXE event
	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_TxE);

	if (temp1 && temp2 && temp3) {

		// TXE flag is set
		//check for device mode
		if (pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL)) {

			//we have to transmit the data
			if (pI2CHandle->TxRxState == I2C_BUSY_IN_TX) {

				I2C_MasterHandleTXEInterrupt(pI2CHandle);

			}
		}else{ // device in slave mode

			// make sure that the slave is in transitter mode
			if (pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_TRA)){

				I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_DATA_REQ);
			}
		}
	}

	//6. Handle For interrupt generated by RXNE event
	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_RxNE);

	if (temp1 && temp2 && temp3) {

		// RXNE flag is set
		if (pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL)) {
			if (pI2CHandle->TxRxState == I2C_BUSY_IN_RX) {

				I2C_MasterHandleRXNEInterrupt(pI2CHandle);

			}
		}else{ // device in slave mode

			// make sure that the slave is in receiver mode
			if(!(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_TRA))){

				I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_DATA_RCV);

			}
		}
	}
}

void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle)
{

	uint32_t temp1,temp2;

    //Know the status of  ITERREN control bit in the CR2
	temp2 = (pI2CHandle->pI2Cx->CR2) & ( 1 << I2C_CR2_ITERREN);


/***********************Check for Bus error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1<< I2C_SR1_BERR);
	if(temp1  && temp2 )
	{
		//This is Bus error

		//Implement the code to clear the buss error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_BERR);

		//Implement the code to notify the application about the error
	   I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_BERR);
	}

/***********************Check for arbitration lost error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_ARLO );
	if(temp1  && temp2)
	{
		//This is arbitration lost error

		//Implement the code to clear the arbitration lost error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_ARLO);

		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_ARLO);

	}

/***********************Check for ACK failure  error************************************/

	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_AF);
	if(temp1  && temp2)
	{
		//This is ACK failure error

	    //Implement the code to clear the ACK failure error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_AF);

		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_AF);

	}

/***********************Check for Overrun/underrun error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_OVR);
	if(temp1  && temp2)
	{
		//This is Overrun/underrun

	    //Implement the code to clear the Overrun/underrun error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_OVR);


		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_OVR);

	}

/***********************Check for Time out error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_TIMEOUT);
	if(temp1  && temp2)
	{
		//This is Time out error

	    //Implement the code to clear the Time out error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_TIMEOUT);


		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_TIMEOUT);

	}

}


