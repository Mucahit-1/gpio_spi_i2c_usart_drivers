/*
 * stm32f407xx_gpio_driver.c
 *
 *  Created on: Jun 29, 2023
 *      Author: mucahit
 */
#include "stm32f407xx_gpio_driver.h"

//peripheral clock setup
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi) {

	if (EnorDi == ENABLE) {

		if (pGPIOx == GPIOA) {
			GPIOA_PCLK_EN();
		} else if (pGPIOx == GPIOB) {
			GPIOB_PCLK_EN();
		} else if (pGPIOx == GPIOC) {
			GPIOC_PCLK_EN();
		} else if (pGPIOx == GPIOD) {
			GPIOD_PCLK_EN();
		} else if (pGPIOx == GPIOE) {
			GPIOE_PCLK_EN();
		} else if (pGPIOx == GPIOF) {
			GPIOF_PCLK_EN();
		} else if (pGPIOx == GPIOG) {
			GPIOG_PCLK_EN();
		} else if (pGPIOx == GPIOH) {
			GPIOH_PCLK_EN();
		} else if (pGPIOx == GPIOI) {
			GPIOI_PCLK_EN();
		} else if (pGPIOx == GPIOJ) {
			GPIOJ_PCLK_EN();
		} else if (pGPIOx == GPIOK) {
			GPIOK_PCLK_EN();
		}
	} else {

		if (pGPIOx == GPIOA) {
			GPIOA_PCLK_DI();
		} else if (pGPIOx == GPIOB) {
			GPIOB_PCLK_DI();
		} else if (pGPIOx == GPIOC) {
			GPIOC_PCLK_DI();
		} else if (pGPIOx == GPIOD) {
			GPIOD_PCLK_DI();
		} else if (pGPIOx == GPIOE) {
			GPIOE_PCLK_DI();
		} else if (pGPIOx == GPIOF) {
			GPIOF_PCLK_DI();
		} else if (pGPIOx == GPIOG) {
			GPIOG_PCLK_DI();
		} else if (pGPIOx == GPIOH) {
			GPIOH_PCLK_DI();
		} else if (pGPIOx == GPIOI) {
			GPIOI_PCLK_DI();
		} else if (pGPIOx == GPIOJ) {
			GPIOJ_PCLK_DI();
		} else if (pGPIOx == GPIOK) {
			GPIOK_PCLK_DI();
		}
	}

}

//init and de-init
void GPIO_Init(GPIO_Handle_t *pGPIOHandle) {

	//enabling the port clock
	GPIO_PeriClockControl(pGPIOHandle->pGPIOx, ENABLE);

	uint32_t temp = 0;
	//1. configure pin mode
	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG) {
		// 2* because to bits for each pin
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode
				<< (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		//before setting the value we must clearing we should write 00 to clear so we write ~11 which mean 3
		pGPIOHandle->pGPIOx->MODER &= ~(0x3
				<< pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //clearing
		pGPIOHandle->pGPIOx->MODER |= temp; // here we set the value of temp in the register
	}

	else {

		if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT) {

			//setting falling edge
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			//clearing raising edge
			EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		} else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT) {

			//setting raising edge
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			//clearing falling edge
			EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		} else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FR) {

			//setting falling edge
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			//setting raising edge
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}

		//configure the GPIO port selection in SYSCFG_EXTICR
		uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
		uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;
		uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
		SYSCFG_PCLK_EN();
		SYSCFG->EXTICR[temp1] |= portcode << (4 * temp2);

		//enable the EXTI interrupt delivery using IMR
		EXTI->IMR |= 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber;

	}

	temp = 0;
	//2. configure pin speed
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed
			<< (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3
			<< pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //clearing
	pGPIOHandle->pGPIOx->OSPEEDR |= temp; //setting

	temp = 0;
	//3. configure pin pu pd
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl
			<< (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3
			<< pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //clearing
	pGPIOHandle->pGPIOx->PUPDR |= temp; //setting

	temp = 0;
	//4. configure pin output type
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType
			<< pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x1
			<< pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //clearing
	pGPIOHandle->pGPIOx->OTYPER |= temp; //setting

	//5. configure ALT FUN
	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN) {

		uint8_t temp1, temp2;
		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;
		pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << (4 * temp2)); //clearing
		pGPIOHandle->pGPIOx->AFR[temp1] |=
				(pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * temp2)); //setting
	}
}

void GPIO_DeInit(GPIO_RegDef_t *pGPIOx) {

	if (pGPIOx == GPIOA) {
		GPIOA_REG_RESET();
	} else if (pGPIOx == GPIOB) {
		GPIOB_REG_RESET();
	} else if (pGPIOx == GPIOC) {
		GPIOC_REG_RESET();
	} else if (pGPIOx == GPIOD) {
		GPIOD_REG_RESET();
	} else if (pGPIOx == GPIOE) {
		GPIOE_REG_RESET();
	} else if (pGPIOx == GPIOF) {
		GPIOF_REG_RESET();
	} else if (pGPIOx == GPIOG) {
		GPIOG_REG_RESET();
	} else if (pGPIOx == GPIOH) {
		GPIOH_REG_RESET();
	} else if (pGPIOx == GPIOI) {
		GPIOI_REG_RESET();
	} else if (pGPIOx == GPIOJ) {
		GPIOJ_REG_RESET();
	} else if (pGPIOx == GPIOK) {
		GPIOK_REG_RESET();
	}
}

//data read and write
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber) {
	// here we shift the pin number in register to the lsb and musk the register by
	uint8_t value;
	value = (uint8_t) ((pGPIOx->IDR >> PinNumber) & 0x00000001);
	return value;

}

uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx) {

	uint16_t value;
	value = (uint16_t) pGPIOx->IDR;
	return value;

}

void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber,
		uint8_t value) {

	if (value == GPIO_PIN_SET) { // this mean if the value 1

		pGPIOx->ODR |= (1 << PinNumber); // setting the output value 1

	} else {

		pGPIOx->ODR &= ~(1 << PinNumber); //here we clear the bit same as we setting it 0

	}

}

void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t value) {

	pGPIOx->ODR = value;

}

void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber) {

	pGPIOx->ODR ^= (1 << PinNumber); // هاد القسم مافهمتو بعدين حدور عشرحو

}

//IRQ configuration and ISR handling
// this configuration is in the processor side so we check the processor registers from data sheet of processor
// in my board the processor is cortex M4
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi) {

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

void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority) {

	uint8_t iprx = IRQNumber / 4; // we divide by 4 because each registers take 4 IRQ Num
	uint8_t iprx_section = IRQNumber % 4;
	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLIMENTED); // here multiply by 8 because each IRQ num has 8 bit
	/*(8 - NO_PR_BITS_IMPLIMENTED) this is because in the 8 bits the implementation
	 just be in the higher 4 bits  for example 0x00001000 this value will be read 0000 so
	 we do shifting by 4 and the value be 1000  */
	*(NVIC_PR_BASE_ADDR + iprx) |= (IRQPriority << shift_amount);

}

void GPIO_IRQHandling(uint8_t PinNumber) { // هاد القسم ماكتير فهمتو بشوفو بعدين شرحو بالفيديو114

	//clear the EXTI PR register
	if (EXTI->PR & (1 << PinNumber)) {

		//clear
		EXTI->PR |= (1 << PinNumber);
	}

}
