/*
 * stm32f407xx.h
 *
 *  Created on: Jun 27, 2023
 *      Author: riyad
 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

#include <stddef.h>
#include <stdint.h>

#define  __vo volatile
#define __weak __attribute__((weak))

/********************** PROCESSOR SPECIFIC DETAILS *******************************************/
// ARM cortex Mx NVIC ISERx register addresses ISER interrupt set enable register

#define NVIC_ISER0       ((__vo uint32_t*)0xE000E100)
#define NVIC_ISER1       ((__vo uint32_t*)0xE000E104)
#define NVIC_ISER2       ((__vo uint32_t*)0xE000E108)
#define NVIC_ISER3       ((__vo uint32_t*)0xE000E10C)

// ARM cortex Mx NVIC ICERx register addresses ICER interrupt clear enable register

#define NVIC_ICER0       ((__vo uint32_t*)0xE000E180)
#define NVIC_ICER1       ((__vo uint32_t*)0xE000E184)
#define NVIC_ICER2       ((__vo uint32_t*)0xE000E188)
#define NVIC_ICER3       ((__vo uint32_t*)0xE000E18C)

// ARM cortex Mx priority register addresses

#define NVIC_PR_BASE_ADDR    ((__vo uint32_t*)0xE000E400)

#define NO_PR_BITS_IMPLIMENTED            4


/************************************************************************************************/

// defining base addresses of SRAM and flash memories of MCU


#define FLASH_BASEADDR  0x08000000U  // main memory
#define SRAM1_BASEADDR  0x20000000U  // 112KB
#define SRAM2_BASEADDR  0x20001C00U  // 16KB
#define ROM_BASEADDR    0x1FFF0000U  //system memory

// defining base addresses of AHBx and APBx buses

#define APB1_BASEADDR   0x40000000U  // peripheral base address
#define APB2_BASEADDR   0x40010000U
#define AHB1_BASEADDR   0x40020000U
#define AHB2_BASEADDR   0x50000000U




// defining base addresses of peripheral which are hanging on AHB1 bus

#define GPIOA_BASEADDR   (AHB1_BASEADDR + 0x0000) // base address + offset
#define GPIOB_BASEADDR   (AHB1_BASEADDR + 0x0400)
#define GPIOC_BASEADDR   (AHB1_BASEADDR + 0x0800)
#define GPIOD_BASEADDR   (AHB1_BASEADDR + 0x0C00)
#define GPIOE_BASEADDR   (AHB1_BASEADDR + 0x1000)
#define GPIOF_BASEADDR   (AHB1_BASEADDR + 0x1400)
#define GPIOG_BASEADDR   (AHB1_BASEADDR + 0x1800)
#define GPIOH_BASEADDR   (AHB1_BASEADDR + 0x1C00)
#define GPIOI_BASEADDR   (AHB1_BASEADDR + 0x2000)
#define GPIOJ_BASEADDR   (AHB1_BASEADDR + 0x2400)
#define GPIOK_BASEADDR   (AHB1_BASEADDR + 0x2800)
#define RCC_BASEADDR     (AHB1_BASEADDR + 0x3800)

// defining base addresses of peripheral which are hanging on APB1 bus

#define I2C1_BASEADDR     (APB1_BASEADDR + 0x5400)
#define I2C2_BASEADDR     (APB1_BASEADDR + 0x5800)
#define I2C3_BASEADDR     (APB1_BASEADDR + 0x5C00)

#define SPI2_BASEADDR     (APB1_BASEADDR + 0x3800)
#define SPI3_BASEADDR     (APB1_BASEADDR + 0x3C00)

#define USART2_BASEADDR   (APB1_BASEADDR + 0x4400)
#define USART3_BASEADDR   (APB1_BASEADDR + 0x4800)
#define UART4_BASEADDR    (APB1_BASEADDR + 0x4C00)
#define UART5_BASEADDR    (APB1_BASEADDR + 0x5000)

// defining base addresses of peripheral which are hanging on APB2 bus

#define EXTI_BASEADDR      (APB2_BASEADDR + 0x3C00)
#define SPI1_BASEADDR      (APB2_BASEADDR + 0x3000)
#define SPI4_BASEADDR      (APB2_BASEADDR + 0X3400)

#define SYSCFG_BASEADDR    (APB2_BASEADDR + 0x3800)
#define USART1_BASEADDR    (APB2_BASEADDR + 0x1000)
#define USART6_BASEADDR    (APB2_BASEADDR + 0x1400)


/*****************************peripheral registers definition structures***********************************/

//instead of define each register separately we create a structure data type
// we use volatile because the values of these registers are frequently change

typedef struct{

	__vo uint32_t MODER;         //GPIO port mode register
	__vo uint32_t OTYPER;        //GPIO port output type register
	__vo uint32_t OSPEEDR;       //GPIO port output speed register
	__vo uint32_t PUPDR;         //GPIO port pull-up/pull-down register
	__vo uint32_t IDR;           //GPIO port input data register
	__vo uint32_t ODR;           //GPIO port output data register
	__vo uint32_t BSRR;          //GPIO port bit set/reset register
	__vo uint32_t LCKR;          //GPIO port configuration lock register
	__vo uint32_t AFR[2];          //GPIO alternate function low and high registers
}GPIO_RegDef_t;

typedef struct{

	__vo uint32_t CR;                  //RCC clock control register
	__vo uint32_t PLLCFGR;             //RCC PLL configuration register
	__vo uint32_t CFGR;                //RCC clock configuration register
	__vo uint32_t CIR;                 //RCC clock interrupt register
	__vo uint32_t AHB1RSTR;            //RCC AHB1 peripheral reset register
	__vo uint32_t AHB2RSTR;            //RCC AHB2 peripheral reset register
	__vo uint32_t AHB3RSTR;            //RCC AHB3 peripheral reset register
	uint32_t RESERVED0;            //RESERVED
	__vo uint32_t APB1RSTR;            //RCC APB1 peripheral reset register
	__vo uint32_t APB2RSTR;            //RCC APB2 peripheral reset register
	uint32_t RESERVED1[2];         //RESERVED
	__vo uint32_t AHB1ENR;             //RCC AHB1 peripheral clock enable register
	__vo uint32_t AHB2ENR;             //RCC AHB2 peripheral clock enable register
	__vo uint32_t AHB3ENR;             //RCC AHB3 peripheral clock enable register
	uint32_t RESERVED2;            //RESERVED
	__vo uint32_t APB1ENR;             //RCC APB1 peripheral clock enable register
	__vo uint32_t APB2ENR;             //RCC APB2 peripheral clock enable register
	uint32_t RESERVED3[2];         //RESERVED
	__vo uint32_t AHB1LPENR;           //RCC AHB1 peripheral clock enable in low power mode register
	__vo uint32_t AHB2LPENR;           //RCC AHB2 peripheral clock enable in low power mode register
	__vo uint32_t AHB3LPENR;           //RCC AHB3 peripheral clock enable in low power mode register
	uint32_t RESERVED4;            //RESERVED
	__vo uint32_t APB1LPENR;           //RCC APB1 peripheral clock enable in low power mode register
	__vo uint32_t APB2LPENR;           //RCC APB2 peripheral clock enable in low power mode register
	uint32_t RESERVED5[2];         //RESERVED
	__vo uint32_t BDCR;                //RCC Backup domain control register
	__vo uint32_t CSR;                 //RCC clock control & status register
	uint32_t RESERVED6[2];         //RESERVED
	__vo uint32_t SSCGR;               //RCC spread spectrum clock generation register
	__vo uint32_t PLLI2SCFGR;          //RCC PLLI2S configuration register
	__vo uint32_t PLLSAICFGR;          //RCC PLL configuration register
	__vo uint32_t DCKCFGR;             //RCC Dedicated Clock Configuration Register

}RCC_RegDef_t;



typedef struct{

	__vo uint32_t IMR;
	__vo uint32_t EMR;
	__vo uint32_t RTSR;
	__vo uint32_t FTSR;
	__vo uint32_t SWIER;
	__vo uint32_t PR;

}EXTI_RegDef_t;





typedef struct{

	__vo uint32_t MEMRMP;
	__vo uint32_t PMC;
	__vo uint32_t EXTICR[4];
	uint32_t RESERVED1[2];
	__vo uint32_t CMPCR;
	uint32_t RESERVED2[2];
	__vo uint32_t CFGR;

}SYSCFG_RegDef_t;


// SPI register definition
typedef struct{

	__vo uint32_t CR1;             //SPI control register 1
	__vo uint32_t CR2;             //SPI control register 2
	__vo uint32_t SR;              //SPI status register
	__vo uint32_t DR;              //SPI data register
	__vo uint32_t CRCPR;           //SPI CRC polynomial register
	__vo uint32_t RXCRCR;          //SPI RX CRC register
	__vo uint32_t TXCRCR;          //SPI TX CRC register
	__vo uint32_t I2SCFGR;         //SPI_I2S configuration register
	__vo uint32_t I2SPR;           //SPI_I2S prescaler register

}SPI_RegDef_t;


// I2C register definition
typedef struct{

	__vo uint32_t CR1;                //I2C Control register 1
	__vo uint32_t CR2;                //I2C Control register 2
	__vo uint32_t OAR1;               //I2C Own address register 1
	__vo uint32_t OAR2;               //I2C Own address register 2
	__vo uint32_t DR;                 //I2C Data register
	__vo uint32_t SR1;                //I2C Status register 1
	__vo uint32_t SR2;                //I2C Status register 2
	__vo uint32_t CCR;                //I2C Clock control register
	__vo uint32_t TRISE;              //I2C TRISE register
	__vo uint32_t FLTR;               //I2C FLTR register

}I2C_RegDef_t;


// USART register definition
typedef struct{

	__vo uint32_t SR;                //USART Status register
	__vo uint32_t DR;                //USART Data register
	__vo uint32_t BRR;               //USART Baud rate register
	__vo uint32_t CR1;               //USART Control register 1
	__vo uint32_t CR2;               //USART Control register 2
	__vo uint32_t CR3;               //USART Control register 3
	__vo uint32_t GTPR;              //Guard time and prescaler register

}USART_RegDef_t;





//peripheral definitions

#define GPIOA     ((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB     ((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC     ((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD     ((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE     ((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF     ((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG     ((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define GPIOH     ((GPIO_RegDef_t*)GPIOH_BASEADDR)
#define GPIOI     ((GPIO_RegDef_t*)GPIOI_BASEADDR)
#define GPIOJ     ((GPIO_RegDef_t*)GPIOJ_BASEADDR)
#define GPIOK     ((GPIO_RegDef_t*)GPIOK_BASEADDR)

#define RCC       ((RCC_RegDef_t*)RCC_BASEADDR)
#define EXTI      ((EXTI_RegDef_t*)EXTI_BASEADDR)
#define SYSCFG    ((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)

#define SPI1       ((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI2       ((SPI_RegDef_t*)SPI2_BASEADDR)
#define SPI3       ((SPI_RegDef_t*)SPI3_BASEADDR)
#define SPI4       ((SPI_RegDef_t*)SPI4_BASEADDR)

#define I2C1       ((I2C_RegDef_t*)I2C1_BASEADDR)
#define I2C2       ((I2C_RegDef_t*)I2C2_BASEADDR)
#define I2C3       ((I2C_RegDef_t*)I2C3_BASEADDR)

#define USART1     ((USART_RegDef_t*)USART1_BASEADDR)
#define USART2     ((USART_RegDef_t*)USART2_BASEADDR)
#define USART3     ((USART_RegDef_t*)USART3_BASEADDR)
#define UART4      ((USART_RegDef_t*)UART4_BASEADDR)
#define UART5      ((USART_RegDef_t*)UART5_BASEADDR)
#define USART6     ((USART_RegDef_t*)USART6_BASEADDR)





//clock enable for GPIOx peripherals

#define GPIOA_PCLK_EN()    (RCC->AHB1ENR |= ( 1 << 0 ))
#define GPIOB_PCLK_EN()    (RCC->AHB1ENR |= ( 1 << 1 ))
#define GPIOC_PCLK_EN()    (RCC->AHB1ENR |= ( 1 << 2 ))
#define GPIOD_PCLK_EN()    (RCC->AHB1ENR |= ( 1 << 3 ))
#define GPIOE_PCLK_EN()    (RCC->AHB1ENR |= ( 1 << 4 ))
#define GPIOF_PCLK_EN()    (RCC->AHB1ENR |= ( 1 << 5 ))
#define GPIOG_PCLK_EN()    (RCC->AHB1ENR |= ( 1 << 6 ))
#define GPIOH_PCLK_EN()    (RCC->AHB1ENR |= ( 1 << 7 ))
#define GPIOI_PCLK_EN()    (RCC->AHB1ENR |= ( 1 << 8 ))
#define GPIOJ_PCLK_EN()    (RCC->AHB1ENR |= ( 1 << 9 ))
#define GPIOK_PCLK_EN()    (RCC->AHB1ENR |= ( 1 << 10 ))



//clock enable for I2Cx peripherals

#define I2C1_PCLK_EN()    (RCC->APB1ENR |= ( 1 << 21 ))
#define I2C2_PCLK_EN()    (RCC->APB1ENR |= ( 1 << 22 ))
#define I2C3_PCLK_EN()    (RCC->APB1ENR |= ( 1 << 23 ))



//clock enable for SPIx peripherals

#define SPI1_PCLK_EN()    (RCC->APB2ENR |= ( 1 << 12 ))
#define SPI2_PCLK_EN()    (RCC->APB1ENR |= ( 1 << 14 ))
#define SPI3_PCLK_EN()    (RCC->APB1ENR |= ( 1 << 15 ))
#define SPI4_PCLK_EN()    (RCC->APB2ENR |= ( 1 << 13 ))



//clock enable for USARTx peripherals

#define USART1_PCLK_EN()    (RCC->APB2ENR |= ( 1 << 4 ))
#define USART6_PCLK_EN()    (RCC->APB2ENR |= ( 1 << 5 ))
#define USART2_PCLK_EN()    (RCC->APB1ENR |= ( 1 << 17 ))
#define USART3_PCLK_EN()    (RCC->APB1ENR |= ( 1 << 18 ))
#define UART4_PCLK_EN()    (RCC->APB1ENR |= ( 1 << 19 ))
#define UART5_PCLK_EN()    (RCC->APB1ENR |= ( 1 << 20 ))



//clock enable for SYSCFG peripheral

#define SYSCFG_PCLK_EN()    (RCC->APB2ENR |= ( 1 << 14 ))





//clock disable for GPIOx peripherals

#define GPIOA_PCLK_DI()    (RCC->AHB1ENR &= ~( 1 << 0 ))   // in embedded world we use |= to set value and &= to reset it
#define GPIOB_PCLK_DI()    (RCC->AHB1ENR &= ~( 1 << 1 ))
#define GPIOC_PCLK_DI()    (RCC->AHB1ENR &= ~( 1 << 2 ))
#define GPIOD_PCLK_DI()    (RCC->AHB1ENR &= ~( 1 << 3 ))
#define GPIOE_PCLK_DI()    (RCC->AHB1ENR &= ~( 1 << 4 ))
#define GPIOF_PCLK_DI()    (RCC->AHB1ENR &= ~( 1 << 5 ))
#define GPIOG_PCLK_DI()    (RCC->AHB1ENR &= ~( 1 << 6 ))
#define GPIOH_PCLK_DI()    (RCC->AHB1ENR &= ~( 1 << 7 ))
#define GPIOI_PCLK_DI()    (RCC->AHB1ENR &= ~( 1 << 8 ))
#define GPIOJ_PCLK_DI()    (RCC->AHB1ENR &= ~( 1 << 9 ))
#define GPIOK_PCLK_DI()    (RCC->AHB1ENR &= ~( 1 << 10 ))



//clock disable for I2Cx peripherals

#define I2C1_PCLK_DI()    (RCC->APB1ENR &= ~( 1 << 21 ))
#define I2C2_PCLK_DI()    (RCC->APB1ENR &= ~( 1 << 22 ))
#define I2C3_PCLK_DI()    (RCC->APB1ENR &= ~( 1 << 23 ))



//clock disable for SPIx peripherals

#define SPI1_PCLK_DI()    (RCC->APB2ENR &= ~( 1 << 12 ))
#define SPI2_PCLK_DI()    (RCC->APB1ENR &= ~( 1 << 14 ))
#define SPI3_PCLK_DI()    (RCC->APB1ENR &= ~( 1 << 15 ))
#define SPI4_PCLK_DI()    (RCC->APB2ENR &= ~( 1 << 13 ))



//clock disable for USARTx peripherals

#define USART1_PCLK_DI()    (RCC->APB2ENR &= ~( 1 << 4 ))
#define USART6_PCLK_DI()    (RCC->APB2ENR &= ~( 1 << 5 ))
#define USART2_PCLK_DI()    (RCC->APB1ENR &= ~( 1 << 17 ))
#define USART3_PCLK_DI()    (RCC->APB1ENR &= ~( 1 << 18 ))
#define UART4_PCLK_DI()    (RCC->APB1ENR &= ~( 1 << 19 ))
#define UART5_PCLK_DI()    (RCC->APB1ENR &= ~( 1 << 20 ))



//clock disable for SYSCFG peripheral

#define SYSCFG_PCLK_DI()    (RCC->APB2ENR &= ~( 1 << 14 ))



// macros to reset GPIOx peripheral
// in this code we set the value 1 to reset and change it to 0 it should not still 1
// in c programming to write to statement in 1 macro we use do while
#define   GPIOA_REG_RESET()     do{(RCC->AHB1RSTR |= ( 1 << 0 ));(RCC->AHB1RSTR &= ~( 1 << 0 ));}while(0)
#define   GPIOB_REG_RESET()     do{(RCC->AHB1RSTR |= ( 1 << 1 ));(RCC->AHB1RSTR &= ~( 1 << 1 ));}while(0)
#define   GPIOC_REG_RESET()     do{(RCC->AHB1RSTR |= ( 1 << 2 ));(RCC->AHB1RSTR &= ~( 1 << 2 ));}while(0)
#define   GPIOD_REG_RESET()     do{(RCC->AHB1RSTR |= ( 1 << 3 ));(RCC->AHB1RSTR &= ~( 1 << 3 ));}while(0)
#define   GPIOE_REG_RESET()     do{(RCC->AHB1RSTR |= ( 1 << 4 ));(RCC->AHB1RSTR &= ~( 1 << 4 ));}while(0)
#define   GPIOF_REG_RESET()     do{(RCC->AHB1RSTR |= ( 1 << 5 ));(RCC->AHB1RSTR &= ~( 1 << 5 ));}while(0)
#define   GPIOG_REG_RESET()     do{(RCC->AHB1RSTR |= ( 1 << 6 ));(RCC->AHB1RSTR &= ~( 1 << 6 ));}while(0)
#define   GPIOH_REG_RESET()     do{(RCC->AHB1RSTR |= ( 1 << 7 ));(RCC->AHB1RSTR &= ~( 1 << 7 ));}while(0)
#define   GPIOI_REG_RESET()     do{(RCC->AHB1RSTR |= ( 1 << 8 ));(RCC->AHB1RSTR &= ~( 1 << 8 ));}while(0)
#define   GPIOJ_REG_RESET()     do{(RCC->AHB1RSTR |= ( 1 << 9 ));(RCC->AHB1RSTR &= ~( 1 << 9 ));}while(0)
#define   GPIOK_REG_RESET()     do{(RCC->AHB1RSTR |= ( 1 << 10 ));(RCC->AHB1RSTR &= ~( 1 << 10 ));}while(0)

#define   SPI1_REG_RESET()      do{(RCC->APB2RSTR |= ( 1 << 12 ));(RCC->APB1RSTR &= ~( 1 << 12 ));}while(0)
#define   SPI2_REG_RESET()      do{(RCC->APB1RSTR |= ( 1 << 14 ));(RCC->APB1RSTR &= ~( 1 << 14 ));}while(0)
#define   SPI3_REG_RESET()      do{(RCC->APB1RSTR |= ( 1 << 15 ));(RCC->APB1RSTR &= ~( 1 << 15 ));}while(0)
#define   SPI4_REG_RESET()      do{(RCC->APB2RSTR |= ( 1 << 13 ));(RCC->APB1RSTR &= ~( 1 << 13 ));}while(0)

#define   I2C1_REG_RESET()      do{(RCC->APB2RSTR |= ( 1 << 21 ));(RCC->APB1RSTR &= ~( 1 << 21 ));}while(0)
#define   I2C2_REG_RESET()      do{(RCC->APB1RSTR |= ( 1 << 22 ));(RCC->APB1RSTR &= ~( 1 << 22 ));}while(0)
#define   I2C3_REG_RESET()      do{(RCC->APB1RSTR |= ( 1 << 23 ));(RCC->APB1RSTR &= ~( 1 << 23 ));}while(0)

#define   USART1_REG_RESET()      do{(RCC->APB2RSTR |= ( 1 << 4 ));(RCC->APB1RSTR &= ~( 1 << 4 ));}while(0)
#define   USART2_REG_RESET()      do{(RCC->APB1RSTR |= ( 1 << 5 ));(RCC->APB1RSTR &= ~( 1 << 5 ));}while(0)
#define   USART3_REG_RESET()      do{(RCC->APB1RSTR |= ( 1 << 17 ));(RCC->APB1RSTR &= ~( 1 << 17 ));}while(0)
#define   UART4_REG_RESET()       do{(RCC->APB2RSTR |= ( 1 << 18 ));(RCC->APB1RSTR &= ~( 1 << 18 ));}while(0)
#define   UART5_REG_RESET()       do{(RCC->APB1RSTR |= ( 1 << 19 ));(RCC->APB1RSTR &= ~( 1 << 19 ));}while(0)
#define   USART6_REG_RESET()      do{(RCC->APB2RSTR |= ( 1 << 20 ));(RCC->APB1RSTR &= ~( 1 << 20 ));}while(0)




//this code mean if x=gpioa than put the value 0  ?mean if  :\mean else
#define GPIO_BASEADDR_TO_CODE(x)  ((x==GPIOA)?0:\
		                           (x==GPIOB)?1:\
                                   (x==GPIOC)?2:\
			                       (x==GPIOD)?3:\
		                           (x==GPIOE)?4:\
		                           (x==GPIOF)?5:\
		                           (x==GPIOG)?6:\
		                           (x==GPIOH)?7:0 )



// IRQ interrupt request number for stm32f407xx MCU
#define IRQ_NO_EXTI0               6
#define IRQ_NO_EXTI1               7
#define IRQ_NO_EXTI2               8
#define IRQ_NO_EXTI3               9
#define IRQ_NO_EXTI4               10
#define IRQ_NO_EXTI9_5             23
#define IRQ_NO_EXTI15_10           40
#define IRQ_NO_SPI1                35
#define IRQ_NO_SPI2                36
#define IRQ_NO_SPI3                51
#define IRQ_NO_I2C1_EV             31
#define IRQ_NO_I2C1_ER             32
#define IRQ_NO_I2C2_EV             33
#define IRQ_NO_I2C2_ER             34
#define IRQ_NO_I2C3_EV             72
#define IRQ_NO_I2C3_ER             73
#define IRQ_NO_USART1              37
#define IRQ_NO_USART2              38
#define IRQ_NO_USART3              39
#define IRQ_NO_UART4               52
#define IRQ_NO_UART5               53
#define IRQ_NO_USART6             71


// macros for all the possible priority levels
#define NVIC_IRQ_PRI0               0
#define NVIC_IRQ_PRI15              15


//some generic macros

#define ENABLE                  1
#define DISABLE                 0
#define SET                ENABLE
#define RESET             DISABLE
#define GPIO_PIN_SET          SET
#define GPIO_PIN_RESET      RESET
#define FLAG_RESET          RESET
#define FLAG_SET              SET




/**************************************************
 *  BIT POSITION DEFINITION FOR SPI PERIPHERAL
 **************************************************/
//CR1 register bit fields

#define SPI_CR1_CPHA          0
#define SPI_CR1_CPOL          1
#define SPI_CR1_MSTR          2
#define SPI_CR1_BR            3
#define SPI_CR1_SPE           6
#define SPI_CR1_LSBFIRST      7
#define SPI_CR1_SSI           8
#define SPI_CR1_SSM           9
#define SPI_CR1_RXONLY        10
#define SPI_CR1_DFF           11
#define SPI_CR1_CRCNEXT       12
#define SPI_CR1_CRCEN         13
#define SPI_CR1_BIDIOE        14
#define SPI_CR1_BIDIMODE      15


//CR2 register bit fields

#define SPI_CR2_RXDMAEN        0
#define SPI_CR2_TXDMAEN        1
#define SPI_CR2_SSOE           2
#define SPI_CR2_FRF            4
#define SPI_CR2_ERRIE          5
#define SPI_CR2_RXNEIE         6
#define SPI_CR2_TXEIE          7


//CR2 register bit fields

#define SPI_SR_RXNE           0
#define SPI_SR_TXE            1
#define SPI_SR_CHSIDE         2
#define SPI_SR_UDR            3
#define SPI_SR_CRCERR         4
#define SPI_SR_MODF           5
#define SPI_SR_OVR            6
#define SPI_SR_BSY            7
#define SPI_SR_FRE            8



/**************************************************
 *  BIT POSITION DEFINITION FOR I2C PERIPHERAL    *
 **************************************************/
//CR1 register bit fields

#define I2C_CR1_PE              0
#define I2C_CR1_SMBUS           1
#define I2C_CR1_SMBTYPE         3
#define I2C_CR1_ENARP           4
#define I2C_CR1_ENPEC           5
#define I2C_CR1_ENGC            6
#define I2C_CR1_NOSTRETCH       7
#define I2C_CR1_START           8
#define I2C_CR1_STOP            9
#define I2C_CR1_ACK             10
#define I2C_CR1_POS             11
#define I2C_CR1_PEC             12
#define I2C_CR1_ALERT           13
#define I2C_CR1_SWRST           15


//CR2 register bit fields

#define I2C_CR2_FREQ              0
#define I2C_CR2_ITERREN           8
#define I2C_CR2_ITEVTEN           9
#define I2C_CR2_ITBUFEN           10
#define I2C_CR2_DMAEN             11
#define I2C_CR2_LAST              12


//SR1 register bit fields

#define I2C_SR1_SB               0
#define I2C_SR1_ADDR             1
#define I2C_SR1_BTF              2
#define I2C_SR1_ADD10            3
#define I2C_SR1_STOPF            4
#define I2C_SR1_RxNE             6
#define I2C_SR1_TxE              7
#define I2C_SR1_BERR             8
#define I2C_SR1_ARLO             9
#define I2C_SR1_AF               10
#define I2C_SR1_OVR              11
#define I2C_SR1_PECERR           12
#define I2C_SR1_TIMEOUT          14
#define I2C_SR1_SMBALERT         15


//SR2 register bit fields

#define I2C_SR2_MSL                0
#define I2C_SR2_BUSY               1
#define I2C_SR2_TRA                2
#define I2C_SR2_GENCALL            4
#define I2C_SR2_SMBDEFAULT         5
#define I2C_SR2_SMBHOST            6
#define I2C_SR2_DUALF              7
#define I2C_SR2_PEC                8



//CCR register bit fields

#define I2C_CCR_CCR                0
#define I2C_CCR_DUTY               14
#define I2C_CCR_FS                 15


/**************************************************
 *  BIT POSITION DEFINITION FOR USART PERIPHERAL    *
 **************************************************/


// SR register bit fields
#define USART_SR_PE         0
#define USART_SR_FE         1
#define USART_SR_NF         2
#define USART_SR_ORE        3
#define USART_SR_IDLE       4
#define USART_SR_RXNE       5
#define USART_SR_TC         6
#define USART_SR_TXE        7
#define USART_SR_LBD        8
#define USART_SR_CTS        9


// DR register bit fields
#define USART_DR_DR         0


// BRR register bit fields
#define USART_BRR_DIV_Fraction         0
#define USART_BRR_DIV_Mantissa         4


// CR1 register bit fields
#define USART_CR1_SBK            0
#define USART_CR1_RWU            1
#define USART_CR1_RE             2
#define USART_CR1_TE             3
#define USART_CR1_IDLEIE         4
#define USART_CR1_RXNEIE         5
#define USART_CR1_TCIE           6
#define USART_CR1_TXEIE          7
#define USART_CR1_PEIE           8
#define USART_CR1_PS             9
#define USART_CR1_PCE            10
#define USART_CR1_WAKE           11
#define USART_CR1_M              12
#define USART_CR1_UE             13
#define USART_CR1_OVER8          15


// CR2 register bit fields
#define USART_CR2_ADD            0
#define USART_CR2_LBDL           5
#define USART_CR2_LBDIE          6
#define USART_CR2_LBCL           8
#define USART_CR2_CPHA           9
#define USART_CR2_CPOL           10
#define USART_CR2_CLKEN          11
#define USART_CR2_STOP           12
#define USART_CR2_LINEN          14


// CR3 register bit fields
#define USART_CR3_EIE             0
#define USART_CR3_IREN            1
#define USART_CR3_IRLP            2
#define USART_CR3_HDSEL           3
#define USART_CR3_NACK            4
#define USART_CR3_SCEN            5
#define USART_CR3_DMAR            6
#define USART_CR3_DMAT            7
#define USART_CR3_RTSE            8
#define USART_CR3_CTSE            9
#define USART_CR3_CTSIE           10
#define USART_CR3_ONEBIT          11



// GTPR register bit fields
#define USART_GTPR_PSC             0
#define USART_GTPR_GT              8








#include "stm32f407xx_gpio_driver.h"
#include "stm32f407xx_spi_driver.h"
#include "stm32f407xx_i2c_driver.h"
#include "stm32f407xx_usart_driver.h"
#include "stm32f407xx_rcc_driver.h"




#endif /* INC_STM32F407XX_H_ */
