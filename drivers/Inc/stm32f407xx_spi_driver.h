/*
 * stm32f407xx_spi_driver.h
 *
 *  Created on: Jul 7, 2023
 *      Author: riyad
 */

#ifndef INC_STM32F407XX_SPI_DRIVER_H_
#define INC_STM32F407XX_SPI_DRIVER_H_

#include "stm32f407xx.h"

typedef struct {

	uint8_t SPI_DeviceMode;              // possible values from @SPI_DeviceMode
	uint8_t SPI_BusConfig;                // possible values from @SPI_BusConfig
	uint8_t SPI_SclkSpeed;                // possible values from @SPI_SclkSpeed
	uint8_t SPI_DFF;                       // possible values from @SPI_DFF
	uint8_t SPI_CPOL;                      // possible values from @SPI_CPOL
	uint8_t SPI_CPHA;                      // possible values from @SPI_CPHA
	uint8_t SPI_SSM;                       // possible values from @SPI_SSM

} SPI_Config_t;

//this is a handle structure for SPIx

typedef struct {

	SPI_RegDef_t *pSPIx;             // this holds the base address of the SPIx
	SPI_Config_t SPIConfig;          // this holds SPI configuration settings
	uint8_t *pTxBuffer;
	uint8_t *pRxBuffer;
	uint32_t TxLen;
	uint32_t RxLen;
	uint8_t TxState;
	uint8_t RxState;

} SPI_Handle_t;

//SPI application state
#define SPI_READY                     0
#define SPI_BUSY_IN_RX                1
#define SPI_BUSY_IN_TX                2

//possible spi application events

#define  SPI_EVENT_TX_CMPLT           1
#define  SPI_EVENT_RX_CMPLT           2
#define  SPI_EVENT_OVR_ERR            3
#define  SPI_EVENT_CRC_ERR            4

// @SPI_DeviceMode

#define SPI_DEVICE_MODE_MASTER      1
#define SPI_DEVICE_MODE_SLAVE       0  // by default slave

// @SPI_BusConfig

#define SPI_FULL_DUPLEX             1
#define SPI_HALF_DUPLEX             2
#define SPI_SIMPLEX_RXONLY          3

// @SPI_SclkSpeed

#define SCLK_SPEED_DIV2               0
#define SCLK_SPEED_DIV4               1
#define SCLK_SPEED_DIV8               2
#define SCLK_SPEED_DIV16              3
#define SCLK_SPEED_DIV32              4
#define SCLK_SPEED_DIV64              5
#define SCLK_SPEED_DIV128             6
#define SCLK_SPEED_DIV256             7

//in the next 4 definition the macros that has 0 values these are the default

// @SPI_DFF

#define SPI_DFF_8BITS                 0
#define SPI_DFF_16BITS                1

//  @SPI_CPOL

#define SPI_CPOL_HIGH                 1
#define SPI_CPOL_LOW                  0

//  @SPI_CPHA

#define SPI_CPHA_HIGH                 1
#define SPI_CPHA_LOW                  0

// @SPI_SSM

#define SPI_SSM_EN                    1
#define SPI_SSM_DI                    0

#define SPI_TXE_FLAG     ( 1 << SPI_SR_TXE)
#define SPI_RXE_FLAG     ( 1 << SPI_SR_RXNE)
#define SPI_BUSY_FLAG    ( 1 << SPI_SR_BSY)

/***************************************************************************************
 *                          APIs supported by this SPI driver
 ***************************************************************************************/

//peripheral clock setup
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);

//init and de-init
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);

// Data send and receive
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len);

uint8_t SPI_SendDataIT(SPI_Handle_t *pHandle, uint8_t *pTxBuffer, uint32_t Len);
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pHandle, uint8_t *pRxBuffer,uint32_t Len);

// IRQ configuration and ISR handling

void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);

void SPI_IRQHandling(SPI_Handle_t *pHandle);

// other peripheral control APIs

void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName);
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi);
void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx);
void SPI_CloseTransmisson(SPI_Handle_t *pHandle);
void SPI_CloseReception(SPI_Handle_t *pHandle);

//application call back

void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEv);

#endif /* INC_STM32F407XX_SPI_DRIVER_H_ */

