/*
 * stm32f407xx_usart_driver.h
 *
 *  Created on: Jul 19, 2023
 *      Author: riyad
 */

#ifndef INC_STM32F407XX_USART_DRIVER_H_
#define INC_STM32F407XX_USART_DRIVER_H_



#include "stm32f407xx.h"


// configuration structure for USART peripheral
typedef struct{

	uint8_t USART_Mode;                         // possible values from @USART_Mode
	uint8_t USART_Baud;                         // possible values from @USART_Baud
	uint8_t USART_NOOfStopBits;                 // possible values from @USART_NOOfStopBits
	uint8_t USART_WordLength;                   // possible values from @USART_WordLength
	uint8_t USART_ParityControl;                // possible values from @USART_ParityControl
	uint8_t USART_HWFlowControl;                // possible values from @USART_HWFlowControl

}USART_Config_t;



//this is a handle structure for USARTx

typedef struct
{
    USART_RegDef_t *pUSARTx;    /* This holds the base address of USARTx(x:0,1,2) peripheral */
    USART_Config_t USART_Config;
    uint8_t *pTxBuffer;         /* Stores application Tx buffer address                      */
    uint8_t *pRxBuffer;         /* Stores application Rx buffer address                      */
    uint32_t TxLen;             /* Stores Tx length                                          */
    uint32_t RxLen;             /* Stores Rx length                                          */
    uint8_t TxBusyState;        /* Transmission is in  busy state                            */
    uint8_t RxBusyState;        /* Receiving is in  busy state                               */

}USART_Handle_t;



//@USART_Mode   // from TE and RE bit fields in CR1 register we can configure this options

#define USART_MODE_ONLY_TX   0
#define USART_MODE_ONLY_RX   1
#define USART_MODE_TXRX      2


//@USART_Baud   // we configure the baud rate from BRR register

#define USART_STD_BAUD_1200					1200
#define USART_STD_BAUD_2400					400
#define USART_STD_BAUD_9600					9600
#define USART_STD_BAUD_19200 				19200
#define USART_STD_BAUD_38400 				38400
#define USART_STD_BAUD_57600 				57600
#define USART_STD_BAUD_115200 				115200
#define USART_STD_BAUD_230400 				230400
#define USART_STD_BAUD_460800 				460800
#define USART_STD_BAUD_921600 				921600
#define USART_STD_BAUD_2M 					2000000
#define SUART_STD_BAUD_3M 					3000000


//@USART_NOOfStopBits  // from STOP bit fields in CR2 register we can configure this options
// in higher baud rate like 2Mbps or 3Mbps we better to use more stop bits if it is less the 1Mbps we can use 1 stop bit

#define USART_STOPBITS_1     0
#define USART_STOPBITS_0_5   1
#define USART_STOPBITS_2     2
#define USART_STOPBITS_1_5   3

//@USART_WordLength   // from M bit field in CR1 register we can configure this options

#define USART_WORDLEN_8BITS  0
#define USART_WORDLEN_9BITS  1


//@USART_ParityControl   // from PCE and PS bit fields in CR1 register we can configure this options

#define USART_PARITY_EN_ODD     2
#define USART_PARITY_EN_EVEN    1
#define USART_PARITY_DISABLE    0


//@USART_HWFlowControl    // from CTSE and RTSE bit fields in CR3 register we can configure this options

#define USART_HW_FLOW_CTRL_NONE     	0
#define USART_HW_FLOW_CTRL_CTS    	    1
#define USART_HW_FLOW_CTRL_RTS       	2
#define USART_HW_FLOW_CTRL_CTS_RTS  	3


/*
 * USART Flags
 */
#define USART_FLAG_TXE      ( 1 << USART_SR_TXE  )
#define USART_FLAG_RXNE     ( 1 << USART_SR_RXNE )
#define USART_FLAG_TC       ( 1 << USART_SR_TC   )

/*
 * Application states
 */
#define USART_READY         0
#define USART_BUSY_IN_RX    1
#define USART_BUSY_IN_TX    2



/* Application events */
#define USART_EVENT_TX_CMPLT    0
#define	USART_EVENT_RX_CMPLT    1
#define	USART_EVENT_IDLE        2
#define	USART_EVENT_CTS         3
#define	USART_EVENT_PE          4
#define	USART_ERR_FE            5
#define	USART_ERR_NE            6
#define	USART_ERR_ORE           7


/******************************************************************************************
 *								APIs supported by this driver
 *		 For more information about the APIs check the function definitions
 ******************************************************************************************/
/*
 * Peripheral Clock setup
 */
void USART_PeriClockControl(USART_RegDef_t *pUSARTx, uint8_t EnorDi);

/*
 * Init and De-init
 */
void USART_Init(USART_Handle_t *pUSARTHandle);
void USART_DeInit(USART_RegDef_t *pUSARTx);


/*
 * Data Send and Receive
 */
void USART_SendData(USART_Handle_t *pUSARTHandle,uint8_t *pTxBuffer, uint32_t Len);
void USART_ReceiveData(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len);
uint8_t USART_SendDataIT(USART_Handle_t *pUSARTHandle,uint8_t *pTxBuffer, uint32_t Len);
uint8_t USART_ReceiveDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len);

/*
 * IRQ Configuration and ISR handling
 */
void USART_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void USART_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void USART_IRQHandling(USART_Handle_t *pHandle);

/*
 * Other Peripheral Control APIs
 */
void USART_PeripheralControl(USART_RegDef_t *pUSARTx, uint8_t EnOrDi);
uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx , uint32_t FlagName);
void USART_ClearFlag(USART_RegDef_t *pUSARTx, uint16_t StatusFlagName);
void USART_SetBaudRate(USART_RegDef_t *pUSARTx, uint32_t BaudRate);

/*
 * Application callback
 */
void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHandle,uint8_t AppEv);





#endif /* INC_STM32F407XX_USART_DRIVER_H_ */
