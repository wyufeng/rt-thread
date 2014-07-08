/*
 * File      : usart.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2009, RT-Thread Development Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 2009-01-05     Bernard      the first version
 * 2010-03-29     Bernard      remove interrupt Tx and DMA Rx mode
 * 2012-02-08     aozima       update for F4.
 */

#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "usart.h"
#include "board.h"
#include <serial.h>

/*
 * Use UART1 as console output and finsh input
 * interrupt Rx and poll Tx (stream mode)
 *
 * Use UART2 with interrupt Rx and poll Tx
 * Use UART3 with DMA Tx and interrupt Rx -- DMA channel 2
 *
 * USART DMA setting on STM32
 * USART1 Tx --> DMA Channel 4
 * USART1 Rx --> DMA Channel 5
 * USART2 Tx --> DMA Channel 7
 * USART2 Rx --> DMA Channel 6
 * USART3 Tx --> DMA Channel 2
 * USART3 Rx --> DMA Channel 3
 */

#ifdef RT_USING_UART1
struct stm32_serial_int_rx uart1_int_rx;
struct stm32_serial_device uart1 =
{
    USART1,
    &uart1_int_rx,
    RT_NULL
};
struct rt_device uart1_device;
#endif

#ifdef RT_USING_UART2
struct stm32_serial_int_rx uart2_int_rx;
struct stm32_serial_device uart2 =
{
    USART2,
    &uart2_int_rx,
    RT_NULL
};
struct rt_device uart2_device;
#endif

#ifdef RT_USING_UART3
struct stm32_serial_int_rx uart3_int_rx;
struct stm32_serial_dma_tx uart3_dma_tx;
struct stm32_serial_device uart3 =
{
    USART3,
    &uart3_int_rx,
    &uart3_dma_tx
};
struct rt_device uart3_device;
#endif

//#define USART1_DR_Base  0x40013804
//#define USART2_DR_Base  0x40004404
//#define USART3_DR_Base  0x40004804

/* USART1_REMAP = 0 */
#define UART1_GPIO_TX       GPIO_Pin_9
#define UART1_TX_PIN_SOURCE GPIO_PinSource9
#define UART1_GPIO_RX       GPIO_Pin_10
#define UART1_RX_PIN_SOURCE GPIO_PinSource10
#define UART1_GPIO          GPIOA
#define UART1_GPIO_RCC      RCC_AHB1Periph_GPIOA
#define RCC_APBPeriph_UART1 RCC_APB2Periph_USART1
#define UART1_TX_DMA        DMA1_Channel4
#define UART1_RX_DMA        DMA1_Channel5

#define UART2_GPIO_TX       GPIO_Pin_2
#define UART2_TX_PIN_SOURCE GPIO_PinSource2
#define UART2_GPIO_RX       GPIO_Pin_3
#define UART2_RX_PIN_SOURCE GPIO_PinSource3
#define UART2_GPIO          GPIOA
#define UART2_GPIO_RCC      RCC_AHB1Periph_GPIOA
#define RCC_APBPeriph_UART2 RCC_APB1Periph_USART2

/* USART3_REMAP[1:0] = 00 */
#define UART3_GPIO_TX       GPIO_Pin_10
#define UART3_TX_PIN_SOURCE GPIO_PinSource10
#define UART3_GPIO_RX       GPIO_Pin_11
#define UART3_RX_PIN_SOURCE GPIO_PinSource11
#define UART3_GPIO          GPIOB
#define UART3_GPIO_RCC      RCC_AHB1Periph_GPIOB
#define RCC_APBPeriph_UART3 RCC_APB1Periph_USART3
#define UART3_TX_DMA        DMA1_Stream3
//#define UART3_RX_DMA        DMA1_Stream3

static void RCC_Configuration(void)
{
#ifdef RT_USING_UART1
    /* Enable USART1 GPIO clocks */
		__GPIOA_CLK_ENABLE();
#endif

#ifdef RT_USING_UART2
    /* Enable USART2 GPIO clocks */
	  __GPIOA_CLK_ENABLE();
#endif

#ifdef RT_USING_UART3
    /* Enable USART3 GPIO clocks */
		__GPIOB_CLK_ENABLE();
#endif
}


/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

void HAL_UART_MspInit(UART_HandleTypeDef* huart)
{

  GPIO_InitTypeDef GPIO_InitStruct;
#ifdef RT_USING_UART1
  {
    /* Peripheral clock enable */
    __USART1_CLK_ENABLE();
  
    /**USART1 GPIO Configuration    
    PA10     ------> USART1_RX
    PB6     ------> USART1_TX 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  }
  #elif RT_USING_UART2
  {
    /* Peripheral clock enable */
    __USART2_CLK_ENABLE();
  
    /**USART2 GPIO Configuration    
    PA2     ------> USART2_TX
    PA3     ------> USART2_RX 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  }
	#elif ifdef RT_USING_UART3
  else if(huart->Instance==USART3)
  {
    /* Peripheral clock enable */
    __USART3_CLK_ENABLE();
  
    /**USART3 GPIO Configuration    
    PB10     ------> USART3_TX
    PB11     ------> USART3_RX 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  }
	#endif

}
static void NVIC_Configuration(void)
{
IRQn_Type USARTx_IRQn;
#ifdef RT_USING_UART1
    /* Enable the USART1 Interrupt */
    USARTx_IRQn= USART1_IRQn;
	
#endif

#ifdef RT_USING_UART2
    /* Enable the USART2 Interrupt */
			USARTx_IRQn= USART2_IRQn;
	
#endif

#ifdef RT_USING_UART3
    /* Enable the USART3 Interrupt */
			
		USARTx_IRQn= USART3_IRQn;

    /* Enable the DMA1 Channel2 Interrupt */
		HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 1);
		HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
		
#endif


  HAL_NVIC_SetPriority(USARTx_IRQn, 0, 1);
  HAL_NVIC_EnableIRQ(USARTx_IRQn);

}

/*
 * Init all related hardware in here
 * rt_hw_serial_init() will register all supported USART device
 */

void rt_hw_usart_init()
{
    UART_HandleTypeDef USART_Init;

    RCC_Configuration();

		NVIC_Configuration();
	
    /* uart init */
#ifdef RT_USING_UART1

	  USART_Init.Instance = USART1;

    /* register uart1 */
    rt_hw_serial_register(&uart1_device, "uart1",
        RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX | RT_DEVICE_FLAG_STREAM,
        &uart1);
#endif

#ifdef RT_USING_UART2

	USART_Init.Instance = USART2;

    /* register uart2 */
    rt_hw_serial_register(&uart2_device, "uart2",
        RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX | RT_DEVICE_FLAG_STREAM,
        &uart2);

    /* Enable USART2 DMA Rx request */
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
#endif

#ifdef RT_USING_UART3

		USART_Init.Instance = USART3;
	{
		DMA_HandleTypeDef		hdma_usart3_tx;
		
    hdma_usart3_tx.Instance = DMA1_Stream3;
    hdma_usart3_tx.Init.Channel = DMA_CHANNEL_4;
    hdma_usart3_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_usart3_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart3_tx.Init.MemInc = DMA_MINC_DISABLE;
    hdma_usart3_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart3_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart3_tx.Init.Mode = DMA_NORMAL;
    hdma_usart3_tx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_usart3_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    HAL_DMA_Init(&hdma_usart3_tx);
	}
    
	  uart3_dma_tx.dma_channel= UART3_TX_DMA;

    /* register uart3 */
    rt_hw_serial_register(&uart3_device, "uart3",
        RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX | RT_DEVICE_FLAG_DMA_TX,
        &uart3);

    /* Enable USART3 DMA Tx request */
//    USART_DMACmd(USART3, USART_DMAReq_Tx , ENABLE);
	  
      USART3->CR3|=0X80;
    /* enable interrupt */
#endif
    
		USART_Init.Init.BaudRate = 115200;
		USART_Init.Init.WordLength = UART_WORDLENGTH_8B;
		USART_Init.Init.StopBits = UART_STOPBITS_1;
		USART_Init.Init.Parity = UART_PARITY_NONE;
		USART_Init.Init.Mode = UART_MODE_TX_RX;
		USART_Init.Init.HwFlowCtl = UART_HWCONTROL_NONE;
		USART_Init.Init.OverSampling = UART_OVERSAMPLING_8;
		HAL_UART_Init(&USART_Init);

		
		USART_Init.Instance->CR1|=UART_IT_RXNE;
    HAL_UART_MspInit(&USART_Init);
		
}
