/*
 * uart_handler.c
 *
 *  Created on: Jan 10, 2021
 *      Author: tom_h
 */
#include <main.h>
#include "stm32f7xx_hal.h"
#include "uart_handler.h"



/**
  * @brief RX interrrupt handler for 7 or 8 bits data word length .
  * @param huart UART handle.
  * @retval None
  */
void uart_handler_RxISR(UART_HandleTypeDef *huart)
{
  uint16_t uhMask = huart->Mask;
  uint16_t  uhdata;

  /* Check that a Rx process is ongoing */
  if (huart->RxState == HAL_UART_STATE_BUSY_RX)
  {
    uhdata = (uint16_t) READ_REG(huart->Instance->RDR);
    *huart->pRxBuffPtr = (uint8_t)(uhdata & (uint8_t)uhMask);
    huart->pRxBuffPtr++;
    huart->RxXferCount--;

    if (0)    //(huart->RxXferCount == 0U)
    {
      /* Disable the UART Parity Error Interrupt and RXNE interrupts */
      CLEAR_BIT(huart->Instance->CR1, (USART_CR1_RXNEIE | USART_CR1_PEIE));

      /* Disable the UART Error Interrupt: (Frame error, noise error, overrun error) */
      CLEAR_BIT(huart->Instance->CR3, USART_CR3_EIE);

      /* Rx process is completed, restore huart->RxState to Ready */
      huart->RxState = HAL_UART_STATE_READY;

      /* Clear RxISR function pointer */
      huart->RxISR = NULL;

#if (USE_HAL_UART_REGISTER_CALLBACKS == 1)
      /*Call registered Rx complete callback*/
      huart->RxCpltCallback(huart);
#else
      /*Call legacy weak Rx complete callback*/
      HAL_UART_RxCpltCallback(huart);
#endif /* USE_HAL_UART_REGISTER_CALLBACKS */
    }
  }
  else
  {
    /* Clear RXNE interrupt flag */
    //__HAL_UART_SEND_REQ(huart, UART_RXDATA_FLUSH_REQUEST);
    //#define __HAL_UART_SEND_REQ(__HANDLE__, __REQ__) ((__HANDLE__)->Instance->RQR |= (uint16_t)(__REQ__))

  }
}



