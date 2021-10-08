/*-----------------------------------------------------------------------------*/
/*
 * i2c_se.c
 *
 *  SW emulated I2C interface
 *
 *     Created:  9. 5. 2011
 *    Modified: 19. 9. 2019
 *      Author: Martin Polasek, polasek.martin@seznam.cz, http://www.polasekm.cz
 *
 *        ver.: 2.1.0
 */
/*-----------------------------------------------------------------------------*/
#include "uart_se.h"
/*-----------------------------------------------------------------------------*/
#include "tim.h"
//---------------------------------------------------------------------

uint8_t uart_se_rx_state;
uint8_t uart_se_tx_state;

uint8_t uart_se_rx_b;
uint8_t uart_se_tx_b;

uint8_t *uart_se_tx_buff;
uint16_t uart_se_tx_remain;

uint8_t *uart_se_rx_buff;
uint16_t uart_se_rx_remain;

GPIO_InitTypeDef GPIO_InitStruct = {0};
//---------------------------------------------------------------------
void uart_se_init()
{
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(UART_SE_PORT, UART_SE_TX, GPIO_PIN_SET);

  /*Configure GPIO pins : TX_Pin */
  GPIO_InitStruct.Pin = UART_SE_TX;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(UART_SE_PORT, &GPIO_InitStruct);

  /*Configure GPIO pins : RX_Pin */
  /*GPIO_InitStruct.Pin = UART_SE_RX;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(UART_SE_PORT, &GPIO_InitStruct);*/

  /*Configure GPIO pins : RX_Pin */
  GPIO_InitStruct.Pin = UART_SE_RX;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(UART_SE_PORT, &GPIO_InitStruct);


  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  //--------------------------------------------------------------------
  uart_se_rx_state = 0;
  uart_se_tx_state = 0;
}
/*-----------------------------------------------------------------------------*/
void uart_se_receive_it(uint8_t *buff, uint16_t size)
{
  if(uart_se_rx_state == 0)
  {
    uart_se_rx_buff = buff;
    uart_se_rx_remain = size;

    /*Configure GPIO pins : RX_Pin */
    /*GPIO_InitStruct.Pin = UART_SE_RX;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(UART_SE_PORT, &GPIO_InitStruct);*/

    /* Enable and set Button EXTI Interrupt to the hight priority */
    //HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);

    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_5);
    HAL_NVIC_ClearPendingIRQ(EXTI9_5_IRQn);
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

    uart_se_rx_state = 1;
  }
}
/*-----------------------------------------------------------------------------*/
void uart_se_receive_sb()
{
  if(uart_se_rx_state == 1)
  {
    HAL_GPIO_TogglePin(USART2_RTS_GPIO_Port, USART2_RTS_Pin);

    /*Configure GPIO pins : RX_Pin */
    /*GPIO_InitStruct.Pin = UART_SE_RX;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(UART_SE_PORT, &GPIO_InitStruct);*/

    HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);

    uart_se_rx_state = 2;
    uart_se_rx_b = 0;

    TIM12->CNT = 0;
    TIM12->ARR = 365;
    TIM12->SR = 0;
    HAL_NVIC_ClearPendingIRQ(TIM8_BRK_TIM12_IRQn);

    HAL_TIM_Base_Start_IT(&htim12);
  }
}
/*-----------------------------------------------------------------------------*/
void uart_se_receive_tim()
{
  if(uart_se_rx_state == 2)
  {
    TIM12->CNT = 0;
    TIM12->ARR = 729;
    TIM12->SR = 0;
    HAL_NVIC_ClearPendingIRQ(TIM8_BRK_TIM12_IRQn);

    //test start b

    if(HAL_GPIO_ReadPin(UART_se_PORT, UART_se_RX) == GPIO_PIN_RESET)
    {
      uart_se_rx_state = 3;
    }
    else
    {
      HAL_TIM_Base_Stop_IT(&htim12);

      uart_se_rx_state = 0;
      uart_se_receive_it(&uart_se_rx_buff, 1);
    }
  }
  else if(uart_se_rx_state == 11)
  {
    if(HAL_GPIO_ReadPin(UART_se_PORT, UART_se_RX) == GPIO_PIN_SET)
    {
      //OK - zapsat
      *uart_se_rx_buff = uart_se_rx_b;

      HAL_TIM_Base_Stop_IT(&htim12);

      uart_se_rx_state = 0;
      rs232c_rx_proc();
    }
    else
    {
      HAL_TIM_Base_Stop_IT(&htim12);

      uart_se_rx_state = 0;
      uart_se_receive_it(&uart_se_rx_buff, 1);
    }
  }
  else
  {
    uart_se_rx_b = uart_se_rx_b << 1;

    if(HAL_GPIO_ReadPin(UART_se_PORT, UART_se_RX) != GPIO_PIN_RESET)
    {
      uart_se_rx_b |= 1;
    }

    uart_se_rx_state++;
  }
}
/*-----------------------------------------------------------------------------*/
void uart_se_transmit_it(uint8_t *buff, uint16_t size)
{
  if(uart_se_tx_state == 0 && size != 0)
  {
    uart_se_tx_state = 1;
    uart_se_tx_b = *buff;

    uart_se_tx_buff = buff++;
    uart_se_tx_remain = size - 1;

    uart_se_transmit_tim();
  }
}
/*-----------------------------------------------------------------------------*/
void uart_se_transmit_tim()
{
  if(uart_se_tx_state == 1)
  {
    //start bit
    HAL_GPIO_TogglePin(USART2_DTR_GPIO_Port, USART2_DTR_Pin);

    uart_se_tx_state = 2;

    TIM14->CNT = 0;
    TIM14->SR = 0;
    HAL_NVIC_ClearPendingIRQ(TIM8_TRG_COM_TIM14_IRQn);

    HAL_GPIO_WritePin(UART_se_PORT, UART_se_TX, GPIO_PIN_RESET);
    HAL_TIM_Base_Start_IT(&htim14);
  }
  else if(uart_se_tx_state == 10 || uart_se_tx_state == 11)
  {
    //stop bity
    HAL_GPIO_WritePin(UART_se_PORT, UART_se_TX, GPIO_PIN_SET);
    uart_se_tx_state++;
  }
  else if(uart_se_tx_state == 12)
  {
    //konec
    HAL_GPIO_TogglePin(USART2_DTR_GPIO_Port, USART2_DTR_Pin);

    uart_se_tx_state = 0;

    if(uart_se_tx_remain != 0)
    {
      uart_se_tx_state = 1;
      uart_se_tx_b = *uart_se_tx_buff;

      uart_se_tx_buff++;
      uart_se_tx_remain--;

      uart_se_transmit_tim();
    }
    else
    {
      HAL_TIM_Base_Stop_IT(&htim14);

      uart_se_tx_state = 0;
      //----------------------
      rs232c_tx_proc();
      //----------------------
    }
  }
  else
  {
    // bity
    if((uart_se_tx_b & 1) == 0)
    {
      HAL_GPIO_WritePin(UART_se_PORT, UART_se_TX, GPIO_PIN_RESET);
    }
    else
    {
      HAL_GPIO_WritePin(UART_se_PORT, UART_se_TX, GPIO_PIN_SET);
    }

    uart_se_tx_b = uart_se_tx_b >> 1;
    uart_se_tx_state++;
  }
}
/*-----------------------------------------------------------------------------*/
