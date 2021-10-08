/*-----------------------------------------------------------------------------*/
/*
 * i2c_swe.h
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
#ifndef UART_SE_H_INCLUDED
#define UART_SE_H_INCLUDED
//---------------------------------------------------------------------
#include <stddef.h>

#include "main.h"

//---------------------------------------------------------------------
/**USART2 GPIO Configuration
PD5     ------> USART2_RX
PD6     ------> USART2_TX
*/

#define UART_SE_RX      GPIO_PIN_5
#define UART_SE_TX      GPIO_PIN_6
#define UART_SE_PORT    GPIOD
#define UART_SE         UART2
//---------------------------------------------------------------------


//---------------------------------------------------------------------
void uart_se_init();

void uart_se_receive_sb();

void uart_se_receive_tim();
void uart_se_transmit_tim();

void uart_se_receive_it(uint8_t *buff, uint16_t size);
void uart_se_transmit_it(uint8_t *buff, uint16_t size);
//---------------------------------------------------------------------

#endif /* UART_SWE_H_INCLUDED */
