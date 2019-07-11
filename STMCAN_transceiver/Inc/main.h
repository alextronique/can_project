/**
  ******************************************************************************
  * File Name          : main.h
  * Description        : This file contains the common defines of the application
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H
  /* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */
#include "stm32f4xx_hal.h"
#include "stdlib.h"
#include "string.h"
#include "stdint.h"
/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define PC6	GPIO_PIN_6
#define PC5	GPIO_PIN_5
#define PC8	GPIO_PIN_8
#define PORTC GPIOC
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

// Tableau de char final (destuffé)
#define TAILLE_MAX_STUFFED		17
#define TAILLE_MAX_UNSTUFFED	14

// Définition des masques
#define M_SOF		0x80	// Case 0
#define M_ID0		0x7F	// Case 0
#define M_ID1		0xF0	// Case 1
#define M_RTR		0x08	// Case 1
#define M_R0		0x04	// Case 1
#define M_R1		0x02	// Case 1
#define M_DLC0	0x01	// Case 1
#define M_DLC1	0xE0	// Case 2

/*#define DATA0	0x1F	// Case 2
#define DATA1	0xFF	// Case 3
#define DATA2	0xFF	// Case 4
#define DATA3	0xFF	// Case 5
#define DATA4	0xFF	// Case 6
#define DATA5	0xFF	// Case 7
#define DATA6	0xFF	// Case 8
#define DATA7	0xFF	// Case 9
#define DATA8	0xE0	// Case 10
#define CRC0
#define CRC1
#define CRC2
#define ACK*/

#define STUFFING_ERROR 	0x01
#define ACK_ERROR		0x02
#define FORM_ERROR		0x04
#define BIT_ERROR		0x08
#define CRC_ERROR		0x10

#define FLAG_TXE		0x00000080


/**/
#define __PACKED

typedef union
{
	uint8_t frame_tab[TAILLE_MAX_UNSTUFFED] __PACKED;
//	uint8_t v[17] __PACKED;

	struct __PACKED
	{
		uint8_t ID_H : 7;
		uint8_t SOF : 1;

		uint8_t DLCH : 1;
		uint8_t R1 : 1;
		uint8_t R0 : 1;
		uint8_t RTR : 1;
		uint8_t ID_L : 4;

		uint8_t DATA1H : 5;
		uint8_t DLCL : 3;

		uint8_t DATA2H : 5;
		uint8_t DATA1L : 3;

		uint8_t DATA3H : 5;
		uint8_t DATA2L : 3;

		uint8_t DATA4H : 5;
		uint8_t DATA3L : 3;

		uint8_t DATA5H : 5;
		uint8_t DATA4L : 3;

		uint8_t DATA6H : 5;
		uint8_t DATA5L : 3;

		uint8_t DATA7H : 5;
		uint8_t DATA6L : 3;

		uint8_t DATA8H : 5;
		uint8_t DATA7L : 3;

		uint8_t CRC_1 : 5;
		uint8_t DATA8L : 3;

		uint8_t CRC_3 : 5;
		uint8_t CRC_2 : 3;

		uint8_t END_OF_FRAME_H : 3;
		uint8_t ACK : 2;
		uint8_t CRC_4 : 3;

		uint8_t NUL : 1;
		uint8_t INTER : 3;
		uint8_t END_OF_FRAME_L : 4;
	} bits;

} frame_t;

 /* */

/* USER CODE END Private defines */

void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)

/**
  * @}
  */ 

/**
  * @}
*/ 

#endif /* __MAIN_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
