/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"


/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
/*

uint8_t newTab[²];*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART2_UART_Init(void);
void NVIC_Init(void);


/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
uint8_t FrameDestuff(uint8_t * frameCAN, uint8_t * newTab);
uint8_t FrameRead(uint8_t * tab, frame_t * frameStruct);
void ErrorManagement(frame_t * frameStruct, uint8_t errorReport, UART_HandleTypeDef huart);
void UART_SendChar(UART_HandleTypeDef huart, uint8_t data);
void UART_SendString(UART_HandleTypeDef huart, char *p);
uint16_t CAN_execCrc(uint16_t crc, uint8_t data);
void canTrameTransfo(uint8_t* trame, uint8_t* res, uint8_t size);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
void EXTI9_5_IRQHandler (void)
{
	if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_6) != RESET)
	{
		// Lancer TIM1
		__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_6);
		HAL_GPIO_EXTI_Callback(GPIO_PIN_6);
	}
}

/* Timer 1 interrupt @ 8us */
void TIM1_UP_TIM10_IRQHandler(void)
{
	HAL_TIM_IRQHandler(&htim1);
	HAL_GPIO_TogglePin(PORTC, PC5);
}

/* Timer 2 interrupt @ 3us */
void TIM2_IRQHandler(void)
{
	HAL_TIM_IRQHandler(&htim2);
	HAL_GPIO_TogglePin(PORTC, PC8);
}
/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
  /* Private variables ----------------------------------------------------------*/
	// Structure de la trame
	static frame_t frameStruct;
	static uint8_t crcTab[256];

	// Tableau de test
	uint8_t tmpTab[TAILLE_MAX_STUFFED] =
	{		0xFB, 0x41, 0xDA, 0x1F, 0x01, 0xF2, 0xFF, 0x04, 0x1A, 0xC6,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
	};


	//Trame sans erreur de stuffing


	//Trame avec erreur de stuffing - 6bits de même valeur a la suite
	uint8_t tmpTab2[TAILLE_MAX_STUFFED] =
	{		0x9F, 0x51, 0x50, 0x77, 0xF2, 0xDC, 0xDF, 0x14, 0x79, 0xF1,
			0x0A, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x00
	};

	uint8_t tmpCRC[]={0x00, 0x04, 0x92, 0x90, 0x90};

	//Trame avec erreur de forme

	//Trame avec erreur de bit

	//Trame avec erreur de CRC

	uint8_t errorReport = 0;

	// Tableau destuffé
	static uint8_t unstuffTab[TAILLE_MAX_UNSTUFFED] = {0};
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  /*MX_TIM1_Init();
  MX_TIM2_Init();*/
  MX_USART2_UART_Init();


  /* USER CODE BEGIN 2 */
  //NVIC_Init();
  //HAL_TIM_Base_Start_IT(&htim1);
  //HAL_TIM_Base_Start_IT(&htim2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  //Demarrage du sniffing CAN
  UART_SendString(huart2, "\nDemarrage du sniffing CAN");

  //Destuffing de la trame
  errorReport |= FrameDestuff(tmpTab2, unstuffTab);

  //Traitement de la trame destuffée
  errorReport |= FrameRead(unstuffTab, &frameStruct);


  /* Transformation de la trame pour le calcul du CRC */
  uint8_t DLC = 0;
  uint8_t frameSizeOctet = 0;
  uint8_t frameSizeBit = 0;
  uint16_t crc = 0;

  //DLC |= ( (frameStruct.bits.DLCH << 3) | (frameStruct.bits.DLCL) );
  //frameSizeOctet = (6+DLC)-1;
  //frameSizeBit = ((DLC*8)+19-1);

  frameSizeBit = 34;
  frameSizeOctet = 5;
  canTrameTransfo(tmpCRC, crcTab, frameSizeBit);
  for(int i = 0; i < frameSizeOctet+1; i++)
  {
      crc = CAN_execCrc(crc, crcTab[i]);
  }

  //Affichage des erreurs
  ErrorManagement(&frameStruct, errorReport, huart2);


  // Gestion des erreurs
  // Lecture des "return" avec masques

  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
	  ;;
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Activate the Over-Drive mode 
    */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* TIM1 init function */
static void MX_TIM1_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 130;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 10;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 130;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(PORTC, PC5, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(PORTC, PC8, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PC8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PC5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PC6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */



/* _____________________________________________________________________________________________*/
// Fonction permettant de récupérer un tableau de char provenant								//
// de l'échantillonnage, ou les bits sont rangés à la volée										//
// On veut déstuffer cette trame, afin de l'identifier comme									//
// valide ou non, et trier les données															//
uint8_t FrameDestuff(uint8_t * frameCAN, uint8_t * newTab)
{
	// Pour gérer le destuffing, on a besoin d'un compteur
	// On incrémente ce compteur à chaque fois que le bit n+1
	// est de même valeur que le bit n. On le reset lorsqu'ils
	// sont différents
	uint8_t cptStuffing = 0;
	uint8_t bitPre = -1; 	// Stock la valeur du bit précédent, pour comparaison
	uint8_t stock = 1;		// Indique si la prochaine valeur est a stocker ou a destuffer
	uint8_t cptChar = 0;	// Compteur de caractères
	uint8_t cptBit = 0;		// Compteur de bits
	uint8_t ErrorReport = 0;// Rapport d'erreur
	uint8_t dlcO = 0;
	int k=0, l=0;



	// On commence le destuffing
	// Première boucle for pour parcourir les cases du tableau
	for (uint8_t i=0 ; i < TAILLE_MAX_STUFFED ; i++)
	{
		// Deuxième boucle for pour parcourir chaque bits de la case
		for (uint8_t j=0 ; j < 8 ; j++)
		{
			// Si on a déjà destuffé au moins le DLC
			if( (cptChar == 2) && (cptBit==3) )
				// Traitement DLC pour remplir la structure
				dlcO = ((newTab[1] & M_DLC0) << 3) | ((newTab[2] & M_DLC1) >> 5);

			// Puis si on dépasse la case du CRC
			if (cptChar == ((6+dlcO)-2))
			{
				// Si on dépasse aussi le bit du CRC
				if(cptBit == 2)
				{
					// On veut stocker les dernières cases du tableau de base dans le nouveau
					// On refait un compteur qui repart de la case où on s'est arrêté de newTab
					for(k=cptChar; k<((6+dlcO)+1); k++)
					{
						// Compteur de bit de newTab
						for(l=0 ; l<8; l++)
						{
							// Le premier tour commence à cptBit, c'est le dernier bit de CRC
							// Aux tours suivants, on reprend au bit 0
							if(k==cptChar && l == 0)
								l=cptBit;

							if (j>l)
								*(newTab+k) |= (( (*(frameCAN+i)) & (0b10000000 >> j)) << (abs(j-l)));
							else
								*(newTab+k) |= (( (*(frameCAN+i)) & (0b10000000 >> j)) >> (abs(j-l)));

							//Incrémentation des compteurs du tableau de base
							if(j>=7)
							{
								j=0;
								i++;
							}
							else
								j++;

						}
					}
					// Sortie de la fonction
					return ErrorReport;
				}

			}

			// Mise en place d'un compteur pour les bits de même valeur
			// Permet de faire le destuffing jusqu'au CRC, ensuite il n'y a plus de destuffing

			// Si le bit précédent est de même valeur que le bit actuel
			if (bitPre == (( (*(frameCAN+i)) ) & (0b10000000 >> j)) >> (7-j))
			{
				// Incrémentation du compteur de stuffing
				cptStuffing++;
			}

			else
				cptStuffing=0;

			// Si on doit stocker le bit actuel dans le nouveau tableau
			if (stock)
			{
				// Décalage différent en fonction de l'emplacement de chaque cpt de bit.
				// Il arrive que le cpt de bit du tab de base ait une valeur plus petite que celui du newTab
				// car il a un caractere d'avance. En effet, dans newTab on a les bits de stuffing en moins
				if (j>cptBit)
					*(newTab+cptChar) |= (( (*(frameCAN+i)) & (0b10000000 >> j)) << (abs(j-cptBit)));
				else
					*(newTab+cptChar) |= (( (*(frameCAN+i)) & (0b10000000 >> j)) >> (abs(j-cptBit)));

				// Incrementation des cpt de bit
				if(cptBit == 7)
				{
					cptChar++;
					cptBit = 0;
				}
				else
					cptBit++;
			}

			// Test sur le compteur du stuffing
			if (cptStuffing == 4)
				stock = 0; // Prochaine valeur = bitstuffing -> on ne la conserve pas
			else if (cptStuffing < 4)
				stock = 1;
			else
			// Une erreur de stuffing a eu lieu, c'est-a-dire qu'il y a eu un 0 après des 0 ou un 1 après des 1
			// On envoie un rapport d'erreur, on remet a 0 le cpt et on stocke la valeur suivante
			{
				ErrorReport |= STUFFING_ERROR;
				cptStuffing = 0;
				stock = 1;
			}

			// Mise a jour de la valeur du bit precedent
			bitPre = (( (*(frameCAN+i)) & (0b10000000 >> j)) >> (7-j));
		}
	}
	return 0;
}




/* _____________________________________________________________________________________________*/
// Fonction de mise en forme de la trame reçue
// Après destuffing, la trame est rangée dans un tableau
// Cette fonction permet d'ordonner les différents éléments de la trame dans une structure
uint8_t FrameRead(uint8_t * tab, frame_t * frameStruct)
{
	uint8_t dlcO; 							// dlcBits et dlcOctets
	uint8_t ErrorReport = 0; 				// Rapport d'erreur

	// Traitement DLC pour remplir la structure
	dlcO = ((tab[1] & M_DLC0) << 3) | ((tab[2] & M_DLC1) >> 5);

	// RAZ du tableau de la structure
	for (int i = 0 ; i < TAILLE_MAX_UNSTUFFED ; i++)
	{
		frameStruct->frame_tab[i] = 0;
	}

	for (int i = 0 ; i < TAILLE_MAX_UNSTUFFED ; i++)
	{
		// Correspond à la case 2 contenant 3bits DLC et (soit 5 bits DATA, soit 5 bits CRC)
		// et à la dernière case Data + début CRC en fonction de dlcO
		if (i==(2+dlcO))
		{
			frameStruct->frame_tab[i] = (tab[i]&(~0x1F));
			frameStruct->frame_tab[i+(8-dlcO)] = (tab[i]&0x1F);
		}

		// Correspond aux deux dernières cases, qui changent
		// de place en fonction de dlcO
		else if (i>(2+dlcO))
			frameStruct->frame_tab[i+(8-dlcO)] = tab[i];

		// Cas "normal" (début), ne change jamais
		else
			frameStruct->frame_tab[i] = tab[i];
	}
	// On retourne les erreurs
	return ErrorReport;
}


/*______________________________________________________________________________________________*/
// Fonction de gestion des erreurs de trame
void ErrorManagement(frame_t * frameStruct, uint8_t errorReport, UART_HandleTypeDef huart)
{
	if((errorReport & 0x01) == 0x01)
	{
		// La trame possède une erreur de stuffing
		UART_SendString(huart, "\nCode erreur : ");
		UART_SendChar(huart, errorReport);
		UART_SendString(huart, "\nErreur de sniffing\n");
	}
	if((errorReport & 0x04) == 0x04)
	{
		// La trame possède une erreur de forme
		UART_SendString(huart, "\nCode erreur : ");
		UART_SendChar(huart, errorReport);
		UART_SendString(huart, "\nErreur de forme\n");
	}
	if((errorReport & 0x10) == 0x10)
	{
		// La trame possède une erreur de CRC
		UART_SendString(huart, "\nCode erreur : ");
		UART_SendChar(huart, errorReport);
		UART_SendString(huart, "\nErreur de CRC\n");
	}
	if(errorReport == 0x00)
	{
		//La trame ne possède aucune erreur
		UART_SendString(huart, "\nCode erreur : ");
		UART_SendChar(huart, errorReport);
		UART_SendString(huart, "Aucune erreur\n");
	}
}


/*______________________________________________________________________________________________*/
// Send a character on UART
void UART_SendChar(UART_HandleTypeDef huart, uint8_t data)
{
	while((huart.Instance->SR & FLAG_TXE) == 0);
	huart.Instance->DR = data;
}

/*______________________________________________________________________________________________*/
// Send a string on UART
void UART_SendString(UART_HandleTypeDef huart, char *p)
{
	while(*p != 0)
	{
		UART_SendChar(huart,*p);
		p++;
	}
}


/*_________________________________________________________________________*/
/* Calculate the CRC of the frame */
uint16_t CAN_execCrc(uint16_t crc, uint8_t data)
{
	uint8_t i;
	uint16_t poly;

	poly = 0xc599;

	crc ^= (uint16_t)data << 7; //On applique un XOR sur le crc et la data décalé de 7.

	for (i = 0; i < 8; i++) {
		crc <<= 1; 				// Décalage à gauche.
		if (crc & 0x8000) { 	// On décale à gauche jusqu'a ce que un ET du mask et du CRC soit égale à 1.
			crc ^= poly; 		// Valeur du polynome avec X = 2;
		}
	}
	return crc & 0x7fff; 		// Utilisation d'un masque pour ne récupérer que les 15bits
}


/*__________________________________________________________________________________________________*/
/* Create a frame to calculate CRC */
void canTrameTransfo(uint8_t* trame, uint8_t* res, uint8_t size)
{
    int i;

    for(i = 0; i < size+1; i++)
        res[i] = ((trame[i-2] << 3) & 0xF8) | ((trame[i-1] >> 5));

    res[0] = 0;
    res[1] = (trame[0] >> 5);
}

/*______________________________________________________________________________________________*/
// Init NVIC
void NVIC_Init(void)
{
	HAL_NVIC_SetPriority(EXTI9_5_IRQn,0,0);
	HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
	HAL_NVIC_SetPriority(TIM1_UP_TIM10_IRQn,0,0);
	HAL_NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);
	HAL_NVIC_SetPriority(TIM2_IRQn,0,0);
	HAL_NVIC_EnableIRQ(TIM2_IRQn);
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
