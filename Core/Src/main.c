/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h" //Includes HAL Library functions

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include<stdio.h>
#include<string.h>
#include<stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1; //CAN handle to control CAN1 peripheral

/* USER CODE BEGIN PV */
CAN_FilterTypeDef Scanfilter;//Struct to define CAN filter config
CAN_RxHeaderTypeDef RxHeader; // hold metadeta(ID, DLC) for received messages
CAN_TxHeaderTypeDef TxHeader;//hold metadeta(ID, DLC) for transmitted messages

uint32_t TxMailbox; //buffer for transmission

uint8_t TxData[1];//Stores 1 byte for transmission
uint8_t RxData[1];//For receiving
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
/* USER CODE BEGIN PFP */


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();//Reset peripherals, Flash interface and Systick

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();//Configures system clock

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();//Initialize GPIO and CAN peripherals
  MX_CAN1_Init();
  /* USER CODE BEGIN 2 */
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING|CAN_IT_RX_FIFO1_MSG_PENDING);//Enables CAN interrups when messages arrive in FIFO1
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if(RxData[0]==0x01){
		  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_SET);//For debugging purpose

	  }
	  if(RxData[0]==0x02){
		  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_RESET);

  }
  /* USER CODE END 3 */
}
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;//For mulitple CAN we use hcan2
  hcan1.Init.Prescaler = 16;//Controls CAN bit timing(baud rate).
  hcan1.Init.Mode = CAN_MODE_NORMAL;//Open declaration to see info about multiple modes
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;//detects timing error
  hcan1.Init.TimeSeg1 = CAN_BS1_4TQ;//Synchronizes nodes-> how long the data bits are sampled
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;//Defines post-sampling timing handle phase errors
  hcan1.Init.TimeTriggeredMode = DISABLE;//Check if BMS and MC expects timesstamps
  hcan1.Init.AutoBusOff = DISABLE;//If enabled, the MCU auto. retries after a cool-down period
  hcan1.Init.AutoWakeUp = DISABLE;// For power saving, get's enabled only if CAN-traffic is incoming
  hcan1.Init.AutoRetransmission = ENABLE;// Auto. retries failed transmission
  hcan1.Init.ReceiveFifoLocked = DISABLE;// Blocks new messages when FIFO is full. Must be disabled to refresh data if BMS sends data fast
  hcan1.Init.TransmitFifoPriority = DISABLE;// If enabled smaller IDs transmit first..
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */
  TxHeader.IDE=CAN_ID_STD;
  TxHeader.StdId=0x11;
  TxHeader.RTR=CAN_RTR_DATA;
  TxHeader.DLC=1;


Scanfilter.FilterActivation=CAN_FILTER_ENABLE;
Scanfilter.FilterBank=0;//Each filter holds one or more filter banks
Scanfilter.FilterFIFOAssignment=CAN_FILTER_FIFO0;//Incoming messages go to FIFO0. Stm32 has 2 FIFO lines with FIFO0 having higher priority.
Scanfilter.FilterIdHigh=0x0000;//hold 11 bits for standard CAN
Scanfilter.FilterIdLow=0x0000;//Stays 0 for standard IDs but for 29 bits both high and low hold the ID together
Scanfilter.FilterMaskIdHigh=0xFFFF;;//Specifies which ID to accept(ID must match entirely)
Scanfilter.FilterMaskIdLow=0x0000;//Ignores the low part entirely
Scanfilter.FilterMode=CAN_FILTERMODE_IDMASK;//Better  for ranges
Scanfilter.FilterScale=CAN_FILTERSCALE_32BIT;//Better for specific IDs
Scanfilter.SlaveStartFilterBank=14;//defines where CAN2 filter 2 start

if(HAL_CAN_ConfigFilter(&hcan1, &Scanfilter)!=HAL_OK){//To process only relevant messages
	Error_Handler();
}
if(HAL_CAN_Start(&hcan1)!=HAL_OK){
	Error_Handler();
}

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)//
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);//This set according to the configuration done in the IOC file.

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
// When the Rx of MC detects that the message is coming in the RXFIFO0
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0,&RxHeader, RxData);
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
