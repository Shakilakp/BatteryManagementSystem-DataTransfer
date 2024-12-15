/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32f4xx_hal.h"
#include "stdio.h"
#include "stdint.h"
#include "string.h"
#include "func.h"
#include "stdlib.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

uint8_t SendNumbers[120]; // Array to store the numbers

int indexx;

// Global Variables for cell voltages, temperatures, Stack voltage, PACK Pin voltage, LD Pin voltage, CC2 current
int16_t CellVoltage [16] = {260,100,12,0x00,0x00,0x00,0x00,51,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
uint16_t Stack_Voltage = 0x00;
uint16_t Pack_Voltage = 0x00;
uint16_t LD_Voltage = 0x00;
int16_t CC2_current;
int16_t Temperature [6] = {20,0,0,80,0,0};
uint16_t soc_char;
uint16_t value_CB_ACTIVE_CELLS =123; // Cell Balancing Active Cells 
uint16_t value_CBSTATUS1 = 60;  //Cell Balancing Time
uint8_t value_SafetyStatusA;  // Safety Status Register A
uint8_t value_SafetyStatusB;  // Safety Status Register B
uint8_t value_SafetyStatusC;  // Safety Status Register C
uint8_t value_SafetyAlertA;  // Safety Alert Register A
uint8_t value_SafetyAlertB;  // Safety Alert Register B
uint8_t value_SafetyAlertC;  // Safety Alert Register C
uint16_t value_MANUFACTURINGSTATUS = 35; 
uint16_t value_BatteryStatus =688;  // Battery Status
uint16_t AlarmBits = 466; // Alarm Status
uint8_t FET_Status = 150;  // FET Status register contents  - Shows states of FETs
uint8_t log_data[6];

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

RTC_HandleTypeDef hrtc;

/* USER CODE BEGIN PV */

CAN_HandleTypeDef hcan1;
CAN_TxHeaderTypeDef   pHeader;
CAN_RxHeaderTypeDef   pRxHeader;
uint32_t  TxMailBox;

uint8_t  DataS[8];

CAN_FilterTypeDef sFilterConfig;

uint16_t OwnID = 0x123;
uint16_t RemoteID = 0x124;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_RTC_Init(void);
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
    RTC_TimeTypeDef sTime;
    RTC_DateTypeDef sDate;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

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
  MX_CAN1_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */
	
		//Set Transmit header paramters
	pHeader.DLC = 8;
	pHeader.IDE = CAN_ID_STD;
	pHeader.StdId = OwnID;
	pHeader.RTR = CAN_RTR_DATA;
	
	//config filter  
  sFilterConfig.FilterFIFOAssignment=CAN_FILTER_FIFO0;
	sFilterConfig.FilterIdHigh=RemoteID<<5;
	sFilterConfig.FilterIdLow=0;
	sFilterConfig.FilterMaskIdHigh=0;
	sFilterConfig.FilterMaskIdLow=0;
	sFilterConfig.FilterScale=CAN_FILTERSCALE_32BIT;
	sFilterConfig.FilterActivation=ENABLE;
	
	
	HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig);
	
  HAL_CAN_Start(&hcan1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		indexx = 0;
		
		AddUint8ToArray(value_SafetyStatusA, SendNumbers, indexx);
		indexx++;
		
		AddUint8ToArray(value_SafetyStatusB, SendNumbers, indexx);
		indexx++;
		
		AddUint8ToArray(value_SafetyStatusC, SendNumbers, indexx);
		indexx++;
		
		AddUint8ToArray(value_SafetyAlertA, SendNumbers, indexx);
		indexx++;
		
		AddUint8ToArray(value_SafetyAlertB, SendNumbers, indexx);
		indexx++;
		
		AddUint8ToArray(value_SafetyAlertC, SendNumbers, indexx);
		indexx++;
		
		AddUint8ToArray(FET_Status, SendNumbers, indexx);
		indexx++;
		
		// uint_8 done 6
		
		for (int i = 0; i < 16; i++)
		{
			AddInt16ToArray(CellVoltage[i], SendNumbers, indexx);
			indexx += 2;
		}
		
		for (int i = 0; i < 6; i++)
		{
			AddInt16ToArray(Temperature[i], SendNumbers, indexx);
			indexx += 2;
		}
		
		AddInt16ToArray(CC2_current, SendNumbers, indexx);
		indexx += 2;
		
		// int16_t done
		
		AddUint16ToArray(Stack_Voltage, SendNumbers, indexx);
		indexx += 2;
		
		AddUint16ToArray(Pack_Voltage, SendNumbers, indexx);
		indexx += 2;
		
		AddUint16ToArray(LD_Voltage, SendNumbers, indexx);
		indexx += 2;
		
		AddUint16ToArray(soc_char, SendNumbers, indexx);
		indexx += 2;
		
		AddUint16ToArray(value_CB_ACTIVE_CELLS, SendNumbers, indexx);
		indexx += 2;
		
		AddUint16ToArray(value_CBSTATUS1, SendNumbers, indexx);
		indexx += 2;
				
		AddUint16ToArray(value_MANUFACTURINGSTATUS, SendNumbers, indexx);
		indexx += 2;
		
		AddUint16ToArray(value_BatteryStatus, SendNumbers, indexx);
		indexx += 2;
		
		AddUint16ToArray(AlarmBits, SendNumbers, indexx);
		indexx += 2;
		
		// uint16_t done
		
		HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
    HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
		/* Prepare the CAN data */
    
		log_data[0] = sDate.Year;
    log_data[1] = sDate.Month;
    log_data[2] = sDate.Date;
    log_data[3] = sTime.Hours;
    log_data[4] = sTime.Minutes;
    log_data[5] = sTime.Seconds;
		
		for (int i = 0; i < 6; i++) {
			
			AddUint8ToArray(log_data[i], SendNumbers, indexx);
			indexx++;
		}
		
		// Timing done

		
		HAL_Delay(20);
		
    uint8_t DataSend[8] = {0}; // Initialize to zeros		
		            // Loop to send the numbers array over CAN
            for (int i = 0; i < 15; i++) {

                // Set the data bytes of the CAN message

                for (int j = 0; j < 8; j++) {
                        DataSend[j] = SendNumbers[i * 8 + j];
                }
                // Send CAN message
                HAL_CAN_AddTxMessage(&hcan1, &pHeader, DataSend, &TxMailBox);

                // Delay to ensure CAN message is transmitted before proceeding to the next one
                HAL_Delay(1); // Adjust delay as needed depending on CAN bus speed and message length                

                
							}

    HAL_Delay(100);
		HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
		HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13);
		HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14);
		HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_15);
		
		HAL_Delay(100);
		HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
		HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13);
		HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14);
		HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_15);		

  }
  /* USER CODE END 3 */
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 48;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_4TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable the reference Clock input
  */
  if (HAL_RTCEx_SetRefClock(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pins : PD12 PD13 PD14 PD15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
