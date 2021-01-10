/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include <stdio.h>
#include <string.h>
#include "console.h"
#include "msg.h"
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

UART_HandleTypeDef huart3;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for SendRadio */
osThreadId_t SendRadioHandle;
const osThreadAttr_t SendRadio_attributes = {
  .name = "SendRadio",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for ReceiveRadio */
osThreadId_t ReceiveRadioHandle;
const osThreadAttr_t ReceiveRadio_attributes = {
  .name = "ReceiveRadio",
  .priority = (osPriority_t) osPriorityAboveNormal,
  .stack_size = 128 * 4
};
/* Definitions for SendSms */
osThreadId_t SendSmsHandle;
const osThreadAttr_t SendSms_attributes = {
  .name = "SendSms",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 128 * 4
};
/* Definitions for ReceiveSms */
osThreadId_t ReceiveSmsHandle;
const osThreadAttr_t ReceiveSms_attributes = {
  .name = "ReceiveSms",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 128 * 4
};
/* Definitions for SendRadioQueue */
osMessageQueueId_t SendRadioQueueHandle;
const osMessageQueueAttr_t SendRadioQueue_attributes = {
  .name = "SendRadioQueue"
};
/* Definitions for Queue02 */
osMessageQueueId_t Queue02Handle;
const osMessageQueueAttr_t Queue02_attributes = {
  .name = "Queue02"
};
/* Definitions for ReceiveRadioQueue */
osMessageQueueId_t ReceiveRadioQueueHandle;
const osMessageQueueAttr_t ReceiveRadioQueue_attributes = {
  .name = "ReceiveRadioQueue"
};
/* Definitions for SendSmsQueue */
osMessageQueueId_t SendSmsQueueHandle;
const osMessageQueueAttr_t SendSmsQueue_attributes = {
  .name = "SendSmsQueue"
};
/* Definitions for ReceiveSmsQueue */
osMessageQueueId_t ReceiveSmsQueueHandle;
const osMessageQueueAttr_t ReceiveSmsQueue_attributes = {
  .name = "ReceiveSmsQueue"
};
/* Definitions for RawRxSmsQueue */
osMessageQueueId_t RawRxSmsQueueHandle;
const osMessageQueueAttr_t RawRxSmsQueue_attributes = {
  .name = "RawRxSmsQueue"
};
/* Definitions for TimerScanKeypad */
osTimerId_t TimerScanKeypadHandle;
const osTimerAttr_t TimerScanKeypad_attributes = {
  .name = "TimerScanKeypad"
};
/* Definitions for MsgHandlerSema */
osSemaphoreId_t MsgHandlerSemaHandle;
const osSemaphoreAttr_t MsgHandlerSema_attributes = {
  .name = "MsgHandlerSema"
};
/* Definitions for SmsHandlerSema */
osSemaphoreId_t SmsHandlerSemaHandle;
const osSemaphoreAttr_t SmsHandlerSema_attributes = {
  .name = "SmsHandlerSema"
};
/* Definitions for RadioHandlerSema */
osSemaphoreId_t RadioHandlerSemaHandle;
const osSemaphoreAttr_t RadioHandlerSema_attributes = {
  .name = "RadioHandlerSema"
};
/* USER CODE BEGIN PV */
typedef struct{
	uint16_t Value;
	uint8_t  Source;
} Data;

Data DataToSend1={0x2018,1};
Data DataToSend2={0x2019,2};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
void StartDefaultTask(void *argument);
void StartSendRadio(void *argument);
void StartReceiveRadio(void *argument);
void StartSendSms(void *argument);
void StartReceiveSms(void *argument);
void CallbackScanKeypad(void *argument);

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
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  /* USER CODE BEGIN 2 */

  msg_initialize();
  //uint8_t cbuf[32];
  //uint8_t *pdata = cbuf;
  //HAL_StatusTypeDef uart_status;

  /*
  ConsoleInitialize(&huart3);
  ConsoleWr(development, "https://github.com/infrapale/STM32F746_SMS_RFM_Gateway", 1);
  ConsoleWrDec(development, "Elaman tarkoitus on ", 42 ,".", 1);

  ConsoleWrDec(development, "Free messages= ", msg_free_rows(),".", 1);
  ConsoleRdLn(pdata, 10 );
  ConsoleWr(development, (char *)pdata, 1U);

  uart_status = ConsoleRdChar( pdata );
  if (uart_status == HAL_OK) {
      ConsoleWrChar(development, cbuf[0]);
  }
  else {
	  ConsoleWrDec(development, "HAL status", uart_status ,"!", 1);

  }
  */


  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of MsgHandlerSema */
  MsgHandlerSemaHandle = osSemaphoreNew(1, 1, &MsgHandlerSema_attributes);

  /* creation of SmsHandlerSema */
  SmsHandlerSemaHandle = osSemaphoreNew(1, 1, &SmsHandlerSema_attributes);

  /* creation of RadioHandlerSema */
  RadioHandlerSemaHandle = osSemaphoreNew(1, 1, &RadioHandlerSema_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* creation of TimerScanKeypad */
  TimerScanKeypadHandle = osTimerNew(CallbackScanKeypad, osTimerPeriodic, NULL, &TimerScanKeypad_attributes);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of SendRadioQueue */
  SendRadioQueueHandle = osMessageQueueNew (16, sizeof(uint8_t), &SendRadioQueue_attributes);

  /* creation of Queue02 */
  Queue02Handle = osMessageQueueNew (16, sizeof(Data), &Queue02_attributes);

  /* creation of ReceiveRadioQueue */
  ReceiveRadioQueueHandle = osMessageQueueNew (16, sizeof(uint8_t), &ReceiveRadioQueue_attributes);

  /* creation of SendSmsQueue */
  SendSmsQueueHandle = osMessageQueueNew (16, sizeof(uint8_t), &SendSmsQueue_attributes);

  /* creation of ReceiveSmsQueue */
  ReceiveSmsQueueHandle = osMessageQueueNew (16, sizeof(uint8_t), &ReceiveSmsQueue_attributes);

  /* creation of RawRxSmsQueue */
  RawRxSmsQueueHandle = osMessageQueueNew (16, sizeof(uint8_t), &RawRxSmsQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of SendRadio */
  SendRadioHandle = osThreadNew(StartSendRadio, NULL, &SendRadio_attributes);

  /* creation of ReceiveRadio */
  ReceiveRadioHandle = osThreadNew(StartReceiveRadio, NULL, &ReceiveRadio_attributes);

  /* creation of SendSms */
  SendSmsHandle = osThreadNew(StartSendSms, NULL, &SendSms_attributes);

  /* creation of ReceiveSms */
  ReceiveSmsHandle = osThreadNew(StartReceiveSms, NULL, &ReceiveSms_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
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
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART3|RCC_PERIPHCLK_CLK48;
  PeriphClkInitStruct.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48SOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */
  HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin);

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */
  SET_BIT(huart3.Instance->CR1, USART_CR1_RXNEIE);
  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */
  //HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin);
  /* Disable the UART Parity Error Interrupt and RXNE interrupts */
  SET_BIT(huart3.Instance->CR1, USART_CR1_RXNEIE);
  //SET_BIT(huart3.Instance->CR1, USART_CR1_PEIE);


  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 6;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RMII_MDC_Pin RMII_RXD0_Pin RMII_RXD1_Pin */
  GPIO_InitStruct.Pin = RMII_MDC_Pin|RMII_RXD0_Pin|RMII_RXD1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : RMII_REF_CLK_Pin RMII_MDIO_Pin RMII_CRS_DV_Pin */
  GPIO_InitStruct.Pin = RMII_REF_CLK_Pin|RMII_MDIO_Pin|RMII_CRS_DV_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : RMII_TXD1_Pin */
  GPIO_InitStruct.Pin = RMII_TXD1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(RMII_TXD1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RMII_TX_EN_Pin RMII_TXD0_Pin */
  GPIO_InitStruct.Pin = RMII_TX_EN_Pin|RMII_TXD0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void AddSmsRxToQueue(uint8_t rx_c){
	osMessagePut(RawRxSmsQueueHandle,rx_c,200);

}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  osTimerStart(TimerScanKeypadHandle,100);
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartSendRadio */
/**
* @brief Function implementing the SendRadio thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartSendRadio */
void StartSendRadio(void *argument)
{
  /* USER CODE BEGIN StartSendRadio */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartSendRadio */
}

/* USER CODE BEGIN Header_StartReceiveRadio */
/**
* @brief Function implementing the ReceiveRadio thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartReceiveRadio */
void StartReceiveRadio(void *argument)
{
  /* USER CODE BEGIN StartReceiveRadio */
  uint8_t  rx_buf[ MAX_MSG_LEN];
  memset(rx_buf,0x00,sizeof(rx_buf));
  /* Infinite loop */
  for(;;)
  {

      osDelay(1);
  }
  /* USER CODE END StartReceiveRadio */
}

/* USER CODE BEGIN Header_StartSendSms */
/**
* @brief Function implementing the SendSms thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartSendSms */
void StartSendSms(void *argument)
{
  /* USER CODE BEGIN StartSendSms */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartSendSms */
}

/* USER CODE BEGIN Header_StartReceiveSms */
/**
* @brief Function implementing the ReceiveSms thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartReceiveSms */
void StartReceiveSms(void *argument)
{
  /* USER CODE BEGIN StartReceiveSms */
    uint8_t  sms_in_buf[ MAX_MSG_LEN];
    uint8_t  c;
    //HAL_StatusTypeDef uart_status;

    //ConsoleWr(development, "StartReceiveSms", 1);
	memset(sms_in_buf,0x00,sizeof(sms_in_buf));


	/* Infinite loop */
	for(;;)
	{
	c = (uint8_t)osMessageGet(RawRxSmsQueueHandle,10);
		/*
	    uart_status = HAL_UART_Receive(&huart3, sms_in_buf, 1, 0);
	    switch (uart_status){
	    case HAL_OK:
	    	ConsoleWrChar(application,  sms_in_buf[0]);
	    	break;
	    case  HAL_ERROR:
	    	ConsoleWr(development, "HAL_ERROR", 1);
	    	break;
	    case  HAL_BUSY:
	    	ConsoleWr(development, "HAL_BUSY", 1);
	    	break;
	    case  HAL_TIMEOUT:
	    	break;
	    }

 	    osDelay(1);
 	    */
	}
  /* USER CODE END StartReceiveSms */
}

/* CallbackScanKeypad function */
void CallbackScanKeypad(void *argument)
{
  /* USER CODE BEGIN CallbackScanKeypad */

  /* USER CODE END CallbackScanKeypad */
}

 /**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM7 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM7) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
