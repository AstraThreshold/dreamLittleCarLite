/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  * CH -> 00 FF 46 -> MODE SWITCH
  * - -> 00 FF 07 -> STOP
  * 2 -> 00 FF 18 -> FORWARD
  * 8 -> 00 FF 52 -> BACK
  * 4 -> 00 FF 08 -> LEFT
  * 6 -> 00 FF 5A -> RIGHT
  * play -> 00 FF 43 -> GO
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TRACK1 HAL_GPIO_ReadPin(TRACK_OUT1_GPIO_Port, TRACK_OUT1_Pin)
#define TRACK2 HAL_GPIO_ReadPin(TRACK_OUT2_GPIO_Port, TRACK_OUT2_Pin)
#define TRACK3 HAL_GPIO_ReadPin(TRACK_OUT3_GPIO_Port, TRACK_OUT3_Pin)
#define TRACK4 HAL_GPIO_ReadPin(TRACK_OUT4_GPIO_Port, TRACK_OUT4_Pin)
#define TRACK5 HAL_GPIO_ReadPin(TRACK_OUT5_GPIO_Port, TRACK_OUT5_Pin)

#define sr04TrigHigh() HAL_GPIO_WritePin(SR04_TRIG_GPIO_Port,SR04_TRIG_Pin,GPIO_PIN_SET)
#define sr04TrigLow()  HAL_GPIO_WritePin(SR04_TRIG_GPIO_Port,SR04_TRIG_Pin,GPIO_PIN_RESET)
#define sr04Echo()  HAL_GPIO_ReadPin(SR04_ECHO_GPIO_Port,SR04_ECHO_Pin)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
uint8_t ifMsgGet = 0;
uint8_t msg = 0;
uint8_t speed = 0;
uint8_t tickFlag = 0;
uint16_t pwmVal = 1;

float kp = 30;
float ki = 1.3;
float kd = 3.5;

extern uint16_t msHcCount = 0;

uint16_t cntWhile;

uint8_t firstTimeRun = 1;

uint8_t rxBuffer[4], rxFlag; // 4 bits buffer enough
uint8_t rxMsg;
uint8_t offSet;

uint8_t carMode = 0; //0->bluetooth 1->IR

uint8_t irStart = 0;
uint8_t irStop = 0;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
void littleCarMove();

void goForward();

void goBack();

void turnLeft();

void turnRight();

void stopAll();

float pidOutput();

void sr04Init();

void sr04TimerMode(uint8_t mode);

float sr04GetDistant();

float sr04GetDistantAfterFilter(uint8_t cnt);

void delayus(uint32_t nus);
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
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_TIM4_Init();
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  pwmVal = 235;

  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

  HAL_UART_Receive_IT(&huart2, (uint8_t *)&rxMsg, 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  HAL_UART_Transmit(&huart1, "Ready!\r\n", sizeof("Ready!\r\n"), 20);
  HAL_UART_Transmit(&huart1, "Waiting...\r\n", sizeof("Waiting...\r\n"), 20);
  HAL_UART_Transmit(&huart1, "Motor Power:", sizeof("Motor Power:"), 20);
  char powerMsg[10] = "";
  sprintf(powerMsg, "%.2f", 1000000.0 / pwmVal / 10000.0);
  HAL_UART_Transmit(&huart1, powerMsg, sizeof(powerMsg), 20);

    __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_3, pwmVal);
    __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_4, pwmVal - 30);

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    if(rxFlag == 1) //receive message
    {
      //switch mode
      //HAL_UART_Transmit(&huart1, &rxBuffer[2], sizeof(rxBuffer[2]), 13);

      if (rxBuffer[2] == 0x46)
      {
        carMode++;
        if (carMode > 1)  carMode = 0;
        if (carMode == 1) HAL_UART_Transmit(&huart1, "IR Mode!\r\n", sizeof("IR Mode!\r\n"), 20);
        if (carMode == 0) HAL_UART_Transmit(&huart1, "BT Mode!\r\n", sizeof("BT Mode!\r\n"), 20);
      }

      //stop
      if (rxBuffer[2] == 0x07)
      {
        stopAll();
        irStop = 1;
        HAL_UART_Transmit(&huart1, "Stop By User.\r\n", sizeof("Stop By User.\r\n"), 13);
      }

      //forward
      if (rxBuffer[2] == 0x18 && carMode == 1) goForward();

      //back
      if (rxBuffer[2] == 0x52 && carMode == 1) goBack();

      //turn right
      if (rxBuffer[2] == 0x5A && carMode == 1) turnRight();

      //turn left
      if (rxBuffer[2] == 0x08 && carMode == 1) turnLeft();

      //start
      if (rxBuffer[2] == 0x43 && carMode == 0) irStart = 1;

      memset(rxBuffer, '\0', sizeof(rxBuffer)); //clear buffer
      rxFlag = 0;
    }

    while ((msg != '1') && HAL_UART_Receive(&huart1, &msg, 1, 0) == HAL_OK);

    if (msg == '1' || !HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin) || irStart == 1)
    {
      if (irStart == 1) irStart = 0;  //if start by ir, reset flag
      ifMsgGet = 1;
      msg = 0;
    }
    if (ifMsgGet)
    {
      HAL_UART_Transmit(&huart1, "Get!\r\n", sizeof("Get!\r\n"), 20);
      ifMsgGet = 0;

      littleCarMove();
    }
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void delayus(uint32_t nus)
{
  uint16_t differ = 0xffff - nus - 5;
    __HAL_TIM_SetCounter(&htim4, differ);
  HAL_TIM_Base_Start(&htim4);

  while (differ < 0xffff - 5)
  {
    differ = __HAL_TIM_GetCounter(&htim4);
  };

  HAL_TIM_Base_Stop(&htim4);
}

void goForward()
{
    __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_3, pwmVal);    //修改比较值，修改占空�?????
  //__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_4, pwmVal);    //修改比较值，修改占空�?????
    __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_4, pwmVal - 9);    //修改比较值，修改占空�?????
  HAL_GPIO_WritePin(MOTOR1_CTRL1_GPIO_Port, MOTOR1_CTRL1_Pin, 1);
  HAL_GPIO_WritePin(MOTOR1_CTRL2_GPIO_Port, MOTOR1_CTRL2_Pin, 0);
  HAL_GPIO_WritePin(MOTOR2_CTRL1_GPIO_Port, MOTOR2_CTRL1_Pin, 0);
  HAL_GPIO_WritePin(MOTOR2_CTRL2_GPIO_Port, MOTOR2_CTRL2_Pin, 1);
}

void goBack()
{
    __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_3, pwmVal);    //修改比较值，修改占空�?????
  //__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_4, pwmVal);    //修改比较值，修改占空�?????
    __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_4, pwmVal - 9);    //修改比较值，修改占空�?????
  HAL_GPIO_WritePin(MOTOR1_CTRL1_GPIO_Port, MOTOR1_CTRL1_Pin, 0);
  HAL_GPIO_WritePin(MOTOR1_CTRL2_GPIO_Port, MOTOR1_CTRL2_Pin, 1);
  HAL_GPIO_WritePin(MOTOR2_CTRL1_GPIO_Port, MOTOR2_CTRL1_Pin, 1);
  HAL_GPIO_WritePin(MOTOR2_CTRL2_GPIO_Port, MOTOR2_CTRL2_Pin, 0);
}

void stopAll()
{
  HAL_GPIO_WritePin(MOTOR1_CTRL1_GPIO_Port, MOTOR1_CTRL1_Pin, 1);
  HAL_GPIO_WritePin(MOTOR1_CTRL2_GPIO_Port, MOTOR1_CTRL2_Pin, 1);
  HAL_GPIO_WritePin(MOTOR2_CTRL1_GPIO_Port, MOTOR2_CTRL1_Pin, 1);
  HAL_GPIO_WritePin(MOTOR2_CTRL2_GPIO_Port, MOTOR2_CTRL2_Pin, 1);
}

void turnLeft()
{
  //电机2反转
//    __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_3, pwmVal-50);    //修改比较值，修改占空�?????
    __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_3, pwmVal - 45);    //修改比较值，修改占空�?????
  HAL_GPIO_WritePin(MOTOR1_CTRL1_GPIO_Port, MOTOR1_CTRL1_Pin, 1);
  HAL_GPIO_WritePin(MOTOR1_CTRL2_GPIO_Port, MOTOR1_CTRL2_Pin, 0);
  HAL_GPIO_WritePin(MOTOR2_CTRL1_GPIO_Port, MOTOR2_CTRL1_Pin, 1);
  HAL_GPIO_WritePin(MOTOR2_CTRL2_GPIO_Port, MOTOR2_CTRL2_Pin, 1);
}

void turnRight()
{
  //电机1反转
//  __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_4, pwmVal-40);    //修改比较值，修改占空�?????
    __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_4, pwmVal - 40);    //修改比较值，修改占空�?????
  HAL_GPIO_WritePin(MOTOR1_CTRL1_GPIO_Port, MOTOR1_CTRL1_Pin, 1);
  HAL_GPIO_WritePin(MOTOR1_CTRL2_GPIO_Port, MOTOR1_CTRL2_Pin, 1);
  HAL_GPIO_WritePin(MOTOR2_CTRL1_GPIO_Port, MOTOR2_CTRL1_Pin, 0);
  HAL_GPIO_WritePin(MOTOR2_CTRL2_GPIO_Port, MOTOR2_CTRL2_Pin, 1);
}

void sr04Init()
{
  sr04TrigHigh();
  delayus(20);
  sr04TrigLow();
}

void sr04TimerMode(uint8_t mode)
{
  //mode=1-->open timer
  //mode=0-->close timer
  if (mode)
  {
      __HAL_TIM_SetCounter(&htim2, 0);
    HAL_TIM_Base_Start(&htim2);
    //HAL_TIM_Base_Start_IT(&htim2);
    msHcCount = 0;
  } else
  {
    HAL_TIM_Base_Stop(&htim2);
    //HAL_TIM_Base_Stop_IT(&htim2);
  }
}

float sr04GetDistant()
{
  sr04Init();
  while (!sr04Echo());
  sr04TimerMode(1);//start timer
  while (sr04Echo());
  sr04TimerMode(0);//stop timer

  return (__HAL_TIM_GetCounter(&htim2)) / 58.0;
}

float sr04GetDistantAfterFilter(uint8_t cnt)
{
  float sum = 0;
  for (int i = 0; i < cnt; i++)
  {
    sum += sr04GetDistant();
  }
  return sum / cnt;
}

float pidOutput()
{
  float err, errPre;
  static float integral;
  float output;

  if (TRACK1 == 1 && TRACK2 == 1 && TRACK3 == 0 && TRACK4 == 1 && TRACK5 == 1) err = 0;

  else if (TRACK1 == 1 && TRACK2 == 0 && TRACK3 == 1 && TRACK4 == 1 && TRACK5 == 1) err = -1;
  else if (TRACK1 == 0 && TRACK2 == 0 && TRACK3 == 1 && TRACK4 == 1 && TRACK5 == 1) err = -2;
  else if (TRACK1 == 0 && TRACK2 == 1 && TRACK3 == 1 && TRACK4 == 1 && TRACK5 == 1) err = -3;

  else if (TRACK1 == 1 && TRACK2 == 1 && TRACK3 == 1 && TRACK4 == 0 && TRACK5 == 1) err = 1;
  else if (TRACK1 == 1 && TRACK2 == 1 && TRACK3 == 1 && TRACK4 == 0 && TRACK5 == 0) err = 2;
  else if (TRACK1 == 1 && TRACK2 == 1 && TRACK3 == 1 && TRACK4 == 1 && TRACK5 == 0) err = 3;

  else err = 0;

  integral += err;
  output = kp * err + ki * integral + kd * (err - errPre);
  errPre = err;
  return output;
}

void littleCarMove()
{
  while (1)
  {
    HAL_UART_Receive(&huart1, &msg, 1, 10);

    if (msg == '3') //emergency stop by usart
    {
      msg = 0;
      HAL_UART_Transmit(&huart1, "Stop By User.\r\n", sizeof("Stop By User.\r\n"), 13);
      stopAll();
      return;
    }

    if (cntWhile % 10 == 0) //per 10 time scan distant, if too close, stop.
    {
      float distant = sr04GetDistantAfterFilter(5);
      if (distant < 20)
      {
        msg = 0;
        HAL_UART_Transmit(&huart1, "Too Close! Stop!\r\n", sizeof("Too Close! Stop!\r\n"), 17);

        stopAll();
        return;
      }
    }

    if (irStop == 1)
    {
      stopAll();
      irStop = 0; //if stop by ir, reset flag.
      return;
    }

    if (firstTimeRun) //run anyway when on power.
    {
      goForward();
      HAL_Delay(1500);
      firstTimeRun = 0;
    }

    if (TRACK2 == 0 || TRACK4 == 0)
    {
      if (TRACK2 == 0)
      {
        HAL_Delay(10);
        if (TRACK2 == 0)
        {
          turnLeft();
        }
      }
      if (TRACK4 == 0)
      {
        HAL_Delay(10);
        if (TRACK4 == 0)
        {
          turnRight();
        }
      }
    } else if (TRACK1 == 0 || TRACK5 == 0)
    {
      if (TRACK1 == 0)
      {
        HAL_Delay(11);
        if (TRACK1 == 0)
        {
          turnLeft();
          while (TRACK3 == 1);
        }
      }

      if (TRACK5 == 0)
      {
        HAL_Delay(11);
        if (TRACK5 == 0)
        {
          turnRight();
          while (TRACK3 == 1);
        }
      }
    } else goForward();

    if (tickFlag == 1)
    {
      HAL_UART_Transmit(&huart1, "Running...\r\n", sizeof("Running...\r\n"), 8);
      tickFlag = 0;
    }
    cntWhile++;
  }
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

