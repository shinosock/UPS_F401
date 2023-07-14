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
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/**
  * 函数功能: 重定向c库函数printf到DEBUG_USARTx
  * 输入参数: �?
  * �? �? �?: �?
  * �?    明：�?
  */
int fputc(int ch, FILE *f)
{
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xffff);
  return ch;
}
 
/**
  * 函数功能: 重定向c库函数getchar,scanf到DEBUG_USARTx
  * 输入参数: �?
  * �? �? �?: �?
  * �?    明：�?
  */
int fgetc(FILE *f)
{
  uint8_t ch = 0;
  HAL_UART_Receive(&huart1, &ch, 1, 0xffff);
  return ch;
}
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint16_t ADC_Value[DMAlenth];
uint32_t adC, adV, adIN;
uint16_t i;
uint16_t pulse = 0;
uint8_t mode = 2;
/*====================================================================================================
PID Function
The PID (比例、积分�?�微�?) function is used in mainly
control applications. PIDCalc performs one iteration of the PID
algorithm.
While the PID function works, main is just a dummy program showing
a typical usage.
=====================================================================================================*/
typedef struct PID
{
	double SetPoint;                                  //设定目标 Desired Value
	double SumError;                                  //误差累计
	double Proportion;                                //比例常数 Proportional Const
	double Integral;                                  //积分常数 Integral Const
	double Derivative;                                //微分常数 Derivative Const
	double LastError;                                 //Error[-1]
	double PrevError;                                 //Error[-2]
} PID;
static PID sPID;
static PID *sptr = &sPID;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void IncPIDInit20(void)
{
	sptr->SumError = 0;
	sptr->LastError = 0;                            // Error[-1]
	sptr->PrevError = 0;                            // Error[-2]
	sptr->Proportion = 0.012;                           // 比例常数 Proportional Const
	sptr->Integral = 0.0;                           // 积分常数Integral Const
	sptr->Derivative = 0.00001;                       // 微分常数 Derivative Const
	sptr->SetPoint = 328;                          
}

void IncPIDInit200(void)
{
	sptr->SumError = 0;
	sptr->LastError = 0;                            // Error[-1]
	sptr->PrevError = 0;                            // Error[-2]
	sptr->Proportion = 0.05;                           // 比例常数 Proportional Const
	sptr->Integral = 0.0;                           // 积分常数Integral Const
	sptr->Derivative = 0.0008;                       // 微分常数 Derivative Const
	sptr->SetPoint = 2630;                           
}

void IncPIDInitV(void)
{
	sptr->SumError = 0;
	sptr->LastError = 0;                            // Error[-1]
	sptr->PrevError = 0;                            // Error[-2]
	sptr->Proportion = 0.05;                           // 比例常数 Proportional Const
	sptr->Integral = 0.0;                           // 积分常数Integral Const
	sptr->Derivative = 0.001;                       // 微分常数 Derivative Const
	sptr->SetPoint = 3081;                           
}
double IncPIDCalc(double NextPoint)
{
    register double iError, iIncpid;                //当前误差
    iError = sptr->SetPoint - NextPoint;            //增量计算
    iIncpid = sptr->Proportion * iError             //E[k]�?
        - sptr->Integral * sptr->LastError          //E[k�?1]�?
            + sptr->Derivative * sptr->PrevError;   //E[k�?2]�?
    //存储误差，用于下次计�?
    sptr->PrevError = sptr->LastError;
    sptr->LastError = iError;
    //返回增量�?
    return(iIncpid);
}
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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  IncPIDInit20();
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ADC_Value, DMAlenth);
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if (0)
    {
      IncPIDInitV;
      mode = 2;
    } 
    if (0)
    {
      IncPIDInit200;
      mode = 1;
    }   
    if (0)
    {
      IncPIDInit20;
      mode = 0;
    } 
    switch (mode)
    {
    case 0:
      {
        TIM1->CCR1 = IncPIDCalc(adC);
 
        break;
      }
      case 1:
      {
        TIM1->CCR1 = IncPIDCalc(adC);
        break; 
      }
    
    default:
      break;
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 168;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
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
