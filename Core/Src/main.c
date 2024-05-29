/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2024 STMicroelectronics.
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
#include "adc.h"
#include "comp.h"
#include "dac.h"
#include "dma.h"
#include "fdcan.h"
#include "opamp.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "sys.h"
#include "delay.h"
#include "AD2S1210.h"
#include "DRV8301.h"
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

/* USER CODE BEGIN PV */

/* AD2S1210 */
unsigned char buf[4] = {0, 0, 0, 0};
unsigned char error[4] = {0, 0, 0, 0};
float angle = 0;
unsigned short velocity0 = 0;
float velocity = 0;
//////////
	
#define PI					3.14159265358979f
#define PHASE_SHIFT_ANGLE (float)(120.0f/360.0f*2.0f*PI)

extern DMA_HandleTypeDef hdma_usart1_tx;

#define RXBUFFERSIZE  256   
char RxBuffer[RXBUFFERSIZE];  
uint8_t aRxBuffer;			   
uint8_t Uart1_Rx_Cnt = 0;	

//float temp[1]; //Vbus
//static uint8_t tempData[8] = {0,0,0,0,0,0,0x80,0x7F}; //Vbus

//float temp[4]; //opamp, PWM
//static uint8_t tempData[20] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0x80,0x7F}; //opamp, PWM

float load_data[7];
static uint8_t tempData[32] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0x80,0x7F};

uint16_t DAC_temp = 0;

float Vbus,Ia,Ib,Ic;
uint8_t Motor_state = 0;
uint16_t IA_Offset,IB_Offset,IC_Offset;
uint16_t adc1_in1, adc1_in2, adc1_in3, Vpoten, adc_vbus;
uint8_t ADC_offset = 0;
	
FDCAN_RxHeaderTypeDef RxHeader;
FDCAN_TxHeaderTypeDef TxHeader;
uint8_t RxData[8]={NULL};
uint8_t TxData[8] = {NULL};
float CANtemp[1];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
int fputc(int ch, FILE *f);
int fgetc(FILE *f);
void FDCAN_Config(void);

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
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_OPAMP1_Init();
  MX_OPAMP2_Init();
  MX_OPAMP3_Init();
  MX_TIM1_Init();
  MX_COMP1_Init();
  MX_DAC3_Init();
  MX_DAC1_Init();
  MX_FDCAN1_Init();
  /* USER CODE BEGIN 2 */
	delay_init(160);	
	
	/* AD2S1210 */
	AD2S1210Initiate(); // 上电时序控制和复位
	
  CLR_SOE();
  SET_RD(); // 串口通信需将RD拉高
	
	AD2S1210SelectMode(CONFIG); // 进入配置模式，对寄存器进行编程，以设置AD2S1210的激励频率、分辨率和故障检测阈值
	
	/* 检查读写功能是否正常 */
  WriteToAD2S1210(CONTROL, 0x7A); //默认0x7E
  ReadFromAD2S1210(CONFIG, CONTROL, buf);
	
	/* DRV8301 */	
  DRV8301Cfg cfg = {
			.GATE_CURRENT = CURRENT0_7A,//","
			.GATE_RESET = GATE_RST_NORMAL,
			.PWM_MODE = PHASE6PWM,
			.OCP_MODE = OCLATCHSTDOWN,
			.OC_ADJ_SET = 18,
			.OCTW_MODE = OCOTBOTH,
			.GAINVALUE = GAIN10,
			.DC_CAL_CH1 = CONNECTLOADPH1,
			.DC_CAL_CH2 = CONNECTLOADPH2,
			.OC_TOFF = CYCLEBYCYCLE//没有","
	};
	spi3init();
	//delay_ms(30000);
	DRV8301Init(cfg);
	uint16_t id = DRV8301IDread();
	printf("ID = %d\r\n", id);
	
	/* Vbus */
//	HAL_UART_Receive_IT(&huart1, (uint8_t *)&aRxBuffer, 1);
//	HAL_ADCEx_Calibration_Start( &hadc1, ADC_SINGLE_ENDED);
//	HAL_ADCEx_Calibration_Start( &hadc2, ADC_SINGLE_ENDED);
	
  /* opamp */
	HAL_OPAMP_Start(&hopamp1);
	HAL_OPAMP_Start(&hopamp2);
	HAL_OPAMP_Start(&hopamp3);
	HAL_UART_Receive_IT(&huart1, (uint8_t *)&aRxBuffer, 1); //?
	HAL_ADCEx_Calibration_Start( &hadc1, ADC_SINGLE_ENDED);
	HAL_ADCEx_Calibration_Start( &hadc2, ADC_SINGLE_ENDED);
	
	/* PWM */
//	TIM1->PSC = 30000;
//	TIM1->ARR = 10000;
//	TIM1->CCR1 = 2000;
//	TIM1->CCR2 = 5000;
//	TIM1->CCR3 = 8000;
	
	/* comp */
//	TIM1->PSC = 30000;
	TIM1->ARR = 8000 - 1;
	TIM1->CCR4 = 8000 - 2;
	TIM1->CCR1 = 2000;
	TIM1->CCR2 = 4000;
	TIM1->CCR3 = 6000;
	
	HAL_TIM_Base_Start( &htim1);
	HAL_TIM_PWM_Start( &htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start( &htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start( &htim1, TIM_CHANNEL_3);
	HAL_TIMEx_PWMN_Start( &htim1, TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Start( &htim1, TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Start( &htim1, TIM_CHANNEL_3);
	
	__HAL_ADC_CLEAR_FLAG( &hadc1, ADC_FLAG_JEOC);
	__HAL_ADC_CLEAR_FLAG( &hadc1, ADC_FLAG_EOC);
	__HAL_ADC_CLEAR_FLAG( &hadc2, ADC_FLAG_JEOC);
	HAL_TIM_PWM_Start( &htim1, TIM_CHANNEL_4);
	HAL_ADCEx_InjectedStart_IT(&hadc1);
	HAL_ADCEx_InjectedStart(&hadc2);
	
//	HAL_DAC_Start(&hdac3,DAC_CHANNEL_1);
//	HAL_DAC_Start(&hdac1,DAC_CHANNEL_1);
//	HAL_DAC_SetValue(&hdac3,DAC_CHANNEL_1,DAC_ALIGN_12B_R,3000);
//	HAL_COMP_Start (&hcomp1);	
	
	/* can */
//	FDCAN_Config();


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		
		/* AD2S1210 */
//    AD2S1210SelectMode(POSITION);
//    ReadFromAD2S1210(POSITION, POS_VEL, buf); // read data register
//    angle = ((buf[2] << 8) | buf[1]) >> 4;
//    angle = angle * 360 / 4095;
//    printf("degree: %f\n", angle);
//    printf("error: %X\n", buf[0]);

//    AD2S1210SelectMode(VELOCITY);              // Normal Mode velocity output
//    ReadFromAD2S1210(VELOCITY, POS_VEL, buf);  // read data register
//    velocity0 = ((buf[2] << 8) | buf[1]) >> 4; // 12位补码
//    if ((velocity0 & 0x800) >> 11)
//    {
//      if (velocity0 == 0xFFF)
//      {
//        printf("rps: 0.000000\n");
//        printf("error: %X\n", buf[0]);
//      }
//      else
//      {
//        // velocity = (~(velocity0 & 0x7FF))& 0x7FF;
//        // velocity = velocity*1000/2048;

//        velocity = 4096 - velocity0; // 原码
//        velocity = velocity * 1000 / 2048;

//        printf("rps:minus %f\n", velocity);
//        printf("error: %X\n", buf[0]);
//      }
//    }
//    else
//    {
//      velocity = velocity0;
//      velocity = velocity * 1000 / 2048;
//      printf("rps: %f\n", velocity);
//      printf("error: %X\n", buf[0]);
//    }
//    delay_ms(100);

    /* PWM */
		if((GPIOA->IDR & GPIO_PIN_8) != 0)
		{
			load_data[0] = 1.0f;
		}
		else
		{
			load_data[0] = 0.0f;
		}
		if((GPIOA->IDR & GPIO_PIN_9) != 0)
		{
			load_data[1] = 3.0f;
		}
		else
		{
			load_data[1] = 2.0f;
		}
		if((GPIOA->IDR & GPIO_PIN_10) != 0)
		{
			load_data[2] = 5.0f;
		}
		else
		{
			load_data[2] = 4.0f;
		}

		/* Vbus */
		HAL_ADC_Start(&hadc1);
		HAL_ADC_Start(&hadc2);
		adc_vbus = HAL_ADC_GetValue(&hadc2);
		Vbus = adc_vbus*3.3f/4096*26;
    load_data[3] = Vbus;
//		memcpy(tempData, (uint8_t *)&load_data, sizeof(load_data));
//		HAL_UART_Transmit_DMA(&huart1,(uint8_t *)tempData,24);
		HAL_Delay(10);

		/* opamp */
//		HAL_ADC_Start(&hadc1);
//		HAL_ADCEx_InjectedStart_IT(&hadc1);
//		HAL_ADCEx_InjectedStart_IT(&hadc2);
//		temp[0] = HAL_ADC_GetValue(&hadc1);
//		temp[0] = temp[0]*3.3f/4096;
//		memcpy(tempData, (uint8_t *)&temp, sizeof(temp));
//		HAL_UART_Transmit_DMA(&huart1,(uint8_t *)tempData,5*4);
//		HAL_Delay(1);

		/* usart 注意，调试此部分时,容易将后续程序卡住 (HAL_MAX_DELAY)*/
//    HAL_UART_Transmit_DMA(&huart1,DataB1,sizeof(DataB1));
//    HAL_UART_Receive(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
//    printf("%d\r\n",ch);
		
		/* can */
//		HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData);

		
//		HAL_Delay(10);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the peripherals clocks
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_ADC12
                              |RCC_PERIPHCLK_FDCAN;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.FdcanClockSelection = RCC_FDCANCLKSOURCE_PCLK1;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12CLKSOURCE_SYSCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

int fputc(int ch, FILE *f)
{
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}

int fgetc(FILE *f)
{
  uint8_t ch;
  HAL_UART_Receive(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}

void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	static uint8_t cnt;
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hadc);
	if(hadc == &hadc1)
	{
		if(ADC_offset == 0)
		{
			cnt++;
			adc1_in1 = hadc1.Instance->JDR1;
			adc1_in2 = hadc2.Instance->JDR1;
			adc1_in3 = hadc1.Instance->JDR2;
			IA_Offset += adc1_in1;
			IB_Offset += adc1_in2;
			IC_Offset += adc1_in3;
			if(cnt >= 10)
			{
				ADC_offset = 1;
				IA_Offset = IA_Offset/10;
				IB_Offset = IB_Offset/10;
				IC_Offset = IC_Offset/10;
			}
		}
		else
		{
			adc1_in1 = hadc1.Instance->JDR1;
			Ia = (adc1_in1 - IA_Offset)*0.0193359375f;
			adc1_in2 = hadc2.Instance->JDR1;
			Ib = (adc1_in2 - IB_Offset)*0.0193359375f;
			adc1_in3 = hadc1.Instance->JDR2;
			Ic = (adc1_in3 - IC_Offset)*0.0193359375f;
//			TIM1->CCR1 = 2000;
//			TIM1->CCR2 = 4000;
//			TIM1->CCR3 = 6000;
			load_data[4] = Ia;
			load_data[5] = Ib;
			load_data[6] = Ic;

			memcpy(tempData, (uint8_t *)&load_data, sizeof(load_data));
			HAL_UART_Transmit_DMA(&huart1,(uint8_t *)tempData,8*4);
		}
	}
  /* NOTE : This function should not be modified. When the callback is needed,
            function HAL_ADCEx_InjectedConvCpltCallback must be implemented in the user file.
  */
}

void FDCAN_Config(void)
{
  FDCAN_FilterTypeDef sFilterConfig;
  
  HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
  
  sFilterConfig.IdType = FDCAN_EXTENDED_ID;  
  sFilterConfig.FilterIndex = 0;             
  sFilterConfig.FilterType = FDCAN_FILTER_RANGE;  
  sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  sFilterConfig.FilterID1 = 0x00000000;
  sFilterConfig.FilterID2 = 0x01ffffff;       
  HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig);
  
  TxHeader.Identifier = 0x1B;
  TxHeader.IdType = FDCAN_EXTENDED_ID;
  TxHeader.TxFrameType = FDCAN_DATA_FRAME;
  TxHeader.DataLength = FDCAN_DLC_BYTES_8;
  TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
  TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
  TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  TxHeader.MessageMarker = 0x52;

  HAL_FDCAN_Start(&hfdcan1);
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
