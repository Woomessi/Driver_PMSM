/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SCLK_Pin GPIO_PIN_2
#define SCLK_GPIO_Port GPIOE
#define _RESET_Pin GPIO_PIN_3
#define _RESET_GPIO_Port GPIOE
#define DIR_Pin GPIO_PIN_4
#define DIR_GPIO_Port GPIOE
#define SDO_Pin GPIO_PIN_5
#define SDO_GPIO_Port GPIOE
#define SDI_Pin GPIO_PIN_6
#define SDI_GPIO_Port GPIOE
#define _SAMPLE_Pin GPIO_PIN_9
#define _SAMPLE_GPIO_Port GPIOF
#define _SOE_Pin GPIO_PIN_10
#define _SOE_GPIO_Port GPIOF
#define _WR_Pin GPIO_PIN_2
#define _WR_GPIO_Port GPIOC
#define _RD_Pin GPIO_PIN_3
#define _RD_GPIO_Port GPIOC
#define _CS_Pin GPIO_PIN_2
#define _CS_GPIO_Port GPIOF
#define RES0_Pin GPIO_PIN_10
#define RES0_GPIO_Port GPIOC
#define RES1_Pin GPIO_PIN_11
#define RES1_GPIO_Port GPIOC
#define DC_CAL_Pin GPIO_PIN_0
#define DC_CAL_GPIO_Port GPIOD
#define EN_GATE_Pin GPIO_PIN_1
#define EN_GATE_GPIO_Port GPIOD
#define NFAULT_Pin GPIO_PIN_3
#define NFAULT_GPIO_Port GPIOD
#define NOCTW_Pin GPIO_PIN_4
#define NOCTW_GPIO_Port GPIOD
#define SDI_DRV8301_Pin GPIO_PIN_5
#define SDI_DRV8301_GPIO_Port GPIOD
#define SDO_DRV8301_Pin GPIO_PIN_6
#define SDO_DRV8301_GPIO_Port GPIOD
#define SCLK_DRV8301_Pin GPIO_PIN_7
#define SCLK_DRV8301_GPIO_Port GPIOD
#define NSCS_DRV8301_Pin GPIO_PIN_3
#define NSCS_DRV8301_GPIO_Port GPIOB
#define A0_Pin GPIO_PIN_4
#define A0_GPIO_Port GPIOB
#define A1_Pin GPIO_PIN_5
#define A1_GPIO_Port GPIOB
#define DOS_Pin GPIO_PIN_0
#define DOS_GPIO_Port GPIOE
#define LOT_Pin GPIO_PIN_1
#define LOT_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
