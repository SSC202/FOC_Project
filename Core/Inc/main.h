/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

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
#define DAC_RET_Pin GPIO_PIN_2
#define DAC_RET_GPIO_Port GPIOE
#define AD2S2_A1_Pin GPIO_PIN_4
#define AD2S2_A1_GPIO_Port GPIOE
#define AD2S2_RD_Pin GPIO_PIN_13
#define AD2S2_RD_GPIO_Port GPIOC
#define AD2S1_SAM_Pin GPIO_PIN_0
#define AD2S1_SAM_GPIO_Port GPIOF
#define AD2S1_RESET_Pin GPIO_PIN_1
#define AD2S1_RESET_GPIO_Port GPIOF
#define AD2S2_SAM_Pin GPIO_PIN_2
#define AD2S2_SAM_GPIO_Port GPIOC
#define AD2S2_RESET_Pin GPIO_PIN_3
#define AD2S2_RESET_GPIO_Port GPIOC
#define M1_V_Pin GPIO_PIN_4
#define M1_V_GPIO_Port GPIOC
#define M1_U_Pin GPIO_PIN_5
#define M1_U_GPIO_Port GPIOC
#define AD2S1_RD_Pin GPIO_PIN_2
#define AD2S1_RD_GPIO_Port GPIOB
#define DAC_LDAC_Pin GPIO_PIN_14
#define DAC_LDAC_GPIO_Port GPIOF
#define DAC_RETL_Pin GPIO_PIN_15
#define DAC_RETL_GPIO_Port GPIOF
#define DAC_SYNC_Pin GPIO_PIN_7
#define DAC_SYNC_GPIO_Port GPIOE
#define DAC_GAIN_Pin GPIO_PIN_14
#define DAC_GAIN_GPIO_Port GPIOE
#define AD2S2_PCS_Pin GPIO_PIN_10
#define AD2S2_PCS_GPIO_Port GPIOB
#define AD2S2_A0_Pin GPIO_PIN_12
#define AD2S2_A0_GPIO_Port GPIOB
#define AD2S1_PCS_Pin GPIO_PIN_11
#define AD2S1_PCS_GPIO_Port GPIOA
#define AD2S1_A0_Pin GPIO_PIN_15
#define AD2S1_A0_GPIO_Port GPIOA
#define AD2S1_DIR_Pin GPIO_PIN_6
#define AD2S1_DIR_GPIO_Port GPIOB
#define SPI1_CS_Pin GPIO_PIN_7
#define SPI1_CS_GPIO_Port GPIOB
#define AD2S1_A1_Pin GPIO_PIN_9
#define AD2S1_A1_GPIO_Port GPIOB
#define AD2S2_DIR_Pin GPIO_PIN_0
#define AD2S2_DIR_GPIO_Port GPIOE
#define SPI2_CS_Pin GPIO_PIN_1
#define SPI2_CS_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
