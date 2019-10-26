/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
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
#include "stm32f4xx_hal.h"

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
#define BlueB_Pin GPIO_PIN_13
#define BlueB_GPIO_Port GPIOC
#define A_Pin GPIO_PIN_0
#define A_GPIO_Port GPIOC
#define B_Pin GPIO_PIN_1
#define B_GPIO_Port GPIOC
#define C_Pin GPIO_PIN_2
#define C_GPIO_Port GPIOC
#define D_Pin GPIO_PIN_3
#define D_GPIO_Port GPIOC
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define E_Pin GPIO_PIN_4
#define E_GPIO_Port GPIOC
#define F_Pin GPIO_PIN_5
#define F_GPIO_Port GPIOC
#define A1_Pin GPIO_PIN_0
#define A1_GPIO_Port GPIOB
#define B1_Pin GPIO_PIN_1
#define B1_GPIO_Port GPIOB
#define C1_Pin GPIO_PIN_2
#define C1_GPIO_Port GPIOB
#define G_Pin GPIO_PIN_6
#define G_GPIO_Port GPIOC
#define H_Pin GPIO_PIN_7
#define H_GPIO_Port GPIOC
#define DS18B20_Pin GPIO_PIN_9
#define DS18B20_GPIO_Port GPIOC
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define D1_Pin GPIO_PIN_4
#define D1_GPIO_Port GPIOB
#define E1_Pin GPIO_PIN_5
#define E1_GPIO_Port GPIOB
#define F1_Pin GPIO_PIN_6
#define F1_GPIO_Port GPIOB
#define G1_Pin GPIO_PIN_7
#define G1_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
#define PC0_ON()			(A_GPIO_Port->BSRR = (A_Pin))
#define PC0_OFF()			(A_GPIO_Port->BSRR = (A_Pin)<<16)
#define PC0_TOGGLE()		{A_GPIO_Port->ODR ^= (A_Pin);};
#define PC0_FLASH()		do{LED1_TOGGLE(); LED1_TOGGLE;} while(0)

#define LD2_ON()			(LD2_GPIO_Port->BSRR = (LD2_Pin))
#define LD2_OFF()			(LD2_GPIO_Port->BSRR = (LD2_Pin)<<16)
#define LD2_TOGGLE()		{LD2_GPIO_Port->ODR ^= (LD2_Pin);};
#define LD2_FLASH()			do{LD2_TOGGLE(); LD2_TOGGLE();} while(0)
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
