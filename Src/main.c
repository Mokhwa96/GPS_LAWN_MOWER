/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* ======================== 헤더 파일 포함 ======================== */
#include "main.h"
#include "tim.h"        /* TIM1 타이머 드라이버 */
#include "usart.h"      /* USART1, USART2 UART 드라이버 */
#include "gpio.h"       /* GPIO 핀 설정 드라이버 */

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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

/* USER CODE END PV */

/* ==================== 함수 프로토타입 선언 ==================== */
void SystemClock_Config(void);  /* 시스템 클록 설정 함수 */
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  애플리케이션 진입점 (메인 함수)
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* ============================================================ */
  /*                    MCU 초기 설정 시작                          */
  /* ============================================================ */

  /* HAL 라이브러리 초기화: Flash, Systick 등 기본 설정 */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* 시스템 클록 설정 (HSI 16MHz 사용) */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* ============================================================ */
  /*                  주변장치(Peripheral) 초기화                    */
  /* ============================================================ */
  MX_GPIO_Init();             /* GPIO 핀 초기화 */
  MX_TIM1_Init();             /* TIM1 타이머 초기화 */
  MX_USART1_UART_Init();      /* USART1 UART 초기화 */
  MX_USART2_UART_Init();      /* USART2 UART 초기화 */
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* ============================================================ */
  /*                       메인 루프                               */
  /* ============================================================ */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
      
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief  시스템 클록 설정 함수
  * @note   HSI(내부 고속 오실레이터) 16MHz를 시스템 클록 소스로 사용
  *         PLL 미사용, 모든 버스 분주비 1:1
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};   /* 오실레이터 설정 구조체 */
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};   /* 클록 설정 구조체 */

  /* ---- 내부 전압 레귤레이터 출력 전압 설정 (Scale 3) ---- */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /* ---- RCC 오실레이터 설정: HSI 16MHz 활성화, PLL 미사용 ---- */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;         /* HSI 오실레이터 선택 */
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;                           /* HSI 활성화 */
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT; /* HSI 캘리브레이션 기본값 */
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;                     /* PLL 사용 안 함 */
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();  /* 오실레이터 설정 실패 시 에러 처리 */
  }

  /* ---- CPU, AHB, APB 버스 클록 설정 ----
   *  SYSCLK = HSI = 16MHz
   *  HCLK   = SYSCLK / 1 = 16MHz  (AHB 버스)
   *  PCLK1  = HCLK / 1   = 16MHz  (APB1 버스)
   *  PCLK2  = HCLK / 1   = 16MHz  (APB2 버스)
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;   /* SYSCLK 소스: HSI */
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;       /* AHB 분주비: 1 */
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;        /* APB1 분주비: 1 */
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;        /* APB2 분주비: 1 */

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();  /* 클록 설정 실패 시 에러 처리 */
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  에러 핸들러 - HAL 에러 발생 시 호출됨
  * @note   인터럽트를 비활성화하고 무한 루프에 진입하여 시스템 정지
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();    /* 모든 인터럽트 비활성화 */
  while (1)
  {
    /* 에러 발생 시 여기서 무한 대기 (디버거로 확인 가능) */
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  assert_param 매크로 실패 시 호출되는 함수
  * @param  file: assert 에러가 발생한 소스 파일 이름
  * @param  line: assert 에러가 발생한 소스 라인 번호
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* 사용 예: printf("잘못된 파라미터: 파일 %s, 라인 %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
