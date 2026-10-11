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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "uart_bsp.h"
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
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
/* Last counter values reported to the console, so the loop below only speaks
   up when something actually goes wrong instead of once a second forever. */
static uint32_t s_rx_dropped_reported;
static uint32_t s_tx_dropped_reported;
static uint32_t s_rx_errors_reported;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* Set to 1 to run a TX->RX self-test at boot instead of the echo console.
   PA9 (USART1_TX) must be wired to PA10 (USART1_RX) for it to pass. The echo
   console is skipped in that mode on purpose: with the jumper fitted, its own
   replies arrive back on RX and it would echo itself in circles. */
#define UART_LOOPBACK_SELFTEST   0

/* Minimal decimal printer. A full printf() would mean retargeting the C
   library's _write()/_sys_write(), which is a lot of machinery for showing
   three counters. */
static void uart_put_u32(uint32_t value)
{
  char    digits[10];
  uint8_t n = 0U;

  if (value == 0U)
  {
    uart_putc((uint8_t)'0');
    return;
  }

  while ((value > 0U) && (n < (uint8_t)sizeof(digits)))
  {
    digits[n] = (char)('0' + (value % 10U));
    value /= 10U;
    n++;
  }

  while (n > 0U)
  {
    n--;
    uart_putc((uint8_t)digits[n]);
  }
}

#if UART_LOOPBACK_SELFTEST
/**
  * @brief  Push two blocks of bytes out of TX and check they come back on RX.
  * @note   Two rounds, because the second one wraps the ring buffer past the
  *         end of its storage -- where an off-by-one in the wrap would hide.
  * @retval 1 if every byte matched, 0 otherwise.
  */
static int uart_loopback_test(void)
{
  /* Deliberately smaller than UART_RX_BUFFER_SIZE - 1, so a single burst
     cannot overflow the queue and be reported as a failure. */
  uint8_t  tx_pattern[200];
  uint8_t  rx_pattern[200];
  uint32_t deadline;
  uint16_t round;
  uint16_t i;
  int      ok = 1;

  for (round = 0U; (round < 2U) && (ok != 0); round++)
  {
    for (i = 0U; i < (uint16_t)sizeof(tx_pattern); i++)
    {
      tx_pattern[i] = (uint8_t)(i + round);
    }

    uart_write(tx_pattern, (uint16_t)sizeof(tx_pattern));
    uart_tx_flush();

    /* The last byte reaches the receiver right as TC sets, so give the RX
       interrupt a moment to land before judging the result. */
    deadline = HAL_GetTick() + 50U;
    while ((uart_available() < (uint16_t)sizeof(rx_pattern)) &&
           ((int32_t)(HAL_GetTick() - deadline) < 0))
    {
    }

    if (uart_available() != (uint16_t)sizeof(rx_pattern))
    {
      ok = 0;
      break;
    }
    if (uart_read(rx_pattern, (uint16_t)sizeof(rx_pattern)) != (uint16_t)sizeof(rx_pattern))
    {
      ok = 0;
      break;
    }

    for (i = 0U; i < (uint16_t)sizeof(rx_pattern); i++)
    {
      if (rx_pattern[i] != tx_pattern[i])
      {
        ok = 0;
        break;
      }
    }
  }

  return ok;
}
#endif /* UART_LOOPBACK_SELFTEST */

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
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  uart_bsp_init();

#if UART_LOOPBACK_SELFTEST
  if (uart_loopback_test() != 0)
  {
    uart_puts("\r\nloopback: PASS\r\n");
  }
  else
  {
    uart_puts("\r\nloopback: FAIL\r\n");
  }

  /* The jumper is still fitted, so the echo console below would receive its own
     replies and echo them in circles. Park here rather than run it. */
  while (1)
  {
  }
#else
  uart_puts("\r\nLynx UART ring buffer ready\r\n");
  uart_puts("type a line and press enter\r\n");
#endif
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    char line[64];

    /* timeout 0: poll the queue and return at once, so the loop never blocks. */
    switch (uart_read_line(line, (uint16_t)sizeof(line), 0U))
    {
      case UART_LINE_OK:
        uart_puts("echo: ");
        uart_puts(line);
        uart_puts("\r\n");
        break;

      case UART_LINE_OVERFLOW:
        uart_puts("error: line too long, discarded\r\n");
        break;

      default:
        break;   /* UART_LINE_TIMEOUT: nothing typed yet */
    }

    /* Only speak up when a counter moves. Climbing rx_dropped means the main
       loop stalls too long between polls; climbing tx_dropped means the console
       produces output faster than the line can carry it. */
    if ((s_rx_dropped_reported != uart_rx_dropped()) ||
        (s_tx_dropped_reported != uart_tx_dropped()) ||
        (s_rx_errors_reported != uart_rx_errors()))
    {
      s_rx_dropped_reported = uart_rx_dropped();
      s_tx_dropped_reported = uart_tx_dropped();
      s_rx_errors_reported  = uart_rx_errors();

      uart_puts("stats: rx_dropped=");
      uart_put_u32(s_rx_dropped_reported);
      uart_puts(" tx_dropped=");
      uart_put_u32(s_tx_dropped_reported);
      uart_puts(" rx_errors=");
      uart_put_u32(s_rx_errors_reported);
      uart_puts("\r\n");
    }
    /* USER CODE END 3 */
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
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  HAL_RCC_MCOConfig(RCC_MCO, RCC_MCO1SOURCE_PLLCLK, RCC_MCODIV_1);
}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
#ifdef USE_FULL_ASSERT
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
