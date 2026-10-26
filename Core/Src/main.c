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
#include <stdarg.h>
#include <stdio.h>
#include "string.h"
#include "bsp_can.h"
#include "CAN_receive.h"
#include "bsp_pid.h"


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* ================= 调试开关 ================= */
/* [审查修改] 本文件共 3 处标记"审查修改"：此处宏定义、主循环速度闭环、USART1 初始化。
 * 其他改动文件：stm32f4xx_hal_msp.c（串口 PA9）、stm32f4xx_hal_conf.h（HSE_VALUE）、MOTOR_SELF.ioc */
/* 板上晶振(MHz)：DJI C 板实测 12（方波指令电机有响应 = CAN 1Mbps 已通，晶振确为 12MHz）。
 * PLLM=6：12/6=2MHz VCO 输入 → 168MHz，官方 C 板同款配置。
 * !!! 改这里必须同步改 Core/Inc/stm32f4xx_hal_conf.h 的 HSE_VALUE !!! */
#define BOARD_HSE_MHZ        12        /* DJI C 板晶振 12 MHz（已实测确认：CAN 修通后电机有响应） */
#define BOARD_PLLM           6         /* 12/6 = 2 MHz VCO 输入 → 168 MHz，官方 C 板同款 */
/* !!! 改这里必须同步改 Core/Inc/stm32f4xx_hal_conf.h 的 HSE_VALUE !!! */

/* 1 = 方波诊断模式：1 s 给电流 / 1 s 归零。!!! 电机会空载飙速，仅用于排线诊断 !!!
 * 0 = 速度闭环（默认）：P 环把转子转速控在 TEST_TARGET_RPM，限幅 + 无反馈保护 */
#define TEST_MODE_PULSE      0
#define TEST_PULSE_CURRENT   1500      /* 方波电流：1500/16384 ≈ 1.8 A */

/* ---- 速度闭环参数（转子转速，C620 反馈值是减速器前转速，输出轴再除 19.2）---- */
#define TEST_TARGET_RPM      500       /* 目标转子转速，输出轴约 26 rpm */
#define CMD_LIMIT            2000      /* 电流限幅 2000/16384 ≈ 2.4 A，防起步过冲 */
#define SPEED_PID_PERIOD_MS  5         /* 与 Gitee 速度环控制周期一致 */

/* Gitee extra-energy-mechanism-code 的 PID 参数 */
#define SPEED_KP             15.0f
#define SPEED_KI             3.0f
#define SPEED_KD             0.75f
#define SPEED_I_SEPARATION   100.0f
#define SPEED_I_LIMIT        1000.0f
#define SPEED_D_GAMMA        0.1f

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
extern CAN_HandleTypeDef hcan1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void usart_printf(const char *format, ...)
{
  char buffer[128];
  va_list args;
  int length;

  va_start(args, format);
  length = vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);

  if (length > 0)
  {
    if (length >= (int)sizeof(buffer))
    {
      length = sizeof(buffer) - 1;
    }

    HAL_UART_Transmit(&huart1,
                      (uint8_t *)buffer,
                      (uint16_t)length,
                      HAL_MAX_DELAY);
  }
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
  MX_CAN1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  can_filter_init();
  PID_Init(&PID_M3508,
           POSITION_PID,
           CMD_LIMIT,
           SPEED_I_LIMIT,
           SPEED_KP,
           SPEED_KI,
           SPEED_KD,
           SPEED_I_SEPARATION,
           SPEED_D_GAMMA);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint32_t tick     = 0;   /* 1 kHz 发送节拍 */
  uint32_t dbg_tick = 0;   /* 10 ms 采样 CAN 寄存器 */
  uint32_t pid_tick = 0;   /* 5 ms 执行一次速度 PID */
  uint32_t rx_seen  = 0;   /* 上次看到的 0x201 收帧计数 */
  uint32_t rx_stamp = 0;   /* 最后一次收到 0x201 的时刻 */
  int16_t  cmd      = 0;   /* 当前下发的电流 */

  while (1)
  {
    uint32_t now = HAL_GetTick();


    if (g_can_id_cnt[0] != rx_seen)
    {
      rx_seen  = g_can_id_cnt[0];
      rx_stamp = now;
    }

    if ((uint32_t)(now - tick) >= 1U)          /* 1 ms 发一帧，C620 期望 1 kHz */
    {
      tick = now;

#if TEST_MODE_PULSE
      /* 方波诊断：1 s 通电 / 1 s 断电。四槽广播，防止 C620 拨码 ID 不是 1。
         !!! 电机会空载飙速，只准短时间诊断用 !!! */
      cmd = ((now / 1000U) & 1U) ? (int16_t)TEST_PULSE_CURRENT : 0;
      CAN_cmd_chassis(cmd, cmd, cmd, cmd);
#else
      /* 只在 100 ms 内收到过 0x201 反馈才输出，防止反馈断线后继续使用旧电流。 */
      if ((uint32_t)(now - rx_stamp) > 100U)
      {
        cmd = 0;
        PID_Reset(&PID_M3508);                     /* 防止恢复反馈时积分突变 */
      }
      else if ((uint32_t)(now - pid_tick) >= SPEED_PID_PERIOD_MS)
      {
        float out;

        pid_tick = now;
        out = PID_Calculate(&PID_M3508,
                            (float)TEST_TARGET_RPM,
                            (float)motor_chassis[0].speed_rpm);
        if (out > (float)CMD_LIMIT)
        {
          out = (float)CMD_LIMIT;
        }
        else if (out < -(float)CMD_LIMIT)
        {
          out = -(float)CMD_LIMIT;
        }
        cmd = (int16_t)out;
      }
      CAN_cmd_chassis(cmd, 0, 0, 0);                /* 闭环只驱动 ID=1 那台 */
#endif
    }

    if ((uint32_t)(now - dbg_tick) >= 10U)     /* 10 ms 采一次 CAN 寄存器 */
    {
      dbg_tick = now;
      can_debug_sample();
    }
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = BOARD_PLLM;   /* = HSE/2 → VCO 输入 2 MHz */
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
  hcan1.Init.Prescaler = 3;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_10TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_3TQ;
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
  /* [审查修改] 原为 HAL_HalfDuplex_Init（配合 PB6 半双工），改 PA9 普通异步后用标准 Init */
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
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();

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
