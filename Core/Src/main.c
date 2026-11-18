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
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "arm_math.h"
#include "math_helper.h"
#include "imu.h"
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* ---- FIR lowpass example (from CMSIS-DSP arm_fir_example_f32.c) ---- */
#define TEST_LENGTH_SAMPLES  320
#define SNR_THRESHOLD_F32    140.0f
#define BLOCK_SIZE            32
#define NUM_TAPS              29

/* ---- real accelerometer data ---- */
#define ACC_FS_HZ            100   /* TIM2 取樣率 (Hz)：PSC=79, ARR=9999 -> 80MHz/80/10000 */
#define ACC_CUTOFF_HZ          5   /* FIR 截止頻率 (Hz) */
#define ACC_NUM_TAPS          29   /* 係數個數 = 每筆輸出回看幾筆 */
#define ACC_BLOCK_SIZE        32   /* 一次 arm_fir_f32 處理幾筆 */
#define ACC_CAPTURE_LEN      512   /* 必須是 ACC_BLOCK_SIZE 的整數倍 (512/32 = 16 blocks) */
#define ACC_AXIS               2   /* 0 = X, 1 = Y, 2 = Z */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */


/* ---- FIR example state ---- */
arm_fir_instance_f32 S;
arm_status status;
float32_t *inputF32;
float32_t *outputF32;
uint32_t i;

/* input signal (1 kHz + 15 kHz) and MATLAB reference output, in arm_fir_data.c */
extern float32_t testInput_f32_1kHz_15kHz[TEST_LENGTH_SAMPLES];
extern float32_t refOutput[TEST_LENGTH_SAMPLES];

static float32_t testOutput[TEST_LENGTH_SAMPLES];
static float32_t firStateF32[BLOCK_SIZE + NUM_TAPS - 1];

/* fir1(28, 6/24) : fs = 48 kHz, cutoff = 6 kHz, 29 taps */
const float32_t firCoeffs32[NUM_TAPS] = {
  -0.0018225230f, -0.0015879294f, +0.0000000000f, +0.0036977508f, +0.0080754303f, +0.0085302217f, -0.0000000000f, -0.0173976984f,
  -0.0341458607f, -0.0333591565f, +0.0000000000f, +0.0676308395f, +0.1522061835f, +0.2229246956f, +0.2504960933f, +0.2229246956f,
  +0.1522061835f, +0.0676308395f, +0.0000000000f, -0.0333591565f, -0.0341458607f, -0.0173976984f, -0.0000000000f, +0.0085302217f,
  +0.0080754303f, +0.0036977508f, +0.0000000000f, -0.0015879294f, -0.0018225230f
};

uint32_t blockSize = BLOCK_SIZE;
uint32_t numBlocks = TEST_LENGTH_SAMPLES / BLOCK_SIZE;
float32_t snr;

/* ---- real accelerometer data ---- */
/* firwin(29, 5, fs=100)：fs = 100 Hz、cutoff = 5 Hz、Hamming window、DC gain = 1.0 */
const float32_t accCoeffs32[ACC_NUM_TAPS] = {
  -0.0017925669f, -0.0018788835f, -0.0020285056f, -0.0016712100f, +0.0000000000f, +0.0038552603f, +0.0106060380f, +0.0205854550f,
  +0.0335846185f, +0.0487894686f, +0.0648420439f, +0.0800227460f, +0.0925224259f, +0.1007515124f, +0.1036231948f, +0.1007515124f,
  +0.0925224259f, +0.0800227460f, +0.0648420439f, +0.0487894686f, +0.0335846185f, +0.0205854550f, +0.0106060380f, +0.0038552603f,
  +0.0000000000f, -0.0016712100f, -0.0020285056f, -0.0018788835f, -0.0017925669f
};

arm_fir_instance_f32 Sacc;
static float32_t accStateF32[ACC_BLOCK_SIZE + ACC_NUM_TAPS - 1];   /* 32 + 29 - 1 = 60 */
static float32_t accRaw[ACC_CAPTURE_LEN];
static float32_t accFilt[ACC_CAPTURE_LEN];

volatile uint8_t  acc_tick   = 0;   /* 由 TIM2 中斷設定 */
volatile uint32_t acc_missed = 0;   /* 來不及處理而漏掉的樣本數 */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

static void print_f32(const char *label, float32_t v, const char *unit);
static void acc_capture_and_filter(void);
static void acc_dump_csv(void);
int __io_putchar(int ch);
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
  MX_I2C2_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  setvbuf(stdout, NULL, _IONBF, 0);   /* unbuffered: printf goes out immediately */

  printf("\r\n=== CMSIS-DSP FIR lowpass example (f32) ===\r\n");
  printf("fs = 48 kHz, input = 1 kHz + 15 kHz, cutoff = 6 kHz, %d taps\r\n", NUM_TAPS);
  printf("samples = %lu, block size = %lu, blocks = %lu\r\n\r\n",
         (unsigned long)TEST_LENGTH_SAMPLES, (unsigned long)blockSize, (unsigned long)numBlocks);

  /* Initialize input and output buffer pointers */
  inputF32  = &testInput_f32_1kHz_15kHz[0];
  outputF32 = &testOutput[0];

  /* Call FIR init function to initialize the instance structure. */
  arm_fir_init_f32(&S, NUM_TAPS, (float32_t *)&firCoeffs32[0], &firStateF32[0], blockSize);

  /* Call the FIR process function for every blockSize samples */
  for (i = 0; i < numBlocks; i++)
  {
    arm_fir_f32(&S, inputF32 + (i * blockSize), outputF32 + (i * blockSize), blockSize);
  }

  /* Compare the generated output against the reference output computed in MATLAB */
  snr = arm_snr_f32(&refOutput[0], &testOutput[0], TEST_LENGTH_SAMPLES);

  status = (snr < SNR_THRESHOLD_F32) ? ARM_MATH_TEST_FAILURE : ARM_MATH_SUCCESS;

  print_f32("SNR vs MATLAB reference = ", snr, " dB\r\n");
  printf("threshold = 140.000 dB  ->  %s\r\n\r\n",
         (status == ARM_MATH_SUCCESS) ? "PASS" : "FAIL");

  /* Dump a window of samples so the filtering can be checked visually / plotted */
  printf("idx,input,output,reference\r\n");
  for (i = 0; i < 64; i++)
  {
    printf("%lu,  ", (unsigned long)i);
    print_f32("", inputF32[i],    ",");
    print_f32("", testOutput[i],  ",");
    print_f32("", refOutput[i],   "\r\n");
  }
  printf("\r\n--- known-signal test done ---\r\n\r\n");

  /* ---- real sensor path: LSM6DSL over I2C2 ---- */
  if (imu_init())
  {
    printf("LSM6DSL ready, WHO_AM_I = 0x%02X, acc ODR = 104 Hz, +-2 g\r\n", imu_who_am_i);
  }
  else
  {
    printf("LSM6DSL NOT found: WHO_AM_I = 0x%02X, expected 0x6A\r\n", imu_who_am_i);
    printf("check PB10 = I2C2_SCL, PB11 = I2C2_SDA and the I2C2 clock\r\n");
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    uint8_t key;

    printf("\r\n=== press any key: capture %d samples of axis %c at %d Hz (%d ms) ===\r\n",
           ACC_CAPTURE_LEN, "XYZ"[ACC_AXIS], ACC_FS_HZ,
           ACC_CAPTURE_LEN * 1000 / ACC_FS_HZ);
    HAL_UART_Receive(&huart1, &key, 1, HAL_MAX_DELAY);

    printf("capturing, shake the board now ...\r\n");
    acc_capture_and_filter();
    printf("captured. dropped samples = %lu\r\n", (unsigned long)acc_missed);
    printf("FIR: %d taps, cutoff %d Hz, group delay %d samples (%d ms)\r\n\r\n",
           ACC_NUM_TAPS, ACC_CUTOFF_HZ,
           (ACC_NUM_TAPS - 1) / 2, (ACC_NUM_TAPS - 1) * 1000 / 2 / ACC_FS_HZ);

    acc_dump_csv();
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
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
}

/* USER CODE BEGIN 4 */

/**
  * @brief Redirect printf / putchar to USART1.
  */
int __io_putchar(int ch)
{
  uint8_t c = (uint8_t)ch;
  HAL_UART_Transmit(&huart1, &c, 1, HAL_MAX_DELAY);
  return ch;
}

/**
  * @brief TIM2 每 10 ms 觸發一次，只設 flag。
  *        不在這裡呼叫 imu_read()：HAL_I2C_Mem_Read() 是 blocking 的，
  *        而且內部用 HAL_GetTick() 判斷 timeout，在 ISR 裡等會有死鎖風險。
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM2)
  {
    if (acc_tick)
    {
      acc_missed++;      /* 上一筆還沒被取走就又來了 -> 漏樣本 */
    }
    acc_tick = 1;
  }
}

/**
  * @brief 以 ACC_FS_HZ 擷取 ACC_CAPTURE_LEN 筆單軸加速度，然後整段跑 FIR。
  */
static void acc_capture_and_filter(void)
{
  int32_t  acc_mg[3], gyr_mdps[3];
  uint32_t n, b;

  acc_missed = 0;
  acc_tick   = 0;

  HAL_TIM_Base_Start_IT(&htim2);
  n = 0;
  while (n < ACC_CAPTURE_LEN)
  {
    if (acc_tick)
    {
      acc_tick = 0;
      imu_read(acc_mg, gyr_mdps);
      accRaw[n++] = (float32_t)acc_mg[ACC_AXIS];
    }
  }
  HAL_TIM_Base_Stop_IT(&htim2);

  /* 真實資料用自己的 instance 和 state buffer：
     和已知訊號那組共用會讓開頭 28 筆吃到殘留值。 */
  arm_fir_init_f32(&Sacc, ACC_NUM_TAPS, (float32_t *)&accCoeffs32[0],
                   &accStateF32[0], ACC_BLOCK_SIZE);

  for (b = 0; b < ACC_CAPTURE_LEN / ACC_BLOCK_SIZE; b++)
  {
    arm_fir_f32(&Sacc, accRaw  + b * ACC_BLOCK_SIZE,
                       accFilt + b * ACC_BLOCK_SIZE, ACC_BLOCK_SIZE);
  }
}

/**
  * @brief 把擷取結果印成 CSV，可直接貼到 Excel / Python 畫圖。
  */
static void acc_dump_csv(void)
{
  uint32_t n;

  printf("idx,t_ms,raw_mg,filtered_mg\r\n");
  for (n = 0; n < ACC_CAPTURE_LEN; n++)
  {
    printf("%lu,%lu,", (unsigned long)n,
           (unsigned long)(n * 1000UL / ACC_FS_HZ));
    print_f32("", accRaw[n],  ",");
    print_f32("", accFilt[n], "\r\n");
  }
}

/**
  * @brief Print a float with 3 decimals.
  *        newlib-nano (--specs=nano.specs) does not support %f in printf,
  *        so the value is split into integer and fractional parts.
  */
static void print_f32(const char *label, float32_t v, const char *unit)
{
  int neg = (v < 0.0f);
  int ip, fp;

  if (neg)
  {
    v = -v;
  }
  ip = (int)v;
  fp = (int)((v - (float32_t)ip) * 1000.0f + 0.5f);
  if (fp >= 1000)
  {
    ip += 1;
    fp -= 1000;
  }
  printf("%s%s%d.%03d%s", label, neg ? "-" : "", ip, fp, unit);
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
