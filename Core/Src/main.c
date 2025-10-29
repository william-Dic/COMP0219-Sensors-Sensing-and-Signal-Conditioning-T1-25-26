#include "main.h"
#include "nau7802.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

/* =================== 用户可配置区（打开工程后就在这里改数值） =================== */
/* --- Load-cell calibration --- */
#define ZERO_OFFSET_EST        142739L
#define COUNTS_PER_GRAM        429.540924f
#define GRAMS_PER_COUNT        0.002328067f
#define N_PER_COUNT_OVERRIDE   0.0f

/* --- Load-cell plate geometry --- */
#define PLATE_W_M      0.09f
#define PLATE_H_M      0.093f
#define AIR_RHO        1.20f
#define DRAG_CD        1.10f
#define CALIBRATED_C_OVERRIDE  0.005729634f

/* --- Load-cell filtering / sampling --- */
#define SAMPLE_PERIOD_MS   5
#define FILTER_ALPHA       0.15f

/* --- Wind-cup configuration --- */
#define WINDCUP_SAMPLE_PERIOD_MS    1000u
#define WINDCUP_PULSES_PER_REV      256
#define WINDCUP_CALIBRATION_FACTOR  0.05f
#define WINDCUP_FILTER_ALPHA        0.20f
#define WINDCUP_MIN_VALID_RPM       0.0f
/* ============================================================================== */

#define FORCE_BIQUAD_B0    0.0200833656f
#define FORCE_BIQUAD_B1    0.0401667311f
#define FORCE_BIQUAD_B2    0.0200833656f
#define FORCE_BIQUAD_A1   (-1.561018109f)
#define FORCE_BIQUAD_A2    0.641351521f

#define PRINT_MODE 1
#define HEADER_EVERY_N_LINES  40

#if PRINT_MODE == 0
static const char CSV_HEADER[] =
  "ts_ms,loadcell.raw_counts,loadcell.net_counts,loadcell.force_newton,loadcell.v_inst_mps,loadcell.v_filt_mps,windcup.rpm,windcup.v_inst_mps,windcup.v_filt_mps";
#endif

I2C_HandleTypeDef hi2c1;
TIM_HandleTypeDef htim3;
UART_HandleTypeDef huart2;

static char uartBuf[256];
static int32_t zeroOffset = ZERO_OFFSET_EST;
static float counts_per_g = COUNTS_PER_GRAM;
static float grams_per_count = GRAMS_PER_COUNT;
static float N_per_count = 0.0f;

static float C0_N_per_mps2 = 0.0f;
static float C_used = 0.0f;
static GPIO_PinState btn_prev_state = GPIO_PIN_SET;
static float force_filt_z1 = 0.0f;
static float force_filt_z2 = 0.0f;
static float velocity_filt_lp = 0.0f;
static uint8_t velocity_filt_initialized = 0u;
static int32_t raw_history[3] = {0};
static uint8_t raw_hist_filled = 0u;
static uint8_t raw_hist_idx = 0u;

/* Wind-cup state */
static uint16_t windcup_last_counter = 0u;
static int32_t windcup_pulse_accum = 0;
static uint32_t windcup_last_report_ms = 0u;
static float windcup_rpm = 0.0f;
static float windcup_speed_inst = 0.0f;
static float windcup_speed_filt = 0.0f;
static uint8_t windcup_speed_initialized = 0u;

static inline uint32_t millis(void) { return HAL_GetTick(); }

static inline int32_t median3(int32_t a, int32_t b, int32_t c)
{
  if (a > b) { int32_t t = a; a = b; b = t; }
  if (b > c) { int32_t t = b; b = c; c = t; }
  if (a > b) { int32_t t = a; a = b; b = t; }
  return b;
}

static void perform_zero_reset(int32_t raw, const char *source)
{
  zeroOffset = raw;
  NAU7802_setZeroOffset(zeroOffset);
  force_filt_z1 = 0.0f;
  force_filt_z2 = 0.0f;
  velocity_filt_lp = 0.0f;
  velocity_filt_initialized = 0u;
  memset(raw_history, 0, sizeof(raw_history));
  raw_hist_filled = 0u;
  raw_hist_idx = 0u;
  windcup_speed_filt = 0.0f;
  windcup_speed_inst = 0.0f;
  windcup_speed_initialized = 0u;
  const char *label = (source != NULL) ? source : "manual";
  int report = snprintf(uartBuf, sizeof(uartBuf),
                        "%s zero reset at t=%lums -> %ld counts\r\n",
                        label, (unsigned long)millis(), (long)zeroOffset);
  if (report > 0) {
    HAL_UART_Transmit(&huart2, (uint8_t*)uartBuf, (uint16_t)report, 50);
  }
  int32_t net_after_tare = raw - zeroOffset;
  int loglen = snprintf(uartBuf, sizeof(uartBuf),
                        "loadcell.zeroed_net=%ld cnt\r\n",
                        (long)net_after_tare);
  if (loglen > 0) {
    HAL_UART_Transmit(&huart2, (uint8_t*)uartBuf, (uint16_t)loglen, 50);
  }
}

static void poll_uart_commands(int32_t raw)
{
  uint8_t ch;
  while (HAL_UART_Receive(&huart2, &ch, 1, 0) == HAL_OK) {
    if (ch == 'r' || ch == 'R' || ch == 'z' || ch == 'Z') {
      perform_zero_reset(raw, "UART");
    }
  }
}

static void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM3_Init(void);

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_TIM3_Init();
  btn_prev_state = HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin);

  if (!NAU7802_begin(&hi2c1)) {
    strcpy(uartBuf, "NAU7802 init failed !\r\n");
    HAL_UART_Transmit(&huart2, (uint8_t*)uartBuf, strlen(uartBuf), 100);
    while (1);
  }
  HAL_UART_Transmit(&huart2, (uint8_t*)"NAU7802 initialized !\r\n", 24, 100);

  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
  windcup_last_counter = __HAL_TIM_GET_COUNTER(&htim3);
  windcup_last_report_ms = millis();

  zeroOffset = (int32_t)ZERO_OFFSET_EST;
  counts_per_g = COUNTS_PER_GRAM;
  grams_per_count = GRAMS_PER_COUNT;

  if (N_PER_COUNT_OVERRIDE > 0.0f) {
    N_per_count = N_PER_COUNT_OVERRIDE;
  } else {
    N_per_count = 0.00980665f * grams_per_count;
  }

  const float A = PLATE_W_M * PLATE_H_M;
  C0_N_per_mps2 = 0.5f * AIR_RHO * DRAG_CD * A;
  C_used = (CALIBRATED_C_OVERRIDE > 0.0f) ? CALIBRATED_C_OVERRIDE : C0_N_per_mps2;

  int len = snprintf(uartBuf, sizeof(uartBuf),
    "Anemometer start\r\n"
    "zeroOffset (b): %ld counts\r\n"
    "counts_per_g (k): %.6f counts/g\r\n"
    "grams_per_count: %.9f g/count\r\n"
    "N_per_count: %.9e N/count\r\n"
    "Plate A: %.6f m^2  rho: %.2f  Cd: %.2f\r\n"
    "C_used (N/(m/s)^2): %.6f\r\n",
    (long)zeroOffset,
    (double)counts_per_g,
    (double)grams_per_count,
    (double)N_per_count,
    (double)A, (double)AIR_RHO, (double)DRAG_CD,
    (double)C_used
  );
#if PRINT_MODE == 0
  len += snprintf(uartBuf + len, sizeof(uartBuf) - (size_t)len,
                  "CSV columns (units):\r\n%s\r\n", CSV_HEADER);
#else
  len += snprintf(uartBuf + len, sizeof(uartBuf) - (size_t)len,
                  "PRETTY telemetry keys: ts_ms, loadcell.raw/net/force/v_inst/v_filt, windcup.rpm/v_inst/v_filt\r\n");
#endif
  HAL_UART_Transmit(&huart2, (uint8_t*)uartBuf, len, 200);

  NAU7802_setZeroOffset(zeroOffset);
  NAU7802_setCalibrationFactor(counts_per_g);

  uint32_t next_tick = millis();
  const float eps = 1e-9f;
#if PRINT_MODE == 0
  uint32_t lines_since_header = HEADER_EVERY_N_LINES;
#endif

  while (1) {
    next_tick += SAMPLE_PERIOD_MS;

    int32_t raw_sample = NAU7802_getAverage(&hi2c1, 2, 5);
    raw_history[raw_hist_idx] = raw_sample;
    raw_hist_idx = (uint8_t)((raw_hist_idx + 1u) % 3u);
    if (raw_hist_filled < 3u) raw_hist_filled++;
    int32_t raw = raw_sample;
    if (raw_hist_filled == 3u) {
      raw = median3(raw_history[0], raw_history[1], raw_history[2]);
    }

    GPIO_PinState btn_now = HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin);
    if (btn_prev_state == GPIO_PIN_SET && btn_now == GPIO_PIN_RESET) {
      perform_zero_reset(raw, "BUTTON");
    }
    btn_prev_state = btn_now;

    poll_uart_commands(raw);

    uint16_t windcup_counter_now = __HAL_TIM_GET_COUNTER(&htim3);
    int16_t pulse_delta = (int16_t)(windcup_counter_now - windcup_last_counter);
    windcup_last_counter = windcup_counter_now;
    windcup_pulse_accum += (int32_t)pulse_delta;

    uint32_t loop_time_ms = millis();
    uint32_t elapsed_since_report = loop_time_ms - windcup_last_report_ms;
    if (elapsed_since_report >= WINDCUP_SAMPLE_PERIOD_MS) {
      float elapsed_s = (float)elapsed_since_report / 1000.0f;
      if (elapsed_s > 0.0f) {
        float pulses = (float)windcup_pulse_accum;
        float revs = pulses / (float)WINDCUP_PULSES_PER_REV;
        float rpm_now = (revs / elapsed_s) * 60.0f;
        if (rpm_now < WINDCUP_MIN_VALID_RPM) rpm_now = 0.0f;
        windcup_rpm = rpm_now;
        windcup_speed_inst = windcup_rpm * WINDCUP_CALIBRATION_FACTOR;
        if (windcup_speed_inst < 0.0f) windcup_speed_inst = 0.0f;
        if (!windcup_speed_initialized) {
          windcup_speed_filt = windcup_speed_inst;
          windcup_speed_initialized = 1u;
        } else {
          windcup_speed_filt += WINDCUP_FILTER_ALPHA * (windcup_speed_inst - windcup_speed_filt);
        }
        if (windcup_speed_filt < 0.0f) windcup_speed_filt = 0.0f;
      } else {
        windcup_rpm = 0.0f;
        windcup_speed_inst = 0.0f;
        windcup_speed_filt = 0.0f;
      }
      windcup_pulse_accum = 0;
      windcup_last_report_ms = loop_time_ms;
    }

    int32_t net = raw - zeroOffset;

    float force_in = (float)net * N_per_count;
    float force_inst = (force_in < 0.0f) ? 0.0f : force_in;
    float force_lp = FORCE_BIQUAD_B0 * force_in + force_filt_z1;
    force_filt_z1 = FORCE_BIQUAD_B1 * force_in - FORCE_BIQUAD_A1 * force_lp + force_filt_z2;
    force_filt_z2 = FORCE_BIQUAD_B2 * force_in - FORCE_BIQUAD_A2 * force_lp;
    if (force_lp < 0.0f) force_lp = 0.0f;

    float v_inst = sqrtf(fmaxf(eps, force_inst / C_used));
    float v_filt = sqrtf(fmaxf(eps, force_lp / C_used));

    if (!velocity_filt_initialized) {
      velocity_filt_lp = v_filt;
      velocity_filt_initialized = 1u;
    } else {
      velocity_filt_lp += FILTER_ALPHA * (v_filt - velocity_filt_lp);
    }
    v_filt = velocity_filt_lp;

    uint32_t t = loop_time_ms;

#if PRINT_MODE == 0
    if (lines_since_header >= HEADER_EVERY_N_LINES) {
      int header_len = snprintf(uartBuf, sizeof(uartBuf), "%s\r\n", CSV_HEADER);
      HAL_UART_Transmit(&huart2, (uint8_t*)uartBuf, header_len, 50);
      lines_since_header = 0;
    }
    int outlen = snprintf(uartBuf, sizeof(uartBuf),
      "ts_ms=%lu,loadcell.raw=%ld,loadcell.net=%ld,loadcell.force=%.6f,loadcell.v_inst=%.4f,loadcell.v_filt=%.4f,windcup.rpm=%.3f,windcup.v_inst=%.3f,windcup.v_filt=%.3f\r\n",
      (unsigned long)t, (long)raw, (long)net,
      (double)force_lp, (double)v_inst, (double)v_filt,
      (double)windcup_rpm, (double)windcup_speed_inst, (double)windcup_speed_filt);
    HAL_UART_Transmit(&huart2, (uint8_t*)uartBuf, outlen, 50);
    lines_since_header++;
#else
    int outlen = snprintf(uartBuf, sizeof(uartBuf),
      "ts=%lums | loadcell.raw=%ld cnt | loadcell.net=%ld cnt | loadcell.force=%.6f N | loadcell.v_inst=%.3f m/s | loadcell.v_filt=%.3f m/s | windcup.rpm=%.3f | windcup.v_inst=%.3f m/s | windcup.v_filt=%.3f m/s\r\n",
      (unsigned long)t, (long)raw, (long)net,
      (double)force_lp, (double)v_inst, (double)v_filt,
      (double)windcup_rpm, (double)windcup_speed_inst, (double)windcup_speed_filt);
    HAL_UART_Transmit(&huart2, (uint8_t*)uartBuf, outlen, 50);
#endif

    int32_t now = (int32_t)millis();
    int32_t wait_ms = (int32_t)next_tick - now;
    if (wait_ms > 0) {
      HAL_Delay((uint32_t)wait_ms);
    } else {
      next_tick = millis();
    }
  }
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                              | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
    Error_Handler();
  }
}

static void MX_I2C1_Init(void)
{
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
    Error_Handler();
  }
}

static void MX_TIM3_Init(void)
{
  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK) {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK) {
    Error_Handler();
  }
}

static void MX_USART2_UART_Init(void)
{
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK) {
    Error_Handler();
  }
}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}

void Error_Handler(void)
{
  __disable_irq();
  while (1) {}
}

