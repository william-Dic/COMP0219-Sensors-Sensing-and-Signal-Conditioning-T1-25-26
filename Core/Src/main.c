/* main.c -- Anemometer single-file version (configurable parameters at top)
   Paste into your STM32Cube/HAL project replacing previous main.c
*/

#include "main.h"
#include "nau7802.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

/* =================== 用户可配置区（打开工程后就在这里改数值） =================== */
/* 校准相关（来自 multipoint calibration 输出，测试用，可改） */
#define ZERO_OFFSET_EST  142739L            // b (counts)
#define COUNTS_PER_GRAM   429.540924f        // k (counts / g)
#define GRAMS_PER_COUNT   0.002328067f       // g / count (可以与上面互相验证)
#define N_PER_COUNT_OVERRIDE 0.0f            // 若你有直接算出的 N/count 可写入，0 表示用 grams_per_count 计算
/* 气动 / 平板几何（用于临时 C0 计算） */
#define PLATE_W_M   0.09f    // 9 cm
#define PLATE_H_M   0.093f   // 9.3 cm
#define AIR_RHO     1.20f    // kg/m^3（室温近似）
#define DRAG_CD     1.10f    // 平板正对迎流经验值

/* 可选：如果你有风洞拟合后的 C（N/(m/s)^2），写在这里替换临时计算值；0 表示使用临时 C0 */
//#define CALIBRATED_C_OVERRIDE  0.0f
#define CALIBRATED_C_OVERRIDE  0.005729634f

/* 滤波与采样 */
#define SAMPLE_PERIOD_MS   5      // 5 ms -> 200 Hz
#define FILTER_ALPHA       0.08f  // smaller -> heavier smoothing
/* ============================================================================== */

#define FORCE_BIQUAD_B0    0.0200833656f
#define FORCE_BIQUAD_B1    0.0401667311f
#define FORCE_BIQUAD_B2    0.0200833656f
#define FORCE_BIQUAD_A1   (-1.561018109f)
#define FORCE_BIQUAD_A2    0.641351521f

/* ==== at the top, near user config ==== */
#define PRINT_MODE 1  // 0 = CSV (machine-friendly), 1 = PRETTY (human-friendly)
#define HEADER_EVERY_N_LINES  40

#if PRINT_MODE == 0
static const char CSV_HEADER[] =
  "timestamp_ms,raw_counts,net_counts_after_tare,force_newton,velocity_air_instant_mps,velocity_air_filtered_mps";
#endif


/* HAL 外设句柄（保持你原本工程的一样） */
I2C_HandleTypeDef hi2c1;
UART_HandleTypeDef huart2;

/* 内部变量 */
char uartBuf[160];
int32_t zeroOffset = ZERO_OFFSET_EST;
float counts_per_g = COUNTS_PER_GRAM;
float grams_per_count = GRAMS_PER_COUNT;
float N_per_count = 0.0f;

float C0_N_per_mps2 = 0.0f; // C 常数 = 0.5 * rho * Cd * A (默认，风洞拟合会替换)
float C_used = 0.0f;        // 实际使用的 C（可能是 CALIBRATED_C_OVERRIDE 或 C0）
static GPIO_PinState btn_prev_state = GPIO_PIN_SET; // track blue button state (PC13, active-high idle)
static float force_filt_z1 = 0.0f;
static float force_filt_z2 = 0.0f;
static float velocity_filt_lp = 0.0f;
static uint8_t velocity_filt_initialized = 0;
static int32_t raw_history[3] = {0};
static uint8_t raw_hist_filled = 0;
static uint8_t raw_hist_idx = 0;

/* 简单毫秒函数 */
static inline uint32_t millis(void) { return HAL_GetTick(); }
static inline int32_t median3(int32_t a, int32_t b, int32_t c)
{
  if (a > b) { int32_t t = a; a = b; b = t; }
  if (b > c) { int32_t t = b; b = c; c = t; }
  if (a > b) { int32_t t = a; a = b; b = t; }
  return b;
}

/* 在 main() 之前添加这些函数原型，确保编译器知道它们的签名 */
static void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);


int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  btn_prev_state = HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin);

  // 初始化 NAU7802
  if (!NAU7802_begin(&hi2c1)) {
    sprintf(uartBuf, "NAU7802 init failed !\r\n");
    HAL_UART_Transmit(&huart2, (uint8_t*)uartBuf, strlen(uartBuf), 100);
    while (1);
  }
  HAL_UART_Transmit(&huart2, (uint8_t*)"NAU7802 initialized !\r\n", 24, 100);

  // 将用户配置复制到运行变量（方便以后用 UI 改）
  zeroOffset = (int32_t)ZERO_OFFSET_EST;
  counts_per_g = COUNTS_PER_GRAM;
  grams_per_count = GRAMS_PER_COUNT;

  // N_per_count：优先使用 override，否则从 grams_per_count 推导
  if (N_PER_COUNT_OVERRIDE > 0.0f) {
    N_per_count = N_PER_COUNT_OVERRIDE;
  } else {
    N_per_count = 0.00980665f * grams_per_count; // 1 g = 0.00980665 N
  }

  // 计算临时气动常数 C0
  const float A = PLATE_W_M * PLATE_H_M;
  C0_N_per_mps2 = 0.5f * AIR_RHO * DRAG_CD * A;

  // 使用优先级：若用户提供了 CALIBRATED_C_OVERRIDE (>0) 则使用之，否则用 C0
  C_used = (CALIBRATED_C_OVERRIDE > 0.0f) ? CALIBRATED_C_OVERRIDE : C0_N_per_mps2;

  // 打印启动报告（包含你要的“可直接放入风速计代码”的数）
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
    "CSV columns (units):\r\n"
    "%s\r\n",
    CSV_HEADER
  );
#else
  len += snprintf(uartBuf + len, sizeof(uartBuf) - (size_t)len,
    "PRETTY print columns (units): t_ms, raw_counts, net_counts_after_tare, force_N, v_inst_mps, v_filt_mps\r\n"
  );
#endif
  HAL_UART_Transmit(&huart2, (uint8_t*)uartBuf, len, 200);

  // 如果需要，可以把 zeroOffset 写回驱动（不强制）
  NAU7802_setZeroOffset(zeroOffset);
  NAU7802_setCalibrationFactor(counts_per_g);

  // 主循环：200 Hz 输出风速
  uint32_t next_tick = millis();
  const float eps = 1e-9f;
#if PRINT_MODE == 0
  uint32_t lines_since_header = HEADER_EVERY_N_LINES; // force header on first loop
#endif
  for (;;) {
    next_tick += SAMPLE_PERIOD_MS;

    // 取样并做 median-of-3 抑制毛刺
    int32_t raw_sample = NAU7802_getAverage(&hi2c1, 2, 5);
    raw_history[raw_hist_idx] = raw_sample;
    raw_hist_idx = (uint8_t)((raw_hist_idx + 1u) % 3u);
    if (raw_hist_filled < 3u) raw_hist_filled++;
    int32_t raw = raw_sample;
    if (raw_hist_filled == 3u) {
      raw = median3(raw_history[0], raw_history[1], raw_history[2]);
    }

    // Check blue button (B1 / PC13). Active-low, latch on falling edge.
    GPIO_PinState btn_now = HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin);
    if (btn_prev_state == GPIO_PIN_SET && btn_now == GPIO_PIN_RESET) {
      zeroOffset = raw;
      NAU7802_setZeroOffset(zeroOffset);
      int report = snprintf(uartBuf, sizeof(uartBuf),
                            "Zero offset reset at t=%lums -> %ld counts\r\n",
                            (unsigned long)millis(), (long)zeroOffset);
      if (report > 0) {
        HAL_UART_Transmit(&huart2, (uint8_t*)uartBuf, (uint16_t)report, 50);
      }
    }
    btn_prev_state = btn_now;

    int32_t net = raw - zeroOffset;

    // 计数 -> 力（牛顿）并使用 Butterworth 低通（DF-II-T）
    float force_in = (float)net * N_per_count;
    float force_inst = force_in;
    if (force_inst < 0.0f) force_inst = 0.0f;
    float force_lp = FORCE_BIQUAD_B0 * force_in + force_filt_z1;
    force_filt_z1 = FORCE_BIQUAD_B1 * force_in - FORCE_BIQUAD_A1 * force_lp + force_filt_z2;
    force_filt_z2 = FORCE_BIQUAD_B2 * force_in - FORCE_BIQUAD_A2 * force_lp;
    if (force_lp < 0.0f) force_lp = 0.0f; // 保护：不输出负力

    // 力 -> 瞬时风速
    float v_inst = sqrtf(fmaxf(eps, force_inst / C_used));
    float v_filt = sqrtf(fmaxf(eps, force_lp / C_used));

    if (!velocity_filt_initialized) {
      velocity_filt_lp = v_filt;
      velocity_filt_initialized = 1u;
    } else {
      velocity_filt_lp += FILTER_ALPHA * (v_filt - velocity_filt_lp);
    }
    v_filt = velocity_filt_lp;

    uint32_t t = millis();

#if PRINT_MODE == 0
    if (lines_since_header >= HEADER_EVERY_N_LINES) {
      int header_len = snprintf(uartBuf, sizeof(uartBuf), "%s\r\n", CSV_HEADER);
      HAL_UART_Transmit(&huart2, (uint8_t*)uartBuf, header_len, 50);
      lines_since_header = 0;
    }

    // 输出 CSV
    int outlen = snprintf(uartBuf, sizeof(uartBuf),
               "%lu,%ld,%ld,%.6f,%.4f,%.4f\r\n",
               (unsigned long)t, (long)raw, (long)net,
               (double)force_lp, (double)v_inst, (double)v_filt);
    HAL_UART_Transmit(&huart2, (uint8_t*)uartBuf, outlen, 50);
    lines_since_header++;
#else
    int outlen = snprintf(uartBuf, sizeof(uartBuf),
               "t=%lums | raw=%ld cnt | net=%ld cnt | force=%.6f N | v_inst=%.3f m/s | v_filt=%.3f m/s\r\n",
               (unsigned long)t, (long)raw, (long)net,
               (double)force_lp, (double)v_inst, (double)v_filt);
    HAL_UART_Transmit(&huart2, (uint8_t*)uartBuf, outlen, 50);
#endif

    // 等待下个周期（若超时则跳过）
    int32_t now = (int32_t)millis();
    int32_t wait_ms = (int32_t)next_tick - now;
    if (wait_ms > 0) HAL_Delay((uint32_t)wait_ms);
    else next_tick = millis();
  }

  // 不会到这里
  return 0;
}

/* 时钟/I2C/UART/GPIO 初始化（与原工程一致） */
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
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) { Error_Handler(); }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) { Error_Handler(); }
}

static void MX_I2C1_Init(void)
{
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed      = 100000;
  hi2c1.Init.DutyCycle       = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1     = 0;
  hi2c1.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2     = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK) { Error_Handler(); }
}

static void MX_USART2_UART_Init(void)
{
  huart2.Instance = USART2;
  huart2.Init.BaudRate   = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits   = UART_STOPBITS_1;
  huart2.Init.Parity     = UART_PARITY_NONE;
  huart2.Init.Mode       = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK) { Error_Handler(); }
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
  while (1) { }
}
