/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define LSM6DS33_ADDR_HIGH        0x6B
#define LSM6DS33_ADDR_LOW         0x6A

#define LSM6DS33_WHO_AM_I_REG     0x0F
#define LSM6DS33_CTRL1_XL         0x10
#define LSM6DS33_CTRL2_G          0x11
#define LSM6DS33_CTRL3_C          0x12

#define LSM6DS33_OUTX_L_G         0x22
#define LSM6DS33_OUTX_L_XL        0x28

#define LSM6DS33_WHO_AM_I_VAL     0x69

#define ACCEL_SENS_G_PER_LSB      0.000061f
#define GYRO_SENS_DPS_PER_LSB     0.00875f

#define GYRO_CAL_SAMPLES          200
#define RAD_TO_DEG                57.2957795f

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
static uint8_t imu_addr = 0;

static uint8_t gyro_buf[6];
static uint8_t accel_buf[6];

static int16_t gyro_x = 0;
static int16_t gyro_y = 0;
static int16_t gyro_z = 0;

static int16_t accel_x = 0;
static int16_t accel_y = 0;
static int16_t accel_z = 0;

static float ax_g = 0.0f;
static float ay_g = 0.0f;
static float az_g = 0.0f;

static float gx_dps = 0.0f;
static float gy_dps = 0.0f;
static float gz_dps = 0.0f;

static float gx_bias = 0.0f;
static float gy_bias = 0.0f;
static float gz_bias = 0.0f;

static float roll_deg = 0.0f;
static float pitch_deg = 0.0f;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void uart_send_text(const char *s);
void uart_send_line(const char *s);

uint8_t probe_addr(uint8_t addr);
void i2c_scan(void);

uint8_t imu_write_reg(uint8_t reg, uint8_t val);
uint8_t imu_read_reg(uint8_t reg, uint8_t *val);
uint8_t imu_read_regs(uint8_t reg, uint8_t *buf, uint16_t len);

uint8_t MiniMU_FindIMU(void);
uint8_t MiniMU_Init(void);
uint8_t MiniMU_ReadGyro(void);
uint8_t MiniMU_ReadAccel(void);
void MiniMU_UpdateScaled(void);
void MiniMU_UpdateAngles(void);
void MiniMU_PrintScaledData(void);
void MiniMU_PrintAngles(void);
uint8_t MiniMU_CalibrateGyro(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int __io_putchar(int ch)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1u, HAL_MAX_DELAY);
    return ch;
}

void uart_send_text(const char *s)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)s, (uint16_t)strlen(s), HAL_MAX_DELAY);
}

void uart_send_line(const char *s)
{
    uart_send_text(s);
    uart_send_text("\r\n");
}

uint8_t probe_addr(uint8_t addr)
{
    return (HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(addr << 1), 3, 100) == HAL_OK) ? 1u : 0u;
}

void i2c_scan(void)
{
    char msg[64];
    uint8_t found = 0;
    uint8_t addr;

    uart_send_line("I2C scan start");

    for (addr = 0x08; addr < 0x78; addr++)
    {
        if (probe_addr(addr))
        {
            snprintf(msg, sizeof(msg), "I2C device found at 0x%02X", addr);
            uart_send_line(msg);
            found = 1;
        }
    }

    if (!found)
    {
        uart_send_line("No I2C devices found");
    }

    uart_send_line("I2C scan end");
}

uint8_t imu_write_reg(uint8_t reg, uint8_t val)
{
    return (HAL_I2C_Mem_Write(&hi2c1,
                              (uint16_t)(imu_addr << 1),
                              reg,
                              I2C_MEMADD_SIZE_8BIT,
                              &val,
                              1,
                              100) == HAL_OK) ? 1u : 0u;
}

uint8_t imu_read_reg(uint8_t reg, uint8_t *val)
{
    return (HAL_I2C_Mem_Read(&hi2c1,
                             (uint16_t)(imu_addr << 1),
                             reg,
                             I2C_MEMADD_SIZE_8BIT,
                             val,
                             1,
                             100) == HAL_OK) ? 1u : 0u;
}

uint8_t imu_read_regs(uint8_t reg, uint8_t *buf, uint16_t len)
{
    return (HAL_I2C_Mem_Read(&hi2c1,
                             (uint16_t)(imu_addr << 1),
                             reg,
                             I2C_MEMADD_SIZE_8BIT,
                             buf,
                             len,
                             100) == HAL_OK) ? 1u : 0u;
}

uint8_t MiniMU_FindIMU(void)
{
    if (probe_addr(LSM6DS33_ADDR_HIGH))
    {
        imu_addr = LSM6DS33_ADDR_HIGH;
        return 1;
    }

    if (probe_addr(LSM6DS33_ADDR_LOW))
    {
        imu_addr = LSM6DS33_ADDR_LOW;
        return 1;
    }

    imu_addr = 0;
    return 0;
}

uint8_t MiniMU_Init(void)
{
    uint8_t who = 0;
    uint8_t ctrl1 = 0;
    uint8_t ctrl2 = 0;
    uint8_t ctrl3 = 0;
    char msg[96];

    i2c_scan();

    if (!MiniMU_FindIMU())
    {
        uart_send_line("Could not find LSM6DS33 at 0x6B or 0x6A");
        return 0;
    }

    snprintf(msg, sizeof(msg), "LSM6DS33 addr = 0x%02X", imu_addr);
    uart_send_line(msg);

    if (!imu_read_reg(LSM6DS33_WHO_AM_I_REG, &who))
    {
        uart_send_line("WHO_AM_I read failed");
        return 0;
    }

    snprintf(msg, sizeof(msg), "WHO_AM_I = 0x%02X", who);
    uart_send_line(msg);

    if (who != LSM6DS33_WHO_AM_I_VAL)
    {
        uart_send_line("WHO_AM_I mismatch");
        return 0;
    }

    if (!imu_write_reg(LSM6DS33_CTRL3_C, 0x01))
    {
        uart_send_line("CTRL3_C reset write failed");
        return 0;
    }

    HAL_Delay(50);

    if (!imu_write_reg(LSM6DS33_CTRL3_C, 0x44))
    {
        uart_send_line("CTRL3_C config write failed");
        return 0;
    }

    if (!imu_write_reg(LSM6DS33_CTRL1_XL, 0x40))
    {
        uart_send_line("CTRL1_XL write failed");
        return 0;
    }

    if (!imu_write_reg(LSM6DS33_CTRL2_G, 0x40))
    {
        uart_send_line("CTRL2_G write failed");
        return 0;
    }

    HAL_Delay(20);

    if (!imu_read_reg(LSM6DS33_CTRL1_XL, &ctrl1))
    {
        uart_send_line("CTRL1_XL readback failed");
        return 0;
    }

    if (!imu_read_reg(LSM6DS33_CTRL2_G, &ctrl2))
    {
        uart_send_line("CTRL2_G readback failed");
        return 0;
    }

    if (!imu_read_reg(LSM6DS33_CTRL3_C, &ctrl3))
    {
        uart_send_line("CTRL3_C readback failed");
        return 0;
    }

    snprintf(msg, sizeof(msg),
             "CTRL1_XL=0x%02X CTRL2_G=0x%02X CTRL3_C=0x%02X",
             ctrl1, ctrl2, ctrl3);
    uart_send_line(msg);

    if ((ctrl1 == 0x00) && (ctrl2 == 0x00))
    {
        uart_send_line("Sensor config did not stick");
        return 0;
    }

    uart_send_line("MiniMU init OK");
    return 1;
}

uint8_t MiniMU_ReadGyro(void)
{
    if (!imu_read_regs(LSM6DS33_OUTX_L_G, gyro_buf, 6))
    {
        return 0;
    }

    gyro_x = (int16_t)((((uint16_t)gyro_buf[1]) << 8) | gyro_buf[0]);
    gyro_y = (int16_t)((((uint16_t)gyro_buf[3]) << 8) | gyro_buf[2]);
    gyro_z = (int16_t)((((uint16_t)gyro_buf[5]) << 8) | gyro_buf[4]);

    return 1;
}

uint8_t MiniMU_ReadAccel(void)
{
    if (!imu_read_regs(LSM6DS33_OUTX_L_XL, accel_buf, 6))
    {
        return 0;
    }

    accel_x = (int16_t)((((uint16_t)accel_buf[1]) << 8) | accel_buf[0]);
    accel_y = (int16_t)((((uint16_t)accel_buf[3]) << 8) | accel_buf[2]);
    accel_z = (int16_t)((((uint16_t)accel_buf[5]) << 8) | accel_buf[4]);

    return 1;
}

void MiniMU_UpdateScaled(void)
{
    ax_g = ((float)accel_x) * ACCEL_SENS_G_PER_LSB;
    ay_g = ((float)accel_y) * ACCEL_SENS_G_PER_LSB;
    az_g = ((float)accel_z) * ACCEL_SENS_G_PER_LSB;

    gx_dps = ((float)gyro_x) * GYRO_SENS_DPS_PER_LSB - gx_bias;
    gy_dps = ((float)gyro_y) * GYRO_SENS_DPS_PER_LSB - gy_bias;
    gz_dps = ((float)gyro_z) * GYRO_SENS_DPS_PER_LSB - gz_bias;
}

void MiniMU_UpdateAngles(void)
{
    float denom_pitch;

    denom_pitch = sqrtf((ax_g * ax_g) + (az_g * az_g));

    if (denom_pitch < 0.0001f)
    {
        denom_pitch = 0.0001f;
    }

    roll_deg = atan2f(ay_g, az_g) * RAD_TO_DEG;
    pitch_deg = atan2f(-ax_g, denom_pitch) * RAD_TO_DEG;
}

void MiniMU_PrintScaledData(void)
{
    char msg[220];

    int ax_milli = (int)(ax_g * 1000.0f);
    int ay_milli = (int)(ay_g * 1000.0f);
    int az_milli = (int)(az_g * 1000.0f);

    int gx_milli = (int)(gx_dps * 1000.0f);
    int gy_milli = (int)(gy_dps * 1000.0f);
    int gz_milli = (int)(gz_dps * 1000.0f);

    snprintf(msg, sizeof(msg),
             "ACC[g] X:%d.%03d Y:%d.%03d Z:%d.%03d | GYRO[dps] X:%d.%03d Y:%d.%03d Z:%d.%03d",
             ax_milli / 1000, (ax_milli < 0 ? -ax_milli : ax_milli) % 1000,
             ay_milli / 1000, (ay_milli < 0 ? -ay_milli : ay_milli) % 1000,
             az_milli / 1000, (az_milli < 0 ? -az_milli : az_milli) % 1000,
             gx_milli / 1000, (gx_milli < 0 ? -gx_milli : gx_milli) % 1000,
             gy_milli / 1000, (gy_milli < 0 ? -gy_milli : gy_milli) % 1000,
             gz_milli / 1000, (gz_milli < 0 ? -gz_milli : gz_milli) % 1000);

    uart_send_line(msg);
}

void MiniMU_PrintAngles(void)
{
    char msg[120];

    int roll_hundredths = (int)(roll_deg * 100.0f);
    int pitch_hundredths = (int)(pitch_deg * 100.0f);

    snprintf(msg, sizeof(msg),
             "ANGLE roll:%d.%02d deg | pitch:%d.%02d deg",
             roll_hundredths / 100, (roll_hundredths < 0 ? -roll_hundredths : roll_hundredths) % 100,
             pitch_hundredths / 100, (pitch_hundredths < 0 ? -pitch_hundredths : pitch_hundredths) % 100);

    uart_send_line(msg);
}

uint8_t MiniMU_CalibrateGyro(void)
{
    uint32_t i;
    int32_t sum_x = 0;
    int32_t sum_y = 0;
    int32_t sum_z = 0;
    char msg[128];

    uart_send_line("Gyro calibration starting. Keep IMU still.");

    for (i = 0; i < GYRO_CAL_SAMPLES; i++)
    {
        if (!MiniMU_ReadGyro())
        {
            uart_send_line("Gyro calibration read failed");
            return 0;
        }

        sum_x += gyro_x;
        sum_y += gyro_y;
        sum_z += gyro_z;

        HAL_Delay(10);
    }

    gx_bias = ((float)sum_x / (float)GYRO_CAL_SAMPLES) * GYRO_SENS_DPS_PER_LSB;
    gy_bias = ((float)sum_y / (float)GYRO_CAL_SAMPLES) * GYRO_SENS_DPS_PER_LSB;
    gz_bias = ((float)sum_z / (float)GYRO_CAL_SAMPLES) * GYRO_SENS_DPS_PER_LSB;

    {
        int bx = (int)(gx_bias * 1000.0f);
        int by = (int)(gy_bias * 1000.0f);
        int bz = (int)(gz_bias * 1000.0f);

        snprintf(msg, sizeof(msg),
                 "Gyro bias[dps] X:%d.%03d Y:%d.%03d Z:%d.%03d",
                 bx / 1000, (bx < 0 ? -bx : bx) % 1000,
                 by / 1000, (by < 0 ? -by : by) % 1000,
                 bz / 1000, (bz < 0 ? -bz : bz) % 1000);
        uart_send_line(msg);
    }

    uart_send_line("Gyro calibration done");
    return 1;
}

/* USER CODE END 0 */

int main(void)
{
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_ADC_Init();
  MX_I2C1_Init();
  MX_TIM6_Init();

  uart_send_line("");
  uart_send_line("Booting...");
  uart_send_line("USART1 ready");
  HAL_Delay(300);

  if (!MiniMU_Init())
  {
      while (1)
      {
          HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_1);
          uart_send_line("MiniMU init failed");
          HAL_Delay(1000);
      }
  }

  if (!MiniMU_CalibrateGyro())
  {
      while (1)
      {
          HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_1);
          uart_send_line("Gyro calibration failed");
          HAL_Delay(1000);
      }
  }

  uart_send_line("Streaming accel + gyro + angles");

  while (1)
  {
      HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_1);

      if (!MiniMU_ReadGyro())
      {
          uart_send_line("Gyro read failed");
      }
      else if (!MiniMU_ReadAccel())
      {
          uart_send_line("Accel read failed");
      }
      else
      {
          MiniMU_UpdateScaled();
          MiniMU_UpdateAngles();
          MiniMU_PrintScaledData();
          MiniMU_PrintAngles();
          uart_send_line("");
      }

      HAL_Delay(1000);
  }
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

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                              | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_1);
    HAL_Delay(150);
  }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  (void)file;
  (void)line;
}
#endif