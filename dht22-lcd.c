/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * ECE190.c

  * This program demonstrates how to interface with a HD44780 LCD, a DHT22
  * humidity and temperature sensor, and a particle sensor. It contains a
  * modification of the code in:
  * https://adastra-soft.com/hd44780-library-for-stm32/ to interface with the
  * HD44780.
  *
  * Pin assignments:
  * RS            PB10
  * RW            PA8
  * E_PULSE       PA9
  * DB4           PB6
  * DB5           PA7
  * DB6           PA6
  * DB7           PA5
  * DHT22         PA10
  8 P_SENSOR_SDA  PB9
  * P_SENSOR_SCL  PB8
  *
  * USART2_TX   PA2 - for debugging so you can print to console
  * USART2_RX   PA3 - for debugging so you can print to console
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define LCD_RS_PORT GPIOB
#define LCD_RS_PIN LCD_RS_Pin
#define LCD_RW_PORT GPIOA
#define LCD_RW_PIN LCD_RW_Pin
#define LCD_E_PORT GPIOA
#define LCD_E_PIN LCD_E_Pin
#define LCD_DB4_PORT GPIOB
#define LCD_DB4_PIN LCD_DB4_Pin
#define LCD_DB5_PORT GPIOA
#define LCD_DB5_PIN LCD_DB5_Pin
#define LCD_DB6_PORT GPIOA
#define LCD_DB6_PIN LCD_DB6_Pin
#define LCD_DB7_PORT GPIOA
#define LCD_DB7_PIN LCD_DB7_Pin
#define DHT22_PORT GPIOA
#define DHT22_PIN DHT22_Pin

#define PIN_CLEAR GPIO_PIN_RESET
#define PIN_SET GPIO_PIN_SET
#define DOWN 0
#define UP 1

#define MAX_CHARS 20
#define MAX_SPACE 16

#define LABEL1 "Humid: "
#define LABEL2 "Temp : "
#define LABEL3 "Dust : "
#define POLL_INTERVAL 30000000

/******************************* data structures ******************************/
struct data_t
{
  uint16_t rh;
  uint16_t temp;
  uint8_t integrity; /* 0 is good; other values are bad. */
};

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
RTC_HandleTypeDef hrtc;
TIM_HandleTypeDef htim2;
UART_HandleTypeDef huart2;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_RTC_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART2_UART_Init(void);

void delay_us(uint32_t us);
void e_pulse(void);
void send_lcd_command(uint8_t);
void send_lcd_char(uint8_t);
void set_data_pins(uint32_t);
void init_lcd(void);
void not_busy(void);
void set_pin_input(GPIO_TypeDef *, uint16_t, uint8_t);
void set_pin_output(GPIO_TypeDef *, uint16_t);
void print_message(char *);
void poll_dht22(GPIO_TypeDef *, uint16_t, struct data_t *);

#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

PUTCHAR_PROTOTYPE
{
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  char  label1[MAX_CHARS] = LABEL1,
        label2[MAX_CHARS] = LABEL2,
        label3[MAX_CHARS] = LABEL3,
        dht_value[10];
  struct data_t *data = calloc(1, sizeof(struct data_t));
  uint8_t lcd_home = 128;

  /*Reset of all peripherals, Initializes the Flash interface and the Systick.*/
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_RTC_Init();
  MX_TIM2_Init();
  MX_USART2_UART_Init();

  HAL_TIM_Base_Start(&htim2); /* needed by the microsecond timer */
  init_lcd();

  delay_us(3000000); /* initial delay after on */

  while (1) /* Infinite loop */
  {
    HAL_TIM_Base_Start(&htim2); /* needed by the microsecond timer */

    send_lcd_command(lcd_home);
    print_message(label1);
    send_lcd_command(lcd_home + 64); /* move the cursor to the 2nd row */
    print_message(label2);

    poll_dht22(DHT22_PORT, DHT22_PIN, data);

    send_lcd_command(128 + strlen(label1) + 1);
    if(data->integrity == 0)
    {
      sprintf(dht_value, "%d.%d%%", (uint16_t)(data->rh / 10),
              (uint16_t)(data->rh % 10));
    }
    else
    {
      sprintf(dht_value, "N.A.");
    }
    print_message(dht_value);

    send_lcd_command(192 + strlen(label2) + 1);
    if(data->integrity == 0)
    {
      sprintf(dht_value, "%d.%dC", (int16_t)(data->temp / 10),
              (uint16_t)(data->temp % 10));
    }
    else
    {
      sprintf(dht_value, "N.A.");
    }
    print_message(dht_value);
    delay_us(POLL_INTERVAL);
  }

  free(data);
  return(0);

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|
                                      RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
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
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{
  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 0x1;
  sDate.Year = 0x0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 42 - 1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295 - 1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
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
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LCD_DB7_Pin|LCD_DB6_Pin|LCD_DB5_Pin|LCD_RW_Pin
                          |LCD_E_Pin|DHT22_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LCD_RS_Pin|LCD_DB4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LCD_DB7_Pin LCD_DB6_Pin LCD_DB5_Pin LCD_RW_Pin
                           LCD_E_Pin DHT22_Pin */
  GPIO_InitStruct.Pin = LCD_DB7_Pin|LCD_DB6_Pin|LCD_DB5_Pin|LCD_RW_Pin
                          |LCD_E_Pin|DHT22_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_RS_Pin LCD_DB4_Pin */
  GPIO_InitStruct.Pin = LCD_RS_Pin|LCD_DB4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

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

#ifdef  USE_FULL_ASSERT
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

void delay_us(uint32_t us)
{
  __HAL_TIM_SET_COUNTER(&htim2, 0);
  while(__HAL_TIM_GET_COUNTER(&htim2) < us);
  return;
}

/***************************** LCD functions **********************************/
void e_pulse(void)
/* sends one millisecond pulse to the LCD */
{
  //HAL_TIM_Base_Start(&htim2); /* needed by the microsecond clock */
  delay_us(2000);
  HAL_GPIO_WritePin(LCD_E_PORT, LCD_E_PIN, PIN_SET);
  delay_us(2000);
  HAL_GPIO_WritePin(LCD_E_PORT, LCD_E_PIN, PIN_CLEAR);

  return;
}

void send_lcd_command(uint8_t command)
{
  set_data_pins(command >> 4); /* first four significant bits */
  HAL_GPIO_WritePin(LCD_RS_PORT, LCD_RS_PIN, PIN_CLEAR);
  e_pulse();

  set_data_pins(command); /* last four bits */
  e_pulse();
  not_busy();

  return;
}

void send_lcd_char(uint8_t character)
{
  set_data_pins(character >> 4); /* first four significant bits */
  HAL_GPIO_WritePin(LCD_RS_PORT, LCD_RS_PIN, PIN_SET);
  e_pulse();

  set_data_pins(character); /* last four bits */
  e_pulse();
  not_busy();

  return;
}

void set_data_pins(uint32_t data)
{
  if((data & 0x08u) == 0u)
  {
    HAL_GPIO_WritePin(LCD_DB7_PORT, LCD_DB7_PIN, PIN_CLEAR);
  }
  else
  {
    HAL_GPIO_WritePin(LCD_DB7_PORT, LCD_DB7_PIN, PIN_SET);
  }

  if((data & 0x04u) == 0u)
  {
    HAL_GPIO_WritePin(LCD_DB6_PORT, LCD_DB6_PIN, PIN_CLEAR);
  }
  else
  {
    HAL_GPIO_WritePin(LCD_DB6_PORT, LCD_DB6_PIN, PIN_SET);
  }

  if((data & 0x02u) == 0u)
  {
    HAL_GPIO_WritePin(LCD_DB5_PORT, LCD_DB5_PIN, PIN_CLEAR);
  }
  else
  {
    HAL_GPIO_WritePin(LCD_DB5_PORT, LCD_DB5_PIN, PIN_SET);
  }

  if((data & 0x01u) == 0u)
  {
    HAL_GPIO_WritePin(LCD_DB4_PORT, LCD_DB4_PIN, PIN_CLEAR);
  }
  else
  {
    HAL_GPIO_WritePin(LCD_DB4_PORT, LCD_DB4_PIN, PIN_SET);
  }

  return;
}

void init_lcd(void)
{
  //HAL_TIM_Base_Start(&htim2); /* needed by the microsecond clock */

  HAL_GPIO_WritePin(LCD_RS_PORT, LCD_RS_PIN, PIN_CLEAR);
  HAL_GPIO_WritePin(LCD_RW_PORT, LCD_RW_PIN, PIN_CLEAR);

  delay_us(40000);

  set_data_pins(0x03u);
  e_pulse();
  delay_us(5000);

  set_data_pins(0x03u);
  e_pulse();
  delay_us(50);

  set_data_pins(0x03u);
  e_pulse();
  delay_us(50);

  set_data_pins(0x02u); /* set to 4-bit mode */
  e_pulse();
  not_busy();

  send_lcd_command(0x28); /* interface length 4 bits, 2 lines, 5X7 font */
  send_lcd_command(0x08); /* display off */
  send_lcd_command(0x01); /* clear display */
  send_lcd_command(0x0C); /* display on, cursor off, no blink */
  send_lcd_command(0x06); /* move cursor after each character */

  return;
}

void not_busy(void)
{
  //HAL_TIM_Base_Start(&htim2); /* needed by the microsecond clock */

  set_pin_input(LCD_DB7_PORT, LCD_DB7_PIN, UP);

  HAL_GPIO_WritePin(LCD_RS_PORT, LCD_RS_PIN, PIN_CLEAR);
  HAL_GPIO_WritePin(LCD_RW_PORT, LCD_RW_PIN, PIN_SET);

  while(HAL_GPIO_ReadPin(LCD_DB7_PORT, LCD_DB7_PIN) == PIN_CLEAR)
  {
    e_pulse();
  }

  HAL_GPIO_WritePin(LCD_RW_PORT, LCD_RW_PIN, PIN_CLEAR);

  set_pin_output(LCD_DB7_PORT, LCD_DB7_PIN);
  return;
}

void set_pin_input(GPIO_TypeDef *port, uint16_t pin, uint8_t pull)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  GPIO_InitStruct.Pin = pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  if(pull == DOWN)
  {
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  }
  else if (pull == UP)
  {
    GPIO_InitStruct.Pull = GPIO_PULLUP;
  }
  else
  {
    GPIO_InitStruct.Pull = GPIO_NOPULL;
  }
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(port, &GPIO_InitStruct);

  return;
}

void set_pin_output(GPIO_TypeDef *port, uint16_t pin)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  HAL_GPIO_WritePin(port, pin, PIN_CLEAR);

  GPIO_InitStruct.Pin = pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(port, &GPIO_InitStruct);

  return;
}

void print_message(char *message)
{
  int i = 0;

  while(message[i] != '\0')
  {
    send_lcd_char(message[i]);
    i++;
  }

  return;
}

/****************************** DHT22 functions *******************************/
void poll_dht22(GPIO_TypeDef *port, uint16_t pin, struct data_t *data)
{
  uint32_t elapsed, databits = 0;
  uint8_t checksum = 0;
  int i;

  //HAL_TIM_Base_Start(&htim2);

  HAL_GPIO_WritePin(port, pin, PIN_SET);
  delay_us(5000);
  HAL_GPIO_WritePin(port, pin, PIN_CLEAR);
  delay_us(8000);
  HAL_GPIO_WritePin(port, pin, PIN_SET);
  delay_us(35);
  set_pin_input(port, pin, DOWN);

  while(HAL_GPIO_ReadPin(port, pin) == PIN_CLEAR); /* lasts 80 microsseconds */

  while(HAL_GPIO_ReadPin(port, pin) == PIN_SET); /* lasts 80 microseconds */

  for(i = 0; i < 40; i++)
  {
    elapsed = 0;
    while(HAL_GPIO_ReadPin(port, pin) == PIN_CLEAR) /* lasts 50 microseconds */
    {
      delay_us(2);
    }

    while(HAL_GPIO_ReadPin(port, pin) == PIN_SET)
    {
      delay_us(2); /*2 seconds per cycle */
      elapsed++;
    }
    //printf("debug: elapsed time = %ld\n\r", elapsed);
    if(i < 32) /* has no hit checksum */
    {
      databits = databits << 1;
      if(elapsed > 25) /* longer than 50 microseconds means bit is 1*/
      {
        databits = databits + 1u;
        //printf("debug: found a 1\n\r");
      }
      //else printf("debug: found a 0\n\r");
    }
    else /* checksum */
    { //printf("debug: on to checksum\n\r");
      checksum = checksum << 1;
      if(elapsed > 25) /* bit is 1*/
      {
        checksum = checksum + 1u;
      }
    }
    //printf("debug i = %d\n\r", i);
    //printf("debug:  databits = %ld\n\r", databits);
  }

  data->rh = (uint16_t)(databits >> 16);
  data->temp = (int16_t)(databits & 0xffff);

  data->integrity = (data->rh >> 8) + (data->rh & 0xff)
                    + (data->temp >> 8) + (data->temp & 0xff)
                    - checksum;


  set_pin_output(port, pin);
  HAL_GPIO_WritePin(port, pin, PIN_SET); /* required for DHT22 */

  return;
}
