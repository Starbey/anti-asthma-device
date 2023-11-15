/*
******************************************************************************
* @file           : main.c
* @brief          : Main program body
******************************************************************************
* @attention
*
* ECE198
*
* Pin assignments:
* RS            PB10
* RW            PA8
* E_PULSE       PA9
* DB4           PB6
* DB5           PA7
* DB6           PA6
* DB7           PA5
*
* DHT22         PA10
*
* P_SENSOR_SDA  PB9
* P_SENSOR_SCL  PB8
*
* BUZZER        PB4
*
* YELL_BUTTON   PB5
* RED_BUTTON    PB3
*
* GREEN_LED     PC0
* RED_LED       PC1
*
* USART2_TX   PA2 - for debugging so you can print to console
* USART2_RX   PA3 - for debugging so you can print to console
*
******************************************************************************
*/
#include "main.h"
#include <stdio.h>
#include <stdlib.h>

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
#define BUZZER_PORT GPIOB
#define BUZZER_PIN BUZZER_Pin
#define YELL_BUTTON_PORT GPIOB
#define YELL_BUTTON_PIN YELL_BUTTON_Pin
#define RED_BUTTON_PORT GPIOB
#define RED_BUTTON_PIN RED_BUTTON_Pin
#define GREEN_LED_PORT GPIOC
#define GREEN_LED_PIN GREEN_LED_Pin
#define RED_LED_PORT GPIOC
#define RED_LED_PIN RED_LED_Pin

#define PIN_CLEAR GPIO_PIN_RESET
#define PIN_SET GPIO_PIN_SET
#define DOWN 0
#define UP 1
#define TRUE 1
#define FALSE 0
#define FOREVER 1

#define PS_ADDR 0x33
#define PM_THRESHOLD 10
#define RH_THRESHOLD 55

#define MAX_CHARS 16
#define NUM_LINES 5
#define LCD_ROW1 0
#define LCD_ROW2 192
#define LABEL1 "Temp  :"
#define LABEL2 "Humid :"
#define LABEL3 "PM1.0 :"
#define LABEL4 "PM2.5 :"
#define LABEL5 "PM10  :"
#define LABEL6 "PM CON:"

#define DATA_MODE 0
#define ALERT_MODE 1
#define ADVANCED_MODE 2

#define BOOT_DELAY 5000000  /* in microseconds */
#define SCREEN_DELAY 3000000  /* in microseconds */
#define POLL_INTERVAL 10000000 /* in microseconds */

#define ALARM_DURATION 3000000 /* in microseconds */
#define SUPP_TIME 20000000 /* in microseconds */
#define ADVANCED_MODE_DETECTED 4000000 /* in microseconds */


/******************************* data structures ******************************/
struct dht_data_t
{
  uint16_t rh;
  uint16_t temp;
  uint8_t integrity; /* 0 is good; other values are bad */
  uint8_t alert; /* alert flag for crossed threshold */
};

struct ps_data_t
{
    uint32_t pm1_0;
    uint32_t pm2_5;
    uint32_t pm10;
    uint8_t error; /* HAL IC2 call OK? */
    uint8_t alert; /* alert flag for crossed threshold */
};

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim5;
UART_HandleTypeDef huart2;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM5_Init(void);
static void MX_USART2_UART_Init(void);

void delay_us(uint16_t);
void delay_us2(uint32_t);
void e_pulse(void);
void send_lcd_command(uint8_t);
void send_lcd_char(uint8_t);
void set_data_pins(uint32_t);
void init_lcd(void);
void not_busy(void);
void set_pin_input(GPIO_TypeDef *, uint16_t, uint8_t);
void set_pin_output(GPIO_TypeDef *, uint16_t);
void print_message(char *);
void boot_display(uint32_t);
void output_data(char [NUM_LINES][MAX_CHARS], struct dht_data_t *,
                  struct ps_data_t *, uint8_t);
uint8_t poll_delay(uint32_t, uint8_t);
uint8_t scroll_delay(uint32_t);

void poll_dht22(GPIO_TypeDef *, uint16_t, struct dht_data_t *);

void poll_psensor(uint16_t, struct ps_data_t *);
uint32_t parse_psensor_data(uint8_t, uint8_t, uint8_t, uint8_t);

uint8_t analyze_data(struct dht_data_t *, struct ps_data_t *);
void sound_alarm(uint32_t);

#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

PUTCHAR_PROTOTYPE  /* To allow printf to console */
{
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}
/************* Global variables set by the EXTI Callback function *************/
uint8_t display_mode, suppress, interrupted, donotbuzz;

/************************************* main ***********************************/
int main(void)
{
  char item[NUM_LINES][MAX_CHARS];
  struct dht_data_t *dht_data = calloc(1, sizeof(struct dht_data_t));
  struct ps_data_t *ps_data = calloc(1, sizeof(struct ps_data_t));
  uint8_t alert = FALSE;

  display_mode = DATA_MODE;
  suppress = FALSE;
  interrupted = FALSE;
  donotbuzz = FALSE;

  /*Reset of all peripherals, Initializes the Flash interface and the Systick.*/
  HAL_Init();
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM5_Init();
  MX_USART2_UART_Init();

  HAL_TIM_Base_Start(&htim2); /* needed by the microsecond timer */
  HAL_TIM_Base_Start(&htim3);
  HAL_TIM_Base_Start(&htim5);

  init_lcd();
  boot_display(BOOT_DELAY);
  HAL_GPIO_WritePin(GREEN_LED_PORT, GREEN_LED_PIN, PIN_SET);

  while (FOREVER)
  {
    poll_dht22(DHT22_PORT, DHT22_PIN, dht_data);  /* get the RH and TEMP */
    poll_psensor(PS_ADDR, ps_data);  /* get the particle counts */

    alert = analyze_data(dht_data, ps_data);

    if(alert == FALSE) /* disable alert led (red) if levels are good */
    {
      HAL_GPIO_WritePin(RED_LED_PORT, RED_LED_PIN, PIN_CLEAR);
      donotbuzz = FALSE;
    }

    if(__HAL_TIM_GET_COUNTER(&htim5) > SUPP_TIME)
    /* suppress timer expired. If threshold is still crossed, sould alarm */
    {
      suppress = FALSE;
    }
printf("debug 61: alert = %d\n\r", alert);
printf("debug 62: suppress = %d\n\r", suppress);
printf("debug 63: donotbuzz = %d\n\r", donotbuzz);

    if(alert == TRUE && suppress == FALSE && donotbuzz == FALSE)
    /* if there is an alert that has not been suppressed */
    {
      HAL_GPIO_WritePin(RED_LED_PORT, RED_LED_PIN, PIN_SET);
      sound_alarm(ALARM_DURATION);
      donotbuzz = TRUE; /* ring the buzzer only once per alarm reset cycle */

    }

    output_data(item, dht_data, ps_data, display_mode);

    delay_us2(POLL_INTERVAL);

    interrupted = FALSE;
  }

  free(dht_data);
  free(ps_data);
  HAL_GPIO_WritePin(GREEN_LED_PORT, GREEN_LED_PIN, PIN_CLEAR);
  HAL_GPIO_WritePin(RED_LED_PORT, RED_LED_PIN, PIN_CLEAR);

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 42;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
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
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 42;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 42;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 4294967295;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GREEN_LED_Pin|RED_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LCD_DB7_Pin|LCD_DB6_Pin|LCD_DB5_Pin|LCD_RW_Pin
                          |LCD_E_Pin|DHT22_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LCD_RS_Pin|BUZZER_Pin|LCD_DB4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : GREEN_LED_Pin RED_LED_Pin */
  GPIO_InitStruct.Pin = GREEN_LED_Pin|RED_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_DB7_Pin LCD_DB6_Pin LCD_DB5_Pin LCD_RW_Pin
                           LCD_E_Pin DHT22_Pin */
  GPIO_InitStruct.Pin = LCD_DB7_Pin|LCD_DB6_Pin|LCD_DB5_Pin|LCD_RW_Pin
                          |LCD_E_Pin|DHT22_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_RS_Pin BUZZER_Pin LCD_DB4_Pin */
  GPIO_InitStruct.Pin = LCD_RS_Pin|BUZZER_Pin|LCD_DB4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : RED_BUTTON_Pin YELL_BUTTON_Pin */
  GPIO_InitStruct.Pin = RED_BUTTON_Pin|YELL_BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
}

/* Callback for GPIO external interrupts (from the red or yellow button) */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if(GPIO_Pin == YELL_BUTTON_PIN)
  {
    /* Check how long the yellow button is pressed */
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    while(HAL_GPIO_ReadPin(YELL_BUTTON_PORT, YELL_BUTTON_PIN) == PIN_SET);

    if(__HAL_TIM_GET_COUNTER(&htim2) > ADVANCED_MODE_DETECTED)
    {
      display_mode = ADVANCED_MODE;
    }

    else if(display_mode == DATA_MODE) /* toggle to ALERT_MODE */
    {
      display_mode = ALERT_MODE;
    }
    else /* toggle to DATA_MODE */
    {
      display_mode = DATA_MODE;
    }
    interrupted = TRUE;
    //delay_us(1000);
    printf("yellow button pressed\n\r");
  }
  else if(GPIO_Pin == RED_BUTTON_PIN)
  {
    suppress = TRUE;
    interrupted = TRUE;
    donotbuzz = FALSE;
    HAL_GPIO_WritePin(RED_LED_PORT, RED_LED_PIN, PIN_CLEAR);
    display_mode = DATA_MODE;  /* always go to data mode after suppressing */
    __HAL_TIM_SET_COUNTER(&htim5, 0);
    //delay_us(50);
  }
  else
  {
      __NOP();
  }

  return;
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
  printf("In Error_Handler with a system error.\n\r");
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
  /*User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/******************************** Delay Functions *****************************/
void delay_us(uint16_t delay)
{
  __HAL_TIM_SET_COUNTER(&htim3, 0);
  while(__HAL_TIM_GET_COUNTER(&htim3) < delay);
  return;
}

void delay_us2(uint32_t delay)
{
  __HAL_TIM_SET_COUNTER(&htim2, 0);
  while(__HAL_TIM_GET_COUNTER(&htim2) < delay && interrupted == FALSE);

  return;
}

void boot_display(uint32_t delay)
{
  int i;
  char countdown[5];

  send_lcd_command(LCD_ROW1);
  print_message("   Booting Up");

  for(i = delay / 1000000; i >= 0; i--)
  {
    sprintf(countdown, " %d ", i);
    send_lcd_command(LCD_ROW2 + 6);
    print_message(countdown);
    HAL_GPIO_TogglePin(GREEN_LED_PORT, GREEN_LED_PIN);
    HAL_Delay(950);
  }

  return;
}

/***************************** LCD functions **********************************/
void e_pulse(void)
/* sends one millisecond pulse to the LCD */
{
  delay_us(1000);
  HAL_GPIO_WritePin(LCD_E_PORT, LCD_E_PIN, PIN_SET);
  delay_us(1000);
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
  HAL_GPIO_WritePin(LCD_RS_PORT, LCD_RS_PIN, PIN_CLEAR);
  HAL_GPIO_WritePin(LCD_RW_PORT, LCD_RW_PIN, PIN_CLEAR);

  HAL_Delay(40);

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
void poll_dht22(GPIO_TypeDef *port, uint16_t pin, struct dht_data_t *data)
{
  uint32_t elapsed, databits = 0;
  uint8_t checksum = 0;
  int i;

  HAL_GPIO_WritePin(port, pin, PIN_SET);
  delay_us(1000);
  HAL_GPIO_WritePin(port, pin, PIN_CLEAR);
  delay_us(5000);
  HAL_GPIO_WritePin(port, pin, PIN_SET);
  delay_us(35);
  set_pin_input(port, pin, DOWN);

  __HAL_TIM_SET_COUNTER(&htim3, 0);
  while(HAL_GPIO_ReadPin(port, pin) == PIN_CLEAR &&
        (__HAL_TIM_GET_COUNTER(&htim3) < 100)); /* lasts 80 microseconds */


  __HAL_TIM_SET_COUNTER(&htim3, 0);
  while(HAL_GPIO_ReadPin(port, pin) == PIN_SET &&
        (__HAL_TIM_GET_COUNTER(&htim3) < 100));  /* lasts 80 microseconds */


  for(i = 0; i < 40; i++)
  {
    elapsed = 0;
//__HAL_TIM_SET_COUNTER(&htim5, 0);
    while(HAL_GPIO_ReadPin(port, pin) == PIN_CLEAR); /* lasts 50 microseconds */
//printf("debug340: elapsed time = %ld\n\r", __HAL_TIM_GET_COUNTER(&htim5));

    __HAL_TIM_SET_COUNTER(&htim2, 0);
    while(HAL_GPIO_ReadPin(port, pin) == PIN_SET);
    elapsed = __HAL_TIM_GET_COUNTER(&htim2);

    //printf("debug350: elapsed time = %ld\n\r", elapsed);

    if(i < 32) /* data before checksum */
    {
      databits = databits << 1;
      if(elapsed > 50) /* longer than 50 microseconds means bit is 1*/
      {
        databits = databits + 1u;
        //printf("debug: found a 1\n\r");
      }
      //else printf("debug: found a 0\n\r");
    }
    else /* checksum */
    {
      checksum = checksum << 1;
      if(elapsed > 50) /* bit is 1*/
      {
        checksum = checksum + 1u;
      }
    }
  }

  data->rh = (uint16_t)(databits >> 16);
  data->temp = (int16_t)(databits & 0xffff);

  data->integrity = (data->rh >> 8) + (data->rh & 0xff)
                    + (data->temp >> 8) + (data->temp & 0xff)
                    - checksum;

  set_pin_output(port, pin);
  HAL_GPIO_WritePin(port, pin, PIN_SET); /* signal DHT to go back to sleep */

  return;
}

/*********************** SN-GCJA5 PSensor Functions ***************************/
void poll_psensor(uint16_t psensor_addr, struct ps_data_t *data)
{
  uint8_t rawdata[12];
  HAL_StatusTypeDef ret;

  /* Cannot use HAL_IC2_Master_Transmit(Receive) as specs expects no stop bit
     after writing to the register.  Reads 12 bytes starting from byte 0x0000 */
  ret = HAL_I2C_Mem_Read(&hi2c1, (psensor_addr << 1), (uint16_t)0,
                      I2C_MEMADD_SIZE_8BIT, rawdata, 12, HAL_MAX_DELAY);

  if(ret != HAL_OK)
  {
    data->error = TRUE;
  }
  else
  {
    data->pm1_0 = parse_psensor_data(rawdata[0], rawdata[1],
                                      rawdata[2], rawdata[3]);
    data->pm2_5 = parse_psensor_data(rawdata[4], rawdata[5],
                                      rawdata[6], rawdata[7]);
    data->pm10 = parse_psensor_data(rawdata[8], rawdata[9],
                                      rawdata[10], rawdata[11]);
    data->error = FALSE;
  }

  return;
}

uint32_t parse_psensor_data(uint8_t d1, uint8_t d2, uint8_t d3, uint8_t d4)
/* converts little endian to big endian */
{
  uint32_t value = 0x0;

  value = (uint32_t)d1;
  value = value | (uint32_t)d2 << 8;
  value = value | (uint32_t)d3 << 16;
  value = value | (uint32_t)d4 << 24;

  return(value);
}

/*********************** Data analysis and warnings ***************************/
 uint8_t analyze_data(struct dht_data_t *dht, struct ps_data_t *ps)
 {
   uint8_t threshold = FALSE;

   if(dht->rh > RH_THRESHOLD * 10)
   {
     threshold = TRUE;
     dht->alert = TRUE;
   }
   else
   {
     dht->alert = FALSE;
   }

   if((ps->pm2_5 + ps->pm10) > (PM_THRESHOLD  * 1000))
   {
     threshold = TRUE;
     ps->alert = TRUE;
   }
   else
   {
     ps->alert = FALSE;
   }

   return(threshold);
 }

 void sound_alarm(uint32_t duration)
 {
   __HAL_TIM_SET_COUNTER(&htim2, 0);

   while(__HAL_TIM_GET_COUNTER(&htim2) < duration && interrupted == FALSE)
   {
     HAL_GPIO_TogglePin(BUZZER_PORT, BUZZER_PIN);
     delay_us(125);
   }

   return;
 }

/*************************** display functions ********************************/
void output_data(char items[NUM_LINES][MAX_CHARS], struct dht_data_t *dht,
                  struct ps_data_t *ps, uint8_t mode)
{
  char rh[MAX_CHARS], temp[MAX_CHARS / 2], pm1_0[MAX_CHARS / 2],
       pm2_5[MAX_CHARS / 2], pm10[MAX_CHARS / 2], pm2_5to10[MAX_CHARS];
  uint8_t i;

  /* convert data from sensors to strings */
  if(dht->integrity == 0) /* DHT data is good */
  {
    if(mode == ALERT_MODE) /* in alert mode */
    {
      if(dht->alert == TRUE)
      {
        sprintf(rh, "EXCEEDED");
      }
      else
      {
        sprintf(rh, "SAFE");
      }
    }
    else /* not in alert mode */
    {
      sprintf(rh, "%d.%d%%", dht->rh / 10, dht->rh % 10);
    }

    if((dht->temp >> 15) == 0) /* temperature is positive if first bit is 0 */
    {
      sprintf(temp, "%d.%dC", dht->temp / 10, dht->temp % 10);
    }
    else /* temperature is negative */
    {
      sprintf(temp, "-%d.%dC", (dht->temp & 0x7fff) / 10, dht->temp % 10);
    }
  }
  else /* DHT data is bad */
  {
    sprintf(rh, "N.A.");
    sprintf(temp, "N.A.");
  }

  if(ps->error == FALSE) /* psensor data is good */
  {
    sprintf(pm1_0, "%lu.%lu", (ps->pm1_0 / 1000), (ps->pm1_0 % 1000));
    sprintf(pm2_5, "%lu.%lu", (ps->pm2_5 / 1000), (ps->pm2_5 % 1000));
    sprintf(pm10, "%lu.%lu", (ps->pm10 / 1000), (ps->pm10 % 1000));

    if(mode == ALERT_MODE)
    {
      if(ps->alert == TRUE)
      {
        sprintf(pm2_5to10, "EXCEEDED");
      }
      else
      {
        sprintf(pm2_5to10, "SAFE");
      }
    }
    else /* not in alert mode */
    {
      sprintf(pm2_5to10, "%lu.%lu", ((ps->pm2_5 + ps->pm10) / 1000),
                                    ((ps->pm2_5 + ps->pm10) % 1000));
    }
  }
  else /* psensor data is bad */
  {
    sprintf(pm1_0, "N.A.");
    sprintf(pm2_5, "N.A.");
    sprintf(pm10, "N.A.");
    sprintf(pm2_5to10, "N.A.");
  }

  /* format the lines to display */
  if(mode == DATA_MODE || mode == ALERT_MODE) /* not advanced mode */
  {
    sprintf(items[0], "%s %s", LABEL2, rh);
    sprintf(items[1], "%s %s", LABEL6, pm2_5to10);

    init_lcd();
    send_lcd_command(LCD_ROW1);
    print_message(items[0]);
    send_lcd_command(LCD_ROW2);
    print_message(items[1]);
  }
  else /* advanced mode */
  {
    sprintf(items[0], "%s %s", LABEL1, temp);
    sprintf(items[1], "%s %s", LABEL2, rh);
    sprintf(items[2], "%s %s", LABEL3, pm1_0);
    sprintf(items[3], "%s %s", LABEL4, pm2_5);
    sprintf(items[4], "%s %s", LABEL5, pm10);

    for(i = 0; i < NUM_LINES; i++) /* scroll through the lines */
    {
      init_lcd();
      i = i % NUM_LINES;
      send_lcd_command(LCD_ROW1);
      print_message(items[i]);

      send_lcd_command(LCD_ROW2);
      print_message(items[(i + 1) % NUM_LINES]);

      delay_us2(SCREEN_DELAY);

    }
  }
  return;
}
