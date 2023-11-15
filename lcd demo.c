/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * This program demonstrates how to interface with a HD44780 LCD.  It is a
  * modification of the code in
  * https://adastra-soft.com/hd44780-library-for-stm32/
  *
  * The BUSY signal from the HD44780 peripheral is not used.  Sufficiet delay
  * is used instead.
  *
  * The PIN connections can be deduced from the PIN names.
  *
  *
  *****************************************************************************/

/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <string.h>

#define RS_PORT GPIOB
#define RS_PIN RS_Pin
#define RW_PORT GPIOA
#define RW_PIN RW_Pin
#define E_PORT GPIOA
#define E_PIN E_PULSE_Pin
#define DB4_PORT GPIOB
#define DB4_PIN DB4_Pin
#define DB5_PORT GPIOA
#define DB5_PIN DB5_Pin
#define DB6_PORT GPIOA
#define DB6_PIN DB6_Pin
#define DB7_PORT GPIOA
#define DB7_PIN DB7_Pin

#define PIN_CLEAR GPIO_PIN_RESET
#define PIN_SET GPIO_PIN_SET

#define MAX_CHARS 20
#define MAX_SPACE 16
#define MESSAGE "Ben"
#define DELAY 150

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);

void e_pulse(void);
void send_lcd_command(uint8_t);
void send_lcd_char(uint8_t);
void set_data_pins(uint32_t);
void init_lcd(void);
void not_busy(void);
void set_pin_input(GPIO_TypeDef *, uint16_t);
void set_pin_output(GPIO_TypeDef *, uint16_t);
void print_message(char *);

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  int i, shift = 0;
  char message[MAX_CHARS] = MESSAGE;

  /*Reset of all peripherals, Initializes the Flash interface and the Systick.*/
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();

  init_lcd();

  while (1) /* Infinite loop */
  {
    for(i = 0; i < MAX_SPACE; i++)
    {
      shift = 128 + i;  /* move the cursor to the right spot */
      send_lcd_command(shift);

      print_message(" ");
      print_message(message);
      HAL_Delay(DELAY);
    }
    for(i = 0; i < MAX_SPACE; i++)
    {
      shift = 192 + i;  /* move the cursor to the right spot */
      send_lcd_command(shift);

      print_message(" ");
      print_message(message);
      HAL_Delay(DELAY);
    }

  }
  return(0);
}
/****************************** functions *************************************/

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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  return;
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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, DB7_Pin|DB6_Pin|DB5_Pin|RW_Pin
                          |E_PULSE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, RS_Pin|DB4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : DB7_Pin DB6_Pin DB5_Pin RW_Pin
                           E_PULSE_Pin */
  GPIO_InitStruct.Pin = DB7_Pin|DB6_Pin|DB5_Pin|RW_Pin
                          |E_PULSE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : RS_Pin DB4_Pin */
  GPIO_InitStruct.Pin = RS_Pin|DB4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
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

/***************************** LCD functions **********************************/
void e_pulse(void)
/* sends one millisecond pulse to the LCD */
{
  HAL_Delay(1);
  HAL_GPIO_WritePin(E_PORT, E_PIN, PIN_SET);
  HAL_Delay(1);
  HAL_GPIO_WritePin(E_PORT, E_PIN, PIN_CLEAR);

  return;
}

void send_lcd_command(uint8_t command)
{
  set_data_pins(command >> 4); /* first four significant bits */
  HAL_GPIO_WritePin(RS_PORT, RS_PIN, PIN_CLEAR);
  e_pulse();

  set_data_pins(command); /* last four bits */
  e_pulse();
  not_busy();

  return;
}

void send_lcd_char(uint8_t character)
{
  set_data_pins(character >> 4); /* first four significant bits */
  HAL_GPIO_WritePin(RS_PORT, RS_PIN, PIN_SET);
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
    HAL_GPIO_WritePin(DB7_PORT, DB7_PIN, PIN_CLEAR);
  }
  else
  {
    HAL_GPIO_WritePin(DB7_PORT, DB7_PIN, PIN_SET);
  }

  if((data & 0x04u) == 0u)
  {
    HAL_GPIO_WritePin(DB6_PORT, DB6_PIN, PIN_CLEAR);
  }
  else
  {
    HAL_GPIO_WritePin(DB6_PORT, DB6_PIN, PIN_SET);
  }

  if((data & 0x02u) == 0u)
  {
    HAL_GPIO_WritePin(DB5_PORT, DB5_PIN, PIN_CLEAR);
  }
  else
  {
    HAL_GPIO_WritePin(DB5_PORT, DB5_PIN, PIN_SET);
  }

  if((data & 0x01u) == 0u)
  {
    HAL_GPIO_WritePin(DB4_PORT, DB4_PIN, PIN_CLEAR);
  }
  else
  {
    HAL_GPIO_WritePin(DB4_PORT, DB4_PIN, PIN_SET);
  }

  return;
}

void init_lcd(void)
{
  HAL_GPIO_WritePin(RS_PORT, RS_PIN, PIN_CLEAR);
  HAL_GPIO_WritePin(RW_PORT, RW_PIN, PIN_CLEAR);
  HAL_Delay(40);

  set_data_pins(0x03u);
  e_pulse();
  HAL_Delay(5);

  set_data_pins(0x03u);
  e_pulse();
  HAL_Delay(1);

  set_data_pins(0x03u);
  e_pulse();
  HAL_Delay(1);

  set_data_pins(0x02u); /* set to 4-bit mode */
  e_pulse();
  not_busy();

  send_lcd_command(0x28); /* interface lenght 4 bits, 2 lines, 5X7 font */
  send_lcd_command(0x08); /* display off */
  send_lcd_command(0x01); /* clear display */
  send_lcd_command(0x0C); /* display on, cursor off, no blink */
  send_lcd_command(0x06); /* move cursor after each character */

  return;
}

void not_busy(void)
{
  set_pin_input(DB7_PORT, DB7_PIN);

  HAL_GPIO_WritePin(RS_PORT, RS_PIN, PIN_CLEAR);
  HAL_GPIO_WritePin(RW_PORT, RW_PIN, PIN_SET);

  while(HAL_GPIO_ReadPin(DB7_PORT, DB7_PIN) == PIN_CLEAR)
  {
    e_pulse();
    HAL_Delay(1);
  }

  HAL_GPIO_WritePin(RW_PORT, RW_PIN, PIN_CLEAR);

  set_pin_output(DB7_PORT, DB7_PIN);
  return;
}

void set_pin_input(GPIO_TypeDef *port, uint16_t pin)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  GPIO_InitStruct.Pin = pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
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
