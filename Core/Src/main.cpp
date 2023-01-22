/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "alt_main.h"
#include <iostream>
#include <string>
#include <string.h>
#include <sstream>
#include <cmath>
using namespace std;
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

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void UART_Transmit(UART_HandleTypeDef &huart, string message, bool newline = true, int timeout_ms = 1000)
{
	if (newline) message += "\r\n";
	HAL_UART_Transmit(&huart, (uint8_t*)message.c_str(), message.length(), timeout_ms);
}

void UART_Transmit(UART_HandleTypeDef &huart, int message, int msgSize = 50, bool newline = true, int timeout_ms = 1000)
{
	char ptr[msgSize] = {'\0'};
	int size = sprintf(ptr, "%d", (int)message);
	string mess = ptr;
	if (newline)
	{
		mess += "\r\n";
		size += 2;
	}
	HAL_UART_Transmit(&huart, (uint8_t*)mess.c_str(), size, timeout_ms);
}

void UART_Transmit(UART_HandleTypeDef &huart, bool message, int msgSize = 50, bool newline = true, int timeout_ms = 1000)
{
	char ptr[msgSize] = {'\0'};
	int size = sprintf(ptr, "%d", (int)message);
	string mess = ptr;
	if (newline)
	{
		mess += "\r\n";
		size += 2;
	}
	HAL_UART_Transmit(&huart, (uint8_t*)mess.c_str(), size, timeout_ms);
}

void UART_Transmit(UART_HandleTypeDef &huart, float message, int msgSize = 50, int precision = 2, bool newline = true, int timeout_ms = 1000)
{
	char ptr[msgSize] = {'\0'};
	int size = sprintf(ptr, "%d.%d", (int)message, (int)((pow(10.0f, precision))*message)%(int)pow(10, precision));
	string mess = ptr;
	if (newline)
	{
		mess += "\r\n";
		size += 2;
	}
	HAL_UART_Transmit(&huart, (uint8_t*)mess.c_str(), size, timeout_ms);
}

void Delay(int milli)
{
	HAL_Delay(milli);
}
////
class PWM
{
public:
	TIM_HandleTypeDef* handle_Time;
	uint32_t channel;
	float duty = 0.0f;

	PWM(TIM_HandleTypeDef *timerHandle, uint32_t channelNumber, float dutyPercentage, bool autoInit = true) : handle_Time(timerHandle)
	{
		if (duty < 0) duty = 0;
		else if (duty > 100) duty = 100;
		else duty = dutyPercentage/100.f;
		if (channelNumber == 1) channel = TIM_CHANNEL_1;
		else if (channelNumber == 2) channel = TIM_CHANNEL_2;
		else if (channelNumber == 3) channel = TIM_CHANNEL_3;
		else if (channelNumber == 4) channel = TIM_CHANNEL_4;
		setDuty(100);
	}
	PWM(TIM_HandleTypeDef *timerHandle, uint32_t channelNumber, bool autoInit = true) : handle_Time(timerHandle)
	{
		if (channelNumber == 1) channel = TIM_CHANNEL_1;
		else if (channelNumber == 2) channel = TIM_CHANNEL_2;
		else if (channelNumber == 3) channel = TIM_CHANNEL_3;
		else if (channelNumber == 4) channel = TIM_CHANNEL_4;
		setDuty(100);
	}
	PWM() {}

	void setDuty(float setDuty)
	{
		duty = setDuty/100.0f;
		unsigned int arr = __HAL_TIM_GET_AUTORELOAD(handle_Time);
		__HAL_TIM_SET_COMPARE(handle_Time, channel, arr * duty);
	}

	float getDuty()
	{
		return duty;
	}

	void init()
	{
		HAL_TIM_PWM_Start(handle_Time, channel);
	}

	void start()
	{
		HAL_TIM_PWM_Start(handle_Time, channel);
	}

	void stop()
	{
		HAL_TIM_PWM_Stop(handle_Time, channel);
	}

};

class Button
{
public:
	bool ledState = false;
	bool lastState = false;
	GPIO_TypeDef *gpio;
	uint16_t nr;
	PWM pwm;

	Button(GPIO_TypeDef *gp, uint16_t n, PWM &p) : gpio(gp), nr(n), pwm(p)
	{

	}
	Button() {}
	void work()
	{
		if(HAL_GPIO_ReadPin(gpio, nr) == 1 && HAL_GPIO_ReadPin(gpio, nr) != lastState)
		{
			if (ledState == 1)
			{
				ledState = 0;
				pwm.stop();
			}
			else
			{
				ledState = 1;
				pwm.start();
			}
		}
		lastState = HAL_GPIO_ReadPin(gpio, nr);
	}
};

#define SLAVE_ADDRESS_LCD 0x4E // change this according to ur setup

void lcd_send_cmd (char cmd)
{
  char data_u, data_l;
	uint8_t data_t[4];
	data_u = (cmd&0xf0);
	data_l = ((cmd<<4)&0xf0);
	data_t[0] = data_u|0x0C;  //en=1, rs=0
	data_t[1] = data_u|0x08;  //en=0, rs=0
	data_t[2] = data_l|0x0C;  //en=1, rs=0
	data_t[3] = data_l|0x08;  //en=0, rs=0
	HAL_I2C_Master_Transmit (&hi2c1, SLAVE_ADDRESS_LCD,(uint8_t *) data_t, 4, 100);
}

void lcd_send_data (char data)
{
	char data_u, data_l;
	uint8_t data_t[4];
	data_u = (data&0xf0);
	data_l = ((data<<4)&0xf0);
	data_t[0] = data_u|0x0D;  //en=1, rs=1
	data_t[1] = data_u|0x09;  //en=0, rs=1
	data_t[2] = data_l|0x0D;  //en=1, rs=1
	data_t[3] = data_l|0x09;  //en=0, rs=1
	HAL_I2C_Master_Transmit (&hi2c1, SLAVE_ADDRESS_LCD,(uint8_t *) data_t, 4, 100);
}

void lcd_init (void)
{
	// 4 bit initialisation
	HAL_Delay(50);  // wait for >40ms
	lcd_send_cmd (0x30);
	HAL_Delay(5);  // wait for >4.1ms
	lcd_send_cmd (0x30);
	HAL_Delay(1);  // wait for >100us
	lcd_send_cmd (0x30);
	HAL_Delay(10);
	lcd_send_cmd (0x20);  // 4bit mode
	HAL_Delay(10);

  // dislay initialisation
	lcd_send_cmd (0x28); // Function set --> DL=0 (4 bit mode), N = 1 (2 line display) F = 0 (5x8 characters)
	HAL_Delay(1);
	lcd_send_cmd (0x08); //Display on/off control --> D=0,C=0, B=0  ---> display off
	HAL_Delay(1);
	lcd_send_cmd (0x01);  // clear display
	HAL_Delay(1);
	HAL_Delay(1);
	lcd_send_cmd (0x06); //Entry mode set --> I/D = 1 (increment cursor) & S = 0 (no shift)
	HAL_Delay(1);
	lcd_send_cmd (0x0C); //Display on/off control --> D = 1, C and B = 0. (Cursor and blink, last two bits)
}

void lcd_send_string (char *str)
{
	while (*str) lcd_send_data (*str++);
}

void lcd_clear (void)
{
	lcd_send_cmd (0x80);
	for (int i=0; i<70; i++)
	{
		lcd_send_data (' ');
	}
}

void lcd_put_cur(int row, int col)
{
    switch (row)
    {
        case 0:
            col |= 0x80;
            break;
        case 1:
            col |= 0xC0;
            break;
    }

    lcd_send_cmd (col);
}

void str2char(string str, char msg[])
{
	for(int i = 0; i < str.length(); i++)
	{
		msg[i] = str[i];
	}
	msg[str.length()] = '\0';
}


void charTest()
{
	char e[16];
	char m[16];

	for(int i = 0; i < 256; i+=32)
	{
		for(int j = 0; j<16; j++)
		{
			e[j] = i+j;
			m[j] = i+16+j;
		}

		lcd_put_cur(0,0);
		lcd_send_string(e);
		lcd_put_cur(1,0);
		lcd_send_string(m);
		HAL_Delay(10000);
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
  MX_USART3_UART_Init();
  MX_TIM2_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_TIM4_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
//  PWM pwm_General(&htim2, 4, false);
//  PWM pwm_Front(&htim3, 1, false);
//  PWM pwm_Rear(&htim3, 2, false);
//  PWM pwm_Blink_Left(&htim3, 3, false);
//  PWM pwm_Blink_Right(&htim3, 4, false);
//
//  Button btn_General(GPIOA, GPIO_PIN_4, pwm_General);
//  Button btn_Front(GPIOB, GPIO_PIN_4, pwm_Front);
//  Button btn_Rear(GPIOB, GPIO_PIN_5, pwm_Rear);
//  Button btn_Blink_Left(GPIOA, GPIO_PIN_15, pwm_Blink_Left);
//  Button btn_Blink_Right(GPIOB, GPIO_PIN_12, pwm_Blink_Right);
  //float abc;
  lcd_init();
  //drawFace(0);
  HAL_ADC_Start(&hadc1);
  //PWM motor4(&htim4, 4, false);
  //PWM motor3(&htim4, 3, false);

  //motor.start();
  //motor.stop();
  //HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_4);
//  DC_MOTOR_Init(DC_MOTOR1);
//  DC_MOTOR_Start(DC_MOTOR1, DIR_CW, 100);

  HAL_ADC
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

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

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
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
