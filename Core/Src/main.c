/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "display.h"
#include "conv.h"
#include "uart.h"
#include "font.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//Przydatne rejstry akcelerometru w projekcie
#define ACC_SLAVE_ADDR 0x3A
#define CTRL_REG4 0x20
#define OUT_X_L 0x28
#define OUT_X_H 0x29
#define OUT_Y_L 0x2A
#define OUT_Y_H 0x2B
#define OUT_Z_L 0x2C
#define OUT_Z_H 0x2D
#define ACC_RESOLUTION 2

//Ograniczneia ekranu
#define min_pos_y 0
#define max_pos_y 47
#define min_pos_x 0
#define max_pos_x 83


//********************************************************************************************

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi3;

UART_HandleTypeDef huart4;

/* USER CODE BEGIN PV */

//struktura przechowująca przeyspieszenie
struct aceleration{
	float x, y, z;
}acc;

//struktura okreslająca polozenie scian
struct constrains{
	uint8_t x1, x2, y1, y2;
};


int8_t mode = 2;

struct constrains walls[] = {
		{0,21,20,20}, 	//pierwsza line
		{21,21,20,39},	//druga line
		{12,21,39,39}, 	//trzecia line
		{12,11,32,39}, 	//czwarta line
		{13,37,10,10},	//piąta lina
		{37,37,0,10}, 	//szósta line
		{32,42,39,39}, 	//śiódma line
		{42,42,39,47}, 	//ósma line
		{36,36,20,31}, //dziewiąta line
		{36,83,31,31}, 	//dziesiąta line
		{60,60,31,39},	//jedynasta line
		{54,59,39,39},	//dwunasta line
		{73,73,42,47},	//trzynasta line
		{48,48,10,21}, 	//czternasta linia
		{48,59,10,10},	//pietnasta linia
		{59,59,0,10},	//szesnasta linia
		{60,71,21,21}, 	//siedemnsta linia
		{71,71,0,21}	//osiamnasta linia
};



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI3_Init(void);
static void MX_UART4_Init(void);
static void MX_I2C2_Init(void);
/* USER CODE BEGIN PFP */


void i2cWrite(uint8_t reg, uint8_t data){
	HAL_I2C_Mem_Write(&hi2c2, ACC_SLAVE_ADDR, reg, 1, &data, 1, 100);;
}

uint8_t i2cRead(uint8_t reg){
	uint8_t data;
	HAL_I2C_Mem_Read(&hi2c2, ACC_SLAVE_ADDR, reg, 1, &data, 1, 100);;
	return data;
}


float odczytX(void)
{

	int16_t axis = (i2cRead(OUT_X_H) << 8) | i2cRead(OUT_X_L);
	return (axis * ACC_RESOLUTION)/(float)(32767);

}

float odczytY(void)
{
	int16_t axis = (i2cRead(OUT_Y_H) << 8) | i2cRead(OUT_Y_L);
	return (axis * ACC_RESOLUTION)/(float)(32767);
	}

float odczytZ(void)
{
	int16_t axis = (i2cRead(OUT_Z_H) << 8) | i2cRead(OUT_Z_L);
	return (axis * ACC_RESOLUTION)/(float)(32767);

}



void ball(void){

	int8_t x_pos=4, y_pos=4, cursorY;		//Zmienne pozycyjne

	displayCmd(0x0d);

 	displayDrawLevel();

  while(1){

	cursorY =  (5 - (y_pos / 8));
	//ustawienie kursora w odpowiednim miejscu
	displaySetCursor(x_pos,cursorY);
	//usuniecie pixela
	displayWrite(labirynth[cursorY * 84 + x_pos]);

    acc.x = odczytX();
    acc.y = odczytY();

    if(acc.y < -0.5){
    	x_pos += 1;
    }else if(acc.y > 0.5){
    	x_pos -= 1;
    }

    if(acc.x > 0.5){
    	y_pos += 1;
    }else if(acc.x < -0.5){
    	y_pos -= 1;
    }

	uint16_t i = 0;


	while(1)
	{

		if((x_pos < walls[i].x1 || x_pos > walls[i].x2)){ ++i; continue; }

		if((y_pos < walls[i].y1 || y_pos > walls[i].y2)){ ++i; continue; }


		if(x_pos >= walls[i].x1 && x_pos <= walls[i].x2){

			if(x_pos == walls[i].x1 && x_pos == walls[i].x2){
				if(acc.y < -0.5){  x_pos -= 1;}
				if(acc.y >  0.5) { x_pos += 1;}
			}

			if(y_pos == walls[i].y1 && y_pos == walls[i].y2){
				if(acc.x >  0.5){  y_pos -= 1;}
				if(acc.x < -0.5){  y_pos += 1;}
			}
		}

		if(y_pos >= walls[i].x1 && y_pos <= walls[i].x1){

			if(x_pos == walls[i].x1 && x_pos == walls[i].x2){
				if(acc.y < -0.5){  x_pos -= 1;}
				if(acc.y >  0.5) { x_pos += 1;}
			}

			if(y_pos == walls[i].y1 && y_pos == walls[i].y2){
			if(acc.x >  0.5){  y_pos -= 1;}
			if(acc.x < -0.5){  y_pos += 1;}
			}
		}

	    //Uwzglednienie odbijania sie od scianek
	    if(x_pos < min_pos_x){	//Osiagnieto pierwsza pozycje skrajna
	      x_pos = min_pos_x + 1;

	    }
	    if(x_pos > max_pos_x){	//Osiagnieto druga pozycje skrajna
	      x_pos = max_pos_x;

	    }
	    if(y_pos < min_pos_y){
	      y_pos = min_pos_y + 1;

	    }
	    if(y_pos > max_pos_y){
	      y_pos = max_pos_y;
	    }

		++i;
		if(i > sizeof(walls)/sizeof(struct constrains)) break;
	}

	//obliczanie rejstru Y
	cursorY = 5 - (y_pos / 8);
	//ustawienie kursora w odpowiednim miejsu
	displaySetCursor(x_pos,cursorY);
	//narysowanie pixela
	displayWrite((0x80 >> (y_pos % 8)) | labirynth[cursorY * 84 + x_pos]);

	if( (y_pos == 46 || y_pos == 45 || y_pos == 44 || y_pos == 43 || y_pos == 42) && ( x_pos == 76 || x_pos == 77 || x_pos == 78 || x_pos == 79 || x_pos == 80 || x_pos == 81 || x_pos == 82)){
		displayClear();
		displaySetCursor(18,3);
		displayPrint("You Won!!!");
		HAL_Delay(2000);
		displayClear();
		x_pos = 4; y_pos = 4;
		displayDrawLevel();
	}
    HAL_Delay(150);	//Ustawienie okresu aktualizacji pomiaru/obrazu
    if(mode != 2){
    	break;
    }

  }
}



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
  MX_SPI3_Init();
  MX_UART4_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */

  uint8_t data = 0x57;
  int16_t axis;
  displayInit();

  i2cWrite(0x20, data);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

		axis = (i2cRead(OUT_X_H) << 8) | i2cRead(OUT_X_L);
		acc.x = (axis * ACC_RESOLUTION)/(float)(32767);

		axis = (i2cRead(OUT_Y_H) << 8) | i2cRead(OUT_Y_L);
		acc.y = (axis * ACC_RESOLUTION)/(float)(32767);

		axis = (i2cRead(OUT_Z_H) << 8) | i2cRead(OUT_Z_L);
		acc.z = (axis * ACC_RESOLUTION)/(float)(32767);

		if(mode == 0){
			displaySetCursor(0,0);
			displayPrint("X =");
			displaySetCursor(15,0);
			displayFloat(acc.x);
			displayPrint("g");


			displaySetCursor(0,2);
			displayPrint("Y =");
			displaySetCursor(15,2);
			displayFloat(acc.y);
			displayPrint("g");


			displaySetCursor(0,4);
			displayPrint("Z =");
			displaySetCursor(15,4);
			displayFloat(acc.z);
			displayPrint("g");

			HAL_Delay(200);
		}
		if(mode == 1){

			displaySetCursor(15,0);
			displayFloat(9.81 * acc.x);
			displayPrint(" m/s");

			displaySetCursor(15,2);
			displayFloat(9.81 * acc.y);
			displayPrint(" m/s");

			displaySetCursor(15,4);
			displayFloat(9.81 * acc.z);
			displayPrint(" m/s");

			HAL_Delay(200);
		}
		if(mode == 2){

			ball();
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
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 400000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_1LINE;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SEL_GPIO_Port, SEL_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, DC_Pin|CE_Pin|RST_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : CS_Pin */
  GPIO_InitStruct.Pin = CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : eode_Pin */
  GPIO_InitStruct.Pin = eode_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(eode_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SEL_Pin */
  GPIO_InitStruct.Pin = SEL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(SEL_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DC_Pin CE_Pin RST_Pin */
  GPIO_InitStruct.Pin = DC_Pin|CE_Pin|RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 4, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
