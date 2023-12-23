/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include "vl53l7cx_api.h"
#include "vl53l7cx_plugin_motion_indicator.h"
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LED1_ON()	HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET)
#define LED1_OFF()	HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET)
#define LED2_ON()	HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET)
#define LED2_OFF()	HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET)

#define BTN1()	HAL_GPIO_ReadPin(BTN1_GPIO_Port, BTN1_Pin)
#define BTN2()	HAL_GPIO_ReadPin(BTN2_GPIO_Port, BTN2_Pin)
#define INT_PIN()	HAL_GPIO_ReadPin(INT_GPIO_Port, INT_Pin)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;

/* USER CODE BEGIN PV */

extern uint8_t UserRxBufferFS[256];

int status;
VL53L7CX_Configuration 	Dev;
VL53L7CX_ResultsData 	Results;
VL53L7CX_Motion_Configuration 	motion_config;

uint8_t resolution=64, ranging_frequency=10, sharpener_percent=50;
uint16_t integration_time=20;
uint8_t data_to_transfer=0;
uint8_t isAlive;


char data[260];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C2_Init(void);
/* USER CODE BEGIN PFP */

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
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */

  //configure platform
  Dev.platform.i2c = hi2c2;
  Dev.platform.address = VL53L7CX_DEFAULT_I2C_ADDRESS;
  Dev.platform.rsc_io.port_name=GPIOB;
  Dev.platform.rsc_io.pin_number=GPIO_PIN_5;
  Dev.platform.lpn_io.port_name=GPIOB;
  Dev.platform.lpn_io.pin_number=GPIO_PIN_0;
  Dev.platform.int_io.port_name=GPIOB;
  Dev.platform.int_io.pin_number=GPIO_PIN_1;

  status = vl53l7cx_resetSensor(&Dev.platform);
  status += vl53l7cx_enableLP(&Dev.platform);
  status += vl53l7cx_is_alive(&Dev, &isAlive);

  if(!isAlive) LED1_OFF();
  else LED1_ON();

  status += vl53l7cx_init(&Dev);
  status += vl53l7cx_set_ranging_mode(&Dev, VL53L7CX_RANGING_MODE_AUTONOMOUS);
  status += vl53l7cx_set_resolution(&Dev, VL53L7CX_RESOLUTION_8X8);
  status += vl53l7cx_set_ranging_frequency_hz(&Dev, 10);
  status += vl53l7cx_set_integration_time_ms(&Dev, 20);
  status += vl53l7cx_set_sharpener_percent(&Dev, 0);
  status += vl53l7cx_start_ranging(&Dev);

  if(status)LED2_OFF();
  else LED2_ON();

  HAL_Delay(1000);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  //data transfer
	  if(UserRxBufferFS[0]!=0)
	  {

		  //set
		  if((UserRxBufferFS[0]=='S')&&(UserRxBufferFS[1]=='E')&&(UserRxBufferFS[2]=='T'))
		  {
			  LED2_OFF();

			  resolution=UserRxBufferFS[3];
			  ranging_frequency=UserRxBufferFS[4];
			  integration_time=UserRxBufferFS[5]<<8;
			  integration_time+=UserRxBufferFS[6];
			  sharpener_percent=UserRxBufferFS[7];
			  data_to_transfer=UserRxBufferFS[8];

			  vl53l7cx_stop_ranging(&Dev);
			  vl53l7cx_set_resolution(&Dev, resolution);
			  vl53l7cx_set_ranging_frequency_hz(&Dev, ranging_frequency);
			  vl53l7cx_set_integration_time_ms(&Dev, integration_time);
			  vl53l7cx_set_sharpener_percent(&Dev, sharpener_percent);
			  vl53l7cx_start_ranging(&Dev);

			  LED2_ON();
		  }
		  //get
		  if((UserRxBufferFS[0]=='G')&&(UserRxBufferFS[1]=='E')&&(UserRxBufferFS[2]=='T'))
		  {
				CDC_Transmit_FS((uint8_t*)data,resolution*3+1);
				HAL_Delay(1);
		  }
		  UserRxBufferFS[0]=0;
	  }

	  //sensor readout
	  if(!vl53l7cx_getINTstatus(&Dev.platform))
	  {
			LED1_ON();

			status = vl53l7cx_get_resolution(&Dev, &resolution);
			status = vl53l7cx_get_ranging_data(&Dev, &Results);

			int j=0;
			for(int i = 0; i < resolution;i++)
			{
				j=i*3;
				switch(data_to_transfer)
				{
					case 0:	//distance
						data[j+0]=0;
						data[j+1]=Results.distance_mm[i]>>8;
						data[j+2]=Results.distance_mm[i]&0xFF;
						break;
					case 1: //sigma
						data[j+0]=0;
						data[j+1]=Results.range_sigma_mm[i]>>8;
						data[j+2]=Results.range_sigma_mm[i]&0xFF;
						break;
					case 2: //reflectance
						data[j+0]=0;
						data[j+1]=0;
						data[j+2]=Results.reflectance[i]&0xFF;
						break;
					case 3: //target status
						data[j+0]=0;
						data[j+1]=0;
						data[j+2]=Results.target_status[i]&0xFF;
						break;
					case 4: //num of targets detected
						data[j+0]=0;
						data[j+1]=0;
						data[j+2]=Results.nb_target_detected[i]&0xFF;
						break;
					case 5: //signal per spad
						data[j+0]=Results.signal_per_spad[i]>>16;
						data[j+1]=Results.signal_per_spad[i]>>8;
						data[j+2]=Results.signal_per_spad[i]&0xFF;
						break;
					case 6: //ambient per spad
						data[j+0]=Results.ambient_per_spad[i]>>16;
						data[j+1]=Results.ambient_per_spad[i]>>8;
						data[j+2]=Results.ambient_per_spad[i]&0xFF;
						break;
					case 7: //num of spads
						data[j+0]=Results.nb_spads_enabled[i]>>16;
						data[j+1]=Results.nb_spads_enabled[i]>>8;
						data[j+2]=Results.nb_spads_enabled[i]&0xFF;
						break;
					case 8://motion
						//To be implemented
						break;

				}
			}
			data[j+3]=Results.silicon_temp_degc;
	  }
	  else LED1_OFF();

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
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED1_Pin|LED2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LPn_Pin|RSC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : BTN1_Pin BTN2_Pin */
  GPIO_InitStruct.Pin = BTN1_Pin|BTN2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LED1_Pin LED2_Pin */
  GPIO_InitStruct.Pin = LED1_Pin|LED2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LPn_Pin RSC_Pin */
  GPIO_InitStruct.Pin = LPn_Pin|RSC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : INT_Pin */
  GPIO_InitStruct.Pin = INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(INT_GPIO_Port, &GPIO_InitStruct);

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
