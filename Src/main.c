#include "main.h"
#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_flash.h"
#include "string.h"
#include <stdio.h>
#include <stdlib.h>
#include "flash.h"
#include "ring_buffer.h"
#include <stdlib.h>
/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);

typedef enum
{
	WAIT_BEGIN = 0,
	INCOMMING
}	State_Uart; 

State_Uart State_Receive_Data = WAIT_BEGIN;
uint8_t buffer[32];
SER_RING_BUF_T Ring_Buffer;

	uint32_t Number_On = 0;
	uint32_t Number_Off =  0;

	char Buffer_On[32];
	char Buffer_Off[32];

	

int main(void)
 {
	
  HAL_Init();

  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
	
	HAL_UART_Receive_IT(&huart2,(uint8_t*)	"haha", 1000);
	//HAL_UART_Transmit(&huart2,"Hello, Connected to STM32L476RG",strlen("Hello, Connected to STM32L476RG"),1000);
	//HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5);
	
  /* USER CODE BEGIN WHILE */
	
	uint8_t number_data = 0;
	uint8_t Read_Ring_Buffer;
	uint32_t Number_On_Off[2];

	 
//		uint32_t test[2] = {'R','U'};
		initFlash();
//		eraseFlash(0x0806F800, 256);
//		writeFlash(0x0806F800, test, 2);
	
  while (1)
  {	
		
		readFlash(0x0804F800, Number_On_Off,2);
		Number_On =  0xFFFFFFFF - Number_On_Off[0];
		Number_Off = 0xFFFFFFFF - Number_On_Off[1];
		
		if(!SER_RING_BUF_EMPTY(Ring_Buffer))			
		{
			
			switch(State_Receive_Data)
			{
				case WAIT_BEGIN:
				{
					if(SER_RING_BUF_RD(Ring_Buffer) == '$')
					{
						State_Receive_Data = INCOMMING;
					}
					break;
				}
				
				case INCOMMING:
				{
						Read_Ring_Buffer = SER_RING_BUF_RD(Ring_Buffer);
						if( Read_Ring_Buffer != ';')
						{
								buffer[number_data] = Read_Ring_Buffer;
								number_data ++;
							
						}
						
						else 
						{
								if(memcmp(buffer, "ON",2) == 0)
								{
										if(HAL_GPIO_ReadPin( GPIOA,GPIO_PIN_5) == 0)
										{ 
												HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_SET);
											  HAL_UART_Transmit(&huart2,"ON LED", strlen("ON LED"),1000);
												number_data = 0;
										    memset(buffer, 0 ,32);
												State_Receive_Data = WAIT_BEGIN;
											
												Number_On = Number_On + 1;
												Number_On_Off[0] = 0xFFFFFFFF - Number_On;
											
												eraseFlash(0x0804F800, 256);
												writeFlash(0x0804F800 ,Number_On_Off, 2);
										}	
										else
										{
												HAL_UART_Transmit(&huart2,"LED IS ON", strlen("LED IS ON"),1000);	
												number_data = 0;
										    memset(buffer, 0 ,32);
											  State_Receive_Data = WAIT_BEGIN;
										}
										

								}
								
							  else if(memcmp(buffer, "OFF",3) == 0)
								{
										if(HAL_GPIO_ReadPin( GPIOA,GPIO_PIN_5) == 1)
										{
												HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_RESET);
												HAL_UART_Transmit(&huart2,"LED OFF", strlen("LED OFF"),1000);
												number_data = 0;
										    memset(buffer, 0 ,32);
												State_Receive_Data = WAIT_BEGIN;
											
												Number_Off = Number_Off + 1;
												Number_On_Off[1] = 0xFFFFFFFF - Number_Off;
											
												eraseFlash(0x0804F800, 256);
												writeFlash(0x0804F800 ,Number_On_Off, 2);
										}
										
										else
										{
											HAL_UART_Transmit(&huart2,"LED IS OFF", strlen("LED IS OFF"),1000);
											number_data = 0;
										  memset(buffer, 0 ,32);
											State_Receive_Data = WAIT_BEGIN;
										}
										

								}
								
								else if(memcmp(buffer, "HELLO",5) == 0)
								{
										HAL_UART_Transmit(&huart2,"HELLO, CONNECTED WITH MCU STM32L476RG",strlen("HELLO, CONNECTED WITH MCU STM32L476RG"),1000);
										number_data = 0;
										memset(buffer, 0 ,32);
										State_Receive_Data = WAIT_BEGIN;
									
								}
								
								else if (memcmp(buffer,"NUMBER ON OFF",13) == 0)
								{									
										number_data = 0;
										memset(buffer, 0 ,32); // set buffer ve tat ca gia tri 0
										State_Receive_Data = WAIT_BEGIN;
										
									
										sprintf(Buffer_On,"%d",Number_On);
										sprintf(Buffer_Off,"%d",Number_Off);
									
										HAL_UART_Transmit(&huart2,"Number ON:",strlen("Number ON:"),1000);
										HAL_UART_Transmit(&huart2,Buffer_On,32,1000);
										HAL_UART_Transmit(&huart2,"\r\n",strlen("\r\n"),1000);
									
										HAL_UART_Transmit(&huart2,"Number OFF:",strlen("Number OFF:"),1000);
										HAL_UART_Transmit(&huart2,Buffer_Off,32,1000);
										HAL_UART_Transmit(&huart2,"\r\n",strlen("\r\n"),1000);
									
								}
								else if(memcmp(buffer,"CLEAR NUMBER ON OFF",20) == 0 )
								{
										number_data = 0;
										memset(buffer, 0 ,32); // set buffer ve tat ca gia tri 0
										State_Receive_Data = WAIT_BEGIN;
									
										eraseFlash(0x0804F800, 256);
										HAL_UART_Transmit(&huart2,"CLEARED NUMBER ON OFF",strlen("CLEARED NUMBER ON OFF"),1000);
									
																			
								}
							
								else
								{
									HAL_UART_Transmit(&huart2,"COMMAND NOT DEFINE", strlen("COMMAND NOT DEFINE"),1000);
									number_data = 0;
									memset(buffer, 0 ,32); // set buffer ve tat ca gia tri 0
									State_Receive_Data = WAIT_BEGIN;
								}

						}
						
						break;						
				}
				
				default:
					break;
				
			}
			
		}
		
//		eraseFlash(0x0804F800, 256);
//		writeFlash(0x0804F800 ,Number_On_Off, 2);
				
  }
  /* USER CODE END 3 */

}

void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the main internal regulator output voltage 
    */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);

  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
