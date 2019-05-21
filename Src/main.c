/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "dwt_stm32_delay.h"
#include "stdlib.h"
#define GPIO_PIN_SET 1
#define GPIO_PIN_RESET 0
#define RS_Port GPIOA
#define EN_Port GPIOA
#define D4_Port GPIOA
#define D5_Port GPIOA
#define D6_Port GPIOA
#define D7_Port GPIOA
uint8_t buffer1[20];
uint8_t rx_index =0 ;
uint8_t rx_data;
uint8_t Rh_byte1, Rh_byte2, Temp_byte1, Temp_byte2;
uint16_t sum;
uint8_t arr[7];
int value = 0;
int valueGui = 0;
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
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void LCD_Enable()
{
HAL_GPIO_WritePin(EN_Port,EN_Pin,1);
HAL_Delay(1);
HAL_GPIO_WritePin(EN_Port,EN_Pin,0);	
HAL_Delay(1);	
}

void LCD_Send4Bit(unsigned char Data)
{
HAL_GPIO_WritePin(D4_Port,D4_Pin,Data&0x01);
HAL_GPIO_WritePin(D5_Port,D5_Pin,(Data>>1)&0x01);
HAL_GPIO_WritePin(D6_Port,D6_Pin,(Data>>2)&0x01);
HAL_GPIO_WritePin(D7_Port,D7_Pin,(Data>>3)&0x01);	
}

void LCD_SendCommand(unsigned char command)
{
	LCD_Send4Bit(command >>4);
	LCD_Enable();
	LCD_Send4Bit(command);
	LCD_Enable();
}

void LCD_Clear()
{
 	LCD_SendCommand(0x01);  
  HAL_Delay(1);	
}

void LCD_Init()
{
	HAL_GPIO_WritePin(EN_Port,EN_Pin,0);
	HAL_GPIO_WritePin(RS_Port,RS_Pin,0);
	HAL_Delay(20);
	LCD_Send4Bit(0x30);
	LCD_Enable();
	LCD_Enable();
	LCD_Enable();
	LCD_Send4Bit(0x02);
	LCD_Enable();
	LCD_SendCommand(0x28);
	LCD_SendCommand(0x0C);
	LCD_SendCommand(0x06);
	HAL_Delay(2);
	LCD_SendCommand(0x01);
}


void LCD_Gotoxy(unsigned char x, unsigned char y)
{
unsigned char address;
if(y==0)
address=0x80;
else if(y==1)
{
address=0xc0;
}
else if(y==2)
{
address=0x94;
}
else
address=0xd4;
address+=x;
LCD_SendCommand(address);
}

void LCD_PutChar(unsigned char Data)
{
  HAL_GPIO_WritePin(RS_Port,RS_Pin,1);
 	LCD_SendCommand(Data);
  HAL_GPIO_WritePin(RS_Port,RS_Pin,0);
	
}

void LCD_Puts(char *s)
{
   	while (*s){
      	LCD_PutChar(*s);
     	s++;
   	}
}
GPIO_InitTypeDef GPIO_InitStruct;
void set_gpio_output (void)
{
	/*Configure GPIO pin output: PA2 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	
	
}
void set_gpio_input (void)
{
	/*Configure GPIO pin input: PA2 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

void DHT11_start (void)
{
	
	
	set_gpio_output ();  // set the pin as output
	HAL_GPIO_WritePin (GPIOA, GPIO_PIN_6, 0);   // pull the pin low
	DWT_Delay_us (18000);   // wait for 18ms
	set_gpio_input ();   // set as input
}

void check_response (void)
{
	DWT_Delay_us (40);
	if (!(HAL_GPIO_ReadPin (GPIOA, GPIO_PIN_6)))
	{
		DWT_Delay_us (80);
		
	}
	while ((HAL_GPIO_ReadPin (GPIOA, GPIO_PIN_6)));   // wait for the pin to go low
}

uint8_t read_data (void)
{
	uint8_t i,j;
	for (j=0;j<8;j++)
	{
		while (!(HAL_GPIO_ReadPin (GPIOA, GPIO_PIN_6)));   // wait for the pin to go high
		DWT_Delay_us (40);   // wait for 40 us
		if ((HAL_GPIO_ReadPin (GPIOA, GPIO_PIN_6)) == 0)   // if the pin is low 
		{
			i&= ~(1<<(7-j));   // write 0
		}
		else i|= (1<<(7-j));  // if the pin is high, write 1
		while ((HAL_GPIO_ReadPin (GPIOA, GPIO_PIN_6)));  // wait for the pin to go low
	}
	return i;
}
//
_ARMABI int fputc(int c, FILE * stream)
{
	HAL_UART_Transmit(&huart1,(uint8_t *)&c,1,10);
	return 1;
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
		if(huart->Instance == USART1)
			{
						
						if(rx_index == 0)
							{
									// clear all elements of array
									for(int i = 0; i< 20;i++)
											buffer1[i] = 0;
								}
						if(rx_data != '.')
							{
									buffer1[rx_index++] = rx_data;
								}
						else
							{
									
									buffer1[rx_index] = '\0';
									rx_index = 0;
									if(buffer1[0] == '1')
										HAL_GPIO_TogglePin(GPIOA,led1_Pin);
									else if(buffer1[0] == '2')
										HAL_GPIO_TogglePin(GPIOB,led2_Pin);
									else if(buffer1[0] == '3')
										HAL_GPIO_TogglePin(GPIOB,led3_Pin);
									else if(buffer1[0] == '4')
										HAL_GPIO_TogglePin(GPIOB,led4_Pin);
									else if(buffer1[0] == '5')
										HAL_GPIO_TogglePin(GPIOB,led5_Pin);
									else if(buffer1[0] == '6')
										HAL_GPIO_TogglePin(GPIOB,led6_Pin);
									else if(buffer1[0] == '7')
										HAL_GPIO_TogglePin(GPIOB,led7_Pin);
									else if(buffer1[0] == '8')
										HAL_GPIO_TogglePin(GPIOB,led8_Pin);
								
								}
						HAL_UART_Receive_IT(&huart1,&rx_data,1);
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
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
//	__HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
		HAL_UART_Receive_IT(&huart1,&rx_data,1);
		DWT_Delay_Init();
		LCD_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	uint8_t i =0;
	LCD_Gotoxy(0,0);
	LCD_Puts("SINH");
  while (1)
  {
		DHT11_start ();
		check_response ();
		Rh_byte1 = read_data ();
		Rh_byte2 = read_data ();
		Temp_byte1 = read_data ();
		Temp_byte2 = read_data ();
		
		arr[0] = (Temp_byte1/10)+48;
		arr[1] = (Temp_byte1%10)+48;
		arr[2] = Temp_byte2+48;
		arr[3] = (Rh_byte1/10)+48;
		arr[4] = (Rh_byte1%10)+48;
		arr[5] = (Rh_byte2+48);
		arr[6] = '\0'; 
		
		LCD_Gotoxy(0,0);
		LCD_Puts("TEMP: ");
		LCD_PutChar((Temp_byte1/10)+48);
		LCD_PutChar((Temp_byte1%10)+48);
		LCD_PutChar('.');
		
		LCD_PutChar(Temp_byte2+48);
		LCD_PutChar(0XDF);
		LCD_Puts ("C");
	
		LCD_Gotoxy(0,1);
		LCD_Puts("RH  : ");
		LCD_PutChar((Rh_byte1/10)+48);
		LCD_PutChar((Rh_byte1%10)+48);
		LCD_PutChar('.');
		LCD_PutChar(Rh_byte2+48);
		LCD_PutChar('%'); 
	//	valueGui = atoi((char *)arr);
	
		printf("%s",arr);
	
		
		HAL_Delay(10000);
	//	value++;
		/*uint8_t s[] ="hello world";
		HAL_UART_Transmit(&huart1,s,sizeof(s),1000);
		HAL_Delay(1000);*/
		/*printf("sinh noi nhieu\n");
		HAL_Delay(1000);*/
	/*	HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13);
		printf("%0.1f",data[i]);
		i = (i+1)%10;
		HAL_Delay(5000);*/
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

  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /**Initializes the CPU, AHB and APB busses clocks 
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
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */
	
  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, RS_Pin|EN_Pin|D4_Pin|D5_Pin 
                          |D6_Pin|D7_Pin|led1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, led2_Pin|led3_Pin|led4_Pin|led5_Pin 
                          |led6_Pin|led7_Pin|led8_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : RS_Pin EN_Pin D4_Pin D5_Pin 
                           D6_Pin D7_Pin led1_Pin */
  GPIO_InitStruct.Pin = RS_Pin|EN_Pin|D4_Pin|D5_Pin 
                          |D6_Pin|D7_Pin|led1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : led2_Pin led3_Pin led4_Pin led5_Pin 
                           led6_Pin led7_Pin led8_Pin */
  GPIO_InitStruct.Pin = led2_Pin|led3_Pin|led4_Pin|led5_Pin 
                          |led6_Pin|led7_Pin|led8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
