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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
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
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart3;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE BEGIN PV */
uint32_t pMillis;
uint32_t Value1 = 0;
uint32_t Value2 = 0;
uint16_t Distance  = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_SPI1_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
	typedef enum
	{
		Idle_State,
		Key_Press_State,
		Check_Password_State,
		Incorrect_Password_State,
		Correct_Password_State,
		Sound_Alarm_State
	} eSystemState;

	//uint8_t SPIdata[] = {0xf6,0x12,0xea,0xba,0x1e,0xbc,0xfc,0x92,0xfe,0xbe};
	//depends on 74hc595 connection
	// Q7 =B, Q6 = A, Q5 = G, Q4 = C, Q3 = DP, Q2 = D, Q1 = E, Q0 = F
	uint8_t SPIdata[] ={0xd7,	// 0 = a,b,c,d,e,f		1101 0111
						0x90,	// 1 = b,c				1001 0000
						0xe6,	// 2 = a,b,d,e,g		1110 0110
						0xf4,	// 3 = a,b,c,d,g		1111 0100
						0xb1,	// 4 = b,c,f,g			1011 0001
						0x75,	// 5 = a,c,d,f,g		0111 0101
						0x77,	// 6 = a,c,d,e,f,g		0111 0111
						0xd0,	// 7 = a,b,c			1101 0000
						0xf7,	// 8 = a,b,c,d,e,f,g	1111 0111
						0xf1};	// 9 = a,b,c,f,g		1111 0001
	uint8_t letters[] = {0xf3, // a 0
						0x37, // b 1
						0x47, // c 2
						0xb6, // d 3
						0x97, // u 4
						0x67, // e
						0x63, // f
						0x57, // g
						0xb3, // h
						0x03, // i
						0x94, // j
						0x07, // l
						0xd3, // n
						0xd7, // o
						0xe3, // p
						0xf1, // q
						0x43, // r
						0x55, // s

						0x16, // v
						0xb5, // y
						0xc6, // z

	};
	int password = 1234;
	int numPress = 0;
	int buffer = 0;

	int armed = 0; // if 0 false. true = 1



	void sendSPIdata(uint8_t data[], uint8_t byteSize)
	{
		//clear the buffer
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET); //MASTER RECLEAR, ACTIVE LOW - PIN 10

		//Disable the Outputs
		HAL_GPIO_WritePin(GPIOA , GPIO_PIN_6, GPIO_PIN_SET);		//OUT_EN, ACTIVE LOW - PIN 13

		//Open the Buffer for Transmission
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET); //MASTER RECLEAR, ACTIVE LOW - PIN 10

		//transmit data
		HAL_SPI_Transmit(&hspi1, &data[0], byteSize, 10); //byteSize = 1

		//Trigger Registers
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET); //register clk - pin 11
		//HAL_Delay(10);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);

		//Enable the Outputs
		HAL_GPIO_WritePin(GPIOA , GPIO_PIN_6 , GPIO_PIN_RESET); //OUT_EN, ACTIVE LOW - PIN 13
	}
	int displayDigit(int num){
		uint16_t cathode[4] = {0x8000,0x2000,0x1000,0x4000}; // (Pin 15)(Pin 13)(Pin 12)(Pin 14)
		int d[4];			// array that will sort one digit in each index
		int count =0;		// how many digits num contains

		if(num == 0){
			d[count] = 0;
			count = 1;
		}
		//it will sort one digit in each index
		else{
			while(num != 0){
				d[count] = num%10;
				count++;
				num=num/10;
			}
		}

		// display up to four digits on the 4-digit 7-seg display
		for(int i = 0; i < count; i++){
			HAL_GPIO_WritePin(GPIOF, cathode[i], GPIO_PIN_RESET); 	// resets one cathode to GND
			sendSPIdata(&SPIdata[d[i]], 1);							// send data to the register
			HAL_Delay(4);											// it shows that number for 4 ms
			HAL_GPIO_WritePin(GPIOF, cathode[i], GPIO_PIN_SET);		// sets the cathode
		}
		return count;
	}
	int displayLetters(int num){
			uint16_t cathode[4] = {0x8000,0x2000,0x1000,0x4000}; // (Pin 15)(Pin 13)(Pin 12)(Pin 14)
			int d[4];			// array that will sort one digit in each index
			int count =0;		// how many digits num contains

			if(num == 0){
				d[count] = 0;
				count = 1;
			}
			//it will sort one digit in each index
			else{
				while(num != 0){
					d[count] = num%10;
					count++;
					num=num/10;
				}
			}

			// display up to four digits on the 4-digit 7-seg display
			for(int i = 0; i < count; i++){
				HAL_GPIO_WritePin(GPIOF, cathode[i], GPIO_PIN_RESET); 	// resets one cathode to GND
				sendSPIdata(&letters[d[i]], 1);							// send data to the register
				HAL_Delay(4);											// it shows that number for 4 ms
				HAL_GPIO_WritePin(GPIOF, cathode[i], GPIO_PIN_SET);		// sets the cathode
			}
			return count;
		}
	void displayPassword(int num, int press){
		uint16_t cathode[4] = {0x8000,0x2000,0x1000,0x4000};
		int count = displayDigit(num);				// count = 3

		for(int i = 0; i < press; i++){
			if(count - 1 < i){
				HAL_GPIO_WritePin(GPIOF, cathode[i], GPIO_PIN_RESET);
				sendSPIdata(&SPIdata[0], 1);
				HAL_Delay(4);
				HAL_GPIO_WritePin(GPIOF, cathode[i], GPIO_PIN_SET);
			}
		}


	}
	int checkColumn(uint16_t pinVar){
			int num;
			HAL_GPIO_WritePin(GPIOE, C1_Pin|C2_Pin|C3_Pin|C4_Pin, GPIO_PIN_RESET);

			HAL_GPIO_WritePin(GPIOE,C1_Pin,GPIO_PIN_SET);
			if(HAL_GPIO_ReadPin(GPIOE,pinVar) == 1)
				num = 1;

			HAL_GPIO_WritePin(GPIOE,C1_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE,C2_Pin,GPIO_PIN_SET);
			if(HAL_GPIO_ReadPin(GPIOE,pinVar) == 1)
				num = 2;

			HAL_GPIO_WritePin(GPIOE,C2_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE,C3_Pin,GPIO_PIN_SET);
			if(HAL_GPIO_ReadPin(GPIOE,pinVar) == 1)
				num = 3;

			HAL_GPIO_WritePin(GPIOE,C3_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE,C4_Pin,GPIO_PIN_SET);
			if(HAL_GPIO_ReadPin(GPIOE,pinVar) == 1)
				num = 4;
			HAL_GPIO_WritePin(GPIOE, C1_Pin|C2_Pin|C3_Pin|C4_Pin, GPIO_PIN_SET);
			return num;
		}
	int keyPress(void){
		int columnVar;
		int keyArr[] = {1,2,3,11,4,5,6,12,7,8,9,13,16,0,15,14}; //0-9 = 0-9, A-D = 11-14, #-* = 15-16

		if(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_2) == 1){		// row 1
			columnVar = checkColumn(GPIO_PIN_2);
			return keyArr[columnVar - 1];
		}
		else if(HAL_GPIO_ReadPin(GPIOE, R2_Pin) == 1){		// row 2
			columnVar = checkColumn(R2_Pin);
			return keyArr[columnVar +3];
		}
		else if(HAL_GPIO_ReadPin(GPIOE, R3_Pin) == 1){		// row 3
			columnVar = checkColumn(R3_Pin);
			return keyArr[columnVar +7];
		}
		else if(HAL_GPIO_ReadPin(GPIOE, R4_Pin) == 1){		// row 4
			columnVar = checkColumn(R4_Pin);
			return keyArr[columnVar +11];
		}
		return -1;
	}

	uint16_t getDistance(){
		HAL_TIM_Base_Start(&htim1);
		HAL_GPIO_WritePin(GPIOF, GPIO_PIN_7, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOF, GPIO_PIN_7, GPIO_PIN_SET);  // pull the TRIG pin HIGH
			  	     __HAL_TIM_SET_COUNTER(&htim1, 0);
			  	     while (__HAL_TIM_GET_COUNTER (&htim1) < 10);  // wait for 10 us
			  	     HAL_GPIO_WritePin(GPIOF, GPIO_PIN_7, GPIO_PIN_RESET);  // pull the TRIG pin low

			  	     pMillis = HAL_GetTick(); // used this to avoid infinite while loop  (for timeout)
			  	     // wait for the echo pin to go high
			  	     while (!(HAL_GPIO_ReadPin (GPIOF, GPIO_PIN_9)) && pMillis + 10 >  HAL_GetTick());
			  	     Value1 = __HAL_TIM_GET_COUNTER (&htim1);

			  	     pMillis = HAL_GetTick(); // used this to avoid infinite while loop (for timeout)
			  	     // wait for the echo pin to go low
			  	     while ((HAL_GPIO_ReadPin (GPIOF, GPIO_PIN_9)) && pMillis + 50 > HAL_GetTick());
			  	     Value2 = __HAL_TIM_GET_COUNTER (&htim1);

			  	     Distance = (Value2-Value1)* 0.034/2;
			  	     return Distance;
	}
/*Prototype Event Handlers*/
	eSystemState IdleStateHandler(void){
		if(armed == 1){
			HAL_GPIO_WritePin(GPIOG, GPIO_PIN_14, 1);
		}
		else{
			HAL_GPIO_WritePin(GPIOG, GPIO_PIN_14, 0);
		}
		if(numPress > 0)
			displayPassword(buffer,numPress);
		//HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, 1);
		// displayLetters(3422);
		if(keyPress() != -1)
			return Key_Press_State;
			//return Idle_State;
		if(armed == 1){
			if(getDistance() < 30){
				return Sound_Alarm_State;
			}
		}
		//return Key_Press_State;
		return Idle_State;
	}
	eSystemState KeyPressHandler(void){ 		// 0-9 = 0-9, A-D = 11-14, #-* = 15-16
		int num = keyPress();
		switch (num){

		case 11:								// 11 = A, This is enter
			HAL_Delay(500);
			return Check_Password_State;

		case 12:								// 12 = B, This is backspace
				if(numPress > 0){
					buffer = buffer / 10;
					numPress--;
				}
				HAL_Delay(500);
				return Idle_State;

		case 13:								// 13 = C, This is clear
			buffer = 0;
			numPress = 0;
			HAL_Delay(500);
			return Idle_State;

		default:								// 0 - 9 was pressed
			if(numPress < 4 && num < 10){
				buffer = buffer * 10 + num;
				numPress++;
			}
			HAL_Delay(500);
			return Idle_State;
		}
	}
	eSystemState CheckPasswordHandler(void){
		if(buffer == password)
			return Correct_Password_State;
		return Incorrect_Password_State;
	}
	eSystemState IncorrectPasswordHandler(void){
		buffer = 0;
		numPress = 0;
		//TODO
		/*
		 HAl
		 * DELAY .5 SEC
		 * TURN OFF RED LED
		 *
		 * TURN ON RED LED
		 * DELAY .5 SEC
		 * TURN OFF RED LED
		 *
		 * TURN ON RED LED
		 * DELAY .5 SEC
		 * TURN OFF RED LED
		 *
		 */
		HAL_GPIO_WritePin(GPIOG, GPIO_PIN_9, 1);
		HAL_Delay(500);
		HAL_GPIO_WritePin(GPIOG, GPIO_PIN_9, 0);

		return Idle_State;
	}
	eSystemState CorrectPasswordHandler(void){
		buffer = 0;
		numPress = 0;
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, 0);
		if(armed == 1)
			armed = 0;
		else
			armed = 1;
		return Idle_State;
	}
	eSystemState SoundAlarmHandler(void){
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, 1);
		return Idle_State;
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
  MX_USB_OTG_FS_PCD_Init();
  MX_SPI1_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  eSystemState eNextState = Idle_State;
  // sendSPIdata(&SPIdata[8], 1);

  /*
	  switch(eNextState){
	  case Idle_State:
		  eNextState = IdleStateHandler();
		  break;
	  case Key_Press_State:
		  eNextState = KeyPressHandler();
		  break;
	  case Check_Password_State:
		  eNextState = CheckPasswordHandler();
		  break;
	  case Incorrect_Password_State:
		  eNextState = IncorrectPasswordHandler();
		  break;
	  case Correct_Password_State:
		  eNextState = CorrectPasswordHandler();
		  break;
	  }
	  displayDigit(getDistance());

   */
  	HAL_TIM_Base_Start(&htim1);
  	while(HAL_GetTick() < 4000){
  		displayLetters(3422);
  	}
    //HAL_GPIO_WritePin(GPIOF, GPIO_PIN_7, GPIO_PIN_RESET);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	  switch(eNextState){
	  	  case Idle_State:
	  		  eNextState = IdleStateHandler();
	  		  break;
	  	  case Key_Press_State:
	  		  eNextState = KeyPressHandler();
	  		  break;
	  	  case Check_Password_State:
	  		  eNextState = CheckPasswordHandler();
	  		  break;
	  	  case Incorrect_Password_State:
	  		  eNextState = IncorrectPasswordHandler();
	  		  break;
	  	  case Correct_Password_State:
	  		  eNextState = CorrectPasswordHandler();
	  		  break;
	  	  case Sound_Alarm_State:
	  		  eNextState = SoundAlarmHandler();
	  		  break;
	  	  }
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 144;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 6;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00301739;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 71;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 6;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, C1_Pin|C2_Pin|C3_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, OUT_EN_Pin|RCLK_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|GPIO_PIN_2|LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, D2_Pin|D3_Pin|D1_Pin|D4_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8|C4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SRCLR_GPIO_Port, SRCLR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, USB_PowerSwitchOn_Pin|GPIO_PIN_9|GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pins : R1_Pin R2_Pin R3_Pin R4_Pin */
  GPIO_InitStruct.Pin = R1_Pin|R2_Pin|R3_Pin|R4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : C1_Pin PE8 C2_Pin C3_Pin
                           C4_Pin */
  GPIO_InitStruct.Pin = C1_Pin|GPIO_PIN_8|C2_Pin|C3_Pin
                          |C4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : TRIG_Pin */
  GPIO_InitStruct.Pin = TRIG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(TRIG_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ECHO_Pin */
  GPIO_InitStruct.Pin = ECHO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ECHO_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RMII_MDC_Pin RMII_RXD1_Pin */
  GPIO_InitStruct.Pin = RMII_MDC_Pin|RMII_RXD1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : RMII_REF_CLK_Pin RMII_MDIO_Pin */
  GPIO_InitStruct.Pin = RMII_REF_CLK_Pin|RMII_MDIO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : OUT_EN_Pin RCLK_Pin */
  GPIO_InitStruct.Pin = OUT_EN_Pin|RCLK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin PB2 LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|GPIO_PIN_2|LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : D2_Pin D3_Pin D1_Pin D4_Pin */
  GPIO_InitStruct.Pin = D2_Pin|D3_Pin|D1_Pin|D4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : PB11 */
  GPIO_InitStruct.Pin = GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : RMII_TXD1_Pin */
  GPIO_InitStruct.Pin = RMII_TXD1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(RMII_TXD1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SRCLR_Pin */
  GPIO_InitStruct.Pin = SRCLR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SRCLR_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_PowerSwitchOn_Pin PG9 PG14 */
  GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin|GPIO_PIN_9|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RMII_TX_EN_Pin RMII_TXD0_Pin */
  GPIO_InitStruct.Pin = RMII_TX_EN_Pin|RMII_TXD0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

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
