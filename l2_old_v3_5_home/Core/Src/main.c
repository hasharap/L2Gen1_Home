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
#include "stdio.h"
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
DMA_HandleTypeDef hdma_adc1;

IWDG_HandleTypeDef hiwdg;

RTC_HandleTypeDef hrtc;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart1_tx;

/* USER CODE BEGIN PV */
uint16_t debug_count = 0;
bool watchdog_active = false;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_IWDG_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_RTC_Init(void);
/* USER CODE BEGIN PFP */
void voltage_measure();
void voltage_l1(void);
void adc_calibret(void);
void run_led(void);
void SetRTC(void);

void transmitSerial(void);
bool cp_calibrate = false;

volatile char serialRxPort1 = 0;
static volatile int32_t serialPort1TxCount = 0;
static volatile uint32_t serialPort1RxCount = 0;

volatile char serialRxPort2 = 0;
static volatile int32_t serialPort2TxCount = 0;
static volatile uint32_t serialPort2RxCount = 0;

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
	MX_DMA_Init();
	MX_ADC1_Init();
	MX_TIM1_Init();
	MX_TIM3_Init();
	MX_USART1_UART_Init();
//  MX_IWDG_Init();
	MX_USART3_UART_Init();
//  MX_RTC_Init();
	/* USER CODE BEGIN 2 */

	/*---------------------------------------------------
	 Initialization function
	 ---------------------------------------------------*/
	HAL_TIM_Base_Start_IT(&htim3);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
	HAL_ADCEx_Calibration_Start(&hadc1);
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*) adc_store, 11);
	HAL_UART_Receive_IT(&huart1, (uint8_t*) &serialRxPort2, 1);

#if RTC_ACTIVE
	MX_RTC_Init();
	initRTC();
#endif

	init_dataStructures();

	/*---------------------------------------------------
	 Get HW version
	 ---------------------------------------------------*/
	get_HWVersion();

	/*---------------------------------------------------
	 calibrate CP values
	 ---------------------------------------------------*/
	calibrateCP(0);
//	cp_map(1);
	cp_calibrate = true;

	/*---------------------------------------------------
	 Reset GFI sensor
	 ---------------------------------------------------*/
	GFIC_RESET_ON();
	HAL_Delay(100);
	GFIC_RESET_OFF();

	buzzer_en = true;
	run_GFITest = true;
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	/*---------------------------------------------------
	 Initialize serial peripheral and Modbus peripheral
	 ---------------------------------------------------*/
	CSMS_Init();
	ModbusRTU_Init();

	tick_clear(&timeout);
	tick_clear(&gfi_test);
	tick_clear(&rtc_checkCounter);
	tick_clear(&rtc_updateCounter);
	tick_clear(&load_balance6s);
	tick_clear(&error_handler);
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		/*---------------------------------------------------
		 Charger init check
		 ---------------------------------------------------*/
		if (charger_init_flag == false)
		{
			/*---------------------------------------------------
			 Vehicle check functions
			 ---------------------------------------------------*/
			bootup_vehicleCheck();

			if ((networkSide_bootup == true)		//Serial check
					&& (bootup_timeUpdate == true)			//RTC check
					&& ((timeout.timeout_30s == true) || (emeter_bootup == true)))//Modbus check
			{
				charger_init_flag = true;
			}
		}

		if (charger_setup_flag == false)
		{
			get_chargerConfig();
			charger_setup_flag = true;
		}

		if (charger_configSet.config_enable == true)
		{
			set_chargerConfig();
			charger_configSet.config_enable = false;
			HAL_Delay(100);
			HAL_NVIC_SystemReset();
		}

		if (charger_configSet.config_counter == 10)
		{

		}

		/*---------------------------------------------------
		 Vehicle connector function
		 ---------------------------------------------------*/
		check_vehicleConnector();
		check_PWMActive();

		/*---------------------------------------------------
		 Power side monitoring functions
		 ---------------------------------------------------*/
		//		gfci_sense();
		if (gfic_test_run && (!stuck_relay_test))
		{
			//			GFCI_test();
		}
		else
		{
			if ((powerSide_data.errorStatus.bit.error_SR_C == 1)
					|| (powerSide_data.errorStatus.bit.error_GFI_test == 1))
			{
				powerSide_data.status.bit.powerSide_ready = 0;
			}
			else
			{
				powerSide_data.status.bit.powerSide_ready = 1;
			}

			if (controlSide_data.networkSide_request.bit.vehicle_Check == 1)
			{
				powerSide_data.status.bit.powerSide_ready = 1; //when the power restore
			}
		}

		/*---------------------------------------------------
		 Diode check function
		 ---------------------------------------------------*/
#if DIODE_CHECK
		if (timeout.timeout_0_5s == true)
		{
			diodeCheck();
		}
#else
		diodeCheck_passed = true;
#endif
		/*---------------------------------------------------
		 Error logger
		 ---------------------------------------------------*/
		errorBuffer_log();
		errorBuffer_report();

		/*---------------------------------------------------
		 Energy meter reading functions
		 ---------------------------------------------------*/
		ModbusRTU_functions();
		get_EnergyMeterData();

		/*---------------------------------------------------
		 Serial CRC calculation and filter functions
		 ---------------------------------------------------*/
#if SERIAL_ACTIVE
		if (charger_init_flag == true)
		{
			CSMS_functions();
		}

		transmitSerial();
#endif
		/*---------------------------------------------------
		 RTC functions
		 ---------------------------------------------------*/
#if RTC_ACTIVE
		//---------------------------------------------------- Update time at boot up
		if ((bootup_timeUpdate == false)
		/*&& (networkSide_data.timeReady == 1)*/)
		{
			bootup_timeUpdate = true;
			get_date();
			HAL_RTC_GetTime(&hrtc, &RtcTime, RTC_FORMAT_BIN);
			HAL_RTC_GetDate(&hrtc, &RtcDate, RTC_FORMAT_BIN);

			uint32_t unix_time_set = RTC_ToEpoch(&RtcTime, &RtcDate);
			HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR1, (unix_time_set & 0xFFFF));
			HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR2, (unix_time_set >> 16));
		}

		//---------------------------------------------------- Update alarms
		if (networkSide_data.alarmUpdate == 1)
		{
			updateAlarm();
		}
		//---------------------------------------------------- Alarm check
		if (rtc_checkCounter.timeout_10s == true)
		{
			tick_clear(&rtc_checkCounter);
			if ((networkSide_data.isInternet_available == 1)
					&& (networkSide_data.timeReady == 1))
			{
//				RtcTime.Hours = networkSide_data.setTime.Hours;
//				RtcTime.Minutes = networkSide_data.setTime.Minutes;
//				RtcDate.Date = networkSide_data.setDate.Date;
//				RtcDate.Month = networkSide_data.setDate.Month;
//				RtcDate.Year = networkSide_data.setDate.Year;
				updateRTC();
				updateAlarm();
			}
			else
			{
				HAL_RTC_GetTime(&hrtc, &RtcTime, RTC_FORMAT_BIN);
				HAL_RTC_GetDate(&hrtc, &RtcDate, RTC_FORMAT_BIN);

				uint32_t unix_time_set = RTC_ToEpoch(&RtcTime, &RtcDate);
				HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR1,
						(unix_time_set & 0xFFFF));
				HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR2, (unix_time_set >> 16));

				updateAlarm();
			}

			checkWeekday_Time();

			if (controlSide_data.networkSide_request.bit.rtcUpdateComplete
					== true)
			{
				controlSide_data.networkSide_request.bit.rtcUpdateComplete =
				false;
			}
			if (controlSide_data.networkSide_request.bit.rtcUpdateAlarmComplete
					== true)
			{
				controlSide_data.networkSide_request.bit.rtcUpdateAlarmComplete =
				false;
			}
		}
#else
		bootup_timeUpdate = true;
#endif

		/*---------------------------------------------------
		 Debug LED run function - added to debug serial error
		 ---------------------------------------------------*/
		debug_count++;
		if (debug_count >= 6000)
		{
			debug_count = 0;
			HAL_GPIO_TogglePin(BOARD_LED_GPIO_Port, BOARD_LED_Pin);
			temperature_NTC();
		}

		/*---------------------------------------------------
		 Energy meter reading functions
		 ---------------------------------------------------*/
		get_EnergyMeterData();

		/*---------------------------------------------------
		 Energy calculation functions
		 ---------------------------------------------------*/
		calculate_Energy();

		/*---------------------------------------------------
		 Watchdog function
		 ---------------------------------------------------*/
#if WATCHDOG_ENABLE
		if (watchdog_active == false)
		{
			watchdog_active = true;
			MX_IWDG_Init();
		}

		HAL_IWDG_Refresh(&hiwdg);
#endif
		/*---------------------------------------------------
		 Buzzer function
		 ---------------------------------------------------*/
		if (buzzer_en == true)
		{
			buzzer_en = false;
			buzzer_on();
		}
#if ERROR_ALARM
		errorAlarm_set();
		errorAlarm_sound();
#endif

		/*---------------------------------------------------
		 OTA Update
		 ---------------------------------------------------*/
		if (OTA_flag == true)
		{
			OTA_flag = false;
			Flag_up();
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
	RCC_OscInitTypeDef RCC_OscInitStruct =
	{ 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct =
	{ 0 };
	RCC_PeriphCLKInitTypeDef PeriphClkInit =
	{ 0 };

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI
			| RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.LSIState = RCC_LSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
	{
		Error_Handler();
	}
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC | RCC_PERIPHCLK_ADC;
	PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
	PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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

	ADC_ChannelConfTypeDef sConfig =
	{ 0 };

	/* USER CODE BEGIN ADC1_Init 1 */

	/* USER CODE END ADC1_Init 1 */

	/** Common config
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
	hadc1.Init.ContinuousConvMode = ENABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 11;
	if (HAL_ADC_Init(&hadc1) != HAL_OK)
	{
		Error_Handler();
	}

	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_0;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_7CYCLES_5;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}

	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_1;
	sConfig.Rank = ADC_REGULAR_RANK_2;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}

	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_2;
	sConfig.Rank = ADC_REGULAR_RANK_3;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}

	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_3;
	sConfig.Rank = ADC_REGULAR_RANK_4;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}

	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_4;
	sConfig.Rank = ADC_REGULAR_RANK_5;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}

	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_5;
	sConfig.Rank = ADC_REGULAR_RANK_6;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}

	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_6;
	sConfig.Rank = ADC_REGULAR_RANK_7;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}

	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_8;
	sConfig.Rank = ADC_REGULAR_RANK_8;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}

	/** Configure Regular Channel
	 */
	sConfig.Rank = ADC_REGULAR_RANK_9;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}

	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
	sConfig.Rank = ADC_REGULAR_RANK_10;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}

	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_VREFINT;
	sConfig.Rank = ADC_REGULAR_RANK_11;
	sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN ADC1_Init 2 */

	/* USER CODE END ADC1_Init 2 */

}

/**
 * @brief IWDG Initialization Function
 * @param None
 * @retval None
 */
static void MX_IWDG_Init(void)
{

	/* USER CODE BEGIN IWDG_Init 0 */

	/* USER CODE END IWDG_Init 0 */

	/* USER CODE BEGIN IWDG_Init 1 */

	/* USER CODE END IWDG_Init 1 */
	hiwdg.Instance = IWDG;
	hiwdg.Init.Prescaler = IWDG_PRESCALER_64;
	hiwdg.Init.Reload = 4094;
	if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN IWDG_Init 2 */

	/* USER CODE END IWDG_Init 2 */

}

/**
 * @brief RTC Initialization Function
 * @param None
 * @retval None
 */
static void MX_RTC_Init(void)
{

	/* USER CODE BEGIN RTC_Init 0 */

	/* USER CODE END RTC_Init 0 */

	RTC_TimeTypeDef sTime =
	{ 0 };
	RTC_DateTypeDef DateToUpdate =
	{ 0 };

	/* USER CODE BEGIN RTC_Init 1 */

	/* USER CODE END RTC_Init 1 */

	/** Initialize RTC Only
	 */
	hrtc.Instance = RTC;
	hrtc.Init.AsynchPrediv = RTC_AUTO_1_SECOND;
	hrtc.Init.OutPut = RTC_OUTPUTSOURCE_ALARM;
	if (HAL_RTC_Init(&hrtc) != HAL_OK)
	{
		Error_Handler();
	}

	/* USER CODE BEGIN Check_RTC_BKUP */

	/* USER CODE END Check_RTC_BKUP */

	/** Initialize RTC and set the Time and Date
	 */
	sTime.Hours = 0x0;
	sTime.Minutes = 0x0;
	sTime.Seconds = 0x0;

	if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
	{
		Error_Handler();
	}
	DateToUpdate.WeekDay = RTC_WEEKDAY_MONDAY;
	DateToUpdate.Month = RTC_MONTH_JANUARY;
	DateToUpdate.Date = 0x1;
	DateToUpdate.Year = 0x0;

	if (HAL_RTC_SetDate(&hrtc, &DateToUpdate, RTC_FORMAT_BCD) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN RTC_Init 2 */

	/* USER CODE END RTC_Init 2 */

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

	TIM_MasterConfigTypeDef sMasterConfig =
	{ 0 };
	TIM_OC_InitTypeDef sConfigOC =
	{ 0 };
	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig =
	{ 0 };

	/* USER CODE BEGIN TIM1_Init 1 */

	/* USER CODE END TIM1_Init 1 */
	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 71;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 999;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
	{
		Error_Handler();
	}
	sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
	sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
	sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
	sBreakDeadTimeConfig.DeadTime = 0;
	sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
	sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
	sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
	if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM1_Init 2 */

	/* USER CODE END TIM1_Init 2 */
	HAL_TIM_MspPostInit(&htim1);

}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void)
{

	/* USER CODE BEGIN TIM3_Init 0 */

	/* USER CODE END TIM3_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig =
	{ 0 };
	TIM_MasterConfigTypeDef sMasterConfig =
	{ 0 };

	/* USER CODE BEGIN TIM3_Init 1 */

	/* USER CODE END TIM3_Init 1 */
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 500;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 71;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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
	/* USER CODE BEGIN TIM3_Init 2 */

	/* USER CODE END TIM3_Init 2 */

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
	huart1.Init.WordLength = UART_WORDLENGTH_9B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_EVEN;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart1) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN USART1_Init 2 */
	HAL_UART_Receive_IT(&huart1, (uint8_t*) &serialRxPort1, 1); // start uart rx process
	/* USER CODE END USART1_Init 2 */

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
	huart3.Init.BaudRate = 9600;
	huart3.Init.WordLength = UART_WORDLENGTH_8B;
	huart3.Init.StopBits = UART_STOPBITS_1;
	huart3.Init.Parity = UART_PARITY_NONE;
	huart3.Init.Mode = UART_MODE_TX_RX;
	huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart3.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart3) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN USART3_Init 2 */
	HAL_UART_Receive_IT(&huart3, (uint8_t*) &serialRxPort2, 1); // start uart rx process
	/* USER CODE END USART3_Init 2 */

}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void)
{

	/* DMA controller clock enable */
	__HAL_RCC_DMA1_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA1_Channel1_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 1, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
	/* DMA1_Channel4_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct =
	{ 0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */
	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(EC_TEST_GPIO_Port, EC_TEST_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB,
			OCP_RESET_Pin | GFIC_TEST_Pin | GFIC_RESET_Pin | BOARD_LED_Pin
					| BUZZER_Pin | L_RELAY_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : EC_TEST_Pin */
	GPIO_InitStruct.Pin = EC_TEST_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(EC_TEST_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : GFIC_TEST_SENSE_Pin GFIC_SENSE_Pin C_FB_Pin MODE_SET_Pin */
	GPIO_InitStruct.Pin = GFIC_TEST_SENSE_Pin | GFIC_SENSE_Pin | C_FB_Pin
			| MODE_SET_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pins : OCP_RESET_Pin GFIC_TEST_Pin GFIC_RESET_Pin BOARD_LED_Pin
	 BUZZER_Pin L_RELAY_Pin */
	GPIO_InitStruct.Pin = OCP_RESET_Pin | GFIC_TEST_Pin | GFIC_RESET_Pin
			| BOARD_LED_Pin | BUZZER_Pin | L_RELAY_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin : PHASE_SET_Pin */
	GPIO_InitStruct.Pin = PHASE_SET_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(PHASE_SET_GPIO_Port, &GPIO_InitStruct);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
//################################################################################
//--------------------------------------------------------------------------------
//			Timer interrupt handler	- 500us
//--------------------------------------------------------------------------------
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM3)
	{
		/*---------------------------------------------------
		 Timers count functions
		 ---------------------------------------------------*/
		tick_count(&timeout);
		tick_count(&gfi_test);
		tick_count(&load_balance6s);
		tick_count(&error_handler);
		tick_count(&rtc_checkCounter);

		/*---------------------------------------------------
		 Main state machine function
		 ---------------------------------------------------*/
		check_vehicleConnector();
		get_GPIOFeedback();

		if (charger_init_flag == true)
		{
			load_balance(0);
			monitor_cp();

			/*---------------------------------------------------
			 Error logger
			 ---------------------------------------------------*/
			errorDetector();
			errorHandler();

			sm_control_run();
		}

		/*---------------------------------------------------
		 CP duty set function
		 ---------------------------------------------------*/
		if (cp_calibrate == true)
		{
			setCP_duty();
		}

		static uint8_t CSMS_trigCount = 0;

		CSMS_trigCount++;

		if (CSMS_trigCount > 1) // 500 us adapter
		{
			CSMS_trigCount = 0;
			CSMS_Intick(); // 1ms timer
			ModbusRTU_Intick();
		}
	}
}

//################################################################################
//--------------------------------------------------------------------------------
//			ADC interrupt handler - used to run CP voltage function
//--------------------------------------------------------------------------------

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	/* Prevent unused argument(s) compilation warning */
	getCP_voltage();

	VREF = ((1.2 / adc_store[10]) * 4095.0);
	VREF_SUM = VREF_SUM + VREF;
	vref_count++;

	if (vref_count >= 20)
	{
		VREF_AVG = (VREF_SUM / vref_count);
		VREF_SUM = 0.0;
		vref_count = 0;
	}
	/* NOTE : This function should not be modified. When the callback is needed,
	 function HAL_ADC_ConvCpltCallback must be implemented in the user file.
	 */
}

//################################################################################
//--------------------------------------------------------------------------------
//			UART RX interrupt handler - used for MODBUS
//--------------------------------------------------------------------------------

/*----------------------------------------------------------------------------
 * UART Wrappers
 *----------------------------------------------------------------------------*/

static void huart1TransmitIT(uint8_t *pData, uint16_t Length)
{
	UART_HandleTypeDef *huart = &huart1;
	if (HAL_UART_Transmit_IT(huart, pData, Length) == HAL_OK)
	{
		serialPort1TxCount++;
	}
	else
	{
		serialPort1TxCount--;
	}
}

static void huart3TransmitIT(uint8_t *pData, uint16_t Length)
{
	UART_HandleTypeDef *huart = &huart3;
	if (HAL_UART_Transmit_IT(huart, pData, Length) == HAL_OK)
	{
		serialPort2TxCount++;
	}
	else
	{
		serialPort2TxCount--;
	}
}

void transmitSerial(void)
{
	CSMS_transmitSerial(huart1TransmitIT);
	ModbusRTU_transmitSerial(huart3TransmitIT);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
#if SERIAL_ACTIVE
	if (huart == &huart1)
	{
		serialRxPort1 = huart->Instance->DR;

		CSMS_receiveSerial((char*) &serialRxPort1);
		HAL_UART_Receive_IT(&huart1, (uint8_t*) &serialRxPort1, 1);
		serialPort1RxCount++;
	}
#endif
	if (huart == &huart3)
	{
		serialRxPort2 = huart->Instance->DR;

		ModbusRTU_receiveSerial((char*) &serialRxPort2);
		HAL_UART_Receive_IT(&huart3, (uint8_t*) &serialRxPort2, 1);
		serialPort2RxCount++;
	}
}

//################################################################################
//--------------------------------------------------------------------------------
//			UART TX interrupt handler - used for MODBUS
//--------------------------------------------------------------------------------
//void transmitSerialPort1(void)
//{
//	UART_HandleTypeDef *huart = &huart1;
//	if (1)
//	{
////		serialTxFlag = false;
//		static uint8_t instruction = 0;
//
//		if ((CSMS_Queue.ActiveMask & (1 << instruction)) != 0x0) // check whether activation mask not zero
//		{
//			if (HAL_UART_Transmit_IT(huart,
//					(uint8_t*) CSMS_Queue.TxData[instruction],
//					CSMS_Queue.Length[instruction]) == HAL_OK)
//			{
//
//			}
//
//			CSMS_Queue.ActiveMask &= ~(1 << instruction); // reset activation for instruction
//			instruction++; // update instruction starting point
//
//			if ((instruction == CSMS_QUEUE_SIZE)
//					|| (CSMS_Queue.ActiveMask == 0))
//			{
//				instruction = 0;
//			}
//		}
//	}
//}
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
