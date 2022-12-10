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
#include "tim.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "stdbool.h"
#include "math.h"
#include "usbd_cdc_if.h"
//#include "arm_math.h"
#include "FIRFilter.h"
#include "pid.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define SIGNAL_SAMPLES_LENGTH 100

typedef struct{
	float buf[SIGNAL_SAMPLES_LENGTH];
	uint8_t bufIndex;

	float energy;
}SignalFeature_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ARM_THRESHOLD 	0.7
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint32_t adc[2], adc_buffer[2];
int fsr;
int arm_pressure;
uint16_t emg_raw;
uint8_t arm_state = 0;
uint8_t arm_state_manual = 0;
uint8_t calibration_counter = 0;
int16_t arm_minimum_signal;
uint8_t arm_minimum_signal_is_captured = 0;
int16_t arm_maximum_signal;
float signal_energy;

float sum_normalized_arm_signalsqr;
uint8_t signal_energy_cnt_length = 0;
uint8_t signal_energy_length = 100; //sample 100
float normalized_arm_signal;
float emg_rawbfr;
float emg_rawdiff;

uint8_t truth_counter_maxval = 75;
uint8_t truth_counter_thresh;
bool prosthetic_state = false;
bool prosthetic_statebfr;
int arm_condition_thresh = 120;

float kp = 0.45;
float ti = 0.1;
float td = 0.01;
float dt = 0.0005;
float pid_out;


FIRFilter mav;
SignalFeature_t sig;
PID_t pid;

int servo_pwm = 1500;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void Signal_Buf_Init(SignalFeature_t *signal){
	//Clear Filter Buffer
	for(uint8_t n = 0; n < SIGNAL_SAMPLES_LENGTH; n++){
		signal->buf[n] = 0.0f;
	}

	//Clear Buf Index
	signal->bufIndex = 0;

	//Clear Filter Output
	signal->energy = 0.0f;
}

void Signal_Buf_Update(SignalFeature_t *signal, float inp){
	/*Store Latest Sample in buffer */
	signal->buf[signal->bufIndex] = inp;

	/*increment buffer index and wrap around if necessary*/
	signal->bufIndex++;

	if(signal->bufIndex == SIGNAL_SAMPLES_LENGTH){
		signal->bufIndex = 0;
	}
}

float Signal_Energy_Calculate(SignalFeature_t *signal){
	signal->energy = 0.0f;

	uint8_t sumIndex = signal->bufIndex;
	for(uint8_t n = 0; n < SIGNAL_SAMPLES_LENGTH; n++){
		/*Decrement Index and Wrap if Necessary*/
		if(sumIndex < SIGNAL_SAMPLES_LENGTH -1){
			sumIndex++;
		}else{
			sumIndex = 0;
		}

		/*Multiply Impulse Response with Shifted input sample and add to output*/
		signal->energy += (signal->buf[n] * signal->buf[n]);
	}
	return signal->energy;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
   PID_Init(&pid, kp, ti, td, dt);
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
  MX_USB_DEVICE_Init();
  MX_TIM10_Init();
  MX_ADC1_Init();
  MX_TIM9_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim10);
  HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_1);
  HAL_ADC_Start_DMA(&hadc1, adc_buffer, 2);

  FIRFilter_Init(&mav);
  Signal_Buf_Init(&sig);
//  PID_Init(&pid, kp, ti, td, dt);
//  pid.kp = kp;
//  pid.ti = ti;
//  pid.td = td;
//  pid.dt = dt;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//	  if(arm_state == 1){
//		  __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, 2000);
//	  }else{
//		  __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, 1500);
//	  }
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback (TIM_HandleTypeDef *htim){
	if(htim->Instance == TIM10){
//		__HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, servo_pwm);

		char logbuf[1024];
		FIRFilter_Update(&mav, emg_raw);
		pid_out = PID_Update(&pid, arm_pressure, fsr);

		if(pid_out > 2000){
			pid_out = 2000;
		}else if(pid_out < 1000){
			pid_out = 1000;
		}

		if(arm_state_manual == 1){
		  __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, pid_out);
		}else{
		  __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, 1000);
		}

//		emg_rawdiff = mav.out - emg_rawbfr;
		emg_rawdiff = emg_raw - emg_rawbfr;
//		emg_rawdiff = abs(emg_rawdiff);
		if(emg_rawdiff < 0){
			emg_rawdiff = emg_rawdiff * -1;
		}
		Signal_Buf_Update(&sig, emg_rawdiff);
		Signal_Energy_Calculate(&sig);

		if(sig.energy > arm_condition_thresh){
			if(prosthetic_statebfr == true){
				truth_counter_thresh++;
				if(truth_counter_thresh >= truth_counter_maxval){
					prosthetic_state = true;
					arm_state = 1;
					truth_counter_thresh = 0;
				}
			}
			prosthetic_statebfr = true;
		}else{
			if(prosthetic_statebfr == false){
				truth_counter_thresh++;
				if(truth_counter_thresh >= truth_counter_maxval){
					prosthetic_state = false;
					arm_state = 0;
					truth_counter_thresh = 0;
				}
			}
			prosthetic_statebfr = false;
		}

//		sprintf(logbuf, "%.2f,%d,%.2f,%.2f\r\n",emg_raw, fsr, arm_pressure, pid_out);
		sprintf(logbuf, "%d,%d,%.2f\r\n", arm_pressure,fsr, pid_out);
		CDC_Transmit_FS((uint8_t*)logbuf, strlen(logbuf));

//		emg_rawbfr = mav.out;
		emg_rawbfr = emg_raw;
	}
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	for(int i =0; i < 2; i++){
		adc[i] = adc_buffer[i];
	}
	emg_raw = adc[0];
	fsr = adc[1];
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if(GPIO_Pin == GPIO_PIN_0){
		calibration_counter++;
	}
}
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
