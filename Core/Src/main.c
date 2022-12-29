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
#include "pid.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define SIGNAL_SAMPLES_LENGTH 50
//#define EMG_OFFSET 1700
/*#define EMG_OFFSET 1880*/

typedef struct{
	float buf[SIGNAL_SAMPLES_LENGTH];
	uint8_t bufIndex;

	double energy;
	float rms;
}SignalFeature_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint32_t adc[3], adc_buffer[3];
int fsr[2];
int sum_fsr;
float sum_fsr_to_voltage;
int arm_pressure = 2000;
uint16_t emg_raw;
int emg_rmv_offset;
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

uint8_t truth_counter_maxval = 50;
uint16_t truth_counter_maxval_arm_closed = 300;
uint16_t truth_counter_thresh;
bool prosthetic_state = false;
bool prosthetic_statebfr;
int arm_condition_thresh = 300;

float kp = 0.4;
float ti = 0.05;
float td = 0.001;
float dt = 0.0005;
float pid_out;
float pid_out_clamped;

SignalFeature_t sig;
SignalFeature_t sig_rms;
PID_t pid;

int servo_pwm = 1500;

char usbd_buf_recv[128];
char *array[2];
int parsedBuf[2];
uint8_t device_status = 0;
int emg_offset = 1700;
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

float Signal_RMS_Calculate(SignalFeature_t *signal){
	float energy = Signal_Energy_Calculate(signal);
	signal->rms = 0.0f;
	signal->rms = sqrt(energy / SIGNAL_SAMPLES_LENGTH);
	return signal->rms;
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
  HAL_ADC_Start_DMA(&hadc1, adc_buffer, 3);

  Signal_Buf_Init(&sig);
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
		char logbuf[512];

		uint8_t i = 0;
		char *p = strtok(usbd_buf_recv, ",");
//		char *array[2];
//		array[0] = malloc(10);
//		array[1] = malloc(10);

		while(p != NULL){
			array[i++] = p;
			p = strtok(NULL, ",");
		}

		parsedBuf[0] = atoi(array[0]);
		parsedBuf[1] = atoi(array[1]);


		if(parsedBuf[0] == 1){
			device_status = parsedBuf[1];
		}else if(parsedBuf[0] == 2){
			emg_offset = parsedBuf[1];
		}else if(parsedBuf[0] == 3){
			arm_condition_thresh = parsedBuf[1];
		}

		memset(usbd_buf_recv, NULL, sizeof(usbd_buf_recv));
		sum_fsr = fsr[0] + fsr[1];
		sum_fsr_to_voltage = (float)sum_fsr / 4096 * 3.3;
		pid_out = PID_Update(&pid, arm_pressure, sum_fsr) + 1500;
		pid_out_clamped = pid_out;

		if(pid_out > 2000){
			pid_out_clamped = 2000;
		}else if(pid_out < 1000){
			pid_out_clamped = 1000;
		}

		if(arm_state == 1 && device_status == 1){
			__HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, pid_out_clamped);
		}else if(arm_state == 0 && device_status == 1){
			__HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, 1000);
		}else{
			__HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, 1000);
		}

//		emg_rawdiff = mav.out - emg_rawbfr;
		emg_rmv_offset = emg_raw - emg_offset;
		emg_rawdiff = emg_raw - emg_rawbfr;
//		emg_rawdiff = abs(emg_rawdiff);
//		if(emg_rawdiff < 0){
//			emg_rawdiff = emg_rawdiff * -1;
//		}
//
		Signal_Buf_Update(&sig, emg_rmv_offset);
		Signal_RMS_Calculate(&sig);

		if(sig.rms > arm_condition_thresh){
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
				if(truth_counter_thresh >= truth_counter_maxval_arm_closed){
					prosthetic_state = false;
					arm_state = 0;
					truth_counter_thresh = 0;
				}
			}
			prosthetic_statebfr = false;
		}

//		sprintf(logbuf, "%.2f,%d,%.2f,%.2f,%.2f,\r\n",sig.rms, arm_condition_thresh, sum_fsr_to_voltage, pid_out, pid_out_clamped);
		sprintf(logbuf, "%.2f,%d,%.2f,%d,%d,\r\n",sig.rms, arm_condition_thresh, sum_fsr_to_voltage, sum_fsr, arm_pressure);
//		sprintf(logbuf, "%d,%.2f,%.2f,%d,%.2f\r\n", emg_raw, sig.rms, pid_out, sum_fsr, pid_out_clamped);
		CDC_Transmit_FS((uint8_t*)logbuf, strlen(logbuf));

		emg_rawbfr = emg_raw;
	}
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	for(int i =0; i < 3; i++){
		adc[i] = adc_buffer[i];
	}
	emg_raw = adc[0];
	fsr[0] = adc[1];
	fsr[1] = adc[2];

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
