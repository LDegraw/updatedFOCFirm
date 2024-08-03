/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "motor.h"
#include "stm32g4xx.h"
#include "stm32g431xx.h"
#include "stm32g4xx_hal.h"
#include "lsm6dso.h"
#include "lsm6dso_reg.h"
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ADC_BUF_LENGTH 4096
#define TIMCLOCK   150000000
#define PRESCALAR  300
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
DMA_HandleTypeDef hdma_adc1;

COMP_HandleTypeDef hcomp2;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim16;
TIM_HandleTypeDef htim17;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
float gravity = 9.807f;
uint16_t adc1_val[2];
uint16_t adc_val2;
uint8_t rxBuffer;
uint8_t adcDataReady;
uint8_t uartDataReady;
uint32_t tim1Val1 = 0;
uint32_t tim1Val2 = 0;
uint32_t Difference1 = 0;
int isFirstCap1 = 0;
float frequency1 = 0;
uint32_t tim2Val1 = 0;
uint32_t tim2Val2 = 0;
uint32_t Difference2 = 0;
int isFirstCap2 = 0;
float frequency2 = 0;
uint32_t tim3Val1 = 0;
uint32_t tim3Val2 = 0;
uint32_t Difference3 = 0;
int isFirstCap3 = 0;
float frequency3 = 0;
float combinedSpeed;
double angle;
float adc1Avg_U;
float adc1Avg_W;
float yAcceleration;
float xAcceleration;
float zAcceleration;
float temp;
uint32_t xRotation;
uint32_t yRotation;
uint32_t zRotation;

float avgYAcceleration;
float adc2Avg;
motor_t motor;

gpio_Pin Hall_1 = { .gpioGroup = GPIOC, .gpioPin = GPIO_PIN_11 };       //EXTI11
gpio_Pin Hall_2 = { .gpioGroup = GPIOA, .gpioPin = GPIO_PIN_12 };        //EXTI12
gpio_Pin Hall_3 = { .gpioGroup = GPIOC, .gpioPin = GPIO_PIN_4 };       	//EXTI4

gpio_Pin fault = { .gpioGroup = GPIOB, .gpioPin = GPIO_PIN_1 };			//FAULT

gpio_Pin txen = { .gpioGroup = GPIOB, .gpioPin = GPIO_PIN_8 };          //TXEN

gpio_Pin imuInt = { .gpioGroup = GPIOC, .gpioPin = GPIO_PIN_13 };
gpio_Pin imuCS = { .gpioGroup = GPIOA, .gpioPin = GPIO_PIN_4 };				//IMU INT
double newAngle;
bool doDMArx;
gpio_Pin ena = { .gpioGroup = GPIOB, .gpioPin = GPIO_PIN_15 };            	//motor ENA
gpio_Pin enb = { .gpioGroup = GPIOB, .gpioPin = GPIO_PIN_13 };           	//motor ENB
gpio_Pin enc = { .gpioGroup = GPIOC, .gpioPin = GPIO_PIN_6 };            	//motor ENC

gpio_Pin motor_Sleep = { .gpioGroup = GPIOA, .gpioPin = GPIO_PIN_2 };          //motor SLEEP

gpio_Pin proxi = { .gpioGroup = GPIOB, .gpioPin = GPIO_PIN_10 };    	//PROXY
uint8_t rx_buff[5];
uint8_t isDA[] = {0xDA, 0XBA, 0xD0, 0x00};
uint8_t retDA[] = {0x00, 0xD0, 0xBA, 0xDA};
uint8_t address= 0x0F;


int startState;
int nextState;

uint8_t convCompleted;

uint32_t timVal1 = 0;
uint32_t timVal2 = 0;
uint32_t Difference = 0;
int isFirstCap = 0;
float frequency = 0;
uint8_t lastHallState;
int hallCount;
int revolutions;
int timerCount;
int toggleState;
uint32_t node = 0x69;

LSM6DSO_Object_t acc;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM17_Init(void);
static void MX_TIM16_Init(void);
static void MX_TIM7_Init(void);
static void MX_ADC2_Init(void);
static void MX_COMP2_Init(void);
static void MX_SPI3_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
void RGB_Init(void);
void RGB_setIntensity(uint8_t red, uint8_t green, uint8_t blue);
void accRead(uint8_t add, uint8_t* data);
void readAccel(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	int timeSteps;
	timeSteps = __HAL_TIM_GET_COUNTER(&htim17);
	float refClock = TIMCLOCK/(PRESCALAR);
	motor.speed = (60.0f)/((float)timeSteps/refClock);
	motor.hallCount = timeSteps;
	readHalls(&motor);
	htim17.Instance->CNT = 0;

}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim == &htim17){
		motor.speed = 0;
		motor.avg_speed = 0;
		motor.hallspeed = 0;
		htim17.Instance->CNT = 0;
	}
	if(htim == &htim16){

		//MOTOR_FOCtask(&motor);
		//MOTOR_task(&motor);
		htim->Instance->CNT = 0;
	}
	if(htim == &htim7){
		motor.torqueLevel = 15.0f;

		htim->Instance->CNT = 0;
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
    if(1){  // RX is complete

        // SYNC
        if(rx_buff[0] == isDA[0] && rx_buff[1] == isDA[1] && rx_buff[2] == isDA[2] && rx_buff[3] == isDA[3]){
        	RGB_setIntensity(186, 26, 133); //186, 26, 133
        	HAL_GPIO_WritePin(txen.gpioGroup, txen.gpioPin, 1); // setting txen high
        	HAL_UART_Transmit(&huart1, retDA, 4, 100);
        	HAL_GPIO_WritePin(txen.gpioGroup, txen.gpioPin, 0); // setting txen low
        }

    }
}
void readAccel(void){
	address |= 0x80;
	HAL_GPIO_WritePin(imuCS.gpioGroup, imuCS.gpioPin, 0); // setting CS LOW
	HAL_SPI_Transmit(&hspi1, &address, 1, 100);
	HAL_SPI_Receive(&hspi1, &rxBuffer, 1, 100);
	HAL_GPIO_WritePin(imuCS.gpioGroup, imuCS.gpioPin, 1);// setting CS HIGH
	HAL_GPIO_WritePin(txen.gpioGroup, txen.gpioPin, 1); // setting txen high
	HAL_UART_Transmit(&huart1, &rxBuffer, 1, 100);
	HAL_GPIO_WritePin(txen.gpioGroup, txen.gpioPin, 0); // setting txen low
}

void xRot(void){
	uint8_t x_high = 0x22;
	uint8_t x_low =  0x23;
	uint8_t highBuff;
	uint8_t lowBuff;

	accRead(x_high, &highBuff);
	accRead(x_low, &lowBuff);

	xRotation = (highBuff << 8)|lowBuff;
}

void yRot(void){
	uint8_t y_high = 0x24;
	uint8_t y_low =  0x25;
	uint8_t highBuff;
	uint8_t lowBuff;

	accRead(y_high, &highBuff);
	accRead(y_low, &lowBuff);

	yRotation = (highBuff << 8)|lowBuff;
}

void zRot(void){
	uint8_t z_high = 0x26;
	uint8_t z_low =  0x27;
	uint8_t highBuff;
	uint8_t lowBuff;

	accRead(z_high, &highBuff);
	accRead(z_low, &lowBuff);

	zRotation = (highBuff << 8)|lowBuff;
}

void accelerationY(void){
	uint8_t y_high = 0x2B;
	uint8_t y_low =  0x2A;
	uint8_t highBuff;
	uint8_t lowBuff;

	accRead(y_high, &highBuff);
	accRead(y_low, &lowBuff);

	yAcceleration = gravity * ((float)((highBuff << 8)|lowBuff) / 65536.0f);
}

void accelerationX(void){
	uint8_t x_high = 0x29;
	uint8_t x_low =  0x28;
	uint8_t highBuff;
	uint8_t lowBuff;

	//accRead(y_high, highBuff);
	accRead(x_high, &highBuff);
	accRead(x_low, &lowBuff);

	xAcceleration = gravity *((float)((highBuff << 8)|lowBuff) / 65536.0f);
}
void accelerationZ(void){
	uint8_t z_high = 0x2D;
	uint8_t z_low =  0x2C;
	uint8_t highBuff;
	uint8_t lowBuff;

	accRead(z_high, &highBuff);
	accRead(z_low, &lowBuff);

	zAcceleration = gravity*((float)((highBuff << 8)|lowBuff) / 65536.0f);
}

void tempMeasure(void){
	uint8_t temp_high = 0x20;
	uint8_t temp_low =  0x21;
	uint8_t highBuff;
	uint8_t lowBuff;

	accRead(temp_high, &highBuff);
	accRead(temp_low, &lowBuff);

	temp = gravity * ((float)((highBuff << 8)|lowBuff) / 65536.0f);
}

// Function to calculate filter coefficients for higher order Butterworth filter
void butterworth_lowpass_coefficients(int order, float cutoff_frequency, float sampling_rate, float* a, float* b) {
    int n;
    float wc = tanf(cutoff_frequency * 3.1415926f / sampling_rate);
    float wc2 = wc * wc;

    for (n = 0; n < order; ++n) {
        float theta = (float)(2 * n + 1) * 3.1415926f / (2.0f * order);
        float sn = sinf(theta);
        float cs = cosf(theta);
        float beta = 0.5f * ((1.0f - sn * wc) / (1.0f + sn * wc));
        float gamma = (0.5f + beta) * cs;
        float alpha = 0.5f * (0.5f + beta - gamma);

        b[3 * n + 0] = alpha;
        b[3 * n + 1] = 2.0f * alpha;
        b[3 * n + 2] = alpha;

        a[3 * n + 0] = 1.0f;
        a[3 * n + 1] = -2.0f * gamma;
        a[3 * n + 2] = 2.0f * beta;
    }
}

// Function to apply the Butterworth filter
void butterworth_lowpass_filter(float* data, float* output, int length, int order, float* a, float* b) {
    float x1[order], x2[order];
    float y1[order], y2[order];

    for (int i = 0; i < order; ++i) {
        x1[i] = x2[i] = y1[i] = y2[i] = 0.0f;
    }

    for (int i = 0; i < length; ++i) {
        float x0 = data[i];
        float y0 = 0.0f;

        for (int j = 0; j < order; ++j) {
            y0 += (b[3 * j + 0] * x0 + b[3 * j + 1] * x1[j] + b[3 * j + 2] * x2[j] - a[3 * j + 1] * y1[j] - a[3 * j + 2] * y2[j]) / a[3 * j + 0];

            x2[j] = x1[j];
            x1[j] = x0;
            y2[j] = y1[j];
            y1[j] = y0;
        }

        output[i] = y0;
    }
}
void accWrite(uint8_t add, uint8_t *data){
	//read
	uint8_t buff;
	//write
	buff = add & 0b01111111;

	HAL_GPIO_WritePin(imuCS.gpioGroup, imuCS.gpioPin, 0); // setting CS LOW
	HAL_SPI_Transmit(&hspi1, &buff, 1, 100);
	HAL_SPI_Transmit(&hspi1, data, 1, 100);
	HAL_GPIO_WritePin(imuCS.gpioGroup, imuCS.gpioPin, 1);// setting CS HIGH
	return;
}
void accRead(uint8_t add, uint8_t* data){
	uint8_t buff;
	buff = add | 0x80;

	HAL_GPIO_WritePin(imuCS.gpioGroup, imuCS.gpioPin, 0); // setting CS LOW
	HAL_SPI_Transmit(&hspi1, &buff, 1, 100);
	HAL_SPI_Receive(&hspi1, data, 1, 100);
	HAL_GPIO_WritePin(imuCS.gpioGroup, imuCS.gpioPin, 1);// setting CS HIGH
	return;
}

void RGB_setIntensity(uint8_t red, uint8_t green, uint8_t blue){
  if(red > 255){
    red = 255;
  }
  else if(red < 0){
    red = 0;
  }
  if(blue > 255){
    blue = 255;
  }
  else if(blue < 0){
    blue = 0;
  }
  if(green > 255){
    green = 255;
  }
  else if(green < 0){
    green = 0;
  }
  htim2.Instance->CCR1 = red;
  htim2.Instance->CCR2 = green;
  htim2.Instance->CCR3 = blue;
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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM17_Init();
  MX_TIM16_Init();
  MX_TIM7_Init();
  MX_ADC2_Init();
  MX_COMP2_Init();
  MX_SPI3_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  acc.IO.BusType = 4;
  acc.IO.Address = address;
  acc.IO.WriteReg = &accWrite;
  acc.IO.ReadReg = &accRead;

  // set up motor struct
  motor.pwm = &htim1;
  motor.adc = &hadc1;
  motor.encoder = &hspi1;
  // Enable pins
  motor.torqueLevel = 50.0f;

  motor.enablePins[0] = ena;
  motor.enablePins[1] = enb;
  motor.enablePins[2] = enc;
  motor.motorSleep = motor_Sleep;

  motor.hallPins[0] = Hall_1;
  motor.hallPins[1] = Hall_2;
  motor.hallPins[2] = Hall_3;
  motor.dutyCycle = 0.2;
  motor.speedTim = &htim17;
  motor.dir = 1;        // make go forward

  uint8_t dataBuffer;
  uint8_t changeState;

  //accelerometer Control Reg
  accRead(0x10, &dataBuffer);

  changeState = dataBuffer | 0b10100000;
  accWrite(0x10, &changeState);

  //GyroControl Reg
  accRead(0x11, &dataBuffer);

  changeState = dataBuffer | 0b10100000;
  accWrite(0x11, &changeState);

  //
  accRead(0x13, &dataBuffer);

  changeState = dataBuffer | 0b10101100;
  accWrite(0x13, &changeState);

  MOTOR_init(&motor);
  int order = 4; // 2nd order Butterworth filter
  float cutoff_frequency = 5.0f; // Desired cutoff frequency in Hz
  float sampling_rate = 500.0f; // Sampling rate in Hz
  int length = 100; // Length of the data array

  // Example input data (replace with your actual ADC data)
  float adcData[100];
  float filteredData[100];
  float a[3], b[3];

  toggleState = 0;
  double kp = 0.000000215f;
  double kd = 0.00000000205f;
  double ki = 0.0000000288f;
  double lastDuty = 0;
  double lastSpeedErr = 0;
  double speedErr = 0;
  double totalErr = 0;
  int samples = 0;
  int count = 0;
  double lastSpeed;
  uint16_t encoderData;
  //SEGGER_RTT_Init();
  float rollAngle;
  float pitchAngle;
  float yawAngle;
  char * adcString;

  float datatest;
  SEGGER_RTT_Init();
  HAL_COMP_Start(&hcomp2);
  HAL_TIM_Base_Start_IT(&htim17);
  HAL_TIM_Base_Start(&htim17);
  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t *) motor.adcData, 3);
  // setting motor 1
  int x = 0 ;
  float offsetU;
  float offsetV;
  float offsetW;

  while (x < 1000){
	  offsetU += (float)motor.adcData[0];
  	  offsetV += (float)motor.adcData[1];
  	  offsetW += (float)motor.adcData[2];
  	  x++;
  	  }

  motor.offset[0] = round(offsetU/1000.0f);
  motor.offset[1] = round(offsetV/1000.0f);
  motor.offset[2] = round(offsetW/1000.0f);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  motor.hallCount = htim17.Instance->CNT;
	  readHalls(&motor);
	  get_Current(&motor);
	  MOTOR_FOCtask(&motor);
	    speedErr = (float)(30.0f - motor.avg_speed);
	    motor.dutyCycle = motor.dutyCycle + (float)(kp *speedErr) + (float)(kd * (speedErr - lastSpeedErr)) + (float)(ki * totalErr);
	    if(motor.dutyCycle > 0.5f){
	    	motor.dutyCycle = 0.5f;
		}
		else if(motor.dutyCycle < 0.000f){
			motor.dutyCycle = 0.000f;
		}
  	  }
	lastSpeedErr = speedErr;
	lastDuty = motor.dutyCycle;
	totalErr += speedErr;
	lastSpeed = motor.hallspeed;
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 75;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
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

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.GainCompensation = 0;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 3;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T1_CC2;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_6CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_14;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.GainCompensation = 0;
  hadc2.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.NbrOfConversion = 2;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc2.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief COMP2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_COMP2_Init(void)
{

  /* USER CODE BEGIN COMP2_Init 0 */

  /* USER CODE END COMP2_Init 0 */

  /* USER CODE BEGIN COMP2_Init 1 */

  /* USER CODE END COMP2_Init 1 */
  hcomp2.Instance = COMP2;
  hcomp2.Init.InputPlus = COMP_INPUT_PLUS_IO1;
  hcomp2.Init.InputMinus = COMP_INPUT_MINUS_VREFINT;
  hcomp2.Init.OutputPol = COMP_OUTPUTPOL_NONINVERTED;
  hcomp2.Init.Hysteresis = COMP_HYSTERESIS_NONE;
  hcomp2.Init.BlankingSrce = COMP_BLANKINGSRC_NONE;
  hcomp2.Init.TriggerMode = COMP_TRIGGERMODE_NONE;
  if (HAL_COMP_Init(&hcomp2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN COMP2_Init 2 */

  /* USER CODE END COMP2_Init 2 */

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
  hspi1.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
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
  hspi3.Init.Direction = SPI_DIRECTION_2LINES_RXONLY;
  hspi3.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 7;
  hspi3.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

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
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 2;
  htim1.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
  htim1.Init.Period = 765;
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
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC2REF;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
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
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 16;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 255;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 4;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 2556;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 2;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 10000;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */

}

/**
  * @brief TIM17 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM17_Init(void)
{

  /* USER CODE BEGIN TIM17_Init 0 */

  /* USER CODE END TIM17_Init 0 */

  /* USER CODE BEGIN TIM17_Init 1 */

  /* USER CODE END TIM17_Init 1 */
  htim17.Instance = TIM17;
  htim17.Init.Prescaler = 300;
  htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim17.Init.Period = 65535;
  htim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim17.Init.RepetitionCounter = 0;
  htim17.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim17) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM17_Init 2 */

  /* USER CODE END TIM17_Init 2 */

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
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13|GPIO_PIN_15|GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 PC4 PC11 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_4|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB1 PB10 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB13 PB15 PB8 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_15|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PC6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

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
