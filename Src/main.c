
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f3xx_hal.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include <math.h>
#include "my_lib\my_uart_lib.h"

#define my_uart_LIB_en 1

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_NVIC_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
#define MPU_6050_ADDRESS (0x68 << 1)
#define HMC5883L_ADDRESS (0x1e << 1)
#define MS5611_ADDRESS (0x77 << 1)
#define CNT_Register (*(volatile unsigned *)0x40001424)
#define PI 3.141592
double Accel_X, Accel_Y, Accel_Z, Temperature;
double Gyro_X, Gyro_Y, Gyro_Z;
double Magnet_X, Magnet_Y, Magnet_Z, MagAngle, Base_Angle, Heading;
double Base_Accel_X, Base_Accel_Y, Base_Accel_Z;
double Base_Gyro_X, Base_Gyro_Y, Base_Gyro_Z;
double Base_Mage_X, Base_Mage_Y, Base_Mage_Z;
double AccXangle, AccYangle, AccZangle;
double GyroXangle, GyroYangle, GyroZangle;
double CompXangle, CompYangle, CompZangle;
uint16_t SENS_t1, OFF_t1, TCS, TCO, TREF, TEMPSENS;
uint32_t D_PRESS, D_TEMPER;
int32_t d_T, TEMP, Pressure;
int64_t OFF, SENS, T2, OFF2, SENS2;
uint8_t cammand[3] = {0x48, 0x58, 0x00};
int temp[20];
int cnt = 0;
int p_flag = 1;
int num1 = 0;
int Register = 0;

void MPU_6050_init();
void Write_reg(unsigned char senser_Add, unsigned char Register_Add, unsigned char data);
short int Read_reg_8(unsigned char senser_Add, unsigned char Register_Add);
void Read_MPU6050_Data(unsigned char senser_Add, unsigned char Register_Add);
float Tempt_Senser_Read();


void Write_reg(unsigned char senser_Add, unsigned char Register_Add, unsigned char data)		//I2C 통신으로 센서의 레지스터에 데이터 입력
{
	unsigned char buffer[2] = {Register_Add, data};
	HAL_I2C_Master_Transmit(&hi2c1, senser_Add, buffer, 2, 100);
}

void MPU_6050_init()									//MPU_6050 초기 셋팅 합수
{
	Write_reg(MPU_6050_ADDRESS, 0x6b, 0x00);			//sleep mode disable
	Write_reg(MPU_6050_ADDRESS, 0x19, 0x00);			//sample rate setting	8khz / 1 = 8Khz
	Write_reg(MPU_6050_ADDRESS, 0x1a, 0x06);			//DLPF setting 5hz, 19ms, 5hz, 18.6ms   total sample rate = 1khz
	Write_reg(MPU_6050_ADDRESS, 0x1b, 0x00);			//gyroscope full scale	+- 250dps		131LSB
	Write_reg(MPU_6050_ADDRESS, 0x1c, 0x08);			//accelerometer full scale  +-4g	8192

	HAL_Delay(100);										//센서 안정화 딜레이 타임
}

void HMC5883L_init()									//HMC5883L 초기셋팅 함수
{

	Write_reg(MPU_6050_ADDRESS, 0x6a, 0x00);			//mpu_6050 master mode off
	Write_reg(MPU_6050_ADDRESS, 0x37, 0x02);			//mpu_6050 bypass mode on

	Write_reg(HMC5883L_ADDRESS, 0x00, 0x78);			//configuration register : Samples averaged = 8, Output rate = 75Hz, Measurement Mode = Normal
	Write_reg(HMC5883L_ADDRESS, 0x01, 0x40);			//configuration register : Sensor Field Range = +-1.9Ga, Gain = 820
	Write_reg(HMC5883L_ADDRESS, 0x02, 0x00);			//Mode register : Operating Mode = Continuous-Measurement Mode

	HAL_Delay(100);										//센서 안정화 딜레이 타임

	Write_reg(MPU_6050_ADDRESS, 0x37, 0x00);			//mpu_6050 bypass mode off

	Write_reg(MPU_6050_ADDRESS, 0x25, 0x9e);			//slave read,write set and slave addr set
	Write_reg(MPU_6050_ADDRESS, 0x26, 0x03);			//slave register addr set
	Write_reg(MPU_6050_ADDRESS, 0x27, 0x82);			//slave en, bytsw, dis, group, len set

	Write_reg(MPU_6050_ADDRESS, 0x28, 0x9e);			//slave read,write set and slave addr set
	Write_reg(MPU_6050_ADDRESS, 0x29, 0x05);			//slave register addr set
	Write_reg(MPU_6050_ADDRESS, 0x2a, 0x82);			//slave en, bytsw, dis, group, len set

	Write_reg(MPU_6050_ADDRESS, 0x2b, 0x9e);			//slave read,write set and slave addr set
	Write_reg(MPU_6050_ADDRESS, 0x2c, 0x07);			//slave register addr set
	Write_reg(MPU_6050_ADDRESS, 0x2d, 0x82);			//slave en, bytsw, dis, group, len set

	Write_reg(MPU_6050_ADDRESS, 0x6a, 0x20);			//mpu_6050 master mode on

	HAL_Delay(100);
}

void MS5611_init()										//MS5611 초기 셋팅 함수
{
	uint8_t command = 0x1e;
	uint8_t temp_R[6] = {0xa2, 0xa4, 0xa6, 0xa8, 0xaa, 0xac};
	uint8_t temp_D[2];
	uint16_t Data[6];
	unsigned short int calc_data;
	uint8_t *calcA = &calc_data, *calcB;
	calcB = calcA + 1;

	HAL_I2C_Master_Transmit(&hi2c1, MS5611_ADDRESS, &command, 1, 100);
	HAL_Delay(10);
	for(int i = 0 ; i < 6 ; i++){
		HAL_I2C_Master_Transmit(&hi2c1, MS5611_ADDRESS, &temp_R[i], 1, 100);
		HAL_I2C_Master_Receive(&hi2c1, (MS5611_ADDRESS | 0x01), temp_D, 2, 200);
		calc_data = temp_D[1];
		*calcB = temp_D[0];
		Data[i] = calc_data;
	}
	SENS_t1 = Data[0];
	OFF_t1 = Data[1];
	TCS = Data[2];
	TCO = Data[3];
	TREF = Data[4];
	TEMPSENS = Data[5];
	HAL_Delay(10);
}

short int Read_reg_8(unsigned char senser_Add, unsigned char Register_Add)
{
	unsigned char Tmp_buffer[1] = {Register_Add};

	  HAL_I2C_Master_Transmit(&hi2c1, senser_Add, Tmp_buffer, 1, 100);
	  HAL_Delay(10);
	  HAL_I2C_Master_Receive(&hi2c1, (senser_Add | 0x01), Tmp_buffer, 1, 200);
	  return Tmp_buffer[0];
}

void Read_MPU6050_Data(unsigned char senser_Add, unsigned char Register_Add)
{
	unsigned char Tmp_buffer[14] = {Register_Add};
	short int Out_data;
	short int temp_d[7];
	char *calcA = &Out_data;
	char *calcB;
	calcB = calcA + 1;

	HAL_I2C_Mem_Read(&hi2c1, senser_Add, Register_Add, I2C_MEMADD_SIZE_8BIT, &Tmp_buffer[0], 14, 100);			//함수 하나로 읽기 시퀸스 작동 시간값이 굉장히 작아짐
	//HAL_I2C_Master_Transmit(&hi2c1, senser_Add, Tmp_buffer, 1, 100);
	//HAL_Delay(10);
	//HAL_I2C_Master_Receive(&hi2c1, (senser_Add | 0x01), Tmp_buffer, 14, 200);				//burst read sequence

	for(int i = 0 ; i < 7 ; i++){
	*calcB = Tmp_buffer[(2*i)];
	*calcA = Tmp_buffer[(2*i) + 1];
	temp_d[i] = Out_data;
	}

	Accel_X = temp_d[0] ;
	Accel_Y = temp_d[1] ;
	Accel_Z = temp_d[2] ;
	Temperature = temp_d[3] / 340 + 36.53;
	Gyro_X = temp_d[4] / 131.0;
	Gyro_Y = temp_d[5] / 131.0;
	Gyro_Z = temp_d[6] / 131.0;
}

void Read_HMC5883L_Data(unsigned char senser_Add, unsigned char Register_Add)
{
	unsigned char Tmp_buffer[6] = {Register_Add};
	short int Out_data;
	short int temp_d[3];
	char *calcA = &Out_data;
	char *calcB;
	calcB = calcA + 1;

	HAL_I2C_Mem_Read(&hi2c1, senser_Add, Register_Add, I2C_MEMADD_SIZE_8BIT, &Tmp_buffer[0], 6, 100);
	//HAL_I2C_Master_Transmit(&hi2c1, senser_Add, Tmp_buffer, 1, 100);
	//HAL_Delay(10);
	//HAL_I2C_Master_Receive(&hi2c1, (senser_Add | 0x01), Tmp_buffer, 6, 200);

	for(int i = 0 ; i < 3 ; i++){
		*calcB = Tmp_buffer[(2*i)];
		*calcA = Tmp_buffer[(2*i) + 1];
		temp_d[i] = Out_data;
	}

	Magnet_X = temp_d[0] / 820.0;
	Magnet_Y = temp_d[2] / 820.0;
	Magnet_Z = temp_d[1] / 820.0;

}

void Read_MS5611_Data(int Type)
{

	uint8_t temp_A[4] = {0,};
	uint32_t Data= 0;
	char *calcA, *calcB, *calcC = &Data;
	calcB = calcC + 1;
	calcA = calcC + 2;

	switch(Type){

	case 1:
		//HAL_I2C_Master_Transmit(&hi2c1, MS5611_ADDRESS, &cammand[0], 1, 100);
		//HAL_Delay(10);
		HAL_I2C_Master_Transmit(&hi2c1, MS5611_ADDRESS, &cammand[2], 1, 100);
		HAL_I2C_Master_Receive(&hi2c1, (MS5611_ADDRESS | 0x01), temp_A, 4, 200);
		*calcC = temp_A[2];
		*calcB = temp_A[1];
		*calcA = temp_A[0];
		D_PRESS = Data;
		break;

	case -1:
		//HAL_I2C_Master_Transmit(&hi2c1, MS5611_ADDRESS, &cammand[1], 1, 100);
		//HAL_Delay(10);
		HAL_I2C_Master_Transmit(&hi2c1, MS5611_ADDRESS, &cammand[2], 1, 100);
		HAL_I2C_Master_Receive(&hi2c1, (MS5611_ADDRESS | 0x01), temp_A, 4, 200);
		*calcC = temp_A[2];
		*calcB = temp_A[1];
		*calcA = temp_A[0];
		D_TEMPER = Data;
		break;
	}
}

void Calc_TP()
{
	  d_T = D_TEMPER - TREF * pow(2, 8);
	  TEMP = 2000 + d_T * TEMPSENS / pow(2, 23);

	  OFF = OFF_t1 * pow(2, 16) + (TCO * d_T)/pow(2, 7);
	  SENS = SENS_t1 * pow(2, 15) + (TCS * d_T) / pow(2,8);

	  if(TEMP < 2000){
		  T2 = pow(d_T, 2) / pow(2, 31);
		  OFF2 = 5 * pow((TEMP - 2000), 2) / 2;
		  SENS2 = 5 * pow((TEMP - 2000), 2) / 4;
		  if(TEMP < -1500){
			  OFF2 = OFF2 + 7 * pow((TEMP + 1500), 2);
			  SENS2 = SENS2 + 11 * pow((TEMP + 1500) , 2) / 2;
		  }
	  }
	  else{
		  T2 = 0;
		  OFF2 = 0;
		  SENS2 = 0;
	  }

	  TEMP -= T2;
	  OFF -= OFF2;
	  SENS -= SENS2;

	  Pressure = (D_PRESS * SENS / pow(2,21) - OFF) / pow(2, 15);

}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)		//타이머 인터럽트 콜백함수
{
	  //static unsigned char num =0;
	  if(htim -> Instance == TIM7){		//구조체 변수
		  num1++;
	  }
}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */


  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM7_Init();
  MX_TIM2_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_TIM3_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */


  uart_init();				//usart 초기화 and 연결 확인 함수
  MPU_6050_init();			//MPU6050 초기화 함수
  HMC5883L_init();			//HMC5883L 초기설정 함수
  MS5611_init();			//MS5611 초기설정과 보정값 받기


  Read_MS5611_Data(1);
  Read_MS5611_Data(-1);

  d_T = D_TEMPER - TREF * pow(2, 8);
  TEMP = 2000 + d_T * TEMPSENS / pow(2, 23);

  OFF = OFF_t1 * pow(2, 16) + (TCO * d_T)/pow(2, 7);
  SENS = SENS_t1 * pow(2, 15) + (TCS * d_T) / pow(2,8);
  Pressure = (D_PRESS * SENS / pow(2,21) - OFF) / pow(2, 15);


  for(int i = 0; i < 20 ; i++)
  {
	  Read_HMC5883L_Data(MPU_6050_ADDRESS, 0x49);
	  Read_MPU6050_Data(MPU_6050_ADDRESS, 0x3b);

	  Base_Accel_X += Accel_X;
	  Base_Accel_Y += Accel_Y;
	  Base_Accel_Z += Accel_Z;

	  Base_Gyro_X += Gyro_X;
	  Base_Gyro_Y += Gyro_Y;
	  Base_Gyro_Z += Gyro_Z;

	  Base_Mage_X += Magnet_X;
	  Base_Mage_Y += Magnet_Y;
	  Base_Mage_Z += Magnet_Z;
  }

  Base_Accel_X /= 20;
  Base_Accel_Y /= 20;
  Base_Accel_Z /= 20;

  Base_Gyro_X /= 20;
  Base_Gyro_Y /= 20;
  Base_Gyro_Z /= 20;

  Base_Mage_X /= 20;
  Base_Mage_Y /= 20;
  Base_Mage_Z /= 20;

  Read_HMC5883L_Data(MPU_6050_ADDRESS, 0x49);
  Read_MPU6050_Data(MPU_6050_ADDRESS, 0x3b);

  Accel_X = Accel_X - Base_Accel_X;
  Accel_Y = Accel_Y - Base_Accel_Y;
  Accel_Z = Accel_Z + (8192 - Base_Accel_Z);

  AccXangle = atan2(Accel_Y, (sqrt((pow(Accel_X, 2) + pow(Accel_Z, 2))))) * 57.3;		//roll
  AccYangle = atan2(Accel_X, (sqrt((pow(Accel_Y, 2) + pow(Accel_Z, 2))))) * 57.3;		//pitch

  Heading = 0.0;
  MagAngle = 180 * (atan2((-1*Magnet_Y), Magnet_X) - 0.1408) /PI + 180;
  Base_Angle = Heading - MagAngle;

  GyroZangle = Heading;
  GyroXangle = AccXangle;
  GyroYangle = AccYangle;

  CompXangle = AccXangle;
  CompYangle = AccYangle;
  CompZangle = Heading;




  HAL_TIM_Base_Start_IT(&htim7);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  if(cnt == 0){
		  if(p_flag == 1) HAL_I2C_Master_Transmit(&hi2c1, MS5611_ADDRESS, &cammand[0], 1, 100);
		  else HAL_I2C_Master_Transmit(&hi2c1, MS5611_ADDRESS, &cammand[1], 1, 100);
	  }

	  cnt++;
	  if(cnt >= 10){
		  Read_MS5611_Data(p_flag);
		  if(p_flag == -1) Calc_TP();
		  p_flag = -p_flag;
		  cnt = 0;

	  }

	  Read_MPU6050_Data(MPU_6050_ADDRESS, 0x3b);
	  Read_HMC5883L_Data(MPU_6050_ADDRESS, 0x49);

	  Accel_X = Accel_X - Base_Accel_X;
	  Accel_Y = Accel_Y - Base_Accel_Y;
	  Accel_Z = Accel_Z + (8192 - Base_Accel_Z);

	  AccXangle = (atan2(Accel_Y, (sqrt((pow(Accel_X, 2) + pow(Accel_Z, 2))))) * 57.3);
	  AccYangle = (atan2(Accel_X, (sqrt((pow(Accel_Y, 2) + pow(Accel_Z, 2))))) * 57.3);
	  MagAngle = 180 * (atan2((-1*Magnet_Y), Magnet_X) - 0.1408) /PI + 180;
	  Heading = Base_Angle + MagAngle;

	  Register = CNT_Register;
	  //double dt = Register / 10000.0;
	  double ALPHA = 1/(1 + (double)CNT_Register /10000);

	  Gyro_X = Gyro_X - Base_Gyro_X;
	  Gyro_Y = Gyro_Y - Base_Gyro_Y;
	  Gyro_Z = Gyro_Z - Base_Gyro_Z;
/*

	  GyroXangle += Gyro_X*((double)CNT_Register / 10000);
	  GyroYangle += Gyro_Y*((double)CNT_Register / 10000);
	  GyroZangle += Gyro_Z*((double)CNT_Register / 10000);
*/

	  //CompXangle = (ALPHA*(CompXangle + (Gyro_X * (double)(CNT_Register / 10000)))) + ((1 - ALPHA)*AccXangle);			//수행시간 AHLPHA 값을 이용해서 자이로 데이터의 가중치가 실시간 조정
	  //CompYangle = (ALPHA*(CompYangle + (Gyro_Y * (double)(CNT_Register / 10000)))) + ((1 - ALPHA)*AccYangle);
	  CompZangle = (ALPHA*(CompZangle + (Gyro_Z * (double)(CNT_Register / 10000)))) + ((1 - ALPHA)*Heading);


	  CompXangle = (0.99*(CompXangle + (Gyro_X * (double)(CNT_Register / 10000)))) + (0.01*AccXangle);			//상수 0.93과 0.07은 가중치로 +1의 값을 서로 나누어 가짐
	  CompYangle = (0.99*(CompYangle + (Gyro_Y * (double)(CNT_Register / 10000)))) + (0.01*AccYangle);
	  //CompZangle = (0.99*(CompZangle + (Gyro_Z * (double)(CNT_Register / 10000)))) + (0.01*Heading);

	  CNT_Register = 0;


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

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
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
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* EXTI15_10_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
  /* EXTI9_5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
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
