/*
 * my_GY_86_lib.h
 *
 *  Created on: 2019. 1. 22.
 *      Author: ragalia
 */

#ifndef MY_LIB_MY_GY_86_LIB_H_
#define MY_LIB_MY_GY_86_LIB_H_


#ifndef  my_gy86_LIB_en

#include <math.h>

//센서 어드레스와 데이터 저장용 변수 선언
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
int cnt = 0;
int p_flag = 1;
int num1 = 0;
int Register = 0;
uint8_t cammand[3] = { 0x48, 0x58, 0x00 };						//MS5611 ADC cammand
uint16_t SENS_t1, OFF_t1, TCS, TCO, TREF, TEMPSENS;
uint32_t D_PRESS, D_TEMPER;
int32_t d_T, TEMP, Pressure;
int64_t OFF, SENS, T2, OFF2, SENS2;

short int Read_reg_8(unsigned char senser_Add, unsigned char Register_Add);
void Write_reg(unsigned char senser_Add, unsigned char Register_Add, unsigned char data);
void Calc_TP();
void MPU_6050_init();
void HMC5883L_init();
void MS5611_init();
void Read_MPU6050_Data(unsigned char senser_Add, unsigned char Register_Add);
void Read_HMC5883L_Data(unsigned char senser_Add, unsigned char Register_Add);
void Read_MS5611_Data(int Type);
void Total_Calibration();
void While_Loop();

#endif




short int Read_reg_8(unsigned char senser_Add, unsigned char Register_Add)						//I2C 통신으로 레지스터 데이터 확인용 함수
{
	unsigned char Tmp_buffer[1] = { Register_Add };

	HAL_I2C_Master_Transmit(&hi2c1, senser_Add, Tmp_buffer, 1, 100);
	HAL_Delay(10);
	HAL_I2C_Master_Receive(&hi2c1, (senser_Add | 0x01), Tmp_buffer, 1, 200);
	return Tmp_buffer[0];
}


void Write_reg(unsigned char senser_Add, unsigned char Register_Add, unsigned char data)		//I2C 통신으로 센서의 레지스터에 데이터 입력용 함수
{
	unsigned char buffer[2] = { Register_Add, data };
	HAL_I2C_Master_Transmit(&hi2c1, senser_Add, buffer, 2, 100);
}


void Calc_TP()																					//공정 캘리용 데이터를 참조로 기압과 온도 원시데이터를 계산하는 함수
{
	d_T = D_TEMPER - TREF * pow(2, 8);
	TEMP = 2000 + d_T * TEMPSENS / pow(2, 23);

	OFF = OFF_t1 * pow(2, 16) + (TCO * d_T) / pow(2, 7);
	SENS = SENS_t1 * pow(2, 15) + (TCS * d_T) / pow(2, 8);


	//온도 데이터에 따른 두번째 보정용 데이터 계산 
	if (TEMP < 2000) {
		T2 = pow(d_T, 2) / pow(2, 31);
		OFF2 = 5 * pow((TEMP - 2000), 2) / 2;
		SENS2 = 5 * pow((TEMP - 2000), 2) / 4;
		if (TEMP < -1500) {
			OFF2 = OFF2 + 7 * pow((TEMP + 1500), 2);
			SENS2 = SENS2 + 11 * pow((TEMP + 1500), 2) / 2;
		}
	}
	else {
		T2 = 0;
		OFF2 = 0;
		SENS2 = 0;
	}

	TEMP -= T2;
	OFF -= OFF2;
	SENS -= SENS2;

	Pressure = (D_PRESS * SENS / pow(2, 21) - OFF) / pow(2, 15);

}																			


void MPU_6050_init()									//MPU_6050 초기 레지스터 셋팅 함수
{
	Write_reg(MPU_6050_ADDRESS, 0x6b, 0x00);			//sleep mode disable
	Write_reg(MPU_6050_ADDRESS, 0x19, 0x00);			//sample rate setting	8khz / 1 = 8Khz
	Write_reg(MPU_6050_ADDRESS, 0x1a, 0x06);			//DLPF setting 5hz, 19ms, 5hz, 18.6ms   total sample rate = 1khz
	Write_reg(MPU_6050_ADDRESS, 0x1b, 0x00);			//gyroscope full scale	+- 250dps		131LSB
	Write_reg(MPU_6050_ADDRESS, 0x1c, 0x08);			//accelerometer full scale  +-4g	8192

	HAL_Delay(100);										//센서 안정화 딜레이 타임
}


void HMC5883L_init()									//HMC5883L 초기레지스터와 MPU6050 마스터 모드 셋팅
{

	Write_reg(MPU_6050_ADDRESS, 0x6a, 0x00);			//mpu_6050 master mode off
	Write_reg(MPU_6050_ADDRESS, 0x37, 0x02);			//mpu_6050 bypass mode on

	Write_reg(HMC5883L_ADDRESS, 0x00, 0x78);			//configuration register : Samples averaged = 8, Output rate = 75Hz, Measurement Mode = Normal
	Write_reg(HMC5883L_ADDRESS, 0x01, 0x40);			//configuration register : Sensor Field Range = +-1.9Ga, Gain = 820
	Write_reg(HMC5883L_ADDRESS, 0x02, 0x00);			//Mode register : Operating Mode = Continuous-Measurement Mode

	Write_reg(MPU_6050_ADDRESS, 0x37, 0x00);			//mpu_6050 bypass mode off

	HAL_Delay(100);										//HMC5883 센서 안정화 딜레이 타임

	//magdata_X slave setting
	Write_reg(MPU_6050_ADDRESS, 0x25, 0x9e);			//slave read,write set and slave addr set
	Write_reg(MPU_6050_ADDRESS, 0x26, 0x03);			//slave register addr set
	Write_reg(MPU_6050_ADDRESS, 0x27, 0x82);			//slave en, bytsw, dis, group, len set

	//magdata_Z slave setting
	Write_reg(MPU_6050_ADDRESS, 0x28, 0x9e);			//slave read,write set and slave addr set
	Write_reg(MPU_6050_ADDRESS, 0x29, 0x05);			//slave register addr set
	Write_reg(MPU_6050_ADDRESS, 0x2a, 0x82);			//slave en, bytsw, dis, group, len set

	//magdata_Y slave setting
	Write_reg(MPU_6050_ADDRESS, 0x2b, 0x9e);			//slave read,write set and slave addr set
	Write_reg(MPU_6050_ADDRESS, 0x2c, 0x07);			//slave register addr set
	Write_reg(MPU_6050_ADDRESS, 0x2d, 0x82);			//slave en, bytsw, dis, group, len set

	Write_reg(MPU_6050_ADDRESS, 0x6a, 0x20);			//mpu_6050 master mode on

	HAL_Delay(100);										//MPU6050 센서 안정화 딜레이 타임
}


void MS5611_init()													//MS5611 보정용 공정 셋팅 데이터 함수
{
	uint8_t command = 0x1e;
	uint8_t temp_R[6] = { 0xa2, 0xa4, 0xa6, 0xa8, 0xaa, 0xac };		//calibration data register 
	uint8_t temp_D[2];
	uint16_t Data[6];
	unsigned short int calc_data;									//16bit unsigned int value pointer calc
	uint8_t *calcA = &calc_data, *calcB;
	calcB = calcA + 1;

	HAL_I2C_Master_Transmit(&hi2c1, MS5611_ADDRESS, &command, 1, 100);
	HAL_Delay(10);
	for (int i = 0; i < 6; i++) {
		HAL_I2C_Master_Transmit(&hi2c1, MS5611_ADDRESS, &temp_R[i], 1, 100);
		HAL_I2C_Master_Receive(&hi2c1, (MS5611_ADDRESS | 0x01), temp_D, 2, 200);
		calc_data = temp_D[1];
		*calcB = temp_D[0];
		Data[i] = calc_data;
	}
	//calibration data
	SENS_t1 = Data[0];
	OFF_t1 = Data[1];
	TCS = Data[2];
	TCO = Data[3];
	TREF = Data[4];
	TEMPSENS = Data[5];
	HAL_Delay(10);
}


void Read_MPU6050_Data(unsigned char senser_Add, unsigned char Register_Add)					//MPU6050 센서 데이터 추출 함수
{
	unsigned char Tmp_buffer[14] = { Register_Add };
	short int Out_data;
	short int temp_d[7];
	char *calcA = &Out_data;
	char *calcB;
	calcB = calcA + 1;

	HAL_I2C_Mem_Read(&hi2c1, senser_Add, Register_Add, I2C_MEMADD_SIZE_8BIT, &Tmp_buffer[0], 14, 100);		//함수 하나로 읽기 시퀸스 작동 시간값이 굉장히 작아짐
	//HAL_I2C_Master_Transmit(&hi2c1, senser_Add, Tmp_buffer, 1, 100);											
	//HAL_Delay(10);
	//HAL_I2C_Master_Receive(&hi2c1, (senser_Add | 0x01), Tmp_buffer, 14, 200);								//burst read sequence

	for (int i = 0; i < 7; i++) {
		*calcB = Tmp_buffer[(2 * i)];
		*calcA = Tmp_buffer[(2 * i) + 1];
		temp_d[i] = Out_data;
	}

	//Accel X,Y,Z  and Gyro X,Y,Z data save
	Accel_X = temp_d[0];
	Accel_Y = temp_d[1];
	Accel_Z = temp_d[2];
	Temperature = temp_d[3] / 340 + 36.53;
	Gyro_X = temp_d[4] / 131.0;
	Gyro_Y = temp_d[5] / 131.0;
	Gyro_Z = temp_d[6] / 131.0;
}


void Read_HMC5883L_Data(unsigned char senser_Add, unsigned char Register_Add)
{
	unsigned char Tmp_buffer[6] = { Register_Add };
	short int Out_data;
	short int temp_d[3];
	char *calcA = &Out_data;
	char *calcB;
	calcB = calcA + 1;

	HAL_I2C_Mem_Read(&hi2c1, senser_Add, Register_Add, I2C_MEMADD_SIZE_8BIT, &Tmp_buffer[0], 6, 100);
	//HAL_I2C_Master_Transmit(&hi2c1, senser_Add, Tmp_buffer, 1, 100);
	//HAL_Delay(10);
	//HAL_I2C_Master_Receive(&hi2c1, (senser_Add | 0x01), Tmp_buffer, 6, 200);

	for (int i = 0; i < 3; i++) {
		*calcB = Tmp_buffer[(2 * i)];
		*calcA = Tmp_buffer[(2 * i) + 1];
		temp_d[i] = Out_data;
	}

	//Magnet X,Y,Z data save
	Magnet_X = temp_d[0] / 820.0;
	Magnet_Y = temp_d[2] / 820.0;
	Magnet_Z = temp_d[1] / 820.0;

}


void Read_MS5611_Data(int Type)
{

	uint8_t temp_A[4] = { 0, };
	uint32_t Data = 0;
	char *calcA, *calcB, *calcC = &Data;
	calcB = calcC + 1;
	calcA = calcC + 2;

	//MS5611 센서의 데이터 시퀸스 구조상 ADC변환 커맨드를 입력후 최대 10ms의 딜레이 타임이 필요하기 때문에 switch 문을 이용해서 ADC 변환을 10루프당 한번씩 실행
	//총 20 루프가 진행된후에 얻어진 데이터를 가지고 보정작업을 시작
	switch (Type) {

	case 1:			//기압 원시 데이터를 받는 switch
		//HAL_I2C_Master_Transmit(&hi2c1, MS5611_ADDRESS, &cammand[0], 1, 100);
		//HAL_Delay(10);
		HAL_I2C_Master_Transmit(&hi2c1, MS5611_ADDRESS, &cammand[2], 1, 100);
		HAL_I2C_Master_Receive(&hi2c1, (MS5611_ADDRESS | 0x01), temp_A, 4, 200);
		*calcC = temp_A[2];
		*calcB = temp_A[1];
		*calcA = temp_A[0];
		D_PRESS = Data;
		break;

	case -1:		//온도 원시 데이터를 받는 switch
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


void Total_Calibration()					//while 문 밖에서 초기 베이스 데이터 수집과 센서 캘리브레이션 수행
{
	HAL_I2C_Master_Transmit(&hi2c1, MS5611_ADDRESS, &cammand[0], 1, 100);
	HAL_Delay(10);
	Read_MS5611_Data(1);
	HAL_I2C_Master_Transmit(&hi2c1, MS5611_ADDRESS, &cammand[1], 1, 100);
	HAL_Delay(10);
	Read_MS5611_Data(-1);

	Calc_TP();


	for (int i = 0; i < 20; i++)
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
	Accel_Z = Accel_Z + (8192 - Base_Accel_Z);				//Z축 가속도는 중력가속도가 작용하기에 값을 빼주어야함

	AccXangle = atan2(Accel_Y, (sqrt((pow(Accel_X, 2) + pow(Accel_Z, 2))))) * 57.3;		//roll
	AccYangle = atan2(Accel_X, (sqrt((pow(Accel_Y, 2) + pow(Accel_Z, 2))))) * 57.3;		//pitch

	Heading = 0.0;
	MagAngle = 180 * (atan2((-1 * Magnet_Y), Magnet_X) - 0.1408) / PI + 180;
	Base_Angle = Heading - MagAngle;

	GyroZangle = Heading;
	GyroXangle = AccXangle;
	GyroYangle = AccYangle;

	CompXangle = AccXangle;
	CompYangle = AccYangle;
	CompZangle = Heading;


}


void While_Loop()						//while문 안에서 원시 데이터를 받으며 계산하는 작업을 반복 수행하는 코드
{
	if (cnt == 0) {
		if (p_flag == 1) HAL_I2C_Master_Transmit(&hi2c1, MS5611_ADDRESS, &cammand[0], 1, 100);
		else HAL_I2C_Master_Transmit(&hi2c1, MS5611_ADDRESS, &cammand[1], 1, 100);
	}

	cnt++;
	if (cnt >= 10) {
		Read_MS5611_Data(p_flag);
		if (p_flag == -1) Calc_TP();
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
	MagAngle = 180 * (atan2((-1 * Magnet_Y), Magnet_X) - 0.1408) / PI + 180;
	Heading = Base_Angle + MagAngle;

	Register = CNT_Register;
	//double dt = Register / 10000.0;
	double ALPHA = 1 / (1 + (double)CNT_Register / 10000.0);

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
}


#endif /* MY_LIB_MY_GY_86_LIB_H_ */
