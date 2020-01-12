# STM32-MPU6050_I2C

# 비행제어 모듈(GY-86)에 부착되어 있는 센서 세가지 MPU-6050, HMC5883L, MS5611의 RAW데이터를 I2C통신을 통해 받는 프로젝트

초기 I2C셋팅과 데이터 받는 루틴은 HAL_I2C_Mem_Read 함수를 사용하였고 
MPU6050 의 가속도 자이로 Raw 데이터와 HMC5883L 의 지자기 Raw 데이터를 가공하여 
비행각 Roll, Pitch, Yaw 를 구하는것은 내부적인 Timer7 dt측정값을 이용한 상보필터 방식을 사용하여 구하였다.
