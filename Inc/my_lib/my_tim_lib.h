/*
 * my_tim_lib.h
 *
 *  Created on: 2019. 1. 6.
 *      Author: ragalia
 */

#ifndef MY_LIB_MY_TIM_LIB_H_
#define MY_LIB_MY_TIM_LIB_H_

void Tim_init()
{
	HAL_TIM_Base_Start_IT(&htim2);		//Ÿ�̸� ���ͷ�Ʈ�� �����Ű�� ���� Ʈ����
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)		//Ÿ�̸� ���ͷ�Ʈ �ݹ��Լ�
{
	  static unsigned char num =0;
	  if(htim -> Instance == TIM2){		//����ü ����
		  printf("hello\n\r");
		  num++;
	  }
}



#endif /* MY_LIB_MY_TIM_LIB_H_ */
