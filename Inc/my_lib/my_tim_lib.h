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
	HAL_TIM_Base_Start_IT(&htim2);		//타이머 인터럽트를 실행시키기 위한 트리거
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)		//타이머 인터럽트 콜백함수
{
	  static unsigned char num =0;
	  if(htim -> Instance == TIM2){		//구조체 변수
		  printf("hello\n\r");
		  num++;
	  }
}



#endif /* MY_LIB_MY_TIM_LIB_H_ */
