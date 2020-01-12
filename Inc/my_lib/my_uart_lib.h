/*
 * my_uart_lib.h
 *
 *  Created on: 2019. 1. 4.
 *      Author: ragalia
 */

#ifndef MY_LIB_MY_UART_LIB_H_
#define MY_LIB_MY_UART_LIB_H_


#ifndef my_uart_LIB_en

#include <stdio.h>  //printf 사용시 추가
#include <stdarg.h>
#include <string.h>

HAL_StatusTypeDef RcvState;  			//폴링 방식의 Rx사용시 필요
uint8_t Rx_cnt = 0;						//Rx 데이터를 비트별로 카운트
uint8_t Rx_buf[100];					//받은 Rx 데이터를 문장 종료시('\r') 저장하는 버퍼
uint8_t linkData[10] = "Link_OK\n\r"; 	//uart 연결 성공시 출력
char Rx_data[2];						//Rx 데이터가 들어올때 임시로 저장되는 버퍼
//int Rx_end_flag;						//Rx데이터가 다 받아지면 폴링되는 변수
uint8_t print_buf[102];					//Rx_data저장용 버퍼

#endif


void uart_init()	//uart 인터럽트 셋팅과 링크 확인용 초기화 메서드
{
	HAL_UART_Receive_IT(&huart3, (uint8_t*)Rx_data, 1);
	HAL_UART_Transmit(&huart3, linkData, 10, 10);
}


void tx_send(uint8_t tx_data)		//Tx 데이터를 1바이트씩 전송
{
	HAL_UART_Transmit(&huart3, (uint8_t *)&tx_data, 1, 10);
}


void tx(uint8_t *tx_data)			//문자열 전송시 이용
{
	while(tx_data != '\0'){
		tx_send(*tx_data);
		tx_data++;
	}
}

void tx_by_flag()					//Rx_callback 함수에서 데이터를 다 받으면 호출
{
	for(int i  = 0; i <= Rx_cnt ; i++){
		print_buf[i] = Rx_buf[i];
		Rx_buf[i] = 0;
	}


}

int _write(int file, char* p, int len)	//printf 함수를 이용해서 uart로 전송해 디버그로 활용
{
	HAL_UART_Transmit(&huart3, p, len, 10);
	return len;
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)		//callback 함수 작성
{

  if(huart -> Instance == USART3)
  {
	  if(Rx_data[0] == '\r'){
		  Rx_buf[Rx_cnt++] = '\n';
		  Rx_buf[Rx_cnt] = '\r';
		  tx_by_flag();
		  Rx_cnt = 0;
	  }
	  else{
		  Rx_buf[Rx_cnt++] = Rx_data[0];
	  }
  }

  // 다시 수신인터럽트 활성화 = 재 장전
  HAL_UART_Receive_IT(&huart3, (uint8_t *)Rx_data, 1);
}


#endif /* MY_LIB_MY_UART_LIB_H_ */
