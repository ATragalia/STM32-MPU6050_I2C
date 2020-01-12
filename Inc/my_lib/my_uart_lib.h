/*
 * my_uart_lib.h
 *
 *  Created on: 2019. 1. 4.
 *      Author: ragalia
 */

#ifndef MY_LIB_MY_UART_LIB_H_
#define MY_LIB_MY_UART_LIB_H_


#ifndef my_uart_LIB_en

#include <stdio.h>  //printf ���� �߰�
#include <stdarg.h>
#include <string.h>

HAL_StatusTypeDef RcvState;  			//���� ����� Rx���� �ʿ�
uint8_t Rx_cnt = 0;						//Rx �����͸� ��Ʈ���� ī��Ʈ
uint8_t Rx_buf[100];					//���� Rx �����͸� ���� �����('\r') �����ϴ� ����
uint8_t linkData[10] = "Link_OK\n\r"; 	//uart ���� ������ ���
char Rx_data[2];						//Rx �����Ͱ� ���ö� �ӽ÷� ����Ǵ� ����
//int Rx_end_flag;						//Rx�����Ͱ� �� �޾����� �����Ǵ� ����
uint8_t print_buf[102];					//Rx_data����� ����

#endif


void uart_init()	//uart ���ͷ�Ʈ ���ð� ��ũ Ȯ�ο� �ʱ�ȭ �޼���
{
	HAL_UART_Receive_IT(&huart3, (uint8_t*)Rx_data, 1);
	HAL_UART_Transmit(&huart3, linkData, 10, 10);
}


void tx_send(uint8_t tx_data)		//Tx �����͸� 1����Ʈ�� ����
{
	HAL_UART_Transmit(&huart3, (uint8_t *)&tx_data, 1, 10);
}


void tx(uint8_t *tx_data)			//���ڿ� ���۽� �̿�
{
	while(tx_data != '\0'){
		tx_send(*tx_data);
		tx_data++;
	}
}

void tx_by_flag()					//Rx_callback �Լ����� �����͸� �� ������ ȣ��
{
	for(int i  = 0; i <= Rx_cnt ; i++){
		print_buf[i] = Rx_buf[i];
		Rx_buf[i] = 0;
	}


}

int _write(int file, char* p, int len)	//printf �Լ��� �̿��ؼ� uart�� ������ ����׷� Ȱ��
{
	HAL_UART_Transmit(&huart3, p, len, 10);
	return len;
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)		//callback �Լ� �ۼ�
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

  // �ٽ� �������ͷ�Ʈ Ȱ��ȭ = �� ����
  HAL_UART_Receive_IT(&huart3, (uint8_t *)Rx_data, 1);
}


#endif /* MY_LIB_MY_UART_LIB_H_ */
