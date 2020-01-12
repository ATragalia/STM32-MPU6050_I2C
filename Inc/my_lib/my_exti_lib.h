/*
 * my_exti_lib.h
 *
 *  Created on: 2019. 1. 7.
 *      Author: ragalia
 */

#ifndef MY_LIB_MY_EXTI_LIB_H_
#define MY_LIB_MY_EXTI_LIB_H_


#ifndef Register_Address_access_en

#define GPIO_IDR_Register_set (*(volatile unsigned *)0x48000810)		//GPIO의 IDR레지스터 어드레스 접근법


#endif

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == GPIO_PIN_0){

		}
	if(GPIO_Pin == GPIO_PIN_1){

		}
	if(GPIO_Pin == GPIO_PIN_2){

		}
	if(GPIO_Pin == GPIO_PIN_3){

		}
	if(GPIO_Pin == GPIO_PIN_4){

		}
	if(GPIO_Pin == GPIO_PIN_5){

		}
	if(GPIO_Pin == GPIO_PIN_6){

		}
	if(GPIO_Pin == GPIO_PIN_7){

		}
	if(GPIO_Pin == GPIO_PIN_8){

		}
	if(GPIO_Pin == GPIO_PIN_9){

		}
	if(GPIO_Pin == GPIO_PIN_10){

		}
	if(GPIO_Pin == GPIO_PIN_11){

		}
	if(GPIO_Pin == GPIO_PIN_12){

		}
	if(GPIO_Pin == GPIO_PIN_13){

		}
	if(GPIO_Pin == GPIO_PIN_14){

		}
	if(GPIO_Pin == GPIO_PIN_15){

		}

}


#endif /* MY_LIB_MY_EXTI_LIB_H_ */
