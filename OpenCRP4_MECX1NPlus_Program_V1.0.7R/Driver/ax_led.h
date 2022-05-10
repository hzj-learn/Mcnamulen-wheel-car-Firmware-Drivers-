/**			                                                    
		   ____                    _____ _____  _____        XTARK@���˴���
		  / __ \                  / ____|  __ \|  __ \  
		 | |  | |_ __   ___ _ __ | |    | |__) | |__) |
		 | |  | | '_ \ / _ \ '_ \| |    |  _  /|  ___/ 
		 | |__| | |_) |  __/ | | | |____| | \ \| |     
		  \____/| .__/ \___|_| |_|\_____|_|  \_\_|     
		    		| |                                    
		    		|_|  OpenCRP ��ݮ�� ר��ROS�����˿�����                                   
									 
  ****************************************************************************** 
  *           
  * ��Ȩ���У� XTARK@���˴���  ��Ȩ���У�����ؾ�
  * ������վ�� www.xtark.cn
  * �Ա����̣� https://shop246676508.taobao.com
  * ����ý�壺 www.cnblogs.com/xtark�����ͣ�
	* ����΢�ţ� ΢�Ź��ںţ����˴��£���ȡ������Ѷ��
  *      
  ******************************************************************************
  * @��  ��  Musk Han@XTARK
  * @��  ��  V1.0
  * @��  ��  2019-7-26
  * @��  ��  LED�ƿ���
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __AX_LED_H
#define __AX_LED_H

/* Includes ------------------------------------------------------------------*/	 
#include "stm32f10x.h"

//OpenCRP�����ӿں���
void AX_LED_Init(void);

#define AX_LED_Red_Off()  	     GPIO_SetBits(GPIOD, GPIO_Pin_2)      //LEDG��ɫϨ��
#define AX_LED_Red_On()		       GPIO_ResetBits(GPIOD, GPIO_Pin_2)    //LEDG��ɫ����
#define AX_LED_Red_Toggle()      GPIO_WriteBit(GPIOD, GPIO_Pin_2, (BitAction) (1 - GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_2)))	//LEDG��ɫ״̬��ת

#define AX_LED_Green_Off()  	     GPIO_SetBits(GPIOA, GPIO_Pin_8)      //LEDG��ɫϨ��
#define AX_LED_Green_On()		       GPIO_ResetBits(GPIOA, GPIO_Pin_8)    //LEDG��ɫ����
#define AX_LED_Green_Toggle()      GPIO_WriteBit(GPIOA, GPIO_Pin_8, (BitAction) (1 - GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_8)))	//LEDG��ɫ״̬��ת

#endif 

/******************* (C) ��Ȩ 2019 XTARK **************************************/
