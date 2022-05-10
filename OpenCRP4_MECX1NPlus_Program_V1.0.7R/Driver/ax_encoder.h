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
  * @author  Musk Han @ XTARK
  * @version V1.1
  * @date    2019-01-01
  * @brief   PWM������
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __AX_ENCODER_H
#define __AX_ENCODER_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"

//OpenCRP�����ӿں���
void AX_ENCODER_AB_Init(uint16_t cycle);          //��������ʼ��
uint16_t AX_ENCODER_AB_GetCounter(void);          //��������ȡ��������ֵ
void AX_ENCODER_AB_SetCounter(uint16_t count);    //���������ü�������ֵ

void AX_ENCODER_CD_Init(uint16_t cycle);          //��������ʼ��
uint16_t AX_ENCODER_CD_GetCounter(void);          //��������ȡ��������ֵ
void AX_ENCODER_CD_SetCounter(uint16_t count);    //���������ü�������ֵ

void AX_ENCODER_EF_Init(uint16_t cycle);          //��������ʼ��
uint16_t AX_ENCODER_EF_GetCounter(void);          //��������ȡ��������ֵ
void AX_ENCODER_EF_SetCounter(uint16_t count);    //���������ü�������ֵ

void AX_ENCODER_GH_Init(uint16_t cycle);          //��������ʼ��
uint16_t AX_ENCODER_GH_GetCounter(void);          //��������ȡ��������ֵ
void AX_ENCODER_GH_SetCounter(uint16_t count);    //���������ü�������ֵ

#endif

/******************* (C) ��Ȩ 2019 XTARK **************************************/
