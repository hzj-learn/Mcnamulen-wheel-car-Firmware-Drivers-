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
  * @˵  ��
  *
  * 
  ******************************************************************************
  */

#include "ax_led.h"

/**
  * @��  ��  LED ��ʼ��
  * @��  ��  ��
  * @����ֵ  ��
  */
void AX_LED_Init(void) 
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	//LED GPIO����
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOA, ENABLE);	 
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
	GPIO_Init(GPIOD, &GPIO_InitStructure);	
	
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_8;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	//�ر�LED��
	GPIO_SetBits(GPIOD,GPIO_Pin_2);
	GPIO_SetBits(GPIOA,GPIO_Pin_8);	
}

/******************* (C) ��Ȩ 2019 XTARK **************************************/
