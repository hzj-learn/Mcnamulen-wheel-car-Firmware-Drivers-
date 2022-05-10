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
  * @��  ��  2019-8-2
  * @brief   ���PWM���ƺ���
  *
  ******************************************************************************
  * @˵  ��
  *
  ******************************************************************************
  */

#include "ax_motor.h" 

/**
  * @��  ��  ���PWM���Ƴ�ʼ��	
  * @��  ��  freq_khz:PWM���Ƶ�ʣ���Χ1~20,��λKHz
  * @����ֵ  ��
  */
void AX_MOTOR_Init(uint8_t freq_khz)
{ 
	
	GPIO_InitTypeDef GPIO_InitStructure; 
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure; 

	//��ʱ��ͨ��IO����
	//GPIO�����ù���ʱ��ʹ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);	

	//����IO��Ϊ���ù���-��ʱ��ͨ��
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	//����������IO����
	//IOʱ��ʹ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOC, ENABLE);

	//���A�������IO
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	//���B�������IO
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	//���C�������IO
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	//���D�������IO
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_12;
	GPIO_Init(GPIOA, &GPIO_InitStructure);


	//��ʱ�����ã�
	//��������
	if(freq_khz == 0)
	freq_khz = 1;
	if(freq_khz > 20)
	freq_khz = 20;
	 
	//TIMʱ��ʹ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);

	//Time base configuration
	TIM_TimeBaseStructure.TIM_Period = 2000-1;
	TIM_TimeBaseStructure.TIM_Prescaler = 36/freq_khz-1;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;//
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure);

	//PWM1 Mode configuration: Channel1 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0;	    //ռ�ձȳ�ʼ��
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OC1Init(TIM8, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM8, TIM_OCPreload_Enable);

	//PWM1 Mode configuration: Channel2
	TIM_OC2Init(TIM8, &TIM_OCInitStructure);
	TIM_OC2PreloadConfig(TIM8, TIM_OCPreload_Enable);

	//PWM1 Mode configuration: Channel3
	TIM_OC3Init(TIM8, &TIM_OCInitStructure);
	TIM_OC3PreloadConfig(TIM8, TIM_OCPreload_Enable);

	//PWM1 Mode configuration: Channel4
	TIM_OC4Init(TIM8, &TIM_OCInitStructure);
	TIM_OC4PreloadConfig(TIM8, TIM_OCPreload_Enable);

	TIM_ARRPreloadConfig(TIM8, ENABLE);

	//TIM enable counter
	TIM_Cmd(TIM8, ENABLE);   

	//ʹ��MOEλ
	TIM_CtrlPWMOutputs(TIM8,ENABLE);	
}

/**
  * @��  �� ���A PWM�ٶȿ���
  * @��  �� speed ���ת����ֵ����Χ-2000~2000
  * @����ֵ ��
  */
void AX_MOTOR_A_SetSpeed(int16_t speed)
{
	uint16_t temp;

  if(speed > 0)
	{
		GPIO_ResetBits(GPIOC, GPIO_Pin_4);
	  GPIO_SetBits(GPIOC, GPIO_Pin_5);
		temp = speed;	
	}
	else if(speed < 0)
	{
		GPIO_ResetBits(GPIOC, GPIO_Pin_5);
	  GPIO_SetBits(GPIOC, GPIO_Pin_4);
		temp = (-speed);
	}
	else
	{
		GPIO_ResetBits(GPIOC, GPIO_Pin_4);
	  GPIO_ResetBits(GPIOC, GPIO_Pin_5);
		temp = 0;
	}
	
	if(temp>2000)
		temp = 2000;
	
	TIM_SetCompare1(TIM8,temp);
}

/**
  * @��  �� ���B PWM�ٶȿ���
  * @��  �� speed ���ת����ֵ����Χ-2000~2000
  * @����ֵ ��
  */
void AX_MOTOR_B_SetSpeed(int16_t speed)
{
	uint16_t temp;

    if(speed > 0)
	{
		GPIO_ResetBits(GPIOC, GPIO_Pin_2);
	  GPIO_SetBits(GPIOC, GPIO_Pin_3);
		temp = speed;	
	}
	else if(speed < 0)
	{
		GPIO_ResetBits(GPIOC, GPIO_Pin_3);
	  GPIO_SetBits(GPIOC, GPIO_Pin_2);
		temp = (-speed);
	}
	else
	{
		GPIO_ResetBits(GPIOC, GPIO_Pin_2);
	  GPIO_ResetBits(GPIOC, GPIO_Pin_3);
		temp = 0;
	}
	
	if(temp>2000)
		temp = 2000;
	
	TIM_SetCompare2(TIM8,temp);
}
/**
  * @��  �� ���C PWM�ٶȿ���
  * @��  �� speed ���ת����ֵ����Χ-2000~2000
  * @����ֵ ��
  */
void AX_MOTOR_C_SetSpeed(int16_t speed)
{
	uint16_t temp;

  if(speed > 0)
	{
		GPIO_ResetBits(GPIOC, GPIO_Pin_1);
	  GPIO_SetBits(GPIOC, GPIO_Pin_0);
		temp = speed;	
	}
	else if(speed < 0)
	{
		GPIO_ResetBits(GPIOC, GPIO_Pin_0);
	  GPIO_SetBits(GPIOC, GPIO_Pin_1);
		temp = (-speed);
	}
	else
	{
		GPIO_ResetBits(GPIOC, GPIO_Pin_1);
	  GPIO_ResetBits(GPIOC, GPIO_Pin_0);
		temp = 0;
	}
	
	if(temp>2000)
		temp = 2000;
	
	TIM_SetCompare3(TIM8,temp);
}

/**
  * @��  �� ���D PWM�ٶȿ���
  * @��  �� speed ���ת����ֵ����Χ-2000~2000
  * @����ֵ ��
  */
void AX_MOTOR_D_SetSpeed(int16_t speed)
{
	uint16_t temp;

  if(speed > 0)
	{
		GPIO_ResetBits(GPIOA, GPIO_Pin_11);
	  GPIO_SetBits(GPIOA, GPIO_Pin_12);
		temp = speed;	
	}
	else if(speed < 0)
	{
		GPIO_ResetBits(GPIOA, GPIO_Pin_12);
	  GPIO_SetBits(GPIOA, GPIO_Pin_11);
		temp = (-speed);
	}
	else
	{
		GPIO_ResetBits(GPIOA, GPIO_Pin_11);
	  GPIO_ResetBits(GPIOA, GPIO_Pin_12);
		temp = 0;
	}
	
	if(temp>2000)
		temp = 2000;
	
	TIM_SetCompare4(TIM8,temp);
}



/******************* (C) ��Ȩ 2019 XTARK **************************************/
