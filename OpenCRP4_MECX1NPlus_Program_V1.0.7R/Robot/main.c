

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include <stdio.h>
#include <math.h>   

#include "ax_sys.h"    				//ϵͳ����
#include "ax_delay.h"  				//�����ʱ
#include "ax_led.h"    				//LED�ƿ���
#include "ax_vin.h"    				//�����ѹ���
#include "ax_uart_db.h"  			//���Դ���
#include "ax_uart_pi.h"  			//��ݮ�ɴ���
#include "ax_motor.h"    			//ֱ��������ٿ���
#include "ax_encoder.h"  			//����������
#include "ax_tim.h"      			//��ʱ��
#include "ax_mpu6050.h"  			//IMU���ٶ������ǲ���
#include "ax_mpu6050_dmp.h"   //DMP���ܺ���
#include "ax_pid.h"        		//PID����
#include "ax_kinematics.h" 		//�˶�ѧ����


#define ENCODER_MID_VALUE  30000  
#define VBAT_VOL_CHG    1050 
#define VBAT_VOL_OFF    990   


int16_t ax_encoder[4];											//�������ۼ�ֵ
int16_t ax_encoder_delta[4];								//�������仯ֵ
int16_t ax_encoder_delta_target[4] = {0};   //������Ŀ��仯ֵ
int16_t robot_odom[6] = {0}; 								//��̼����ݣ�����ֵ�ͱ仯ֵ��x y yaw dx dy dyaw
int16_t ax_motor_pwm[4];  									//���PWM
uint16_t ax_bat_vol;  											//��ص�ѹ
int16_t mpu_data[10];  											//�����ǣ����ٶȣ���̬��

int16_t robot_target_speed[3] = {0};  			//������Ŀ���ٶ� X Y Yaw
int16_t robot_params[2] = {1000,1000};  		//�����˲���

//��Ҫ��������
void AX_ROBOT_GetImuData(void);  				//��ȡMPU6050����
void AX_ROBOT_MoveCtl(void); 					  //�������˶����ƺ���  
void AX_ROBOT_BatteryManagement(void);  //�����˵�ع���
void AX_ROBOT_SendDataToPi(void);  			//�����˷������ݵ���ݮ��

/**
  * @��  ��  ����������
  * @��  ��  ��
  * @����ֵ  ��
  */
int main(void)
{
	//��ʱ������
	uint8_t cnt = 1;  
	//�����жϷ���
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);   

	//�����˳�ʼ��
	//(1)��ʼ����������10khz��Ƶ�ʿ������PWM��Ƶ������ķ�Χ��(1khz~20khz)
	AX_MOTOR_Init(10);
	//(2)��ʼ����ʱ����
	AX_DELAY_Init(); 
  //(3)��ʼ�����ó�������ģʽ 	
	AX_JTAG_Set(JTAG_SWD_DISABLE);     
	AX_JTAG_Set(SWD_ENABLE); 
  //(4)��ʼ��LED	
	AX_LED_Init();
	//(5)��ʼ����ѹ���ADC������Ӳ������һ��1��10�ķ�ѹ��·��12V�൱��1.2V���뵥Ƭ����ADC
	AX_VIN_Init();
	//(6)��ʼ�������õĴ��ڣ���type-c�Ľӿڵģ������ڴ��ڵ��ԣ�Ҳ��������hex�ļ��̼�
	AX_UART_DB_Init(115200);
	//(7)��ʼ��������ݮ��/nano�Ĵ���
	AX_UART_PI_Init(115200);

	//(8)��ʼ��������������������ĸ�֣�����ֻ���ĸ������TIM��ʱ�����ڲ��������AB���������ƽ��һ����8���������������ߣ�
	AX_ENCODER_AB_Init(ENCODER_MID_VALUE*2);  
	AX_ENCODER_CD_Init(ENCODER_MID_VALUE*2);  
	AX_ENCODER_EF_Init(ENCODER_MID_VALUE*2);  
	AX_ENCODER_GH_Init(ENCODER_MID_VALUE*2);  
	//������AB���ü�������ֵ
	AX_ENCODER_AB_SetCounter(ENCODER_MID_VALUE); 
	AX_ENCODER_CD_SetCounter(ENCODER_MID_VALUE); 
	AX_ENCODER_EF_SetCounter(ENCODER_MID_VALUE); 
	AX_ENCODER_GH_SetCounter(ENCODER_MID_VALUE);
  //(9)MPU6050��ʼ����IMU��(�г��ҿ�ʵ�ֵ�)
	AX_MPU6050_Init();    
	//DMP��ʼ��
	AX_MPU6050_DMP_Init();	
	//(10)��ʱ���жϳ�ʼ�������ڿ�������Ƶ��
	AX_TIM6_Init(10000);
  AX_TIM6_Cmd(ENABLE);
  //(11)������ʾ��Ϣ
	AX_LED_Red_On();
	AX_LED_Green_On();
	AX_Delayms(100);	
	AX_LED_Red_Off();
	AX_LED_Green_Off();
	AX_Delayms(300);
  //�̵Ƶ�������ʾ����
  AX_LED_Green_On();

	while(1)
	{
		//100HZ����Ƶ��
		if(AX_TIM6_CheckIrqStatus())
		{		
			//(1)�����˻�ȡPMU6050����
			AX_ROBOT_GetImuData();	
			//50HZִ��Ƶ��
			if(cnt%2 == 0)
			{
				//(2)�������˶�����
				AX_ROBOT_MoveCtl();
				//(3)�����˵�������
				AX_ROBOT_BatteryManagement();
				//(4)�����˴��ڷ������ݵ�nano
				AX_ROBOT_SendDataToPi();
			}
			//�������ۼ�
			cnt++;
		}
	}
}

/**
  * @��  ��  �����˻�ȡMPU6050 ���ٶ���������̬����
  * @��  ��  ��
  * @����ֵ  ��
  */
void AX_ROBOT_GetImuData(void)
{
	  //�������壬����Ƕ�
    AX_MPU6050_DMP_GetData(mpu_data);
	
}
/**
  * @��  ��  �������˶����ƺ���
  * @��  ��  ��
  * @����ֵ  ��
  */
void AX_ROBOT_MoveCtl(void)
{  
	//(1)��ȡ�������仯ֵ
	ax_encoder_delta[0] = -(AX_ENCODER_AB_GetCounter()-ENCODER_MID_VALUE);
	ax_encoder_delta[1] = AX_ENCODER_CD_GetCounter()  -ENCODER_MID_VALUE;
	ax_encoder_delta[2] = -(AX_ENCODER_EF_GetCounter()-ENCODER_MID_VALUE);
	ax_encoder_delta[3] = AX_ENCODER_GH_GetCounter()  -ENCODER_MID_VALUE;
	
	//(2)���ñ������м�ֵ
	AX_ENCODER_AB_SetCounter(ENCODER_MID_VALUE);
	AX_ENCODER_CD_SetCounter(ENCODER_MID_VALUE);
	AX_ENCODER_EF_SetCounter(ENCODER_MID_VALUE);
	AX_ENCODER_GH_SetCounter(ENCODER_MID_VALUE);
	
	//(3)����������ۼ�ֵ
	ax_encoder[0] = ax_encoder[0] + ax_encoder_delta[0];
	ax_encoder[1] = ax_encoder[1] + ax_encoder_delta[1];
	ax_encoder[2] = ax_encoder[2] + ax_encoder_delta[2];
	ax_encoder[3] = ax_encoder[3] + ax_encoder_delta[3];
	
	//(4)�˶�ѧ����
	AX_Kinematics_Forward(ax_encoder,robot_odom);  //�����˶�ѧ����
	AX_Kinematics_Inverse(robot_target_speed, ax_encoder_delta_target);  //�����˶�ѧ����

	//(5)���PID�ٶȿ���
	ax_motor_pwm[0] = AX_PID_MotorVelocityCtlA(ax_encoder_delta_target[0], ax_encoder_delta[0]);   
	ax_motor_pwm[1] = AX_PID_MotorVelocityCtlB(ax_encoder_delta_target[1], ax_encoder_delta[1]);   
	ax_motor_pwm[2] = AX_PID_MotorVelocityCtlC(ax_encoder_delta_target[2], ax_encoder_delta[2]);   
	ax_motor_pwm[3] = AX_PID_MotorVelocityCtlD(ax_encoder_delta_target[3], ax_encoder_delta[3]);  
						
	AX_MOTOR_A_SetSpeed(-ax_motor_pwm[0]);
	AX_MOTOR_B_SetSpeed(-ax_motor_pwm[1]);  
	AX_MOTOR_C_SetSpeed(ax_motor_pwm[2]);
	AX_MOTOR_D_SetSpeed(ax_motor_pwm[3]); 
		
}

/**
  * @��  ��  �����˵�ع���
  * @��  ��  ��
  * @����ֵ  ��
  */
void AX_ROBOT_BatteryManagement(void)
{
	//(1)��ѹ��������
	static uint16_t ax_bat_vol_cnt = 0;   
	
	//(2)ADC�ɼ���ص�ѹ
	ax_bat_vol = AX_VIN_GetVol_X100();			
	
	if(ax_bat_vol < VBAT_VOL_CHG) //��ص�ѹС�ڱ�����ѹ10.9Vʱ
	{

		AX_LED_Red_Toggle();//��ɫLED����ʾ
		
		if(ax_bat_vol < VBAT_VOL_OFF) //��ص�ѹС�ڱ�����ѹ10.9Vʱ
		{
			ax_bat_vol_cnt++;
      
			//��ѹ����С��VBAT_VOL_OFF 4�룬����ͣ������״̬
			if(ax_bat_vol_cnt > 200 )
			{
				//��Ƴ���������ͣ��״̬�����ֹͣת��
				AX_LED_Red_On();
				AX_LED_Green_Off();
				AX_MOTOR_A_SetSpeed(0);  
				AX_MOTOR_B_SetSpeed(0);  
				AX_MOTOR_C_SetSpeed(0);  
				AX_MOTOR_D_SetSpeed(0);  
				
				//����
				while(1)
				{	
					AX_Delayms(100);
				}								
			}
		}
		else										
		{
			ax_bat_vol_cnt = 0;
		}
	}
	else																 //��ص�ѹ���ڱ�����ѹ10.9Vʱ
	{
		AX_LED_Red_Off();
	}
}


/**
  * @��  ��  �����˷������ݵ���ݮ��
  * @��  ��  ��
  * @����ֵ  ��
  */
void AX_ROBOT_SendDataToPi(void)
{
	    //���ڷ�������
			static uint8_t comdata[60]; 			
	
			//�����ǽ��ٶ� = (ax_gyro/32768) * 2000 ?s
			comdata[0] = (u8)( mpu_data[0] >> 8 );  
			comdata[1] = (u8)( mpu_data[0] );
			comdata[2] = (u8)( mpu_data[1] >> 8 );
			comdata[3] = (u8)( mpu_data[1] );
			comdata[4] = (u8)( mpu_data[2] >> 8 );
			comdata[5] = (u8)( mpu_data[2] );
			
			//���ٶ� = (ax_acc/32768) * 2G  
			comdata[6] = (u8)( mpu_data[3] >> 8 );
			comdata[7] = (u8)( mpu_data[3] );
			comdata[8] = (u8)( mpu_data[4] >> 8 );
			comdata[9] = (u8)( mpu_data[4] );
			comdata[10] = (u8)( mpu_data[5] >> 8 );
			comdata[11] = (u8)( mpu_data[5] );
			
			//��̬�Ƕ� = (ax_angle/100)
			comdata[12] = (u8)( mpu_data[6] >> 8 ); 
			comdata[13] = (u8)( mpu_data[6] );
			comdata[14] = (u8)( mpu_data[7] >> 8 );
			comdata[15] = (u8)( mpu_data[7] );
			comdata[16] = (u8)( mpu_data[8] >> 8 );
			comdata[17] = (u8)( mpu_data[8] );
			
			//��̼����� x(m) y(m) yaw(rad)  odom_frame
			comdata[18] = (u8)( robot_odom[0] >> 8 );
			comdata[19] = (u8)( robot_odom[0] );
			comdata[20] = (u8)( robot_odom[1] >> 8 );
			comdata[21] = (u8)( robot_odom[1] );
			comdata[22] = (u8)( robot_odom[2] >> 8 );
			comdata[23] = (u8)( robot_odom[2] );
			
			//��̼�����仯��  d_x(m) d_y(m) d_yaw(rad)  base_frame
			comdata[24] = (u8)( robot_odom[3] >> 8 );
			comdata[25] = (u8)( robot_odom[3] );
			comdata[26] = (u8)( robot_odom[4] >> 8 );
			comdata[27] = (u8)( robot_odom[4] );
			comdata[28] = (u8)( robot_odom[5] >> 8 );
			comdata[29] = (u8)( robot_odom[5] );
		
		  //��������ǰֵ��Ŀ��ֵ
			comdata[30] = (u8)( ax_encoder_delta[0] >> 8 );  
			comdata[31] = (u8)( ax_encoder_delta[0] );
			comdata[32] = (u8)( ax_encoder_delta[1] >> 8 );
			comdata[33] = (u8)( ax_encoder_delta[1] );
			comdata[34] = (u8)( ax_encoder_delta[2] >> 8 );
			comdata[35] = (u8)( ax_encoder_delta[2] );
			comdata[36] = (u8)( ax_encoder_delta[3] >> 8 );
			comdata[37] = (u8)( ax_encoder_delta[3] );
			
			comdata[38] = (u8)( ax_encoder_delta_target[0] >> 8 );  
			comdata[39] = (u8)( ax_encoder_delta_target[0] );
			comdata[40] = (u8)( ax_encoder_delta_target[1] >> 8 );
			comdata[41] = (u8)( ax_encoder_delta_target[1] );
			comdata[42] = (u8)( ax_encoder_delta_target[2] >> 8 );
			comdata[43] = (u8)( ax_encoder_delta_target[2] );
			comdata[44] = (u8)( ax_encoder_delta_target[3] >> 8 );
			comdata[45] = (u8)( ax_encoder_delta_target[3] );
			
			//������
			comdata[46] = (u8)( ax_bat_vol >> 8 );
			comdata[47] = (u8)( ax_bat_vol );
				
			//���ʹ�������
			AX_UART_PI_SendPacket(comdata, 48, 0x06);
}

/******************* (C) ��Ȩ 2019 XTARK **************************************/

