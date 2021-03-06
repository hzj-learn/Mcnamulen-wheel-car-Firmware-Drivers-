/**			                                                    
		   ____                    _____ _____  _____        XTARK@塔克创新
		  / __ \                  / ____|  __ \|  __ \  
		 | |  | |_ __   ___ _ __ | |    | |__) | |__) |
		 | |  | | '_ \ / _ \ '_ \| |    |  _  /|  ___/ 
		 | |__| | |_) |  __/ | | | |____| | \ \| |     
		  \____/| .__/ \___|_| |_|\_____|_|  \_\_|     
		    		| |                                    
		    		|_|  OpenCRP 树莓派 专用ROS机器人控制器                                   
									 
  ****************************************************************************** 
  *           
  * 版权所有： XTARK@塔克创新  版权所有，盗版必究
  * 官网网站： www.xtark.cn
  * 淘宝店铺： https://shop246676508.taobao.com  
  * 塔克媒体： www.cnblogs.com/xtark（博客）
	* 塔克微信： 微信公众号：塔克创新（获取最新资讯）
  *      
  ******************************************************************************
  * @作  者  Musk Han@XTARK
  * @版  本  V1.0
  * @日  期  2019-8-8
  * @内  容  机器人运动学解析
  *
  ******************************************************************************
  * @说  明
  *  
  * 
  ******************************************************************************
  */


#include "ax_kinematics.h"
#include <stdio.h>
#include "ax_delay.h"
#include <math.h>


//变量定义
int32_t  current_count[4] = {0};
double    ticks_per_meter = 0;
double   linear_correction_factor = 1.0;
double   angular_correction_factor = 1.0;
int32_t  wheel_mult[4] = {0};
float  wheel_track_cali = 0.3;

extern int16_t robot_odom[6];
extern int16_t robot_target_speed[3];

/**
  * @简  述  机器人运动参数设置
  * @参  数  无
  * @返回值  无
  */
	void AX_Kinematics_Init(int16_t* robot_params)
{

	linear_correction_factor    = (float)robot_params[0]/1000;
  angular_correction_factor   = (float)robot_params[1]/1000;
	wheel_track_cali = WHEEL_TRACK/angular_correction_factor;


	robot_odom[0]  = 0;
	robot_odom[1]  = 0;
	robot_odom[2]  = 0;

	ticks_per_meter    = (float)ENCODER_RESOLUTION/(WHEEL_DIAMETER*3.1415926*linear_correction_factor);		
}

/**
  * @简  述  逆向运动学解析，底盘三轴速度->轮子速度
  * @参  数  input:  robot_target_speed[]  机器人三轴速度 m/s*1000
  *          output：ax_encoder_delta_target[] 电机期望速度 count
  * @返回值  无
  */
void AX_Kinematics_Inverse(int16_t* input, int16_t* output)
{
	float x_speed   = ((float)input[0])/1000;//当前底盘x轴速度
	float y_speed   = ((float)input[1])/1000;//当前底盘y轴速度
	float yaw_speed = ((float)input[2])/1000;//当前底盘yaw轴速度
	static float wheel_velocity[4] = {0};    //期望的四个轮子轮子的速度
	//(1)根据当前底盘的三轴速度解算出四个轮子的速度
	wheel_velocity[0] = -y_speed + x_speed - (wheel_track_cali)*yaw_speed;
	wheel_velocity[1] = y_speed + x_speed + (wheel_track_cali)*yaw_speed;
	wheel_velocity[2] = y_speed + x_speed - (wheel_track_cali)*yaw_speed;
	wheel_velocity[3] = -y_speed + x_speed + (wheel_track_cali)*yaw_speed;

	output[0] = (int16_t)(wheel_velocity[0] * ticks_per_meter/PID_RATE);
	output[1] = (int16_t)(wheel_velocity[1] * ticks_per_meter/PID_RATE);
	output[2] = (int16_t)(wheel_velocity[2] * ticks_per_meter/PID_RATE);
	output[3] = (int16_t)(wheel_velocity[3] * ticks_per_meter/PID_RATE);
}

/**
  * @简  述  正向运动学解析，轮子编码值->底盘三轴里程计坐标
  * @参  数  input: ax_encoder[]  编码器累加值
  *          output: robot_odom[] 三轴里程计 x y yaw
  * @返回值  无
  */
void AX_Kinematics_Forward(int16_t* input, int16_t* output)
{
	static double delta_count[4];      //编码器数据
  static double delta_v_ave[3];      //速度平均值
	static double delta_v_integral[2]; //位置平均值
	static int16_t recv_count[4];
	
	recv_count[0] = -input[0];
	recv_count[1] = input[1];
	recv_count[2] = -input[2];
	recv_count[3] = input[3];
	
	//(1)编码器计数溢出处理,正向溢出就是正转一圈，反向溢出就是反转一圈
	for(int i=0;i<4;i++)
	{
			if(recv_count[i] < ENCODER_LOW_WRAP && current_count[i] > ENCODER_HIGH_WRAP)
				wheel_mult[i]++;
			else if(recv_count[i] > ENCODER_HIGH_WRAP && current_count[i] < ENCODER_LOW_WRAP)
				wheel_mult[i]--;
			else
				wheel_mult[i]=0;
	}
	//	printf("%d %d %d %d\r\n",wheel_mult[0],wheel_mult[1],wheel_mult[2],wheel_mult[3]);
	//(2)将编码器数值转化为前进的距离，和轮子直径相关，单位m
	for(int i=0;i<4;i++)
	{	
		//公式：前进的距离增量=(当前计数值+圈数*(最大的计数值-最小的计数值)-上次累计的圈数)/每个计数值对应的米?
			delta_count[i] = 1.0*(recv_count[i] + wheel_mult[i]*(ENCODER_MAX-ENCODER_MIN)-current_count[i])/ticks_per_meter;
			current_count[i] = recv_count[i];
	}
		//(3)计算底盘x轴变化距离、Yaw轴朝向变化rad角度
	delta_v_ave[0] = (delta_count[3] - delta_count[2])/2.0;
	delta_v_ave[1] = (delta_count[2] - delta_count[0])/2.0;
	delta_v_ave[2] = (delta_count[2]+delta_count[1])/(2*wheel_track_cali);
		//(4)计算底盘坐标系下的x轴与Yaw轴的速度
	delta_v_integral[0] = cos(delta_v_ave[2])*delta_v_ave[0] - sin(delta_v_ave[2])*delta_v_ave[1];
	delta_v_integral[1] = -sin(delta_v_ave[2])*delta_v_ave[0] - cos(delta_v_ave[2])*delta_v_ave[1];
   //(5)计算积分计算里程计坐标系(odom_frame)下的机器人X,Y,Yaw轴坐标
	output[0] += (int16_t)((cos((double)output[2]/1000)*delta_v_integral[0] - sin((double)output[2]/1000)*delta_v_integral[1])*1000);
	output[1] += (int16_t)((sin((double)output[2]/1000)*delta_v_integral[0] + cos((double)output[2]/1000)*delta_v_integral[1])*1000);
	output[2] += (int16_t)(delta_v_ave[2]*1000);
		
   //(6)Yaw轴坐标变化范围控制-2Π -> 2Π
		if(output[2] > PI*1000)
			output[2] -= 2*PI*1000;
		else if(output[2] < -PI*1000)
			output[2] += 2*PI*1000;
		
		//(7)发送机器人XY轴Yaw轴速度反馈
	output[3] = (int16_t)(delta_v_ave[0]*1000);
	output[4] = (int16_t)(-delta_v_ave[1]*1000);
	output[5] = (int16_t)(delta_v_ave[2]*1000);
}


/******************* (C) 版权 2019 XTARK **************************************/
