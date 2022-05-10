/**			                                                    
		   ____                    _____ _____  _____        XTARK@Ëş¿Ë´´ĞÂ
		  / __ \                  / ____|  __ \|  __ \  
		 | |  | |_ __   ___ _ __ | |    | |__) | |__) |
		 | |  | | '_ \ / _ \ '_ \| |    |  _  /|  ___/ 
		 | |__| | |_) |  __/ | | | |____| | \ \| |     
		  \____/| .__/ \___|_| |_|\_____|_|  \_\_|     
		    		| |                                    
		    		|_|  OpenCRP Ê÷İ®ÅÉ ×¨ÓÃROS»úÆ÷ÈË¿ØÖÆÆ÷                                   
									 
  ****************************************************************************** 
  *           
  * °æÈ¨ËùÓĞ£º XTARK@Ëş¿Ë´´ĞÂ  °æÈ¨ËùÓĞ£¬µÁ°æ±Ø¾¿
  * ¹ÙÍøÍøÕ¾£º www.xtark.cn
  * ÌÔ±¦µêÆÌ£º https://shop246676508.taobao.com  
  * Ëş¿ËÃ½Ìå£º www.cnblogs.com/xtark£¨²©¿Í£©
	* Ëş¿ËÎ¢ĞÅ£º Î¢ĞÅ¹«ÖÚºÅ£ºËş¿Ë´´ĞÂ£¨»ñÈ¡×îĞÂ×ÊÑ¶£©
  *      
  ******************************************************************************
  * @×÷  Õß  Musk Han@XTARK
  * @°æ  ±¾  V1.0
  * @ÈÕ  ÆÚ  2019-8-8
  * @ÄÚ  Èİ  »úÆ÷ÈËÔË¶¯Ñ§½âÎö
  *
  ******************************************************************************
  * @Ëµ  Ã÷
  *  
  * 
  ******************************************************************************
  */


#include "ax_kinematics.h"
#include <stdio.h>
#include "ax_delay.h"
#include <math.h>


//±äÁ¿¶¨Òå
int32_t  current_count[4] = {0};
double    ticks_per_meter = 0;
double   linear_correction_factor = 1.0;
double   angular_correction_factor = 1.0;
int32_t  wheel_mult[4] = {0};
float  wheel_track_cali = 0.3;

extern int16_t robot_odom[6];
extern int16_t robot_target_speed[3];

/**
  * @¼ò  Êö  »úÆ÷ÈËÔË¶¯²ÎÊıÉèÖÃ
  * @²Î  Êı  ÎŞ
  * @·µ»ØÖµ  ÎŞ
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
  * @¼ò  Êö  ÄæÏòÔË¶¯Ñ§½âÎö£¬µ×ÅÌÈıÖáËÙ¶È->ÂÖ×ÓËÙ¶È
  * @²Î  Êı  input:  robot_target_speed[]  »úÆ÷ÈËÈıÖáËÙ¶È m/s*1000
  *          output£ºax_encoder_delta_target[] µç»úÆÚÍûËÙ¶È count
  * @·µ»ØÖµ  ÎŞ
  */
void AX_Kinematics_Inverse(int16_t* input, int16_t* output)
{
	float x_speed   = ((float)input[0])/1000;//µ±Ç°µ×ÅÌxÖáËÙ¶È
	float y_speed   = ((float)input[1])/1000;//µ±Ç°µ×ÅÌyÖáËÙ¶È
	float yaw_speed = ((float)input[2])/1000;//µ±Ç°µ×ÅÌyawÖáËÙ¶È
	static float wheel_velocity[4] = {0};    //ÆÚÍûµÄËÄ¸öÂÖ×ÓÂÖ×ÓµÄËÙ¶È
	//(1)¸ù¾İµ±Ç°µ×ÅÌµÄÈıÖáËÙ¶È½âËã³öËÄ¸öÂÖ×ÓµÄËÙ¶È
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
  * @¼ò  Êö  ÕıÏòÔË¶¯Ñ§½âÎö£¬ÂÖ×Ó±àÂëÖµ->µ×ÅÌÈıÖáÀï³Ì¼Æ×ø±ê
  * @²Î  Êı  input: ax_encoder[]  ±àÂëÆ÷ÀÛ¼ÓÖµ
  *          output: robot_odom[] ÈıÖáÀï³Ì¼Æ x y yaw
  * @·µ»ØÖµ  ÎŞ
  */
void AX_Kinematics_Forward(int16_t* input, int16_t* output)
{
	static double delta_count[4];      //±àÂëÆ÷Êı¾İ
  static double delta_v_ave[3];      //ËÙ¶ÈÆ½¾ùÖµ
	static double delta_v_integral[2]; //Î»ÖÃÆ½¾ùÖµ
	static int16_t recv_count[4];
	
	recv_count[0] = -input[0];
	recv_count[1] = input[1];
	recv_count[2] = -input[2];
	recv_count[3] = input[3];
	
	//(1)±àÂëÆ÷¼ÆÊıÒç³ö´¦Àí,ÕıÏòÒç³ö¾ÍÊÇÕı×ªÒ»È¦£¬·´ÏòÒç³ö¾ÍÊÇ·´×ªÒ»È¦
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
	//(2)½«±àÂëÆ÷ÊıÖµ×ª»¯ÎªÇ°½øµÄ¾àÀë£¬ºÍÂÖ×ÓÖ±¾¶Ïà¹Ø£¬µ¥Î»m
	for(int i=0;i<4;i++)
	{	
		//¹«Ê½£ºÇ°½øµÄ¾àÀëÔöÁ¿=(µ±Ç°¼ÆÊıÖµ+È¦Êı*(×î´óµÄ¼ÆÊıÖµ-×îĞ¡µÄ¼ÆÊıÖµ)-ÉÏ´ÎÀÛ¼ÆµÄÈ¦Êı)/Ã¿¸ö¼ÆÊıÖµ¶ÔÓ¦µÄÃ×Ê
			delta_count[i] = 1.0*(recv_count[i] + wheel_mult[i]*(ENCODER_MAX-ENCODER_MIN)-current_count[i])/ticks_per_meter;
			current_count[i] = recv_count[i];
	}
		//(3)¼ÆËãµ×ÅÌxÖá±ä»¯¾àÀë¡¢YawÖá³¯Ïò±ä»¯rad½Ç¶È
	delta_v_ave[0] = (delta_count[3] - delta_count[2])/2.0;
	delta_v_ave[1] = (delta_count[2] - delta_count[0])/2.0;
	delta_v_ave[2] = (delta_count[2]+delta_count[1])/(2*wheel_track_cali);
		//(4)¼ÆËãµ×ÅÌ×ø±êÏµÏÂµÄxÖáÓëYawÖáµÄËÙ¶È
	delta_v_integral[0] = cos(delta_v_ave[2])*delta_v_ave[0] - sin(delta_v_ave[2])*delta_v_ave[1];
	delta_v_integral[1] = -sin(delta_v_ave[2])*delta_v_ave[0] - cos(delta_v_ave[2])*delta_v_ave[1];
   //(5)¼ÆËã»ı·Ö¼ÆËãÀï³Ì¼Æ×ø±êÏµ(odom_frame)ÏÂµÄ»úÆ÷ÈËX,Y,YawÖá×ø±ê
	output[0] += (int16_t)((cos((double)output[2]/1000)*delta_v_integral[0] - sin((double)output[2]/1000)*delta_v_integral[1])*1000);
	output[1] += (int16_t)((sin((double)output[2]/1000)*delta_v_integral[0] + cos((double)output[2]/1000)*delta_v_integral[1])*1000);
	output[2] += (int16_t)(delta_v_ave[2]*1000);
		
   //(6)YawÖá×ø±ê±ä»¯·¶Î§¿ØÖÆ-2¦° -> 2¦°
		if(output[2] > PI*1000)
			output[2] -= 2*PI*1000;
		else if(output[2] < -PI*1000)
			output[2] += 2*PI*1000;
		
		//(7)·¢ËÍ»úÆ÷ÈËXYÖáYawÖáËÙ¶È·´À¡
	output[3] = (int16_t)(delta_v_ave[0]*1000);
	output[4] = (int16_t)(-delta_v_ave[1]*1000);
	output[5] = (int16_t)(delta_v_ave[2]*1000);
}


/******************* (C) °æÈ¨ 2019 XTARK **************************************/
