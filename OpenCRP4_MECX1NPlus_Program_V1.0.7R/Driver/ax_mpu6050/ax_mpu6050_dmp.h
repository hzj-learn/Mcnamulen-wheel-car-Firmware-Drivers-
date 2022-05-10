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
  * @日  期  2019-7-26
  * @内  容  MPU6050 DMP操作
  *
  ******************************************************************************
  */ 
  
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __AX_MPU6050_DMP_H
#define __AX_MPU6050_DMP_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"

//OpenCRP驱动接口函数
void AX_MPU6050_DMP_Init(void);    //MPU6050 DMP初始化
void AX_MPU6050_DMP_GetData(int16_t *data);  //MPU6050 MDP获取解算数据，姿态欧拉角

#endif

/******************* (C) 版权 2019 XTARK **************************************/
