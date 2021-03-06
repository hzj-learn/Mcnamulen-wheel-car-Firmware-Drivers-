/**			                                                    
		   ____                    _____ _____  _____        XTARK@???˴???
		  / __ \                  / ____|  __ \|  __ \  
		 | |  | |_ __   ___ _ __ | |    | |__) | |__) |
		 | |  | | '_ \ / _ \ '_ \| |    |  _  /|  ___/ 
		 | |__| | |_) |  __/ | | | |____| | \ \| |     
		  \____/| .__/ \___|_| |_|\_____|_|  \_\_|     
		    		| |                                    
		    		|_|  OpenCRP ??ݮ?? ר??ROS?????˿?????                                   
									 
  ****************************************************************************** 
  *           
  * ??Ȩ???У? XTARK@???˴???  ??Ȩ???У??????ؾ?
  * ??????վ?? www.xtark.cn
  * ?Ա????̣? https://shop246676508.taobao.com  
  * ????ý?壺 www.cnblogs.com/xtark?????ͣ?
	* ????΢?ţ? ΢?Ź??ںţ????˴??£???ȡ??????Ѷ??
  *                              
  ******************************************************************************
  * @??  ??  Musk Han@XTARK
  * @??  ??  V1.0
  * @??  ??  2019-7-26
  * @??  ??  MPU6050 ????
  *
  ******************************************************************************
  */
  
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __AX_MPU6050_H
#define __AX_MPU6050_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"

#define	MPU6050_ADDR    0x68    //MPU6050??ַ

//MPU6050?Ĵ?????ַ
#define MPU6050_SELF_TESTX_REG		0X0D	//?Լ??Ĵ???X
#define MPU6050_SELF_TESTY_REG		0X0E	//?Լ??Ĵ???Y
#define MPU6050_SELF_TESTZ_REG		0X0F	//?Լ??Ĵ???Z
#define MPU6050_SELF_TESTA_REG		0X10	//?Լ??Ĵ???A
#define MPU6050_SAMPLE_RATE_REG		0X19	//????Ƶ?ʷ?Ƶ??
#define MPU6050_CFG_REG				0X1A	//???üĴ???
#define MPU6050_GYRO_CFG_REG		0X1B	//?????????üĴ???
#define MPU6050_ACCEL_CFG_REG		0X1C	//???ٶȼ????üĴ???
#define MPU6050_MOTION_DET_REG		0X1F	//?˶????ֵⷧ???üĴ???
#define MPU6050_FIFO_EN_REG			0X23	//FIFOʹ?ܼĴ???
#define MPU6050_I2CMST_CTRL_REG		0X24	//IIC???????ƼĴ???
#define MPU6050_I2CSLV0_ADDR_REG	0X25	//IIC?ӻ?0??????ַ?Ĵ???
#define MPU6050_I2CSLV0_REG			0X26	//IIC?ӻ?0???ݵ?ַ?Ĵ???
#define MPU6050_I2CSLV0_CTRL_REG	0X27	//IIC?ӻ?0???ƼĴ???
#define MPU6050_I2CSLV1_ADDR_REG	0X28	//IIC?ӻ?1??????ַ?Ĵ???
#define MPU6050_I2CSLV1_REG			0X29	//IIC?ӻ?1???ݵ?ַ?Ĵ???
#define MPU6050_I2CSLV1_CTRL_REG	0X2A	//IIC?ӻ?1???ƼĴ???
#define MPU6050_I2CSLV2_ADDR_REG	0X2B	//IIC?ӻ?2??????ַ?Ĵ???
#define MPU6050_I2CSLV2_REG			0X2C	//IIC?ӻ?2???ݵ?ַ?Ĵ???
#define MPU6050_I2CSLV2_CTRL_REG	0X2D	//IIC?ӻ?2???ƼĴ???
#define MPU6050_I2CSLV3_ADDR_REG	0X2E	//IIC?ӻ?3??????ַ?Ĵ???
#define MPU6050_I2CSLV3_REG			0X2F	//IIC?ӻ?3???ݵ?ַ?Ĵ???
#define MPU6050_I2CSLV3_CTRL_REG	0X30	//IIC?ӻ?3???ƼĴ???
#define MPU6050_I2CSLV4_ADDR_REG	0X31	//IIC?ӻ?4??????ַ?Ĵ???
#define MPU6050_I2CSLV4_REG			0X32	//IIC?ӻ?4???ݵ?ַ?Ĵ???
#define MPU6050_I2CSLV4_DO_REG		0X33	//IIC?ӻ?4д???ݼĴ???
#define MPU6050_I2CSLV4_CTRL_REG	0X34	//IIC?ӻ?4???ƼĴ???
#define MPU6050_I2CSLV4_DI_REG		0X35	//IIC?ӻ?4?????ݼĴ???
#define MPU6050_I2CMST_STA_REG		0X36	//IIC????״̬?Ĵ???
#define MPU6050_INTBP_CFG_REG		0X37	//?ж?/??·???üĴ???
#define MPU6050_INT_EN_REG			0X38	//?ж?ʹ?ܼĴ???
#define MPU6050_INT_STA_REG			0X3A	//?ж?״̬?Ĵ???
#define MPU6050_ACCEL_XOUTH_REG		0X3B	//???ٶ?ֵ,X????8λ?Ĵ???
#define MPU6050_ACCEL_XOUTL_REG		0X3C	//???ٶ?ֵ,X????8λ?Ĵ???
#define MPU6050_ACCEL_YOUTH_REG		0X3D	//???ٶ?ֵ,Y????8λ?Ĵ???
#define MPU6050_ACCEL_YOUTL_REG		0X3E	//???ٶ?ֵ,Y????8λ?Ĵ???
#define MPU6050_ACCEL_ZOUTH_REG		0X3F	//???ٶ?ֵ,Z????8λ?Ĵ???
#define MPU6050_ACCEL_ZOUTL_REG		0X40	//???ٶ?ֵ,Z????8λ?Ĵ???
#define MPU6050_TEMP_OUTH_REG		0X41	//?¶?ֵ?߰?λ?Ĵ???
#define MPU6050_TEMP_OUTL_REG		0X42	//?¶?ֵ??8λ?Ĵ???
#define MPU6050_GYRO_XOUTH_REG		0X43	//??????ֵ,X????8λ?Ĵ???
#define MPU6050_GYRO_XOUTL_REG		0X44	//??????ֵ,X????8λ?Ĵ???
#define MPU6050_GYRO_YOUTH_REG		0X45	//??????ֵ,Y????8λ?Ĵ???
#define MPU6050_GYRO_YOUTL_REG		0X46	//??????ֵ,Y????8λ?Ĵ???
#define MPU6050_GYRO_ZOUTH_REG		0X47	//??????ֵ,Z????8λ?Ĵ???
#define MPU6050_GYRO_ZOUTL_REG		0X48	//??????ֵ,Z????8λ?Ĵ???
#define MPU6050_I2CSLV0_DO_REG		0X63	//IIC?ӻ?0???ݼĴ???
#define MPU6050_I2CSLV1_DO_REG		0X64	//IIC?ӻ?1???ݼĴ???
#define MPU6050_I2CSLV2_DO_REG		0X65	//IIC?ӻ?2???ݼĴ???
#define MPU6050_I2CSLV3_DO_REG		0X66	//IIC?ӻ?3???ݼĴ???
#define MPU6050_I2CMST_DELAY_REG	0X67	//IIC??????ʱ?????Ĵ???
#define MPU6050_SIGPATH_RST_REG		0X68	//?ź?ͨ????λ?Ĵ???
#define MPU6050_MDETECT_CTRL_REG	0X69	//?˶????????ƼĴ???
#define MPU6050_USER_CTRL_REG		0X6A	//?û????ƼĴ???
#define MPU6050_PWR_MGMT1_REG		0X6B	//??Դ?????Ĵ???1
#define MPU6050_PWR_MGMT2_REG		0X6C	//??Դ?????Ĵ???2
#define MPU6050_FIFO_CNTH_REG		0X72	//FIFO?????Ĵ????߰?λ
#define MPU6050_FIFO_CNTL_REG		0X73	//FIFO?????Ĵ????Ͱ?λ
#define MPU6050_FIFO_RW_REG			0X74	//FIFO??д?Ĵ???
#define MPU6050_DEVICE_ID_REG		0X75	//????ID?Ĵ???

//???ٶ?????
#define  AX_ACC_RANGE_2G  0  //2g	 
#define  AX_ACC_RANGE_4G  1  //4g
#define  AX_ACC_RANGE_8G  2  //8g	 
#define  AX_ACC_RANGE_16G 3  //16g

//??????????
#define  AX_GYRO_RANGE_250  0  //250??/??	 
#define  AX_GYRO_RANGE_500  1  //500??/??
#define  AX_GYRO_RANGE_1000 2  //1000??/??	 
#define  AX_GYRO_RANGE_2000 3  //2000??/??	 

//????
#define  AX_DLPF_ACC184_GYRO188 1 //???ٶȴ???184Hz ?????Ǵ???188Hz
#define  AX_DLPF_ACC94_GYRO98   2 //???ٶȴ???94Hz ?????Ǵ???98Hz
#define  AX_DLPF_ACC44_GYRO42   3 //???ٶȴ???44Hz ?????Ǵ???42Hz
#define  AX_DLPF_ACC21_GYRO20   4 //???ٶȴ???21Hz ?????Ǵ???20Hz
#define  AX_DLPF_ACC10_GYRO10   5 //???ٶȴ???10Hz ?????Ǵ???10Hz
#define  AX_DLPF_ACC5_GYRO5     6 //???ٶȴ???5Hz ?????Ǵ???5Hz 

uint8_t MPU6050_I2C_Write(uint8_t dev_addr, uint8_t reg_addr, uint8_t len, const uint8_t *data);
uint8_t MPU6050_I2C_Read(uint8_t dev_addr, uint8_t reg_addr, uint8_t len, uint8_t *data);

/*** OpenCRP?????ӿں???  M-Sensor?˶??????????????? **********/
void AX_MPU6050_Init(void);    //MPU6050????????ʼ??

//???ò???????
void AX_MPU6050_SetAccRange(uint8_t range);  //MPU6050???ü??ٶ????? 
void AX_MPU6050_SetGyroRange(uint8_t range);  //MPU6050??????????????
void AX_MPU6050_SetGyroSmplRate(uint16_t smplrate);  //MPU6050?????????ǲ?????
void AX_MPU6050_SetDLPF(uint8_t bandwidth);  //MPU6050???õ?ͨ?˲???????

//??ȡ???ݺ???
float AX_MPU6050_GetTempValue(void);	//MPU6050??ȡ???????¶?ֵ
int16_t AX_MPU6050_GetAccData_X(void);    //MPU6050??ȡX?????ٶȼĴ???????ֵ
int16_t AX_MPU6050_GetAccData_Y(void);    //MPU6050??ȡY?????ٶȼĴ???????ֵ
int16_t AX_MPU6050_GetAccData_Z(void);    //MPU6050??ȡZ?????ٶȼĴ???????ֵ
void AX_MPU6050_GetAccData(int16_t *pbuf);   //MPU6050??ȡ???????ٶȼĴ???????ֵ
int16_t AX_MPU6050_GetGyroData_X(void);   //MPU6050??ȡX???????ǼĴ???????ֵ
int16_t AX_MPU6050_GetGyroData_Y(void);   //MPU6050??ȡY???????ǼĴ???????ֵ
int16_t AX_MPU6050_GetGyroData_Z(void);   //MPU6050??ȡZ???????ǼĴ???????ֵ
void AX_MPU6050_GetGyroData(int16_t *pbuf);  //MPU6050??ȡ???????????ǼĴ???????ֵ

#endif

/******************* (C) ??Ȩ 2018 XTARK **************************************/
