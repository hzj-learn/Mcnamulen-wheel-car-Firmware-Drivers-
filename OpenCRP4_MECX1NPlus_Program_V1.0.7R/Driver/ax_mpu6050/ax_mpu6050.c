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
  * @��  ��  MPU6050 ����
  *
  ******************************************************************************
  * @˵  ��
  *
  * 1.MPU6050������ٶȴ�����������������
	* 2.IICͨ�Ų���IO��ģ�ⷽʽ
  *
  ******************************************************************************
  */

#include "ax_mpu6050.h" 
#include "ax_sys.h"
#include "ax_delay.h"

//IO��������
#define SDA_IN()  {GPIOB->CRH&=0XFFFFFF0F;GPIOB->CRH|=(u32)8<<4;}
#define SDA_OUT() {GPIOB->CRH&=0XFFFFFF0F;GPIOB->CRH|=(u32)3<<4;}

//IO��������	 
#define IIC_SCL    PBout(8) //SCL
#define IIC_SDA    PBout(9) //SDA	 
#define READ_SDA   PBin(9)  //����SDA 

static void IIC_Init(void);

/**
  * @��  ��  MPU6050д�Ĵ�����
  * @��  ��  ��	  
  * @����ֵ  ��
  */
static void MPU6050_WriteRegister(uint8_t reg_address, uint8_t data)
{
	MPU6050_I2C_Write(MPU6050_ADDR,reg_address,1,&data);
}

/**
  * @��  ��  ��ʼ��������ٶȺ����������ǣ�MPU6050����
  * @��  ��  ��	  
  * @����ֵ  ��
  */
static void MPU6050_ReadRegister(uint8_t reg_address, uint8_t *pdata, uint16_t len)
{
	MPU6050_I2C_Read(MPU6050_ADDR,reg_address,len,pdata);
}


/**
  * @��  ��  MPU6050��������ʼ��
  * @��  ��  ��	  
  * @����ֵ  ��
  */
void AX_MPU6050_Init(void)
{	
	IIC_Init();	//��ʼ��I2C2
	
	//����MPU6050�Ĵ���  
  MPU6050_WriteRegister(MPU6050_PWR_MGMT1_REG,0x00);     //�������״̬

	MPU6050_WriteRegister(MPU6050_GYRO_CFG_REG,0x18);      //���������� Ĭ��2000deg/s
  MPU6050_WriteRegister(MPU6050_ACCEL_CFG_REG,0x00);     //���ټ����� Ĭ��2g	
	
	MPU6050_WriteRegister(MPU6050_INTBP_CFG_REG,0x80);      //INT���ŵ͵�ƽ��Ч
	
	MPU6050_WriteRegister(MPU6050_PWR_MGMT1_REG,0x01);      //����CLKSEL,PLL X��Ϊ�ο�
	MPU6050_WriteRegister(MPU6050_PWR_MGMT2_REG,0x00); 	    //���ٶ��������Ƕ�����
	
	AX_Delayms(100);  //�ȴ��������ȶ�
}


/**
  * @��  ��  MPU6050���ü��ٶ����� 
  * @��  ��  range�� ���ٶ�����2g��4g��8g��16g
  *          �����õļ��ٶ�����ACC_RANGE_2G��ACC_RANGE_4G��ACC_RANGE_8G��ACC_RANGE_16G
  * @����ֵ  ��
  */
void AX_MPU6050_SetAccRange(uint8_t range)
{
	MPU6050_WriteRegister(MPU6050_ACCEL_CFG_REG,range<<3);
}


/**
  * @��  ��  MPU6050��������������
  * @��  ��  range ����������250��/S��500��/S��1000��/S��2000��/S
  *          �����õ�����������GYRO_RANGE_250��GYRO_RANGE_500��GYRO_RANGE_1000��GYRO_RANGE_2000	
  * @����ֵ  ��
  */
void AX_MPU6050_SetGyroRange(uint8_t range)
{
	MPU6050_WriteRegister(MPU6050_GYRO_CFG_REG,range<<3);
}

/**
  * @��  ��  MPU6050���������ǲ�����
  * @��  ��  smplrate �����ǲ����ʣ���Χ10~1000Hz	  
  * @����ֵ  ��
  */
void AX_MPU6050_SetGyroSmplRate(uint16_t smplrate)
{	
	if(smplrate>1000)
		smplrate = 1000;
	if(smplrate<10)
		smplrate = 10;
	
	MPU6050_WriteRegister(MPU6050_SAMPLE_RATE_REG,(uint8_t)(1000/smplrate -1));	
}

/**
  * @��  ��  MPU6050���õ�ͨ�˲�������
  * @��  ��  bandwidth ��ͨ�˲�������
  *          �����õĴ��� DLPF_ACC184_GYRO188��DLPF_ACC94_GYRO98��DLPF_ACC44_GYRO42��
  *                        DLPF_ACC21_GYRO20��DLPF_ACC10_GYRO10��DLPF_ACC5_GYRO5
  * @����ֵ  ��
  */
void AX_MPU6050_SetDLPF(uint8_t bandwidth)
{
	MPU6050_WriteRegister(MPU6050_CFG_REG,bandwidth);
}

/**
  * @��  ��  MPU6050��ȡ�������¶�ֵ
  * @��  ��  ��	  
  * @����ֵ  �������¶�ֵ��
  */
float AX_MPU6050_GetTempValue(void)
{	
	uint8_t buf[2];
	int16_t tmp;

	MPU6050_ReadRegister(MPU6050_TEMP_OUTH_REG,buf,2);

	tmp = (buf[0]<<8)| buf[1];
	
	return ( 36.53f + ((double)tmp/340.0f) );	
}

/**
  * @��  ��  MPU6050��ȡX����ٶȼĴ������ֵ
  * @��  ��  ��	  
  * @����ֵ  X����ٶȼĴ������ݡ�
  */
int16_t AX_MPU6050_GetAccData_X(void)
{
	uint8_t buf[2];
	
	MPU6050_ReadRegister(MPU6050_ACCEL_XOUTH_REG,buf,2);

	return ((buf[0]<<8) | buf[1]);	
}

/**
  * @��  ��  MPU6050��ȡY����ٶȼĴ������ֵ
  * @��  ��  ��	  
  * @����ֵ  Y����ٶȼĴ������ݡ�
  */
int16_t AX_MPU6050_GetAccData_Y(void)
{	
	uint8_t buf[2];

	MPU6050_ReadRegister(MPU6050_ACCEL_YOUTH_REG,buf,2);

	return ((buf[0]<<8) | buf[1]);	
}

/**
  * @��  ��  MPU6050��ȡZ����ٶȼĴ������ֵ
  * @��  ��  ��	  
  * @����ֵ  Z����ٶȼĴ������ݡ�
  */
int16_t AX_MPU6050_GetAccData_Z(void)
{	
	uint8_t buf[2];

	MPU6050_ReadRegister(MPU6050_ACCEL_ZOUTH_REG,buf,2);

	return ((buf[0]<<8) | buf[1]);		
}

/**
  * @��  ��  MPU6050��ȡ������ٶȼĴ������ֵ
  * @��  ��  pbuf����ȡ�����ݻ�����ָ�� 
  * @����ֵ  ��
  */
void AX_MPU6050_GetAccData(int16_t *pbuf)
{	
	uint8_t buf[6];
	
	MPU6050_ReadRegister(MPU6050_ACCEL_XOUTH_REG,buf,6);
	
    pbuf[0] = (buf[0] << 8) | buf[1];
    pbuf[1] = (buf[2] << 8) | buf[3];
    pbuf[2] = (buf[4] << 8) | buf[5];	
}


/**
  * @��  ��  MPU6050��ȡX�������ǼĴ������ֵ
  * @��  ��  ��	  
  * @����ֵ  X�������ǼĴ������ݡ�
  */
int16_t AX_MPU6050_GetGyroData_X(void)
{

	uint8_t buf[2];

	MPU6050_ReadRegister(MPU6050_GYRO_XOUTH_REG,buf,2);

	return ((buf[0]<<8) | buf[1]);		
}

/**
  * @��  ��  MPU6050��ȡY�������ǼĴ������ֵ
  * @��  ��  ��	  
  * @����ֵ  Y�������ǼĴ������ݡ�
  */
int16_t AX_MPU6050_GetGyroData_Y(void)
{	
	uint8_t buf[2];

	MPU6050_ReadRegister(MPU6050_GYRO_YOUTH_REG,buf,2);

	return ((buf[0]<<8) | buf[1]);
}

/**
  * @��  ��  MPU6050��ȡZ�������ǼĴ������ֵ
  * @��  ��  ��	  
  * @����ֵ  Z�������ǼĴ������ݡ�
  */
int16_t AX_MPU6050_GetGyroData_Z(void)
{	
	uint8_t buf[2];

	MPU6050_ReadRegister(MPU6050_GYRO_ZOUTH_REG,buf,2);

	return ((buf[0]<<8) | buf[1]);
}

/**
  * @��  ��  MPU6050��ȡ�����������ǼĴ������ֵ
  * @��  ��  pbuf����ȡ�����ݻ�����ָ�� 	  
  * @����ֵ  ��
  */
void AX_MPU6050_GetGyroData(int16_t *pbuf)
{	
	uint8_t buf[6];
	
	MPU6050_ReadRegister(MPU6050_GYRO_XOUTH_REG,buf,6);
	
    pbuf[0] = (buf[0] << 8) | buf[1];
    pbuf[1] = (buf[2] << 8) | buf[3];
    pbuf[2] = (buf[4] << 8) | buf[5];	
}										


//--------------------------I2C ��ʼ����������-----------------------------------------

/**************************ʵ�ֺ���********************************************
*����ԭ��:		void IIC_Init(void)
*��������:		��ʼ��I2C��Ӧ�Ľӿ����š�
*******************************************************************************/
static  void IIC_Init(void)
{			

	GPIO_InitTypeDef  GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE );	//ʹ��GPIOBʱ��

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP ;   //�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
    IIC_SCL=1;
    IIC_SDA=1;
	
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		void IIC_Start(void)
*��������:		����IIC��ʼ�ź�
*******************************************************************************/
static  int IIC_Start(void)
{
	SDA_OUT();     //sda�����
	IIC_SDA=1;
	if(!READ_SDA)return 0;	
	IIC_SCL=1;
	AX_Delayus(1);
 	IIC_SDA=0;//START:when CLK is high,DATA change form high to low 
	if(READ_SDA)return 0;
	AX_Delayus(1);
	IIC_SCL=0;//ǯסI2C���ߣ�׼�����ͻ�������� 
	return 1;
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		void IIC_Stop(void)
*��������:	    //����IICֹͣ�ź�
*******************************************************************************/	  
static  void IIC_Stop(void)
{
	SDA_OUT();//sda�����
	IIC_SCL=0;
	IIC_SDA=0;//STOP:when CLK is high DATA change form low to high
 	AX_Delayus(1);
	IIC_SCL=1; 
	IIC_SDA=1;//����I2C���߽����ź�
	AX_Delayus(1);							   	
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		u8 IIC_Wait_Ack(void)
*��������:	    �ȴ�Ӧ���źŵ��� 
//����ֵ��1������Ӧ��ʧ��
//        0������Ӧ��ɹ�
*******************************************************************************/
static  int IIC_Wait_Ack(void)
{
	u8 ucErrTime=0;
	SDA_IN();      //SDA����Ϊ����  
	IIC_SDA=1;
	AX_Delayus(1);	   
	IIC_SCL=1;
	AX_Delayus(1);	 
	while(READ_SDA)
	{
		ucErrTime++;
		if(ucErrTime>50)
		{
			IIC_Stop();
			return 0;
		}
	  AX_Delayus(1);
	}
	IIC_SCL=0;//ʱ�����0 	   
	return 1;  
} 

/**************************ʵ�ֺ���********************************************
*����ԭ��:		void IIC_Ack(void)
*��������:	    ����ACKӦ��
*******************************************************************************/
static  void IIC_Ack(void)
{
	IIC_SCL=0;
	SDA_OUT();
	IIC_SDA=0;
	AX_Delayus(1);
	IIC_SCL=1;
	AX_Delayus(1);
	IIC_SCL=0;
}
	
/**************************ʵ�ֺ���********************************************
*����ԭ��:		void IIC_NAck(void)
*��������:	    ����NACKӦ��
*******************************************************************************/	    
static  void IIC_NAck(void)
{
	IIC_SCL=0;
	SDA_OUT();
	IIC_SDA=1;
	AX_Delayus(1);
	IIC_SCL=1;
	AX_Delayus(1);
	IIC_SCL=0;
}
/**************************ʵ�ֺ���********************************************
*����ԭ��:		void IIC_Send_Byte(u8 txd)
*��������:	    IIC����һ���ֽ�
*******************************************************************************/		  
static  void IIC_Send_Byte(u8 txd)
{                        
    u8 t;   
	SDA_OUT(); 	    
    IIC_SCL=0;//����ʱ�ӿ�ʼ���ݴ���
    for(t=0;t<8;t++)
    {              
        IIC_SDA=(txd&0x80)>>7;
        txd<<=1; 	  
		AX_Delayus(1);   
		IIC_SCL=1;
		AX_Delayus(1); 
		IIC_SCL=0;	
		AX_Delayus(1);
    }	 
} 	 
  
/**************************ʵ�ֺ���********************************************
*����ԭ��:		u8 IIC_Read_Byte(unsigned char ack)
*��������:	    //��1���ֽڣ�ack=1ʱ������ACK��ack=0������nACK 
*******************************************************************************/  
static  u8 IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	SDA_IN();//SDA����Ϊ����
    for(i=0;i<8;i++ )
	{
        IIC_SCL=0; 
        AX_Delayus(2);
		IIC_SCL=1;
        receive<<=1;
        if(READ_SDA)receive++;   
		AX_Delayus(2); 
    }					 
    if (ack)
        IIC_Ack(); //����ACK 
    else
        IIC_NAck();//����nACK  
    return receive;
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		bool i2cWrite(uint8_t addr, uint8_t reg, uint8_t data)
*��������:		
*******************************************************************************/
uint8_t MPU6050_I2C_Write(uint8_t dev_addr, uint8_t reg_addr, uint8_t len, const uint8_t *data)
{
		int i;
    if (!IIC_Start())
        return 1;
    IIC_Send_Byte(dev_addr << 1 );
    if (!IIC_Wait_Ack()) {
        IIC_Stop();
        return 1;
    }
    IIC_Send_Byte(reg_addr);
    IIC_Wait_Ack();
		for (i = 0; i < len; i++) {
        IIC_Send_Byte(data[i]);
        if (!IIC_Wait_Ack()) {
            IIC_Stop();
            return 0;
        }
    }
    IIC_Stop();
    return 0;
}
/**************************ʵ�ֺ���********************************************
*����ԭ��:		bool i2cWrite(uint8_t addr, uint8_t reg, uint8_t data)
*��������:		
*******************************************************************************/
uint8_t MPU6050_I2C_Read(uint8_t dev_addr, uint8_t reg_addr, uint8_t len, uint8_t *data)
{
    if (!IIC_Start())
        return 1;
    IIC_Send_Byte(dev_addr << 1);
    if (!IIC_Wait_Ack()) {
        IIC_Stop();
        return 1;
    }
    IIC_Send_Byte(reg_addr);
    IIC_Wait_Ack();
    IIC_Start();
    IIC_Send_Byte((dev_addr << 1)+1);
    IIC_Wait_Ack();
    while (len) {
        if (len == 1)
            *data = IIC_Read_Byte(0);
        else
            *data = IIC_Read_Byte(1);
        data++;
        len--;
    }
    IIC_Stop();
    return 0;
}


/******************* (C) ��Ȩ 2018 XTARK **************************************/
