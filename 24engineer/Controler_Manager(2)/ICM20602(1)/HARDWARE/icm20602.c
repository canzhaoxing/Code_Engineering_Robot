#include "icm20602.h"
#include "usart.h"
#include "tim.h"
 
//========ICM20602寄存器地址========================
/********************************************
*复位后所有寄存器地址都为0，除了
*Register 26  CONFIG				= 0x80
*Register 107 Power Management 1 	= 0x41
*Register 117 WHO_AM_I 				= 0x12
*********************************************/
//陀螺仪温度补偿
#define	ICM20_XG_OFFS_TC_H				0x04
#define	ICM20_XG_OFFS_TC_L				0x05
#define	ICM20_YG_OFFS_TC_H				0x07
#define	ICM20_YG_OFFS_TC_L				0x08
#define	ICM20_ZG_OFFS_TC_H				0x0A
#define	ICM20_ZG_OFFS_TC_L				0x0B
//加速度自检输出(出产时设置，用于与用户的自检输出值比较）
#define	ICM20_SELF_TEST_X_ACCEL			0x0D
#define	ICM20_SELF_TEST_Y_ACCEL			0x0E
#define	ICM20_SELF_TEST_Z_ACCEL			0x0F
//陀螺仪静态偏移
#define	ICM20_XG_OFFS_USRH				0x13
#define	ICM20_XG_OFFS_USRL				0x14
#define	ICM20_YG_OFFS_USRH				0x15
#define	ICM20_YG_OFFS_USRL				0x16
#define	ICM20_ZG_OFFS_USRH				0x17
#define	ICM20_ZG_OFFS_USRL				0x18
 
#define	ICM20_SMPLRT_DIV				0x19//设置采样率
#define	ICM20_CONFIG					0x1A//配置寄存器
#define	ICM20_GYRO_CONFIG				0x1B//设置陀螺仪量程
#define	ICM20_ACCEL_CONFIG				0x1C//设置加速度计量程
#define	ICM20_ACCEL_CONFIG2				0x1D//设置滤波
#define	ICM20_LP_MODE_CFG				0x1E//配置陀螺仪是否进入低功耗模式，本程序未用，即默认：不启用低功耗
 
//运动唤醒加速度阈值，未用
#define	ICM20_ACCEL_WOM_X_THR			0x20
#define	ICM20_ACCEL_WOM_Y_THR			0x21
#define	ICM20_ACCEL_WOM_Z_THR			0x22
 
//未用
#define	ICM20_FIFO_EN					0x23
#define	ICM20_FSYNC_INT					0x36
#define	ICM20_INT_PIN_CFG				0x37
//#define	ICM20_INT_ENABLE				0x38
#define	ICM20_FIFO_WM_INT_STATUS		0x39
#define	ICM20_INT_STATUS				0x3A
 
//加速度输出
#define	ICM20_ACCEL_XOUT_H				0x3B
#define	ICM20_ACCEL_XOUT_L				0x3C
#define	ICM20_ACCEL_YOUT_H				0x3D
#define	ICM20_ACCEL_YOUT_L				0x3E
#define	ICM20_ACCEL_ZOUT_H				0x3F
#define	ICM20_ACCEL_ZOUT_L				0x40
//温度输出
#define	ICM20_TEMP_OUT_H				0x41
#define	ICM20_TEMP_OUT_L				0x42
//角速度输出
#define	ICM20_GYRO_XOUT_H				0x43
#define	ICM20_GYRO_XOUT_L				0x44
#define	ICM20_GYRO_YOUT_H				0x45
#define	ICM20_GYRO_YOUT_L				0x46
#define	ICM20_GYRO_ZOUT_H				0x47
#define	ICM20_GYRO_ZOUT_L				0x48
//陀螺仪自检输出，未用
#define	ICM20_SELF_TEST_X_GYRO			0x50
#define	ICM20_SELF_TEST_Y_GYRO			0x51
#define	ICM20_SELF_TEST_Z_GYRO			0x52
 
#define	ICM20_FIFO_WM_TH1				0x60
#define	ICM20_FIFO_WM_TH2				0x61
#define	ICM20_SIGNAL_PATH_RESET			0x68
#define	ICM20_ACCEL_INTEL_CTRL 			0x69
#define	ICM20_USER_CTRL					0x6A
//电源控制
#define	ICM20_PWR_MGMT_1				0x6B
#define	ICM20_PWR_MGMT_2				0x6C
//未用
#define	ICM20_I2C_IF					0x70
#define	ICM20_FIFO_COUNTH				0x72
#define	ICM20_FIFO_COUNTL				0x73
#define	ICM20_FIFO_R_W					0x74
//未用
#define	ICM20_WHO_AM_I 					0x75
//加速度静态偏移，未用
#define	ICM20_XA_OFFSET_H				0x77
#define	ICM20_XA_OFFSET_L				0x78
#define	ICM20_YA_OFFSET_H				0x7A
#define	ICM20_YA_OFFSET_L				0x7B
#define	ICM20_ZA_OFFSET_H				0x7D
#define	ICM20_ZA_OFFSET_L 				0x7E
//===========================================================
 
extern SPI_HandleTypeDef hspi1;
 
static 	float _accel_scale;
static	float _gyro_scale;
 
static uint8_t tx, rx;
static uint8_t tx_buff[14];
//static uint8_t mpu_buff[14];
#define ICM20602_ADDRESS	0xD2
 
 
 
uint8_t icm20602_read_buffer(uint8_t const regAddr, uint8_t *pData, uint8_t len)
{
  HAL_GPIO_WritePin(CS_ICM_GPIO_Port, CS_ICM_Pin,GPIO_PIN_RESET);
  tx = regAddr | 0x80;
  tx_buff[0] = tx;
  HAL_SPI_TransmitReceive(&hspi1, &tx, &rx, 1, 55);
  HAL_SPI_TransmitReceive(&hspi1, tx_buff, pData, len, 55);
  HAL_GPIO_WritePin(CS_ICM_GPIO_Port, CS_ICM_Pin,GPIO_PIN_SET);
  return 0;
}
 
uint8_t icm20602_write_reg(uint8_t reg,uint8_t val)
{
 
	HAL_GPIO_WritePin(CS_ICM_GPIO_Port, CS_ICM_Pin,GPIO_PIN_RESET);
	tx = reg & 0x7F;
	HAL_SPI_TransmitReceive(&hspi1, &tx, &rx, 1, 55);
	tx = val;
	HAL_SPI_TransmitReceive(&hspi1, &tx, &rx, 1, 55);
	HAL_GPIO_WritePin(CS_ICM_GPIO_Port, CS_ICM_Pin,GPIO_PIN_SET);
	return 0;
}
 
 
uint8_t icm20602_read_reg(uint8_t reg)
{
	HAL_GPIO_WritePin(CS_ICM_GPIO_Port, CS_ICM_Pin,GPIO_PIN_RESET);
	tx = reg | 0x80;
	HAL_SPI_TransmitReceive(&hspi1, &tx, &rx, 1, 55);
	HAL_SPI_TransmitReceive(&hspi1, &tx, &rx, 1, 55);//这个应该是随便发一个数就行
	HAL_GPIO_WritePin(CS_ICM_GPIO_Port, CS_ICM_Pin,GPIO_PIN_SET);//源代码发送的是ff
	return rx;
}
 
uint8_t icm20602_init()
{
	if(icm20602_write_reg(ICM20_PWR_MGMT_1,0x80))	//复位，复位后位0x41,睡眠模式，
	{
		
//		printf("icm_20602 reset fail\r\n");
		return 1;
	}
	
	my_delay_ms(50);
	icm20602_write_reg(ICM20_PWR_MGMT_1,0x01);		//关闭睡眠，自动选择时钟
	my_delay_ms(50);
	
//	printf("icm_20602 id=%x\r\n",icm20602_read_reg(ICM20_WHO_AM_I));//读取ID
	
	
	icm20602_write_reg(ICM20_SMPLRT_DIV,0);			//分频数=为0+1，数据输出速率为内部采样速率,采样周期1KHZ
	icm20602_write_reg(ICM20_CONFIG,DLPF_BW_20);	//GYRO低通滤波设置
	icm20602_write_reg(ICM20_ACCEL_CONFIG2,ACCEL_AVER_4|ACCEL_DLPF_BW_21);	//ACCEL低通滤波设置
	
	
	icm20602_set_accel_fullscale(ICM20_ACCEL_FS_2G);
	icm20602_set_gyro_fullscale(ICM20_GYRO_FS_2000);
	
 
	my_delay_ms(100);
//	printf("icm20602 init pass\r\n\r\n");
	
	return 0;
}
 
 
 
 
 
uint8_t icm20602_set_gyro_fullscale(uint8_t fs)
{
	switch(fs)
	{
		case ICM20_GYRO_FS_250:
			_gyro_scale = 1.0f/131.068f;	//32767/250
		break;
		case ICM20_GYRO_FS_500:
			_gyro_scale = 1.0f/65.534f;
		break;
		case ICM20_GYRO_FS_1000:
			_gyro_scale = 1.0f/32.767f;
		break;
		case ICM20_GYRO_FS_2000:
			_gyro_scale = 1.0f/16.4f;
		break;
		default:
			fs = ICM20_GYRO_FS_2000;
			_gyro_scale = 1.0f/16.3835f;
		break;
 
	}
	
	return icm20602_write_reg(ICM20_GYRO_CONFIG,fs);
	
}
 
 
uint8_t icm20602_set_accel_fullscale(uint8_t fs)
{
	switch(fs)
	{
		case ICM20_ACCEL_FS_2G:
			_accel_scale = 1.0f/16348.0f;
		break;
		case ICM20_ACCEL_FS_4G:
			_accel_scale = 1.0f/8192.0f;
		break;
		case ICM20_ACCEL_FS_8G:
			_accel_scale = 1.0f/4096.0f;
		break;
		case ICM20_ACCEL_FS_16G:
			_accel_scale = 1.0f/2048.0f;
		break;
		default:
			fs = ICM20_ACCEL_FS_8G;
			_accel_scale = 1.0f/4096.0f;
		break;
 
	}
	return icm20602_write_reg(ICM20_ACCEL_CONFIG,fs);
}
 
 
 
//加速度原始数据（ADC）
uint8_t icm20602_get_accel_adc(short *ax,short *ay,short *az)
{
	uint8_t buf[6];
	if(icm20602_read_buffer(ICM20_ACCEL_XOUT_H,buf,6))return 1;
	
	*ax = ((int16_t)buf[0]<<8) | buf[1];
	*ay = ((int16_t)buf[2]<<8) | buf[3];
	*az = ((int16_t)buf[4]<<8) | buf[5];
	return 0;
}




 
//陀螺仪原始数据（ADC）
uint8_t icm20602_get_gyro_adc(short *gx,short *gy,short *gz)
{
	uint8_t buf[6];
	if(icm20602_read_buffer(ICM20_GYRO_XOUT_H,buf,6))return 1;
	*gx = ((int16_t)buf[0]<<8) | buf[1];
	*gy = ((int16_t)buf[2]<<8) | buf[3];
	*gz = ((int16_t)buf[4]<<8) | buf[5];
	return 0;
}

//获取出厂校准值
uint8_t icm20602_get_gyro_adc_chuchang(short *gx,short *gy,short *gz)
{
	uint8_t buf[6];
	if(icm20602_read_buffer(ICM20_SELF_TEST_X_GYRO,buf,6))return 1;
	*gx = (buf[0]<<8) | buf[1];
	*gy = (buf[2]<<8) | buf[3];
	*gz = (buf[4]<<8) | buf[5];
	return 0;
}


//陀螺仪计算值
uint8_t icm20602_get_gyro(float *gyro)
{
	int16_t gyro_adc[3];
	if(icm20602_get_gyro_adc(&gyro_adc[0],&gyro_adc[1],&gyro_adc[2]))return 1;
	
	gyro[0] = _gyro_scale * gyro_adc[0];        //X轴陀螺仪
	gyro[1] = _gyro_scale * gyro_adc[1];        //Y轴陀螺仪
	gyro[2] = _gyro_scale * gyro_adc[2];	    //Z轴陀螺仪
	return 0;
}
//加速度计算值
uint8_t icm20602_get_accel(float *accel)
{
	int16_t accel_adc[3];
	if(icm20602_get_accel_adc(&accel_adc[0],&accel_adc[1],&accel_adc[2]))return 1;
	accel[0] = _accel_scale * accel_adc[0];       //X轴加速度
	accel[1] = _accel_scale * accel_adc[1];		  //Y轴加速度
	accel[2] = _accel_scale * accel_adc[2];	      //Z轴加速度
	return 0;
}
 
float icm20602_get_temp()
{
	int16_t temp_adc;
	uint8_t buf[2];
	if(icm20602_read_buffer(ICM20_TEMP_OUT_H,buf,2))return 0.0f;
 
	temp_adc = (buf[0]<<8)+buf[1];
 
	return (25.0f + (float)temp_adc/326.8f);
}
