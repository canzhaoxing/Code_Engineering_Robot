#include "Receive.h"
#include "string.h"
#include "arm_math.h"
#include "ins_task.h"

#define DATA_D {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1}

union Receive_Typedef Receive_Data[2];//陀螺仪数据接收联合体
struct IMU_data_Typedef IMU_data[2];//陀螺仪数据处理结构体

struct T_Matrix_Data_typdef T_Matrix_Data[4];//变换矩阵数据源
float S_out_f[3][16] = {DATA_D,DATA_D,DATA_D};//运算过程中的中间矩阵数据源
float OUT_f[4] = {1,1,1,1};//运算结果输出数组数据源
float IN_f[4] = {1,1,1,1};// 运算输入数组数据源

arm_matrix_instance_f32 S_in[4];//变换矩阵（T矩阵）
arm_matrix_instance_f32 S_out[3];//运算过程中的中间矩阵
arm_matrix_instance_f32 OUT;//运算输出矩阵
arm_matrix_instance_f32 IN;//运算输入矩阵

uint8_t flag_CUP = 0xf0;
uint8_t flaag = 0;

void Matrixs_Init(void)
{
	/*四个T矩阵初始化*/
	arm_mat_init_f32(&S_in[0],4,4,T_Matrix_Data[0].T_out);
	arm_mat_init_f32(&S_in[1],4,4,T_Matrix_Data[1].T_out);
	arm_mat_init_f32(&S_in[2],4,4,T_Matrix_Data[2].T_out);
	arm_mat_init_f32(&S_in[3],4,4,T_Matrix_Data[3].T_out);
	/*三个中间矩阵初始化*/
	arm_mat_init_f32(&S_out[0],4,4,S_out_f[0]);
	arm_mat_init_f32(&S_out[1],4,4,S_out_f[1]);
	arm_mat_init_f32(&S_out[2],4,4,S_out_f[2]);
	/*输入输出矩阵初始化*/
	arm_mat_init_f32(&IN,4,1,IN_f);
	arm_mat_init_f32(&OUT,4,1,OUT_f);
}

void Matrix_manipulation(void)
{
	IMU_data[0].Pitch = 0.0f;//((float)Receive_Data[0].channels.Pitch)/100;
	IMU_data[0].Roll  = 0.0f;//((float)Receive_Data[0].channels.Roll)/100;
	IMU_data[0].Yaw   = 0.0f;//((float)Receive_Data[0].channels.Yaw)/100;
	
	IMU_data[1].Pitch = 0.0f;//((float)Receive_Data[1].channels.Pitch)/100;
	IMU_data[1].Roll  = 0.0f;//((float)Receive_Data[1].channels.Roll)/100;
	IMU_data[1].Yaw   = 0.0f;//((float)Receive_Data[1].channels.Yaw)/100;
	
	/*四个T矩阵源数组数据更新*/
	T_Matrix(T_Matrix_Data[0].T_out,0.0f,0.0f,90.0f,D_L_OA);
	
	T_Matrix(T_Matrix_Data[1].T_out,
			IMU_data[big].Pitch,
			IMU_data[big].Roll,
			IMU_data[big].Yaw,
			D_L_AB);
	
	T_Matrix(T_Matrix_Data[2].T_out,
			IMU_data[1].Pitch - IMU_data[big].Pitch,
			IMU_data[1].Roll  - IMU_data[big].Roll,
			IMU_data[1].Yaw   - IMU_data[big].Yaw,
			D_L_BC);
			
	T_Matrix(T_Matrix_Data[3].T_out,
			INS.Pitch - IMU_data[1].Pitch,
			INS.Roll - IMU_data[1].Roll,
			INS.YawTotalAngle - IMU_data[1].Yaw,
			D_L_CD);
	/*四个T矩阵相乘*/
	arm_mat_mult_f32(&S_in[0],&S_in[1],&S_out[0]);
	arm_mat_mult_f32(&S_out[0],&S_in[2],&S_out[1]);
	arm_mat_mult_f32(&S_out[1],&S_in[3],&S_out[2]);
	/*输出矩阵=总的T矩阵*输入矩阵*/
	arm_mat_mult_f32(&S_out[2],&IN,&OUT);
}

//变换矩阵源数组数据更新
void T_Matrix(float* T_out, float Pitch, float Roll, float Yaw, float L)
{
	Pitch = Pitch*PI/180.0f;
	Roll = Roll*PI/180.0f;
	Yaw = Yaw*PI/180.0f;
	
	T_out[0] = arm_cos_f32(Yaw)*arm_cos_f32(Pitch)-arm_sin_f32(Roll)*arm_sin_f32(Yaw)*arm_sin_f32(Pitch);
	T_out[1] = -arm_sin_f32(Yaw)*arm_cos_f32(Roll);
	T_out[2] = arm_cos_f32(Yaw)*arm_sin_f32(Pitch)+arm_sin_f32(Yaw)*arm_sin_f32(Roll)*arm_cos_f32(Pitch);
	
	T_out[3] = L*arm_cos_f32(Pitch)*arm_cos_f32(Yaw);
	
	T_out[4] = arm_sin_f32(Yaw)*arm_cos_f32(Pitch)-arm_cos_f32(Yaw)*arm_sin_f32(Roll)*arm_sin_f32(Pitch);
	T_out[5] = -arm_cos_f32(Yaw)*arm_cos_f32(Roll);
	T_out[6] = arm_sin_f32(Yaw)*arm_sin_f32(Pitch)+arm_cos_f32(Yaw)*arm_sin_f32(Roll)*arm_cos_f32(Pitch);
	
	T_out[7] = L*arm_cos_f32(Pitch)*arm_sin_f32(Yaw);
	
	T_out[8] = -arm_cos_f32(Roll)*arm_sin_f32(Pitch);
	T_out[9] = arm_sin_f32(Roll);
	T_out[10]= arm_cos_f32(Roll)*arm_cos_f32(Pitch);
	
	T_out[11]= L*arm_sin_f32(Pitch);
	
	T_out[12]= 0;
	T_out[13]= 0;
	T_out[14]= 0; 
	T_out[15]= 1;
}


/*
陀螺仪编号： 		大臂 -> 0
				小臂 -> 1
				手掌 -> 无
变换矩阵编号：	胸口到肩关节  ->	0,L_OA
				肩关节到肘关节->	1,L_AB
				肘关节到腕关节->	2,L_BC
				腕关节到手掌心->	3,L_CD

*/
