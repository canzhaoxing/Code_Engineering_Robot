#ifndef __RECEIVE_H
#define __RECEIVE_H

#include "usart.h"

#define big 0
#define small 1
#define D_L_OA 1.0f
#define D_L_AB 1.0f
#define D_L_BC 1.0f
#define D_L_CD 100.0f

union Receive_Typedef
{
	struct 
	{
		int16_t Pitch;
		int16_t Roll;
		int16_t Yaw;
		int16_t none;
	}channels;
	uint8_t DATA[8];
};

struct IMU_data_Typedef
{
	float Pitch;
	float Roll;
	float Yaw;
};
	
struct T_Matrix_Data_typdef
{
	float T_out[16];
	float P_in[3];
};


void Matrix_manipulation(void);
void T_Matrix(float* T_out, float Pitch, float Roll, float Yaw, float L);
void Matrixs_Init(void);

extern union Receive_Typedef Receive_Data[2];
extern uint8_t flag_CUP;

#endif
