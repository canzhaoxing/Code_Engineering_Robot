#include "filter.h"


//ÏÞ·ùÂË²¨
#define Error_MAX_Pitch 140.0f
#define Error_MAX_Yaw   160.0f
Filter_param LimitFilter;
void FilterLimit()//ÏÞ·ù
{
	if(Final_Data.Pitch_gyro > Error_MAX_Pitch)
		Final_Data.Pitch_gyro = Error_MAX_Pitch;
	else if(Final_Data.Pitch_gyro < -Error_MAX_Pitch)
		Final_Data.Pitch_gyro = -Error_MAX_Pitch;
	
	if(Final_Data.Yaw_gyro > Error_MAX_Yaw)
		Final_Data.Yaw_gyro = Error_MAX_Yaw;
	else if(Final_Data.Yaw_gyro < -Error_MAX_Yaw)
		Final_Data.Yaw_gyro = -Error_MAX_Yaw;
}


#define error_observe 0.0f
Filter_param My_fliter;
void my_filter()
{
	My_fliter.now[0] = Final_Data.Yaw_gyro2;
	My_fliter.now[1] = Final_Data.Pitch_gyro2;
	if((My_fliter.now[0] - My_fliter.last[0]) > error_observe || (My_fliter.last[0] - My_fliter.now[0]) > error_observe)
	{
		Final_Data.Pitch_gyro = Final_Data.Pitch_gyro2 * 0.2f;
		Final_Data.Yaw_gyro = Final_Data.Yaw_gyro2;
	}
	if((My_fliter.now[1] - My_fliter.last[1]) > error_observe || (My_fliter.last[1] - My_fliter.now[1]) > error_observe)
	{
		Final_Data.Yaw_gyro = Final_Data.Yaw_gyro2;
	}
	My_fliter.last[0] = My_fliter.now[0];
	My_fliter.last[1] = My_fliter.now[1];
}



