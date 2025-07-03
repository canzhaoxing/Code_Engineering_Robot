#ifndef __QUATERNIONS_H
#define __QUATERNIONS_H

struct eulerAngle_Typedef
{
	float pitch;
	float roll ;
	float yaw  ;
};

struct Q_info_Typedef
{
	float q0;
	float q1;
	float q2;
	float q3;
};

void IMU_AHRSupdate_noMagnetic(float gx_f, float gy_f, float gz_f, float ax_f, float ay_f, float az_f);

extern struct Q_info_Typedef Q_info;  // 全局四元数
extern struct eulerAngle_Typedef eulerAngle; //欧拉角

#endif
