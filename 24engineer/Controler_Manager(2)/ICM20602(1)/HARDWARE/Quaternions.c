#include "Quaternions.h"

#define delta_T      0.005f  //5ms计算一次

float I_ex, I_ey, I_ez;  // 误差积分
struct Q_info_Typedef Q_info;  // 全局四元数
struct eulerAngle_Typedef eulerAngle; //欧拉角
float param_Kp = 50.0;   // 加速度计(磁力计)的收敛速率比例增益50 
float param_Ki = 0.20;   //陀螺仪收敛速率的积分增益 0.2

float invSqrt(float x) {
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}

void IMU_AHRSupdate_noMagnetic(float gx_f, float gy_f, float gz_f, float ax_f, float ay_f, float az_f)
{
    float halfT = 0.5 * delta_T;
    float vx, vy, vz;    //当前的机体坐标系上的重力单位向量
    float ex, ey, ez;    //四元数计算值与加速度计测量值的误差
    float q0 = Q_info.q0;
    float q1 = Q_info.q1;
    float q2 = Q_info.q2;
    float q3 = Q_info.q3;
    float q0q0 = q0 * q0;
    float q0q1 = q0 * q1;
    float q0q2 = q0 * q2;
    float q0q3 = q0 * q3;
    float q1q1 = q1 * q1;
    float q1q2 = q1 * q2;
    float q1q3 = q1 * q3;
    float q2q2 = q2 * q2;
    float q2q3 = q2 * q3;
    float q3q3 = q3 * q3;
    float delta_2 = 0;
	
	//对加速度数据进行归一化 得到单位加速度
    float norm = invSqrt(ax_f*ax_f + ay_f*ay_f + az_f*az_f);       
    ax_f = ax_f * norm;
    ay_f = ay_f * norm;
    az_f = az_f * norm;
	
	vx = 2*(q1q3 - q0q2);
    vy = 2*(q0q1 + q2q3);
    vz = q0q0 - q1q1 - q2q2 + q3q3;
	
	ex = ay_f * vz - az_f * vy;
    ey = az_f * vx - ax_f * vz;
    ez = ax_f * vy - ay_f * vx;
	
	//用叉乘误差来做PI修正陀螺零偏，
    //通过调节 param_Kp，param_Ki 两个参数，
    //可以控制加速度计修正陀螺仪积分姿态的速度。
    I_ex += delta_T * ex;   // integral error scaled by Ki
    I_ey += delta_T * ey;
    I_ez += delta_T * ez;
 
    gx_f = gx_f+ param_Kp*ex + param_Ki*I_ex;
    gy_f = gy_f+ param_Kp*ey + param_Ki*I_ey;
    gz_f = gz_f+ param_Kp*ez + param_Ki*I_ez;
	
	//四元数微分方程，其中halfT为测量周期的1/2，gx gy gz为陀螺仪角速度，以下都是已知量，这里使用了一阶龙哥库塔求解四元数微分方程
    q0 = q0 + (-q1*gx_f - q2*gy_f - q3*gz_f)*halfT;
    q1 = q1 + ( q0*gx_f + q2*gz_f - q3*gy_f)*halfT;
    q2 = q2 + ( q0*gy_f - q1*gz_f + q3*gx_f)*halfT;
    q3 = q3 + ( q0*gz_f + q1*gy_f - q2*gx_f)*halfT;
	
	delta_2=(2*halfT*gx_f)*(2*halfT*gx_f)+(2*halfT*gy_f)*(2*halfT*gy_f)+(2*halfT*gz_f)*(2*halfT*gz_f);
    // 整合四元数率    四元数微分方程  四元数更新算法，二阶毕卡法
    q0 = (1-delta_2/8)*q0 + (-q1*gx_f - q2*gy_f - q3*gz_f)*halfT;			
    q1 = (1-delta_2/8)*q1 + (q0*gx_f + q2*gz_f - q3*gy_f)*halfT;
    q2 = (1-delta_2/8)*q2 + (q0*gy_f - q1*gz_f + q3*gx_f)*halfT;
    q3 = (1-delta_2/8)*q3 + (q0*gz_f + q1*gy_f - q2*gx_f)*halfT;
 
    // normalise quaternion
    norm = invSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    Q_info.q0 = q0 * norm;
    Q_info.q1 = q1 * norm;
    Q_info.q2 = q2 * norm;
    Q_info.q3 = q3 * norm;
	
}
