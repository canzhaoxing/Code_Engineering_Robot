#ifndef IMU_TEMP_TASK_H
#define IMU_TEMP_TASK_H


extern float gyro[3], accel[3], temp;
extern float roll,pitch,yaw;

/**
  * @brief          bmi088 temperature control 
  * @param[in]      argument: NULL
  * @retval         none
  */
/**
  * @brief          bmi088ÎÂ¶È¿ØÖÆ
  * @param[in]      argument: NULL
  * @retval         none
  */
void imu_temp_control_task(void const * argument);
void INS_Task(void);
void Imu_Init(void);
#endif



