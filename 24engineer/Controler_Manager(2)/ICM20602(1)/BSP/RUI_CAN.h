/*
 * @Descripttion: 
 * @version: 
 * @Author: Eugene
 * @Date: 2023-12-19 18:35:41
 * @LastEditors: Andy
 * @LastEditTime: 2024-02-15 11:38:24
 */
/*
____/\\\\\\\\\_____        __/\\\________/\\\_        __/\\\\\\\\\\\_
 __/\\\///////\\\___        _\/\\\_______\/\\\_        _\/////\\\///__
  _\/\\\_____\/\\\___        _\/\\\_______\/\\\_        _____\/\\\_____
   _\/\\\\\\\\\\\/____        _\/\\\_______\/\\\_        _____\/\\\_____
    _\/\\\//////\\\____        _\/\\\_______\/\\\_        _____\/\\\_____
     _\/\\\____\//\\\___        _\/\\\_______\/\\\_        _____\/\\\_____
      _\/\\\_____\//\\\__        _\//\\\______/\\\__        _____\/\\\_____
       _\/\\\______\//\\\_        __\///\\\\\\\\\/___        __/\\\\\\\\\\\_
        _\///________\///__        ____\/////////_____        _\///////////__
*/
#ifndef RUI_CAN_H
#define RUI_CAN_H

#include "can.h"
#include "string.h"

union Angle_Aim_Typdef // 使用共用体整合数据
{
    struct
    {
        int16_t Pitch;
		int16_t Roll;
		int16_t Yaw;
		int16_t none;
    } DataNeaten;
    // 接收到的数组
    uint8_t  Data[8];
};


void CAN_Filter_Init(void);
//电机can发送
void RUI_F_CAN_SEDN(CAN_HandleTypeDef* _hcan , int16_t stdid , int16_t num1 , int16_t num2 , int16_t num3 , int16_t num4);

//共用体发送
void RUI_F_CAN_SEDN_UNION(CAN_HandleTypeDef* _hcan , int16_t stdid , uint8_t* Data);

extern union Angle_Aim_Typdef Angle_Aim;
extern union Angle_Aim_Typdef Angle_Transmit;

#endif
