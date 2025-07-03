#include "head_task.h"
#include "MY_define.h"
#include "can.h"
#include "RUI_CAN.h"
#include "Quaternions.h"

uint16_t t = 0;

uint8_t heading(uint8_t DBUS_STATUS)
{	
	Angle_Transmit.DataNeaten.Pitch = (int16_t)(eulerAngle.pitch*100);
	Angle_Transmit.DataNeaten.Roll  = (int16_t)(eulerAngle.roll*100);
	Angle_Transmit.DataNeaten.Yaw	= (int16_t)(eulerAngle.yaw*100);
	RUI_F_CAN_SEDN_UNION(&hcan1,0x051,Angle_Transmit.Data);
	
	t++;
	if(t==500)	{t=0;	LED_TURN;}
	
	
	return RUI_DF_READY;
}
