#include "Sensor.h"
#include "kalanman.h"

float JY_Yaw;
float loca_JY_Yaw;

//¸üÐÂ½Ç¶È
void JY_931(void)
{
	Angle_Calcu();
	Last_Angle_z_temp=Angle_z_temp;
	
}
