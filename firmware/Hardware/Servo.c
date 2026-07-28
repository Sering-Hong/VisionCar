#include "stm32f10x.h"                  // Device header
#include "PWM.h"


void Servo_SetAngle1(float Angle)
{
	PWM2_SetCompare1(Angle / 180 * 2000 + 500);
}

void Servo_SetAngle2(float Angle)
{
	PWM2_SetCompare4(Angle / 180 * 2000 + 500);
}
