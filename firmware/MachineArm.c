#include "stm32f10x.h"                  // Device header
#include "Servo.h"
#include "Delay.h"
#include "PWM.h"

void MachineArm_Get(void)
{
	Servo_SetAngle1(45);
	Servo_SetAngle2(0);
}
