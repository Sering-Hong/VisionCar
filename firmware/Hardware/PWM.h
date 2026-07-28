#ifndef __PWM_H
#define __PWM_H

void PWM_Init(void);
void PWM2_Init(void);
void PWM_SetCompare1(uint16_t Compare);
void PWM_SetCompare2(uint16_t Compare);
void PWM2_SetCompare1(uint16_t Compare);
void PWM2_SetCompare4(uint16_t Compare);
#endif
