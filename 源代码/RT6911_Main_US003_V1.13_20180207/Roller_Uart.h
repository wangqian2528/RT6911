#ifndef __HOT_ROOLER_H__
#define __HOT_ROOLER_H__


#define ROLLER_SPEED_STOP 0
#define ROLLER_SPEED_SLOW 1
#define ROLLER_SPEED_MID  2
#define ROLLER_SPEED_FAST 3

//���¹���ģʽֻ���������Զ�����
/*
000������������ת
001������������ת
010����ʱ����������ת
011����ʱ����������ת
100����ʱ����������ת
101����ʱ����������ת
110����ʱ����
111����ʱ���� 

*/
#define ROLLER_MODE_CON_IN        0
#define ROLLER_MODE_CON_OUT       1
#define ROLLER_MODE_S_INT_IN	  2
#define ROLLER_MODE_S_INT_OUT	  3
#define ROLLER_MODE_L_INT_IN	  4
#define ROLLER_MODE_L_INT_OUT	  5
#define ROLLER_MODE_S_RUB	  6
#define ROLLER_MODE_L_RUB	  7

#define ROLLER_AUTO  1
#define ROLLER_MANUAL 0

#define ROLLER_TO_IN  0
#define ROLLER_TO_OUT 1

#define ROLLER_ON  1
#define ROLLER_OFF 0

void Roller_SetSpeed(unsigned char rollerSpeed);
unsigned char Roller_GetSpeed(void);
void Roller_SetMode(unsigned int mode);
unsigned char Roller_GetMode(void);
void RollerMotor_Control(unsigned int speed,unsigned int phase);
#endif

