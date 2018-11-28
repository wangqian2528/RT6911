#ifndef __HOT_FLEX_H__
#define __HOT_FLEX_H__

/****************************************************************
ST ��Ƭ���ο��ĺ궨��
#define STATE_RUN_FLEX_IN		0  
#define STATE_RUN_FLEX_OUT		1  
#define STATE_FLEX_IDLE			2
#define STATE_RUN_FLEX_RESET		3
#define STATE_RUN_FLEX_MANUAL_OUT	4
#define STATE_RUN_FLEX_TEST_OUT		5
//-----------------------------------------------------------------------------
#define FLEX_MOTOR_DEFAULT_TOP	99
//-----------------------------------------------------------------------------
#define FLEX_RUN			0 //��������
#define FLEX_STOP_AT_IN			1 //��Ϊ�����������г̿��ض�ֹͣ
#define FLEX_STOP_AT_FOOT		2 //��Ϊ�����ŵ׿��ض�ֹͣ
#define FLEX_STOP_AT_FOOT_LEAVE		3 //��Ϊ�������ŵ׿��ض�ֹͣ
#define FLEX_STOP_AT_OUT		4 //��Ϊ�����������г̿��ض�ֹͣ
#define FLEX_STOP_AT_IDLE		5 //��Ϊ�յ�ֹͣ�����ֹͣ
#define FLEX_STOP_AT_ANGLE		6 //��Ϊ�Ƕȶ�ֹͣ
#define FLEX_STOP_AT_GROUND		7 //��Ϊ�Ƕȶ�ֹͣ

#define FLEX_POWER_ON		1
#define FLEX_POWER_OFF		0

#define FLEX_SPEED_STOP		0
#define FLEX_SPEED_SLOW		60
#define FLEX_SPEED_MID		80
#define FLEX_SPEED_FAST		FLEX_MOTOR_DEFAULT_TOP

//�������㷽������I*0.1*5��/3.3*FLEX_MOTOR_DEFAULT_TOP

#define FLEX_CURRENT_1A		30
#define FLEX_CURRENT_15A	40
#define FLEX_CURRENT_2A		50 
#define FLEX_CURRENT_25A	60 
#define FLEX_CURRENT_3A		70
#define FLEX_CURRENT_35A	80 
#define FLEX_CURRENT_4A		90
#define FLEX_CURRENT_5A		100

#define FOOT_SWITCH_ON		0	//�����ŵ׿���
#define FOOT_SWITCH_OFF		1

#define REACH_FLEX_LIMIT	0
//#define REACH_FLEX_LIMIT	1	//��λΪ�ߵ�ƽ, û�е�λΪ�͵�ƽ


#define LEGANGLE_SWITCH_OFF	0
#define LEGANGLE_SWITCH_ON	45
#define LEGANGLE1_SWITCH_ON	20
#define LEGANGLE2_SWITCH_ON	30

#define LEGGROUND_SWITCH_ON	1
#define LEGGROUND_SWITCH_OFF	0

#define FLEX_MOTOR_NORMAL	0x01
#define FLEX_MOTOR_FAIL		0x02

*********************************************************************/

#define FLEX_AUTO  1
#define FLEX_MANUAL 0

#define FLEX_MOTOR_STOP   0  //�������ֹͣ
#define FLEX_TO_OUT       1  //С�����������
#define FLEX_TO_IN        2 //С�����������
#define FLEX_TO_AUTO      3 //�Զ��ҽű�־λ

#define FLEX_ON  0
#define FLEX_OFF 1
#define FLEX_POWER_ON  1
#define FLEX_POWER_OFF 0
#define FLEX_AT_IN  1    //С�ȵ������λ
#define FLEX_AT_OUT 0  //С�ȵ��ﳤ��λ
#define FLEX_AT_MID 2  //С��λ���м�
#define FLEX_AT_SMALL_ANGLE 3//�г̿��ش���

#define FOOT_SWITCH_ON		1 //��������
#define FOOT_SWITCH_OFF		0

#define LEGANGLE_SWITCH_ON	1 //С�������봹ֱ�ߵĽǶȳ�С��15��ʱ���Ѿ�����Σ�սǶȣ���������ǰ����
#define LEGANGLE_SWITCH_OFF	0 //С�������봹ֱ�ߵĽǶȳ�����15��ʱ��������ǰ����

#define LEGGROUND_SWITCH_ON_2	1 //����������		
#define LEGGROUND_SWITCH_OFF_2	0 //δ�������棬��������״̬		


#define FLEX_MOTOR_CURRENT_1A     0
#define FLEX_MOTOR_CURRENT_1D5A    1
#define FLEX_MOTOR_CURRENT_2A     2
#define FLEX_MOTOR_CURRENT_2D5A   3
#define FLEX_MOTOR_CURRENT_3A     4
#define FLEX_MOTOR_CURRENT_3D5A   5
#define FLEX_MOTOR_CURRENT_4A     6
#define FLEX_MOTOR_CURRENT_5A     7

void Flex_SetDirection(unsigned int flexDirection);
unsigned char Flex_GetDirection(void);
void Flex_SetPower(unsigned char power);
unsigned char Flex_GetPower(void);
void Flex_SetMode(unsigned int mode);
unsigned char Flex_GetMode(void);
unsigned char Flex_GetCurrent(void);
void Flex_SetCurrent(unsigned char current);
void FlexMotorFollowingFood(void);
int FlexMotorGetEnable(void);
void FlexMotorSetEnable(void);
void FlexMotorSetDisable(void);
void Flex_SetStatus(unsigned char status);

void Flex_SetDisableAngle(bool disable);
unsigned char Flex_GetDisableAngle();

unsigned char Flex_ControlIn(unsigned int current);
unsigned char Flex_ControlOut(unsigned int current);


void Flex_100msInt(void);

void FlexMotorSetAdjStep(unsigned char by_step);


void Flex_ControlStop(void);
unsigned int FlexPower_Get(void);

void testwgh(void);
#endif

