//��Ħ������״̬����
#include "MassageStatus.h"
#include "AxisMotor.h"
#define KEY_HOLD_ENGINEERING_TIME   50 //��λ0.1�� 

#define IDLE_INDICATE_TIME     20
#define WAIT_INDICATE_TIME     10  
#define SETTLE_INDICATE_TIME   3
#define RUN_INDICATE_TIME      5
#define PROBLEM_INDICATE_TIME  1

//������Ħ��������ģʽ����
//����������ģʽ
#define BACK_MAIN_MODE_IDLE			0 //��CHAIR_STATE_IDLE��Ӧ
#define BACK_MAIN_MODE_SETTLE		        1 //��CHAIR_STATE_SETTLE��Ӧ 
#define BACK_MAIN_MODE_AUTO			2	
#define BACK_MAIN_MODE_MANUAL		        3
#define BACK_MAIN_MODE_3D                       4
#define BACK_MAIN_MODE_DEMO                     5

//#define BACK_MAIN_MODE_CLOUD                     6//add  by taoqingsong




//����������ģʽ
#define BACK_SUB_MODE_AUTO_0			0 //�˶��ָ�
#define BACK_SUB_MODE_AUTO_1			1 //���ʰ�Ħ ��չ���磨������
#define BACK_SUB_MODE_AUTO_2			2 //  ������   
#define BACK_SUB_MODE_AUTO_3			3 //������ѹ
#define BACK_SUB_MODE_AUTO_4			4 //�羱�ص�
#define BACK_SUB_MODE_AUTO_5			5 //��׵�滺 


#define BACK_SUB_MODE_3D1                       6  
#define BACK_SUB_MODE_3D2                       7  
#define BACK_SUB_MODE_3D3                       8 
#define BACK_SUB_MODE_DEMO                      9 
#define BACK_SUB_MODE_6MIN_DEMO                 10

#define BACK_SUB_MODE_NO_ACTION			11
#define BACK_SUB_MODE_KNEAD			12
#define BACK_SUB_MODE_KNOCK			13
#define BACK_SUB_MODE_PRESS			14
#define BACK_SUB_MODE_WAVELET			15// //����ͬ��
#define BACK_SUB_MODE_SOFT_KNOCK		16////ߵ��
#define BACK_SUB_MODE_MUSIC			17////���ɰ�Ħ
//#define BACK_SUB_MODE_BODY_DETECT_0		13
//#define BACK_SUB_MODE_BODY_DETECT_1		14
//#define BACK_SUB_MODE_BODY_DETECT_2		15
#define BACK_SUB_MODE_RUBBING			18//  //�걳
 
#define BACK_SUB_MODE_FULL_AIRBAG               19  
 

#define BACK_SUB_MODE_CLUDE_AUTO_0              20
#define BACK_SUB_MODE_CLUDE_AUTO_1              21
#define BACK_SUB_MODE_CLUDE_AUTO_2              22
#define BACK_SUB_MODE_CLUDE_AUTO_3              23



/*****************************************/
#define SETZERO_LEG_TMIE1			400
#define SETZERO_LEG_TMIE2			1000
#define SETZERO_BACK_TMIE1			400
#define SETZERO_BACK_TMIE2			1000
#define MAXMOTORREMOVETIME		        1600



//========================================================

/********************************************************************
//RT8302S���ߵ���г�
//�����г̿��ص�λ�ź�
#define POSITION_T1				200
#define POSITION_T2				185
#define POSITION_T3				170
#define POSITION_T4				155
#define POSITION_T5				140
#define POSITION_T6				125
#define POSITION_T7				105
#define POSITION_T8				95

#define REACH_LIMIT 				1
//�г̲���
#define BODY_TOUCHED 0               //�󱳹��������ʱ��ֵ
#define BODY_NO_TOUCHED 1               //�󱳹��������ʱ��ֵ
#define LIMIT_POSITION				100  //���λ�õ���͵�
#define TOP_POSITION 				WALK_TOP_POSITION
#define WAIST_POSITION 				0
#define BUTTOCKS_POSITION 			80
#define LIMIT_PRECISION                         10

#define LOW_DIFF				0
#define HIGH_DIFF				0
#define HALF_PARTIAL_DIFF			25
#define PARTIAL_DIFF				65
//��粿��λ��صĳ���
#define DEFAULT_SHOULDER_POSITION	150
#define DEFAULT_SHOULDER_POSITION_RELAX	120
#define DEFAULT_NECK_LENGTH		30  
#define Med_NECK_LENGTH			15  
#define MAX_SHOULDER_ADJUST_TIME	100 //�粿΢��ʱ�䣬��λ��100ms
//#define SHOULDER_ADJUST_STEP		30
#define MAX_SHOULDER_ADJUST_DIFF	52 //�粿���΢�����룺5*SHOULDER_ADJUST_STEP + 2(������ʾ�͹���)
***********************************/

//=================
/*
//RT8302
//RT8600Ϊ���г�
#define POSITION_T1				330
#define POSITION_T2				280
#define POSITION_T3				240
#define POSITION_T4				200
#define POSITION_T5				160
#define POSITION_T6				120
#define POSITION_T7				80
#define POSITION_T8				40

#define REACH_LIMIT 				1
//�г̲���
#define BODY_TOUCHED 0               //�󱳹��������ʱ��ֵ
#define LIMIT_POSITION				100// 100  //���λ�õ���͵�
#define TOP_POSITION 				215// 215  //���λ�õ���ߵ�
#define WAIST_POSITION 				120
#define BUTTOCKS_POSITION 			90
#define LIMIT_PRECISION                         10

//#define TOP_POSITION 				405  //���λ�õ���ߵ�
#define LOW_DIFF				0
#define HIGH_DIFF				0
#define HALF_PARTIAL_DIFF			25
#define PARTIAL_DIFF				65
//��粿��λ��صĳ���
#define DEFAULT_SHOULDER_POSITION	330
#define DEFAULT_SHOULDER_POSITION_RELAX	305
#define DEFAULT_NECK_LENGTH		30  
#define Med_NECK_LENGTH			15  
#define MAX_SHOULDER_ADJUST_TIME	100 //�粿΢��ʱ�䣬��λ��100ms
#define SHOULDER_ADJUST_STEP		30
#define MAX_SHOULDER_ADJUST_DIFF	52 //�粿���΢�����룺5*SHOULDER_ADJUST_STEP + 2(������ʾ�͹���)
*/


//=======================================================
//�����г̿��ص�λ�ź�
//RT8600Ϊ���г�
#define POSITION_T1				330
#define POSITION_T2				280
#define POSITION_T3				240
#define POSITION_T4				200
#define POSITION_T5				160
#define POSITION_T6				120
#define POSITION_T7				80
#define POSITION_T8				40

#define REACH_LIMIT 				1
//�г̲��� 
//����������ͷ��ΪON �ź�=BODY_TOUCHED=1,VOUT=1
#define BODY_TOUCHED 1                          //8600s�޸�ΪͨѶ��ʽ����Ϊ�߼�1���󱳹��������ʱ��ֵ,����⵽��λʱ����������巢�͸ߵ�ƽ�źŸ����ذ壬�о���ת�Ӱ����λ��ƽ�ź�
#define BODY_NO_TOUCHED 0                          //8600s�޸�ΪͨѶ��ʽ����Ϊ�߼�1���󱳹��������ʱ��ֵ


#define LIMIT_POSITION			280//100//8302T	      280//8600   ,        8302=100  //���λ�õ���͵�
#define TOP_POSITION 			399//215//215//8302T      	397//8600   ,        8302=215  //���λ�õ���ߵ�
#define RESET_POSITION		        (TOP_POSITION - 30)  //
#define WAIST_POSITION 			130//5//20//60//   68               //120 modify by taoqingsong//8302T	   130 //8600           //8302 =120                            //3D��Ħʱ���ָѹʱҪ�õ�����λ��
#define BUTTOCKS_POSITION 		50//20//��������������飬�������Զ���������5
#define LIMIT_PRECISION             35//20 // 20// 30// 20// 10//8302T         10

#define START_CHECK_POSITION        240//POSITION_T5//10// 35//50    ����  

#define DEFAULT_EAR_POSITION		52//modify bytaoqingsong



//#define TOP_POSITION 				405  //���λ�õ���ߵ�
#define LOW_DIFF				0
#define HIGH_DIFF				0
#define HALF_PARTIAL_DIFF			25
#define PARTIAL_DIFF				65



//#define _3D_MOTOR_WALK_MAX_POSITION    190//225 // 370 ,�����ߵ��λ�ó������ֵʱ��3D��о��������е���




//��粿��λ��صĳ���
#define DEFAULT_SHOULDER_MAX_POSITION	370//190//��λ��Ӧ����3D����ʱ���ߵ���������ߵ����λ�ã���  150// 150//    150 //8302T              370//8600                         //8302=150

#define DEFAULT_SHOULDER_POSITION	305//190//150

#define DEFAULT_SHOULDER_POSITION_RELAX	 305//120 //8302T              305//8600                          //8302=120
#define DEFAULT_NECK_LENGTH		 26//30  modify bytaoqingsong
#define Med_NECK_LENGTH			13//15   modify bytaoqingsong


//#define DEFAULT_NECK_LENGTH			30  //�粿�Ϸ����ƫ��(����������)   8305A ����
//#define Med_NECK_LENGTH				15  //�粿�Ϸ����ƫ��(����������)  8305A ����


#define MAX_SHOULDER_ADJUST_TIME	100 //�粿΢��ʱ�䣬��λ��100ms
#define SHOULDER_ADJUST_STEP		30  //no use
#define MAX_SHOULDER_ADJUST_DIFF	52 //�粿���΢�����룺5*SHOULDER_ADJUST_STEP + 2(������ʾ�͹���)

//���λ�ñ��������͵����ߵ�֮
//���ߵ���ϵ�/�ϵ糣��
//���ߵ���ϵ�/�ϵ糣��
#ifdef WALK_POWER_LOW
#define WALK_MOTOR_POWER_ON		0
#define WALK_MOTOR_POWER_OFF		1
#endif
#ifdef WALK_POWER_HIGH
#define WALK_MOTOR_POWER_ON		1
#define WALK_MOTOR_POWER_OFF		0
#endif
//���ߵ��ʱ����������
#define MAX_PARK_TIME			0xff //���㷽ʽʱ��WalkMotorControl()����������ֵ��ȷ��SOFT_KNOCK����


//���ߵ����λ�ص�
#define LOCATE_POINT			0   //����
#define LOCATE_FULL_BACK		1    //��о��Ħ��λ��ȫ��
#define LOCATE_UPPER_BACK		2
#define LOCATE_LOWER_BACK		3
#define LOCATE_SHOULDER			4
#define LOCATE_BACK			5
#define LOCATE_WAIST			6
#define LOCATE_PARTIAL			7   //�ֲ�
#define LOCATE_NONE			8



//===================================================================
//���ߵ����λ��ʽ
  //���ߵ����λ��ʽ
  //0:WALK_LOCATE_ABSULATE:�ɾ����������
  //1:WALK_LOCATE_SHOULDER:�ɼ粿λ�þ���
  //2:WALK_LOCATE_TOP:���϶��г̿��ؾ���
  //3:WALK_LOCATE_SHOULDER_OR_ABSULATE:�ɼ粿λ�ú;��������еĽ�С�߾���
  //4:WALK_LOCATE_PARK:ͣ���ڵ�ǰλ��



#define WALK_LOCATE_ABSULATE			0   //�ɾ����������
#define WALK_LOCATE_SHOULDER			3  //���λ��
#define WALK_LOCATE_TOP				2//���϶��г̿��ؾ���
#define WALK_LOCATE_SHOULDER_OR_ABSULATE	1//�ɼ粿λ�ú;��������еĽ�С�߾���
#define WALK_LOCATE_PARK			4//ͣ���ڵ�ǰλ��
//#define WALK_LOCATE_NeckSwitch			5
#define WALK_LOCATE_NeckMed			6  //�����м�λ��
#define WALK_LOCATE_PressNeck			7  //���ӿ������λ��

#define WALK_LOCATE_WAIST             8
#define WALK_SHOULDER_WAIST_1_10      9  //���������λ���ƶ�1/10�ľ���
#define WALK_SHOULDER_WAIST_2_10      10 //���������λ���ƶ�2/10�ľ���
#define WALK_SHOULDER_WAIST_3_10      11 //���������λ���ƶ�3/10�ľ���
#define WALK_SHOULDER_WAIST_4_10      12 //���������λ���ƶ�4/10�ľ���
#define WALK_SHOULDER_WAIST_5_10      13 //���������λ���ƶ�5/10�ľ���
#define WALK_SHOULDER_WAIST_6_10      14 //���������λ���ƶ�6/10�ľ���
#define WALK_SHOULDER_WAIST_7_10      15 //���������λ���ƶ�7/10�ľ���
#define WALK_SHOULDER_WAIST_8_10      16 //���������λ���ƶ�8/10�ľ���
#define WALK_SHOULDER_WAIST_9_10      17 //���������λ���ƶ�9/10�ľ���

#define WALK_LOCATE_Ear			18  //ear 
#define WALK_SHOULDER_WAIST_1_9      19 //���������λ���ƶ�2/10�ľ���
//=========================================================================


//��������������
#define KNEAD_STOP			0 //��Ħ��ͣ�������λ��
#define KNEAD_STOP_AT_MIN		1 //��Ħ��ͣ����խ��λ��
#define KNEAD_STOP_AT_MED		2 //��Ħ��ͣ�����е�λ��
#define KNEAD_STOP_AT_MAX		3 //��Ħ��ͣ���ڿ��λ��
#define KNEAD_RUN			4 //��Ħ��˳ʱ�ӷ�������
#define KNEAD_RUN_STOP		 	5 //��Ħ��CLOCK����nȦ��ͣ�������λ��
#define KNEAD_RUN_STOP_AT_MIN 	        6 //��Ħ��CLOCK����nȦ��ͣ����խ��λ��
#define KNEAD_RUN_STOP_AT_MED 	        7 //��Ħ��CLOCK����nȦ��ͣ�����е�λ��
#define KNEAD_RUN_STOP_AT_MAX 	        8 //��Ħ��CLOCK����nȦ��ͣ���ڿ��λ��
#define KNEAD_RUN_RUBBING 	        9 //
#define KNEAD_ANTIRUN	                10
#define KNEAD_RUN_CYCLE 	        11 //

//����ۿ�ȶ���
/*
#define KNEAD_WIDTH_UNKNOWN		0
#define KNEAD_WIDTH_MIN			1
#define KNEAD_WIDTH_MED			2
#define KNEAD_WIDTH_MAX			3
*/
#define DISPLAY_WIDTH_OFF		0
#define DISPLAY_WIDTH_MIN		1
#define DISPLAY_WIDTH_MED		2
#define DISPLAY_WIDTH_MAX		3
//�����ٶ�PWM��������
//�ô�����������
#define KNOCK_STOP		0 //ֹͣ
#define KNOCK_RUN_WIDTH		1 //����խ��λ��ɺ�����
#define KNOCK_RUN		2 //�������խ��λ����������������
#define KNOCK_RUN_STOP		3 //����խ��λ��ɺ�������ʱ�������ֹͣ
#define KNOCK_RUN_MUSIC		4 //���ֻ���ģʽ�������խ��λ�޹أ�

//3D�����������
#define AXIS_1		0 //3D��Ħͷ��� ������С
#define AXIS_2		1 //3D��Ħͷ�Ͽ��� ���Ƚ�С
#define AXIS_3		2 //3D��Ħͷ���м�λ�� ��������
#define AXIS_4		3 //3D��Ħͷ�Ͽ�ǰ ���Ƚϴ�
#define AXIS_5		4 //3D��Ħͷ�ǰ �������
#define AXIS_AUTO	5 //���ݵ����Զ�������Ħͷλ��

#define KNOCK_RUN_WIDTH		1 //����խ��λ��ɺ�����
#define KNOCK_RUN		2 //�������խ��λ����������������
#define KNOCK_RUN_STOP		3 //����խ��λ��ɺ�������ʱ�������ֹͣ
#define KNOCK_RUN_MUSIC		4 //���ֻ���ģʽ�������խ��λ�޹أ�

#define _3D_MANUAL      0   //�����û��趨3D���ȶ���
#define _3D_PROGRAM     1   //�����趨��3Dλ�ö���
#define _3D_CURRENT     2   //���ݵ������λ�ö���
#define _3D_PARK        3   //3D���ͣ���ڵ�ǰλ��
#define _3D_MANUAL_AUTO_VECTOR   4   //��о����MANUAL ģʽʱ��3D ǰ����


#define _3D_RUN_PRESSURE    1 //3Dָѹ  3D��оǰ���˶� ����ֹͣ �û�ֹͣ �ɵ��ڿ���խ �ɽ��оֲ�����ȫ��Ħ 

#define _3D_WEAK_TIME       20  //3d ָѹʱͣ��������ʱ��
#define _3D_STRONG_TIME     40  //3d ָѹʱͣ����ǰ���ʱ��

//�ô��ٶ�PWM��������(ָѹ�ô�)
//#define MANUAL_SPEED0_PWM 	0 //0%
//#define MANUAL_SPEED1_PWM 	300
//#define MANUAL_SPEED2_PWM 	325
//#define MANUAL_SPEED3_PWM 	350
//#define MANUAL_SPEED4_PWM 	375
//#define MANUAL_SPEED5_PWM 	400
//#define MANUAL_SPEED6_PWM 	425 
//#define MANUAL_SPEED7_PWM 	450
//#define MANUAL_SPEED8_PWM 	475
//#define MANUAL_SPEED9_PWM 	500
//#define MANUAL_SPEED10_PWM 	550

//�����ֻ������
#define MAX_MUSIC_KNOCK_PWM	KNOCK_SPEED6_PWM	
#define MIN_MUSIC_KNOCK_PWM	KNOCK_SPEED1_PWM
#define MUSIC_KNOCK_AD_RATIO 3
//���������ô��ٶȳ�������
#define SPEED_0		0
#define SPEED_1		1
#define SPEED_2		2
#define SPEED_3		3
#define SPEED_4		4
#define SPEED_5		5
#define SPEED_6		6

//�����綯���ϵ�/�ϵ糣��
#define BACKPAD_MOTOR_POWER_ON		1
#define BACKPAD_MOTOR_POWER_OFF		0
//С�ȵ綯���ϵ�/�ϵ糣��
#define LEGPAD_MOTOR_POWER_ON		1
#define LEGPAD_MOTOR_POWER_OFF		0

//�̵���+Mosfet+Brake��·���״̬����(�޼̵���ֻȡǰ��5��״̬)
#define STATE_IDLE				0
#define STATE_RUN_CLOCK				1
#define STATE_STOP_CLOCK_HV			2
#define STATE_STOP_CLOCK_BRAKE		        3
#define STATE_STOP_CLOCK_ZV			4
#define STATE_RUN_ANTICLOCK			5

#define STATE_STOP_ANTICLOCK_HV		        6
#define STATE_STOP_ANTICLOCK_BRAKE	        7
#define STATE_STOP_ANTICLOCK_ZV		        8
#define STATE_RUN_UNCLOCK			9
//�̵���+Mosfet���״̬ʱ��
#define PRE_BRAKE_TIME				30//5  //20 //Unit:10ms �ϵ絽ɲ��ǰ��ʱ��
#define BRAKE_TIME				35 //10 //Unit:10ms ɲ��ʱ��
#define POST_BRAKE_TIME				30//5  //2  //Unit:10ms ɲ��֮�����ʱʱ��
#define RELAY_STABLE_TIME 			25 //15 //Unit:10ms �̵����������ȶ�ʱ��
//�̵���+Mosfet���������
#define BRAKE_OFF				0
#define BRAKE_ON				1
#define DIRECTION_RELAY_CLOCK			0
#define DIRECTION_RELAY_ANTICLOCK		1

//���书�ܳ�������
#define MEMORY_SET_OFF		0
#define MEMORY_SET_START	1		
#define MEMORY_SET_FINISH	2

#define MEMORY_SET_START_TIME	40 //50 //UNIT:100MS		
#define MEMORY_SET_FINISH_TIME	60 //20 //UNIT:100MS


typedef struct Walk_Knead_Knock_Motor_Struct_Auto
{
  //1st byte
  unsigned char nSubFunction ;////�ӹ�������(������ʾ),  //KNEAD,KNOCK,PRESS,WAVELET,PREPARE
  unsigned char nWalkMotorLocateMethod ;// //���ߵ����λ��ʽ,//0:WALK_LOCATE_ABSULATE:�ɾ���������� //1:WALK_LOCATE_SHOULDER:�ɼ粿λ�þ���
  //2nd byte
  unsigned short nWalkMotorLocateParam;  //���г�//���ߵ����λ�ľ����������PARKʱ��ͣ��ʱ��
  //3rd byte
  unsigned nKneadMotorState:4 ;////KNEAD_STOP,	KNEAD_STOP_AT_MIN,/KNEAD_STOP_AT_MED,KNEAD_STOP_AT_MAX
  unsigned nKneadMotorCycles:4 ;//����Ȧ��
  //4th byte(KNOCK_STOP,KNOCK_RUN_WIDTH,KNOCK_RUN,KNOCK_RUN_STOP,KNOCK_RUN_MUSIC)	
  //Only 4 states for auto mode
  unsigned nKnockMotorState:2 ;//// //�ô���Ҫ�ﵽ��״̬, //KNOCK_STOP/KNOCK_RUN_WIDTH/KNOCK_RUN/KNOCK_RUN_STOP
  unsigned nKnockMotorRunTime:6 ;//�ô������ʱ��
  //5th byte
  unsigned nKnockMotorStopTime:5 ;//�ô�ͣ�ٵ�ʱ��
  //unsigned nAxisMotorPosition ;//�ô�ͣ�ٵ�ʱ��
  unsigned nKneadKnockSpeed:3 ;//�����ٶ�
  unsigned char n3D_MotorState;
  unsigned char n3D_MotorPosition;
  unsigned char n3D_MotorSpeed;
  unsigned char n3D_MotorStopTime;
} WALK_KNEAD_KNOCK_MOTOR_STRUCT_AUTO ;//WALK_KNEAD_KNOCK_MOTOR_STRUCT_AUTOΪһ���ṹ�������������ǽṹ������Walk_Knead_Knock_Motor_Struct_Auto�ı���

//WALK_KNEAD_KNOCK_MOTOR_STRUCT_AUTO AutoDirector;//     AutoDirectorΪ����ṹ�����͵ı���
//WALK_KNEAD_KNOCK_MOTOR_STRUCT_AUTO *pCludeAutoFunction_0,*pCludeAutoFunction_1,*pCludeAutoFunction_2,*pCludeAutoFunction_3;//����ṹ��ָ�����

//(INT32U *)DATABASEADDRESS;  �������Ϊ�̶���ַ��ָ��
//int *p  ,   p=(INT32U *)DATABASEADDRESS  ָ��ָ��̶��ĵ�ַ  ,����ָ�����α�����ָ�����
//����ָ��ṹ�������ָ�����
//pCludeAutoFunction_0 = (WALK_KNEAD_KNOCK_MOTOR_STRUCT_AUTO *)(CLUDE_AUTO_0_BASE + CLUDE_AUTO_PROGRAM_OFFSET);
//
/*********************************/
typedef struct Walk_Knead_Knock_Motor_Struct_Manual
{
  //�ӹ�������(������ʾ)
  //KNEAD,KNOCK,PRESS,WAVELET,PREPARE
  unsigned char nSubFunction ;
  //���ߵ����λ��ʽ
  //0:WALK_LOCATE_ABSULATE:�ɾ����������
  //1:WALK_LOCATE_SHOULDER:�ɼ粿λ�þ���
  //2:WALK_LOCATE_TOP:���϶��г̿��ؾ���
  //3:WALK_LOCATE_SHOULDER_OR_ABSULATE:�ɼ粿λ�ú;��������еĽ�С�߾���
  //4:WALK_LOCATE_PARK:ͣ���ڵ�ǰλ��
  unsigned char nWalkMotorLocateMethod ;
  //���ߵ����λ�ľ����������PARKʱ��ͣ��ʱ��
  unsigned short nWalkMotorLocateParam ; //���г�
  //����������(�����˽�������)
  //KNEAD_STOP			0 //��Ħ��ͣ�������λ��
  //KNEAD_STOP_AT_MIN		1 //��Ħ��ͣ����խ��λ��
  //KNEAD_STOP_AT_MED		2 //��Ħ��ͣ�����е�λ��
  //KNEAD_STOP_AT_MAX		3 //��Ħ��ͣ���ڿ��λ��
  //KNEAD_RUN			4 //��Ħ��˳ʱ�ӷ�������
  //KNEAD_RUN_STOP		5 //��Ħ��nȦ��ͣ�������λ��
  //KNEAD_RUN_STOP_AT_MIN       6 //��Ħ��nȦ��ͣ����խ��λ��
  //KNEAD_RUN_STOP_AT_MED       7 //��Ħ��nȦ��ͣ�����е�λ��
  //KNEAD_RUN_STOP_AT_MAX       8 //��Ħ��nȦ��ͣ���ڿ��λ��	
  unsigned char nKneadMotorState ;
  //����Ȧ��
  unsigned char nKneadMotorCycles ;
  //������ RT8600
  // unsigned char nKneadMotorPhase ;  
  //�ô���Ҫ�ﵽ��״̬
  //KNOCK_STOP/KNOCK_RUN_WIDTH/KNOCK_RUN/KNOCK_RUN_STOP
  unsigned char nKnockMotorState ;
  unsigned char nKnockMotorRunTime ;//�ô������ʱ��
  unsigned char nKnockMotorStopTime ;//�ô�ͣ�ٵ�ʱ��
  //�������ô���ٶ�	
  //SPEED_0,SPEED_2,SPEED_3,SPEED_4
  unsigned char nKneadKnockSpeed ;
  unsigned char _3D_Position;
  unsigned char _3D_Speed;
  unsigned char n3D_MotorState;
  unsigned char n3D_MotorPosition;
  unsigned char n3D_MotorSpeed;
  unsigned char n3D_MotorStopTime;
} WALK_KNEAD_KNOCK_MOTOR_STRUCT_MANUAL ;








/*************************************************************/
