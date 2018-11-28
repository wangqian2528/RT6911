//按摩椅运行状态描述
#include "MassageStatus.h"
#include "AxisMotor.h"
#define KEY_HOLD_ENGINEERING_TIME   50 //单位0.1秒 

#define IDLE_INDICATE_TIME     20
#define WAIT_INDICATE_TIME     10  
#define SETTLE_INDICATE_TIME   3
#define RUN_INDICATE_TIME      5
#define PROBLEM_INDICATE_TIME  1

//背部按摩机构运行模式描述
//背背运行主模式
#define BACK_MAIN_MODE_IDLE			0 //与CHAIR_STATE_IDLE对应
#define BACK_MAIN_MODE_SETTLE		        1 //与CHAIR_STATE_SETTLE对应 
#define BACK_MAIN_MODE_AUTO			2	
#define BACK_MAIN_MODE_MANUAL		        3
#define BACK_MAIN_MODE_3D                       4
#define BACK_MAIN_MODE_DEMO                     5

//#define BACK_MAIN_MODE_CLOUD                     6//add  by taoqingsong




//背背运行子模式
#define BACK_SUB_MODE_AUTO_0			0 //运动恢复
#define BACK_SUB_MODE_AUTO_1			1 //舒适按摩 舒展活络（拉升）
#define BACK_SUB_MODE_AUTO_2			2 //  休憩促眠   
#define BACK_SUB_MODE_AUTO_3			3 //工作减压
#define BACK_SUB_MODE_AUTO_4			4 //肩颈重点
#define BACK_SUB_MODE_AUTO_5			5 //腰椎舒缓 


#define BACK_SUB_MODE_3D1                       6  
#define BACK_SUB_MODE_3D2                       7  
#define BACK_SUB_MODE_3D3                       8 
#define BACK_SUB_MODE_DEMO                      9 
#define BACK_SUB_MODE_6MIN_DEMO                 10

#define BACK_SUB_MODE_NO_ACTION			11
#define BACK_SUB_MODE_KNEAD			12
#define BACK_SUB_MODE_KNOCK			13
#define BACK_SUB_MODE_PRESS			14
#define BACK_SUB_MODE_WAVELET			15// //揉捏同步
#define BACK_SUB_MODE_SOFT_KNOCK		16////叩击
#define BACK_SUB_MODE_MUSIC			17////韵律按摩
//#define BACK_SUB_MODE_BODY_DETECT_0		13
//#define BACK_SUB_MODE_BODY_DETECT_1		14
//#define BACK_SUB_MODE_BODY_DETECT_2		15
#define BACK_SUB_MODE_RUBBING			18//  //搓背
 
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
//RT8302S行走电机行程
//行走行程开关到位信号
#define POSITION_T1				200
#define POSITION_T2				185
#define POSITION_T3				170
#define POSITION_T4				155
#define POSITION_T5				140
#define POSITION_T6				125
#define POSITION_T7				105
#define POSITION_T8				95

#define REACH_LIMIT 				1
//行程参数
#define BODY_TOUCHED 0               //后背光耦开关碰到时的值
#define BODY_NO_TOUCHED 1               //后背光耦开关碰到时的值
#define LIMIT_POSITION				100  //肩膀位置的最低点
#define TOP_POSITION 				WALK_TOP_POSITION
#define WAIST_POSITION 				0
#define BUTTOCKS_POSITION 			80
#define LIMIT_PRECISION                         10

#define LOW_DIFF				0
#define HIGH_DIFF				0
#define HALF_PARTIAL_DIFF			25
#define PARTIAL_DIFF				65
//与肩部定位相关的常数
#define DEFAULT_SHOULDER_POSITION	150
#define DEFAULT_SHOULDER_POSITION_RELAX	120
#define DEFAULT_NECK_LENGTH		30  
#define Med_NECK_LENGTH			15  
#define MAX_SHOULDER_ADJUST_TIME	100 //肩部微调时间，单位：100ms
//#define SHOULDER_ADJUST_STEP		30
#define MAX_SHOULDER_ADJUST_DIFF	52 //肩部最大微调距离：5*SHOULDER_ADJUST_STEP + 2(考虑显示和惯性)
***********************************/

//=================
/*
//RT8302
//RT8600为长行程
#define POSITION_T1				330
#define POSITION_T2				280
#define POSITION_T3				240
#define POSITION_T4				200
#define POSITION_T5				160
#define POSITION_T6				120
#define POSITION_T7				80
#define POSITION_T8				40

#define REACH_LIMIT 				1
//行程参数
#define BODY_TOUCHED 0               //后背光耦开关碰到时的值
#define LIMIT_POSITION				100// 100  //肩膀位置的最低点
#define TOP_POSITION 				215// 215  //肩膀位置的最高点
#define WAIST_POSITION 				120
#define BUTTOCKS_POSITION 			90
#define LIMIT_PRECISION                         10

//#define TOP_POSITION 				405  //肩膀位置的最高点
#define LOW_DIFF				0
#define HIGH_DIFF				0
#define HALF_PARTIAL_DIFF			25
#define PARTIAL_DIFF				65
//与肩部定位相关的常数
#define DEFAULT_SHOULDER_POSITION	330
#define DEFAULT_SHOULDER_POSITION_RELAX	305
#define DEFAULT_NECK_LENGTH		30  
#define Med_NECK_LENGTH			15  
#define MAX_SHOULDER_ADJUST_TIME	100 //肩部微调时间，单位：100ms
#define SHOULDER_ADJUST_STEP		30
#define MAX_SHOULDER_ADJUST_DIFF	52 //肩部最大微调距离：5*SHOULDER_ADJUST_STEP + 2(考虑显示和惯性)
*/


//=======================================================
//行走行程开关到位信号
//RT8600为长行程
#define POSITION_T1				330
#define POSITION_T2				280
#define POSITION_T3				240
#define POSITION_T4				200
#define POSITION_T5				160
#define POSITION_T6				120
#define POSITION_T7				80
#define POSITION_T8				40

#define REACH_LIMIT 				1
//行程参数 
//手拉下揉捏头是为ON 信号=BODY_TOUCHED=1,VOUT=1
#define BODY_TOUCHED 1                          //8600s修改为通讯方式，改为逻辑1，后背光耦开关碰到时的值,当检测到肩位时，镜向计数板发送高电平信号给主控板，有镜向转接板检测肩位电平信号
#define BODY_NO_TOUCHED 0                          //8600s修改为通讯方式，改为逻辑1，后背光耦开关碰到时的值


#define LIMIT_POSITION			280//100//8302T	      280//8600   ,        8302=100  //肩膀位置的最低点
#define TOP_POSITION 			399//215//215//8302T      	397//8600   ,        8302=215  //肩膀位置的最高点
#define RESET_POSITION		        (TOP_POSITION - 30)  //
#define WAIST_POSITION 			130//5//20//60//   68               //120 modify by taoqingsong//8302T	   130 //8600           //8302 =120                            //3D按摩时深度指压时要用到腰部位置
#define BUTTOCKS_POSITION 		50//20//代替腰上面的数组，仅用于自动程序数组5
#define LIMIT_PRECISION             35//20 // 20// 30// 20// 10//8302T         10

#define START_CHECK_POSITION        240//POSITION_T5//10// 35//50    代替  

#define DEFAULT_EAR_POSITION		52//modify bytaoqingsong



//#define TOP_POSITION 				405  //肩膀位置的最高点
#define LOW_DIFF				0
#define HIGH_DIFF				0
#define HALF_PARTIAL_DIFF			25
#define PARTIAL_DIFF				65



//#define _3D_MOTOR_WALK_MAX_POSITION    190//225 // 370 ,当行走电机位置超过这个值时，3D机芯会碰到机械外壳




//与肩部定位相关的常数
#define DEFAULT_SHOULDER_MAX_POSITION	370//190//该位置应该是3D动作时行走电机可以行走的最大位置，，  150// 150//    150 //8302T              370//8600                         //8302=150

#define DEFAULT_SHOULDER_POSITION	305//190//150

#define DEFAULT_SHOULDER_POSITION_RELAX	 305//120 //8302T              305//8600                          //8302=120
#define DEFAULT_NECK_LENGTH		 26//30  modify bytaoqingsong
#define Med_NECK_LENGTH			13//15   modify bytaoqingsong


//#define DEFAULT_NECK_LENGTH			30  //肩部上方最大偏移(即颈部长度)   8305A 参数
//#define Med_NECK_LENGTH				15  //肩部上方最大偏移(即颈部长度)  8305A 参数


#define MAX_SHOULDER_ADJUST_TIME	100 //肩部微调时间，单位：100ms
#define SHOULDER_ADJUST_STEP		30  //no use
#define MAX_SHOULDER_ADJUST_DIFF	52 //肩部最大微调距离：5*SHOULDER_ADJUST_STEP + 2(考虑显示和惯性)

//肩膀位置必须介于最低点和最高点之
//行走电机上电/断电常数
//行走电机上电/断电常数
#ifdef WALK_POWER_LOW
#define WALK_MOTOR_POWER_ON		0
#define WALK_MOTOR_POWER_OFF		1
#endif
#ifdef WALK_POWER_HIGH
#define WALK_MOTOR_POWER_ON		1
#define WALK_MOTOR_POWER_OFF		0
#endif
//行走电机时间描述常数
#define MAX_PARK_TIME			0xff //定点方式时，WalkMotorControl()函数不返回值，确保SOFT_KNOCK正常


//行走电机定位特点
#define LOCATE_POINT			0   //定点
#define LOCATE_FULL_BACK		1    //机芯按摩部位，全程
#define LOCATE_UPPER_BACK		2
#define LOCATE_LOWER_BACK		3
#define LOCATE_SHOULDER			4
#define LOCATE_BACK			5
#define LOCATE_WAIST			6
#define LOCATE_PARTIAL			7   //局部
#define LOCATE_NONE			8



//===================================================================
//行走电机定位方式
  //行走电机定位方式
  //0:WALK_LOCATE_ABSULATE:由绝对坐标决定
  //1:WALK_LOCATE_SHOULDER:由肩部位置决定
  //2:WALK_LOCATE_TOP:由上端行程开关决定
  //3:WALK_LOCATE_SHOULDER_OR_ABSULATE:由肩部位置和绝对坐标中的较小者决定
  //4:WALK_LOCATE_PARK:停留在当前位置



#define WALK_LOCATE_ABSULATE			0   //由绝对坐标决定
#define WALK_LOCATE_SHOULDER			3  //肩膀位置
#define WALK_LOCATE_TOP				2//由上端行程开关决定
#define WALK_LOCATE_SHOULDER_OR_ABSULATE	1//由肩部位置和绝对坐标中的较小者决定
#define WALK_LOCATE_PARK			4//停留在当前位置
//#define WALK_LOCATE_NeckSwitch			5
#define WALK_LOCATE_NeckMed			6  //脖子中间位置
#define WALK_LOCATE_PressNeck			7  //脖子靠近肩膀位置

#define WALK_LOCATE_WAIST             8
#define WALK_SHOULDER_WAIST_1_10      9  //肩膀向腰的位置移动1/10的距离
#define WALK_SHOULDER_WAIST_2_10      10 //肩膀向腰的位置移动2/10的距离
#define WALK_SHOULDER_WAIST_3_10      11 //肩膀向腰的位置移动3/10的距离
#define WALK_SHOULDER_WAIST_4_10      12 //肩膀向腰的位置移动4/10的距离
#define WALK_SHOULDER_WAIST_5_10      13 //肩膀向腰的位置移动5/10的距离
#define WALK_SHOULDER_WAIST_6_10      14 //肩膀向腰的位置移动6/10的距离
#define WALK_SHOULDER_WAIST_7_10      15 //肩膀向腰的位置移动7/10的距离
#define WALK_SHOULDER_WAIST_8_10      16 //肩膀向腰的位置移动8/10的距离
#define WALK_SHOULDER_WAIST_9_10      17 //肩膀向腰的位置移动9/10的距离

#define WALK_LOCATE_Ear			18  //ear 
#define WALK_SHOULDER_WAIST_1_9      19 //肩膀向腰的位置移动2/10的距离
//=========================================================================


//揉捏电机描述常数
#define KNEAD_STOP			0 //按摩臂停留在随机位置
#define KNEAD_STOP_AT_MIN		1 //按摩臂停留在窄的位置
#define KNEAD_STOP_AT_MED		2 //按摩臂停留在中的位置
#define KNEAD_STOP_AT_MAX		3 //按摩臂停留在宽的位置
#define KNEAD_RUN			4 //按摩臂顺时钟方向揉捏
#define KNEAD_RUN_STOP		 	5 //按摩臂CLOCK方向n圈后停留在随机位置
#define KNEAD_RUN_STOP_AT_MIN 	        6 //按摩臂CLOCK方向n圈后停留在窄的位置
#define KNEAD_RUN_STOP_AT_MED 	        7 //按摩臂CLOCK方向n圈后停留在中的位置
#define KNEAD_RUN_STOP_AT_MAX 	        8 //按摩臂CLOCK方向n圈后停留在宽的位置
#define KNEAD_RUN_RUBBING 	        9 //
#define KNEAD_ANTIRUN	                10
#define KNEAD_RUN_CYCLE 	        11 //

//揉捏臂宽度定义
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
//揉捏速度PWM常数定义
//敲打电机描述常数
#define KNOCK_STOP		0 //停止
#define KNOCK_RUN_WIDTH		1 //宽中窄定位完成后启动
#define KNOCK_RUN		2 //无需宽中窄定位，无条件立即启动
#define KNOCK_RUN_STOP		3 //宽中窄定位完成后启动短时间后马上停止
#define KNOCK_RUN_MUSIC		4 //音乐互动模式（与宽中窄定位无关）

//3D电机描述常数
#define AXIS_1		0 //3D按摩头最靠后 力度最小
#define AXIS_2		1 //3D按摩头较靠后 力度较小
#define AXIS_3		2 //3D按摩头在中间位置 力度适中
#define AXIS_4		3 //3D按摩头较靠前 力度较大
#define AXIS_5		4 //3D按摩头最靠前 力度最大
#define AXIS_AUTO	5 //依据电流自动调整按摩头位置

#define KNOCK_RUN_WIDTH		1 //宽中窄定位完成后启动
#define KNOCK_RUN		2 //无需宽中窄定位，无条件立即启动
#define KNOCK_RUN_STOP		3 //宽中窄定位完成后启动短时间后马上停止
#define KNOCK_RUN_MUSIC		4 //音乐互动模式（与宽中窄定位无关）

#define _3D_MANUAL      0   //依据用户设定3D力度动作
#define _3D_PROGRAM     1   //依据设定的3D位置动作
#define _3D_CURRENT     2   //依据电流检测位置动作
#define _3D_PARK        3   //3D马达停留在当前位置
#define _3D_MANUAL_AUTO_VECTOR   4   //机芯处于MANUAL 模式时，3D 前后工作


#define _3D_RUN_PRESSURE    1 //3D指压  3D机芯前后运动 揉捏停止 敲击停止 可调节宽中窄 可进行局部定点全身按摩 

#define _3D_WEAK_TIME       20  //3d 指压时停在最后面的时间
#define _3D_STRONG_TIME     40  //3d 指压时停在最前面的时间

//敲打速度PWM常数定义(指压敲打)
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

//与音乐互动相关
#define MAX_MUSIC_KNOCK_PWM	KNOCK_SPEED6_PWM	
#define MIN_MUSIC_KNOCK_PWM	KNOCK_SPEED1_PWM
#define MUSIC_KNOCK_AD_RATIO 3
//整机揉捏敲打速度常数定义
#define SPEED_0		0
#define SPEED_1		1
#define SPEED_2		2
#define SPEED_3		3
#define SPEED_4		4
#define SPEED_5		5
#define SPEED_6		6

//靠背电动缸上电/断电常数
#define BACKPAD_MOTOR_POWER_ON		1
#define BACKPAD_MOTOR_POWER_OFF		0
//小腿电动缸上电/断电常数
#define LEGPAD_MOTOR_POWER_ON		1
#define LEGPAD_MOTOR_POWER_OFF		0

//继电器+Mosfet+Brake电路电机状态定义(无继电器只取前面5种状态)
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
//继电器+Mosfet电机状态时间
#define PRE_BRAKE_TIME				30//5  //20 //Unit:10ms 断电到刹车前的时间
#define BRAKE_TIME				35 //10 //Unit:10ms 刹车时间
#define POST_BRAKE_TIME				30//5  //2  //Unit:10ms 刹车之后的延时时间
#define RELAY_STABLE_TIME 			25 //15 //Unit:10ms 继电器换向后的稳定时间
//继电器+Mosfet电机方向定义
#define BRAKE_OFF				0
#define BRAKE_ON				1
#define DIRECTION_RELAY_CLOCK			0
#define DIRECTION_RELAY_ANTICLOCK		1

//记忆功能常数定义
#define MEMORY_SET_OFF		0
#define MEMORY_SET_START	1		
#define MEMORY_SET_FINISH	2

#define MEMORY_SET_START_TIME	40 //50 //UNIT:100MS		
#define MEMORY_SET_FINISH_TIME	60 //20 //UNIT:100MS


typedef struct Walk_Knead_Knock_Motor_Struct_Auto
{
  //1st byte
  unsigned char nSubFunction ;////子功能索引(用于显示),  //KNEAD,KNOCK,PRESS,WAVELET,PREPARE
  unsigned char nWalkMotorLocateMethod ;// //行走电机定位方式,//0:WALK_LOCATE_ABSULATE:由绝对坐标决定 //1:WALK_LOCATE_SHOULDER:由肩部位置决定
  //2nd byte
  unsigned short nWalkMotorLocateParam;  //长行程//行走电机定位的绝对坐标或在PARK时的停顿时间
  //3rd byte
  unsigned nKneadMotorState:4 ;////KNEAD_STOP,	KNEAD_STOP_AT_MIN,/KNEAD_STOP_AT_MED,KNEAD_STOP_AT_MAX
  unsigned nKneadMotorCycles:4 ;//揉捏圈数
  //4th byte(KNOCK_STOP,KNOCK_RUN_WIDTH,KNOCK_RUN,KNOCK_RUN_STOP,KNOCK_RUN_MUSIC)	
  //Only 4 states for auto mode
  unsigned nKnockMotorState:2 ;//// //敲打电机要达到的状态, //KNOCK_STOP/KNOCK_RUN_WIDTH/KNOCK_RUN/KNOCK_RUN_STOP
  unsigned nKnockMotorRunTime:6 ;//敲打持续的时间
  //5th byte
  unsigned nKnockMotorStopTime:5 ;//敲打停顿的时间
  //unsigned nAxisMotorPosition ;//敲打停顿的时间
  unsigned nKneadKnockSpeed:3 ;//揉敲速度
  unsigned char n3D_MotorState;
  unsigned char n3D_MotorPosition;
  unsigned char n3D_MotorSpeed;
  unsigned char n3D_MotorStopTime;
} WALK_KNEAD_KNOCK_MOTOR_STRUCT_AUTO ;//WALK_KNEAD_KNOCK_MOTOR_STRUCT_AUTO为一个结构体类型名，它是结构体类型Walk_Knead_Knock_Motor_Struct_Auto的别名

//WALK_KNEAD_KNOCK_MOTOR_STRUCT_AUTO AutoDirector;//     AutoDirector为定义结构体类型的变量
//WALK_KNEAD_KNOCK_MOTOR_STRUCT_AUTO *pCludeAutoFunction_0,*pCludeAutoFunction_1,*pCludeAutoFunction_2,*pCludeAutoFunction_3;//定义结构体指针变量

//(INT32U *)DATABASEADDRESS;  可以理解为固定地址的指针
//int *p  ,   p=(INT32U *)DATABASEADDRESS  指针指向固定的地址  ,定义指向整形变量的指针变量
//定义指向结构体变量的指针变量
//pCludeAutoFunction_0 = (WALK_KNEAD_KNOCK_MOTOR_STRUCT_AUTO *)(CLUDE_AUTO_0_BASE + CLUDE_AUTO_PROGRAM_OFFSET);
//
/*********************************/
typedef struct Walk_Knead_Knock_Motor_Struct_Manual
{
  //子功能索引(用于显示)
  //KNEAD,KNOCK,PRESS,WAVELET,PREPARE
  unsigned char nSubFunction ;
  //行走电机定位方式
  //0:WALK_LOCATE_ABSULATE:由绝对坐标决定
  //1:WALK_LOCATE_SHOULDER:由肩部位置决定
  //2:WALK_LOCATE_TOP:由上端行程开关决定
  //3:WALK_LOCATE_SHOULDER_OR_ABSULATE:由肩部位置和绝对坐标中的较小者决定
  //4:WALK_LOCATE_PARK:停留在当前位置
  unsigned char nWalkMotorLocateMethod ;
  //行走电机定位的绝对坐标或在PARK时的停顿时间
  unsigned short nWalkMotorLocateParam ; //长行程
  //揉捏电机描述(包含了结束条件)
  //KNEAD_STOP			0 //按摩臂停留在随机位置
  //KNEAD_STOP_AT_MIN		1 //按摩臂停留在窄的位置
  //KNEAD_STOP_AT_MED		2 //按摩臂停留在中的位置
  //KNEAD_STOP_AT_MAX		3 //按摩臂停留在宽的位置
  //KNEAD_RUN			4 //按摩臂顺时钟方向揉捏
  //KNEAD_RUN_STOP		5 //按摩臂n圈后停留在随机位置
  //KNEAD_RUN_STOP_AT_MIN       6 //按摩臂n圈后停留在窄的位置
  //KNEAD_RUN_STOP_AT_MED       7 //按摩臂n圈后停留在中的位置
  //KNEAD_RUN_STOP_AT_MAX       8 //按摩臂n圈后停留在宽的位置	
  unsigned char nKneadMotorState ;
  //揉捏圈数
  unsigned char nKneadMotorCycles ;
  //揉捏方向 RT8600
  // unsigned char nKneadMotorPhase ;  
  //敲打电机要达到的状态
  //KNOCK_STOP/KNOCK_RUN_WIDTH/KNOCK_RUN/KNOCK_RUN_STOP
  unsigned char nKnockMotorState ;
  unsigned char nKnockMotorRunTime ;//敲打持续的时间
  unsigned char nKnockMotorStopTime ;//敲打停顿的时间
  //揉捏与敲打的速度	
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
