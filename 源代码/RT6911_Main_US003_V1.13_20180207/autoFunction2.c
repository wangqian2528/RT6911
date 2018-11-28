

/*
//轻松按摩(有肩部位置检测),力度较弱//307
const WALK_KNEAD_KNOCK_MOTOR_STRUCT_AUTO AutoFunction2[] = 
{	
  //以下为正常动作	
  //揉捶同步到底部（低速）
  {BACK_SUB_MODE_WAVELET,WALK_LOCATE_ABSULATE,0,KNEAD_RUN,0,KNOCK_RUN,0,0,SPEED_1,_3D_PROGRAM,_3D_3,_3D_SPEED_7,0},
  //揉捏到顶部（低速）	
  {BACK_SUB_MODE_KNOCK,WALK_LOCATE_TOP,0,KNEAD_STOP,0,KNOCK_RUN,0,0,SPEED_2,_3D_PROGRAM,_3D_2,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_PARK,0,KNEAD_RUN_STOP_AT_MIN,3,KNOCK_STOP,0,0,SPEED_2,_3D_PROGRAM,_3D_4,_3D_SPEED_7,0},
  
  //颈部来回敲打
  {BACK_SUB_MODE_KNOCK,WALK_LOCATE_SHOULDER,0,KNEAD_STOP,0,KNOCK_RUN,0,0,SPEED_1,_3D_PROGRAM,_3D_2,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNOCK,WALK_LOCATE_TOP,0,KNEAD_STOP,0,KNOCK_RUN,0,0,SPEED_1,_3D_PROGRAM,_3D_3,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNOCK,WALK_LOCATE_SHOULDER,0,KNEAD_STOP,0,KNOCK_RUN,0,0,SPEED_1,_3D_PROGRAM,_3D_2,_3D_SPEED_7,0},	
  {BACK_SUB_MODE_KNOCK,WALK_LOCATE_TOP,0,KNEAD_STOP,0,KNOCK_RUN,0,0,SPEED_1,_3D_PROGRAM,_3D_3,_3D_SPEED_7,0},
  //颈部来回揉捏		
  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_PARK,40,KNEAD_RUN,0,KNOCK_STOP,0,0,SPEED_2,_3D_PROGRAM,_3D_3,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_SHOULDER,20,KNEAD_RUN,0,KNOCK_STOP,0,0,SPEED_2,_3D_PROGRAM,_3D_4,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_PARK,40,KNEAD_RUN,0,KNOCK_STOP,0,0,SPEED_2,_3D_PROGRAM,_3D_4,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_SHOULDER,0,KNEAD_RUN,0,KNOCK_STOP,0,0,SPEED_2,_3D_PROGRAM,_3D_4,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_PARK,40,KNEAD_RUN,0,KNOCK_STOP,0,0,SPEED_2,_3D_PROGRAM,_3D_4,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_NeckMed,0,KNEAD_RUN,0,KNOCK_STOP,0,0,SPEED_3,_3D_PROGRAM,_3D_4,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_PARK,40,KNEAD_RUN,0,KNOCK_STOP,0,0,SPEED_2,_3D_PROGRAM,_3D_4,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_SHOULDER,0,KNEAD_RUN,0,KNOCK_STOP,0,0,SPEED_2,_3D_PROGRAM,_3D_4,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_PARK,40,KNEAD_RUN,0,KNOCK_STOP,0,0,SPEED_2,_3D_PROGRAM,_3D_4,_3D_SPEED_7,0},
  
  //搓背  
  {BACK_SUB_MODE_RUBBING,WALK_LOCATE_ABSULATE,0,KNEAD_RUN_RUBBING,0,KNOCK_STOP,0,0,SPEED_5,_3D_PROGRAM,_3D_4,_3D_SPEED_7,0},
  {BACK_SUB_MODE_RUBBING,WALK_LOCATE_PressNeck,0,KNEAD_RUN_RUBBING,0,KNOCK_STOP,0,0,SPEED_5,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},
  {BACK_SUB_MODE_RUBBING,WALK_LOCATE_ABSULATE,WAIST_POSITION,KNEAD_RUN_RUBBING,0,KNOCK_STOP,0,0,SPEED_4,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},
  {BACK_SUB_MODE_RUBBING,WALK_LOCATE_PressNeck,0,KNEAD_RUN_RUBBING,0,KNOCK_STOP,0,0,SPEED_4,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},
  {BACK_SUB_MODE_RUBBING,WALK_LOCATE_ABSULATE,WAIST_POSITION,KNEAD_RUN_RUBBING,0,KNOCK_STOP,0,0,SPEED_4,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},

  //在腰部来回揉捏
  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_WAIST,0,KNEAD_RUN,0,KNOCK_STOP,0,0,SPEED_3,_3D_PROGRAM,_3D_3,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_PARK,40,KNEAD_RUN,0,KNOCK_STOP,0,0,SPEED_3,_3D_PROGRAM,_3D_4,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNEAD,WALK_SHOULDER_WAIST_2_10,0,KNEAD_RUN,0,KNOCK_STOP,0,0,SPEED_3,_3D_PROGRAM,_3D_4,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_PARK,40,KNEAD_RUN,0,KNOCK_STOP,0,0,SPEED_3,_3D_PROGRAM,_3D_4,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNEAD,WALK_SHOULDER_WAIST_4_10,80,KNEAD_RUN,0,KNOCK_STOP,0,0,SPEED_3,_3D_PROGRAM,_3D_4,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_PARK,40,KNEAD_RUN,0,KNOCK_STOP,0,0,SPEED_3,_3D_PROGRAM,_3D_3,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNEAD,WALK_SHOULDER_WAIST_2_10,20,KNEAD_RUN,0,KNOCK_STOP,0,0,SPEED_3,_3D_PROGRAM,_3D_2,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_PARK,30,KNEAD_RUN,0,KNOCK_STOP,0,0,SPEED_3,_3D_PROGRAM,_3D_1,_3D_SPEED_7,0},
  
  //往返敲打
  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_PARK,0,KNEAD_RUN_STOP_AT_MIN,1,KNOCK_STOP,0,0,SPEED_3,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNOCK,WALK_LOCATE_ABSULATE,80,KNEAD_STOP,0,KNOCK_RUN,0,0,SPEED_2,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNOCK,WALK_LOCATE_PressNeck,0,KNEAD_STOP,0,KNOCK_RUN,0,0,SPEED_2,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNOCK,WALK_LOCATE_ABSULATE,40,KNEAD_STOP,0,KNOCK_RUN,0,0,SPEED_2,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNOCK,WALK_LOCATE_ABSULATE,80,KNEAD_STOP,0,KNOCK_RUN,0,0,SPEED_2,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNOCK,WALK_LOCATE_WAIST,0,KNEAD_STOP,0,KNOCK_RUN,0,0,SPEED_2,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},
  
  //Waist
  {BACK_SUB_MODE_KNOCK,WALK_LOCATE_PARK,30,KNEAD_STOP,0,KNOCK_RUN,0,0,SPEED_2,_3D_PROGRAM,_3D_3,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNOCK,WALK_LOCATE_PARK,40,KNEAD_STOP,0,KNOCK_RUN,0,0,SPEED_3,_3D_PROGRAM,_3D_4,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNOCK,WALK_LOCATE_PARK,30,KNEAD_STOP,0,KNOCK_RUN,0,0,SPEED_2,_3D_PROGRAM,_3D_4,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_PARK,0,KNEAD_RUN_STOP_AT_MAX,5,KNOCK_STOP,0,0,SPEED_3,_3D_PROGRAM,_3D_3,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNOCK,WALK_LOCATE_PARK,30,KNEAD_STOP,0,KNOCK_RUN,0,0,SPEED_2,_3D_PROGRAM,_3D_4,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNOCK,WALK_LOCATE_PARK,40,KNEAD_STOP,0,KNOCK_RUN,0,0,SPEED_3,_3D_PROGRAM,_3D_3,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNOCK,WALK_LOCATE_PARK,30,KNEAD_STOP,0,KNOCK_RUN,0,0,SPEED_2,_3D_PROGRAM,_3D_4,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_PARK,0,KNEAD_RUN_STOP_AT_MIN,5,KNOCK_STOP,0,0,SPEED_3,_3D_PROGRAM,_3D_3,_3D_SPEED_7,0},
  //全程敲打到顶		
  {BACK_SUB_MODE_KNOCK,WALK_LOCATE_TOP,0,KNEAD_STOP,0,KNOCK_RUN,0,0,SPEED_1,_3D_PROGRAM,_3D_2,_3D_SPEED_7,0},
  //低速敲打到肩部
  {BACK_SUB_MODE_KNOCK,WALK_LOCATE_SHOULDER,0,KNEAD_STOP,0,KNOCK_RUN,0,0,SPEED_1,_3D_PROGRAM,_3D_4,_3D_SPEED_7,0},
  //低速揉捏到顶
  {BACK_SUB_MODE_KNOCK,WALK_LOCATE_TOP,0,KNEAD_STOP,0,KNOCK_RUN,0,0,SPEED_2,_3D_PROGRAM,_3D_4,_3D_SPEED_7,0},
  //往返叩击
  {BACK_SUB_MODE_SOFT_KNOCK,WALK_LOCATE_SHOULDER,0,KNEAD_STOP,0,KNOCK_RUN_STOP,1,4,SPEED_3,_3D_PROGRAM,_3D_4,_3D_SPEED_7,0},
  {BACK_SUB_MODE_SOFT_KNOCK,WALK_LOCATE_TOP,0,KNEAD_STOP,0,KNOCK_RUN_STOP,1,4,SPEED_3,_3D_PROGRAM,_3D_4,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_NeckMed,0,KNEAD_RUN,0,KNOCK_STOP,0,0,SPEED_3,_3D_PROGRAM,_3D_4,_3D_SPEED_7,0},
  {BACK_SUB_MODE_SOFT_KNOCK,WALK_LOCATE_SHOULDER,0,KNEAD_STOP,0,KNOCK_RUN_STOP,1,4,SPEED_3,_3D_PROGRAM,_3D_4,_3D_SPEED_7,0},
  {BACK_SUB_MODE_SOFT_KNOCK,WALK_LOCATE_PressNeck,0,KNEAD_STOP,0,KNOCK_RUN_STOP,1,4,SPEED_3,_3D_PROGRAM,_3D_3,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_NeckMed,0,KNEAD_RUN,0,KNOCK_STOP,0,0,SPEED_3,_3D_PROGRAM,_3D_4,_3D_SPEED_7,0},
  {BACK_SUB_MODE_SOFT_KNOCK,WALK_LOCATE_ABSULATE,80,KNEAD_STOP,0,KNOCK_RUN_STOP,1,4,SPEED_3,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},
  {BACK_SUB_MODE_SOFT_KNOCK,WALK_LOCATE_PressNeck,0,KNEAD_STOP,0,KNOCK_RUN_STOP,1,4,SPEED_3,_3D_PROGRAM,_3D_4,_3D_SPEED_7,0},
  {BACK_SUB_MODE_SOFT_KNOCK,WALK_LOCATE_ABSULATE,40,KNEAD_STOP,0,KNOCK_RUN_STOP,1,4,SPEED_3,_3D_PROGRAM,_3D_3,_3D_SPEED_7,0},
  {BACK_SUB_MODE_SOFT_KNOCK,WALK_LOCATE_ABSULATE,80,KNEAD_STOP,0,KNOCK_RUN_STOP,1,4,SPEED_3,_3D_PROGRAM,_3D_4,_3D_SPEED_7,0},
  {BACK_SUB_MODE_SOFT_KNOCK,WALK_LOCATE_ABSULATE,0,KNEAD_STOP,0,KNOCK_RUN_STOP,1,4,SPEED_3,_3D_PROGRAM,_3D_3,_3D_SPEED_7,0},
  //屁股
  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_PARK,0,KNEAD_RUN_STOP_AT_MIN,5,KNOCK_STOP,0,0,SPEED_3,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNOCK,WALK_LOCATE_PARK,30,KNEAD_STOP,0,KNOCK_RUN,0,0,SPEED_2,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNOCK,WALK_LOCATE_PARK,35,KNEAD_STOP,0,KNOCK_RUN,0,0,SPEED_3,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNOCK,WALK_LOCATE_PARK,40,KNEAD_STOP,0,KNOCK_RUN,0,0,SPEED_4,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_PARK,0,KNEAD_RUN_STOP_AT_MED,5,KNOCK_STOP,0,0,SPEED_3,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNOCK,WALK_LOCATE_PARK,30,KNEAD_STOP,0,KNOCK_RUN,0,0,SPEED_2,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNOCK,WALK_LOCATE_PARK,35,KNEAD_STOP,0,KNOCK_RUN,0,0,SPEED_3,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNOCK,WALK_LOCATE_PARK,40,KNEAD_STOP,0,KNOCK_RUN,0,0,SPEED_4,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_PARK,0,KNEAD_RUN_STOP_AT_MAX,5,KNOCK_STOP,0,0,SPEED_3,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNOCK,WALK_LOCATE_PARK,30,KNEAD_STOP,0,KNOCK_RUN,0,0,SPEED_2,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNOCK,WALK_LOCATE_PARK,35,KNEAD_STOP,0,KNOCK_RUN,0,0,SPEED_3,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNOCK,WALK_LOCATE_PARK,40,KNEAD_STOP,0,KNOCK_RUN,0,0,SPEED_4,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_PARK,50,KNEAD_RUN,0,KNOCK_STOP,0,0,SPEED_3,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},
  
  //搓背  
  {BACK_SUB_MODE_RUBBING,WALK_LOCATE_ABSULATE,0,KNEAD_RUN_RUBBING,0,KNOCK_STOP,0,0,SPEED_4,_3D_PROGRAM,_3D_4,_3D_SPEED_7,0},
  {BACK_SUB_MODE_RUBBING,WALK_LOCATE_PressNeck,0,KNEAD_RUN_RUBBING,0,KNOCK_STOP,0,0,SPEED_4,_3D_PROGRAM,_3D_4,_3D_SPEED_7,0},
  {BACK_SUB_MODE_RUBBING,WALK_LOCATE_ABSULATE,WAIST_POSITION,KNEAD_RUN_RUBBING,0,KNOCK_STOP,0,0,SPEED_4,_3D_PROGRAM,_3D_4,_3D_SPEED_7,0},
  {BACK_SUB_MODE_RUBBING,WALK_LOCATE_PressNeck,0,KNEAD_RUN_RUBBING,0,KNOCK_STOP,0,0,SPEED_4,_3D_PROGRAM,_3D_4,_3D_SPEED_7,0},
  {BACK_SUB_MODE_RUBBING,WALK_LOCATE_ABSULATE,WAIST_POSITION,KNEAD_RUN_RUBBING,0,KNOCK_STOP,0,0,SPEED_4,_3D_PROGRAM,_3D_3,_3D_SPEED_7,0},
 // {BACK_SUB_MODE_RUBBING,WALK_LOCATE_PressNeck,0,KNEAD_RUN_RUBBING,0,KNOCK_STOP,0,0,SPEED_4,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},
 // {BACK_SUB_MODE_RUBBING,WALK_LOCATE_ABSULATE,WAIST_POSITION,KNEAD_RUN_RUBBING,0,KNOCK_STOP,0,0,SPEED_4,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},


} ;
*/

/*  自动程序数组
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
} WALK_KNEAD_KNOCK_MOTOR_STRUCT_AUTO ;

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



#define KNOCK_STOP		0 //停止
#define KNOCK_RUN_WIDTH		1 //宽中窄定位完成后启动
#define KNOCK_RUN		2 //无需宽中窄定位，无条件立即启动
#define KNOCK_RUN_STOP		3 //宽中窄定位完成后启动短时间后马上停止
#define KNOCK_RUN_MUSIC		4 //音乐互动模式（与宽中窄定位无关）
//------------------------------------
#define _3D_MANUAL      0   //依据用户设定3D力度动作
#define _3D_PROGRAM     1   //依据设定的3D位置动作
#define _3D_CURRENT     2   //依据电流检测位置动作
#define _3D_PARK        3   //3D马达停留在当前位置
#define AXIS_STRONGEST 4  
#define AXIS_STRONGER  3
#define AXIS_MIDDLE    2
#define AXIS_WEAKER    1
#define AXIS_WEAKEST   0

//这个参数的单位是10ms,是指脉冲宽度

#define _3D_SPEED_1 16
#define _3D_SPEED_2 15
#define _3D_SPEED_3 14
#define _3D_SPEED_4 13
#define _3D_SPEED_5 12
#define _3D_SPEED_4 11
#define _3D_SPEED_7 10
#define _3D_SPEED_8 9
#define _3D_SPEED_9 8
#define _3D_SPEED_10 7



*/

//轻松按摩(有肩部位置检测),力度较弱//307
const WALK_KNEAD_KNOCK_MOTOR_STRUCT_AUTO AutoFunction2[] = 
{	
  
  
  
  //以下为正常动作	
  //揉捶同步到底部（低速）
  {BACK_SUB_MODE_WAVELET,WALK_LOCATE_ABSULATE,0,KNEAD_RUN,0,KNOCK_RUN,0,0,SPEED_1,_3D_PROGRAM,_3D_3,_3D_SPEED_7,0},
  //揉捏到顶部（低速）	
  {BACK_SUB_MODE_KNOCK,WALK_LOCATE_TOP,0,KNEAD_STOP,0,KNOCK_RUN,0,0,SPEED_2,_3D_PROGRAM,_3D_2,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_PARK,0,KNEAD_RUN_STOP_AT_MIN,3,KNOCK_STOP,0,0,SPEED_2,_3D_PROGRAM,_3D_4,_3D_SPEED_7,0},
  
  //颈部来回敲打
  {BACK_SUB_MODE_KNOCK,WALK_LOCATE_SHOULDER,0,KNEAD_STOP,0,KNOCK_RUN,0,0,SPEED_1,_3D_PROGRAM,_3D_2,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNOCK,WALK_LOCATE_TOP,0,KNEAD_STOP,0,KNOCK_RUN,0,0,SPEED_1,_3D_PROGRAM,_3D_3,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNOCK,WALK_LOCATE_SHOULDER,0,KNEAD_STOP,0,KNOCK_RUN,0,0,SPEED_1,_3D_PROGRAM,_3D_2,_3D_SPEED_7,0},	
  {BACK_SUB_MODE_KNOCK,WALK_LOCATE_TOP,0,KNEAD_STOP,0,KNOCK_RUN,0,0,SPEED_1,_3D_PROGRAM,_3D_3,_3D_SPEED_7,0},
  //颈部来回揉捏		
  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_PARK,40,KNEAD_RUN,0,KNOCK_STOP,0,0,SPEED_2,_3D_PROGRAM,_3D_3,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_SHOULDER,20,KNEAD_RUN,0,KNOCK_STOP,0,0,SPEED_2,_3D_PROGRAM,_3D_4,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_PARK,40,KNEAD_RUN,0,KNOCK_STOP,0,0,SPEED_2,_3D_PROGRAM,_3D_4,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_SHOULDER,0,KNEAD_RUN,0,KNOCK_STOP,0,0,SPEED_2,_3D_PROGRAM,_3D_4,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_PARK,40,KNEAD_RUN,0,KNOCK_STOP,0,0,SPEED_2,_3D_PROGRAM,_3D_4,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_NeckMed,0,KNEAD_RUN,0,KNOCK_STOP,0,0,SPEED_3,_3D_PROGRAM,_3D_4,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_PARK,40,KNEAD_RUN,0,KNOCK_STOP,0,0,SPEED_2,_3D_PROGRAM,_3D_4,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_SHOULDER,0,KNEAD_RUN,0,KNOCK_STOP,0,0,SPEED_2,_3D_PROGRAM,_3D_4,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_PARK,40,KNEAD_RUN,0,KNOCK_STOP,0,0,SPEED_2,_3D_PROGRAM,_3D_4,_3D_SPEED_7,0},
  
  //搓背  
  {BACK_SUB_MODE_RUBBING,WALK_LOCATE_ABSULATE,0,KNEAD_RUN_RUBBING,0,KNOCK_STOP,0,0,SPEED_5,_3D_PROGRAM,_3D_4,_3D_SPEED_7,0},
  {BACK_SUB_MODE_RUBBING,WALK_LOCATE_PressNeck,0,KNEAD_RUN_RUBBING,0,KNOCK_STOP,0,0,SPEED_5,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},
  {BACK_SUB_MODE_RUBBING,WALK_LOCATE_ABSULATE,WAIST_POSITION,KNEAD_RUN_RUBBING,0,KNOCK_STOP,0,0,SPEED_4,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},
  {BACK_SUB_MODE_RUBBING,WALK_LOCATE_PressNeck,0,KNEAD_RUN_RUBBING,0,KNOCK_STOP,0,0,SPEED_4,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},
  {BACK_SUB_MODE_RUBBING,WALK_LOCATE_ABSULATE,WAIST_POSITION,KNEAD_RUN_RUBBING,0,KNOCK_STOP,0,0,SPEED_4,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},

  //在腰部来回揉捏
  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_WAIST,0,KNEAD_RUN,0,KNOCK_STOP,0,0,SPEED_3,_3D_PROGRAM,_3D_3,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_PARK,40,KNEAD_RUN,0,KNOCK_STOP,0,0,SPEED_3,_3D_PROGRAM,_3D_4,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNEAD,WALK_SHOULDER_WAIST_2_10,0,KNEAD_RUN,0,KNOCK_STOP,0,0,SPEED_3,_3D_PROGRAM,_3D_4,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_PARK,40,KNEAD_RUN,0,KNOCK_STOP,0,0,SPEED_3,_3D_PROGRAM,_3D_4,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNEAD,WALK_SHOULDER_WAIST_4_10,80,KNEAD_RUN,0,KNOCK_STOP,0,0,SPEED_3,_3D_PROGRAM,_3D_4,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_PARK,40,KNEAD_RUN,0,KNOCK_STOP,0,0,SPEED_3,_3D_PROGRAM,_3D_3,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNEAD,WALK_SHOULDER_WAIST_2_10,20,KNEAD_RUN,0,KNOCK_STOP,0,0,SPEED_3,_3D_PROGRAM,_3D_2,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_PARK,30,KNEAD_RUN,0,KNOCK_STOP,0,0,SPEED_3,_3D_PROGRAM,_3D_1,_3D_SPEED_7,0},
  
  //往返敲打
  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_PARK,0,KNEAD_RUN_STOP_AT_MIN,1,KNOCK_STOP,0,0,SPEED_3,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNOCK,WALK_LOCATE_ABSULATE,80,KNEAD_STOP,0,KNOCK_RUN,0,0,SPEED_2,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNOCK,WALK_LOCATE_PressNeck,0,KNEAD_STOP,0,KNOCK_RUN,0,0,SPEED_2,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNOCK,WALK_LOCATE_ABSULATE,40,KNEAD_STOP,0,KNOCK_RUN,0,0,SPEED_2,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNOCK,WALK_LOCATE_ABSULATE,80,KNEAD_STOP,0,KNOCK_RUN,0,0,SPEED_2,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNOCK,WALK_LOCATE_WAIST,0,KNEAD_STOP,0,KNOCK_RUN,0,0,SPEED_2,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},
  
  //Waist
  {BACK_SUB_MODE_KNOCK,WALK_LOCATE_PARK,30,KNEAD_STOP,0,KNOCK_RUN,0,0,SPEED_2,_3D_PROGRAM,_3D_3,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNOCK,WALK_LOCATE_PARK,40,KNEAD_STOP,0,KNOCK_RUN,0,0,SPEED_3,_3D_PROGRAM,_3D_4,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNOCK,WALK_LOCATE_PARK,30,KNEAD_STOP,0,KNOCK_RUN,0,0,SPEED_2,_3D_PROGRAM,_3D_4,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_PARK,0,KNEAD_RUN_STOP_AT_MAX,5,KNOCK_STOP,0,0,SPEED_3,_3D_PROGRAM,_3D_3,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNOCK,WALK_LOCATE_PARK,30,KNEAD_STOP,0,KNOCK_RUN,0,0,SPEED_2,_3D_PROGRAM,_3D_4,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNOCK,WALK_LOCATE_PARK,40,KNEAD_STOP,0,KNOCK_RUN,0,0,SPEED_3,_3D_PROGRAM,_3D_3,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNOCK,WALK_LOCATE_PARK,30,KNEAD_STOP,0,KNOCK_RUN,0,0,SPEED_2,_3D_PROGRAM,_3D_4,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_PARK,0,KNEAD_RUN_STOP_AT_MIN,5,KNOCK_STOP,0,0,SPEED_3,_3D_PROGRAM,_3D_3,_3D_SPEED_7,0},
  //全程敲打到顶		
  {BACK_SUB_MODE_KNOCK,WALK_LOCATE_TOP,0,KNEAD_STOP,0,KNOCK_RUN,0,0,SPEED_1,_3D_PROGRAM,_3D_2,_3D_SPEED_7,0},
  //低速敲打到肩部
  {BACK_SUB_MODE_KNOCK,WALK_LOCATE_SHOULDER,0,KNEAD_STOP,0,KNOCK_RUN,0,0,SPEED_1,_3D_PROGRAM,_3D_4,_3D_SPEED_7,0},
  //低速揉捏到顶
  {BACK_SUB_MODE_KNOCK,WALK_LOCATE_TOP,0,KNEAD_STOP,0,KNOCK_RUN,0,0,SPEED_2,_3D_PROGRAM,_3D_4,_3D_SPEED_7,0},
  //往返叩击
  {BACK_SUB_MODE_SOFT_KNOCK,WALK_LOCATE_SHOULDER,0,KNEAD_STOP,0,KNOCK_RUN_STOP,1,4,SPEED_3,_3D_PROGRAM,_3D_4,_3D_SPEED_7,0},
  {BACK_SUB_MODE_SOFT_KNOCK,WALK_LOCATE_TOP,0,KNEAD_STOP,0,KNOCK_RUN_STOP,1,4,SPEED_3,_3D_PROGRAM,_3D_4,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_NeckMed,0,KNEAD_RUN,0,KNOCK_STOP,0,0,SPEED_3,_3D_PROGRAM,_3D_4,_3D_SPEED_7,0},
  {BACK_SUB_MODE_SOFT_KNOCK,WALK_LOCATE_SHOULDER,0,KNEAD_STOP,0,KNOCK_RUN_STOP,1,4,SPEED_3,_3D_PROGRAM,_3D_4,_3D_SPEED_7,0},
  {BACK_SUB_MODE_SOFT_KNOCK,WALK_LOCATE_PressNeck,0,KNEAD_STOP,0,KNOCK_RUN_STOP,1,4,SPEED_3,_3D_PROGRAM,_3D_3,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_NeckMed,0,KNEAD_RUN,0,KNOCK_STOP,0,0,SPEED_3,_3D_PROGRAM,_3D_4,_3D_SPEED_7,0},
  {BACK_SUB_MODE_SOFT_KNOCK,WALK_LOCATE_ABSULATE,80,KNEAD_STOP,0,KNOCK_RUN_STOP,1,4,SPEED_3,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},
  {BACK_SUB_MODE_SOFT_KNOCK,WALK_LOCATE_PressNeck,0,KNEAD_STOP,0,KNOCK_RUN_STOP,1,4,SPEED_3,_3D_PROGRAM,_3D_4,_3D_SPEED_7,0},
  {BACK_SUB_MODE_SOFT_KNOCK,WALK_LOCATE_ABSULATE,40,KNEAD_STOP,0,KNOCK_RUN_STOP,1,4,SPEED_3,_3D_PROGRAM,_3D_3,_3D_SPEED_7,0},
  {BACK_SUB_MODE_SOFT_KNOCK,WALK_LOCATE_ABSULATE,80,KNEAD_STOP,0,KNOCK_RUN_STOP,1,4,SPEED_3,_3D_PROGRAM,_3D_4,_3D_SPEED_7,0},
  {BACK_SUB_MODE_SOFT_KNOCK,WALK_LOCATE_ABSULATE,0,KNEAD_STOP,0,KNOCK_RUN_STOP,1,4,SPEED_3,_3D_PROGRAM,_3D_3,_3D_SPEED_7,0},
  //屁股
  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_PARK,0,KNEAD_RUN_STOP_AT_MIN,5,KNOCK_STOP,0,0,SPEED_3,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNOCK,WALK_LOCATE_PARK,30,KNEAD_STOP,0,KNOCK_RUN,0,0,SPEED_2,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNOCK,WALK_LOCATE_PARK,35,KNEAD_STOP,0,KNOCK_RUN,0,0,SPEED_3,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNOCK,WALK_LOCATE_PARK,40,KNEAD_STOP,0,KNOCK_RUN,0,0,SPEED_4,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_PARK,0,KNEAD_RUN_STOP_AT_MED,5,KNOCK_STOP,0,0,SPEED_3,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNOCK,WALK_LOCATE_PARK,30,KNEAD_STOP,0,KNOCK_RUN,0,0,SPEED_2,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNOCK,WALK_LOCATE_PARK,35,KNEAD_STOP,0,KNOCK_RUN,0,0,SPEED_3,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNOCK,WALK_LOCATE_PARK,40,KNEAD_STOP,0,KNOCK_RUN,0,0,SPEED_4,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_PARK,0,KNEAD_RUN_STOP_AT_MAX,5,KNOCK_STOP,0,0,SPEED_3,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNOCK,WALK_LOCATE_PARK,30,KNEAD_STOP,0,KNOCK_RUN,0,0,SPEED_2,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNOCK,WALK_LOCATE_PARK,35,KNEAD_STOP,0,KNOCK_RUN,0,0,SPEED_3,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNOCK,WALK_LOCATE_PARK,40,KNEAD_STOP,0,KNOCK_RUN,0,0,SPEED_4,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_PARK,50,KNEAD_RUN,0,KNOCK_STOP,0,0,SPEED_3,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},
  
  //搓背  
  {BACK_SUB_MODE_RUBBING,WALK_LOCATE_ABSULATE,0,KNEAD_RUN_RUBBING,0,KNOCK_STOP,0,0,SPEED_4,_3D_PROGRAM,_3D_4,_3D_SPEED_7,0},
  {BACK_SUB_MODE_RUBBING,WALK_LOCATE_PressNeck,0,KNEAD_RUN_RUBBING,0,KNOCK_STOP,0,0,SPEED_4,_3D_PROGRAM,_3D_4,_3D_SPEED_7,0},
  {BACK_SUB_MODE_RUBBING,WALK_LOCATE_ABSULATE,WAIST_POSITION,KNEAD_RUN_RUBBING,0,KNOCK_STOP,0,0,SPEED_4,_3D_PROGRAM,_3D_4,_3D_SPEED_7,0},
  {BACK_SUB_MODE_RUBBING,WALK_LOCATE_PressNeck,0,KNEAD_RUN_RUBBING,0,KNOCK_STOP,0,0,SPEED_4,_3D_PROGRAM,_3D_4,_3D_SPEED_7,0},
  {BACK_SUB_MODE_RUBBING,WALK_LOCATE_ABSULATE,WAIST_POSITION,KNEAD_RUN_RUBBING,0,KNOCK_STOP,0,0,SPEED_4,_3D_PROGRAM,_3D_3,_3D_SPEED_7,0},



} ;













