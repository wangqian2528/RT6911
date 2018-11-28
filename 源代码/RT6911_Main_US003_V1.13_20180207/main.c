

/******************************************************************************
小腿 TIMER0     CC0 ,       #2 
零重力  TIMER0  CC2 ,       #2
靠背   TIMER0   CC1,        #2



行走  TIMER1   CC0,         #2
摇摆  TIMER1   CC1          #2


敲击   TIMER2 ,    CC0 ,         #0
3D电机 TIMER2 ,    CC2,          #0
揉捏   TIMER2 ,    CC1           #0

LED1     TIMER3   CC0  ,        #1
LED2     TIMER3   CC1           #1
LED3     TIMER3   CC2           #1
unsigned int BackMotor_VoltageAdj(unsigned int setDuty)  //靠背电机中该函数有问题，目前还没有处理

*****************************************************************************/



//RT8302T   modify by taoqingsong 2015-5-14
//BOOTLAODER程序会丢失。
//工程模式测试
//修改靠背、小腿、零重力电动缸模块
//这个版本软件 单独控制零重力电动缸，未考虑靠背、小腿、零重力电动缸模块之间的联动
//该版本所有电动缸都能正常工作

//重新初始化//修改靠背、小腿、零重力电动缸模块

//该版本目前工程模式基本正常，行走上下键有时不正常
//new
//修改主板控制24个气阀，不需小腿板控制气阀
//该版本软件气阀进行简单的替换，用的是8600S的气阀程序，测试用  ,工程模式测试OK
//该版本将气囊程序改为8305A的气囊程序，软件还需要调试2015-6-9
 //靠背电机测试ok
//除3D自动程序外，其他正常
//8600S肩位范围 125-250   ，最大坐标为397
//修改软件增加为3个气帮
//将行走，揉捏，敲等电机电压调到24V 2015-6-23

//靠背、小腿、零重力电动缸电机电压偏小 
//靠背、小腿、零重力电动缸电机电压偏小  处理完毕  #define SLIDE_SET_VOLTAGE    280000//250000 //25V
//调试行走电机坐标与手控器对不上问题点
//add 自动更新网络程序功能
//测试 自动更新网络程序功能2015-6-27

//该方案使用XOMDOEM协议传输数据时关闭整个中断
//这个版本出错时发送NAK 给上位机，上位机没有继续进行传输

/*将波特率配置为9600，同时关闭写FLASH，上位机软件才可以正常写进去*/

//该版本单步仿真时软件可以写进入FLASH----
//写FLASH测试OK
//写FLASH OK2--可以循环进行写FLASH
//x写FALSH 超时处理检测和循环发送测试OK
//该程序可以更新云端地址1和地址2的自动程序
//该版本可以跟平板电脑相连，蓝牙名为RT8600S-008
//该版本MEMORY函数用于云端更新的函数没有被移到bluetooth.c模块中
//增加XOMDEM校验和
//增加XOMDEM校验和 test ok
//该逻辑是老的XMODEM函数来下载，
//处理找脚和拉升程序
//--
//可以正常进入退出蓝牙更新模块 
//修改软件，FLASH 一次只能插除2K
//将蓝牙模块改成文件形式
//修改肩位检测
//体型检测OK

//替换void Main_Start_Manual(void)
//处理找脚程序的BUG
//BEEP声上传给主板正常
//修改向外拉升程序
//修改气囊程序名称与APP相对应
//参考王贵华睡眠模式程序将3D动作时的3D速度调小
//增加主板上传ID号给APP,test ok
//ADD DEMO程序
//将拉伸程序改为只往下拉，（因个子过高的人向外拉升时无法伸直）
//正常按摩时将椅子位置定位在第一零重力位置处
//肩位检测不对，重新设计肩位检测程序

//靠背，小腿，零重力有动作，暂时关闭摇摆，
//增加靠背移动后肩位检测的补偿 ,test ok
//测试180V-250V气囊充气
//测试3D电机电流大小 ,3D电流可以检测，只是不太稳定。
//行走电机坐标大于230时，肩部气囊关闭，要不然人会不舒服
//
//取消自动程序时靠背在零重力位置，改为第二零重力点


//处理小腿和零重力电压250V 偏低问题点

//处理工程模式自动检测摇摆电机不动问题点

 // GPIO_PinOutClear(BACK_MOTOR_DECAY_PORT, BACK_MOTOR_DECAY_BIT);
 //   GPIO_PinOutSet(BACK_MOTOR_DECAY_PORT, BACK_MOTOR_DECAY_BIT);//为高电平时，250V ，靠背电压会下降到15V
//处理8305T 小腿伸缩无法自检问题
//处理休息促眠模式手臂不显示问题
//自动检测，椅子不在复位状态时，检测不到角度开关

//处理体型检测时左手臂充3次，右边充一次现象
//处理角度开关位置，报警声不对问题点
//2015-10-11找脚程序
//处理自动6显示洗敲揉显示不对问题，
//zaici jiancha找脚程序go o
//go on  foot  c
//处理体型检测未结束就开始拉腿
//处理工作减压揉捏运行结束后返回第一行显示揉捏实际上揉捏动作没进行是挤压的感觉 2015-10-19

/*


void Main_BlueToothSend(void)
  return;//
20160624
修改 APP
*/
/*
20170207  20170208...........
3.自动模式里的上半身按摩，肩部气囊动作多一些
4.手动气囊选择，由单选改为复选
5.添加5分钟DEMO
6.拉伸控制在体型检测1分钟后开始
*/

#include "efm32.h"
#include "em_cmu.h"
#include "em_chip.h"
#include "em_gpio.h"

#include "em_msc.h"
#include "EFM32_def.h"
#include "EFM32_types.h"

#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>

//#include "efm32_aes.h"


#include "em_aes.h"









#include "ControlBox.h"
#include "backaction.h"

#include "autoFunction0.c"
#include "autoFunction1.c"
#include "autoFunction2.c"
#include "autoFunction3.c"
#include "autoFunction4.c"
#include "autoFunction5.c"
#include "DemoFunction.c"
#include "Demo6MinFunction.c"
#include "_3DFunction0.c"
#include "_3DFunction1.c"
#include "_3DFunction2.c"
#include "AutoFunctionStretch.c"

#include "airbagaction.h"

#include "testCommand.h"
#include "system.h"
#include "ADC_Scan.h"
#include "AxisMotor.h"
#include "BackPad.h"
#include "KnockMotor_8840.h"
#include "IndicateLED.h"
#include "input.h"
#include "HandUart.h"
#include "KneadMotor.h"
#include "LED_RGB.h"
#include "LegMotor.h"
#include "memory.h"
#include "Mp3Ctrl.h" 
#include "power.h"
#include "SlideMotor.h"
#include "Valve.h"
#include "WalkMotor.h"
#include "WaistHot.h"
#include "UartLeg.h"
#include "LegKnead_Uart.h"
#include "Data_Cul.h"
#include "timer.h"
#include "Roller_Uart.h"
#include "Flex_Uart.h"
#include "LegKnead_Uart.h"
#include "BlueTooth.h"
#include "LEUart_DMA.h"
#include "DMA_Ctrl.h"
#include "VoiceUart.h"
#include "problem.h"        
#include "main.h"
#include "waver_Flex_Motor.h"
#include "xmodem.h"


extern unsigned char  _3D_Max_Position ; 
extern unsigned char  _3D_More_Men_Position; 
extern unsigned char  _3D_Men_Position; 
extern unsigned char  _3D_More_Min_Position; 
extern unsigned char  _3D_Min_Position; 

unsigned char nStretchVigor;
unsigned int presstime;
unsigned char nKneadTurn ;
/******************************************************************/
unsigned int w_walk_pos;
//char by_t;
extern bool bRollerDisplay ;
__no_init BITS GlobalFlags0 ;
#define bKneadWidthMaxPulseLevel0 	GlobalFlags0.bD0
#define bKneadWidthMaxPulseLevel1 	GlobalFlags0.bD1
#define bKneadWidthMaxPulseLevel2 	GlobalFlags0.bD2
#define bKneadWidthMaxPulseLevel3 	GlobalFlags0.bD3
#define bHasKneadWidthMaxPulse	 	GlobalFlags0.bD4//揉捏电机处于中的位置
#define bDisplayKneadWidthMax		GlobalFlags0.bD5
#define bDisplayKneadTrackMax		GlobalFlags0.bD6
#define bUpdateLocate 			GlobalFlags0.bD7//行走电机坐标更新标志，置位时更新一次坐标

__no_init BITS GlobalFlags1 ;
#define bKneadWidthMedPulseLevel0 	GlobalFlags1.bD0
#define bKneadWidthMedPulseLevel1 	GlobalFlags1.bD1
#define bKneadWidthMedPulseLevel2	GlobalFlags1.bD2
#define bKneadWidthMedPulseLevel3 	GlobalFlags1.bD3
#define bHasKneadWidthMedPulse		GlobalFlags1.bD4//揉捏电机处于中的位置
#define bDisplayKneadWidthMed		GlobalFlags1.bD5
#define bDisplayKneadTrackMed		GlobalFlags1.bD6
#define bLegPadLinkage 			GlobalFlags1.bD7   //小腿起落联动标志

__no_init BITS GlobalFlags2 ;
#define bKneadWidthMinPulseLevel0 	GlobalFlags2.bD0
#define bKneadWidthMinPulseLevel1 	GlobalFlags2.bD1
#define bKneadWidthMinPulseLevel2 	GlobalFlags2.bD2
#define bKneadWidthMinPulseLevel3 	GlobalFlags2.bD3
#define bHasKneadWidthMinPulse	 	GlobalFlags2.bD4// bHasKneadWidthMinPulse = TRUE ;//揉捏电机处于窄的位置
#define bDisplayKneadWidthMin		GlobalFlags2.bD5
#define bDisplayKneadTrackMin		GlobalFlags2.bD6
#define bWaveMotorFail 			GlobalFlags2.bD7//摇摆电机故障标志位

__no_init BITS GlobalFlags3 ;
#define bShoulderOK	                GlobalFlags3.bD0
#define bBlueToothStatus		GlobalFlags3.bD1  //蓝牙打开标志位
//#define bKeyPowerSwitch 		GlobalFlags3.bD2
#define bKeyWaistHeat 			GlobalFlags3.bD3
//#define bSlowDisplayFlash		GlobalFlags3.bD4
#define bKeySeatVibrate 		GlobalFlags3.bD5   //坐垫摇摆按键标志
#define bKeySeatEnable 		        GlobalFlags3.bD6   //坐垫摇摆按键标志
#define bVibratePause 		        GlobalFlags3.bD7   //坐垫摇摆按键标志bKeySeatVibrate

//#define bMP3RunMode	 		GlobalFlags3.bD7

//位变量
__no_init BITS GlobalFlags4 ; 
#define bAutoProgramOver 		GlobalFlags4.bD0
#define bTimer10MS 			GlobalFlags4.bD1
#define bZeroTimer100MS 		GlobalFlags4.bD2
#define bBootlooth10ms 		GlobalFlags4.bD3
//#define bKneadMotorPowerFlag 		GlobalFlags4.bD4
//#define bKnockMotorPowerFlag 		GlobalFlags4.bD5
#define bBackLegPadSettle 		GlobalFlags4.bD6
#define bDisplayFlash 			GlobalFlags4.bD7

__no_init BITS GlobalFlags5 ;
#define bBackAutoModeInit 			GlobalFlags5.bD0///自动程序中 机芯  开始动作的初始化标志位，初始化完成后标志位清0
#define bBackManualModeInit 		        GlobalFlags5.bD1//手动程序中  机芯   电机开始动作的初始化标志位
#define bWalkMotorInProcess 		        GlobalFlags5.bD2 //行走电机程序开始执行标志,行走马达开始运行，当运行完毕后标志位清0
#define bKneadMotorInProcess 		        GlobalFlags5.bD3 //揉捏电机程序开始执行标志，例如顺时针揉捏3圈后停止
#define bKnockMotorInProcess 		        GlobalFlags5.bD4 //敲击电机程序执行标志
#define bGetNextActionStep 			GlobalFlags5.bD5//自动程序中获取下一步动作标志位
#define bKeyWalkUp 				GlobalFlags5.bD6   //用户按下向上行走按键标志值，当释放按键时 清0标志位
#define bKeyWalkDown 				GlobalFlags5.bD7   //按下向下行走按键标志值




__no_init BITS GlobalFlags6 ;
#define b3D_MotorInProcess 			GlobalFlags6.bD0//3D马达正在运行标志位
#define bMassagePositionUpdate 			GlobalFlags6.bD1   //零重力电机开始动作的标志位，动作完毕后软件清0
//#define bMarkSpace				GlobalFlags6.bD2
#define bSendBuzzerMode 			GlobalFlags6.bD3   //全局变量，BEEP发音使能
//#define bSignalSendPacket 			GlobalFlags6.bD4
#define bMasterSendPacket 			GlobalFlags6.bD5 



//bReconfigFlag 			GlobalFlags6.bD6
#define bZeroflash                              GlobalFlags6.bD6
#define bKneadWidthChange			GlobalFlags6.bD7






__no_init BITS GlobalFlags7 ;
#define bKeyBackPadUp 				GlobalFlags7.bD0   //用户按下靠背向上按键后设定的标志位
#define bKeyBackPadDown 			GlobalFlags7.bD1//用户按下靠背向下按键后设定的标志位
#define bOzonEnable 	                        GlobalFlags7.bD2
//#define bReachBackPadDownPosition 	        GlobalFlags7.bD3
//#define bBackPadMotorPowerFlag		GlobalFlags7.bD4
//#define bGetAirBagNextStep 			GlobalFlags7.bD5
//#define bCurActionStepChange		        GlobalFlags7.bD6
//#define bWalkLocateChange			GlobalFlags7.bD7

__no_init BITS GlobalFlags8 ;
#define bKeyLegPadUp 				GlobalFlags8.bD0  //用户按下小腿向上按键标志位
#define bKeyLegPadDown 				GlobalFlags8.bD1  //小腿起落电动缸落标志，在按键处理里面设置
#define bKeyFlexOut 		                GlobalFlags8.bD2  //用户按下小腿向外伸按键
#define bKeyFlexIn 	                        GlobalFlags8.bD3   //用户按下小腿向里缩按键
//#define bZeroPadMotorPowerFlag		GlobalFlags8.bD4
//#define bWalkMotorLocateChange 		GlobalFlags8.bD5
//#define bReachWalkUpLimitFlag		        GlobalFlags8.bD6
//#define bReachWalkDownLimitFlag		GlobalFlags8.bD7

__no_init BITS GlobalFlags9 ;
#define b6MinDemoStretch                        GlobalFlags9.bD0
#define b6MinDemoStretchEnable			GlobalFlags9.bD1
//#define bBodyDetectSuccess			GlobalFlags9.bD1
//#define bKeyZeroUp			        GlobalFlags9.bD2
//#define bGetArmAirBagNextStep 		GlobalFlags9.bD3
//#define bZeroTransition			GlobalFlags9.bD4
//#define bZeroRestFlag				GlobalFlags9.bD5
//#define bZeroRunFlag				GlobalFlags9.bD6
//#define bGetBodyUpAirBagNextStep 	        GlobalFlags9.bD7

__no_init BITS GlobalFlags10 ;
//#define bZeroRunUpFlag			GlobalFlags10.bD0
//#define bZeroRunDownFlag			GlobalFlags10.bD1
//#define bMP3_AD_Enable			GlobalFlags10.bD2
//#define bKeyZeroDown    			GlobalFlags10.bD3
//#define bBackMotorUpFlag			GlobalFlags10.bD4
//#define bLegkMotorUpFlag			GlobalFlags10.bD5
#define bBlueToothMasterSendPacket		GlobalFlags10.bD6
#define bBlueToothSendBuzzerMode		GlobalFlags10.bD7


//----------------------------------------------------------------------------------------
//该区域用于云端更新
//__no_init BITS GlobalFlags11 ;
//#define bCloud_MasterSendPacket			        GlobalFlags11.bD0   //200ms timer
//#define bCloud_MasterSendHandlePacket			GlobalFlags11.bD0   //200ms timer

//#define bZeroRunDownFlag			GlobalFlags10.bD1
//#define bMP3_AD_Enable			GlobalFlags10.bD2
//#define bKeyZeroDown    			GlobalFlags10.bD3
//#define bBackMotorUpFlag			GlobalFlags10.bD4
//#define bLegkMotorUpFlag			GlobalFlags10.bD5
//#define bBlueToothMasterSendPacket		GlobalFlags10.bD6
//#define bBlueToothSendBuzzerMode		GlobalFlags10.bD7

//云端程序定义 从88K的地方开始
/*
地址0,1,2,3 程序ID号 ID号为0或0xFFFFFFFF则代表为空
地址4,5 数据长度
地址6,7 数据checksum
*/
#define CLUDE_AUTO_0_BASE  ((uint32_t) (88*1024))
#define CLUDE_AUTO_1_BASE  ((uint32_t) (96*1024))
#define CLUDE_AUTO_2_BASE  ((uint32_t) (104*1024))
#define CLUDE_AUTO_3_BASE  ((uint32_t) (112*1024))
#define CLUDE_AUTO_ID_ADDRESS    0  //32位
#define CLUDE_AUTO_SIZE_ADDRESS  4  //16位
#define CLUDE_AUTO_CHECKSUM_ADDRESS  8//16位
#define CLUDE_AUTO_VERSION_ADDRESS  12//16位
#define CLUDE_AUTO_PROGRAM_OFFSET  32

WALK_KNEAD_KNOCK_MOTOR_STRUCT_AUTO *pCludeAutoFunction_0,*pCludeAutoFunction_1,*pCludeAutoFunction_2,*pCludeAutoFunction_3;
 __no_init unsigned char by_CLUDE_DATA[16];//={0};
 
 __no_init unsigned int BACK_CLOUD_STEPS[4];
               
unsigned int BACK_CLOUD_START_STEP[4]={0,0,0,0};
__no_init unsigned char bZeroTimer100MScount;
__no_init unsigned char nRockModeEnterEnable;
//--------------------------------------------------------------------------------------


//------------mp3--------------------
#define    BLUE_SONG_ON   1
#define    USB_SONG_ON    2
#define    AUX_SONG_ON   3
unsigned char n_usb_indicate;
unsigned char n_usb_send_buf[10];
unsigned int   n_mp3_key_old;
unsigned int   n_mp3_key;
unsigned  char nRockMode;
//MP3 变量
extern unsigned char currentBackPadMotorState;
/******************************************************************/
#define MAX_INBUFFER_COUNT			10
#define MAX_OUTBUFFER_COUNT			20
/******************************************************************/
#define MAX_WAIT_COMMAND_TIME		120 //120*0.5s=60s=1min

__no_init unsigned char nSettleMode;
/******************************************************************/
__no_init StretchStruct st_Stretch;
__no_init StretchStruct st_Stretch1;
/******************************************************************/
__no_init unsigned char nBuzzerMode;//beep发音的模式
/******************************************************************/

/*
#define BACK_SUB_MODE_KNEAD			7
#define BACK_SUB_MODE_KNOCK			8
#define BACK_SUB_MODE_PRESS			9
#define BACK_SUB_MODE_WAVELET			10// //揉捏同步
#define BACK_SUB_MODE_SOFT_KNOCK		11////叩击
#define BACK_SUB_MODE_MUSIC			12////韵律按摩
*/
 //         nBackMainRunMode = BACK_MAIN_MODE_AUTO;
 //        nBackSubRunMode = BACK_SUB_MODE_AUTO_5;
__no_init unsigned char nBackMainRunMode,nBackSubRunMode ;
__no_init unsigned char nCurSubFunction ;     //nCurSubFunction为当中的{BACK_SUB_MODE_KNEAD...BACK_SUB_MODE_MUSIC}之间的一种按摩手法
__no_init unsigned char nCurKneadKnockSpeed ; 
/******************************************************************/
__no_init unsigned int nCurActionStep,nPreActionStep ;  // 自动程序当前动作步数
__no_init unsigned int nMaxActionStep ;//自动程序最大动作步数
__no_init unsigned char nStartActionStep ;//自动程序开始步数，默认为0
/******************************************************************/
__no_init unsigned char nTargetMassagePosition;//机器归位时，零重力电机， 靠背，小腿上下，小腿伸缩电机的位置
__no_init unsigned short nShoulderPosition,nShoulderPositionTop,nShoulderPositionBottom;//肩部位置绝对坐标，肩部位置最高点，最低点，在肩位检测后得到正确的肩部位置
__no_init unsigned int ShoulderSteps;
__no_init int BodyDetectStep;   
//BODY_DETECT_PREPARE:  未开始检测 
//BODY_DETECT_SHOULDER: 正在检测肩膀位置
//BODY_DETECT_3D:       正在检测3D力度 
//BODY_DETECT_OVER:     检测完成 

/******************************************************************/
//控制手控器需要知道的相关信息的变量
__no_init unsigned char nKeySeatVibrateStrength, nKeySeatVibrateStrength_old;//振动力度，对应outbuf[3]3,4,5位 ，摇摆电机力度
//static bool Enable_Vibrate=FALSE; 
extern unsigned int nRoller3sCnt;
__no_init unsigned char nKeyBackLocate;//机芯按摩部位，对应outbuf[4]5,6位    ,全局，定点，局部按摩
__no_init unsigned int w_PresetTime;  //程序预设时间，对应outbuf[12]0,1位
__no_init unsigned char nKeyAirBagLocate ;    //气囊按摩区域,对应outbuf[12]2.3.4位，选中的气囊按摩程序,对应通信协议的 地址12 时间和气囊
/******************************************************************/
 bool bTapping;       
unsigned short nLegAngle;
extern unsigned char nFlexStatus;
unsigned char nLegReady;

unsigned int topPositionRefreshedFlag;
int shoulderPos[3];
 bool bRockEnable = false;
unsigned short  rocktimecount; 
__no_init unsigned char WorkStep; //摇摆
//__no_init unsigned char cloud_dat[8192];

//__no_init unsigned char cloud_dat2[8192];



//与通信相关的变量
//两种类型的数据包，命令包和状态包，命令包需要应答，状态包无需应答
__no_init unsigned char OutBuffer[MAX_OUTBUFFER_COUNT] ;
__no_init unsigned char OutBufferBlueTooth[MAX_OUTBUFFER_COUNT] ;
__no_init unsigned char InBuffer[MAX_INBUFFER_COUNT] ;
__no_init unsigned char nInBufferCount ;
__no_init unsigned char nOutBufferCount ;
//__no_init unsigned char nOutBufferBlueToothCount;
__no_init unsigned char nSendCount ;
//__no_init unsigned char nCommandID ;
__no_init unsigned char nSendPacketID ;

__no_init unsigned char OutLegBuffer[MAX_OUTBUFFER_COUNT] ;
__no_init unsigned char InLegBuffer[MAX_INBUFFER_COUNT] ;
__no_init unsigned char nOutLegBufferCount;
bool bMasterSendLegPacket;
__no_init unsigned char _3D_Mode_Step;
unsigned int sysTimer;

bool bRollerEnable,bLegKneadEnable;   //全局变量，滚轮开关

__no_init unsigned char nRollerPWM;

bool bFlexEnable;

unsigned char nFlexMode;
#define FLEX_AUTO   1
#define FLEX_MANUAL 0

unsigned char nFlexDirection;
#define FLEX_IN    0
#define FLEX_OUT   1
unsigned char nLegAirBagMode;
#define LEG_AIRBAG_MODE1  0x14
#define LEG_AIRBAG_MODE2  0x15
#define LEG_AIRBAG_MODE3  0x16

unsigned char nLegAirBagStrength;
#define LEG_AIRBAG_OFF     0x17
#define LEG_AIRBAG_WEAK    0x18
#define LEG_AIRBAG_MIDDLE  0x19
#define LEG_AIRBAG_STRONG  0x1a

bool bLegModulePower;
unsigned char  pressstep;
bool bLegHeat;
/******************************************************************/
//9种手动程序
//#define nMaunalSubMode_KNEAD		0
//#define nMaunalSubMode_KNOCK		3
//#define nMaunalSubMode_WAVELET		2////揉捏同步
//#define nMaunalSubMode_SOFT_KNOCK	1////叩击
//#define nMaunalSubMode_PRESS		4
//#define nMaunalSubMode_MUSIC		5

//#define nMaunalSubMode_3DMODE_1         6
//#define nMaunalSubMode_3DMODE_2         7
//#define nMaunalSubMode_3DMODE_3         8


__no_init unsigned char nKeyKneadWidth ;//通过手控器按键获取的揉捏宽中窄位置
__no_init unsigned char nKeyKneadKnockSpeed ;//通过手控器按键获取的揉敲速度值{1，2,3.。6}
__no_init unsigned char nMaunalSubMode;    //通过手控器按键获取的手动程序. nMaunalSubMode_KNEAD,nMaunalSubMode_KNOCK,nMaunalSubMode_WAVELET
/******************************************************************/
unsigned int password;
/******************************************************************/
unsigned int nPowerMotorHighTime;
unsigned int nPowerMotorLowTime;
unsigned int nPowerValveHighTime;
unsigned int nPowerValveLowTime;
unsigned int nPowerVCCHighTime;
unsigned int nPowerVCCLowTime;
unsigned char nVoicekey;
// nAxisStrength:总强度  
//unsigned char  nAxisStrength/*,nAxisStrengthBase,nAxisAuto,nAxisMode*/; 
/*
  nKeyAxisStrength 用户设定值  数值范围0-4
  nSetAxisStrength 程序设定值  数值范围0-4
  nFinalAxisStrength 实际值    数值范围0-4
  nFinalAxisStrength 计算方式：
*/
unsigned char  nKeyAxisStrength,nSetAxisStrength,nFinalAxisStrength,nAxisUpdateCounter; ////3D 力度，3D马达的坐标，3D马达坐标越大，挤压力度越强
//unsigned char  nDisplayAxisStrength; //添加肩部显示
unsigned int nWidthOverTime;
unsigned int nPowerOverTime;
unsigned int nWalkOverTime;
unsigned int nBackOverTime;
unsigned int nLegOverTime;
unsigned int nZeroOverTime;
unsigned int nWaveOverTime;
unsigned int nFlexOverTime;

/******************************************************************/
__no_init unsigned short nFinalWalkMotorLocate ;//行走马达绝对坐标
/******************************************************************/
//自动程序下时间计数器
__no_init unsigned char nCurActionStepCounter;       //当前步骤时间计数器(适用于所有行走，揉捏，敲打电机)=机芯
__no_init unsigned char nCurKnockRunStopCounter;   //叩击动作记数器
__no_init unsigned char nCur3D_MotorStopCounter;//3D电机自动程序中，到达目标位置的停止时间
__no_init unsigned char nCurShoulderAdjustCounter ;
__no_init unsigned char n3DMotorRunCounter;
/******************************************************************/
unsigned char nCurKneadWidth = KNEAD_WIDTH_UNKNOWN ;//KNEAD_WIDTH_MIN,当前揉捏头的位置
__no_init unsigned char nCurKneadMotorCycles ;//揉捏圈数

//nFinalKneadMotorState = STATE_RUN_CLOCK ;//揉捏头状态，顺时针，停止，反时钟等
unsigned char nCurKneadMotorState,nPrevKneadMotorState,nFinalKneadMotorState ;

/******************************************************************/
__no_init unsigned char nWalkMotorControlParam1;
__no_init unsigned short nWalkMotorControlParam2 ;
__no_init unsigned char nKneadMotorControlParam1,nKneadMotorControlParam2 ;
__no_init unsigned char n3D_MotorControlState,n3D_MotorControlPosition,n3D_MotorControlSpeed,n3D_MotorControlStopTime;
__no_init unsigned char nKnockMotorControlParam1,nKnockMotorControlParam2,nKnockMotorControlParam3 ;

unsigned char engineeringTime_10msFlag = 0; //工程模式使用
unsigned char nStretchFlextimer; //define flex脱离脚心的时间    wgh20170215
//unsigned short adcAudio_L_Base,adcAudio_R_Base;



//=======================================================
//摇摆马达PWM值
 unsigned char const VIB_STRENGTH[] = 
{
  //0,3,6,10,14,18
  WAVE_LEVEL0,WAVE_LEVEL1,WAVE_LEVEL2,WAVE_LEVEL3
} ;
unsigned char by_waver_time;
//=============================================================

unsigned int  w_ZeroPosition;

unsigned short w_current,w_current2;
//========================================================
unsigned int w_leg_time;

/******************************************************************/
__no_init WALK_KNEAD_KNOCK_MOTOR_STRUCT_AUTO AutoDirector;//AutoDirector为定义结构体变量

__no_init WALK_KNEAD_KNOCK_MOTOR_STRUCT_MANUAL ManualDirector[4] ;//手动程序  ManualDirector[4]为定义结构体数组
//下面定义7个气囊程序
__no_init st_AirBag st_AirBagLegFoot,st_AirBagSeat, st_AirBagArmNeck,st_AirBagAuto;//,st_AirBag_Neck;//20170205 WGH 
__no_init st_AirBag st_AirBagLegFoot_Seat,st_AirBagLegFoot_Arm,st_AirBagArm_Seat,st_AirBagLegFoot_Arm_Seat,st_AirBagAuto_Upbody;
/******************************************************************/
#define AUTO_FUNCTION_0_STEPS	sizeof(AutoFunction0)/sizeof(struct Walk_Knead_Knock_Motor_Struct_Auto)////定义第一种自动程序运行的步数，主要是AutoFunction0数组的长度
#define AUTO_FUNCTION_1_STEPS	sizeof(AutoFunction1)/sizeof(struct Walk_Knead_Knock_Motor_Struct_Auto)
#define AUTO_FUNCTION_2_STEPS	sizeof(AutoFunction2)/sizeof(struct Walk_Knead_Knock_Motor_Struct_Auto)
#define AUTO_FUNCTION_3_STEPS	sizeof(AutoFunction3)/sizeof(struct Walk_Knead_Knock_Motor_Struct_Auto)
#define AUTO_FUNCTION_4_STEPS	sizeof(AutoFunction4)/sizeof(struct Walk_Knead_Knock_Motor_Struct_Auto)
#define AUTO_FUNCTION_5_STEPS	sizeof(AutoFunction5)/sizeof(struct Walk_Knead_Knock_Motor_Struct_Auto)
                                      
#define AUTO_FUNCTION_6_STEPS	sizeof(_3DFunction0)/sizeof(struct Walk_Knead_Knock_Motor_Struct_Auto)
#define AUTO_FUNCTION_7_STEPS	sizeof(_3DFunction1)/sizeof(struct Walk_Knead_Knock_Motor_Struct_Auto)
#define AUTO_FUNCTION_8_STEPS	sizeof(_3DFunction2)/sizeof(struct Walk_Knead_Knock_Motor_Struct_Auto)
#define AUTO_FUNCTION_9_STEPS	sizeof(DemoFunction)/sizeof(struct Walk_Knead_Knock_Motor_Struct_Auto)
#define AUTO_FUNCTION_10_STEPS	sizeof(Demo6MinFunction)/sizeof(struct Walk_Knead_Knock_Motor_Struct_Auto)


//10种自动程序步骤总数
const unsigned int BACK_AUTO_STEPS[] =
{
	AUTO_FUNCTION_0_STEPS,
	AUTO_FUNCTION_1_STEPS,
	AUTO_FUNCTION_2_STEPS,
	AUTO_FUNCTION_3_STEPS,
	AUTO_FUNCTION_4_STEPS,
	AUTO_FUNCTION_5_STEPS,
        
        AUTO_FUNCTION_6_STEPS,
        AUTO_FUNCTION_7_STEPS,
        AUTO_FUNCTION_8_STEPS,
        AUTO_FUNCTION_9_STEPS,
        
        AUTO_FUNCTION_10_STEPS,
        
        
        
} ;
//自动程序循环运行开始的步骤
const unsigned char BACK_AUTO_START_STEP[] =
{
	0,
	0,
	0,
	0,
	0,
	0,
        0,
        0,
        0,
        0,
} ;
/******************************************************************/
__no_init unsigned char nStretchStep;
/******************************************************************/
__no_init unsigned short nPartialTop,nPartialBottom ;
/******************************************************************/
bool bAxisUpdate;  //3D电机坐标更新标志位，当为TRUE时3D运行，当为0时3D停止运行

__no_init unsigned char n3Dpointturn;
bool bAxisUpdate_Manual_Waitting_Timef;
bool bAxisUpdate_Program_Waitting_Timef;

unsigned char nAxisUpdate_Manual_Waitting_Time_cnt;
unsigned char nAxisUpdate_Program_Waitting_Time_cnt;
bool bEnableStretchDemo = false;//20170208
bool bEnableStretchDemoRun = false;//20170208

//
bool bHaveMan;


/******************************************************************/
void BodyDataRefresh(void);
void Main_Initial_Data(void);

void Main_Initial_IO(void)
{
    __disable_irq();
    System_Initial_IO();
    // __enable_irq(); //test
    // while(1);
    Power_Initial_IO();
    IndicateLED_Initial_IO();
    KneadMotor_Initial_IO();//揉捏电机   test ok
    UART0_Initial_IO();//手控器UART
    Valve_Initial_IO();//气阀.和脚底滚轮控制
    Axis_Initial_IO();//ZDirectionMotor 控制



    Waver_FlexMotor_Initial_IO();//坐垫摇摆电机 test ok
//    ZeroMotor_Initial_IO();//零重力马达
    
    SlideMotor_Initial_IO();
    
    
    LegMotor_Initial_IO();//小腿   test ok
    BackMotor_Initial_IO();//靠背 TEST OK
    WalkMotor_Initial_IO();//行走 test ok
    Input_Initial_IO();//
    KnockMotor_Initial_IO();//敲击     test ok
    MP3Control1_Initial_IO();
    WaistHeat_Initial_IO();//腰部加热
    LED_RGB_Initial_IO();
    UartLeg_Initial_IO();
    BlueToothUart_Initial_IO();//WIFI初始化程序
    DMA_Ctrl_Init();
    ADC_Data_Init();
    //LEUart_Initial_IO();
    LEUART0_Initial_IO();//读取3D机芯3D脉冲和状态信号
  //  UART1_Initial_IO();//语音程序初始化，8600S不需要
    __enable_irq();
  
}

#define		ADSTRONG_ON  8  //有声音
#define		ADSTRONG1  10  //声音慢慢增大
#define		ADSTRONG2  30
#define		ADSTRONG3  50
#define		ADSTRONG4  70
#define		ADSTRONG5  100
#define		ADSTRONG6  150
unsigned int nAvrADResult0 ;
unsigned int nMusicKnockPWM ;//用于音乐互动
void MusicSampling(void)
{
    unsigned int adcAudio_L,adcAudio_R;
    
    if(ADC_Get_Updata() < 0)
    {
     return; 
    }
    
    adcAudio_L = *(pADC + ADC_AUDIO_L);
    adcAudio_R = *(pADC + ADC_AUDIO_R);
    
    if(adcAudio_L >= adcAudio_R)
    {
        nAvrADResult0 = adcAudio_L - adcAudio_R  ;
    }
    else
    {
        nAvrADResult0 = adcAudio_R - adcAudio_L  ;
    } 
    
    if(nAvrADResult0 > ADSTRONG1)
    {
   //   VoiceUart_SetMusicStatus(1);  //有音乐   delete by taoqngsong
    }
    else
    {
  //    VoiceUart_SetMusicStatus(0); //无音乐   //de by taoqingseong
    } 
}


//敲打音乐互动
unsigned int AD_KNOCK_PWM(unsigned int nADValue)
{
    unsigned int nRetPWM ;
    if(nADValue < ADSTRONG1)  nRetPWM = KNOCK_SPEED0_PWM;
    else if(nADValue < ADSTRONG2)  nRetPWM = KNOCK_SPEED1_PWM;
    else if(nADValue < ADSTRONG3)  nRetPWM = KNOCK_SPEED2_PWM;
    else if(nADValue < ADSTRONG4)  nRetPWM = KNOCK_SPEED3_PWM;
    else if(nADValue < ADSTRONG5)  nRetPWM = KNOCK_SPEED4_PWM;
    else if(nADValue < ADSTRONG6)  nRetPWM = KNOCK_SPEED5_PWM;
    else nRetPWM = KNOCK_SPEED6_PWM;
    return nRetPWM ;
}  
/*
n3D_position: 1-5 5个力度位置
n3D_MotorSpeed：运行速度
n3D_MotorStopTime：运行到位置后的停留时间
*/
/*
void _3DMotorControl(unsigned char state,unsigned char position,unsigned char speed,unsigned char stopTime)
{
  if(Problem_GetWalkSwitchFault())
  {
     b3D_MotorInProcess = false; 
     AxisMotor_Control(STATE_AXIS_IDLE,0,_3D_SPEED_10); 
     return;
  }
  switch(state)
  {
  default: 
   case  _3D_MANUAL:  //手动控制 
        b3D_MotorInProcess = FALSE ; 
        if(Input_GetWalkMotorPosition() > _3D_MOTOR_WALK_MAX_POSITION)
        {//行走电机走到最高点 
          AxisMotor_Control(STATE_RUN_AXIS_FORWARD,0,_3D_SPEED_7);
          bAxisUpdate = true;  //3D电机坐标更新标志位，当为TRUE时3D运行，当为0时3D停止运行
          nDisplayAxisStrength = 4;// wgh 20140421
          break;      
        }
      
        if(bAxisUpdate)
        {
          nFinalAxisStrength = nKeyAxisStrength;  //3D 力度，3D马达的坐标，3D马达坐标越大，挤压力度越强
          nDisplayAxisStrength = nFinalAxisStrength;
          if(AxisMotor_Control(STATE_RUN_AXIS_POSITION,nFinalAxisStrength,_3D_SPEED_10) == TRUE)
          {
            bAxisUpdate = FALSE;  //3D电机走到目标位置
          }
        }
        else
        {
         AxisMotor_Control(STATE_AXIS_IDLE,0,_3D_SPEED_10); 
        }
        break;
        
   case  _3D_PROGRAM:
        if(Input_GetWalkMotorPosition() > _3D_MOTOR_WALK_MAX_POSITION)
        {
          AxisMotor_Control(STATE_RUN_AXIS_FORWARD,0,_3D_SPEED_7);
          b3D_MotorInProcess = false;
          bAxisUpdate = true; // //3D电机坐标更新标志位，当为TRUE时3D运行，当为0时3D停止运行
          nDisplayAxisStrength = 4;// wgh 20140421
          break;      
        }
        if(bAxisUpdate)
        {
           unsigned char strength = nSetAxisStrength;
           switch(nKeyAxisStrength)
           {
            
            case 0:  if(strength > 0) strength--;
            case 1:  if(strength > 0) strength--;
            case 2:  nFinalAxisStrength = strength; 
                     nDisplayAxisStrength = nFinalAxisStrength;//20150421 wgh 添加肩部显示
                     break;
            case 4:  if(strength < 4) strength++;
            case 3:  if(strength < 4) strength++;
                     nFinalAxisStrength = strength; 
                     nDisplayAxisStrength = nFinalAxisStrength;//20150421 wgh 添加肩部显示
                     break;  
           }
         }
         
         if(!b3D_MotorInProcess && !bAxisUpdate)
         {
           AxisMotor_Control(STATE_AXIS_IDLE,0,_3D_SPEED_10); 
           break;
         }
         
        if(AxisMotor_Control(STATE_RUN_AXIS_POSITION,nFinalAxisStrength,speed))
        {
          bAxisUpdate = false;
          if(nCur3D_MotorStopCounter >= stopTime)//3D自动程序到达目标位置的停止时间
          {
            b3D_MotorInProcess = FALSE ; 
          }
        }
        else//3D电机未到目标位置
        {
          nCur3D_MotorStopCounter = 0;
        }
        break;
        
  case _3D_PARK:
    b3D_MotorInProcess = false;
    AxisMotor_Control(STATE_AXIS_IDLE,0,_3D_SPEED_10); 
    break;
  }
  
  
  
  
}*/

void AxisUpdate_Waitting_100msInt(void)//3D启动10秒，不到位，认为到位
{
  
  if(bAxisUpdate_Manual_Waitting_Timef == TRUE)
  {
    nAxisUpdate_Manual_Waitting_Time_cnt++;
    if(nAxisUpdate_Manual_Waitting_Time_cnt >=100)
    {
      bAxisUpdate = FALSE;b3D_MotorInProcess = false;
    }
  }
  else
  {
    nAxisUpdate_Manual_Waitting_Time_cnt =0;
  }
  
  if(bAxisUpdate_Program_Waitting_Timef == TRUE)
  {
    nAxisUpdate_Program_Waitting_Time_cnt++;
    if(nAxisUpdate_Program_Waitting_Time_cnt >=100)
    {
      bAxisUpdate = FALSE;b3D_MotorInProcess = false;
    }
  }
  else
  {
    nAxisUpdate_Program_Waitting_Time_cnt =0;
  }
 
}

/*
//                                            _3D_PROGRAM,AXIS_WEAKEST,_3D_SPEED_7,10},
void _3DMotorControl(unsigned char state,unsigned char position,unsigned char speed,unsigned char stopTime)
{
  if(Problem_GetWalkSwitchFault())
  {
     b3D_MotorInProcess = false; 
     AxisMotor_Control(STATE_AXIS_IDLE,0,_3D_SPEED_10); 
     return;
  }
  switch(state)
  {
  default: 
   case  _3D_MANUAL:  //手动控制 
     //3D启动10秒，不到位，认为到位
     bAxisUpdate_Program_Waitting_Timef = FALSE;
     nAxisUpdate_Program_Waitting_Time_cnt = 0;
     
     b3D_MotorInProcess = FALSE ; 
   if(Input_GetWalkMotorPosition() > _3D_MOTOR_WALK_MAX_POSITION)
    {
      AxisMotor_Control(STATE_RUN_AXIS_FORWARD,0,_3D_SPEED_7);
      bAxisUpdate = true; 
      nDisplayAxisStrength = 4;// wgh 20140421
      break;      
    }
  
    if(bAxisUpdate)
    {//      //nKeyAxisStrength=3D力度手控器上的快捷键
      nFinalAxisStrength = nKeyAxisStrength; //nKeyAxisStrength=3D力度手控器上的快捷键
      nDisplayAxisStrength = nFinalAxisStrength;//显示3D力度大小
      if(nKeyAxisStrength ==0)
      {
        if(AxisMotor_Control(STATE_RUN_AXIS_POSITION,nFinalAxisStrength,_3D_SPEED_4) == TRUE)
        {
          bAxisUpdate = FALSE;  
        }
      }
      else
      {
        if(AxisMotor_Control(STATE_RUN_AXIS_POSITION,nFinalAxisStrength,_3D_SPEED_8) == TRUE)
        {
          bAxisUpdate = FALSE;  
        }
      }
      //3D启动10秒，不到位，认为到位
      //bAxisUpdate_Manual_Waitting_Timef = TRUE;
    }
   else
   {
     AxisMotor_Control(STATE_AXIS_IDLE,0,_3D_SPEED_10); 
     //3D启动10秒，不到位，认为到位
        nAxisUpdate_Manual_Waitting_Time_cnt = 0;
        nAxisUpdate_Program_Waitting_Time_cnt = 0;
        bAxisUpdate_Manual_Waitting_Timef = FALSE;
        bAxisUpdate_Program_Waitting_Timef = FALSE;
   }
     break;
   case  _3D_PROGRAM:
     bAxisUpdate_Manual_Waitting_Timef = FALSE;
     nAxisUpdate_Manual_Waitting_Time_cnt = 0;
    if(Input_GetWalkMotorPosition() > _3D_MOTOR_WALK_MAX_POSITION)
    {
      AxisMotor_Control(STATE_RUN_AXIS_FORWARD,0,_3D_SPEED_7);
      b3D_MotorInProcess = false;
      bAxisUpdate = true; 
      nDisplayAxisStrength = 4;// wgh 20140421
      break;      
    }
     if(bAxisUpdate)
     {
       unsigned char strength = nSetAxisStrength;
       switch(nKeyAxisStrength)
       {
        
        case 0:  if(strength > 0) strength--;
        case 1:  if(strength > 0) strength--;
        case 2:  nFinalAxisStrength = strength; 
                 nDisplayAxisStrength = nFinalAxisStrength;//20150421 wgh 添加肩部显示
                 break;
        case 4:  if(strength < 4) strength++;
        case 3:  if(strength < 4) strength++;
                 nFinalAxisStrength = strength; 
                 nDisplayAxisStrength = nFinalAxisStrength;//20150421 wgh 添加肩部显示
                 break;  
       }
       bAxisUpdate_Program_Waitting_Timef = TRUE;
     }
     
     if(!b3D_MotorInProcess && !bAxisUpdate)
     {
       AxisMotor_Control(STATE_AXIS_IDLE,0,_3D_SPEED_10); 
        nAxisUpdate_Manual_Waitting_Time_cnt = 0;
        nAxisUpdate_Program_Waitting_Time_cnt = 0;
        bAxisUpdate_Manual_Waitting_Timef = FALSE;
        bAxisUpdate_Program_Waitting_Timef = FALSE;
       break;
     }
     
     if(AxisMotor_Control(STATE_RUN_AXIS_POSITION,nFinalAxisStrength,speed))
    {
      bAxisUpdate = false;
      if(nCur3D_MotorStopCounter >= stopTime)
      {
        b3D_MotorInProcess = FALSE ; 
      }
    }
    else
    {
      nCur3D_MotorStopCounter = 0;
    }
     break;
  case _3D_PARK:
    nAxisUpdate_Manual_Waitting_Time_cnt = 0;
    nAxisUpdate_Program_Waitting_Time_cnt = 0;
    bAxisUpdate_Manual_Waitting_Timef = FALSE;
    bAxisUpdate_Program_Waitting_Timef = FALSE;
    b3D_MotorInProcess = false;
    AxisMotor_Control(STATE_AXIS_IDLE,0,_3D_SPEED_10); 
    break;
  }
}*/


void _3DMotorControl(unsigned char state,unsigned char position,unsigned char speed,unsigned char stopTime)
{
  if(/*Problem_Get3DFault() || */Problem_GetWalkSwitchFault())
  {
     b3D_MotorInProcess = false; 
     AxisMotor_Control(STATE_AXIS_IDLE,0,_3D_SPEED_8); 
     return;
  }
  switch(state)
  {
    
    //机芯处于MANUAL 模式时，3D 前后工作
    
  case _3D_MANUAL_AUTO_VECTOR:
    //++++++++++++++++++++++++++
    if(Input_GetWalkMotorPosition() > _3D_MOTOR_WALK_MAX_POSITION)
    {
      AxisMotor_Control(STATE_RUN_AXIS_FORWARD,0,_3D_SPEED_7);
      bAxisUpdate = true; 
      //if(Input_Get_KeyAxisStrengthManualEnable()==false) 
      //{
        //nDisplayAxisStrength = 4;//
      //}
      b3D_MotorInProcess = FALSE ;//20170228 WGH  在TAPOFF 中，拉伸3次后机芯停在上边不下来，就加这一句解决问题
      break;      
    }
    //--------------------------
    if(b3D_MotorInProcess)    
    {     
      if(AxisMotor_Control(STATE_RUN_AXIS_REAL_VECTOR,position,speed))
      {
        bAxisUpdate = false;
        if(nCur3D_MotorStopCounter >= stopTime)
        {
          b3D_MotorInProcess = FALSE ; 
        }
      }
      else
      {
        nCur3D_MotorStopCounter = 0;
      }
    }   
    if(!b3D_MotorInProcess)// &&(!bAxisUpdate)   )
    {
      AxisMotor_Control(STATE_AXIS_IDLE,0,_3D_SPEED_8); 
      nAxisUpdate_Manual_Waitting_Time_cnt = 0;
      nAxisUpdate_Program_Waitting_Time_cnt = 0;
      bAxisUpdate_Manual_Waitting_Timef = FALSE;
      bAxisUpdate_Program_Waitting_Timef = FALSE;
      break;
    }  
    
    
    break; 
    
    
    
    
  default: 
   case  _3D_MANUAL:  //手动控制 
     //3D启动10秒，不到位，认为到位
     bAxisUpdate_Program_Waitting_Timef = FALSE;
     nAxisUpdate_Program_Waitting_Time_cnt = 0;
     
     b3D_MotorInProcess = FALSE ; 
   if(Input_GetWalkMotorPosition() > _3D_MOTOR_WALK_MAX_POSITION)
    {
      AxisMotor_Control(STATE_RUN_AXIS_FORWARD,0,_3D_SPEED_7);
      bAxisUpdate = true; 
      //nDisplayAxisStrength = 4;// wgh 20140421
      break;      
    }
  
    if(bAxisUpdate) //nKeyAxisStrength=3D力度手控器上的快捷键
    {
      nFinalAxisStrength = nKeyAxisStrength;  //nDisplayAxisStrength = nFinalAxisStrength;
      if(nKeyAxisStrength ==0)
      {
        if(AxisMotor_Control(STATE_RUN_AXIS_VECTOR,nFinalAxisStrength,_3D_SPEED_4) == TRUE)
        {
          bAxisUpdate = FALSE;  
        }
      }
      else
      {
        if(AxisMotor_Control(STATE_RUN_AXIS_VECTOR,nFinalAxisStrength,_3D_SPEED_8) == TRUE)
        {
          bAxisUpdate = FALSE;  
        }
      }
      //3D启动10秒，不到位，认为到位
      //bAxisUpdate_Manual_Waitting_Timef = TRUE;
    }
   else
   {
     AxisMotor_Control(STATE_AXIS_IDLE,0,_3D_SPEED_8); 
     //3D启动10秒，不到位，认为到位
        nAxisUpdate_Manual_Waitting_Time_cnt = 0;
        nAxisUpdate_Program_Waitting_Time_cnt = 0;
        bAxisUpdate_Manual_Waitting_Timef = FALSE;
        bAxisUpdate_Program_Waitting_Timef = FALSE;
   }
     break;
   case  _3D_PROGRAM:
     bAxisUpdate_Manual_Waitting_Timef = FALSE;
     nAxisUpdate_Manual_Waitting_Time_cnt = 0;
    if(Input_GetWalkMotorPosition() > _3D_MOTOR_WALK_MAX_POSITION)
    {
      AxisMotor_Control(STATE_RUN_AXIS_FORWARD,0,_3D_SPEED_7);
      b3D_MotorInProcess = false;
      bAxisUpdate = true; 
      //nDisplayAxisStrength = 4;// wgh 20140421
      break;      
    }
     if(bAxisUpdate)
     {
       unsigned char strength = nSetAxisStrength;
       switch(nKeyAxisStrength)
       {
        
        case 0:  if(strength > 0) strength--;
        case 1:  if(strength > 0) strength--;
        case 2:  nFinalAxisStrength = strength; 
                 //nDisplayAxisStrength = nFinalAxisStrength;//20150421 wgh 添加肩部显示
                 break;
        case 4:  if(strength < 4) strength++;
        case 3:  if(strength < 4) strength++;
                 nFinalAxisStrength = strength; 
                 //nDisplayAxisStrength = nFinalAxisStrength;//20150421 wgh 添加肩部显示
                 break;  
       }
       bAxisUpdate_Program_Waitting_Timef = TRUE;
     }
     
     if(!b3D_MotorInProcess && !bAxisUpdate)
     {
       AxisMotor_Control(STATE_AXIS_IDLE,0,_3D_SPEED_8); 
        nAxisUpdate_Manual_Waitting_Time_cnt = 0;
        nAxisUpdate_Program_Waitting_Time_cnt = 0;
        bAxisUpdate_Manual_Waitting_Timef = FALSE;
        bAxisUpdate_Program_Waitting_Timef = FALSE;
       break;
     }
     
     if(AxisMotor_Control(STATE_RUN_AXIS_VECTOR,nFinalAxisStrength,speed))
    {
      bAxisUpdate = false;
      if(nCur3D_MotorStopCounter >= stopTime)
      {
        b3D_MotorInProcess = FALSE ; 
      }
    }
    else
    {
      nCur3D_MotorStopCounter = 0;
    }
     break;
  case _3D_PARK:
    nAxisUpdate_Manual_Waitting_Time_cnt = 0;
    nAxisUpdate_Program_Waitting_Time_cnt = 0;
    bAxisUpdate_Manual_Waitting_Timef = FALSE;
    bAxisUpdate_Program_Waitting_Timef = FALSE;
    b3D_MotorInProcess = false;
    AxisMotor_Control(STATE_AXIS_IDLE,0,_3D_SPEED_8); 
    break;
  }
}







/*
//捶击电机控制
#define KNOCK_STOP		0 //停止
#define KNOCK_RUN_WIDTH		1 //宽中窄定位完成后启动
#define KNOCK_RUN		2 //无需宽中窄定位，无条件立即启动
#define KNOCK_RUN_STOP		3 //宽中窄定位完成后启动短时间后马上停止
#define KNOCK_RUN_MUSIC		4 //音乐互动模式（与宽中窄定位无关）
*/

//捶击电机控制
void KnockMotorControl(unsigned char nKnockMotorState,unsigned char nKnockingMotorRunTime,unsigned char nKnockingMotorStopTime)
{
  static bool  bKnockMotorPowerFlag;
  
  if(nBackMainRunMode == BACK_MAIN_MODE_3D)
  {
     if(Input_GetWalkMotorPosition() > _3D_MOTOR_WALK_MAX_POSITION)
      {
        KnockMotor_Set_Pwm_Data(KNOCK_SPEED0_PWM);
        KnockMotor_Break();
        bKnockMotorInProcess = FALSE ;
        bKnockMotorPowerFlag = FALSE ;
        nCurKnockRunStopCounter = 0 ;//叩击动作记数器
        return;      
      }
  }
    //static int step = 0;
    //敲打电机音乐互动（高频）
    if((nBackMainRunMode == BACK_MAIN_MODE_MANUAL) &&
       (nBackSubRunMode == BACK_SUB_MODE_MUSIC) &&
       (nKnockMotorState == KNOCK_RUN_MUSIC))
    {
        nMusicKnockPWM = AD_KNOCK_PWM(nAvrADResult0) ;
        KnockMotor_Set_Pwm_Data(nMusicKnockPWM);
        bKnockMotorInProcess = FALSE ;
    }
    else
    {
        if(bKnockMotorInProcess == TRUE)
        {
            switch(nKnockMotorState)
            {
            default:
                bKnockMotorInProcess = FALSE ;
                while(1)
                {
                    //WDOG_Feed();
                }
                break;
            case KNOCK_STOP:
                bKnockMotorPowerFlag = FALSE ;
                if(nCurActionStepCounter >= nKnockingMotorRunTime)
                {
                    bKnockMotorInProcess = FALSE ;
                }
                break ;
            case KNOCK_RUN_WIDTH://定位完成后进行
                if(bKneadMotorInProcess == TRUE)
                {
                    bKnockMotorPowerFlag = FALSE ;
                    nCurActionStepCounter = 0 ;
                }
                else
                {
                    bKnockMotorPowerFlag = TRUE ;
                    if(nCurActionStepCounter >= nKnockingMotorRunTime)
                    {
                        bKnockMotorInProcess = FALSE ;
                    }
                }
                break ;
            case KNOCK_RUN:
                bKnockMotorPowerFlag = TRUE ;
                if(nCurActionStepCounter >= nKnockingMotorRunTime)
                {
                     bKnockMotorInProcess = FALSE ;
                }
                break ;
            case KNOCK_RUN_STOP:  //叩击
                if(bKneadMotorInProcess == TRUE)
                {
                    bKnockMotorPowerFlag = FALSE ;
                    nCurKnockRunStopCounter = 0 ;//叩击动作记数器
                }
                else
                {
                    if(nCurKnockRunStopCounter < nKnockingMotorRunTime)//nCurKnockRunStopCounter:单位:2ms; nKnockingMotorRunTime:单位:100ms;
                    {
                        bKnockMotorPowerFlag = TRUE ;
                    }
                    if((nCurKnockRunStopCounter >= nKnockingMotorRunTime) && (nCurKnockRunStopCounter < (nKnockingMotorRunTime + nKnockingMotorStopTime)))
                    {
                        bKnockMotorPowerFlag = FALSE ;
                        //行走电机完成动作时，该动作也结束
                        

                        
                    }
                    if(nCurKnockRunStopCounter >= (nKnockingMotorRunTime + nKnockingMotorStopTime))
                    {
                        nCurKnockRunStopCounter = 0 ;
                        if(bWalkMotorInProcess == FALSE)
                        {
                            bKnockMotorInProcess = FALSE ;
                        }                        
                        
                    }
                }
                break ;
            case KNOCK_RUN_MUSIC:
                break ;
            }
        }
        if(bKnockMotorPowerFlag == TRUE)
        {
            switch(nCurKneadKnockSpeed)
            {
            case 1:
                KnockMotor_Set_Pwm_Data(KNOCK_SPEED1_PWM);
                break ;
            case 2:
                KnockMotor_Set_Pwm_Data(KNOCK_SPEED2_PWM);
                break ;
            case 3:
                KnockMotor_Set_Pwm_Data(KNOCK_SPEED3_PWM);
                break ;
            case 4:
                KnockMotor_Set_Pwm_Data(KNOCK_SPEED4_PWM);
                break ;
            case 5:
                KnockMotor_Set_Pwm_Data(KNOCK_SPEED5_PWM);
                break ;
            case 6:
                KnockMotor_Set_Pwm_Data(KNOCK_SPEED6_PWM);
                break ;
            default:
                KnockMotor_Set_Pwm_Data(KNOCK_SPEED1_PWM);
                break ;
            }
            KnockMotor_ClockRun();
        }
        else
        {  
            KnockMotor_Set_Pwm_Data(KNOCK_SPEED0_PWM);
            KnockMotor_Break();
        }
    }
}


/*
void KnockMotorControl(unsigned char nKnockMotorState,unsigned char nKnockingMotorRunTime,unsigned char nKnockingMotorStopTime)
{
  static bool  bKnockMotorPowerFlag;
  
  if(nBackMainRunMode == BACK_MAIN_MODE_3D)
  {
     if(Input_GetWalkMotorPosition() > _3D_MOTOR_WALK_MAX_POSITION)
      {
        KnockMotor_Set_Pwm_Data(KNOCK_SPEED0_PWM);
        KnockMotor_Break();
        bKnockMotorInProcess = FALSE ;
        bKnockMotorPowerFlag = FALSE ;
        nCurKnockRunStopCounter = 0 ;//叩击动作记数器
        return;      
      }
  } 
  
  
  
    //static int step = 0;
    //敲打电机音乐互动（高频）
    if((nBackMainRunMode == BACK_MAIN_MODE_MANUAL) &&
       (nBackSubRunMode == BACK_SUB_MODE_MUSIC) &&
       (nKnockMotorState == KNOCK_RUN_MUSIC))
    {
        nMusicKnockPWM = AD_KNOCK_PWM(nAvrADResult0) ;
        KnockMotor_Set_Pwm_Data(nMusicKnockPWM);
        bKnockMotorInProcess = FALSE ;
    }
    else
    {
        if(bKnockMotorInProcess == TRUE)
        {
            switch(nKnockMotorState)
            {
            default:
                bKnockMotorInProcess = FALSE ;
                while(1)
                {
                    //WDOG_Feed();
                }
                break;
            case KNOCK_STOP://行走和敲击马达共有一个时间计数器，当敲击计数开始时， 行走马达不可以计数，必须停止在当前位置
                bKnockMotorPowerFlag = FALSE ;
                if(nCurActionStepCounter >= nKnockingMotorRunTime)////当前动作时间计数（行走或敲击时间计数）
                {
                    bKnockMotorInProcess = FALSE ;
                }
                break ;
            case KNOCK_RUN_WIDTH://定位完成后进行,宽中窄定位完成后启动
                if(bKneadMotorInProcess == TRUE)
                {
                    bKnockMotorPowerFlag = FALSE ;
                    nCurActionStepCounter = 0 ;
                }
                else
                {
                    bKnockMotorPowerFlag = TRUE ;
                    if(nCurActionStepCounter >= nKnockingMotorRunTime)
                    {
                        bKnockMotorInProcess = FALSE ;
                    }
                }
                break ;
            case KNOCK_RUN:
                bKnockMotorPowerFlag = TRUE ;
                if(nCurActionStepCounter >= nKnockingMotorRunTime)//100
                {
                     bKnockMotorInProcess = FALSE ;
                }
                break ;
            case KNOCK_RUN_STOP:  //叩击  ，该动作配合行走电机结束才结束
                if(bKneadMotorInProcess == TRUE)
                {
                    bKnockMotorPowerFlag = FALSE ;
                    nCurKnockRunStopCounter = 0 ;//叩击动作记数器
                }
                else
                {
                    if(nCurKnockRunStopCounter < nKnockingMotorRunTime)//nCurKnockRunStopCounter:单位:2ms; nKnockingMotorRunTime:单位:100ms;
                    {
                        bKnockMotorPowerFlag = TRUE ;
                    }
                    if((nCurKnockRunStopCounter >= nKnockingMotorRunTime) && (nCurKnockRunStopCounter < (nKnockingMotorRunTime + nKnockingMotorStopTime)))
                    {
                        bKnockMotorPowerFlag = FALSE ;
                        //行走电机完成动作时，该动作也结束
                        
                        if(bWalkMotorInProcess == FALSE) //
                        {
                            bKnockMotorInProcess = FALSE ;
                        }
                        
                    }
                    if(nCurKnockRunStopCounter >= (nKnockingMotorRunTime + nKnockingMotorStopTime))
                    {
                        nCurKnockRunStopCounter = 0 ;
                    }
                }
                break ;
            case KNOCK_RUN_MUSIC:
                break ;
            }
        }
        if(bKnockMotorPowerFlag == TRUE)
        {
            switch(nCurKneadKnockSpeed)
            {
            case 1:
                KnockMotor_Set_Pwm_Data(KNOCK_SPEED1_PWM);
                break ;
            case 2:
                KnockMotor_Set_Pwm_Data(KNOCK_SPEED2_PWM);
                break ;
            case 3:
                KnockMotor_Set_Pwm_Data(KNOCK_SPEED3_PWM);
                break ;
            case 4:
                KnockMotor_Set_Pwm_Data(KNOCK_SPEED4_PWM);
                break ;
            case 5:
                KnockMotor_Set_Pwm_Data(KNOCK_SPEED5_PWM);
                break ;
            case 6:
                KnockMotor_Set_Pwm_Data(KNOCK_SPEED6_PWM);
                break ;
            default:
                KnockMotor_Set_Pwm_Data(KNOCK_SPEED1_PWM);
                break ;
            }
            KnockMotor_ClockRun();
        }
        else
        {  
            KnockMotor_Set_Pwm_Data(KNOCK_SPEED0_PWM);
            KnockMotor_Break();
        }
    }
}


*/
//==========================================================

//#ifdef TWIST_ON

       /* if(IsPowerZeroBackLeg() == ZERO_BACK_LEG_POWER_ON)       
          {
            WaveMotorStop();
          }*/
    
//bKeySeatEnable,bKeySeatVibrate这两个变量还没有初始化，与手控器接口中应该增加摇摆打开/关闭按键

void Main_WaveMotorStop(void)
{
  if(WaveMotor_IsRun() == FLEX_POWER_ON)
  {
    if((Input_GetWaveMotorPosition() == REACH_WAVER_LIMIT)||(bWaveMotorFail))  
    {
      Waveringly_Set_Pwm_Data(VIB_STRENGTH[0]);
    }
    else
    {
      Waveringly_Set_Pwm_Data(VIB_STRENGTH[1]);
    }
  }
}
//摇摆电机制函数IsPowerZeroBackLeg(void)//ZERO_BACK_LEG_POWER_ONWaveringly_Set_Pwm_Data(0);
void Main_VibrateMotorControl(void)
{
 // if(nChairRunState == CHAIR_STATE_PAUSE) return ;
  if(!bKeySeatVibrate)//if(bKeySeatVibrate) Timer_Counter_Clear(C_TIMER_WAVE_START);
  {
    nKeySeatVibrateStrength = 0;
    Main_WaveMotorStop();
    return;
  }
  if(bKeySeatEnable)
  {//nKeySeatVibrateStrength=2;//nKeySeatVibrateStrength_old
    
    
  
      
       if(  (IsPowerZeroBackLeg() == ZERO_BACK_LEG_POWER_ON) || (Is_THIGH_CHR()==THITH_PUMP_ON))
        {
          
             Waveringly_Set_Pwm_Data(0);
          
        }
        else
        {
   
     Waveringly_Set_Pwm_Data(VIB_STRENGTH[nKeySeatVibrateStrength]);
        }
        

  }
  else
  {
     Main_WaveMotorStop();
  }
}
//#endif

//=========================================================


void main_GetKneadPosition(void)
{            
    static unsigned char nLastKneadPosition = KNEAD_WIDTH_UNKNOWN ;
    unsigned char nNowKneadPosition = Input_GetKneadPosition();
    if(nNowKneadPosition != nLastKneadPosition) 
    {
        nWidthOverTime = 0;
        if(nNowKneadPosition == KNEAD_WIDTH_MIN)
        {
            bHasKneadWidthMinPulse = TRUE ;//揉捏电机处于窄的位置
            bHasKneadWidthMedPulse = FALSE ;
            bHasKneadWidthMaxPulse = FALSE ;
            bDisplayKneadTrackMin = TRUE ;
            bDisplayKneadTrackMed = FALSE ;
            bDisplayKneadTrackMax = FALSE ;
            bDisplayKneadWidthMin = TRUE ;
            bDisplayKneadWidthMed = FALSE ;
            bDisplayKneadWidthMax = FALSE ;
            nLastKneadPosition = KNEAD_WIDTH_MIN ;
        }
        if(nNowKneadPosition == KNEAD_WIDTH_MED)
        {
            bHasKneadWidthMinPulse = FALSE ;
            bHasKneadWidthMedPulse = TRUE ;//揉捏电机处于中的位置
            bHasKneadWidthMaxPulse = FALSE ;
            bDisplayKneadTrackMin = FALSE ;
            bDisplayKneadTrackMed = TRUE ;
            bDisplayKneadTrackMax = FALSE ;
            bDisplayKneadWidthMin = FALSE ;
            bDisplayKneadWidthMed = TRUE ;
            bDisplayKneadWidthMax = FALSE ;
            nLastKneadPosition = KNEAD_WIDTH_MED ;
        }
        if(nNowKneadPosition == KNEAD_WIDTH_MAX)
        {
            bHasKneadWidthMinPulse = FALSE ;
            bHasKneadWidthMedPulse = FALSE ;
            bHasKneadWidthMaxPulse = TRUE ;//揉捏电机处于中的位置
            bDisplayKneadTrackMin = FALSE ;
            bDisplayKneadTrackMed = FALSE ;
            bDisplayKneadTrackMax = TRUE ;
            bDisplayKneadWidthMin = FALSE ;
            bDisplayKneadWidthMed = FALSE ;
            bDisplayKneadWidthMax = TRUE ;
            nLastKneadPosition = KNEAD_WIDTH_MAX ;
        }
    }
}
  unsigned int nstepcnt;
void KneadMotorControl(unsigned char nKneadMotorState,unsigned char nKneadMotorCycles)
{
    unsigned int speed;
    unsigned int step;
  
       if(nBackMainRunMode == BACK_MAIN_MODE_3D)
       {
            if(Input_GetWalkMotorPosition() > _3D_MOTOR_WALK_MAX_POSITION)
            {
              KneadMotor_Control(STATE_KNEAD_IDLE,KNEAD_SPEED0_PWM);
              nFinalKneadMotorState = STATE_IDLE ;
              bKneadMotorInProcess = FALSE ;
              return;      
            }
        }  
    
    
    
    if(bKneadMotorInProcess == TRUE)
    {
        switch(nKneadMotorState)
        {
        default:
        case KNEAD_STOP:
            nFinalKneadMotorState = STATE_IDLE ;
            bKneadMotorInProcess = FALSE ;
            break ;
        case KNEAD_STOP_AT_MIN:////按摩臂停留在窄的位置
            if(nCurKneadWidth == KNEAD_WIDTH_MIN)//电机到达窄的位置
            {
                nFinalKneadMotorState = STATE_IDLE ;
                bKneadMotorInProcess = FALSE ;
            }
            else
            {
                if(bHasKneadWidthMinPulse == TRUE)
                {
                    bHasKneadWidthMinPulse = FALSE ;
                    if(Input_GetKneadMin() == 0)
                    {
                        nCurKneadWidth = KNEAD_WIDTH_MIN ;
                        nFinalKneadMotorState = STATE_IDLE ;
                        bKneadMotorInProcess = FALSE ;
                    }
                }
                else
                {
                    nCurKneadWidth = KNEAD_WIDTH_UNKNOWN ;
                    nFinalKneadMotorState = STATE_RUN_CLOCK ;//揉捏电机顺时钟运行，向窄的方向运行
                    nKneadTurn = 1;
                }
            }
            break ;
        case KNEAD_STOP_AT_MED://按摩臂停留在中的位置
            if(nCurKneadWidth == KNEAD_WIDTH_MED)
            {
                nFinalKneadMotorState = STATE_IDLE ;
                bKneadMotorInProcess = FALSE ;
            }
            else
            {
                if(bHasKneadWidthMedPulse == TRUE)
                {
                    bHasKneadWidthMedPulse = FALSE ;
                    if(Input_GetKneadMid() == 0)
                    {
                        nCurKneadWidth = KNEAD_WIDTH_MED ;
                        nFinalKneadMotorState = STATE_IDLE ;
                        bKneadMotorInProcess = FALSE ;
                    }
                }
                else
                {
                    nCurKneadWidth = KNEAD_WIDTH_UNKNOWN ;
                    nFinalKneadMotorState = STATE_RUN_CLOCK ;//揉捏头状态，顺时针，停止，反时钟等
                    nKneadTurn = 1;
                }
            }
            break ;
        case KNEAD_STOP_AT_MAX://按摩臂停留在宽的位置
            if(nCurKneadWidth == KNEAD_WIDTH_MAX)
            {
                nFinalKneadMotorState = STATE_IDLE ;
                bKneadMotorInProcess = FALSE ;
            }
            else
            {
                if(bHasKneadWidthMaxPulse == TRUE)
                {
                    bHasKneadWidthMaxPulse = FALSE ;
                    if(Input_GetKneadMax() == 0)
                    {
                        nCurKneadWidth = KNEAD_WIDTH_MAX ;
                        nFinalKneadMotorState = STATE_IDLE ;
                        bKneadMotorInProcess = FALSE ;
                    }
                }
                else
                {
                    nCurKneadWidth = KNEAD_WIDTH_UNKNOWN ;
                    nFinalKneadMotorState = STATE_RUN_CLOCK ;nKneadTurn = 1;
                }
            }
            break ;
        case KNEAD_RUN:nKneadTurn = 1;
            nCurKneadWidth = KNEAD_WIDTH_UNKNOWN ;
            nFinalKneadMotorState = STATE_RUN_CLOCK ;
            bKneadMotorInProcess = FALSE ;//在自动程序数组同步其他电机的停止状态，只要其他电机都停止默认该电机也会停止
            break ;
        case KNEAD_ANTIRUN:nKneadTurn = 2;
            nCurKneadWidth = KNEAD_WIDTH_UNKNOWN ;
            nFinalKneadMotorState = STATE_RUN_UNCLOCK ;
            bKneadMotorInProcess = FALSE ;//在自动程序数组同步其他电机的停止状态，只要其他电机都停止默认该电机也会停止
            break ;
        case KNEAD_RUN_STOP:
        case KNEAD_RUN_STOP_AT_MIN:////按摩臂CLOCK方向n圈后停留在窄的位置
            if(bHasKneadWidthMinPulse == TRUE)
            {
                bHasKneadWidthMinPulse = FALSE ;
                nCurKneadMotorCycles++ ;
                if(nCurKneadMotorCycles > nKneadMotorCycles)
                {
                    if(Input_GetKneadMin() == 0)  
                    {
                        nCurKneadWidth = KNEAD_WIDTH_MIN ;
                        nFinalKneadMotorState = STATE_IDLE ;
                        bKneadMotorInProcess = FALSE ;
                    }
                }
            }
            else
            {
                nCurKneadWidth = KNEAD_WIDTH_UNKNOWN ;
                nFinalKneadMotorState = STATE_RUN_CLOCK ;nKneadTurn = 1;
            }
            break ;
        case KNEAD_RUN_STOP_AT_MED://按摩臂CLOCK方向n圈后停留在中的位置
            if(bHasKneadWidthMedPulse == TRUE)
            {
                bHasKneadWidthMedPulse = FALSE ;
                nCurKneadMotorCycles++ ;
                if(nCurKneadMotorCycles > nKneadMotorCycles)
                {
                    if(Input_GetKneadMid() == 0)
                    {
                        nCurKneadWidth = KNEAD_WIDTH_MED ;
                        nFinalKneadMotorState = STATE_IDLE ;
                        bKneadMotorInProcess = FALSE ;
                    }
                }
            }
            else
            {
                nCurKneadWidth = KNEAD_WIDTH_UNKNOWN ;
                nFinalKneadMotorState = STATE_RUN_CLOCK ;nKneadTurn = 1;
            }
            break ;
        case KNEAD_RUN_STOP_AT_MAX://按摩臂CLOCK方向n圈后停留在宽的位置
            if(bHasKneadWidthMaxPulse == TRUE)
            {
                bHasKneadWidthMaxPulse = FALSE ;
                nCurKneadMotorCycles++ ;
                if(nCurKneadMotorCycles > nKneadMotorCycles)
                {
                    if(Input_GetKneadMax() == 0)
                    {
                        nCurKneadWidth = KNEAD_WIDTH_MAX ;
                        nFinalKneadMotorState = STATE_IDLE ;
                        bKneadMotorInProcess = FALSE ;
                    }
                }
            }
            else
            {
                nCurKneadWidth = KNEAD_WIDTH_UNKNOWN ;
                nFinalKneadMotorState = STATE_RUN_CLOCK ;nKneadTurn = 1;
            }
            break ;
////////////////////////////////////////////////////////////////////////////////////////
         case KNEAD_RUN_CYCLE:
           step = nCurKneadMotorCycles % 4;
            switch(step)
            {
            case 0: 
                /**************判断是否到达最窄处*************************/
                if(bHasKneadWidthMinPulse == TRUE)//电机到达窄的位置
                {
                    bHasKneadWidthMinPulse = FALSE ;
                    nstepcnt++;
                    if(nstepcnt>=2)
                    {nstepcnt=0;
                      nCurKneadWidth = KNEAD_WIDTH_MIN ;
                      nCurKneadMotorCycles++ ;       //到达窄位置加1
                      Timer_Counter_Clear(C_TIME_RUBBING); 
                      nFinalKneadMotorState = STATE_IDLE ;
                    }
                }
                else
                {
                    nCurKneadWidth = KNEAD_WIDTH_UNKNOWN ;
                    nFinalKneadMotorState = STATE_RUN_UNCLOCK ; //还未到达最窄处，继续逆时针转动
                    nKneadTurn = 2;
                }
                /*********************************************/
                break; 
            case 1:  //停在最窄处
                /**************判断刹车时间************************/
                if(Timer_Counter(C_TIME_RUBBING,1)) 
                {
                    nCurKneadMotorCycles++ ;       //加1
                    nFinalKneadMotorState = STATE_RUN_CLOCK ; //从最窄到最宽，进行顺时针运动
                    nKneadTurn = 1;
                }
                else
                {
                    nFinalKneadMotorState = STATE_IDLE ;  //停在最窄处100ms 
                    
                }
                /*********************************************/
                break;
            case 2: 
                /**************判断是否到达最窄处*************************/
                if(bHasKneadWidthMaxPulse == TRUE)
                {
                  nstepcnt++;
                  if(nstepcnt>=2)
                  {nstepcnt=0;
                  bHasKneadWidthMaxPulse = FALSE ;
                  nCurKneadWidth = KNEAD_WIDTH_MAX ;
                  nCurKneadMotorCycles++ ;       //到达宽位置加1
                  Timer_Counter_Clear(C_TIME_RUBBING); 
                  nFinalKneadMotorState = STATE_IDLE ;
                  }
                }
                else
                {
                    nCurKneadWidth = KNEAD_WIDTH_UNKNOWN ;
                    nFinalKneadMotorState = STATE_RUN_CLOCK ; //从窄到宽顺时针转动
                    nKneadTurn = 1;
                }
                /*********************************************/
                break;  
            case 3: 
                /**************判断刹车时间************************/
                if(Timer_Counter(C_TIME_RUBBING,1)) 
                {
                    nCurKneadMotorCycles++ ;       //加1
                    
                    if(nCurKneadMotorCycles > nKneadMotorCycles)
                    {
                        nFinalKneadMotorState = STATE_IDLE ;
                        
                        if(bWalkMotorInProcess == FALSE)
                        {
                            bKneadMotorInProcess = FALSE ;
                        }
                    }
                    else
                    {
                        nFinalKneadMotorState = STATE_RUN_UNCLOCK ; //从最窄到最宽，进行顺时针运动
                        nKneadTurn = 2;
                    }
                }
                else
                {
                    nFinalKneadMotorState = STATE_IDLE ;
                }
                /*********************************************/
                break;  
            }

            break;               
////////////////////////////////////////////////////////////////////////////////////////            
            //顺时针：窄-中-宽-半圈空闲-窄     
            //逆时针：宽-中-窄-半圈空闲-宽          
            /*
            搓背程序依据  nCurKneadMotorCycles的值控制揉捏电机    
            */
        case KNEAD_RUN_RUBBING:
            step = nCurKneadMotorCycles % 4;
            switch(step)
            {
            case 0: 
                /**************判断是否到达最窄处*************************/
                if(bHasKneadWidthMinPulse == TRUE)//电机到达窄的位置
                {
                    bHasKneadWidthMinPulse = FALSE ;
                    nCurKneadWidth = KNEAD_WIDTH_MIN ;
                    nCurKneadMotorCycles++ ;       //到达窄位置加1
                    Timer_Counter_Clear(C_TIME_RUBBING); 
                    nFinalKneadMotorState = STATE_IDLE ;
                }
                else
                {
                    nCurKneadWidth = KNEAD_WIDTH_UNKNOWN ;
                    nFinalKneadMotorState = STATE_RUN_UNCLOCK ; //还未到达最窄处，继续逆时针转动
                    nKneadTurn = 2;
                }
                /*********************************************/
                break; 
            case 1:  //停在最窄处
                /**************判断刹车时间************************/
                if(Timer_Counter(C_TIME_RUBBING,1)) 
                {
                    nCurKneadMotorCycles++ ;       //加1
                    nFinalKneadMotorState = STATE_RUN_CLOCK ; //从最窄到最宽，进行顺时针运动
                    nKneadTurn = 1;
                }
                else
                {
                    nFinalKneadMotorState = STATE_IDLE ;  //停在最窄处100ms 
                    
                }
                /*********************************************/
                break;
            case 2: 
                /**************判断是否到达最窄处*************************/
                if(bHasKneadWidthMaxPulse == TRUE)
                {
                    bHasKneadWidthMaxPulse = FALSE ;
                    nCurKneadWidth = KNEAD_WIDTH_MAX ;
                    nCurKneadMotorCycles++ ;       //到达宽位置加1
                    Timer_Counter_Clear(C_TIME_RUBBING); 
                    nFinalKneadMotorState = STATE_IDLE ;
                }
                else
                {
                    nCurKneadWidth = KNEAD_WIDTH_UNKNOWN ;
                    nFinalKneadMotorState = STATE_RUN_CLOCK ; //从窄到宽顺时针转动
                    nKneadTurn = 1;
                }
                /*********************************************/
                break;  
            case 3: 
                /**************判断刹车时间************************/
                if(Timer_Counter(C_TIME_RUBBING,1)) 
                {
                    nCurKneadMotorCycles++ ;       //加1
                    
                    if(nCurKneadMotorCycles > nKneadMotorCycles)
                    {
                        nFinalKneadMotorState = STATE_IDLE ;
                        
                        if(bWalkMotorInProcess == FALSE)
                        {
                            bKneadMotorInProcess = FALSE ;
                        }
                    }
                    else
                    {
                        nFinalKneadMotorState = STATE_RUN_UNCLOCK ; //从最窄到最宽，进行顺时针运动
                        nKneadTurn = 2;
                    }
                }
                else
                {
                    nFinalKneadMotorState = STATE_IDLE ;
                }
                /*********************************************/
                break;  
            }
            
            break;
        }
    }
    //确定揉捏马达的速度
    if((nKneadMotorState == KNEAD_STOP_AT_MIN) ||
       (nKneadMotorState == KNEAD_STOP_AT_MED) ||
           (nKneadMotorState == KNEAD_STOP_AT_MAX) ||
               (nKneadMotorState == KNEAD_RUN_STOP) )
    {
        speed =  KNEAD_SPEED2_PWM;
    }
    else
    {
        switch(nCurKneadKnockSpeed)
        {
        default:  
        case 1:speed = KNEAD_SPEED1_PWM;  break ;
        case 2:speed = KNEAD_SPEED2_PWM;  break ;
        case 3:speed = KNEAD_SPEED3_PWM;  break ;
        case 4:speed = KNEAD_SPEED4_PWM;  break ;
        case 5:speed = KNEAD_SPEED5_PWM;  break ;
        case 6:speed = KNEAD_SPEED6_PWM;  break ;
        }
    }
    if(nFinalKneadMotorState == STATE_RUN_CLOCK)
    {
        KneadMotor_Control(STATE_KNEAD_CLOCK_RUN,speed);nKneadTurn = 1;
    }
    if(nFinalKneadMotorState == STATE_RUN_UNCLOCK)
    {
        KneadMotor_Control(STATE_KNEAD_UNCLOCK_RUN,speed);nKneadTurn = 2;
    }
    if(nFinalKneadMotorState == STATE_IDLE)
    {
        KneadMotor_Control(STATE_KNEAD_IDLE,speed);
    }
}


unsigned char WalkMotorControl(unsigned char nWalkMotorLocateMethod,unsigned short nWalkMotorLocateParam)
{
   //坐标更新，只有在更换动作时才执行一次
    unsigned short by_TopPosition = TOP_POSITION;
    
      if(Input_GetWalkUpSwitch() == REACH_WALK_LIMIT)
      {
        Input_SetWalkMotorPosition(TOP_POSITION);//4
      }
      if(Input_GetWalkDownSwitch() == REACH_WALK_LIMIT)
      {
        Input_SetWalkMotorPosition(0);
      }
    
    
    
    if(bUpdateLocate == TRUE)
    {
        bUpdateLocate = FALSE ;
        //nWalkMotorLocateState = nWalkMotorLocateMethod;
        switch(nWalkMotorLocateMethod)
        {
        default:  
            bWalkMotorInProcess = FALSE ;
            break;
        case WALK_SHOULDER_WAIST_1_10:
             nFinalWalkMotorLocate = nShoulderPosition - ((nShoulderPosition - WAIST_POSITION)/10);
     //        printf("S:%d\n\r",nShoulderPosition);
    //         printf("1_10:%d\n\r",nFinalWalkMotorLocate);
             break ;
        case WALK_SHOULDER_WAIST_1_9:
           //  nFinalWalkMotorLocate = nShoulderPosition -3- ((nShoulderPosition - WAIST_POSITION)/10);
          nFinalWalkMotorLocate = nShoulderPosition -15- ((nShoulderPosition - WAIST_POSITION)/10);
     //        printf("1_9:%d\n\r",nFinalWalkMotorLocate);
             break ;              
        case WALK_SHOULDER_WAIST_2_10:
             nFinalWalkMotorLocate = nShoulderPosition - ((nShoulderPosition - WAIST_POSITION)/10*2);
       //      printf("2_10:%d\n\r",nFinalWalkMotorLocate);
             break ;  
     
             
             
        case WALK_SHOULDER_WAIST_3_10:
             nFinalWalkMotorLocate = nShoulderPosition - ((nShoulderPosition - WAIST_POSITION)/10*3);
     //        printf("3_10:%d\n\r",nFinalWalkMotorLocate);
             break ;          
        case WALK_SHOULDER_WAIST_4_10:
             nFinalWalkMotorLocate = nShoulderPosition - ((nShoulderPosition - WAIST_POSITION)/10*4);
     //        printf("4_10:%d\n\r",nFinalWalkMotorLocate);
             break ;               
        case WALK_SHOULDER_WAIST_5_10:
             nFinalWalkMotorLocate = nShoulderPosition - ((nShoulderPosition - WAIST_POSITION)/10*5);
     //        printf("5_10:%d\n\r",nFinalWalkMotorLocate);
             break ;                    
        case WALK_SHOULDER_WAIST_6_10:
             nFinalWalkMotorLocate = nShoulderPosition - ((nShoulderPosition - WAIST_POSITION)/10*6);
     //        printf("6_10:%d\n\r",nFinalWalkMotorLocate);
             break ;                       
        case WALK_SHOULDER_WAIST_7_10:
             nFinalWalkMotorLocate = nShoulderPosition - ((nShoulderPosition - WAIST_POSITION)/10*7);
     //        printf("7_10:%d\n\r",nFinalWalkMotorLocate);
             break ;                       
        case WALK_SHOULDER_WAIST_8_10:
             nFinalWalkMotorLocate = nShoulderPosition - ((nShoulderPosition - WAIST_POSITION)/10*8);
     //        printf("8_10:%d\n\r",nFinalWalkMotorLocate);
             break ;                       
        case WALK_SHOULDER_WAIST_9_10:
             nFinalWalkMotorLocate = nShoulderPosition - ((nShoulderPosition - WAIST_POSITION)/10*9);
     //        printf("9_10:%d\n\r",nFinalWalkMotorLocate);
             break ;                       
        case WALK_LOCATE_WAIST:
             nFinalWalkMotorLocate = WAIST_POSITION ; 
      //       printf("W:%d\n\r",nFinalWalkMotorLocate);
             break ;
        case WALK_LOCATE_ABSULATE:    //运行到绝对位置
            nFinalWalkMotorLocate = nWalkMotorLocateParam ; 
            break ;
        case WALK_LOCATE_SHOULDER:    //运行到肩膀位置
            nFinalWalkMotorLocate =  nShoulderPosition - 10;
            break ;
        case WALK_LOCATE_TOP:  //运行到上端行程
#ifdef TOP_BY_LIMIT
            nFinalWalkMotorLocate = by_TopPosition ;
#else
            if(nBackMainRunMode == BACK_MAIN_MODE_AUTO)
            {
                if(nShoulderPosition >= by_TopPosition - DEFAULT_NECK_LENGTH)
                {
                    nFinalWalkMotorLocate = by_TopPosition ;
                }
                else
                {
                    nFinalWalkMotorLocate = nShoulderPosition + DEFAULT_NECK_LENGTH ;
                }
            }
            else
            {
                nFinalWalkMotorLocate = by_TopPosition ;
            }
#endif
            break ;
        case WALK_LOCATE_SHOULDER_OR_ABSULATE:  //由肩部位置和绝对坐标中的较小者决定
            if(nWalkMotorLocateParam > nShoulderPosition)
            {
                nFinalWalkMotorLocate = nShoulderPosition ;
            }
            else
            {
                nFinalWalkMotorLocate = nWalkMotorLocateParam ;
            }
            break ;
        case WALK_LOCATE_PARK: //停留在当前位置
            WalkMotor_Control(STATE_WALK_IDLE,0);
            nCurActionStepCounter = 0 ;
            break ;

        case WALK_LOCATE_NeckMed: //脖子中间位置
            if(nShoulderPosition >= by_TopPosition - Med_NECK_LENGTH)
            {
                nFinalWalkMotorLocate = by_TopPosition ;
            }
            else
            {
                nFinalWalkMotorLocate = nShoulderPosition + Med_NECK_LENGTH ;
            }
  //             printf("NeckMed:%d\n\r",nFinalWalkMotorLocate);
            break;
        case WALK_LOCATE_PressNeck: //脖子位置,靠近肩膀
            nFinalWalkMotorLocate = nShoulderPosition;	//10 ;
            break;
        case WALK_LOCATE_Ear:
            if(nBackMainRunMode == BACK_MAIN_MODE_AUTO)
            {
                if(nShoulderPosition >= by_TopPosition - DEFAULT_NECK_LENGTH)
                {
                    nFinalWalkMotorLocate = by_TopPosition ;
                }
                else
                {
                    nFinalWalkMotorLocate = nShoulderPosition + DEFAULT_EAR_POSITION ;
                }
            }
            else
            {
                nFinalWalkMotorLocate = by_TopPosition ;
            }
          break;
            
            
            
        }//end switch
        //保证不超过最高位
        if(nFinalWalkMotorLocate > by_TopPosition)
            nFinalWalkMotorLocate = by_TopPosition;   
    }//end if
    
    //以下判断 walk 行程（bWalkMotorInProcess）何时停止 
    
    if(nWalkMotorLocateMethod == WALK_LOCATE_PARK)
    { //判断是否到达停留时间
        WalkMotor_Control(STATE_WALK_IDLE,0);//nCurActionStepCounter
        if((nWalkMotorLocateParam != MAX_PARK_TIME) && 
           (nCurActionStepCounter >= nWalkMotorLocateParam))//nCurActionStepCounter
        {
            bWalkMotorInProcess = FALSE ;
        }
    }
    else
    {
        if(nFinalWalkMotorLocate == 0)  //行程最终位置为0
        {
            if(WalkMotor_Control(STATE_RUN_WALK_DOWN,0))
            {
                bWalkMotorInProcess = FALSE ;
            }
        }
        else if(nFinalWalkMotorLocate >= by_TopPosition) //行程最终位置为最高
        {
            if(WalkMotor_Control(STATE_RUN_WALK_UP,0))
            {
                bWalkMotorInProcess = FALSE ;
            }
        }
        else
        {   //行程最终位置为任意位置
            if(WalkMotor_Control(STATE_RUN_WALK_POSITION,nFinalWalkMotorLocate))
            {
                bWalkMotorInProcess = FALSE ;
            }
        }
    }
    return 0;
}


/*
//    nWalkMotorLocateParam=定位坐标，或停顿时间                                            
unsigned char WalkMotorControl(unsigned char nWalkMotorLocateMethod,unsigned short nWalkMotorLocateParam)
{
   //坐标更新，只有在更换动作时才执行一次
    unsigned short by_TopPosition = TOP_POSITION;
    if(bUpdateLocate == TRUE)//设置行走电机行走电机坐标更新标志，置位时更新一次坐标
    {
        bUpdateLocate = FALSE ;
        //nWalkMotorLocateState = nWalkMotorLocateMethod;
        switch(nWalkMotorLocateMethod)//行走电机定位方式，自动数组里有行走电机定位方式
        {
        default:  
            bWalkMotorInProcess = FALSE ;
            break;
        case WALK_SHOULDER_WAIST_1_10:
             nFinalWalkMotorLocate = nShoulderPosition - ((nShoulderPosition - WAIST_POSITION)/10);
             //printf("S:%d\n\r",nShoulderPosition);
             //printf("1_10:%d\n\r",nFinalWalkMotorLocate);
             break ;
         case WALK_SHOULDER_WAIST_1_9:
             nFinalWalkMotorLocate = nShoulderPosition -3- ((nShoulderPosition - WAIST_POSITION)/10);
             //printf("2_10:%d\n\r",nFinalWalkMotorLocate);
             break ;                
             
        case WALK_SHOULDER_WAIST_2_10:
             nFinalWalkMotorLocate = nShoulderPosition - ((nShoulderPosition - WAIST_POSITION)/10*2);
             //printf("2_10:%d\n\r",nFinalWalkMotorLocate);
             break ;     
        case WALK_SHOULDER_WAIST_3_10:
             nFinalWalkMotorLocate = nShoulderPosition - ((nShoulderPosition - WAIST_POSITION)/10*3);
             //printf("3_10:%d\n\r",nFinalWalkMotorLocate);
             break ;          
        case WALK_SHOULDER_WAIST_4_10:
             nFinalWalkMotorLocate = nShoulderPosition - ((nShoulderPosition - WAIST_POSITION)/10*4);
             //printf("4_10:%d\n\r",nFinalWalkMotorLocate);
             break ;               
        case WALK_SHOULDER_WAIST_5_10:
             nFinalWalkMotorLocate = nShoulderPosition - ((nShoulderPosition - WAIST_POSITION)/10*5);
             //printf("5_10:%d\n\r",nFinalWalkMotorLocate);
             break ;                    
        case WALK_SHOULDER_WAIST_6_10:
             nFinalWalkMotorLocate = nShoulderPosition - ((nShoulderPosition - WAIST_POSITION)/10*6);
             //printf("6_10:%d\n\r",nFinalWalkMotorLocate);
             break ;                       
        case WALK_SHOULDER_WAIST_7_10:
             nFinalWalkMotorLocate = nShoulderPosition - ((nShoulderPosition - WAIST_POSITION)/10*7);
             //printf("7_10:%d\n\r",nFinalWalkMotorLocate);
             break ;                       
        case WALK_SHOULDER_WAIST_8_10:
             nFinalWalkMotorLocate = nShoulderPosition - ((nShoulderPosition - WAIST_POSITION)/10*8);
             //printf("8_10:%d\n\r",nFinalWalkMotorLocate);
             break ;                       
        case WALK_SHOULDER_WAIST_9_10:
             nFinalWalkMotorLocate = nShoulderPosition - ((nShoulderPosition - WAIST_POSITION)/10*9);
             //printf("9_10:%d\n\r",nFinalWalkMotorLocate);
             break ;                       
        case WALK_LOCATE_WAIST:
             nFinalWalkMotorLocate = WAIST_POSITION ; 
             //printf("W:%d\n\r",nFinalWalkMotorLocate);
             break ;
        case WALK_LOCATE_ABSULATE:    //运行到绝对位置
            nFinalWalkMotorLocate = nWalkMotorLocateParam ; 
            break ;
        case WALK_LOCATE_SHOULDER:    //运行到肩膀位置
            nFinalWalkMotorLocate =  nShoulderPosition - 10;//nShoulderPosition=实际检测到的肩部位置
            break ;
        case WALK_LOCATE_TOP:  //运行到上端行程
#ifdef TOP_BY_LIMIT
            nFinalWalkMotorLocate = by_TopPosition ;
#else
            if(nBackMainRunMode == BACK_MAIN_MODE_AUTO)//by_TopPosition = TOP_POSITION;
            {
                if(nShoulderPosition >= by_TopPosition - DEFAULT_NECK_LENGTH)
                {
                    nFinalWalkMotorLocate = by_TopPosition ;
                }
                else
                {
                    nFinalWalkMotorLocate = nShoulderPosition + DEFAULT_NECK_LENGTH ;
                }
            }
            else
            {
                nFinalWalkMotorLocate = by_TopPosition ;
            }
#endif
            break ;
        case WALK_LOCATE_SHOULDER_OR_ABSULATE:  //由肩部位置和绝对坐标中的较小者决定
            if(nWalkMotorLocateParam > nShoulderPosition)
            {
                nFinalWalkMotorLocate = nShoulderPosition ;
            }
            else
            {
                nFinalWalkMotorLocate = nWalkMotorLocateParam ;
            }
            break ;
        case WALK_LOCATE_PARK: //停留在当前位置
            WalkMotor_Control(STATE_WALK_IDLE,0);
            nCurActionStepCounter = 0 ;
            break ;

        case WALK_LOCATE_NeckMed: //脖子中间位置
            if(nShoulderPosition >= by_TopPosition - Med_NECK_LENGTH)
            {
                nFinalWalkMotorLocate = by_TopPosition ;
            }
            else
            {
                nFinalWalkMotorLocate = nShoulderPosition + Med_NECK_LENGTH ;
            }
            
            break;
        case WALK_LOCATE_PressNeck: //脖子位置,靠近肩膀
            nFinalWalkMotorLocate = nShoulderPosition;	//10 ;
            break;
        }//end switch
        //保证不超过最高位
        if(nFinalWalkMotorLocate > by_TopPosition)
            nFinalWalkMotorLocate = by_TopPosition;   
    }//end if
    
    //以下判断 walk 行程（bWalkMotorInProcess）何时停止 
    
    if(nWalkMotorLocateMethod == WALK_LOCATE_PARK)
    { //判断是否到达停留时间
        WalkMotor_Control(STATE_WALK_IDLE,0);
        if((nWalkMotorLocateParam != MAX_PARK_TIME) && 
           (nCurActionStepCounter >= nWalkMotorLocateParam))//100ms TIMER
        {
            bWalkMotorInProcess = FALSE ;
        }
    }
    else//马达走到目标位置后停止运行
    {
        if(nFinalWalkMotorLocate == 0)  //行程最终位置为0
        {
            if(WalkMotor_Control(STATE_RUN_WALK_DOWN,0))
            {
                bWalkMotorInProcess = FALSE ;
            }
        }
        else if(nFinalWalkMotorLocate >= by_TopPosition) //行程最终位置为最高
        {
            if(WalkMotor_Control(STATE_RUN_WALK_UP,0))
            {
                bWalkMotorInProcess = FALSE ;
            }
        }
        else
        {   //行程最终位置为任意位置
            if(WalkMotor_Control(STATE_RUN_WALK_POSITION,nFinalWalkMotorLocate))//马达到达目的位置后停止运行
            {
                bWalkMotorInProcess = FALSE ;//行走电机停止运行
            }
        }
    }
    return 0;
}
*/





void startBodyDetect(void)
{
    //往上走
    nFinalWalkMotorLocate = TOP_POSITION;
    //肩部检测清空
    //bodyDetectSuccess = 0;
    //检测步骤清零
    //shoulderPositionScanStep = 0;
}





/*
#define BACK_SUB_MODE_KNEAD			7
#define BACK_SUB_MODE_KNOCK			8
#define BACK_SUB_MODE_PRESS			9
#define BACK_SUB_MODE_WAVELET			10// //揉捏同步
#define BACK_SUB_MODE_SOFT_KNOCK		11////叩击
#define BACK_SUB_MODE_MUSIC			12////韵律按摩
{BACK_SUB_MODE_KNEAD...BACK_SUB_MODE_MUSIC}之间的一种按摩手法
*/

void walkRefreshUp(unsigned char key)
{
  if(nKeyBackLocate == LOCATE_NONE)
    {
      nKeyBackLocate = LOCATE_FULL_BACK ;
    }
  if(nKeyBackLocate == LOCATE_FULL_BACK)		//全程
  {
    nKeyBackLocate = LOCATE_FULL_BACK;
    {
      ManualDirector[0].nWalkMotorLocateMethod = WALK_LOCATE_TOP ;
      ManualDirector[0].nWalkMotorLocateParam = 0 ;      
      ManualDirector[1].nWalkMotorLocateMethod = WALK_LOCATE_ABSULATE ;
      ManualDirector[1].nWalkMotorLocateParam = 0 ;
      ManualDirector[2].nWalkMotorLocateMethod = WALK_LOCATE_TOP ;
      ManualDirector[2].nWalkMotorLocateParam = 0 ;
      ManualDirector[3].nWalkMotorLocateMethod = WALK_LOCATE_ABSULATE ;
      ManualDirector[3].nWalkMotorLocateParam = 0 ;

    }
  }
  if(nKeyBackLocate == LOCATE_PARTIAL)
  {
    nKeyBackLocate = LOCATE_PARTIAL ;
    if(Input_GetWalkMotorPosition() >= (TOP_POSITION - HALF_PARTIAL_DIFF))
    {
      nPartialTop = TOP_POSITION ;
      nPartialBottom = Input_GetWalkMotorPosition() - PARTIAL_DIFF ;
    }
    else if(Input_GetWalkMotorPosition() <= HALF_PARTIAL_DIFF)
    {
      nPartialTop = PARTIAL_DIFF ;
      nPartialBottom = 0 ;
    }
    else
    {
      nPartialTop = Input_GetWalkMotorPosition() + HALF_PARTIAL_DIFF ;
      nPartialBottom = Input_GetWalkMotorPosition() - HALF_PARTIAL_DIFF ;
    }
    {
      ManualDirector[0].nWalkMotorLocateMethod = WALK_LOCATE_ABSULATE ;
      ManualDirector[0].nWalkMotorLocateParam = nPartialTop ; ;      
      ManualDirector[1].nWalkMotorLocateMethod = WALK_LOCATE_ABSULATE ;
      ManualDirector[1].nWalkMotorLocateParam = nPartialBottom ;
      ManualDirector[2].nWalkMotorLocateMethod = WALK_LOCATE_ABSULATE ;
      ManualDirector[2].nWalkMotorLocateParam = nPartialTop ; ;
      ManualDirector[3].nWalkMotorLocateMethod = WALK_LOCATE_ABSULATE ;
      ManualDirector[3].nWalkMotorLocateParam = nPartialBottom ;

    }
  }
  if(nKeyBackLocate == LOCATE_POINT)
  {
    nKeyBackLocate = LOCATE_POINT ;
    ManualDirector[0].nWalkMotorLocateMethod = WALK_LOCATE_PARK ;
    ManualDirector[0].nWalkMotorLocateParam = MAX_PARK_TIME ;
    ManualDirector[1].nWalkMotorLocateMethod = WALK_LOCATE_PARK ;
    ManualDirector[1].nWalkMotorLocateParam = MAX_PARK_TIME ;
    ManualDirector[2].nWalkMotorLocateMethod = WALK_LOCATE_PARK ;
    ManualDirector[2].nWalkMotorLocateParam = MAX_PARK_TIME ;
    ManualDirector[3].nWalkMotorLocateMethod = WALK_LOCATE_PARK ;
    ManualDirector[3].nWalkMotorLocateParam = MAX_PARK_TIME ;
  }
  bBackManualModeInit = TRUE ;
}
void walkRefreshDown(unsigned char key)
{
  if(nKeyBackLocate == LOCATE_NONE)
    {
      nKeyBackLocate = LOCATE_FULL_BACK ;
    }
  if(nKeyBackLocate == LOCATE_FULL_BACK)		//全程
  {
    nKeyBackLocate = LOCATE_FULL_BACK;
    {
      ManualDirector[0].nWalkMotorLocateMethod = WALK_LOCATE_ABSULATE ;
      ManualDirector[0].nWalkMotorLocateParam = 0 ;
      ManualDirector[1].nWalkMotorLocateMethod = WALK_LOCATE_TOP ;
      ManualDirector[1].nWalkMotorLocateParam = 0 ;
      ManualDirector[2].nWalkMotorLocateMethod = WALK_LOCATE_ABSULATE ;
      ManualDirector[2].nWalkMotorLocateParam = 0 ;
      ManualDirector[3].nWalkMotorLocateMethod = WALK_LOCATE_TOP ;
      ManualDirector[3].nWalkMotorLocateParam = 0 ;
    }
  }
  if(nKeyBackLocate == LOCATE_PARTIAL)
  {
    nKeyBackLocate = LOCATE_PARTIAL ;
    if(Input_GetWalkMotorPosition() >= (TOP_POSITION - HALF_PARTIAL_DIFF))
    {
      nPartialTop = TOP_POSITION ;
      nPartialBottom = Input_GetWalkMotorPosition() - PARTIAL_DIFF ;
    }
    else if(Input_GetWalkMotorPosition() <= HALF_PARTIAL_DIFF)
    {
      nPartialTop = PARTIAL_DIFF ;
      nPartialBottom = 0 ;
    }
    else
    {
      nPartialTop = Input_GetWalkMotorPosition() + HALF_PARTIAL_DIFF ;
      nPartialBottom = Input_GetWalkMotorPosition() - HALF_PARTIAL_DIFF ;
    }
    {
      ManualDirector[0].nWalkMotorLocateMethod = WALK_LOCATE_ABSULATE ;
      ManualDirector[0].nWalkMotorLocateParam = nPartialBottom ;
      ManualDirector[1].nWalkMotorLocateMethod = WALK_LOCATE_ABSULATE ;
      ManualDirector[1].nWalkMotorLocateParam = nPartialTop ; ;
      ManualDirector[2].nWalkMotorLocateMethod = WALK_LOCATE_ABSULATE ;
      ManualDirector[2].nWalkMotorLocateParam = nPartialBottom ;
      ManualDirector[3].nWalkMotorLocateMethod = WALK_LOCATE_ABSULATE ;
      ManualDirector[3].nWalkMotorLocateParam = nPartialTop ; ;
    }
  }
  if(nKeyBackLocate == LOCATE_POINT)
  {
    nKeyBackLocate = LOCATE_POINT ;
    ManualDirector[0].nWalkMotorLocateMethod = WALK_LOCATE_PARK ;
    ManualDirector[0].nWalkMotorLocateParam = MAX_PARK_TIME ;
    ManualDirector[1].nWalkMotorLocateMethod = WALK_LOCATE_PARK ;
    ManualDirector[1].nWalkMotorLocateParam = MAX_PARK_TIME ;
    ManualDirector[2].nWalkMotorLocateMethod = WALK_LOCATE_PARK ;
    ManualDirector[2].nWalkMotorLocateParam = MAX_PARK_TIME ;
    ManualDirector[3].nWalkMotorLocateMethod = WALK_LOCATE_PARK ;
    ManualDirector[3].nWalkMotorLocateParam = MAX_PARK_TIME ;
  }
  bBackManualModeInit = TRUE ;
}










void refreshAutoDirector(void)
{
  
  unsigned int position = Input_GetWalkMotorPosition();
  nCurSubFunction = AutoDirector.nSubFunction ;
  nCurKneadKnockSpeed = AutoDirector.nKneadKnockSpeed ;
  //设置行走电机
  if(nBackMainRunMode == BACK_MAIN_MODE_3D)
  {
    if(Input_GetWalkMotorPosition() > _3D_MOTOR_WALK_MAX_POSITION)
    nCurKneadKnockSpeed = 0;
    
    switch(nKeyBackLocate)//LOCATE_FULL_BACK:
    {
    case LOCATE_POINT:
      bWalkMotorInProcess = TRUE ;
      bUpdateLocate = TRUE ;
      nWalkMotorControlParam1 = WALK_LOCATE_PARK ;//AutoDirector.nWalkMotorLocateMethod ;
      nWalkMotorControlParam2 = 0;//MAX_PARK_TIME ;//AutoDirector.nWalkMotorLocateParam ; 
      //WalkMotor_Control(STATE_WALK_IDLE,0);
                                       //nCurActionStepCounter = 0 ;
      break;
    case LOCATE_PARTIAL:
      if(bKeyWalkUp == TRUE)
      {
        bWalkMotorInProcess = TRUE ;
        bUpdateLocate = TRUE ;
        nWalkMotorControlParam1 = WALK_LOCATE_TOP;//WALK_LOCATE_ABSULATE ;
        nWalkMotorControlParam2 = 0;//nPartialBottom ;
        break;
      }
      if(bKeyWalkDown == TRUE)
      {
        bWalkMotorInProcess = TRUE ;
        bUpdateLocate = TRUE ;
        nWalkMotorControlParam1 = WALK_LOCATE_ABSULATE;//WALK_LOCATE_ABSULATE ;
        nWalkMotorControlParam2 = 0;//nPartialBottom ;
        break;
      }
      /*
      //
      if(Input_GetWalkMotorPosition() >= (TOP_POSITION - HALF_PARTIAL_DIFF))
      {
        nPartialTop = TOP_POSITION ;
        nPartialBottom = Input_GetWalkMotorPosition() - PARTIAL_DIFF ;
      }
      else if(Input_GetWalkMotorPosition() <= HALF_PARTIAL_DIFF)
      {
        nPartialTop = PARTIAL_DIFF ;
        nPartialBottom = 0 ;
      }
      else
      {
        nPartialTop = Input_GetWalkMotorPosition() + HALF_PARTIAL_DIFF ;
        nPartialBottom = Input_GetWalkMotorPosition() - HALF_PARTIAL_DIFF ;
      }*/
      if(n3Dpointturn%2==0)
      {
        bWalkMotorInProcess = TRUE ;
        bUpdateLocate = TRUE ;
        nWalkMotorControlParam1 = WALK_LOCATE_ABSULATE ;
        nWalkMotorControlParam2 = nPartialBottom ;
      }
      else
      {
        bWalkMotorInProcess = TRUE ;
        bUpdateLocate = TRUE ;
        nWalkMotorControlParam1 = WALK_LOCATE_ABSULATE ;
        nWalkMotorControlParam2 = nPartialTop ;
      }
      break;
    case LOCATE_FULL_BACK:
      bWalkMotorInProcess = TRUE ;
      bUpdateLocate = TRUE ;
      nWalkMotorControlParam1 = AutoDirector.nWalkMotorLocateMethod ;
      nWalkMotorControlParam2 = AutoDirector.nWalkMotorLocateParam ;
      break;
    }
  }
  else
  {
    bWalkMotorInProcess = TRUE ;
    bUpdateLocate = TRUE ;
    nWalkMotorControlParam1 = AutoDirector.nWalkMotorLocateMethod ;
    nWalkMotorControlParam2 = AutoDirector.nWalkMotorLocateParam ;
  }
  
  
  //设置揉捏电机
  bKneadMotorInProcess = TRUE ;
  nKneadMotorControlParam1 = AutoDirector.nKneadMotorState ;
  nKneadMotorControlParam2 = AutoDirector.nKneadMotorCycles ;
  //设置捶击电机
  bKnockMotorInProcess = TRUE ;
  nKnockMotorControlParam1 = AutoDirector.nKnockMotorState ;
  nKnockMotorControlParam2 = AutoDirector.nKnockMotorRunTime ;
  nKnockMotorControlParam3 = AutoDirector.nKnockMotorStopTime ;
  
  
              if((bTapping == 0)&&(AutoDirector.nSubFunction == BACK_SUB_MODE_SOFT_KNOCK))//MODY BY WGH 20161019
              {
                AutoDirector.nSubFunction = BACK_SUB_MODE_PRESS;
                bKnockMotorInProcess = TRUE ;
                nKnockMotorControlParam1 = 0 ;
                nKnockMotorControlParam2 = 0 ;
                nKnockMotorControlParam3 = 0 ;
                nCurSubFunction = BACK_SUB_MODE_PRESS;
                nCurKneadKnockSpeed = 0 ;
              }
              if((bTapping == 0)&&(AutoDirector.nSubFunction == BACK_SUB_MODE_KNOCK))
              {
                AutoDirector.nSubFunction = BACK_SUB_MODE_PRESS;
                bKnockMotorInProcess = TRUE ;
                nKnockMotorControlParam1 = 0 ;
                nKnockMotorControlParam2 = 0 ;
                nKnockMotorControlParam3 = 0 ;
                nCurSubFunction = BACK_SUB_MODE_PRESS;
                nCurKneadKnockSpeed = 0 ;
              }
              if((bTapping == 0)&&(AutoDirector.nSubFunction == BACK_SUB_MODE_WAVELET))
              {
                AutoDirector.nSubFunction = BACK_SUB_MODE_KNEAD;
                bKnockMotorInProcess = TRUE ;
                nKnockMotorControlParam1 = 0 ;
                nKnockMotorControlParam2 = 0 ;
                nKnockMotorControlParam3 = 0 ;
                       
                bKneadMotorInProcess = TRUE ;
                nKneadMotorControlParam1 = KNEAD_RUN ;
                nKneadMotorControlParam2 = 0 ;
                nCurKneadKnockSpeed = AutoDirector.nKneadKnockSpeed ;;          
              }
  
  
  
  
  //设置3D马达力度 
  
  if((nBackMainRunMode == BACK_MAIN_MODE_3D)&&(nKeyBackLocate == LOCATE_POINT))
  {
    if(n3Dpointturn%2==0)
    {
      b3D_MotorInProcess = TRUE ;
      n3D_MotorControlState = _3D_PROGRAM;//AutoDirector.n3D_MotorState ;
      n3D_MotorControlPosition = AXIS_STRONGEST;//AutoDirector.n3D_MotorPosition ;
      nSetAxisStrength = n3D_MotorControlPosition;
      bAxisUpdate = true; 
      n3D_MotorControlSpeed = _3D_SPEED_5;//AutoDirector.n3D_MotorSpeed ;
      n3D_MotorControlStopTime = 20;//AutoDirector.n3D_MotorStopTime ;         
      
    }
    else
    {
    b3D_MotorInProcess = TRUE ;
    n3D_MotorControlState = _3D_PROGRAM;//AutoDirector.n3D_MotorState ;
    n3D_MotorControlPosition = AXIS_WEAKEST;//AutoDirector.n3D_MotorPosition ;
    nSetAxisStrength = n3D_MotorControlPosition;
    bAxisUpdate = true; 
    n3D_MotorControlSpeed = _3D_SPEED_5;//AutoDirector.n3D_MotorSpeed ;
    n3D_MotorControlStopTime = 20;//AutoDirector.n3D_MotorStopTime ;    
    
    }
    
  }
  else
  {
    b3D_MotorInProcess = TRUE ;
    n3D_MotorControlState = AutoDirector.n3D_MotorState ;
    n3D_MotorControlPosition = AutoDirector.n3D_MotorPosition ;
    nSetAxisStrength = n3D_MotorControlPosition;
    bAxisUpdate = true; 
    n3D_MotorControlSpeed = AutoDirector.n3D_MotorSpeed ;
    n3D_MotorControlStopTime = AutoDirector.n3D_MotorStopTime ;      
  }
  
  
}
void Main_BackProce(void)
{        
    if(st_Stretch.active)//在拉腿时刻执行拉腿程序的程序数组，不执行自动程序数组1
     {
           return;
     }
    switch(nBackMainRunMode)
    {
     case BACK_MAIN_MODE_DEMO:  
      if(bBackAutoModeInit == TRUE)
        {
            bBackAutoModeInit = FALSE;
            nMaxActionStep = BACK_AUTO_STEPS[nBackSubRunMode];
            nStartActionStep = BACK_AUTO_START_STEP[nBackSubRunMode];
            bGetNextActionStep = TRUE;
            nCurActionStep = 0;
            nStretchStep = 0;
            //如果不等当前的动作完成，而强行将InProcess置成FALSE，会造成冲顶
        }
        else
        {
          if((bWalkMotorInProcess == FALSE) &&
             (bKneadMotorInProcess == FALSE) &&
               (bKnockMotorInProcess == FALSE)&&
                 (b3D_MotorInProcess == FALSE))
          {
            nCurActionStep++ ; //自动程序步骤增加
            if(nCurActionStep >= nMaxActionStep)
            {
              nCurActionStep = nStartActionStep ;
            }
            bGetNextActionStep = TRUE ;
          }
        }
        if(bGetNextActionStep == TRUE)
        {
          bGetNextActionStep = FALSE ;
          switch(nBackSubRunMode)
          {
           
          case BACK_SUB_MODE_AUTO_0:
            AutoDirector = AutoFunction0[nCurActionStep] ;                                            
            break ;
          case BACK_SUB_MODE_AUTO_1:
          
            AutoDirector = AutoFunction1[nCurActionStep] ;
            break ;
          case BACK_SUB_MODE_AUTO_2:
            AutoDirector = AutoFunction2[nCurActionStep] ;
            break ;
          case BACK_SUB_MODE_AUTO_3:
            AutoDirector = AutoFunction3[nCurActionStep] ;
            break ;
          case BACK_SUB_MODE_AUTO_4:
            AutoDirector = AutoFunction4[nCurActionStep] ;
            break ;
          case BACK_SUB_MODE_AUTO_5:
            AutoDirector = AutoFunction5[nCurActionStep] ;
            break;
          case BACK_SUB_MODE_3D1:
            AutoDirector = _3DFunction0[nCurActionStep] ;
            break ;
          case BACK_SUB_MODE_3D2:
            AutoDirector = _3DFunction1[nCurActionStep] ;
            break ;
          case BACK_SUB_MODE_3D3:
            AutoDirector = _3DFunction2[nCurActionStep] ;
            break ;   
          case BACK_SUB_MODE_DEMO:
            AutoDirector = DemoFunction[nCurActionStep] ;
            break ;       
            
            
          }
          //每次更换动作需要更新的变量
          //行走和敲击马达共有一个时间计数器，当敲击计数开始时， 行走马达不可以计数，必须停止在当前位置
          nCurActionStepCounter = 0 ;//当前动作时间计数（行走或敲击、行走时间计数）  ,仅用于行走或敲击时间计数,两个计数器不可以同时进行计数
          nCurShoulderAdjustCounter = 0 ;
          if(!((nCurSubFunction == BACK_SUB_MODE_SOFT_KNOCK) && (AutoDirector.nSubFunction == BACK_SUB_MODE_SOFT_KNOCK)))
          {
            nCurKnockRunStopCounter = 0 ;//叩击动作记数器
          }
          nCurKneadMotorCycles = 0 ;//揉捏圈数计数（揉捏）
          refreshAutoDirector();
        }
      break;
    case BACK_MAIN_MODE_AUTO:
    case BACK_MAIN_MODE_3D:    
 //  case BACK_MAIN_MODE_CLOUD  :
      if(bBackAutoModeInit == TRUE)//当用户选中自动程序按摩开始时，靠背电机的初始化标志位
        {
            bBackAutoModeInit = FALSE;
            
            if(nBackSubRunMode < BACK_SUB_MODE_3D1)//选用5种自动程序
            {
              nMaxActionStep = BACK_AUTO_STEPS[nBackSubRunMode];//自动程序运行的步数，主要是AutoFunction0数组的长度
              nStartActionStep = BACK_AUTO_START_STEP[nBackSubRunMode];//从第0步开始执行
            }
            else 
            {
              
                if(nBackSubRunMode<=BACK_SUB_MODE_DEMO)
                {
                   nMaxActionStep = BACK_AUTO_STEPS[nBackSubRunMode-BACK_SUB_MODE_3D1+6];//a[6-8]为3种3D程序
               
                   nStartActionStep = BACK_AUTO_START_STEP[nBackSubRunMode-BACK_SUB_MODE_3D1+6];
                }
                else if(nBackSubRunMode == BACK_SUB_MODE_6MIN_DEMO  )
                {
                   nMaxActionStep = BACK_AUTO_STEPS[BACK_SUB_MODE_AUTO_1];//a[6-8]为3种3D程序
                   nStartActionStep = BACK_AUTO_START_STEP[BACK_SUB_MODE_AUTO_1];
                }
                else
                {
                    nMaxActionStep = BACK_CLOUD_STEPS[nBackSubRunMode-BACK_SUB_MODE_CLUDE_AUTO_0];//a[6-8]为3种3D程序
                    nStartActionStep = BACK_CLOUD_START_STEP[nBackSubRunMode-BACK_SUB_MODE_CLUDE_AUTO_0];
                }
            } 
            
            
            nMaxActionStep = BACK_AUTO_STEPS[nBackSubRunMode];//自动程序运行的步数，主要是AutoFunction0数组的长度
            nStartActionStep = BACK_AUTO_START_STEP[nBackSubRunMode];//从第0步开始执行
            
            bGetNextActionStep = TRUE;
            nCurActionStep = 0;
            nStretchStep = 0;
            nCur3D_MotorStopCounter = 0;//3D自动程序到达目标位置的停止时间
            //如果不等当前的动作完成，而强行将InProcess置成FALSE，会造成冲顶
        }
        else////如果不等当前的动作完成，而强行将InProcess置成FALSE，会造成冲顶
        {

          //所有电机动作完成后才会进行自动程序下一步数组的动作，
            if((bWalkMotorInProcess == FALSE) &&
                    (bKneadMotorInProcess == FALSE) &&
                    (bKnockMotorInProcess == FALSE)&&
                      (b3D_MotorInProcess == FALSE))
            {
                nCurActionStep++ ; //自动程序步骤增加
                if(nCurActionStep ==1)b6MinDemoStretchEnable = TRUE;
                if((nBackSubRunMode == BACK_SUB_MODE_6MIN_DEMO)&&(nCurActionStep == 3)&&(b6MinDemoStretch == FALSE)&&(b6MinDemoStretchEnable == TRUE))
                {
                    nPreActionStep = nCurActionStep;
                    b6MinDemoStretch = TRUE;
                    //if(b6MinDemoStretchEnable == TRUE)
                    {
                        b6MinDemoStretchEnable = FALSE;
                        //b6MinDemoStretch = FALSE;
                        
                        st_Stretch.active = TRUE;
                        SetStretchingEnable(1);
                        st_Stretch.init = TRUE; 
                        st_Stretch.times = 3;
                        bZeroflash = FALSE;
                        bKeyWaistHeat = TRUE ;
                    }
                    
                    
                    
                    
                }
                if((nBackSubRunMode == BACK_SUB_MODE_6MIN_DEMO)&&(nCurActionStep == 4))
                {         
                    RockFunctionEnable(true);
                    nRockMode = ROCK_AUTO;
                }         
                
                if((nBackSubRunMode == BACK_SUB_MODE_6MIN_DEMO)&&(nCurActionStep == 8))
                {         

                    RockFunctionEnable(false);
                    //回到第一零重力状态
                    nTargetMassagePosition = MASSAGE_OPTIMAL2_POSITION;
                    bMassagePositionUpdate = TRUE;    
                    
                    nCurActionStep = 9;  
                }             
                
                
                
                
                
                if(nCurActionStep >= nMaxActionStep)
                {
                  nCurActionStep = nStartActionStep ;
                }

                
            
                bGetNextActionStep = TRUE ;///所有电机动作完成后才会进行自动程序下一步数组的动作，
            }
            
            //bTapping
              if((bTapping == 0)&&(AutoDirector.nSubFunction == BACK_SUB_MODE_SOFT_KNOCK))//MODY BY WGH 20161019
              {
                AutoDirector.nSubFunction = BACK_SUB_MODE_PRESS;
                bKnockMotorInProcess = TRUE ;
                nKnockMotorControlParam1 = 0 ;
                nKnockMotorControlParam2 = 0 ;
                nKnockMotorControlParam3 = 0 ;
                nCurSubFunction = BACK_SUB_MODE_PRESS;
                nCurKneadKnockSpeed = 0 ;
              }
              if((bTapping == 0)&&(AutoDirector.nSubFunction == BACK_SUB_MODE_KNOCK))
              {
                AutoDirector.nSubFunction = BACK_SUB_MODE_PRESS;
                bKnockMotorInProcess = TRUE ;
                nKnockMotorControlParam1 = 0 ;
                nKnockMotorControlParam2 = 0 ;
                nKnockMotorControlParam3 = 0 ;
                nCurSubFunction = BACK_SUB_MODE_PRESS;
                nCurKneadKnockSpeed = 0 ;
              }
              if((bTapping == 0)&&(AutoDirector.nSubFunction == BACK_SUB_MODE_WAVELET))
              {
                AutoDirector.nSubFunction = BACK_SUB_MODE_KNEAD;
                bKnockMotorInProcess = TRUE ;
                nKnockMotorControlParam1 = 0 ;
                nKnockMotorControlParam2 = 0 ;
                nKnockMotorControlParam3 = 0 ;
                       
                bKneadMotorInProcess = TRUE ;
                nKneadMotorControlParam1 = KNEAD_RUN ;
                nKneadMotorControlParam2 = 0 ;
                
                nCurSubFunction = BACK_SUB_MODE_KNEAD;
                nCurKneadKnockSpeed = AutoDirector.nKneadKnockSpeed ;;          
              }
            /*定點指壓*/
            if((bTapping ==0)&&(AutoDirector.nSubFunction == BACK_SUB_MODE_PRESS)&&(nWalkMotorControlParam1 == WALK_LOCATE_PARK))
            {
              //AutoDirector.nSubFunction = BACK_SUB_MODE_PRESS;
              switch(pressstep)
              {
              case 0:
              default:
                
                b3D_MotorInProcess = TRUE;
                n3D_MotorControlState = _3D_MANUAL_AUTO_VECTOR;//
                n3D_MotorControlPosition = 2;//3D机芯回到力度第一点
                n3D_MotorControlSpeed = 4;//
                n3D_MotorControlStopTime = 0;//
                pressstep++;
                presstime=0;
                break;
              case 1:
                if(b3D_MotorInProcess == false)//停4秒
                {
                  if(presstime >= 20)
                  {
                    pressstep++;
                  }
                }
                break;
              case 2:  
                
                b3D_MotorInProcess = TRUE;
                n3D_MotorControlState = _3D_MANUAL_AUTO_VECTOR;//
                switch(nKeyAxisStrength)//依据力度，设定往外的X轴位移
                {
                case 0:
                  n3D_MotorControlPosition = 10;
                  break;
                case 1:
                  n3D_MotorControlPosition = 17;
                  break;  
                case 2:
                  n3D_MotorControlPosition = 24;
                  break; 
                case 3:
                  n3D_MotorControlPosition = 31;
                  break; 
                case 4:
                  n3D_MotorControlPosition = 38;
                  break; 
                }
                
                
                
                n3D_MotorControlSpeed = 6;//ManualDirector[nCurActionStep].n3D_MotorSpeed ;
                n3D_MotorControlStopTime = 0;//ManualDirector[nCurActionStep].n3D_MotorStopTime ;
                pressstep++;
                presstime=0;
                break;
              case 3:
                if(b3D_MotorInProcess == false)//停4秒
                {
                  if(presstime >= 20)
                  {
                    pressstep++;
                  }
                }
                break;
              }//SWITCH
            }
              //
            
        }
      //refreshAutoDirector();//将bWalkMotorInProcess=true,bKneadMotorInProcess=true,bKnockMotorInProcess=true,b3D_MotorInProcess=true，机芯开始按照自动程序开始运行
      //进行下一布动作时 ，将电机动作标志位置1，bWalkMotorInProcess=true,bKneadMotorInProcess=true,bKnockMotorInProcess=true,b3D_MotorInProcess=true，
        if(bGetNextActionStep == TRUE)//开始下一步运行的标志位， //所有电机动作完成后才会进行自动程序下一步数组的动作，
        {
          bGetNextActionStep = FALSE ;
     //     printf("step:%d,time:%d\n",nCurActionStep,Data_Get_TimeSecond());
          
           n3Dpointturn ++;
          switch(nBackSubRunMode)
          {
          
          case BACK_SUB_MODE_AUTO_0:
           AutoDirector = AutoFunction0[nCurActionStep] ;//将自动程序结构体数组的值赋给结构体变量                   
            
            break ;
          case BACK_SUB_MODE_AUTO_1:
           AutoDirector = AutoFunction1[nCurActionStep] ;
            break ;
          case BACK_SUB_MODE_AUTO_2:
            AutoDirector = AutoFunction2[nCurActionStep] ;
            break ;
          case BACK_SUB_MODE_AUTO_3:
            AutoDirector = AutoFunction3[nCurActionStep] ;
            break ;
          case BACK_SUB_MODE_AUTO_4:
            AutoDirector = AutoFunction4[nCurActionStep] ;
            break ;
          case BACK_SUB_MODE_AUTO_5:
            AutoDirector = AutoFunction5[nCurActionStep] ;
            break;
          case BACK_SUB_MODE_3D1:
            AutoDirector = _3DFunction0[nCurActionStep] ;
            break ;
          case BACK_SUB_MODE_3D2:
            AutoDirector = _3DFunction1[nCurActionStep] ;
            break ;
          case BACK_SUB_MODE_3D3:
            AutoDirector = _3DFunction2[nCurActionStep] ;
            break ;   
          case BACK_SUB_MODE_6MIN_DEMO:
            AutoDirector = Demo6MinFunction[nCurActionStep] ;
            break ;
            
            
          }
          //每次更换动作需要更新的变量
          nCurActionStepCounter = 0 ;//自动程序中 当前动作时间计数（行走或敲击时间计数）
          nCurShoulderAdjustCounter = 0 ;
          if(!((nCurSubFunction == BACK_SUB_MODE_SOFT_KNOCK) && (AutoDirector.nSubFunction == BACK_SUB_MODE_SOFT_KNOCK)))
          {
            nCurKnockRunStopCounter = 0 ;//叩击动作记数器
          }
   
          nCurKneadMotorCycles = 0 ;//揉捏圈数计数（揉捏）
          refreshAutoDirector();//将bWalkMotorInProcess=true,bKneadMotorInProcess=true,bKnockMotorInProcess=true,b3D_MotorInProcess=true，机芯开始按照自动程序开始运行
        }
        break ;
      
      
      
      
      
             
    case BACK_MAIN_MODE_MANUAL:
        if(bBackManualModeInit == TRUE)
        {
            bBackManualModeInit = FALSE ;
            bGetNextActionStep = TRUE ;
            nCurActionStep = 0 ;
            b3D_MotorInProcess = false;
            n3D_MotorControlState = _3D_MANUAL ;
        }
        else
        {
            if((bWalkMotorInProcess == FALSE) &&
               (bKneadMotorInProcess == FALSE) &&
               (bKnockMotorInProcess == FALSE) /*&&
                 (b3D_MotorInProcess == FALSE)*/)
            {
                nCurActionStep++ ;
                if(nCurActionStep >= nMaxActionStep)
                {
                    nCurActionStep = nStartActionStep ;
                }
                bGetNextActionStep = TRUE ;
            }
        }
        if(bGetNextActionStep == TRUE)
        {
            bGetNextActionStep = FALSE ;
            //每次更换动作需要更新的变量
            nCurActionStepCounter = 0 ;//当前动作时间计数（行走或敲击时间计数）
            nCurKnockRunStopCounter = 0 ;//叩击动作记数器
            nCurKneadMotorCycles = 0 ;//揉捏圈数计数（揉捏）
            nCur3D_MotorStopCounter = 0;//3D电机到达目标位置的停止时间
            nCurSubFunction = ManualDirector[nCurActionStep].nSubFunction ;
            nCurKneadKnockSpeed = ManualDirector[nCurActionStep].nKneadKnockSpeed ;
            //设置行走电机
            bWalkMotorInProcess = TRUE ;//设置行走电机正在运行标志位，停止运行时该位为0
            bUpdateLocate = TRUE ;//行走电机坐标更新标志，置位时更新一次坐标
            nWalkMotorControlParam1 = ManualDirector[nCurActionStep].nWalkMotorLocateMethod ;
            nWalkMotorControlParam2 = ManualDirector[nCurActionStep].nWalkMotorLocateParam ;
            //设置揉捏电机
            bKneadMotorInProcess = TRUE ;
            nKneadMotorControlParam1 = ManualDirector[nCurActionStep].nKneadMotorState ;
            nKneadMotorControlParam2 = ManualDirector[nCurActionStep].nKneadMotorCycles ;
            //设置捶击电机
            bKnockMotorInProcess = TRUE ;
            nKnockMotorControlParam1 = ManualDirector[nCurActionStep].nKnockMotorState ;
            nKnockMotorControlParam2 = ManualDirector[nCurActionStep].nKnockMotorRunTime ;
            nKnockMotorControlParam3 = ManualDirector[nCurActionStep].nKnockMotorStopTime ;
            /*
            //设置3D马达
            b3D_MotorInProcess = TRUE;
            n3D_MotorControlState = ManualDirector[nCurActionStep].n3D_MotorState ;
            n3D_MotorControlPosition = ManualDirector[nCurActionStep].n3D_MotorPosition ;
            n3D_MotorControlSpeed = ManualDirector[nCurActionStep].n3D_MotorSpeed ;
            n3D_MotorControlStopTime = ManualDirector[nCurActionStep].n3D_MotorStopTime ;
            */
        }
        
        
      
        if((nMaunalSubMode == nMaunalSubMode_PRESS)&&(nKeyBackLocate == LOCATE_POINT))
        {
          switch(pressstep)
          {
          case 0:
          default:

            b3D_MotorInProcess = TRUE;
            n3D_MotorControlState = _3D_MANUAL_AUTO_VECTOR;//
            n3D_MotorControlPosition = 2;//3D机芯回到力度第一点
            n3D_MotorControlSpeed = 4;//
            n3D_MotorControlStopTime = 0;//
            pressstep++;
            presstime=0;
            break;
          case 1:
            if(b3D_MotorInProcess == false)//停4秒
            {
              if(presstime >= 40)
              {
              pressstep++;
              }
            }
            break;
         case 2:  

            b3D_MotorInProcess = TRUE;
            n3D_MotorControlState = _3D_MANUAL_AUTO_VECTOR;//
            switch(nKeyAxisStrength)//依据力度，设定往外的X轴位移
            {
            case 0:
              n3D_MotorControlPosition = 2;
              break;
            case 1:
              n3D_MotorControlPosition = 10;
              break;  
            case 2:
              n3D_MotorControlPosition = 20;
              break; 
            case 3:
              n3D_MotorControlPosition = 30;
              break; 
            case 4:
              n3D_MotorControlPosition = 38;
              break; 
            }
            
            
            
            n3D_MotorControlSpeed = 6;//ManualDirector[nCurActionStep].n3D_MotorSpeed ;
            n3D_MotorControlStopTime = 0;//ManualDirector[nCurActionStep].n3D_MotorStopTime ;
            pressstep++;
            presstime=0;
            break;
          case 3:
            if(b3D_MotorInProcess == false)//停4秒
            {
              if(presstime >= 40)
              {
              pressstep++;
              }
            }
            break;
          }
        }//DOIT
        
        break;
    }
}

int Main_FlexPad_Proce(void)
{
    int retval = 0;
    if(st_Stretch.active) return 0;
    if(bMassagePositionUpdate) return 0;
    
    if(bKeyFlexOut == TRUE)//按下小腿伸按键
    {
      FlexMotorSetDisable();//  FlexMotorEnable = false;
      if(Flex_ControlOut(FLEX_MOTOR_CURRENT_2A))//(FLEX_MOTOR_CURRENT_3A))
      {

        nBuzzerMode = BUZZER_MODE_FAST ;
        bSendBuzzerMode = TRUE ;//发送蜂鸣器声音使能信号，手控板
        bBlueToothSendBuzzerMode = TRUE;//发送蜂鸣器声音使能信号，平板
        retval = 1; 

      }
      else      
      {
        nBuzzerMode = BUZZER_MODE_SLOW ;
        bSendBuzzerMode = TRUE ;
        bBlueToothSendBuzzerMode = TRUE;
      }
    }
    if(bKeyFlexIn == TRUE)
    {
      FlexMotorSetDisable();
      if(Flex_ControlIn(FLEX_MOTOR_CURRENT_1D5A))
      {
        nBuzzerMode = BUZZER_MODE_FAST ;
        bSendBuzzerMode = TRUE ;
        bBlueToothSendBuzzerMode = TRUE;
        retval = 1; 
      }
      else      
      {
        nBuzzerMode = BUZZER_MODE_SLOW ;
        bSendBuzzerMode = TRUE ;
        bBlueToothSendBuzzerMode = TRUE;
      }
    }
    if((bKeyFlexOut == FALSE) && (bKeyFlexIn == FALSE))//没有压下伸缩按键，
    {
      //  Flex_SetDirection(FLEX_MOTOR_STOP);
        Flex_ControlStop();
      
        retval = 0;
    }
    return retval;
}



//小腿起落电动缸控制程序
void Main_LegPad_Proce(void)
{   
    LegMotor_Proce();//小腿在运行时 ，坐标计数（通过10ms定时器）,
    if(st_Stretch.active) return;  //如果拉退程序有效，退出
    if(bMassagePositionUpdate) return; //在强制设置按摩位置时也会进行小腿处理，故当强制设置按摩位置时不执行此函数内容

   if((bKeyLegPadUp == TRUE)||(bKeyLegPadDown == TRUE))
   {
     Flex_ControlStop();
   }



    if(bLegPadLinkage == FALSE) //小腿单独动，此时不考虑前滑电动缸的位置
    {
        if(bKeyLegPadUp == TRUE)
        {
            FlexMotorSetEnable();
            if(LegMotor_Control(STATE_RUN_LEG_UP) != LEG_RUN)//小腿向上行走，直到碰到行程开关,此时beep快速响
            {
                nBuzzerMode = BUZZER_MODE_FAST ;
                bSendBuzzerMode = TRUE ;
            }
            else  //碰到行程开关
            {
                nBuzzerMode = BUZZER_MODE_SLOW ;
                bSendBuzzerMode = TRUE ;
            }
        }
        
        if(bKeyLegPadDown == TRUE)
        {
            FlexMotorSetEnable();
            switch(LegMotor_Control(STATE_RUN_LEG_DOWN))
            {
            case LEG_RUN:
                {
                    nBuzzerMode = BUZZER_MODE_SLOW ;
                    bSendBuzzerMode = TRUE ;
                }
                break;
            case LEG_STOP_AT_DOWN:
                {
                    nBuzzerMode = BUZZER_MODE_FAST ;
                    bSendBuzzerMode = TRUE ;
                }
                break;
            case  LEG_STOP_AT_GROUND:
                
                //FlexMotor_Control(STATE_RUN_FLEX_RESET,FLEX_SPEED_FAST,FLEX_CURRENT_3A);
                
                break;
            case LEG_STOP_AT_ANGLE:
                break;
            }
        }
        
    }
    else  //靠背和小腿联动，前滑电动缸必须在最前位置
    {
        //if(SlideMotor_Get_Location() == SLIDE_MOTOR_AT_FORWARD)
        //{   
            if(bKeyLegPadUp == TRUE)
            {
                LegMotor_Control(STATE_RUN_LEG_UP);
                FlexMotorSetEnable();
            }
            if(bKeyLegPadDown == TRUE)
            {
                LegMotor_Control(STATE_RUN_LEG_DOWN);
                FlexMotorSetEnable();
            }
        //}
    }
    
    if((bKeyLegPadUp == FALSE) && (bKeyLegPadDown == FALSE))
    {
        LegMotor_Control(STATE_LEG_IDLE) ;
    }
}
#define STRETCH_GO_DOWN 0
#define STRETCH_GO_OUT  1
/*
StretchProgramStruct const stretchProgram_30[] =
{
  {29,2,STRETCH_GO_DOWN},
  {25,2,STRETCH_GO_DOWN},
  {22,2,STRETCH_GO_DOWN},
  {19,2,STRETCH_GO_DOWN},
  {15,2,STRETCH_GO_DOWN},
  {12,2,STRETCH_GO_DOWN},
  {9,2,STRETCH_GO_DOWN},
  {6,2,STRETCH_GO_DOWN},
  {3,3,STRETCH_GO_DOWN},
  
};
StretchProgramStruct const stretchProgram_20[] =
{
  {19,2,STRETCH_GO_DOWN},
  {15,2,STRETCH_GO_DOWN},
  {12,2,STRETCH_GO_DOWN},
  {9,2,STRETCH_GO_DOWN},
  {6,2,STRETCH_GO_DOWN},
  {3,3,STRETCH_GO_DOWN},
};
StretchProgramStruct const stretchProgram_10[] =
{
  {9,2,STRETCH_GO_DOWN},//4次向外拉   9单位为分钟
  {6,2,STRETCH_GO_DOWN},//3次向下拉
  {3,2,STRETCH_GO_DOWN},//3次向下拉 
};


void Valve_StretchControlProce(void)
{
  
  
  bool bStatus;//,bStatus_2;
  int legFlag,BackFlag,FlexFlag;//,SlideFlag;
  static int stretchMode = STRETCH_GO_DOWN;
  if(!st_Stretch.active) //在拉腿空闲时刻才会运行自动程序数组1的程序
  {
    unsigned int RunTime = Data_Get_TimeSecond();//时间倒计时
    unsigned int Minutes,i;
    StretchProgramStruct const *p;
    unsigned int totalTimes;
    
    if(RunTime%60 != 0)  return; //0秒开始拉腿
    
    if(w_PresetTime == RUN_TIME_10) //10minitue    #define RUN_TIME_10    60*10
    {
      p = stretchProgram_10;
      totalTimes = sizeof(stretchProgram_10)/sizeof(StretchProgramStruct);//totalTimes=3
    }
    else if(w_PresetTime == RUN_TIME_30) //30minitue  60*30
    {
      p = stretchProgram_30;
      totalTimes = sizeof(stretchProgram_30)/sizeof(StretchProgramStruct);//totalTimes=10
    }
    else
    {
      p = stretchProgram_20;//20miniute
      totalTimes = sizeof(stretchProgram_20)/sizeof(StretchProgramStruct);//totalTimes=设定多少个拉腿疗程 =7
    } 
    Minutes = RunTime/60; //获取当前分钟数
    
    if(Minutes == 0) 
    {
      st_Stretch1.times = 0;
      st_Stretch.times = 0;
      return; //最后一分钟停止拉腿
    }
    //一个拉腿疗程为一个拉腿结构体
    for(i=0;i<totalTimes;i++)//totalTimes=设定多少个拉腿疗程  totalTimes=3个拉腿疗程=10分钟拉腿  ,对于10分钟拉腿为3个拉腿疗程
    {
      if((Minutes == (p+i)->time)&&((Minutes==6)||(Minutes==9)||(Minutes==29)||(Minutes==25))||(Minutes==19)||(Minutes==15)) // //每个疗程下的拉退程序执行时间   (p+i)->time=单位为分钟
      {//拉退开始运行时间标志位
        st_Stretch.active = TRUE;
        st_Stretch.init = TRUE; 
        stretchMode = (p+i)->mode;//拉退模式为行程控制
        st_Stretch.times = (p+i)->times;//一个疗程的拉退次数 ,即一个拉腿结构体的拉腿次数
        st_Stretch1.times=0;
        
        
        break;
      }
      else if(Minutes ==  (p+i)->time&&((Minutes==3)||(Minutes==22)||(Minutes==12))) // //每个疗程下的拉退程序执行时间   (p+i)->time=单位为分钟
      {//拉退开始运行时间标志位
        st_Stretch.active = TRUE;
        st_Stretch1.init = TRUE; 
        stretchMode = (p+i)->mode;//拉退模式为行程控制
        st_Stretch1.times = (p+i)->times;//一个疗程的拉退次数 ,即一个拉腿结构体的拉腿次数
        st_Stretch.times=0;
        
        
        break;
      }
    }
    if(!st_Stretch.active)  return;
  }
  if(st_Stretch1.init)
  {
    nStretchStep = 0;
    st_Stretch1.step = 0;//拉退程序步骤
    st_Stretch.timer = 0;////拉退程序计时定时器，单位0.1s
    st_Stretch1.init = FALSE;
  }
  if(st_Stretch.init)
  {
    nStretchStep = 0;
    st_Stretch.step = 0;//拉退程序步骤
    st_Stretch.timer = 0;////拉退程序计时定时器，单位0.1s
    st_Stretch.init = FALSE;
    
    
  }
  if(st_Stretch1.times > 0)//一个疗程的拉退次数 一般为3次,一个拉腿结构体的拉腿次数，对于10分钟拉腿，第一个拉腿疗程为4，第二个拉腿疗程为3
  {
    switch(st_Stretch1.step)
    {
    case 0:
      Valve_SetStretchChargeATOUT(1);//足部和坐垫先充气	
      
      nStretchStep = 0;
      AutoDirector = AutoFunctionStretch[nStretchStep] ;////按摩椅到达拉腿位置，机芯到达最低处 ,行走电机到达最低点
      refreshAutoDirector();
      st_Stretch1.step++;
      break;
      
    case 1:
      
      FlexFlag = Flex_ControlOut(FLEX_MOTOR_CURRENT_1D5A);//小腿缩  FlexFlag = Flex_ControlOut(FLEX_MOTOR_CURRENT_1D5A);
      if(FlexFlag || st_Stretch.timer > 50)
      {
        bStatus = 1;
        st_Stretch1.step++;
        st_Stretch.timer = 0; 
        Flex_ControlStop();
      }
      else
      {
        bStatus = 0;
        Valve_SetStretchChargeATOUT(1);//足部和坐垫先充气
        if(bRollerEnable)
        {
          RollerMotor_Control(ROLLER_SPEED_SLOW,0);
        }
      }	
      
      
      
      
      
      
      break;
    case 2: 
      legFlag = LegMotor_Control(STATE_RUN_LEG_UP);
      BackFlag = BackMotor_Control(STATE_RUN_BACK_UP);
      if(legFlag && BackFlag )
      {
        FlexMotorSetEnable();
        st_Stretch1.step++;
        st_Stretch.timer = 0; 
      }
      break;
    case 3:
      if(stretchMode == STRETCH_GO_OUT)//stretchMode 
      {
        Valve_SetStretchChargeOut(0); 
      }
      else
      {
        Valve_SetStretchCharge(1); 
      }
      
      //    if(st_Stretch.timer >= C_STRETCH_CHARGE_TIME)//100*60=6sec ,程序在 case 5之间循环，直到充气时间结束
      
      if(stretchMode == STRETCH_GO_OUT)
      {
        if(st_Stretch.timer >= C_STRETCH_CHARGE_TIME_OUT)//100*60=6sec ,程序在 case 5之间循环，直到充气时间结束
        {  //判断是否已达充气时间
          st_Stretch1.step++;
          st_Stretch.timer = 0;
        }     
      }
      else
      {
        if(st_Stretch.timer >= C_STRETCH_CHARGE_TIME)//100*60=6sec ,程序在 case 5之间循环，直到充气时间结束
        {  //判断是否已达充气时间
          st_Stretch1.step++;
          st_Stretch.timer = 0;
        }           
        
      }
      
      break;
    case 4:
      nStretchStep = 1;
      AutoDirector = AutoFunctionStretch[nStretchStep] ;////按摩椅到达拉腿位置，机芯到达最低处 ,行走电机到达最低点
      refreshAutoDirector();
      
      legFlag = LegMotor_Control(STATE_RUN_LEG_DOWN);
      BackFlag = BackMotor_Control(STATE_RUN_BACK_DOWN);
      FlexFlag = Flex_ControlIn(FLEX_MOTOR_CURRENT_1D5A);//按摩椅小腿往外拉  5cm,有可能碰到行程开关
      
      if(legFlag)// && BackFlag )
      {
        st_Stretch1.step++;
        FlexMotorSetDisable();
        st_Stretch.timer = 0;BackMotor_Control(STATE_BACK_IDLE );
      }
      
      
      
      break;
    case 5:BackMotor_Control(STATE_BACK_IDLE );
      if(st_Stretch.timer >50)
      {
        st_Stretch1.step++;
        
      }
    case 6:
      st_Stretch1.step = 0;
      st_Stretch.timer = 0;
      st_Stretch1.times--;//一个,一个拉腿结构体的拉腿次数，对于10分钟拉腿，第一个拉腿疗程为4次，从第9分钟开始
      //  Valve_SetStretchUp();  //气阀关闭，放气，（除腰部气阀保持充气外）
      Valve_SetStretchChargeATOUT(1);//足部和坐垫先充气
      
      nStretchStep = 0;
      if(st_Stretch1.times == 0)
      {
        // nZLB_RunState = 1;            //拉退动作完成强制回到第一个零重力点
        nTargetMassagePosition =MASSAGE_OPTIMAL_POSITION;// MASSAGE_OPTIMAL2_POSITION; 
        bMassagePositionUpdate = TRUE;
        //bZLBMotorRunFlag = TRUE;
        st_Stretch1.bBackLegFlag = FALSE;
        st_Stretch.timer = 0;
        st_Stretch.active = FALSE;
        bBackAutoModeInit = true;  //为了避免机芯出现差错，机芯按摩从头开始
      }
      
      break;
    default:break;
    
    }
    
    
  }
  else if(st_Stretch.times > 0)//一个疗程的拉退次数 一般为3次,一个拉腿结构体的拉腿次数，对于10分钟拉腿，第一个拉腿疗程为4，第二个拉腿疗程为3
  {
    switch(st_Stretch.step)// //拉退程序步骤    //在拉腿空闲时刻才会运行自动程序数组1的程序
    {
    case  0:   //机芯动作初始化
      //if(!limt_backmoto)
      //	{
      nStretchStep = 0;
      AutoDirector = AutoFunctionStretch[nStretchStep] ;////按摩椅到达拉腿位置，机芯到达最低处 ,行走电机到达最低点
      refreshAutoDirector();
      //	}
      st_Stretch.step++;
      st_Stretch.timer = 0; 
      break;
    case  1:  //按摩椅到最大位置
      //Valve_SetStretchUp();  //气阀配合拉腿
      //    LegKnead_Control(LEG_KNEAD_SPEED_STOP,LEG_KNEAD_TO_IN);//8600S小腿板控制，8305T 没有使用小腿揉捏电机，不起作用
      //     SlideFlag = SlideMotorControl(STATE_RUN_SLIDE_FORWARD); //
      Valve_SetStretchCharge_ARM_SHOULD(0);
      
      if(Input_GetBackDownSwitch() == REACH_BACK_LIMIT)
        
      {
        BackFlag = true;BackMotor_Control(STATE_BACK_IDLE);
      }
      else
      {
        BackFlag = BackMotor_Control(STATE_RUN_BACK_DOWN);
      }
      
      
      //   FlexFlag = Flex_ControlIn(FLEX_MOTOR_CURRENT_15A);//小腿缩
      //靠背最下面，小腿最上面，伸缩电机缩到最里面
      //if((bWalkMotorInProcess == FALSE) &&
        if( (bKneadMotorInProcess == FALSE) &&
           (bKnockMotorInProcess == FALSE)&&
             (b3D_MotorInProcess == FALSE))//&&
               //BackFlag)
      {
        __no_operation();
        __no_operation();
        __no_operation();
        __no_operation(); 
        __no_operation();
        __no_operation();                  
        st_Stretch.step ++;   //小腿和靠背电动缸到位，开始运行拉腿程序
        st_Stretch.timer = 0; 
      }
      
      if(st_Stretch.timer >  250)//40  000
      {
        st_Stretch.timer = 0; 
        st_Stretch.step++;
        BackMotor_Control(STATE_BACK_IDLE);
      }
      
      
      break;
    case 2: 
      BackMotor_Control(STATE_BACK_IDLE);
      __no_operation();
      __no_operation();
      __no_operation();
      __no_operation(); 
      __no_operation();
      __no_operation();     
      legFlag = LegMotor_Control(STATE_RUN_LEG_UP);
      Valve_SetStretchCharge_FOOT_THIGH(0);
      if((legFlag)&&(st_Stretch.timer >  130))
      {
        st_Stretch.timer = 0; 
        st_Stretch.step++;
        
      }
      if(stretchMode != STRETCH_GO_OUT)
      {
        //       FlexMotorSetEnable(); //该标志位打开自动找脚 程序
        FlexMotorSetDisable();
        break;
      }
      break;
      
    case 3:
      
      FlexFlag = Flex_ControlOut(FLEX_MOTOR_CURRENT_5A);//按摩椅小腿往外拉  5cm,有可能碰到行程开关
      nStretchStep = 1;
      AutoDirector = AutoFunctionStretch[nStretchStep] ;///停在坐标0位置
      refreshAutoDirector();
      
      if(FlexFlag )//|| st_Stretch.timer > 50)
      {
        if(st_Stretch.timer > 50)
        {
        bStatus = 1;
        st_Stretch.step++;
        st_Stretch.timer = 0; 
        Flex_ControlStop();
        }
      }
      else
      {
        bStatus = 0;
        //Valve_SetStretchChargeATOUT(1);//足部和坐垫先充气
        if(bRollerEnable)
        {
          RollerMotor_Control(ROLLER_SPEED_SLOW,0);
        }
      }
      //Valve_SetStretchChargeATOUT(1);//足部和坐垫先充气
      //Valve_SetStretchCharge_FOOT_THIGH(0);
      Valve_SetStretchCharge_FOOT_THIGH_SHOULD(0);
      break;
      
      
    case 4:
      
      FlexMotorSetEnable();
      Valve_SetStretchChargeATOUTFootHeelOFF();//释放足部气囊
      
      st_Stretch.step++;
      break;
      
      
      
    case 5:  //调整电动小腿位置
      BackMotor_Control(STATE_BACK_IDLE);
      if(stretchMode == STRETCH_GO_OUT)
      {
        st_Stretch.step++;
        break;
      }
      if(FlexMotorGetEnable() == false)
      {
        
        st_Stretch.step++;
        st_Stretch.timer = 0;  //足左，足右充气打开，其他气囊关闭  充气开始定时器清 0
        
      }
      
      break;
    case  6: 
      
      if(stretchMode == STRETCH_GO_OUT)
      {
        Valve_SetStretchChargeOut(0); //
      }
      else
      {
        Valve_SetStretchCharge(0); 
      }
      if(bRollerEnable)
      {
        RollerMotor_Control(ROLLER_SPEED_SLOW,0);
      }
      //   if(st_Stretch.timer >70)
      //  	{
      st_Stretch.step++;
      st_Stretch.timer = 0;  //足左，足右充气打开，其他气囊关闭  充气开始定时器清 0
      //  	}
      break;
    case 7:
      Valve_LeftHandSetStretchChargeOut(0);
      if(st_Stretch.timer >70)
      {
        st_Stretch.step++;
        st_Stretch.timer=0;
        
      }
      break;
      
    case  8: 
      BackMotor_Control(STATE_BACK_IDLE);
      nStretchStep = 0;
      AutoDirector = AutoFunctionStretch[nStretchStep] ;///停在坐标0位置
      refreshAutoDirector();
      Valve_RightHandSetStretchChargeOut(0);
      if(st_Stretch.timer >70)
      {
        st_Stretch.step++;
        st_Stretch.timer=0;
        
      }
      break;
    case 9:	*/	
     /* if(stretchMode == STRETCH_GO_OUT)//stretchMode 
      {
        Valve_SetStretchChargeOut(0); 
      }
      else
      {
        Valve_SetStretchCharge(0); 
      }*/

/*
      Valve_SetStretchCharge_FOOT_THIGH_LEG_SHOULD(0);
      //    if(st_Stretch.timer >= C_STRETCH_CHARGE_TIME)//100*60=6sec ,程序在 case 5之间循环，直到充气时间结束
      
      if(stretchMode == STRETCH_GO_OUT)
      {
        if(st_Stretch.timer >= C_STRETCH_CHARGE_TIME_OUT)//100*60=6sec ,程序在 case 5之间循环，直到充气时间结束
        {  //判断是否已达充气时间
          st_Stretch.step++;
          st_Stretch.timer = 0;
        }     
      }
      else
      {
        if(st_Stretch.timer >= C_STRETCH_CHARGE_TIME)//100*60=6sec ,程序在 case 5之间循环，直到充气时间结束
        {  //判断是否已达充气时间
          st_Stretch.step++;
          st_Stretch.timer = 0;
        }           
        
      }
      
      break;
      
    case 10: 
      
      st_Stretch.step++;
      break;
    case 11:          
      nStretchStep = 1;
      AutoDirector = AutoFunctionStretch[nStretchStep] ;////按摩椅到达拉腿位置，机芯到达最低处 ,行走电机到达最低点
      refreshAutoDirector();
      if(stretchMode == STRETCH_GO_DOWN)   //小腿向下运行到最低点 
      { 
        Valve_SetStretchCharge(0); //关掉肩部气囊
        //     LegKnead_Control(LEG_KNEAD_SPEED_MID,LEG_KNEAD_TO_IN);
        if(LEG_STOP_AT_DOWN == LegMotor_Control(STATE_RUN_LEG_DOWN))
        {
          bStatus = 1;
          nStretchStep = 2;
          AutoDirector = AutoFunctionStretch[nStretchStep] ;////按摩椅到达拉腿位置，机芯到达最低处 ,行走电机到达最低点
          //limt_backmoto=1;
          
          refreshAutoDirector();
        }
        else
        {
          bStatus = 0;
        }
      }  
      else  //STRETCH_GO_OUT=伸缩电机向外伸到最大位置
      {
        
        
        Valve_SetStretchChargeOut(0);          
        FlexFlag = Flex_ControlOut(FLEX_MOTOR_CURRENT_2A);//if(FlexFlag || st_Stretch.timer > 100)///拉腿时间计数  100ms  counter          
        if((FlexFlag) || (st_Stretch.timer >100))
        {                   
          // FlexMotorSetAdjStep(2);
          // st_Stretch.step=11;
          bStatus = 1; 
        }
        else
        {
          bStatus = 0;
        }
      }
      
      if(bStatus)
      {  
        if(bRollerEnable)
        {
          RollerMotor_Control(ROLLER_SPEED_SLOW,1);
        }
        Flex_SetDirection(FLEX_MOTOR_STOP);
        st_Stretch.step++;
        st_Stretch.timer = 0;
        //  nStretchStep = 3;
        //  AutoDirector = AutoFunctionStretch[nStretchStep] ;
        // refreshAutoDirector();
      }
      break;
      
      
      
      //气囊气充满以后一直为保持状态，直到关闭气阀才会放气
    case 12:    //加压时间  ,小腿伸到最大位置或小腿向下到最大位置，开始气囊加压
      Valve_SetStretchHold();//足左，足右，脚后跟，小腿上侧，小腿下侧，小腿上底，小腿下底，右腰，左腰充气，加压
      st_Stretch.step++;
      st_Stretch.timer = 0;
      break;
    case 13:   //加压时间长，力度太大人不舒服
      //   if(st_Stretch.timer >= (Valve_GetAirBagStrength()*10))//加压时间根据气囊力度来定，2*10=20 ，100ms*20=2sec,气囊力度通过外部按键获得，共有5种气囊力度
      //    if(st_Stretch.timer >= (Valve_GetAirBagStrength()*6))//加压时间根据气囊力度来定，2*10=20 ，100ms*20=2sec,气囊力度通过外部按键获得，共有5种气囊力度
      if(st_Stretch.timer >= (Valve_GetAirBagStrength()*20))//加压时间根据气囊力度来定，2*10=20 ，100ms*20=2sec,气囊力度通过外部按键获得，共有5种气囊力度
      {  //判断是否已达加压时间
        st_Stretch.step++;
        st_Stretch.timer = 0;
        RollerMotor_Control(ROLLER_SPEED_STOP,0);
        //       LegKnead_Control(LEG_KNEAD_SPEED_STOP,LEG_KNEAD_TO_IN);
      }
      break;
    case 14:
      st_Stretch.step = 0;
      st_Stretch.timer = 0;
      st_Stretch.times--;//一个,一个拉腿结构体的拉腿次数，对于10分钟拉腿，第一个拉腿疗程为4次，从第9分钟开始
      Valve_SetStretchUp();  //气阀关闭，放气，（除腰部气阀保持充气外）
      nStretchStep = 0;
      if(st_Stretch.times == 0)
      {
        
        // nZLB_RunState = 1;            //拉退动作完成强制回到第一个零重力点
        nTargetMassagePosition =MASSAGE_OPTIMAL_POSITION;// MASSAGE_OPTIMAL2_POSITION; 
        bMassagePositionUpdate = TRUE;
        //bZLBMotorRunFlag = TRUE;
        st_Stretch.bBackLegFlag = FALSE;
        st_Stretch.timer = 0;
        st_Stretch.active = FALSE;
        bBackAutoModeInit = true;  //为了避免机芯出现差错，机芯按摩从头开始
        //limt_backmoto=0;
        
      }
      break;
    default:
      break;
    }
  }  
}

*/



StretchProgramStruct const stretchProgram_30[] =
{
  {28,3,STRETCH_GO_DOWN},//{28,3,STRETCH_GO_DOWN},
  {23,3,STRETCH_GO_DOWN},
  {18,3,STRETCH_GO_DOWN},
  {13,3,STRETCH_GO_DOWN},
  {8,3,STRETCH_GO_DOWN},
  {5,3,STRETCH_GO_DOWN},
};
StretchProgramStruct const stretchProgram_20[] =
{
  {18,3,STRETCH_GO_DOWN},//{18,3,STRETCH_GO_DOWN},
  {13,3,STRETCH_GO_DOWN},
  {8,3,STRETCH_GO_DOWN},
  {5,3,STRETCH_GO_DOWN},
};
StretchProgramStruct const stretchProgram_10[] =
{
  {8,4,STRETCH_GO_DOWN},//4次向外拉   9单位为分钟
  {4,3,STRETCH_GO_DOWN},//3次向外拉    
};

/*

1档：100%
2档：100%
3档：100%
4档；100%
5档：100%



*/
  unsigned int p_LegLocation;
unsigned int wghp_BackLocation;
void Valve_StretchControlProce(void)
{
  bool bStatus,bBACKStatus;
  int legFlag,BackFlag,FlexFlag;//,SlideFlag;
  unsigned int p_BackLocation;

  float s_AirTimeadj;
  unsigned char s_AirBagStrength = Valve_GetAirBagStrength();
  //static int stretchMode = STRETCH_GO_DOWN;
        p_LegLocation = LegMotor_Get_Position();
        wghp_BackLocation=BackMotor_Get_Location();
  if(s_AirBagStrength == 1)
  {
      s_AirTimeadj = 0.5;
  }
  else
  {
      s_AirTimeadj = 1;
  }
  
  
  if(!st_Stretch.active) 
   {
    unsigned int RunTime = Data_Get_TimeSecond();
    unsigned int Minutes,i;
    StretchProgramStruct const *p;
    unsigned int totalTimes;
    
    if(RunTime%60 != 0)  return; //0秒开始拉腿
        
    if(w_PresetTime == RUN_TIME_10) 
    {
      p = stretchProgram_10;
      totalTimes = sizeof(stretchProgram_10)/sizeof(StretchProgramStruct);
    }
    else if(w_PresetTime == RUN_TIME_30) 
    {
      p = stretchProgram_30;
      totalTimes = sizeof(stretchProgram_30)/sizeof(StretchProgramStruct);
    }
    else
    {
      p = stretchProgram_20;
      totalTimes = sizeof(stretchProgram_20)/sizeof(StretchProgramStruct);
    } 
    Minutes = RunTime/60; //获取当前分钟数
   
    if(Minutes == 0) 
    {
      st_Stretch.times = 0;
      return; //最后一分钟停止拉腿
    }
    
    for(i=0;i<totalTimes;i++)
    {
       
      if((bShoulderOK == 1)  &&(st_Stretch.active == FALSE) )
      {
          /*
        if(nBackSubRunMode == BACK_SUB_MODE_6MIN_DEMO) // //舒适按摩带拉伸功能
        {
            if(
               (Minutes == 30 )||
                   (Minutes == 29 )||
                       (Minutes == 24 )||
                           (Minutes == 23 )||  
                               (Minutes == 20 )||
                                   (Minutes == 19 )||
                                       (Minutes == 14 )||
                                           (Minutes == 13 )||  
                                               (Minutes == 10 )||
                                                   (Minutes == 9 )||
                                                       (Minutes == 4 )||
                                                           (Minutes == 3 )               
                                                               )
          {
            st_Stretch.active = TRUE;
            SetStretchingEnable(1);
            st_Stretch.init = TRUE; 
            st_Stretch.times = 3;
            bZeroflash = FALSE;
            break;
          }
          
          
        }
          */
          
         if(w_PresetTime == RUN_TIME_30) 
        {
          if((Minutes == 30 )||
             (Minutes == 29 ))
          {
            st_Stretch.active = TRUE;
            SetStretchingEnable(1);
            st_Stretch.init = TRUE; 
            st_Stretch.times = 3;
            bZeroflash = FALSE;
            break;
          }
        }
        else if(w_PresetTime == RUN_TIME_20) 
        {
          if((Minutes == 20 )||
             (Minutes == 19 ))
          {
            st_Stretch.active = TRUE;
            SetStretchingEnable(1);
            st_Stretch.init = TRUE; 
            st_Stretch.times = 3;
            bZeroflash = FALSE;
            break;
          }
        }
        else
        {
          if((Minutes == 10 )||
             (Minutes == 9 )||
             (Minutes == 4 )||
             (Minutes == 3 )               
               )
          {
            st_Stretch.active = TRUE;
            SetStretchingEnable(1);
            st_Stretch.init = TRUE; 
            st_Stretch.times = 4;
            bZeroflash = FALSE;
            break;
          }
        } 
        
        
        
        
        
      }
      
      if(Minutes ==  (p+i)->time) 
      {
       st_Stretch.active = TRUE;SetStretchingEnable(1);
       st_Stretch.init = TRUE; 
       //stretchMode = (p+i)->mode;
       st_Stretch.times = (p+i)->times;
       bZeroflash = FALSE;
       break;
      }
    }
    if(!st_Stretch.active)  return;
    
  }
  if(st_Stretch.init)
  {
    nStretchStep = 0;
    st_Stretch.step = 0;
    st_Stretch.timer = 0;
    st_Stretch.init = FALSE;
  }
  


  
  if(Input_GetBackDownSwitch() == REACH_BACK_LIMIT) BackMotor_Set_Location(BACK_MOTOR_MAX_LOCATION);
  if(Input_GetBackUpSwitch() == REACH_BACK_LIMIT) BackMotor_Set_Location(0);
   
  wghp_BackLocation = BackMotor_Get_Location();
  if(st_Stretch.times > 0)
  {
    WorkStep =0;
    switch(st_Stretch.step)
    {
    case  0:   //机芯动作初始化   机芯到底
      nStretchStep = 0;
      AutoDirector = AutoFunctionStretch[nStretchStep] ;
      refreshAutoDirector();
      st_Stretch.step++;
      legFlag = FALSE;
      BackFlag = FALSE;
      st_Stretch.timer = 0; 
      break;
    case  1:  //按摩椅角度：小腿在上限位，靠背指定位置
      p_BackLocation = BackMotor_Get_Location();
      legFlag = LegMotor_Control(STATE_RUN_LEG_UP);
      if(p_BackLocation > (MASSAGE_BACK_STRCHUP_LOCATION + 20))
      {
        BackMotor_Stretch_Control(STATE_RUN_BACK_UP) ;//BackMotor_Stretch_Control
        BackFlag = FALSE;
      }
      else if(p_BackLocation < (MASSAGE_BACK_STRCHUP_LOCATION - 20))
      {
        BackMotor_Stretch_Control(STATE_RUN_BACK_DOWN) ;
        BackFlag = FALSE;
      }
      
      
      else
      {
        BackFlag = BackMotor_Stretch_Control(STATE_BACK_IDLE) ;
      }
      
      if((bWalkMotorInProcess == FALSE) &&
         (bKneadMotorInProcess == FALSE) &&
           (bKnockMotorInProcess == FALSE)&&
             (b3D_MotorInProcess == FALSE)&&
             legFlag &&
               BackFlag)
      {
        st_Stretch.step ++;
        st_Stretch.timer = 0; 
        //Valve_SetStretchCharge_FOOT(1);
        FlexMotorSetDisable();//RESET
        //FlexMotorSetEnable();//20170214
      }//
      else
      {
        
        //Valve_SetStretchCharge_ARM(0);//Valve_CloseAll(); 
      }
      
      Valve_SetStretchCharge_ARM(0,s_AirBagStrength);
      
      //}
      
     /* 
      if(st_Stretch.timer > 250) 
      {
        st_Stretch.step ++;
        st_Stretch.timer = 0; 
        Valve_SetStretchCharge_FOOT(1);
        FlexMotorSetDisable();//RESET
        FlexMotorSetEnable();
        LegMotor_Control(STATE_LEG_IDLE);
        BackMotor_Stretch_Control(STATE_BACK_IDLE) ;
      }*/
      
      if(st_Stretch.timer > 150) //20170515
      {  
        st_Stretch.step++;
        st_Stretch.timer = 0;
        FlexMotorSetDisable();
      }
      
      
      
      break;
    case 2: // 等待时，机芯定点揉捏  3D开始使力
      BackMotor_Stretch_Control(STATE_BACK_IDLE) ;
      nStretchStep = 1;
      AutoDirector = AutoFunctionStretch[nStretchStep] ;
      refreshAutoDirector();
                   //st_Stretch.step++;
      //Valve_SetStretchCharge_FOOT(1);
     // if(st_Stretch.timer > 40) 
      {  
        st_Stretch.step++;
        st_Stretch.timer = 0; 
      }
        //FlexMotorSetEnable();//------------------------------找脚
        bHaveMan = FALSE;
        break;
       
    case 3: 
      Valve_SetStretchCharge_ARM(0,s_AirBagStrength);
      FlexFlag = Flex_ControlIn(FLEX_MOTOR_CURRENT_2A);//1D5A);//20170515
      if(FlexFlag )
      {
        st_Stretch.step++;
        st_Stretch.timer = 0;
        
      }
      
      //st_Stretch.step++;st_Stretch.timer = 0;
      if(nFlexStatus&0x04) //脚底碰到脚底开关信号
      {
        bHaveMan = TRUE;
      }
      if(st_Stretch.timer > 150) //20170515
      {  
        st_Stretch.step++;
        st_Stretch.timer = 0;
      }
      break;
    case 4 :
      Valve_SetStretchCharge_FOOT_THIGH(1); 
      if(FlexMotorGetEnable() == false)
      {
        if(st_Stretch.timer > (unsigned char)(s_AirTimeadj*120)) //----------------------找脚后，充气时间准备前拉
        { 
          st_Stretch.step++;
          st_Stretch.timer = 0;
        }
      }
      else
      {
        st_Stretch.timer = 0;
      }

      if(st_Stretch.timer > 150) //20170515
      {  
        st_Stretch.step++;
        st_Stretch.timer = 0;
      }
     nStretchFlextimer = 0; 

      break;
      
    case 5:    
      //离开脚心，停止
      FlexFlag = Flex_ControlOut(FLEX_MOTOR_CURRENT_1D5A);//Flex_ControlOut(FLEX_MOTOR_CURRENT_5A);
      if(FlexFlag )//&&(bWalkMotorInProcess == FALSE) )//|| st_Stretch.timer > 100)
      {
        Flex_SetDirection(FLEX_MOTOR_STOP);
        //FlexMotor_Control(STATE_FLEX_IDLE, FLEX_SPEED_FAST, FLEX_CURRENT_3A);
        st_Stretch.step++;
        st_Stretch.timer = 0;
      }
      
      if( (st_Stretch.timer>35)&&(bHaveMan == FALSE) )//---------------------------------保护停止
      {
        Flex_SetDirection(FLEX_MOTOR_STOP);
        st_Stretch.step++;
        st_Stretch.timer = 0;
      } 
      //if((nStretchFlextimer > 10)&&( (nFlexStatus&0x04) == 0)&&(bHaveMan))//---------------------------------保护停止(nFlexStatus&0x04)
      if( ((nFlexStatus&0x04) == 0)&&(bHaveMan))//---------------------------------保护停止(nFlexStatus&0x04)
      {
        Flex_SetDirection(FLEX_MOTOR_STOP);
        st_Stretch.step++;
        st_Stretch.timer = 0;
      } 
      //if(nFlexStatus&0x04)
      //{
      //  nStretchFlextimer = 0;
      //}    
      
      if(bRollerEnable)
      {
        if(st_Stretch.times % 2== 0)
        {
          RollerMotor_Control(ROLLER_SPEED_SLOW,0);
        }
        else
        {
          RollerMotor_Control(ROLLER_SPEED_SLOW,1);
        }
      }
      
      if(st_Stretch.timer > 150) //20170515
      {  
        st_Stretch.step++;
        st_Stretch.timer = 0;
      }
      
      break;
      
 case 6: //add wgh 20170227

      if(bHaveMan == TRUE) //脚底碰到脚底开关信号
      {
        FlexFlag = Flex_ControlIn(FLEX_MOTOR_CURRENT_2A);//1D5A);//20170515
        if(st_Stretch.timer>20) //--保护停止
        {
          Flex_SetDirection(FLEX_MOTOR_STOP);
          st_Stretch.step++;
          st_Stretch.timer = 0;
        } 
      }
      else
      {
        st_Stretch.step++;
        st_Stretch.timer = 0;
      }
      
      if(st_Stretch.timer > 150) //20170515
      {  
        st_Stretch.step++;
        st_Stretch.timer = 0;
      }
      
      
      break;    

      
      
    case  7: 
      Valve_SetStretchCharge_FOOT_THIGH_LEG(0);//全充
       if(st_Stretch.timer > (unsigned char)(s_AirTimeadj*70)) //---------------------------------在前伸离开脚心处保压
        {
          st_Stretch.step++;
          st_Stretch.timer = 0;
          
        }
      //st_Stretch.step++;
      break;
   
    case 8:
     // if(bWalkMotorInProcess == FALSE)
      
      //机芯揉
      {
        nStretchStep = 1;
        AutoDirector = AutoFunctionStretch[nStretchStep] ;
        refreshAutoDirector();
        st_Stretch.step++;st_Stretch.timer = 0;
      }
      break;
    case  9: 
        Valve_SetStretchCharge_FOOT_THIGH_LEG_SHOULD(0);//全充

      if(st_Stretch.timer >= 40)//-------------------------充压时间
      {  //判断是否已达充气时间
        st_Stretch.step++;
        st_Stretch.timer = 0;
      }
      break;
     

    case 10: 
      //bRoll_InStretch	= FALSE;
      nStretchStep = 2;
      AutoDirector = AutoFunctionStretch[nStretchStep] ;
      refreshAutoDirector();
      
      //判断是否已达充气时间
      st_Stretch.step++;
      st_Stretch.timer = 0;
      
      break;
    case 11:  //拉倒最低端        

      Valve_SetStretchCharge_FOOT_THIGH_LEG_SHOULD(0);//--------下拉充气
      //legFlag = LegMotor_Control(STATE_RUN_LEG_DOWN);
      //bBACKStatus = BackMotor_Stretch_Control(STATE_RUN_BACK_DOWN) ;
       p_BackLocation = BackMotor_Get_Location();  
        p_LegLocation = LegMotor_Get_Position(); 
        //350-:450-500--564
        if(nStretchVigor==1)
       //if(st_Stretch.times == 3)
       {
           
           
            if(p_BackLocation > (500 + 20))
            {
                BackMotor_Control(STATE_RUN_BACK_UP) ;
                bBACKStatus = FALSE;
            }
            else if(p_BackLocation < (500 - 20))
            {
                BackMotor_Control(STATE_RUN_BACK_DOWN) ;
                bBACKStatus = FALSE;
            }
            else
            {
                BackMotor_Control(STATE_BACK_IDLE) ;
                bBACKStatus = TRUE;
            } 
           
           //p_LegLocation = LegMotor_Get_Position(); 
           if(p_LegLocation > (900 + POSITION_CTRL_OFFSET))
           {
               LegMotor_Control(STATE_RUN_LEG_DOWN) ;
               legFlag = FALSE;
           }
           else if(p_LegLocation < (900 - POSITION_CTRL_OFFSET))
           {
               LegMotor_Control(STATE_RUN_LEG_UP);//STATE_RUN_BACK_UP) ;
               legFlag = FALSE;
           }
           else
           {
               LegMotor_Control(STATE_LEG_IDLE);//STATE_BACK_IDLE) ;
               legFlag = TRUE;
           } 
           
           
           
           
       }
       if(nStretchVigor==2)
       //if(st_Stretch.times == 2)
       {
           
            if(p_BackLocation > (580 + 20))
            {
                BackMotor_Control(STATE_RUN_BACK_UP) ;
                bBACKStatus = FALSE;
            }
            else if(p_BackLocation < (580 - 20))
            {
                BackMotor_Control(STATE_RUN_BACK_DOWN) ;
                bBACKStatus = FALSE;
            }
            else
            {
                BackMotor_Control(STATE_BACK_IDLE) ;BackMotor_Set_Pwm_Data(0);
                bBACKStatus = TRUE;
            } 
           
           
          
           if(p_LegLocation > (600 + POSITION_CTRL_OFFSET))
           {
               LegMotor_Control(STATE_RUN_LEG_DOWN) ;
               legFlag = FALSE;
           }
           else if(p_LegLocation < (600 - POSITION_CTRL_OFFSET))
           {
               LegMotor_Control(STATE_RUN_LEG_UP);//STATE_RUN_BACK_UP) ;
               legFlag = FALSE;
           }
           else
           {
               LegMotor_Control(STATE_LEG_IDLE);//STATE_BACK_IDLE) ;
               legFlag = TRUE;
           }
       }
       if(nStretchVigor==3)
       //if(st_Stretch.times == 1)
       {
           legFlag = LegMotor_Control(STATE_RUN_LEG_DOWN);
           bBACKStatus = BackMotor_Stretch_Control(STATE_RUN_BACK_DOWN) ;  
       }     
      
      
      
      
      
      
      
      
      
      
      
      
      
      /////////////////////
      if((legFlag==TRUE)&&(bBACKStatus==TRUE))
      {  
        if(bRollerEnable)
        {
          RollerMotor_Control(ROLLER_SPEED_SLOW,0);
        }
        //Flex_SetDirection(FLEX_MOTOR_STOP);
        st_Stretch.step++;
        st_Stretch.timer = 0;
        //bRoll_InStretch	= TRUE;
        nStretchStep = 3;
        AutoDirector = AutoFunctionStretch[nStretchStep] ;
        refreshAutoDirector();       
        
      }
      if(st_Stretch.timer > 180) //20170515//20180402
      {  
        st_Stretch.step++;
        st_Stretch.timer = 0;
      }
      
      break;
    case 12:    //加压时间
        
        LegMotor_Control(STATE_LEG_IDLE);//STATE_BACK_IDLE) ;
        BackMotor_Control(STATE_BACK_IDLE) ;
        BackMotor_Set_Pwm_Data(0);
     // Valve_SetStretchHold();
       Valve_SetStretchChargeSTEEL(0); 
      //st_Stretch.step++;
      //st_Stretch.timer = 0;
      if(st_Stretch.timer > 100) //20170515 20180402
      {  
        st_Stretch.step++;
        st_Stretch.timer = 0;
      }
      
      break;
    case 13:
      Valve_SetStretchChargeSTEEL(0);
      if(st_Stretch.timer >= (Valve_GetAirBagStrength()*(unsigned char)(s_AirTimeadj*20)))//----------在下部保压时间
      {  //判断是否已达加压时间
        st_Stretch.step++;
        st_Stretch.timer = 0;
        RollerMotor_Control(ROLLER_SPEED_STOP,0);
        //bRoll_InStretch	= TRUE;
      }
      break;
    case 14:
      //bRoll_InStretch	= FALSE;
      st_Stretch.step = 0;
      st_Stretch.timer = 0;
      st_Stretch.times--;
      Valve_SetStretchUp();  
      nStretchStep = 0;
      if(st_Stretch.times == 0)
      {
          // nZLB_RunState = 1;            //拉退动作完成强制回到第一个零重力点
          nTargetMassagePosition = MASSAGE_OPTIMAL2_POSITION;//MASSAGE_RESET_POSITION;// MASSAGE_RESET_POSITION;//
          bMassagePositionUpdate = TRUE;
          //bZLBMotorRunFlag = TRUE;
          st_Stretch.bBackLegFlag = FALSE;
          st_Stretch.timer = 0;
          st_Stretch.active = FALSE;SetStretchingEnable(0);
          //bBackAutoModeInit = true;  //为了避免机芯出现差错，机芯按摩从头开始
          
          
          if((nBackMainRunMode == BACK_MAIN_MODE_AUTO) && (nBackSubRunMode == BACK_SUB_MODE_6MIN_DEMO)     )
          {
              nCurActionStep = nPreActionStep;
              bGetNextActionStep = TRUE ;///所有电机动作完成后才会进行自动程序下一步数组的动作，
              b6MinDemoStretch = FALSE;
              bKeyWaistHeat = FALSE ;
          }
          else
          {
             bBackAutoModeInit = true;  //为了避免机芯出现差错，机芯按摩从头开始  
          }
        
      }
      break;
    default:
      break;
    }
  }   
  
}


void Main_Valve_Proce(void)
{
    if(nBackMainRunMode == BACK_MAIN_MODE_AUTO)
    {
        Valve_SetBackMode(1);
    }
    else
    {
        Valve_SetBackMode(0);
    }

    unsigned char by_EngineeringAirBag = ReadEEByte(AIRBAG_STRETCH_ADDRESS + USER_DATA_BASE);//READ 气囊力度

    if(nBackMainRunMode == BACK_MAIN_MODE_AUTO)
    {
        if( nBackSubRunMode == BACK_SUB_MODE_AUTO_2)//休憩促眠模式是一个轻松按摩的时刻，关掉肩部和手臂气囊
               
        {//该模式是轻松按摩，颈部气囊关闭，同时要为该模式重新定义一个自动气囊程序，
               
            //    Valve_Control(VALVE_DISABLE,&st_AirBagArmNeck,by_EngineeringAirBag);
               
        }   
      
      
        if((Data_Get_ProgramExecTime() > VALVE_START_TIME) || !bMassagePositionUpdate )  //判断气阀开始时间是否到达
        {
            goto VALVE_START;  //如果按摩椅已经到达最佳位置
        }
        Valve_Control(VALVE_DISABLE,&st_AirBagAuto,by_EngineeringAirBag);
        Valve_FootRollerProce(0,0,&st_AirBagAuto);      
        Valve_AirPumpACPowerOff();

        return;
    }

VALVE_START:

    if(nBackMainRunMode == BACK_MAIN_MODE_AUTO)
    {
        Valve_SetEnableSholder(0);
    }
    else
    {
        Valve_SetEnableSholder(1);
    }

    if(nKeyAirBagLocate == AIRBAG_LOCATE_AUTO)
    {
        if((nBackMainRunMode == BACK_MAIN_MODE_AUTO) && ( nBackSubRunMode == BACK_SUB_MODE_AUTO_1)  )// //舒适按摩带拉伸功能
        {
           //自动程序中的拉腿程序   //暂时关闭，因为气囊程序未调试好
            Valve_StretchControlProce();  //执行拉退，气囊由算法控制
            if(st_Stretch.active == TRUE)//在拉退时刻，由数组控制手臂气囊程序
            {
        //     Valve_Control(VALVE_ENABLE, &st_AirBagArmNeck, by_EngineeringAirBag);
            }
            else
            {
                Valve_Control(VALVE_ENABLE,&st_AirBagAuto,by_EngineeringAirBag);  //在拉退空闲时刻，由数组控制气囊程序     st_AirBagAuto.locate = AIRBAG_LOCATE_AUTO;
                Valve_FootRollerProce(bRollerEnable,1,&st_AirBagAuto);

            }
        }
        else if((nBackMainRunMode == BACK_MAIN_MODE_AUTO) && (nBackSubRunMode == BACK_SUB_MODE_6MIN_DEMO)&& (b6MinDemoStretch == TRUE)    )// //舒适按摩带拉伸功能
        {
           
            
            
            
           //自动程序中的拉腿程序   //暂时关闭，因为气囊程序未调试好
            Valve_StretchControlProce();  //执行拉退，气囊由算法控制
            if(st_Stretch.active == TRUE)//在拉退时刻，由数组控制手臂气囊程序
            {
        //     Valve_Control(VALVE_ENABLE, &st_AirBagArmNeck, by_EngineeringAirBag);
            }
            else
            {
                Valve_Control(VALVE_ENABLE,&st_AirBagAuto,by_EngineeringAirBag);  //在拉退空闲时刻，由数组控制气囊程序     st_AirBagAuto.locate = AIRBAG_LOCATE_AUTO;
                Valve_FootRollerProce(bRollerEnable,1,&st_AirBagAuto);

            }
        }
        //add by wgh 20170208
       else if((nBackMainRunMode == BACK_MAIN_MODE_AUTO) && ( nBackSubRunMode == BACK_SUB_MODE_AUTO_4))
        {  
          //自动程序但是非拉腿程序  //暂时关闭，因为气囊程序未调试好
          if(st_Stretch.active)
          {
            st_Stretch.active = FALSE;
            st_Stretch.init = FALSE;
            bKeyLegPadUp = FALSE ;
            bKeyLegPadDown = FALSE ;
            bLegPadLinkage = FALSE ;
            bKeyBackPadUp = FALSE ;
            bKeyBackPadDown = FALSE ;
          }
          
          Valve_Control(VALVE_ENABLE,&st_AirBagAuto_Upbody,by_EngineeringAirBag);// st_AirBagAuto.pAirBagArray = AirBagModeAuto;// AirBagModeAuto[] =气囊数组
          Valve_FootRollerProce(bRollerEnable,1,&st_AirBagAuto_Upbody);      
          
            
        }  
        
        
        
        
        
        else
        {  
          //自动程序但是非拉腿程序  //暂时关闭，因为气囊程序未调试好
          if(st_Stretch.active)
          {
            st_Stretch.active = FALSE;
            st_Stretch.init = FALSE;
            bKeyLegPadUp = FALSE ;
            bKeyLegPadDown = FALSE ;
            bLegPadLinkage = FALSE ;
            bKeyBackPadUp = FALSE ;
            bKeyBackPadDown = FALSE ;
          }
          
          Valve_Control(VALVE_ENABLE,&st_AirBagAuto,by_EngineeringAirBag);// st_AirBagAuto.pAirBagArray = AirBagModeAuto;// AirBagModeAuto[] =气囊数组
          Valve_FootRollerProce(bRollerEnable,1,&st_AirBagAuto);      
          
            
        }
    }
    else//手动程序打开气阀
    {

        if(st_Stretch.active)
        {
            st_Stretch.active = FALSE;
            st_Stretch.init = FALSE;
            bKeyLegPadUp = FALSE ;
            bKeyLegPadDown = FALSE ;
            bLegPadLinkage = FALSE ;
            bKeyBackPadUp = FALSE ;
            bKeyBackPadDown = FALSE ;
        }
//  if(Valve_Enable)  //        0表示 自动滚轮模式 关闭 
        Valve_FootRollerProce(bRollerEnable, 0, &st_AirBagLegFoot);//手动气阀，打开手动滚轮
  //      Valve_LegKneadProce(bLegKneadEnable, 0, &st_AirBagLegFoot);
        switch(nKeyAirBagLocate)
        {
        case  AIRBAG_LOCATE_NONE:
          Valve_Control(VALVE_DISABLE, &st_AirBagArmNeck, by_EngineeringAirBag);
          Valve_Control(VALVE_DISABLE, &st_AirBagLegFoot, by_EngineeringAirBag);
          Valve_Control(VALVE_DISABLE, &st_AirBagSeat, by_EngineeringAirBag);
          Valve_AirPumpACPowerOff();
          break;
        case  AIRBAG_LOCATE_LEG_FOOT:                             //use
          Valve_Control(VALVE_DISABLE, &st_AirBagSeat, by_EngineeringAirBag);
          Valve_Control(VALVE_DISABLE, &st_AirBagArmNeck, by_EngineeringAirBag);
          Valve_Control(VALVE_ENABLE, &st_AirBagLegFoot, by_EngineeringAirBag);
          break;
        case  AIRBAG_LOCATE_ARM_NECK:                   //use st_AirBagArmNeck
          Valve_Control(VALVE_DISABLE, &st_AirBagSeat, by_EngineeringAirBag);
          Valve_Control(VALVE_DISABLE, &st_AirBagLegFoot, by_EngineeringAirBag);
          Valve_Control(VALVE_ENABLE, &st_AirBagArmNeck, by_EngineeringAirBag);
          break;
        case AIRBAG_LOCATE_SEAT:
          Valve_Control(VALVE_DISABLE, &st_AirBagArmNeck, by_EngineeringAirBag);
          Valve_Control(VALVE_DISABLE, &st_AirBagLegFoot, by_EngineeringAirBag);
          Valve_Control(VALVE_ENABLE, &st_AirBagSeat, by_EngineeringAirBag);
          break;
          //add wgh 20170206  st_AirBagLegFoot_Arm_Seat;
        case (AIRBAG_LOCATE_LEG_FOOT | AIRBAG_LOCATE_ARM_NECK): 
          Valve_Control(VALVE_DISABLE, &st_AirBagSeat, by_EngineeringAirBag);
          Valve_Control(VALVE_ENABLE, &st_AirBagLegFoot_Arm, by_EngineeringAirBag);
          break;
        case (AIRBAG_LOCATE_LEG_FOOT | AIRBAG_LOCATE_SEAT):
          Valve_Control(VALVE_DISABLE, &st_AirBagArmNeck, by_EngineeringAirBag);
          Valve_Control(VALVE_ENABLE, &st_AirBagLegFoot_Seat, by_EngineeringAirBag);
          break;
        case (AIRBAG_LOCATE_ARM_NECK| AIRBAG_LOCATE_SEAT):
          Valve_Control(VALVE_DISABLE, &st_AirBagLegFoot, by_EngineeringAirBag);
          Valve_Control(VALVE_ENABLE, &st_AirBagArm_Seat, by_EngineeringAirBag);
          break;    
        case (AIRBAG_LOCATE_LEG_FOOT | AIRBAG_LOCATE_ARM_NECK | AIRBAG_LOCATE_SEAT):
          Valve_Control(VALVE_ENABLE, &st_AirBagLegFoot_Arm_Seat, by_EngineeringAirBag);
          break;        
        }
    }
}

//#include "_3D_Position_TAB.c"
//extern  const unsigned char _3D_Position[];
unsigned int wghw_legLocation;
unsigned int wghw_backLocation;
void Main_Massage_Position_Proce(void)
{
    bool bBackPadFinish,bLegPadFinish,bSliderFinish,bFlexPadFinish;
    bool bAdjFlex = 0;
    unsigned int w_LegPosition;
    unsigned int w_BackLocation;
#ifdef  RT8305T_1
        if((bKeyBackPadUp == TRUE) || 
       (bKeyBackPadDown == TRUE) || 
       (bKeyLegPadUp == TRUE) || 
       (bKeyLegPadDown == TRUE) ||
       (bKeyFlexOut == TRUE) ||
       (bKeyFlexIn == TRUE) ||
       (st_Stretch.active)) 
        {
            bMassagePositionUpdate = 0;    //手动优先      bMassagePositionUpdate=1 为靠背，小腿上下，伸缩，零重力电机正在进行归位状态，
               //SlideMotorControl(STATE_SLIDE_IDLE); zlw  //bMassagePositionUpdate = 0;当用户进行手动操作时 ，以手动操作为优先，不自动执行Main_Massage_Position_Proce（）函数
            return;
        }
#else 
    if((bKeyBackPadUp == TRUE) || 
       (bKeyBackPadDown == TRUE) || 
       (bKeyLegPadUp == TRUE) || 
       (bKeyLegPadDown == TRUE) ||
       /*(bKeyFlexOut == TRUE) ||
       (bKeyFlexIn == TRUE) ||*/
       (st_Stretch.active)) 
    {
        bMassagePositionUpdate = 0;    //手动优先      bMassagePositionUpdate=1 为靠背，小腿上下，伸缩，零重力电机正在进行归位状态，
           //SlideMotorControl(STATE_SLIDE_IDLE); zlw  //bMassagePositionUpdate = 0;当用户进行手动操作时 ，以手动操作为优先，不自动执行Main_Massage_Position_Proce（）函数
        return;
    }
#endif
    
    w_LegPosition = LegMotor_Get_Position(); 
    
    wghw_legLocation = LegMotor_Get_Position();
    w_BackLocation = BackMotor_Get_Location();
    wghw_backLocation = BackMotor_Get_Location();
    
    //BackMotor_Proce();//靠背电机行走
    LegMotor_Proce();//行走电机行走
    
    if(!bMassagePositionUpdate) 
    {
      
      //BackMotor_Control(STATE_BACK_IDLE);
      
      
        //SlideMotorControl(STATE_SLIDE_IDLE);//零重力电机
        return;
    }
    switch(nTargetMassagePosition)
    {
    case MASSAGE_RESET_POSITION: //椅子复位
        if(BackMotor_Control(STATE_RUN_BACK_UP) == TRUE)//椅子复位时，靠背马达向上行走，直到走到最高点
        {
            bBackPadFinish = TRUE;                           //靠背向上
  
        }
        else
        {
            bBackPadFinish = FALSE;
  
        }
        //----------------------------------------------------------------
 
        if(LegMotor_Control(STATE_RUN_LEG_DOWN) == LEG_STOP_AT_DOWN)//小腿向下
        {
            bLegPadFinish = TRUE;
        }
        else
        {
            bLegPadFinish = FALSE;
        }
  
        
        //--------------------------------------------------------  
#ifdef  RT8305T_1          
        if((nFlexStatus&0x03) ==  FLEX_AT_IN) //小腿伸缩电机缩进来
        {
            bFlexPadFinish = TRUE;
        }
        else
        {
           Flex_SetCurrent(FLEX_MOTOR_CURRENT_1D5A);
           Flex_SetDirection(FLEX_TO_IN);
           bFlexPadFinish = FALSE;
        }
        
#endif         
        
        //------------------------------------------------------------
      /*   if(SlideMotorControl(STATE_RUN_SLIDE_BACKWARD) == TRUE)  //椅子升起来,零重力电机在最下面的行程开关处
         {
          bSliderFinish = TRUE;
         }
         else
         {
          bSliderFinish = FALSE;
         }
         w_ZeroPosition = 0;*/
        //--------------------------------------------------------------
        
        break;
    case MASSAGE_INIT_POSITION:

            
            if(BackMotor_Control(STATE_RUN_BACK_UP) == TRUE)
            {
                bBackPadFinish = TRUE;
            }
            else
            {
                bBackPadFinish = FALSE;
            } 
            if(LegMotor_Control(STATE_RUN_LEG_DOWN) == TRUE)
            {
                bLegPadFinish = TRUE;
            }
            else
            {
                bLegPadFinish = FALSE;
            }
            
    #ifdef  RT8305T_1         
            
            if((nFlexStatus&0x03) ==  FLEX_AT_IN) 
            {
            bFlexPadFinish = TRUE;
            }
            else
            {
               Flex_SetCurrent(FLEX_MOTOR_CURRENT_1D5A);
               Flex_SetDirection(FLEX_TO_IN);
               bFlexPadFinish = FALSE;
            }
            
   #endif             
            
         /*  if(SlideMotorControl(STATE_RUN_SLIDE_BACKWARD) == TRUE)  //椅子升起来 ,电机向下运行，
           {
            bSliderFinish = TRUE;
           }
           else 
           {
            bSliderFinish = FALSE;
           }
           w_ZeroPosition = 0;*/

          FlexMotorSetEnable();
        break;
    case MASSAGE_OPTIMAL_POSITION://STATE_RUN_SLIDE_FORWARD=向上   ,第一个零重力点 时，零重力向上运行到行程开关位置处,椅子趟下
        bAdjFlex = true;
          /* if(SlideMotorControl(STATE_RUN_SLIDE_FORWARD) == TRUE)  //零重力位置1 ,椅子向下 .电机向上运行
            {
              bSliderFinish = TRUE; 
            }
            else
            {
              bSliderFinish = FALSE; 
            }*/
          //----------------------------------------------------------------
            
            if(w_BackLocation > (MASSAGE_BACK_OPTIMAL_LOCATION + 20))
            {
                BackMotor_Control(STATE_RUN_BACK_UP) ;
                bBackPadFinish = FALSE;
            }
            else if(w_BackLocation < (MASSAGE_BACK_OPTIMAL_LOCATION - 20))
            {
                BackMotor_Control(STATE_RUN_BACK_DOWN) ;
                bBackPadFinish = FALSE;
            }
            else
            {
                BackMotor_Control(STATE_BACK_IDLE) ;
                bBackPadFinish = TRUE;
            } 
           
         //-------------------------------------------------------------  
            
            w_LegPosition = LegMotor_Get_Position(); 
            wghw_legLocation = LegMotor_Get_Position();
            
            if(w_LegPosition > 850)//(MASSAGE_LEG_OPTIMAL_POSITION + 50))
            {
                LegMotor_Control(STATE_RUN_LEG_DOWN) ;
                bLegPadFinish = FALSE;
            }
            else if(w_LegPosition < 750)//(MASSAGE_LEG_OPTIMAL_POSITION - 50))
            {
                LegMotor_Control(STATE_RUN_LEG_UP) ;
                bLegPadFinish = FALSE;
            }
            else
            {
                LegMotor_Control(STATE_LEG_IDLE) ;
                bLegPadFinish = TRUE;
            }
            //---------------------------------------------
             
            
            if((nFlexStatus&0x03) ==  FLEX_AT_IN) 
            {
            bFlexPadFinish = TRUE;
            }
            else
            {
               Flex_SetCurrent(FLEX_MOTOR_CURRENT_1D5A);
               Flex_SetDirection(FLEX_TO_IN);
               bFlexPadFinish = FALSE;
            }

       

           
          
         FlexMotorSetEnable();   
            
            
        break;
        
    case MASSAGE_OPTIMAL2_POSITION:
        bAdjFlex = true;
          //------------------------------------------------------------------
         /*   if(SlideMotorControl(STATE_RUN_SLIDE_FORWARD) == TRUE)  
            {
              bSliderFinish = TRUE; 
            }
            else
            {
              bSliderFinish = FALSE; 
            }       
        */
        
           //---------------------------------------------------------
            w_BackLocation = BackMotor_Get_Location();
            if(w_BackLocation > (MASSAGE_BACK_OPTIMAL1_LOCATION + 20))//POSITION_CTRL_OFFSET))
            {
                BackMotor_Control(STATE_RUN_BACK_UP) ;
                bBackPadFinish = FALSE;
            }
            else if(w_BackLocation < (MASSAGE_BACK_OPTIMAL1_LOCATION - 20))//POSITION_CTRL_OFFSET))
            {
                BackMotor_Control(STATE_RUN_BACK_DOWN) ;
                bBackPadFinish = FALSE;
            }
            else
            {
                BackMotor_Control(STATE_BACK_IDLE) ;
                bBackPadFinish = TRUE;
            } 
            
            //---------------------------------------------------------
            w_LegPosition = LegMotor_Get_Position(); 
            if(w_LegPosition > (MASSAGE_LEG_OPTIMAL1_POSITION + POSITION_CTRL_OFFSET))
            {
                LegMotor_Control(STATE_RUN_LEG_DOWN) ;
                bLegPadFinish = FALSE;
            }
            else if(w_LegPosition < (MASSAGE_LEG_OPTIMAL1_POSITION - POSITION_CTRL_OFFSET))
            {
                LegMotor_Control(STATE_RUN_LEG_UP);//STATE_RUN_BACK_UP) ;
                bLegPadFinish = FALSE;
            }
            else
            {
                LegMotor_Control(STATE_LEG_IDLE);//STATE_BACK_IDLE) ;
                bLegPadFinish = TRUE;
            }
            //----------------------------------------------------------
#ifdef  RT8305T_1          
          if((nFlexStatus&0x03) ==  FLEX_AT_IN) 
            {
            bFlexPadFinish = TRUE;
            }
           else
           {
           Flex_SetCurrent(FLEX_MOTOR_CURRENT_1D5A);
           Flex_SetDirection(FLEX_TO_IN);
           bFlexPadFinish = FALSE;
           }


        
 #endif             
        FlexMotorSetEnable();    
            
            
        break;  
    case MASSAGE_MAX_POSITION:
        bAdjFlex = true;

           /* if(SlideMotorControl(STATE_RUN_SLIDE_FORWARD) == TRUE)  
            {
              bSliderFinish = TRUE; 
            }
            else
            {
              bSliderFinish = FALSE; 
            }*/
            //------------------------------------
            if(BackMotor_Control(STATE_RUN_BACK_DOWN) == TRUE)
            {
                bBackPadFinish = TRUE;
            }
            else
            {
                bBackPadFinish = FALSE;
            } 
            //--------------------------------------
            if(LegMotor_Control(STATE_RUN_LEG_UP) == LEG_STOP_AT_UP)
            {
                bLegPadFinish = TRUE;
            }
            else
            {
                bLegPadFinish = FALSE;
            }
            //---------------------------------------
   #ifdef  RT8305T_1           
            
       
            
         if((nFlexStatus&0x03) ==  FLEX_AT_IN) 
            {
            bFlexPadFinish = TRUE;
            }
            else
            {
              Flex_SetCurrent(FLEX_MOTOR_CURRENT_1D5A);
              Flex_SetDirection(FLEX_TO_IN);
              bFlexPadFinish = FALSE;
            }


            FlexMotorSetEnable();
            
    #endif               
            
             break;
    case MASSAGE_ANY_POSITION:
    default:  
        bMassagePositionUpdate = 0;bZeroflash = FALSE;
        break;
    }
#ifdef  RT8305T_1    
    
    if( (bBackPadFinish == TRUE) && (bLegPadFinish == TRUE)&& (bFlexPadFinish == TRUE))
    {
        bMassagePositionUpdate = 0;bZeroflash = FALSE;
         
        if(bAdjFlex)
        {
           FlexMotorSetEnable(); 
        }
    }
    
#else 
    if((bSliderFinish == TRUE) && (bBackPadFinish == TRUE) && (bLegPadFinish == TRUE)/*&& (bFlexPadFinish == TRUE)*/)
    {
        bMassagePositionUpdate = 0;
         
       /* if(bAdjFlex)
        {
           FlexMotorSetEnable(); 
        }*/
    }
 #endif
    
    
    
}
 unsigned int wwww_LegPosition;
unsigned int wwwr_BackLocation;
//靠背电动缸控制程序
void Main_BackPad_Proce(void)
{
  


  wwww_LegPosition = LegMotor_Get_Position();
  wwwr_BackLocation= BackMotor_Get_Location(); 
  
  
  
  
  
    //BackMotor_Proce();//靠背马达运行时 坐标开始计数
    if(st_Stretch.active) return;
    if(bMassagePositionUpdate) return;

  if((bKeyBackPadUp == TRUE)||(bKeyBackPadDown == TRUE))
  {
    Flex_ControlStop();
  }
    
    if(bKeyBackPadUp == TRUE)//  if(bKeyBackPadDown == TRUE) if((bKeyBackPadUp == FALSE) && (bKeyBackPadDown == FALSE))
    {
      /*  if(bKeySeatVibrate)
        {
          bVibratePause=1;//摇摆电机暂停运行
          bKeySeatVibrate=0;
          
         Waveringly_Set_Pwm_Data(0);
         
         nKeySeatVibrateStrength_old=nKeySeatVibrateStrength;
         //nKeySeatVibrateStrength=2;//nKeySeatVibrateStrength_old
         
        }*/
        
      
        if(BackMotor_Control(STATE_RUN_BACK_UP) == TRUE)
        {
              if(bRockEnable == false)
              {
                nBuzzerMode = BUZZER_MODE_FAST ;
                bSendBuzzerMode = TRUE ;
              }
        }
        else
        {
            if(bBackLegPadSettle == FALSE  && nRockModeEnterEnable == ExitRock/* && st_Stretch.bBackLegFlag == FALSE*/)
            {
                nBuzzerMode = BUZZER_MODE_SLOW ;//wgh
                bSendBuzzerMode = TRUE ;
            }        
          
          
        }
        

    }
    if(bKeyBackPadDown == TRUE)
    {

         /*   if(bKeySeatVibrate)
            {
              bVibratePause=1;//摇摆电机暂停运行
              bKeySeatVibrate=0;
              Waveringly_Set_Pwm_Data(0);
               nKeySeatVibrateStrength_old=nKeySeatVibrateStrength;
              
            }*/
            if(BackMotor_Control(STATE_RUN_BACK_DOWN) == TRUE)
            {
              if(bRockEnable == false)
              {
                nBuzzerMode = BUZZER_MODE_FAST ;
                bSendBuzzerMode = TRUE ;
              }
            }
            else
            {
            if(bBackLegPadSettle == FALSE  && nRockModeEnterEnable == ExitRock/* && st_Stretch.bBackLegFlag == FALSE*/)
            {
                nBuzzerMode = BUZZER_MODE_SLOW ;//wgh
                bSendBuzzerMode = TRUE ;
            }
            }

            

    }
    if((bKeyBackPadUp == FALSE) && (bKeyBackPadDown == FALSE))
    {
        BackMotor_Control(STATE_BACK_IDLE) ;
        
       /* if(bVibratePause)
        {
           nKeySeatVibrateStrength=nKeySeatVibrateStrength_old;
          bKeySeatVibrate=1;
  
          
          bKeySeatEnable=1;
          bVibratePause=0;
          
        }*/
        

    }
}

//小腿电动缸控制程序
/*
void Main_LegPad_Proce(void)
{
    LegMotor_Proce();
    if(st_Stretch.active) return;
    if(bMassagePositionUpdate) return;
    if(bLegPadLinkage == FALSE) //小腿单独动，此时不考虑前滑电动缸的位置
    {
        if(bKeyLegPadUp == TRUE)
        {
            FlexMotorSetEnable();
            if(LegMotor_Control(STATE_RUN_LEG_UP) != LEG_RUN)
            {
                nBuzzerMode = BUZZER_MODE_FAST ;
                bSendBuzzerMode = TRUE ;
            }
            else
            {
                nBuzzerMode = BUZZER_MODE_SLOW ;
                bSendBuzzerMode = TRUE ;
            }
        }

        if(bKeyLegPadDown == TRUE)
        {
            FlexMotorSetEnable();
            switch(LegMotor_Control(STATE_RUN_LEG_DOWN))
            {
            case LEG_RUN:
            {
                nBuzzerMode = BUZZER_MODE_SLOW ;
                bSendBuzzerMode = TRUE ;
            }
            break;
            case LEG_STOP_AT_DOWN:
            {
                nBuzzerMode = BUZZER_MODE_FAST ;
                bSendBuzzerMode = TRUE ;
            }
            break;
            case  LEG_STOP_AT_GROUND:

                FlexMotor_Control(STATE_RUN_FLEX_RESET, FLEX_SPEED_FAST, FLEX_CURRENT_3A);

                break;
            case LEG_STOP_AT_ANGLE:
                break;
            }
        }

    }
    else  //靠背和小腿联动，前滑电动缸必须在最前位置
    {
        if(SlideMotor_Get_Location() == SLIDE_MOTOR_AT_FORWARD)
        {
            if(bKeyLegPadUp == TRUE)
            {
                LegMotor_Control(STATE_RUN_LEG_UP);
                FlexMotorSetEnable();
            }
            if(bKeyLegPadDown == TRUE)
            {
                LegMotor_Control(STATE_RUN_LEG_DOWN);
                FlexMotorSetEnable();
            }
        }
    }

    if((bKeyLegPadUp == FALSE) && (bKeyLegPadDown == FALSE))
    {
        LegMotor_Control(STATE_LEG_IDLE) ;
    }
}
*/
void BodyDataRefresh(void)
{
    
    unsigned short by_TopPosition = TOP_POSITION;//nShoulderPosition

    if(nShoulderPosition >= (by_TopPosition - MAX_SHOULDER_ADJUST_DIFF))
    {
        nShoulderPositionTop = by_TopPosition ;
        nShoulderPositionBottom = nShoulderPosition - MAX_SHOULDER_ADJUST_DIFF ;
    }
    else if(nShoulderPosition < MAX_SHOULDER_ADJUST_DIFF)
    {
        nShoulderPositionTop = nShoulderPosition + MAX_SHOULDER_ADJUST_DIFF ;
        nShoulderPositionBottom = 0 ;
    }
    else
    {
        nShoulderPositionTop = nShoulderPosition + MAX_SHOULDER_ADJUST_DIFF ;
        nShoulderPositionBottom = nShoulderPosition - MAX_SHOULDER_ADJUST_DIFF ;
    }
    /*
    nZoneStep = nShoulderPosition / 6 ;
    nZoneStepRemain = nShoulderPosition % 6 ;
    WALK_MOTOR_ZONE[0] = nZoneStep + ((nZoneStepRemain > 0)?1:0) ;
    if(nZoneStepRemain > 0) nZoneStepRemain-- ;
    WALK_MOTOR_ZONE[1] = WALK_MOTOR_ZONE[0] + nZoneStep + ((nZoneStepRemain > 0)?1:0) ;
    if(nZoneStepRemain > 0) nZoneStepRemain-- ;
    WALK_MOTOR_ZONE[2] = WALK_MOTOR_ZONE[1] + nZoneStep + ((nZoneStepRemain > 0)?1:0) ;
    if(nZoneStepRemain > 0) nZoneStepRemain-- ;
    WALK_MOTOR_ZONE[3] = WALK_MOTOR_ZONE[2] + nZoneStep + ((nZoneStepRemain > 0)?1:0) ;
    if(nZoneStepRemain > 0) nZoneStepRemain-- ;
    WALK_MOTOR_ZONE[4] = WALK_MOTOR_ZONE[3] + nZoneStep + ((nZoneStepRemain > 0)?1:0) ;
    if(nZoneStepRemain > 0) nZoneStepRemain-- ;
    WALK_MOTOR_ZONE[5] = WALK_MOTOR_ZONE[4] + nZoneStep + ((nZoneStepRemain > 0)?1:0) ;
    */
}
bool isFIRSTZeroPosition(void)
{
  unsigned int w_BackPosition= BackMotor_Get_Location();
  unsigned int w_LegPosition = LegMotor_Get_Position(); 
  bool result;    
  result = (w_BackPosition < (MASSAGE_BACK_OPTIMAL_LOCATION + 30));
  if(result) result = (w_BackPosition > (MASSAGE_BACK_OPTIMAL_LOCATION - 30));
  if(result) result = (w_LegPosition > (MASSAGE_LEG_OPTIMAL_POSITION - POSITION_DISPLAY_OFFSET));
  if(result) result = (w_LegPosition < (MASSAGE_LEG_OPTIMAL_POSITION + POSITION_DISPLAY_OFFSET));
  
 // if(result)result=Input_GetSlideForwardSwitch();
  return(result);   
}
bool isZeroPosition(void)
{/*
  static  unsigned int w_BackLocation;// = BackMotor_Get_Location();
 static   unsigned int w_LegPosition;// = LegMotor_Get_Position(); 
 w_BackLocation = BackMotor_Get_Location();
 w_LegPosition= LegMotor_Get_Position();
 
 w_leg_time  =LegMotor_Get_Position();
 */
 unsigned int w_BackLocation= BackMotor_Get_Location();
  unsigned int w_LegPosition = LegMotor_Get_Position(); 
/*
  if(Input_GetSlideForwardSwitch() == REACH_SLIDE_LIMIT )
  {
    
    
    
    
  }*/
  
  
  
  
  
    bool result;    
    result = (w_BackLocation < (MASSAGE_BACK_OPTIMAL1_LOCATION + 30));//POSITION_DISPLAY_OFFSET));
    if(result) result = (w_BackLocation > (MASSAGE_BACK_OPTIMAL1_LOCATION - 30));//POSITION_DISPLAY_OFFSET));
    if(result) result = (w_LegPosition > (MASSAGE_LEG_OPTIMAL1_POSITION - POSITION_DISPLAY_OFFSET));
    if(result) result = (w_LegPosition < (MASSAGE_LEG_OPTIMAL1_POSITION + POSITION_DISPLAY_OFFSET));
    
    //if(result)result=Input_GetSlideForwardSwitch();
    return(result);   
}

void Main_Close_Power(void)
{

    nKeyBackLocate = LOCATE_NONE;
   

    
    //气囊按摩停止
    nKeyAirBagLocate = AIRBAG_LOCATE_NONE;
    //振动功能停止
    //加热功能停止
    bKeySeatVibrate = FALSE;
    bKeyWaistHeat = FALSE;
    //定时变量复位
    Data_Set_Start(0,0);
   bRollerEnable = FALSE; 
    //nRollerPWM = 0;
    //Valve_SetRollerPWM(nRollerPWM); 
    //bRunTimeChange = TRUE ;
 
   
   
}

void BackManualModeNoAction(void)
{
    nBackMainRunMode = BACK_MAIN_MODE_MANUAL ;
    nBackSubRunMode = BACK_SUB_MODE_NO_ACTION ;
    nKeyKneadKnockSpeed = SPEED_0 ;
    nCurKneadKnockSpeed = SPEED_0 ;
    ManualDirector[0].nSubFunction = BACK_SUB_MODE_NO_ACTION ;
    ManualDirector[0].nKneadMotorState = KNEAD_STOP ;//KNEAD_STOP_AT_MAX ;
    ManualDirector[0].nKneadMotorCycles = 0 ;
    ManualDirector[0].nKnockMotorState = KNOCK_STOP ;
    ManualDirector[0].nKnockMotorRunTime = 0 ;
    ManualDirector[0].nKneadKnockSpeed = nKeyKneadKnockSpeed ;
    nMaxActionStep = 1 ;
    nStartActionStep = 0 ;
    bBackManualModeInit = TRUE ;
}

void Main_Send_Leg(void)// 1 方向 主板到小腿控制板
{
  unsigned char buffer;
  
    if(bMasterSendLegPacket)
    {
        OutLegBuffer[0] = SOI ;
        //buffer 1
        OutLegBuffer[1] = Roller_GetSpeed() & 0x03;
        buffer = Roller_GetMode() << 2;
        OutLegBuffer[1] |= buffer;
        OutLegBuffer[1] |= 1 << 6;  //allways on
        //buffer 2
        
        OutLegBuffer[2] = Flex_GetDirection();
        buffer = Flex_GetCurrent() << 2;
        OutLegBuffer[2] |= buffer;
        
        if(Flex_GetDisableAngle())  OutLegBuffer[2] |= (1<<5);// 伸缩不依赖角度
        
        //buffer 3
  //=================================================================================/////     
        OutLegBuffer[3] = LegKnead_GetSpeed() & 0x03;    //buffer3 =8302T 不用 
        buffer = LegKnead_GetMode() << 2;
        OutLegBuffer[3] |= buffer;
        buffer = LegKnead_GetPower() << 4;
        OutLegBuffer[3] |= buffer;
 //======================================================================================       
        //buffer 3
        
        unsigned char valve;
        valve = BITS_ValveData[1].nByte;
        
        OutLegBuffer[4] = valve&0x3f;

        OutLegBuffer[5] = EOI;
        nOutLegBufferCount = 6;
        UartLeg_Transmit_Packet(OutLegBuffer,nOutLegBufferCount);
        bMasterSendLegPacket = FALSE ;
    }
    
   if(UartLeg_GetRXStatus())
   {
      UartLeg_ClearRXStatus(); 
      nLegAngle = UartLeg_GetAngle();
      nFlexStatus = UartLeg_GetFlexStatus();
      Flex_SetStatus(nFlexStatus);
      nLegReady = UartLeg_GetLegStatus();
   }
}
/*
void Main_MassageSignalSend(void)
{
  unsigned char outBuffer[5];
  int count;
    if(bSignalSendPacket)
    {
       outBuffer[0] = SOI ;
        
       if(WalkPower_Get() == WALK_MOTOR_POWER_ON)
       {
        outBuffer[1] = BIT0; 
       }
    
       if(WalkMotor_GetDirection() ==  WALK_MOTOR_GO_UP)
       {
        outBuffer[1] |= BIT1; 
       } 
       
       if(Input_GetWalkUpSwitch() == REACH_WALK_LIMIT)
       {
         outBuffer[1] |= BIT2; 
       }
       if(Input_GetWalkDownSwitch() == REACH_WALK_LIMIT)
       {
         outBuffer[1] |= BIT3; 
       }
       
       if(AxisMotor_IsRun())
       {
        outBuffer[1] |= BIT4; 
       } 
       
       if(AxisMotor_GetDirection() == AXIS_MOTOR_GO_FORWARD)     
       {
         outBuffer[1] |= BIT5; 
       }
       outBuffer[2] = ~outBuffer[1];
       outBuffer[2] &= 0x7f;
       outBuffer[3] = EOI;
       count = 4;
       LEUART0_Transmit_Packet(outBuffer,count);
       bSignalSendPacket = FALSE ;
    } 
}
*/ 
unsigned char nPreRollerSpeed;
void Main_Send(void)
{
    unsigned int Send_BackLocation = BackMotor_Get_Location();
    if(bMasterSendPacket)
    {
      OutBuffer[0] = SOI ;
      //标识 1	按摩椅运行状态 1	按摩手法 3	按摩程序 3
      if(nChairRunState == CHAIR_STATE_IDLE || nChairRunState == CHAIR_STATE_SLEEP)
      {
        OutBuffer[1] = 0<<6;//0：按摩椅处于待机,主电源关闭，省电模式
      }
      else
      {
        OutBuffer[1] = 1<<6;//按摩椅处于非待机状态，此时手控器相应的图标点亮
      }
      
      /*****************************************************/
      //按摩手法显示
      switch(nCurSubFunction)
      {
        //00：停止
        //01：揉捏
        //02：敲击
        //03：揉敲同步
        //04：叩击
        //05：指压
        //06：韵律按摩
        //07：保留
      case BACK_SUB_MODE_KNEAD			: OutBuffer[1] |= 1<<3;break;
      case BACK_SUB_MODE_KNOCK			: OutBuffer[1] |= 4<<3;break;//OutBuffer[1] |= 2<<3;break;
      case BACK_SUB_MODE_WAVELET		: OutBuffer[1] |= 3<<3;break;
      case BACK_SUB_MODE_SOFT_KNOCK		: OutBuffer[1] |= 2<<3;break;//OutBuffer[1] |= 4<<3;break;
      case BACK_SUB_MODE_PRESS			: OutBuffer[1] |= 5<<3;break;
      case BACK_SUB_MODE_MUSIC			: OutBuffer[1] |= 6<<3;break;
      case BACK_SUB_MODE_RUBBING                : OutBuffer[1] |= 7<<3;break;
      default                                   : OutBuffer[1] |= 0<<3;break; 
      }
      
      OutBuffer[1] |= 0x01;  //3D 标识  机器具有3D功能
      /*
      if(nBackMainRunMode == BACK_MAIN_MODE_MANUAL)
      {
      OutBuffer[1] |=  7;
    }
        else if(nBackMainRunMode == BACK_MAIN_MODE_AUTO)
      {           
      BYTE mode = (nBackSubRunMode + 1);
      
      if(nChairRunState == CHAIR_STATE_DEMO)
      {
      mode &= bDisplayFlash;
    }
      
      OutBuffer[1] |= mode&0x7;
    }
        else if(nChairRunState == CHAIR_STATE_RUN)
      {
      OutBuffer[1] |=  7;
    }
      */
      //标识 1 加热 1	保留 1	按摩机芯速度 3 	揉捏头宽度位置 2
      //00-03 自定义
OutBuffer[2] = 0; 
      unsigned char speed;
      if(nBackMainRunMode == BACK_MAIN_MODE_IDLE || nBackMainRunMode == BACK_MAIN_MODE_3D)
      {
        speed = 0;
      }
      else 
      {
          
          
         // if((nBackSubRunMode == BACK_SUB_MODE_PRESS)&&((nBackMainRunMode==BACK_MAIN_MODE_AUTO)||(nBackMainRunMode==BACK_MAIN_MODE_MANUAL)))
        
      //  if((nBackMainRunMode==BACK_MAIN_MODE_AUTO)||(nBackMainRunMode==BACK_MAIN_MODE_MANUAL))
         if(nBackMainRunMode==BACK_MAIN_MODE_MANUAL)
          {
            
               if(nBackSubRunMode == BACK_SUB_MODE_PRESS)
                speed = 0;
                  else
                  speed = nCurKneadKnockSpeed;//按摩机芯速度  
                    
          }
         
          if(nBackMainRunMode==BACK_MAIN_MODE_AUTO)
          {
            
                if(AutoDirector.nSubFunction==BACK_SUB_MODE_PRESS)
                {
         
                      speed = 0;
            
                            
                 
                  
                }
                else
                    speed = nCurKneadKnockSpeed;//按摩机芯速度
            
            
          }
         
         
         
       /*   else
          {

            speed = nCurKneadKnockSpeed;//按摩机芯速度
          }*/
          
          
          
      }
      OutBuffer[2] =((bKeyWaistHeat&0x1)<<6)|((speed&0x7)<<2)|(Input_GetKneadPosition()&0x3);
      
      if(bRollerEnable)
      {
        
      //  if(bRollerDisplay ==false)
      //  {
      //    OutBuffer[2] |= (0<<5);
      //  }
      //else
       // {
          
          OutBuffer[2] |= (1<<5);
       // }

      }
      
      
      
     OutBuffer[3] = 0;
      // 标识 1	负离子开关 1 	 振动（或扭腰）强度 3	气压强度 3

      //OutBuffer[3] = ;//(nKeySeatVibrateStrength&0x7)<<3;
      
   /*   
      if(bOzonEnable)//负离子开关，8302T默认为关 ，
      {
        OutBuffer[3] |= (1<<6);
      }
      else
      {
        OutBuffer[3] &= ~(1<<6);
      }
     */ 
      if(nKeyAirBagLocate != AIRBAG_LOCATE_NONE)
      {
        OutBuffer[3] |= (Valve_GetAirBagStrength()&0x7);//默认5档气压强度,气囊力度
      }
      //标识 1	机芯按摩部位 2	运行时间高5位 5
      //显示位置
      OutBuffer[4] = 0;           
      if(nBackMainRunMode == BACK_MAIN_MODE_AUTO)
      {
        if((nBackSubRunMode == BACK_SUB_MODE_AUTO_0) ||
           (nBackSubRunMode == BACK_SUB_MODE_AUTO_1) ||
             (nBackSubRunMode == BACK_SUB_MODE_AUTO_2) ||
               (nBackSubRunMode == BACK_SUB_MODE_AUTO_3))
        {
          OutBuffer[4] = 1<<5;           
        }
        else
        {
          OutBuffer[4] = 2<<5;
        }
      }
      else
      {
        switch(nKeyBackLocate)
        {
        case LOCATE_FULL_BACK:
          OutBuffer[4] = 1<<5;           
          break ;
        case LOCATE_PARTIAL:
          OutBuffer[4] = 2<<5;
          break ;
        case LOCATE_POINT:
          OutBuffer[4] = 3<<5; ;
          break ;
        default://include LOCATE_NONE
          break ;
        }
      }
#ifdef FORCE_CONTROLLER
      unsigned int time; 
      time = KneadMotor_GetCurrent();
      time *= 60;
#else
      unsigned int time = Data_Get_TimeSecond();//按摩椅剩余运行秒数
#endif    
      OutBuffer[4] |=(time>>7)& 0x1f;
      //标识 1	运行时间低7位 7
      OutBuffer[5] = time & 0x7f;
      
      OutBuffer[6] = 0x00;
      if( (bLeftFootAirBagValve) | (bRightFootAirBagValve) |(bFootHeelAirBagValve))//足部气囊动作指示
      {
        OutBuffer[6] |= 0x01;
      }
      if( (bLegDownBottomAirBagValve) |(bLegDownSideAirBagValve)|(bLegUpBottomAirBagValve)|(bLegUpSideAirBagValve))//小腿气囊动作指示
      {
       OutBuffer[6] |= 0x02;
      }
      if((bLeftThighAirBagValve) | (bRightThighAirBagValve ))//大腿气囊动作指示
      {
        OutBuffer[6] |= 0x04;
      }
 //===================================================     
  //  在该区域要增加坐垫气囊动作指示
     // if(bButtocksAirBagValve)
     // {
     //   OutBuffer[6] |= 0x08;
     // }
  //===================================================    
   //手臂气囊动作指示   
      if((bLeftArmUpAirBagVave) | (bLeftArmDownAirBagValve)| (bRightArmDownAirBagValve)|(bRightArmUpAirBagValve))
      {
        OutBuffer[6] |= 0x10;
      }
    /*  
      if(bRollerEnable)
      {
        OutBuffer[6] |= (Roller_GetSpeed() << 5);
      }
      else
      {
        OutBuffer[6] |= (0<<5);     
      }
      */
      
      if(bRollerEnable)
        {
          if(Valve_RollerIsAuto())
          {
            //unsigned int rollerPWM;
            //rollerPWM = displayPWM;
            if(Roller_GetSpeed() == ROLLER_SPEED_STOP) OutBuffer[6] |= (0 << 5);
            else if(Roller_GetSpeed() == ROLLER_SPEED_SLOW) OutBuffer[6] |= (1 << 5);
            else if(Roller_GetSpeed() == ROLLER_SPEED_MID) OutBuffer[6] |= (2 << 5);
            else if(Roller_GetSpeed() == ROLLER_SPEED_FAST) OutBuffer[6] |= (3 << 5);
          }
          else
          {
            if(   Roller_GetSpeed() != 0  )
            {
              OutBuffer[6] |= (Roller_GetSpeed() << 5);
              nPreRollerSpeed = (Roller_GetSpeed() << 5);
            }
            else
            {
              OutBuffer[6] |= nPreRollerSpeed ;
            }
            
            
            
          }
        }
        else
        {
            OutBuffer[6] |= (0 << 5);
            nPreRollerSpeed = 0;
        }
      
      OutBuffer[7] = 0x0;
      

      //if((bSholderAirBagValve))// | (bRightSholderAirBagValve))//肩部气囊动作指示
       if(  (bRightSholderAirBagValve)||(bLeftSholderAirBagValve))            
      {
        OutBuffer[7] |=  0x10;
      }


      

 
      
 //=================================================================     
      
      OutBuffer[7] &= 0xf0;
      
      BYTE state = nChairRunState;
      if(nChairRunState == CHAIR_STATE_SLEEP) 
      {
        state = CHAIR_STATE_IDLE;
      }
      if(nChairRunState == CHAIR_STATE_DEMO) 
      {
        state = CHAIR_STATE_RUN;
      }
      if(nChairRunState == CHAIR_STATE_CALIBRATION) 
      {
        state = CHAIR_STATE_RUN;
      }
      OutBuffer[7] |= (state&0x0f);
      
      
      OutBuffer[8] =0;
      
      unsigned int data = Input_GetWalkMotorPosition();
   //   data /= 23;//31;
      //0   
      /*data /= 26;//31; 总共8个点针图，屁股点阵按摩不到
      if(data >= 7) data = 7;
      OutBuffer[8] = (data+0);   //玉喜该图标
      */
      data /= 31;
      if(data >= 13) data = 13;
      OutBuffer[8] = data;   
     
      
///////////////////////////////////////////////////////////////
      if(isFIRSTZeroPosition())//零重力提示,
      {
        if(bRockEnable == false)
          OutBuffer[8] |= 1<<5;  
      }
      else if(isZeroPosition())
      {
        if(bRockEnable == false)
          OutBuffer[8] |= 1<<6;	
        
      }
      else
      {
        if(bMassagePositionUpdate)
        {
          if(nTargetMassagePosition == MASSAGE_OPTIMAL_POSITION)
          {
            if(bDisplayFlash)
            {
              if(bRockEnable == false)OutBuffer[8] |= (1<<5);	
            }
          }
          else if(nTargetMassagePosition == MASSAGE_OPTIMAL2_POSITION)
          {
            if(bDisplayFlash)
            {
              if(bRockEnable == false)OutBuffer[8] |= (1<<6);	
            }
            
            
          }
          else
          {
            OutBuffer[8] |= 0<<5; 
            OutBuffer[8] |= 0<<6; 
          } 
        }
      }
      
      
      
/*
      if(isZeroPosition())
      {
        if((bRockEnable == FALSE)&&(bZeroflash == FALSE)&&(st_Stretch.active == FALSE) ) 
        {
          OutBuffer[8] |= 1<<6; 
        }
        else
        {
          OutBuffer[8] |= 0<<6;
        }
      }
      else//到位   闪烁 
      {
        if((bZeroflash == TRUE)&&(BackMotor_GetDirection() == BACK_MOTOR_GO_DOWN )  )
        {
          if(bZeroTimer100MS == TRUE)
          {
            bZeroTimer100MScount++;
          }            
          if(bZeroTimer100MScount<10)
          {
            if(bRockEnable == FALSE)
            {
              OutBuffer[8] |= 1<<6;
            }
            else
            {
              OutBuffer[8] |= 0<<6;
            }
            //OutBuffer[10] = 1<<6; 
          }            
          else if( (bZeroTimer100MScount>=10)&&(bZeroTimer100MScount<20))
          {
            OutBuffer[8] |= 0<<6;
          }
          else
          {
            bZeroTimer100MScount =0;
          }            
          
        }
        else if((bZeroflash == TRUE)&&(BackMotor_GetDirection() == BACK_MOTOR_GO_UP )&&(Send_BackLocation > (MASSAGE_BACK_OPTIMAL1_LOCATION - 30))  )
        {
          if(bZeroTimer100MS == TRUE)
          {
            bZeroTimer100MScount++;
          }           
          if(bZeroTimer100MScount<10)
          {
            if(bRockEnable == FALSE)
            {
              OutBuffer[8] |= 1<<6;
            }
            else
            {
              OutBuffer[8] |= 0<<6;
            }
            //OutBuffer[10] = 1<<6; 
          }            
          else if( (bZeroTimer100MScount>=10)&&(bZeroTimer100MScount<20))
          {
            OutBuffer[8] |= 0<<6;
          }
          else
          {
            bZeroTimer100MScount =0;
          }            
          
        }
        else
        {
          bZeroTimer100MScount =0;
          OutBuffer[8] |= 0<<6;
        }
        
        
        
      }
       
      
      
 */     
      
      
//////////////////////////////////////////////////////////////////      
      OutBuffer[9] = 0;
      if(BodyDetectStep == DETECT_SHOULDER) 
      {
        if((ShoulderSteps >= BODY_DETECT_PREPARE) && (ShoulderSteps < BODY_DETECT_OVER))
        { 
          OutBuffer[9] = 1<<6;//1：执行体型检测程序
        }
        else
        {
          OutBuffer[9] = 0<<6;
        }
        
        if(ShoulderSteps == BODY_DETECT_ADJ)
        {
          OutBuffer[9] |= 1<<5;
          data = nShoulderPositionTop - nShoulderPositionBottom;
          time = data /15;
          data = (Input_GetWalkMotorPosition()-nShoulderPositionBottom) / time;
          if(data == 0) data = 1;
          if(data > 15) data = 15;
          
          OutBuffer[9] |= data&0x0f;
          
        }
      }
      //标识 1	运行指示 1	小腿电动缸运行方向指示 3	靠背电动缸运行方向指示 3
      OutBuffer[10] = 0;
   // if((nBackMainRunMode == BACK_MAIN_MODE_AUTO) && ( nBackSubRunMode == BACK_SUB_MODE_AUTO_1))

      //==================================================================这里要改为零重力电动缸指示
    /*  if(SlideMotor_GetPower() == SLIDE_MOTOR_POWER_ON)//零重力电动缸或滑动电动缸运行指示 
        //if(BackMotor_GetPower() == BACK_MOTOR_POWER_ON)//靠背电动缸运行指示
        //if(LegMotor_GetPower() == LEG_MOTOR_POWER_ON)//小腿电动缸运行指示
      {
        if(SlideMotor_GetDirection() == SLIDE_MOTOR_GO_FORWARD)
        {
          OutBuffer[10] |= 0x01<<4;
        }
        else
        {
          OutBuffer[10] |= 0x02<<4;
        }
      }*/
      //=================================================================
      if(BackMotor_GetPower() == BACK_MOTOR_POWER_ON)//靠背电动缸运行指示
      {
        if(BackMotor_GetDirection() == BACK_MOTOR_GO_UP)
        {
          OutBuffer[10] |= 0x01;
        }
        else
        {
          OutBuffer[10] |= 0x02;
        }
      }
      //===============================================================
      if(LegMotor_GetPower() == LEG_MOTOR_POWER_ON)//小腿电动缸运行指示
      {
        if(LegMotor_GetDirection() == LEG_MOTOR_GO_UP)
        {
          OutBuffer[10] |= (0x01<<2);
        }
        else
        {
          OutBuffer[10] |= (0x02<<2);
        }
      }
      //====================================================
      //标识	1 蜂鸣器模式 2 音乐开关 1	音量 4
      if(bSendBuzzerMode == TRUE)
      {
             
        OutBuffer[11] = (nBuzzerMode&0x3)<<5;                 
       // bSendBuzzerMode = FALSE ;  在蓝牙通讯中清零
      }
      else
      {
        OutBuffer[11] = 0;
      }
      
      if(bBlueToothStatus)
      //if(BlueToothMuteState() == BlueTooth_Speak_Out_On)
      {
        OutBuffer[11] |= 1 << 4;   //蓝牙音乐开关
      }
      else
      {
        OutBuffer[11] &= ~(1 << 4); //BlueTooth_MutePin_Value is equal to "0xef"
      }
      switch(w_PresetTime)
      {
      case RUN_TIME_10: OutBuffer[12] = 1;  break;
      case RUN_TIME_20: OutBuffer[12] = 2;  break;
      case RUN_TIME_30: OutBuffer[12] = 3;  break;
      default: OutBuffer[12] = 0; break;
      }
      //OutBuffer[12] |= ((nKeyAirBagLocate & 0x07) << 2);//选中的气囊按摩部位
      OutBuffer[12] |= ((nKeyAirBagLocate & 0x1F) << 2);//选中的气囊按摩部位
      //滚轮方向
    /*  if(bRollerEnable)
      {
        // if(ReadRollerPhase() == 1)
        {
          OutBuffer[13] = 1;
        }
        // else if(ReadRollerPhase() == 0)
        {
          OutBuffer[13] = 2;
        }
        // else
        OutBuffer[13] = 0;
      }
      else
      {
        OutBuffer[13] = 0;
      }*/
      OutBuffer[13] = 0;

      BYTE mode;
      if(nChairRunState == CHAIR_STATE_DEMO)
      {
        mode = 1 & bDisplayFlash;
        OutBuffer[13] |= mode << 2;
        time = Data_Get_ProgramExecTime()/60;
        OutBuffer[4] &= 0xE0;
        OutBuffer[4] |=(time>>7)& 0x1f;
        //标识 1	运行时间低7位 7
        OutBuffer[5] = time & 0x7f;
      }
      else
      {
        if(nBackMainRunMode == BACK_MAIN_MODE_AUTO)//自动按摩程序标志
        {
              mode = (nBackSubRunMode + 1);
              
             /*  if(nBackSubRunMode >= BACK_SUB_MODE_CLUDE_AUTO_0&&nBackSubRunMode <= BACK_SUB_MODE_CLUDE_AUTO_3)
               {
                 mode = nBackSubRunMode-BACK_SUB_MODE_CLUDE_AUTO_0+8;//配合显示云养程序标志位  ,不考虑云养程序，
               }*/
              
              if(mode == 11)
              {
                OutBuffer[13] |= 0x0d << 2; 
              }
              
              else
              {
                OutBuffer[13] |= (mode & 0x0f) << 2; 
              }
             
          
          
          
          
        }
        else if(nBackMainRunMode == BACK_MAIN_MODE_3D)
        {
          //OutBuffer[13] |=  0x0C << 2;  //OLD =0X0B  配合云养程序修改地址
          OutBuffer[13] |=  0X0B << 2;  //OLD =0X0B  配合云养程序修改地址
        }
        else if(nChairRunState == CHAIR_STATE_RUN)
        {
          OutBuffer[13] |=  7 << 2;
        }
      }
 	 //======bit 4   0:蓝牙  1:usb==============================
/*	 if(USB_SONG_ON == n_usb_indicate)
	 {
		OutBuffer[13] |= 0x01;
	 }
	 else if(BLUE_SONG_ON == n_usb_indicate)
	 {
		OutBuffer[13] |= 0x00;
	 }
      //-------------------------------------------------------------
      
      OutBuffer[13] |= 0x01;
      */
      if(nAxisUpdateCounter < 30)
      {
        if(nAxisUpdateCounter < 5)  OutBuffer[14] = (nKeyAxisStrength+1) & 0x07;//3D按摩力度
        else if(nAxisUpdateCounter < 10)  OutBuffer[14] = 0;
        else if(nAxisUpdateCounter < 15)  OutBuffer[14] = (nKeyAxisStrength+1) & 0x07;
        else if(nAxisUpdateCounter < 20)  OutBuffer[14] = 0;
        else if(nAxisUpdateCounter < 25)  OutBuffer[14] = (nKeyAxisStrength+1) & 0x07;
        else OutBuffer[14] = 0;
      }
      else
      {
        //OutBuffer[14] = (nFinalAxisStrength+1) & 0x07;
        OutBuffer[14] = (nKeyAxisStrength+1) & 0x07;//(nDisplayAxisStrength+1) & 0x07;//20150421 wgh 添加肩部显示
        
      } 
      
      //OutBuffer[14] = (nAxisStrength+1) & 0x07;
      /*
      unsigned short curPosition = Input_GetAxisMotorPosition();
      
      curPosition /= 8;
      if(curPosition > 4) curPosition = 4;
      OutBuffer[14] = (unsigned char)((curPosition+1) & 0x07);
      */  
      
      if(nBackSubRunMode == BACK_SUB_MODE_3D1)
      {
        OutBuffer[14] |= 1 << 3;
      }
      if(nBackSubRunMode == BACK_SUB_MODE_3D2)
      {
        OutBuffer[14] |= 2 << 3;
      }
      if(nBackSubRunMode == BACK_SUB_MODE_3D3)
      {
        OutBuffer[14] |= 3 << 3;
      }
  
      OutBuffer[15] =0;
  
      OutBuffer[15] = nStretchVigor;
      
      
      
      
     if(bRockEnable)
     {
       OutBuffer[15] |= 1<<2;
     }
     else
     {
       OutBuffer[15] |= 0<<2;
     } 
      
      if(bTapping == 1)          
      {
        OutBuffer[15] |= 1<<5; 
      }
     if(nCurSubFunction == BACK_SUB_MODE_KNEAD)
     {
       OutBuffer[15] |=  nKneadTurn<<3;
     }
     else
     {
       OutBuffer[15] |=  0<<3;
     }
     if(st_Stretch.active == true)
     {
       OutBuffer[15] |=  1<<6;
     }
     else
     {
       OutBuffer[15] |=  0<<6;
     }
      
      
      OutBuffer[16] = 0;
      OutBuffer[17] = EOI;
      nOutBufferCount = 18;
      /*
      OutBuffer[13] = EOI;
      nOutBufferCount = 14;
      */
      HandUart_Transmit_Packet(OutBuffer,nOutBufferCount);
      bMasterSendPacket = FALSE ;
    } 
    Main_Send_Leg();//主板发数据给小腿板
   // Main_MassageSignalSend();
}
//BlueTooth

void Main_BlueToothSend(void)
{
  
  unsigned char OutBufferBlueTooth[MAX_OUTBUFFER_COUNT] ;
  static unsigned char OutBufferBlueTooth_Old[MAX_OUTBUFFER_COUNT] ;
  unsigned char nOutBufferBlueToothCount;
  //return;//
  
  if(bBlueToothStatus)
       {
         BlueToothOn();
       }
        else
       {
         BlueToothOff();
         bSendBuzzerMode = FALSE ;
         return;
       }
  if(bBlueToothMasterSendPacket)
  {
    OutBufferBlueTooth[0] = SOI ;
    OutBufferBlueTooth[1] = 0;
    OutBufferBlueTooth[1] = 0x01;  //3D 标识   BIT0
#ifdef  RT8305T_1
    OutBufferBlueTooth[1] |= 0x00; //机器无伸缩小腿=1    ,当有伸缩小腿为0  ,BIT1
#else
    OutBufferBlueTooth[1] |= 0x02; //0=机器有伸缩小腿    ,当无伸缩小腿为1  ,BIT1
 #endif
    
    
    
    OutBufferBlueTooth[1] |= 0x04;  //3D 标识  //自动程序名称标志位  BIT2
    //标识 1	按摩椅运行状态 1	按摩手法 3	按摩程序 3
    if(nChairRunState == CHAIR_STATE_IDLE || nChairRunState == CHAIR_STATE_SLEEP)
    {
      OutBufferBlueTooth[1] |= 0 << 6;
    }
    else
    {
      OutBufferBlueTooth[1] |= 1 << 6;
    }
    
    /*****************************************************/
    //按摩手法显示
    switch(nCurSubFunction)
    {
      //00：停止
      //01：揉捏
      //02：敲击
      //03：揉敲同步
      //04：叩击
      //05：指压
      //06：韵律按摩
      //07：保留
    case BACK_SUB_MODE_KNEAD			:
      OutBufferBlueTooth[1] |= 1 << 3;
      break;
    case BACK_SUB_MODE_KNOCK			:
      OutBufferBlueTooth[1] |= 4 << 3;//OutBufferBlueTooth[1] |= 2 << 3;
      break;
    case BACK_SUB_MODE_WAVELET		        :
      OutBufferBlueTooth[1] |= 3 << 3;
      break;
    case BACK_SUB_MODE_SOFT_KNOCK		:
      OutBufferBlueTooth[1] |= 2 << 3;//OutBufferBlueTooth[1] |= 4 << 3;
      break;
    case BACK_SUB_MODE_PRESS			:
      OutBufferBlueTooth[1] |= 5 << 3;
      break;
    case BACK_SUB_MODE_MUSIC			:
      OutBufferBlueTooth[1] |= 6 << 3;
      break;
    default		:
      OutBufferBlueTooth[1] |= 0 << 3;
      break;
    case BACK_SUB_MODE_RUBBING:
      OutBufferBlueTooth[1] |= 7 << 3;
      break;
      break ;
    }
    /*****************************************************/
    /*
    if(nBackMainRunMode == BACK_MAIN_MODE_MANUAL)
    {
      OutBufferBlueTooth[1] |=  7;
    }
    else if(nBackMainRunMode == BACK_MAIN_MODE_AUTO)
    {
      BYTE mode = (nBackSubRunMode + 1);
      OutBufferBlueTooth[1] |= mode & 0x7;
    }
    else  if(nChairRunState == CHAIR_STATE_RUN)
    {
      OutBufferBlueTooth[1] |=  7;
    }
    */
    //标识 1 加热 1	保留 1	按摩机芯速度 3 	揉捏头宽度位置 2
    //00-03 自定义
  
    unsigned char speed;
    if(nBackMainRunMode == BACK_MAIN_MODE_IDLE)
    {
      speed = 0;
    }
    else
    {
      speed = nCurKneadKnockSpeed;
    }
    OutBufferBlueTooth[2] = ((bKeyWaistHeat & 0x1) << 6) | ((speed & 0x7) << 2) | (Input_GetKneadPosition() & 0x3);
    
    if(bRollerEnable)
    {
      OutBufferBlueTooth[2] |= (1 << 5);
    }
    else
    {
      OutBufferBlueTooth[2] &= ~(1 << 5);
    }
    // 标识 1	负离子开关 1 	 振动（或扭腰）强度 3	气压强度 3
    OutBufferBlueTooth[3] = (nKeySeatVibrateStrength & 0x7) << 3;
    
    if(nKeyAirBagLocate != AIRBAG_LOCATE_NONE)
    {
      OutBufferBlueTooth[3] |= (Valve_GetAirBagStrength() & 0x7);
    }
    //标识 1	机芯按摩部位 2	运行时间高5位 5
    //显示位置
    OutBufferBlueTooth[4] = 0;
    if(nBackMainRunMode == BACK_MAIN_MODE_AUTO)
    {
      if((nBackSubRunMode == BACK_SUB_MODE_AUTO_0) ||
         (nBackSubRunMode == BACK_SUB_MODE_AUTO_1) ||
           (nBackSubRunMode == BACK_SUB_MODE_AUTO_2) ||
             (nBackSubRunMode == BACK_SUB_MODE_AUTO_3))
      {
        OutBufferBlueTooth[4] = 1 << 5;
      }
      else
      {
        OutBufferBlueTooth[4] = 2 << 5;
      }
    }
    else
    {
      switch(nKeyBackLocate)
      {
      case LOCATE_FULL_BACK:
        OutBufferBlueTooth[4] = 1 << 5;
        break ;
      case LOCATE_PARTIAL:
        OutBufferBlueTooth[4] = 2 << 5;
        break ;
      case LOCATE_POINT:
        OutBufferBlueTooth[4] = 3 << 5; ;
        break ;
      default://include LOCATE_NONE
        //OutBufferBlueTooth[4] = 3<<5; ;
        break ;
      }
    }
    
#ifdef FORCE_CONTROLLER
    unsigned int time;
    //time = (KnockMotor_GetCurrent()&0x0f)<<4;
    time = KneadMotor_GetCurrent();
    time *= 60;
#else
    unsigned int time = Data_Get_TimeSecond();
#endif
    if(nChairRunState == CHAIR_STATE_DEMO)
    {
      time /= 60;    //demo模式 时间按分显示
    }
    OutBufferBlueTooth[4] |= (time >> 7) & 0x1f;
    //标识 1	运行时间低7位 7
    OutBufferBlueTooth[5] = time & 0x7f;

    OutBufferBlueTooth[6] = 0x00;
     if( (bLeftFootAirBagValve) | (bRightFootAirBagValve) |(bFootHeelAirBagValve))
      {
        OutBuffer[6] |= 0x01;
      }
      
      if( (bLegDownBottomAirBagValve) |(bLegDownSideAirBagValve)|(bLegUpBottomAirBagValve)|(bLegUpSideAirBagValve))
      {
       OutBuffer[6] |= 0x02;
      }
    if((bLeftThighAirBagValve) | (bRightThighAirBagValve ))
    {
      OutBufferBlueTooth[6] |= 0x04;
    }
    if((bLeftArmUpAirBagVave) | (bLeftArmDownAirBagValve) | (bRightArmDownAirBagValve) | (bRightArmUpAirBagValve) )
    {
      OutBufferBlueTooth[6] |= 0x10;
    }
    
    if(bRollerEnable)
        {
            if(Valve_RollerIsAuto())
            {
                if(Roller_GetSpeed() == ROLLER_SPEED_STOP) OutBufferBlueTooth[6] |= (0 << 5);
                else if(Roller_GetSpeed() == ROLLER_SPEED_SLOW) OutBufferBlueTooth[6] |= (1 << 5);
                else if(Roller_GetSpeed() == ROLLER_SPEED_MID) OutBufferBlueTooth[6] |= (2 << 5);
                else if(Roller_GetSpeed() == ROLLER_SPEED_FAST) OutBufferBlueTooth[6] |= (3 << 5);
            }
            else
            {
                OutBufferBlueTooth[6] |= (Roller_GetSpeed() << 5);
            }
        }
        else
        {
            OutBufferBlueTooth[6] |= (0 << 5);
        }

    OutBufferBlueTooth[7] = 0x0;
    
    if((bRightSholderAirBagValve)||(bLeftSholderAirBagValve) )
    {
      OutBufferBlueTooth[7] |=  0x10;
    }

    
    OutBufferBlueTooth[7] &= 0xf0;
    
    BYTE state = nChairRunState;
    if(nChairRunState == CHAIR_STATE_SLEEP)
    {
      state = CHAIR_STATE_IDLE;
    }
    if(nChairRunState == CHAIR_STATE_DEMO)
    {
      state = CHAIR_STATE_RUN;
    }
    OutBufferBlueTooth[7] |= (state & 0x0f);
    
    /*
    int data;
    if(nBackMainRunMode == BACK_MAIN_MODE_AUTO)
    {
    data = nFinalWalkMotorLocate; //自动模式使用肩膀位置
  }
    else
    {
    data = TOP_POSITION;   //手动模式使用自动形成
  }
    */
    unsigned int data = Input_GetWalkMotorPosition();
    data /= 31;
    if(data >= 13) data = 13;
    OutBufferBlueTooth[8] = data;
    
    if(BodyDetectStep == DETECT_SHOULDER) 
      {
        if((ShoulderSteps > BODY_DETECT_PREPARE) && (ShoulderSteps < BODY_DETECT_OVER))
        { 
          OutBufferBlueTooth[9] = 1<<6;
        }
        else
        {
          OutBufferBlueTooth[9] = 0<<6;
        }
        
        if(ShoulderSteps == BODY_DETECT_ADJ)
        {
          OutBufferBlueTooth[9] |= 1<<5;
          data = nShoulderPositionTop - nShoulderPositionBottom;
          time = data /15;
          data = (Input_GetWalkMotorPosition()-nShoulderPositionBottom) / time;
          if(data == 0) data = 1;
          if(data > 15) data = 15;
          
          OutBufferBlueTooth[9] |= data&0x0f;
          
        }
      }
   
    OutBufferBlueTooth[10] = 0;
    if(isZeroPosition())
    {
      if((bRockEnable == FALSE)&&(bZeroflash == FALSE)&&(st_Stretch.active == FALSE) )
      {
        OutBufferBlueTooth[10] = 1 << 6;
      }
      else
      {
        OutBufferBlueTooth[10] = 0 << 6;
      }
    }
    else
    {
      OutBufferBlueTooth[10] = 0 << 6;
    }
    //=================================================================
   /* if(SlideMotor_GetPower() == SLIDE_MOTOR_POWER_ON)
    {
      if(SlideMotor_GetDirection() == SLIDE_MOTOR_GO_FORWARD)
      {
        OutBufferBlueTooth[10] = 0x01 << 4;
      }
      else
      {
        OutBufferBlueTooth[10] = 0x02 << 4;
      }
    }*/
    //================================================================
    if(BackMotor_GetPower() == BACK_MOTOR_POWER_ON)
    {
      if(BackMotor_GetDirection() == BACK_MOTOR_GO_UP)
      {
        OutBufferBlueTooth[10] |= 0x01;
      }
      else
      {
        OutBufferBlueTooth[10] |= 0x02;
      }
    }
    //==================================================================
    if(LegMotor_GetPower() == LEG_MOTOR_POWER_ON)
    {
      if(LegMotor_GetDirection() == LEG_MOTOR_GO_UP)
      {
        OutBufferBlueTooth[10] |= (0x01 << 2);
      }
      else
      {
        OutBufferBlueTooth[10] |= (0x02 << 2);
      }
    }
    //================================================================================
     //标识	1 蜂鸣器模式 2 音乐开关 1	音量 4
      if(bSendBuzzerMode == TRUE)
      {
        OutBufferBlueTooth[11] = (nBuzzerMode&0x3)<<5;
        bSendBuzzerMode = FALSE ;
      }
      else
      {
        OutBufferBlueTooth[11] = 0;
      }
    
    /*
    //标识	1 蜂鸣器模式 2 音乐开关 1	音量 4
    if(bBlueToothSendBuzzerMode == TRUE)
    {
      OutBufferBlueTooth[11] = (nBuzzerMode & 0x3) << 5;
      bBlueToothSendBuzzerMode = FALSE ;
    }
    else
    {
      OutBufferBlueTooth[11] = 0;
    }
    */
//    OutBufferBlueTooth[11] |= ((nvcBluetoothPower & 0x1) << 4);
    
    switch(w_PresetTime)
    {
    case RUN_TIME_10:
      OutBufferBlueTooth[12] = 1;
      break;
    case RUN_TIME_20:
      OutBufferBlueTooth[12] = 2;
      break;
    case RUN_TIME_30:
      OutBufferBlueTooth[12] = 3;
      break;
    default:
      OutBufferBlueTooth[12] = 0;
      break;
    }
    unsigned int locate = 0;
  /*  switch(nKeyAirBagLocate)
    {
    case AIRBAG_LOCATE_NONE: break;
    case AIRBAG_LOCATE_LEG_FOOT:locate = 0x04;break;//腿脚气囊程序
  //  case AIRBAG_LOCATE_BODY_UP: locate = 0x20;break;
 //   case AIRBAG_LOCATE_BACK_WAIST: locate = 0x08; break;//背腰气囊程序
 //   case AIRBAG_LOCATE_ARM_SHOLDER: locate = 0x10; break;//臂肩气囊程序
    case AIRBAG_LOCATE_SEAT:locate = 0x20;break;//坐垫气囊程序
    case AIRBAG_LOCATE_ARM_NECK:locate = 0x10;break;
    
    
    case AIRBAG_LOCATE_AUTO:locate = 0x40;break;//全身气囊程序
   // case AIRBAG_LOCATE_ARM:break;
    }
    OutBufferBlueTooth[12] |= locate;//地址12 时间和气囊 ,为用户选中的气囊程序
    */
    OutBufferBlueTooth[12] |= ((nKeyAirBagLocate & 0x1f) <<2);//add wgh 20170208
    
    //滚轮方向
    if(bRollerEnable)
    {
     // if(ReadRollerPhase() == 1)
      if(1)
      {
        OutBufferBlueTooth[13] = 1;
      }
      /*
      else if(ReadRollerPhase() == 0)
      {
        OutBufferBlueTooth[13] = 2;
      }
      else
        OutBufferBlueTooth[13] = 0;
      */
    }
    else
    {
      OutBufferBlueTooth[13] = 0;
    }
    
     if(nBackMainRunMode == BACK_MAIN_MODE_AUTO)
    {
      BYTE mode = (nBackSubRunMode + 1);
      OutBufferBlueTooth[13] |= (mode & 0x0f) << 2;
    }
    else if(nBackMainRunMode == BACK_MAIN_MODE_3D)
    {
      OutBufferBlueTooth[13] |=  0x0B << 2;
    }
    else if(nChairRunState == CHAIR_STATE_RUN)
    {
        OutBufferBlueTooth[13] |=  7 << 2;
    }
   
    if(nAxisUpdateCounter < 30)
    {
      if(nAxisUpdateCounter < 5)  OutBufferBlueTooth[14] = (nKeyAxisStrength+1) & 0x07;
      else if(nAxisUpdateCounter < 10)  OutBufferBlueTooth[14] = 0;
      else if(nAxisUpdateCounter < 15)  OutBufferBlueTooth[14] = (nKeyAxisStrength+1) & 0x07;
      else if(nAxisUpdateCounter < 20)  OutBufferBlueTooth[14] = 0;
      else if(nAxisUpdateCounter < 25)  OutBufferBlueTooth[14] = (nKeyAxisStrength+1) & 0x07;
      else OutBufferBlueTooth[14] = 0;
    }
    else
    {
      //OutBufferBlueTooth[14] = (nFinalAxisStrength+1) & 0x07;
      OutBufferBlueTooth[14] = (nKeyAxisStrength+1) & 0x07;//(nDisplayAxisStrength+1) & 0x07;//(nDisplayAxisStrength+1) & 0x07;// 20150421 wgh 添加肩部显示
    } 
    
    if(nBackSubRunMode == BACK_SUB_MODE_3D1)
    {
      OutBufferBlueTooth[14] |= 1 << 3;
    }
    if(nBackSubRunMode == BACK_SUB_MODE_3D2)
    {
      OutBufferBlueTooth[14] |= 2 << 3;
    }
    if(nBackSubRunMode == BACK_SUB_MODE_3D3)
    {
      OutBufferBlueTooth[14] |= 3 << 3;
    }
    
    unsigned char checkSum = 0;
    for(int i=1;i<15;i++)
    {
      checkSum += OutBufferBlueTooth[i];
    }
    checkSum = ~checkSum;
    checkSum &= 0x7f;
    OutBufferBlueTooth[15] = checkSum;
    OutBufferBlueTooth[16] = EOI;
    nOutBufferBlueToothCount = 17;
    
    if(memcmp(OutBufferBlueTooth,OutBufferBlueTooth_Old,nOutBufferBlueToothCount) != 0)
    {
      BlueToothUart_Transmit_Packet(OutBufferBlueTooth, nOutBufferBlueToothCount);
      memcpy(OutBufferBlueTooth_Old,OutBufferBlueTooth,nOutBufferBlueToothCount);  
    }
    else
    {
     OutBufferBlueTooth[0] = SOI;
     unsigned short ID;
     OutBufferBlueTooth[1] = 0x85; //信息帧识别码
     ID = *(unsigned int*)CLOUD_PROGAME1_START_ADDRESS;
     if(ID == 0xffff) ID = 0;
     OutBufferBlueTooth[2] = (ID>>7)&0x7f;
     OutBufferBlueTooth[3] = (ID)&0x7f;
     ID = *(unsigned int*)CLOUD_PROGAME2_START_ADDRESS;
     if(ID == 0xffff) ID = 0;
     OutBufferBlueTooth[4] = (ID>>7)&0x7f;
     OutBufferBlueTooth[5] = (ID)&0x7f;
     ID = *(unsigned int*)CLOUD_PROGAME3_START_ADDRESS;
     if(ID == 0xffff) ID = 0;
     OutBufferBlueTooth[6] = (ID>>7)&0x7f;
     OutBufferBlueTooth[7] = (ID)&0x7f;
     ID = *(unsigned int*)CLOUD_PROGAME4_START_ADDRESS;
     if(ID == 0xffff) ID = 0;
     OutBufferBlueTooth[8] = (ID>>7)&0x7f;
     OutBufferBlueTooth[9] = (ID)&0x7f;
//     OutBufferBlueTooth[10] = EOI;   //必须去掉停止位 这样才能老的APP兼容
     nOutBufferBlueToothCount =10;// 11;
     BlueToothUart_Transmit_Packet(OutBufferBlueTooth, nOutBufferBlueToothCount);
     OutBufferBlueTooth_Old[0] = 0;  //下一帧为状态帧
    }  
    
    
    
 //   BlueToothUart_Transmit_Packet(OutBufferBlueTooth, nOutBufferBlueToothCount);
    bBlueToothMasterSendPacket = FALSE ;
  }
}

void Main_Initial_Data(void)
{
    bAxisUpdate = 1;  //上电后3D马达先归零
    
    GlobalFlags0.nByte = 0;
    GlobalFlags1.nByte = 0;
    GlobalFlags2.nByte = 0;
    GlobalFlags3.nByte = 0;
    GlobalFlags4.nByte = 0;
    GlobalFlags5.nByte = 0;
    GlobalFlags6.nByte = 0;
    GlobalFlags7.nByte = 0;
    GlobalFlags8.nByte = 0;
    GlobalFlags9.nByte = 0;
    GlobalFlags10.nByte = 0;
        

        
    
    
    unsigned int pw_Information[10];
    memset(pw_Information,0,sizeof(pw_Information));
    PBYTE pInformation = (PBYTE)pw_Information;//typedef unsigned char *     PBYTE;
    // 
    if((SOFT_MAIN_VER != ReadEEByte(USER_DATA_BASE + SOFT_MAIN_VER_ADDRESS)) || (SOFT_SECONDARY_VER != ReadEEByte(USER_DATA_BASE + SOFT_SECONDARY_VER_ADDRESS))) 
    {  //首次使用需要初始化数据
        *(pInformation + SOFT_MAIN_VER_ADDRESS) = SOFT_MAIN_VER;
        *(pInformation + SOFT_SECONDARY_VER_ADDRESS) = SOFT_SECONDARY_VER;
        *(pInformation + SETTLE_ADDRESS) = MEMORY_DEFAULT_SETTLE;     
        *(pInformation + AIRBAG_STRETCH_ADDRESS) = MEMORY_DEFAULT_AIR;          //按摩椅内部气囊力度     //
        *(pInformation + SLIDE_MOTOR_ENABLE_ADDRESS) = SLIDE_DEFAULT_ENABLE; 
        *(pInformation + DEFAULT_TIME_ADDRESS) = RUN_TIME_20/60; //默认运行时间20分钟
        *(pInformation + BLUETOOTH_STATUS_ADDRESS) = BLUETOOTH_STATUS_DEFAULT; 
        MEM_Write_Memory(pw_Information,7*2);
        xmodem__Erase_Block(CLOUD_PROGAME1_START_ADDRESS,CLOUD_PROGAME4_END_ADDRESS);
      __no_operation();
      __no_operation();      
      __no_operation();
      __no_operation();
      __no_operation();
      __no_operation();   
      
      
    }
 //   by_t = *(pInformation + AIRBAG_STRETCH_ADDRESS);
        //           by_t = *(pInformation + AIRBAG_STRETCH_ADDRESS);
  //by_t =    ReadEEByte(AIRBAG_STRETCH_ADDRESS + USER_DATA_BASE);//READ 气囊力度
    
    
      __no_operation();
      __no_operation();
      __no_operation();
      __no_operation();
      __no_operation();
      __no_operation();
      __no_operation();
      __no_operation();
      __no_operation();
      __no_operation();      
      __no_operation();
      __no_operation();
      __no_operation();
      __no_operation();   
    
    
    //test
    /*
    printf("Main ver:%d\n",ReadEEByte(USER_DATA_BASE + SOFT_MAIN_VER_ADDRESS));
    printf("Sec ver:%d\n",ReadEEByte(USER_DATA_BASE + SOFT_SECONDARY_VER_ADDRESS));
    printf("default time:%d\n",ReadEEByte(USER_DATA_BASE + DEFAULT_TIME_ADDRESS)*60);
    printf("bluetooth status:%d\n",ReadEEByte(USER_DATA_BASE + BLUETOOTH_STATUS_ADDRESS));
    printf("Slide:%d\n",ReadEEByte(USER_DATA_BASE + SLIDE_MOTOR_ENABLE_ADDRESS));
    printf("air:%d\n",ReadEEByte(USER_DATA_BASE + AIRBAG_STRETCH_ADDRESS));
    printf("settle:%d\n",ReadEEByte(USER_DATA_BASE + SETTLE_ADDRESS));
    */
    //拉退程序依据行程开关控制
    st_Stretch.mode = STRETCH_MODE_SWITCH;
    st_Stretch.PresetTime = 200;//200*0.1sec=20sec
    st_Stretch.active = false;
    bRockEnable = false;
    nRockModeEnterEnable = ExitRock;
   
    

    
    st_AirBagLegFoot.pAirBagArray = AirBagModeLegFoot;
    st_AirBagLegFoot.nTotalSteps = sizeof(AirBagModeLegFoot)/sizeof(struct AirBagStruct);
    st_AirBagLegFoot.locate = AIRBAG_LOCATE_LEG_FOOT;
    

    
    st_AirBagSeat.pAirBagArray = AirBagModeSeat;  //坐垫臀部
    st_AirBagSeat.nTotalSteps = sizeof(AirBagModeSeat)/sizeof(struct AirBagStruct);
    st_AirBagSeat.locate = AIRBAG_LOCATE_SEAT;
    

    
    

    
    //20170205 WGH
    /*
    //---------------------------------------------------
    st_AirBag_Neck.pAirBagArray = AirBagModeNeck;
    st_AirBag_Neck.nTotalSteps = sizeof(AirBagModeNeck)/sizeof(struct AirBagStruct);
    st_AirBag_Neck.locate = AIRBAG_LOCATE_NECK;
    //-------------------------------------------------------
    */
   
    st_AirBagArmNeck.pAirBagArray = AirBagModeArmSholder;//AirBagModeArmNeck;   //臂颈气囊.实际使用的是臂肩气囊
    st_AirBagArmNeck.nTotalSteps = sizeof(AirBagModeArmSholder)/sizeof(struct AirBagStruct);
    st_AirBagArmNeck.locate = AIRBAG_LOCATE_ARM_NECK;
    //----------------------------------
    
    st_AirBagAuto.pAirBagArray = AirBagModeAuto;
    st_AirBagAuto.nTotalSteps = sizeof(AirBagModeAuto)/sizeof(struct AirBagStruct);
    st_AirBagAuto.locate = AIRBAG_LOCATE_AUTO;
        
st_AirBagLegFoot_Seat.pAirBagArray     = AirBagModeLegFoot_Seat;
st_AirBagLegFoot_Seat.nTotalSteps = sizeof(AirBagModeLegFoot_Seat)/sizeof(struct AirBagStruct);
st_AirBagLegFoot_Seat.locate = AIRBAG_LOCATE_LEG_FOOT|AIRBAG_LOCATE_SEAT;

st_AirBagLegFoot_Arm.pAirBagArray      = AirBagModeLegFoot_Arm;
st_AirBagLegFoot_Arm.nTotalSteps = sizeof(AirBagModeLegFoot_Arm)/sizeof(struct AirBagStruct);
st_AirBagLegFoot_Arm.locate = AIRBAG_LOCATE_LEG_FOOT|AIRBAG_LOCATE_ARM_NECK;

st_AirBagArm_Seat.pAirBagArray         = AirBagModeArm_Seat;
st_AirBagArm_Seat.nTotalSteps = sizeof(AirBagModeArm_Seat)/sizeof(struct AirBagStruct);
st_AirBagArm_Seat.locate = AIRBAG_LOCATE_ARM_NECK|AIRBAG_LOCATE_SEAT;

st_AirBagLegFoot_Arm_Seat.pAirBagArray = AirBagModeLegFoot_Arm_Seat;
st_AirBagLegFoot_Arm_Seat.nTotalSteps = sizeof(AirBagModeLegFoot_Arm_Seat)/sizeof(struct AirBagStruct);
st_AirBagLegFoot_Arm_Seat.locate = AIRBAG_LOCATE_LEG_FOOT|AIRBAG_LOCATE_ARM_NECK|AIRBAG_LOCATE_SEAT;    
 

//
    st_AirBagAuto_Upbody.pAirBagArray = AirBagModeAuto_Upbody;
    st_AirBagAuto_Upbody.nTotalSteps = sizeof(AirBagModeAuto_Upbody)/sizeof(struct AirBagStruct);
    st_AirBagAuto_Upbody.locate = AIRBAG_LOCATE_AUTO;
      
    //bKneckCheckSwitchLast = Input_GetVout();
    
    //Back Variables
    nBackMainRunMode = BACK_MAIN_MODE_IDLE ;//机芯状态为空闲状态
    nKeyBackLocate = LOCATE_NONE;   //全程，局部，定点定位标志
    nKeyKneadWidth = KNEAD_WIDTH_UNKNOWN ;
    nKeyKneadKnockSpeed = SPEED_0 ;
    //Walk Motor Variables
    bWalkMotorInProcess = FALSE ;
    nWalkMotorControlParam1 = WALK_LOCATE_PARK ;
    nWalkMotorControlParam2 = 0 ;
    bUpdateLocate = TRUE ;     //行走电机坐标更新标志，置位时更新一次坐标
    nShoulderPosition = DEFAULT_SHOULDER_POSITION ;
    BodyDataRefresh() ;
    nKneadMotorControlParam1 = KNEAD_STOP ;
    nFinalKneadMotorState = STATE_IDLE ;
    //nLastKneadPosition = KNEAD_WIDTH_UNKNOWN ;
   // nCurBackPadMotorState = STATE_IDLE ;
   // nCurLegPadMotorState = STATE_IDLE ;
    
    nKeyAirBagLocate = AIRBAG_LOCATE_NONE ;
    nFinalWalkMotorLocate = TOP_POSITION;
   // bRunTimeChange = TRUE ;
    
    ////bMP3_AD_Enable = FALSE;
    //Communication
    bMasterSendPacket = FALSE ;  
    nSendPacketID = PACKET_MASTER_GET_COMMAND ;
    nBuzzerMode = BUZZER_MODE_OFF ;
    bSendBuzzerMode = TRUE ;
    
    nTargetMassagePosition = MASSAGE_RESET_POSITION;   //目标按摩位置
    WorkStep = 0;
     w_ZeroPosition = 0;//零重力电机归位
bZeroflash = FALSE;
    
    bMassagePositionUpdate = FALSE;
    
    w_PresetTime = ReadEEByte(USER_DATA_BASE + DEFAULT_TIME_ADDRESS)*60;//按摩预设时间

    bBlueToothStatus = ReadEEByte(USER_DATA_BASE + BLUETOOTH_STATUS_ADDRESS);//蓝牙默认状态为打开
     
    //滚轮数据初始化
    //nRollerPWM = 0;
    bRollerEnable = false;
    //扭腰数据初始化
    nKeySeatVibrateStrength=0;//nKeySeatVibrateStrength_old
    bKeySeatVibrate=0;
    nKeySeatVibrateStrength_old=0;
    
  //  Enable_Vibrate=0;
    //==============================
    
    
    
    Data_Init();
    LEUART0_Initial_Data();////读取3D机芯3D脉冲和状态信号
    
    memset(OutBuffer,0,sizeof(OutBuffer))  ;
    memset(InBuffer,0,sizeof(InBuffer))  ;
    
    memset(&st_Stretch,0,sizeof(StretchStruct));
    
    Valve_Initial_Data();
    nCurActionStep = 0;
    
    ShoulderSteps = BODY_DETECT_OVER;
    BodyDetectStep = DETECT_NO_START;
    
    Timer_Initial();
    
    nKeyAxisStrength = 0;//
    nVoicekey = H10_KEY_NONE;
  _3D_Max_Position = 38; 
  _3D_More_Men_Position = 30; 
  _3D_Men_Position = 20; 
  _3D_More_Min_Position = 10; 
  _3D_Min_Position = 2; 
    //----------------------------------
    CloudProgrameInit();
    
     bShoulderOK =0;
          
   // BodyDetectStep = DETECT_INITIAL;
    n_usb_indicate = BLUE_SONG_ON;
    n_mp3_key = H10_KEY_NONE;
    n_mp3_key_old = H10_KEY_NONE;
    //nvcBluetoothPower = 1;
    
    bTapping =1;
    nKneadTurn = 0;
    
    //
    Input_SetWalkMotorPosition(TOP_POSITION-30);//4
    
    
    //20170208WGH
    if(bEnableStretchDemoRun == TRUE)
    {
      bEnableStretchDemoRun = FALSE;//WGH 20161107
    }  

       nStretchVigor=3;    
    
    
}

unsigned char Main_GetKey(void)
{
 // static int count = 0 ;
   unsigned char by_Key = H10_KEY_NONE;
   
   
    if(HandUart_GetRXStatus() == TRUE)
    {
        HandUart_ClearRXStatus();
  //      VoiceUart_ClearRXStatus();
        by_Key = HandUart_GetKey();
        HandUart_SetKey(H10_KEY_NONE);
        return by_Key;
    }
    
  /*  if(VoiceUart_GetRXStatus() == TRUE)  语音控制来接收按键
    {
        VoiceUart_ClearRXStatus();
        //printf("%d:[%d]\n",count++,VoiceUart_GetKey());
        switch (VoiceUart_GetKey())
       {
       case 0x01: by_Key = H10_KEY_CHAIR_AUTO_1 | VOICE_KEY_MASK;   BlueToothUart_AMP_Volume_On();break;  
       case 0x02: by_Key = H10_KEY_CHAIR_AUTO_3 | VOICE_KEY_MASK;   BlueToothUart_AMP_Volume_On();break;
       case 0x03: by_Key = H10_KEY_CHAIR_AUTO_0 | VOICE_KEY_MASK;   BlueToothUart_AMP_Volume_On();break;
       case 0x04: by_Key = H10_KEY_CHAIR_AUTO_2 | VOICE_KEY_MASK;   BlueToothUart_AMP_Volume_On();break;
       case 0x05: by_Key = H10_KEY_CHAIR_AUTO_4 | VOICE_KEY_MASK;   BlueToothUart_AMP_Volume_On();break;
       case 0x06: by_Key = H10_KEY_CHAIR_AUTO_5 | VOICE_KEY_MASK;   BlueToothUart_AMP_Volume_On();break;
       case 0x07: by_Key = H10_KEY_AIRBAG_AUTO | VOICE_KEY_MASK;   BlueToothUart_AMP_Volume_On();break;  
       case 0x08: by_Key = H10_KEY_3DMODE_1 | VOICE_KEY_MASK;   BlueToothUart_AMP_Volume_On();break;  
       case 0x09: by_Key = H10_KEY_3DMODE_2 | VOICE_KEY_MASK;   BlueToothUart_AMP_Volume_On();break;  
       case 0x0a: by_Key = H10_KEY_3DMODE_3 | VOICE_KEY_MASK;   BlueToothUart_AMP_Volume_On();break;  
       case 0x0b: by_Key = H10_KEY_POWER_SWITCH | VOICE_KEY_MASK;   BlueToothUart_AMP_Volume_On();break;  
       case 0x20: by_Key = H10_KEY_VOICE_OFF;   BlueToothUart_AMP_Volume_Off();break;
       case 0x21: by_Key = H10_KEY_VOICE_ON;   BlueToothUart_AMP_Volume_On();break;
       default: break;
       }
        return by_Key;
    }*/
    
  
   if(BlueToothUart_GetRXStatus() == TRUE)
    {
      BlueToothUart_ClearRXStatus();
      by_Key = BlueToothUart_GetKey();
      return by_Key;
    }
   return by_Key;
}

void Main_Walk_Beep_Proce(void)
{
 
  if(bKeyWalkUp == TRUE)
  {
 
    if(((nBackMainRunMode == BACK_MAIN_MODE_MANUAL)||(nBackMainRunMode == BACK_MAIN_MODE_3D) )&& ((nKeyBackLocate == LOCATE_POINT)||(nKeyBackLocate == LOCATE_PARTIAL)))
    {
      if(Input_GetWalkUpSwitch() == REACH_WALK_LIMIT)
      {
        //设置连续蜂鸣器声音
        nBuzzerMode = BUZZER_MODE_FAST ;
        bSendBuzzerMode = TRUE ;
      }
      else
      {
        nBuzzerMode = BUZZER_MODE_SLOW ;
        bSendBuzzerMode = TRUE ;
      }
    }
    else if(/*(nBackMainRunMode == BACK_MAIN_MODE_AUTO) && */(ShoulderSteps == BODY_DETECT_ADJ))
    {
      if(Input_GetWalkMotorPosition() >= nShoulderPositionTop - 3)
      {
        //设置连续蜂鸣器声音
        nBuzzerMode = BUZZER_MODE_FAST ;
        bSendBuzzerMode = TRUE ;
      }
      else
      {
        nBuzzerMode = BUZZER_MODE_SLOW ;
        bSendBuzzerMode = TRUE ;
      }
    }
  }
  if(bKeyWalkDown == TRUE)
  {
 
    if(((nBackMainRunMode == BACK_MAIN_MODE_MANUAL)||(nBackMainRunMode == BACK_MAIN_MODE_3D) )&& ((nKeyBackLocate == LOCATE_POINT)||(nKeyBackLocate == LOCATE_PARTIAL)))
    {
      //设置连续蜂鸣器声音
      if(Input_GetWalkDownSwitch() == REACH_WALK_LIMIT)
      {
        //设置连续蜂鸣器声音
        nBuzzerMode = BUZZER_MODE_FAST ;
        bSendBuzzerMode = TRUE ;
      }
      else
      {
        nBuzzerMode = BUZZER_MODE_SLOW ;
        bSendBuzzerMode = TRUE ;
      }
    }
    else if(/*(nBackMainRunMode == BACK_MAIN_MODE_AUTO) &&*/ (ShoulderSteps == BODY_DETECT_ADJ))
    {
      if(Input_GetWalkMotorPosition() <= nShoulderPositionBottom + 3)
      {
        //设置连续蜂鸣器声音
        nBuzzerMode = BUZZER_MODE_FAST ;
        bSendBuzzerMode = TRUE ;
      }
      else
      {
        nBuzzerMode = BUZZER_MODE_SLOW ;
        bSendBuzzerMode = TRUE ;
      }
    }
  }
}
//停止所有的执行装置
void Main_Stop_All(void)
{
    WaistHeat_Off();
    WalkMotor_Control(STATE_WALK_IDLE, 0);
    KneadMotor_Control(STATE_KNEAD_IDLE, KNEAD_SPEED0_PWM);
    LegMotor_Control(STATE_LEG_IDLE);
    BackMotor_Control(STATE_BACK_IDLE);
    SlideMotorControl(STATE_SLIDE_IDLE);
    Flex_SetDirection(FLEX_MOTOR_STOP);
    KnockMotor_Set_Pwm_Data(0);
    
    LED_RGB_Set_All(0);
    AxisMotor_Control(STATE_AXIS_IDLE,0,_3D_SPEED_10);
    Valve_CloseAll();
    LegKnead_SetPower(LEG_KNEAD_OFF);
    Roller_SetSpeed(ROLLER_SPEED_STOP);
    
 //   Main_WaveMotorStop();//摇摆电机停止
//     Waveringly_Set_Pwm_Data(0);
    
    Waveringly_Set_Pwm_TEST_Data(0);
    
}

void engineering_stop_all(void)
{
 Main_Stop_All(); 
}

BITS engineerData1old;
#define walk_up_old         engineerData1old.bD0
#define walk_down_old       engineerData1old.bD1
#define shoulder_detect_old engineerData1old.bD2
#define knead_width_min_old engineerData1old.bD3
#define knead_width_mid_old engineerData1old.bD4
#define knead_width_max_old engineerData1old.bD5
#define back_up_old         engineerData1old.bD6
#define back_down_old       engineerData1old.bD7

BITS engineerData2old;
#define leg_up_old          engineerData2old.bD0
#define leg_down_old        engineerData2old.bD1
#define test_finish         engineerData2old.bD2
#define foot_Switch_old     engineerData2old.bD3
#define _3D_Switch_Forward  engineerData2old.bD4
#define _3D_Switch_Back     engineerData2old.bD5
#define _3D_Switch_Pluse    engineerData2old.bD6


//#define test_finish     engineerData2old.bD2
BITS engineerData1;
#define walk_up         engineerData1.bD0
#define walk_down       engineerData1.bD1
#define shoulder_detect engineerData1.bD2
#define knead_width_min engineerData1.bD3
#define knead_width_mid engineerData1.bD4
#define knead_width_max engineerData1.bD5
#define back_up         engineerData1.bD6
#define back_down       engineerData1.bD7
BITS engineerData2;
#define leg_up          engineerData2.bD0
#define leg_down        engineerData2.bD1
#define has_leg         engineerData2.bD2
#define knock           engineerData2.bD3
#define roller          engineerData2.bD4
#define heat            engineerData2.bD5
#define has_heat        engineerData2.bD6
#define air_bag         engineerData2.bD7

BITS engineerData5;
#define slide_backward  engineerData5.bD0
#define slide_forward   engineerData5.bD1
#define flex_up         engineerData5.bD2
#define flex_down       engineerData5.bD3
#define foot_Switch     engineerData5.bD4   //脚底开关
//#define leg_angle       engineerData5.bD5   //角度开关
//#define leg_ground      engineerData5.bD6//触地开关


#define knead_phase     engineerData5.bD7


BITS engineerData6;
 #define      leg_angle_Switch_old        engineerData6.bD0
 #define      leg_groud_switch_old        engineerData6.bD1
    
 #define      leg_angle                   engineerData6.bD2   //角度开关
 #define      leg_ground                  engineerData6.bD3//触地开关                  
        
 
                  




typedef union
{
    struct
    {
        unsigned bD0: 2 ;
        unsigned bD1: 2 ;
        unsigned bD2: 2 ;
        unsigned bD3: 2 ;
    } ;
    unsigned char nByte ;
} BITS2 ;
BITS2 engineerData3;
#define walk_check_count     engineerData3.bD0
#define shoulder_check_count engineerData3.bD1
#define knead_check_count    engineerData3.bD2
#define back_check_count     engineerData3.bD3

BITS2 engineerData4;
#define leg_check_count      engineerData4.bD0
//#define waver_check          engineerData4.bD1      //摇摆电机检测



#define TIME_COUNT      100
//此函数执行完毕会引起CPU复位
void Main_Engineering(void)
{
    //unsigned short nLegAngleOld;
    int leg_flex_step = 0;
    int slide_step = 0;
    has_heat = 1;
    has_leg = 1;
    heat = 1;
    knock = 1;
    roller = 1;
    //bool strengthMode,sleepMode;
    unsigned int back_position, walk_position;
    unsigned char oneKeyStep = 0, oneKeyStepLength = 4, enAirbagStep;
    unsigned char oneKeyLegCountDown = 0;
    unsigned int engineerTimeCount = 0, air_bagTimeCount = 0;
    unsigned short /*adcWalkCurrent,*/adcAxisCurrent/*,adcKnockCurrent*/,adc24,adcVcc,adc24_1,tempture;
    //unsigned int counter = 0;
  int engStatus = LINGO_ENG;//for test
    unsigned int overCounter = 0;
    bool status = true;
    bool bProgram = false;
   char lingo;
    bool bHeat = false;
    
    
    bool bwaverCheck_First=1;
    
   bool bwaverCheck_Sencod=0;
    
   bool bwaverCheck_First2=0;
   
   bool waver_check=0;
   
   
    char command;
    unsigned char PWM = 0;
  char airbagIndex = 1,airpumpIndex = 7;
    unsigned int airbag;
    unsigned int pw_Information[10];
    unsigned char strength;
    unsigned char rollerSpeed = 0;
    unsigned char rollerPhase = 0;
    unsigned char legKneadSpeed = 0;
    unsigned char legKneadPhase = 0;
    unsigned char color;
    unsigned char kneadSpeed = 0;
    unsigned char kneadPhase = 0;
    
  //  unsigned char waverSpeed=2;//设定初始摇摆速度为2
    
   // unsigned char nWaverCounter=0;//设定摇摆电机回中位次数
    
    
    
    bool bUpKey = false;
    bool bDownKey = false;
    
    
       bool bRightKey = false;
       bool bLeftKey  = false;
    
    Power_All_On();
    IndicateLED_On();
    ADC_Get_Voltage(ADC_VCC,&adcVcc); 
    ADC_Get_Voltage(ADC_V24,&adc24); 
    ADC_Get_Voltage(ADC_V24_1,&adc24_1); 
    tempture = ADC_Get_Inttemp();
    memset(pw_Information,0,sizeof(pw_Information));
    PBYTE pInformation = (PBYTE)pw_Information;
    MEM_Read_Memory(pw_Information,MEMORY_LENGTH_OF_BYTES);
    
   // BlueToothEnterCmdMode();  //蓝牙模块工作在命令模式
    
  //  BlueToothUart_GetName();
    
     nKeySeatVibrateStrength = 0 ;//设定初始摇摆速度为2
    
         waver_check=0;  //摇摆电机检测
         //nWaverCounter=0;//摇摆电机检测计数
       Timer_Initial();
     
     
   //   Valve_Test_Set_Data(0x0101);//
    //while(1)
    //{
  
    
   // Valve_Send_Data();
  // }    
    
    while(status)
    { 
        //WDOG_Feed();
        lingo = Main_GetKey();//读取受控器按键值
        
        switch(lingo)
        {
        case LINGO_AIRBAG:    //气阀测试
            {
                engStatus = LINGO_AIRBAG;
                command = HandUart_GetExternKey();
                switch(command)
                {
                case SYS_KEY_UP: 
                    airbagIndex &= 0x7f;
                    airbagIndex++;
                    airbagIndex %= 24;
                    break;
                case SYS_KEY_DOWN:
                    airbagIndex &= 0x7f;
                    airbagIndex--;
                    if(airbagIndex > 24)
                        airbagIndex = 23;
                    break;
                case SYS_KEY_LEFT://airpumpIndex++;
               // airpumpIndex &= 0x03;
                  airpumpIndex=0;
                break;
                case SYS_KEY_RIGHT:
                  //  airpumpIndex--;
                 //   airpumpIndex &= 0x03;
                   airpumpIndex=7;
                    break; 
                case SYS_KEY_ENTER:
                    airbagIndex |= 0x80;
                    break;   
                }
            }
            break;
    //用滚轮测试程序代替小腿揉搓程序    
        case LINGO_ROLLER_TEST: //脚底滚轮测试
            {
                engStatus = LINGO_ROLLER_TEST;
                command = HandUart_GetExternKey();
                switch(command)
                {
                case SYS_KEY_UP: 
                    if(rollerSpeed < 3)
                        rollerSpeed++;
                    break;
                case SYS_KEY_DOWN:
                    if(rollerSpeed > 0)
                        rollerSpeed--;
                    break;
                case SYS_KEY_LEFT:
                case SYS_KEY_RIGHT:  
                    if(rollerPhase == 0)
                        rollerPhase = 1;
                    else
                        rollerPhase = 0;
                    break;
                case SYS_KEY_ENTER:
                    break;   
                }
            }
            break; 
        
            
        case LINGO_LEG_KNEAD_TEST:
            {
                engStatus = LINGO_LEG_KNEAD_TEST;
                command = HandUart_GetExternKey();
                switch(command)
                {
                case SYS_KEY_UP: 
                    if(legKneadSpeed < 3)
                        legKneadSpeed++;
                    break;
                case SYS_KEY_DOWN:
                    if(legKneadSpeed > 0)
                        legKneadSpeed--;
                    break;
                case SYS_KEY_LEFT:
                case SYS_KEY_RIGHT:  
                    if(legKneadPhase == 0)
                        legKneadPhase = 1;
                    else
                        legKneadPhase = 0;
                    break;
                case SYS_KEY_ENTER:
                    break;   
                }
            }
            break; 
            
        case LINGO_SLIDE_TEST:    //零重力电动缸测试
            {
                engStatus = LINGO_SLIDE_TEST;
                command = HandUart_GetExternKey();
                switch(command)
                {
                case SYS_KEY_UP: 
                    bUpKey = true;
                    bDownKey = false;
                    break;
                case SYS_KEY_UP_RELEASE: 
                case SYS_KEY_DOWN_RELEASE: 
                    bUpKey = false;
                    bDownKey = false;
                    break;
                case SYS_KEY_DOWN:
                    bUpKey = false;
                    bDownKey = true;
                    break;
                }
            }
            break;  
            
        case LINGO_WAVER_TEST  : //摇摆电动缸测试
            /* {
                engStatus = LINGO_WAVER_TEST;
                command = HandUart_GetExternKey();
                switch(command)
                {
                case SYS_KEY_UP: 
                  //  bUpKey = true;
                 //   bDownKey = false;
                     if( nKeySeatVibrateStrength<3)
                      
                      nKeySeatVibrateStrength++;
                    else 
                       nKeySeatVibrateStrength=3;
                  
                  
                  
                    break;
              //  case SYS_KEY_UP_RELEASE: 
              //  case SYS_KEY_DOWN_RELEASE: 
                   // bUpKey = false;
                  //  bDownKey = false;
                    break;
                case SYS_KEY_DOWN:
                  //  bUpKey = false;
                  //  bDownKey = true;
                        if(nKeySeatVibrateStrength>0)
                        
                        nKeySeatVibrateStrength--; 
                  
                    break;
                    


                    
                    
                }
            }*/
            break;           
            
            
            
        case LINGO_BACK_TEST:     //靠背电机测试
            {
                engStatus = LINGO_BACK_TEST;
                command = HandUart_GetExternKey();
                switch(command)
                {
                case SYS_KEY_UP: 
                    bUpKey = true;
                    bDownKey = false;
                    break;
                case SYS_KEY_UP_RELEASE: 
                case SYS_KEY_DOWN_RELEASE: 
                    bUpKey = false;
                    bDownKey = false;
                    break;
                case SYS_KEY_DOWN:
                    bUpKey = false;
                    bDownKey = true;
                    break;
                }
            }
            break;
        case LINGO_LEG_TEST:   //小腿上下测试
            {
                engStatus = LINGO_LEG_TEST;
                command = HandUart_GetExternKey();
                switch(command)
                {
                case SYS_KEY_UP: 
                    bUpKey = true;
                    bDownKey = false;
                    break;
                case SYS_KEY_UP_RELEASE: 
                case SYS_KEY_DOWN_RELEASE: 
                    bUpKey = false;
                    bDownKey = false;
                    break;
                case SYS_KEY_DOWN:
                    bUpKey = false;
                    bDownKey = true;
                    break;
                }
            }
            break;   
            
          case LINGO_ONE_KEY_TEST:
          {
            engStatus = LINGO_ONE_KEY_TEST;
            command = HandUart_GetExternKey();
            engineerTimeCount = 1, air_bagTimeCount = 1; //清时间，不设零防止跳到下一步
   //           Waveringly_Set_Pwm_TEST_Data(VIB_STRENGTH[2]); 
            switch(command)
            {
            case SYS_KEY_UP://上一步
                if(oneKeyStep > 1)oneKeyStep--;
                else oneKeyStep = oneKeyStepLength;
                if(test_finish && oneKeyStep == 0)oneKeyStep = oneKeyStepLength;//// 一键自动测试=oneKeyStepLength = 4  总共分4步 ，={ 1=heat.2=knock, 3=roller, 4=air_ba   }
                break;
            case SYS_KEY_DOWN://下一步
                if(oneKeyStep < oneKeyStepLength)oneKeyStep++;
                else oneKeyStep = 0;
                break;
            case SYS_KEY_LEFT://开，并且下一步(或气囊：上一步)
                switch(oneKeyStep)
                {
                case 1:
                    if(has_heat)heat = 1;
                    if(oneKeyStep < oneKeyStepLength)oneKeyStep++;
                    else oneKeyStep = 0;
                    break;
                case 2:
                    knock = 1;
                    if(oneKeyStep < oneKeyStepLength)oneKeyStep++;
                    else oneKeyStep = 0;
                    break;
                case 3:
                    roller = 1;
                    if(oneKeyStep < oneKeyStepLength)oneKeyStep++;
                    else oneKeyStep = 0;
                    break;
                case 4:
                    air_bag = 1;
                    if(enAirbagStep > 0)
                    {
                        enAirbagStep--;
                    }
                    else
                    {
                        enAirbagStep = 24;
                    }
                    break;
                }
                break;
            case SYS_KEY_RIGHT://关，并且下一步(或气囊：下一步)
                switch(oneKeyStep)
                {
                case 1:
                    if(has_heat)heat = 0;
                    if(oneKeyStep < oneKeyStepLength)oneKeyStep++;
                    else oneKeyStep = 0;
                    break;
                case 2:
                    knock = 0;
                    if(oneKeyStep < oneKeyStepLength)oneKeyStep++;
                    else oneKeyStep = 0;
                    break;
                case 3:
                    roller = 0;
                    if(oneKeyStep < oneKeyStepLength)oneKeyStep++;
                    else oneKeyStep = 0;
                    break;
                case 4:
                    air_bag = 1;
                    if(enAirbagStep < 24)
                    {
                        enAirbagStep++;
                    }
                    else
                    {
                        enAirbagStep = 0;
                    }
                    break;
                }
                break;
            case 248://菜单中进入此界面
                //初始化
                engineerData1old.nByte = 0;
                engineerData2old.nByte = 0;
                engineerData3.nByte = 0;
                engineerData4.nByte = 0;
                engineerData6.nByte=0;
             
                oneKeyStep = 0;
                enAirbagStep = 0;
                test_finish = 0;
                heat = 1;
                knock = 1;
                roller = 1;
                air_bag = 1;
                walk_up = 0;
                walk_down = 0;
                shoulder_detect = 0;
                knead_width_min = 0;
                knead_width_mid = 0;
                knead_width_max = 0;
                leg_up = 0;
                leg_down = 0;
                back_up = 0;
                back_down = 0;
                
                
                //nWaverCounter=0;//摇摆电机检测计数
              
                
                back_position = 0;
                walk_position = 0;
                engineering_stop_all();
                engineerData5.nByte = 0;
                
                
                bwaverCheck_First=1;
                bwaverCheck_Sencod=0;
                bwaverCheck_First2=0;
                waver_check=0;  //摇摆电机检测
                nKeySeatVibrateStrength=2;//摇摆电机速度
                
                leg_flex_step = 0;
                slide_step = 0;

                
                 if(nFlexStatus&0x20) //碰到角度开关
                 {
                   
                   leg_angle_Switch_old=1;
                 }
                 else
                 {
                    leg_angle_Switch_old=0;
                 }
                 
                 
                if(nFlexStatus&0x40)//碰到地面
                {
                  leg_groud_switch_old=1;
                  
                }
                else
                {
                  leg_groud_switch_old=0;//leg_angle_Switch_old
                  
                }
                
                
                
                if(nFlexStatus&0x04) //脚底碰到脚底开关信号
                {
                  foot_Switch_old = 1;
                }
                else
                {
                  foot_Switch_old = 0;
                }
                shoulder_detect_old = Input_GetVout();//肩位检测
                
                break;
            case 15:
                engineering_stop_all();
                oneKeyStep = 0;
                break;
            default:
                break;
            }
        }
        break;   
            
        case LINGO_HEAT_TEST:  //加热测试
            {
                engStatus = LINGO_HEAT_TEST;
                command = HandUart_GetExternKey();
                switch(command)
                {
                case SYS_KEY_ENTER:
                    if(bHeat) 
                    {
                        bHeat = 0;
                        WaistHeat_Off();
                    }
                    else
                    {
                        bHeat = 1;
                        WaistHeat_On();
                    } 
                    break;
                }
            }
            break;

        case LINGO_FLEX_TEST:       //小腿伸缩测试
            {
                engStatus = LINGO_FLEX_TEST;
                command = HandUart_GetExternKey();
                switch(command)
                {
                case SYS_KEY_UP: 
                    bUpKey = true;
                    bDownKey = false;
                    break;
                case SYS_KEY_UP_RELEASE: 
                case SYS_KEY_DOWN_RELEASE: 
                    bUpKey = false;
                    bDownKey = false;
                    break;
                case SYS_KEY_DOWN:
                    bUpKey = false;
                    bDownKey = true;
                    break;
                }
            }
            break;

        case LINGO_WALK_TEST:   //行走电电机测试
            {
                engStatus = LINGO_WALK_TEST;
                command = HandUart_GetExternKey();
                switch(command)
                {
                case SYS_KEY_UP: 
                    bUpKey = true;
                    bDownKey = false;
                    break;
                case SYS_KEY_UP_RELEASE: 
                case SYS_KEY_DOWN_RELEASE: 
                    bUpKey = false;
                    bDownKey = false;
                    break;

                  
                  
                case SYS_KEY_DOWN:
                    bUpKey = false;
                    bDownKey = true;
                    break;
                    
                 case SYS_KEY_LEFT_RELEASE:
                 case SYS_KEY_RIGHT_RELEASE:
                      bRightKey=  false;
                      bLeftKey= false;
 
 
                     break;               
                 case SYS_KEY_LEFT:
                        bRightKey=  false;
                        bLeftKey= true;
                        break;
                 case SYS_KEY_RIGHT:   
                        bLeftKey= false;
                        bRightKey= true;
                        break;
                    
                    
                }
            }
            break;     

         case LINGO_3D_TEST:    //3D镜向电机测试
            {
                engStatus = LINGO_3D_TEST;
                command = HandUart_GetExternKey();
                switch(command)
                {
                case SYS_KEY_UP: 
                    bUpKey = true;
                    bDownKey = false;
                    break;
                case SYS_KEY_UP_RELEASE: 
                case SYS_KEY_DOWN_RELEASE: 
                    bUpKey = false;
                    bDownKey = false;
                    break;
                case SYS_KEY_DOWN:
                    bUpKey = false;
                    bDownKey = true;
                    break;
                    
                 case SYS_KEY_LEFT_RELEASE:
                 case SYS_KEY_RIGHT_RELEASE:
                     bLeftKey= false;
                     bRightKey=  false;
      
                     break;               
                 case SYS_KEY_LEFT:
                        bRightKey=  false;
                        bLeftKey= true;//bRightKey
                        break;
                 case SYS_KEY_RIGHT:   
                        bLeftKey= false;
                        bRightKey= true;
                        break;                 
                    
                    
                    
                    
                }
            }
            break;     
            
        case LINGO_LED_TEST:     //LED彩灯测试
            {
                engStatus = LINGO_LED_TEST;
                command = HandUart_GetExternKey();
                switch(command)
                {
                case SYS_KEY_UP: 
                    break;
                case SYS_KEY_DOWN:
                    break;
                case SYS_KEY_LEFT:
                case SYS_KEY_RIGHT:  
                    break;
                case SYS_KEY_ENTER:
                    color++;
                    color %= 3;
                    break;   
                }
            }
            break;
        case LINGO_KNEAD_TEST: 
            {
                engStatus = LINGO_KNEAD_TEST;
                command = HandUart_GetExternKey();
                switch(command)
                {
                case SYS_KEY_UP: 
                    if(kneadSpeed < 6)
                        kneadSpeed++;
                    break;
                case SYS_KEY_DOWN:
                    if(kneadSpeed > 0)
                        kneadSpeed--;
                    break;
                case SYS_KEY_LEFT:
                case SYS_KEY_RIGHT:  
                    if(kneadPhase == 0)
                        kneadPhase = 1;
                    else
                        kneadPhase = 0;
                    break;
                case SYS_KEY_ENTER:
                    break;   
                }
            }
            break;  
        case LINGO_KNOCK_TEST: 
            {
                engStatus = LINGO_KNOCK_TEST;
                command = HandUart_GetExternKey();
                switch(command)
                {
                case SYS_KEY_UP: 
                    if(kneadSpeed < 6)
                        kneadSpeed++;
                    break;
                case SYS_KEY_DOWN:
                    if(kneadSpeed > 0)
                        kneadSpeed--;
                    break;
                case SYS_KEY_LEFT:
                case SYS_KEY_RIGHT:  
                    if(kneadPhase == 0)
                        kneadPhase = 1;
                    else
                        kneadPhase = 0;
                    break;
                case SYS_KEY_ENTER:
                    break;   
                }
            }
            break;   
        case LINGO_INPUT: 
            {
                engStatus = LINGO_INPUT;
                command = HandUart_GetExternKey();
                switch(command)
                {
                case SYS_KEY_UP: 
                    break;
                case SYS_KEY_DOWN:
                    break;
                case SYS_KEY_LEFT:
                    break;
                case SYS_KEY_RIGHT:
                    break; 
                case SYS_KEY_ENTER:
                    break;   
                }
            }
            break;
        case LINGO_MUSIC_TEST:
            {
                engStatus = LINGO_MUSIC_TEST;
                command = HandUart_GetExternKey();
                switch(command)
                {
                case SYS_KEY_UP: 
                    break;
                case SYS_KEY_DOWN:
                    break;
                case SYS_KEY_LEFT:
                    break;
                case SYS_KEY_RIGHT:
                    break; 
                case SYS_KEY_ENTER:
                    //Power_AMP_Off();
                    Timer_Counter_Clear(C_TIMER_TEMP);
                    break;   
                }
            }
            break;
        case LINGO_PROGRAM: 
            engStatus = LINGO_PROGRAM;
            if(*(pInformation + PROGRAM_ENABLE_ADDRESS) != PROGRAM_FLAG)
            {
                *(pInformation + PROGRAM_ENABLE_ADDRESS) = PROGRAM_FLAG; //写编程标志位
                MEM_Write_Memory(pw_Information,MEMORY_LENGTH_OF_BYTES);
            }
            bProgram = true;
            break;
        case LINGO_PROGRAM_BY_BLUETOOTH: 
            break;
        case LINGO_BLUETOOTH_BR115200:
            break;  
        case LINGO_ENG:
            {
                ADC_Stop();
                engStatus = LINGO_ENG;
                command = HandUart_GetExternKey(); 
                switch(command) 
                {
                case ENG_CMD_RESET:  //关机是否复位
                    
                    if(*(pInformation + SETTLE_ADDRESS))
                    {
                        *(pInformation + SETTLE_ADDRESS) = 0; 
                    }
                    else
                    {
                        *(pInformation + SETTLE_ADDRESS) = 1; 
                    }
                    MEM_Write_Memory(pw_Information,MEMORY_LENGTH_OF_BYTES);  
                    break;  
                case ENG_CMD_DEC_STRENGTH:  //气囊力度减1
                    strength = *(pInformation + AIRBAG_STRETCH_ADDRESS);
                    //   by_t = *(pInformation + AIRBAG_STRETCH_ADDRESS);

                    if(strength == 0) break;
                    strength--;
                    strength %= 3;  //防止因为断电等原因导致数据错误
                    *(pInformation + AIRBAG_STRETCH_ADDRESS) = strength; 
                    MEM_Write_Memory(pw_Information,MEMORY_LENGTH_OF_BYTES);  
                    break;
                case ENG_CMD_ADD_STRENGTH:  //气囊力度加1
                    strength = *(pInformation + AIRBAG_STRETCH_ADDRESS);
              //       by_t = *(pInformation + AIRBAG_STRETCH_ADDRESS);
                    if(strength >= 2) break;
                    strength++;
                    *(pInformation + AIRBAG_STRETCH_ADDRESS) = strength; 
                    MEM_Write_Memory(pw_Information,MEMORY_LENGTH_OF_BYTES);  
                    break;
                case ENG_CMD_SLIDE:   //滑动使能禁止
                    if(*(pInformation + SLIDE_MOTOR_ENABLE_ADDRESS))
                    {
                        *(pInformation + SLIDE_MOTOR_ENABLE_ADDRESS) = 0; 
                        
                    }
                    else
                    {
                        *(pInformation + SLIDE_MOTOR_ENABLE_ADDRESS) = 1; 
                    }
                    MEM_Write_Memory(pw_Information,MEMORY_LENGTH_OF_BYTES);  
                    break; 
                default: break;    
                }
            }
            break;
        case LINGO_RESET:  
            password = 0;  
            NVIC_SystemReset();
            break; //复位CPU
        case LINGO_MENU:  engStatus = LINGO_MENU;
        break; //复位CPU
        }
        /*******以下程序为气囊测试*************************/  
        switch(engStatus)
        {
        
        case LINGO_ONE_KEY_TEST:
        {
            //检测信号 TODO,测试无信号时的情况
            Input_Proce();
            //使用中断标志
            if(engineeringTime_10msFlag)//10ms timer
            {
                engineerTimeCount++;
                engineerTimeCount %= 10 * TIME_COUNT; //10秒走一步   #define TIME_COUNT      100
                //engineerTimeCount %= 1000;  //当engineerTimeCount=1000时余数为0 ，运行时间为10sec
                air_bagTimeCount++;
                air_bagTimeCount %= 7 * TIME_COUNT; //7秒走一步
                // air_bagTimeCount %= 700;//air_bagTimeCount=700时余数为0 ，运行时间为7sec
                if(oneKeyLegCountDown > 0)oneKeyLegCountDown--;
                //时间中断清零
                engineeringTime_10msFlag = 0;
            }
            //实现
            //参数
            //back_position = Input_GetBackMotorPosition();
            back_position = 0;
            walk_position = Input_GetWalkMotorPosition();
            //自动测试步骤
            if(oneKeyStep == 0)
            {
          
              
              
                //行走
                if(!walk_up)//自动测试时第一步马达先向上行走，                       第一步
                {
                    if(Input_GetWalkUpSwitch() == REACH_WALK_LIMIT)
                    {
                        if(walk_up_old == 0)
                        {
                            if(walk_check_count < 3)
                            {
                                walk_check_count++;
                            }
                            else
                            {
                                //上行程OK
                                walk_up = 1;
                                //清零
                                walk_check_count = 0;
                            }
                        }
                        WalkMotor_Control(STATE_RUN_WALK_DOWN, 0);
                    }
                    else
                    {
                        WalkMotor_Control(STATE_RUN_WALK_UP, 0);
                    }
                }//肩位
                else if(!walk_down)//待马达向上走到最高位置时，行走马达向下行走，   第二步
                {
                    if(Input_GetWalkDownSwitch() == REACH_WALK_LIMIT)
                    {
                        Input_SetWalkMotorPosition(0);
                        if(walk_down_old == 0)
                        {
                            if(walk_check_count < 3)
                            {
                                walk_check_count++;
                            }
                            else
                            {
                                //下行程OK
                                walk_down = 1;
                                //清零
                                walk_check_count = 0;
                            }
                        }
                        WalkMotor_Control(STATE_RUN_WALK_UP, 0);
                    }
                    else
                    {
                        WalkMotor_Control(STATE_RUN_WALK_DOWN, 0);
                    }
                }
                else                                                 //待向上，向下走完一个行程后，行走电机向上走到最高点停下来
                {
                    if(Input_GetWalkUpSwitch() != REACH_WALK_LIMIT)
                    {
                        WalkMotor_Control(STATE_RUN_WALK_UP, 0);
                    }
                    else
                    {
                        WalkMotor_Control(STATE_WALK_IDLE, 0);
                    }
                }
                //揉捏
                if(!knead_width_min)                 //第一步揉捏 窄位置寻找
                {
                    if(Input_GetKneadMin() == 0)
                    {
                        if(knead_width_min_old == 0)
                        {
                            if(knead_check_count < 3)
                            {
                                knead_check_count++;
                            }
                            else
                            {
                                //OK
                                knead_width_min = 1;
                                //清零
                                knead_check_count = 0;
                            }
                        }
                        KneadMotor_Control(STATE_KNEAD_CLOCK_RUN, KNEAD_SPEED1_PWM);
                    }
                    else
                    {
                        KneadMotor_Control(STATE_KNEAD_UNCLOCK_RUN, KNEAD_SPEED1_PWM);
                    }
                }
                else if(!knead_width_mid)// //  第二步 揉捏 中位置寻找
                {
                    if(Input_GetKneadMid() == 0)
                    {
                        if(knead_width_mid_old == 0)
                        {
                            if(knead_check_count < 3)
                            {
                                knead_check_count++;
                            }
                            else
                            {
                                //OK
                                knead_width_mid = 1;
                                //清零
                                knead_check_count = 0;
                            }
                        }
                        KneadMotor_Control(STATE_KNEAD_CLOCK_RUN, KNEAD_SPEED6_PWM);
                    }
                    else
                    {
                        KneadMotor_Control(STATE_KNEAD_UNCLOCK_RUN, KNEAD_SPEED6_PWM);
                    }
                }
                else if(!knead_width_max)   //第三步  宽位置寻找
                {
                    if(Input_GetKneadMax() == 0)
                    {
                        if(knead_width_max_old == 0)
                        {
                            if(knead_check_count < 3)
                            {
                                knead_check_count++;
                            }
                            else
                            {
                                //OK
                                knead_width_max = 1;
                                //清零
                                knead_check_count = 0;
                            }
                        }
                        KneadMotor_Control(STATE_KNEAD_CLOCK_RUN, KNEAD_SPEED1_PWM);
                    }
                    else
                    {
                        KneadMotor_Control(STATE_KNEAD_UNCLOCK_RUN, KNEAD_SPEED1_PWM);
                    }
                }
                else if (!_3D_Switch_Forward)//第一步3D机芯先向前  ，3D脉冲信号增加
                {
                  KneadMotor_Control(STATE_KNEAD_IDLE, KNEAD_SPEED0_PWM);
                  if(AxisMotor_Control(STATE_RUN_AXIS_FORWARD,0,_3D_SPEED_7))
                  //if(Input_Get3DFrontSwitch()) 
                  {
                    AxisMotor_Control(STATE_AXIS_IDLE,0,_3D_SPEED_10); 
                    _3D_Switch_Forward = 1;
                    //printf("3d_forward\n");
                  }
                }
                else if (!_3D_Switch_Back)  //第二步 3D机芯再向后    3D脉冲信号减小
                {
                  if(AxisMotor_Control(STATE_RUN_AXIS_BEHIND,0,_3D_SPEED_7))
                  //if(Input_Get3DBackSwitch()) 
                  {
                    AxisMotor_Control(STATE_AXIS_IDLE,0,_3D_SPEED_10); 
                    _3D_Switch_Back = 1;
                    //printf("3d_back\n");
                  }
                }
                else if (!_3D_Switch_Pluse)   //检查 3D脉冲信号是否正常
                 
                  
                {
                  if(AxisMotor_Control(STATE_RUN_AXIS_REAL_VECTOR,20,_3D_SPEED_7))
                  {
                    AxisMotor_Control(STATE_AXIS_IDLE,0,_3D_SPEED_10); 
                    _3D_Switch_Pluse = 1;
                   // printf("3d_pluse\n");
                  }
                }
                else AxisMotor_Control(STATE_AXIS_IDLE,0,_3D_SPEED_10); 
                  
                //小腿

           //========================================================================
       
#ifdef    RT8305T_1    
                
               if(has_leg)
                {
                  Flex_SetDisableAngle(1);
                    if((!leg_up) &&(!leg_down)&&(!flex_up)&&(!flex_down))  
                    {   //测试小腿上行程开关
                       //FlexMotor_Control(STATE_RUN_FLEX_RESET, FLEX_SPEED_FAST, FLEX_CURRENT_3A);
                      Flex_SetDirection(FLEX_MOTOR_STOP);
                       switch(leg_flex_step)
                       {
                        case 0:  //到达up位置
                                if(LegMotor_Control(STATE_RUN_LEG_UP)) 
                                {
                                  leg_flex_step++;
                                  Timer_Counter_Clear(C_TIMER_ENG1);
                                }
                                break;  
                         case 1:  //停在up位置0.5秒
                               LegMotor_Control(STATE_LEG_IDLE);
                               if(Timer_Counter(C_TIMER_ENG1,5))
                                {
                                  leg_flex_step++;
                                }
                               break;  
                         case 2:  //离开up位置
                                LegMotor_Control(STATE_RUN_LEG_DOWN);
                                if(Input_GetLegUpSwitch() != REACH_LEG_LIMIT)
                                {
                                  Timer_Counter_Clear(C_TIMER_ENG1);
                                  leg_flex_step++;
                                }
                                break;
                          case 3:  //停0.5秒
                               LegMotor_Control(STATE_LEG_IDLE);
                               if(Timer_Counter(C_TIMER_ENG1,5))
                                {
                                  leg_flex_step++;
                                }
                               break;
                          case 4:  //到达up位置
                                if(LegMotor_Control(STATE_RUN_LEG_UP)) 
                                {
                                  leg_flex_step = 0;
                                  leg_up = 1;
                                }
                                break;       
                       }
                    }
                    if((leg_up) &&(!leg_down)&&(!flex_up)&&(!flex_down))  
                      {//测试电动伸缩小腿上（外）行程开关
                        LegMotor_Control(STATE_LEG_IDLE);
                        switch(leg_flex_step)
                       {
                        case 0:  //到达up位置
                                //if(FlexMotor_Control(STATE_RUN_FLEX_MANUAL_OUT, FLEX_SPEED_FAST, FLEX_CURRENT_3A))
                              if(Flex_ControlOut(FLEX_MOTOR_CURRENT_2A))
                                {
                                  leg_flex_step++;
                                  Timer_Counter_Clear(C_TIMER_ENG1);
                                }
                                break;  
                         case 1:  //停在up位置0.5秒
                               //FlexMotor_Control(STATE_FLEX_IDLE, FLEX_SPEED_FAST, FLEX_CURRENT_3A);
                               if(Timer_Counter(C_TIMER_ENG1,5))
                                {
                                  leg_flex_step++;
                                }
                               break;  
                         case 2:  //离开up位置
                                //FlexMotor_Control(STATE_RUN_FLEX_RESET, FLEX_SPEED_FAST, FLEX_CURRENT_3A);
                               if(Flex_ControlIn(FLEX_MOTOR_CURRENT_1D5A))
                                //if(Input_GetFlexOutSwitch() != REACH_FLEX_LIMIT)
                                {
                                  Timer_Counter_Clear(C_TIMER_ENG1);
                                  leg_flex_step++;
                                }
                                break;
                          case 3:  //停0.5秒
                               //FlexMotor_Control(STATE_FLEX_IDLE, FLEX_SPEED_FAST, FLEX_CURRENT_3A);
                               if(Timer_Counter(C_TIMER_ENG1,5))
                                {
                                  leg_flex_step++;
                                }
                               break;
                          case 4:  //到达up位置
                               // if(FlexMotor_Control(STATE_RUN_FLEX_MANUAL_OUT, FLEX_SPEED_FAST, FLEX_CURRENT_3A))
                                if(Flex_ControlOut(FLEX_MOTOR_CURRENT_2A))
                                {
                                  leg_flex_step = 0;
                                  flex_up = 1;
                                }
                                break;       
                       }
                      }
                    if((leg_up) &&(!leg_down)&&(flex_up)&&(!flex_down))  
                      {//测试电动伸缩小腿上（外）行程开关
                        LegMotor_Control(STATE_LEG_IDLE);
                        switch(leg_flex_step)
                       {
                        case 0:  //到达in位置
                                if(Flex_ControlIn(FLEX_MOTOR_CURRENT_3A))
                                //if(FlexMotor_Control(STATE_RUN_FLEX_RESET, FLEX_SPEED_FAST, FLEX_CURRENT_3A))
                                {
                                  leg_flex_step++;
                                  Timer_Counter_Clear(C_TIMER_ENG1);
                                }
                                break;  
                         case 1:  //停在in位置0.5秒
                              // FlexMotor_Control(STATE_FLEX_IDLE, FLEX_SPEED_FAST, FLEX_CURRENT_3A);
                               if(Timer_Counter(C_TIMER_ENG1,5))
                                {
                                  leg_flex_step++;
                                }
                               break;  
                         case 2:  //离开in位置
                                //FlexMotor_Control(STATE_RUN_FLEX_MANUAL_OUT, FLEX_SPEED_FAST, FLEX_CURRENT_3A);
                                //if(Input_GetFlexInSwitch() != REACH_FLEX_LIMIT)
                              if(Flex_ControlOut(FLEX_MOTOR_CURRENT_2A))
                                {
                                  Timer_Counter_Clear(C_TIMER_ENG1);
                                  leg_flex_step++;
                                }
                                break;
                          case 3:  //停0.5秒
                              // FlexMotor_Control(STATE_FLEX_IDLE, FLEX_SPEED_FAST, FLEX_CURRENT_3A);
                               if(Timer_Counter(C_TIMER_ENG1,5))
                                {
                                  leg_flex_step++;
                                }
                               break;
                          case 4:  //到达In位置
                               // if(FlexMotor_Control(STATE_RUN_FLEX_RESET, FLEX_SPEED_FAST, FLEX_CURRENT_3A))
                               if(Flex_ControlIn(FLEX_MOTOR_CURRENT_3A))
                                {
                                  leg_flex_step = 0;
                                  flex_down = 1;
                                }
                                break;       
                       }
                      }
                   if((leg_up) &&(!leg_down)&&(flex_up)&&(flex_down))   
                   {   //测试小腿下行程开关
                       //FlexMotor_Control(STATE_FLEX_IDLE, FLEX_SPEED_FAST, FLEX_CURRENT_3A); //关闭电动伸缩小腿
                       switch(leg_flex_step)
                       {
                        case 0:  //到达down位置
                                if(LegMotor_Control(STATE_RUN_LEG_DOWN)) 
                                {
                                  leg_flex_step++;
                                  Timer_Counter_Clear(C_TIMER_ENG1);
                                }
                                break;  
                         case 1:  //停在DOWN位置0.5秒
                               LegMotor_Control(STATE_LEG_IDLE);
                               if(Timer_Counter(C_TIMER_ENG1,5))
                                {
                                  leg_flex_step++;
                                }
                               break;  
                         case 2:  //离开DOWN位置
                                LegMotor_Control(STATE_RUN_LEG_UP) ;
                                if(Input_GetLegDownSwitch() != REACH_LEG_LIMIT);
                                {
                                  Timer_Counter_Clear(C_TIMER_ENG1);
                                  leg_flex_step++;
                                }
                                break;
                          case 3:  //停0.5秒
                               LegMotor_Control(STATE_LEG_IDLE);
                               if(Timer_Counter(C_TIMER_ENG1,5))
                                {
                                  leg_flex_step++;
                                }
                               break;
                          case 4:  //到达Down位置
                                if(LegMotor_Control(STATE_RUN_LEG_DOWN)) 
                                {
                                  leg_flex_step = 0;
                                  leg_down = 1;
                                }
                                break;       
                       }
                    }
                }//has_leg         
                
                
                
#else 

                if(has_leg)
                {
             //     Flex_SetDisableAngle(1);
                    if((!leg_up) &&(!leg_down))//&&(!flex_up)&&(!flex_down))  
                    {   //测试小腿上行程开关
  
                      Flex_SetDirection(FLEX_MOTOR_STOP);
                       switch(leg_flex_step)
                       {
                        case 0:  //到达up位置
                                if(LegMotor_Control(STATE_RUN_LEG_UP)) 
                                {
                                  leg_flex_step++;
                                  Timer_Counter_Clear(C_TIMER_ENG1);
                                }
                                break;  
                         case 1:  //停在up位置0.5秒
                               LegMotor_Control(STATE_LEG_IDLE);
                               if(Timer_Counter(C_TIMER_ENG1,5))
                                {
                                  leg_flex_step++;
                                }
                               break;  
                         case 2:  //离开up位置
                                LegMotor_Control(STATE_RUN_LEG_DOWN);
                                if(Input_GetLegUpSwitch() != REACH_LEG_LIMIT)
                                {
                                  Timer_Counter_Clear(C_TIMER_ENG1);
                                  leg_flex_step++;
                                }
                                break;
                          case 3:  //停0.5秒
                               LegMotor_Control(STATE_LEG_IDLE);
                               if(Timer_Counter(C_TIMER_ENG1,5))
                                {
                                  leg_flex_step++;
                                }
                               break;
                          case 4:  //到达up位置
                                if(LegMotor_Control(STATE_RUN_LEG_UP)) 
                                {
                                  leg_flex_step = 0;
                                  leg_up = 1;
                                }
                                break;       
                       }
                    }
             /*       if((leg_up) &&(!leg_down)&&(!flex_up)&&(!flex_down))  
                      {//测试电动伸缩小腿上（外）行程开关
                        LegMotor_Control(STATE_LEG_IDLE);
                        switch(leg_flex_step)
                       {
                        case 0:  //到达up位置
                                //if(FlexMotor_Control(STATE_RUN_FLEX_MANUAL_OUT, FLEX_SPEED_FAST, FLEX_CURRENT_3A))
                              if(Flex_ControlOut(FLEX_MOTOR_CURRENT_3A))
                                {
                                  leg_flex_step++;
                                  Timer_Counter_Clear(C_TIMER_ENG1);
                                }
                                break;  
                         case 1:  //停在up位置0.5秒
                               //FlexMotor_Control(STATE_FLEX_IDLE, FLEX_SPEED_FAST, FLEX_CURRENT_3A);
                               if(Timer_Counter(C_TIMER_ENG1,5))
                                {
                                  leg_flex_step++;
                                }
                               break;  
                         case 2:  //离开up位置
                                //FlexMotor_Control(STATE_RUN_FLEX_RESET, FLEX_SPEED_FAST, FLEX_CURRENT_3A);
                               if(Flex_ControlIn(FLEX_MOTOR_CURRENT_3A))
                                //if(Input_GetFlexOutSwitch() != REACH_FLEX_LIMIT)
                                {
                                  Timer_Counter_Clear(C_TIMER_ENG1);
                                  leg_flex_step++;
                                }
                                break;
                          case 3:  //停0.5秒
                               //FlexMotor_Control(STATE_FLEX_IDLE, FLEX_SPEED_FAST, FLEX_CURRENT_3A);
                               if(Timer_Counter(C_TIMER_ENG1,5))
                                {
                                  leg_flex_step++;
                                }
                               break;
                          case 4:  //到达up位置
                               // if(FlexMotor_Control(STATE_RUN_FLEX_MANUAL_OUT, FLEX_SPEED_FAST, FLEX_CURRENT_3A))
                                if(Flex_ControlOut(FLEX_MOTOR_CURRENT_3A))
                                {
                                  leg_flex_step = 0;
                                  flex_up = 1;
                                }
                                break;       
                       }
                      }
                    if((leg_up) &&(!leg_down)&&(flex_up)&&(!flex_down))  
                      {//测试电动伸缩小腿上（外）行程开关
                        LegMotor_Control(STATE_LEG_IDLE);
                        switch(leg_flex_step)
                       {
                        case 0:  //到达in位置
                                if(Flex_ControlIn(FLEX_MOTOR_CURRENT_3A))
                                //if(FlexMotor_Control(STATE_RUN_FLEX_RESET, FLEX_SPEED_FAST, FLEX_CURRENT_3A))
                                {
                                  leg_flex_step++;
                                  Timer_Counter_Clear(C_TIMER_ENG1);
                                }
                                break;  
                         case 1:  //停在in位置0.5秒
                              // FlexMotor_Control(STATE_FLEX_IDLE, FLEX_SPEED_FAST, FLEX_CURRENT_3A);
                               if(Timer_Counter(C_TIMER_ENG1,5))
                                {
                                  leg_flex_step++;
                                }
                               break;  
                         case 2:  //离开in位置
                                //FlexMotor_Control(STATE_RUN_FLEX_MANUAL_OUT, FLEX_SPEED_FAST, FLEX_CURRENT_3A);
                                //if(Input_GetFlexInSwitch() != REACH_FLEX_LIMIT)
                              if(Flex_ControlOut(FLEX_MOTOR_CURRENT_3A))
                                {
                                  Timer_Counter_Clear(C_TIMER_ENG1);
                                  leg_flex_step++;
                                }
                                break;
                          case 3:  //停0.5秒
                              // FlexMotor_Control(STATE_FLEX_IDLE, FLEX_SPEED_FAST, FLEX_CURRENT_3A);
                               if(Timer_Counter(C_TIMER_ENG1,5))
                                {
                                  leg_flex_step++;
                                }
                               break;
                          case 4:  //到达In位置
                               // if(FlexMotor_Control(STATE_RUN_FLEX_RESET, FLEX_SPEED_FAST, FLEX_CURRENT_3A))
                               if(Flex_ControlIn(FLEX_MOTOR_CURRENT_3A))
                                {
                                  leg_flex_step = 0;
                                  flex_down = 1;
                                }
                                break;       
                       }
                      }*/
                   if((leg_up) &&(!leg_down))//&&(flex_up)&&(flex_down))   
                   {   //测试小腿下行程开关
                       //FlexMotor_Control(STATE_FLEX_IDLE, FLEX_SPEED_FAST, FLEX_CURRENT_3A); //关闭电动伸缩小腿
                       switch(leg_flex_step)
                       {
                        case 0:  //到达down位置
                                if(LegMotor_Control(STATE_RUN_LEG_DOWN)) 
                                {
                                  leg_flex_step++;
                                  Timer_Counter_Clear(C_TIMER_ENG1);
                                }
                                break;  
                         case 1:  //停在DOWN位置0.5秒
                               LegMotor_Control(STATE_LEG_IDLE);
                               if(Timer_Counter(C_TIMER_ENG1,5))
                                {
                                  leg_flex_step++;
                                }
                               break;  
                         case 2:  //离开DOWN位置
                                LegMotor_Control(STATE_RUN_LEG_UP) ;
                                if(Input_GetLegDownSwitch() != REACH_LEG_LIMIT);
                                {
                                  Timer_Counter_Clear(C_TIMER_ENG1);
                                  leg_flex_step++;
                                }
                                break;
                          case 3:  //停0.5秒
                               LegMotor_Control(STATE_LEG_IDLE);
                               if(Timer_Counter(C_TIMER_ENG1,5))
                                {
                                  leg_flex_step++;
                                }
                               break;
                          case 4:  //到达Down位置
                                if(LegMotor_Control(STATE_RUN_LEG_DOWN)) 
                                {
                                  leg_flex_step = 0;
                                  leg_down = 1;
                                }
                                break;       
                       }
                    }
                }//has_leg
   
#endif
                
                
                //=================================================================
                //靠背
#ifdef  RT8305T_1
              
               
                      /*          if(nFlexStatus&0x20) //碰到角度开关
                 {
                   
                   leg_angle_Switch_old=1;
                 }
                 else
                 {
                    leg_angle_Switch_old=0;
                 }*/
                   
               
               
               if(nFlexStatus&0x20) //碰到角度开关 ,角度开关坏的情况下 一直为ON ,角度开关没坏时为0FF,硬件电路默认为ON
               {
                 if(leg_angle_Switch_old==0)leg_angle=1;//leg_ground
                 
               }
               else
               {
                  if(leg_ground)leg_angle=1;//有时出现机器一直到角度开关位置，此时触地已经打开
                 
                  if(leg_angle_Switch_old!=0)leg_angle=1;
                 
               }
              
               
                if(nFlexStatus&0x40)//碰到地面
                {                 
                  if(leg_groud_switch_old==0)leg_ground=1;                                  
                }
                else
                {
                     if(leg_groud_switch_old!=0)leg_ground=1;     
                  
                }
                
                
               if(nFlexStatus&0x04) 
                {
                  if(foot_Switch_old == 0) foot_Switch = 1;
                }
                else
                {
                  if(foot_Switch_old != 0) foot_Switch = 0;
                }
               /*
               if(nFlexStatus&0x04) 
                {
                   foot_Switch = 1;
                }
                else
                {
                   foot_Switch = 0;        
                }
               */
    #endif
               
               
              if(shoulder_detect_old != Input_GetVout())//肩位检测
              {
                shoulder_detect = 1;
              }
     
                if(!back_up)   //靠背电机向上行走，
                {
                    if(Input_GetBackUpSwitch() == REACH_BACK_LIMIT)
                    {
                        if(back_up_old == 0)
                        {
                            if(back_check_count < 3)
                            {
                                back_check_count++;
                            }
                            else
                            {
                                //OK
                                back_up = 1;
                                //清零
                                back_check_count = 0;
                            }
                        }
                        BackMotor_Control(STATE_RUN_BACK_DOWN);
                    }
                    else
                    {
                        BackMotor_Control(STATE_RUN_BACK_UP);
                    }
                }
                else if(!back_down)  //靠背再向下行走
                {
                    if(Input_GetBackDownSwitch() == REACH_BACK_LIMIT)
                    {
                        if(back_down_old == 0)
                        {
                            if(back_check_count < 3)
                            {
                                back_check_count++;
                            }
                            else
                            {
                                //OK
                                back_down = 1;
                                //清零
                                back_check_count = 0;
                            }
                        }
                        BackMotor_Control(STATE_RUN_BACK_UP);
                    }
                    else
                    {
                        BackMotor_Control(STATE_RUN_BACK_DOWN);
                    }
                }
                else   //靠背向上行走复位
                {
                    if(Input_GetBackUpSwitch() != REACH_BACK_LIMIT)
                    {
                        BackMotor_Control(STATE_RUN_BACK_UP);
                    }
                    else
                    {
                        BackMotor_Control(STATE_BACK_IDLE);
                    }
                }
              //---------------摇摆电机检测
            
    //     waver_check=1;      
                if(waver_check==0)
                { 
                  
              //     Input_Proce();             
                   if(bwaverCheck_First)
                   {
                      // Waveringly_Set_Pwm_TEST_Data(VIB_STRENGTH[2]);     
                       
                        Waveringly_Set_Pwm_Data(VIB_STRENGTH[3]);
                       
                     
                       bwaverCheck_First=0;                    
                       bwaverCheck_Sencod=0;
                       bwaverCheck_First2=0;
                      Timer_Counter_Clear(2);
                   }

                      //if(Timer_Counter(2,100))
                    if(Timer_Counter(2,300))//3min
                      {
                        
                          bwaverCheck_First2=1;      
                      }
                  
                   if(bwaverCheck_First2)
                   { 
                     //Waveringly_Set_Pwm_TEST_Data(VIB_STRENGTH[1]);   
                     if(Input_GetWaveMotorPosition()!= REACH_WAVER_LIMIT)
                     {
                       bwaverCheck_Sencod=1;
                     }
                     else
                     {
                       
                        bwaverCheck_Sencod=1;
                       
                     }
                     
                   }
   	
                  if( bwaverCheck_Sencod)
                  {
                       //   Waveringly_Set_Pwm_TEST_Data(VIB_STRENGTH[1]);   
                          Waveringly_Set_Pwm_Data(VIB_STRENGTH[1]);
                          if(Input_GetWaveMotorPosition() == REACH_WAVER_LIMIT)
                          {
                             Waveringly_Set_Pwm_Data(0);
                             waver_check=1;
                             bwaverCheck_First2=0;
                             bwaverCheck_First=1;
                             bwaverCheck_Sencod=0;
                          }

                   }     
                  
                }
               

                //-----------------------------------------------------------
#ifdef  RT8305T_1   
                 
               
               if(walk_up && walk_down && shoulder_detect && knead_width_min && knead_width_mid
                        && knead_width_max && ((!has_leg) || (leg_up && leg_down)) && back_up && back_down 
                          &&  flex_up && flex_down && foot_Switch 
                            && leg_angle  && _3D_Switch_Forward && _3D_Switch_Back && _3D_Switch_Pluse && waver_check &&leg_ground)
                {
                    if(oneKeyStep == 0)//上面所有电机正常检查后，计数增加1 ，表示完成第零步自动测试
                    {
                        oneKeyStep++;
                        engineerTimeCount = 1;
                    }
                }          
               
               
 #else
              
                flex_up=1;flex_down=1;foot_Switch=1;leg_angle=1;leg_ground=1;
                if(walk_up && walk_down && shoulder_detect && knead_width_min && knead_width_mid
                        && knead_width_max && ((!has_leg) || (leg_up && leg_down)) && back_up && back_down 
                          /*&& slide_backward && slide_forward && flex_up && flex_down && foot_Switch 
                            && leg_angle*/  && _3D_Switch_Forward && _3D_Switch_Back && _3D_Switch_Pluse && waver_check /*&&leg_ground*/)
                {
                    if(oneKeyStep == 0)//上面所有电机正常检查后，计数增加1 ，表示完成第零步自动测试
                    {
                        oneKeyStep++;
                        engineerTimeCount = 1;
                    }
                }
#endif         
                
                
            }
            else//  if(oneKeyStep == 0)   一键自动测试完成第一步测试标志
            {
                if(Input_GetWalkUpSwitch() != REACH_WALK_LIMIT && walk_up)//完成第一步测试：将所有电机回到默认的位置
                {
                    WalkMotor_Control(STATE_RUN_WALK_UP, 0);
                }
                else
                {
                    WalkMotor_Control(STATE_WALK_IDLE, 0);
                }
                if(Input_GetLegDownSwitch() != REACH_LEG_LIMIT && has_leg && leg_down)
                {
                    LegMotor_Control(STATE_RUN_LEG_DOWN);
                }
                else
                {
                    LegMotor_Control(STATE_LEG_IDLE);
                }
                if(Input_GetBackUpSwitch() != REACH_BACK_LIMIT && back_up)
                {
                    BackMotor_Control(STATE_RUN_BACK_UP);
                }
                else
                {
                    BackMotor_Control(STATE_BACK_IDLE);
                }
                //==========================将摇摆电机回中位
                if(Input_GetWaveMotorPosition() != REACH_WAVER_LIMIT)
                {
                   Main_WaveMotorStop();
                  
                  
                }

                
                
            }
            //需手动配合的部分
            //加热

            if(has_heat)
            {
                if(heat)WaistHeat_On();
                else WaistHeat_Off();
            }
            //敲击
            if(knock == 1)
            {
                if(engineerTimeCount < 9 * TIME_COUNT)//9sec
                {
                    KnockMotor_ClockRun();
                    KnockMotor_Set_Pwm_Data(KNOCK_SPEED1_PWM);
                }
                else
                {
                    KnockMotor_UnClockRun();
                    KnockMotor_Set_Pwm_Data(KNOCK_SPEED6_PWM);
                }
            }
            else
            {
                KnockMotor_Set_Pwm_Data(KNOCK_SPEED0_PWM);
                KnockMotor_Break();
            }
            //滚轮
            if(roller)
            {
                if(engineerTimeCount < 5 * TIME_COUNT)
                {
                    RollerMotor_Control(ROLLER_SPEED_SLOW, 0);
                }
                else
                {
                    RollerMotor_Control(ROLLER_SPEED_FAST, 1);
                }
            }
            else
            {
                RollerMotor_Control(ROLLER_SPEED_STOP, 0);
            }
            //气囊
            if(air_bag)
            {
             // Vavle_Pump_Switch(0, 1);
             // Vavle_Pump_Switch(1, 1);
                Valve_AirPumpACPowerOn();


                BITS_ValveData[0].nByte = 0;
                BITS_ValveData[1].nByte = 0;
                //BITS_ValveData[2].nByte = 0;
                //if(enAirbagStep > 16)BITS_ValveData[2].nByte = (1 << (enAirbagStep - 17)) & 0xff;
                //else 
                  if(enAirbagStep > 8)BITS_ValveData[1].nByte = (1 << (enAirbagStep - 9)) & 0xff;
                else BITS_ValveData[0].nByte = (1 << (enAirbagStep - 1)) & 0xff;
                //10秒后自动下一步
                if(air_bagTimeCount == 0)// air_bagTimeCount %= 7 * TIME_COUNT; //7秒走一步
                {
                    air_bagTimeCount++;//防止循环内重复调用
                    enAirbagStep++;   //气阀测试时每一个气囊的保持时间为7秒
                }
                //测试结束
  #ifdef  RT8305T_1              
               if(enAirbagStep > 24)
                {
                    enAirbagStep = 0;//清零
                    
                        if(walk_up && walk_down && shoulder_detect && knead_width_min && knead_width_mid
                        && knead_width_max && ((!has_leg) || (leg_up && leg_down)) && back_up && back_down 
                          /*&& slide_backward && slide_forward */&& flex_up && flex_down && foot_Switch 
                            && leg_angle && _3D_Switch_Forward && _3D_Switch_Back && _3D_Switch_Pluse&&leg_ground)
                    {
                        test_finish = 1;
                        air_bag = 0;
                    }
                }       
                
                
   #else             
                if(enAirbagStep > 24)
                {
                    enAirbagStep = 0;//清零
                      flex_up=1;flex_down=1;foot_Switch=1;leg_angle=1;leg_ground=1;
                        if(walk_up && walk_down && shoulder_detect && knead_width_min && knead_width_mid
                        && knead_width_max && ((!has_leg) || (leg_up && leg_down)) && back_up && back_down 
                          /*&& slide_backward && slide_forward && flex_up && flex_down && foot_Switch 
                            && leg_angle &&*/ _3D_Switch_Forward && _3D_Switch_Back && _3D_Switch_Pluse/*&&leg_ground*/)
                    {
                        test_finish = 1;
                        air_bag = 0;
                    }
                }
  #endif              
                
                
                
            }
            else
            {
                //Vavle_Pump_Switch(0, 0);
                //Vavle_Pump_Switch(1, 0);
               Valve_AirPumpACPowerOff();
               
                BITS_ValveData[0].nByte = 0;
                BITS_ValveData[1].nByte = 0;
                BITS_ValveData[2].nByte = 0;
            }
            Valve_Send_Data();
            //手动配合步骤
            //1：加热，2：敲击，3：滚轮，4：气囊,加热时不自动增加
            if(oneKeyStep > 0 && oneKeyStep < oneKeyStepLength)
            {
                //10秒后自动下一步
                if(engineerTimeCount == 0)
                {
                    engineerTimeCount++;//防止循环内重复调用
                    switch(oneKeyStep)
                    {
                    case 2:
                        knock = 0;
                        break;
                    case 3:
                        roller = 0;
                        break;
                    }
                    oneKeyStep++;
                }
            }
            if(bMasterSendPacket)//上传检测结果给手控器
            {
                OutBuffer[0] = SOI;
                OutBuffer[1] = 0;
                OutBuffer[1] |= heat;
                OutBuffer[1] |= has_heat << 1;
                OutBuffer[1] |= walk_up << 2;
                OutBuffer[1] |= walk_down << 3;
                OutBuffer[1] |= shoulder_detect << 4;
                OutBuffer[1] |= knead_width_min << 5;
                OutBuffer[1] |= knead_width_mid << 6;
                OutBuffer[1] |= knead_width_max << 7;
                OutBuffer[2] = 0;
                OutBuffer[2] |= has_leg;
                OutBuffer[2] |= leg_up << 1;
                OutBuffer[2] |= leg_down << 2;
                OutBuffer[2] |= back_up << 3;
                OutBuffer[2] |= back_down << 4;
                OutBuffer[2] |= (back_position & 0x7) << 5;
                OutBuffer[3] = ((back_position >> 3) & 0x7f) | ((walk_position & 0x1) << 7);
                OutBuffer[4] = (walk_position >> 1) & 0xff;
                OutBuffer[5] = (enAirbagStep & 0x1f) | ((oneKeyStep & 0x7) << 5);
                OutBuffer[6] = (knock << 7) | (roller << 6) | (test_finish << 5);
                
                OutBuffer[7] = 0x03;
                //OutBuffer[7] |= slide_backward;   //向下
                //OutBuffer[7] |= slide_forward << 1;//向上
                
                OutBuffer[7] |= flex_up << 2;
                OutBuffer[7] |= flex_down << 3;
                OutBuffer[7] |= foot_Switch << 4;
                OutBuffer[7] |= leg_angle << 5;
                OutBuffer[7] |= leg_ground << 6;
                
                OutBuffer[8] = 0;
                OutBuffer[8] |= _3D_Switch_Forward;
                OutBuffer[8] |= _3D_Switch_Back<<1;
                OutBuffer[8] |= _3D_Switch_Pluse<<2;
                //OutBuffer[8] |= waver_check<<3;//摇摆电机回中位正常
                
                OutBuffer[8] |= 1<<3;//摇摆电机回中位正常
                
                
                OutBuffer[9] = EOI;
                
                
                
                nOutBufferCount = 10;
                HandUart_Transmit_Packet(OutBuffer, nOutBufferCount);
                bMasterSendPacket = FALSE;
            }
            Main_Send_Leg();
            walk_up_old = (Input_GetWalkUpSwitch() == REACH_WALK_LIMIT);
            walk_down_old = (Input_GetWalkDownSwitch() == REACH_WALK_LIMIT);
            shoulder_detect_old = (Input_GetVout() == BODY_TOUCHED);
            knead_width_min_old = (Input_GetKneadMin() == 0);
            knead_width_mid_old = (Input_GetKneadMid() == 0);
            knead_width_max_old = (Input_GetKneadMax() == 0);
            back_up_old = (Input_GetBackUpSwitch() == REACH_BACK_LIMIT);
            back_down_old = (Input_GetBackDownSwitch() == REACH_BACK_LIMIT);
            leg_up_old   = (Input_GetLegUpSwitch() == REACH_BACK_LIMIT);
            leg_down_old  = (Input_GetLegDownSwitch() == REACH_BACK_LIMIT);
        }
        break;  //=以上为一键自动测试所有硬件接口
     //-------------------------------------------------------------------------   
        case LINGO_HEAT_TEST:
            {
                if(bMasterSendPacket)
                {
                    OutBuffer[0] = SOI ;
                    OutBuffer[1] = (unsigned char)bHeat;
                    OutBuffer[2] = EOI ;
                    nOutBufferCount = 3;
                    HandUart_Transmit_Packet(OutBuffer,nOutBufferCount);
                    bMasterSendPacket = FALSE ;
                }
            }
            break;
            
            
        case LINGO_MUSIC_TEST:
            {
                if(Timer_Counter(C_TIMER_TEMP,1))
                {
                    //Power_AMP_On();  //0.1秒后开启蓝牙
                }
                if(bMasterSendPacket)  
                {/*
                  if(BlueToothUart_GetRXStatus())
                  {
                    unsigned char *name;
                    BlueToothUart_GetModlueName(name);
                    nOutBufferCount = strlen(name) + 2;
                    OutBuffer[strlen(name)] = EOI;           
                    HandUart_Transmit_Packet(OutBuffer,nOutBufferCount);
                    bMasterSendPacket = FALSE ;
                  }
                  else
                  */
                  {
                    OutBuffer[0] = EOI;
                    OutBuffer[1] = 0;
                    OutBuffer[2] = SOI;
                    nOutBufferCount = 3;
                    HandUart_Transmit_Packet(OutBuffer,nOutBufferCount);
                    bMasterSendPacket = FALSE ;
                  }
                }
            }
            break;
            
        case LINGO_LED_TEST:
            {
                Valve_Send_Data();
                Input_Proce();
                
                if(color == 0) 
                {
                    LED_RGB_Set_Red_Data(0);
                    LED_RGB_Set_Green_Data(100);
                    LED_RGB_Set_Blue_Data(0);
                }
                if(color == 1) 
                {
                    LED_RGB_Set_Red_Data(100);
                    LED_RGB_Set_Green_Data(0);
                    LED_RGB_Set_Blue_Data(0);
                } 
                if(color == 2) 
                {
                    LED_RGB_Set_Red_Data(0);
                    LED_RGB_Set_Green_Data(0);
                    LED_RGB_Set_Blue_Data(100);
                }
                
                if(bMasterSendPacket)
                {
                    OutBuffer[0] = SOI ;
                    OutBuffer[1] = color;
                    OutBuffer[2] = EOI ;
                    nOutBufferCount = 3;
                    HandUart_Transmit_Packet(OutBuffer,nOutBufferCount);
                    bMasterSendPacket = FALSE ;
                }
            }
            break;
           
        case LINGO_SLIDE_TEST:    //零重力电机测试
            {
                Valve_Send_Data();
                Input_Proce();
                if(bUpKey) SlideMotorControl(STATE_RUN_SLIDE_FORWARD);//向上   ,第一个零重力点 时，零重力向上运行到行程开关位置处
                if(bDownKey) SlideMotorControl(STATE_RUN_SLIDE_BACKWARD);//零重力电机向下
                if(!bUpKey && !bDownKey) SlideMotorControl(STATE_SLIDE_IDLE);
                if(bMasterSendPacket)
                {
                    OutBuffer[0] = SOI ;
                    OutBuffer[1] = 0;
                    if(Input_GetSlideForwardSwitch() == REACH_SLIDE_LIMIT)  OutBuffer[1] |= 0x01;
                    else  OutBuffer[1] &= ~0x01;
                    if(Input_GetSlideBackwardSwitch() == REACH_SLIDE_LIMIT) OutBuffer[1] |= 0x02;
                    else OutBuffer[1] &= ~0x02;
                    OutBuffer[2] = EOI ;
                    nOutBufferCount = 3;
                    HandUart_Transmit_Packet(OutBuffer,nOutBufferCount);
                    bMasterSendPacket = FALSE ;
                }
            }
            break;
          
         case LINGO_WAVER_TEST:    //摇摆电机测试
            {
                Valve_Send_Data();
                Input_Proce();
                 Main_Send_Leg();
             //   nKeySeatVibrateStrength=3;
            /*  if( nKeySeatVibrateStrength==0)
              {
                
                Waveringly_Set_Pwm_Data(0);// Main_WaveMotorStop();
              }
                
              else
              {
                    switch(nKeySeatVibrateStrength)
                    {
                    default:  
                    case 1:Waveringly_Set_Pwm_Data(VIB_STRENGTH[1]);  break ;
                    case 2:Waveringly_Set_Pwm_Data(VIB_STRENGTH[2]);  break ;
                    case 3:Waveringly_Set_Pwm_Data(VIB_STRENGTH[3]);  break ;
                    }

               }        
              */
              
                
              /*  if(bUpKey) Waveringly_Set_Pwm_Data(VIB_STRENGTH[3]);
                if(bDownKey) Waveringly_Set_Pwm_Data(VIB_STRENGTH[3]);//nKeySeatVibrateStrength
                if(!bUpKey && !bDownKey) Waveringly_Set_Pwm_Data(0);// Main_WaveMotorStop();
                */
                
                
                if(bMasterSendPacket)
                {
                    OutBuffer[0] = SOI ;
                    OutBuffer[1] = 0;
                    //if(Input_GetWaveMotorPosition() == REACH_WAVER_LIMIT)  OutBuffer[1] |= 0x01;//摇摆电机回中位
                    //else  OutBuffer[1] &= ~0x01;
                    //OutBuffer[1] = 0x01;
                     //OutBuffer[1] |= 0<<1;//摇摆速度
                     
                    
                  //  if(Input_GetSlideForwardSwitch() == REACH_SLIDE_LIMIT)  OutBuffer[1] |= 0x01;
                   // if(Input_GetSlideBackwardSwitch() == REACH_SLIDE_LIMIT) OutBuffer[1] |= 0x02;
                    OutBuffer[2] = EOI ;
                    nOutBufferCount = 3;
                    HandUart_Transmit_Packet(OutBuffer,nOutBufferCount);
                    bMasterSendPacket = FALSE ;
                }
            }
            break;           
            
            
            
            
            
        case LINGO_3D_TEST:  //3D电机前后行程测试
            {
                Valve_Send_Data();
                Input_Proce();
                // Main_MassageSignalSend();
                if(bUpKey) AxisMotor_Control(STATE_RUN_AXIS_FORWARD,0,_3D_SPEED_3);//坐标最大，力度最大 ，向前
                if(bDownKey) AxisMotor_Control(STATE_RUN_AXIS_BEHIND,0,_3D_SPEED_3);//坐标为0  力度最小  ，向后 AxisMotor_Control(STATE_RUN_AXIS_BEHIND,0,_3D_SPEED_8))
//              if(bUpKey) AxisMotor_Control(STATE_RUN_AXIS_FORWARD,0);
//              if(bDownKey) AxisMotor_Control(STATE_RUN_AXIS_BEHIND,0);
                /*
                if(Timer_Counter(C_TIMER_TEMP+T_LOOP,20))
                {
                   printf("counter:%d,speed:%d\n",++counter,LEUART0_Get3D_Speed());
                }
                */
                
            /*     if(bLeftKey)//= true;//bRightKey
                 {
                   
                   WalkMotor_Control(STATE_RUN_WALK_UP,0);// 
                 }
                 if(bRightKey)
                 {
                   WalkMotor_Control(STATE_RUN_WALK_DOWN,0);
                   
                   
                 }
                 if(!bLeftKey && !bRightKey)
                 {
                  
                   WalkMotor_Control(STATE_WALK_IDLE,0);
                  
                 }*/
                
                if(!bUpKey && !bDownKey) AxisMotor_Control(STATE_AXIS_IDLE,0,_3D_SPEED_10);
                if(bMasterSendPacket)
                {
                    OutBuffer[0] = SOI ;
                    OutBuffer[1] = 0;
                    if(Input_Get3DFrontSwitch()) OutBuffer[1] |= 0x02;
                    else OutBuffer[1] &= ~0x02;
                    if(Input_Get3DBackSwitch()) OutBuffer[1] |= 0x01;  
                    else OutBuffer[1] &= ~0x01;  
                    OutBuffer[2] = Input_GetAxisMotorPosition();
                    
                    ADC_Get_Voltage(ADC_Vaxis,&adcAxisCurrent);
                     __no_operation();
                     __no_operation();
                     __no_operation();
                     __no_operation();  
                     adcAxisCurrent=adcAxisCurrent*2;//单位为采样的电流为mA,空载电流350mA左右
                     __no_operation();
                     __no_operation();   
               //      w_current2=adcAxisCurrent;
                     __no_operation();
                     __no_operation();     
                 //    w_current2=0x320;//800;   buf[3]=0x03,buf[4]=0x20
                     
                     
              //     w_current=adcAxisCurrent*5;
                     
             //       w_current=w_current2*5;
                      __no_operation();
                     __no_operation();   
                    OutBuffer[3] = adcAxisCurrent >> 8 ;  //手控器将电流的采样值除以5
                    OutBuffer[4] = (unsigned char)adcAxisCurrent ;
                   
             //       OutBuffer[3] = w_current2 >> 8 ;  //手控器将电流的采样值除以5
               //     OutBuffer[4] = (unsigned char)w_current2 ;        
              
                    OutBuffer[5] = EOI ;
                    nOutBufferCount = 6;
                    HandUart_Transmit_Packet(OutBuffer,nOutBufferCount);
                    bMasterSendPacket = FALSE ;
                }
            }

            break;
        case LINGO_BACK_TEST:   //靠背电机测试
            {
                Valve_Send_Data();
                Input_Proce();
                Main_Send_Leg();
               if(bUpKey) BackMotor_Control(STATE_RUN_BACK_UP);
               if(bDownKey) BackMotor_Control(STATE_RUN_BACK_DOWN);

                if(!bUpKey && !bDownKey) BackMotor_Control(STATE_BACK_IDLE);
                
                if(bMasterSendPacket)
                {
                    OutBuffer[0] = SOI ;
                    OutBuffer[1] = 0;
                    if(Input_GetBackUpSwitch() == REACH_BACK_LIMIT) OutBuffer[1] |= 0x01;
               //     else OutBuffer[1] &= ~0x01;
                    if(Input_GetBackDownSwitch() == REACH_BACK_LIMIT) OutBuffer[1] |= 0x02;
                //    else  OutBuffer[1] &= ~0x02;
                    OutBuffer[2] = (unsigned char)(BackMotor_Get_Location()); //low
                    OutBuffer[3] = (unsigned char)((BackMotor_Get_Location()) >>8);  //high
                    OutBuffer[4] = EOI ;
                    nOutBufferCount = 5;
                    HandUart_Transmit_Packet(OutBuffer,nOutBufferCount);
                    bMasterSendPacket = FALSE ;
                }
            }
            break;
        case LINGO_LEG_TEST:   //小腿上下测试
            {
                Valve_Send_Data();
                Input_Proce();
                Main_Send_Leg();
                if(bUpKey) LegMotor_Control(STATE_RUN_LEG_UP);
                if(bDownKey) LegMotor_Control(STATE_RUN_LEG_DOWN);
                if(!bUpKey && !bDownKey) LegMotor_Control(STATE_LEG_IDLE);
                
                if(bMasterSendPacket)
                {
                    OutBuffer[0] = SOI ;
                    OutBuffer[1] = 0;
                    if(Input_GetLegUpSwitch() == REACH_BACK_LIMIT)
                    {
                        OutBuffer[1] |= 0x01;
                    }
                    else OutBuffer[1] &= ~0x01;
                    if(Input_GetLegDownSwitch() == REACH_BACK_LIMIT)
                    {
                        OutBuffer[1] |= 0x02;
                    }
                    else OutBuffer[1] &= ~0x02;
                    

                    /*if(nFlexStatus&0x20) OutBuffer[1] |= 0x08;//碰到角度开关
                    else  OutBuffer[1] &= ~0x08;
                     if(nFlexStatus&0x40) OutBuffer[1] |= 0x10;//碰到地面
                     else OutBuffer[1] &= ~0x10;*/
                    
                   // if(nFlexStatus&0x20) OutBuffer[1] |= 0x08;//碰到角度开关
                   // else  OutBuffer[1] &= ~0x08;
                   /* if(nFlexStatus&0x20) OutBuffer[1] |= 0x04;//碰到角度开关
                    else  OutBuffer[1] &= ~0x04;                  
                    
                     if(nFlexStatus&0x04) OutBuffer[1] |= 0x10;//foot
                     else OutBuffer[1] &= ~0x10;  
                     
                     
                     if(nFlexStatus&0x40) OutBuffer[1] |= 0x08;//碰到地面
                     else OutBuffer[1] &= ~0x08;  
                     */
                     /////////////////////////////////////////////////////////////////
                    if(nFlexStatus&0x04) OutBuffer[1] |= 0x04; //碰到脚底开关
                    else OutBuffer[1] &= ~0x04;
                    
                    if(nFlexStatus&0x20) OutBuffer[1] |= 0x08;//碰到角度开关
                    else OutBuffer[1] &= ~0x08;
                    
                     if(nFlexStatus&0x40) OutBuffer[1] |= 0x10;//碰到地面
                     else  OutBuffer[1] &= ~0x10;
                     
                     
                     
                     
                     
       //             OutBuffer[2] = (unsigned char)nLegAngle; //low
       //             OutBuffer[3] = (unsigned char)(nLegAngle >>8);  //high
                    OutBuffer[4] = EOI ;
                    nOutBufferCount = 5;
                    HandUart_Transmit_Packet(OutBuffer,nOutBufferCount);
                    bMasterSendPacket = FALSE ;
                }
            }
            break;
            
        case LINGO_FLEX_TEST:  //小腿伸缩测试
            {
                Valve_Send_Data();
                Input_Proce();
                Main_Send_Leg();
                if(bUpKey) 
                {
                  Flex_SetDisableAngle(1);
 //                 Flex_SetCurrent(FLEX_MOTOR_CURRENT_4A);
                   Flex_SetCurrent(FLEX_MOTOR_CURRENT_2A);
                  
                  Flex_SetDirection(FLEX_TO_OUT);  //FLEX_TO_OUT=1  小腿伸
      
                }
                if(bDownKey) 
                {
                  Flex_SetDisableAngle(1);
 //                 Flex_SetCurrent(FLEX_MOTOR_CURRENT_4A);
                   Flex_SetCurrent(FLEX_MOTOR_CURRENT_2A);
                  
                 Flex_SetDirection(FLEX_TO_IN);  //小腿缩
       
                }
                if(!bUpKey && !bDownKey) 
                {
                  Flex_SetDisableAngle(0);
                  Flex_SetDirection(FLEX_MOTOR_STOP);
                }
                if(bMasterSendPacket)
                {
                    OutBuffer[0] = SOI ;
                    OutBuffer[1] = 0;

                   if((nFlexStatus&0x03) ==  FLEX_AT_OUT) OutBuffer[1] |= 0x01;
               
                   if((nFlexStatus&0x03) ==  FLEX_AT_IN) OutBuffer[1] |= 0x02;

/*                  
                    if(nFlexStatus&0x04) OutBuffer[1] |= 0x10; //碰到脚底开关
                    else OutBuffer[1] &= ~0x10;
                    
                    if(nFlexStatus&0x20) OutBuffer[1] |= 0x04;//碰到角度开关
                    else OutBuffer[1] &= ~0x04;
                    
                     if(nFlexStatus&0x40) OutBuffer[1] |= 0x08;//碰到地面
                     else  OutBuffer[1] &= ~0x08;   
*/
                     
                     
                  
                    if(nFlexStatus&0x04) OutBuffer[1] |= 0x04; //碰到脚底开关
                    else OutBuffer[1] &= ~0x04;
                    
                    if(nFlexStatus&0x20) OutBuffer[1] |= 0x08;//碰到角度开关
                    else OutBuffer[1] &= ~0x08;
                    
                     if(nFlexStatus&0x40) OutBuffer[1] |= 0x10;//碰到地面
                     else  OutBuffer[1] &= ~0x10;
                  
                     
                    
        //            OutBuffer[2] = (unsigned char)nLegAngle; //low
         //           OutBuffer[3] = (unsigned char)(nLegAngle >>8);  //high
                    OutBuffer[4] = EOI ;
                    nOutBufferCount = 5;
                    HandUart_Transmit_Packet(OutBuffer,nOutBufferCount);
                    bMasterSendPacket = FALSE ;
                }
            }
            break;
            
        case LINGO_WALK_TEST: //行走电机测试
            {
                Valve_Send_Data();
                Input_Proce();
                //Main_MassageSignalSend();
                if(bUpKey) 
                {
                  WalkMotor_Control(STATE_RUN_WALK_UP,0);// WalkMotor_Control(STATE_RUN_WALK_DOWN,0); WalkMotor_Control(STATE_WALK_IDLE,0);
             //     KnockMotor_ClockRun();
             //     KnockMotor_Set_Pwm_Data(KNOCK_SPEED6_PWM); 
    //               KneadMotor_Control(STATE_KNEAD_CLOCK_RUN,KNEAD_SPEED1_PWM);//
              
                  
                }
                
                if(bDownKey)
                {
                  WalkMotor_Control(STATE_RUN_WALK_DOWN,0);
             //     KnockMotor_ClockRun();
             //     KnockMotor_Set_Pwm_Data(KNOCK_SPEED6_PWM);       
   //              KneadMotor_Control(STATE_KNEAD_CLOCK_RUN,KNEAD_SPEED1_PWM);//
                  
                }
                
                if(!bUpKey && !bDownKey) 
                {

        
                  WalkMotor_Control(STATE_WALK_IDLE,0);
    //              KnockMotor_Set_Pwm_Data(0);  
          
   //                KneadMotor_Control(STATE_KNEAD_IDLE,KNEAD_SPEED0_PWM);
                  
                }
                
               /* if(bLeftKey)
                {
               
                
                   KneadMotor_Control(STATE_KNEAD_CLOCK_RUN,KNEAD_SPEED5_PWM);
                  
                }
                if(bRightKey)
                {
                  KneadMotor_Control(STATE_KNEAD_IDLE,KNEAD_SPEED0_PWM);
                  
                }*/
                
      /*                         if(bUpKey) BackMotor_Control(STATE_RUN_BACK_UP);
               if(bDownKey) BackMotor_Control(STATE_RUN_BACK_DOWN);

                if(!bUpKey && !bDownKey) BackMotor_Control(STATE_BACK_IDLE);
                */
                
                
                if(bLeftKey)
                {
                  BackMotor_Control(STATE_RUN_BACK_UP);
                  
                }
                if(bRightKey)
                {
                  BackMotor_Control(STATE_RUN_BACK_DOWN);
                }
                
                 if(!bLeftKey && !bRightKey)
                 {
                   
                   
                   BackMotor_Control(STATE_BACK_IDLE);
                   
                 }
                
                
                
                if(bMasterSendPacket)
                {
                    OutBuffer[0] = SOI ;
                    OutBuffer[1] = 0;
                    if(Input_GetWalkUpSwitch() == REACH_WALK_LIMIT) OutBuffer[1] |= 0x01;
                    else OutBuffer[1] &= ~0x01;
                    if(Input_GetWalkDownSwitch() == REACH_WALK_LIMIT) OutBuffer[1] |= 0x02;
                    else OutBuffer[1] &= ~0x02;
                    if(Input_GetVout() == BODY_TOUCHED)//手拉下揉捏头是为ON 信号  VOUT=1
                    {
                     __no_operation();
                     __no_operation();
                     __no_operation();
                     __no_operation();     
                     __no_operation();
                     __no_operation();  
                     __no_operation();
                     __no_operation();
                     __no_operation();
                     __no_operation();     
                     __no_operation();
                     __no_operation();                       
                     OutBuffer[1] |= 0x04;                                           
                    }                    
  
                    
                    OutBuffer[2] = Input_GetWalkMotorPosition() >> 8;
                    OutBuffer[3] = Input_GetWalkMotorPosition() ;
                    
                    
                    w_walk_pos= Input_GetWalkMotorPosition() ;
                    
                   // ADC_Get_Voltage(ADC_Vwalk,&adcWalkCurrent);
                   // OutBuffer[4] = adcWalkCurrent >> 8;
                   // OutBuffer[5] = adcWalkCurrent;
                    OutBuffer[6] = EOI ;
                    nOutBufferCount = 7;
                    HandUart_Transmit_Packet(OutBuffer,nOutBufferCount);
                    bMasterSendPacket = FALSE ;
                }
            }
            break;
        case LINGO_LEG_KNEAD_TEST:
          {
                Main_Send_Leg();
                if(legKneadSpeed == 0)
                {
                    LegKnead_SetPower(LEG_KNEAD_OFF);
                    LegKnead_SetSpeed(LEG_KNEAD_SPEED_STOP);
                }
                else
                {
                   LegKnead_SetPower(LEG_KNEAD_ON);
                    switch(legKneadSpeed)
                    {
                    default:  
                    case 1:LegKnead_SetSpeed(LEG_KNEAD_SPEED_SLOW);  break ;
                    case 2:LegKnead_SetSpeed(LEG_KNEAD_SPEED_MID);  break ;
                    case 3:LegKnead_SetSpeed(LEG_KNEAD_SPEED_FAST);  break ;
                    }
                    if(legKneadPhase == 0)
                        LegKnead_SetMode(LEG_KNEAD_TO_IN);
                    else
                        LegKnead_SetMode(LEG_KNEAD_TO_OUT);
                }
                if(bMasterSendPacket)
                {
                    OutBuffer[0] = SOI ;
                    OutBuffer[1] = legKneadSpeed;
                    OutBuffer[2] = legKneadPhase;
                    if(nFlexStatus&0x10) 
                    {
                      OutBuffer[3] = 1;
                    }
                    else 
                    {
                      OutBuffer[3] = 0;
                    }
                    OutBuffer[4] = EOI ;
                    nOutBufferCount = 5;
                    HandUart_Transmit_Packet(OutBuffer,nOutBufferCount);
                    bMasterSendPacket = FALSE ;
                }
            }
          break;
        case LINGO_ROLLER_TEST:   //滚轮测试
          {
                Main_Send_Leg();
                if(rollerSpeed == 0)
                {
                    Roller_SetSpeed(ROLLER_SPEED_STOP);
                }
                else
                {
                    switch(rollerSpeed)
                    {
                    default:  
                    case 1:Roller_SetSpeed(ROLLER_SPEED_SLOW);  break ;
                    case 2:Roller_SetSpeed(ROLLER_SPEED_MID);  break ;
                    case 3:Roller_SetSpeed(ROLLER_SPEED_FAST);  break ;
                    }
                    if(rollerPhase == 0)
                        Roller_SetMode(ROLLER_MODE_CON_IN);
                    else
                        Roller_SetMode(ROLLER_MODE_CON_OUT);
                }
                if(bMasterSendPacket)
                {
                    OutBuffer[0] = SOI ;
                    OutBuffer[1] = rollerSpeed;
                    OutBuffer[2] = rollerPhase;
                    OutBuffer[3] = EOI ;
                    nOutBufferCount = 4;
                    HandUart_Transmit_Packet(OutBuffer,nOutBufferCount);
                    bMasterSendPacket = FALSE ;
                }
            }
            break;
            
        case LINGO_KNOCK_TEST:  
            {
                Valve_Send_Data();
                Input_Proce();
                if(kneadSpeed == 0)
                {
                    KnockMotor_Break();
                }
                else
                {
                    switch(kneadSpeed)
                    {
                    default:  
                    case 0:PWM = KNOCK_SPEED0_PWM;  break ;
                    case 1:PWM = KNOCK_SPEED1_PWM;  break ;
                    case 2:PWM = KNOCK_SPEED2_PWM;  break ;
                    case 3:PWM = KNOCK_SPEED3_PWM;  break ;
                    case 4:PWM = KNOCK_SPEED4_PWM;  break ;
                    case 5:PWM = KNOCK_SPEED5_PWM;  break ;
                    case 6:PWM = KNOCK_SPEED6_PWM;  break ;
                    }
                    if(kneadPhase == 0)
                    KnockMotor_ClockRun();
                    else
                    KnockMotor_UnClockRun();

                    KnockMotor_Set_Pwm_Data(PWM);//KNOCK_SPEED6_PWM KnockMotor_ClockRun();
                }
                
                if(bMasterSendPacket)
                {
                    OutBuffer[0] = SOI ;
                    OutBuffer[1] = kneadSpeed;
                    OutBuffer[2] = kneadPhase;
                    OutBuffer[3] = Input_GetKneadPosition();
                    //ADC_Get_Voltage(ADC_Vknock,&adcKnockCurrent);
                    //OutBuffer[4] = adcKnockCurrent >> 8;
                    //OutBuffer[5] = adcKnockCurrent;
                    OutBuffer[6] = EOI ;
                    nOutBufferCount = 7;
                    HandUart_Transmit_Packet(OutBuffer,nOutBufferCount);
                    bMasterSendPacket = FALSE ;
                }
            }
            break;
        case LINGO_KNEAD_TEST:
            {
                Valve_Send_Data();
                //Main_MassageSignalSend();
                Input_Proce();
                if(kneadSpeed == 0)
                {
                    KneadMotor_Control(STATE_KNEAD_IDLE,KNEAD_SPEED0_PWM);//KneadMotor_Control(STATE_KNEAD_CLOCK_RUN,PWM);
                }
                else
                {
                    switch(kneadSpeed)
                    {
                    default:  
                    case 0:PWM = KNEAD_SPEED0_PWM;  break ;
                    case 1:PWM = KNEAD_SPEED1_PWM;  break ;
                    case 2:PWM = KNEAD_SPEED2_PWM;  break ;
                    case 3:PWM = KNEAD_SPEED3_PWM;  break ;
                    case 4:PWM = KNEAD_SPEED4_PWM;  break ;
                    case 5:PWM = KNEAD_SPEED5_PWM;  break ;
                    case 6:PWM = KNEAD_SPEED6_PWM;  break ;
                    }
                    if(kneadPhase == 0)
                        KneadMotor_Control(STATE_KNEAD_CLOCK_RUN,PWM);//
                    else
                        KneadMotor_Control(STATE_KNEAD_UNCLOCK_RUN,PWM);
                }
                
                if(bMasterSendPacket)
                {
                    OutBuffer[0] = SOI ;
                    OutBuffer[1] = kneadSpeed;
                    OutBuffer[2] = kneadPhase;
                    OutBuffer[3] = Input_GetKneadPosition();
                    OutBuffer[4] = EOI ;
                    nOutBufferCount = 5;
                    HandUart_Transmit_Packet(OutBuffer,nOutBufferCount);
                    bMasterSendPacket = FALSE ;
                }
            }
            break;  
        case LINGO_INPUT:
            if(bMasterSendPacket)
            {
                if(Timer_Counter(C_TIMER_TEMP+T_LOOP,10))
                {
                    ADC_Get_Voltage(ADC_VCC,&adcVcc); 
                    ADC_Get_Voltage(ADC_V24,&adc24); 
                    ADC_Get_Voltage(ADC_V24_1,&adc24_1); 
                    tempture = ADC_Get_Inttemp();
                }
                OutBuffer[0] = SOI ;
                //5V电压
                OutBuffer[1] = (unsigned char)(adcVcc/100);
                OutBuffer[2] = (unsigned char)(adcVcc%100);
                //24V马达电压
                OutBuffer[3] = (unsigned char)(adc24/100);
                OutBuffer[4] = (unsigned char)(adc24%100);
                //24V气阀电压
                OutBuffer[5] = (unsigned char)(adc24_1/100);
                OutBuffer[6] = (unsigned char)(adc24_1%100);
                //CPU温度
                OutBuffer[7] = (unsigned char)(tempture/100);
                OutBuffer[8] = (unsigned char)(tempture%100);
                
                unsigned int pm25;
                if(VoiceUart_GetPM25(&pm25) == -1)
                {
                  pm25 = 0x7f7f;
                }
                
                OutBuffer[9] = (unsigned char)(pm25);
                OutBuffer[10] = (unsigned char)(pm25>>8);
                
                OutBuffer[11] = EOI ;
                nOutBufferCount = 12;
                HandUart_Transmit_Packet(OutBuffer,nOutBufferCount);
                bMasterSendPacket = FALSE ;
            }
            break;
        case LINGO_AIRBAG:   //气阀测试
      //      Main_Send_Leg();
            if(bMasterSendPacket)
            {
                OutBuffer[0] = SOI ;
                OutBuffer[1] = airpumpIndex;
                OutBuffer[2] = airbagIndex;
                OutBuffer[3] = EOI ;
                nOutBufferCount = 4;
                HandUart_Transmit_Packet(OutBuffer,nOutBufferCount);
                bMasterSendPacket = FALSE ;
            }
            if(airbagIndex & 0x80)
            {
                airbag = 0xffffff;
                Valve_Test_Set_Data(airbag);
            }
            else//两个气囊在220V以下不能同时充气,,足左，足右
            {
              //小腿上侧上底下侧下底同时充气时，190V可以正常工作，足左和脚后跟在190V时不能同时充气

        //      if(airbagIndex==0)airbag=0xf0;  //经测试小腿气泵在180V不能两个气囊同时充气。其他两个气泵是可以的
           //      if(airbagIndex==0)airbag=0x11;
               //  if(airbagIndex==0)airbag=0x15;//190V只能充2个气囊
              
       //        if(airbagIndex==0)airbag=0x12;//足左气阀有问题
            //     if(airbagIndex==0)airbag=0x07;
              
              
            //    if(airbagIndex==1)airbag=0x00;
                 
          /*         if(airbagIndex==8)airbag=0xc300;
                  if(airbagIndex==9)airbag=0x0000;
                 
                  if(airbagIndex==16)airbag=0x530000;
                  if(airbagIndex==17)airbag=0x000000;    //用于测试气阀*/
                  
                  
                airbag = 1 << airbagIndex;
                Valve_Test_Set_Data(airbag);
            }
            
             
            /* 
           if(airpumpIndex&0x01)
            {
                
            }
            else
            {
                 
            }
            if(airpumpIndex&0x02)
            {
                
            }
            else
            {
                
            }*/
            if(airpumpIndex&0x03 != 0)
            {
                Valve_AirPumpACPowerOn(); //手臂
            }
            else
            {
   
                Valve_AirPumpACPowerOff();
            }        
            
            
            Valve_Send_Data();
            break;
        case LINGO_ENG:  
        case LINGO_MENU:   
             Main_Send_Leg();
            LegKnead_SetPower(LEG_KNEAD_OFF);
            LegKnead_SetSpeed(LEG_KNEAD_SPEED_STOP);
            KnockMotor_Set_Pwm_Data(KNOCK_SPEED0_PWM);
            Roller_SetSpeed(ROLLER_SPEED_STOP);
            KneadMotor_Control(STATE_KNEAD_IDLE,KNEAD_SPEED0_PWM);
            Valve_AirPumpACPowerOff();
            Valve_Test_Set_Data(0);
            {
                if(bMasterSendPacket)
                {
                    OutBuffer[0] = SOI ;
                    
                    unsigned int snH = DEVINFO->UNIQUEH;
                    unsigned int snL = DEVINFO->UNIQUEL;
                    
                    OutBuffer[1] = (unsigned char)(snH >> 24);
                    OutBuffer[2] = (unsigned char)(snH >> 16);
                    OutBuffer[3] = (unsigned char)(snH >> 8);
                    OutBuffer[4] = (unsigned char)(snH);
                    
                    OutBuffer[5] = (unsigned char)(snL >> 24);
                    OutBuffer[6] = (unsigned char)(snL >> 16);
                    OutBuffer[7] = (unsigned char)(snL >> 8);
                    OutBuffer[8] = (unsigned char)(snL);
                    
                    OutBuffer[9] = (unsigned char)ReadEEByte(USER_DATA_BASE+SOFT_MAIN_VER_ADDRESS);
                    OutBuffer[10] = (unsigned char)ReadEEByte(USER_DATA_BASE+SOFT_SECONDARY_VER_ADDRESS);
                    OutBuffer[11] = (unsigned char)ReadEEByte(USER_DATA_BASE+SETTLE_ADDRESS);
                    OutBuffer[12] = (unsigned char)ReadEEByte(USER_DATA_BASE+AIRBAG_STRETCH_ADDRESS);
                    OutBuffer[13] = (unsigned char)ReadEEByte(USER_DATA_BASE+SLIDE_MOTOR_ENABLE_ADDRESS);
                    
                    
                    // OutBuffer[14] = (unsigned char)ReadEEByte(USER_DATA_BASE+ACC_TIME_0_ADDRESS);
                    // OutBuffer[15] = (unsigned char)ReadEEByte(USER_DATA_BASE+ACC_TIME_1_ADDRESS);
                    // OutBuffer[16] = (unsigned char)ReadEEByte(USER_DATA_BASE+ACC_TIME_2_ADDRESS);
                    // OutBuffer[17] = (unsigned char)ReadEEByte(USER_DATA_BASE+ACC_TIME_3_ADDRESS);
                    
                    OutBuffer[14] = EOI ;
                    nOutBufferCount = 15;
                    HandUart_Transmit_Packet(OutBuffer,nOutBufferCount);
                    bMasterSendPacket = FALSE ;
                }
            }
            break;
        case LINGO_PROGRAM:  
            {
                Valve_AirPumpACPowerOff();
                Valve_Test_Set_Data(0);
                if(bMasterSendPacket)
                {
                    if(bProgram)
                    {
                        OutBuffer[0] = SOI ;
                        OutBuffer[1] = 'p';
                        OutBuffer[2] = 'r';
                        OutBuffer[3] = 'o' ;
                        OutBuffer[4] = 'g' ;
                        OutBuffer[5] = 'r' ;
                        OutBuffer[6] = 'a' ;
                        OutBuffer[7] = 'm' ;
                        OutBuffer[8] = EOI ;
                        nOutBufferCount = 9;
                        HandUart_Transmit_Packet(OutBuffer,nOutBufferCount);
                        overCounter++;
                        if(overCounter >= 3)
                        {
                            password = 0;  
                            NVIC_SystemReset(); //复位CPU
                        }
                    }
                    bMasterSendPacket = FALSE ;
                }
            }
            break;
        default:
             Valve_AirPumpACPowerOff();
            Valve_Test_Set_Data(0);
            break;
        }
        /******************************/
          if (engStatus == LINGO_MENU)
          {
             Valve_AirPumpACPowerOff();
              Waveringly_Set_Pwm_Data(0);
          }  
        
         if( engStatus != LINGO_WAVER_TEST)
         {
           Waveringly_Set_Pwm_Data(0);
           
         }
         if( engStatus != LINGO_ROLLER_TEST)
         {
           //Waveringly_Set_Pwm_Data(0);
           Roller_SetSpeed(ROLLER_SPEED_STOP);
         }    
        
    //     Main_Send_Leg(); //处理小腿BEEP声音
        
        if(HandUart_GetCtrlType() != ENGGER_CTRL)
        {
            password = 0;  
            NVIC_SystemReset(); //复位CPU
        }
    }
    Main_Initial_Data(); //重新初始化数据
}

void main_200ms_int(void)
{
  bBlueToothMasterSendPacket = TRUE;
  
 // bCloud_MasterSendPacket=TRUE;
  
 // bCloud_MasterSendHandlePacket=TRUE;
  
  
}

void main_50ms_int(void)
{
    bMasterSendPacket = TRUE;
    bMasterSendLegPacket = TRUE;
    //bSignalSendPacket =TRUE;
}
void zero_100msInt(void)
{
  bZeroTimer100MS = TRUE;
  
}
void main_10ms_int(void)
{
    bTimer10MS = TRUE ;
    engineeringTime_10msFlag = 1;
    bBootlooth10ms = true;
}

void Main_Save_Acctime(void)
{
    unsigned int time;
    time = Data_Get_ProgramExecTime();
    if(time == 0) return;
    Data_Clear_ProgramExecTime();
}
BYTE Main_GetKeyNoClear(void)
{
  BYTE by_Key = H10_KEY_NONE;
  if(HandUart_GetRXStatus() == TRUE)
  {
    //HandUart_ClearRXStatus();
    by_Key = HandUart_GetKey();
    //HandUart_SetKey(H10_KEY_NONE);
  }  
  //if The command is from Bluetooth ,then awake from sleep mode for there's keys arrive.
  if(BlueToothUart_GetRXStatus() == TRUE)
  {
    by_Key = BlueToothUart_GetKey();
  }
  return by_Key;
}
void Main_Sleep(void)
{
    bool bPowerOn = false;
    int powerCounter = 0;
    int ledCounter;
    BYTE key;
    nChairRunState = CHAIR_STATE_SLEEP; 
    nVoicekey = H10_KEY_NONE;
    Power_All_Off();
    unsigned int pw_Information[10];
    bool bInformationUpdate = 0;
    memset(pw_Information,0,sizeof(pw_Information));
    PBYTE pInformation = (PBYTE)pw_Information;
    MEM_Read_Memory(pw_Information,MEMORY_LENGTH_OF_BYTES);
    if(w_PresetTime != (ReadEEByte(USER_DATA_BASE + DEFAULT_TIME_ADDRESS)*60))
    {
       *(pInformation + DEFAULT_TIME_ADDRESS) = w_PresetTime/60; 
       bInformationUpdate = 1;
    }
    if(bBlueToothStatus != ReadEEByte(USER_DATA_BASE + BLUETOOTH_STATUS_ADDRESS))
    {
      *(pInformation + BLUETOOTH_STATUS_ADDRESS) = bBlueToothStatus; 
      bInformationUpdate = 1;
    }
    if( bInformationUpdate)
    {
      MEM_Write_Memory(pw_Information,7*2);
    }
    
    Main_Initial_Data();  //software initial
    BlueToothUart_AMP_Volume_On();//打开WIFI接口的WIFI_IO3  ,该引脚已经被改变
    
   currentBackPadMotorState = STATE_BACK_IDLE;
    while(nChairRunState == CHAIR_STATE_SLEEP)
    {
        if(HandUart_GetCtrlType() == ENGGER_CTRL )
        {
            nChairRunState = CHAIR_STATE_ENGINEERING;

            return;
        }
        
        
         if(BlueToothUart_GetCtrlType()==PROGARM_CTRL)
         {
           
            nChairRunState = CHAIR_STATE_UPDATE;
            return;     
                   
         }
        
        
        if(HandUart_GetCtrlType() == PROGARM_CTRL )
        {

          
            nChairRunState = CHAIR_STATE_UPDATE;
            return;
        }       
        
        key = Main_GetKey();

        
        if(key != H10_KEY_NONE)
        {
            if( key == H10_KEY_POWER_SWITCH ||
                key == H10_KEY_BACKPAD_UP_START ||
                key == H10_KEY_BACKPAD_DOWN_START ||
                key == H10_KEY_LEGPAD_UP_START ||
                key == H10_KEY_LEGPAD_DOWN_START ||
                key == H10_KEY_LEGPAD_EXTEND_START ||
                key == H10_KEY_LEGPAD_CONTRACT_START)
            {
                bPowerOn = true;
                Power_All_On();
                  //Power_3V3_On();
                  //Power_5V_On();
                
                
            }
        }  
        if(key&VOICE_KEY_MASK)
        {
          if((key&VOICE_KEY_MASK) != H10_KEY_POWER_SWITCH)
          {
            bPowerOn = true;
            nVoicekey = key&0x7f;
            Power_All_On();
          }
        }
        if(bTimer10MS == TRUE)//bTimer10MS=全局变量
        {
            ledCounter++;
            ledCounter %= 200;
            bTimer10MS = FALSE ;
            if(bPowerOn)
            {
                powerCounter++;   
                if(powerCounter > 4)//40ms delay
                {
                  nChairRunState = CHAIR_STATE_WAIT_COMMAND; 
                }
            }
            else
            {
                powerCounter = 0;   
            }
            
            BlueToothIntData_to_Cmd_Scan();//10ms scan   in main while
        }
        if(ledCounter < 10)
        {
           IndicateLED_On();
        }
        else
        {
           IndicateLED_Off();
        }
        
        if(bPowerOn)
        {
          Input_Proce();
          Valve_Send_Data();
        }
        Main_Send();//主板与受控器与小腿板通信
        //BlueToothUart_AMP_Volume_On();
        Main_BlueToothSend();//蓝牙通信
        
         Main_WaveMotorStop();
        
        
    }
}
//#define CURRENT_POINT_COUNT 10
//肩位检测时行走电机向下走到最低点，3D电机力度调到最大，然后开始肩位检测，不同的人坐标不一样



#define CURRENT_POINT_COUNT 10

void Auto_Calibration(int detect3D )//bShoulderOK == 0)   // Auto_Calibration(0);  //进入主程序之前，先进行体型检测BodyDetectStep=DETECT_INITIAL
{
    static int steps = 0;
    static unsigned int positionCount,positionTicks;
    bool _b3D_OK,bKnead_OK,bWalk_OK;
    

    static  unsigned char Re_DETECT_SHOULDER=0;
    
    if(BodyDetectStep == DETECT_INITIAL)//(bShoulderOK == 0)
    {
      BodyDetectStep = DETECT_SHOULDER;
      nShoulderPosition = DEFAULT_SHOULDER_POSITION;
      ShoulderSteps = BODY_DETECT_PREPARE;
      steps = 0;
      bShoulderOK = 0;
      

      
    }
       if(DETECT_SHOULDER == BodyDetectStep)
       {
         switch(ShoulderSteps)  
         {
         case BODY_DETECT_PREPARE:   //准备 停止敲击马达 3D马达揉捏头停在最前面，宽位置 
           {
             KnockMotor_Break();
             _b3D_OK = AxisMotor_Control(STATE_RUN_AXIS_VECTOR,2,_3D_SPEED_5);

             bKnead_OK = KneadMotor_Control(STATE_KNEAD_IDLE,KNEAD_SPEED0_PWM);
             bWalk_OK = WalkMotor_Control(STATE_RUN_WALK_UP,0);
             if(_b3D_OK && bKnead_OK && bWalk_OK)
             {
               ShoulderSteps = BODY_DETECT_WALK_POSITION;
             }
           }
           break;  
        case BODY_DETECT_WALK_POSITION:                    //行走电机下行

          
           //bWalk_OK = WalkMotor_Control(STATE_RUN_WALK_POSITION,WAIST_POSITION);//START_CHECK_POSITION);//,在腰部位置向上30个脉冲位置
           bWalk_OK = WalkMotor_Control(STATE_RUN_WALK_POSITION,200);
           
           
           if((bWalk_OK))
           {
             
               /*  if(Re_DETECT_SHOULDER)//for test
                 {nShoulderPosition=Input_GetWalkMotorPosition();
                   printf("W4:%d,vout:%d\n",nShoulderPosition,Re_DETECT_SHOULDER);
                 }*/
             
             ShoulderSteps = BODY_DETECT_KNEAD_MIN;
             
            
           }

          break;
         case BODY_DETECT_KNEAD_MIN:
           bKnead_OK = KneadMotor_Control(STATE_KNEAD_STOP_AT_MIN,KNEAD_SPEED2_PWM);
           if(bKnead_OK)
           { 
             ShoulderSteps = BODY_DETECT_KNEAD_MAX;
             
             
           }
           break;
         case BODY_DETECT_KNEAD_MAX:
           bKnead_OK = KneadMotor_Control(STATE_KNEAD_STOP_AT_MAX,KNEAD_SPEED2_PWM);
           if(bKnead_OK)
           {
             ShoulderSteps = BODY_DETECT_3D_FORWARD;
              
           }
           
           break;  
         case BODY_DETECT_3D_FORWARD:
           //bWalk_OK = WalkMotor_Control(STATE_RUN_WALK_POSITION,START_CHECK_POSITION);//WAIST_POSITION);;,在腰部位置向上30个脉冲位置
           //if((bWalk_OK))
           {
             _b3D_OK = AxisMotor_Control(STATE_RUN_AXIS_VECTOR,4,_3D_SPEED_5);
             if(_b3D_OK )
             {
                 /*  if(Re_DETECT_SHOULDER)
                   {
                    nShoulderPosition=Input_GetWalkMotorPosition();
                   printf("W6:%d,vout:%d\n",nShoulderPosition,Re_DETECT_SHOULDER);
                   }*/
               nShoulderPosition = DEFAULT_SHOULDER_MAX_POSITION;//DEFAULT_SHOULDER_POSITION;//#define DEFAULT_SHOULDER_POSITION	190//该位置应该是3D动作时行走电机可以行走的最大位置，
               ShoulderSteps = BODY_DETECT_UP_AUTO;
             }
           }
           break;

         case BODY_DETECT_UP_AUTO:  //行走马达上行到脖子位置
           
            if(WalkMotor_Control(STATE_RUN_WALK_POSITION,nShoulderPosition))
             {
               /*  该放法为第一次找不到肩位，肩位为默认的最高位置
                 if(Re_DETECT_SHOULDER!=0)
                 {            
                       ShoulderSteps = BODY_DETECT_DATA_REFRESH;
              //       printf("W1:%d,vout:%d\n",nShoulderPosition,BackMotor_Get_Location());//
                      break;
                 }
                 else
                 {
             //        printf("W3:%d,vout:%d\n",nShoulderPosition,BackMotor_Get_Location());
                       ShoulderSteps= BODY_DETECT_WALK_POSITION;
                       Re_DETECT_SHOULDER=1;
                       break;  
                 }*/
    
                //if(WalkMotor_Control(STATE_RUN_WALK_POSITION,nShoulderPosition))
                // {
                   ShoulderSteps = BODY_DETECT_DATA_REFRESH;
    
                // }
               
               
             //  break;

             }
            
            if(Input_GetVout() == BODY_TOUCHED)//光耦揉捏臂被人压下时为OFF ,当没有被压下时即走到肩部位置以上时为ON
            {
              //waitingcount++;
              if((Input_GetVout() == BODY_TOUCHED)&&(Input_GetWalkMotorPosition()>= LIMIT_POSITION))//&&(waitingcount>=1) )
              {
                WalkMotor_Control(STATE_WALK_IDLE,0);
                nShoulderPosition = Input_GetWalkMotorPosition();
                ShoulderSteps = BODY_DETECT_DATA_REFRESH;
             //肩膀位置修正   
                if(nShoulderPosition+LIMIT_PRECISION > TOP_POSITION)
                {
                   nShoulderPosition = TOP_POSITION;
    /*           //    printf("W2:%d,vout:%d\n",nShoulderPosition,Input_GetVout());              
 //                    printf("W2:%d,vout:%d\n",nShoulderPosition,BackMotor_Get_Location());   */ 
                }
                else
                {
                      nShoulderPosition += LIMIT_PRECISION;//当自动程序处于第一零重力位置时，是否考虑肩位检测不加LIMIT_PRECISION
/*
               //      printf("W2:%d,vout:%d\n",nShoulderPosition,Input_GetVout());
  //                    printf("W2:%d,vout:%d\n",nShoulderPosition,BackMotor_Get_Location());   */

                }
       //         printf("W2:%d,vout:%d\n",nShoulderPosition,Input_GetVout());
               //肩膀位置修正完成  
              } 
            }
            else
            {
              //waitingcount =0;
            }
           break;  
         case BODY_DETECT_DATA_REFRESH:  //数据刷新
           {
           
            BodyDataRefresh();
            if(WalkMotor_Control(STATE_RUN_WALK_POSITION,nShoulderPosition))  
            {
             ShoulderSteps = BODY_DETECT_ADJ;//BODY_DETECT_ADJ
             Timer_Counter_Clear(C_TIMER_5);
            }
           }
           break;
         case BODY_DETECT_ADJ:  //揉脖子并调整脖子位置
            KneadMotor_Control(STATE_KNEAD_CLOCK_RUN,KNEAD_SPEED3_PWM);
            if(bKeyWalkUp)
            {
              if(WalkMotor_Control(STATE_RUN_WALK_POSITION,nShoulderPositionTop)) //nShoulderPositionBottom
              {
                nBuzzerMode = BUZZER_MODE_FAST ;
                bSendBuzzerMode = TRUE ;
              }
              else
              {
                nBuzzerMode = BUZZER_MODE_SLOW ;
                bSendBuzzerMode = TRUE ;
                Timer_Counter_Clear(C_TIMER_5);//add by wgh 20150313
              }
            }
            if(bKeyWalkDown)
            {
              if(WalkMotor_Control(STATE_RUN_WALK_POSITION,nShoulderPositionBottom)) //nShoulderPositionBottom
              {
                nBuzzerMode = BUZZER_MODE_FAST ;
                bSendBuzzerMode = TRUE ;
              }
              else
              {
                nBuzzerMode = BUZZER_MODE_SLOW ;
                bSendBuzzerMode = TRUE ;
                Timer_Counter_Clear(C_TIMER_5);//add by wgh 20150313
              }
            }
            if((!bKeyWalkUp) && (!bKeyWalkDown))
            {
              WalkMotor_Control(STATE_WALK_IDLE, 0);
              nBuzzerMode = BUZZER_MODE_OFF ;
              bSendBuzzerMode = TRUE ;
            }
            if(Timer_Counter(C_TIMER_5,10*10))
            {
              nShoulderPosition = Input_GetWalkMotorPosition();  
              ShoulderSteps = BODY_DETECT_OVER;
              break;
            }
            break;
          case BODY_DETECT_OVER:  
            bShoulderOK = 1;
            if((nBackMainRunMode == BACK_MAIN_MODE_AUTO) && ( nBackSubRunMode == BACK_SUB_MODE_AUTO_2)  )
            {
              RockFunctionEnable(true);//////////////////////////////////////////
            }
            //add y wgh 20170208
            
            if(/*(bEnableStretchDemoRun == TRUE) &&*/
               (nBackMainRunMode == BACK_MAIN_MODE_AUTO) &&
                 ((nBackSubRunMode == BACK_SUB_MODE_AUTO_1)/*||(nBackSubRunMode == BACK_SUB_MODE_6MIN_DEMO)*/)
                   &&(  st_Stretch.active == FALSE ))
            {
              st_Stretch.active = TRUE;
              SetStretchingEnable(1);
              st_Stretch.init = TRUE; 
              st_Stretch.times = 3;
              bZeroflash = FALSE;
              //bEnableStretchDemoRun = FALSE;
              if(w_PresetTime == RUN_TIME_10) 
              {
                st_Stretch.times = 4;
              }
              
              
              
              
            }
            
            
            
            
            
            
            Re_DETECT_SHOULDER=0;// 为下次肩位检测失败清0
            
            if(detect3D)          
            {
              BodyDetectStep = DETECT_3D;
            }
            break;
         }
       }
       if(DETECT_3D == BodyDetectStep)
       {
         switch(steps)
         {
         case 0:   //准备
           {
             KnockMotor_Break();
             if(KneadMotor_Control(STATE_KNEAD_STOP_AT_MED,KNEAD_SPEED2_PWM) && (AxisMotor_Control(STATE_RUN_AXIS_FORWARD,0,_3D_SPEED_8)))    
             {
               steps++;
             }
             positionCount = 0;
             positionTicks = nShoulderPosition/_3D_FULL_POINT;   //设置3D电流采集点之间的行走脉冲数量 
           }
           break;
         case 1:  
           if(WalkMotor_Control(STATE_RUN_WALK_POSITION,positionCount*positionTicks)) //到达行走位置点
           {
             steps++;
           }
           break;
         case 2:  //3D马达运动到最后面   
           {
             if(AxisMotor_Control(STATE_RUN_AXIS_BEHIND,0,_3D_SPEED_8))
             {
               steps++;
               AxisMotor_UpdataPosition();
             }
           }
           break;    
         case 3:  //3D马达运动到最前面  
           {
             if(AxisMotor_Control(STATE_RUN_AXIS_FORWARD,0,_3D_SPEED_8))
             {
               steps++;
             }
             AxisMotor_StorageCurrent(positionCount,positionCount*positionTicks);  //存储电流值11*40个点
           }
           break;
         case 4:  //数据记录与处理
           {
             positionCount++;
             if(positionCount > _3D_FULL_POINT )  //一共采集数据_3D_FULL_POINT+1次
             {
               steps++;
               break;
             }
             steps = 1; 
           }
           break;
         case 5:  //结束
           {
             nChairRunState = CHAIR_STATE_RUN;
             BodyDetectStep = DETECT_FINISH;
              // bodyDectect = 1;
             AxisMotor_CurrentAdj();
           }
           break;  
         }//end switch
         } // end if
   /***************程序退出区**************************/
}


//存储模式 ，气阀关闭，滚轮关闭，等， 未考虑零重力电动缸相关电机的关闭
//未考虑小腿伸缩电机的状态
//         nChairRunState = CHAIR_STATE_SETTLE;  //按摩时间到
//         nSettleMode = RUN_OVER_RESET;

//                if(bUpKey) AxisMotor_Control(STATE_RUN_AXIS_FORWARD,0,_3D_SPEED_3);//坐标最大，力度最大 ，向前
//                if(bDownKey) AxisMotor_Control(STATE_RUN_AXIS_BEHIND,0,_3D_SPEED_3);//坐标为0  力度最小  ，向后 AxisMotor_Control(STATE_RUN_AXIS_BEHIND,0,_3D_SPEED_8))

void Main_Settle(void)
{
    bool bEngineeringSettle = ReadEEByte(USER_DATA_BASE+SETTLE_ADDRESS);
    bool waver_finish=0;
    
    unsigned int steps2 = 0;
    BYTE key;
    //变量初始化区域
    //函数初始化区域
    Power_All_On();
 //   VoiceUart_PowerOff();  //复位过程中语音不起作用    delete by taoqignsong
    bKeyBackPadUp = FALSE ;
    bKeyBackPadDown = FALSE ;
    bKeyLegPadUp = FALSE ;
    bKeyLegPadDown = FALSE ;
    nChairRunState = CHAIR_STATE_SETTLE ;//按摩椅处于收藏状态
    
    if(bEnableStretchDemoRun == TRUE)
    {
      bEnableStretchDemoRun = FALSE;//WGH 20161107
    } 
    
    nBackSubRunMode = BACK_SUB_MODE_NO_ACTION ;
    Timer_Counter_Clear(C_TIMER_INDICATE);
    Timer_Counter_Clear(C_TIMER_TEMP);
    bBackLegPadSettle = TRUE ;
    Main_Close_Power();
    
    Valve_AirPumpACPowerOff();
    
    Valve_CloseAll();
    LegKnead_SetPower(LEG_KNEAD_OFF);
    LegKnead_SetSpeed(LEG_KNEAD_SPEED_STOP);
    Roller_SetSpeed(ROLLER_SPEED_STOP);
    
    BackMotor_Control(STATE_BACK_IDLE) ;
    LegMotor_Control(STATE_LEG_IDLE) ;
    SlideMotorControl(STATE_SLIDE_IDLE) ;
    Valve_OzonOff();
    
     WaistHeat_Off();//add by taoqingosng
          //        nChairRunState = CHAIR_STATE_SETTLE;  //按摩时间到
    //     nSettleMode = RUN_OVER_RESET;
     
    //bEngineeringSettle=0，程序运行结束椅子不复位，当按下电源键时椅子复位
    if(bEngineeringSettle || nSettleMode == POWER_KEY_RESET) //如果为真，则关机复位，否则关机不复位
    {
        bBackLegPadSettle = TRUE ;
        nTargetMassagePosition = MASSAGE_RESET_POSITION;
        bMassagePositionUpdate = TRUE;
        w_ZeroPosition = 0;
        
        waver_finish=1;
        
    }
    else
    {
      bMassagePositionUpdate = FALSE;
       waver_finish=1;bZeroflash = FALSE;
      
    }
    BlueToothUart_AMP_Volume_On();
    //主循环
    while(nChairRunState == CHAIR_STATE_SETTLE)
    {
      //按键处理区
        key = Main_GetKey();
        if(key != H10_KEY_NONE)
        {
          Power_All_Off();

          bBackLegPadSettle = FALSE;
          nChairRunState = CHAIR_STATE_SLEEP; 
   
          
        }
        
    /*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */    
      //时间处理区
       if(Timer_Counter(C_TIMER_INDICATE + T_LOOP,SETTLE_INDICATE_TIME))
        {
          
          IndicateLED_Toggle();
        }
       Input_Proce();
       Valve_Send_Data();
       Main_Send();//包含Main_Send_Leg();
       Main_BlueToothSend();
       Main_Massage_Position_Proce();//settle
       Problem_Proce();
       MusicSampling();
      //VoiceUart_Proce();    //delete by taoqingsong
       

    //   if(waver_finish)
   //    {
         
         Main_WaveMotorStop();
         
     //  }
       
       
       
         switch(steps2)  
         {
         case 0:   //揉捏马达停在最宽处
           KnockMotor_Break();
          // if(KneadMotor_Control(STATE_KNEAD_STOP_AT_MAX,KNEAD_SPEED2_PWM))    
           if(KneadMotor_Control(STATE_KNEAD_IDLE,KNEAD_SPEED0_PWM)) 
           {
             steps2++;    
           }
           break;
         case 1:
           
          if(KneadMotor_Control(STATE_KNEAD_STOP_AT_MAX,KNEAD_SPEED3_PWM))    
           {
             steps2++;    
           }
          break;
         case 2:
             if( AxisMotor_Control(STATE_RUN_AXIS_VECTOR,2,_3D_SPEED_5))//3D在中间位置
            {
                steps2++;    
            }  
           break;
          
          
         case 3:   //行走马达停在上行程开关位置
            if(WalkMotor_Control(STATE_RUN_WALK_UP,0))
            {
             steps2++;    
            }  
           break;
           
         case 4:   //行走马达停在复位位置
            if(WalkMotor_Control(STATE_RUN_WALK_POSITION,RESET_POSITION))
            {
             steps2++;    
            }  
           break;
             
        case 5: 
           if( AxisMotor_Control(STATE_RUN_AXIS_BEHIND,0,_3D_SPEED_8))//坐标为0  力度最小 3D 停止在靠后位置
            {
            
                steps2++;    
            }  
           break;
           
       /*  case 6:    
              if(WaveMotor_IsRun() == FLEX_POWER_ON)
              {
                
                if((Input_GetWaveMotorPosition() == REACH_WAVER_LIMIT)||(bWaveMotorFail))  
                {
                  Waveringly_Set_Pwm_Data(VIB_STRENGTH[0]);
                  steps2++;   
                  
                }
                else
                {
                  Waveringly_Set_Pwm_Data(VIB_STRENGTH[1]);
                }
              }
              else
              {
                steps2++;   
                
              }

              break;*/
         case 6://7:      
         default:    
            steps2 = 100;    
            break;
         }

         
         
        if((!bMassagePositionUpdate) && (steps2 == 100))
        {
          
          nChairRunState = CHAIR_STATE_SLEEP; 
        }
        
        if(Timer_Counter(C_TIMER_TEMP+T_LOOP,2*60*10))
        {
          nChairRunState = CHAIR_STATE_SLEEP;   //2分钟时间保护
        }
        
        /* 
        //判断所有的电动缸和机芯是否复位
        if((Input_GetBackUpSwitch() == REACH_BACK_LIMIT) &&
                (Input_GetLegDownSwitch() == REACH_LEG_LIMIT) &&
                   (Input_GetSlideBackwardSwitch() == REACH_SLIDE_LIMIT) &&
                     (steps == 100))
            {
               nChairRunState = CHAIR_STATE_SLEEP; 
            }
        */
        
    } //end while
   /***************程序退出区**************************/
    /*WalkMotor_Control(STATE_WALK_IDLE,0);
    SlideMotorControl(STATE_LEG_IDLE);
    BackMotor_Control(STATE_BACK_IDLE) ;
    LegMotor_Control(STATE_LEG_IDLE) ;
    Waveringly_Set_Pwm_Data(VIB_STRENGTH[0]);
    KneadMotor_Control(STATE_KNEAD_IDLE,KNEAD_SPEED0_PWM);*/
 
    WalkMotor_Control(STATE_WALK_IDLE,0);
    SlideMotorControl(STATE_LEG_IDLE);
    BackMotor_Control(STATE_BACK_IDLE) ;
    BackMotor_Initial_IO();
    LegMotor_Control(STATE_LEG_IDLE) ;
    bMassagePositionUpdate = FALSE;    
    Waveringly_Set_Pwm_Data(VIB_STRENGTH[0]);

    
}
void Main_WaitCommand(void)
{
    BYTE key;
    bool bEnableDemo = false;
    //变量初始化区域
    //函数初始化区域
    Power_All_On();
   //  VoiceUart_PowerOn();//  delelt by taoqingsong
    bBackLegPadSettle = FALSE ;
    bKeyBackPadUp = FALSE ;
    bKeyBackPadDown = FALSE ;
    bKeyLegPadUp = FALSE ;
    bKeyLegPadDown = FALSE ;
    nCurSubFunction = BACK_SUB_MODE_NO_ACTION;
    nChairRunState = CHAIR_STATE_WAIT_COMMAND ;//按摩椅等待按键命令
    Data_Set_Start(0,0);
    nBackSubRunMode = BACK_SUB_MODE_NO_ACTION ;
    Timer_Counter_Clear(C_TIMER_INDICATE);
    Timer_Counter_Clear(C_TIMER_TEMP);
    Timer_Counter_Clear(C_TIMER_500MS);
    Main_Stop_All();
    nBackMainRunMode = BACK_MAIN_MODE_IDLE ;
    if(bEnableStretchDemoRun == TRUE)
    {
      bEnableStretchDemoRun = FALSE;//WGH 20161107
    }
    //主循环
    BlueToothUart_AMP_Volume_On();
    while(nChairRunState == CHAIR_STATE_WAIT_COMMAND)
    {
      //按键处理区
        if(nVoicekey != H10_KEY_NONE)
        {
          key = nVoicekey;
        }
        else
        {
          key = Main_GetKey();
          key &= 0x7f;
        }

        if(H10_KEY_NONE != key)
        {
          Timer_Counter_Clear(C_TIMER_TEMP);
          switch(key)
          {
            /*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */
   //         {
            RockFunctionEnable(false);
            case H10_KEY_MENU:
              break;
              
//
//#define H10_KEY_STRETCH_VIGOR1                   0x06
//#define H10_KEY_STRETCH_VIGOR2                   0x07
//#define H10_KEY_STRETCH_VIGOR3                   0x08  
          case H10_KEY_STRETCH_VIGOR1:
              nStretchVigor =1;
              nBuzzerMode = BUZZER_MODE_ONETIME ;
              bSendBuzzerMode = TRUE ;
              break;
          case H10_KEY_STRETCH_VIGOR2:
              nStretchVigor=2;
              nBuzzerMode = BUZZER_MODE_ONETIME ;
              bSendBuzzerMode = TRUE ;
              break;          
          case H10_KEY_STRETCH_VIGOR3:
              
              nStretchVigor=3;
              nBuzzerMode = BUZZER_MODE_ONETIME ;
              bSendBuzzerMode = TRUE ;
              break;       
              
              
 //++++++++++++++++++++++++++++++++++++++++++++++++++++++         
        case H10_KEY_SWAY_ONOFF:                             //0X76        
          nRockMode = ROCK_MANL;
          RockFunctionEnable(true);
          nChairRunState = CHAIR_STATE_RUN ;
          //nBuzzerMode = BUZZER_MODE_ONETIME ;
          //bSendBuzzerMode = TRUE ;
               
              if(nKeyAirBagLocate != AIRBAG_LOCATE_AUTO)
              {
                nKeyAirBagLocate = AIRBAG_LOCATE_AUTO ;
                  nChairRunState = CHAIR_STATE_RUN ;
                if( Valve_GetAirBagStrength() == AIRBAG_STRENGTH_0)
                {
                  Valve_SetAirBagStrength(AIRBAG_STRENGTH_1);//设定气囊力度为3
                }

                st_AirBagAuto.pAirBagArray = AirBagModeAuto;
                st_AirBagAuto.nTotalSteps = sizeof(AirBagModeAuto)/sizeof(struct AirBagStruct);
                st_AirBagLegFoot.init = TRUE ;
                st_AirBagArmNeck.init = TRUE ;
                st_AirBagAuto.init = TRUE;       
               
                bRollerEnable = TRUE;nRoller3sCnt = 0;
              }
              else
              {
 
              }
              
              nBuzzerMode = BUZZER_MODE_ONETIME ;
              bSendBuzzerMode = TRUE ;               
                
          
          
          
          break;          
//-------------------------------------------------------       
              
              
              
              
              
              
              
//=================USB   MP3================================================
//A2 02 00 5C 
//A2 03 00 5B
//A2 04 00 5A
//A2 05 00 59
//A2 06 00 58
//A2 07 00 57
//A2 08 00 56
//A2 10 0A 44   //BC  1011,1100:  0100,0011 +1  = 0100,0100
/*
                case H10_KEY_USB_PLAY_COMMAND:	
			n_mp3_key = H10_KEY_USB_PLAY_COMMAND;
			n_mp3_key_old = n_mp3_key;
			//
			//if((bBlueToothStatus == 1)&&(n_usb_indicate == USB_SONG_ON))
			{
			n_usb_send_buf[0]= 0xA2;
			n_usb_send_buf[1]= 0x02;
			n_usb_send_buf[2]= 0x00;
			n_usb_send_buf[3]= 0x5C;
			n_usb_send_buf[4]= 0x00;
			//BlueTooth_DMAUart_Transmit_Packet(n_usb_send_buf,4);
                        BlueToothUart_Transmit_Packet(n_usb_send_buf,4);
			}
                    break ;

                case H10_KEY_USB_PAUSE_COMMAND:					
			n_mp3_key = H10_KEY_USB_PAUSE_COMMAND;
			n_mp3_key_old = n_mp3_key;
                    //					
			//if((bBlueToothStatus == 1)&&(n_usb_indicate == USB_SONG_ON))
			{					
			n_usb_send_buf[0]= 0xA2;
			n_usb_send_buf[1]= 0x03;
			n_usb_send_buf[2]= 0x00;
			n_usb_send_buf[3]= 0x5B;
			n_usb_send_buf[4]= 0x00;
			//BlueTooth_DMAUart_Transmit_Packet(n_usb_send_buf,4);
                        BlueToothUart_Transmit_Packet(n_usb_send_buf,4);
			}
                    break ;

                case H10_KEY_USB_STOP_COMMAND:			
			if(H10_KEY_USB_PAUSE_COMMAND == n_mp3_key_old)		
			{
				break;   // 如果上次是PAUSE 那么就不能按STOP
			}
			else	
			{
			n_mp3_key = H10_KEY_USB_STOP_COMMAND;
			n_mp3_key_old = n_mp3_key;
			}
			
                    //
			//if((bBlueToothStatus == 1)&&(n_usb_indicate == USB_SONG_ON))
			{					
			n_usb_send_buf[0]= 0xA2;
			n_usb_send_buf[1]= 0x04;
			n_usb_send_buf[2]= 0x00;
			n_usb_send_buf[3]= 0x5A;
			n_usb_send_buf[4]= 0x00;
			//BlueTooth_DMAUart_Transmit_Packet(n_usb_send_buf,4);
                        BlueToothUart_Transmit_Packet(n_usb_send_buf,4);
			}			
                    break ;

                case H10_KEY_USB_PRE_COMMAND:					
			n_mp3_key = H10_KEY_USB_PRE_COMMAND;
			n_mp3_key_old = n_mp3_key;
                    //					
			//if((bBlueToothStatus == 1)&&(n_usb_indicate == USB_SONG_ON))
			{					
			n_usb_send_buf[0]= 0xA2;
			n_usb_send_buf[1]= 0x05;
			n_usb_send_buf[2]= 0x00;
			n_usb_send_buf[3]= 0x59;
			n_usb_send_buf[4]= 0x00;
			//BlueTooth_DMAUart_Transmit_Packet(n_usb_send_buf,4);
                        BlueToothUart_Transmit_Packet(n_usb_send_buf,4);
			}			
                    break ;
					
                case H10_KEY_USB_NEX_COMMAND:					
			n_mp3_key = H10_KEY_USB_NEX_COMMAND;
			n_mp3_key_old = n_mp3_key;
                    //					
			//if((bBlueToothStatus == 1)&&(n_usb_indicate == USB_SONG_ON))
			{					
			n_usb_send_buf[0]= 0xA2;
			n_usb_send_buf[1]= 0x06;
			n_usb_send_buf[2]= 0x00;
			n_usb_send_buf[3]= 0x58;
			n_usb_send_buf[4]= 0x00;
			//BlueTooth_DMAUart_Transmit_Packet(n_usb_send_buf,4);
                        BlueToothUart_Transmit_Packet(n_usb_send_buf,4);
			}			
                    break ;

                case H10_KEY_USB_VOL_UP_COMMAND:					
			n_mp3_key = H10_KEY_USB_VOL_UP_COMMAND;
			n_mp3_key_old = n_mp3_key;
                    //					
			//if((bBlueToothStatus == 1)&&(n_usb_indicate == USB_SONG_ON))
			{					
			n_usb_send_buf[0]= 0xA2;
			n_usb_send_buf[1]= 0x07;
			n_usb_send_buf[2]= 0x00;
			n_usb_send_buf[3]= 0x57;
			n_usb_send_buf[4]= 0x00;
			//BlueTooth_DMAUart_Transmit_Packet(n_usb_send_buf,4);
                        BlueToothUart_Transmit_Packet(n_usb_send_buf,4);
			}			
                    break ;


                case H10_KEY_USB_VOL_DW_COMMAND:					
			n_mp3_key = H10_KEY_USB_VOL_DW_COMMAND;
			n_mp3_key_old = n_mp3_key;
                    //					
			//if((bBlueToothStatus == 1)&&(n_usb_indicate == USB_SONG_ON))
			{					
			n_usb_send_buf[0]= 0xA2;
			n_usb_send_buf[1]= 0x08;
			n_usb_send_buf[2]= 0x00;
			n_usb_send_buf[3]= 0x56;
			n_usb_send_buf[4]= 0x00;
			//BlueTooth_DMAUart_Transmit_Packet(n_usb_send_buf,4);
                        BlueToothUart_Transmit_Packet(n_usb_send_buf,4);
			}
                    break ;  
*/              
//---------------USB MP3----------------------------------------------------------           
              
              
              
            case H10_KEY_POWER_SWITCH: //再按一次电源键椅子进入到复位存储状态
              
                bBackLegPadSettle = TRUE ;
                nChairRunState = CHAIR_STATE_SETTLE ;
                nSettleMode = POWER_KEY_RESET;                 
                                       
               nTargetMassagePosition = MASSAGE_RESET_POSITION;
               bMassagePositionUpdate = TRUE;
               w_ZeroPosition = 0; //零重力电机归位
                           
              nBuzzerMode = BUZZER_MODE_ONETIME ;
              bSendBuzzerMode = TRUE ;
              break ;
            case H10_KEY_BLUETOOTH_POWER_SWITCH:
              if(bBlueToothStatus)
              {
                bBlueToothStatus = 0;
              }
              else
              {
                bBlueToothStatus = 1;
              }
              nBuzzerMode = BUZZER_MODE_ONETIME ;
              bSendBuzzerMode = TRUE ;
              break;
            case H10_KEY_CHAIR_AUTO_0:             //椅子进入到运行状态，主程序进入到自动状态,具体的自动状态为AUTO_1
              nChairRunState = CHAIR_STATE_RUN ;
              nBackMainRunMode = BACK_MAIN_MODE_AUTO;
              nBackSubRunMode = BACK_SUB_MODE_AUTO_0;
           /*   if(nTargetMassagePosition == MASSAGE_RESET_POSITION)
              {
                nTargetMassagePosition =MASSAGE_OPTIMAL2_POSITION;
                bMassagePositionUpdate = TRUE;
                w_ZeroPosition = 1;
              }        */
              
              
              
              break ;
            case H10_KEY_CHAIR_AUTO_1:
              nChairRunState = CHAIR_STATE_RUN ;
              nBackMainRunMode = BACK_MAIN_MODE_AUTO;
              nBackSubRunMode = BACK_SUB_MODE_AUTO_1;
             /* if(nTargetMassagePosition == MASSAGE_RESET_POSITION)
              {
                nTargetMassagePosition = MASSAGE_OPTIMAL2_POSITION;
                bMassagePositionUpdate = TRUE;
                w_ZeroPosition = 1;
              }                   */
              
              
              break ;
            case H10_KEY_CHAIR_AUTO_2:
              nChairRunState = CHAIR_STATE_RUN ;
              nBackMainRunMode = BACK_MAIN_MODE_AUTO;
              nBackSubRunMode = BACK_SUB_MODE_AUTO_2;
              /* if(nTargetMassagePosition == MASSAGE_RESET_POSITION)
              {
                nTargetMassagePosition = MASSAGE_OPTIMAL2_POSITION;
                bMassagePositionUpdate = TRUE;
                w_ZeroPosition = 1;
              }                */    
              
              
              break ;
            case H10_KEY_CHAIR_AUTO_3:
              nChairRunState = CHAIR_STATE_RUN ;
              nBackMainRunMode = BACK_MAIN_MODE_AUTO;
              nBackSubRunMode = BACK_SUB_MODE_AUTO_3;
             /* if(nTargetMassagePosition == MASSAGE_RESET_POSITION)
              {
                nTargetMassagePosition = MASSAGE_OPTIMAL2_POSITION;
                bMassagePositionUpdate = TRUE;
                w_ZeroPosition = 1;
              }                */     
              
              
              
              break ;
            case H10_KEY_CHAIR_AUTO_4:
              nChairRunState = CHAIR_STATE_RUN ;
              nBackMainRunMode = BACK_MAIN_MODE_AUTO;
              nBackSubRunMode = BACK_SUB_MODE_AUTO_4;
            /*  if(nTargetMassagePosition == MASSAGE_RESET_POSITION)
              {
                nTargetMassagePosition = MASSAGE_OPTIMAL2_POSITION;
                bMassagePositionUpdate = TRUE;
                w_ZeroPosition = 1;
              }                     */
              
              
              
              break ;
            case H10_KEY_CHAIR_AUTO_5:
              nChairRunState = CHAIR_STATE_RUN ;
              nBackMainRunMode = BACK_MAIN_MODE_AUTO;
              nBackSubRunMode = BACK_SUB_MODE_AUTO_5;
          /*     if(nTargetMassagePosition == MASSAGE_RESET_POSITION)
              {
                nTargetMassagePosition = MASSAGE_OPTIMAL2_POSITION;
                bMassagePositionUpdate = TRUE;
                w_ZeroPosition = 1;
              }                    */
              
              
              
              break ;      
              

       /*    case H10_KEY_CHAIR_CLOUD_0:

      
             if( pCludeAutoFunction_0 == NULL)
             {
                 nBuzzerMode = BUZZER_MODE_SLOW ;
                 bSendBuzzerMode = TRUE ;
                 bMasterSendPacket=1;
               break;            
             }
             
              nChairRunState = CHAIR_STATE_RUN ;
              nBackMainRunMode = BACK_MAIN_MODE_AUTO;
              nBackSubRunMode = BACK_SUB_MODE_CLUDE_AUTO_0;*/
            /*  if(nTargetMassagePosition == MASSAGE_RESET_POSITION)
              {
                nTargetMassagePosition = MASSAGE_OPTIMAL2_POSITION;
                bMassagePositionUpdate = TRUE;
                w_ZeroPosition = 1;
              }          */
            
            
               // break;
            
          /*  case H10_KEY_CHAIR_CLOUD_1:

              if( pCludeAutoFunction_1 == NULL)
              {
                    nBuzzerMode = BUZZER_MODE_SLOW ;
                    bSendBuzzerMode = TRUE ;     
                      bMasterSendPacket=1;
                
                   break;     
              }
            
              nChairRunState = CHAIR_STATE_RUN ;
              nBackMainRunMode = BACK_MAIN_MODE_AUTO;
              nBackSubRunMode = BACK_SUB_MODE_CLUDE_AUTO_1;*/
            /*  if(nTargetMassagePosition == MASSAGE_RESET_POSITION)
              {
                nTargetMassagePosition = MASSAGE_OPTIMAL2_POSITION;
                bMassagePositionUpdate = TRUE;
                w_ZeroPosition = 1;
              }     */
            
                //break;           
         /*   case H10_KEY_CHAIR_CLOUD_2:

              
              if( pCludeAutoFunction_2 == NULL)
              {
                
                    nBuzzerMode = BUZZER_MODE_SLOW ;
                    bSendBuzzerMode = TRUE ;
                    bMasterSendPacket=1;
                
                
                break; 
              }
            
              nChairRunState = CHAIR_STATE_RUN ;
              nBackMainRunMode = BACK_MAIN_MODE_AUTO;
              nBackSubRunMode = BACK_SUB_MODE_CLUDE_AUTO_2;*/
            /*  if(nTargetMassagePosition == MASSAGE_RESET_POSITION)
              {
                nTargetMassagePosition = MASSAGE_OPTIMAL2_POSITION;
                bMassagePositionUpdate = TRUE;
                w_ZeroPosition = 1;
              }            */

               // break;           
            case H10_KEY_CHAIR_CLOUD_3:

              
              if( pCludeAutoFunction_3 == NULL)
              {
                   nBuzzerMode = BUZZER_MODE_SLOW ;
                   bSendBuzzerMode = TRUE ;
                   bMasterSendPacket=1;
                  
                break;     
              }
              
               nChairRunState = CHAIR_STATE_RUN ;
              nBackMainRunMode = BACK_MAIN_MODE_AUTO;
              nBackSubRunMode = BACK_SUB_MODE_CLUDE_AUTO_3;
           /*   if(nTargetMassagePosition == MASSAGE_RESET_POSITION)
              {
                nTargetMassagePosition = MASSAGE_OPTIMAL2_POSITION;
                bMassagePositionUpdate = TRUE;
                w_ZeroPosition = 1;
              }       */
            
            
                break;     
              
                
                
                
                
            case H10_KEY_3DMODE_1://  //3D菜单中的3D手法1       //椅子进入到运行状态，主程序进入到自3D状态,具体的自动状态为3D1
              nBackSubRunMode = BACK_SUB_MODE_3D1;
              nBackMainRunMode = BACK_MAIN_MODE_3D;
              nChairRunState = CHAIR_STATE_RUN ;
              nKeyBackLocate =  LOCATE_FULL_BACK;
              
              
              break;
            case H10_KEY_3DMODE_2://3D菜单中的3D手法2
              nBackSubRunMode = BACK_SUB_MODE_3D2;
              nBackMainRunMode = BACK_MAIN_MODE_3D;
              nChairRunState = CHAIR_STATE_RUN ;
                nKeyBackLocate =  LOCATE_FULL_BACK;
              
              break;  
            case H10_KEY_3DMODE_3:////3D菜单中的3D手法3
              nBackSubRunMode = BACK_SUB_MODE_3D3;
              nBackMainRunMode = BACK_MAIN_MODE_3D;
              nChairRunState = CHAIR_STATE_RUN ;
                nKeyBackLocate =  LOCATE_FULL_BACK;
              
              break;  
            case H10_KEY_3DMODE:
              nBackSubRunMode = BACK_SUB_MODE_3D1;
              nBackMainRunMode = BACK_MAIN_MODE_3D;
              nChairRunState = CHAIR_STATE_RUN ;
              break;      
      //     case H10_KEY_ZERO_START://在等待命令模式零重力按键无效，为确认键
      //       break;
           case H10_KEY_ZERO_START://零重力键值  wait
                           //if(isZeroPosition())
                           // {
                           //   nTargetMassagePosition = MASSAGE_INIT_POSITION;
                           // }
                           // else
                           // {
                            //  nTargetMassagePosition = MASSAGE_OPTIMAL2_POSITION;
                           // }
                           // bMassagePositionUpdate = TRUE;
                           // nBuzzerMode = BUZZER_MODE_ONETIME ;
                          //  bSendBuzzerMode = TRUE ;
         /*     if(isZeroPosition())
              {
                nTargetMassagePosition = MASSAGE_INIT_POSITION;
              }
              else
              {
                nTargetMassagePosition = MASSAGE_OPTIMAL2_POSITION;bZeroflash = TRUE;
              }       
             
             */
             
                 if(st_Stretch.active) 
                {
                  nBuzzerMode = BUZZER_MODE_TWOTIME ;
                  bSendBuzzerMode = TRUE ;
                  break ;
                }
                w_ZeroPosition++;
                w_ZeroPosition %= 3;  //0位置为初始位置   1位置为第一个零重力点    2位置为第二个零重力点    
                
                if(w_ZeroPosition == 0)
                {
                  nTargetMassagePosition = MASSAGE_INIT_POSITION;
                }
                if(w_ZeroPosition == 1)
                {
                  nTargetMassagePosition = MASSAGE_OPTIMAL_POSITION;//MASSAGE_OPTIMAL_POSITION;
                }
                if(w_ZeroPosition == 2)
                {
                  nTargetMassagePosition = MASSAGE_OPTIMAL2_POSITION;
                }
                
                
                bMassagePositionUpdate = TRUE;
                nBuzzerMode = BUZZER_MODE_ONETIME ;
                bSendBuzzerMode = TRUE ;            
              
                break;
                
        case H10_KEY_ZERO_1:	
          RockFunctionEnable(false);
          st_Stretch.active = FALSE;
          SetStretchingEnable(0);

            nTargetMassagePosition = MASSAGE_OPTIMAL_POSITION;//MASSAGE_OPTIMAL_POSITION;
          bMassagePositionUpdate = TRUE;
          nBuzzerMode = BUZZER_MODE_ONETIME ;
          bSendBuzzerMode = TRUE ;
          
          break;
          case H10_KEY_TAPPING_ONOFF :
            
            if(bTapping == 0)
            {
              bTapping = 1;
            }
            else
            {
              bTapping = 0;
            }
            
            
            break;
          
          
          
        case H10_KEY_ZERO_2:	
          RockFunctionEnable(false);
          st_Stretch.active = FALSE;
          SetStretchingEnable(0);
   
            nTargetMassagePosition = MASSAGE_OPTIMAL2_POSITION;//MASSAGE_OPTIMAL_POSITION;
          bMassagePositionUpdate = TRUE;
          nBuzzerMode = BUZZER_MODE_ONETIME ;
          bSendBuzzerMode = TRUE ;
          
          break;
        case H10_KEY_ZERO_OFF:	
          RockFunctionEnable(false);
          st_Stretch.active = FALSE;
          SetStretchingEnable(0);

          nTargetMassagePosition = MASSAGE_INIT_POSITION;

          bMassagePositionUpdate = TRUE;
          nBuzzerMode = BUZZER_MODE_ONETIME ;
          bSendBuzzerMode = TRUE ;
          
          break;
                
            case  H10_KEY_3D_STRENGTH:    //受控器自动进入到3D菜单
              break;
              /*
              if((nBackMainRunMode == BACK_MAIN_MODE_MANUAL) || (nBackMainRunMode == BACK_MAIN_MODE_AUTO))
              {
                nKeyAxisStrength++;
                nKeyAxisStrength %= 5;
                bAxisUpdate = TRUE;
                nBuzzerMode = BUZZER_MODE_ONETIME ;
                bSendBuzzerMode = TRUE ;   
              }
              */
    
              
            case H10_KEY_WORK_TIME_10MIN:
              w_PresetTime = RUN_TIME_10;
              Data_Update_Time(w_PresetTime);
              nBuzzerMode = BUZZER_MODE_ONETIME ;
              bSendBuzzerMode = TRUE ;
              break;
            case H10_KEY_WORK_TIME_20MIN:
              w_PresetTime = RUN_TIME_20;
              Data_Update_Time(w_PresetTime);
              nBuzzerMode = BUZZER_MODE_ONETIME ;
              bSendBuzzerMode = TRUE ;
              break;
            case H10_KEY_WORK_TIME_30MIN:
              w_PresetTime = RUN_TIME_30;
              Data_Update_Time(w_PresetTime);
              nBuzzerMode = BUZZER_MODE_ONETIME ;
              bSendBuzzerMode = TRUE ;
              break;
              
            
            case H10_KEY_AIRBAG_STRENGTH_1:
              break;
            case H10_KEY_AIRBAG_STRENGTH_2:
              break;
            case H10_KEY_AIRBAG_STRENGTH_3:
              break;
            case H10_KEY_AIRBAG_STRENGTH_4:
              break;
            case H10_KEY_AIRBAG_STRENGTH_5:
              break;  
            case H10_KEY_AIRBAG_STRENGTH_OFF:
              break;    
              
            case H10_KEY_3DSPEED_1:
            case H10_KEY_3DSPEED_2:
            case H10_KEY_3DSPEED_3:
            case H10_KEY_3DSPEED_4:
            case H10_KEY_3DSPEED_5:
              /*
              if((nBackMainRunMode == BACK_MAIN_MODE_MANUAL) || (nBackMainRunMode == BACK_MAIN_MODE_AUTO))    
              {
                nKeyAxisStrength = H10_KEY_3DSPEED_5 - key;
                nKeyAxisStrength %= 5;
                bAxisUpdate = TRUE;
                nBuzzerMode = BUZZER_MODE_ONETIME ;
                bSendBuzzerMode = TRUE ;   
              }
              */
              break;
            case H10_KEY_KNEAD:
              nMaunalSubMode = nMaunalSubMode_KNEAD;
              nBackMainRunMode = BACK_MAIN_MODE_MANUAL;
              nChairRunState = CHAIR_STATE_RUN ;
              break;
          case H10_KEY_KNEAD_CLOCK:
            nKneadTurn = 1;
            nMaunalSubMode = nMaunalSubMode_KNEAD;
            nBackMainRunMode = BACK_MAIN_MODE_MANUAL;
            nChairRunState = CHAIR_STATE_RUN ;
            
            break;
          case H10_KEY_KNEAD_ANTICLOCK:
            nKneadTurn = 2;
            nMaunalSubMode = nMaunalSubMode_KNEAD;
            nBackMainRunMode = BACK_MAIN_MODE_MANUAL;
            nChairRunState = CHAIR_STATE_RUN ;
            break;  
              
              
              
            case H10_KEY_KNOCK:
              nMaunalSubMode = nMaunalSubMode_KNOCK;
              nBackMainRunMode = BACK_MAIN_MODE_MANUAL;
              nChairRunState = CHAIR_STATE_RUN ;
              break;
              
            case H10_KEY_WAVELET:
              nMaunalSubMode = nMaunalSubMode_WAVELET;
              nBackMainRunMode = BACK_MAIN_MODE_MANUAL;
              nChairRunState = CHAIR_STATE_RUN ;
              break;
              
            case H10_KEY_SOFT_KNOCK:
              nMaunalSubMode = nMaunalSubMode_SOFT_KNOCK;
              nBackMainRunMode = BACK_MAIN_MODE_MANUAL;
              nChairRunState = CHAIR_STATE_RUN ;
              break;
              
            case H10_KEY_PRESS:
              nMaunalSubMode = nMaunalSubMode_PRESS;
              nBackMainRunMode = BACK_MAIN_MODE_MANUAL;
              nChairRunState = CHAIR_STATE_RUN ;
              break;
              
            case H10_KEY_MUSIC:             
              nMaunalSubMode = nMaunalSubMode_MUSIC;
              nBackMainRunMode = BACK_MAIN_MODE_MANUAL;
              nChairRunState = CHAIR_STATE_RUN ;////椅子由等待命令进入到运行状态
              break;  
            case H10_KEY_MANUAL:
              nBackMainRunMode = BACK_MAIN_MODE_MANUAL;
              nMaunalSubMode = 5;
              nChairRunState = CHAIR_STATE_RUN ;//
              break;
              
            case H10_KEY_LOCATE_FULL:
            case H10_KEY_LOCATE_POINT:
            case H10_KEY_LOCATE_PART:  
              break;
         /*   case H10_KEY_OZON_SWITCH:////负离子
              bOzonEnable = TRUE;
              nChairRunState = CHAIR_STATE_RUN ;////椅子由等待命令进入到运行状态
               if(Data_Get_Time() == 0)
                {
                  Data_Set_Start(1, w_PresetTime);
                }
              nBuzzerMode = BUZZER_MODE_ONETIME ;
              bSendBuzzerMode = TRUE ;
              break;  */
              
              
//            case H10_KEY_SPEED_DECREASE:
            case H10_KEY_SPEED_1:
            case H10_KEY_SPEED_2:
            case H10_KEY_SPEED_3:
            case H10_KEY_SPEED_4:
            case H10_KEY_SPEED_5:  
            case H10_KEY_SPEED_6:
              break ;
            case H10_KEY_WIDTH_INCREASE:
            case H10_KEY_WIDTH_DECREASE:
            case H10_KEY_WIDTH_MIN:  
            case H10_KEY_WIDTH_MED:  
            case H10_KEY_WIDTH_MAX: 
              break ;
      //==============================================================        
             case H10_KEY_AIRBAG_AUTO:
              
               
              if(nKeyAirBagLocate != AIRBAG_LOCATE_AUTO)
              {
                nKeyAirBagLocate = AIRBAG_LOCATE_AUTO ;
                  nChairRunState = CHAIR_STATE_RUN ;
                if( Valve_GetAirBagStrength() == AIRBAG_STRENGTH_0)
                {
                  Valve_SetAirBagStrength(AIRBAG_STRENGTH_1);//设定气囊力度为3
                }

                st_AirBagAuto.pAirBagArray = AirBagModeAuto;
                st_AirBagAuto.nTotalSteps = sizeof(AirBagModeAuto)/sizeof(struct AirBagStruct);
                st_AirBagLegFoot.init = TRUE ;
                st_AirBagArmNeck.init = TRUE ;
                st_AirBagAuto.init = TRUE;       
               
                bRollerEnable = TRUE;nRoller3sCnt = 0;
              }
              else
              {
 
              }
              
              nBuzzerMode = BUZZER_MODE_ONETIME ;
              bSendBuzzerMode = TRUE ;               
               
               
               
               
               
              break ;              
              
              
              
              
            case H10_KEY_AIRBAG_LEG:    //选中腿脚气囊按摩程序
              nKeyAirBagLocate = AIRBAG_LOCATE_LEG_FOOT ;
              st_AirBagLegFoot.init = TRUE ;
              if(Valve_GetAirBagStrength() == AIRBAG_STRENGTH_0)
              {
                Valve_SetAirBagStrength(AIRBAG_STRENGTH_1);
              }
              nChairRunState = CHAIR_STATE_RUN ;
              nBuzzerMode = BUZZER_MODE_ONETIME ;
              bSendBuzzerMode = TRUE ;
              break;

            case H10_KEY_AIRBAG_WAIST://背腰气囊按摩程序
             /* nKeyAirBagLocate = AIRBAG_LOCATE_BACK_WAIST ;
              st_AirBagBackWaist.init = TRUE ;
              if( Valve_GetAirBagStrength() == AIRBAG_STRENGTH_0)
              {
                Valve_SetAirBagStrength(AIRBAG_STRENGTH_3);
              }
              nChairRunState = CHAIR_STATE_RUN ;
              nBuzzerMode = BUZZER_MODE_ONETIME ;
              bSendBuzzerMode = TRUE ;*/
              break;
            case H10_KEY_AIRBAG_BUTTOCKS:  //屁股气囊按摩程序
              nKeyAirBagLocate = AIRBAG_LOCATE_SEAT ;
              st_AirBagSeat.init = TRUE ;
              if( Valve_GetAirBagStrength() == AIRBAG_STRENGTH_0)
              {
                Valve_SetAirBagStrength(AIRBAG_STRENGTH_1);
              }
              nChairRunState = CHAIR_STATE_RUN ;////椅子由等待命令进入到运行状态
              nBuzzerMode = BUZZER_MODE_ONETIME ;
              bSendBuzzerMode = TRUE ;
              break;  
              
              
          case H10_KEY_AIRBAG_ARM:
               nKeyAirBagLocate = AIRBAG_LOCATE_ARM_NECK ;//AIRBAG_LOCATE_ARM_NECK
              st_AirBagArmNeck.init = TRUE ;//st_AirBagArmNeck
              if( Valve_GetAirBagStrength() == AIRBAG_STRENGTH_0)
              {
                Valve_SetAirBagStrength(AIRBAG_STRENGTH_1);
              }
              nChairRunState = CHAIR_STATE_RUN ;
              nBuzzerMode = BUZZER_MODE_ONETIME ;
              bSendBuzzerMode = TRUE ;       
  
              break;
    //===========================================================================          
            case H10_KEY_WALK_UP_START:
              Timer_Counter_Clear(C_TIMER_500MS);
              bEnableDemo = true;
              break ;
            case H10_KEY_WALK_UP_STOP:
              Timer_Counter_Clear(C_TIMER_500MS);
              bEnableDemo = false;
              break ;
            case H10_KEY_WALK_DOWN_START:
              Timer_Counter_Clear(C_TIMER_500MS);
              bEnableStretchDemo = true;
              break ;
            case H10_KEY_WALK_DOWN_STOP:
              Timer_Counter_Clear(C_TIMER_500MS);
              bEnableStretchDemo = FALSE;
              break ;
              
            case H10_KEY_BACKPAD_UP_START:
              Timer_Counter_Clear(C_TIMER_TEMP);         
              bKeyBackPadUp = TRUE ;
              bKeyBackPadDown = FALSE ;
              bKeyLegPadDown = TRUE ;
              bKeyLegPadUp = FALSE ;
              bLegPadLinkage = TRUE ;////小腿单独动，此时不考虑前滑电动缸的位置
              bKeyFlexOut = FALSE ;
              bKeyFlexIn = FALSE ;bKeyFlexIn = FALSE ;bZeroflash = FALSE;
              break ;
            case H10_KEY_BACKPAD_UP_STOP:
              st_Stretch.active = FALSE;
              bKeyBackPadUp = FALSE ;
              bKeyBackPadDown = FALSE ;
              bKeyLegPadDown = FALSE ;
              bKeyLegPadUp = FALSE ;
              bLegPadLinkage = TRUE ;
              bKeyFlexOut = FALSE ;
              bKeyFlexIn = FALSE ;
              break ;
            case H10_KEY_BACKPAD_DOWN_START:
              Timer_Counter_Clear(C_TIMER_TEMP);         
              bKeyBackPadUp = FALSE ;
              bKeyBackPadDown = TRUE ;//靠背向下按键按下的标志位
              bKeyLegPadDown = FALSE ;
              bKeyLegPadUp = TRUE ;
              bLegPadLinkage = TRUE ;
              bKeyFlexOut = FALSE ;
              bKeyFlexIn = FALSE ;bKeyFlexIn = FALSE ;bZeroflash = FALSE;
              break ;
            case H10_KEY_BACKPAD_DOWN_STOP://释放按键发送的键值
              bKeyBackPadUp = FALSE ;
              bKeyBackPadDown = FALSE ;
              bKeyLegPadDown = FALSE ;
              bKeyLegPadUp = FALSE ;
              bLegPadLinkage = TRUE ;
              bKeyFlexOut = FALSE ;
              bKeyFlexIn = FALSE ;
              break ;
            case H10_KEY_LEGPAD_EXTEND_START://小腿伸
              Timer_Counter_Clear(C_TIMER_TEMP);         
              bKeyLegPadUp = FALSE ;
              bKeyLegPadDown = FALSE ;
              bLegPadLinkage = FALSE ;
              bKeyBackPadUp = FALSE ;
              bKeyBackPadDown = FALSE ;
              bKeyFlexOut = TRUE ;
              bKeyFlexIn = FALSE ;
              break;
            case H10_KEY_LEGPAD_EXTEND_STOP:
            case H10_KEY_LEGPAD_CONTRACT_STOP:
              bKeyLegPadUp = FALSE ;
              bKeyLegPadDown = FALSE ;
              bLegPadLinkage = FALSE ;
              bKeyBackPadUp = FALSE ;
              bKeyBackPadDown = FALSE ;
              bKeyFlexOut = FALSE;
              bKeyFlexIn = FALSE ;
              break;
            case H10_KEY_LEGPAD_CONTRACT_START:
              Timer_Counter_Clear(C_TIMER_TEMP);         
              bKeyLegPadUp = FALSE ;
              bKeyLegPadDown = FALSE ;
              bLegPadLinkage = FALSE ;
              bKeyBackPadUp = FALSE ;
              bKeyBackPadDown = FALSE ;
              bKeyFlexOut = FALSE;
              bKeyFlexIn = TRUE ;
              break;
            case H10_KEY_LEGPAD_UP_START:
              Timer_Counter_Clear(C_TIMER_TEMP);         
              bKeyLegPadUp = TRUE ;
              bKeyLegPadDown = FALSE ;
              bLegPadLinkage = FALSE ;
              bKeyBackPadUp = FALSE ;
              bKeyBackPadDown = FALSE ;
              bKeyFlexOut = FALSE ;
              bKeyFlexIn = FALSE ;bKeyFlexIn = FALSE ;bZeroflash = FALSE;
              break ;
            case H10_KEY_LEGPAD_UP_STOP:
              bKeyLegPadUp = FALSE ;
              bKeyLegPadDown = FALSE ;
              bLegPadLinkage = FALSE ;
              bKeyBackPadUp = FALSE ;
              bKeyBackPadDown = FALSE ;
              bKeyFlexOut = FALSE ;
              bKeyFlexIn = FALSE ;
              break ;
            case H10_KEY_LEGPAD_DOWN_START:
              Timer_Counter_Clear(C_TIMER_TEMP);         
              bKeyLegPadUp = FALSE ;
              bKeyLegPadDown = TRUE ;
              bLegPadLinkage = FALSE ;
              bKeyBackPadUp = FALSE ;
              bKeyBackPadDown = FALSE ;
              bKeyFlexOut = FALSE ;
              bKeyFlexIn = FALSE ;bKeyFlexIn = FALSE ;bZeroflash = FALSE;
              break ;
            case H10_KEY_LEGPAD_DOWN_STOP:
              bKeyLegPadUp = FALSE ;
              bKeyLegPadDown = FALSE ;
              bLegPadLinkage = FALSE ;
              bKeyBackPadUp = FALSE ;
              bKeyBackPadDown = FALSE ;
              bKeyFlexOut = FALSE ;
              bKeyFlexIn = FALSE ;
              break ;
              
            case H10_KEY_WHEEL_SPEED_OFF:
              break;
            case H10_KEY_WHEEL_SPEED_SLOW:
            case H10_KEY_WHEEL_SPEED_MED:
            case H10_KEY_WHEEL_SPEED_FAST:
              bRollerEnable = TRUE;nRoller3sCnt = 0;
              if(key ==  H10_KEY_WHEEL_SPEED_SLOW)
              {
                nRollerPWM = 1;
              }
              if(key ==  H10_KEY_WHEEL_SPEED_MED)
              {
                nRollerPWM = 2;
              }
              if(key ==  H10_KEY_WHEEL_SPEED_FAST)
              {
                nRollerPWM = 3;
              }
              Valve_SetRollerPWM(nRollerPWM);
              if(nRollerPWM != 0)
              {
                nChairRunState = CHAIR_STATE_RUN ;////椅子由等待命令进入到运行状态
                if(Data_Get_Time() == 0)
                {
                  Data_Set_Start(1, w_PresetTime);
                }
              }
              nBuzzerMode = BUZZER_MODE_ONETIME ;
              bSendBuzzerMode = TRUE ;
              break;
            case H10_KEY_HEAT:    //加热
              if(bKeyWaistHeat == FALSE)
              {
                bKeyWaistHeat = TRUE ;
                nChairRunState = CHAIR_STATE_RUN ;//椅子由等待命令进入到运行状态
              }
              else
              {
                bKeyWaistHeat = FALSE ;
              }
              nBuzzerMode = BUZZER_MODE_ONETIME ;
              bSendBuzzerMode = TRUE ;
              break;     
  //-------------------------------------------------------------            
         /*   case H10_KEY_SEAT_VIBRATE_0:   //摇摆按键 摇摆关
               break ;                                     
             case H10_KEY_SEAT_VIBRATE_1:    
             bKeySeatVibrate = TRUE ;
                nKeySeatVibrateStrength = 1;
                nChairRunState = CHAIR_STATE_RUN ;           
                if(Data_Get_Time() == 0)
                {
                  Data_Set_Start(1,w_PresetTime); 
                }          
                 nBuzzerMode = BUZZER_MODE_ONETIME ;
                 bSendBuzzerMode = TRUE ;     
                 break;                          
           case H10_KEY_SEAT_VIBRATE_2:                    
                 bKeySeatVibrate = TRUE ;
                 nKeySeatVibrateStrength = 2;               
                  __no_operation();
                 __no_operation();
                 __no_operation();
                 __no_operation();
                if(Data_Get_Time() == 0)
                {
                 Data_Set_Start(1,w_PresetTime); 
                }
                nChairRunState = CHAIR_STATE_RUN ;               
                nBuzzerMode = BUZZER_MODE_ONETIME ;
                bSendBuzzerMode = TRUE ;                   
                break;    */         
          /*  case H10_KEY_SEAT_VIBRATE_3:    
              bKeySeatVibrate = TRUE ;
               nKeySeatVibrateStrength = 3;
               nChairRunState = CHAIR_STATE_RUN ;           
                if(Data_Get_Time() == 0)
                {
                  Data_Set_Start(1,w_PresetTime); 
                }                            
              nBuzzerMode = BUZZER_MODE_ONETIME ;
              bSendBuzzerMode = TRUE ;     
              break;*/
  //-----------------------------------------------------------------------            
            default:       
              break;
            }
          //}/
        }
        
    /*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */    
      //时间处理区
        
        if( bBootlooth10ms == true)
        {
          bBootlooth10ms = false;
          USB_MP3_SCAN_PROC();
        }
        
        
        
        
        if(bEnableDemo)
        {
          if(Timer_Counter(C_TIMER_500MS,50))//5SEC后进入DEMO程序
          {
             nChairRunState = CHAIR_STATE_DEMO; 
             bEnableDemo = FALSE;
          }
        }
        else if(bEnableStretchDemo)
        {
          if(Timer_Counter(C_TIMER_500MS,50))
          {
            nChairRunState = CHAIR_STATE_RUN ;
            nBackMainRunMode = BACK_MAIN_MODE_AUTO;
            nBackSubRunMode = BACK_SUB_MODE_6MIN_DEMO;//BACK_SUB_MODE_AUTO_1;//20170408
            bEnableStretchDemoRun = TRUE;
            bEnableStretchDemo = FALSE;
            //nBackSubRunMode
            
          }
        }
        else
        {
          Timer_Counter_Clear(C_TIMER_500MS);
        }
       if(Timer_Counter(C_TIMER_INDICATE + T_LOOP,WAIT_INDICATE_TIME))
        {
          IndicateLED_Toggle();
        }
        if(Timer_Counter(C_TIMER_TEMP,60*10))
        {
          nChairRunState = CHAIR_STATE_SLEEP; 
        }
  
        
        
       Input_Proce();
       Valve_Send_Data();
       Main_Send();
       Main_BlueToothSend();
       //Main_MassageSignalSend();
       //靠背升降电机手动处理
        Main_BackPad_Proce();
        //小腿升降电机手动处理
        Main_LegPad_Proce();
        //小腿伸缩电机手动处理
        
  #ifdef RT8305T_1  
         Main_FlexPad_Proce();
   #endif     
         //振动(摇摆)处理
      //    Main_VibrateMotorControl() ;   
          
      //    if(Input_GetWaveMotorPosition() == REACH_WAVER_LIMIT)  
      //    {
      //      nWaveOverTime = 0;
     //      }      
      //    
        //===========================
        Main_Massage_Position_Proce();//wait 椅子复位时的所有电机状态,包含零重力电机的控制
        
  #ifdef RT8305T_1  
        FlexMotorFollowingFood();//自动找脚程序
  #endif  
        Main_Send_Leg();
        Problem_Proce();//检查3D ,伸缩电机，行走电机是否运行超时,考虑是否增加摇摆电机异常处理显示
         MusicSampling();
 //       VoiceUart_Proce();   //modify by taoqingsong
        if((bKeyBackPadUp == FALSE) && (bKeyBackPadDown == FALSE) &&
                (bKeyLegPadUp == FALSE) && (bKeyLegPadDown == FALSE) &&
                 (bKeyFlexOut == FALSE) && (bKeyFlexIn == FALSE))  
        {
            if((nBuzzerMode == BUZZER_MODE_FAST) ||
               (nBuzzerMode == BUZZER_MODE_SLOW))
            {
              nBuzzerMode = BUZZER_MODE_OFF ;
              bSendBuzzerMode = TRUE ;
            }
        }
        
    } //end while
   /***************程序退出区**************************/
}
//开始自动程序
void Main_Start_Auto(void)
{
  /*if(nTargetMassagePosition != MASSAGE_OPTIMAL2_POSITION)
  {
    nTargetMassagePosition = MASSAGE_OPTIMAL2_POSITION;//MASSAGE_OPTIMAL_POSITION
    bMassagePositionUpdate = TRUE;
  }*/
  
if(nTargetMassagePosition != MASSAGE_OPTIMAL_POSITION)
  {
    nTargetMassagePosition = MASSAGE_OPTIMAL_POSITION;//MASSAGE_OPTIMAL_POSITION
    bMassagePositionUpdate = TRUE;
  }
  
  
  bRollerEnable = TRUE;nRoller3sCnt = 0;
  bBackAutoModeInit = TRUE ;//选中自动程序时 ，自动进行靠背电机初始化标志
  //设置气囊功能
  nKeyAirBagLocate = AIRBAG_LOCATE_AUTO ;
  if( Valve_GetAirBagStrength() == AIRBAG_STRENGTH_0)
  {
    Valve_SetAirBagStrength(AIRBAG_STRENGTH_1);
  }
  

  
/*  if( nBackSubRunMode == BACK_SUB_MODE_AUTO_5)
  {
    
     st_AirBagAuto.pAirBagArray = AirBagModeAutoRelax;
     st_AirBagAuto.nTotalSteps = sizeof(AirBagModeAutoRelax)/sizeof(struct AirBagStruct);  
  }
  else*/
  {
    st_AirBagAuto.pAirBagArray = AirBagModeAuto;// AirBagModeAuto[] =
    st_AirBagAuto.nTotalSteps = sizeof(AirBagModeAuto)/sizeof(struct AirBagStruct);   
  }

   st_AirBagLegFoot.init = TRUE ;
   st_AirBagArmNeck.init = TRUE ;
   st_AirBagAuto.init = TRUE;     
  
  st_AirBagSeat.init = TRUE;
  
  //st_AirBag_Neck.init = TRUE;//20170205 WGH
}

void Main_Start_3D(void)
{
 /*if(nTargetMassagePosition != MASSAGE_OPTIMAL2_POSITION)
  {
    nTargetMassagePosition = MASSAGE_OPTIMAL2_POSITION;
    bMassagePositionUpdate = TRUE;
  }*/
  
 if(nTargetMassagePosition != MASSAGE_OPTIMAL_POSITION)
  {
    nTargetMassagePosition = MASSAGE_OPTIMAL_POSITION;//MASSAGE_OPTIMAL_POSITION
    bMassagePositionUpdate = TRUE;
  }

 
  bRollerEnable = TRUE;nRoller3sCnt = 0;
  bBackAutoModeInit = TRUE ;
  //设置气囊功能
  nKeyAirBagLocate = AIRBAG_LOCATE_AUTO ;
  if( Valve_GetAirBagStrength() == AIRBAG_STRENGTH_0)
  {
    Valve_SetAirBagStrength(AIRBAG_STRENGTH_1);
  }


 //  st_AirBagAuto.pAirBagArray = AirBagModeAuto;
  // st_AirBagAuto.nTotalSteps = sizeof(AirBagModeAuto)/sizeof(struct AirBagStruct);
  if( nBackSubRunMode == BACK_SUB_MODE_AUTO_5)
  {
    
     st_AirBagAuto.pAirBagArray =AirBagModeAutoRelax;
     st_AirBagAuto.nTotalSteps = sizeof(AirBagModeAutoRelax)/sizeof(struct AirBagStruct);  
  }
  else
  {
    st_AirBagAuto.pAirBagArray = AirBagModeAuto;
    st_AirBagAuto.nTotalSteps = sizeof(AirBagModeAuto)/sizeof(struct AirBagStruct);   
  } 
   
   
   
   
   st_AirBagLegFoot.init = TRUE ;
   st_AirBagArmNeck.init = TRUE ;

   st_AirBagAuto.init = TRUE;       

   st_AirBagSeat.init = TRUE;            
  //st_AirBag_Neck.init = TRUE;//20170205 WGH
  
  
  
  switch(nBackSubRunMode)
  {
   case  BACK_SUB_MODE_3D1: Data_Set_Start(1, RUN_TIME_10); break;
   case  BACK_SUB_MODE_3D2: Data_Set_Start(1, RUN_TIME_5); break;
   case  BACK_SUB_MODE_3D3: Data_Set_Start(1, RUN_TIME_5); break;  
  }
  nCurSubFunction = BACK_SUB_MODE_NO_ACTION;  //20150303增加，防止显示错乱
  
  
}

//开始手动程序
void Main_Start_Manual(void)
{
  if(bEnableStretchDemoRun == true)
  {
    bEnableStretchDemoRun = false;
  }
    switch(nMaunalSubMode)	
  {
  case nMaunalSubMode_KNEAD: 
     if(nBackSubRunMode == BACK_SUB_MODE_KNEAD) 
    {
      //设置背部功能
      //BackManualModeNoAction() ;
    bKneadMotorInProcess = TRUE ;
     if(nKneadTurn <= 1)
    {
     // nKneadMotorControlParam1 = KNEAD_RUN ;
      ManualDirector[0].nKneadMotorState = KNEAD_RUN ;
      ManualDirector[1].nKneadMotorState = KNEAD_RUN ;
      ManualDirector[2].nKneadMotorState = KNEAD_RUN ;
      ManualDirector[3].nKneadMotorState = KNEAD_RUN ;
    }
    else
    {
      //nKneadMotorControlParam1 = KNEAD_ANTIRUN ;
      ManualDirector[0].nKneadMotorState = KNEAD_ANTIRUN ;
      ManualDirector[1].nKneadMotorState = KNEAD_ANTIRUN ;
      ManualDirector[2].nKneadMotorState = KNEAD_ANTIRUN ;
      ManualDirector[3].nKneadMotorState = KNEAD_ANTIRUN ;
    }   
    
    nKneadMotorControlParam1 = ManualDirector[nCurActionStep].nKneadMotorState ;
    nKneadMotorControlParam2 = ManualDirector[nCurActionStep].nKneadMotorCycles ;
      break ;
    }
    nBackSubRunMode = BACK_SUB_MODE_KNEAD ;
    if(nKeyKneadKnockSpeed == SPEED_0)
    {
      nKeyKneadKnockSpeed = SPEED_1 ;
    }
    if(nKeyKneadWidth == KNEAD_WIDTH_UNKNOWN)
    {
      nKeyKneadWidth = KNEAD_WIDTH_MED ;
    }
    ManualDirector[0].nSubFunction = BACK_SUB_MODE_KNEAD ;
    //ManualDirector[0].nKneadMotorState = KNEAD_RUN ;
    ManualDirector[0].nKneadMotorCycles = 0 ;
    ManualDirector[0].nKnockMotorState = KNOCK_STOP ;
    ManualDirector[0].nKnockMotorRunTime = 0 ;
    ManualDirector[0].nKneadKnockSpeed = nKeyKneadKnockSpeed ;
    ManualDirector[0].n3D_MotorState = _3D_MANUAL;
    
    ManualDirector[1].nSubFunction = BACK_SUB_MODE_KNEAD ;
    //ManualDirector[1].nKneadMotorState = KNEAD_ANTIRUN ;
    ManualDirector[1].nKneadMotorCycles = 0 ;
    ManualDirector[1].nKnockMotorState = KNOCK_STOP ;
    ManualDirector[1].nKnockMotorRunTime = 0 ;
    ManualDirector[1].nKneadKnockSpeed = nKeyKneadKnockSpeed ;    
    ManualDirector[1].n3D_MotorState = _3D_MANUAL;
    
    
    ManualDirector[2].nSubFunction = BACK_SUB_MODE_KNEAD ;
    //ManualDirector[2].nKneadMotorState = KNEAD_RUN ;
    ManualDirector[2].nKneadMotorCycles = 0 ;
    ManualDirector[2].nKnockMotorState = KNOCK_STOP ;
    ManualDirector[2].nKnockMotorRunTime = 0 ;
    ManualDirector[2].nKneadKnockSpeed = nKeyKneadKnockSpeed ;
    ManualDirector[2].n3D_MotorState = _3D_MANUAL;
    
    ManualDirector[3].nSubFunction = BACK_SUB_MODE_KNEAD ;
    //ManualDirector[3].nKneadMotorState = KNEAD_ANTIRUN ;
    ManualDirector[3].nKneadMotorCycles = 0 ;
    ManualDirector[3].nKnockMotorState = KNOCK_STOP ;
    ManualDirector[3].nKnockMotorRunTime = 0 ;
    ManualDirector[3].nKneadKnockSpeed = nKeyKneadKnockSpeed ;
    ManualDirector[3].n3D_MotorState = _3D_MANUAL;
    //设置揉捏电机
    bKneadMotorInProcess = TRUE ;
     if(nKneadTurn <= 1)
    {
     // nKneadMotorControlParam1 = KNEAD_RUN ;
      ManualDirector[0].nKneadMotorState = KNEAD_RUN ;
      ManualDirector[1].nKneadMotorState = KNEAD_RUN ;
      ManualDirector[2].nKneadMotorState = KNEAD_RUN ;
      ManualDirector[3].nKneadMotorState = KNEAD_RUN ;
    }
    else
    {
      //nKneadMotorControlParam1 = KNEAD_ANTIRUN ;
      ManualDirector[0].nKneadMotorState = KNEAD_ANTIRUN ;
      ManualDirector[1].nKneadMotorState = KNEAD_ANTIRUN ;
      ManualDirector[2].nKneadMotorState = KNEAD_ANTIRUN ;
      ManualDirector[3].nKneadMotorState = KNEAD_ANTIRUN ;
    }   
    
    nKneadMotorControlParam1 = ManualDirector[nCurActionStep].nKneadMotorState ;
    nKneadMotorControlParam2 = ManualDirector[nCurActionStep].nKneadMotorCycles ;

    
    
    
    
    //设置捶击电机
    bKnockMotorInProcess = TRUE ;
    nKnockMotorControlParam1 = ManualDirector[nCurActionStep].nKnockMotorState ;
    nKnockMotorControlParam2 = ManualDirector[nCurActionStep].nKnockMotorRunTime ;
    nKnockMotorControlParam3 = ManualDirector[nCurActionStep].nKnockMotorStopTime ;
    nMaxActionStep = 2 ;
    nStartActionStep = 0 ;
    bBackManualModeInit = TRUE ;
    break;
    
  case nMaunalSubMode_KNOCK:
    if(nBackSubRunMode == BACK_SUB_MODE_KNOCK) 
    {
      //设置背部功能
      BackManualModeNoAction() ;
      break ;
    }
    nBackSubRunMode = BACK_SUB_MODE_KNOCK ;
    //根据当前的状态设置nKeyKneadKnockSpeed,nKeyBackLocate
    if(nKeyKneadKnockSpeed == SPEED_0)
    {
      nKeyKneadKnockSpeed = SPEED_1 ;
    }
    
    if(nKeyKneadWidth == KNEAD_WIDTH_UNKNOWN)
    {
      nKeyKneadWidth = KNEAD_WIDTH_MED ;
    }
    switch(nKeyKneadWidth)
    {
    case KNEAD_WIDTH_MIN:
      ManualDirector[0].nKneadMotorState = KNEAD_STOP_AT_MIN ;
      ManualDirector[1].nKneadMotorState = KNEAD_STOP_AT_MIN ;
      ManualDirector[2].nKneadMotorState = KNEAD_STOP_AT_MIN ;
      ManualDirector[3].nKneadMotorState = KNEAD_STOP_AT_MIN ;
      break ;
    case KNEAD_WIDTH_MED:
      ManualDirector[0].nKneadMotorState = KNEAD_STOP_AT_MED ;
      ManualDirector[1].nKneadMotorState = KNEAD_STOP_AT_MED ;
      ManualDirector[2].nKneadMotorState = KNEAD_STOP_AT_MED ;
      ManualDirector[3].nKneadMotorState = KNEAD_STOP_AT_MED ;
      break ;
    case KNEAD_WIDTH_MAX:
      ManualDirector[0].nKneadMotorState = KNEAD_STOP_AT_MAX ;
      ManualDirector[1].nKneadMotorState = KNEAD_STOP_AT_MAX ;
      ManualDirector[2].nKneadMotorState = KNEAD_STOP_AT_MAX ;
      ManualDirector[3].nKneadMotorState = KNEAD_STOP_AT_MAX ;
      break ;
    }
    ManualDirector[0].nSubFunction = BACK_SUB_MODE_KNOCK ;
    ManualDirector[0].nKneadMotorCycles = 0 ;
    ManualDirector[0].nKnockMotorState = KNOCK_RUN_WIDTH ;
    ManualDirector[0].nKnockMotorRunTime = 0 ;
    ManualDirector[0].nKneadKnockSpeed = nKeyKneadKnockSpeed ;
    ManualDirector[0].n3D_MotorState = _3D_MANUAL;
    
    ManualDirector[1].nSubFunction = BACK_SUB_MODE_KNOCK ;
    ManualDirector[1].nKneadMotorCycles = 0 ;
    ManualDirector[1].nKnockMotorState = KNOCK_RUN_WIDTH ;
    ManualDirector[1].nKnockMotorRunTime = 0 ;
    ManualDirector[1].nKneadKnockSpeed = nKeyKneadKnockSpeed ;
    ManualDirector[1].n3D_MotorState = _3D_MANUAL;
    
    ManualDirector[2].nSubFunction = BACK_SUB_MODE_KNOCK ;
    ManualDirector[2].nKneadMotorCycles = 0 ;
    ManualDirector[2].nKnockMotorState = KNOCK_RUN_WIDTH ;
    ManualDirector[2].nKnockMotorRunTime = 0 ;
    ManualDirector[2].nKneadKnockSpeed = nKeyKneadKnockSpeed ;
    ManualDirector[2].n3D_MotorState = _3D_MANUAL;
    
    ManualDirector[3].nSubFunction = BACK_SUB_MODE_KNOCK ;
    ManualDirector[3].nKneadMotorCycles = 0 ;
    ManualDirector[3].nKnockMotorState = KNOCK_RUN_WIDTH ;
    ManualDirector[3].nKnockMotorRunTime = 0 ;
    ManualDirector[3].nKneadKnockSpeed = nKeyKneadKnockSpeed ;
    ManualDirector[3].n3D_MotorState = _3D_MANUAL;
    //设置揉捏电机
    bKneadMotorInProcess = TRUE ;
    nKneadMotorControlParam1 = ManualDirector[nCurActionStep].nKneadMotorState ;
    nKneadMotorControlParam2 = ManualDirector[nCurActionStep].nKneadMotorCycles ;
    //设置捶击电机
    bKnockMotorInProcess = TRUE ;
    nKnockMotorControlParam1 = ManualDirector[nCurActionStep].nKnockMotorState ;
    nKnockMotorControlParam2 = ManualDirector[nCurActionStep].nKnockMotorRunTime ;
    nKnockMotorControlParam3 = ManualDirector[nCurActionStep].nKnockMotorStopTime ;
    
    nMaxActionStep = 2 ;
    nStartActionStep = 0 ;
    bBackManualModeInit = TRUE ;
    break;
    
  case nMaunalSubMode_WAVELET:
    if(nBackSubRunMode == BACK_SUB_MODE_WAVELET) 
    {
      //设置背部功能
      BackManualModeNoAction() ;
      break ;
    }
    nBackSubRunMode = BACK_SUB_MODE_WAVELET ;
    //根据当前的状态设置nKeyKneadKnockSpeed,nKeyBackLocate和nKeyKneadWidth
    if(nKeyKneadKnockSpeed == SPEED_0)
    {
      nKeyKneadKnockSpeed = SPEED_1 ;
    }
    
    if(nKeyKneadWidth == KNEAD_WIDTH_UNKNOWN)
    {
      nKeyKneadWidth = KNEAD_WIDTH_MED ;
    }
    ManualDirector[0].nSubFunction = BACK_SUB_MODE_WAVELET ;
    ManualDirector[0].nKneadMotorState = KNEAD_RUN ;
    ManualDirector[0].nKneadMotorCycles = 0 ;
    ManualDirector[0].nKnockMotorState = KNOCK_RUN ;
    ManualDirector[0].nKnockMotorRunTime = 0 ;
    ManualDirector[0].nKneadKnockSpeed = nKeyKneadKnockSpeed ;
    ManualDirector[0].n3D_MotorState = _3D_MANUAL;
    ManualDirector[1].nSubFunction = BACK_SUB_MODE_WAVELET ;
    ManualDirector[1].nKneadMotorState = KNEAD_RUN ;
    ManualDirector[1].nKneadMotorCycles = 0 ;
    ManualDirector[1].nKnockMotorState = KNOCK_RUN ;
    ManualDirector[1].nKnockMotorRunTime = 0 ;
    ManualDirector[1].nKneadKnockSpeed = nKeyKneadKnockSpeed ;
    ManualDirector[1].n3D_MotorState = _3D_MANUAL;
    ManualDirector[2].nSubFunction = BACK_SUB_MODE_WAVELET ;
    ManualDirector[2].nKneadMotorState = KNEAD_RUN ;
    ManualDirector[2].nKneadMotorCycles = 0 ;
    ManualDirector[2].nKnockMotorState = KNOCK_RUN ;
    ManualDirector[2].nKnockMotorRunTime = 0 ;
    ManualDirector[2].nKneadKnockSpeed = nKeyKneadKnockSpeed ;
    ManualDirector[2].n3D_MotorState = _3D_MANUAL;
    ManualDirector[3].nSubFunction = BACK_SUB_MODE_WAVELET ;
    ManualDirector[3].nKneadMotorState = KNEAD_RUN ;
    ManualDirector[3].nKneadMotorCycles = 0 ;
    ManualDirector[3].nKnockMotorState = KNOCK_RUN ;
    ManualDirector[3].nKnockMotorRunTime = 0 ;
    ManualDirector[3].nKneadKnockSpeed = nKeyKneadKnockSpeed ;
    ManualDirector[3].n3D_MotorState = _3D_MANUAL;
    //设置揉捏电机
    bKneadMotorInProcess = TRUE ;
    nKneadMotorControlParam1 = ManualDirector[nCurActionStep].nKneadMotorState ;
    nKneadMotorControlParam2 = ManualDirector[nCurActionStep].nKneadMotorCycles ;
    //设置捶击电机
    bKnockMotorInProcess = TRUE ;
    nKnockMotorControlParam1 = ManualDirector[nCurActionStep].nKnockMotorState ;
    nKnockMotorControlParam2 = ManualDirector[nCurActionStep].nKnockMotorRunTime ;
    nKnockMotorControlParam3 = ManualDirector[nCurActionStep].nKnockMotorStopTime ;
    
    nMaxActionStep = 2 ;
    nStartActionStep = 0 ;
    bBackManualModeInit = TRUE ;
    break;
    
  case nMaunalSubMode_SOFT_KNOCK:
    if(nBackSubRunMode == BACK_SUB_MODE_SOFT_KNOCK) 
    {
      //设置背部功能
      BackManualModeNoAction() ;
      break ;
    }
    nBackSubRunMode = BACK_SUB_MODE_SOFT_KNOCK ;
    //根据当前的状态设置nKeyKneadKnockSpeed,nKeyBackLocate
    if(nKeyKneadKnockSpeed == SPEED_0)
    {
      nKeyKneadKnockSpeed = SPEED_1 ;
    }
    if(nKeyKneadWidth == KNEAD_WIDTH_UNKNOWN)
    {
      nKeyKneadWidth = KNEAD_WIDTH_MED ;
    }
    switch(nKeyKneadWidth)
    {
    case KNEAD_WIDTH_MIN:
      ManualDirector[0].nKneadMotorState = KNEAD_STOP_AT_MIN ;
      ManualDirector[1].nKneadMotorState = KNEAD_STOP_AT_MIN ;
      ManualDirector[2].nKneadMotorState = KNEAD_STOP_AT_MIN ;
      ManualDirector[3].nKneadMotorState = KNEAD_STOP_AT_MIN ;
      break ;
    case KNEAD_WIDTH_MED:
      ManualDirector[0].nKneadMotorState = KNEAD_STOP_AT_MED ;
      ManualDirector[1].nKneadMotorState = KNEAD_STOP_AT_MED ;
      ManualDirector[2].nKneadMotorState = KNEAD_STOP_AT_MED ;
      ManualDirector[3].nKneadMotorState = KNEAD_STOP_AT_MED ;
      break ;
    case KNEAD_WIDTH_MAX:
      ManualDirector[0].nKneadMotorState = KNEAD_STOP_AT_MAX ;
      ManualDirector[1].nKneadMotorState = KNEAD_STOP_AT_MAX ;
      ManualDirector[2].nKneadMotorState = KNEAD_STOP_AT_MAX ;
      ManualDirector[3].nKneadMotorState = KNEAD_STOP_AT_MAX ;
      break ;
    }
    ManualDirector[0].nSubFunction = BACK_SUB_MODE_SOFT_KNOCK ;
    ManualDirector[0].nKneadMotorCycles = 0 ;
    ManualDirector[0].nKnockMotorState = KNOCK_RUN_STOP ;
    ManualDirector[0].nKnockMotorRunTime = 1 ;
    ManualDirector[0].nKnockMotorStopTime = 4 ;
    ManualDirector[0].nKneadKnockSpeed = nKeyKneadKnockSpeed ;
    ManualDirector[0].n3D_MotorState = _3D_MANUAL;
    ManualDirector[1].nSubFunction = BACK_SUB_MODE_SOFT_KNOCK ;
    ManualDirector[1].nKneadMotorCycles = 0 ;
    ManualDirector[1].nKnockMotorState = KNOCK_RUN_STOP ;
    ManualDirector[1].nKnockMotorRunTime = 1 ;
    ManualDirector[1].nKnockMotorStopTime = 4 ;
    ManualDirector[1].nKneadKnockSpeed = nKeyKneadKnockSpeed ;
    ManualDirector[1].n3D_MotorState = _3D_MANUAL;
    //设置揉捏电机(立即更新动作)
    bKneadMotorInProcess = TRUE ;
    nKneadMotorControlParam1 = ManualDirector[nCurActionStep].nKneadMotorState ;
    nKneadMotorControlParam2 = ManualDirector[nCurActionStep].nKneadMotorCycles ;
    //设置捶击电机(立即更新动作)
    bKnockMotorInProcess = TRUE ;
    nKnockMotorControlParam1 = ManualDirector[nCurActionStep].nKnockMotorState ;
    nKnockMotorControlParam2 = ManualDirector[nCurActionStep].nKnockMotorRunTime ;
    nKnockMotorControlParam3 = ManualDirector[nCurActionStep].nKnockMotorStopTime ;
    
    nMaxActionStep = 2 ;
    nStartActionStep = 0 ;
    bBackManualModeInit = TRUE ;
    break;
    
  case nMaunalSubMode_PRESS:
#ifdef _3D_MANUAL_TEST                                    
   {
     if(nBackSubRunMode == BACK_SUB_MODE_PRESS) 
    {
      //设置背部功能
      BackManualModeNoAction() ;
      break ;
    }
    nBackSubRunMode = BACK_SUB_MODE_PRESS ;
    //根据当前的状态设置nKeyKneadKnockSpeed,nKeyBackLocate
    if(nKeyKneadKnockSpeed == SPEED_0)
    {
      nKeyKneadKnockSpeed = SPEED_1 ;
    }
   
    if(nKeyKneadWidth == KNEAD_WIDTH_UNKNOWN)
    {
      nKeyKneadWidth = KNEAD_WIDTH_MED ;
    }
    switch(nKeyKneadWidth)
    {
    case KNEAD_WIDTH_MIN:
      ManualDirector[0].nKneadMotorState = KNEAD_STOP_AT_MIN ;
      ManualDirector[1].nKneadMotorState = KNEAD_STOP_AT_MIN ;
      ManualDirector[2].nKneadMotorState = KNEAD_STOP_AT_MIN ;
      ManualDirector[3].nKneadMotorState = KNEAD_STOP_AT_MIN ;
      break ;
    case KNEAD_WIDTH_MED:
      ManualDirector[0].nKneadMotorState = KNEAD_STOP_AT_MED ;
      ManualDirector[1].nKneadMotorState = KNEAD_STOP_AT_MED ;
      ManualDirector[2].nKneadMotorState = KNEAD_STOP_AT_MED ;
      ManualDirector[3].nKneadMotorState = KNEAD_STOP_AT_MED ;
      break ;
    case KNEAD_WIDTH_MAX:
      ManualDirector[0].nKneadMotorState = KNEAD_STOP_AT_MAX ;
      ManualDirector[1].nKneadMotorState = KNEAD_STOP_AT_MAX ;
      ManualDirector[2].nKneadMotorState = KNEAD_STOP_AT_MAX ;
      ManualDirector[3].nKneadMotorState = KNEAD_STOP_AT_MAX ;
      break ;
    }
    ManualDirector[0].nSubFunction = BACK_SUB_MODE_PRESS ;
    ManualDirector[0].nKneadMotorCycles = 0 ;
    ManualDirector[0].nKnockMotorState = KNOCK_STOP ;
    ManualDirector[0].nKnockMotorRunTime = 0 ;
    ManualDirector[0].nKneadKnockSpeed = nKeyKneadKnockSpeed ;
    ManualDirector[0].n3D_MotorControlState = _3D_MANUAL;
    ManualDirector[1].nSubFunction = BACK_SUB_MODE_PRESS ;
    ManualDirector[1].nKneadMotorCycles = 0 ;
    ManualDirector[1].nKnockMotorState = KNOCK_STOP ;
    ManualDirector[1].nKnockMotorRunTime = 0 ;
    ManualDirector[1].nKneadKnockSpeed = nKeyKneadKnockSpeed ;
    ManualDirector[1].n3D_MotorControlState = _3D_MANUAL;
    //设置揉捏电机
    bKneadMotorInProcess = TRUE ;
    nKneadMotorControlParam1 = ManualDirector[nCurActionStep].nKneadMotorState ;
    nKneadMotorControlParam2 = ManualDirector[nCurActionStep].nKneadMotorCycles ;
    //设置捶击电机
    bKnockMotorInProcess = TRUE ;
    nKnockMotorControlParam1 = ManualDirector[nCurActionStep].nKnockMotorState ;
    nKnockMotorControlParam2 = ManualDirector[nCurActionStep].nKnockMotorRunTime ;
    nKnockMotorControlParam3 = ManualDirector[nCurActionStep].nKnockMotorStopTime ;
    
    nMaxActionStep = 2 ;
    nStartActionStep = 0 ;
    bBackManualModeInit = TRUE ;
   
  }
#else /*_3D_MANUAL_TEST    */
    if(nBackSubRunMode == BACK_SUB_MODE_PRESS) 
    {
      //设置背部功能
      BackManualModeNoAction() ;
      break ;
    }
    nBackSubRunMode = BACK_SUB_MODE_PRESS ;
    //根据当前的状态设置nKeyKneadKnockSpeed,nKeyBackLocate
    if(nKeyKneadKnockSpeed == SPEED_0)
    {
      nKeyKneadKnockSpeed = SPEED_1 ;
    }
    
    if(nKeyKneadWidth == KNEAD_WIDTH_UNKNOWN)
    {
      nKeyKneadWidth = KNEAD_WIDTH_MED ;
    }
    switch(nKeyKneadWidth)
    {
    case KNEAD_WIDTH_MIN:
      ManualDirector[0].nKneadMotorState = KNEAD_STOP_AT_MIN ;
      ManualDirector[1].nKneadMotorState = KNEAD_STOP_AT_MIN ;
      ManualDirector[2].nKneadMotorState = KNEAD_STOP_AT_MIN ;
      ManualDirector[3].nKneadMotorState = KNEAD_STOP_AT_MIN ;
      break ;
    case KNEAD_WIDTH_MED:
      ManualDirector[0].nKneadMotorState = KNEAD_STOP_AT_MED ;
      ManualDirector[1].nKneadMotorState = KNEAD_STOP_AT_MED ;
      ManualDirector[2].nKneadMotorState = KNEAD_STOP_AT_MED ;
      ManualDirector[3].nKneadMotorState = KNEAD_STOP_AT_MED ;
      break ;
    case KNEAD_WIDTH_MAX:
      ManualDirector[0].nKneadMotorState = KNEAD_STOP_AT_MAX ;
      ManualDirector[1].nKneadMotorState = KNEAD_STOP_AT_MAX ;
      ManualDirector[2].nKneadMotorState = KNEAD_STOP_AT_MAX ;
      ManualDirector[3].nKneadMotorState = KNEAD_STOP_AT_MAX ;
      break ;
    }
    ManualDirector[0].nSubFunction = BACK_SUB_MODE_PRESS ;
    ManualDirector[0].nKneadMotorCycles = 0 ;
    ManualDirector[0].nKnockMotorState = KNOCK_STOP ;
    ManualDirector[0].nKnockMotorRunTime = 0 ;
    ManualDirector[0].nKneadKnockSpeed = nKeyKneadKnockSpeed ;
    ManualDirector[0].n3D_MotorState = _3D_MANUAL;
    ManualDirector[1].nSubFunction = BACK_SUB_MODE_PRESS ;
    ManualDirector[1].nKneadMotorCycles = 0 ;
    ManualDirector[1].nKnockMotorState = KNOCK_STOP ;
    ManualDirector[1].nKnockMotorRunTime = 0 ;
    ManualDirector[1].nKneadKnockSpeed = nKeyKneadKnockSpeed ;
    ManualDirector[1].n3D_MotorState = _3D_MANUAL;
    //设置揉捏电机
    bKneadMotorInProcess = TRUE ;
    nKneadMotorControlParam1 = ManualDirector[nCurActionStep].nKneadMotorState ;
    nKneadMotorControlParam2 = ManualDirector[nCurActionStep].nKneadMotorCycles ;
    //设置捶击电机
    bKnockMotorInProcess = TRUE ;
    nKnockMotorControlParam1 = ManualDirector[nCurActionStep].nKnockMotorState ;
    nKnockMotorControlParam2 = ManualDirector[nCurActionStep].nKnockMotorRunTime ;
    nKnockMotorControlParam3 = ManualDirector[nCurActionStep].nKnockMotorStopTime ;
    
    nMaxActionStep = 2 ;
    nStartActionStep = 0 ;
    bBackManualModeInit = TRUE ;
    
#endif /*_3D_MANUAL_TEST   */                                    
    break;
    
  case nMaunalSubMode_MUSIC:
    if(nBackSubRunMode == BACK_SUB_MODE_MUSIC) 
    {
      //设置背部功能
      BackManualModeNoAction() ;
      break ;
    }
    nBackSubRunMode = BACK_SUB_MODE_MUSIC ;
    //根据当前的状态设置nKeyKneadKnockSpeed,nKeyBackLocate
    if(nKeyKneadKnockSpeed == SPEED_0)
    {
      nKeyKneadKnockSpeed = SPEED_1 ;
    }
    // if((nKeyBackLocate == LOCATE_NONE) || (nKeyBackLocate == LOCATE_POINT))
    {
      nKeyBackLocate = LOCATE_FULL_BACK ;
      
      ManualDirector[0].nWalkMotorLocateMethod = WALK_LOCATE_ABSULATE ;
      ManualDirector[0].nWalkMotorLocateParam = 0 ;
      ManualDirector[1].nWalkMotorLocateMethod = WALK_LOCATE_TOP ;
      ManualDirector[1].nWalkMotorLocateParam = 0 ;
      ManualDirector[2].nWalkMotorLocateMethod = WALK_LOCATE_ABSULATE ;
      ManualDirector[2].nWalkMotorLocateParam = 0 ;
      ManualDirector[3].nWalkMotorLocateMethod = WALK_LOCATE_TOP ;
      ManualDirector[3].nWalkMotorLocateParam = 0 ;
    }
    if(nKeyKneadWidth == KNEAD_WIDTH_UNKNOWN)
    {
      nKeyKneadWidth = KNEAD_WIDTH_MED ;
    }
    ManualDirector[0].nSubFunction = BACK_SUB_MODE_MUSIC ;
    ManualDirector[0].nKneadMotorState = KNEAD_RUN ;
    ManualDirector[0].nKneadMotorCycles = 0 ;
    ManualDirector[0].nKnockMotorState = KNOCK_RUN_MUSIC ;
    ManualDirector[0].nKnockMotorRunTime = 0 ;
    ManualDirector[0].nKnockMotorStopTime = 0 ;
    ManualDirector[0].nKneadKnockSpeed = nKeyKneadKnockSpeed ;
    ManualDirector[0].n3D_MotorState = _3D_MANUAL;
    ManualDirector[1].nSubFunction = BACK_SUB_MODE_MUSIC ;
    ManualDirector[1].nKneadMotorState = KNEAD_STOP_AT_MIN ;
    ManualDirector[1].nKneadMotorCycles = 0 ;
    ManualDirector[1].nKnockMotorState = KNOCK_RUN_MUSIC ;
    ManualDirector[1].nKnockMotorRunTime = 0 ;
    ManualDirector[1].nKnockMotorStopTime = 0 ;
    ManualDirector[1].nKneadKnockSpeed = nKeyKneadKnockSpeed ;
    ManualDirector[1].n3D_MotorState = _3D_MANUAL;
    ManualDirector[2].nSubFunction = BACK_SUB_MODE_MUSIC ;
    ManualDirector[2].nKneadMotorState = KNEAD_RUN ;
    ManualDirector[2].nKneadMotorCycles = 0 ;
    ManualDirector[2].nKnockMotorState = KNOCK_RUN_MUSIC ;
    ManualDirector[2].nKnockMotorRunTime = 0 ;
    ManualDirector[2].nKnockMotorStopTime = 0 ;
    ManualDirector[2].nKneadKnockSpeed = nKeyKneadKnockSpeed ;
    ManualDirector[2].n3D_MotorState = _3D_MANUAL;
    ManualDirector[3].nSubFunction = BACK_SUB_MODE_MUSIC ;
    ManualDirector[3].nKneadMotorState = KNEAD_STOP_AT_MAX ;
    ManualDirector[3].nKneadMotorCycles = 0 ;
    ManualDirector[3].nKnockMotorState = KNOCK_RUN_MUSIC ;
    ManualDirector[3].nKnockMotorRunTime = 0 ;
    ManualDirector[3].nKnockMotorStopTime = 0 ;
    ManualDirector[3].nKneadKnockSpeed = nKeyKneadKnockSpeed ;
    ManualDirector[3].n3D_MotorState = _3D_MANUAL;
    //设置揉捏电机(立即更新动作)
    bKneadMotorInProcess = TRUE ;
    nKneadMotorControlParam1 = ManualDirector[nCurActionStep].nKneadMotorState ;
    nKneadMotorControlParam2 = ManualDirector[nCurActionStep].nKneadMotorCycles ;
    //设置捶击电机(立即更新动作)
    bKnockMotorInProcess = TRUE ;
    nKnockMotorControlParam1 = ManualDirector[nCurActionStep].nKnockMotorState ;
    nKnockMotorControlParam2 = ManualDirector[nCurActionStep].nKnockMotorRunTime ;
    nKnockMotorControlParam3 = ManualDirector[nCurActionStep].nKnockMotorStopTime ;
    
    nMaxActionStep = 4 ;
    nStartActionStep = 0 ;
    bBackManualModeInit = TRUE ;
    break;
    
  default:
    //设置背部功能
    BackManualModeNoAction() ;
    break ;
  }
  walkRefreshDown(nKeyBackLocate);
  
  
}
void Main_Work(void)// if(BodyDetectStep == DETECT_INITIAL)//(bShoulderOK == 0)
{
    BYTE key;
    BodyDetectStep = DETECT_INITIAL;
    bAutoProgramOver = false;
    Power_All_On();
    //VoiceUart_PowerOn();    //delet by taoqingsong
    Timer_Counter_Clear(C_TIMER_INDICATE);
    Timer_Counter_Clear(C_TIMER_TEMP);
    Timer_Counter_Clear(C_TIMER_500MS);
  //  w_PresetTime = ReadEEByte(USER_DATA_BASE + DEFAULT_TIME_ADDRESS)*60; //lgt
    Data_Set_Start(1, w_PresetTime);
    if(bEnableStretchDemoRun == TRUE)
    {
      //bEnableStretchDemoRun = FALSE;
      Data_Set_Start(1, 6*60);
      //bKeyWaistHeat = TRUE ;
    }
    if(nBackMainRunMode == BACK_MAIN_MODE_AUTO)
    {
     Main_Start_Auto();
    }   
    else if(nBackMainRunMode == BACK_MAIN_MODE_3D) //3D按摩
    {
     Main_Start_3D();
    }
    else if(nBackMainRunMode == BACK_MAIN_MODE_MANUAL) //手动
    {
       bMassagePositionUpdate = false;bZeroflash = FALSE;
       Main_Start_Manual(); 
    }
    //主循环
    while(CHAIR_STATE_RUN == nChairRunState)
    {
      //按键处理区
        key = Main_GetKey();
        key &= 0x7f;
        
        switch(key)
        {
        case H10_KEY_MENU:
          break;
        case H10_KEY_TAPPING_ONOFF :
          
          if(bTapping == 0)
          {
            bTapping = 1;
          }
          else
          {
            bTapping = 0;
          }
          
          
          break;      
          //
//
//#define H10_KEY_STRETCH_VIGOR1                   0x06
//#define H10_KEY_STRETCH_VIGOR2                   0x07
//#define H10_KEY_STRETCH_VIGOR3                   0x08  
          case H10_KEY_STRETCH_VIGOR1:
              nStretchVigor =1;
              nBuzzerMode = BUZZER_MODE_ONETIME ;
              bSendBuzzerMode = TRUE ;
              break;
          case H10_KEY_STRETCH_VIGOR2:
              nStretchVigor=2;
              nBuzzerMode = BUZZER_MODE_ONETIME ;
              bSendBuzzerMode = TRUE ;
              break;          
          case H10_KEY_STRETCH_VIGOR3:
              
              nStretchVigor=3;
              nBuzzerMode = BUZZER_MODE_ONETIME ;
              bSendBuzzerMode = TRUE ;
              break;   
          
        case H10_KEY_SWAY_ONOFF:                             //0X76    

          if(nBackMainRunMode == BACK_MAIN_MODE_AUTO)break;
          
          if( bRockEnable == false)
          {
            RockFunctionEnable(true);
            nChairRunState = CHAIR_STATE_RUN ; 
            nRockMode = ROCK_MANL;
          }
          else
          {
            bRockEnable = false;
            nRockMode = ROCK_IDLE;
            bKeyBackPadUp = FALSE;
            bKeyBackPadDown = FALSE;
            
                                    /*if(1==1)
                                    {
                                          nKeyAirBagLocate = AIRBAG_LOCATE_NONE ;
                                          nRollerPWM = 0;
                                          bRollerEnable = FALSE;
                                          Valve_SetRollerPWM(nRollerPWM);
                                          Valve_SetAirBagStrength(AIRBAG_STRENGTH_0);
                                    }*/
                                    
            
            
            
            
            
          }
          //
          
          /*
          ShoulderSteps = BODY_DETECT_OVER;
          BodyDetectStep = DETECT_NO_START;
          //
          nKeyAirBagLocate = AIRBAG_LOCATE_NONE ;
          nRollerPWM = 0;
          bRollerEnable = FALSE;
          Valve_SetRollerPWM(nRollerPWM);       
          
          
          */
          //         
          //bRockEnable = FALSE ;//Settle      
          
          nBuzzerMode = BUZZER_MODE_ONETIME ;
          bSendBuzzerMode = TRUE ;
          break;         
//=================USB   MP3================================================
//A2 02 00 5C 
//A2 03 00 5B
//A2 04 00 5A
//A2 05 00 59
//A2 06 00 58
//A2 07 00 57
//A2 08 00 56
//A2 10 0A 44   //BC  1011,1100:  0100,0011 +1  = 0100,0100
/*
                case H10_KEY_USB_PLAY_COMMAND:	
			n_mp3_key = H10_KEY_USB_PLAY_COMMAND;
			n_mp3_key_old = n_mp3_key;
			//
			//if((bBlueToothStatus == 1)&&(n_usb_indicate == USB_SONG_ON))
			{
			n_usb_send_buf[0]= 0xA2;
			n_usb_send_buf[1]= 0x02;
			n_usb_send_buf[2]= 0x00;
			n_usb_send_buf[3]= 0x5C;
			n_usb_send_buf[4]= 0x00;
			//BlueTooth_DMAUart_Transmit_Packet(n_usb_send_buf,4);
                        BlueToothUart_Transmit_Packet(n_usb_send_buf,4);
			}
                    break ;

                case H10_KEY_USB_PAUSE_COMMAND:					
			n_mp3_key = H10_KEY_USB_PAUSE_COMMAND;
			n_mp3_key_old = n_mp3_key;
                    //					
			//if((bBlueToothStatus == 1)&&(n_usb_indicate == USB_SONG_ON))
			{					
			n_usb_send_buf[0]= 0xA2;
			n_usb_send_buf[1]= 0x03;
			n_usb_send_buf[2]= 0x00;
			n_usb_send_buf[3]= 0x5B;
			n_usb_send_buf[4]= 0x00;
			//BlueTooth_DMAUart_Transmit_Packet(n_usb_send_buf,4);
                        BlueToothUart_Transmit_Packet(n_usb_send_buf,4);
			}
                    break ;

                case H10_KEY_USB_STOP_COMMAND:			
			if(H10_KEY_USB_PAUSE_COMMAND == n_mp3_key_old)		
			{
				break;   // 如果上次是PAUSE 那么就不能按STOP
			}
			else	
			{
			n_mp3_key = H10_KEY_USB_STOP_COMMAND;
			n_mp3_key_old = n_mp3_key;
			}
			
                    //
			//if((bBlueToothStatus == 1)&&(n_usb_indicate == USB_SONG_ON))
			{					
			n_usb_send_buf[0]= 0xA2;
			n_usb_send_buf[1]= 0x04;
			n_usb_send_buf[2]= 0x00;
			n_usb_send_buf[3]= 0x5A;
			n_usb_send_buf[4]= 0x00;
			//BlueTooth_DMAUart_Transmit_Packet(n_usb_send_buf,4);
                        BlueToothUart_Transmit_Packet(n_usb_send_buf,4);
			}			
                    break ;

                case H10_KEY_USB_PRE_COMMAND:					
			n_mp3_key = H10_KEY_USB_PRE_COMMAND;
			n_mp3_key_old = n_mp3_key;
                    //					
			//if((bBlueToothStatus == 1)&&(n_usb_indicate == USB_SONG_ON))
			{					
			n_usb_send_buf[0]= 0xA2;
			n_usb_send_buf[1]= 0x05;
			n_usb_send_buf[2]= 0x00;
			n_usb_send_buf[3]= 0x59;
			n_usb_send_buf[4]= 0x00;
			//BlueTooth_DMAUart_Transmit_Packet(n_usb_send_buf,4);
                        BlueToothUart_Transmit_Packet(n_usb_send_buf,4);
			}			
                    break ;
					
                case H10_KEY_USB_NEX_COMMAND:					
			n_mp3_key = H10_KEY_USB_NEX_COMMAND;
			n_mp3_key_old = n_mp3_key;
                    //					
			//if((bBlueToothStatus == 1)&&(n_usb_indicate == USB_SONG_ON))
			{					
			n_usb_send_buf[0]= 0xA2;
			n_usb_send_buf[1]= 0x06;
			n_usb_send_buf[2]= 0x00;
			n_usb_send_buf[3]= 0x58;
			n_usb_send_buf[4]= 0x00;
			//BlueTooth_DMAUart_Transmit_Packet(n_usb_send_buf,4);
                        BlueToothUart_Transmit_Packet(n_usb_send_buf,4);
			}			
                    break ;

                case H10_KEY_USB_VOL_UP_COMMAND:					
			n_mp3_key = H10_KEY_USB_VOL_UP_COMMAND;
			n_mp3_key_old = n_mp3_key;
                    //					
			//if((bBlueToothStatus == 1)&&(n_usb_indicate == USB_SONG_ON))
			{					
			n_usb_send_buf[0]= 0xA2;
			n_usb_send_buf[1]= 0x07;
			n_usb_send_buf[2]= 0x00;
			n_usb_send_buf[3]= 0x57;
			n_usb_send_buf[4]= 0x00;
			//BlueTooth_DMAUart_Transmit_Packet(n_usb_send_buf,4);
                        BlueToothUart_Transmit_Packet(n_usb_send_buf,4);
			}			
                    break ;


                case H10_KEY_USB_VOL_DW_COMMAND:					
			n_mp3_key = H10_KEY_USB_VOL_DW_COMMAND;
			n_mp3_key_old = n_mp3_key;
                    //					
			//if((bBlueToothStatus == 1)&&(n_usb_indicate == USB_SONG_ON))
			{					
			n_usb_send_buf[0]= 0xA2;
			n_usb_send_buf[1]= 0x08;
			n_usb_send_buf[2]= 0x00;
			n_usb_send_buf[3]= 0x56;
			n_usb_send_buf[4]= 0x00;
			//BlueTooth_DMAUart_Transmit_Packet(n_usb_send_buf,4);
                        BlueToothUart_Transmit_Packet(n_usb_send_buf,4);
			}
                    break ;  
  */            
//---------------USB MP3----------------------------------------------------------           
              
          
        case H10_KEY_BLUETOOTH_POWER_SWITCH:    
            if(bBlueToothStatus)
              {
                bBlueToothStatus = 0;//nvcBluetoothPower = 0;
              }
              else
              {
                bBlueToothStatus = 1;//nvcBluetoothPower = 1;
              }
              nBuzzerMode = BUZZER_MODE_ONETIME ;
              bSendBuzzerMode = TRUE ;
          break;
        case H10_KEY_POWER_SWITCH: //椅子运行时按下电源键
          {
            //按摩机构回位
            nChairRunState = CHAIR_STATE_SETTLE ;
            nSettleMode = POWER_KEY_RESET;            
            
            nTargetMassagePosition = MASSAGE_RESET_POSITION;
            bMassagePositionUpdate = TRUE;
            w_ZeroPosition = 0; //零重力电机归位        

            
            
          }
          RockFunctionEnable(false);
          st_Stretch.active = FALSE;SetStretchingEnable(0);
          nBuzzerMode = BUZZER_MODE_ONETIME ;
          bSendBuzzerMode = TRUE ;
          break ;
        case H10_KEY_CHAIR_AUTO_0:
         
          
          if(nBackSubRunMode == BACK_SUB_MODE_AUTO_0)  break;
          nBackMainRunMode = BACK_MAIN_MODE_AUTO ;
          nBackSubRunMode = BACK_SUB_MODE_AUTO_0 ;
          Main_Start_Auto(); 
          if(BodyDetectStep != DETECT_FINISH)//进行肩位 检测
          {
            BodyDetectStep = DETECT_INITIAL;
          }
          Data_Set_Start(1, w_PresetTime);
          RockFunctionEnable(false);
                    if(bEnableStretchDemoRun == TRUE)
          {
            bEnableStretchDemoRun = FALSE;//WGH 20161107
          }
          
          st_Stretch.active = FALSE;SetStretchingEnable(0);
          nBuzzerMode = BUZZER_MODE_ONETIME ;
          bSendBuzzerMode = TRUE ;
          
       /*   if(nTargetMassagePosition == MASSAGE_RESET_POSITION)
           {
                nTargetMassagePosition = MASSAGE_OPTIMAL2_POSITION;
                bMassagePositionUpdate = TRUE;
                w_ZeroPosition = 1;
           }    */
          
          
          
          break ;
        case H10_KEY_CHAIR_AUTO_1:
          if(bEnableStretchDemoRun == TRUE)
          {
            bEnableStretchDemoRun = FALSE;//WGH 20161107
          }
          else
          {
          if(nBackSubRunMode == BACK_SUB_MODE_AUTO_1)  break;
          }
          nBackMainRunMode = BACK_MAIN_MODE_AUTO ;
          nBackSubRunMode = BACK_SUB_MODE_AUTO_1 ;
          Main_Start_Auto(); 
         if(BodyDetectStep != DETECT_FINISH)
          {
            BodyDetectStep = DETECT_INITIAL;
          }
          Data_Set_Start(1, w_PresetTime);
         RockFunctionEnable(false);          

         
          //st_Stretch.active = FALSE;SetStretchingEnable(0);
          nBuzzerMode = BUZZER_MODE_ONETIME ;
          bSendBuzzerMode = TRUE ;
        /*  if(nTargetMassagePosition == MASSAGE_RESET_POSITION)
          {
             nTargetMassagePosition = MASSAGE_OPTIMAL2_POSITION;
             bMassagePositionUpdate = TRUE;
             w_ZeroPosition = 1;
          }          */ 
          
          
          
          break ;
        case H10_KEY_CHAIR_AUTO_2:
          if(nBackSubRunMode == BACK_SUB_MODE_AUTO_2)  break;
          nBackMainRunMode = BACK_MAIN_MODE_AUTO ;
          nBackSubRunMode = BACK_SUB_MODE_AUTO_2 ;
          Main_Start_Auto(); 
          if(BodyDetectStep != DETECT_FINISH)
          {
            BodyDetectStep = DETECT_INITIAL;
          }
          Data_Set_Start(1, w_PresetTime);
          nBuzzerMode = BUZZER_MODE_ONETIME ;
          bSendBuzzerMode = TRUE ;
         RockFunctionEnable(false);
         nRockMode = ROCK_IDLE;
         
          if(bEnableStretchDemoRun == TRUE)
          {
            bEnableStretchDemoRun = FALSE;//WGH 20161107
          }
          //st_Stretch.active = FALSE;SetStretchingEnable(0);     
       /*  if(nTargetMassagePosition == MASSAGE_RESET_POSITION)
          {
             nTargetMassagePosition = MASSAGE_OPTIMAL2_POSITION;
             bMassagePositionUpdate = TRUE;
             w_ZeroPosition = 1;
           }          */
          
          
          break ;
        case H10_KEY_CHAIR_AUTO_3:
          if(nBackSubRunMode == BACK_SUB_MODE_AUTO_3)  break;
          nBackMainRunMode = BACK_MAIN_MODE_AUTO ;
          nBackSubRunMode = BACK_SUB_MODE_AUTO_3 ;
          Main_Start_Auto(); 
         if(BodyDetectStep != DETECT_FINISH)
          {
            BodyDetectStep = DETECT_INITIAL;
          }
          Data_Set_Start(1, w_PresetTime);
         RockFunctionEnable(false);
          st_Stretch.active = FALSE;SetStretchingEnable(0);
          
          if(bEnableStretchDemoRun == TRUE)
          {
            bEnableStretchDemoRun = FALSE;//WGH 20161107
          }
          
          nBuzzerMode = BUZZER_MODE_ONETIME ;
          bSendBuzzerMode = TRUE ;   
          
      /*    if(nTargetMassagePosition == MASSAGE_RESET_POSITION)//add by taoqingosng
          {
                nTargetMassagePosition = MASSAGE_OPTIMAL2_POSITION;
                bMassagePositionUpdate = TRUE;
                w_ZeroPosition = 1;
          }          */
          
          
          break; 
        case H10_KEY_CHAIR_AUTO_4:
          if(nBackSubRunMode == BACK_SUB_MODE_AUTO_4)  break;
          nBackMainRunMode = BACK_MAIN_MODE_AUTO ;
          nBackSubRunMode = BACK_SUB_MODE_AUTO_4 ;
          Main_Start_Auto(); 
         if(BodyDetectStep != DETECT_FINISH)
          {
            BodyDetectStep = DETECT_INITIAL;////bShoulderOK == 0)   //进入主程序之前，先进行体型检测BodyDetectStep=DETECT_INITIAL
          }
          Data_Set_Start(1, w_PresetTime);
         RockFunctionEnable(false);
          st_Stretch.active = FALSE;SetStretchingEnable(0);
          
          if(bEnableStretchDemoRun == TRUE)
          {
            bEnableStretchDemoRun = FALSE;//WGH 20161107
          }
          nBuzzerMode = BUZZER_MODE_ONETIME ;
          bSendBuzzerMode = TRUE ;    
      /*    if(nTargetMassagePosition == MASSAGE_RESET_POSITION)
          {
                nTargetMassagePosition = MASSAGE_OPTIMAL2_POSITION;
                bMassagePositionUpdate = TRUE;
                w_ZeroPosition = 1;
          }           
          */
          
          
          
          break;   
        case H10_KEY_CHAIR_AUTO_5:  //不进行体型检测
          if(nBackSubRunMode == BACK_SUB_MODE_AUTO_5)  break;
          nBackMainRunMode = BACK_MAIN_MODE_AUTO;
          nBackSubRunMode = BACK_SUB_MODE_AUTO_5;
          Main_Start_Auto(); 

          Data_Set_Start(1, w_PresetTime);
          RockFunctionEnable(false);
          st_Stretch.active = FALSE;SetStretchingEnable(0);
          
          if(bEnableStretchDemoRun == TRUE)
          {
            bEnableStretchDemoRun = FALSE;//WGH 20161107
          }      
          
          nBuzzerMode = BUZZER_MODE_ONETIME ;
          bSendBuzzerMode = TRUE ;
      /*    if(nTargetMassagePosition == MASSAGE_RESET_POSITION)
          {
                nTargetMassagePosition = MASSAGE_OPTIMAL2_POSITION;
                bMassagePositionUpdate = TRUE;
                w_ZeroPosition = 1;
           }                     */                          
          break;     
/*
         case H10_KEY_CHAIR_CLOUD_0:
             
              if( pCludeAutoFunction_0 == NULL)
              {
                 nBuzzerMode = BUZZER_MODE_SLOW ;
                 bSendBuzzerMode = TRUE ;
                   bMasterSendPacket=1;
                
                break;  
              }
              
              if(nBackSubRunMode == BACK_SUB_MODE_CLUDE_AUTO_0)  break;
              RockFunctionEnable(false);
          st_Stretch.active = FALSE;SetStretchingEnable(0);
              nChairRunState = CHAIR_STATE_RUN ;
              nBackMainRunMode = BACK_MAIN_MODE_AUTO;
              nBackSubRunMode = BACK_SUB_MODE_CLUDE_AUTO_0;
              Main_Start_Auto(); 
              //BodyDetectStep = DETECT_FINISH;  //不进行体型检测
            if(BodyDetectStep != DETECT_FINISH)
            {
            BodyDetectStep = DETECT_INITIAL;
            }                       
              Data_Set_Start(1, w_PresetTime);
              nBuzzerMode = BUZZER_MODE_ONETIME ;
              bSendBuzzerMode = TRUE ;*/
          /*    if(nTargetMassagePosition == MASSAGE_RESET_POSITION)
              {
                    nTargetMassagePosition = MASSAGE_OPTIMAL2_POSITION;
                    bMassagePositionUpdate = TRUE;
                    w_ZeroPosition = 1;
               }       */ 
            
            
             // break;         
          
        /*      case H10_KEY_CHAIR_CLOUD_1:
             
              if( pCludeAutoFunction_1 == NULL)
              {
                 nBuzzerMode = BUZZER_MODE_SLOW ;
                 bSendBuzzerMode = TRUE ;
                   bMasterSendPacket=1;
                break;           
              }
              
              if(nBackSubRunMode == BACK_SUB_MODE_CLUDE_AUTO_1)  break;
              RockFunctionEnable(false);
          st_Stretch.active = FALSE;SetStretchingEnable(0);
              nChairRunState = CHAIR_STATE_RUN ;
              nBackMainRunMode = BACK_MAIN_MODE_AUTO;
              nBackSubRunMode = BACK_SUB_MODE_CLUDE_AUTO_1;
              Main_Start_Auto(); 
              //BodyDetectStep = DETECT_FINISH;  //不进行体型检测
            if(BodyDetectStep != DETECT_FINISH)
            {
            BodyDetectStep = DETECT_INITIAL;
            }  
              Data_Set_Start(1, w_PresetTime);
              nBuzzerMode = BUZZER_MODE_ONETIME ;
              bSendBuzzerMode = TRUE ;*/
            /*  if(nTargetMassagePosition == MASSAGE_RESET_POSITION)
              {
                    nTargetMassagePosition = MASSAGE_OPTIMAL2_POSITION;
                    bMassagePositionUpdate = TRUE;
                    w_ZeroPosition = 1;
               }        */
            
            
             //   break;              
          
      /*        case H10_KEY_CHAIR_CLOUD_2:
             
              if( pCludeAutoFunction_2 == NULL)
              {
                 nBuzzerMode = BUZZER_MODE_SLOW ;
                 bSendBuzzerMode = TRUE ;
                   bMasterSendPacket=1;
                break;           
              }
              
              if(nBackSubRunMode == BACK_SUB_MODE_CLUDE_AUTO_2)  break;
              RockFunctionEnable(false);
          st_Stretch.active = FALSE;SetStretchingEnable(0);
              nChairRunState = CHAIR_STATE_RUN ;
              nBackMainRunMode = BACK_MAIN_MODE_AUTO;
              nBackSubRunMode = BACK_SUB_MODE_CLUDE_AUTO_2;
              Main_Start_Auto(); 
              //BodyDetectStep = DETECT_FINISH;  //不进行体型检测
            if(BodyDetectStep != DETECT_FINISH)
            {
            BodyDetectStep = DETECT_INITIAL;
            }  
              Data_Set_Start(1, w_PresetTime);
              nBuzzerMode = BUZZER_MODE_ONETIME ;
              bSendBuzzerMode = TRUE ;*/
            /*  if(nTargetMassagePosition == MASSAGE_RESET_POSITION)
              {
                    nTargetMassagePosition = MASSAGE_OPTIMAL2_POSITION;
                    bMassagePositionUpdate = TRUE;
                    w_ZeroPosition = 1;
               }   */     
            
            
              //  break;             
          
           case H10_KEY_CHAIR_CLOUD_3:
             
              if( pCludeAutoFunction_3 == NULL)
              {
                 nBuzzerMode = BUZZER_MODE_SLOW ;
                 bSendBuzzerMode = TRUE ;
                   bMasterSendPacket=1;
                break;      
              }
              
              if(nBackSubRunMode == BACK_SUB_MODE_CLUDE_AUTO_3)  break;
              RockFunctionEnable(false);
          st_Stretch.active = FALSE;SetStretchingEnable(0);
              nChairRunState = CHAIR_STATE_RUN ;
              nBackMainRunMode = BACK_MAIN_MODE_AUTO;
              nBackSubRunMode = BACK_SUB_MODE_CLUDE_AUTO_3;
              Main_Start_Auto(); 
              //BodyDetectStep = DETECT_FINISH;  //不进行体型检测
             if(BodyDetectStep != DETECT_FINISH)
            {
            BodyDetectStep = DETECT_INITIAL;
            }  
              Data_Set_Start(1, w_PresetTime);
              nBuzzerMode = BUZZER_MODE_ONETIME ;
              bSendBuzzerMode = TRUE ;
          /*    if(nTargetMassagePosition == MASSAGE_RESET_POSITION)
              {
                    nTargetMassagePosition = MASSAGE_OPTIMAL2_POSITION;
                    bMassagePositionUpdate = TRUE;
                    w_ZeroPosition = 1;
               }        */
            
            
              break;             
                                     
         case H10_KEY_3DMODE_1:   //3D手法1
         if(nBackSubRunMode == BACK_SUB_MODE_3D1)  break;
           RockFunctionEnable(false);
          st_Stretch.active = FALSE;SetStretchingEnable(0);
          
          if(bEnableStretchDemoRun == TRUE)
          {
            bEnableStretchDemoRun = FALSE;//WGH 20161107
          }
          nBackMainRunMode = BACK_MAIN_MODE_3D;
         nBackSubRunMode = BACK_SUB_MODE_3D1 ;
         Main_Start_3D();
         if(BodyDetectStep != DETECT_FINISH)
          {
            BodyDetectStep = DETECT_INITIAL;
          }
         //Data_Set_Start(1, w_PresetTime);
         nBuzzerMode = BUZZER_MODE_ONETIME ;
         bSendBuzzerMode = TRUE ;
          nKeyBackLocate =  LOCATE_FULL_BACK;
         
         
         break;
        case H10_KEY_3DMODE_2:// //3D手法2

         if(nBackSubRunMode == BACK_SUB_MODE_3D2)  break;
          RockFunctionEnable(false);
          st_Stretch.active = FALSE;SetStretchingEnable(0);   
          
           if(bEnableStretchDemoRun == TRUE)
          {
            bEnableStretchDemoRun = FALSE;//WGH 20161107
          }
         nBackMainRunMode = BACK_MAIN_MODE_3D;
         nBackSubRunMode = BACK_SUB_MODE_3D2 ;
         Main_Start_3D();
         if(BodyDetectStep != DETECT_FINISH)
          {
            BodyDetectStep = DETECT_INITIAL;
          }
         nBuzzerMode = BUZZER_MODE_ONETIME ;
         bSendBuzzerMode = TRUE ;
         nKeyBackLocate =  LOCATE_FULL_BACK;
         
         break;  
        case H10_KEY_3DMODE_3:// //3D手法3
          if(nBackSubRunMode == BACK_SUB_MODE_3D3)  break;
          RockFunctionEnable(false);
          st_Stretch.active = FALSE;SetStretchingEnable(0);
          if(bEnableStretchDemoRun == TRUE)
          {
            bEnableStretchDemoRun = FALSE;//WGH 20161107
          }
  //       
         nBackMainRunMode = BACK_MAIN_MODE_3D;
         nBackSubRunMode = BACK_SUB_MODE_3D3 ;
         Main_Start_3D();
         if(BodyDetectStep != DETECT_FINISH)
          {
            BodyDetectStep = DETECT_INITIAL;
          }
         nBuzzerMode = BUZZER_MODE_ONETIME ;
         bSendBuzzerMode = TRUE ;
         nKeyBackLocate =  LOCATE_FULL_BACK;
         break;  
       
        case H10_KEY_3DMODE:
          
          if(nBackMainRunMode != BACK_MAIN_MODE_3D)
          {
            nBackMainRunMode = BACK_MAIN_MODE_3D;
            nBackSubRunMode = BACK_SUB_MODE_3D1 ;
          }
          else
          {
            switch(nBackSubRunMode)
            {
            case BACK_SUB_MODE_3D1: nBackSubRunMode = BACK_SUB_MODE_3D2; break;
            case BACK_SUB_MODE_3D2: nBackSubRunMode = BACK_SUB_MODE_3D3; break;  
            default:
            case BACK_SUB_MODE_3D3: nBackSubRunMode = BACK_SUB_MODE_3D1; break;  
            }   
          }
          Main_Start_3D();
         if(BodyDetectStep != DETECT_FINISH)
          {
            BodyDetectStep = DETECT_INITIAL;
          }
         
          if(bEnableStretchDemoRun == TRUE)
          {
            bEnableStretchDemoRun = FALSE;//WGH 20161107
          }
         Data_Set_Start(1, w_PresetTime);
         nBuzzerMode = BUZZER_MODE_ONETIME ;
         bSendBuzzerMode = TRUE ;
          break;    
         
         
        case H10_KEY_ZERO_START://work
          
          RockFunctionEnable(false);
          st_Stretch.active = FALSE;SetStretchingEnable(0);
             /* if(isZeroPosition())
              {
                nTargetMassagePosition = MASSAGE_INIT_POSITION;
              }
              else
              {
                nTargetMassagePosition = MASSAGE_OPTIMAL2_POSITION;bZeroflash = TRUE;
              }*/
          
          w_ZeroPosition++;
          w_ZeroPosition %= 3;  //0位置为初始位置   1位置为第一个零重力点    2位置为第二个零重力点    
          
          if(w_ZeroPosition == 0)
          {
            nTargetMassagePosition = MASSAGE_INIT_POSITION;
          }
          if(w_ZeroPosition == 1)
          {
            nTargetMassagePosition = MASSAGE_OPTIMAL_POSITION;//MASSAGE_OPTIMAL_POSITION;
          }
          if(w_ZeroPosition == 2)
          {
            nTargetMassagePosition = MASSAGE_OPTIMAL2_POSITION;
          }
          
          
          bMassagePositionUpdate = TRUE;
          nBuzzerMode = BUZZER_MODE_ONETIME ;
          bSendBuzzerMode = TRUE ;


          
          break;
          
        case H10_KEY_ZERO_1:	
          RockFunctionEnable(false);
          st_Stretch.active = FALSE;
          SetStretchingEnable(0);

            nTargetMassagePosition = MASSAGE_OPTIMAL_POSITION;//MASSAGE_OPTIMAL_POSITION;
          bMassagePositionUpdate = TRUE;
          nBuzzerMode = BUZZER_MODE_ONETIME ;
          bSendBuzzerMode = TRUE ;
          
          break;
        case H10_KEY_ZERO_2:	
          RockFunctionEnable(false);
          st_Stretch.active = FALSE;
          SetStretchingEnable(0);
   
            nTargetMassagePosition = MASSAGE_OPTIMAL2_POSITION;//MASSAGE_OPTIMAL_POSITION;
          bMassagePositionUpdate = TRUE;
          nBuzzerMode = BUZZER_MODE_ONETIME ;
          bSendBuzzerMode = TRUE ;
          
          break;
        case H10_KEY_ZERO_OFF:	
          RockFunctionEnable(false);
          st_Stretch.active = FALSE;
          SetStretchingEnable(0);

          nTargetMassagePosition = MASSAGE_INIT_POSITION;

          bMassagePositionUpdate = TRUE;
          nBuzzerMode = BUZZER_MODE_ONETIME ;
          bSendBuzzerMode = TRUE ;
          
          break;
          
          
          
          
          
        case  H10_KEY_3D_STRENGTH:     //3D力度手控器上的快捷键
#ifdef test_shoulder
           bShoulderOK =0;
          
           BodyDetectStep = DETECT_INITIAL;

          
#else         
          if((nBackMainRunMode == BACK_MAIN_MODE_MANUAL) || (nBackMainRunMode == BACK_MAIN_MODE_AUTO))
          {
            nKeyAxisStrength++;
            nKeyAxisStrength %= 5;
            bAxisUpdate = TRUE;   //3D电机行走标志位，为TRUE时，3D电机运行
            nAxisUpdateCounter = 0;
            nBuzzerMode = BUZZER_MODE_ONETIME ;
            bSendBuzzerMode = TRUE ;   
          }
          
 #endif       
          break;
        case H10_KEY_WORK_TIME_10MIN:
          w_PresetTime = RUN_TIME_10;
          Data_Update_Time(w_PresetTime);
          nBuzzerMode = BUZZER_MODE_ONETIME ;
          bSendBuzzerMode = TRUE ;
          break;
        case H10_KEY_WORK_TIME_20MIN:
          w_PresetTime = RUN_TIME_20;
          Data_Update_Time(w_PresetTime);
          nBuzzerMode = BUZZER_MODE_ONETIME ;
          bSendBuzzerMode = TRUE ;
          break;
        case H10_KEY_WORK_TIME_30MIN:
          w_PresetTime = RUN_TIME_30;
          Data_Update_Time(w_PresetTime);
          nBuzzerMode = BUZZER_MODE_ONETIME ;
          bSendBuzzerMode = TRUE ;
          break;
             
        case H10_KEY_AIRBAG_STRENGTH_1:
          Valve_SetAirBagStrength(AIRBAG_STRENGTH_1);
          nBuzzerMode = BUZZER_MODE_ONETIME ;
          bSendBuzzerMode = TRUE ;
          break;
        case H10_KEY_AIRBAG_STRENGTH_2:
          Valve_SetAirBagStrength(AIRBAG_STRENGTH_2);
          nBuzzerMode = BUZZER_MODE_ONETIME ;
          bSendBuzzerMode = TRUE ;
          break;
        case H10_KEY_AIRBAG_STRENGTH_3:
          Valve_SetAirBagStrength(AIRBAG_STRENGTH_3);
          nBuzzerMode = BUZZER_MODE_ONETIME ;
          bSendBuzzerMode = TRUE ;
          break;
        case H10_KEY_AIRBAG_STRENGTH_4:
          Valve_SetAirBagStrength(AIRBAG_STRENGTH_4);
          nBuzzerMode = BUZZER_MODE_ONETIME ;
          bSendBuzzerMode = TRUE ;
          break;
        case H10_KEY_AIRBAG_STRENGTH_5:
          Valve_SetAirBagStrength(AIRBAG_STRENGTH_5);
          nBuzzerMode = BUZZER_MODE_ONETIME ;
          bSendBuzzerMode = TRUE ;
          break;  
        case H10_KEY_AIRBAG_STRENGTH_OFF:
           if(nKeyAirBagLocate == AIRBAG_LOCATE_AUTO)
              {
                  nRollerPWM = 0;
                  bRollerEnable = FALSE;
                  Valve_SetRollerPWM(nRollerPWM);
                  Valve_SetAirBagStrength(AIRBAG_STRENGTH_0);
              }
           nKeyAirBagLocate = AIRBAG_LOCATE_NONE;
          break;    
          
 case H10_KEY_3DSPEED_1:
        case H10_KEY_3DSPEED_2:
        case H10_KEY_3DSPEED_3:
        case H10_KEY_3DSPEED_4:
        case H10_KEY_3DSPEED_5:
         if(nBackMainRunMode == BACK_MAIN_MODE_MANUAL)
          {
           // printf("[%d]\n",key);
            nKeyAxisStrength = key - H10_KEY_3DSPEED_1;// H10_KEY_3DSPEED_5 - key;
            nKeyAxisStrength %= 5;
            bAxisUpdate = TRUE;
            nAxisUpdateCounter = 0;
            nBuzzerMode = BUZZER_MODE_ONETIME ;
            bSendBuzzerMode = TRUE ;   
            
            _3D_Max_Position = 38; 
            _3D_More_Men_Position = 30; 
            _3D_Men_Position = 20; 
            _3D_More_Min_Position = 10; 
            _3D_Min_Position = 2; 
             break;   
            
          }
          if(nBackMainRunMode == BACK_MAIN_MODE_AUTO)
          {
           // printf("[%d]\n",key);
            nKeyAxisStrength = key - H10_KEY_3DSPEED_1;// H10_KEY_3DSPEED_5 - key;
            
            nKeyAxisStrength %= 5;
            //nKeyAxisStrength_all = nKeyAxisStrength;
            //nDisplayAxisStrength = nKeyAxisStrength;
            bAxisUpdate = TRUE;
            nAxisUpdateCounter = 0;
            nBuzzerMode = BUZZER_MODE_ONETIME ;
            bSendBuzzerMode = TRUE ; 

         _3D_Min_Position      = 2 + nKeyAxisStrength*4  ;
         _3D_More_Min_Position = 2 + nKeyAxisStrength*4 + 5  ;
         _3D_Men_Position      = 2 + nKeyAxisStrength*4 + 10  ;
         _3D_More_Men_Position = 2 + nKeyAxisStrength*4  + 15  ;
         _3D_Max_Position      = 2 + nKeyAxisStrength*4 + 20  ;         
          break ;
          }
          break;
         
        case H10_KEY_KNEAD_CLOCK:
        case H10_KEY_KNEAD_ANTICLOCK:
        case H10_KEY_KNEAD:
        case H10_KEY_KNOCK:
        case H10_KEY_WAVELET:
        case H10_KEY_SOFT_KNOCK:
        case H10_KEY_PRESS:
        case H10_KEY_MUSIC:             
        case H10_KEY_MANUAL:
           if((bRockEnable == true)&&(nRockMode == ROCK_AUTO ))
           {
              RockFunctionEnable(false);
           }
          //RockFunctionEnable(false);
          //st_Stretch.active = FALSE;SetStretchingEnable(0);
          if(nBackMainRunMode != BACK_MAIN_MODE_MANUAL)
          {
            nBackMainRunMode = BACK_MAIN_MODE_MANUAL;
            nMaunalSubMode = 5;
            nKeyBackLocate = LOCATE_FULL_BACK;//work
          }
          nBuzzerMode = BUZZER_MODE_ONETIME ;
          bSendBuzzerMode = TRUE ;
          
          if(key == H10_KEY_MANUAL)
          { 
            //设置背部功能
            BackManualModeNoAction() ;
            
          }
          //设置气囊功能
          //设置运行时间
          
          switch(key)
          {
          case H10_KEY_KNEAD:       
            nMaunalSubMode = nMaunalSubMode_KNEAD;
            break;
          case H10_KEY_KNEAD_CLOCK:
            nKneadTurn = 1;
            nMaunalSubMode = nMaunalSubMode_KNEAD;
            break;
          case H10_KEY_KNEAD_ANTICLOCK:
            nKneadTurn = 2;
            nMaunalSubMode = nMaunalSubMode_KNEAD;
            break;  
          
          
          case H10_KEY_KNOCK:       nMaunalSubMode = nMaunalSubMode_KNOCK;break;
          case H10_KEY_WAVELET:     nMaunalSubMode = nMaunalSubMode_WAVELET;break;
          case H10_KEY_SOFT_KNOCK:  nMaunalSubMode = nMaunalSubMode_SOFT_KNOCK;break;
          case H10_KEY_PRESS:       nMaunalSubMode = nMaunalSubMode_PRESS;break;
          case H10_KEY_MUSIC:       nMaunalSubMode = nMaunalSubMode_MUSIC;break;
          case H10_KEY_MANUAL:
            nMaunalSubMode++;
            nMaunalSubMode %= 6;
            break;
          }
          Main_Start_Manual();
          break ;
          


          case H10_KEY_LOCATE_FULL:
         case H10_KEY_LOCATE_POINT:
         case H10_KEY_LOCATE_PART:  //新手控器为局部
          if(nBackMainRunMode == BACK_MAIN_MODE_MANUAL)// break ;
          {
            switch(key)
            {
            case H10_KEY_LOCATE_FULL:     nKeyBackLocate = LOCATE_FULL_BACK; break;
            case H10_KEY_LOCATE_PART:     nKeyBackLocate = LOCATE_PARTIAL; break;
            case H10_KEY_LOCATE_POINT:    nKeyBackLocate = LOCATE_POINT; break;
            }   
            walkRefreshDown(nKeyBackLocate);
            bBackManualModeInit = TRUE ;
            nBuzzerMode = BUZZER_MODE_ONETIME ;
            bSendBuzzerMode = TRUE ;
          }
           if(nBackMainRunMode == BACK_MAIN_MODE_3D)
           {
             switch(key)
             {
             case H10_KEY_LOCATE_FULL:
              nKeyBackLocate = LOCATE_FULL_BACK;
              break;//work
             case H10_KEY_LOCATE_PART:
              nKeyBackLocate = LOCATE_PARTIAL;
  
      //
              if(Input_GetWalkMotorPosition() >= (TOP_POSITION - HALF_PARTIAL_DIFF))
              {
                nPartialTop = TOP_POSITION ;
                nPartialBottom = Input_GetWalkMotorPosition() - PARTIAL_DIFF ;
              }
              else if(Input_GetWalkMotorPosition() <= HALF_PARTIAL_DIFF)
              {
                nPartialTop = PARTIAL_DIFF ;
                nPartialBottom = 0 ;
              }
              else
              {
                nPartialTop = Input_GetWalkMotorPosition() + HALF_PARTIAL_DIFF ;
                nPartialBottom = Input_GetWalkMotorPosition() - HALF_PARTIAL_DIFF ;
              }
              if(n3Dpointturn%2==0)
              {
                bWalkMotorInProcess = TRUE ;
                bUpdateLocate = TRUE ;
                nWalkMotorControlParam1 = WALK_LOCATE_ABSULATE ;
                nWalkMotorControlParam2 = nPartialBottom ;
              }
              else
              {
                bWalkMotorInProcess = TRUE ;
                bUpdateLocate = TRUE ;
                nWalkMotorControlParam1 = WALK_LOCATE_ABSULATE ;
                nWalkMotorControlParam2 = nPartialTop ;
              }
              break;//work
             case H10_KEY_LOCATE_POINT:
              nKeyBackLocate = LOCATE_POINT;
              bWalkMotorInProcess = TRUE ;
              bUpdateLocate = TRUE ;
              nWalkMotorControlParam1 = WALK_LOCATE_PARK ;//AutoDirector.nWalkMotorLocateMethod ;
               nWalkMotorControlParam2 = 0;//MAX_PARK_TIME ;//AutoDirector.nWalkMotorLocateParam ;
              break;//work
             }   
             //walkRefreshen(nKeyBackLocate);
             //bBackManualModeInit = TRUE ;
             nBuzzerMode = BUZZER_MODE_ONETIME ;
             bSendBuzzerMode = TRUE ; 
             
           }
          break ; 
          
          
          
          
          
          
        case H10_KEY_SPEED_1:
        case H10_KEY_SPEED_2:
        case H10_KEY_SPEED_3:
        case H10_KEY_SPEED_4:
        case H10_KEY_SPEED_5:  
        case H10_KEY_SPEED_6:
          if(nBackMainRunMode != BACK_MAIN_MODE_MANUAL) break;
          if((nBackSubRunMode == BACK_SUB_MODE_PRESS) || (nBackSubRunMode == BACK_SUB_MODE_NO_ACTION)) break ;
          nBuzzerMode = BUZZER_MODE_ONETIME ;
          bSendBuzzerMode = TRUE ;

          
          if(key == H10_KEY_SPEED_1)
          {
            nKeyKneadKnockSpeed = 1;
          }
          
          if(key == H10_KEY_SPEED_2)
          {
            nKeyKneadKnockSpeed = 2;
          }
          if(key == H10_KEY_SPEED_3)
          {
            nKeyKneadKnockSpeed = 3;
          }
          if(key == H10_KEY_SPEED_4)
          {
            nKeyKneadKnockSpeed = 4;
          }
          if(key == H10_KEY_SPEED_5)
          {
            nKeyKneadKnockSpeed = 5;
          }
          if(key == H10_KEY_SPEED_6)
          {
            nKeyKneadKnockSpeed = 6;
          }
          ManualDirector[0].nKneadKnockSpeed = nKeyKneadKnockSpeed ;
          ManualDirector[1].nKneadKnockSpeed = nKeyKneadKnockSpeed ;
          ManualDirector[2].nKneadKnockSpeed = nKeyKneadKnockSpeed ;
          ManualDirector[3].nKneadKnockSpeed = nKeyKneadKnockSpeed ;
          nCurKneadKnockSpeed = nKeyKneadKnockSpeed ;    
          break ;
        case H10_KEY_WIDTH_INCREASE:
        case H10_KEY_WIDTH_DECREASE:
        case H10_KEY_WIDTH_MIN:  
        case H10_KEY_WIDTH_MED:  
        case H10_KEY_WIDTH_MAX: 
          if(nBackMainRunMode != BACK_MAIN_MODE_MANUAL) break ;
          if(!((nBackSubRunMode == BACK_SUB_MODE_KNOCK) || (nBackSubRunMode == BACK_SUB_MODE_PRESS) || (nBackSubRunMode == BACK_SUB_MODE_SOFT_KNOCK))) break ;
          nBuzzerMode = BUZZER_MODE_ONETIME ;
          bSendBuzzerMode = TRUE ;
          bKneadWidthChange = FALSE ;
          bKneadWidthChange = TRUE ;
          switch(key)
          {
          case  H10_KEY_WIDTH_INCREASE:
            {
              if(nKeyKneadWidth < 3)
              {
                nKeyKneadWidth++ ;
              }
              else
              {
                nKeyKneadWidth = 1 ;
              }
            }
            break;
          case H10_KEY_WIDTH_MIN:
            nKeyKneadWidth = KNEAD_WIDTH_MIN;
            break;
          case H10_KEY_WIDTH_MED:
            nKeyKneadWidth = KNEAD_WIDTH_MED;
            break;
          case H10_KEY_WIDTH_MAX:
            nKeyKneadWidth = KNEAD_WIDTH_MAX;
            break;
          }
          if(bKneadWidthChange == TRUE)
          {
            switch(nKeyKneadWidth)
            {
            case KNEAD_WIDTH_MIN:
              ManualDirector[0].nKneadMotorState = KNEAD_STOP_AT_MIN ;
              ManualDirector[1].nKneadMotorState = KNEAD_STOP_AT_MIN ;
              ManualDirector[2].nKneadMotorState = KNEAD_STOP_AT_MIN ;
              ManualDirector[3].nKneadMotorState = KNEAD_STOP_AT_MIN ;
              break ;
            case KNEAD_WIDTH_MED:
              ManualDirector[0].nKneadMotorState = KNEAD_STOP_AT_MED ;
              ManualDirector[1].nKneadMotorState = KNEAD_STOP_AT_MED ;
              ManualDirector[2].nKneadMotorState = KNEAD_STOP_AT_MED ;
              ManualDirector[3].nKneadMotorState = KNEAD_STOP_AT_MED ;
              break ;
            case KNEAD_WIDTH_MAX:
              ManualDirector[0].nKneadMotorState = KNEAD_STOP_AT_MAX ;
              ManualDirector[1].nKneadMotorState = KNEAD_STOP_AT_MAX ;
              ManualDirector[2].nKneadMotorState = KNEAD_STOP_AT_MAX ;
              ManualDirector[3].nKneadMotorState = KNEAD_STOP_AT_MAX ;
              break ;
            }
            ManualDirector[0].nKneadMotorCycles = 0 ;
            //重新定位
            nKneadMotorControlParam1 = ManualDirector[0].nKneadMotorState ;
            nKneadMotorControlParam2 = 0 ;
            bKneadMotorInProcess = TRUE ;
            //Knock motor 要等定位完成后进行
            bKnockMotorInProcess = TRUE ;
          }
          break ;           
 //-----------------------------------------------------------------------------------         
           /*typedef struct
          {
              unsigned char init; 
              unsigned char active; 
              unsigned char nCurAirBagStep;
              unsigned char nCurKeepTime1;
              unsigned char nCurKeepTime2; 
              unsigned char nCurKeepTime3; 
              unsigned char nCurKeepTime4;
              unsigned char nCurKeepTime5; 
              const struct AirBagStruct * pAirBagArray;
              UINT32 nCurPumpValveState;
              UINT16 nTotalSteps;
              unsigned char nAirBagCounter ;
              unsigned char locate ;
          }st_AirBag;            */                                   
         
          
    
                
  //------------------------------------------------------------------------        
          
         case H10_KEY_AIRBAG_AUTO:
              if(nKeyAirBagLocate != AIRBAG_LOCATE_AUTO)
              {
                nKeyAirBagLocate = AIRBAG_LOCATE_AUTO ;
                if( Valve_GetAirBagStrength() == AIRBAG_STRENGTH_0)
                {
                  Valve_SetAirBagStrength(AIRBAG_STRENGTH_1);
                }

                st_AirBagAuto.pAirBagArray = AirBagModeAuto;
                st_AirBagAuto.nTotalSteps = sizeof(AirBagModeAuto)/sizeof(struct AirBagStruct);
                st_AirBagLegFoot.init = TRUE ;
                st_AirBagArmNeck.init = TRUE ;
                st_AirBagAuto.init = TRUE;       
                
                bRollerEnable = TRUE;nRoller3sCnt = 0;
              }
              else // nKeyAirBagLocate = AIRBAG_LOCATE_NONE;
              {
                  nKeyAirBagLocate = AIRBAG_LOCATE_NONE ;
                  nRollerPWM = 0;
                  bRollerEnable = FALSE;
                  Valve_SetRollerPWM(nRollerPWM);
                  Valve_SetAirBagStrength(AIRBAG_STRENGTH_0);
              }
              nBuzzerMode = BUZZER_MODE_ONETIME ;
              bSendBuzzerMode = TRUE ;
              break ;          
          
          
         /*       if(nKeyAirBagLocate == AIRBAG_LOCATE_AUTO)
              {
                  nRollerPWM = 0;
                  bRollerEnable = FALSE;
                  Valve_SetRollerPWM(nRollerPWM);
                  Valve_SetAirBagStrength(AIRBAG_STRENGTH_0);
              }
           nKeyAirBagLocate = AIRBAG_LOCATE_NONE;*/         
              
              
              
              
          
        case H10_KEY_AIRBAG_LEG:   //气囊运动程序,气囊按摩区域,对应outbuf[12]2.3.4位，选中的气囊按摩程序,对应通信协议的 地址12 时间和气囊
          if(nKeyAirBagLocate == AIRBAG_LOCATE_AUTO)//全身气囊程序
          {
            bRollerEnable = false;
            nKeyAirBagLocate  = AIRBAG_LOCATE_NONE;
          }
          if( Valve_GetAirBagStrength() == AIRBAG_STRENGTH_0)
          {
            Valve_SetAirBagStrength(AIRBAG_STRENGTH_1);
          }
          switch(nKeyAirBagLocate)
          {
          case AIRBAG_LOCATE_NONE:
            nKeyAirBagLocate  = AIRBAG_LOCATE_LEG_FOOT;
            st_AirBagLegFoot.init = TRUE ;
            break;
          case AIRBAG_LOCATE_LEG_FOOT:
            
            nKeyAirBagLocate  = AIRBAG_LOCATE_NONE;
            st_AirBagLegFoot.init = FALSE ;
            Valve_SetAirBagStrength(AIRBAG_STRENGTH_0);
            break;   
            //加
          case AIRBAG_LOCATE_ARM_NECK:
            nKeyAirBagLocate  |= AIRBAG_LOCATE_LEG_FOOT;
            st_AirBagLegFoot_Arm.init = TRUE ;
            break; 
          case AIRBAG_LOCATE_SEAT:  
            nKeyAirBagLocate  |= AIRBAG_LOCATE_LEG_FOOT;
            st_AirBagLegFoot_Seat.init = TRUE ;
            break; 
          case AIRBAG_LOCATE_ARM_NECK|AIRBAG_LOCATE_SEAT: 
            nKeyAirBagLocate  |= AIRBAG_LOCATE_LEG_FOOT;
            st_AirBagLegFoot_Arm_Seat.init = TRUE ;
            break; 
            //去
          case AIRBAG_LOCATE_LEG_FOOT|AIRBAG_LOCATE_ARM_NECK:
            nKeyAirBagLocate  &= ~AIRBAG_LOCATE_LEG_FOOT;
            st_AirBagArmNeck.init = TRUE ;
            break;  
          case AIRBAG_LOCATE_LEG_FOOT|AIRBAG_LOCATE_SEAT: 
            nKeyAirBagLocate  &= ~AIRBAG_LOCATE_LEG_FOOT;
            st_AirBagSeat.init = TRUE ;
            break;  
          case AIRBAG_LOCATE_LEG_FOOT|AIRBAG_LOCATE_ARM_NECK|AIRBAG_LOCATE_SEAT: 
            nKeyAirBagLocate  &= ~AIRBAG_LOCATE_LEG_FOOT;
            st_AirBagArm_Seat.init = TRUE ;
            break;  
          }        
          
          
          
          
          
          
          nBuzzerMode = BUZZER_MODE_ONETIME ;
          bSendBuzzerMode = TRUE ;
          break;

       case H10_KEY_AIRBAG_WAIST://腰背气囊程序
         /*  if(nKeyAirBagLocate == AIRBAG_LOCATE_AUTO)
              {
                bRollerEnable = false;
              }
           
         if(nKeyAirBagLocate != AIRBAG_LOCATE_BACK_WAIST) 
         {
            nKeyAirBagLocate = AIRBAG_LOCATE_BACK_WAIST ;
            st_AirBagBackWaist.init = TRUE ;
            if( Valve_GetAirBagStrength() == AIRBAG_STRENGTH_0)
            {
              Valve_SetAirBagStrength(AIRBAG_STRENGTH_3);
            }
         }
         else
         {
            //nKeyAirBagLocate = AIRBAG_LOCATE_NONE ;
         }
         
          nBuzzerMode = BUZZER_MODE_ONETIME ;
          bSendBuzzerMode = TRUE ;*/
          break;
        case H10_KEY_AIRBAG_BUTTOCKS:   //坐垫气囊程序
          if(nKeyAirBagLocate == AIRBAG_LOCATE_AUTO)
          {
            bRollerEnable = false;
            nKeyAirBagLocate = AIRBAG_LOCATE_NONE ;
          }
           
         /* if(nKeyAirBagLocate != AIRBAG_LOCATE_SEAT)   
          {
           
              nKeyAirBagLocate = AIRBAG_LOCATE_SEAT ;
              st_AirBagSeat.init = TRUE ;
              if( Valve_GetAirBagStrength() == AIRBAG_STRENGTH_0)
              {
                Valve_SetAirBagStrength(AIRBAG_STRENGTH_1);
              }
          }
          else
          {
            
             //nKeyAirBagLocate = AIRBAG_LOCATE_NONE ;
          }*/
          if( Valve_GetAirBagStrength() == AIRBAG_STRENGTH_0)
          {
            Valve_SetAirBagStrength(AIRBAG_STRENGTH_1);
          }
          switch(nKeyAirBagLocate)
          {
          case AIRBAG_LOCATE_NONE:
            nKeyAirBagLocate  = AIRBAG_LOCATE_SEAT;
            st_AirBagSeat.init = TRUE ;
            break;
          case AIRBAG_LOCATE_SEAT:
                st_AirBagSeat.init = false;
                nKeyAirBagLocate = AIRBAG_LOCATE_NONE ;
                Valve_SetAirBagStrength(AIRBAG_STRENGTH_0);
            break;   
            //加
          case AIRBAG_LOCATE_ARM_NECK:
            nKeyAirBagLocate  |= AIRBAG_LOCATE_SEAT;
            st_AirBagArm_Seat.init = TRUE ;
            break; 
          case AIRBAG_LOCATE_LEG_FOOT:  
            nKeyAirBagLocate  |= AIRBAG_LOCATE_SEAT;
            st_AirBagLegFoot_Seat.init = TRUE ;
            break; 
          case AIRBAG_LOCATE_ARM_NECK|AIRBAG_LOCATE_LEG_FOOT: 
            nKeyAirBagLocate  |= AIRBAG_LOCATE_SEAT;
            st_AirBagLegFoot_Arm_Seat.init = TRUE ;
            break; 
            //去
          case AIRBAG_LOCATE_SEAT|AIRBAG_LOCATE_ARM_NECK:
            nKeyAirBagLocate  &= ~AIRBAG_LOCATE_SEAT;
            st_AirBagArmNeck.init = TRUE ;
            break;  
          case AIRBAG_LOCATE_SEAT|AIRBAG_LOCATE_LEG_FOOT: 
            nKeyAirBagLocate  &= ~AIRBAG_LOCATE_SEAT;
            st_AirBagLegFoot.init = TRUE ;
            break;  
          case AIRBAG_LOCATE_LEG_FOOT|AIRBAG_LOCATE_ARM_NECK|AIRBAG_LOCATE_SEAT: 
            nKeyAirBagLocate  &= ~AIRBAG_LOCATE_SEAT;
            st_AirBagLegFoot_Arm.init = TRUE ;
            break;  
          }        
          nBuzzerMode = BUZZER_MODE_ONETIME ;
          bSendBuzzerMode = TRUE ;
          
        
          break;
          
          

          
        case H10_KEY_AIRBAG_ARM://臂肩气囊   实际使用的是臂肩气囊
           if(nKeyAirBagLocate == AIRBAG_LOCATE_AUTO)
              {
                bRollerEnable = false;
                nKeyAirBagLocate = AIRBAG_LOCATE_NONE ;
              }
           /*
            if(nKeyAirBagLocate != AIRBAG_LOCATE_ARM_NECK)   
            {
              nKeyAirBagLocate = AIRBAG_LOCATE_ARM_NECK ;
              st_AirBagArmNeck.init = TRUE ;
              if( Valve_GetAirBagStrength() == AIRBAG_STRENGTH_0)
              {
                Valve_SetAirBagStrength(AIRBAG_STRENGTH_1);
              }
            }
             else
             {
                 //nKeyAirBagLocate = AIRBAG_LOCATE_NONE ;
             }*/
          if( Valve_GetAirBagStrength() == AIRBAG_STRENGTH_0)
          {
            Valve_SetAirBagStrength(AIRBAG_STRENGTH_1);
          }
          switch(nKeyAirBagLocate)
          {
          case AIRBAG_LOCATE_NONE:
            nKeyAirBagLocate  = AIRBAG_LOCATE_ARM_NECK;
            st_AirBagArmNeck.init = TRUE ;
            break;
          case AIRBAG_LOCATE_ARM_NECK:
            nKeyAirBagLocate  = AIRBAG_LOCATE_NONE;
            st_AirBagArmNeck.init = FALSE ;
            Valve_SetAirBagStrength(AIRBAG_STRENGTH_0);
            break;   
            //加
          case AIRBAG_LOCATE_LEG_FOOT:
            nKeyAirBagLocate  |= AIRBAG_LOCATE_ARM_NECK;
            st_AirBagLegFoot_Arm.init = TRUE ;
            break; 
          case AIRBAG_LOCATE_SEAT:  
            nKeyAirBagLocate  |= AIRBAG_LOCATE_ARM_NECK;
            st_AirBagArm_Seat.init = TRUE ;
            break; 
          case AIRBAG_LOCATE_LEG_FOOT|AIRBAG_LOCATE_SEAT: 
            nKeyAirBagLocate  |= AIRBAG_LOCATE_ARM_NECK;
            st_AirBagLegFoot_Arm_Seat.init = TRUE ;
            break; 
            //去
          case AIRBAG_LOCATE_ARM_NECK|AIRBAG_LOCATE_LEG_FOOT:
            nKeyAirBagLocate  &= ~AIRBAG_LOCATE_ARM_NECK;
            st_AirBagLegFoot.init = TRUE ;
            break;  
          case AIRBAG_LOCATE_ARM_NECK|AIRBAG_LOCATE_SEAT: 
            nKeyAirBagLocate  &= ~AIRBAG_LOCATE_ARM_NECK;
            st_AirBagSeat.init = TRUE ;
            break;  
          case AIRBAG_LOCATE_ARM_NECK|AIRBAG_LOCATE_LEG_FOOT|AIRBAG_LOCATE_SEAT: 
            nKeyAirBagLocate  &= ~AIRBAG_LOCATE_ARM_NECK;
            st_AirBagArm_Seat.init = TRUE ;
            break;  
          }        
              nBuzzerMode = BUZZER_MODE_ONETIME ;
              bSendBuzzerMode = TRUE ;
              break;        
          
          
          
        //---------------------------------------------------------------------------  
        case H10_KEY_WALK_UP_START://用户按下向上行走按键
        
          if((nBackMainRunMode == BACK_MAIN_MODE_MANUAL) && (nKeyBackLocate == LOCATE_POINT))
          {
            ManualDirector[0].nWalkMotorLocateMethod = WALK_LOCATE_TOP ;
            ManualDirector[0].nWalkMotorLocateParam = 0 ;
            bBackManualModeInit = TRUE ;
            bKeyWalkUp = TRUE ;
          }
          if((nBackMainRunMode == BACK_MAIN_MODE_MANUAL)&&(nKeyBackLocate == LOCATE_PARTIAL))
          {
           //ManualDirector[0].nWalkMotorLocateMethod = WALK_LOCATE_TOP ;
            //ManualDirector[0].nWalkMotorLocateParam = 0 ;
              ManualDirector[0].nWalkMotorLocateMethod = WALK_LOCATE_TOP ;
              ManualDirector[0].nWalkMotorLocateParam = 0;      
              ManualDirector[1].nWalkMotorLocateMethod = WALK_LOCATE_TOP ;
              ManualDirector[1].nWalkMotorLocateParam = 0;
              ManualDirector[2].nWalkMotorLocateMethod = WALK_LOCATE_TOP ;
              ManualDirector[2].nWalkMotorLocateParam = 0;
              ManualDirector[3].nWalkMotorLocateMethod = WALK_LOCATE_TOP ;
              ManualDirector[3].nWalkMotorLocateParam = 0;            
            bBackManualModeInit = TRUE ;
            
          //   if(Input_GetWalkUpSwitch() != REACH_WALK_LIMIT) bKeyWalkUp = TRUE ;
           //  else bKeyWalkUp =FALSE;// TRUE ;
            bKeyWalkUp = TRUE ;
           
          }
          ////////
          if((nBackMainRunMode == BACK_MAIN_MODE_3D)&&(nKeyBackLocate == LOCATE_POINT))
          {
           
              bWalkMotorInProcess = TRUE ;
              bUpdateLocate = TRUE ;
              nWalkMotorControlParam1 = WALK_LOCATE_TOP;//WALK_LOCATE_ABSULATE ;
              nWalkMotorControlParam2 = 0;//WALK_LOCATE_TOP ;
              bKeyWalkUp = TRUE ;
          }
          if((nBackMainRunMode == BACK_MAIN_MODE_3D)&&(nKeyBackLocate == LOCATE_PARTIAL))
          {
            
              bWalkMotorInProcess = TRUE ;
              bUpdateLocate = TRUE ;
              nWalkMotorControlParam1 = WALK_LOCATE_TOP;//WALK_LOCATE_ABSULATE ;
              nWalkMotorControlParam2 = 0;//WALK_LOCATE_TOP ;
              bKeyWalkUp = TRUE ;
              
          }
           if(ShoulderSteps == BODY_DETECT_ADJ)
               {
                 bKeyWalkUp = TRUE ;
               }    
          
          

          break ;
        case H10_KEY_WALK_UP_STOP:// 手动定点按摩释放按键值
          
          bKeyWalkUp = FALSE ;
          bKeyWalkDown = FALSE ; //only pc test
          nBuzzerMode = BUZZER_MODE_OFF ;
          bSendBuzzerMode = TRUE ;
          if((nBackMainRunMode == BACK_MAIN_MODE_MANUAL) && (nKeyBackLocate == LOCATE_POINT))
          {
            ManualDirector[0].nWalkMotorLocateMethod = WALK_LOCATE_PARK ;
            ManualDirector[0].nWalkMotorLocateParam = MAX_PARK_TIME ;
            bBackManualModeInit = TRUE ;
          }
          if((nBackMainRunMode == BACK_MAIN_MODE_MANUAL)&&(nKeyBackLocate == LOCATE_PARTIAL))
          {
             walkRefreshUp(nKeyBackLocate);
             bBackManualModeInit = TRUE ;
          }
          /////
          if((nBackMainRunMode == BACK_MAIN_MODE_3D)&&(nKeyBackLocate == LOCATE_POINT))
          {
            bWalkMotorInProcess = TRUE ;
            bUpdateLocate = TRUE ;
            nWalkMotorControlParam1 = WALK_LOCATE_PARK ;//AutoDirector.nWalkMotorLocateMethod ;
            nWalkMotorControlParam2 = 0;//MAX_PARK_TIME ;//AutoDirector.nWalkMotorLocateParam ; 
                    
            //WalkMotor_Control(STATE_WALK_IDLE,0);
           // nCurActionStepCounter = 0 ;
          }
          if((nBackMainRunMode == BACK_MAIN_MODE_3D)&&(nKeyBackLocate == LOCATE_PARTIAL))
          {
            //bWalkMotorInProcess = TRUE ;
            //bUpdateLocate = TRUE ;
            //nWalkMotorControlParam1 = WALK_LOCATE_PARK ;//AutoDirector.nWalkMotorLocateMethod ;
            //nWalkMotorControlParam2 = 0;//MAX_PARK_TIME ;//AutoDirector.nWalkMotorLocateParam ; 
            //
            if(Input_GetWalkMotorPosition() >= (TOP_POSITION - HALF_PARTIAL_DIFF))
            {
              nPartialTop = TOP_POSITION ;
              nPartialBottom = Input_GetWalkMotorPosition() - PARTIAL_DIFF ;
            }
            else if(Input_GetWalkMotorPosition() <= HALF_PARTIAL_DIFF)
            {
              nPartialTop = PARTIAL_DIFF ;
              nPartialBottom = 0 ;
            }
            else
            {
              nPartialTop = Input_GetWalkMotorPosition() + HALF_PARTIAL_DIFF ;
              nPartialBottom = Input_GetWalkMotorPosition() - HALF_PARTIAL_DIFF ;
            }
            
            if(n3Dpointturn%2==0)
            {
              bWalkMotorInProcess = TRUE ;
              bUpdateLocate = TRUE ;
              nWalkMotorControlParam1 = WALK_LOCATE_ABSULATE ;
              nWalkMotorControlParam2 = nPartialBottom ;
            }
            else
            {
              bWalkMotorInProcess = TRUE ;
              bUpdateLocate = TRUE ;
              nWalkMotorControlParam1 = WALK_LOCATE_ABSULATE ;
              nWalkMotorControlParam2 = nPartialTop ;
            }
            
            
          }  
          
          

          break ;
          //---------------------------------------------------------------
        case H10_KEY_WALK_DOWN_START:
          
          if((nBackMainRunMode == BACK_MAIN_MODE_MANUAL) && (nKeyBackLocate == LOCATE_POINT))
          {
            ManualDirector[0].nWalkMotorLocateMethod = WALK_LOCATE_ABSULATE ;
            ManualDirector[0].nWalkMotorLocateParam = 0 ;
            bBackManualModeInit = TRUE ;
            bKeyWalkDown = TRUE ;
          }
          if((nBackMainRunMode == BACK_MAIN_MODE_MANUAL)&&(nKeyBackLocate == LOCATE_PARTIAL))
          {
           //ManualDirector[0].nWalkMotorLocateMethod = WALK_LOCATE_ABSULATE ;
            //ManualDirector[0].nWalkMotorLocateParam = 0 ;
              ManualDirector[0].nWalkMotorLocateMethod = WALK_LOCATE_ABSULATE ;
              ManualDirector[0].nWalkMotorLocateParam = 0;      
              ManualDirector[1].nWalkMotorLocateMethod = WALK_LOCATE_ABSULATE ;
              ManualDirector[1].nWalkMotorLocateParam = 0;
              ManualDirector[2].nWalkMotorLocateMethod = WALK_LOCATE_ABSULATE ;
              ManualDirector[2].nWalkMotorLocateParam = 0;
              ManualDirector[3].nWalkMotorLocateMethod = WALK_LOCATE_ABSULATE ;
              ManualDirector[3].nWalkMotorLocateParam = 0;       
            
            bBackManualModeInit = TRUE ;
            bKeyWalkDown = TRUE ;
          }
          //////
          if((nBackMainRunMode == BACK_MAIN_MODE_3D)&&(nKeyBackLocate == LOCATE_POINT))
          {
              bWalkMotorInProcess = TRUE ;
              bUpdateLocate = TRUE ;
              nWalkMotorControlParam1 = WALK_LOCATE_ABSULATE ;
              nWalkMotorControlParam2 = 0 ;
              bKeyWalkDown = TRUE ;
          }
          if((nBackMainRunMode == BACK_MAIN_MODE_3D)&&(nKeyBackLocate == LOCATE_PARTIAL))
          {
              bWalkMotorInProcess = TRUE ;
              bUpdateLocate = TRUE ;
              nWalkMotorControlParam1 = WALK_LOCATE_ABSULATE ;
              nWalkMotorControlParam2 = 0 ;
              bKeyWalkDown = TRUE ;
          }
          if(ShoulderSteps == BODY_DETECT_ADJ)
               {
                 bKeyWalkDown = TRUE ;
               }    
          

          break ;
        case H10_KEY_WALK_DOWN_STOP:
          
          bKeyWalkDown = FALSE ;
          nBuzzerMode = BUZZER_MODE_OFF ;
          bSendBuzzerMode = TRUE ;
          if((nBackMainRunMode == BACK_MAIN_MODE_MANUAL) && (nKeyBackLocate == LOCATE_POINT))
          {
            ManualDirector[0].nWalkMotorLocateMethod = WALK_LOCATE_PARK ;
            ManualDirector[0].nWalkMotorLocateParam = MAX_PARK_TIME ;
            bBackManualModeInit = TRUE ;
          }
          if((nBackMainRunMode == BACK_MAIN_MODE_MANUAL)&&(nKeyBackLocate == LOCATE_PARTIAL))
          {
             walkRefreshDown(nKeyBackLocate);
             bBackManualModeInit = TRUE ;
          }
          ////
           if((nBackMainRunMode == BACK_MAIN_MODE_3D)&&(nKeyBackLocate == LOCATE_POINT))
          {
            bWalkMotorInProcess = TRUE ;
            bUpdateLocate = TRUE ;
            nWalkMotorControlParam1 = WALK_LOCATE_PARK ;//AutoDirector.nWalkMotorLocateMethod ;
            nWalkMotorControlParam2 = 0;//MAX_PARK_TIME ;//AutoDirector.nWalkMotorLocateParam ; 
          } 
          if((nBackMainRunMode == BACK_MAIN_MODE_3D)&&(nKeyBackLocate == LOCATE_PARTIAL))
          {
            //bWalkMotorInProcess = TRUE ;
            //bUpdateLocate = TRUE ;
            //nWalkMotorControlParam1 = WALK_LOCATE_PARK ;//AutoDirector.nWalkMotorLocateMethod ;
            //nWalkMotorControlParam2 = 0;//MAX_PARK_TIME ;//AutoDirector.nWalkMotorLocateParam ; 
            //WalkMotor_Control(STATE_WALK_IDLE,0);
            //nCurActionStepCounter = 0 ;
            if(Input_GetWalkMotorPosition() >= (TOP_POSITION - HALF_PARTIAL_DIFF))
            {
              nPartialTop = TOP_POSITION ;
              nPartialBottom = Input_GetWalkMotorPosition() - PARTIAL_DIFF ;
            }
            else if(Input_GetWalkMotorPosition() <= HALF_PARTIAL_DIFF)
            {
              nPartialTop = PARTIAL_DIFF ;
              nPartialBottom = 0 ;
            }
            else
            {
              nPartialTop = Input_GetWalkMotorPosition() + HALF_PARTIAL_DIFF ;
              nPartialBottom = Input_GetWalkMotorPosition() - HALF_PARTIAL_DIFF ;
            }
            if(n3Dpointturn%2==0)
            {
              bWalkMotorInProcess = TRUE ;
              bUpdateLocate = TRUE ;
              nWalkMotorControlParam1 = WALK_LOCATE_ABSULATE ;
              nWalkMotorControlParam2 = nPartialBottom ;
            }
            else
            {
              bWalkMotorInProcess = TRUE ;
              bUpdateLocate = TRUE ;
              nWalkMotorControlParam1 = WALK_LOCATE_ABSULATE ;
              nWalkMotorControlParam2 = nPartialTop ;
            }
            
            
            
            
          } 
          

          break ;
         //-----------------------------------------------------------------------------
        case H10_KEY_BACKPAD_UP_START:  //用户按下靠背向上按键不放时，小腿此时也要向下运行，故设定联动标志位
          st_Stretch.active = FALSE;
          RockFunctionEnable(false);
          st_Stretch.active = FALSE;SetStretchingEnable(0);
          bKeyBackPadUp = TRUE ;
          bKeyBackPadDown = FALSE ;
          //小腿联动设置
          bKeyLegPadDown = TRUE ;  //设定小腿向下按键标志位
          bKeyLegPadUp = FALSE ;
          bLegPadLinkage = TRUE ;    //小腿联动标志位
          bKeyFlexOut = FALSE ;
          bKeyFlexIn = FALSE ;bKeyFlexIn = FALSE ;bZeroflash = FALSE;
          break ;
        case H10_KEY_BACKPAD_UP_STOP:  //用户释放靠背向上按键
          RockFunctionEnable(false);
          st_Stretch.active = FALSE;SetStretchingEnable(0);
          st_Stretch.active = FALSE;
          bKeyBackPadUp = FALSE ;
          bKeyBackPadDown = FALSE ;
          //小腿联动设置
          bKeyLegPadDown = FALSE ;
          bKeyLegPadUp = FALSE ;
          bLegPadLinkage = TRUE ;
          bKeyFlexOut = FALSE ;
          bKeyFlexIn = FALSE ;
          break ;
         //-----------------------------------------------------------------------
        case H10_KEY_BACKPAD_DOWN_START://靠背向下运行
          RockFunctionEnable(false);
          st_Stretch.active = FALSE;SetStretchingEnable(0);
          st_Stretch.active = FALSE;
          bKeyBackPadUp = FALSE ;
          bKeyBackPadDown = TRUE ;
          //小腿联动设置
          bKeyLegPadDown = FALSE ;
          bKeyLegPadUp = TRUE ;   //小腿向上联动
          bLegPadLinkage = TRUE ;
          bKeyFlexOut = FALSE ;
          bKeyFlexIn = FALSE ;bKeyFlexIn = FALSE ;bZeroflash = FALSE;
          break ;
        case H10_KEY_BACKPAD_DOWN_STOP:
          RockFunctionEnable(false);
          st_Stretch.active = FALSE;SetStretchingEnable(0);
          st_Stretch.active = FALSE;
          bKeyBackPadUp = FALSE ;
          bKeyBackPadDown = FALSE ;
          //小腿联动设置
          bKeyLegPadDown = FALSE ;
          bKeyLegPadUp = FALSE ;
          bLegPadLinkage = TRUE ;
          bKeyFlexOut = FALSE ;
          bKeyFlexIn = FALSE ;
          break ;
          //-----------------------------------------------------------------------
        case H10_KEY_LEGPAD_EXTEND_START://小腿向外伸
          RockFunctionEnable(false);
          st_Stretch.active = FALSE;SetStretchingEnable(0);
          st_Stretch.active = FALSE;
          bKeyLegPadUp = FALSE ;
          bKeyLegPadDown = FALSE ;
          bLegPadLinkage = FALSE ;
          bKeyBackPadUp = FALSE ;
          bKeyBackPadDown = FALSE ;
          bKeyFlexOut = TRUE ;    //小腿向外伸
          bKeyFlexIn = FALSE ;

          
          
          break;
        case H10_KEY_LEGPAD_EXTEND_STOP:
        case H10_KEY_LEGPAD_CONTRACT_STOP:
          RockFunctionEnable(false);
          st_Stretch.active = FALSE;SetStretchingEnable(0);
          st_Stretch.active = FALSE;
          bKeyLegPadUp = FALSE ;
          bKeyLegPadDown = FALSE ;
          bLegPadLinkage = FALSE ;
          bKeyBackPadUp = FALSE ;
          bKeyBackPadDown = FALSE ;
          bKeyFlexOut = FALSE;
          bKeyFlexIn = FALSE ;
          break;

        case H10_KEY_LEGPAD_CONTRACT_START:
          RockFunctionEnable(false);
          st_Stretch.active = FALSE;SetStretchingEnable(0);
          st_Stretch.active = FALSE;
          bKeyLegPadUp = FALSE ;
          bKeyLegPadDown = FALSE ;
          bLegPadLinkage = FALSE ;
          bKeyBackPadUp = FALSE ;
          bKeyBackPadDown = FALSE ;
          bKeyFlexOut = FALSE;
          bKeyFlexIn = TRUE ;//x小腿向里缩

          
          
          break;
          //--------------------------------------------------
        case H10_KEY_LEGPAD_UP_START://小腿向上
          RockFunctionEnable(false);
          st_Stretch.active = FALSE;SetStretchingEnable(0);
          st_Stretch.active = FALSE;
          bKeyLegPadUp = TRUE ;
          bKeyLegPadDown = FALSE ;
          bLegPadLinkage = FALSE ;
          bKeyBackPadUp = FALSE ;
          bKeyBackPadDown = FALSE ;
          bKeyFlexOut = FALSE ;
          bKeyFlexIn = FALSE ;bKeyFlexIn = FALSE ;bZeroflash = FALSE;
          break ;

        case H10_KEY_LEGPAD_UP_STOP:
          RockFunctionEnable(false);
          st_Stretch.active = FALSE;SetStretchingEnable(0);
          st_Stretch.active = FALSE;
          bKeyLegPadUp = FALSE ;
          bKeyLegPadDown = FALSE ;
          bLegPadLinkage = FALSE ;
          bKeyBackPadUp = FALSE ;
          bKeyBackPadDown = FALSE ;
          bKeyFlexOut = FALSE ;
          bKeyFlexIn = FALSE ;
          break ;
          //----------------------------------------
        case H10_KEY_LEGPAD_DOWN_START:
RockFunctionEnable(false);
          st_Stretch.active = FALSE;SetStretchingEnable(0);
          st_Stretch.active = FALSE;
          bKeyLegPadUp = FALSE ;
          bKeyLegPadDown = TRUE ;
          bLegPadLinkage = FALSE ;
          bKeyBackPadUp = FALSE ;
          bKeyBackPadDown = FALSE ;
          bKeyFlexOut = FALSE ;
          bKeyFlexIn = FALSE ;bKeyFlexIn = FALSE ;bZeroflash = FALSE;
          break ;
        case H10_KEY_LEGPAD_DOWN_STOP:
RockFunctionEnable(false);
          st_Stretch.active = FALSE;SetStretchingEnable(0);
          st_Stretch.active = FALSE;
          bKeyLegPadUp = FALSE ;
          bKeyLegPadDown = FALSE ;
          bLegPadLinkage = FALSE ;
          bKeyBackPadUp = FALSE ;
          bKeyBackPadDown = FALSE ;
          bKeyFlexOut = FALSE ;
          bKeyFlexIn = FALSE ;
          break ;
          //--------------------------------
        case H10_KEY_WHEEL_SPEED_OFF://脚底滚轮速度为0
          bRollerEnable = FALSE;
          nBuzzerMode = BUZZER_MODE_ONETIME ;
          bSendBuzzerMode = TRUE ;
          nRollerPWM = 0;
          Valve_SetRollerPWM(nRollerPWM);
          break;
        case H10_KEY_WHEEL_SPEED_SLOW:
        case H10_KEY_WHEEL_SPEED_MED:
        case H10_KEY_WHEEL_SPEED_FAST:
          if(bRollerEnable != FALSE)
          {
            if(nKeyAirBagLocate == AIRBAG_LOCATE_AUTO) break; //在自动气囊程序中滚速度不可以调整
          }
          if(bRollerEnable == FALSE)
          {
            bRollerEnable = TRUE;nRoller3sCnt = 0;
          }
          if(key ==  H10_KEY_WHEEL_SPEED_SLOW)
          {
            nRollerPWM = 1;
          }
          if(key ==  H10_KEY_WHEEL_SPEED_MED)
          {
            nRollerPWM = 2;
          }
          if(key ==  H10_KEY_WHEEL_SPEED_FAST)
          {
            nRollerPWM = 3;
          }
          Valve_SetRollerPWM(nRollerPWM);
          if(nRollerPWM != 0)
          {
            nChairRunState = CHAIR_STATE_RUN ;
            if(Data_Get_Time() == 0)
            {
              Data_Set_Start(1, w_PresetTime);
            }
          }
          nBuzzerMode = BUZZER_MODE_ONETIME ;
          bSendBuzzerMode = TRUE ;
          break;
        case H10_KEY_HEAT:    //加热
          if(bKeyWaistHeat == FALSE)
              {
                bKeyWaistHeat = TRUE ;
              }
              else
              {
                bKeyWaistHeat = FALSE ;
              }
              nBuzzerMode = BUZZER_MODE_ONETIME ;
              bSendBuzzerMode = TRUE ;
          break;     
 //-------------------------------------------------------        
          

     /*      
         case H10_KEY_SEAT_VIBRATE_0:   //摇摆按键 摇摆关
              bKeySeatVibrate = FALSE ;
              nKeySeatVibrateStrength = 0 ;
              nBuzzerMode = BUZZER_MODE_ONETIME ;
              bSendBuzzerMode = TRUE ;
              break ;             
          case H10_KEY_SEAT_VIBRATE_1:    
           case H10_KEY_SEAT_VIBRATE_2:   
           case H10_KEY_SEAT_VIBRATE_3:    
                bKeySeatVibrate = TRUE ;
                if(key==H10_KEY_SEAT_VIBRATE_1) nKeySeatVibrateStrength = 1;   
                if(key==H10_KEY_SEAT_VIBRATE_2) nKeySeatVibrateStrength = 2;   
                if(key==H10_KEY_SEAT_VIBRATE_3) nKeySeatVibrateStrength = 3;      
           
                if(Data_Get_Time() == 0)
                {
                  Data_Set_Start(1,w_PresetTime); 
                }        
                nBuzzerMode = BUZZER_MODE_ONETIME ;
                bSendBuzzerMode = TRUE ;     
                break;        */      
     //     case H10_KEY_SEAT_VIBRATE_1: nKeySeatVibrateStrength = 1;   break;
          
      //    case H10_KEY_SEAT_VIBRATE_3: nKeySeatVibrateStrength = 3;   break;
          
 //-----------------------------------------------------------         
        default:       
          break;
        }
        
      //VoiceUart_Proce();     //delete by taqingosng
        
     if(Data_Get_Time() == 1 || bAutoProgramOver)  
       {
           Data_Set_Start(1,w_PresetTime); 
        /*  if((nBackMainRunMode == BACK_MAIN_MODE_AUTO) && (nBackSubRunMode == BACK_SUB_MODE_6MIN_DEMO))
          {
         nChairRunState = CHAIR_STATE_SETTLE;  //按摩时间到
         nSettleMode = POWER_KEY_RESET;   
          }
          else
          {
         nChairRunState = CHAIR_STATE_SETTLE;  //按摩时间到
         nSettleMode = RUN_OVER_RESET;              
          }
*/
         
         
         
       }
      //时间处理区
       if(Timer_Counter(C_TIMER_RUN + T_LOOP,1))//100ms timer couter
       {
         if(nAxisUpdateCounter<255) nAxisUpdateCounter++;
         nCurActionStepCounter++ ;//当前动作时间计数（行走或敲击时间计数）,行走和敲击马达共有一个时间计数器，当敲击计数开始时， 行走马达不可以计数，必须停止在当前位置
         nCurShoulderAdjustCounter++ ;
         nCurKnockRunStopCounter++ ;
         nCur3D_MotorStopCounter++;//3D电机在目标位置的停止时间适用于3D自动程序中
         //气囊程序运行时间计数器
         //st_AirBagBackWaist.nAirBagCounter++;
         st_AirBagLegFoot.nAirBagCounter++ ;

         st_AirBagSeat.nAirBagCounter++;//work
         //add by  taoqingsong
          st_AirBagArmNeck.nAirBagCounter++;//st_AirBagArmNeck         
          st_AirBagAuto.nAirBagCounter++;
         //ADD BY WGH 20170211 

          st_AirBagLegFoot_Seat.nAirBagCounter++;
          st_AirBagLegFoot_Arm.nAirBagCounter++;
          st_AirBagArm_Seat.nAirBagCounter++;
          st_AirBagLegFoot_Arm_Seat.nAirBagCounter++;
          st_AirBagAuto_Upbody.nAirBagCounter++;
          
          
          
          
         //--------------------------------
         if(st_Stretch.timer<255)st_Stretch.timer++;
         if(nStretchFlextimer<255)nStretchFlextimer++;
         n3DMotorRunCounter++;   //3D马达运行时间计数，软件暂无使用
         rocktimecount++;
         presstime++;
         BlueToothIntData_to_Cmd_Scan();//10ms scan   in main while
       }
       if(Timer_Counter(C_TIMER_500MS + T_LOOP,5))
       {
         bDisplayFlash = ~bDisplayFlash ;
         USB_MP3_SCAN_PROC();
       }
       
       
    //-----------------------------------------------------------------       
       if(bKeySeatVibrate)//坐垫摇摆处理
        {        
          if(Timer_Counter(C_TIMER_WAVE_START,30))//if(bKeySeatEnable) Timer_Counter_Clear(C_TIMER_WAVE_START);
          {
            bKeySeatEnable = TRUE;///if(bKeySeatVibrate) {bKeySeatVibrate=0;}Timer_Counter_Clear(C_TIMER_WAVE_START);
          }
        }
        else
        {
          Timer_Counter_Clear(C_TIMER_WAVE_START);
          bKeySeatEnable = FALSE;
        }     
 //-------------------------------------------------------------         
       
       
       
       Input_Proce();
       Valve_Send_Data();//气阀处理
       Main_Send();//上传按摩信息给手控器(包含Main_Send_Leg();)
       Main_BlueToothSend();//上传按摩信息给平板
       //Main_MassageSignalSend();
       LED_RGB_Proce(nChairRunState);
       main_GetKneadPosition();//获取揉捏电机的宽、中、窄位置
       Data_Time_Counter_Proce();
        
       Main_Walk_Beep_Proce();//行走电机BEEP声音
       
        //靠背升降电机手动处理
       Main_BackPad_Proce();//靠背电机行走,计数
       
       
        //小腿升降电机手动处理  ，计数
       Main_LegPad_Proce();
       
              //摇摆处理
       RockProcess();
        //小腿伸缩电机手动处理
         
 #ifdef RT8305T_1
       Main_FlexPad_Proce();//    Main_Send_Leg();
#endif
    //      Main_BackProce();//机芯处理 ,含有自动程序的数组
       
       //摇摆电机处理
       Main_VibrateMotorControl();
       if(Input_GetWaveMotorPosition() == REACH_WAVER_LIMIT)  
       {
          nWaveOverTime = 0;
       }           
       
       Main_Massage_Position_Proce();//work  零重力处理
       
#ifdef RT8305T_1   
       FlexMotorFollowingFood();
#endif     
       
       
       Main_Valve_Proce();//气阀处理
       Problem_Proce();
       MusicSampling();
       
       

       Main_ReStartShoulderCheck();
       
       
      switch(nBackMainRunMode)
      {
      case  BACK_MAIN_MODE_AUTO:  
        {
          if(bShoulderOK == 0)   // Auto_Calibration(0);  //进入主程序之前，先进行体型检测
          {
            Auto_Calibration(0);  //进入主程序之前，先进行体型检测
          }
          else
          {
                      
            
            Main_BackProce();//机芯处理 ,含有自动程序的数组
            //        n3D_MotorControlState=  _3D_MANUAL,OR PROGRAME  ,//手动、自动
            _3DMotorControl(n3D_MotorControlState,n3D_MotorControlPosition,n3D_MotorControlSpeed,n3D_MotorControlStopTime);   
            WalkMotorControl(nWalkMotorControlParam1,nWalkMotorControlParam2) ;//行走电机坐标或在当前位置停止时间
            KneadMotorControl(nKneadMotorControlParam1,nKneadMotorControlParam2) ;//揉捏电机运行状态，和运行圈数
            KnockMotorControl(nKnockMotorControlParam1,nKnockMotorControlParam2,nKnockMotorControlParam3) ;//敲打电机状态，持续时间，停顿时间
          }
        }
        break;
      case  BACK_MAIN_MODE_3D:  
        {
          if(bShoulderOK == 0)    
          {
            Auto_Calibration(0);  //进入主程序之前，先进行体型检测
          }
          else
          {
            
                                         
            Main_BackProce();
            _3DMotorControl(n3D_MotorControlState,n3D_MotorControlPosition,n3D_MotorControlSpeed,n3D_MotorControlStopTime);   
            WalkMotorControl(nWalkMotorControlParam1,nWalkMotorControlParam2) ;
            KneadMotorControl(nKneadMotorControlParam1,nKneadMotorControlParam2) ;
            KnockMotorControl(nKnockMotorControlParam1,nKnockMotorControlParam2,nKnockMotorControlParam3) ;
          }
        }
        break;  
      case  BACK_MAIN_MODE_MANUAL:  
        {
          //ShoulderSteps = BODY_DETECT_PREPARE;
          ShoulderSteps = BODY_DETECT_OVER;
          Main_BackProce();
          _3DMotorControl(n3D_MotorControlState,n3D_MotorControlPosition,n3D_MotorControlSpeed,n3D_MotorControlStopTime);   
          WalkMotorControl(nWalkMotorControlParam1,nWalkMotorControlParam2) ;
          KneadMotorControl(nKneadMotorControlParam1,nKneadMotorControlParam2) ;
          KnockMotorControl(nKnockMotorControlParam1,nKnockMotorControlParam2,nKnockMotorControlParam3) ;
          
        }
        break;    
      }
        
       if((nBackMainRunMode == BACK_MAIN_MODE_IDLE) &&
            (nKeyAirBagLocate == AIRBAG_LOCATE_NONE) &&
            (bKeyWaistHeat == FALSE) &&
            (bRollerEnable == FALSE)  &&
              (bKeySeatVibrate == FALSE)&&
                (bRockEnable == false)     )//
        {
         nChairRunState = CHAIR_STATE_WAIT_COMMAND ;
        }
      
   /*     if((nChairRunState == CHAIR_STATE_RUN) &&
     (nBackMainRunMode == BACK_MAIN_MODE_IDLE) &&
       (nKeyAirBagLocate == AIRBAG_LOCATE_NONE) &&
         (bKeyWaistHeat == FALSE) &&
           (bKeySeatVibrate == FALSE) &&
             bRollerEnable == FALSE)
  {
    nChairRunState = CHAIR_STATE_WAIT_COMMAND ;
    nChairStateCount = 0 ;
    nIndicateTimer = WAIT_INDICATE_TIME;
    Data_Set_Start(0,0);
    // Power_On();
  }*/
       if(Timer_Counter(C_TIMER_INDICATE + T_LOOP,RUN_INDICATE_TIME))
        {
          
          IndicateLED_Toggle();
        }   
      
      
         //加热处理
        if(bKeyWaistHeat == TRUE)
        {
            WaistHeat_On();
        }
        else
        {
            WaistHeat_Off();
        }
        
        if(bOzonEnable == TRUE)
        {
            Valve_OzonOn();
        }
        else
        {
            Valve_OzonOff();
        }
        
       if((bKeyBackPadUp == FALSE) && (bKeyBackPadDown == FALSE) &&
                (bKeyLegPadUp == FALSE) && (bKeyLegPadDown == FALSE) &&
                (bKeyWalkUp == FALSE) && (bKeyWalkDown == FALSE) &&
                 (bKeyFlexOut == FALSE) && (bKeyFlexIn == FALSE))  
        {
            if((nBuzzerMode == BUZZER_MODE_FAST) ||
               (nBuzzerMode == BUZZER_MODE_SLOW))
            {
                {
                    nBuzzerMode = BUZZER_MODE_OFF ;
                    bSendBuzzerMode = TRUE ;
                }
            }
        } 
       
       
    } //end while
   /***************程序退出区**************************/
    st_Stretch.init = false;
    bKeyBackPadUp = false;
    bKeyBackPadDown = false;
    bKeyLegPadUp = false;
    bKeyLegPadDown = false;
    bKeyFlexOut = false;
    bKeyFlexIn = false;
    st_Stretch.active = false; 
    bKeyWalkUp = false; 
    bKeyWalkDown = false; 
}
void Main_Idle(void)
{
    BYTE key;
    //变量初始化区域
    //函数初始化区域
    Power_All_On();
    Timer_Counter_Clear(C_TIMER_INDICATE);
    Timer_Counter_Clear(C_TIMER_TEMP);
    //主循环
    while(CHAIR_STATE_IDLE == nChairRunState)
    {
      //按键处理区
        key = Main_GetKey();
        key &= 0x7f;
        if(H10_KEY_POWER_SWITCH == key)
        {
          nChairRunState = CHAIR_STATE_WAIT_COMMAND; //按了电源键后
        }
        if(HandUart_GetCtrlType() == ENGGER_CTRL)
        {
            nChairRunState = CHAIR_STATE_ENGINEERING;
            return;
        }
        
          if(BlueToothUart_GetCtrlType()==PROGARM_CTRL)
         {
           
            nChairRunState = CHAIR_STATE_UPDATE;
            return;     
                   
         }
        
        
        if(HandUart_GetCtrlType() == PROGARM_CTRL )
        {

          
            nChairRunState = CHAIR_STATE_UPDATE;
            return;
        }          
        
        
        
      //时间处理区
       if(Timer_Counter(C_TIMER_INDICATE + T_LOOP,CHAIR_STATE_IDLE))
        {
          IndicateLED_Toggle();
        }
        if(Timer_Counter(C_TIMER_TEMP,100))
        {
          nChairRunState = CHAIR_STATE_SLEEP; 
        }
       Input_Proce();
       Valve_Send_Data();
       Main_Send();
       Main_BlueToothSend();
 //      Main_Send_Leg();
       //Main_MassageSignalSend();
    } //end while
   /***************程序退出区**************************/
}

void Main_Demo(void)
{
    int demoStep = 0;
    BYTE key;
    Power_All_On();
    Timer_Counter_Clear(C_TIMER_INDICATE);
    Timer_Counter_Clear(C_TIMER_TEMP);
    Timer_Counter_Clear(C_TIMER_500MS);
    Data_Set_Start(1, w_PresetTime);
    bEnableStretchDemoRun = FALSE;//WGH 20161107
    nBackMainRunMode = BACK_MAIN_MODE_DEMO;
    nBackSubRunMode = BACK_SUB_MODE_DEMO;
    Main_Start_Auto();
    //主循环
    while(CHAIR_STATE_DEMO == nChairRunState)
    {
        //按键处理区
        key = Main_GetKey();//demo
        switch(key)
        {
            case H10_KEY_MENU:
              break;
            case H10_KEY_POWER_SWITCH: 
              {
                nChairRunState = CHAIR_STATE_SETTLE ;
                nSettleMode = POWER_KEY_RESET;                 
              }
              nBuzzerMode = BUZZER_MODE_ONETIME ;
              bSendBuzzerMode = TRUE ;
              break ;
              
            case H10_KEY_ZERO_START://demo
             /* if(isZeroPosition())
              {
                nTargetMassagePosition = MASSAGE_INIT_POSITION;
              }
              else
              {
                nTargetMassagePosition = MASSAGE_OPTIMAL2_POSITION;bZeroflash = TRUE;
              }*/
              
          w_ZeroPosition++;
          w_ZeroPosition %= 3;  //0位置为初始位置   1位置为第一个零重力点    2位置为第二个零重力点    
          
          if(w_ZeroPosition == 0)
          {
            nTargetMassagePosition = MASSAGE_INIT_POSITION;
          }
          if(w_ZeroPosition == 1)
          {
            nTargetMassagePosition = MASSAGE_OPTIMAL_POSITION;//MASSAGE_OPTIMAL_POSITION;
          }
          if(w_ZeroPosition == 2)
          {
            nTargetMassagePosition = MASSAGE_OPTIMAL2_POSITION;
          }
          
              
              bMassagePositionUpdate = TRUE;
              nBuzzerMode = BUZZER_MODE_ONETIME ;
              bSendBuzzerMode = TRUE ;
              break;
            case H10_KEY_WIDTH_INCREASE:
            case H10_KEY_WIDTH_DECREASE:
            case H10_KEY_WIDTH_MIN:  
            case H10_KEY_WIDTH_MED:  
            case H10_KEY_WIDTH_MAX: 
              break ;
              
            case H10_KEY_BACKPAD_UP_START:
              RockFunctionEnable(false);
          st_Stretch.active = FALSE;SetStretchingEnable(0);
              bKeyBackPadUp = TRUE ;
              bKeyBackPadDown = FALSE ;
              bKeyLegPadDown = TRUE ;
              bKeyLegPadUp = FALSE ;
              bLegPadLinkage = TRUE ;
              bKeyFlexOut = FALSE ;
              bKeyFlexIn = FALSE ;bKeyFlexIn = FALSE ;bZeroflash = FALSE; 
              break ;
            case H10_KEY_BACKPAD_UP_STOP:
              RockFunctionEnable(false);
          st_Stretch.active = FALSE;SetStretchingEnable(0);
              bKeyBackPadUp = FALSE ;
              bKeyBackPadDown = FALSE ;
              bKeyLegPadDown = FALSE ;
              bKeyLegPadUp = FALSE ;
              bLegPadLinkage = TRUE ;
              bKeyFlexOut = FALSE ;
              bKeyFlexIn = FALSE ;
              break ;
            case H10_KEY_BACKPAD_DOWN_START:
              bKeyBackPadUp = FALSE ;
              bKeyBackPadDown = TRUE ;
              bKeyLegPadDown = FALSE ;
              bKeyLegPadUp = TRUE ;
              bLegPadLinkage = TRUE ;
              bKeyFlexOut = FALSE ;
              bKeyFlexIn = FALSE ;bKeyFlexIn = FALSE ;bZeroflash = FALSE;
              break ;
            case H10_KEY_BACKPAD_DOWN_STOP:
              bKeyBackPadUp = FALSE ;
              bKeyBackPadDown = FALSE ;
              bKeyLegPadDown = FALSE ;
              bKeyLegPadUp = FALSE ;
              bLegPadLinkage = TRUE ;
              bKeyFlexOut = FALSE ;
              bKeyFlexIn = FALSE ;
              break ;
            case H10_KEY_LEGPAD_EXTEND_START:
RockFunctionEnable(false);
          st_Stretch.active = FALSE;SetStretchingEnable(0);
              bKeyLegPadUp = FALSE ;
              bKeyLegPadDown = FALSE ;
              bLegPadLinkage = FALSE ;
              bKeyBackPadUp = FALSE ;
              bKeyBackPadDown = FALSE ;
              bKeyFlexOut = TRUE ;
              bKeyFlexIn = FALSE ;
              break;
            case H10_KEY_LEGPAD_EXTEND_STOP:
            case H10_KEY_LEGPAD_CONTRACT_STOP:
RockFunctionEnable(false);
          st_Stretch.active = FALSE;SetStretchingEnable(0);
              bKeyLegPadUp = FALSE ;
              bKeyLegPadDown = FALSE ;
              bLegPadLinkage = FALSE ;
              bKeyBackPadUp = FALSE ;
              bKeyBackPadDown = FALSE ;
              bKeyFlexOut = FALSE;
              bKeyFlexIn = FALSE ;
              break;
            case H10_KEY_LEGPAD_CONTRACT_START:
RockFunctionEnable(false);
          st_Stretch.active = FALSE;SetStretchingEnable(0);
              bKeyLegPadUp = FALSE ;
              bKeyLegPadDown = FALSE ;
              bLegPadLinkage = FALSE ;
              bKeyBackPadUp = FALSE ;
              bKeyBackPadDown = FALSE ;
              bKeyFlexOut = FALSE;
              bKeyFlexIn = TRUE ;
              break;
            case H10_KEY_LEGPAD_UP_START:
              RockFunctionEnable(false);
          st_Stretch.active = FALSE;SetStretchingEnable(0);
          bKeyFlexIn = FALSE ;bZeroflash = FALSE;
              bKeyLegPadUp = TRUE ;
              bKeyLegPadDown = FALSE ;
              bLegPadLinkage = FALSE ;
              bKeyBackPadUp = FALSE ;
              bKeyBackPadDown = FALSE ;
              bKeyFlexOut = FALSE ;
              bKeyFlexIn = FALSE ;
              break ;
            case H10_KEY_LEGPAD_UP_STOP:
              RockFunctionEnable(false);
          st_Stretch.active = FALSE;SetStretchingEnable(0);
              bKeyLegPadUp = FALSE ;
              bKeyLegPadDown = FALSE ;
              bLegPadLinkage = FALSE ;
              bKeyBackPadUp = FALSE ;
              bKeyBackPadDown = FALSE ;
              bKeyFlexOut = FALSE ;
              bKeyFlexIn = FALSE ;
              break ;
            case H10_KEY_LEGPAD_DOWN_START:
              RockFunctionEnable(false);
          st_Stretch.active = FALSE;SetStretchingEnable(0);
          bKeyFlexIn = FALSE ;bZeroflash = FALSE;
              bKeyLegPadUp = FALSE ;
              bKeyLegPadDown = TRUE ;
              bLegPadLinkage = FALSE ;
              bKeyBackPadUp = FALSE ;
              bKeyBackPadDown = FALSE ;
              bKeyFlexOut = FALSE ;
              bKeyFlexIn = FALSE ;
              break ;
            case H10_KEY_LEGPAD_DOWN_STOP:
              RockFunctionEnable(false);
              st_Stretch.active = FALSE;SetStretchingEnable(0);              
              bKeyLegPadUp = FALSE ;
              bKeyLegPadDown = FALSE ;
              bLegPadLinkage = FALSE ;
              bKeyBackPadUp = FALSE ;
              bKeyBackPadDown = FALSE ;
              bKeyFlexOut = FALSE ;
              bKeyFlexIn = FALSE ;
              break ;
              
            default:       
              break;
            }
     if((!bMassagePositionUpdate) && (!bKeyFlexIn) && (!bKeyFlexOut))
     {
       switch(demoStep)
       {
       default: 
             demoStep = 0;
        case 0: 
          if(Flex_ControlIn(FLEX_MOTOR_CURRENT_1D5A))
          {
            demoStep++;
            Timer_Counter_Clear(C_TIMER_TEMP);
          }
          break;
       case 1:
         {
           if(Timer_Counter(C_TIMER_TEMP,15))
           {
             demoStep++;
           }
         }
         break;
       case 2:
         if(Flex_ControlOut(FLEX_MOTOR_CURRENT_2A))
          {
            demoStep++;
            Timer_Counter_Clear(C_TIMER_TEMP);
          }
         break;
       case 3:
         {
           if(Timer_Counter(C_TIMER_TEMP,15))
           {
             demoStep++;
           }
         }
         break;  
       }
     }
     if(Data_Get_Time() == 0) 
       {
         Data_Set_Start(1, w_PresetTime);
       }
      //时间处理区
       if(Timer_Counter(C_TIMER_RUN + T_LOOP,1))
       {
         nCurActionStepCounter++ ;
         nCurShoulderAdjustCounter++ ;
         nCurKnockRunStopCounter++ ;
         nCur3D_MotorStopCounter++;
         //气囊记数器
         //st_AirBagBackWaist.nAirBagCounter++;
         st_AirBagLegFoot.nAirBagCounter++ ;
         st_AirBagSeat.nAirBagCounter++;
         
             //add by  taoqingsong

          st_AirBagArmNeck.nAirBagCounter++;//st_AirBagArmNeck         
          st_AirBagAuto.nAirBagCounter++;     
         //ADD BY WGH 20170211 

          st_AirBagLegFoot_Seat.nAirBagCounter++;
          st_AirBagLegFoot_Arm.nAirBagCounter++;
          st_AirBagArm_Seat.nAirBagCounter++;
          st_AirBagLegFoot_Arm_Seat.nAirBagCounter++;
          st_AirBagAuto_Upbody.nAirBagCounter++;
          
          
         if(st_Stretch.timer<255)st_Stretch.timer++;
         if(nStretchFlextimer<255)nStretchFlextimer++;
         n3DMotorRunCounter++;
         
         
         
         
       }
       if(Timer_Counter(C_TIMER_500MS + T_LOOP,5))
       {
         bDisplayFlash = ~bDisplayFlash ;
       }
       Input_Proce();
       Valve_Send_Data();
       Main_Send();
       Main_BlueToothSend();
       //Main_MassageSignalSend();
       LED_RGB_Proce(nChairRunState);
       main_GetKneadPosition();
       Data_Time_Counter_Proce();
        
       Main_Walk_Beep_Proce();
        //靠背升降电机手动处理
       Main_BackPad_Proce();
        //小腿升降电机手动处理
       Main_LegPad_Proce();
        //小腿伸缩电机手动处理
 #ifdef  RT8305T_1   
      Main_FlexPad_Proce();
 #endif    
       
       Main_Massage_Position_Proce();//demo
       
       
 #ifdef  RT8305T_1     
       FlexMotorFollowingFood();//demo
 #endif     
       Problem_Proce();
       
      switch(nBackMainRunMode)
      {
      case  BACK_MAIN_MODE_AUTO:  
        break;
      case  BACK_MAIN_MODE_DEMO:  
        {
            Main_BackProce();
            _3DMotorControl(n3D_MotorControlState,n3D_MotorControlPosition,n3D_MotorControlSpeed,n3D_MotorControlStopTime);   
            WalkMotorControl(nWalkMotorControlParam1,nWalkMotorControlParam2) ;
            KneadMotorControl(nKneadMotorControlParam1,nKneadMotorControlParam2) ;
            KnockMotorControl(nKnockMotorControlParam1,nKnockMotorControlParam2,nKnockMotorControlParam3) ;
        }
        break;  
      case  BACK_MAIN_MODE_MANUAL:  
        break;    
      }
     
        Main_Valve_Proce();
  /*      if((nBackMainRunMode == BACK_MAIN_MODE_IDLE) &&
            (nKeyAirBagLocate == AIRBAG_LOCATE_NONE) &&
            (bKeyWaistHeat == FALSE) &&
            (bRollerEnable == FALSE)  &&
              (bOzonEnable == FALSE))
        {
         nChairRunState = CHAIR_STATE_WAIT_COMMAND ;
        }*/
         //加热处理
        if(bKeyWaistHeat == TRUE)
        {
            WaistHeat_On();
        }
        else
        {
            WaistHeat_Off();
        }
        
        if(bOzonEnable == TRUE)
        {
            Valve_OzonOn();
        }
        else
        {
            Valve_OzonOff();
        }
        
       if((bKeyBackPadUp == FALSE) && (bKeyBackPadDown == FALSE) &&
                (bKeyLegPadUp == FALSE) && (bKeyLegPadDown == FALSE) &&
                (bKeyWalkUp == FALSE) && (bKeyWalkDown == FALSE) &&
                 (bKeyFlexOut == FALSE) && (bKeyFlexIn == FALSE))  
        {
            if((nBuzzerMode == BUZZER_MODE_FAST) ||
               (nBuzzerMode == BUZZER_MODE_SLOW))
            {
                {
                    nBuzzerMode = BUZZER_MODE_OFF ;
                    bSendBuzzerMode = TRUE ;
                }
            }
        } 
       
       
    } //end while
   /***************程序退出区**************************/
}







/*******************************************************
按摩椅初始化程序： 3D 马达复位

********************************************************/
void Main_Initial(void)
{
    bool bMassageSignalOK = false;
//    bool bLegSignalOK = false;
    bool b3DMotorInit = false;
   // unsigned short _3D_Current = 0,adcAxisCurrent;
   // BYTE key;
    //变量初始化区域
    //函数初始化区域
    Power_All_On();
    Timer_Counter_Clear(C_TIMER_INDICATE);
    Timer_Counter_Clear(C_TIMER_TEMP);
    //主循环
    while(CHAIR_STATE_INITIAL == nChairRunState)
    {
      //按键处理区
      if(HandUart_GetCtrlType() == ENGGER_CTRL)
      {
        nChairRunState = CHAIR_STATE_ENGINEERING;
        AxisMotor_Control(STATE_AXIS_IDLE,0,_3D_SPEED_10);
        return;
      }
       if(BlueToothUart_GetCtrlType()==PROGARM_CTRL)
         {
           
            nChairRunState = CHAIR_STATE_UPDATE;
            return;     
                   
         }
        
        
      if(HandUart_GetCtrlType() == PROGARM_CTRL )
        {

          
            nChairRunState = CHAIR_STATE_UPDATE;
            return;
        }        
      
      
      
      
      bMassageSignalOK = LEUART0_isOK();//初始化检测3D机芯数据线与主板是否相连
      if(bMassageSignalOK) 
      {
        //if(!b3DMotorInit)
        //{
         // if(AxisMotor_Control(STATE_RUN_AXIS_FORWARD,0,_3D_SPEED_7))
         // {
            b3DMotorInit = true;  
          //}
       // }
       // else
        //{
          AxisMotor_Control(STATE_AXIS_IDLE,0,_3D_SPEED_10);
        //}
      }
      else
      {
        AxisMotor_Control(STATE_AXIS_IDLE,0,_3D_SPEED_10);
      } 
      //时间处理区
      if(Timer_Counter(C_TIMER_INDICATE + T_LOOP,CHAIR_STATE_IDLE))
      {
        IndicateLED_Toggle();
      }
      Input_Proce();
      Valve_Send_Data();
      Main_Send();
      Main_BlueToothSend();
      Problem_Proce();
      
      
      if(Problem_Get3DFault())
      {
        AxisMotor_Control(STATE_AXIS_IDLE,0,_3D_SPEED_10); 
        return;
      }
      //if(b3DMotorInit)
      //{
        nChairRunState = CHAIR_STATE_IDLE;
      //}
    } //end while
   /***************程序退出区**************************/
   AxisMotor_Control(STATE_AXIS_IDLE,0,_3D_SPEED_10);
}

void Main_Problem(void)
{
    BYTE key;
    //变量初始化区域
    //函数初始化区域
    Power_All_On();
    Timer_Counter_Clear(C_TIMER_INDICATE);
    Timer_Counter_Clear(C_TIMER_TEMP);
    //主循环
    while(CHAIR_STATE_PROBLEM == nChairRunState)
    {
      //按键处理区
        key = Main_GetKey();//problem
        if(H10_KEY_POWER_SWITCH == key)
        {
          nChairRunState = CHAIR_STATE_WAIT_COMMAND; //按了电源键后
        }
      //时间处理区
       if(Timer_Counter(C_TIMER_INDICATE + T_LOOP,CHAIR_STATE_IDLE))
        {
          IndicateLED_Toggle();
        }
       Input_Proce();
       Valve_Send_Data();
       Main_Send();
       Main_BlueToothSend();
       //Main_MassageSignalSend();
    } //end while
   /***************程序退出区**************************/
}

void Main_MassageSignalTest(void)
{
  int indicateTime;
  Timer_Counter_Clear(C_TIMER_INDICATE);
  while(1)
  {
    //Main_MassageSignalSend();
    
    if(LEUART0_isOK())
    {
      indicateTime = 10;
    }
    else
    {
      indicateTime = 2;
    } 
    if(Timer_Counter(C_TIMER_INDICATE + T_LOOP,indicateTime))
    {
      IndicateLED_Toggle();
    }     
  }
}


bool RockBackLegProcess(void)
{
unsigned int w_LegPosition;
unsigned int r_BackLocation;
  
  bool bBackpositiondone,bLegpositiondone;

  if(Input_GetBackDownSwitch() == REACH_BACK_LIMIT) BackMotor_Set_Location(BACK_MOTOR_MAX_LOCATION);
  if(Input_GetBackUpSwitch() == REACH_BACK_LIMIT) BackMotor_Set_Location(0);
  w_LegPosition = LegMotor_Get_Position();
  r_BackLocation= BackMotor_Get_Location();
 if( bRockEnable == FALSE) return 1;
  ///////////////////////////////////////////////////////////////////////////////
  if(r_BackLocation <= MASSAGE_BACK_ROCK_LOCATION)
  {
    bKeyBackPadUp = FALSE;
    bKeyBackPadDown = TRUE;
    bBackpositiondone = false;
  }
  else if(r_BackLocation >= (MASSAGE_BACK_ROCK_LOCATION + 20))
  {
    bKeyBackPadUp = TRUE;
    bKeyBackPadDown = FALSE;
    bBackpositiondone = false;
  }
  else 
  {
    bKeyBackPadUp = FALSE;
    bKeyBackPadDown = FALSE;
    bBackpositiondone = true;
  }
  //////////////////////////////////////////////////////////////////////////////
  if(w_LegPosition <= MASSAGE_LEG_ROCK_POSITION)
  {
    bKeyLegPadUp = TRUE;
    bKeyLegPadDown = FALSE;
    bLegpositiondone = false;
  }
  else if(w_LegPosition >= MASSAGE_LEG_ROCK_POSITION + 20)
  {
    bKeyLegPadUp = FALSE;
    bKeyLegPadDown = TRUE;
    bLegpositiondone = false;
  }
  else
  {
    bKeyLegPadUp = FALSE;
    bKeyLegPadDown = FALSE;
    bLegpositiondone = true;
  }
  if((bLegpositiondone == true)&&(bBackpositiondone == true)) return true;  //到达了预定的位置
  else return false;  //未到达预定的位置

}
void RockFunctionEnable(bool Enable)
{

  bKeyBackPadUp = FALSE;
  bKeyBackPadDown = FALSE;
  bKeyLegPadUp = FALSE;
  bKeyLegPadDown = FALSE;
                  //bLegPadLinkage = FALSE ;
  switch(Enable)
  {
  case RockDisable:
    bRockEnable = false;
    SetRockingEnable(Enable);
    break;
  case RockEnable:
                    bLegPadLinkage = TRUE ;
    SetRockingEnable(Enable);
    nRockModeEnterEnable = EnterRock;
    bRockEnable = true;
    WorkStep = 0;//一旦刚进入该模式就开始尝试下躺
    break;
  default :
    bRockEnable = false;
    break;
  }

}
unsigned int ptrwgh;
void RockProcess(void)
{
unsigned int rock_BackLocation;  
  int leg_Flag,Back_Flag;//,SlideFlag;
  //轻松模式下
  //if(nBackMainRunMode == BACK_MAIN_MODE_SLEEP &&
   if((nBackMainRunMode == BACK_MAIN_MODE_AUTO) &&
     (nBackSubRunMode == BACK_SUB_MODE_AUTO_2))
  {
    unsigned int CurTime = Data_Get_TimeSecond();
    if((CurTime == 28 * 60)||
       (CurTime == 27 * 60)||
       (CurTime == 23 * 60)||
       //(CurTime == 20 * 60)||
       (CurTime == 18 * 60)||
       (CurTime == 13 * 60)||
       (CurTime == 10 * 60)||
       (CurTime == 4 * 60))
    {
      RockFunctionEnable(true);
      nRockMode = ROCK_AUTO;
    }
    
   if(bShoulderOK == 1) 
    {
      if((CurTime == 30 * 60)||(CurTime == 20 * 60)||(CurTime == 10 * 60))
      {
        RockFunctionEnable(true);
        nRockMode = ROCK_AUTO;
      }
    }
    //else
    //{
   //   RockFunctionEnable(false);
   // }
    
    if((CurTime == 25 * 60)||
       (CurTime == 16 * 60) ||
       (CurTime == 7 * 60)||
       (CurTime == 1 * 60))
    {
      RockFunctionEnable(false);
      //回到第一零重力状态
      nTargetMassagePosition = MASSAGE_OPTIMAL2_POSITION;
      bMassagePositionUpdate = TRUE;
      //w_ZeroPosition = 1;
    }
  }
  //没有在摇摆模式时就直接退出
  if(nRockModeEnterEnable == ExitRock) return;
  


  
  if(Input_GetBackDownSwitch() == REACH_BACK_LIMIT) BackMotor_Set_Location(BACK_MOTOR_MAX_LOCATION);
  if(Input_GetBackUpSwitch() == REACH_BACK_LIMIT) BackMotor_Set_Location(0);
  rock_BackLocation = BackMotor_Get_Location();  
  if(bRockEnable)
  {
    
    FlexMotorSetDisable();
    switch(WorkStep)
    {
    case StartRock:     
      if(RockBackLegProcess() == true)
      {
        WorkStep = LieDownStop;//++;     
      }
      ptrwgh =0;
      break;
    case LieDown:
      //CurrentBackMotorPosition = BackMotor_Get_Position();
      if(Input_GetBackDownSwitch() == REACH_BACK_LIMIT)
      {
          bKeyBackPadUp = FALSE;
          bKeyBackPadDown = FALSE;
      }
      else
      {
        if(rock_BackLocation < MASSAGE_BACK_OPTIMAL1_LOCATION)
        {
          bKeyBackPadUp = FALSE;
          bKeyBackPadDown = TRUE;
        }
        else
        {
          bKeyBackPadUp = FALSE;
          bKeyBackPadDown = FALSE;
        }         
      }
      if(RockBackLegProcess() == true)
      {
        WorkStep++;
      }
      ptrwgh =1;rocktimecount = 0;
      break;
    case LieDownStop:
      Back_Flag = BackMotor_Control(STATE_BACK_IDLE) ;
      leg_Flag = LegMotor_Control(STATE_LEG_IDLE) ;
      if( (Back_Flag)&&(leg_Flag) )
      {
        if(rocktimecount >= 10)WorkStep ++;
      }
      ptrwgh =2;
      break; 
    case LieUP:
      //CurrentBackMotorPosition = BackMotor_Get_Position();
      if(Input_GetBackUpSwitch() == REACH_BACK_LIMIT)
      {
        bKeyBackPadUp = FALSE;
        bKeyBackPadDown = FALSE;
        WorkStep ++;// = LieDown;
      }
      else
      {
        if(rock_BackLocation > MASSAGE_BACK_OPTIMAL_LOCATION)
        {
          bKeyBackPadUp = TRUE;
          bKeyBackPadDown = FALSE;
        }
        else
        {
          bKeyBackPadUp = FALSE;
          bKeyBackPadDown = FALSE;
          WorkStep ++;//= LieDown;
        }       
      }
      ptrwgh =3;rocktimecount = 0;
      break;
    case LieUpStop:
      Back_Flag = BackMotor_Control(STATE_BACK_IDLE) ;
      leg_Flag = LegMotor_Control(STATE_LEG_IDLE) ;
      if( (Back_Flag)&&(leg_Flag) )
      {
        if(rocktimecount >= 10)WorkStep = LieDown;
      }
      ptrwgh =4;
      break; 
      
    default :
      break;
    }
  }
  else
  {
    nRockModeEnterEnable = ExitRock;
  }
}
void Main_Auto_Program_Test(void)
{
  for(int j=0;j<6;j++) 
  { 
    for(int i=0;i< BACK_AUTO_STEPS[j];i++)
    {
      switch(j)
      {
      case 0: AutoDirector = AutoFunction0[i] ; break;
      case 1: AutoDirector = AutoFunction1[i] ; break;
      case 2: AutoDirector = AutoFunction2[i] ; break;
      case 3: AutoDirector = AutoFunction3[i] ; break;
      case 4: AutoDirector = AutoFunction4[i] ; break;
      case 5: AutoDirector = AutoFunction5[i] ; break;
      }
      
      switch(AutoDirector.nSubFunction)
      {
      case BACK_SUB_MODE_KNEAD:	
        {
          if((AutoDirector.nKneadMotorState == KNEAD_STOP)||(AutoDirector.nKnockMotorState != KNOCK_STOP))
          {
            printf("auto%d-KNEAD-step:[%d]\n",j,i);
          }
        }
        break;
        
      case BACK_SUB_MODE_KNOCK:			
        {
          if((AutoDirector.nKnockMotorState == KNOCK_STOP)||((AutoDirector.nKneadMotorState != KNEAD_STOP)
                                                             &&(AutoDirector.nKneadMotorState != KNEAD_STOP_AT_MIN)
                                                               &&(AutoDirector.nKneadMotorState != KNEAD_STOP_AT_MED)
                                                                 &&(AutoDirector.nKneadMotorState != KNEAD_STOP_AT_MAX)))
          {
            printf("auto%d-KNOCK-step:[%d]\n",j,i);
          }
        }
        break;
        
      case BACK_SUB_MODE_WAVELET:		        
        {
          if(AutoDirector.nKnockMotorState == KNOCK_STOP ||
             AutoDirector.nKneadMotorState == KNEAD_STOP )
          {
            printf("auto%d-WAVELET-step:[%d]\n",j,i);
          }
        }
        break;
      case BACK_SUB_MODE_SOFT_KNOCK:	
        {
          if((AutoDirector.nKnockMotorState != KNOCK_RUN_STOP)||((AutoDirector.nKneadMotorState != KNEAD_STOP)
                                                                 &&(AutoDirector.nKneadMotorState != KNEAD_STOP_AT_MIN)
                                                                   &&(AutoDirector.nKneadMotorState != KNEAD_STOP_AT_MED)
                                                                     &&(AutoDirector.nKneadMotorState != KNEAD_STOP_AT_MAX)))
          {
            printf("auto%d-SOFT_KNOCK-step:[%d]\n",j,i);
          }
        }
        break;
        
      case BACK_SUB_MODE_PRESS:			
        {
          if(AutoDirector.nKnockMotorState != KNOCK_STOP ||((AutoDirector.nKneadMotorState != KNEAD_STOP)
                                                            &&(AutoDirector.nKneadMotorState != KNEAD_STOP_AT_MIN)
                                                              &&(AutoDirector.nKneadMotorState != KNEAD_STOP_AT_MED)
                                                                &&(AutoDirector.nKneadMotorState != KNEAD_STOP_AT_MAX)))
          {
            printf("auto%d-PRESS-step:[%d]\n",j,i);
          }
        }
        break;
         case BACK_SUB_MODE_RUBBING:
        {
        if(AutoDirector.nKneadMotorState != KNEAD_RUN_RUBBING)
        {
        printf("auto%d-RUBBING-step:[%d]\n",j,i);
      }
      }
        break ;  
      case BACK_SUB_MODE_MUSIC:			
      default: 
        //printf("error[%d]\n",j,0);
        break;
      }
    }
  }
  while(1);
}

//--------------------------------------------------------------------------

void CloudProgrameInit(void)
{

  unsigned int *p;
  //处理网络数据0
  p = (unsigned int *)CLUDE_AUTO_0_BASE;

  if(*p == 0 || *p == 0xffffffff) 
  {
    pCludeAutoFunction_0 = NULL;

  }
  else
  {
    pCludeAutoFunction_0 = (WALK_KNEAD_KNOCK_MOTOR_STRUCT_AUTO *)(CLUDE_AUTO_0_BASE + CLUDE_AUTO_PROGRAM_OFFSET);       
    BACK_CLOUD_STEPS[0] =(*(unsigned int *)(CLUDE_AUTO_0_BASE+CLUDE_AUTO_SIZE_ADDRESS))/16;
  }
  //处理网络数据1
  p = (unsigned int *)CLUDE_AUTO_1_BASE;
  if(*p == 0 || *p == 0xffffffff) 
  {
    pCludeAutoFunction_1 = NULL;
  }
  else
  {
    pCludeAutoFunction_1 = (WALK_KNEAD_KNOCK_MOTOR_STRUCT_AUTO *)(CLUDE_AUTO_0_BASE + CLUDE_AUTO_PROGRAM_OFFSET);
       BACK_CLOUD_STEPS[1] =(*(unsigned int *)(CLUDE_AUTO_1_BASE+CLUDE_AUTO_SIZE_ADDRESS))/16;
    
  }
  
  //处理网络数据2
  p = (unsigned int *)CLUDE_AUTO_2_BASE;
  if(*p == 0 || *p == 0xffffffff) 
  {
    pCludeAutoFunction_2 = NULL;
  }
  else
  {
    pCludeAutoFunction_2 = (WALK_KNEAD_KNOCK_MOTOR_STRUCT_AUTO *)(CLUDE_AUTO_2_BASE + CLUDE_AUTO_PROGRAM_OFFSET);
    
      BACK_CLOUD_STEPS[2] =(*(unsigned int *)(CLUDE_AUTO_2_BASE+CLUDE_AUTO_SIZE_ADDRESS))/16;
    
    
  }
  
  //处理网络数据2
  p = (unsigned int *)CLUDE_AUTO_3_BASE;
  if(*p == 0 || *p == 0xffffffff) 
  {
    pCludeAutoFunction_3 = NULL;
  }
  else
  {
    pCludeAutoFunction_3 = (WALK_KNEAD_KNOCK_MOTOR_STRUCT_AUTO *)(CLUDE_AUTO_3_BASE + CLUDE_AUTO_PROGRAM_OFFSET);
    
       BACK_CLOUD_STEPS[3] =(*(unsigned int *)(CLUDE_AUTO_3_BASE+CLUDE_AUTO_SIZE_ADDRESS))/16;
    
    
  }

}


/*******************************
判断电动缸是否继续运行
*******************************/

 /*       if((SlideMotor_GetPower() == SLIDE_MOTOR_POWER_ON)||(BackMotor_GetPower() == BACK_MOTOR_POWER_ON)||(LegMotor_GetPower() == LEG_MOTOR_POWER_ON))
        {
          bVibratePause=1;//摇摆电机暂停运行
          bKeySeatVibrate=0;
          
         Waveringly_Set_Pwm_Data(0);
         
         nKeySeatVibrateStrength_old=nKeySeatVibrateStrength;   
        }

        if((SlideMotor_GetPower() == SLIDE_MOTOR_POWER_OFF)||(BackMotor_GetPower() == BACK_MOTOR_POWER_OFF)||(LegMotor_GetPower() == LEG_MOTOR_POWER_OFF))
        {
           if(bVibratePause)
           {
          
            nKeySeatVibrateStrength=nKeySeatVibrateStrength_old;
            bKeySeatVibrate=1;     
            bKeySeatEnable=1;
            bVibratePause=0;
           }
           
          
        }   */





unsigned char IsPowerZeroBackLeg(void)//ZERO_BACK_LEG_POWER_ONWaveringly_Set_Pwm_Data(0);
{
 if( (SlideMotor_GetPower() == SLIDE_MOTOR_POWER_ON) || (BackMotor_GetPower() == BACK_MOTOR_POWER_ON) || (LegMotor_GetPower() == LEG_MOTOR_POWER_ON))   return ZERO_BACK_LEG_POWER_ON;
 return ZERO_BACK_LEG_POWER_OFF;
}





void Main_Update(void)
{
    BuleTooth_xmodem_update();  
}

unsigned char Is_THIGH_CHR(void)//THITH_PUMP_ON
{


    if(  ((bLeftThighAirBagValve == VALVE_ON)&& (bRightThighAirBagValve == VALVE_OFF)) || ((bLeftThighAirBagValve == VALVE_OFF)&& (bRightThighAirBagValve == VALVE_ON)) )return THITH_PUMP_ON;
    return THITH_PUMP_OFF;



}
//    if(bKeyBackPadUp == TRUE)//  if(bKeyBackPadDown == TRUE) if((bKeyBackPadUp == FALSE) && (bKeyBackPadDown == FALSE))

void Main_ReStartShoulderCheck(void)
{
static unsigned int w_old_Positon=1700;//初始化大于最大值1600
static unsigned int w_cur_Postion;
static unsigned int w_delta_Postion;
static unsigned char  bfirst=0;
static unsigned char  bfirst2=0;


 // bShoulderOK=1;

  if(!bShoulderOK)return;

  
  
  if((nBackMainRunMode == BACK_MAIN_MODE_AUTO)&&(nBackSubRunMode == BACK_SUB_MODE_AUTO_1))return;
 
  
  if((nBackMainRunMode==BACK_MAIN_MODE_3D) ||(nBackMainRunMode==BACK_MAIN_MODE_AUTO))
  {
    
    if( (bKeyBackPadDown == TRUE) ||  (bKeyBackPadUp==TRUE) )
    {
      
           if(bfirst==0)
           {
              w_old_Positon=BackMotor_Get_Location();
              bfirst=1;
              bfirst2=1;
             
             
           }
       return;
    }
    if((bKeyBackPadUp == FALSE) && (bKeyBackPadDown == FALSE)&& (bfirst2==1))
    {
      
 
         w_cur_Postion=BackMotor_Get_Location();
         bfirst=0;
         bfirst2=0;
         if(w_cur_Postion>=w_old_Positon)
         {
             if(w_cur_Postion <= (w_old_Positon + 200))return;//2sec内不做补偿  200*0.01=2sec
               
           
         }
         else
         {
            if(w_old_Positon <= (w_cur_Postion + 200))return;//2sec内不做补偿
           
         }
         
         if(w_cur_Postion>500)
         {
           
             if(w_cur_Postion>=w_old_Positon)
             {
               w_delta_Postion=w_cur_Postion-w_old_Positon;///100;//单位1000ms
      
                 w_delta_Postion=w_delta_Postion/100;
                 
               nShoulderPosition=nShoulderPosition-w_delta_Postion*5;
               
               
             }
             else
             {
                w_delta_Postion=(w_old_Positon-w_cur_Postion);//100;
                  w_delta_Postion=w_delta_Postion/100;
                nShoulderPosition=nShoulderPosition+w_delta_Postion*5;
               
             }
      
             
           
         }
         else
         {
             if(w_cur_Postion>=w_old_Positon)
             {
                w_delta_Postion=(w_cur_Postion-w_old_Positon);///100;//单位1000ms
      
                w_delta_Postion=w_delta_Postion/100;
               
                nShoulderPosition=nShoulderPosition-w_delta_Postion*2;
               
             }
             else
             {
                w_delta_Postion=(w_old_Positon-w_cur_Postion);//100;        
                  w_delta_Postion=w_delta_Postion/100;
               nShoulderPosition=nShoulderPosition+w_delta_Postion*2;
             }      
             
             
         }
      
         if(w_delta_Postion>=2)//2sec 
         {
                if(nShoulderPosition>TOP_POSITION)
                {
                  nShoulderPosition=TOP_POSITION;
                }
   //              printf("W8:%d,vout:%d\n",nShoulderPosition,nShoulderPosition);   
                BodyDataRefresh();
         }         
          
      
    }
    
   
  // if(bfirst2==1)//(w_old_Positon<=BACK_MOTOR_MAX_POSITION)
  // {


     
 //  }

 //  w_old_Positon=w_cur_Postion;
  
  }
  
}



/**************************************************************
//初始明文
unsigned char AES_PlantTest1[16] = "This aaaaa text";
//使用该变量来查看加密之后的密文
unsigned char AES_PlantTest2[16];
//使用该变量来查看解密之后的明文
unsigned char AES_PlantTest3[16];
//密钥
unsigned char g_ucKey[16] =
{
    0x12, 0x23, 0x34, 0x45, 0x56, 0x67, 0x78, 0x89,
    0x9A, 0xAB, 0xBC, 0xCD, 0xDE, 0xEF, 0xF0, 0x00
};


void AES_ECB_128bit_Encrypt(void)
{
    AES_ECB128(AES_PlantTest2,AES_PlantTest1,16,g_ucKey,true);
    
    AES_DecryptKey128(g_ucKey,g_ucKey);
    
    AES_ECB128(AES_PlantTest3,AES_PlantTest2,16,g_ucKey,false);
}

**********************************************************************/




#define PLANT_ADDR_BASE          ((uint32_t) 0x0001f800UL)//((uint32_t) 0x0FE00200UL)

const unsigned char AES_PlantTest1[16] = "Rongtai Health";//明文

void AES_ECB_128bit_Encrypt(void)
{
  static unsigned char AES_PlantTest2[16];//使用该变量来查看加密之后的密文
  static  unsigned char AES_PlantTest3[16];//使用该变量来查看解密之后的明文
  CMU_ClockEnable(cmuClock_AES, true);  
  for(unsigned char i = 0; i < 16; i++)
  {

    AES_PlantTest2[i] =0;

    AES_PlantTest3[i] =0;
    
  }
  
  unsigned char g_ucKey[16] =
  {
      0x12, 0x23, 0x34, 0x45, 0x56, 0x67, 0x78, 0x89,
      0x9A, 0xAB, 0xBC, 0xCD, 0xDE, 0xEF, 0xF0, 0x00
  };  
  
    unsigned int PlantAddr = PLANT_ADDR_BASE;
       
    unsigned int snH = DEVINFO->UNIQUEH;
    unsigned int snL = DEVINFO->UNIQUEL;
    
    for(unsigned char i = 0; i < 16; i++)
    {    
        AES_PlantTest2[i] = ReadEEByte(PlantAddr);
        PlantAddr++;
    }
     
    g_ucKey[1] = (unsigned char)(snH >> 24);
    g_ucKey[2] = (unsigned char)(snH >> 16);
    g_ucKey[3] = (unsigned char)(snH >> 8);
    g_ucKey[4] = (unsigned char)(snH);
    
    g_ucKey[5] = (unsigned char)(snL >> 24);
    g_ucKey[6] = (unsigned char)(snL >> 16);
    g_ucKey[7] = (unsigned char)(snL >> 8);
    g_ucKey[8] = (unsigned char)(snL);
       __no_operation();
      __no_operation();
      __no_operation();
      __no_operation();  
      __no_operation();
      __no_operation();  
 //   AES_ECB128(AES_PlantTest2,AES_PlantTest1,16,g_ucKey,true);//加密
        __no_operation();
      __no_operation();
      __no_operation();
      __no_operation();  
      __no_operation();
      __no_operation();     
    AES_DecryptKey128(g_ucKey,g_ucKey);//计算解密密钥
   //          解密之后的明文  加密之后的密文
    AES_ECB128(AES_PlantTest3,AES_PlantTest2,16,g_ucKey,false);//进行AES 128 位的ECB 解密
      __no_operation();
      __no_operation();
      __no_operation();
      __no_operation();  
      __no_operation();
      __no_operation();   
      
    if(strcmp(AES_PlantTest1,AES_PlantTest3) != 0)
    {
        while(1);
    }
    CMU_ClockEnable(cmuClock_AES, false);  
}



//unsigned int w_test;
//unsigned char by_dat1,by_dat2;



extern unsigned short __checksum; 


void main(void)
{
  SCB->VTOR = (uint32_t)(8 * 1024);    //将地
  if(__checksum == 0) __checksum = 1;//址改为0，不需要加载BOOTLOADER
  
  
  
   // Main_Auto_Program_Test();
     //     AxisMotor_10msInt();//20MS镜向电机不走自动停止  ,目前8600S主板上已经增加该保护
   //     WalkMotor_10msInt();  //行走电机不动作，20ms后自动关闭行走电机 目前8600S主板上已经增加该保护
    Main_Initial_IO(); //硬件初始化

   //AES_ECB_128bit_Encrypt();
      
    

    

   
    Main_Initial_Data();  //software initial
      __no_operation();
      __no_operation();
      __no_operation();
      __no_operation();
      __no_operation();

      __no_operation();
      __no_operation();
      __no_operation();
      __no_operation();
      __no_operation();


 //by_dat1=0x01;
 //  by_dat2=0x03;
   
 // w_test=    by_dat1<<8;
  
      __no_operation();
      __no_operation();
      __no_operation();
      __no_operation();
      __no_operation();   
   // w_test|= by_dat2; 
      
      
 //    printf("1_10:%d\n\r",sizeof(struct WALK_KNEAD_KNOCK_MOTOR_STRUCT_AUTO));
     
   //  sizeof(AutoFunction0)/sizeof(struct Walk_Knead_Knock_Motor_Struct_Auto)/
     
     
  //    printf("1_10:%d\n\r", sizeof(AutoFunction0));
   //   printf("1_10:%d\n\r", sizeof(Walk_Knead_Knock_Motor_Struct_Manual));
      
   //     printf("1_10:%d\n\r", sizeof(AutoFunction1));
      
      __no_operation();
      __no_operation();
      __no_operation();
      __no_operation();
      __no_operation();   
    nChairRunState =CHAIR_STATE_INITIAL;//CHAIR_STATE_UPDATE;//CHAIR_STATE_INITIAL;//CHAIR_STATE_SLEEP;//

    //Valve_SetAirBagStrength(1);//test
    //Valve_SetStretchCharge_ARM(0,1);//test
    
    //testwgh();
    while(1)
    {
        switch(nChairRunState)
        {
        default:
        case CHAIR_STATE_INITIAL:       Main_Initial();break;  
        case CHAIR_STATE_IDLE:          Main_Idle();break;//空闲模式，打开按键进入等待命令模式，10秒无按键进入睡眠=//待机状态,刚上电系统进入待机模式（Main_Sleep）,也可以进入到工程模式
        case CHAIR_STATE_SETTLE:        Main_Settle();break;
        case CHAIR_STATE_WAIT_COMMAND:  Main_WaitCommand();break; 
        case CHAIR_STATE_RUN:           Main_Work();break; 
        case CHAIR_STATE_PROBLEM:       Main_Problem();break;  
        case CHAIR_STATE_ENGINEERING:   Main_Engineering();break;
        case CHAIR_STATE_SLEEP:         Main_Sleep();break;//当有按键按下时进入到等待命令模式，睡眠状态就是待机状态，手控器显示待机状态
        case CHAIR_STATE_DEMO:          Main_Demo();break;
        case CHAIR_STATE_UPDATE:        Main_Update();break;
       // case CHAIR_STATE_CALIBRATION:   Main_Auto_Calibration();break;
        }
    }
}

void  USB_MP3_SCAN_PROC(void)
{

       unsigned char cmd;
       cmd= BlueToothUart_GetKey();
	if(true ==  BlueToothUart_GetRXStatus())
	{
		if(0xc7 == cmd)
		{
			n_usb_indicate = USB_SONG_ON;
		}
		else if(0x03 == cmd)
		{
			n_usb_indicate = BLUE_SONG_ON;
			//

                           //  增加音量定阶。
			n_usb_send_buf[0]= 0xA2;
			n_usb_send_buf[1]= 0x10;
			n_usb_send_buf[2]= 0x01;
			n_usb_send_buf[3]= 0x0F;
			n_usb_send_buf[4]= 0x3E;
			n_usb_send_buf[5]= 0x00;  // C1 : 1100,0001 = 0011,1110+1 = 3F
			//BlueTooth_DMAUart_Transmit_Packet(n_usb_send_buf,5);	
                        BlueToothUart_Transmit_Packet(n_usb_send_buf,4);


			//
		}
		else if((0x0f== cmd)||(0xcf== cmd))
		{

			n_usb_indicate = AUX_SONG_ON;

		}
		BlueToothUart_ClearRXStatus();
		BlueToothUart_SetKey(0);
	}
	return;
}


/*
switch(nChairRunState)
{
default:
case CHAIR_STATE_INITIAL:
  Main_Initial();
  break;  
case CHAIR_STATE_IDLE:          Main_Idle();break;//空闲模式，打开按键进入等待命令模式，10秒无按键进入睡眠=//待机状态,刚上电系统进入待机模式（Main_Sleep）,也可以进入到工程模式
case CHAIR_STATE_SETTLE:        Main_Settle();break;
case CHAIR_STATE_WAIT_COMMAND:  Main_WaitCommand();break; 
case CHAIR_STATE_RUN:           Main_Work();break; 
case CHAIR_STATE_PROBLEM:       Main_Problem();break;  
case CHAIR_STATE_ENGINEERING:   Main_Engineering();break;
case CHAIR_STATE_SLEEP:         Main_Sleep();break;//当有按键按下时进入到等待命令模式，睡眠状态就是待机状态，手控器显示待机状态
case CHAIR_STATE_DEMO:          Main_Demo();break;
case CHAIR_STATE_UPDATE:        Main_Update();break;
// case CHAIR_STATE_CALIBRATION:   Main_Auto_Calibration();break;
}

*/