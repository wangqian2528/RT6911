

/******************************************************************************
С�� TIMER0     CC0 ,       #2 
������  TIMER0  CC2 ,       #2
����   TIMER0   CC1,        #2



����  TIMER1   CC0,         #2
ҡ��  TIMER1   CC1          #2


�û�   TIMER2 ,    CC0 ,         #0
3D��� TIMER2 ,    CC2,          #0
����   TIMER2 ,    CC1           #0

LED1     TIMER3   CC0  ,        #1
LED2     TIMER3   CC1           #1
LED3     TIMER3   CC2           #1
unsigned int BackMotor_VoltageAdj(unsigned int setDuty)  //��������иú��������⣬Ŀǰ��û�д���

*****************************************************************************/



//RT8302T   modify by taoqingsong 2015-5-14
//BOOTLAODER����ᶪʧ��
//����ģʽ����
//�޸Ŀ�����С�ȡ��������綯��ģ��
//����汾��� ���������������綯�ף�δ���ǿ�����С�ȡ��������綯��ģ��֮�������
//�ð汾���е綯�׶�����������

//���³�ʼ��//�޸Ŀ�����С�ȡ��������綯��ģ��

//�ð汾Ŀǰ����ģʽ�����������������¼���ʱ������
//new
//�޸��������24������������С�Ȱ��������
//�ð汾����������м򵥵��滻���õ���8600S���������򣬲�����  ,����ģʽ����OK
//�ð汾�����ҳ����Ϊ8305A�����ҳ����������Ҫ����2015-6-9
 //�����������ok
//��3D�Զ������⣬��������
//8600S��λ��Χ 125-250   ���������Ϊ397
//�޸��������Ϊ3������
//�����ߣ������õȵ����ѹ����24V 2015-6-23

//������С�ȡ��������綯�׵����ѹƫС 
//������С�ȡ��������綯�׵����ѹƫС  �������  #define SLIDE_SET_VOLTAGE    280000//250000 //25V
//�������ߵ���������ֿ����Բ��������
//add �Զ��������������
//���� �Զ��������������2015-6-27

//�÷���ʹ��XOMDOEMЭ�鴫������ʱ�ر������ж�
//����汾����ʱ����NAK ����λ������λ��û�м������д���

/*������������Ϊ9600��ͬʱ�ر�дFLASH����λ������ſ�������д��ȥ*/

//�ð汾��������ʱ�������д����FLASH----
//дFLASH����OK
//дFLASH OK2--����ѭ������дFLASH
//xдFALSH ��ʱ�������ѭ�����Ͳ���OK
//�ó�����Ը����ƶ˵�ַ1�͵�ַ2���Զ�����
//�ð汾���Ը�ƽ�����������������ΪRT8600S-008
//�ð汾MEMORY���������ƶ˸��µĺ���û�б��Ƶ�bluetooth.cģ����
//����XOMDEMУ���
//����XOMDEMУ��� test ok
//���߼����ϵ�XMODEM���������أ�
//�����ҽź���������
//--
//�������������˳���������ģ�� 
//�޸������FLASH һ��ֻ�ܲ��2K
//������ģ��ĳ��ļ���ʽ
//�޸ļ�λ���
//���ͼ��OK

//�滻void Main_Start_Manual(void)
//�����ҽų����BUG
//BEEP���ϴ�����������
//�޸�������������
//�޸����ҳ���������APP���Ӧ
//�ο�����˯��ģʽ����3D����ʱ��3D�ٶȵ�С
//���������ϴ�ID�Ÿ�APP,test ok
//ADD DEMO����
//����������Ϊֻ��������������ӹ��ߵ�����������ʱ�޷���ֱ��
//������Ħʱ������λ�ö�λ�ڵ�һ������λ�ô�
//��λ��ⲻ�ԣ�������Ƽ�λ������

//������С�ȣ��������ж�������ʱ�ر�ҡ�ڣ�
//���ӿ����ƶ����λ���Ĳ��� ,test ok
//����180V-250V���ҳ���
//����3D���������С ,3D�������Լ�⣬ֻ�ǲ�̫�ȶ���
//���ߵ���������230ʱ���粿���ҹرգ�Ҫ��Ȼ�˻᲻���
//
//ȡ���Զ�����ʱ������������λ�ã���Ϊ�ڶ���������


//����С�Ⱥ���������ѹ250V ƫ�������

//������ģʽ�Զ����ҡ�ڵ�����������

 // GPIO_PinOutClear(BACK_MOTOR_DECAY_PORT, BACK_MOTOR_DECAY_BIT);
 //   GPIO_PinOutSet(BACK_MOTOR_DECAY_PORT, BACK_MOTOR_DECAY_BIT);//Ϊ�ߵ�ƽʱ��250V ��������ѹ���½���15V
//����8305T С�������޷��Լ�����
//������Ϣ����ģʽ�ֱ۲���ʾ����
//�Զ���⣬���Ӳ��ڸ�λ״̬ʱ����ⲻ���Ƕȿ���

//�������ͼ��ʱ���ֱ۳�3�Σ��ұ߳�һ������
//����Ƕȿ���λ�ã����������������
//2015-10-11�ҽų���
//�����Զ�6��ʾϴ������ʾ�������⣬
//zaici jiancha�ҽų���go o
//go on  foot  c
//�������ͼ��δ�����Ϳ�ʼ����
//��������ѹ�������н����󷵻ص�һ����ʾ����ʵ����������û�����Ǽ�ѹ�ĸо� 2015-10-19

/*


void Main_BlueToothSend(void)
  return;//
20160624
�޸� APP
*/
/*
20170207  20170208...........
3.�Զ�ģʽ����ϰ���Ħ���粿���Ҷ�����һЩ
4.�ֶ�����ѡ���ɵ�ѡ��Ϊ��ѡ
5.���5����DEMO
6.������������ͼ��1���Ӻ�ʼ
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
#define bHasKneadWidthMaxPulse	 	GlobalFlags0.bD4//�����������е�λ��
#define bDisplayKneadWidthMax		GlobalFlags0.bD5
#define bDisplayKneadTrackMax		GlobalFlags0.bD6
#define bUpdateLocate 			GlobalFlags0.bD7//���ߵ��������±�־����λʱ����һ������

__no_init BITS GlobalFlags1 ;
#define bKneadWidthMedPulseLevel0 	GlobalFlags1.bD0
#define bKneadWidthMedPulseLevel1 	GlobalFlags1.bD1
#define bKneadWidthMedPulseLevel2	GlobalFlags1.bD2
#define bKneadWidthMedPulseLevel3 	GlobalFlags1.bD3
#define bHasKneadWidthMedPulse		GlobalFlags1.bD4//�����������е�λ��
#define bDisplayKneadWidthMed		GlobalFlags1.bD5
#define bDisplayKneadTrackMed		GlobalFlags1.bD6
#define bLegPadLinkage 			GlobalFlags1.bD7   //С������������־

__no_init BITS GlobalFlags2 ;
#define bKneadWidthMinPulseLevel0 	GlobalFlags2.bD0
#define bKneadWidthMinPulseLevel1 	GlobalFlags2.bD1
#define bKneadWidthMinPulseLevel2 	GlobalFlags2.bD2
#define bKneadWidthMinPulseLevel3 	GlobalFlags2.bD3
#define bHasKneadWidthMinPulse	 	GlobalFlags2.bD4// bHasKneadWidthMinPulse = TRUE ;//����������խ��λ��
#define bDisplayKneadWidthMin		GlobalFlags2.bD5
#define bDisplayKneadTrackMin		GlobalFlags2.bD6
#define bWaveMotorFail 			GlobalFlags2.bD7//ҡ�ڵ�����ϱ�־λ

__no_init BITS GlobalFlags3 ;
#define bShoulderOK	                GlobalFlags3.bD0
#define bBlueToothStatus		GlobalFlags3.bD1  //�����򿪱�־λ
//#define bKeyPowerSwitch 		GlobalFlags3.bD2
#define bKeyWaistHeat 			GlobalFlags3.bD3
//#define bSlowDisplayFlash		GlobalFlags3.bD4
#define bKeySeatVibrate 		GlobalFlags3.bD5   //����ҡ�ڰ�����־
#define bKeySeatEnable 		        GlobalFlags3.bD6   //����ҡ�ڰ�����־
#define bVibratePause 		        GlobalFlags3.bD7   //����ҡ�ڰ�����־bKeySeatVibrate

//#define bMP3RunMode	 		GlobalFlags3.bD7

//λ����
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
#define bBackAutoModeInit 			GlobalFlags5.bD0///�Զ������� ��о  ��ʼ�����ĳ�ʼ����־λ����ʼ����ɺ��־λ��0
#define bBackManualModeInit 		        GlobalFlags5.bD1//�ֶ�������  ��о   �����ʼ�����ĳ�ʼ����־λ
#define bWalkMotorInProcess 		        GlobalFlags5.bD2 //���ߵ������ʼִ�б�־,������￪ʼ���У���������Ϻ��־λ��0
#define bKneadMotorInProcess 		        GlobalFlags5.bD3 //����������ʼִ�б�־������˳ʱ������3Ȧ��ֹͣ
#define bKnockMotorInProcess 		        GlobalFlags5.bD4 //�û��������ִ�б�־
#define bGetNextActionStep 			GlobalFlags5.bD5//�Զ������л�ȡ��һ��������־λ
#define bKeyWalkUp 				GlobalFlags5.bD6   //�û������������߰�����־ֵ�����ͷŰ���ʱ ��0��־λ
#define bKeyWalkDown 				GlobalFlags5.bD7   //�����������߰�����־ֵ




__no_init BITS GlobalFlags6 ;
#define b3D_MotorInProcess 			GlobalFlags6.bD0//3D����������б�־λ
#define bMassagePositionUpdate 			GlobalFlags6.bD1   //�����������ʼ�����ı�־λ��������Ϻ������0
//#define bMarkSpace				GlobalFlags6.bD2
#define bSendBuzzerMode 			GlobalFlags6.bD3   //ȫ�ֱ�����BEEP����ʹ��
//#define bSignalSendPacket 			GlobalFlags6.bD4
#define bMasterSendPacket 			GlobalFlags6.bD5 



//bReconfigFlag 			GlobalFlags6.bD6
#define bZeroflash                              GlobalFlags6.bD6
#define bKneadWidthChange			GlobalFlags6.bD7






__no_init BITS GlobalFlags7 ;
#define bKeyBackPadUp 				GlobalFlags7.bD0   //�û����¿������ϰ������趨�ı�־λ
#define bKeyBackPadDown 			GlobalFlags7.bD1//�û����¿������°������趨�ı�־λ
#define bOzonEnable 	                        GlobalFlags7.bD2
//#define bReachBackPadDownPosition 	        GlobalFlags7.bD3
//#define bBackPadMotorPowerFlag		GlobalFlags7.bD4
//#define bGetAirBagNextStep 			GlobalFlags7.bD5
//#define bCurActionStepChange		        GlobalFlags7.bD6
//#define bWalkLocateChange			GlobalFlags7.bD7

__no_init BITS GlobalFlags8 ;
#define bKeyLegPadUp 				GlobalFlags8.bD0  //�û�����С�����ϰ�����־λ
#define bKeyLegPadDown 				GlobalFlags8.bD1  //С������綯�����־���ڰ���������������
#define bKeyFlexOut 		                GlobalFlags8.bD2  //�û�����С�������찴��
#define bKeyFlexIn 	                        GlobalFlags8.bD3   //�û�����С������������
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
//�����������ƶ˸���
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

//�ƶ˳����� ��88K�ĵط���ʼ
/*
��ַ0,1,2,3 ����ID�� ID��Ϊ0��0xFFFFFFFF�����Ϊ��
��ַ4,5 ���ݳ���
��ַ6,7 ����checksum
*/
#define CLUDE_AUTO_0_BASE  ((uint32_t) (88*1024))
#define CLUDE_AUTO_1_BASE  ((uint32_t) (96*1024))
#define CLUDE_AUTO_2_BASE  ((uint32_t) (104*1024))
#define CLUDE_AUTO_3_BASE  ((uint32_t) (112*1024))
#define CLUDE_AUTO_ID_ADDRESS    0  //32λ
#define CLUDE_AUTO_SIZE_ADDRESS  4  //16λ
#define CLUDE_AUTO_CHECKSUM_ADDRESS  8//16λ
#define CLUDE_AUTO_VERSION_ADDRESS  12//16λ
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
//MP3 ����
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
__no_init unsigned char nBuzzerMode;//beep������ģʽ
/******************************************************************/

/*
#define BACK_SUB_MODE_KNEAD			7
#define BACK_SUB_MODE_KNOCK			8
#define BACK_SUB_MODE_PRESS			9
#define BACK_SUB_MODE_WAVELET			10// //����ͬ��
#define BACK_SUB_MODE_SOFT_KNOCK		11////ߵ��
#define BACK_SUB_MODE_MUSIC			12////���ɰ�Ħ
*/
 //         nBackMainRunMode = BACK_MAIN_MODE_AUTO;
 //        nBackSubRunMode = BACK_SUB_MODE_AUTO_5;
__no_init unsigned char nBackMainRunMode,nBackSubRunMode ;
__no_init unsigned char nCurSubFunction ;     //nCurSubFunctionΪ���е�{BACK_SUB_MODE_KNEAD...BACK_SUB_MODE_MUSIC}֮���һ�ְ�Ħ�ַ�
__no_init unsigned char nCurKneadKnockSpeed ; 
/******************************************************************/
__no_init unsigned int nCurActionStep,nPreActionStep ;  // �Զ�����ǰ��������
__no_init unsigned int nMaxActionStep ;//�Զ��������������
__no_init unsigned char nStartActionStep ;//�Զ�����ʼ������Ĭ��Ϊ0
/******************************************************************/
__no_init unsigned char nTargetMassagePosition;//������λʱ������������� ������С�����£�С�����������λ��
__no_init unsigned short nShoulderPosition,nShoulderPositionTop,nShoulderPositionBottom;//�粿λ�þ������꣬�粿λ����ߵ㣬��͵㣬�ڼ�λ����õ���ȷ�ļ粿λ��
__no_init unsigned int ShoulderSteps;
__no_init int BodyDetectStep;   
//BODY_DETECT_PREPARE:  δ��ʼ��� 
//BODY_DETECT_SHOULDER: ���ڼ����λ��
//BODY_DETECT_3D:       ���ڼ��3D���� 
//BODY_DETECT_OVER:     ������ 

/******************************************************************/
//�����ֿ�����Ҫ֪���������Ϣ�ı���
__no_init unsigned char nKeySeatVibrateStrength, nKeySeatVibrateStrength_old;//�����ȣ���Ӧoutbuf[3]3,4,5λ ��ҡ�ڵ������
//static bool Enable_Vibrate=FALSE; 
extern unsigned int nRoller3sCnt;
__no_init unsigned char nKeyBackLocate;//��о��Ħ��λ����Ӧoutbuf[4]5,6λ    ,ȫ�֣����㣬�ֲ���Ħ
__no_init unsigned int w_PresetTime;  //����Ԥ��ʱ�䣬��Ӧoutbuf[12]0,1λ
__no_init unsigned char nKeyAirBagLocate ;    //���Ұ�Ħ����,��Ӧoutbuf[12]2.3.4λ��ѡ�е����Ұ�Ħ����,��Ӧͨ��Э��� ��ַ12 ʱ�������
/******************************************************************/
 bool bTapping;       
unsigned short nLegAngle;
extern unsigned char nFlexStatus;
unsigned char nLegReady;

unsigned int topPositionRefreshedFlag;
int shoulderPos[3];
 bool bRockEnable = false;
unsigned short  rocktimecount; 
__no_init unsigned char WorkStep; //ҡ��
//__no_init unsigned char cloud_dat[8192];

//__no_init unsigned char cloud_dat2[8192];



//��ͨ����صı���
//�������͵����ݰ����������״̬�����������ҪӦ��״̬������Ӧ��
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

bool bRollerEnable,bLegKneadEnable;   //ȫ�ֱ��������ֿ���

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
//9���ֶ�����
//#define nMaunalSubMode_KNEAD		0
//#define nMaunalSubMode_KNOCK		3
//#define nMaunalSubMode_WAVELET		2////����ͬ��
//#define nMaunalSubMode_SOFT_KNOCK	1////ߵ��
//#define nMaunalSubMode_PRESS		4
//#define nMaunalSubMode_MUSIC		5

//#define nMaunalSubMode_3DMODE_1         6
//#define nMaunalSubMode_3DMODE_2         7
//#define nMaunalSubMode_3DMODE_3         8


__no_init unsigned char nKeyKneadWidth ;//ͨ���ֿ���������ȡ���������խλ��
__no_init unsigned char nKeyKneadKnockSpeed ;//ͨ���ֿ���������ȡ�������ٶ�ֵ{1��2,3.��6}
__no_init unsigned char nMaunalSubMode;    //ͨ���ֿ���������ȡ���ֶ�����. nMaunalSubMode_KNEAD,nMaunalSubMode_KNOCK,nMaunalSubMode_WAVELET
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
// nAxisStrength:��ǿ��  
//unsigned char  nAxisStrength/*,nAxisStrengthBase,nAxisAuto,nAxisMode*/; 
/*
  nKeyAxisStrength �û��趨ֵ  ��ֵ��Χ0-4
  nSetAxisStrength �����趨ֵ  ��ֵ��Χ0-4
  nFinalAxisStrength ʵ��ֵ    ��ֵ��Χ0-4
  nFinalAxisStrength ���㷽ʽ��
*/
unsigned char  nKeyAxisStrength,nSetAxisStrength,nFinalAxisStrength,nAxisUpdateCounter; ////3D ���ȣ�3D�������꣬3D�������Խ�󣬼�ѹ����Խǿ
//unsigned char  nDisplayAxisStrength; //��Ӽ粿��ʾ
unsigned int nWidthOverTime;
unsigned int nPowerOverTime;
unsigned int nWalkOverTime;
unsigned int nBackOverTime;
unsigned int nLegOverTime;
unsigned int nZeroOverTime;
unsigned int nWaveOverTime;
unsigned int nFlexOverTime;

/******************************************************************/
__no_init unsigned short nFinalWalkMotorLocate ;//��������������
/******************************************************************/
//�Զ�������ʱ�������
__no_init unsigned char nCurActionStepCounter;       //��ǰ����ʱ�������(�������������ߣ������ô���)=��о
__no_init unsigned char nCurKnockRunStopCounter;   //ߵ������������
__no_init unsigned char nCur3D_MotorStopCounter;//3D����Զ������У�����Ŀ��λ�õ�ֹͣʱ��
__no_init unsigned char nCurShoulderAdjustCounter ;
__no_init unsigned char n3DMotorRunCounter;
/******************************************************************/
unsigned char nCurKneadWidth = KNEAD_WIDTH_UNKNOWN ;//KNEAD_WIDTH_MIN,��ǰ����ͷ��λ��
__no_init unsigned char nCurKneadMotorCycles ;//����Ȧ��

//nFinalKneadMotorState = STATE_RUN_CLOCK ;//����ͷ״̬��˳ʱ�룬ֹͣ����ʱ�ӵ�
unsigned char nCurKneadMotorState,nPrevKneadMotorState,nFinalKneadMotorState ;

/******************************************************************/
__no_init unsigned char nWalkMotorControlParam1;
__no_init unsigned short nWalkMotorControlParam2 ;
__no_init unsigned char nKneadMotorControlParam1,nKneadMotorControlParam2 ;
__no_init unsigned char n3D_MotorControlState,n3D_MotorControlPosition,n3D_MotorControlSpeed,n3D_MotorControlStopTime;
__no_init unsigned char nKnockMotorControlParam1,nKnockMotorControlParam2,nKnockMotorControlParam3 ;

unsigned char engineeringTime_10msFlag = 0; //����ģʽʹ��
unsigned char nStretchFlextimer; //define flex������ĵ�ʱ��    wgh20170215
//unsigned short adcAudio_L_Base,adcAudio_R_Base;



//=======================================================
//ҡ�����PWMֵ
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
__no_init WALK_KNEAD_KNOCK_MOTOR_STRUCT_AUTO AutoDirector;//AutoDirectorΪ����ṹ�����

__no_init WALK_KNEAD_KNOCK_MOTOR_STRUCT_MANUAL ManualDirector[4] ;//�ֶ�����  ManualDirector[4]Ϊ����ṹ������
//���涨��7�����ҳ���
__no_init st_AirBag st_AirBagLegFoot,st_AirBagSeat, st_AirBagArmNeck,st_AirBagAuto;//,st_AirBag_Neck;//20170205 WGH 
__no_init st_AirBag st_AirBagLegFoot_Seat,st_AirBagLegFoot_Arm,st_AirBagArm_Seat,st_AirBagLegFoot_Arm_Seat,st_AirBagAuto_Upbody;
/******************************************************************/
#define AUTO_FUNCTION_0_STEPS	sizeof(AutoFunction0)/sizeof(struct Walk_Knead_Knock_Motor_Struct_Auto)////�����һ���Զ��������еĲ�������Ҫ��AutoFunction0����ĳ���
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


//10���Զ�����������
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
//�Զ�����ѭ�����п�ʼ�Ĳ���
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
bool bAxisUpdate;  //3D���������±�־λ����ΪTRUEʱ3D���У���Ϊ0ʱ3Dֹͣ����

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
    KneadMotor_Initial_IO();//������   test ok
    UART0_Initial_IO();//�ֿ���UART
    Valve_Initial_IO();//����.�ͽŵ׹��ֿ���
    Axis_Initial_IO();//ZDirectionMotor ����



    Waver_FlexMotor_Initial_IO();//����ҡ�ڵ�� test ok
//    ZeroMotor_Initial_IO();//���������
    
    SlideMotor_Initial_IO();
    
    
    LegMotor_Initial_IO();//С��   test ok
    BackMotor_Initial_IO();//���� TEST OK
    WalkMotor_Initial_IO();//���� test ok
    Input_Initial_IO();//
    KnockMotor_Initial_IO();//�û�     test ok
    MP3Control1_Initial_IO();
    WaistHeat_Initial_IO();//��������
    LED_RGB_Initial_IO();
    UartLeg_Initial_IO();
    BlueToothUart_Initial_IO();//WIFI��ʼ������
    DMA_Ctrl_Init();
    ADC_Data_Init();
    //LEUart_Initial_IO();
    LEUART0_Initial_IO();//��ȡ3D��о3D�����״̬�ź�
  //  UART1_Initial_IO();//���������ʼ����8600S����Ҫ
    __enable_irq();
  
}

#define		ADSTRONG_ON  8  //������
#define		ADSTRONG1  10  //������������
#define		ADSTRONG2  30
#define		ADSTRONG3  50
#define		ADSTRONG4  70
#define		ADSTRONG5  100
#define		ADSTRONG6  150
unsigned int nAvrADResult0 ;
unsigned int nMusicKnockPWM ;//�������ֻ���
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
   //   VoiceUart_SetMusicStatus(1);  //������   delete by taoqngsong
    }
    else
    {
  //    VoiceUart_SetMusicStatus(0); //������   //de by taoqingseong
    } 
}


//�ô����ֻ���
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
n3D_position: 1-5 5������λ��
n3D_MotorSpeed�������ٶ�
n3D_MotorStopTime�����е�λ�ú��ͣ��ʱ��
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
   case  _3D_MANUAL:  //�ֶ����� 
        b3D_MotorInProcess = FALSE ; 
        if(Input_GetWalkMotorPosition() > _3D_MOTOR_WALK_MAX_POSITION)
        {//���ߵ���ߵ���ߵ� 
          AxisMotor_Control(STATE_RUN_AXIS_FORWARD,0,_3D_SPEED_7);
          bAxisUpdate = true;  //3D���������±�־λ����ΪTRUEʱ3D���У���Ϊ0ʱ3Dֹͣ����
          nDisplayAxisStrength = 4;// wgh 20140421
          break;      
        }
      
        if(bAxisUpdate)
        {
          nFinalAxisStrength = nKeyAxisStrength;  //3D ���ȣ�3D�������꣬3D�������Խ�󣬼�ѹ����Խǿ
          nDisplayAxisStrength = nFinalAxisStrength;
          if(AxisMotor_Control(STATE_RUN_AXIS_POSITION,nFinalAxisStrength,_3D_SPEED_10) == TRUE)
          {
            bAxisUpdate = FALSE;  //3D����ߵ�Ŀ��λ��
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
          bAxisUpdate = true; // //3D���������±�־λ����ΪTRUEʱ3D���У���Ϊ0ʱ3Dֹͣ����
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
                     nDisplayAxisStrength = nFinalAxisStrength;//20150421 wgh ��Ӽ粿��ʾ
                     break;
            case 4:  if(strength < 4) strength++;
            case 3:  if(strength < 4) strength++;
                     nFinalAxisStrength = strength; 
                     nDisplayAxisStrength = nFinalAxisStrength;//20150421 wgh ��Ӽ粿��ʾ
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
          if(nCur3D_MotorStopCounter >= stopTime)//3D�Զ����򵽴�Ŀ��λ�õ�ֹͣʱ��
          {
            b3D_MotorInProcess = FALSE ; 
          }
        }
        else//3D���δ��Ŀ��λ��
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

void AxisUpdate_Waitting_100msInt(void)//3D����10�룬����λ����Ϊ��λ
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
   case  _3D_MANUAL:  //�ֶ����� 
     //3D����10�룬����λ����Ϊ��λ
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
    {//      //nKeyAxisStrength=3D�����ֿ����ϵĿ�ݼ�
      nFinalAxisStrength = nKeyAxisStrength; //nKeyAxisStrength=3D�����ֿ����ϵĿ�ݼ�
      nDisplayAxisStrength = nFinalAxisStrength;//��ʾ3D���ȴ�С
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
      //3D����10�룬����λ����Ϊ��λ
      //bAxisUpdate_Manual_Waitting_Timef = TRUE;
    }
   else
   {
     AxisMotor_Control(STATE_AXIS_IDLE,0,_3D_SPEED_10); 
     //3D����10�룬����λ����Ϊ��λ
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
                 nDisplayAxisStrength = nFinalAxisStrength;//20150421 wgh ��Ӽ粿��ʾ
                 break;
        case 4:  if(strength < 4) strength++;
        case 3:  if(strength < 4) strength++;
                 nFinalAxisStrength = strength; 
                 nDisplayAxisStrength = nFinalAxisStrength;//20150421 wgh ��Ӽ粿��ʾ
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
    
    //��о����MANUAL ģʽʱ��3D ǰ����
    
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
      b3D_MotorInProcess = FALSE ;//20170228 WGH  ��TAPOFF �У�����3�κ��оͣ���ϱ߲��������ͼ���һ��������
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
   case  _3D_MANUAL:  //�ֶ����� 
     //3D����10�룬����λ����Ϊ��λ
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
  
    if(bAxisUpdate) //nKeyAxisStrength=3D�����ֿ����ϵĿ�ݼ�
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
      //3D����10�룬����λ����Ϊ��λ
      //bAxisUpdate_Manual_Waitting_Timef = TRUE;
    }
   else
   {
     AxisMotor_Control(STATE_AXIS_IDLE,0,_3D_SPEED_8); 
     //3D����10�룬����λ����Ϊ��λ
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
                 //nDisplayAxisStrength = nFinalAxisStrength;//20150421 wgh ��Ӽ粿��ʾ
                 break;
        case 4:  if(strength < 4) strength++;
        case 3:  if(strength < 4) strength++;
                 nFinalAxisStrength = strength; 
                 //nDisplayAxisStrength = nFinalAxisStrength;//20150421 wgh ��Ӽ粿��ʾ
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
//�����������
#define KNOCK_STOP		0 //ֹͣ
#define KNOCK_RUN_WIDTH		1 //����խ��λ��ɺ�����
#define KNOCK_RUN		2 //�������խ��λ����������������
#define KNOCK_RUN_STOP		3 //����խ��λ��ɺ�������ʱ�������ֹͣ
#define KNOCK_RUN_MUSIC		4 //���ֻ���ģʽ�������խ��λ�޹أ�
*/

//�����������
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
        nCurKnockRunStopCounter = 0 ;//ߵ������������
        return;      
      }
  }
    //static int step = 0;
    //�ô������ֻ�������Ƶ��
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
            case KNOCK_RUN_WIDTH://��λ��ɺ����
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
            case KNOCK_RUN_STOP:  //ߵ��
                if(bKneadMotorInProcess == TRUE)
                {
                    bKnockMotorPowerFlag = FALSE ;
                    nCurKnockRunStopCounter = 0 ;//ߵ������������
                }
                else
                {
                    if(nCurKnockRunStopCounter < nKnockingMotorRunTime)//nCurKnockRunStopCounter:��λ:2ms; nKnockingMotorRunTime:��λ:100ms;
                    {
                        bKnockMotorPowerFlag = TRUE ;
                    }
                    if((nCurKnockRunStopCounter >= nKnockingMotorRunTime) && (nCurKnockRunStopCounter < (nKnockingMotorRunTime + nKnockingMotorStopTime)))
                    {
                        bKnockMotorPowerFlag = FALSE ;
                        //���ߵ����ɶ���ʱ���ö���Ҳ����
                        

                        
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
        nCurKnockRunStopCounter = 0 ;//ߵ������������
        return;      
      }
  } 
  
  
  
    //static int step = 0;
    //�ô������ֻ�������Ƶ��
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
            case KNOCK_STOP://���ߺ��û���ﹲ��һ��ʱ������������û�������ʼʱ�� ������ﲻ���Լ���������ֹͣ�ڵ�ǰλ��
                bKnockMotorPowerFlag = FALSE ;
                if(nCurActionStepCounter >= nKnockingMotorRunTime)////��ǰ����ʱ����������߻��û�ʱ�������
                {
                    bKnockMotorInProcess = FALSE ;
                }
                break ;
            case KNOCK_RUN_WIDTH://��λ��ɺ����,����խ��λ��ɺ�����
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
            case KNOCK_RUN_STOP:  //ߵ��  ���ö���������ߵ�������Ž���
                if(bKneadMotorInProcess == TRUE)
                {
                    bKnockMotorPowerFlag = FALSE ;
                    nCurKnockRunStopCounter = 0 ;//ߵ������������
                }
                else
                {
                    if(nCurKnockRunStopCounter < nKnockingMotorRunTime)//nCurKnockRunStopCounter:��λ:2ms; nKnockingMotorRunTime:��λ:100ms;
                    {
                        bKnockMotorPowerFlag = TRUE ;
                    }
                    if((nCurKnockRunStopCounter >= nKnockingMotorRunTime) && (nCurKnockRunStopCounter < (nKnockingMotorRunTime + nKnockingMotorStopTime)))
                    {
                        bKnockMotorPowerFlag = FALSE ;
                        //���ߵ����ɶ���ʱ���ö���Ҳ����
                        
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
    
//bKeySeatEnable,bKeySeatVibrate������������û�г�ʼ�������ֿ����ӿ���Ӧ������ҡ�ڴ�/�رհ���

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
//ҡ�ڵ���ƺ���IsPowerZeroBackLeg(void)//ZERO_BACK_LEG_POWER_ONWaveringly_Set_Pwm_Data(0);
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
            bHasKneadWidthMinPulse = TRUE ;//����������խ��λ��
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
            bHasKneadWidthMedPulse = TRUE ;//�����������е�λ��
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
            bHasKneadWidthMaxPulse = TRUE ;//�����������е�λ��
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
        case KNEAD_STOP_AT_MIN:////��Ħ��ͣ����խ��λ��
            if(nCurKneadWidth == KNEAD_WIDTH_MIN)//�������խ��λ��
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
                    nFinalKneadMotorState = STATE_RUN_CLOCK ;//������˳ʱ�����У���խ�ķ�������
                    nKneadTurn = 1;
                }
            }
            break ;
        case KNEAD_STOP_AT_MED://��Ħ��ͣ�����е�λ��
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
                    nFinalKneadMotorState = STATE_RUN_CLOCK ;//����ͷ״̬��˳ʱ�룬ֹͣ����ʱ�ӵ�
                    nKneadTurn = 1;
                }
            }
            break ;
        case KNEAD_STOP_AT_MAX://��Ħ��ͣ���ڿ��λ��
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
            bKneadMotorInProcess = FALSE ;//���Զ���������ͬ�����������ֹͣ״̬��ֻҪ���������ֹͣĬ�ϸõ��Ҳ��ֹͣ
            break ;
        case KNEAD_ANTIRUN:nKneadTurn = 2;
            nCurKneadWidth = KNEAD_WIDTH_UNKNOWN ;
            nFinalKneadMotorState = STATE_RUN_UNCLOCK ;
            bKneadMotorInProcess = FALSE ;//���Զ���������ͬ�����������ֹͣ״̬��ֻҪ���������ֹͣĬ�ϸõ��Ҳ��ֹͣ
            break ;
        case KNEAD_RUN_STOP:
        case KNEAD_RUN_STOP_AT_MIN:////��Ħ��CLOCK����nȦ��ͣ����խ��λ��
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
        case KNEAD_RUN_STOP_AT_MED://��Ħ��CLOCK����nȦ��ͣ�����е�λ��
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
        case KNEAD_RUN_STOP_AT_MAX://��Ħ��CLOCK����nȦ��ͣ���ڿ��λ��
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
                /**************�ж��Ƿ񵽴���խ��*************************/
                if(bHasKneadWidthMinPulse == TRUE)//�������խ��λ��
                {
                    bHasKneadWidthMinPulse = FALSE ;
                    nstepcnt++;
                    if(nstepcnt>=2)
                    {nstepcnt=0;
                      nCurKneadWidth = KNEAD_WIDTH_MIN ;
                      nCurKneadMotorCycles++ ;       //����խλ�ü�1
                      Timer_Counter_Clear(C_TIME_RUBBING); 
                      nFinalKneadMotorState = STATE_IDLE ;
                    }
                }
                else
                {
                    nCurKneadWidth = KNEAD_WIDTH_UNKNOWN ;
                    nFinalKneadMotorState = STATE_RUN_UNCLOCK ; //��δ������խ����������ʱ��ת��
                    nKneadTurn = 2;
                }
                /*********************************************/
                break; 
            case 1:  //ͣ����խ��
                /**************�ж�ɲ��ʱ��************************/
                if(Timer_Counter(C_TIME_RUBBING,1)) 
                {
                    nCurKneadMotorCycles++ ;       //��1
                    nFinalKneadMotorState = STATE_RUN_CLOCK ; //����խ���������˳ʱ���˶�
                    nKneadTurn = 1;
                }
                else
                {
                    nFinalKneadMotorState = STATE_IDLE ;  //ͣ����խ��100ms 
                    
                }
                /*********************************************/
                break;
            case 2: 
                /**************�ж��Ƿ񵽴���խ��*************************/
                if(bHasKneadWidthMaxPulse == TRUE)
                {
                  nstepcnt++;
                  if(nstepcnt>=2)
                  {nstepcnt=0;
                  bHasKneadWidthMaxPulse = FALSE ;
                  nCurKneadWidth = KNEAD_WIDTH_MAX ;
                  nCurKneadMotorCycles++ ;       //�����λ�ü�1
                  Timer_Counter_Clear(C_TIME_RUBBING); 
                  nFinalKneadMotorState = STATE_IDLE ;
                  }
                }
                else
                {
                    nCurKneadWidth = KNEAD_WIDTH_UNKNOWN ;
                    nFinalKneadMotorState = STATE_RUN_CLOCK ; //��խ����˳ʱ��ת��
                    nKneadTurn = 1;
                }
                /*********************************************/
                break;  
            case 3: 
                /**************�ж�ɲ��ʱ��************************/
                if(Timer_Counter(C_TIME_RUBBING,1)) 
                {
                    nCurKneadMotorCycles++ ;       //��1
                    
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
                        nFinalKneadMotorState = STATE_RUN_UNCLOCK ; //����խ���������˳ʱ���˶�
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
            //˳ʱ�룺խ-��-��-��Ȧ����-խ     
            //��ʱ�룺��-��-խ-��Ȧ����-��          
            /*
            �걳��������  nCurKneadMotorCycles��ֵ����������    
            */
        case KNEAD_RUN_RUBBING:
            step = nCurKneadMotorCycles % 4;
            switch(step)
            {
            case 0: 
                /**************�ж��Ƿ񵽴���խ��*************************/
                if(bHasKneadWidthMinPulse == TRUE)//�������խ��λ��
                {
                    bHasKneadWidthMinPulse = FALSE ;
                    nCurKneadWidth = KNEAD_WIDTH_MIN ;
                    nCurKneadMotorCycles++ ;       //����խλ�ü�1
                    Timer_Counter_Clear(C_TIME_RUBBING); 
                    nFinalKneadMotorState = STATE_IDLE ;
                }
                else
                {
                    nCurKneadWidth = KNEAD_WIDTH_UNKNOWN ;
                    nFinalKneadMotorState = STATE_RUN_UNCLOCK ; //��δ������խ����������ʱ��ת��
                    nKneadTurn = 2;
                }
                /*********************************************/
                break; 
            case 1:  //ͣ����խ��
                /**************�ж�ɲ��ʱ��************************/
                if(Timer_Counter(C_TIME_RUBBING,1)) 
                {
                    nCurKneadMotorCycles++ ;       //��1
                    nFinalKneadMotorState = STATE_RUN_CLOCK ; //����խ���������˳ʱ���˶�
                    nKneadTurn = 1;
                }
                else
                {
                    nFinalKneadMotorState = STATE_IDLE ;  //ͣ����խ��100ms 
                    
                }
                /*********************************************/
                break;
            case 2: 
                /**************�ж��Ƿ񵽴���խ��*************************/
                if(bHasKneadWidthMaxPulse == TRUE)
                {
                    bHasKneadWidthMaxPulse = FALSE ;
                    nCurKneadWidth = KNEAD_WIDTH_MAX ;
                    nCurKneadMotorCycles++ ;       //�����λ�ü�1
                    Timer_Counter_Clear(C_TIME_RUBBING); 
                    nFinalKneadMotorState = STATE_IDLE ;
                }
                else
                {
                    nCurKneadWidth = KNEAD_WIDTH_UNKNOWN ;
                    nFinalKneadMotorState = STATE_RUN_CLOCK ; //��խ����˳ʱ��ת��
                    nKneadTurn = 1;
                }
                /*********************************************/
                break;  
            case 3: 
                /**************�ж�ɲ��ʱ��************************/
                if(Timer_Counter(C_TIME_RUBBING,1)) 
                {
                    nCurKneadMotorCycles++ ;       //��1
                    
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
                        nFinalKneadMotorState = STATE_RUN_UNCLOCK ; //����խ���������˳ʱ���˶�
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
    //ȷ�����������ٶ�
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
   //������£�ֻ���ڸ�������ʱ��ִ��һ��
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
        case WALK_LOCATE_ABSULATE:    //���е�����λ��
            nFinalWalkMotorLocate = nWalkMotorLocateParam ; 
            break ;
        case WALK_LOCATE_SHOULDER:    //���е����λ��
            nFinalWalkMotorLocate =  nShoulderPosition - 10;
            break ;
        case WALK_LOCATE_TOP:  //���е��϶��г�
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
        case WALK_LOCATE_SHOULDER_OR_ABSULATE:  //�ɼ粿λ�ú;��������еĽ�С�߾���
            if(nWalkMotorLocateParam > nShoulderPosition)
            {
                nFinalWalkMotorLocate = nShoulderPosition ;
            }
            else
            {
                nFinalWalkMotorLocate = nWalkMotorLocateParam ;
            }
            break ;
        case WALK_LOCATE_PARK: //ͣ���ڵ�ǰλ��
            WalkMotor_Control(STATE_WALK_IDLE,0);
            nCurActionStepCounter = 0 ;
            break ;

        case WALK_LOCATE_NeckMed: //�����м�λ��
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
        case WALK_LOCATE_PressNeck: //����λ��,�������
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
        //��֤���������λ
        if(nFinalWalkMotorLocate > by_TopPosition)
            nFinalWalkMotorLocate = by_TopPosition;   
    }//end if
    
    //�����ж� walk �г̣�bWalkMotorInProcess����ʱֹͣ 
    
    if(nWalkMotorLocateMethod == WALK_LOCATE_PARK)
    { //�ж��Ƿ񵽴�ͣ��ʱ��
        WalkMotor_Control(STATE_WALK_IDLE,0);//nCurActionStepCounter
        if((nWalkMotorLocateParam != MAX_PARK_TIME) && 
           (nCurActionStepCounter >= nWalkMotorLocateParam))//nCurActionStepCounter
        {
            bWalkMotorInProcess = FALSE ;
        }
    }
    else
    {
        if(nFinalWalkMotorLocate == 0)  //�г�����λ��Ϊ0
        {
            if(WalkMotor_Control(STATE_RUN_WALK_DOWN,0))
            {
                bWalkMotorInProcess = FALSE ;
            }
        }
        else if(nFinalWalkMotorLocate >= by_TopPosition) //�г�����λ��Ϊ���
        {
            if(WalkMotor_Control(STATE_RUN_WALK_UP,0))
            {
                bWalkMotorInProcess = FALSE ;
            }
        }
        else
        {   //�г�����λ��Ϊ����λ��
            if(WalkMotor_Control(STATE_RUN_WALK_POSITION,nFinalWalkMotorLocate))
            {
                bWalkMotorInProcess = FALSE ;
            }
        }
    }
    return 0;
}


/*
//    nWalkMotorLocateParam=��λ���꣬��ͣ��ʱ��                                            
unsigned char WalkMotorControl(unsigned char nWalkMotorLocateMethod,unsigned short nWalkMotorLocateParam)
{
   //������£�ֻ���ڸ�������ʱ��ִ��һ��
    unsigned short by_TopPosition = TOP_POSITION;
    if(bUpdateLocate == TRUE)//�������ߵ�����ߵ��������±�־����λʱ����һ������
    {
        bUpdateLocate = FALSE ;
        //nWalkMotorLocateState = nWalkMotorLocateMethod;
        switch(nWalkMotorLocateMethod)//���ߵ����λ��ʽ���Զ������������ߵ����λ��ʽ
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
        case WALK_LOCATE_ABSULATE:    //���е�����λ��
            nFinalWalkMotorLocate = nWalkMotorLocateParam ; 
            break ;
        case WALK_LOCATE_SHOULDER:    //���е����λ��
            nFinalWalkMotorLocate =  nShoulderPosition - 10;//nShoulderPosition=ʵ�ʼ�⵽�ļ粿λ��
            break ;
        case WALK_LOCATE_TOP:  //���е��϶��г�
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
        case WALK_LOCATE_SHOULDER_OR_ABSULATE:  //�ɼ粿λ�ú;��������еĽ�С�߾���
            if(nWalkMotorLocateParam > nShoulderPosition)
            {
                nFinalWalkMotorLocate = nShoulderPosition ;
            }
            else
            {
                nFinalWalkMotorLocate = nWalkMotorLocateParam ;
            }
            break ;
        case WALK_LOCATE_PARK: //ͣ���ڵ�ǰλ��
            WalkMotor_Control(STATE_WALK_IDLE,0);
            nCurActionStepCounter = 0 ;
            break ;

        case WALK_LOCATE_NeckMed: //�����м�λ��
            if(nShoulderPosition >= by_TopPosition - Med_NECK_LENGTH)
            {
                nFinalWalkMotorLocate = by_TopPosition ;
            }
            else
            {
                nFinalWalkMotorLocate = nShoulderPosition + Med_NECK_LENGTH ;
            }
            
            break;
        case WALK_LOCATE_PressNeck: //����λ��,�������
            nFinalWalkMotorLocate = nShoulderPosition;	//10 ;
            break;
        }//end switch
        //��֤���������λ
        if(nFinalWalkMotorLocate > by_TopPosition)
            nFinalWalkMotorLocate = by_TopPosition;   
    }//end if
    
    //�����ж� walk �г̣�bWalkMotorInProcess����ʱֹͣ 
    
    if(nWalkMotorLocateMethod == WALK_LOCATE_PARK)
    { //�ж��Ƿ񵽴�ͣ��ʱ��
        WalkMotor_Control(STATE_WALK_IDLE,0);
        if((nWalkMotorLocateParam != MAX_PARK_TIME) && 
           (nCurActionStepCounter >= nWalkMotorLocateParam))//100ms TIMER
        {
            bWalkMotorInProcess = FALSE ;
        }
    }
    else//����ߵ�Ŀ��λ�ú�ֹͣ����
    {
        if(nFinalWalkMotorLocate == 0)  //�г�����λ��Ϊ0
        {
            if(WalkMotor_Control(STATE_RUN_WALK_DOWN,0))
            {
                bWalkMotorInProcess = FALSE ;
            }
        }
        else if(nFinalWalkMotorLocate >= by_TopPosition) //�г�����λ��Ϊ���
        {
            if(WalkMotor_Control(STATE_RUN_WALK_UP,0))
            {
                bWalkMotorInProcess = FALSE ;
            }
        }
        else
        {   //�г�����λ��Ϊ����λ��
            if(WalkMotor_Control(STATE_RUN_WALK_POSITION,nFinalWalkMotorLocate))//��ﵽ��Ŀ��λ�ú�ֹͣ����
            {
                bWalkMotorInProcess = FALSE ;//���ߵ��ֹͣ����
            }
        }
    }
    return 0;
}
*/





void startBodyDetect(void)
{
    //������
    nFinalWalkMotorLocate = TOP_POSITION;
    //�粿������
    //bodyDetectSuccess = 0;
    //��ⲽ������
    //shoulderPositionScanStep = 0;
}





/*
#define BACK_SUB_MODE_KNEAD			7
#define BACK_SUB_MODE_KNOCK			8
#define BACK_SUB_MODE_PRESS			9
#define BACK_SUB_MODE_WAVELET			10// //����ͬ��
#define BACK_SUB_MODE_SOFT_KNOCK		11////ߵ��
#define BACK_SUB_MODE_MUSIC			12////���ɰ�Ħ
{BACK_SUB_MODE_KNEAD...BACK_SUB_MODE_MUSIC}֮���һ�ְ�Ħ�ַ�
*/

void walkRefreshUp(unsigned char key)
{
  if(nKeyBackLocate == LOCATE_NONE)
    {
      nKeyBackLocate = LOCATE_FULL_BACK ;
    }
  if(nKeyBackLocate == LOCATE_FULL_BACK)		//ȫ��
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
  if(nKeyBackLocate == LOCATE_FULL_BACK)		//ȫ��
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
  //�������ߵ��
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
  
  
  //����������
  bKneadMotorInProcess = TRUE ;
  nKneadMotorControlParam1 = AutoDirector.nKneadMotorState ;
  nKneadMotorControlParam2 = AutoDirector.nKneadMotorCycles ;
  //���ô������
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
  
  
  
  
  //����3D������� 
  
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
    if(st_Stretch.active)//������ʱ��ִ�����ȳ���ĳ������飬��ִ���Զ���������1
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
            //������ȵ�ǰ�Ķ�����ɣ���ǿ�н�InProcess�ó�FALSE������ɳ嶥
        }
        else
        {
          if((bWalkMotorInProcess == FALSE) &&
             (bKneadMotorInProcess == FALSE) &&
               (bKnockMotorInProcess == FALSE)&&
                 (b3D_MotorInProcess == FALSE))
          {
            nCurActionStep++ ; //�Զ�����������
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
          //ÿ�θ���������Ҫ���µı���
          //���ߺ��û���ﹲ��һ��ʱ������������û�������ʼʱ�� ������ﲻ���Լ���������ֹͣ�ڵ�ǰλ��
          nCurActionStepCounter = 0 ;//��ǰ����ʱ����������߻��û�������ʱ�������  ,���������߻��û�ʱ�����,����������������ͬʱ���м���
          nCurShoulderAdjustCounter = 0 ;
          if(!((nCurSubFunction == BACK_SUB_MODE_SOFT_KNOCK) && (AutoDirector.nSubFunction == BACK_SUB_MODE_SOFT_KNOCK)))
          {
            nCurKnockRunStopCounter = 0 ;//ߵ������������
          }
          nCurKneadMotorCycles = 0 ;//����Ȧ������������
          refreshAutoDirector();
        }
      break;
    case BACK_MAIN_MODE_AUTO:
    case BACK_MAIN_MODE_3D:    
 //  case BACK_MAIN_MODE_CLOUD  :
      if(bBackAutoModeInit == TRUE)//���û�ѡ���Զ�����Ħ��ʼʱ����������ĳ�ʼ����־λ
        {
            bBackAutoModeInit = FALSE;
            
            if(nBackSubRunMode < BACK_SUB_MODE_3D1)//ѡ��5���Զ�����
            {
              nMaxActionStep = BACK_AUTO_STEPS[nBackSubRunMode];//�Զ��������еĲ�������Ҫ��AutoFunction0����ĳ���
              nStartActionStep = BACK_AUTO_START_STEP[nBackSubRunMode];//�ӵ�0����ʼִ��
            }
            else 
            {
              
                if(nBackSubRunMode<=BACK_SUB_MODE_DEMO)
                {
                   nMaxActionStep = BACK_AUTO_STEPS[nBackSubRunMode-BACK_SUB_MODE_3D1+6];//a[6-8]Ϊ3��3D����
               
                   nStartActionStep = BACK_AUTO_START_STEP[nBackSubRunMode-BACK_SUB_MODE_3D1+6];
                }
                else if(nBackSubRunMode == BACK_SUB_MODE_6MIN_DEMO  )
                {
                   nMaxActionStep = BACK_AUTO_STEPS[BACK_SUB_MODE_AUTO_1];//a[6-8]Ϊ3��3D����
                   nStartActionStep = BACK_AUTO_START_STEP[BACK_SUB_MODE_AUTO_1];
                }
                else
                {
                    nMaxActionStep = BACK_CLOUD_STEPS[nBackSubRunMode-BACK_SUB_MODE_CLUDE_AUTO_0];//a[6-8]Ϊ3��3D����
                    nStartActionStep = BACK_CLOUD_START_STEP[nBackSubRunMode-BACK_SUB_MODE_CLUDE_AUTO_0];
                }
            } 
            
            
            nMaxActionStep = BACK_AUTO_STEPS[nBackSubRunMode];//�Զ��������еĲ�������Ҫ��AutoFunction0����ĳ���
            nStartActionStep = BACK_AUTO_START_STEP[nBackSubRunMode];//�ӵ�0����ʼִ��
            
            bGetNextActionStep = TRUE;
            nCurActionStep = 0;
            nStretchStep = 0;
            nCur3D_MotorStopCounter = 0;//3D�Զ����򵽴�Ŀ��λ�õ�ֹͣʱ��
            //������ȵ�ǰ�Ķ�����ɣ���ǿ�н�InProcess�ó�FALSE������ɳ嶥
        }
        else////������ȵ�ǰ�Ķ�����ɣ���ǿ�н�InProcess�ó�FALSE������ɳ嶥
        {

          //���е��������ɺ�Ż�����Զ�������һ������Ķ�����
            if((bWalkMotorInProcess == FALSE) &&
                    (bKneadMotorInProcess == FALSE) &&
                    (bKnockMotorInProcess == FALSE)&&
                      (b3D_MotorInProcess == FALSE))
            {
                nCurActionStep++ ; //�Զ�����������
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
                    //�ص���һ������״̬
                    nTargetMassagePosition = MASSAGE_OPTIMAL2_POSITION;
                    bMassagePositionUpdate = TRUE;    
                    
                    nCurActionStep = 9;  
                }             
                
                
                
                
                
                if(nCurActionStep >= nMaxActionStep)
                {
                  nCurActionStep = nStartActionStep ;
                }

                
            
                bGetNextActionStep = TRUE ;///���е��������ɺ�Ż�����Զ�������һ������Ķ�����
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
            /*���cָ��*/
            if((bTapping ==0)&&(AutoDirector.nSubFunction == BACK_SUB_MODE_PRESS)&&(nWalkMotorControlParam1 == WALK_LOCATE_PARK))
            {
              //AutoDirector.nSubFunction = BACK_SUB_MODE_PRESS;
              switch(pressstep)
              {
              case 0:
              default:
                
                b3D_MotorInProcess = TRUE;
                n3D_MotorControlState = _3D_MANUAL_AUTO_VECTOR;//
                n3D_MotorControlPosition = 2;//3D��о�ص����ȵ�һ��
                n3D_MotorControlSpeed = 4;//
                n3D_MotorControlStopTime = 0;//
                pressstep++;
                presstime=0;
                break;
              case 1:
                if(b3D_MotorInProcess == false)//ͣ4��
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
                switch(nKeyAxisStrength)//�������ȣ��趨�����X��λ��
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
                if(b3D_MotorInProcess == false)//ͣ4��
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
      //refreshAutoDirector();//��bWalkMotorInProcess=true,bKneadMotorInProcess=true,bKnockMotorInProcess=true,b3D_MotorInProcess=true����о��ʼ�����Զ�����ʼ����
      //������һ������ʱ �������������־λ��1��bWalkMotorInProcess=true,bKneadMotorInProcess=true,bKnockMotorInProcess=true,b3D_MotorInProcess=true��
        if(bGetNextActionStep == TRUE)//��ʼ��һ�����еı�־λ�� //���е��������ɺ�Ż�����Զ�������һ������Ķ�����
        {
          bGetNextActionStep = FALSE ;
     //     printf("step:%d,time:%d\n",nCurActionStep,Data_Get_TimeSecond());
          
           n3Dpointturn ++;
          switch(nBackSubRunMode)
          {
          
          case BACK_SUB_MODE_AUTO_0:
           AutoDirector = AutoFunction0[nCurActionStep] ;//���Զ�����ṹ�������ֵ�����ṹ�����                   
            
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
          //ÿ�θ���������Ҫ���µı���
          nCurActionStepCounter = 0 ;//�Զ������� ��ǰ����ʱ����������߻��û�ʱ�������
          nCurShoulderAdjustCounter = 0 ;
          if(!((nCurSubFunction == BACK_SUB_MODE_SOFT_KNOCK) && (AutoDirector.nSubFunction == BACK_SUB_MODE_SOFT_KNOCK)))
          {
            nCurKnockRunStopCounter = 0 ;//ߵ������������
          }
   
          nCurKneadMotorCycles = 0 ;//����Ȧ������������
          refreshAutoDirector();//��bWalkMotorInProcess=true,bKneadMotorInProcess=true,bKnockMotorInProcess=true,b3D_MotorInProcess=true����о��ʼ�����Զ�����ʼ����
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
            //ÿ�θ���������Ҫ���µı���
            nCurActionStepCounter = 0 ;//��ǰ����ʱ����������߻��û�ʱ�������
            nCurKnockRunStopCounter = 0 ;//ߵ������������
            nCurKneadMotorCycles = 0 ;//����Ȧ������������
            nCur3D_MotorStopCounter = 0;//3D�������Ŀ��λ�õ�ֹͣʱ��
            nCurSubFunction = ManualDirector[nCurActionStep].nSubFunction ;
            nCurKneadKnockSpeed = ManualDirector[nCurActionStep].nKneadKnockSpeed ;
            //�������ߵ��
            bWalkMotorInProcess = TRUE ;//�������ߵ���������б�־λ��ֹͣ����ʱ��λΪ0
            bUpdateLocate = TRUE ;//���ߵ��������±�־����λʱ����һ������
            nWalkMotorControlParam1 = ManualDirector[nCurActionStep].nWalkMotorLocateMethod ;
            nWalkMotorControlParam2 = ManualDirector[nCurActionStep].nWalkMotorLocateParam ;
            //����������
            bKneadMotorInProcess = TRUE ;
            nKneadMotorControlParam1 = ManualDirector[nCurActionStep].nKneadMotorState ;
            nKneadMotorControlParam2 = ManualDirector[nCurActionStep].nKneadMotorCycles ;
            //���ô������
            bKnockMotorInProcess = TRUE ;
            nKnockMotorControlParam1 = ManualDirector[nCurActionStep].nKnockMotorState ;
            nKnockMotorControlParam2 = ManualDirector[nCurActionStep].nKnockMotorRunTime ;
            nKnockMotorControlParam3 = ManualDirector[nCurActionStep].nKnockMotorStopTime ;
            /*
            //����3D���
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
            n3D_MotorControlPosition = 2;//3D��о�ص����ȵ�һ��
            n3D_MotorControlSpeed = 4;//
            n3D_MotorControlStopTime = 0;//
            pressstep++;
            presstime=0;
            break;
          case 1:
            if(b3D_MotorInProcess == false)//ͣ4��
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
            switch(nKeyAxisStrength)//�������ȣ��趨�����X��λ��
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
            if(b3D_MotorInProcess == false)//ͣ4��
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
    
    if(bKeyFlexOut == TRUE)//����С���찴��
    {
      FlexMotorSetDisable();//  FlexMotorEnable = false;
      if(Flex_ControlOut(FLEX_MOTOR_CURRENT_2A))//(FLEX_MOTOR_CURRENT_3A))
      {

        nBuzzerMode = BUZZER_MODE_FAST ;
        bSendBuzzerMode = TRUE ;//���ͷ���������ʹ���źţ��ֿذ�
        bBlueToothSendBuzzerMode = TRUE;//���ͷ���������ʹ���źţ�ƽ��
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
    if((bKeyFlexOut == FALSE) && (bKeyFlexIn == FALSE))//û��ѹ������������
    {
      //  Flex_SetDirection(FLEX_MOTOR_STOP);
        Flex_ControlStop();
      
        retval = 0;
    }
    return retval;
}



//С������綯�׿��Ƴ���
void Main_LegPad_Proce(void)
{   
    LegMotor_Proce();//С��������ʱ �����������ͨ��10ms��ʱ����,
    if(st_Stretch.active) return;  //������˳�����Ч���˳�
    if(bMassagePositionUpdate) return; //��ǿ�����ð�Ħλ��ʱҲ�����С�ȴ����ʵ�ǿ�����ð�Ħλ��ʱ��ִ�д˺�������

   if((bKeyLegPadUp == TRUE)||(bKeyLegPadDown == TRUE))
   {
     Flex_ControlStop();
   }



    if(bLegPadLinkage == FALSE) //С�ȵ���������ʱ������ǰ���綯�׵�λ��
    {
        if(bKeyLegPadUp == TRUE)
        {
            FlexMotorSetEnable();
            if(LegMotor_Control(STATE_RUN_LEG_UP) != LEG_RUN)//С���������ߣ�ֱ�������г̿���,��ʱbeep������
            {
                nBuzzerMode = BUZZER_MODE_FAST ;
                bSendBuzzerMode = TRUE ;
            }
            else  //�����г̿���
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
    else  //������С��������ǰ���綯�ױ�������ǰλ��
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
  {9,2,STRETCH_GO_DOWN},//4��������   9��λΪ����
  {6,2,STRETCH_GO_DOWN},//3��������
  {3,2,STRETCH_GO_DOWN},//3�������� 
};


void Valve_StretchControlProce(void)
{
  
  
  bool bStatus;//,bStatus_2;
  int legFlag,BackFlag,FlexFlag;//,SlideFlag;
  static int stretchMode = STRETCH_GO_DOWN;
  if(!st_Stretch.active) //�����ȿ���ʱ�̲Ż������Զ���������1�ĳ���
  {
    unsigned int RunTime = Data_Get_TimeSecond();//ʱ�䵹��ʱ
    unsigned int Minutes,i;
    StretchProgramStruct const *p;
    unsigned int totalTimes;
    
    if(RunTime%60 != 0)  return; //0�뿪ʼ����
    
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
      totalTimes = sizeof(stretchProgram_20)/sizeof(StretchProgramStruct);//totalTimes=�趨���ٸ������Ƴ� =7
    } 
    Minutes = RunTime/60; //��ȡ��ǰ������
    
    if(Minutes == 0) 
    {
      st_Stretch1.times = 0;
      st_Stretch.times = 0;
      return; //���һ����ֹͣ����
    }
    //һ�������Ƴ�Ϊһ�����Ƚṹ��
    for(i=0;i<totalTimes;i++)//totalTimes=�趨���ٸ������Ƴ�  totalTimes=3�������Ƴ�=10��������  ,����10��������Ϊ3�������Ƴ�
    {
      if((Minutes == (p+i)->time)&&((Minutes==6)||(Minutes==9)||(Minutes==29)||(Minutes==25))||(Minutes==19)||(Minutes==15)) // //ÿ���Ƴ��µ����˳���ִ��ʱ��   (p+i)->time=��λΪ����
      {//���˿�ʼ����ʱ���־λ
        st_Stretch.active = TRUE;
        st_Stretch.init = TRUE; 
        stretchMode = (p+i)->mode;//����ģʽΪ�г̿���
        st_Stretch.times = (p+i)->times;//һ���Ƴ̵����˴��� ,��һ�����Ƚṹ������ȴ���
        st_Stretch1.times=0;
        
        
        break;
      }
      else if(Minutes ==  (p+i)->time&&((Minutes==3)||(Minutes==22)||(Minutes==12))) // //ÿ���Ƴ��µ����˳���ִ��ʱ��   (p+i)->time=��λΪ����
      {//���˿�ʼ����ʱ���־λ
        st_Stretch.active = TRUE;
        st_Stretch1.init = TRUE; 
        stretchMode = (p+i)->mode;//����ģʽΪ�г̿���
        st_Stretch1.times = (p+i)->times;//һ���Ƴ̵����˴��� ,��һ�����Ƚṹ������ȴ���
        st_Stretch.times=0;
        
        
        break;
      }
    }
    if(!st_Stretch.active)  return;
  }
  if(st_Stretch1.init)
  {
    nStretchStep = 0;
    st_Stretch1.step = 0;//���˳�����
    st_Stretch.timer = 0;////���˳����ʱ��ʱ������λ0.1s
    st_Stretch1.init = FALSE;
  }
  if(st_Stretch.init)
  {
    nStretchStep = 0;
    st_Stretch.step = 0;//���˳�����
    st_Stretch.timer = 0;////���˳����ʱ��ʱ������λ0.1s
    st_Stretch.init = FALSE;
    
    
  }
  if(st_Stretch1.times > 0)//һ���Ƴ̵����˴��� һ��Ϊ3��,һ�����Ƚṹ������ȴ���������10�������ȣ���һ�������Ƴ�Ϊ4���ڶ��������Ƴ�Ϊ3
  {
    switch(st_Stretch1.step)
    {
    case 0:
      Valve_SetStretchChargeATOUT(1);//�㲿�������ȳ���	
      
      nStretchStep = 0;
      AutoDirector = AutoFunctionStretch[nStretchStep] ;////��Ħ�ε�������λ�ã���о������ʹ� ,���ߵ��������͵�
      refreshAutoDirector();
      st_Stretch1.step++;
      break;
      
    case 1:
      
      FlexFlag = Flex_ControlOut(FLEX_MOTOR_CURRENT_1D5A);//С����  FlexFlag = Flex_ControlOut(FLEX_MOTOR_CURRENT_1D5A);
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
        Valve_SetStretchChargeATOUT(1);//�㲿�������ȳ���
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
      
      //    if(st_Stretch.timer >= C_STRETCH_CHARGE_TIME)//100*60=6sec ,������ case 5֮��ѭ����ֱ������ʱ�����
      
      if(stretchMode == STRETCH_GO_OUT)
      {
        if(st_Stretch.timer >= C_STRETCH_CHARGE_TIME_OUT)//100*60=6sec ,������ case 5֮��ѭ����ֱ������ʱ�����
        {  //�ж��Ƿ��Ѵ����ʱ��
          st_Stretch1.step++;
          st_Stretch.timer = 0;
        }     
      }
      else
      {
        if(st_Stretch.timer >= C_STRETCH_CHARGE_TIME)//100*60=6sec ,������ case 5֮��ѭ����ֱ������ʱ�����
        {  //�ж��Ƿ��Ѵ����ʱ��
          st_Stretch1.step++;
          st_Stretch.timer = 0;
        }           
        
      }
      
      break;
    case 4:
      nStretchStep = 1;
      AutoDirector = AutoFunctionStretch[nStretchStep] ;////��Ħ�ε�������λ�ã���о������ʹ� ,���ߵ��������͵�
      refreshAutoDirector();
      
      legFlag = LegMotor_Control(STATE_RUN_LEG_DOWN);
      BackFlag = BackMotor_Control(STATE_RUN_BACK_DOWN);
      FlexFlag = Flex_ControlIn(FLEX_MOTOR_CURRENT_1D5A);//��Ħ��С��������  5cm,�п��������г̿���
      
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
      st_Stretch1.times--;//һ��,һ�����Ƚṹ������ȴ���������10�������ȣ���һ�������Ƴ�Ϊ4�Σ��ӵ�9���ӿ�ʼ
      //  Valve_SetStretchUp();  //�����رգ����������������������ֳ����⣩
      Valve_SetStretchChargeATOUT(1);//�㲿�������ȳ���
      
      nStretchStep = 0;
      if(st_Stretch1.times == 0)
      {
        // nZLB_RunState = 1;            //���˶������ǿ�ƻص���һ����������
        nTargetMassagePosition =MASSAGE_OPTIMAL_POSITION;// MASSAGE_OPTIMAL2_POSITION; 
        bMassagePositionUpdate = TRUE;
        //bZLBMotorRunFlag = TRUE;
        st_Stretch1.bBackLegFlag = FALSE;
        st_Stretch.timer = 0;
        st_Stretch.active = FALSE;
        bBackAutoModeInit = true;  //Ϊ�˱����о���ֲ����о��Ħ��ͷ��ʼ
      }
      
      break;
    default:break;
    
    }
    
    
  }
  else if(st_Stretch.times > 0)//һ���Ƴ̵����˴��� һ��Ϊ3��,һ�����Ƚṹ������ȴ���������10�������ȣ���һ�������Ƴ�Ϊ4���ڶ��������Ƴ�Ϊ3
  {
    switch(st_Stretch.step)// //���˳�����    //�����ȿ���ʱ�̲Ż������Զ���������1�ĳ���
    {
    case  0:   //��о������ʼ��
      //if(!limt_backmoto)
      //	{
      nStretchStep = 0;
      AutoDirector = AutoFunctionStretch[nStretchStep] ;////��Ħ�ε�������λ�ã���о������ʹ� ,���ߵ��������͵�
      refreshAutoDirector();
      //	}
      st_Stretch.step++;
      st_Stretch.timer = 0; 
      break;
    case  1:  //��Ħ�ε����λ��
      //Valve_SetStretchUp();  //�����������
      //    LegKnead_Control(LEG_KNEAD_SPEED_STOP,LEG_KNEAD_TO_IN);//8600SС�Ȱ���ƣ�8305T û��ʹ��С������������������
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
      
      
      //   FlexFlag = Flex_ControlIn(FLEX_MOTOR_CURRENT_15A);//С����
      //���������棬С�������棬�����������������
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
        st_Stretch.step ++;   //С�ȺͿ����綯�׵�λ����ʼ�������ȳ���
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
        //       FlexMotorSetEnable(); //�ñ�־λ���Զ��ҽ� ����
        FlexMotorSetDisable();
        break;
      }
      break;
      
    case 3:
      
      FlexFlag = Flex_ControlOut(FLEX_MOTOR_CURRENT_5A);//��Ħ��С��������  5cm,�п��������г̿���
      nStretchStep = 1;
      AutoDirector = AutoFunctionStretch[nStretchStep] ;///ͣ������0λ��
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
        //Valve_SetStretchChargeATOUT(1);//�㲿�������ȳ���
        if(bRollerEnable)
        {
          RollerMotor_Control(ROLLER_SPEED_SLOW,0);
        }
      }
      //Valve_SetStretchChargeATOUT(1);//�㲿�������ȳ���
      //Valve_SetStretchCharge_FOOT_THIGH(0);
      Valve_SetStretchCharge_FOOT_THIGH_SHOULD(0);
      break;
      
      
    case 4:
      
      FlexMotorSetEnable();
      Valve_SetStretchChargeATOUTFootHeelOFF();//�ͷ��㲿����
      
      st_Stretch.step++;
      break;
      
      
      
    case 5:  //�����綯С��λ��
      BackMotor_Control(STATE_BACK_IDLE);
      if(stretchMode == STRETCH_GO_OUT)
      {
        st_Stretch.step++;
        break;
      }
      if(FlexMotorGetEnable() == false)
      {
        
        st_Stretch.step++;
        st_Stretch.timer = 0;  //�������ҳ����򿪣��������ҹر�  ������ʼ��ʱ���� 0
        
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
      st_Stretch.timer = 0;  //�������ҳ����򿪣��������ҹر�  ������ʼ��ʱ���� 0
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
      AutoDirector = AutoFunctionStretch[nStretchStep] ;///ͣ������0λ��
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
      //    if(st_Stretch.timer >= C_STRETCH_CHARGE_TIME)//100*60=6sec ,������ case 5֮��ѭ����ֱ������ʱ�����
      
      if(stretchMode == STRETCH_GO_OUT)
      {
        if(st_Stretch.timer >= C_STRETCH_CHARGE_TIME_OUT)//100*60=6sec ,������ case 5֮��ѭ����ֱ������ʱ�����
        {  //�ж��Ƿ��Ѵ����ʱ��
          st_Stretch.step++;
          st_Stretch.timer = 0;
        }     
      }
      else
      {
        if(st_Stretch.timer >= C_STRETCH_CHARGE_TIME)//100*60=6sec ,������ case 5֮��ѭ����ֱ������ʱ�����
        {  //�ж��Ƿ��Ѵ����ʱ��
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
      AutoDirector = AutoFunctionStretch[nStretchStep] ;////��Ħ�ε�������λ�ã���о������ʹ� ,���ߵ��������͵�
      refreshAutoDirector();
      if(stretchMode == STRETCH_GO_DOWN)   //С���������е���͵� 
      { 
        Valve_SetStretchCharge(0); //�ص��粿����
        //     LegKnead_Control(LEG_KNEAD_SPEED_MID,LEG_KNEAD_TO_IN);
        if(LEG_STOP_AT_DOWN == LegMotor_Control(STATE_RUN_LEG_DOWN))
        {
          bStatus = 1;
          nStretchStep = 2;
          AutoDirector = AutoFunctionStretch[nStretchStep] ;////��Ħ�ε�������λ�ã���о������ʹ� ,���ߵ��������͵�
          //limt_backmoto=1;
          
          refreshAutoDirector();
        }
        else
        {
          bStatus = 0;
        }
      }  
      else  //STRETCH_GO_OUT=������������쵽���λ��
      {
        
        
        Valve_SetStretchChargeOut(0);          
        FlexFlag = Flex_ControlOut(FLEX_MOTOR_CURRENT_2A);//if(FlexFlag || st_Stretch.timer > 100)///����ʱ�����  100ms  counter          
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
      
      
      
      //�����������Ժ�һֱΪ����״̬��ֱ���ر������Ż����
    case 12:    //��ѹʱ��  ,С���쵽���λ�û�С�����µ����λ�ã���ʼ���Ҽ�ѹ
      Valve_SetStretchHold();//�������ң��ź����С���ϲ࣬С���²࣬С���ϵף�С���µף�������������������ѹ
      st_Stretch.step++;
      st_Stretch.timer = 0;
      break;
    case 13:   //��ѹʱ�䳤������̫���˲����
      //   if(st_Stretch.timer >= (Valve_GetAirBagStrength()*10))//��ѹʱ�������������������2*10=20 ��100ms*20=2sec,��������ͨ���ⲿ������ã�����5����������
      //    if(st_Stretch.timer >= (Valve_GetAirBagStrength()*6))//��ѹʱ�������������������2*10=20 ��100ms*20=2sec,��������ͨ���ⲿ������ã�����5����������
      if(st_Stretch.timer >= (Valve_GetAirBagStrength()*20))//��ѹʱ�������������������2*10=20 ��100ms*20=2sec,��������ͨ���ⲿ������ã�����5����������
      {  //�ж��Ƿ��Ѵ��ѹʱ��
        st_Stretch.step++;
        st_Stretch.timer = 0;
        RollerMotor_Control(ROLLER_SPEED_STOP,0);
        //       LegKnead_Control(LEG_KNEAD_SPEED_STOP,LEG_KNEAD_TO_IN);
      }
      break;
    case 14:
      st_Stretch.step = 0;
      st_Stretch.timer = 0;
      st_Stretch.times--;//һ��,һ�����Ƚṹ������ȴ���������10�������ȣ���һ�������Ƴ�Ϊ4�Σ��ӵ�9���ӿ�ʼ
      Valve_SetStretchUp();  //�����رգ����������������������ֳ����⣩
      nStretchStep = 0;
      if(st_Stretch.times == 0)
      {
        
        // nZLB_RunState = 1;            //���˶������ǿ�ƻص���һ����������
        nTargetMassagePosition =MASSAGE_OPTIMAL_POSITION;// MASSAGE_OPTIMAL2_POSITION; 
        bMassagePositionUpdate = TRUE;
        //bZLBMotorRunFlag = TRUE;
        st_Stretch.bBackLegFlag = FALSE;
        st_Stretch.timer = 0;
        st_Stretch.active = FALSE;
        bBackAutoModeInit = true;  //Ϊ�˱����о���ֲ����о��Ħ��ͷ��ʼ
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
  {8,4,STRETCH_GO_DOWN},//4��������   9��λΪ����
  {4,3,STRETCH_GO_DOWN},//3��������    
};

/*

1����100%
2����100%
3����100%
4����100%
5����100%



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
    
    if(RunTime%60 != 0)  return; //0�뿪ʼ����
        
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
    Minutes = RunTime/60; //��ȡ��ǰ������
   
    if(Minutes == 0) 
    {
      st_Stretch.times = 0;
      return; //���һ����ֹͣ����
    }
    
    for(i=0;i<totalTimes;i++)
    {
       
      if((bShoulderOK == 1)  &&(st_Stretch.active == FALSE) )
      {
          /*
        if(nBackSubRunMode == BACK_SUB_MODE_6MIN_DEMO) // //���ʰ�Ħ�����칦��
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
    case  0:   //��о������ʼ��   ��о����
      nStretchStep = 0;
      AutoDirector = AutoFunctionStretch[nStretchStep] ;
      refreshAutoDirector();
      st_Stretch.step++;
      legFlag = FALSE;
      BackFlag = FALSE;
      st_Stretch.timer = 0; 
      break;
    case  1:  //��Ħ�νǶȣ�С��������λ������ָ��λ��
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
    case 2: // �ȴ�ʱ����о��������  3D��ʼʹ��
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
        //FlexMotorSetEnable();//------------------------------�ҽ�
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
      if(nFlexStatus&0x04) //�ŵ������ŵ׿����ź�
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
        if(st_Stretch.timer > (unsigned char)(s_AirTimeadj*120)) //----------------------�ҽź󣬳���ʱ��׼��ǰ��
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
      //�뿪���ģ�ֹͣ
      FlexFlag = Flex_ControlOut(FLEX_MOTOR_CURRENT_1D5A);//Flex_ControlOut(FLEX_MOTOR_CURRENT_5A);
      if(FlexFlag )//&&(bWalkMotorInProcess == FALSE) )//|| st_Stretch.timer > 100)
      {
        Flex_SetDirection(FLEX_MOTOR_STOP);
        //FlexMotor_Control(STATE_FLEX_IDLE, FLEX_SPEED_FAST, FLEX_CURRENT_3A);
        st_Stretch.step++;
        st_Stretch.timer = 0;
      }
      
      if( (st_Stretch.timer>35)&&(bHaveMan == FALSE) )//---------------------------------����ֹͣ
      {
        Flex_SetDirection(FLEX_MOTOR_STOP);
        st_Stretch.step++;
        st_Stretch.timer = 0;
      } 
      //if((nStretchFlextimer > 10)&&( (nFlexStatus&0x04) == 0)&&(bHaveMan))//---------------------------------����ֹͣ(nFlexStatus&0x04)
      if( ((nFlexStatus&0x04) == 0)&&(bHaveMan))//---------------------------------����ֹͣ(nFlexStatus&0x04)
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

      if(bHaveMan == TRUE) //�ŵ������ŵ׿����ź�
      {
        FlexFlag = Flex_ControlIn(FLEX_MOTOR_CURRENT_2A);//1D5A);//20170515
        if(st_Stretch.timer>20) //--����ֹͣ
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
      Valve_SetStretchCharge_FOOT_THIGH_LEG(0);//ȫ��
       if(st_Stretch.timer > (unsigned char)(s_AirTimeadj*70)) //---------------------------------��ǰ���뿪���Ĵ���ѹ
        {
          st_Stretch.step++;
          st_Stretch.timer = 0;
          
        }
      //st_Stretch.step++;
      break;
   
    case 8:
     // if(bWalkMotorInProcess == FALSE)
      
      //��о��
      {
        nStretchStep = 1;
        AutoDirector = AutoFunctionStretch[nStretchStep] ;
        refreshAutoDirector();
        st_Stretch.step++;st_Stretch.timer = 0;
      }
      break;
    case  9: 
        Valve_SetStretchCharge_FOOT_THIGH_LEG_SHOULD(0);//ȫ��

      if(st_Stretch.timer >= 40)//-------------------------��ѹʱ��
      {  //�ж��Ƿ��Ѵ����ʱ��
        st_Stretch.step++;
        st_Stretch.timer = 0;
      }
      break;
     

    case 10: 
      //bRoll_InStretch	= FALSE;
      nStretchStep = 2;
      AutoDirector = AutoFunctionStretch[nStretchStep] ;
      refreshAutoDirector();
      
      //�ж��Ƿ��Ѵ����ʱ��
      st_Stretch.step++;
      st_Stretch.timer = 0;
      
      break;
    case 11:  //������Ͷ�        

      Valve_SetStretchCharge_FOOT_THIGH_LEG_SHOULD(0);//--------��������
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
    case 12:    //��ѹʱ��
        
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
      if(st_Stretch.timer >= (Valve_GetAirBagStrength()*(unsigned char)(s_AirTimeadj*20)))//----------���²���ѹʱ��
      {  //�ж��Ƿ��Ѵ��ѹʱ��
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
          // nZLB_RunState = 1;            //���˶������ǿ�ƻص���һ����������
          nTargetMassagePosition = MASSAGE_OPTIMAL2_POSITION;//MASSAGE_RESET_POSITION;// MASSAGE_RESET_POSITION;//
          bMassagePositionUpdate = TRUE;
          //bZLBMotorRunFlag = TRUE;
          st_Stretch.bBackLegFlag = FALSE;
          st_Stretch.timer = 0;
          st_Stretch.active = FALSE;SetStretchingEnable(0);
          //bBackAutoModeInit = true;  //Ϊ�˱����о���ֲ����о��Ħ��ͷ��ʼ
          
          
          if((nBackMainRunMode == BACK_MAIN_MODE_AUTO) && (nBackSubRunMode == BACK_SUB_MODE_6MIN_DEMO)     )
          {
              nCurActionStep = nPreActionStep;
              bGetNextActionStep = TRUE ;///���е��������ɺ�Ż�����Զ�������һ������Ķ�����
              b6MinDemoStretch = FALSE;
              bKeyWaistHeat = FALSE ;
          }
          else
          {
             bBackAutoModeInit = true;  //Ϊ�˱����о���ֲ����о��Ħ��ͷ��ʼ  
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

    unsigned char by_EngineeringAirBag = ReadEEByte(AIRBAG_STRETCH_ADDRESS + USER_DATA_BASE);//READ ��������

    if(nBackMainRunMode == BACK_MAIN_MODE_AUTO)
    {
        if( nBackSubRunMode == BACK_SUB_MODE_AUTO_2)//������ģʽ��һ�����ɰ�Ħ��ʱ�̣��ص��粿���ֱ�����
               
        {//��ģʽ�����ɰ�Ħ���������ҹرգ�ͬʱҪΪ��ģʽ���¶���һ���Զ����ҳ���
               
            //    Valve_Control(VALVE_DISABLE,&st_AirBagArmNeck,by_EngineeringAirBag);
               
        }   
      
      
        if((Data_Get_ProgramExecTime() > VALVE_START_TIME) || !bMassagePositionUpdate )  //�ж�������ʼʱ���Ƿ񵽴�
        {
            goto VALVE_START;  //�����Ħ���Ѿ��������λ��
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
        if((nBackMainRunMode == BACK_MAIN_MODE_AUTO) && ( nBackSubRunMode == BACK_SUB_MODE_AUTO_1)  )// //���ʰ�Ħ�����칦��
        {
           //�Զ������е����ȳ���   //��ʱ�رգ���Ϊ���ҳ���δ���Ժ�
            Valve_StretchControlProce();  //ִ�����ˣ��������㷨����
            if(st_Stretch.active == TRUE)//������ʱ�̣�����������ֱ����ҳ���
            {
        //     Valve_Control(VALVE_ENABLE, &st_AirBagArmNeck, by_EngineeringAirBag);
            }
            else
            {
                Valve_Control(VALVE_ENABLE,&st_AirBagAuto,by_EngineeringAirBag);  //�����˿���ʱ�̣�������������ҳ���     st_AirBagAuto.locate = AIRBAG_LOCATE_AUTO;
                Valve_FootRollerProce(bRollerEnable,1,&st_AirBagAuto);

            }
        }
        else if((nBackMainRunMode == BACK_MAIN_MODE_AUTO) && (nBackSubRunMode == BACK_SUB_MODE_6MIN_DEMO)&& (b6MinDemoStretch == TRUE)    )// //���ʰ�Ħ�����칦��
        {
           
            
            
            
           //�Զ������е����ȳ���   //��ʱ�رգ���Ϊ���ҳ���δ���Ժ�
            Valve_StretchControlProce();  //ִ�����ˣ��������㷨����
            if(st_Stretch.active == TRUE)//������ʱ�̣�����������ֱ����ҳ���
            {
        //     Valve_Control(VALVE_ENABLE, &st_AirBagArmNeck, by_EngineeringAirBag);
            }
            else
            {
                Valve_Control(VALVE_ENABLE,&st_AirBagAuto,by_EngineeringAirBag);  //�����˿���ʱ�̣�������������ҳ���     st_AirBagAuto.locate = AIRBAG_LOCATE_AUTO;
                Valve_FootRollerProce(bRollerEnable,1,&st_AirBagAuto);

            }
        }
        //add by wgh 20170208
       else if((nBackMainRunMode == BACK_MAIN_MODE_AUTO) && ( nBackSubRunMode == BACK_SUB_MODE_AUTO_4))
        {  
          //�Զ������Ƿ����ȳ���  //��ʱ�رգ���Ϊ���ҳ���δ���Ժ�
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
          
          Valve_Control(VALVE_ENABLE,&st_AirBagAuto_Upbody,by_EngineeringAirBag);// st_AirBagAuto.pAirBagArray = AirBagModeAuto;// AirBagModeAuto[] =��������
          Valve_FootRollerProce(bRollerEnable,1,&st_AirBagAuto_Upbody);      
          
            
        }  
        
        
        
        
        
        else
        {  
          //�Զ������Ƿ����ȳ���  //��ʱ�رգ���Ϊ���ҳ���δ���Ժ�
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
          
          Valve_Control(VALVE_ENABLE,&st_AirBagAuto,by_EngineeringAirBag);// st_AirBagAuto.pAirBagArray = AirBagModeAuto;// AirBagModeAuto[] =��������
          Valve_FootRollerProce(bRollerEnable,1,&st_AirBagAuto);      
          
            
        }
    }
    else//�ֶ����������
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
//  if(Valve_Enable)  //        0��ʾ �Զ�����ģʽ �ر� 
        Valve_FootRollerProce(bRollerEnable, 0, &st_AirBagLegFoot);//�ֶ����������ֶ�����
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
            bMassagePositionUpdate = 0;    //�ֶ�����      bMassagePositionUpdate=1 Ϊ������С�����£�������������������ڽ��й�λ״̬��
               //SlideMotorControl(STATE_SLIDE_IDLE); zlw  //bMassagePositionUpdate = 0;���û������ֶ�����ʱ �����ֶ�����Ϊ���ȣ����Զ�ִ��Main_Massage_Position_Proce��������
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
        bMassagePositionUpdate = 0;    //�ֶ�����      bMassagePositionUpdate=1 Ϊ������С�����£�������������������ڽ��й�λ״̬��
           //SlideMotorControl(STATE_SLIDE_IDLE); zlw  //bMassagePositionUpdate = 0;���û������ֶ�����ʱ �����ֶ�����Ϊ���ȣ����Զ�ִ��Main_Massage_Position_Proce��������
        return;
    }
#endif
    
    w_LegPosition = LegMotor_Get_Position(); 
    
    wghw_legLocation = LegMotor_Get_Position();
    w_BackLocation = BackMotor_Get_Location();
    wghw_backLocation = BackMotor_Get_Location();
    
    //BackMotor_Proce();//�����������
    LegMotor_Proce();//���ߵ������
    
    if(!bMassagePositionUpdate) 
    {
      
      //BackMotor_Control(STATE_BACK_IDLE);
      
      
        //SlideMotorControl(STATE_SLIDE_IDLE);//���������
        return;
    }
    switch(nTargetMassagePosition)
    {
    case MASSAGE_RESET_POSITION: //���Ӹ�λ
        if(BackMotor_Control(STATE_RUN_BACK_UP) == TRUE)//���Ӹ�λʱ����������������ߣ�ֱ���ߵ���ߵ�
        {
            bBackPadFinish = TRUE;                           //��������
  
        }
        else
        {
            bBackPadFinish = FALSE;
  
        }
        //----------------------------------------------------------------
 
        if(LegMotor_Control(STATE_RUN_LEG_DOWN) == LEG_STOP_AT_DOWN)//С������
        {
            bLegPadFinish = TRUE;
        }
        else
        {
            bLegPadFinish = FALSE;
        }
  
        
        //--------------------------------------------------------  
#ifdef  RT8305T_1          
        if((nFlexStatus&0x03) ==  FLEX_AT_IN) //С���������������
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
      /*   if(SlideMotorControl(STATE_RUN_SLIDE_BACKWARD) == TRUE)  //����������,�������������������г̿��ش�
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
            
         /*  if(SlideMotorControl(STATE_RUN_SLIDE_BACKWARD) == TRUE)  //���������� ,����������У�
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
    case MASSAGE_OPTIMAL_POSITION://STATE_RUN_SLIDE_FORWARD=����   ,��һ���������� ʱ���������������е��г̿���λ�ô�,��������
        bAdjFlex = true;
          /* if(SlideMotorControl(STATE_RUN_SLIDE_FORWARD) == TRUE)  //������λ��1 ,�������� .�����������
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
//�����綯�׿��Ƴ���
void Main_BackPad_Proce(void)
{
  


  wwww_LegPosition = LegMotor_Get_Position();
  wwwr_BackLocation= BackMotor_Get_Location(); 
  
  
  
  
  
    //BackMotor_Proce();//�����������ʱ ���꿪ʼ����
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
          bVibratePause=1;//ҡ�ڵ����ͣ����
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
              bVibratePause=1;//ҡ�ڵ����ͣ����
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

//С�ȵ綯�׿��Ƴ���
/*
void Main_LegPad_Proce(void)
{
    LegMotor_Proce();
    if(st_Stretch.active) return;
    if(bMassagePositionUpdate) return;
    if(bLegPadLinkage == FALSE) //С�ȵ���������ʱ������ǰ���綯�׵�λ��
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
    else  //������С��������ǰ���綯�ױ�������ǰλ��
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
   

    
    //���Ұ�Ħֹͣ
    nKeyAirBagLocate = AIRBAG_LOCATE_NONE;
    //�񶯹���ֹͣ
    //���ȹ���ֹͣ
    bKeySeatVibrate = FALSE;
    bKeyWaistHeat = FALSE;
    //��ʱ������λ
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

void Main_Send_Leg(void)// 1 ���� ���嵽С�ȿ��ư�
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
        
        if(Flex_GetDisableAngle())  OutLegBuffer[2] |= (1<<5);// �����������Ƕ�
        
        //buffer 3
  //=================================================================================/////     
        OutLegBuffer[3] = LegKnead_GetSpeed() & 0x03;    //buffer3 =8302T ���� 
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
      //��ʶ 1	��Ħ������״̬ 1	��Ħ�ַ� 3	��Ħ���� 3
      if(nChairRunState == CHAIR_STATE_IDLE || nChairRunState == CHAIR_STATE_SLEEP)
      {
        OutBuffer[1] = 0<<6;//0����Ħ�δ��ڴ���,����Դ�رգ�ʡ��ģʽ
      }
      else
      {
        OutBuffer[1] = 1<<6;//��Ħ�δ��ڷǴ���״̬����ʱ�ֿ�����Ӧ��ͼ�����
      }
      
      /*****************************************************/
      //��Ħ�ַ���ʾ
      switch(nCurSubFunction)
      {
        //00��ֹͣ
        //01������
        //02���û�
        //03������ͬ��
        //04��ߵ��
        //05��ָѹ
        //06�����ɰ�Ħ
        //07������
      case BACK_SUB_MODE_KNEAD			: OutBuffer[1] |= 1<<3;break;
      case BACK_SUB_MODE_KNOCK			: OutBuffer[1] |= 4<<3;break;//OutBuffer[1] |= 2<<3;break;
      case BACK_SUB_MODE_WAVELET		: OutBuffer[1] |= 3<<3;break;
      case BACK_SUB_MODE_SOFT_KNOCK		: OutBuffer[1] |= 2<<3;break;//OutBuffer[1] |= 4<<3;break;
      case BACK_SUB_MODE_PRESS			: OutBuffer[1] |= 5<<3;break;
      case BACK_SUB_MODE_MUSIC			: OutBuffer[1] |= 6<<3;break;
      case BACK_SUB_MODE_RUBBING                : OutBuffer[1] |= 7<<3;break;
      default                                   : OutBuffer[1] |= 0<<3;break; 
      }
      
      OutBuffer[1] |= 0x01;  //3D ��ʶ  ��������3D����
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
      //��ʶ 1 ���� 1	���� 1	��Ħ��о�ٶ� 3 	����ͷ���λ�� 2
      //00-03 �Զ���
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
                  speed = nCurKneadKnockSpeed;//��Ħ��о�ٶ�  
                    
          }
         
          if(nBackMainRunMode==BACK_MAIN_MODE_AUTO)
          {
            
                if(AutoDirector.nSubFunction==BACK_SUB_MODE_PRESS)
                {
         
                      speed = 0;
            
                            
                 
                  
                }
                else
                    speed = nCurKneadKnockSpeed;//��Ħ��о�ٶ�
            
            
          }
         
         
         
       /*   else
          {

            speed = nCurKneadKnockSpeed;//��Ħ��о�ٶ�
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
      // ��ʶ 1	�����ӿ��� 1 	 �񶯣���Ť����ǿ�� 3	��ѹǿ�� 3

      //OutBuffer[3] = ;//(nKeySeatVibrateStrength&0x7)<<3;
      
   /*   
      if(bOzonEnable)//�����ӿ��أ�8302TĬ��Ϊ�� ��
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
        OutBuffer[3] |= (Valve_GetAirBagStrength()&0x7);//Ĭ��5����ѹǿ��,��������
      }
      //��ʶ 1	��о��Ħ��λ 2	����ʱ���5λ 5
      //��ʾλ��
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
      unsigned int time = Data_Get_TimeSecond();//��Ħ��ʣ����������
#endif    
      OutBuffer[4] |=(time>>7)& 0x1f;
      //��ʶ 1	����ʱ���7λ 7
      OutBuffer[5] = time & 0x7f;
      
      OutBuffer[6] = 0x00;
      if( (bLeftFootAirBagValve) | (bRightFootAirBagValve) |(bFootHeelAirBagValve))//�㲿���Ҷ���ָʾ
      {
        OutBuffer[6] |= 0x01;
      }
      if( (bLegDownBottomAirBagValve) |(bLegDownSideAirBagValve)|(bLegUpBottomAirBagValve)|(bLegUpSideAirBagValve))//С�����Ҷ���ָʾ
      {
       OutBuffer[6] |= 0x02;
      }
      if((bLeftThighAirBagValve) | (bRightThighAirBagValve ))//�������Ҷ���ָʾ
      {
        OutBuffer[6] |= 0x04;
      }
 //===================================================     
  //  �ڸ�����Ҫ�����������Ҷ���ָʾ
     // if(bButtocksAirBagValve)
     // {
     //   OutBuffer[6] |= 0x08;
     // }
  //===================================================    
   //�ֱ����Ҷ���ָʾ   
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
      

      //if((bSholderAirBagValve))// | (bRightSholderAirBagValve))//�粿���Ҷ���ָʾ
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
      /*data /= 26;//31; �ܹ�8������ͼ��ƨ�ɵ���Ħ����
      if(data >= 7) data = 7;
      OutBuffer[8] = (data+0);   //��ϲ��ͼ��
      */
      data /= 31;
      if(data >= 13) data = 13;
      OutBuffer[8] = data;   
     
      
///////////////////////////////////////////////////////////////
      if(isFIRSTZeroPosition())//��������ʾ,
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
      else//��λ   ��˸ 
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
          OutBuffer[9] = 1<<6;//1��ִ�����ͼ�����
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
      //��ʶ 1	����ָʾ 1	С�ȵ綯�����з���ָʾ 3	�����綯�����з���ָʾ 3
      OutBuffer[10] = 0;
   // if((nBackMainRunMode == BACK_MAIN_MODE_AUTO) && ( nBackSubRunMode == BACK_SUB_MODE_AUTO_1))

      //==================================================================����Ҫ��Ϊ�������綯��ָʾ
    /*  if(SlideMotor_GetPower() == SLIDE_MOTOR_POWER_ON)//�������綯�׻򻬶��綯������ָʾ 
        //if(BackMotor_GetPower() == BACK_MOTOR_POWER_ON)//�����綯������ָʾ
        //if(LegMotor_GetPower() == LEG_MOTOR_POWER_ON)//С�ȵ綯������ָʾ
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
      if(BackMotor_GetPower() == BACK_MOTOR_POWER_ON)//�����綯������ָʾ
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
      if(LegMotor_GetPower() == LEG_MOTOR_POWER_ON)//С�ȵ綯������ָʾ
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
      //��ʶ	1 ������ģʽ 2 ���ֿ��� 1	���� 4
      if(bSendBuzzerMode == TRUE)
      {
             
        OutBuffer[11] = (nBuzzerMode&0x3)<<5;                 
       // bSendBuzzerMode = FALSE ;  ������ͨѶ������
      }
      else
      {
        OutBuffer[11] = 0;
      }
      
      if(bBlueToothStatus)
      //if(BlueToothMuteState() == BlueTooth_Speak_Out_On)
      {
        OutBuffer[11] |= 1 << 4;   //�������ֿ���
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
      //OutBuffer[12] |= ((nKeyAirBagLocate & 0x07) << 2);//ѡ�е����Ұ�Ħ��λ
      OutBuffer[12] |= ((nKeyAirBagLocate & 0x1F) << 2);//ѡ�е����Ұ�Ħ��λ
      //���ַ���
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
        //��ʶ 1	����ʱ���7λ 7
        OutBuffer[5] = time & 0x7f;
      }
      else
      {
        if(nBackMainRunMode == BACK_MAIN_MODE_AUTO)//�Զ���Ħ�����־
        {
              mode = (nBackSubRunMode + 1);
              
             /*  if(nBackSubRunMode >= BACK_SUB_MODE_CLUDE_AUTO_0&&nBackSubRunMode <= BACK_SUB_MODE_CLUDE_AUTO_3)
               {
                 mode = nBackSubRunMode-BACK_SUB_MODE_CLUDE_AUTO_0+8;//�����ʾ���������־λ  ,��������������
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
          //OutBuffer[13] |=  0x0C << 2;  //OLD =0X0B  ������������޸ĵ�ַ
          OutBuffer[13] |=  0X0B << 2;  //OLD =0X0B  ������������޸ĵ�ַ
        }
        else if(nChairRunState == CHAIR_STATE_RUN)
        {
          OutBuffer[13] |=  7 << 2;
        }
      }
 	 //======bit 4   0:����  1:usb==============================
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
        if(nAxisUpdateCounter < 5)  OutBuffer[14] = (nKeyAxisStrength+1) & 0x07;//3D��Ħ����
        else if(nAxisUpdateCounter < 10)  OutBuffer[14] = 0;
        else if(nAxisUpdateCounter < 15)  OutBuffer[14] = (nKeyAxisStrength+1) & 0x07;
        else if(nAxisUpdateCounter < 20)  OutBuffer[14] = 0;
        else if(nAxisUpdateCounter < 25)  OutBuffer[14] = (nKeyAxisStrength+1) & 0x07;
        else OutBuffer[14] = 0;
      }
      else
      {
        //OutBuffer[14] = (nFinalAxisStrength+1) & 0x07;
        OutBuffer[14] = (nKeyAxisStrength+1) & 0x07;//(nDisplayAxisStrength+1) & 0x07;//20150421 wgh ��Ӽ粿��ʾ
        
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
    Main_Send_Leg();//���巢���ݸ�С�Ȱ�
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
    OutBufferBlueTooth[1] = 0x01;  //3D ��ʶ   BIT0
#ifdef  RT8305T_1
    OutBufferBlueTooth[1] |= 0x00; //����������С��=1    ,��������С��Ϊ0  ,BIT1
#else
    OutBufferBlueTooth[1] |= 0x02; //0=����������С��    ,��������С��Ϊ1  ,BIT1
 #endif
    
    
    
    OutBufferBlueTooth[1] |= 0x04;  //3D ��ʶ  //�Զ��������Ʊ�־λ  BIT2
    //��ʶ 1	��Ħ������״̬ 1	��Ħ�ַ� 3	��Ħ���� 3
    if(nChairRunState == CHAIR_STATE_IDLE || nChairRunState == CHAIR_STATE_SLEEP)
    {
      OutBufferBlueTooth[1] |= 0 << 6;
    }
    else
    {
      OutBufferBlueTooth[1] |= 1 << 6;
    }
    
    /*****************************************************/
    //��Ħ�ַ���ʾ
    switch(nCurSubFunction)
    {
      //00��ֹͣ
      //01������
      //02���û�
      //03������ͬ��
      //04��ߵ��
      //05��ָѹ
      //06�����ɰ�Ħ
      //07������
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
    //��ʶ 1 ���� 1	���� 1	��Ħ��о�ٶ� 3 	����ͷ���λ�� 2
    //00-03 �Զ���
  
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
    // ��ʶ 1	�����ӿ��� 1 	 �񶯣���Ť����ǿ�� 3	��ѹǿ�� 3
    OutBufferBlueTooth[3] = (nKeySeatVibrateStrength & 0x7) << 3;
    
    if(nKeyAirBagLocate != AIRBAG_LOCATE_NONE)
    {
      OutBufferBlueTooth[3] |= (Valve_GetAirBagStrength() & 0x7);
    }
    //��ʶ 1	��о��Ħ��λ 2	����ʱ���5λ 5
    //��ʾλ��
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
      time /= 60;    //demoģʽ ʱ�䰴����ʾ
    }
    OutBufferBlueTooth[4] |= (time >> 7) & 0x1f;
    //��ʶ 1	����ʱ���7λ 7
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
    data = nFinalWalkMotorLocate; //�Զ�ģʽʹ�ü��λ��
  }
    else
    {
    data = TOP_POSITION;   //�ֶ�ģʽʹ���Զ��γ�
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
     //��ʶ	1 ������ģʽ 2 ���ֿ��� 1	���� 4
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
    //��ʶ	1 ������ģʽ 2 ���ֿ��� 1	���� 4
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
    case AIRBAG_LOCATE_LEG_FOOT:locate = 0x04;break;//�Ƚ����ҳ���
  //  case AIRBAG_LOCATE_BODY_UP: locate = 0x20;break;
 //   case AIRBAG_LOCATE_BACK_WAIST: locate = 0x08; break;//�������ҳ���
 //   case AIRBAG_LOCATE_ARM_SHOLDER: locate = 0x10; break;//�ۼ����ҳ���
    case AIRBAG_LOCATE_SEAT:locate = 0x20;break;//�������ҳ���
    case AIRBAG_LOCATE_ARM_NECK:locate = 0x10;break;
    
    
    case AIRBAG_LOCATE_AUTO:locate = 0x40;break;//ȫ�����ҳ���
   // case AIRBAG_LOCATE_ARM:break;
    }
    OutBufferBlueTooth[12] |= locate;//��ַ12 ʱ������� ,Ϊ�û�ѡ�е����ҳ���
    */
    OutBufferBlueTooth[12] |= ((nKeyAirBagLocate & 0x1f) <<2);//add wgh 20170208
    
    //���ַ���
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
      OutBufferBlueTooth[14] = (nKeyAxisStrength+1) & 0x07;//(nDisplayAxisStrength+1) & 0x07;//(nDisplayAxisStrength+1) & 0x07;// 20150421 wgh ��Ӽ粿��ʾ
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
     OutBufferBlueTooth[1] = 0x85; //��Ϣ֡ʶ����
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
//     OutBufferBlueTooth[10] = EOI;   //����ȥ��ֹͣλ ���������ϵ�APP����
     nOutBufferBlueToothCount =10;// 11;
     BlueToothUart_Transmit_Packet(OutBufferBlueTooth, nOutBufferBlueToothCount);
     OutBufferBlueTooth_Old[0] = 0;  //��һ֡Ϊ״̬֡
    }  
    
    
    
 //   BlueToothUart_Transmit_Packet(OutBufferBlueTooth, nOutBufferBlueToothCount);
    bBlueToothMasterSendPacket = FALSE ;
  }
}

void Main_Initial_Data(void)
{
    bAxisUpdate = 1;  //�ϵ��3D����ȹ���
    
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
    {  //�״�ʹ����Ҫ��ʼ������
        *(pInformation + SOFT_MAIN_VER_ADDRESS) = SOFT_MAIN_VER;
        *(pInformation + SOFT_SECONDARY_VER_ADDRESS) = SOFT_SECONDARY_VER;
        *(pInformation + SETTLE_ADDRESS) = MEMORY_DEFAULT_SETTLE;     
        *(pInformation + AIRBAG_STRETCH_ADDRESS) = MEMORY_DEFAULT_AIR;          //��Ħ���ڲ���������     //
        *(pInformation + SLIDE_MOTOR_ENABLE_ADDRESS) = SLIDE_DEFAULT_ENABLE; 
        *(pInformation + DEFAULT_TIME_ADDRESS) = RUN_TIME_20/60; //Ĭ������ʱ��20����
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
  //by_t =    ReadEEByte(AIRBAG_STRETCH_ADDRESS + USER_DATA_BASE);//READ ��������
    
    
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
    //���˳��������г̿��ؿ���
    st_Stretch.mode = STRETCH_MODE_SWITCH;
    st_Stretch.PresetTime = 200;//200*0.1sec=20sec
    st_Stretch.active = false;
    bRockEnable = false;
    nRockModeEnterEnable = ExitRock;
   
    

    
    st_AirBagLegFoot.pAirBagArray = AirBagModeLegFoot;
    st_AirBagLegFoot.nTotalSteps = sizeof(AirBagModeLegFoot)/sizeof(struct AirBagStruct);
    st_AirBagLegFoot.locate = AIRBAG_LOCATE_LEG_FOOT;
    

    
    st_AirBagSeat.pAirBagArray = AirBagModeSeat;  //�����β�
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
   
    st_AirBagArmNeck.pAirBagArray = AirBagModeArmSholder;//AirBagModeArmNeck;   //�۾�����.ʵ��ʹ�õ��Ǳۼ�����
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
    nBackMainRunMode = BACK_MAIN_MODE_IDLE ;//��о״̬Ϊ����״̬
    nKeyBackLocate = LOCATE_NONE;   //ȫ�̣��ֲ������㶨λ��־
    nKeyKneadWidth = KNEAD_WIDTH_UNKNOWN ;
    nKeyKneadKnockSpeed = SPEED_0 ;
    //Walk Motor Variables
    bWalkMotorInProcess = FALSE ;
    nWalkMotorControlParam1 = WALK_LOCATE_PARK ;
    nWalkMotorControlParam2 = 0 ;
    bUpdateLocate = TRUE ;     //���ߵ��������±�־����λʱ����һ������
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
    
    nTargetMassagePosition = MASSAGE_RESET_POSITION;   //Ŀ�갴Ħλ��
    WorkStep = 0;
     w_ZeroPosition = 0;//�����������λ
bZeroflash = FALSE;
    
    bMassagePositionUpdate = FALSE;
    
    w_PresetTime = ReadEEByte(USER_DATA_BASE + DEFAULT_TIME_ADDRESS)*60;//��ĦԤ��ʱ��

    bBlueToothStatus = ReadEEByte(USER_DATA_BASE + BLUETOOTH_STATUS_ADDRESS);//����Ĭ��״̬Ϊ��
     
    //�������ݳ�ʼ��
    //nRollerPWM = 0;
    bRollerEnable = false;
    //Ť�����ݳ�ʼ��
    nKeySeatVibrateStrength=0;//nKeySeatVibrateStrength_old
    bKeySeatVibrate=0;
    nKeySeatVibrateStrength_old=0;
    
  //  Enable_Vibrate=0;
    //==============================
    
    
    
    Data_Init();
    LEUART0_Initial_Data();////��ȡ3D��о3D�����״̬�ź�
    
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
    
  /*  if(VoiceUart_GetRXStatus() == TRUE)  �������������հ���
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
        //������������������
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
        //������������������
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
      //������������������
      if(Input_GetWalkDownSwitch() == REACH_WALK_LIMIT)
      {
        //������������������
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
        //������������������
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
//ֹͣ���е�ִ��װ��
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
    
 //   Main_WaveMotorStop();//ҡ�ڵ��ֹͣ
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
#define foot_Switch     engineerData5.bD4   //�ŵ׿���
//#define leg_angle       engineerData5.bD5   //�Ƕȿ���
//#define leg_ground      engineerData5.bD6//���ؿ���


#define knead_phase     engineerData5.bD7


BITS engineerData6;
 #define      leg_angle_Switch_old        engineerData6.bD0
 #define      leg_groud_switch_old        engineerData6.bD1
    
 #define      leg_angle                   engineerData6.bD2   //�Ƕȿ���
 #define      leg_ground                  engineerData6.bD3//���ؿ���                  
        
 
                  




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
//#define waver_check          engineerData4.bD1      //ҡ�ڵ�����



#define TIME_COUNT      100
//�˺���ִ����ϻ�����CPU��λ
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
    
  //  unsigned char waverSpeed=2;//�趨��ʼҡ���ٶ�Ϊ2
    
   // unsigned char nWaverCounter=0;//�趨ҡ�ڵ������λ����
    
    
    
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
    
   // BlueToothEnterCmdMode();  //����ģ�鹤��������ģʽ
    
  //  BlueToothUart_GetName();
    
     nKeySeatVibrateStrength = 0 ;//�趨��ʼҡ���ٶ�Ϊ2
    
         waver_check=0;  //ҡ�ڵ�����
         //nWaverCounter=0;//ҡ�ڵ��������
       Timer_Initial();
     
     
   //   Valve_Test_Set_Data(0x0101);//
    //while(1)
    //{
  
    
   // Valve_Send_Data();
  // }    
    
    while(status)
    { 
        //WDOG_Feed();
        lingo = Main_GetKey();//��ȡ�ܿ�������ֵ
        
        switch(lingo)
        {
        case LINGO_AIRBAG:    //��������
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
    //�ù��ֲ��Գ������С��������    
        case LINGO_ROLLER_TEST: //�ŵ׹��ֲ���
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
            
        case LINGO_SLIDE_TEST:    //�������綯�ײ���
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
            
        case LINGO_WAVER_TEST  : //ҡ�ڵ綯�ײ���
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
            
            
            
        case LINGO_BACK_TEST:     //�����������
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
        case LINGO_LEG_TEST:   //С�����²���
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
            engineerTimeCount = 1, air_bagTimeCount = 1; //��ʱ�䣬�������ֹ������һ��
   //           Waveringly_Set_Pwm_TEST_Data(VIB_STRENGTH[2]); 
            switch(command)
            {
            case SYS_KEY_UP://��һ��
                if(oneKeyStep > 1)oneKeyStep--;
                else oneKeyStep = oneKeyStepLength;
                if(test_finish && oneKeyStep == 0)oneKeyStep = oneKeyStepLength;//// һ���Զ�����=oneKeyStepLength = 4  �ܹ���4�� ��={ 1=heat.2=knock, 3=roller, 4=air_ba   }
                break;
            case SYS_KEY_DOWN://��һ��
                if(oneKeyStep < oneKeyStepLength)oneKeyStep++;
                else oneKeyStep = 0;
                break;
            case SYS_KEY_LEFT://����������һ��(�����ң���һ��)
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
            case SYS_KEY_RIGHT://�أ�������һ��(�����ң���һ��)
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
            case 248://�˵��н���˽���
                //��ʼ��
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
                
                
                //nWaverCounter=0;//ҡ�ڵ��������
              
                
                back_position = 0;
                walk_position = 0;
                engineering_stop_all();
                engineerData5.nByte = 0;
                
                
                bwaverCheck_First=1;
                bwaverCheck_Sencod=0;
                bwaverCheck_First2=0;
                waver_check=0;  //ҡ�ڵ�����
                nKeySeatVibrateStrength=2;//ҡ�ڵ���ٶ�
                
                leg_flex_step = 0;
                slide_step = 0;

                
                 if(nFlexStatus&0x20) //�����Ƕȿ���
                 {
                   
                   leg_angle_Switch_old=1;
                 }
                 else
                 {
                    leg_angle_Switch_old=0;
                 }
                 
                 
                if(nFlexStatus&0x40)//��������
                {
                  leg_groud_switch_old=1;
                  
                }
                else
                {
                  leg_groud_switch_old=0;//leg_angle_Switch_old
                  
                }
                
                
                
                if(nFlexStatus&0x04) //�ŵ������ŵ׿����ź�
                {
                  foot_Switch_old = 1;
                }
                else
                {
                  foot_Switch_old = 0;
                }
                shoulder_detect_old = Input_GetVout();//��λ���
                
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
            
        case LINGO_HEAT_TEST:  //���Ȳ���
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

        case LINGO_FLEX_TEST:       //С����������
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

        case LINGO_WALK_TEST:   //���ߵ�������
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

         case LINGO_3D_TEST:    //3D����������
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
            
        case LINGO_LED_TEST:     //LED�ʵƲ���
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
                *(pInformation + PROGRAM_ENABLE_ADDRESS) = PROGRAM_FLAG; //д��̱�־λ
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
                case ENG_CMD_RESET:  //�ػ��Ƿ�λ
                    
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
                case ENG_CMD_DEC_STRENGTH:  //�������ȼ�1
                    strength = *(pInformation + AIRBAG_STRETCH_ADDRESS);
                    //   by_t = *(pInformation + AIRBAG_STRETCH_ADDRESS);

                    if(strength == 0) break;
                    strength--;
                    strength %= 3;  //��ֹ��Ϊ�ϵ��ԭ�������ݴ���
                    *(pInformation + AIRBAG_STRETCH_ADDRESS) = strength; 
                    MEM_Write_Memory(pw_Information,MEMORY_LENGTH_OF_BYTES);  
                    break;
                case ENG_CMD_ADD_STRENGTH:  //�������ȼ�1
                    strength = *(pInformation + AIRBAG_STRETCH_ADDRESS);
              //       by_t = *(pInformation + AIRBAG_STRETCH_ADDRESS);
                    if(strength >= 2) break;
                    strength++;
                    *(pInformation + AIRBAG_STRETCH_ADDRESS) = strength; 
                    MEM_Write_Memory(pw_Information,MEMORY_LENGTH_OF_BYTES);  
                    break;
                case ENG_CMD_SLIDE:   //����ʹ�ܽ�ֹ
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
            break; //��λCPU
        case LINGO_MENU:  engStatus = LINGO_MENU;
        break; //��λCPU
        }
        /*******���³���Ϊ���Ҳ���*************************/  
        switch(engStatus)
        {
        
        case LINGO_ONE_KEY_TEST:
        {
            //����ź� TODO,�������ź�ʱ�����
            Input_Proce();
            //ʹ���жϱ�־
            if(engineeringTime_10msFlag)//10ms timer
            {
                engineerTimeCount++;
                engineerTimeCount %= 10 * TIME_COUNT; //10����һ��   #define TIME_COUNT      100
                //engineerTimeCount %= 1000;  //��engineerTimeCount=1000ʱ����Ϊ0 ������ʱ��Ϊ10sec
                air_bagTimeCount++;
                air_bagTimeCount %= 7 * TIME_COUNT; //7����һ��
                // air_bagTimeCount %= 700;//air_bagTimeCount=700ʱ����Ϊ0 ������ʱ��Ϊ7sec
                if(oneKeyLegCountDown > 0)oneKeyLegCountDown--;
                //ʱ���ж�����
                engineeringTime_10msFlag = 0;
            }
            //ʵ��
            //����
            //back_position = Input_GetBackMotorPosition();
            back_position = 0;
            walk_position = Input_GetWalkMotorPosition();
            //�Զ����Բ���
            if(oneKeyStep == 0)
            {
          
              
              
                //����
                if(!walk_up)//�Զ�����ʱ��һ��������������ߣ�                       ��һ��
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
                                //���г�OK
                                walk_up = 1;
                                //����
                                walk_check_count = 0;
                            }
                        }
                        WalkMotor_Control(STATE_RUN_WALK_DOWN, 0);
                    }
                    else
                    {
                        WalkMotor_Control(STATE_RUN_WALK_UP, 0);
                    }
                }//��λ
                else if(!walk_down)//����������ߵ����λ��ʱ����������������ߣ�   �ڶ���
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
                                //���г�OK
                                walk_down = 1;
                                //����
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
                else                                                 //�����ϣ���������һ���г̺����ߵ�������ߵ���ߵ�ͣ����
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
                //����
                if(!knead_width_min)                 //��һ������ խλ��Ѱ��
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
                                //����
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
                else if(!knead_width_mid)// //  �ڶ��� ���� ��λ��Ѱ��
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
                                //����
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
                else if(!knead_width_max)   //������  ��λ��Ѱ��
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
                                //����
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
                else if (!_3D_Switch_Forward)//��һ��3D��о����ǰ  ��3D�����ź�����
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
                else if (!_3D_Switch_Back)  //�ڶ��� 3D��о�����    3D�����źż�С
                {
                  if(AxisMotor_Control(STATE_RUN_AXIS_BEHIND,0,_3D_SPEED_7))
                  //if(Input_Get3DBackSwitch()) 
                  {
                    AxisMotor_Control(STATE_AXIS_IDLE,0,_3D_SPEED_10); 
                    _3D_Switch_Back = 1;
                    //printf("3d_back\n");
                  }
                }
                else if (!_3D_Switch_Pluse)   //��� 3D�����ź��Ƿ�����
                 
                  
                {
                  if(AxisMotor_Control(STATE_RUN_AXIS_REAL_VECTOR,20,_3D_SPEED_7))
                  {
                    AxisMotor_Control(STATE_AXIS_IDLE,0,_3D_SPEED_10); 
                    _3D_Switch_Pluse = 1;
                   // printf("3d_pluse\n");
                  }
                }
                else AxisMotor_Control(STATE_AXIS_IDLE,0,_3D_SPEED_10); 
                  
                //С��

           //========================================================================
       
#ifdef    RT8305T_1    
                
               if(has_leg)
                {
                  Flex_SetDisableAngle(1);
                    if((!leg_up) &&(!leg_down)&&(!flex_up)&&(!flex_down))  
                    {   //����С�����г̿���
                       //FlexMotor_Control(STATE_RUN_FLEX_RESET, FLEX_SPEED_FAST, FLEX_CURRENT_3A);
                      Flex_SetDirection(FLEX_MOTOR_STOP);
                       switch(leg_flex_step)
                       {
                        case 0:  //����upλ��
                                if(LegMotor_Control(STATE_RUN_LEG_UP)) 
                                {
                                  leg_flex_step++;
                                  Timer_Counter_Clear(C_TIMER_ENG1);
                                }
                                break;  
                         case 1:  //ͣ��upλ��0.5��
                               LegMotor_Control(STATE_LEG_IDLE);
                               if(Timer_Counter(C_TIMER_ENG1,5))
                                {
                                  leg_flex_step++;
                                }
                               break;  
                         case 2:  //�뿪upλ��
                                LegMotor_Control(STATE_RUN_LEG_DOWN);
                                if(Input_GetLegUpSwitch() != REACH_LEG_LIMIT)
                                {
                                  Timer_Counter_Clear(C_TIMER_ENG1);
                                  leg_flex_step++;
                                }
                                break;
                          case 3:  //ͣ0.5��
                               LegMotor_Control(STATE_LEG_IDLE);
                               if(Timer_Counter(C_TIMER_ENG1,5))
                                {
                                  leg_flex_step++;
                                }
                               break;
                          case 4:  //����upλ��
                                if(LegMotor_Control(STATE_RUN_LEG_UP)) 
                                {
                                  leg_flex_step = 0;
                                  leg_up = 1;
                                }
                                break;       
                       }
                    }
                    if((leg_up) &&(!leg_down)&&(!flex_up)&&(!flex_down))  
                      {//���Ե綯����С���ϣ��⣩�г̿���
                        LegMotor_Control(STATE_LEG_IDLE);
                        switch(leg_flex_step)
                       {
                        case 0:  //����upλ��
                                //if(FlexMotor_Control(STATE_RUN_FLEX_MANUAL_OUT, FLEX_SPEED_FAST, FLEX_CURRENT_3A))
                              if(Flex_ControlOut(FLEX_MOTOR_CURRENT_2A))
                                {
                                  leg_flex_step++;
                                  Timer_Counter_Clear(C_TIMER_ENG1);
                                }
                                break;  
                         case 1:  //ͣ��upλ��0.5��
                               //FlexMotor_Control(STATE_FLEX_IDLE, FLEX_SPEED_FAST, FLEX_CURRENT_3A);
                               if(Timer_Counter(C_TIMER_ENG1,5))
                                {
                                  leg_flex_step++;
                                }
                               break;  
                         case 2:  //�뿪upλ��
                                //FlexMotor_Control(STATE_RUN_FLEX_RESET, FLEX_SPEED_FAST, FLEX_CURRENT_3A);
                               if(Flex_ControlIn(FLEX_MOTOR_CURRENT_1D5A))
                                //if(Input_GetFlexOutSwitch() != REACH_FLEX_LIMIT)
                                {
                                  Timer_Counter_Clear(C_TIMER_ENG1);
                                  leg_flex_step++;
                                }
                                break;
                          case 3:  //ͣ0.5��
                               //FlexMotor_Control(STATE_FLEX_IDLE, FLEX_SPEED_FAST, FLEX_CURRENT_3A);
                               if(Timer_Counter(C_TIMER_ENG1,5))
                                {
                                  leg_flex_step++;
                                }
                               break;
                          case 4:  //����upλ��
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
                      {//���Ե綯����С���ϣ��⣩�г̿���
                        LegMotor_Control(STATE_LEG_IDLE);
                        switch(leg_flex_step)
                       {
                        case 0:  //����inλ��
                                if(Flex_ControlIn(FLEX_MOTOR_CURRENT_3A))
                                //if(FlexMotor_Control(STATE_RUN_FLEX_RESET, FLEX_SPEED_FAST, FLEX_CURRENT_3A))
                                {
                                  leg_flex_step++;
                                  Timer_Counter_Clear(C_TIMER_ENG1);
                                }
                                break;  
                         case 1:  //ͣ��inλ��0.5��
                              // FlexMotor_Control(STATE_FLEX_IDLE, FLEX_SPEED_FAST, FLEX_CURRENT_3A);
                               if(Timer_Counter(C_TIMER_ENG1,5))
                                {
                                  leg_flex_step++;
                                }
                               break;  
                         case 2:  //�뿪inλ��
                                //FlexMotor_Control(STATE_RUN_FLEX_MANUAL_OUT, FLEX_SPEED_FAST, FLEX_CURRENT_3A);
                                //if(Input_GetFlexInSwitch() != REACH_FLEX_LIMIT)
                              if(Flex_ControlOut(FLEX_MOTOR_CURRENT_2A))
                                {
                                  Timer_Counter_Clear(C_TIMER_ENG1);
                                  leg_flex_step++;
                                }
                                break;
                          case 3:  //ͣ0.5��
                              // FlexMotor_Control(STATE_FLEX_IDLE, FLEX_SPEED_FAST, FLEX_CURRENT_3A);
                               if(Timer_Counter(C_TIMER_ENG1,5))
                                {
                                  leg_flex_step++;
                                }
                               break;
                          case 4:  //����Inλ��
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
                   {   //����С�����г̿���
                       //FlexMotor_Control(STATE_FLEX_IDLE, FLEX_SPEED_FAST, FLEX_CURRENT_3A); //�رյ綯����С��
                       switch(leg_flex_step)
                       {
                        case 0:  //����downλ��
                                if(LegMotor_Control(STATE_RUN_LEG_DOWN)) 
                                {
                                  leg_flex_step++;
                                  Timer_Counter_Clear(C_TIMER_ENG1);
                                }
                                break;  
                         case 1:  //ͣ��DOWNλ��0.5��
                               LegMotor_Control(STATE_LEG_IDLE);
                               if(Timer_Counter(C_TIMER_ENG1,5))
                                {
                                  leg_flex_step++;
                                }
                               break;  
                         case 2:  //�뿪DOWNλ��
                                LegMotor_Control(STATE_RUN_LEG_UP) ;
                                if(Input_GetLegDownSwitch() != REACH_LEG_LIMIT);
                                {
                                  Timer_Counter_Clear(C_TIMER_ENG1);
                                  leg_flex_step++;
                                }
                                break;
                          case 3:  //ͣ0.5��
                               LegMotor_Control(STATE_LEG_IDLE);
                               if(Timer_Counter(C_TIMER_ENG1,5))
                                {
                                  leg_flex_step++;
                                }
                               break;
                          case 4:  //����Downλ��
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
                    {   //����С�����г̿���
  
                      Flex_SetDirection(FLEX_MOTOR_STOP);
                       switch(leg_flex_step)
                       {
                        case 0:  //����upλ��
                                if(LegMotor_Control(STATE_RUN_LEG_UP)) 
                                {
                                  leg_flex_step++;
                                  Timer_Counter_Clear(C_TIMER_ENG1);
                                }
                                break;  
                         case 1:  //ͣ��upλ��0.5��
                               LegMotor_Control(STATE_LEG_IDLE);
                               if(Timer_Counter(C_TIMER_ENG1,5))
                                {
                                  leg_flex_step++;
                                }
                               break;  
                         case 2:  //�뿪upλ��
                                LegMotor_Control(STATE_RUN_LEG_DOWN);
                                if(Input_GetLegUpSwitch() != REACH_LEG_LIMIT)
                                {
                                  Timer_Counter_Clear(C_TIMER_ENG1);
                                  leg_flex_step++;
                                }
                                break;
                          case 3:  //ͣ0.5��
                               LegMotor_Control(STATE_LEG_IDLE);
                               if(Timer_Counter(C_TIMER_ENG1,5))
                                {
                                  leg_flex_step++;
                                }
                               break;
                          case 4:  //����upλ��
                                if(LegMotor_Control(STATE_RUN_LEG_UP)) 
                                {
                                  leg_flex_step = 0;
                                  leg_up = 1;
                                }
                                break;       
                       }
                    }
             /*       if((leg_up) &&(!leg_down)&&(!flex_up)&&(!flex_down))  
                      {//���Ե綯����С���ϣ��⣩�г̿���
                        LegMotor_Control(STATE_LEG_IDLE);
                        switch(leg_flex_step)
                       {
                        case 0:  //����upλ��
                                //if(FlexMotor_Control(STATE_RUN_FLEX_MANUAL_OUT, FLEX_SPEED_FAST, FLEX_CURRENT_3A))
                              if(Flex_ControlOut(FLEX_MOTOR_CURRENT_3A))
                                {
                                  leg_flex_step++;
                                  Timer_Counter_Clear(C_TIMER_ENG1);
                                }
                                break;  
                         case 1:  //ͣ��upλ��0.5��
                               //FlexMotor_Control(STATE_FLEX_IDLE, FLEX_SPEED_FAST, FLEX_CURRENT_3A);
                               if(Timer_Counter(C_TIMER_ENG1,5))
                                {
                                  leg_flex_step++;
                                }
                               break;  
                         case 2:  //�뿪upλ��
                                //FlexMotor_Control(STATE_RUN_FLEX_RESET, FLEX_SPEED_FAST, FLEX_CURRENT_3A);
                               if(Flex_ControlIn(FLEX_MOTOR_CURRENT_3A))
                                //if(Input_GetFlexOutSwitch() != REACH_FLEX_LIMIT)
                                {
                                  Timer_Counter_Clear(C_TIMER_ENG1);
                                  leg_flex_step++;
                                }
                                break;
                          case 3:  //ͣ0.5��
                               //FlexMotor_Control(STATE_FLEX_IDLE, FLEX_SPEED_FAST, FLEX_CURRENT_3A);
                               if(Timer_Counter(C_TIMER_ENG1,5))
                                {
                                  leg_flex_step++;
                                }
                               break;
                          case 4:  //����upλ��
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
                      {//���Ե綯����С���ϣ��⣩�г̿���
                        LegMotor_Control(STATE_LEG_IDLE);
                        switch(leg_flex_step)
                       {
                        case 0:  //����inλ��
                                if(Flex_ControlIn(FLEX_MOTOR_CURRENT_3A))
                                //if(FlexMotor_Control(STATE_RUN_FLEX_RESET, FLEX_SPEED_FAST, FLEX_CURRENT_3A))
                                {
                                  leg_flex_step++;
                                  Timer_Counter_Clear(C_TIMER_ENG1);
                                }
                                break;  
                         case 1:  //ͣ��inλ��0.5��
                              // FlexMotor_Control(STATE_FLEX_IDLE, FLEX_SPEED_FAST, FLEX_CURRENT_3A);
                               if(Timer_Counter(C_TIMER_ENG1,5))
                                {
                                  leg_flex_step++;
                                }
                               break;  
                         case 2:  //�뿪inλ��
                                //FlexMotor_Control(STATE_RUN_FLEX_MANUAL_OUT, FLEX_SPEED_FAST, FLEX_CURRENT_3A);
                                //if(Input_GetFlexInSwitch() != REACH_FLEX_LIMIT)
                              if(Flex_ControlOut(FLEX_MOTOR_CURRENT_3A))
                                {
                                  Timer_Counter_Clear(C_TIMER_ENG1);
                                  leg_flex_step++;
                                }
                                break;
                          case 3:  //ͣ0.5��
                              // FlexMotor_Control(STATE_FLEX_IDLE, FLEX_SPEED_FAST, FLEX_CURRENT_3A);
                               if(Timer_Counter(C_TIMER_ENG1,5))
                                {
                                  leg_flex_step++;
                                }
                               break;
                          case 4:  //����Inλ��
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
                   {   //����С�����г̿���
                       //FlexMotor_Control(STATE_FLEX_IDLE, FLEX_SPEED_FAST, FLEX_CURRENT_3A); //�رյ綯����С��
                       switch(leg_flex_step)
                       {
                        case 0:  //����downλ��
                                if(LegMotor_Control(STATE_RUN_LEG_DOWN)) 
                                {
                                  leg_flex_step++;
                                  Timer_Counter_Clear(C_TIMER_ENG1);
                                }
                                break;  
                         case 1:  //ͣ��DOWNλ��0.5��
                               LegMotor_Control(STATE_LEG_IDLE);
                               if(Timer_Counter(C_TIMER_ENG1,5))
                                {
                                  leg_flex_step++;
                                }
                               break;  
                         case 2:  //�뿪DOWNλ��
                                LegMotor_Control(STATE_RUN_LEG_UP) ;
                                if(Input_GetLegDownSwitch() != REACH_LEG_LIMIT);
                                {
                                  Timer_Counter_Clear(C_TIMER_ENG1);
                                  leg_flex_step++;
                                }
                                break;
                          case 3:  //ͣ0.5��
                               LegMotor_Control(STATE_LEG_IDLE);
                               if(Timer_Counter(C_TIMER_ENG1,5))
                                {
                                  leg_flex_step++;
                                }
                               break;
                          case 4:  //����Downλ��
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
                //����
#ifdef  RT8305T_1
              
               
                      /*          if(nFlexStatus&0x20) //�����Ƕȿ���
                 {
                   
                   leg_angle_Switch_old=1;
                 }
                 else
                 {
                    leg_angle_Switch_old=0;
                 }*/
                   
               
               
               if(nFlexStatus&0x20) //�����Ƕȿ��� ,�Ƕȿ��ػ�������� һֱΪON ,�Ƕȿ���û��ʱΪ0FF,Ӳ����·Ĭ��ΪON
               {
                 if(leg_angle_Switch_old==0)leg_angle=1;//leg_ground
                 
               }
               else
               {
                  if(leg_ground)leg_angle=1;//��ʱ���ֻ���һֱ���Ƕȿ���λ�ã���ʱ�����Ѿ���
                 
                  if(leg_angle_Switch_old!=0)leg_angle=1;
                 
               }
              
               
                if(nFlexStatus&0x40)//��������
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
               
               
              if(shoulder_detect_old != Input_GetVout())//��λ���
              {
                shoulder_detect = 1;
              }
     
                if(!back_up)   //��������������ߣ�
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
                                //����
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
                else if(!back_down)  //��������������
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
                                //����
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
                else   //�����������߸�λ
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
              //---------------ҡ�ڵ�����
            
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
                    if(oneKeyStep == 0)//�������е���������󣬼�������1 ����ʾ��ɵ��㲽�Զ�����
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
                    if(oneKeyStep == 0)//�������е���������󣬼�������1 ����ʾ��ɵ��㲽�Զ�����
                    {
                        oneKeyStep++;
                        engineerTimeCount = 1;
                    }
                }
#endif         
                
                
            }
            else//  if(oneKeyStep == 0)   һ���Զ�������ɵ�һ�����Ա�־
            {
                if(Input_GetWalkUpSwitch() != REACH_WALK_LIMIT && walk_up)//��ɵ�һ�����ԣ������е���ص�Ĭ�ϵ�λ��
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
                //==========================��ҡ�ڵ������λ
                if(Input_GetWaveMotorPosition() != REACH_WAVER_LIMIT)
                {
                   Main_WaveMotorStop();
                  
                  
                }

                
                
            }
            //���ֶ���ϵĲ���
            //����

            if(has_heat)
            {
                if(heat)WaistHeat_On();
                else WaistHeat_Off();
            }
            //�û�
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
            //����
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
            //����
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
                //10����Զ���һ��
                if(air_bagTimeCount == 0)// air_bagTimeCount %= 7 * TIME_COUNT; //7����һ��
                {
                    air_bagTimeCount++;//��ֹѭ�����ظ�����
                    enAirbagStep++;   //��������ʱÿһ�����ҵı���ʱ��Ϊ7��
                }
                //���Խ���
  #ifdef  RT8305T_1              
               if(enAirbagStep > 24)
                {
                    enAirbagStep = 0;//����
                    
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
                    enAirbagStep = 0;//����
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
            //�ֶ���ϲ���
            //1�����ȣ�2���û���3�����֣�4������,����ʱ���Զ�����
            if(oneKeyStep > 0 && oneKeyStep < oneKeyStepLength)
            {
                //10����Զ���һ��
                if(engineerTimeCount == 0)
                {
                    engineerTimeCount++;//��ֹѭ�����ظ�����
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
            if(bMasterSendPacket)//�ϴ���������ֿ���
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
                //OutBuffer[7] |= slide_backward;   //����
                //OutBuffer[7] |= slide_forward << 1;//����
                
                OutBuffer[7] |= flex_up << 2;
                OutBuffer[7] |= flex_down << 3;
                OutBuffer[7] |= foot_Switch << 4;
                OutBuffer[7] |= leg_angle << 5;
                OutBuffer[7] |= leg_ground << 6;
                
                OutBuffer[8] = 0;
                OutBuffer[8] |= _3D_Switch_Forward;
                OutBuffer[8] |= _3D_Switch_Back<<1;
                OutBuffer[8] |= _3D_Switch_Pluse<<2;
                //OutBuffer[8] |= waver_check<<3;//ҡ�ڵ������λ����
                
                OutBuffer[8] |= 1<<3;//ҡ�ڵ������λ����
                
                
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
        break;  //=����Ϊһ���Զ���������Ӳ���ӿ�
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
                    //Power_AMP_On();  //0.1���������
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
           
        case LINGO_SLIDE_TEST:    //�������������
            {
                Valve_Send_Data();
                Input_Proce();
                if(bUpKey) SlideMotorControl(STATE_RUN_SLIDE_FORWARD);//����   ,��һ���������� ʱ���������������е��г̿���λ�ô�
                if(bDownKey) SlideMotorControl(STATE_RUN_SLIDE_BACKWARD);//�������������
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
          
         case LINGO_WAVER_TEST:    //ҡ�ڵ������
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
                    //if(Input_GetWaveMotorPosition() == REACH_WAVER_LIMIT)  OutBuffer[1] |= 0x01;//ҡ�ڵ������λ
                    //else  OutBuffer[1] &= ~0x01;
                    //OutBuffer[1] = 0x01;
                     //OutBuffer[1] |= 0<<1;//ҡ���ٶ�
                     
                    
                  //  if(Input_GetSlideForwardSwitch() == REACH_SLIDE_LIMIT)  OutBuffer[1] |= 0x01;
                   // if(Input_GetSlideBackwardSwitch() == REACH_SLIDE_LIMIT) OutBuffer[1] |= 0x02;
                    OutBuffer[2] = EOI ;
                    nOutBufferCount = 3;
                    HandUart_Transmit_Packet(OutBuffer,nOutBufferCount);
                    bMasterSendPacket = FALSE ;
                }
            }
            break;           
            
            
            
            
            
        case LINGO_3D_TEST:  //3D���ǰ���г̲���
            {
                Valve_Send_Data();
                Input_Proce();
                // Main_MassageSignalSend();
                if(bUpKey) AxisMotor_Control(STATE_RUN_AXIS_FORWARD,0,_3D_SPEED_3);//�������������� ����ǰ
                if(bDownKey) AxisMotor_Control(STATE_RUN_AXIS_BEHIND,0,_3D_SPEED_3);//����Ϊ0  ������С  ����� AxisMotor_Control(STATE_RUN_AXIS_BEHIND,0,_3D_SPEED_8))
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
                     adcAxisCurrent=adcAxisCurrent*2;//��λΪ�����ĵ���ΪmA,���ص���350mA����
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
                    OutBuffer[3] = adcAxisCurrent >> 8 ;  //�ֿ����������Ĳ���ֵ����5
                    OutBuffer[4] = (unsigned char)adcAxisCurrent ;
                   
             //       OutBuffer[3] = w_current2 >> 8 ;  //�ֿ����������Ĳ���ֵ����5
               //     OutBuffer[4] = (unsigned char)w_current2 ;        
              
                    OutBuffer[5] = EOI ;
                    nOutBufferCount = 6;
                    HandUart_Transmit_Packet(OutBuffer,nOutBufferCount);
                    bMasterSendPacket = FALSE ;
                }
            }

            break;
        case LINGO_BACK_TEST:   //�����������
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
        case LINGO_LEG_TEST:   //С�����²���
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
                    

                    /*if(nFlexStatus&0x20) OutBuffer[1] |= 0x08;//�����Ƕȿ���
                    else  OutBuffer[1] &= ~0x08;
                     if(nFlexStatus&0x40) OutBuffer[1] |= 0x10;//��������
                     else OutBuffer[1] &= ~0x10;*/
                    
                   // if(nFlexStatus&0x20) OutBuffer[1] |= 0x08;//�����Ƕȿ���
                   // else  OutBuffer[1] &= ~0x08;
                   /* if(nFlexStatus&0x20) OutBuffer[1] |= 0x04;//�����Ƕȿ���
                    else  OutBuffer[1] &= ~0x04;                  
                    
                     if(nFlexStatus&0x04) OutBuffer[1] |= 0x10;//foot
                     else OutBuffer[1] &= ~0x10;  
                     
                     
                     if(nFlexStatus&0x40) OutBuffer[1] |= 0x08;//��������
                     else OutBuffer[1] &= ~0x08;  
                     */
                     /////////////////////////////////////////////////////////////////
                    if(nFlexStatus&0x04) OutBuffer[1] |= 0x04; //�����ŵ׿���
                    else OutBuffer[1] &= ~0x04;
                    
                    if(nFlexStatus&0x20) OutBuffer[1] |= 0x08;//�����Ƕȿ���
                    else OutBuffer[1] &= ~0x08;
                    
                     if(nFlexStatus&0x40) OutBuffer[1] |= 0x10;//��������
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
            
        case LINGO_FLEX_TEST:  //С����������
            {
                Valve_Send_Data();
                Input_Proce();
                Main_Send_Leg();
                if(bUpKey) 
                {
                  Flex_SetDisableAngle(1);
 //                 Flex_SetCurrent(FLEX_MOTOR_CURRENT_4A);
                   Flex_SetCurrent(FLEX_MOTOR_CURRENT_2A);
                  
                  Flex_SetDirection(FLEX_TO_OUT);  //FLEX_TO_OUT=1  С����
      
                }
                if(bDownKey) 
                {
                  Flex_SetDisableAngle(1);
 //                 Flex_SetCurrent(FLEX_MOTOR_CURRENT_4A);
                   Flex_SetCurrent(FLEX_MOTOR_CURRENT_2A);
                  
                 Flex_SetDirection(FLEX_TO_IN);  //С����
       
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
                    if(nFlexStatus&0x04) OutBuffer[1] |= 0x10; //�����ŵ׿���
                    else OutBuffer[1] &= ~0x10;
                    
                    if(nFlexStatus&0x20) OutBuffer[1] |= 0x04;//�����Ƕȿ���
                    else OutBuffer[1] &= ~0x04;
                    
                     if(nFlexStatus&0x40) OutBuffer[1] |= 0x08;//��������
                     else  OutBuffer[1] &= ~0x08;   
*/
                     
                     
                  
                    if(nFlexStatus&0x04) OutBuffer[1] |= 0x04; //�����ŵ׿���
                    else OutBuffer[1] &= ~0x04;
                    
                    if(nFlexStatus&0x20) OutBuffer[1] |= 0x08;//�����Ƕȿ���
                    else OutBuffer[1] &= ~0x08;
                    
                     if(nFlexStatus&0x40) OutBuffer[1] |= 0x10;//��������
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
            
        case LINGO_WALK_TEST: //���ߵ������
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
                    if(Input_GetVout() == BODY_TOUCHED)//����������ͷ��ΪON �ź�  VOUT=1
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
        case LINGO_ROLLER_TEST:   //���ֲ���
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
                //5V��ѹ
                OutBuffer[1] = (unsigned char)(adcVcc/100);
                OutBuffer[2] = (unsigned char)(adcVcc%100);
                //24V����ѹ
                OutBuffer[3] = (unsigned char)(adc24/100);
                OutBuffer[4] = (unsigned char)(adc24%100);
                //24V������ѹ
                OutBuffer[5] = (unsigned char)(adc24_1/100);
                OutBuffer[6] = (unsigned char)(adc24_1%100);
                //CPU�¶�
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
        case LINGO_AIRBAG:   //��������
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
            else//����������220V���²���ͬʱ����,,��������
            {
              //С���ϲ��ϵ��²��µ�ͬʱ����ʱ��190V������������������ͽź����190Vʱ����ͬʱ����

        //      if(airbagIndex==0)airbag=0xf0;  //������С��������180V������������ͬʱ�������������������ǿ��Ե�
           //      if(airbagIndex==0)airbag=0x11;
               //  if(airbagIndex==0)airbag=0x15;//190Vֻ�ܳ�2������
              
       //        if(airbagIndex==0)airbag=0x12;//��������������
            //     if(airbagIndex==0)airbag=0x07;
              
              
            //    if(airbagIndex==1)airbag=0x00;
                 
          /*         if(airbagIndex==8)airbag=0xc300;
                  if(airbagIndex==9)airbag=0x0000;
                 
                  if(airbagIndex==16)airbag=0x530000;
                  if(airbagIndex==17)airbag=0x000000;    //���ڲ�������*/
                  
                  
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
                Valve_AirPumpACPowerOn(); //�ֱ�
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
                            NVIC_SystemReset(); //��λCPU
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
        
    //     Main_Send_Leg(); //����С��BEEP����
        
        if(HandUart_GetCtrlType() != ENGGER_CTRL)
        {
            password = 0;  
            NVIC_SystemReset(); //��λCPU
        }
    }
    Main_Initial_Data(); //���³�ʼ������
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
    BlueToothUart_AMP_Volume_On();//��WIFI�ӿڵ�WIFI_IO3  ,�������Ѿ����ı�
    
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
        if(bTimer10MS == TRUE)//bTimer10MS=ȫ�ֱ���
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
        Main_Send();//�������ܿ�����С�Ȱ�ͨ��
        //BlueToothUart_AMP_Volume_On();
        Main_BlueToothSend();//����ͨ��
        
         Main_WaveMotorStop();
        
        
    }
}
//#define CURRENT_POINT_COUNT 10
//��λ���ʱ���ߵ�������ߵ���͵㣬3D������ȵ������Ȼ��ʼ��λ��⣬��ͬ�������겻һ��



#define CURRENT_POINT_COUNT 10

void Auto_Calibration(int detect3D )//bShoulderOK == 0)   // Auto_Calibration(0);  //����������֮ǰ���Ƚ������ͼ��BodyDetectStep=DETECT_INITIAL
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
         case BODY_DETECT_PREPARE:   //׼�� ֹͣ�û���� 3D�������ͷͣ����ǰ�棬��λ�� 
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
        case BODY_DETECT_WALK_POSITION:                    //���ߵ������

          
           //bWalk_OK = WalkMotor_Control(STATE_RUN_WALK_POSITION,WAIST_POSITION);//START_CHECK_POSITION);//,������λ������30������λ��
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
           //bWalk_OK = WalkMotor_Control(STATE_RUN_WALK_POSITION,START_CHECK_POSITION);//WAIST_POSITION);;,������λ������30������λ��
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
               nShoulderPosition = DEFAULT_SHOULDER_MAX_POSITION;//DEFAULT_SHOULDER_POSITION;//#define DEFAULT_SHOULDER_POSITION	190//��λ��Ӧ����3D����ʱ���ߵ���������ߵ����λ�ã�
               ShoulderSteps = BODY_DETECT_UP_AUTO;
             }
           }
           break;

         case BODY_DETECT_UP_AUTO:  //����������е�����λ��
           
            if(WalkMotor_Control(STATE_RUN_WALK_POSITION,nShoulderPosition))
             {
               /*  �÷ŷ�Ϊ��һ���Ҳ�����λ����λΪĬ�ϵ����λ��
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
            
            if(Input_GetVout() == BODY_TOUCHED)//��������۱���ѹ��ʱΪOFF ,��û�б�ѹ��ʱ���ߵ��粿λ������ʱΪON
            {
              //waitingcount++;
              if((Input_GetVout() == BODY_TOUCHED)&&(Input_GetWalkMotorPosition()>= LIMIT_POSITION))//&&(waitingcount>=1) )
              {
                WalkMotor_Control(STATE_WALK_IDLE,0);
                nShoulderPosition = Input_GetWalkMotorPosition();
                ShoulderSteps = BODY_DETECT_DATA_REFRESH;
             //���λ������   
                if(nShoulderPosition+LIMIT_PRECISION > TOP_POSITION)
                {
                   nShoulderPosition = TOP_POSITION;
    /*           //    printf("W2:%d,vout:%d\n",nShoulderPosition,Input_GetVout());              
 //                    printf("W2:%d,vout:%d\n",nShoulderPosition,BackMotor_Get_Location());   */ 
                }
                else
                {
                      nShoulderPosition += LIMIT_PRECISION;//���Զ������ڵ�һ������λ��ʱ���Ƿ��Ǽ�λ��ⲻ��LIMIT_PRECISION
/*
               //      printf("W2:%d,vout:%d\n",nShoulderPosition,Input_GetVout());
  //                    printf("W2:%d,vout:%d\n",nShoulderPosition,BackMotor_Get_Location());   */

                }
       //         printf("W2:%d,vout:%d\n",nShoulderPosition,Input_GetVout());
               //���λ���������  
              } 
            }
            else
            {
              //waitingcount =0;
            }
           break;  
         case BODY_DETECT_DATA_REFRESH:  //����ˢ��
           {
           
            BodyDataRefresh();
            if(WalkMotor_Control(STATE_RUN_WALK_POSITION,nShoulderPosition))  
            {
             ShoulderSteps = BODY_DETECT_ADJ;//BODY_DETECT_ADJ
             Timer_Counter_Clear(C_TIMER_5);
            }
           }
           break;
         case BODY_DETECT_ADJ:  //�ಱ�Ӳ���������λ��
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
            
            
            
            
            
            
            Re_DETECT_SHOULDER=0;// Ϊ�´μ�λ���ʧ����0
            
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
         case 0:   //׼��
           {
             KnockMotor_Break();
             if(KneadMotor_Control(STATE_KNEAD_STOP_AT_MED,KNEAD_SPEED2_PWM) && (AxisMotor_Control(STATE_RUN_AXIS_FORWARD,0,_3D_SPEED_8)))    
             {
               steps++;
             }
             positionCount = 0;
             positionTicks = nShoulderPosition/_3D_FULL_POINT;   //����3D�����ɼ���֮��������������� 
           }
           break;
         case 1:  
           if(WalkMotor_Control(STATE_RUN_WALK_POSITION,positionCount*positionTicks)) //��������λ�õ�
           {
             steps++;
           }
           break;
         case 2:  //3D����˶��������   
           {
             if(AxisMotor_Control(STATE_RUN_AXIS_BEHIND,0,_3D_SPEED_8))
             {
               steps++;
               AxisMotor_UpdataPosition();
             }
           }
           break;    
         case 3:  //3D����˶�����ǰ��  
           {
             if(AxisMotor_Control(STATE_RUN_AXIS_FORWARD,0,_3D_SPEED_8))
             {
               steps++;
             }
             AxisMotor_StorageCurrent(positionCount,positionCount*positionTicks);  //�洢����ֵ11*40����
           }
           break;
         case 4:  //���ݼ�¼�봦��
           {
             positionCount++;
             if(positionCount > _3D_FULL_POINT )  //һ���ɼ�����_3D_FULL_POINT+1��
             {
               steps++;
               break;
             }
             steps = 1; 
           }
           break;
         case 5:  //����
           {
             nChairRunState = CHAIR_STATE_RUN;
             BodyDetectStep = DETECT_FINISH;
              // bodyDectect = 1;
             AxisMotor_CurrentAdj();
           }
           break;  
         }//end switch
         } // end if
   /***************�����˳���**************************/
}


//�洢ģʽ �������رգ����ֹرգ��ȣ� δ�����������綯����ص���Ĺر�
//δ����С�����������״̬
//         nChairRunState = CHAIR_STATE_SETTLE;  //��Ħʱ�䵽
//         nSettleMode = RUN_OVER_RESET;

//                if(bUpKey) AxisMotor_Control(STATE_RUN_AXIS_FORWARD,0,_3D_SPEED_3);//�������������� ����ǰ
//                if(bDownKey) AxisMotor_Control(STATE_RUN_AXIS_BEHIND,0,_3D_SPEED_3);//����Ϊ0  ������С  ����� AxisMotor_Control(STATE_RUN_AXIS_BEHIND,0,_3D_SPEED_8))

void Main_Settle(void)
{
    bool bEngineeringSettle = ReadEEByte(USER_DATA_BASE+SETTLE_ADDRESS);
    bool waver_finish=0;
    
    unsigned int steps2 = 0;
    BYTE key;
    //������ʼ������
    //������ʼ������
    Power_All_On();
 //   VoiceUart_PowerOff();  //��λ������������������    delete by taoqignsong
    bKeyBackPadUp = FALSE ;
    bKeyBackPadDown = FALSE ;
    bKeyLegPadUp = FALSE ;
    bKeyLegPadDown = FALSE ;
    nChairRunState = CHAIR_STATE_SETTLE ;//��Ħ�δ����ղ�״̬
    
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
          //        nChairRunState = CHAIR_STATE_SETTLE;  //��Ħʱ�䵽
    //     nSettleMode = RUN_OVER_RESET;
     
    //bEngineeringSettle=0���������н������Ӳ���λ�������µ�Դ��ʱ���Ӹ�λ
    if(bEngineeringSettle || nSettleMode == POWER_KEY_RESET) //���Ϊ�棬��ػ���λ������ػ�����λ
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
    //��ѭ��
    while(nChairRunState == CHAIR_STATE_SETTLE)
    {
      //����������
        key = Main_GetKey();
        if(key != H10_KEY_NONE)
        {
          Power_All_Off();

          bBackLegPadSettle = FALSE;
          nChairRunState = CHAIR_STATE_SLEEP; 
   
          
        }
        
    /*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */    
      //ʱ�䴦����
       if(Timer_Counter(C_TIMER_INDICATE + T_LOOP,SETTLE_INDICATE_TIME))
        {
          
          IndicateLED_Toggle();
        }
       Input_Proce();
       Valve_Send_Data();
       Main_Send();//����Main_Send_Leg();
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
         case 0:   //�������ͣ�����
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
             if( AxisMotor_Control(STATE_RUN_AXIS_VECTOR,2,_3D_SPEED_5))//3D���м�λ��
            {
                steps2++;    
            }  
           break;
          
          
         case 3:   //�������ͣ�����г̿���λ��
            if(WalkMotor_Control(STATE_RUN_WALK_UP,0))
            {
             steps2++;    
            }  
           break;
           
         case 4:   //�������ͣ�ڸ�λλ��
            if(WalkMotor_Control(STATE_RUN_WALK_POSITION,RESET_POSITION))
            {
             steps2++;    
            }  
           break;
             
        case 5: 
           if( AxisMotor_Control(STATE_RUN_AXIS_BEHIND,0,_3D_SPEED_8))//����Ϊ0  ������С 3D ֹͣ�ڿ���λ��
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
          nChairRunState = CHAIR_STATE_SLEEP;   //2����ʱ�䱣��
        }
        
        /* 
        //�ж����еĵ綯�׺ͻ�о�Ƿ�λ
        if((Input_GetBackUpSwitch() == REACH_BACK_LIMIT) &&
                (Input_GetLegDownSwitch() == REACH_LEG_LIMIT) &&
                   (Input_GetSlideBackwardSwitch() == REACH_SLIDE_LIMIT) &&
                     (steps == 100))
            {
               nChairRunState = CHAIR_STATE_SLEEP; 
            }
        */
        
    } //end while
   /***************�����˳���**************************/
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
    //������ʼ������
    //������ʼ������
    Power_All_On();
   //  VoiceUart_PowerOn();//  delelt by taoqingsong
    bBackLegPadSettle = FALSE ;
    bKeyBackPadUp = FALSE ;
    bKeyBackPadDown = FALSE ;
    bKeyLegPadUp = FALSE ;
    bKeyLegPadDown = FALSE ;
    nCurSubFunction = BACK_SUB_MODE_NO_ACTION;
    nChairRunState = CHAIR_STATE_WAIT_COMMAND ;//��Ħ�εȴ���������
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
    //��ѭ��
    BlueToothUart_AMP_Volume_On();
    while(nChairRunState == CHAIR_STATE_WAIT_COMMAND)
    {
      //����������
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
                  Valve_SetAirBagStrength(AIRBAG_STRENGTH_1);//�趨��������Ϊ3
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
				break;   // ����ϴ���PAUSE ��ô�Ͳ��ܰ�STOP
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
              
              
              
            case H10_KEY_POWER_SWITCH: //�ٰ�һ�ε�Դ�����ӽ��뵽��λ�洢״̬
              
                bBackLegPadSettle = TRUE ;
                nChairRunState = CHAIR_STATE_SETTLE ;
                nSettleMode = POWER_KEY_RESET;                 
                                       
               nTargetMassagePosition = MASSAGE_RESET_POSITION;
               bMassagePositionUpdate = TRUE;
               w_ZeroPosition = 0; //�����������λ
                           
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
            case H10_KEY_CHAIR_AUTO_0:             //���ӽ��뵽����״̬����������뵽�Զ�״̬,������Զ�״̬ΪAUTO_1
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
              
                
                
                
                
            case H10_KEY_3DMODE_1://  //3D�˵��е�3D�ַ�1       //���ӽ��뵽����״̬����������뵽��3D״̬,������Զ�״̬Ϊ3D1
              nBackSubRunMode = BACK_SUB_MODE_3D1;
              nBackMainRunMode = BACK_MAIN_MODE_3D;
              nChairRunState = CHAIR_STATE_RUN ;
              nKeyBackLocate =  LOCATE_FULL_BACK;
              
              
              break;
            case H10_KEY_3DMODE_2://3D�˵��е�3D�ַ�2
              nBackSubRunMode = BACK_SUB_MODE_3D2;
              nBackMainRunMode = BACK_MAIN_MODE_3D;
              nChairRunState = CHAIR_STATE_RUN ;
                nKeyBackLocate =  LOCATE_FULL_BACK;
              
              break;  
            case H10_KEY_3DMODE_3:////3D�˵��е�3D�ַ�3
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
      //     case H10_KEY_ZERO_START://�ڵȴ�����ģʽ������������Ч��Ϊȷ�ϼ�
      //       break;
           case H10_KEY_ZERO_START://��������ֵ  wait
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
                w_ZeroPosition %= 3;  //0λ��Ϊ��ʼλ��   1λ��Ϊ��һ����������    2λ��Ϊ�ڶ�����������    
                
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
                
            case  H10_KEY_3D_STRENGTH:    //�ܿ����Զ����뵽3D�˵�
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
              nChairRunState = CHAIR_STATE_RUN ;////�����ɵȴ�������뵽����״̬
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
         /*   case H10_KEY_OZON_SWITCH:////������
              bOzonEnable = TRUE;
              nChairRunState = CHAIR_STATE_RUN ;////�����ɵȴ�������뵽����״̬
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
                  Valve_SetAirBagStrength(AIRBAG_STRENGTH_1);//�趨��������Ϊ3
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
              
              
              
              
            case H10_KEY_AIRBAG_LEG:    //ѡ���Ƚ����Ұ�Ħ����
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

            case H10_KEY_AIRBAG_WAIST://�������Ұ�Ħ����
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
            case H10_KEY_AIRBAG_BUTTOCKS:  //ƨ�����Ұ�Ħ����
              nKeyAirBagLocate = AIRBAG_LOCATE_SEAT ;
              st_AirBagSeat.init = TRUE ;
              if( Valve_GetAirBagStrength() == AIRBAG_STRENGTH_0)
              {
                Valve_SetAirBagStrength(AIRBAG_STRENGTH_1);
              }
              nChairRunState = CHAIR_STATE_RUN ;////�����ɵȴ�������뵽����״̬
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
              bLegPadLinkage = TRUE ;////С�ȵ���������ʱ������ǰ���綯�׵�λ��
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
              bKeyBackPadDown = TRUE ;//�������°������µı�־λ
              bKeyLegPadDown = FALSE ;
              bKeyLegPadUp = TRUE ;
              bLegPadLinkage = TRUE ;
              bKeyFlexOut = FALSE ;
              bKeyFlexIn = FALSE ;bKeyFlexIn = FALSE ;bZeroflash = FALSE;
              break ;
            case H10_KEY_BACKPAD_DOWN_STOP://�ͷŰ������͵ļ�ֵ
              bKeyBackPadUp = FALSE ;
              bKeyBackPadDown = FALSE ;
              bKeyLegPadDown = FALSE ;
              bKeyLegPadUp = FALSE ;
              bLegPadLinkage = TRUE ;
              bKeyFlexOut = FALSE ;
              bKeyFlexIn = FALSE ;
              break ;
            case H10_KEY_LEGPAD_EXTEND_START://С����
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
                nChairRunState = CHAIR_STATE_RUN ;////�����ɵȴ�������뵽����״̬
                if(Data_Get_Time() == 0)
                {
                  Data_Set_Start(1, w_PresetTime);
                }
              }
              nBuzzerMode = BUZZER_MODE_ONETIME ;
              bSendBuzzerMode = TRUE ;
              break;
            case H10_KEY_HEAT:    //����
              if(bKeyWaistHeat == FALSE)
              {
                bKeyWaistHeat = TRUE ;
                nChairRunState = CHAIR_STATE_RUN ;//�����ɵȴ�������뵽����״̬
              }
              else
              {
                bKeyWaistHeat = FALSE ;
              }
              nBuzzerMode = BUZZER_MODE_ONETIME ;
              bSendBuzzerMode = TRUE ;
              break;     
  //-------------------------------------------------------------            
         /*   case H10_KEY_SEAT_VIBRATE_0:   //ҡ�ڰ��� ҡ�ڹ�
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
      //ʱ�䴦����
        
        if( bBootlooth10ms == true)
        {
          bBootlooth10ms = false;
          USB_MP3_SCAN_PROC();
        }
        
        
        
        
        if(bEnableDemo)
        {
          if(Timer_Counter(C_TIMER_500MS,50))//5SEC�����DEMO����
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
       //������������ֶ�����
        Main_BackPad_Proce();
        //С����������ֶ�����
        Main_LegPad_Proce();
        //С����������ֶ�����
        
  #ifdef RT8305T_1  
         Main_FlexPad_Proce();
   #endif     
         //��(ҡ��)����
      //    Main_VibrateMotorControl() ;   
          
      //    if(Input_GetWaveMotorPosition() == REACH_WAVER_LIMIT)  
      //    {
      //      nWaveOverTime = 0;
     //      }      
      //    
        //===========================
        Main_Massage_Position_Proce();//wait ���Ӹ�λʱ�����е��״̬,��������������Ŀ���
        
  #ifdef RT8305T_1  
        FlexMotorFollowingFood();//�Զ��ҽų���
  #endif  
        Main_Send_Leg();
        Problem_Proce();//���3D ,������������ߵ���Ƿ����г�ʱ,�����Ƿ�����ҡ�ڵ���쳣������ʾ
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
   /***************�����˳���**************************/
}
//��ʼ�Զ�����
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
  bBackAutoModeInit = TRUE ;//ѡ���Զ�����ʱ ���Զ����п��������ʼ����־
  //�������ҹ���
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
  //�������ҹ���
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
  nCurSubFunction = BACK_SUB_MODE_NO_ACTION;  //20150303���ӣ���ֹ��ʾ����
  
  
}

//��ʼ�ֶ�����
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
      //���ñ�������
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
    //����������
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

    
    
    
    
    //���ô������
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
      //���ñ�������
      BackManualModeNoAction() ;
      break ;
    }
    nBackSubRunMode = BACK_SUB_MODE_KNOCK ;
    //���ݵ�ǰ��״̬����nKeyKneadKnockSpeed,nKeyBackLocate
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
    //����������
    bKneadMotorInProcess = TRUE ;
    nKneadMotorControlParam1 = ManualDirector[nCurActionStep].nKneadMotorState ;
    nKneadMotorControlParam2 = ManualDirector[nCurActionStep].nKneadMotorCycles ;
    //���ô������
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
      //���ñ�������
      BackManualModeNoAction() ;
      break ;
    }
    nBackSubRunMode = BACK_SUB_MODE_WAVELET ;
    //���ݵ�ǰ��״̬����nKeyKneadKnockSpeed,nKeyBackLocate��nKeyKneadWidth
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
    //����������
    bKneadMotorInProcess = TRUE ;
    nKneadMotorControlParam1 = ManualDirector[nCurActionStep].nKneadMotorState ;
    nKneadMotorControlParam2 = ManualDirector[nCurActionStep].nKneadMotorCycles ;
    //���ô������
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
      //���ñ�������
      BackManualModeNoAction() ;
      break ;
    }
    nBackSubRunMode = BACK_SUB_MODE_SOFT_KNOCK ;
    //���ݵ�ǰ��״̬����nKeyKneadKnockSpeed,nKeyBackLocate
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
    //����������(�������¶���)
    bKneadMotorInProcess = TRUE ;
    nKneadMotorControlParam1 = ManualDirector[nCurActionStep].nKneadMotorState ;
    nKneadMotorControlParam2 = ManualDirector[nCurActionStep].nKneadMotorCycles ;
    //���ô������(�������¶���)
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
      //���ñ�������
      BackManualModeNoAction() ;
      break ;
    }
    nBackSubRunMode = BACK_SUB_MODE_PRESS ;
    //���ݵ�ǰ��״̬����nKeyKneadKnockSpeed,nKeyBackLocate
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
    //����������
    bKneadMotorInProcess = TRUE ;
    nKneadMotorControlParam1 = ManualDirector[nCurActionStep].nKneadMotorState ;
    nKneadMotorControlParam2 = ManualDirector[nCurActionStep].nKneadMotorCycles ;
    //���ô������
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
      //���ñ�������
      BackManualModeNoAction() ;
      break ;
    }
    nBackSubRunMode = BACK_SUB_MODE_PRESS ;
    //���ݵ�ǰ��״̬����nKeyKneadKnockSpeed,nKeyBackLocate
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
    //����������
    bKneadMotorInProcess = TRUE ;
    nKneadMotorControlParam1 = ManualDirector[nCurActionStep].nKneadMotorState ;
    nKneadMotorControlParam2 = ManualDirector[nCurActionStep].nKneadMotorCycles ;
    //���ô������
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
      //���ñ�������
      BackManualModeNoAction() ;
      break ;
    }
    nBackSubRunMode = BACK_SUB_MODE_MUSIC ;
    //���ݵ�ǰ��״̬����nKeyKneadKnockSpeed,nKeyBackLocate
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
    //����������(�������¶���)
    bKneadMotorInProcess = TRUE ;
    nKneadMotorControlParam1 = ManualDirector[nCurActionStep].nKneadMotorState ;
    nKneadMotorControlParam2 = ManualDirector[nCurActionStep].nKneadMotorCycles ;
    //���ô������(�������¶���)
    bKnockMotorInProcess = TRUE ;
    nKnockMotorControlParam1 = ManualDirector[nCurActionStep].nKnockMotorState ;
    nKnockMotorControlParam2 = ManualDirector[nCurActionStep].nKnockMotorRunTime ;
    nKnockMotorControlParam3 = ManualDirector[nCurActionStep].nKnockMotorStopTime ;
    
    nMaxActionStep = 4 ;
    nStartActionStep = 0 ;
    bBackManualModeInit = TRUE ;
    break;
    
  default:
    //���ñ�������
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
    else if(nBackMainRunMode == BACK_MAIN_MODE_3D) //3D��Ħ
    {
     Main_Start_3D();
    }
    else if(nBackMainRunMode == BACK_MAIN_MODE_MANUAL) //�ֶ�
    {
       bMassagePositionUpdate = false;bZeroflash = FALSE;
       Main_Start_Manual(); 
    }
    //��ѭ��
    while(CHAIR_STATE_RUN == nChairRunState)
    {
      //����������
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
				break;   // ����ϴ���PAUSE ��ô�Ͳ��ܰ�STOP
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
        case H10_KEY_POWER_SWITCH: //��������ʱ���µ�Դ��
          {
            //��Ħ������λ
            nChairRunState = CHAIR_STATE_SETTLE ;
            nSettleMode = POWER_KEY_RESET;            
            
            nTargetMassagePosition = MASSAGE_RESET_POSITION;
            bMassagePositionUpdate = TRUE;
            w_ZeroPosition = 0; //�����������λ        

            
            
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
          if(BodyDetectStep != DETECT_FINISH)//���м�λ ���
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
            BodyDetectStep = DETECT_INITIAL;////bShoulderOK == 0)   //����������֮ǰ���Ƚ������ͼ��BodyDetectStep=DETECT_INITIAL
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
        case H10_KEY_CHAIR_AUTO_5:  //���������ͼ��
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
              //BodyDetectStep = DETECT_FINISH;  //���������ͼ��
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
              //BodyDetectStep = DETECT_FINISH;  //���������ͼ��
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
              //BodyDetectStep = DETECT_FINISH;  //���������ͼ��
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
              //BodyDetectStep = DETECT_FINISH;  //���������ͼ��
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
                                     
         case H10_KEY_3DMODE_1:   //3D�ַ�1
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
        case H10_KEY_3DMODE_2:// //3D�ַ�2

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
        case H10_KEY_3DMODE_3:// //3D�ַ�3
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
          w_ZeroPosition %= 3;  //0λ��Ϊ��ʼλ��   1λ��Ϊ��һ����������    2λ��Ϊ�ڶ�����������    
          
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
          
          
          
          
          
        case  H10_KEY_3D_STRENGTH:     //3D�����ֿ����ϵĿ�ݼ�
#ifdef test_shoulder
           bShoulderOK =0;
          
           BodyDetectStep = DETECT_INITIAL;

          
#else         
          if((nBackMainRunMode == BACK_MAIN_MODE_MANUAL) || (nBackMainRunMode == BACK_MAIN_MODE_AUTO))
          {
            nKeyAxisStrength++;
            nKeyAxisStrength %= 5;
            bAxisUpdate = TRUE;   //3D������߱�־λ��ΪTRUEʱ��3D�������
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
            //���ñ�������
            BackManualModeNoAction() ;
            
          }
          //�������ҹ���
          //��������ʱ��
          
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
         case H10_KEY_LOCATE_PART:  //���ֿ���Ϊ�ֲ�
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
            //���¶�λ
            nKneadMotorControlParam1 = ManualDirector[0].nKneadMotorState ;
            nKneadMotorControlParam2 = 0 ;
            bKneadMotorInProcess = TRUE ;
            //Knock motor Ҫ�ȶ�λ��ɺ����
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
              
              
              
              
          
        case H10_KEY_AIRBAG_LEG:   //�����˶�����,���Ұ�Ħ����,��Ӧoutbuf[12]2.3.4λ��ѡ�е����Ұ�Ħ����,��Ӧͨ��Э��� ��ַ12 ʱ�������
          if(nKeyAirBagLocate == AIRBAG_LOCATE_AUTO)//ȫ�����ҳ���
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
            //��
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
            //ȥ
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

       case H10_KEY_AIRBAG_WAIST://�������ҳ���
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
        case H10_KEY_AIRBAG_BUTTOCKS:   //�������ҳ���
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
            //��
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
            //ȥ
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
          
          

          
        case H10_KEY_AIRBAG_ARM://�ۼ�����   ʵ��ʹ�õ��Ǳۼ�����
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
            //��
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
            //ȥ
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
        case H10_KEY_WALK_UP_START://�û������������߰���
        
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
        case H10_KEY_WALK_UP_STOP:// �ֶ����㰴Ħ�ͷŰ���ֵ
          
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
        case H10_KEY_BACKPAD_UP_START:  //�û����¿������ϰ�������ʱ��С�ȴ�ʱҲҪ�������У����趨������־λ
          st_Stretch.active = FALSE;
          RockFunctionEnable(false);
          st_Stretch.active = FALSE;SetStretchingEnable(0);
          bKeyBackPadUp = TRUE ;
          bKeyBackPadDown = FALSE ;
          //С����������
          bKeyLegPadDown = TRUE ;  //�趨С�����°�����־λ
          bKeyLegPadUp = FALSE ;
          bLegPadLinkage = TRUE ;    //С��������־λ
          bKeyFlexOut = FALSE ;
          bKeyFlexIn = FALSE ;bKeyFlexIn = FALSE ;bZeroflash = FALSE;
          break ;
        case H10_KEY_BACKPAD_UP_STOP:  //�û��ͷſ������ϰ���
          RockFunctionEnable(false);
          st_Stretch.active = FALSE;SetStretchingEnable(0);
          st_Stretch.active = FALSE;
          bKeyBackPadUp = FALSE ;
          bKeyBackPadDown = FALSE ;
          //С����������
          bKeyLegPadDown = FALSE ;
          bKeyLegPadUp = FALSE ;
          bLegPadLinkage = TRUE ;
          bKeyFlexOut = FALSE ;
          bKeyFlexIn = FALSE ;
          break ;
         //-----------------------------------------------------------------------
        case H10_KEY_BACKPAD_DOWN_START://������������
          RockFunctionEnable(false);
          st_Stretch.active = FALSE;SetStretchingEnable(0);
          st_Stretch.active = FALSE;
          bKeyBackPadUp = FALSE ;
          bKeyBackPadDown = TRUE ;
          //С����������
          bKeyLegPadDown = FALSE ;
          bKeyLegPadUp = TRUE ;   //С����������
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
          //С����������
          bKeyLegPadDown = FALSE ;
          bKeyLegPadUp = FALSE ;
          bLegPadLinkage = TRUE ;
          bKeyFlexOut = FALSE ;
          bKeyFlexIn = FALSE ;
          break ;
          //-----------------------------------------------------------------------
        case H10_KEY_LEGPAD_EXTEND_START://С��������
          RockFunctionEnable(false);
          st_Stretch.active = FALSE;SetStretchingEnable(0);
          st_Stretch.active = FALSE;
          bKeyLegPadUp = FALSE ;
          bKeyLegPadDown = FALSE ;
          bLegPadLinkage = FALSE ;
          bKeyBackPadUp = FALSE ;
          bKeyBackPadDown = FALSE ;
          bKeyFlexOut = TRUE ;    //С��������
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
          bKeyFlexIn = TRUE ;//xС��������

          
          
          break;
          //--------------------------------------------------
        case H10_KEY_LEGPAD_UP_START://С������
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
        case H10_KEY_WHEEL_SPEED_OFF://�ŵ׹����ٶ�Ϊ0
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
            if(nKeyAirBagLocate == AIRBAG_LOCATE_AUTO) break; //���Զ����ҳ����й��ٶȲ����Ե���
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
        case H10_KEY_HEAT:    //����
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
         case H10_KEY_SEAT_VIBRATE_0:   //ҡ�ڰ��� ҡ�ڹ�
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
         nChairRunState = CHAIR_STATE_SETTLE;  //��Ħʱ�䵽
         nSettleMode = POWER_KEY_RESET;   
          }
          else
          {
         nChairRunState = CHAIR_STATE_SETTLE;  //��Ħʱ�䵽
         nSettleMode = RUN_OVER_RESET;              
          }
*/
         
         
         
       }
      //ʱ�䴦����
       if(Timer_Counter(C_TIMER_RUN + T_LOOP,1))//100ms timer couter
       {
         if(nAxisUpdateCounter<255) nAxisUpdateCounter++;
         nCurActionStepCounter++ ;//��ǰ����ʱ����������߻��û�ʱ�������,���ߺ��û���ﹲ��һ��ʱ������������û�������ʼʱ�� ������ﲻ���Լ���������ֹͣ�ڵ�ǰλ��
         nCurShoulderAdjustCounter++ ;
         nCurKnockRunStopCounter++ ;
         nCur3D_MotorStopCounter++;//3D�����Ŀ��λ�õ�ֹͣʱ��������3D�Զ�������
         //���ҳ�������ʱ�������
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
         n3DMotorRunCounter++;   //3D�������ʱ��������������ʹ��
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
       if(bKeySeatVibrate)//����ҡ�ڴ���
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
       Valve_Send_Data();//��������
       Main_Send();//�ϴ���Ħ��Ϣ���ֿ���(����Main_Send_Leg();)
       Main_BlueToothSend();//�ϴ���Ħ��Ϣ��ƽ��
       //Main_MassageSignalSend();
       LED_RGB_Proce(nChairRunState);
       main_GetKneadPosition();//��ȡ�������Ŀ��С�խλ��
       Data_Time_Counter_Proce();
        
       Main_Walk_Beep_Proce();//���ߵ��BEEP����
       
        //������������ֶ�����
       Main_BackPad_Proce();//�����������,����
       
       
        //С����������ֶ�����  ������
       Main_LegPad_Proce();
       
              //ҡ�ڴ���
       RockProcess();
        //С����������ֶ�����
         
 #ifdef RT8305T_1
       Main_FlexPad_Proce();//    Main_Send_Leg();
#endif
    //      Main_BackProce();//��о���� ,�����Զ����������
       
       //ҡ�ڵ������
       Main_VibrateMotorControl();
       if(Input_GetWaveMotorPosition() == REACH_WAVER_LIMIT)  
       {
          nWaveOverTime = 0;
       }           
       
       Main_Massage_Position_Proce();//work  ����������
       
#ifdef RT8305T_1   
       FlexMotorFollowingFood();
#endif     
       
       
       Main_Valve_Proce();//��������
       Problem_Proce();
       MusicSampling();
       
       

       Main_ReStartShoulderCheck();
       
       
      switch(nBackMainRunMode)
      {
      case  BACK_MAIN_MODE_AUTO:  
        {
          if(bShoulderOK == 0)   // Auto_Calibration(0);  //����������֮ǰ���Ƚ������ͼ��
          {
            Auto_Calibration(0);  //����������֮ǰ���Ƚ������ͼ��
          }
          else
          {
                      
            
            Main_BackProce();//��о���� ,�����Զ����������
            //        n3D_MotorControlState=  _3D_MANUAL,OR PROGRAME  ,//�ֶ����Զ�
            _3DMotorControl(n3D_MotorControlState,n3D_MotorControlPosition,n3D_MotorControlSpeed,n3D_MotorControlStopTime);   
            WalkMotorControl(nWalkMotorControlParam1,nWalkMotorControlParam2) ;//���ߵ��������ڵ�ǰλ��ֹͣʱ��
            KneadMotorControl(nKneadMotorControlParam1,nKneadMotorControlParam2) ;//����������״̬��������Ȧ��
            KnockMotorControl(nKnockMotorControlParam1,nKnockMotorControlParam2,nKnockMotorControlParam3) ;//�ô���״̬������ʱ�䣬ͣ��ʱ��
          }
        }
        break;
      case  BACK_MAIN_MODE_3D:  
        {
          if(bShoulderOK == 0)    
          {
            Auto_Calibration(0);  //����������֮ǰ���Ƚ������ͼ��
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
      
      
         //���ȴ���
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
   /***************�����˳���**************************/
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
    //������ʼ������
    //������ʼ������
    Power_All_On();
    Timer_Counter_Clear(C_TIMER_INDICATE);
    Timer_Counter_Clear(C_TIMER_TEMP);
    //��ѭ��
    while(CHAIR_STATE_IDLE == nChairRunState)
    {
      //����������
        key = Main_GetKey();
        key &= 0x7f;
        if(H10_KEY_POWER_SWITCH == key)
        {
          nChairRunState = CHAIR_STATE_WAIT_COMMAND; //���˵�Դ����
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
        
        
        
      //ʱ�䴦����
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
   /***************�����˳���**************************/
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
    //��ѭ��
    while(CHAIR_STATE_DEMO == nChairRunState)
    {
        //����������
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
          w_ZeroPosition %= 3;  //0λ��Ϊ��ʼλ��   1λ��Ϊ��һ����������    2λ��Ϊ�ڶ�����������    
          
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
      //ʱ�䴦����
       if(Timer_Counter(C_TIMER_RUN + T_LOOP,1))
       {
         nCurActionStepCounter++ ;
         nCurShoulderAdjustCounter++ ;
         nCurKnockRunStopCounter++ ;
         nCur3D_MotorStopCounter++;
         //���Ҽ�����
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
        //������������ֶ�����
       Main_BackPad_Proce();
        //С����������ֶ�����
       Main_LegPad_Proce();
        //С����������ֶ�����
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
         //���ȴ���
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
   /***************�����˳���**************************/
}







/*******************************************************
��Ħ�γ�ʼ������ 3D ��︴λ

********************************************************/
void Main_Initial(void)
{
    bool bMassageSignalOK = false;
//    bool bLegSignalOK = false;
    bool b3DMotorInit = false;
   // unsigned short _3D_Current = 0,adcAxisCurrent;
   // BYTE key;
    //������ʼ������
    //������ʼ������
    Power_All_On();
    Timer_Counter_Clear(C_TIMER_INDICATE);
    Timer_Counter_Clear(C_TIMER_TEMP);
    //��ѭ��
    while(CHAIR_STATE_INITIAL == nChairRunState)
    {
      //����������
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
      
      
      
      
      bMassageSignalOK = LEUART0_isOK();//��ʼ�����3D��о�������������Ƿ�����
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
      //ʱ�䴦����
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
   /***************�����˳���**************************/
   AxisMotor_Control(STATE_AXIS_IDLE,0,_3D_SPEED_10);
}

void Main_Problem(void)
{
    BYTE key;
    //������ʼ������
    //������ʼ������
    Power_All_On();
    Timer_Counter_Clear(C_TIMER_INDICATE);
    Timer_Counter_Clear(C_TIMER_TEMP);
    //��ѭ��
    while(CHAIR_STATE_PROBLEM == nChairRunState)
    {
      //����������
        key = Main_GetKey();//problem
        if(H10_KEY_POWER_SWITCH == key)
        {
          nChairRunState = CHAIR_STATE_WAIT_COMMAND; //���˵�Դ����
        }
      //ʱ�䴦����
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
   /***************�����˳���**************************/
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
  if((bLegpositiondone == true)&&(bBackpositiondone == true)) return true;  //������Ԥ����λ��
  else return false;  //δ����Ԥ����λ��

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
    WorkStep = 0;//һ���ս����ģʽ�Ϳ�ʼ��������
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
  //����ģʽ��
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
      //�ص���һ������״̬
      nTargetMassagePosition = MASSAGE_OPTIMAL2_POSITION;
      bMassagePositionUpdate = TRUE;
      //w_ZeroPosition = 1;
    }
  }
  //û����ҡ��ģʽʱ��ֱ���˳�
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
  //������������0
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
  //������������1
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
  
  //������������2
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
  
  //������������2
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
�жϵ綯���Ƿ��������
*******************************/

 /*       if((SlideMotor_GetPower() == SLIDE_MOTOR_POWER_ON)||(BackMotor_GetPower() == BACK_MOTOR_POWER_ON)||(LegMotor_GetPower() == LEG_MOTOR_POWER_ON))
        {
          bVibratePause=1;//ҡ�ڵ����ͣ����
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
static unsigned int w_old_Positon=1700;//��ʼ���������ֵ1600
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
             if(w_cur_Postion <= (w_old_Positon + 200))return;//2sec�ڲ�������  200*0.01=2sec
               
           
         }
         else
         {
            if(w_old_Positon <= (w_cur_Postion + 200))return;//2sec�ڲ�������
           
         }
         
         if(w_cur_Postion>500)
         {
           
             if(w_cur_Postion>=w_old_Positon)
             {
               w_delta_Postion=w_cur_Postion-w_old_Positon;///100;//��λ1000ms
      
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
                w_delta_Postion=(w_cur_Postion-w_old_Positon);///100;//��λ1000ms
      
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
//��ʼ����
unsigned char AES_PlantTest1[16] = "This aaaaa text";
//ʹ�øñ������鿴����֮�������
unsigned char AES_PlantTest2[16];
//ʹ�øñ������鿴����֮�������
unsigned char AES_PlantTest3[16];
//��Կ
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

const unsigned char AES_PlantTest1[16] = "Rongtai Health";//����

void AES_ECB_128bit_Encrypt(void)
{
  static unsigned char AES_PlantTest2[16];//ʹ�øñ������鿴����֮�������
  static  unsigned char AES_PlantTest3[16];//ʹ�øñ������鿴����֮�������
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
 //   AES_ECB128(AES_PlantTest2,AES_PlantTest1,16,g_ucKey,true);//����
        __no_operation();
      __no_operation();
      __no_operation();
      __no_operation();  
      __no_operation();
      __no_operation();     
    AES_DecryptKey128(g_ucKey,g_ucKey);//���������Կ
   //          ����֮�������  ����֮�������
    AES_ECB128(AES_PlantTest3,AES_PlantTest2,16,g_ucKey,false);//����AES 128 λ��ECB ����
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
  SCB->VTOR = (uint32_t)(8 * 1024);    //����
  if(__checksum == 0) __checksum = 1;//ַ��Ϊ0������Ҫ����BOOTLOADER
  
  
  
   // Main_Auto_Program_Test();
     //     AxisMotor_10msInt();//20MS�����������Զ�ֹͣ  ,Ŀǰ8600S�������Ѿ����Ӹñ���
   //     WalkMotor_10msInt();  //���ߵ����������20ms���Զ��ر����ߵ�� Ŀǰ8600S�������Ѿ����Ӹñ���
    Main_Initial_IO(); //Ӳ����ʼ��

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
        case CHAIR_STATE_IDLE:          Main_Idle();break;//����ģʽ���򿪰�������ȴ�����ģʽ��10���ް�������˯��=//����״̬,���ϵ�ϵͳ�������ģʽ��Main_Sleep��,Ҳ���Խ��뵽����ģʽ
        case CHAIR_STATE_SETTLE:        Main_Settle();break;
        case CHAIR_STATE_WAIT_COMMAND:  Main_WaitCommand();break; 
        case CHAIR_STATE_RUN:           Main_Work();break; 
        case CHAIR_STATE_PROBLEM:       Main_Problem();break;  
        case CHAIR_STATE_ENGINEERING:   Main_Engineering();break;
        case CHAIR_STATE_SLEEP:         Main_Sleep();break;//���а�������ʱ���뵽�ȴ�����ģʽ��˯��״̬���Ǵ���״̬���ֿ�����ʾ����״̬
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

                           //  �����������ס�
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
case CHAIR_STATE_IDLE:          Main_Idle();break;//����ģʽ���򿪰�������ȴ�����ģʽ��10���ް�������˯��=//����״̬,���ϵ�ϵͳ�������ģʽ��Main_Sleep��,Ҳ���Խ��뵽����ģʽ
case CHAIR_STATE_SETTLE:        Main_Settle();break;
case CHAIR_STATE_WAIT_COMMAND:  Main_WaitCommand();break; 
case CHAIR_STATE_RUN:           Main_Work();break; 
case CHAIR_STATE_PROBLEM:       Main_Problem();break;  
case CHAIR_STATE_ENGINEERING:   Main_Engineering();break;
case CHAIR_STATE_SLEEP:         Main_Sleep();break;//���а�������ʱ���뵽�ȴ�����ģʽ��˯��״̬���Ǵ���״̬���ֿ�����ʾ����״̬
case CHAIR_STATE_DEMO:          Main_Demo();break;
case CHAIR_STATE_UPDATE:        Main_Update();break;
// case CHAIR_STATE_CALIBRATION:   Main_Auto_Calibration();break;
}

*/