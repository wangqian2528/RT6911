
/*
* Function: Input signal process
*/
/************************************************************************************/

#include "efm32.h"
#include "em_chip.h"
#include "em_gpio.h"
#include "em_cmu.h"
#include "em_timer.h"
#include "efm32_types.h"
#include "efm32_def.h"
#include "LegMotor.h"
#include "Valve.h"
#include "WalkMotor.h"
#include "AxisMotor.h"
#include "MassageStatus.h"
#include "memory.h"
#include "LEUart_DMA.h"
#include "backaction.h"
#include "input.h"


#include "KneadMotor.h"
#include "BackPad.h"

BITS InputData1;
#define bLegDownSwitch 	        InputData1.bD0   //=1在下行程开关处
#define bLegUpSwitch		InputData1.bD1   //=1在上行程开关处

#define bZeroMotorUpSwitch	InputData1.bD2  //bD2=1椅子升起来，
#define bZeroMotorDownSwitch	InputData1.bD3   // return(st_ZeroMotorDown.flag);//bZeroMotorDownSwitch  bD3=1椅子趟下处于零重力位置
//处于零重力位置时，零重力行程开关处于上行程开关位置=bZeroMotorDownSwitch=1
/*    椅子处于零重力位置时，靠背电机向上很快就会到达靠背上行程位置*/

#define bWalkDownSwitch 	InputData1.bD4   
#define bWalkUpSwitch           InputData1.bD5   
#define bBackUpSwitch		InputData1.bD6
#define bBackDownSwitch		InputData1.bD7
//#define bBackUpSwitch		InputData1.bD7  //=1表示靠背电机到上行程
//#define bBackDownSwitch		InputData1.bD6//=1表示靠背电机到下行程




//#define bSlidePadUpSwitch	InputData1.bD2  
//#define bSlidePadDownSwitch	InputData1.bD3  


#define Data1Offset         0
#define Data2Offset         1
#define Data3Offset         2

typedef  struct
{
        unsigned char timer_H;
        unsigned char timer_L;
        unsigned char flag;
}INPUT_ST;
//bool bVout;
bool bWalkChange,bInputReady;
bool _3DfrontSwitch, _3DBackSwitch;
  
INPUT_ST st_FlexGround,st_StretchOut,st_StretchIn,st_Foot,st_Angle,st_Vout,st_AxisSW;
INPUT_ST st_SlideUp,st_SlideDown,st_BackUp,st_BackDown,st_LegUp,st_LegDown,st_WalkUp,st_WalkDown;

INPUT_ST st_ZeroMotorUp,st_ZeroMotorDown,st_Wave;



bool b5msFlag;
BYTE nPulseHigh;
UINT16 WalkCount = 0xffff;  // 0xffff indicate no initial, at top =0 at bottom = maximal, up:dec down:add
BYTE by_KneadPosition = KNEAD_WIDTH_UNKNOWN;
volatile unsigned short nCurWalkLocate;
//unsigned short nCounterCurWalkLocate;
unsigned short nCurAxisLocate;
unsigned short nCounterCurAxisLocate;
unsigned int tickAxisCount;
bool bKneadMin,bKneadMid,bKneadMax;

bool bWalkPulseCount,bPreWalkPulseCount;

volatile unsigned int nCurZeroLocate;  //零重力电机位置坐标


volatile unsigned int nCurBackLocate;


//unsigned int axisTickTimedec[100];
//unsigned int axisTickTimeadd[100];

bool bShould,bShouldDelay;

unsigned int wghttt;

/**************************************************************************//**
 * @brief GPIO_ODD_IRQHandler
 * Interrupt Service Routine Odd GPIO Interrupt Line
 *****************************************************************************/
void GPIO_ODD_IRQHandler(void)
{
  if(GPIO_IntGet()&(1<<INPUT_BACK_PULSE_BIT))
  {
    GPIO_IntClear(1<<INPUT_BACK_PULSE_BIT);
    if(BackMotor_GetDirection() !=BACK_MOTOR_GO_UP )//  BACK_MOTOR_GO_DOWN 
    {
      wghttt=0;
      if(nCurBackLocate < BACK_MOTOR_MAX_LOCATION)
      {
        nCurBackLocate++ ;
      }       
      if(Input_GetBackDownSwitch() == REACH_BACK_LIMIT)
      {
        nCurBackLocate = BACK_MOTOR_MAX_LOCATION;
      }
    }
    else
    {   wghttt++;
      if(nCurBackLocate > 0 )
      {
        nCurBackLocate-- ;
      }
      if(Input_GetBackUpSwitch() == REACH_BACK_LIMIT)
      {
        nCurBackLocate =0;
      }
    }
    
  }
}
//unsigned short nCurWalkLocateTY;
void GPIO_EVEN_IRQHandler(void)//读取3D机芯的行走电机脉冲信号
{ 
 /* if(GPIO_IntGet()&(1<<INPUT_WALK_PULSE_BIT))
  {
    bWalkChange = TRUE;
    GPIO_IntClear(1<<INPUT_WALK_PULSE_BIT);
    if(WalkMotor_GetDirection() != WALK_MOTOR_GO_UP)
    { 
      //nCurWalkLocateTY=0;
      if(nCurWalkLocate > 0 )
      {
          nCurWalkLocate-- ;
      }
      if(Input_GetWalkDownSwitch() == REACH_WALK_LIMIT)
      {
        Input_SetWalkMotorPosition(0);
      }
    }
    else
    {
      //nCurWalkLocateTY++;
      
      if(nCurWalkLocate < 1000)
      {
          nCurWalkLocate++ ;
      }
      if(Input_GetWalkUpSwitch() == REACH_WALK_LIMIT)
      {
        Input_SetWalkMotorPosition(TOP_POSITION);//1
      }
    }
  }*/
}
//==========================================================
/*
void Input_SetZeroMotorPosition(unsigned int Position)//add by taoqingosng
{
  nCurZeroLocate = Position;
}
unsigned int Input_GetZeroPosition(void)
{
  return nCurZeroLocate;
}*/
//===============================================================



void Input_SetWalkMotorPosition(unsigned short locate)
{
  nCurWalkLocate = (unsigned short)locate<<2;
}
unsigned short Input_GetWalkMotorPosition(void)
{
  return(nCurWalkLocate>>2);
}

//相反方向
void Input_SetCounterWalkMotorPosition(unsigned short locate) //8600
{
  return;
  //nCounterCurWalkLocate = (unsigned short)locate;
  //nCounterCurWalkLocate *= 2;
}
/*
unsigned short Input_GetCounterWalkMotorPosition(void) //8600
{
  return(nCounterCurWalkLocate/2);
}
*/
void Input_SetAxisMotorPosition(unsigned short locate)//3D机芯镜向坐标
{
  nCurAxisLocate = (unsigned short)locate;
}
unsigned short Input_GetAxisMotorPosition(void)//3D机芯镜向坐标
{
  return(nCurAxisLocate);
}

//相反方向
void Input_SetCounterAxisMotorPosition(unsigned short locate) //8600
{
  nCounterCurAxisLocate = (unsigned short)locate;
  nCounterCurAxisLocate *= 2;
}
unsigned short Input_GetCounterAxisMotorPosition(void) //8600
{
  return(nCounterCurAxisLocate/2);
}

void Input_Initial_IO(void)
{
 // GPIO_PinModeSet(INPUT_KNEAD_MAX_PORT, INPUT_KNEAD_MAX_BIT, INPUT_KNEAD_MAX_MODE, 1);
 // GPIO_PinModeSet(INPUT_KNEAD_MID_PORT, INPUT_KNEAD_MID_BIT, INPUT_KNEAD_MID_MODE, 1);
 // GPIO_PinModeSet(INPUT_KNEAD_MIN_PORT, INPUT_KNEAD_MIN_BIT, INPUT_KNEAD_MIN_MODE, 1);
  

    
  //GPIO_IntConfig(INPUT_KNEAD_MAX_PORT, INPUT_KNEAD_MAX_BIT, false, true, true);
  //GPIO_IntConfig(INPUT_KNEAD_MID_PORT, INPUT_KNEAD_MID_BIT, false, true, true);
  //GPIO_IntConfig(INPUT_KNEAD_MIN_PORT, INPUT_KNEAD_MIN_BIT, false, true, true);
  
  
  GPIO_PinModeSet(INPUT_WAVE_CHECK_PORT, INPUT_WAVE_CHECK_BIT, INPUT_WAVE_CHECK_MODE, 1);
  
  
  GPIO_PinModeSet(INPUT_WALK_PULSE_PORT, INPUT_WALK_PULSE_BIT, INPUT_WALK_PULSE_MODE, 1);
  //GPIO_IntConfig(INPUT_WALK_PULSE_PORT, INPUT_WALK_PULSE_BIT, false, true, true);
  GPIO_PinModeSet(INPUT_BACK_PULSE_PORT, INPUT_BACK_PULSE_BIT, INPUT_BACK_PULSE_MODE, 1);
  GPIO_IntConfig(INPUT_BACK_PULSE_PORT, INPUT_BACK_PULSE_BIT, false, true, true);

  
//  GPIO_PinModeSet(INPUT_BACK_PULSE_PORT, INPUT_BACK_PULSE_BIT, INPUT_BACK_PULSE_MODE, 1);
  //GPIO_IntConfig(INPUT_BACK_PULSE_PORT, INPUT_BACK_PULSE_BIT, false, true, true);
  
//  GPIO_PinModeSet(INPUT_AXIS_PULSE_PORT, INPUT_AXIS_PULSE_BIT, INPUT_AXIS_PULSE_MODE, 1);
 // GPIO_IntConfig(INPUT_AXIS_PULSE_PORT, INPUT_AXIS_PULSE_BIT, false, true, true);
 // GPIO_PinModeSet(INPUT_AXIS_SW_PORT, INPUT_AXIS_SW_BIT, INPUT_AXIS_SW_MODE, 1);
//  GPIO_PinModeSet(INPUT_VOUT_PORT, INPUT_VOUT_BIT, INPUT_VOUT_MODE, 1);
  
//  GPIO_PinModeSet(INPUT_BACK_UP_PORT, INPUT_BACK_UP_BIT, INPUT_BACK_UP_MODE, 1); 
//  GPIO_PinModeSet(INPUT_BACK_DOWN_PORT, INPUT_BACK_DOWN_BIT, INPUT_BACK_DOWN_MODE, 1); 
  
//  GPIO_PinModeSet(INPUT_LEG_UP_PORT, INPUT_LEG_UP_BIT, INPUT_LEG_UP_MODE, 1); 
//  GPIO_PinModeSet(INPUT_LEG_DOWN_PORT, INPUT_LEG_DOWN_BIT, INPUT_LEG_DOWN_MODE, 1); 
  
//  GPIO_PinModeSet(INPUT_WALK_UP_PORT, INPUT_WALK_UP_BIT, INPUT_WALK_UP_MODE, 1); 
//  GPIO_PinModeSet(INPUT_WALK_DOWN_PORT, INPUT_WALK_DOWN_BIT, INPUT_WALK_DOWN_MODE, 1); 
  
//  GPIO_PinModeSet(INPUT_ZERO_UP_PORT, INPUT_ZERO_UP_BIT, INPUT_ZERO_UP_MODE, 1); 
//  GPIO_PinModeSet(INPUT_ZERO_DOWN_PORT, INPUT_ZERO_DOWN_BIT, INPUT_ZERO_DOWN_MODE, 1); 
  //  GPIO_PinModeSet(INPUT_WAVE_PORT, INPUT_WAVE_BIT, INPUT_WAVE_MODE, 1); 
  
    NVIC_EnableIRQ(GPIO_ODD_IRQn);
    //NVIC_EnableIRQ(GPIO_EVEN_IRQn);
}
void Input_5ms_Int(void)
{
    tickAxisCount++;
    if(AxisMotor_IsRun())
    {
      tickAxisCount++;
    }
    b5msFlag = 1;
}

void Input_High(INPUT_ST* p)
{
  p->timer_L = 0;
  p->timer_H ++; 
  if((p->timer_H) >= 2)
  {
    p->timer_H = 2;
    p->flag = 1;
  }
}
void Input_Low(INPUT_ST* p)
{
  p->timer_H = 0;
  p->timer_L ++; 
  if((p->timer_L) >= 2)
  {
    p->timer_L = 2;
    p->flag = 0;
  }
}
/*
BITS InputData1;
#define bLegDownSwitch 	        InputData1.bD0   
#define bLegUpSwitch		InputData1.bD1   
#define bSlidePadUpSwitch	InputData1.bD2  
#define bSlidePadDownSwitch	InputData1.bD3   
#define bWalkDownSwitch 	InputData1.bD4   
#define bWalkUpSwitch           InputData1.bD5   
#define bBackUpSwitch		InputData1.bD6
#define bBackDownSwitch		InputData1.bD7*/


unsigned short nUPWalkLocate;
unsigned short nDOWNWalkLocate;
unsigned short nUPWalkLocatePRE;
unsigned short nDOWNWalkLocatePRE;
//1580
//1578


void Input_Proce(void)
{
unsigned char signal;
  static int counter;  
  static unsigned short nOldCurAxisLocate = 0;
  if(!b5msFlag) return;		
  b5msFlag = 0;
  if(++counter >= 4) bInputReady = true;
  InputData1.nByte = ~(*(pInputData + Data1Offset));//8302T将74hc165的数据取反 ，因为硬件上增加了反向器,modify by taoqingsong
  //InputData2.nByte = *(pInputData + Data2Offset);
  //InputData3.nByte = *(pInputData + Data3Offset);
  //nCounterCurWalkLocate = LEUART0_GetWalkPluse();  
  
  nCurAxisLocate = LEUART0_Get3DPluse();//3D机芯镜向位置坐标
  if(nOldCurAxisLocate |= nCurAxisLocate);
  {
    AxisMotor_UpdataPosition();
    nOldCurAxisLocate = nCurAxisLocate;
  }

  signal = LEUART0_GetMassageSignal();//读取3D行程开关信号
  //		   7        6	     5              4	           3            2   1   0//敲击平位置=3,       4=肩膀位置
  //xxxx xxxx  --> 0 3DbackSwitch 3DfrontSwitch shoulderPosition MotorPosition Max Med Min
  
  if(signal&BIT2)
  {
    by_KneadPosition = KNEAD_WIDTH_MIN;    
    bKneadMin = 0;
    bKneadMid = 1; 
    bKneadMax = 1;
  }
  if(signal&BIT1)
  {
    by_KneadPosition = KNEAD_WIDTH_MED;    
    bKneadMin = 1;
    bKneadMid = 0; 
    bKneadMax = 1;
  }
  if(signal&BIT0)
  {
    by_KneadPosition = KNEAD_WIDTH_MAX;  
    bKneadMin = 1;
    bKneadMid = 1; 
    bKneadMax = 0;
  }
  
  KneadMotor_CalculateSpeed(by_KneadPosition);
  
  if(signal&BIT4)
  {
   // bVout = 1;
        bShould = 1;bShouldDelay =1;//nShouldDelay = 0;
  }
  else
  {
   // bVout = 0;
            bShould = 0;
            bShouldDelay =0;
    
  }
  
  if(signal&BIT6)
  {
    _3DfrontSwitch = 1;
  }
  else
  {
    _3DfrontSwitch = 0; 
  }
  if(signal&BIT5)
  {
    _3DBackSwitch = 1;
  }
  else
  {
    _3DBackSwitch = 0;
  }
  
  //bLegStretchGroundSwitch?Input_High(&st_FlexGround):Input_Low(&st_FlexGround);
  bLegUpSwitch ? Input_High(&st_LegUp):Input_Low(&st_LegUp);
  bLegDownSwitch ? Input_High(&st_LegDown):Input_Low(&st_LegDown);
  bBackUpSwitch ? Input_High(&st_BackUp):Input_Low(&st_BackUp);
  bBackDownSwitch ? Input_High(&st_BackDown):Input_Low(&st_BackDown);		
  
//  bSlidePadUpSwitch ? Input_High(&st_SlideUp):Input_Low(&st_SlideUp);
 // bSlidePadDownSwitch ? Input_High(&st_SlideDown):Input_Low(&st_SlideDown);
  bZeroMotorUpSwitch ? Input_High(&st_ZeroMotorUp):Input_Low(&st_ZeroMotorUp);
  bZeroMotorDownSwitch ? Input_High(&st_ZeroMotorDown):Input_Low(&st_ZeroMotorDown);
  
  
  
   //    return(st_ZeroMotorDown.flag);//bZeroMotorDownSwitch
  
  
  
  bWalkUpSwitch ? Input_High(&st_WalkUp):Input_Low(&st_WalkUp);
  bWalkDownSwitch ? Input_High(&st_WalkDown):Input_Low(&st_WalkDown);
  
    GPIO_PinInGet(INPUT_WAVE_CHECK_PORT, INPUT_WAVE_CHECK_BIT)?Input_High(&st_Wave):Input_Low(&st_Wave);
  
  //bLegStretchOutSwitch ? Input_High(&st_StretchOut):Input_Low(&st_StretchOut);
  //bLegStretchInSwitch ? Input_High(&st_StretchIn):Input_Low(&st_StretchIn);
  //GPIO_PinInGet(INPUT_LEG_FOOT_PORT, INPUT_LEG_FOOT_BIT)?Input_High(&st_Foot):Input_Low(&st_Foot);
  //bLegStretchAngleSwitch? Input_High(&st_Angle):Input_Low(&st_Angle);
  //GPIO_PinInGet(INPUT_KNEAD_MAX_PORT, INPUT_KNEAD_MAX_BIT) ? Input_High(&st_KneadMax):Input_Low(&st_KneadMax);	
  //GPIO_PinInGet(INPUT_KNEAD_MAX_PORT, INPUT_KNEAD_MID_BIT) ? Input_High(&st_KneadMid):Input_Low(&st_KneadMid);
  //GPIO_PinInGet(INPUT_KNEAD_MAX_PORT, INPUT_KNEAD_MIN_BIT) ? Input_High(&st_KneadMin):Input_Low(&st_KneadMin);			
  //GPIO_PinInGet(INPUT_AXIS_SW_PORT, INPUT_AXIS_SW_BIT) ? Input_High(&st_AxisSW):Input_Low(&st_AxisSW);
  bWalkPulseCount = GPIO_PinInGet(INPUT_WALK_PULSE_PORT, INPUT_WALK_PULSE_BIT);
  if(bPreWalkPulseCount != bWalkPulseCount)
  { 
    if(WalkMotor_GetDirection() != WALK_MOTOR_GO_UP)
    {
      nUPWalkLocate = 0;
      nDOWNWalkLocate++;     
      nDOWNWalkLocatePRE=nDOWNWalkLocate;
      
      if(nCurWalkLocate > 0 )
      {
        nCurWalkLocate-- ;
      }
      if(Input_GetWalkDownSwitch() == REACH_WALK_LIMIT)
      {
        Input_SetWalkMotorPosition(0);
      }
    }
    else
    {
      nUPWalkLocate++ ;
      nDOWNWalkLocate= 0;
      nUPWalkLocatePRE = nUPWalkLocate ;
      if(nCurWalkLocate < 2000)
      {
        nCurWalkLocate++ ;
      }
      if(Input_GetWalkUpSwitch() == REACH_WALK_LIMIT)
      {
        Input_SetWalkMotorPosition(TOP_POSITION);//1
      }
    }
    
    
    
    
  }
  bPreWalkPulseCount = bWalkPulseCount;
    
    
    
    
    
    
    
}

//return 0 or 1
unsigned int Input_GetKneadMax(void)
{
  return(bKneadMax);
}
//return 0 or 1
unsigned int Input_GetKneadMid(void)
{
    return(bKneadMid);
}
//return 0 or 1
unsigned int Input_GetKneadMin(void)
{
    return(bKneadMin);
}

unsigned int Input_GetBackUpSwitch(void)
{
  return(st_BackUp.flag);
}                     
unsigned int Input_GetBackDownSwitch(void)
{
  return(st_BackDown.flag);
}                     
unsigned int Input_GetLegUpSwitch(void)
{
  return(st_LegUp.flag);
}                     
unsigned int Input_GetLegDownSwitch(void)
{
  return(st_LegDown.flag);
}



unsigned int Input_GetZeroDownSwitch(void)
{
    return(st_ZeroMotorDown.flag);
}

unsigned int Input_GetZeroUpSwitch(void)
{
  
    return(st_ZeroMotorUp.flag);
  
}




unsigned int Input_GetSlideForwardSwitch(void)
{
 /* bool enable = ReadEEByte(USER_DATA_BASE + SLIDE_MOTOR_ENABLE_ADDRESS);
  if(enable)
     return(st_SlideDown.flag);
  else
    return(1);*/
  
  //     return(st_ZeroDown.flag);
 // return 0;
  
      return(st_ZeroMotorDown.flag);
  
}
unsigned int Input_GetSlideBackwardSwitch(void)
{
 /* bool enable = ReadEEByte(USER_DATA_BASE + SLIDE_MOTOR_ENABLE_ADDRESS);
  if(enable)
   return(st_SlideUp.flag);
  else return(1);*/
  
   //  return(st_ZeroUp.flag);
 // return 0;
      return(st_ZeroMotorUp.flag);
  
}

unsigned int Input_GetKneadPosition(void)
{
  return (unsigned int)by_KneadPosition;
}
/*
#define  BODY_TOUCHED     1
#define  BODY_NO_TOUCHED  0

#define  BODY_TOUCHED     0
#define  BODY_NO_TOUCHED  1
*/
//肩部位置检测信号    
unsigned int Input_GetVout(void)
{
  //if(bVout) return BODY_TOUCHED;
  //return BODY_NO_TOUCHED;
  
    if(bShouldDelay==0) return BODY_TOUCHED; //if(bShould==0) return BODY_TOUCHED;
    return BODY_NO_TOUCHED;
  
}   

/*
unsigned int Input_GetAxisSW(void)
{
  return(st_AxisSW.flag);
}
*/
//unsigned int Input_GetFlexGroundSwitch(void)
//{
 // return(st_FlexGround.flag);
//}

unsigned int Input_GetMp3Status(void)
{
  return 1;
}
unsigned int Input_PowerCheck(void)
{
  return 1;
}

/*
bool Input_GetWalkChange(void)
{
  return(bWalkChange);
}

void Input_ClearWalkChange(void)
{
  bWalkChange = 0;
}
*/
unsigned int Input_GetWalkPosition(void)
{
  if(Input_GetWalkDownSwitch() == REACH_WALK_LIMIT)
    return WALK_MOTOR_AT_BOTTOM;
  if(Input_GetWalkUpSwitch() == REACH_WALK_LIMIT)
    return WALK_MOTOR_AT_TOP;
  else
    return WALK_MOTOR_AT_MID;
}
/*
//最前面位置
unsigned int Input_GetFlexOutSwitch(void)
{
  return(st_StretchOut.flag);
} 
//最后面位置
unsigned int Input_GetFlexInSwitch(void)
{
  return(st_StretchIn.flag);
}                     

unsigned int Input_GetFlexFootSwitch(void)
{
  return(st_Foot.flag);
} 

unsigned int Input_GetFlexAngleSwitch(void)
{
  return(st_Angle.flag);
} 
*/

unsigned int Input_GetWaveMotorPosition(void)
{
 return(st_Wave.flag);
}





unsigned int Input_GetWalkUpSwitch(void)
{
  return(st_WalkUp.flag);
}
unsigned int Input_GetWalkDownSwitch(void)
{
  return(st_WalkDown.flag);
}
unsigned int Input_GetReady(void)
{
    if(bInputReady) return 1;
    return 0;
}
bool Input_Get3DFrontSwitch(void)
{
  return(_3DfrontSwitch);
} 

bool Input_Get3DBackSwitch(void)
{
  return(_3DBackSwitch);
} 

unsigned int BackMotor_Get_Location(void)
{
   return nCurBackLocate;
}
void BackMotor_Set_Location(unsigned short locate)
{
   nCurBackLocate = locate;
}

//获取靠背电动缸的脉冲计数位置
/*
unsigned int Input_GetBackPosition(void)
{
  return nCurBackLocate;
}
void Input_SetBackMotorPosition(unsigned int Position)
{
  nCurBackLocate = Position;
}




                     if(nFlexStatus&0x04) OutBuffer[1] |= 0x10;//碰到地面
                     else OutBuffer[1] &= ~0x10;                  
                     if(nFlexStatus&0x40) OutBuffer[1] |= 0x08;//碰到地面
                     else OutBuffer[1] &= ~0x08;  
*/