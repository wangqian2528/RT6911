#include "EFM32_def.h"
#include "EFM32_types.h"
#include "efm32.h"
#include "em_chip.h"
#include "em_gpio.h"
#include "em_timer.h"
#include "MassageStatus.h"
//#include "ZeroMotor.h"
#include "AxisMotor.h"
#include "Flex_Uart.h"
#include "input.h"
#include "problem.h"        

#define _3D_MAX_RUN_TIME    10
#define FLEX_MAX_RUN_TIME   20

static bool bFlag10ms;
static int _3D_ForwardCount;
static int _3D_BackCount;
static int flexOutCount;
static int flexInCount;
     
bool b3DOverTime,bFlexOverTime,bWalkSwitchFail;
void Problem_Initial_IO(void)
{
}
void Problem_10ms_Int(void)
{
  bFlag10ms = true;
}
  
void Problem_Proce(void)
{
  if(!bFlag10ms) return;
  bFlag10ms = false;
//_3D_Motor_error_Recoder
  if(AxisMotor_IsRun())//检查3D马达是否允许超时
  {
     if(AxisMotor_GetDirection() ==  AXIS_MOTOR_GO_FORWARD)
     {
       _3D_ForwardCount++;
       _3D_BackCount = 0;
     }
     else
     {
       _3D_BackCount++;
       _3D_ForwardCount = 0;
     } 
     if(_3D_ForwardCount > _3D_MAX_RUN_TIME || _3D_BackCount > _3D_MAX_RUN_TIME)//100ms 
       {
         _3D_ForwardCount = 0;
         _3D_BackCount = 0;
         b3DOverTime = true; //3D电机运行超时
       }
  }
  else
  {
     _3D_ForwardCount = 0;
     _3D_BackCount = 0;
     // b3DOverTime = false; 
  }
//Flex_Motor_error_Recoder
  if(Flex_GetDirection())
  {
   if(Flex_GetDirection() ==  FLEX_TO_OUT)
   {
     flexOutCount++;
     flexInCount = 0;
   }
   else
   {
     flexInCount++;
     flexOutCount = 0;
   } 
   if(flexOutCount > FLEX_MAX_RUN_TIME || flexInCount > FLEX_MAX_RUN_TIME)
     {
       flexOutCount = 0;
       flexInCount = 0;
       bFlexOverTime = true; //伸缩电机运行超时
     }
  }
  else
  {
     flexOutCount = 0;
     flexInCount = 0;
  }
  
  if(Input_GetWalkDownSwitch() == REACH_WALK_LIMIT && Input_GetWalkUpSwitch() == REACH_WALK_LIMIT)
  {
    bWalkSwitchFail = true;//行走电机开关异常
  }
  else
  {
    bWalkSwitchFail = false;
  } 
}
bool Problem_GetWalkSwitchFault(void)
{
  return(bWalkSwitchFail);
}

bool Problem_Get3DFault(void)
{
  return(b3DOverTime);
}
bool Problem_GetFlexFault(void)
{
  return(bFlexOverTime);
}
