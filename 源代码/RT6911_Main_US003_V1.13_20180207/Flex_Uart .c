#include "efm32.h"
#include "em_chip.h"
#include "em_gpio.h"
#include "em_cmu.h"
#include "em_timer.h"
#include "LegMotor.h"
#include "Flex_Uart.h"

#include "EFM32_def.h"
#include "EFM32_types.h"


#define FLEX_START_ANGLE  220
static unsigned char nFlexMode;
static unsigned char nFlexDirection;
static unsigned char nFlexCurrent;
static bool FlexMotorEnable = false;
//角度禁止标志 当角度大于45度时，不允许伸出，以防顶翻机器
static bool angleFlexDisable = true;

 //   angleFlexDisable = true;//angleFlexDisable=1,不依赖角度

 //   angleFlexDisable = false;//=angleFlexDisable=0,依赖角度



static unsigned int w_FlexAdjStep;

//unsigned int w_FlexAdjStep;//test

 unsigned char nFlexStatus;
unsigned char nFlexStatus5;//test


static volatile unsigned int w_Timer;
extern unsigned short nLegAngle;

//signed char nFlexStatus_test,nFlexStatus_test2;



void Flex_SetDirection(unsigned int flexDirection)
{
   nFlexDirection = flexDirection;
}
unsigned char Flex_GetDirection(void)
{
   return(nFlexDirection);
}


void Flex_ControlStop(void)
{
  Flex_SetDirection(FLEX_MOTOR_STOP);
}


void Flex_SetDisableAngle(bool disable)
{
  if(disable)
  {
    angleFlexDisable = true;//angleFlexDisable=1,不依赖角度
  }
  else
  {
    angleFlexDisable = false;//=angleFlexDisable=0,依赖角度
  }
}

unsigned char Flex_GetDisableAngle()
{
  return (unsigned char)(angleFlexDisable);//
}

unsigned char Flex_GetCurrent(void)
{
   return(nFlexCurrent);
}

void Flex_SetCurrent(unsigned char current)
{
   nFlexCurrent = current;
}

void Flex_SetMode(unsigned int mode)
{
    nFlexMode = mode;
}
unsigned char Flex_GetMode(void)
{
    return(nFlexMode);
}





//自动找脚程序 
BYTE w_Timer_OutTime;
void FlexMotorFollowingFood(void)
{
  nFlexStatus5=nFlexStatus;

  if(!FlexMotorEnable)
  {
    return;
  }
  if(LegPower_Get())
  {
    return;  //小腿上下电动缸还在运行中
  }
  //printf("s0\n");
  switch(w_FlexAdjStep)
  {
  case 0:
    {

   //   w_FlexAdjStep++;//w_FlexAdjStep=1; 碰到脚向外走，
      w_FlexAdjStep=1; //碰到脚向外走，
   //  if(nFlexStatus&0x04 == 0)  
   //   if((nFlexStatus&0x04) == 0)  //modify by  taoq没有碰到脚向里走
   //   {  
        // w_FlexAdjStep++;  //没有碰到脚  w_FlexAdjStep=2;
     //   w_FlexAdjStep=2;
    //  }   
      
      w_Timer = 0;
      if((nFlexStatus&0x03) ==  FLEX_AT_IN) 
      {
        w_Timer_OutTime = 60;
      }
      else
      {
        w_Timer_OutTime = 30;
      }
      
    }
    break;
  case 1://先向外走     if((nFlexStatus&0x20) ==  LEGANGLE_SWITCH_ON)  

    if((nFlexStatus&0x03) ==  FLEX_AT_OUT) //处理个子太高，脚一直伸到行程开关位置处，此时停止找脚程序
    { //向外走到底了
      w_FlexAdjStep=2;//WGH  20160627
       //FlexMotorEnable = false;
       Flex_SetDirection(FLEX_MOTOR_STOP);

       
       break;
    }
    
    
    
      if(nFlexStatus&0x40)//向外走碰到地面
      {
         //  FlexMotorEnable = false;
          Flex_SetDirection(FLEX_MOTOR_STOP);
                 w_FlexAdjStep++;
      w_Timer = 0;
          break;
      }
    
    
 //   if((nFlexStatus&0x20) == LEGANGLE_SWITCH_ON)
    if(nFlexStatus&0x20)//modify by taoqingsong 2015-10-10
    {
      //  FlexMotorEnable = false;
        Flex_SetDirection(FLEX_MOTOR_STOP);
       w_FlexAdjStep++;
      w_Timer = 0;
        break;                
    }
    /////////////////
if((nFlexStatus &0x04))//没有碰到脚，向里
    {  //向外走碰不到脚了 再向回来点
//       w_FlexAdjStep++;
      
      w_Timer_OutTime = 20;//20;//30   
       w_Timer = 0;
       //Flex_ControlStop();
     //  Flex_ControlIn(FLEX_MOTOR_CURRENT_1D5A);//FLEX_CURRENT_3A
    }
    
    
    ////////////////
    if(w_Timer < w_Timer_OutTime)
    {
       Flex_ControlOut(FLEX_MOTOR_CURRENT_2A);
      // printf("s2\n");
      // w_FlexAdjStep++;
      
     // w_FlexAdjStep=2;
       
    }
    else
    {
       w_FlexAdjStep++;
      w_Timer = 0;
    }
    
  /*  
    if(!(nFlexStatus &0x04))//没有碰到脚，向里
    {  //向外走碰不到脚了 再向回来点
//       w_FlexAdjStep++;
      
      w_FlexAdjStep=2;
       w_Timer = 0;
       //Flex_ControlStop();
       Flex_ControlIn(FLEX_MOTOR_CURRENT_1D5A);//FLEX_CURRENT_3A
    }
    else  //碰到脚，向外
    {
      //向外走
      Flex_ControlOut(FLEX_MOTOR_CURRENT_2A);
    }
*/
    break;
  case 2://再向里走
    
   if(nFlexStatus&0x04)//碰到脚
    {  //向里碰到脚了 
       w_FlexAdjStep++;
      
      //w_FlexAdjStep=3;//20160919
       w_Timer = 0;
     //  printf("s1\n");
       break;
    }
    
    if((nFlexStatus&0x03) ==  FLEX_AT_IN) 
    { //向里走到底了
       FlexMotorEnable = false;
       Flex_SetDirection(FLEX_MOTOR_STOP);
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
      
       break;
    }
    //向里走
    Flex_ControlIn(FLEX_MOTOR_CURRENT_1D5A); 
    //Flex_ControlIn(FLEX_MOTOR_CURRENT_2A);   
    break;
   case 3:  //碰到脚后 再向里走1秒
    //if(w_Timer > 30)//走1秒
     if(w_Timer > 5)  //走2秒 
    {
       FlexMotorEnable = false;
       Flex_SetDirection(FLEX_MOTOR_STOP);
      // printf("s2\n");
       break;
    }
    if((nFlexStatus&0x03) ==  FLEX_AT_IN) 
    { //向里走到底了
       FlexMotorEnable = false;
       Flex_SetDirection(FLEX_MOTOR_STOP);
       break;
    }
    Flex_ControlIn(FLEX_MOTOR_CURRENT_2A);   
     break;
   default:
    FlexMotorEnable = false;
    Flex_SetDirection(FLEX_MOTOR_STOP);
    break;
  }
}














//1 执行自动跟脚程序
int FlexMotorGetEnable(void)
{
  return(FlexMotorEnable);
}
void FlexMotorSetEnable(void)
{
  FlexMotorEnable = true;  //打开自动找脚标志位
  w_FlexAdjStep = 0; //自动找脚程序步骤清0
}

void FlexMotorSetAdjStep(unsigned char by_step)
{
  
    FlexMotorEnable = true;  //打开自动找脚标志位
    w_FlexAdjStep = by_step; //自动找脚程序步骤清0
}

void FlexMotorSetDisable(void)
{
  FlexMotorEnable = false;
  w_FlexAdjStep = 0; 
}

void Flex_SetStatus(unsigned char status)
{
   nFlexStatus = status;
}

unsigned char Flex_ControlIn(unsigned int current)
{
  //if((nFlexStatus&0x03) ==  FLEX_AT_IN)  //检测小腿是否到达行  小腿到达短限位
  if((nFlexStatus&0x03) ==  FLEX_AT_IN || (nFlexStatus&BIT5))  
  {
    Flex_SetDirection(FLEX_MOTOR_STOP);
    return 1;
  }
  Flex_SetCurrent(current);
  Flex_SetDirection(FLEX_TO_IN);
  return 0;
}

	/*   if(Input_GetFlexAngleSwitch2() == LEGANGLE_SWITCH_ON)
			{	//小于15度
				bPowerFlag3 = FALSE;
				FlexMotor_Brake();
				nFlexRetVal = FLEX_STOP_AT_ANGLE ;
				break;
			}
	
	    if(Input_GetFlexGroundSwitch() == LEGGROUND_SWITCH_ON)//add by taoqingsong
      { //碰到地面了
      bPowerFlag3 = FALSE;
      FlexMotor_Brake();
      nFlexRetVal = FLEX_STOP_AT_GROUND;
      break;
      }			

                    if(nFlexStatus&0x20) OutBuffer[1] |= 0x08;//碰到角度开关
                    else OutBuffer[1] &= ~0x08;
                    
                     if(nFlexStatus&0x40) OutBuffer[1] |= 0x10;//碰到地面
                     else  OutBuffer[1] &= ~0x10;

*/


unsigned char Flex_ControlOut(unsigned int current)
{

  if((nFlexStatus&0x03) == (FLEX_AT_OUT))  
  {
    Flex_SetDirection(FLEX_MOTOR_STOP);
    return 1;
  }
  // if((nFlexStatus&0x20) == (LEGANGLE_SWITCH_ON))
   if(nFlexStatus&0x20)//碰到角度开关
   {
        Flex_SetDirection(FLEX_MOTOR_STOP);
        return 1; 
     
   }
  
 //  if((nFlexStatus&0x40) ==(LEGGROUND_SWITCH_ON_2))
//   if((nFlexStatus_test&0x40) ==LEGGROUND_SWITCH_ON_2)
   if(nFlexStatus&0x40)//碰到地面
   {
        Flex_SetDirection(FLEX_MOTOR_STOP);
        return 1; 
     
   }  
  Flex_SetCurrent(current);
  Flex_SetDirection(FLEX_TO_OUT);
  return 0;
}

void Flex_100msInt(void)
{
  w_Timer++;
}
unsigned char asw;
void testwgh()
{
  
      asw = 1;
    if(asw)
    {
      asw=1;
    } 
    if(asw == 1)
    {
      asw = 1;
    }
    if(asw)
    {
      asw = 2;
    }
    if( asw )
    {
      asw = 0x20;
    }
    if(asw)
    {
      asw=1;
    } 
  
  
}