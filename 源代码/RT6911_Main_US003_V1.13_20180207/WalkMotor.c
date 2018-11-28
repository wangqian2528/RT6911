#include "EFM32_def.h"
#include "EFM32_types.h"
#include "efm32.h"
#include "em_chip.h"
#include "em_gpio.h"
#include "em_timer.h"
#include "input.h"
#include "WalkMotor.h"

#include "LEUart_DMA.h"
#include "ADC_Scan.h"//fww

#define TOP_POSITION 	     399//215// 225// 215//8302T              397//8600               //8302=215  //肩膀位置的最高点
volatile unsigned char nWalkLoss;
static bool bFalg; 

void WalkMotor_Initial_IO(void)
{
    GPIO_PinModeSet(WALK_MOTOR_RESET_PORT, WALK_MOTOR_RESET_BIT, WALK_MOTOR_RESET_MODE, 0);
    GPIO_PinModeSet(WALK_MOTOR_ENBL_PORT, WALK_MOTOR_ENBL_BIT, WALK_MOTOR_ENBL_MODE, 0);
    GPIO_PinModeSet(WALK_MOTOR_PHASE_PORT, WALK_MOTOR_PHASE_BIT, WALK_MOTOR_PHASE_MODE, 0);
    GPIO_PinModeSet(WALK_MOTOR_DECAY_PORT, WALK_MOTOR_DECAY_BIT, WALK_MOTOR_DECAY_MODE, 0);
    GPIO_PinModeSet(WALK_MOTOR_FAULT_PORT, WALK_MOTOR_FAULT_BIT, WALK_MOTOR_FAULT_MODE, 1);
    
    TIMER_InitCC_TypeDef timerCCInit = WALK_MOTOR_Timer_CCInit;
    TIMER_InitCC(WALK_MOTOR_TIMER, WALK_MOTOR_TIMER_CHANNEL, &timerCCInit);
    WALK_MOTOR_TIMER->ROUTE |= (WALK_MOTOR_ROUTE_EN | WALK_MOTOR_ROUTE_LOCATION); 
    TIMER_TopSet(WALK_MOTOR_TIMER, WALK_MOTOR_DEFAULT_TOP);
    TIMER_CompareBufSet(WALK_MOTOR_TIMER, WALK_MOTOR_TIMER_CHANNEL, 0);
    
  //    TIMER_CompareBufSet(WALK_MOTOR_TIMER, WALK_MOTOR_TIMER_CHANNEL, 65);
      
    
    TIMER_Init_TypeDef timerInit = WALK_MOTOR_Timer_Init;
    TIMER_Init(WALK_MOTOR_TIMER, &timerInit);
}

void WalkMotor_10ms_Int(void)//10ms  tiemr
{
    bFalg = true; 
}




unsigned int WalkMotor_VoltageAdj(unsigned int setDuty)
{
  unsigned short adc24;      //此处的电压值已经扩大了100倍
  ADC_Get_Voltage(ADC_V24,&adc24);     
  if(adc24 <= WALK_SET_VOLTAGE/100) 
  {
    return setDuty;        //电压值偏低，返回预设值
  }
  unsigned int scale = WALK_SET_VOLTAGE / adc24; //计算与设定电压的比例值
  setDuty *= scale;
  unsigned int yushu = setDuty  % 100;
  setDuty /= 100;
  if(yushu > 50) setDuty++;
  return setDuty; 
  
  
  
 /* unsigned short adc24;
  ADC_Get_Voltage(ADC_V24,&adc24);     
  unsigned int duty = WALK_MOTOR_DEFAULT_TOP;
  
  if(adc24 < WALK_SET_VOLTAGE/100) 
  {
    return duty; 
  }
  unsigned int yushu = WALK_SET_VOLTAGE % adc24;
  duty = WALK_SET_VOLTAGE / adc24;
  if(yushu > adc24/2) duty++;
  return duty; 
  */
  
}


void WalkMotor_Set_Pwm_Data(unsigned long ulDuty)
{
 /*  unsigned int duty ;
    if(ulDuty == 0)
    {
        TIMER_CompareBufSet(WALK_MOTOR_TIMER, WALK_MOTOR_TIMER_CHANNEL, ulDuty);
        return;
    }
    duty = TIMER_CompareBufGet(WALK_MOTOR_TIMER, WALK_MOTOR_TIMER_CHANNEL);
    if(ulDuty == duty)
    {
        if(WalkMotor_Get_Fault() == WALK_MOTOR_NORMAL) return;
        WalkMotor_Reset();
        __no_operation();
        __no_operation();
        WalkMotor_Reset_Cancel();
        return; 
    }
    if(!bFalg) return;
    bFalg = false;
    if(duty < ulDuty)
    {
        if(duty < WALK_MOTOR_DEFAULT_TOP/2)
            duty = WALK_MOTOR_DEFAULT_TOP/2;
        else duty++;
    }
    
    TIMER_CompareBufSet(WALK_MOTOR_TIMER, WALK_MOTOR_TIMER_CHANNEL, duty);*/
    
    if(ulDuty == 0)
    {
        TIMER_CompareBufSet(WALK_MOTOR_TIMER, WALK_MOTOR_TIMER_CHANNEL, ulDuty);
        return;
    }
    if(WalkMotor_Get_Fault() == WALK_MOTOR_FAIL)
    {
      WalkMotor_Reset();
      __no_operation();
      __no_operation();
      WalkMotor_Reset_Cancel();
    }
    ulDuty = WalkMotor_VoltageAdj(ulDuty);
    TIMER_CompareBufSet(WALK_MOTOR_TIMER, WALK_MOTOR_TIMER_CHANNEL, ulDuty);   
    
    
}

static void WalkPower_On(void)
{
    //Power_On();
    // GPIO_PinOutSet(WALK_MOTOR_ENBL_PORT, WALK_MOTOR_ENBL_BIT);
    WalkMotor_Set_Pwm_Data(WALK_MOTOR_DEFAULT_TOP);
}
static void WalkPower_Off(void)
{
    //GPIO_PinOutClear(WALK_MOTOR_ENBL_PORT, WALK_MOTOR_ENBL_BIT);
    WalkMotor_Set_Pwm_Data(0);
}
unsigned int WalkRelay_Get(void)
{
    return(GPIO_PinOutGet(WALK_MOTOR_PHASE_PORT, WALK_MOTOR_PHASE_BIT));
}

unsigned int WalkMotor_GetDirection(void)
{
    if(GPIO_PinOutGet(WALK_MOTOR_PHASE_PORT, WALK_MOTOR_PHASE_BIT) == 0)
    {
      return WALK_MOTOR_GO_UP;
    }
    return WALK_MOTOR_GO_DOWN;
}

unsigned int WalkPower_Get(void)
{
    unsigned long  ulDuty;
    ulDuty = TIMER_CompareBufGet(WALK_MOTOR_TIMER, WALK_MOTOR_TIMER_CHANNEL);
    if(ulDuty > 0) return WALK_MOTOR_POWER_ON;
    else return WALK_MOTOR_POWER_OFF;
}

static void WalkMotor_Up(void)
{
    GPIO_PinOutSet(WALK_MOTOR_RESET_PORT, WALK_MOTOR_RESET_BIT);
    GPIO_PinOutClear(WALK_MOTOR_PHASE_PORT, WALK_MOTOR_PHASE_BIT);
    GPIO_PinOutClear(WALK_MOTOR_DECAY_PORT, WALK_MOTOR_DECAY_BIT);
}
static void WalkMotor_Down(void)
{
    GPIO_PinOutSet(WALK_MOTOR_RESET_PORT, WALK_MOTOR_RESET_BIT);
    GPIO_PinOutSet(WALK_MOTOR_PHASE_PORT, WALK_MOTOR_PHASE_BIT);
    GPIO_PinOutClear(WALK_MOTOR_DECAY_PORT, WALK_MOTOR_DECAY_BIT);
}
static void WalkMotor_Break(void)
{
    GPIO_PinOutClear(WALK_MOTOR_DECAY_PORT, WALK_MOTOR_DECAY_BIT);
    WalkMotor_Set_Pwm_Data(0);
}
static void WalkMotor_Reset(void)
{
    GPIO_PinOutClear(WALK_MOTOR_RESET_PORT, WALK_MOTOR_RESET_BIT);
}
static void WalkMotor_Reset_Cancel(void)
{
    GPIO_PinOutSet(WALK_MOTOR_RESET_PORT, WALK_MOTOR_RESET_BIT);
}

int WalkMotor_Get_Fault(void)
{
    if(GPIO_PinInGet(WALK_MOTOR_FAULT_PORT, WALK_MOTOR_FAULT_BIT))
        return WALK_MOTOR_NORMAL;
    return WALK_MOTOR_FAIL;
}

//BackPad motor control function    ,最低点位置为0，最高点位置计数最大            停止位置
unsigned char WalkMotor_Control(unsigned char nFinalWalkMotorState,unsigned short stopPosition)
{
    unsigned char nRetVal ;
    bool bPowerFlag;
    nRetVal = FALSE ;
    unsigned short curPosition = Input_GetWalkMotorPosition();
    nWalkLoss = 0;
    
   if(!LEUART0_isOK())///在正常收到3D机芯的数据标志，后行走电机开始运行
   {
     WalkMotor_Break();
     WalkPower_Off();
     return(unsigned char)(TRUE);
   }
   
   
   
    switch(nFinalWalkMotorState)//#define TOP_POSITION 	397//8600 215  //肩膀位置的最高点
    {
    case STATE_RUN_WALK_POSITION:  
        if(stopPosition > (curPosition + 3))
        {
            if(Input_GetWalkUpSwitch() == REACH_WALK_LIMIT)
            {
                Input_SetWalkMotorPosition(TOP_POSITION);
                bPowerFlag = FALSE;
                nRetVal = TRUE ;
                WalkMotor_Break();
                break;
            }
            WalkMotor_Up();
            bPowerFlag = TRUE;
            break;
        }
        if((stopPosition + 3) < curPosition)
        {
            if(Input_GetWalkDownSwitch() == REACH_WALK_LIMIT)
            {
                Input_SetWalkMotorPosition(0);
                bPowerFlag = FALSE;
                nRetVal = TRUE ;
                WalkMotor_Break();
                break;
            }
            WalkMotor_Down();
            bPowerFlag = TRUE;
            break;
        }
        bPowerFlag = FALSE;
        nRetVal = TRUE ;
        WalkMotor_Break();
        break;
    case STATE_RUN_WALK_DOWN:  //back motor go down
        if(Input_GetWalkDownSwitch() == REACH_WALK_LIMIT)
        {
            Input_SetWalkMotorPosition(0);
            bPowerFlag = FALSE;
            nRetVal = TRUE ;
            WalkMotor_Break();
            break;
        }
        WalkMotor_Down();
        bPowerFlag = TRUE;
        break ;
    case STATE_RUN_WALK_UP:  //back motor go up
        if(Input_GetWalkUpSwitch() == REACH_WALK_LIMIT)
        {
            Input_SetWalkMotorPosition(TOP_POSITION);
            bPowerFlag = FALSE;
            nRetVal = TRUE ;
            WalkMotor_Break();
            break;
        }
        WalkMotor_Up();
        bPowerFlag = TRUE;
        break ;
    case STATE_WALK_IDLE:
        nRetVal = TRUE ;
        WalkMotor_Break();
        bPowerFlag = FALSE;
        break ;
    default://异常处理
        break ;
    }
    //电源部分的处理
    if(bPowerFlag == TRUE)
    {
        WalkPower_On();
    }
    else
    {
        WalkPower_Off();
    }
    return nRetVal ;
}


void WalkMotor_10msInt(void)//行走电机不动作，20ms后自动关闭行走电机
{
  nWalkLoss++;
  if(nWalkLoss > 2)
  {
    //Power_All_Off();
    TIMER_CompareBufSet(WALK_MOTOR_TIMER, WALK_MOTOR_TIMER_CHANNEL, 0);
  }
}