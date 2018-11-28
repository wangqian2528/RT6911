#include "EFM32_def.h"
#include "EFM32_types.h"
#include "efm32.h"
#include "em_chip.h"
#include "em_gpio.h"
#include "em_timer.h"
#include "input.h"
//#include "valve.h"

//#include "LegMotor.h"
//#include "MassageStatus.h"
#include "waver_Flex_Motor.h"
//#include "Hot_Roller.h"


#include "ADC_Scan.h"//fww


static bool bFalg; 
//void FlexMotor_Reset(void);
//void FlexMotor_Reset_Cancel(void);
//static bool FlexMotorEnable = false;
//static unsigned int w_FlexAdjStep;
void Waver_FlexMotor_Initial_IO(void)
{
  GPIO_PinModeSet( FLEX_MOTOR_RESET_PORT,  FLEX_MOTOR_RESET_BIT,  FLEX_MOTOR_RESET_MODE, 0);
  GPIO_PinModeSet( FLEX_MOTOR_ENBL_PORT,   FLEX_MOTOR_ENBL_BIT,  FLEX_MOTOR_ENBL_MODE, 0);
  GPIO_PinModeSet( FLEX_MOTOR_PHASE_PORT,  FLEX_MOTOR_PHASE_BIT,  FLEX_MOTOR_PHASE_MODE, 0);
  GPIO_PinModeSet( FLEX_MOTOR_DECAY_PORT,  FLEX_MOTOR_DECAY_BIT,  FLEX_MOTOR_DECAY_MODE, 0);

  GPIO_PinModeSet( FLEX_MOTOR_FAULT_PORT,  FLEX_MOTOR_FAULT_BIT,  FLEX_MOTOR_FAULT_MODE, 1);
  
  TIMER_InitCC_TypeDef timerCCInit = FLEX_MOTOR_Timer_CCInit;

  TIMER_InitCC(FLEX_MOTOR_TIMER, FLEX_MOTOR_TIMER_CHANNEL, &timerCCInit); 
  TIMER_TopSet(FLEX_MOTOR_TIMER, FLEX_MOTOR_DEFAULT_TOP);
  TIMER_CompareBufSet(FLEX_MOTOR_TIMER, FLEX_MOTOR_TIMER_CHANNEL, 0); 
  
  
  //  TIMER_CompareBufSet(FLEX_MOTOR_TIMER, FLEX_MOTOR_TIMER_CHANNEL, 100); 
  
  TIMER_Init_TypeDef timerInit = FLEX_MOTOR_Timer_Init;
  TIMER_Init(FLEX_MOTOR_TIMER, &timerInit); 
  FLEX_MOTOR_TIMER->ROUTE |= (FLEX_MOTOR_ROUTE_EN |  FLEX_MOTOR_ROUTE_LOCATION); 


//  TIMER_CompareBufSet(FLEX_MOTOR_TIMER, FLEX_MOTOR_TIMER_CUR_CHANNEL, FLEX_CURRENT_4A); //最大电流4A(80/131*3.3)
}

void Waver_FlexMotor_10ms_Int(void)
{
  bFalg = true; 
}


unsigned int WaverMotor_VoltageAdj(unsigned int setDuty)
{
  unsigned short adc24;      //此处的电压值已经扩大了100倍
  ADC_Get_Voltage(ADC_V24,&adc24);     
  if(adc24 <= WAVER_SET_VOLTAGE/100) 
  {
    return setDuty;        //电压值偏低，返回预设值
  }
  unsigned int scale = WAVER_SET_VOLTAGE / adc24; //计算与设定电压的比例值
  setDuty *= scale;
  unsigned int yushu = setDuty  % 100;
  setDuty /= 100;
  if(yushu > 50) setDuty++;
  return setDuty; 
}



void Waver_FlexkMotor_Set_Pwm_Data(unsigned long ulDuty)
{
  /*
  unsigned int duty ;
  if(ulDuty == 0)
  {
    TIMER_CompareBufSet(FLEX_MOTOR_TIMER, FLEX_MOTOR_TIMER_CHANNEL, ulDuty);
    return;
  }
  duty = TIMER_CompareBufGet(FLEX_MOTOR_TIMER, FLEX_MOTOR_TIMER_CHANNEL);
  if(ulDuty == duty)
  {
   if(Waver_FlexMotor_Get_Fault() == FLEX_MOTOR_NORMAL) return;
    Waver_FlexMotor_Reset();
    __no_operation();
    __no_operation();
    Waver_FlexMotor_Reset_Cancel();
   return; 
  }
  if(!bFalg) return;
  bFalg = false;
  if(duty < ulDuty)
  {
    if(duty < FLEX_MOTOR_DEFAULT_TOP/2)
      duty = FLEX_MOTOR_DEFAULT_TOP/2;
    else duty++;
  }
  TIMER_CompareBufSet(FLEX_MOTOR_TIMER, FLEX_MOTOR_TIMER_CHANNEL, duty);*/
  
  
  if(ulDuty == 0)
  {
      TIMER_CompareBufSet(FLEX_MOTOR_TIMER, FLEX_MOTOR_TIMER_CHANNEL, ulDuty);
      return;
  }
    if(Waver_FlexMotor_Get_Fault() == FLEX_MOTOR_FAIL)
    {
      Waver_FlexMotor_Reset();
      __no_operation();
      __no_operation();
      Waver_FlexMotor_Reset_Cancel();
    }
    ulDuty = WaverMotor_VoltageAdj(ulDuty);
   TIMER_CompareBufSet(FLEX_MOTOR_TIMER, FLEX_MOTOR_TIMER_CHANNEL, ulDuty);   
  
  
  
  
}

int Waver_FlexPower_On(unsigned char speed)
{
  Waver_FlexkMotor_Set_Pwm_Data(speed);

  return 0;
}

void Waver_FlexPower_Off(void)
{
  //GPIO_PinOutClear(FLEX_MOTOR_ENBL_PORT, FLEX_MOTOR_ENBL_BIT);
  Waver_FlexkMotor_Set_Pwm_Data(0);
}

unsigned int Waver_FlexPower_Get(void)
{
  unsigned long  ulDuty;
  ulDuty = TIMER_CompareBufGet(FLEX_MOTOR_TIMER, FLEX_MOTOR_TIMER_CHANNEL);
  if(ulDuty > 0) return FLEX_POWER_ON;
  else return FLEX_POWER_OFF;
}

void Waver_FlexMotor_Out(void)
{
 // Power_On();
  GPIO_PinOutSet(FLEX_MOTOR_RESET_PORT, FLEX_MOTOR_RESET_BIT);
  GPIO_PinOutSet(FLEX_MOTOR_PHASE_PORT, FLEX_MOTOR_PHASE_BIT);
  GPIO_PinOutClear(FLEX_MOTOR_DECAY_PORT, FLEX_MOTOR_DECAY_BIT);
}

void Waver_FlexMotor_In(void)
{
 // Power_On();
  GPIO_PinOutClear(FLEX_MOTOR_DECAY_PORT, FLEX_MOTOR_DECAY_BIT);
  GPIO_PinOutSet(FLEX_MOTOR_RESET_PORT, FLEX_MOTOR_RESET_BIT);
  GPIO_PinOutClear(FLEX_MOTOR_PHASE_PORT, FLEX_MOTOR_PHASE_BIT);
}

void Waver_FlexMotor_Break(void)
{
  GPIO_PinOutClear(FLEX_MOTOR_DECAY_PORT, FLEX_MOTOR_DECAY_BIT);
 // GPIO_PinOutClear(FLEX_MOTOR_ENBL_PORT, FLEX_MOTOR_ENBL_BIT);
  Waver_FlexkMotor_Set_Pwm_Data(0);
}

void Waver_FlexMotor_Reset(void)
{
  GPIO_PinOutClear(FLEX_MOTOR_RESET_PORT, FLEX_MOTOR_RESET_BIT);
}

void Waver_FlexMotor_Reset_Cancel(void)
{
  GPIO_PinOutSet(FLEX_MOTOR_RESET_PORT, FLEX_MOTOR_RESET_BIT);
}

int Waver_FlexMotor_Get_Fault(void)
{
  if(GPIO_PinInGet(FLEX_MOTOR_FAULT_PORT, FLEX_MOTOR_FAULT_BIT))
    return FLEX_MOTOR_NORMAL;
  return FLEX_MOTOR_FAIL;
}

//====================================================================

//150402
void Waveringly_Set_Pwm_Data(unsigned int ulDuty)
{
  Waver_FlexkMotor_Set_Pwm_Data(ulDuty);
  Waver_FlexMotor_In();
}


void Waveringly_Set_Pwm_TEST_Data(unsigned int ulDuty)
{
   TIMER_CompareBufSet(FLEX_MOTOR_TIMER, FLEX_MOTOR_TIMER_CHANNEL, ulDuty);   
   Waver_FlexMotor_In();
}


unsigned int WaveMotor_IsRun(void)
{
 return(Waver_FlexPower_Get());
}