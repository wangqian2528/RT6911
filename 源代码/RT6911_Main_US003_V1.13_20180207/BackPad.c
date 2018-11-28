#include "EFM32_def.h"
#include "EFM32_types.h"
#include "efm32.h"
//#include "em_device.h"
#include "em_chip.h"
#include "em_gpio.h"
#include "em_timer.h"
#include "input.h"
#include "MassageStatus.h"
#include "valve.h"
#include "ADC_Scan.h"
//#include "SlideMotor.h"
//#include "LegMotor.h"


#include "BackPad.h"


bool speedUp_10MS_Int = false; 
bool slow_break_10MS_Int = false;
bool manslow_break_50MS_Int = false;
bool isRocking = false;
bool isStretching = false;
int currentSpeed = BACK_MOTOR_DEFAULT_TOP;
unsigned char currentBackPadMotorState = STATE_BACK_IDLE;



//�����綯��λ�� ��ߣ�0 ��ͣ����λ��
//static UINT32 w_Position;
//static bool bBackMotorFlag;

//static unsigned int motorStatus = MOTOR_STOP_BREAK;
static volatile unsigned int motorBreakTime;

void BackMotor_Initial_IO(void)
{
  GPIO_PinModeSet(BACK_MOTOR_RESET_PORT, BACK_MOTOR_RESET_BIT, BACK_MOTOR_RESET_MODE, 0);
  GPIO_PinModeSet(BACK_MOTOR_ENBL_PORT, BACK_MOTOR_ENBL_BIT, BACK_MOTOR_ENBL_MODE, 0);
  GPIO_PinModeSet(BACK_MOTOR_PHASE_PORT, BACK_MOTOR_PHASE_BIT, BACK_MOTOR_PHASE_MODE, 0);
  GPIO_PinModeSet(BACK_MOTOR_DECAY_PORT, BACK_MOTOR_DECAY_BIT, BACK_MOTOR_DECAY_MODE, 0);
  GPIO_PinModeSet(BACK_MOTOR_FAULT_PORT, BACK_MOTOR_FAULT_BIT, BACK_MOTOR_FAULT_MODE, 1);
  /*
  GPIO_PinModeSet(BACK_MOTOR_RESET_PORT, BACK_MOTOR_RESET_BIT, BACK_MOTOR_RESET_MODE, 1);
  GPIO_PinModeSet(BACK_MOTOR_ENBL_PORT, BACK_MOTOR_ENBL_BIT, BACK_MOTOR_ENBL_MODE, 1);
  GPIO_PinModeSet(BACK_MOTOR_PHASE_PORT, BACK_MOTOR_PHASE_BIT, BACK_MOTOR_PHASE_MODE, 1);
  GPIO_PinModeSet(BACK_MOTOR_DECAY_PORT, BACK_MOTOR_DECAY_BIT, BACK_MOTOR_DECAY_MODE, 0);
  GPIO_PinModeSet(BACK_MOTOR_FAULT_PORT, BACK_MOTOR_FAULT_BIT, BACK_MOTOR_FAULT_MODE, 1);
  
  Power_On();
  while(1);
  */
  TIMER_InitCC_TypeDef timerCCInit = BACK_MOTOR_Timer_CCInit;
  TIMER_InitCC(BACK_MOTOR_TIMER, BACK_MOTOR_TIMER_CHANNEL, &timerCCInit);
  BACK_MOTOR_TIMER->ROUTE |= (BACK_MOTOR_ROUTE_EN | BACK_MOTOR_ROUTE_LOCATION); 
  TIMER_TopSet(BACK_MOTOR_TIMER, BACK_MOTOR_DEFAULT_TOP);
  TIMER_CompareBufSet(BACK_MOTOR_TIMER, BACK_MOTOR_TIMER_CHANNEL, 0);
  TIMER_Init_TypeDef timerInit = BACK_MOTOR_Timer_Init;
  TIMER_Init(BACK_MOTOR_TIMER, &timerInit);
  
}
/*
int BackMotor_GetPower(void)
{
  if(GPIO_PinOutGet(BACK_MOTOR_ENBL_PORT,BACK_MOTOR_ENBL_BIT))
  {
   return BACK_MOTOR_POWER_ON; 
  }
  return BACK_MOTOR_POWER_OFF; 
}
*/
int BackMotor_GetDirection(void)
{
  if(GPIO_PinOutGet(BACK_MOTOR_PHASE_PORT,BACK_MOTOR_PHASE_BIT))
  {
   return BACK_MOTOR_GO_UP;//BACK_MOTOR_GO_DOWN; 
  }
  return BACK_MOTOR_GO_DOWN; //BACK_MOTOR_GO_UP; 
}

void BackPower_Off(void)
{
  TIMER_CompareBufSet(BACK_MOTOR_TIMER, BACK_MOTOR_TIMER_CHANNEL, 0);
  //BackMotor_Set_Pwm_Data(0);
}

unsigned int BackMotor_VoltageAdj(unsigned int setDuty)
{
  unsigned short adc24;      //�˴��ĵ�ѹֵ�Ѿ�������100��
  ADC_Get_Voltage(ADC_V24,&adc24);     
  if(adc24 <= BACK_SET_VOLTAGE/100) 
  {
    return setDuty;        //��ѹֵƫ�ͣ�����Ԥ��ֵ
  }
  unsigned int scale = BACK_SET_VOLTAGE / adc24; //�������趨��ѹ�ı���ֵ
  setDuty *= scale;
  unsigned int yushu = setDuty  % 100;
  setDuty /= 100;
  if(yushu > 50) setDuty++;
  return setDuty; 
}
void BackMotor_Set_Pwm_Data(unsigned long ulDuty)
{
 // unsigned int duty ;
  if(ulDuty == 0)
  {
    TIMER_CompareBufSet(BACK_MOTOR_TIMER, BACK_MOTOR_TIMER_CHANNEL, ulDuty);
    currentBackPadMotorState = STATE_BACK_IDLE;
    return;
  }

  if(BackMotor_Get_Fault() == BACK_MOTOR_FAIL) 
   {
      BackMotor_Reset();
     __no_operation();
     __no_operation();
     BackMotor_Reset_Cancel();
   }

  ulDuty = BackMotor_VoltageAdj(ulDuty);
  TIMER_CompareBufSet(BACK_MOTOR_TIMER, BACK_MOTOR_TIMER_CHANNEL, ulDuty);
}
/*
void BackPower_On(void)
{
  unsigned long  ulDuty;
  ulDuty = BackMotor_VoltageAdj();
  if(BackMotor_Get_Fault() == BACK_MOTOR_FAIL) 
   {
      BackMotor_Reset();
     __no_operation();
     __no_operation();
     BackMotor_Reset_Cancel();
   }
  TIMER_CompareBufSet(BACK_MOTOR_TIMER, BACK_MOTOR_TIMER_CHANNEL, ulDuty);
}*/
/*
static void BackPower_On(void)
{
  BackMotor_Set_Pwm_Data(BACK_MOTOR_DEFAULT_TOP);
}*/
//static void BackPower_Off(void)
//{
//  BackMotor_Set_Pwm_Data(0);
//}

/*
void BackMotor_Up(void)
{
  //Power_On();
  GPIO_PinOutSet(BACK_MOTOR_RESET_PORT, BACK_MOTOR_RESET_BIT);
  GPIO_PinOutClear(BACK_MOTOR_PHASE_PORT, BACK_MOTOR_PHASE_BIT);
  GPIO_PinOutSet(BACK_MOTOR_DECAY_PORT, BACK_MOTOR_DECAY_BIT);
}
void BackMotor_Down(void)
{
  //Power_On();
  GPIO_PinOutSet(BACK_MOTOR_RESET_PORT, BACK_MOTOR_RESET_BIT);
  GPIO_PinOutSet(BACK_MOTOR_PHASE_PORT, BACK_MOTOR_PHASE_BIT);
  GPIO_PinOutSet(BACK_MOTOR_DECAY_PORT, BACK_MOTOR_DECAY_BIT);
}
*/
void speedUp(void)
{
  if(!speedUp_10MS_Int) return;
  speedUp_10MS_Int = false;
  //������Ҫ����С���ȣ���ѹռ�ձȣ�
  if(currentSpeed < BACK_MIN_SPEED)currentSpeed = BACK_MIN_SPEED;
  if(isRocking)
  {
    if(currentSpeed < BACK_MOTOR_DEFAULT_TOP - 10/* - 20*/)
    {
      currentSpeed++;
    }
  }
  else
  {
    if(currentSpeed < BACK_MOTOR_DEFAULT_TOP)
    {
      currentSpeed+=2;
    }  
  }
}
void speedstretchUp(void){
  if(!speedUp_10MS_Int) return;
  speedUp_10MS_Int = false;
  //������Ҫ����С���ȣ���ѹռ�ձȣ�
  if(currentSpeed < BACK_MIN_SPEED)currentSpeed = BACK_MIN_SPEED;
  if(isRocking)
  {
    if(currentSpeed < BACK_MOTOR_DEFAULT_TOP - 10/* - 20*/)
    {
      currentSpeed++;
    }
  }
  else
  {
           if(currentSpeed < BACK_MOTOR_STRETCH_TOP)
          {
            currentSpeed+=2;
          }  
  }
}



void BackMotor_SlowBreak(void){
  
  if(!slow_break_10MS_Int)return;
  slow_break_10MS_Int = false;
 // if(currentSpeed > 0)currentSpeed--;
 // if(currentSpeed < BACK_MIN_SPEED)currentSpeed = 0;
  
  
   if(currentSpeed > 75)
   {
     currentSpeed = 75;
   }
  if(currentSpeed > BACK_MIN_SPEED)
  {
    //�˵㼱ɲ��
    if(Input_GetBackDownSwitch() == REACH_BACK_LIMIT ||
       Input_GetBackUpSwitch() == REACH_BACK_LIMIT)
    {
      currentSpeed -= 3+currentSpeed/40+currentSpeed/20;
    }
    //�м�һ��ɲ��
    else
    {
      currentSpeed -= 1+currentSpeed/100+currentSpeed/70;
    }
  }
  if(currentSpeed <= BACK_MIN_SPEED)currentSpeed = 0;
  //currentSpeed = 0;
}

void BackMotor_SlowManBreak(void)
{
 if(!manslow_break_50MS_Int)return;
  manslow_break_50MS_Int = false;
   //if(currentSpeed > 75)
   //{
   //  currentSpeed = 75;
   //}
  if(currentSpeed > BACK_MIN_SPEED)
  {
    //�˵㼱ɲ��
    if(Input_GetBackDownSwitch() == REACH_BACK_LIMIT ||
       Input_GetBackUpSwitch() == REACH_BACK_LIMIT)
    {
      currentSpeed -= 3+currentSpeed/40+currentSpeed/20;
    }
    //�м�һ��ɲ��
    else
    {
      currentSpeed -= 1+currentSpeed/100+currentSpeed/70;
    }
  }
  if(currentSpeed <= BACK_MIN_SPEED)currentSpeed = 0;
  
  //currentSpeed = 0;
}
//���������������8600S��
void BackMotor_Down(void)
{
  GPIO_PinOutSet(BACK_MOTOR_RESET_PORT, BACK_MOTOR_RESET_BIT);
  //GPIO_PinOutSet(BACK_MOTOR_PHASE_PORT, BACK_MOTOR_PHASE_BIT);
   GPIO_PinOutClear(BACK_MOTOR_PHASE_PORT, BACK_MOTOR_PHASE_BIT);
    
  GPIO_PinOutClear(BACK_MOTOR_DECAY_PORT, BACK_MOTOR_DECAY_BIT);
  // GPIO_PinOutSet(BACK_MOTOR_DECAY_PORT, BACK_MOTOR_DECAY_BIT);//Ϊ�ߵ�ƽʱ��250V ��������ѹ���½���15V
}
void BackMotor_Up(void)
{
  
  GPIO_PinOutSet(BACK_MOTOR_RESET_PORT, BACK_MOTOR_RESET_BIT);
  //GPIO_PinOutClear(BACK_MOTOR_PHASE_PORT, BACK_MOTOR_PHASE_BIT);
  GPIO_PinOutSet(BACK_MOTOR_PHASE_PORT, BACK_MOTOR_PHASE_BIT);
  
  
  GPIO_PinOutClear(BACK_MOTOR_DECAY_PORT, BACK_MOTOR_DECAY_BIT);
  
   //GPIO_PinOutSet(BACK_MOTOR_DECAY_PORT, BACK_MOTOR_DECAY_BIT);//Ϊ�ߵ�ƽʱ��250V ��������ѹ���½���15V
  
}
void BackMotor_Break(void)
{
  /*
  BackPower_Off();    //�ر�����Դ
  if(motorStatus == MOTOR_RUN)
  {
  //tt GPIO_PinOutSet(BACK_MOTOR_DECAY_PORT, BACK_MOTOR_DECAY_BIT); //ʹ���˿ڴ��ڸ��裬��ʱ������ڹ��Ի���
   motorStatus = MOTOR_STOP_HZ;
   motorBreakTime = 0;
  }
  if(motorStatus == MOTOR_STOP_HZ)
  {
    if(motorBreakTime < MOTOR_STOP_HZ_TIME) return;
  }
 //tt GPIO_PinOutSet(BACK_MOTOR_DECAY_PORT, BACK_MOTOR_DECAY_BIT);
  //GPIO_PinOutClear(BACK_MOTOR_DECAY_PORT, BACK_MOTOR_DECAY_BIT);  //��·�����������ɲ��״̬
  motorStatus = MOTOR_STOP_BREAK;
  */

  //if(!breakDown_10MS_Int)return;
  //breakDown_10MS_Int = false;
  //�˵�ɲ������137.5
  if(currentSpeed > BACK_MIN_SPEED)
  {
    //�˵㼱ɲ��
    //if(Input_GetBackDownSwitch() == REACH_BACK_LIMIT ||
    //   Input_GetBackUpSwitch() == REACH_BACK_LIMIT)
    {
      currentSpeed -= 3+currentSpeed/40+currentSpeed/20;
    }
    //�м�һ��ɲ��
   /* else
    {
      currentSpeed -= 1+currentSpeed/100+currentSpeed/70;
    }*/
  }
  if(currentSpeed <= BACK_MIN_SPEED)currentSpeed = 0;
  
  
}

void BackMotor_Reset(void)
{
  GPIO_PinOutClear(BACK_MOTOR_RESET_PORT, BACK_MOTOR_RESET_BIT);
}
void BackMotor_Reset_Cancel(void)
{
  GPIO_PinOutSet(BACK_MOTOR_RESET_PORT, BACK_MOTOR_RESET_BIT);
}
int BackMotor_Get_Fault(void)
{
  if(GPIO_PinInGet(BACK_MOTOR_FAULT_PORT, BACK_MOTOR_FAULT_BIT))
    return BACK_MOTOR_NORMAL;
  return BACK_MOTOR_FAIL;
}
//���ؿ����綯�׵Ĵ�λ�� ��ߣ���ͻ��м�
/*unsigned int BackMotor_Get_Location(void)
{
  if(Input_GetBackUpSwitch() == REACH_BACK_LIMIT)   return BACK_MOTOR_AT_TOP;
  if(Input_GetBackDownSwitch() == REACH_BACK_LIMIT)   return BACK_MOTOR_AT_BOTTOM;
  return BACK_MOTOR_AT_MID;
}*/
//���ؿ����綯�׵ľ���λ�ã���ʱ������¼�������綯��λ����ߴ�ʱ��Ϊ0����λ10ms
//unsigned int BackMotor_Get_Position(void)
//{
//  return w_Position;
//}

int BackMotor_GetPower(void)
{
  if(TIMER_CompareBufGet(BACK_MOTOR_TIMER, BACK_MOTOR_TIMER_CHANNEL))
  {
   return BACK_MOTOR_POWER_ON; 
  }
  return BACK_MOTOR_POWER_OFF; 
}

void BackMotor_10ms_int(void)
{
  //bBackMotorFlag = TRUE;
  speedUp_10MS_Int = TRUE;
  slow_break_10MS_Int = TRUE;
  //breakDown_10MS_Int = TRUE;
  
  manslow_break_50MS_Int = TRUE;
  
}
/*
void BackMotor_Proce(void)
{
  if(!bBackMotorFlag) return;
  
  if(REACH_BACK_LIMIT == Input_GetBackDownSwitch())
    {
      w_Position = BACK_MOTOR_MAX_POSITION; 
      return;
    }
  if(REACH_BACK_LIMIT == Input_GetBackUpSwitch())
    {
      w_Position = 0; 
      return;
    }
  
  bBackMotorFlag = FALSE;
  
  if(!TIMER_CompareBufGet(BACK_MOTOR_TIMER, BACK_MOTOR_TIMER_CHANNEL)) return;
  
  if(!GPIO_PinOutGet(BACK_MOTOR_PHASE_PORT, BACK_MOTOR_PHASE_BIT))
  {
    if(w_Position > 0) w_Position--;  
  }
  else
  {
    if(w_Position < BACK_MOTOR_MAX_POSITION) w_Position++;  
  }
}*/
void currentBackPadMotorState_reset()
{
  //currentBackPadMotorState = STATE_BACK_IDLE;
}
//BackPad motor control function
unsigned char BackMotor_Control(unsigned char nFinalBackPadMotorState)
{
  //static unsigned int position = 0;
  unsigned char nRetVal ;
  //bool bPowerFlag;
  nRetVal = FALSE ;
  
  unsigned char speedState = BACK_SPEED_STATE_STOP;//ֹͣ0��ɲ��1������2����ɲ��3��������4
  //nBackLoss =0;
  //BackMotor_Proce();
  if(currentBackPadMotorState != nFinalBackPadMotorState
     && currentBackPadMotorState != STATE_BACK_IDLE)
  {
    if((currentBackPadMotorState == STATE_RUN_BACK_DOWN && Input_GetBackDownSwitch() == REACH_BACK_LIMIT)
       ||(currentBackPadMotorState == STATE_RUN_BACK_UP && Input_GetBackUpSwitch() == REACH_BACK_LIMIT))
    {
      //����ʱ����ɲ��
      speedState = BACK_SPEED_STATE_BREAK;
    }
    else if(isRocking)//((isRocking)||(isStretching))
    {
      //ҡ��ʱ,��ɲ��
      speedState = BACK_SPEED_STATE_SLOW_BREAK;
    }
    else
    {
      //�ֶ�ʱ����ɲ��
      speedState = BACK_SPEED_STATE_SLOW_MAN_BREAK;//BACK_SPEED_STATE_BREAK;
    }
    if(currentSpeed <= 0)
    {
      currentBackPadMotorState = nFinalBackPadMotorState;
    }
    
  }
  else
  {
    currentBackPadMotorState = nFinalBackPadMotorState;
    switch(nFinalBackPadMotorState)
    {
    case STATE_RUN_BACK_DOWN:  //back motor go down

      if(Input_GetBackDownSwitch() == REACH_BACK_LIMIT)
      {
        nRetVal = TRUE;
        speedState = BACK_SPEED_STATE_BREAK;
        break;
      }
      else
      {
        speedState = BACK_SPEED_STATE_UP;
        BackMotor_Down();
      }
      break ;
    case STATE_RUN_BACK_UP:  //back motor go up
      if(Input_GetBackUpSwitch() == REACH_BACK_LIMIT)
      {
        nRetVal = TRUE;
        speedState = BACK_SPEED_STATE_BREAK;
        break;
      }else{
        speedState = BACK_SPEED_STATE_SLOW_UP;//BACK_SPEED_STATE_UP;
        BackMotor_Up();
      }
      break ;
    case STATE_BACK_IDLE:
      nRetVal = TRUE ;
      speedState = BACK_SPEED_STATE_BREAK;
      break ;
    default://�쳣����
      break ;
    }
    //����������
  }
  //���÷���
  if(currentBackPadMotorState == STATE_RUN_BACK_DOWN)
  {
    BackMotor_Down();
  }
  else if(currentBackPadMotorState == STATE_RUN_BACK_UP)
  {
    BackMotor_Up();
  }
  //////////////////////////
    //�����ٶ�
  switch(speedState){
  case BACK_SPEED_STATE_STOP:
    currentSpeed = 0;
    break;
  case BACK_SPEED_STATE_BREAK:
    currentSpeed = 0;
    //BackMotor_Break();
    BackMotor_Set_Pwm_Data(0);
    //GPIO_PinOutSet(BACK_MOTOR_DECAY_PORT, BACK_MOTOR_DECAY_BIT);
    break;
  case BACK_SPEED_STATE_UP:
    speedUp();
    GPIO_PinOutClear(BACK_MOTOR_DECAY_PORT, BACK_MOTOR_DECAY_BIT);
    break;
  case BACK_SPEED_STATE_SLOW_BREAK:
    BackMotor_SlowBreak();
    //GPIO_PinOutSet(BACK_MOTOR_DECAY_PORT, BACK_MOTOR_DECAY_BIT);
    break;
  case BACK_SPEED_STATE_SLOW_MAN_BREAK:
    BackMotor_SlowManBreak();
   // GPIO_PinOutSet(BACK_MOTOR_DECAY_PORT, BACK_MOTOR_DECAY_BIT);
    break;
  case BACK_SPEED_STATE_SLOW_UP:
    //ҡ�μ����ж��ں�����
    speedUp();
    GPIO_PinOutClear(BACK_MOTOR_DECAY_PORT, BACK_MOTOR_DECAY_BIT);
    break;
  }

  //Խ�紦��
  
  if(currentSpeed > BACK_MOTOR_DEFAULT_TOP)
  {
    currentSpeed = BACK_MOTOR_DEFAULT_TOP;
  }
  
  if(currentSpeed < 0)
  {
    currentSpeed = 0;
  }
  GPIO_PinOutClear(BACK_MOTOR_DECAY_PORT, BACK_MOTOR_DECAY_BIT);
  //GPIO_PinOutSet(BACK_MOTOR_DECAY_PORT, BACK_MOTOR_DECAY_BIT);
  //���õ�ѹռ�ձȣ�Ӱ�������ٶ�
  BackMotor_Set_Pwm_Data(currentSpeed);
  return nRetVal ;
 
}
//BackPad motor control function
unsigned char BackMotor_Stretch_Control(unsigned char nFinalBackPadMotorState)
{
  //static unsigned int position = 0;
  unsigned char nRetVal ;
  //bool bPowerFlag;
  nRetVal = FALSE ;
  unsigned char speedState = BACK_SPEED_STATE_STOP;//ֹͣ0��ɲ��1������2����ɲ��3��������4
  //BackMotor_Proce();
  if(currentBackPadMotorState != nFinalBackPadMotorState
     && currentBackPadMotorState != STATE_BACK_IDLE)
  {
    if((currentBackPadMotorState == STATE_RUN_BACK_DOWN && Input_GetBackDownSwitch() == REACH_BACK_LIMIT)
       ||(currentBackPadMotorState == STATE_RUN_BACK_UP && Input_GetBackUpSwitch() == REACH_BACK_LIMIT))
    {
      //����ʱ����ɲ��
      speedState = BACK_SPEED_STATE_BREAK;
      BackMotor_Set_Pwm_Data(0);
    }
    else if(isStretching)
    {
      //ҡ��ʱ,��ɲ��
      speedState = BACK_SPEED_STATE_SLOW_BREAK;
    }
    else
    {
      //�ֶ�ʱ����ɲ��
      speedState = BACK_SPEED_STATE_SLOW_BREAK;//by wgh 20150314
    }
    if(currentSpeed <= 0)
    {
      currentBackPadMotorState = nFinalBackPadMotorState;
    }
    
  }
  else
  {
    currentBackPadMotorState = nFinalBackPadMotorState;
    switch(nFinalBackPadMotorState)
    {
    case STATE_RUN_BACK_DOWN:  //back motor go down

      if(Input_GetBackDownSwitch() == REACH_BACK_LIMIT)
      {
        BackMotor_Set_Pwm_Data(0);
        nRetVal = TRUE;
        speedState = BACK_SPEED_STATE_BREAK;
        break;
      }
      else
      {
        speedState = BACK_SPEED_STATE_UP;
        BackMotor_Down();
      }
      break ;
    case STATE_RUN_BACK_UP:  //back motor go up
     
      if(Input_GetBackUpSwitch() == REACH_BACK_LIMIT)
      {
        nRetVal = TRUE;
        speedState = BACK_SPEED_STATE_BREAK;
        BackMotor_Set_Pwm_Data(0);
        break;
      }
      else
      {
        speedState = BACK_SPEED_STATE_UP;
        BackMotor_Up();
      }
      break ;
    case STATE_BACK_IDLE:

       if(( Input_GetBackDownSwitch() == REACH_BACK_LIMIT)
       || (Input_GetBackUpSwitch() == REACH_BACK_LIMIT))
       {
         speedState = BACK_SPEED_STATE_BREAK;
       }
       else
       {
         speedState = BACK_SPEED_STATE_SLOW_BREAK;
       }
      nRetVal = TRUE ;
      break ;
    default://�쳣����
      break ;
    }
    //����������
  }
  //���÷���
  if(currentBackPadMotorState == STATE_RUN_BACK_DOWN)
  {
    BackMotor_Down();
  }else if(currentBackPadMotorState == STATE_RUN_BACK_UP)
  {
    BackMotor_Up();
  }
  //////////////////////////
    //�����ٶ�
  switch(speedState){
  case BACK_SPEED_STATE_STOP:
    currentSpeed = 0;
    BackMotor_Set_Pwm_Data(0);
    break;
  case BACK_SPEED_STATE_BREAK:
    //BackMotor_SlowBreak();
    currentSpeed = 0;
    BackMotor_Set_Pwm_Data(0);
    //BackMotor_Break();
    break;
  case BACK_SPEED_STATE_UP:
    speedstretchUp();
    break;
  case BACK_SPEED_STATE_SLOW_BREAK:
    BackMotor_SlowBreak();
    break;
  case BACK_SPEED_STATE_SLOW_UP:
    //ҡ�μ����ж��ں�����
    speedUp();
    break;
  }

  //Խ�紦��

      if(currentSpeed > BACK_MOTOR_STRETCH_TOP)
      {
        currentSpeed = BACK_MOTOR_STRETCH_TOP;
      }

  if(currentSpeed < 0)
  {
    currentSpeed = 0;
  }
  //set:fast decay, clear:slow decay
  //set����ɲ���У�clear����ɲ����
  GPIO_PinOutClear(BACK_MOTOR_DECAY_PORT, BACK_MOTOR_DECAY_BIT);
  //GPIO_PinOutSet(BACK_MOTOR_DECAY_PORT, BACK_MOTOR_DECAY_BIT);
  //���õ�ѹռ�ձȣ�Ӱ�������ٶ�
  BackMotor_Set_Pwm_Data(currentSpeed);
  
  /*
  //��Դ���ֵĴ���
  if(bPowerFlag == TRUE)
  {
    BackPower_On();
  }
  else
  {
    BackPower_Off();
  }*/
  return nRetVal ;
 
}
void SetRockingEnable(bool flag)
{
  isRocking = flag;
}


void SetStretchingEnable(bool flag)
{
  isStretching = flag;
}






