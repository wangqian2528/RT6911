#ifndef __LEG_MOTOR_H__
#define __LEG_MOTOR_H__



#ifdef NO_USE_THIS_LEG

#define LEG_MOTOR_ENBL_PORT   gpioPortF
#define LEG_MOTOR_ENBL_BIT    6
#define LEG_MOTOR_ENBL_MODE   gpioModePushPull

#define LEG_MOTOR_PHASE_PORT   gpioPortE
#define LEG_MOTOR_PHASE_BIT    8
#define LEG_MOTOR_PHASE_MODE   gpioModePushPull

#define LEG_MOTOR_DECAY_PORT   gpioPortE
#define LEG_MOTOR_DECAY_BIT    9
#define LEG_MOTOR_DECAY_MODE   gpioModePushPull

#define LEG_MOTOR_FAULT_PORT   gpioPortE
#define LEG_MOTOR_FAULT_BIT    10
#define LEG_MOTOR_FAULT_MODE   gpioModeInputPullFilter

#define LEG_MOTOR_RESET_PORT   gpioPortD
#define LEG_MOTOR_RESET_BIT    11
#define LEG_MOTOR_RESET_MODE   gpioModePushPull




#define LEG_MIN_SPEED                 3


#define LEG_MOTOR_TIMER               TIMER0
#define LEG_MOTOR_CHANNEL             0
#define LEG_MOTOR_ROUTE_EN            TIMER_ROUTE_CC0PEN
#define LEG_MOTOR_ROUTE_LOCATION      TIMER_ROUTE_LOCATION_LOC2






#define LEG_MOTOR_DEFAULT_TOP      100//  100//BACK_MOTOR_DEFAULT_TOP
#define LEG_SET_VOLTAGE             270000//  270000 //25V

#define LEG_MOTOR_PRESCALE     timerPrescale4

#define LEG_MOTOR_Timer_Init       \
{                                   \
    true,                           \
    true,                           \
    LEG_MOTOR_PRESCALE,            \
    timerClkSelHFPerClk,            \
    timerInputActionNone,           \
    timerInputActionNone,           \
    timerModeUp,                    \
    false,                          \
    false,                          \
    false,                          \
    false,                          \
}

#define LEG_MOTOR_Timer_CCInit     \
{                                   \
    timerEventEveryEdge,            \
    timerEdgeBoth,                  \
    timerPRSSELCh0,                 \
    timerOutputActionNone,          \
    timerOutputActionNone,          \
    timerOutputActionToggle,        \
    timerCCModePWM,                 \
    false,                          \
    false,                          \
    false,                          \
    false,                          \
}

enum
{
  LEG_MOTOR_POWER_ON, 
  LEG_MOTOR_POWER_OFF 
};

enum
{
  LEG_MOTOR_GO_UP, 
  LEG_MOTOR_GO_DOWN 
};

enum
{
 LEG_MOTOR_CURRENT_HIGH,
 LEG_MOTOR_CURRENT_LOW
};

enum
{
 LEG_MOTOR_NORMAL,
 LEG_MOTOR_FAIL
};
#define LEG_MOTOR_AT_MID     0
#define LEG_MOTOR_AT_BOTTOM  1 //小腿电机在最下面
#define LEG_MOTOR_AT_TOP     2  //小腿电机的状态，在最上面
//leg motor max run time
#define LEG_MOTOR_MAX_POSITION 1600

#define STATE_RUN_LEG_DOWN  0    //小腿电机向下运行
#define STATE_RUN_LEG_UP    1    //小腿电机向上运行
#define STATE_LEG_IDLE      2

#define  LEG_RUN            0
#define  LEG_STOP_AT_DOWN   1
#define  LEG_STOP_AT_GROUND 2
#define  LEG_STOP_AT_UP     3
#define  LEG_STOP_IDLE      4
#define  LEG_STOP_AT_ANGLE  5

void LegMotor_Initial_IO(void);
void LegRelay_On(void);
void LegRelay_Off(void);
int LegPower_On(void);
void LegPower_Off(void);
unsigned int LegRelay_Get(void);
unsigned int LegPower_Get(void);

//void LegMotor_Set_Current(int current);
void LegMotor_Up(void);
void LegMotor_Down(void);
void LegMotor_Break(void);
void LegMotor_Reset(void);
int LegMotor_Get_Fault(void);
unsigned int LegMotor_Get_Location(void);

void LegMotor_10ms_int(void);
void LegMotor_Proce(void);
unsigned char LegMotor_Control(unsigned char nFinalLegPadMotorState);
unsigned int LegMotor_Get_Position(void);

int LegMotor_GetPower(void);
int LegMotor_GetDirection(void);
unsigned int LegPower_Get(void);
unsigned int LegPower_Get(void);
void LegMotor_Set_PWM(unsigned long ulDuty);


#else

#define LEG_MOTOR_ENBL_PORT   gpioPortF
#define LEG_MOTOR_ENBL_BIT    6
#define LEG_MOTOR_ENBL_MODE   gpioModePushPull

#define LEG_MOTOR_PHASE_PORT   gpioPortE
#define LEG_MOTOR_PHASE_BIT    8
#define LEG_MOTOR_PHASE_MODE   gpioModePushPull

#define LEG_MOTOR_DECAY_PORT   gpioPortE
#define LEG_MOTOR_DECAY_BIT    9
#define LEG_MOTOR_DECAY_MODE   gpioModePushPull

#define LEG_MOTOR_FAULT_PORT   gpioPortE
#define LEG_MOTOR_FAULT_BIT    10
#define LEG_MOTOR_FAULT_MODE   gpioModeInputPullFilter

#define LEG_MOTOR_RESET_PORT   gpioPortD  //gpioPortE
#define LEG_MOTOR_RESET_BIT    11
#define LEG_MOTOR_RESET_MODE   gpioModePushPull

#define LEG_MOTOR_TIMER           TIMER0
#define LEG_MOTOR_TIMER_CHANNEL   0
#define LEG_MOTOR_ROUTE_EN        TIMER_ROUTE_CC0PEN
#define LEG_MOTOR_ROUTE_LOCATION  TIMER_ROUTE_LOCATION_LOC2

#define LEG_MOTOR_PRESCALE        timerPrescale4
#define LEG_MOTOR_DEFAULT_TOP     131

#define LEG_MOTOR_Timer_CCInit     \
{                                   \
    timerEventEveryEdge,            \
    timerEdgeBoth,                  \
    timerPRSSELCh0,                 \
    timerOutputActionNone,          \
    timerOutputActionNone,          \
    timerOutputActionToggle,        \
    timerCCModePWM,                 \
    false,                          \
    false,                          \
    false,                          \
    false,                          \
} 

#define LEG_MOTOR_Timer_Init       \
{                                   \
    true,                           \
    true,                           \
    LEG_MOTOR_PRESCALE,             \
    timerClkSelHFPerClk,            \
    false,                          \
    false,                          \
    timerInputActionNone,           \
    timerInputActionNone,           \
    timerModeUp,                    \
    false,                          \
    false,                          \
    false,                          \
    false,                          \
}


enum
{
  LEG_MOTOR_POWER_ON, 
  LEG_MOTOR_POWER_OFF 
};

enum
{
  LEG_MOTOR_GO_UP, 
  LEG_MOTOR_GO_DOWN 
};

enum
{
 LEG_MOTOR_CURRENT_HIGH,
 LEG_MOTOR_CURRENT_LOW
};

enum
{
 LEG_MOTOR_NORMAL,
 LEG_MOTOR_FAIL
};

#define LEG_SET_VOLTAGE   260000

#define LEG_MOTOR_AT_MID     0
#define LEG_MOTOR_AT_BOTTOM  1
#define LEG_MOTOR_AT_TOP     2
//leg motor max run time
#define LEG_MOTOR_MAX_POSITION 1200//1600//1111

#define STATE_RUN_LEG_DOWN  0
#define STATE_RUN_LEG_UP    1
#define STATE_LEG_IDLE      2

#define  LEG_RUN            0
#define  LEG_STOP_AT_DOWN   1
#define  LEG_STOP_AT_GROUND 2
#define  LEG_STOP_AT_UP     3
#define  LEG_STOP_IDLE      4
#define  LEG_STOP_AT_ANGLE  5

void LegMotor_Initial_IO(void);
//void LegRelay_On(void);
//void LegRelay_Off(void);
//int LegPower_On(void);
void LegPower_Off(void);
//unsigned int LegRelay_Get(void);
unsigned int LegPower_Get(void);

//void LegMotor_Set_Current(int current);
void LegMotor_Up(void);
void LegMotor_Down(void);
void LegMotor_Break(void);
void LegMotor_Reset(void);
void LegMotor_Reset_Cancel(void);
int LegMotor_Get_Fault(void);
unsigned int LegMotor_Get_Location(void);

void LegMotor_10ms_int(void);
void LegMotor_Proce(void);
unsigned char LegMotor_Control(unsigned char nFinalLegPadMotorState);
unsigned int LegMotor_Get_Position(void);
unsigned int LegMotor_VoltageAdj(unsigned int setDuty);
void LegMotor_Set_Pwm_Data(unsigned long ulDuty);
int LegMotor_GetDirection(void);
int LegMotor_GetPower(void);
unsigned int LegPower_Get(void);





#endif



#endif
