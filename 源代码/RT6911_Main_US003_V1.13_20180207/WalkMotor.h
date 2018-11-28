#ifndef __WALK_MOTOR_H__
#define __WALK_MOTOR_H__

#define WALK_MOTOR_ENBL_PORT   gpioPortB    //PWM
#define WALK_MOTOR_ENBL_BIT    0
#define WALK_MOTOR_ENBL_MODE   gpioModePushPull

#define WALK_MOTOR_PHASE_PORT   gpioPortA
#define WALK_MOTOR_PHASE_BIT    3
#define WALK_MOTOR_PHASE_MODE   gpioModePushPull

#define WALK_MOTOR_DECAY_PORT   gpioPortA
#define WALK_MOTOR_DECAY_BIT    4
#define WALK_MOTOR_DECAY_MODE   gpioModePushPull

#define WALK_MOTOR_FAULT_PORT   gpioPortA
#define WALK_MOTOR_FAULT_BIT    5
#define WALK_MOTOR_FAULT_MODE   gpioModeInputPullFilter

#define WALK_MOTOR_RESET_PORT   gpioPortA
#define WALK_MOTOR_RESET_BIT    6
#define WALK_MOTOR_RESET_MODE   gpioModePushPull

#define STATE_RUN_WALK_DOWN     0
#define STATE_RUN_WALK_UP       1
#define STATE_WALK_IDLE         2
#define STATE_RUN_WALK_POSITION 3

#define WALK_MOTOR_TIMER           TIMER1
#define WALK_MOTOR_TIMER_CHANNEL   0
#define WALK_MOTOR_ROUTE_EN        TIMER_ROUTE_CC0PEN
#define WALK_MOTOR_ROUTE_LOCATION  TIMER_ROUTE_LOCATION_LOC2

#define WALK_MOTOR_PRESCALE       timerPrescale4// timerPrescale4////1/(28/8)HHZ=3.5MHZ=0.285us
#define WALK_MOTOR_DEFAULT_TOP     131// //0.285us*131=37.5us  Fre=26KHZ


#define WALK_SET_VOLTAGE     260000//250000//fww



#define WALK_MOTOR_Timer_CCInit     \
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

#define WALK_MOTOR_Timer_Init       \
   {                                   \
       true,                           \
       true,                           \
       WALK_MOTOR_PRESCALE,            \
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
    WALK_MOTOR_POWER_ON, 
    WALK_MOTOR_POWER_OFF 
};

enum
{
    WALK_MOTOR_GO_UP, 
    WALK_MOTOR_GO_DOWN 
};

enum
{
    WALK_MOTOR_CURRENT_HIGH,
    WALK_MOTOR_CURRENT_LOW
};

enum
{
    WALK_MOTOR_NORMAL,
    WALK_MOTOR_FAIL
};

void WalkPower_On(void);
void WalkPower_Off(void);
void WalkMotor_Up(void);
void WalkMotor_Down(void);
void WalkMotor_Break(void);
void WalkMotor_Reset(void);
void WalkMotor_Reset_Cancel(void);
unsigned int WalkRelay_Get(void);
unsigned int WalkPower_Get(void);
void WalkMotor_Initial_IO(void);
int WalkMotor_Get_Fault(void);
unsigned char WalkMotor_Control(unsigned char nFinalWalkPadMotorState,unsigned short stopPosition);
void WalkMotor_10ms_Int(void);
unsigned int WalkMotor_GetDirection(void);
void WalkMotor_10msInt(void);

unsigned int WalkMotor_VoltageAdj(unsigned int setDuty);



#endif

