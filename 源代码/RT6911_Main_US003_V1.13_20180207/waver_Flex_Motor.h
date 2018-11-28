#ifndef __FLEXPAD_H__
#define __FLEXPAD_H__

#define  FLEX_MOTOR_ENBL_PORT   gpioPortB  //pwm
#define  FLEX_MOTOR_ENBL_BIT    1
#define  FLEX_MOTOR_ENBL_MODE   gpioModePushPull



#define  FLEX_MOTOR_PHASE_PORT   gpioPortC
#define  FLEX_MOTOR_PHASE_BIT    6
#define  FLEX_MOTOR_PHASE_MODE   gpioModePushPull

#define  FLEX_MOTOR_FAULT_PORT   gpioPortC
#define  FLEX_MOTOR_FAULT_BIT    7
#define  FLEX_MOTOR_FAULT_MODE   gpioModeInputPullFilter

#define  FLEX_MOTOR_DECAY_PORT   gpioPortC
#define  FLEX_MOTOR_DECAY_BIT    12
#define  FLEX_MOTOR_DECAY_MODE   gpioModePushPull

#define  FLEX_MOTOR_RESET_PORT   gpioPortC
#define  FLEX_MOTOR_RESET_BIT    13
#define  FLEX_MOTOR_RESET_MODE   gpioModePushPull
/*
#define STATE_RUN_FLEX_IN   0  
#define STATE_RUN_FLEX_OUT  1  
#define STATE_FLEX_IDLE     2
#define STATE_RUN_FLEX_RESET  3
#define STATE_RUN_FLEX_MANUAL_OUT 4
#define STATE_RUN_FLEX_TEST_OUT   5

#define FLEX_RUN                0 //正常运行
#define FLEX_STOP_AT_IN         1 //因为碰到最里面行程开关而停止
#define FLEX_STOP_AT_FOOT       2 //因为碰到脚底开关而停止
#define FLEX_STOP_AT_FOOT_LEAVE 3 //因为碰不到脚底开关而停止
#define FLEX_STOP_AT_OUT        4 //因为碰到最外面行程开关而停止
#define FLEX_STOP_AT_IDLE       5 //因为收到停止命令而停止
#define FLEX_STOP_AT_ANGLE      6 //因为角度而停止
#define FLEX_STOP_AT_GROUND     7 //因为角度而停止
*/
#define FLEX_POWER_ON  1
#define FLEX_POWER_OFF 0

#define  FLEX_MOTOR_TIMER           TIMER1

#define  FLEX_MOTOR_TIMER_CHANNEL     1
#define  FLEX_MOTOR_ROUTE_EN          TIMER_ROUTE_CC1PEN

//#define FLEX_MOTOR_TIMER_CUR_CHANNEL 0
//#define FLEX_MOTOR_ROUTE_CUR_EN      TIMER_ROUTE_CC0PEN

#define  FLEX_MOTOR_ROUTE_LOCATION  TIMER_ROUTE_LOCATION_LOC2

#define  FLEX_MOTOR_PRESCALE        timerPrescale8
#define  FLEX_MOTOR_DEFAULT_TOP     131

//#define FLEX_MOTOR_CMU_TIMER       cmuClock_TIMER0


#define WAVER_SET_VOLTAGE   300000// 280000 //扩大10000倍


/*
#define FLEX_SPEED_STOP     0
#define FLEX_SPEED_SLOW     85
#define FLEX_SPEED_MID      110
#define FLEX_SPEED_FAST     FLEX_MOTOR_DEFAULT_TOP*/
/*
电流计算方法：（I*0.1*5）/3.3*FLEX_MOTOR_DEFAULT_TOP
*/

/*
#define FLEX_CURRENT_2A    40 
#define FLEX_CURRENT_3A    84//73//62 临时测试
#define FLEX_CURRENT_4A    83
#define FLEX_CURRENT_5A    105
#define FLEX_CURRENT_RESET  FLEX_CURRENT_2A
#define FLEX_CURRENT_DRAG   FLEX_CURRENT_2A*/
//150402
#define WAVE_LEVEL0       0
#define WAVE_LEVEL1   80// 80//   80//90                          //40%
#define WAVE_LEVEL2    105// 105//  105//110                          //50%
#define WAVE_LEVEL3       132





#define FLEX_MOTOR_Timer_CCInit     \
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

#define FLEX_MOTOR_Timer_Init       \
{                                   \
    true,                           \
    true,                           \
    FLEX_MOTOR_PRESCALE,            \
    timerClkSelHFPerClk,            \
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
  FLEX_MOTOR_CURRENT_HIGH,
  FLEX_MOTOR_CURRENT_LOW
};
enum
{
  FLEX_MOTOR_NORMAL,
  FLEX_MOTOR_FAIL
};

void Waver_FlexMotor_Reset_Cancel(void);
void Waver_FlexMotor_Reset(void);

int Waver_FlexPower_On(unsigned char speed);
void Waver_FlexPower_Off(void);
unsigned int Waver_FlexRelay_Get(void);
unsigned int Waver_FlexPower_Get(void);
void Waver_FlexMotor_Initial_IO(void);
void Waver_FlexMotor_Break(void);

int Waver_FlexMotor_Get_Fault(void);
void Waver_FlexMotor_Out(void);    //电动小腿向外延伸，不带电源
void Waver_FlexMotor_In(void);     //电动小腿向里收缩，不带电源
//unsigned char FlexMotor_Control(unsigned char nFinalFlexPadMotorState,unsigned char speed,unsigned char current);
//void FlexMotorFollowingFood(void);
//void FlexMotorSetEnable(void);
//void FlexMotorSetDisable(void);
//int FlexMotorGetEnable(void);
void Waver_FlexMotor_10ms_Int(void);
void Waveringly_Set_Pwm_Data(unsigned int ulDuty);
unsigned int WaveMotor_IsRun(void);


unsigned int WaverMotor_VoltageAdj(unsigned int setDuty);


void Waveringly_Set_Pwm_TEST_Data(unsigned int ulDuty);


#endif /*__FLEXPAD_H__*/
