#include "efm32.h"
#include "em_chip.h"
#include "em_gpio.h"
#include "em_cmu.h"
#include "em_timer.h"
#include "Roller_Uart.h"

static unsigned char nRollerSpeed;
static unsigned char nRollerMode;

void Roller_SetSpeed(unsigned char rollerSpeed)
{
   nRollerSpeed = rollerSpeed;
   
}
unsigned char Roller_GetSpeed(void)
{
    return nRollerSpeed;
}

void Roller_SetMode(unsigned int mode)
{
    nRollerMode = mode;
}
unsigned char Roller_GetMode(void)
{
    return(nRollerMode);
}
void RollerMotor_Control(unsigned int speed,unsigned int phase)
{
  Roller_SetSpeed(speed);
  if(phase)
  {
    Roller_SetMode(ROLLER_MODE_CON_IN);//滚轮运行模式，向里，向外，
  }
  else
  {
   Roller_SetMode(ROLLER_MODE_CON_OUT);
  }
}
