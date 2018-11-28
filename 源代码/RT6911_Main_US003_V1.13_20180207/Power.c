#include "efm32.h"
#include "em_chip.h"
#include "em_gpio.h"
#include "power.h"

void Power_Initial_IO(void)
{
  GPIO_PinModeSet(POWER_GENERAL_PORT, POWER_GENERAL_BIT, POWER_GENERAL_MODE, GENERAL_POWER_OFF); 
  GPIO_PinModeSet(POWER_3V3_PORT, POWER_3V3_BIT, POWER_3V3_MODE, GENERAL_3V3_ON); 
  GPIO_PinModeSet(POWER_5V_PORT, POWER_5V_BIT, POWER_5V_MODE, GENERAL_5V_ON); 
}

unsigned int Power_Get(void)
{
  return(GPIO_PinOutGet(POWER_GENERAL_PORT,POWER_GENERAL_BIT));
}

void Power_3V3_On(void)
{
  GPIO_PinOutSet(POWER_3V3_PORT,POWER_3V3_BIT);
}
void Power_3V3_Off(void)
{
  GPIO_PinOutClear(POWER_3V3_PORT,POWER_3V3_BIT);
}    
void Power_On(void)
{
  GPIO_PinOutSet(POWER_GENERAL_PORT,POWER_GENERAL_BIT);
}
void Power_Off(void)
{
  GPIO_PinOutClear(POWER_GENERAL_PORT,POWER_GENERAL_BIT);
}    
void Power_5V_On(void)
{
  GPIO_PinOutSet(POWER_5V_PORT,POWER_5V_BIT);
}
void Power_5V_Off(void)
{
  GPIO_PinOutClear(POWER_5V_PORT,POWER_5V_BIT);
}

void Power_All_Off(void)
{
  Power_Off();   //power ctrl 24V变压器电源
  Power_3V3_Off();  //3.3V  ldo
  Power_5V_Off();   //5V LDO VCC_IO电源开关
// VoiceUart_PowerOff();  //语音开关，TX3，RX3 ,8302T不用， modify by taoqingsong
}

void Power_All_On(void)
{
  Power_On();
  Power_3V3_On();
  Power_5V_On();
}
