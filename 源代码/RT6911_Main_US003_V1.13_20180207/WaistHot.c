#include "efm32.h"
#include "em_chip.h"
#include "em_gpio.h"
#include "em_timer.h"
#include "WaistHot.h"
#include "ADC_Scan.h"
#define HEAT_SET_VOLTAGE 2600
unsigned int heatOnTime = 8;
unsigned int heatCounter;  
void WaistHeat_Initial_IO(void)
{
    GPIO_PinModeSet(WAIST_HEAT_PORT, WAIST_HEAT_BIT, WAIST_HEAT_MODE, 1);   
}

void WaistHeat_100ms_Int(void)//100ms timer
{
  heatCounter++;
}

unsigned int WaistHeat_VoltageAdj(unsigned int setDuty)
{
  unsigned short adc24;      //�˴��ĵ�ѹֵ�Ѿ�������100��
  ADC_Get_Voltage(ADC_V24,&adc24);     
  if(adc24 <= HEAT_SET_VOLTAGE) 
  {
    return setDuty;        //��ѹֵƫ�ͣ�����Ԥ��ֵ
  }
  unsigned int scale = HEAT_SET_VOLTAGE * 100 / adc24; //�������趨��ѹ�ı���ֵ,�ٷ�֮scale
  setDuty *= scale;
  unsigned int yushu = setDuty  % 100;
  setDuty /= 100;   //setDuty���԰ٷ�֮scale
  if(yushu > 50) setDuty++;
  
  if(setDuty >= 10)
  {
    setDuty = 10; 
  }
  return setDuty; 
  
}

void WaistHeat_On(void)
{
  unsigned int timeCounter;
  timeCounter = heatCounter;
  if(timeCounter < heatOnTime)
   {
    GPIO_PinOutClear(WAIST_HEAT_PORT, WAIST_HEAT_BIT);    
   }
   else if(timeCounter < 10)
   {
    GPIO_PinOutSet(WAIST_HEAT_PORT, WAIST_HEAT_BIT);    
   }
   else 
   {
     heatOnTime = WaistHeat_VoltageAdj(10);
    heatCounter = 0;
   }
}

void WaistHeat_Off(void)
{
    GPIO_PinOutSet(WAIST_HEAT_PORT, WAIST_HEAT_BIT);   
}