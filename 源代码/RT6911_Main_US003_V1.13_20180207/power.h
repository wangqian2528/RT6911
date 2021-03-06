#ifndef __POWER_H__
#define __POWER_H__

//电源总开关
#define POWER_GENERAL_PORT      gpioPortD
#define POWER_GENERAL_BIT       10
#define POWER_GENERAL_MODE      gpioModePushPull    //24V 变压器输入
//3.3V LDO 电源使能
#define POWER_3V3_PORT          gpioPortC
#define POWER_3V3_BIT           11
#define POWER_3V3_MODE          gpioModePushPull
//AMP 电源使能
#define POWER_5V_PORT           gpioPortC
#define POWER_5V_BIT            10
#define POWER_5V_MODE           gpioModePushPull

#define GENERAL_POWER_ON   1
#define GENERAL_POWER_OFF  0
#define GENERAL_3V3_ON   1
#define GENERAL_3V3_OFF  0
#define GENERAL_5V_ON   1
#define GENERAL_5V_OFF  0

void Power_Initial_IO(void);
/*;
void Power_On(void);
void Power_Off(void);
void Power_3V3_On(void);
void Power_3V3_Off(void);
*/
unsigned int Power_Get(void);
void Power_All_Off(void);
void Power_All_On(void);

void Power_On(void);




#endif /*__POWER_H__*/
