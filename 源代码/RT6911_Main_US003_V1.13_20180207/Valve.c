#include "efm32_types.h"
#include "efm32_def.h"
#include "efm32.h"
#include "em_chip.h"
#include "em_gpio.h"
#include "em_cmu.h"
#include "em_usart.h"
#include "LegKnead_Uart.h"
#include "Roller_Uart.h"
#include "Flex_Uart.h"
#include "Valve.h"
#include "Input.h"
#include "UartLeg.h"
#define VALVE_DATA_LENGTH   3// 2  modify by taoqingsong
bool bRollerDisplay;
unsigned char ucSendData[VALVE_DATA_LENGTH];
unsigned char ucReceiveData[VALVE_DATA_LENGTH];
//unsigned char* pValveData = ucSendData;
unsigned char* pInputData = ucReceiveData;
unsigned int w_RollerCounter,bSholderEnable;
unsigned int SholderTime = 0;
static bool bAutoRoller;
unsigned int nRoller3sCnt,nRoller3sCnt2n;
//手动模式PWM设置
__no_init unsigned int w_RollerPWM;
//手动模式 速度和方式
__no_init unsigned char LegKneadSpeed,LegKneadMode;
static bool bValveFlag,bRollerFlag,bLegKneadFlag,bRoller3sFlag;
extern StretchStruct st_Stretch;
__no_init static unsigned char nKeyAirBagStrength;
//74595串并转换(4片74595)
BITS BITS_ValveData[3];//BITS_ValveData[2]=存储小腿控制气阀,发送到小腿控制板


//BITS_ValveData[0]=1-8位气阀，BITS_ValveData[1]=9-16位气阀，BITS_ValveData[2]=17-24位气阀，

bool bBackauto;

void Valve_Initial_IO(void)//8600S使用2个气帮
{
    USART_TypeDef *spi = VALVE_SPI;
    USART_InitSync_TypeDef InitSync_Init = VALVE_USART_INITSYNC;
    
    /* Clearing old transfers/receptions, and disabling interrupts */
    //spi->CMD = USART_CMD_CLEARRX | USART_CMD_CLEARTX;
    //spi->IEN = 0;
    
    USART_InitSync(spi,&InitSync_Init);
    
    USART0->ROUTE = (USART0->ROUTE & ~_USART_ROUTE_LOCATION_MASK) | USART_ROUTE_LOCATION_LOC1;
    /* Enabling pins and setting location */
    spi->ROUTE |= USART_ROUTE_RXPEN | USART_ROUTE_TXPEN | USART_ROUTE_CLKPEN;
    
    /* Clear previous interrupts */
    //spi->IFC = _USART_IFC_MASK;
    
    
    
    /*
    USART_TypeDef *spi = VALVE_SPI;
    USART_InitSync_TypeDef InitSync_Init = VALVE_USART_INITSYNC;
    
    spi->CMD   = USART_CMD_MASTEREN | USART_CMD_TXEN ;
    spi->CTRL |= USART_CTRL_AUTOCS;
    
    spi->IEN = 0;
    
    spi->ROUTE = USART_ROUTE_TXPEN | USART_ROUTE_CLKPEN | VALVE_SPI_ROUTE_LOCAITON;
    
    spi->IFC = _USART_IFC_MASK;
    
    USART_InitSync(spi,&InitSync_Init);
    
    IO configuration */
    // GPIO_PinModeSet(VALVE_POWER_PORT,VALVE_POWER_BIT,VALVE_POWER_MODE,0);
    GPIO_PinModeSet(VALVE_LOAD_PORT,VALVE_LOAD_BIT,VALVE_LOAD_MODE,0);
    GPIO_PinModeSet(VALVE_CLK_PORT,VALVE_CLK_BIT,VALVE_CLK_MODE,0);
    GPIO_PinModeSet(VALVE_LATCH_PORT,VALVE_LATCH_BIT,VALVE_LATCH_MODE,1);
    GPIO_PinModeSet(VALVE_DATA_PORT,VALVE_DATA_BIT,VALVE_DATA_MODE,1);
    GPIO_PinModeSet(VALVE_DATA_IN_PORT,VALVE_DATA_IN_BIT,VALVE_DATA_IN_MODE,1);
    
    GPIO_PinModeSet(VALVE_AIRPUMP1_PORT,VALVE_AIRPUMP1_BIT,VALVE_AIRPUMP1_MODE,0);
    GPIO_PinModeSet(VALVE_AIRPUMP2_PORT,VALVE_AIRPUMP2_BIT,VALVE_AIRPUMP2_MODE,0);
    GPIO_PinModeSet(VALVE_AIRPUMP3_PORT,VALVE_AIRPUMP3_BIT,VALVE_AIRPUMP3_MODE,0);
    
    GPIO_PinModeSet(VALVE_OZON_PORT,VALVE_OZON_BIT,VALVE_OZON_MODE,0);
    
//    Power_All_On();
    BITS_ValveData[0].nByte = 0;   //OUT1=BIT0 ,OUT2=BIT1 ,OUT3=BIT2,OUT4=BIT3.OUT5=BIT4,OUT6=BIT5.OUT7=BIT6,OUT8=BIT7
    BITS_ValveData[1].nByte = 0;//SPI先发送BIT7，
    BITS_ValveData[2].nByte = 0;
}

void Valve_Initial_Data(void)
{
    w_RollerPWM = ROLLER_SPEED_STOP;
    LegKneadSpeed = LEG_KNEAD_SPEED_STOP;
    nKeyAirBagStrength = 0;//软件初始化时气囊力度为0
}


void Valve_SetData(void)
{
    GPIO_PinOutSet(VALVE_DATA_PORT,VALVE_DATA_BIT);
}
void Valve_ClearData(void)
{
    GPIO_PinOutClear(VALVE_DATA_PORT,VALVE_DATA_BIT);
}

void Valve_SetClock(void)
{
    GPIO_PinOutSet(VALVE_CLK_PORT,VALVE_CLK_BIT);
}
void Valve_ClearClock(void)
{
    GPIO_PinOutClear(VALVE_CLK_PORT,VALVE_CLK_BIT);
}

void Valve_ClearLatch(void)
{
    GPIO_PinOutClear(VALVE_LATCH_PORT,VALVE_LATCH_BIT);
}
void Valve_SetLatch(void)
{
    GPIO_PinOutSet(VALVE_LATCH_PORT,VALVE_LATCH_BIT);
}

void Valve_5ms_Int(void)//system中是5ms timer
{
    bValveFlag = true;
    bRollerFlag = true;
    bLegKneadFlag = true;
    bRoller3sFlag =true;
    w_RollerCounter++;
    SholderTime++;
}
void Valve_1ms_Int(void)
{
    //bValveFlag = true;
}

static unsigned char SPI_FlashWrite(unsigned char data)
{
    VALVE_SPI->TXDATA = data;
    while (!(VALVE_SPI->STATUS & USART_STATUS_TXC))
    {
    }
    return (uint8_t)(VALVE_SPI->RXDATA);
}

/*本程序10ms执行一次，放在主程序中，数据长度固定为4*/
void Valve_Send_Data(void)
{
    unsigned int i;
    unsigned char ucLength = VALVE_DATA_LENGTH;
    
    if(!bValveFlag) return;
    bValveFlag = false;
    
    for(int i = 0; i < 2; i++)
    {
        *(ucSendData + i) = BITS_ValveData[i].nByte;
    }
    
    GPIO_PinOutSet(VALVE_LOAD_PORT,VALVE_LOAD_BIT);////当该引脚为低电平时，并行数据被置入寄存器，当为高电平时，寄存器数据保持不变
    
    
    for(i = 0;i < 2;i++)
    {
        *(ucReceiveData + i) = SPI_FlashWrite(*(ucSendData + i));
    }
    
    for(i = 100;i > 0;i--) __no_operation();
    
    GPIO_PinOutSet(VALVE_LATCH_PORT,VALVE_LATCH_BIT);////高电平锁存数据，经过3*8=24个时钟周期后锁存数据,将数据输出到OUT引脚上
    
    for( i = 100;i > 0;i--) __no_operation();
    
    GPIO_PinOutClear(VALVE_LATCH_PORT,VALVE_LATCH_BIT);////clr 锁存
    
    GPIO_PinOutClear(VALVE_LOAD_PORT,VALVE_LOAD_BIT);
}




/*void LegFootAirBagAction(bool Enable,unsigned int action)//8600S
{    


 
    if((Flex_GetDirection() == FLEX_TO_OUT) || (Flex_GetDirection() == FLEX_TO_IN))
    {
     Enable = FALSE; 
    }
  
    if(!Enable)
    {
        bFootHeelAirBagValve = VALVE_OFF;
        bRightFootAirBagValve = VALVE_OFF;
        bLeftFootAirBagValve = VALVE_OFF;
        bLegLeftAirBagValve = VALVE_OFF;		        
        bLegRightAirBagValve = VALVE_OFF;  	        
        return;
    }
   
    if(action & LEG_LEFT)
    {
        bLegRightAirBagValve = VALVE_ON ;
    }
    else
    {
        bLegRightAirBagValve = VALVE_OFF ;
    }
    
    
    if(action & LEG_RIGHT)
    {
        bLegLeftAirBagValve = VALVE_ON ;
    }
    else
    {
        bLegLeftAirBagValve = VALVE_OFF ;
    }
    
    if(action & F_L_SIDE)
    {
        bLeftFootAirBagValve = VALVE_ON ;
    }
    else
    {
        bLeftFootAirBagValve = VALVE_OFF ;
    }
    if(action & F_HEEL)
    {
        bFootHeelAirBagValve = VALVE_ON ;
    }
    else
    {
        bFootHeelAirBagValve = VALVE_OFF ;
    }
    if(action & F_R_SIDE)
    {
        bRightFootAirBagValve = VALVE_ON ;
    }
    else
    {
        bRightFootAirBagValve = VALVE_OFF ;
    }
    
    

}*/

//--------------------------------------------------

void LegFootAirBagAction(bool Enable,unsigned int action)//modify by taoqingosng
{    
    if((Flex_GetDirection() == FLEX_TO_OUT) || (Flex_GetDirection() == FLEX_TO_IN))
    {
     Enable = FALSE; 
    }

  
    if(!Enable)
    {
    bFootHeelAirBagValve = VALVE_OFF;
    bRightFootAirBagValve = VALVE_OFF;
    bLeftFootAirBagValve = VALVE_OFF;
    bLegDownBottomAirBagValve = VALVE_OFF;
    bLegDownSideAirBagValve = VALVE_OFF; 
    bLegUpBottomAirBagValve = VALVE_OFF;
    bLegUpSideAirBagValve = VALVE_OFF;

    return;
    }
    else
    {
      

      if(action & LEG0_CHR)
      {
        bLegUpBottomAirBagValve = VALVE_ON ;
      }
      else
      {
        bLegUpBottomAirBagValve = VALVE_OFF ;
      }
      if(action & LEG1_CHR)
      {
        bLegUpSideAirBagValve = VALVE_ON ;
      }
      else
      {
        bLegUpSideAirBagValve = VALVE_OFF ;
      }
      
      if(action & LEG2_CHR)
      {
        bLegDownBottomAirBagValve = VALVE_ON ;
      }
      else
      {
        bLegDownBottomAirBagValve = VALVE_OFF ;
      }
      
      if(action & LEG3_CHR)
      {
        bLegDownSideAirBagValve = VALVE_ON ;
      }
      else
      {
        bLegDownSideAirBagValve = VALVE_OFF ;
      }
      
      if(action & FOOT0_CHR)
      {
        bLeftFootAirBagValve = VALVE_ON ;
      }
      else
      {
        bLeftFootAirBagValve = VALVE_OFF ;
      }
      if(action & FOOT1_CHR)
      {
        bFootHeelAirBagValve = VALVE_ON ;
      }
      else
      {
        bFootHeelAirBagValve = VALVE_OFF ;
      }
      if(action & FOOT2_CHR)
      {
        bRightFootAirBagValve = VALVE_ON ;
      }
      else
      {
        bRightFootAirBagValve = VALVE_OFF ;
      }
      
      
      
    }
}

void KneckAirBagAction(bool Enable,unsigned int action)
{
  
  /*  if(!Enable)
    {

      bLeftNeckAirBagValve		=  VALVE_OFF ;
      bRightNeckAirBagValve 	=  VALVE_OFF ;

     return;
    }
    else
    {
      
      if((action & LEFT_KNECK_CHR))
      {
        bLeftNeckAirBagValve = VALVE_ON ;
      }
      else
      {
        bLeftNeckAirBagValve = VALVE_OFF ;
      }
      
      if((action & RIGHT_KNECK_CHR) )
      {
        bRightNeckAirBagValve = VALVE_ON ;
      }
      else
      {
        bRightNeckAirBagValve = VALVE_OFF ;
      }
    } */
  
  
}



void BodyUpAirBagAction(bool Enable,unsigned int action)//上半身气囊  modify by  taoqingsong
{    
    if(!Enable)
    {
     //bButtocksAirBagValve	      =  VALVE_OFF ;
     //bLeftWaistAirBagValve        =  VALVE_OFF ;
     //bRightWaistAirBagValve       =  VALVE_OFF ;  
     bLeftThighAirBagValve   	  =  VALVE_OFF ;
     bRightThighAirBagValve   	  =  VALVE_OFF ;
     //bLeftNeckAirBagValve		=  VALVE_OFF ;
      //bRightNeckAirBagValve 	=  VALVE_OFF ;

     return;
    }
    else
    {

      if(action & LEFT_THIGH_CHR)
      {
          bLeftThighAirBagValve = VALVE_ON ;
      }
      else
      {
         bLeftThighAirBagValve = VALVE_OFF ;
      }
      if(action & RIGHT_THIGH_CHR)
      {
          bRightThighAirBagValve = VALVE_ON ;
      }
      else
      {
         bRightThighAirBagValve = VALVE_OFF ;
      }

      
    }
}


void ArmNeckAirBagAction(bool Enable,unsigned int action,unsigned char locate)//modify by taoqingsong
{    
    if(!Enable)
    {
      //bSholderAirBagValve       =  VALVE_OFF ;
           bRightSholderAirBagValve = VALVE_OFF ;  	
           bLeftSholderAirBagValve = VALVE_OFF ;
      bLeftArmUpAirBagVave 		=  VALVE_OFF ;
      bLeftArmDownAirBagValve	=  VALVE_OFF ;
      bRightArmUpAirBagValve	=  VALVE_OFF ;
      bRightArmDownAirBagValve  =  VALVE_OFF ;
      return;
    }
    else
    {

      if(action & RIGHT_ARM_UP_CHR)
      {
        bRightArmUpAirBagValve = VALVE_ON ;
      }
      else
      {
        bRightArmUpAirBagValve = VALVE_OFF ;
      }
      if(action & RIGHT_ARM_DOWN_CHR)
      {
        bRightArmDownAirBagValve = VALVE_ON ;
      }
      else
      {
        bRightArmDownAirBagValve = VALVE_OFF ;
      }
       if(action & LEFT_ARM_UP_CHR)
      {
        bLeftArmUpAirBagVave = VALVE_ON ;
      }
      else
      {
        bLeftArmUpAirBagVave = VALVE_OFF ;
      }
      if(action & LEFT_ARM_DOWN_CHR)
      {
        bLeftArmDownAirBagValve = VALVE_ON ;
      }
      else
      {
        bLeftArmDownAirBagValve = VALVE_OFF ;
      }
      // if(!bSholderEnable)
    if(action & LEFT_SHOULDER_CHR)
    {
        bLeftSholderAirBagValve = VALVE_ON ;
    }
    else
    {
        bLeftSholderAirBagValve = VALVE_OFF ;
    }
    if(action & RIGHT_SHOULDER_CHR)
    {
        bRightSholderAirBagValve = VALVE_ON ;
    }
    else
    {
        bRightSholderAirBagValve = VALVE_OFF ;
    }
      
    /*  if(bBackauto)
       {
        // if((action & SHOULDER_CHR) && (Input_GetWalkMotorPosition() <70))// 40 ))//bBackauto比肩位最低点低 时可以打开气囊
         if((action & SHOULDER_CHR) && (Input_GetWalkMotorPosition() <40))// 40 ))//bBackauto比肩位最低点低 时可以打开气囊
         {
           bRightSholderAirBagValve = VALVE_ON ;  	
           bLeftSholderAirBagValve = VALVE_ON ; 
         }
         else
         {
           bRightSholderAirBagValve = VALVE_OFF ;  	
           bLeftSholderAirBagValve = VALVE_OFF ; 
           
         }
       }
      else
      {
        if(action & SHOULDER_CHR)
        {
          //bSholderAirBagValve = VALVE_ON ;
           bRightSholderAirBagValve = VALVE_ON ;  	
           bLeftSholderAirBagValve = VALVE_ON ;
        }
        else
        {
          //bSholderAirBagValve = VALVE_OFF ;
           bRightSholderAirBagValve = VALVE_OFF ;  	
           bLeftSholderAirBagValve = VALVE_OFF ;
        } 
      }*/
    }
}


//------------------------------------------------------------------

void SeatAirBagAction(bool Enable,unsigned int action)//no use by taoqingosng
{
  
    if(!Enable)
    {
     //bButtocksAirBagValve	      =  VALVE_OFF ;
     bLeftThighAirBagValve   	  =  VALVE_OFF ;
     bRightThighAirBagValve   	  =  VALVE_OFF ;
     return;
    }
    else
    {

      if(action & LEFT_THIGH_CHR)
      {
          bLeftThighAirBagValve = VALVE_ON ;
      }
      else
      {
         bLeftThighAirBagValve = VALVE_OFF ;
      }
      if(action & RIGHT_THIGH_CHR)
      {
          bRightThighAirBagValve = VALVE_ON ;
      }
      else
      {
         bRightThighAirBagValve = VALVE_OFF ;
      }

    }

}

void ArmSholderAirBagAction(bool Enable,unsigned int action)//no use by taoqingosng
{    
  /*  if(!Enable)
    {
        bRightArmUpAirBagValve1   =  VALVE_OFF ;
        bRightArmUpAirBagValve2   =  VALVE_OFF ;
        bRightArmUpAirBagValve3   =  VALVE_OFF ;
        bLeftArmUpAirBagValve1    =  VALVE_OFF ;
        bLeftArmUpAirBagValve2    =  VALVE_OFF ;
        bLeftArmUpAirBagValve3    =  VALVE_OFF ;
        bLeftSholderAirBagValve   =  VALVE_OFF ;
        bRightSholderAirBagValve  =  VALVE_OFF ;
        return;
    }
    
    unsigned short nCurWalkMotorLocate = Input_GetWalkMotorPosition();

    if(action & R_ARM_1)
    {
        bRightArmUpAirBagValve1 = VALVE_ON ;
    }
    else
    {
        bRightArmUpAirBagValve1 = VALVE_OFF ;
    }
    if(action & R_ARM_2)
    {
        bRightArmUpAirBagValve2 = VALVE_ON ;
    }
    else
    {
        bRightArmUpAirBagValve2 = VALVE_OFF ;
    }
    if(action & R_ARM_3)
    {
        bRightArmUpAirBagValve3 = VALVE_ON ;
    }
    else
    {
        bRightArmUpAirBagValve3 = VALVE_OFF ;
    }
    if(action & L_ARM_1)
    {
        bLeftArmUpAirBagValve1 = VALVE_ON ;
    }
    else
    {
        bLeftArmUpAirBagValve1 = VALVE_OFF ;
    }
    if(action & L_ARM_2)
    {
        bLeftArmUpAirBagValve2 = VALVE_ON ;
    }
    else
    {
        bLeftArmUpAirBagValve2 = VALVE_OFF ;
    }
    if(action & L_ARM_3)
    {
        bLeftArmUpAirBagValve3 = VALVE_ON ;
    }
    else
    {
        bLeftArmUpAirBagValve3 = VALVE_OFF ;
    }
    if(nCurWalkMotorLocate > 230 && bBackauto)  //机芯超过这个高度不允许动作
    {
        bLeftSholderAirBagValve = VALVE_OFF ;
        bRightSholderAirBagValve = VALVE_OFF ;
        return;
    }
    if(action & L_SHOLDER)
    {
        bLeftSholderAirBagValve = VALVE_ON ;
    }
    else
    {
        bLeftSholderAirBagValve = VALVE_OFF ;
    }
    if(action & R_SHOLDER)
    {
        bRightSholderAirBagValve = VALVE_ON ;
    }
    else
    {
        bRightSholderAirBagValve = VALVE_OFF ;
    }
    */
}

void ArmAirBagAction(bool Enable,unsigned int action)//no use  by taoqingsong
{    
 /*   if(!Enable)
    {
        bRightArmUpAirBagValve1   =  VALVE_OFF ;
        bRightArmUpAirBagValve2   =  VALVE_OFF ;
        bRightArmUpAirBagValve3   =  VALVE_OFF ;
        bLeftArmUpAirBagValve1    =  VALVE_OFF ;
        bLeftArmUpAirBagValve2    =  VALVE_OFF ;
        bLeftArmUpAirBagValve3    =  VALVE_OFF ;
        return;
    }
    if(action & R_ARM_1)
    {
        bRightArmUpAirBagValve1 = VALVE_ON ;
    }
    else
    {
        bRightArmUpAirBagValve1 = VALVE_OFF ;
    }
    if(action & R_ARM_2)
    {
        bRightArmUpAirBagValve2 = VALVE_ON ;
    }
    else
    {
        bRightArmUpAirBagValve2 = VALVE_OFF ;
    }
    if(action & R_ARM_3)
    {
        bRightArmUpAirBagValve3 = VALVE_ON ;
    }
    else
    {
        bRightArmUpAirBagValve3 = VALVE_OFF ;
    }
    if(action & L_ARM_1)
    {
        bLeftArmUpAirBagValve1 = VALVE_ON ;
    }
    else
    {
        bLeftArmUpAirBagValve1 = VALVE_OFF ;
    }
    if(action & L_ARM_2)
    {
        bLeftArmUpAirBagValve2 = VALVE_ON ;
    }
    else
    {
        bLeftArmUpAirBagValve2 = VALVE_OFF ;
    }
    if(action & L_ARM_3)
    {
        bLeftArmUpAirBagValve3 = VALVE_ON ;
    }
    else
    {
        bLeftArmUpAirBagValve3 = VALVE_OFF ;
    }*/
}

//每一种气囊力度对应一种保持时间，保持时间越长，气囊力度越大
// pBag->nCurKeepTime2
bool AirBagGetNextStep(st_AirBag* pBag)  //bNextStep是真或假，即返回是否执行下一步骤
{
    bool bNextStep = FALSE;
    unsigned char counter = pBag->nAirBagCounter;//当前气囊保持时间计数器，100ms timer counter
    switch(nKeyAirBagStrength)
    {
    case 1:   //气囊力度1对应的保持时间
        if(counter > pBag->nCurKeepTime1)  
        {
            bNextStep = TRUE ;
        }
        break ;
    case 2:////气囊力度2对应的保持时间
        if(counter > pBag->nCurKeepTime2)
        {
            bNextStep = TRUE ;
        }
        break ;
    case 3://气囊力度3对应的保持时间
        if(counter > pBag->nCurKeepTime3)
        {
            bNextStep = TRUE ;
        }
        break ;
    case 4://气囊力度4对应的保持时间
        if(counter > pBag->nCurKeepTime4)
        {
            bNextStep = TRUE ;
        }
        break ;
    case 5://气囊力度5对应的保持时间
        if(counter > pBag->nCurKeepTime5)
        {
            bNextStep = TRUE ;
        }
        break ;
    }
    return (bNextStep);
}

void Valve_CloseAll(void)  
{
    BITS_ValveData[0].nByte = 0;
    BITS_ValveData[1].nByte = 0;
    BITS_ValveData[2].nByte = 0;
    Valve_AirPumpACPowerOff();
}



void Valve_SetStretchUp(void)//modify by taoqingsong
{
  
   Valve_AirPumpACPowerOn();
 
  bLeftThighAirBagValve  = VALVE_OFF ;
  bRightThighAirBagValve = VALVE_OFF ;
  //bRightWaistAirBagValve = VALVE_ON ;
  //bLeftWaistAirBagValve  = VALVE_ON ;
  //bButtocksAirBagValve   = VALVE_OFF;
  
  

  
  bRightFootAirBagValve = VALVE_OFF;
  bLeftFootAirBagValve = VALVE_OFF;
  bFootHeelAirBagValve = VALVE_OFF;
  
  
  bLegDownSideAirBagValve = VALVE_OFF; 
  bLegUpSideAirBagValve = VALVE_OFF;
  bLegDownBottomAirBagValve = VALVE_OFF;
  bLegUpBottomAirBagValve = VALVE_OFF;
  
  
  
//    bButtocksAirBagValve = VALVE_OFF;
    ////bSholderAirBagValve = VALVE_OFF;
           bRightSholderAirBagValve = VALVE_OFF ;  	
           bLeftSholderAirBagValve = VALVE_OFF ;
    
    
  
  bLeftArmUpAirBagVave 	=  VALVE_OFF ;
  bLeftArmDownAirBagValve	=  VALVE_OFF ;
  bRightArmDownAirBagValve	=  VALVE_OFF ;
  bRightArmUpAirBagValve =  VALVE_OFF ;
  //bLeftNeckAirBagValve= VALVE_OFF;
  //bRightNeckAirBagValve= VALVE_OFF;
  
}



void Valve_SetStretchCharge(unsigned int start)//modify by  taoqingsong
{

      Valve_AirPumpACPowerOn();//add by taongsong 2015-10-16
    //Valve_AirPumpACPowerOn();
if((Flex_GetDirection() == FLEX_TO_OUT) || (Flex_GetDirection() == FLEX_TO_IN))
{
      bLegUpSideAirBagValve = VALVE_OFF;                     
    bLegDownSideAirBagValve = VALVE_OFF;              
    //足部气囊
    bRightFootAirBagValve = VALVE_OFF;
    bLeftFootAirBagValve = VALVE_OFF;
    bFootHeelAirBagValve = VALVE_OFF;
}
else
{
      bLegUpSideAirBagValve = VALVE_ON;                     
    bLegDownSideAirBagValve = VALVE_ON;              
    //足部气囊
    bRightFootAirBagValve = VALVE_ON;
    bLeftFootAirBagValve = VALVE_ON;
    bFootHeelAirBagValve = VALVE_OFF;
  
}
    //小腿气囊
    //bLegUpSideAirBagValve = VALVE_ON;                     
    //bLegDownSideAirBagValve = VALVE_ON;              
    //足部气囊
    //bRightFootAirBagValve = VALVE_ON;
    //bLeftFootAirBagValve = VALVE_ON;
    //bFootHeelAirBagValve = VALVE_OFF;
    //大腿气囊
    bLeftThighAirBagValve       =  VALVE_ON ;
    bRightThighAirBagValve 	=  VALVE_ON ;  
    //胳膊气囊
    bLeftSholderAirBagValve   =  VALVE_ON ;
    bRightSholderAirBagValve  =  VALVE_ON ;
    //背腰气囊
    
    bLeftArmUpAirBagVave   =  VALVE_OFF ; 	 
    bLeftArmDownAirBagValve    =  VALVE_OFF ;  	  
    bRightArmUpAirBagValve    =  VALVE_OFF ;        
    bRightArmDownAirBagValve   =  VALVE_OFF ;  
/*  
  //----------------------------------------------------------
  static int step = 0;
  if(start)
  {
   step =  0; 
   SholderTime = 0;
  }
  switch(step)
  {
   case 0: //bSholderAirBagValve  = VALVE_ON ; //充气2。5sec
           bRightSholderAirBagValve = VALVE_ON ;  	
           bLeftSholderAirBagValve = VALVE_ON ;
           SholderTime = 0;
           step++;
           break;
   case 1: if(SholderTime > 500)//500*5ms=2.5sec=肩部充气2.5sec    
           step++;
           break;       
   case 2: //bSholderAirBagValve  =VALVE_ON;//VALVE_OFF ; //放气1sec
           bRightSholderAirBagValve = VALVE_ON ;  	
           bLeftSholderAirBagValve = VALVE_ON ;
           step++;
           SholderTime = 0;
           break;       
   case 3: if(SholderTime > 200)//肩部关闭放气200*5ms=1sec
           step = 0;
           break;               
           
  }
//-----------------------------------------

  bLeftArmUpAirBagVave=VALVE_OFF;//add  2015-10-16
  bRightArmUpAirBagValve=VALVE_OFF;
  //bLeftNeckAirBagValve=VALVE_OFF;
  //bRightNeckAirBagValve=VALVE_OFF;
  bLeftArmDownAirBagValve=VALVE_OFF;
  bRightArmDownAirBagValve=VALVE_OFF;  
  
 //---------------------------------------- 
  
  
  
  bRightFootAirBagValve = VALVE_ON;
  bLeftFootAirBagValve = VALVE_ON;
  bLegDownSideAirBagValve =VALVE_ON;//VALVE_OFF; //VALVE_ON; 
  bLegUpSideAirBagValve =VALVE_ON;//VALVE_OFF;// VALVE_ON;
  //bRightWaistAirBagValve =VALVE_ON;// VALVE_OFF;//VALVE_ON ;
  //bLeftWaistAirBagValve = VALVE_ON;//VALVE_OFF;//VALVE_ON ;
  
  
  
  //bButtocksAirBagValve = VALVE_OFF;
  bLeftThighAirBagValve = VALVE_OFF;
  bRightThighAirBagValve = VALVE_OFF; 
  bFootHeelAirBagValve = VALVE_OFF;
  bLegDownBottomAirBagValve = VALVE_OFF;
  bLegUpBottomAirBagValve = VALVE_OFF;*/
}







/*void Valve_SetStretchChargeOut(unsigned int start)//8600s
{
  Valve_SetStretchCharge(start);
   //小腿气囊
    bLegLeftAirBagValve = VALVE_OFF;
    bLegRightAirBagValve = VALVE_OFF;
    

}*/
void  Valve_SetStretchChargeATOUT(unsigned int start)
{
   Valve_AirPumpACPowerOn();
   
  //----------------------------------------------------------
  static int step = 0;
/*  if(start)
  {
   step =  0; 
   SholderTime = 0;
  }
  switch(step)
  {
   case 0: //bSholderAirBagValve  = VALVE_OFF ; //充气2。5sec
           bRightSholderAirBagValve = VALVE_ON ;  	
           bLeftSholderAirBagValve = VALVE_ON ;
           SholderTime = 0;
           step++;
           break;
   case 1: if(SholderTime > 500)//500*5ms=2.5sec=肩部充气2.5sec    
           step++;
           break;       
   case 2: //bSholderAirBagValve  =VALVE_OFF;//VALVE_OFF ; //放气1sec
           bRightSholderAirBagValve = VALVE_OFF ;  	
           bLeftSholderAirBagValve = VALVE_OFF ;
           step++;
           SholderTime = 0;
           break;       
   case 3: if(SholderTime > 200)//肩部关闭放气200*5ms=1sec
           step = 0;
           break;               
           
  }*/
  
           bRightSholderAirBagValve = VALVE_ON ;  	
           bLeftSholderAirBagValve = VALVE_ON ;
//-----------------------------------------

  bLeftArmUpAirBagVave=VALVE_OFF;//add  2015-10-16
  bRightArmUpAirBagValve=VALVE_OFF;
  //bLeftNeckAirBagValve=VALVE_OFF;
  //bRightNeckAirBagValve=VALVE_OFF;
  bLeftArmDownAirBagValve=VALVE_OFF;
  bRightArmDownAirBagValve=VALVE_OFF;  
  
 //---------------------------------------- 
  
  
  
  bRightFootAirBagValve = VALVE_ON;
  bLeftFootAirBagValve = VALVE_ON;
  bFootHeelAirBagValve = VALVE_ON;
  bLegDownSideAirBagValve =VALVE_OFF; //VALVE_ON; 
  bLegUpSideAirBagValve =VALVE_OFF;// VALVE_ON;
  //bRightWaistAirBagValve = VALVE_OFF;//VALVE_ON ;
  //bLeftWaistAirBagValve = VALVE_OFF;//VALVE_ON ;
  
  
  
  //bButtocksAirBagValve = VALVE_ON;//VALVE_OFF;
  bLeftThighAirBagValve = VALVE_ON;
  bRightThighAirBagValve = VALVE_ON; 
  
  
  bFootHeelAirBagValve = VALVE_OFF;
  bLegDownBottomAirBagValve = VALVE_OFF;
  bLegUpBottomAirBagValve = VALVE_OFF;
  
  
  
}
void Valve_LeftHandSetStretchChargeOut(unsigned int start)//modify by taoqingosng
{
  
Valve_AirPumpACPowerOn();
  
           //bRightSholderAirBagValve = VALVE_OFF ;  	
           ///bLeftSholderAirBagValve = VALVE_OFF ;
  static int step = 0;
  if(start)
  {
    step =  0; 
    SholderTime = 0;
  }
  switch(step)
  {
  case 0:   
    bRightSholderAirBagValve = VALVE_OFF ;  	
           bLeftSholderAirBagValve = VALVE_ON ;
  SholderTime = 0;
  step++;
  break;
  case 1:// if(SholderTime >2000)// 500)//500*5ms=2.5sec=肩部充气2.5sec	  
    if(SholderTime >500)// 500)//500*5ms=2.5sec=肩部充气2.5sec  
      step++;
    break; 	  
  case 2:  
    bRightSholderAirBagValve = VALVE_OFF ;  	
           bLeftSholderAirBagValve = VALVE_OFF ; 
  step++;
  SholderTime = 0;
  break; 	  
  case 3: if(SholderTime > 200)//肩部关闭放气200*5ms=1sec
    step = 0;
    break; 			  
    
  }
  
  
  
  //------------------------------------------
  /*
  bLeftArmUpAirBagVave=VALVE_ON;
  bRightArmUpAirBagValve=VALVE_ON;
  bLeftArmDownAirBagValve=VALVE_ON;
  bRightArmDownAirBagValve=VALVE_ON;	
  
  
  bLeftNeckAirBagValve=VALVE_OFF;
  bRightNeckAirBagValve=VALVE_OFF;
  */
  bLeftArmUpAirBagVave=VALVE_ON; ;//add	2015-10-16
  bRightArmUpAirBagValve=VALVE_OFF;
 // bLeftNeckAirBagValve=VALVE_OFF;
 // bRightNeckAirBagValve=VALVE_OFF;
  bLeftArmDownAirBagValve=VALVE_ON; ;
  bRightArmDownAirBagValve=VALVE_OFF; 
  //----------------------------------------------
  
  
  
  bRightFootAirBagValve = VALVE_ON;
  bLeftFootAirBagValve = VALVE_ON;bFootHeelAirBagValve = VALVE_ON;
  bLegDownSideAirBagValve =VALVE_OFF;// VALVE_ON; 
  bLegUpSideAirBagValve = VALVE_OFF;//VALVE_ON;
 // bRightWaistAirBagValve =VALVE_ON;// VALVE_ON ;
 // bLeftWaistAirBagValve =VALVE_OFF;// VALVE_ON ;
  
  
  
 // bButtocksAirBagValve = VALVE_ON;
  bLeftThighAirBagValve = VALVE_ON;
  bRightThighAirBagValve = VALVE_ON; 
  bFootHeelAirBagValve = VALVE_OFF;//VALVE_ON;
  bLegDownBottomAirBagValve = VALVE_OFF;
  bLegUpBottomAirBagValve = VALVE_OFF; 
}
void Valve_RightHandSetStretchChargeOut(unsigned int start)//modify by taoqingosng
{
  
 Valve_AirPumpACPowerOn();// Valve_ArmAirPumpACPowerOn(); //	modify by taoqingosng  2015-10-15
 // bRightSholderAirBagValve = VALVE_OFF ;  	
 // bLeftSholderAirBagValve = VALVE_OFF ;
  
  static int step = 0;
  if(start)
  {
    step =  0; 
    SholderTime = 0;
  }
  switch(step)
  {
  case 0: //bSholderAirBagValve	= VALVE_ON;//VALVE_OFF ; 
    
    bRightSholderAirBagValve = VALVE_ON ;  	
    bLeftSholderAirBagValve = VALVE_OFF ;  
    SholderTime = 0;
    step++;
    break;
  case 1:// if(SholderTime >2000)// 500)//500*5ms=2.5sec=肩部充气2.5sec	
    if(SholderTime >500)// 500)//500*5ms=2.5sec=肩部充气2.5sec  
      step++;
    break;		
  case 2:// bSholderAirBagValve	=VALVE_ON;//VALVE_OFF ; 
    bRightSholderAirBagValve = VALVE_OFF ;  	
    bLeftSholderAirBagValve = VALVE_OFF ;
    
    step++;
    SholderTime = 0;
    break;		
  case 3: if(SholderTime > 200)//肩部关闭放气200*5ms=1sec
    step = 0;
    break;				
    
  }
  
  
  
  //------------------------------------------
  /*
  bLeftArmUpAirBagVave=VALVE_ON;
  bRightArmUpAirBagValve=VALVE_ON;
  bLeftArmDownAirBagValve=VALVE_ON;
  bRightArmDownAirBagValve=VALVE_ON;  
  
  
  bLeftNeckAirBagValve=VALVE_OFF;
  bRightNeckAirBagValve=VALVE_OFF;
  */
  bLeftArmUpAirBagVave=VALVE_OFF; ;//add 2015-10-16
  bRightArmUpAirBagValve=VALVE_ON;
  //bLeftNeckAirBagValve=VALVE_OFF;
  //bRightNeckAirBagValve=VALVE_OFF;
  bLeftArmDownAirBagValve=VALVE_OFF; ;
  bRightArmDownAirBagValve=VALVE_ON; 
  //----------------------------------------------
  
  
  
  bRightFootAirBagValve = VALVE_ON;
  bLeftFootAirBagValve = VALVE_ON;bFootHeelAirBagValve = VALVE_ON;
  bLegDownSideAirBagValve =VALVE_OFF;// VALVE_ON; 
  bLegUpSideAirBagValve = VALVE_OFF;//VALVE_ON;
  //bRightWaistAirBagValve =VALVE_OFF;// VALVE_ON ;
  //bLeftWaistAirBagValve =VALVE_ON;// VALVE_ON ;
  
  
  
  //bButtocksAirBagValve = VALVE_OFF;
  bLeftThighAirBagValve = VALVE_ON;
  bRightThighAirBagValve = VALVE_ON; 
  bFootHeelAirBagValve = VALVE_OFF;//VALVE_ON;
  bLegDownBottomAirBagValve = VALVE_OFF;
  bLegUpBottomAirBagValve = VALVE_OFF; 
  
  
  
  
}
void Valve_SetStretchChargeOut(unsigned int start)//modify by taoqingosng
{
    
  
   Valve_AirPumpACPowerOn();//add by taongsong 2015-10-16
   
  static int step = 0;
  if(start)
  {
   step =  0; 
   SholderTime = 0;
  }
  switch(step)
  {
   case 0: //bSholderAirBagValve  = VALVE_ON;//VALVE_OFF ;
           bRightSholderAirBagValve = VALVE_ON ;  	
           bLeftSholderAirBagValve = VALVE_ON ;
           SholderTime = 0;
           step++;
           break;
   case 1:// if(SholderTime >2000)// 500)//500*5ms=2.5sec=肩部充气2.5sec    
           if(SholderTime >2000)// 500)//500*5ms=2.5sec=肩部充气2.5sec  
           step++;
           break;       
   case 2:// bSholderAirBagValve  =VALVE_ON;//VALVE_OFF ;
           bRightSholderAirBagValve = VALVE_ON ;  	
           bLeftSholderAirBagValve = VALVE_ON ;
           step++;
           SholderTime = 0;
           break;       
   case 3: if(SholderTime > 800)//肩部关闭放气200*5ms=1sec
           step = 0;
           break;               
           
  }

  
  
  //------------------------------------------
  /*
  bLeftArmUpAirBagVave=VALVE_ON;
  bRightArmUpAirBagValve=VALVE_ON;
     bLeftArmDownAirBagValve=VALVE_ON;
  bRightArmDownAirBagValve=VALVE_ON;  
    
    
  bLeftNeckAirBagValve=VALVE_OFF;
  bRightNeckAirBagValve=VALVE_OFF;
*/
    bLeftArmUpAirBagVave=VALVE_OFF;//add  2015-10-16
  bRightArmUpAirBagValve=VALVE_OFF;
  //bLeftNeckAirBagValve=VALVE_OFF;
  //bRightNeckAirBagValve=VALVE_OFF;
  bLeftArmDownAirBagValve=VALVE_OFF;
  bRightArmDownAirBagValve=VALVE_OFF; 
  //----------------------------------------------
  
  
  
  bRightFootAirBagValve = VALVE_ON;
  bLeftFootAirBagValve = VALVE_ON;
  bLegDownSideAirBagValve =VALVE_OFF;// VALVE_ON; 
  bLegUpSideAirBagValve = VALVE_OFF;//VALVE_ON;
  //bRightWaistAirBagValve =VALVE_OFF;// VALVE_ON ;
 // bLeftWaistAirBagValve =VALVE_OFF;// VALVE_ON ;
  
  
  
  //bButtocksAirBagValve = VALVE_OFF;
  bLeftThighAirBagValve = VALVE_OFF;
  bRightThighAirBagValve = VALVE_OFF; 
  bFootHeelAirBagValve = VALVE_OFF;//VALVE_ON;
  bLegDownBottomAirBagValve = VALVE_OFF;
  bLegUpBottomAirBagValve = VALVE_OFF; 
  
  
  

}
void Valve_SetStretchChargeATOUTFootHeelOFF(void)//找脚，释放小腿和 足部
{
  
 Valve_AirPumpACPowerOn();// Valve_ArmAirPumpACPowerOff(); //   modify by taoqingosng  2015-10-15
  
  //-----------------------------------------
 // bSholderAirBagValve  = VALVE_ON ; 
           bRightSholderAirBagValve = VALVE_ON ;  	
           bLeftSholderAirBagValve = VALVE_ON ;
  
  
  bLeftArmUpAirBagVave=VALVE_OFF;//add  2015-10-16
  bRightArmUpAirBagValve=VALVE_OFF;
  //bLeftNeckAirBagValve=VALVE_OFF;
  //bRightNeckAirBagValve=VALVE_OFF;
  bLeftArmDownAirBagValve=VALVE_OFF;
  bRightArmDownAirBagValve=VALVE_OFF;  
  
  //---------------------------------------- 
  
  
  
  bRightFootAirBagValve = VALVE_OFF;//VALVE_ON;
  bLeftFootAirBagValve =VALVE_OFF;// VALVE_ON;
  bLegDownSideAirBagValve =VALVE_OFF; 
  bLegUpSideAirBagValve = VALVE_OFF;
 // bRightWaistAirBagValve = VALVE_OFF;//VALVE_ON ;
 // bLeftWaistAirBagValve = VALVE_OFF;//VALVE_ON ;
  
  
  
 // bButtocksAirBagValve = VALVE_ON;//VALVE_OFF;
  bLeftThighAirBagValve = VALVE_ON;//VALVE_OFF;
  bRightThighAirBagValve = VALVE_ON;//VALVE_OFF; 
  bFootHeelAirBagValve = VALVE_OFF;
  bLegDownBottomAirBagValve = VALVE_OFF;
  bLegUpBottomAirBagValve = VALVE_OFF;
  
  
}







void Valve_SetStretchHold(void)    //modify by taoqingosng
{

    Valve_AirPumpACPowerOn(); //   modify by taoqingosng  2015-10-15
  
  bRightFootAirBagValve = VALVE_ON;
  bLeftFootAirBagValve = VALVE_ON;
  bFootHeelAirBagValve = VALVE_ON;
  bLegDownSideAirBagValve = VALVE_ON; 
  bLegUpSideAirBagValve = VALVE_ON;
  
  bLegDownBottomAirBagValve = VALVE_ON;
  bLegUpBottomAirBagValve = VALVE_ON;
  //bRightWaistAirBagValve = VALVE_ON ;
  //bLeftWaistAirBagValve = VALVE_ON ;
  //bButtocksAirBagValve  = VALVE_ON ;
  
  bLeftThighAirBagValve = VALVE_OFF;
  bRightThighAirBagValve = VALVE_OFF; 
           bRightSholderAirBagValve = VALVE_OFF ;  	
           bLeftSholderAirBagValve = VALVE_OFF ;
//------------------------------------------------------  
  bLeftArmUpAirBagVave=VALVE_OFF;//add  2015-10-16
  bRightArmUpAirBagValve=VALVE_OFF;
  //bLeftNeckAirBagValve=VALVE_OFF;
  //bRightNeckAirBagValve=VALVE_OFF;
  bLeftArmDownAirBagValve=VALVE_OFF;
  bRightArmDownAirBagValve=VALVE_OFF; 
//--------------------------------------------  
  
}
/*
   by_Data *10/15 = by_Data *2/3
*/
unsigned char Valve_Level_Decrease(unsigned char by_Data)
{
    unsigned char retval;
    unsigned int w_Data;
    unsigned int mod;
    if(by_Data <= 5) 
    {
        retval = by_Data;
    }
    else
    {
        w_Data = by_Data;
        w_Data *= 10;
        mod = w_Data % 15;
        w_Data /= 15;
        if(mod > 7) w_Data++;
        by_Data = (unsigned char)w_Data;
        retval = by_Data;
    }
    return retval;
}
/*
   by_Data *15/10 = by_Data *3/2
*/
unsigned char Valve_Level_Increase(unsigned char by_Data)
{
    unsigned char retval;
    unsigned int w_Data;
    unsigned int mod;
    if(by_Data <= 5) 
    {
        retval = by_Data;
    }
    else
    {
        w_Data = by_Data;
        w_Data *= 15;
        w_Data /= 10;
        mod = w_Data % 10;
        if(mod > 5) w_Data++;
        if(w_Data > 255) w_Data = 255;
        by_Data = (unsigned char)w_Data;
        retval = by_Data;
    }
    return retval;
}

void Valve_SetEnableSholder(unsigned int enable) //当背部主运行模式为自动模式时置0，否则置1，实际上此函数未使用
{
    bSholderEnable = enable;
}
#define VALVE_CLOSE_ALL_AIRPUMP  0
#define VALVE_OPEN_ALL_AIRPUMP   1
#define VALVE_NORMAL   2
void Valve_Airpump_Ctrl(unsigned int ctrl)
{
  if(ctrl == VALVE_CLOSE_ALL_AIRPUMP)
  {
    Valve_AirPumpACPowerOff();
    return;
  }
  if(ctrl == VALVE_OPEN_ALL_AIRPUMP)
  {
     Valve_AirPumpACPowerOn();
    return;
  }

  //bLeftWaistAirBagValve ||
     //bRightWaistAirBagValve ||
      if( bRightSholderAirBagValve ||
       bLeftSholderAirBagValve ||
           bLeftThighAirBagValve ||
             bRightThighAirBagValve|| 

    //bLeftNeckAirBagValve ||
                 //bRightNeckAirBagValve ||
                   bLeftArmUpAirBagVave ||
                     bLeftArmDownAirBagValve ||
                       bRightArmDownAirBagValve ||
                         bRightArmUpAirBagValve||
                           
 bFootHeelAirBagValve ||
        bLeftFootAirBagValve ||
          bRightFootAirBagValve ||
            bLegDownBottomAirBagValve ||
              bLegDownSideAirBagValve ||
                bLegUpBottomAirBagValve ||
                  bLegUpSideAirBagValve)     
  {
    Valve_AirPumpACPowerOn();//
  }
  else
  {
    Valve_AirPumpACPowerOff();
  }       
}


void Valve_Control(unsigned char nAirBagSwitch,st_AirBag* pBag,unsigned char level)
{
    bool bNextStep = FALSE;
    // static unsigned int SholdTime;
    static unsigned int AirTime;
    
    /*if(!bSholderEnable)
    {
      if((Input_GetWalkMotorPosition() > 40 ))
      {
        //bSholderAirBagValve = VALVE_OFF ;
           bRightSholderAirBagValve = VALVE_OFF ;  	
           bLeftSholderAirBagValve = VALVE_OFF ;
      }
    }*/
    
    if(nAirBagSwitch == VALVE_DISABLE)
    {
      //Valve_Airpump_Ctrl(VALVE_CLOSE_ALL_AIRPUMP);
      switch(pBag->locate)
      {
      case AIRBAG_LOCATE_LEG_FOOT:        LegFootAirBagAction(FALSE,pBag->nCurPumpValveState); break;     
      case AIRBAG_LOCATE_ARM_NECK:        ArmNeckAirBagAction(FALSE,pBag->nCurPumpValveState,0); break;       
      case AIRBAG_LOCATE_SEAT:            SeatAirBagAction(FALSE,pBag->nCurPumpValveState); break;
      case AIRBAG_LOCATE_AUTO:
        LegFootAirBagAction(FALSE,pBag->nCurPumpValveState); //大腿小腿气囊
        ArmNeckAirBagAction(FALSE,pBag->nCurPumpValveState,0);  //手臂+肩
        SeatAirBagAction(FALSE,pBag->nCurPumpValveState); //臀部
        break;       
     //add wgh 20170208
        
      case (AIRBAG_LOCATE_LEG_FOOT | AIRBAG_LOCATE_ARM_NECK): 
        LegFootAirBagAction(FALSE,pBag->nCurPumpValveState); //大腿小腿气囊
        ArmNeckAirBagAction(FALSE,pBag->nCurPumpValveState,0);  //手臂+肩
        break;
      case (AIRBAG_LOCATE_LEG_FOOT | AIRBAG_LOCATE_SEAT):
        LegFootAirBagAction(FALSE,pBag->nCurPumpValveState); //大腿小腿气囊
        SeatAirBagAction(FALSE,pBag->nCurPumpValveState); //臀部
        break;
      case (AIRBAG_LOCATE_ARM_NECK| AIRBAG_LOCATE_SEAT):
        ArmNeckAirBagAction(FALSE,pBag->nCurPumpValveState,0);  //手臂+肩
        SeatAirBagAction(FALSE,pBag->nCurPumpValveState); //臀部
        break;    
      case (AIRBAG_LOCATE_LEG_FOOT | AIRBAG_LOCATE_ARM_NECK | AIRBAG_LOCATE_SEAT):
        LegFootAirBagAction(FALSE,pBag->nCurPumpValveState); //大腿小腿气囊
        ArmNeckAirBagAction(FALSE,pBag->nCurPumpValveState,0);  //手臂+肩
        SeatAirBagAction(FALSE,pBag->nCurPumpValveState); //臀部
        break;     
        
        
        
      }
      return; 
    }
    if(pBag->init == TRUE)////当用户按气囊按键时，init=TRUE表示该气囊程序将开始运行的标志
    {
        bNextStep = TRUE;
    }
    else
    {
        bNextStep = AirBagGetNextStep(pBag);   //读取气囊数组的气囊状态st_AirBagAuto.pAirBagArray = AirBagModeAuto;// AirBagModeAuto[] =气囊数组
    }
    if(bNextStep == TRUE)//气囊程序运行下一步指示标志位(每一步气囊运行一段时间结束后自动进入到下一步)
    {
        if(pBag->locate == AIRBAG_LOCATE_LEG_FOOT)
        {
            w_RollerCounter = 0;
        }
        if(pBag->init == TRUE)
        {
            pBag->init = FALSE;//当用户第一次进入后，关闭该标志位，等待程序自动获取下一步气囊动作
            pBag->nCurAirBagStep = 0;
            pBag->nAirBagCounter = 0;
        }
        else//第二次进入该模块，当前气囊动作步数增加1，
        {
            pBag->nCurAirBagStep++;
            if(pBag->nCurAirBagStep >= pBag->nTotalSteps)
            {
                pBag->nCurAirBagStep = 0;
            }
        }
        //工程模式有3种气囊力度，在这里将3种气囊力度分为5档
        pBag->nCurPumpValveState = pBag->pAirBagArray[pBag->nCurAirBagStep].nPumpValveState ;// //当前选中的气囊气阀状态,第一次为第0步的气囊状态
        pBag->nCurKeepTime1 = pBag->pAirBagArray[pBag->nCurAirBagStep].nKeepTime1 ;//当前步数的气囊保存时间 
        pBag->nCurKeepTime3 = pBag->pAirBagArray[pBag->nCurAirBagStep].nKeepTime2 ;
        pBag->nCurKeepTime5 = pBag->pAirBagArray[pBag->nCurAirBagStep].nKeepTime3 ;//pBag->nCurAirBagStep=当前动作步骤
 // pBag->nCurKeepTime1=.nKeepTime1=当前状态保持时间,对应弱力度,同时对应气囊力度1
 //  pBag->nCurKeepTime3=   .nKeepTime2 ;对应气囊力度3
 // pBag->nCurKeepTime5 = nKeepTime3 ;对应气囊力度5 
  // pBag->nCurKeepTime2  =Middle(pBag->nCurKeepTime1,pBag->nCurKeepTime3);气囊力度2，保持时间在力度1和力度3之间
   // pBag->nCurKeepTime4  =Middle(pBag->nCurKeepTime3,pBag->nCurKeepTime5);气囊力度2，保持时间在力度1和力度3之间       


        //pBag->nCurKeepTime1 = (pBag->nCurKeepTime1>>1);//50%
        AirTime = (pBag->nCurKeepTime1)*2;//20170117
        AirTime = (AirTime)/5;            //20170117    80/100=8/10=4/5
        if(AirTime == 0)AirTime = 1;
        pBag->nCurKeepTime1 = (unsigned char)(AirTime);//20170117


        if(pBag->nCurPumpValveState != ALL_DIS)//表示选中某一个气阀或一组气阀
        {
            if(level  == 0)
            {
                pBag->nCurKeepTime1 = Valve_Level_Decrease(pBag->nCurKeepTime1);
                pBag->nCurKeepTime3 = Valve_Level_Decrease(pBag->nCurKeepTime3);
                pBag->nCurKeepTime5 = Valve_Level_Decrease(pBag->nCurKeepTime5);
                
                
            }
            if(level  == 2)
            {
                pBag->nCurKeepTime1 = Valve_Level_Increase(pBag->nCurKeepTime1);
                pBag->nCurKeepTime3 = Valve_Level_Increase(pBag->nCurKeepTime3);
                pBag->nCurKeepTime5 = Valve_Level_Increase(pBag->nCurKeepTime5);
        
            }
        }
        
        pBag->nCurKeepTime2 = Middle(pBag->nCurKeepTime1,pBag->nCurKeepTime3);
        pBag->nCurKeepTime2 = (pBag->nCurKeepTime2 - pBag->nCurKeepTime2/5);//20%
        
        
        //pBag->nCurKeepTime2 = Middle(pBag->nCurKeepTime1,pBag->nCurKeepTime3);
        pBag->nCurKeepTime4 = Middle(pBag->nCurKeepTime3,pBag->nCurKeepTime5);
  //以上为每一种气囊力度规定保持时间      
        
        /*
                pBag->nCurKeepTime1 = pBag->nCurKeepTime2;
                pBag->nCurKeepTime2 = pBag->nCurKeepTime3;
                pBag->nCurKeepTime3 = pBag->nCurKeepTime4;
                pBag->nCurKeepTime4 = pBag->nCurKeepTime5;
                pBag->nCurKeepTime5 = Valve_Level_Increase(pBag->nCurKeepTime5);
        */
        
        
        pBag->nAirBagCounter = 0;// pBag->nAirBagCounter;//当前气囊保持时间计数器，100ms timer counter
        switch(pBag->locate)
        {
        case AIRBAG_LOCATE_LEG_FOOT:        LegFootAirBagAction(TRUE,pBag->nCurPumpValveState); break;
     
        case AIRBAG_LOCATE_ARM_NECK:        ArmNeckAirBagAction(TRUE,pBag->nCurPumpValveState,0); break;       
        case AIRBAG_LOCATE_SEAT:            SeatAirBagAction(TRUE,pBag->nCurPumpValveState); break;
        case AIRBAG_LOCATE_AUTO:
          LegFootAirBagAction(TRUE,pBag->nCurPumpValveState); 
          ArmNeckAirBagAction(TRUE,pBag->nCurPumpValveState,0); 
          SeatAirBagAction(TRUE,pBag->nCurPumpValveState);      
          break;
          
     //add wgh 20170208
        
      case (AIRBAG_LOCATE_LEG_FOOT | AIRBAG_LOCATE_ARM_NECK): 
        LegFootAirBagAction(TRUE,pBag->nCurPumpValveState); //大腿小腿气囊
        ArmNeckAirBagAction(TRUE,pBag->nCurPumpValveState,0);  //手臂+肩
        break;
      case (AIRBAG_LOCATE_LEG_FOOT | AIRBAG_LOCATE_SEAT):
        LegFootAirBagAction(TRUE,pBag->nCurPumpValveState); //大腿小腿气囊
        SeatAirBagAction(TRUE,pBag->nCurPumpValveState); //臀部
        break;
      case (AIRBAG_LOCATE_ARM_NECK| AIRBAG_LOCATE_SEAT):
        ArmNeckAirBagAction(TRUE,pBag->nCurPumpValveState,0);  //手臂+肩
        SeatAirBagAction(TRUE,pBag->nCurPumpValveState); //臀部
        break;    
      case (AIRBAG_LOCATE_LEG_FOOT | AIRBAG_LOCATE_ARM_NECK | AIRBAG_LOCATE_SEAT):
        LegFootAirBagAction(TRUE,pBag->nCurPumpValveState); //大腿小腿气囊
        ArmNeckAirBagAction(TRUE,pBag->nCurPumpValveState,0);  //手臂+肩
        SeatAirBagAction(TRUE,pBag->nCurPumpValveState); //臀部
        break;     
          
          
          
          
          
        }
       // Valve_Airpump_Ctrl(VALVE_NORMAL);
    }
     Valve_Airpump_Ctrl(VALVE_NORMAL);
}

//24-31位为滚轮旋转方式
/*  0-1位为滚轮速度 00 停止 01 慢速 10 中速 11 高速
2-3位为滚轮旋转方式 01短间歇 02长间歇 10 连续
4-7位保留
*/

void Valve_LegKneadProce(unsigned char bLegKneadEnable,unsigned char Valve_Enable,st_AirBag* pBag)
{
  unsigned int speed,mode;
  if(bLegKneadFlag == 0) return;
  bLegKneadFlag = 0;
  bLegKneadEnable = 1; //test
  if(!bLegKneadEnable)
  {
    LegKnead_SetPower(LEG_KNEAD_OFF);
    LegKnead_SetSpeed(LEG_KNEAD_SPEED_STOP);
    return;
  }
  LegKnead_SetPower(LEG_KNEAD_ON);
  if(Valve_Enable)  //自动揉搓模式
  {
    speed = ((pBag->nCurPumpValveState >> 21) & 0x03);
    switch(speed)
    {
    default:  
    case 0:LegKnead_SetSpeed(LEG_KNEAD_SPEED_STOP);  break ;  
    case 1:LegKnead_SetSpeed(LEG_KNEAD_SPEED_SLOW);  break ;
    case 2:LegKnead_SetSpeed(LEG_KNEAD_SPEED_MID);  break ;
    case 3:LegKnead_SetSpeed(LEG_KNEAD_SPEED_FAST);  break ;
    }
    mode = ((pBag->nCurPumpValveState >> 23) & 0x03);
    switch(mode)
    {
    case 0:LegKnead_SetMode(LEG_KNEAD_TO_IN);  break ;  
    case 1:LegKnead_SetMode(LEG_KNEAD_TO_OUT);  break ;
    case 2:LegKnead_SetMode(LEG_KNEAD_TO_SWAY);  break ;
    default:
    case 3:  break ;
    }   
    return; 
  }
  //手动滚轮模式
  switch(LegKneadSpeed)
  {
  default:  
  case 0:LegKnead_SetSpeed(LEG_KNEAD_SPEED_STOP);  break ;  
  case 1:LegKnead_SetSpeed(LEG_KNEAD_SPEED_SLOW);  break ;
  case 2:LegKnead_SetSpeed(LEG_KNEAD_SPEED_MID);  break ;
  case 3:LegKnead_SetSpeed(LEG_KNEAD_SPEED_FAST);  break ;
  }
  switch(LegKneadMode)
  {
  case 0:LegKnead_SetMode(LEG_KNEAD_TO_IN);  break ;  
  case 1:LegKnead_SetMode(LEG_KNEAD_TO_OUT);  break ;
  case 2:LegKnead_SetMode(LEG_KNEAD_TO_SWAY);  break ;
  default:
  case 3:  break ;
  }   
}

void Valve_FootRollerProce(unsigned char bRollerEnable,unsigned char Valve_Enable,st_AirBag* pBag)
{ 
  unsigned int speed,mode;
    if(bRollerFlag == 0) return;
    bRollerFlag = 0;
    if(!bRollerEnable)
    {
       Roller_SetSpeed(ROLLER_SPEED_STOP);
       return;
    }
    if((Flex_GetDirection() == FLEX_TO_OUT) || (Flex_GetDirection() == FLEX_TO_IN))
    {
       Roller_SetSpeed(ROLLER_SPEED_STOP);
       return; 
    }
    if(Valve_Enable)  //自动滚轮模式
     {
        bAutoRoller = true;   
        speed = ((pBag->nCurPumpValveState >> 25) & 0x03);
        switch(speed)
        {
        default:  
        case 0:
          Roller_SetSpeed(ROLLER_SPEED_STOP);  bRollerDisplay =false;
          break ;  
        case 1:
          Roller_SetSpeed(ROLLER_SPEED_SLOW);  bRollerDisplay = true;
          break ;
        case 2:
          Roller_SetSpeed(ROLLER_SPEED_MID);  bRollerDisplay = true;
        break ;
        case 3:
          Roller_SetSpeed(ROLLER_SPEED_FAST);  bRollerDisplay = true;
        break ;
        }
        mode = ((pBag->nCurPumpValveState >> 27) & 0x07);
        Roller_SetMode(mode);
        return; 
    } 
    bAutoRoller = false;  //手动滚轮模式
    //(ROLLER_MODE_CON_IN);  //手动滚轮
    //Roller_SetSpeed(w_RollerPWM);
/*    if(bRoller3sFlag == true)
    {
      bRoller3sFlag =false;
      nRoller3sCnt++;
      if( nRoller3sCnt <= 3600)
      {
        //RollerMotor_Control(w_RollerPWM,1);
        Roller_SetSpeed(w_RollerPWM);
        Roller_SetMode(1);
      }
      else if(   ( nRoller3sCnt >3600)&&(nRoller3sCnt <7200)  )
      {
        //RollerMotor_Control(w_RollerPWM,0);
        Roller_SetSpeed(w_RollerPWM);
        Roller_SetMode(0);
      }
      else
      {
        nRoller3sCnt = 0;
      }
      
    }*/
    nRoller3sCnt2n++;
    if(nRoller3sCnt2n%2==0)
    {
        nRoller3sCnt++;
    }
    if( nRoller3sCnt <= 600)//5*3600= 18s
      {
        RollerMotor_Control(w_RollerPWM,0);
      }
      else if(   ( nRoller3sCnt > 600)&&(nRoller3sCnt < 1200)  )//3600
      {
        RollerMotor_Control(w_RollerPWM,1);
      }   
      else if(   ( nRoller3sCnt > 1200)&&(nRoller3sCnt < 1800)  )
      {
        RollerMotor_Control(w_RollerPWM,0);
      }
      else if(   ( nRoller3sCnt > 1800)&&(nRoller3sCnt < 2400)  )
      {
        RollerMotor_Control(w_RollerPWM,1);
      }
    
 //     
      else if(   ( nRoller3sCnt > 2400)&&(nRoller3sCnt < 2700)  )//300
      {
        RollerMotor_Control(w_RollerPWM,0);
      }
       else  if(   ( nRoller3sCnt > 2700)&&(nRoller3sCnt < 3000)  )
      {
        RollerMotor_Control(w_RollerPWM,1);
      }
      else if(   ( nRoller3sCnt >3000)&&(nRoller3sCnt < 3300)  )//200
      {
        RollerMotor_Control(w_RollerPWM,0);
      }
       else  if(   ( nRoller3sCnt > 3300)&&(nRoller3sCnt < 3600)  )
      {
        RollerMotor_Control(w_RollerPWM,1);
      }
    
      else if(   ( nRoller3sCnt > 3600)&&(nRoller3sCnt < 3900)  )//400
      {
        RollerMotor_Control(w_RollerPWM,0);
      }
       else  if(   ( nRoller3sCnt > 3900)&&(nRoller3sCnt < 4200)  )
      {
        RollerMotor_Control(w_RollerPWM,1);
      }
 //  -  - - 
      else if(   ( nRoller3sCnt > 4200)&&(nRoller3sCnt < 4500)  )//200
      {
        RollerMotor_Control(w_RollerPWM,0);
      }
       else  if(   ( nRoller3sCnt > 4500)&&(nRoller3sCnt < 4700)  )
      {
        RollerMotor_Control(0,0);
      }
      else if(   ( nRoller3sCnt > 4700)&&(nRoller3sCnt < 5100)  )
      {
        RollerMotor_Control(w_RollerPWM,0);
      }
       else  if(   ( nRoller3sCnt > 5100)&&(nRoller3sCnt < 5300)  )
      {
        RollerMotor_Control(0,0);
      }
      else if(   ( nRoller3sCnt > 5300)&&(nRoller3sCnt < 5700)  )
      {
        RollerMotor_Control(w_RollerPWM,0);
      }
       else  if(   ( nRoller3sCnt > 5700)&&(nRoller3sCnt < 5900)  )
      {
        RollerMotor_Control(0,0);
      }
 //   
      else if(   ( nRoller3sCnt > 5900)&&(nRoller3sCnt < 6300)  )//400
      {
        RollerMotor_Control(w_RollerPWM,0);
      }
       else  if(   ( nRoller3sCnt > 6300)&&(nRoller3sCnt < 6500)  )
      {
        RollerMotor_Control(0,0);
      }
      else if(   ( nRoller3sCnt > 6500)&&(nRoller3sCnt < 6900)  )
      {
        RollerMotor_Control(w_RollerPWM,0);
      }
       else  if(   ( nRoller3sCnt > 6900)&&(nRoller3sCnt < 7100)  )
      {
        RollerMotor_Control(0,0);
      }
      else if(   ( nRoller3sCnt > 7100)&&(nRoller3sCnt < 7400)  )
      {
        RollerMotor_Control(w_RollerPWM,0);
      }
       else  if(   ( nRoller3sCnt > 7400)&&(nRoller3sCnt < 7600)  )
      {
        RollerMotor_Control(0,0);
      }
 //////////////////////////////////////////   
      else if(   ( nRoller3sCnt > 7600)&&(nRoller3sCnt < 8000)  )//400
      {
        RollerMotor_Control(w_RollerPWM,1);
      }
       else  if(   ( nRoller3sCnt > 8000)&&(nRoller3sCnt < 8200)  )
      {
        RollerMotor_Control(0,0);
      }
      else if(   ( nRoller3sCnt > 8200)&&(nRoller3sCnt < 8600)  )
      {
        RollerMotor_Control(w_RollerPWM,1);
      }
       else  if(   ( nRoller3sCnt > 8600)&&(nRoller3sCnt < 8800)  )
      {
        RollerMotor_Control(0,0);
      }
      else if(   ( nRoller3sCnt > 8800)&&(nRoller3sCnt < 9200)  )
      {
        RollerMotor_Control(w_RollerPWM,1);
      }
       else  if(   ( nRoller3sCnt > 9200)&&(nRoller3sCnt < 9400)  )
      {
        RollerMotor_Control(0,0);
      }
      else if(   ( nRoller3sCnt > 9400)&&(nRoller3sCnt < 9800)  )//400
      {
        RollerMotor_Control(w_RollerPWM,1);
      }
       else  if(   ( nRoller3sCnt > 9800)&&(nRoller3sCnt < 10000)  )
      {
        RollerMotor_Control(0,0);
      }
      else if(   ( nRoller3sCnt > 10000)&&(nRoller3sCnt < 10400)  )
      {
        RollerMotor_Control(w_RollerPWM,1);
      }
       else  if(   ( nRoller3sCnt > 10400)&&(nRoller3sCnt < 10600)  )
      {
        RollerMotor_Control(0,0);
      }
      else if(   ( nRoller3sCnt > 10600)&&(nRoller3sCnt < 11000)  )
      {
        RollerMotor_Control(w_RollerPWM,1);
      }
       else  if(   ( nRoller3sCnt > 11000)&&(nRoller3sCnt < 11200)  )
      {
        RollerMotor_Control(0,0);
      }
//
      else if(   ( nRoller3sCnt >= 11200) )
      {
        nRoller3sCnt = 0;
      }  
    
    
    
    
}

void Valve_SetRollerPWM(unsigned char level)
{
    switch(level)
    {
    case 0:  w_RollerPWM = ROLLER_SPEED_STOP; break;
    case 1:  w_RollerPWM = ROLLER_SPEED_SLOW; break;
    case 2:  w_RollerPWM = ROLLER_SPEED_MID;  break;
    case 3:  w_RollerPWM = ROLLER_SPEED_FAST; break;
    }
}

unsigned char Valve_GetRollerLevel(void)
{
    unsigned char level = 0;
    if(w_RollerPWM == ROLLER_SPEED_SLOW) level = 1;
    if(w_RollerPWM == ROLLER_SPEED_MID) level = 2;
    if(w_RollerPWM == ROLLER_SPEED_FAST) level = 3;
    return level;
}

void Valve_SetLegKneadSpeed(unsigned char speed)
{
  if(speed > LEG_KNEAD_SPEED_FAST) return;
  LegKneadSpeed = speed;
}
unsigned char Valve_GetLegKneadSpeed(void)
{
    return(LegKneadSpeed);
}

unsigned char Valve_GetAirBagStrength(void)
{
    return(nKeyAirBagStrength);
}

void Valve_SetAirBagStrength(unsigned char strength)//气囊强度
{
    nKeyAirBagStrength = strength; //5种气囊力度
}

void Valve_AddAirBagStrength(void)
{
    nKeyAirBagStrength++;
    if(nKeyAirBagStrength > 5) nKeyAirBagStrength =1;
}


void Valve_AirPumpACPowerOff(void)
{
  GPIO_PinOutClear(VALVE_AIRPUMP1_PORT,VALVE_AIRPUMP1_BIT);

}

void Valve_AirPumpACPowerOn(void)//PB14
{
   GPIO_PinOutSet(VALVE_AIRPUMP1_PORT,VALVE_AIRPUMP1_BIT);
  
}





void Valve_Test_Set_Data(unsigned int ValveTestData)
{
    BITS_ValveData[0].nByte = (unsigned char)ValveTestData;
    BITS_ValveData[1].nByte = (unsigned char)(ValveTestData >> 8);
    //BITS_ValveData[2].nByte = (unsigned char)(ValveTestData >> 16);
}

void Valve_SetBackMode(int backauto) //背部自动模式时置1，否则置0
{
    bBackauto = (bool)backauto;
}

void  Valve_OzonOn(void)
{
      GPIO_PinOutSet(VALVE_OZON_PORT,VALVE_OZON_BIT);
}
void  Valve_OzonOff(void)
{
     GPIO_PinOutClear(VALVE_OZON_PORT,VALVE_OZON_BIT);
}

int Valve_RollerIsAuto(void)
{
 return (bAutoRoller) ;
}

/////////////////////////////////////////////////////

void Valve_SetStretchCharge0(unsigned int start)
{
    Valve_AirPumpACPowerOn();
    if((Flex_GetDirection() == FLEX_TO_OUT) || (Flex_GetDirection() == FLEX_TO_IN))
    {
      bLegUpSideAirBagValve = VALVE_OFF;                     
    bLegDownSideAirBagValve = VALVE_OFF;              
    //足部气囊
    bRightFootAirBagValve = VALVE_OFF;
    bLeftFootAirBagValve = VALVE_OFF;
    bFootHeelAirBagValve = VALVE_OFF;
    //大腿气囊
    bLeftThighAirBagValve       =  VALVE_OFF ;
    bRightThighAirBagValve 	=  VALVE_OFF ;  
    //胳膊气囊
    bLeftSholderAirBagValve   =  VALVE_OFF ;
    bRightSholderAirBagValve  =  VALVE_OFF ;
    //背腰气囊
    
    bLeftArmUpAirBagVave   =  VALVE_OFF ; 	 
    bLeftArmDownAirBagValve    =  VALVE_OFF ;  	  
    bRightArmUpAirBagValve    =  VALVE_OFF ;        
    bRightArmDownAirBagValve   =  VALVE_OFF ;  
    return;
    }

    //臂肩气囊

    //小腿气囊
    bLegUpSideAirBagValve = VALVE_OFF;                     
    bLegDownSideAirBagValve = VALVE_OFF;              
    //足部气囊
    bRightFootAirBagValve = VALVE_ON;
    bLeftFootAirBagValve = VALVE_ON;
    bFootHeelAirBagValve = VALVE_OFF;
    //大腿气囊
    bLeftThighAirBagValve       =  VALVE_OFF ;
    bRightThighAirBagValve 	=  VALVE_OFF ;  
    //胳膊气囊
    bLeftSholderAirBagValve   =  VALVE_OFF ;
    bRightSholderAirBagValve  =  VALVE_OFF ;
    //背腰气囊
    
    bLeftArmUpAirBagVave   =  VALVE_OFF ; 	 
    bLeftArmDownAirBagValve    =  VALVE_OFF ;  	  
    bRightArmUpAirBagValve    =  VALVE_OFF ;        
    bRightArmDownAirBagValve   =  VALVE_OFF ;   
}
void Valve_SetStretchChargepre(unsigned int start)
{
    Valve_AirPumpACPowerOn();

    static int step = 0;
    //if(start)
    //{
    //    step =  0; 
    //    SholderTime = 0;
    //}
    //臂肩气囊
    switch(step)
    {

    case 0: 
    bLeftArmUpAirBagVave   =  VALVE_OFF ; 	 
    bLeftArmDownAirBagValve    =  VALVE_OFF ;  	  
    bRightArmUpAirBagValve    =  VALVE_OFF ;        
    bRightArmDownAirBagValve   =  VALVE_OFF ;  

      if(SholderTime > 800)
      {step++;SholderTime = 0;}
      break;
    case 1: 
    bLeftArmUpAirBagVave   =  VALVE_OFF ; 	 
    bLeftArmDownAirBagValve    =  VALVE_OFF ;  	  
      bRightArmUpAirBagValve    =  VALVE_ON ;        
      bRightArmUpAirBagValve   =  VALVE_ON ; 
      //SholderTime = 0;
      //step++;
      if(SholderTime > 1200)
      {step++;SholderTime = 0;}
      break;
    case 2: 
      bLeftArmUpAirBagVave   =  VALVE_ON ; 	 
      bLeftArmDownAirBagValve    =  VALVE_ON ;  	  
      bRightArmUpAirBagValve    =  VALVE_OFF ;        
      bRightArmDownAirBagValve   =  VALVE_OFF ;  
      //SholderTime = 0;
      //step++;
      if(SholderTime > 1200)
      {step = 1;SholderTime = 0;}
      break;
 
    
    default:
      step =0;
      SholderTime =0;
      bLeftThighAirBagValve       =  VALVE_OFF ;
      bRightThighAirBagValve 	=  VALVE_OFF ; 

      break;
      
    }
    //小腿气囊
    bLegUpSideAirBagValve = VALVE_OFF;                     
    bLegDownSideAirBagValve = VALVE_OFF;              
    //足部气囊
    bRightFootAirBagValve = VALVE_OFF;
    bLeftFootAirBagValve = VALVE_OFF;
    bFootHeelAirBagValve = VALVE_OFF;
    //大腿气囊
    bLeftThighAirBagValve       =  VALVE_OFF ;
    bRightThighAirBagValve 	=  VALVE_OFF ;  
    //胳膊气囊
    bLeftSholderAirBagValve   =  VALVE_OFF ;
    bRightSholderAirBagValve  =  VALVE_OFF ;
    //背腰气囊
    
    //bLeftArmDownAirBagValve   =  VALVE_OFF ; 	 
    //bLeftArmUpAirBagVave    =  VALVE_OFF ;  	  
    //bRightArmUpAirBagValve    =  VALVE_OFF ;        
    //bRightArmDownAirBagValve   =  VALVE_OFF ;  
}

void Valve_SetStretchCharge1(unsigned int start)
{
    Valve_AirPumpACPowerOn();

if((Flex_GetDirection() == FLEX_TO_OUT) || (Flex_GetDirection() == FLEX_TO_IN))
{
      bLegUpSideAirBagValve = VALVE_OFF;                     
    bLegDownSideAirBagValve = VALVE_OFF;              
    //足部气囊
    bRightFootAirBagValve = VALVE_OFF;
    bLeftFootAirBagValve = VALVE_OFF;
    bFootHeelAirBagValve = VALVE_OFF;
}
else
{
      bLegUpSideAirBagValve = VALVE_ON;                     
    bLegDownSideAirBagValve = VALVE_ON;              
    //足部气囊
    bRightFootAirBagValve = VALVE_ON;
    bLeftFootAirBagValve = VALVE_ON;
    bFootHeelAirBagValve = VALVE_ON;
  
}
    //小腿气囊
    //bLegUpSideAirBagValve = VALVE_ON;                     
    //bLegDownSideAirBagValve = VALVE_ON;              
    //足部气囊
   // bRightFootAirBagValve = VALVE_ON;
   // bLeftFootAirBagValve = VALVE_ON;
   // bFootHeelAirBagValve = VALVE_OFF;
    //大腿气囊
    bLeftThighAirBagValve       =  VALVE_OFF ;
    bRightThighAirBagValve 	=  VALVE_OFF ;  
    //胳膊气囊
    bLeftSholderAirBagValve   =  VALVE_OFF ;
    bRightSholderAirBagValve  =  VALVE_OFF ;
    //背腰气囊
    
    bLeftArmUpAirBagVave   =  VALVE_OFF ; 	 
    bLeftArmDownAirBagValve    =  VALVE_OFF ;  	  
    bRightArmUpAirBagValve    =  VALVE_OFF ;        
    bRightArmDownAirBagValve   =  VALVE_OFF ;  
    
}


void Valve_SetStretchChargeSTEEL(unsigned int start)
{
   static int step = 0;
   /* if(start)
    {
        step =  0; 
        SholderTime = 0;
    }*/
    //臂肩气囊
    switch(step)
    {

    case 0: 
      bLegUpBottomAirBagValve   =  VALVE_OFF ; 	 
      bLegDownBottomAirBagValve    =  VALVE_OFF ;  	  
      bFootHeelAirBagValve    =  VALVE_OFF ;        

      if(SholderTime > 400)
      {step++;SholderTime = 0;}
      break;
    case 1: 
      bLegUpBottomAirBagValve   =  VALVE_ON ; 	 
      bLegDownBottomAirBagValve    =  VALVE_ON ;  	  
      bFootHeelAirBagValve    =  VALVE_ON ;     

      if(SholderTime > 1200)
      {step++;SholderTime = 0;}
      break;
     
    default:
      step =0;
      SholderTime =0;
      bLegUpBottomAirBagValve   =  VALVE_OFF ; 	 
      bLegDownBottomAirBagValve    =  VALVE_OFF ;  	  
      bFootHeelAirBagValve    =  VALVE_OFF ;   

      break;
      
    }
  
    Valve_AirPumpACPowerOn();

    //小腿气囊
    bLegUpSideAirBagValve = VALVE_ON;                     
    bLegDownSideAirBagValve = VALVE_ON;              
    //足部气囊
    bRightFootAirBagValve = VALVE_ON;
    bLeftFootAirBagValve = VALVE_ON;
    bFootHeelAirBagValve = VALVE_ON;
    //大腿气囊
    bLeftThighAirBagValve       =  VALVE_ON ;
    bRightThighAirBagValve 	=  VALVE_ON ;  
    //胳膊气囊
    bLeftSholderAirBagValve   =  VALVE_ON ;
    bRightSholderAirBagValve  =  VALVE_ON ;
    //背腰气囊
    
    bLeftArmUpAirBagVave   =  VALVE_OFF ; 	 
    bLeftArmDownAirBagValve    =  VALVE_OFF ;  	  
    bRightArmUpAirBagValve    =  VALVE_OFF ;        
    bRightArmDownAirBagValve   =  VALVE_OFF ;   
}

void Valve_SetStretchCharge_ARM_SHOULD(unsigned int start)
{
    Valve_AirPumpACPowerOn();

    static int step = 0;
    //if(start)
    //{
    //    step =  0; 
    //    SholderTime = 0;
    //}
    //臂肩气囊
    switch(step)
    {

    case 0: 
      bLeftArmDownAirBagValve   =  VALVE_OFF ; 	 
      bLeftArmUpAirBagVave    =  VALVE_OFF ;  	  
      bRightArmUpAirBagValve    =  VALVE_OFF ;        
      bRightArmDownAirBagValve   =  VALVE_OFF ; 

      if(SholderTime > 400)
      {step++;SholderTime = 0;}
      break;
    case 1: 
      bLeftArmDownAirBagValve   =  VALVE_OFF ; 	 
      bLeftArmUpAirBagVave    =  VALVE_OFF ;  	  
      bRightArmUpAirBagValve    =  VALVE_ON ;        
      bRightArmDownAirBagValve   =  VALVE_ON ; 
      
    bLeftSholderAirBagValve   =  VALVE_ON ;
    bRightSholderAirBagValve  =  VALVE_OFF ;
      
      //SholderTime = 0;
      //step++;
      if(SholderTime > 1500)
      {step++;SholderTime = 0;}
      break;
    case 2: 
      bLeftArmDownAirBagValve   =  VALVE_ON ; 	 
      bLeftArmUpAirBagVave    =  VALVE_ON ;  	  
      bRightArmUpAirBagValve    =  VALVE_OFF ;        
      bRightArmDownAirBagValve   =  VALVE_OFF ; 
      
    bLeftSholderAirBagValve   =  VALVE_OFF ;
    bRightSholderAirBagValve  =  VALVE_ON ;
      //SholderTime = 0;
      //step++;
      if(SholderTime > 1500)
      {step = 1;SholderTime = 0;}
      break;
 
    
    default:
      step =0;
      SholderTime =0;
      bLeftThighAirBagValve       =  VALVE_OFF ;
      bRightThighAirBagValve 	=  VALVE_OFF ; 
    bLeftSholderAirBagValve   =  VALVE_OFF ;
    bRightSholderAirBagValve  =  VALVE_OFF ;
      break;
      
    }
    //小腿气囊
    bLegUpSideAirBagValve = VALVE_OFF;                     
    bLegDownSideAirBagValve = VALVE_OFF;              
    //足部气囊
    bRightFootAirBagValve = VALVE_OFF;
    bLeftFootAirBagValve = VALVE_OFF;
    bFootHeelAirBagValve = VALVE_OFF;
    //大腿气囊
    bLeftThighAirBagValve       =  VALVE_OFF ;
    bRightThighAirBagValve 	=  VALVE_OFF ;  
    //胳膊气囊
    //bLeftSholderAirBagValve   =  VALVE_OFF ;
    //bRightSholderAirBagValve  =  VALVE_OFF ;
    //背腰气囊
    
    //bLeftArmDownAirBagValve   =  VALVE_OFF ; 	 
    //bLeftArmUpAirBagVave    =  VALVE_OFF ;  	  
    //bRightArmUpAirBagValve    =  VALVE_OFF ;        
    //bRightArmDownAirBagValve   =  VALVE_OFF ;  
}
//////////////////////////////////////////////////////////////
void Valve_SetStretchCharge_ARM(unsigned int start,unsigned char sstretch)
{
    Valve_AirPumpACPowerOn();

    static int step = 0;
    float vtimeadj; 
    float avtime; 
    unsigned int aat;
    
    if(nKeyAirBagStrength ==1)
    {
      vtimeadj = 0.8;
    }
    else
    {
      vtimeadj = 1;
    }
    //avtime = (400*vtimeadj);
     //aat = (unsigned int)(400*vtimeadj);
      
    //if(start)
    //{
    //    step =  0; 
    //    SholderTime = 0;
    //}
    //臂肩气囊
    switch(step)
    {
    case 0: 
      bLeftArmDownAirBagValve   =  VALVE_OFF ; 	 
      bLeftArmUpAirBagVave    =  VALVE_OFF ;  	  
      bRightArmUpAirBagValve    =  VALVE_OFF ;        
      bRightArmDownAirBagValve   =  VALVE_OFF ; 

      if(SholderTime > (unsigned int)(400*vtimeadj))//5*400=2000
      {step++;SholderTime = 0;}
      break;
    case 1: 
      bLeftArmDownAirBagValve   =  VALVE_OFF ; 	 
      bLeftArmUpAirBagVave    =  VALVE_OFF ;  	  
      bRightArmUpAirBagValve    =  VALVE_ON ;        
      bRightArmDownAirBagValve   =  VALVE_ON ; 
      //SholderTime = 0;
      //step++;
      if(SholderTime > (unsigned int)(1600*vtimeadj))
      {step++;SholderTime = 0;}
      break;
    case 2: 
      bLeftArmDownAirBagValve   =  VALVE_ON ; 	 
      bLeftArmUpAirBagVave    =  VALVE_ON ;  	  
      bRightArmUpAirBagValve    =  VALVE_OFF ;        
      bRightArmDownAirBagValve   =  VALVE_OFF ; 
      //SholderTime = 0;
      //step++;
      if(SholderTime > (unsigned int)(1600*vtimeadj))
      {step = 1;SholderTime = 0;}
      break;
 
    
    default:
      step =0;
      SholderTime =0;
      bLeftThighAirBagValve       =  VALVE_OFF ;
      bRightThighAirBagValve 	=  VALVE_OFF ; 

      break;
      
    }
    //小腿气囊
    bLegUpSideAirBagValve = VALVE_OFF;                     
    bLegDownSideAirBagValve = VALVE_OFF;              
    //足部气囊
    bRightFootAirBagValve = VALVE_OFF;
    bLeftFootAirBagValve = VALVE_OFF;
    bFootHeelAirBagValve = VALVE_OFF;
    //大腿气囊
    bLeftThighAirBagValve       =  VALVE_OFF ;
    bRightThighAirBagValve 	=  VALVE_OFF ;  
    //胳膊气囊
    bLeftSholderAirBagValve   =  VALVE_OFF ;
    bRightSholderAirBagValve  =  VALVE_OFF ;
    //背腰气囊
    
    //bLeftArmDownAirBagValve   =  VALVE_OFF ; 	 
    //bLeftArmUpAirBagVave    =  VALVE_OFF ;  	  
    //bRightArmUpAirBagValve    =  VALVE_OFF ;        
    //bRightArmDownAirBagValve   =  VALVE_OFF ;  
}
void Valve_SetStretchCharge_FOOT_THIGH_LEG(unsigned int start)
{
    Valve_AirPumpACPowerOn();

    //小腿气囊
    bLegUpSideAirBagValve = VALVE_ON;                     
    bLegDownSideAirBagValve = VALVE_ON;              
    //足部气囊
    bRightFootAirBagValve = VALVE_ON;
    bLeftFootAirBagValve = VALVE_ON;
    bFootHeelAirBagValve = VALVE_ON;
    //大腿气囊
    bLeftThighAirBagValve       =  VALVE_ON ;
    bRightThighAirBagValve 	=  VALVE_ON ;  
    //胳膊气囊
    bLeftSholderAirBagValve   =  VALVE_OFF ;
    bRightSholderAirBagValve  =  VALVE_OFF ;
    //背腰气囊
    
    bLeftArmDownAirBagValve   =  VALVE_OFF ; 	 
    bLeftArmUpAirBagVave    =  VALVE_OFF ;  	  
    bRightArmUpAirBagValve    =  VALVE_OFF ;        
    bRightArmDownAirBagValve   =  VALVE_OFF ;  
}
void Valve_SetStretchCharge_FOOT_THIGH_LEG_SHOULD(unsigned int start)
{
    Valve_AirPumpACPowerOn();

    //小腿气囊
    bLegUpSideAirBagValve = VALVE_ON;                     
    bLegDownSideAirBagValve = VALVE_ON;              
    //足部气囊
    bRightFootAirBagValve = VALVE_ON;
    bLeftFootAirBagValve = VALVE_ON;
    bFootHeelAirBagValve = VALVE_ON;
    //大腿气囊
    bLeftThighAirBagValve       =  VALVE_ON ;
    bRightThighAirBagValve 	=  VALVE_ON ;  
    //胳膊气囊
    bLeftSholderAirBagValve   =  VALVE_ON ;
    bRightSholderAirBagValve  =  VALVE_ON ;
    //背腰气囊
    
    bLeftArmDownAirBagValve   =  VALVE_OFF ; 	 
    bLeftArmUpAirBagVave    =  VALVE_OFF ;  	  
    bRightArmUpAirBagValve    =  VALVE_OFF ;        
    bRightArmDownAirBagValve   =  VALVE_OFF ;  
}

void Valve_SetStretchCharge_FOOT(unsigned int start)
{
    Valve_AirPumpACPowerOn();


    //臂肩气囊

    //小腿气囊
    bLegUpSideAirBagValve = VALVE_OFF;                     
    bLegDownSideAirBagValve = VALVE_OFF;              
    //足部气囊
    bRightFootAirBagValve = VALVE_ON;
    bLeftFootAirBagValve = VALVE_ON;
    bFootHeelAirBagValve = VALVE_ON;
    //大腿气囊
    //bLeftThighAirBagValve       =  VALVE_OFF ;//KR021
    //bRightThighAirBagValve 	=  VALVE_OFF ; //KR021 
    
    bLeftThighAirBagValve       =  VALVE_ON ;//US003
    bRightThighAirBagValve 	=  VALVE_ON ; //us003 
    
    //胳膊气囊
    bLeftSholderAirBagValve   =  VALVE_OFF ;
    bRightSholderAirBagValve  =  VALVE_OFF ;
    //背腰气囊
    
    bLeftArmDownAirBagValve   =  VALVE_OFF ; 	 
    bLeftArmUpAirBagVave    =  VALVE_OFF ;  	  
    bRightArmUpAirBagValve    =  VALVE_OFF ;        
    bRightArmDownAirBagValve   =  VALVE_OFF ;  
}
void Valve_SetStretchCharge_FOOT_THIGH(unsigned int start)
{
    Valve_AirPumpACPowerOn();


    //臂肩气囊

    //小腿气囊
    bLegUpSideAirBagValve = VALVE_OFF;                     
    bLegDownSideAirBagValve = VALVE_OFF;              
    //足部气囊
    bRightFootAirBagValve = VALVE_ON;
    bLeftFootAirBagValve = VALVE_ON;
    bFootHeelAirBagValve = VALVE_ON;
    //大腿气囊
    
    bLeftThighAirBagValve       =  VALVE_ON ;//US003
    bRightThighAirBagValve 	=  VALVE_ON ; //us003 
    
    //胳膊气囊
    bLeftSholderAirBagValve   =  VALVE_OFF ;
    bRightSholderAirBagValve  =  VALVE_OFF ;
    //背腰气囊
    
    bLeftArmDownAirBagValve   =  VALVE_OFF ; 	 
    bLeftArmUpAirBagVave    =  VALVE_OFF ;  	  
    bRightArmUpAirBagValve    =  VALVE_OFF ;        
    bRightArmDownAirBagValve   =  VALVE_OFF ;  
}

void Valve_SetStretchCharge_FOOT_THIGH_SHOULD(unsigned int start)
{
    Valve_AirPumpACPowerOn();


    //臂肩气囊

    //小腿气囊
    bLegUpSideAirBagValve = VALVE_OFF;                     
    bLegDownSideAirBagValve = VALVE_OFF;              
    //足部气囊
    bRightFootAirBagValve = VALVE_ON;
    bLeftFootAirBagValve = VALVE_ON;
    bFootHeelAirBagValve = VALVE_ON;
    //大腿气囊
    
    bLeftThighAirBagValve       =  VALVE_ON ;//US003
    bRightThighAirBagValve 	=  VALVE_ON ; //us003 
    
    //胳膊气囊
    bLeftSholderAirBagValve   =  VALVE_ON ;
    bRightSholderAirBagValve  =  VALVE_ON ;
    //背腰气囊
    
    bLeftArmDownAirBagValve   =  VALVE_OFF ; 	 
    bLeftArmUpAirBagVave    =  VALVE_OFF ;  	  
    bRightArmUpAirBagValve    =  VALVE_OFF ;        
    bRightArmDownAirBagValve   =  VALVE_OFF ;  
}