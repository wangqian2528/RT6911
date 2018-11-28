#include "efm32.h"
#include "em_chip.h"
#include "em_gpio.h"
#include "em_cmu.h"
#include "em_dma.h"
#include "em_leuart.h"
#include "efm32_types.h"
#include "efm32_def.h"
#include "ControlBox.h"
#include "DMA_Ctrl.h"
#include "LEUart_DMA.h"
#include "string.h"

static unsigned char ucRXBuffer1[BUFFER_LENGTH] = {0};
//static unsigned char ucTXBuffer1[BUFFER_LENGTH] = {0};
/* DMA control block, must be aligned to 256. */
//#pragma data_alignment=256
//extern DMA_DESCRIPTOR_TypeDef dmaControlBlock[DMA_CHAN_COUNT * 2];
static unsigned char massageSignal,massage3DPluse,massage3D_Speed;
//static unsigned short walkPluse;
static bool  bMassageSignalOK;
static unsigned char volatile errorCount;
void LEUART0_Initial_Data(void)
{
  bMassageSignalOK = false;//表示还没有接受到正确的数据
}
void LEUART0_Initial_IO(void)
{
  //test
    LEUART_Init_TypeDef Dmauart_Init = DMAUART_UART_Init;
    
    /* Reseting and initializing LEUART1 */
    LEUART_Reset(DMAUART);

    LEUART_Init(DMAUART, &Dmauart_Init);
    
    /* Route LEUART1 TX pin to DMA location 0 */
    DMAUART->ROUTE = DMAUART_ROUTE_EN | DMAUART_ROUTE_LOCATION;
    
    /* Enable GPIO for LEUART0.  */
    //GPIO_PinModeSet(DMAUART_TX_PORT,DMAUART_TX_BIT,DMAUART_TX_MODE,1);
    GPIO_PinModeSet(DMAUART_RX_PORT,DMAUART_RX_BIT,DMAUART_RX_MODE,1);
    
    cb[DMAUART_DMA_RX_CHANNEL].cbFunc  = NULL;
    cb[DMAUART_DMA_RX_CHANNEL].userPtr = NULL;
    
    
    DMA_CfgChannel_TypeDef chnlCfg =
    {
        .highPri   = false,                     /* Normal priority */
        .enableInt = true,                     /* No interupt enabled for callback functions */
        .select    = DMAREQ_LEUART0_RXDATAV,    /* Set LEUART0 RX data avalible as source of DMA signals */
        .cb        = &(cb[DMAUART_DMA_RX_CHANNEL]),
    };
    
    DMA_CfgChannel(DMAUART_DMA_RX_CHANNEL, &chnlCfg);

/* Setting up channel descriptor */
    DMA_CfgDescr_TypeDef descrCfg =
    {
        .dstInc  = dmaDataInc1,       /* Increment destination address by one byte */
        .srcInc  = dmaDataIncNone,    /* Do no increment source address  */
        .size    = dmaDataSize1,      /* Data size is one byte */
        .arbRate = dmaArbitrate1,     /* Rearbitrate for each byte recieved*/
        .hprot   = 0,                 /* No read/write source protection */
    };
    DMA_CfgDescr(DMAUART_DMA_RX_CHANNEL, true, &descrCfg);

    /* Starting the transfer. Using Basic Mode */
    DMA_ActivateBasic(DMAUART_DMA_RX_CHANNEL,     /* Activate channel selected */
                      true,                       /* Use primary descriptor */
                      false,                      /* No DMA burst */
                      (void *) &ucRXBuffer1,       /* Destination address */
                      (void *) &DMAUART->RXDATA,  /* Source address*/
                      BUFFER_LENGTH - 1);         /* Size of buffer minus1 */
    
    /* Set LEUART signal frame */
    DMAUART->SIGFRAME = EOI;
    
    DMAUART->STARTFRAME = SOI;
    
/************************************************************************
    cb[DMAUART_DMA_TX_CHANNEL].cbFunc  = NULL;
    cb[DMAUART_DMA_TX_CHANNEL].userPtr = NULL;
     DMA_CfgChannel_TypeDef chnlCfg1 =
    {
        .highPri   = false,                     
        .enableInt = true,                     
        .select    = DMAREQ_LEUART0_TXEMPTY,    
        .cb        = &(cb[DMAUART_DMA_TX_CHANNEL]),
    };
    DMA_CfgChannel(DMAUART_DMA_TX_CHANNEL, &chnlCfg1);
    
    DMA_CfgDescr_TypeDef descrCfg1 =
    {
        .dstInc  = dmaDataIncNone,    
        .srcInc  = dmaDataInc1,       
        .size    = dmaDataSize1,      
        .arbRate = dmaArbitrate1,     
        .hprot   = 0,                 
    };
    DMA_CfgDescr(DMAUART_DMA_TX_CHANNEL, true, &descrCfg1);
    
    (dmaControlBlock + DMAUART_DMA_TX_CHANNEL)->DSTEND = &DMAUART->TXDATA;
    
*******************************************************************/    
    /* Enable LEUART Signal Frame Interrupt */
    LEUART_IntEnable(DMAUART, DMAUART_IEN_STARTF | DMAUART_IEN_SIGF /*| DMAUART_IEN_TXC*/);
    
    /* Enable LEUART1 interrupt vector */
    NVIC_EnableIRQ(DMAUART_IRQ);
    
    /* Make sure the LEUART wakes up the DMA on RX data */
    DMAUART->CTRL = LEUART_CTRL_RXDMAWU;
    //DMA->IEN &= ~0X0006;
    //DMA->IEN &= ~(1 << DMAUART_DMA_TX_CHANNEL);
    DMA->IEN &= ~(1 << DMAUART_DMA_RX_CHANNEL);
}
/*
void LEUART0_Transmit_Packet(unsigned char* buf,unsigned int length)
{
    memcpy(ucTXBuffer1,buf,length);
  
    (dmaControlBlock + DMAUART_DMA_TX_CHANNEL)->SRCEND = ucTXBuffer1 + length - 1;
    
     DMA_ActivateBasic(DMAUART_DMA_TX_CHANNEL,      
                    true,                         
                    false,                        
                    NULL,                         
                    NULL,                         
                    length - 1);           
}
*/
void LEUART0_IRQHandler(void)//读取3D电机脉冲信号
{
  unsigned long leuartif;
  unsigned long len;
  unsigned char checkSum;
  /* Store and reset pending interupts */
  leuartif = LEUART_IntGet(DMAUART);
  LEUART_IntClear(DMAUART, leuartif);

  /* Signal frame found. */
  if (leuartif & LEUART_IF_SIGF)
  {  //接收到正确的结束祯信号，取出数据
    len  = BUFFER_LENGTH - (dmaControlBlock->CTRL >> 4) & 0x3FF;
    if(len==5)  //判断数据长度
    {
      checkSum = ucRXBuffer1[0];
      checkSum += ucRXBuffer1[1];
      //checkSum += ucRXBuffer1[2];
      checkSum = ~checkSum;
      checkSum &= 0x7f;
      if(checkSum == ucRXBuffer1[2])
      {
        massageSignal = ucRXBuffer1[0];    //机芯信号
        massage3DPluse = ucRXBuffer1[1];//3D脉冲
        massage3D_Speed = 5;//3D机芯速度默认为5档
        bMassageSignalOK = true;//正常收到3D机芯的数据标志
        errorCount = 0;
      }
    }
  }
  if(leuartif & LEUART_IF_STARTF)  //接收到正确的开始祯信号，重置DMA
  {
      /* Reactivate DMA */
      DMA_ActivateBasic(DMAUART_DMA_RX_CHANNEL,/* Activate DMA channel 0 */
                        true,            /* Activate using primary descriptor */
                        false,           /* No DMA burst */
                        NULL,            /* Keep source */
                        NULL,            /* Keep destination */
                        BUFFER_LENGTH - 1);/* Number of DMA transfer elements (minus 1) */
  }
  if (leuartif & LEUART_IF_TXBL)
  {
      len++; // just for test
  }
}
/*
unsigned char LEUART0_GetRXStatus(void)
{
  return((unsigned char)bRXOK);
}
void LEUART0_ClearRXStatus(void)
{
  bRXOK = 0;
}
*/
bool LEUART0_isOK(void)
{
  return(bMassageSignalOK);////正常收到3D机芯的数据标志
}

unsigned char LEUART0_GetMassageSignal(void)
{
  return(massageSignal);//3D机芯开关状态信号
}

unsigned char LEUART0_Get3DPluse(void)
{
  return(massage3DPluse);
}

unsigned char LEUART0_Get3D_Speed(void)
{
  return(massage3D_Speed);
}

/*
unsigned short LEUART0_GetWalkPluse(void)
{
  return(walkPluse);
}
*/
void LEUART0_10msInt(void)//10ms timer      机芯检测板 每30ms发送数据给主板
{
  if(errorCount < 255) errorCount++;
  if(errorCount > 10) bMassageSignalOK = false;
}

