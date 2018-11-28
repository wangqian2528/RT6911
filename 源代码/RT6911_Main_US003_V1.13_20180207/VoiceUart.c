//#include "string.h"
#include "VoiceUart.h"
#include "efm32.h"
#include "em_chip.h"
#include "em_gpio.h"
#include "em_cmu.h"
#include "em_usart.h"
#include "efm32_types.h"
#include "efm32_def.h"
#include "ControlBox.h"

/* Declare a circular buffer structure to use for Rx and Tx queues */
//#define test_blue  1
//#define BUFFERSIZE          16
#define BUFFER_LENGTH       8

/* Setup UART1 in async mode for RS232*/
static USART_TypeDef           * uart   = UART1;
static USART_InitAsync_TypeDef uartInit = USART_INITASYNC_DEFAULT;

static unsigned char ucRXBuffer[BUFFER_LENGTH] = {0};
//__no_init static char TxBuffer[BUFFER_LENGTH];

static volatile struct circularBuffer
{
    uint8_t  data[BUFFER_LENGTH];  /* data buffer */
    // uint32_t rdI;               /* read index */
    uint32_t wrI;               /* write index */
    uint32_t pendingBytes;      /* count of how many bytes are not yet handled */
    bool     overflow;          /* buffer overflow indicator */
} txBuf = { 0, 0, 0, false };


static volatile int     rxWriteIndex = 0;
static unsigned char RX_Index;
static bool bPM25OK,bVoiceKeyOK;
static unsigned char voiceKey;
static unsigned int PM25 = NORMAL_CTRL;

static bool bEnablePluse;
static bool bMusicOn;
static unsigned int enablePluseCount;
static bool b100ms;
static unsigned int musicOnCounter,musicOffCounter;
static unsigned int pm25_FailCount;

/******************************************************************************
* @brief  uartSetup function
*
******************************************************************************/
void UART1_Initial_IO(void)
{
    /* Enable clock for GPIO module (required for pin configuration) */
    // CMU_ClockEnable(cmuClock_GPIO, true);
    /* Configure GPIO pins */
    GPIO_PinModeSet(UART1_RX_PORT,UART1_RX_BIT,UART1_RX_MODE, 1);
    
    GPIO_PinModeSet(VOICE_POWER_PORT,VOICE_POWER_BIT,VOICE_POWER_MODE, 1);
    
    /* Prepare struct for initializing UART in asynchronous mode*/
    uartInit.enable       = usartDisable;   /* Don't enable UART upon intialization */
    uartInit.refFreq      = 0;              /* Provide information on reference frequency. When set to 0, the reference frequency is */
#ifdef test_blue 
    uartInit.baudrate     = 115200;         /* Baud rate */
#else    
    uartInit.baudrate     = 2400;          /* Baud rate */
#endif    
    uartInit.oversampling = usartOVS16;     /* Oversampling. Range is 4x, 6x, 8x or 16x */
    uartInit.databits     = usartDatabits8; /* Number of data bits. Range is 4 to 10 */
    uartInit.parity       = usartNoParity;  /* Parity mode */
    uartInit.stopbits     = usartStopbits1; /* Number of stop bits. Range is 0 to 2 */
    //uartInit.mvdis        = false;          /* Disable majority voting */
    //uartInit.prsRxEnable  = false;          /* Enable USART Rx via Peripheral Reflex System */
    //uartInit.prsRxCh      = usartPrsRxCh0;  /* Select PRS channel if enabled */
    
    /* Initialize USART with uartInit struct */
    USART_InitAsync(uart, &uartInit);
    
    /* Prepare UART Rx and Tx interrupts */
    USART_IntClear(uart, _UART_IF_MASK);
    USART_IntEnable(uart, UART_IF_RXDATAV);
    NVIC_ClearPendingIRQ(UART1_RX_IRQn);
    NVIC_EnableIRQ(UART1_RX_IRQn);
    /* Enable I/O pins at UART1 location #3 */
    uart->ROUTE = UART_ROUTE_RXPEN |  UART_ROUTE_LOCATION_LOC0;
    /* Write welcome message to UART */
    // uartPutData((uint8_t*) welcomeString, welLen);
#ifdef test_blue     
    NVIC_ClearPendingIRQ(UART1_TX_IRQn);
    NVIC_EnableIRQ(UART1_TX_IRQn);
    /* Enable I/O pins at UART1 location #3 */
    GPIO_PinModeSet(UART1_TX_PORT,UART1_TX_BIT,UART1_TX_MODE, 1);
   /* 
    GPIO_PinOutToggle(UART1_TX_PORT, UART1_TX_BIT);
    GPIO_PinOutToggle(UART1_TX_PORT, UART1_TX_BIT);
    GPIO_PinOutToggle(UART1_TX_PORT, UART1_TX_BIT);
    GPIO_PinOutToggle(UART1_TX_PORT, UART1_TX_BIT);
    GPIO_PinOutToggle(UART1_TX_PORT, UART1_TX_BIT);
   */ 
    uart->ROUTE |= UART_ROUTE_TXPEN |  UART_ROUTE_LOCATION_LOC0;
#endif    
    /* Enable UART */
    USART_Enable(uart, usartEnable);
    
}


/**************************************************************************//**
* @brief UART1 RX IRQ Handler
*
* Set up the interrupt prior to use
*
* Note that this function handles overflows in a very simple way.
*
*****************************************************************************/
void UART1_RX_IRQHandler(void)
{
  /* Check for RX data valid interrupt */
  if (uart->STATUS & UART_STATUS_RXDATAV)
  {
    /* Copy data into RX Buffer */
    uint8_t rxData = USART_Rx(uart);
    switch(rxData)
    { 
      //正常处理情况1
    case SOI:
      RX_Index = 0; 
      ucRXBuffer[0] = rxData;
      RX_Index++; 
      break ;
      //正常处理情况2
    case EOI:                 
      if(ucRXBuffer[0] == SOI)
      {
       PM25 = (ucRXBuffer[2]&0x7f);
       PM25 = PM25<<7;
       PM25 |= (ucRXBuffer[3]&0x7f); 
       bPM25OK = true;
       pm25_FailCount = 0;
       bEnablePluse = 1;
       if(ucRXBuffer[1] == 0xff) 
       {
        break;
       }
       voiceKey = (ucRXBuffer[1]&0x7f);  
       /*
       switch (ucRXBuffer[1]&0x7f)
       {
       case 0x01: voiceKey = H10_KEY_CHAIR_AUTO_1 | VOICE_KEY_MASK;   break;  
       case 0x02: voiceKey = H10_KEY_CHAIR_AUTO_3 | VOICE_KEY_MASK;   break;
       case 0x03: voiceKey = H10_KEY_CHAIR_AUTO_0 | VOICE_KEY_MASK;   break;
       case 0x04: voiceKey = H10_KEY_CHAIR_AUTO_2 | VOICE_KEY_MASK;   break;
       case 0x05: voiceKey = H10_KEY_CHAIR_AUTO_4 | VOICE_KEY_MASK;   break;
       case 0x06: voiceKey = H10_KEY_CHAIR_AUTO_5 | VOICE_KEY_MASK;   break;
       case 0x07: voiceKey = H10_KEY_AIRBAG_AUTO | VOICE_KEY_MASK;   break;  
       case 0x08: voiceKey = H10_KEY_3DMODE_1 | VOICE_KEY_MASK;   break;  
       case 0x09: voiceKey = H10_KEY_3DMODE_2 | VOICE_KEY_MASK;   break;  
       case 0x0a: voiceKey = H10_KEY_3DMODE_3 | VOICE_KEY_MASK;   break;  
       case 0x0b: voiceKey = H10_KEY_POWER_SWITCH | VOICE_KEY_MASK;   break;  
       default: voiceKey = ucRXBuffer[1]&0x7f;
       }
       */
       bVoiceKeyOK = 1;
      }
      break ;
      //正常处理情况3
    default:
      ucRXBuffer[RX_Index] = rxData;
      RX_Index++;
      RX_Index %= BUFFER_LENGTH;
      break ;
    }
    /* Clear RXDATAV interrupt */
    USART_IntClear(uart, UART_IF_RXDATAV);
  }
}

/**************************************************************************//**
* @brief UART1 TX IRQ Handler
*
* Set up the interrupt prior to use
*
*****************************************************************************/
#ifdef test_blue 
void UART1_TX_IRQHandler(void)
{
  uint32_t irqFlags = USART_IntGet(uart);
  if (uart->STATUS & USART_STATUS_TXBL)
  {
    if (txBuf.pendingBytes > 0)
    {
      /* Transmit pending character */
      USART_Tx(uart, txBuf.data[txBuf.wrI]);
      txBuf.wrI++;
      txBuf.pendingBytes--;
    }
    /* Disable Tx interrupt if no more bytes in queue */
    if (txBuf.pendingBytes == 0)
    {
      USART_IntDisable(uart, UART_IF_TXBL);
    }
  }
}


static void uartPutData(uint8_t * dataPtr, uint32_t dataLen)
{
    int i = 0;
    
    /* Check if buffer is large enough for data */
    if (dataLen > BUFFER_LENGTH)
    {
        /* Buffer can never fit the requested amount of data */
        return;
    }
    while (i < dataLen)
    {
        txBuf.wrI = 0;
        txBuf.data[i] = *(dataPtr + i);
        i++;
    }
    
    /* Increment pending byte counter */
    txBuf.pendingBytes = dataLen;
    
    /* Enable interrupt on USART TX Buffer*/
    USART_IntEnable(uart, UART_IF_TXBL);
}

void UART1_Transmit_Packet(unsigned char* buf,unsigned int length)
{
    uartPutData(buf,length);
}
#endif
unsigned char VoiceUart_GetRXStatus(void)
{
    return((unsigned char)bVoiceKeyOK);
}

unsigned char VoiceUart_GetKey(void)
{
  return((unsigned char)voiceKey);
}

void VoiceUart_SetKey(unsigned char by_Data)
{
    voiceKey = by_Data;
}
void VoiceUart_ClearRXStatus(void)
{
    bVoiceKeyOK = 0;
} 

void VoiceUart_SetRXStatus(void)
{
    bVoiceKeyOK = 1;
} 

int VoiceUart_GetPM25(unsigned int* PM)
{
    if(bPM25OK)
    {
      bPM25OK = false;
      *PM = PM25; 
      return(1); 
    }
    else
    {
      return(-1); 
    } 
}

void VoiceUart_1msInt(void)
{
  return;  
  //static bool justMustOn = false;
   if(bEnablePluse)
   {
     enablePluseCount++;
     if(enablePluseCount == 10)
     {
       if(bMusicOn) 
       {
    
         UART1->ROUTE &= ~UART_ROUTE_RXPEN;
         GPIO_PinModeSet(UART1_RX_PORT,UART1_RX_BIT,gpioModePushPull, 0);
         __no_operation();
       }
     }
     if(enablePluseCount == 20)
     {
       //GPIO_PinModeSet(UART1_RX_PORT,UART1_RX_BIT,UART1_RX_MODE, 1);
    
       UART1->ROUTE &= ~UART_ROUTE_RXPEN;
       GPIO_PinModeSet(UART1_RX_PORT,UART1_RX_BIT,UART1_RX_MODE, 1);
     }
     else if(enablePluseCount > 20)
     {
       bEnablePluse = 0;
       enablePluseCount = 0;
     }
   }
}
void VoiceUart_100msInt(void)
{
  b100ms = 1; 
  if(pm25_FailCount > 15)
  {
     bPM25OK = false;
  }
  else
  {
    pm25_FailCount++; 
  }
}

void VoiceUart_PowerOn(void)
{
  GPIO_PinOutSet(VOICE_POWER_PORT,VOICE_POWER_BIT); 
}
void VoiceUart_PowerOff(void)
{
  GPIO_PinOutClear(VOICE_POWER_PORT,VOICE_POWER_BIT); 
}

void VoiceUart_SetMusicStatus(bool musicOn)
{
  if(!b100ms) return;
  b100ms = 0;
  if(musicOn) 
  {
    musicOffCounter = 0;
    
    ///*
    if(musicOnCounter > 5)
    {
      bMusicOn = 1;
    }
    else
    {
      musicOnCounter++;  
    }
    //*/
    /*
    if(musicOnCounter == 5)
    {
      VoiceUart_PowerOff();
      bMusicOn = 1;
    }
    if(musicOnCounter == 10)
    {
      VoiceUart_PowerOn();
    }
    if(musicOnCounter > 10)
    {
      musicOnCounter = 100;   
    }
    */
  }
  else
  {
    musicOnCounter = 0;
    if(musicOffCounter > 5)
    {
      bMusicOn = 0;
    }
    else
    {
      musicOffCounter++;   
    }
  }
}

void VoiceUart_Proce(void)
{
 
}