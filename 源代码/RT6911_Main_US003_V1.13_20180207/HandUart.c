//#include "string.h"
#include "HandUart.h"
#include "efm32.h"
#include "em_chip.h"
#include "em_gpio.h"
#include "em_cmu.h"
#include "em_usart.h"
#include "efm32_types.h"
#include "efm32_def.h"
#include "ControlBox.h"
//#include "efm32.h"
//#include "em_cmu.h"
//#include "em_int.h"
//#include "em_gpio.h"
//#include "em_usart.h"
//#include "em_dma.h"

/* Declare some strings */
//const char     welcomeString[]  = "Energy Micro RS-232 - Please press a key\n";
//const char     overflowString[] = "\n---RX OVERFLOW---\n";
//const uint32_t welLen           = sizeof(welcomeString) - 1;
//const uint32_t ofsLen           = sizeof(overflowString) - 1;

/* Define termination character */
//#define TERMINATION_CHAR    '.'

/* Declare a circular buffer structure to use for Rx and Tx queues */
#define BUFFERSIZE          256

volatile struct circularBuffer
{
    uint8_t  data[BUFFERSIZE];  /* data buffer */
    // uint32_t rdI;               /* read index */
    uint32_t wrI;               /* write index */
    uint32_t pendingBytes;      /* count of how many bytes are not yet handled */
    bool     overflow;          /* buffer overflow indicator */
} txBuf = { 0, 0, 0, false };

/* Setup UART1 in async mode for RS232*/
static USART_TypeDef           * uart   = UART0;
static USART_InitAsync_TypeDef uartInit = USART_INITASYNC_DEFAULT;

unsigned char ucRXBuffer[BUFFER_LENGTH] = {0};
unsigned char ucTXBuffer[BUFFER_LENGTH] = {0};
static volatile int     rxWriteIndex = 0;
unsigned char RX_Index;
static bool  bRXOK;

//DMA_DESCRIPTOR_TypeDef dmaControlBlock[DMA_CHAN_COUNT * 2];
static unsigned char by_Key,by_Key1;

unsigned int ctrlType = NORMAL_CTRL;

unsigned char *blueToothBuffer1 = "MS=02,";
unsigned char *blueToothBuffer2 = "RS=02,";
unsigned char blueToothAddr[12];
unsigned char blueToothAddrIndex;
unsigned char blueToothCompareStep = 0;
bool blueToothShouldConnect = false;
unsigned int blueToothCount = 0;
bool compareBluetoothMSG(uint8_t rxData);
bool compareBluetoothMSG2(uint8_t rxData);

/*下面变量用于更新云端自动程序*/
//----------------------------------------------------
//static unsigned char by_lingo ,by_command,by_prog_data;
//----------------------------------------------------

/******************************************************************************
* @brief  uartSetup function
*
******************************************************************************/
void UART0_Initial_IO(void)
{
    /* Enable clock for GPIO module (required for pin configuration) */
    // CMU_ClockEnable(cmuClock_GPIO, true);
    /* Configure GPIO pins */
    GPIO_PinModeSet(UART0_TX_PORT,UART0_TX_BIT,UART0_TX_MODE, 1);
    GPIO_PinModeSet(UART0_RX_PORT,UART0_RX_BIT,UART0_RX_MODE, 1);
    
    /* Prepare struct for initializing UART in asynchronous mode*/
    uartInit.enable       = usartDisable;   /* Don't enable UART upon intialization */
    uartInit.refFreq      = 0;              /* Provide information on reference frequency. When set to 0, the reference frequency is */
    uartInit.baudrate     = 9600;//115200;         /* Baud rate */
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
    NVIC_ClearPendingIRQ(UART0_RX_IRQn);
    NVIC_ClearPendingIRQ(UART0_TX_IRQn);
    NVIC_EnableIRQ(UART0_RX_IRQn);
    NVIC_EnableIRQ(UART0_TX_IRQn);
    
    
    /* Enable I/O pins at UART1 location #3 */
    uart->ROUTE = UART_ROUTE_RXPEN | UART_ROUTE_TXPEN | UART_ROUTE_LOCATION_LOC3;
    
    /* Enable UART */
    USART_Enable(uart, usartEnable);
    
    
    /* Write welcome message to UART */
    // uartPutData((uint8_t*) welcomeString, welLen);
}

unsigned char  AsciiToVal(unsigned char nAscii)
{
    if(nAscii>=0x30 && nAscii<=0x39) return(nAscii-0x30);
    if(nAscii>=0x41 && nAscii<=0x46) return(nAscii-0x37);
    if(nAscii>=0x61 && nAscii<=0x66) return(nAscii-0x57);
    return 0 ;
}

/**************************************************************************//**
* @brief UART0 RX IRQ Handler
*
* Set up the interrupt prior to use
*
* Note that this function handles overflows in a very simple way.
*
*****************************************************************************/
void UART0_RX_IRQHandler(void)
{
    unsigned char by_Data;
    /* Check for RX data valid interrupt */
    
    if (uart->STATUS & UART_STATUS_RXDATAV)//USART_STATUS_RXDATAV
    {
        /* Copy data into RX Buffer */
        uint8_t rxData = USART_Rx(uart);
        
        if(rxData == SOI)////   开始字节（SOI）：0XF0 数据帧开始字节
        {
            RX_Index = 0;  
        }
        else
        {
            if(rxData == EOI)// 结束字节（EOI）：0XF1 数据帧结束字节
            {
                by_Data = ucRXBuffer[0];
                ctrlType = by_Data;////接收手控器标识（ID） ，   0x82 TFT点阵显示手控器
      __no_operation();
      __no_operation();
      __no_operation();
      __no_operation();
       __no_operation();
      __no_operation();     
                by_Key = ucRXBuffer[1];    //按键编码
                      if(by_Key < 0x80) //按键值0-127
                      {
                          bRXOK = 1;
                      }
                      by_Key1 = ucRXBuffer[2]; //常规模式无此按键，硬件测试时使用                
                  

                
                
            }
            else
            {
                ucRXBuffer[RX_Index] = rxData;
                RX_Index++;
                RX_Index %= BUFFER_LENGTH;
            }
        }
        
        /* Clear RXDATAV interrupt */
        
         /* Clear RXDATAV interrupt *///  usart->IFC = flags;, 数据手册该位为保留位 
    //接收中断标志位在读取Rx数据之后，自动清零   
        
        USART_IntClear(UART0, UART_IF_RXDATAV);
    }
}

/**************************************************************************//**
* @brief UART1 TX IRQ Handler
*
* Set up the interrupt prior to use
*
*****************************************************************************/
void UART0_TX_IRQHandler(void)
{
    uint32_t irqFlags = USART_IntGet(UART0);
    
    /* Check TX buffer level status *///
    if (uart->STATUS & UART_STATUS_TXBL)////当TX_BUFFER=空 或半满状态时开始向BUFFER写数据
    {
        if (txBuf.pendingBytes > 0)
        {
            /* Transmit pending character */
            USART_Tx(uart, txBuf.data[txBuf.wrI]);
            txBuf.wrI++;
            txBuf.pendingBytes--;
        }
        
        /* Disable Tx interrupt if no more bytes in queue */
              //TXBL 0 RW TX Buffer Level Interrupt Enable
        if (txBuf.pendingBytes == 0)
        {
            USART_IntDisable(uart, UART_IF_TXBL);
        }
    }
}

/******************************************************************************
* @brief  uartPutData function
*每50ms刷新数据
*****************************************************************************/
static void uartPutData(uint8_t * dataPtr, uint32_t dataLen)
{
    int i = 0;
    
    /* Check if buffer is large enough for data */
    if (dataLen > BUFFERSIZE)
    {
        /* Buffer can never fit the requested amount of data */
        return;
    }
    /*
    if ((txBuf.pendingBytes + dataLen) > BUFFERSIZE)
    {
    while ((txBuf.pendingBytes + dataLen) > BUFFERSIZE) ;
}
    */
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

void HandUart_Transmit_Packet(unsigned char* buf,unsigned int length)
{
    uartPutData(buf,length);
}
unsigned char HandUart_GetRXStatus(void)
{
    return((unsigned char)bRXOK);
}

unsigned char HandUart_GetKey(void)
{
    return((unsigned char)by_Key);
}
unsigned char HandUart_GetExternKey(void)
{
    return((unsigned char)by_Key1);
}


void HandUart_SetKey(unsigned char by_Data)
{
    by_Key = by_Data;
}
void HandUart_ClearRXStatus(void)
{
    bRXOK = 0;
} 

void HandUart_SetRXStatus(void)
{
    bRXOK = 1;
} 
unsigned int HandUart_GetCtrlType(void)
{
    return(ctrlType); 
}




//-----------------------------------------------
void HandUart_SetCtrlType(unsigned char by_ctrlType)
{
  
  ctrlType=by_ctrlType;
  
  
}






/*下面函数用于跟新云端自动程序*/
//-----------------------------------------------
/*
unsigned char HandUart_Get_LINGO(void)
{
    return((unsigned char)by_lingo);
}


unsigned char HandUart_Get_COMMAND(void)
{
    return((unsigned char)by_command);
}


unsigned char HandUart_Get_PROG_DATA(void)
{
    return((unsigned char)by_prog_data);
}*/
//------------------------------------------------
