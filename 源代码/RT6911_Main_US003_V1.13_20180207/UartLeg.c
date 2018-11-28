#include "em_gpio.h"
#include "em_usart.h"
#include "UartLeg.h"
#include "EFM32_def.h"
#include "EFM32_types.h"
#define LEG_BUFFERSIZE          32
#define LEG_BUFFER_LENGTH             32

static volatile struct circularBuffer
{
    uint8_t  data[LEG_BUFFERSIZE];  /* data buffer */
    // uint32_t rdI;               /* read index */
    uint32_t wrI;               /* write index */
    uint32_t pendingBytes;      /* count of how many bytes are not yet handled */
    bool     overflow;          /* buffer overflow indicator */
}txLegBuf = { 0, 0, 0, false };

volatile unsigned char ucLegRXBuffer[LEG_BUFFER_LENGTH] = {0};
volatile unsigned char ucLegTXBuffer[LEG_BUFFER_LENGTH] = {0};

unsigned char RX_Leg_Index;
bool  bRXOK_Leg;

unsigned char by_Leg_Key,by_Leg_Key1;

static USART_TypeDef           * uart   = USART1;
static USART_InitAsync_TypeDef uartInit = USART_INITASYNC_DEFAULT;

#define SOI                    0XF0
#define EOI                    0XF1

void UartLeg_Initial_IO(void)
{
    /* Enable clock for GPIO module (required for pin configuration) */
    // CMU_ClockEnable(cmuClock_GPIO, true);
    /* Configure GPIO pins */
    GPIO_PinModeSet(UARTLEG_TX_PORT,UARTLEG_TX_BIT,UARTLEG_TX_MODE, 1);
    GPIO_PinModeSet(UARTLEG_RX_PORT,UARTLEG_RX_BIT,UARTLEG_RX_MODE, 1);
    
    /* Prepare struct for initializing UART in asynchronous mode*/
    uartInit.enable       = usartDisable;   /* Don't enable UART upon intialization */
    uartInit.refFreq      = 0;              /* Provide information on reference frequency. When set to 0, the reference frequency is */
    uartInit.baudrate     = 9600;//9600;//115200;         /* Baud rate */
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
    USART_IntClear(uart, _USART_IF_MASK);// usart->IFC = flags;,该函数清除所有中断标志寄存器，=发送和接收中断标志位
    USART_IntEnable(uart, USART_IF_RXDATAV);////USART0->IEN = USART_IEN_RXDATAV;使能UART接收中断。
    //该中断使能寄存器包含使能接收中断，和使能发送中断，目前只使能接收中断，
  //USART_IntEnable(uart, UART_IF_TXC);//能发送中断
 //   USART_IntEnable(uart, UART_IF_TXBL);
  //TXBL 0 RW TX Buffer Level Interrupt Enable
  //TXC 0 RW TX Complete Interrupt Enable
  //  USART_IntDisable(uart, UART_IF_TXBL);// usart->IEN &= ~(flags);  
    
    NVIC_ClearPendingIRQ(USART1_RX_IRQn);
    NVIC_ClearPendingIRQ(USART1_TX_IRQn);
    NVIC_EnableIRQ(USART1_RX_IRQn);
    NVIC_EnableIRQ(USART1_TX_IRQn);
    
    
    /* Enable I/O pins at UART1 location #3 */
    uart->ROUTE = USART_ROUTE_RXPEN | USART_ROUTE_TXPEN | USART_ROUTE_LOCATION_LOC0;
    
    /* Enable UART */  // usartEnable   = (USART_CMD_RXEN | USART_CMD_TXEN)使能发送和接收
    USART_Enable(uart, usartEnable);//// usart->CMD = (uint32_t)(enable);
    
    
    /* Write welcome message to UART */
    // uartPutData((uint8_t*) welcomeString, welLen);
}

/**************************************************************************//**
* @brief UART0 RX IRQ Handler
*
* Set up the interrupt prior to use
*
* Note that this function handles overflows in a very simple way.
*
*****************************************************************************/
// //USART_IntEnable(uart, UART_IF_RXDATAV);//USART0->IEN = USART_IEN_RXDATAV;使能UART接收中断
void USART1_RX_IRQHandler(void)
{
  //  unsigned char by_Data;
    /* Check for RX data valid interrupt */
    
    if (uart->STATUS & USART_STATUS_RXDATAV)//RXDATAV=1表示BUFFER有 数据可以接收
    {
        /* Copy data into RX Buffer */
        uint8_t rxData = USART_Rx(uart);
        
        if(rxData == SOI)
        {
            RX_Leg_Index = 0;  
        }
        else
        {
            if(rxData == EOI)
            {     
                bRXOK_Leg = 1;    
            }
            else
            {
                ucLegRXBuffer[RX_Leg_Index] = rxData;
                RX_Leg_Index++;
                RX_Leg_Index %= LEG_BUFFER_LENGTH;
            }
        }
        
        /* Clear RXDATAV interrupt */      //   //接收中断标志位在读取Rx数据之后，自动清零
        USART_IntClear(USART1, USART_IF_RXDATAV);
    }
}

/**************************************************************************//**
* @brief UART1 TX IRQ Handler
*
* Set up the interrupt prior to use
*
*****************************************************************************/
void USART1_TX_IRQHandler(void)
{
    uint32_t irqFlags = USART_IntGet(USART1);
    
    /* Check TX buffer level status */
    if (uart->STATUS & USART_STATUS_TXBL)////当TX_BUFFER=空 或半满状态时开始向BUFFER写数据
    {
        if (txLegBuf.pendingBytes > 0)
        {
            /* Transmit pending character */
            USART_Tx(uart, txLegBuf.data[txLegBuf.wrI]);
            txLegBuf.wrI++;
            txLegBuf.pendingBytes--;
        }
        
        /* Disable Tx interrupt if no more bytes in queue */
        if (txLegBuf.pendingBytes == 0)
        {     //TXBL 0 RW TX Buffer Level Interrupt Enable
            USART_IntDisable(uart, USART_IF_TXBL);//发送完成禁止发送中断
        }
    }
}

void uartLegPutData(uint8_t * dataPtr, uint32_t dataLen)
{
  int i = 0;

  /* Check if buffer is large enough for data */
  if (dataLen > LEG_BUFFERSIZE)
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
    txLegBuf.wrI = 0;
    txLegBuf.data[i] = *(dataPtr + i);
    i++;
  }

  /* Increment pending byte counter */
  txLegBuf.pendingBytes = dataLen;

  /* Enable interrupt on USART TX Buffer*/
  USART_IntEnable(uart, USART_IF_TXBL);//使能发送中断,当发送缓冲器半满状态或空状态产生中断
}

void UartLeg_Transmit_Packet(unsigned char* buf,unsigned int length)//发送数据包
{
   uartLegPutData(buf,length);
}

unsigned char UartLeg_GetRXStatus(void)//小腿正常接收到一个数据包字节
{
  return((unsigned char)bRXOK_Leg);
}

void UartLeg_ClearRXStatus(void)
{
  bRXOK_Leg = 0;
}

unsigned short UartLeg_GetAngle(void)
{
    unsigned short angle;
    angle = ucLegRXBuffer[1];
    angle = angle << 7;
    angle |= ucLegRXBuffer[0];
    return(angle);
}
unsigned char UartLeg_GetFlexStatus(void)//地址 3小腿状态,腿伸缩状态,接近传感,.气阀工作
{
    unsigned char status;
    status = ucLegRXBuffer[2];
    status &= (~BIT7);
    return((unsigned char)status);
}
unsigned char UartLeg_GetLegStatus(void)// 小腿控制板上传复位状态， 地址 3 复位完成状态, 0x55：复位结束 0x5a:  复位中 ,非0x55和0x5a的0-0x7f之间的数据 则为故障代码
{
    unsigned char status;
    status = ucLegRXBuffer[3];
    status &= (~BIT7);
    return((unsigned char)status);
}