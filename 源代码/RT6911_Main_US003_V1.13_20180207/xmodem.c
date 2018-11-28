

#include <stdio.h>
#include <stdbool.h>
#include "string.h"
#include "BlueTooth.h"
#include "HandUart.h"
#include "efm32.h"
#include "em_chip.h"
#include "em_gpio.h"
#include "em_cmu.h"
#include "em_usart.h"
#include "efm32_types.h"
#include "efm32_def.h"
#include "ControlBox.h"

#include "xmodem.h"
#include "timer.h"
#include "memory.h"
/*bluetooh修改接收中断函数，增加XOMDEM文件，覆盖 MEMORY文件


meomory文件需要增加函数与main_update配合*/

static bool  bCloud_BlueToothRXOK=0;
//static bool  bColud_BlueTooth_5msFlag;
static bool  bColud_BlueTooth_Busy;
static unsigned char by_lingo=0;
static unsigned char by_command=0;
static bool bCloud_Xmodem_Start_Rec=0;
unsigned char by_Cloud_Xmodem_Rec_Data[136];//[128];
unsigned int Rec_Xmodem_index;
bool  bbCloud_Xmodem_Start_Rec_ok=0;
unsigned char by_Cloud_checksum;
//static bool bCloud_50msFlag=0;
unsigned int w_Cloud_Checksum=0;
unsigned short w_Cloud_progarm_ID;
#define bCloud_Packet_Len    133
#define BLUETOOTH_LEN    8



#define LINGO_BY_BLUETOOTH       0x10
#define LINGO_EXIT_BLUETOOTH     0x11
#define CLOUD_DOWNLOAD_CMD       0x01
#define CLOUD_DELETE_CMD         0x02




//unsigned short cal_test;



//unsigned int  BuleTooth_Cloud_Get_Checksum(void)
//{
  
//return w_Cloud_Checksum;


//}

//void BlueTooth_Cloud_clr_Checksum(void)
//{
//  w_Cloud_Checksum=0;
  
//}

//void BuleTooth_Cloud_5ms_Int(void)
//{
//bCloud_BlueTooth_5msFlag=1;
//}
unsigned char BlueToothCloud_GetRXStatus(void)
{
  return((unsigned char)bCloud_BlueToothRXOK);
  
}
void Cloud_Bluetooth_command_mode(void)
{
  
bCloud_Xmodem_Start_Rec=0;

}


void BlueToothCloud_Clr_RXStatus(void)
{
  bCloud_BlueToothRXOK=0;
}
/*unsigned char BuleTooth_Cloud_Get5msFlag(void)
{
  
  return ((unsigned char)bCloud_BlueTooth_5msFlag);
  
}*/
/*
void BuleTooth_Cloud_Clr_5msFlag(void)
{
  
  bCloud_BlueTooth_5msFlag=0;
  
}*/

unsigned char BlueTooth_Cloud_Get_LINGO(void)
{
    return((unsigned char)by_lingo);
}


unsigned char BlueTooth_Cloud_Get_COMMAND(void)
{
    return((unsigned char)by_command);
}


unsigned short BlueTooth_Cloud_Get_PROG_ID(void)
{
    return((unsigned short)w_Cloud_progarm_ID);
}
  

unsigned char BuleTooth_Cloud_GetBusyFlag(void)
{
  
  return ((unsigned char)bColud_BlueTooth_Busy);
  
}

//void BlueTooth_50ms_Int(void)
//{
  
  //bCloud_50msFlag=TRUE;
  
  
//}



unsigned char BlueTooth_Cloud_checksum(unsigned char*PDATA)
{
  unsigned char checkSum,i;

     checkSum=0;
     for(i=1;i<BLUETOOTH_LEN-2;i++)
      {
          checkSum += (*(PDATA+i));
                  
      }
      checkSum = ~checkSum;
      checkSum &= 0x7f;
      if(checkSum == (*(PDATA+BLUETOOTH_LEN-2)))//ucBlueToothRXBuffer[BlueToothRX_Index-2])
       {
         
                  by_lingo = *(PDATA+2);
                  by_command=*(PDATA+3);
         
                  w_Cloud_progarm_ID=0;
                  w_Cloud_progarm_ID = (*(PDATA+4)<<8);
 
                  w_Cloud_progarm_ID |= *(PDATA+5);
                  bCloud_BlueToothRXOK=1;
                 return 1;
                 
       }
      return 0;
  
}


 unsigned char BuleTooth_Cloud_rxByte(void)
{
 // unsigned int  timer = 2000000;
 // while (!(BlueTooth_UART->STATUS & USART_STATUS_RXDATAV) && --timer ) ;
 // if (timer > 0)
 // {
 //   return(( unsigned char)(BlueTooth_UART->RXDATA & 0xFF));
  //}
  //else
  //{
  //  return 0;
  //}
  
  while (!(BlueTooth_UART->STATUS & USART_STATUS_RXDATAV));
  return(( unsigned char)(BlueTooth_UART->RXDATA & 0xFF));   
  
  
 //return Xmodem_data;
  
}


int BuleTooth_Cloud_txByte(unsigned char data)
   
{

  while (!(BlueTooth_UART->STATUS & USART_STATUS_TXBL)) ;

  BlueTooth_UART->TXDATA = (uint32_t) data;
  return (int) data;
}





 unsigned short CRC_calc(unsigned char  *start, unsigned char *end)
{
  unsigned short crc = 0x0;
  unsigned char  *data;

  for (data = start; data < end; data++)
  {
    crc  = (crc >> 8) | (crc << 8);
    crc ^= *data;
    crc ^= (crc & 0xff) >> 4;
    crc ^= crc << 12;
    crc ^= (crc & 0xff) << 5;
  }
  return crc;
}



 int Cloud_verifyPacketChecksum(unsigned char *pkt, int sequenceNumber,unsigned char by_checksum)
 {
   
   unsigned short packetCRC;
   unsigned short calculatedCRC;



  if( (*(pkt+1)) + (*(pkt+2)) != 255)//数据包编号校验出错,检查packetNumber，packetNumberC是否正确 ,确认信息包序号的完整性
  {

    
       return -1;
  }


  if (*(pkt+1) != (sequenceNumber % 256))//检查信息包序号是否是应该接收的(期望的)信息包序列号，因为每接收一个信息包，sequenceNumber++，
  {

    
       return -1; //将sequenceNumber与接收的信息包pkt->packetNumber进行比较
  }


  
  
  
  calculatedCRC = CRC_calc(pkt+3, pkt+131);
  packetCRC    = (*(pkt+131)) << 8 | (*(pkt+132));

//  if(by_checksum != pkt->checksum)
 // {
 //   return -1;
  //}//
  
//  cal_test=calculatedCRC;
  
  
  
  if (calculatedCRC != packetCRC)
  {
    return -1;
  }

  

  
  

  return 0; 
   
   
   
   
 }


//bCloud_Xmodem_Start_Rec
unsigned char Cloud_Xmodem_Start_Rec_status(void)
{
  
    return bCloud_Xmodem_Start_Rec;
    
}


 void Cloud_Xmodem_Rec_packet(unsigned char rxData)
 {
   
       by_Cloud_Xmodem_Rec_Data[Rec_Xmodem_index++]=rxData;

       if(Rec_Xmodem_index>=bCloud_Packet_Len) 
       {                 
         bbCloud_Xmodem_Start_Rec_ok=1;
         bCloud_Xmodem_Start_Rec=0;         
         
       }
   

 }



int BlueTooth_Cloud_Xmodem(unsigned int baseAddress, unsigned int endAddress)
{
    unsigned int      sequenceNumber =1;
    unsigned int addr;
    
  
   for (addr = baseAddress; addr < endAddress; addr += Get_FlashPageSize())//efm32lg=2kbytes
   {
           Mem_Erase_Block(addr);
           Timer_Counter_Clear(1);
           while( Timer_Counter(1,1)==0);  
     
   }
            
   Timer_Counter_Clear(1);
   while( Timer_Counter(1,1)==0);

    Rec_Xmodem_index=0;
    bCloud_Xmodem_Start_Rec=1;//Rec_Xmodem_index
    BuleTooth_Cloud_txByte(XMODEM_NCG);
    
    bbCloud_Xmodem_Start_Rec_ok=0;
    Timer_Counter_Clear(1);

    sequenceNumber = 1;


  while (1)
  {
      if(Timer_Counter(1 + T_LOOP,10))
      {

          BuleTooth_Cloud_txByte(XMODEM_NCG);//(XMODEM_NCG);//不停发送 发送‘C’字符，直到接受缓冲器有字符可以接受

      }
   
      if(by_Cloud_Xmodem_Rec_Data[0]==XMODEM_SOH)

      {
        Timer_Counter_Clear(2);
        goto xmodem_transfer;
     
      }

   

   
  } 
   
xmodem_transfer:   
    while(1)
    {
      __no_operation();
      __no_operation();    
      __no_operation();
      __no_operation();  
      __no_operation();
      __no_operation();    
      __no_operation();
      __no_operation(); 
      if(by_Cloud_Xmodem_Rec_Data[0]==XMODEM_EOT)
      {   
      __no_operation();
      __no_operation();    
      __no_operation();
      __no_operation();  
      __no_operation();
      __no_operation();    
      __no_operation();
      __no_operation(); 
  
         break;
      }    

      
      
      
      if(by_Cloud_Xmodem_Rec_Data[0]!=XMODEM_SOH)
      {
            __no_operation();
            __no_operation();    
            __no_operation();
            __no_operation();  
            __no_operation();
            __no_operation();    
            __no_operation();
            __no_operation(); 
           BuleTooth_Cloud_txByte(XMODEM_CAN);

           return -1;
        
      }
      if(sequenceNumber>=65)//最大8K有64个128字节的数据包
      {
        
        break;
      }
      
      if(bbCloud_Xmodem_Start_Rec_ok)
      {         
        if(Cloud_verifyPacketChecksum(by_Cloud_Xmodem_Rec_Data, sequenceNumber,by_Cloud_checksum)==0)
         {
            MEM_Write_Block((baseAddress+(sequenceNumber - 1) * XMODEM_DATA_SIZE), &by_Cloud_Xmodem_Rec_Data[3] ,XMODEM_DATA_SIZE);
            __no_operation();
            __no_operation();    
            __no_operation();
            __no_operation();  
            __no_operation();
            __no_operation();    
            __no_operation();
            __no_operation(); 
            Timer_Counter_Clear(1);
            while( Timer_Counter(1,1)==0);   
            

            bbCloud_Xmodem_Start_Rec_ok=0;
            Rec_Xmodem_index=0;

            sequenceNumber++;                                                      
            BuleTooth_Cloud_txByte(XMODEM_ACK);
            bCloud_Xmodem_Start_Rec=1;
            Timer_Counter_Clear(2);
         }
         else
         {

            bbCloud_Xmodem_Start_Rec_ok=0;
            Rec_Xmodem_index=0;
            BuleTooth_Cloud_txByte(XMODEM_NAK);
            bCloud_Xmodem_Start_Rec=1;       
           Timer_Counter_Clear(2);
         }
               
      }
           
    }
    
  bCloud_Xmodem_Start_Rec=0;  
  bbCloud_Xmodem_Start_Rec_ok=0;
  Rec_Xmodem_index=0;
  return 0;
   
  
}
void xmodem__Erase_Block(unsigned int start_addr,unsigned int end_addr)
{
    MEM_Initial();
    Timer_Initial();
    for (unsigned int addr = start_addr; addr < end_addr; addr += Get_FlashPageSize())//efm32lg=2kbytes
    {
               Mem_Erase_Block(addr);
               Timer_Counter_Clear(1);
               while( Timer_Counter(1,1)==0);  
       
    }

}



void BuleTooth_xmodem_update(void)
{
  
  
    /*进入蓝牙更新网络程序时手控器不需做任何事*/
  unsigned char lingo,engStatus,command;//,CtrlType;
  unsigned char checksum=0;
  unsigned short bCloud_OutBufferCount;
  unsigned char bCloud_OutBuffer[12];
 // unsigned char by_Cloud_TimerCounter=0;
   unsigned char by_Update_Program_ID;    
    unsigned char overCounter=0;  
   //  unsigned char bHnad_OutBufferCount;
  //   unsigned char bHand_OutBuffer[8];
     unsigned short by_Cloud_progarm1_ID,by_Cloud_progarm2_ID,by_Cloud_progarm3_ID,by_Cloud_progarm4_ID;//网络程序ID号  
  //   unsigned short    by_Cloud_progarm1_ID_BUF,by_Cloud_progarm2_ID_BUF,by_Cloud_progarm3_ID_BUF,by_Cloud_progarm4_ID_BUF;
     bool status = true;
     bool BlueToothCloud_Rec_Success=0;     
     bool bUpdatekey=false;
     bool bExitUpdate=false;
     bool bDeleteUpdate=false;   
     //GlobalFlags11.nByte = 0;
    // bCloud_MasterSendPacket=0;
     MEM_Initial();
     //Power_All_Off();    
     BlueToothOn();   
     BlueToothEnterDataMode();//蓝牙进入到数据传输模式
     Cloud_Bluetooth_command_mode();
     //BlueTooth_Cloud_clr_Checksum();
     Timer_Initial();     
     engStatus = LINGO_BY_BLUETOOTH;     
     by_Cloud_progarm1_ID=0x0101;
     by_Cloud_progarm2_ID=0x0202;
     by_Cloud_progarm3_ID=0x0303;
     by_Cloud_progarm4_ID=0x0404;
     


     Timer_Counter_Clear(1);
     Timer_Counter_Clear(2);
     Timer_Counter_Clear(3);
     Timer_Counter_Clear(4);    
    while(status)
    {      
        if(BlueToothCloud_GetRXStatus())
        {
          BlueToothCloud_Rec_Success=1; 
          BlueToothCloud_Clr_RXStatus();          
        }                    
        if(BlueToothCloud_Rec_Success)
        {   
          lingo = BlueTooth_Cloud_Get_LINGO();//固定为0x10表示主板通过APP下载网络程序
          switch(lingo)
          {
              case LINGO_BY_BLUETOOTH: 
              {
                
                  engStatus = LINGO_BY_BLUETOOTH;
                  command =  BlueTooth_Cloud_Get_COMMAND();//当数据为0X01时表示APP 下载网络程序到主板中，当数据为0X02时表示APP 删除主板中已下载网络程序
                  switch(command)
                  {
                  case CLOUD_DOWNLOAD_CMD : //0X01
                    
                     if(BlueTooth_Cloud_Get_PROG_ID()==by_Cloud_progarm1_ID)
                     {
                          by_Update_Program_ID=1;
                          bUpdatekey=1;
                //          by_Cloud_progarm1_ID_BUF=by_Cloud_progarm1_ID;
                     }
                     else if(BlueTooth_Cloud_Get_PROG_ID()==by_Cloud_progarm2_ID)
                     {
                           by_Update_Program_ID=2;
                           bUpdatekey=1;
                  //         by_Cloud_progarm2_ID_BUF=by_Cloud_progarm2_ID;
                          
                     }                   
                     else if(BlueTooth_Cloud_Get_PROG_ID()==by_Cloud_progarm3_ID)
                     {
                           by_Update_Program_ID=3;
                           bUpdatekey=1;
                    //       by_Cloud_progarm3_ID_BUF=by_Cloud_progarm3_ID;
                          
                          
                     }                       
                     else if(BlueTooth_Cloud_Get_PROG_ID()==by_Cloud_progarm4_ID)
                     {
                            by_Update_Program_ID=4;                        
                            bUpdatekey=1;
                  //         by_Cloud_progarm4_ID_BUF=by_Cloud_progarm4_ID;
                     }   
                     
                     
                      break;
    
                  case CLOUD_DELETE_CMD ://0X02
                    
                     if(BlueTooth_Cloud_Get_PROG_ID()==by_Cloud_progarm1_ID)
                     {
                         by_Update_Program_ID=1;
                           bDeleteUpdate=TRUE;
                    //       by_Cloud_progarm1_ID_BUF=by_Cloud_progarm1_ID;
                     }
                     else if(BlueTooth_Cloud_Get_PROG_ID()==by_Cloud_progarm2_ID)
                     {
                         by_Update_Program_ID=2;
                           bDeleteUpdate=TRUE;
                    //       by_Cloud_progarm2_ID_BUF=by_Cloud_progarm2_ID;
                     }                    
                     
                     else   if(BlueTooth_Cloud_Get_PROG_ID()==by_Cloud_progarm3_ID)
                     {
                         by_Update_Program_ID=3;
                           bDeleteUpdate=TRUE;
                       //    by_Cloud_progarm3_ID_BUF=by_Cloud_progarm3_ID;
                     }                 
                     else if(BlueTooth_Cloud_Get_PROG_ID()==by_Cloud_progarm4_ID)
                     {
                            by_Update_Program_ID=4;
                            bDeleteUpdate=TRUE;
                       //     by_Cloud_progarm4_ID_BUF=by_Cloud_progarm4_ID;
                     }                  
                  
                  break;

  
                  default:break;
                  }        
              }
              
              break;
             case LINGO_EXIT_BLUETOOTH:
                 bExitUpdate=TRUE;
                 Timer_Counter_Clear(4);
              break;
              
            
                
             default:break;
          
          }//exit switch(lingo)
          BlueToothCloud_Rec_Success=0;
        }
      
        if(bUpdatekey)//初始化XMODEM传输
        {
          
         //  GPIO_PinModeSet(gpioPortF, 4, gpioModePushPull, 1);  //关闭蓝牙

              switch(by_Update_Program_ID)//(BlueTooth_Cloud_Get_PROG_ID())
              {
              case  1  ://by_Cloud_progarm1_ID:    //下载网络程序1+
   
                    if(BlueTooth_Cloud_Xmodem(CLOUD_PROGAME1_START_ADDRESS, CLOUD_PROGAME1_END_ADDRESS)==0)
                    {
                          unsigned int* pSum;
                          pSum = (unsigned int*)(CLOUD_PROGAME1_START_ADDRESS+CLUDE_AUTO_CHECKSUM_ADDRESS);//checksum addr
                          
                          unsigned int *pLenght;//
                          pLenght=(unsigned int*)(CLOUD_PROGAME1_START_ADDRESS+CLUDE_AUTO_SIZE_ADDRESS);//lenght addr             
                                 
                          unsigned int appCheckSum = 0;                          
                         for(unsigned int i=(CLOUD_PROGAME1_START_ADDRESS+CLUDE_AUTO_PROGRAM_OFFSET);i<(CLOUD_PROGAME1_START_ADDRESS+CLUDE_AUTO_PROGRAM_OFFSET+(*pLenght));i++)//               
                         {
                
                           appCheckSum += *(unsigned char*)i;//读取FLASH中的数据
                         }                   

                         if(*pSum == appCheckSum)
                         {                                                  
                          BuleTooth_Cloud_txByte(XMODEM_ACK);    
                           engStatus=LINGO_BY_BLUETOOTH;//       
                           bUpdatekey=0;
                           by_Update_Program_ID=0;
                         }
                         else  //发送效验和失败标志位上位机，上位机收到标志位后重新下传数据
                         {
                            bUpdatekey=0;
                           by_Update_Program_ID=0;
                           BuleTooth_Cloud_txByte(XMODEM_CAN);    
                           
                         }
                        
                          
                          
                          
                    }
                    else//发送出错终止传输
                    {
                       __no_operation();    
                       __no_operation();
                       __no_operation();  
                        bUpdatekey=0;
                       by_Update_Program_ID=0;
                      // BuleTooth_Cloud_txByte(XMODEM_CAN);                           
                    }
                      break;

              case 2:

                    if(BlueTooth_Cloud_Xmodem(CLOUD_PROGAME2_START_ADDRESS, CLOUD_PROGAME2_END_ADDRESS)==0)
                    {

                          unsigned int* pSum;
                          pSum = (unsigned int*)(CLOUD_PROGAME2_START_ADDRESS+CLUDE_AUTO_CHECKSUM_ADDRESS);//checksum addr
                          
                          unsigned int *pLenght;//
                          pLenght=(unsigned int*)(CLOUD_PROGAME2_START_ADDRESS+CLUDE_AUTO_SIZE_ADDRESS);//lenght addr                                                   
                          unsigned int appCheckSum = 0;  
                                                   
                        for(unsigned int i=(CLOUD_PROGAME2_START_ADDRESS+CLUDE_AUTO_PROGRAM_OFFSET);i<(CLOUD_PROGAME2_START_ADDRESS+CLUDE_AUTO_PROGRAM_OFFSET+(*pLenght));i++)//                         
                         {
                
                           appCheckSum += *(unsigned char*)i;//读取FLASH中的数据
                         }                         
                          
                         if(*pSum == appCheckSum)
                         {
                        
                           BuleTooth_Cloud_txByte(XMODEM_ACK);    
                           engStatus=LINGO_BY_BLUETOOTH;//       
                           bUpdatekey=0;
                           by_Update_Program_ID=0;
                         }
                         else  //发送效验和失败标志位上位机，上位机收到标志位后重新下传数据
                         {
                            bUpdatekey=0;
                           by_Update_Program_ID=0;
                           BuleTooth_Cloud_txByte(XMODEM_CAN);    
                           
                         }
                        
                          
                          
                          
                    }
                    else//发送出错终止传输
                    {
                       __no_operation();    
                       __no_operation();
                       __no_operation();  
                        bUpdatekey=0;
                       by_Update_Program_ID=0;
                      // BuleTooth_Cloud_txByte(XMODEM_CAN);                           
                    }      
                
                      break;
                  case 3:    
                    
                     if(BlueTooth_Cloud_Xmodem(CLOUD_PROGAME3_START_ADDRESS, CLOUD_PROGAME3_END_ADDRESS)==0)
                    {

                          unsigned int* pSum;
                          pSum = (unsigned int*)(CLOUD_PROGAME3_START_ADDRESS+CLUDE_AUTO_CHECKSUM_ADDRESS);//checksum addr
                          
                          unsigned int *pLenght;//
                          pLenght=(unsigned int*)(CLOUD_PROGAME3_START_ADDRESS+CLUDE_AUTO_SIZE_ADDRESS);//lenght addr                                                   
                          unsigned int appCheckSum = 0;
                          
                        for(unsigned int i=(CLOUD_PROGAME3_START_ADDRESS+CLUDE_AUTO_PROGRAM_OFFSET);i<(CLOUD_PROGAME3_START_ADDRESS+CLUDE_AUTO_PROGRAM_OFFSET+(*pLenght));i++)//   
                        
                         {
                
                           appCheckSum += *(unsigned char*)i;//读取FLASH中的数据
                         }
                          
                          
                         if(*pSum == appCheckSum)
                         {

                          BuleTooth_Cloud_txByte(XMODEM_ACK);    
                           engStatus=LINGO_BY_BLUETOOTH;//       
                           bUpdatekey=0;
                           by_Update_Program_ID=0;
                         }
                         else  //发送效验和失败标志位上位机，上位机收到标志位后重新下传数据
                         {
                            bUpdatekey=0;
                           by_Update_Program_ID=0;
                           BuleTooth_Cloud_txByte(XMODEM_CAN);    
                           
                         }
                        
                          
                          
                          
                    }
                    else//发送出错终止传输
                    {
                       __no_operation();    
                       __no_operation();
                       __no_operation();  
                        bUpdatekey=0;
                       by_Update_Program_ID=0;
                      // BuleTooth_Cloud_txByte(XMODEM_CAN);                           
                    }       
                    
                    break;
                    
                    
                  case 4:
                    if(BlueTooth_Cloud_Xmodem(CLOUD_PROGAME4_START_ADDRESS, CLOUD_PROGAME4_END_ADDRESS)==0)
                    {

                          unsigned int* pSum;
                          pSum = (unsigned int*)(CLOUD_PROGAME4_START_ADDRESS+CLUDE_AUTO_CHECKSUM_ADDRESS);//checksum addr
                          
                          unsigned int *pLenght;//
                          pLenght=(unsigned int*)(CLOUD_PROGAME4_START_ADDRESS+CLUDE_AUTO_SIZE_ADDRESS);//lenght addr                                                   
                          unsigned int appCheckSum = 0;
                          
                         for(unsigned int i=(CLOUD_PROGAME4_START_ADDRESS+CLUDE_AUTO_PROGRAM_OFFSET);i<(CLOUD_PROGAME4_START_ADDRESS+CLUDE_AUTO_PROGRAM_OFFSET+(*pLenght));i++)//   
                         {
                
                           appCheckSum += *(unsigned char*)i;//读取FLASH中的数据
                         }//*/

                         if(*pSum == appCheckSum)
                         {

                          BuleTooth_Cloud_txByte(XMODEM_ACK);    
                           engStatus=LINGO_BY_BLUETOOTH;//       
                           bUpdatekey=0;
                           by_Update_Program_ID=0;
                         }
                         else  //发送效验和失败标志位上位机，上位机收到标志位后重新下传数据
                         {
                            bUpdatekey=0;
                           by_Update_Program_ID=0;
                           BuleTooth_Cloud_txByte(XMODEM_CAN);    
                           
                         }
                          
                    }
                    else//发送出错终止传输
                    {
                       __no_operation();    
                       __no_operation();
                       __no_operation();  
                        bUpdatekey=0;
                       by_Update_Program_ID=0;
                      // BuleTooth_Cloud_txByte(XMODEM_CAN);                           
                    }     
                    
                    break;
                    
                  default:break;
              }

        }
        if(bDeleteUpdate)
        {
              switch(by_Update_Program_ID)
              {
                case 1:
                                                      
                 for (unsigned int addr = CLOUD_PROGAME1_START_ADDRESS; addr < CLOUD_PROGAME1_END_ADDRESS; addr += Get_FlashPageSize())//efm32lg=2kbytes
                   {
                        Mem_Erase_Block(addr);
                        Timer_Counter_Clear(1);
                        while( Timer_Counter(1,1)==0);  
       
                   }
                  bDeleteUpdate=0;
                  by_Update_Program_ID=0;
      

                     
                  break;
                
                case 2:
                  for (unsigned int addr = CLOUD_PROGAME2_START_ADDRESS; addr < CLOUD_PROGAME2_END_ADDRESS; addr += Get_FlashPageSize())//efm32lg=2kbytes
                   {
                        Mem_Erase_Block(addr);
                        Timer_Counter_Clear(1);
                        while( Timer_Counter(1,1)==0);  
       
                   }
                  bDeleteUpdate=0;
                  by_Update_Program_ID=0;
                  
            
                  
                  break;
                case 3:  
                  for (unsigned int addr = CLOUD_PROGAME3_START_ADDRESS; addr < CLOUD_PROGAME3_END_ADDRESS; addr += Get_FlashPageSize())//efm32lg=2kbytes
                   {
                        Mem_Erase_Block(addr);
                        Timer_Counter_Clear(1);
                        while( Timer_Counter(1,1)==0);  
       
                   }
                  bDeleteUpdate=0;
                  by_Update_Program_ID=0;
                  
            
                    
                    break;
                
                case 4:
                  for (unsigned int addr = CLOUD_PROGAME4_START_ADDRESS; addr < CLOUD_PROGAME4_END_ADDRESS; addr += Get_FlashPageSize())//efm32lg=2kbytes
                   {
                        Mem_Erase_Block(addr);
                        Timer_Counter_Clear(1);
                        while( Timer_Counter(1,1)==0);  
       
                   }
                  bDeleteUpdate=0;
                  by_Update_Program_ID=0;                   
         
                    break;
                    
                default:break;
                
              }                
        }
        
        if(bExitUpdate)//退出云更新程序
        {

                 if(Timer_Counter(T_LOOP+4,2))//  
                 { 
                        overCounter++;
                        if(overCounter >= 5)
                        {
                           
                            NVIC_SystemReset(); //复位CPU
                       }     
                 }                                  
        }

        
        switch(engStatus)
        {
          case LINGO_BY_BLUETOOTH:
            if(Timer_Counter(T_LOOP+2,2))//                
            {
                bCloud_OutBuffer[0] = SOI ;
                bCloud_OutBuffer[1] = PROGARM_CTRL ;
                
           //     bCloud_OutBuffer[3] = 0x7f & by_Cloud_progarm1_ID;//网络程序1 ID号的低7位
          //      bCloud_OutBuffer[2] = (by_Cloud_progarm1_ID>>8)& 0x7f;//网络程序1 ID号的高7位
                
                 bCloud_OutBuffer[3]=(*(unsigned char*)(CLOUD_PROGAME1_START_ADDRESS+CLUDE_AUTO_ID_ADDRESS))&0x7f;
                
                 bCloud_OutBuffer[2] = (*(unsigned char*)(CLOUD_PROGAME1_START_ADDRESS+CLUDE_AUTO_ID_ADDRESS+1))&0x7f;
                 if( bCloud_OutBuffer[2]==0x7f && bCloud_OutBuffer[3]==0x7f)
                 {
                   bCloud_OutBuffer[2]=0;
                   bCloud_OutBuffer[3]=0;
                 }
                 
                 
            
         //       bCloud_OutBuffer[5] = 0x7f & by_Cloud_progarm2_ID;//网络程序2 ID号的低7位
         //       bCloud_OutBuffer[4] = (by_Cloud_progarm2_ID>>8)& 0x7f;//网络程序1 ID号的高7位
                
                 bCloud_OutBuffer[5]= (*(unsigned char*)(CLOUD_PROGAME2_START_ADDRESS+CLUDE_AUTO_ID_ADDRESS))&0x7f;
                
                 bCloud_OutBuffer[4] = (*(unsigned char*)(CLOUD_PROGAME2_START_ADDRESS+CLUDE_AUTO_ID_ADDRESS+1))&0x7f;  
                 if( bCloud_OutBuffer[4]==0x7f && bCloud_OutBuffer[5]==0x7f)
                 {
                   bCloud_OutBuffer[4]=0;
                   bCloud_OutBuffer[5]=0;
                 }       
                 
                 
                 
                 

            //    bCloud_OutBuffer[7] = 0x7f & by_Cloud_progarm3_ID;//网络程序3 ID号的低7位
             //   bCloud_OutBuffer[6] = (by_Cloud_progarm3_ID>>8)& 0x7f;//网络程序1 ID号的高7位
                
                bCloud_OutBuffer[7]= (*(unsigned char*)(CLOUD_PROGAME3_START_ADDRESS+CLUDE_AUTO_ID_ADDRESS))&0x7f;
                
                 bCloud_OutBuffer[6] = (*(unsigned char*)(CLOUD_PROGAME3_START_ADDRESS+CLUDE_AUTO_ID_ADDRESS+1))&0x7f;        
                if( bCloud_OutBuffer[6]==0x7f && bCloud_OutBuffer[7]==0x7f)
                 {
                   bCloud_OutBuffer[6]=0;
                   bCloud_OutBuffer[7]=0;
                 }             
                 
                 
                 
                 
             //   bCloud_OutBuffer[9] = 0x7f & by_Cloud_progarm4_ID;//网络程序4 ID号的低7位
             //   bCloud_OutBuffer[8] = (by_Cloud_progarm4_ID>>8)& 0x7f;//网络程序1 ID号的高7位       
                 bCloud_OutBuffer[9]= (*(unsigned char*)(CLOUD_PROGAME4_START_ADDRESS+CLUDE_AUTO_ID_ADDRESS))&0x7f; 
                
                 bCloud_OutBuffer[8] = (*(unsigned char*)(CLOUD_PROGAME4_START_ADDRESS+CLUDE_AUTO_ID_ADDRESS+1))&0x7f;     
                 if( bCloud_OutBuffer[8]==0x7f && bCloud_OutBuffer[9]==0x7f)
                 {
                   bCloud_OutBuffer[8]=0;
                   bCloud_OutBuffer[9]=0;
                 }             
                 
                for(unsigned char i=1;i<9;i++)
                {
                checksum +=bCloud_OutBuffer[i];
                }
                checksum = ~checksum;
                checksum &= 0x7f;
                bCloud_OutBuffer[10]= checksum;
                bCloud_OutBuffer[11] = EOI ;
                bCloud_OutBufferCount = 12;

                BlueToothUart_Transmit_Packet(bCloud_OutBuffer, bCloud_OutBufferCount);
            }
            break;    
            
        case LINGO_EXIT_BLUETOOTH:
            if(Timer_Counter(T_LOOP+3,2))//  
            {

                bCloud_OutBuffer[0] = SOI ;
                bCloud_OutBuffer[1] = PROGARM_CTRL ;
                
                bCloud_OutBuffer[2] =LINGO_EXIT_BLUETOOTH;// 0x7f & by_Cloud_progarm1_ID;//网络程序1 ID号的低7位
                bCloud_OutBuffer[3] =0;// (by_Cloud_progarm1_ID>>8)& 0x7f;//网络程序1 ID号的高7位
                
            
                bCloud_OutBuffer[4] = 0;//0x7f & by_Cloud_progarm2_ID;//网络程序2 ID号的低7位
                bCloud_OutBuffer[5] = 0;//(by_Cloud_progarm2_ID>>8)& 0x7f;//网络程序1 ID号的高7位

         
                for(unsigned char i=1;i<6;i++)
                {
                checksum +=bCloud_OutBuffer[i];
                }
                checksum = ~checksum;
                checksum &= 0x7f;
                bCloud_OutBuffer[6]= checksum;
                bCloud_OutBuffer[7] = EOI ;
                bCloud_OutBufferCount = 8;

                BlueToothUart_Transmit_Packet(bCloud_OutBuffer, bCloud_OutBufferCount);

            }
           break;
                 
            default:break;
               
        }//exit  engStatus
    }    
      Cloud_Bluetooth_command_mode(); 
  }














//----------------------------------------------------------------------------
