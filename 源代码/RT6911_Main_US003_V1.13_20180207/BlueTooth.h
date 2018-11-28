#ifndef __BLUETOOTH_H__
#define __BLUETOOTH_H__
#include "MassageStatus.h"
//#include <stdint.h>
//#include "em_usart.h"

#define BlueTooth_TX_PORT           gpioPortC
#define BlueTooth_TX_BIT            2
#define BlueTooth_TX_MODE           gpioModePushPull

#define BlueTooth_RX_PORT           gpioPortC
#define BlueTooth_RX_BIT            3
#define BlueTooth_RX_MODE           gpioModeInputPull

#define BlueTooth_IO1_PORT          gpioPortC 
#define BlueTooth_IO1_BIT           4
#define BlueTooth_IO1_MODE          gpioModePushPull

#define BlueTooth_IO2_PORT          gpioPortC
#define BlueTooth_IO2_BIT           5
#define BlueTooth_IO2_MODE          gpioModePushPull

#define BlueTooth_IO3_PORT          gpioPortB
#define BlueTooth_IO3_BIT           6
#define BlueTooth_IO3_MODE          gpioModePushPull

#define BlueTooth_Volum_PORT          gpioPortD
#define BlueTooth_Volum_BIT           0
#define BlueTooth_Volum_MODE          gpioModeInput

//BlueTooth's Switch Port=�������翪�� =0=����������
#define BlueTooth_MUTE_PORT         gpioPortE
#define BlueTooth_MUTE_BIT          11
#define BlueTooth_MUTE_MODE         gpioModePushPull
#define BlueTooth_MUTE_INIT_ON      0

#define BlueTooth_UART              USART2
#define BlueTooth_CLK               cmuClock_USART2
#define BlueTooth_RX                USART_Rx
#define BlueTooth_LOCATION          USART_ROUTE_LOCATION_LOC0

#define DMAUART_IRQn              USART2_RX_IRQn

#define BlueTooth_SOI                0XF0
#define BlueTooth_EOI                0XF1
//Master->Slave Packet Type
#define PACKET_MASTER_GET_COMMAND 0x00
#define PACKET_MASTER_ACK_COMMAND 0x01
#define PACKET_MASTER_SET_STATE   0x02
//Slave->Master Packet Type
#define PACKET_SLAVE_SEND_COMMAND 0x03

//BlueTooth'mute pin State
#define BlueTooth_Speak_Out_On          0
#define BlueTooth_Speak_Out_Off         1
#define BlueTooth_MutePin_Value         0xef    //������λ���������Buffer�ĵ�4λ����

void BlueToothUart_Initial_IO(void);
void BlueToothUart_Transmit_Packet(unsigned char* buf,unsigned int length);
unsigned char BlueToothUart_GetRXStatus(void);
unsigned char BlueToothUart_GetKey(void);
void BlueToothUart_SetKey(unsigned char by_Data);
void BlueToothUart_ClearRXStatus(void);
unsigned int BlueToothUart_GetCtrlType(void);
unsigned char BlueToothMuteState(void);
void BlueToothOff(void);
void BlueToothOn(void);

void BlueToothUart_GetModlueName(unsigned char* str);
void BlueToothUart_10ms_Int(void);


void BlueToothUart_AMP_Volume_On(void);
void BlueToothEnterDataMode(void);
//unsigned int  BuleTooth_Cloud_Get_Checksum(void);
void BlueTooth_Cloud_clr_Checksum(void);

/*
//-------------------------------------------------------------------------------------
//------------------���������ƶ˸����Զ�����

#define XMODEM_SOH                1       //0x01   ���⿪ʼ  start of heading
#define XMODEM_EOT                4       //0x04     end of transmission  �������
#define XMODEM_ACK                6       //0x06 acknowledge  �յ�֪ͨ
#define XMODEM_NAK                21      //0x15 negative acknowlege �ܾ�����
#define XMODEM_CAN                24      //0x18  cancel ȡ��
#define XMODEM_NCG                67      //0x43  ��д��ĸC

#define XMODEM_DATA_SIZE          128



void BuleTooth_Cloud_5ms_Int(void);
unsigned char BlueToothCloud_GetRXStatus(void);
unsigned char BuleTooth_Cloud_Get5msFlag(void);
unsigned char BuleTooth_Cloud_GetBusyFlag(void);
void BuleTooth_Cloud_Clr_5msFlag(void);
void BlueToothCloud_Clr_RXStatus(void);

unsigned char BlueTooth_Cloud_checksum(void);

unsigned char BlueTooth_Cloud_Get_LINGO(void);
unsigned char BlueTooth_Cloud_Get_COMMAND(void);
unsigned short BlueTooth_Cloud_Get_PROG_ID(void);
unsigned char BuleTooth_Cloud_rxByte(void);

int BuleTooth_Cloud_txByte(unsigned char data);

int BlueTooth_Cloud_Xmodem(unsigned int baseAddress, unsigned int endAddress);

int Cloud_verifyPacketChecksum(unsigned char *pkt, int sequenceNumber,unsigned char by_checksum);
 
unsigned short CRC_calc(unsigned char  *start, unsigned char *end);
void BlueTooth_50ms_Int(void);

void Cloud_Bluetooth_command_mode(void);
//------------------------------------------------------------------------------------------------
*/
 
#endif // __BLUETOOTH_H__