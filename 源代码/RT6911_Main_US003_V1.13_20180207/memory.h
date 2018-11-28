




#ifndef __MEMORY_H__
#define __MEMORY_H__




#define MEMORY_LENGTH 13
//#define PRODUCT_ID_ADDR         ((uint32_t) (128*1024-5)) //((uint32_t) 0x0000FFFBUL) 
#define USER_DATA_BASE          ((uint32_t) 0x0FE00000UL)  /**< user data flash base address  */
//���µ�ַ�ǰ����ֽڴ��
#define SOFT_MAIN_VER_ADDRESS         0
#define SOFT_SECONDARY_VER_ADDRESS    1
#define SETTLE_ADDRESS                2  //��Ħ����Ƿ�λ ����Ϊ1��λ
#define AIRBAG_STRETCH_ADDRESS        3   //��Ħ���ڲ���������
#define SLIDE_MOTOR_ENABLE_ADDRESS    4   //�������ʹ�����ֹ
#define PROGRAM_ENABLE_ADDRESS        5   //���ʹ�ܵ�ַ
#define DEFAULT_TIME_ADDRESS          6   //����Ĭ��ʱ���ַ
#define BLUETOOTH_STATUS_ADDRESS      7   //��������״̬

#define DEMO_RUN_ON     0x5A

#define ACC_TIME_0_ADDRESS  0x10
#define ACC_TIME_1_ADDRESS  0x11
#define ACC_TIME_2_ADDRESS  0x12
#define ACC_TIME_3_ADDRESS  0x13

#define MEMORY_LENGTH_OF_BYTES      14
#define PROGRAM_FLAG               'p'
#define PROGRAM_BY_BLUETOOTH_FLAG  'l'
#define SOFT_MAIN_VER               1
//2.2 ���Ӹ�λʱ�����������ֹͣ
//   �����綯����С�ȵ��������ֶ�ʱΪ3A�ڸ�λʱΪ2A

#define SOFT_SECONDARY_VER      13  // 20150108��Ϊ1.02��
/*
V1.13 
3D Ĭ��Ϊ3����ʾ
�������ͣһ��
�����3��

*/

#define   ROCK_IDLE   0
#define   ROCK_MANL   1
#define   ROCK_AUTO   2

void xmodem(unsigned int start_addr,unsigned char *pw_Buffer ,int numBytes);

//����ģʽ�洢���ݶ���
/********�洢��ַ����********************
0: ��ʶID
1: ��ʶID
2: ����ǿ��
3: ����ǿ��
4: �ػ���λ
*************************************/ 
#define MEMORY_DEFAULT_AIR     0 //0,1,2  ���ҷ�Ϊ 0,1 ��2�����������������
#define MEMORY_DEFAULT_SETTLE  0 //0: ���н����ػ�����λ,1:���н����ػ���λ

#define SLIDE_DEFAULT_ENABLE  0// 1 //����״̬ 5Ϊ�г̿���


#define BLUETOOTH_STATUS_DEFAULT  1  //1 Ϊ�� 0Ϊ��
unsigned char ReadEEByte(unsigned int nAddress);
void MEM_Write_Memory(PUINT32 pw_Buffer,int numBytes);
void MEM_Read_Memory(PUINT32 pw_Buffer,int numBytes);


//-----------------------------------------------------------------
//�������������ƶ˸���
//extern uint32_t flashPageSize;
void Mem_Erase_Block(unsigned int start_addr);
void MEM_Write_Block(unsigned int start_addr,unsigned char *pw_Buffer ,int numBytes);
void MEM_Initial(void);

unsigned int Get_FlashPageSize(void);
//---------------------------------------------------------------------
#endif
