#ifndef __VALVE_H__
#define __VALVE_H__
#include "efm32_def.h"
#include "efm32_types.h"
#define PUMP_ON				0
#define PUMP_OFF			1
#define VALVE_ON			1
#define VALVE_OFF			0

#define VALVE_DISABLE 0
#define VALVE_ENABLE  1


//0-7λΪ�㲿���Һ�����
//0-7λΪ�㲿���Һ�����
#define FOOT1_DIS		0x00000000UL
#define FOOT1_CHR		0x00000001UL  //�ź��
#define FOOT0_DIS		0x00000000UL
#define FOOT0_CHR		0x00000002UL  //����
#define FOOT2_DIS		0x00000000UL
#define FOOT2_CHR		0x00000004UL   //����
#define LEG0_DIS		0x00000000UL
#define LEG0_CHR		0x00000008UL//С���ϵ�
#define LEG1_DIS		0x00000000UL  
#define LEG1_CHR		0x00000010UL//С���ϲ�
#define LEG2_DIS		0x00000000UL
#define LEG2_CHR		0x00000020UL //С���µ�
#define LEG3_DIS		0x00000000UL
#define LEG3_CHR		0x00000040UL//С���²�
//#define PUMP_DIS		0x00000000UL
//#define PUMP_ENA		0x00000080UL



//8-15λΪ�ֱ����Һ�����
//8-15λΪ�ֱ����Һ�����
#define RIGHT_ARM_UP_DIS	0x00000000UL
#define RIGHT_ARM_UP_CHR	0x00000100UL   //�ұ���
#define RIGHT_ARM_DOWN_DIS	0x00000000UL
#define RIGHT_ARM_DOWN_CHR	0x00000200UL   //�ұ���

#define LEFT_ARM_UP_DIS	        0x00000000UL
#define LEFT_ARM_UP_CHR	        0x00000400UL       //�����
#define LEFT_ARM_DOWN_DIS	0x00000000UL
//#define LEFT_ARM_DOWN_CHR	0x00000800UL   //�����
#define LEFT_ARM_DOWN_CHR	0x00000080UL   //�����


//#define RIGHT_KNECK_DIS	    0x00000000UL
//#define RIGHT_KNECK_CHR	    0x00001000UL  //�Ҿ�
//#define LEFT_KNECK_DIS	    0x00000000UL
//#define LEFT_KNECK_CHR	    0x00002000UL   //��
//#define PUMP_ARM_DIS		0x00000000UL
//#define PUMP_ARM_ENA		0x00008000UL
#define R_SHOLDER	        0x00020000UL      //�Ҽ�
#define L_SHOLDER	        0x00040000UL      //���
#define LEFT_SHOULDER_CHR       L_SHOLDER
#define RIGHT_SHOULDER_CHR      R_SHOLDER

//16-23λΪ�ϰ������Һ�����
//16-23λΪ�ϰ������Һ�����
//#define RIGHT_WAIST_DIS	    0x00000000UL
//#define RIGHT_WAIST_CHR	    0x00010000UL   //����
//#define LEFT_WAIST_DIS	    0x00000000UL
//#define LEFT_WAIST_CHR	    0x00020000UL   //���� 
//#define BUTTOCKS_DIS	    0x00000000UL
//#define BUTTOCKS_CHR	    0x00040000UL    //ƨ��

//#define BUTTOCKS_DIS	    0x00000000UL
//#define BUTTOCKS_CHR	    0x00040000UL    //ƨ��
#define SHOULDER_DIS	    0x00000000UL
#define SHOULDER_CHR	    0x00080000UL    //��
#define RIGHT_THIGH_DIS	    0x00000000UL
#define RIGHT_THIGH_CHR	    0x00100000UL   //�Ҵ���
#define LEFT_THIGH_DIS	    0x00000000UL
#define LEFT_THIGH_CHR	    0x00200000UL   //�����

//#define PUMP_BODY_DIS	    0x00000000UL
//#define PUMP_BODY_ENA	    0x80000000UL




/*
//0-20λΪ���Һ�����  ,����Ϊ������λ
#define F_L_SIDE		0x00000001UL        //��������
#define F_R_SIDE		0x00000002UL        //��������
#define F_HEEL		        0x00000004UL        //�ź������
#define LEG_LEFT                0x00000008UL        //����
#define LEG_RIGHT               0x00000010UL        //��������
#define R_ARM_1	                0x00000020UL        //�Ҹ첲1
#define R_ARM_2     	        0x00000040UL        //�Ҹ첲2
#define R_ARM_3	                0x00000080UL        //�Ҹ첲3
#define L_ARM_1	                0x00000100UL        ////��첲1
#define L_ARM_2	                0x00000200UL        ////��첲2
#define L_ARM_3	                0x00000400UL        //��첲3
#define LEFT_ARM_1_CHR    L_ARM_1
#define LEFT_ARM_2_CHR    L_ARM_2
#define LEFT_ARM_3_CHR    L_ARM_3
#define RIGHT_ARM_1_CHR   R_ARM_1
#define RIGHT_ARM_2_CHR   R_ARM_2
#define RIGHT_ARM_3_CHR   R_ARM_3
#define PE1	                0x00000800UL                            //�����ͱۼ�
#define PE_ARM                  PE1
#define PE2	                0x00001000UL                            //С�Ⱥʹ���
#define R_U_WAIST	        0x00002000UL      //���������� 
#define R_D_WAIST	        0x00004000UL      //����������
#define L_U_WAIST	        0x00008000UL      //����������
#define L_D_WAIST	        0x00010000UL      //��Ҫ������
#define R_SHOLDER	        0x00020000UL      //�Ҽ�
#define L_SHOLDER	        0x00040000UL      //���
#define LEFT_SHOULDER_CHR       L_SHOLDER
#define RIGHT_SHOULDER_CHR      R_SHOLDER
#define R_THIGH	                0x00080000UL      //�Ҵ���
#define L_THIGH	                0x00100000UL      //�����

*/


//21-24λΪ������ת��ʽ
/*  
0-1λΪ����ٶ� 00 ֹͣ 01 ���� 10 ���� 11 ����
2-3λΪ��귽ʽ 00������ת 01������ת 10 ҡ��
*/

#define KNEAD_LEG_STOP        (0x00<<21)//0b00000000   
#define KNEAD_SLOW_IN         (0x01<<21)//0b00000001   
#define KNEAD_SLOW_OUT        (0x05<<21)//0b00000101   
#define KNEAD_SLOW_SWAY       (0x09<<21)//0b00001001   
#define KNEAD_MID_IN          (0x02<<21)//0b00000010   
#define KNEAD_MID_OUT         (0x06<<21)//0b00000110   
#define KNEAD_MID_SWAY        (0x0A<<21)//0b00001010   
#define KNEAD_FAST_IN         (0x03<<21)//0b00000011   
#define KNEAD_FAST_OUT        (0x07<<21)//0b00000111   
#define KNEAD_FAST_SWAY       (0x0B<<21)//0b00001011   

//25-26λΪ������ת�ٶ�
/*0-1λΪ�����ٶ� 00 ֹͣ 01 ���� 10 ���� 11 ����*/
#define ROLLER_STOP	          (0x00<<25)
#define ROLLER_SLOW	          (0x01<<25)
#define ROLLER_MID	          (0x02<<25)
#define ROLLER_FAST	          (0x03<<25)
//27-29λΪ������ת��ʽ
/*  
000 ����������ת
001 ����������ת
010 �̼�Ъ����
011 �̼�Ъ����
100 ����Ъ����
101 ����Ъ����
110 ���г̴��
111 ���г̴��
*/
#define ROLLER_CON_IN	          (0x00<<27)
#define ROLLER_CON_OUT	          (0x01<<27)
#define ROLLER_S_INT_IN	          (0x02<<27)
#define ROLLER_S_INT_OUT	  (0x03<<27)
#define ROLLER_L_INT_IN	          (0x04<<27)
#define ROLLER_L_INT_OUT	  (0x05<<27)
#define ROLLER_S_RUB	          (0x06<<27)
#define ROLLER_L_RUB	          (0x07<<27)
//#define ROLLER_PHASE	  0x10000000//0b00010000

#define ROLLER_INTERMITTENT_TIME      40
#define ROLLER_INTERMITTENT_ON_TIME   10
#define ROLLER_SEMI_CIRCLE_TIME       80
#define ROLLER_SEMI_CIRCLE_ON_TIME    30
//28-31λΪ������ת��ʽ
#define STRETCH_STOP		0x00000000  //0b0000 0000 ����
#define STRETCH_UP		0x10000000  //0b0001 0000�������ɣ�С��������һֱ��С����������ߵ�
#define STRETCH_DOWN		0x20000000  //0b0010 0000����������С���½���һֱ��С���½�����͵� 
#define STRETCH_RESET   	0x30000000  //0b0011 0000�����ص�һ���Ƕȣ�С�Ȳ��� 

#define ALL_DIS			0x00000000

//#define VALVE_POWER_PORT      gpioPortF
//#define VALVE_POWER_BIT       3
//#define VALVE_POWER_MODE      gpioModePushPull

#define VALVE_LOAD_PORT              gpioPortC   //165 load
#define VALVE_LOAD_BIT               9
#define VALVE_LOAD_MODE              gpioModePushPull

#define VALVE_CLK_PORT               gpioPortE
#define VALVE_CLK_BIT                5
#define VALVE_CLK_MODE               gpioModePushPull

#define VALVE_LATCH_PORT             gpioPortE   //DRV8804 latch 
#define VALVE_LATCH_BIT              4
#define VALVE_LATCH_MODE             gpioModePushPull

#define VALVE_DATA_PORT              gpioPortE   //drv8804 serial data in SPI MOSI
#define VALVE_DATA_BIT               7
#define VALVE_DATA_MODE              gpioModePushPull

#define VALVE_DATA_IN_PORT           gpioPortE   //SPI MISO
#define VALVE_DATA_IN_BIT            6
#define VALVE_DATA_IN_MODE           gpioModeInput

#define VALVE_SPI                    USART0
#define VALVE_SPI_ROUTE_LOCAITON     USART_ROUTE_LOCATION_LOC1
#define VALVE_CMU_SPI                cmuClock_USART0     

#define VALVE_AIRPUMP1_PORT          gpioPortB   //�ϰ������� �ұ�һ������ 
#define VALVE_AIRPUMP1_BIT           14
#define VALVE_AIRPUMP1_MODE          gpioModePushPull

#define VALVE_AIRPUMP2_PORT          gpioPortB   //�°������� �м�һ������
#define VALVE_AIRPUMP2_BIT           13
#define VALVE_AIRPUMP2_MODE          gpioModePushPull

#define VALVE_AIRPUMP3_PORT          gpioPortD    // ���һ������
#define VALVE_AIRPUMP3_BIT           8
#define VALVE_AIRPUMP3_MODE          gpioModePushPull

#define VALVE_OZON_PORT          gpioPortE    // ������
#define VALVE_OZON_BIT           3
#define VALVE_OZON_MODE          gpioModePushPull


/*
typedef union
{
	struct
	{
		unsigned bD0:1 ;
		unsigned bD1:1 ;
		unsigned bD2:1 ;
		unsigned bD3:1 ;
		unsigned bD4:1 ;
		unsigned bD5:1 ;
		unsigned bD6:1 ;
		unsigned bD7:1 ;
	} ;
	unsigned char nByte ;
}BITS ;*/




//ͨ�������� 8600Sʹ����19��������2��������ƣ�
extern BITS BITS_ValveData[3] ;

////////////////////////////////////////////////////////////////////////////////////////////
/*
//����λ�������� cn31  ,                      �㲿����=CN27=AirPumpSwitch3
#define bValveData2NC0		        BITS_ValveData[0].bD3   //         V16     bag3
#define bFootHeelAirBagValve		BITS_ValveData[0].bD2   //�ź��   V17     bag2
#define bLeftFootAirBagValve 		BITS_ValveData[0].bD1   //�����   V18     bag1
#define bRightFootAirBagValve		BITS_ValveData[0].bD0   //���Ҳ�   V19     bag0
#define bLegDownBottomAirBagValve	BITS_ValveData[0].bD7   //С���µ� V20     bag7
#define bLegDownSideAirBagValve		BITS_ValveData[0].bD6   //С���²� V21     bag6
#define bLegUpBottomAirBagValve		BITS_ValveData[0].bD5   //С���ϵ� V22     bag5
#define bLegUpSideAirBagValve		BITS_ValveData[0].bD4   //С���ϲ� V23     bag4




//����λ��������   cn28=�ϰ�������         =�ϰ�������= =CN29=AirPumpSwitch2
//#define bLeftWaistAirBagValve  	        BITS_ValveData[0].bD1  //����  V10
//#define bRightWaistAirBagValve 	        BITS_ValveData[0].bD0  //����  V11

//#define bButtocksAirBagValve	    BITS_ValveData[0].bD6  //ƨ��      V13

#define bLeftArmUpAirBagVave  		BITS_ValveData[1].bD0   //V4    ���ֱ����� 
#define bLeftArmDownAirBagValve		BITS_ValveData[1].bD1   //V5    ���ֱ����� 
#define bLeftThighAirBagValve           BITS_ValveData[1].bD2  //�����    V14
#define bRightThighAirBagValve          BITS_ValveData[1].bD3  // �Ҵ���   V15  
#define bRightArmUpAirBagValve    	BITS_ValveData[1].bD4   //V6    ���ֱ����� 
#define bRightArmDownAirBagValve  	BITS_ValveData[1].bD5   //V7    ���ֱ�����  
//#define bLeftNeckAirBagValve		BITS_ValveData[0].bD6   //V2    ��                                     
//#define bRightNeckAirBagValve   	BITS_ValveData[0].bD7   //V3    �Ҿ�   
  //#define bRightSholderAirBagValve  	  BITS_ValveData[0].bD6
  //#define bLeftSholderAirBagValve 	  BITS_ValveData[0].bD7     
#define bSholderAirBagValve         BITS_ValveData[1].bD7  //�粿      V12    


*/
//����λ�������� cn31  ,                      �㲿����=CN27=AirPumpSwitch3



#define bLeftArmDownAirBagValve		BITS_ValveData[0].bD0   //V5    ���ֱ����� 
#define bLeftArmUpAirBagVave  		BITS_ValveData[0].bD1   //V4    ���ֱ����� 
#define bRightThighAirBagValve          BITS_ValveData[0].bD2  // �Ҵ���   V15 
#define bLeftThighAirBagValve           BITS_ValveData[0].bD3 //�����    V14
#define bRightArmUpAirBagValve    	BITS_ValveData[0].bD4   //V6    ���ֱ����� 
#define bRightArmDownAirBagValve  	BITS_ValveData[0].bD5   //V7    ���ֱ�����
#define bRightSholderAirBagValve  	BITS_ValveData[0].bD6
#define bLeftSholderAirBagValve 	BITS_ValveData[0].bD7


	


//����λ��������   cn28=�ϰ�������         =�ϰ�������= =CN29=AirPumpSwitch2
#define bValveData2NC0		        BITS_ValveData[1].bD3   //         V16     bag3
#define bRightFootAirBagValve	        BITS_ValveData[1].bD2   //�ź��   V17     bag2
#define bFootHeelAirBagValve		BITS_ValveData[1].bD1   //�����   V18     bag1
#define bLeftFootAirBagValve            BITS_ValveData[1].bD0   //���Ҳ�   V19     bag0
#define bLegDownBottomAirBagValve	BITS_ValveData[1].bD7   //С���µ� V20     bag7
#define bLegDownSideAirBagValve		BITS_ValveData[1].bD6   //С���²� V21     bag6
#define bLegUpBottomAirBagValve		BITS_ValveData[1].bD5   //С���ϵ� V22     bag5
#define bLegUpSideAirBagValve		BITS_ValveData[1].bD4   //С���ϲ� V23     bag4
/*
//data ����λ�������� cn26                 �ֱ�����=CN30=AirPumpSwitch1
#define bValveData1NC1 		        BITS_ValveData[0].bD0  //��    V9
#define bLeftWaistAirBagValve  	        BITS_ValveData[0].bD1  //����  V10
#define bRightWaistAirBagValve 	        BITS_ValveData[0].bD2  //����  V11
#define bValveData3NC0		        BITS_ValveData[0].bD3   //v0
#define bValveData3NC1 	                BITS_ValveData[0].bD4   //v1
#define bLeftNeckAirBagValve		BITS_ValveData[0].bD5   //V2    ��                                     
#define bRightNeckAirBagValve   	BITS_ValveData[0].bD6   //V3    �Ҿ�       
#define bButtocksAirBagValve	        BITS_ValveData[0].bD7  //ƨ��      V13
*/
 
struct AirBagStruct
{
    UINT32 nPumpValveState ;//���ú�������״̬
    unsigned char nKeepTime1 ;//��ǰ״̬����ʱ��,��Ӧ������
    unsigned char nKeepTime2 ;//��ǰ״̬����ʱ��,��Ӧ������
    unsigned char nKeepTime3 ;//��ǰ״̬����ʱ��,��Ӧǿ����
};

#define STRETCH_MODE_TIME   1 //����ģʽ 1Ϊʱ�����
#define STRETCH_MODE_SWITCH 0 //����ģʽΪ�г̿���

typedef struct
{
    unsigned char timer;        //���˳����ʱ��ʱ������λ0.1s
    unsigned char step ;        //���˳�����
    unsigned char bBackLegFlag; //���˳����е綯�׵�״̬
    unsigned char active;
    unsigned char init;
    unsigned char times;        //���˳���ѭ������
    unsigned char mode;         //����ģʽ 1Ϊʱ����� 0Ϊ�г̿���
    unsigned char PresetTime;   //����ģʽΪʱ�����ʱ��Ԥ��ʱ�䣬��λ0.1��
}StretchStruct;

typedef struct
{
    unsigned char time;        //���˳���ִ��ʱ��
    unsigned char times;       //һ���غϵ����˴��� һ��Ϊ3��
    unsigned char mode;        //����ģʽ STRETCH_GO_OUT��ǰ�� STRETCH_GO_DOWN������
}StretchProgramStruct;

#define C_Stretch_Up    1
#define C_Stretch_Stop  2
#define C_STRETCH_HOLD_TIME   30 //��λ0.1s
#define C_STRETCH_RESET_TIME  100 //��λ0.1s
#define C_STRETCH_CHARGE_TIME 100 //60//��λ0.1s

#define C_STRETCH_CHARGE_TIME_OUT 60 //��λ0.1s


#define VALVE_USART_INITSYNC                                                                  \
    {                                                                                             \
        usartEnable,       /* Enable RX/TX when init completed. */                                \
        0,                 /* Use current configured reference clock for configuring baudrate. */ \
        1000000,           /* 1 Mbits/s. */                                                       \
        usartDatabits8,    /* 8 databits. */                                                      \
        true,              /* Master mode. */                                                     \
        true,              /* Send least significant bit first. */                                 \
        usartClockMode0    /* Clock idle low, sample on rising edge. */                           \
    }

//���涨��8�������˶��������

/*
#define AIRBAG_LOCATE_BACK_WAIST     209//����
#define AIRBAG_LOCATE_ARM_SHOLDER     210//�粿
#define AIRBAG_LOCATE_SEAT            211//����


#define AIRBAG_LOCATE_ARM             212//�۲�   ok

#define AIRBAG_LOCATE_ARM_SHOLDER_WAIST  213
#define AIRBAG_LOCATE_LEG_FOOT_SEAT      214
*/
 /*   //0:�ر�
    //1:�Ƚ�
    //2:����
    //3:�ۼ�
    //4:����
    //5:ȫ��
nAirBagModeStore = ((ucRXBuffer[12]>>2) & 0x07);*/

//#define AIRBAG_LOCATE_BODY_UP         2//0x02   //ok
#define AIRBAG_LOCATE_NONE            0
#define AIRBAG_LOCATE_LEG_FOOT        0x01//0x01   ////�Ƚ�  ok
//#define AIRBAG_LOCATE_BACK_WAIST      0x02//����
#define AIRBAG_LOCATE_ARM_NECK        0x04//3//0x03     //�۾�  //20170205 WGH
#define AIRBAG_LOCATE_SEAT            0x08//4//����             //20170205 WGH
#define AIRBAG_LOCATE_AUTO            0x10//5//0x04            //20170205 WGH
//#define AIRBAG_LOCATE_NECK            6//0x04  //20170205 WGH
//add by wgh 20170208



typedef struct
{
    unsigned char init; 
    unsigned char active; 
    unsigned char nCurAirBagStep;
    unsigned char nCurKeepTime1;
    unsigned char nCurKeepTime2; 
    unsigned char nCurKeepTime3; 
    unsigned char nCurKeepTime4;
    unsigned char nCurKeepTime5; 
    const struct AirBagStruct * pAirBagArray;
    UINT32 nCurPumpValveState;
    UINT16 nTotalSteps;
    unsigned char nAirBagCounter ;
    unsigned char locate ;
}st_AirBag;                                               


struct WaveMotorStruct
{
    unsigned char speed;   //ҡ������ٶ� 0-3
    unsigned int  time;    //ҡ��������ʱ�� ��λ1sec  
};

extern unsigned char* pValveData;
extern unsigned char* pInputData;
void Valve_Initial_IO(void);
//void Valve_Send_Data(unsigned char * ucData,unsigned char ucLength);
//void Valve_Send_Data(unsigned char * ucSendData,unsigned char * ucReceiveData,unsigned char ucLength);
void Valve_Send_Data(void);
void Valve_5ms_Int(void);
void Valve_SetData(void);
void Valve_ClearData(void);
void Valve_SetClock(void);
void Valve_ClearClock(void);
void Valve_ClearLatch(void);
void Valve_SetLatch(void);




void Valve_AirPumpACPowerOn(void);
void Valve_AirPumpACPowerOff(void);




unsigned char Valve_GetAirBagStrength(void);
void Valve_SetAirBagStrength(unsigned char strength);
void Valve_AddAirBagStrength(void);

void Valve_FootRollerProce(unsigned char bRollerEnable,unsigned char Valve_Enable,st_AirBag* pBag);
void Valve_LegKneadProce(unsigned char bLegKneadEnable,unsigned char Valve_Enable,st_AirBag* pBag);
void Valve_SetRollerPWM(unsigned char level);

void Valve_SetStretchUp(void);
void Valve_SetStretchCharge(unsigned int start);
void Valve_SetStretchHold(void);

void Valve_Test_Set_Data(unsigned int ValveTestData);

void Valve_Control(unsigned char nAirBagSwitch,st_AirBag* pBag,unsigned char level);

unsigned char Valve_Level_Decrease(unsigned char by_Data);
unsigned char Valve_Level_Increase(unsigned char by_Data);

void Valve_SetEnableSholder(unsigned int enable);
void Valve_1ms_Int(void);
//unsigned char Valve_GetRollerLevel(void);
void Valve_SetBackMode(int backauto);
void Valve_Initial_Data(void);
void Valve_CloseAll(void);  

void  Valve_OzonOn(void);
void  Valve_OzonOff(void);
unsigned char Valve_GetRollerLevel(void);
//unsigned char Valve_GetRollerLevel(void);
void Valve_SetStretchChargeOut(unsigned int start);

int Valve_RollerIsAuto(void);

void Valve_SetStretchChargepre(unsigned int start);
void Valve_SetStretchCharge0(unsigned int start);
void Valve_SetStretchCharge1(unsigned int start);

void Valve_SetStretchCharge_FOOT(unsigned int start);
//void Valve_SetStretchCharge1(unsigned int start);
void Valve_SetStretchCharge_ARM(unsigned int start,unsigned char sstretch);
void Valve_ArmProce(unsigned char bArmEnable,unsigned char Valve_Enable,st_AirBag* pBag);
void Valve_SetStretchChargeSTEEL(unsigned int start);
void Valve_SetStretchCharge_FOOT_THIGH_LEG_SHOULD(unsigned int start);
void Valve_SetStretchCharge_FOOT_THIGH(unsigned int start);
void Valve_SetStretchCharge_FOOT_THIGH_SHOULD(unsigned int start);
//////////////
void  Valve_SetStretchChargeATOUT(unsigned int start);
void Valve_LeftHandSetStretchChargeOut(unsigned int start);//modify by taoqingosng
void Valve_RightHandSetStretchChargeOut(unsigned int start);//modify by taoqingosng
void Valve_SetStretchChargeATOUTFootHeelOFF(void);//�ҽţ��ͷ�С�Ⱥ� �㲿

void Valve_SetStretchCharge_FOOT_THIGH_LEG(unsigned int start);
#endif
