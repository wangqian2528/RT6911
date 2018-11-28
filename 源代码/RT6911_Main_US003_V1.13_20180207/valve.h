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


//0-7位为足部气囊和气泵
//0-7位为足部气囊和气泵
#define FOOT1_DIS		0x00000000UL
#define FOOT1_CHR		0x00000001UL  //脚后跟
#define FOOT0_DIS		0x00000000UL
#define FOOT0_CHR		0x00000002UL  //足左
#define FOOT2_DIS		0x00000000UL
#define FOOT2_CHR		0x00000004UL   //足右
#define LEG0_DIS		0x00000000UL
#define LEG0_CHR		0x00000008UL//小腿上底
#define LEG1_DIS		0x00000000UL  
#define LEG1_CHR		0x00000010UL//小腿上侧
#define LEG2_DIS		0x00000000UL
#define LEG2_CHR		0x00000020UL //小腿下底
#define LEG3_DIS		0x00000000UL
#define LEG3_CHR		0x00000040UL//小腿下侧
//#define PUMP_DIS		0x00000000UL
//#define PUMP_ENA		0x00000080UL



//8-15位为手臂气囊和气泵
//8-15位为手臂气囊和气泵
#define RIGHT_ARM_UP_DIS	0x00000000UL
#define RIGHT_ARM_UP_CHR	0x00000100UL   //右臂上
#define RIGHT_ARM_DOWN_DIS	0x00000000UL
#define RIGHT_ARM_DOWN_CHR	0x00000200UL   //右臂下

#define LEFT_ARM_UP_DIS	        0x00000000UL
#define LEFT_ARM_UP_CHR	        0x00000400UL       //左臂上
#define LEFT_ARM_DOWN_DIS	0x00000000UL
//#define LEFT_ARM_DOWN_CHR	0x00000800UL   //左臂下
#define LEFT_ARM_DOWN_CHR	0x00000080UL   //左臂下


//#define RIGHT_KNECK_DIS	    0x00000000UL
//#define RIGHT_KNECK_CHR	    0x00001000UL  //右颈
//#define LEFT_KNECK_DIS	    0x00000000UL
//#define LEFT_KNECK_CHR	    0x00002000UL   //左颈
//#define PUMP_ARM_DIS		0x00000000UL
//#define PUMP_ARM_ENA		0x00008000UL
#define R_SHOLDER	        0x00020000UL      //右肩
#define L_SHOLDER	        0x00040000UL      //左肩
#define LEFT_SHOULDER_CHR       L_SHOLDER
#define RIGHT_SHOULDER_CHR      R_SHOLDER

//16-23位为上半身气囊和气泵
//16-23位为上半身气囊和气泵
//#define RIGHT_WAIST_DIS	    0x00000000UL
//#define RIGHT_WAIST_CHR	    0x00010000UL   //右腰
//#define LEFT_WAIST_DIS	    0x00000000UL
//#define LEFT_WAIST_CHR	    0x00020000UL   //左腰 
//#define BUTTOCKS_DIS	    0x00000000UL
//#define BUTTOCKS_CHR	    0x00040000UL    //屁股

//#define BUTTOCKS_DIS	    0x00000000UL
//#define BUTTOCKS_CHR	    0x00040000UL    //屁股
#define SHOULDER_DIS	    0x00000000UL
#define SHOULDER_CHR	    0x00080000UL    //肩
#define RIGHT_THIGH_DIS	    0x00000000UL
#define RIGHT_THIGH_CHR	    0x00100000UL   //右大腿
#define LEFT_THIGH_DIS	    0x00000000UL
#define LEFT_THIGH_CHR	    0x00200000UL   //左大腿

//#define PUMP_BODY_DIS	    0x00000000UL
//#define PUMP_BODY_ENA	    0x80000000UL




/*
//0-20位为气囊和气泵  ,下面为气阀部位
#define F_L_SIDE		0x00000001UL        //足左气阀
#define F_R_SIDE		0x00000002UL        //足右气阀
#define F_HEEL		        0x00000004UL        //脚后跟气阀
#define LEG_LEFT                0x00000008UL        //左腿
#define LEG_RIGHT               0x00000010UL        //右腿气阀
#define R_ARM_1	                0x00000020UL        //右胳膊1
#define R_ARM_2     	        0x00000040UL        //右胳膊2
#define R_ARM_3	                0x00000080UL        //右胳膊3
#define L_ARM_1	                0x00000100UL        ////左胳膊1
#define L_ARM_2	                0x00000200UL        ////左胳膊2
#define L_ARM_3	                0x00000400UL        //左胳膊3
#define LEFT_ARM_1_CHR    L_ARM_1
#define LEFT_ARM_2_CHR    L_ARM_2
#define LEFT_ARM_3_CHR    L_ARM_3
#define RIGHT_ARM_1_CHR   R_ARM_1
#define RIGHT_ARM_2_CHR   R_ARM_2
#define RIGHT_ARM_3_CHR   R_ARM_3
#define PE1	                0x00000800UL                            //背腰和臂肩
#define PE_ARM                  PE1
#define PE2	                0x00001000UL                            //小腿和大腿
#define R_U_WAIST	        0x00002000UL      //右腰背上面 
#define R_D_WAIST	        0x00004000UL      //右腰背下面
#define L_U_WAIST	        0x00008000UL      //左腰背上面
#define L_D_WAIST	        0x00010000UL      //左要背下面
#define R_SHOLDER	        0x00020000UL      //右肩
#define L_SHOLDER	        0x00040000UL      //左肩
#define LEFT_SHOULDER_CHR       L_SHOLDER
#define RIGHT_SHOULDER_CHR      R_SHOLDER
#define R_THIGH	                0x00080000UL      //右大腿
#define L_THIGH	                0x00100000UL      //左大腿

*/


//21-24位为滚轮旋转方式
/*  
0-1位为揉搓速度 00 停止 01 慢速 10 中速 11 高速
2-3位为揉搓方式 00向里旋转 01向外旋转 10 摇摆
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

//25-26位为滚轮旋转速度
/*0-1位为滚轮速度 00 停止 01 慢速 10 中速 11 高速*/
#define ROLLER_STOP	          (0x00<<25)
#define ROLLER_SLOW	          (0x01<<25)
#define ROLLER_MID	          (0x02<<25)
#define ROLLER_FAST	          (0x03<<25)
//27-29位为滚轮旋转方式
/*  
000 连续向里旋转
001 连续向外旋转
010 短间歇向里
011 短间歇向外
100 长间歇向里
101 长间歇向外
110 短行程搓脚
111 长行程搓脚
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
//28-31位为滚轮旋转方式
#define STRETCH_STOP		0x00000000  //0b0000 0000 靠背
#define STRETCH_UP		0x10000000  //0b0001 0000靠背后躺，小腿上升，一直到小腿上升到最高点
#define STRETCH_DOWN		0x20000000  //0b0010 0000靠背不动，小腿下降，一直到小腿下降到最低点 
#define STRETCH_RESET   	0x30000000  //0b0011 0000靠背回到一定角度，小腿不动 

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

#define VALVE_AIRPUMP1_PORT          gpioPortB   //上半身气泵 右边一个气泵 
#define VALVE_AIRPUMP1_BIT           14
#define VALVE_AIRPUMP1_MODE          gpioModePushPull

#define VALVE_AIRPUMP2_PORT          gpioPortB   //下半身气泵 中间一个气泵
#define VALVE_AIRPUMP2_BIT           13
#define VALVE_AIRPUMP2_MODE          gpioModePushPull

#define VALVE_AIRPUMP3_PORT          gpioPortD    // 左边一个气泵
#define VALVE_AIRPUMP3_BIT           8
#define VALVE_AIRPUMP3_MODE          gpioModePushPull

#define VALVE_OZON_PORT          gpioPortE    // 负离子
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




//通过软件监控 8600S使用了19个气阀，2个气帮控制，
extern BITS BITS_ValveData[3] ;

////////////////////////////////////////////////////////////////////////////////////////////
/*
//气阀位变量定义 cn31  ,                      足部气阀=CN27=AirPumpSwitch3
#define bValveData2NC0		        BITS_ValveData[0].bD3   //         V16     bag3
#define bFootHeelAirBagValve		BITS_ValveData[0].bD2   //脚后跟   V17     bag2
#define bLeftFootAirBagValve 		BITS_ValveData[0].bD1   //足左侧   V18     bag1
#define bRightFootAirBagValve		BITS_ValveData[0].bD0   //足右侧   V19     bag0
#define bLegDownBottomAirBagValve	BITS_ValveData[0].bD7   //小腿下底 V20     bag7
#define bLegDownSideAirBagValve		BITS_ValveData[0].bD6   //小腿下侧 V21     bag6
#define bLegUpBottomAirBagValve		BITS_ValveData[0].bD5   //小腿上底 V22     bag5
#define bLegUpSideAirBagValve		BITS_ValveData[0].bD4   //小腿上侧 V23     bag4




//气阀位变量定义   cn28=上半身气囊         =上半身气帮= =CN29=AirPumpSwitch2
//#define bLeftWaistAirBagValve  	        BITS_ValveData[0].bD1  //左腰  V10
//#define bRightWaistAirBagValve 	        BITS_ValveData[0].bD0  //右腰  V11

//#define bButtocksAirBagValve	    BITS_ValveData[0].bD6  //屁股      V13

#define bLeftArmUpAirBagVave  		BITS_ValveData[1].bD0   //V4    左手臂上面 
#define bLeftArmDownAirBagValve		BITS_ValveData[1].bD1   //V5    左手臂下面 
#define bLeftThighAirBagValve           BITS_ValveData[1].bD2  //左大腿    V14
#define bRightThighAirBagValve          BITS_ValveData[1].bD3  // 右大腿   V15  
#define bRightArmUpAirBagValve    	BITS_ValveData[1].bD4   //V6    右手臂上面 
#define bRightArmDownAirBagValve  	BITS_ValveData[1].bD5   //V7    右手臂下面  
//#define bLeftNeckAirBagValve		BITS_ValveData[0].bD6   //V2    左颈                                     
//#define bRightNeckAirBagValve   	BITS_ValveData[0].bD7   //V3    右颈   
  //#define bRightSholderAirBagValve  	  BITS_ValveData[0].bD6
  //#define bLeftSholderAirBagValve 	  BITS_ValveData[0].bD7     
#define bSholderAirBagValve         BITS_ValveData[1].bD7  //肩部      V12    


*/
//气阀位变量定义 cn31  ,                      足部气阀=CN27=AirPumpSwitch3



#define bLeftArmDownAirBagValve		BITS_ValveData[0].bD0   //V5    左手臂下面 
#define bLeftArmUpAirBagVave  		BITS_ValveData[0].bD1   //V4    左手臂上面 
#define bRightThighAirBagValve          BITS_ValveData[0].bD2  // 右大腿   V15 
#define bLeftThighAirBagValve           BITS_ValveData[0].bD3 //左大腿    V14
#define bRightArmUpAirBagValve    	BITS_ValveData[0].bD4   //V6    右手臂上面 
#define bRightArmDownAirBagValve  	BITS_ValveData[0].bD5   //V7    右手臂下面
#define bRightSholderAirBagValve  	BITS_ValveData[0].bD6
#define bLeftSholderAirBagValve 	BITS_ValveData[0].bD7


	


//气阀位变量定义   cn28=上半身气囊         =上半身气帮= =CN29=AirPumpSwitch2
#define bValveData2NC0		        BITS_ValveData[1].bD3   //         V16     bag3
#define bRightFootAirBagValve	        BITS_ValveData[1].bD2   //脚后跟   V17     bag2
#define bFootHeelAirBagValve		BITS_ValveData[1].bD1   //足左侧   V18     bag1
#define bLeftFootAirBagValve            BITS_ValveData[1].bD0   //足右侧   V19     bag0
#define bLegDownBottomAirBagValve	BITS_ValveData[1].bD7   //小腿下底 V20     bag7
#define bLegDownSideAirBagValve		BITS_ValveData[1].bD6   //小腿下侧 V21     bag6
#define bLegUpBottomAirBagValve		BITS_ValveData[1].bD5   //小腿上底 V22     bag5
#define bLegUpSideAirBagValve		BITS_ValveData[1].bD4   //小腿上侧 V23     bag4
/*
//data 气阀位变量定义 cn26                 手臂气帮=CN30=AirPumpSwitch1
#define bValveData1NC1 		        BITS_ValveData[0].bD0  //空    V9
#define bLeftWaistAirBagValve  	        BITS_ValveData[0].bD1  //左腰  V10
#define bRightWaistAirBagValve 	        BITS_ValveData[0].bD2  //右腰  V11
#define bValveData3NC0		        BITS_ValveData[0].bD3   //v0
#define bValveData3NC1 	                BITS_ValveData[0].bD4   //v1
#define bLeftNeckAirBagValve		BITS_ValveData[0].bD5   //V2    左颈                                     
#define bRightNeckAirBagValve   	BITS_ValveData[0].bD6   //V3    右颈       
#define bButtocksAirBagValve	        BITS_ValveData[0].bD7  //屁股      V13
*/
 
struct AirBagStruct
{
    UINT32 nPumpValveState ;//气泵和气阀的状态
    unsigned char nKeepTime1 ;//当前状态保持时间,对应弱力度
    unsigned char nKeepTime2 ;//当前状态保持时间,对应中力度
    unsigned char nKeepTime3 ;//当前状态保持时间,对应强力度
};

#define STRETCH_MODE_TIME   1 //拉退模式 1为时间控制
#define STRETCH_MODE_SWITCH 0 //拉退模式为行程控制

typedef struct
{
    unsigned char timer;        //拉退程序计时定时器，单位0.1s
    unsigned char step ;        //拉退程序步骤
    unsigned char bBackLegFlag; //拉退程序中电动缸的状态
    unsigned char active;
    unsigned char init;
    unsigned char times;        //拉退程序循环次数
    unsigned char mode;         //拉退模式 1为时间控制 0为行程控制
    unsigned char PresetTime;   //拉退模式为时间控制时的预设时间，单位0.1秒
}StretchStruct;

typedef struct
{
    unsigned char time;        //拉退程序执行时间
    unsigned char times;       //一个回合的拉退次数 一般为3次
    unsigned char mode;        //拉退模式 STRETCH_GO_OUT向前拉 STRETCH_GO_DOWN向下拉
}StretchProgramStruct;

#define C_Stretch_Up    1
#define C_Stretch_Stop  2
#define C_STRETCH_HOLD_TIME   30 //单位0.1s
#define C_STRETCH_RESET_TIME  100 //单位0.1s
#define C_STRETCH_CHARGE_TIME 100 //60//单位0.1s

#define C_STRETCH_CHARGE_TIME_OUT 60 //单位0.1s


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

//下面定义8种气囊运动程序组合

/*
#define AIRBAG_LOCATE_BACK_WAIST     209//背腰
#define AIRBAG_LOCATE_ARM_SHOLDER     210//肩部
#define AIRBAG_LOCATE_SEAT            211//坐垫


#define AIRBAG_LOCATE_ARM             212//臂部   ok

#define AIRBAG_LOCATE_ARM_SHOLDER_WAIST  213
#define AIRBAG_LOCATE_LEG_FOOT_SEAT      214
*/
 /*   //0:关闭
    //1:腿脚
    //2:背腰
    //3:臂肩
    //4:坐垫
    //5:全身
nAirBagModeStore = ((ucRXBuffer[12]>>2) & 0x07);*/

//#define AIRBAG_LOCATE_BODY_UP         2//0x02   //ok
#define AIRBAG_LOCATE_NONE            0
#define AIRBAG_LOCATE_LEG_FOOT        0x01//0x01   ////腿脚  ok
//#define AIRBAG_LOCATE_BACK_WAIST      0x02//背腰
#define AIRBAG_LOCATE_ARM_NECK        0x04//3//0x03     //臂颈  //20170205 WGH
#define AIRBAG_LOCATE_SEAT            0x08//4//坐垫             //20170205 WGH
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
    unsigned char speed;   //摇摆马达速度 0-3
    unsigned int  time;    //摇摆马达持续时间 单位1sec  
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
void Valve_SetStretchChargeATOUTFootHeelOFF(void);//找脚，释放小腿和 足部

void Valve_SetStretchCharge_FOOT_THIGH_LEG(unsigned int start);
#endif
