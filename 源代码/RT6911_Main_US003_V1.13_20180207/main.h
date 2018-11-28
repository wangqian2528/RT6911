#ifndef __MAIN_H__
#define __MAIN_H__
#define MACHINEID             1
#define C_TIMER_TEMP        0
#define C_TIMER_RUN        1
#define C_TIMER_SLOW        2
#define C_TIMER_500MS       3
#define C_TIMER_WAVE_START  4
#define C_TIMER_5           5
#define C_TIME_RUBBING      6       //用于搓背程序
#define C_TIMER_INDICATE    7

#define  C_TIMER_ENG1   3    //用于工程模式
#define  C_TIMER_ENG2   0    //用于工程模式

#define A_STATE_IDLE          1   //待机状态
#define A_STATE_SETTLE        2   //收藏状态：按摩椅复位
#define A_STATE_WAIT_COMMAND  3//等待命令：按摩椅点亮主屏，等待用户操作
#define A_STATE_PROBLEM       4    //严重的故障模式，例如主板24V电源过低，按摩椅不具备工作条件，此时手控器只需要显示错误代码就可以
#define A_STATE_WAIT_MEMORY   5//数据存储 此项功能待定
#define A_STATE_RUN           6//正常运行模式：此时手控器需要显示时间，按摩手法，或按摩程序等信息
#define A_TEST                7

#define M_NONE          1
#define M_NEXT          2
#define	M_IDLE          3
#define	M_RUN           4
#define M_PROBLEM       5
#define	M_WAIT_COMMAND  6
#define	M_ENG           7
#define	M_SETTLE        8
#define	M_TEST          9

#define ZERO_POSITION_RESET         0  //所有的电动缸复位
#define ZERO_POSITION1              1  //第一个零重力点  
#define ZERO_POSITION2              2  //第二个零重力点  
#define ZERO_POSITION_STRETCH_UP    3  //向上拉退  
#define ZERO_POSITION_STRETCH_DOWN  4  //向下拉退  
#define RockDisable                   false
#define RockEnable                    true
#define ExitRock                      0
#define EnterRock                     1
#define StartRock                     0
#define LieDown                       1
#define LieDownStop                   2
#define LieUP                         3
#define LieUpStop                     4

#define TWIST_ON  1   //add by taoqingsong ,增加椅子摇摆功能


//#define test_shoulder   1   //for test 
#define RT8305T_1    1   //

#define MASSAGE_BACK_OPTIMAL_LOCATION  276
#define MASSAGE_BACK_OPTIMAL1_LOCATION  568 //1000//1100


#define MASSAGE_BACK_STRCHUP_LOCATION  350//1100
#define MASSAGE_BACK_STRCHDOWN_LOCATION  550 //970//1100   STRCHDOWN   

#define MASSAGE_BACK_ROCK_LOCATION       550//1500

#define MASSAGE_LEG_ROCK_POSITION         1170//1200


#ifdef  RT8305T_1
      #define MASSAGE_BACK_OPTIMAL_POSITION  400//1000//400   //第一个零重力点位置
      #define MASSAGE_LEG_OPTIMAL_POSITION   800//900//800
      #define MASSAGE_BACK_OPTIMAL1_POSITION     1200//1100// 1200// 1100  //8600S=1100，第二个零重力点位置  RT8302=1200  ,RT8302S/7700  =1300
      #define MASSAGE_LEG_OPTIMAL1_POSITION  1170//1300//1040// 1200


#else
      #define MASSAGE_BACK_OPTIMAL_POSITION       400//400   //第一个零重力点位置
      #define MASSAGE_LEG_OPTIMAL_POSITION        900// 800
      #define MASSAGE_BACK_OPTIMAL1_POSITION      1200//1100//1200//1100// 1200// 1100  //8600S=1100，第二个零重力点位置  RT8302=1200  ,RT8302S/7700  =1300
      #define MASSAGE_LEG_OPTIMAL1_POSITION       1300//1200////1040// 1200
#endif


//---------------------------------------------------------------------------
#define POSITION_CTRL_OFFSET            50
#define POSITION_DISPLAY_OFFSET         80

#define   BODY_DETECT_PREPARE           0
#define   BODY_DETECT_WALK_POSITION     1


//#define   BODY_DETECT_3D_1              2
//#define   BODY_DETECT_3D_2              3
//#define   BODY_DETECT_3D_3              4
//#define   BODY_DETECT_3D_4              5

#define   BODY_DETECT_KNEAD_MIN              2
#define   BODY_DETECT_KNEAD_MAX              3
#define   BODY_DETECT_3D_FORWARD              4
#define   BODY_DETECT_3D_4              5


#define   BODY_DETECT_DATA_CHECK        6
#define   BODY_DETECT_UP_AUTO           7
#define   BODY_DETECT_DATA_REFRESH      8
#define   BODY_DETECT_ADJ               9
#define   BODY_DETECT_WALK_POSITION_1   10
#define   BODY_DETECT_WALK_POSITION_2   11
#define   BODY_DETECT_OVER              12

#define   DETECT_NO_START           0
#define   DETECT_SHOULDER           1
#define   DETECT_3D                 2
#define   DETECT_FINISH             3
#define   DETECT_INITIAL            4 
#define   SHOULDER_DETECT_FINISH    5



#define  ZERO_BACK_LEG_POWER_ON   0
#define  ZERO_BACK_LEG_POWER_OFF  1



#define  THITH_PUMP_ON   0
#define  THITH_PUMP_OFF  1
//------------mp3--------------------
#define    BLUE_SONG_ON   1
#define    USB_SONG_ON    2
#define    AUX_SONG_ON   3


enum
{
   MASSAGE_RESET_POSITION,    // 按摩椅复位位置，前滑电动缸收回，靠背电动缸在最高，小腿电动缸在最低，也是关机后的位置
   MASSAGE_INIT_POSITION,     // 按摩椅初始位置，前滑电动缸在最前，靠背电动缸在最高，小腿电动缸在最低
   MASSAGE_OPTIMAL_POSITION, //前滑电动缸在最前，靠背电动缸和小腿电动缸在自动程序初始位置
   MASSAGE_OPTIMAL2_POSITION, //前滑电动缸在最前，靠背电动缸和小腿电动缸在自动程序初始位置之后
   MASSAGE_UNKNOW_POSITION, //
   MASSAGE_MAX_POSITION,      //按摩椅最平位置 前滑电动缸在最前，靠背电动缸在最低，小腿电动缸在最高
   MASSAGE_ANY_POSITION  
};

void Main_Initial_IO(void);
void Main_Idle(void);
void Main_Settle(void);
void Main_WaitCommand(void);
void Main_Work(void);
void Main_Problem(void);

void Main_VibrateMotorControl(void);

void Main_WaveMotorStop(void);

void Main_Update(void);
void CloudProgrameInit(void);
unsigned char IsPowerZeroBackLeg(void);
unsigned char Is_THIGH_CHR(void);
void Main_ReStartShoulderCheck(void);
void RockFunctionEnable(bool Enable);
void RockProcess(void);
void  USB_MP3_SCAN_PROC(void);
#endif
