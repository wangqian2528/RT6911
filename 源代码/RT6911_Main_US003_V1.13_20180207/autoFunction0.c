
/*
const WALK_KNEAD_KNOCK_MOTOR_STRUCT_AUTO AutoFunction0t[] = 
{ 
  //{BACK_SUB_MODE_KNEAD,WALK_LOCATE_ABSULATE,250,KNEAD_RUN,0,KNOCK_STOP,0,0,SPEED_6,_3D_PROGRAM,AXIS_STRONGEST,_3D_SPEED_7,0},
  //{BACK_SUB_MODE_KNEAD,WALK_LOCATE_PressNeck,0,KNEAD_RUN,0,KNOCK_STOP,0,0,SPEED_6,_3D_PROGRAM,AXIS_STRONGEST,_3D_SPEED_7,0},
  {BACK_SUB_MODE_WAVELET,WALK_LOCATE_ABSULATE,255,KNEAD_RUN,0,KNOCK_RUN,0,0,SPEED_4,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_5,0},	  
  {BACK_SUB_MODE_WAVELET,WALK_LOCATE_SHOULDER,0,KNEAD_RUN,0,KNOCK_RUN,0,0,SPEED_3,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_5,0},	

  {BACK_SUB_MODE_SOFT_KNOCK,WALK_LOCATE_ABSULATE,255,KNEAD_STOP_AT_MED,1,KNOCK_RUN_STOP,1,4,SPEED_4,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_5,0},	  
  {BACK_SUB_MODE_SOFT_KNOCK,WALK_LOCATE_SHOULDER,0,KNEAD_STOP,0,KNOCK_RUN_STOP,1,4,SPEED_3,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_5,0},	
  //{BACK_SUB_MODE_KNOCK,WALK_LOCATE_SHOULDER  ,0,KNEAD_STOP,0,KNOCK_RUN,0,0,SPEED_3,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_5,0},
  //{BACK_SUB_MODE_KNOCK,WALK_LOCATE_PARK    ,200,KNEAD_STOP,0,KNOCK_RUN,0,0,SPEED_4,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_5,0},
  {BACK_SUB_MODE_KNOCK,WALK_LOCATE_ABSULATE,200,KNEAD_STOP,0,KNOCK_RUN,0,0,SPEED_3,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_5,0},
  {BACK_SUB_MODE_KNOCK,WALK_LOCATE_PARK    ,200,KNEAD_STOP,0,KNOCK_RUN,0,0,SPEED_3,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_5,0},
  
};
*/



//疲劳恢复(有肩部位置检测)	力度较强
const WALK_KNEAD_KNOCK_MOTOR_STRUCT_AUTO AutoFunction0[] = 
{ 
/*  
  {BACK_SUB_MODE_WAVELET,WALK_LOCATE_ABSULATE,0,KNEAD_RUN,0,KNOCK_RUN,0,0,SPEED_4,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_5,0},	  
  {BACK_SUB_MODE_WAVELET,WALK_LOCATE_SHOULDER,0,KNEAD_RUN,0,KNOCK_RUN,0,0,SPEED_3,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_5,0},	

  {BACK_SUB_MODE_SOFT_KNOCK,WALK_LOCATE_ABSULATE,0,KNEAD_STOP_AT_MED,1,KNOCK_RUN_STOP,1,4,SPEED_4,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_5,0},	  
  {BACK_SUB_MODE_SOFT_KNOCK,WALK_LOCATE_SHOULDER,0,KNEAD_STOP,0,KNOCK_RUN_STOP,1,4,SPEED_3,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_5,0},	
*/  


  //{BACK_SUB_MODE_KNEAD,WALK_LOCATE_ABSULATE,0,KNEAD_RUN,0,KNOCK_STOP,0,0,SPEED_6,_3D_PROGRAM,AXIS_STRONGEST,_3D_SPEED_7,0},
  //{BACK_SUB_MODE_KNEAD,WALK_LOCATE_PressNeck,0,KNEAD_ANTIRUN,0,KNOCK_STOP,0,0,SPEED_6,_3D_PROGRAM,AXIS_STRONGEST,_3D_SPEED_7,0},
  //{BACK_SUB_MODE_KNEAD,WALK_LOCATE_ABSULATE,0,KNEAD_RUN_CYCLE,0,KNOCK_STOP,0,0,SPEED_6,_3D_PROGRAM,AXIS_STRONGEST,_3D_SPEED_7,0},
  //{BACK_SUB_MODE_KNEAD,WALK_LOCATE_PressNeck,0,KNEAD_RUN_CYCLE,0,KNOCK_STOP,0,0,SPEED_6,_3D_PROGRAM,AXIS_STRONGEST,_3D_SPEED_7,0},
  
 // {BACK_SUB_MODE_SOFT_KNOCK,WALK_LOCATE_SHOULDER,0,KNEAD_STOP,0,KNOCK_RUN_STOP,1,4,SPEED_3,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},	
 // {BACK_SUB_MODE_SOFT_KNOCK,WALK_LOCATE_ABSULATE,0,KNEAD_RUN_STOP_AT_MED,0,KNOCK_RUN_STOP,1,4,SPEED_4,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},	
  

  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_NeckMed,0,KNEAD_RUN,0,KNOCK_STOP,0,0,SPEED_1,_3D_PROGRAM,AXIS_STRONGEST,_3D_SPEED_7,10},
  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_PARK,20,KNEAD_ANTIRUN,0,KNOCK_STOP,0,0,SPEED_1,_3D_PARK,0,0,0},
  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_PressNeck,0,KNEAD_RUN,0,KNOCK_STOP,0,0,SPEED_1,SPEED_1,_3D_PARK,0,0},
  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_PARK,20,KNEAD_ANTIRUN,0,KNOCK_STOP,0,0,SPEED_1,SPEED_1,_3D_PARK,0,0},
  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_NeckMed,0,KNEAD_RUN,0,KNOCK_STOP,0,0,SPEED_2,SPEED_1,_3D_PARK,0,0},
  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_PARK,20,KNEAD_ANTIRUN,0,KNOCK_STOP,0,0,SPEED_2,SPEED_1,_3D_PARK,0,0},
  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_PressNeck,0,KNEAD_RUN,0,KNOCK_STOP,0,0,SPEED_2,SPEED_1,_3D_PARK,0,0},
  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_PARK,20,KNEAD_RUN,0,KNOCK_STOP,0,0,SPEED_2,SPEED_1,_3D_PARK,0,0}, 
  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_ABSULATE,0,KNEAD_RUN,0,KNOCK_STOP,0,0,SPEED_1,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,10},
  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_ABSULATE,0,KNEAD_RUN_STOP_AT_MIN,5,KNOCK_STOP,0,0,SPEED_5,_3D_PARK,0,0,0},
  {BACK_SUB_MODE_WAVELET,WALK_LOCATE_PARK,80,KNEAD_RUN,0,KNOCK_RUN,0,0,SPEED_2,_3D_PARK,0,0,0},
  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_ABSULATE,0,KNEAD_RUN_STOP_AT_MIN,5,KNOCK_STOP,0,0,SPEED_5,_3D_PROGRAM,AXIS_STRONGEST,_3D_SPEED_7,10},
  {BACK_SUB_MODE_WAVELET,WALK_LOCATE_PARK,80,KNEAD_RUN,0,KNOCK_RUN,0,0,SPEED_2,_3D_PARK,0,0,0}, 
//搓背  
  {BACK_SUB_MODE_RUBBING,WALK_LOCATE_ABSULATE,0,KNEAD_RUN_RUBBING,0,KNOCK_STOP,0,0,SPEED_6,_3D_PROGRAM,AXIS_STRONGEST,_3D_SPEED_7,0},
  {BACK_SUB_MODE_RUBBING,WALK_LOCATE_PressNeck,0,KNEAD_RUN_RUBBING,0,KNOCK_STOP,0,0,SPEED_6,_3D_PROGRAM,AXIS_STRONGEST,_3D_SPEED_7,0},
  {BACK_SUB_MODE_RUBBING,WALK_LOCATE_ABSULATE,WAIST_POSITION,KNEAD_RUN_RUBBING,0,KNOCK_STOP,0,0,SPEED_6,_3D_PROGRAM,AXIS_STRONGEST,_3D_SPEED_7,0},
  {BACK_SUB_MODE_RUBBING,WALK_LOCATE_PressNeck,0,KNEAD_RUN_RUBBING,0,KNOCK_STOP,0,0,SPEED_6,_3D_PROGRAM,AXIS_STRONGEST,_3D_SPEED_7,0},
  {BACK_SUB_MODE_RUBBING,WALK_LOCATE_ABSULATE,WAIST_POSITION,KNEAD_RUN_RUBBING,0,KNOCK_STOP,0,0,SPEED_6,_3D_PROGRAM,AXIS_STRONGEST,_3D_SPEED_7,0}, 
  {BACK_SUB_MODE_SOFT_KNOCK,WALK_LOCATE_PressNeck,0,KNEAD_STOP,0,KNOCK_RUN_STOP,1,4,SPEED_1,_3D_PROGRAM,AXIS_WEAKER,_3D_SPEED_7,0},
  {BACK_SUB_MODE_SOFT_KNOCK,WALK_LOCATE_PARK,50,KNEAD_STOP_AT_MAX,0,KNOCK_RUN_STOP,1,4,SPEED_5,_3D_PROGRAM,AXIS_WEAKER,_3D_SPEED_7,0},
  {BACK_SUB_MODE_SOFT_KNOCK,WALK_LOCATE_PARK,50,KNEAD_STOP_AT_MIN,0,KNOCK_RUN_STOP,8,2,SPEED_4,_3D_PROGRAM,AXIS_WEAKER,_3D_SPEED_7,0},
  {BACK_SUB_MODE_SOFT_KNOCK,WALK_LOCATE_PARK,50,KNEAD_STOP_AT_MAX,0,KNOCK_RUN_STOP,8,2,SPEED_4,_3D_PROGRAM,AXIS_WEAKER,_3D_SPEED_7,0},
  {BACK_SUB_MODE_SOFT_KNOCK,WALK_LOCATE_PARK,50,KNEAD_STOP_AT_MIN,0,KNOCK_RUN_STOP,9,1,SPEED_3,_3D_PROGRAM,AXIS_WEAKER,_3D_SPEED_7,0},	  
  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_ABSULATE,0,KNEAD_RUN,0,KNOCK_STOP,0,0,SPEED_1,_3D_PROGRAM,AXIS_STRONGER,_3D_SPEED_7,0},							
  {BACK_SUB_MODE_KNOCK,WALK_LOCATE_PARK,50,KNEAD_STOP_AT_MAX,0,KNOCK_RUN_WIDTH,0,0,SPEED_4,_3D_PROGRAM,AXIS_STRONGER,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNOCK,WALK_LOCATE_PARK,50,KNEAD_STOP_AT_MIN,0,KNOCK_RUN_WIDTH,0,0,SPEED_5,_3D_PROGRAM,AXIS_STRONGER,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNOCK,WALK_LOCATE_PARK,50,KNEAD_STOP_AT_MAX,0,KNOCK_RUN_WIDTH,0,0,SPEED_6,_3D_PROGRAM,AXIS_STRONGER,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNOCK,WALK_LOCATE_PARK,50,KNEAD_STOP_AT_MIN,0,KNOCK_RUN_WIDTH,0,0,SPEED_6,_3D_PROGRAM,AXIS_STRONGER,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNOCK,WALK_LOCATE_PARK,50,KNEAD_STOP_AT_MAX,0,KNOCK_RUN_WIDTH,0,0,SPEED_4,_3D_PROGRAM,AXIS_STRONGER,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNOCK,WALK_LOCATE_PARK,100,KNEAD_STOP_AT_MIN,0,KNOCK_RUN_WIDTH,0,0,SPEED_3,_3D_PROGRAM,AXIS_STRONGER,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNOCK,WALK_LOCATE_PARK,100,KNEAD_STOP_AT_MAX,0,KNOCK_RUN_WIDTH,0,0,SPEED_4,_3D_PROGRAM,AXIS_STRONGER,_3D_SPEED_7,0},  
  {BACK_SUB_MODE_WAVELET,WALK_LOCATE_PressNeck,0,KNEAD_RUN,0,KNOCK_RUN,0,0,SPEED_1,_3D_PROGRAM,AXIS_STRONGEST,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_PARK,30,KNEAD_RUN,0,KNOCK_STOP,0,0,SPEED_1,_3D_PROGRAM,AXIS_STRONGEST,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_NeckMed,0,KNEAD_RUN,0,KNOCK_STOP,0,0,SPEED_2,_3D_PROGRAM,AXIS_STRONGEST,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_PARK,20,KNEAD_RUN,0,KNOCK_STOP,0,0,SPEED_2,_3D_PROGRAM,AXIS_STRONGEST,_3D_SPEED_7,0},
  {BACK_SUB_MODE_WAVELET,WALK_LOCATE_ABSULATE,0,KNEAD_RUN,0,KNOCK_RUN,0,0,SPEED_2,_3D_PROGRAM,AXIS_STRONGEST,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_PARK,30,KNEAD_RUN,0,KNOCK_STOP,0,0,SPEED_2,_3D_PROGRAM,AXIS_STRONGEST,_3D_SPEED_7,0},
  //搓背  
  {BACK_SUB_MODE_RUBBING,WALK_LOCATE_ABSULATE,0,KNEAD_RUN_RUBBING,0,KNOCK_STOP,0,0,SPEED_6,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},
  {BACK_SUB_MODE_RUBBING,WALK_LOCATE_PressNeck,0,KNEAD_RUN_RUBBING,0,KNOCK_STOP,0,0,SPEED_6,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},
  {BACK_SUB_MODE_RUBBING,WALK_LOCATE_ABSULATE,WAIST_POSITION,KNEAD_RUN_RUBBING,0,KNOCK_STOP,0,0,SPEED_6,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},
  {BACK_SUB_MODE_RUBBING,WALK_LOCATE_PressNeck,0,KNEAD_RUN_RUBBING,0,KNOCK_STOP,0,0,SPEED_6,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},
  {BACK_SUB_MODE_RUBBING,WALK_LOCATE_ABSULATE,WAIST_POSITION,KNEAD_RUN_RUBBING,0,KNOCK_STOP,0,0,SPEED_6,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},  
  {BACK_SUB_MODE_WAVELET,WALK_LOCATE_PressNeck,0,KNEAD_RUN,0,KNOCK_RUN,0,0,SPEED_2,_3D_PROGRAM,AXIS_STRONGER,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_PARK,20,KNEAD_ANTIRUN,0,KNOCK_STOP,0,0,SPEED_2,_3D_PROGRAM,AXIS_STRONGER,_3D_SPEED_7,0},	
  //肩部捶打
  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_PARK,0,KNEAD_RUN_STOP_AT_MIN,5,KNOCK_STOP,0,0,SPEED_3,_3D_PROGRAM,AXIS_STRONGER,_3D_SPEED_7,0},
  {BACK_SUB_MODE_SOFT_KNOCK,WALK_LOCATE_PARK,30,KNEAD_STOP,0,KNOCK_RUN_STOP,8,2,SPEED_3},
  {BACK_SUB_MODE_SOFT_KNOCK,WALK_LOCATE_PARK,30,KNEAD_STOP_AT_MAX,0,KNOCK_RUN_STOP,7,3,SPEED_3,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},
  {BACK_SUB_MODE_SOFT_KNOCK,WALK_LOCATE_PARK,30,KNEAD_STOP_AT_MIN,0,KNOCK_RUN_STOP,6,4,SPEED_4,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},
  {BACK_SUB_MODE_SOFT_KNOCK,WALK_LOCATE_PARK,30,KNEAD_STOP_AT_MAX,0,KNOCK_RUN_STOP,5,5,SPEED_5,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},
  {BACK_SUB_MODE_SOFT_KNOCK,WALK_LOCATE_PARK,30,KNEAD_STOP_AT_MIN,0,KNOCK_RUN_STOP,1,4,SPEED_6,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},
  {BACK_SUB_MODE_SOFT_KNOCK,WALK_LOCATE_PARK,30,KNEAD_STOP_AT_MAX,0,KNOCK_RUN_STOP,1,4,SPEED_6,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},
  {BACK_SUB_MODE_SOFT_KNOCK,WALK_LOCATE_PARK,30,KNEAD_STOP_AT_MIN,0,KNOCK_RUN_STOP,8,2,SPEED_4,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},
  {BACK_SUB_MODE_SOFT_KNOCK,WALK_LOCATE_PARK,30,KNEAD_STOP_AT_MAX,0,KNOCK_RUN_STOP,8,2,SPEED_4,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},
  {BACK_SUB_MODE_SOFT_KNOCK,WALK_LOCATE_PARK,30,KNEAD_STOP_AT_MIN,0,KNOCK_RUN_STOP,9,1,SPEED_3,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},
  {BACK_SUB_MODE_SOFT_KNOCK,WALK_LOCATE_PARK,30,KNEAD_STOP_AT_MAX,0,KNOCK_RUN_STOP,9,1,SPEED_3,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},
  //搓背  
  {BACK_SUB_MODE_RUBBING,WALK_LOCATE_ABSULATE,0,KNEAD_RUN_RUBBING,0,KNOCK_STOP,0,0,SPEED_6,_3D_PROGRAM,AXIS_STRONGEST,_3D_SPEED_7,0},
  {BACK_SUB_MODE_RUBBING,WALK_LOCATE_PressNeck,0,KNEAD_RUN_RUBBING,0,KNOCK_STOP,0,0,SPEED_6,_3D_PROGRAM,AXIS_STRONGEST,_3D_SPEED_7,0},
  {BACK_SUB_MODE_RUBBING,WALK_LOCATE_ABSULATE,WAIST_POSITION,KNEAD_RUN_RUBBING,0,KNOCK_STOP,0,0,SPEED_6,_3D_PROGRAM,AXIS_STRONGEST,_3D_SPEED_7,0},
  {BACK_SUB_MODE_RUBBING,WALK_LOCATE_PressNeck,0,KNEAD_RUN_RUBBING,0,KNOCK_STOP,0,0,SPEED_6,_3D_PROGRAM,AXIS_STRONGEST,_3D_SPEED_7,0},
  {BACK_SUB_MODE_RUBBING,WALK_LOCATE_ABSULATE,WAIST_POSITION,KNEAD_RUN_RUBBING,0,KNOCK_STOP,0,0,SPEED_6,_3D_PROGRAM,AXIS_STRONGEST,_3D_SPEED_7,0},
  {BACK_SUB_MODE_RUBBING,WALK_LOCATE_PressNeck,0,KNEAD_RUN_RUBBING,0,KNOCK_STOP,0,0,SPEED_6,_3D_PROGRAM,AXIS_STRONGEST,_3D_SPEED_7,0},
  {BACK_SUB_MODE_RUBBING,WALK_LOCATE_ABSULATE,WAIST_POSITION,KNEAD_RUN_RUBBING,0,KNOCK_STOP,0,0,SPEED_6,_3D_PROGRAM,AXIS_STRONGEST,_3D_SPEED_7,0}, 
  //捶击定位（窄中宽）
  {BACK_SUB_MODE_PRESS,WALK_LOCATE_ABSULATE,WAIST_POSITION,KNEAD_STOP,0,KNOCK_STOP,0,0,SPEED_1,_3D_PROGRAM,AXIS_WEAKER,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNOCK,WALK_LOCATE_PARK,20,KNEAD_RUN_STOP_AT_MIN,0,KNOCK_RUN_WIDTH,0,0,SPEED_4,_3D_PROGRAM,AXIS_WEAKER,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNOCK,WALK_LOCATE_PARK,20,KNEAD_RUN_STOP_AT_MIN,0,KNOCK_RUN_WIDTH,0,0,SPEED_4,_3D_PROGRAM,AXIS_WEAKER,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNOCK,WALK_LOCATE_PARK,20,KNEAD_RUN_STOP_AT_MAX,0,KNOCK_RUN_WIDTH,0,0,SPEED_4,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNOCK,WALK_LOCATE_PARK,20,KNEAD_RUN_STOP_AT_MIN,0,KNOCK_RUN_WIDTH,0,0,SPEED_4,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNOCK,WALK_LOCATE_PARK,20,KNEAD_RUN_STOP_AT_MAX,0,KNOCK_RUN_WIDTH,0,0,SPEED_4,_3D_PROGRAM,AXIS_WEAKER,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNOCK,WALK_LOCATE_PARK,20,KNEAD_RUN_STOP_AT_MIN,0,KNOCK_RUN_WIDTH,0,0,SPEED_4,_3D_PROGRAM,AXIS_WEAKER,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNOCK,WALK_LOCATE_PARK,30,KNEAD_STOP,0,KNOCK_RUN,0,0,SPEED_5,_3D_PROGRAM,AXIS_WEAKER,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNOCK,WALK_LOCATE_PARK,30,KNEAD_STOP,0,KNOCK_RUN,0,0,SPEED_6,_3D_PROGRAM,AXIS_WEAKER,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNOCK,WALK_LOCATE_PARK,30,KNEAD_RUN_STOP_AT_MED,0,KNOCK_RUN_WIDTH,0,0,SPEED_3,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNOCK,WALK_LOCATE_PARK,30,KNEAD_STOP,0,KNOCK_RUN,0,0,SPEED_5,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNOCK,WALK_LOCATE_PARK,30,KNEAD_STOP,0,KNOCK_RUN,0,0,SPEED_6,_3D_PROGRAM,AXIS_WEAKER,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNOCK,WALK_LOCATE_PARK,30,KNEAD_RUN_STOP_AT_MAX,0,KNOCK_RUN_WIDTH,0,0,SPEED_3,_3D_PROGRAM,AXIS_WEAKER,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNOCK,WALK_LOCATE_PARK,30,KNEAD_STOP,0,KNOCK_RUN,0,0,SPEED_5,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNOCK,WALK_LOCATE_PARK,30,KNEAD_STOP,0,KNOCK_RUN,0,0,SPEED_6,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_ABSULATE,0,KNEAD_RUN,0,KNOCK_STOP,0,0,SPEED_5,_3D_PROGRAM,AXIS_STRONGER,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_PARK,50,KNEAD_RUN,0,KNOCK_STOP,0,0,SPEED_4,_3D_PROGRAM,AXIS_STRONGER,_3D_SPEED_7,0}, 
  //捶击定位（窄中宽）
  {BACK_SUB_MODE_KNOCK,WALK_LOCATE_PARK,20,KNEAD_RUN_STOP_AT_MIN,0,KNOCK_RUN_WIDTH,0,0,SPEED_4,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNOCK,WALK_LOCATE_PARK,20,KNEAD_RUN_STOP_AT_MAX,0,KNOCK_RUN_WIDTH,0,0,SPEED_4,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNOCK,WALK_LOCATE_PARK,20,KNEAD_RUN_STOP_AT_MIN,0,KNOCK_RUN_WIDTH,0,0,SPEED_5,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNOCK,WALK_LOCATE_PARK,20,KNEAD_RUN_STOP_AT_MAX,0,KNOCK_RUN_WIDTH,0,0,SPEED_5,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNOCK,WALK_LOCATE_PARK,20,KNEAD_RUN_STOP_AT_MIN,0,KNOCK_RUN_WIDTH,0,0,SPEED_6,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNOCK,WALK_LOCATE_PARK,20,KNEAD_RUN_STOP_AT_MAX,0,KNOCK_RUN_WIDTH,0,0,SPEED_6,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNOCK,WALK_LOCATE_PARK,30,KNEAD_RUN_STOP_AT_MIN,0,KNOCK_RUN_WIDTH,0,0,SPEED_4,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNOCK,WALK_LOCATE_PARK,30,KNEAD_STOP,0,KNOCK_RUN,0,0,SPEED_5,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNOCK,WALK_LOCATE_PARK,30,KNEAD_STOP,0,KNOCK_RUN,0,0,SPEED_6,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNOCK,WALK_LOCATE_PARK,30,KNEAD_RUN_STOP_AT_MED,0,KNOCK_RUN_WIDTH,0,0,SPEED_3,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNOCK,WALK_LOCATE_PARK,30,KNEAD_STOP,0,KNOCK_RUN,0,0,SPEED_5,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNOCK,WALK_LOCATE_PARK,30,KNEAD_STOP,0,KNOCK_RUN,0,0,SPEED_6,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNOCK,WALK_LOCATE_PARK,30,KNEAD_RUN_STOP_AT_MAX,0,KNOCK_RUN_WIDTH,0,0,SPEED_3,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNOCK,WALK_LOCATE_PARK,30,KNEAD_STOP,0,KNOCK_RUN,0,0,SPEED_5,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNOCK,WALK_LOCATE_PARK,30,KNEAD_STOP,0,KNOCK_RUN,0,0,SPEED_6,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},
  //揉捏
  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_PARK,0,KNEAD_RUN_STOP_AT_MIN,5,KNOCK_STOP,0,0,SPEED_5,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},
  //向上行走15，捶击定位（窄）
  {BACK_SUB_MODE_KNOCK,WALK_LOCATE_ABSULATE,15,KNEAD_STOP,0,KNOCK_RUN,0,0,SPEED_4,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},	
  {BACK_SUB_MODE_KNOCK,WALK_LOCATE_PARK,30,KNEAD_STOP,0,KNOCK_RUN,0,0,SPEED_3,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},	
  //向上行走15，捶击定位（窄）
  {BACK_SUB_MODE_KNOCK,WALK_LOCATE_ABSULATE,30,KNEAD_STOP,0,KNOCK_RUN,0,0,SPEED_3,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNOCK,WALK_LOCATE_PARK,30,KNEAD_STOP,0,KNOCK_RUN,0,0,SPEED_5,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},	
  //向上行走15，捶击定位（窄）80
  {BACK_SUB_MODE_KNOCK,WALK_LOCATE_ABSULATE,45,KNEAD_STOP,0,KNOCK_RUN,0,0,SPEED_3,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},	
  {BACK_SUB_MODE_KNOCK,WALK_LOCATE_PARK,30,KNEAD_STOP,0,KNOCK_RUN,0,0,SPEED_5,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},	
  //向上行走15，捶击定位（窄）
  {BACK_SUB_MODE_KNOCK,WALK_LOCATE_ABSULATE,60,KNEAD_STOP,0,KNOCK_RUN,0,0,SPEED_3,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},	
  {BACK_SUB_MODE_KNOCK,WALK_LOCATE_PARK,30,KNEAD_STOP,0,KNOCK_RUN,0,0,SPEED_4,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},	
  //向下揉捏到底
  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_ABSULATE,0,KNEAD_RUN,0,KNOCK_STOP,0,0,SPEED_3,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_PARK,0,KNEAD_RUN_STOP_AT_MIN,5,KNOCK_STOP,0,0,SPEED_6,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},
  //定点轻敲5次
  {BACK_SUB_MODE_SOFT_KNOCK,WALK_LOCATE_PARK,25,KNEAD_STOP,0,KNOCK_RUN_STOP,1,4,SPEED_5,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},	
  //轻敲到顶部
  {BACK_SUB_MODE_SOFT_KNOCK,WALK_LOCATE_TOP,0,KNEAD_STOP,0,KNOCK_RUN_STOP,1,4,SPEED_4,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},	
  //定点低速揉捏5
  {BACK_SUB_MODE_SOFT_KNOCK,WALK_LOCATE_PARK,30,KNEAD_STOP,0,KNOCK_RUN_STOP,1,4,SPEED_1,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},
  //向下捶击行走到肩部	////
  {BACK_SUB_MODE_KNOCK,WALK_LOCATE_PressNeck,0,KNEAD_STOP,0,KNOCK_RUN,0,0,SPEED_1,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},		
  //定点捶击2秒
  {BACK_SUB_MODE_KNOCK,WALK_LOCATE_PARK,0,KNEAD_STOP,0,KNOCK_RUN,20,0,SPEED_1,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},
  //向上捶击行走到顶部	100
  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_TOP,0,KNEAD_ANTIRUN,0,KNOCK_STOP,0,0,SPEED_1,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},		
  //定点捶击2秒
  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_PARK,0,KNEAD_ANTIRUN,0,KNOCK_STOP,20,0,SPEED_1,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},
  //向下捶击行走到肩部	
  {BACK_SUB_MODE_KNOCK,WALK_LOCATE_PressNeck,0,KNEAD_STOP,0,KNOCK_RUN,0,0,SPEED_1,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},		
  //定点捶击2秒
  {BACK_SUB_MODE_KNOCK,WALK_LOCATE_PARK,0,KNEAD_STOP,0,KNOCK_RUN,20,0,SPEED_1,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},
  //指压到顶	
  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_SHOULDER,0,KNEAD_RUN,0,KNOCK_STOP,0,0,SPEED_2,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},		
  //定点揉捏5
  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_PARK,0,KNEAD_RUN_STOP_AT_MIN,5,KNOCK_STOP,0,0,SPEED_3,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},
  //向下捶击行走到底	
  {BACK_SUB_MODE_KNOCK,WALK_LOCATE_ABSULATE,0,KNEAD_STOP,0,KNOCK_RUN,0,0,SPEED_3,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},
  {BACK_SUB_MODE_WAVELET,WALK_LOCATE_PARK,30,KNEAD_ANTIRUN,0,KNOCK_RUN,0,0,SPEED_3,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_PARK,0,KNEAD_RUN_STOP_AT_MIN,4,KNOCK_STOP,0,0,SPEED_4,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNOCK,WALK_LOCATE_PARK,30,KNEAD_STOP,0,KNOCK_RUN,0,0,SPEED_5,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_PARK,50,KNEAD_ANTIRUN,0,KNOCK_STOP,0,0,SPEED_4,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},  
  //搓背  
  {BACK_SUB_MODE_RUBBING,WALK_LOCATE_ABSULATE,0,KNEAD_RUN_RUBBING,0,KNOCK_STOP,0,0,SPEED_6,_3D_PROGRAM,AXIS_STRONGEST,_3D_SPEED_7,0},
  {BACK_SUB_MODE_RUBBING,WALK_LOCATE_PressNeck,0,KNEAD_RUN_RUBBING,0,KNOCK_STOP,0,0,SPEED_6,_3D_PROGRAM,AXIS_STRONGEST,_3D_SPEED_7,0},
  {BACK_SUB_MODE_RUBBING,WALK_LOCATE_ABSULATE,WAIST_POSITION,KNEAD_RUN_RUBBING,0,KNOCK_STOP,0,0,SPEED_6,_3D_PROGRAM,AXIS_STRONGEST,_3D_SPEED_7,0},
  {BACK_SUB_MODE_RUBBING,WALK_LOCATE_PressNeck,0,KNEAD_RUN_RUBBING,0,KNOCK_STOP,0,0,SPEED_6,_3D_PROGRAM,AXIS_STRONGEST,_3D_SPEED_7,0},
  {BACK_SUB_MODE_RUBBING,WALK_LOCATE_ABSULATE,WAIST_POSITION,KNEAD_RUN_RUBBING,0,KNOCK_STOP,0,0,SPEED_6,_3D_PROGRAM,AXIS_STRONGEST,_3D_SPEED_7,0},
  {BACK_SUB_MODE_RUBBING,WALK_LOCATE_PressNeck,0,KNEAD_RUN_RUBBING,0,KNOCK_STOP,0,0,SPEED_6,_3D_PROGRAM,AXIS_STRONGEST,_3D_SPEED_7,0},
  {BACK_SUB_MODE_RUBBING,WALK_LOCATE_ABSULATE,WAIST_POSITION,KNEAD_RUN_RUBBING,0,KNOCK_STOP,0,0,SPEED_6,_3D_PROGRAM,AXIS_STRONGEST,_3D_SPEED_7,0},
  //向上揉捶行走25/115
  {BACK_SUB_MODE_WAVELET,WALK_LOCATE_ABSULATE,25,KNEAD_RUN,0,KNOCK_RUN,0,0,SPEED_2,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},
  {BACK_SUB_MODE_WAVELET,WALK_LOCATE_PARK,30,KNEAD_RUN,0,KNOCK_RUN,0,0,SPEED_4,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},
  {BACK_SUB_MODE_WAVELET,WALK_LOCATE_ABSULATE,50,KNEAD_RUN,0,KNOCK_RUN,0,0,SPEED_2,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},
  {BACK_SUB_MODE_WAVELET,WALK_LOCATE_PARK,30,KNEAD_RUN,0,KNOCK_RUN,0,0,SPEED_3,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},
  //向下揉捏到底
  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_ABSULATE,0,KNEAD_ANTIRUN,0,KNOCK_STOP,0,0,SPEED_3,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNOCK,WALK_LOCATE_PARK,30,KNEAD_STOP_AT_MIN,0,KNOCK_RUN_WIDTH,0,0,SPEED_4,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_PARK,50,KNEAD_RUN,0,KNOCK_STOP,0,0,SPEED_4,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},
  {BACK_SUB_MODE_WAVELET,WALK_LOCATE_PARK,30,KNEAD_RUN,0,KNOCK_RUN,0,0,SPEED_4,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_PARK,50,KNEAD_RUN,0,KNOCK_STOP,0,0,SPEED_4,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},
  //揉捏到肩部
  {BACK_SUB_MODE_WAVELET,WALK_LOCATE_SHOULDER,0,KNEAD_RUN,0,KNOCK_RUN,0,0,SPEED_1,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},
  {BACK_SUB_MODE_WAVELET,WALK_LOCATE_ABSULATE,60,KNEAD_RUN,0,KNOCK_RUN,0,0,SPEED_2,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},
  {BACK_SUB_MODE_WAVELET,WALK_LOCATE_ABSULATE,0,KNEAD_RUN,0,KNOCK_RUN,0,0,SPEED_3,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},
  {BACK_SUB_MODE_WAVELET,WALK_LOCATE_PARK,30,KNEAD_RUN,0,KNOCK_RUN,0,0,SPEED_3,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},
  //定点揉捏5
  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_PARK,0,KNEAD_RUN_STOP_AT_MIN,3,KNOCK_STOP,0,0,SPEED_4,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},
  //窄指压到顶
  {BACK_SUB_MODE_PRESS,WALK_LOCATE_TOP,0,KNEAD_STOP,0,KNOCK_STOP,0,0,SPEED_1,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},
  //颈部动作组合
  {BACK_SUB_MODE_KNOCK,WALK_LOCATE_SHOULDER,0,KNEAD_STOP,0,KNOCK_RUN,0,0,SPEED_1,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNOCK,WALK_LOCATE_TOP,0,KNEAD_STOP,0,KNOCK_RUN,0,0,SPEED_1,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},  
  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_PARK,30,KNEAD_RUN,0,KNOCK_STOP,0,0,SPEED_2,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},		
  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_SHOULDER,20,KNEAD_RUN,0,KNOCK_STOP,0,0,SPEED_2,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},		
  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_PARK,40,KNEAD_RUN,0,KNOCK_STOP,0,0,SPEED_2,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},		
  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_SHOULDER,0,KNEAD_RUN,0,KNOCK_STOP,0,0,SPEED_2,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},		
  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_PARK,30,KNEAD_RUN,0,KNOCK_STOP,0,0,SPEED_2,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},		
  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_PARK,0,KNEAD_RUN_STOP_AT_MIN,3,KNOCK_STOP,0,0,SPEED_2,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},
  //低速敲打到底部/141
  {BACK_SUB_MODE_KNOCK,WALK_LOCATE_TOP,0,KNEAD_STOP,0,KNOCK_RUN,0,0,SPEED_1,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNOCK,WALK_LOCATE_PARK,20,KNEAD_STOP,0,KNOCK_RUN,0,0,SPEED_1,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},		
  {BACK_SUB_MODE_KNOCK,WALK_LOCATE_SHOULDER,0,KNEAD_STOP,0,KNOCK_RUN,0,0,SPEED_1,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_PARK,0,KNEAD_RUN_STOP_AT_MIN,3,KNOCK_STOP,0,0,SPEED_2,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNOCK,WALK_LOCATE_PARK,20,KNEAD_STOP,0,KNOCK_RUN,0,0,SPEED_1,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_PARK,0,KNEAD_RUN_STOP_AT_MIN,3,KNOCK_STOP,0,0,SPEED_2,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},		
  {BACK_SUB_MODE_KNOCK,WALK_LOCATE_NeckMed,0,KNEAD_STOP,0,KNOCK_RUN,0,0,SPEED_1,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNOCK,WALK_LOCATE_PARK,20,KNEAD_STOP,0,KNOCK_RUN,0,0,SPEED_1,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_PARK,0,KNEAD_RUN_STOP_AT_MIN,3,KNOCK_STOP,0,0,SPEED_2,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},		
  {BACK_SUB_MODE_KNOCK,WALK_LOCATE_PressNeck,0,KNEAD_STOP,0,KNOCK_RUN,0,0,SPEED_1,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNOCK,WALK_LOCATE_PARK,20,KNEAD_STOP,0,KNOCK_RUN,0,0,SPEED_1,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_PARK,0,KNEAD_RUN_STOP_AT_MIN,3,KNOCK_STOP,0,0,SPEED_3,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},		
  {BACK_SUB_MODE_KNOCK,WALK_LOCATE_ABSULATE,80,KNEAD_STOP,0,KNOCK_RUN,0,0,SPEED_1,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNOCK,WALK_LOCATE_PARK,20,KNEAD_STOP,0,KNOCK_RUN,0,0,SPEED_1,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_PARK,0,KNEAD_RUN_STOP_AT_MIN,3,KNOCK_STOP,0,0,SPEED_3,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},		
  {BACK_SUB_MODE_KNOCK,WALK_LOCATE_ABSULATE,40,KNEAD_STOP,0,KNOCK_RUN,0,0,SPEED_2,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNOCK,WALK_LOCATE_PARK,20,KNEAD_STOP,0,KNOCK_RUN,0,0,SPEED_2,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_PARK,0,KNEAD_RUN_STOP_AT_MIN,3,KNOCK_STOP,0,0,SPEED_4,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},		
  {BACK_SUB_MODE_KNOCK,WALK_LOCATE_ABSULATE,0,KNEAD_STOP,0,KNOCK_RUN,0,0,SPEED_3,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNOCK,WALK_LOCATE_PARK,20,KNEAD_STOP,0,KNOCK_RUN,0,0,SPEED_3,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_PARK,0,KNEAD_RUN_STOP_AT_MIN,3,KNOCK_STOP,0,0,SPEED_5,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},	  
  //搓背  
  {BACK_SUB_MODE_RUBBING,WALK_LOCATE_ABSULATE,0,KNEAD_RUN_RUBBING,0,KNOCK_STOP,0,0,SPEED_6,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},
  {BACK_SUB_MODE_RUBBING,WALK_LOCATE_PressNeck,0,KNEAD_RUN_RUBBING,0,KNOCK_STOP,0,0,SPEED_6,_3D_PROGRAM,AXIS_STRONGEST,_3D_SPEED_7,0},
  {BACK_SUB_MODE_RUBBING,WALK_LOCATE_ABSULATE,WAIST_POSITION,KNEAD_RUN_RUBBING,0,KNOCK_STOP,0,0,SPEED_6,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},
  {BACK_SUB_MODE_RUBBING,WALK_LOCATE_PressNeck,0,KNEAD_RUN_RUBBING,0,KNOCK_STOP,0,0,SPEED_6,_3D_PROGRAM,AXIS_STRONGEST,_3D_SPEED_7,0},
  {BACK_SUB_MODE_RUBBING,WALK_LOCATE_ABSULATE,WAIST_POSITION,KNEAD_RUN_RUBBING,0,KNOCK_STOP,0,0,SPEED_6,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},
  {BACK_SUB_MODE_RUBBING,WALK_LOCATE_PressNeck,0,KNEAD_RUN_RUBBING,0,KNOCK_STOP,0,0,SPEED_6,_3D_PROGRAM,AXIS_STRONGEST,_3D_SPEED_7,0},
  {BACK_SUB_MODE_RUBBING,WALK_LOCATE_ABSULATE,WAIST_POSITION,KNEAD_RUN_RUBBING,0,KNOCK_STOP,0,0,SPEED_6,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0}, 
  //腰部变速敲打161
  {BACK_SUB_MODE_KNOCK,WALK_LOCATE_PARK,20,KNEAD_STOP,0,KNOCK_RUN,0,0,SPEED_2,_3D_PROGRAM,AXIS_WEAKER,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNOCK,WALK_LOCATE_PARK,20,KNEAD_STOP,0,KNEAD_ANTIRUN,0,0,SPEED_3,_3D_PROGRAM,AXIS_WEAKER,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNOCK,WALK_LOCATE_PARK,30,KNEAD_STOP,0,KNOCK_RUN,0,0,SPEED_4,_3D_PROGRAM,AXIS_WEAKER,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNOCK,WALK_LOCATE_PARK,35,KNEAD_STOP,0,KNEAD_ANTIRUN,0,0,SPEED_5,_3D_PROGRAM,AXIS_WEAKER,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNOCK,WALK_LOCATE_PARK,35,KNEAD_STOP,0,KNOCK_RUN,0,0,SPEED_6,_3D_PROGRAM,AXIS_WEAKER,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNOCK,WALK_LOCATE_PARK,35,KNEAD_STOP,0,KNEAD_ANTIRUN,0,0,SPEED_5,_3D_PROGRAM,AXIS_WEAKER,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNOCK,WALK_LOCATE_PARK,30,KNEAD_STOP,0,KNOCK_RUN,0,0,SPEED_4,_3D_PROGRAM,AXIS_WEAKER,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNOCK,WALK_LOCATE_PARK,20,KNEAD_STOP,0,KNEAD_ANTIRUN,0,0,SPEED_3,_3D_PROGRAM,AXIS_WEAKER,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNOCK,WALK_LOCATE_PARK,20,KNEAD_STOP,0,KNOCK_RUN,0,0,SPEED_2,_3D_PROGRAM,AXIS_WEAKER,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_PARK,0,KNEAD_RUN_STOP_AT_MED,5,KNOCK_STOP,0,0,SPEED_5,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNOCK,WALK_LOCATE_PARK,20,KNEAD_STOP,0,KNOCK_RUN,0,0,SPEED_2,_3D_PROGRAM,AXIS_WEAKER,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNOCK,WALK_LOCATE_PARK,20,KNEAD_STOP,0,KNOCK_RUN,0,0,SPEED_3,_3D_PROGRAM,AXIS_WEAKER,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNOCK,WALK_LOCATE_PARK,30,KNEAD_STOP,0,KNOCK_RUN,0,0,SPEED_4,_3D_PROGRAM,AXIS_WEAKER,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNOCK,WALK_LOCATE_PARK,35,KNEAD_STOP,0,KNOCK_RUN,0,0,SPEED_5,_3D_PROGRAM,AXIS_WEAKER,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNOCK,WALK_LOCATE_PARK,35,KNEAD_STOP,0,KNOCK_RUN,0,0,SPEED_6,_3D_PROGRAM,AXIS_WEAKER,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNOCK,WALK_LOCATE_PARK,35,KNEAD_STOP,0,KNOCK_RUN,0,0,SPEED_5,_3D_PROGRAM,AXIS_WEAKER,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNOCK,WALK_LOCATE_PARK,30,KNEAD_STOP,0,KNOCK_RUN,0,0,SPEED_4,_3D_PROGRAM,AXIS_WEAKER,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNOCK,WALK_LOCATE_PARK,20,KNEAD_STOP,0,KNOCK_RUN,0,0,SPEED_3,_3D_PROGRAM,AXIS_WEAKER,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNOCK,WALK_LOCATE_PARK,20,KNEAD_STOP,0,KNOCK_RUN,0,0,SPEED_2,_3D_PROGRAM,AXIS_WEAKER,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_PARK,0,KNEAD_RUN_STOP_AT_MIN,5,KNOCK_STOP,0,0,SPEED_5,_3D_PROGRAM,AXIS_WEAKER,_3D_SPEED_7,0},
  //行走揉摧同步182
  {BACK_SUB_MODE_WAVELET,WALK_LOCATE_PARK,40,KNEAD_RUN,0,KNOCK_RUN,0,0,SPEED_6,_3D_PROGRAM,AXIS_WEAKEST,_3D_SPEED_7,0},
  {BACK_SUB_MODE_WAVELET,WALK_LOCATE_PARK,40,KNEAD_RUN,0,KNOCK_RUN,0,0,SPEED_5,_3D_PROGRAM,AXIS_WEAKEST,_3D_SPEED_7,0},
  {BACK_SUB_MODE_WAVELET,WALK_LOCATE_ABSULATE,40,KNEAD_RUN,0,KNOCK_RUN,0,0,SPEED_4,_3D_PROGRAM,AXIS_WEAKER,_3D_SPEED_7,0},
  {BACK_SUB_MODE_WAVELET,WALK_LOCATE_ABSULATE,80,KNEAD_RUN,0,KNOCK_RUN,0,0,SPEED_3,_3D_PROGRAM,AXIS_WEAKER,_3D_SPEED_7,0},
  {BACK_SUB_MODE_WAVELET,WALK_LOCATE_NeckMed,0,KNEAD_RUN,0,KNOCK_RUN,0,0,SPEED_3,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},
  {BACK_SUB_MODE_WAVELET,WALK_LOCATE_PARK,30,KNEAD_RUN,0,KNOCK_RUN,0,0,SPEED_2,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},
  {BACK_SUB_MODE_WAVELET,WALK_LOCATE_SHOULDER,0,KNEAD_RUN,0,KNOCK_RUN,0,0,SPEED_2,_3D_PROGRAM,AXIS_STRONGER,_3D_SPEED_7,0},
  {BACK_SUB_MODE_WAVELET,WALK_LOCATE_PARK,30,KNEAD_RUN,0,KNOCK_RUN,0,0,SPEED_2,_3D_PROGRAM,AXIS_STRONGER,_3D_SPEED_7,0},
  {BACK_SUB_MODE_WAVELET,WALK_LOCATE_SHOULDER,20,KNEAD_RUN,0,KNOCK_RUN,0,0,SPEED_2,_3D_PROGRAM,AXIS_STRONGEST,_3D_SPEED_7,0},
  {BACK_SUB_MODE_WAVELET,WALK_LOCATE_PARK,30,KNEAD_RUN,0,KNOCK_RUN,0,0,SPEED_2,_3D_PROGRAM,AXIS_STRONGEST,_3D_SPEED_7,0}, 
  //搓背  
  {BACK_SUB_MODE_RUBBING,WALK_LOCATE_ABSULATE,0,KNEAD_RUN_RUBBING,0,KNOCK_STOP,0,0,SPEED_6,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},
  {BACK_SUB_MODE_RUBBING,WALK_LOCATE_PressNeck,0,KNEAD_RUN_RUBBING,0,KNOCK_STOP,0,0,SPEED_6,_3D_PROGRAM,AXIS_STRONGER,_3D_SPEED_7,0},
  {BACK_SUB_MODE_RUBBING,WALK_LOCATE_ABSULATE,WAIST_POSITION,KNEAD_RUN_RUBBING,0,KNOCK_STOP,0,0,SPEED_6,_3D_PROGRAM,AXIS_STRONGEST,_3D_SPEED_7,0},
  {BACK_SUB_MODE_RUBBING,WALK_LOCATE_PressNeck,0,KNEAD_RUN_RUBBING,0,KNOCK_STOP,0,0,SPEED_6,_3D_PROGRAM,AXIS_STRONGER,_3D_SPEED_7,0},
  {BACK_SUB_MODE_RUBBING,WALK_LOCATE_ABSULATE,WAIST_POSITION,KNEAD_RUN_RUBBING,0,KNOCK_STOP,0,0,SPEED_6,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},
  //颈部动作组合194
  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_PARK,20,KNEAD_RUN,0,KNOCK_STOP,0,0,SPEED_2,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_SHOULDER,20,KNEAD_RUN,0,KNOCK_STOP,0,0,SPEED_2,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_PARK,30,KNEAD_RUN,0,KNOCK_STOP,0,0,SPEED_2,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_SHOULDER,0,KNEAD_RUN,0,KNOCK_STOP,0,0,SPEED_2,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_PARK,0,KNEAD_RUN_STOP_AT_MIN,3,KNOCK_STOP,0,0,SPEED_2,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},  
  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_PARK,20,KNEAD_RUN,0,KNOCK_STOP,0,0,SPEED_1,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_SHOULDER,20,KNEAD_RUN,0,KNOCK_STOP,0,0,SPEED_1,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_PARK,20,KNEAD_RUN,0,KNOCK_STOP,0,0,SPEED_1,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_TOP,0,KNEAD_RUN,0,KNOCK_STOP,0,0,SPEED_1,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_PARK,20,KNEAD_RUN,0,KNOCK_STOP,0,0,SPEED_1,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_SHOULDER,0,KNEAD_RUN,0,KNOCK_STOP,0,0,SPEED_3,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_PARK,0,KNEAD_RUN_STOP_AT_MIN,1,KNOCK_STOP,0,0,SPEED_3,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_TOP,0,KNEAD_RUN,0,KNOCK_STOP,0,0,SPEED_1,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_PARK,20,KNEAD_RUN,0,KNOCK_STOP,0,0,SPEED_1,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},
  //轻敲到肩部//208
  {BACK_SUB_MODE_SOFT_KNOCK,WALK_LOCATE_PressNeck,0,KNEAD_STOP,0,KNOCK_RUN_STOP,1,4,SPEED_3,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},	
  //轻敲到顶部
  {BACK_SUB_MODE_SOFT_KNOCK,WALK_LOCATE_TOP,0,KNEAD_STOP,0,KNOCK_RUN_STOP,1,4,SPEED_3,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},	
  //轻敲
  {BACK_SUB_MODE_SOFT_KNOCK,WALK_LOCATE_PressNeck,0,KNEAD_STOP,0,KNOCK_RUN_STOP,1,4,SPEED_3,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},	
  //轻敲			////////////////////////////////////
  {BACK_SUB_MODE_SOFT_KNOCK,WALK_LOCATE_SHOULDER,0,KNEAD_STOP,0,KNOCK_RUN_STOP,1,4,SPEED_4,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},	
  //轻敲
  {BACK_SUB_MODE_SOFT_KNOCK,WALK_LOCATE_ABSULATE,60,KNEAD_STOP,0,KNOCK_RUN_STOP,1,4,SPEED_4,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},	
  //轻敲
  {BACK_SUB_MODE_SOFT_KNOCK,WALK_LOCATE_PressNeck,0,KNEAD_STOP,0,KNOCK_RUN_STOP,1,4,SPEED_4,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},	
  //轻敲
  {BACK_SUB_MODE_SOFT_KNOCK,WALK_LOCATE_ABSULATE,0,KNEAD_STOP,0,KNOCK_RUN_STOP,1,4,SPEED_4,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},	
  {BACK_SUB_MODE_SOFT_KNOCK,WALK_LOCATE_PARK,25,KNEAD_STOP,0,KNOCK_RUN_STOP,1,4,SPEED_5,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},	
  {BACK_SUB_MODE_SOFT_KNOCK,WALK_LOCATE_PARK,25,KNEAD_STOP,0,KNOCK_RUN_STOP,1,4,SPEED_6,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},	
  //揉捏到顶/217
  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_PARK,40,KNEAD_RUN,0,KNOCK_STOP,0,0,SPEED_2,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_ABSULATE,40,KNEAD_RUN,0,KNOCK_STOP,0,0,SPEED_3,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_PARK,30,KNEAD_RUN,0,KNOCK_STOP,0,0,SPEED_3,_3D_PROGRAM,AXIS_STRONGER,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_ABSULATE,80,KNEAD_RUN,0,KNOCK_STOP,0,0,SPEED_3,_3D_PROGRAM,AXIS_STRONGER,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_PARK,30,KNEAD_RUN,0,KNOCK_STOP,0,0,SPEED_3,_3D_PROGRAM,AXIS_STRONGEST,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_PressNeck,0,KNEAD_RUN,0,KNOCK_STOP,0,0,SPEED_3,_3D_PROGRAM,AXIS_STRONGEST,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_PARK,30,KNEAD_RUN,0,KNOCK_STOP,0,0,SPEED_3,_3D_PROGRAM,AXIS_STRONGEST,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_NeckMed,0,KNEAD_RUN,0,KNOCK_STOP,0,0,SPEED_3,_3D_PROGRAM,AXIS_STRONGEST,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_PARK,30,KNEAD_RUN,0,KNOCK_STOP,0,0,SPEED_3,_3D_PROGRAM,AXIS_STRONGER,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_SHOULDER,0,KNEAD_RUN,0,KNOCK_STOP,0,0,SPEED_3,_3D_PROGRAM,AXIS_STRONGER,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_PARK,30,KNEAD_RUN,0,KNOCK_STOP,0,0,SPEED_3,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_SHOULDER,20,KNEAD_RUN,0,KNOCK_STOP,0,0,SPEED_3,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_PARK,30,KNEAD_RUN,0,KNOCK_STOP,0,0,SPEED_3,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_TOP,0,KNEAD_RUN,0,KNOCK_STOP,0,0,SPEED_3,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},
  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_PARK,30,KNEAD_RUN,0,KNOCK_STOP,0,0,SPEED_3,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},
  //变换幅度的轻敲/222
  //轻敲到肩部
  {BACK_SUB_MODE_SOFT_KNOCK,WALK_LOCATE_SHOULDER,0,KNEAD_STOP,0,KNOCK_RUN_STOP,1,4,SPEED_3,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},	
  {BACK_SUB_MODE_SOFT_KNOCK,WALK_LOCATE_ABSULATE,0,KNEAD_RUN_STOP_AT_MED,1,KNOCK_RUN_STOP,1,4,SPEED_4,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},	
  {BACK_SUB_MODE_SOFT_KNOCK,WALK_LOCATE_TOP,0,KNEAD_RUN_STOP_AT_MIN,1,KNOCK_RUN_STOP,1,4,SPEED_3,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_7,0},	


} ;


















