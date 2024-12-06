

#ifndef _SF_BLDC_SHARED_STRUCT_h
#define _SF_BLDC_SHARED_STRUCT_h


struct SF_BLDC_DATA {
  float M0_Angle;
  float M0_ElecAngle;
  float M0_Vel;
  float M0_Uq;
  float M0_Ud;
  float M0_Iq;
  float M0_Id;

  float M1_Angle;
  float M1_ElecAngle;
  float M1_Vel;
  float M1_Uq;
  float M1_Ud;
  float M1_Iq;
  float M1_Id;
};

struct SF_BLDC_Wireless_DATA {
  float M0_Target;
  float M1_Target;

  float M0_Mode;
  float M1_Mode;

  //PID 无线设置相关
  //M0
  float M0_POS_P;
  float M0_POS_I;
  float M0_POS_D;
  float M0_POS_LIM;

  float M0_VEL_P;
  float M0_VEL_I;
  float M0_VEL_D;
  float M0_VEL_LIM;
  
  float M0_CURR_P;
  float M0_CURR_I;
  float M0_CURR_D;
  float M0_CURR_LIM;

  //M1
  float M1_POS_P;
  float M1_POS_I;
  float M1_POS_D;
  float M1_POS_LIM;

  float M1_VEL_P;
  float M1_VEL_I;
  float M1_VEL_D;
  float M1_VEL_LIM;
  
  float M1_CURR_P;
  float M1_CURR_I;
  float M1_CURR_D;
  float M1_CURR_LIM;
};

#endif
