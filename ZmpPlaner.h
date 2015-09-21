#ifndef ZMPPLANER_H
#define ZMPPLANER_H

#include <deque>

//#define NEAR0 1e-8
// OpenHRP
//#include "hrpModel/Body.h"
//#include "hrpModel/Link.h"
//#include "hrpModel/JointPath.h"
//#include "hrpModel/ModelLoaderUtil.h"
//#include "hrpUtil/MatrixSolvers.h"

//#include "hrpUtil/uBlasCommonTypes.h"

#include "myfunc.h"
#include "spline.h"
#include "wuNewType.h"
//use preview control operater
#include "preview_control/PreviewControl.h"
class ZmpPlaner {
  
 public:
  ZmpPlaner();
  //ZmpPlaner(FootType FT, double *prm);
  ~ZmpPlaner();
  void setInit(vector2 &Ini);
  void setInit(double &xIni, double &yIni);
  //void calcWaistR( FootType FT,  Matrix3 *R_ref);
  Matrix3 calcWaistR( FootType FT,  BodyPtr m_robot, string *end_link);
  void atan2adjust(double &pre, double &cur);

  //void StopZMP(FootType FT, std::deque<vector2> &rfzmp, int count);

  //capture point/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_
  void PlanCP( BodyPtr m_robot, FootType FT, Vector3 *p_ref, Matrix3 *R_ref, Vector3 swLegRef_p, Matrix3 input_ref_R, std::deque<vector2> &rfzmp, bool usePivot, string *end_link, bool ifLastStep = 0);

  void PlanCPstop(BodyPtr m_robot ,FootType FT, Vector3 *p_ref, Matrix3 *R_ref, Vector3 swLegRef_p, Matrix3 input_ref_R, std::deque<vector2> &rfzmp, string *end_link); 
 
  void calcSwingLegCP( BodyPtr m_robot, FootType FT, Vector3 *p_ref, Matrix3 *R_ref, Vector3 swLegRef_p, Matrix3 object_ref_R, bool usePivot, string *end_link);
  
  void setw(double &cm_z_in);
  void setZmpOffsetX(double &cm_offset_x);

  void getNextCom(Vector3 &cm_ref);
  void setWpgParam(wpgParam param);

  std::deque<Vector3> swingLegTraj;//x y theta
  std::deque<vector2> swLegxy;
  std::deque<double> Trajzd;
  std::deque<Matrix3> swLeg_R;
  std::deque<Matrix3> rot_pitch;
  std::deque<double> index;
  //for pitch
  std::deque<Vector3> link_b_deque;
  Vector3 link_b_front;
  Vector3 link_b_rear;
  Vector3 link_b_ankle;

  double offsetZMPx;
  double offsetZMPy;
  double offsetZMPy_stepping;
  bool stopOper;
 
  int beforeUpNum;
  double Tsup;
  double Tsup_in;
  double Tsup_stepping_in;
  double Tdbl;
  double Tdbl_in;
  double ankle_height;

  deque<double> cm_z_deque;

  //for capture point
  std::deque<vector2> cp_deque;
  vector2 cp;//last cp of one step
  double w;
  vector2 cZMP;
  vector2 cm_vel;
  ///

 private:
  //new
  vector2 zmpInit;
  vector2 offsetZMPr;
  vector2 offsetZMPl; 
  double Zup;
  double Zup_in;
  double Tv;
  double dt;
  int it;
  double *zmptemp;
  double pitch_angle;
  double Tp;
  
  double cm_z;
  double cm_z_cur;
  
  double stopPoint;
  double pitchMax;

};

#endif
