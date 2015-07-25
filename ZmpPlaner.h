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
  //void PlanZMPnew(FootType FT, BodyPtr body, vector2 swLegRef_p, Matrix3 object_ref_R, std::deque<vector2> &rfzmp); 

  //void PlanZMPnew(FootType FT, Vector3 *p_ref, Matrix3 *R_ref, vector2 swLegRef_p, Matrix3 object_ref_R, std::deque<vector2> &rfzmp); 

  //void calcSwingLegXYONew(FootType FT, BodyPtr body,  vector2 swLegRef_p, Matrix3 object_ref_R);
  void PlanZMPnew_toe(BodyPtr body, FootType FT, Vector3 *p_ref, Matrix3 *R_ref, vector2 swLegRef_p, Matrix3 object_ref_R, std::deque<vector2> &rfzmp);  
  //void PlanZMPnew_toe_dynamic(BodyPtr body, FootType FT, Vector3 *p_ref, Matrix3 *R_ref, vector2 swLegRef_p, Matrix3 object_ref_R, std::deque<vector2> &rfzmp);  

 void calcSwingLegXYONew(FootType FT, Vector3 *p_ref, Matrix3 *R_ref, vector2 swLegRef_p, Matrix3 object_ref_R);
 void calcSwingLegXYONew_toe(BodyPtr body, FootType FT, Vector3 *p_ref, Matrix3 *R_ref, vector2 swLegRef_p, Matrix3 object_ref_R);

 //void calcWaistR( FootType FT,  Matrix3 *R_ref);
 Matrix3 calcWaistR( FootType FT,  BodyPtr m_robot, string *end_link);
  void atan2adjust(double &pre, double &cur);

  void StopZMP(FootType FT, std::deque<vector2> &rfzmp, int count);

  //capture point/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_
  void PlanCP( BodyPtr m_robot, FootType FT, Vector3 *p_ref, Matrix3 *R_ref, vector2 swLegRef_p, Matrix3 object_ref_R, std::deque<vector2> &rfzmp, bool usePivot, string *end_link);

  void PlanCPstop(BodyPtr m_robot ,FootType FT, Vector3 *p_ref, Matrix3 *R_ref, vector2 swLegRef_p, Matrix3 object_ref_R, std::deque<vector2> &rfzmp, string *end_link); 
 
  void calcSwingLegCP( BodyPtr m_robot, FootType FT, Vector3 *p_ref, Matrix3 *R_ref, vector2 swLegRef_p, Matrix3 object_ref_R, bool usePivot, string *end_link);
  
  void setw(double &wIn);
  void setZmpOffsetX(double &cm_offset_x);

  void getNextCom(Vector3 &cm_ref);
  void setWpgParam(wpgParam param);


  //_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_
  /*
    void ShiftZMP(FootType FT,double *prm, double *prmPre, BodyPtr body, std::deque<vector2> &rfzmp, int count, Vector3 cm, PreviewControl *PCOri, Vector3* p_Init);
    void gaitChange(FootType FT,double *prm, double *prmPre, int count, BodyPtr body,Vector3* p_Init);

  //pc
  void setPreviewControl(double z);
  bool calZmpErr(PreviewControl *PCIn, std::deque<vector2> &rfzmp,int tem, vector2 &offset);
  */

  int step1Num;
  int NomalPaceNum;

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

  double offsetZMPx;
  double offsetZMPy;
  bool stopOper;
  int TsupNum,TdblNum;
  int beforeUpNum;
  double Tsup;
  double Tdbl;
  double ankle_height;

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
  double Tv;
  double dt;
  int it;
  double *zmptemp;
  double pitch_angle;
  double Tp;

  double stopPoint;
  double pitchMax;

};

#endif
