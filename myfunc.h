#ifndef MYFUNC_H
#define MYFUNC_H 

#define NEAR0 1e-8

#include <rtm/idl/BasicDataTypeSkel.h>
#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/CorbaPort.h>
#include <rtm/DataInPort.h>
#include <rtm/DataOutPort.h>

#include <rtm/CorbaNaming.h>
// OpenHRP
//#include "hrpModel/Body.h"
//#include "hrpModel/Link.h"
//#include "hrpModel/JointPath.h"
//#include "hrpModel/ModelLoaderUtil.h"
//#include "hrpUtil/MatrixSolvers.h"
//#include "hrpUtil/EigenTypes.h" 

#include "wuNewType.h"
#include <iostream>
#include <deque>
//#include "SequencePlayerServiceStub.h"

//using namespace std;
//using namespace hrp;
//using namespace RTC;

//void ShowMatrix(Matrix3& M,int k,int l);

double* CalcInterplation1(double xs,double xf,double tf);
void CalcInterplation1Vector2(Vector2 xs, Vector2 xf, double tf, std::deque<Vector2> &DQ);
double* CalcInterplation3(double xs,double dxs,double xf,double dxf,double tf);

template<class T>
void Interplation5(T ps, T dps, T ddps, T pf,T dpf, T ddpf,  double tf, std::deque<T>& input)
{
  //cout<<"IP5 template"<<endl;
  T a0,a1,a2,a3,a4,a5;
  int num=(int)(tf/0.005+NEAR0);

  a0=ps;
  a1=dps;
  a2=ddps/2.0;
  a3=(20.0*pf - 20.0*ps - (8.0*dpf + 12.0*dps)*tf - (3.0*ddps - ddpf)*pow(tf,2))/(2.0*pow(tf,3));
  a4=(30.0*ps - 30.0*pf + (14.0*dpf + 16.0*dps)*tf + (3.0*ddps - 2.0*ddpf)*pow(tf,2))/(2.0*pow(tf,4));
  a5=(12.0*pf - 12.0*ps - (6.0*dpf + 6.0*dps)*tf - (ddps - ddpf)*pow(tf,2))/(2.0*pow(tf,5));

  for(int i=1;i<num+1;i++){
    double ti=0.005*i;
    T tmp;
    tmp=a0+a1*ti+a2*pow(ti,2)+a3*pow(ti,3)+a4*pow(ti,4)+a5*pow(ti,5);//頭抜き
    input.push_back(tmp);
  }
}

template<class T>
void Interplation3(T xs, T dxs, T xf,T dxf, double tf, std::deque<T>& input)
{
  T a0,a1,a2,a3;
  double dt=0.005;
  int num=(int)(tf/dt+NEAR0);//correct
 
  a0=xs;
  a1=dxs;
  //a2=1/(2*pow(tf,2))*(-6*(xs-xf)+2*(dxs-dxf)*tf);
  //a3=1/pow(tf,3)*(2*(xs-xf)-(dxs-dxf)*tf);
  a2=1/pow(tf,2)*(3*(xf-xs)-(2*dxs+dxf)*tf);
  a3=1/pow(tf,3)*(2*(xs-xf)+(dxs+dxf)*tf);
  
  for(int i=1;i<num+1;i++)//頭抜き
    //for(int i=0;i<num;i++)//ビリ抜き
    {
      double ti=dt*i;
      T tmp;
      tmp=a0+a1*ti+a2*pow(ti,2)+a3*pow(ti,3);//頭抜き
      input.push_back(tmp);
    }
}

template<class T>
void Interplation1(T xs,T xf,double tf, std::deque<T>& input)
{
  double dt=0.005;
  int num=(int)(tf/dt+NEAR0);
  T tmp;
  for(int i=1;i<num+1;i++){
    tmp=xs+(xf-xs)*i/num;
    input.push_back(tmp);
  }
}

//double sigma(dvector arr,int x,int y);


double vel(double* Array,int i);

double acc(double* Array,int i);


//void ShowMatrix(dmatrix& M,int k,int l);


double sum(double* Array1st,int n);

////for robot../////
//void RenewModel(BodyPtr body,Vector3  *p_now, Matrix3 *R_now);

void RenewModel(BodyPtr body,Vector3  *p_now, Matrix3 *R_now, string *end_link);

void setModelPosture( BodyPtr body,  TimedDoubleSeq &m_q, FootType FT, string *end_link);

void getModelPosture( BodyPtr body,  TimedDoubleSeq &m_refq);

Matrix3 yowRotate(double& q);

//Vector3 calcZMP(BodyPtr body, TimedDoubleSeq &m_rfsensor, TimedDoubleSeq &m_lfsensor);

//TimedDoubleSeq calcZMPTDS(BodyPtr body, TimedDoubleSeq &m_rfsensor, TimedDoubleSeq &m_lfsensor);

//void calcJs(BodyPtr body, dmatrix &Js);

int swLeg(FootType FT);

bool loadtxt(std::string path,  std::deque<double> &data);

Vector3 velicity(std::deque<Vector3> &deq,int i);

Vector3 acclar(std::deque<Vector3> &deq,int i);

FootType FTselect(BodyPtr body,StepDir dir);

//dmatrix diag(const hrp::dmatrix &in1, const hrp::dmatrix &in2);

//dmatrix diag(const hrp::dmatrix &in1, const hrp::dmatrix &in2, const hrp::dmatrix &in3, const hrp::dmatrix &in4);

Vector3 rot2rpy(Matrix3 R);
matrix22 RfromMatrix3(Matrix3 R);
Matrix3 extractYow(Matrix3 Rin);

vector2 pfromVector3(Vector3 p);

void NaturalZmp(BodyPtr body, Vector3 &absZMP, double offset_x, string *end_link);

void updateInit(Vector3 *p_now, Vector3 *p_Init, Matrix3 *R_now, Matrix3 *R_Init);

bool walkJudge( BodyPtr body, FootType FT, Vector3 RLEG_ref_p, Vector3 LLEG_ref_p, Matrix3  LEG_ref_R, string *end_link);

void adjust_M_PI(double &v);

void atan2adjust(Vector3 &pre, Vector3 &cur);


bool CalcIVK_biped(BodyPtr body, Vector3& CM_p, Vector3 *p_ref, Matrix3 *R_ref, FootType FT, string *end_link);

void CalJo_biped(BodyPtr body, FootType FT, Eigen::MatrixXd& out_J, string *end_link);


bool CalcIVK_biped_toe(BodyPtr body, Vector3& CM_p, Vector3 *p_ref, Matrix3 *R_ref, FootType FT, string *end_link);
void CalJo_biped_toe(BodyPtr body, FootType FT, Eigen::MatrixXd& out_J,string *end_link);


//bool CalcIVK_biped_toe(BodyPtr body, Vector3& CM_p, Vector3 *p_ref, Matrix3 *R_ref, FootType FT, Vector3  *p_Init, Matrix3 *R_Init);
//void CalJo_biped_toe(BodyPtr body, FootType FT,dmatrix& Jacobian);
Matrix3 rodoriges(Vector3 omega, double dt);

void SeqPlay32(vector32 body_cur,   vector32 body_ref,  std::deque<vector32>& bodyDeque, double dt);

Matrix3 rotationZ(double theta);
Matrix3 rotationY(double theta);
#endif
