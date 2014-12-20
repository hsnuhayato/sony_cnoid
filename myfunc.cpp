#include<stdio.h>
#include<fstream>
#include<math.h>
#include "myfunc.h"
//std::ofstream ofs("/home/grxuser/users/wu/hrp2rtc/myfun.log");
/*
void ShowMatrix(Matrix33& M,int k,int l)
{
  for(int i=0;i<3;i++)
    {
      for(int j=0;j<3;j++)
	{
	  std::cout<<M(i,j)<<" ";
	}std::cout<<std::endl;
    }
}
*/
double* CalcInterplation1(double xs,double xf,double tf)
{
 double* tmp;
 double dt=0.005;
 int num=(int)(tf/dt+NEAR0);
 tmp=new double[num];
 for(int i=1;i<num+1;i++)
   {
     tmp[i-1]=xs+(xf-xs)*i/num;
   }


 return tmp;
}

void CalcInterplation1Vector2(Vector2 xs, Vector2 xf, double tf, std::deque<Vector2> &DQ)
{
  DQ.clear();
  double dt=0.005;
  int num=(int)(tf/dt+NEAR0);
  Vector2 DQtem;
  for(int i=1;i<num+1;i++){
    DQtem=xs+(xf-xs)*i/num;
    DQ.push_back(DQtem);
  }
}

double* CalcInterplation3(double xs,double dxs,double xf,double dxf,double tf)
{
  double* tmp;
  double a0,a1,a2,a3;
  double dt=0.005;
  int num=(int)(tf/dt+NEAR0);//correct
  //cout<<"numinC5 "<<num<<"v"<<(int)(tf/dt)<<endl;
  tmp=new double[num];
  //cout<<"iter5 number "<<num<<endl;
  a0=xs;
  a1=dxs;
  a2=1/(2*pow(tf,2))*(-6*(xs-xf)+2*(dxs-dxf)*tf);
  a3=1/pow(tf,3)*(2*(xs-xf)-(dxs-dxf)*tf);

  
  for(int i=1;i<num+1;i++)//頭抜き
    //for(int i=0;i<num;i++)//ビリ抜き
    {
      double ti=dt*i;
      tmp[i-1]=a0+a1*ti+a2*pow(ti,2)+a3*pow(ti,3);//頭抜き
      //tmp[i]=a0+a1*ti+a2*pow(ti,2)+a3*pow(ti,3);//ビリ抜き
      //  cout<<"5inert "<<tmp[i-1]<<endl;
    }
  return tmp;
  delete tmp;
}

/*
 double sigma(dvector arr,int x,int y)
{
  double total=0;
  for(int i=x;i<y+1;i++)
    total+=arr[i];
  return total;
}
*/

double vel(double* Array,int i)
{
  double vel=0;
  vel=(Array[i+1]-Array[i-1])/0.01;
  return vel;
}

double acc(double* Array,int i)
{
  double acc=0;
  acc=(Array[i-1]-2*Array[i]+Array[i+1])/(0.005*0.005);
  return acc;
}

/*
void ShowMatrix(dmatrix& M,int k,int l)
{
  for(int i=0;i<k;i++)
    {
      for(int j=0;j<l;j++)
	{
	  std::cout<<M(i,j)<<" ";
	}std::cout<<std::endl;
    }
}
*/

double sum(double* Array1st,int n)
{
  double total=0;
  for(int i=0;i<n;i++){
    //cout<<"AR "<<Array1st[i]<<endl;
    total+=Array1st[i];}
  return total;
}
////for robot..///
void getModelPosture( BodyPtr body,  TimedDoubleSeq &m_refq)
{
  int dof=body->numJoints();
  for(unsigned int i=0;i<dof;i++)
    m_refq.data[i]=body->joint(i)->q();
}
//old

//new
void setModelPosture( BodyPtr body,  TimedDoubleSeq &m_q, FootType FT, Vector3  *p_Init, Matrix3 *R_Init)
{
  int dof=body->numJoints();
  for(unsigned int i=0;i<dof;i++)
    body->joint(i)->q()=m_q.data[i];

  JointPathPtr Lf2Rf= getCustomJointPath(body, body->link("LLEG_JOINT5"),body->link("RLEG_JOINT5"));
  JointPathPtr Rf2Lf= getCustomJointPath(body, body->link("RLEG_JOINT5"),body->link("LLEG_JOINT5"));
  
  if((FT==FSRFsw)||FT==RFsw){
    body->link("LLEG_JOINT5")->p()=p_Init[LLEG];
    body->link("LLEG_JOINT5")->R()=R_Init[LLEG];//tvmet::identity<matrix33>();
    Lf2Rf->calcForwardKinematics();
  }
  else if((FT==FSLFsw)||FT==LFsw){
    body->link("RLEG_JOINT5")->p()=p_Init[RLEG];
    body->link("RLEG_JOINT5")->R()=R_Init[RLEG];//tvmet::identity<matrix33>();
    Rf2Lf->calcForwardKinematics();
  }
  body->calcForwardKinematics();
}


Matrix3 rotationZ(double theta){
  Matrix3 R;
  double c = cos(theta);
  double s = sin(theta);
  R <<  c,  -s,  0.0,
    s,   c,  0.0,
    0.0, 0.0, 1.0;
  return R;
}

Matrix3 yowRotate(double& q)
{
  Matrix3 Rmatrix;
  double theta=deg2rad(q);
  for(int i=0;i<3;i++)
    {
      Rmatrix(i,2)=Rmatrix(2,i)=0;
    }
  Rmatrix(2,2)=1;
  Rmatrix(0,0)=Rmatrix(1,1)=cos(theta);
  Rmatrix(1,0)=sin(theta);
  Rmatrix(0,1)=-sin(theta);
  return Rmatrix;
}


void RenewModel(BodyPtr body,Vector3  *p_now, Matrix3 *R_now)
{
  string RARM_END,LARM_END;
  if( body->numJoints()==32){
    RARM_END= "RARM_JOINT6";
    LARM_END= "LARM_JOINT6";
  }
  else if( body->numJoints()==30){
    RARM_END= "RARM_JOINT5";
    LARM_END= "LARM_JOINT5";
  }

  p_now[0]=body->link("RLEG_JOINT5")->p();
  p_now[1]=body->link("LLEG_JOINT5")->p();
  p_now[2]=body->link(RARM_END)->p();
  p_now[3]=body->link(LARM_END)->p();
  p_now[4]=body->link("WAIST")->p();
 
  R_now[0]=body->link("RLEG_JOINT5")->R();
  R_now[1]=body->link("LLEG_JOINT5")->R();
  R_now[2]=body->link(RARM_END)->R();
  R_now[3]=body->link(LARM_END)->R();
  R_now[4]=body->link("WAIST")->R();
}

/*
Vector3 calcZMP(BodyPtr body, TimedDoubleSeq &m_rfsensor, TimedDoubleSeq &m_lfsensor)
{
  matrix33 w_R_rf,w_R_lf;
  Vector3 w_P_rf,w_P_lf;
  Vector3 wZMP_R,wZMP_L, wZMP;
  Vector3 rZMP_R,lZMP_L;
  double d = 0.01;//足底からセンサ中心の距離（適当に0.01[m]とした）
  rZMP_R << (-m_rfsensor.data[4] - m_rfsensor.data[0]*d)/m_rfsensor.data[2],
           ( m_rfsensor.data[3] - m_rfsensor.data[1]*d)/m_rfsensor.data[2],
            -0.135;
  lZMP_L << (-m_lfsensor.data[4] - m_lfsensor.data[0]*d)/m_lfsensor.data[2],
           ( m_lfsensor.data[3] - m_lfsensor.data[1]*d)/m_lfsensor.data[2],
            -0.135;
  
    w_P_rf=body->link("RLEG_JOINT5")->p();
    w_P_lf=body->link("LLEG_JOINT5")->p();
    // w_P_lf(i)=body->link("LLEG_JOINT5")->p()[i];
    w_R_rf=body->link("RLEG_JOINT5")->R;
    w_R_lf=body->link("LLEG_JOINT5")->R;
    
    wZMP_R = w_P_rf + w_R_rf*rZMP_R;
    wZMP_L = w_P_lf + w_R_lf*lZMP_L;
    
    wZMP[0] = (wZMP_R(0)*m_rfsensor.data[2] + wZMP_L(0)*m_lfsensor.data[2])/(m_rfsensor.data[2] + m_lfsensor.data[2]);
    wZMP[1] = (wZMP_R(1)*m_rfsensor.data[2] + wZMP_L(1)*m_lfsensor.data[2])/(m_rfsensor.data[2] + m_lfsensor.data[2]);
    wZMP[2] = wZMP_R(2);
  
  return wZMP;
}
*/

 /*
TimedDoubleSeq calcZMPTDS(BodyPtr body, TimedDoubleSeq &m_rfsensor, TimedDoubleSeq &m_lfsensor)
{
  matrix33 w_R_rf,w_R_lf;
  Vector3 w_P_rf,w_P_lf;
  Vector3 wZMP_R,wZMP_L;
  Vector3 rZMP_R,lZMP_L;
  TimedDoubleSeq wZMP;
  double d = 0.01;//足底からセンサ中心の距離（適当に0.01[m]とした）

  wZMP.data.length(3); 
  rZMP_R << (-m_rfsensor.data[4] - m_rfsensor.data[0]*d)/m_rfsensor.data[2],
           ( m_rfsensor.data[3] - m_rfsensor.data[1]*d)/m_rfsensor.data[2],
            -0.135;
  lZMP_L << (-m_lfsensor.data[4] - m_lfsensor.data[0]*d)/m_lfsensor.data[2],
           ( m_lfsensor.data[3] - m_lfsensor.data[1]*d)/m_lfsensor.data[2],
            -0.135;
  
    w_P_rf=body->link("RLEG_JOINT5")->p();
    w_P_lf=body->link("LLEG_JOINT5")->p();
    // w_P_lf(i)=body->link("LLEG_JOINT5")->p()[i];
    w_R_rf=body->link("RLEG_JOINT5")->R;
    w_R_lf=body->link("LLEG_JOINT5")->R;
    
    wZMP_R = w_P_rf + w_R_rf*rZMP_R;
    wZMP_L = w_P_lf + w_R_lf*lZMP_L;
    
    wZMP.data[0] = (wZMP_R(0)*m_rfsensor.data[2] + wZMP_L(0)*m_lfsensor.data[2])/(m_rfsensor.data[2] + m_lfsensor.data[2]);
    wZMP.data[1] = (wZMP_R(1)*m_rfsensor.data[2] + wZMP_L(1)*m_lfsensor.data[2])/(m_rfsensor.data[2] + m_lfsensor.data[2]);
    wZMP.data[2] = wZMP_R(2);
  
  return wZMP;

}
 */
/*
void calcJs(BodyPtr body, MatrixXd &Js)
{
  int dof=body->numJoints();
  Js=Eigen::MatrixXd::Identity(dof,dof);
  int id = body->link("CHEST_JOINT0")->jointId;
  Js(id,id)=0;
  id = body->link("CHEST_JOINT1")->jointId;
  Js(id,id)=0;
  id = body->link("HEAD_JOINT0")->jointId;
  Js(id,id)=0;
  id = body->link("HEAD_JOINT1")->jointId;
  Js(id,id)=0;
  //ShowMatrix(Js, dof, dof);
}
*/


int swLeg(FootType FT)
{
  //swingLeg
    int swingLeg;
    if((FT==FSRFsw)||(FT==RFsw)){
      return RLEG;
    }
    else if((FT==FSLFsw)||(FT==LFsw)){
      return LLEG;
    }

}

bool loadtxt(std::string path,  std::deque<double> &data)
{
  ifstream file(path.c_str());
  if( !file )
    return false;

  string st;
  while(getline(file, st)) {
    if( st.size() > 1 ) {
      stringstream ss;
      ss.str(st);
      double tmp;
      ss >> tmp;
      data.push_back(tmp);
      //ofs<<tmp<<endl;
    }
  }
  file.close();

  return true;
}

Vector3 velicity(std::deque<Vector3> &deq, int i)
{
  Vector3 vel=Vector3d::Zero(3);
  vel=(deq.at(i+1)-deq.at(i-1))/0.01;
  return vel;
}

Vector3 acclar(std::deque<Vector3> &deq, int i)
{
  Vector3 acc=Vector3d::Zero(3);
  acc=(deq.at(i-1)-2*deq.at(i)+deq.at(i+1))/(0.005*0.005);
  return acc;
}


FootType FTselect(BodyPtr body,StepDir dir)
{
  FootType FT;
  //rf is front //initial pos rleg is behind lleg
  if( body->link("RLEG_JOINT5")->p()(0) >= body->link("LLEG_JOINT5")->p()(0) ){
    if(dir==front)
      FT=FSLFsw;
    else
      FT=FSRFsw;
  }
  else{//lf is front
     if(dir==front)
      FT=FSRFsw;
    else
      FT=FSLFsw;
  }

  return FT;

}

//----------------------------------------------------------------------------------
//  math function
//----------------------------------------------------------------------------------
/*
hrp::dmatrix diag(const hrp::dmatrix &in1, const hrp::dmatrix &in2)
{
  int size1 = in1.rows() + in2.rows();
  int size2 = in1.cols() + in2.cols();

  hrp::dmatrix out(Eigen::MatrixXd::Zero(size1,size2)); 
  
  for(int i = 0; i < in1.rows(); i++)
    for(int j = 0; j < in1.cols(); j++)
      out(i,j) = in1(i,j);


  int shift1 = in1.rows();
  int shift2 = in1.cols();
  for(int i = 0; i < in2.rows(); i++)
    for(int j = 0; j < in2.cols(); j++)
      out(i+shift1,j+shift2) = in2(i,j);

  return out;
}

hrp::dmatrix diag(const hrp::dmatrix &in1, const hrp::dmatrix &in2, const hrp::dmatrix &in3, const hrp::dmatrix &in4)
{
  int size1 = in1.rows() + in2.rows()+in3.rows() + in4.rows();
  int size2 = in1.cols() + in2.cols() + in3.cols() + in4.cols();

  hrp::dmatrix out(Eigen::MatrixXd::Zero(size1,size2)); 
   
  for(int i = 0; i < in1.rows(); i++)
    for(int j = 0; j < in1.cols(); j++)
      out(i,j) = in1(i,j);


  int shift1 = in1.rows();
  int shift2 = in1.cols();
  for(int i = 0; i < in2.rows(); i++)
    for(int j = 0; j < in2.cols(); j++)
      out(i+shift1,j+shift2) = in2(i,j);

  shift1 += in2.rows();
  shift2 += in2.cols();
  for(int i = 0; i < in3.rows(); i++)
    for(int j = 0; j < in3.cols(); j++)
      out(i+shift1,j+shift2) = in3(i,j);

  shift1 += in3.rows();
  shift2 += in3.cols();
  for(int i = 0; i < in4.rows(); i++)
    for(int j = 0; j < in4.cols(); j++)
      out(i+shift1,j+shift2) = in4(i,j);


  return out;
}
*/
matrix22 RfromMatrix3(Matrix3 Rin)//bimyou
{
  Vector3 rpy=rpyFromRot(Rin);
  Matrix3 R=rotationZ(rpy(2));
  matrix22 R_xy;
  for (int i=0;i<2;i++){
    for(int j=0;j<2;j++){
      R_xy(i,j)=R(i,j);
    }
  }

  return R_xy;
}

Matrix3 extractYow(Matrix3 Rin)
{
  //Vector3 rpy=rpyFromRot(Rin);
  //Matrix3 R=rotationZ(rpy(2));

  Vector3 euler = Rin.eulerAngles(2,1,0);
  Matrix3 R=rotationZ(euler(0));

  return R;
}


Vector2 pfromVector3(Vector3 p)
{
  Vector2 p_xy;
  p_xy(0)=p(0);
  p_xy(1)=p(1);
  return p_xy;
}

void NaturalZmp(BodyPtr body, Vector3 &absZMP)
{
  matrix22 RLeg_R(RfromMatrix3(body->link("RLEG_JOINT5")->R()));
  matrix22 LLeg_R(RfromMatrix3(body->link("LLEG_JOINT5")->R()));
  Vector2 pfzmp;
  Vector2 zmpoffset(MatrixXd::Zero(2,1));
  pfzmp(0)=(body->link("RLEG_JOINT5")->p()(0) + body->link("LLEG_JOINT5")->p()(0))/2;
  pfzmp(1)=(body->link("RLEG_JOINT5")->p()(1) + body->link("LLEG_JOINT5")->p()(1))/2;
  pfzmp= pfzmp + (RLeg_R*zmpoffset + LLeg_R*zmpoffset)/2;
  
  absZMP(0)=pfzmp(0);
  absZMP(1)=pfzmp(1);
  absZMP(2)=0; 
}

void clacWaistR(BodyPtr body, FootType FT,Matrix3&  R_ref_WAIST, double swLegYow)
{
  Vector3 sup_leg_rpy;
  Matrix3 sup_leg_R; 
  if((FT==FSRFsw)||(FT==RFsw)){
    sup_leg_rpy=rpyFromRot(body->link("LLEG_JOINT5")->R());
    sup_leg_R=body->link("LLEG_JOINT5")->R();
  }
  else if((FT==FSLFsw)||(FT==LFsw)){
    sup_leg_rpy=rpyFromRot(body->link("RLEG_JOINT5")->R());
    sup_leg_R=body->link("RLEG_JOINT5")->R();
  } 

  double YowAvg= (swLegYow +  sup_leg_rpy(2))/2;
  R_ref_WAIST=rotationZ(YowAvg);
 
}

void updateInit(Vector3 *p_now, Vector3 *p_Init, Matrix3 *R_now, Matrix3 *R_Init)
{//update footprint place leg only
   for(int i=0;i<LINKNUM;i++){
     p_Init[i]=p_now[i];
     R_Init[i]=R_now[i];
   }
}

bool walkJudge(BodyPtr body, Vector3 *p_ref, Matrix3 *R_ref, FootType FT, Vector3 RLEG_ref_p, Vector3 LLEG_ref_p, Matrix3  LEG_ref_R)
{
  Vector3 FErr_R(VectorXd::Zero(3));
  Vector3 FErr_L(VectorXd::Zero(3));
  Vector3 omega_err_R(VectorXd::Zero(3));
  Vector3 omega_err_L(VectorXd::Zero(3));

  //use parent p calculate XX
  //body->link("RLEG_JOINT5")->p better

  //usuall
  /*
  FErr_R=  RLEG_ref_p - p_ref[RLEG]; 
  FErr_L=  LLEG_ref_p - p_ref[LLEG]; 
  omega_err_R=R_ref[RLEG] * omegaFromRot(Matrix3(trans(R_ref[RLEG]) * LEG_ref_R));
  omega_err_L=R_ref[LLEG] * omegaFromRot(Matrix3(trans(R_ref[LLEG]) * LEG_ref_R));
  */
  //for toe mode
  FErr_R=  RLEG_ref_p - body->link("RLEG_JOINT5")->p(); 
  FErr_L=  LLEG_ref_p - body->link("LLEG_JOINT5")->p(); 
  Matrix3 temR_R=extractYow(body->link("RLEG_JOINT5")->R());
  Matrix3 temR_L=extractYow(body->link("LLEG_JOINT5")->R());
  omega_err_R=temR_R * omegaFromRot(temR_R.transpose() * LEG_ref_R);
  omega_err_L=temR_L * omegaFromRot(temR_L.transpose() * LEG_ref_R);
 
  bool start2walk=0;

  if((sqrt(FErr_L(0)*FErr_L(0)+FErr_L(1)*FErr_L(1))>0.03)||(sqrt(FErr_R(0)*FErr_R(0)+FErr_R(1)*FErr_R(1))>0.03))
    start2walk=1;//start to walk
  if(omega_err_R.dot(omega_err_R)>0.03||(omega_err_L.dot(omega_err_L)>0.03))
    start2walk=1;//start to walk

  return start2walk;
}

void adjust_M_PI(double &v)
{
  while(true) {
    if( v > M_PI )
      v -= ( 2 * M_PI );
    else if( v < -M_PI )
      v += ( 2 * M_PI );
    else
      break;
  }
  
}

void atan2adjust(Vector3 &pre, Vector3 &cur)
{
  for(int i=0;i<3;i++){
    if(fabs(pre(i)- cur(i))>M_PI){
      if(pre(i) > cur(i) ){
	while(fabs(pre(i) - cur(i))>M_PI)
	  cur(i)+=2*M_PI;
      }
      else if(pre(i) < cur(i) ){
	while(fabs(pre(i) - cur(i))>M_PI)
	  cur(i)-=2*M_PI;
      }   
    }// if(fabs(pre - cur)>M_PI)
  }//for

  pre=cur;
}


bool CalcIVK_biped(BodyPtr body, Vector3& CM_p, Vector3 *p_ref, Matrix3 *R_ref, FootType FT, Vector3  *p_Init, Matrix3 *R_Init)
{
  Link* SupLeg;
  Link* SwLeg;
  JointPathPtr SupLeg2SwLeg,SupLeg2W;
  Vector3 cm=body->calcCenterOfMass();
  Vector3  SwLeg_p; 
  Matrix3 W_R,SwLeg_R;  
 
  Eigen::ColPivHouseholderQR<MatrixXd> QR;
  //careful
  MatrixXd Jacobian=MatrixXd::Zero(12,12);//leg only
  int swingLegId;

  if((FT==FSRFsw)||FT==RFsw){
    SupLeg=body->link("LLEG_JOINT5");
    SwLeg=body->link("RLEG_JOINT5");
    swingLegId=0;
  }
  else if((FT==FSLFsw)||FT==LFsw){
    SupLeg=body->link("RLEG_JOINT5");
    SwLeg=body->link("LLEG_JOINT5");
    swingLegId=1;
  }
  SupLeg2SwLeg= getCustomJointPath(body, SupLeg, SwLeg);  
  SupLeg2W= getCustomJointPath(body, SupLeg,body->link("WAIST"));   
  
  static const int MAX_IK_ITERATION = 50;
  static const double LAMBDA = 0.9;
  
  const int n = 12;//leg only
  
  //Link* target = jpp->endLink();
  
  std::vector<double> qorg(n);
    for(int i=0; i < 12; ++i){
      qorg[i] = body->joint(i)->q();
    }
    
    //careful
    VectorXd dq(n);
    VectorXd v(12);
    double maxIKErrorSqr=1.0e-16;
    
    bool converged = false;
  
    for(int i=0; i < MAX_IK_ITERATION; i++){
      
      W_R=body->link("WAIST")->R();
      SwLeg_p = SwLeg->p();
      SwLeg_R = SwLeg->R();
      //tag
      CalJo_biped(body, FT, Jacobian);        
      ///

      Vector3 CM_dp=CM_p- body->calcCenterOfMass();// + body->link("LLEG_JOINT5")->p);
      Vector3 W_omega=W_R* omegaFromRot(W_R.transpose() * R_ref[WAIST]);
      Vector3 SW_dp =p_ref[swingLegId] - SwLeg_p;  
      Vector3 SW_omega = SwLeg_R* omegaFromRot(SwLeg_R.transpose() * R_ref[swingLegId]);
     
      //v<< CM_dp, W_omega, SW_dp, SW_omega;
      v.head<3>()=CM_dp;
      v.segment<3>(3)=W_omega;
      v.segment<3>(6)=SW_dp;
      v.segment<3>(9)=SW_omega;

      //double errsqr =  CM_dp.dot(CM_dp) + W_omega.dot(W_omega) + SW_dp.dot(SW_dp) + SW_omega.dot( SW_omega);
      double errsqr=v.squaredNorm();

      if(errsqr < maxIKErrorSqr){
	converged = true;
	break;
      }
      
      /*
      //solveLinearEquationSVD(Jacobian, v, dq);  // dq = pseudoInverse(J) * v
      //if(solveLinearEquationSVD(Jacobian, v, dq)!=0){
      if(solveLinearEquationLU(Jacobian, v, dq)!=0){
	std::cerr<<"SVD out"<<std::endl;
	converged=0;
	break;
      }
      */
      MatrixXd JJ;
      double dampingConstantSqr=1e-12;
      JJ = Jacobian *  Jacobian.transpose() + dampingConstantSqr * MatrixXd::Identity(Jacobian.rows(), Jacobian.rows());
      dq = Jacobian.transpose() * QR.compute(JJ).solve(v);


      for(int j=0; j < n; ++j){
	body->joint(j)->q() += LAMBDA * dq(j);
      }
     
      //A
      if((FT==FSRFsw)||FT==RFsw){
	body->link("LLEG_JOINT5")->p()=p_Init[LLEG];
	body->link("LLEG_JOINT5")->R()=R_Init[LLEG];
      }
      else if((FT==FSLFsw)||FT==LFsw){
	body->link("RLEG_JOINT5")->p()=p_Init[RLEG];
	body->link("RLEG_JOINT5")->R()=R_Init[RLEG];
      }
      SupLeg2SwLeg->calcForwardKinematics();
      body->calcForwardKinematics();
      
    }//for
    
    if(!converged){
      
      for(int j=0; j < n; ++j){
	body->joint(j)->q() = qorg[j];
      }
    
      //A
      if((FT==FSRFsw)||FT==RFsw){
	body->link("LLEG_JOINT5")->p()=p_Init[LLEG];
	body->link("LLEG_JOINT5")->R()=R_Init[LLEG];//tvmet::identity<matrix33>();
      }
      else if((FT==FSLFsw)||FT==LFsw){
	body->link("RLEG_JOINT5")->p()=p_Init[RLEG];
	body->link("RLEG_JOINT5")->R()=R_Init[RLEG];//tvmet::identity<matrix33>();
      }
      
      SupLeg2SwLeg->calcForwardKinematics();
      body->calcForwardKinematics();
    }
    
    return converged;          
}


//////ivk_jacobian/////////
void CalJo_biped(BodyPtr body, FootType FT, Eigen::MatrixXd& out_J)
{ 
  Link* SupLeg;
  Link* SwLeg;
  JointPathPtr  SupLeg2SwLeg,SupLeg2W;
  MatrixXd JSupLeg2SwLeg ,JSupLeg2W,Jcom;

  
  if((FT==FSRFsw)||FT==RFsw){
    SupLeg=body->link("LLEG_JOINT5");
    SwLeg=body->link("RLEG_JOINT5");
  }
  else if((FT==FSLFsw)||FT==LFsw){
    SupLeg=body->link("RLEG_JOINT5");
    SwLeg=body->link("LLEG_JOINT5");
  }
  SupLeg2SwLeg= getCustomJointPath(body, SupLeg, SwLeg);  
  SupLeg2W= getCustomJointPath(body, SupLeg,body->link("WAIST"));   

  body->calcCenterOfMass();
  calcCMJacobian(body, SupLeg, Jcom);

  SupLeg2SwLeg->calcJacobian(JSupLeg2SwLeg);
  SupLeg2W->calcJacobian(JSupLeg2W);
  
  
  //push in
  for(int i=0;i<3;i++){
    for(int j=0;j<12;j++){
      out_J(i,j)=Jcom(i,j);
    }
  }
  
  // SupLeg2W
  for(int i = 0; i <  SupLeg2W->numJoints(); i++) {
    int id =  SupLeg2W->joint(i)->jointId();
    for(int j=3;j<6;j++){
      out_J(j,id)=JSupLeg2W(j,i);
    }
  }
  
  //SupLeg2SwLeg
  for(int i = 0; i < SupLeg2SwLeg->numJoints(); i++) {
    int id = SupLeg2SwLeg->joint(i)->jointId();
    for(int j=6;j<12;j++){
      out_J(j,id)=JSupLeg2SwLeg(j-6,i);
    }
  }
  
}

/*
///ivk_toe
bool CalcIVK_biped_toe(BodyPtr body, Vector3& CM_p, Vector3 *p_ref, Matrix3 *R_ref, FootType FT, Vector3  *p_Init, Matrix3 *R_Init)
{
  Link* SupLeg;
  Link* SwLeg;
  JointPathPtr SupLeg2SwLeg;
  Vector3 cm=body->calcCM();
  Vector3  SwLeg_p; 
  Matrix3 W_R,SwLeg_R;  
 
  //careful
  dmatrix Jacobian=dzeromatrix(12,12);//leg only
  int swingLegId;

  if((FT==FSRFsw)||FT==RFsw){
    SupLeg=body->link("LLEG_JOINT5")->child;
    SwLeg=body->link("RLEG_JOINT5")->child;
    swingLegId=0;
  }
  else if((FT==FSLFsw)||FT==LFsw){
    SupLeg=body->link("RLEG_JOINT5")->child;
    SwLeg=body->link("LLEG_JOINT5")->child;
    swingLegId=1;
  }
  SupLeg2SwLeg=JointPathPtr(new JointPath(SupLeg,SwLeg));
   
  static const int MAX_IK_ITERATION = 50;
  static const double LAMBDA = 0.9;
  
  const int n = 12;//leg only
  
  //Link* target = jpp->endLink();
  
  std::vector<double> qorg(n);
    for(int i=0; i < 12; ++i){
      qorg[i] = body->joint(i)->q;
    }
    
    dvector dq(n);
    //careful
    dvector v(12);
    double maxIKErrorSqr=1.0e-16;
    
    bool converged = false;
  
    for(int i=0; i < MAX_IK_ITERATION; i++){
      
      W_R=body->link("WAIST")->R;
      SwLeg_p = SwLeg->p();
      SwLeg_R = SwLeg->R;


      //tag
      CalJo_biped_toe(body, FT, Jacobian);        
      ///

      Vector3 CM_dp(CM_p- body->calcCM());// + body->link("LLEG_JOINT5")->p());
      Vector3 W_omega(W_R* omegaFromRot(Matrix3(trans(W_R) * R_ref[4])));
      Vector3 SW_dp;      
      Vector3 SW_omega;
      SW_dp= p_ref[swingLegId] - SwLeg_p;      
      SW_omega = SwLeg_R* omegaFromRot(Matrix3(trans(SwLeg_R) * R_ref[swingLegId]));
      
      double errsqr =  dot(CM_dp,CM_dp) + dot(W_omega, W_omega) + dot(SW_dp,SW_dp) + dot(SW_omega, SW_omega);
      if(errsqr < maxIKErrorSqr){
	converged = true;
	break;
      }
      
      setVector3(CM_dp   , v, 0);
      setVector3(W_omega, v, 3);
      setVector3(SW_dp, v, 6);
      setVector3(SW_omega, v, 9);
    
      //cerr<<v<<endl;
      //solveLinearEquationSVD(Jacobian, v, dq);  // dq = pseudoInverse(J) * v
      //if(solveLinearEquationSVD(Jacobian, v, dq)!=0){
      if(solveLinearEquationLU(Jacobian, v, dq)!=0){
	std::cerr<<"SVD out"<<std::endl;
	converged=0;
	break;
      }
      for(int j=0; j < n; ++j){
	body->joint(j)->q += LAMBDA * dq(j);
      }
     
      //A
      if((FT==FSRFsw)||FT==RFsw){
	body->link("LLEG_JOINT5")->child->p()=p_Init[LLEG];
	body->link("LLEG_JOINT5")->child->R()=R_Init[LLEG];
      }
      else if((FT==FSLFsw)||FT==LFsw){
	body->link("RLEG_JOINT5")->child->p()=p_Init[RLEG];
	body->link("RLEG_JOINT5")->child->R()=R_Init[RLEG];
      }
      SupLeg2SwLeg->calcForwardKinematics();
      body->calcForwardKinematics();
     
    }//for
    
    if(!converged){
 
      for(int j=0; j < n; ++j){
	body->joint(j)->q = qorg[j];
      }
    
      //A
      if((FT==FSRFsw)||FT==RFsw){
	body->link("LLEG_JOINT5")->child->p()=p_Init[LLEG];
	body->link("LLEG_JOINT5")->child->R()=R_Init[LLEG];//tvmet::identity<matrix33>();
      }
      else if((FT==FSLFsw)||FT==LFsw){
	body->link("RLEG_JOINT5")->child->p()=p_Init[RLEG];
	body->link("RLEG_JOINT5")->child->R()=R_Init[RLEG];//tvmet::identity<matrix33>();
      }
      SupLeg2SwLeg->calcForwardKinematics();
      body->calcForwardKinematics();
   
    }
    
    return converged;          
}

//////ivk_jacobian_toe/////////
void CalJo_biped_toe(BodyPtr body, FootType FT,dmatrix& Jacobian)
{ 
  Link* SupLeg;
  Link* SwLeg;
  JointPathPtr  SupLeg2SwLeg,SupLeg2W;
  dmatrix JSupLeg2SwLeg ,JSupLeg2W,Jcom;

  if((FT==FSRFsw)||FT==RFsw){
    SupLeg=body->link("LLEG_JOINT5")->child;
    SwLeg=body->link("RLEG_JOINT5")->child;
  }
  else if((FT==FSLFsw)||FT==LFsw){
    SupLeg=body->link("RLEG_JOINT5")->child;
    SwLeg=body->link("LLEG_JOINT5")->child;
  }
  SupLeg2SwLeg=JointPathPtr(new JointPath(SupLeg,SwLeg));
  SupLeg2W= JointPathPtr(new JointPath(SupLeg,body->link("WAIST")));   

  body->calcCM();
  body->calcCMJacobian(SupLeg->parent, Jcom);

  SupLeg2SwLeg->calcJacobian(JSupLeg2SwLeg);
  SupLeg2W->calcJacobian(JSupLeg2W);
  
  //push in
  for(int i=0;i<3;i++){
    for(int j=0;j<12;j++){
      Jacobian(i,j)=Jcom(i,j);
    }
  }
  
  // SupLeg2W
  for(int i = 0; i <  SupLeg2W->numJoints(); i++) {
    int id =  SupLeg2W->joint(i)->jointId;
    for(int j=3;j<6;j++){
      Jacobian(j,id)=JSupLeg2W(j,i);
    }
  }
  
  //SupLeg2SwLeg
  for(int i = 0; i < SupLeg2SwLeg->numJoints(); i++) {
    int id = SupLeg2SwLeg->joint(i)->jointId;
    for(int j=6;j<12;j++){
      Jacobian(j,id)=JSupLeg2SwLeg(j-6,i);
    }
  }
    
}
*/


void SeqPlay32(vector32 body_cur,   vector32 body_ref,  std::deque<vector32>& bodyDeque, double dt)
{
  vector32 zero(MatrixXd::Zero(32,1));
  Interplation5(body_cur,  zero,  zero, body_ref,  zero,  zero, dt, bodyDeque);
}

Matrix3 rodoriges(Vector3 omega, double dt)
{
  //Vector3 omega_norm 
  double w=omega.norm();

  if (w<1.0e-3)
    omega=Vector3(MatrixXd::Zero(3,1));
  else
    omega= omega/w;
  Matrix3 hat_a(hat(omega));
  
  Matrix3 E(MatrixXd::Identity(3,3));
  Matrix3 R;

  R=E + hat_a*sin(w*dt)+ hat_a*hat_a*(1-cos(w*dt));
  return R;
}


/*
//print dmatrix
  int row,column;
  row=gg.rows();
  column=gg.cols();
  
  for(int i=0;i<row;i++){
  for(int j=0;j<column;j++){
  ofs<<gg(i,j)<<" ";
  }ofs<<std::endl;
  }
  ofs<<'\n'<<endl;
*/
