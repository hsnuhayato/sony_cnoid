#include<stdio.h>
#include<fstream>
#include<math.h>

#include "ZmpPlaner.h"


//std::ofstream ofszmp("/home/wu/src/HRP3.1x/sony_cnoid/zmp.log");

ZmpPlaner::ZmpPlaner()
{
  /*
  Tsup=0.7;
  Tdbl=0.1;//for preview
  //Tdbl is set in calcSwingLegCP no necessery set here

  dt=0.005;
  offsetZMPy=0.01;//usually
  offsetZMPy=0.025;
  offsetZMPx=0.015;
  Zup=0.05;
  Tv=0.0;
  //Tv=0.08; //use no toe mode
   //pivot
  pitch_angle=10*M_PI/180;
  //Tp=Tdbl/4;
  Tp=Tdbl;
  link_b_front<< 0.13, 0.0, -0.105;
  link_b_rear<< -0.1, 0.0, -0.105;
  */

  //capture point init
  cp<<0.0, 0.0;


  std::cout<<"zmpplaner "<<std::endl;
  stopOper=1;

  
}
 
void ZmpPlaner::setWpgParam(wpgParam param)
{
  Tsup=param.Tsup;
  Tp=Tdbl=param.Tdbl;
  dt=param.dt;
  offsetZMPy=param.offsetZMPy;
  offsetZMPx=param.offsetZMPx;
  Zup=param.Zup;
  Tv=param.Tv;
  pitch_angle=param.pitch_angle*M_PI/180;
  link_b_front<<param.link_b_front[0],param.link_b_front[1],param.link_b_front[2];
  link_b_rear<<param.link_b_rear[0],param.link_b_rear[1],param.link_b_rear[2];


  //new
  offsetZMPr(0)=offsetZMPl(0)=offsetZMPx;
  offsetZMPr(1)= offsetZMPy;
  offsetZMPl(1)=-offsetZMPy;
  ///
  TsupNum=(int)(Tsup/dt+NEAR0);
  TdblNum=(int)(Tdbl/dt+NEAR0);
  step1Num=(int)(1.5*TsupNum+7*0.25*TdblNum+NEAR0);
  NomalPaceNum=(int)(TsupNum+0.5*TdblNum+NEAR0);

  /*
  cout<<Tsup<<" "<<Tdbl<<" "<<offsetZMPy<<" "<<offsetZMPx<<" "<<Zup<<" "<<Tv<<" "<<pitch_angle<<endl;
  cout<<link_b_front<<endl;
  cout<<link_b_rear<<endl;
  */

}
     
void ZmpPlaner::setInit(double &xIni, double &yIni)
{
  zmpInit<<xIni , yIni;
  cp<<xIni, yIni;
}
void ZmpPlaner::setZmpOffsetX(double &cm_offset_x)
{
  offsetZMPx=cm_offset_x;
  offsetZMPr(0)=offsetZMPl(0)=offsetZMPx;
}
//unused
void ZmpPlaner::setInit(vector2 &Ini)
{
  zmpInit=Ini;
}
//////////////
void ZmpPlaner::setw(double &wIn)
{
  w= wIn;
}
void ZmpPlaner::PlanCP( BodyPtr m_robot, FootType FT, Vector3 *p_ref, Matrix3 *R_ref, vector2 swLegRef_p, Matrix3 object_ref_R, std::deque<vector2> &rfzmp, bool usePivot, string *end_link)
{      
  matrix22 swLegRef_R, SwLeg_R, SupLeg_R;      //yow only already okla
  swLegRef_R=RfromMatrix3(object_ref_R);
  calcSwingLegCP(m_robot, FT, p_ref,R_ref, swLegRef_p, object_ref_R, usePivot, end_link);

  vector2 SupLeg_p(MatrixXd::Zero(2,1));
  vector2 offsetZMP_SupLeg,offsetZMP_SwLeg;
  Link *SupLeg;
  Link *SwLeg;

  //for rfzmp
  Vector2 swLeg_cur_p;

  //////////parameter calculate/////////////////////
  if((FT==FSRFsw)||(FT==RFsw)){
    SupLeg=m_robot->link(end_link[LLEG]);
    SwLeg =m_robot->link(end_link[RLEG]);
    //SupLeg_p=pfromVector3(p_ref[LLEG]);
    //matrix22 SupLeg_R(RfromMatrix3(R_ref[LLEG]));
    offsetZMP_SupLeg=offsetZMPl;
    offsetZMP_SwLeg =offsetZMPr;
  }
  else if((FT==FSLFsw)||(FT==LFsw)){
    SupLeg=m_robot->link(end_link[RLEG]);
    SwLeg =m_robot->link(end_link[LLEG]);
    //SupLeg_p=pfromVector3(p_ref[RLEG]);
    //matrix22 SupLeg_R(RfromMatrix3(R_ref[RLEG]));
    offsetZMP_SupLeg=offsetZMPr;
    offsetZMP_SwLeg =offsetZMPl;
  }
  
  SupLeg_p=pfromVector3(SupLeg->p());
  SupLeg_R=RfromMatrix3(SupLeg->R());
  SupLeg_p  +=   SupLeg_R*offsetZMP_SupLeg;
  swLegRef_p+= swLegRef_R*offsetZMP_SwLeg;
  //for rfzmp
  swLeg_cur_p=pfromVector3(SwLeg->p());
  SwLeg_R=RfromMatrix3(SwLeg->R());
  swLeg_cur_p+= SwLeg_R*offsetZMP_SwLeg;


  ////////////plan//////////////
  vector2 zero(MatrixXd::Zero(2,1));
  if((FT==FSRFsw)||(FT==FSLFsw)){

    //for capture point//
    vector2 cp_cur(zmpInit);
    double b=exp(w*Tsup);
    cZMP= (SupLeg_p- b*cp_cur)/(1-b);
    for(int i=1;i<TsupNum+1;i++){
      cp=cZMP+ exp( w*i*dt )*(cp_cur - cZMP);
      cp_deque.push_back(cp);
    }

    //for rzmp//
    Interplation5(cZMP, zero, zero, cZMP, zero, zero,  Tsup, rfzmp);//

  }
  if((FT==RFsw)||(FT==LFsw)){

    //for capture point//
    Interplation5(cp, zero, zero, cp, zero, zero,  Tdbl, cp_deque);//
    vector2 cZMP_pre(cZMP);
    vector2 cp_cur(cp);
    //double b=exp(w*(Tsup));
    double b=exp(w*(Tsup+Tp));
    cZMP= (swLegRef_p- b*cp_cur)/(1-b);

    //int timeLength=(int)((Tsup)/0.005+NEAR0);
    int timeLength=(int)((Tsup+Tp)/0.005+NEAR0);
    for(int i=1;i<timeLength+1;i++){
      cp=cZMP+ exp( w*i*dt )*(cp_cur - cZMP);
      cp_deque.push_back(cp);
    }
    
    //timeLength=(int)((Tp)/0.005+NEAR0);
    //for(int i=0;i<timeLength;i++)
    //  cp_deque.push_back(cp);
   
    
    //for rzmp
    //Interplation5(cZMP_pre, zero, zero, cZMP, zero, zero, Tdbl, rfzmp);
    //Interplation5(SupLeg_p, zero, zero, SupLeg_p, zero, zero, Tsup+Tp, rfzmp);
    ////Interplation5(cZMP, zero, zero, cZMP, zero, zero, Tsup, rfzmp);//no good
  
    //try
    //Interplation5(cZMP_pre, zero, zero, cZMP, zero, zero, 2*Tdbl, rfzmp);
    Interplation5(swLeg_cur_p, zero, zero, SupLeg_p, zero, zero, 2*Tdbl, rfzmp);
    timeLength=(int)((Tdbl)/0.005+NEAR0);
    for(int i=0;i<timeLength;i++)
      rfzmp.pop_front();
    //Interplation5(SupLeg_p, zero, zero, SupLeg_p, zero, zero, Tsup, rfzmp);
    Interplation5(cZMP, zero, zero, cZMP, zero, zero, Tsup, rfzmp);

    Interplation5(SupLeg_p, zero, zero, swLegRef_p, zero, zero, 2*Tp, rfzmp);
    for(int i=0;i<timeLength;i++)
      rfzmp.pop_back();

   }

  //cout<<"cp "<<cp_deque.size()<<endl;
}


void ZmpPlaner::PlanCPstop(BodyPtr m_robot, FootType FT, Vector3 *p_ref, Matrix3 *R_ref, vector2 swLegRef_p, Matrix3 object_ref_R, std::deque<vector2> &rfzmp, string *end_link)
{      
  matrix22 swLegRef_R;      //yow only already okla
  swLegRef_R=RfromMatrix3(object_ref_R);
  //foot no move
  //calcSwingLegCP(FT, p_ref,R_ref, swLegRef_p, object_ref_R );

  vector2 SupLeg_p(MatrixXd::Zero(2,1));
  Link *SupLeg;

  //////////parameter calculate/////////////////////
  if((FT==FSRFsw)||(FT==RFsw)){
    //SupLeg_p=pfromVector3(p_ref[LLEG]);
    //matrix22 SupLeg_R(RfromMatrix3(R_ref[LLEG]));
    SupLeg=m_robot->link(end_link[LLEG]);
    SupLeg_p=pfromVector3(SupLeg->p());
    matrix22 SupLeg_R(RfromMatrix3(SupLeg->R()));
    SupLeg_p+= SupLeg_R*offsetZMPl;
    swLegRef_p+= swLegRef_R*offsetZMPr;
  }
  else if((FT==FSLFsw)||(FT==LFsw)){
    //SupLeg_p=pfromVector3(p_ref[RLEG]);
    //matrix22 SupLeg_R(RfromMatrix3(R_ref[RLEG]));
    SupLeg=m_robot->link(end_link[RLEG]);
    SupLeg_p=pfromVector3(SupLeg->p());
    matrix22 SupLeg_R(RfromMatrix3(SupLeg->R()));
    SupLeg_p+= SupLeg_R*offsetZMPr;
    swLegRef_p+= swLegRef_R*offsetZMPl;
  }

  ////////////plan//////////////
  vector2 zero(MatrixXd::Zero(2,1));
 
  if((FT==RFsw)||(FT==LFsw)){
    //focus on unity
    //for capture point//
    Interplation5(cp, zero, zero, cp, zero, zero,  Tdbl, cp_deque);//
    vector2 cZMP_pre(cZMP);
    vector2 cp_cur(cp);
    double b=exp(w*Tsup);
    vector2 middle_of_foot;
    middle_of_foot=(swLegRef_p+ SupLeg_p)/2;
    cZMP= (middle_of_foot - b*cp_cur)/(1-b);
    for(int i=1;i<TsupNum+1;i++){
      cp=cZMP+ exp( w*i*dt )*(cp_cur - cZMP);
      cp_deque.push_back(cp);
    }

    //for rzmp
    Interplation5(cZMP_pre, zero, zero, cZMP, zero, zero, Tdbl, rfzmp);
    Interplation5(cZMP, zero, zero, middle_of_foot, zero, zero, Tsup, rfzmp);
    

    /*
    //quick version
    //for capture point//
    vector2 cZMP_pre(cZMP);
    vector2 cp_cur(cp);
    double b=exp(w*Tsup);
    vector2 middle_of_foot;
    middle_of_foot=(swLegRef_p+ SupLeg_p)/2;
    cZMP= (middle_of_foot - b*cp_cur)/(1-b);
    Interplation5(cp, zero, zero, middle_of_foot, zero, zero,  Tdbl, cp_deque);//
    Interplation5(middle_of_foot, zero, zero, middle_of_foot, zero, zero,  Tsup, cp_deque);

    //for rzmp
    Interplation5(cZMP, zero, zero, cZMP, zero, zero, Tdbl, rfzmp);
    Interplation5(cZMP, zero, zero, middle_of_foot, zero, zero, Tsup, rfzmp);
    */
  }
}

void ZmpPlaner::getNextCom(Vector3 &cm_ref)
{
  //if(cp_deque.empty())
  //  return;
    
  vector2 cm_cur, cp_cur, cm_vel, cm_out;
  cm_cur<<cm_ref(0), cm_ref(1);

  if(!cp_deque.empty()){
    cp_cur=cp_deque.at(0);
    cp_deque.pop_front();
  }
  else{ 
    cp_cur=cp;
  }
  cm_vel= w*( cp_cur- cm_cur);

  cm_out=cm_cur + cm_vel*dt;
  cm_ref[0]=cm_out[0];
  cm_ref[1]=cm_out[1];

 

  //ofszmp<<cp_cur[0]<<" "<<cp_cur[1]<<" "<<cm_out[0]<<" "<<cm_out[1]<<endl;
}

//////
/*
void ZmpPlaner::PlanZMPnew(FootType FT, Vector3 *p_ref, Matrix3 *R_ref, vector2 swLegRef_p, Matrix3 object_ref_R, std::deque<vector2> &rfzmp)
{      
  matrix22 swLegRef_R;      //yow only already okla
  swLegRef_R=RfromMatrix3(object_ref_R);
  calcSwingLegXYONew(FT, p_ref,R_ref, swLegRef_p, object_ref_R );

  //unuse
  if((FT==FSRFsw)||(FT==FSLFsw))
    beforeUpNum=(int)(0.5*TsupNum+1.5*TdblNum+NEAR0);
  else if((FT==RFsw)||(FT==LFsw))
    beforeUpNum=(int)(0.25*TdblNum+NEAR0);

  vector2 SupLeg_p(MatrixXd::Zero(2,1));
  vector2 PreSupLeg_p(MatrixXd::Zero(2,1));
  vector2 nnextFootPlace(MatrixXd::Zero(2,1));

  if((FT==FSRFsw)||(FT==RFsw)){
    SupLeg_p=pfromVector3(p_ref[LLEG]);
    if(FT==RFsw){
      PreSupLeg_p=pfromVector3(p_ref[RLEG]);
      matrix22 PreSupLeg_R(RfromMatrix3(R_ref[RLEG]));
      PreSupLeg_p+= PreSupLeg_R*offsetZMPr;
    }
    matrix22 SupLeg_R(RfromMatrix3(R_ref[LLEG]));
    nnextFootPlace= swLegRef_p+swLegRef_R*RLEG2LLEG;
    SupLeg_p+= SupLeg_R*offsetZMPl;
    nnextFootPlace+=swLegRef_R*offsetZMPl;
    swLegRef_p+= swLegRef_R*offsetZMPr;
  }
  else if((FT==FSLFsw)||(FT==LFsw)){
    SupLeg_p=pfromVector3(p_ref[RLEG]);
    if(FT==LFsw){
      PreSupLeg_p=pfromVector3(p_ref[LLEG]);
      matrix22 PreSupLeg_R(RfromMatrix3(R_ref[LLEG]));
      PreSupLeg_p+= PreSupLeg_R*offsetZMPl;//debug new
    }
    matrix22 SupLeg_R(RfromMatrix3(R_ref[RLEG]));
    nnextFootPlace = swLegRef_p+swLegRef_R*LLEG2RLEG;
    SupLeg_p+= SupLeg_R*offsetZMPr;
    //presume RLEG_R is same as landing swLeg
    nnextFootPlace+=swLegRef_R*offsetZMPr;
    swLegRef_p+= swLegRef_R*offsetZMPl;
  }

  vector2 zero(MatrixXd::Zero(2,1));
  if((FT==FSRFsw)||(FT==FSLFsw)){
    vector2 Now(zmpInit);
    Interplation5(Now, zero, zero, Now, zero, zero,  0.5*Tsup, rfzmp);//
    Interplation5(Now, zero, zero, SupLeg_p, zero, zero,  1.5*Tdbl, rfzmp);
    Interplation5(SupLeg_p, zero, zero, SupLeg_p, zero, zero, Tsup, rfzmp);
    Interplation5(SupLeg_p, zero, zero, swLegRef_p, zero, zero, 0.5*Tdbl, rfzmp);
    Interplation5(swLegRef_p, zero, zero, swLegRef_p, zero, zero, Tsup, rfzmp);
    Interplation5(swLegRef_p, zero, zero, nnextFootPlace , zero, zero, 0.5*Tdbl, rfzmp);
    Interplation5(nnextFootPlace, zero, zero, nnextFootPlace, zero, zero, Tsup, rfzmp);
    Interplation5(nnextFootPlace, zero, zero,  swLegRef_p, zero, zero, 0.5*Tdbl, rfzmp);
    Interplation5(swLegRef_p, zero, zero, swLegRef_p, zero, zero, Tsup, rfzmp);
  }
  if((FT==RFsw)||(FT==LFsw)){
    Interplation5(PreSupLeg_p, zero, zero, SupLeg_p, zero, zero,  0.5*Tdbl, rfzmp);//
    Interplation5(SupLeg_p, zero, zero, SupLeg_p, zero, zero, Tsup, rfzmp);
    Interplation5(SupLeg_p, zero, zero, swLegRef_p, zero, zero, 0.5*Tdbl, rfzmp);
    Interplation5(swLegRef_p, zero, zero, swLegRef_p, zero, zero, Tsup, rfzmp);
    Interplation5(swLegRef_p, zero, zero, nnextFootPlace , zero, zero, 0.5*Tdbl, rfzmp);
    Interplation5(nnextFootPlace, zero, zero, nnextFootPlace, zero, zero, Tsup, rfzmp);
    Interplation5(nnextFootPlace, zero, zero,  swLegRef_p, zero, zero, 0.5*Tdbl, rfzmp);
    Interplation5(swLegRef_p, zero, zero, swLegRef_p, zero, zero, Tsup, rfzmp);

    //delete span of previous step
    for(int i=0;i<TdblNum/4;i++)
      rfzmp.pop_front();
  }

  //rfzmpOri= std::deque<vector2> (rfzmp);
 
 
  
  //std::deque<vector2>::iterator itr = rfzmp.begin();  
  //for( itr = rfzmp.begin(); itr != rfzmp.end(); itr++ ){
  //  ofszmp<<(*(itr))[0]<<" "<<(*(itr))[1]<<endl;
  // }
  //ofszmp<<'\n'<<std::endl;
  
}
*/
/*
void ZmpPlaner::calcWaistR( FootType FT,  Matrix3 *R_ref)
{

 int swingLeg=swLeg(FT);
  ///
  Matrix3 sup_R;
  if((FT==FSRFsw)||(FT==RFsw)){
    sup_R=extractYow(R_ref[LLEG]);
  }
  else if((FT==FSLFsw)||(FT==LFsw)){
    sup_R=extractYow(R_ref[RLEG]);
  } 
  Matrix3 sw_R(extractYow(R_ref[swingLeg]));

  //Matrix3 Rmid( trans(sup_R) * R_ref[swingLeg]);
  Matrix3 Rmid( sup_R.transpose() * sw_R);//for toe
  Vector3 omega( omegaFromRot(Rmid));
  //R_ref[WAIST]= sup_R*rodrigues(omega, 0.5);
  R_ref[WAIST]= sup_R*rodoriges(omega, 0.5);
}
*/

Matrix3 ZmpPlaner::calcWaistR( FootType FT,  BodyPtr m_robot, string *end_link)
{
  Link* SwLeg;
  Link* SupLeg;


  if((FT==FSRFsw)||(FT==RFsw)){
    SwLeg=m_robot->link(end_link[RLEG]);
    SupLeg=m_robot->link(end_link[LLEG]);
  }
  else if((FT==FSLFsw)||(FT==LFsw)){
    SwLeg=m_robot->link(end_link[LLEG]);
    SupLeg=m_robot->link(end_link[RLEG]);
  } 
  Matrix3 sup_R=extractYow(SupLeg->R());
  Matrix3 sw_R=extractYow(SwLeg->R());

  Matrix3 Rmid( sup_R.transpose() * sw_R);//for toe
  Vector3 omega( omegaFromRot(Rmid));
  //R_ref[WAIST]= sup_R*rodrigues(omega, 0.5);
  Matrix3 R_ref_WAIST= sup_R*rodoriges(omega, 0.5);
  
  return R_ref_WAIST;
}


void ZmpPlaner::atan2adjust(double &pre, double &cur)
{
  if(fabs(pre - cur)>M_PI){
    
    if(pre > cur ){
      while(fabs(pre - cur)>M_PI)
	cur+=2*M_PI;
    }
    else if(pre < cur ){
      while(fabs(pre - cur)>M_PI)
	cur-=2*M_PI;
    }
    
  }// if(fabs(pre - cur)>M_PI)
   
}


void ZmpPlaner::calcSwingLegCP( BodyPtr m_robot, FootType FT, Vector3 *p_ref, Matrix3 *R_ref, vector2 swLegRef_p, Matrix3 object_ref_R, bool usePivot, string *end_link)
{
  vector2 swLegIni_p=MatrixXd::Zero(2,1);

  int it=0;
  double zs=0;
  vector2 zero(MatrixXd::Zero(2,1));
  Vector3 zerov3(MatrixXd::Zero(3,1));
  swLegxy.clear();
  Trajzd.clear();
  swLeg_R.clear();
  index.clear();

  link_b_deque.clear();
  rot_pitch.clear();

  Link* SwLeg;
  //Vector3 rpytemp=rpyFromRot(object_ref_R);
  //Matrix3 tar_R=rotationZ(rpytemp(2));
  Matrix3 tar_R=extractYow(object_ref_R);
  Matrix3 swLeg_cur_R;
  //nannnnnndatooo!!!!
  if((FT==FSRFsw)||(FT==RFsw)){
    SwLeg= m_robot->link(end_link[RLEG]);
    //swLegIni_p=pfromVector3(p_ref[RLEG]);
    //swLeg_cur_R=R_ref[RLEG];
  }
  else if((FT==FSLFsw)||(FT==LFsw)){
    SwLeg= m_robot->link(end_link[LLEG]);
    //swLegIni_p=pfromVector3(p_ref[LLEG]);
    //swLeg_cur_R=R_ref[LLEG];
  }
  swLegIni_p=pfromVector3(SwLeg->p());
  swLeg_cur_R=SwLeg->R();
  
  Matrix3 Rmid( swLeg_cur_R.transpose() * tar_R);
  Vector3 omega( omegaFromRot(Rmid));
  Interplation5(0.0, 0.0, 0.0 , 1.0, 0.0, 0.0, Tsup-2*Tv, index);
 

  ///pivot
  Vector3 link_b_s;
  Vector3 link_b_f;
  double pitch_s;
  double pitch_f;
  matrix22 swLegIni_R=RfromMatrix3(swLeg_cur_R);//33>>22
  vector2 swLegIni_p_nomal=swLegIni_R.transpose()*swLegIni_p;
  vector2 swLegRef_p_nomal=swLegIni_R.transpose()*swLegRef_p;
  //for pivot////////////////////////////////////////////
  if(usePivot){
 
 
    //cout<<swLegRef_p_nomal(0)-swLegIni_p_nomal(0)<<endl;
    
    if((swLegRef_p_nomal(0)-swLegIni_p_nomal(0))>0.1){
      //front;
      Tdbl=Tp=0.1;
      link_b_s= link_b_front;
      link_b_f= link_b_rear;
      pitch_s=pitch_angle;
      pitch_f=-pitch_angle;
      //cout<<"front"<<endl;
    }
    else if((swLegRef_p_nomal(0)-swLegIni_p_nomal(0))<-0.08){
      //back;
      Tdbl=Tp=0.1;
      link_b_s= link_b_rear;
      link_b_f= link_b_front;
      pitch_s=-pitch_angle*0.5;
      pitch_f=pitch_angle;//*0.7;
      //pitch=0.0;
      //cout<<"back"<<endl;
    }
    else{
      Tdbl=Tp=0.05;
      link_b_s<<0.015, 0.0, -0.105;
      link_b_f<<0.015, 0.0, -0.105;
      pitch_s=pitch_f=0.0;
      //cout<<"same"<<endl;
    }
    
    
    swLegIni_p+= pfromVector3(Vector3(swLeg_cur_R*link_b_s));
    swLegRef_p+= pfromVector3(Vector3(tar_R*link_b_f));
  }
  /////////////////////////////////////////////////////
  
  if((FT==FSRFsw)||(FT==FSLFsw)){
    //foot no move during this span
  }
  else if((FT==RFsw)||(FT==LFsw)){
    //x y
    if(usePivot){
      double vel=pitch_s *0.13/(Tp+Tv);  
      vector2 sp;
      sp<<vel*sin(pitch_s)*1.5,0.0;
      double sz=vel*cos(pitch_s)*1.5;

      //test param
      vel=  pitch_s/Tp*0.3*0.23;
      sp<<vel*sin(pitch_s)*5, 0.0;
      sp=swLegIni_R*sp;
      sz=vel*cos(pitch_s)*2.5;

      //interpolation ver
      //Interplation5(swLegIni_p, zero, zero, swLegIni_p, zero, zero, Tdbl+Tv, swLegxy);
      ////Interplation5(swLegIni_p, sp, zero, swLegRef_p, zero, zero, Tsup-2*Tv, swLegxy);
      //Interplation5(swLegIni_p, zero, zero, swLegRef_p, zero, zero, Tsup-2*Tv, swLegxy);
      //Interplation5(swLegRef_p, zero, zero, swLegRef_p, zero, zero, Tv+Tp, swLegxy);

      //optimal solution ver
      Vector4 Cx,Cy,Px,Py;
      Matrix4 E,Einv;
      double tf=Tsup-2*Tv;
      Px<<swLegIni_p(0), swLegRef_p(0), 0.0, 0.0;
      Py<<swLegIni_p(1), swLegRef_p(1), 0.0, 0.0;
   
      E<<1.0, 1.0, 0.0, 1.0,
	exp(tf), exp(-tf), tf, 1.0,
	1.0, -1.0, 1.0, 0.0,
        exp(tf), -exp(-tf), 1.0, 0.0;
      Einv=E.inverse();
      Cx=Einv*Px;Cy=Einv*Py;
      Interplation5(swLegIni_p, zero, zero, swLegIni_p, zero, zero, Tdbl+Tv, swLegxy);
      int timeLength=(int)(tf/0.005+NEAR0);
      for(int i=1;i<timeLength+1;i++){
	Vector2 temp;
	temp(0)=Cx(0)*exp(i*dt)+Cx(1)*exp(-i*dt)+Cx(2)*i*dt+Cx(3);
	temp(1)=Cy(0)*exp(i*dt)+Cy(1)*exp(-i*dt)+Cy(2)*i*dt+Cy(3);
	swLegxy.push_back(temp);
      }
      Interplation5(swLegRef_p, zero, zero, swLegRef_p, zero, zero, Tv+Tp, swLegxy);


      //z interpolation ver
      /*
      zs=0.0;
      Interplation3(zs, 0.0,  zs, 0.0, Tdbl, Trajzd);
      //Interplation3(zs, sz,  Zup, 0.0, 0.5*Tsup, Trajzd);
      Interplation3(zs, 0.0,  Zup, 0.0, 0.5*Tsup, Trajzd);
      Interplation3(Zup, 0.0, zs, 0.0, 0.5*Tsup, Trajzd);
      Interplation3(zs, 0.0,  zs, 0.0, Tp, Trajzd);
      */

      
      //spline.ver
      zs=0.0;
      Interplation3(zs, 0.0,  zs, 0.0, Tdbl, Trajzd);
      std::vector<double> X(5), Y(5);
      X[0]=0.0; X[1]=0.1*Tsup;  X[2]=0.5*Tsup;   X[3]=Tsup-0.011; X[4]= Tsup;      
      Y[0]=0.0; Y[1]=Zup*0.05;  Y[2]=Zup;        Y[3]=Zup*0.005;  Y[4]= 0.0;   
  
      tk::spline s;
      s.set_points(X,Y);  
      for(int i=0;i<Tsup/dt;i++){
	double temz=s((i+1)*dt);
	Trajzd.push_back(temz);
      }
      Interplation3(zs, 0.0,  zs, 0.0, Tp, Trajzd);
      //cout<<"sw "<<swLegxy.size()<<endl;
      
    }
    /////////////////////unused//////////////////////////////
    else{//non piovot mode
      Interplation5(swLegIni_p, zero, zero, swLegIni_p, zero, zero, Tdbl+Tv, swLegxy);//
      Interplation5(swLegIni_p, zero, zero, swLegRef_p, zero, zero, Tsup-2*Tv, swLegxy);//
      Interplation5(swLegRef_p, zero, zero, swLegRef_p, zero, zero, Tv, swLegxy);//

      //z
      zs=0.0;
      Interplation3(zs, 0.0,  zs, 0.0, Tdbl, Trajzd);
      Interplation3(zs, 0.0,  Zup, 0.0, 0.5*Tsup, Trajzd);
      Interplation3(Zup, 0.0, zs, 0.0, 0.5*Tsup, Trajzd);


      //spline.ver
      std::vector<double> X(6), Y(6);
      X[0]=0.0; X[1]=Tp*0.2;       X[2]=Tp;      X[3]=Tp+Tsup;  X[4]= Tp+Tsup+Tp-0.011; X[5]=Tp+Tsup+Tp;      
      Y[0]=0.0; Y[1]=pitch_s*0.05; Y[2]=pitch_s; Y[3]=pitch_f;  Y[4]= pitch_f*0.005;             Y[5]=0.0;

      tk::spline s;
      s.set_points(X,Y);  
      
      for(int i=0;i<0.9/dt;i++){
	Matrix3 pushin(rotationY(s((i+1)*dt)));
	rot_pitch.push_back (pushin);
      }
    
    }


    //yow
    int tem=  (int)((Tdbl+Tv)/dt +NEAR0 );
    for(int i=0;i<tem;i++)
      swLeg_R.push_back(swLeg_cur_R);//Tdbl+tv
    while(!index.empty()){
      Matrix3 pushin(swLeg_cur_R*rodoriges(omega, index.at(0)));
      swLeg_R.push_back (pushin);//Tsup-2tv
      index.pop_front();
    }
    Matrix3 temR(swLeg_cur_R*rodoriges(omega, 1));

    tem=  (int)((Tv+Tp)/dt +NEAR0 );
    for(int i=0;i<tem;i++)
      swLeg_R.push_back(temR);//tv
   


    //////pivot//////////////////
    if(usePivot){
      //link_b
      //Interplation5(link_b_s,zerov3,zerov3,link_b_s, zerov3, zerov3, Tdbl+Tv, link_b_deque);
      //Interplation5(link_b_s,zerov3,zerov3,link_b_f, zerov3, zerov3, Tsup-2*Tv, link_b_deque);
      //Interplation5(link_b_f,zerov3,zerov3,link_b_f, zerov3, zerov3, Tv+Tp, link_b_deque);//
     
      //optimal solution ver
      Interplation5(link_b_s,zerov3,zerov3,link_b_s, zerov3, zerov3, Tdbl+Tv, link_b_deque);
      Vector4 Cx,Cz,Px,Pz;
      Matrix4 E,Einv;
      double tf=Tsup-2*Tv;
      Px<<link_b_s(0), link_b_f(0), 0.0, 0.0;
      Pz<<link_b_s(2), link_b_f(2), 0.0, 0.0;
      E<<1.0, 1.0, 0.0, 1.0,
	exp(tf), exp(-tf), tf, 1.0,
	1.0, -1.0, 1.0, 0.0,
        exp(tf), -exp(-tf), 1.0, 0.0;
      Einv=E.inverse();
      Cx=Einv*Px;Cz=Einv*Pz;
      int timeL=(int)(tf/0.005+NEAR0);
      for(int i=1;i<timeL+1;i++){
	Vector3 temp;
	temp<<Cx(0)*exp(i*dt)+Cx(1)*exp(-i*dt)+Cx(2)*i*dt+Cx(3), 0.0, 
              Cz(0)*exp(i*dt)+Cz(1)*exp(-i*dt)+Cz(2)*i*dt+Cz(3);
	link_b_deque.push_back(temp);
      }
      Interplation5(link_b_f,zerov3,zerov3,link_b_f, zerov3, zerov3, Tv+Tp, link_b_deque);//
    


      //pitch angle
      index.clear();
      /*
      Interplation3(0.0, 0.0, pitch_s,  pitch_s/(Tp)*0.3, Tp, index);
      Interplation3(pitch_s,  pitch_s/(Tp)*0.3, pitch_f,  0.0, Tsup, index);
      Interplation3(pitch_f,  0.0,  0.0, 0.0, Tp, index);
      //Interplation3(0.0, 0.0, pitch_s,  0.0, Tp, index);
      //Interplation3(pitch_s, 0.0, pitch_f,  0.0, Tsup, index);
      //Interplation3(pitch_f,  0.0,  0.0, 0.0, Tp, index);
      while(!index.empty()){
	Matrix3 pushin(rotationY(index.at(0)));
	rot_pitch.push_back (pushin);
	index.pop_front();
      }
      */

      //spline.ver
      std::vector<double> X(5), Y(5);
      X[0]=0.0; X[1]=Tp;      X[2]=Tp+Tsup;  X[3]= Tp+Tsup+Tp-0.011; X[4]=Tp+Tsup+Tp;      
      Y[0]=0.0; Y[1]=pitch_s; Y[2]=pitch_f;  Y[3]= pitch_f*0.005;             Y[4]=0.0;
      
      tk::spline s;
      s.set_points(X,Y);    // currently it is required that X is already sorted
      int timeLength=(int)((Tsup+2*Tp)/dt+NEAR0);
      for(int i=0;i<timeLength;i++){
	Matrix3 pushin(rotationY(s((i+1)*dt)));
	rot_pitch.push_back (pushin);
      }
      
    }
    /////////////////////////
  }
}

void ZmpPlaner::calcSwingLegXYONew(FootType FT, Vector3 *p_ref, Matrix3 *R_ref, vector2 swLegRef_p, Matrix3 object_ref_R)
{
  vector2 swLegIni_p=MatrixXd::Zero(2,1);

  int it=0;
  double zs=0;
  vector2 zero(MatrixXd::Zero(2,1));
  Vector3 zerov3(MatrixXd::Zero(3,1));
  swLegxy.clear();
  Trajzd.clear();
  swLeg_R.clear();
  index.clear();

  //Vector3 rpytemp=rpyFromRot(object_ref_R);
  //Matrix3 tar_R=rotationZ(rpytemp(2));
  Matrix3 tar_R=extractYow(object_ref_R);
  Matrix3 swLeg_cur_R;
  //nannnnnndatooo!!!!
  if((FT==FSRFsw)||(FT==RFsw)){
    swLegIni_p=pfromVector3(p_ref[RLEG]);
    swLeg_cur_R=R_ref[RLEG];
  }
 else if((FT==FSLFsw)||(FT==LFsw)){
    swLegIni_p=pfromVector3(p_ref[LLEG]);
    swLeg_cur_R=R_ref[LLEG];
 }

  Matrix3 Rmid( swLeg_cur_R.transpose() * tar_R);
  Vector3 omega( omegaFromRot(Rmid));
  Interplation5(0.0, 0.0, 0.0 , 1.0, 0.0, 0.0, Tsup-2*Tv, index);
  
  //swLeg_cur_R*rodrigues(omega, index.at(0))
  
  if((FT==FSRFsw)||(FT==FSLFsw)){
    //x y 
    Interplation5(swLegIni_p, zero, zero, swLegIni_p, zero, zero, 0.5*Tsup+1.5*Tdbl+Tv, swLegxy);//
    Interplation5(swLegIni_p, zero, zero, swLegRef_p, zero, zero, Tsup-2*Tv, swLegxy);//
    Interplation5(swLegRef_p, zero, zero, swLegRef_p, zero, zero, Tv+0.25*Tdbl, swLegxy);//
    //yow
    int tem=  (int)((0.5*Tsup+1.5*Tdbl+Tv)/dt +NEAR0 );
    for(int i=0;i<tem;i++)
      swLeg_R.push_back(swLeg_cur_R);
    while(!index.empty()){
      Matrix3 pushin(swLeg_cur_R*rodoriges(omega, index.at(0)));
      swLeg_R.push_back (pushin);
      index.pop_front();
    }
    //cerr<<*(swLeg_R.end())<<endl;
    Matrix3 temR(swLeg_cur_R*rodoriges(omega, 1));
    //cerr<< temR<<endl;
    //cerr<<tar_R<<endl;

    tem=  (int)((Tv+0.25*Tdbl)/dt +NEAR0 );
    for(int i=0;i<tem;i++)
      swLeg_R.push_back(temR);      
    //swLeg_R.push_back(tar_R);//??tar_R!=temR small error
    
    //z
    zs=0.0;
    Interplation3(zs, 0.0,  zs, 0.0, 0.5*Tsup+1.5*Tdbl, Trajzd);
    Interplation3(zs, 0.0,  Zup, 0.0, 0.5*Tsup, Trajzd);
    Interplation3(Zup, 0.0, zs, 0.0, 0.5*Tsup, Trajzd);
    Interplation3(zs, 0.0,  zs, 0.0, 0.25*Tdbl, Trajzd);
    
    
  }
  else if((FT==RFsw)||(FT==LFsw)){
    //x y
    Interplation5(swLegIni_p, zero, zero, swLegIni_p, zero, zero, 0.25*Tdbl+Tv, swLegxy);//
    Interplation5(swLegIni_p, zero, zero, swLegRef_p, zero, zero, Tsup-2*Tv, swLegxy);//
    Interplation5(swLegRef_p, zero, zero, swLegRef_p, zero, zero, Tv+0.25*Tdbl, swLegxy);//
    //yow
    int tem=  (int)((0.25*Tdbl+Tv)/dt +NEAR0 );
    for(int i=0;i<tem;i++)
      swLeg_R.push_back(swLeg_cur_R);
    while(!index.empty()){
      Matrix3 pushin(swLeg_cur_R*rodoriges(omega, index.at(0)));
      swLeg_R.push_back (pushin);
      index.pop_front();
    }
    Matrix3 temR(swLeg_cur_R*rodoriges(omega, 1));

    tem=  (int)((Tv+0.25*Tdbl)/dt +NEAR0 );
    for(int i=0;i<tem;i++)
      swLeg_R.push_back(temR);
    
    //z
    zs=0.0;
    Interplation3(zs, 0.0,  zs, 0.0, 0.25*Tdbl, Trajzd);
    Interplation3(zs, 0.0,  Zup, 0.0, 0.5*Tsup, Trajzd);
    Interplation3(Zup, 0.0, zs, 0.0, 0.5*Tsup, Trajzd);
    Interplation3(zs, 0.0,  zs, 0.0, 0.25*Tdbl, Trajzd);
  }

  /*
  std::deque<vector2>::iterator itr = swLegxy.begin();  
  for( itr = swLegxy.begin(); itr != swLegxy.end(); itr++ ){
    ofszmp<<(*(itr))[0]<<" "<<(*(itr))[1]<<endl;
  }
  ofszmp<<'\n'<<std::endl;
  */
}
/////
/*
void ZmpPlaner::PlanZMPnew_toe(BodyPtr body, FootType FT, Vector3 *p_ref, Matrix3 *R_ref, vector2 swLegRef_p, Matrix3 object_ref_R, std::deque<vector2> &rfzmp)
{      
  matrix22 swLegRef_R;      //yow only already okla
  swLegRef_R=RfromMatrix3(object_ref_R);
  calcSwingLegXYONew_toe(body, FT, p_ref,R_ref, swLegRef_p, object_ref_R );
  /////////////////////////////////////////////////////


  vector2 SupLeg_p(0);
  vector2 PreSupLeg_p(0);
  vector2 nnextFootPlace(0);
  Link* SpLeg= new hrp::Link();
  Link* SwLeg= new hrp::Link();
  if((FT==FSRFsw)||(FT==RFsw)){
    SpLeg=body->link("LLEG_JOINT5");
    SwLeg=body->link("RLEG_JOINT5");
    SupLeg_p=pfromVector3( SpLeg->p);
    if(FT==RFsw){
      PreSupLeg_p=pfromVector3(SwLeg->p);
      matrix22 PreSupLeg_R(RfromMatrix3(SwLeg->R));
      PreSupLeg_p+= PreSupLeg_R*offsetZMPr;
    }
    matrix22 SupLeg_R(RfromMatrix3(SpLeg->R));
    nnextFootPlace= swLegRef_p+swLegRef_R*RLEG2LLEG;
    SupLeg_p+= SupLeg_R*offsetZMPl;
    nnextFootPlace+=swLegRef_R*offsetZMPl;
    swLegRef_p+= swLegRef_R*offsetZMPr;
  }
  else if((FT==FSLFsw)||(FT==LFsw)){
    SpLeg=body->link("RLEG_JOINT5");
    SwLeg=body->link("LLEG_JOINT5");
    SupLeg_p=pfromVector3(SpLeg->p);
    if(FT==LFsw){
      PreSupLeg_p=pfromVector3(SwLeg->p);
      matrix22 PreSupLeg_R(RfromMatrix3(SwLeg->R));
      PreSupLeg_p+= PreSupLeg_R*offsetZMPr;
    }
    matrix22 SupLeg_R(RfromMatrix3(SpLeg->R));
    nnextFootPlace = swLegRef_p+swLegRef_R*LLEG2RLEG;
    SupLeg_p+= SupLeg_R*offsetZMPr;
    //presume RLEG_R is same as landing swLeg
    nnextFootPlace+=swLegRef_R*offsetZMPr;
    swLegRef_p+= swLegRef_R*offsetZMPl;
  }

  vector2 zero(0);
  if((FT==FSRFsw)||(FT==FSLFsw)){
    vector2 Now(zmpInit);
    Interplation5(Now, zero, zero, Now, zero, zero,  0.5*Tsup, rfzmp);//
    Interplation5(Now, zero, zero, SupLeg_p, zero, zero,  1.5*Tdbl, rfzmp);
    Interplation5(SupLeg_p, zero, zero, SupLeg_p, zero, zero, Tsup, rfzmp);
    Interplation5(SupLeg_p, zero, zero, swLegRef_p, zero, zero, 0.5*Tdbl, rfzmp);
    Interplation5(swLegRef_p, zero, zero, swLegRef_p, zero, zero, Tsup, rfzmp);
    Interplation5(swLegRef_p, zero, zero, nnextFootPlace , zero, zero, 0.5*Tdbl, rfzmp);
    Interplation5(nnextFootPlace, zero, zero, nnextFootPlace, zero, zero, Tsup, rfzmp);
    Interplation5(nnextFootPlace, zero, zero,  swLegRef_p, zero, zero, 0.5*Tdbl, rfzmp);
    Interplation5(swLegRef_p, zero, zero, swLegRef_p, zero, zero, Tsup, rfzmp);
  }
  if((FT==RFsw)||(FT==LFsw)){
    Interplation5(PreSupLeg_p, zero, zero, SupLeg_p, zero, zero,  0.5*Tdbl, rfzmp);//
    Interplation5(SupLeg_p, zero, zero, SupLeg_p, zero, zero, Tsup, rfzmp);
    Interplation5(SupLeg_p, zero, zero, swLegRef_p, zero, zero, 0.5*Tdbl, rfzmp);
    Interplation5(swLegRef_p, zero, zero, swLegRef_p, zero, zero, Tsup, rfzmp);
    Interplation5(swLegRef_p, zero, zero, nnextFootPlace , zero, zero, 0.5*Tdbl, rfzmp);
    Interplation5(nnextFootPlace, zero, zero, nnextFootPlace, zero, zero, Tsup, rfzmp);
    Interplation5(nnextFootPlace, zero, zero,  swLegRef_p, zero, zero, 0.5*Tdbl, rfzmp);
    Interplation5(swLegRef_p, zero, zero, swLegRef_p, zero, zero, Tsup, rfzmp);

    //delete span of previous step
    for(int i=0;i<TdblNum/4;i++)
      rfzmp.pop_front();
  }

  //rfzmpOri= std::deque<vector2> (rfzmp);
 
}

/////
void ZmpPlaner::calcSwingLegXYONew_toe(BodyPtr body, FootType FT, Vector3 *p_ref, Matrix3 *R_ref, vector2 swLegRef_p, Matrix3 object_ref_R)
{
  vector2 swLegIni_p(0);

  double zs=0;
  vector2 zero(0);
  Vector3 zerov3(0);
  swLegxy.clear();
  Trajzd.clear();
  swLeg_R.clear();
  index.clear();

  link_b_deque.clear();
  rot_pitch.clear();

  Matrix3 tar_R=extractYow(object_ref_R);
  Matrix3 swLeg_cur_R; 
  //nannnnnndatooo!!!!
  if((FT==FSRFsw)||(FT==RFsw)){
    //swLegIni_p=pfromVector3(p_ref[RLEG]);
    swLegIni_p=pfromVector3(body->link("RLEG_JOINT5")->p);
    swLeg_cur_R=R_ref[RLEG];
  }
 else if((FT==FSLFsw)||(FT==LFsw)){
   //swLegIni_p=pfromVector3(p_ref[LLEG]);
    swLegIni_p=pfromVector3(body->link("LLEG_JOINT5")->p);
    swLeg_cur_R=R_ref[LLEG];
 }

  //for pitch///
  matrix22 swLegIni_R;  
  swLegIni_R=RfromMatrix3(swLeg_cur_R);//33>>22
  vector2 swLegIni_p_nomal(trans(swLegIni_R)*swLegIni_p);
  vector2 swLegRef_p_nomal(trans(swLegIni_R)*swLegRef_p);
  Vector3 link_b_s(0);
  Vector3 link_b_f(0);
  double pitch;

  if((swLegRef_p_nomal(0)-swLegIni_p_nomal(0))>=0){
    //front;
    link_b_s= link_b_front;
    link_b_f= link_b_rear;
    pitch=pitch_angle;
  }
  else{
    //back;
    link_b_s= link_b_rear;
    link_b_f= link_b_front;
    pitch=-pitch_angle;
  }

  swLegIni_p+= pfromVector3(Vector3(swLeg_cur_R*link_b_s));
  swLegRef_p+= pfromVector3(Vector3(tar_R*link_b_f));
  ////

  //for yow
  Matrix3 Rmid( trans(swLeg_cur_R) * tar_R);
  Vector3 omega( omegaFromRot(Rmid));
  Interplation5(0.0, 0.0, 0.0 , 1.0, 0.0, 0.0, Tsup-2*Tv, index);
  
  //swLeg_cur_R*rodrigues(omega, index.at(0))
  double vel=pitch *0.13/(Tp+Tv);  
  vector2 sp;
  sp=vel*sin(pitch_angle)*1.5, 0.0;
  double sz=vel*cos(pitch_angle)*1.5;

  if((FT==FSRFsw)||(FT==FSLFsw)){
    //x y of pivot
    Interplation5(swLegIni_p, zero, zero, swLegIni_p, zero, zero, 0.5*Tsup+1.5*Tdbl+Tv, swLegxy);//
    //Interplation5(swLegIni_p, zero, zero, swLegRef_p, zero, zero, Tsup-2*Tv, swLegxy);//
    //Interplation5(swLegIni_p, sp, zero, swLegRef_p, zero, zero, Tsup-2*Tv, swLegxy);//
    Interplation3(swLegIni_p, sp, swLegRef_p, zero, Tsup-2*Tv, swLegxy);//
    Interplation5(swLegRef_p, zero, zero, swLegRef_p, zero, zero, Tv+0.25*Tdbl, swLegxy);//
    //pivot b
    Interplation5(link_b_s, zerov3, zerov3, link_b_s, zerov3, zerov3, 0.5*Tsup+1.5*Tdbl+Tv, link_b_deque);//
    Interplation5(link_b_s, zerov3, zerov3, link_b_f, zerov3, zerov3, Tsup-2*Tv, link_b_deque);//
    Interplation5(link_b_f, zerov3, zerov3, link_b_f, zerov3, zerov3, Tv+0.25*Tdbl, link_b_deque);//

    //yow
    int tem=  (int)((0.5*Tsup+1.5*Tdbl+Tv)/dt +NEAR0 );
    for(int i=0;i<tem;i++)
      swLeg_R.push_back(swLeg_cur_R);
    while(!index.empty()){
      Matrix3 pushin(swLeg_cur_R*rodrigues(omega, index.at(0)));
      swLeg_R.push_back (pushin);
      index.pop_front();
    }
    //cerr<<*(swLeg_R.end())<<endl;
    Matrix3 temR(swLeg_cur_R*rodrigues(omega, 1));
    //cerr<< temR<<endl;
    //cerr<<tar_R<<endl;

    tem=  (int)((Tv+0.25*Tdbl)/dt +NEAR0 );
    for(int i=0;i<tem;i++)
      swLeg_R.push_back(temR);      
    //swLeg_R.push_back(tar_R);//??tar_R!=temR small error
    
    //pitch
    index.clear();
    tem= (int)((0.5*Tsup+1.5*Tdbl-Tp)/dt +NEAR0 );
    for(int i=0;i<tem;i++){
      rot_pitch.push_back(Matrix3(tvmet::identity<matrix33>()));
    }  
    
    //Interplation5(0.0, 0.0, 0.0 , pitch, 0.0, 0.0, Tp+Tv, index);//TC    
    //Interplation5(pitch, 0.0, 0.0 , -pitch, 0.0, 0.0, Tsup-2*Tv, index);//SW
    //Interplation5(-pitch, 0.0, 0.0 , 0.0, 0.0, 0.0, Tv+Tp, index);//HC
    
    Interplation3(0.0, 0.0, pitch, 0.0, Tp+Tv, index);//TC     
    Interplation3(pitch, 0.0, -pitch, 0.0, Tsup-2*Tv, index);//SW
    Interplation3(-pitch,  pitch/(Tp+Tv),  0.0, 0.0, Tv+Tp, index);//HC

    while(!index.empty()){
      Matrix3 pushin(rotationY(index.at(0)));
      rot_pitch.push_back (pushin);
      index.pop_front();
    }

    
    //z
    zs=0.0;
    Interplation3(zs, 0.0,  zs, 0.0, 0.5*Tsup+1.5*Tdbl+Tv, Trajzd);
    //Interplation3(zs, 0.0,  Zup, 0.0, 0.5*Tsup-Tv, Trajzd);
    Interplation3(zs, sz,  Zup, 0.0, 0.5*Tsup-Tv, Trajzd);
    Interplation3(Zup, 0.0, zs, 0.0, 0.5*Tsup-Tv, Trajzd);
    Interplation3(zs, 0.0,  zs, 0.0, 0.25*Tdbl+Tv, Trajzd);
    
  }
  else if((FT==RFsw)||(FT==LFsw)){
    //x y
    Interplation5(swLegIni_p, zero, zero, swLegIni_p, zero, zero, 0.25*Tdbl+Tv, swLegxy);//
    //Interplation5(swLegIni_p, zero, zero, swLegRef_p, zero, zero, Tsup-2*Tv, swLegxy);//
    //Interplation5(swLegIni_p, sp, zero, swLegRef_p, zero, zero, Tsup-2*Tv, swLegxy);//
    Interplation3(swLegIni_p, sp, swLegRef_p, zero, Tsup-2*Tv, swLegxy);//
    Interplation5(swLegRef_p, zero, zero, swLegRef_p, zero, zero, Tv+0.25*Tdbl, swLegxy);//
    //pivot b
    Interplation5(link_b_s, zerov3, zerov3, link_b_s, zerov3, zerov3, 0.25*Tdbl+Tv, link_b_deque);//
    Interplation5(link_b_s, zerov3, zerov3, link_b_f, zerov3, zerov3, Tsup-2*Tv, link_b_deque);//
    Interplation5(link_b_f, zerov3, zerov3, link_b_f, zerov3, zerov3, Tv+0.25*Tdbl, link_b_deque);//

    //yow
    int tem=  (int)((0.25*Tdbl+Tv)/dt +NEAR0 );
    for(int i=0;i<tem;i++)
      swLeg_R.push_back(swLeg_cur_R);
    while(!index.empty()){
      Matrix3 pushin(swLeg_cur_R*rodrigues(omega, index.at(0)));
      swLeg_R.push_back (pushin);
      index.pop_front();
    }
    Matrix3 temR(swLeg_cur_R*rodrigues(omega, 1));

    tem=  (int)((Tv+0.25*Tdbl)/dt +NEAR0 );
    for(int i=0;i<tem;i++)
      swLeg_R.push_back(temR);
    

    //pitch
    index.clear();
    
    //Interplation5(0.0, 0.0, 0.0 , pitch, 0.0, 0.0, Tp+Tv, index);//TC
    //Interplation5(pitch, 0.0, 0.0 , -pitch, 0.0, 0.0, Tsup-2*Tv, index);//SW
    //Interplation5(-pitch, 0.0, 0.0 , 0.0, 0.0, 0.0, Tv+Tp, index);//HC
    
    Interplation3(0.0, 0.0, pitch, 0.0, Tp+Tv, index);//TC
    Interplation3(pitch, 0.0, -pitch, 0.0, Tsup-2*Tv, index);//SW
    Interplation3(-pitch, pitch/(Tp+Tv),  0.0, 0.0, Tv+Tp, index);//HC

    while(!index.empty()){
      Matrix3 pushin(rotationY(index.at(0)));
      rot_pitch.push_back (pushin);
      index.pop_front();
    }
    
    //z
    
    zs=0.0;
    Interplation3(zs, 0.0,  zs, 0.0, 0.25*Tdbl+Tv, Trajzd);
    //Interplation3(zs, 0.0,  Zup, 0.0, 0.5*Tsup-Tv, Trajzd);
    Interplation3(zs, sz,  Zup, 0.0, 0.5*Tsup-Tv, Trajzd);
    Interplation3(Zup, 0.0, zs, 0.0, 0.5*Tsup-Tv, Trajzd);
    Interplation3(zs, 0.0,  zs, 0.0, 0.25*Tdbl+Tv, Trajzd);
  
}

  
  //std::deque<vector2>::iterator itr = swLegxy.begin();  
  //for( itr = swLegxy.begin(); itr != swLegxy.end(); itr++ ){
  //  ofszmp<<(*(itr))[0]<<" "<<(*(itr))[1]<<endl;
  //}
  //ofszmp<<'\n'<<std::endl;
  
}
*/
/////
void ZmpPlaner::StopZMP(FootType FT,  std::deque<vector2> &rfzmp, int count)
{
  double  reInpoTime= 1.25*Tdbl + 3*Tsup;
  int reInpoNum= (int)(reInpoTime/dt+NEAR0);
  vector2 vector2tem; 

  int stopIndex=0;

  if((FT==FSRFsw)||(FT==FSLFsw))//same as length
    stopIndex=(int)((1.5*Tsup+1.75*Tdbl)/0.005+NEAR0);
  else if((FT==RFsw)||(FT==LFsw))
    stopIndex=(int)((Tsup+0.5*Tdbl)/0.005+NEAR0);
  
  std::deque<vector2>::iterator itr = rfzmp.begin();  
  itr= itr+stopIndex-count-1;
  vector2tem = *itr;
  if(stopOper){
    rfzmp.erase(itr+1,rfzmp.end());    
   
    for(int i=0;i<reInpoNum;i++)
	rfzmp.push_back(vector2tem);
    stopOper=0;
    //log
    //for(itr=rfzmp.begin();itr<rfzmp.end();itr++)
    //  	ofszmp<< (*itr)[0]<<" "<<(*itr)[1] <<endl; 
	
    }
    
}

/*
////////NG ersion////////
void ZmpPlaner::PlanZMPnew_toe_dynamic(BodyPtr body, FootType FT, Vector3 *p_ref, Matrix3 *R_ref, vector2 swLegRef_p, Matrix3 object_ref_R, std::deque<vector2> &rfzmp)
{      
  matrix22 swLegRef_R;      //yow only already okla
  swLegRef_R=RfromMatrix3(object_ref_R);
  calcSwingLegXYONew_toe(body, FT, p_ref,R_ref, swLegRef_p, object_ref_R );
  /////////////////////////////////////////////////////
  Matrix3 tar_R=extractYow(object_ref_R);
  Matrix3 swLeg_cur_R, spLeg_cur_R; 
  vector2 swLegIni_p, spLegTip_s_p, swLegTip_s_p;
  if((FT==FSRFsw)||(FT==RFsw)){
    swLegIni_p=pfromVector3(body->link("RLEG_JOINT5")->p);
    spLegTip_s_p=pfromVector3(body->link("LLEG_JOINT5")->p);
    swLeg_cur_R=R_ref[RLEG];
    spLeg_cur_R=R_ref[LLEG]; 
  }
 else if((FT==FSLFsw)||(FT==LFsw)){
   swLegIni_p=pfromVector3(body->link("LLEG_JOINT5")->p);
   spLegTip_s_p=pfromVector3(body->link("RLEG_JOINT5")->p);
   swLeg_cur_R=R_ref[LLEG];
   spLeg_cur_R=R_ref[RLEG];
 }

  //for pitch///
  matrix22 swLegIni_R;  
  swLegIni_R=RfromMatrix3(swLeg_cur_R);//33>>22
  vector2 swLegIni_p_nomal(trans(swLegIni_R)*swLegIni_p);
  vector2 swLegRef_p_nomal(trans(swLegIni_R)*swLegRef_p);
  Vector3 link_b_s(0);
  Vector3 link_b_f(0);
  double pitch;
  vector2 rzmpStart;
  if((swLegRef_p_nomal(0)-swLegIni_p_nomal(0))>=0){
    //front;
    link_b_s= link_b_front;
    link_b_f= link_b_rear;
    pitch=pitch_angle;
  }
  else{
    //back;
    link_b_s= link_b_rear;
    link_b_f= link_b_front;
    pitch=-pitch_angle;
  }
  //start and f
  //swLegIni_p+= pfromVector3(Vector3(swLeg_cur_R*link_b_s));//unused here
  spLegTip_s_p+= pfromVector3(Vector3(spLeg_cur_R*link_b_s));
  swLegTip_s_p=swLegRef_p+ pfromVector3(Vector3(tar_R*link_b_f));

 if((FT==RFsw)||(FT==LFsw)){
   rzmpStart=rzmpEnd;
 }
 rzmpEnd=swLegTip_s_p;

  //////////
  vector2 SupLeg_p(0);
  vector2 PreSupLeg_p(0);
  vector2 nnextFootPlace(0);
  Link* SpLeg= new hrp::Link();
  Link* SwLeg= new hrp::Link();
  if((FT==FSRFsw)||(FT==RFsw)){
    SpLeg=body->link("LLEG_JOINT5");
    SwLeg=body->link("RLEG_JOINT5");
    SupLeg_p=pfromVector3( SpLeg->p);
    if(FT==RFsw){
      PreSupLeg_p=pfromVector3(SwLeg->p);
      matrix22 PreSupLeg_R(RfromMatrix3(SwLeg->R));
      PreSupLeg_p+= PreSupLeg_R*offsetZMPr;
    }
    matrix22 SupLeg_R(RfromMatrix3(SpLeg->R));
    nnextFootPlace= swLegRef_p+swLegRef_R*RLEG2LLEG;
    SupLeg_p+= SupLeg_R*offsetZMPl;
    nnextFootPlace+=swLegRef_R*offsetZMPl;
    swLegRef_p+= swLegRef_R*offsetZMPr;
  }
  else if((FT==FSLFsw)||(FT==LFsw)){
    SpLeg=body->link("RLEG_JOINT5");
    SwLeg=body->link("LLEG_JOINT5");
    SupLeg_p=pfromVector3(SpLeg->p);
    if(FT==LFsw){
      PreSupLeg_p=pfromVector3(SwLeg->p);
      matrix22 PreSupLeg_R(RfromMatrix3(SwLeg->R));
      PreSupLeg_p+= PreSupLeg_R*offsetZMPr;
    }
    matrix22 SupLeg_R(RfromMatrix3(SpLeg->R));
    nnextFootPlace = swLegRef_p+swLegRef_R*LLEG2RLEG;
    SupLeg_p+= SupLeg_R*offsetZMPr;
    //presume RLEG_R is same as landing swLeg
    nnextFootPlace+=swLegRef_R*offsetZMPr;
    swLegRef_p+= swLegRef_R*offsetZMPl;
  }

  vector2 zero(0);
  if((FT==FSRFsw)||(FT==FSLFsw)){
    vector2 Now(zmpInit);
    Interplation5(Now, zero, zero, Now, zero, zero,  0.5*Tsup, rfzmp);//
    Interplation5(Now, zero, zero, SupLeg_p, zero, zero,  1.5*Tdbl, rfzmp);
    Interplation5(SupLeg_p, zero, zero, SupLeg_p, zero, zero, Tsup, rfzmp); 
    Interplation5(SupLeg_p, zero, zero, swLegTip_s_p, zero, zero, 0.25*Tdbl, rfzmp);
    Interplation5(spLegTip_s_p, zero, zero, swLegRef_p, zero, zero, 0.25*Tdbl, rfzmp);
    //ika cyotto tekidou
    Interplation5(swLegRef_p, zero, zero, swLegRef_p, zero, zero, Tsup, rfzmp);
    Interplation5(swLegRef_p, zero, zero, nnextFootPlace , zero, zero, 0.5*Tdbl, rfzmp);
    Interplation5(nnextFootPlace, zero, zero, nnextFootPlace, zero, zero, Tsup, rfzmp);
    Interplation5(nnextFootPlace, zero, zero,  swLegRef_p, zero, zero, 0.5*Tdbl, rfzmp);
    Interplation5(swLegRef_p, zero, zero, swLegRef_p, zero, zero, Tsup, rfzmp);
  }
  if((FT==RFsw)||(FT==LFsw)){
    Interplation5(rzmpStart, zero, zero, SupLeg_p, zero, zero,  0.25*Tdbl, rfzmp);//
    Interplation5(SupLeg_p, zero, zero, SupLeg_p, zero, zero, Tsup, rfzmp);
    Interplation5(SupLeg_p, zero, zero, swLegTip_s_p, zero, zero, 0.25*Tdbl, rfzmp);
    Interplation5(spLegTip_s_p, zero, zero, swLegRef_p, zero, zero, 0.25*Tdbl, rfzmp);
    //ika cyotto tekidou
    Interplation5(swLegRef_p, zero, zero, swLegRef_p, zero, zero, Tsup, rfzmp);
    Interplation5(swLegRef_p, zero, zero, nnextFootPlace , zero, zero, 0.5*Tdbl, rfzmp);
    Interplation5(nnextFootPlace, zero, zero, nnextFootPlace, zero, zero, Tsup, rfzmp);
    Interplation5(nnextFootPlace, zero, zero,  swLegRef_p, zero, zero, 0.5*Tdbl, rfzmp);
    Interplation5(swLegRef_p, zero, zero, swLegRef_p, zero, zero, Tsup, rfzmp);

    
    //delete span of previous step for normal mode
    //for(int i=0;i<TdblNum/4;i++)
    //  rfzmp.pop_front();
    
  }
  //rfzmpOri= std::deque<vector2> (rfzmp);
 
}
*/
