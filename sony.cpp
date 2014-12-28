// -*- C++ -*-
/*!
 * @file  sony.cpp * @brief sonyComponent * $Date$ 
 *
 * $Id$ 
 */
#include "sony.h"
//std::ofstream ofs("/home/wu/src/HRP3.1x/sony_cnoid/sony.log");
// Module specification
// <rtc-template block="module_spec">
static const char* sony_spec[] =
  {
    "implementation_id", "sony",
    "type_name",         "sony",
    "description",       "sonyComponent",
    "version",           "1.0",
    "vendor",            "tohoku",
    "category",          "example",
    "activity_type",     "SPORADIC",
    "kind",              "sonyComponent",
    "max_instance",      "1",
    "language",          "C++",
    "lang_type",         "compile",
    // Configuration variables
    ""
  };
// </rtc-template>

sony::sony(RTC::Manager* manager):
  hrp2Base(manager),
  m_axesIn("axes", m_axes),
  m_buttonsIn("buttons", m_buttons),
  m_sonyServicePort("sonyService")

    // </rtc-template>
{
  m_service0.setComponent(this);
}

sony::~sony()
{
}


RTC::ReturnCode_t sony::onInitialize()
{
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  addInPort("axes", m_axesIn);
  addInPort("buttons", m_buttonsIn);
  //for ps3 controll
  m_axes.data.length(8);
  m_buttons.data.length(11);
  // Set OutPort buffer

  // Set service provider to Ports
  m_sonyServicePort.registerProvider("service0", "sonyService", m_service0);

  // Set service consumers to Ports

  // Set CORBA Service Ports
  addPort(m_sonyServicePort);

  //ini base
  hrp2Base::onInitialize();
  coil::stringTo(m_nStep, prop["wutest.nStep"].c_str());

  cerr<<"start sony nstep "<<m_nStep<<endl;
  //pini
  playflag=0;
  stopflag=1;
  count=0;
  step_counter=0;
  absZMP<<0.0, 0.0, 0.0;
  relZMP<<0.0, 0.0, 0.0;
  step=0;

  flagcalczmp=0;
  FT =FSRFsw;
  CommandIn=5;
  time2Neutral=0.5;

  //test paraini
  velobj=Eigen::MatrixXd::Zero(6,1);
  yawTotal=0;
  object_ref= new Link();
  pt= new Link();
  pt_L= new Link();
  pt_R= new Link();

  //base
  m_basePos.data.x=m_robot->rootLink()->p()(0);
  m_basePos.data.y=m_robot->rootLink()->p()(1);
  m_basePos.data.z=m_robot->rootLink()->p()(2);
  m_baseRpy.data.r=0.0;
  m_baseRpy.data.p=0.0;
  m_baseRpy.data.y=0.0;
  
  //Link* TLink=forceSensors[0]->link();
  //Link* TLink=m_robot->link("LLEG_JOINT5");
  return RTC::RTC_OK;
}


RTC::ReturnCode_t sony::onExecute(RTC::UniqueId ec_id)
{
  //if(!m_rhsensorIn.isNew())
  //  return RTC::RTC_OK;

  //sychronize with simulator
  step_counter+=1;
  step_counter=step_counter%m_nStep;
  if(step_counter!=0)
    return RTC::RTC_OK;

  //read inport
  hrp2Base::updates();

  //gamepad
  if(m_axesIn.isNew()){
    m_axesIn.read();

    velobj(0)=m_axes.data[1]*-5;
    velobj(1)=m_axes.data[0]*-5;
    velobj(5)=m_axes.data[2]*-3;
    /*
    double velsqr=pow(velobj(0),2) + pow(velobj(1),2);
    if( velsqr > 64){
      velobj(0)= velobj(0)* 8/ sqrt(velsqr);
      velobj(1)= velobj(1)* 8/ sqrt(velsqr);    
      }
    */
  }
  
  if(m_buttonsIn.isNew()){
    m_buttonsIn.read();
    //cout<<"buttom is new"<<endl;
  
    if(m_buttons.data[1]==1)//o button
      step=1;
    else if(m_buttons.data[0]==1)//x burron
      step=0;
  }

   //_/_/_/_/_/_/_/_/_/_/_/_/main algorithm_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/
  if(playflag){
    object_operate();   
    prmGenerator( flagcalczmp);//stopflag off here

    walkingMotion(m_robot, FT, cm_ref, absZMP, p_Init, p_ref, R_ref, rfzmp, PC, zmpP, count);

    /*
    if(stopflag){//waiting
      waitingMotion(cm_ref, PC);
    }
    else{//walking
      walkingMotion(m_robot, FT, cm_ref, absZMP, p_Init, p_ref, R_ref, rfzmp, PC, zmpP, count);
      //toe mode
      //walkingMotion(m_robot, FT, cm_ref, absZMP, p_Init, p_ref_toe, R_ref_toe, rfzmp, PC, zmpP, count);
    }    
    */

    calcWholeIVK(); //write in refq
    zmpHandler();

    //base
    m_basePos.data.x=m_robot->rootLink()->p()(0);
    m_basePos.data.y=m_robot->rootLink()->p()(1);
    m_basePos.data.z=m_robot->rootLink()->p()(2);
    Vector3 rpy=R_ref[WAIST].eulerAngles(2, 1, 0);
    //m_baseRpy.data.r=rpy(2);
    //m_baseRpy.data.p=rpy(1);
    m_baseRpy.data.r=0.0;
    m_baseRpy.data.p=0.0;
    m_baseRpy.data.y=rpy(0);
    

    //for next step
    if(ChangeSupLeg(m_robot, FT, count, zmpP, PC, stopflag, CommandIn, p_ref, p_Init, R_ref, R_Init))
      flagcalczmp=1;

    
    //////////////write///////////////
    rzmp2st();
    m_contactStatesOut.write();
    m_basePosOut.write();
    m_baseRpyOut.write();
  }//playflag

  //_/_/_/_/_/_/_/_/_test/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/  
  if(!bodyDeque.empty()){
    for(int i=0;i<m_robot->numJoints();i++)
      m_refq.data[i]=bodyDeque.at(0)(i);
    bodyDeque.pop_front();
    m_refqOut.write();
  }
    
 
  //ofs<<m_rfsensor.data[2]<<" "<<m_lfsensor.data[2]<<endl;
   
 
  return RTC::RTC_OK;
}


//_/_/_/_/_/_/_/_/_function/_//_/_/_/_/_/_/_/_/_/_/_/_/_/_/_  //_/_/_/_/_/_/_/_/_/_/_/_/_/_/_ 
inline void sony::rzmp2st()
{
  relZMP = R_ref[WAIST].transpose()*(absZMP - m_robot->link("WAIST")->p());
  //for(int i=0;i< m_rzmp.data.length();i++)
  //  m_rzmp.data[i]=relZMP[i];    
  m_rzmp.data.x=relZMP[0];
  m_rzmp.data.y=relZMP[1];     
  m_rzmp.data.z=relZMP[2];
  m_rzmpOut.write();
}

inline void sony::calcWholeIVK()
{
  if(CalcIVK_biped(m_robot, cm_ref, p_ref, R_ref, FT, p_Init, R_Init)){
  //if(CalcIVK_biped_toe(m_robot, cm_ref, p_ref_toe, R_ref_toe, FT, p_Init, R_Init)){//toe mode
    getInvResult();
    /*
    //for toe mode
    p_ref[RLEG]=m_robot->link("RLEG_JOINT5")->p();
    R_ref[RLEG]=m_robot->link("RLEG_JOINT5")->R();
    p_ref[LLEG]=m_robot->link("LLEG_JOINT5")->p();
    R_ref[LLEG]=m_robot->link("LLEG_JOINT5")->R();
    */
  }
  else{
    cerr<<"ivk err"<<endl;
  }
}

inline void sony::zmpHandler()
{
  //waiting
  if(stopflag){
    NaturalZmp(m_robot, absZMP);
  }
  //walking
  else{
    ///rzmp To st
    absZMP[0]=rfzmp.at(0)(0);
    absZMP[1]=rfzmp.at(0)(1);
    rfzmp.pop_front();
  }
}

inline void sony::getInvResult()
{
 getModelPosture(m_robot, m_refq);
 m_refqOut.write();
}

inline void sony::object_operate()
{
  //by operator
  Vector3 tep;
  //translation
  tep<<velobj(0)*0.00005,velobj(1)*0.00005, velobj(2)*0.00005;

  //ref////////
  yawTotal+=0.01*velobj(5)*M_PI/180;
  Matrix3 rZ(rotationZ( 0.01*velobj(5)*M_PI/180));
  rotRTemp = rZ*rotRTemp ;
  
  object_ref->R() = rotRTemp;
  object_ref->p() = object_ref->p() + rotationZ(yawTotal)*tep; 
}

inline void sony::calcRefLeg()
{
  Matrix3 Rtem_Q=extractYow(object_ref->R());

  //actually in x-y plan only
  RLEG_ref_p = object_ref->p() + Rtem_Q * p_obj2RLEG; 
  LLEG_ref_p = object_ref->p() + Rtem_Q * p_obj2LLEG;
  LEG_ref_R= Rtem_Q * R_LEG_ini;
}

inline void sony::prmGenerator(bool &calczmpflag)//this is calcrzmp flag
{
  calcRefLeg();
  //////////////usually obmit when keep walking//////////////////
  //start to walk or not A
  //waiting
  if( stopflag && PC->CoM2Stop.empty() ){
    if(walkJudge(m_robot, p_ref, R_ref, FT, RLEG_ref_p, LLEG_ref_p, LEG_ref_R) || step){
      CommandIn=0;//start to walk
      start2walk(m_robot, zmpP, PC, stopflag, cm_ref);//stopflag off
      cerr<<"start2walk "<<endl;
      //calc trajectory 
      prm2Planzmp(FT, p_ref, R_ref, RLEG_ref_p, LLEG_ref_p, LEG_ref_R, rfzmp, zmpP);
      calczmpflag=0;//1008 revise
    }
  }

  
  /*
  //walking&&if stop buckup
  else if(count==(stepLength(FT,zmpP)- 2*zmpP->TdblNum)){
    if( (!walkJudge(m_robot, p_ref, R_ref, FT, RLEG_ref_p, LLEG_ref_p, LEG_ref_R))&& !step){
      CommandIn=5;//stop to walk>>quick stop
      zmpP->StopZMP(FT, rfzmp, count);
    }
  }
  */
  else if(calczmpflag==1){//keep walking start from new leg
    
    if( (!walkJudge(m_robot, p_ref, R_ref, FT, RLEG_ref_p, LLEG_ref_p, LEG_ref_R))&& zmpP->cp_deque.empty() && !step){
      //stopflag=1;
      CommandIn=5;
      cout<<"should stop here"<<endl;
    }
    
    prm2Planzmp(FT, p_ref, R_ref, RLEG_ref_p, LLEG_ref_p, LEG_ref_R, rfzmp, zmpP);
    calczmpflag=0;


  }
}

int sony::stepLength(FootType FT, ZmpPlaner *zmpP)
{
  int stepLength;

 if((FT==FSRFsw)||(FT==FSLFsw))
   stepLength=zmpP->step1Num;
  else if((FT==RFsw)||(FT==LFsw))
   stepLength=zmpP->NomalPaceNum;

 return stepLength;
}

void sony::waitingMotion(Vector3 &cm_ref, PreviewControl *PC)
{
  //CoM to nutral
  if(!(PC->CoM2Stop.empty())){
    cm_ref(0)=PC->CoM2Stop.at(0)(0);
    cm_ref(1)=PC->CoM2Stop.at(0)(1);
    PC->CoM2Stop.pop_front();
  }
}

void sony::start2walk(BodyPtr m_robot, ZmpPlaner *zmpP, PreviewControl *PC, bool &stopflag, Vector3 cm_ref)
{// this is for FSRF or FSLF
  Vector3 rzmpInit;
  NaturalZmp(m_robot, rzmpInit);
  zmpP->setInit( rzmpInit(0) , rzmpInit(1) );
  
  PC->setInitial(cm_ref); 
  PC->Inituk();

  stopflag=0;//comment out when test mode
}

void sony::prm2Planzmp(FootType FT, Vector3 *p_ref, Matrix3 *R_ref, Vector3 RLEG_ref_p, Vector3 LLEG_ref_p, Matrix3 LEG_ref_R, std::deque<vector2> &rfzmp, ZmpPlaner *zmpP)
{
  vector2  swLegRef_p;
  if((FT==FSRFsw)||(FT==RFsw)){
    swLegRef_p = pfromVector3(RLEG_ref_p) ;
  }
  else if((FT==FSLFsw)||(FT==LFsw)){
    swLegRef_p = pfromVector3(LLEG_ref_p) ;
  }
  
  //_/_/_/_/_/_/_/_/_/_/_/_/_/_/_//_/_/_/_/_/_/_/_/_/_/_/_/_/_/_  //_/_/_/_/_/_/_/_/_/_/_/_/_/_/_
  //limit
  int SupLeg;
  Vector3  SwLeg_p_ref;
  double limit_y;
  //RLEG_ref_R= LLEG_ref_R=obj
  if((FT==FSRFsw)||FT==RFsw){
    SupLeg=LLEG;
    SwLeg_p_ref=RLEG_ref_p;
    limit_y=-0.17;
  }
  else if((FT==FSLFsw)||FT==LFsw){
    SupLeg=RLEG;
    SwLeg_p_ref=LLEG_ref_p;
    limit_y=0.17;
  }
  
  Vector3 Shift2Zero(R_ref[SupLeg].transpose()*( SwLeg_p_ref - p_ref[SupLeg]));
  if(fabs(Shift2Zero(1))<0.17)
    {
      Shift2Zero(1)=limit_y;
      SwLeg_p_ref= p_ref[SupLeg] + R_ref[SupLeg] * Shift2Zero;
      //adjust
      swLegRef_p= pfromVector3( SwLeg_p_ref);
      //cerr<<"interference"<<endl;
    }
  //_/_/_/_/_/_/_/_/_/_/_/_/_/_/_//_/_/_/_/_/_/_/_/_/_/_/_/_/_/_  //_/_/_/_/_/_/_/_/_/_/_/_/_/_/_
  
  rfzmp.clear();
  
  if(CommandIn==5)
    zmpP->PlanCPstop(FT, p_ref, R_ref, swLegRef_p, LEG_ref_R, rfzmp);
  else 
    zmpP->PlanCP(FT, p_ref, R_ref, swLegRef_p, LEG_ref_R, rfzmp);
  
 
  //zmpP->PlanZMPnew(FT, p_ref, R_ref, swLegRef_p, LEG_ref_R, rfzmp);///plan rzmp&swingLeg traje
  //toe mode
  //zmpP->PlanZMPnew_toe(FT, p_ref, R_ref, swLegRef_p, LEG_ref_R, rfzmp);///plan rzmp&swingLeg traje
  
}

void sony::walkingMotion(BodyPtr m_robot, FootType FT, Vector3 &cm_ref, Vector3 &absZMP, Vector3 *p_Init, Vector3 *p_ref, Matrix3 *R_ref, std::deque<vector2> &rfzmp, PreviewControl *PC, ZmpPlaner *zmpP, int &count)
{
  //capture point 
  zmpP->getNextCom(cm_ref);
  //swingLeg nomal mode
  if(!zmpP->swLegxy.empty()){
    int swingLeg=swLeg(FT);
    p_ref[swingLeg](0)=zmpP->swLegxy.at(0)[0];
    p_ref[swingLeg](1)=zmpP->swLegxy.at(0)[1];
    p_ref[swingLeg](2)=p_Init[swingLeg][2]+zmpP->Trajzd.at(0);
    R_ref[swingLeg]= zmpP->swLeg_R.at(0);
    zmpP->calcWaistR(FT,  R_ref); 


    //contact states
    if(zmpP->Trajzd.at(0)<1e-9)
      m_contactStates.data[swingLeg]=1;
    else 
      m_contactStates.data[swingLeg]=0;


    zmpP->swLegxy.pop_front();
    zmpP->Trajzd.pop_front();
    zmpP->swLeg_R.pop_front();    
  }

  /*
  //preview control
  PC->calcInput(rfzmp);//input recalculated
  PC->calcNextState();
  cm_ref(0)=PC->cur_state(0,0);
  cm_ref(1)=PC->cur_state(0,1);
  PC->update();
  
  ///rzmp To st (already in zmphandler?)
  absZMP[0]=rfzmp.at(0)(0);
  absZMP[1]=rfzmp.at(0)(1);

  //swingLeg nomal mode
  int swingLeg=swLeg(FT);
  p_ref[swingLeg](0)=zmpP->swLegxy.at(count)(0);
  p_ref[swingLeg](1)=zmpP->swLegxy.at(count)(1);
  p_ref[swingLeg](2)=p_Init[swingLeg](2)+zmpP->Trajzd.at(count);
  R_ref[swingLeg]= zmpP->swLeg_R.at(count);
  zmpP->calcWaistR(FT,  R_ref); 
  */


  /*
  //toe mode
  if((FT==FSRFsw)||(FT==RFsw)){
    pt_R->b =zmpP->link_b_deque.at(count);
    s2sw_R->calcForwardKinematics();
  }
  else if((FT==FSLFsw)||(FT==LFsw)){
    pt_L->b =zmpP->link_b_deque.at(count);
    s2sw_L->calcForwardKinematics();
  }
  
  //swingLeg
  int swingLeg=swLeg(FT);
  p_ref[swingLeg](0)=zmpP->swLegxy.at(count)[0];
  p_ref[swingLeg](1)=zmpP->swLegxy.at(count)[1];
  p_ref[swingLeg](2)=zmpP->Trajzd.at(count);
  R_ref[swingLeg]= zmpP->swLeg_R.at(count) * zmpP->rot_pitch.at(count);
  zmpP->calcWaistR(FT, R_ref_toe); 
  */
  count++;
}

//change supleg buckup
/*
bool sony::ChangeSupLeg(BodyPtr m_robot, FootType &FT, int &count, ZmpPlaner *zmpP, PreviewControl *PC, bool &stopflag, int &CommandIn, Vector3 *p_ref, Vector3 *p_Init, Matrix3 *R_ref, Matrix3 *R_Init)
{
  bool ifchange=0;

  switch(FT){
  case FSRFsw:
    if((count== zmpP->step1Num )&&(!stopflag)){
    FT=LFsw;
      IniNewStep(m_robot, FT, count, zmpP, PC, stopflag, CommandIn, p_ref, p_Init, R_ref, R_Init);
      ifchange=1;
    }
    break;
    
  case FSLFsw:
    if((count== zmpP->step1Num)&&(!stopflag)){
      FT=RFsw;
      IniNewStep(m_robot, FT, count, zmpP, PC, stopflag, CommandIn, p_ref, p_Init, R_ref, R_Init);
      ifchange=1;
    }
    break;
    
  case LFsw:
    if(count==zmpP->NomalPaceNum){	
      FT=RFsw; 
      IniNewStep(m_robot, FT, count, zmpP, PC, stopflag, CommandIn, p_ref, p_Init, R_ref, R_Init);
      ifchange=1;
    }
    break;
    
  case RFsw:
    if(count==zmpP->NomalPaceNum){
      FT=LFsw;
      IniNewStep(m_robot, FT, count, zmpP, PC, stopflag, CommandIn, p_ref, p_Init, R_ref, R_Init);
      ifchange=1;
    }
    break;
  } 
  return ifchange;
}
*/
bool sony::ChangeSupLeg(BodyPtr m_robot, FootType &FT, int &count, ZmpPlaner *zmpP, PreviewControl *PC, bool &stopflag, int &CommandIn, Vector3 *p_ref, Vector3 *p_Init, Matrix3 *R_ref, Matrix3 *R_Init)
{
  bool ifchange=0;

  if((zmpP->cp_deque.empty())&&(!stopflag)){

    //cp walking
    if(FT==FSRFsw||FT==LFsw)
      FT=RFsw; 
    else if(FT==FSLFsw||FT==RFsw)
      FT=LFsw; 

  /*
  //buckup
    if(FT==FSRFsw || FT==RFsw)
      FT=LFsw; 
    else if(FT==FSLFsw || FT==LFsw)
      FT=RFsw; 
    */

    //change leg
    IniNewStep(m_robot, FT, count, zmpP, PC, stopflag, CommandIn, p_ref, p_Init, R_ref, R_Init);
    ifchange=1;
  }

  return ifchange;
}


void sony::IniNewStep(BodyPtr m_robot, FootType &FT, int &count, ZmpPlaner *zmpP, PreviewControl *PC, bool &stopflag, int &CommandIn, Vector3 *p_ref, Vector3 *p_Init, Matrix3 *R_ref, Matrix3 *R_Init)
{ 
  updateInit(p_ref, p_Init, R_ref, R_Init);
  count=0;
  //ifstop
  if(CommandIn==5){
    stopflag=1;
    //PC->CoMInpo(m_robot, time2Neutral);//for preview control
    
    if (FT==RFsw)
      FT=FSRFsw;
    else if(FT==LFsw)
      FT=FSLFsw;
  }
  zmpP->stopOper=1;//unused
}

//_/_/_/_/_/_/_/_/_service port/_//_/_/_/_/_/_/_/_/_/_/_/_/_/_/_  //_/_/_/_/_/_/_/_/_/_/_/_/_/_/_   
void sony::start()
{
  //for expos
  m_mcIn.read();
  for(unsigned int i=0;i<m_mc.data.length();i++)
    m_refq.data[i]=body_cur(i)=m_mc.data[i];
  setModelPosture(m_robot, m_mc, FT, p_Init, R_Init);
  RenewModel(m_robot, p_now, R_now);

  cm_ref=m_robot->calcCenterOfMass();// 
  cout<<"cm "<<cm_ref<<endl;
  //cout<<"inipos"<<'\n'<<m_robot->link("RLEG_JOINT5")->R()<<'\n'<<m_robot->link("LLEG_JOINT5")->R()<<endl;

  //for expos
  for(int i=0;i<LINKNUM;i++){
    p_Init[i]=p_now[i];
    R_Init[i]=R_now[i];
    p_ref[i]=p_now[i];
    R_ref[i]=R_now[i];
    p_ref_toe[i]=p_now[i];
    R_ref_toe[i]=R_now[i];
  }
  R_ref[WAIST]=Eigen::MatrixXd::Identity(3,3);
 
 
  //tvmet::identity<hrp::Matrix3>();
  object_ref->R()= Eigen::MatrixXd::Identity(3,3);
  object_ref->p()= (p_Init[RLEG] +  p_Init[RLEG] )/2;
  
  //class ini
  zmpP= new ZmpPlaner;
  PC= new PreviewControl(0.005, cm_ref(2), 9.8);
  if(!PC->loadGain( kgain,  fgain))
    std::cerr<<"GainLoadErr"<<std::endl; 

  //for path planning/////////////////////////////////////////
  //ini
  p_obj2RLEG = p_Init[RLEG] - object_ref->p(); 
  p_obj2LLEG = p_Init[LLEG] - object_ref->p(); 
  R_LEG_ini=  LEG_ref_R= Eigen::MatrixXd::Identity(3,3);

  //for pivot///////////////////////////////////////////////
  Position T;
  T.linear()= Eigen::MatrixXd::Identity(3,3);
  T.translation()=Vector3(0.13, 0.0, -0.105);
  pt_R->setOffsetPosition(T);
  pt_L->setOffsetPosition(T);
  pt_R->setName("pivot_R");
  pt_L->setName("pivot_L");
  pt_R->setJointType(cnoid::Link::FIXED_JOINT);
  pt_L->setJointType(cnoid::Link::FIXED_JOINT);
  m_robot->link("RLEG_JOINT5")->appendChild(pt_R);
  m_robot->link("LLEG_JOINT5")->appendChild(pt_L);
  m_robot->updateLinkTree();
  m_robot->calcForwardKinematics();
  //cout<<m_profile.instance_name<<":pivot "<<m_robot->link("pivot_R")->p()<<endl;
  //cout<<m_robot->link("RLEG_JOINT5")->p()<<endl;
  
  /*  
  pt_R->b()<<0.13, 0.0, -0.105;
  pt_L->b()<<0.13, 0.0, -0.105;
  pt_R->jointType=cnoid::Link::FIXED_JOINT;
  pt_L->jointType=cnoid::Link::FIXED_JOINT;
  m_robot->link("RLEG_JOINT5")->addChild(pt_R);
  m_robot->link("LLEG_JOINT5")->addChild(pt_L);
 
  s2sw_R=JointPathPtr(new JointPath(m_robot->link("LLEG_JOINT5"), m_robot->link("RLEG_JOINT5")->child));
  s2sw_R->calcForwardKinematics();
  s2sw_L=JointPathPtr(new JointPath(m_robot->link("RLEG_JOINT5"), m_robot->link("LLEG_JOINT5")->child));
  s2sw_L->calcForwardKinematics();
  p_ref_toe[RLEG]=m_robot->link("RLEG_JOINT5")->child->p();
  p_ref_toe[LLEG]=m_robot->link("LLEG_JOINT5")->child->p();
  */
  //////////////////////////////////////////////////////////////

  rotRTemp=object_ref->R();
  cout<<"startQ "<<cm_ref(2)<<endl;
  

  double w=sqrt(9.806/cm_ref(2));
  zmpP->setw(w);

  //ooo
  playflag=1;
  
}

void sony::stepping()
{
  step=!step;
  cout<<"step"<<endl;
}

void sony::testMove()
{
  cout<<"test move"<<endl;
  vector32 zero;
  vector32 body_cur;
  zero=MatrixXd::Zero(32,1);
  body_cur=MatrixXd::Zero(32,1);
  /*
  //ver1
  body_ref<<0, 0.00332796, -0.482666, 0.859412, -0.370882, -0.00322683,
            0, 0.00332796, -0.482666, 0.859412, -0.370882, -0.00322683,  0,  0,  0,  0,
            0.135465, -0.290561, 0.14261, -1.81385, 1.30413, 0.0651451, 0.202547,  0,
            0.135465, 0.290561, -0.14261, -1.81385, -1.30413, -0.0651451, 0.202547,  0;
  //ver2
 body_ref<<8.40779e-07, 0.00313577, -0.450571, 0.78395, -0.332793, -0.00312566, 
           8.41168e-07, 0.00313571, -0.450557, 0.783916, -0.332772, -0.00312559, 0, 0, 0, 0,
           0.698132, -0.122173, -0, -1.50971, -0.122173, 0, 0, 0, 
           0.698132, 0.122173, 0, -1.50971, 0.122173, 0, 0, 0;
  */

  
  body_ref<<7.6349e-07, 0.00326766, -0.409632, 0.787722, -0.377504, -0.00325755,
    //7.63749e-07, 0.0032676, -0.409578, 0.787609, -0.377443, -0.00325748, 
            7.6349e-07, 0.00326766, -0.409632, 0.787722, -0.377504, -0.00325755,
            0, 0, 0, 0, 
            0.698132, -0.122173, 0, -1.50971, -0.122173, 0, 0, 0,
            0.698132,  0.122173, 0, -1.50971,  0.122173, 0, 0, 0;
	 
  Interplation5(body_cur,  zero,  zero, body_ref,  zero,  zero, 1, bodyDeque);
  /*
  //
  //new posture
   body_ref<<0, 0.00332796, -0.482666, 0.859412, -0.370882, -0.00322683,
            0, 0.00332796, -0.482666, 0.859412, -0.370882, -0.00322683,  0,  0,  0,  0,
       deg2rad(40.0), -deg2rad(7.0),  -deg2rad(0.0),   deg2rad(-86.5),  -deg2rad(7.0), 0.0, 0.0, 0.0,
       deg2rad(40.0),  deg2rad(7.0),   deg2rad(0.0),   deg2rad(-86.5),  deg2rad(7.0), 0.0, 0.0, 0.0 ;
  
 for(int i=0;i<32;i++)
   m_mc.data[i]=body_ref(i);
 setModelPosture(m_robot, m_mc, FT, p_Init, R_Init);
 RenewModel(m_robot, p_now, R_now);
 cm_ref=m_robot->calcCenterOfMass();// 
 cout<<"cm "<<cm_ref<<endl;
 //for expos
 for(int i=0;i<LINKNUM;i++){
   p_Init[i]=p_now[i];
   R_Init[i]=R_now[i];
   p_ref[i]=p_now[i];
   R_ref[i]=R_now[i];
   p_ref_toe[i]=p_now[i];
   R_ref_toe[i]=R_now[i];
 }
 R_ref[WAIST]=Eigen::MatrixXd::Identity(3,3);

 cm_ref(0)=cm_ref(1)=0.0;
 cm_ref(2)=0.827752;
 if(CalcIVK_biped(m_robot, cm_ref, p_ref, R_ref, FT, p_Init, R_Init)){
   cout<<"okok"<<endl;
   for(unsigned int i=0;i<32;i++){
     body_ref(i)=m_robot->joint(i)->q();
     cout<<body_ref(i)<<", ";
   }
   cout<<endl;
 }
 else
   cout<<"ivk error"<<endl;
 Interplation5(body_cur,  zero,  zero, body_ref,  zero,  zero, 3, bodyDeque);
 */

  /*
  cm_ref(0)+=0.125;
  p_ref[RLEG](0)+=0.25;

 if( CalcIVK_biped(m_robot, cm_ref, p_ref, R_ref, FT, p_Init, R_Init)){
    for(int i=0;i<m_robot->numJoints();i++)
      body_ref(i)=m_robot->joint(i)->q;
    SeqPlay32(body_cur, body_ref, bodyDeque, 1);
  }
  else 
    cerr<<"errrr"<<endl;
  */

  /*  
  JointPathPtr C2RARM;
  Vector3 tar_p(p_now[RARM]);
  Matrix3 tar_R(R_now[RARM]);
   
  C2RARM=m_robot->getJointPath(m_robot->link("CHEST_JOINT1"), m_robot->link("RARM_JOINT6"));
 pt= new hrp::Link();
  for(int i=0;i<m_robot->numJoints();i++)
    body_cur[i]=m_robot->joint(i)->q;

  //tar_p(0)+=0.02;
  Vector3 rpyTemp;
  rpyTemp=Vector3(-30*M_PI/180, 0, 0) ;
  Matrix3 rotRTemp(hrp::rotFromRpy(rpyTemp));
  tar_R = rotRTemp * R_now[RARM];

  if( C2RARM->calcInverseKinematics(tar_p, tar_R)){
    for(int i=0;i<m_robot->numJoints();i++)
      body_ref[i]=m_robot->joint(i)->q;
  }
  else
    cerr<<"inv arm err"<<endl;
  
  SeqPlay32(body_cur, body_ref, bodyDeque, 1);
  */
  
  
  /*
  //pt in aram
  pt->b=0,0.15,0;
  pt->name="sase";
  pt->jointType=hrp::Link::FIXED_JOINT;
  m_robot->link("RARM_JOINT6")->addChild(pt);
  JointPathPtr C2pt;
  //cerr<<m_robot->link("RARM_JOINT6")->child->jointType<<endl;
  //cerr<<m_robot->link("RARM_JOINT6")->d<<endl;

  //cerr<<m_robot->link("sase")->jointType<<endl;//ng
  C2pt=JointPathPtr(new JointPath(m_robot->link("CHEST_JOINT1"), m_robot->link("RARM_JOINT6")->child));
  C2pt->calcForwardKinematics();
  //cerr<< m_robot->link("RARM_JOINT6")->R()<<'\n'<<m_robot->link("RARM_JOINT6")->child->R()<<endl;
  dmatrix gg;
  C2pt->calcJacobian(gg);
  //cerr<<C2pt->numJoints()<<endl;
  
  Vector3  tar_p(m_robot->link("RARM_JOINT6")->child->p());
  Matrix3 tar_R(m_robot->link("RARM_JOINT6")->child->R());
  
   for(int i=0;i<m_robot->numJoints();i++)
    body_cur[i]=m_robot->joint(i)->q;

  //tar_p(0)+=0.02;
  Vector3 rpyTemp;
  rpyTemp=Vector3(0, 60*M_PI/180, 0) ;
  Matrix3 rotRTemp(hrp::rotFromRpy(rpyTemp));
  tar_R = rotRTemp * m_robot->link("RARM_JOINT6")->child->R();

  if( C2pt->calcInverseKinematics(tar_p, tar_R)){
    for(int i=0;i<m_robot->numJoints();i++)
      body_ref[i]=m_robot->joint(i)->q;
  }
  else
    cerr<<"inv arm err"<<endl;

    SeqPlay32(body_cur, body_ref, bodyDeque, 1);
  */

  /*
  pt->b=0.13, 0,-0.105;
  //pt->b=0, 0, -0.105;
  pt->name="pivot";
  pt->jointType=hrp::Link::FIXED_JOINT;
  m_robot->link("RLEG_JOINT5")->addChild(pt);

  JointPathPtr s2sw;
  //cerr<<m_robot->link("RARM_JOINT6")->child->jointType<<endl;
  //cerr<<m_robot->link("RARM_JOINT6")->d<<endl;

  //cerr<<m_robot->link("sase")->jointType<<endl;//ng
  s2sw=JointPathPtr(new JointPath(m_robot->link("LLEG_JOINT5"), m_robot->link("RLEG_JOINT5")->child));
  s2sw->calcForwardKinematics();
  cerr<<m_robot->link("RLEG_JOINT5")->child->p()<<'\n'<<m_robot->link("RLEG_JOINT5")->p()<<endl;
  */ 

  /*
  Vector3  tar_p(m_robot->link("RLEG_JOINT5")->child->p());
  Matrix3 tar_R(m_robot->link("RLEG_JOINT5")->child->R());
  for(int i=0;i<m_robot->numJoints();i++)
    body_cur[i]=m_robot->joint(i)->q;

  Vector3 rpyTemp;
  rpyTemp=Vector3(0, 10*M_PI/180, 0) ;
  Matrix3 rotRTemp(hrp::rotFromRpy(rpyTemp));
  tar_R = rotRTemp * m_robot->link("RLEG_JOINT5")->child->R();
  //tar_p(0)+=0.02;
  //cm_ref(1)=p_ref[LLEG](1);

  p_ref[RLEG]=tar_p;
  R_ref[RLEG]=tar_R;

  if( CalcIVK_biped_toe(m_robot, cm_ref, p_ref, R_ref, FT, p_Init, R_Init)){
    for(int i=0;i<m_robot->numJoints();i++)
      body_ref[i]=m_robot->joint(i)->q;
    SeqPlay32(body_cur, body_ref, bodyDeque, 5);
  }
  else 
    cerr<<"errrr"<<endl;
  */
}

void sony::setObjectV(double x, double y, double z, double roll, double pitch, double yaw)
{ 
  /*
  vector32 body_cur;
  vector32 body_ref;
  cm_ref(1)=p_ref[LLEG](1);
  for(int i=0;i<m_robot->numJoints();i++)
   body_cur[i]=m_robot->joint(i)->q;

  if( CalcIVK_biped(m_robot, cm_ref, p_ref, R_ref, FT, p_Init, R_Init)){
    for(int i=0;i<m_robot->numJoints();i++)
      body_ref[i]=m_robot->joint(i)->q;
    SeqPlay32(body_cur, body_ref, bodyDeque, 5);
  }
  else 
    cerr<<"errrr"<<endl;
  */

  velobj<< x,y,z,roll,pitch,yaw;
}
//_/_/_/_/_/_/_/_/_/_/_/_/_/_/_//_/_/_/_/_/_/_/_/_/_/_/_/_/_/_  //_/_/_/_/_/_/_/_/_/_/_/_/_/_/_
   
extern "C"
{
 
  void sonyInit(RTC::Manager* manager)
  {
    coil::Properties profile(sony_spec);
    manager->registerFactory(profile,
                             RTC::Create<sony>,
                             RTC::Delete<sony>);
  }
  
};



