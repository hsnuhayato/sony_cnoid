// -*- C++ -*-
/*!
 * @file  sony.cpp * @brief sonyComponent * $Date$ 
 *
 * $Id$ 
 */
#include "hrp2Base.h"

// Module specification
// <rtc-template block="module_spec">
static const char* hrp2Base_spec[] =
  {
    "implementation_id", "hrp2Base",
    "type_name",         "hrp2Base",
    "description",       "hrp2BaseComponent",
    "version",           "1.0",
    "vendor",            "tohoku",
    "category",          "example",
    "activity_type",     "SPORADIC",
    "kind",              "hrp2BaseComponent",
    "max_instance",      "1",
    "language",          "C++",
    "lang_type",         "compile",
    // Configuration variables
    ""
  };
// </rtc-template>

hrp2Base::hrp2Base(RTC::Manager* manager)
    // <rtc-template block="initializer">
  : RTC::DataFlowComponentBase(manager),
    m_qIn("q", m_q),
    m_rhsensorIn("rhsensor", m_rhsensor),
    m_lhsensorIn("lhsensor", m_lhsensor),
    m_rfsensorIn("rfsensor", m_rfsensor),
    m_lfsensorIn("lfsensor", m_lfsensor),
    m_mcIn("mc", m_mc),
    m_rzmpOut("rzmp", m_rzmp),
    m_contactStatesOut("contactStates", m_contactStates),
    m_basePosOut("basePosOut", m_basePos),
    m_baseRpyOut("baseRpyOut", m_baseRpy),
    m_refqOut("refq", m_refq)
    //m_hrp2BaseServicePort("hrp2BaseService")
    // </rtc-template>
{
}

hrp2Base::~hrp2Base()
{
}


RTC::ReturnCode_t hrp2Base::onInitialize()
{
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  addInPort("q", m_qIn);
  addInPort("rhsensor", m_rhsensorIn);
  addInPort("lhsensor", m_lhsensorIn);
  addInPort("rfsensor", m_rfsensorIn);
  addInPort("lfsensor", m_lfsensorIn);
  addInPort("mc", m_mcIn);

  // Set OutPort buffer
  addOutPort("rzmp", m_rzmpOut);
  addOutPort("refq", m_refqOut);
  addOutPort("basePosIn", m_basePosOut);
  addOutPort("baseRpyIn", m_baseRpyOut);
  addOutPort("contactStates", m_contactStatesOut);

  // Set service provider to Ports
  //m_hrp2BaseServicePort.registerProvider("service0", "hrp2BaseService", m_service0);

  // Set service consumers to Ports

  // Set CORBA Service Ports
  //addPort(m_hrp2BaseServicePort);

  // </rtc-template>

  // <rtc-template block="bind_config">
  // Bind variables and configuration variable

  // </rtc-template>
  RTC::Properties& prop = getProperties();
  /*  
  RTC::Manager& rtcManager = RTC::Manager::instance();
  std::string nameServer = rtcManager.getConfig()["corba.nameservers"];
  int commaPos = nameServer.find(",");
  if (commaPos > 0)
    nameServer = nameServer.substr(0, commaPos);
  RTC::CorbaNaming naming(rtcManager.getORB(), nameServer.c_str());
  m_robot = hrp::BodyPtr(new hrp::Body());
   CosNaming::NamingContext_var m_rootNameContext = CosNaming::NamingContext::_duplicate(naming.getRootContext());

  if (!hrp::loadBodyFromModelLoader(m_robot,  prop["model"].c_str(), m_rootNameContext))
    {
      std::cerr <<" : failed to load model" << std::endl;
    }
  else
    cout<<"model load OK"<<endl;
  */

  cnoid::BodyLoader bl;
  m_robot=bl.load( prop["model"].c_str());
  std::cout<<"sony dof robot "<<m_robot->numJoints()<<std::endl;
  m_robot->rootLink()->p()<<0.0, 0.0, 0.705;
  for(unsigned int i=0;i<dof;i++)
    m_robot->joint(i)->q()=0; 
  //m_robot->rootLink()->p()(0)=0.0;
  //m_robot->rootLink()->p()=Vector3(0, 0 ,0);
  //std::cout<<"R "<<m_robot->rootLink()->name()<<std::endl;
  m_robot->calcForwardKinematics();
  Vector3 cm=m_robot->calcCenterOfMass();
  std::cout<<"sony centerof mass "<<cm<<endl;//m_robot->mass()

  mass=m_robot->mass();

  dof = m_robot->numJoints();
  if( m_robot->numJoints()==32){
    armDof=7;
  }
  else if( m_robot->numJoints()==30){
    armDof=6;
  }  

  prop["kgain"]>>kgain;
  prop["fgain"]>>fgain;
  
  //body set
  /*
  for(unsigned int i=0;i<dof;i++)
    m_robot->joint(i)->q()=0; 
  m_robot->link("WAIST")->p()<<0, 0, 0.705;
  m_robot->calcForwardKinematics();
  */
  RenewModel(m_robot, p_now, R_now);
  updateInit(p_now, p_Init, R_now, R_Init);
  m_robot->calcCenterOfMass();

  //sensor set
  //fsensorRLEG = m_robot->sensor<cnoid::ForceSensor>(0);
  //fsensorLLEG = m_robot->sensor<cnoid::ForceSensor>(1);
  forceSensors = m_robot->devices();
  AccelSensors = m_robot->devices();
  RateGyroSensors= m_robot->devices();
  //cout<<"forcesener localp "<<'\n'<<forceSensors[0]->p_local()<<endl;
  //cout<<"forcesener localR "<<'\n'<<forceSensors[0]->R_local()<<endl;
  //for(int i=0;i< (m_robot->numDevices());i++)
  cout<<"device num "<<m_robot->numDevices()<<endl;
  //cout<<"name "<<forceSensors[0]->name()<<endl;
  //cout<<"name "<<forceSensors[1]->name()<<endl;
  cout<<"name "<<AccelSensors[0]->name()<<endl;
  //Link* TLink=forceSensors[0]->link();
  //Link* TLink=m_robot->link("LLEG_JOINT5");
  RateGyroSensor* sen=RateGyroSensors[0];
  cout<<"GG "<<sen->link()->name()<<endl;

  Isometry3 local;
  local.linear()=AccelSensors[0]->R_local();
  local.translation()=AccelSensors[0]->p_local();
  //cout<<  local.translation()<<'\n'<<  local.linear()<<endl;
  //cout<<"forcesener f "<<'\n'<<forceSensors[0]->f()<<endl;
  //cout<<"forcesener tau "<<'\n'<<forceSensors[0]->tau()<<endl;
  //Matrix3 testR= forceSensors[0]->link()->R()* forceSensors[0]->R_local();
  ForceSensor* sensor=forceSensors[0];
  Matrix3 testR=sensor->link()->R()* sensor->R_local();
  cout<<"test R"<<'\n'<<testR<<endl;
  cout<<sensor->link()->name()<<endl;


    //data port
  m_q.data.length(dof);
  m_mc.data.length(dof);
  m_rhsensor.data.length(6);
  m_lhsensor.data.length(6);
  m_rfsensor.data.length(6);
  m_lfsensor.data.length(6);  
  //m_rzmp.data.length(3); 
  m_refq.data.length(dof);
  m_contactStates.data.length(2);
  m_contactStates.data[0]=m_contactStates.data[1]=1;
  
  return RTC::RTC_OK;
}

void hrp2Base::updates()
{
  /*
  if(m_mcIn.isNew()){
    m_mcIn.read();
    for(unsigned int i=0;i<m_mc.data.length();i++)
      m_refq.data[i]=m_mc.data[i];
    setModelPosture(m_robot, m_mc, FT, p_Init, R_Init);
    //RenewModel(m_robot, p_now, R_now);
  } 
  */
  if( m_rhsensorIn.isNew() ) 
    m_rhsensorIn.read();
  if( m_lhsensorIn.isNew() ) 
    m_lhsensorIn.read();
  if( m_rfsensorIn.isNew() ) 
    m_rfsensorIn.read();
  if( m_lfsensorIn.isNew() ) 
    m_lfsensorIn.read();
}

/*
RTC::ReturnCode_t hrp2Base::onFinalize()
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t hrp2Base::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t hrp2Base::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t hrp2Base::onActivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t hrp2Base::onDeactivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t hrp2Base::onExecute(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t hrp2Base::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t hrp2Base::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t hrp2Base::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t hrp2Base::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t hrp2Base::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/


extern "C"
{
 
  void hrp2BaseInit(RTC::Manager* manager)
  {
    coil::Properties profile(hrp2Base_spec);
    manager->registerFactory(profile,
                             RTC::Create<hrp2Base>,
                             RTC::Delete<hrp2Base>);
  }
  
};
 


