// -*- C++ -*-
/*!
 * @file  sony.h * @brief sonyComponent * @date  $Date$ 
 *
 * $Id$ 
 */
#ifndef HRP2BASE_H
#define HRP2BASE_H

#include <rtm/idl/BasicDataTypeSkel.h>
#include <rtm/idl/ExtendedDataTypesSkel.h>
#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/CorbaPort.h>
#include <rtm/DataInPort.h>
#include <rtm/DataOutPort.h>
//add
#include <rtm/CorbaNaming.h>

// Service implementation headers
// <rtc-template block="service_impl_h">

// </rtc-template>

// Service Consumer stub headers
// <rtc-template block="consumer_stub_h">

// </rtc-template>
// OpenHRP
//#include "hrpModel/Sensor.h"
#include <vector>
#include <iostream>
#include <sstream>

// </rtc-template>
//#include <Eigen/Dense>
//user
#include "myfunc.h"
#include "VConvert.h"
#include "ZmpPlaner.h"
#include "preview_control/PreviewControl.h"
//class


using namespace RTC;

class hrp2Base  : public RTC::DataFlowComponentBase
{
 public:
  hrp2Base(RTC::Manager* manager);
  ~hrp2Base();

  // The initialize action (on CREATED->ALIVE transition)
  // formaer rtc_init_entry() 
 virtual RTC::ReturnCode_t onInitialize();

  // The finalize action (on ALIVE->END transition)
  // formaer rtc_exiting_entry()
  // virtual RTC::ReturnCode_t onFinalize();

  // The startup action when ExecutionContext startup
  // former rtc_starting_entry()
  // virtual RTC::ReturnCode_t onStartup(RTC::UniqueId ec_id);

  // The shutdown action when ExecutionContext stop
  // former rtc_stopping_entry()
  // virtual RTC::ReturnCode_t onShutdown(RTC::UniqueId ec_id);
 
  // The activated action (Active state entry action)
  // former rtc_active_entry()
  // virtual RTC::ReturnCode_t onActivated(RTC::UniqueId ec_id);

  // The deactivated action (Active state exit action)
  // former rtc_active_exit()
  // virtual RTC::ReturnCode_t onDeactivated(RTC::UniqueId ec_id);

  // The execution action that is invoked periodically
  // former rtc_active_do()
  //virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);

  // The aborting action when main logic error occurred.
  // former rtc_aborting_entry()
  // virtual RTC::ReturnCode_t onAborting(RTC::UniqueId ec_id);

  // The error action in ERROR state
  // former rtc_error_do()
  // virtual RTC::ReturnCode_t onError(RTC::UniqueId ec_id);

  // The reset action that is invoked resetting
  // This is same but different the former rtc_init_entry()
  // virtual RTC::ReturnCode_t onReset(RTC::UniqueId ec_id);
  
  // The state update action that is invoked after onExecute() action
  // no corresponding operation exists in OpenRTm-aist-0.2.0
  // virtual RTC::ReturnCode_t onStateUpdate(RTC::UniqueId ec_id);

  // The action that is invoked when execution context's rate is changed
  // no corresponding operation exists in OpenRTm-aist-0.2.0
  // virtual RTC::ReturnCode_t onRateChanged(RTC::UniqueId ec_id);


 protected:
  // Configuration variable declaration
  // <rtc-template block="config_declare">
  
  // </rtc-template>

  // DataInPort declaration
  // <rtc-template block="inport_declare">
  TimedDoubleSeq m_q;
  InPort<TimedDoubleSeq> m_qIn;
  TimedDoubleSeq m_rhsensor;
  InPort<TimedDoubleSeq> m_rhsensorIn;
  TimedDoubleSeq m_lhsensor;
  InPort<TimedDoubleSeq> m_lhsensorIn;
  TimedDoubleSeq m_rfsensor;
  InPort<TimedDoubleSeq> m_rfsensorIn;
  TimedDoubleSeq m_lfsensor;
  InPort<TimedDoubleSeq> m_lfsensorIn;
  TimedDoubleSeq m_mc;
  InPort<TimedDoubleSeq> m_mcIn;

  // </rtc-template>

  // DataOutPort declaration
  // <rtc-template block="outport_declare">
  //old
  //TimedDoubleSeq m_rzmp;
  //OutPort<TimedDoubleSeq> m_rzmpOut;
  TimedPoint3D m_rzmp;
  OutPort<TimedPoint3D> m_rzmpOut;
  TimedDoubleSeq m_refq;
  OutPort<TimedDoubleSeq> m_refqOut;
  TimedBooleanSeq m_contactStates;
  OutPort<TimedBooleanSeq> m_contactStatesOut;
  TimedPoint3D m_basePos;
  OutPort<TimedPoint3D> m_basePosOut;
  TimedOrientation3D m_baseRpy;
  OutPort<TimedOrientation3D> m_baseRpyOut;
  // </rtc-template>

  // CORBA Port declaration
  // <rtc-template block="corbaport_declare">
  //RTC::CorbaPort m_hrp2BaseServicePort;

  // </rtc-template>

  // Service declaration
  // <rtc-template block="service_declare">
  //hrp2BaseService_impl m_service0;

  // </rtc-template>

  // Consumer declaration
  // <rtc-template block="consumer_declare">

  // </rtc-template>
  ZmpPlaner *zmpP;
  PreviewControl *PC;
  Properties prop;
  BodyPtr m_robot;
  int dof;
  double mass;
  int armDof;
  Vector3 absZMP,relZMP;
  Vector3 cm_ref;
  std::deque<vector2> rfzmp;
  
  std::vector<double> kgain;
  std::vector<double> fgain;
  FootType FT;

  Vector3 p_now[LINKNUM];
  Matrix3 R_now[LINKNUM];
  Vector3 p_ref[LINKNUM];
  Matrix3 R_ref[LINKNUM];
  Vector3 p_Init[LINKNUM];
  Matrix3 R_Init[LINKNUM];

  //ForceSensor * fsensorRLEG;
  //ForceSensor * fsensorLLEG;
  DeviceList<ForceSensor> forceSensors;
  DeviceList<AccelSensor> AccelSensors;
  DeviceList<RateGyroSensor> RateGyroSensors;
  void updates();

 private:

};


extern "C"
{
  DLL_EXPORT void hrp2BaseInit(RTC::Manager* manager);
};

#endif // HRP2BASE_H

