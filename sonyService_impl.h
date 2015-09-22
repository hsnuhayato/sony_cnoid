// -*-C++-*-
/*!
 * @file  sonyService_impl.h
 * @brief Service implementation header of sonyService.idl
 *
 */

#include "sonyService.hh"
#include "sony.h"

#ifndef SONYSERVICE_IMPL_H
#define SONYSERVICE_IMPL_H
 
/*
 * Example class implementing IDL interface OpenHRP::sonyService
 */
class sony;

class sonyService_impl
  : public virtual POA_OpenHRP::sonyService,
    public virtual PortableServer::RefCountServantBase
{
private:
  // Make sure all instances are built on the heap by making the
  // destructor non-public
  //virtual ~sonyService_impl();
  sony * m_comp;

public:
  // standard constructor
  sonyService_impl();
  virtual ~sonyService_impl();

  // attributes and operations
  void start();
  void setObjectV(::CORBA::Double x, ::CORBA::Double y, ::CORBA::Double z, ::CORBA::Double roll, ::CORBA::Double pitch, ::CORBA::Double yaw);
  void testMove();
  void stepping();
  void stop();
  void omniWalkSwitch();
 
  void setFootPosR2();
  void setFootPosL2();
  void setFootPosR(double x, double y, double z, double r, double p, double w);
  void setFootPosL(double x, double y, double z, double r, double p, double w);

  void setComponent (sony * i_comp) {
    m_comp = i_comp;
  }
};



#endif // SONYSERVICE_IMPL_H


