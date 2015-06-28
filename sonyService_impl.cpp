// -*-C++-*-
/*!
 * @file  sonyService_impl.cpp
 * @brief Service implementation code of sonyService.idl
 *
 */

#include "sonyService_impl.h"

/*
 * Example implementational code for IDL interface OpenHRP::sonyService
 */
sonyService_impl::sonyService_impl()
{
  // Please add extra constructor code here.
}


sonyService_impl::~sonyService_impl()
{
  // Please add extra destructor code here.
}


/*
 * Methods corresponding to IDL attributes and operations
 */
void sonyService_impl::start()
{
  m_comp->start();
}

void sonyService_impl::setObjectV(::CORBA::Double x, ::CORBA::Double y, ::CORBA::Double z, ::CORBA::Double roll, ::CORBA::Double pitch, ::CORBA::Double yaw)
{
  m_comp->setObjectV(x, y, z, roll, pitch, yaw);
}

void sonyService_impl::stepping()
{
  m_comp->stepping();
}

void sonyService_impl::testMove()
{
  m_comp->testMove();
}

void sonyService_impl::stop()
{
  m_comp->stop();
}



// End of example implementational code



