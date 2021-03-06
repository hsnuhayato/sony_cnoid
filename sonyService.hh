// This file is generated by omniidl (C++ backend)- omniORB_4_1. Do not edit.
#ifndef __sonyService_hh__
#define __sonyService_hh__

#ifndef __CORBA_H_EXTERNAL_GUARD__
#include <omniORB4/CORBA.h>
#endif

#ifndef  USE_stub_in_nt_dll
# define USE_stub_in_nt_dll_NOT_DEFINED_sonyService
#endif
#ifndef  USE_core_stub_in_nt_dll
# define USE_core_stub_in_nt_dll_NOT_DEFINED_sonyService
#endif
#ifndef  USE_dyn_stub_in_nt_dll
# define USE_dyn_stub_in_nt_dll_NOT_DEFINED_sonyService
#endif






#ifdef USE_stub_in_nt_dll
# ifndef USE_core_stub_in_nt_dll
#  define USE_core_stub_in_nt_dll
# endif
# ifndef USE_dyn_stub_in_nt_dll
#  define USE_dyn_stub_in_nt_dll
# endif
#endif

#ifdef _core_attr
# error "A local CPP macro _core_attr has already been defined."
#else
# ifdef  USE_core_stub_in_nt_dll
#  define _core_attr _OMNIORB_NTDLL_IMPORT
# else
#  define _core_attr
# endif
#endif

#ifdef _dyn_attr
# error "A local CPP macro _dyn_attr has already been defined."
#else
# ifdef  USE_dyn_stub_in_nt_dll
#  define _dyn_attr _OMNIORB_NTDLL_IMPORT
# else
#  define _dyn_attr
# endif
#endif





_CORBA_MODULE OpenHRP

_CORBA_MODULE_BEG

#ifndef __OpenHRP_msonyService__
#define __OpenHRP_msonyService__

  class sonyService;
  class _objref_sonyService;
  class _impl_sonyService;
  
  typedef _objref_sonyService* sonyService_ptr;
  typedef sonyService_ptr sonyServiceRef;

  class sonyService_Helper {
  public:
    typedef sonyService_ptr _ptr_type;

    static _ptr_type _nil();
    static _CORBA_Boolean is_nil(_ptr_type);
    static void release(_ptr_type);
    static void duplicate(_ptr_type);
    static void marshalObjRef(_ptr_type, cdrStream&);
    static _ptr_type unmarshalObjRef(cdrStream&);
  };

  typedef _CORBA_ObjRef_Var<_objref_sonyService, sonyService_Helper> sonyService_var;
  typedef _CORBA_ObjRef_OUT_arg<_objref_sonyService,sonyService_Helper > sonyService_out;

#endif

  // interface sonyService
  class sonyService {
  public:
    // Declarations for this interface type.
    typedef sonyService_ptr _ptr_type;
    typedef sonyService_var _var_type;

    static _ptr_type _duplicate(_ptr_type);
    static _ptr_type _narrow(::CORBA::Object_ptr);
    static _ptr_type _unchecked_narrow(::CORBA::Object_ptr);
    
    static _ptr_type _nil();

    static inline void _marshalObjRef(_ptr_type, cdrStream&);

    static inline _ptr_type _unmarshalObjRef(cdrStream& s) {
      omniObjRef* o = omniObjRef::_unMarshal(_PD_repoId,s);
      if (o)
        return (_ptr_type) o->_ptrToObjRef(_PD_repoId);
      else
        return _nil();
    }

    static _core_attr const char* _PD_repoId;

    // Other IDL defined within this scope.
    
  };

  class _objref_sonyService :
    public virtual ::CORBA::Object,
    public virtual omniObjRef
  {
  public:
    void start();
    void setObjectV(::CORBA::Double x, ::CORBA::Double y, ::CORBA::Double z, ::CORBA::Double roll, ::CORBA::Double pitch, ::CORBA::Double yaw);
    void testMove();
    void stepping();
    void stop();

    inline _objref_sonyService()  { _PR_setobj(0); }  // nil
    _objref_sonyService(omniIOR*, omniIdentity*);

  protected:
    virtual ~_objref_sonyService();

    
  private:
    virtual void* _ptrToObjRef(const char*);

    _objref_sonyService(const _objref_sonyService&);
    _objref_sonyService& operator = (const _objref_sonyService&);
    // not implemented

    friend class sonyService;
  };

  class _pof_sonyService : public _OMNI_NS(proxyObjectFactory) {
  public:
    inline _pof_sonyService() : _OMNI_NS(proxyObjectFactory)(sonyService::_PD_repoId) {}
    virtual ~_pof_sonyService();

    virtual omniObjRef* newObjRef(omniIOR*,omniIdentity*);
    virtual _CORBA_Boolean is_a(const char*) const;
  };

  class _impl_sonyService :
    public virtual omniServant
  {
  public:
    virtual ~_impl_sonyService();

    virtual void start() = 0;
    virtual void setObjectV(::CORBA::Double x, ::CORBA::Double y, ::CORBA::Double z, ::CORBA::Double roll, ::CORBA::Double pitch, ::CORBA::Double yaw) = 0;
    virtual void testMove() = 0;
    virtual void stepping() = 0;
    virtual void stop() = 0;
    
  public:  // Really protected, workaround for xlC
    virtual _CORBA_Boolean _dispatch(omniCallHandle&);

  private:
    virtual void* _ptrToInterface(const char*);
    virtual const char* _mostDerivedRepoId();
    
  };


  _CORBA_MODULE_VAR _dyn_attr const ::CORBA::TypeCode_ptr _tc_sonyService;

_CORBA_MODULE_END



_CORBA_MODULE POA_OpenHRP
_CORBA_MODULE_BEG

  class sonyService :
    public virtual OpenHRP::_impl_sonyService,
    public virtual ::PortableServer::ServantBase
  {
  public:
    virtual ~sonyService();

    inline ::OpenHRP::sonyService_ptr _this() {
      return (::OpenHRP::sonyService_ptr) _do_this(::OpenHRP::sonyService::_PD_repoId);
    }
  };

_CORBA_MODULE_END



_CORBA_MODULE OBV_OpenHRP
_CORBA_MODULE_BEG

_CORBA_MODULE_END





#undef _core_attr
#undef _dyn_attr

void operator<<=(::CORBA::Any& _a, OpenHRP::sonyService_ptr _s);
void operator<<=(::CORBA::Any& _a, OpenHRP::sonyService_ptr* _s);
_CORBA_Boolean operator>>=(const ::CORBA::Any& _a, OpenHRP::sonyService_ptr& _s);



inline void
OpenHRP::sonyService::_marshalObjRef(::OpenHRP::sonyService_ptr obj, cdrStream& s) {
  omniObjRef::_marshal(obj->_PR_getobj(),s);
}




#ifdef   USE_stub_in_nt_dll_NOT_DEFINED_sonyService
# undef  USE_stub_in_nt_dll
# undef  USE_stub_in_nt_dll_NOT_DEFINED_sonyService
#endif
#ifdef   USE_core_stub_in_nt_dll_NOT_DEFINED_sonyService
# undef  USE_core_stub_in_nt_dll
# undef  USE_core_stub_in_nt_dll_NOT_DEFINED_sonyService
#endif
#ifdef   USE_dyn_stub_in_nt_dll_NOT_DEFINED_sonyService
# undef  USE_dyn_stub_in_nt_dll
# undef  USE_dyn_stub_in_nt_dll_NOT_DEFINED_sonyService
#endif

#endif  // __sonyService_hh__

