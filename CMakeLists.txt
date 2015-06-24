cmake_minimum_required(VERSION 2.6)

set(IDL_DIR ${CMAKE_CURRENT_BINARY_DIR}/)
set(CMAKE_ALLOW_LOOSE_LOOP_CONSTRUCTS true)
if (NOT CMAKE_MODULE_PATH)
  #old HDD
  #set(CMAKE_MODULE_PATH /opt/grx/share/hrpsys/cmake_modules/ CACHE STRING "set path to cmake_modules")
  set(CMAKE_MODULE_PATH ~/src/hrpsys/trunk/cmake_modules/ CACHE STRING "set path to cmake_modules")
  #set(CMAKE_MODULE_PATH ~/src/hrpsys_base/trunk/cmake_modules/ CACHE STRING "set path to cmake_modules")
  #set(CMAKE_MODULE_PATH ../..//cmake_modules/ CACHE STRING "set path to cmake_modules")
endif (NOT CMAKE_MODULE_PATH)

#message (STATUS "/_/_/_/_/_/_/_/_/_/_/_/=${IDL_DIR}")

####
## settings for OpenRTM and OpenHRP
##
include(FindPkgConfig)
include(CheckIncludeFiles)
find_package(OpenRTM REQUIRED)
find_package(OpenHRP REQUIRED)

#find_package(OpenSceneGraph REQUIRED)
#find_package(Choreonoid REQUIRED)

include_directories(${OPENRTM_INCLUDE_DIRS})
link_directories(${OPENRTM_LIBRARY_DIRS})
include_directories(${OPENHRP_INCLUDE_DIRS})
link_directories(${OPENHRP_LIBRARY_DIRS})

include_directories(${CHOREONOID_INCLUDE_DIRS})
link_directories(${CHOREONOID_LIBRARY_DIRS})

#include_directories(/opt/grx/include)
include_directories(/usr/local/include)
include_directories(${CMAKE_MODULE_PATH})
include_directories(${CMAKE_CURRENT_BINARY_DIR})
#link_directories(${CMAKE_MODULE_PATH})
#include_directories(${CMAKE_CURRENT_SOURCE_DIR}/preview_control)#noneed
##################
#########
##################
if(NOT CHOREONOID_DIR)
 if(UNIX)
    pkg_check_modules(CHOREONOID REQUIRED choreonoid-body-plugin)
    if(CHOREONOID_FOUND)
      set(CHOREONOID_DIR ${CHOREONOID_INCLUDE_DIRS})
    endif()
  endif()
else()
  set(CHOREONOID_INCLUDE_DIRS ${CHOREONOID_DIR})
endif()

message("korekore " ${CHOREONOID_DIR})

set(CHOREONOID_DIR ${CHOREONOID_DIR} CACHE PATH "The directory of the choreonoid library")

if(NOT CHOREONOID_INCLUDE_DIRS)
  message(FATAL_ERROR "Please specify the choreonoid directory to CHOREONOID_DIR.")
else()
  include_directories(${CHOREONOID_INCLUDE_DIRS})
  link_directories(${CHOREONOID_LIBRARY_DIRS})
endif()
######################
##################
# eigen
#if(NOT EIGEN_DIR)
# if(UNIX)
#    pkg_check_modules(EIGEN eigen3)
#    if(EIGEN_FOUND)
#      set(EIGEN_DIR ${EIGEN_INCLUDE_DIRS})
#    endif()
#  endif()
#else()
#  set(EIGEN_INCLUDE_DIRS ${EIGEN_DIR})
#endif()

#set(EIGEN_DIR ${EIGEN_DIR} CACHE PATH "The directory of the Eigen library")

#if(NOT EIGEN_INCLUDE_DIRS)
#  message(FATAL_ERROR "Please specify the Eigen directory to EIGEN_DIR.")
#else()
#  include_directories(${EIGEN_INCLUDE_DIRS})
#endif()
######################

#if(QNXNTO)
#  include_directories(/usr/pkg/include)
#  set(NO_COMP true)
#  set(CPACK_GENERATOR "DEB")
#  set(CPACK_DEB_COMPONENT_INSTALL ON)
#  if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
#    set(CPACK_SYSTEM_NAME "amd64")
#  else(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
#    set(CPACK_SYSTEM_NAME "i386")
#  endif(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
#  set(CPACK_PACKAGE_NAME "sonyComponent")
#  set(CPACK_DEBIAN_PACKAGE_MAINTAINTER "GRX Support")
#  set(CPACK_PACKAGE_CONTACT "support@generalrobotix.com")
#  set(CPACK_PACKAGE_VENDOR "General Robotix, Inc.")
#  #set(CPACK_PACKAGE_INSTALL_DIRECTORY ${CMAKE_INSTALL_PREFIX})
#  #set(CPACK_INSTALL_DIRECTORIES ${CMAKE_INSTALL_PREFIX})
#  #set(CPACK_INSTALL_PREFIX ${CMAKE_INSTALL_PREFIX})
#  set(CPACK_DEBIAN_BUILD_DEPENDS g++)
#  include(CPack)
#  include(UploadPPA)
#endif(QNXNTO)

include(GenerateStubSkel)

####
## for each component
##
set(comp_name sony)
set(comp_sources ${comp_name}.cpp)

generate_stub_skel(${comp_name}Service)
set(comp_sources ${comp_sources} ${comp_name}Service_impl.cpp ${comp_name}ServiceSk.cpp)
#generate_hrpsys_stub_skel_wrapper(SequencePlayerService)

#set(comp_sources ${comp_sources} SequencePlayerServiceStub.cpp)
set(comp_sources ${comp_sources} hrp2Base.cpp)
set(comp_sources ${comp_sources} myfunc.cpp)# to be revise tvmet
set(comp_sources ${comp_sources} ZmpPlaner.cpp)
#set(comp_sources ${comp_sources} ${CMAKE_CURRENT_SOURCE_DIR}/preview_control/PreviewControl.cpp)
#set(comp_sources ${comp_sources} NMSclassQ.cpp)



add_library(${comp_name} SHARED ${comp_sources})
target_link_libraries(${comp_name} ${CHOREONOID_LIBRARIES} CnoidBody CnoidBodyPlugin CnoidUtil CnoidBase )
target_link_libraries(${comp_name} ${OPENRTM_LIBRARIES} hrpModel-3.1 hrpUtil-3.1)

target_link_libraries(${comp_name} lapack blas f2c m)
set_target_properties(${comp_name} PROPERTIES PREFIX "")

if (NO_COMP)
  set(target ${comp_name})
else()
  set(target ${comp_name} ${comp_name}Comp)
  add_executable(${comp_name}Comp ${comp_name}Comp.cpp ${comp_sources})
  target_link_libraries(${comp_name}Comp ${OPENRTM_LIBRARIES} hrpModel-3.1 hrpUtil-3.1)
  target_link_libraries(${comp_name}Comp ${CHOREONOID_LIBRARIES} CnoidBody CnoidBodyPlugin CnoidUtil CnoidBase )
  target_link_libraries(${comp_name}Comp boost_filesystem)
  target_link_libraries(${comp_name}Comp m)
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -ggdb3 -fno-schedule-insns -fno-schedule-insns2 -fno-strict-aliasing")

#install(TARGETS  ${comp_name}
#  RUNTIME DESTINATION bin CONFIGURATIONS Release Debug
#  LIBRARY DESTINATION lib CONFIGURATIONS Release Debug
#  )

#INSTALL(FILES ${comp_name}.so  DESTINATION  "/home/wu/downloads/src/chnoid_dev/choreonoid/build/lib/choreonoid-1.5/rtc")
<<<<<<< HEAD
INSTALL(FILES ${comp_name}.so  DESTINATION  "~/choreonoid/build/lib/choreonoid-1.5/rtc")

=======

INSTALL(FILES ${comp_name}.so  DESTINATION  "~/choreonoid/build/lib/choreonoid-1.5/rtc")
>>>>>>> 8d3b60d78637226278cc12da0731af7b3893c841
#INSTALL(FILES  ${comp_name}Service.jar  DESTINATION  "/usr/local/share/java")
