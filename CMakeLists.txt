cmake_minimum_required(VERSION 2.6)

set(CMAKE_ALLOW_LOOSE_LOOP_CONSTRUCTS true)
if (NOT CMAKE_MODULE_PATH)
  set(CMAKE_MODULE_PATH /opt/grx/share/hrpsys/cmake_modules/ CACHE STRING "set path to cmake_modules")
endif (NOT CMAKE_MODULE_PATH)

####
## settings for OpenRTM and Choreonoid
##
include(FindPkgConfig)
include(CheckIncludeFiles)
pkg_check_modules(OPENRTM REQUIRED openrtm-aist)

include_directories(${OPENRTM_INCLUDE_DIRS})
link_directories(${OPENRTM_LIBRARY_DIRS})

execute_process(
  COMMAND $ENV{RTM_ROOT}/bin/rtm-config --rtm-idldir
  OUTPUT_VARIABLE OPENRTM_IDL_DIR
  RESULT_VARIABLE RESULT
  OUTPUT_STRIP_TRAILING_WHITESPACE)
include(GenerateStubSkel)


pkg_check_modules(CHOREONOID REQUIRED choreonoid-body-plugin)
include_directories(${CHOREONOID_INCLUDE_DIRS})
link_directories(${CHOREONOID_LIBRARY_DIRS})


set(CREEK_PATH $ENV{HOME}/workspace/git/creeklib)
include_directories(${CREEK_PATH})
include_directories(${CMAKE_MODULE_PATH})


####
## for each component
##
set(comp_name sony)
set(comp_sources ${comp_name}.cpp)

generate_stub_skel(${comp_name}Service)
set(comp_sources ${comp_sources} ${comp_name}Service_impl.cpp ${comp_name}ServiceSk.cpp)

set(comp_sources ${comp_sources} hrp2Base.cpp)
set(comp_sources ${comp_sources} myfunc.cpp)# to be revise tvmet
set(comp_sources ${comp_sources} ZmpPlaner.cpp)


add_library(${comp_name} SHARED ${comp_sources})
target_link_libraries(${comp_name} ${OPENRTM_LIBRARIES})
target_link_libraries(${comp_name} ${CHOREONOID_LIBRARIES})
target_link_libraries(${comp_name} lapack blas f2c m)
set_target_properties(${comp_name} PROPERTIES PREFIX "")
set_property(DIRECTORY PROPERTY COMPILE_DEFINITIONS BOOST_DISABLE_ASSERTS)

if (NO_COMP)
  set(target ${comp_name})
else()
  set(target ${comp_name} ${comp_name}Comp)
  add_executable(${comp_name}Comp ${comp_name}Comp.cpp ${comp_sources})
  target_link_libraries(${comp_name}Comp ${OPENRTM_LIBRARIES})
  target_link_libraries(${comp_name}Comp ${CHOREONOID_LIBRARIES})
  target_link_libraries(${comp_name}Comp boost_filesystem)
  target_link_libraries(${comp_name}Comp m)
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -ggdb3 -fno-schedule-insns -fno-schedule-insns2 -fno-strict-aliasing")

#install(TARGETS ${target}
install(TARGETS ${comp_name} LIBRARY DESTINATION rtc CONFIGURATIONS)
#install(FILES connect.conf DESTINATION rtc)
