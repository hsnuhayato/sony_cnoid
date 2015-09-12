#ifndef WUNEWTYPES_H
#define WUNEWTYPES_H


//#include "hrpModel/Body.h"
//#include "hrpModel/Link.h"
//#include "hrpModel/JointPath.h"
//#include "hrpModel/ModelLoaderUtil.h"
//#include "hrpUtil/MatrixSolvers.h"
//#include "hrpUtil/EigenTypes.h" 

#include<fstream>
#include<math.h>



#include <cnoid/Body>
#include<cnoid/BodyLoader>
#include<cnoid/VRMLBodyLoader>
#include <cnoid/EigenTypes>
#include <cnoid/JointPath>
#include <cnoid/Jacobian>
#include <cnoid/EigenUtil>
#include <cnoid/Sensor>
#include <cnoid/Link>
using std::cout; using std::endl;
using std::cerr;
using namespace std;
//using namespace hrp;
using namespace cnoid;
using namespace RTC;
using namespace Eigen;

typedef Eigen::Matrix<double, 3,3>  matrix33;
typedef Eigen::Matrix<double, 3,2> matrix32;
typedef Eigen::Matrix<double, 2,2> matrix22;
typedef Eigen::Matrix<double, 6,6> matrix66;
typedef Eigen::Matrix<double, 3,1>  vector3;
typedef Eigen::Matrix<double, 2,1> vector2;
typedef Eigen::Matrix<double, 4,1> vector4;
typedef Eigen::Matrix<double, 6,1> vector6;
typedef Eigen::Matrix<double, 12,1> vector12;
typedef Eigen::Matrix<double, 24,1> vector24;
typedef Eigen::Matrix<double, 32,1> vector32;


enum FootType {FSRFsw, FSLFsw, RFsw, LFsw};
enum{RLEG, LLEG, RARM, LARM, WAIST, LINKNUM};
enum StepDir {front,back};

struct wpgParam
{
  double Tsup;
  double Tsup_stepping;
  double Tdbl;
  double dt;
  double offsetZMPy;
  double offsetZMPy_stepping;
  double offsetZMPx;
  double Zup;
  double Tv;
  double pitch_angle;
  std::vector<double> link_b_front;
  std::vector<double> link_b_rear;
  std::vector<double> link_b_ankle;
  double ankle_height;
};

//#define Kgain_path "/home/grxuser/users/wu/hrp2rtc/preview_control/prm/"
//#define canonPace 0.09

#ifndef deg2rad
#define deg2rad(x)  (M_PI/180*(x))
#endif
#ifndef rad2deg
#define rad2deg(x)  ((x)*180/M_PI)
#endif

#endif
