#ifndef PREVIEWCONTROL_H
#define PREVIEWCONTROL_H

#include <iostream>
#include <deque>
//#include <tvmet/Matrix.h>
//#include <tvmet/Vector.h>
#include<math.h>
// OpenHRP
//#include "hrpModel/Body.h"
//#include "hrpModel/Link.h"
//#include "hrpModel/JointPath.h"
//#include "hrpModel/ModelLoaderUtil.h"
//#include "hrpUtil/MatrixSolvers.h"
#//include "hrpUtil/EigenTypes.h" 
#include "myfunc.h"
#include "wuNewType.h"
class PreviewControl {
  
 public:
  PreviewControl();
  PreviewControl(double dt, double com_height, double gravity=0.98);
  ~PreviewControl();
  
  inline int numPreview() const {
    return preview_num;
  }

  void setParameter(double dt, double com_height, double gravity=0.98);
  bool loadGain(std::string path_K, std::string path_f);
  bool loadGain(std::vector<double> kgain, std::vector<double> fgain);
  //void calcNextState(const matrix32 &cur_state, const vector2 &input, matrix32 &next_state, vector2 &output);
  //void calcOutput(const matrix32 &state, vector2 &output);
  //void calcInput(const matrix32 &cur_state, const matrix32 &pre_state, const vector2 &cur_output, const std::deque<vector2> &ref_outputs, vector2 &input);

  void calcInput(const std::deque<vector2> &ref_outputs);  
  void calcNextState();

  void setInitial(Vector3 &cm);
  void update();
  void Inituk();
  //calc all CoM of one pace
  //void calcCOMs( FootType FT, std::deque<vector2> &rfzmp, std::deque<vector2> &rcoms, std::deque<vector2> &rzmps);

  matrix32 cur_state;
  matrix32 pre_state;
  vector2 uk;
  vector2 cur_output;

  void CoMInpo(BodyPtr body,double time2Neutral);
  std::deque<vector2> CoM2Stop;

 private:
   
  int preview_num;
  
  
  double  Ks;
  vector3 Kx;
  std::deque<double> fj;
  
  matrix33 A_pre;
  vector3  b_pre, c_pre;
  //wu///
  //bool first;
  matrix32 next_state;
  vector2 input;
  vector2 output;

};



#endif
