#include "PreviewControl.h"
#include <fstream>
#include <algorithm> // min()

using namespace std;

PreviewControl::PreviewControl()
{
  preview_num = 0;
  uk = vector2(0.0);
}


PreviewControl::PreviewControl(double dt, double com_height, double gravity)
{
  preview_num = 0;
  uk = vector2(0.0);
  
  setParameter(dt, com_height, gravity);
  // plus by wu
  isInit=1;
}


void PreviewControl::setParameter(double dt, double com_height, double gravity)
{
  // set A
  A_pre = tvmet::identity<matrix33>();
  A_pre(0,1) = dt;
  A_pre(0,2) = dt*dt/2.0;
  A_pre(1,2) = dt;

  // set b
  b_pre(0) = dt*dt*dt/6.0;
  b_pre(1) = dt*dt/2.0;
  b_pre(2) = dt;

  // set c
  c_pre(0) = 1.0;
  c_pre(1) = 0.0;
  c_pre(2) = -com_height/gravity;
}


bool PreviewControl::loadGain(string path_K, string path_f)
{
  ifstream file_K(path_K.c_str());
  if( !file_K )
    return false;

  file_K >> Ks;
  for(int i = 0; i < 3; i++)
    file_K >> Kx(i);

  file_K.close();



  ifstream file_f(path_f.c_str());
  if( !file_f )
    return false;

  preview_num = 0;
  string st;
  while(getline(file_f, st)) {
    if( st.size() > 1 ) {
      stringstream ss;
      ss.str(st);

      double tmp;
      ss >> tmp;
      fj.push_back(tmp);

      preview_num++;
    }
  }
  file_f.close();

  return true;
}


void PreviewControl::calcNextState(const matrix32 &cur_state, const vector2 &input, matrix32 &next_state,vector2 &output)
{
  next_state = A_pre * cur_state + prod(b_pre, input);
  output = tvmet::prod(tvmet::trans(next_state), c_pre);
}


void PreviewControl::calcOutput(const matrix32 &state, vector2 &output)
{
  output = tvmet::prod(tvmet::trans(state), c_pre);
}


void PreviewControl::calcInput(const matrix32 &cur_state, const matrix32 &pre_state, const vector2 &cur_output, const deque<vector2> &ref_outputs, vector2 &input)
{
  int max_loop = min(preview_num, (int)ref_outputs.size()) - 1;

  vector2 sum_fj_zmp(0.0);
  for(int i = 0; i < max_loop; i++) {
    sum_fj_zmp += ( fj[i] * (ref_outputs[i+1] - ref_outputs[i]) );
  }

  vector2 duk;
  duk = -Ks * (cur_output - ref_outputs[0]) - tvmet::prod(tvmet::trans((cur_state - pre_state)), Kx) + sum_fj_zmp;
  uk += duk;
  input = uk;
}

matrix32 PreviewControl::prod(const vector3 &v3, const vector2 &v2)
{
  matrix32 out;
  for(int i = 0; i < 2; i++) {
    for(int j = 0; j < 3; j++) {
      out(j,i) = v3(j) * v2(i);
    }
  }

  return out;
}
