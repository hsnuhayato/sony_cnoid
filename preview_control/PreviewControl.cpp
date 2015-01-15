#include "PreviewControl.h"
#include <fstream>
#include <algorithm> // min()
//std::ofstream ofspre("/home/grxuser/users/wu/hrp2rtc/pre.log");
using namespace std;
//unused
PreviewControl::PreviewControl()
{
  preview_num = 0;
  uk<<0.0, 0.0;
}


PreviewControl::PreviewControl(double dt, double com_height, double gravity)
{
  preview_num = 0;
  uk<<0.0, 0.0;
  setParameter(dt, com_height, gravity);
  CoM2Stop.clear();
  //first=1;
}


void PreviewControl::setParameter(double dt, double com_height, double gravity)
{
  // set A
  A_pre = Eigen::MatrixXd::Identity(3,3);
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

/*
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
*/

bool PreviewControl::loadGain(std::vector<double> kgain, std::vector<double> fgain)
{
  if( kgain.empty() )
    return false;

  Ks= kgain[0];

  for(int i = 1; i < 4; i++)
    Kx(i-1)=kgain[i];


 if( fgain.empty() )
    return false;

  preview_num = 0;
  std::vector<double>::iterator itr=fgain.begin();
  for(itr=fgain.begin();itr!=fgain.end();itr++){
    double tmp;
    tmp=fgain[preview_num];
    fj.push_back(tmp);
    preview_num++;
  }

  cerr<<preview_num<<endl;
  return true;
}

//wu
void PreviewControl::calcInput(const deque<vector2> &ref_outputs)
{
  int max_loop = min(preview_num, (int)ref_outputs.size()) - 1;
  vector2 sum_fj_zmp(MatrixXd::Zero(2,1));

  for(int i = 0; i < max_loop; i++) {
    sum_fj_zmp += ( fj[i] * (ref_outputs[i+1] - ref_outputs[i]) );
  }
  vector2 duk(MatrixXd::Zero(2,1));
  matrix32 temp;
  temp=cur_state - pre_state;
  duk = -Ks * (cur_output - ref_outputs[0]) - temp.transpose()* Kx + sum_fj_zmp;
  uk += duk;
  input = uk;
}

void PreviewControl::calcNextState()
{
  next_state = A_pre * cur_state + b_pre * input.transpose();
  output = next_state.transpose() * c_pre;
}


////////add by wu//////////
void PreviewControl::setInitial(Vector3 &cm)
{
  cur_state<< cm(0) , cm(1),
            0.0, 0.0,
            0.0, 0.0;
  pre_state<< cm(0), cm(1),
            0.0, 0.0,
            0.0, 0.0;
 cur_output<< cm(0), cm(1);
 input<<0.0, 0.0;
 output<<cm(0),cm(1);

}

void PreviewControl::update()
{
  pre_state=cur_state;
  cur_state=next_state;
  cur_output=output;
}
void PreviewControl::Inituk()
{
  uk <<0.0, 0.0;
}

void PreviewControl::CoMInpo(BodyPtr body,double time2Neutral)
{
  //cerr<< cur_state(0,0)<<" "<<cur_state(0,1)<<endl;
  cout<<"CoMInpo"<<endl;
  CoM2Stop.clear();
 
  vector2 ps(2,1),dps(2,1),ddps(2,1),pf(2,1),dpf(2,1),ddpf(2,1);
  vector2 zmpoffset(2,1);
  //zmpoffset=0.01371,0.0;
  zmpoffset<<0.0,0.0;

  ps<<cur_state(0,0),cur_state(0,1);
  dps<<cur_state(1,0),cur_state(1,1);
  ddps<<cur_state(2,0),cur_state(2,1); 
  dpf<<0.0,0.0;
  ddpf<<0.0,0.0;                         
  ///////////////////////////////////////////////////////////////////my offset///////
  //matrix22 RLeg_R(RfromMatrix33(body->link("RLEG_JOINT5")->R));
  //matrix22 LLeg_R(RfromMatrix33(body->link("LLEG_JOINT5")->R));
  pf(0)=(body->link("RLEG_JOINT5")->p()(0) + body->link("LLEG_JOINT5")->p()(0))/2;
  pf(1)=(body->link("RLEG_JOINT5")->p()(1) + body->link("LLEG_JOINT5")->p()(1))/2;
  //pf= pf + (RLeg_R*zmpoffset + LLeg_R*zmpoffset)/2;
  Interplation5(ps, dps, ddps, pf, dpf, ddpf, time2Neutral, CoM2Stop);
}

