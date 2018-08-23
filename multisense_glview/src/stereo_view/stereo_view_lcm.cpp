#include <stdio.h>
#include <inttypes.h>
#include <iostream>

#include <stereo_view/stereo_view.hpp>


using namespace std;

struct CommandLineConfig
{
  std::string param_file;
};

class Pass{
  public:
    Pass(const CommandLineConfig& cl_cfg_);
    
    ~Pass(){
    }    

    void readParamFromLaunch();

  private:

    const CommandLineConfig cl_cfg_;
    StereoView* stereoView_;

};



Pass::Pass(const CommandLineConfig& cl_cfg_):
    cl_cfg_(cl_cfg_){

  stereoView_ = new StereoView();
}


int main( int argc, char** argv ){
  CommandLineConfig cl_cfg;

  Pass app(cl_cfg);
  cout << "Ready" << endl << "============================" << endl;

  return 0;
}
