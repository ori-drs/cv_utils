#include <ros/ros.h>
#include <ros/console.h>

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
    Pass(ros::NodeHandle node_, 
      const CommandLineConfig& cl_cfg_);
    
    ~Pass(){
    }    

  private:
    ros::NodeHandle node_;

    const CommandLineConfig cl_cfg_;
    StereoView* stereoView_;

};



Pass::Pass(ros::NodeHandle node_, const CommandLineConfig& cl_cfg_):
    cl_cfg_(cl_cfg_),node_(node_){

  std::string node_name = ros::this_node::getName();
  ROS_INFO("Node name: %s", node_name.c_str() );

  stereoView_ = new StereoView();
}


int main( int argc, char** argv ){
  ros::init(argc, argv, "stereo_view_app");
 
  CommandLineConfig cl_cfg;
  ros::NodeHandle nh("~");

  Pass app(nh, cl_cfg);
  cout << "Ready" << endl << "============================" << endl;

  ROS_INFO_STREAM("stereo_view_app ready");
  ros::spin();
  return 0;
}
