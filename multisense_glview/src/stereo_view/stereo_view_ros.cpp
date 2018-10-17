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

/*
    ros::Subscriber stopSub_, enableSub_, poseAnymalSub_, poseHuskySub_, drivingSub_, drivingRvizSub_, drivingRviz2Sub_, footstepSub_;
    void stopWalkingHandler(const std_msgs::StringConstPtr& msg);
    void enableHandler(const std_msgs::StringConstPtr& msg);
    void poseHandler(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg);
    void newDrivingGoalHandler(const geometry_msgs::PoseStampedConstPtr& msg);
    void newDrivingGoalRvizHandler(const geometry_msgs::PoseStampedConstPtr& msg);
    void newFootstepPlanRequestHandler(const geometry_msgs::PoseStampedConstPtr& msg);


    void computeControlCommand(Eigen::Isometry3d msg_pose, int64_t msg_utime, std_msgs::Header msg_header);

    ros::Publisher pathFollowerPub_,stopWalkingPub_;
    ros::Publisher visualizeCurrentGoalPub_, visualizeRemainingGoalsPub_;
*/
};



Pass::Pass(ros::NodeHandle node_, const CommandLineConfig& cl_cfg_):
    cl_cfg_(cl_cfg_),node_(node_){

  std::string node_name = ros::this_node::getName();
  ROS_INFO("Node name: %s", node_name.c_str() );

/*
  std::string poseInputTopic = "/state_estimator/pose_in_odom";
  if (!node_.getParam("pose_input_topic", poseInputTopic)) {
    ROS_ERROR("Could not read parameter `pose_input_topic`.");
    exit(-1);
  }
  ROS_INFO("Receive pelvis/base pose from %s", poseInputTopic.c_str());
  poseAnymalSub_     = node_.subscribe(poseInputTopic, 100, &Pass::poseHandler, this);

  stopSub_     = node_.subscribe(std::string("/stop_walking"), 100, &Pass::stopWalkingHandler, this);
  enableSub_   = node_.subscribe(std::string("/enable_path_follower"), 100, &Pass::enableHandler, this);
  drivingSub_  = node_.subscribe(std::string("/driving_plan_request"), 100, &Pass::newDrivingGoalHandler, this);
  drivingRvizSub_  = node_.subscribe(std::string("/goal"), 100, &Pass::newDrivingGoalRvizHandler, this);
  drivingRviz2Sub_  = node_.subscribe(std::string("/move_base_simple/goal"), 100, &Pass::newDrivingGoalRvizHandler, this); // rviz in certain configurations
  footstepSub_ = node_.subscribe(std::string("/path_follower/footstep_plan_request"), 100, &Pass::newFootstepPlanRequestHandler, this);

  std::string pathFollowerTopic;
  if (!node_.getParam("output_topic", pathFollowerTopic)) {
    ROS_ERROR("Could not read parameter `output_topic`.");
    exit(-1);
  }
  ROS_INFO("Publish commands to %s", pathFollowerTopic.c_str());
  pathFollowerPub_ = node_.advertise<geometry_msgs::Twist>(  pathFollowerTopic, 10);
  //pathFollowerPub_ = node_.advertise<geometry_msgs::Twist>( std::string( "/path_follower/" + pathFollowerTopic), 10);
  
  stopWalkingPub_ = node_.advertise<std_msgs::Int16>("/stop_walking_cmd",10);

  // diagnostics:
  visualizeCurrentGoalPub_ = node_.advertise<geometry_msgs::PoseStamped>("/path_follower/current_goal", 10);
  visualizeRemainingGoalsPub_ = node_.advertise<geometry_msgs::PoseArray>("/path_follower/remaining_goals", 10);
*/
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
