#include <ros/ros.h>
#include <ros/console.h>

#include <stdio.h>
#include <iostream>

#include <message_filters/time_synchronizer.h>
#include <message_filters/subscriber.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

using namespace std;


class Pass{
  public:
    Pass(ros::NodeHandle node_);
    
    ~Pass(){
    }    


  private:
    ros::NodeHandle node_;

    ros::Publisher image_a_out_pub_;
    ros::Publisher info_a_out_pub_;
    ros::Publisher image_b_out_pub_;
    ros::Publisher info_b_out_pub_;

    void depthImageCallback(const sensor_msgs::ImageConstPtr& image_a_in,
      const sensor_msgs::CameraInfoConstPtr& info_a_in);

};



Pass::Pass(ros::NodeHandle node_){
  std::string image_topic = "image_topic";
  node_.getParam("image_topic", image_topic);

  std::string info_topic = "info_topic";
  node_.getParam("info_topic", info_topic);

  ////
  std::string image_topic_out = "image_topic_out";
  node_.getParam("image_topic_out", image_topic_out);

  std::string info_topic_out = "info_topic_out";
  node_.getParam("info_topic_out", info_topic_out);

  message_filters::Subscriber<sensor_msgs::Image> image_a_sub(node_, image_topic, 30);
  message_filters::Subscriber<sensor_msgs::CameraInfo> info_a_sub(node_, info_topic, 30);
  message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::CameraInfo> sync(image_a_sub, info_a_sub, 10);
  sync.registerCallback(boost::bind(&Pass::depthImageCallback, this, _1, _2));

  image_a_out_pub_ = node_.advertise<sensor_msgs::Image>(image_topic_out, 10);
  info_a_out_pub_ = node_.advertise<sensor_msgs::CameraInfo>(info_topic_out, 10);

  std::string node_name = ros::this_node::getName() ;

  cout << "Ready to downsample" << endl << "============================" << endl;
  ros::spin();


}


void Pass::depthImageCallback(const sensor_msgs::ImageConstPtr& image_in,
                         const sensor_msgs::CameraInfoConstPtr& info_in){


  std::string node_name = ros::this_node::getName() ;
  ROS_INFO_STREAM_THROTTLE(1,"got msg: " << node_name);
  cv_bridge::CvImagePtr cv_image = cv_bridge::toCvCopy(image_in);
  int stride = 2;
  int rows = cv_image->image.rows;
  int cols = cv_image->image.cols;
  
  cv::Mat mat_out = cv::Mat::zeros(rows/stride, cols/stride, CV_16UC1);
  for(int i = 0, out_i = 0; i < rows; i+=stride, ++out_i)
  {
    const unsigned short* in_row = cv_image->image.ptr<unsigned short>(i);
    unsigned short* out_row = mat_out.ptr<unsigned short>(out_i);
    for(int j = 0, out_j = 0; j < cols; j+=stride, ++out_j)
    { 
      out_row[out_j] = in_row[j];
    }
  }
  
  cv_bridge::CvImage cv_image_out;
  cv_image_out.image = mat_out;
  cv_image_out.encoding = image_in->encoding;
  cv_image_out.header = image_in->header;
  
  sensor_msgs::Image image_out;
  cv_image_out.toImageMsg(image_out); 

  sensor_msgs::CameraInfo info_out = *info_in;
  info_out.height =  info_out.height/stride;
  info_out.width =  info_out.width/stride;
  image_a_out_pub_.publish(image_out);
  info_a_out_pub_.publish(info_out);

}

int main( int argc, char** argv ){

  ros::init(argc, argv, "downsample_pointcloud");
  ros::NodeHandle nh("~");

  Pass app(nh);
  return 0;
}
