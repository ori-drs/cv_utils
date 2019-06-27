#include <ros/ros.h>
#include <ros/console.h>

#include <stdio.h>
#include <iostream>

#include <message_filters/time_synchronizer.h>
#include <message_filters/subscriber.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/CameraInfo.h>

#include <sensor_msgs/PointCloud2.h>

#include <depth_image_utils/depth_image_utils.hpp>

using namespace std;


class Pass{
  public:
    Pass(ros::NodeHandle node_);
    
    ~Pass(){
    }    

  private:
    ros::NodeHandle node_;

    DepthImageUtils utils_;
    int counter_;

    std::string processed_cloud_pub_topic_;
    ros::Publisher processed_cloud_pub_;

    boost::shared_ptr<image_transport::SubscriberFilter> image_a_sub_, image_b_sub_;
    boost::shared_ptr<message_filters::Subscriber<sensor_msgs::CameraInfo> > info_a_sub_, info_b_sub_;
    boost::shared_ptr<message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::Image,
    sensor_msgs::CameraInfo> > sync_;

    std::string image_topic_a_, image_a_transport_, info_topic_a_;
    std::string image_topic_b_, image_b_transport_, info_topic_b_;

    boost::shared_ptr<image_transport::ImageTransport> it_;

    void depthImageCallback(const sensor_msgs::ImageConstPtr& image_a,
                            const sensor_msgs::CameraInfoConstPtr& info_a,
                            const sensor_msgs::ImageConstPtr& image_b,
                            const sensor_msgs::CameraInfoConstPtr& info_b);

};



Pass::Pass(ros::NodeHandle node_):node_(node_){
  processed_cloud_pub_topic_ = "/realsense_d435_front/depth/color/points";

  processed_cloud_pub_ = node_.advertise<sensor_msgs::PointCloud2>(processed_cloud_pub_topic_, 10);


  image_topic_a_     = "/realsense_d435_front/color/image_raw";
  image_a_transport_ = "compressed";
  info_topic_a_      = "/realsense_d435_front/color/camera_info";
  image_topic_b_     = "/realsense_d435_front/aligned_depth_to_color/image_raw";
  image_b_transport_ = "compressedDepth";
  info_topic_b_      = "/realsense_d435_front/aligned_depth_to_color/camera_info";

  image_a_sub_ = boost::make_shared<image_transport::SubscriberFilter>();
  image_b_sub_ = boost::make_shared<image_transport::SubscriberFilter>();
  it_ = boost::make_shared<image_transport::ImageTransport>(node_);
  image_a_sub_->subscribe(*it_, ros::names::resolve(image_topic_a_), 100, image_transport::TransportHints( image_a_transport_ ));
  image_b_sub_->subscribe(*it_, ros::names::resolve(image_topic_b_), 100, image_transport::TransportHints( image_b_transport_ ));

  info_a_sub_ = boost::make_shared<message_filters::Subscriber<sensor_msgs::CameraInfo> >();
  info_b_sub_ = boost::make_shared<message_filters::Subscriber<sensor_msgs::CameraInfo> >();
  info_a_sub_->subscribe(node_, ros::names::resolve(info_topic_a_), 100);
  info_b_sub_->subscribe(node_, ros::names::resolve(info_topic_b_), 100);

  sync_ = boost::make_shared<message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::Image,
      sensor_msgs::CameraInfo> >(10);
  sync_->connectInput(*image_a_sub_, *info_a_sub_, *image_b_sub_, *info_b_sub_);
  sync_->registerCallback(boost::bind(&Pass::depthImageCallback, this, _1, _2, _3, _4));


  counter_ = 0;
  cout << "Ready to convert" << endl << "============================" << endl;
  ros::spin();


}


void Pass::depthImageCallback(const sensor_msgs::ImageConstPtr& image_a,
                              const sensor_msgs::CameraInfoConstPtr& info_a,
                              const sensor_msgs::ImageConstPtr& image_b,
                              const sensor_msgs::CameraInfoConstPtr& info_b){
  counter_++;
  ROS_INFO_THROTTLE(10, "images received %d. %d so far", image_a->header.stamp.sec, counter_);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  utils_.unpackImage(image_a, info_a, image_b, info_b, cloud);

  //
  std::string frame_id = image_b->header.frame_id;
  ros::Time time = image_a->header.stamp;



  sensor_msgs::PointCloud2 msg;
  pcl::toROSMsg(*cloud, msg);

  // use the depth image to provide the frame and stamp
  msg.header.stamp = image_b->header.stamp; 
  msg.header.frame_id = image_b->header.frame_id;
  processed_cloud_pub_.publish(msg);
}

int main( int argc, char** argv ){

  ros::init(argc, argv, "depth_to_pointcloud_node");
  ros::NodeHandle nh("~");

  Pass app(nh);
  return 0;
}
