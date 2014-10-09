// A node that logs the average point of the given point cloud.
//
// Sample usage:
//   rosrun pcl_sample processor_node pointcloud2_in:=/camera/depth_registered/points
//
// Note: the PointCloud2 data from the lab Turtlebots uses a different
// coordinate system from the ROS convention. We will discuss this more later.

// Technically, console.h, node_handle.h, and subscriber.h are transitively
// included with ros.h. However, it's generally a good C++ practice to always
// "include what you use" (IWYU).
#include "pcl/point_types.h"                 // PointXYZRGB
#include "pcl_conversions/pcl_conversions.h" // pcl::fromROSMsg
#include "pcl_ros/point_cloud.h"             // pcl::PointCloud
#include "pcl_sample/processor.h"            // pcl_sample::Processor
#include "ros/console.h"                     // ROS_INFO
#include "ros/ros.h"                         // ros::init, ros::spin, etc.
#include "ros/node_handle.h"                 // ros::NodeHandle
#include "ros/subscriber.h"                  // ros::Subscriber
#include "sensor_msgs/PointCloud2.h"         // sensor_msgs::PointCloud2

using pcl_sample::Processor;

// Callback for when a PointCloud2 message is received.
void callback(const sensor_msgs::PointCloud2ConstPtr& msg) {
  // Use PCL's conversion.
  pcl::PointCloud<pcl::PointXYZRGB> cloud;
  pcl::fromROSMsg(*msg, cloud);
  pcl::PointXYZRGB average;

  // Call our library function. We're lucky it's a static method, since we want
  // to call it inside of our callback. If our processor class had some state,
  // then we'd need to make this callback a method in a new class, e.g.,
  // ProcessorNode. Or, this function could be a functor.
  // See http://wiki.ros.org/roscpp/Overview/Publishers%20and%20Subscribers#Callback_Types
  Processor::Average(cloud, &average);

  // Log the average point at the INFO level. You can specify other logging
  // levels using ROS_DEBUG, ROS_WARN, etc. Search for "ros logging" online.
  ROS_INFO("x: %f y: %f z: %f, r: %d g: %d b: %d",
    average.x, average.y, average.z,
    average.r, average.g, average.b);
}

int main(int argc, char** argv) {
  // Initialize this program as a ROS node.
  ros::init(argc, argv, "sub_pcl");
  ros::NodeHandle nh;

  // Subscribe to /pointcloud2_in. Normally this would be remapped to another
  // topic, such as /camera/depth_registered/points. Calls the callback
  // function when a point cloud comes in.
  ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>(
    "pointcloud2_in", 1, callback);
  ros::spin();
}

