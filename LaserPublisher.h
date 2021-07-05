#ifndef ROSARIALASERPUBLISHER_H
#define ROSARIALASERPUBLISHER_H

#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Transform.h>



class ArLaser;
class ArTime;

class LaserPublisher
{
public:
  LaserPublisher(ArLaser* _l, rclcpp::Node* _n, bool _broadcast_transform = true, const std::string& _tf_frame = "laser", const std::string& _parent_tf_frame = "base_link", const std::string& _global_tf_frame = "odom");
  ~LaserPublisher();
protected:
  void readingsCB();
  void publishLaserScan();
  void publishPointCloud();

  ArFunctorC<LaserPublisher> laserReadingsCB;
  rclcpp::Node *node;
  ArLaser *laser;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laserscan_pub;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr pointcloud_pub;
  sensor_msgs::msg::LaserScan laserscan;
  sensor_msgs::msg::PointCloud  pointcloud;
  std::string tfname;
  std::string parenttfname;
  std::string globaltfname;
  tf2::Transform lasertf;
  std::shared_ptr<tf2_ros::TransformBroadcaster> transform_broadcaster;
  bool broadcast_tf;

  //ArTime *readingsCallbackTime;
};

#endif

