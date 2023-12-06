
#ifndef CLUSTER_OBSTACLES_H_
#define CLUSTER_OBSTACLES_H_

#include <ros/ros.h>

#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <iostream>
#include <cmath>
#include <vector>
#include <bits/stdc++.h>
#include "turtlebot3_gazebo/Obstacle.h"
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#define DEG_2RAD (M_PI / 180.0)
#define RAD_2DEG (180.0 / M_PI)

class ClusterObstacles
{
 public:
  ClusterObstacles();
  ~ClusterObstacles();
  bool init();
  bool computeObstacles();

 private:
  // ROS NodeHandle
  ros::NodeHandle nh_;
  ros::NodeHandle nh_priv_;

  // ROS Parameters

  // ROS Time

  // ROS Topic Publishers
  ros::Publisher cmd_vel_pub_;
  ros::Publisher obstacle_info_pub_;
  ros::Publisher obstacle_cloud_pub_;

  // ROS Topic Subscribers
  ros::Subscriber laser_scan_sub_;
  ros::Subscriber odom_sub_;

  // Variables
  double escape_range_;
  double check_forward_dist_;
  double check_side_dist_;

  double scan_data_[3] = {0.0, 0.0, 0.0};
  double robot_pose_;

  // Function prototypes
  void updatecommandVelocity(double linear, double angular);
  void laserScanMsgCallBack(const sensor_msgs::LaserScan::ConstPtr &msg);
  void odomMsgCallBack(const nav_msgs::Odometry::ConstPtr &msg);
};
#endif // CLUSTER_OBSTACLES_H_
