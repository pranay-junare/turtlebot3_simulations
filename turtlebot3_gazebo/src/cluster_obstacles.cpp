
#include "turtlebot3_gazebo/cluster_obstacles.h"

ClusterObstacles::ClusterObstacles()
  : nh_priv_("~")
{
  //Init gazebo ros turtlebot3 node
  ROS_INFO("Cluster Distinct Obstacles Node Init");
  auto ret = init();
  ROS_ASSERT(ret);
}

ClusterObstacles::~ClusterObstacles()
{
  updatecommandVelocity(0.0, 0.0);
  ros::shutdown();
}

/*******************************************************************************
* Init function
*******************************************************************************/
bool ClusterObstacles::init()
{
  // initialize ROS parameter
  std::string cmd_vel_topic_name = nh_.param<std::string>("cmd_vel_topic_name", "");

  // initialize variables
  escape_range_       = 30.0 * DEG_2RAD;
  check_forward_dist_ = 0.7;
  check_side_dist_    = 0.7;

  // initialize publishers
  cmd_vel_pub_   = nh_.advertise<geometry_msgs::Twist>(cmd_vel_topic_name, 10);
  obstacle_info_pub_   = nh_.advertise<turtlebot3_gazebo::Obstacle>("obstacle_info", 10);
  obstacle_cloud_pub_   = nh_.advertise<sensor_msgs::LaserScan>("obstacle_cloud_info", 10);

  // initialize subscribers
  laser_scan_sub_  = nh_.subscribe("scan", 10, &ClusterObstacles::laserScanMsgCallBack, this);
  odom_sub_ = nh_.subscribe("odom", 10, &ClusterObstacles::odomMsgCallBack, this);

  return true;
}

void ClusterObstacles::laserScanMsgCallBack(const sensor_msgs::LaserScan::ConstPtr &scan)
{
 pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  // Planar PointCloud creation from 2D laserscan data
  for (unsigned int i = 0; i < scan->ranges.size(); ++i)
  {
    float range = scan->ranges[i];
    if (range > scan->range_min && range < scan->range_max)
    {
      pcl::PointXYZ point;
      point.x = range * cos(scan->angle_min + i * scan->angle_increment);
      point.y = range * sin(scan->angle_min + i * scan->angle_increment);
      point.z = 0;
      cloud->points.push_back(point);
    }
  }

  //RANSAC using LINE_MODEL
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_LINE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(10);
  seg.setInputCloud(cloud);
  seg.segment(*inliers, *coefficients);

  // Segmented cloud 
  sensor_msgs::LaserScan output_scan = *scan;
  output_scan.ranges.clear();
  output_scan.intensities.clear();
  output_scan.header.frame_id = "base_scan";
  output_scan.angle_min = -M_PI / 2;
  output_scan.angle_max = M_PI / 2;
  output_scan.angle_increment = M_PI / 180;
  output_scan.time_increment = 0;
  output_scan.scan_time = 1.0 / 30.0;
  output_scan.range_min = 0.0;
  output_scan.range_max = 5.0;
  output_scan.header.stamp = ros::Time::now();
  
  std::vector<float> all_lengths;
  for (int i = 0 ; i < inliers->indices.size() - 1; i++)
  {
    float x1 = cloud->points[inliers->indices[0]].x;
    float y1 = cloud->points[inliers->indices[0]].y;
    float x2 = cloud->points[inliers->indices[inliers->indices.size() + 1]].x;
    float y2 = cloud->points[inliers->indices[inliers->indices.size() + 1]].y;
    float slope = (y2 - y1) / (x2 - x1);
    float intercept = y1 - slope * x1;
    float angle = atan2(slope, 1);
    float distance = fabs(intercept / sqrt(slope * slope + 1));
    all_lengths.push_back (sqrt(pow((x1-x2),2)+pow((y1-y2),2)));
    
    output_scan.ranges.push_back(distance);
    output_scan.intensities.push_back(angle);
  }
  obstacle_cloud_pub_.publish(output_scan);

  // Clustering using KD-Tree
  // ToDO:
}

void ClusterObstacles::odomMsgCallBack(const nav_msgs::Odometry::ConstPtr &msg)
{
  double siny = 2.0 * (msg->pose.pose.orientation.w * msg->pose.pose.orientation.z + msg->pose.pose.orientation.x * msg->pose.pose.orientation.y);
	double cosy = 1.0 - 2.0 * (msg->pose.pose.orientation.y * msg->pose.pose.orientation.y + msg->pose.pose.orientation.z * msg->pose.pose.orientation.z);  
	robot_pose_ = atan2(siny, cosy);
}

void ClusterObstacles::updatecommandVelocity(double linear, double angular)
{
  geometry_msgs::Twist cmd_vel;

  cmd_vel.linear.x  = linear;
  cmd_vel.angular.z = angular;
  cmd_vel_pub_.publish(cmd_vel);
}

/*******************************************************************************
* Finds Obstacle
*******************************************************************************/
bool ClusterObstacles::computeObstacles()
{
  turtlebot3_gazebo::Obstacle obstacle_info;

  // TODO
  obstacle_info.radius = { 1, 2, 3 };
  obstacle_info.obstacle_count = 3;
  ROS_INFO("Debug: %d %d %d", 1 , 2, 3);
  obstacle_info_pub_.publish(obstacle_info);
  return true;
}


/*******************************************************************************
* Main function
*******************************************************************************/
int main(int argc, char* argv[])
{
  ros::init(argc, argv, "cluster_obstacles");
  ClusterObstacles cluster_obstacles;
  ros::spin();
  return 0;
}
