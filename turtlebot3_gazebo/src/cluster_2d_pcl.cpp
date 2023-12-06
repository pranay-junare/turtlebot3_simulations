
#include "turtlebot3_gazebo/cluster_obstacles.h"
using namespace std;

#define PI 3.14159265
#define EMPTY_VALUE 2000000 
#define MIN_DISTANCE 0.6 // minimum valid distance of object
#define VALIDATE_CLUSTER 7 //minimum points required 

ros::Publisher pub;
const float THRESHOLD_DIST = 0.5; // min dist. between 2 points for valid new Cluster

ClusterObstacles::ClusterObstacles()
    : nh_priv_("~")
{
  // Init gazebo ros turtlebot3 node
  ROS_INFO("Cluster Distinct Obstacles Node Init");
  auto ret = init();
  ROS_ASSERT(ret);
}

ClusterObstacles::~ClusterObstacles()
{
  ros::shutdown();
}

/*******************************************************************************
 * Init function
 *******************************************************************************/
bool ClusterObstacles::init()
{
  std::string cmd_vel_topic_name = nh_.param<std::string>("cmd_vel_topic_name", "");

  // initialize variables
  escape_range_ = 30.0 * DEG_2RAD;
  check_forward_dist_ = 0.7;
  check_side_dist_ = 0.6;
  robot_pose_ = 0.0;

  // initialize publishers
  cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>(cmd_vel_topic_name, 10);
  obstacle_info_pub_ = nh_.advertise<turtlebot3_gazebo::Obstacle>("obstacle_info", 10);
  return true;
}

double findAngle(double M1, double M2)
{
  // Angle between 2 lines with known slope
  return atan(abs((M2 - M1) / (1 + M1 * M2))) * RAD_2DEG;
}

double findValidX1(int idx, vector<int> numOfPoints, vector<double> sign_distX)
{
  if (sign_distX[idx] == EMPTY_VALUE)
  {
    idx = idx + 1;
    findValidX1(idx, numOfPoints, sign_distX);
  }
  return sign_distX[idx];
}

double findValidY1(int idx, vector<int> numOfPoints, vector<double> sign_distY)
{
  if (sign_distY[idx] == EMPTY_VALUE)
  {
    idx = idx + 1;
    findValidY1(idx, numOfPoints, sign_distY);
  }
  return sign_distY[idx];
}

double findValidX2(int idx, vector<int> numOfPoints, vector<double> sign_distX, int j)
{
  if (sign_distX[idx + numOfPoints[j] - 1] == EMPTY_VALUE)
  {
    idx = idx - 1;
    findValidX2(idx, numOfPoints, sign_distX, j);
  }
 return sign_distX[idx + numOfPoints[j] - 1];
}

double findValidY2(int idx, vector<int> numOfPoints, vector<double> sign_distY, int j)
{

  if (sign_distY[idx + numOfPoints[j] - 1] == EMPTY_VALUE)
  {
    idx = idx - 1;
    findValidY2(idx, numOfPoints, sign_distY, j);
  }
  return sign_distY[idx + numOfPoints[j] - 1];
}

/*******************************************************************************
 * Finds distinct Obstacle count and its approximate length 
 *******************************************************************************/
void compute_obstacles_cb(const sensor_msgs::LaserScan::ConstPtr &laser_scan)
{
  int numOfClusters = 0;            // Number of Clusters Counter
  vector<int> numOfPoints;          // Number of Points for Each Cluster
  vector<long double> distPoints;   // Distace of points for each Cluster
  vector<double> sign_distX;
  vector<double> sign_distY;
  bool prev_inf;

  int length_range = laser_scan->ranges.size(); // initialised to 360
  float x_prev = 0.0, x_curr = 0.0;
  float y_prev = 0.0, y_curr = 0.0;
  float theta, distance;

  // Find new clusters
  for (int i = 0; i < length_range - 1; i++)
  {
    // If the point is a Finite value only then perform clustering
    if (isfinite(laser_scan->ranges[i]))
    {
      theta = laser_scan->angle_min + (i * laser_scan->angle_increment);

      x_curr = laser_scan->ranges[i] * cos(theta);
      y_curr = laser_scan->ranges[i] * sin(theta);

      if (prev_inf == false)
      {
        bool is_valid_data = false;
        distance = sqrt(pow((x_curr - x_prev), 2) + pow((y_curr - y_prev), 2));
        if (distance > THRESHOLD_DIST)
        {
          numOfClusters++;
          numOfPoints.push_back(1);
          if (laser_scan->ranges[i] > MIN_DISTANCE)
          {
            sign_distX.push_back(x_curr);
            sign_distY.push_back(y_curr);
            distPoints.push_back(laser_scan->ranges[i]);
            is_valid_data = true;
          }
        }
        else
        {
          numOfPoints[numOfClusters - 1]++;
          if (laser_scan->ranges[i] > MIN_DISTANCE)
          {
            sign_distX.push_back(x_curr);
            sign_distY.push_back(y_curr);
            distPoints.push_back(laser_scan->ranges[i]);
            is_valid_data = true;
          }
        }
        if (!is_valid_data)
        {
          sign_distX.push_back(EMPTY_VALUE);
          sign_distY.push_back(EMPTY_VALUE);
          distPoints.push_back(EMPTY_VALUE);
        }
      }
      else
      {
        numOfClusters++;
        numOfPoints.push_back(1);
        if (laser_scan->ranges[i] > MIN_DISTANCE)
        {
          sign_distX.push_back(x_curr);
          sign_distY.push_back(y_curr);
          distPoints.push_back(laser_scan->ranges[i]);
        }
        else
        {
          sign_distX.push_back(EMPTY_VALUE);
          sign_distY.push_back(EMPTY_VALUE);
          distPoints.push_back(EMPTY_VALUE);
        }
      }
      x_prev = x_curr;
      y_prev = y_curr;
      prev_inf = false;
    }
    else  // Else the point is infinity
      prev_inf = true;
  }

  ROS_INFO("No. of Clusters : %d", numOfClusters);
  int sum = 0;

  vector<double> angle;
  vector<double> size;
  int obstacle_count = 0;

  // From the found clusters compute angle and distance
  for (int j = 0; j < numOfPoints.size(); j++)
  {
    double x1 = findValidX1(sum, numOfPoints, sign_distX);
    double y1 = findValidY1(sum, numOfPoints, sign_distY);
    double x2 = findValidX2(sum, numOfPoints, sign_distX, j);
    double y2 = findValidY2(sum, numOfPoints, sign_distY, j);

    double m1 = y1 / x1;
    double m2 = y2 / x2;

    double sizeD = distance = sqrt(pow((x1 - x2), 2) + pow((y1 - y2), 2));
    double angleD = findAngle(m1, m2);

    if (numOfPoints[j] > VALIDATE_CLUSTER)
    {
      ROS_INFO("Cluster %d : %d points", j + 1, numOfPoints[j]);

      angle.push_back(angleD);
      size.push_back(sizeD);
      obstacle_count++;
    }
    sum = sum + numOfPoints[j];
  }

  turtlebot3_gazebo::Obstacle obstacle_info;
  obstacle_info.radius = size;
  obstacle_info.obstacle_count = obstacle_count;
  pub.publish(obstacle_info);
}

/*******************************************************************************
 * Main function
 *******************************************************************************/
int main(int argc, char *argv[])
{
  ros::init(argc, argv, "cluster_2d_pcl");
  ros::NodeHandle n;
 
  pub = n.advertise<turtlebot3_gazebo::Obstacle>("obstacle_info", 10);
  ros::Subscriber cluster_sub = n.subscribe("scan", 1, compute_obstacles_cb);
  ros::spin();
  return 0;
}
