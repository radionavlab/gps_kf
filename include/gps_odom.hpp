#ifndef GPS_ODOM_NODE_HPP_
#define GPS_ODOM_NODE_HPP_

#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Geometry>
#include <ppfusion_msgs/Attitude2D.h>
#include <ppfusion_msgs/SingleBaselineRTK.h>

#include "filter.h"

namespace gps_odom
{
class gpsOdom
{
 public:
  gpsOdom(ros::NodeHandle &nh);

  void gpsCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
  void singleBaselineRTKCallback(const ppfusion_msgs::SingleBaselineRTK::ConstPtr &msg);
  void attitude2DCallback(const ppfusion_msgs::Attitude2D::ConstPtr &msg);

 private:
  void PublishTransform(const geometry_msgs::Pose &pose,
                        const std_msgs::Header &header,
                        const std::string &child_frame_id);

  gps_odom::KalmanFilter kf_;
  ros::Publisher odom_pub_;
  ros::Publisher localOdom_pub_;
  ros::Publisher mocap_pub_;
  ros::Publisher internalPosePub_; //publishes /Valkyrie/pose to itself
  std::string child_frame_id_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  Eigen::Vector3d baseECEF_vector, baseENU_vector, WRW0_ecef;
  Eigen::Matrix3d Recef2enu;
  bool publish_tf_;
  ros::Subscriber gps_sub_, rtkSub_, a2dSub_;
  //geometry_msgs::PoseStamped::ConstPtr initPose_;
  geometry_msgs::PoseStamped initPose_;
  geometry_msgs::PoseStamped centerInENU;
  //geometry_msgs::PoseStamped initPose_;
  Eigen::Vector3d arenaRefCenter, internalPose;
  Eigen::Quaterniond internalQuat;
  int centerFlag, internalSeq, sec_in_week;
  double lastRTKtime, lastA2Dtime, minTestStat, dt, max_accel;
  bool validRTKtest, validA2Dtest, kfInit, hasAlreadyReceivedA2D, hasAlreadyReceivedRTK;
  double pi;

};

} // gps_odom

#endif // GPS_ODOM_NODE_HPP_
