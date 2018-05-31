#ifndef GPS_ODOM_NODE_HPP_
#define GPS_ODOM_NODE_HPP_

#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Joy.h>
#include <Eigen/Geometry>
#include <gbx_ros_bridge_msgs/SingleBaselineRTK.h>
#include <gbx_ros_bridge_msgs/Attitude2D.h>
#include "px4_control/updatePx4param.h" 
#include "filter.h"
#include <geometry_msgs/TransformStamped.h>
#include "filterTW.h"
#include "transformations.hpp"
#include <gps_kf/twUpdate.h>
#include <gps_kf/odomWithGpsTime.h>

namespace gps_odom
{
class gpsOdom
{
 public:
  gpsOdom(ros::NodeHandle &nh);

  void gpsCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
  //void gpsCallback(const geometry_msgs::TransformStamped::ConstPtr &msg);
  void singleBaselineRTKCallback(const gbx_ros_bridge_msgs::SingleBaselineRTK::ConstPtr &msg);
  void attitude2DCallback(const gbx_ros_bridge_msgs::Attitude2D::ConstPtr &msg);
  void throttleCallback(const std_msgs::Float64::ConstPtr &msg);
  void attSetCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
  void joyCallback(const sensor_msgs::Joy::ConstPtr &msg);
  void viconCallback(const geometry_msgs::TransformStamped::ConstPtr &msg);
  Eigen::Matrix3d rotMatFromEuler(Eigen::Vector3d ee);
  Eigen::Matrix3d rotMatFromQuat(Eigen::Quaterniond qq);
  void timerCallback(const ros::TimerEvent &event);

 private:
  void PublishTransform(const geometry_msgs::Pose &pose,
                        const std_msgs::Header &header,
                        const std::string &child_frame_id);

  gps_odom::KalmanFilter kf_;
  gps_odom::KalmanTW kfTW_;
  KalmanFilter::State_t xCurr;
  ros::Publisher odom_pub_;
  ros::Publisher localOdom_pub_;
  ros::Publisher mocap_pub_;
  ros::Publisher internalPosePub_; //publishes /Valkyrie/pose to itself
  ros::Publisher twPub_;
  ros::Publisher odomTimePub_;
  std::string child_frame_id_;
  std::string quadName;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  Eigen::Vector3d baseECEF_vector, baseENU_vector, WRW0_ecef, arenaRefCenter, internalPose, n_err, L_cg2p;
  Eigen::Matrix3d Recef2enu, RBI;
  bool publish_tf_;
  ros::Subscriber gps_sub_, rtkSub_, a2dSub_, joy_sub_, attSub_, thrustSub_;
  ros::Timer timerPub_;
  ros::Time lastRosTime;
  //geometry_msgs::PoseStamped::ConstPtr initPose_;
  geometry_msgs::PoseStamped initPose_;
  geometry_msgs::PoseStamped centerInENU;
  //geometry_msgs::PoseStamped initPose_;
  Eigen::Quaterniond internalQuat, quaternionSetpoint, internalQuatPrev;
  int centerFlag, internalSeq, sec_in_week;
  double lastRTKtime, lastA2Dtime, minTestStat, dt, max_accel, throttleSetpoint, throttleMax, quadMass, pubRate;
  bool validRTKtest, validA2Dtest, kfInit, hasAlreadyReceivedA2D, hasAlreadyReceivedRTK, isArmed, runTW;
  double pi;
  Eigen::Matrix<double,200,1> twStorage;
  int twCounter;
  ros::ServiceClient quadParamService; 
  Eigen::Matrix3d Rwrw, Rclass;
  int gpsWeek_, gpsSec_;
  double gpsFracSec_;
};

} // gps_odom

#endif // GPS_ODOM_NODE_HPP_
