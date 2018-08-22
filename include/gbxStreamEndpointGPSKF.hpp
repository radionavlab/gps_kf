#pragma once

#include "gbxstreamendpointin.h"
#include "gbxstream.h"
#include "typedefs.h"
#include "report.h"
#include <Eigen/Geometry>
#include <ros/ros.h>
#include <gbx_ros_bridge_msgs/Imu.h>
#include <gbx_ros_bridge_msgs/ImuConfig.h>
#include <gbx_ros_bridge_msgs/SingleBaselineRTK.h>
#include <gbx_ros_bridge_msgs/Attitude2D.h>
#include <gps_to_ros/RostimeToGps.h>
#include "navtoolbox.h"
#include <sys/time.h>
#include <iostream>
#include <csignal>
#include <iostream>
#include <thread>
#include <boost/program_options.hpp>

Eigen::Matrix3d ecef2enu_rotMatrix(Eigen::Vector3d &ECEF);
Eigen::Vector3d ecef2enu(Eigen::Vector3d &ECEF);

class GbxStreamEndpointGPSKF : public GbxStreamEndpoint
{
public:
    GbxStreamEndpointGPSKF();
    virtual ~GbxStreamEndpointGPSKF();
    void configure(ros::NodeHandle &nh);
    void donothing(); //compiler test
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

protected:
    virtual bool openSinkStream_() override;
    virtual void closeSinkStream_() override;
    virtual bool isValidSinkStream_() const;
    virtual bool isSinkEndpoint_() const { return true; }
    virtual bool isProcessEndpoint_() const { return true; }
    virtual bool writeBytes_(const u8* buffer, size_t size);

    virtual GbxStreamEndpoint::ProcessReportReturn processReport_(
            std::shared_ptr<const ReportCoda>&& pReport, const u8 streamId);
    virtual GbxStreamEndpoint::ProcessReportReturn processReport_(
            std::shared_ptr<const ReportAttitude2D>&& pReport, const u8 streamId);
    virtual GbxStreamEndpoint::ProcessReportReturn processReport_(
            std::shared_ptr<const ReportSingleBaselineRtk>&& pReport, const u8 streamId);
    virtual GbxStreamEndpoint::ProcessReportReturn processReport_(
            std::shared_ptr<const ReportImu>&& pReport, const u8 streamId);
    virtual GbxStreamEndpoint::ProcessReportReturn processReport_(
            std::shared_ptr<const ReportImuConfig>&& pReport, const u8 streamId);   
private:
    bool validRTKtest, validA2Dtest, hasAlreadyReceivedA2D, hasAlreadyReceivedRTK;
    int gpsWeek_, gpsSec_, internalSeq, sec_in_week;;
    double gpsFracSec_, dtRX_, minTestStat, lastRTKtime, lastA2Dtime;
    Eigen::Quaterniond internalQuat;
    Eigen::Vector3d internalPose, baseECEF_vector, L_cg2p;
    Eigen::Matrix3d RBI, Recef2enu;
    ros::Publisher ip_imu_, ip_imuC_, ip_sbrtk_, ip_a2d_, ip_timer_;

};

