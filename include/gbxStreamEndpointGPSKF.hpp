#pragma once
#pragma message("IS READING GBXSTREAMENDPOINTGPSKF")
#include "gbxstreamendpoint.h"
#include "typedefs.h"
#include "report.h"
#include <Eigen/Geometry>
#include <ros/ros.h>

class GbxStreamEndpointGPSKF : public GbxStreamEndpoint
{
public:
    GbxStreamEndpointGPSKF();
    virtual ~GbxStreamEndpointGPSKF();
    void configure(ros::NodeHandle &nh, Eigen::Vector3d baseECEF_vector_in,
            Eigen::Matrix3d Recef2enu_in);
    void donothing();

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
private:
    bool validRTKtest, validA2Dtest, hasAlreadyReceivedA2D, hasAlreadyReceivedRTK;
    int gpsWeek_, gpsSec_, internalSeq, sec_in_week;;
    double gpsFracSec_, dtRX_, minTestStat, lastRTKtime, lastA2Dtime;
    Eigen::Quaterniond internalQuat;
    Eigen::Vector3d internalPose, baseECEF_vector;
    Eigen::Matrix3d RBI, Recef2enu;

};

