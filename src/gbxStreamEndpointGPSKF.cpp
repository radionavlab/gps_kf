#include "gbxStreamEndpointGPSKF.hpp"


GbxStreamEndpointGPSKF::GbxStreamEndpointGPSKF()
{
    hasAlreadyReceivedA2D=false;
    hasAlreadyReceivedRTK=false;
    gpsSec_=0;
    gpsWeek_=0;
    gpsFracSec_=0;
    L_cg2p << 0.1013, -0.0004, 0.0472;
}

GbxStreamEndpointGPSKF::~GbxStreamEndpointGPSKF() {
  closeSinkStream_();
}

void GbxStreamEndpointGPSKF::configure(ros::NodeHandle &nh, Eigen::Vector3d baseECEF_vector_in,
            Eigen::Matrix3d Recef2enu_in)
{
    std::string GPSKFName, posePubTopic;
    GPSKFName = ros::this_node::getName();
    Recef2enu = Recef2enu_in;
    baseECEF_vector_in = baseECEF_vector;

    ros::param::get(GPSKFName + "/posePubTopic", posePubTopic);
    ros::param::get(GPSKFName + "/minimumTestStat",minTestStat);
    internalPosePub_ = nh.advertise<geometry_msgs::PoseStamped>(posePubTopic,1);
    internalSeq=0;
    sec_in_week = 604800;
    L_cg2p << 0.1013,-0.0004,0.0472;
}

void GbxStreamEndpointGPSKF::donothing()
{
    std::cout << "Do nothing called" << std::endl;
    return;
}

bool GbxStreamEndpointGPSKF::openSinkStream_() {
  return true;
}

void GbxStreamEndpointGPSKF::closeSinkStream_() {
  return;
}

bool GbxStreamEndpointGPSKF::isValidSinkStream_() const {
  return true;
}

bool GbxStreamEndpointGPSKF::writeBytes_(const u8* buffer, size_t size) {
  return true;
}

GbxStreamEndpoint::ProcessReportReturn GbxStreamEndpointGPSKF::processReport_(
    std::shared_ptr<const ReportCoda>&& pReport, const u8 streamId) {
  return ProcessReportReturn::REJECTED;
}


GbxStreamEndpoint::ProcessReportReturn GbxStreamEndpointGPSKF::processReport_(
    std::shared_ptr<const ReportAttitude2D>&& pReport, const u8 streamId)
{

    ProcessReportReturn retval = ProcessReportReturn::ACCEPTED;
    return retval;
}


GbxStreamEndpoint::ProcessReportReturn GbxStreamEndpointGPSKF::processReport_(
    std::shared_ptr<const ReportSingleBaselineRtk>&& pReport, const u8 streamId)
{

    ProcessReportReturn retval = ProcessReportReturn::ACCEPTED;
    return retval;
}


GbxStreamEndpoint::ProcessReportReturn GbxStreamEndpointGPSKF::processReport_(
    std::shared_ptr<const ReportIMU>&& pReport, const u8 streamId)
{
    ProcessReportReturn retval = ProcessReportReturn::ACCEPTED;
    return retval;    
}

GbxStreamEndpoint::ProcessReportReturn GbxStreamEndpointGPSKF::processReport_(
    std::shared_ptr<const ReportIMUConfig>&& pReport, const u8 streamId)
{

    ProcessReportReturn retval = ProcessReportReturn::ACCEPTED;
    return retval;  
}
