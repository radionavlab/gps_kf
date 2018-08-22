#include "gbxStreamEndpointGPSKF.hpp"

Eigen::Matrix3d ecef2enu_rotMatrix(Eigen::Vector3d &ECEF){


    //Define WGS-84 Earth parameters
    const double aa = 6378137.00000;
    const double bb = 6356752.3142518;
    const double ee = 0.0818191908334158;
    const double ep = sqrt((aa*aa - bb*bb)/(bb*bb));
    const double ee2 = (aa*aa-bb*bb)/(aa*aa);

    //Convert to (phi,lambda,h) geodetic coordinates
    double x = ECEF(0);
    double y = ECEF(1);
    double z = ECEF(2);
    double lambda = atan2(y, x);
    double p = sqrt(x*x + y*y);
    double theta = atan2(z*aa, p*bb);
    double phi=atan2(z,(1-ee2)*p);
    double N,h, phiprev;
    bool contvar=true;
    while(contvar)
    {
        phiprev=phi;
        N=aa/sqrt(1-ee2*sin(phi)*sin(phi));
        h=p/cos(phi)-N;
        phi=atan2(z,(1-ee2*N/(N+h))*p);
        if(abs(phiprev-phi)<1e-6)
        {
            contvar=false;
        }
    }

    //Form the rotation matrix
    Eigen::Matrix3d Renu_ecef = Eigen::Matrix3d::Zero();
    Renu_ecef(0,0) = -sin(lambda);
    Renu_ecef(0,1) = cos(lambda);
    Renu_ecef(0,2) = 0;
    Renu_ecef(1,0) = -sin(phi)*cos(lambda);
    Renu_ecef(1,1) = -sin(phi)*sin(lambda);
    Renu_ecef(1,2) = cos(phi);
    Renu_ecef(2,0) = cos(phi)*cos(lambda);
    Renu_ecef(2,1) = cos(phi)*sin(lambda);
    Renu_ecef(2,2) = sin(phi);

 return Renu_ecef;
}


Eigen::Vector3d ecef2enu(Eigen::Vector3d &ECEF){
    Eigen::Matrix3d R = ecef2enu_rotMatrix(ECEF);
    Eigen::Vector3d ENU = R*ECEF;
    return ENU;
}


GbxStreamEndpointGPSKF::GbxStreamEndpointGPSKF()
{

}

GbxStreamEndpointGPSKF::~GbxStreamEndpointGPSKF() {
  closeSinkStream_();
}

void GbxStreamEndpointGPSKF::configure(ros::NodeHandle &nh)
{
    //NOTE: Will need to merge with timerNode
    
    std::string GPSKFName, posePubTopic;
    GPSKFName = ros::this_node::getName();
    Recef2enu = Recef2enu_in;
    baseECEF_vector_in = baseECEF_vector;

    ros::param::get(GPSKFName + "/posePubTopic", posePubTopic);
    ros::param::get(GPSKFName + "/minimumTestStat",minTestStat);
    ip_imu_ = nh.advertise<gbx_ros_bridge_msgs::Imu>("Imu",1);
    ip_imuC_ = nh.advertise<gbx_ros_bridge_msgs::ImuConfig>("ImuConfig",1);
    ip_sbrtk_ = nh.advertise<gbx_ros_bridge_msgs::SingleBaselineRTK>("SingleBaselineRTK",1);
    ip_a2d_ = nh.advertise<gbx_ros_bridge_msgs::Attitude2D>("Attitude2D",1);
    ip_timer_ = nh.advertise<gps_to_ros::RostimeToGps>("RostimeToGps",1);
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
    gbx_ros_bridge_msgs::Attitude2D a2dmsg;
    gps_to_ros::RostimeToGps timemsg;
    a2dmsg.rx = pReport->rx();
    a2dmsg.ry = pReport->ry();
    a2dmsg.rz = pReport->rz();
    a2dmsg.rxRov = pReport->rxRov();
    a2dmsg.ryRov = pReport->ryRov();
    a2dmsg.rzRov = pReport->rzRov();
    pReport->tSolution.get(a2dmsg.tSolution.week,
                           a2dmsg.tSolution.secondsOfWeek,
                           a2dmsg.tSolution.fractionOfSecond);
    a2dmsg.deltRSec = pReport->deltRSec();
    a2dmsg.P.resize(pReport->lCov);
    for(int i=0;i<pReport->lCov;++i)
    {   a2dmsg.P[i] = pReport->P(i); }
    a2dmsg.nCov = pReport->nCov;
    a2dmsg.azAngle = pReport->azAngle();
    a2dmsg.elAngle = pReport->elAngle();
    a2dmsg.azSigma = pReport->azSigma();
    a2dmsg.elSigma = pReport->elSigma();
    a2dmsg.testStat = pReport->testStat();
    a2dmsg.numDD = pReport->numDD();
    a2dmsg.bitfield = pReport->bitfield();

    ip_a2d_.publish(a2dmsg);

    timemsg.header.stamp=ros::Time::now();
    timemsg.tSolution=a2dmsg.tSolution;
    timemsg.deltRSec=a2dmsg.deltRSec;

    ip_timer_.publish(timemsg);

    ProcessReportReturn retval = ProcessReportReturn::ACCEPTED;
    return retval;
}


GbxStreamEndpoint::ProcessReportReturn GbxStreamEndpointGPSKF::processReport_(
    std::shared_ptr<const ReportSingleBaselineRtk>&& pReport, const u8 streamId)
{
    gbx_ros_bridge_msgs::SingleBaselineRTK sbrtkmsg;
    sbrtkmsg.rx = pReport->rx();
    sbrtkmsg.ry = pReport->ry();
    sbrtkmsg.rz = pReport->rz();
    sbrtkmsg.rxRov = pReport->rxRov();
    sbrtkmsg.ryRov = pReport->ryRov();
    sbrtkmsg.rzRov = pReport->rzRov();
    sbrtkmsg.nCov = pReport->nCov;
    sbrtkmsg.P.resize(pReport->lCov);
    for(int i=0;i<pReport->lCov;++i)
    {   sbrtkmsg.P[i] = pReport->P(i); }
    pReport->tSolution.get(sbrtkmsg.tSolution.week,
                           sbrtkmsg.tSolution.secondsOfWeek,
                           sbrtkmsg.tSolution.fractionOfSecond);
    sbrtkmsg.deltRSec = pReport->deltRSec();
    sbrtkmsg.testStat = pReport->testStat();
    sbrtkmsg.ageOfReferenceData = pReport->ageOfReferenceData();
    sbrtkmsg.numDD = pReport->numDD();
    sbrtkmsg.bitfield = pReport->bitfield();

    ip_sbrtk_.publish(sbrtkmsg);

    ProcessReportReturn retval = ProcessReportReturn::ACCEPTED;
    return retval;
}


GbxStreamEndpoint::ProcessReportReturn GbxStreamEndpointGPSKF::processReport_(
    std::shared_ptr<const ReportIMU>&& pReport, const u8 streamId)
{
    gbx_ros_bridge_msgs::Imu imumsg;
    imumsg.tIndexTrunc = pReport->tIndexTruncated();
    const s16* const accel{ pReport->acceleration() };
    imumsg.acceleration[0] = accel[0];
    imumsg.acceleration[1] = accel[1];
    imumsg.acceleration[2] = accel[2];
    const s16* const angRate{ pReport->angularRate() };
    imumsg.angularRate[0] = angRate[0];
    imumsg.angularRate[1] = angRate[1];
    imumsg.angularRate[2] = angRate[2];
    imumsg.temperature = pReport->temperature();


    ip_imu_.publish(imumsg);
 
    ProcessReportReturn retval = ProcessReportReturn::ACCEPTED;
    return retval;    
}

GbxStreamEndpoint::ProcessReportReturn GbxStreamEndpointGPSKF::processReport_(
    std::shared_ptr<const ReportIMUConfig>&& pReport, const u8 streamId)
{
    gbx_ros_bridge_msgs::ImuConfig imuCmsg;
    imuCmsg.tIndexk = pReport->tIndexk();
    imuCmsg.rangeMetersPerSecSq = pReport->rangeMetersPerSecSq();
    imuCmsg.rangeRadPerSec = pReport->rangeRadPerSec();
    imuCmsg.lsbToMetersPerSecSq = pReport->lsbToMetersPerSecSq();
    imuCmsg.lsbToRadPerSec = pReport->lsbToRadPerSec();
    imuCmsg.lsbToDegK = pReport->lsbToDegK();
    imuCmsg.temperatureCenterDegC = pReport->temperatureCenterDegC();
    imuCmsg.accelerationBandwidthHz = pReport->accelerationBandwidthHz();
    imuCmsg.angularRateBandwidthHz = pReport->angularRateBandwidthHz();
    imuCmsg.measurementRateHz = pReport->measurementRateHz();
    imuCmsg.sampleFreqNumerator = pReport->sampleFreqNumerator();
    imuCmsg.sampleFreqDenominator = pReport->sampleFreqDenominator();

    ip_imuC_.publish(imuCmsg);
 
    ProcessReportReturn retval = ProcessReportReturn::ACCEPTED;
    return retval;   
}
