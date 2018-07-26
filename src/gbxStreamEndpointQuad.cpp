#include "gbxstreamendpointquad.h"
#include "navtoolbox.h"
#include <sys/time.h>

GbxStreamEndpointQuad::GbxStreamEndpointQuad() {}

GbxStreamEndpointQuad::~GbxStreamEndpointQuad() {
  closeSinkStream_();
}

void GbxStreamEndpointQuad::configure(ros::NodeHandle &nh, Eigen::Vector3d baseECEF_vector_in,
            Eigen::Matrix3d Recef2enu_in)
{
    quadName = ros::this_node::getName();
    std::string posePubTopic;
    Recef2enu = Recef2enu_in;
    baseECEF_vector_in = baseECEF_vector;

    ros::param::get(quadName + "/posePubTopic", posePubTopic);
    ros::param::get(quadName + "/minimumTestStat",minTestStat);
    internalPosePub_ = nh.advertise<geometry_msgs::PoseStamped>(posePubTopic,1);
    internalSeq=0;
    sec_in_week = 604800;
    L_cg2p << 0.1013,-0.0004,0.0472;
}

bool GbxStreamEndpointQuad::openSinkStream_() {
  return true;
}

void GbxStreamEndpointQuad::closeSinkStream_() {
  return;
}

bool GbxStreamEndpointQuad::isValidSinkStream_() const {
  return true;
}

bool GbxStreamEndpointQuad::writeBytes_(const u8* buffer, size_t size) {
  return true;
}

GbxStreamEndpoint::ProcessReportReturn GbxStreamEndpointQuad::processReport_(
    std::shared_ptr<const ReportCoda>&& pReport, const u8 streamId) {
  return ProcessReportReturn::REJECTED;
}


GbxStreamEndpoint::ProcessReportReturn gpsOdom::processReport_(
    std::shared_ptr<const ReportMultiBaselineRtkAttitude2D>&& pReport, const u8 streamId)
{
    dtRX_=pReport->deltRSec();
    pReport->tSolution.get(gpsWeek_, gpsSec_, gpsFracSec_);
    double ttime = gpsSec_ + gpsFracSec_ + gpsWeek_*sec_in_week - dtRX_;

    //if everything is working
    if(ttime>lastA2Dtime)  //Only use newest time. Ignore 0 messages.
    {
        hasAlreadyReceivedA2D=true;
        lastA2Dtime=ttime;
        if(pReport->testStat() > minTestStat)
        {
            double thetaWRWLim;
            validA2Dtest=true;
            //attitude vec is Euler=[0,0, pi/2-azAngle (+-) thetaWRW]
            thetaWRWLim=pi/180*6.2;
            //thetaWRWLim=0;
            //Check against order convention in px4
            internalQuat = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX())
                * Eigen::AngleAxisd(-1.0*pReport->elAngle(), Eigen::Vector3d::UnitY())
                * Eigen::AngleAxisd(pi/2+thetaWRWLim-pReport->azAngle(), Eigen::Vector3d::UnitZ());
//            internalQuat = Eigen::AngleAxisd(pi/2+thetaWRWLim-msg->azAngle, Eigen::Vector3d::UnitZ())
//                * Eigen::AngleAxisd(-1.0*msg->elAngle, Eigen::Vector3d::UnitY());

            //Check for sign flops in quaternion
            if(internalQuat.z()*internalQuatPrev.z()<0 && internalQuat.w()*internalQuatPrev.w()<0)
            {
                internalQuat.x()=-1*internalQuat.x();
                internalQuat.y()=-1*internalQuat.y();
                internalQuat.z()=-1*internalQuat.z();
                internalQuat.w()=-1*internalQuat.w();
            }
            internalQuatPrev=internalQuat;
            RBI=internalQuat.normalized().toRotationMatrix();


        }else if(!validA2Dtest)
        {
            //If the previous A2D was also bad, then play a sound and declare this one bad.
            validA2Dtest=false;
        }else
        {
            validA2Dtest=false;
        }

        //If the message is new, both messages have been received, and all teststats are good, then publish.
        if(abs(lastRTKtime-lastA2Dtime)<.001 && validA2Dtest && validRTKtest
                && hasAlreadyReceivedRTK && hasAlreadyReceivedA2D)  //only resend pose if new
        {
            internalSeq++;
            geometry_msgs::PoseStamped selfmsg;
            selfmsg.header.seq=internalSeq;
            selfmsg.header.stamp=ros::Time(lastRTKtime);
            //Subtraction is the apparent convention in /Valkyrie/pose
            selfmsg.header.frame_id="wrw0";
            //selfmsg.header.frame_id="fcu";
            
            //Convert primary to CG
            internalPose = internalPose - RBI*L_cg2p;

            selfmsg.pose.position.x=internalPose(0);
            selfmsg.pose.position.y=internalPose(1);
            selfmsg.pose.position.z=internalPose(2);
            selfmsg.pose.orientation.x=internalQuat.x();
            selfmsg.pose.orientation.y=internalQuat.y();
            selfmsg.pose.orientation.z=internalQuat.z();
            selfmsg.pose.orientation.w=internalQuat.w();
            internalPosePub_.publish(selfmsg);
            //ROS_INFO("internalpose: %f %f %f",internalPose(0),internalPose(1),internalPose(2));
            //Reset for next message
            hasAlreadyReceivedRTK=false; hasAlreadyReceivedA2D=false;
        }else if(abs(lastRTKtime-lastA2Dtime)<.001 && validRTKtest && hasAlreadyReceivedRTK
                && hasAlreadyReceivedA2D) /*if A2D stops publishing, go for pose anyways. May change
                this behavior later*/
        {
            internalSeq++;
            hasAlreadyReceivedRTK=false; hasAlreadyReceivedA2D=false;
        }
    }
    
    retval = ProcessReportReturn::ACCEPTED;
    return retval;
}


GbxStreamEndpoint::ProcessReportReturn gpsOdom::processReport_(
    std::shared_ptr<const ReportSingleBaselineRtk>&& pReport, const u8 streamId)
{
    dtRX_=pReport->deltRSec();
    pReport->tSolution.get(gpsWeek_, gpsSec_, gpsFracSec_);
    double ttime = gpsSec_ + gpsFracSec_ + gpsWeek_*sec_in_week - dtRX_;

    if(ttime>lastRTKtime)  //only use newest time
    {
        hasAlreadyReceivedRTK=true;
        lastRTKtime=ttime;
        if(pReport->testStat() > minTestStat)
        {
            validRTKtest=true;
            Eigen::Vector3d tmpvec;
            tmpvec(0) = pReport->rx() - baseECEF_vector(0); //error vector from ECEF at init time
            tmpvec(1) = pReport->ry() - baseECEF_vector(1);
            tmpvec(2) = pReport->rz() - baseECEF_vector(2);
            internalPose = Recef2enu*tmpvec - n_err; //rescaling

            //std::cout<<"rI from SBRTK"<<std::endl<<internalPose<<std::endl;
            //ROS_INFO("%f %f %f %f",msg->rx, msg->rxRov, tmpvec(0), internalPose(0)); //debugging
        }else{validRTKtest=false;}

        //If the time is new, both messages for this time have been received, and teststats are good
        if(abs(lastRTKtime-lastA2Dtime)<.001 && validA2Dtest && validRTKtest
                && hasAlreadyReceivedRTK && hasAlreadyReceivedA2D)  //only resend pose if new
        {
            internalSeq++;
            geometry_msgs::PoseStamped selfmsg;
            selfmsg.header.seq=internalSeq;
            selfmsg.header.stamp=ros::Time(lastRTKtime);
            //Subtraction is the apparent convention in /Valkyrie/pose
            selfmsg.header.frame_id="wrw0";
            //selfmsg.header.frame_id="fcu";
            
            //Convert primary location to CG
            internalPose = internalPose - RBI*L_cg2p;

            selfmsg.pose.position.x=internalPose(0);
            selfmsg.pose.position.y=internalPose(1);
            selfmsg.pose.position.z=internalPose(2);
            selfmsg.pose.orientation.x=internalQuat.x();
            selfmsg.pose.orientation.y=internalQuat.y();
            selfmsg.pose.orientation.z=internalQuat.z();
            selfmsg.pose.orientation.w=internalQuat.w(); //Quaternion(0) is w
            internalPosePub_.publish(selfmsg);
            hasAlreadyReceivedRTK=false; hasAlreadyReceivedA2D=false; //reset to avoid publishing twice
            //ROS_INFO("internalpose: %f %f %f",internalPose(0),internalPose(1),internalPose(2));
        }else{ lastRTKtime=ttime;}
    }
    retval = ProcessReportReturn::ACCEPTED;
    return retval;
}

