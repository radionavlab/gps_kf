#include <Eigen/Geometry>
#include "gps_odom.hpp"
#include <string>
#include <iostream>
//Contains the two "wordy" A2D and SBRTK callbacks.
//Also contains some rotation matrix construction

namespace gps_odom
{
void gpsOdom::singleBaselineRTKCallback(const gbx_ros_bridge_msgs::SingleBaselineRTK::ConstPtr &msg)
{
    //if(baseECEF_vector.squaredNorm() < 0.01)
    //{
    //}
    gpsWeek_=msg->tSolution.week;
    gpsSec_=msg->tSolution.secondsOfWeek;
    gpsFracSec_=msg->tSolution.fractionOfSecond;
    dtRX_ = msg->deltRSec;
    double ttime=msg->tSolution.secondsOfWeek + msg->tSolution.fractionOfSecond
        + msg->tSolution.week * sec_in_week -msg->deltRSec;
    ROS_INFO("SBR:%f",ttime);
    if(ttime>lastRTKtime)  //only use newest time
    {
        hasAlreadyReceivedRTK=true;
        lastRTKtime=ttime;
        if(msg->testStat > minTestStat)
        {
            validRTKtest=true;
            Eigen::Vector3d tmpvec;
            tmpvec(0) = msg->rxRov - baseECEF_vector(0); //error vector from ECEF at init time
            tmpvec(1) = msg->ryRov - baseECEF_vector(1);
            tmpvec(2) = msg->rzRov - baseECEF_vector(2);
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
            ROS_INFO("PUB:%f",ttime);
            hasAlreadyReceivedRTK=false; hasAlreadyReceivedA2D=false; //reset to avoid publishing twice
            //ROS_INFO("internalpose: %f %f %f",internalPose(0),internalPose(1),internalPose(2));
        }else{ lastRTKtime=msg->tSolution.secondsOfWeek+msg->tSolution.fractionOfSecond-msg->deltRSec +
                        msg->tSolution.week * sec_in_week;}
    }
}


void gpsOdom::attitude2DCallback(const gbx_ros_bridge_msgs::Attitude2D::ConstPtr &msg)
{
    double ttime=msg->tSolution.secondsOfWeek + msg->tSolution.fractionOfSecond + msg->tSolution.week * sec_in_week
          - msg->deltRSec;
    /*
    if(ttime < 0.10)  //if A2D starts publishing blank messages, publish 0 yaw
    {
        hasAlreadyReceivedA2D=true;
        if(hasAlreadyReceivedRTK && validRTKtest)
        {
            internalSeq++;
            geometry_msgs::PoseStamped selfmsg;
            selfmsg.header.seq=internalSeq;
            selfmsg.header.stamp=ros::Time(lastRTKtime);
            // subtraction is the apparent convention in /Valkyrie/pose
            selfmsg.header.frame_id="fcu";
            selfmsg.pose.position.x=internalPose(0);
            selfmsg.pose.position.y=internalPose(1);
            selfmsg.pose.position.z=internalPose(2);
            selfmsg.pose.orientation.x=0;
            selfmsg.pose.orientation.y=0;
            selfmsg.pose.orientation.z=0;
            selfmsg.pose.orientation.w=1;
            internalPosePub_.publish(selfmsg);
            //ROS_INFO("internalpose: %f %f %f",internalPose(0),internalPose(1),internalPose(2));
            hasAlreadyReceivedRTK=false; hasAlreadyReceivedA2D=false;            
        }
    }*/
    gpsWeek_=msg->tSolution.week;
    gpsSec_=msg->tSolution.secondsOfWeek;
    gpsFracSec_=msg->tSolution.fractionOfSecond;
    dtRX_ = msg->deltRSec;
    //if everything is working
    ROS_INFO("A2D:%f",ttime);
    if(ttime>lastA2Dtime)  //Only use newest time. Ignore 0 messages.
    {
        hasAlreadyReceivedA2D=true;
        lastA2Dtime=ttime;
        if(msg->testStat > minTestStat)
        {
            double thetaWRWLim;
            validA2Dtest=true;
            //attitude vec is Euler=[0,0, pi/2-azAngle (+-) thetaWRW]
            thetaWRWLim=pi/180*6.2;
            //thetaWRWLim=0;
            //Check against order convention in px4
            internalQuat = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX())
                * Eigen::AngleAxisd(-1.0*msg->elAngle, Eigen::Vector3d::UnitY())
                * Eigen::AngleAxisd(pi/2+thetaWRWLim-msg->azAngle, Eigen::Vector3d::UnitZ());
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
            RBI=rotMatFromQuat(internalQuat);


        }else if(!validA2Dtest)
        {
            //If the previous A2D was also bad, then play a sound and declare this one bad.
            validA2Dtest=false;
            std::cout << '\a';
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
            ROS_INFO("PUB:%f",ttime);
            hasAlreadyReceivedRTK=false; hasAlreadyReceivedA2D=false;
        }else if(abs(lastRTKtime-lastA2Dtime)<.001 && validRTKtest && hasAlreadyReceivedRTK
                && hasAlreadyReceivedA2D) /*if A2D stops publishing, go for pose anyways. May change
                this behavior later*/
        {
            internalSeq++;
            //Uncomment to publish message with 0 yaw
            /*geometry_msgs::PoseStamped selfmsg;
            selfmsg.header.seq=internalSeq;
            selfmsg.header.stamp=ros::Time(lastRTKtime);
            //subtraction is the apparent convention in /Valkyrie/pose
            selfmsg.header.frame_id="fcu";
            selfmsg.pose.position.x=internalPose(0);
            selfmsg.pose.position.y=internalPose(1);
            selfmsg.pose.position.z=internalPose(2);
            selfmsg.pose.orientation.x=0;
            selfmsg.pose.orientation.y=0;
            selfmsg.pose.orientation.z=0;
            selfmsg.pose.orientation.w=1;
            internalPosePub_.publish(selfmsg);
            //ROS_INFO("internalpose: %f %f %f",internalPose(0),internalPose(1),internalPose(2));
            */
            hasAlreadyReceivedRTK=false; hasAlreadyReceivedA2D=false;
        }

    }
}


Eigen::Matrix3d gpsOdom::rotMatFromEuler(Eigen::Vector3d ee)
{
  double phi=ee(0);
  double theta=ee(1);
  double psi=ee(2);
  Eigen::Matrix3d RR;
  RR<<cos(theta)*cos(psi), cos(theta)*sin(psi), -sin(theta),
      sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi), sin(theta)*sin(phi)*sin(psi)+cos(phi)*cos(psi), sin(phi)*cos(theta),
      cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi), cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi), cos(phi)*cos(theta);
  return RR;
}
Eigen::Matrix3d gpsOdom::rotMatFromQuat(Eigen::Quaterniond qq)
{
  double xx=qq.x();
  double yy=qq.y();
  double zz=qq.z();
  double ww=qq.w();
  Eigen::Matrix3d RR;
  /*RR << 1-2*yy*yy-2*zz*zz, 2*xx*yy+2*ww*zz, 2*xx*zz-2*ww*yy,
        2*xx*yy-2*ww*zz, 1-2*xx*xx-2*zz*zz, 2*yy*zz+2*ww*xx,
        2*xx*zz+2*ww*yy, 2*yy*zz-2*ww*xx, 1-2*xx*xx-2*yy*yy; // the transposed derp*/
  RR << 1-2*yy*yy-2*zz*zz, 2*xx*yy-2*ww*zz, 2*xx*zz+2*ww*yy,
        2*xx*yy+2*ww*zz, 1-2*xx*xx-2*zz*zz, 2*yy*zz-2*ww*xx,
        2*xx*zz-2*ww*yy, 2*yy*zz+2*ww*xx, 1-2*xx*xx-2*yy*yy;
  return RR;
}


} //end namespace
