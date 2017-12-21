#include <Eigen/Geometry>
#include "gps_odom.hpp"
#include <string>
#include <iostream>

Eigen::Matrix3d ecef2enu_rotMatrix(Eigen::Vector3d ECEF){


    //----- Define WGS-84 Earth parameters
    const double aa = 6378137.00000;
    const double bb = 6356752.3142518;
    const double ee = 0.0818191908334158;
    const double ep = sqrt((aa*aa - bb*bb)/(bb*bb));
    const double ee2 = (aa*aa-bb*bb)/(aa*aa);

    //----- Convert to (phi,lambda,h) geodetic coordinates
    double x = ECEF(0);
    double y = ECEF(1);
    double z = ECEF(2);
    double lambda = atan2(y, x);
    double p = sqrt(x*x + y*y);
    double theta = atan2(z*aa, p*bb);
    /*double phi = atan2(z + ep*ep*bb*pow(sin(theta),3),
                    p - ee*ee*aa*pow(cos(theta),3));*/
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

    //----- Form the rotation matrix
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

Eigen::Vector3d ecef2enu(Eigen::Vector3d ECEF){
    Eigen::Matrix3d R = ecef2enu_rotMatrix(ECEF);
    Eigen::Vector3d ENU = R*ECEF;
    /*ROS_INFO("R row 1: %f %f %f", R(0,0), R(0,1), R(0,2));
    ROS_INFO("handmat: %f %f %f", R(0,0)*ECEF(0),  R(0,1)*ECEF(1),  R(0,2)*ECEF(2));
    ROS_INFO("ENUy: %f  ECEFy: %f", ENU(1), ECEF(1)); */
    return ENU;  /*tfw someone else forgot the return and I thought it was my code in error.
            I TRUSTED YOU. YOU HAVE BETRAYED ME. I REALLY SHOULD GET SOME SLEEP.*/
}

namespace gps_odom
{
gpsOdom::gpsOdom(ros::NodeHandle &nh)
{

  //Get data about node and topic to listen
  std::string quadPoseTopic, quadName, rtktopic, a2dtopic, posePubTopic;
  double tmax;
  quadName = ros::this_node::getName();
//  Eigen::Vector3d enuInput;
  ros::param::get(quadName + "/quadPoseTopic", quadPoseTopic);
  ros::param::get(quadName + "/arenaCenterX", baseECEF_vector(0));
  ros::param::get(quadName + "/arenaCenterY", baseECEF_vector(1));
  ros::param::get(quadName + "/arenaCenterZ", baseECEF_vector(2));
  ros::param::get(quadName + "/arenaCenterX_ENU", n_err(0));
  ros::param::get(quadName + "/arenaCenterY_ENU", n_err(1));
  ros::param::get(quadName + "/arenaCenterZ_ENU", n_err(2));
  ros::param::get(quadName + "/rtktopic", rtktopic);
  ros::param::get(quadName + "/a2dtopic", a2dtopic);
  ros::param::get(quadName + "/posePubTopic", posePubTopic);
  ros::param::get(quadName + "/minimumTestStat",minTestStat);
  ros::param::get(quadName + "/maxTW",tmax);
  ros::param::get(quadName + "/mass",quadMass);
  throttleMax = tmax*9.81;

  twCounter=0;

  //Get additional parameters for the kalkman filter
  nh.param(quadName + "/max_accel", max_accel, 2.0);
  nh.param(quadName + "/publish_tf", publish_tf_, true);
  nh.param<std::string>(quadName + "/child_frame_id", child_frame_id_, "base_link");
  if(publish_tf_ && child_frame_id_.empty())
    throw std::runtime_error("gps_odom: child_frame_id required for publishing tf");

  // There should only be one gps_fps, so we read from nh
  double gps_fps;
  nh.param(quadName + "/gps_fps", gps_fps, 20.0);
  ROS_ASSERT(gps_fps > 0.0);
  dt = 1.0 / gps_fps;

  //should be a const but catkin doesn't like scoping it
  pi = std::atan(1.0)*4;

         /*baseECEF_vector(0) = msg->rx+msg->rxRov; //NOTE: THIS SHOULD BE READ IN VIA .LAUNCH WHEN USING GLOBAL FRAME
        baseECEF_vector(1) = msg->ry+msg->ryRov;
        baseECEF_vector(2) = msg->rz+msg->rzRov;*/
  Recef2enu=ecef2enu_rotMatrix(baseECEF_vector);
  //std::cout << Recef2enu << std::endl;
  //baseENU_vector=Recef2enu*baseECEF_vector;

  lastRTKtime=0;
  lastA2Dtime=0;
  //internalQuat.resize(4);
  internalSeq=0;
  sec_in_week = 604800;
  isArmed=false;
  kfInit=false; //KF will need to be initialized
  throttleSetpoint = 9.81/throttleMax; //the floor is the throttle
  quaternionSetpoint.x()=0; quaternionSetpoint.y()=0; quaternionSetpoint.z()=0; quaternionSetpoint.w()=1;

  //verbose parameters
  ROS_INFO("max_accel: %f", max_accel);
  ROS_INFO("publish_tf_: %d", publish_tf_);
  ROS_INFO("child_frame_id: %s", child_frame_id_.c_str());
  ROS_INFO("ROS topic: %s", quadPoseTopic.c_str());
  ROS_INFO("Node name: %s", quadName.c_str());
  ROS_INFO("gps_fps: %f", gps_fps);

  // Initialize publishers and subscribers
  odom_pub_ = nh.advertise<nav_msgs::Odometry>("odom", 10); //MUST have a node namespace, ns="quadName", in launchfile
  localOdom_pub_ = nh.advertise<nav_msgs::Odometry>("local_odom", 10);
  mocap_pub_ = nh.advertise<geometry_msgs::PoseStamped>("mavros/mocap/pose", 10);
  gps_sub_ = nh.subscribe(quadPoseTopic, 10, &gpsOdom::gpsCallback,
                            this, ros::TransportHints().tcpNoDelay());
  internalPosePub_ = nh.advertise<geometry_msgs::PoseStamped>(posePubTopic,10);
  rtkSub_ = nh.subscribe("SingleBaselineRTK",10,&gpsOdom::singleBaselineRTKCallback,
                            this, ros::TransportHints().tcpNoDelay());
  a2dSub_ = nh.subscribe("Attitude2D",10,&gpsOdom::attitude2DCallback,
                            this, ros::TransportHints().tcpNoDelay());
  thrustSub_ = nh.subscribe("mavros/setpoint_attitude/att_throttle", 10,
                            &gpsOdom::throttleCallback,this, ros::TransportHints().tcpNoDelay());
  attSub_ = nh.subscribe("mavros/setpoint_attitude/attitude", 10,
                            &gpsOdom::attSetCallback,this, ros::TransportHints().tcpNoDelay());
  joy_sub_ = nh.subscribe("joy",10,&gpsOdom::joyCallback, this, ros::TransportHints().tcpNoDelay()); 
  quadParamService = nh.serviceClient<px4_control::updatePx4param>("px4_control_node/updateQuadParam");
  ROS_INFO("Kalman Filter Node started! Listening to ROS input topic: %s within specified namespace", (quadPoseTopic).c_str());

  //Get initial pose
  //initPose_ = ros::topic::waitForMessage<geometry_msgs::PoseStamped>(quadPoseTopic);
  //geometry_msgs::PoseStamped initPose_;

}


void gpsOdom::throttleCallback(const std_msgs::Float64::ConstPtr &msg)
{
  //if on the ground / else if taken off
  if(xCurr(2)<0.05)
  {
    throttleSetpoint = 9.81/throttleMax; //the floor is the throttle
  }else{
    throttleSetpoint = throttleMax * msg->data;
  }
}
void gpsOdom::attSetCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
  if(xCurr(2)<0.05)
  {
    quaternionSetpoint.x() = 0;
    quaternionSetpoint.y() = 0;
    quaternionSetpoint.z() = 0;
    quaternionSetpoint.w() = 1;
  }else{
    quaternionSetpoint.x() = msg->pose.orientation.x;
    quaternionSetpoint.y() = msg->pose.orientation.y;
    quaternionSetpoint.z() = msg->pose.orientation.z;
    quaternionSetpoint.w() = msg->pose.orientation.w;
  }
}


void gpsOdom::gpsCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
  Eigen::Vector3d zMeas;
  if(kfInit)  //only filter if initPose_ has been set
  {   
      //ROS_INFO("Callback running!");
      static ros::Time t_last_proc = msg->header.stamp;
      //static int counter=0;
      static ros::Time time_of_last_fix=msg->header.stamp;
      double lastKnownSpeed=0;
      //counter++;
      double dt = (msg->header.stamp - t_last_proc).toSec();
      t_last_proc = msg->header.stamp;
      static Eigen::Vector3d z_last(0.0, 0.0, 0.0);
      Eigen::Vector3d uvec=rotMatFromQuat(quaternionSetpoint)*Eigen::Vector3d(0.0,0.0,throttleSetpoint);

      //timing
      static ros::Time t_last_meas = msg->header.stamp; //initialize static variable (basically a scope-limited global)
      double meas_dt = (msg->header.stamp - t_last_meas).toSec();
      t_last_meas = msg->header.stamp; //update static variable after being used

      // Kalman filter
      kf_.processUpdate(dt);

      //extract measurement and calculate residuals to hypothesis test for kf_
      Eigen::Vector3d ECEF;
      ECEF(0) = msg->pose.position.x;
      ECEF(1) = msg->pose.position.y;
      ECEF(2) = msg->pose.position.z;
      const KalmanFilter::Measurement_t meas(msg->pose.position.x, msg->pose.position.y,
                                             msg->pose.position.z);
      Eigen::Matrix<double,6,1> xbar=kf_.getState();
      Eigen::Vector3d z_expected;  //propagating distance forwards based on estimated speed
      z_expected(0)=z_last(0)+xbar(3)*dt;
      z_expected(1)=z_last(1)+xbar(4)*dt;
      z_expected(2)=z_last(2)+xbar(5)*dt;
      double dt_last_fix=(msg->header.stamp-time_of_last_fix).toSec();
    //  double estDistBySpeed=sqrt(xbar(3)*xbar(3)+xbar(4)*xbar(4)+xbar(5)*xbar(5))*dt_meas;
      double accel_max=2.0;
      double dz_hyp_test = sqrt( pow(z_last(0)-meas(0),2)+pow(z_last(1)-meas(1),2)+pow(z_last(2)-meas(2),2) );
      //maximum expected speed. Normalized by speed rather than pose to handle lost measurement packets
      double hypothesis_test_threshold=(0.5+accel_max*dt_last_fix*dt_last_fix/2+lastKnownSpeed*dt_last_fix)*meas_dt/0.05;
      //ROS_INFO("dt:%f:  dz1: %f:  dz2: %f:  dz3: %f",meas_dt,z_last(0)-meas(0),z_last(1)-meas(1),z_last(2)-meas(2));
      Eigen::Matrix<double, 3, 6> HH;
      HH.setZero();
      HH(0, 0) = 1;
      HH(1, 1) = 1;
      HH(2, 2) = 1;
      Eigen::Vector3d resid = meas - HH*xbar;
      Eigen::Matrix<double,3,3> meas_covmat; //placeholder until I can get the full covariance solution
      meas_covmat.setZero();
      meas_covmat(0,0) = 1;
      meas_covmat(1,1) = 1;
      meas_covmat(2,2) = 1;
      Eigen::Matrix<double,6,6> proc_noise_cov;
      proc_noise_cov = kf_.getProcessNoise();
      //replace residCost with dz_hyp_test when full covariance solution is available
      double residCost=resid.transpose()*(meas_covmat+HH*proc_noise_cov*HH.transpose())*resid; //unused until full covariance is available

      if (dz_hyp_test <= hypothesis_test_threshold)
      {
        kf_.measurementUpdate(meas, meas_dt);
        time_of_last_fix=msg->header.stamp;
        lastKnownSpeed=sqrt(xbar(3)*xbar(3)+xbar(4)*xbar(4)+xbar(5)*xbar(5));
        if (dz_hyp_test>=hypothesis_test_threshold*2/3)
        {
          ROS_INFO("Possible outlier warning at dz=%f", dz_hyp_test);
        }
      }
      else if (msg->header.seq>1) //bypass first step
      {
        ROS_INFO("Outlier of value %f rejected at seq %d", dz_hyp_test,msg->header.seq);
      }
      /*kf_.measurementUpdate(meas, meas_dt);
      time_of_last_fix=msg->header.stamp;*/

      //update static variable AFTER using it
      z_last(0)=msg->pose.position.x;
      z_last(1)=msg->pose.position.y;
      z_last(2)=msg->pose.position.z;

      const KalmanFilter::State_t state = kf_.getState();
      const KalmanFilter::ProcessCov_t proc_noise = kf_.getProcessNoise();
      xCurr = kf_.getState();

      //T/W filter
      if(state(2)>=0.10 && isArmed)
      {
        kfTW_.processUpdate(dt,uvec);
        Eigen::Matrix<double,7,1> xStateAfterProp=kfTW_.getState();
        //ROS_INFO("T/W after propagation: %f",xStateAfterProp(6));
        //update kfTW_
        const KalmanTW::Measurement_t measM(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
        kfTW_.measurementUpdate(measM,meas_dt);
        Eigen::Matrix<double,7,1> xTWstate=kfTW_.getState();
        //std::cout<<xTWstate(6)<<std::endl;

        twCounter++;
        twCounter=twCounter%200;
        twStorage(twCounter)=xTWstate(6)*throttleMax/9.81;  //throttleMax=9.81*tw[0]
        if(twCounter==0)
        {
          double meanTW=twStorage.sum()/200.0;
          double battstat;
          battstat = 10.0+90.0/(1.75-1.40)*(meanTW-1.40); //approx battery percent
          ROS_INFO("Updating T/W to %f. Battery at approximately %f%%",meanTW,battstat);
              //ROS_INFO("service called");
          px4_control::updatePx4param param_srv;
          param_srv.request.data.resize(3);
          param_srv.request.data[0]=quadMass;
          param_srv.request.data[1]=9.81;
          param_srv.request.data[2]=meanTW;
          quadParamService.call(param_srv);
        }
      }

      nav_msgs::Odometry odom_msg, localOdom_msg;
      odom_msg.header = msg->header;
      odom_msg.child_frame_id = msg->header.frame_id;
      // odom_msg.child_frame_id = "quadFrame";
      odom_msg.pose.pose.position.x = state(0);
      odom_msg.pose.pose.position.y = state(1);
      odom_msg.pose.pose.position.z = state(2);
      odom_msg.twist.twist.linear.x = state(3);
      odom_msg.twist.twist.linear.y = state(4);
      odom_msg.twist.twist.linear.z = state(5);
      for(int i = 0; i < 3; i++)
      {
        for(int j = 0; j < 3; j++)
        {
          odom_msg.pose.covariance[6 * i + j] = proc_noise(i, j);
          odom_msg.twist.covariance[6 * i + j] = proc_noise(3 + i, 3 + j);
        }
      }

      odom_msg.pose.pose.orientation = msg->pose.orientation;

      // Single step differentitation for angular velocity
      static Eigen::Matrix3d R_prev(Eigen::Matrix3d::Identity());
      Eigen::Matrix3d R(Eigen::Quaterniond(msg->pose.orientation.w, msg->pose.orientation.x,
                                           msg->pose.orientation.y, msg->pose.orientation.z));
      if(dt > 1e-6)
      {
        const Eigen::Matrix3d R_dot = (R - R_prev) / dt;
        const Eigen::Matrix3d w_hat = R_dot * R.transpose();

        odom_msg.twist.twist.angular.x = w_hat(2, 1);
        odom_msg.twist.twist.angular.y = w_hat(0, 2);
        odom_msg.twist.twist.angular.z = w_hat(1, 0);
      }
      R_prev = R;

      odom_pub_.publish(odom_msg);

      //Publish tf  
      if(publish_tf_)
      {
        PublishTransform(odom_msg.pose.pose, odom_msg.header,
                         child_frame_id_);
      }

      //Publish local odometry message
      localOdom_msg = odom_msg;
      localOdom_msg.pose.pose.position.x = localOdom_msg.pose.pose.position.x;
      localOdom_msg.pose.pose.position.y = localOdom_msg.pose.pose.position.y;
      localOdom_msg.pose.pose.position.z = localOdom_msg.pose.pose.position.z;
      localOdom_pub_.publish(localOdom_msg);

      // Publish message for px4 mocap topic
      geometry_msgs::PoseStamped mocap_msg;
      mocap_msg.pose.position.x = msg->pose.position.x;
      mocap_msg.pose.position.y = msg->pose.position.y;
      mocap_msg.pose.position.z = msg->pose.position.z;
      mocap_msg.pose.orientation = msg->pose.orientation;
      mocap_msg.header = msg->header;
      mocap_msg.header.frame_id = "fcu";
      mocap_pub_.publish(mocap_msg);
    }else{
        initPose_.pose.position.x=msg->pose.position.x;
        initPose_.pose.position.y=msg->pose.position.y;
        initPose_.pose.position.z=msg->pose.position.z;
        ROS_INFO("msg: %f %f %f",msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
        ROS_INFO("initpose: %f %f %f",initPose_.pose.position.x,initPose_.pose.position.y,initPose_.pose.position.z);

        // Initialize KalmanFilter
        KalmanFilter::State_t proc_noise_diag;
        proc_noise_diag(0) = 0.5 * max_accel * dt * dt;
        proc_noise_diag(1) = 0.5 * max_accel * dt * dt;
        proc_noise_diag(2) = 0.5 * max_accel * dt * dt;
        proc_noise_diag(3) = max_accel * dt;
        proc_noise_diag(4) = max_accel * dt;
        proc_noise_diag(5) = max_accel * dt;
        proc_noise_diag = proc_noise_diag.array().square();
        KalmanFilter::Measurement_t meas_noise_diag;
        meas_noise_diag(0) = 1e-2;
        meas_noise_diag(1) = 1e-2;
        meas_noise_diag(2) = 1e-2;
        meas_noise_diag = meas_noise_diag.array().square();
        Eigen::Matrix<double, 6, 1> initStates;
        initStates << initPose_.pose.position.x, initPose_.pose.position.y, initPose_.pose.position.z, 0.0, 0.0, 0.0;
/*        initStates << msg->pose.position.x-initPose_.pose.position.x, msg->pose.position.y-initPose_.pose.position.y, 
          msg->pose.position.z-initPose_.pose.position.z, 0,0,0; */
        kf_.initialize(initStates, 1.0 * KalmanFilter::ProcessCov_t::Identity(),
                    proc_noise_diag.asDiagonal(), meas_noise_diag.asDiagonal());


        // Initialize Kalman Filter with T/W modification
        KalmanTW::ProcessCov_t procNoise = (Eigen::Matrix<double,4,1>(0.5,0.5,0.5,0.005)).asDiagonal();
        KalmanTW::MeasurementCov_t measNoise = 1e-2 * Eigen::Matrix3d::Identity();
        KalmanTW::StateCov_t initCov = Eigen::Matrix<double,7,7>::Identity();
        initCov(6,6)=0.05;
        KalmanTW::State_t initState;
        initState << initPose_.pose.position.x, initPose_.pose.position.y, initPose_.pose.position.z, 0.0, 0.0, 0.0, 1.0;
        kfTW_.initialize(initState,initCov,procNoise,measNoise);
        Eigen::Matrix<double,7,1> checkvec;
        //checkvec = kfTW_.getState();
        //ROS_INFO("inittest: %f",checkvec(6));

        // do not repeat initialization
        kfInit=true;
    }
}


void gpsOdom::PublishTransform(const geometry_msgs::Pose &pose,
                               const std_msgs::Header &header,
                               const std::string &child_frame_id)
{
  // Publish tf
  geometry_msgs::Vector3 translation;
  translation.x = pose.position.x;
  translation.y = pose.position.y;
  translation.z = pose.position.z;

  geometry_msgs::TransformStamped transform_stamped;
  transform_stamped.header = header;
  transform_stamped.child_frame_id = child_frame_id;
  transform_stamped.transform.translation = translation;
  transform_stamped.transform.rotation = pose.orientation;

  tf_broadcaster_.sendTransform(transform_stamped);
}


void gpsOdom::singleBaselineRTKCallback(const gbx_ros_bridge_msgs::SingleBaselineRTK::ConstPtr &msg)
{
    //if(baseECEF_vector.squaredNorm() < 0.01)
    //{
    //}
    double ttime=msg->tSolution.secondsOfWeek + msg->tSolution.fractionOfSecond
        + msg->tSolution.week * sec_in_week -msg->deltRSec;
    if(ttime>lastRTKtime)  //only use newest time
    {
        hasAlreadyReceivedRTK=true;
        lastRTKtime=ttime;
        if(msg->testStat > minTestStat)
        {
            validRTKtest=true;
            Eigen::Vector3d tmpvec;
            tmpvec(0) = msg->rx + msg->rxRov - baseECEF_vector(0); //error vector from ECEF at init time
            tmpvec(1) = msg->ry + msg->ryRov - baseECEF_vector(1);
            tmpvec(2) = msg->rz + msg->rzRov - baseECEF_vector(2);
            internalPose = 0.5*Recef2enu*tmpvec - n_err; //rescaling
            //ROS_INFO("%f %f %f %f",msg->rx, msg->rxRov, tmpvec(0), internalPose(0)); //debugging
        }else{validRTKtest=false;}

        if(abs(lastRTKtime-lastA2Dtime)<.001 && validA2Dtest && validRTKtest
                && hasAlreadyReceivedRTK && hasAlreadyReceivedA2D)  //only resend pose if new
        {
            internalSeq++;
            geometry_msgs::PoseStamped selfmsg;
            selfmsg.header.seq=internalSeq;
            selfmsg.header.stamp=ros::Time(lastRTKtime);
            /* subtraction is the apparent convention in /Valkyrie/pose*/
            selfmsg.header.frame_id="fcu";
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
        }else{ lastRTKtime=msg->tSolution.secondsOfWeek+msg->tSolution.fractionOfSecond-msg->deltRSec +
                        msg->tSolution.week * sec_in_week;}
    }
}


void gpsOdom::attitude2DCallback(const gbx_ros_bridge_msgs::Attitude2D::ConstPtr &msg)
{
    double ttime=msg->tSolution.secondsOfWeek + msg->tSolution.fractionOfSecond + msg->tSolution.week * sec_in_week
          - msg->deltRSec;
    if(ttime < 0.10)  //if A2D starts publishing blank messages
    {
        hasAlreadyReceivedA2D=true;
        if(hasAlreadyReceivedRTK && validRTKtest)
        {
            internalSeq++;
            geometry_msgs::PoseStamped selfmsg;
            selfmsg.header.seq=internalSeq;
            selfmsg.header.stamp=ros::Time(lastRTKtime);
            /* subtraction is the apparent convention in /Valkyrie/pose*/
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
    }

    //if everything is working
    if(ttime>lastA2Dtime)  //only use newest time
    {
        hasAlreadyReceivedA2D=true;
        lastA2Dtime=ttime;
        if(msg->testStat > minTestStat)
        {
            validA2Dtest=true;
            //attitude vec is Euler=[0,0, pi/2-azAngle]
            internalQuat = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX())
                * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY())
                * Eigen::AngleAxisd(pi/2-msg->azAngle, Eigen::Vector3d::UnitZ());

        }else{validA2Dtest=false;}

        if(abs(lastRTKtime-lastA2Dtime)<.001 && validA2Dtest && validRTKtest
                && hasAlreadyReceivedRTK && hasAlreadyReceivedA2D)  //only resend pose if new
        {
            internalSeq++;
            geometry_msgs::PoseStamped selfmsg;
            selfmsg.header.seq=internalSeq;
            selfmsg.header.stamp=ros::Time(lastRTKtime);
            /* subtraction is the apparent convention in /Valkyrie/pose*/
            selfmsg.header.frame_id="fcu";
            selfmsg.pose.position.x=internalPose(0);
            selfmsg.pose.position.y=internalPose(1);
            selfmsg.pose.position.z=internalPose(2);
            selfmsg.pose.orientation.x=internalQuat.x();
            selfmsg.pose.orientation.y=internalQuat.y();
            selfmsg.pose.orientation.z=internalQuat.z();
            selfmsg.pose.orientation.w=internalQuat.w();
            internalPosePub_.publish(selfmsg);
            //ROS_INFO("internalpose: %f %f %f",internalPose(0),internalPose(1),internalPose(2));
            hasAlreadyReceivedRTK=false; hasAlreadyReceivedA2D=false;

        }else if(abs(lastRTKtime-lastA2Dtime)<.001 && validRTKtest && hasAlreadyReceivedRTK
                && hasAlreadyReceivedA2D) /*if A2D stops publishing, go for pose anyways. May change
                this behavior later*/
        {
            internalSeq++;
            geometry_msgs::PoseStamped selfmsg;
            selfmsg.header.seq=internalSeq;
            selfmsg.header.stamp=ros::Time(lastRTKtime);
            /* subtraction is the apparent convention in /Valkyrie/pose*/
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

    }
}

void gpsOdom::joyCallback(const sensor_msgs::Joy::ConstPtr &msg)
{
  if(msg->buttons[1]==1 || msg->buttons[2]==1 || msg->buttons[3]==1)
  {  isArmed=true;}
  else if(msg->buttons[0]==1)
  {  isArmed=false;}
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


int main(int argc, char **argv)
{
  ros::init(argc, argv, "gps_odom");
  ros::NodeHandle nh;

  try
  {
    gps_odom::gpsOdom gps_odom(nh);
    ros::spin();
  }
  catch(const std::exception &e)
  {
    ROS_ERROR("%s: %s", nh.getNamespace().c_str(), e.what());
    return 1;
  }
  return 0;
}
