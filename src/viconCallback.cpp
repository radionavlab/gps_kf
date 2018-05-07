#include "gps_odom.hpp"

namespace gps_odom
{
void gpsOdom::viconCallback(const geometry_msgs::TransformStamped::ConstPtr &msg)
{
  Eigen::Vector3d zMeas;
  if(kfInit)  //only filter if initPose_ has been set
  {   
      static ros::Time t_last_proc = msg->header.stamp;
      static ros::Time time_of_last_fix=msg->header.stamp;
      double lastKnownSpeed=0;
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
      ECEF(0) = msg->transform.translation.x;
      ECEF(1) = msg->transform.translation.y;
      ECEF(2) = msg->transform.translation.z;
      KalmanFilter::Measurement_t meas(msg->transform.translation.x, msg->transform.translation.y,
                                             msg->transform.translation.z);
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
      meas = Rwrw*meas;
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

      /*//Hypothesis test
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
      }*/
      kf_.measurementUpdate(meas, meas_dt);
      time_of_last_fix=msg->header.stamp;

      //update static variable AFTER using it
      z_last(0)=msg->transform.translation.x;
      z_last(1)=msg->transform.translation.y;
      z_last(2)=msg->transform.translation.z;
      z_last=Rwrw*z_last;

      const KalmanFilter::State_t state = kf_.getState();
      const KalmanFilter::ProcessCov_t proc_noise = kf_.getProcessNoise();
      xCurr = kf_.getState();

      //T/W filter
      if(state(2)>=0.10 && isArmed && runTW)
      {
        kfTW_.processUpdate(dt,uvec);
        Eigen::Matrix<double,7,1> xStateAfterProp=kfTW_.getState();
        //ROS_INFO("T/W after propagation: %f",xStateAfterProp(6));
        //update kfTW_
        KalmanTW::Measurement_t measM(msg->transform.translation.x, msg->transform.translation.y,
                                             msg->transform.translation.z);
        measM = Rwrw*measM;
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
          battstat = 10.0+90.0/(1.80-1.40)*(meanTW-1.40); //approx battery percent
          //Saturate meanTW
          if(meanTW>1.8){meanTW=1.8;}else if(meanTW<1.3){meanTW=1.3;}
          ROS_INFO("Updating T/W to %f. Battery at approximately %f%%",meanTW,battstat);
              //ROS_INFO("service called");          

          //Call service
          px4_control::updatePx4param param_srv;
          param_srv.request.data.resize(3);
          param_srv.request.data[0]=quadMass;
          param_srv.request.data[1]=9.81;
          param_srv.request.data[2]=meanTW;
          quadParamService.call(param_srv);

          //Publish TW to TW topic.  NOTE: This does not update TW on the quad. The
          //purpose of this publisher is to produce a value that can be observed in
          //a rosbag as rosbags do not record service calls.
          gps_kf::twUpdate tw_msg;
          tw_msg.rosTime = t_last_meas.toSec();
          tw_msg.estimatedTW = meanTW;
          twPub_.publish(tw_msg);
        }
      }


      nav_msgs::Odometry localOdom_msg;
      localOdom_msg.header = msg->header;
      localOdom_msg.child_frame_id = msg->header.frame_id;
      localOdom_msg.header.frame_id = "world";
      localOdom_msg.child_frame_id = "world";
      localOdom_msg.pose.pose.position.x = state(0);
      localOdom_msg.pose.pose.position.y = state(1);
      localOdom_msg.pose.pose.position.z = state(2);
      localOdom_msg.twist.twist.linear.x = state(3);
      localOdom_msg.twist.twist.linear.y = state(4);
      localOdom_msg.twist.twist.linear.z = state(5);
      for(int i = 0; i < 3; i++)
      {
        for(int j = 0; j < 3; j++)
        {
          localOdom_msg.pose.covariance[6 * i + j] = proc_noise(i, j);
          localOdom_msg.twist.covariance[6 * i + j] = proc_noise(3 + i, 3 + j);
        }
      }

      localOdom_msg.pose.pose.orientation = msg->transform.rotation;

      // Single step differentitation for angular velocity
      static Eigen::Matrix3d R_prev(Eigen::Matrix3d::Identity());
      Eigen::Matrix3d R(Eigen::Quaterniond(msg->transform.rotation.w, msg->transform.rotation.x,
                                           msg->transform.rotation.y, msg->transform.rotation.z));
      if(dt > 1e-6)
      {
        const Eigen::Matrix3d R_dot = (R - R_prev) / dt;
        const Eigen::Matrix3d w_hat = R_dot * R.transpose();

        localOdom_msg.twist.twist.angular.x = w_hat(2, 1);
        localOdom_msg.twist.twist.angular.y = w_hat(0, 2);
        localOdom_msg.twist.twist.angular.z = w_hat(1, 0);
      }
      R_prev = R;

      //odom_pub_.publish(localOdom_msg);
      // //Publish tf  
      // if(publish_tf_)
      // {
      //   PublishTransform(localOdom_msg.pose.pose, localOdom_msg.header,
      //                    child_frame_id_);
      // }

      //Publish local odometry message
      localOdom_pub_.publish(localOdom_msg);

      // Publish message for px4 mocap topic
      geometry_msgs::PoseStamped mocap_msg;
      mocap_msg.pose.position.x = msg->transform.translation.x;
      mocap_msg.pose.position.y = msg->transform.translation.y;
      mocap_msg.pose.position.z = msg->transform.translation.z;
      mocap_msg.pose.orientation = msg->transform.rotation;
      mocap_msg.header = msg->header;
      //mocap_msg.header.frame_id = "refnet_enu";
      mocap_msg.header.frame_id = "fcu";
      mocap_pub_.publish(mocap_msg);
    }else{  //run intitialization
        Eigen::Vector3d p0(msg->transform.translation.x,msg->transform.translation.y,msg->transform.translation.z);
        p0=Rwrw*p0;
        initPose_.pose.position.x=p0(0);
        initPose_.pose.position.y=p0(1);
        initPose_.pose.position.z=p0(2);
        //ROS_INFO("msg: %f %f %f",msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
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
        KalmanTW::ProcessCov_t procNoise = (Eigen::Matrix<double,4,1>(0.5,0.5,0.5,0.000003)).asDiagonal();
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

}//end namespace