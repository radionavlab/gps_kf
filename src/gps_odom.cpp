//FOR USE WITH GPS ONLY

#include <Eigen/Geometry>
#include "gps_odom.hpp"
#include <string>
#include <iostream>

namespace po = boost::program_options;

namespace gps_odom
{
gpsOdom::gpsOdom(ros::NodeHandle &nh, int argc, char **argv)
{

	//Get data about node and topic to listen
	std::string quadPoseTopic, rtktopic, a2dtopic, posePubTopic;
	double tmax;
	quadName = ros::this_node::getName();
//  Eigen::Vector3d enuInput;
	bool useVicon=false; //temporary var
	runTW=true; //globally used var
	ros::param::get(quadName + "/quadPoseTopic", quadPoseTopic);
	ros::param::get(quadName + "/arenaCenterX", baseECEF_vector(0));
	ros::param::get(quadName + "/arenaCenterY", baseECEF_vector(1));
	ros::param::get(quadName + "/arenaCenterZ", baseECEF_vector(2));
	ros::param::get(quadName + "/arenaCenterX_ENU", n_err(0));
	ros::param::get(quadName + "/arenaCenterY_ENU", n_err(1));
	ros::param::get(quadName + "/arenaCenterZ_ENU", n_err(2));
	ros::param::get(quadName + "/posePubTopic", posePubTopic);
	ros::param::get(quadName + "/minimumTestStat",minTestStat);
	ros::param::get(quadName + "/maxTW",tmax);
	ros::param::get(quadName + "/mass",quadMass);
	ros::param::get(quadName + "/run_TW",runTW);
	ros::param::get(quadName + "/useViconInsteadOfGps",useVicon);
	ros::param::get(quadName + "/pubRate",pubRate);
	ROS_INFO("Completed primary reads");
    throttleMax = tmax*9.81;
    int gbxport;
    ros::param::get(quadName + "/gbxport",gbxport);

	twCounter=0;

	//Get additional parameters for the kalkman filter
	nh.param(quadName + "/max_accel", max_accel, 3.0);
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

        ROS_INFO("gbx listening on port %i",gbxport);
    
        //http://eigen.tuxfamily.org/dox-devel/group__TopicUnalignedArrayAssert.html
        /*baseECEF_vector(0) = msg->rx+msg->rxRov; //NOTE: THIS SHOULD BE READ IN VIA .LAUNCH WHEN USING GLOBAL FRAME
	baseECEF_vector(1) = msg->ry+msg->ryRov;
	baseECEF_vector(2) = msg->rz+msg->rzRov;*/
	Recef2enu=ecef2enu_rotMatrix(baseECEF_vector);
	//std::cout << Recef2enu << std::endl;
	//baseENU_vector=Recef2enu*baseECEF_vector;

	//Account for WRW not being perfectly aligned in ENU
	double thetaWRW;
	thetaWRW = 6.2*pi/180; //angle of rooftop coordinate system WRT ENU
	Rwrw << cos(thetaWRW), -1*sin(thetaWRW), 0,
			sin(thetaWRW), cos(thetaWRW), 0,
			0, 0, 1.0;
	RBI << 1.0, 0.0, 0.0,
		 0.0, 1.0, 0.0,
		 0.0, 0.0, 1.0;


	/*Read separate GBX port named  "gbxport".  This is to avoid an override for port number
		in ROS at the --port or -p argument.*/
/*	auto gbxStream = std::make_shared<GbxStream>();
	gbxStream->pauseStream();

    int port = gbxport;
    const uint16_t DEFAULT_PORT = 0;
    uint16_t port = DEFAULT_PORT;

    //po::options_description desc("Allowed options");
    //desc.add_options()
    //    ("gbxport", po::value<uint16_t>(&port), "GBX Input Port");
	//po::variables_map vm;
	//po::store(po::parse_command_line(argc,argv,desc),vm);
	//po::notify(vm);

  	if(port == DEFAULT_PORT) {
    	std::cerr << "Specify  --\'gbxport\'  as a program option." << std::endl;
    	exit(EXIT_FAILURE);
  	}

	auto epOutput = std::make_shared<GbxStreamEndpointGPSKF>();
	epOutput->configure(nh, baseECEF_vector, Recef2enu);
	//epOutput->donothing();
	epOutput->filter(GbxStream::DEFAULT_PRIMARY).addReportType(Report::CODA);
	epOutput->filter(GbxStream::DEFAULT_PRIMARY).addReportType(Report::SINGLE_BASELINE_RTK);
	epOutput->filter(GbxStream::DEFAULT_PRIMARY).addReportType(Report::ATTITUDE_2D);
	epOutput->filter(GbxStream::DEFAULT_PRIMARY).enableWhitelist();
	//make endpoint
	auto epInput = std::make_shared<GbxStreamEndpointIN>(port, OptionObject::protocol_enum::IP_UDP, OptionObject::peer_type_enum::ROVER);
 	gbxStream->resumeStream();
  	ROS_INFO("Pipe created");*/

	lastRTKtime=0;
	lastA2Dtime=0;
	//internalQuat.resize(4);
	internalSeq=0;
	sec_in_week = 604800;
	isArmed=false;
	kfInit=false; //KF will need to be initialized
	throttleSetpoint = 9.81/throttleMax; //the floor is the throttle
	quaternionSetpoint.x()=0; quaternionSetpoint.y()=0; quaternionSetpoint.z()=0; quaternionSetpoint.w()=1;
	internalQuatPrev.x()=0; internalQuatPrev.y()=0; internalQuatPrev.z()=0; internalQuatPrev.w()=1;

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
	
	bool useUDP = false;
	ros::param::get(quadName + "/useUDP",useUDP);

	//Vicon or GPS
	if(useVicon)
	{
	//NOTE: THE NEW VICON POSE TOPIC MUST BE ASSIGNED IN THE LAUNCH FILE VIA GLOBAL REFERENCE
	gps_sub_ = nh.subscribe(quadPoseTopic,1,&gpsOdom::viconCallback,
							this, ros::TransportHints().tcpNoDelay());
	}else
	{
	//Publisher
	internalPosePub_ = nh.advertise<geometry_msgs::PoseStamped>(posePubTopic,11);

	bool useUDP = false;
	ros::param::get(quadName + "/useUDP",useUDP);
	if(useUDP)
	{
		gps_sub_ = nh.subscribe(quadPoseTopic, 1, &gpsOdom::gpsCallback,
							this, ros::TransportHints().unreliable().reliable().tcpNoDelay());
	}else
	{
		gps_sub_ = nh.subscribe(quadPoseTopic, 10, &gpsOdom::gpsCallback,
							this, ros::TransportHints().tcpNoDelay());
	}
	if(pubRate > 1e-6)
	{
		bool oneshot = false;
		timerPub_ = nh.createTimer(ros::Duration(1.0/pubRate), &gpsOdom::timerCallback, this, oneshot);
//    timerPub_ = nh.createTimer(ros::Duration(1.0/40.0), &gpsOdom::timerCallback, this, bool oneshot = false); //correct declaration per docs
	}
	}

	//T/W filter terms
	if(useUDP)
	{
	thrustSub_ = nh.subscribe("mavros/setpoint_attitude/att_throttle", 10,
							&gpsOdom::throttleCallback,this, ros::TransportHints().unreliable().reliable().tcpNoDelay());
	attSub_ = nh.subscribe("mavros/setpoint_attitude/attitude", 10,
							&gpsOdom::attSetCallback,this, ros::TransportHints().unreliable().reliable().tcpNoDelay());
	joy_sub_ = nh.subscribe("joy",10,&gpsOdom::joyCallback, this, ros::TransportHints().unreliable().reliable().tcpNoDelay());
	}else
	{
	thrustSub_ = nh.subscribe("mavros/setpoint_attitude/att_throttle", 10,
							&gpsOdom::throttleCallback,this, ros::TransportHints().tcpNoDelay());
	attSub_ = nh.subscribe("mavros/setpoint_attitude/attitude", 10,
							&gpsOdom::attSetCallback,this, ros::TransportHints().tcpNoDelay());
	joy_sub_ = nh.subscribe("joy",10,&gpsOdom::joyCallback, this, ros::TransportHints().tcpNoDelay()); 
	}
	

	twPub_ = nh.advertise<gps_kf::twUpdate>("ThrustToWeight",10);
	odomTimePub_ = nh.advertise<gps_kf::odomWithGpsTime>("OdomWithGpsTime",10);
	quadParamService = nh.serviceClient<px4_control::updatePx4param>("px4_control_node/updateQuadParam");

	L_cg2p<< 0.1013,-0.0004,0.0472;
	
	ROS_INFO("Kalman Filter Node started! Listening to ROS input topic: %s within specified namespace", (quadPoseTopic).c_str());

	//Get initial pose
	//initPose_ = ros::topic::waitForMessage<geometry_msgs::PoseStamped>(quadPoseTopic);
	//geometry_msgs::PoseStamped initPose_;

}

void gpsOdom::timerCallback(const ros::TimerEvent &event)
{
	if(kfInit)
	{
	KalmanFilter::State_t state = kf_.getState();
	ros::Time thisTime = ros::Time::now();
	const double dtRosTime = thisTime.toSec() - lastRosTime.toSec();

	//constant velocity model
	Eigen::Matrix<double,6,6> Amat=Eigen::Matrix<double,6,6>::Identity();
	Amat.topRightCorner(3,3) = dtRosTime*Eigen::Matrix3d::Identity();
	state = Amat*state;

	//Publish the odom message
	nav_msgs::Odometry localOdom_msg;
	localOdom_msg.header.stamp = thisTime;
	localOdom_msg.header.frame_id = "world";
	localOdom_msg.child_frame_id = "world";
	localOdom_msg.pose.pose.position.x = state(0);
	localOdom_msg.pose.pose.position.y = state(1);
	localOdom_msg.pose.pose.position.z = state(2);
	localOdom_msg.twist.twist.linear.x = state(3);
	localOdom_msg.twist.twist.linear.y = state(4);
	localOdom_msg.twist.twist.linear.z = state(5);
	const KalmanFilter::ProcessCov_t proc_noise = kf_.getProcessNoise();
	for(int i = 0; i < 3; i++)
	{
		for(int j = 0; j < 3; j++)
		{
		localOdom_msg.pose.covariance[6 * i + j] = proc_noise(i, j);
		localOdom_msg.twist.covariance[6 * i + j] = proc_noise(3 + i, 3 + j);
		}
	}

	// Single step differentitation for angular velocity
	Eigen::Matrix3d R = rotMatFromQuat(internalQuat);
	if(dtRosTime > 1.0e-6 && dtRosTime < 0.5)
	{
		const Eigen::Matrix3d R_dot = (R - Rclass) / dt;
		const Eigen::Matrix3d w_hat = R_dot * R.transpose();

		localOdom_msg.twist.twist.angular.x = w_hat(2, 1);
		localOdom_msg.twist.twist.angular.y = w_hat(0, 2);
		localOdom_msg.twist.twist.angular.z = w_hat(1, 0);

		//propagate internal quat
		Eigen::Quaterniond dQ;
		dQ.x() = 0.5*dtRosTime*w_hat(0);
		dQ.y() = 0.5*dtRosTime*w_hat(1);
		dQ.z() = 0.5*dtRosTime*w_hat(2);
		dQ.w() = 1.0;
		dQ.normalize();
		//quaternionConstraint(dQ);
		internalQuat = dQ*internalQuat;
	}

	internalQuat.normalize();

	localOdom_msg.pose.pose.orientation.x = internalQuat.x();
	localOdom_msg.pose.pose.orientation.y = internalQuat.y();
	localOdom_msg.pose.pose.orientation.z = internalQuat.z();
	localOdom_msg.pose.pose.orientation.w = internalQuat.w();

	localOdom_pub_.publish(localOdom_msg);


	gps_kf::odomWithGpsTime odom2;
	odom2.odometry = localOdom_msg;
	odom2.time.week = gpsWeek_;
	odom2.time.secondsOfWeek = gpsSec_;
	odom2.time.fractionOfSecond = gpsFracSec_;
	odom2.dtRX = dtRX_;
	odomTimePub_.publish(odom2);
	// Publish message for px4 mocap topic
	geometry_msgs::PoseStamped mocap_msg;
	mocap_msg.pose.position.x = state(0);
	mocap_msg.pose.position.y = state(1);
	mocap_msg.pose.position.z = state(2);
	mocap_msg.pose.orientation = localOdom_msg.pose.pose.orientation;
	mocap_msg.header.seq = 0;
	mocap_msg.header.stamp = thisTime;
	//mocap_msg.header.frame_id = "refnet_enu";
	mocap_msg.header.frame_id = "fcu";
	mocap_pub_.publish(mocap_msg);    

	lastRosTime=thisTime;

	}
	return;
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
		ECEF(0) = msg->pose.position.x;
		ECEF(1) = msg->pose.position.y;
		ECEF(2) = msg->pose.position.z;
		KalmanFilter::Measurement_t meas(msg->pose.position.x, msg->pose.position.y,
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
		z_last(0)=msg->pose.position.x;
		z_last(1)=msg->pose.position.y;
		z_last(2)=msg->pose.position.z;
		z_last=Rwrw*z_last;

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
		KalmanTW::Measurement_t measM(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
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
			battstat = 10.0+90.0/(1.75-1.40)*(meanTW-1.40); //approx battery percent

			//Saturate meanTW
			if(meanTW>1.8){meanTW=1.8;}else if(meanTW<1.3){meanTW=1.3;}
			ROS_INFO("Updating T/W to %f. Battery at approximately %f%%",meanTW,battstat);
				//ROS_INFO("service called");

			//Call service
			if(runTW)
			{
			px4_control::updatePx4param param_srv;
			param_srv.request.data.resize(3);
			param_srv.request.data[0]=quadMass;
			param_srv.request.data[1]=9.81;
			param_srv.request.data[2]=meanTW;
			quadParamService.call(param_srv);
			}
	
			//Publish TW to TW topic.  NOTE: This does not update TW on the quad. The
			//purpose of this publisher is to produce a value that can be observed in
			//a rosbag as rosbags do not record service calls.
			gps_kf::twUpdate tw_msg;
			tw_msg.rosTime = t_last_meas.toSec();
			tw_msg.estimatedTW = meanTW;
			twPub_.publish(tw_msg);
			
		}
		}

		// Single step differentitation for angular velocity
		static Eigen::Matrix3d R_prev(Eigen::Matrix3d::Identity());
		Eigen::Matrix3d R(Eigen::Quaterniond(msg->pose.orientation.w, msg->pose.orientation.x,
											 msg->pose.orientation.y, msg->pose.orientation.z));
		Eigen::Matrix3d w_hat = Eigen::Matrix3d::Zero();
		if(dt > 1e-6)
		{
		const Eigen::Matrix3d R_dot = (R - R_prev) / dt;
		w_hat = R_dot * R.transpose();
		}
		R_prev = R; //R_prev is used internally in measurement proc step
		Rclass = R; //R_class is used with the timer

		//odom_pub_.publish(localOdom_msg);
		// //Publish tf  
		// if(publish_tf_)
		// {
		//   PublishTransform(localOdom_msg.pose.pose, localOdom_msg.header,
		//                    child_frame_id_);
		// }

		//Publish local odometry message. Setting pubRate=0 publishes here, nonzero publishes with timer
		if(pubRate<1e-6)
		{
		nav_msgs::Odometry localOdom_msg;
		localOdom_msg.header = msg->header;
//      localOdom_msg.child_frame_id = msg->header.frame_id;
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

		localOdom_msg.pose.pose.orientation = msg->pose.orientation;
		localOdom_msg.twist.twist.angular.x = w_hat(2, 1);
		localOdom_msg.twist.twist.angular.y = w_hat(0, 2);
		localOdom_msg.twist.twist.angular.z = w_hat(1, 0);      
		localOdom_pub_.publish(localOdom_msg);
		
		// Publish message for px4 mocap topic
		geometry_msgs::PoseStamped mocap_msg;
		mocap_msg.pose.position.x = msg->pose.position.x;
		mocap_msg.pose.position.y = msg->pose.position.y;
		mocap_msg.pose.position.z = msg->pose.position.z;
		mocap_msg.pose.orientation = msg->pose.orientation;
		mocap_msg.header = msg->header;
		//mocap_msg.header.frame_id = "refnet_enu";
		mocap_msg.header.frame_id = "fcu";
		mocap_pub_.publish(mocap_msg);      
		}


	}else{  //run intitialization
		Eigen::Vector3d p0(msg->pose.position.x,msg->pose.position.y,msg->pose.position.z);
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

	//lastRosTime is the ROS timestamp corresponding to the last time published  
	lastRosTime = ros::Time::now();
	return;
}

/*
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
}*/


void gpsOdom::joyCallback(const sensor_msgs::Joy::ConstPtr &msg)
{
	if(msg->buttons[1]==1 || msg->buttons[2]==1 || msg->buttons[3]==1)
	{
	isArmed=true;
	}
	else if(msg->buttons[0]==1)
	{
	isArmed=false;
	}
	
}

} //end namespace


