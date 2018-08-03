#include "gps_odom.hpp"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "gps_odom");
	ros::NodeHandle nh;

  	std::signal(SIGTERM, signalHandler);
  	std::signal(SIGINT, signalHandler);
  	std::signal(SIGHUP, signalHandler);
 	std::signal(SIGQUIT, signalHandler);


	try
	{
		//create gbx stream
		quadName = ros::this_node::getName();
	    int gbxport;
	    Eigen::Vector3d baseECEF_vector;
	    ros::param::get(quadName + "/arenaCenterX", baseECEF_vector(0));
		ros::param::get(quadName + "/arenaCenterY", baseECEF_vector(1));
		ros::param::get(quadName + "/arenaCenterZ", baseECEF_vector(2));
    	ros::param::get(quadName + "/gbxport",gbxport);
    	Recef2enu = ecef2enu_rotMatrix(baseECEF_vector);
		auto gbxStream = std::make_shared<GbxStream>();
		gbxStream->pauseStream();

        int port = gbxport;
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
  		ROS_INFO("Pipe created");

  		//create gps node
		gps_odom::gpsOdom gps_odom(nh,argc,argv);
		//spin enables subscribers
        ros::spin();

        //Properly handle exceptions to fix the stuck pipe issue
        gbxStream->waitOnSourceDetach();
        gbxStream->detachSinkEndpoint(epOutput.get());

	}
	catch(const std::exception &e)
	{
		ROS_ERROR("%s: %s", nh.getNamespace().c_str(), e.what());
		return 1;
	}
	return 0;
}

