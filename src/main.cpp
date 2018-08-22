#include "gbxStreamEndpointGPSKF.hpp"


//handles sigkills, borrowed from pplink-demo
const char strSIGTERM[] = "SIGTERM";
const char strSIGINT[] = "SIGINT";
const char strSIGHUP[] = "SIGHUP";
const char *ptrSigString = nullptr;
static volatile sig_atomic_t sigterm_caught = 0;
extern "C" void signalHandler(int signum) {
    if(!sigterm_caught) {
        if(signum == SIGTERM || signum == SIGINT || signum == SIGHUP) {
            if(!ptrSigString) {
                if(signum == SIGTERM) ptrSigString = strSIGTERM;
                if(signum == SIGINT) ptrSigString = strSIGINT;
                if(signum == SIGHUP) ptrSigString = strSIGHUP;
            }
        sigterm_caught = 1;
        }
    }
}


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
        std::string quadName = ros::this_node::getName();
        int gbxport;
        Eigen::Vector3d baseECEF_vector;
        ros::param::get(quadName + "/arenaCenterX", baseECEF_vector(0));
        ros::param::get(quadName + "/arenaCenterY", baseECEF_vector(1));
        ros::param::get(quadName + "/arenaCenterZ", baseECEF_vector(2));
        ros::param::get(quadName + "/gbxport",gbxport);
        Eigen::Matrix3d Recef2enu = ecef2enu_rotMatrix(baseECEF_vector);
        auto gbxStream = std::make_shared<GbxStream>();
        gbxStream->pauseStream();

        int port = gbxport;
        auto epOutput = std::make_shared<GbxStreamEndpointGPSKF>();
        epOutput->configure(nh);
        //epOutput->donothing();
        epOutput->filter(GbxStream::DEFAULT_PRIMARY).addReportType(Report::CODA);
        epOutput->filter(GbxStream::DEFAULT_PRIMARY).addReportType(Report::SINGLE_BASELINE_RTK);
        epOutput->filter(GbxStream::DEFAULT_PRIMARY).addReportType(Report::ATTITUDE_2D);
        epOutput->filter(GbxStream::DEFAULT_PRIMARY).addReportType(Report::IMU);
        epOutput->filter(GbxStream::DEFAULT_PRIMARY).addReportType(Report::IMU_CONFIG);
        epOutput->filter(GbxStream::DEFAULT_PRIMARY).enableWhitelist();
        //make endpoint
        auto epInput = std::make_shared<GbxStreamEndpointIN>(port, OptionObject::protocol_enum::IP_UDP, OptionObject::peer_type_enum::ROVER);
        gbxStream->resumeStream();
    
         
        //Attach and throw errors if the attach fails       
        if (!gbxStream->attachSinkEndpoint(epOutput))
        {
            std::cerr << "Attachment failed! (output)" << std::endl;
            return -1;
        }
        if (!gbxStream->attachSourceEndpoint(epInput))
        {
            std::cerr << "Attachment failed! (input)" << std::endl;
            return -1;
        }
    
        ROS_INFO("Pipe created");
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

