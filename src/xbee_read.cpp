#include "xbee_pro.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "xbee_read_node"); 
    ros::NodeHandle nh("~");
    lddddd::XbeePro XbeePro("/dev/ttyUSB0", 115200, lddddd::XbeePro::O_RD, nh);

    lddddd::CommunicationData CooperationData;
    while(ros::ok())
    {
        XbeePro.XbeeFrameRead(&CooperationData);
    }
    return 0;
}

