#include "xbee_pro.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "xbee_pro_node"); 
    ros::NodeHandle nh("~");
    lddddd::XbeePro XbeePro("/dev/ttyUSB0", 115200, lddddd::XbeePro::RDWR, nh);

    lddddd::CommunicationData CooperationData;
    while(ros::ok())
    {
        XbeePro.XbeeFrameRead(&CooperationData);
    }

    XbeePro.Serial::ClosePort();
    return 0;
}

