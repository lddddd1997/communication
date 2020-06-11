#include "xbee_pro.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "xbee_pro_node"); 
    ros::NodeHandle nh("~");
    XbeePro XbeePro("/dev/ttyUSB0", 115200);
    ros::Timer loop_timer = nh.createTimer(ros::Duration(0.07), &XbeePro::LoopTimerCallback, &XbeePro);

    CommunicationData CooperationData;
    while(ros::ok())
    {
        ros::spinOnce();
        XbeePro.XbeeFrameRead(&CooperationData);
    }

    XbeePro.Serial::ClosePort();
    return 0;
}

