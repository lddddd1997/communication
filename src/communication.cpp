#include "xbee_pro.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "xbee_pro_node"); 
    ros::NodeHandle nh("~");
    lddddd::XbeePro XbeePro(lddddd::XbeePro::RDWR, nh);

    while(ros::ok())
    {
        XbeePro.TaskRun();
        // (XbeePro.*(XbeePro.FunctionPointer_))();
    }

    return 0;
}

