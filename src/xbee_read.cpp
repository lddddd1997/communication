#include "xbee_pro.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "xbee_read_node"); 
    ros::NodeHandle nh("~");
    lddddd::XbeePro XbeePro(lddddd::XbeePro::O_RD, nh);

    while(ros::ok())
    {
        XbeePro.TaskRun();
    }
    return 0;
}

