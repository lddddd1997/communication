#include "xbee_pro.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "xbee_write_node"); 
    ros::NodeHandle nh("~");
    lddddd::XbeePro XbeePro(lddddd::XbeePro::O_WR, nh);
    
    while(ros::ok())
    {
        XbeePro.TaskRun();
    }
    return 0;
}

