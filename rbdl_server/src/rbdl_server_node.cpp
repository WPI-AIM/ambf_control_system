#include <ros/ros.h>
#include "rbdl_server/RBDLServer.h"



int main(int argc, char* argv[])
{

    ros::init(argc, argv, "rbdl_server");
    ros::NodeHandle n;
    RBDLServer my_server(&n);
    ros::spin();
     return 0;

}
