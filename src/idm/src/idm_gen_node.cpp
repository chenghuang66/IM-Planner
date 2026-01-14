#include "../include/idm/idm.hpp"
#include <ros/spinner.h>


int main(int argc, char **argv)
{
    ros::init(argc, argv, "idm_gen_node");
    ros::NodeHandle nh;
    IDM idm(nh);

    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        ros::spinOnce();
        // ROS_INFO("------------------------------");
        loop_rate.sleep();

    }


    // ros::spin();
    return 0;
}