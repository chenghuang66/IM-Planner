#include "../include/iha/iha.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "iha_run_node");
    ros::NodeHandle nh;
    IHA iha(nh);
    ros::spin();
    return 0;
}