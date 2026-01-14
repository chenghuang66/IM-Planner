#include <tf/tf.h>
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose.h>

class Tricker
{
public:
    Tricker(ros::NodeHandle &nh) : nh_(nh)
    {
        ref_suber_ = nh.subscribe("/ref_path", 1, &Tricker::refCallback, this);
        pos_puber_ = nh.advertise<geometry_msgs::Pose>("/carla/ego_vehicle/control/set_transform", 1, true);
        freq_ = nh.createTimer(ros::Duration(0.05), &Tricker::ctrl, this);
    }

private:
    void refCallback(const nav_msgs::PathConstPtr &msg)
    {
        // ROS_INFO("Subscribe Ref Path!");
        x_.clear();
        x_.resize(msg->poses.size());
        y_.clear();
        y_.resize(msg->poses.size());
        phi_.clear();
        phi_.resize(msg->poses.size());
        for (size_t i = 0; i < msg->poses.size(); ++i)
        {
            x_.at(i) = msg->poses.at(i).pose.position.x;
            y_.at(i) = msg->poses.at(i).pose.position.y;
            phi_.at(i) = tf::getYaw(msg->poses.at(i).pose.orientation);
            std::cout << x_.at(i) << ", " << y_.at(i) << ", " << phi_.at(i) << std::endl;
        }
    }
    void ctrl(const ros::TimerEvent &event)
    {
        if (x_.empty())
            return;
        geometry_msgs::Pose pos;
        pos.position.x = x_.at(0);
        pos.position.y = y_.at(0);
        pos.position.z = 0.0;
        pos.orientation = tf::createQuaternionMsgFromYaw(phi_.at(0));
        // ROS_INFO("Publish Car Pose: %f, %f, %f!", x_.at(0), y_.at(0), phi_.at(0));
        x_.erase(x_.begin());
        y_.erase(y_.begin());
        phi_.erase(phi_.begin());
        pos_puber_.publish(pos);
    }
    ros::NodeHandle nh_;
    ros::Subscriber ref_suber_;
    ros::Publisher pos_puber_;
    ros::Timer freq_;
    std::vector<double> x_;
    std::vector<double> y_;
    std::vector<double> phi_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "trick_node");
    ros::NodeHandle nh;
    Tricker tricker(nh);
    ros::spin();
    return 0;
}