#include <ros/ros.h>
#include <turtlesim/Pose.h>

void turtle_pose_callback(const turtlesim::Pose &msg)
{
    ROS_INFO("X=%f", msg.x);
    ROS_INFO_STREAM("Y=" << msg.y);
    ROS_INFO("T=%f", msg.theta);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pose_subscriber");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("turtle1/pose", 10, turtle_pose_callback);
    ros::spin();
}
