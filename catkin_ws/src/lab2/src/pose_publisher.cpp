# include <ros/ros.h>
# include <turtlesim/Pose.h>
# include <geometry_msgs/PoseStamped.h>
# include <turtlesim/Pose.h>
# include <tf/transform_datatypes.h>

int main (int argc, char **argv)  {
    ros::init(argc, argv, "pose_publisher");
    ros::NodeHandle nh;

    ros::Publisher pose_pub = nh.advertise<geometry_msgs::PoseStamped>("pose_output", 1000);
    ros::Rate loop_rate(10);

    int count = 0;
    while (ros::ok()) {
        geometry_msgs::PoseStamped msg;
        
        msg.header.frame_id = "map";
        msg.header.stamp = ros::Time::now();
        msg.header.seq = count++;
        
        
        msg.pose.orientation = tf::createQuaternionMsgFromYaw(0);
        msg.pose.position.x = 1;
        msg.pose.position.y = 1;


        pose_pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
