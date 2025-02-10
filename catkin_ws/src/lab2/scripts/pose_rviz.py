#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped, Quaternion
from turtlesim.msg import Pose
import tf

def callback(data):
    msg = PoseStamped()
    msg.header.frame_id = "map"
    msg.header.seq = 0
    msg.header.stamp = rospy.Time.now()

    q = tf.transformations.quaternion_from_euler(0, 0, data.theta)
    msg.pose.orientation = Quaternion(*q)
    msg.pose.position.x = data.x
    msg.pose.position.y = data.y
    
    # Publish the transformed pose
    pub.publish(msg)

if __name__ == '__main__':
    try:
        # Initialize the ROS node
        rospy.init_node('pose_rviz', anonymous=True)
        global pub
        pub = rospy.Publisher('turtle1/pose_stamped', PoseStamped, queue_size=10)
        rospy.Subscriber('/turtle1/pose', Pose, callback)
        rospy.spin()
        
    except rospy.ROSInterruptException:
        pass
