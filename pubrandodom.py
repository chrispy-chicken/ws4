# publish random odometry data and scan data
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import random
import time

import threading

def publish_odometry():
    rospy.init_node("odometry_publisher")
    pub_odom = rospy.Publisher("/odom", Odometry, queue_size=10)
    
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        msg = Odometry()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "map"
        msg.child_frame_id = "base_link"
        
        msg.pose.pose.position.x = random.uniform(-10.0, 10.0)
        msg.pose.pose.position.y = random.uniform(-10.0, 10.0)
        msg.pose.pose.position.z = random.uniform(-10.0, 10.0)

        msg.pose.pose.orientation.x = random.uniform(-10.0, 10.0)
        msg.pose.pose.orientation.y = random.uniform(-10.0, 10.0)
        msg.pose.pose.orientation.z = random.uniform(-10.0, 10.0)
        msg.pose.pose.orientation.w = random.uniform(-10.0, 10.0)

        msg.twist.twist.linear.x = random.uniform(-10.0, 10.0)
        msg.twist.twist.linear.y = random.uniform(-10.0, 10.0)
        msg.twist.twist.linear.z = random.uniform(-10.0, 10.0)

        msg.twist.twist.angular.x = random.uniform(-10.0, 10.0)
        msg.twist.twist.angular.y = random.uniform(-10.0, 10.0)
        msg.twist.twist.angular.z = random.uniform(-10.0, 10.0)

        msg.pose.covariance = [
            0.1, 0, 0, 0, 0, 0,
            0, 0.1, 0, 0, 0, 0,
            0, 0, 0.1, 0, 0, 0,
            0, 0, 0, 0.1, 0, 0,
            0, 0, 0, 0, 0.1, 0,
            0, 0, 0, 0, 0, 0.1
        ]

        msg.twist.covariance = [
            0.1, 0, 0, 0, 0, 0,
            0, 0.1, 0, 0, 0, 0,
            0, 0, 0.1, 0, 0, 0,
            0, 0, 0, 0.1, 0, 0,
            0, 0, 0, 0, 0.1, 0,
            0, 0, 0, 0, 0, 0.1
        ]


        # while rospy.Time.now().nsecs % 10 != 0:
        #     pass

        pub_odom.publish(msg)
        rate.sleep()



def main():
    # run both publishers in parallel
    publish_odometry()

    


if __name__ == "__main__":
    main()
