import rosbag
from nav_msgs.msg import Odometry
import rospy

# Define the covariance matrix you want to set for each message
COVARIANCE_MATRIX = [
    1e-2, 0, 0, 0, 0, 0,
    0, 1e-2, 0, 0, 0, 0,
    0, 0, 1e4, 0, 0, 0,
    0, 0, 0, 1e4, 0, 0,
    0, 0, 0, 0, 1e4, 0,
    0, 0, 0, 0, 0, 1e4
]

# 3x3 matrix
IMU_COVARIANCE_MATRIX = [
    1e5, 0, 0,
    0, 1e5, 0,
    0, 0, 1e5
]

# Input and output bag files
input_bagfile = 'input.bag'
output_bagfile = 'output.bag'

with rosbag.Bag(output_bagfile, 'w') as outbag:
    with rosbag.Bag(input_bagfile, 'r') as inbag:
        for topic, msg, t in inbag.read_messages():
            if topic == "/odom":
                # print("Updating covariance matrices in the bag file.")
                msg.pose.covariance = COVARIANCE_MATRIX
                msg.twist.covariance = COVARIANCE_MATRIX
            if topic == "/imu":
                msg.orientation_covariance = IMU_COVARIANCE_MATRIX
                msg.angular_velocity_covariance = IMU_COVARIANCE_MATRIX
                msg.linear_acceleration_covariance = IMU_COVARIANCE_MATRIX
            outbag.write(topic, msg, t)
            

print("Finished updating covariance matrices in the bag file.")
