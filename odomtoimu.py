# read imu data from a text file and publish odom messages to /odom
# import rospy
# from nav_msgs.msg import Odometry

# def read_imu_data():
#     with open('imu.txt', 'r') as f:
#         lines = f.readlines()
#         for line in lines:
#             data = line.strip().split()
#             acc = [float(data[i]) for i in range(3)]
#             gyr = [float(data[i]) for i in range(3, 6)]
#             quaternion = [float(data[i]) for i in range(6, 10)]
#             yield acc, gyr, quaternion

# def publish_odom():

#     acc, gyr, quaternion = next(read_imu_data())
#     rospy.init_node('odom_publisher', anonymous=True)
#     pub = rospy.Publisher('odom', Odometry, queue_size=10)

#     #imu to odom prediction
#     odom = Odometry()
#     odom.header.stamp = rospy.Time.now()
#     odom.header.frame_id = "odom"
#     odom.child_frame_id = "base_link"

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Point, Pose, Twist, Vector3
from tf.transformations import quaternion_from_euler
import numpy as np
from scipy.spatial.transform import Rotation as R

class IMUOdometry:
    def __init__(self):
        self.position = np.zeros(3)
        self.velocity = np.zeros(3)
        self.orientation = R.from_quat([0, 0, 0, 1])  # Identity quaternion

    def update(self, linear_acceleration, angular_velocity, orientation_quat, dt):
        # Update orientation using the new quaternion
        self.orientation = R.from_quat(orientation_quat)

        # Convert angular velocity to orientation change
        angular_velocity_vector = np.array(angular_velocity) * dt
        angular_velocity_quat = R.from_rotvec(angular_velocity_vector)

        # Update orientation
        self.orientation *= angular_velocity_quat

        # Transform linear_acceleration to the global frame
        linear_acceleration_global = self.orientation.apply(linear_acceleration)

        # Update velocity and position
        self.velocity += linear_acceleration_global * dt
        self.position += self.velocity * dt

    def get_position(self):
        return self.position

    def get_velocity(self):
        return self.velocity

    def get_orientation(self):
        return self.orientation.as_quat()

def publish_odometry(odom_pub, imu_odometry):
    odom_msg = Odometry()
    odom_msg.header.stamp = rospy.Time.now()
    odom_msg.header.frame_id = "odom"
    odom_msg.child_frame_id = "base_link"

    position = imu_odometry.get_position()
    velocity = imu_odometry.get_velocity()
    orientation = imu_odometry.get_orientation()

    odom_msg.pose.pose = Pose(Point(*position), Quaternion(*orientation))
    odom_msg.twist.twist = Twist(Vector3(*velocity), Vector3(0, 0, 0))

    odom_pub.publish(odom_msg)


    # with open('odom.txt', 'a') as f:
    #     f.write(f"({position[0]}, {position[1]})\n")

def main():
    rospy.init_node('imu_odometry_publisher')

    odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)
    imu_odometry = IMUOdometry()

    rate = rospy.Rate(100)  # 100 Hz
    dt = 0.01  # Time step in seconds

    with open('imu.txt', 'r') as f:
        lines = f.readlines()
        for line in lines:
            data = line.strip().split()
            acc = [float(data[i]) for i in range(3)]
            gyr = [float(data[i]) for i in range(3, 6)]
            quaternion = [float(data[i]) for i in range(6, 10)]

            imu_odometry.update(acc, gyr, quaternion, dt)
            publish_odometry(odom_pub, imu_odometry)
            rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass


