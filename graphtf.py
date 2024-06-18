import rospy
import tf2_ros
import matplotlib.pyplot as plt
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped

# Initialize the ROS node
rospy.init_node('tf_listener')

# Data storage for transforms
transforms = []

# Callback function to store transforms
def tf_callback(msg):
    for transform in msg.transforms:
        transforms.append(transform)

# Subscriber to /tf topic
rospy.Subscriber('/tf', TFMessage, tf_callback)

# Allow some time to collect transforms
rospy.sleep(5)

# Extract and plot the transforms
fig, ax = plt.subplots()

for transform in transforms:
    translation = transform.transform.translation
    frame = transform.child_frame_id

    # Plot the translation components
    ax.scatter(translation.x, translation.y, label=frame)

    # Optionally, you can also plot the rotation or any other transform details
    # Uncomment the following lines to plot rotations
    # ax.scatter(rotation.x, rotation.y, c='r', marker='x')

# Labeling the plot
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_title('Transforms in /tf Topic')
ax.legend()
plt.show()

rospy.spin()
