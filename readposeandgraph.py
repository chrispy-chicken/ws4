import rospy
from geometry_msgs.msg import PoseStamped
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np
import threading
import time

# Global variables
x_data = []
y_data = []
reset_flag = False

# Function to handle key press events
def on_key(event):
    global reset_flag
    if event.key == 'p':
        reset_flag = True

# Function to reset data
def reset_data():
    global x_data, y_data, reset_flag
    while not rospy.is_shutdown():
        if reset_flag:
            x_data = []
            y_data = []
            reset_flag = False
        time.sleep(0.1)

# Callback function for the PoseStamped message
def pose_callback(msg):
    global x_data, y_data
    x_data.append(msg.pose.position.x)
    y_data.append(msg.pose.position.y)

# Function to update the plot
def update(frame):
    plt.clf()
    plt.plot(x_data, y_data, 'bo-')
    plt.xlim(-10, 10)  # Adjust the limits as needed
    plt.ylim(-10, 10)
    plt.xlabel('X position')
    plt.ylabel('Y position')
    plt.title('Tracked Pose')

# Main function
def main():
    global x_data, y_data

    rospy.init_node('pose_plotter', anonymous=True)
    rospy.Subscriber('/tracked_pose', PoseStamped, pose_callback)

    # Create a thread for resetting the data
    reset_thread = threading.Thread(target=reset_data)
    reset_thread.start()

    # Set up the plot
    fig = plt.figure()
    fig.canvas.mpl_connect('key_press_event', on_key)
    ani = FuncAnimation(fig, update, interval=100)

    plt.show()

    rospy.spin()
    reset_thread.join()

if __name__ == '__main__':
    main()
