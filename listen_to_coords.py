import rospy
from tf2_msgs.msg import TFMessage

f_name = 'verlengsnoer.txt'

def callback(data):
    for transform in data.transforms:
        if transform.child_frame_id == "laser":  # Replace "your_frame_id" with the actual frame ID you're interested in
            
            x = transform.transform.translation.x
            y = transform.transform.translation.y
            pos = (x, y)
            # append the position to the list
            with open(f_name, 'a') as f:
                f.write(str(pos) + '\n')

def listener():
    rospy.init_node('tf_listener', anonymous=True)
    rospy.Subscriber("/tf", TFMessage, callback)
    rospy.spin()


if __name__ == '__main__':
    with open(f_name, 'w') as f:
        pass
    from time import sleep
    sleep(3)
    print("go!")
    sleep(1)
    listener()