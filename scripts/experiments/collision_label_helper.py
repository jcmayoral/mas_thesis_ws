import rospy
from std_msgs.msg import Header, Float64
import sys

def subCB(msg):
    global collision, const_collision_value
    collision = const_collision_value

def publishLabel():
    global collision, const_collision_value
    rospy.init_node("labeler_collision", anonymous=True)
    rospy.Subscriber("/collision_label", Header, subCB)
    pub = rospy.Publisher("/label", Float64)
    msg = Float64()
    collision = 0
    counter = 0
    rate =rospy.Rate(1)

    while not rospy.is_shutdown():
        msg.data = collision
        pub.publish(msg)
        if collision == const_collision_value:
            counter = counter + 1
        if counter is 2:
            collision = 0
            counter = 0
        rate.sleep()

if __name__ == '__main__':
    global const_collision_value

    if len(sys.argv) > 1:
        const_collision_value = float(sys.argv[1])
    else:
        const_collision_value = 1.0
    publishLabel()
