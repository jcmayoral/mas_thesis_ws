import rospy
from std_msgs.msg import Header, Float64


def subCB(msg):
    global collision
    collision = 0.1

def publishLabel():
    global collision
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
        if collision == 0.1:
            counter = counter + 1
        if counter is 2:
            collision = 0
            counter = 0
        rate.sleep()

if __name__ == '__main__':
    publishLabel()
