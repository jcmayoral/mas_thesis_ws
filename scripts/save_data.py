import rosbag
import rospy
from geometry_msgs.msg import AccelStamped, Twist
from nav_msgs.msg import Odometry


class BagRecorder():
    def __init__(self):
        self.bag = rosbag.Bag('test.bag', 'w')
        rospy.Subscriber("accel", AccelStamped, self.accCB)
        rospy.Subscriber("cmd_vel", Twist, self.cmdCB)
        rospy.Subscriber("odom", Odometry, self.odomCB)
        rospy.loginfo("Initializing")

    def writeToBag(self,topic, msgs):
        self.bag.write(topic, msgs)
        
    def odomCB(self,msg):
        self.writeToBag("odom", msg)

    def accCB(self,msg):
        self.writeToBag("accel", msg)
    
    def cmdCB(self,msg):
        self.writeToBag("cmd_vel", msg)
    
    def close(self):
        self.bag.close()
    
rospy.init_node("my_bag_recorder")
bagRecord = BagRecorder()

while not rospy.is_shutdown():
    rospy.spin()

bagRecord.close()
