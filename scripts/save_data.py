import rosbag
import rospy
from geometry_msgs.msg import AccelStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, LaserScan


class BagRecorder():
    def __init__(self):
        self.bag = rosbag.Bag('test.bag', 'w')
        rospy.Subscriber("/accel", AccelStamped, self.mainCB, "/accel")
        rospy.Subscriber("/cmd_vel", Twist, self.mainCB, "/cmd_vel")
        rospy.Subscriber("/odom", Odometry, self.mainCB, "/odom")
        rospy.Subscriber("/scan_front", LaserScan, self.mainCB, "/scan_front") 
        rospy.Subscriber("/scan_rear", LaserScan, self.mainCB, "/scan_rear") 
        rospy.Subscriber("/arm_cam3d/rgb/image_raw", Image, self.mainCB, "/arm_cam3d/rgb/image_raw")
        rospy.loginfo("Initializing")

    def writeToBag(self,topic, msgs):
        self.bag.write(topic, msgs)
        
    def mainCB(self,msg, topic_name):
        self.writeToBag(topic_name, msg)    
        
    def close(self):
        rospy.loginfo("Closing Bag File")
        self.bag.close()
    
rospy.init_node("my_bag_recorder")
bagRecord = BagRecorder()

while not rospy.is_shutdown():
    rospy.spin()

bagRecord.close()
