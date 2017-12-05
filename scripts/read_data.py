import rosbag
from geometry_msgs.msg import AccelStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, LaserScan
import rospy
import sys

rospy.init_node("my_bag_reader")
file_name = '/home/jose/ROS/thesis_ws/my_ws/src/mas_thesis_ws/scripts/test.bag'

if len(sys.argv) > 1:
  file_name=str(sys.argv[1])

mytypes = [AccelStamped, Twist, Odometry, Odometry, LaserScan, LaserScan, LaserScan, Image, Image]

mytopics = ["/accel", "/cmd_vel", "/odom", "/base/odometry_controller/odom",
            "/scan_front", "/scan_rear", "/scan_unified",
            "/arm_cam3d/rgb/image_raw","/cam3d/rgb/image_raw"]

myPublishers = list()

for topic_name, msg_type in zip(mytopics,mytypes):
    publisher = rospy.Publisher(topic_name, msg_type)
    myPublishers.append([publisher,topic_name])

print myPublishers
bag = rosbag.Bag(file_name)

r = rospy.Rate(50)

for topic, msg, t in bag.read_messages(topics=mytopics):
    for p, topic_name in myPublishers:
        if topic_name == topic:
            #print "printing on ", topic_name
            p.publish(msg)
            r.sleep()

bag.close()
