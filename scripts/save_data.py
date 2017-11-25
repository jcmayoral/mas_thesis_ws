import rosbag
import rospy
import sys
from std_msgs.msg import Int32, String

bag = rosbag.Bag('test.bag', 'w')


while not rospy.is_shutdown():
  try:
    str = String()
    str.data = 'foo'

    i = Int32()
    i.data = 42

    bag.write('chatter', str)
    bag.write('numbers', i)
  except:
    print "Unexpected error:", sys.exc_info()[0]
 
bag.close()
