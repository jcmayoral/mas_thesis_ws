import rosbag
import rospy
import smach
import smach_ros
import roslib
import actionlib
from geometry_msgs.msg import AccelStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, LaserScan
import sys
from mdr_move_base_safe.msg import MoveBaseSafeAction, MoveBaseSafeGoal


class MyBagRecorder(smach.State):
    def __init__(self,file_name):
        self.busy = False
        self.bag = rosbag.Bag(file_name +'.bag', 'w')
        self.is_finished = False
        rospy.Subscriber("/accel", AccelStamped, self.mainCB, "/accel")
        rospy.Subscriber("/cmd_vel", Twist, self.mainCB, "/cmd_vel")
        rospy.Subscriber("/base/twist_mux/command_navigation", Twist, self.mainCB, "/base/twist_mux/command_navigation")
        rospy.Subscriber("/odom", Odometry, self.mainCB, "/odom")
        rospy.Subscriber("/base/odometry_controller/odometry", Odometry, self.mainCB, "/base/odometry_controller/odometry")
        rospy.Subscriber("/scan_front", LaserScan, self.mainCB, "/scan_front")
        rospy.Subscriber("/scan_rear", LaserScan, self.mainCB, "/scan_rear")
        rospy.Subscriber("/scan_unified", LaserScan, self.mainCB, "/scan_unified")
        rospy.Subscriber("/arm_cam3d/rgb/image_raw", Image, self.mainCB, "/arm_cam3d/rgb/image_raw")
        rospy.Subscriber("/cam3d/rgb/image_raw", Image, self.mainCB, "/cam3d/rgb/image_raw")

        smach.State.__init__(self,
                             outcomes=['RESTART_RECORD','END_RECORD'],
                             input_keys=['foo_counter_in', 'shared_string'],
                             output_keys=['foo_counter_out'])
        rospy.loginfo("Initializing")

    def execute(self, userdata):

        while not self.is_finished:
            pass

        self.is_finished = False

    def writeToBag(self,topic, msgs):
        while (self.busy is True):
            pass
        self.busy = True
        self.bag.write(topic, msgs)
        self.busy = False

    def mainCB(self,msg, topic_name):
        self.writeToBag(topic_name, msg)

    def close(self):
        rospy.loginfo("Closing Bag File")
        self.bag.close()
        self.is_finished = True

class MoveClient():
    def __init__(self):
        client = actionlib.SimpleActionClient('move_base_safe_server', MoveBaseSafeAction)
        client.wait_for_server()
        smach.State.__init__(self,
                             outcomes=['GOAL_ACHIEVED','GOAL_NOT_ACHIEVED'],
                             input_keys=['goal_location'])

    def execute(self,userdata)
        goal = MoveBaseSafeGoal()
        goal.arm_safe_position = 'folded'
        try:
            goal.source_location = "START"
            goal.destination_location = userdata.goal_location
            timeout = 150.0
            rospy.loginfo('Sending action lib goal to move_base_safe_server, source : ' +
                            goal.source_location + ' , destination : ' + goal.destination_location)
            client.send_goal(goal)
            client.wait_for_result(rospy.Duration.from_sec(int(timeout)))
            print client.get_result()
            return "GOAL_ACHIEVED"
       except:
           pass
       else:
           rospy.logerr('Arguments were not received in the proper format !')
           rospy.loginfo('usage : move_base_safe SOURCE DESTINATION')


rospy.init_node("my_bag_recorder")

file_name = 'test'

if len(sys.argv) > 1:
  file_name=str(sys.argv[1])

bagRecord = MyBagRecorder(file_name)

while not rospy.is_shutdown():
    rospy.spin()

bagRecord.close()
