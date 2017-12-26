import rosbag
import rospy
import smach
from smach_ros import SimpleActionState
import roslib
import actionlib
from actionlib_msgs.msg import GoalStatus
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

rospy.init_node("my_bag_recorder")

file_name = 'test'
#bagRecord = MyBagRecorder(file_name)

#while not rospy.is_shutdown():
#    rospy.spin()

 #smach.StateMachine.add('SEND_GOAL',
#                           SimpleActionState('action_server_namespace',
#                                             MoveBaseSafeAction),
#                           transitions={'succeeded':'APPROACH_PLUG'})

sm = smach.StateMachine(['succeeded','aborted','preempted','END_SM'])
sm.userdata.goal_location = "couch_table"
with sm:
    def goal_cb(userdata, goal):
        print "Sending to " , userdata.goal_location
        goal = MoveBaseSafeGoal()
        goal.arm_safe_position = 'folded'
        goal.source_location = 'START'
        goal.destination_location = userdata.goal_location
        return goal

    def result_cb(userdata, status, result):
        print type(status)
        print result
        if status == GoalStatus.SUCCEEDED or status == GoalStatus.PREEMPTED:
            return 'succeeded'

    smach.StateMachine.add('TRIGGER_MOVE',
                      SimpleActionState('move_base_safe_server',
                                        MoveBaseSafeAction,
                                        goal_cb=goal_cb,
                                        result_cb=result_cb,
                                        server_wait_timeout=rospy.Duration(200.0),
                                        exec_timeout = rospy.Duration(150.0),
                                        input_keys=['goal_location']),
                      transitions={'succeeded':'END_SM', 'aborted':'END_SM'},
                      remapping={'gripper_input':'userdata_input'})

#bagRecord.close()
outcome = sm.execute()
