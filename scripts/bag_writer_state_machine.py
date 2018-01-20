import rosbag
import rospy
import smach
from smach_ros import SimpleActionState, IntrospectionServer
import roslib
import actionlib
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import AccelStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, LaserScan
import random
import sys
from mdr_move_base_safe.msg import MoveBaseSafeAction, MoveBaseSafeGoal

class Setup(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['SETUP_DONE', 'FINISH_REQUEST'],
                             input_keys=['counter_in', 'restart_requested'],
                             output_keys=['counter_out', 'restart_requested_out'])

    def execute(self, userdata):
        rospy.loginfo('Executing SETUP')
        rospy.sleep(0.5)
        if userdata.counter_in < 50:
            userdata.counter_out = userdata.counter_in +1
            userdata.restart_requested_out = True
            return 'SETUP_DONE'
        else:
            userdata.restart_requested_out = None
            return 'FINISH_REQUEST'

class MyBagRecorder(smach.State):
    def __init__(self):
        self.busy = False
        self.is_finished = False
        self.is_bag_started = False
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
        self.startBag()
        smach.State.__init__(self,
                             outcomes=['RECORD_STARTED','END_RECORD'],
                             input_keys=['counter_in', 'shared_string', 'restart_requested'],
                             output_keys=['counter_out', 'restart_requested_out'])
        rospy.loginfo("Initializing")

    def startBag(self):
        self.bag = rosbag.Bag('starter.bag', 'w')
        self.is_finished = False
        self.is_bag_started= True
        rospy.sleep(5)

    def execute(self, userdata):

        #while not self.is_finished:
        #    pass

        #self.is_finished = False

        if userdata.restart_requested:
            print ("NEW FILE")
            rospy.sleep(0.5)
            self.close()
            self.is_finished = True # Filter error of cb when file is already closed
            self.bag = rosbag.Bag(userdata.shared_string + str(userdata.counter_in) +'.bag', 'w')
            #userdata.counter_out = userdata.counter_in + 1
            self.is_bag_started = True
            rospy.sleep(0.5)
            self.is_finished = False # End Filter
            userdata.restart_requested_out = False
            return "RECORD_STARTED"

        if userdata.restart_requested is None:
            print ("EXITING")
            self.is_finished = True
            self.close()
            return "END_RECORD"

    def writeToBag(self,topic, msgs):
        while (self.busy):
            pass
        self.busy = True
        if not self.is_finished and self.is_bag_started:
            self.bag.write(topic, msgs)
        self.busy = False

    def mainCB(self,msg, topic_name):
        self.writeToBag(topic_name, msg)

    def close(self):
        rospy.loginfo("Closing Bag File")
        self.is_bag_started = False
        self.bag.close()
        self.is_finished = True

rospy.init_node("my_bag_recorder")

#file_name = 'test'
#bagRecord = MyBagRecorder(file_name)

#while not rospy.is_shutdown():
#    rospy.spin()

 #smach.StateMachine.add('SEND_GOAL',
#                           SimpleActionState('action_server_namespace',
#                                             MoveBaseSafeAction),
#                           transitions={'succeeded':'APPROACH_PLUG'})

sm = smach.StateMachine(['succeeded','aborted','preempted','END_SM'])
sm.userdata.goal_location = list()
sm.userdata.goal_location.append("couch_table")
sm.userdata.goal_location.append("kitchen")
#sm.userdata.goal_location.append("dinner_table")
sm.userdata.goal_location.append("narrow_passge")
sm.userdata.goal_location.append("small_dining_room")
#sm.userdata.goal_location.append("lamp")
#sm.userdata.goal_location.append("exit")
sm.userdata.goal_location.append("exit")
sm.userdata.last_location = "START"
sm.userdata.sm_counter = 31
sm.userdata.bag_family = "cob3-attempt-2001-"#"testing_bag"
sm.userdata.restart_requested = True

with sm:
    def goal_cb(userdata, goal):

        while True:
            current_goal = random.choice(userdata.goal_location)
            if not current_goal == userdata.last_location:
                break

        print "Sending to " , current_goal
        goal = MoveBaseSafeGoal()
        goal.arm_safe_position = 'folded'
        goal.source_location = 'START'
        goal.destination_location = current_goal
        userdata.last_location = current_goal
        return goal

    def result_cb(userdata, status, result):
        print type(status)
        print result
        if status == GoalStatus.SUCCEEDED or status == GoalStatus.PREEMPTED:
            return 'succeeded'

    smach.StateMachine.add('SETUP', Setup(),
                       transitions={'SETUP_DONE':'CONFIG_WRITER', 'FINISH_REQUEST': 'CONFIG_WRITER'},
                       remapping={'counter_in':'sm_counter',
                                  'counter_out':'sm_counter',
                                  'restart_requested_out':'restart_requested'})

    smach.StateMachine.add('CONFIG_WRITER', MyBagRecorder(),
                   transitions={'RECORD_STARTED':'TRIGGER_MOVE', 'END_RECORD': 'END_SM'},
                   remapping={'counter_in':'sm_counter',
                              'counter_out':'sm_counter',
                              'shared_string':'bag_family',
                              'restart_requested_out':'restart_requested'})

    smach.StateMachine.add('TRIGGER_MOVE',
                      SimpleActionState('move_base_safe_server',
                                        MoveBaseSafeAction,
                                        goal_cb=goal_cb,
                                        result_cb=result_cb,
                                        server_wait_timeout=rospy.Duration(200.0),
                                        exec_timeout = rospy.Duration(150.0),
                                        input_keys=['goal_location', 'last_location'],
                                        output_keys=['last_location']),
                      transitions={'succeeded':'SETUP', 'aborted':'SETUP'})

#bagRecord.close()
#Instrospection
sis = IntrospectionServer('bag_writer', sm, '/SM_ROOT')
sis.start()
# Execute the state machine
outcome = sm.execute()
# Wait for ctrl-c to stop the application
sis.stop()
