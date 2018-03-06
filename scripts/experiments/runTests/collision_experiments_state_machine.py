import rosbag
import rospy
import smach
from smach_ros import SimpleActionState, IntrospectionServer
import roslib
import actionlib
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import AccelStamped, Twist, PoseStamped
from nav_msgs.msg import Odometry
from audio_common_msgs.msg import AudioData
from std_msgs.msg import Header
from sensor_msgs.msg import Image, LaserScan, Imu
import random
import sys
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from mdr_move_base_safe.msg import MoveBaseSafeAction, MoveBaseSafeGoal
import tf

class Setup(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['SETUP_DONE', 'FINISH_REQUEST'],
                             input_keys=['counter_in', 'restart_requested'],
                             output_keys=['counter_out', 'restart_requested_out', 'finish_requested_out'])

    def execute(self, userdata):
        rospy.loginfo('Executing SETUP')
        rospy.sleep(0.5)
        if userdata.counter_in < 5: # TODO Automatic File Indexing
            userdata.counter_out = userdata.counter_in +1
            userdata.restart_requested_out = True
            return 'SETUP_DONE'
        else:
            userdata.finish_requested_out = True
            return 'FINISH_REQUEST'

class MyBagRecorder(smach.State):
    def __init__(self):
        self.busy = False
        self.is_finished = False
        self.is_bag_started = False
        #TODO  yaml file to load topics
        rospy.Subscriber("/accel", AccelStamped, self.mainCB, "/accel", queue_size=300)
        rospy.Subscriber("/imu", Imu, self.mainCB, "/imu", queue_size=300)
        rospy.Subscriber("/imu/data", Imu, self.mainCB, "/imu/data", queue_size=300)
        rospy.Subscriber("/collision_label", Header, self.mainCB, "/collision_label", queue_size=300)
        rospy.Subscriber("/audio", AudioData, self.mainCB, "/audio", queue_size=300)
        rospy.Subscriber("/cmd_vel", Twist, self.mainCB, "/cmd_vel", queue_size=300)
        rospy.Subscriber("/base/twist_mux/command_navigation", Twist, self.mainCB, "/base/twist_mux/command_navigation", queue_size=300)
        rospy.Subscriber("/odom", Odometry, self.mainCB, "/odom", queue_size=300)
        rospy.Subscriber("/base/odometry_controller/odometry", Odometry, self.mainCB, "/base/odometry_controller/odometry", queue_size=300)
        rospy.Subscriber("/scan_front", LaserScan, self.mainCB, "/scan_front", queue_size=300)
        rospy.Subscriber("/scan_rear", LaserScan, self.mainCB, "/scan_rear", queue_size=300)
        rospy.Subscriber("/scan_unified", LaserScan, self.mainCB, "/scan_unified", queue_size=300)
        rospy.Subscriber("/arm_cam3d/rgb/image_raw", Image, self.mainCB, "/arm_cam3d/rgb/image_raw", queue_size=300)
        rospy.Subscriber("/cam3d/rgb/image_raw", Image, self.mainCB, "/cam3d/rgb/image_raw", queue_size=300)
        self.startBag()
        smach.State.__init__(self,
                             outcomes=['RECORD_STARTED', 'RECORD_STOPPED', 'END_RECORD'],
                             input_keys=['counter_in', 'shared_string', 'restart_requested', 'stop_requested', 'finish_requested'],
                             output_keys=['counter_out', 'restart_requested_out', 'stop_requested_out', 'finish_requested_out'])
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

        if userdata.stop_requested:
            print ("Stoping  Bag Recorder")
            rospy.sleep(0.5)
            self.close()
            self.is_finished = True # Filter error of cb when file is already closed
            userdata.stop_requested_out = False
            return "RECORD_STOPPED"


        if userdata.restart_requested:
            print ("Restarting Bag Recorder")
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

        if userdata.finish_requested:
            print ("Finishing Bag Recorder")
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

rospy.init_node("my_collision_bag_recorder")

sm = smach.StateMachine(['succeeded','aborted','preempted','END_SM'])
sm.userdata.sm_counter = 0
sm.userdata.bag_family = "collision_bags_bags"
sm.userdata.restart_requested = True # This flag restart the cycle
sm.userdata.stop_requested = False # This flag stops the recorder
sm.userdata.finish_requested = False # This flag finishes sm

with sm:
    def collision_goal_cb(userdata, goal):

        tf_listener = tf.TransformListener()
        pose = PoseStamped()
        pose.header.frame_id = '/base_link'
        pose.pose.position.x = 2.0 # Default 2.0
        pose.pose.orientation.w = 1

        goal = MoveBaseGoal()
        goal.target_pose = pose
        return goal

    def goal_cb(userdata, goal):

        goal = MoveBaseSafeGoal()
        goal.arm_safe_position = 'folded'
        goal.source_location = 'START'
        goal.destination_location = 'start_collision_position'# DEFAULT'start_collision_position'
        return goal

    def result_cb(userdata, status, result):
        print result
        if status == GoalStatus.ABORTED or status == GoalStatus.PREEMPTED:
            return 'aborted'

        if status == GoalStatus.SUCCEEDED:
            return 'succeeded'


    def collision_result_cb(userdata, status, result):
        print type(status)
        print result
        #if status == GoalStatus.SUCCEEDED or status == GoalStatus.ABORTED:
        print ("Waiting 3 seconds")
        rospy.sleep(3)
        userdata.stop_requested_out = True
        return 'succeeded'



    #GO TO START
    smach.StateMachine.add('GO_START',
                      SimpleActionState('move_base_safe_server',
                                        MoveBaseSafeAction,
                                        goal_cb=goal_cb,
                                        result_cb=result_cb,
                                        server_wait_timeout=rospy.Duration(200.0),
                                        exec_timeout = rospy.Duration(150.0),
                                        input_keys=['sm_counter']),
                      transitions={'succeeded':'SETUP', 'aborted':'GO_START'})

    smach.StateMachine.add('SETUP', Setup(),
                       transitions={'SETUP_DONE':'CONFIG_WRITER', 'FINISH_REQUEST': 'CONFIG_WRITER'},
                       remapping={'counter_in':'sm_counter',
                                  'counter_out':'sm_counter',
                                  'restart_requested_out':'restart_requested',
                                  'finish_requested_out':'finish_requested'})

    smach.StateMachine.add('CONFIG_WRITER', MyBagRecorder(),
                   transitions={'RECORD_STARTED':'TRIGGER_MOVE', 'RECORD_STOPPED': 'GO_START', 'END_RECORD': 'END_SM'},
                   remapping={'counter_in':'sm_counter',
                              'counter_out':'sm_counter',
                              'shared_string':'bag_family',
                              'restart_requested_out':'restart_requested',
                              'stop_requested_out':'stop_requested'})

    smach.StateMachine.add('TRIGGER_MOVE',
                      SimpleActionState('move_base',
                                        MoveBaseAction,
                                        goal_cb=collision_goal_cb,
                                        result_cb=collision_result_cb,
                                        server_wait_timeout=rospy.Duration(30.0),
                                        output_keys=['stop_requested_out'],
                                        exec_timeout = rospy.Duration(30.0)),
                      transitions={'succeeded':'CONFIG_WRITER', 'aborted':'CONFIG_WRITER'},
                      remapping={'stop_requested_out':'stop_requested'})

#bagRecord.close()
#Instrospection
sis = IntrospectionServer('bag_writer', sm, '/SM_ROOT')
sis.start()
# Execute the state machine
outcome = sm.execute()
# Wait for ctrl-c to stop the application
sis.stop()
