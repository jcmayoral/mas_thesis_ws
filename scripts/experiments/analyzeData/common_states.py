from __future__ import print_function
import rospy
import smach
import smach_ros
import rosbag
from geometry_msgs.msg import AccelStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, LaserScan, Imu
from std_msgs.msg import Empty, String, Header
from audio_common_msgs.msg import AudioData
import subprocess

# define state ReadBag
class MyBagReader(smach.State):
    def __init__(self,  limit=float("inf"), max_bag_file = 100):
        mytypes = [AccelStamped, Twist, Odometry, Odometry, LaserScan, LaserScan, LaserScan, Image,
                   Image, Odometry, Header, Imu,AudioData, Imu]
        #self.path = '/home/jose/ROS/thesis_ws/my_ws/rosbag_test/cob3/static_runs_2911/static_runs/' #TODO
        #self.path = '/home/jose/ROS/thesis_ws/my_ws/rosbag_test/cob3/cob3-test-2301/'
        self.max_bag_file = max_bag_file
        self.mytopics = ["/accel", "/cmd_vel", "/odom", "/base/odometry_controller/odom",
            "/scan_front", "/scan_rear", "/scan_unified",
            "/arm_cam3d/rgb/image_raw","/cam3d/rgb/image_raw",
            "/base/odometry_controller/odometry", "/collision_label",
            "/imu/data", "/audio", '/imu']

        self.myPublishers = list()
        self.limit = limit
        self.finish_pub = rospy.Publisher("finish_reading", String, queue_size=1)

        for topic_name, msg_type in zip(self.mytopics,mytypes):
            publisher = rospy.Publisher(topic_name, msg_type, queue_size=1)
            self.myPublishers.append([publisher,topic_name])

        smach.State.__init__(self,
                             outcomes=['RESTART_READER','END_READER'],
                             input_keys=['foo_counter_in', 'shared_string', 'path'],
                             output_keys=['foo_counter_out'])

    def execute(self, userdata):
        rospy.loginfo('Executing state Reader')

        self.is_file_ok = False
        max_bag_file = self.max_bag_file
        while not self.is_file_ok:
            try:
                file_name = userdata.path + userdata.shared_string + str(userdata.foo_counter_in)+".bag"
                command = "rosbag play " + file_name
                self.p = subprocess.Popen(command, stdin=subprocess.PIPE, shell=True)
                #self.bag = rosbag.Bag(file_name)
                print ("file_name" , file_name )
                self.is_file_ok = True
            except:
                rospy.loginfo("Skipping File " + str(userdata.foo_counter_in))
                userdata.foo_counter_out = userdata.foo_counter_in + 1

                if userdata.foo_counter_in > max_bag_file:
                    fb = String()
                    fb.data = "END_BAG"
                    userdata.foo_counter_out = 1
                    self.finish_pub.publish(fb)
                    rospy.sleep(2)
                    return 'END_READER'
                #print("aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa")

        rospy.loginfo(self.p)
        self.p.wait()

        if userdata.foo_counter_in < max_bag_file:  #n number of bag files // TODO default 35
            userdata.foo_counter_out = userdata.foo_counter_in + 1
            fb = String()
            fb.data = "NEXT_BAG"
            self.finish_pub.publish(fb)
            rospy.sleep(2)
            return 'RESTART_READER'
        else:
            fb = String()
            fb.data = "END_BAG"
            userdata.foo_counter_out = 1
            self.finish_pub.publish(fb)
            rospy.sleep(2)
            return 'END_READER'


class RestartReader(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['NEXT_BAG'],
                             input_keys=['bar_counter_in'])
        #rospy.spin()
        self.monitor_reset_pub = rospy.Publisher('/sm_reset', Empty, queue_size=1)
        rospy.sleep(0.2)

    def execute(self, userdata):
        rospy.loginfo('Executing state RESTART READER')
        #rospy.loginfo('Counter = %f'%userdata.bar_counter_in)

        #print (monitor_reset_pub.get_num_connections())
        self.monitor_reset_pub.publish(Empty())
        print ("Send EMPTY")
        rospy.sleep(2)
        return 'NEXT_BAG'
