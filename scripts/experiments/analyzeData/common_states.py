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

# define state ReadBag
class MyBagReader(smach.State):
    def __init__(self,  limit=float("inf"), max_bag_file = 100):
        mytypes = [AccelStamped, Twist, Odometry, Odometry, LaserScan, LaserScan, LaserScan, Image,
                   Image, Odometry, Header, Imu,AudioData]
        #self.path = '/home/jose/ROS/thesis_ws/my_ws/rosbag_test/cob3/static_runs_2911/static_runs/' #TODO
        #self.path = '/home/jose/ROS/thesis_ws/my_ws/rosbag_test/cob3/cob3-test-2301/'
        self.max_bag_file = max_bag_file
        self.mytopics = ["/accel", "/cmd_vel", "/odom", "/base/odometry_controller/odom",
            "/scan_front", "/scan_rear", "/scan_unified",
            "/arm_cam3d/rgb/image_raw","/cam3d/rgb/image_raw",
            "/base/odometry_controller/odometry", "/collision_label",
            "/imu/data", "/audio"]

        self.myPublishers = list()
        self.limit = limit
        self.finish_pub = rospy.Publisher("finish_reading", String, queue_size=1)
        rospy.sleep(2)

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
                self.bag = rosbag.Bag(file_name)
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


        try:
            start_time = self.bag.get_start_time()
            end_time = self.bag.get_end_time()
            duration_time = end_time - start_time
            r = rospy.Rate(300) # Default 100
            for topic, msg, t in self.bag.read_messages(topics=self.mytopics):
                if t.to_sec() - start_time > self.limit:
                    ROS_INFO("Quitting")
                    break
                print ("ROSBag  Running ", t.to_sec() - start_time, " of " , duration_time, end="\r")
                for p, topic_name in self.myPublishers:
                    if topic_name == topic:
                        #print "printing on ", topic_name
                        p.publish(msg)
                        r.sleep()
                        break
            print ("\n")
            self.bag.close()
        except:
            rospy.logwarn("ROSBAG FILE NOT FOUND")
            self.bag.close()

        if userdata.foo_counter_in < max_bag_file:  #n number of bag files // TODO default 35
            print("restarting")
            userdata.foo_counter_out = userdata.foo_counter_in + 1
            print("ending")
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
        rospy.sleep(0.2)

    def execute(self, userdata):
        rospy.loginfo('Executing state RESTART READER')
        #rospy.loginfo('Counter = %f'%userdata.bar_counter_in)
        monitor_reset_pub = rospy.Publisher('/sm_reset', Empty, queue_size=1)
        while monitor_reset_pub.get_num_connections() < 1:
            pass
        #print (monitor_reset_pub.get_num_connections())
        monitor_reset_pub.publish(Empty())
        print ("Send EMPTY")
        rospy.sleep(0.5)
        return 'NEXT_BAG'
